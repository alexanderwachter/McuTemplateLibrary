/*
 * Template-based state machine on top of the mtl library.
 * SPDX-License-Identifier: Apache-2.0
 *
 * States are classes with optional members detected by requires-expressions:
 * onEntry(), onExit(), static constexpr timeout, and static constexpr
 * members watched by observers. The state set is derived from the table;
 * an initial<STATE> table role picks the initial state (default: the first
 * state of the first transition).
 *
 * Events may carry payload: a target state constructible from the
 * triggering event is emplaced with it, any other target state is
 * default-constructed. Observer hooks run after the emplace and receive
 * the machine, so e.g. a driver observer can read the delivered payload
 * through machine.getIf<NEW_STATE>().
 *
 * fsm::internal_transition<from<S>, on<E>> handles E in S without a
 * state change: no exit/entry, no observer hooks, a running timeout
 * timer is untouched. The current state instance handles the event via
 * handle(E const&), typically updating its context. A guard applies as
 * usual and internal transitions group with regular ones as
 * alternatives; from<any_state> is not supported.
 *
 * States may keep data in machine-owned context that survives
 * transitions: a state declaring a reference member named context is
 * constructed with a reference to the matching context instance -
 * (event, context) when such a constructor exists, (context) alone
 * otherwise, which every context state must provide. The machine
 * value-initializes one instance per distinct context type; states
 * naming the same type share the instance, and its data persists across
 * arbitrary transitions for the machine's lifetime. Context types must
 * be default constructible; context states need no default constructor.
 *
 * A guard gates the transition it is attached to; it provides
 * check(state, event), check(state), or check() - most specific form
 * wins. Transitions may share
 * a (state, event) pair when guards distinguish them: the alternatives
 * are tried in table order and the first whose guard passes fires; an
 * unguarded alternative is the catch-all and must be the last of its
 * group - an entry after it could never fire, so a second unguarded
 * entry for the pair (a plain duplicate included) is a static_assert.
 * When no alternative fires, process() returns false, no
 * exit/entry/hook runs, a running timeout timer keeps running, but a
 * blocked fsm::timeout transition does not re-arm the one-shot timer.
 *
 * from<any_state> matches every state, and a state may handle the same
 * event itself: its exact (state, event) group then replaces the
 * wildcard group entirely - also when every guard of the exact group
 * refuses, which yields false rather than the wildcard (so an exact
 * pair is the way to exempt a state from a wildcard). Wildcard entries
 * form alternatives among themselves like any other group.
 *
 * Timer policy contract (owned by fsm::timed<TIMER>):
 *   start(ms, fsm::timer_callback, void* context) arms a one-shot timer
 *   that invokes callback(context) once; restarting re-arms. stop()
 *   disarms and must tolerate an unarmed timer. The callback runs in the
 *   policy's execution context; process() is not re-entrant and not
 *   thread-safe - callback and process() must be serialized externally.
 *
 * Observer contract: injected by reference, must outlive the machine.
 * Optional per-edge hooks, each detected by a requires-expression, run in
 * observer parameter order (place fsm::timed before value observers):
 *   template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
 *   void onExitState(MACHINE&);  - transition fires, old state still alive
 *   void onEnterState(MACHINE&); - new state emplaced, before onEntry();
 *                                    on construction with OLD = mtl::nil_type,
 *                                    OLD = fsm::any_state on the shared
 *                                    wildcard path (see process())
 *   template<typename FROM_STATE, typename EVENT, typename TO_STATE, typename MACHINE>
 *   void onTransition(MACHINE&); - after the change completed (after
 *                                    onEntry()); TO = fsm::internal_target for
 *                                    an internal transition, FROM =
 *                                    fsm::any_state on the shared wildcard
 *                                    path, which the observer accepts by
 *                                    declaring `static constexpr bool
 *                                    source_agnostic = true` (otherwise its
 *                                    hook forces the per-source expansion)
 *   template<typename TABLE> static constexpr void validate();
 *                                  - invoked at machine instantiation: the
 *                                    place for an observer's compile-time
 *                                    checks against the transition table
 * The machine itself imposes nothing on the table beyond its shape: a state
 * feature no injected observer consumes (a timeout without fsm::timed, an
 * annotation nobody watches) is silently unobserved.
 * Value observation with change suppression: see fsm::observing below.
 */

#pragma once

#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <chrono>
#include <concepts>
#include <cstdint>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

namespace fsm {

// Matches every state in from<>; a state's own (state, event) group
// replaces the wildcard group, even when all its guards refuse
struct any_state {};

// Event injected by the timer policy when a state's timeout expires
struct timeout {};

// Event injected by the deadline policy when a phase deadline expires
// (fsm::deadlined below); distinct from timeout so a state can react
// to both
struct deadline {};

namespace internal {

// A state opts into machine-owned context by holding a reference
// member named context, initialized by its constructors
template<typename T>
concept context_holder = requires { T::context; } && std::is_reference_v<decltype(T::context)>;

template<typename STATE>
struct has_context : std::bool_constant<context_holder<STATE>> {};

template<context_holder STATE>
struct context_of : std::type_identity<std::remove_reference_t<decltype(STATE::context)>> {};

template<context_holder STATE>
using context_of_t = typename context_of<STATE>::type;

// Whether STATE is constructed from this event (with its context when
// it has one)
template<typename STATE, typename EVENT>
consteval bool payloadConstructible()
{
    if constexpr (context_holder<STATE>) {
        return std::constructible_from<STATE, EVENT const&, context_of_t<STATE>&>;
    } else {
        return std::constructible_from<STATE, EVENT const&>;
    }
}

template<typename STATE, typename EVENT>
inline constexpr bool payload_constructible_v = payloadConstructible<STATE, EVENT>();

// Arguments constructing STATE in place inside a variant. The tuple
// round-trip through make_from_tuple is free: its prvalue is elided
// into the variant (measured GCC 15 -Os: direct stores, no tuple, no
// move)
template<typename STATE, typename CONTEXT_TUPLE>
constexpr auto initialArgs(CONTEXT_TUPLE& contexts)
{
    if constexpr (context_holder<STATE>) {
        return std::forward_as_tuple(std::in_place_type<STATE>,
                                     std::get<context_of_t<STATE>>(contexts));
    } else {
        return std::make_tuple(std::in_place_type<STATE>);
    }
}

} // namespace internal

namespace concepts {

// States are classes; on entry they are constructed from the triggering
// event if such a constructor exists, default-constructed otherwise.
// Context states are constructed with their context instead.
template<typename T>
concept state = std::is_class_v<T> &&
                (std::default_initializable<T> || internal::context_holder<T>);

} // namespace concepts

template<concepts::state STATE>
struct from {};

template<typename EVENT>
struct on {};

template<concepts::state STATE>
struct to {};

template<typename GUARD>
struct guard {};

template<concepts::state STATE>
struct initial {};

namespace internal {

template<typename T> struct is_from : std::false_type {};
template<typename S> struct is_from<fsm::from<S>> : std::true_type {};

template<typename T> struct is_on : std::false_type {};
template<typename E> struct is_on<fsm::on<E>> : std::true_type {};

template<typename T> struct is_to : std::false_type {};
template<typename S> struct is_to<fsm::to<S>> : std::true_type {};

template<typename T> struct is_guard : std::false_type {};
template<typename G> struct is_guard<fsm::guard<G>> : std::true_type {};

template<typename T> struct is_initial : std::false_type {};
template<typename S> struct is_initial<fsm::initial<S>> : std::true_type {};

template<typename T> struct unwrap;
template<typename S> struct unwrap<fsm::from<S>>    { using type = S; };
template<typename E> struct unwrap<fsm::on<E>>      { using type = E; };
template<typename S> struct unwrap<fsm::to<S>>      { using type = S; };
template<typename G> struct unwrap<fsm::guard<G>>   { using type = G; };
template<typename S> struct unwrap<fsm::initial<S>> { using type = S; };
template<>           struct unwrap<mtl::nil_type>   { using type = mtl::nil_type; };

// Payload of the first role matching PREDICATE; nil_type if there is none
template<mtl::concepts::typelist LIST, template<typename> typename PREDICATE>
using find_role_t = typename unwrap<mtl::find_if_t<LIST, PREDICATE>>::type;

// Stand-in for any event in unevaluated contexts: validates a guard's
// two-argument (state, event) form when the event type is unknown
struct any_payload {
    template<typename T>
    operator T const&() const;
};

} // namespace internal

namespace concepts {

// The guard contract is guard_for below, validated against the
// transition's from-state; there is no state-independent guard concept
// because a templated check (a guard shared by several states via
// check(auto const&)) cannot be probed by name alone

// check(from_state, event) for conditions on the event payload before
// any handler applied it, check(from_state) for conditions on state
// data, check() for state-independent ones. The event form is
// validated with a payload stand-in - the concrete event type is only
// known at the process() call
template<typename GUARD, typename STATE>
concept guard_for = requires(STATE const& state) {
    { GUARD::check(state) } -> std::convertible_to<bool>;
} || requires(STATE const& state) {
    { GUARD::check(state, internal::any_payload{}) } -> std::convertible_to<bool>;
} || requires {
    { GUARD::check() } -> std::convertible_to<bool>;
};

// Anything exposing the four role aliases works as a transition
template<typename T>
concept transition = requires {
    typename T::from;
    typename T::event;
    typename T::to;
    typename T::guard;
};

template<typename T>
concept transition_table_entry = transition<T> || internal::is_initial<T>::value;

template<typename T>
concept transition_role = internal::is_from<T>::value || internal::is_on<T>::value ||
                          internal::is_to<T>::value || internal::is_guard<T>::value;

} // namespace concepts

// The named arguments may appear in any order
template<concepts::transition_role... ROLEs>
struct transition {
private:
    using roles = mtl::typelist<ROLEs...>;
    static_assert(mtl::count_if_v<roles, internal::is_from> == 1,
                  "transition: exactly one from<STATE> required");
    static_assert(mtl::count_if_v<roles, internal::is_on> == 1,
                  "transition: exactly one on<EVENT> required");
    static_assert(mtl::count_if_v<roles, internal::is_to> == 1,
                  "transition: exactly one to<STATE> required");
    static_assert(mtl::count_if_v<roles, internal::is_guard> <= 1,
                  "transition: at most one guard<GUARD> allowed");

public:
    using from  = internal::find_role_t<roles, internal::is_from>;
    using event = internal::find_role_t<roles, internal::is_on>;
    using to    = internal::find_role_t<roles, internal::is_to>;
    using guard = internal::find_role_t<roles, internal::is_guard>; // nil_type if absent

private:
    static_assert(std::is_same_v<guard, mtl::nil_type> || concepts::guard_for<guard, from>,
                  "transition: guard must provide static bool check(FROM const&) "
                  "or static bool check()");
};

// The to-alias of internal transitions: never a state of the table
struct internal_target {};

// Handles the event inside the from-state instead of transitioning
template<concepts::transition_role... ROLEs>
struct internal_transition {
private:
    using roles = mtl::typelist<ROLEs...>;
    static_assert(mtl::count_if_v<roles, internal::is_from> == 1,
                  "internal_transition: exactly one from<STATE> required");
    static_assert(mtl::count_if_v<roles, internal::is_on> == 1,
                  "internal_transition: exactly one on<EVENT> required");
    static_assert(mtl::count_if_v<roles, internal::is_to> == 0,
                  "internal_transition: to<STATE> is not allowed");
    static_assert(mtl::count_if_v<roles, internal::is_guard> <= 1,
                  "internal_transition: at most one guard<GUARD> allowed");

public:
    using from  = internal::find_role_t<roles, internal::is_from>;
    using event = internal::find_role_t<roles, internal::is_on>;
    using to    = internal_target;
    using guard = internal::find_role_t<roles, internal::is_guard>; // nil_type if absent

private:
    static_assert(!std::is_same_v<from, any_state>,
                  "internal_transition: from<any_state> is not supported");
    static_assert(std::is_same_v<guard, mtl::nil_type> || concepts::guard_for<guard, from>,
                  "internal_transition: guard must provide static bool check(FROM const&) "
                  "or static bool check()");
};

namespace internal {

template<typename TRANSITION>
inline constexpr bool is_internal_v = std::is_same_v<typename TRANSITION::to, internal_target>;

template<typename T>
struct is_internal_target : std::is_same<T, internal_target> {};

} // namespace internal

using timer_callback = void (*)(void*);

namespace concepts {

template<typename T>
concept timer = requires(T t, std::chrono::milliseconds duration,
                         timer_callback callback, void* context) {
    t.start(duration, callback, context);
    t.stop();
};

template<typename T>
concept transition_table = mtl::concepts::typelist<typename T::transitions> &&
                           mtl::concepts::typelist<typename T::states>;

} // namespace concepts

namespace internal {

// Fold-based alternative to std::visit for process(): every visitor
// instantiation is inlinable and no function-pointer table or
// bad_variant_access path can be emitted. A valueless variant matches no
// alternative and yields false (unreachable in process(): the states can
// never make the variant valueless).
// Measured GCC 15.2 x86-64 -Os (traffic_light.cpp): 32 bytes .text
// larger than std::visit. Measured arm-zephyr-eabi GCC 14.3 -Os
// (Cortex-M0+, 14-state/20-event machine): 3.7 kB smaller - std::visit
// emits per-(event, state) invoke thunks and tables that dominate at
// scale.
template<typename VISITOR, typename... ALTERNATIVEs>
    requires (std::predicate<VISITOR, ALTERNATIVEs&> && ...)
constexpr bool visit(VISITOR&& visitor, std::variant<ALTERNATIVEs...>& variant)
{
    return [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
        bool result = false;
        ((variant.index() == INDEXs &&
          (result = visitor(*std::get_if<INDEXs>(&variant)), true)) || ...);
        return result;
    }(std::index_sequence_for<ALTERNATIVEs...>{});
}

// The fold is the default: it wins clearly on embedded targets with
// large machines (measurements above), losing only a few bytes on
// hosted libstdc++ with small ones. Define MTL_FSM_FOLD_VISIT to 0 to
// use std::visit instead.
#ifndef MTL_FSM_FOLD_VISIT
#  define MTL_FSM_FOLD_VISIT 1
#endif

template<typename VISITOR, typename... ALTERNATIVEs>
    requires (std::predicate<VISITOR, ALTERNATIVEs&> && ...)
constexpr bool dispatch(VISITOR&& visitor, std::variant<ALTERNATIVEs...>& variant)
{
#if MTL_FSM_FOLD_VISIT
    return internal::visit(std::forward<VISITOR>(visitor), variant);
#else
    return std::visit(std::forward<VISITOR>(visitor), variant);
#endif
}

template<typename FROM, typename EVENT>
struct matches {
    template<typename TRANSITION>
    struct pred : std::bool_constant<std::is_same_v<typename TRANSITION::from, FROM> &&
                                     std::is_same_v<typename TRANSITION::event, EVENT>> {};
};

// The second-stage predicate for lists already grouped by source:
// re-checking FROM there would only add instantiations
template<typename EVENT>
struct matches_event {
    template<typename TRANSITION>
    struct pred : std::is_same<typename TRANSITION::event, EVENT> {};
};

template<typename T>
struct is_any_state : std::is_same<T, any_state> {};

// All from/to states of a transition list, in order of appearance
template<mtl::concepts::typelist LIST>
struct endpoints;

template<typename... TRANSITIONs>
struct endpoints<mtl::typelist<TRANSITIONs...>> {
    using type = mtl::typelist<typename TRANSITIONs::from..., typename TRANSITIONs::to...>;
};

template<typename STATE>
inline constexpr bool has_timeout_v = requires { STATE::timeout; };

template<typename TABLE>
struct timeout_handled_in {
    template<typename STATE>
    struct pred : std::bool_constant<
        !has_timeout_v<STATE> ||
        !std::is_same_v<typename TABLE::template find_transition<STATE, timeout>,
                        mtl::nil_type>> {};
};

template<typename STATE>
inline constexpr bool has_deadline_v = requires { STATE::deadline; };

// A zero deadline is the phase-target sentinel: it stops the clock
// like an unannotated state, but says so explicitly
template<typename STATE>
constexpr bool activeDeadline()
{
    if constexpr (has_deadline_v<STATE>) {
        return STATE::deadline != decltype(STATE::deadline){};
    } else {
        return false;
    }
}

// Whether the edge continues one running phase: the state left
// carries the same nonzero deadline as the one entered
template<typename OLD_STATE, typename NEW_STATE>
constexpr bool continuesDeadline()
{
    if constexpr (has_deadline_v<OLD_STATE> && has_deadline_v<NEW_STATE>) {
        return activeDeadline<OLD_STATE>() && OLD_STATE::deadline == NEW_STATE::deadline;
    } else {
        return false;
    }
}

template<typename TABLE>
struct deadline_handled_in {
    template<typename STATE>
    struct pred : std::bool_constant<
        !activeDeadline<STATE>() ||
        !std::is_same_v<typename TABLE::template find_transition<STATE, deadline>,
                        mtl::nil_type>> {};
};

template<typename TRANSITION>
inline constexpr bool has_guard_v = !std::is_same_v<typename TRANSITION::guard, mtl::nil_type>;

// A guard provides check(state, event), check(state), or check() -
// the most specific overload wins. The event form decides on the
// payload before any handler has applied it to the state. Callability
// was validated by the transition's guard_for static_assert
template<typename GUARD, concepts::state STATE, typename EVENT>
bool checkGuard([[maybe_unused]] STATE const& state, [[maybe_unused]] EVENT const& event)
{
    if constexpr (requires { GUARD::check(state, event); }) {
        return GUARD::check(state, event);
    } else if constexpr (requires { GUARD::check(state); }) {
        return GUARD::check(state);
    } else {
        return GUARD::check();
    }
}

// True when TRANSITION may fire from the given state instance
template<typename TRANSITION, typename STATE, typename EVENT>
bool allowed(STATE const& state, EVENT const& event)
{
    if constexpr (has_guard_v<TRANSITION>) {
        return checkGuard<typename TRANSITION::guard>(state, event);
    } else {
        return true;
    }
}

// The shared wildcard path cannot name the source state, so only the
// stateless guard form applies there (shareability requires it)
template<typename TRANSITION>
bool wildcardAllowed()
{
    if constexpr (has_guard_v<TRANSITION>) {
        return TRANSITION::guard::check();
    } else {
        return true;
    }
}

// Alternatives for one (state, event) pair are tried in table order; an
// unguarded transition always fires, so anything after it is dead
template<mtl::concepts::typelist LIST>
struct no_shadowed_alternatives;

template<>
struct no_shadowed_alternatives<mtl::typelist<>> : std::true_type {};

template<typename FIRST, typename... RESTs>
struct no_shadowed_alternatives<mtl::typelist<FIRST, RESTs...>>
    : std::bool_constant<
          (has_guard_v<FIRST> ||
           mtl::count_if_v<mtl::typelist<RESTs...>,
                           matches<typename FIRST::from,
                                   typename FIRST::event>::template pred> == 0) &&
          no_shadowed_alternatives<mtl::typelist<RESTs...>>::value> {};

template<typename OBSERVER, typename STATE>
inline constexpr bool observes_v = requires { OBSERVER::template annotation<STATE>(); };

template<typename OBSERVER, typename STATE, typename OTHER>
constexpr bool annotation_changes()
{
    if constexpr (!observes_v<OBSERVER, STATE>) {
        return false;
    } else if constexpr (!observes_v<OBSERVER, OTHER>) {
        return true;
    } else if constexpr (!std::is_same_v<decltype(OBSERVER::template annotation<STATE>()),
                                         decltype(OBSERVER::template annotation<OTHER>())>) {
        return true; // different annotation types always differ
    } else {
        return OBSERVER::template annotation<STATE>() != OBSERVER::template annotation<OTHER>();
    }
}

// An annotation type declaring `static constexpr bool idempotent =
// true` promises that notifying its value again with no change in
// between is harmless - the shared wildcard path may then re-notify
// where the per-edge path would have change-suppressed
template<typename OBSERVER, typename STATE>
inline constexpr bool idempotent_annotation_v = requires {
    requires std::remove_cvref_t<decltype(OBSERVER::template annotation<STATE>())>::idempotent;
};

} // namespace internal

// Value observer base: the derived class names the watched member once and
// provides notifyEntry(value) (new state's value) and/or notifyExit(value)
// (old state's value, old state still alive), each optional:
//
//   struct lamp_driver : fsm::observing<lamp_driver> {
//       template<typename STATE>
//       static constexpr auto observe_static() -> decltype(STATE::lamps)
//       {
//           return STATE::lamps;
//       }
//       void notifyEntry(lamps_t const& lamps);
//   };
//
// The trailing return type makes states without the member drop out via
// SFINAE. The change check runs at compile time: edges between equal
// values emit no code.
//
// observe_static() is for static constexpr members: the value is read at
// type level - states need not be constructible, and naming a non-static
// member is a compile error rather than a silently wrong probe value.
// Equal-value edges are elided at compile time. For a non-static member
// (e.g. an event payload delivered into the state) provide
// observe_nonstatic() instead: it reads the current state instance, and
// the notify hooks run on every edge into/out of an observing state -
// per-instance values cannot be change-suppressed:
//
//   struct phy_driver : fsm::observing<phy_driver> {
//       static constexpr auto observe_nonstatic(auto const& state)
//           -> decltype((state.tx_message))
//       {
//           return state.tx_message;
//       }
//       void notifyEntry(message_t const& message); // hand to hardware
//   };
//
// An observer may declare both; on the same edge the static hook is
// guaranteed to run before the nonstatic one. This ordering is part of
// the contract: a static annotation can prepare (e.g. reset) what the
// nonstatic observation then consumes.
template<typename DERIVED>
struct observing {
    template<typename STATE>
    static constexpr auto annotation()
        requires requires { DERIVED::template observe_static<STATE>(); }
    {
        return DERIVED::template observe_static<STATE>();
    }

    // The static path stays per edge (compile-time change suppression
    // needs both states); the nonstatic path delegates to one body per
    // observed state
    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onExitState(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (internal::annotation_changes<DERIVED, OLD_STATE, NEW_STATE>()) {
            if constexpr (requires { self.notifyExit(DERIVED::template annotation<OLD_STATE>()); }) {
                self.notifyExit(DERIVED::template annotation<OLD_STATE>());
            }
        }
        this->template nonstaticExit<OLD_STATE>(machine);
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterState(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (internal::annotation_changes<DERIVED, NEW_STATE, OLD_STATE>()) {
            if constexpr (requires { self.notifyEntry(DERIVED::template annotation<NEW_STATE>()); }) {
                self.notifyEntry(DERIVED::template annotation<NEW_STATE>());
            }
        }
        this->template nonstaticEnter<NEW_STATE>(machine);
    }

private:
    template<typename STATE, typename MACHINE>
    void nonstaticExit(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (requires {
                          self.notifyExit(
                              DERIVED::observe_nonstatic(*machine.template getIf<STATE>()));
                      }) {
            self.notifyExit(DERIVED::observe_nonstatic(*machine.template getIf<STATE>()));
        }
    }

    template<typename STATE, typename MACHINE>
    void nonstaticEnter(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (requires {
                          self.notifyEntry(
                              DERIVED::observe_nonstatic(*machine.template getIf<STATE>()));
                      }) {
            self.notifyEntry(DERIVED::observe_nonstatic(*machine.template getIf<STATE>()));
        }
    }

public:
    // Compile-time facts for the machine's shared wildcard path (one
    // transition body per (event, target) instead of one per source
    // state). exit_silent: leaving OLD_STATE notifies nothing at all.
    // entry_shared_from: entering NEW_STATE without knowing the source
    // behaves exactly like entering it from OLD_STATE - the annotation
    // fires either way, there is nothing to notify, or the annotation
    // type declares re-notification idempotent
    template<typename OLD_STATE, typename MACHINE>
    static constexpr bool exit_silent =
        !requires(DERIVED self) {
            self.notifyExit(DERIVED::template annotation<OLD_STATE>());
        } &&
        !requires(DERIVED self, MACHINE machine) {
            self.notifyExit(DERIVED::observe_nonstatic(*machine.template getIf<OLD_STATE>()));
        };

    // An observer declaring `static constexpr bool renotify_safe =
    // true` promises its entry hooks tolerate re-notification with an
    // unchanged value (e.g. it suppresses at runtime itself) - the
    // per-annotation `idempotent` declaration, generalized
    template<typename NEW_STATE, typename OLD_STATE>
    static constexpr bool entry_shared_from =
        internal::annotation_changes<DERIVED, NEW_STATE, OLD_STATE>() ||
        !requires(DERIVED self) {
            self.notifyEntry(DERIVED::template annotation<NEW_STATE>());
        } ||
        internal::idempotent_annotation_v<DERIVED, NEW_STATE> ||
        requires { requires DERIVED::renotify_safe; };
};

// Observer implementing the state-timeout semantics on top of a TIMER
// policy. timed<POLICY> owns a default-constructed policy instance;
// timed<POLICY&> holds a caller-owned one, for policies that need
// configuration (constructor arguments) or are not default-constructible
template<concepts::timer TIMER>
struct timed {
    timed()
        requires(!std::is_reference_v<TIMER>)
    = default;

    explicit timed(TIMER timer_ref)
        requires std::is_reference_v<TIMER>
        : timer(timer_ref)
    {
    }
    // A timed state whose fsm::timeout the table ignores is a bug: the
    // timer would fire into nothing
    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(mtl::all_of_v<typename TABLE::states,
                          internal::timeout_handled_in<TABLE>::template pred>,
                      "fsm::timed: state has a timeout but no transition for fsm::timeout");
    }

    // The edge hooks delegate to per-state bodies: one instantiation
    // per timed state instead of one per edge
    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onExitState(MACHINE&)
    {
        this->template stopFor<OLD_STATE>();
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterState(MACHINE& machine)
    {
        this->template startFor<NEW_STATE>(machine);
    }

private:
    template<typename STATE>
    void stopFor()
    {
        if constexpr (internal::has_timeout_v<STATE>) {
            timer.stop(); // no timer may fire mid-transition
        }
    }

    template<typename STATE, typename MACHINE>
    void startFor(MACHINE& machine)
    {
        if constexpr (internal::has_timeout_v<STATE>) {
            constexpr auto duration =
                std::chrono::ceil<std::chrono::milliseconds>(STATE::timeout);
            static_assert(duration.count() >= 0 &&
                              duration.count() <= std::numeric_limits<std::uint32_t>::max(),
                          "fsm::timed: timeout out of the 32-bit millisecond range");
            this->startTimer(static_cast<std::uint32_t>(duration.count()), machine);
        }
    }

    // One body per machine, the duration passed as a 32-bit value:
    // materializing the 64-bit chrono constant in every per-state
    // start measured ~40 bytes each on Thumb-1 (-Os, GCC 14)
    template<typename MACHINE>
    void startTimer(std::uint32_t duration_ms, MACHINE& machine)
    {
        timer.start(
            std::chrono::milliseconds{duration_ms},
            [](void* context) {
                static_cast<MACHINE*>(context)->process(timeout{});
            },
            &machine);
    }

public:

    TIMER timer;
};

namespace internal {

// The machine's shared wildcard path special-cases the timed
// observer: it cannot name the state being left, so it stops the
// timer unconditionally - the contract tolerates stopping an unarmed
// timer, and a running one always belongs to the state being left
template<typename OBSERVER>
struct is_timed : std::false_type {};

template<typename TIMER>
struct is_timed<timed<TIMER>> : std::true_type {};

template<typename OBSERVER>
inline constexpr bool is_timed_v = is_timed<OBSERVER>::value;

template<typename OBSERVER>
void stopIfTimed(OBSERVER& observer)
{
    if constexpr (is_timed_v<OBSERVER>) {
        observer.timer.stop();
    }
}

} // namespace internal

// Observer implementing phase deadlines on top of a TIMER policy: a
// hard time budget spanning several states. A state annotates
//
//   static constexpr auto deadline = <duration>;
//
// and entering it arms the timer - unless the state left carries the
// SAME nonzero value, which continues the running phase without
// re-arming, so bouncing between the phase's states cannot extend the
// budget. Entering a state without the annotation stops the clock; so
// does the zero-duration sentinel, which marks the phase target
// explicitly. Expiry injects fsm::deadline - distinct from
// fsm::timeout, and driven by its own TIMER instance, so a state may
// carry both a per-state timeout and a phase deadline.
// timed<POLICY>/timed<POLICY&> ownership semantics apply
template<concepts::timer TIMER>
struct deadlined {
    deadlined()
        requires(!std::is_reference_v<TIMER>)
    = default;

    explicit deadlined(TIMER timer_ref)
        requires std::is_reference_v<TIMER>
        : timer(timer_ref)
    {
    }

    // A deadline the table ignores is a bug: the timer would fire into
    // nothing (the zero sentinel is exempt - it never arms)
    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(mtl::all_of_v<typename TABLE::states,
                          internal::deadline_handled_in<TABLE>::template pred>,
                      "fsm::deadlined: state has a deadline but no transition for "
                      "fsm::deadline");
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterState(MACHINE& machine)
    {
        if constexpr (internal::continuesDeadline<OLD_STATE, NEW_STATE>()) {
            // the phase's clock keeps running
        } else if constexpr (internal::activeDeadline<NEW_STATE>()) {
            constexpr auto duration =
                std::chrono::ceil<std::chrono::milliseconds>(NEW_STATE::deadline);
            static_assert(duration.count() >= 0 &&
                              duration.count() <= std::numeric_limits<std::uint32_t>::max(),
                          "fsm::deadlined: deadline out of the 32-bit millisecond range");
            this->startTimer(static_cast<std::uint32_t>(duration.count()), machine);
        } else if constexpr (internal::activeDeadline<OLD_STATE>()) {
            timer.stop(); // left the phase: unannotated or the target
        }
    }

private:
    // One body per machine, the duration as a 32-bit value - same
    // measured rationale as fsm::timed::startTimer
    template<typename MACHINE>
    void startTimer(std::uint32_t duration_ms, MACHINE& machine)
    {
        timer.start(
            std::chrono::milliseconds{duration_ms},
            [](void* context) { static_cast<MACHINE*>(context)->process(deadline{}); },
            &machine);
    }

public:
    TIMER timer;
};

// Transitions plus an optional initial<STATE> role; without it the first
// state of the first transition is the initial state
template<concepts::transition_table_entry... ENTRYs>
struct transition_table {
private:
    using entries = mtl::typelist<ENTRYs...>;
    static_assert(mtl::count_if_v<entries, internal::is_initial> <= 1,
                  "transition_table: at most one initial<STATE> allowed");

    using explicit_initial = internal::find_role_t<entries, internal::is_initial>;

public:
    using transitions = mtl::remove_if_t<entries, internal::is_initial>;

private:
    static_assert(internal::no_shadowed_alternatives<transitions>::value,
                  "transition_table: an unguarded (state, event) transition must be "
                  "the last of its alternatives");

    using endpoints =
        mtl::remove_if_t<mtl::remove_if_t<typename internal::endpoints<transitions>::type,
                                          internal::is_any_state>,
                         internal::is_internal_target>;
    static_assert(std::is_same_v<explicit_initial, mtl::nil_type> ||
                      mtl::has_a_v<endpoints, explicit_initial>,
                  "transition_table: initial<STATE> is not a state of the table");

    // Transitions grouped by their exact source, computed once per
    // FROM: the per-(FROM, EVENT) lookups filter the small group
    // instead of the whole table - a large table is otherwise
    // re-walked for every (state, event) pair the machine dispatches,
    // which dominates compile time (measured)
    template<typename FROM>
    struct from_group {
        template<typename TRANSITION>
        struct pred : std::is_same<typename TRANSITION::from, FROM> {};

        using type = mtl::filter_t<transitions, pred>;
    };

    template<typename FROM, typename EVENT>
    using find_exact = mtl::find_if_t<typename from_group<FROM>::type,
                                      internal::matches_event<EVENT>::template pred>;

    template<typename FROM, typename EVENT>
    using find_all_exact = mtl::filter_t<typename from_group<FROM>::type,
                                         internal::matches_event<EVENT>::template pred>;

public:
    // Deduplicated in order of first appearance: front is the initial state
    using states = mtl::unique_t<std::conditional_t<
        std::is_same_v<explicit_initial, mtl::nil_type>,
        endpoints,
        mtl::prepend_t<explicit_initial, endpoints>>>;

    // Exact (FROM, EVENT) first, then the (any_state, EVENT) wildcard;
    // mtl::nil_type if neither is in the table
    template<typename FROM, typename EVENT>
    using find_transition = std::conditional_t<
        std::is_same_v<find_exact<FROM, EVENT>, mtl::nil_type>,
        find_exact<any_state, EVENT>,
        find_exact<FROM, EVENT>>;

    // All alternatives for (FROM, EVENT) in table order; the wildcard
    // group applies only when no exact pair exists
    template<typename FROM, typename EVENT>
    using find_transitions = std::conditional_t<
        std::is_same_v<find_all_exact<FROM, EVENT>, mtl::typelist<>>,
        find_all_exact<any_state, EVENT>,
        find_all_exact<FROM, EVENT>>;

    // The two halves of find_transitions, split: the machine's shared
    // wildcard path dispatches exact pairs per state and the wildcard
    // group through one body per (event, target)
    template<typename FROM, typename EVENT>
    using exact_transitions = find_all_exact<FROM, EVENT>;

    template<typename EVENT>
    using wildcard_transitions = find_all_exact<any_state, EVENT>;
};

// A range of acceptable timeouts, e.g. a specification's min/max pair.
// Microsecond resolution: spec bounds may be fractions of a millisecond
struct timeout_range {
    std::chrono::microseconds min;
    std::chrono::microseconds max;

    constexpr bool contains(auto duration) const { return min <= duration && duration <= max; }
};

// Trait behind concepts::timeout_range; specialize it for a custom
// range type providing contains(duration)
template<typename T>
struct is_timeout_range : std::false_type {};

template<>
struct is_timeout_range<timeout_range> : std::true_type {};

namespace concepts {

template<typename T>
concept timeout_range = is_timeout_range<T>::value;

} // namespace concepts

// Timer-range map entry: binds a state to its acceptable timing - a
// timeout_range or an exact duration. A map is a typelist of entries;
// maps compose by typelist concatenation, like the transition tables
// they describe. BOUND is a reference, not a value: chrono durations
// are not structural types, so the constexpr bound object cannot itself
// be a template argument - the reference to it can, and keeps every use
// a constant expression
template<concepts::state STATE, auto const& BOUND>
struct timed_by {
    using state = STATE;
    static constexpr auto& bound = BOUND;
};

namespace internal {

template<typename STATE>
struct entries_for {
    template<typename ENTRY>
    struct pred : std::is_same<typename ENTRY::state, STATE> {};
};

// A range entry must contain the timeout, an exact duration must equal it
template<typename STATE, typename ENTRY,
         bool RANGE = concepts::timeout_range<std::remove_cvref_t<decltype(ENTRY::bound)>>>
struct entry_bounds_timeout : std::bool_constant<ENTRY::bound.contains(STATE::timeout)> {};

template<typename STATE, typename ENTRY>
struct entry_bounds_timeout<STATE, ENTRY, false>
    : std::bool_constant<STATE::timeout == ENTRY::bound> {};

// A timed state needs exactly one entry; the specialization keeps the
// entry's bound uninstantiated for any other count
template<typename STATE, typename MAP,
         std::size_t ENTRIES = mtl::count_if_v<MAP, entries_for<STATE>::template pred>>
struct timed_state_bounded : std::false_type {};

template<typename STATE, typename MAP>
struct timed_state_bounded<STATE, MAP, 1>
    : entry_bounds_timeout<STATE,
                           mtl::find_if_t<MAP, entries_for<STATE>::template pred>> {};

template<typename TABLE>
struct maps_a_state_of {
    template<typename ENTRY>
    struct pred : mtl::has_a<typename TABLE::states, typename ENTRY::state> {};
};

} // namespace internal

// Whether STATE is consistent with a timer-range map (a typelist of
// timed_by entries): a timed state has exactly one entry whose
// timeout_range contains its timeout (an exact duration must equal
// it), an untimed state has none
template<typename MAP, typename STATE, bool TIMED = internal::has_timeout_v<STATE>>
struct timeout_within_bounds : internal::timed_state_bounded<STATE, MAP> {};

template<typename MAP, typename STATE>
struct timeout_within_bounds<MAP, STATE, false>
    : std::bool_constant<
          mtl::count_if_v<MAP, internal::entries_for<STATE>::template pred> == 0> {};

template<typename MAP, typename STATE>
inline constexpr bool timeout_within_bounds_v = timeout_within_bounds<MAP, STATE>::value;

namespace internal {

template<typename MAP>
struct bounded_in {
    template<typename STATE>
    struct pred : timeout_within_bounds<MAP, STATE> {};
};

} // namespace internal

// Proves the table's states consistent with a timer-range map, both
// ways: every timed state bounded by exactly one entry, every entry
// naming a timed state of the table. Assert next to the table (and
// probe individual states with timeout_within_bounds when it fails):
//   static_assert(fsm::timeouts_within_bounds_v<my_table, my_timer_ranges>);
template<typename TABLE, typename MAP>
struct timeouts_within_bounds
    : std::bool_constant<
          mtl::all_of_v<MAP, internal::maps_a_state_of<TABLE>::template pred> &&
          mtl::all_of_v<typename TABLE::states, internal::bounded_in<MAP>::template pred>> {};

template<typename TABLE, typename MAP>
inline constexpr bool timeouts_within_bounds_v = timeouts_within_bounds<TABLE, MAP>::value;

namespace internal {

// The deadline mirror of the timeout bounds chain; maps reuse
// timed_by entries. The zero sentinel counts as no deadline
template<typename STATE, typename ENTRY,
         bool RANGE = concepts::timeout_range<std::remove_cvref_t<decltype(ENTRY::bound)>>>
struct entry_bounds_deadline : std::bool_constant<ENTRY::bound.contains(STATE::deadline)> {};

template<typename STATE, typename ENTRY>
struct entry_bounds_deadline<STATE, ENTRY, false>
    : std::bool_constant<STATE::deadline == ENTRY::bound> {};

template<typename STATE, typename MAP,
         std::size_t ENTRIES = mtl::count_if_v<MAP, entries_for<STATE>::template pred>>
struct deadline_state_bounded : std::false_type {};

template<typename STATE, typename MAP>
struct deadline_state_bounded<STATE, MAP, 1>
    : entry_bounds_deadline<STATE,
                            mtl::find_if_t<MAP, entries_for<STATE>::template pred>> {};

} // namespace internal

// Whether STATE is consistent with a deadline-range map (timed_by
// entries): a state with an active deadline has exactly one entry
// bounding it, every other state has none
template<typename MAP, typename STATE, bool ACTIVE = internal::activeDeadline<STATE>()>
struct deadline_within_bounds : internal::deadline_state_bounded<STATE, MAP> {};

template<typename MAP, typename STATE>
struct deadline_within_bounds<MAP, STATE, false>
    : std::bool_constant<
          mtl::count_if_v<MAP, internal::entries_for<STATE>::template pred> == 0> {};

template<typename MAP, typename STATE>
inline constexpr bool deadline_within_bounds_v = deadline_within_bounds<MAP, STATE>::value;

namespace internal {

template<typename MAP>
struct deadline_bounded_in {
    template<typename STATE>
    struct pred : deadline_within_bounds<MAP, STATE> {};
};

} // namespace internal

// Proves the table's states consistent with a deadline-range map,
// both ways - the deadline counterpart of timeouts_within_bounds
template<typename TABLE, typename MAP>
struct deadlines_within_bounds
    : std::bool_constant<
          mtl::all_of_v<MAP, internal::maps_a_state_of<TABLE>::template pred> &&
          mtl::all_of_v<typename TABLE::states,
                        internal::deadline_bounded_in<MAP>::template pred>> {};

template<typename TABLE, typename MAP>
inline constexpr bool deadlines_within_bounds_v = deadlines_within_bounds<TABLE, MAP>::value;

namespace internal {

// Transitions leaving the SET: internal transitions stay in place, a
// wildcard source leaves from every state (the set is never empty)
template<typename SET>
struct leaves_from {
    template<typename TRANSITION>
    struct pred : std::bool_constant<
        !is_internal_v<TRANSITION> &&
        (std::is_same_v<typename TRANSITION::from, any_state> ||
         mtl::has_a_v<SET, typename TRANSITION::from>)> {};
};

template<typename TRANSITION>
struct to_of : std::type_identity<typename TRANSITION::to> {};

// Fixed point of one-transition expansion; the lazy conditional keeps
// the recursion from instantiating past the fixed point
template<typename SET, typename TRANSITIONS>
struct reachable_closure {
    using targets =
        mtl::transform_t<mtl::filter_t<TRANSITIONS, leaves_from<SET>::template pred>, to_of>;
    using expanded = mtl::unique_t<mtl::concat_t<SET, targets>>;
    using type     = typename std::conditional_t<std::is_same_v<expanded, SET>,
                                                 std::type_identity<SET>,
                                                 reachable_closure<expanded, TRANSITIONS>>::type;
};

template<typename TABLE>
using reachable_states_t =
    typename reachable_closure<mtl::typelist<mtl::front_t<typename TABLE::states>>,
                               typename TABLE::transitions>::type;

} // namespace internal

// Whether the table's transitions can take the machine from its
// initial state to STATE
template<typename TABLE, typename STATE>
struct is_reachable
    : std::bool_constant<mtl::has_a_v<internal::reachable_states_t<TABLE>, STATE>> {};

template<typename TABLE, typename STATE>
inline constexpr bool is_reachable_v = is_reachable<TABLE, STATE>::value;

namespace internal {

template<typename TABLE>
struct reachable_in {
    template<typename STATE>
    struct pred : is_reachable<TABLE, STATE> {};
};

} // namespace internal

// Proves every state of the table reachable from the initial state. A
// state that only appears as a transition source is dead code the
// machine can never enter - typically a leftover of a table edit.
// Assert next to the table (and probe individual states with
// is_reachable when it fails):
//   static_assert(fsm::all_states_reachable_v<my_table>);
template<typename TABLE>
struct all_states_reachable
    : std::bool_constant<
          mtl::all_of_v<typename TABLE::states, internal::reachable_in<TABLE>::template pred>> {};

template<typename TABLE>
inline constexpr bool all_states_reachable_v = all_states_reachable<TABLE>::value;

// Whether the table has a transition - regular, internal, or through
// the any_state wildcard - for EVENT in STATE. The static
// approximation of "the event is not dropped": alternatives whose
// guards all decline at runtime still count as handled
template<typename TABLE, typename STATE, typename EVENT>
struct handles_event
    : std::bool_constant<!std::is_same_v<
          typename TABLE::template find_transition<STATE, EVENT>, mtl::nil_type>> {};

template<typename TABLE, typename STATE, typename EVENT>
inline constexpr bool handles_event_v = handles_event<TABLE, STATE, EVENT>::value;

// --- optional features as tags ----------------------------------------------
// A state declares the feature it belongs to (`using feature = TAG;`),
// an observer declaring the same tag (`using enables = TAG;`, or an
// mtl::typelist of tags) switches the feature on. A disabled feature's
// states - and every table entry touching them, initial<> included -
// are filtered out of the entry list at compile time; states without a
// feature always stay. Build the table from the filtered list, keyed by
// the observers that will be injected:
//   template<typename... OBSERVERs>
//   struct my_table : mtl::rebind_t<fsm::remove_disabled_features_t<entries, OBSERVERs...>,
//                                   fsm::transition_table> {};

namespace internal {

template<typename ENABLES, typename TAG>
struct enables_lists : std::is_same<ENABLES, TAG> {};

template<typename... TAGs, typename TAG>
struct enables_lists<mtl::typelist<TAGs...>, TAG>
    : std::bool_constant<(std::is_same_v<TAGs, TAG> || ...)> {};

template<typename STATE>
concept featured = requires { typename STATE::feature; };

} // namespace internal

// Whether OBSERVER's enables declaration names TAG
template<typename OBSERVER, typename TAG>
struct observer_enables : std::false_type {};

template<typename OBSERVER, typename TAG>
    requires requires { typename OBSERVER::enables; }
struct observer_enables<OBSERVER, TAG> : internal::enables_lists<typename OBSERVER::enables, TAG> {};

template<typename OBSERVER, typename TAG>
inline constexpr bool observer_enables_v = observer_enables<OBSERVER, TAG>::value;

// Whether STATE declares TAG as its feature
template<typename STATE, typename TAG>
struct state_in_feature : std::false_type {};

template<internal::featured STATE, typename TAG>
struct state_in_feature<STATE, TAG> : std::is_same<typename STATE::feature, TAG> {};

template<typename STATE, typename TAG>
inline constexpr bool state_in_feature_v = state_in_feature<STATE, TAG>::value;

// Whether any of the observers enables TAG
template<typename TAG, typename... OBSERVERs>
inline constexpr bool feature_enabled_v = (observer_enables_v<OBSERVERs, TAG> || ...);

namespace internal {

// Entries touching a state the STATE_PRED selects: transitions from or
// to it, an initial<> naming it (the next entry's source leads then),
// and a timer-range map's timed_by<> entry for it
template<template<typename> typename STATE_PRED>
struct entry_touching {
    template<typename ENTRY>
    struct pred : std::false_type {};

    template<concepts::transition ENTRY>
    struct pred<ENTRY> : std::bool_constant<STATE_PRED<typename ENTRY::from>::value ||
                                            STATE_PRED<typename ENTRY::to>::value> {};

    template<typename STATE>
    struct pred<fsm::initial<STATE>> : STATE_PRED<STATE> {};

    template<typename STATE, auto const& BOUND>
    struct pred<fsm::timed_by<STATE, BOUND>> : STATE_PRED<STATE> {};
};

// A state of any of the listed features
template<typename TAGS>
struct in_features;

template<typename... TAGs>
struct in_features<mtl::typelist<TAGs...>> {
    template<typename STATE>
    struct pred : std::bool_constant<(state_in_feature_v<STATE, TAGs> || ...)> {};
};

// A featured state whose feature none of the observers enables
template<typename... OBSERVERs>
struct in_disabled_feature {
    template<typename STATE>
    struct pred : std::false_type {};

    template<featured STATE>
    struct pred<STATE>
        : std::bool_constant<!feature_enabled_v<typename STATE::feature, OBSERVERs...>> {};
};

} // namespace internal

// The entries with every feature none of the observers enables removed:
// the states of those features and everything touching them, in one
// pass; works on transition lists and on timer-range maps alike
template<mtl::concepts::typelist LIST, typename... OBSERVERs>
using remove_disabled_features_t = mtl::remove_if_t<
    LIST,
    internal::entry_touching<internal::in_disabled_feature<OBSERVERs...>::template pred>::template pred>;

// The same for an explicit mtl::typelist of feature tags
template<mtl::concepts::typelist LIST, mtl::concepts::typelist TAGS>
using remove_features_t = mtl::remove_if_t<
    LIST, internal::entry_touching<internal::in_features<TAGS>::template pred>::template pred>;

template<mtl::concepts::typelist LIST, typename TAG>
using remove_feature_t = remove_features_t<LIST, mtl::typelist<TAG>>;

namespace concepts {

// OBSERVER's static observation of STATE reaches a notify hook: the
// annotation exists and a notifyEntry/notifyExit overload accepts it.
// This is the observing dispatch's own requires-expression, so the
// concept cannot drift from what actually runs on an edge
template<typename OBSERVER, typename STATE>
concept notified_of =
    requires(OBSERVER observer) {
        observer.notifyEntry(OBSERVER::template annotation<STATE>());
    } ||
    requires(OBSERVER observer) {
        observer.notifyExit(OBSERVER::template annotation<STATE>());
    };

} // namespace concepts

// Whether OBSERVER's static observation covers STATE at all - even
// without a hook accepting the annotation
template<typename OBSERVER, typename STATE>
struct is_observed : std::bool_constant<internal::observes_v<OBSERVER, STATE>> {};

template<typename OBSERVER, typename STATE>
inline constexpr bool is_observed_v = is_observed<OBSERVER, STATE>::value;

template<typename OBSERVER, typename STATE>
struct is_notified_of : std::bool_constant<concepts::notified_of<OBSERVER, STATE>> {};

template<typename OBSERVER, typename STATE>
inline constexpr bool is_notified_of_v = is_notified_of<OBSERVER, STATE>::value;

namespace internal {

template<typename OBSERVER, typename EXCEPTIONS>
struct notified_in {
    template<typename STATE>
    struct pred : std::bool_constant<is_notified_of_v<OBSERVER, STATE> ||
                                     mtl::has_a_v<EXCEPTIONS, STATE>> {};
};

} // namespace internal

// Proves the observer notified of every state of the table: an
// unannotated state (or one whose annotation no hook accepts) is
// silently skipped by the observing dispatch, which for a driver
// observer means stale hardware on entry. States in EXCEPTIONS may go
// unobserved. Typically asserted from the observer's validate() hook so
// every machine built with the observer is covered
template<typename OBSERVER, typename TABLE, typename EXCEPTIONS = mtl::typelist<>>
struct all_states_notified
    : std::bool_constant<mtl::all_of_v<
          typename TABLE::states,
          internal::notified_in<OBSERVER, EXCEPTIONS>::template pred>> {};

template<typename OBSERVER, typename TABLE, typename EXCEPTIONS = mtl::typelist<>>
inline constexpr bool all_states_notified_v =
    all_states_notified<OBSERVER, TABLE, EXCEPTIONS>::value;

namespace internal {

template<typename OBSERVER, typename TABLE>
constexpr bool validated()
{
    if constexpr (requires { OBSERVER::template validate<TABLE>(); }) {
        OBSERVER::template validate<TABLE>();
    }
    return true;
}

template<typename OLD_STATE, typename NEW_STATE, typename OBSERVER, typename MACHINE>
void exitHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (requires { observer.template onExitState<OLD_STATE, NEW_STATE>(machine); }) {
        observer.template onExitState<OLD_STATE, NEW_STATE>(machine);
    }
}

template<typename OLD_STATE, typename NEW_STATE, typename OBSERVER, typename MACHINE>
void enterHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (requires { observer.template onEnterState<OLD_STATE, NEW_STATE>(machine); }) {
        observer.template onEnterState<OLD_STATE, NEW_STATE>(machine);
    }
}

template<typename FROM_STATE, typename EVENT, typename TO_STATE, typename OBSERVER,
         typename MACHINE>
void transitionHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (requires {
                      observer.template onTransition<FROM_STATE, EVENT, TO_STATE>(machine);
                  }) {
        observer.template onTransition<FROM_STATE, EVENT, TO_STATE>(machine);
    }
}

} // namespace internal

// Composite observer: forwards every hook to caller-owned member
// observers in member order. fsm::observing allows one static and one
// nonstatic observation per observer; a group bundles several such
// observers so they can be injected into the machine as one, letting a
// library predefine a cohesive set behind a single reference
template<typename... OBSERVERs>
class observer_group {
public:
    explicit observer_group(OBSERVERs&... members) : members_(members...) {}

    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert((internal::validated<OBSERVERs, TABLE>() && ...));
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onExitState(MACHINE& machine)
    {
        std::apply(
            [&machine](auto&... member) {
                (internal::exitHook<OLD_STATE, NEW_STATE>(member, machine), ...);
            },
            members_);
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterState(MACHINE& machine)
    {
        std::apply(
            [&machine](auto&... member) {
                (internal::enterHook<OLD_STATE, NEW_STATE>(member, machine), ...);
            },
            members_);
    }

    template<typename FROM_STATE, typename EVENT, typename TO_STATE, typename MACHINE>
    void onTransition(MACHINE& machine)
    {
        std::apply(
            [&machine](auto&... member) {
                (internal::transitionHook<FROM_STATE, EVENT, TO_STATE>(member, machine), ...);
            },
            members_);
    }

private:
    std::tuple<OBSERVERs&...> members_;
};

namespace internal {

// Group detection by derived-to-base conversion: the shared wildcard
// path judges a group by its members, not by the forwarding hooks
template<typename... MEMBERs>
constexpr mtl::typelist<MEMBERs...> groupMembersOf(observer_group<MEMBERs...> const&);

template<typename OBSERVER>
concept grouped_observer = requires(OBSERVER const& observer) { groupMembersOf(observer); };

template<grouped_observer OBSERVER>
using group_members_t = decltype(groupMembersOf(std::declval<OBSERVER const&>()));

} // namespace internal

template<concepts::transition_table TRANSITION_TABLE, typename... OBSERVERs>
class state_machine {
    using TRANSITIONS = TRANSITION_TABLE;

    // Observers get a chance to reject the table at compile time
    static_assert((internal::validated<OBSERVERs, TRANSITIONS>() && ...));

public:
    using table         = TRANSITION_TABLE; // named tables identify the machine (fsm::tracing)
    using state_variant = mtl::rebind_t<typename TRANSITIONS::states, std::variant>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

private:
    using context_states = mtl::filter_t<typename TRANSITIONS::states, internal::has_context>;
    // Deduplicated: states naming the same context type share one instance
    using context_types = mtl::unique_t<mtl::transform_t<context_states, internal::context_of>>;
    using context_tuple = mtl::rebind_t<context_types, std::tuple>;

    static_assert(mtl::all_of_v<context_types, std::is_default_constructible>,
                  "state_machine: context types must be default constructible");

    template<typename STATE>
    struct constructible_from_context
        : std::bool_constant<std::constructible_from<STATE, internal::context_of_t<STATE>&>> {};
    static_assert(mtl::all_of_v<context_states, constructible_from_context>,
                  "state_machine: a context state must be constructible from its context alone");

public:
    explicit state_machine(OBSERVERs&... observers)
        : observers_(observers...),
          current_(std::make_from_tuple<state_variant>(
              internal::initialArgs<initial_state>(contexts_)))
    {
        this->template enter<mtl::nil_type, initial_state>();
    }

    // Observer hooks receive *this and may retain the address beyond the
    // hook: the machine must stay at one address for its lifetime
    state_machine(state_machine const&)            = delete;
    state_machine& operator=(state_machine const&) = delete;

    // Returns true if a transition fired (false: no matching transition, or
    // every alternative's guard said no).
    // A from<any_state> transition is fired through one shared body per
    // (event, target) when that provably cannot be observed - the
    // per-source expansion otherwise emits one near-identical
    // transition body per state (measured kilobytes in a machine with a
    // handful of wildcard events). Shareability is proved per event at
    // compile time (wildcardShareable below); anything unprovable falls
    // back to the exact per-source expansion
    template<typename EVENT>
    bool process(EVENT const& event)
    {
        if constexpr (wildcardShareable<EVENT>()) {
            bool const fired = internal::dispatch(
                [this, &event](auto& state) -> bool {
                    using state_type = std::decay_t<decltype(state)>;
                    using alternatives =
                        typename TRANSITIONS::template exact_transitions<state_type, EVENT>;
                    return this->template tryAlternatives<state_type>(alternatives{}, state,
                                                                      event);
                },
                current_);
            if (fired) {
                return true;
            }
            if (this->template exactAlternativesExist<EVENT>()) {
                return false; // a refused exact group shadows the wildcard
            }
            return this->fireWildcards(
                typename TRANSITIONS::template wildcard_transitions<EVENT>{}, event);
        } else {
            return internal::dispatch(
                [this, &event](auto& state) -> bool {
                    using state_type = std::decay_t<decltype(state)>;
                    using alternatives =
                        typename TRANSITIONS::template find_transitions<state_type, EVENT>;
                    return this->template tryAlternatives<state_type>(alternatives{}, state,
                                                                      event);
                },
                current_);
        }
    }

    // Is STATE the active state?
    template<concepts::state STATE>
    [[nodiscard]] bool is() const
    {
        return std::holds_alternative<STATE>(current_);
    }

    // Pointer to the active state object, nullptr if STATE is not active.
    // The next transition destroys the object: do not keep the pointer.
    template<concepts::state STATE>
    [[nodiscard]] STATE* getIf()
    {
        return std::get_if<STATE>(&current_);
    }

    template<concepts::state STATE>
    [[nodiscard]] STATE const* getIf() const
    {
        return std::get_if<STATE>(&current_);
    }

    // Read-only view of the machine-owned context instance of type T
    // (one some state declares as its context member). Observation
    // only, like getIf(): every mutation goes through process() - a
    // state writes its context, seeded by event payload if needed
    template<typename T>
    [[nodiscard]] T const& context() const
    {
        return std::get<T>(contexts_);
    }

private:
    // --- shared wildcard path -----------------------------------------------

    // An onTransition hook would receive the real source state; on the
    // shared path it gets fsm::any_state instead, which an observer
    // accepts by declaring `static constexpr bool source_agnostic = true`
    template<typename OBSERVER, typename OLD_STATE, typename EVENT, typename NEW_STATE>
    static constexpr bool transitionHookSharesEdge()
    {
        return !requires(OBSERVER observer, state_machine& machine) {
            observer.template onTransition<OLD_STATE, EVENT, NEW_STATE>(machine);
        } || requires { requires OBSERVER::source_agnostic; };
    }

    // One observer's view of the shared edge into NEW_STATE from the
    // unknowable OLD_STATE: the timed observer is handled by the
    // unconditional stop, an observing one must prove its exit silent
    // and its entry source-independent, and a raw per-edge hook (which
    // would receive the real source state) blocks sharing entirely
    template<typename OBSERVER, typename OLD_STATE, typename EVENT, typename NEW_STATE>
    static constexpr bool observerSharesEdge()
    {
        if constexpr (internal::is_timed_v<OBSERVER>) {
            return true;
        } else if constexpr (internal::grouped_observer<OBSERVER>) {
            // judged by its members: the group's own forwarding hooks
            // exist for every edge and would block sharing wholesale
            return membersShareEdge<OLD_STATE, EVENT, NEW_STATE>(
                internal::group_members_t<OBSERVER>{});
        } else if constexpr (!transitionHookSharesEdge<OBSERVER, OLD_STATE, EVENT, NEW_STATE>()) {
            return false;
        } else if constexpr (std::derived_from<OBSERVER, observing<OBSERVER>>) {
            return OBSERVER::template exit_silent<OLD_STATE, state_machine> &&
                   OBSERVER::template entry_shared_from<NEW_STATE, OLD_STATE>;
        } else {
            return !requires(OBSERVER observer, state_machine& machine) {
                observer.template onExitState<OLD_STATE, NEW_STATE>(machine);
            } && !requires(OBSERVER observer, state_machine& machine) {
                observer.template onEnterState<OLD_STATE, NEW_STATE>(machine);
            };
        }
    }

    template<typename OLD_STATE, typename EVENT, typename NEW_STATE, typename... MEMBERs>
    static constexpr bool membersShareEdge(mtl::typelist<MEMBERs...>)
    {
        return (observerSharesEdge<MEMBERs, OLD_STATE, EVENT, NEW_STATE>() && ...);
    }

    // States without an exact group for EVENT: exactly those the
    // wildcard can fire from
    template<typename EVENT>
    struct exactless_for {
        template<typename STATE>
        struct pred
            : std::is_same<typename TRANSITIONS::template exact_transitions<STATE, EVENT>,
                           mtl::typelist<>> {};
    };

    template<typename TO, typename EVENT>
    struct wildcard_source_ok {
        template<typename STATE>
        struct pred
            : std::bool_constant<!requires(STATE state) { state.onExit(); } &&
                                 (observerSharesEdge<OBSERVERs, STATE, EVENT, TO>() && ...)> {};
    };

    template<typename EVENT, typename... WILDCARDs>
    static constexpr bool wildcardsShareable(mtl::typelist<WILDCARDs...>)
    {
        using exactless =
            mtl::filter_t<typename TRANSITIONS::states, exactless_for<EVENT>::template pred>;
        return ((!internal::is_internal_v<WILDCARDs> &&
                 (!internal::has_guard_v<WILDCARDs> ||
                  requires {
                      { WILDCARDs::guard::check() } -> std::convertible_to<bool>;
                  }) &&
                 mtl::all_of_v<exactless, wildcard_source_ok<typename WILDCARDs::to,
                                                             EVENT>::template pred>) &&
                ...);
    }

    template<typename EVENT>
    static constexpr bool wildcardShareable()
    {
        using wildcards = typename TRANSITIONS::template wildcard_transitions<EVENT>;
        if constexpr (std::is_same_v<wildcards, mtl::typelist<>>) {
            return false;
        } else {
            return wildcardsShareable<EVENT>(wildcards{});
        }
    }

    // Whether the active state has an exact group for EVENT - a refused
    // exact group shadows the wildcard, exactly like find_transitions.
    // States without one contribute no code to the fold
    template<typename EVENT>
    bool exactAlternativesExist() const
    {
        return [this]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            return ((!std::is_same_v<
                         typename TRANSITIONS::template exact_transitions<
                             std::variant_alternative_t<INDEXs, state_variant>, EVENT>,
                         mtl::typelist<>> &&
                     current_.index() == INDEXs) ||
                    ...);
        }(std::make_index_sequence<std::variant_size_v<state_variant>>{});
    }

    template<typename... WILDCARDs, typename EVENT>
    bool fireWildcards(mtl::typelist<WILDCARDs...>, EVENT const& event)
    {
        bool fired = false;
        static_cast<void>(((internal::wildcardAllowed<WILDCARDs>() &&
                            (fired = this->template fireShared<WILDCARDs>(event), true)) ||
                           ...));
        return fired;
    }

    // The shared transition body: stop any armed timer (it belongs to
    // the state being left), replace it, and enter the target from
    // any_state - the source is unknowable here, and the shareability
    // proof made that unobservable
    template<typename TRANSITION, typename EVENT>
    bool fireShared(EVENT const& event)
    {
        using NEW_STATE = typename TRANSITION::to;
        std::apply([](auto&... observer) { (internal::stopIfTimed(observer), ...); },
                   observers_);
        if constexpr (internal::payload_constructible_v<NEW_STATE, EVENT>) {
            if constexpr (internal::context_holder<NEW_STATE>) {
                current_.template emplace<NEW_STATE>(
                    event, std::get<internal::context_of_t<NEW_STATE>>(contexts_));
            } else {
                current_.template emplace<NEW_STATE>(event);
            }
        } else if constexpr (internal::context_holder<NEW_STATE>) {
            current_.template emplace<NEW_STATE>(
                std::get<internal::context_of_t<NEW_STATE>>(contexts_));
        } else {
            current_.template emplace<NEW_STATE>();
        }
        this->template enter<any_state, NEW_STATE>();
        this->template notifyTransition<any_state, EVENT, NEW_STATE>();
        return true;
    }

    // --- per-edge path ------------------------------------------------------

    // First alternative whose guard passes fires; false when none does.
    // The fold short-circuits after a firing: the state reference is
    // dangling from that point on
    template<typename STATE, typename... ALTERNATIVEs, typename EVENT>
    bool tryAlternatives(mtl::typelist<ALTERNATIVEs...>, STATE& state, EVENT const& event)
    {
        bool fired = false;
        static_cast<void>(((internal::allowed<ALTERNATIVEs>(state, event) &&
                            (fired = this->template fire<ALTERNATIVEs>(state, event), true)) ||
                           ...));
        return fired;
    }

    // Already instantiated per (transition, state, event): the only
    // place the per-edge bodies below can stay event-agnostic while the
    // onTransition hook still learns the event. After the emplace the
    // state reference is dead - the hook only ever receives the machine
    template<typename TRANSITION, typename STATE, typename EVENT>
    bool fire(STATE& state, EVENT const& event)
    {
        using TO_STATE = typename TRANSITION::to;
        if constexpr (internal::is_internal_v<TRANSITION>) {
            static_assert(requires { state.handle(event); },
                          "internal transition: the state must provide handle(EVENT const&)");
            state.handle(event);
        } else if constexpr (internal::payload_constructible_v<TO_STATE, EVENT>) {
            this->template doTransition<STATE, TO_STATE>(event);
        } else {
            // the emplace does not depend on the event: one body per edge
            this->template doDefaultTransition<STATE, TO_STATE>();
        }
        this->template notifyTransition<STATE, EVENT, TO_STATE>();
        return true;
    }

    // Runs after the transition completed (after onEntry()), so a trace
    // line follows the effects of the change
    template<typename FROM_STATE, typename EVENT, typename TO_STATE>
    void notifyTransition()
    {
        std::apply(
            [this](auto&... observer) {
                (internal::transitionHook<FROM_STATE, EVENT, TO_STATE>(observer, *this), ...);
            },
            observers_);
    }

    // Payload delivery: instantiated per (edge, event) - only for
    // targets constructible from the event
    template<typename OLD_STATE, typename NEW_STATE, typename EVENT>
    bool doTransition(EVENT const& event)
    {
        this->template leave<OLD_STATE, NEW_STATE>();
        if constexpr (internal::context_holder<NEW_STATE>) {
            current_.template emplace<NEW_STATE>(
                event, std::get<internal::context_of_t<NEW_STATE>>(contexts_));
        } else {
            current_.template emplace<NEW_STATE>(event);
        }
        this->template enter<OLD_STATE, NEW_STATE>();
        return true;
    }

    // Event-independent construction: instantiated once per edge and
    // shared by all events triggering it
    template<typename OLD_STATE, typename NEW_STATE>
    bool doDefaultTransition()
    {
        this->template leave<OLD_STATE, NEW_STATE>();
        if constexpr (internal::context_holder<NEW_STATE>) {
            current_.template emplace<NEW_STATE>(
                std::get<internal::context_of_t<NEW_STATE>>(contexts_));
        } else {
            current_.template emplace<NEW_STATE>();
        }
        this->template enter<OLD_STATE, NEW_STATE>();
        return true;
    }

    template<typename OLD_STATE, typename NEW_STATE>
    void leave()
    {
        std::apply([this](auto&... observer) {
                       (internal::exitHook<OLD_STATE, NEW_STATE>(observer, *this), ...);
                   },
                   observers_);
        if constexpr (requires(OLD_STATE& state) { state.onExit(); }) {
            std::get_if<OLD_STATE>(&current_)->onExit();
        }
    }

    template<typename OLD_STATE, typename NEW_STATE>
    void enter()
    {
        std::apply([this](auto&... observer) {
                       (internal::enterHook<OLD_STATE, NEW_STATE>(observer, *this), ...);
                   },
                   observers_);
        if constexpr (requires(NEW_STATE& state) { state.onEntry(); }) {
            std::get_if<NEW_STATE>(&current_)->onEntry();
        }
    }

    context_tuple contexts_{}; // one shared instance per distinct context type
    std::tuple<OBSERVERs&...> observers_;
    state_variant current_; // constructed by the constructor via initialArgs()
};

} // namespace fsm
