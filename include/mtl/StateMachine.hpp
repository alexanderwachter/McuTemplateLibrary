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
 * A guard gates the transition it is attached to. Transitions may share
 * a (state, event) pair when guards distinguish them: the alternatives
 * are tried in table order and the first whose guard passes fires; an
 * unguarded alternative is the catch-all and must be the last of its
 * group. When no alternative fires, process() returns false, no
 * exit/entry/hook runs, a running timeout timer keeps running, but a
 * blocked fsm::timeout transition does not re-arm the one-shot timer.
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
 *                                    on construction with OLD = mtl::nil_type
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
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

namespace fsm {

// Matches every state in from<>; a specific (state, event) transition
// takes precedence over the wildcard
struct any_state {};

// Event injected by the timer policy when a state's timeout expires
struct timeout {};

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

} // namespace internal

namespace concepts {

template<typename T>
concept guard = requires { &T::check; };

// check(from_state) for conditions on state data, check() for
// state-independent ones
template<typename GUARD, typename STATE>
concept guard_for = requires(STATE const& state) {
    { GUARD::check(state) } -> std::convertible_to<bool>;
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

template<typename TRANSITION>
inline constexpr bool has_guard_v = !std::is_same_v<typename TRANSITION::guard, mtl::nil_type>;

template<concepts::guard GUARD, concepts::state STATE>
bool checkGuard([[maybe_unused]] STATE const& state)
{
    if constexpr (requires { GUARD::check(state); }) {
        return GUARD::check(state);
    } else {
        return GUARD::check();
    }
}

// True when TRANSITION may fire from the given state instance
template<typename TRANSITION, typename STATE>
bool allowed(STATE const& state)
{
    if constexpr (has_guard_v<TRANSITION>) {
        return checkGuard<typename TRANSITION::guard>(state);
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
            timer.start(
                std::chrono::ceil<std::chrono::milliseconds>(STATE::timeout),
                [](void* context) {
                    static_cast<MACHINE*>(context)->process(timeout{});
                },
                &machine);
        }
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

    template<typename FROM, typename EVENT>
    using find_exact = mtl::find_if_t<transitions, internal::matches<FROM, EVENT>::template pred>;

    template<typename FROM, typename EVENT>
    using find_all_exact = mtl::filter_t<transitions, internal::matches<FROM, EVENT>::template pred>;

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
};

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

} // namespace internal

template<concepts::transition_table TRANSITION_TABLE, typename... OBSERVERs>
class state_machine {
    using TRANSITIONS = TRANSITION_TABLE;

    // Observers get a chance to reject the table at compile time
    static_assert((internal::validated<OBSERVERs, TRANSITIONS>() && ...));

public:
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
    // every alternative's guard said no)
    template<typename EVENT>
    bool process(EVENT const& event)
    {
        return internal::dispatch(
            [this, &event](auto& state) -> bool {
                using state_type = std::decay_t<decltype(state)>;
                using alternatives =
                    typename TRANSITIONS::template find_transitions<state_type, EVENT>;
                return this->template tryAlternatives<state_type>(alternatives{}, state, event);
            },
            current_);
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

private:
    // First alternative whose guard passes fires; false when none does.
    // The fold short-circuits after a firing: the state reference is
    // dangling from that point on
    template<typename STATE, typename... ALTERNATIVEs, typename EVENT>
    bool tryAlternatives(mtl::typelist<ALTERNATIVEs...>, STATE& state, EVENT const& event)
    {
        bool fired = false;
        static_cast<void>(((internal::allowed<ALTERNATIVEs>(state) &&
                            (fired = this->template fire<ALTERNATIVEs>(state, event), true)) ||
                           ...));
        return fired;
    }

    template<typename TRANSITION, typename STATE, typename EVENT>
    bool fire(STATE& state, EVENT const& event)
    {
        if constexpr (internal::is_internal_v<TRANSITION>) {
            static_assert(requires { state.handle(event); },
                          "internal transition: the state must provide handle(EVENT const&)");
            state.handle(event);
            return true;
        } else if constexpr (internal::payload_constructible_v<typename TRANSITION::to, EVENT>) {
            return this->template doTransition<STATE, typename TRANSITION::to>(event);
        } else {
            // the emplace does not depend on the event: one body per edge
            return this->template doDefaultTransition<STATE, typename TRANSITION::to>();
        }
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
