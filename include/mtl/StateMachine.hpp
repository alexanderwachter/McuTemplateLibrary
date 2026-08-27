/*
 * Template-based state machine on top of the mtl library.
 * SPDX-License-Identifier: Apache-2.0
 *
 * - Each state is a class. Optional members, detected via requires-expressions:
 *     void on_entry();
 *     void on_exit();
 *     static constexpr <duration> timeout;
 *     static constexpr <value> members observed by observer policies (below)
 * - The transition table is a pack of transition<from<A>, on<E>, to<B>>;
 *   the named arguments may appear in any order. A transition may carry an
 *   optional guard<G> (see fsm::concepts::guard): G::check(from_state) or
 *   G::check() is evaluated first when the event is processed; returning
 *   false blocks the transition - process() returns false, no
 *   on_exit/on_entry/notify runs, and a running timeout timer keeps
 *   running. A blocked fsm::timeout transition does NOT re-arm the
 *   one-shot timer. (state, event) pairs stay unique: a guard gates the
 *   single matching transition, it does not select among alternatives.
 * - The set of states is derived from the table (mtl::unique) and stored in
 *   a std::variant. The first state of the first transition is the initial
 *   state.
 * - Timeouts are delegated to a timer policy (see fsm::concepts::timer):
 *   the machine starts the timer on entry into a timed state and stops it
 *   when leaving; the timer callback injects fsm::timeout into process().
 *
 * Timer policy contract:
 *   void start(std::chrono::milliseconds, fsm::timer_callback callback, void* context);
 *     Arms a one-shot timer. Invokes callback(context) once after the
 *     duration. Restarting an armed timer re-arms it.
 *   void stop();
 *     Disarms the timer. Must be tolerated when the timer is not armed
 *     (e.g. after it already fired).
 *   The callback runs in the policy's execution context (superloop, work
 *   queue, ISR, ...). process() is not re-entrant and not thread-safe; the
 *   policy or its user must ensure callback and process() are serialized.
 *
 * Observer policy contract (any number can be injected):
 *   template<typename STATE>
 *   static constexpr auto annotation() requires requires { STATE::member; }
 *   { return STATE::member; }
 *     Extracts the observed static constexpr member from a state. The
 *     requires-clause makes the observer opt out of states that do not
 *     carry the member.
 *   void notify(<annotation value>);
 *     Called with the new state's annotation value
 *     - once on construction, with the initial state's value (if present)
 *     - on every transition where the value differs from the previous
 *       state's value (compared at compile time), or where the previous
 *       state had no annotation.
 *     Transitions between states with equal values, and transitions into
 *     states without the annotation, do not notify.
 *   Observers are notified after the timer is armed and before on_entry()
 *   of the new state runs. Observers are injected by reference into the
 *   constructor and are not owned by the machine: they must outlive it.
 *   Note that the first notification happens during machine construction.
 *   The fsm::observing CRTP base generates annotation() for the common
 *   observe-one-member case; deriving from it is optional.
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

// Named parameters for a transition
template<typename STATE>
struct from {};

template<typename EVENT>
struct on {};

template<typename STATE>
struct to {};

template<typename GUARD>
struct guard {};

namespace internal {

template<typename T> struct is_from : std::false_type {};
template<typename S> struct is_from<fsm::from<S>> : std::true_type {};

template<typename T> struct is_on : std::false_type {};
template<typename E> struct is_on<fsm::on<E>> : std::true_type {};

template<typename T> struct is_to : std::false_type {};
template<typename S> struct is_to<fsm::to<S>> : std::true_type {};

template<typename T> struct is_guard : std::false_type {};
template<typename G> struct is_guard<fsm::guard<G>> : std::true_type {};

template<typename T> struct unwrap;
template<typename S> struct unwrap<fsm::from<S>>  { using type = S; };
template<typename E> struct unwrap<fsm::on<E>>    { using type = E; };
template<typename S> struct unwrap<fsm::to<S>>    { using type = S; };
template<typename G> struct unwrap<fsm::guard<G>> { using type = G; };
template<>           struct unwrap<mtl::nil_type> { using type = mtl::nil_type; };

} // namespace internal

namespace concepts {

// Guard contract: called with the transition's from-state for conditions on
// state data, or with no arguments for state-independent conditions.
template<typename GUARD, typename STATE>
concept guard = requires(STATE const& state) {
    { GUARD::check(state) } -> std::convertible_to<bool>;
} || requires {
    { GUARD::check() } -> std::convertible_to<bool>;
};

} // namespace concepts

// One entry of the transition table. The named arguments may appear in any
// order: transition<from<a>, on<e>, to<b>> == transition<on<e>, to<b>, from<a>>
template<typename... ARGs>
struct transition {
private:
    using args = mtl::typelist<ARGs...>;
    static_assert(mtl::count_if_v<args, internal::is_from> == 1,
                  "transition: exactly one from<STATE> required");
    static_assert(mtl::count_if_v<args, internal::is_on> == 1,
                  "transition: exactly one on<EVENT> required");
    static_assert(mtl::count_if_v<args, internal::is_to> == 1,
                  "transition: exactly one to<STATE> required");
    static_assert(mtl::count_if_v<args, internal::is_guard> <= 1,
                  "transition: at most one guard<GUARD> allowed");
    static_assert(sizeof...(ARGs) == 3U + mtl::count_if_v<args, internal::is_guard>,
                  "transition: takes exactly from<>, on<>, to<> and optionally guard<>");

public:
    using from  = typename internal::unwrap<mtl::find_if_t<args, internal::is_from>>::type;
    using event = typename internal::unwrap<mtl::find_if_t<args, internal::is_on>>::type;
    using to    = typename internal::unwrap<mtl::find_if_t<args, internal::is_to>>::type;
    // mtl::nil_type when the transition has no guard
    using guard = typename internal::unwrap<mtl::find_if_t<args, internal::is_guard>>::type;

private:
    static_assert(std::is_same_v<guard, mtl::nil_type> || concepts::guard<guard, from>,
                  "transition: guard must provide static bool check(FROM const&) "
                  "or static bool check()");
};

// Event injected by the timer policy when a state's timeout expires
struct timeout {};

// Signature of the expiry callback a timer policy has to store and invoke
using timer_callback = void (*)(void*);

namespace concepts {

template<typename T>
concept timer = requires(T t, std::chrono::milliseconds duration,
                         timer_callback callback, void* context) {
    t.start(duration, callback, context);
    t.stop();
};

} // namespace concepts

// CRTP convenience base for the common observer that watches one static
// constexpr member. The derived observer names the member once, in a getter
// with a trailing return type:
//
//   struct lamp_driver : fsm::observing<lamp_driver> {
//       static constexpr auto observe(auto const& state) -> decltype(state.lamps)
//       {
//           return state.lamps;
//       }
//       void notify(lamps_t const& lamps);
//   };
//
// The trailing return type is what makes states without the member drop out:
// substitution fails in the immediate context, so annotation() below is
// simply not provided for them - no requires-clause needed in the derived
// class. The state is default-constructed to evaluate the getter at compile
// time; states must therefore be constexpr default-constructible (the
// machine already requires them to be default-constructible).
template<typename DERIVED>
struct observing {
    template<typename STATE>
    static constexpr auto annotation()
        requires requires(STATE const& state) { DERIVED::observe(state); }
    {
        return DERIVED::observe(STATE{});
    }
};

namespace internal {

// Rebind a mtl::typelist into a std::variant
template<mtl::concepts::typelist LIST>
struct to_variant;

template<typename... ELEMENTs>
struct to_variant<mtl::typelist<ELEMENTs...>> {
    using type = std::variant<ELEMENTs...>;
};

template<mtl::concepts::typelist LIST>
using to_variant_t = typename to_variant<LIST>::type;

// Optional drop-in replacement for std::visit (swap the std::visit call
// site below to internal::visit). Expands to an index-compare chain the
// optimizer folds into a switch: every visitor instantiation is inlinable,
// no function-pointer table can be emitted, and there is no
// valueless_by_exception path (the machine's variant only holds
// nothrow-constructible states and can never become valueless; the last
// alternative is therefore dispatched unconditionally).
// Measured on GCC 13 / x86-64 at -Os: std::visit already compiles to a
// switch with no table and no bad_variant_access code, and is 96 bytes
// SMALLER than this chain - hence std::visit is the default. Use this
// alternative if your toolchain (e.g. clang/libc++, older GCC) emits
// function-pointer tables or pulls in the throw machinery for std::visit.
template<std::size_t INDEX = 0, typename VISITOR, typename VARIANT>
constexpr decltype(auto) visit(VISITOR&& visitor, VARIANT& variant)
{
    constexpr std::size_t last = std::variant_size_v<VARIANT> - 1U;
    if constexpr (INDEX == last) {
        return std::forward<VISITOR>(visitor)(*std::get_if<INDEX>(&variant));
    } else {
        if (variant.index() == INDEX) {
            return visitor(*std::get_if<INDEX>(&variant));
        }
        return visit<INDEX + 1U>(std::forward<VISITOR>(visitor), variant);
    }
}

// Predicate factory: matches<FROM, EVENT>::pred<TRANSITION> is true if
// TRANSITION handles EVENT in state FROM. Usable with mtl::find_if /
// mtl::count_if.
template<typename FROM, typename EVENT>
struct matches {
    template<typename TRANSITION>
    struct pred : std::bool_constant<std::is_same_v<typename TRANSITION::from, FROM> &&
                                     std::is_same_v<typename TRANSITION::event, EVENT>> {};
};

template<typename STATE>
inline constexpr bool has_timeout_v = requires { STATE::timeout; };

template<typename TRANSITION>
inline constexpr bool has_guard_v = !std::is_same_v<typename TRANSITION::guard, mtl::nil_type>;

// Evaluate a transition's guard role against the current (from) state.
template<typename GUARD, typename STATE>
bool check_guard([[maybe_unused]] STATE const& state)
{
    if constexpr (requires { GUARD::check(state); }) {
        return GUARD::check(state);
    } else {
        return GUARD::check();
    }
}

// Does OBSERVER observe STATE, i.e. does annotation<STATE>() exist?
template<typename OBSERVER, typename STATE>
inline constexpr bool observes_v = requires { OBSERVER::template annotation<STATE>(); };

} // namespace internal

template<typename... TRANSITIONs>
struct transition_table {
    using transitions = mtl::typelist<TRANSITIONs...>;

    // Every (from, event) pair must be unambiguous
    static_assert(((mtl::count_if_v<transitions,
                        internal::matches<typename TRANSITIONs::from,
                                          typename TRANSITIONs::event>::template pred> == 1) && ...),
                  "transition_table: duplicate (state, event) pair");

    // All states of the table, deduplicated, in order of first appearance.
    using states = mtl::unique_t<
        mtl::typelist<typename TRANSITIONs::from..., typename TRANSITIONs::to...>>;

    // First transition matching (FROM, EVENT), or mtl::nil_type if the pair
    // is not in the table.
    template<typename FROM, typename EVENT>
    using find_transition =
        mtl::find_if_t<transitions, internal::matches<FROM, EVENT>::template pred>;
};

template<typename TRANSITION_TABLE, concepts::timer TIMER, typename... OBSERVERs>
class state_machine {
    using TRANSITIONS = TRANSITION_TABLE;

    // A state with a timeout but no transition for fsm::timeout is a bug:
    // the timer would fire into a table that ignores it.
    template<typename STATE>
    struct timed_state_handled
        : std::bool_constant<
              !internal::has_timeout_v<STATE> ||
              !std::is_same_v<typename TRANSITIONS::template find_transition<STATE, timeout>,
                              mtl::nil_type>> {};
    static_assert(mtl::all_of_v<typename TRANSITIONS::states, timed_state_handled>,
                  "state_machine: state has a timeout but no transition for fsm::timeout");

public:
    using state_variant = internal::to_variant_t<typename TRANSITIONS::states>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

    state_machine()
        requires (sizeof...(OBSERVERs) == 0) && std::is_default_constructible_v<TIMER>
        : state_machine(TIMER{})
    {
    }

    // Observers are injected by reference and must outlive the state machine.
    explicit state_machine(TIMER timer, OBSERVERs&... observers)
        : timer_(std::move(timer)), observers_(observers...)
    {
        this->template enter<mtl::nil_type, initial_state>(); // observers get the initial value
    }

    // Feed an event through the table. Returns true if a transition fired
    // (false: no matching transition, or its guard said no). The visitor
    // only selects and evaluates the guard; the transition body lives in
    // the per-edge function do_transition<OLD, NEW>, shared by all events
    // that trigger the same edge.
    template<typename EVENT>
    bool process(EVENT const&)
    {
        return std::visit(
            [this](auto& state) -> bool {
                using state_type = std::decay_t<decltype(state)>;
                using matched = typename TRANSITIONS::template find_transition<state_type, EVENT>;
                if constexpr (std::is_same_v<matched, mtl::nil_type>) {
                    return false; // this state ignores this event
                } else {
                    if constexpr (internal::has_guard_v<matched>) {
                        if (!internal::check_guard<typename matched::guard>(state)) {
                            return false; // guard blocked the transition
                        }
                    }
                    return this->template do_transition<state_type, typename matched::to>();
                }
            },
            current_);
    }

    template<typename STATE>
    [[nodiscard]] bool is() const
    {
        return std::holds_alternative<STATE>(current_);
    }

    // Access the current state object for a given type (e.g. to read outputs)
    template<typename STATE>
    [[nodiscard]] STATE* get_if()
    {
        return std::get_if<STATE>(&current_);
    }

    // Access the timer policy (e.g. for configuration or tests)
    [[nodiscard]] TIMER& timer() { return timer_; }

private:
    // Notify one observer if NEW_STATE carries its annotation and the value
    // changed relative to OLD_STATE (or OLD_STATE had none). The comparison
    // happens at compile time: unchanged annotations generate no code.
    template<typename OLD_STATE, typename NEW_STATE, typename OBSERVER>
    void notify(OBSERVER& observer)
    {
        if constexpr (internal::observes_v<OBSERVER, NEW_STATE>) {
            if constexpr (!internal::observes_v<OBSERVER, OLD_STATE>) {
                observer.notify(OBSERVER::template annotation<NEW_STATE>());
            } else if constexpr (OBSERVER::template annotation<OLD_STATE>() !=
                                 OBSERVER::template annotation<NEW_STATE>()) {
                observer.notify(OBSERVER::template annotation<NEW_STATE>());
            }
        }
    }

    // Transition body, instantiated once per (OLD_STATE, NEW_STATE) edge of
    // the table - independent of the triggering event: all events that
    // trigger the same edge share this instantiation.
    template<typename OLD_STATE, typename NEW_STATE>
    bool do_transition()
    {
        if constexpr (internal::has_timeout_v<OLD_STATE>) {
            timer_.stop(); // no timer may fire mid-transition
        }
        if constexpr (requires(OLD_STATE& state) { state.on_exit(); }) {
            std::get_if<OLD_STATE>(&current_)->on_exit();
        }
        current_.template emplace<NEW_STATE>();
        this->template enter<OLD_STATE, NEW_STATE>();
        return true;
    }

    // Entry into NEW_STATE: both endpoints are statically known, so no
    // visit is needed here.
    template<typename OLD_STATE, typename NEW_STATE>
    void enter()
    {
        if constexpr (internal::has_timeout_v<NEW_STATE>) {
            timer_.start(
                std::chrono::ceil<std::chrono::milliseconds>(NEW_STATE::timeout),
                [](void* context) {
                    static_cast<state_machine*>(context)->process(timeout{});
                },
                this);
        }
        std::apply([this](auto&... observer) {
                       (notify<OLD_STATE, NEW_STATE>(observer), ...);
                   },
                   observers_);
        if constexpr (requires(NEW_STATE& state) { state.on_entry(); }) {
            std::get_if<NEW_STATE>(&current_)->on_entry();
        }
    }

    state_variant current_{}; // default-constructs the initial state
    TIMER timer_;
    std::tuple<OBSERVERs&...> observers_;
};

} // namespace fsm
