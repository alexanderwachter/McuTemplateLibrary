/*
 * Template-based state machine on top of the mtl library.
 * SPDX-License-Identifier: Apache-2.0
 *
 * States are classes with optional members detected by requires-expressions:
 * on_entry(), on_exit(), static constexpr timeout, and static constexpr
 * members watched by observers. The state set is derived from the table;
 * an initial<STATE> table role picks the initial state (default: the first
 * state of the first transition).
 *
 * A guard gates the single transition it is attached to ((state, event)
 * pairs stay unique). check() returning false blocks it: process() returns
 * false, no exit/entry/hook runs, a running timeout timer keeps running,
 * but a blocked fsm::timeout transition does not re-arm the one-shot timer.
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
 *   void on_exit_state(MACHINE&);  - transition fires, old state still alive
 *   void on_enter_state(MACHINE&); - new state emplaced, before on_entry();
 *                                    on construction with OLD = mtl::nil_type
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

template<typename STATE>
struct from {};

template<typename EVENT>
struct on {};

template<typename STATE>
struct to {};

template<typename GUARD>
struct guard {};

template<typename STATE>
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

} // namespace internal

namespace concepts {

// check(from_state) for conditions on state data, check() for
// state-independent ones
template<typename GUARD, typename STATE>
concept guard = requires(STATE const& state) {
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

} // namespace concepts

// The named arguments may appear in any order
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
    using guard = typename internal::unwrap<mtl::find_if_t<args, internal::is_guard>>::type; // nil_type if absent

private:
    static_assert(std::is_same_v<guard, mtl::nil_type> || concepts::guard<guard, from>,
                  "transition: guard must provide static bool check(FROM const&) "
                  "or static bool check()");
};

// Event injected by the timer policy when a state's timeout expires
struct timeout {};

using timer_callback = void (*)(void*);

namespace concepts {

template<typename T>
concept timer = requires(T t, std::chrono::milliseconds duration,
                         timer_callback callback, void* context) {
    t.start(duration, callback, context);
    t.stop();
};

} // namespace concepts

namespace internal {

template<mtl::concepts::typelist LIST>
struct to_variant;

template<typename... ELEMENTs>
struct to_variant<mtl::typelist<ELEMENTs...>> {
    using type = std::variant<ELEMENTs...>;
};

template<mtl::concepts::typelist LIST>
using to_variant_t = typename to_variant<LIST>::type;

// Optional drop-in for the std::visit call in process(), for toolchains
// where std::visit emits a function-pointer table or bad_variant_access
// handling (e.g. clang/libc++, older GCC). The last alternative dispatches
// unconditionally: the variant only holds nothrow-constructible states and
// can never be valueless. On GCC 13 x86-64 -Os std::visit is 96 bytes
// smaller than this chain - hence it is the default.
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

template<typename FROM, typename EVENT>
struct matches {
    template<typename TRANSITION>
    struct pred : std::bool_constant<std::is_same_v<typename TRANSITION::from, FROM> &&
                                     std::is_same_v<typename TRANSITION::event, EVENT>> {};
};

// All from/to states of a transition list, in order of appearance
template<mtl::concepts::typelist LIST>
struct endpoints;

template<typename... TRANSITIONs>
struct endpoints<mtl::typelist<TRANSITIONs...>> {
    using type = mtl::typelist<typename TRANSITIONs::from..., typename TRANSITIONs::to...>;
};

template<typename LIST>
struct unambiguous_in {
    template<typename TRANSITION>
    struct pred : std::bool_constant<
        mtl::count_if_v<LIST, matches<typename TRANSITION::from,
                                      typename TRANSITION::event>::template pred> == 1> {};
};

template<typename STATE>
inline constexpr bool has_timeout_v = requires { STATE::timeout; };

template<typename TRANSITION>
inline constexpr bool has_guard_v = !std::is_same_v<typename TRANSITION::guard, mtl::nil_type>;

template<typename GUARD, typename STATE>
bool check_guard([[maybe_unused]] STATE const& state)
{
    if constexpr (requires { GUARD::check(state); }) {
        return GUARD::check(state);
    } else {
        return GUARD::check();
    }
}

template<typename OBSERVER, typename STATE>
inline constexpr bool observes_v = requires { OBSERVER::template annotation<STATE>(); };

} // namespace internal

// Value observer base: the derived class names the watched member once and
// provides notify_entry(value) (new state's value) and/or notify_exit(value)
// (old state's value, old state still alive), each optional:
//
//   struct lamp_driver : fsm::observing<lamp_driver> {
//       static constexpr auto observe(auto const& state) -> decltype(state.lamps)
//       {
//           return state.lamps;
//       }
//       void notify_entry(lamps_t const& lamps);
//   };
//
// The trailing return type makes states without the member drop out via
// SFINAE; a custom annotation<STATE>() may replace observe(). The change
// check runs at compile time: edges between equal values emit no code.
// Observed states must be constexpr default-constructible.
template<typename DERIVED>
struct observing {
    template<typename STATE>
    static constexpr auto annotation()
        requires requires(STATE const& state) { DERIVED::observe(state); }
    {
        return DERIVED::observe(STATE{});
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void on_exit_state(MACHINE&)
    {
        if constexpr (annotation_changes<OLD_STATE, NEW_STATE>()) {
            auto& self = static_cast<DERIVED&>(*this);
            if constexpr (requires { self.notify_exit(DERIVED::template annotation<OLD_STATE>()); }) {
                self.notify_exit(DERIVED::template annotation<OLD_STATE>());
            }
        }
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void on_enter_state(MACHINE&)
    {
        if constexpr (annotation_changes<NEW_STATE, OLD_STATE>()) {
            auto& self = static_cast<DERIVED&>(*this);
            if constexpr (requires { self.notify_entry(DERIVED::template annotation<NEW_STATE>()); }) {
                self.notify_entry(DERIVED::template annotation<NEW_STATE>());
            }
        }
    }

private:
    template<typename STATE, typename OTHER>
    static constexpr bool annotation_changes()
    {
        if constexpr (!internal::observes_v<DERIVED, STATE>) {
            return false;
        } else if constexpr (!internal::observes_v<DERIVED, OTHER>) {
            return true;
        } else {
            return DERIVED::template annotation<STATE>() != DERIVED::template annotation<OTHER>();
        }
    }
};

// Observer implementing the state-timeout semantics on top of a TIMER policy
template<concepts::timer TIMER>
struct timed {
    static constexpr bool handles_timeout = true;

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void on_exit_state(MACHINE&)
    {
        if constexpr (internal::has_timeout_v<OLD_STATE>) {
            timer.stop(); // no timer may fire mid-transition
        }
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void on_enter_state(MACHINE& machine)
    {
        if constexpr (internal::has_timeout_v<NEW_STATE>) {
            timer.start(
                std::chrono::ceil<std::chrono::milliseconds>(NEW_STATE::timeout),
                [](void* context) {
                    static_cast<MACHINE*>(context)->process(timeout{});
                },
                &machine);
        }
    }

    TIMER timer{};
};

// Transitions plus an optional initial<STATE> role; without it the first
// state of the first transition is the initial state
template<concepts::transition_table_entry... ENTRYs>
struct transition_table {
private:
    using entries = mtl::typelist<ENTRYs...>;
    static_assert(mtl::count_if_v<entries, internal::is_initial> <= 1,
                  "transition_table: at most one initial<STATE> allowed");

    using explicit_initial =
        typename internal::unwrap<mtl::find_if_t<entries, internal::is_initial>>::type;

public:
    using transitions = mtl::remove_if_t<entries, internal::is_initial>;

private:
    static_assert(mtl::all_of_v<transitions, internal::unambiguous_in<transitions>::template pred>,
                  "transition_table: duplicate (state, event) pair");

    using endpoints = typename internal::endpoints<transitions>::type;
    static_assert(std::is_same_v<explicit_initial, mtl::nil_type> ||
                      mtl::has_a_v<endpoints, explicit_initial>,
                  "transition_table: initial<STATE> is not a state of the table");

public:
    // Deduplicated in order of first appearance: front is the initial state
    using states = mtl::unique_t<std::conditional_t<
        std::is_same_v<explicit_initial, mtl::nil_type>,
        endpoints,
        mtl::prepend_t<explicit_initial, endpoints>>>;

    // mtl::nil_type if the pair is not in the table
    template<typename FROM, typename EVENT>
    using find_transition =
        mtl::find_if_t<transitions, internal::matches<FROM, EVENT>::template pred>;
};

template<typename TRANSITION_TABLE, typename... OBSERVERs>
class state_machine {
    using TRANSITIONS = TRANSITION_TABLE;

    template<typename STATE>
    struct timed_state_handled
        : std::bool_constant<
              !internal::has_timeout_v<STATE> ||
              !std::is_same_v<typename TRANSITIONS::template find_transition<STATE, timeout>,
                              mtl::nil_type>> {};
    static_assert(mtl::all_of_v<typename TRANSITIONS::states, timed_state_handled>,
                  "state_machine: state has a timeout but no transition for fsm::timeout");

    template<typename STATE>
    struct is_timed : std::bool_constant<internal::has_timeout_v<STATE>> {};

    template<typename OBSERVER>
    static constexpr bool handles_timeout_v = requires { requires OBSERVER::handles_timeout; };

    static_assert(mtl::count_if_v<typename TRANSITIONS::states, is_timed> == 0U ||
                      (handles_timeout_v<OBSERVERs> || ...),
                  "state_machine: table has timed states but no timeout-capable "
                  "observer (inject fsm::timed<TIMER>)");

public:
    using state_variant = internal::to_variant_t<typename TRANSITIONS::states>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

    explicit state_machine(OBSERVERs&... observers)
        : observers_(observers...)
    {
        this->template enter<mtl::nil_type, initial_state>();
    }

    // Returns true if a transition fired (false: no matching transition, or
    // its guard said no)
    template<typename EVENT>
    bool process(EVENT const&)
    {
        return std::visit(
            [this](auto& state) -> bool {
                using state_type = std::decay_t<decltype(state)>;
                using matched = typename TRANSITIONS::template find_transition<state_type, EVENT>;
                if constexpr (std::is_same_v<matched, mtl::nil_type>) {
                    return false;
                } else {
                    if constexpr (internal::has_guard_v<matched>) {
                        if (!internal::check_guard<typename matched::guard>(state)) {
                            return false;
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

    template<typename STATE>
    [[nodiscard]] STATE* get_if()
    {
        return std::get_if<STATE>(&current_);
    }

private:
    template<typename OLD_STATE, typename NEW_STATE, typename OBSERVER>
    void exit_hook(OBSERVER& observer)
    {
        if constexpr (requires { observer.template on_exit_state<OLD_STATE, NEW_STATE>(*this); }) {
            observer.template on_exit_state<OLD_STATE, NEW_STATE>(*this);
        }
    }

    template<typename OLD_STATE, typename NEW_STATE, typename OBSERVER>
    void enter_hook(OBSERVER& observer)
    {
        if constexpr (requires { observer.template on_enter_state<OLD_STATE, NEW_STATE>(*this); }) {
            observer.template on_enter_state<OLD_STATE, NEW_STATE>(*this);
        }
    }

    // Instantiated per edge, not per event: all events triggering the same
    // edge share one instantiation
    template<typename OLD_STATE, typename NEW_STATE>
    bool do_transition()
    {
        std::apply([this](auto&... observer) {
                       (exit_hook<OLD_STATE, NEW_STATE>(observer), ...);
                   },
                   observers_);
        if constexpr (requires(OLD_STATE& state) { state.on_exit(); }) {
            std::get_if<OLD_STATE>(&current_)->on_exit();
        }
        current_.template emplace<NEW_STATE>();
        this->template enter<OLD_STATE, NEW_STATE>();
        return true;
    }

    template<typename OLD_STATE, typename NEW_STATE>
    void enter()
    {
        std::apply([this](auto&... observer) {
                       (enter_hook<OLD_STATE, NEW_STATE>(observer), ...);
                   },
                   observers_);
        if constexpr (requires(NEW_STATE& state) { state.on_entry(); }) {
            std::get_if<NEW_STATE>(&current_)->on_entry();
        }
    }

    state_variant current_{}; // default-constructs the initial state
    std::tuple<OBSERVERs&...> observers_;
};

} // namespace fsm
