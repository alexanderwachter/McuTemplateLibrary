/*
 * fsm: timeout and deadline annotations of states, and the timer-range
 * maps (timed_by) proving them against a specification
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Table.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <chrono>
#include <cstddef>
#include <type_traits>

namespace fsm {

namespace internal {

template<typename STATE>
inline constexpr bool has_timeout_v = requires { STATE::timeout; };

template<typename TABLE>
// A timed state needs an unguarded fsm::timeout alternative (own or
// wildcard): a refused timeout would leave the state without its
// one-shot timer
struct timeout_handled_in {
    template<typename STATE>
    struct pred : std::bool_constant<
        !has_timeout_v<STATE> ||
        mtl::any_of_v<transitions_for_t<TABLE, STATE, timeout>, is_unguarded>> {};
};

// A zero deadline is the phase-target sentinel: it stops the clock
// like an unannotated state, but says so explicitly
template<typename STATE>
inline constexpr bool active_deadline_v =
    requires { requires STATE::deadline != decltype(STATE::deadline){}; };

// Whether the edge continues one running phase: the state left
// carries the same nonzero deadline as the one entered
template<typename OLD_STATE, typename NEW_STATE>
inline constexpr bool continues_deadline_v =
    active_deadline_v<OLD_STATE> &&
    requires { requires OLD_STATE::deadline == NEW_STATE::deadline; };

template<typename TABLE>
struct deadline_handled_in {
    template<typename STATE>
    struct pred : std::bool_constant<
        !active_deadline_v<STATE> ||
        mtl::any_of_v<transitions_for_t<TABLE, STATE, deadline>, is_unguarded>> {};
};

template<typename STATE>
struct is_timed_state : std::bool_constant<has_timeout_v<STATE>> {};

template<typename STATE>
struct is_deadlined_state : std::bool_constant<active_deadline_v<STATE>> {};

} // namespace internal

// Whether any state of TABLE carries a timeout / an active deadline:
// what a facade asks to decide which timers a machine needs at all
template<typename TABLE>
inline constexpr bool has_timed_states_v =
    mtl::any_of_v<typename TABLE::states, internal::is_timed_state>;

template<typename TABLE>
inline constexpr bool has_deadlined_states_v =
    mtl::any_of_v<typename TABLE::states, internal::is_deadlined_state>;

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

// The duration a map bounds: a state's timeout, or its deadline
template<typename STATE>
struct timeout_of {
    static constexpr auto value = STATE::timeout;
};

template<typename STATE>
struct deadline_of {
    static constexpr auto value = STATE::deadline;
};

// A range entry must contain the duration, an exact duration must equal it
template<typename STATE, typename ENTRY, template<typename> typename DURATION,
         bool RANGE = concepts::timeout_range<std::remove_cvref_t<decltype(ENTRY::bound)>>>
struct entry_bounds : std::bool_constant<ENTRY::bound.contains(DURATION<STATE>::value)> {};

template<typename STATE, typename ENTRY, template<typename> typename DURATION>
struct entry_bounds<STATE, ENTRY, DURATION, false>
    : std::bool_constant<DURATION<STATE>::value == ENTRY::bound> {};

// A bounded state needs exactly one entry; the specialization keeps the
// entry's bound uninstantiated for any other count
template<typename STATE, typename MAP, template<typename> typename DURATION,
         std::size_t ENTRIES = mtl::count_if_v<MAP, entries_for<STATE>::template pred>>
struct state_bounded : std::false_type {};

template<typename STATE, typename MAP, template<typename> typename DURATION>
struct state_bounded<STATE, MAP, DURATION, 1>
    : entry_bounds<STATE, mtl::find_if_t<MAP, entries_for<STATE>::template pred>, DURATION> {};

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
struct timeout_within_bounds : internal::state_bounded<STATE, MAP, internal::timeout_of> {};

template<typename MAP, typename STATE>
struct timeout_within_bounds<MAP, STATE, false>
    : std::bool_constant<
          mtl::count_if_v<MAP, internal::entries_for<STATE>::template pred> == 0> {};

template<typename MAP, typename STATE>
inline constexpr bool timeout_within_bounds_v = timeout_within_bounds<MAP, STATE>::value;

namespace internal {

// The per-state check of a map as a predicate over the table's states
template<typename MAP, template<typename, typename> typename WITHIN_BOUNDS>
struct bounded_in {
    template<typename STATE>
    struct pred : WITHIN_BOUNDS<MAP, STATE> {};
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
          mtl::all_of_v<typename TABLE::states,
                        internal::bounded_in<MAP, timeout_within_bounds>::template pred>> {};

template<typename TABLE, typename MAP>
inline constexpr bool timeouts_within_bounds_v = timeouts_within_bounds<TABLE, MAP>::value;

// Whether STATE is consistent with a deadline-range map (timed_by
// entries): a state with an active deadline has exactly one entry
// bounding it, every other state has none. The zero sentinel counts as
// no deadline
template<typename MAP, typename STATE, bool ACTIVE = internal::active_deadline_v<STATE>>
struct deadline_within_bounds : internal::state_bounded<STATE, MAP, internal::deadline_of> {};

template<typename MAP, typename STATE>
struct deadline_within_bounds<MAP, STATE, false>
    : std::bool_constant<
          mtl::count_if_v<MAP, internal::entries_for<STATE>::template pred> == 0> {};

template<typename MAP, typename STATE>
inline constexpr bool deadline_within_bounds_v = deadline_within_bounds<MAP, STATE>::value;

// Proves the table's states consistent with a deadline-range map,
// both ways - the deadline counterpart of timeouts_within_bounds
template<typename TABLE, typename MAP>
struct deadlines_within_bounds
    : std::bool_constant<
          mtl::all_of_v<MAP, internal::maps_a_state_of<TABLE>::template pred> &&
          mtl::all_of_v<typename TABLE::states,
                        internal::bounded_in<MAP, deadline_within_bounds>::template pred>> {};

template<typename TABLE, typename MAP>
inline constexpr bool deadlines_within_bounds_v = deadlines_within_bounds<TABLE, MAP>::value;

} // namespace fsm
