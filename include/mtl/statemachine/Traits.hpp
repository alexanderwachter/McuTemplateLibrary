/*
 * fsm: compile-time checks on tables and observers - reachability,
 * event handling, optional features as tags, observer coverage
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Feature.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Timeout.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

namespace fsm {

namespace internal {

// Transitions leaving the SET: internal transitions stay in place, a
// wildcard source leaves from every state (the set is never empty)
template<mtl::concepts::typelist SET>
struct leaves_from {
    template<concepts::transition TRANSITION>
    struct pred : std::bool_constant<
        !is_internal_v<TRANSITION> &&
        (std::is_same_v<typename TRANSITION::from, any_state> ||
         mtl::has_a_v<SET, typename TRANSITION::from>)> {};
};

template<concepts::transition TRANSITION>
struct to_of : std::type_identity<typename TRANSITION::to> {};

// Fixed point of one-transition expansion; the lazy conditional keeps
// the recursion from instantiating past the fixed point
template<mtl::concepts::typelist SET, mtl::concepts::typelist TRANSITIONS>
struct reachable_closure {
    using targets =
        mtl::transform_t<mtl::filter_t<TRANSITIONS, leaves_from<SET>::template pred>, to_of>;
    using expanded = mtl::unique_t<mtl::concat_t<SET, targets>>;
    using type     = typename std::conditional_t<std::is_same_v<expanded, SET>,
                                                 std::type_identity<SET>,
                                                 reachable_closure<expanded, TRANSITIONS>>::type;
};

template<concepts::transition_table TABLE>
using reachable_states_t =
    typename reachable_closure<mtl::typelist<mtl::front_t<typename TABLE::states>>,
                               typename TABLE::transitions>::type;

} // namespace internal

// Whether the table's transitions can take the machine from its
// initial state to STATE
template<concepts::transition_table TABLE, concepts::state STATE>
struct is_reachable
    : std::bool_constant<mtl::has_a_v<internal::reachable_states_t<TABLE>, STATE>> {};

template<concepts::transition_table TABLE, concepts::state STATE>
inline constexpr bool is_reachable_v = is_reachable<TABLE, STATE>::value;

namespace internal {

template<concepts::transition_table TABLE>
struct reachable_in {
    template<concepts::state STATE>
    struct pred : is_reachable<TABLE, STATE> {};
};

template<concepts::transition_table TABLE>
struct every_state_reachable_in
    : std::bool_constant<
          mtl::all_of_v<typename TABLE::states, reachable_in<TABLE>::template pred>> {};

} // namespace internal

// Proves every state of the table reachable from the initial state -
// and every sub-state from its submachine's initial state. A state
// that only appears as a transition source is dead code the machine
// can never enter - typically a leftover of a table edit.
// Assert next to the table (and probe individual states with
// is_reachable when it fails):
//   static_assert(fsm::all_states_reachable_v<my_table>);
template<concepts::transition_table TABLE>
struct all_states_reachable
    : std::bool_constant<
          mtl::all_of_v<nested_tables_t<TABLE>, internal::every_state_reachable_in>> {};

template<concepts::transition_table TABLE>
inline constexpr bool all_states_reachable_v = all_states_reachable<TABLE>::value;

// Whether the table has a transition - regular, internal, or through
// the any_state wildcard - for EVENT in STATE. The static
// approximation of "the event is not dropped": alternatives whose
// guards all decline at runtime still count as handled
template<concepts::transition_table TABLE, concepts::state STATE, concepts::event EVENT>
struct handles_event
    : std::bool_constant<
          !std::is_same_v<transition_for_t<TABLE, STATE, EVENT>, mtl::nil_type>> {};

template<concepts::transition_table TABLE, concepts::state STATE, concepts::event EVENT>
inline constexpr bool handles_event_v = handles_event<TABLE, STATE, EVENT>::value;

// Whether OBSERVER's observation covers STATE at all - a static
// annotation even without a hook accepting it; a state's annotation set
// or instance values count when one of their elements reaches a hook
template<concepts::observer OBSERVER, concepts::state STATE>
struct is_observed : std::bool_constant<internal::observes_v<OBSERVER, STATE> ||
                                        internal::set_notified_v<OBSERVER, STATE> ||
                                        internal::values_notified_v<OBSERVER, STATE>> {};

template<concepts::observer OBSERVER, concepts::state STATE>
inline constexpr bool is_observed_v = is_observed<OBSERVER, STATE>::value;

template<concepts::observer OBSERVER, concepts::state STATE>
struct is_notified_of : std::bool_constant<concepts::notified_of<OBSERVER, STATE>> {};

template<concepts::observer OBSERVER, concepts::state STATE>
inline constexpr bool is_notified_of_v = is_notified_of<OBSERVER, STATE>::value;

namespace internal {

template<concepts::observer OBSERVER, mtl::concepts::typelist EXCEPTIONS>
struct notified_in {
    template<concepts::state STATE>
    struct pred : std::bool_constant<is_notified_of_v<OBSERVER, STATE> ||
                                     mtl::has_a_v<EXCEPTIONS, STATE>> {};
};

} // namespace internal

// Proves the observer notified of every state of the table, sub-tables
// included: an unannotated state (or one whose annotation no hook
// accepts) is silently skipped by the observing dispatch, which for a
// driver observer means stale hardware on entry. States in EXCEPTIONS
// may go unobserved. Typically asserted from the observer's validate()
// hook so every machine built with the observer is covered
template<concepts::observer OBSERVER, concepts::transition_table TABLE,
         mtl::concepts::typelist EXCEPTIONS = mtl::typelist<>>
struct all_states_notified
    : std::bool_constant<mtl::all_of_v<
          all_states_t<TABLE>,
          internal::notified_in<OBSERVER, EXCEPTIONS>::template pred>> {};

template<concepts::observer OBSERVER, concepts::transition_table TABLE,
         mtl::concepts::typelist EXCEPTIONS = mtl::typelist<>>
inline constexpr bool all_states_notified_v =
    all_states_notified<OBSERVER, TABLE, EXCEPTIONS>::value;

namespace internal {

// T is carried at some level of every active path through TABLE: by
// the state itself, or - for a composite state that leaves T to the
// level below - by every state of its submachine. Below a composite
// state carrying T nothing is asked: its value holds for the whole
// submachine
template<concepts::transition_table TABLE, concepts::annotation T,
         mtl::concepts::typelist EXCEPTIONS>
struct carried_throughout;

template<concepts::annotation T, mtl::concepts::typelist EXCEPTIONS>
struct carried_by {
    template<concepts::state STATE>
    struct pred : std::bool_constant<carrying<T>::template pred<STATE>::value ||
                                     mtl::has_a_v<EXCEPTIONS, STATE>> {};

    template<composite STATE>
    struct pred<STATE>
        : std::bool_constant<carrying<T>::template pred<STATE>::value ||
                             mtl::has_a_v<EXCEPTIONS, STATE> ||
                             carried_throughout<submachine_t<STATE>, T, EXCEPTIONS>::value> {};
};

template<concepts::transition_table TABLE, concepts::annotation T,
         mtl::concepts::typelist EXCEPTIONS>
struct carried_throughout
    : std::bool_constant<
          mtl::all_of_v<typename TABLE::states, carried_by<T, EXCEPTIONS>::template pred>> {};

// Every event REQUIRED_EVENTS<STATE>::type names has a transition
// from STATE in TABLE - the state's own table, a sub-state's row
// counts only there
template<concepts::transition_table TABLE, template<typename> typename REQUIRED_EVENTS>
struct handles_required_in {
    template<concepts::state STATE>
    struct handled_by {
        template<concepts::event EVENT>
        struct pred : handles_event<TABLE, STATE, EVENT> {};
    };

    template<concepts::state STATE>
    struct pred : std::bool_constant<mtl::all_of_v<typename REQUIRED_EVENTS<STATE>::type,
                                                   handled_by<STATE>::template pred>> {};
};

template<template<typename> typename REQUIRED_EVENTS>
struct table_handles_required {
    template<concepts::transition_table TABLE>
    struct pred : std::bool_constant<mtl::all_of_v<
                      typename TABLE::states,
                      handles_required_in<TABLE, REQUIRED_EVENTS>::template pred>> {};
};

} // namespace internal

// Proves the annotation T - in a state's static set or its instance
// values - is in force whatever the machine's active states are, so an
// observer consuming T always knows the value that applies. A state
// carries T itself, or the composite state above it does for its whole
// submachine (the level where the value changes annotates it); a
// composite state without T is covered by the states of its
// submachine. States in EXCEPTIONS may go without. One element at a
// time: a driver applying two elements asks twice, and a state
// carrying only one of them fails the other question (one observer
// accepting both would pass it)
template<concepts::transition_table TABLE, concepts::annotation T,
         mtl::concepts::typelist EXCEPTIONS = mtl::typelist<>>
struct all_states_carry : internal::carried_throughout<TABLE, T, EXCEPTIONS> {};

template<concepts::transition_table TABLE, concepts::annotation T,
         mtl::concepts::typelist EXCEPTIONS = mtl::typelist<>>
inline constexpr bool all_states_carry_v = all_states_carry<TABLE, T, EXCEPTIONS>::value;

// Proves every state handles the events it owes: REQUIRED_EVENTS<STATE>
// ::type is the fsm::events<...> STATE must have a transition
// for in its own table (empty for a state owing nothing) - an
// environment report the table would silently drop is a bug, a lost
// detach at worst. Walks the sub-tables too, each state against its
// own table
template<concepts::transition_table TABLE, template<typename> typename REQUIRED_EVENTS>
struct all_states_handle
    : std::bool_constant<mtl::all_of_v<
          nested_tables_t<TABLE>,
          internal::table_handles_required<REQUIRED_EVENTS>::template pred>> {};

template<concepts::transition_table TABLE, template<typename> typename REQUIRED_EVENTS>
inline constexpr bool all_states_handle_v = all_states_handle<TABLE, REQUIRED_EVENTS>::value;

} // namespace fsm
