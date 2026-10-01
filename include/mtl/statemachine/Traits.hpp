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
    : std::bool_constant<
          !std::is_same_v<transition_for_t<TABLE, STATE, EVENT>, mtl::nil_type>> {};

template<typename TABLE, typename STATE, typename EVENT>
inline constexpr bool handles_event_v = handles_event<TABLE, STATE, EVENT>::value;

// Whether OBSERVER's observation covers STATE at all - a static
// annotation even without a hook accepting it; a state's annotation set
// or instance values count when one of their elements reaches a hook
template<typename OBSERVER, typename STATE>
struct is_observed : std::bool_constant<internal::observes_v<OBSERVER, STATE> ||
                                        internal::set_notified_v<OBSERVER, STATE> ||
                                        internal::values_notified_v<OBSERVER, STATE>> {};

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

// Proves the observer notified of every state of the table, sub-tables
// included: an unannotated state (or one whose annotation no hook
// accepts) is silently skipped by the observing dispatch, which for a
// driver observer means stale hardware on entry. States in EXCEPTIONS
// may go unobserved. Typically asserted from the observer's validate()
// hook so every machine built with the observer is covered
template<typename OBSERVER, typename TABLE, typename EXCEPTIONS = mtl::typelist<>>
struct all_states_notified
    : std::bool_constant<mtl::all_of_v<
          all_states_t<TABLE>,
          internal::notified_in<OBSERVER, EXCEPTIONS>::template pred>> {};

template<typename OBSERVER, typename TABLE, typename EXCEPTIONS = mtl::typelist<>>
inline constexpr bool all_states_notified_v =
    all_states_notified<OBSERVER, TABLE, EXCEPTIONS>::value;

namespace internal {

template<typename T, typename EXCEPTIONS>
struct carrying_unless {
    template<typename STATE>
    struct pred : std::bool_constant<carrying<T>::template pred<STATE>::value ||
                                     mtl::has_a_v<EXCEPTIONS, STATE>> {};
};

// Every event REQUIRED_EVENTS<STATE>::type names has a transition
// from STATE in TABLE - the state's own table, a sub-state's row
// counts only there
template<typename TABLE, template<typename> typename REQUIRED_EVENTS>
struct handles_required_in {
    template<typename STATE>
    struct handled_by {
        template<typename EVENT>
        struct pred : handles_event<TABLE, STATE, EVENT> {};
    };

    template<typename STATE>
    struct pred : std::bool_constant<mtl::all_of_v<typename REQUIRED_EVENTS<STATE>::type,
                                                   handled_by<STATE>::template pred>> {};
};

template<template<typename> typename REQUIRED_EVENTS>
struct table_handles_required {
    template<typename TABLE>
    struct pred : std::bool_constant<mtl::all_of_v<
                      typename TABLE::states,
                      handles_required_in<TABLE, REQUIRED_EVENTS>::template pred>> {};
};

} // namespace internal

// Proves every state of the table, sub-tables included, carries the
// annotation T - in its static set or its instance values - so an
// observer consuming T hears about every entry. States in EXCEPTIONS
// may go without. One element at a time: a driver applying two
// elements asks twice, and a state carrying only one of them fails
// the other question (one observer accepting both would pass it)
template<typename TABLE, typename T, typename EXCEPTIONS = mtl::typelist<>>
struct all_states_carry
    : std::bool_constant<mtl::all_of_v<all_states_t<TABLE>,
                                       internal::carrying_unless<T, EXCEPTIONS>::template pred>> {
};

template<typename TABLE, typename T, typename EXCEPTIONS = mtl::typelist<>>
inline constexpr bool all_states_carry_v = all_states_carry<TABLE, T, EXCEPTIONS>::value;

// Proves every state handles the events it owes: REQUIRED_EVENTS<STATE>
// ::type is the mtl::typelist of events STATE must have a transition
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
