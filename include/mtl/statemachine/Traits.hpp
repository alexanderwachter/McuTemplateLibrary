/*
 * fsm: compile-time checks on tables and observers - reachability,
 * event handling, optional features as tags, observer coverage
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

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
    } || internal::set_notified_v<OBSERVER, STATE>;

} // namespace concepts

// Whether OBSERVER's static observation covers STATE at all - even
// without a hook accepting the annotation; a state's annotation set
// counts when one of its elements reaches a hook
template<typename OBSERVER, typename STATE>
struct is_observed : std::bool_constant<internal::observes_v<OBSERVER, STATE> ||
                                        internal::set_notified_v<OBSERVER, STATE>> {};

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

} // namespace fsm
