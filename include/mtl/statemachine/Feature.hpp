/*
 * fsm: optional features as tags - the states of a feature, the
 * observers enabling it, and the filter removing a disabled feature
 * from an entry list. The contract lives in <mtl/StateMachine.hpp>
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

namespace fsm {

// A timer-range map entry (Timeout.hpp): the filter removes the entry
// of a disabled state from a map as from a table
template<concepts::state STATE, auto const& BOUND>
struct timed_by;

// --- optional features as tags ----------------------------------------------
// A state declares the feature it belongs to (`using feature = TAG;`),
// an observer declaring the same tag (`using enables = TAG;`, or an
// mtl::typelist of tags) switches the feature on. A disabled feature's
// states - and every table entry touching them, initial<> included -
// are filtered out at compile time; states without a feature always
// stay. The machine applies the filter itself, keyed by the observers
// injected into it (fsm::enabled_table_t, Table.hpp), at every level
// of a hierarchy; a timer-range map is filtered by hand with
// remove_disabled_features_t and the same observers

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

// An entry touching a disabled feature, the observers given as one
// mtl::typelist (the form the table and hierarchy traits take);
// mtl::nil_type in its place disables nothing - every state in view
template<typename OBSERVER_LIST>
struct entry_disabled_for;

template<typename... OBSERVERs>
struct entry_disabled_for<mtl::typelist<OBSERVERs...>> {
    template<typename ENTRY>
    struct pred
        : entry_touching<in_disabled_feature<OBSERVERs...>::template pred>::template pred<ENTRY> {};
};

template<>
struct entry_disabled_for<mtl::nil_type> {
    template<typename ENTRY>
    struct pred : std::false_type {};
};

// Whether any entry of LIST touches a feature none of the observers
// enables: the question a machine asks before filtering, so a table
// with nothing to remove is used as it is
template<mtl::concepts::typelist LIST, typename OBSERVER_LIST>
inline constexpr bool has_disabled_features_v =
    mtl::any_of_v<LIST, entry_disabled_for<OBSERVER_LIST>::template pred>;

template<mtl::concepts::typelist LIST, typename OBSERVER_LIST>
using without_disabled_features_t =
    mtl::remove_if_t<LIST, entry_disabled_for<OBSERVER_LIST>::template pred>;

} // namespace internal

// The entries with every feature none of the observers enables removed:
// the states of those features and everything touching them, in one
// pass; works on transition lists and on timer-range maps alike
template<mtl::concepts::typelist LIST, typename... OBSERVERs>
using remove_disabled_features_t =
    internal::without_disabled_features_t<LIST, mtl::typelist<OBSERVERs...>>;

// The same for an explicit mtl::typelist of feature tags
template<mtl::concepts::typelist LIST, mtl::concepts::typelist TAGS>
using remove_features_t = mtl::remove_if_t<
    LIST, internal::entry_touching<internal::in_features<TAGS>::template pred>::template pred>;

template<mtl::concepts::typelist LIST, typename TAG>
using remove_feature_t = remove_features_t<LIST, mtl::typelist<TAG>>;

} // namespace fsm
