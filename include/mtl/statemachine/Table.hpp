/*
 * fsm: the transition table, its lookups and the guard evaluation
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <algorithm>
#include <cstddef>
#include <type_traits>

namespace fsm {

namespace concepts {

template<typename T>
concept transition_table = mtl::concepts::typelist<typename T::transitions> &&
                           mtl::concepts::typelist<typename T::states>;

} // namespace concepts

// The table's lookups (transition_table below) as traits: spelled
// without typename/template in dependent contexts
template<concepts::transition_table TABLE, typename FROM, typename EVENT>
using exact_transitions_t = typename TABLE::template exact_transitions<FROM, EVENT>;

template<concepts::transition_table TABLE, typename EVENT>
using wildcard_transitions_t = typename TABLE::template wildcard_transitions<EVENT>;

template<concepts::transition_table TABLE, typename FROM, typename EVENT>
using transitions_for_t = typename TABLE::template transitions_for<FROM, EVENT>;

template<concepts::transition_table TABLE, typename FROM, typename EVENT>
using transition_for_t = typename TABLE::template transition_for<FROM, EVENT>;

namespace internal {

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

template<typename TRANSITION>
struct event_of : std::type_identity<typename TRANSITION::event> {};

// All from/to states of a transition list, in order of appearance
template<mtl::concepts::typelist LIST>
struct endpoints;

template<typename... TRANSITIONs>
struct endpoints<mtl::typelist<TRANSITIONs...>> {
    using type = mtl::typelist<typename TRANSITIONs::from..., typename TRANSITIONs::to...>;
};

template<typename TRANSITION>
inline constexpr bool has_guard_v = !mtl::empty_v<typename TRANSITION::guards>;

template<typename TRANSITION>
struct is_guarded : std::bool_constant<has_guard_v<TRANSITION>> {};

template<typename TRANSITION>
struct is_unguarded : std::bool_constant<!has_guard_v<TRANSITION>> {};

// The most specific guard form wins. A static guard answers itself -
template<typename GUARD, concepts::state STATE, typename EVENT>
bool checkStaticGuard([[maybe_unused]] STATE const& state, [[maybe_unused]] EVENT const& event)
{
    if constexpr (concepts::event_guard_for<GUARD, STATE, EVENT>) {
        return GUARD::check(state, event);
    } else if constexpr (concepts::state_guard_for<GUARD, STATE>) {
        return GUARD::check(state);
    } else {
        return GUARD::check();
    }
}

// - an injected object answers with the tag selecting its overload
template<typename GUARD, typename OBJECT, concepts::state STATE, typename EVENT>
bool askGuard(OBJECT& object, [[maybe_unused]] STATE const& state,
              [[maybe_unused]] EVENT const& event)
{
    if constexpr (concepts::answers_event_guard<OBJECT, GUARD, STATE, EVENT>) {
        return object.check(GUARD{}, state, event);
    } else if constexpr (concepts::answers_state_guard<OBJECT, GUARD, STATE>) {
        return object.check(GUARD{}, state);
    } else {
        return object.check(GUARD{});
    }
}

// The injected objects answering GUARD asked from STATE
template<typename GUARD, typename STATE>
struct answering {
    template<typename OBJECT>
    struct pred : std::bool_constant<concepts::answers_guard_for<OBJECT, GUARD, STATE>> {};
};

// Whether the machine can resolve one part of a row's guard asked from
// FROM: answered by exactly one of the injected OBJECTS, or by a static
// check of the guard's own (a not_<G> part resolves G)
template<typename OBJECTS, typename FROM>
struct part_answered_in {
    template<typename PART, typename GUARD = guard_of_t<PART>>
    struct pred
        : std::bool_constant<
              mtl::count_if_v<OBJECTS, answering<GUARD, FROM>::template pred> == 1 ||
              (mtl::count_if_v<OBJECTS, answering<GUARD, FROM>::template pred> == 0 &&
               concepts::guard_for<GUARD, FROM>)> {};
};

template<typename OBJECTS, typename FROM>
struct part_answered_once_in {
    template<typename PART>
    struct pred
        : std::bool_constant<
              mtl::count_if_v<OBJECTS, answering<guard_of_t<PART>, FROM>::template pred> <= 1> {};
};

// ... and every part of TRANSITION's guard
template<typename OBJECTS>
struct guard_answered_in {
    template<typename TRANSITION>
    struct pred
        : std::bool_constant<mtl::all_of_v<
              typename TRANSITION::guards,
              part_answered_in<OBJECTS, typename TRANSITION::from>::template pred>> {};
};

template<typename OBJECTS>
struct guard_answered_once_in {
    template<typename TRANSITION>
    struct pred
        : std::bool_constant<mtl::all_of_v<
              typename TRANSITION::guards,
              part_answered_once_in<OBJECTS, typename TRANSITION::from>::template pred>> {};
};

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

} // namespace internal

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

public:
    // Deduplicated in order of first appearance: front is the initial state
    using states = mtl::unique_t<std::conditional_t<
        std::is_same_v<explicit_initial, mtl::nil_type>,
        endpoints,
        mtl::prepend_t<explicit_initial, endpoints>>>;

    // Every event the table reacts to, deduplicated in table order:
    // the alternatives of fsm::queued's event storage
    using events = mtl::unique_t<mtl::transform_t<transitions, internal::event_of>>;

    // The exact (FROM, EVENT) group and the (any_state, EVENT) wildcard
    // group, each in table order: the machine's shared wildcard path
    // dispatches exact pairs per state and the wildcard group through
    // one body per (event, target)
    template<typename FROM, typename EVENT>
    using exact_transitions = mtl::filter_t<typename from_group<FROM>::type,
                                            internal::matches_event<EVENT>::template pred>;

    template<typename EVENT>
    using wildcard_transitions = exact_transitions<any_state, EVENT>;

    // All alternatives for (FROM, EVENT) in priority order: the exact
    // group, then the wildcard group (dead behind an unguarded exact
    // entry, which always fires)
    template<typename FROM, typename EVENT>
    using transitions_for =
        mtl::concat_t<exact_transitions<FROM, EVENT>, wildcard_transitions<EVENT>>;

    // The first alternative; mtl::nil_type if there is none
    template<typename FROM, typename EVENT>
    using transition_for = mtl::front_or_t<transitions_for<FROM, EVENT>, mtl::nil_type>;
};

namespace internal {

// Whether STATE can reach a wildcard for EVENT: an unguarded own entry
// always fires first and overrides it
template<typename TABLE, typename STATE, typename EVENT>
inline constexpr bool wildcard_source_v = std::is_same_v<
    mtl::find_if_t<exact_transitions_t<TABLE, STATE, EVENT>, is_unguarded>, mtl::nil_type>;

// --- hierarchy ----------------------------------------------------------------
// A composite state declares the table of its submachine:
//   using submachine = my_sub_table;
// The machine owning the table constructs a machine of that table when
// the state is entered and destroys it when the state is left
template<typename STATE>
concept composite = requires { typename STATE::submachine; } &&
                    concepts::transition_table<typename STATE::submachine>;

template<typename STATE>
struct submachine : std::type_identity<mtl::nil_type> {};

template<composite STATE>
struct submachine<STATE> : std::type_identity<typename STATE::submachine> {};

template<typename STATE>
using submachine_t = typename submachine<STATE>::type;

template<typename STATE>
struct is_composite : std::bool_constant<composite<STATE>> {};

// Every context type the states of TABLE declare, deduplicated, in
// order of first appearance
template<typename TABLE>
using table_contexts_t =
    mtl::unique_t<mtl::linearize_t<mtl::transform_t<typename TABLE::states, contexts_of>>>;

// A composite state lends contexts of its own machine to its
// submachine:
//   using parent_contexts = mtl::typelist<line_status>;
// A sub-state declaring a lent type binds to the parent machine's
// instance instead of a fresh one - the same lifetime as the parent's;
// every other context type of the sub-table is the child's own, fresh
// on each entry of the composite. A lent type the submachine does not
// use itself may be lent further down by a composite of the sub-table
template<typename STATE>
concept context_lender = requires { typename STATE::parent_contexts; } &&
                         mtl::concepts::typelist<typename STATE::parent_contexts>;

template<typename STATE>
struct parent_contexts_of : std::type_identity<mtl::typelist<>> {};

template<context_lender STATE>
struct parent_contexts_of<STATE> : std::type_identity<typename STATE::parent_contexts> {};

template<typename STATE>
using parent_contexts_t = typename parent_contexts_of<STATE>::type;

// Membership of a list, as a predicate
template<typename LIST>
struct member_of {
    template<typename T>
    struct pred : std::bool_constant<mtl::has_a_v<LIST, T>> {};
};

// The checks on a lending state, each a trait so a failing one can be
// asked per state: only a composite lends; it lends what its machine
// holds (CONTEXTS: the machine's own and inherited contexts); and what
// it lends, some state below it declares - a lent type nobody uses is
// a dead declaration
template<typename STATE>
struct lender_is_composite : std::bool_constant<!context_lender<STATE> || composite<STATE>> {};

template<typename CONTEXTS>
struct lends_from {
    template<typename STATE>
    struct pred : std::bool_constant<mtl::all_of_v<parent_contexts_t<STATE>,
                                                   member_of<CONTEXTS>::template pred>> {};
};

// The table a parent machine builds its child machine from: the
// sub-table itself plus the child's nesting depth (the root is 0),
// which the timer observers use to pick their timer slot, and the
// contexts the composite lends it. Every user-facing alias
// (StateMachine::table, trace names, validate) sees the plain table
template<concepts::transition_table TABLE, std::size_t DEPTH,
         mtl::concepts::typelist INHERITED = mtl::typelist<>>
struct nested : TABLE {
    static constexpr std::size_t depth = DEPTH;
    using inherited_contexts            = INHERITED;
};

template<typename TABLE>
struct plain_table : std::type_identity<TABLE> {};

template<typename TABLE, std::size_t DEPTH, typename INHERITED>
struct plain_table<nested<TABLE, DEPTH, INHERITED>> : std::type_identity<TABLE> {};

template<typename TABLE>
using plain_table_t = typename plain_table<TABLE>::type;

template<typename TABLE>
inline constexpr std::size_t table_depth_v = 0;

template<typename TABLE, std::size_t DEPTH, typename INHERITED>
inline constexpr std::size_t table_depth_v<nested<TABLE, DEPTH, INHERITED>> = DEPTH;

// The contexts a machine inherits from its parent: none for a root
template<typename TABLE>
struct inherited_contexts : std::type_identity<mtl::typelist<>> {};

template<typename TABLE, std::size_t DEPTH, typename INHERITED>
struct inherited_contexts<nested<TABLE, DEPTH, INHERITED>> : std::type_identity<INHERITED> {};

template<typename TABLE>
using inherited_contexts_t = typename inherited_contexts<TABLE>::type;

// The number of machine levels below a state: none for a plain state,
// the levels of its submachine for a composite one
template<typename TABLE>
struct levels;

template<typename STATE>
struct levels_below : std::integral_constant<std::size_t, 0> {};

template<composite STATE>
struct levels_below<STATE> : levels<submachine_t<STATE>> {};

template<typename... STATEs>
constexpr std::size_t maxLevelsBelow(mtl::typelist<STATEs...>)
{
    return std::max({std::size_t{0}, levels_below<STATEs>::value...});
}

template<typename TABLE>
struct levels
    : std::integral_constant<std::size_t, 1 + maxLevelsBelow(typename TABLE::states{})> {};

// TABLE and every table nested below it, TABLE first
template<typename TABLE>
struct nested_tables;

template<typename STATE>
struct tables_below : std::type_identity<mtl::typelist<>> {};

template<composite STATE>
struct tables_below<STATE> : nested_tables<submachine_t<STATE>> {};

template<typename TABLE>
struct nested_tables
    : std::type_identity<mtl::unique_t<mtl::prepend_t<
          TABLE, mtl::linearize_t<mtl::transform_t<typename TABLE::states, tables_below>>>>> {};

template<typename TABLE>
struct states_of : std::type_identity<typename TABLE::states> {};

template<typename TABLE>
struct events_of_table : std::type_identity<typename TABLE::events> {};

} // namespace internal

// How many machine levels a table spans: 1 for a flat table, one more
// per level of composite states below it. A machine at nesting depth d
// (root 0) is level d; fsm::timed<TIMER, LEVELS> needs one timer per
// level
template<concepts::transition_table TABLE>
inline constexpr std::size_t levels_v = internal::levels<TABLE>::value;

// TABLE followed by every sub-table it nests, recursively, each once:
// what a table-wide proof walks when it has to cover the whole machine
template<concepts::transition_table TABLE>
using nested_tables_t = typename internal::nested_tables<TABLE>::type;

// The states of TABLE and of every table nested below it, in that order
template<concepts::transition_table TABLE>
using all_states_t =
    mtl::linearize_t<mtl::transform_t<nested_tables_t<TABLE>, internal::states_of>>;

// Every event any level of the machine reacts to, each once: the
// alternatives of a queued machine's event storage
template<concepts::transition_table TABLE>
using nested_events_t = mtl::unique_t<
    mtl::linearize_t<mtl::transform_t<nested_tables_t<TABLE>, internal::events_of_table>>>;

namespace internal {

// The contexts declared anywhere below a composite state: what it may
// usefully lend
template<typename STATE>
struct contexts_below : std::type_identity<mtl::typelist<>> {};

template<composite STATE>
struct contexts_below<STATE>
    : std::type_identity<mtl::unique_t<mtl::linearize_t<
          mtl::transform_t<all_states_t<submachine_t<STATE>>, contexts_of>>>> {};

template<typename STATE>
struct lent_contexts_declared_below
    : std::bool_constant<
          mtl::all_of_v<parent_contexts_t<STATE>,
                        member_of<typename contexts_below<STATE>::type>::template pred>> {};

} // namespace internal

} // namespace fsm
