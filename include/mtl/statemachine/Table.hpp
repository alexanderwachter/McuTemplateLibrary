/*
 * fsm: the transition table, its lookups and the guard evaluation
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Feature.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <algorithm>
#include <cstddef>
#include <type_traits>

namespace fsm {

namespace internal {

// fsm::transition_table<...> as written, not a struct derived from it
template<typename T>
struct is_unnamed_table : std::false_type {};

template<typename T>
inline constexpr bool is_unnamed_table_v = is_unnamed_table<T>::value;

} // namespace internal

namespace concepts {

// What a table is declared from: its entries, and unnamed tables
// standing for theirs
template<typename T>
concept entry_or_table = internal::is_unnamed_table_v<T> || transition_table_entry<T>;

template<typename T>
concept transition_table = mtl::concepts::typelist<typename T::transitions> &&
                           mtl::concepts::typelist<typename T::states>;

} // namespace concepts

template<concepts::entry_or_table... ENTRY_OR_TABLEs>
struct transition_table;

namespace internal {

// Matched by its arguments: the table itself stays uninstantiated
template<typename... ENTRY_OR_TABLEs>
struct is_unnamed_table<transition_table<ENTRY_OR_TABLEs...>> : std::true_type {};

} // namespace internal

// The table's lookups (transition_table below) as traits: spelled
// without typename/template in dependent contexts
template<concepts::transition_table TABLE, concepts::state FROM, concepts::event EVENT>
using exact_transitions_t = typename TABLE::template exact_transitions<FROM, EVENT>;

template<concepts::transition_table TABLE, concepts::event EVENT>
using wildcard_transitions_t = typename TABLE::template wildcard_transitions<EVENT>;

template<concepts::transition_table TABLE, concepts::state FROM, concepts::event EVENT>
using transitions_for_t = typename TABLE::template transitions_for<FROM, EVENT>;

template<concepts::transition_table TABLE, concepts::state FROM, concepts::event EVENT>
using transition_for_t = typename TABLE::template transition_for<FROM, EVENT>;

namespace internal {

template<concepts::state FROM, concepts::event EVENT>
struct matches {
    template<concepts::transition TRANSITION>
    struct pred : std::bool_constant<std::is_same_v<typename TRANSITION::from, FROM> &&
                                     std::is_same_v<typename TRANSITION::event, EVENT>> {};
};

// The second-stage predicate for lists already grouped by source:
// re-checking FROM there would only add instantiations
template<concepts::event EVENT>
struct matches_event {
    template<concepts::transition TRANSITION>
    struct pred : std::is_same<typename TRANSITION::event, EVENT> {};
};

template<concepts::state STATE>
struct is_any_state : std::is_same<STATE, any_state> {};

// The transitions leaving FROM, in table order, selected in one pass
// over the pack
template<concepts::state FROM, mtl::concepts::typelist TRANSITIONS>
struct grouped_by_from;

template<concepts::state FROM, concepts::transition... TRANSITIONs>
struct grouped_by_from<FROM, mtl::typelist<TRANSITIONs...>>
    : mtl::concat<std::conditional_t<std::is_same_v<typename TRANSITIONs::from, FROM>,
                                     mtl::typelist<TRANSITIONs>, mtl::typelist<>>...> {};

template<concepts::transition TRANSITION>
struct event_of : std::type_identity<typename TRANSITION::event> {};

// All from/to states of a transition list, in order of appearance
template<mtl::concepts::typelist LIST>
struct endpoints;

template<concepts::transition... TRANSITIONs>
struct endpoints<mtl::typelist<TRANSITIONs...>> {
    using type = mtl::typelist<typename TRANSITIONs::from..., typename TRANSITIONs::to...>;
};

template<concepts::transition TRANSITION>
inline constexpr bool has_guard_v = !mtl::empty_v<typename TRANSITION::guards>;

template<concepts::transition TRANSITION>
struct is_guarded : std::bool_constant<has_guard_v<TRANSITION>> {};

template<concepts::transition TRANSITION>
struct is_unguarded : std::bool_constant<!has_guard_v<TRANSITION>> {};

// The most specific guard form wins. A static guard answers itself -
template<concepts::guard GUARD, concepts::state STATE, concepts::event EVENT>
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
template<concepts::guard GUARD, concepts::observer OBJECT, concepts::state STATE,
         concepts::event EVENT>
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
template<concepts::guard GUARD, concepts::state STATE>
struct answering {
    template<concepts::observer OBJECT>
    struct pred : std::bool_constant<concepts::answers_guard_for<OBJECT, GUARD, STATE>> {};
};

// Whether the machine can resolve one part of a row's guard asked from
// FROM: answered by exactly one of the injected OBJECTS, or by a static
// check of the guard's own (a not_<G> part resolves G)
template<mtl::concepts::typelist OBJECTS, concepts::state FROM>
struct part_answered_in {
    template<concepts::guard_part PART, concepts::guard GUARD = guard_of_t<PART>>
    struct pred
        : std::bool_constant<
              mtl::count_if_v<OBJECTS, answering<GUARD, FROM>::template pred> == 1 ||
              (mtl::count_if_v<OBJECTS, answering<GUARD, FROM>::template pred> == 0 &&
               concepts::guard_for<GUARD, FROM>)> {};
};

template<mtl::concepts::typelist OBJECTS, concepts::state FROM>
struct part_answered_once_in {
    template<concepts::guard_part PART>
    struct pred
        : std::bool_constant<
              mtl::count_if_v<OBJECTS, answering<guard_of_t<PART>, FROM>::template pred> <= 1> {};
};

// ... and every part of TRANSITION's guard
template<mtl::concepts::typelist OBJECTS>
struct guard_answered_in {
    template<concepts::transition TRANSITION>
    struct pred
        : std::bool_constant<mtl::all_of_v<
              typename TRANSITION::guards,
              part_answered_in<OBJECTS, typename TRANSITION::from>::template pred>> {};
};

template<mtl::concepts::typelist OBJECTS>
struct guard_answered_once_in {
    template<concepts::transition TRANSITION>
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

template<concepts::transition FIRST, concepts::transition... RESTs>
struct no_shadowed_alternatives<mtl::typelist<FIRST, RESTs...>>
    : std::bool_constant<
          (has_guard_v<FIRST> ||
           mtl::count_if_v<mtl::typelist<RESTs...>,
                           matches<typename FIRST::from,
                                   typename FIRST::event>::template pred> == 0) &&
          no_shadowed_alternatives<mtl::typelist<RESTs...>>::value> {};

// The entries of a table that are not transitions
template<concepts::transition_table_entry ENTRY>
struct is_table_role
    : std::bool_constant<is_initial<ENTRY>::value || is_final_role<ENTRY>::value> {};

// Membership of a list, as a predicate
template<mtl::concepts::typelist LIST>
struct member_of {
    template<typename T>
    struct pred : std::bool_constant<mtl::has_a_v<LIST, T>> {};
};

// A transition leaving one of the states of LIST by name
template<mtl::concepts::typelist LIST>
struct leaving_one_of {
    template<concepts::transition TRANSITION>
    struct pred : std::bool_constant<mtl::has_a_v<LIST, typename TRANSITION::from>> {};
};

// A state may name an event its entry hands to the composite state
// above, when the table is a submachine's:
//   using emits = attempt_failed;
// The parent machine's transitions for it - guards included - decide
// whether the composite state is left or the submachine carries on
template<typename STATE>
concept emitting = requires { typename STATE::emits; };

template<concepts::state STATE>
struct is_emitting : std::bool_constant<emitting<STATE>> {};

template<emitting STATE>
struct emitted_by : std::type_identity<typename STATE::emits> {};

template<emitting STATE>
using emitted_t = typename emitted_by<STATE>::type;

template<concepts::state STATE>
struct owns_submachine : std::bool_constant<requires { typename STATE::submachine; }> {};

// The entries a table is declared from, in order: every unnamed table
// replaced by its entries. A declaration of entries only is the list
// already
template<typename... ENTRY_OR_TABLEs>
struct declared_entries : std::type_identity<mtl::typelist<ENTRY_OR_TABLEs...>> {};

template<concepts::entry_or_table ENTRY>
struct entries_of : std::type_identity<mtl::typelist<ENTRY>> {};

template<typename... ENTRY_OR_TABLEs>
struct entries_of<transition_table<ENTRY_OR_TABLEs...>> : declared_entries<ENTRY_OR_TABLEs...> {};

template<typename... ENTRY_OR_TABLEs>
    requires (is_unnamed_table_v<ENTRY_OR_TABLEs> || ...)
struct declared_entries<ENTRY_OR_TABLEs...>
    : mtl::linearize<mtl::typelist<typename entries_of<ENTRY_OR_TABLEs>::type...>> {};

} // namespace internal

// Transitions plus two kinds of role entries: an optional
// initial<STATE> - without it the first state of the first transition
// is the initial state - and a final<STATE> per state the machine ends
// in. An unnamed table among them contributes its entries in place, so
// tables compose from the transitions they share:
//   using shared_transitions = fsm::transition_table<...>;
//   struct my_table : fsm::transition_table<shared_transitions, ...> {};
// The table a machine runs is a struct of its own, as my_table is
template<concepts::entry_or_table... ENTRY_OR_TABLEs>
struct transition_table {
    // The entries with the roles included: what a machine rebuilds the
    // table from when a feature of its states is disabled
    using entries = typename internal::declared_entries<ENTRY_OR_TABLEs...>::type;

private:
    static_assert(mtl::count_if_v<entries, internal::is_initial> <= 1,
                  "transition_table: at most one initial<STATE> allowed");

    using explicit_initial = internal::find_role_t<entries, internal::is_initial>;

public:
    using transitions = mtl::remove_if_t<entries, internal::is_table_role>;

    // The states the machine ends in: nothing leaves a final state
    using final_states = mtl::unique_t<
        mtl::transform_t<mtl::filter_t<entries, internal::is_final_role>, internal::unwrap>>;

private:
    using endpoints =
        mtl::remove_if_t<mtl::remove_if_t<typename internal::endpoints<transitions>::type,
                                          internal::is_any_state>,
                         internal::is_internal_target>;
    static_assert(std::is_same_v<explicit_initial, mtl::nil_type> ||
                      mtl::has_a_v<endpoints, explicit_initial>,
                  "transition_table: initial<STATE> is not a state of the table");
    static_assert(mtl::all_of_v<final_states, internal::member_of<endpoints>::template pred>,
                  "transition_table: final<STATE> is not a state of the table");
    static_assert(
        mtl::none_of_v<transitions, internal::leaving_one_of<final_states>::template pred>,
        "transition_table: a transition leaves a final state - nothing leaves it");
    static_assert(mtl::none_of_v<final_states, internal::owns_submachine>,
                  "transition_table: a final state owns a submachine");

    // Transitions grouped by their exact source, computed once per
    // FROM: the per-(FROM, EVENT) lookups and the table checks work on
    // the small group instead of the whole table
    template<concepts::state FROM>
    struct from_group : internal::grouped_by_from<FROM, transitions> {};

public:
    // Deduplicated in order of first appearance: front is the initial state
    using states = mtl::unique_t<std::conditional_t<
        std::is_same_v<explicit_initial, mtl::nil_type>,
        endpoints,
        mtl::prepend_t<explicit_initial, endpoints>>>;

private:
    template<concepts::state FROM>
    struct group_unshadowed : internal::no_shadowed_alternatives<typename from_group<FROM>::type> {};
    static_assert(mtl::all_of_v<mtl::prepend_t<any_state, states>, group_unshadowed>,
                  "transition_table: an unguarded (state, event) transition must be "
                  "the last of its alternatives");

    static_assert(!mtl::has_a_v<final_states, mtl::front_or_t<states, mtl::nil_type>>,
                  "transition_table: the initial state is a final state");

public:
    // Every event the table reacts to, deduplicated in table order:
    // the alternatives of fsm::queued's event storage
    using events = mtl::unique_t<mtl::transform_t<transitions, internal::event_of>>;

    // The exact (FROM, EVENT) group and the (any_state, EVENT) wildcard
    // group, each in table order: the machine's shared wildcard path
    // dispatches exact pairs per state and the wildcard group through
    // one body per (event, target)
    template<concepts::state FROM, concepts::event EVENT>
    using exact_transitions = mtl::filter_t<typename from_group<FROM>::type,
                                            internal::matches_event<EVENT>::template pred>;

    template<concepts::event EVENT>
    using wildcard_transitions = exact_transitions<any_state, EVENT>;

    // All alternatives for (FROM, EVENT) in priority order: the exact
    // group, then the wildcard group (dead behind an unguarded exact
    // entry, which always fires). None for a final state: the
    // wildcards do not apply to it
    template<concepts::state FROM, concepts::event EVENT>
    using transitions_for = std::conditional_t<
        mtl::has_a_v<final_states, FROM>, mtl::typelist<>,
        mtl::concat_t<exact_transitions<FROM, EVENT>, wildcard_transitions<EVENT>>>;

    // The first alternative; mtl::nil_type if there is none
    template<concepts::state FROM, concepts::event EVENT>
    using transition_for = mtl::front_or_t<transitions_for<FROM, EVENT>, mtl::nil_type>;
};

namespace internal {

// The table a machine runs for its observers, given as one
// mtl::typelist: TABLE itself while every feature its states declare
// is enabled by one of them (and for mtl::nil_type, which disables
// nothing), else the table rebuilt from the entries with the disabled
// features removed. Lazy: a table with nothing to remove is never
// rebuilt, and keeps its name
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST,
         bool FILTERED = has_disabled_features_v<typename TABLE::entries, OBSERVER_LIST>>
struct enabled_table : std::type_identity<TABLE> {};

template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST>
struct enabled_table<TABLE, OBSERVER_LIST, true>
    : std::type_identity<
          mtl::rebind_t<without_disabled_features_t<typename TABLE::entries, OBSERVER_LIST>,
                        transition_table>> {};

} // namespace internal

template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST = mtl::nil_type>
using enabled_table_t = typename internal::enabled_table<TABLE, OBSERVER_LIST>::type;

namespace internal {

// Whether STATE can reach a wildcard for EVENT: an unguarded own entry
// always fires first and overrides it, and nothing leaves a final state
template<concepts::transition_table TABLE, concepts::state STATE, concepts::event EVENT>
inline constexpr bool wildcard_source_v =
    !mtl::has_a_v<typename TABLE::final_states, STATE> &&
    std::is_same_v<mtl::find_if_t<exact_transitions_t<TABLE, STATE, EVENT>, is_unguarded>,
                   mtl::nil_type>;

// --- hierarchy ----------------------------------------------------------------
// A composite state declares the table of its submachine:
//   using submachine = my_sub_table;
// The machine owning the table constructs a machine of that table when
// the state is entered and destroys it when the state is left
template<typename STATE>
concept composite = concepts::state<STATE> && requires { typename STATE::submachine; } &&
                    concepts::transition_table<typename STATE::submachine>;

template<concepts::state STATE>
struct submachine : std::type_identity<mtl::nil_type> {};

template<composite STATE>
struct submachine<STATE> : std::type_identity<typename STATE::submachine> {};

template<concepts::state STATE>
using submachine_t = typename submachine<STATE>::type;

template<concepts::state STATE>
struct is_composite : std::bool_constant<composite<STATE>> {};

// Every context type the states of TABLE declare, deduplicated, in
// order of first appearance
template<concepts::transition_table TABLE>
using table_contexts_t =
    mtl::unique_t<mtl::linearize_t<mtl::transform_t<typename TABLE::states, contexts_of>>>;

// A composite state names the contexts of its own machine that its
// submachine inherits:
//   using parent_contexts = fsm::contexts<line_status>;
// A sub-state declaring an inherited type binds to the parent
// machine's instance instead of a fresh one - the same lifetime as the
// parent's; every other context type of the sub-table is the child's
// own, fresh on each entry of the composite. An inherited type the
// submachine does not use itself may be inherited further down through
// a composite of the sub-table
template<typename STATE>
concept declares_parent_contexts =
    concepts::state<STATE> && requires { typename STATE::parent_contexts; } &&
    mtl::concepts::typelist<typename STATE::parent_contexts>;

template<concepts::state STATE>
struct parent_contexts_of : std::type_identity<mtl::typelist<>> {};

template<declares_parent_contexts STATE>
struct parent_contexts_of<STATE> : std::type_identity<typename STATE::parent_contexts> {};

template<concepts::state STATE>
using parent_contexts_t = typename parent_contexts_of<STATE>::type;

// The checks on a parent_contexts declaration, each a trait so a
// failing one can be asked per state: only a composite has a child to
// inherit; the child inherits what the machine holds (CONTEXTS: the
// machine's own and inherited contexts); and some state of the
// submachine declares every inherited type - one nobody uses is a dead
// declaration
template<concepts::state STATE>
struct parent_contexts_on_composite
    : std::bool_constant<!declares_parent_contexts<STATE> || composite<STATE>> {};

template<mtl::concepts::typelist CONTEXTS>
struct parent_contexts_held_in {
    template<concepts::state STATE>
    struct pred : std::bool_constant<mtl::all_of_v<parent_contexts_t<STATE>,
                                                   member_of<CONTEXTS>::template pred>> {};
};

// The table a parent machine builds its child machine from: the
// sub-table itself plus the child's nesting depth (the root is 0),
// which the timer observers use to pick their timer slot, and the
// contexts the child inherits. Every user-facing alias
// (StateMachine::table, trace names, validate) sees the plain table
template<concepts::transition_table TABLE, std::size_t DEPTH,
         mtl::concepts::typelist INHERITED = mtl::typelist<>>
struct nested : TABLE {
    static constexpr std::size_t depth = DEPTH;
    using inherited_contexts            = INHERITED;
};

template<concepts::transition_table TABLE>
struct plain_table : std::type_identity<TABLE> {};

template<concepts::transition_table TABLE, std::size_t DEPTH, mtl::concepts::typelist INHERITED>
struct plain_table<nested<TABLE, DEPTH, INHERITED>> : std::type_identity<TABLE> {};

template<concepts::transition_table TABLE>
using plain_table_t = typename plain_table<TABLE>::type;

template<concepts::transition_table TABLE>
inline constexpr std::size_t table_depth_v = 0;

template<concepts::transition_table TABLE, std::size_t DEPTH, mtl::concepts::typelist INHERITED>
inline constexpr std::size_t table_depth_v<nested<TABLE, DEPTH, INHERITED>> = DEPTH;

// The contexts a machine inherits from its parent: none for a root
template<concepts::transition_table TABLE>
struct inherited_contexts : std::type_identity<mtl::typelist<>> {};

template<concepts::transition_table TABLE, std::size_t DEPTH, mtl::concepts::typelist INHERITED>
struct inherited_contexts<nested<TABLE, DEPTH, INHERITED>> : std::type_identity<INHERITED> {};

template<concepts::transition_table TABLE>
using inherited_contexts_t = typename inherited_contexts<TABLE>::type;

// The number of machine levels a state's submachine spans: none for a
// plain state, the levels of its submachine table for a composite one
template<concepts::transition_table TABLE>
struct levels;

template<concepts::state STATE>
struct submachine_levels : std::integral_constant<std::size_t, 0> {};

template<composite STATE>
struct submachine_levels<STATE> : levels<submachine_t<STATE>> {};

template<concepts::state... STATEs>
constexpr std::size_t deepestSubmachine(mtl::typelist<STATEs...>)
{
    return std::max({std::size_t{0}, submachine_levels<STATEs>::value...});
}

template<concepts::transition_table TABLE>
struct levels
    : std::integral_constant<std::size_t, 1 + deepestSubmachine(typename TABLE::states{})> {};

// TABLE and every submachine table nested in it, TABLE first, each as
// the observers enable it (OBSERVER_LIST: fsm::observers<...>, or
// mtl::nil_type for every state in view)
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST>
struct nested_tables;

// The tables of a state's submachine - the submachine's own and those
// of the composites inside it; none for a plain state
template<concepts::observer_list OBSERVER_LIST>
struct submachine_tables_for {
    template<concepts::state STATE>
    struct of : std::type_identity<mtl::typelist<>> {};

    template<composite STATE>
    struct of<STATE>
        : nested_tables<enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>, OBSERVER_LIST> {};
};

template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST>
struct nested_tables
    : std::type_identity<mtl::unique_t<mtl::prepend_t<
          TABLE, mtl::linearize_t<mtl::transform_t<
                     typename TABLE::states, submachine_tables_for<OBSERVER_LIST>::template of>>>>> {
};

template<concepts::transition_table TABLE>
struct states_of : std::type_identity<typename TABLE::states> {};

template<concepts::transition_table TABLE>
struct events_of_table : std::type_identity<typename TABLE::events> {};

} // namespace internal

// How many machine levels a table spans: 1 for a flat table, one more
// per level of composite states below it. A machine at nesting depth d
// (root 0) is level d; fsm::timed<TIMER, LEVELS> needs one timer per
// level
template<concepts::transition_table TABLE>
inline constexpr std::size_t levels_v = internal::levels<TABLE>::value;

// TABLE followed by every sub-table it nests, recursively, each once:
// what a table-wide proof walks when it has to cover the whole machine.
// With the machine's observers as an mtl::typelist, every level is the
// table that machine runs (enabled_table_t); without, every state is
// in view
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST = mtl::nil_type>
using nested_tables_t =
    typename internal::nested_tables<enabled_table_t<TABLE, OBSERVER_LIST>, OBSERVER_LIST>::type;

// The states of TABLE and of every submachine table nested in it, in
// that order
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST = mtl::nil_type>
using all_states_t = mtl::linearize_t<
    mtl::transform_t<nested_tables_t<TABLE, OBSERVER_LIST>, internal::states_of>>;

// Every event any level of the machine reacts to, each once: the
// alternatives of a queued machine's event storage
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST = mtl::nil_type>
using nested_events_t = mtl::unique_t<
    mtl::linearize_t<mtl::transform_t<nested_tables_t<TABLE, OBSERVER_LIST>, internal::events_of_table>>>;

namespace internal {

// The contexts declared by the states of a composite's submachine, at
// any level: what its child can usefully inherit
template<concepts::state STATE>
struct submachine_contexts : std::type_identity<mtl::typelist<>> {};

template<composite STATE>
struct submachine_contexts<STATE>
    : std::type_identity<mtl::unique_t<mtl::linearize_t<
          mtl::transform_t<all_states_t<submachine_t<STATE>>, contexts_of>>>> {};

template<concepts::state STATE>
struct parent_contexts_declared_in_submachine
    : std::bool_constant<
          mtl::all_of_v<parent_contexts_t<STATE>,
                        member_of<typename submachine_contexts<STATE>::type>::template pred>> {};

// The states of the submachine a composite STATE owns that emit an
// event, as the observers enable that submachine
template<composite STATE, concepts::observer_list OBSERVER_LIST>
using submachine_emitting_states_t = mtl::filter_t<
    typename enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>::states, is_emitting>;

// What a composite state of TABLE owes its submachine: a transition
// for every event a state of it emits - own or through a wildcard,
// guarded or not. An event nobody could take is a dead declaration
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST>
struct emitted_events_taken_in {
    template<composite STATE>
    struct taken_by {
        template<emitting EMITTING>
        struct pred
            : std::bool_constant<
                  !mtl::empty_v<transitions_for_t<TABLE, STATE, emitted_t<EMITTING>>>> {};
    };

    template<composite STATE>
    struct pred
        : std::bool_constant<mtl::all_of_v<submachine_emitting_states_t<STATE, OBSERVER_LIST>,
                                           taken_by<STATE>::template pred>> {};
};

// A submachine is entered at its initial state before anyone could
// take an event: that state does not emit
template<concepts::observer_list OBSERVER_LIST>
struct submachine_starts_silent {
    template<composite STATE>
    struct pred
        : std::bool_constant<!emitting<mtl::front_t<
              typename enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>::states>>> {};
};

} // namespace internal

// The states TABLE ends in, as its final<> entries name them
template<concepts::transition_table TABLE>
using final_states_t = typename TABLE::final_states;

// The events the states of TABLE and of every submachine nested in it
// emit, each once. An emitted event is a submachine's own word to the
// composite state above it: it reaches the parent machine's
// transitions for that state and nothing else - it is never processed
// from outside
template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST = mtl::nil_type>
using emitted_events_t = mtl::unique_t<mtl::transform_t<
    mtl::filter_t<all_states_t<TABLE, OBSERVER_LIST>, internal::is_emitting>,
    internal::emitted_by>>;

} // namespace fsm
