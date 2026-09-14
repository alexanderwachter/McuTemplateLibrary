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

// All from/to states of a transition list, in order of appearance
template<mtl::concepts::typelist LIST>
struct endpoints;

template<typename... TRANSITIONs>
struct endpoints<mtl::typelist<TRANSITIONs...>> {
    using type = mtl::typelist<typename TRANSITIONs::from..., typename TRANSITIONs::to...>;
};

template<typename TRANSITION>
inline constexpr bool has_guard_v = !std::is_same_v<typename TRANSITION::guard, mtl::nil_type>;

template<typename TRANSITION>
struct is_guarded : std::bool_constant<has_guard_v<TRANSITION>> {};

template<typename TRANSITION>
struct is_unguarded : std::bool_constant<!has_guard_v<TRANSITION>> {};

// The most specific guard form wins. Callability was validated by the
// transition's guard_for static_assert
template<typename GUARD, concepts::state STATE, typename EVENT>
bool checkGuard([[maybe_unused]] STATE const& state, [[maybe_unused]] EVENT const& event)
{
    if constexpr (concepts::event_guard_for<GUARD, STATE, EVENT>) {
        return GUARD::check(state, event);
    } else if constexpr (concepts::state_guard_for<GUARD, STATE>) {
        return GUARD::check(state);
    } else {
        return GUARD::check();
    }
}

// True when TRANSITION may fire from the given state instance
template<typename TRANSITION, typename STATE, typename EVENT>
bool allowed(STATE const& state, EVENT const& event)
{
    if constexpr (has_guard_v<TRANSITION>) {
        return checkGuard<typename TRANSITION::guard>(state, event);
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

} // namespace fsm
