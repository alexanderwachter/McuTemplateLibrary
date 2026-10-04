/*
 * fsm: the observer hooks - detection of the two forms of each hook
 * and their delivery to one observer
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Table.hpp>
#include <mtl/Typelist.hpp>

#include <cstddef>
#include <type_traits>
#include <utility>

namespace fsm {

namespace internal {

template<concepts::observer OBSERVER, concepts::transition_table TABLE>
constexpr bool validated()
{
    if constexpr (requires { OBSERVER::template validate<TABLE>(); }) {
        OBSERVER::template validate<TABLE>();
    }
    return true;
}

// The two forms of each hook an observer may define: with the source
// state (From) or without
template<typename OBSERVER, typename FROM, typename TO, typename MACHINE>
concept has_exit_from = requires(OBSERVER observer, MACHINE& machine) {
    observer.template onExitFrom<FROM, TO>(machine);
};

template<typename OBSERVER, typename TO, typename MACHINE>
concept has_exit = requires(OBSERVER observer, MACHINE& machine) {
    observer.template onExit<TO>(machine);
};

template<typename OBSERVER, typename FROM, typename TO, typename MACHINE>
concept has_enter_from = requires(OBSERVER observer, MACHINE& machine) {
    observer.template onEnterFrom<FROM, TO>(machine);
};

template<typename OBSERVER, typename TO, typename MACHINE>
concept has_enter = requires(OBSERVER observer, MACHINE& machine) {
    observer.template onEnter<TO>(machine);
};

template<typename OBSERVER, typename FROM, typename EVENT, typename TO, typename MACHINE>
concept has_transition_from = requires(OBSERVER observer, MACHINE& machine) {
    observer.template onTransitionFrom<FROM, EVENT, TO>(machine);
};

template<typename OBSERVER, typename EVENT, typename TO, typename MACHINE>
concept has_transition = requires(OBSERVER observer, MACHINE& machine) {
    observer.template onTransition<EVENT, TO>(machine);
};

// Whether an observer has a hook for the edge, in either form
template<typename OBSERVER, typename FROM, typename TO, typename MACHINE>
concept has_exit_hook =
    has_exit_from<OBSERVER, FROM, TO, MACHINE> || has_exit<OBSERVER, FROM, MACHINE>;

template<typename OBSERVER, typename FROM, typename TO, typename MACHINE>
concept has_enter_hook =
    has_enter_from<OBSERVER, FROM, TO, MACHINE> || has_enter<OBSERVER, TO, MACHINE>;

template<typename OBSERVER, typename FROM, typename EVENT, typename TO, typename MACHINE>
concept has_transition_hook = has_transition_from<OBSERVER, FROM, EVENT, TO, MACHINE> ||
                              has_transition<OBSERVER, EVENT, TO, MACHINE>;

// The positions of the observers a hook is delivered to: those where
// HOOKED is true, as a std::index_sequence
template<bool... HOOKED>
struct hooked_indices;

template<bool... HOOKED>
using hooked_indices_t = typename hooked_indices<HOOKED...>::type;

template<mtl::concepts::typelist POSITIONS>
struct as_index_sequence;

template<std::size_t... POSITIONs>
struct as_index_sequence<mtl::typelist<std::integral_constant<std::size_t, POSITIONs>...>>
    : std::type_identity<std::index_sequence<POSITIONs...>> {};

template<typename INDEXES, bool... HOOKED>
struct hooked_positions;

template<std::size_t... INDEXs, bool... HOOKED>
struct hooked_positions<std::index_sequence<INDEXs...>, HOOKED...>
    : as_index_sequence<mtl::concat_t<
          std::conditional_t<HOOKED, mtl::typelist<std::integral_constant<std::size_t, INDEXs>>,
                             mtl::typelist<>>...>> {};

template<bool... HOOKED>
struct hooked_indices
    : hooked_positions<std::make_index_sequence<sizeof...(HOOKED)>, HOOKED...> {};

// An edge with a known source: the edge form when defined, else the
// one-state form - of the state left for the exit, of the state
// entered for the entry
template<concepts::state FROM, concepts::state TO, concepts::observer OBSERVER, typename MACHINE>
void exitHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (has_exit_from<OBSERVER, FROM, TO, MACHINE>) {
        observer.template onExitFrom<FROM, TO>(machine);
    } else if constexpr (has_exit<OBSERVER, FROM, MACHINE>) {
        observer.template onExit<FROM>(machine);
    }
}

template<concepts::state FROM, concepts::state TO, concepts::observer OBSERVER, typename MACHINE>
void enterHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (has_enter_from<OBSERVER, FROM, TO, MACHINE>) {
        observer.template onEnterFrom<FROM, TO>(machine);
    } else if constexpr (has_enter<OBSERVER, TO, MACHINE>) {
        observer.template onEnter<TO>(machine);
    }
}

template<concepts::state FROM, concepts::event EVENT, concepts::state TO,
         concepts::observer OBSERVER, typename MACHINE>
void transitionHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (has_transition_from<OBSERVER, FROM, EVENT, TO, MACHINE>) {
        observer.template onTransitionFrom<FROM, EVENT, TO>(machine);
    } else if constexpr (has_transition<OBSERVER, EVENT, TO, MACHINE>) {
        observer.template onTransition<EVENT, TO>(machine);
    }
}

} // namespace internal

} // namespace fsm
