/*
 * fsm: the observer hooks - detection of the two forms of each hook,
 * their delivery, and the ObserverGroup composite
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Table.hpp>
#include <mtl/Typelist.hpp>

#include <cstddef>
#include <tuple>
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

// Composite observer: forwards every hook to caller-owned member
// observers in member order. fsm::observing allows one static and one
// nonstatic observation per observer; a group bundles several such
// observers so they can be injected into the machine as one, letting a
// library predefine a cohesive set behind a single reference
template<concepts::observer... OBSERVERs>
class ObserverGroup {
public:
    explicit ObserverGroup(OBSERVERs&... members) : members_(members...) {}

    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert((internal::validated<OBSERVERs, TABLE>() && ...));
    }

    // The From forms forward the source to each member's preferred form
    // and exist for the edges some member has a hook for; a plain form
    // exists only when every member has it, so a member asking for the
    // source is never left without one
    template<concepts::state FROM, concepts::state TO, typename MACHINE>
        requires(internal::has_exit_hook<OBSERVERs, FROM, TO, MACHINE> || ...)
    void onExitFrom(MACHINE& machine)
    {
        this->template exitMembers<FROM, TO>(
            machine, internal::hooked_indices_t<
                         internal::has_exit_hook<OBSERVERs, FROM, TO, MACHINE>...>{});
    }

    template<concepts::state TO, typename MACHINE>
        requires(internal::has_exit<OBSERVERs, TO, MACHINE> && ...)
    void onExit(MACHINE& machine)
    {
        this->forEachMember([&machine](auto& member) { member.template onExit<TO>(machine); });
    }

    template<concepts::state FROM, concepts::state TO, typename MACHINE>
        requires(internal::has_enter_hook<OBSERVERs, FROM, TO, MACHINE> || ...)
    void onEnterFrom(MACHINE& machine)
    {
        this->template enterMembers<FROM, TO>(
            machine, internal::hooked_indices_t<
                         internal::has_enter_hook<OBSERVERs, FROM, TO, MACHINE>...>{});
    }

    template<concepts::state TO, typename MACHINE>
        requires(internal::has_enter<OBSERVERs, TO, MACHINE> && ...)
    void onEnter(MACHINE& machine)
    {
        this->forEachMember([&machine](auto& member) { member.template onEnter<TO>(machine); });
    }

    template<concepts::state FROM, concepts::event EVENT, concepts::state TO, typename MACHINE>
        requires(internal::has_transition_hook<OBSERVERs, FROM, EVENT, TO, MACHINE> || ...)
    void onTransitionFrom(MACHINE& machine)
    {
        this->template transitionMembers<FROM, EVENT, TO>(
            machine, internal::hooked_indices_t<internal::has_transition_hook<
                         OBSERVERs, FROM, EVENT, TO, MACHINE>...>{});
    }

    template<concepts::event EVENT, concepts::state TO, typename MACHINE>
        requires(internal::has_transition<OBSERVERs, EVENT, TO, MACHINE> && ...)
    void onTransition(MACHINE& machine)
    {
        this->forEachMember(
            [&machine](auto& member) { member.template onTransition<EVENT, TO>(machine); });
    }

private:
    template<concepts::state FROM, concepts::state TO, typename MACHINE, std::size_t... INDEXs>
    void exitMembers(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (internal::exitHook<FROM, TO>(std::get<INDEXs>(members_), machine), ...);
    }

    template<concepts::state FROM, concepts::state TO, typename MACHINE, std::size_t... INDEXs>
    void enterMembers(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (internal::enterHook<FROM, TO>(std::get<INDEXs>(members_), machine), ...);
    }

    template<concepts::state FROM, concepts::event EVENT, concepts::state TO, typename MACHINE,
             std::size_t... INDEXs>
    void transitionMembers(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (internal::transitionHook<FROM, EVENT, TO>(std::get<INDEXs>(members_), machine), ...);
    }

    template<typename F>
    void forEachMember(F&& f)
    {
        this->forEachMember(f, std::index_sequence_for<OBSERVERs...>{});
    }

    template<typename F, std::size_t... INDEXs>
    void forEachMember(F& f, std::index_sequence<INDEXs...>)
    {
        (f(std::get<INDEXs>(members_)), ...);
    }

    std::tuple<OBSERVERs&...> members_;
};

} // namespace fsm
