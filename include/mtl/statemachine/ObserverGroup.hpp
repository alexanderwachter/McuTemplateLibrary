/*
 * fsm: ObserverGroup - several observers injected into a machine as one
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/internal/ObserverHooks.hpp>
#include <mtl/statemachine/Table.hpp>

#include <cstddef>
#include <tuple>
#include <utility>

namespace fsm {

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

    // The group has a hook exactly where a member has it, in that form.
    // A From form exists where some member has the From form and
    // forwards the source to every member with a hook for the edge, in
    // the member's preferred form; a plain form exists where some
    // member has the plain form and goes to those members. The machine
    // sees the edge through the group only where a member asks for it
    template<concepts::state FROM, concepts::state TO, typename MACHINE>
        requires(internal::has_exit_from<OBSERVERs, FROM, TO, MACHINE> || ...)
    void onExitFrom(MACHINE& machine)
    {
        exitMembers<FROM, TO>(
            machine, internal::hooked_indices_t<
                         internal::has_exit_hook<OBSERVERs, FROM, TO, MACHINE>...>{});
    }

    template<concepts::state TO, typename MACHINE>
        requires(internal::has_exit<OBSERVERs, TO, MACHINE> || ...)
    void onExit(MACHINE& machine)
    {
        exitMembersOfState<TO>(
            machine, internal::hooked_indices_t<internal::has_exit<OBSERVERs, TO, MACHINE>...>{});
    }

    template<concepts::state FROM, concepts::state TO, typename MACHINE>
        requires(internal::has_enter_from<OBSERVERs, FROM, TO, MACHINE> || ...)
    void onEnterFrom(MACHINE& machine)
    {
        enterMembers<FROM, TO>(
            machine, internal::hooked_indices_t<
                         internal::has_enter_hook<OBSERVERs, FROM, TO, MACHINE>...>{});
    }

    template<concepts::state TO, typename MACHINE>
        requires(internal::has_enter<OBSERVERs, TO, MACHINE> || ...)
    void onEnter(MACHINE& machine)
    {
        enterMembersOfState<TO>(
            machine, internal::hooked_indices_t<internal::has_enter<OBSERVERs, TO, MACHINE>...>{});
    }

    template<concepts::state FROM, concepts::event EVENT, concepts::state TO, typename MACHINE>
        requires(internal::has_transition_from<OBSERVERs, FROM, EVENT, TO, MACHINE> || ...)
    void onTransitionFrom(MACHINE& machine)
    {
        transitionMembers<FROM, EVENT, TO>(
            machine, internal::hooked_indices_t<internal::has_transition_hook<
                         OBSERVERs, FROM, EVENT, TO, MACHINE>...>{});
    }

    template<concepts::event EVENT, concepts::state TO, typename MACHINE>
        requires(internal::has_transition<OBSERVERs, EVENT, TO, MACHINE> || ...)
    void onTransition(MACHINE& machine)
    {
        transitionMembersOfState<EVENT, TO>(
            machine,
            internal::hooked_indices_t<internal::has_transition<OBSERVERs, EVENT, TO, MACHINE>...>{});
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

    // INDEXs: the members with the plain form
    template<concepts::state TO, typename MACHINE, std::size_t... INDEXs>
    void exitMembersOfState(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (std::get<INDEXs>(members_).template onExit<TO>(machine), ...);
    }

    template<concepts::state TO, typename MACHINE, std::size_t... INDEXs>
    void enterMembersOfState(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (std::get<INDEXs>(members_).template onEnter<TO>(machine), ...);
    }

    template<concepts::event EVENT, concepts::state TO, typename MACHINE, std::size_t... INDEXs>
    void transitionMembersOfState(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (std::get<INDEXs>(members_).template onTransition<EVENT, TO>(machine), ...);
    }

    std::tuple<OBSERVERs&...> members_;
};

} // namespace fsm
