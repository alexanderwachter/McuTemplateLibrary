/*
 * fsm: ObserverGroup - several observers injected into a machine as one
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/ObserverHooks.hpp>
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

    // The From forms forward the source to each member's preferred form
    // and exist for the edges some member has a hook for; a plain form
    // exists only when every member has it, so a member asking for the
    // source is never left without one
    template<concepts::state FROM, concepts::state TO, typename MACHINE>
        requires(internal::has_exit_hook<OBSERVERs, FROM, TO, MACHINE> || ...)
    void onExitFrom(MACHINE& machine)
    {
        exitMembers<FROM, TO>(
            machine, internal::hooked_indices_t<
                         internal::has_exit_hook<OBSERVERs, FROM, TO, MACHINE>...>{});
    }

    template<concepts::state TO, typename MACHINE>
        requires(internal::has_exit<OBSERVERs, TO, MACHINE> && ...)
    void onExit(MACHINE& machine)
    {
        forEachMember([&machine](auto& member) { member.template onExit<TO>(machine); });
    }

    template<concepts::state FROM, concepts::state TO, typename MACHINE>
        requires(internal::has_enter_hook<OBSERVERs, FROM, TO, MACHINE> || ...)
    void onEnterFrom(MACHINE& machine)
    {
        enterMembers<FROM, TO>(
            machine, internal::hooked_indices_t<
                         internal::has_enter_hook<OBSERVERs, FROM, TO, MACHINE>...>{});
    }

    template<concepts::state TO, typename MACHINE>
        requires(internal::has_enter<OBSERVERs, TO, MACHINE> && ...)
    void onEnter(MACHINE& machine)
    {
        forEachMember([&machine](auto& member) { member.template onEnter<TO>(machine); });
    }

    template<concepts::state FROM, concepts::event EVENT, concepts::state TO, typename MACHINE>
        requires(internal::has_transition_hook<OBSERVERs, FROM, EVENT, TO, MACHINE> || ...)
    void onTransitionFrom(MACHINE& machine)
    {
        transitionMembers<FROM, EVENT, TO>(
            machine, internal::hooked_indices_t<internal::has_transition_hook<
                         OBSERVERs, FROM, EVENT, TO, MACHINE>...>{});
    }

    template<concepts::event EVENT, concepts::state TO, typename MACHINE>
        requires(internal::has_transition<OBSERVERs, EVENT, TO, MACHINE> && ...)
    void onTransition(MACHINE& machine)
    {
        forEachMember(
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
        forEachMember(f, std::index_sequence_for<OBSERVERs...>{});
    }

    template<typename F, std::size_t... INDEXs>
    void forEachMember(F& f, std::index_sequence<INDEXs...>)
    {
        (f(std::get<INDEXs>(members_)), ...);
    }

    std::tuple<OBSERVERs&...> members_;
};

} // namespace fsm
