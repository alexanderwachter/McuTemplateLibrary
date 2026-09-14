/*
 * fsm: the observer hooks - detection of the two forms of each hook,
 * their delivery, and the observer_group composite
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <tuple>
#include <utility>

namespace fsm {

namespace internal {

template<typename OBSERVER, typename TABLE>
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

// An edge with a known source: the edge form when defined, else the
// one-state form - of the state left for the exit, of the state
// entered for the entry
template<typename FROM, typename TO, typename OBSERVER, typename MACHINE>
void exitHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (has_exit_from<OBSERVER, FROM, TO, MACHINE>) {
        observer.template onExitFrom<FROM, TO>(machine);
    } else if constexpr (has_exit<OBSERVER, FROM, MACHINE>) {
        observer.template onExit<FROM>(machine);
    }
}

template<typename FROM, typename TO, typename OBSERVER, typename MACHINE>
void enterHook(OBSERVER& observer, MACHINE& machine)
{
    if constexpr (has_enter_from<OBSERVER, FROM, TO, MACHINE>) {
        observer.template onEnterFrom<FROM, TO>(machine);
    } else if constexpr (has_enter<OBSERVER, TO, MACHINE>) {
        observer.template onEnter<TO>(machine);
    }
}

template<typename FROM, typename EVENT, typename TO, typename OBSERVER, typename MACHINE>
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
template<typename... OBSERVERs>
class observer_group {
public:
    explicit observer_group(OBSERVERs&... members) : members_(members...) {}

    template<typename TABLE>
    static constexpr void validate()
    {
        static_assert((internal::validated<OBSERVERs, TABLE>() && ...));
    }

    // The From forms forward the source to each member's preferred form;
    // a plain form exists only when every member has it, so a member
    // asking for the source is never left without one
    template<typename FROM, typename TO, typename MACHINE>
    void onExitFrom(MACHINE& machine)
    {
        std::apply([&machine](auto&... member) { (internal::exitHook<FROM, TO>(member, machine), ...); },
                   members_);
    }

    template<typename TO, typename MACHINE>
        requires(internal::has_exit<OBSERVERs, TO, MACHINE> && ...)
    void onExit(MACHINE& machine)
    {
        std::apply([&machine](auto&... member) { (member.template onExit<TO>(machine), ...); },
                   members_);
    }

    template<typename FROM, typename TO, typename MACHINE>
    void onEnterFrom(MACHINE& machine)
    {
        std::apply([&machine](auto&... member) { (internal::enterHook<FROM, TO>(member, machine), ...); },
                   members_);
    }

    template<typename TO, typename MACHINE>
        requires(internal::has_enter<OBSERVERs, TO, MACHINE> && ...)
    void onEnter(MACHINE& machine)
    {
        std::apply([&machine](auto&... member) { (member.template onEnter<TO>(machine), ...); },
                   members_);
    }

    template<typename FROM, typename EVENT, typename TO, typename MACHINE>
    void onTransitionFrom(MACHINE& machine)
    {
        std::apply(
            [&machine](auto&... member) {
                (internal::transitionHook<FROM, EVENT, TO>(member, machine), ...);
            },
            members_);
    }

    template<typename EVENT, typename TO, typename MACHINE>
        requires(internal::has_transition<OBSERVERs, EVENT, TO, MACHINE> && ...)
    void onTransition(MACHINE& machine)
    {
        std::apply(
            [&machine](auto&... member) {
                (member.template onTransition<EVENT, TO>(machine), ...);
            },
            members_);
    }

private:
    std::tuple<OBSERVERs&...> members_;
};

} // namespace fsm
