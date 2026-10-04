/*
 * fsm: the guards of a transition - who answers them, and whether they
 * hold for an event
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/InjectedObservers.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

namespace fsm::internal {

// The injected objects answering GUARD asked from STATE
template<concepts::guard GUARD, concepts::state STATE>
struct answering {
    template<concepts::observer OBJECT>
    struct pred : std::bool_constant<concepts::answers_guard_for<OBJECT, GUARD, STATE>> {};
};

// Whether the machine can resolve one part of a transition's guard
// asked from FROM: answered by exactly one of the injected OBJECTS, or
// by a static check of the guard's own (a not_<G> part resolves G)
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

// The guards of the transitions of one machine. A guard is answered by
// the one injected observer with a check for its tag, else by its own
// static check; either way the most specific form wins. The class
// holds nothing: the machine hands in its observers
template<concepts::observer... OBSERVERs>
class TransitionGuards {
    using observer_list      = mtl::typelist<OBSERVERs...>;
    using injected_observers = InjectedObservers<OBSERVERs...>;

public:
    // --- what the table must satisfy ----------------------------------------

    // By one observer, or by the guard's own static check
    template<concepts::transition_table TRANSITIONS>
    static constexpr bool every_guard_answered =
        mtl::all_of_v<typename TRANSITIONS::transitions,
                      guard_answered_in<observer_list>::template pred>;

    template<concepts::transition_table TRANSITIONS>
    static constexpr bool no_guard_answered_by_two_observers =
        mtl::all_of_v<typename TRANSITIONS::transitions,
                      guard_answered_once_in<observer_list>::template pred>;

    // --- an event arrives ---------------------------------------------------

    // Does every part of TRANSITION's guard hold for the event arriving
    // in its from-state? An unguarded transition is always allowed
    template<concepts::transition TRANSITION, concepts::state STATE, concepts::event EVENT>
    static bool allow(injected_observers& observers, STATE const& state, EVENT const& event)
    {
        return everyPartHolds<typename TRANSITION::guards>(observers, state, event);
    }

private:
    template<concepts::guard GUARD, concepts::state STATE>
    static constexpr bool any_observer_answers =
        (concepts::answers_guard_for<OBSERVERs, GUARD, STATE> || ...);

    // One body per guard list: transitions with the same guards share it
    template<mtl::concepts::typelist GUARDS, concepts::state STATE, concepts::event EVENT>
    static bool everyPartHolds([[maybe_unused]] injected_observers& observers,
                               [[maybe_unused]] STATE const& state,
                               [[maybe_unused]] EVENT const& event)
    {
        return [&]<concepts::guard_part... PARTs>(mtl::typelist<PARTs...>) {
            return (partHolds<PARTs>(observers, state, event) && ...);
        }(GUARDS{});
    }

    template<concepts::guard_part PART, concepts::state STATE, concepts::event EVENT>
    static bool partHolds(injected_observers& observers, STATE const& state, EVENT const& event)
    {
        if constexpr (is_negated_v<PART>) {
            return !answerTo<guard_of_t<PART>>(observers, state, event);
        } else {
            return answerTo<PART>(observers, state, event);
        }
    }

    // The observer answering GUARD decides, else the guard's own static
    // check
    template<concepts::guard GUARD, concepts::state STATE, concepts::event EVENT>
    static bool answerTo([[maybe_unused]] injected_observers& observers, STATE const& state,
                         EVENT const& event)
    {
        if constexpr (any_observer_answers<GUARD, STATE>) {
            using answerer = mtl::find_if_t<observer_list, answering<GUARD, STATE>::template pred>;
            return askObserver<GUARD>(observers.template observer<answerer>(), state, event);
        } else {
            return checkStatic<GUARD>(state, event);
        }
    }

    // The most specific form wins: check(GUARD, state, event),
    // check(GUARD, state), check(GUARD) - the tag selects the overload
    template<concepts::guard GUARD, concepts::observer OBSERVER, concepts::state STATE,
             concepts::event EVENT>
    static bool askObserver(OBSERVER& observer, [[maybe_unused]] STATE const& state,
                            [[maybe_unused]] EVENT const& event)
    {
        if constexpr (concepts::answers_event_guard<OBSERVER, GUARD, STATE, EVENT>) {
            return observer.check(GUARD{}, state, event);
        } else if constexpr (concepts::answers_state_guard<OBSERVER, GUARD, STATE>) {
            return observer.check(GUARD{}, state);
        } else {
            return observer.check(GUARD{});
        }
    }

    // ... and of the guard's own: check(state, event), check(state), check()
    template<concepts::guard GUARD, concepts::state STATE, concepts::event EVENT>
    static bool checkStatic([[maybe_unused]] STATE const& state,
                            [[maybe_unused]] EVENT const& event)
    {
        if constexpr (concepts::event_guard_for<GUARD, STATE, EVENT>) {
            return GUARD::check(state, event);
        } else if constexpr (concepts::state_guard_for<GUARD, STATE>) {
            return GUARD::check(state);
        } else {
            return GUARD::check();
        }
    }
};

} // namespace fsm::internal
