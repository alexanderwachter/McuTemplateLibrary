/*
 * fsm: the observers injected into a machine - their references, the
 * delivery of the hooks, and what the observers answer together
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/internal/ObserverHooks.hpp>
#include <mtl/statemachine/Feature.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

namespace fsm::internal {

// The states a wildcard for EVENT can leave from which OBSERVER has the
// edge form of the hook
template<concepts::observer OBSERVER, concepts::transition_table TRANSITIONS,
         concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE>
struct is_wildcard_source_with_enter_from {
    template<concepts::state STATE>
    struct pred : std::bool_constant<wildcard_source_v<TRANSITIONS, STATE, EVENT> &&
                                     has_enter_from<OBSERVER, STATE, NEW_STATE, MACHINE>> {};
};

template<concepts::observer OBSERVER, concepts::transition_table TRANSITIONS,
         concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE>
using wildcard_sources_with_enter_from =
    mtl::filter_t<typename TRANSITIONS::states,
                  is_wildcard_source_with_enter_from<OBSERVER, TRANSITIONS, EVENT, NEW_STATE,
                                                     MACHINE>::template pred>;

template<concepts::observer OBSERVER, concepts::transition_table TRANSITIONS,
         concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE>
struct is_wildcard_source_with_transition_from {
    template<concepts::state STATE>
    struct pred
        : std::bool_constant<wildcard_source_v<TRANSITIONS, STATE, EVENT> &&
                             has_transition_from<OBSERVER, STATE, EVENT, NEW_STATE, MACHINE>> {};
};

template<concepts::observer OBSERVER, concepts::transition_table TRANSITIONS,
         concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE>
using wildcard_sources_with_transition_from =
    mtl::filter_t<typename TRANSITIONS::states,
                  is_wildcard_source_with_transition_from<OBSERVER, TRANSITIONS, EVENT, NEW_STATE,
                                                          MACHINE>::template pred>;

template<typename OBSERVER, typename TRANSITIONS, typename EVENT, typename NEW_STATE,
         typename MACHINE>
concept has_enter_hook_after_wildcard =
    has_enter<OBSERVER, NEW_STATE, MACHINE> ||
    !mtl::empty_v<wildcard_sources_with_enter_from<OBSERVER, TRANSITIONS, EVENT, NEW_STATE,
                                                   MACHINE>>;

template<typename OBSERVER, typename TRANSITIONS, typename EVENT, typename NEW_STATE,
         typename MACHINE>
concept has_transition_hook_after_wildcard =
    has_transition<OBSERVER, EVENT, NEW_STATE, MACHINE> ||
    !mtl::empty_v<wildcard_sources_with_transition_from<OBSERVER, TRANSITIONS, EVENT, NEW_STATE,
                                                        MACHINE>>;

// The observers of one machine, by reference: a parent machine hands its
// own to every child. A hook goes, in injection order, to the observers
// that have it for the edge. MACHINE is the machine a hook is delivered
// for; a wildcard's hooks read its MACHINE::enabled_table
template<concepts::observer... OBSERVERs>
class InjectedObservers {
public:
    using observer_list = mtl::typelist<OBSERVERs...>;

    explicit InjectedObservers(OBSERVERs&... observers) : observers_(observers...) {}

    // --- what the observers answer together ---------------------------------

    // TABLE without the features none of the observers enables
    template<concepts::transition_table TABLE>
    using table_with_enabled_features = enabled_table_t<TABLE, observer_list>;

    template<concepts::feature_tag TAG>
    static constexpr bool any_observer_enables = feature_enabled_v<TAG, OBSERVERs...>;

    // Does a notify hook of any observer take an annotation or value of
    // STATE?
    template<concepts::state STATE>
    static constexpr bool any_observer_notified_by =
        (concepts::notified_of<OBSERVERs, STATE> || ...);

    // Does an observer's exit hook of any of STATES ask for the edge to TO?
    template<concepts::state TO, typename MACHINE>
    struct exit_hook_from_asked_for {
        template<concepts::state FROM>
        struct pred : std::bool_constant<(has_exit_from<OBSERVERs, FROM, TO, MACHINE> || ...)> {};
    };

    template<mtl::concepts::typelist STATES, concepts::state TO, typename MACHINE>
    static constexpr bool any_exit_hook_from =
        mtl::any_of_v<STATES, exit_hook_from_asked_for<TO, MACHINE>::template pred>;

    // Runs every observer's own compile-time proof of TABLE (validate)
    template<concepts::transition_table TABLE>
    static constexpr bool all_observers_validate = (validated<OBSERVERs, TABLE>() && ...);

    // --- one of them --------------------------------------------------------

    template<concepts::observer OBSERVER>
    [[nodiscard]] OBSERVER& observer()
    {
        return std::get<mtl::index_of_v<OBSERVER, observer_list>>(observers_);
    }

    // --- the hooks of an edge -----------------------------------------------

    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE>
    void deliverExitHooks(MACHINE& machine)
    {
        deliverExitHooksTo<OLD_STATE, NEW_STATE>(
            machine,
            hooked_indices_t<has_exit_hook<OBSERVERs, OLD_STATE, NEW_STATE, MACHINE>...>{});
    }

    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE>
    void deliverEnterHooks(MACHINE& machine)
    {
        deliverEnterHooksTo<OLD_STATE, NEW_STATE>(
            machine,
            hooked_indices_t<has_enter_hook<OBSERVERs, OLD_STATE, NEW_STATE, MACHINE>...>{});
    }

    template<concepts::state FROM_STATE, concepts::event EVENT, concepts::state TO_STATE,
             typename MACHINE>
    void deliverTransitionHooks(MACHINE& machine)
    {
        deliverTransitionHooksTo<FROM_STATE, EVENT, TO_STATE>(
            machine, hooked_indices_t<has_transition_hook<OBSERVERs, FROM_STATE, EVENT, TO_STATE,
                                                          MACHINE>...>{});
    }

    // --- the entry side of a from<any_state> transition ---------------------
    // The state left is only known by its index in the machine's states.
    // As on an exact edge, an observer's hook of the edge is taken where
    // it has one for the state left - through a compare chain over those
    // sources - and the hook of the state entered otherwise

    template<concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE>
    void deliverEnterHooksAfterWildcard(MACHINE& machine, std::size_t state_left)
    {
        deliverEnterHooksAfterWildcardTo<EVENT, NEW_STATE>(
            machine, state_left,
            hooked_indices_t<has_enter_hook_after_wildcard<
                OBSERVERs, typename MACHINE::enabled_table, EVENT, NEW_STATE, MACHINE>...>{});
    }

    template<concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE>
    void deliverTransitionHooksAfterWildcard(MACHINE& machine, std::size_t state_left)
    {
        deliverTransitionHooksAfterWildcardTo<EVENT, NEW_STATE>(
            machine, state_left,
            hooked_indices_t<has_transition_hook_after_wildcard<
                OBSERVERs, typename MACHINE::enabled_table, EVENT, NEW_STATE, MACHINE>...>{});
    }

private:
    // INDEXs: the observers with a hook for the edge

    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE,
             std::size_t... INDEXs>
    void deliverExitHooksTo(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (exitHook<OLD_STATE, NEW_STATE>(std::get<INDEXs>(observers_), machine), ...);
    }

    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE,
             std::size_t... INDEXs>
    void deliverEnterHooksTo(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (enterHook<OLD_STATE, NEW_STATE>(std::get<INDEXs>(observers_), machine), ...);
    }

    template<concepts::state FROM_STATE, concepts::event EVENT, concepts::state TO_STATE,
             typename MACHINE, std::size_t... INDEXs>
    void deliverTransitionHooksTo(MACHINE& machine, std::index_sequence<INDEXs...>)
    {
        (transitionHook<FROM_STATE, EVENT, TO_STATE>(std::get<INDEXs>(observers_), machine), ...);
    }

    template<concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE,
             std::size_t... INDEXs>
    void deliverEnterHooksAfterWildcardTo(MACHINE& machine, std::size_t state_left,
                                                 std::index_sequence<INDEXs...>)
    {
        (enterHookAfterWildcard<EVENT, NEW_STATE>(
             std::get<INDEXs>(observers_), machine, state_left),
         ...);
    }

    template<concepts::event EVENT, concepts::state NEW_STATE, typename MACHINE,
             std::size_t... INDEXs>
    void deliverTransitionHooksAfterWildcardTo(MACHINE& machine, std::size_t state_left,
                                                      std::index_sequence<INDEXs...>)
    {
        (transitionHookAfterWildcard<EVENT, NEW_STATE>(
             std::get<INDEXs>(observers_), machine, state_left),
         ...);
    }

    // The hook of the edge for the sources the observer has it for, the
    // hook of the state entered for the rest
    template<concepts::event EVENT, concepts::state NEW_STATE, concepts::observer OBSERVER,
             typename MACHINE>
    void enterHookAfterWildcard(OBSERVER& observer, MACHINE& machine,
                                       [[maybe_unused]] std::size_t state_left)
    {
        if (enterHookFromStateLeft<NEW_STATE>(
                observer, machine, state_left,
                wildcard_sources_with_enter_from<OBSERVER, typename MACHINE::enabled_table, EVENT,
                                                 NEW_STATE, MACHINE>{})) {
            return;
        }
        if constexpr (has_enter<OBSERVER, NEW_STATE, MACHINE>) {
            observer.template onEnter<NEW_STATE>(machine);
        }
    }

    template<concepts::event EVENT, concepts::state NEW_STATE, concepts::observer OBSERVER,
             typename MACHINE>
    void transitionHookAfterWildcard(OBSERVER& observer, MACHINE& machine,
                                            [[maybe_unused]] std::size_t state_left)
    {
        if (transitionHookFromStateLeft<EVENT, NEW_STATE>(
                observer, machine, state_left,
                wildcard_sources_with_transition_from<OBSERVER, typename MACHINE::enabled_table,
                                                      EVENT, NEW_STATE, MACHINE>{})) {
            return;
        }
        if constexpr (has_transition<OBSERVER, EVENT, NEW_STATE, MACHINE>) {
            observer.template onTransition<EVENT, NEW_STATE>(machine);
        }
    }

    // Whether the state left was one of SOURCEs, its hook delivered
    template<concepts::state NEW_STATE, concepts::observer OBSERVER, typename MACHINE,
             concepts::state... SOURCEs>
    bool enterHookFromStateLeft(OBSERVER& observer, MACHINE& machine,
                                       std::size_t state_left, mtl::typelist<SOURCEs...>)
    {
        return (
            (state_left == mtl::index_of_v<SOURCEs, typename MACHINE::enabled_table::states> &&
             (observer.template onEnterFrom<SOURCEs, NEW_STATE>(machine), true)) ||
            ...);
    }

    template<concepts::event EVENT, concepts::state NEW_STATE, concepts::observer OBSERVER,
             typename MACHINE, concepts::state... SOURCEs>
    bool transitionHookFromStateLeft(OBSERVER& observer, MACHINE& machine,
                                            std::size_t state_left, mtl::typelist<SOURCEs...>)
    {
        return (
            (state_left == mtl::index_of_v<SOURCEs, typename MACHINE::enabled_table::states> &&
             (observer.template onTransitionFrom<SOURCEs, EVENT, NEW_STATE>(machine), true)) ||
            ...);
    }

    std::tuple<OBSERVERs&...> observers_;
};

} // namespace fsm::internal
