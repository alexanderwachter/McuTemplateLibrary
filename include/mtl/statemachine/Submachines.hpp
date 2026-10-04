/*
 * fsm: the submachines of a machine - the child machine of its active
 * composite state: entered and left with that state, and the first to
 * react to an event
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Checks.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <cstddef>
#include <optional>
#include <type_traits>
#include <variant>

namespace fsm::internal {

// What an event did to a machine's active state. A child machine
// answers its parent with it: only a state entered emits its event
enum class reaction : std::size_t { none, in_place, state_entered };

// A composite state whose submachine is TABLE
template<concepts::transition_table TABLE>
struct composite_nesting_table {
    template<composite STATE>
    struct pred : std::is_same<submachine_t<STATE>, TABLE> {};
};

// A composite state whose hierarchy carries T somewhere below it
template<concepts::annotation T>
struct nesting_carrier {
    template<composite STATE>
    struct pred : annotation_in_table<submachine_t<STATE>, T> {};
};

// An annotation type belongs to one level of a nesting path: a
// composite state carrying T forbids T below it. A sub-state refining
// the parent's value would leave the observers and the machine's
// annotation<T>() query disagreeing - moving on to a sibling without T
// re-notifies nothing, while the parent's T is still the active one
template<concepts::state STATE>
struct annotation_levels_exclusive : std::true_type {};

template<composite STATE>
struct annotation_levels_exclusive<STATE>
    : std::bool_constant<mtl::none_of_v<annotation_types_t<STATE>,
                                        carried_in<submachine_t<STATE>>::template pred>> {};

template<emitting STATE>
struct emits_default_constructible : std::is_default_constructible<emitted_t<STATE>> {};

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

// The child machines of one machine: at most one lives, that of the
// active composite state. COMPOSITES: the composite states of the
// table the machine runs; CHILD_MACHINES: the machine each of them
// owns while active, in the same order. A friend of the machine: the
// class drives a child through its react() and leaveActiveState() and
// hands an emitted event to the parent machine's reactInOwnTable()
template<mtl::concepts::typelist COMPOSITES, mtl::concepts::typelist CHILD_MACHINES>
class Submachines {
public:
    template<composite COMPOSITE>
    static constexpr bool has_composite = mtl::has_a_v<COMPOSITES, COMPOSITE>;

    template<composite COMPOSITE>
    using child_of = mtl::at_t<mtl::index_of_v<COMPOSITE, COMPOSITES>, CHILD_MACHINES>;

    // --- what the table must satisfy ----------------------------------------

    // TABLE: the table the composite states belong to. A table nesting
    // itself would never end
    template<concepts::transition_table TABLE>
    static constexpr bool no_composite_nests_its_own_table =
        mtl::none_of_v<COMPOSITES, composite_nesting_table<TABLE>::template pred>;

    // An annotation of a composite state does not recur in its submachine
    static constexpr bool annotations_exclusive_per_level =
        mtl::all_of_v<COMPOSITES, annotation_levels_exclusive>;

    // The parent machine constructs the event a state of TABLE emits
    template<concepts::transition_table TABLE>
    static constexpr bool emitted_events_default_constructible =
        mtl::all_of_v<mtl::filter_t<typename TABLE::states, is_emitting>,
                      emits_default_constructible>;

    // By a transition of TABLE, as the observers enable the submachines
    template<concepts::transition_table TABLE, concepts::observer_list OBSERVER_LIST>
    static constexpr bool every_composite_takes_emitted_events =
        mtl::all_of_v<COMPOSITES, emitted_events_taken_in<TABLE, OBSERVER_LIST>::template pred>;

    template<concepts::observer_list OBSERVER_LIST>
    static constexpr bool every_submachine_starts_silent =
        mtl::all_of_v<COMPOSITES, submachine_starts_silent<OBSERVER_LIST>::template pred>;

    // --- a child lives as long as its composite state -----------------------

    // After the composite state's own entry hooks: the child machine's
    // constructor enters its initial state. The parent machine hands in
    // what a child is constructed from. Nothing to do for a plain state
    template<concepts::state STATE, typename PARENT_CONTEXTS, typename OBSERVERS>
    void enterWith([[maybe_unused]] PARENT_CONTEXTS& parent_contexts,
                   [[maybe_unused]] OBSERVERS const& observers)
    {
        if constexpr (composite<STATE>) {
            children_.template emplace<child_of<STATE>>(parent_contexts, observers);
        }
    }

    // Before the composite state's own exit hooks: the child leaves its
    // active state, innermost first
    template<concepts::state STATE>
    void leaveWith()
    {
        if constexpr (composite<STATE>) {
            this->template activeChildOf<STATE>().leaveActiveState();
            children_.template emplace<std::monostate>();
        }
    }

    // --- an event arrives ---------------------------------------------------

    // The child of the active composite state reacts before the parent
    // machine's own table. A local event (fsm::timeout) belongs to one
    // level and does not descend
    template<typename PARENT, concepts::event EVENT>
    reaction react([[maybe_unused]] PARENT& parent, [[maybe_unused]] EVENT const& event)
    {
        if constexpr (local_event_v<EVENT>) {
            return reaction::none;
        } else {
            return this->reactInActiveChild(parent, passedDown(event));
        }
    }

    // --- queries ------------------------------------------------------------

    // The child machine of COMPOSITE, nullptr while another child or
    // none lives
    template<composite COMPOSITE>
    [[nodiscard]] child_of<COMPOSITE> const* childOf() const
    {
        return std::get_if<child_of<COMPOSITE>>(&children_);
    }

    // Only the composite states with T somewhere below ask their child
    template<concepts::annotation T, typename PARENT>
    void annotationOfActiveChild(PARENT const& parent, std::optional<T>& annotation) const
    {
        using nesting_carriers = mtl::filter_t<COMPOSITES, nesting_carrier<T>::template pred>;
        [&]<typename... NESTINGs>(mtl::typelist<NESTINGs...>) {
            static_cast<void>(
                ((parent.template is<NESTINGs>() &&
                  (annotation = this->template childOf<NESTINGs>()->template annotation<T>(),
                   true)) ||
                 ...));
        }(nesting_carriers{});
    }

private:
    template<typename PARENT, concepts::event EVENT>
    reaction reactInActiveChild([[maybe_unused]] PARENT& parent,
                                [[maybe_unused]] EVENT const& event)
    {
        reaction result = reaction::none;
        [&]<typename... COMPOSITEs>(mtl::typelist<COMPOSITEs...>) {
            static_cast<void>(
                ((parent.template is<COMPOSITEs>() &&
                  (result = this->template reactInChildOf<COMPOSITEs>(parent, event), true)) ||
                 ...));
        }(COMPOSITES{});
        return result;
    }

    // Whatever the child does happens in place for the parent machine -
    // unless it entered a state whose emitted event moves the parent on
    template<composite COMPOSITE, typename PARENT, concepts::event EVENT>
    reaction reactInChildOf(PARENT& parent, EVENT const& event)
    {
        reaction const of_child = this->template activeChildOf<COMPOSITE>().react(event);
        if (of_child == reaction::state_entered &&
            this->template reactToEmittedEvent<COMPOSITE>(parent) == reaction::state_entered) {
            return reaction::state_entered;
        }
        return of_child == reaction::none ? reaction::none : reaction::in_place;
    }

    // The event the child's active state emits goes to the parent
    // machine's own table. The reaction may destroy the child: the fold
    // stops at the first match
    template<composite COMPOSITE, typename PARENT>
    reaction reactToEmittedEvent([[maybe_unused]] PARENT& parent)
    {
        using emitting_states =
            mtl::filter_t<typename child_of<COMPOSITE>::enabled_table::states, is_emitting>;
        reaction result = reaction::none;
        [&]<typename... EMITTINGs>(mtl::typelist<EMITTINGs...>) {
            [[maybe_unused]] auto const& child = this->template activeChildOf<COMPOSITE>();
            static_cast<void>(
                ((child.template is<EMITTINGs>() &&
                  (result = parent.reactInOwnTable(emitted_t<EMITTINGs>{}), true)) ||
                 ...));
        }(emitting_states{});
        return result;
    }

    template<composite COMPOSITE>
    child_of<COMPOSITE>& activeChildOf()
    {
        auto* const child = std::get_if<child_of<COMPOSITE>>(&children_);
#if MTL_FSM_CHECKS
        MTL_FSM_ASSERT(child != nullptr, "fsm: composite state active without its submachine");
#endif
        return *child;
    }

    // One alternative per distinct child machine, std::monostate while
    // no composite state is active; nothing in a table without one
    using child_variant =
        mtl::rebind_t<mtl::prepend_t<std::monostate, mtl::unique_t<CHILD_MACHINES>>, std::variant>;
    using child_storage =
        std::conditional_t<mtl::empty_v<COMPOSITES>, mtl::nil_type, child_variant>;

    [[no_unique_address]] child_storage children_{};
};

} // namespace fsm::internal
