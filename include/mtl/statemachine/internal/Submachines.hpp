/*
 * fsm: the submachines of a machine - the child machine a composite
 * state holds in its alternative of the state variant: entered and
 * left with that state, and the first to react to an event
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/internal/Checks.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Traits.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <cstddef>
#include <memory>
#include <type_traits>

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

// The emitting states of a composite's submachine (ENABLED_SUBMACHINE:
// its table as the observers enable it) that the submachine can reach
// from where this composite starts it: two composites over one
// sub-table may each reach only part of its ends, and a composite
// owes a transition for the events of its own part alone
template<composite STATE, concepts::transition_table ENABLED_SUBMACHINE>
using emitting_states_reached_t = mtl::filter_t<
    typename reachable_closure<mtl::typelist<submachine_initial_t<STATE, ENABLED_SUBMACHINE>>,
                               typename ENABLED_SUBMACHINE::transitions>::type,
    is_emitting>;

template<composite STATE, concepts::observer_list OBSERVER_LIST>
using submachine_emitting_states_t =
    emitting_states_reached_t<STATE, enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>>;

// What a composite state of TABLE owes its submachine: a transition
// for every event a state reached from its initial emits - own or
// through a wildcard, guarded or not. An event nobody could take is a
// dead declaration, a transition for one nobody could emit dead code
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

// A submachine is entered at its initial state - the sub-table's, or
// the one its composite state names - before anyone could take an
// event: that state does not emit
template<concepts::observer_list OBSERVER_LIST>
struct submachine_starts_silent {
    template<composite STATE>
    struct pred
        : std::bool_constant<!emitting<submachine_initial_t<
              STATE, enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>>>> {};
};

// The initial state a composite names is a state of its submachine, as
// the observers enable it, and not a final one (nothing leaves a final
// state - a submachine started in one could only be left)
template<concepts::observer_list OBSERVER_LIST>
struct initial_in_submachine {
    template<composite STATE>
    struct pred : std::true_type {};

    template<starting STATE>
    struct pred<STATE>
        : std::bool_constant<
              mtl::has_a_v<typename enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>::states,
                           typename STATE::initial> &&
              !mtl::has_a_v<
                  typename enabled_table_t<submachine_t<STATE>, OBSERVER_LIST>::final_states,
                  typename STATE::initial>> {};
};

// What a composite state is in its machine's state variant: the state
// itself, with its child machine behind it. The child is constructed
// after the state's own entry and transition hooks and leaves its
// active state before the state's exit hooks - innermost first - but
// is destroyed only with the state: the one time a state is alone is
// between its construction and its hooks, inside a process() call,
// where nothing destroys it. A friend of the machine: leaving the
// child is the child's private leaveActiveState()
template<composite STATE, typename CHILD>
class StateWithSubmachine : public STATE {
public:
    using STATE::STATE;

    // Trivial where the child is: a destructor of its own on every
    // alternative puts the variant's destructor dispatch into every
    // state change
    ~StateWithSubmachine()
        requires std::is_trivially_destructible_v<CHILD>
    = default;

    ~StateWithSubmachine()
    {
        std::destroy_at(&child_.machine);
    }

    // The child machine's constructor enters its initial state - the
    // sub-table's, or the one the composite state names. The parent
    // machine hands in what a child is constructed from
    template<typename PARENT_CONTEXTS, typename OBSERVERS>
    void enterChild(PARENT_CONTEXTS& parent_contexts, OBSERVERS const& observers)
    {
        using initial = submachine_initial_t<STATE, typename CHILD::enabled_table>;
        std::construct_at(&child_.machine, std::type_identity<initial>{}, parent_contexts,
                          observers);
    }

    void leaveChild()
    {
        child_.machine.leaveActiveState();
    }

    [[nodiscard]] CHILD& child()
    {
        return child_.machine;
    }

    [[nodiscard]] CHILD const& child() const
    {
        return child_.machine;
    }

private:
    // Room for the child, constructed and destroyed by hand: a plain
    // member would be constructed with the state, before its hooks. A
    // union of its own, so that the state's constructors - inherited
    // above - stay usable: an anonymous union with a non-trivial
    // member deletes them
    union child_storage {
        child_storage() {}
        ~child_storage()
            requires std::is_trivially_destructible_v<CHILD>
        = default;
        ~child_storage() {}
        CHILD machine;
    };

    child_storage child_;
};

// The state behind an alternative of the state variant
template<concepts::state ALTERNATIVE>
struct plain_state : std::type_identity<ALTERNATIVE> {};

template<composite STATE, typename CHILD>
struct plain_state<StateWithSubmachine<STATE, CHILD>> : std::type_identity<STATE> {};

template<concepts::state ALTERNATIVE>
using plain_state_t = typename plain_state<ALTERNATIVE>::type;

// The child machines of one machine: at most one lives, that of the
// active composite state, inside that state's alternative of the
// state variant. COMPOSITES: the composite states of the table the
// machine runs; CHILD_MACHINES: the machine each of them owns while
// active, in the same order. A friend of the machine: the class drives
// a child through its processWithReaction() and hands an emitted event
// to the parent machine's processInThisTable()
template<mtl::concepts::typelist COMPOSITES, mtl::concepts::typelist CHILD_MACHINES>
class Submachines {
public:
    template<composite COMPOSITE>
    static constexpr bool has_composite = mtl::has_a_v<COMPOSITES, COMPOSITE>;

    template<composite COMPOSITE>
    using child_of = mtl::at_t<mtl::index_of_v<COMPOSITE, COMPOSITES>, CHILD_MACHINES>;

    // What a state of the table is in the state variant
    template<concepts::state STATE>
    struct alternative : std::type_identity<STATE> {};

    template<composite STATE>
    struct alternative<STATE> : std::type_identity<StateWithSubmachine<STATE, child_of<STATE>>> {};

    template<concepts::state STATE>
    using alternative_t = typename alternative<STATE>::type;

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

    template<concepts::observer_list OBSERVER_LIST>
    static constexpr bool every_initial_in_its_submachine =
        mtl::all_of_v<COMPOSITES, initial_in_submachine<OBSERVER_LIST>::template pred>;

    // --- an event arrives ---------------------------------------------------

    // The child of the active composite state reacts before the parent
    // machine's own table. Whatever the child does happens in place for
    // the parent - unless it entered a state whose emitted event moves
    // the parent on. One body per distinct child machine: two
    // composites sharing a child share it
    template<typename PARENT, typename CHILD, concepts::event EVENT>
    static reaction reactIn(PARENT& parent, CHILD& child, EVENT const& event)
    {
        reaction const of_child = child.processWithReaction(event);
        if (of_child == reaction::state_entered &&
            reactToEmittedEvent(parent, child) == reaction::state_entered) {
            return reaction::state_entered;
        }
        return of_child == reaction::none ? reaction::none : reaction::in_place;
    }

private:
    // The emitting states a child can be in: those its owning composite
    // states can reach from where each starts it
    template<typename CHILD>
    struct owned_by {
        template<composite COMPOSITE>
        struct pred : std::is_same<child_of<COMPOSITE>, CHILD> {};
    };

    template<composite COMPOSITE>
    struct emitting_states_reached
        : std::type_identity<
              emitting_states_reached_t<COMPOSITE, typename child_of<COMPOSITE>::enabled_table>> {
    };

    template<typename CHILD>
    using emitting_states_of_t = mtl::unique_t<mtl::linearize_t<mtl::transform_t<
        mtl::filter_t<COMPOSITES, owned_by<CHILD>::template pred>, emitting_states_reached>>>;

    // The event the child's active state emits goes to the parent
    // machine's own table. The reaction may destroy the child: the fold
    // stops at the first match
    template<typename PARENT, typename CHILD>
    static reaction reactToEmittedEvent([[maybe_unused]] PARENT& parent,
                                        [[maybe_unused]] CHILD const& child)
    {
        reaction result = reaction::none;
        [&]<typename... EMITTINGs>(mtl::typelist<EMITTINGs...>) {
            static_cast<void>(
                ((child.template is<EMITTINGs>() &&
                  (result = parent.processInThisTable(emitted_t<EMITTINGs>{}), true)) ||
                 ...));
        }(emitting_states_of_t<CHILD>{});
        return result;
    }
};

} // namespace fsm::internal
