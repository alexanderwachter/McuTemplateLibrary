/*
 * fsm: the state machine itself and its dispatch.
 * The contract lives in <mtl/StateMachine.hpp>
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Checks.hpp>
#include <mtl/statemachine/Contexts.hpp>
#include <mtl/statemachine/Guards.hpp>
#include <mtl/statemachine/InjectedObservers.hpp>
#include <mtl/statemachine/Submachines.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/statemachine/Visit.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

namespace fsm {

template<concepts::transition_table TRANSITION_TABLE, concepts::observer... OBSERVERs>
class StateMachine {
    // drives and leaves the child machines, for their parent
    template<mtl::concepts::typelist, mtl::concepts::typelist>
    friend class internal::Submachines;

    using injected_observers = internal::InjectedObservers<OBSERVERs...>;
    using injected           = typename injected_observers::observer_list;
    using reaction           = internal::reaction;

    using TRANSITIONS =
        typename injected_observers::template table_with_enabled_features<TRANSITION_TABLE>;
    static_assert(!mtl::empty_v<typename TRANSITIONS::states>,
                  "StateMachine: every entry of the table belongs to a disabled feature - a "
                  "submachine emptied this way is a feature itself: tag its composite state");

public:
    // The table as the user named it; a child machine's comes wrapped
    // in internal::nested
    using table         = internal::plain_table_t<TRANSITION_TABLE>;
    // The table this machine runs: without the features no observer enables
    using enabled_table = TRANSITIONS;
    using state_variant = mtl::rebind_t<typename TRANSITIONS::states, std::variant>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

    // 0 for a root, one more per composite state above this machine
    static constexpr std::size_t depth = internal::table_depth_v<TRANSITION_TABLE>;

private:
    using contexts = internal::machine_contexts_t<TRANSITION_TABLE, TRANSITIONS>;
    using guards   = internal::TransitionGuards<OBSERVERs...>;

    using final_states = typename TRANSITIONS::final_states;

    // The machine a composite state owns while active: of its
    // submachine's table, one level down, with the contexts it inherits
    template<internal::composite COMPOSITE>
    struct child_machine_of
        : std::type_identity<StateMachine<
              internal::nested<internal::submachine_t<COMPOSITE>, StateMachine::depth + 1,
                               typename contexts::template inherited_by<COMPOSITE>>,
              OBSERVERs...>> {};

    using composites     = mtl::filter_t<typename TRANSITIONS::states, internal::is_composite>;
    using child_machines = mtl::transform_t<composites, child_machine_of>;
    using submachines    = internal::Submachines<composites, child_machines>;

public:
    explicit StateMachine(OBSERVERs&... observers)
        : observers_(observers...),
          current_(std::make_from_tuple<state_variant>(
              contexts_.template initialArgumentsOf<initial_state>()))
    {
        this->enterInitialState();
    }

    // A child machine, built by its parent with the contexts it inherits
    template<mtl::concepts::typelist PARENT_DECLARED, mtl::concepts::typelist PARENT_INHERITED>
    StateMachine(internal::MachineContexts<PARENT_DECLARED, PARENT_INHERITED>& parent_contexts,
                 injected_observers const& observers)
        : contexts_(parent_contexts), observers_(observers),
          current_(std::make_from_tuple<state_variant>(
              contexts_.template initialArgumentsOf<initial_state>()))
    {
        this->enterInitialState();
    }

    // Observers may keep the machine's address beyond a hook
    StateMachine(StateMachine const&)            = delete;
    StateMachine& operator=(StateMachine const&) = delete;

    // True when the event was handled: by a transition of this table or
    // by a submachine
    template<concepts::event EVENT>
    bool process(EVENT const& event)
    {
        return this->processWithReaction(event) != reaction::none;
    }

    template<concepts::state STATE>
    [[nodiscard]] bool is() const
    {
        return std::holds_alternative<STATE>(current_);
    }

    // Does the machine rest in a state the table marks final<>?
    [[nodiscard]] bool isFinished() const
    {
        return [this]<typename... FINALs>(mtl::typelist<FINALs...>) {
            return (this->template is<FINALs>() || ...);
        }(final_states{});
    }

    // The active state object, nullptr if STATE is not active. The next
    // transition destroys it: do not keep the pointer
    template<concepts::state STATE>
    [[nodiscard]] STATE const* getIf() const
    {
        return std::get_if<STATE>(&current_);
    }

    template<concepts::context T>
    [[nodiscard]] T const& context() const
    {
        return contexts_.template context<T>();
    }

    // The child machine of COMPOSITE while that state is active,
    // nullptr otherwise
    template<internal::composite COMPOSITE>
    [[nodiscard]] auto const* submachine() const
    {
        static_assert(submachines::template has_composite<COMPOSITE>,
                      "StateMachine::submachine: not a composite state of this table");
        return this->template is<COMPOSITE>() ? submachines_.template childOf<COMPOSITE>()
                                              : nullptr;
    }

    // The annotation element T of the active state, or of the active
    // sub-state at the one level carrying T; empty while none does
    template<concepts::annotation T>
    [[nodiscard]] std::optional<T> annotation() const
    {
        static_assert(annotation_in_table_v<table, T>,
                      "StateMachine::annotation: no state of the table carries this annotation");
        std::optional<T> result = this->template annotationOfActiveState<T>();
        submachines_.annotationOfActiveChild(*this, result);
        return result;
    }

private:
    // --- an event arrives ---------------------------------------------------

    // The active composite state's child machine first, then this
    // machine's own table
    template<concepts::event EVENT>
    reaction processWithReaction(EVENT const& event)
    {
        this->beginProcessing();
        reaction result = submachines_.react(*this, event);
        if (result == reaction::none) {
            result = this->reactInOwnTable(event);
        }
        this->endProcessing();
        return result;
    }

    // The active state's guarded alternatives in table order, then its
    // unguarded one; without that, the wildcards - never from a final
    // state. A wildcard only exits inside the visitor, where the state
    // left is known, and enters its target afterwards
    template<concepts::event EVENT>
    reaction reactInOwnTable(EVENT const& event)
    {
        using wildcards = wildcard_transitions_t<TRANSITIONS, EVENT>;
        std::size_t const outcome = internal::dispatch(
            [this, &event](auto& state) -> std::size_t {
                using state_type = std::decay_t<decltype(state)>;
                using own        = exact_transitions_t<TRANSITIONS, state_type, EVENT>;
                using guarded    = mtl::filter_t<own, internal::is_guarded>;
                using unguarded  = mtl::find_if_t<own, internal::is_unguarded>;
                if constexpr (!mtl::empty_v<guarded>) {
                    if (reaction const fired = this->fireFirstAllowed(guarded{}, state, event);
                        fired != reaction::none) {
                        return static_cast<std::size_t>(fired);
                    }
                }
                if constexpr (!std::is_same_v<unguarded, mtl::nil_type>) {
                    return static_cast<std::size_t>(this->template fire<unguarded>(state, event));
                } else if constexpr (!mtl::empty_v<wildcards> &&
                                     !mtl::has_a_v<final_states, state_type>) {
                    return this->exitForFirstAllowed(wildcards{}, state, event);
                } else {
                    return static_cast<std::size_t>(reaction::none);
                }
            },
            current_);
        if constexpr (!mtl::empty_v<wildcards>) {
            if (outcome >= StateMachine::exited_for_wildcard) {
                // the state left is still the current one: its index names it
                this->enterTargetOf(outcome - StateMachine::exited_for_wildcard, current_.index(),
                                    wildcards{}, event);
                return reaction::state_entered;
            }
        }
        return static_cast<reaction>(outcome);
    }

    // --- transitions of the active state ------------------------------------

    // The state reference dangles once a transition fired: the fold
    // stops there
    template<concepts::state STATE, concepts::transition... GUARDEDs, concepts::event EVENT>
    reaction fireFirstAllowed(mtl::typelist<GUARDEDs...>, STATE& state, EVENT const& event)
    {
        reaction result = reaction::none;
        static_cast<void>(
            ((guards::template allow<GUARDEDs>(observers_, state, event) &&
              (result = this->template fire<GUARDEDs>(state, event), true)) ||
             ...));
        return result;
    }

    template<concepts::transition TRANSITION, concepts::state STATE, concepts::event EVENT>
    reaction fire(STATE& state, EVENT const& event)
    {
        using TO_STATE = typename TRANSITION::to;
        if constexpr (internal::is_internal_v<TRANSITION>) {
            static_assert(requires { state.handle(event); },
                          "internal transition: the state must provide handle(EVENT const&)");
            state.handle(event);
        } else if constexpr (internal::payload_constructible_v<TO_STATE, EVENT>) {
            this->template changeState<STATE, TO_STATE>(event);
        } else {
            this->template changeState<STATE, TO_STATE>();
        }
        this->template notifyTransition<STATE, EVENT, TO_STATE>();
        this->template enterSubmachine<TO_STATE>();
        return internal::is_internal_v<TRANSITION> ? reaction::in_place : reaction::state_entered;
    }

    // Instantiated per (edge, event), for targets built from the event
    template<concepts::state OLD_STATE, concepts::state NEW_STATE, concepts::event EVENT>
    void changeState(EVENT const& event)
    {
        this->template leave<OLD_STATE, NEW_STATE>();
        this->template construct<NEW_STATE>(event);
        this->template enter<OLD_STATE, NEW_STATE>();
    }

    // Instantiated once per edge, shared by every event triggering it
    template<concepts::state OLD_STATE, concepts::state NEW_STATE>
    void changeState()
    {
        this->template leave<OLD_STATE, NEW_STATE>();
        this->template construct<NEW_STATE>();
        this->template enter<OLD_STATE, NEW_STATE>();
    }

    // A composite state's child machine is left before the state itself
    template<concepts::state OLD_STATE, concepts::state NEW_STATE>
    void leave()
    {
        submachines_.template leaveWith<OLD_STATE>();
        observers_.template deliverExitHooks<OLD_STATE, NEW_STATE>(*this);
    }

    template<concepts::state NEW_STATE, typename... ARGs>
    void construct(ARGs const&... args)
    {
        std::apply(
            [&](auto&... state_contexts) {
                current_.template emplace<NEW_STATE>(args..., state_contexts...);
            },
            contexts_.template contextsOf<NEW_STATE>());
    }

    template<concepts::state OLD_STATE, concepts::state NEW_STATE>
    void enter()
    {
        observers_.template deliverEnterHooks<OLD_STATE, NEW_STATE>(*this);
    }

    template<concepts::state FROM_STATE, concepts::event EVENT, concepts::state TO_STATE>
    void notifyTransition()
    {
        observers_.template deliverTransitionHooks<FROM_STATE, EVENT, TO_STATE>(*this);
    }

    void enterInitialState()
    {
        this->beginProcessing();
        this->template enter<mtl::nil_type, initial_state>();
        this->template enterSubmachine<initial_state>();
        this->endProcessing();
    }

    // A composite state's child machine is entered after the state
    // itself, from this machine's contexts and observers
    template<concepts::state STATE>
    void enterSubmachine()
    {
        submachines_.template enterWith<STATE>(contexts_, observers_);
    }

    // What a parent machine does to its child before destroying it; a
    // root's destructor runs no hooks
    void leaveActiveState()
    {
        this->beginProcessing();
        internal::dispatch(
            [this](auto& state) {
                this->template leave<std::decay_t<decltype(state)>, mtl::nil_type>();
                return true;
            },
            current_);
        this->endProcessing();
    }

    // --- from<any_state> transitions ----------------------------------------
    // One shared body per (event, target) enters the target: expanding
    // whole edges per source measured kilobytes in a machine with a
    // handful of wildcard events. Only an observer asking for the edge
    // pays, with a switch on the state left

    // What the visitor of the active state answers: a reaction, or
    // exited_for_wildcard + the index of the wildcard to enter. One
    // value: the index through a captured local instead measured
    // +432 B on a 14-state machine (arm-zephyr-eabi GCC, -Os)
    static constexpr std::size_t exited_for_wildcard =
        static_cast<std::size_t>(reaction::state_entered) + 1;

    // Runs the exit hooks of STATE for the first wildcard its guards allow
    template<concepts::state STATE, concepts::transition... WILDCARDs, concepts::event EVENT>
    std::size_t exitForFirstAllowed(mtl::typelist<WILDCARDs...>, STATE& state, EVENT const& event)
    {
        std::size_t outcome = static_cast<std::size_t>(reaction::none);
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(
                ((guards::template allow<WILDCARDs>(observers_, state, event) &&
                  (this->template leave<STATE, typename WILDCARDs::to>(),
                   outcome = StateMachine::exited_for_wildcard + INDEXs, true)) ||
                 ...));
        }(std::index_sequence_for<WILDCARDs...>{});
        return outcome;
    }

    template<concepts::transition... WILDCARDs, concepts::event EVENT>
    void enterTargetOf(std::size_t wildcard, std::size_t state_left, mtl::typelist<WILDCARDs...>,
                       EVENT const& event)
    {
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(
                ((wildcard == INDEXs &&
                  (this->template enterShared<typename WILDCARDs::to>(state_left, event), true)) ||
                 ...));
        }(std::index_sequence_for<WILDCARDs...>{});
    }

    template<concepts::state NEW_STATE, concepts::event EVENT>
    void enterShared(std::size_t state_left, EVENT const& event)
    {
        if constexpr (internal::payload_constructible_v<NEW_STATE, EVENT>) {
            this->template construct<NEW_STATE>(event);
        } else {
            this->template construct<NEW_STATE>();
        }
        observers_.template deliverEnterHooksAfterWildcard<EVENT, NEW_STATE>(*this, state_left);
        observers_.template deliverTransitionHooksAfterWildcard<EVENT, NEW_STATE>(*this,
                                                                                  state_left);
        this->template enterSubmachine<NEW_STATE>();
    }

    // --- annotation query ---------------------------------------------------

    // A compare chain over the states carrying T, each yielding its constant
    template<concepts::annotation T>
    std::optional<T> annotationOfActiveState() const
    {
        using states   = typename TRANSITIONS::states;
        using carriers = mtl::filter_t<states, internal::carrying<T>::template pred>;
        return [this]<typename... CARRIERs>(mtl::typelist<CARRIERs...>) {
            std::optional<T> annotation;
            static_cast<void>(((current_.index() == mtl::index_of_v<CARRIERs, states> &&
                                (annotation = CARRIERs::annotations.template get<T>(), true)) ||
                               ...));
            return annotation;
        }(carriers{});
    }

    // --- the re-entrancy check ----------------------------------------------

    void beginProcessing()
    {
#if MTL_FSM_CHECKS
        MTL_FSM_ASSERT(!processing_, "fsm: process() re-entered from a hook");
        MTL_FSM_ASSERT(!current_.valueless_by_exception(),
                       "fsm: a state constructor threw, the machine has no state");
        processing_ = true;
#endif
    }

    void endProcessing()
    {
#if MTL_FSM_CHECKS
        processing_ = false;
#endif
    }

    // --- what the table must satisfy ----------------------------------------

    static_assert(!internal::is_unnamed_table_v<table>,
                  "StateMachine: the table needs a name of its own - struct my_table : "
                  "fsm::transition_table<...> {}; - unnamed, all of its transitions are "
                  "spelled out in every symbol of the machine, which is not good for compile "
                  "time and memory usage");

    // Once, at the root: an observer's proof walks the whole hierarchy
    // and would be wrong on a sub-table alone
    static constexpr bool observersValidated()
    {
        if constexpr (StateMachine::depth == 0) {
            return injected_observers::template all_observers_validate<table>;
        } else {
            return true;
        }
    }
    static_assert(StateMachine::observersValidated());

    static_assert(guards::template no_guard_answered_by_two_observers<TRANSITIONS>,
                  "StateMachine: a guard is answered by more than one injected object");
    static_assert(guards::template every_guard_answered<TRANSITIONS>,
                  "StateMachine: a guard of the table has no static check and no injected "
                  "object answers it - inject one with bool check(GUARD, FROM const&[, EVENT "
                  "const&]) or bool check(GUARD)");

    static_assert(contexts::own_contexts_default_constructible,
                  "StateMachine: context types must be default constructible");
    static_assert(contexts::template every_state_constructible_from_its_contexts<TRANSITIONS>,
                  "StateMachine: a state must be constructible from its declared contexts "
                  "alone, in their order (default constructible without any)");

    static_assert(submachines::template no_composite_nests_its_own_table<table>,
                  "StateMachine: a state's submachine is the table it belongs to");
    static_assert(submachines::annotations_exclusive_per_level,
                  "StateMachine: an annotation of a composite state may not recur in its "
                  "submachine - annotate at the level where the value changes");

    static_assert(
        contexts::template parent_contexts_only_on_composites<TRANSITIONS>,
        "StateMachine: parent_contexts is declared by a state without a submachine");
    static_assert(
        contexts::template holds_parent_contexts_of_every_composite<TRANSITIONS>,
        "StateMachine: a submachine inherits a context its parent machine does not hold - a "
        "state of this table declares it, or this machine inherits it in turn");
    static_assert(contexts::template every_submachine_declares_its_parent_contexts<TRANSITIONS>,
                  "StateMachine: a submachine inherits a context no state of it (or of the "
                  "submachines inside it) declares");

    static_assert(submachines::template emitted_events_default_constructible<TRANSITIONS>,
                  "StateMachine: the event a state emits must be default constructible");
    static_assert(
        submachines::template every_composite_takes_emitted_events<TRANSITIONS, injected>,
        "StateMachine: a composite state has no transition for an event a state of its "
        "submachine emits");
    static_assert(
        submachines::template every_submachine_starts_silent<injected>,
        "StateMachine: the initial state of a submachine emits an event - nobody is there to "
        "take it yet");

    // --- data ---------------------------------------------------------------

    contexts contexts_;
    injected_observers observers_;
#if MTL_FSM_CHECKS
    bool processing_ = false;
#endif
    state_variant current_;
    [[no_unique_address]] submachines submachines_{};
};

} // namespace fsm
