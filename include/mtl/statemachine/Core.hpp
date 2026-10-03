/*
 * fsm: the state machine itself and its dispatch.
 * The contract lives in <mtl/StateMachine.hpp>
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Observer.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

// Debug checks on the machine's use: process() re-entered from a hook
// (the contract forbids it: on the wildcard path the exit hooks have
// run but the state has not changed yet), or a state constructor that
// threw and left the machine without a state. On without NDEBUG;
// define MTL_FSM_CHECKS to 0 or 1 to decide explicitly - the same
// value in every translation unit, the flag is a member. The failing
// check goes through MTL_FSM_ASSERT(condition, "message"), assert() by
// default
#ifndef MTL_FSM_CHECKS
#  ifdef NDEBUG
#    define MTL_FSM_CHECKS 0
#  else
#    define MTL_FSM_CHECKS 1
#  endif
#endif

#ifndef MTL_FSM_ASSERT
#  define MTL_FSM_ASSERT(condition, message) assert((condition) && message)
#endif

namespace fsm {

namespace internal {

// Fold-based alternative to std::visit for process(): every visitor
// instantiation is inlinable and no function-pointer table or
// bad_variant_access path can be emitted. A valueless variant matches no
// alternative and yields a value-initialized result (unreachable in
// process(): the states can never make the variant valueless).
// Measured GCC 15.2 x86-64 -Os (traffic_light.cpp): 32 bytes .text
// larger than std::visit. Measured arm-zephyr-eabi GCC 14.3 -Os
// (Cortex-M0+, 14-state/20-event machine): 3.7 kB smaller - std::visit
// emits per-(event, state) invoke thunks and tables that dominate at
// scale.
// The visitor answers every alternative with the type it answers the
// first with
template<typename VISITOR, typename FIRST, typename... ALTERNATIVEs>
constexpr auto visit(VISITOR&& visitor, std::variant<FIRST, ALTERNATIVEs...>& variant)
{
    using result_type = std::remove_cvref_t<decltype(visitor(std::declval<FIRST&>()))>;
    return [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
        result_type result{};
        static_cast<void>(((variant.index() == INDEXs &&
                            (result = visitor(*std::get_if<INDEXs>(&variant)), true)) ||
                           ...));
        return result;
    }(std::index_sequence_for<FIRST, ALTERNATIVEs...>{});
}

// The fold is the default: it wins clearly on embedded targets with
// large machines (measurements above), losing only a few bytes on
// hosted libstdc++ with small ones. Define MTL_FSM_FOLD_VISIT to 0 to
// use std::visit instead.
#ifndef MTL_FSM_FOLD_VISIT
#  define MTL_FSM_FOLD_VISIT 1
#endif

template<typename VISITOR, typename... ALTERNATIVEs>
constexpr auto dispatch(VISITOR&& visitor, std::variant<ALTERNATIVEs...>& variant)
{
#if MTL_FSM_FOLD_VISIT
    return internal::visit(std::forward<VISITOR>(visitor), variant);
#else
    return std::visit(std::forward<VISITOR>(visitor), variant);
#endif
}

// What an event did to a machine's active state. A child machine
// answers its parent with it: only a state entered emits its event
enum class reaction : std::size_t { none, in_place, state_entered };

} // namespace internal

template<concepts::transition_table TRANSITION_TABLE, concepts::observer... OBSERVERs>
class StateMachine {
    // a parent machine drives and leaves its child machines
    template<concepts::transition_table, concepts::observer...>
    friend class StateMachine;

    using injected = mtl::typelist<OBSERVERs...>;
    using reaction = internal::reaction;

    // The table without the features none of the observers enables
    using TRANSITIONS = enabled_table_t<TRANSITION_TABLE, injected>;
    static_assert(!mtl::empty_v<typename TRANSITIONS::states>,
                  "StateMachine: every entry of the table belongs to a disabled feature - a "
                  "submachine emptied this way is a feature itself: tag its composite state");

public:
    // The table as the user named it; a child machine's comes wrapped
    // in internal::nested
    using table         = internal::plain_table_t<TRANSITION_TABLE>;
    using state_variant = mtl::rebind_t<typename TRANSITIONS::states, std::variant>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

    // 0 for a root, one more per composite state above this machine
    static constexpr std::size_t depth = internal::table_depth_v<TRANSITION_TABLE>;

    // inherited: references to the parent machine's instances.
    // own: one instance per other type the states declare
    using inherited_contexts = internal::inherited_contexts_t<TRANSITION_TABLE>;
    using own_contexts =
        mtl::remove_if_t<internal::table_contexts_t<TRANSITIONS>,
                         internal::member_of<inherited_contexts>::template pred>;
    using context_types = mtl::concat_t<own_contexts, inherited_contexts>;

private:
    using own_context_tuple = mtl::rebind_t<own_contexts, std::tuple>;
    using inherited_context_tuple =
        mtl::rebind_t<mtl::transform_t<inherited_contexts, std::add_lvalue_reference>, std::tuple>;
    using context_tuple =
        decltype(std::tuple_cat(own_context_tuple{}, std::declval<inherited_context_tuple>()));

    using final_states    = typename TRANSITIONS::final_states;
    using emitting_states = mtl::filter_t<typename TRANSITIONS::states, internal::is_emitting>;

    using composites = mtl::filter_t<typename TRANSITIONS::states, internal::is_composite>;
    static constexpr bool has_composites = !mtl::empty_v<composites>;

    template<internal::composite COMPOSITE>
    using submachine_of =
        StateMachine<internal::nested<internal::submachine_t<COMPOSITE>, StateMachine::depth + 1,
                                      internal::parent_contexts_t<COMPOSITE>>,
                     OBSERVERs...>;

    template<internal::composite COMPOSITE>
    struct make_submachine : std::type_identity<submachine_of<COMPOSITE>> {};

    // At most one composite state is active: one alternative per
    // distinct child machine, std::monostate while none is
    using submachine_variant = mtl::rebind_t<
        mtl::prepend_t<std::monostate,
                       mtl::unique_t<mtl::transform_t<composites, make_submachine>>>,
        std::variant>;
    using submachine_storage =
        std::conditional_t<StateMachine::has_composites, submachine_variant, mtl::nil_type>;

public:
    explicit StateMachine(OBSERVERs&... observers)
        : observers_(observers...),
          current_(std::make_from_tuple<state_variant>(
              internal::initialArgs<initial_state>(contexts_)))
    {
        this->enterInitialState();
    }

    // A child machine, built by its parent with the contexts it inherits
    StateMachine(inherited_context_tuple const& inherited, OBSERVERs&... observers)
        : contexts_(std::tuple_cat(own_context_tuple{}, inherited)), observers_(observers...),
          current_(std::make_from_tuple<state_variant>(
              internal::initialArgs<initial_state>(contexts_)))
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
        return this->react(event) != reaction::none;
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
        return internal::contextOf<T>(contexts_);
    }

    // The child machine of COMPOSITE while that state is active,
    // nullptr otherwise
    template<internal::composite COMPOSITE>
    [[nodiscard]] submachine_of<COMPOSITE> const* submachine() const
    {
        static_assert(mtl::has_a_v<composites, COMPOSITE>,
                      "StateMachine::submachine: not a composite state of this table");
        return this->template is<COMPOSITE>() ? std::get_if<submachine_of<COMPOSITE>>(&sub_)
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
        if constexpr (StateMachine::has_composites) {
            this->template annotationOfSubmachine<T>(result);
        }
        return result;
    }

private:
    // --- an event arrives ---------------------------------------------------

    // The submachine first, then this machine's own table. A local
    // event (fsm::timeout) belongs to one level and does not descend
    template<concepts::event EVENT>
    reaction react(EVENT const& event)
    {
        this->beginProcessing();
        reaction result = reaction::none;
        if constexpr (!local_event_v<EVENT>) {
            result = this->reactInSubmachine(internal::passedDown(event));
        }
        if (result == reaction::none) {
            result = this->reactInOwnTable(event);
        }
        this->endProcessing();
        return result;
    }

    template<concepts::event EVENT>
    reaction reactInSubmachine([[maybe_unused]] EVENT const& event)
    {
        reaction result = reaction::none;
        [&]<typename... COMPOSITEs>(mtl::typelist<COMPOSITEs...>) {
            static_cast<void>(
                ((this->template is<COMPOSITEs>() &&
                  (result = this->template reactInSubmachineOf<COMPOSITEs>(event), true)) ||
                 ...));
        }(composites{});
        return result;
    }

    // Whatever the submachine does happens in place for this machine -
    // unless it entered a state whose emitted event moves this machine on
    template<internal::composite COMPOSITE, concepts::event EVENT>
    reaction reactInSubmachineOf(EVENT const& event)
    {
        reaction const of_submachine = this->template submachineOf<COMPOSITE>().react(event);
        if (of_submachine == reaction::state_entered &&
            this->template reactToEmittedEvent<COMPOSITE>() == reaction::state_entered) {
            return reaction::state_entered;
        }
        return of_submachine == reaction::none ? reaction::none : reaction::in_place;
    }

    // The event the submachine's active state emits goes to this
    // machine's own table. The reaction may destroy the submachine: the
    // fold stops at the first match
    template<internal::composite COMPOSITE>
    reaction reactToEmittedEvent()
    {
        reaction result = reaction::none;
        [&]<typename... EMITTINGs>(mtl::typelist<EMITTINGs...>) {
            auto const& submachine = this->template submachineOf<COMPOSITE>();
            static_cast<void>(
                ((submachine.template is<EMITTINGs>() &&
                  (result = this->reactInOwnTable(internal::emitted_t<EMITTINGs>{}), true)) ||
                 ...));
        }(typename submachine_of<COMPOSITE>::emitting_states{});
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
            ((this->template guardsHold<typename GUARDEDs::guards>(state, event) &&
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

    // A composite state's submachine is left before the state itself
    template<concepts::state OLD_STATE, concepts::state NEW_STATE>
    void leave()
    {
        this->template leaveSubmachine<OLD_STATE>();
        this->forEachObserver(
            [this](auto& observer) { internal::exitHook<OLD_STATE, NEW_STATE>(observer, *this); });
    }

    template<concepts::state NEW_STATE, typename... ARGs>
    void construct(ARGs const&... args)
    {
        [&]<typename... CONTEXTs>(mtl::typelist<CONTEXTs...>) {
            current_.template emplace<NEW_STATE>(args...,
                                                 internal::contextOf<CONTEXTs>(contexts_)...);
        }(internal::contexts_of_t<NEW_STATE>{});
    }

    template<concepts::state OLD_STATE, concepts::state NEW_STATE>
    void enter()
    {
        this->forEachObserver(
            [this](auto& observer) { internal::enterHook<OLD_STATE, NEW_STATE>(observer, *this); });
    }

    template<concepts::state FROM_STATE, concepts::event EVENT, concepts::state TO_STATE>
    void notifyTransition()
    {
        this->forEachObserver([this](auto& observer) {
            internal::transitionHook<FROM_STATE, EVENT, TO_STATE>(observer, *this);
        });
    }

    void enterInitialState()
    {
        this->beginProcessing();
        this->template enter<mtl::nil_type, initial_state>();
        this->template enterSubmachine<initial_state>();
        this->endProcessing();
    }

    // --- guards -------------------------------------------------------------

    template<concepts::transition TRANSITION, concepts::state STATE, concepts::event EVENT>
    bool allowed(STATE const& state, EVENT const& event)
    {
        if constexpr (internal::has_guard_v<TRANSITION>) {
            return this->template guardsHold<typename TRANSITION::guards>(state, event);
        } else {
            return true;
        }
    }

    template<mtl::concepts::typelist GUARDS, concepts::state STATE, concepts::event EVENT>
    bool guardsHold(STATE const& state, EVENT const& event)
    {
        return [&]<typename... PARTs>(mtl::typelist<PARTs...>) {
            return (this->template partHolds<PARTs>(state, event) && ...);
        }(GUARDS{});
    }

    template<concepts::guard_part PART, concepts::state STATE, concepts::event EVENT>
    bool partHolds(STATE const& state, EVENT const& event)
    {
        if constexpr (internal::is_negated_v<PART>) {
            return !this->template answerTo<internal::guard_of_t<PART>>(state, event);
        } else {
            return this->template answerTo<PART>(state, event);
        }
    }

    // The injected object answering GUARD decides, else the guard's own
    // static check
    template<concepts::guard GUARD, concepts::state STATE, concepts::event EVENT>
    bool answerTo(STATE const& state, EVENT const& event)
    {
        using answerer =
            mtl::find_if_t<injected, internal::answering<GUARD, STATE>::template pred>;
        if constexpr (std::is_same_v<answerer, mtl::nil_type>) {
            return internal::checkStaticGuard<GUARD>(state, event);
        } else {
            return internal::askGuard<GUARD>(
                std::get<mtl::index_of_v<answerer, injected>>(observers_), state, event);
        }
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
                ((this->template allowed<WILDCARDs>(state, event) &&
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
        this->forEachObserver([&](auto& observer) {
            this->template notifyEntered<EVENT, NEW_STATE>(observer, state_left);
        });
        this->forEachObserver([&](auto& observer) {
            this->template notifyTransitioned<EVENT, NEW_STATE>(observer, state_left);
        });
        this->template enterSubmachine<NEW_STATE>();
    }

    template<concepts::event EVENT, concepts::state NEW_STATE, concepts::observer OBSERVER>
    void notifyEntered(OBSERVER& observer, [[maybe_unused]] std::size_t state_left)
    {
        if constexpr (internal::has_enter<OBSERVER, NEW_STATE, StateMachine>) {
            observer.template onEnter<NEW_STATE>(*this);
        } else if constexpr (StateMachine::enters_from_some_state<OBSERVER, NEW_STATE>) {
            this->template forStateLeft<EVENT>(state_left, [&](auto tag) {
                internal::enterHook<typename decltype(tag)::type, NEW_STATE>(observer, *this);
            });
        }
    }

    // Whether OBSERVER has an edge-form entry hook for NEW_STATE from any
    // state: without one the switch on the state left has nothing to do
    template<concepts::observer OBSERVER, concepts::state NEW_STATE>
    static constexpr bool enters_from_some_state = []<typename... STATEs>(mtl::typelist<STATEs...>) {
        return (internal::has_enter_from<OBSERVER, STATEs, NEW_STATE, StateMachine> || ...);
    }(typename TRANSITIONS::states{});

    template<concepts::event EVENT, concepts::state NEW_STATE, concepts::observer OBSERVER>
    void notifyTransitioned(OBSERVER& observer, [[maybe_unused]] std::size_t state_left)
    {
        if constexpr (internal::has_transition<OBSERVER, EVENT, NEW_STATE, StateMachine>) {
            observer.template onTransition<EVENT, NEW_STATE>(*this);
        } else {
            this->template forStateLeft<EVENT>(state_left, [&](auto tag) {
                internal::transitionHook<typename decltype(tag)::type, EVENT, NEW_STATE>(observer,
                                                                                         *this);
            });
        }
    }

    // f(std::type_identity<STATE>{}) for the state at INDEX, among the
    // states a wildcard for EVENT can leave
    template<concepts::event EVENT, typename F>
    void forStateLeft(std::size_t index, F&& f)
    {
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(([&] {
                using STATE = std::variant_alternative_t<INDEXs, state_variant>;
                if constexpr (internal::wildcard_source_v<TRANSITIONS, STATE, EVENT>) {
                    if (index == INDEXs) {
                        f(std::type_identity<STATE>{});
                        return true;
                    }
                }
                return false;
            }() || ...));
        }(std::make_index_sequence<std::variant_size_v<state_variant>>{});
    }

    // --- submachines --------------------------------------------------------

    // After the composite state's own entry hooks: the child machine's
    // constructor enters its initial state
    template<concepts::state STATE>
    void enterSubmachine()
    {
        if constexpr (internal::composite<STATE>) {
            std::apply(
                [this](auto&... observer) {
                    sub_.template emplace<submachine_of<STATE>>(
                        this->template contextsInheritedBy<STATE>(), observer...);
                },
                observers_);
        }
    }

    template<concepts::state STATE>
    void leaveSubmachine()
    {
        if constexpr (internal::composite<STATE>) {
            this->template submachineOf<STATE>().leaveActiveState();
            sub_.template emplace<std::monostate>();
        }
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

    template<internal::composite COMPOSITE>
    submachine_of<COMPOSITE>& submachineOf()
    {
        auto* const child = std::get_if<submachine_of<COMPOSITE>>(&sub_);
#if MTL_FSM_CHECKS
        MTL_FSM_ASSERT(child != nullptr, "fsm: composite state active without its submachine");
#endif
        return *child;
    }

    // References to this machine's instances - own, or inherited in turn
    template<internal::composite COMPOSITE>
    auto contextsInheritedBy()
    {
        return [this]<typename... INHERITEDs>(mtl::typelist<INHERITEDs...>) {
            return std::tie(internal::contextOf<INHERITEDs>(contexts_)...);
        }(internal::parent_contexts_t<COMPOSITE>{});
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

    // Only the composite states with T somewhere below ask their child
    template<concepts::annotation T>
    void annotationOfSubmachine(std::optional<T>& annotation) const
    {
        using states  = typename TRANSITIONS::states;
        using nesting = mtl::filter_t<composites, internal::nesting_carrier<T>::template pred>;
        [&]<typename... NESTINGs>(mtl::typelist<NESTINGs...>) {
            static_cast<void>(
                ((current_.index() == mtl::index_of_v<NESTINGs, states> &&
                  (annotation = this->template submachine<NESTINGs>()->template annotation<T>(),
                   true)) ||
                 ...));
        }(nesting{});
    }

    // --- observers and the re-entrancy check --------------------------------

    template<typename F>
    void forEachObserver(F&& f)
    {
        this->forEachObserver(f, std::index_sequence_for<OBSERVERs...>{});
    }

    template<typename F, std::size_t... INDEXs>
    void forEachObserver(F& f, std::index_sequence<INDEXs...>)
    {
        (f(std::get<INDEXs>(observers_)), ...);
    }

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
            return (internal::validated<OBSERVERs, table>() && ...);
        } else {
            return true;
        }
    }
    static_assert(StateMachine::observersValidated());

    static_assert(mtl::all_of_v<typename TRANSITIONS::transitions,
                                internal::guard_answered_once_in<injected>::template pred>,
                  "StateMachine: a guard is answered by more than one injected object");
    static_assert(mtl::all_of_v<typename TRANSITIONS::transitions,
                                internal::guard_answered_in<injected>::template pred>,
                  "StateMachine: a guard of the table has no static check and no injected "
                  "object answers it - inject one with bool check(GUARD, FROM const&[, EVENT "
                  "const&]) or bool check(GUARD)");

    static_assert(mtl::all_of_v<own_contexts, std::is_default_constructible>,
                  "StateMachine: context types must be default constructible");
    static_assert(mtl::all_of_v<typename TRANSITIONS::states, internal::context_constructible>,
                  "StateMachine: a state must be constructible from its declared contexts "
                  "alone, in their order (default constructible without any)");

    template<internal::composite COMPOSITE>
    struct nests_this_table : std::is_same<internal::submachine_t<COMPOSITE>, table> {};
    static_assert(mtl::none_of_v<composites, nests_this_table>,
                  "StateMachine: a state's submachine is the table it belongs to");
    static_assert(mtl::all_of_v<composites, internal::annotation_levels_exclusive>,
                  "StateMachine: an annotation of a composite state may not recur in its "
                  "submachine - annotate at the level where the value changes");

    static_assert(
        mtl::all_of_v<typename TRANSITIONS::states, internal::parent_contexts_on_composite>,
        "StateMachine: parent_contexts is declared by a state without a submachine");
    static_assert(
        mtl::all_of_v<composites, internal::parent_contexts_held_in<context_types>::template pred>,
        "StateMachine: a submachine inherits a context its parent machine does not hold - a "
        "state of this table declares it, or this machine inherits it in turn");
    static_assert(mtl::all_of_v<composites, internal::parent_contexts_declared_in_submachine>,
                  "StateMachine: a submachine inherits a context no state of it (or of the "
                  "submachines inside it) declares");

    template<internal::emitting STATE>
    struct emits_default_constructible
        : std::is_default_constructible<internal::emitted_t<STATE>> {};
    static_assert(mtl::all_of_v<emitting_states, emits_default_constructible>,
                  "StateMachine: the event a state emits must be default constructible");
    static_assert(
        mtl::all_of_v<composites,
                      internal::emitted_events_taken_in<TRANSITIONS, injected>::template pred>,
        "StateMachine: a composite state has no transition for an event a state of its "
        "submachine emits");
    static_assert(
        mtl::all_of_v<composites, internal::submachine_starts_silent<injected>::template pred>,
        "StateMachine: the initial state of a submachine emits an event - nobody is there to "
        "take it yet");

    // --- data ---------------------------------------------------------------

    context_tuple contexts_{}; // the own instances, then the inherited references
    std::tuple<OBSERVERs&...> observers_;
#if MTL_FSM_CHECKS
    bool processing_ = false;
#endif
    state_variant current_;
    [[no_unique_address]] submachine_storage sub_{}; // the active composite state's child machine
};

} // namespace fsm
