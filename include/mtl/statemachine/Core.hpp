/*
 * fsm: the state machine itself and its dispatch
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
template<typename VISITOR, typename... ALTERNATIVEs>
    requires (std::invocable<VISITOR, ALTERNATIVEs&> && ...)
constexpr auto visit(VISITOR&& visitor, std::variant<ALTERNATIVEs...>& variant)
{
    using result_type = std::common_type_t<std::invoke_result_t<VISITOR, ALTERNATIVEs&>...>;
    return [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
        result_type result{};
        static_cast<void>(((variant.index() == INDEXs &&
                            (result = visitor(*std::get_if<INDEXs>(&variant)), true)) ||
                           ...));
        return result;
    }(std::index_sequence_for<ALTERNATIVEs...>{});
}

// The fold is the default: it wins clearly on embedded targets with
// large machines (measurements above), losing only a few bytes on
// hosted libstdc++ with small ones. Define MTL_FSM_FOLD_VISIT to 0 to
// use std::visit instead.
#ifndef MTL_FSM_FOLD_VISIT
#  define MTL_FSM_FOLD_VISIT 1
#endif

template<typename VISITOR, typename... ALTERNATIVEs>
    requires (std::invocable<VISITOR, ALTERNATIVEs&> && ...)
constexpr auto dispatch(VISITOR&& visitor, std::variant<ALTERNATIVEs...>& variant)
{
#if MTL_FSM_FOLD_VISIT
    return internal::visit(std::forward<VISITOR>(visitor), variant);
#else
    return std::visit(std::forward<VISITOR>(visitor), variant);
#endif
}

} // namespace internal

template<concepts::transition_table TRANSITION_TABLE, typename... OBSERVERs>
class StateMachine {
    using TRANSITIONS = TRANSITION_TABLE;

    // A parent machine leaves its child through the child's private
    // leave path
    template<concepts::transition_table, typename...>
    friend class StateMachine;

public:
    // The user's table: named tables identify the machine (fsm::tracing).
    // A child machine is built from internal::nested<TABLE, DEPTH>, the
    // table plus its nesting depth - stripped here
    using table         = internal::plain_table_t<TRANSITION_TABLE>;
    using state_variant = mtl::rebind_t<typename TRANSITIONS::states, std::variant>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

    // Nesting depth of this machine: 0 for a root, one more per level of
    // composite states above it. The timer observers pick their timer
    // slot by it
    static constexpr std::size_t depth = internal::table_depth_v<TRANSITION_TABLE>;

private:
    // Observers get a chance to reject the table at compile time - once,
    // at the root, with the whole hierarchy in view: a table-wide proof
    // (an observed annotation, a timed state's timeout row) walks the
    // nested tables itself, and would be wrong on a sub-table alone
    static constexpr bool observersValidated()
    {
        if constexpr (StateMachine::depth == 0) {
            return (internal::validated<OBSERVERs, table>() && ...);
        } else {
            return true;
        }
    }
    static_assert(StateMachine::observersValidated());

    // Every guard of the table is answered: by exactly one injected
    // object with a check(GUARD, ...) overload, or by a static check of
    // the guard itself (an injected answer wins over the static one)
    using injected = mtl::typelist<OBSERVERs...>;
    static_assert(mtl::all_of_v<typename TRANSITIONS::transitions,
                                internal::guard_answered_once_in<injected>::template pred>,
                  "StateMachine: a guard is answered by more than one injected object");
    static_assert(mtl::all_of_v<typename TRANSITIONS::transitions,
                                internal::guard_answered_in<injected>::template pred>,
                  "StateMachine: a guard of the table has no static check and no injected "
                  "object answers it - inject one with bool check(GUARD, FROM const&[, EVENT "
                  "const&]) or bool check(GUARD)");

    // --- hierarchy ----------------------------------------------------------
    // The composite states of this table each own a child machine of
    // their submachine table, alive while the state is: one variant
    // next to the state variant holds the active one (at most one
    // composite state is active), std::monostate while none is
    using composites = mtl::filter_t<typename TRANSITIONS::states, internal::is_composite>;
    static constexpr bool has_composites = !mtl::empty_v<composites>;

    template<typename STATE>
    struct nests_this_table : std::is_same<internal::submachine_t<STATE>, table> {};
    static_assert(mtl::none_of_v<composites, nests_this_table>,
                  "StateMachine: a state's submachine is the table it belongs to");
    static_assert(mtl::all_of_v<composites, internal::annotation_levels_exclusive>,
                  "StateMachine: an annotation of a composite state may not recur in its "
                  "submachine - annotate at the level where the value changes");

    template<typename STATE>
    using submachine_of =
        StateMachine<internal::nested<internal::submachine_t<STATE>, StateMachine::depth + 1>,
                     OBSERVERs...>;

    template<typename STATE>
    struct make_submachine : std::type_identity<submachine_of<STATE>> {};

    using submachine_variant = mtl::rebind_t<
        mtl::prepend_t<std::monostate, mtl::transform_t<composites, make_submachine>>,
        std::variant>;
    using submachine_storage =
        std::conditional_t<StateMachine::has_composites, submachine_variant, mtl::nil_type>;

public:
    // Every context type any state declares, deduplicated: states naming
    // the same type share one instance
    using context_types = mtl::unique_t<
        mtl::linearize_t<mtl::transform_t<typename TRANSITIONS::states, internal::contexts_of>>>;
    using context_tuple = mtl::rebind_t<context_types, std::tuple>;

    static_assert(mtl::all_of_v<context_types, std::is_default_constructible>,
                  "StateMachine: context types must be default constructible");
    static_assert(mtl::all_of_v<typename TRANSITIONS::states, internal::context_constructible>,
                  "StateMachine: a state must be constructible from its declared contexts "
                  "alone, in their order (default constructible without any)");

public:
    explicit StateMachine(OBSERVERs&... observers)
        : observers_(observers...),
          current_(std::make_from_tuple<state_variant>(
              internal::initialArgs<initial_state>(contexts_)))
    {
        this->beginProcessing();
        this->template enter<mtl::nil_type, initial_state>();
        this->template enterSubmachine<initial_state>();
        this->endProcessing();
    }

    // Observer hooks receive *this and may retain the address beyond the
    // hook: the machine must stay at one address for its lifetime
    StateMachine(StateMachine const&)            = delete;
    StateMachine& operator=(StateMachine const&) = delete;

    // Returns true if a transition fired (false: no matching transition, or
    // every alternative's guard said no).
    // A from<any_state> transition changes the state through one shared
    // body per (event, target) - expanding whole edges per source would
    // emit one near-identical transition body per state (measured
    // kilobytes in a machine with a handful of wildcard events). Its
    // guards and exit hooks run in the arm, where the state left is
    // known; only an edge-form entry or transition hook is delivered per
    // source afterwards, through a switch on the state left.
    // A composite state offers the event to its submachine first: what
    // the active sub-state handles counts as fired here without
    // touching this level; what it does not - no row, or every guard
    // refused - is tried against this state's own alternatives. A local
    // event (fsm::timeout, fsm::deadline) is addressed to this machine
    // and never descends
    template<typename EVENT>
    bool process(EVENT const& event)
    {
        using wildcards = wildcard_transitions_t<TRANSITIONS, EVENT>;
        this->beginProcessing();
        std::size_t const outcome = internal::dispatch(
            [this, &event](auto& state) -> std::size_t {
                using state_type = std::decay_t<decltype(state)>;
                using own        = exact_transitions_t<TRANSITIONS, state_type, EVENT>;
                using guarded    = mtl::filter_t<own, internal::is_guarded>;
                using unguarded  = mtl::find_if_t<own, internal::is_unguarded>;
                // 0. the active sub-state's own chance
                if constexpr (internal::composite<state_type> && !local_event_v<EVENT>) {
                    if (this->template submachineOf<state_type>().process(event)) {
                        return StateMachine::fired;
                    }
                }
                // 1. the state's guarded alternatives in table order
                if constexpr (!mtl::empty_v<guarded>) {
                    if (this->template tryGuarded<state_type>(guarded{}, state, event)) {
                        return StateMachine::fired;
                    }
                }
                // 2. its unguarded catch-all always fires
                if constexpr (!std::is_same_v<unguarded, mtl::nil_type>) {
                    this->template doTransition<unguarded>(state, event);
                    return StateMachine::fired;
                }
                // 3. the wildcards, behind the state's own alternatives:
                //    the first whose guard passes is left here, fired below
                else if constexpr (!mtl::empty_v<wildcards>) {
                    return this->template leaveForWildcard<state_type>(wildcards{}, state, event);
                }
                // 4. nothing: the event is ignored (these arms emit no code)
                else {
                    return StateMachine::ignored;
                }
            },
            current_);
        bool changed = outcome == StateMachine::fired;
        if constexpr (!mtl::empty_v<wildcards>) {
            if (outcome >= StateMachine::pending) {
                // the state left is still the current one: its index is
                // the source the edge-form hooks may ask for
                this->fireWildcard(outcome - StateMachine::pending, current_.index(),
                                   wildcards{}, event);
                changed = true;
            }
        }
        this->endProcessing();
        return changed;
    }

    // Is STATE the active state?
    template<concepts::state STATE>
    [[nodiscard]] bool is() const
    {
        return std::holds_alternative<STATE>(current_);
    }

    // Pointer to the active state object, nullptr if STATE is not active.
    // The next transition destroys the object: do not keep the pointer.
    // Read-only: every mutation goes through process() - a state changes
    // itself in an internal transition
    template<concepts::state STATE>
    [[nodiscard]] STATE const* getIf() const
    {
        return std::get_if<STATE>(&current_);
    }

    // Read-only view of the machine-owned context instance of type T
    // (one some state declares as its context member). Observation
    // only, like getIf(): every mutation goes through process() - a
    // state writes its context, seeded by event payload if needed
    template<typename T>
    [[nodiscard]] T const& context() const
    {
        return std::get<T>(contexts_);
    }

    // The child machine of the composite state STATE while STATE is
    // active, nullptr otherwise - getIf() for the nested level. Read-only
    // like the parent: events enter at the root and descend
    template<internal::composite STATE>
    [[nodiscard]] submachine_of<STATE> const* submachine() const
    {
        static_assert(mtl::has_a_v<composites, STATE>,
                      "StateMachine::submachine: not a composite state of this table");
        return std::get_if<submachine_of<STATE>>(&sub_);
    }

    // The active state's annotation element of type T - its static
    // fsm::annotate set - empty while the active state carries no T.
    // A facade asks the machine what the observers see instead of
    // enumerating states with is<>(): the states declare the fact,
    // the query reads it. An element no state of the table carries
    // could never be answered: a static_assert. Instance values
    // (values()) stay with the state object, see getIf().
    // Over a hierarchy the answer comes from the one level carrying T
    // along the active path (annotation_levels_exclusive): this level's
    // state, or else the active submachine's
    template<typename T>
    [[nodiscard]] std::optional<T> annotation() const
    {
        static_assert(annotation_in_table_v<table, T>,
                      "StateMachine::annotation: no state of the table carries this annotation");
        // only the states carrying T take part: a compare chain over
        // the carriers, each yielding its constant
        using states   = typename TRANSITIONS::states;
        using carriers = mtl::filter_t<states, internal::carrying<T>::template pred>;
        std::optional<T> result = [this]<typename... CARRIERs>(mtl::typelist<CARRIERs...>) {
            std::optional<T> own;
            static_cast<void>(((current_.index() == mtl::index_of_v<CARRIERs, states> &&
                                (own = CARRIERs::annotations.template get<T>(), true)) ||
                               ...));
            return own;
        }(carriers{});
        if constexpr (StateMachine::has_composites) {
            // ... and the composites whose hierarchy carries T ask their child
            using nesting = mtl::filter_t<composites, internal::nesting_carrier<T>::template pred>;
            [this, &result]<typename... NESTINGs>(mtl::typelist<NESTINGs...>) {
                static_cast<void>(((current_.index() == mtl::index_of_v<NESTINGs, states> &&
                                    (result = this->template submachine<NESTINGs>()
                                                  ->template annotation<T>(),
                                     true)) ||
                                   ...));
            }(nesting{});
        }
        return result;
    }

private:
    // --- wildcards ----------------------------------------------------------

    // The visitor's outcome: an own alternative fired, nothing applies,
    // or wildcard alternative (outcome - pending) is left and to be fired
    static constexpr std::size_t ignored = 0;
    static constexpr std::size_t fired   = 1;
    static constexpr std::size_t pending = 2;

    // In the arm of the state left: the first wildcard whose guard passes
    // (guards see the real state) has its exit hooks run here
    template<typename STATE, typename... WILDCARDs, typename EVENT>
    std::size_t leaveForWildcard(mtl::typelist<WILDCARDs...>, STATE& state, EVENT const& event)
    {
        std::size_t outcome = StateMachine::ignored;
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(
                ((this->template allowed<WILDCARDs>(state, event) &&
                  (this->template leave<STATE, typename WILDCARDs::to>(),
                   outcome = StateMachine::pending + INDEXs, true)) ||
                 ...));
        }(std::index_sequence_for<WILDCARDs...>{});
        return outcome;
    }

    // After the dispatch: the chosen alternative's shared body
    template<typename... WILDCARDs, typename EVENT>
    void fireWildcard(std::size_t chosen, std::size_t source, mtl::typelist<WILDCARDs...>,
                      EVENT const& event)
    {
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(((chosen == INDEXs &&
                                (this->template changeShared<WILDCARDs>(source, event), true)) ||
                               ...));
        }(std::index_sequence_for<WILDCARDs...>{});
    }

    // One shared body per (event, target): the change, then the entry and
    // transition hooks - the one-state forms once, an edge form per
    // possible source through the switch on the state left
    template<typename TRANSITION, typename EVENT>
    void changeShared(std::size_t source, EVENT const& event)
    {
        using NEW_STATE = typename TRANSITION::to;
        if constexpr (internal::payload_constructible_v<NEW_STATE, EVENT>) {
            this->template construct<NEW_STATE>(event);
        } else {
            this->template construct<NEW_STATE>();
        }
        this->forEachObserver([&](auto& observer) {
            this->template enterFromSource<EVENT, NEW_STATE>(observer, source);
        });
        this->forEachObserver([&](auto& observer) {
            this->template transitionFromSource<EVENT, NEW_STATE>(observer, source);
        });
        this->template enterSubmachine<NEW_STATE>();
    }

    // The switch on the state left: f(std::type_identity<STATE>{}) for
    // the state at INDEX. Only the states that can reach a wildcard for
    // EVENT get an arm; arms whose f is empty cost nothing
    template<typename EVENT, typename F>
    void withSource(std::size_t index, F&& f)
    {
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(([&] {
                using SOURCE = std::variant_alternative_t<INDEXs, state_variant>;
                if constexpr (internal::wildcard_source_v<TRANSITIONS, SOURCE, EVENT>) {
                    if (index == INDEXs) {
                        f(std::type_identity<SOURCE>{});
                        return true;
                    }
                }
                return false;
            }() || ...));
        }(std::make_index_sequence<std::variant_size_v<state_variant>>{});
    }

    template<typename EVENT, typename NEW_STATE, typename OBSERVER>
    void enterFromSource(OBSERVER& observer, [[maybe_unused]] std::size_t source)
    {
        if constexpr (internal::has_enter<OBSERVER, NEW_STATE, StateMachine>) {
            observer.template onEnter<NEW_STATE>(*this);
        } else {
            this->template withSource<EVENT>(source, [&](auto tag) {
                internal::enterHook<typename decltype(tag)::type, NEW_STATE>(observer, *this);
            });
        }
    }

    template<typename EVENT, typename NEW_STATE, typename OBSERVER>
    void transitionFromSource(OBSERVER& observer, [[maybe_unused]] std::size_t source)
    {
        if constexpr (internal::has_transition<OBSERVER, EVENT, NEW_STATE, StateMachine>) {
            observer.template onTransition<EVENT, NEW_STATE>(*this);
        } else {
            this->template withSource<EVENT>(source, [&](auto tag) {
                internal::transitionHook<typename decltype(tag)::type, EVENT, NEW_STATE>(observer,
                                                                                         *this);
            });
        }
    }

    // --- per-edge path ------------------------------------------------------

    // First guarded alternative whose guard passes fires; false when
    // none does. The fold short-circuits after a firing: the state
    // reference is dangling from that point on
    template<typename STATE, typename... GUARDEDs, typename EVENT>
    bool tryGuarded(mtl::typelist<GUARDEDs...>, STATE& state, EVENT const& event)
    {
        return ((this->template checkGuards<typename GUARDEDs::guards>(state, event) &&
                 (this->template doTransition<GUARDEDs>(state, event), true)) ||
                ...);
    }

    // A row's condition: every part in order, short-circuit; a not_<G>
    // part is G's answer inverted
    template<typename GUARDS, typename STATE, typename EVENT>
    bool checkGuards(STATE const& state, EVENT const& event)
    {
        return [&]<typename... PARTs>(mtl::typelist<PARTs...>) {
            return (this->template checkPart<PARTs>(state, event) && ...);
        }(GUARDS{});
    }

    template<typename PART, typename STATE, typename EVENT>
    bool checkPart(STATE const& state, EVENT const& event)
    {
        if constexpr (internal::is_negated_v<PART>) {
            return !this->template checkGuard<internal::guard_of_t<PART>>(state, event);
        } else {
            return this->template checkGuard<PART>(state, event);
        }
    }

    // One guard's answer: from the injected object answering it, else
    // from its own static check
    template<typename GUARD, typename STATE, typename EVENT>
    bool checkGuard(STATE const& state, EVENT const& event)
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

    // True when TRANSITION may fire from the given state instance
    template<typename TRANSITION, typename STATE, typename EVENT>
    bool allowed(STATE const& state, EVENT const& event)
    {
        if constexpr (internal::has_guard_v<TRANSITION>) {
            return this->template checkGuards<typename TRANSITION::guards>(state, event);
        } else {
            return true;
        }
    }

    // Already instantiated per (transition, state, event): the only
    // place the per-edge bodies below can stay event-agnostic while the
    // onTransition hook still learns the event. After the emplace the
    // state reference is dead - the hook only ever receives the machine
    template<typename TRANSITION, typename STATE, typename EVENT>
    void doTransition(STATE& state, EVENT const& event)
    {
        using TO_STATE = typename TRANSITION::to;
        if constexpr (internal::is_internal_v<TRANSITION>) {
            static_assert(requires { state.handle(event); },
                          "internal transition: the state must provide handle(EVENT const&)");
            state.handle(event);
        } else if constexpr (internal::payload_constructible_v<TO_STATE, EVENT>) {
            this->template changeState<STATE, TO_STATE>(event);
        } else {
            // the construction does not depend on the event: one body per edge
            this->template changeState<STATE, TO_STATE>();
        }
        this->template notifyTransition<STATE, EVENT, TO_STATE>();
        this->template enterSubmachine<TO_STATE>(); // never for internal_target
    }

    // Runs after the transition completed (new state constructed and
    // entered), so a trace line follows the effects of the change
    template<typename FROM_STATE, typename EVENT, typename TO_STATE>
    void notifyTransition()
    {
        this->forEachObserver([this](auto& observer) {
            internal::transitionHook<FROM_STATE, EVENT, TO_STATE>(observer, *this);
        });
    }

    // Leave OLD_STATE, construct NEW_STATE, enter it. With payload:
    // instantiated per (edge, event), only for targets constructible
    // from the event
    template<typename OLD_STATE, typename NEW_STATE, typename EVENT>
    void changeState(EVENT const& event)
    {
        this->template leave<OLD_STATE, NEW_STATE>();
        this->template construct<NEW_STATE>(event);
        this->template enter<OLD_STATE, NEW_STATE>();
    }

    // Event-independent construction: instantiated once per edge and
    // shared by all events triggering it
    template<typename OLD_STATE, typename NEW_STATE>
    void changeState()
    {
        this->template leave<OLD_STATE, NEW_STATE>();
        this->template construct<NEW_STATE>();
        this->template enter<OLD_STATE, NEW_STATE>();
    }

    // Constructs NEW_STATE in place, its contexts appended in declared order
    template<typename NEW_STATE, typename... ARGs>
    void construct(ARGs const&... args)
    {
        [&]<typename... CONTEXTs>(mtl::typelist<CONTEXTs...>) {
            current_.template emplace<NEW_STATE>(args..., std::get<CONTEXTs>(contexts_)...);
        }(internal::contexts_of_t<NEW_STATE>{});
    }

    // Innermost first: a composite's child is left before the state itself
    template<typename OLD_STATE, typename NEW_STATE>
    void leave()
    {
        this->template leaveSubmachine<OLD_STATE>();
        this->forEachObserver(
            [this](auto& observer) { internal::exitHook<OLD_STATE, NEW_STATE>(observer, *this); });
    }

    template<typename OLD_STATE, typename NEW_STATE>
    void enter()
    {
        this->forEachObserver(
            [this](auto& observer) { internal::enterHook<OLD_STATE, NEW_STATE>(observer, *this); });
    }

    // --- hierarchy ----------------------------------------------------------

    // Entering a composite state constructs its child machine, after the
    // state's own entry and transition hooks ran: the child's initial
    // state is entered by the child's constructor, hooks included, so a
    // trace reads parent line, then child line. Leaving a composite
    // state leaves the child's active state through the child's private
    // leave path - its exit hooks, then the destruction - before the
    // state's own exit hooks
    template<typename STATE>
    void enterSubmachine()
    {
        if constexpr (internal::composite<STATE>) {
            std::apply(
                [this](auto&... observer) {
                    sub_.template emplace<submachine_of<STATE>>(observer...);
                },
                observers_);
        }
    }

    template<typename STATE>
    void leaveSubmachine()
    {
        if constexpr (internal::composite<STATE>) {
            this->template submachineOf<STATE>().leaveCurrent();
            sub_.template emplace<std::monostate>();
        }
    }

    // The counterpart of the constructor's entry: the active state is
    // left once with TO = mtl::nil_type, its own submachine first. Only
    // a parent calls it; a root machine's destructor runs no hooks
    void leaveCurrent()
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

    // The live child of the active composite state STATE
    template<typename STATE>
    submachine_of<STATE>& submachineOf()
    {
        auto* const child = std::get_if<submachine_of<STATE>>(&sub_);
#if MTL_FSM_CHECKS
        MTL_FSM_ASSERT(child != nullptr, "fsm: composite state active without its submachine");
#endif
        return *child;
    }

    // Every observer in injection order
    template<typename F>
    void forEachObserver(F&& f)
    {
        std::apply([&](auto&... observer) { (f(observer), ...); }, observers_);
    }

    // The debug checks (MTL_FSM_CHECKS) around every run of hooks
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

    context_tuple contexts_{}; // one shared instance per distinct context type
    std::tuple<OBSERVERs&...> observers_;
#if MTL_FSM_CHECKS
    bool processing_ = false;
#endif
    state_variant current_; // constructed by the constructor via initialArgs()
    // the active composite state's child machine; nil_type in a flat table
    [[no_unique_address]] submachine_storage sub_{};
};

} // namespace fsm
