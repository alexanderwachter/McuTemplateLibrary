/*
 * fsm: the state machine itself and its dispatch
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Observer.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

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
class state_machine {
    using TRANSITIONS = TRANSITION_TABLE;

    // Observers get a chance to reject the table at compile time
    static_assert((internal::validated<OBSERVERs, TRANSITIONS>() && ...));

public:
    using table         = TRANSITION_TABLE; // named tables identify the machine (fsm::tracing)
    using state_variant = mtl::rebind_t<typename TRANSITIONS::states, std::variant>;
    using initial_state = mtl::front_t<typename TRANSITIONS::states>;

private:
    using context_states = mtl::filter_t<typename TRANSITIONS::states, internal::has_context>;
    // Deduplicated: states naming the same context type share one instance
    using context_types = mtl::unique_t<mtl::transform_t<context_states, internal::context_of>>;
    using context_tuple = mtl::rebind_t<context_types, std::tuple>;

    static_assert(mtl::all_of_v<context_types, std::is_default_constructible>,
                  "state_machine: context types must be default constructible");

    template<typename STATE>
    struct constructible_from_context
        : std::bool_constant<std::constructible_from<STATE, internal::context_of_t<STATE>&>> {};
    static_assert(mtl::all_of_v<context_states, constructible_from_context>,
                  "state_machine: a context state must be constructible from its context alone");

public:
    explicit state_machine(OBSERVERs&... observers)
        : observers_(observers...),
          current_(std::make_from_tuple<state_variant>(
              internal::initialArgs<initial_state>(contexts_)))
    {
        this->template enter<mtl::nil_type, initial_state>();
    }

    // Observer hooks receive *this and may retain the address beyond the
    // hook: the machine must stay at one address for its lifetime
    state_machine(state_machine const&)            = delete;
    state_machine& operator=(state_machine const&) = delete;

    // Returns true if a transition fired (false: no matching transition, or
    // every alternative's guard said no).
    // A from<any_state> transition changes the state through one shared
    // body per (event, target) - expanding whole edges per source would
    // emit one near-identical transition body per state (measured
    // kilobytes in a machine with a handful of wildcard events). Its
    // guards and exit hooks run in the arm, where the state left is
    // known; only an edge-form entry or transition hook is delivered per
    // source afterwards, through a switch on the state left
    template<typename EVENT>
    bool process(EVENT const& event)
    {
        using wildcards = wildcard_transitions_t<TRANSITIONS, EVENT>;
        std::size_t const outcome = internal::dispatch(
            [this, &event](auto& state) -> std::size_t {
                using state_type = std::decay_t<decltype(state)>;
                using own        = exact_transitions_t<TRANSITIONS, state_type, EVENT>;
                using guarded    = mtl::filter_t<own, internal::is_guarded>;
                using unguarded  = mtl::find_if_t<own, internal::is_unguarded>;
                // 1. the state's guarded alternatives in table order
                if constexpr (!mtl::empty_v<guarded>) {
                    if (this->template tryGuarded<state_type>(guarded{}, state, event)) {
                        return fired;
                    }
                }
                // 2. its unguarded catch-all always fires
                if constexpr (!std::is_same_v<unguarded, mtl::nil_type>) {
                    this->template doTransition<unguarded>(state, event);
                    return fired;
                }
                // 3. the wildcards, behind the state's own alternatives:
                //    the first whose guard passes is left here, fired below
                else if constexpr (!mtl::empty_v<wildcards>) {
                    return this->template leaveForWildcard<state_type>(wildcards{}, state, event);
                }
                // 4. nothing: the event is ignored (these arms emit no code)
                else {
                    return ignored;
                }
            },
            current_);
        if constexpr (!mtl::empty_v<wildcards>) {
            if (outcome >= pending) {
                this->fireWildcard(outcome - pending, wildcards{}, event);
                return true;
            }
        }
        return outcome == fired;
    }

    // Is STATE the active state?
    template<concepts::state STATE>
    [[nodiscard]] bool is() const
    {
        return std::holds_alternative<STATE>(current_);
    }

    // Pointer to the active state object, nullptr if STATE is not active.
    // The next transition destroys the object: do not keep the pointer.
    template<concepts::state STATE>
    [[nodiscard]] STATE* getIf()
    {
        return std::get_if<STATE>(&current_);
    }

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
        std::size_t outcome = ignored;
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(
                ((internal::allowed<WILDCARDs>(state, event) &&
                  (this->template leave<STATE, typename WILDCARDs::to>(), outcome = pending + INDEXs,
                   true)) ||
                 ...));
        }(std::index_sequence_for<WILDCARDs...>{});
        return outcome;
    }

    // After the dispatch: the chosen alternative's shared body
    template<typename... WILDCARDs, typename EVENT>
    void fireWildcard(std::size_t chosen, mtl::typelist<WILDCARDs...>, EVENT const& event)
    {
        [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            static_cast<void>(((chosen == INDEXs &&
                                (this->template changeShared<WILDCARDs>(event), true)) ||
                               ...));
        }(std::index_sequence_for<WILDCARDs...>{});
    }

    // One shared body per (event, target): the change, then the entry and
    // transition hooks - the one-state forms once, an edge form per
    // possible source through the switch on the state left
    template<typename TRANSITION, typename EVENT>
    void changeShared(EVENT const& event)
    {
        using NEW_STATE = typename TRANSITION::to;
        source_         = current_.index();
        if constexpr (internal::payload_constructible_v<NEW_STATE, EVENT>) {
            this->template construct<NEW_STATE>(event);
        } else {
            this->template construct<NEW_STATE>();
        }
        this->forEachObserver([&](auto& observer) {
            this->template enterFromSource<EVENT, NEW_STATE>(observer);
        });
        this->forEachObserver([&](auto& observer) {
            this->template transitionFromSource<EVENT, NEW_STATE>(observer);
        });
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
    void enterFromSource(OBSERVER& observer)
    {
        if constexpr (internal::has_enter<OBSERVER, NEW_STATE, state_machine>) {
            observer.template onEnter<NEW_STATE>(*this);
        } else {
            this->template withSource<EVENT>(source_, [&](auto tag) {
                internal::enterHook<typename decltype(tag)::type, NEW_STATE>(observer, *this);
            });
        }
    }

    template<typename EVENT, typename NEW_STATE, typename OBSERVER>
    void transitionFromSource(OBSERVER& observer)
    {
        if constexpr (internal::has_transition<OBSERVER, EVENT, NEW_STATE, state_machine>) {
            observer.template onTransition<EVENT, NEW_STATE>(*this);
        } else {
            this->template withSource<EVENT>(source_, [&](auto tag) {
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
        bool fired = false;
        static_cast<void>(((internal::checkGuard<typename GUARDEDs::guard>(state, event) &&
                            (fired = this->template doTransition<GUARDEDs>(state, event), true)) ||
                           ...));
        return fired;
    }

    // Already instantiated per (transition, state, event): the only
    // place the per-edge bodies below can stay event-agnostic while the
    // onTransition hook still learns the event. After the emplace the
    // state reference is dead - the hook only ever receives the machine
    template<typename TRANSITION, typename STATE, typename EVENT>
    bool doTransition(STATE& state, EVENT const& event)
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
        return true;
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

    // Constructs NEW_STATE in place, its context appended when it holds one
    template<typename NEW_STATE, typename... ARGs>
    void construct(ARGs const&... args)
    {
        if constexpr (internal::context_holder<NEW_STATE>) {
            current_.template emplace<NEW_STATE>(
                args..., std::get<internal::context_of_t<NEW_STATE>>(contexts_));
        } else {
            current_.template emplace<NEW_STATE>(args...);
        }
    }

    template<typename OLD_STATE, typename NEW_STATE>
    void leave()
    {
        this->forEachObserver(
            [this](auto& observer) { internal::exitHook<OLD_STATE, NEW_STATE>(observer, *this); });
    }

    template<typename OLD_STATE, typename NEW_STATE>
    void enter()
    {
        this->forEachObserver(
            [this](auto& observer) { internal::enterHook<OLD_STATE, NEW_STATE>(observer, *this); });
    }

    // Every observer in injection order
    template<typename F>
    void forEachObserver(F&& f)
    {
        std::apply([&](auto&... observer) { (f(observer), ...); }, observers_);
    }

    context_tuple contexts_{}; // one shared instance per distinct context type
    std::tuple<OBSERVERs&...> observers_;
    std::size_t source_ = 0; // set by a wildcard's shared body before the change
    state_variant current_; // constructed by the constructor via initialArgs()
};

} // namespace fsm
