/*
 * fsm: the timer policy contract and the observers driving it -
 * fsm::timed (state timeouts) and fsm::deadlined (phase deadlines)
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Timeout.hpp>
#include <mtl/TypelistAlgorithms.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

namespace fsm {

using timer_callback = void (*)(void*);

namespace concepts {

template<typename T>
concept timer = requires(T t, std::chrono::milliseconds duration,
                         timer_callback callback, void* context) {
    t.start(duration, callback, context);
    t.stop();
};

} // namespace concepts

namespace internal {

// A timer policy whose owner brings its expiries to the machine
// declares
//   static constexpr bool delivered_by_owner = true;
// fsm::QueuedTimer does: the queue it is bound to delivers an expiry
// itself, addressed by the level of the slot. The callback and context
// an observer would arm such a timer with are not used
template<typename TIMER>
concept delivered_by_owner = requires { requires TIMER::delivered_by_owner; };

// The expiry of the timer slot LEVEL, as EVENT for that level: it
// enters at the root, decorated once per level on the way up
// (fsm::for_level_t). The slot is known at run time, the decoration is
// a type - one compare per level
template<concepts::event EVENT, std::size_t LEVELS, typename ROOT>
void processForLevel(ROOT& root, std::size_t level)
{
    [&]<std::size_t... SLOTs>(std::index_sequence<SLOTs...>) {
        static_cast<void>(
            ((level == SLOTs && (root.process(for_level_t<EVENT, SLOTs>{}), true)) || ...));
    }(std::make_index_sequence<LEVELS>{});
}

// One TIMER per machine level, for the timer observers: a composite
// state and its active sub-state may both be timed, so a hierarchy
// of LEVELS levels needs LEVELS timers - the machine at nesting depth
// d arms slot d. The owning form default-constructs its timers,
// the reference form (TIMER a reference) takes one caller-owned timer
// per level, for policies that need configuration.
// An expiry is an event for one level and enters the machine at its
// root: a flat machine is its own root; with more levels the observer
// remembers the root when its initial state is entered - unless the
// timer's owner delivers
template<concepts::timer TIMER, std::size_t LEVELS>
class timer_slots {
    static_assert(LEVELS > 0, "fsm: a timer observer serves at least one level");

public:
    using timer_type = std::remove_reference_t<TIMER>;

    timer_slots()
        requires(!std::is_reference_v<TIMER>)
    = default;

    template<typename... TIMERs>
        requires(std::is_reference_v<TIMER> && sizeof...(TIMERs) == LEVELS &&
                 (std::is_same_v<TIMERs, timer_type> && ...))
    explicit timer_slots(TIMERs&... timers) : slots_{&timers...}
    {
    }

    timer_type& timer(std::size_t level = 0)
    {
        if constexpr (std::is_reference_v<TIMER>) {
            return *slots_[level];
        } else {
            return slots_[level];
        }
    }

protected:
    static constexpr bool remembers_root = LEVELS > 1 && !delivered_by_owner<timer_type>;

    // The root machine, as its initial state is entered: where the
    // expiries of every level go in
    template<concepts::event EVENT, typename ROOT>
    void rememberRoot([[maybe_unused]] ROOT& root)
    {
        if constexpr (timer_slots::remembers_root) {
            root_ = {.machine = &root, .expire = [](void* machine, std::size_t level) {
                         processForLevel<EVENT, LEVELS>(*static_cast<ROOT*>(machine), level);
                     }};
        }
    }

    // Arms the slot of MACHINE's level; the expiry is EVENT for that
    // level. One body per machine, the duration passed as a 32-bit
    // value: materializing the 64-bit chrono constant in every
    // per-state start measured ~40 bytes each on Thumb-1 (-Os, GCC 14)
    template<concepts::event EVENT, typename MACHINE>
    void arm(std::uint32_t duration_ms, [[maybe_unused]] MACHINE& machine)
    {
        auto const duration = std::chrono::milliseconds{duration_ms};
        if constexpr (delivered_by_owner<timer_type>) {
            this->timer(MACHINE::depth).start(duration, nullptr, nullptr);
        } else if constexpr (LEVELS == 1) {
            this->timer().start(
                duration,
                [](void* context) { static_cast<MACHINE*>(context)->process(EVENT{}); },
                &machine);
        } else {
            this->timer(MACHINE::depth).start(
                duration,
                [](void* self) {
                    auto& root = static_cast<timer_slots*>(self)->root_;
                    root.expire(root.machine, MACHINE::depth);
                },
                this);
        }
    }

private:
    using storage = std::conditional_t<std::is_reference_v<TIMER>,
                                       std::array<timer_type*, LEVELS>,
                                       std::array<timer_type, LEVELS>>;
    struct root_address {
        void* machine                                  = nullptr;
        void (*expire)(void* machine, std::size_t level) = nullptr;
    };

    storage slots_{};
    [[no_unique_address]] std::conditional_t<timer_slots::remembers_root, root_address,
                                             mtl::nil_type>
        root_{};
};

} // namespace internal

// Observer implementing the state-timeout semantics on top of a TIMER
// policy. timed<POLICY> owns default-constructed policy instances;
// timed<POLICY&> holds caller-owned ones, for policies that need
// configuration (constructor arguments) or are not default-constructible.
// LEVELS is the number of machine levels the observer serves - one
// timer each, fsm::levels_v<table> for a table with composite states
// (the default covers a flat table); timer(level) reads a slot
template<concepts::timer TIMER, std::size_t LEVELS = 1>
struct timed : internal::timer_slots<TIMER, LEVELS> {
    using internal::timer_slots<TIMER, LEVELS>::timer_slots;

    static constexpr std::size_t levels = LEVELS;

    // A timed state whose fsm::timeout the table ignores, or may refuse,
    // is a bug: the timer would fire into nothing, or the state would
    // sit there without its one-shot timer. Every level's states are
    // checked against their own table: a sub-state's timeout never
    // reaches the parent's rows
    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(LEVELS >= levels_v<TABLE>,
                      "fsm::timed: the table nests deeper than the observer has timers - "
                      "declare fsm::timed<TIMER, fsm::levels_v<table>>");
        static_assert(mtl::all_of_v<nested_tables_t<TABLE>, internal::timeouts_handled_in_table>,
                      "fsm::timed: a timed state needs an unguarded transition for fsm::timeout");
    }

    // Hooks of one state: leaving a timed state stops its timer, entering
    // one arms it - one instantiation per timed state, none per edge
    template<concepts::state STATE, typename MACHINE>
    void onExit(MACHINE&)
    {
        if constexpr (internal::has_timeout_v<STATE>) {
            this->timer(MACHINE::depth).stop(); // no timer may fire mid-transition
        }
    }

    template<concepts::state STATE, typename MACHINE>
    void onEnter(MACHINE& machine)
    {
        if constexpr (timed::remembers_root && MACHINE::depth == 0 &&
                      std::is_same_v<STATE, typename MACHINE::initial_state>) {
            this->template rememberRoot<timeout>(machine);
        }
        if constexpr (internal::has_timeout_v<STATE>) {
            constexpr auto duration =
                std::chrono::ceil<std::chrono::milliseconds>(STATE::timeout);
            static_assert(duration.count() > 0 &&
                              duration.count() <= std::numeric_limits<std::uint32_t>::max(),
                          "fsm::timed: timeout must be positive and within the 32-bit "
                          "millisecond range");
            this->template arm<timeout>(static_cast<std::uint32_t>(duration.count()), machine);
        }
    }
};

// Observer implementing phase deadlines on top of a TIMER policy: a
// hard time budget spanning several states. A state annotates
//
//   static constexpr auto deadline = <duration>;
//
// and entering it arms the timer - unless the state left carries the
// SAME nonzero value, which continues the running phase without
// re-arming, so bouncing between the phase's states cannot extend the
// budget. Entering a state without the annotation stops the clock; so
// does the zero-duration sentinel, which marks the phase target
// explicitly. Expiry injects fsm::deadline - distinct from
// fsm::timeout, and driven by its own TIMER instance, so a state may
// carry both a per-state timeout and a phase deadline.
// timed<POLICY>/timed<POLICY&> ownership and LEVELS semantics apply
template<concepts::timer TIMER, std::size_t LEVELS = 1>
struct deadlined : internal::timer_slots<TIMER, LEVELS> {
    using internal::timer_slots<TIMER, LEVELS>::timer_slots;

    static constexpr std::size_t levels = LEVELS;

    // A deadline the table ignores, or may refuse, is a bug: the timer
    // would fire into nothing, or the phase would outlive its budget
    // (the zero sentinel is exempt - it never arms)
    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(LEVELS >= levels_v<TABLE>,
                      "fsm::deadlined: the table nests deeper than the observer has timers - "
                      "declare fsm::deadlined<TIMER, fsm::levels_v<table>>");
        static_assert(mtl::all_of_v<nested_tables_t<TABLE>, internal::deadlines_handled_in_table>,
                      "fsm::deadlined: a state with a deadline needs an unguarded transition "
                      "for fsm::deadline");
    }

    // Whether the phase continues is a property of the edge (the state
    // left carries the same deadline): the edge hook, per source on a
    // wildcard
    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE>
    void onEnterFrom(MACHINE& machine)
    {
        if constexpr (deadlined::remembers_root && MACHINE::depth == 0 &&
                      std::is_same_v<OLD_STATE, mtl::nil_type>) {
            this->template rememberRoot<deadline>(machine); // the root is being constructed
        }
        if constexpr (internal::continues_deadline_v<OLD_STATE, NEW_STATE>) {
            // the phase's clock keeps running
        } else if constexpr (internal::active_deadline_v<NEW_STATE>) {
            constexpr auto duration =
                std::chrono::ceil<std::chrono::milliseconds>(NEW_STATE::deadline);
            static_assert(duration.count() > 0 &&
                              duration.count() <= std::numeric_limits<std::uint32_t>::max(),
                          "fsm::deadlined: deadline must be positive and within the 32-bit "
                          "millisecond range");
            this->template arm<deadline>(static_cast<std::uint32_t>(duration.count()), machine);
        } else if constexpr (internal::active_deadline_v<OLD_STATE>) {
            this->timer(MACHINE::depth).stop(); // left the phase: unannotated or the target
        }
    }
};

} // namespace fsm
