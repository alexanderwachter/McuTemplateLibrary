/*
 * fsm: the timer policy contract and the observers driving it -
 * fsm::timed (state timeouts) and fsm::deadlined (phase deadlines)
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Timeout.hpp>
#include <mtl/TypelistAlgorithms.hpp>

#include <chrono>
#include <cstdint>
#include <limits>
#include <type_traits>

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

// Observer implementing the state-timeout semantics on top of a TIMER
// policy. timed<POLICY> owns a default-constructed policy instance;
// timed<POLICY&> holds a caller-owned one, for policies that need
// configuration (constructor arguments) or are not default-constructible
template<concepts::timer TIMER>
struct timed {
    timed()
        requires(!std::is_reference_v<TIMER>)
    = default;

    explicit timed(TIMER timer_ref)
        requires std::is_reference_v<TIMER>
        : timer(timer_ref)
    {
    }
    // A timed state whose fsm::timeout the table ignores is a bug: the
    // timer would fire into nothing
    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(mtl::all_of_v<typename TABLE::states,
                          internal::timeout_handled_in<TABLE>::template pred>,
                      "fsm::timed: state has a timeout but no transition for fsm::timeout");
    }

    // Hooks of one state: leaving a timed state stops its timer, entering
    // one arms it - one instantiation per timed state, none per edge
    template<typename STATE, typename MACHINE>
    void onExit(MACHINE&)
    {
        if constexpr (internal::has_timeout_v<STATE>) {
            timer.stop(); // no timer may fire mid-transition
        }
    }

    template<typename STATE, typename MACHINE>
    void onEnter(MACHINE& machine)
    {
        if constexpr (internal::has_timeout_v<STATE>) {
            constexpr auto duration =
                std::chrono::ceil<std::chrono::milliseconds>(STATE::timeout);
            static_assert(duration.count() >= 0 &&
                              duration.count() <= std::numeric_limits<std::uint32_t>::max(),
                          "fsm::timed: timeout out of the 32-bit millisecond range");
            this->startTimer(static_cast<std::uint32_t>(duration.count()), machine);
        }
    }

private:
    // One body per machine, the duration passed as a 32-bit value:
    // materializing the 64-bit chrono constant in every per-state
    // start measured ~40 bytes each on Thumb-1 (-Os, GCC 14)
    template<typename MACHINE>
    void startTimer(std::uint32_t duration_ms, MACHINE& machine)
    {
        timer.start(
            std::chrono::milliseconds{duration_ms},
            [](void* context) {
                static_cast<MACHINE*>(context)->process(timeout{});
            },
            &machine);
    }

public:

    TIMER timer;
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
// timed<POLICY>/timed<POLICY&> ownership semantics apply
template<concepts::timer TIMER>
struct deadlined {
    deadlined()
        requires(!std::is_reference_v<TIMER>)
    = default;

    explicit deadlined(TIMER timer_ref)
        requires std::is_reference_v<TIMER>
        : timer(timer_ref)
    {
    }

    // A deadline the table ignores is a bug: the timer would fire into
    // nothing (the zero sentinel is exempt - it never arms)
    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        static_assert(mtl::all_of_v<typename TABLE::states,
                          internal::deadline_handled_in<TABLE>::template pred>,
                      "fsm::deadlined: state has a deadline but no transition for "
                      "fsm::deadline");
    }

    // Whether the phase continues is a property of the edge (the state
    // left carries the same deadline): the edge hook, per source on a
    // wildcard
    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterFrom(MACHINE& machine)
    {
        if constexpr (internal::continues_deadline_v<OLD_STATE, NEW_STATE>) {
            // the phase's clock keeps running
        } else if constexpr (internal::active_deadline_v<NEW_STATE>) {
            constexpr auto duration =
                std::chrono::ceil<std::chrono::milliseconds>(NEW_STATE::deadline);
            static_assert(duration.count() >= 0 &&
                              duration.count() <= std::numeric_limits<std::uint32_t>::max(),
                          "fsm::deadlined: deadline out of the 32-bit millisecond range");
            this->startTimer(static_cast<std::uint32_t>(duration.count()), machine);
        } else if constexpr (internal::active_deadline_v<OLD_STATE>) {
            timer.stop(); // left the phase: unannotated or the target
        }
    }

private:
    // One body per machine, the duration as a 32-bit value - same
    // measured rationale as fsm::timed::startTimer
    template<typename MACHINE>
    void startTimer(std::uint32_t duration_ms, MACHINE& machine)
    {
        timer.start(
            std::chrono::milliseconds{duration_ms},
            [](void* context) { static_cast<MACHINE*>(context)->process(deadline{}); },
            &machine);
    }

public:
    TIMER timer;
};

} // namespace fsm
