/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Traffic light example for the fsm state machine.
//
// The light cycles red -> red_yellow -> green -> yellow -> red, driven
// entirely by state timeouts. A pedestrian button shortens the green phase,
// but a guard keeps green up for a minimum time. A lamp_driver observer
// prints the lamp levels whenever they change; on a real target it would
// write the GPIO levels instead.
//
// The timer policy is a superloop-style polling timer: the machine arms it
// on entry into a timed state, and the main loop calls poll(), which injects
// fsm::timeout into the machine when the deadline has passed. On a real
// target the policy would wrap a hardware timer or a work queue instead.

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <print>
#include <thread>

using namespace std::chrono_literals;

namespace {

// --- timer policy -----------------------------------------------------------
class polling_timer {
public:
    void start(std::chrono::milliseconds duration, fsm::timer_callback callback, void* context)
    {
        deadline_ = std::chrono::steady_clock::now() + duration;
        callback_ = callback;
        context_  = context;
        armed_    = true;
    }

    void stop() { armed_ = false; }

    // Called from the main loop; fires the callback once the deadline passed
    void poll()
    {
        if (armed_ && std::chrono::steady_clock::now() >= deadline_) {
            armed_ = false;
            callback_(context_);
        }
    }

private:
    std::chrono::steady_clock::time_point deadline_{};
    fsm::timer_callback callback_ = nullptr;
    void* context_                = nullptr;
    bool armed_                   = false;
};
static_assert(fsm::concepts::timer<polling_timer>);

// --- lamps and the observer driving them ------------------------------------
struct lamps_t {
    bool red;
    bool yellow;
    bool green;
    constexpr bool operator==(lamps_t const&) const = default;
};

// Observer policy: notified when the lamp levels change across a transition.
// The fsm::observing CRTP base generates the annotation<STATE>() machinery;
// the trailing return type makes states without a lamps member drop out.
struct lamp_driver : fsm::observing<lamp_driver> {
    static constexpr auto observe(auto const& state) -> decltype(state.lamps)
    {
        return state.lamps;
    }

    void notify(lamps_t const& lamps)
    {
        auto const elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        std::print("[{:>6}] lamps: red={:<5} yellow={:<5} green={:<5}\n",
                   elapsed, lamps.red, lamps.yellow, lamps.green);
    }

    std::chrono::steady_clock::time_point start;
};

// --- events and states ------------------------------------------------------
struct pedestrian_button {};

struct red {
    static constexpr lamps_t lamps{.red = true, .yellow = false, .green = false};
    static constexpr auto timeout = 2000ms;
};

struct red_yellow {
    static constexpr lamps_t lamps{.red = true, .yellow = true, .green = false};
    static constexpr auto timeout = 500ms;
};

struct green {
    static constexpr lamps_t lamps{.red = false, .yellow = false, .green = true};
    static constexpr auto timeout = 6000ms; // full phase without a button press

    std::chrono::steady_clock::time_point entered;
    void on_entry() { entered = std::chrono::steady_clock::now(); }
};

struct yellow {
    static constexpr lamps_t lamps{.red = false, .yellow = true, .green = false};
    static constexpr auto timeout = 1000ms;
};

// Guard: the pedestrian button only shortens green after a minimum green time
struct minimum_green_elapsed {
    static constexpr auto minimum = 2000ms;
    static bool check(green const& state)
    {
        return std::chrono::steady_clock::now() - state.entered >= minimum;
    }
};

using table = fsm::transition_table<
    fsm::transition<fsm::from<red>,        fsm::on<fsm::timeout>,      fsm::to<red_yellow>>,
    fsm::transition<fsm::from<red_yellow>, fsm::on<fsm::timeout>,      fsm::to<green>>,
    fsm::transition<fsm::from<green>,      fsm::on<fsm::timeout>,      fsm::to<yellow>>,
    fsm::transition<fsm::from<yellow>,     fsm::on<fsm::timeout>,      fsm::to<red>>,
    fsm::transition<fsm::from<green>,      fsm::on<pedestrian_button>, fsm::to<yellow>,
                    fsm::guard<minimum_green_elapsed>>>;

using machine = fsm::state_machine<table, polling_timer, lamp_driver>;

} // namespace

int main()
{
    auto const start   = std::chrono::steady_clock::now();
    auto const elapsed = [start] {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
    };

    lamp_driver driver{.start = start}; // owned by the application, injected by reference
    machine sm{polling_timer{}, driver};

    auto const press_button = [&] {
        std::print("[{:>6}] pedestrian button pressed\n", elapsed());
        if (!sm.process(pedestrian_button{})) {
            std::print("[{:>6}] ignored (minimum green time not elapsed)\n", elapsed());
        }
    };

    // Superloop: poll the timer and press the button twice during green -
    // the first press is blocked by the guard, the second one is accepted.
    bool first_press  = false;
    bool second_press = false;
    while (elapsed() < 10s) {
        sm.timer().poll();
        if (!first_press && elapsed() >= 3500ms) {
            first_press = true;
            press_button();
        }
        if (!second_press && elapsed() >= 5500ms) {
            second_press = true;
            press_button();
        }
        std::this_thread::sleep_for(20ms);
    }
}
