/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Traffic light example: red -> red_yellow -> green -> yellow -> red,
// driven by state timeouts. A pedestrian button shortens the green phase,
// guarded by a minimum green time. lamp_driver prints the lamp levels;
// on a real target it would write GPIOs and the polling timer would be a
// hardware timer or work queue. trace_printer prints every transition in
// the fsm::tracing grammar, so the run can be watched live with
// tools/fsmview:
//   TrafficLightExample --dot > traffic_light.dot
//   TrafficLightExample | fsmview.py traffic_light.dot --stdin

#include <mtl/StateMachine.hpp>
#include <mtl/StateMachineDot.hpp>
#include <mtl/StateMachineTrace.hpp>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <print>
#include <string_view>
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

struct lamp_driver : fsm::observing<lamp_driver> {
    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::lamps)
    {
        return STATE::lamps;
    }

    void notifyEntry(lamps_t const& lamps)
    {
        auto const elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        std::print("[{:>6}] lamps: red={:<5} yellow={:<5} green={:<5}\n",
                   elapsed, lamps.red, lamps.yellow, lamps.green);
    }

    std::chrono::steady_clock::time_point start;
};

// Flushed per line: through a pipe stdout is block-buffered, and the
// live view wants every line as it happens
struct trace_printer : fsm::tracing<trace_printer> {
    void traceInitial(char const* machine, char const* state)
    {
        std::println(fsm::trace_format::initial, machine, state);
        std::fflush(stdout);
    }
    void traceTransition(char const* machine, char const* from, char const* event, char const* to)
    {
        std::println(fsm::trace_format::transition, machine, from, event, to);
        std::fflush(stdout);
    }
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
    void onEntry() { entered = std::chrono::steady_clock::now(); }
};

struct yellow {
    static constexpr lamps_t lamps{.red = false, .yellow = true, .green = false};
    static constexpr auto timeout = 1000ms;
};

struct minimum_green_elapsed {
    static constexpr auto minimum = 2000ms;
    static bool check(green const& state)
    {
        return std::chrono::steady_clock::now() - state.entered >= minimum;
    }
};

// A named table: its short name identifies the machine in trace lines
// and in the DOT graph
struct traffic_light_table : fsm::transition_table<
    fsm::initial<red>,
    fsm::transition<fsm::from<red>,        fsm::on<fsm::timeout>,      fsm::to<red_yellow>>,
    fsm::transition<fsm::from<red_yellow>, fsm::on<fsm::timeout>,      fsm::to<green>>,
    fsm::transition<fsm::from<green>,      fsm::on<fsm::timeout>,      fsm::to<yellow>>,
    fsm::transition<fsm::from<yellow>,     fsm::on<fsm::timeout>,      fsm::to<red>>,
    fsm::transition<fsm::from<green>,      fsm::on<pedestrian_button>, fsm::to<yellow>,
                    fsm::guard<minimum_green_elapsed>>> {};

using machine = fsm::state_machine<traffic_light_table, fsm::timed<polling_timer>, lamp_driver,
                                   trace_printer>;

} // namespace

int main(int argc, char* argv[])
{
    if (argc > 1 && std::string_view{argv[1]} == "--dot") {
        fsm::writeDot<traffic_light_table>(std::cout, "traffic_light");
        return 0;
    }

    auto const start   = std::chrono::steady_clock::now();
    auto const elapsed = [start] {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
    };

    fsm::timed<polling_timer> timeouts;
    lamp_driver driver{.start = start};
    trace_printer tracer;
    machine sm{timeouts, driver, tracer};

    auto const press_button = [&] {
        std::print("[{:>6}] pedestrian button pressed\n", elapsed());
        if (!sm.process(pedestrian_button{})) {
            std::print("[{:>6}] ignored (minimum green time not elapsed)\n", elapsed());
        }
    };

    // First press falls into the minimum green time, second one is accepted
    bool first_press  = false;
    bool second_press = false;
    while (elapsed() < 10s) {
        timeouts.timer.poll();
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
