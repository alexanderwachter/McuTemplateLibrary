/*
 * The traffic light's states and table, free of Zephyr so that the
 * host-built dotgen.cpp can write its graph: the one kernel service the
 * guard needs, the uptime, comes through uptimeMs().
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/StateMachine.hpp>

#include <chrono>
#include <cstdint>

namespace traffic_light {

using namespace std::chrono_literals;

// Milliseconds since boot; main.cpp binds it to k_uptime_get()
int64_t uptimeMs();

struct pedestrian_button {};

// The three lamps, one type each: an observer picks them by overload,
// and a phase change notifies only the lamps that actually switch
struct red_lamp {
    bool on;
    constexpr bool operator==(red_lamp const&) const = default;
};
struct yellow_lamp {
    bool on;
    constexpr bool operator==(yellow_lamp const&) const = default;
};
struct green_lamp {
    bool on;
    constexpr bool operator==(green_lamp const&) const = default;
};

struct red {
    static constexpr auto timeout     = 2000ms;
    static constexpr auto annotations = fsm::annotate(red_lamp{true}, yellow_lamp{false}, green_lamp{false});
};
struct red_yellow {
    static constexpr auto timeout     = 500ms;
    static constexpr auto annotations = fsm::annotate(red_lamp{true}, yellow_lamp{true}, green_lamp{false});
};
struct green {
    static constexpr auto timeout     = 6000ms; // full phase without a button press
    static constexpr auto annotations = fsm::annotate(red_lamp{false}, yellow_lamp{false}, green_lamp{true});

    int64_t entered = 0;
    void onEntry() { entered = uptimeMs(); }
};
struct yellow {
    static constexpr auto timeout     = 1000ms;
    static constexpr auto annotations = fsm::annotate(red_lamp{false}, yellow_lamp{true}, green_lamp{false});
};

struct minimum_green_elapsed {
    static constexpr int64_t minimum = 2000;
    static bool check(green const& state) { return uptimeMs() - state.entered >= minimum; }
};

// Named: the short name is the machine id in trace lines and the graph
struct traffic_light_table : fsm::transition_table<
    fsm::initial<red>,
    fsm::transition<fsm::from<red>,        fsm::on<fsm::timeout>,      fsm::to<red_yellow>>,
    fsm::transition<fsm::from<red_yellow>, fsm::on<fsm::timeout>,      fsm::to<green>>,
    fsm::transition<fsm::from<green>,      fsm::on<fsm::timeout>,      fsm::to<yellow>>,
    fsm::transition<fsm::from<yellow>,     fsm::on<fsm::timeout>,      fsm::to<red>>,
    fsm::transition<fsm::from<green>,      fsm::on<pedestrian_button>, fsm::to<yellow>,
                    fsm::guard<minimum_green_elapsed>>> {};

} // namespace traffic_light
