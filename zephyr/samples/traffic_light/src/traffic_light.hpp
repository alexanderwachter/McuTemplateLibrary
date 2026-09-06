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

struct red {
    static constexpr auto timeout = 2000ms;
};
struct red_yellow {
    static constexpr auto timeout = 500ms;
};
struct green {
    static constexpr auto timeout = 6000ms; // full phase without a button press

    int64_t entered = 0;
    void onEntry() { entered = uptimeMs(); }
};
struct yellow {
    static constexpr auto timeout = 1000ms;
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
