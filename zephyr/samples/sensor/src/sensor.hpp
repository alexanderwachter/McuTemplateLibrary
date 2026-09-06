/*
 * The sensor monitor's events, states and tables - free of Zephyr so
 * the host-built graph generator (west build -t dot) can include them.
 *
 * A reading is started by entering the reading state (the virtual
 * sensor observer starts it) and answered with reading_done{value} or
 * reading_failed. Failures are retried through the retrying state up to
 * a budget kept in machine-owned context; a value above the limit takes
 * the alarm branch. The button is an emergency stop from every state,
 * and the calibration at start exists only when a calibrator observer
 * is injected: the table is composed accordingly.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>
#include <mtl/TypelistAlgorithms.hpp>

#include <chrono>

namespace sensor {

using namespace std::chrono_literals;

// --- events -----------------------------------------------------------------
struct reading_done {
    int value;
};
struct reading_failed {};
struct calibrated {
    int offset;
};
struct button {};

// --- what the LED shows per state (consumed by the led_controller observer)
enum class led_pattern { off, on, blink };

// --- machine-owned context: survives transitions, one instance per type
struct retry_budget {
    int failures = 0;
};
struct stop_log {
    int readings_ignored = 0;
};

// --- states -----------------------------------------------------------------
struct idle {
    static constexpr auto timeout = 1000ms;
    static constexpr auto led     = led_pattern::off;

    retry_budget& context;
    explicit idle(retry_budget& budget) : context(budget) { context.failures = 0; }
};

// Feature state: in the table only with a calibrator observer, which
// answers with calibrated{offset}; the timeout is the fallback
struct calibrating {
    static constexpr auto timeout = 3000ms;
    static constexpr auto led     = led_pattern::on;
};

struct reading {
    static constexpr auto timeout = 2000ms; // the sensor never answered
    static constexpr auto led     = led_pattern::on;

    retry_budget& context;
    explicit reading(retry_budget& budget) : context(budget) {}
};

struct retrying {
    static constexpr auto timeout = 200ms;
    static constexpr auto led     = led_pattern::off;

    retry_budget& context;
    // constructed from the failure: one more attempt used
    retrying(reading_failed const&, retry_budget& budget) : context(budget)
    {
        ++context.failures;
    }
    explicit retrying(retry_budget& budget) : context(budget) {}
};

struct alarm {
    static constexpr auto timeout = 2000ms;
    static constexpr auto led     = led_pattern::blink;

    int value = 0;
    alarm() = default;
    explicit alarm(reading_done const& event) : value(event.value) {} // payload delivery
};

struct failed {
    static constexpr auto timeout = 3000ms;
    static constexpr auto led     = led_pattern::blink;
};

struct emergency {
    static constexpr auto led = led_pattern::on;

    stop_log& context;
    explicit emergency(stop_log& log) : context(log) {}
    // a reading finishing while stopped is handled in place
    void handle(reading_done const&) { ++context.readings_ignored; }
    void handle(reading_failed const&) { ++context.readings_ignored; }
};

// --- guards -----------------------------------------------------------------
struct above_limit {
    static constexpr int limit = 75;
    static bool check(reading const&, reading_done const& event) { return event.value > limit; }
};

struct retries_left {
    static constexpr int max_retries = 3;
    static bool check(reading const& state) { return state.context.failures < max_retries; }
};

// --- tables: the core, and the calibration feature prepended when enabled
using core_transitions = mtl::typelist<
    fsm::transition<fsm::from<idle>,     fsm::on<fsm::timeout>,   fsm::to<reading>>,
    fsm::transition<fsm::from<reading>,  fsm::on<reading_done>,   fsm::to<alarm>,
                    fsm::guard<above_limit>>,
    fsm::transition<fsm::from<reading>,  fsm::on<reading_done>,   fsm::to<idle>>,
    fsm::transition<fsm::from<reading>,  fsm::on<reading_failed>, fsm::to<retrying>,
                    fsm::guard<retries_left>>,
    fsm::transition<fsm::from<reading>,  fsm::on<reading_failed>, fsm::to<failed>>,
    fsm::transition<fsm::from<reading>,  fsm::on<fsm::timeout>,   fsm::to<failed>>,
    fsm::transition<fsm::from<retrying>, fsm::on<fsm::timeout>,   fsm::to<reading>>,
    fsm::transition<fsm::from<alarm>,    fsm::on<fsm::timeout>,   fsm::to<idle>>,
    fsm::transition<fsm::from<failed>,   fsm::on<fsm::timeout>,   fsm::to<idle>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<button>,   fsm::to<emergency>>,
    fsm::transition<fsm::from<emergency>, fsm::on<button>,        fsm::to<idle>>,
    fsm::internal_transition<fsm::from<emergency>, fsm::on<reading_done>>,
    fsm::internal_transition<fsm::from<emergency>, fsm::on<reading_failed>>>;

using calibration_transitions = mtl::typelist<
    fsm::initial<calibrating>,
    fsm::transition<fsm::from<calibrating>, fsm::on<calibrated>,   fsm::to<idle>>,
    fsm::transition<fsm::from<calibrating>, fsm::on<fsm::timeout>, fsm::to<failed>>>;

// Named: the short names identify the machines in trace lines and graphs
struct sensor_table : mtl::rebind_t<core_transitions, fsm::transition_table> {};

struct calibrating_sensor_table
    : mtl::rebind_t<mtl::concat_t<calibration_transitions, core_transitions>,
                    fsm::transition_table> {};

} // namespace sensor
