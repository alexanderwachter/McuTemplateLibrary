/*
 * The sensor monitor's events, states and tables - free of Zephyr so
 * the host-built graph generator (west build -t dot) can include them.
 *
 * A reading is started by entering the reading state (the virtual
 * sensor observer starts it) and answered with reading_done{value} or
 * reading_failed. Failures are retried through the retrying state up to
 * a budget kept in machine-owned context; a value above the limit takes
 * the alarm branch. The button is an emergency stop from every state.
 *
 * The calibration at start is a feature: its state declares
 * `using feature = calibration_feature;`, and only an injected observer
 * declaring `using enables = calibration_feature;` switches it on -
 * otherwise the feature's states and every entry touching them are
 * filtered out of the table at compile time.
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
#include <type_traits>

namespace sensor {

using namespace std::chrono_literals;

// --- the feature tag: declared by its states and by the enabling observer
struct calibration_feature {};

// --- events -----------------------------------------------------------------
struct reading_done {
    int value;
};
struct reading_failed {};
struct calibrated {
    int offset;
};
struct button {};

// --- what the LED shows per state (consumed by the LedController observer)
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

// Feature state: in the table only with an observer enabling the
// feature, which answers with calibrated{offset}; the timeout is the
// fallback
struct calibrating {
    using feature = calibration_feature;

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

// --- the table: one list, features included; a disabled feature is
// filtered out. Without the calibration entries the first transition's
// source, idle, is the initial state
using sensor_transitions = mtl::typelist<
    fsm::initial<calibrating>,
    fsm::transition<fsm::from<calibrating>, fsm::on<calibrated>,   fsm::to<idle>>,
    fsm::transition<fsm::from<calibrating>, fsm::on<fsm::timeout>, fsm::to<failed>>,
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

// The table for the observers injected into the machine: every feature
// none of them enables is removed. Named (a struct, not an alias): the
// short name, sensor_table, identifies the machine in trace lines and
// graphs whatever the observers are
template<typename... OBSERVERs>
struct sensor_table
    : mtl::rebind_t<fsm::remove_disabled_features_t<sensor_transitions, OBSERVERs...>,
                    fsm::transition_table> {};

// A stand-in observer enabling every feature, for the graph generator
// and the checks below (the real enablers live with the board code)
struct every_feature {
    using enables = calibration_feature;
};

static_assert(std::is_same_v<mtl::front_t<sensor_table<every_feature>::states>, calibrating>);
static_assert(std::is_same_v<mtl::front_t<sensor_table<>::states>, idle>);
static_assert(!mtl::has_a_v<sensor_table<>::states, calibrating>);

} // namespace sensor
