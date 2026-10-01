/*
 * The sensor monitor's events, states and tables - free of Zephyr so
 * the host-built graph generator (west build -t dot) can include them.
 *
 * A reading is started by entering the reading state (the virtual
 * sensor observer starts it) and answered with reading_done{value} or
 * reading_failed. The retry loop is a composite state: measuring owns
 * a submachine of reading and retrying, failures are retried there up
 * to a budget kept in the submachine's context (fresh on every entry
 * of measuring), and what the submachine does not handle - a finished
 * reading, a failure once the retries are used up, the overall time
 * budget - is measuring's own transition. A value above the limit takes
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

// --- per-state annotations, observed by type: the LED pattern (the
// LedController observer) and the sensor's power rail (PowerRail)
enum class led_pattern { off, on, blink };

struct sensor_power {
    bool on;
    constexpr bool operator==(sensor_power const&) const = default;
};

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
    static constexpr auto annotations = fsm::annotate(led_pattern::off, sensor_power{false});
};

// Feature state: in the table only with an observer enabling the
// feature, which answers with calibrated{offset}; the timeout is the
// fallback
struct calibrating {
    using feature = calibration_feature;

    static constexpr auto timeout = 3000ms;
    static constexpr auto annotations = fsm::annotate(led_pattern::on, sensor_power{true});
};

// The sub-states of measuring: they annotate the LED, which changes
// between them, while the sensor's power rail - on for the whole
// measurement - is measuring's own annotation. An annotation type lives
// on one level of the hierarchy
struct reading {
    static constexpr auto timeout = 2000ms; // the sensor never answered
    static constexpr auto annotations = fsm::annotate(led_pattern::on);

    using contexts = mtl::typelist<retry_budget>;
    retry_budget& context;
    explicit reading(retry_budget& budget) : context(budget) {}
};

struct retrying {
    static constexpr auto timeout = 200ms;
    static constexpr auto annotations = fsm::annotate(led_pattern::off);

    using contexts = mtl::typelist<retry_budget>;
    retry_budget& context;
    // one more attempt used, whether the sensor reported the failure or
    // never answered (the timeout path constructs without the event)
    retrying(reading_failed const&, retry_budget& budget) : retrying(budget) {}
    explicit retrying(retry_budget& budget) : context(budget) { ++context.failures; }
};

struct alarm {
    static constexpr auto timeout = 2000ms;
    static constexpr auto annotations = fsm::annotate(led_pattern::blink, sensor_power{false});

    int value = 0;
    alarm() = default;
    explicit alarm(reading_done const& event) : value(event.value) {} // payload delivery
};

struct failed {
    static constexpr auto timeout = 3000ms;
    static constexpr auto annotations = fsm::annotate(led_pattern::blink, sensor_power{false});
};

struct emergency {
    static constexpr auto annotations = fsm::annotate(led_pattern::on, sensor_power{false});

    using contexts = mtl::typelist<stop_log>;
    stop_log& context;
    explicit emergency(stop_log& log) : context(log) {}
    // a reading finishing while stopped is handled in place
    void handle(reading_done const&) { ++context.readings_ignored; }
    void handle(reading_failed const&) { ++context.readings_ignored; }
};

// --- guards -----------------------------------------------------------------
// A question the table asks, answered by an object injected into the
// machine (AlarmPolicy in main.cpp holds the limit): the table decides
// where the question is asked, not what the answer depends on
struct above_limit {};

// A guard answering itself: static, needs no injection
struct retries_left {
    static constexpr int max_retries = 3;
    static bool check(reading const& state) { return state.context.failures < max_retries; }
};

// --- the retry loop: the submachine of measuring ----------------------------
// Every row here is the submachine's own business; a reading_done, or a
// reading_failed once the guard refuses, is not handled here and falls
// through to measuring's rows in the parent table. The timeouts run on
// the second timer slot, next to measuring's budget on the first
struct measuring_table : fsm::transition_table<
    fsm::transition<fsm::from<reading>,  fsm::on<reading_failed>, fsm::to<retrying>,
                    fsm::guard<retries_left>>,
    fsm::transition<fsm::from<reading>,  fsm::on<fsm::timeout>,   fsm::to<retrying>>,
    fsm::transition<fsm::from<retrying>, fsm::on<fsm::timeout>,   fsm::to<reading>>> {};

// The composite state: constructed with a fresh submachine (and a fresh
// retry_budget - the submachine's context) on every entry, its timeout
// the budget for the whole loop
struct measuring {
    using submachine = measuring_table;
    static constexpr auto timeout     = 6000ms;
    static constexpr auto annotations = fsm::annotate(sensor_power{true});
};

// --- the table: one list, features included; a disabled feature is
// filtered out. Without the calibration entries the first transition's
// source, idle, is the initial state
using sensor_transitions = mtl::typelist<
    fsm::initial<calibrating>,
    fsm::transition<fsm::from<calibrating>, fsm::on<calibrated>,   fsm::to<idle>>,
    fsm::transition<fsm::from<calibrating>, fsm::on<fsm::timeout>, fsm::to<failed>>,
    fsm::transition<fsm::from<idle>,      fsm::on<fsm::timeout>,   fsm::to<measuring>>,
    fsm::transition<fsm::from<measuring>, fsm::on<reading_done>,   fsm::to<alarm>,
                    fsm::guard<above_limit>>,
    fsm::transition<fsm::from<measuring>, fsm::on<reading_done>,   fsm::to<idle>>,
    fsm::transition<fsm::from<measuring>, fsm::on<reading_failed>, fsm::to<failed>>, // retries used up
    fsm::transition<fsm::from<measuring>, fsm::on<fsm::timeout>,   fsm::to<failed>>, // the budget
    fsm::transition<fsm::from<alarm>,     fsm::on<fsm::timeout>,   fsm::to<idle>>,
    fsm::transition<fsm::from<failed>,    fsm::on<fsm::timeout>,   fsm::to<idle>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<button>,    fsm::to<emergency>>,
    fsm::transition<fsm::from<emergency>, fsm::on<button>,         fsm::to<idle>>,
    fsm::internal_transition<fsm::from<emergency>, fsm::on<reading_done>>,
    fsm::internal_transition<fsm::from<emergency>, fsm::on<reading_failed>>>;

// The table, features included: the machine built on it removes every
// feature none of its observers enables, at every level. Named (a
// struct, not an alias): the short name, sensor_table, identifies the
// machine in trace lines and graphs
struct sensor_table : mtl::rebind_t<sensor_transitions, fsm::transition_table> {};

// A stand-in observer enabling every feature, for the checks below
// (the real enabler lives with the board code)
struct every_feature {
    using enables = calibration_feature;
};

// what a machine runs: with the calibrator, the table as named; without
// an enabler, the table minus the feature, idle leading
using with_calibration    = fsm::enabled_table_t<sensor_table, mtl::typelist<every_feature>>;
using without_calibration = fsm::enabled_table_t<sensor_table, mtl::typelist<>>;
static_assert(std::is_same_v<with_calibration, sensor_table>);
static_assert(std::is_same_v<mtl::front_t<with_calibration::states>, calibrating>);
static_assert(std::is_same_v<mtl::front_t<without_calibration::states>, idle>);
static_assert(!mtl::has_a_v<without_calibration::states, calibrating>);
// two machine levels: the facade brings two timers
static_assert(fsm::levels_v<sensor_table> == 2);
static_assert(std::is_same_v<fsm::nested_tables_t<sensor_table>,
                             mtl::typelist<sensor_table, measuring_table>>);

} // namespace sensor
