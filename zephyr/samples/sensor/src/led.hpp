/*
 * The LED's own state machine: off, on, or blinking through two timed
 * states. A pattern event carries the wanted pattern; guards on the
 * payload pick the target from any state. Free of Zephyr for the graph
 * generator.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sensor.hpp"

#include <mtl/StateMachine.hpp>

#include <chrono>

namespace led {

using namespace std::chrono_literals;

struct pattern {
    sensor::led_pattern kind;
};

// Each state's level, consumed by the led_driver observer
struct off {
    static constexpr bool lit = false;
};
struct on {
    static constexpr bool lit = true;
};
struct blink_on {
    static constexpr bool lit     = true;
    static constexpr auto timeout = 150ms;
};
struct blink_off {
    static constexpr bool lit     = false;
    static constexpr auto timeout = 150ms;
};

// Guard on the event payload, from whichever state (the templated form
// is validated against any_state, the wildcard's source)
template<sensor::led_pattern KIND>
struct wants {
    static bool check(auto const&, pattern const& event) { return event.kind == KIND; }
};

struct led_table : fsm::transition_table<
    fsm::initial<off>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<pattern>, fsm::to<off>,
                    fsm::guard<wants<sensor::led_pattern::off>>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<pattern>, fsm::to<on>,
                    fsm::guard<wants<sensor::led_pattern::on>>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<pattern>, fsm::to<blink_on>>,
    fsm::transition<fsm::from<blink_on>,  fsm::on<fsm::timeout>, fsm::to<blink_off>>,
    fsm::transition<fsm::from<blink_off>, fsm::on<fsm::timeout>, fsm::to<blink_on>>> {};

} // namespace led
