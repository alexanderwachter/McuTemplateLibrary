/*
 * Traffic light on Zephyr: mtl::zephyr::StateMachine is the machine in
 * one declaration - a queued machine with its timeout timer and a
 * workqueue of its own, so both event sources stay in their ISRs: the
 * state timeouts (the k_timer expiry only latches) and the board's
 * user button (alias sw0), the pedestrian
 * button shortening the green phase after a minimum green time. Every
 * transition is logged by mtl::zephyr::TraceLogger on the mtl_fsm
 * module. Watch it live:
 *
 *   west build -t dot                           (writes build/traffic_light_table.dot)
 *   stty -F /dev/ttyACM0 115200 raw -echo -icrnl && cat /dev/ttyACM0 | fsmview.py build/traffic_light_table.dot --stdin
 *
 * The states and the table live in traffic_light.hpp, free of Zephyr,
 * so the host-built graph generator can include them.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "traffic_light.hpp"

#include <mtl/zephyr/StateMachine.hpp>
#include <mtl/zephyr/TraceLogger.hpp>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(traffic_light, LOG_LEVEL_INF);

namespace traffic_light {

int64_t uptimeMs()
{
    return k_uptime_get();
}

} // namespace traffic_light

namespace {

using traffic_light::pedestrian_button;
using traffic_light::traffic_light_table;

// One observer for the three lamps: each lamp type has its overload,
// and only a lamp that switches is notified (red stays on from red to
// red_yellow: yellow alone is reported). The green lamp also drives the
// board's led0 when there is one
struct LampDriver : fsm::observing<LampDriver> {
    void notifyEntry(traffic_light::red_lamp lamp) { LOG_INF("red lamp %s", lamp.on ? "on" : "off"); }
    void notifyEntry(traffic_light::yellow_lamp lamp)
    {
        LOG_INF("yellow lamp %s", lamp.on ? "on" : "off");
    }
    void notifyEntry(traffic_light::green_lamp lamp)
    {
        LOG_INF("green lamp %s", lamp.on ? "on" : "off");
        green = lamp.on;
        if (ready) {
            gpio_pin_set_dt(&led, green);
        }
    }

    // The machine is constructed before main() configures the pin
    void attach()
    {
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led0))
        ready = gpio_is_ready_dt(&led) && gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) == 0;
        if (ready) {
            gpio_pin_set_dt(&led, green);
        }
#endif
    }

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led0))
    gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
    gpio_dt_spec led{}; // never touched: ready stays false
#endif
    bool ready = false;
    bool green = false;
};

// The whole machine in one declaration: the table has timed states, so
// it brings its timeout timer; it drains on a workqueue thread of its
// own, named after the table; the observers' types are deduced.
// Static: the kernel objects and the machine's address must stay put
LampDriver lamps;
mtl::zephyr::TraceLogger trace_logger;
mtl::zephyr::StateMachine light{mtl::zephyr::table<traffic_light_table>, lamps, trace_logger};

// --- pedestrian button ------------------------------------------------------
// The ISR processes the event itself: process() only queues it, the
// machine runs on the system workqueue, serialized with the timeouts.
// A press within the debounce time of the previous one is contact
// bounce; a press the table ignores (not green, or the minimum green
// time not elapsed) simply fires nothing
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(sw0))

constexpr int64_t debounce_ms = 200;
gpio_dt_spec const button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
gpio_callback button_callback;
int64_t last_press = -debounce_ms;

void buttonIsr(device const*, gpio_callback*, uint32_t)
{
    auto const now = k_uptime_get();
    if (now - last_press < debounce_ms) {
        return;
    }
    last_press = now;
    light.process(pedestrian_button{});
}

int initButton()
{
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("button port not ready");
        return -ENODEV;
    }
    int error = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (error == 0) {
        error = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    }
    if (error != 0) {
        LOG_ERR("button configuration failed: %d", error);
        return error;
    }
    gpio_init_callback(&button_callback, buttonIsr, BIT(button.pin));
    return gpio_add_callback(button.port, &button_callback);
}

#else

int initButton()
{
    LOG_WRN("no sw0 button on this board: timeouts only");
    return 0;
}

#endif

} // namespace

int main()
{
    // Construction already logged the initial state, reported the lamps
    // and armed red's timeout; everything else is drained on the system
    // workqueue
    lamps.attach();
    initButton();
    k_sleep(K_FOREVER);
    return 0;
}
