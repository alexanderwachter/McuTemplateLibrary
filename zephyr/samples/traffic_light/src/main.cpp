/*
 * Traffic light on Zephyr: the state timeouts run on the system
 * workqueue (mtl::zephyr::WorkqueueTimer), the board's user button
 * (alias sw0) is the pedestrian button shortening the green phase after
 * a minimum green time, and every transition is logged by
 * mtl::zephyr::TraceLogger on the mtl_fsm module. Watch it live:
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

#include <mtl/zephyr/Timer.hpp>
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

using machine = fsm::state_machine<traffic_light_table, fsm::timed<mtl::zephyr::WorkqueueTimer>,
                                   mtl::zephyr::TraceLogger>;

// Static: the timer's work item and the machine's address must stay put
fsm::timed<mtl::zephyr::WorkqueueTimer> timeouts;
mtl::zephyr::TraceLogger trace_logger;
machine light{timeouts, trace_logger};

// --- pedestrian button ------------------------------------------------------
// The ISR only queues work: the machine runs on the system workqueue,
// which serializes the button with the timeouts. A press within the
// debounce time of the previous one is contact bounce
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(sw0))

constexpr int64_t debounce_ms = 200;
gpio_dt_spec const button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
gpio_callback button_callback;
k_work button_work;
int64_t last_press = -debounce_ms;

void pressButton(k_work*)
{
    LOG_INF("pedestrian button pressed");
    if (!light.process(pedestrian_button{})) {
        LOG_INF("ignored (not green, or minimum green time not elapsed)");
    }
}

void buttonIsr(device const*, gpio_callback*, uint32_t)
{
    auto const now = k_uptime_get();
    if (now - last_press < debounce_ms) {
        return;
    }
    last_press = now;
    k_work_submit(&button_work);
}

int initButton()
{
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("button port not ready");
        return -ENODEV;
    }
    k_work_init(&button_work, pressButton);
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
    // Construction already logged the initial state and armed red's
    // timeout; everything else runs on the system workqueue
    initButton();
    k_sleep(K_FOREVER);
    return 0;
}
