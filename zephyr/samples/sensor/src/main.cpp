/*
 * Sensor monitor: a tour of the state machine's features on a Zephyr
 * board. The tables live in sensor.hpp and led.hpp (Zephyr-free, for
 * the graph generator); this file provides the observers:
 *
 *   VirtualSensor  - starts a "conversion" when the reading state is
 *                     entered (a delayed work) and injects reading_done
 *                     {value} or reading_failed when it finishes
 *   Calibrator      - the calibration feature: when injected, the table
 *                     gains the calibrating state, which it answers with
 *                     calibrated{offset} (CONFIG_SAMPLE_CALIBRATION)
 *   LedController  - a value observer on each state's led annotation,
 *                     driving its own LED state machine (the sub machine
 *                     is an observer of the sensor machine)
 *   LedDriver      - value observer of the LED machine writing led0
 *   TraceLogger     - both machines trace to the mtl_fsm log module
 *
 * The user button (alias sw0) is the emergency stop from any state and
 * resumes from emergency; everything runs on the system workqueue.
 *
 *   west build -t dot        (sensor_table, calibrating_sensor_table, led_table)
 *   west fsm_liveview        (reads /dev/ttyACM0, graphs from build/)
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "led.hpp"
#include "sensor.hpp"

#include <mtl/Typelist.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/zephyr/Timer.hpp>
#include <mtl/zephyr/TraceLogger.hpp>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <type_traits>

LOG_MODULE_REGISTER(sensor_sample, LOG_LEVEL_INF);

namespace {

// --- virtual sensor ---------------------------------------------------------
// Starts a conversion when the reading state is entered; the hook is
// constrained to that state, so it neither runs on other edges nor
// keeps the button's any_state transition from its shared body
class VirtualSensor {
public:
    VirtualSensor() { k_work_init_delayable(&work_, &VirtualSensor::finish); }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        requires std::is_same_v<NEW_STATE, sensor::reading>
    void onEnterState(MACHINE& machine)
    {
        machine_ = &machine;
        done_    = [](void* m, int value) {
            static_cast<MACHINE*>(m)->process(sensor::reading_done{value});
        };
        failed_ = [](void* m) { static_cast<MACHINE*>(m)->process(sensor::reading_failed{}); };
        k_work_reschedule(&work_, K_MSEC(300));
    }

private:
    static void finish(k_work* work)
    {
        auto* self = CONTAINER_OF(k_work_delayable_from_work(work), VirtualSensor, work_);
        if (++self->conversions_ % 4 == 0) { // every fourth conversion fails
            LOG_INF("sensor: conversion failed");
            self->failed_(self->machine_);
            return;
        }
        self->value_ = 35 + (self->value_ + 13) % 60; // a wandering value, 35..94
        LOG_INF("sensor: %d", self->value_);
        self->done_(self->machine_, self->value_);
    }

    k_work_delayable work_;
    void* machine_                  = nullptr;
    void (*done_)(void*, int)       = nullptr;
    void (*failed_)(void*)          = nullptr;
    int conversions_                = 0;
    int value_                      = 0;
};

// --- calibration feature ----------------------------------------------------
class Calibrator {
public:
    Calibrator() { k_work_init_delayable(&work_, &Calibrator::finish); }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        requires std::is_same_v<NEW_STATE, sensor::calibrating>
    void onEnterState(MACHINE& machine)
    {
        machine_ = &machine;
        done_    = [](void* m, int offset) {
            static_cast<MACHINE*>(m)->process(sensor::calibrated{offset});
        };
        k_work_reschedule(&work_, K_MSEC(1500));
    }

private:
    static void finish(k_work* work)
    {
        auto* self = CONTAINER_OF(k_work_delayable_from_work(work), Calibrator, work_);
        LOG_INF("calibrated: offset 3");
        self->done_(self->machine_, 3);
    }

    k_work_delayable work_;
    void* machine_            = nullptr;
    void (*done_)(void*, int) = nullptr;
};

// --- LED: a driver observer on the LED machine, the machine inside the
// observer of the sensor machine ---------------------------------------------
struct LedDriver : fsm::observing<LedDriver> {
    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::lit)
    {
        return STATE::lit;
    }

    void notifyEntry(bool lit)
    {
        this->lit = lit;
        if (ready) {
            gpio_pin_set_dt(&pin, lit);
        }
    }

    // The machine is constructed before main() configures the pin:
    // remember the level until then
    void attach()
    {
        ready = gpio_is_ready_dt(&pin) && gpio_pin_configure_dt(&pin, GPIO_OUTPUT_INACTIVE) == 0;
        if (ready) {
            gpio_pin_set_dt(&pin, lit);
        } else {
            LOG_WRN("no led0 on this board: LED states run unlit");
        }
    }

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led0))
    gpio_dt_spec pin = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
    gpio_dt_spec pin{};
#endif
    bool ready = false;
    bool lit   = false;
};

struct LedController : fsm::observing<LedController> {
    // re-notifying an unchanged pattern only restarts the same pattern:
    // lets the button's any_state transition keep its shared body
    static constexpr bool renotify_safe = true;

    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::led)
    {
        return STATE::led;
    }

    void notifyEntry(sensor::led_pattern kind) { machine.process(led::pattern{kind}); }

    // observers before the machine they are injected into
    fsm::timed<mtl::zephyr::WorkqueueTimer> timeouts;
    LedDriver driver;
    mtl::zephyr::TraceLogger tracer;
    fsm::state_machine<led::led_table, fsm::timed<mtl::zephyr::WorkqueueTimer>, LedDriver,
                       mtl::zephyr::TraceLogger>
        machine{timeouts, driver, tracer};
};

// --- the sensor machine: its table depends on the injected observers ---------
template<typename... OBSERVERs>
using table_for = std::conditional_t<mtl::has_a_v<mtl::typelist<OBSERVERs...>, Calibrator>,
                                     sensor::calibrating_sensor_table, sensor::sensor_table>;

template<typename... OBSERVERs>
using sensor_machine = fsm::state_machine<table_for<OBSERVERs...>,
                                          fsm::timed<mtl::zephyr::WorkqueueTimer>, OBSERVERs...>;

// Static: work items and machine addresses must stay put. Order: timer
// first (armed before anything is notified), the tracer last (its line
// follows the effects)
fsm::timed<mtl::zephyr::WorkqueueTimer> timeouts;
VirtualSensor sensor;
LedController leds;
mtl::zephyr::TraceLogger tracer;
#ifdef CONFIG_SAMPLE_CALIBRATION
Calibrator cal;
sensor_machine<VirtualSensor, Calibrator, LedController, mtl::zephyr::TraceLogger>
    monitor{timeouts, sensor, cal, leds, tracer};
#else
sensor_machine<VirtualSensor, LedController, mtl::zephyr::TraceLogger>
    monitor{timeouts, sensor, leds, tracer};
#endif

// --- emergency button -------------------------------------------------------
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(sw0))

constexpr int64_t debounce_ms = 200;
gpio_dt_spec const button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
gpio_callback button_callback;
k_work button_work;
int64_t last_press = -debounce_ms;

void pressButton(k_work*)
{
    LOG_INF("button pressed");
    monitor.process(sensor::button{});
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
    LOG_WRN("no sw0 button on this board: no emergency stop");
    return 0;
}

#endif

} // namespace

int main()
{
    leds.driver.attach();
    initButton();
    k_sleep(K_FOREVER);
    return 0;
}
