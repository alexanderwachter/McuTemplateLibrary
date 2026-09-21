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
 *   LedController  - picks the led_pattern element of each state's
 *                     annotation set and drives its own LED state machine
 *                     (the sub machine is an observer of the sensor machine)
 *   PowerRail      - picks the sensor_power element of the same sets
 *   LedDriver      - value observer of the LED machine writing led0
 *   TraceLogger     - both machines trace to the mtl_fsm log module
 *
 * Both machines are fsm::QueuedMachine: process() only queues, the
 * drain runs on a workqueue of their own (mtl::zephyr::WorkQueue,
 * WorkOn), the FIFO is guarded by mtl::zephyr::SpinLock. Every event
 * source may therefore process() from where it is - the button from its
 * ISR, the timeouts from k_timer's ISR (mtl::zephyr::QueuedTimer: they
 * only latch), the sensor's work items from the system workqueue, and
 * the LED controller from inside the sensor machine's hook. The user button (alias sw0) is the emergency
 * stop from any state and resumes from emergency.
 *
 *   west build -t dot        (sensor_table as configured, led_table)
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
#include <mtl/zephyr/Work.hpp>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <type_traits>

LOG_MODULE_REGISTER(sensor_sample, LOG_LEVEL_INF);

namespace {

// The queued machine is the one way in: a hook only ever sees the raw
// machine, for reading it. What the observers below have to report
// goes through these, defined next to the machine further down
void reportReading(int value);
void reportReadingFailed();
void reportCalibrated(int offset);

// --- virtual sensor ---------------------------------------------------------
// Entering reading starts a conversion: a hook of one state, the edge
// does not matter. Glue of this table's own module, so naming the state
// is fine here
class VirtualSensor {
public:
    VirtualSensor() { k_work_init_delayable(&work_, &VirtualSensor::finish); }

    template<typename STATE, typename MACHINE>
        requires std::is_same_v<STATE, sensor::reading>
    void onEnter(MACHINE&)
    {
        k_work_reschedule(&work_, K_MSEC(300));
    }

private:
    static void finish(k_work* work)
    {
        auto* self = CONTAINER_OF(k_work_delayable_from_work(work), VirtualSensor, work_);
        if (++self->conversions_ % 4 == 0) { // every fourth conversion fails
            LOG_INF("sensor: conversion failed");
            reportReadingFailed();
            return;
        }
        self->value_ = 35 + (self->value_ + 13) % 60; // a wandering value, 35..94
        LOG_INF("sensor: %d", self->value_);
        reportReading(self->value_);
    }

    k_work_delayable work_;
    int conversions_ = 0;
    int value_       = 0;
};

// --- calibration feature ----------------------------------------------------
// Declaring the feature's tag switches its states into the table
class Calibrator {
public:
    using enables = sensor::calibration_feature;

    Calibrator() { k_work_init_delayable(&work_, &Calibrator::finish); }

    // calibrating is the initial state: the construction-time entry is
    // the entry that starts the calibration
    template<typename STATE, typename MACHINE>
        requires std::is_same_v<STATE, sensor::calibrating>
    void onEnter(MACHINE&)
    {
        k_work_reschedule(&work_, K_MSEC(1500));
    }

private:
    static void finish(k_work*)
    {
        LOG_INF("calibrated: offset 3");
        reportCalibrated(3);
    }

    k_work_delayable work_;
};

// Both machines drain on a workqueue of their own rather than the
// system workqueue: its thread is theirs alone. Declared before the
// machines - the constructor starts the thread
mtl::zephyr::WorkQueue<2048, 5> fsm_queue{"sensor_fsm"};
using Work = mtl::zephyr::WorkOn<fsm_queue>;

// --- LED: a driver observer on the LED machine, the machine inside the
// observer of the sensor machine ---------------------------------------------
// Boards without led0 get the LED machine without its driver: the
// states still run (and trace), nothing is written
#define SAMPLE_HAS_LED DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led0))

#if SAMPLE_HAS_LED
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
            LOG_ERR("led0 not ready: LED states run unlit");
        }
    }

    gpio_dt_spec pin = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    bool ready       = false;
    bool lit         = false;
};
#endif

// Picks the led_pattern element of each state's annotation set by the
// overload's type alone; declaring the type makes the machine check
// that some state carries it
struct LedController : fsm::observing<LedController> {
    using observes = mtl::typelist<sensor::led_pattern>;

    // Called from inside the sensor machine's hook: the LED machine's
    // process() only queues, its own drain follows on the workqueue
    void notifyEntry(sensor::led_pattern kind) { stateMachine.process(led::pattern{kind}); }

    // observers before the state machine they are injected into
    fsm::timed<mtl::zephyr::QueuedTimer> timeouts;
    mtl::zephyr::TraceLogger tracer;
#if SAMPLE_HAS_LED
    LedDriver driver;
    fsm::QueuedMachine<led::led_table, 4, Work, mtl::zephyr::SpinLock,
                       fsm::timed<mtl::zephyr::QueuedTimer>, LedDriver, mtl::zephyr::TraceLogger>
        stateMachine{timeouts, driver, tracer};
#else
    fsm::QueuedMachine<led::led_table, 4, Work, mtl::zephyr::SpinLock,
                       fsm::timed<mtl::zephyr::QueuedTimer>, mtl::zephyr::TraceLogger>
        stateMachine{timeouts, tracer};
#endif
};

// --- sensor power rail: the other element of the same annotation sets;
// reading -> retrying changes the LED but not the rail, so only the LED
// is notified there
struct PowerRail : fsm::observing<PowerRail> {
    using observes = mtl::typelist<sensor::sensor_power>;

    void notifyEntry(sensor::sensor_power power)
    {
        LOG_INF("sensor power %s", power.on ? "on" : "off");
    }
};

// --- the sensor state machine: its table is filtered by the injected observers
template<typename... OBSERVERs>
using SensorStateMachine =
    fsm::QueuedMachine<sensor::sensor_table<OBSERVERs...>, 4, Work, mtl::zephyr::SpinLock,
                       fsm::timed<mtl::zephyr::QueuedTimer>, OBSERVERs...>;

// Static: kernel objects and machine addresses must stay put. Order:
// timer first (armed before anything is notified), the tracer last (its
// line follows the effects)
fsm::timed<mtl::zephyr::QueuedTimer> timeouts;
VirtualSensor sensor;
LedController leds;
PowerRail rail;
mtl::zephyr::TraceLogger tracer;
#ifdef CONFIG_SAMPLE_CALIBRATION
Calibrator cal;
SensorStateMachine<VirtualSensor, Calibrator, LedController, PowerRail, mtl::zephyr::TraceLogger>
    monitor{timeouts, sensor, cal, leds, rail, tracer};
#else
SensorStateMachine<VirtualSensor, LedController, PowerRail, mtl::zephyr::TraceLogger>
    monitor{timeouts, sensor, leds, rail, tracer};
#endif

// What the observers report, through the queue
void reportReading(int value)
{
    monitor.process(sensor::reading_done{value});
}

void reportReadingFailed()
{
    monitor.process(sensor::reading_failed{});
}

// without the calibration feature the table has no such event: ignored
void reportCalibrated(int offset)
{
    monitor.process(sensor::calibrated{offset});
}

// --- emergency button -------------------------------------------------------
// The ISR processes the event itself: process() only queues it
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
    monitor.process(sensor::button{});
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
    LOG_WRN("no sw0 button on this board: no emergency stop");
    return 0;
}

#endif

} // namespace

int main()
{
#if SAMPLE_HAS_LED
    leds.driver.attach();
#else
    LOG_WRN("no led0 on this board: LED states run unlit");
#endif
    initButton();
    k_sleep(K_FOREVER);
    return 0;
}
