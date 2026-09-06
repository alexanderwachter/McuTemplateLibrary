/*
 * Zephyr timer policies for fsm::timed.
 *
 * IsrTimer runs the expiry - and with it the machine's
 * process(fsm::timeout) - straight from k_timer's ISR context. Use it
 * only when every other event source of the machine is serialized with
 * that ISR (e.g. everything runs under irq_lock or from the same IRQ).
 *
 * WorkqueueTimer runs the expiry from a k_work_delayable on a
 * workqueue (the system workqueue by default). Feeding the machine's
 * other events from the same workqueue serializes everything without
 * further locking - the intended setup.
 *
 * Both tolerate stop() on an unarmed timer and restart on start()
 * (policy contract). Cancelling does not wait for an in-flight
 * handler: mutual exclusion comes from running everything in one
 * context, as the fsm timer contract demands. Instances are pinned:
 * the kernel objects hold their address.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>

#include <zephyr/kernel.h>

#include <chrono>

namespace mtl::zephyr {

class IsrTimer {
public:
    IsrTimer()
    {
        k_timer_init(&timer_, &IsrTimer::expiry, nullptr);
        k_timer_user_data_set(&timer_, this);
    }
    IsrTimer(IsrTimer const&)            = delete;
    IsrTimer& operator=(IsrTimer const&) = delete;

    void start(std::chrono::milliseconds duration, fsm::timer_callback callback, void* context)
    {
        callback_ = callback;
        context_  = context;
        k_timer_start(&timer_, K_MSEC(duration.count()), K_NO_WAIT);
    }

    void stop() { k_timer_stop(&timer_); }

private:
    static void expiry(k_timer* timer)
    {
        auto* self = static_cast<IsrTimer*>(k_timer_user_data_get(timer));
        self->callback_(self->context_);
    }

    k_timer timer_;
    fsm::timer_callback callback_ = nullptr;
    void* context_                = nullptr;
};
static_assert(fsm::concepts::timer<IsrTimer>);

class WorkqueueTimer {
public:
    // nullptr runs on the system workqueue
    explicit WorkqueueTimer(k_work_q* queue = nullptr) : queue_(queue)
    {
        k_work_init_delayable(&work_, &WorkqueueTimer::run);
    }
    WorkqueueTimer(WorkqueueTimer const&)            = delete;
    WorkqueueTimer& operator=(WorkqueueTimer const&) = delete;

    void start(std::chrono::milliseconds duration, fsm::timer_callback callback, void* context)
    {
        callback_ = callback;
        context_  = context;
        if (queue_ != nullptr) {
            k_work_reschedule_for_queue(queue_, &work_, K_MSEC(duration.count()));
        } else {
            k_work_reschedule(&work_, K_MSEC(duration.count()));
        }
    }

    void stop() { k_work_cancel_delayable(&work_); }

private:
    static void run(k_work* work)
    {
        auto* delayable = k_work_delayable_from_work(work);
        auto* self      = CONTAINER_OF(delayable, WorkqueueTimer, work_);
        self->callback_(self->context_);
    }

    k_work_delayable work_;
    k_work_q* queue_;
    fsm::timer_callback callback_ = nullptr;
    void* context_                = nullptr;
};
static_assert(fsm::concepts::timer<WorkqueueTimer>);

} // namespace mtl::zephyr
