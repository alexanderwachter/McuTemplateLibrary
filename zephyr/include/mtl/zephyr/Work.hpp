/*
 * Zephyr policies for fsm::QueuedMachine.
 *
 * WorkOn<queue> is the WORK policy: the machine's drain runs as a
 * k_work item on the named queue, its one serialized context. Submits
 * before the item ran coalesce, as the contract allows. The policy is
 * default-constructed inside the machine, so the queue comes through
 * the type - an object with handle() returning the k_work_q:
 *
 *   mtl::zephyr::WorkQueue<2048, 5> fsm_queue{"fsm"};      // its own thread
 *   using Work = mtl::zephyr::WorkOn<fsm_queue>;
 *
 * or mtl::zephyr::SystemWork for the system workqueue. Declare a
 * WorkQueue before the machines running on it: its constructor starts
 * the thread.
 *
 * SpinLock is the LOCK policy guarding the event FIFO: with both,
 * process() is safe from any context, ISRs included - a button ISR
 * processes its event directly instead of bouncing through a work
 * item of its own. The lock only ever covers the ring's bookkeeping
 * and the copy of one event in or out - no hook, guard or state code
 * runs under it - so a k_spinlock (interrupts masked, other cores
 * kept out) is the right weight.
 *
 * Timers: mtl::zephyr::QueuedTimer (Timer.hpp), one line per observer.
 *
 * Instances are pinned: the kernel objects hold their address.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <cstddef>

namespace mtl::zephyr {

// A workqueue of its own: k_work_q, thread and stack in one static
// object, started by the constructor
template<std::size_t STACK_SIZE, int PRIORITY>
class WorkQueue {
public:
    explicit WorkQueue(char const* name = "fsm")
    {
        k_work_queue_config config{};
        config.name = name;
        k_work_queue_init(&queue_);
        k_work_queue_start(&queue_, stack_, K_KERNEL_STACK_SIZEOF(stack_), PRIORITY, &config);
    }
    WorkQueue(WorkQueue const&)            = delete;
    WorkQueue& operator=(WorkQueue const&) = delete;

    k_work_q* handle() { return &queue_; }

private:
    k_work_q queue_;
    K_KERNEL_STACK_MEMBER(stack_, STACK_SIZE);
};

// The system workqueue, as the queue object WorkOn names
struct SystemQueue {
    static k_work_q* handle() { return &k_sys_work_q; }
};
inline SystemQueue system_queue;

template<auto& QUEUE>
class WorkOn {
public:
    WorkOn() { k_work_init(&work_, &WorkOn::run); }
    WorkOn(WorkOn const&)            = delete;
    WorkOn& operator=(WorkOn const&) = delete;

    // Any context. The machine always submits the same callback and
    // context, so a concurrent submit rewrites identical values
    void submit(fsm::work_callback callback, void* context)
    {
        callback_ = callback;
        context_  = context;
        k_work_submit_to_queue(QUEUE.handle(), &work_);
    }

private:
    static void run(k_work* work)
    {
        auto* self = CONTAINER_OF(work, WorkOn, work_);
        self->callback_(self->context_);
    }

    k_work work_;
    fsm::work_callback callback_ = nullptr;
    void* context_               = nullptr;
};

using SystemWork = WorkOn<system_queue>;
static_assert(fsm::concepts::work_queue<SystemWork>);

// The key lives in the lock: only the holder writes it, and nothing
// else can take the lock before unlock() has read it back
class SpinLock {
public:
    void lock() { key_ = k_spin_lock(&lock_); }
    void unlock() { k_spin_unlock(&lock_, key_); }

private:
    k_spinlock lock_{};
    k_spinlock_key_t key_{};
};
static_assert(fsm::concepts::basic_lockable<SpinLock>);

} // namespace mtl::zephyr
