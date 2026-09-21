/*
 * mtl::zephyr::StateMachine - a state machine on Zephyr in one
 * declaration. It assembles what a queued machine needs and owns it:
 *
 *   - the timeout timer, if the table has a timed state
 *   - the deadline timer, if the table has a state with a deadline
 *   - the workqueue the machine drains on: its own thread, named after
 *     the table, or one the caller shares between machines
 *   - the fsm::QueuedMachine itself, its FIFO under a spinlock
 *
 * so process() is safe from any context, ISRs included, and the caller
 * only names the table and hands over its observers, whose types are
 * deduced:
 *
 *   mtl::zephyr::StateMachine light{mtl::zephyr::table<light_table>, lamps, tracer};
 *
 * (class template argument deduction cannot take the table explicitly
 * and deduce the rest, hence the table as a tag value). A table that
 * is a template over the observers - one filtered by the features they
 * enable - is named by table_for:
 *
 *   mtl::zephyr::StateMachine monitor{mtl::zephyr::table_for<sensor_table>, leds, rail};
 *
 * and a WorkQueue after the tag shares that queue instead of owning one:
 *
 *   mtl::zephyr::WorkQueue<2048, 5> queue{"fsm"};
 *   mtl::zephyr::StateMachine a{mtl::zephyr::table<table_a>, queue, observer};
 *   mtl::zephyr::StateMachine b{mtl::zephyr::table<table_b>, queue};
 *
 * The event buffer's capacity, and an owned workqueue's stack size and
 * priority, default to Kconfig (MTL_FSM_EVENT_BUFFER_CAPACITY,
 * MTL_FSM_WORKQUEUE_*); a machine overrides them by name, after the
 * table:
 *
 *   using mtl::zephyr::machine_config;
 *   mtl::zephyr::StateMachine big{mtl::zephyr::table<big_table>,
 *                                 mtl::zephyr::config<machine_config{.event_buffer_capacity = 8}>,
 *                                 observer};
 *
 * Deduction is not available for a class member; spell it with
 * StateMachineWithOwnWorkqueue<TABLE, OBSERVERs...> or
 * StateMachineOnSharedWorkqueue<TABLE, OBSERVERs...>.
 *
 * The timers come first in the machine's observer order, so they are
 * armed before anything is notified. Instances are pinned; construct
 * from the thread that starts the system (a static object is fine),
 * before events can arrive.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>
#include <mtl/TypeName.hpp>
#include <mtl/Typelist.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/zephyr/Timer.hpp>
#include <mtl/zephyr/Work.hpp>

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

namespace mtl::zephyr {

// What a machine may tune, the rest from Kconfig:
//   mtl::zephyr::config<mtl::zephyr::machine_config{.event_buffer_capacity = 8}>
struct machine_config {
    // events waiting for the workqueue; timer expiries take no slot
    std::size_t event_buffer_capacity = CONFIG_MTL_FSM_EVENT_BUFFER_CAPACITY;
    // of the thread of a workqueue the machine owns
    std::size_t workqueue_stack_size = CONFIG_MTL_FSM_WORKQUEUE_STACK_SIZE;
    int workqueue_priority           = CONFIG_MTL_FSM_WORKQUEUE_PRIORITY;
};

template<machine_config CONFIG>
struct config_tag {};

template<machine_config CONFIG>
inline constexpr config_tag<CONFIG> config{};

// The table, as a tag value: a plain table, or a template over the
// observers (one filtered by the features they enable)
template<typename TABLE>
struct fixed_table {
    template<typename...>
    using type = TABLE;
};

template<template<typename...> typename TABLE>
struct table_template {
    template<typename... OBSERVERs>
    using type = TABLE<OBSERVERs...>;
};

template<typename MAKER>
struct table_tag {};

template<typename TABLE>
inline constexpr table_tag<fixed_table<TABLE>> table{};

template<template<typename...> typename TABLE>
inline constexpr table_tag<table_template<TABLE>> table_for{};

// Whose thread the machine drains on: its own, or a caller's WorkQueue
struct own_workqueue {};
struct shared_workqueue {};

namespace internal {

template<typename KIND, machine_config CONFIG>
struct queue_holder;

template<machine_config CONFIG>
struct queue_holder<own_workqueue, CONFIG> {
    explicit queue_holder(char const* name) : queue(name) {}
    k_work_q* handle() { return queue.handle(); }

    WorkQueue<CONFIG.workqueue_stack_size, CONFIG.workqueue_priority> queue;
};

template<machine_config CONFIG>
struct queue_holder<shared_workqueue, CONFIG> {
    explicit queue_holder(k_work_q* queue_handle) : queue(queue_handle) {}
    k_work_q* handle() { return queue; }

    k_work_q* queue;
};

template<bool WANTED, typename OBSERVER>
using observer_if_t = std::conditional_t<WANTED, mtl::typelist<OBSERVER>, mtl::typelist<>>;

template<typename TABLE, std::size_t CAPACITY, typename TIMERS, typename... OBSERVERs>
struct queued_machine;

template<typename TABLE, std::size_t CAPACITY, typename... TIMERs, typename... OBSERVERs>
struct queued_machine<TABLE, CAPACITY, mtl::typelist<TIMERs...>, OBSERVERs...> {
    using type = fsm::QueuedMachine<TABLE, CAPACITY, Work&, SpinLock, TIMERs..., OBSERVERs...>;
};

} // namespace internal

template<typename TABLE, typename QUEUE, machine_config CONFIG, typename... OBSERVERs>
class StateMachine {
    using timed_type     = fsm::timed<QueuedTimer>;
    using deadlined_type = fsm::deadlined<QueuedTimer>;

    static constexpr bool has_timeouts  = fsm::has_timed_states_v<TABLE>;
    static constexpr bool has_deadlines = fsm::has_deadlined_states_v<TABLE>;

    using timers = mtl::concat_t<internal::observer_if_t<has_timeouts, timed_type>,
                                 internal::observer_if_t<has_deadlines, deadlined_type>>;

public:
    using table = TABLE;
    using machine_type =
        typename internal::queued_machine<TABLE, CONFIG.event_buffer_capacity, timers,
                                          OBSERVERs...>::type;

    // The tags only carry the table and the configuration into the
    // deduction guides below
    template<typename TAG>
    explicit StateMachine(TAG, OBSERVERs&... observers)
        requires std::same_as<QUEUE, own_workqueue>
        : queue_(mtl::short_name_of<TABLE>), work_(queue_.handle()),
          machine_(this->construct(observers...))
    {
    }

    template<typename TAG>
    StateMachine(TAG, config_tag<CONFIG>, OBSERVERs&... observers)
        requires std::same_as<QUEUE, own_workqueue>
        : StateMachine(TAG{}, observers...)
    {
    }

    template<typename TAG, std::size_t STACK_SIZE, int PRIORITY>
    StateMachine(TAG, WorkQueue<STACK_SIZE, PRIORITY>& queue, OBSERVERs&... observers)
        requires std::same_as<QUEUE, shared_workqueue>
        : queue_(queue.handle()), work_(queue_.handle()), machine_(this->construct(observers...))
    {
    }

    template<typename TAG, std::size_t STACK_SIZE, int PRIORITY>
    StateMachine(TAG, config_tag<CONFIG>, WorkQueue<STACK_SIZE, PRIORITY>& queue,
                 OBSERVERs&... observers)
        requires std::same_as<QUEUE, shared_workqueue>
        : StateMachine(TAG{}, queue, observers...)
    {
    }

    // From any context: the event is queued and delivered on the
    // machine's workqueue. False when the FIFO is full (the event is
    // lost) or the table does not know the event
    template<typename EVENT>
    bool process(EVENT const& event)
    {
        return machine_.process(event);
    }

    template<typename STATE>
    [[nodiscard]] bool is() const
    {
        return machine_.template is<STATE>();
    }

    template<typename STATE>
    [[nodiscard]] STATE const* getIf() const
    {
        return machine_.template getIf<STATE>();
    }

    template<typename T>
    [[nodiscard]] T const& context() const
    {
        return machine_.template context<T>();
    }

private:
    // A prvalue: the pinned machine is constructed in place
    machine_type construct(OBSERVERs&... observers)
    {
        return std::make_from_tuple<machine_type>(std::tuple_cat(
            std::tuple<Work&>(work_), this->timerObservers(), std::tie(observers...)));
    }

    auto timerObservers()
    {
        if constexpr (has_timeouts && has_deadlines) {
            return std::tie(timed_, deadlined_);
        } else if constexpr (has_timeouts) {
            return std::tie(timed_);
        } else if constexpr (has_deadlines) {
            return std::tie(deadlined_);
        } else {
            return std::tuple<>{};
        }
    }

    // In dependency order: the queue, the work item on it, the timers,
    // then the machine - whose construction already arms and notifies
    internal::queue_holder<QUEUE, CONFIG> queue_;
    Work work_;
    [[no_unique_address]] std::conditional_t<has_timeouts, timed_type, mtl::nil_type> timed_{};
    [[no_unique_address]] std::conditional_t<has_deadlines, deadlined_type, mtl::nil_type>
        deadlined_{};
    machine_type machine_;
};

template<typename MAKER, typename... OBSERVERs>
StateMachine(table_tag<MAKER>, OBSERVERs&...)
    -> StateMachine<typename MAKER::template type<OBSERVERs...>, own_workqueue, machine_config{},
                    OBSERVERs...>;

template<typename MAKER, machine_config CONFIG, typename... OBSERVERs>
StateMachine(table_tag<MAKER>, config_tag<CONFIG>, OBSERVERs&...)
    -> StateMachine<typename MAKER::template type<OBSERVERs...>, own_workqueue, CONFIG, OBSERVERs...>;

template<typename MAKER, std::size_t STACK_SIZE, int PRIORITY, typename... OBSERVERs>
StateMachine(table_tag<MAKER>, WorkQueue<STACK_SIZE, PRIORITY>&, OBSERVERs&...)
    -> StateMachine<typename MAKER::template type<OBSERVERs...>, shared_workqueue, machine_config{},
                    OBSERVERs...>;

template<typename MAKER, machine_config CONFIG, std::size_t STACK_SIZE, int PRIORITY,
         typename... OBSERVERs>
StateMachine(table_tag<MAKER>, config_tag<CONFIG>, WorkQueue<STACK_SIZE, PRIORITY>&,
             OBSERVERs&...)
    -> StateMachine<typename MAKER::template type<OBSERVERs...>, shared_workqueue, CONFIG,
                    OBSERVERs...>;

// Where deduction is not available (a machine as a class member)
template<typename TABLE, typename... OBSERVERs>
using StateMachineWithOwnWorkqueue = StateMachine<TABLE, own_workqueue, machine_config{}, OBSERVERs...>;

template<typename TABLE, typename... OBSERVERs>
using StateMachineOnSharedWorkqueue = StateMachine<TABLE, shared_workqueue, machine_config{}, OBSERVERs...>;

} // namespace mtl::zephyr
