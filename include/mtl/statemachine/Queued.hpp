/*
 * fsm: the queued machine - run-to-completion event delivery
 *
 * fsm::QueuedMachine owns the state machine as a private member and
 * is the only way in: process() puts the event into a bounded FIFO
 * and triggers the WORK policy, whose callback drains the queue one
 * completed transition at a time. An event processed from within a
 * delivery (a driver reporting synchronously from an entry hook) is
 * thereby queued and fires right after the current transition
 * completes - the re-entrancy the raw machine's contract forbids
 * becomes ordered delivery.
 *
 * WORK policy contract: submit(callback, context) arranges for
 * callback(context) to run in the machine's ONE serialized context;
 * submits before it ran may coalesce. The default, inline_work, calls
 * back immediately: the single-context integration, where process()
 * itself must only be called from that context. A real work queue
 * (e.g. Zephyr k_work) makes process() safe from any context, ISRs
 * included, when a real LOCK policy guards the FIFO alongside it.
 *
 * Timers: fsm::QueuedTimer adapts a platform timer to the queue, and
 * the fsm::timed/fsm::deadlined observer injected into a queued
 * machine must run on one (statically checked; at most one of each -
 * states carry a single timeout and a single deadline annotation),
 * with one channel per machine level (fsm::timed<TIMER, LEVELS>).
 * The expiry callback (any context, ISRs included) only latches the
 * channel's pending flag and triggers WORK - the drain finds the flag
 * and delivers the callback the observer armed the channel with, to
 * the machine it armed it for: the root's fsm::timeout, or a
 * submachine's, which never passes the parent's dispatch. So timed
 * and deadlined work unchanged on top. The latch, not the
 * FIFO, carries expiries: stop() and start() clear it, which retracts
 * a stale expiry exactly when the arming that produced it is gone (a
 * queued event that leaves a timed state stops the timer before the
 * expiry could be delivered - the wrong-state timeout cannot happen,
 * while an expiry whose arming survives the queued events is
 * delivered; leaving a composite state stops its child's slot). The
 * observer's kind decides the expiry's place relative to queued
 * events, and among a kind's channels the outer level goes first:
 *   fsm::timed     after the FIFO - an event that arrived before the
 *                  expiry wins; arrival order is the on-wire truth
 *                  for response timers
 *   fsm::deadlined before the FIFO - the budget gates progress: no
 *                  queued event may advance a phase whose deadline
 *                  has passed
 * Timer policy contract addition: after stop() returns, the expiry
 * callback is neither running nor pending.
 *
 * Observer hooks still receive the raw machine reference (for
 * getIf()/context()); calling process() on it from a hook remains
 * forbidden and is caught by the machine's re-entrancy assert.
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Core.hpp>
#include <mtl/statemachine/Timer.hpp>
#include <mtl/TypelistAlgorithms.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

namespace fsm {

using work_callback = void (*)(void*);

namespace concepts {

template<typename T>
concept work_queue = requires(T work, work_callback callback, void* context) {
    work.submit(callback, context);
};

template<typename T>
concept basic_lockable = requires(T lock) {
    lock.lock();
    lock.unlock();
};

} // namespace concepts

// The default WORK: callback right away, in the caller's context -
// process() may then only be called from the one serialized context
struct inline_work {
    void submit(work_callback callback, void* context) { callback(context); }
};

// The default LOCK: single-context integrations need none
struct no_lock {
    void lock() {}
    void unlock() {}
};

// One timer channel: the latch plus the callback it stands for, free
// of the TIMER type. bind() and deliver() are the QueuedMachine's
// wiring, not for users
class QueuedTimerBase {
public:
    void bind(void* queue, work_callback notify)
    {
        queue_  = queue;
        notify_ = notify;
    }

    // Deliver a pending expiry: true once per expiry, having run the
    // callback the observer armed the channel with - the machine's
    // process(fsm::timeout) or process(fsm::deadline), the machine
    // being the level whose state was armed. Serialized context only.
    // The latch is loaded and cleared, not exchanged: a new expiry
    // needs a re-arm, which happens in this same context - nothing can
    // set the latch between the two (and Cortex-M0-class cores inline
    // atomic loads and stores but need a library call for
    // read-modify-write)
    bool deliver()
    {
        if (!pending_.load(std::memory_order_acquire)) {
            return false;
        }
        pending_.store(false, std::memory_order_relaxed);
        callback_(context_);
        return true;
    }

    QueuedTimerBase(QueuedTimerBase const&)            = delete;
    QueuedTimerBase& operator=(QueuedTimerBase const&) = delete;

protected:
    QueuedTimerBase() = default;

    void* queue_             = nullptr;
    work_callback notify_    = nullptr;
    timer_callback callback_ = nullptr;
    void* context_           = nullptr;
    std::atomic<bool> pending_{false};
};

// Adapts a platform TIMER to a queue: satisfies the timer policy
// contract, so fsm::timed/fsm::deadlined run on it like on the timer
// itself. The QueuedMachine binds it at construction, before any
// state could arm it
template<concepts::timer TIMER>
class QueuedTimer : public QueuedTimerBase {
public:
    explicit QueuedTimer(TIMER& timer) : timer_(timer) {}

    // The callback and context the observer arms with are kept for
    // the drain: behind a queue the expiry is delivered from the
    // serialized context, to the machine the observer armed for
    void start(std::chrono::milliseconds duration, timer_callback callback, void* context)
    {
        timer_.stop(); // an expiry of the previous arming must not leak into this one
        pending_.store(false, std::memory_order_relaxed);
        callback_ = callback;
        context_  = context;
        timer_.start(duration, &QueuedTimer::expired, this);
    }

    void stop()
    {
        timer_.stop(); // contract: the callback is not running nor pending after this
        pending_.store(false, std::memory_order_relaxed);
    }

private:
    // The platform timer's context - only latches and notifies
    static void expired(void* self)
    {
        auto& channel = *static_cast<QueuedTimer*>(self);
        channel.pending_.store(true, std::memory_order_release);
        channel.notify_(channel.queue_);
    }

    TIMER& timer_;
};

namespace internal {

// Base-from-member: the owned timer exists before the QueuedTimer
// referring to it
template<typename TIMER>
struct timer_holder {
    TIMER platform_timer{};

protected:
    timer_holder() = default;
};

} // namespace internal

// A QueuedTimer owning its platform timer: default-constructible, so
// the observer is declared in one line, as on the raw machine -
//   fsm::timed<fsm::OwningQueuedTimer<my_timer>> timeouts;
// QueuedTimer<TIMER> remains for a timer the caller owns or configures
template<concepts::timer TIMER>
    requires std::default_initializable<TIMER>
class OwningQueuedTimer : private internal::timer_holder<TIMER>, public QueuedTimer<TIMER> {
public:
    OwningQueuedTimer() : QueuedTimer<TIMER>(internal::timer_holder<TIMER>::platform_timer) {}

    TIMER& platformTimer() { return internal::timer_holder<TIMER>::platform_timer; }
};

namespace internal {

template<typename OBSERVER>
struct is_timed_observer : std::false_type {};
template<typename TIMER, std::size_t LEVELS>
struct is_timed_observer<timed<TIMER, LEVELS>> : std::true_type {};

template<typename OBSERVER>
struct is_deadlined_observer : std::false_type {};
template<typename TIMER, std::size_t LEVELS>
struct is_deadlined_observer<deadlined<TIMER, LEVELS>> : std::true_type {};

// QueuedTimer<TIMER> or a class built on one (OwningQueuedTimer)
template<typename T>
struct is_queued_timer : std::is_base_of<QueuedTimerBase, T> {};

// A timed/deadlined observer in a queued machine must run on a
// QueuedTimer - a raw platform timer would fire straight into the
// machine, outside the queue
template<typename OBSERVER>
struct queue_compatible : std::true_type {};
template<typename TIMER, std::size_t LEVELS>
struct queue_compatible<timed<TIMER, LEVELS>> : is_queued_timer<std::remove_reference_t<TIMER>> {};
template<typename TIMER, std::size_t LEVELS>
struct queue_compatible<deadlined<TIMER, LEVELS>>
    : is_queued_timer<std::remove_reference_t<TIMER>> {};

template<typename OBSERVER>
inline constexpr bool queue_compatible_v = queue_compatible<OBSERVER>::value;

// The channels a timer observer brings - one per machine level - and
// none for any other observer
template<typename OBSERVER>
inline constexpr std::size_t timer_channels_v = 0;
template<typename TIMER, std::size_t LEVELS>
inline constexpr std::size_t timer_channels_v<timed<TIMER, LEVELS>> = LEVELS;
template<typename TIMER, std::size_t LEVELS>
inline constexpr std::size_t timer_channels_v<deadlined<TIMER, LEVELS>> = LEVELS;

// Timer events never travel through the FIFO - expiries live in the
// channels' latches - so the ring's variant leaves them out: their
// delivery arms would duplicate the heaviest per-event dispatch
// (every timed state and its guards), which the latch path already
// instantiates (measured ~2.4 kB on the pd_drp sample)
template<typename EVENT>
struct is_timer_event
    : std::bool_constant<std::is_same_v<EVENT, timeout> || std::is_same_v<EVENT, deadline>> {};

} // namespace internal

// The queue-owning machine: TABLE and OBSERVERs as in
// fsm::StateMachine, CAPACITY bounds the FIFO (expiries live in the
// timer channels' latches and cannot overflow it), WORK runs the
// drain, LOCK guards the FIFO against foreign-context process() calls
template<typename TABLE, std::size_t CAPACITY, concepts::work_queue WORK = inline_work,
         concepts::basic_lockable LOCK = no_lock, typename... OBSERVERs>
class QueuedMachine {
public:
    using machine_type = StateMachine<TABLE, OBSERVERs...>;
    // Every level's events: a submachine's are processed at the root
    // and descend. fsm::timeout and fsm::deadline enter through the
    // latches, never the ring: the variant leaves them out
    using queueable_events =
        mtl::remove_if_t<nested_events_t<TABLE>, internal::is_timer_event>;
    using event_variant =
        mtl::rebind_t<mtl::prepend_t<std::monostate, queueable_events>, std::variant>;

    static_assert(CAPACITY > 0, "QueuedMachine: CAPACITY must be at least 1");
    static_assert(mtl::all_of_v<queueable_events, std::is_copy_constructible>,
                  "QueuedMachine: every event of the table must be copy constructible");
    static_assert((internal::queue_compatible_v<OBSERVERs> && ...),
                  "QueuedMachine: timed/deadlined observers must run on fsm::QueuedTimer");
    static_assert((std::size_t{internal::is_timed_observer<OBSERVERs>::value} + ... +
                   std::size_t{0}) <= 1,
                  "QueuedMachine: at most one fsm::timed observer (one timeout annotation)");
    static_assert((std::size_t{internal::is_deadlined_observer<OBSERVERs>::value} + ... +
                   std::size_t{0}) <= 1,
                  "QueuedMachine: at most one fsm::deadlined observer (one deadline annotation)");

    // Construction counts as a delivery: the initial state's hooks may
    // already process events - they are drained before this returns.
    // Construct from the serialized context, before event sources run
    explicit QueuedMachine(OBSERVERs&... observers) // channels bind before the
        requires(!std::is_reference_v<WORK>)
        : machine_((this->bindChannels(observers), observers)...) // initial state can arm
    {
        draining_ = false;
        this->drain();
    }

    // QueuedMachine<TABLE, N, WORK&, ...> runs on a caller-owned WORK,
    // for policies that need configuration (the queue to run on) -
    // the fsm::timed<POLICY&> convention
    explicit QueuedMachine(WORK work, OBSERVERs&... observers)
        requires std::is_reference_v<WORK>
        : work_(work), machine_((this->bindChannels(observers), observers)...)
    {
        draining_ = false;
        this->drain();
    }

    // The one way in, from any context the WORK and LOCK policies
    // cover: enqueue, then let the work queue drain. Returns false
    // when the FIFO is full and the event is lost (debug-asserted) -
    // size CAPACITY for the worst burst; true means fired or queued.
    // An event no transition of the table mentions is ignored, like
    // on the raw machine
    template<typename EVENT>
    bool process(EVENT const& event)
    {
        if constexpr (!mtl::has_a_v<queueable_events, EVENT>) {
            return false;
        } else {
            return this->enqueue(event);
        }
    }

    // Read-only views of the machine, as on fsm::StateMachine
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

    template<typename T>
    [[nodiscard]] std::optional<T> annotation() const
    {
        return machine_.template annotation<T>();
    }

    template<typename STATE>
    [[nodiscard]] auto const* submachine() const
    {
        return machine_.template submachine<STATE>();
    }

private:
    static void drainHook(void* self) { static_cast<QueuedMachine*>(self)->drain(); }

    // Timer expiries land here through the channels' notify binding
    static void notifyHook(void* self)
    {
        auto& queue = *static_cast<QueuedMachine*>(self);
        queue.work_.submit(&QueuedMachine::drainHook, &queue);
    }

    // Deliver until nothing is pending; no-op while a drain is running
    // (a process() from a hook only enqueues - the running drain
    // delivers it after the current transition)
    // Whether the observer pack brings the channels at all, and how
    // many (one per machine level): without any, the kind's delivery
    // check drops out of the drain entirely
    static constexpr bool has_timeout =
        (internal::is_timed_observer<OBSERVERs>::value || ...);
    static constexpr bool has_deadline =
        (internal::is_deadlined_observer<OBSERVERs>::value || ...);
    template<template<typename> typename KIND>
    static constexpr std::size_t channelsOf()
    {
        return ((KIND<OBSERVERs>::value ? internal::timer_channels_v<OBSERVERs> : 0) + ... +
                std::size_t{0});
    }
    static constexpr std::size_t timeout_channels =
        QueuedMachine::channelsOf<internal::is_timed_observer>();
    static constexpr std::size_t deadline_channels =
        QueuedMachine::channelsOf<internal::is_deadlined_observer>();

    template<std::size_t CHANNELS>
    using channels = std::array<QueuedTimerBase*, CHANNELS == 0 ? 1 : CHANNELS>;

    void drain()
    {
        if (draining_) {
            return;
        }
        draining_ = true;
        while (true) {
            if constexpr (QueuedMachine::has_deadline) {
                if (QueuedMachine::deliverAny(deadline_)) {
                    continue;
                }
            }
            event_variant event{}; // copied out: a delivery may refill the slot
            if (this->popInto(event)) {
                internal::dispatch(
                    [this](auto const& popped) -> bool {
                        using event_type = std::decay_t<decltype(popped)>;
                        if constexpr (!std::is_same_v<event_type, std::monostate>) {
                            return machine_.process(popped);
                        } else {
                            return false;
                        }
                    },
                    event);
                continue;
            }
            if constexpr (QueuedMachine::has_timeout) {
                if (QueuedMachine::deliverAny(timeout_)) {
                    continue;
                }
            }
            break;
        }
        draining_ = false;
    }

    // The first pending channel of a kind delivers, outer level first:
    // a parent's expiry leaving its composite state stops the child's
    // slot, which retracts the child's pending expiry
    template<std::size_t CHANNELS>
    static bool deliverAny(std::array<QueuedTimerBase*, CHANNELS>& kind)
    {
        for (std::size_t level = 0; level < CHANNELS; ++level) {
            if (kind[level]->deliver()) {
                return true;
            }
        }
        return false;
    }

    // Split so the ring bookkeeping is one shared body: only the
    // event's emplace stays with each per-event instantiation. The
    // lock is held from a successful acquire until the commit
    template<typename EVENT>
    bool enqueue(EVENT const& event)
    {
        event_variant* const slot = this->acquireSlot();
        if (slot == nullptr) {
            return false;
        }
        slot->template emplace<EVENT>(event);
        this->commitSlot();
        return true;
    }

    event_variant* acquireSlot()
    {
        lock_.lock();
        if (count_ == CAPACITY) {
            lock_.unlock();
            MTL_FSM_ASSERT(false, "QueuedMachine: event queue overflow");
            return nullptr;
        }
        return &ring_[(read_ + count_) % CAPACITY];
    }

    void commitSlot()
    {
        ++count_;
        lock_.unlock();
        work_.submit(&QueuedMachine::drainHook, this);
    }

    bool popInto(event_variant& event)
    {
        lock_.lock();
        if (count_ == 0) {
            lock_.unlock();
            return false;
        }
        event = ring_[read_];
        read_ = (read_ + 1) % CAPACITY;
        --count_;
        lock_.unlock();
        return true;
    }

    // The timed/deadlined observers are known types: their channels,
    // one per level, are bound to this queue before the machine (and
    // with it the initial state, which may arm them) is constructed
    template<typename OBSERVER>
    void bindChannels(OBSERVER& observer)
    {
        if constexpr (internal::is_deadlined_observer<OBSERVER>::value) {
            this->bindLevels(observer, deadline_);
        } else if constexpr (internal::is_timed_observer<OBSERVER>::value) {
            this->bindLevels(observer, timeout_);
        }
    }

    template<typename OBSERVER, std::size_t CHANNELS>
    void bindLevels(OBSERVER& observer, std::array<QueuedTimerBase*, CHANNELS>& kind)
    {
        for (std::size_t level = 0; level < OBSERVER::levels; ++level) {
            QueuedTimerBase& channel = observer.timer(level);
            channel.bind(this, &QueuedMachine::notifyHook);
            kind[level] = &channel;
        }
    }

    // Declared before machine_: the initial state's hooks may already
    // enqueue events or trigger the work queue
    [[no_unique_address]] LOCK lock_{};
    [[no_unique_address]] WORK work_{}; // a WORK& is bound by its constructor instead
    std::array<event_variant, CAPACITY> ring_{};
    std::size_t read_           = 0;
    std::size_t count_          = 0;
    bool draining_              = true; // construction counts as a delivery
    channels<QueuedMachine::deadline_channels> deadline_{};
    channels<QueuedMachine::timeout_channels> timeout_{};
    machine_type machine_;
};

} // namespace fsm
