/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <chrono>
#include <print>
#include <source_location>
#include <type_traits>
#include <variant>
#include <vector>

using namespace std::chrono_literals;

namespace {

// --- timer policy (host/test implementation) --------------------------------
struct manual_timer {
    std::chrono::milliseconds duration{};
    fsm::timer_callback callback = nullptr;
    void* context                = nullptr;
    bool armed                   = false;

    void start(std::chrono::milliseconds d, fsm::timer_callback cb, void* ctx)
    {
        duration = d;
        callback = cb;
        context  = ctx;
        armed    = true;
    }
    void stop() { armed = false; }
    void expire()
    {
        if (armed) {
            armed = false;
            callback(context);
        }
    }
};
static_assert(fsm::concepts::timer<manual_timer>);

// --- outputs and the controller observing them ------------------------------
struct outputs_t {
    bool led;
    bool fan;
    constexpr bool operator==(outputs_t const&) const = default;
};

// Records notifications for the checks below
struct output_controller : fsm::observing<output_controller> {
    template<typename STATE>
    static constexpr auto observe_static() -> decltype(STATE::outputs)
    {
        return STATE::outputs;
    }

    void notifyEntry(outputs_t const& out) { log.push_back(out); }
    void notifyExit(outputs_t const& out) { exit_log.push_back(out); }

    std::vector<outputs_t> log;
    std::vector<outputs_t> exit_log;
};

// --- events and states ------------------------------------------------------
struct button_press {};
struct lock_key {};

struct off {
    static constexpr outputs_t outputs{.led = false, .fan = false};
};

struct running {
    static constexpr outputs_t outputs{.led = true, .fan = true};
    static constexpr auto timeout = 50ms;
};

struct cooldown { // fan keeps running after the led went off
    static constexpr outputs_t outputs{.led = false, .fan = true};
    static constexpr auto timeout = 100ms;
};

struct locked { // same outputs as off -> transition must NOT notify
    static constexpr outputs_t outputs{.led = false, .fan = false};
};

using table = fsm::transition_table<
    fsm::transition<fsm::from<off>,      fsm::on<button_press>, fsm::to<running>>,
    fsm::transition<fsm::from<running>,  fsm::on<button_press>, fsm::to<off>>,
    fsm::transition<fsm::from<running>,  fsm::on<fsm::timeout>, fsm::to<cooldown>>,
    fsm::transition<fsm::from<cooldown>, fsm::on<fsm::timeout>, fsm::to<off>>,
    fsm::transition<fsm::from<off>,      fsm::on<lock_key>,     fsm::to<locked>>,
    fsm::transition<fsm::from<locked>,   fsm::on<lock_key>,     fsm::to<off>>>;

using machine = fsm::state_machine<table, fsm::timed<manual_timer>, output_controller>;

} // namespace

// --- compile-time checks ----------------------------------------------------

namespace TransitionRoles {
    using reference = fsm::transition<fsm::from<off>, fsm::on<button_press>, fsm::to<running>>;
    using reordered = fsm::transition<fsm::on<button_press>, fsm::to<running>, fsm::from<off>>;

    static_assert(std::is_same_v<reference::from, off>);
    static_assert(std::is_same_v<reference::event, button_press>);
    static_assert(std::is_same_v<reference::to, running>);
    // the named arguments may appear in any order
    static_assert(std::is_same_v<reordered::from, reference::from>);
    static_assert(std::is_same_v<reordered::event, reference::event>);
    static_assert(std::is_same_v<reordered::to, reference::to>);
} // namespace TransitionRoles

namespace Table {
    // deduplicated, in order of first appearance -> off is the initial state
    static_assert(std::is_same_v<table::states, mtl::typelist<off, running, cooldown, locked>>);

    static_assert(std::is_same_v<table::find_transition<off, button_press>::to, running>);
    static_assert(std::is_same_v<table::find_transition<cooldown, fsm::timeout>::to, off>);
    // pairs not in the table resolve to nil_type
    static_assert(std::is_same_v<table::find_transition<running, lock_key>, mtl::nil_type>);
    static_assert(std::is_same_v<table::find_transition<locked, button_press>, mtl::nil_type>);
} // namespace Table

namespace Concepts {
    static_assert(fsm::concepts::state<off> && fsm::concepts::state<running>);
    static_assert(!fsm::concepts::state<int>);
    static_assert(fsm::concepts::transition_table<table>);
    static_assert(!fsm::concepts::transition_table<off>);
} // namespace Concepts

namespace MachineTypes {
    static_assert(std::is_same_v<machine::initial_state, off>);
    static_assert(std::is_same_v<machine::state_variant,
                                 std::variant<off, running, cooldown, locked>>);
    // observers may retain the machine address from a hook
    static_assert(!std::is_copy_constructible_v<machine>);
    static_assert(!std::is_move_constructible_v<machine>);
} // namespace MachineTypes

namespace ExplicitInitial {
    using lock_first = fsm::transition_table<
        fsm::initial<locked>,
        fsm::transition<fsm::from<off>,    fsm::on<lock_key>, fsm::to<locked>>,
        fsm::transition<fsm::from<locked>, fsm::on<lock_key>, fsm::to<off>>>;

    // the chosen state moves to the front and becomes the initial state
    static_assert(std::is_same_v<lock_first::states, mtl::typelist<locked, off>>);
    static_assert(std::is_same_v<fsm::state_machine<lock_first>::initial_state, locked>);

    // initial<> may appear anywhere in the table
    using reordered = fsm::transition_table<
        fsm::transition<fsm::from<off>,    fsm::on<lock_key>, fsm::to<locked>>,
        fsm::initial<locked>,
        fsm::transition<fsm::from<locked>, fsm::on<lock_key>, fsm::to<off>>>;
    static_assert(std::is_same_v<reordered::states, lock_first::states>);
    static_assert(std::is_same_v<reordered::transitions, lock_first::transitions>);
} // namespace ExplicitInitial

namespace Guards {
    struct always { static bool check() { return true; } };

    using unguarded = fsm::transition<fsm::from<off>, fsm::on<button_press>, fsm::to<running>>;
    using guarded   = fsm::transition<fsm::from<off>, fsm::on<button_press>, fsm::to<running>,
                                      fsm::guard<always>>;
    using reordered = fsm::transition<fsm::guard<always>, fsm::to<running>,
                                      fsm::from<off>, fsm::on<button_press>>;

    static_assert(std::is_same_v<unguarded::guard, mtl::nil_type>);
    static_assert(std::is_same_v<guarded::guard, always>);
    static_assert(std::is_same_v<reordered::guard, always>);

    struct with_state { static bool check(off const&) { return true; } };
    static_assert(fsm::concepts::guard_for<with_state, off>);
    static_assert(!fsm::concepts::guard_for<with_state, running>); // wrong state
    static_assert(fsm::concepts::guard_for<always, off>); // no-argument form
    static_assert(!fsm::concepts::guard_for<off, off>); // no check member

    // a templated check(auto const&) is a guard shared by several states
    struct generic { static bool check(auto const&) { return true; } };
    static_assert(fsm::concepts::guard_for<generic, off>);
    static_assert(fsm::concepts::guard_for<generic, running>);

    struct with_event { static bool check(off const&, button_press const&) { return true; } };
    static_assert(fsm::concepts::guard_for<with_event, off>); // (state, event) form
    static_assert(!fsm::concepts::guard_for<with_event, running>); // wrong state
} // namespace Guards

namespace Wildcard {
    struct advance {};
    struct shutdown {};
    struct idle {};
    struct stage1 {};
    struct stage2 {};

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>,           fsm::on<advance>,  fsm::to<stage1>>,
        fsm::transition<fsm::from<stage1>,         fsm::on<advance>,  fsm::to<stage2>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<shutdown>, fsm::to<idle>>>;

    // any_state is not a state of the machine
    static_assert(std::is_same_v<tbl::states, mtl::typelist<idle, stage1, stage2>>);
    // the wildcard matches states without an exact (state, event) pair
    static_assert(std::is_same_v<tbl::find_transition<stage2, shutdown>::to, idle>);
    static_assert(std::is_same_v<tbl::find_transition<stage1, advance>::to, stage2>);

    // an exact pair takes precedence over the wildcard
    using with_override = fsm::transition_table<
        fsm::transition<fsm::from<idle>,           fsm::on<advance>,  fsm::to<stage1>>,
        fsm::transition<fsm::from<stage1>,         fsm::on<shutdown>, fsm::to<stage2>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<shutdown>, fsm::to<idle>>>;
    static_assert(std::is_same_v<with_override::find_transition<stage1, shutdown>::to, stage2>);
    static_assert(std::is_same_v<with_override::find_transition<stage2, shutdown>::to, idle>);
} // namespace Wildcard

namespace Payload {
    struct message { int id; };
    struct send { message msg; };
    struct cancel {};

    struct idle {};
    struct sending {
        sending() = default;
        explicit sending(send const& event) : msg(event.msg) {}
        message msg{};
    };

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<send>,   fsm::to<sending>>,
        fsm::transition<fsm::from<sending>, fsm::on<cancel>, fsm::to<idle>>>;

    // Driver observer: reads the payload delivered into the sending state
    struct tx_driver {
        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onEnterState(MACHINE& machine)
        {
            if constexpr (std::is_same_v<NEW_STATE, sending>) {
                transmitted.push_back(machine.template getIf<sending>()->msg.id);
            }
        }

        std::vector<int> transmitted;
    };

    // observing-based counterpart to tx_driver: names the watched member
    // once, gets the live payload without getIf plumbing
    struct live_driver : fsm::observing<live_driver> {
        static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.msg))
        {
            return state.msg;
        }
        void notifyEntry(message const& msg) { entered.push_back(msg.id); }
        void notifyExit(message const& msg) { exited.push_back(msg.id); }

        std::vector<int> entered;
        std::vector<int> exited;
    };
} // namespace Payload

namespace Context {
    struct attempt_log {
        int attempts = 0;
        int payload  = 0;
    };

    struct start { int payload; };
    struct fail {};
    struct done {};
    struct restart {};

    struct idle {}; // no context

    struct trying {
        static constexpr auto timeout = 50ms;

        trying(start const& event, attempt_log& log) : context(log)
        {
            context.attempts = 1;
            context.payload  = event.payload;
        }
        explicit trying(attempt_log& log) : context(log) { ++context.attempts; }

        attempt_log& context;
    };

    struct succeeded { // same context type as trying: shared instance
        explicit succeeded(attempt_log& log) : context(log) {}
        attempt_log& context;
    };

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<start>,        fsm::to<trying>>,
        fsm::transition<fsm::from<trying>,  fsm::on<fsm::timeout>, fsm::to<trying>>,
        fsm::transition<fsm::from<trying>,  fsm::on<fail>,         fsm::to<trying>>,
        fsm::transition<fsm::from<trying>,  fsm::on<done>,         fsm::to<succeeded>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<restart>, fsm::to<idle>>>;

    // context states need no default constructor
    static_assert(!std::default_initializable<trying>);
    static_assert(fsm::concepts::state<trying>);
} // namespace Context

namespace Internal {
    struct tick {};
    struct note {
        int value;
    };

    struct log {
        int noted = 0;
    };

    struct waiting {
        static constexpr auto timeout = 50ms;

        explicit waiting(log& l) : context(l) {}
        void handle(note const& event) { context.noted = event.value; }
        void handle(tick const&) {} // consumed without effect while noted

        log& context;
    };
    struct done {};

    struct already_noted {
        static bool check(waiting const& state) { return state.context.noted != 0; }
    };

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<waiting>, fsm::on<fsm::timeout>, fsm::to<done>>,
        fsm::internal_transition<fsm::from<waiting>, fsm::on<note>>,
        // internal and regular transitions group as alternatives
        fsm::transition<fsm::from<done>, fsm::on<tick>, fsm::to<waiting>>,
        fsm::internal_transition<fsm::from<waiting>, fsm::on<tick>,
                                 fsm::guard<already_noted>>,
        fsm::transition<fsm::from<waiting>, fsm::on<tick>, fsm::to<done>>>;

    // internal_target never becomes a state of the table
    static_assert(std::is_same_v<tbl::states, mtl::typelist<waiting, done>>);

    struct hook_counter {
        int enters = 0;
        int exits  = 0;
        template<typename OLD_STATE, typename NEW_STATE, typename SM>
        void onEnterState(SM&)
        {
            ++enters;
        }
        template<typename OLD_STATE, typename NEW_STATE, typename SM>
        void onExitState(SM&)
        {
            ++exits;
        }
    };
} // namespace Internal

namespace Alternatives {
    struct tick {};
    struct budget {
        int used  = 0;
        int limit = 2;
    };

    struct idle {};
    struct pending {
        explicit pending(budget& b) : context(b) { ++context.used; }
        budget& context;
    };
    struct exhausted {};

    struct within_budget {
        static bool check(pending const& state) { return state.context.used < state.context.limit; }
    };

    // Two transitions share (pending, tick): the guarded one first, the
    // unguarded catch-all last
    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>,      fsm::on<tick>, fsm::to<pending>>,
        fsm::transition<fsm::from<pending>,   fsm::on<tick>, fsm::to<pending>,
                        fsm::guard<within_budget>>,
        fsm::transition<fsm::from<pending>,   fsm::on<tick>, fsm::to<exhausted>>,
        fsm::transition<fsm::from<exhausted>, fsm::on<tick>, fsm::to<idle>>>;
} // namespace Alternatives

// --- runtime checks ---------------------------------------------------------

namespace {

int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        ++failures;
        std::print("FAILED: {}:{}\n", location.file_name(), location.line());
    }
}

void initialStateAndNotification()
{
    output_controller ctrl; // owned by the application, injected by reference
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.is<off>());
    check(!tim.timer.armed); // off has no timeout
    // observers get the initial state's value during construction
    check(ctrl.log.size() == 1 && ctrl.log.back() == off::outputs);
}

void transitionOnEvent()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.process(button_press{})); // off -> running
    check(sm.is<running>());
    check(ctrl.log.size() == 2 && ctrl.log.back() == running::outputs);
}

void ignoredEventReportsFalse()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};
    sm.process(button_press{}); // running has no transition for lock_key

    check(!sm.process(lock_key{}));
    check(sm.is<running>());
    check(ctrl.log.size() == 2); // an ignored event must not notify
}

void timerArmedOnEntryStoppedOnExit()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    sm.process(button_press{}); // off -> running: timed state
    check(tim.timer.armed);
    check(tim.timer.duration == 50ms);

    sm.process(button_press{}); // running -> off: leaving must disarm
    check(!tim.timer.armed);
}

void timeoutChain()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};
    sm.process(button_press{}); // off -> running

    tim.timer.expire();        // running -> cooldown (led off, fan still on)
    check(sm.is<cooldown>());
    check(tim.timer.armed);    // cooldown re-arms with its own timeout
    check(tim.timer.duration == 100ms);
    check(ctrl.log.size() == 3 && ctrl.log.back() == cooldown::outputs);

    tim.timer.expire();        // cooldown -> off
    check(sm.is<off>());
    check(!tim.timer.armed);
    check(ctrl.log.size() == 4 && ctrl.log.back() == off::outputs);
}

void equalAnnotationsDoNotNotify()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.process(lock_key{})); // off -> locked: equal outputs, no notification
    check(sm.is<locked>());
    check(ctrl.log.size() == 1);

    check(sm.process(lock_key{})); // locked -> off: equal outputs, no notification
    check(sm.is<off>());
    check(ctrl.log.size() == 1);
    check(ctrl.exit_log.empty()); // suppression also applies to exit values
}

void exitValuesAreNotified()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};
    check(ctrl.exit_log.empty()); // construction only enters

    sm.process(button_press{}); // off -> running: leaving off's outputs
    check(ctrl.exit_log.size() == 1 && ctrl.exit_log.back() == off::outputs);

    sm.process(button_press{}); // running -> off
    check(ctrl.exit_log.size() == 2 && ctrl.exit_log.back() == running::outputs);
}

void getIfAccessesCurrentState()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.getIf<off>() != nullptr);
    check(sm.getIf<running>() == nullptr);

    machine const& read_only = sm;
    check(read_only.getIf<off>() != nullptr);
    check(read_only.getIf<running>() == nullptr);
}

void explicitInitialState()
{
    fsm::state_machine<ExplicitInitial::lock_first> sm;

    check(sm.is<locked>());
    check(sm.process(lock_key{})); // locked -> off
    check(sm.is<off>());
}

void anyStateReachesTargetFromEverywhere()
{
    using namespace Wildcard;
    fsm::state_machine<tbl> sm;

    check(sm.process(shutdown{})); // wildcard also matches the target state itself
    check(sm.is<idle>());

    sm.process(advance{});
    sm.process(advance{});
    check(sm.is<stage2>());
    check(sm.process(shutdown{})); // stage2 -> idle via the wildcard
    check(sm.is<idle>());
}

void eventPayloadConstructsTargetState()
{
    using namespace Payload;
    fsm::state_machine<tbl> sm;

    check(sm.process(send{.msg = {.id = 42}}));
    check(sm.is<sending>());
    check(sm.getIf<sending>()->msg.id == 42);

    check(sm.process(cancel{})); // idle has no constructor from cancel
    check(sm.is<idle>());
}

void liveObservationDeliversInstanceValues()
{
    using namespace Payload;

    live_driver driver;
    fsm::state_machine<tbl, live_driver> sm{driver};

    check(sm.process(send{.msg = {.id = 7}}));
    check(driver.entered.size() == 1 && driver.entered.back() == 7);
    check(driver.exited.empty()); // idle has no msg: exit hook dropped out

    check(sm.process(cancel{}));
    check(driver.exited.size() == 1 && driver.exited.back() == 7);

    // an equal value notifies again: live observation has no suppression
    check(sm.process(send{.msg = {.id = 7}}));
    check(driver.entered.size() == 2 && driver.entered.back() == 7);
}

void payloadReachesObserverThroughState()
{
    using namespace Payload;
    tx_driver driver;
    fsm::state_machine<tbl, tx_driver> sm{driver};

    sm.process(send{.msg = {.id = 7}});
    sm.process(cancel{});
    sm.process(send{.msg = {.id = 9}});

    check(driver.transmitted.size() == 2);
    check(driver.transmitted[0] == 7 && driver.transmitted[1] == 9);
}

void machineWithOnlyATimerObserver()
{
    fsm::timed<manual_timer> tim;
    fsm::state_machine<table, fsm::timed<manual_timer>> sm{tim};

    check(sm.process(button_press{}));
    check(sm.is<running>());
    check(tim.timer.armed); // running is a timed state
}

// --- optional onEntry()/onExit() hooks ------------------------------------
namespace hooks {
    int entries = 0;
    int exits   = 0;

    struct ping {};
    struct plain { void onExit() { ++exits; } };    // no onEntry
    struct hooked { void onEntry() { ++entries; } }; // no onExit

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<plain>,  fsm::on<ping>, fsm::to<hooked>>,
        fsm::transition<fsm::from<hooked>, fsm::on<ping>, fsm::to<plain>>>;
} // namespace hooks

void entryAndExitHooks()
{
    using namespace hooks;
    fsm::state_machine<tbl> sm; // no timed states, no observers: nothing to inject
    check(entries == 0 && exits == 0); // initial entry runs no exit, plain has no onEntry

    sm.process(ping{}); // plain -> hooked: plain::onExit, hooked::onEntry
    check(exits == 1);
    check(entries == 1);

    sm.process(ping{}); // hooked -> plain: neither state has the other hook
    check(exits == 1);
    check(entries == 1);
}

// --- guarded transitions ----------------------------------------------------
namespace guards {
    struct push {};
    struct gate {
        bool open = false;
    };
    struct passed {};

    struct gate_is_open { // state-argument form: condition on state data
        static bool check(gate const& s) { return s.open; }
    };
    struct return_allowed { // no-argument form, toggled by the test
        static inline bool allow = false;
        static bool check() { return allow; }
    };

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<gate>,   fsm::on<push>, fsm::to<passed>,
                        fsm::guard<gate_is_open>>,
        fsm::transition<fsm::from<passed>, fsm::on<push>, fsm::to<gate>,
                        fsm::guard<return_allowed>>>;
} // namespace guards

void guardBlocksAndAllows()
{
    using namespace guards;
    fsm::state_machine<tbl> sm;

    check(!sm.process(push{})); // gate closed: guard blocks, nothing happens
    check(sm.is<gate>());

    sm.getIf<gate>()->open = true;
    check(sm.process(push{}));  // guard passes now
    check(sm.is<passed>());

    return_allowed::allow = false;
    check(!sm.process(push{})); // no-argument guard form blocks
    check(sm.is<passed>());

    return_allowed::allow = true;
    check(sm.process(push{}));
    check(sm.is<gate>());
    check(!sm.getIf<gate>()->open); // re-entry default-constructs the state
}

// --- raw lifecycle hooks (observer without the fsm::observing base) ---------
namespace raw_hooks {
    struct transition_counter {
        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onExitState(MACHINE&) { ++exits; }

        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void onEnterState(MACHINE&) { ++enters; }

        int exits  = 0;
        int enters = 0;
    };
} // namespace raw_hooks

// --- guards deciding on the event payload ------------------------------------
namespace event_guard {
    struct reading {
        int value = 0;
    };
    struct closed {};
    struct open {};

    // fires only for readings above the threshold the state holds -
    // the event form sees the payload before any handler applies it
    struct above_threshold {
        static bool check(closed const&, reading const& event) { return event.value > 10; }
    };

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<closed>, fsm::on<reading>, fsm::to<open>,
                        fsm::guard<above_threshold>>>;
} // namespace event_guard

void guardSeesTheEventPayload()
{
    using namespace event_guard;
    fsm::state_machine<tbl> sm;

    check(!sm.process(reading{.value = 5})); // below: guard blocks
    check(sm.is<closed>());
    check(sm.process(reading{.value = 11}));
    check(sm.is<open>());
}

// --- static-before-nonstatic ordering within one observer -------------------
namespace ordering {
    struct go {
        int value = 0;
    };
    struct mode_t {
        int mode;
        constexpr bool operator==(mode_t const&) const = default;
    };

    struct idle {
        static constexpr mode_t mode{0};
    };
    struct active {
        static constexpr mode_t mode{1};
        active() = default;
        explicit active(go const& event) : value(event.value) {}
        int value = 0;
    };

    // observes the static mode and the nonstatic value; the contract
    // guarantees the static hook runs first on the same entry
    struct dual_observer : fsm::observing<dual_observer> {
        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::mode)
        {
            return STATE::mode;
        }
        static constexpr auto observe_nonstatic(auto const& state) -> decltype((state.value))
        {
            return state.value;
        }
        void notifyEntry(mode_t const&) { sequence.push_back('s'); }
        void notifyEntry(int value) { sequence.push_back('n'); last_value = value; }

        std::vector<char> sequence;
        int last_value = -1;
    };

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<active>>>;
} // namespace ordering

void staticHookRunsBeforeNonstaticHook()
{
    ordering::dual_observer observer;
    fsm::state_machine<ordering::tbl, ordering::dual_observer> sm{observer};

    check(observer.sequence == std::vector{'s'}); // initial entry: static only

    check(sm.process(ordering::go{.value = 7}));
    check(observer.sequence == std::vector{'s', 's', 'n'});
    check(observer.last_value == 7);
}

void observerGroupForwardsHooksInMemberOrder()
{
    fsm::timed<manual_timer> tim;
    output_controller ctrl;
    raw_hooks::transition_counter counter;
    fsm::observer_group<fsm::timed<manual_timer>, output_controller,
                        raw_hooks::transition_counter>
        group{tim, ctrl, counter};
    fsm::state_machine<table, decltype(group)> sm{group}; // one reference, three observers

    check(counter.enters == 1 && counter.exits == 0);
    check(ctrl.log.size() == 1); // initial off outputs

    sm.process(button_press{}); // off -> running
    check(tim.timer.armed && tim.timer.duration == 50ms); // timed's validate/hooks forwarded
    check(counter.enters == 2 && counter.exits == 1);
    check(ctrl.log.back() == outputs_t{.led = true, .fan = true});

    check(!sm.process(lock_key{})); // ignored event: nothing forwarded
    check(counter.enters == 2 && counter.exits == 1);
}

void rawHookObserverSeesEveryTransition()
{
    raw_hooks::transition_counter counter;
    fsm::timed<manual_timer> tim;
    fsm::state_machine<table, fsm::timed<manual_timer>, raw_hooks::transition_counter> sm{tim, counter};

    check(counter.enters == 1); // initial entry, no exit
    check(counter.exits == 0);

    sm.process(button_press{}); // off -> running
    check(counter.enters == 2 && counter.exits == 1);

    check(!sm.process(lock_key{})); // ignored event: no hooks
    check(counter.enters == 2 && counter.exits == 1);
}

} // namespace

void contextIsMachineOwnedAndShared()
{
    using namespace Context;
    fsm::state_machine<tbl> sm; // timeout in trying stays unobserved: no timer injected

    check(sm.process(start{.payload = 7}));
    auto const* log = &sm.getIf<trying>()->context;
    check(log->attempts == 1 && log->payload == 7);

    check(sm.process(fail{})); // re-entry: fresh state object, same context
    check(&sm.getIf<trying>()->context == log);
    check(log->attempts == 2 && log->payload == 7);

    check(sm.process(done{})); // succeeded names the same context type
    check(&sm.getIf<succeeded>()->context == log);
    check(log->attempts == 2);

    check(sm.process(restart{})); // context also outlives contextless states
    check(sm.process(start{.payload = 9}));
    check(log->attempts == 1 && log->payload == 9);
}

void contextSurvivesTimeoutRetry()
{
    using namespace Context;
    fsm::timed<manual_timer> tim;
    fsm::state_machine<tbl, fsm::timed<manual_timer>> sm{tim};

    check(sm.process(start{.payload = 3}));
    check(tim.timer.armed);

    tim.timer.expire(); // the retry loses neither payload nor attempt count
    check(sm.is<trying>());
    check(sm.getIf<trying>()->context.attempts == 2);
    check(sm.getIf<trying>()->context.payload == 3);
    check(tim.timer.armed); // re-armed for the next attempt
}

void contextInitialState()
{
    using namespace Context;
    using tbl2 = fsm::transition_table<
        fsm::initial<trying>,
        fsm::transition<fsm::from<trying>, fsm::on<done>, fsm::to<succeeded>>>;
    fsm::state_machine<tbl2> sm; // initial state constructed from its context

    check(sm.is<trying>());
    check(sm.getIf<trying>()->context.attempts == 1);
}

void internalTransitionHandlesInPlace()
{
    using namespace Internal;

    fsm::timed<manual_timer> tim;
    hook_counter hooks;
    fsm::state_machine<tbl, fsm::timed<manual_timer>, hook_counter> sm{tim, hooks};

    check(sm.is<waiting>() && tim.timer.armed);
    auto const enters_before   = hooks.enters;
    auto const duration_before = tim.timer.duration;

    check(sm.process(note{.value = 7})); // handled in place
    check(sm.is<waiting>());
    check(sm.getIf<waiting>()->context.noted == 7);
    check(hooks.enters == enters_before && hooks.exits == 0); // no exit/entry ran
    check(tim.timer.armed && tim.timer.duration == duration_before); // timer untouched

    // the guarded internal alternative wins over the regular fallback
    check(sm.process(tick{}));
    check(sm.is<waiting>());

    // with the guard failing, the fallback transition fires
    sm.getIf<waiting>()->context.noted = 0;
    check(sm.process(tick{}));
    check(sm.is<done>());
    check(!tim.timer.armed);
}

void guardedAlternativesFirstPassWins()
{
    using namespace Alternatives;
    fsm::state_machine<tbl> sm;

    check(sm.process(tick{})); // idle -> pending, used = 1
    check(sm.is<pending>());
    check(sm.process(tick{})); // guard passes (1 < 2): retry, used = 2
    check(sm.is<pending>() && sm.getIf<pending>()->context.used == 2);
    check(sm.process(tick{})); // guard fails (2 < 2): catch-all fires
    check(sm.is<exhausted>());

    check(sm.process(tick{})); // exhausted -> idle
    check(sm.process(tick{})); // budget is shared context: used keeps counting
    check(sm.getIf<pending>()->context.used == 3);
}

void timerInjectedByReference()
{
    manual_timer timer; // caller-owned policy instance
    fsm::timed<manual_timer&> tim{timer};
    output_controller ctrl;
    fsm::state_machine<table, fsm::timed<manual_timer&>, output_controller> sm{tim, ctrl};

    check(sm.process(button_press{})); // off -> running, timeout armed
    check(timer.armed && timer.duration == 50ms);
    timer.expire();
    check(sm.is<cooldown>());
}

int statemachineTests()
{
    initialStateAndNotification();
    transitionOnEvent();
    ignoredEventReportsFalse();
    timerArmedOnEntryStoppedOnExit();
    timeoutChain();
    equalAnnotationsDoNotNotify();
    exitValuesAreNotified();
    getIfAccessesCurrentState();
    explicitInitialState();
    anyStateReachesTargetFromEverywhere();
    eventPayloadConstructsTargetState();
    payloadReachesObserverThroughState();
    liveObservationDeliversInstanceValues();
    machineWithOnlyATimerObserver();
    entryAndExitHooks();
    guardBlocksAndAllows();
    rawHookObserverSeesEveryTransition();
    guardSeesTheEventPayload();
    staticHookRunsBeforeNonstaticHook();
    observerGroupForwardsHooksInMemberOrder();
    contextIsMachineOwnedAndShared();
    contextSurvivesTimeoutRetry();
    contextInitialState();
    guardedAlternativesFirstPassWins();
    internalTransitionHandlesInPlace();
    timerInjectedByReference();
    return failures;
}
