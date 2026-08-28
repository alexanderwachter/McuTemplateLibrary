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

// Records notifications for the checks below; uses the hand-written
// annotation() form (instead of observe()) to keep it covered
struct output_controller : fsm::observing<output_controller> {
    template<typename STATE>
    static constexpr auto annotation() requires requires { STATE::outputs; }
    {
        return STATE::outputs;
    }

    void notify_entry(outputs_t const& out) { log.push_back(out); }
    void notify_exit(outputs_t const& out) { exit_log.push_back(out); }

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
    static_assert(fsm::concepts::guard<with_state> && fsm::concepts::guard<always>);
    static_assert(!fsm::concepts::guard<off>); // no check member
    static_assert(fsm::concepts::guard_for<with_state, off>);
    static_assert(!fsm::concepts::guard_for<with_state, running>); // wrong state
    static_assert(fsm::concepts::guard_for<always, off>); // no-argument form
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
        void on_enter_state(MACHINE& machine)
        {
            if constexpr (std::is_same_v<NEW_STATE, sending>) {
                transmitted.push_back(machine.template get_if<sending>()->msg.id);
            }
        }

        std::vector<int> transmitted;
    };
} // namespace Payload

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

void initial_state_and_notification()
{
    output_controller ctrl; // owned by the application, injected by reference
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.is<off>());
    check(!tim.timer.armed); // off has no timeout
    // observers get the initial state's value during construction
    check(ctrl.log.size() == 1 && ctrl.log.back() == off::outputs);
}

void transition_on_event()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.process(button_press{})); // off -> running
    check(sm.is<running>());
    check(ctrl.log.size() == 2 && ctrl.log.back() == running::outputs);
}

void ignored_event_reports_false()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};
    sm.process(button_press{}); // running has no transition for lock_key

    check(!sm.process(lock_key{}));
    check(sm.is<running>());
    check(ctrl.log.size() == 2); // an ignored event must not notify
}

void timer_armed_on_entry_stopped_on_exit()
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

void timeout_chain()
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

void equal_annotations_do_not_notify()
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

void exit_values_are_notified()
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

void get_if_accesses_current_state()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};

    check(sm.get_if<off>() != nullptr);
    check(sm.get_if<running>() == nullptr);

    machine const& read_only = sm;
    check(read_only.get_if<off>() != nullptr);
    check(read_only.get_if<running>() == nullptr);
}

void explicit_initial_state()
{
    fsm::state_machine<ExplicitInitial::lock_first> sm;

    check(sm.is<locked>());
    check(sm.process(lock_key{})); // locked -> off
    check(sm.is<off>());
}

void any_state_reaches_target_from_everywhere()
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

void event_payload_constructs_target_state()
{
    using namespace Payload;
    fsm::state_machine<tbl> sm;

    check(sm.process(send{.msg = {.id = 42}}));
    check(sm.is<sending>());
    check(sm.get_if<sending>()->msg.id == 42);

    check(sm.process(cancel{})); // idle has no constructor from cancel
    check(sm.is<idle>());
}

void payload_reaches_observer_through_state()
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

void machine_with_only_a_timer_observer()
{
    fsm::timed<manual_timer> tim;
    fsm::state_machine<table, fsm::timed<manual_timer>> sm{tim};

    check(sm.process(button_press{}));
    check(sm.is<running>());
    check(tim.timer.armed); // running is a timed state
}

// --- optional on_entry()/on_exit() hooks ------------------------------------
namespace hooks {
    int entries = 0;
    int exits   = 0;

    struct ping {};
    struct plain { void on_exit() { ++exits; } };    // no on_entry
    struct hooked { void on_entry() { ++entries; } }; // no on_exit

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<plain>,  fsm::on<ping>, fsm::to<hooked>>,
        fsm::transition<fsm::from<hooked>, fsm::on<ping>, fsm::to<plain>>>;
} // namespace hooks

void entry_and_exit_hooks()
{
    using namespace hooks;
    fsm::state_machine<tbl> sm; // no timed states, no observers: nothing to inject
    check(entries == 0 && exits == 0); // initial entry runs no exit, plain has no on_entry

    sm.process(ping{}); // plain -> hooked: plain::on_exit, hooked::on_entry
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

void guard_blocks_and_allows()
{
    using namespace guards;
    fsm::state_machine<tbl> sm;

    check(!sm.process(push{})); // gate closed: guard blocks, nothing happens
    check(sm.is<gate>());

    sm.get_if<gate>()->open = true;
    check(sm.process(push{}));  // guard passes now
    check(sm.is<passed>());

    return_allowed::allow = false;
    check(!sm.process(push{})); // no-argument guard form blocks
    check(sm.is<passed>());

    return_allowed::allow = true;
    check(sm.process(push{}));
    check(sm.is<gate>());
    check(!sm.get_if<gate>()->open); // re-entry default-constructs the state
}

// --- raw lifecycle hooks (observer without the fsm::observing base) ---------
namespace raw_hooks {
    struct transition_counter {
        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void on_exit_state(MACHINE&) { ++exits; }

        template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        void on_enter_state(MACHINE&) { ++enters; }

        int exits  = 0;
        int enters = 0;
    };
} // namespace raw_hooks

void raw_hook_observer_sees_every_transition()
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

int statemachine_tests()
{
    initial_state_and_notification();
    transition_on_event();
    ignored_event_reports_false();
    timer_armed_on_entry_stopped_on_exit();
    timeout_chain();
    equal_annotations_do_not_notify();
    exit_values_are_notified();
    get_if_accesses_current_state();
    explicit_initial_state();
    any_state_reaches_target_from_everywhere();
    event_payload_constructs_target_state();
    payload_reaches_observer_through_state();
    machine_with_only_a_timer_observer();
    entry_and_exit_hooks();
    guard_blocks_and_allows();
    raw_hook_observer_sees_every_transition();
    return failures;
}
