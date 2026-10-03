/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachine.hpp>
#include <mtl/TypeName.hpp>
#include <mtl/Typelist.hpp>

#include <chrono>
#include <print>
#include <source_location>
#include <string>
#include <string_view>
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
    int starts                   = 0; // re-arm detection

    void start(std::chrono::milliseconds d, fsm::timer_callback cb, void* ctx)
    {
        duration = d;
        callback = cb;
        context  = ctx;
        armed    = true;
        ++starts;
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

struct table : fsm::transition_table<
    fsm::transition<fsm::from<off>,      fsm::on<button_press>, fsm::to<running>>,
    fsm::transition<fsm::from<running>,  fsm::on<button_press>, fsm::to<off>>,
    fsm::transition<fsm::from<running>,  fsm::on<fsm::timeout>, fsm::to<cooldown>>,
    fsm::transition<fsm::from<cooldown>, fsm::on<fsm::timeout>, fsm::to<off>>,
    fsm::transition<fsm::from<off>,      fsm::on<lock_key>,     fsm::to<locked>>,
    fsm::transition<fsm::from<locked>,   fsm::on<lock_key>,     fsm::to<off>>> {};

using machine = fsm::StateMachine<table, fsm::timed<manual_timer>, output_controller>;

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

    static_assert(std::is_same_v<fsm::transition_for_t<table, off, button_press>::to, running>);
    static_assert(std::is_same_v<fsm::transition_for_t<table, cooldown, fsm::timeout>::to, off>);
    // pairs not in the table resolve to nil_type
    static_assert(std::is_same_v<fsm::transition_for_t<table, running, lock_key>, mtl::nil_type>);
    static_assert(std::is_same_v<fsm::transition_for_t<table, locked, button_press>, mtl::nil_type>);
} // namespace Table

namespace Concepts {
    static_assert(fsm::concepts::state<off> && fsm::concepts::state<running>);
    static_assert(!fsm::concepts::state<int>);
    static_assert(fsm::concepts::transition_table<table>);
    static_assert(!fsm::concepts::transition_table<off>);

    // what the role-named lists take
    static_assert(fsm::concepts::event<button_press> && fsm::concepts::event<int>);
    static_assert(!fsm::concepts::event<button_press const> && !fsm::concepts::event<button_press&>);
    static_assert(fsm::concepts::annotation<outputs_t> && !fsm::concepts::annotation<outputs_t const&>);
    struct needs_an_argument {
        explicit needs_an_argument(int);
    };
    static_assert(fsm::concepts::context<outputs_t> && !fsm::concepts::context<needs_an_argument>);
    static_assert(fsm::concepts::observer<output_controller> && !fsm::concepts::observer<int>);
    inline constexpr auto some_time = std::chrono::milliseconds{1};
    static_assert(fsm::concepts::timed_by_or_timer_ranges<fsm::timed_by<running, some_time>>);
    static_assert(fsm::concepts::timed_by_or_timer_ranges<
                  fsm::timer_ranges<fsm::timed_by<running, some_time>>>);
    static_assert(!fsm::concepts::timed_by_or_timer_ranges<running>);
    static_assert(std::is_same_v<fsm::events<button_press, lock_key>,
                                 mtl::typelist<button_press, lock_key>>);
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
    struct lock_first : fsm::transition_table<
        fsm::initial<locked>,
        fsm::transition<fsm::from<off>,    fsm::on<lock_key>, fsm::to<locked>>,
        fsm::transition<fsm::from<locked>, fsm::on<lock_key>, fsm::to<off>>> {};

    // the chosen state moves to the front and becomes the initial state
    static_assert(std::is_same_v<lock_first::states, mtl::typelist<locked, off>>);
    static_assert(std::is_same_v<fsm::StateMachine<lock_first>::initial_state, locked>);

    // initial<> may appear anywhere in the table
    struct reordered : fsm::transition_table<
        fsm::transition<fsm::from<off>,    fsm::on<lock_key>, fsm::to<locked>>,
        fsm::initial<locked>,
        fsm::transition<fsm::from<locked>, fsm::on<lock_key>, fsm::to<off>>> {};
    static_assert(std::is_same_v<reordered::states, lock_first::states>);
    static_assert(std::is_same_v<reordered::transitions, lock_first::transitions>);
} // namespace ExplicitInitial

namespace Composition {
    using locking = fsm::transition_table<
        fsm::transition<fsm::from<off>,    fsm::on<lock_key>, fsm::to<locked>>,
        fsm::transition<fsm::from<locked>, fsm::on<lock_key>, fsm::to<off>>>;
    using running_cycle = fsm::transition_table<
        fsm::transition<fsm::from<off>,     fsm::on<button_press>, fsm::to<running>>,
        fsm::transition<fsm::from<running>, fsm::on<button_press>, fsm::to<off>>>;

    struct spelled_out : fsm::transition_table<
        fsm::initial<locked>,
        fsm::transition<fsm::from<off>,     fsm::on<button_press>, fsm::to<running>>,
        fsm::transition<fsm::from<running>, fsm::on<button_press>, fsm::to<off>>,
        fsm::transition<fsm::from<off>,     fsm::on<fsm::timeout>, fsm::to<off>>,
        fsm::transition<fsm::from<off>,     fsm::on<lock_key>,     fsm::to<locked>>,
        fsm::transition<fsm::from<locked>,  fsm::on<lock_key>,     fsm::to<off>>> {};

    // an unnamed table among the entries stands for its entries, in place
    struct composed : fsm::transition_table<
        fsm::initial<locked>,
        running_cycle,
        fsm::transition<fsm::from<off>, fsm::on<fsm::timeout>, fsm::to<off>>,
        locking> {};
    static_assert(std::is_same_v<composed::entries, spelled_out::entries>);
    static_assert(std::is_same_v<composed::states, mtl::typelist<locked, off, running>>);
    static_assert(std::is_same_v<fsm::transition_for_t<composed, locked, lock_key>::to, off>);
    static_assert(std::is_same_v<fsm::StateMachine<composed>::initial_state, locked>);

    // at any depth
    using cycle_then_idling = fsm::transition_table<
        running_cycle,
        fsm::transition<fsm::from<off>, fsm::on<fsm::timeout>, fsm::to<off>>>;
    struct composed_twice
        : fsm::transition_table<fsm::initial<locked>, cycle_then_idling, locking> {};
    static_assert(std::is_same_v<composed_twice::entries, spelled_out::entries>);

    // a table standing in another one is not instantiated by itself: this
    // one names an initial state it has no transition for
    using only_the_initial_state = fsm::transition_table<fsm::initial<locked>>;
    struct completed : fsm::transition_table<only_the_initial_state, locking> {};
    static_assert(std::is_same_v<completed::states, mtl::typelist<locked, off>>);

    static_assert(fsm::concepts::entry_or_table<locking>);
    static_assert(fsm::concepts::entry_or_table<fsm::initial<locked>>);
    static_assert(!fsm::concepts::entry_or_table<off>);
    // a named table is the one a machine runs, not a building block
    static_assert(!fsm::concepts::entry_or_table<composed>);
} // namespace Composition

namespace Guards {
    struct always { static bool check() { return true; } };

    using unguarded = fsm::transition<fsm::from<off>, fsm::on<button_press>, fsm::to<running>>;
    using guarded   = fsm::transition<fsm::from<off>, fsm::on<button_press>, fsm::to<running>,
                                      fsm::guard<always>>;
    using reordered = fsm::transition<fsm::guard<always>, fsm::to<running>,
                                      fsm::from<off>, fsm::on<button_press>>;

    static_assert(std::is_same_v<unguarded::guards, mtl::typelist<>>);
    static_assert(std::is_same_v<guarded::guards, mtl::typelist<always>>);
    static_assert(std::is_same_v<reordered::guards, mtl::typelist<always>>);

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

namespace TimeoutBounds {
    static_assert(fsm::is_timeout_range<fsm::timeout_range>::value);
    static_assert(fsm::concepts::timeout_range<fsm::timeout_range>);
    static_assert(!fsm::concepts::timeout_range<std::chrono::milliseconds>);

    struct waiting { static constexpr auto timeout = std::chrono::milliseconds{150}; };

    struct timed_table : fsm::transition_table<
        fsm::transition<fsm::from<off>,     fsm::on<button_press>, fsm::to<waiting>>,
        fsm::transition<fsm::from<waiting>, fsm::on<fsm::timeout>, fsm::to<off>>> {};

    // an entry's range must contain the timeout, an exact duration must equal it
    inline constexpr fsm::timeout_range wait_range{std::chrono::milliseconds{100},
                                                   std::chrono::milliseconds{200}};
    inline constexpr auto exact_wait = std::chrono::milliseconds{150};

    using ranged_map = fsm::timer_ranges<fsm::timed_by<waiting, wait_range>>;
    using exact_map  = fsm::timer_ranges<fsm::timed_by<waiting, exact_wait>>;

    static_assert(fsm::timeout_within_bounds_v<ranged_map, waiting>);
    static_assert(fsm::timeouts_within_bounds_v<timed_table, ranged_map>);
    static_assert(fsm::timeouts_within_bounds_v<timed_table, exact_map>);

    // a map among the entries stands for its entries, like a table in a table
    static_assert(fsm::timeouts_within_bounds_v<
                  timed_table, fsm::timer_ranges<fsm::timer_ranges<>, ranged_map>>);
    static_assert(std::is_same_v<fsm::timer_ranges<fsm::timer_ranges<>, ranged_map>, ranged_map>);

    // rejected: a timeout outside the range, a duration that differs, a
    // timed state without an entry, and an entry for an untimed state
    inline constexpr fsm::timeout_range low_range{std::chrono::milliseconds{10},
                                                  std::chrono::milliseconds{20}};
    inline constexpr auto other_wait = std::chrono::milliseconds{100};
    static_assert(!fsm::timeout_within_bounds_v<fsm::timer_ranges<fsm::timed_by<waiting, low_range>>,
                                                waiting>);
    static_assert(!fsm::timeout_within_bounds_v<fsm::timer_ranges<fsm::timed_by<waiting, other_wait>>,
                                                waiting>);
    static_assert(!fsm::timeouts_within_bounds_v<timed_table, fsm::timer_ranges<>>);
    static_assert(!fsm::timeouts_within_bounds_v<
                  timed_table, fsm::timer_ranges<ranged_map, fsm::timed_by<off, wait_range>>>);
} // namespace TimeoutBounds

namespace Reachability {
    struct start {};
    struct step {};
    struct orphan {};
    struct end {};
    struct advance {};
    struct abort {};

    struct linear : fsm::transition_table<
        fsm::transition<fsm::from<start>, fsm::on<advance>, fsm::to<step>>,
        fsm::transition<fsm::from<step>,  fsm::on<advance>, fsm::to<end>>> {};
    static_assert(fsm::is_reachable_v<linear, end>);
    static_assert(fsm::all_states_reachable_v<linear>);

    // a state appearing only as a transition source is dead code
    struct orphaned : fsm::transition_table<
        fsm::transition<fsm::from<start>,  fsm::on<advance>, fsm::to<end>>,
        fsm::transition<fsm::from<orphan>, fsm::on<advance>, fsm::to<end>>> {};
    static_assert(!fsm::is_reachable_v<orphaned, orphan>);
    static_assert(!fsm::all_states_reachable_v<orphaned>);

    // a wildcard source leaves from every state; internal transitions
    // stay in place and reach nothing
    struct through_wildcard : fsm::transition_table<
        fsm::transition<fsm::from<start>, fsm::on<advance>, fsm::to<step>>,
        fsm::internal_transition<fsm::from<step>, fsm::on<advance>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<abort>, fsm::to<end>>> {};
    static_assert(fsm::is_reachable_v<through_wildcard, end>);
    static_assert(fsm::all_states_reachable_v<through_wildcard>);
} // namespace Reachability

namespace AnnotationCoverage {
    struct tick {};
    struct annotated { static constexpr int level = 3; };
    struct wrongly_annotated { static constexpr char const* level = "high"; };
    struct bare {};

    struct level_watcher : fsm::observing<level_watcher> {
        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::level)
        {
            return STATE::level;
        }
        void notifyEntry(int);
    };

    // the base is a mixin: constructible only as part of its derived class
    static_assert(!std::is_default_constructible_v<fsm::observing<level_watcher>>);
    static_assert(std::is_default_constructible_v<level_watcher>);

    static_assert(fsm::is_observed_v<level_watcher, annotated>);
    static_assert(!fsm::is_observed_v<level_watcher, bare>);
    static_assert(fsm::concepts::notified_of<level_watcher, annotated>);
    static_assert(fsm::is_notified_of_v<level_watcher, annotated>);
    static_assert(!fsm::is_notified_of_v<level_watcher, bare>);

    // observed, but no hook accepts the annotation's type: the dispatch
    // would silently skip the state
    static_assert(fsm::is_observed_v<level_watcher, wrongly_annotated>);
    static_assert(!fsm::is_notified_of_v<level_watcher, wrongly_annotated>);

    struct mixed_table : fsm::transition_table<
        fsm::transition<fsm::from<annotated>, fsm::on<tick>, fsm::to<bare>>,
        fsm::transition<fsm::from<bare>,      fsm::on<tick>, fsm::to<annotated>>> {};

    static_assert(!fsm::all_states_notified_v<level_watcher, mixed_table>);
    static_assert(fsm::all_states_notified_v<level_watcher, mixed_table, mtl::typelist<bare>>);
} // namespace AnnotationCoverage

namespace EventHandling {
    struct go {};
    struct halt {};
    struct ignored {};

    struct table : fsm::transition_table<
        fsm::transition<fsm::from<off>, fsm::on<go>, fsm::to<running>>,
        fsm::internal_transition<fsm::from<running>, fsm::on<go>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<halt>, fsm::to<off>>> {};

    static_assert(fsm::handles_event_v<table, off, go>);
    static_assert(fsm::handles_event_v<table, running, go>); // internal counts
    static_assert(fsm::handles_event_v<table, running, halt>); // via the wildcard
    static_assert(!fsm::handles_event_v<table, off, ignored>);
} // namespace EventHandling

namespace Wildcard {
    struct advance {};
    struct shutdown {};
    struct idle {};
    struct stage1 {};
    struct stage2 {};

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<idle>,           fsm::on<advance>,  fsm::to<stage1>>,
        fsm::transition<fsm::from<stage1>,         fsm::on<advance>,  fsm::to<stage2>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<shutdown>, fsm::to<idle>>> {};

    // any_state is not a state of the machine
    static_assert(std::is_same_v<tbl::states, mtl::typelist<idle, stage1, stage2>>);
    // the wildcard matches states without an exact (state, event) pair
    static_assert(std::is_same_v<fsm::transition_for_t<tbl, stage2, shutdown>::to, idle>);
    static_assert(std::is_same_v<fsm::transition_for_t<tbl, stage1, advance>::to, stage2>);

    // an exact pair takes precedence over the wildcard
    struct with_override : fsm::transition_table<
        fsm::transition<fsm::from<idle>,           fsm::on<advance>,  fsm::to<stage1>>,
        fsm::transition<fsm::from<stage1>,         fsm::on<shutdown>, fsm::to<stage2>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<shutdown>, fsm::to<idle>>> {};
    static_assert(std::is_same_v<fsm::transition_for_t<with_override, stage1, shutdown>::to, stage2>);
    static_assert(std::is_same_v<fsm::transition_for_t<with_override, stage2, shutdown>::to, idle>);
    // the exact pair first, the wildcard behind it
    static_assert(mtl::count_v<fsm::transitions_for_t<with_override, stage1, shutdown>> == 2);
    static_assert(mtl::count_v<fsm::transitions_for_t<with_override, stage2, shutdown>> == 1);

    // An edge hook is delivered per possible source of a wildcard: stage1
    // overrides the shutdown wildcard, so the edge stage1 -> idle can
    // never fire and must not even be instantiated
    struct edge_recorder {
        template<typename FROM, typename TO, typename MACHINE>
        void onEnterFrom(MACHINE&)
        {
            static_assert(!(std::is_same_v<FROM, stage1> && std::is_same_v<TO, idle>),
                          "an impossible wildcard edge was instantiated");
            ++entries;
        }
        int entries = 0;
    };
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
        // the instance's values as a set: the payload, by reference
        auto values() const { return fsm::annotate_ref(msg); }
    };
    static_assert(std::is_same_v<decltype(std::declval<sending const&>().values()),
                                 fsm::annotation_set<message const&>>);
    static_assert(std::is_same_v<decltype(std::declval<sending const&>().values())::types,
                                 mtl::typelist<message>>);

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<send>,   fsm::to<sending>>,
        fsm::transition<fsm::from<sending>, fsm::on<cancel>, fsm::to<idle>>> {};

    // Driver observer: reads the payload delivered into the sending state
    struct tx_driver {
        template<typename NEW_STATE, typename MACHINE>
        void onEnter(MACHINE& machine)
        {
            if constexpr (std::is_same_v<NEW_STATE, sending>) {
                transmitted.push_back(machine.template getIf<sending>()->msg.id);
            }
        }

        std::vector<int> transmitted;
    };

    // observing-based counterpart to tx_driver: consumes the state's
    // instance value by type, no getIf plumbing, no accessor named
    struct live_driver : fsm::observing<live_driver> {
        using observes = fsm::annotations<message>;

        void notifyEntry(message const& msg) { entered.push_back(msg.id); }
        void notifyExit(message const& msg) { exited.push_back(msg.id); }

        std::vector<int> entered;
        std::vector<int> exited;
    };
    // instance values count for coverage and for the declared-type check
    static_assert(fsm::is_notified_of_v<live_driver, sending>);
    static_assert(!fsm::is_notified_of_v<live_driver, idle>);
    static_assert(fsm::annotation_in_table_v<tbl, message>);
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

    struct history { // a second context type, another lifetime
        int successes = 0;
    };

    struct trying {
        using contexts = fsm::contexts<attempt_log>;
        static constexpr auto timeout = 50ms;

        trying(start const& event, attempt_log& log) : context(log)
        {
            context.attempts = 1;
            context.payload  = event.payload;
        }
        explicit trying(attempt_log& log) : context(log) { ++context.attempts; }

        attempt_log& context;
    };

    struct succeeded { // shares trying's instance, and declares a second
        using contexts = fsm::contexts<attempt_log, history>;
        succeeded(attempt_log& log, history& past) : context(log), past(past) { ++past.successes; }
        attempt_log& context;
        history& past;
    };

    static_assert(std::is_same_v<fsm::internal::contexts_of_t<idle>, mtl::typelist<>>);
    static_assert(std::is_same_v<fsm::internal::contexts_of_t<succeeded>,
                                 mtl::typelist<attempt_log, history>>);
    static_assert(fsm::internal::payload_constructible_v<trying, start>);
    static_assert(!fsm::internal::payload_constructible_v<trying, fail>);
    static_assert(fsm::internal::context_constructible<succeeded>::value);

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<start>,        fsm::to<trying>>,
        fsm::transition<fsm::from<trying>,  fsm::on<fsm::timeout>, fsm::to<trying>>,
        fsm::transition<fsm::from<trying>,  fsm::on<fail>,         fsm::to<trying>>,
        fsm::transition<fsm::from<trying>,  fsm::on<done>,         fsm::to<succeeded>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<restart>, fsm::to<idle>>> {};

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
        using contexts = fsm::contexts<log>;
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

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<waiting>, fsm::on<fsm::timeout>, fsm::to<done>>,
        fsm::internal_transition<fsm::from<waiting>, fsm::on<note>>,
        // internal and regular transitions group as alternatives
        fsm::transition<fsm::from<done>, fsm::on<tick>, fsm::to<waiting>>,
        fsm::internal_transition<fsm::from<waiting>, fsm::on<tick>,
                                 fsm::guard<already_noted>>,
        fsm::transition<fsm::from<waiting>, fsm::on<tick>, fsm::to<done>>> {};

    // internal_target never becomes a state of the table
    static_assert(std::is_same_v<tbl::states, mtl::typelist<waiting, done>>);

    struct hook_counter {
        int enters = 0;
        int exits  = 0;
        template<typename STATE, typename SM>
        void onEnter(SM&)
        {
            ++enters;
        }
        template<typename STATE, typename SM>
        void onExit(SM&)
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
        using contexts = fsm::contexts<budget>;
        explicit pending(budget& b) : context(b) { ++context.used; }
        budget& context;
    };
    struct exhausted {};

    struct within_budget {
        static bool check(pending const& state) { return state.context.used < state.context.limit; }
    };

    // Two transitions share (pending, tick): the guarded one first, the
    // unguarded catch-all last
    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<idle>,      fsm::on<tick>, fsm::to<pending>>,
        fsm::transition<fsm::from<pending>,   fsm::on<tick>, fsm::to<pending>,
                        fsm::guard<within_budget>>,
        fsm::transition<fsm::from<pending>,   fsm::on<tick>, fsm::to<exhausted>>,
        fsm::transition<fsm::from<exhausted>, fsm::on<tick>, fsm::to<idle>>> {};
} // namespace Alternatives

namespace AnnotationSets {
    enum class light { off, on };
    struct level {
        int value;
        constexpr bool operator==(level const&) const = default;
    };
    struct heat {
        bool on;
        constexpr bool operator==(heat const&) const = default;
    };

    struct next {};
    struct kill {};

    struct dark {
        static constexpr auto annotations = fsm::annotate(light::off, level{1});
    };
    struct lit {
        static constexpr auto annotations = fsm::annotate(light::on, level{1}, heat{true});
    };
    struct bare {}; // no set at all
    struct dead {
        static constexpr auto annotations = fsm::annotate(heat{false});
    };

    static_assert(dark::annotations.has<light> && dark::annotations.has<level>);
    static_assert(!dark::annotations.has<heat>);
    static_assert(dark::annotations.get<level>() == level{1});
    static_assert(std::is_same_v<decltype(lit::annotations)::types, mtl::typelist<light, level, heat>>);

    // an observer consuming two of the three elements through overloads
    struct panel : fsm::observing<panel> {
        void notifyEntry(light l) { lights.push_back(l); }
        void notifyEntry(level const& l) { levels.push_back(l.value); }
        void notifyExit(level const& l) { level_exits.push_back(l.value); }
        std::vector<light> lights;
        std::vector<int> levels;
        std::vector<int> level_exits;
    };

    // the coverage traits see set elements
    static_assert(fsm::is_observed_v<panel, dark>);
    static_assert(fsm::is_notified_of_v<panel, dark>);
    static_assert(!fsm::is_notified_of_v<panel, bare>);
    static_assert(!fsm::is_notified_of_v<panel, dead>); // heat has no hook in panel

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<dark>, fsm::on<next>, fsm::to<lit>>,
        fsm::transition<fsm::from<lit>,  fsm::on<next>, fsm::to<bare>>,
        fsm::transition<fsm::from<bare>, fsm::on<next>, fsm::to<dark>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>> {};

    // Whether an annotation type is carried by any state: the trait
    // behind fsm::observing's validate() for the types an observer
    // declares (an observer naming sound would not instantiate)
    struct sound {};
    static_assert(fsm::annotation_in_table_v<tbl, heat>);
    static_assert(!fsm::annotation_in_table_v<tbl, sound>);

    struct heater : fsm::observing<heater> {
        using observes = fsm::annotations<heat>;
        void notifyEntry(heat) { ++heats; }
        int heats = 0;
    };
} // namespace AnnotationSets

namespace Features {
    struct go {};
    struct swap_feature {};
    struct vconn_feature {};

    struct plain {};
    struct swapping {
        using feature = swap_feature;
    };
    struct powering {
        using feature = vconn_feature;
    };

    struct swap_policy {
        using enables = swap_feature;
    };
    struct both_policies {
        using enables = mtl::typelist<vconn_feature, swap_feature>;
    };
    struct bystander {};

    static_assert(fsm::observer_enables_v<swap_policy, swap_feature>);
    static_assert(!fsm::observer_enables_v<swap_policy, vconn_feature>);
    static_assert(fsm::observer_enables_v<both_policies, swap_feature>);
    static_assert(fsm::observer_enables_v<both_policies, vconn_feature>);
    static_assert(!fsm::observer_enables_v<bystander, swap_feature>);

    static_assert(fsm::state_in_feature_v<swapping, swap_feature>);
    static_assert(!fsm::state_in_feature_v<swapping, vconn_feature>);
    static_assert(!fsm::state_in_feature_v<plain, swap_feature>);

    static_assert(fsm::feature_enabled_v<swap_feature, bystander, swap_policy>);
    static_assert(!fsm::feature_enabled_v<vconn_feature, bystander, swap_policy>);
    static_assert(!fsm::feature_enabled_v<swap_feature>); // no observers at all

    using swap_in    = fsm::transition<fsm::from<plain>, fsm::on<go>, fsm::to<swapping>>;
    using swap_out   = fsm::transition<fsm::from<swapping>, fsm::on<go>, fsm::to<plain>>;
    using power_in   = fsm::transition<fsm::from<plain>, fsm::on<fsm::timeout>, fsm::to<powering>>;
    using plain_self = fsm::transition<fsm::from<plain>, fsm::on<fsm::timeout>, fsm::to<plain>>;
    using entries    = mtl::typelist<fsm::initial<swapping>, swap_in, swap_out, power_in, plain_self>;

    // one feature removed: its initial<> and both transitions go, the rest stays
    static_assert(std::is_same_v<fsm::remove_feature_t<entries, swap_feature>,
                                 mtl::typelist<power_in, plain_self>>);

    // filtered by observers: unenabled features go, featureless states stay
    static_assert(std::is_same_v<fsm::remove_disabled_features_t<entries, bystander>,
                                 mtl::typelist<plain_self>>);
    static_assert(std::is_same_v<fsm::remove_disabled_features_t<entries, swap_policy>,
                                 mtl::typelist<fsm::initial<swapping>, swap_in, swap_out, plain_self>>);
    static_assert(std::is_same_v<fsm::remove_disabled_features_t<entries, bystander, both_policies>, entries>);

    // the table built from the filtered list: without the initial<>, the
    // first remaining transition's source leads
    using trimmed = mtl::rebind_t<fsm::remove_disabled_features_t<entries, bystander>, fsm::transition_table>;
    static_assert(std::is_same_v<mtl::front_t<trimmed::states>, plain>);
    static_assert(!mtl::has_a_v<trimmed::states, swapping>);

    // several features in one pass, and the same filter on a timer-range map
    static_assert(std::is_same_v<
                  fsm::remove_features_t<entries, mtl::typelist<swap_feature, vconn_feature>>,
                  mtl::typelist<plain_self>>);

    constexpr fsm::timeout_range any_time{0us, 1s};
    using ranges = fsm::timer_ranges<fsm::timed_by<powering, any_time>, fsm::timed_by<plain, any_time>>;
    static_assert(std::is_same_v<fsm::remove_feature_t<ranges, vconn_feature>,
                                 fsm::timer_ranges<fsm::timed_by<plain, any_time>>>);
    static_assert(std::is_same_v<fsm::remove_disabled_features_t<ranges, both_policies>, ranges>);

    // the table a machine runs: the one given while nothing is disabled
    // (no observer list, or observers enabling every feature), else
    // rebuilt without the disabled features
    struct full_table
        : fsm::transition_table<fsm::initial<swapping>, swap_in, swap_out, plain_self> {};
    static_assert(std::is_same_v<fsm::enabled_table_t<full_table>, full_table>);
    static_assert(std::is_same_v<fsm::enabled_table_t<full_table, fsm::observers<both_policies>>,
                                 full_table>);
    using for_bystander = fsm::enabled_table_t<full_table, fsm::observers<bystander>>;
    static_assert(!std::is_same_v<for_bystander, full_table>);
    static_assert(std::is_same_v<for_bystander::states, mtl::typelist<plain>>);
    static_assert(std::is_same_v<for_bystander::transitions, mtl::typelist<plain_self>>);
    // the machine filters by its own observers and keeps the table's name
    static_assert(std::is_same_v<fsm::StateMachine<full_table, bystander>::table, full_table>);

    // a switch enables the tags whose condition holds, nothing else
    using swap_only = fsm::feature_switch<fsm::enabled<swap_feature, true>,
                                          fsm::enabled<vconn_feature, false>>;
    static_assert(std::is_same_v<swap_only::enables, mtl::typelist<swap_feature>>);
    static_assert(fsm::observer_enables_v<swap_only, swap_feature>);
    static_assert(!fsm::observer_enables_v<swap_only, vconn_feature>);
    static_assert(std::is_same_v<fsm::feature_switch<>::enables, mtl::typelist<>>);
    static_assert(std::is_same_v<fsm::enabled_table_t<full_table, fsm::observers<swap_only>>,
                                 full_table>);
    static_assert(std::is_same_v<
                  fsm::enabled_table_t<full_table, fsm::observers<fsm::feature_switch<>>>::states,
                  mtl::typelist<plain>>);

    // a tag enabled by a guard: answering the question is what brings
    // the feature in, declaring the tag still works too
    struct swap_allowed {};
    struct asked_swap_feature {
        using enabled_by = swap_allowed;
    };
    struct answering_policy {
        bool check(swap_allowed) { return true; }
    };
    struct asked_swapping {
        using feature = asked_swap_feature;
    };
    static_assert(fsm::observer_answers_for_v<asked_swap_feature, answering_policy>);
    static_assert(!fsm::observer_answers_for_v<asked_swap_feature, bystander>);
    static_assert(!fsm::observer_answers_for_v<swap_feature, answering_policy>); // no enabled_by
    static_assert(fsm::feature_enabled_v<asked_swap_feature, bystander, answering_policy>);
    static_assert(!fsm::feature_enabled_v<asked_swap_feature, bystander>);
    struct asked_table : fsm::transition_table<
        fsm::transition<fsm::from<plain>, fsm::on<go>, fsm::to<asked_swapping>, fsm::guard<swap_allowed>>,
        fsm::transition<fsm::from<asked_swapping>, fsm::on<go>, fsm::to<plain>>, plain_self> {};
    // the guard on the removed row needs no answerer: the machine builds without one
    static_assert(std::is_same_v<fsm::enabled_table_t<asked_table, fsm::observers<bystander>>::transitions,
                                 mtl::typelist<plain_self>>);
    static_assert(std::is_same_v<fsm::StateMachine<asked_table, bystander>::table, asked_table>);
    static_assert(std::is_same_v<fsm::enabled_table_t<asked_table, fsm::observers<answering_policy>>,
                                 asked_table>);
    // the feature's voice among the observers, either way of enabling
    static_assert(std::is_same_v<
                  fsm::feature_enabler_t<swap_feature, fsm::observers<bystander, swap_policy>>,
                  swap_policy>);
    static_assert(std::is_same_v<fsm::feature_enabler_t<vconn_feature, fsm::observers<bystander>>,
                                 mtl::nil_type>);
    static_assert(std::is_same_v<fsm::feature_enabler_t<asked_swap_feature,
                                                        fsm::observers<bystander, answering_policy>>,
                                 answering_policy>);
} // namespace Features

namespace InjectedObservers {
    struct go {};
    struct swap_feature {};
    struct allowed {};
    struct lamp {
        constexpr bool operator==(lamp const&) const = default;
    };

    struct idle {};
    struct lit {
        static constexpr auto annotations = fsm::annotate(lamp{});
    };
    struct swapping {
        using feature = swap_feature;
    };

    struct in_table : fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<lit>, fsm::guard<allowed>>,
        fsm::transition<fsm::from<lit>, fsm::on<go>, fsm::to<swapping>>,
        fsm::transition<fsm::from<swapping>, fsm::on<go>, fsm::to<idle>>> {};

    struct swap_policy {
        using enables = swap_feature;
        bool check(allowed) const { return true; }
    };
    struct second_voice {
        bool check(allowed) const { return false; }
    };
    struct lamp_driver : fsm::observing<lamp_driver> {
        void notifyEntry(lamp) {}
    };
    struct bystander {};

    using with_policy = fsm::internal::InjectedObservers<bystander, swap_policy, lamp_driver>;
    using without     = fsm::internal::InjectedObservers<bystander>;

    static_assert(std::is_same_v<with_policy::observer_list,
                                 mtl::typelist<bystander, swap_policy, lamp_driver>>);

    static_assert(with_policy::any_observer_enables<swap_feature>);
    static_assert(!without::any_observer_enables<swap_feature>);
    static_assert(std::is_same_v<with_policy::table_with_enabled_features<in_table>, in_table>);
    static_assert(!mtl::has_a_v<without::table_with_enabled_features<in_table>::states, swapping>);

    static_assert(with_policy::any_observer_notified_by<lit>);
    static_assert(!with_policy::any_observer_notified_by<idle>);
    static_assert(!without::any_observer_notified_by<lit>);

    static_assert(with_policy::any_observer_answers_guard<allowed, idle>);
    static_assert(!without::any_observer_answers_guard<allowed, idle>);
    static_assert(with_policy::every_guard_answered<in_table>);
    static_assert(!without::every_guard_answered<in_table>);
    static_assert(with_policy::no_guard_answered_by_two_observers<in_table>);
    static_assert(!fsm::internal::InjectedObservers<swap_policy, second_voice>::
                      no_guard_answered_by_two_observers<in_table>);

    static_assert(with_policy::all_observers_validate<in_table>);
} // namespace InjectedObservers

namespace SharedWildcard {
    struct go {};
    struct kill {
        int code;
    };

    struct mode_tag {
        constexpr bool operator==(mode_tag const&) const = default;
    };

    struct a {
        static constexpr auto timeout = 50ms;
        static constexpr mode_tag mode{};
    };
    struct b {
        static constexpr mode_tag mode{};
    };
    struct dead {
        dead() = default;
        explicit dead(kill const& event) : code(event.code) {}
        int code = 0;
    };

    struct mode_watcher : fsm::observing<mode_watcher> {
        int notified = 0;
        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::mode)
        {
            return STATE::mode;
        }
        void notifyEntry(mode_tag) { ++notified; }
    };

    // an exit hook: notified per source, so the value of the state
    // left arrives on a wildcard edge too
    struct exit_watcher : fsm::observing<exit_watcher> {
        int exits = 0;
        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::mode)
        {
            return STATE::mode;
        }
        void notifyExit(mode_tag) { ++exits; }
    };

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<a>, fsm::on<go>, fsm::to<b>>,
        fsm::transition<fsm::from<a>, fsm::on<fsm::timeout>, fsm::to<b>>,
        fsm::transition<fsm::from<b>, fsm::on<go>, fsm::to<a>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>> {};

    struct never {
        static bool check(b const&) { return false; }
    };

    // a wildcard back into an annotated state: its entry has no edge to
    // compare against
    struct reset {};
    struct home_tbl : fsm::transition_table<
        fsm::transition<fsm::from<a>, fsm::on<go>, fsm::to<b>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<reset>, fsm::to<a>>> {};

    // b has an own pair for kill whose guard refuses: the wildcard is
    // the next alternative, exactly like transitions_for says
    struct guarded_tbl : fsm::transition_table<
        fsm::transition<fsm::from<a>, fsm::on<go>, fsm::to<b>>,
        fsm::transition<fsm::from<a>, fsm::on<fsm::timeout>, fsm::to<b>>,
        fsm::transition<fsm::from<b>, fsm::on<go>, fsm::to<a>>,
        fsm::transition<fsm::from<b>, fsm::on<kill>, fsm::to<a>, fsm::guard<never>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>> {};
} // namespace SharedWildcard

namespace Deadline {
    struct step {};
    struct bounce {};
    struct retry {};

    // a two-state phase under one 80 ms budget; probing carries a
    // per-state timeout alongside the phase deadline
    struct searching {
        static constexpr auto deadline = 80ms;
    };
    struct probing {
        static constexpr auto deadline = 80ms;
        static constexpr auto timeout  = 20ms;
    };
    struct rearmed { // a different value: a new phase, re-armed
        static constexpr auto deadline = 30ms;
    };
    struct arrived { // the zero sentinel: the phase target, clock stopped
        static constexpr auto deadline = 0ms;
    };
    struct gave_up {};

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<searching>, fsm::on<step>,          fsm::to<probing>>,
        fsm::transition<fsm::from<probing>,   fsm::on<bounce>,        fsm::to<searching>>,
        fsm::transition<fsm::from<probing>,   fsm::on<fsm::timeout>,  fsm::to<searching>>,
        fsm::transition<fsm::from<probing>,   fsm::on<step>,          fsm::to<arrived>>,
        fsm::transition<fsm::from<searching>, fsm::on<fsm::deadline>, fsm::to<gave_up>>,
        fsm::transition<fsm::from<probing>,   fsm::on<fsm::deadline>, fsm::to<gave_up>>,
        fsm::transition<fsm::from<arrived>,   fsm::on<retry>,         fsm::to<rearmed>>,
        fsm::transition<fsm::from<rearmed>,   fsm::on<fsm::deadline>, fsm::to<gave_up>>,
        fsm::transition<fsm::from<gave_up>,   fsm::on<retry>,         fsm::to<searching>>> {};

    // the deadline bounds mirror of the timeout map checks
    inline constexpr fsm::timeout_range phase_range{70ms, 90ms};
    inline constexpr fsm::timeout_range short_range{20ms, 40ms};
    using ranges = fsm::timer_ranges<fsm::timed_by<searching, phase_range>,
                                 fsm::timed_by<probing, phase_range>,
                                 fsm::timed_by<rearmed, short_range>>;
    static_assert(fsm::deadlines_within_bounds_v<tbl, ranges>);
    static_assert(fsm::deadline_within_bounds_v<ranges, searching>);
    // the zero sentinel and unannotated states must have no entry ...
    static_assert(fsm::deadline_within_bounds_v<ranges, arrived>);
    static_assert(!fsm::deadline_within_bounds_v<
                  fsm::timer_ranges<fsm::timed_by<arrived, phase_range>>, arrived>);
    // ... and an active deadline outside its range fails
    static_assert(!fsm::deadline_within_bounds_v<
                  fsm::timer_ranges<fsm::timed_by<rearmed, phase_range>>, rearmed>);
} // namespace Deadline

// --- queued machine: run-to-completion delivery ------------------------------

namespace Queued {

struct go {};
struct halt {};
struct note {};

struct idle {};
struct armed {
    static constexpr auto timeout = 50ms;
    void handle(note const&) {}
};
struct done {};
struct timed_out {};

struct table : fsm::transition_table<
    fsm::initial<idle>,
    fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<armed>>,
    fsm::transition<fsm::from<armed>, fsm::on<halt>, fsm::to<done>>,
    fsm::transition<fsm::from<armed>, fsm::on<fsm::timeout>, fsm::to<timed_out>>,
    fsm::internal_transition<fsm::from<armed>, fsm::on<note>>> {};

// Drives the queue from inside a delivery: what a synchronous driver
// report from an entry hook does in production. The post runs before
// the expiry - the event "arrived" first
struct sync_actor {
    void* queue         = nullptr;
    void (*post)(void*) = nullptr;
    manual_timer* clock = nullptr;
    std::vector<char> log;

    template<typename STATE, typename MACHINE>
    void onEnter(MACHINE&)
    {
        if constexpr (std::is_same_v<STATE, armed>) {
            if (post != nullptr) {
                post(queue);
            }
            if (clock != nullptr) {
                clock->expire();
            }
        }
        if constexpr (std::is_same_v<STATE, done>) {
            log.push_back('d');
        }
        if constexpr (std::is_same_v<STATE, timed_out>) {
            log.push_back('t');
        }
    }

    template<typename EVENT, typename TO, typename MACHINE>
    void onTransition(MACHINE&)
    {
        if constexpr (std::is_same_v<EVENT, note>) {
            log.push_back('n');
        }
    }
};

using queued_timed = fsm::timed<fsm::QueuedTimer<manual_timer>&>;
using machine =
    fsm::QueuedMachine<table, 4, fsm::inline_work, fsm::no_lock, queued_timed, sync_actor>;

static_assert(fsm::concepts::timer<fsm::QueuedTimer<manual_timer>>);

// the owning form: the observer declared in one line, no timer to wire
using owning_timed = fsm::timed<fsm::OwningQueuedTimer<manual_timer>>;
using owning_machine =
    fsm::QueuedMachine<table, 4, fsm::inline_work, fsm::no_lock, owning_timed, sync_actor>;

static_assert(fsm::concepts::timer<fsm::OwningQueuedTimer<manual_timer>>);
// the channel base is only ever part of a queued timer
static_assert(!std::is_default_constructible_v<fsm::QueuedTimerBase>);

// a WORK the caller owns and configures, handed over by reference
struct counting_work {
    int submits = 0;
    void submit(fsm::work_callback callback, void* context)
    {
        ++submits;
        callback(context);
    }
};
using shared_work_machine =
    fsm::QueuedMachine<table, 4, counting_work&, fsm::no_lock, owning_timed, sync_actor>;

// which timers a table needs at all
static_assert(fsm::has_timed_states_v<table>);
static_assert(!fsm::has_deadlined_states_v<table>);

} // namespace Queued

namespace QueuedDeadline {

struct advance {};
struct finish {};

struct phase_a {
    static constexpr auto deadline = 100ms;
};
struct phase_b {
    static constexpr auto deadline = 100ms; // same budget: the phase continues
};
struct expired {};
struct finished {};

struct table : fsm::transition_table<
    fsm::initial<phase_a>,
    fsm::transition<fsm::from<phase_a>, fsm::on<advance>, fsm::to<phase_b>>,
    fsm::transition<fsm::from<phase_a>, fsm::on<fsm::deadline>, fsm::to<expired>>,
    fsm::transition<fsm::from<phase_b>, fsm::on<fsm::deadline>, fsm::to<expired>>,
    fsm::transition<fsm::from<phase_b>, fsm::on<finish>, fsm::to<finished>>> {};

struct sync_actor {
    void* queue         = nullptr;
    void (*post)(void*) = nullptr;
    manual_timer* clock = nullptr;

    template<typename STATE, typename MACHINE>
    void onEnter(MACHINE&)
    {
        if constexpr (std::is_same_v<STATE, phase_b>) {
            if (post != nullptr) {
                post(queue); // queued first...
            }
            if (clock != nullptr) {
                clock->expire(); // ...but the deadline gates progress
            }
        }
    }
};

using queued_deadlined = fsm::deadlined<fsm::QueuedTimer<manual_timer>&>;
using machine = fsm::QueuedMachine<table, 4, fsm::inline_work, fsm::no_lock,
                                   queued_deadlined, sync_actor>;

} // namespace QueuedDeadline

namespace Nested {

struct go {};
struct stop {};
struct tick {};
struct inner_only {};

struct power {
    bool on;
    constexpr bool operator==(power const&) const = default;
};
struct lamp {
    bool lit;
    constexpr bool operator==(lamp const&) const = default;
};

struct never {
    static bool check() { return false; }
};

// the sub-states: a timed one, a guarded row that always refuses
struct low {
    static constexpr auto annotations = fsm::annotate(lamp{false});
};
struct high {
    static constexpr auto timeout     = 10ms;
    static constexpr auto annotations = fsm::annotate(lamp{true});
    void handle(inner_only const&) {}
};

struct inner_table : fsm::transition_table<
    fsm::transition<fsm::from<low>,  fsm::on<tick>,         fsm::to<high>>,
    fsm::transition<fsm::from<high>, fsm::on<fsm::timeout>, fsm::to<low>>,
    fsm::transition<fsm::from<high>, fsm::on<tick>,         fsm::to<low>, fsm::guard<never>>,
    fsm::internal_transition<fsm::from<high>, fsm::on<inner_only>>> {};

struct idle {
    static constexpr auto annotations = fsm::annotate(power{false});
};
// the composite: timed itself, annotated at its own level only
struct active {
    using submachine = inner_table;
    static constexpr auto timeout     = 100ms;
    static constexpr auto annotations = fsm::annotate(power{true});
};
struct done {};

struct reset {};

struct outer_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>,   fsm::on<go>,           fsm::to<active>>,
    fsm::transition<fsm::from<active>, fsm::on<stop>,         fsm::to<done>>,
    fsm::transition<fsm::from<active>, fsm::on<fsm::timeout>, fsm::to<done>>,
    fsm::transition<fsm::from<active>, fsm::on<tick>,         fsm::to<idle>>, // only when the child refused
    fsm::transition<fsm::from<done>,   fsm::on<go>,           fsm::to<idle>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<reset>, fsm::to<idle>>> {};

// Records every hook with the machine it came from - the table's short
// name tells the level
struct recorder {
    template<typename STATE, typename MACHINE>
    void onEnter(MACHINE&)
    {
        log.push_back(std::string{"enter "} + mtl::short_name_of<typename MACHINE::table> + ":" +
                      mtl::short_name_of<STATE>);
    }

    template<typename STATE, typename MACHINE>
    void onExit(MACHINE&)
    {
        log.push_back(std::string{"exit "} + mtl::short_name_of<typename MACHINE::table> + ":" +
                      mtl::short_name_of<STATE>);
    }

    template<typename EVENT, typename TO, typename MACHINE>
    void onTransition(MACHINE&)
    {
        log.push_back(std::string{"transition "} + mtl::short_name_of<typename MACHINE::table> +
                      ":" + mtl::short_name_of<TO>);
    }

    std::vector<std::string> log;
};

// Sees both levels' annotations through one observer
struct level_watcher : fsm::observing<level_watcher> {
    void notifyEntry(power value) { powers.push_back(value); }
    void notifyEntry(lamp value) { lamps.push_back(value); }
    void notifyExit(lamp value) { left_lamps.push_back(value); }

    std::vector<power> powers;
    std::vector<lamp> lamps;
    std::vector<lamp> left_lamps;
};

using nested_machine = fsm::StateMachine<outer_table, recorder, level_watcher>;
static_assert(nested_machine::depth == 0);
static_assert(std::is_same_v<nested_machine::table, outer_table>);

// annotation completeness per element, over the hierarchy: the
// composite state carries power for its whole submachine, and leaves
// lamp to its sub-states
static_assert(fsm::all_states_carry_v<inner_table, lamp>);
static_assert(!fsm::all_states_carry_v<outer_table, power>); // done carries none
static_assert(fsm::all_states_carry_v<outer_table, power, mtl::typelist<done>>);
static_assert(!fsm::all_states_carry_v<outer_table, lamp, mtl::typelist<done>>); // nor idle
static_assert(fsm::all_states_carry_v<outer_table, lamp, mtl::typelist<idle, done>>);

// reachability and the timer-range maps cover the sub-states
static_assert(fsm::all_states_reachable_v<outer_table>);
struct unreached {};
struct orphaned_inner_table : fsm::transition_table<
    fsm::transition<fsm::from<low>,       fsm::on<tick>, fsm::to<high>>,
    fsm::transition<fsm::from<high>,      fsm::on<fsm::timeout>, fsm::to<low>>,
    fsm::transition<fsm::from<unreached>, fsm::on<tick>, fsm::to<low>>> {};
struct nesting_an_orphan {
    using submachine = orphaned_inner_table;
};
struct orphaned_outer_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<nesting_an_orphan>>> {};
static_assert(fsm::all_states_reachable_v<fsm::transition_table<
                  fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<done>>>>);
static_assert(!fsm::all_states_reachable_v<orphaned_outer_table>);

inline constexpr fsm::timeout_range active_budget{.min = 50ms, .max = 150ms};
inline constexpr fsm::timeout_range high_pulse{.min = 5ms, .max = 20ms};
static_assert(fsm::timeouts_within_bounds_v<
    outer_table, fsm::timer_ranges<fsm::timed_by<active, active_budget>,
                               fsm::timed_by<high, high_pulse>>>);
// a timed sub-state without an entry is caught from the root table
static_assert(!fsm::timeouts_within_bounds_v<
    outer_table, fsm::timer_ranges<fsm::timed_by<active, active_budget>>>);

// event coverage: every state handles the events it owes in its own table
template<typename STATE>
struct owes_tick : std::type_identity<fsm::events<>> {};
template<>
struct owes_tick<low> : std::type_identity<fsm::events<tick>> {};
template<>
struct owes_tick<high> : std::type_identity<fsm::events<tick>> {}; // a guarded row counts
static_assert(fsm::all_states_handle_v<outer_table, owes_tick>);
template<typename STATE>
struct owes_stop : std::type_identity<fsm::events<>> {};
template<>
struct owes_stop<low> : std::type_identity<fsm::events<stop>> {}; // the parent's row is not low's
static_assert(!fsm::all_states_handle_v<outer_table, owes_stop>);

// a composite without a timeout of its own: the table is timed through its child
struct wrapper {
    using submachine = inner_table;
};
struct wrapped_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<wrapper>>> {};

// the declaration and its traits
static_assert(fsm::internal::composite<active>);
static_assert(!fsm::internal::composite<idle>);
static_assert(std::is_same_v<fsm::internal::submachine_t<active>, inner_table>);
static_assert(std::is_same_v<fsm::internal::submachine_t<idle>, mtl::nil_type>);

// levels: a flat table is one, each nesting adds one
static_assert(fsm::levels_v<inner_table> == 1);
static_assert(fsm::levels_v<outer_table> == 2);
static_assert(fsm::levels_v<wrapped_table> == 2);

// the tables and states of the whole machine, root first
static_assert(std::is_same_v<fsm::nested_tables_t<outer_table>,
                             mtl::typelist<outer_table, inner_table>>);
static_assert(std::is_same_v<fsm::nested_tables_t<inner_table>, mtl::typelist<inner_table>>);
static_assert(std::is_same_v<fsm::all_states_t<outer_table>,
                             mtl::typelist<idle, active, done, low, high>>);
// every level's events, each once (tick is on both levels)
static_assert(std::is_same_v<fsm::nested_events_t<outer_table>,
                             mtl::typelist<go, stop, fsm::timeout, tick, reset, inner_only>>);

// a table is timed through a timed sub-state alone
static_assert(fsm::has_timed_states_v<wrapped_table>);
static_assert(!fsm::has_deadlined_states_v<wrapped_table>);
static_assert(fsm::annotation_in_table_v<wrapped_table, lamp>); // carried by a sub-state

// timer events stay with the machine they were injected into
static_assert(fsm::local_event_v<fsm::timeout>);
static_assert(fsm::local_event_v<fsm::deadline>);
static_assert(!fsm::local_event_v<tick>);

// the wrapper a parent builds its child from: the same table plus its depth
using child_table = fsm::internal::nested<inner_table, 1>;
static_assert(fsm::concepts::transition_table<child_table>);
static_assert(child_table::depth == 1);
static_assert(fsm::internal::table_depth_v<child_table> == 1);
static_assert(fsm::internal::table_depth_v<inner_table> == 0);
static_assert(std::is_same_v<fsm::internal::plain_table_t<child_table>, inner_table>);
static_assert(std::is_same_v<fsm::internal::plain_table_t<inner_table>, inner_table>);

// an annotation type lives on one level of a nesting path
struct refining {
    using submachine = inner_table;
    static constexpr auto annotations = fsm::annotate(lamp{true}); // lamp is the sub-states'
};
static_assert(fsm::internal::annotation_levels_exclusive<active>::value);
static_assert(fsm::internal::annotation_levels_exclusive<idle>::value);
static_assert(!fsm::internal::annotation_levels_exclusive<refining>::value);

// --- inherited contexts: a composite state's child shares its
// machine's instance, everything else the child declares is its own

struct port_line {
    int cc = 0; // the root's, inherited two levels down
};
struct phase_budget {
    int tries = 0; // the probing phase's own, fresh on every entry
};
struct sense {
    int cc;
};

struct probing {
    using contexts = fsm::contexts<port_line, phase_budget>;
    probing(port_line& line_ref, phase_budget& budget_ref) : line(line_ref), budget(budget_ref) {}
    probing(sense const& event, port_line& line_ref, phase_budget& budget_ref)
        : probing(line_ref, budget_ref)
    {
        line.cc = event.cc;
        ++budget.tries;
    }
    port_line& line;
    phase_budget& budget;
};

struct probe_table : fsm::transition_table<
    fsm::transition<fsm::from<probing>, fsm::on<sense>, fsm::to<probing>>> {};

// the phase passes the line on, declaring no context of its own: its
// child inherits what it inherited
struct trying {
    using submachine      = probe_table;
    using parent_contexts = fsm::contexts<port_line>;
};
struct waiting {};

struct session_table : fsm::transition_table<
    fsm::transition<fsm::from<waiting>, fsm::on<go>, fsm::to<trying>>> {};

struct resting {
    using contexts = fsm::contexts<port_line>;
    explicit resting(port_line&) {}
};
struct session {
    using submachine      = session_table;
    using parent_contexts = fsm::contexts<port_line>;
};

struct inheriting_table : fsm::transition_table<
    fsm::transition<fsm::from<resting>, fsm::on<go>,   fsm::to<session>>,
    fsm::transition<fsm::from<session>, fsm::on<stop>, fsm::to<resting>>> {};

using inheriting_machine = fsm::StateMachine<inheriting_table>;
using session_machine = std::remove_cvref_t<
    decltype(*std::declval<inheriting_machine const&>().submachine<session>())>;
using probe_machine =
    std::remove_cvref_t<decltype(*std::declval<session_machine const&>().submachine<trying>())>;

// the declaration
static_assert(fsm::internal::declares_parent_contexts<trying>);
static_assert(!fsm::internal::declares_parent_contexts<waiting>);
static_assert(std::is_same_v<fsm::internal::parent_contexts_t<trying>, mtl::typelist<port_line>>);
static_assert(std::is_same_v<fsm::internal::parent_contexts_t<waiting>, mtl::typelist<>>);

// each level's contexts: the root owns the line, the session only
// inherits it, the probe inherits it next to its own budget
static_assert(std::is_same_v<inheriting_machine::own_contexts, mtl::typelist<port_line>>);
static_assert(std::is_same_v<inheriting_machine::inherited_contexts, mtl::typelist<>>);
static_assert(std::is_same_v<session_machine::own_contexts, mtl::typelist<>>);
static_assert(std::is_same_v<session_machine::inherited_contexts, mtl::typelist<port_line>>);
static_assert(std::is_same_v<probe_machine::own_contexts, mtl::typelist<phase_budget>>);
static_assert(std::is_same_v<probe_machine::inherited_contexts, mtl::typelist<port_line>>);
static_assert(std::is_same_v<probe_machine::context_types, mtl::typelist<phase_budget, port_line>>);

// the tuple holds an inherited context as a reference
static_assert(
    fsm::internal::holds_inherited_context<port_line, std::tuple<phase_budget, port_line&>>::value);
static_assert(!fsm::internal::holds_inherited_context<phase_budget,
                                                      std::tuple<phase_budget, port_line&>>::value);

// the checks on a parent_contexts declaration, each askable per state
struct plain_with_parent_contexts {
    using parent_contexts = fsm::contexts<port_line>; // no submachine to inherit it
};
struct parent_contexts_unused {
    using submachine      = inner_table; // low and high declare no context
    using parent_contexts = fsm::contexts<port_line>;
};
static_assert(fsm::internal::parent_contexts_on_composite<trying>::value);
static_assert(fsm::internal::parent_contexts_on_composite<waiting>::value);
static_assert(!fsm::internal::parent_contexts_on_composite<plain_with_parent_contexts>::value);
static_assert(fsm::internal::parent_contexts_held_in<mtl::typelist<port_line>>::pred<trying>::value);
static_assert(
    !fsm::internal::parent_contexts_held_in<mtl::typelist<phase_budget>>::pred<trying>::value);
static_assert(fsm::internal::parent_contexts_declared_in_submachine<trying>::value);
static_assert(fsm::internal::parent_contexts_declared_in_submachine<session>::value); // two levels
static_assert(!fsm::internal::parent_contexts_declared_in_submachine<parent_contexts_unused>::value);

// --- a feature inside a submachine: the child machine filters its
// table by the same observers as the root

struct boost_feature {};
struct push {};

struct calm {};
struct warm {};
struct boost {
    using feature = boost_feature;
};

struct deep_table : fsm::transition_table<
    fsm::transition<fsm::from<calm>,  fsm::on<tick>, fsm::to<warm>>,
    fsm::transition<fsm::from<warm>,  fsm::on<tick>, fsm::to<calm>>,
    fsm::transition<fsm::from<calm>,  fsm::on<push>, fsm::to<boost>>,
    fsm::transition<fsm::from<boost>, fsm::on<push>, fsm::to<calm>>> {};

struct engine {
    using submachine = deep_table;
};
// the same feature at the root, and a composite that is itself a feature state
struct turbo {
    using feature = boost_feature;
};
struct optional_engine {
    using feature    = boost_feature;
    using submachine = deep_table;
};

struct featured_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>,            fsm::on<go>,   fsm::to<engine>>,
    fsm::transition<fsm::from<engine>,          fsm::on<stop>, fsm::to<idle>>,
    fsm::transition<fsm::from<idle>,            fsm::on<push>, fsm::to<turbo>>,
    fsm::transition<fsm::from<turbo>,           fsm::on<stop>, fsm::to<idle>>,
    fsm::transition<fsm::from<idle>,            fsm::on<tick>, fsm::to<optional_engine>>,
    fsm::transition<fsm::from<optional_engine>, fsm::on<stop>, fsm::to<idle>>> {};

struct booster {
    using enables = boost_feature;
};

// disabled: both levels lose the feature's states, and push with them
static_assert(std::is_same_v<fsm::all_states_t<featured_table, fsm::observers<>>,
                             mtl::typelist<idle, engine, calm, warm>>);
static_assert(!mtl::has_a_v<fsm::nested_events_t<featured_table, fsm::observers<>>, push>);
// enabled, or no observer list at all: everything in view, the sub-table as named
static_assert(std::is_same_v<fsm::all_states_t<featured_table, fsm::observers<booster>>,
                             mtl::typelist<idle, engine, turbo, optional_engine, calm, warm, boost>>);
static_assert(std::is_same_v<fsm::nested_tables_t<featured_table, fsm::observers<booster>>,
                             mtl::typelist<featured_table, deep_table>>);
static_assert(std::is_same_v<fsm::nested_tables_t<featured_table>,
                             mtl::typelist<featured_table, deep_table>>);
static_assert(mtl::has_a_v<fsm::nested_events_t<featured_table>, push>);
// the queued machine's ring follows its observers
using queued_off = fsm::QueuedMachine<featured_table, 4, fsm::inline_work, fsm::no_lock, recorder>;
using queued_on =
    fsm::QueuedMachine<featured_table, 4, fsm::inline_work, fsm::no_lock, booster, recorder>;
static_assert(!mtl::has_a_v<queued_off::queueable_events, push>);
static_assert(mtl::has_a_v<queued_on::queueable_events, push>);

} // namespace Nested

namespace Final {

struct start {};
struct fail {};
struct poke {};
struct retry {};
struct halt {};

// --- a root that ends: nothing leaves a final state, wildcards included
struct working {};
struct stopped {};
struct finished {};

struct ending_table : fsm::transition_table<
    fsm::transition<fsm::from<working>, fsm::on<start>, fsm::to<finished>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<halt>, fsm::to<stopped>>,
    fsm::transition<fsm::from<stopped>, fsm::on<start>, fsm::to<working>>,
    fsm::final<finished>> {};

static_assert(std::is_same_v<fsm::final_states_t<ending_table>, mtl::typelist<finished>>);
static_assert(mtl::empty_v<fsm::final_states_t<Nested::outer_table>>);
// the role is no transition, and the wildcard does not apply to the final state
static_assert(std::is_same_v<ending_table::states, mtl::typelist<working, stopped, finished>>);
static_assert(!mtl::empty_v<fsm::transitions_for_t<ending_table, working, halt>>);
static_assert(mtl::empty_v<fsm::transitions_for_t<ending_table, finished, halt>>);
static_assert(fsm::internal::wildcard_source_v<ending_table, working, halt>);
static_assert(!fsm::internal::wildcard_source_v<ending_table, finished, halt>);
static_assert(fsm::all_states_reachable_v<ending_table>);

// --- a submachine telling its parent: the events its states emit
struct progress {};
struct attempt_succeeded {};
struct attempt_failed {};

struct trying {
    static constexpr auto timeout = 10ms;
};
// not final: the parent's guard decides whether the attempt goes on
struct halfway {
    using emits = progress;
    void handle(poke const&) {}
};
struct succeeded {
    using emits = attempt_succeeded;
};
struct failed {
    using emits = attempt_failed;
};
struct given_up {}; // final and silent: the submachine rests finished

struct attempt_table : fsm::transition_table<
    fsm::transition<fsm::from<trying>,  fsm::on<start>,        fsm::to<halfway>>,
    fsm::transition<fsm::from<trying>,  fsm::on<fsm::timeout>, fsm::to<failed>>,
    fsm::transition<fsm::from<trying>,  fsm::on<fail>,         fsm::to<given_up>>,
    fsm::internal_transition<fsm::from<halfway>, fsm::on<poke>>,
    fsm::transition<fsm::from<halfway>, fsm::on<start>,        fsm::to<succeeded>>,
    fsm::final<succeeded>,
    fsm::final<failed>,
    fsm::final<given_up>> {};

struct idle {};
struct attempt {
    using submachine = attempt_table;
};
struct done {};
struct broken {};
struct interrupted {};

// the table's question at the halfway report, answered by an injected object
struct stop_halfway {};

struct job_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>,    fsm::on<start>,             fsm::to<attempt>>,
    fsm::transition<fsm::from<attempt>, fsm::on<progress>,          fsm::to<interrupted>,
                    fsm::guard<stop_halfway>>,
    fsm::transition<fsm::from<attempt>, fsm::on<attempt_succeeded>, fsm::to<done>>,
    fsm::transition<fsm::from<attempt>, fsm::on<attempt_failed>,    fsm::to<broken>>,
    fsm::transition<fsm::from<attempt>, fsm::on<retry>,             fsm::to<attempt>>> {};

struct interrupter {
    bool check(stop_halfway)
    {
        ++asked;
        return stop;
    }
    bool stop = false;
    int asked = 0;
};

static_assert(std::is_same_v<fsm::final_states_t<attempt_table>,
                             mtl::typelist<succeeded, failed, given_up>>);
static_assert(std::is_same_v<fsm::emitted_events_t<attempt_table>,
                             mtl::typelist<progress, attempt_failed, attempt_succeeded>>);
static_assert(std::is_same_v<fsm::emitted_events_t<job_table>,
                             mtl::typelist<progress, attempt_failed, attempt_succeeded>>);
static_assert(mtl::empty_v<fsm::emitted_events_t<ending_table>>);

// a composite state owes a transition for every event its submachine emits
static_assert(
    fsm::internal::emitted_events_taken_in<job_table, mtl::nil_type>::pred<attempt>::value);
struct careless_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>,    fsm::on<start>,          fsm::to<attempt>>,
    fsm::transition<fsm::from<attempt>, fsm::on<attempt_failed>, fsm::to<broken>>> {};
static_assert(
    !fsm::internal::emitted_events_taken_in<careless_table, mtl::nil_type>::pred<attempt>::value);

// a local event for a level below is decorated once per level on the way up
static_assert(std::is_same_v<fsm::for_level_t<fsm::timeout, 0>, fsm::timeout>);
static_assert(std::is_same_v<fsm::for_level_t<fsm::timeout, 2>,
                             fsm::for_submachine<fsm::for_submachine<fsm::timeout>>>);
static_assert(!fsm::local_event_v<fsm::for_submachine<fsm::timeout>>);

// an emitted event is taken inside the machine: the ring does not carry it
using queued_timers = fsm::timed<fsm::OwningQueuedTimer<manual_timer>, 2>;
using queued_job =
    fsm::QueuedMachine<job_table, 4, fsm::inline_work, fsm::no_lock, queued_timers, interrupter>;
static_assert(mtl::has_a_v<queued_job::queueable_events, start>);
static_assert(!mtl::has_a_v<queued_job::queueable_events, progress>);
static_assert(!mtl::has_a_v<queued_job::queueable_events, attempt_failed>);

// --- three levels: an end taken one level up may end that level in turn
struct campaign_failed {};

struct campaign_lost {
    using emits = campaign_failed;
};
struct battle {
    using submachine = attempt_table;
    void handle(progress const&) {}
};

struct campaign_table : fsm::transition_table<
    fsm::internal_transition<fsm::from<battle>, fsm::on<progress>>,
    fsm::transition<fsm::from<battle>, fsm::on<attempt_succeeded>, fsm::to<battle>>,
    fsm::transition<fsm::from<battle>, fsm::on<attempt_failed>,    fsm::to<campaign_lost>>,
    fsm::final<campaign_lost>> {};

struct war {
    using submachine = campaign_table;
};
struct peace {};

struct war_table : fsm::transition_table<
    fsm::transition<fsm::from<war>, fsm::on<campaign_failed>, fsm::to<peace>>> {};

static_assert(fsm::levels_v<war_table> == 3);

// --- a feature takes its final<> entry along
struct ending_feature {};
struct optional_end {
    using feature = ending_feature;
};
struct featured_ending_table : fsm::transition_table<
    fsm::transition<fsm::from<working>, fsm::on<start>, fsm::to<optional_end>>,
    fsm::transition<fsm::from<working>, fsm::on<fail>,  fsm::to<stopped>>,
    fsm::final<optional_end>> {};
static_assert(std::is_same_v<fsm::final_states_t<featured_ending_table>,
                             mtl::typelist<optional_end>>);
static_assert(mtl::empty_v<
    fsm::final_states_t<fsm::enabled_table_t<featured_ending_table, fsm::observers<>>>>);

} // namespace Final

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
    check(!tim.timer().armed); // off has no timeout
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
    check(tim.timer().armed);
    check(tim.timer().duration == 50ms);

    sm.process(button_press{}); // running -> off: leaving must disarm
    check(!tim.timer().armed);
}

void timeoutChain()
{
    output_controller ctrl;
    fsm::timed<manual_timer> tim;
    machine sm{tim, ctrl};
    sm.process(button_press{}); // off -> running

    tim.timer().expire();        // running -> cooldown (led off, fan still on)
    check(sm.is<cooldown>());
    check(tim.timer().armed);    // cooldown re-arms with its own timeout
    check(tim.timer().duration == 100ms);
    check(ctrl.log.size() == 3 && ctrl.log.back() == cooldown::outputs);

    tim.timer().expire();        // cooldown -> off
    check(sm.is<off>());
    check(!tim.timer().armed);
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
    fsm::StateMachine<ExplicitInitial::lock_first> sm;

    check(sm.is<locked>());
    check(sm.process(lock_key{})); // locked -> off
    check(sm.is<off>());
}

void anyStateReachesTargetFromEverywhere()
{
    using namespace Wildcard;
    fsm::StateMachine<tbl> sm;

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
    fsm::StateMachine<tbl> sm;

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
    fsm::StateMachine<tbl, live_driver> sm{driver};

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
    fsm::StateMachine<tbl, tx_driver> sm{driver};

    sm.process(send{.msg = {.id = 7}});
    sm.process(cancel{});
    sm.process(send{.msg = {.id = 9}});

    check(driver.transmitted.size() == 2);
    check(driver.transmitted[0] == 7 && driver.transmitted[1] == 9);
}

void machineWithOnlyATimerObserver()
{
    fsm::timed<manual_timer> tim;
    fsm::StateMachine<table, fsm::timed<manual_timer>> sm{tim};

    check(sm.process(button_press{}));
    check(sm.is<running>());
    check(tim.timer().armed); // running is a timed state
}

// --- entry is construction, exit is destruction -----------------------------
namespace lifetime {
    int entries = 0;
    int exits   = 0;

    struct ping {};
    struct plain { ~plain() { ++exits; } };      // counts its exits
    struct counted { counted() { ++entries; } }; // counts its entries

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<plain>,   fsm::on<ping>, fsm::to<counted>>,
        fsm::transition<fsm::from<counted>, fsm::on<ping>, fsm::to<plain>>> {};
} // namespace lifetime

void entryIsConstructionExitIsDestruction()
{
    using namespace lifetime;
    fsm::StateMachine<tbl> sm; // no timed states, no observers: nothing to inject
    check(entries == 0 && exits == 0); // the initial state is constructed in place, once

    sm.process(ping{}); // plain -> counted: ~plain(), counted()
    check(exits == 1);
    check(entries == 1);

    sm.process(ping{}); // counted -> plain: neither counts this edge
    check(exits == 1);
    check(entries == 1);
}

// --- guarded transitions ----------------------------------------------------
namespace guards {
    struct push {};
    struct unlock {};
    struct gate {
        bool open = false;
        void handle(unlock const&) { open = true; }
    };
    struct passed {};

    struct gate_is_open { // state-argument form: condition on state data
        static bool check(gate const& s) { return s.open; }
    };
    struct return_allowed { // no-argument form, toggled by the test
        static inline bool allow = false;
        static bool check() { return allow; }
    };

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<gate>,   fsm::on<push>, fsm::to<passed>,
                        fsm::guard<gate_is_open>>,
        fsm::internal_transition<fsm::from<gate>, fsm::on<unlock>>,
        fsm::transition<fsm::from<passed>, fsm::on<push>, fsm::to<gate>,
                        fsm::guard<return_allowed>>> {};
} // namespace guards

// --- injected guards: the table asks, an injected object answers -----------
namespace InjectedGuards {
    struct push {};
    struct closed {};
    struct open {};

    // the table's questions: a pure tag that must be answered, and a tag
    // with a static default an injected answer overrides
    struct door_unlocked {};
    struct after_hours {
        static bool check() { return false; }
    };

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<closed>, fsm::on<push>, fsm::to<open>,
                        fsm::guard<door_unlocked>>,
        fsm::transition<fsm::from<open>,   fsm::on<push>, fsm::to<closed>,
                        fsm::guard<after_hours>>> {};

    // answers door_unlocked from its own data and counts entries: a guard
    // and an observer at once, the table naming neither
    struct keeper {
        bool unlocked = false;
        int opened    = 0;
        bool check(door_unlocked, closed const&) const { return unlocked; }
        template<typename STATE, typename MACHINE>
        void onEnter(MACHINE&)
        {
            if constexpr (std::is_same_v<STATE, open>) {
                ++opened;
            }
        }
    };
    // overrides after_hours' static default
    struct wall_clock {
        bool late = false;
        bool check(after_hours) const { return late; }
    };

    static_assert(fsm::concepts::answers_guard_for<keeper, door_unlocked, closed>);
    static_assert(!fsm::concepts::answers_guard_for<keeper, after_hours, open>);
    static_assert(fsm::concepts::answers_guard_for<wall_clock, after_hours, open>);
    static_assert(fsm::concepts::guard_for<after_hours, open>);    // answers itself
    static_assert(!fsm::concepts::guard_for<door_unlocked, closed>); // a pure tag
} // namespace InjectedGuards

// --- combined guards: a conjunction of parts, a part inverted ---------------
namespace CombinedGuards {
    struct push {};
    struct closed {
        void handle(push const&) {} // locked: the push does nothing
    };
    struct open {};
    struct refused {};

    struct door_unlocked { // static
        static inline bool unlocked = false;
        static bool check() { return unlocked; }
    };
    struct after_hours {}; // a pure tag, injected answer

    struct wall_clock {
        bool late = false;
        bool check(after_hours) const { return late; }
    };

    // unlocked and not late opens; unlocked but late is refused; locked
    // stays - a disjunction is the next row of the pair
    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<closed>, fsm::on<push>, fsm::to<open>,
                        fsm::guard<door_unlocked, fsm::not_<after_hours>>>,
        fsm::transition<fsm::from<closed>, fsm::on<push>, fsm::to<refused>,
                        fsm::guard<door_unlocked>>,
        fsm::internal_transition<fsm::from<closed>, fsm::on<push>>,
        fsm::transition<fsm::from<open>,    fsm::on<push>, fsm::to<closed>>,
        fsm::transition<fsm::from<refused>, fsm::on<push>, fsm::to<closed>>> {};

    using opening = mtl::front_t<tbl::transitions>;
    static_assert(std::is_same_v<opening::guards,
                                 mtl::typelist<door_unlocked, fsm::not_<after_hours>>>);
    static_assert(fsm::internal::is_negated_v<fsm::not_<after_hours>>);
    static_assert(!fsm::internal::is_negated_v<after_hours>);
    static_assert(std::is_same_v<fsm::internal::guard_of_t<fsm::not_<after_hours>>, after_hours>);
} // namespace CombinedGuards

void combinedGuardsAskEveryPart()
{
    using namespace CombinedGuards;
    wall_clock clock;
    fsm::StateMachine<tbl, wall_clock> sm{clock};

    door_unlocked::unlocked = false;
    check(sm.process(push{})); // the internal row: locked, nothing changes
    check(sm.is<closed>());

    door_unlocked::unlocked = true; // both parts hold: static yes, inverted injected no
    check(sm.process(push{}));
    check(sm.is<open>());
    check(sm.process(push{})); // back

    clock.late = true; // the inverted part refuses, the next row takes it
    check(sm.process(push{}));
    check(sm.is<refused>());
}

void guardBlocksAndAllows()
{
    using namespace guards;
    fsm::StateMachine<tbl> sm;

    check(!sm.process(push{})); // gate closed: guard blocks, nothing happens
    check(sm.is<gate>());

    check(sm.process(unlock{})); // the state opens itself in place
    check(sm.process(push{}));   // guard passes now
    check(sm.is<passed>());

    return_allowed::allow = false;
    check(!sm.process(push{})); // no-argument guard form blocks
    check(sm.is<passed>());

    return_allowed::allow = true;
    check(sm.process(push{}));
    check(sm.is<gate>());
    check(!sm.getIf<gate>()->open); // re-entry default-constructs the state
}

void injectedObjectAnswersGuard()
{
    using namespace InjectedGuards;
    keeper k;
    wall_clock clock;
    fsm::StateMachine<tbl, keeper, wall_clock> sm{k, clock};

    check(!sm.process(push{})); // door_unlocked: the keeper says no
    check(sm.is<closed>());
    k.unlocked = true;          // the behavior changes from outside the table
    check(sm.process(push{}));
    check(sm.is<open>() && k.opened == 1);

    check(!sm.process(push{})); // after_hours: the injected clock overrides the static default
    clock.late = true;
    check(sm.process(push{}));
    check(sm.is<closed>());
}

void staticGuardIsTheDefaultAnswer()
{
    using namespace InjectedGuards;
    keeper k;
    fsm::StateMachine<tbl, keeper> sm{k}; // nobody answers after_hours: its static check does

    k.unlocked = true;
    check(sm.process(push{}));
    check(!sm.process(push{})); // the static default: never late
    check(sm.is<open>());
}

// --- raw lifecycle hooks (observer without the fsm::observing base) ---------
namespace raw_hooks {
    struct transition_counter {
        template<typename STATE, typename MACHINE>
        void onExit(MACHINE&) { ++exits; }

        template<typename STATE, typename MACHINE>
        void onEnter(MACHINE&) { ++enters; }

        int exits  = 0;
        int enters = 0;
    };
} // namespace raw_hooks

// --- transition hook: the edge and its event, after the change --------------
namespace transition_hook {
    struct go {};
    struct tick {};
    struct kill {
        int code;
    };

    struct idle {
        int ticks = 0;
        void handle(tick const&) { ++ticks; }
    };
    struct busy {};
    struct dead {
        dead() = default;
        explicit dead(kill const& event) : code(event.code) {}
        int code = 0;
    };

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>,
        fsm::internal_transition<fsm::from<idle>, fsm::on<tick>>,
        fsm::transition<fsm::from<busy>, fsm::on<go>, fsm::to<idle>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>> {};

    struct step {
        std::string_view from;
        std::string_view event;
        std::string_view to;
        bool operator==(step const&) const = default;
    };

    // the edge form: pays one body per possible source on a wildcard
    struct recorder {
        template<typename FROM_STATE, typename EVENT, typename TO_STATE, typename MACHINE>
        void onTransitionFrom(MACHINE&)
        {
            steps.push_back({mtl::short_name<FROM_STATE>(), mtl::short_name<EVENT>(),
                             mtl::short_name<TO_STATE>()});
        }
        std::vector<step> steps;
    };

    // the one-state form on top: on a wildcard the machine takes it, once,
    // and the recorder writes any_state for the source it did not ask for
    struct agnostic_recorder : recorder {
        template<typename EVENT, typename TO_STATE, typename MACHINE>
        void onTransition(MACHINE&)
        {
            steps.push_back({"any_state", mtl::short_name<EVENT>(), mtl::short_name<TO_STATE>()});
        }
    };
} // namespace transition_hook

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

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<closed>, fsm::on<reading>, fsm::to<open>,
                        fsm::guard<above_threshold>>> {};
} // namespace event_guard

void guardSeesTheEventPayload()
{
    using namespace event_guard;
    fsm::StateMachine<tbl> sm;

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
        int values() const { return value; } // one value: a one-element set
    };

    // observes the static mode and the instance value; the contract
    // guarantees the static hook runs first on the same entry
    struct dual_observer : fsm::observing<dual_observer> {
        template<typename STATE>
        static constexpr auto observe_static() -> decltype(STATE::mode)
        {
            return STATE::mode;
        }
        void notifyEntry(mode_t const&) { sequence.push_back('s'); }
        void notifyEntry(int value) { sequence.push_back('n'); last_value = value; }

        std::vector<char> sequence;
        int last_value = -1;
    };

    struct tbl : fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<active>>> {};
} // namespace ordering

void declaredObservationsAreValidated()
{
    using namespace AnnotationSets;
    heater watcher;
    fsm::StateMachine<tbl, heater> sm{watcher}; // validate(): heat is in the table

    check(watcher.heats == 0);  // dark carries no heat
    check(sm.process(next{}));  // lit: heat{true}
    check(watcher.heats == 1);
    check(sm.process(kill{}));  // dead: heat{false}
    check(watcher.heats == 2);
}

void activeStateAnnotationIsQueried()
{
    using namespace AnnotationSets;
    panel p;
    fsm::StateMachine<tbl, panel> sm{p};

    // the query reads the active state's static set: an element it
    // carries as the value, one it lacks as an empty optional
    check(sm.annotation<light>() == light::off);
    check(sm.annotation<level>() == level{1});
    check(!sm.annotation<heat>());
    check(sm.process(next{})); // lit
    check(sm.annotation<heat>() == heat{true});
    check(sm.process(next{})); // bare: no set at all
    check(!sm.annotation<light>() && !sm.annotation<heat>());
    check(sm.process(kill{})); // dead
    check(sm.annotation<heat>() == heat{false});
    // an element no state carries is refused at compile time
    // (sm.annotation<sound>() would static_assert)
}

void annotationSetElementsAreNotifiedIndependently()
{
    using namespace AnnotationSets;
    panel p;
    fsm::StateMachine<tbl, panel> sm{p};

    // construction: every consumed element of dark
    check(p.lights == std::vector<light>{light::off});
    check(p.levels == std::vector<int>{1});

    check(sm.process(next{})); // dark -> lit: light changes, level stays 1
    check(p.lights == std::vector<light>{light::off, light::on});
    check(p.levels == std::vector<int>{1});
    check(p.level_exits.empty());

    check(sm.process(next{})); // lit -> bare: bare has no set, both leave
    check(p.level_exits == std::vector<int>{1});
    check(p.lights.size() == 2);

    check(sm.process(next{})); // bare -> dark: both arrive again
    check(p.lights == std::vector<light>{light::off, light::on, light::off});
    check(p.levels == std::vector<int>{1, 1});

    check(sm.process(kill{})); // -> dead: heat only, nothing for the panel
    check(p.level_exits == std::vector<int>{1, 1});
    check(p.lights.size() == 3);
}

void staticHookRunsBeforeNonstaticHook()
{
    ordering::dual_observer observer;
    fsm::StateMachine<ordering::tbl, ordering::dual_observer> sm{observer};

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
    fsm::ObserverGroup<fsm::timed<manual_timer>, output_controller,
                        raw_hooks::transition_counter>
        group{tim, ctrl, counter};
    fsm::StateMachine<table, decltype(group)> sm{group}; // one reference, three observers

    check(counter.enters == 1 && counter.exits == 0);
    check(ctrl.log.size() == 1); // initial off outputs

    sm.process(button_press{}); // off -> running
    check(tim.timer().armed && tim.timer().duration == 50ms); // timed's validate/hooks forwarded
    check(counter.enters == 2 && counter.exits == 1);
    check(ctrl.log.back() == outputs_t{.led = true, .fan = true});

    check(!sm.process(lock_key{})); // ignored event: nothing forwarded
    check(counter.enters == 2 && counter.exits == 1);
}

void rawHookObserverSeesEveryTransition()
{
    raw_hooks::transition_counter counter;
    fsm::timed<manual_timer> tim;
    fsm::StateMachine<table, fsm::timed<manual_timer>, raw_hooks::transition_counter> sm{tim, counter};

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
    fsm::StateMachine<tbl> sm; // timeout in trying stays unobserved: no timer injected

    check(sm.process(start{.payload = 7}));
    auto const* log = &sm.getIf<trying>()->context;
    check(log->attempts == 1 && log->payload == 7);

    check(sm.process(fail{})); // re-entry: fresh state object, same context
    check(&sm.getIf<trying>()->context == log);
    check(log->attempts == 2 && log->payload == 7);

    check(sm.process(done{})); // succeeded names the same context type
    check(&sm.getIf<succeeded>()->context == log);
    check(log->attempts == 2);
    check(sm.context<history>().successes == 1); // its second context, another instance

    check(sm.process(restart{})); // context also outlives contextless states
    check(sm.process(start{.payload = 9}));
    check(log->attempts == 1 && log->payload == 9);
}

void contextSurvivesTimeoutRetry()
{
    using namespace Context;
    fsm::timed<manual_timer> tim;
    fsm::StateMachine<tbl, fsm::timed<manual_timer>> sm{tim};

    check(sm.process(start{.payload = 3}));
    check(tim.timer().armed);

    tim.timer().expire(); // the retry loses neither payload nor attempt count
    check(sm.is<trying>());
    check(sm.getIf<trying>()->context.attempts == 2);
    check(sm.getIf<trying>()->context.payload == 3);
    check(tim.timer().armed); // re-armed for the next attempt
}

void contextInitialState()
{
    using namespace Context;
    struct tbl2 : fsm::transition_table<
        fsm::initial<trying>,
        fsm::transition<fsm::from<trying>, fsm::on<done>, fsm::to<succeeded>>> {};
    fsm::StateMachine<tbl2> sm; // initial state constructed from its context

    check(sm.is<trying>());
    check(sm.getIf<trying>()->context.attempts == 1);
}

void internalTransitionHandlesInPlace()
{
    using namespace Internal;

    fsm::timed<manual_timer> tim;
    hook_counter hooks;
    fsm::StateMachine<tbl, fsm::timed<manual_timer>, hook_counter> sm{tim, hooks};

    check(sm.is<waiting>() && tim.timer().armed);
    auto const enters_before   = hooks.enters;
    auto const duration_before = tim.timer().duration;

    check(sm.process(note{.value = 7})); // handled in place
    check(sm.is<waiting>());
    check(sm.getIf<waiting>()->context.noted == 7);
    check(hooks.enters == enters_before && hooks.exits == 0); // no exit/entry ran
    check(tim.timer().armed && tim.timer().duration == duration_before); // timer untouched

    // the guarded internal alternative wins over the regular fallback
    check(sm.process(tick{}));
    check(sm.is<waiting>());

    // with the guard failing, the fallback transition fires
    check(sm.process(note{0})); // clears noted in place
    check(sm.process(tick{}));
    check(sm.is<done>());
    check(!tim.timer().armed);
}

void guardedAlternativesFirstPassWins()
{
    using namespace Alternatives;
    fsm::StateMachine<tbl> sm;

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

void sharedWildcardFiresLikePerSource()
{
    using namespace SharedWildcard;
    mode_watcher watcher;
    fsm::timed<manual_timer> tim;
    fsm::StateMachine<tbl, fsm::timed<manual_timer>, mode_watcher> sm{tim, watcher};

    check(watcher.notified == 1); // initial entry into a
    check(tim.timer().armed);       // a is timed

    check(sm.process(kill{7}));   // wildcard from a, delivered shared
    check(sm.is<dead>() && sm.getIf<dead>()->code == 7); // payload arrived
    check(!tim.timer().armed);      // the left state's timer was stopped
    check(watcher.notified == 1); // dead carries no mode annotation

    check(sm.process(kill{9}));   // dead has no exact pair: fires again
    check(sm.getIf<dead>()->code == 9);
    check(!sm.process(go{}));     // no transition at all still reports false
}

void sharedWildcardDeliversExitValues()
{
    using namespace SharedWildcard;
    exit_watcher watcher;
    fsm::StateMachine<tbl, exit_watcher> sm{watcher};

    check(sm.process(go{}));  // a -> b: equal annotations, exit suppressed
    check(watcher.exits == 0);
    check(sm.process(kill{3})); // wildcard from b: b's exit notified, the edge known there
    check(sm.is<dead>());
    check(watcher.exits == 1);
}

void wildcardEntryRenotifiesUnchangedValue()
{
    using namespace SharedWildcard;
    mode_watcher watcher;
    fsm::StateMachine<home_tbl, mode_watcher> sm{watcher};

    check(watcher.notified == 1); // initial entry into a
    check(sm.process(go{}));      // a -> b: equal values, the edge suppresses
    check(watcher.notified == 1);
    check(sm.process(reset{}));   // wildcard into a: no edge, a's value notified again
    check(sm.is<a>());
    check(watcher.notified == 2);
}

void refusedOwnGroupFallsThroughToWildcard()
{
    using namespace SharedWildcard;
    mode_watcher watcher;
    fsm::timed<manual_timer> tim;
    fsm::StateMachine<guarded_tbl, fsm::timed<manual_timer>, mode_watcher> sm{tim, watcher};

    check(sm.process(go{}));    // a -> b
    check(sm.process(kill{1})); // b's own pair refused: the wildcard is next
    check(sm.is<dead>() && sm.getIf<dead>()->code == 1);
}

void unguardedOwnEntryOverridesWildcard()
{
    using namespace Wildcard;
    edge_recorder edges;
    fsm::StateMachine<with_override, edge_recorder> sm{edges};

    check(edges.entries == 1);     // construction
    check(sm.process(advance{}));  // idle -> stage1
    check(sm.process(shutdown{})); // stage1's own pair always fires: not the wildcard
    check(sm.is<stage2>());
    check(sm.process(shutdown{})); // stage2 has no own pair: the wildcard
    check(sm.is<idle>());
    check(edges.entries == 4);     // the wildcard's entry delivered per source once
}

void deadlineSpansPhaseWithoutRearming()
{
    using namespace Deadline;
    manual_timer clock; // the deadline's own timer, next to fsm::timed's
    fsm::deadlined<manual_timer&> ded{clock};
    fsm::timed<manual_timer> tim;
    fsm::StateMachine<tbl, fsm::deadlined<manual_timer&>, fsm::timed<manual_timer>> sm{ded,
                                                                                       tim};

    check(clock.armed && clock.duration == 80ms); // armed on phase entry
    check(clock.starts == 1);

    check(sm.process(step{})); // searching -> probing: same value
    check(tim.timer().armed);    // the per-state timeout runs alongside
    check(sm.process(bounce{})); // ... and back: still the same phase
    check(sm.process(step{}));
    check(clock.starts == 1); // bouncing never re-armed the deadline

    clock.expire(); // the budget is up, wherever the phase stands
    check(sm.is<gave_up>());
    check(!tim.timer().armed); // probing's timeout stopped by the exit

    check(sm.process(retry{})); // gave_up -> searching: a fresh phase
    check(clock.starts == 2 && clock.duration == 80ms);
    check(sm.process(step{})); // -> probing
    check(sm.process(step{})); // -> arrived: the zero sentinel
    check(!clock.armed);       // target reached, clock stopped

    check(sm.process(retry{})); // arrived -> rearmed: a new value
    check(clock.starts == 3 && clock.duration == 30ms);
    clock.expire();
    check(sm.is<gave_up>());
}

void transitionHookSeesEdgeAndEvent()
{
    using namespace transition_hook;
    recorder rec;
    fsm::StateMachine<tbl, recorder> sm{rec};

    check(rec.steps.empty()); // construction is no transition

    check(sm.process(tick{})); // handled in place
    check(rec.steps.back() == step{"idle", "tick", "internal_target"});
    check(sm.getIf<idle>()->ticks == 1);

    check(sm.process(go{})); // default-constructed target
    check(rec.steps.back() == step{"idle", "go", "busy"});

    check(sm.process(kill{3})); // payload edge, wildcard: the real source
    check(rec.steps.back() == step{"busy", "kill", "dead"});
    check(sm.getIf<dead>()->code == 3);
    check(rec.steps.size() == 3);
}

void sourceAgnosticHookSeesAnyState()
{
    using namespace transition_hook;
    agnostic_recorder rec;
    fsm::StateMachine<tbl, agnostic_recorder> sm{rec};

    check(sm.process(go{}));
    check(rec.steps.back() == step{"idle", "go", "busy"}); // exact edges unchanged

    check(sm.process(kill{5})); // shared body: the source is any_state
    check(rec.steps.back() == step{"any_state", "kill", "dead"});
    check(rec.steps.size() == 2); // exactly one notification per firing
    check(sm.getIf<dead>()->code == 5);
}

void observerGroupForwardsTransitionHook()
{
    using namespace transition_hook;
    recorder rec;
    fsm::ObserverGroup<recorder> group{rec};
    fsm::StateMachine<tbl, fsm::ObserverGroup<recorder>> sm{group};

    check(sm.process(go{}));
    check(rec.steps == std::vector<step>{{"idle", "go", "busy"}});
    check(sm.process(kill{1})); // the member is not agnostic: the real source
    check(rec.steps.back() == step{"busy", "kill", "dead"});
}

// A group is source-agnostic when every member is
void observerGroupOfAgnosticMembersIsAgnostic()
{
    using namespace transition_hook;
    agnostic_recorder rec;
    fsm::ObserverGroup<agnostic_recorder> group{rec};
    fsm::StateMachine<tbl, fsm::ObserverGroup<agnostic_recorder>> sm{group};

    check(sm.process(kill{2}));
    check(rec.steps == std::vector<step>{{"any_state", "kill", "dead"}});
}

void timerInjectedByReference()
{
    manual_timer timer; // caller-owned policy instance
    fsm::timed<manual_timer&> tim{timer};
    output_controller ctrl;
    fsm::StateMachine<table, fsm::timed<manual_timer&>, output_controller> sm{tim, ctrl};

    check(sm.process(button_press{})); // off -> running, timeout armed
    check(timer.armed && timer.duration == 50ms);
    timer.expire();
    check(sm.is<cooldown>());
}

// The regression all of these guard: a hook driving the queue used to
// re-enter process() and trip the machine's assert - now it queues
void queuedDeliversAfterTransitionCompletes()
{
    manual_timer clock;
    fsm::QueuedTimer<manual_timer> channel{clock};
    Queued::queued_timed tim{channel};
    Queued::sync_actor actor;
    Queued::machine sm{tim, actor};

    actor.queue = &sm;
    actor.post  = [](void* queue) {
        static_cast<Queued::machine*>(queue)->process(Queued::halt{});
    };
    check(sm.process(Queued::go{})); // the armed entry hook processes halt
    check(sm.is<Queued::done>());    // ...delivered after go's transition completed
    check(sm.getIf<Queued::done>() != nullptr);
    check(actor.log == std::vector<char>{'d'});
}

void queuedOwningTimerIsOneLine()
{
    Queued::owning_timed tim; // owns the platform timer and the queued channel
    Queued::sync_actor actor;
    Queued::owning_machine sm{tim, actor};

    check(sm.process(Queued::go{}));
    check(sm.is<Queued::armed>());
    check(tim.timer().platformTimer().armed);
    tim.timer().platformTimer().expire(); // latches, the inline work drains
    check(sm.is<Queued::timed_out>());
    check(actor.log == std::vector<char>{'t'});
}

void queuedRunsOnCallerOwnedWork()
{
    Queued::counting_work work;
    Queued::owning_timed tim;
    Queued::sync_actor actor;
    Queued::shared_work_machine sm{work, tim, actor};

    check(work.submits == 0); // construction drains on its own
    check(sm.process(Queued::go{}));
    check(work.submits == 1);
    check(sm.is<Queued::armed>());
}

void queuedStaleTimeoutRetracted()
{
    manual_timer clock;
    fsm::QueuedTimer<manual_timer> channel{clock};
    Queued::queued_timed tim{channel};
    Queued::sync_actor actor;
    Queued::machine sm{tim, actor};

    actor.queue = &sm;
    actor.post  = [](void* queue) {
        static_cast<Queued::machine*>(queue)->process(Queued::halt{});
    };
    actor.clock = &clock; // the expiry latches behind the queued halt

    check(sm.process(Queued::go{}));
    // halt arrived first: it wins, leaving armed stops the timer, and the
    // stop retracts the latched expiry - no timeout fires on done
    check(sm.is<Queued::done>());
    check(actor.log == std::vector<char>{'d'});
}

void queuedTimeoutDeliveredInArrivalOrder()
{
    manual_timer clock;
    fsm::QueuedTimer<manual_timer> channel{clock};
    Queued::queued_timed tim{channel};
    Queued::sync_actor actor;
    Queued::machine sm{tim, actor};

    actor.queue = &sm;
    actor.post  = [](void* queue) {
        static_cast<Queued::machine*>(queue)->process(Queued::note{});
    };
    actor.clock = &clock;

    check(sm.process(Queued::go{}));
    // the note is internal - armed survives it, so the latched expiry
    // stays valid and is delivered right after the queued events
    check(sm.is<Queued::timed_out>());
    check(actor.log == (std::vector<char>{'n', 't'}));
}

void queuedDeadlineGatesQueuedEvents()
{
    manual_timer clock;
    fsm::QueuedTimer<manual_timer> channel{clock};
    QueuedDeadline::queued_deadlined ded{channel};
    QueuedDeadline::sync_actor actor;
    QueuedDeadline::machine sm{ded, actor};

    actor.queue = &sm;
    actor.post  = [](void* queue) {
        static_cast<QueuedDeadline::machine*>(queue)->process(QueuedDeadline::finish{});
    };
    actor.clock = &clock;

    check(sm.process(QueuedDeadline::advance{})); // continues the phase, hook fires
    // finish was queued before the expiry, but a deadline gates progress:
    // it is delivered first, and finish then lands in expired - ignored
    check(sm.is<QueuedDeadline::expired>());
}

// --- hierarchy ----------------------------------------------------------------

void nestedChildIsConstructedOnEntry()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};

    check(sm.is<idle>());
    check(sm.submachine<active>() == nullptr); // no composite active
    check(rec.log == (std::vector<std::string>{"enter outer_table:idle"}));

    check(sm.process(go{}));
    check(sm.is<active>());
    auto const* child = sm.submachine<active>();
    check(child != nullptr && child->is<low>());
    static_assert(std::decay_t<decltype(*child)>::depth == 1);
    static_assert(std::is_same_v<std::decay_t<decltype(*child)>::table, inner_table>);
    // the parent's entry and transition hooks first, then the child's
    // construction enters its initial state
    check(rec.log == (std::vector<std::string>{"enter outer_table:idle", "exit outer_table:idle",
                                               "enter outer_table:active",
                                               "transition outer_table:active",
                                               "enter inner_table:low"}));
}

void nestedChildHandlesEventFirst()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    rec.log.clear();

    check(sm.process(tick{})); // low -(tick)-> high inside the child
    check(sm.is<active>());
    check(sm.submachine<active>()->is<high>());
    check(rec.log == (std::vector<std::string>{"exit inner_table:low", "enter inner_table:high",
                                               "transition inner_table:high"}));
}

void nestedUnhandledEventBubblesUp()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    rec.log.clear();

    check(sm.process(stop{})); // the child has no row for stop: the parent's fires
    check(sm.is<done>());
    check(sm.submachine<active>() == nullptr);
    // innermost first: the child's active state is left, then the composite
    check(rec.log == (std::vector<std::string>{"exit inner_table:low", "exit outer_table:active",
                                               "enter outer_table:done",
                                               "transition outer_table:done"}));
}

void nestedRefusedChildFallsThroughToParent()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    sm.process(tick{}); // -> high, whose tick row is guarded by never

    check(sm.process(tick{})); // refused in the child: active -(tick)-> idle
    check(sm.is<idle>());
    check(sm.submachine<active>() == nullptr);
}

void nestedReentryRestartsChild()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    sm.process(tick{}); // child in high
    sm.process(stop{}); // done
    sm.process(go{});   // idle
    sm.process(go{});   // active again

    check(sm.submachine<active>()->is<low>()); // no history: the initial sub-state
}

void nestedInternalTransitionStaysInChild()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    sm.process(tick{}); // high handles inner_only in place
    rec.log.clear();

    check(sm.process(inner_only{}));
    check(sm.submachine<active>()->is<high>());
    check(rec.log == (std::vector<std::string>{"transition inner_table:internal_target"}));
    check(!sm.process(inner_only{}) == false); // still handled...
    sm.process(stop{});
    check(!sm.process(inner_only{})); // ...and ignored once no level has a row
}

void nestedLocalEventStaysAtItsLevel()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    sm.process(tick{}); // high: a timed sub-state with its own timeout row

    check(sm.process(fsm::timeout{})); // injected at the root: the root's row fires
    check(sm.is<done>());
}

void nestedWildcardLeavesComposite()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};
    sm.process(go{});
    sm.process(tick{});
    rec.log.clear();

    check(sm.process(reset{})); // any_state -(reset)-> idle, from inside the composite
    check(sm.is<idle>());
    check(sm.submachine<active>() == nullptr);
    check(rec.log == (std::vector<std::string>{"exit inner_table:high", "exit outer_table:active",
                                               "enter outer_table:idle",
                                               "transition outer_table:idle"}));
}

void nestedAnnotationsAreQueriedAndObservedPerLevel()
{
    using namespace Nested;
    recorder rec;
    level_watcher watcher;
    nested_machine sm{rec, watcher};

    check(sm.annotation<power>() == power{false});
    check(!sm.annotation<lamp>().has_value()); // idle nests nothing

    sm.process(go{}); // active carries the power, its child the lamp
    check(sm.annotation<power>() == power{true});
    check(sm.annotation<lamp>() == lamp{false});

    sm.process(tick{});
    check(sm.annotation<lamp>() == lamp{true});
    check(sm.annotation<power>() == power{true});

    // each level notified its own value once, in entry order
    check(watcher.powers == (std::vector<power>{{false}, {true}}));
    check(watcher.lamps == (std::vector<lamp>{{false}, {true}}));
    check(watcher.left_lamps == (std::vector<lamp>{{false}}));

    sm.process(stop{}); // leaving the composite leaves the lamp behind
    check(watcher.left_lamps == (std::vector<lamp>{{false}, {true}}));
    check(!sm.annotation<lamp>().has_value());
    check(!sm.annotation<power>().has_value()); // done carries no power
}

void nestedTimersArmOneSlotPerLevel()
{
    using namespace Nested;
    fsm::timed<manual_timer, 2> tim; // fsm::levels_v<outer_table> slots
    static_assert(decltype(tim)::levels == fsm::levels_v<outer_table>);
    recorder rec;
    fsm::StateMachine<outer_table, fsm::timed<manual_timer, 2>, recorder> sm{tim, rec};

    sm.process(go{}); // active is timed, its initial sub-state low is not
    check(tim.timer(0).armed && tim.timer(0).duration == 100ms);
    check(!tim.timer(1).armed);

    sm.process(tick{}); // high: the child's slot arms, the parent's runs on
    check(tim.timer(1).armed && tim.timer(1).duration == 10ms);
    check(tim.timer(0).armed && tim.timer(0).starts == 1);

    tim.timer(1).expire(); // the child's expiry is the child's: high -> low
    check(sm.is<active>() && sm.submachine<active>()->is<low>());
    check(!tim.timer(1).armed);
    check(tim.timer(0).armed && tim.timer(0).starts == 1);

    sm.process(tick{}); // high again, both slots armed
    tim.timer(0).expire(); // the parent's expiry leaves the composite
    check(sm.is<done>());
    check(!tim.timer(0).armed);
    check(!tim.timer(1).armed); // the child's slot was stopped on the way out
}

void nestedTimersInjectedByReferencePerLevel()
{
    using namespace Nested;
    manual_timer outer_clock;
    manual_timer inner_clock;
    fsm::timed<manual_timer&, 2> tim{outer_clock, inner_clock};
    fsm::StateMachine<outer_table, fsm::timed<manual_timer&, 2>> sm{tim};

    sm.process(go{});
    sm.process(tick{});
    check(outer_clock.armed && inner_clock.armed);
    inner_clock.expire();
    check(sm.submachine<active>()->is<low>());
    check(&tim.timer(1) == &inner_clock);
}

void nestedInheritedContextIsTheParentsInstance()
{
    using namespace Nested;
    inheriting_machine sm;
    sm.process(go{}); // session: waiting
    sm.process(go{}); // trying: probing, two levels below the line's owner

    check(sm.process(sense{5}));
    check(sm.context<port_line>().cc == 5); // the child wrote the root's instance
    auto const* probe = sm.submachine<session>()->submachine<trying>();
    check(probe != nullptr);
    check(&probe->context<port_line>() == &sm.context<port_line>()); // one instance
    check(probe->context<phase_budget>().tries == 1);

    sm.process(sense{7});
    check(sm.context<port_line>().cc == 7);
    check(probe->context<phase_budget>().tries == 2);
}

void nestedOwnContextIsFreshOnReentryInheritedOnePersists()
{
    using namespace Nested;
    inheriting_machine sm;
    sm.process(go{});
    sm.process(go{});
    sm.process(sense{5});

    sm.process(stop{}); // leaves the session: the probe's budget dies with it
    check(sm.is<resting>());
    check(sm.context<port_line>().cc == 5); // the root's line outlives the phase

    sm.process(go{});
    sm.process(go{});
    auto const* probe = sm.submachine<session>()->submachine<trying>();
    check(probe->context<phase_budget>().tries == 0); // fresh
    check(probe->context<port_line>().cc == 5);       // still the root's
}

void nestedFeatureDisabledAtEveryLevel()
{
    using namespace Nested;
    recorder rec;
    fsm::StateMachine<featured_table, recorder> sm{rec}; // nobody enables boost_feature
    static_assert(std::is_same_v<decltype(sm)::table, featured_table>); // the name stays

    check(!sm.process(push{})); // turbo is gone from the root
    check(!sm.process(tick{})); // and the optional engine with it
    sm.process(go{});
    check(sm.submachine<engine>()->is<calm>());
    check(!sm.process(push{})); // boost is gone from the child: no level handles push
    check(sm.process(tick{}));  // the child's featureless rows stay
    check(sm.submachine<engine>()->is<warm>());
}

void nestedFeatureEnabledByAnObserver()
{
    using namespace Nested;
    booster enabler;
    recorder rec;
    fsm::StateMachine<featured_table, booster, recorder> sm{enabler, rec};

    sm.process(go{});
    check(sm.process(push{}));
    check(sm.submachine<engine>()->is<boost>());
    sm.process(stop{});
    check(sm.process(tick{}));
    check(sm.is<optional_engine>());
    // the two composites share deep_table: the child belongs to the active one
    check(sm.submachine<optional_engine>() != nullptr &&
          sm.submachine<optional_engine>()->is<calm>());
    check(sm.submachine<engine>() == nullptr);
}

void machineFiltersItsOwnTable()
{
    using namespace Features;
    bystander nobody;
    swap_policy policy;
    fsm::StateMachine<full_table, bystander> without{nobody};
    fsm::StateMachine<full_table, swap_policy> with{policy};

    check(without.is<plain>());   // the initial<swapping> went with its feature
    check(with.is<swapping>());
}

static_assert(fsm::deadlined<manual_timer, 2>::levels == 2); // the same slots for deadlines

void queuedNestedExpiryReachesItsLevel()
{
    using namespace Nested;
    using timers = fsm::timed<fsm::OwningQueuedTimer<manual_timer>, 2>;
    using queued =
        fsm::QueuedMachine<outer_table, 4, fsm::inline_work, fsm::no_lock, timers, recorder>;
    // the ring takes every level's events
    static_assert(mtl::has_a_v<queued::queueable_events, inner_only>);
    static_assert(!mtl::has_a_v<queued::queueable_events, fsm::timeout>);

    timers tim;
    recorder rec;
    queued sm{tim, rec};

    check(sm.process(go{}));
    check(sm.process(tick{}));
    check(sm.submachine<active>()->is<high>());
    check(sm.process(inner_only{})); // a child-only event, through the queue
    check(rec.log.back() == "transition inner_table:internal_target");

    check(tim.timer(1).platformTimer().armed);
    tim.timer(1).platformTimer().expire(); // latches; the inline work drains to the child
    check(sm.is<active>() && sm.submachine<active>()->is<low>());

    sm.process(tick{});
    tim.timer(0).platformTimer().expire(); // the parent's channel: leaves the composite
    check(sm.is<done>());
    check(!tim.timer(1).platformTimer().armed);
}

void finalStateEndsTheMachine()
{
    using namespace Final;
    fsm::StateMachine<ending_table> sm;
    check(!sm.isFinished());

    check(sm.process(halt{})); // the wildcard applies to every other state
    check(sm.is<stopped>() && !sm.isFinished());
    check(sm.process(start{}));

    check(sm.process(start{}));
    check(sm.is<finished>() && sm.isFinished());
    check(!sm.process(halt{})); // nothing leaves a final state, not even the wildcard
    check(!sm.process(start{}));
    check(sm.is<finished>());
}

void emittedEventIsTakenByTheCompositeState()
{
    using namespace Final;
    interrupter guard;
    fsm::StateMachine<job_table, interrupter> sm{guard};
    sm.process(start{}); // attempt: trying
    check(sm.submachine<attempt>()->is<trying>());

    // halfway emits progress: the parent's guard is asked and refuses,
    // the submachine carries on
    check(sm.process(start{}));
    check(guard.asked == 1);
    check(sm.is<attempt>() && sm.submachine<attempt>()->is<halfway>());

    // handled in place: the state was not entered again, nothing is emitted
    check(sm.process(poke{}));
    check(guard.asked == 1);

    // succeeded is final and emits: the composite state is left
    check(sm.process(start{}));
    check(sm.is<done>());
}

void parentGuardEndsTheSubmachine()
{
    using namespace Final;
    interrupter guard{.stop = true};
    fsm::StateMachine<job_table, interrupter> sm{guard};
    sm.process(start{});

    check(sm.process(start{})); // halfway reports, the guard says stop
    check(guard.asked == 1);
    check(sm.is<interrupted>());
    check(sm.submachine<attempt>() == nullptr);
}

void silentFinalStateRestsUntilTheParentLeaves()
{
    using namespace Final;
    interrupter guard;
    fsm::StateMachine<job_table, interrupter> sm{guard};
    sm.process(start{});

    check(sm.process(fail{})); // given_up: final, emits nothing
    check(sm.is<attempt>());
    check(sm.submachine<attempt>()->isFinished());
    check(!sm.isFinished()); // the parent has not ended
    check(!sm.process(start{})); // a finished submachine handles nothing

    check(sm.process(retry{})); // the parent's own row: re-entry restarts the submachine
    check(sm.submachine<attempt>()->is<trying>());
    check(!sm.submachine<attempt>()->isFinished());
}

void subStateTimeoutEndsItsCompositeState()
{
    using namespace Final;
    fsm::timed<manual_timer, 2> tim;
    interrupter guard;
    fsm::StateMachine<job_table, fsm::timed<manual_timer, 2>, interrupter> sm{tim, guard};
    sm.process(start{});
    check(tim.timer(1).armed && tim.timer(1).duration == 10ms);

    // the expiry enters at the root, decorated for the level below:
    // trying -> failed, whose event the composite state takes
    tim.timer(1).expire();
    check(sm.is<broken>());
    check(!tim.timer(1).armed);
}

void endTakenOneLevelUpEndsThatLevelInTurn()
{
    using namespace Final;
    fsm::timed<manual_timer, 3> tim;
    fsm::StateMachine<war_table, fsm::timed<manual_timer, 3>> sm{tim};
    check(sm.is<war>());
    auto const* campaign = sm.submachine<war>();
    check(campaign->is<battle>() && campaign->submachine<battle>()->is<trying>());

    sm.process(start{}); // halfway: progress, handled in place one level up
    sm.process(start{}); // succeeded: the battle is fought again
    check(sm.submachine<war>()->submachine<battle>()->is<trying>());

    // two levels down: failed -> campaign_lost -> peace, in one run
    check(tim.timer(2).armed);
    tim.timer(2).expire();
    check(sm.is<peace>());
}

void queuedSubStateTimeoutEndsItsCompositeState()
{
    using namespace Final;
    queued_timers tim;
    interrupter guard;
    queued_job sm{tim, guard};
    check(sm.process(start{}));
    check(!sm.process(progress{})); // an emitted event is never processed from outside
    check(sm.is<attempt>() && !sm.isFinished());

    check(tim.timer(1).platformTimer().armed);
    tim.timer(1).platformTimer().expire(); // latches; the queue delivers it for level 1
    check(sm.is<broken>());
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
    entryIsConstructionExitIsDestruction();
    guardBlocksAndAllows();
    combinedGuardsAskEveryPart();
    injectedObjectAnswersGuard();
    staticGuardIsTheDefaultAnswer();
    rawHookObserverSeesEveryTransition();
    guardSeesTheEventPayload();
    activeStateAnnotationIsQueried();
    annotationSetElementsAreNotifiedIndependently();
    staticHookRunsBeforeNonstaticHook();
    observerGroupForwardsHooksInMemberOrder();
    contextIsMachineOwnedAndShared();
    contextSurvivesTimeoutRetry();
    contextInitialState();
    guardedAlternativesFirstPassWins();
    internalTransitionHandlesInPlace();
    timerInjectedByReference();
    transitionHookSeesEdgeAndEvent();
    sourceAgnosticHookSeesAnyState();
    observerGroupForwardsTransitionHook();
    observerGroupOfAgnosticMembersIsAgnostic();
    sharedWildcardFiresLikePerSource();
    sharedWildcardDeliversExitValues();
    wildcardEntryRenotifiesUnchangedValue();
    refusedOwnGroupFallsThroughToWildcard();
    unguardedOwnEntryOverridesWildcard();
    declaredObservationsAreValidated();
    deadlineSpansPhaseWithoutRearming();
    nestedChildIsConstructedOnEntry();
    nestedChildHandlesEventFirst();
    nestedUnhandledEventBubblesUp();
    nestedRefusedChildFallsThroughToParent();
    nestedReentryRestartsChild();
    nestedInternalTransitionStaysInChild();
    nestedLocalEventStaysAtItsLevel();
    nestedWildcardLeavesComposite();
    nestedAnnotationsAreQueriedAndObservedPerLevel();
    nestedTimersArmOneSlotPerLevel();
    nestedTimersInjectedByReferencePerLevel();
    nestedInheritedContextIsTheParentsInstance();
    nestedOwnContextIsFreshOnReentryInheritedOnePersists();
    nestedFeatureDisabledAtEveryLevel();
    nestedFeatureEnabledByAnObserver();
    machineFiltersItsOwnTable();
    queuedNestedExpiryReachesItsLevel();
    finalStateEndsTheMachine();
    emittedEventIsTakenByTheCompositeState();
    parentGuardEndsTheSubmachine();
    silentFinalStateRestsUntilTheParentLeaves();
    subStateTimeoutEndsItsCompositeState();
    endTakenOneLevelUpEndsThatLevelInTurn();
    queuedSubStateTimeoutEndsItsCompositeState();
    queuedDeliversAfterTransitionCompletes();
    queuedOwningTimerIsOneLine();
    queuedRunsOnCallerOwnedWork();
    queuedStaleTimeoutRetracted();
    queuedTimeoutDeliveredInArrivalOrder();
    queuedDeadlineGatesQueuedEvents();
    return failures;
}
