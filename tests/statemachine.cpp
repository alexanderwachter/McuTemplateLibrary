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

namespace TimeoutBounds {
    static_assert(fsm::is_timeout_range<fsm::timeout_range>::value);
    static_assert(fsm::concepts::timeout_range<fsm::timeout_range>);
    static_assert(!fsm::concepts::timeout_range<std::chrono::milliseconds>);

    struct waiting { static constexpr auto timeout = std::chrono::milliseconds{150}; };

    using timed_table = fsm::transition_table<
        fsm::transition<fsm::from<off>,     fsm::on<button_press>, fsm::to<waiting>>,
        fsm::transition<fsm::from<waiting>, fsm::on<fsm::timeout>, fsm::to<off>>>;

    // an entry's range must contain the timeout, an exact duration must equal it
    inline constexpr fsm::timeout_range wait_range{std::chrono::milliseconds{100},
                                                   std::chrono::milliseconds{200}};
    inline constexpr auto exact_wait = std::chrono::milliseconds{150};

    using ranged_map = mtl::typelist<fsm::timed_by<waiting, wait_range>>;
    using exact_map  = mtl::typelist<fsm::timed_by<waiting, exact_wait>>;

    static_assert(fsm::timeout_within_bounds_v<ranged_map, waiting>);
    static_assert(fsm::timeouts_within_bounds_v<timed_table, ranged_map>);
    static_assert(fsm::timeouts_within_bounds_v<timed_table, exact_map>);

    // maps compose by concatenation, like the tables they describe
    static_assert(fsm::timeouts_within_bounds_v<
                  timed_table, mtl::concat_t<mtl::typelist<>, ranged_map>>);

    // rejected: a timeout outside the range, a duration that differs, a
    // timed state without an entry, and an entry for an untimed state
    inline constexpr fsm::timeout_range low_range{std::chrono::milliseconds{10},
                                                  std::chrono::milliseconds{20}};
    inline constexpr auto other_wait = std::chrono::milliseconds{100};
    static_assert(!fsm::timeout_within_bounds_v<mtl::typelist<fsm::timed_by<waiting, low_range>>,
                                                waiting>);
    static_assert(!fsm::timeout_within_bounds_v<mtl::typelist<fsm::timed_by<waiting, other_wait>>,
                                                waiting>);
    static_assert(!fsm::timeouts_within_bounds_v<timed_table, mtl::typelist<>>);
    static_assert(!fsm::timeouts_within_bounds_v<
                  timed_table, mtl::concat_t<ranged_map,
                                             mtl::typelist<fsm::timed_by<off, wait_range>>>>);
} // namespace TimeoutBounds

namespace Reachability {
    struct start {};
    struct step {};
    struct orphan {};
    struct end {};
    struct advance {};
    struct abort {};

    using linear = fsm::transition_table<
        fsm::transition<fsm::from<start>, fsm::on<advance>, fsm::to<step>>,
        fsm::transition<fsm::from<step>,  fsm::on<advance>, fsm::to<end>>>;
    static_assert(fsm::is_reachable_v<linear, end>);
    static_assert(fsm::all_states_reachable_v<linear>);

    // a state appearing only as a transition source is dead code
    using orphaned = fsm::transition_table<
        fsm::transition<fsm::from<start>,  fsm::on<advance>, fsm::to<end>>,
        fsm::transition<fsm::from<orphan>, fsm::on<advance>, fsm::to<end>>>;
    static_assert(!fsm::is_reachable_v<orphaned, orphan>);
    static_assert(!fsm::all_states_reachable_v<orphaned>);

    // a wildcard source leaves from every state; internal transitions
    // stay in place and reach nothing
    using through_wildcard = fsm::transition_table<
        fsm::transition<fsm::from<start>, fsm::on<advance>, fsm::to<step>>,
        fsm::internal_transition<fsm::from<step>, fsm::on<advance>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<abort>, fsm::to<end>>>;
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

    using mixed_table = fsm::transition_table<
        fsm::transition<fsm::from<annotated>, fsm::on<tick>, fsm::to<bare>>,
        fsm::transition<fsm::from<bare>,      fsm::on<tick>, fsm::to<annotated>>>;

    static_assert(!fsm::all_states_notified_v<level_watcher, mixed_table>);
    static_assert(fsm::all_states_notified_v<level_watcher, mixed_table, mtl::typelist<bare>>);
} // namespace AnnotationCoverage

namespace EventHandling {
    struct go {};
    struct halt {};
    struct ignored {};

    using table = fsm::transition_table<
        fsm::transition<fsm::from<off>, fsm::on<go>, fsm::to<running>>,
        fsm::internal_transition<fsm::from<running>, fsm::on<go>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<halt>, fsm::to<off>>>;

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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>,           fsm::on<advance>,  fsm::to<stage1>>,
        fsm::transition<fsm::from<stage1>,         fsm::on<advance>,  fsm::to<stage2>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<shutdown>, fsm::to<idle>>>;

    // any_state is not a state of the machine
    static_assert(std::is_same_v<tbl::states, mtl::typelist<idle, stage1, stage2>>);
    // the wildcard matches states without an exact (state, event) pair
    static_assert(std::is_same_v<fsm::transition_for_t<tbl, stage2, shutdown>::to, idle>);
    static_assert(std::is_same_v<fsm::transition_for_t<tbl, stage1, advance>::to, stage2>);

    // an exact pair takes precedence over the wildcard
    using with_override = fsm::transition_table<
        fsm::transition<fsm::from<idle>,           fsm::on<advance>,  fsm::to<stage1>>,
        fsm::transition<fsm::from<stage1>,         fsm::on<shutdown>, fsm::to<stage2>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<shutdown>, fsm::to<idle>>>;
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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<send>,   fsm::to<sending>>,
        fsm::transition<fsm::from<sending>, fsm::on<cancel>, fsm::to<idle>>>;

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
        using observes = mtl::typelist<message>;

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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<dark>, fsm::on<next>, fsm::to<lit>>,
        fsm::transition<fsm::from<lit>,  fsm::on<next>, fsm::to<bare>>,
        fsm::transition<fsm::from<bare>, fsm::on<next>, fsm::to<dark>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>>;

    // Whether an annotation type is carried by any state: the trait
    // behind fsm::observing's validate() for the types an observer
    // declares (an observer naming sound would not instantiate)
    struct sound {};
    static_assert(fsm::annotation_in_table_v<tbl, heat>);
    static_assert(!fsm::annotation_in_table_v<tbl, sound>);

    struct heater : fsm::observing<heater> {
        using observes = mtl::typelist<heat>;
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
    using ranges = mtl::typelist<fsm::timed_by<powering, any_time>, fsm::timed_by<plain, any_time>>;
    static_assert(std::is_same_v<fsm::remove_feature_t<ranges, vconn_feature>,
                                 mtl::typelist<fsm::timed_by<plain, any_time>>>);
    static_assert(std::is_same_v<fsm::remove_disabled_features_t<ranges, both_policies>, ranges>);
} // namespace Features

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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<a>, fsm::on<go>, fsm::to<b>>,
        fsm::transition<fsm::from<a>, fsm::on<fsm::timeout>, fsm::to<b>>,
        fsm::transition<fsm::from<b>, fsm::on<go>, fsm::to<a>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>>;

    struct never {
        static bool check(b const&) { return false; }
    };

    // a wildcard back into an annotated state: its entry has no edge to
    // compare against
    struct reset {};
    using home_tbl = fsm::transition_table<
        fsm::transition<fsm::from<a>, fsm::on<go>, fsm::to<b>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<reset>, fsm::to<a>>>;

    // b has an own pair for kill whose guard refuses: the wildcard is
    // the next alternative, exactly like transitions_for says
    using guarded_tbl = fsm::transition_table<
        fsm::transition<fsm::from<a>, fsm::on<go>, fsm::to<b>>,
        fsm::transition<fsm::from<a>, fsm::on<fsm::timeout>, fsm::to<b>>,
        fsm::transition<fsm::from<b>, fsm::on<go>, fsm::to<a>>,
        fsm::transition<fsm::from<b>, fsm::on<kill>, fsm::to<a>, fsm::guard<never>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>>;
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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<searching>, fsm::on<step>,          fsm::to<probing>>,
        fsm::transition<fsm::from<probing>,   fsm::on<bounce>,        fsm::to<searching>>,
        fsm::transition<fsm::from<probing>,   fsm::on<fsm::timeout>,  fsm::to<searching>>,
        fsm::transition<fsm::from<probing>,   fsm::on<step>,          fsm::to<arrived>>,
        fsm::transition<fsm::from<searching>, fsm::on<fsm::deadline>, fsm::to<gave_up>>,
        fsm::transition<fsm::from<probing>,   fsm::on<fsm::deadline>, fsm::to<gave_up>>,
        fsm::transition<fsm::from<arrived>,   fsm::on<retry>,         fsm::to<rearmed>>,
        fsm::transition<fsm::from<rearmed>,   fsm::on<fsm::deadline>, fsm::to<gave_up>>,
        fsm::transition<fsm::from<gave_up>,   fsm::on<retry>,         fsm::to<searching>>>;

    // the deadline bounds mirror of the timeout map checks
    inline constexpr fsm::timeout_range phase_range{70ms, 90ms};
    inline constexpr fsm::timeout_range short_range{20ms, 40ms};
    using ranges = mtl::typelist<fsm::timed_by<searching, phase_range>,
                                 fsm::timed_by<probing, phase_range>,
                                 fsm::timed_by<rearmed, short_range>>;
    static_assert(fsm::deadlines_within_bounds_v<tbl, ranges>);
    static_assert(fsm::deadline_within_bounds_v<ranges, searching>);
    // the zero sentinel and unannotated states must have no entry ...
    static_assert(fsm::deadline_within_bounds_v<ranges, arrived>);
    static_assert(!fsm::deadline_within_bounds_v<
                  mtl::typelist<fsm::timed_by<arrived, phase_range>>, arrived>);
    // ... and an active deadline outside its range fails
    static_assert(!fsm::deadline_within_bounds_v<
                  mtl::typelist<fsm::timed_by<rearmed, phase_range>>, rearmed>);
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

// --- entry is construction, exit is destruction -----------------------------
namespace lifetime {
    int entries = 0;
    int exits   = 0;

    struct ping {};
    struct plain { ~plain() { ++exits; } };      // counts its exits
    struct counted { counted() { ++entries; } }; // counts its entries

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<plain>,   fsm::on<ping>, fsm::to<counted>>,
        fsm::transition<fsm::from<counted>, fsm::on<ping>, fsm::to<plain>>>;
} // namespace lifetime

void entryIsConstructionExitIsDestruction()
{
    using namespace lifetime;
    fsm::state_machine<tbl> sm; // no timed states, no observers: nothing to inject
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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<gate>,   fsm::on<push>, fsm::to<passed>,
                        fsm::guard<gate_is_open>>,
        fsm::internal_transition<fsm::from<gate>, fsm::on<unlock>>,
        fsm::transition<fsm::from<passed>, fsm::on<push>, fsm::to<gate>,
                        fsm::guard<return_allowed>>>;
} // namespace guards

void guardBlocksAndAllows()
{
    using namespace guards;
    fsm::state_machine<tbl> sm;

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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>,
        fsm::internal_transition<fsm::from<idle>, fsm::on<tick>>,
        fsm::transition<fsm::from<busy>, fsm::on<go>, fsm::to<idle>>,
        fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>>;

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

    using tbl = fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<active>>>;
} // namespace ordering

void declaredObservationsAreValidated()
{
    using namespace AnnotationSets;
    heater watcher;
    fsm::state_machine<tbl, heater> sm{watcher}; // validate(): heat is in the table

    check(watcher.heats == 0);  // dark carries no heat
    check(sm.process(next{}));  // lit: heat{true}
    check(watcher.heats == 1);
    check(sm.process(kill{}));  // dead: heat{false}
    check(watcher.heats == 2);
}

void annotationSetElementsAreNotifiedIndependently()
{
    using namespace AnnotationSets;
    panel p;
    fsm::state_machine<tbl, panel> sm{p};

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
    check(sm.process(note{0})); // clears noted in place
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

void sharedWildcardFiresLikePerSource()
{
    using namespace SharedWildcard;
    mode_watcher watcher;
    fsm::timed<manual_timer> tim;
    fsm::state_machine<tbl, fsm::timed<manual_timer>, mode_watcher> sm{tim, watcher};

    check(watcher.notified == 1); // initial entry into a
    check(tim.timer.armed);       // a is timed

    check(sm.process(kill{7}));   // wildcard from a, delivered shared
    check(sm.is<dead>() && sm.getIf<dead>()->code == 7); // payload arrived
    check(!tim.timer.armed);      // the left state's timer was stopped
    check(watcher.notified == 1); // dead carries no mode annotation

    check(sm.process(kill{9}));   // dead has no exact pair: fires again
    check(sm.getIf<dead>()->code == 9);
    check(!sm.process(go{}));     // no transition at all still reports false
}

void sharedWildcardDeliversExitValues()
{
    using namespace SharedWildcard;
    exit_watcher watcher;
    fsm::state_machine<tbl, exit_watcher> sm{watcher};

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
    fsm::state_machine<home_tbl, mode_watcher> sm{watcher};

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
    fsm::state_machine<guarded_tbl, fsm::timed<manual_timer>, mode_watcher> sm{tim, watcher};

    check(sm.process(go{}));    // a -> b
    check(sm.process(kill{1})); // b's own pair refused: the wildcard is next
    check(sm.is<dead>() && sm.getIf<dead>()->code == 1);
}

void unguardedOwnEntryOverridesWildcard()
{
    using namespace Wildcard;
    edge_recorder edges;
    fsm::state_machine<with_override, edge_recorder> sm{edges};

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
    fsm::state_machine<tbl, fsm::deadlined<manual_timer&>, fsm::timed<manual_timer>> sm{ded,
                                                                                       tim};

    check(clock.armed && clock.duration == 80ms); // armed on phase entry
    check(clock.starts == 1);

    check(sm.process(step{})); // searching -> probing: same value
    check(tim.timer.armed);    // the per-state timeout runs alongside
    check(sm.process(bounce{})); // ... and back: still the same phase
    check(sm.process(step{}));
    check(clock.starts == 1); // bouncing never re-armed the deadline

    clock.expire(); // the budget is up, wherever the phase stands
    check(sm.is<gave_up>());
    check(!tim.timer.armed); // probing's timeout stopped by the exit

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
    fsm::state_machine<tbl, recorder> sm{rec};

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
    fsm::state_machine<tbl, agnostic_recorder> sm{rec};

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
    fsm::observer_group<recorder> group{rec};
    fsm::state_machine<tbl, fsm::observer_group<recorder>> sm{group};

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
    fsm::observer_group<agnostic_recorder> group{rec};
    fsm::state_machine<tbl, fsm::observer_group<agnostic_recorder>> sm{group};

    check(sm.process(kill{2}));
    check(rec.steps == std::vector<step>{{"any_state", "kill", "dead"}});
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
    check(tim.timer.platformTimer().armed);
    tim.timer.platformTimer().expire(); // latches, the inline work drains
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
    rawHookObserverSeesEveryTransition();
    guardSeesTheEventPayload();
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
    queuedDeliversAfterTransitionCompletes();
    queuedOwningTimerIsOneLine();
    queuedRunsOnCallerOwnedWork();
    queuedStaleTimeoutRetracted();
    queuedTimeoutDeliveredInArrivalOrder();
    queuedDeadlineGatesQueuedEvents();
    return failures;
}
