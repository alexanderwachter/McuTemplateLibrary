/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <print>
#include <source_location>
#include <type_traits>

// fsm::internal::TransitionGuards: who answers a table's guards, and
// what the answer is
namespace TransitionGuards {
    struct go {};
    struct stop {};
    struct idle {};
    struct busy {
        int load = 0;
    };

    struct permitted {}; // a pure tag: an injected object must answer
    struct light_load {  // answers itself, an injected answer overrides
        static bool check(busy const& state) { return state.load < 10; }
    };

    using starting = fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>,
                                     fsm::guard<permitted>>;
    using stopping = fsm::transition<fsm::from<busy>, fsm::on<stop>, fsm::to<idle>,
                                     fsm::guard<light_load, fsm::not_<permitted>>>;
    using resting  = fsm::transition<fsm::from<busy>, fsm::on<go>, fsm::to<idle>>;

    struct in_table : fsm::transition_table<starting, stopping, resting> {};
    struct self_answered : fsm::transition_table<
        fsm::transition<fsm::from<busy>, fsm::on<stop>, fsm::to<idle>, fsm::guard<light_load>>,
        resting> {};

    struct supervisor {
        bool permits = false;
        bool check(permitted) const { return permits; }
    };
    struct second_voice {
        bool check(permitted) const { return false; }
    };
    struct load_limit {
        bool check(light_load, busy const& state) const { return state.load < 100; }
    };
    struct bystander {};

    using with_supervisor = fsm::internal::TransitionGuards<bystander, supervisor>;
    using without         = fsm::internal::TransitionGuards<bystander>;
    using with_two_voices = fsm::internal::TransitionGuards<supervisor, second_voice>;

    static_assert(with_supervisor::every_guard_answered<in_table>);
    static_assert(!without::every_guard_answered<in_table>);
    static_assert(without::every_guard_answered<self_answered>); // by its static check
    static_assert(!with_two_voices::every_guard_answered<in_table>);

    static_assert(with_supervisor::no_guard_answered_by_two_observers<in_table>);
    static_assert(without::no_guard_answered_by_two_observers<in_table>);
    static_assert(!with_two_voices::no_guard_answered_by_two_observers<in_table>);
} // namespace TransitionGuards

namespace {

using namespace TransitionGuards;

int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        ++failures;
        std::print("FAILED: {}:{}\n", location.file_name(), location.line());
    }
}

void allowAsksEveryPart()
{
    bystander nobody;
    supervisor boss;
    fsm::internal::InjectedObservers<bystander, supervisor> observers{nobody, boss};

    check(with_supervisor::allow<resting>(observers, busy{}, go{})); // unguarded

    check(!with_supervisor::allow<starting>(observers, idle{}, go{})); // the injected answer
    boss.permits = true;
    check(with_supervisor::allow<starting>(observers, idle{}, go{}));

    check(!with_supervisor::allow<stopping>(observers, busy{}, stop{})); // not_<permitted>
    boss.permits = false;
    check(with_supervisor::allow<stopping>(observers, busy{}, stop{}));
    check(!with_supervisor::allow<stopping>(observers, busy{50}, stop{})); // the static check

    // an injected answer wins over the guard's own
    load_limit limit;
    fsm::internal::InjectedObservers<supervisor, load_limit> limited{boss, limit};
    using with_limit = fsm::internal::TransitionGuards<supervisor, load_limit>;
    check(with_limit::allow<stopping>(limited, busy{50}, stop{}));
    check(!with_limit::allow<stopping>(limited, busy{500}, stop{}));
}

} // namespace

int transitionGuardsTests()
{
    allowAsksEveryPart();
    return failures;
}
