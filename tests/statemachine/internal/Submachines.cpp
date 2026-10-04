/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

// fsm::internal::Submachines: the child machines of a machine's
// composite states
namespace Submachines {
    using fsm::internal::nested;

    struct go {};
    struct tick {};

    struct power {
        bool on;
        constexpr bool operator==(power const&) const = default;
    };
    struct lamp {
        bool lit;
        constexpr bool operator==(lamp const&) const = default;
    };

    struct low {
        static constexpr auto annotations = fsm::annotate(lamp{false});
    };
    struct high {
        static constexpr auto annotations = fsm::annotate(lamp{true});
    };
    struct inner_table : fsm::transition_table<
        fsm::transition<fsm::from<low>,  fsm::on<tick>, fsm::to<high>>,
        fsm::transition<fsm::from<high>, fsm::on<tick>, fsm::to<low>>> {};

    struct idle {};
    struct active {
        using submachine = inner_table;
        static constexpr auto annotations = fsm::annotate(power{true});
    };
    struct wrapper {
        using submachine = inner_table;
    };
    struct outer_table : fsm::transition_table<
        fsm::transition<fsm::from<idle>,   fsm::on<go>, fsm::to<active>>,
        fsm::transition<fsm::from<active>, fsm::on<go>, fsm::to<idle>>> {};

    // a submachine whose final states tell the composite state how it ended
    struct attempt_succeeded {};
    struct attempt_failed {};

    struct trying {};
    struct succeeded {
        using emits = attempt_succeeded;
    };
    struct failed {
        using emits = attempt_failed;
    };
    struct attempt_table : fsm::transition_table<
        fsm::transition<fsm::from<trying>, fsm::on<go>,   fsm::to<succeeded>>,
        fsm::transition<fsm::from<trying>, fsm::on<tick>, fsm::to<failed>>,
        fsm::final<succeeded>,
        fsm::final<failed>> {};

    struct attempt {
        using submachine = attempt_table;
    };
    struct done {};
    struct broken {};
    struct job_table : fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<go>,                fsm::to<attempt>>,
        fsm::transition<fsm::from<attempt>, fsm::on<attempt_succeeded>, fsm::to<done>>,
        fsm::transition<fsm::from<attempt>, fsm::on<attempt_failed>,    fsm::to<broken>>> {};

    using inner_machine   = fsm::StateMachine<nested<inner_table, 1>>;
    using attempt_machine = fsm::StateMachine<nested<attempt_table, 1>>;

    using of_flat   = fsm::internal::Submachines<mtl::typelist<>, mtl::typelist<>>;
    using of_outer  = fsm::internal::Submachines<mtl::typelist<active>,
                                                 mtl::typelist<inner_machine>>;
    using of_job    = fsm::internal::Submachines<mtl::typelist<attempt>,
                                                 mtl::typelist<attempt_machine>>;
    using of_shared = fsm::internal::Submachines<mtl::typelist<active, wrapper>,
                                                 mtl::typelist<inner_machine, inner_machine>>;

    // the child of each composite; a flat table stores nothing, two
    // composites with the same child machine share one alternative
    static_assert(std::is_same_v<of_outer::child_of<active>, inner_machine>);
    static_assert(std::is_same_v<of_shared::child_of<wrapper>, inner_machine>);
    static_assert(of_outer::has_composite<active>);
    static_assert(!of_outer::has_composite<wrapper>);
    static_assert(std::is_empty_v<of_flat>);
    static_assert(sizeof(of_shared) == sizeof(of_outer));

    static_assert(of_outer::no_composite_nests_its_own_table<outer_table>);
    static_assert(!of_outer::no_composite_nests_its_own_table<inner_table>);

    // an annotation type lives on one level of a nesting path
    struct refining {
        using submachine = inner_table;
        static constexpr auto annotations = fsm::annotate(lamp{true}); // lamp is the sub-states'
    };
    using of_refining = fsm::internal::Submachines<mtl::typelist<refining>,
                                                   mtl::typelist<inner_machine>>;
    static_assert(fsm::internal::annotation_levels_exclusive<active>::value);
    static_assert(fsm::internal::annotation_levels_exclusive<idle>::value);
    static_assert(!fsm::internal::annotation_levels_exclusive<refining>::value);
    static_assert(of_outer::annotations_exclusive_per_level);
    static_assert(of_flat::annotations_exclusive_per_level);
    static_assert(!of_refining::annotations_exclusive_per_level);

    // an emitted event is default-constructed by the machine
    struct report {
        explicit report(int) {}
    };
    struct reporting {
        using emits = report;
    };
    struct reporting_table : fsm::transition_table<
        fsm::transition<fsm::from<trying>, fsm::on<go>, fsm::to<reporting>>> {};
    static_assert(of_flat::emitted_events_default_constructible<attempt_table>);
    static_assert(of_flat::emitted_events_default_constructible<job_table>); // emits none
    static_assert(!of_flat::emitted_events_default_constructible<reporting_table>);

    // a composite state owes a transition for every event its submachine emits
    struct deaf_table : fsm::transition_table<
        fsm::transition<fsm::from<idle>,    fsm::on<go>,             fsm::to<attempt>>,
        fsm::transition<fsm::from<attempt>, fsm::on<attempt_failed>, fsm::to<broken>>> {};
    static_assert(
        fsm::internal::emitted_events_taken_in<job_table, mtl::nil_type>::pred<attempt>::value);
    static_assert(
        !fsm::internal::emitted_events_taken_in<deaf_table, mtl::nil_type>::pred<attempt>::value);
    static_assert(of_job::every_composite_takes_emitted_events<job_table, mtl::nil_type>);
    static_assert(!of_job::every_composite_takes_emitted_events<deaf_table, mtl::nil_type>);
    static_assert(of_flat::every_composite_takes_emitted_events<deaf_table, mtl::nil_type>);

    // a submachine's initial state emits nothing
    struct loud_table : fsm::transition_table<
        fsm::transition<fsm::from<succeeded>, fsm::on<go>, fsm::to<trying>>> {};
    struct loud {
        using submachine = loud_table;
    };
    using of_loud = fsm::internal::Submachines<
        mtl::typelist<loud>, mtl::typelist<fsm::StateMachine<nested<loud_table, 1>>>>;
    static_assert(of_job::every_submachine_starts_silent<mtl::nil_type>);
    static_assert(!of_loud::every_submachine_starts_silent<mtl::nil_type>);
} // namespace Submachines
