/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

// fsm::internal::InjectedObservers: the questions a machine asks its
// observers as a set
namespace InjectedObservers {
    struct go {};
    struct tuning_feature {};
    struct allowed {};
    struct lamp {
        constexpr bool operator==(lamp const&) const = default;
    };

    struct idle {};
    struct lit {
        static constexpr auto annotations = fsm::annotate(lamp{});
    };
    struct tuning {
        using feature = tuning_feature;
    };

    struct in_table : fsm::transition_table<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<lit>, fsm::guard<allowed>>,
        fsm::transition<fsm::from<lit>, fsm::on<go>, fsm::to<tuning>>,
        fsm::transition<fsm::from<tuning>, fsm::on<go>, fsm::to<idle>>> {};

    struct tuner {
        using enables = tuning_feature;
        bool check(allowed) const { return true; }
    };
    struct lamp_driver : fsm::observing<lamp_driver> {
        void notifyEntry(lamp) {}
    };
    struct bystander {};

    using with_tuner = fsm::internal::InjectedObservers<bystander, tuner, lamp_driver>;
    using without     = fsm::internal::InjectedObservers<bystander>;

    static_assert(std::is_same_v<with_tuner::observer_list,
                                 mtl::typelist<bystander, tuner, lamp_driver>>);

    static_assert(with_tuner::any_observer_enables<tuning_feature>);
    static_assert(!without::any_observer_enables<tuning_feature>);
    static_assert(std::is_same_v<with_tuner::table_with_enabled_features<in_table>, in_table>);
    static_assert(!mtl::has_a_v<without::table_with_enabled_features<in_table>::states, tuning>);

    static_assert(with_tuner::any_observer_notified_by<lit>);
    static_assert(!with_tuner::any_observer_notified_by<idle>);
    static_assert(!without::any_observer_notified_by<lit>);

    static_assert(with_tuner::all_observers_validate<in_table>);
} // namespace InjectedObservers
