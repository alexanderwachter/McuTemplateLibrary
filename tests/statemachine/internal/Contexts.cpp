/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <type_traits>

// fsm::internal::MachineContexts: the contexts a machine owns and the
// ones it inherits from its parent, over three levels - the root owns
// the line, the session only inherits it, the probe inherits it next to
// its own budget
namespace MachineContexts {
    using fsm::internal::machine_contexts_t;
    using fsm::internal::nested;

    struct go {};
    struct stop {};
    struct sense {
        int level;
    };

    struct signal_line {
        int level = 0;
    };
    struct phase_budget {
        int tries = 0;
    };

    struct probing {
        using contexts = fsm::contexts<signal_line, phase_budget>;
        probing(signal_line&, phase_budget&) {}
    };
    struct probe_table : fsm::transition_table<
        fsm::transition<fsm::from<probing>, fsm::on<sense>, fsm::to<probing>>> {};

    // passes the line on, declaring no context of its own
    struct trying {
        using submachine      = probe_table;
        using parent_contexts = fsm::contexts<signal_line>;
    };
    struct waiting {};
    struct session_table : fsm::transition_table<
        fsm::transition<fsm::from<waiting>, fsm::on<go>, fsm::to<trying>>> {};

    struct resting {
        using contexts = fsm::contexts<signal_line>;
        explicit resting(signal_line&) {}
    };
    struct session {
        using submachine      = session_table;
        using parent_contexts = fsm::contexts<signal_line>;
    };
    struct inheriting_table : fsm::transition_table<
        fsm::transition<fsm::from<resting>, fsm::on<go>,   fsm::to<session>>,
        fsm::transition<fsm::from<session>, fsm::on<stop>, fsm::to<resting>>> {};

    // the declaration
    static_assert(fsm::internal::declares_parent_contexts<trying>);
    static_assert(!fsm::internal::declares_parent_contexts<waiting>);
    static_assert(std::is_same_v<fsm::internal::parent_contexts_t<trying>,
                                 mtl::typelist<signal_line>>);
    static_assert(std::is_same_v<fsm::internal::parent_contexts_t<waiting>, mtl::typelist<>>);

    using of_root    = machine_contexts_t<inheriting_table, inheriting_table>;
    using session_in = nested<session_table, 1, mtl::typelist<signal_line>>;
    using of_session = machine_contexts_t<session_in, session_in>;
    using probe_in   = nested<probe_table, 2, mtl::typelist<signal_line>>;
    using of_probe   = machine_contexts_t<probe_in, probe_in>;

    static_assert(std::is_same_v<of_root::own_contexts, mtl::typelist<signal_line>>);
    static_assert(std::is_same_v<of_root::inherited_contexts, mtl::typelist<>>);
    static_assert(std::is_same_v<of_session::own_contexts, mtl::typelist<>>);
    static_assert(std::is_same_v<of_session::inherited_contexts, mtl::typelist<signal_line>>);
    static_assert(std::is_same_v<of_probe::own_contexts, mtl::typelist<phase_budget>>);
    static_assert(std::is_same_v<of_probe::inherited_contexts, mtl::typelist<signal_line>>);
    static_assert(std::is_same_v<of_probe::context_types,
                                 mtl::typelist<phase_budget, signal_line>>);

    // the class is named by the two lists alone
    static_assert(std::is_same_v<of_probe, fsm::internal::MachineContexts<
                                               mtl::typelist<signal_line, phase_budget>,
                                               mtl::typelist<signal_line>>>);
    static_assert(std::is_same_v<of_root::inherited_by<session>, mtl::typelist<signal_line>>);

    // a child's is built from its parent's, a root's from nothing
    static_assert(std::is_default_constructible_v<of_root>);
    static_assert(!std::is_default_constructible_v<of_session>);
    static_assert(std::is_constructible_v<of_session, of_root&>);
    static_assert(std::is_constructible_v<of_probe, of_session&>);
    static_assert(!std::is_copy_constructible_v<of_root>);

    // stored by alignment, whatever the order of declaration
    struct flag {
        char set = 0;
    };
    struct mark {
        char set = 0;
    };
    struct count {
        long long value = 0;
    };
    static_assert(sizeof(fsm::internal::MachineContexts<mtl::typelist<flag, count, mark>,
                                                        mtl::typelist<>>) ==
                  2 * sizeof(long long));
    static_assert(sizeof(fsm::internal::MachineContexts<mtl::typelist<flag, count, mark>,
                                                        mtl::typelist<count>>) ==
                  2 * sizeof(void*));
    static_assert(fsm::internal::stored_alignment_v<flag&> == alignof(void*));
    static_assert(fsm::internal::stored_alignment_v<flag> == 1);

    struct no_default {
        explicit no_default(int) {}
    };
    static_assert(of_probe::own_contexts_default_constructible);
    static_assert(!fsm::internal::MachineContexts<mtl::typelist<no_default>, mtl::typelist<>>::
                      own_contexts_default_constructible);
    // inherited: the parent constructed it
    static_assert(fsm::internal::MachineContexts<mtl::typelist<no_default>,
                                                 mtl::typelist<no_default>>::
                      own_contexts_default_constructible);

    struct needs_more {
        using contexts = fsm::contexts<signal_line>;
        needs_more(signal_line&, int) {}
    };
    struct needy_table : fsm::transition_table<
        fsm::transition<fsm::from<resting>, fsm::on<go>, fsm::to<needs_more>>> {};
    static_assert(of_root::every_state_constructible_from_its_contexts<inheriting_table>);
    static_assert(!of_root::every_state_constructible_from_its_contexts<needy_table>);

    // parent_contexts belongs on a composite state
    struct plain_inheriting {
        using parent_contexts = fsm::contexts<signal_line>; // no submachine to inherit it
    };
    struct misplaced_table : fsm::transition_table<
        fsm::transition<fsm::from<resting>, fsm::on<go>, fsm::to<plain_inheriting>>> {};
    static_assert(fsm::internal::parent_contexts_on_composite<trying>::value);
    static_assert(fsm::internal::parent_contexts_on_composite<waiting>::value);
    static_assert(!fsm::internal::parent_contexts_on_composite<plain_inheriting>::value);
    static_assert(of_root::parent_contexts_only_on_composites<inheriting_table>);
    static_assert(!of_root::parent_contexts_only_on_composites<misplaced_table>);

    // the parent holds it: the root does, a machine of the session
    // table alone does not
    static_assert(
        fsm::internal::parent_contexts_held_in<mtl::typelist<signal_line>>::pred<trying>::value);
    static_assert(
        !fsm::internal::parent_contexts_held_in<mtl::typelist<phase_budget>>::pred<trying>::value);
    static_assert(of_root::holds_parent_contexts_of_every_composite<inheriting_table>);
    static_assert(of_session::holds_parent_contexts_of_every_composite<session_table>);
    static_assert(!machine_contexts_t<session_table, session_table>::
                      holds_parent_contexts_of_every_composite<session_table>);

    // some state of the submachine declares it
    struct contextless {};
    struct contextless_table : fsm::transition_table<
        fsm::transition<fsm::from<contextless>, fsm::on<go>, fsm::to<contextless>>> {};
    struct inheriting_unused {
        using submachine      = contextless_table;
        using parent_contexts = fsm::contexts<signal_line>;
    };
    struct unused_table : fsm::transition_table<
        fsm::transition<fsm::from<resting>, fsm::on<go>, fsm::to<inheriting_unused>>> {};
    static_assert(fsm::internal::parent_contexts_declared_in_submachine<trying>::value);
    static_assert(fsm::internal::parent_contexts_declared_in_submachine<session>::value); // two levels
    static_assert(!fsm::internal::parent_contexts_declared_in_submachine<inheriting_unused>::value);
    static_assert(of_root::every_submachine_declares_its_parent_contexts<inheriting_table>);
    static_assert(!of_root::every_submachine_declares_its_parent_contexts<unused_table>);
} // namespace MachineContexts
