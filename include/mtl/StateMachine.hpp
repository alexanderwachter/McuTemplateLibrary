/*
 * Template-based state machine on top of the mtl library.
 * SPDX-License-Identifier: Apache-2.0
 *
 * States are classes, constructed on entry and destroyed on exit: the
 * constructor and the destructor are the entry and exit hooks. Optional
 * members are detected by requires-expressions: static constexpr timeout,
 * and static constexpr members watched by observers - everything else a
 * state could do on an edge is an observer's job. The state set is derived
 * from the table;
 * an initial<STATE> table role picks the initial state (default: the first
 * state of the first transition), a final<STATE> role marks a state the
 * machine ends in (see "Final states" below).
 *
 * Events may carry payload: a target state constructible from the
 * triggering event is emplaced with it, any other target state is
 * default-constructed. Observer hooks run after the emplace and receive
 * the machine, so e.g. a driver observer can read the delivered payload
 * through machine.getIf<NEW_STATE>().
 *
 * fsm::internal_transition<from<S>, on<E>> handles E in S without a
 * state change: no exit/entry, no observer hooks, a running timeout
 * timer is untouched. The current state instance handles the event via
 * handle(E const&), typically updating its context. A guard applies as
 * usual and internal transitions group with regular ones as
 * alternatives; from<any_state> is not supported.
 *
 * States may keep data in machine-owned context that survives
 * transitions: a state declaring `using contexts = fsm::contexts<A,
 * B>;` is constructed with references to the matching instances, in
 * that order - (event, A&, B&) when such a constructor exists, (A&, B&)
 * otherwise, which every context state must provide - and keeps them
 * under member names of its own. The machine value-initializes one
 * instance per distinct context type; states naming the same type
 * share the instance, and its data persists across arbitrary
 * transitions for the machine's lifetime. Splitting data of different
 * lifetimes into separate context types lets a state reset one by
 * assignment without touching the other, and declare only what it
 * touches. Context types must be default constructible; context states
 * need no default constructor.
 *
 * A guard gates the transition it is attached to: a question the table
 * asks, answered with check(state, event), check(state), or check() -
 * most specific form wins - either by the guard type itself (static
 * check) or by an object injected into the machine like an observer,
 * with the guard as tag in front: check(GUARD, state, event). The table
 * names the question, not the answerer. An injected answer wins over a
 * static one; a question nobody answers, or two injected objects
 * answer, is a static_assert. guard<A, B, ...> asks every part in
 * order (short-circuit) and a part not_<G> holds when G does not, so
 * guards stay primitive and reusable; a disjunction is the next
 * alternative of the same pair. Transitions may share
 * a (state, event) pair when guards distinguish them: the alternatives
 * are tried in table order and the first whose guard passes fires; an
 * unguarded alternative is the catch-all and must be the last of its
 * group - an entry after it could never fire, so a second unguarded
 * entry for the pair (a plain duplicate included) is a static_assert.
 * When no alternative fires, process() returns false, no
 * exit/entry/hook runs, a running timeout timer keeps running. A
 * refused fsm::timeout would leave a timed state without its one-shot
 * timer, so fsm::timed's validate() requires an unguarded fsm::timeout
 * alternative for every timed state (fsm::deadlined likewise).
 *
 * from<any_state> matches every state and is the last alternative: a
 * state's own (state, event) group is tried first, in table order, then
 * the wildcard group. An unguarded own entry therefore overrides the
 * wildcard - that is how a state is exempted from one - while a guarded
 * own entry that refuses falls through to it.
 *
 * Hierarchy: a state declaring `using submachine = sub_table;` is a
 * composite state owning a machine of that table while it is active.
 * Entering it constructs the child after the state's own entry and
 * transition hooks (the child's initial state is entered by the
 * child's constructor, hooks included: a trace reads parent line, then
 * child line); leaving it leaves the child's active state first - its
 * exit hooks with TO = mtl::nil_type, innermost first - then runs the
 * state's own exit hooks. Events enter at the root and descend: the
 * active sub-state gets an event first, what it handles counts as
 * fired (process() returns true) without touching the parent, what it
 * does not - no row, or every guard refused - is tried against the
 * parent's own alternatives and then the wildcards. A local event
 * (fsm::is_local_event: fsm::timeout, fsm::deadline, own
 * specializations) belongs to one level and never descends from it.
 * A sub-state's expiry enters at the root too, decorated once per
 * level it lies below (fsm::for_level_t<EVENT, LEVEL>, one
 * fsm::for_submachine<> per level), and each machine passes it to its
 * submachine with one decoration less - no table names the decorated
 * type, so the machines on the way do not react to it. The timer
 * observers and the queued machine do the decorating.
 * There is no history: a transition to a composite state
 * restarts its child at the child's initial state. The child is a
 * real StateMachine with the same observers, its `table` the
 * sub-table and its `depth` one more than the parent's; the parent's
 * submachine<STATE>() is getIf() for the nested level, and
 * annotation<T>() answers from the one level of the active path that
 * carries T - an annotation type of a composite state may not recur
 * in its submachine (annotate at the level where the value changes).
 * Observers are validated once, at the root, with the whole hierarchy
 * in view (fsm::nested_tables_t, all_states_t walk it; fsm::levels_v
 * counts the levels).
 * Contexts are per machine: a sub-table's context types are the
 * child's own instances, fresh on every entry of the composite state
 * (the lifetime of a phase: a retry budget, a debounce record). A
 * composite state names the contexts of its own machine that its
 * child inherits:
 *   using parent_contexts = fsm::contexts<line_status>;
 * the sub-states declaring an inherited type then bind to the parent
 * machine's instance - the parent's lifetime - and an inherited type
 * the sub-table does not declare itself may be inherited further down
 * through one of its composites. The sub-states stay plain (`contexts`
 * is still their constructor signature), so the same state serves a
 * root table and a sub-table alike; only the composite, which knows it
 * nests, names what its child inherits. Checked at compile time: only
 * a composite declares parent_contexts, the parent machine holds every
 * named type (declared by a state of its table or inherited in turn),
 * and some state below declares it. context<T>() answers for an own
 * and an inherited context alike.
 *
 * Final states: a table entry fsm::final<STATE> - next to
 * fsm::initial<STATE>, one entry per state - marks a state the
 * machine ends in. Nothing leaves it: no transition names it as its
 * source, from<any_state> does not apply to it, it owns no submachine
 * and is not the initial state. isFinished() says whether the machine
 * rests in one. A finished root ignores every event from then on; a
 * finished submachine refuses them, so they are the composite state's,
 * and it is restarted when that state is entered again.
 *
 * Emitted events: a state of a submachine may declare
 *   using emits = attempt_failed;
 * and entering it hands that event - default-constructed - to the
 * machine above: the composite state's own alternatives, guards
 * included, then the wildcards; it is not offered back to the
 * submachine. The parent's table thus decides whether the composite
 * state is left - ending the submachine - or the submachine carries
 * on. The event is taken once, in the run that entered the state (an
 * internal transition in that state does not emit again), whatever
 * brought the submachine there - an event from outside or its own
 * timer. A final state that emits is the submachine's end with an
 * outcome; a state that only emits reports on the way. Checked at
 * compile time: the composite state has a transition for every event
 * its submachine emits, and a submachine's initial state emits none.
 * An emitted event is the submachine's own word: process() of a
 * queued machine does not accept it from outside
 * (fsm::emitted_events_t). At a root `emits` means nothing.
 *
 * Optional features: a state declaring `using feature = TAG;` belongs
 * to the feature TAG, and an observer declaring `using enables = TAG;`
 * (or an mtl::typelist of tags) switches it on - or the tag declares
 * `using enabled_by = GUARD;` and any injected object answering that
 * guard (check(GUARD)) switches it on: the feature's states ask the
 * question, whoever answers brings them in. fsm::feature_switch<
 * fsm::enabled<TAG, CONDITION>...> is an observer enabling tags by
 * compile-time conditions, for configuration symbols. The machine runs the
 * table minus every feature none of its injected observers enables -
 * the tagged states and every entry touching them, initial<> included
 * (then the next entry's source leads) - and a child machine filters
 * its submachine's table with the same observers, so a disabled
 * feature is gone at every level; the table keeps its name. A table
 * with nothing to remove is used as it is (fsm::enabled_table_t, also
 * what a test asks to see the table a machine runs). A guard on a
 * removed entry needs no answerer. Timer-range maps are filtered
 * alongside with fsm::remove_disabled_features_t.
 *
 * Timer policy contract (owned by fsm::timed<TIMER, LEVELS>):
 *   start(ms, fsm::timer_callback, void* context) arms a one-shot timer
 *   that invokes callback(context) once; restarting re-arms. stop()
 *   disarms and must tolerate an unarmed timer. The callback runs in the
 *   policy's execution context; process() is not re-entrant and not
 *   thread-safe - callback and process() must be serialized externally.
 *   The observer holds one timer per machine level (a composite state
 *   and its active sub-state may both be timed): LEVELS defaults to 1,
 *   fsm::levels_v<table> covers a hierarchy, timer(level) reads a slot.
 *   An expiry is the event for its slot's level and enters the machine
 *   at the root; an observer serving more than one level remembers the
 *   root for that when the root's initial state is entered.
 *
 * Observer contract: injected by reference, must outlive the machine.
 * Optional hooks, each detected by a requires-expression, run in observer
 * parameter order (place fsm::timed before value observers). Each hook
 * has a form of one state and a form of the edge; the observer's author
 * chooses - a hook of one state is instantiated once per state, a hook
 * of the edge once per edge, and on a wildcard once per possible source:
 *   template<typename STATE, typename MACHINE>
 *   void onExit(MACHINE&);            - STATE is being left, still alive
 *   void onEnter(MACHINE&);           - STATE was entered (constructed)
 *   template<typename EVENT, typename TO, typename MACHINE>
 *   void onTransition(MACHINE&);      - after the change completed; TO =
 *                                       fsm::internal_target for an internal
 *                                       transition
 *   template<typename FROM, typename TO, typename MACHINE>
 *   void onExitFrom(MACHINE&);        - the edge forms of the same three
 *   void onEnterFrom(MACHINE&);         hooks; on machine construction the
 *   template<typename FROM, typename EVENT, typename TO, typename MACHINE>
 *   void onTransitionFrom(MACHINE&);    initial state is entered once with
 *                                       FROM = mtl::nil_type; when a parent
 *                                       leaves a composite state, the
 *                                       child's active state is left once
 *                                       with TO = mtl::nil_type
 *   Where both forms exist the edge form is used when the edge is known.
 *   A from<any_state> transition fires through one shared body per
 *   (event, target): the exit hooks run where the state left is known,
 *   the change and the entry hooks once; an edge-form entry or transition
 *   hook is then delivered per possible source through a switch on the
 *   state left - the flash cost of asking for the edge
 *   template<typename TABLE> static constexpr void validate();
 *                                  - invoked at machine instantiation: the
 *                                    place for an observer's compile-time
 *                                    checks against the transition table
 * The machine itself imposes nothing on the table beyond its shape: a state
 * feature no injected observer consumes (a timeout without fsm::timed, an
 * annotation nobody watches) is silently unobserved.
 * Value observation with change suppression: see fsm::observing
 * (statemachine/Observing.hpp). A facade asks the active state's
 * annotation with machine.annotation<T>() - std::optional<T>, empty
 * while the active state carries no T - and reads the fact the states
 * declare instead of enumerating them with is<STATE>().
 *
 * This header is the whole library; the parts live in statemachine/:
 *   Transition.hpp  states, transition roles, transition types
 *   Table.hpp       transition_table and its lookups
 *   Timeout.hpp     timeout/deadline annotations, timer-range maps
 *   Timer.hpp       the timer policy contract, fsm::timed, fsm::deadlined
 *   Observing.hpp   annotation sets, fsm::observing
 *   ObserverHooks.hpp      the hook forms, their delivery to one observer
 *   ObserverGroup.hpp      several observers injected as one
 *   InjectedObservers.hpp  the observers of a machine: their references,
 *                          hook delivery, what they answer together
 *   Guards.hpp      the guards of a transition: who answers them,
 *                   whether they hold for an event
 *   Feature.hpp     features as tags and the filter removing one
 *   Traits.hpp      reachability, observer and annotation coverage,
 *                   event coverage - the table-wide proofs
 *   Core.hpp        the state machine and its dispatch
 *   Queued.hpp      fsm::QueuedMachine - run-to-completion delivery
 *                   through a bounded FIFO + WORK policy; QueuedTimer
 *                   latches expiries into the same serialized drain
 */

#pragma once

#include <mtl/statemachine/Contexts.hpp>
#include <mtl/statemachine/Core.hpp>
#include <mtl/statemachine/Feature.hpp>
#include <mtl/statemachine/Guards.hpp>
#include <mtl/statemachine/InjectedObservers.hpp>
#include <mtl/statemachine/ObserverGroup.hpp>
#include <mtl/statemachine/ObserverHooks.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Queued.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Timeout.hpp>
#include <mtl/statemachine/Timer.hpp>
#include <mtl/statemachine/Traits.hpp>
#include <mtl/statemachine/Transition.hpp>
