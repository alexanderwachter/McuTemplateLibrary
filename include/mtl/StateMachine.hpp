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
 * state of the first transition).
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
 * transitions: a state declaring a reference member named context is
 * constructed with a reference to the matching context instance -
 * (event, context) when such a constructor exists, (context) alone
 * otherwise, which every context state must provide. The machine
 * value-initializes one instance per distinct context type; states
 * naming the same type share the instance, and its data persists across
 * arbitrary transitions for the machine's lifetime. Context types must
 * be default constructible; context states need no default constructor.
 *
 * A guard gates the transition it is attached to; it provides
 * check(state, event), check(state), or check() - most specific form
 * wins. Transitions may share
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
 * Timer policy contract (owned by fsm::timed<TIMER>):
 *   start(ms, fsm::timer_callback, void* context) arms a one-shot timer
 *   that invokes callback(context) once; restarting re-arms. stop()
 *   disarms and must tolerate an unarmed timer. The callback runs in the
 *   policy's execution context; process() is not re-entrant and not
 *   thread-safe - callback and process() must be serialized externally.
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
 *                                       FROM = mtl::nil_type
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
 * (statemachine/Observing.hpp).
 *
 * This header is the whole library; the parts live in statemachine/:
 *   Transition.hpp  states, transition roles, transition types
 *   Table.hpp       transition_table, its lookups, guard evaluation
 *   Timeout.hpp     timeout/deadline annotations, timer-range maps
 *   Timer.hpp       the timer policy contract, fsm::timed, fsm::deadlined
 *   Observing.hpp   annotation sets, fsm::observing
 *   Observer.hpp    the hook forms, their delivery, observer_group
 *   Traits.hpp      reachability, features as tags, observer coverage
 *   Core.hpp        the state machine and its dispatch
 */

#pragma once

#include <mtl/statemachine/Core.hpp>
#include <mtl/statemachine/Observer.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Timeout.hpp>
#include <mtl/statemachine/Timer.hpp>
#include <mtl/statemachine/Traits.hpp>
#include <mtl/statemachine/Transition.hpp>
