# Project context: template-based state machine (fsm)

Repo and branch are WIP with no users yet: do not care about breaking
changes or backward compatibility - pick the best design and update all
call sites.

C++20 header-only state machine built on the mtl library in this repo
(`include/mtl`: `typelist`, `find_if`, `count_if`, `unique`, `all_of`, `front`).
Main files: `StateMachine.hpp` (the contract comment; includes the parts in
`statemachine/`: `Transition.hpp`, `Table.hpp`, `Timeout.hpp`, `Timer.hpp`,
`Observing.hpp`, `ObserverGroup.hpp`, `Feature.hpp`, `Lists.hpp`,
`Traits.hpp`, and - what only the machine itself uses, everything in
them `fsm::internal` - in `statemachine/internal/`: `ObserverHooks.hpp`,
`InjectedObservers.hpp`, `Guards.hpp`, `Contexts.hpp`, `Submachines.hpp`,
`Visit.hpp`, `Checks.hpp`; `statemachine/StateMachine.hpp` - the machine and
its dispatch, `Queued.hpp` - `fsm::QueuedMachine`, the queue-owning
wrapper that turns process() into an enqueue drained by a WORK policy
under a LOCK policy, with `QueuedTimer<TIMER>` (caller-owned timer) and
`OwningQueuedTimer<TIMER>` (default-constructible, one-line observer)
latching expiries), `StateMachineDot.hpp` (Graphviz
output, host tooling), `StateMachineTrace.hpp` (`fsm::tracing` observer,
target-suitable), `TypeName.hpp` (compile-time type names),
`tests/statemachine.cpp` (test suite with a `manual_timer` policy and an
`output_controller` observer), `tests/dot.cpp`, `tests/trace.cpp`,
`examples/statemachine/traffic_light.cpp` (runnable demo, also the host
feed for `tools/fsmview`), `tools/fsmview/` (Python live/replay viewer).

## Build & test
```
cmake -B build -G Ninja && cmake --build build && ./build/tests/TemplateMetaProgrammingTests
python3 -m unittest tools/fsmview/test_fsmview.py
clang++ -std=c++23 -Iinclude tests/*.cpp -o /tmp/mtl-clang-tests && /tmp/mtl-clang-tests
```
(clang is on the host, not in the flatpak sandbox: `flatpak-spawn --host
sh -c 'cd <repo> && ...'`; the host does not see the sandbox's /tmp.)
Tests follow the repo style: compile-time checks as `static_assert` in named
namespaces; runtime checks (state machine only) as small isolated test
functions using `check()`, reported through `main.cpp`.
Compile-time cost: `benchmarks/typelist_compile_time.py` (mechanical -
N generated types, one unit per typelist operation, any number of
include trees side by side; GGC MB from `-ftime-report` is the
deterministic figure, CPU seconds drift ±10% here). Results and the
design notes they settled are in `benchmarks/README.md`; a change to an
algorithm comes with its before/after row.
Follow the repo's naming conventions: ALL_CAPS template parameters, packs
ending in `s` (`TRANSITIONs`, `OBSERVERs`), `_t`/`_v` aliases, camelCase
member functions and hooks (`onEnter`, `onEnterFrom`, `notifyEntry`,
`getIf`), snake_case constexpr flags.

## Design decisions (do not regress)
- Transitions use named, order-independent wrappers:
  `fsm::transition<fsm::from<A>, fsm::on<E>, fsm::to<B>>`. Roles are extracted
  with `mtl::find_if`; `mtl::count_if` static_asserts exactly one of each.
  "Transition" is the only name for a table entry - no "row" terminology.
  `fsm::internal_transition<from<S>, on<E>>` handles E in S in place via
  `S::handle(E const&)`; its `to` alias is `fsm::internal_target`.
- Concepts are the types of metaprogramming (the author's rule,
  2026-10-02): every template parameter with a role is declared by its
  concept - public traits, machine members, observer hooks, internal
  traits and the predicates handed to mtl algorithms alike (a typed
  predicate binds to an untyped `template<typename> typename`
  parameter: host GCC 15.2, ARM GCC 12.2, clang 21.1). `fsm::concepts`:
  `state`, `event`, `guard`, `guard_part` (a guard or `not_<guard>`),
  `guard_for<G, STATE>` (precise guard contract), `transition`,
  `transition_role`, `transition_table_entry`, `entry_or_table`,
  `transition_table`, `observer`, `observer_list` (`fsm::observers<...>`
  or `mtl::nil_type`), `context`, `annotation`, `feature_tag`,
  `feature_condition`, `timer_range_entry`, `timed_by_or_timer_ranges`,
  `timer`. A refining concept names what it refines (`composite`,
  `emitting`, `featured`, `annotated`, `valued` start with
  `concepts::state<STATE> &&`) so a specialization on it is more
  constrained than a primary on `concepts::state`. `typename` stays
  where the language asks for it: a `concept` definition's own
  parameters; a hook's `MACHINE`/`ROOT` (probed while the machine class
  is incomplete - a constraint reading its members would silently hide
  the hook); parameters that take any type (`is_unnamed_table<T>`,
  `member_of<LIST>::pred<T>`, `annotation_set<Ts...>` whose elements may
  be references); and the context helpers in Transition.hpp that
  `concepts::state` is defined from. A wrong guard is a failed
  `concepts::guard_part` on `fsm::guard<...>` (was a static_assert in
  the transition).
- The lists an author writes have fsm names with the concept on their
  arguments: `fsm::contexts<...>` (contexts, parent_contexts),
  `fsm::annotations<...>` (an observer's `observes`), `fsm::events<...>`
  (what a state owes), `fsm::observers<...>` (the observer list of
  `enabled_table_t` / `feature_enabler_t`), `fsm::timer_ranges<...>` (a
  timer-range map; takes other maps in place of their entries). All are
  aliases of `mtl::typelist` and live together in `Lists.hpp` (public
  API, grouped 2026-10-04; they used to sit next to their concepts).
  No generic `fsm::list`: a list that is
  only a list stays `mtl::typelist`.
- Optional `guard<G>` role (`mtl::count_if` allows 0 or 1; `transition::guard`
  is `mtl::nil_type` when absent). G is a question, a default-constructible
  class: answered by its own static `check(FROM const&, EVENT const&)`,
  `check(FROM const&)` or `check()` (`fsm::concepts::guard_for`), or by
  an object injected into the machine with the tag in front,
  `check(G, FROM const&[, EVENT const&])` / `check(G)`
  (`concepts::answers_guard_for<OBJECT, G, FROM>`); the most specific
  form wins, an injected answer wins over the static one. The machine
  static_asserts every guard answered and by at most one injected
  object; `internal::TransitionGuards` (below) asks the two questions
  and resolves the answer per (guard, from-state). The table never names the answerer's
  type - tables stay Zephyr-free and mockable. Alternatives for one `(state, event)` pair
  are tried in table order, the first passing guard fires; an unguarded
  alternative must be the last of its group. A blocked transition: process()
  returns false, no exit/entry/hook runs, a timed state's timer keeps
  running. A refused `fsm::timeout` would strand a timed state without
  its one-shot timer, so `fsm::timed::validate` requires an unguarded
  `fsm::timeout` alternative (own or wildcard) for every timed state;
  `fsm::deadlined` likewise. Timeouts and deadlines must be positive.
  Debug checks (`MTL_FSM_CHECKS`, on without NDEBUG, `MTL_FSM_ASSERT`):
  process() re-entered from a hook, variant valueless after a throwing
  state constructor.
- The state set is derived from the table via `mtl::unique_t` over all
  `from`/`to` types; order of first appearance is preserved, so the first
  state of the first transition is the initial state (`mtl::front_t`). An
  optional `fsm::initial<STATE>` table role (0 or 1, anywhere in the pack,
  filtered out of `transitions` via `mtl::remove_if`) prepends STATE so it
  becomes the front/initial state; it must be a state of the table.
  The table a machine runs is a named struct (`struct my_table :
  fsm::transition_table<...> {}`), enforced by a static_assert in the
  machine (`internal::is_unnamed_table`, also for a `submachine`): the
  short type name is the machine id in trace lines and DOT graphs and
  stands for the table in every symbol. `StateMachine` exposes it as
  `table`. An unnamed `fsm::transition_table<...>` among a table's
  entries contributes its entries in place, to any depth
  (`internal::declared_entries` / `entries_of`; `transition_table::
  entries` is the flat list): shared transitions are written as unnamed
  tables, user code needs no `mtl::typelist`, `concat_t`, `linearize_t`
  or `rebind_t`. The unnamed table is matched by its arguments and never
  instantiated by itself, so it runs no table check of its own (test
  namespace `Composition`); a named struct is not accepted as an entry.
- States are classes, constructed on entry and destroyed on exit: the
  constructor and destructor are the entry and exit hooks (no
  `onEntry()`/`onExit()` members - anything else on an edge is an
  observer's job). Optional members detected by requires-expressions:
  `static constexpr timeout`, plus any `static constexpr` members
  watched by observers.
- The machine core knows nothing about timers or annotations. Its only
  extension point is observers: `StateMachine<table, OBSERVERs...>` with
  optional hooks, each in two forms the observer's author chooses
  between - of one state: `onExit<STATE>(machine&)` (STATE being left,
  still alive), `onEnter<STATE>(machine&)` (STATE constructed),
  `onTransition<EVENT, TO>(machine&)` (after the change completed, once
  per firing; `TO = fsm::internal_target` for internal transitions); of
  the edge: `onExitFrom<FROM, TO>`, `onEnterFrom<FROM, TO>`,
  `onTransitionFrom<FROM, EVENT, TO>` (on construction once with `FROM =
  mtl::nil_type`). Where both exist the edge form is used when the edge
  is known (`internal::exitHook`/`enterHook`/`transitionHook`). A
  one-state hook is instantiated once per state, an edge hook once per
  edge and, on a wildcard, once per possible source: flash for
  exactness. Plus an optional `static constexpr validate<TABLE>()`
  invoked at machine instantiation for observer-side compile-time table
  checks. Each hook is detected individually by a requires-expression
  (`internal::has_exit`, `has_exit_from`, ...); hooks run in observer
  pack order. Observers observe annotations, not states: a hook naming
  a state couples the observer to one table, an annotation type keeps
  them independent. The machine imposes nothing on the table itself: a state
  feature no injected observer consumes (timeout without `fsm::timed`,
  annotation nobody watches) is silently unobserved. The machine is
  non-copyable and non-movable: hooks hand out `*this`, and observers
  may retain the address beyond the hook (e.g. `fsm::timed` arms it into
  its timer callback). `ObserverGroup` forwards the edge forms to each
  member's preferred form and offers a one-state form only when every
  member has it.
- Wildcards: a `from<any_state>` transition changes the state through
  one shared body per (event, target) (`enterStateAfterWildcard`; expanding whole
  edges per source cost 2780 B on the firmware's `pd_drp`, 14 states).
  Its guards and exit hooks run in the visitor arm, where the state left
  is known (`leaveForWildcard`, which returns the chosen alternative as
  `pending + index`); after the dispatch `fireWildcard` changes the state
  once and runs the entry and transition hooks - one-state forms once,
  an edge form per possible source through `withSource`, a switch on
  the variant index `process` reads before the change and hands down
  as a local (no member). Nothing else knows the
  source: no `any_state` in hooks, no flags, no tables, no memory in
  observers. `fsm::timed` uses one-state hooks (leaving a timed state
  stops, entering one arms: one body per timed state); `fsm::observing`
  defines both forms, so exact edges and a wildcard's exit side suppress
  unchanged values at compile time while a wildcard's entry side, which
  has no edge, notifies every value of the state entered (test
  `wildcardEntryRenotifiesUnchangedValue`); `fsm::deadlined` and
  `fsm::tracing` use the edge forms (the phase continuation and the
  line's source are edge properties) and pay flash on wildcards.
  Measured with the firmware migrated to the two forms (its routers'
  hooks live): pd_drp 58728 B at 9efe0c9 against 58124 B at a464e23 -
  the redesign commit 46335a1 costs 852 B, the later commits win 248 B
  back. (Earlier numbers quoted here, 57364 B, were taken with the
  firmware's old-named router hooks silently undetected: not
  comparable.)
- Hierarchy: a state declaring `using submachine = sub_table;`
  (`internal::composite`, `submachine_t`) owns a `StateMachine<
  internal::nested<sub_table, depth + 1>, OBSERVERs...>` while active.
  The parent keeps the active child in one `std::variant<std::monostate,
  children...>` next to its state variant (`submachines_`, an
  `internal::Submachines`, below; empty in a flat table), constructs it
  in `enterSubmachine<STATE>` after the entry and
  transition hooks (constructor, `doTransition`, `enterStateAfterWildcard`), and
  tears it down first thing in `leaveState<OLD, NEW>` via the child's private
  `leaveActiveState()` - the active state left with `TO = nil_type`,
  innermost first (`Submachines` is a friend of the machine).
  A root's destructor still runs no hooks. Dispatch: public `process`
  is `processWithReaction(event) != reaction::none`;
  `processWithReaction` = `submachines_.react` (the active composite's
  child processes the event first) then `processInThisTable`
  (guarded, unguarded, wildcards). `internal::reaction {none, in_place,
  state_entered}` is the child's answer to its parent. Every event
  enters at the root, timer expiries included: `fsm::is_local_event`
  (`timeout`, `deadline`, user-specializable) belongs to one level and
  never descends from it, and a sub-state's expiry arrives decorated
  once per level (`fsm::for_submachine<EVENT>`, `fsm::for_level_t<EVENT,
  LEVEL>`); `internal::passedDown` strips one decoration per level, no
  table names the decorated type so the machines on the way ignore it.
  There is no parent link and no `processAt` (both tried 2026-10-01 and
  dropped: the link had the child destroyed under its own call, the
  level-addressed entry doubled the dispatch). `StateMachine::table` is
  the plain table (`plain_table_t`: trace names, DOT, validate see the
  user's type), `StateMachine::depth` the level. Observers' `validate`
  runs once at the root (`observersValidated`), the guard static_asserts
  per level; table-wide traits recurse via `fsm::nested_tables_t` /
  `all_states_t` / `nested_events_t` (`has_timed_states_v`,
  `annotation_in_table_v`, `all_states_notified`, the timed/deadlined
  validate), `fsm::levels_v<TABLE>` counts levels (flat = 1). Queries:
  `submachine<STATE>()` (getIf for the nested level), `annotation<T>()`
  answers from the one level of the active path carrying T -
  `internal::annotation_levels_exclusive` static_asserts that a
  composite's annotation type does not recur below it (a refining child
  would leave observers and query disagreeing). No history: entering a
  composite restarts the child at its initial state. Contexts are per
  machine - a child's own context is fresh on every entry - unless the
  child inherits them: `using parent_contexts = fsm::contexts<T...>;`
  on the composite (`internal::declares_parent_contexts`,
  `parent_contexts_t`; never on a sub-state, which must stay usable
  at a root). One word for it everywhere: the child *inherits*. The
  list rides on the nesting wrapper (`internal::nested<TABLE, DEPTH,
  INHERITED>`, `inherited_contexts_t`); the child's
  `inherited_contexts` are held as references behind its
  `own_contexts` (declared minus inherited; `context_types` = own ++
  inherited) in its `internal::MachineContexts`, and the parent
  constructs the child through the second constructor
  `StateMachine(MachineContexts<...>& parent_contexts,
  injected_observers const&)`: the child's contexts pick their
  references out of the parent's with `context<T>()`, own or inherited
  there, so a middle machine passes on what it inherited. Static checks per composite:
  `parent_contexts_on_composite`, `parent_contexts_held_in<
  context_types>` (the parent holds it),
  `parent_contexts_declared_in_submachine` (some state of the
  sub-table or the submachines inside it declares it). Tests:
  namespace `Nested`.
- Final states and emitted events (2026-10-01): `fsm::final<STATE>` is
  a table entry like `initial<>` (`internal::is_final_role`,
  `is_table_role`; `transition_table::final_states`,
  `fsm::final_states_t`). Nothing leaves a final state: table
  static_asserts (no transition from it, no submachine, not the
  initial state, a state of the table), `transitions_for` is empty for
  it and `wildcard_source_v` false - `from<any_state>` skips it.
  `isFinished()` on the machine (forwarded by `QueuedMachine`). The
  feature filter drops a `final<>` entry with its state
  (`internal::refers_to_state_where`, formerly `entry_touching`).
  Independent of that, any state of a submachine may declare `using
  emits = EVENT;` (`internal::emitting`, `emitted_t`,
  `fsm::emitted_events_t`): when a child's `processWithReaction` answers
  `state_entered`, `Submachines::reactToEmittedEvent<COMPOSITE>` finds
  the emitting state the child is in and runs the parent's `processInThisTable` with
  the default-constructed event - own rows with their guards, then
  wildcards, not offered back to the child. So the parent's table
  decides whether the composite is left; an internal transition in the
  emitting state does not emit again (`in_place`). Static checks in
  the machine: the composite has a transition for every emitted event
  (`emitted_events_taken_in`), the submachine's initial state emits
  none (`submachine_starts_silent`), the event is default
  constructible. The queued ring leaves emitted events out. Tests:
  namespace `Final`. DOT: `peripheries=2` on a final state, an
  "emits X" row.
- statemachine/StateMachine.hpp readability pass (2026-10-01, on the author's request:
  many comments = code not self-explaining): the table checks sit in
  one block at the end of the class, names say what a function does
  (`process*`, `doTransition`, `doFirstAllowedTransition`,
  `leaveStateForFirstAllowedWildcard`,
  `enterTargetStateOfWildcard`, `enterStateAfterWildcard`, `forStateLeft`, `guardsHold`,
  `partHolds`, `answerTo`). One measured limit: the visitor of the
  active state must answer with ONE value (a reaction, or
  `exited_for_wildcard + index`) - passing the wildcard index out
  through a captured local cost +432 B on pd_drp.
- Contexts are a class of their own (2026-10-04):
  `internal::MachineContexts<DECLARED_CONTEXTS, INHERITED_CONTEXTS>`
  (Contexts.hpp), the machine's `contexts_`, named by two lists of
  context types - never by the table, whose unnamed enabled form
  would be spelled out in every symbol; the type factory
  `internal::machine_contexts_t<TABLE, ENABLED_TABLE>` computes them
  (`table_contexts_t`, `inherited_contexts_t`), so statemachine/StateMachine.hpp names no
  context trait. The class owns the type computation (`own_contexts`,
  `inherited_contexts`, `context_types`, `inherited_by<COMPOSITE>` -
  the list on the child's `nested<>`), the instances (access is by type;
  the storage order is private: own instances and inherited
  references together, the most aligned first - `stored_contexts`,
  `stored_no_less_aligned`, a reference counting as a pointer - so no
  padding lies between them; a child's elements are constructed one by
  one, `element<ELEMENT>(parent)`. Measured on pd_drp: RAM equal at
  13008 B - its contexts had no holes to close - flash 62228 ->
  62152 B, 64 B below the state before the split: the contexts sit
  at other offsets and the `emplaceNewState` bodies reaching them come
  out 2-4 B smaller each), construction (default for a root,
  from the parent machine's `MachineContexts&` for a child; copying is
  deleted - a copy would take the parent's own values where a child
  must start fresh), access (`context<T>()`, `contextsOf<STATE>()` -
  a std::tie the machine applies to its emplace,
  `initialArgumentsOf<STATE>()` for make_from_tuple) and the
  questions under the machine's static_asserts:
  `own_contexts_default_constructible`,
  `every_state_constructible_from_its_contexts<TABLE>`,
  `parent_contexts_only_on_composites<TABLE>`,
  `holds_parent_contexts_of_every_composite<TABLE>`,
  `every_submachine_declares_its_parent_contexts<TABLE>`. The machine
  keeps `current_`, the emplace, `sub_` and the forwarding
  `context<T>()`; its public `own_contexts` / `inherited_contexts` /
  `context_types` aliases are gone (ask the class). Table.hpp knows
  contexts only as the third parameter of `nested`; all
  `parent_contexts` traits live in Contexts.hpp, Transition.hpp keeps
  what `concepts::state` is defined from (`contexts_of_t`,
  `context_constructible`). No friendship, no member of another
  class named. Measured on pd_drp: +12 B flash (62216 -> 62228 B),
  RAM equal (13008 B), main.cpp compile time equal (about 22 s) -
  `emplaceNewState` applies the `contextsOf` tuple and GCC inlines two
  edges differently. Tried: the machine expanding `contexts_of_t`
  over `contexts_.context<T>()` itself 0 B (rejected: leaves a
  context trait in statemachine/StateMachine.hpp), the class emplacing into the variant
  the machine hands it +36 B. Tests: namespace `MachineContexts`.
- Guards are a class of their own (2026-10-04):
  `internal::TransitionGuards<OBSERVERs...>` (Guards.hpp), the
  machine's `guards` alias - named by the observers only, never by
  the table, so it needs no type factory. It holds nothing: the
  machine hands in its `InjectedObservers&` where it passed `this`.
  Public: the questions under the machine's static_asserts
  (`every_guard_answered<TRANSITIONS>`,
  `no_guard_answered_by_two_observers<TRANSITIONS>`) and
  `allow<TRANSITION>(observers, state, event)` - true for an unguarded
  transition, else every part holds. Private: `everyPartHolds<GUARDS>`
  (one body per guard list), `partHolds` (`not_<guard>`), `answerTo`
  (the injected answer wins, else the static check),
  `any_observer_answers`, `askObserver` / `checkStatic` (the most
  specific form wins). statemachine/StateMachine.hpp names no guard trait and evaluates
  nothing: `doFirstAllowedTransition` and
  `leaveStateForFirstAllowedWildcard` ask
  `guards::allow`. Guards.hpp also holds the traits only answering
  needs (`answering`, `part_answered_in`, `part_answered_once_in`,
  `guard_answered_in`, `guard_answered_once_in`, from Table.hpp).
  Table.hpp keeps `has_guard_v` / `is_guarded` / `is_unguarded` (the
  order of alternatives, `no_shadowed_alternatives`, the timed
  validate, DOT); Transition.hpp keeps the role and what concepts are
  defined from (`guard`, `guard_part`, `guard_of_t`, `guards_t`,
  `guard_for` and the `answers_*` concepts - `answers_stateless_guard`
  is also Feature.hpp's `enabled_by`) and `is_negated`, the role's
  spelling DOT reads. No friendship, no member of another class named.
  Measured on pd_drp: 0 B - flash 62152 B, RAM 13008 B, no symbol
  changed size, main.cpp compile time equal (22.5 s). Tried, each also
  0 B with no symbol changed: `allow` keyed by the transition all the
  way down (no shared body per guard list), a one-reference view
  object `guards{observers_}.allow<TRANSITION>(state, event)` with
  member functions - GCC inlines all three alike; the static form
  stays as the one that cannot grow a member. Not built: the class
  holding a reference as a machine member (4 B RAM per level for
  nothing). Tests: namespace `TransitionGuards`.
- Submachines are a class of their own (2026-10-04):
  `internal::Submachines<COMPOSITES, CHILD_MACHINES>`
  (Submachines.hpp), the machine's `submachines_` - named by two
  parallel lists, the composite states of the enabled table and the
  child machine of each, never by the table. Only the machine can
  spell a child machine, so the type factory stays in statemachine/StateMachine.hpp
  (`child_machine_of`, `composites`, `child_machines`, `submachines`).
  Unlike the observers, contexts and guards classes it IS a friend of
  the machine (the author's decision: the no-friend rule came from
  those): it calls a child's private `processWithReaction()` and
  `leaveActiveState()` and the parent's `processInThisTable()`; the
  machines no longer befriend each other. statemachine/StateMachine.hpp asks it three
  things: `react(parent, event)` (the active composite's child first;
  a local event does not descend, `passedDown` strips a decoration;
  private `reactInActiveChild`, `reactInChildOf`,
  `reactToEmittedEvent`), `enterWith<STATE>(contexts, observers)` and
  `leaveWith<STATE>()` (no-ops for a plain state; `activeChildOf` holds
  the `MTL_FSM_CHECKS` assert). Queries: `childOf<COMPOSITE>()` behind
  the public `submachine<>()` (now returning `auto const*`),
  `annotationOfActiveChild<T>(parent, optional&)`. Storage: one variant
  alternative per distinct child machine, `nil_type` without
  composites (the class is empty). Questions under the machine's
  static_asserts: `no_composite_nests_its_own_table<TABLE>`,
  `annotations_exclusive_per_level`,
  `emitted_events_default_constructible<TABLE>`,
  `every_composite_takes_emitted_events<TABLE, OBSERVER_LIST>`,
  `every_submachine_starts_silent<OBSERVER_LIST>`. Moved here:
  `reaction` (from statemachine/StateMachine.hpp), `emitted_events_taken_in`,
  `submachine_starts_silent`, `submachine_emitting_states_t` (from
  Table.hpp), `nesting_carrier`, `annotation_levels_exclusive` (from
  Observing.hpp); the debug-check macros went to Checks.hpp, which
  both headers include. Everything a concept or public trait is
  defined from stays in Table.hpp (`composite`, `submachine_t`,
  `emitting`, `emitted_t`, `nested`, `levels`, `nested_tables`).
  Measured: pd_drp runs `drp_preference::none` and has NO composite
  state - 0 B there (62152 B, no symbol changed) proves nothing; the
  hot path is a copy of pd_drp with `drp_preference::sink` (Try.SNK,
  TryWait.SRC: composites, emitted events, inherited contexts),
  baseline 64124 B flash / 13072 B RAM. Final: 64116 B (-8 B, RAM
  equal), sensor sample on nucleo_g474re 33144 B (0 B), main.cpp
  compile time equal (24-25 s both, within drift). The machine keeps
  a one-line `enterSubmachine<STATE>()` handing in `contexts_` and
  `observers_`: with the three call sites calling `enterWith`
  directly GCC inlines the emplace into each, +68 B (64192 B).
  Tried on top of that +68 B state: the react functions static,
  taking only the parent and reaching `parent.submachines_` +80 B;
  the emitted event staying in the machine (per-composite fold and
  `reactToEmittedEvent` in statemachine/StateMachine.hpp, the class only handing out the
  child) +80 B - the cost was on the enter side in all three. Not
  built: a callable for the emitted event (the parent reference is
  the same thing), the class answering an index into the emitted
  events. Tests: namespace `Submachines`.
- Timeouts are an observer concern: `fsm::timed<TIMER, LEVELS = 1>` owns
  injected timer policies (`fsm::concepts::timer`, `start(ms,
  fsm::timer_callback, void*)` / `stop()`), one-shot, one per machine
  level (`internal::timer_slots`, `timer(level)`; the reference form
  `timed<TIMER&, N>` takes N caller-owned timers), armed in its
  `onEnter<STATE>` hook on slot `MACHINE::depth`, stopped in
  `onExit<STATE>` when a timed state is left. The expiry is
  `fsm::timeout` for the slot's level and enters at the root
  (`internal::processForLevel<EVENT, LEVELS>(root, level)` maps the
  run-time slot to the decorated type): a one-level observer calls the
  machine directly, one serving more levels remembers the root when
  the root's initial state is entered (`timer_slots::rememberRoot`,
  two pointers, `remembers_root`), and a timer policy declaring
  `static constexpr bool delivered_by_owner = true` (`QueuedTimer`) is
  armed with a null callback - its queue delivers. Its
  `validate<TABLE>()` static_asserts `LEVELS >= levels_v<TABLE>` and that
  every timed state of every level has an unguarded `fsm::timeout`
  transition in its own table. Place it before value observers so timers
  are armed before they are notified. `process()` is not re-entrant;
  serialization is the policy user's job. `QueuedTimerBase` is only
  the latch (`takeExpiry()`; the callback/context it used to keep are
  gone, -8 B per channel): the drain's `deliverAny<EVENT>` takes the
  first pending channel, outer level first, and calls
  `processForLevel` on the root with the channel's index;
  `QueuedMachine`'s ring takes `nested_events_t` minus the timer
  events and the emitted events.
- Observers are injected BY REFERENCE (`std::tuple<OBSERVERs&...>`) and
  must outlive the machine. The tuple lives in
  `internal::InjectedObservers<OBSERVERs...>` (InjectedObservers.hpp),
  the machine's `observers_`, copied into every child machine. The
  class owns everything about the observers as a set: hook delivery to
  the observers with a hook for the edge (`deliverExitHooks<OLD, NEW>`,
  `deliverEnterHooks`, `deliverTransitionHooks`, and the
  `...AfterWildcard` pair with the per-source compare chain), and the
  questions `table_with_enabled_features<TABLE>`,
  `any_observer_enables<TAG>`, `any_observer_notified_by<STATE>`,
  `all_observers_validate<TABLE>`; `observer<OBSERVER>()` hands out
  one reference by type (what `TransitionGuards` asks the answering
  observer through - the class has no guard member any more).
  The class knows the machine only through its public interface: no
  friendship, no member names; a wildcard's hooks read
  `StateMachine::enabled_table` (public: the table the machine runs,
  disabled features removed). Measured on pd_drp: this split costs
  +100 B flash (62116 -> 62216 B, RAM equal) - every delivery passes
  the observers and the machine, two arguments where the machine's own
  member functions passed one. Tried: static delivery reaching
  `machine.observers_` as a friend 0 B (rejected: couples the class to
  a private member of another), scoped lambdas over the index sequence
  in place of the named `...To` helpers +640 B, `observers_` as first
  member and `always_inline` wrappers no change.
  `ObserverGroup` keeps its own member delivery. Value observation lives in the
  `fsm::observing<DERIVED>` CRTP base: derived provides
  `observe_static<STATE>() -> decltype(STATE::member)` (trailing return
  type = SFINAE opt-out for states without the member) plus
  `notifyEntry(value)` (new state's value, on entry) and/or
  `notifyExit(value)` (old state's value, on exit, old state still
  alive) - each detected by requires. The base implements both hooks
  with the change check as `if constexpr` on constexpr values:
  equal-value transitions emit no code. First `notifyEntry` fires
  during machine construction; `notifyExit` never does. The hooks exist
  only for states whose observation reaches a `notifyEntry`/`notifyExit`
  overload (`observing::exits_notified<STATE>` /
  `entries_notified<STATE>`, 2026-10-03): a state the observer ignores
  costs the machine no function on any of its edges. Both forms go
  together per state - hiding only the edge form would make the
  machine fall back to the one-state form and re-notify an unchanged
  value (tried, caught by `wildcardEntryRenotifiesUnchangedValue`'s
  neighbours).
  Annotation sets: a state's `static constexpr auto annotations =
  fsm::annotate(a, b)` (`fsm::annotation_set<Ts...>`, types distinct,
  `has<T>`/`get<T>()` keyed by the plain type) is observed without any
  observe declaration - each element type with a `notifyEntry`/`notifyExit`
  overload is delivered, change-suppressed per element
  (`internal::set_annotation_changes`), in set order after the static
  hook. Instance values are the same thing per instance: a state's
  `auto values() const { return fsm::annotate_ref(msg, report()); }`
  (`annotate_ref` keeps lvalues by reference; a single value counts as
  a one-element set - `internal::instanceValues`, `instance_types_t`,
  `has_value_v`) is delivered on every edge, after the static set, no
  suppression. There is no `observe_nonstatic` any more. The coverage
  traits (`is_observed_v`, `is_notified_of_v`) include set elements and
  instance values.
  `observing::validate<TABLE>()` static_asserts that every type in an
  optional `using observes = fsm::annotations<...>;` is carried by a state
  (`annotation_in_table_v`); no declaration, no check - the firmware
  hands the same observers to every machine it owns, so an automatic
  "notified of at least one state" rule is wrong there (tried, dropped).
  Traits touching `STATE::annotations` must
  stay lazy (`has_annotation` is a specialization on the `annotated`
  concept - a plain `&&` substitutes both operands for `nil_type`).
- Tracing is an observer concern too: `fsm::tracing<DERIVED>`
  (`StateMachineTrace.hpp`) turns `onTransitionFrom` and the
  construction-only `onEnterFrom` into optional sinks `traceInitial(machine, state)` /
  `traceTransition(machine, from, event, to)` with static-storage C strings
  (`mtl::short_name_of`), suitable for deferred loggers. Line grammar (the
  contract with `tools/fsmview`; `fsm::trace_format::*` spells it for
  `std::format` and printf): `fsm[<machine>] -> <state>` on construction,
  `fsm[<machine>] <from> -(<event>)-> <to>` per transition, names verbatim
  (`internal_target` included; a wildcard names its real source - the
  tool still resolves `any_state` for older logs).
- Features as tags (Feature.hpp): a state declares `using feature =
  TAG;`, an observer `using enables = TAG;` (or an `mtl::typelist` of
  tags) - or the tag itself declares `using enabled_by = GUARD;` and
  an injected object answering `check(GUARD)` enables it
  (`observer_answers_for_v`; the firmware's swap features are enabled
  by the policy answering `pr_swap_allowed` etc., nothing else
  declared). `fsm::feature_switch<fsm::enabled<TAG, bool>...>` is an
  observer enabling tags by conditions (`enabled_features_t` is its
  list) - for config symbols and the firmware's compliance variants.
  `fsm::observer_enables_v`, `state_in_feature_v`,
  `feature_enabled_v<TAG, OBSERVERs...>`; `remove_features_t<LIST,
  typelist<TAGs...>>` / `remove_feature_t` remove the tagged states'
  entries in one pass - transitions from/to, `initial<>` (the next
  entry's source leads), `timed_by<>` map entries - and
  `remove_disabled_features_t<LIST, OBSERVERs...>` removes every
  feature no observer enables. The machine filters its own table: its
  `TRANSITIONS` is `fsm::enabled_table_t<TABLE, typelist<OBSERVERs...>>`
  - TABLE itself while nothing is disabled (lazy: `internal::
  has_disabled_features_v` over `transition_table::entries`, now
  public; `mtl::nil_type` as the list disables nothing), else the
  table rebuilt from the entries without them; `StateMachine::table`
  stays the user's type for names. A child machine filters its
  sub-table with the same observers, so a disabled feature is gone at
  every level; a sub-table emptied this way is a static_assert (tag
  the composite instead). The hierarchy traits take the observer list
  as a second parameter (`nested_tables_t<TABLE, OBSERVER_LIST>`,
  `all_states_t`, `nested_events_t` - the queued ring uses it; default
  `nil_type` = every state in view, what validate hooks see). Tables
  need no observer template any more (the sensor sample's
  `sensor_table` is plain; `mtl::zephyr::table_for` stays for tables
  that are). Timer-range maps are still filtered by hand with
  `remove_disabled_features_t`. Origin: the USB-C firmware's `pe::`
  machinery. Tests: namespaces `Features` and `Nested` in
  tests/statemachine.cpp. `fsm::feature_enabler_t<TAG, OBSERVER_LIST>`
  is the first observer enabling TAG either way (nil_type: none) -
  how a facade finds the feature's voice.
- Table-wide proofs a user asks (Traits.hpp), all walking the
  sub-tables: `all_states_notified<OBSERVER, TABLE, EXCEPTIONS>`,
  `all_states_carry<TABLE, T, EXCEPTIONS>` (the annotation T is in
  force on every active path - one element at a time, a probe observer
  per element was the firmware's workaround; over a hierarchy
  (`internal::carried_throughout`, 2026-10-01) a composite carrying T
  covers its whole submachine and a composite without T is covered by
  its sub-states - before that a table with a composite could never
  pass), `all_states_handle<TABLE,
  REQUIRED_EVENTS>` (every state has a transition for each event
  `REQUIRED_EVENTS<STATE>::type` lists, in its own table - event
  coverage: an environment report never silently dropped),
  `all_states_reachable` (per nested table, each from its own initial
  state), `timeouts_within_bounds` / `deadlines_within_bounds` (over
  `all_states_t`: a map entry may name a sub-state, a timed sub-state
  needs one). Rule agreed with
  the firmware (2026-10-01): a user of fsm never writes traits over
  tables, states or observers to work around something missing here -
  the trait is added here instead.
- `fsm::writeDot<TABLE>(out, name)` writes `// table: <short name>` as
  the first line inside the digraph, titles the graph with `name`
  (`label="<name>"; labelloc=t;` - visible in a rendered PNG/SVG, a
  `<text>` directly in the graph's group that fsmview's node and edge
  selectors do not touch), writes node labels as HTML-like
  tables (`label=<<table ...>`, text HTML-escaped) with an `<hr/>` rule
  between the sections name / timeout / annotation set / notes
  (Graphviz renders the rules as `<polygon>`s inside the node's `<g>`, so
  fsmview colors only the node's `<path>`/`<ellipse>`), lists a state's
  annotation set (`mtl::short_value_name<V>()`: the
  compiler's spelling of the constexpr value via `__PRETTY_FUNCTION__` with
  the value as template argument - `color::red`, `lamp{true}`; under
  clang a class value's type in front comes from `type_name<decltype(V)>`
  (`internal::class_value_name_storage`, one char array per value):
  clang spells that type as written where the value was first named,
  `std::get`'s `__tuple_element_t<...>{3}` in writeDot, which the short
  form cut to `level>>{3}`. Inside the braces the spelling stays the
  compiler's - GCC `pair{ns::lamp{false}, ns::color::green}` and
  `inner()`, clang `pair{{false}, 1}` and `inner{}` - tested per
  compiler behind `__clang__` in tests/typename.cpp; the GCC path is
  untouched, so GCC still spells a value named through an alias with
  the alias (`ns::lamp_alias{false}`). An element
  whose type is not structural falls back to its type name), and gives every edge
  `id="<from>__<event>__<to>__<index>"` (index in `TABLE::transitions`,
  keeps guarded alternatives distinct; `internal_target` as `<to>` for
  internal edges). Graphviz passes the ids into its SVG; `tools/fsmview`
  highlights `g.edge[id^="<from>__<event>__<to>__"]` and nodes by
  `<title>`.
- Transition bodies live in `changeState<OLD, NEW>(event)` (payload) /
  `changeState<OLD, NEW>()` / `leaveState`/`enterState<OLD, NEW>`, instantiated
  per EDGE, not per event: all events triggering the same edge share one
  instantiation. The `onTransition` hook is called from
  `doTransition<TRANSITION, STATE, EVENT>` (already per (transition,
  state, event)) so the bodies stay event-agnostic. Measured 17% .text reduction vs. inlining per event
  (GCC 13, -Os).
- Dispatch in `process` goes through `internal::visit` (Visit.hpp,
  moved out of statemachine/StateMachine.hpp 2026-10-04 - a variant utility that knows no
  state, event or table; the machine's event dispatch stays in
  statemachine/StateMachine.hpp, it is the core): a fold expression, measured 3.7 kB smaller
  than `std::visit` on arm-zephyr-eabi GCC 14.3 -Os for a 14-state
  machine; 32 bytes larger on hosted libstdc++ for traffic_light.cpp.
  The `std::visit` path (`MTL_FSM_FOLD_VISIT=0`, `internal::dispatch`
  choosing between the two) was removed 2026-10-04 on the author's
  word: it was never the better one.
- The visitor in `process` instantiates `operator()` for EVERY state, also
  states with no transition for the event - this is NOT waste, do not
  "optimize" it: the `return false` arms are the ignore semantics (process
  can be called in any state), and measured on GCC 13 x86-64 -Os they merge
  into one shared branch - e.g. a 4-state machine, event handled by 2 states,
  compiles to 9 instructions with a single test for both ignoring states.
  Equal to what a hand-filtered dispatch would emit; the only real cost is
  N states x E event types of trivial compile-time instantiations.

## Compile-time checks that must keep firing
- unguarded transition shadowing later alternatives of its `(state, event)`
  pair -> static_assert
- state with `timeout` but no `fsm::timeout` transition -> static_assert
  from `fsm::timed::validate` (fires only when `fsm::timed` is injected;
  without it, timeouts are silently unobserved by design)
- malformed transition (missing/duplicate `from<>`/`on<>`/`to<>`, more than
  one `guard<>`) -> static_assert
- more than one `initial<>`, or `initial<>` naming a state not in the
  table -> static_assert; non-transition table argument -> constraint
  violation
- a guard nobody answers (no static check, no injected object with the
  tagged check), or answered by two injected objects -> static_assert
- a composite state whose submachine is its own table, or whose
  annotation type recurs in its submachine -> static_assert; a
  `fsm::timed`/`deadlined` with fewer levels than `fsm::levels_v<table>`
  -> static_assert from its validate
- `parent_contexts` on a state without a submachine, naming a type
  the machine does not hold, or a type no state of the submachine
  declares -> static_assert

- `fsm::any_state` in `from<>` matches every state and is the last
  alternative: `transitions_for` is the exact (FROM, EVENT) group
  followed by the (any_state, EVENT) group. An unguarded own entry
  overrides the wildcard (how a state is exempted from one); a guarded
  own entry that refuses falls through to it (test
  `refusedOwnGroupFallsThroughToWildcard`). The shared-body proof counts
  every state without an unguarded own entry as a possible source. `any_state` is filtered out of the
  derived state set and satisfies `concepts::state` (usable in `from<>`
  only by convention). The wildcard also matches its own target state
  (self-transition with full exit/entry).
- A state may handle one event several times: alternatives, tried in
  table order, first passing guard fires. An unguarded entry must be
  the last of its group; a second unguarded entry for the pair (a plain
  duplicate included) is the `no_shadowed_alternatives` static_assert.
  Both rules are spelled out in the header's contract comment.

- Events may carry payload: a target state constructible from the
  triggering event (`std::constructible_from<NEW, EVENT const&>`) is
  emplaced with it; otherwise default-constructed. Only the thin payload
  `changeState` overload is per (edge, event); the shared bodies live in
  `leaveState<OLD, NEW>`/`enterState<OLD, NEW>`, still per edge. Observer hooks run
  after the emplace and can read the payload via `getIf<NEW>()` (e.g. a
  driver observer transmitting a message stored in the state).

## Zephyr module (`zephyr/`)
`zephyr/module.yml` (name `mtl`), `zephyr/Kconfig` (`CONFIG_MTL` selects
CPP + full libc++; `CONFIG_MTL_FSM_TRACE` adds the `mtl_fsm` log module
with the standard log-level template), `zephyr/CMakeLists.txt` (adds
`include/` and `zephyr/include/`, compiles `zephyr/src/TraceLogger.cpp`
when tracing is on). Glue headers under `zephyr/include/mtl/zephyr/`:
`Timer.hpp` (`IsrTimer`, `WorkqueueTimer` policies for `fsm::timed`;
`QueuedTimer = fsm::OwningQueuedTimer<IsrTimer>` for queued machines),
`Work.hpp` (`WorkQueue<STACK, PRIO>` - k_work_q, thread and stack in one
object started by its constructor; `Work` the WORK policy, system queue
by default or `Work{queue}` handed to `QueuedMachine<..., Work&, ...>`
by reference; `SpinLock` the LOCK policy, a k_spinlock because the lock
only covers the ring bookkeeping and one event copy),
`StateMachine.hpp` (the facade: `StateMachine{table<T>, observers...}`
the default with its own workqueue, `StateMachineOnSharedWorkqueue{table<T>,
queue, observers...}`, and their base `ConfiguredStateMachine{table<T>,
config<machine_config{...}>, [queue,] observers...}`; derived classes,
not alias templates - clang < 19 cannot deduce through aliases. CTAD via a
table tag - `table<T>` or `table_for<TT>` for a table template over the
deduced observers; holds `fsm::timed`/`fsm::deadlined` on `QueuedTimer`
only if `fsm::has_timed_states_v`/`has_deadlined_states_v`, timers
first in the pack; own workqueue named after the table, or a shared
one; `machine_config` fields default to Kconfig
`MTL_FSM_EVENT_BUFFER_CAPACITY`, `MTL_FSM_WORKQUEUE_STACK_SIZE`,
`MTL_FSM_WORKQUEUE_PRIORITY`; a class member cannot use CTAD and
names `StateMachine<T, O...>` / `StateMachineOnSharedWorkqueue<T,
O...>`; both samples use it) and
`TraceLogger.hpp` (`mtl::zephyr::TraceLogger : fsm::tracing`, lines at
info level of `mtl_fsm`; sinks exist only when
`CONFIG_MTL_FSM_LOG_LEVEL >= LOG_LEVEL_INF`, so the observer compiles to
nothing otherwise - measured 424 B flash on stm32g081b_eval). Zephyr's
LOG macros paste the format string, hence the `MTL_FSM_TRACE_*_PRINTF`
literal macros next to the `fsm::trace_format` constants.
Samples (each adds the repo itself via `ZEPHYR_EXTRA_MODULES`, keeps its
tables in Zephyr-free headers for `west build -t dot`):
`zephyr/samples/traffic_light` (minimal: timeouts, button, tracing) and
`zephyr/samples/sensor` (feature tour: observer-driven virtual sensor with
payload events, the retry loop as a composite state `measuring` owning
`measuring_table` with the retry budget as the submachine's context and
the budget timeout on the parent, guarded alternatives, payload guard,
`any_state` emergency with exact override and internal transitions, LED
sub-machine inside an `fsm::observing` observer (an orthogonal region
driven by annotations, deliberately not a composite), the calibration feature
present only when the `Calibrator` observer is injected -
`CONFIG_SAMPLE_CALIBRATION`, the table a named struct deriving from the
`sensor_transitions` alias of its unnamed table). Build check from the firmware
workspace with the repo's own venv (`build-venv/`, git-ignored; never
install into the user's venvs):
```
cd ~/Documents/Firmware/UsbTypeC && PATH=<repo>/build-venv/bin:$PATH <repo>/build-venv/bin/west build -b nucleo_g474re -d <builddir> <repo>/zephyr/samples/sensor
```
(the shell here is a flatpak sandbox: run it via `flatpak-spawn --host`).

## tools/dotgen
`tools/dotgen/dotgen.py` (also `west fsm_dotgen`, registered through
`zephyr/scripts/west-commands.yml`) crawls headers for named
non-template tables (`struct X : ... fsm::transition_table<...>`, or
`struct X : Y` where any scanned header declares `using Y =
fsm::transition_table<...>` - `table_aliases`, `derives_from_table`;
namespaces tracked, comments/strings blanked), generates a C++ program calling
`fsm::writeDot` per table, builds it with the host compiler against
`include/`, runs it: one `<table>.dot` per table. Templated tables are
skipped with a hint; `--table NS::T<args>=name` renders instantiations,
and naming a scanned table that way renames it (two tables sharing a
struct name in different namespaces).
Table headers must be host-compilable (the Zephyr sample keeps its table
in `traffic_light.hpp` behind a declared `uptimeMs()`). The sample's
`west build -t dot` target calls the script. Tests:
`python3 -m unittest tools/dotgen/test_dotgen.py`.

## tools/fsmview
Python 3 stdlib + one HTML page (`tools/fsmview/README.md`); also the
west extension command `fsm_liveview` (same arguments; both tools keep
their `WestCommand` class behind a guarded `from west.commands import`,
listed in `zephyr/scripts/west-commands.yml`, which the consuming
manifest names in the project's `west-commands:` key - west ignores
`module.yml` and `ZEPHYR_EXTRA_MODULES` for commands; the root `west.yml`
registers them when this repo is the manifest repo). Sources:
`--serial` (default source `/dev/ttyACM0@115200`, pyserial - the one
dependency, checked before the server starts), `--tcp`,
`--listen`, `--stdin`, `--replay`; default graphs are `build/*.dot`
(the sample's `dot` target); live lines are saved verbatim. The server
keeps every received line (`Trace.lines`, ANSI stripped) and serves
lines - `/events?from=N`, SSE `/stream` with the line index as event id -
where a trace line carries its resolved step; the page shows the other
lines behind the "all log lines" checkbox. The pump thread starts from
`Server.service_actions()` so a source failing at once can shut the
server down (a `shutdown()` before `serve_forever()` is lost). A source
failing before its first line shuts the server down with the error
(errors in the pump thread must never be swallowed - `sys.exit` in a
thread only ends the thread). Graphs are rendered with `dot` when found
(`--dot PATH`), else with viz.js from jsDelivr in the browser. `--map
MACHINE=GRAPH` disambiguates templated tables whose short names collide.
Host check: `TrafficLightExample --dot > tl.dot; TrafficLightExample |
fsmview.py tl.dot --stdin`.

## Open ideas (discussed, not implemented)
- Optional `action<A>` transition role (0-or-1 like `guard<G>`, which is
  implemented).
- Tracing ignored events (process() returning false) as a second hook and
  a third line form, shown by fsmview as a flash on the current state.
