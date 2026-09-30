# Project context: template-based state machine (fsm)

Repo and branch are WIP with no users yet: do not care about breaking
changes or backward compatibility - pick the best design and update all
call sites.

C++20 header-only state machine built on the mtl library in this repo
(`include/mtl`: `typelist`, `find_if`, `count_if`, `unique`, `all_of`, `front`).
Main files: `StateMachine.hpp` (the contract comment; includes the parts in
`statemachine/`: `Transition.hpp`, `Table.hpp`, `Timeout.hpp`, `Timer.hpp`,
`Observing.hpp`, `Observer.hpp`, `Traits.hpp`, `Core.hpp` - the machine and
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
```
Tests follow the repo style: compile-time checks as `static_assert` in named
namespaces; runtime checks (state machine only) as small isolated test
functions using `check()`, reported through `main.cpp`.
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
- Constrain template parameters with concepts wherever applicable; prefer
  concepts over SFINAE. `fsm::concepts`: `state`, `guard_for<G, STATE>`
  (precise guard contract), `transition`, `transition_role`,
  `transition_table_entry`, `transition_table`, `timer`.
- Optional `guard<G>` role (`mtl::count_if` allows 0 or 1; `transition::guard`
  is `mtl::nil_type` when absent). G is a question, a default-constructible
  class: answered by its own static `check(FROM const&, EVENT const&)`,
  `check(FROM const&)` or `check()` (`fsm::concepts::guard_for`), or by
  an object injected into the machine with the tag in front,
  `check(G, FROM const&[, EVENT const&])` / `check(G)`
  (`concepts::answers_guard_for<OBJECT, G, FROM>`); the most specific
  form wins, an injected answer wins over the static one. The machine
  static_asserts every guard answered (`internal::guard_answered_in`)
  and by at most one injected object; `StateMachine::checkGuard` resolves
  it per (guard, from-state). The table never names the answerer's
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
  Give tables a name (`struct my_table : fsm::transition_table<...> {}`):
  the short type name is the machine id in trace lines and DOT graphs, an
  alias reads "transition_table". `StateMachine` exposes it as `table`.
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
  one shared body per (event, target) (`changeShared`; expanding whole
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
  children...>` next to its state variant (`sub_`; `nil_type` in a flat
  table), constructs it in `enterSubmachine<STATE>` after the entry and
  transition hooks (constructor, `doTransition`, `changeShared`), and
  tears it down first thing in `leave<OLD, NEW>` via the child's private
  `leaveCurrent()` - the active state left with `TO = nil_type`,
  innermost first (`StateMachine` befriends all its specializations).
  A root's destructor still runs no hooks. Dispatch step 0 in
  `process`: a composite offers the event to `submachineOf<STATE>()`
  first, `true` is `fired` without touching the parent, `false` falls
  through to the own group and the wildcards. `fsm::is_local_event`
  (`timeout`, `deadline`, user-specializable) never descends: the timed
  observer injects an expiry into the machine it armed for, and a timed
  sub-state would otherwise swallow the root's. `StateMachine::table` is
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
  composite restarts the child at its initial state; contexts are per
  machine (a child's context is fresh on every entry); features are not
  filtered inside sub-tables. Tests: namespace `Nested`.
- Timeouts are an observer concern: `fsm::timed<TIMER, LEVELS = 1>` owns
  injected timer policies (`fsm::concepts::timer`, `start(ms,
  fsm::timer_callback, void*)` / `stop()`), one-shot, one per machine
  level (`internal::timer_slots`, `timer(level)`; the reference form
  `timed<TIMER&, N>` takes N caller-owned timers), armed in its
  `onEnter<STATE>` hook on slot `MACHINE::depth`, stopped in
  `onExit<STATE>` when a timed state is left. The callback injects
  `fsm::timeout` into the machine whose state was armed. Its
  `validate<TABLE>()` static_asserts `LEVELS >= levels_v<TABLE>` and that
  every timed state of every level has an unguarded `fsm::timeout`
  transition in its own table. Place it before value observers so timers
  are armed before they are notified. `process()` is not re-entrant;
  serialization is the policy user's job. `QueuedTimerBase` keeps the
  armed callback/context and `deliver()`s them from the drain (one
  channel per level, outer first); `QueuedMachine`'s ring takes
  `nested_events_t`.
- Observers are injected BY REFERENCE (`std::tuple<OBSERVERs&...>`) and
  must outlive the machine. Value observation lives in the
  `fsm::observing<DERIVED>` CRTP base: derived provides
  `observe_static<STATE>() -> decltype(STATE::member)` (trailing return
  type = SFINAE opt-out for states without the member) plus
  `notifyEntry(value)` (new state's value, on entry) and/or
  `notifyExit(value)` (old state's value, on exit, old state still
  alive) - each detected by requires. The base implements both hooks
  with the change check as `if constexpr` on constexpr values:
  equal-value transitions emit no code. First `notifyEntry` fires
  during machine construction; `notifyExit` never does.
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
  optional `using observes = mtl::typelist<...>;` is carried by a state
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
- Features as tags (library, StateMachine.hpp "optional features"): a
  state declares `using feature = TAG;`, an observer `using enables =
  TAG;` (or an `mtl::typelist` of tags). `fsm::observer_enables_v`,
  `state_in_feature_v`, `feature_enabled_v<TAG, OBSERVERs...>`;
  `remove_features_t<LIST, typelist<TAGs...>>` / `remove_feature_t`
  remove the tagged states' entries in one pass - transitions from/to,
  `initial<>` (the next entry's source leads), `timed_by<>` map entries -
  and `remove_disabled_features_t<LIST, OBSERVERs...>` removes every feature no
  observer enables. Origin: the USB-C firmware's `pe::` machinery, now
  replaced by these. Tests: namespace `Features` in tests/statemachine.cpp.
- `fsm::writeDot<TABLE>(out, name)` writes `// table: <short name>` as
  the first line inside the digraph, writes node labels as HTML-like
  tables (`label=<<table ...>`, text HTML-escaped) with an `<hr/>` rule
  between the sections name / timeout / annotation set / notes
  (Graphviz renders the rules as `<polygon>`s inside the node's `<g>`, so
  fsmview colors only the node's `<path>`/`<ellipse>`), lists a state's
  annotation set (`mtl::short_value_name<V>()`: the
  compiler's spelling of the constexpr value via `__PRETTY_FUNCTION__` with
  the value as template argument - `color::red`, `lamp{true}`; an element
  whose type is not structural falls back to its type name), and gives every edge
  `id="<from>__<event>__<to>__<index>"` (index in `TABLE::transitions`,
  keeps guarded alternatives distinct; `internal_target` as `<to>` for
  internal edges). Graphviz passes the ids into its SVG; `tools/fsmview`
  highlights `g.edge[id^="<from>__<event>__<to>__"]` and nodes by
  `<title>`.
- Transition bodies live in `changeState<OLD, NEW>(event)` (payload) /
  `changeState<OLD, NEW>()` / `leave`/`enter<OLD, NEW>`, instantiated
  per EDGE, not per event: all events triggering the same edge share one
  instantiation. The `onTransition` hook is called from
  `doTransition<TRANSITION, STATE, EVENT>` (already per (transition,
  state, event)) so the bodies stay event-agnostic. Measured 17% .text reduction vs. inlining per event
  (GCC 13, -Os).
- Dispatch in `process` goes through `internal::dispatch`: the
  fold-expression `internal::visit` by default (measured 3.7 kB smaller
  than `std::visit` on arm-zephyr-eabi GCC 14.3 -Os for a 14-state
  machine; 32 bytes larger on hosted libstdc++ for traffic_light.cpp),
  `MTL_FSM_FOLD_VISIT=0` selects `std::visit`. Both paths run the full
  test suite.
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
  `leave<OLD, NEW>`/`enter<OLD, NEW>`, still per edge. Observer hooks run
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
payload events, context retry budget, guarded alternatives, payload guard,
`any_state` emergency with exact override and internal transitions, LED
sub-machine inside an `fsm::observing` observer, the calibration feature
present only when the `Calibrator` observer is injected -
`CONFIG_SAMPLE_CALIBRATION`, table composed from typelists with
`mtl::concat_t`/`mtl::rebind_t`). Build check from the firmware
workspace with the repo's own venv (`build-venv/`, git-ignored; never
install into the user's venvs):
```
cd ~/Documents/Firmware/UsbTypeC && PATH=<repo>/build-venv/bin:$PATH <repo>/build-venv/bin/west build -b nucleo_g474re -d <builddir> <repo>/zephyr/samples/sensor
```
(the shell here is a flatpak sandbox: run it via `flatpak-spawn --host`).

## tools/dotgen
`tools/dotgen/dotgen.py` (also `west fsm_dotgen`, registered through
`zephyr/scripts/west-commands.yml`) crawls headers for named
non-template tables (`struct X : ... fsm::transition_table<...>`, namespaces
tracked, comments/strings blanked), generates a C++ program calling
`fsm::writeDot` per table, builds it with the host compiler against
`include/`, runs it: one `<table>.dot` per table. Templated tables are
skipped with a hint; `--table NS::T<args>=name` renders instantiations.
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
