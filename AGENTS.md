# Working on this repository

Rules and know-how for contributors, human or AI agent. README.md says
what the library does; this file says how to change it without breaking
its design.

## Layout

| Path | What |
|---|---|
| `include/mtl/Typelist.hpp`, `TypelistAlgorithms.hpp` | typelists and compile-time algorithms; everything else builds on them |
| `include/mtl/TypeName.hpp` | compile-time type names (`short_name_of<T>` is the machine/state/event id everywhere) |
| `include/mtl/StateMachine.hpp` | the `fsm` state machine: tables, machine, observers (`timed`, `observing`, `observer_group`), compile-time checks |
| `include/mtl/StateMachineTrace.hpp` | `fsm::tracing` observer and the trace line grammar (target code) |
| `include/mtl/StateMachineDot.hpp` | Graphviz output (host tooling only) |
| `tests/` | one `int xTests()` per file, summed in `main.cpp` |
| `examples/statemachine/traffic_light.cpp` | host demo, also the host feed for the viewer (`--dot`, trace lines on stdout) |
| `zephyr/` | Zephyr module: `module.yml`, `Kconfig`, `CMakeLists.txt`, glue in `include/mtl/zephyr/` (`Timer.hpp`, `TraceLogger.hpp`), `src/TraceLogger.cpp`, `samples/traffic_light`, `samples/sensor`, `scripts/west-commands.yml` |
| `tools/dotgen` | crawls headers for tables, builds a host generator, writes `.dot` files; `west fsm_dotgen` |
| `tools/fsmview` | live/replay viewer, Python + one HTML page; `west fsm_liveview` |


## Build and test - run all of it before claiming done

```sh
cmake -B build -G Ninja && cmake --build build && ./build/tests/TemplateMetaProgrammingTests
cmake -B build-visit -G Ninja -DCMAKE_CXX_FLAGS="-DMTL_FSM_FOLD_VISIT=0" && cmake --build build-visit && ./build-visit/tests/TemplateMetaProgrammingTests
python3 -m unittest tools/fsmview/test_fsmview.py tools/dotgen/test_dotgen.py
```

Both dispatch paths (the fold and `std::visit`) run the full suite. A
change under `zephyr/` needs a real Zephyr build of both samples from a
west workspace (`west build -b nucleo_g474re <repo>/zephyr/samples/sensor`,
plus `-- -DCONFIG_SAMPLE_CALIBRATION=n`, plus `-t dot`); GCC 15 on the
host, arm-zephyr-eabi GCC 14 on the target. Use your own Python venv for
west, never a developer's.

Tests follow the repo style: compile-time checks are `static_assert`s in
named namespaces; runtime checks are small isolated functions using
`check()`, registered in the file's `xTests()`. Negative tests (things
that must not compile) are done with scratch probes and `-fsyntax-only`,
expecting the specific `static_assert` message - a class template's
`static_assert` fires only on instantiation.

## Conventions

- Names: ALL_CAPS template parameters, packs ending in `s` (`TRANSITIONs`,
  `OBSERVERs`), `_t`/`_v` aliases for every trait, camelCase member
  functions and hooks (`onEnterState`, `notifyEntry`, `getIf`).
  Two layers of type names: the library (`include/mtl`) is snake_case
  like the standard library - `state_machine`, `timed`, `observing` -
  and so are all states, events, guards and tables everywhere
  (`reading`, `reading_done`, `sensor_table`); Zephyr glue and
  application classes are PascalCase - `TraceLogger`, `WorkqueueTimer`,
  a sample's `VirtualSensor`, `LedController`. Constexpr flags are
  snake_case (`renotify_safe`, `source_agnostic`).
- Concepts over SFINAE. Constrain template parameters with named concepts
  in a nested `concepts` namespace; detect optional members with
  requires-expressions and `if constexpr`. The one SFINAE idiom kept is
  the trailing-return-type opt-out (`-> decltype(STATE::member)`).
- A missing optional member means "not wanted", never an error: hooks,
  annotations, sinks are all detected individually.
- Comments explain contracts and non-obvious behavior
  (ordering constraints), not what the code says. Keep the header's
  contract comment at the top of `StateMachine.hpp` the source of truth
  for the rules; README's "Rules and edge cases" mirrors it.
- Codegen claims are measured before they are written down (compiler,
  flags, size).
- No backward-compatibility shims: the repo is pre-release. Pick the
  best design and update every call site, samples and tools included.
- Commit small, descriptive commits on a topic branch off `statemachine`;
  the branch is squash-merged. Commit only what you built and tested.
  Never commit `__pycache__`, build trees.

## Design rules that must not regress

- Tables are types built from named, order-independent roles; the state
  set is derived from the table; the first state of the first transition
  is the initial state unless `fsm::initial<S>` says otherwise.
- The machine core knows nothing about timers, annotations, tracing or
  targets. Observers are its only extension point; new behavior goes into
  an observer, not into `state_machine`.
- Optional features are tags: states declare `using feature = TAG;`,
  observers `using enables = TAG;`, and the table is one full entry
  list filtered with `fsm::remove_disabled_features_t` (or `remove_features_t`)
  (one pass, `initial<>` and `timed_by<>` entries included). Do not
  compose feature tables by hand from partial lists, and do not
  reimplement the filter in an application.
- Transition bodies are instantiated per edge, not per event. Anything
  that must know the event runs from `fire()`, which is already per
  (transition, state, event) - do not push the event into the per-edge
  bodies. The wildcard's shared body per (event, target) must stay
  provably unobservable; a new hook needs its shareability rule in
  `observerSharesEdge`.
- `process()` instantiates the visitor for every state on purpose (the
  `return false` arms are the ignore semantics); do not "optimize" it.
- Alternatives: first passing guard in table order fires, an unguarded
  entry is last, a second unguarded one is a `static_assert`. A state's
  own `(state, event)` group replaces the wildcard even when it refuses.
- Observers are injected by reference and outlive the machine; the
  machine is neither copyable nor movable.
- Construction enters the initial state with `OLD = mtl::nil_type`; the
  shared wildcard path enters with `OLD = fsm::any_state`; `onTransition`
  never fires on construction.

## Contracts between C++ and the tools

- Trace line grammar (`StateMachineTrace.hpp`, parsed by `fsmview.py`):
  `fsm[<machine>] -> <state>` and `fsm[<machine>] <from> -(<event>)-> <to>`,
  names verbatim as `short_name_of` spells them, `internal_target` and
  `any_state` included. Anything before `fsm[` is ignored, ANSI colors
  stripped. Change the grammar in the header, the printf macros, the
  parser and its tests together.
- DOT output (`StateMachineDot.hpp`, read by `fsmview.py` and
  `dotgen.py`): `// table: <short name>` as the first line inside the
  digraph, edge ids `<from>__<event>__<to>__<index>`. `tests/dot.cpp`
  pins the exact text.
- Machine id = the table's short type name: tables are named structs
  (`struct my_table : fsm::transition_table<...> {}`); an alias reads
  `transition_table`, template instantiations read alike.
- Tables that a Zephyr sample wants graphed live in Zephyr-free headers
  so the host compiler can build the generator (`traffic_light.hpp`,
  `sensor.hpp`, `led.hpp`); kernel calls go behind a declared function.

## Zephyr specifics

- `CONFIG_MTL` (headers), `CONFIG_MTL_FSM_TRACE` (log module `mtl_fsm`,
  `TraceLogger`, lines at info level; below that the observer compiles
  to nothing, name strings included - keep it that way).
- Zephyr's `LOG_*` macros paste the format string: it must be a literal,
  hence `MTL_FSM_TRACE_*_PRINTF` macros next to the `trace_format` constants.
- West extension commands come from the manifest (`west-commands:` in the
  project entry, or the root `west.yml` when this repo is the manifest
  repo), not from `module.yml` or `ZEPHYR_EXTRA_MODULES`. The tools keep
  their `WestCommand` class behind a guarded `from west.commands import`.
- Static machines are constructed before `main()`; observers touching
  hardware defer until an explicit attach from `main()`.

## Pitfalls seen before

- VS Code IntelliSense reports "alias template incompatible" and similar
  errors in the headers; they are EDG misparses of C++20. Trust GCC.
- Hooks are detected with requires-expressions, so an observer with two
  constrained `onEnterState` overloads whose constraints can both hold
  for one edge makes that call ambiguous - and the machine then treats
  the hook as absent, silently. Keep such constraints mutually exclusive
  (the sensor sample's `VirtualSensor`).
- In `fsmview.py`, the source runs in a thread: `sys.exit` there only
  ends the thread. Raise, catch in `pump()`, shut the server down.
- Graphviz `dot` may exist on the host but not in a sandbox; the viewer
  falls back to viz.js, dotgen needs a host C++ compiler.
- The submodule copy of this library inside a firmware may be ahead of
  `origin`; compare before planning against a checkout.
