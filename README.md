# McuTemplateLibrary (mtl)

Header-only C++20 for microcontrollers: typelists and compile-time
algorithms, and on top of them `fsm`, a template-based state machine
whose tables are types. Also a Zephyr module, with a trace logger and
timer policies, and two host tools: a graph generator and a live viewer.

```
cmake -B build -G Ninja && cmake --build build && ./build/tests/TemplateMetaProgrammingTests
python3 -m unittest tools/fsmview/test_fsmview.py tools/dotgen/test_dotgen.py
```

## Typelists and compile-time utilities

Everything is in namespace `mtl`; each `X` comes as a trait with `::type`
or `::value` and an `X_t`/`X_v` alias. Predicates and comparators are
template template parameters (`template<typename> typename PREDICATE`
yielding `::value`, `template<typename, typename> typename COMPARE`).

**`Typelist.hpp` - `mtl::typelist<Ts...>`**

| Name | Meaning |
|---|---|
| `typelist<Ts...>` | the list; `nil_type` is the "nothing" result of searches |
| `is_typelist_v<T>`, `concepts::typelist` | is T a typelist |
| `count_v<LIST>` | number of elements |
| `front_t<LIST>`, `back_t<LIST>` | first / last element |
| `at_t<INDEX, LIST>` | element by index (`__type_pack_element` when the compiler has it, otherwise an 8-at-a-time recursion) |
| `index_of_v<ELEMENT, LIST>` | index of the first occurrence |
| `append_t<T, LIST>`, `prepend_t<T, LIST>` | add at the back / front |
| `concat_t<LIST1, LIST2>` | concatenation |
| `reverse_t<LIST>` | reversed order |
| `remove_front_t<LIST>`, `remove_back_t<LIST>`, `remove_at_t<INDEX, LIST>` | drop one element |
| `linearize_t<LIST>` | flatten nested typelists into one |
| `rebind_t<LIST, TARGET>` | `TARGET<Ts...>` for any variadic template (`std::variant`, `std::tuple`, `fsm::transition_table`) |
| `common_type_t<LIST>`, `common_value_type_t<LIST>` | `std::common_type` of the elements / of their `value_type`s |

**`TypelistAlgorithms.hpp`**

| Name | Meaning |
|---|---|
| `all_of_v`, `any_of_v`, `none_of_v<LIST, PREDICATE>` | quantifiers |
| `has_a_v<LIST, T>` | membership |
| `count_if_v<LIST, PREDICATE>` | how many elements satisfy the predicate |
| `find_if_t<LIST, PREDICATE>` | first element satisfying it, `nil_type` if none |
| `filter_t<LIST, PREDICATE>`, `remove_if_t<LIST, PREDICATE>` | keep / drop the elements satisfying it |
| `transform_t<LIST, OPERATION>` | apply `OPERATION<T>::type` to each element |
| `unique_t<LIST>` | deduplicate, keeping the first occurrence (`unique_keep_last_t` keeps the last) |
| `sort_t<LIST, COMPARE>` | insertion sort by a comparator |
| `min_t`, `max_t<LIST, COMPARE>` | the extreme element; the default comparators order by `::value` |
| `sum_v<LIST>` | sum of the elements' `::value` |
| `gt_size`, `lt_size`, `gt_size_v`, `lt_size_v<T1, T2>` | comparators by `sizeof`, for `sort_t`/`min_t`/`max_t` |

**`TypeName.hpp`** - `type_name<T>()` (the compiler's spelling),
`short_name<T>()` (namespaces and template arguments stripped), both
`constexpr std::string_view`; `short_name_of<T>` the short name as a
null-terminated `char const*` in static storage, for C APIs and
deferred loggers.

**`Utils.hpp`** - `width_to_uint_t<WIDTH>` / `width_to_int_t<WIDTH>`:
the smallest fixed-width integer holding WIDTH bits (`bool` for 1);
the `_exact` forms accept 1, 8, 16, 32, 64 only.

## fsm - the state machine

`include/mtl/StateMachine.hpp`. A machine is `fsm::state_machine<TABLE,
OBSERVERs...>`; events go in through `process(event)`, which returns
whether a transition fired.

### Features

- **Tables are types.** A table is a list of `fsm::transition<fsm::from<A>,
  fsm::on<E>, fsm::to<B>>` (roles in any order, an optional
  `fsm::guard<G>`), plus an optional `fsm::initial<S>`. The state set is
  derived from the table. Give tables a name - `struct my_table :
  fsm::transition_table<...> {}` - and compose them from typelists
  (`mtl::concat_t`, `mtl::rebind_t`) to switch features in and out.
- **States are plain classes.** Optional members are detected: `onEntry()`,
  `onExit()`, `static constexpr timeout`, and any `static constexpr`
  annotation an observer watches. The initial state is the first state
  of the first transition unless `fsm::initial<S>` says otherwise.
- **Events carry payload.** A target state constructible from the event
  is constructed from it; otherwise it is default-constructed.
- **Guards** gate a transition with `check(state, event)`, `check(state)`
  or `check()`, the most specific form the guard provides.
- **Alternatives.** One `(state, event)` pair may have several entries:
  the first whose guard passes fires; an unguarded entry is the
  catch-all.
- **Wildcard.** `fsm::from<fsm::any_state>` matches every state; a state's
  own entry for the event overrides it.
- **Internal transitions.** `fsm::internal_transition<from<S>, on<E>>`
  handles E inside S via `S::handle(E const&)`: no exit, no entry, no
  timer restart.
- **Context.** A state with a reference member named `context` is
  constructed with a machine-owned instance of that type, shared by
  every state naming it and surviving transitions: a retry budget, a
  running count.
- **Observers** are the only extension point. Injected by reference,
  they get `onExitState<OLD, NEW>`, `onEnterState<OLD, NEW>` and
  `onTransition<FROM, EVENT, TO>` hooks (each optional) and a
  compile-time `validate<TABLE>()`. Built on them: `fsm::timed<TIMER>`
  (state timeouts through an injected timer policy), `fsm::observing`
  (value observers on state annotations, with compile-time change
  suppression), `fsm::tracing` (transition trace lines),
  `fsm::observer_group` (several observers as one). A state feature no
  observer consumes is silently unobserved.
- **Checked at compile time.** Malformed transitions, dead alternatives,
  an `initial<>` that is no state, timed states without a timeout
  transition (with `fsm::timed`), guards not matching their state,
  reachability and timeout bounds (`fsm::all_states_reachable_v`,
  `fsm::timeouts_within_bounds_v`), observer coverage
  (`fsm::all_states_notified_v`).
- **Small.** Transition bodies are instantiated per edge, not per event;
  wildcard transitions fire through one shared body per target when no
  observer could tell the source apart; dispatch is a fold, no tables.

### Rules and edge cases

- `process()` returns false when the state has no entry for the event or
  every alternative's guard refuses. Nothing runs then: no exit, no
  entry, no hook, and a running timeout keeps running - but a refused
  `fsm::timeout` transition does not re-arm the one-shot timer.
- Alternatives are tried in table order. An unguarded entry must be the
  last of its `(state, event)` group; anything after it could never
  fire, so a second unguarded entry - a plain duplicate included - is a
  `static_assert`.
- A state's own `(state, event)` group replaces the wildcard group
  entirely, also when all its guards refuse: the result is false, not
  the wildcard. An exact pair is therefore the way to exempt a state
  from a wildcard. Wildcard entries form alternatives among themselves,
  and the wildcard matches its own target too (a full self-transition).
- Internal transitions group with regular ones as alternatives, take a
  guard like any other, and do not support `from<any_state>`.
- Order on a transition: observers' `onExitState` (old state alive),
  the old state's `onExit()`, the new state is constructed, observers'
  `onEnterState`, the new state's `onEntry()`, observers'
  `onTransition`. Observers run in injection order: put `fsm::timed`
  first so timers are armed before anything is notified, a tracer last
  so its line follows the effects.
- On construction the initial state is entered with
  `OLD = mtl::nil_type`; no `onTransition` fires. On the shared wildcard
  path the source is unknowable: hooks see `OLD = fsm::any_state`. A raw
  `onExitState`/`onEnterState` hook, or an `onTransition` without
  `static constexpr bool source_agnostic = true`, makes the machine fall
  back to per-source bodies; a hook constrained to particular states
  (`requires std::is_same_v<NEW_STATE, reading>`) does not.
- `fsm::observing` notifies a value only when it changes between the two
  states (decided at compile time for static annotations); an
  annotation type declaring `idempotent`, or an observer declaring
  `renotify_safe`, tolerates re-notification and keeps wildcards shared.
- Context types are default-constructible; context states need a
  constructor from `(context&)` and may add `(event const&, context&)`.
  Context is never reset by the machine - a state's constructor does it.
- Guards run before any exit action, on the still-current state; the
  event form sees the payload before it is delivered.
- `process()` is not re-entrant and not thread-safe: the timer policy's
  callback and every other event source must run in one context
  (Zephyr: one workqueue). Observers hand out the machine's address and
  may keep it, so the machine is neither copyable nor movable.
- Machine ids: trace lines and graphs name a machine by its table's
  short type name, so a table alias reads `transition_table` and every
  instantiation of a templated table reads alike.

## Zephyr

The repository is a Zephyr module (`zephyr/module.yml`, name `mtl`).
`CONFIG_MTL` adds the headers, `CONFIG_MTL_FSM_TRACE` the `mtl_fsm` log
module with `mtl::zephyr::TraceLogger`; `mtl::zephyr::IsrTimer` and
`WorkqueueTimer` are timer policies for `fsm::timed`. Samples:
`zephyr/samples/traffic_light` (minimal) and `zephyr/samples/sensor`
(a tour of the features above). Listing the repository in a west
manifest with `west-commands: zephyr/scripts/west-commands.yml` adds
`west fsm_dotgen` and `west fsm_liveview`.

## Tools

- `tools/dotgen`: finds the tables in a source tree and writes one
  Graphviz `.dot` per table, built with the host compiler.
- `tools/fsmview`: live and replay viewer in the browser, fed by trace
  lines from a serial port, TCP, a pipe or a saved log.
