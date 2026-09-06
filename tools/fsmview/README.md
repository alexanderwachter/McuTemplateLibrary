# fsmview - live and replay viewer for fsm state machines

Shows the Graphviz graph of a state machine (as written by
`fsm::writeDot`) and follows what the machine actually does: the current
state and the edge just taken are highlighted, the last N steps fade out
behind them. The steps come from the trace lines of an `fsm::tracing`
observer, either live from a log stream or from a saved log that can be
stepped through forward and backward.

Python 3 with pyserial for the serial port (`pip install pyserial`;
Zephyr's west workspace requirements bring it along); the graphs are
rendered with the Graphviz `dot` binary when it is installed, otherwise
in the browser with viz.js from jsDelivr (needs internet access).

## As a west command

In a west workspace the viewer is the extension command `west
fsm_liveview` (and the graph generator `west fsm_dotgen`), with the same
arguments as the scripts. West loads extension commands only from
projects whose manifest entry names them, so the entry for this
repository needs the `west-commands` key:

```yaml
manifest:
  projects:
    - name: McuTemplateLibrary
      url: https://github.com/alexanderwachter/McuTemplateLibrary
      revision: statemachine
      path: modules/mtl
      west-commands: zephyr/scripts/west-commands.yml
```

No `west update` is needed after adding the key; `west help` then lists
both commands under the project. `ZEPHYR_EXTRA_MODULES` and
`zephyr/module.yml` register the library for the build only, not its
commands.

## Running the viewer

Three things are needed: the graph of the machine, a source of trace
lines, and a browser.

### 1. Get the graph

`tools/dotgen` finds the tables in a source tree and writes one `.dot`
per table (see its README). From a west workspace with the module at
`modules/mtl`, for the traffic light sample:

```sh
west fsm_dotgen modules/mtl/zephyr/samples/traffic_light/src -o graphs
```

The same for your own application's headers, or with the script
directly:

```sh
west fsm_dotgen app/src -o graphs
python3 modules/mtl/tools/dotgen/dotgen.py app/src -o graphs
```

The traffic light sample has it as a build target (`west build -t dot`),
and the host example writes its own graph:

```sh
cmake -B build -G Ninja && cmake --build build
./build/examples/statemachine/TrafficLightExample --dot > traffic_light.dot
```

One `.dot` per machine; the viewer takes several files or a directory.

### 2. Start the viewer on the log source

A board on its console - the default source is `/dev/ttyACM0` at
115200 baud (pyserial), and the default graphs are the `.dot` files in
`build/`:

```sh
tools/fsmview/fsmview.py                                        # build/*.dot, /dev/ttyACM0
tools/fsmview/fsmview.py traffic_light.dot --serial /dev/ttyUSB0@921600
```

A port bridged to TCP with socat (or any log server):

```sh
socat TCP-LISTEN:5555,reuseaddr,fork /dev/ttyACM0,b115200,raw &
tools/fsmview/fsmview.py traffic_light.dot --tcp localhost:5555
```

A program on the host, straight through a pipe:

```sh
./build/examples/statemachine/TrafficLightExample | tools/fsmview/fsmview.py traffic_light.dot --stdin
```

Every received line is saved verbatim to `fsmview-<timestamp>.log` in
the working directory (`--save FILE` picks the name). The viewer prints
where it is running:

```
fsmview: rendering with /usr/bin/dot
fsmview: saving the log to fsmview-20260906-120000.log
fsmview: running - open http://localhost:8420/ in your browser
```

Without pyserial, `stty -F /dev/ttyACM0 115200 raw -echo -icrnl && cat
/dev/ttyACM0 | fsmview.py graph.dot --stdin` does the same with shell
tools. If the port needs permissions (`Permission denied` on `/dev/ttyACM0`),
add yourself to the `dialout` group (`sudo usermod -aG dialout $USER`,
then log in again) or `sudo chmod a+rw /dev/ttyACM0` for the moment.

### 3. Open the page

Open the printed link. `LIVE` follows the newest step; stepping back
(buttons, slider, arrow keys, a click on a log line) leaves live mode,
`>|` re-enters it. Play replays at a fixed pace or in real time when
the lines carry timestamps (Zephyr's `[HH:MM:SS.mmm,uuu]` or the
example's `[ 1234ms]`). The tab bar switches between machines; "follow
machine" jumps to the machine of the current step. Stop the viewer with
Ctrl+C.

### 4. Replay a saved log

```sh
tools/fsmview/fsmview.py traffic_light.dot --replay fsmview-20260906-120000.log
```

The page starts at the first step; use the same controls.

## Options

```
fsmview.py [GRAPH...] [--tcp HOST:PORT | --listen PORT | --serial DEV[@BAUD] | --stdin | --replay FILE]
           [--save FILE] [--http PORT] [--history N] [--map MACHINE=GRAPH]... [--dot PATH]
```

- `GRAPH`: `.dot` files or directories of them, one graph per machine
  (default: the `.dot` files in `build/`).
- At most one source: `--serial` reads a port with pyserial (default
  `/dev/ttyACM0@115200`, and the default source), `--tcp` connects to a
  raw log stream, `--listen` accepts one pusher, `--stdin` reads a pipe,
  `--replay` serves a saved log without a live source.
- `--save FILE`: where the live log goes (default `fsmview-<timestamp>.log`).
- `--history N`: number of fading steps (default 8).
- `--http PORT`: the page's port (default 8420, bound to localhost).
- `--map MACHINE=GRAPH` picks the graph (file stem) for a machine id when
  several graphs claim it - the id is the table's short type name without
  template arguments, so the variants of a templated table all read alike.
  A firmware runs one of them; map it, or load only its `.dot`.
- `--dot PATH`: the Graphviz binary (default: `dot` from PATH or `$DOT`).

## Trace lines

```
fsm[<machine>] -> <state>                  initial state, on construction
fsm[<machine>] <from> -(<event>)-> <to>    a fired transition
```

Anything before `fsm[` is ignored, so a logger's timestamp, module
prefix and terminal colors may precede it. `<machine>` is the short name
of the transition table, matched against the `// table: <name>` comment
in the graph (then the digraph name, then the file name). `<to>` is
`internal_target` for an internal transition - the machine stays in
`<from>` and the dashed self-edge lights up; `<from>` is `any_state` when
a wildcard transition fired through the machine's shared body - the tool
knows the state it was in. A `<from>` that does not match the tracked
state (dropped log lines) and names missing from the graph (a stale
`.dot`) are flagged in the page.

The line grammar and the C++ side live in `include/mtl/StateMachineTrace.hpp`;
the format strings `fsm::trace_format::*` spell it for `std::format` and
printf-style loggers, `mtl::zephyr::TraceLogger` logs it on Zephyr.

## Example: a Zephyr board

`zephyr/samples/traffic_light` runs the traffic light on a board with
`mtl::zephyr::TraceLogger` (module `mtl_fsm`, info level) and the user
button as pedestrian button. Build and flash it, write its graph with
the sample's `dot` target, and read the board's console:

```sh
west build -b nucleo_g474re modules/mtl/zephyr/samples/traffic_light && west flash
west build -t dot                       # writes build/traffic_light_table.dot
west fsm_liveview                       # reads /dev/ttyACM0 at 115200, graphs from build/
```

Or without the build target: `west fsm_dotgen
modules/mtl/zephyr/samples/traffic_light/src -o graphs` and then `west
fsm_liveview graphs`. `zephyr/samples/sensor` is the larger tour with
two machines (sensor monitor and LED), one tab each.

`west fsm_liveview` is this viewer as a west command (manifest note
above); `tools/fsmview/fsmview.py` takes its place outside a workspace.
Details in `zephyr/samples/traffic_light/README.md`.

## Checks

```sh
python3 -m unittest tools/fsmview/test_fsmview.py
```
