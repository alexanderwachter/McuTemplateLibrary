# Traffic light on Zephyr

The traffic light of `examples/statemachine` running on a Zephyr board:
state timeouts through `mtl::zephyr::WorkqueueTimer` on the system
workqueue, the board's user button (devicetree alias `sw0`) as the
pedestrian button - it shortens the green phase once the minimum green
time has elapsed, its ISR only queues work so the machine stays on the
workqueue - and every transition logged by `mtl::zephyr::TraceLogger` on
the `mtl_fsm` log module in the line grammar of `tools/fsmview`. Boards
without `sw0` run on timeouts alone.

## Build

The sample adds the library itself as an extra Zephyr module, so any
west workspace works:

```sh
west build -b <board> modules/mtl/zephyr/samples/traffic_light   # the module's path in the workspace
west flash
```

To use the library from your own application, list this repository in
your `west.yml` (the module is named `mtl`) or set
`ZEPHYR_EXTRA_MODULES`, and enable `CONFIG_MTL=y` plus
`CONFIG_MTL_FSM_TRACE=y` and `CONFIG_STD_CPP20=y`. The `west-commands`
key of the manifest entry is what makes `west fsm_dotgen` and `west
fsm_liveview` available:

```yaml
    - name: McuTemplateLibrary
      url: https://github.com/alexanderwachter/McuTemplateLibrary
      revision: statemachine
      path: modules/mtl
      west-commands: zephyr/scripts/west-commands.yml
```

## Watch it live

The graph comes from the sample's own table (`src/traffic_light.hpp`,
kept free of Zephyr so the host compiler can build the generator):

```sh
west build -t dot                       # writes build/traffic_light_table.dot
west fsm_liveview                       # reads /dev/ttyACM0 at 115200, graphs from build/
```

Then open the printed link, http://localhost:8420/. The defaults are
the board's console and the `.dot` files of `build/`; another port is
`--serial /dev/ttyUSB0@921600`, other graphs are given as arguments.
`west fsm_liveview` is the viewer `tools/fsmview/fsmview.py` as a west
command (see the manifest note under Build; without it, call the script
directly). `west fsm_dotgen modules/mtl/zephyr/samples/traffic_light/src
-o graphs` writes the graph without a build directory (then `west
fsm_liveview graphs`). Silence the trace with
`CONFIG_MTL_FSM_LOG_LEVEL_OFF=y`: the observer then compiles to nothing.
