# Sensor monitor

A tour of the state machine's features on a Zephyr board: a monitor
reads a virtual sensor once a second, retries failed conversions, raises
an alarm above a limit, drives an LED through a second state machine,
and stops on the user button. Every transition of both machines is
traced, so `tools/fsmview` shows the whole thing live.

| Feature | Where |
|---|---|
| Observer starting work on a state, event with payload | `VirtualSensor` (main.cpp) enters `reading`, answers `reading_done{value}` / `reading_failed` |
| Machine-owned context, constructor from `(event, context)` | `retry_budget`: `retrying` counts a failure, `idle` resets (sensor.hpp) |
| Guarded alternatives, guard on state data | `reading -(reading_failed)->` `retrying` while `retries_left`, else `failed` |
| Guard on the event payload | `reading -(reading_done)->` `alarm` when `above_limit`, else `idle` |
| State constructed from the event | `alarm(reading_done const&)` keeps the value |
| Wildcard source, exact pair overriding it | `any_state -(button)-> emergency`, `emergency -(button)-> idle` |
| Internal transitions | `emergency` counts readings finishing while stopped, in place |
| Sub state machine as an observer | `LedController` observes each state's `led` annotation and runs the LED machine (led.hpp) |
| Value observers with change suppression | `LedController` (`led`), `LedDriver` (`lit`, writes `led0`) |
| Feature enabled by an observer, tagged | `calibrating` declares `using feature = calibration_feature`, `Calibrator` declares `using enables = calibration_feature`; `sensor_table<OBSERVERs...>` is the full list minus every feature none of the injected observers enables (`fsm::remove_disabled_features_t`; `CONFIG_SAMPLE_CALIBRATION`) |
| Explicit initial state, timeouts, wildcard sharing | `led_table`; `fsm::timed` on both machines; the button's transition keeps its shared body (`renotify_safe`, constrained hooks) |
| Tracing | `mtl::zephyr::TraceLogger` on both machines, module `mtl_fsm` |

The tables (`sensor_table<OBSERVERs...>`, filtered by the observers, and `led_table`) are
Zephyr-free headers, so the graph generator builds them on the host.

## Build

```sh
west build -b nucleo_g474re modules/mtl/zephyr/samples/sensor && west flash
west build -b nucleo_g474re modules/mtl/zephyr/samples/sensor -- -DCONFIG_SAMPLE_CALIBRATION=n
```

The second form leaves the calibrator out: no observer enables the
feature, so the machine is built on `sensor_table`, which has no
`calibrating` state at all. Boards without `led0` or `sw0` run without
the LED driver or the emergency stop.

## Watch it live

```sh
west build -t dot        # sensor_table.dot (the configured variant), led_table.dot in build/
west fsm_liveview        # /dev/ttyACM0 at 115200, graphs from build/
```

The page gets one tab per machine; "follow machine" jumps to the one
that just moved. Press the button
during a reading to see the internal transition in `emergency`.
