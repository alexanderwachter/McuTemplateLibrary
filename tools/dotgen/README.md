# dotgen - graphs of every state machine in a source tree

Finds the fsm transition tables in a source tree and writes one Graphviz
`.dot` per table, the graphs `tools/fsmview` loads. It scans headers for
named tables (`struct my_table : fsm::transition_table<...> {}`, also
through `mtl::rebind_t`), generates a C++ program that includes those
headers and calls `fsm::writeDot` for each table, builds it with the
host C++ compiler against the library headers, runs it, and reports the
files written.

```sh
python3 tools/dotgen/dotgen.py [PATH...] [-I DIR]... [-D MACRO]... [-o DIR] [--table NS::TABLE[=NAME]]...
west fsm_dotgen ...                # the same as an extension command of the mtl Zephyr module
```

From a west workspace with the module at `modules/mtl`, the traffic
light sample's graph:

```sh
west fsm_dotgen modules/mtl/zephyr/samples/traffic_light/src -o graphs
```

- `PATH`: headers or directories to scan (default: the current directory).
  Tables in `.cpp` files are not found - move them into a header.
- `-I DIR`: include directories the table headers need; the directories
  of the scanned headers and the library's `include/` are added.
- `-D MACRO[=VALUE]`: preprocessor definitions the table headers need.
- `-o DIR`: output directory (default `dot/`), one `<table>.dot` per table;
  the file stem and graph title are the struct's name, which is also the
  machine id in the trace lines.
- `--table NS::TABLE[=NAME]`: also render this type, for templated tables
  the scanner has to skip: `--table 'tc::drp::table_for<timing, pref>=tc_drp'`.
- `--cxx`, `--std`: host compiler (default `$CXX` or `c++`) and standard
  (default `c++20`); `--keep` leaves the generated `dotgen.cpp` in the
  output directory.

The table headers must compile on the host, so keep them free of target
headers: put kernel calls behind a declared function, as
`zephyr/samples/traffic_light/src/traffic_light.hpp` does with
`uptimeMs()`. The generator never runs guards or entry actions, so the
declaration alone suffices.

`west fsm_dotgen` (and `west fsm_liveview` for the viewer) is available
in a west workspace whose manifest lists this repository with its
commands - west reads extension commands from the manifest, not from
`ZEPHYR_EXTRA_MODULES` or `module.yml`:

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
the commands under the project.

The traffic light sample also wraps the generator as a build target:
`west build -t dot` writes `traffic_light_table.dot` into the build
directory.

## Checks

```sh
python3 -m unittest tools/dotgen/test_dotgen.py
```
