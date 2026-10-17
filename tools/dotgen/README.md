# dotgen - graphs of every state machine in a source tree

Finds the fsm transition tables in a source tree and writes one Graphviz
`.dot` per table, the graphs `tools/fsmview` loads. It scans headers for
named tables (`struct my_table : fsm::transition_table<...> {}`, or
`struct my_table : my_transitions {}` where a scanned header declares
`using my_transitions = fsm::transition_table<...>;`), generates a C++
program that includes those
headers and calls `fsm::writeDot` for each table, builds it with the
host C++ compiler against the library headers, runs it, and reports the
files written.

```sh
python3 tools/dotgen/dotgen.py [PATH...] [-I DIR]... [-D MACRO]... [-o DIR] [--table NS::TABLE[=NAME]]... [--elf IMAGE]
west fsm_dotgen [-d BUILD_DIR] ...  # the Zephyr command over it, zephyr/scripts/fsm_dotgen.py
```

The generator knows nothing of Zephyr. In a west workspace the usual
call is the bare `west fsm_dotgen` right after `west build`: the
command lets the configured build directory (`build`, or west's
`build.dir-fmt`) answer everything - its image is the `--elf`, its
toolchain's `nm` the `--nm`, its application and modules the `PATH`s,
`<build>/dot` the output. `-d` names another build directory; any
argument given wins over the build's answer, and a `PATH` or `--elf`
on the command line turns the build directory default off.

```sh
west build -b stm32g081b_eval UsbTypeC/samples/pd_drp
west fsm_dotgen                    # build/dot/<table>.dot for every machine in the image
```

From a west workspace with the module at `modules/mtl`, the traffic
light sample's graph:

```sh
west fsm_dotgen modules/mtl/zephyr/samples/traffic_light/src -o graphs
```

- `PATH`: headers or directories to scan (default: the current directory).
  Tables in `.cpp` files are not found - move them into a header.
- `-I DIR`: include directories the table headers need. Rarely needed:
  besides the directories of the scanned headers and the library's
  `include/`, the headers the tables include are looked for in the
  scanned trees and the current directory, and wherever they turn up
  becomes an include directory too. The search follows those headers
  into their own includes, skips `build*` and `dot/` output directories,
  and leaves the headers the host compiler already provides to it, so a
  freestanding libc sitting in the tree cannot shadow the real
  `<cstdint>`.
- `-D MACRO[=VALUE]`: preprocessor definitions the table headers need.
- `-o DIR`: output directory (default `dot/`), one `<table>.dot` per table;
  the file stem and graph title are the struct's name, which is also the
  machine id in the trace lines. The title is drawn at the top of the
  graph, so it shows in a rendered PNG or SVG.
- `--table NS::TABLE[=NAME]`: also render this type, for templated tables
  the scanner has to skip: `--table 'tc::drp::table_for<timing, pref>=tc_drp'`.
  Naming a table the scan found renames it: how two tables sharing a
  struct name in different namespaces are told apart
  (`--table tc::sink_table=tc_sink_table`). Without a NAME an
  instantiation is named after its template with the arguments
  appended, namespaces dropped: `vcs_table<(usbc::power_role)0>` is
  `vcs_table_power_role_0`.
- `--elf IMAGE`: render the tables of the machines built into this
  image and nothing else. The tables are read from the image's symbols
  (`nm -C`; `--nm` names the toolchain's nm, default `$NM` or `nm`), so
  a templated table arrives as the instantiation the image runs -
  `table_for<usbc::default_drp_timing, (usbc::drp_preference)0>` -
  and a sub-table of a composite state as its own machine. The scanned
  headers only say where the declarations live. The arguments must be
  declared in those headers: a timing constant defined in the
  application's `main.cpp` cannot be compiled on the host. A table in
  an anonymous namespace is reported and skipped. The symbols name the
  table a facade hands to the machine, so the graph shows every
  feature; the states of a feature the image's observers leave disabled
  are drawn too.
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
directory. A target reading the image it just built passes
`--elf ${PROJECT_BINARY_DIR}/${KERNEL_ELF_NAME} --nm ${CMAKE_NM}` and
depends on `zephyr_final`.

## Checks

```sh
python3 -m unittest tools/dotgen/test_dotgen.py zephyr/scripts/test_fsm_dotgen.py
```
