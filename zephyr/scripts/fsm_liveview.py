#!/usr/bin/env python3
# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""west fsm_liveview: the viewer on the graphs of a Zephyr build.

The generic viewer, tools/fsmview/fsmview.py, takes the graphs to show
as arguments. In a west workspace they are where "west fsm_dotgen" (or
a sample's "west build -t dot") wrote them, <build>/dot, so this
command takes them from there when no GRAPH is given:

  west fsm_liveview [-d BUILD_DIR] [GRAPH...] [--serial DEV[@BAUD]] ...

-d names the build directory; without it the configured one is taken,
as for west build, when it holds a build. Every other argument is
fsmview.py's.
"""

import argparse
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
MTL_ROOT = SCRIPTS.parents[1]
sys.path.insert(0, str(MTL_ROOT / "tools" / "fsmview"))
sys.path.insert(0, str(SCRIPTS))
import fsmview  # noqa: E402
from fsm_dotgen import configured_build_dir  # noqa: E402


def graphs_of(build_dir):
    """Where the build's graphs are: what west fsm_dotgen writes."""
    return str(Path(build_dir) / "dot")


def supply(args, build_dir):
    """The build's graphs for a run naming none."""
    if not args.graphs and build_dir:
        args.graphs = [graphs_of(build_dir)]


def build_parser(parser):
    parser.add_argument("-d", "--build-dir", metavar="DIR",
                        help="the Zephyr build directory whose DIR/dot holds the graphs when "
                             "no GRAPH is given (default: the configured build directory, as "
                             "for west build, when it holds a build)")
    return fsmview.build_parser(parser)


try:
    from west.commands import WestCommand
except ImportError:  # the tests import this file without west
    WestCommand = None

if WestCommand is not None:

    class LiveView(WestCommand):
        def __init__(self):
            super().__init__(
                "fsm_liveview",
                "watch or replay fsm state machines in the browser",
                __doc__ + "\n" + fsmview.__doc__.split("Usage:")[0],
                requires_workspace=False,
            )

        def do_add_parser(self, parser_adder):
            parser = parser_adder.add_parser(
                self.name, help=self.help, description=self.description,
                formatter_class=argparse.RawDescriptionHelpFormatter)
            return build_parser(parser)

        def do_run(self, args, unknown_args):
            if not args.graphs:
                build_dir = args.build_dir or configured_build_dir(self.config)
                if build_dir:
                    self.inf(f"fsmview: graphs from {graphs_of(build_dir)}")
                supply(args, build_dir)
            fsmview.run(args)
