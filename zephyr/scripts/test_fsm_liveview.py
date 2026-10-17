# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Checks for the west fsm_liveview command's default graphs:
python3 -m unittest zephyr/scripts/test_fsm_liveview.py"""

import argparse
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fsm_liveview  # noqa: E402


class DefaultGraphs(unittest.TestCase):
    """Without a GRAPH the viewer takes the build's dot/, where west
    fsm_dotgen writes - the same place, so the two commands meet."""

    def parse(self, arguments):
        return fsm_liveview.build_parser(argparse.ArgumentParser()).parse_args(arguments)

    def test_the_build_supplies_its_graphs(self):
        args = self.parse([])
        fsm_liveview.supply(args, "/work/build")
        self.assertEqual(args.graphs, [os.path.join("/work/build", "dot")])

    def test_given_graphs_win(self):
        args = self.parse(["graphs/"])
        fsm_liveview.supply(args, "/work/build")
        self.assertEqual(args.graphs, ["graphs/"])

    def test_no_build_leaves_the_viewer_its_own_default(self):
        args = self.parse([])
        fsm_liveview.supply(args, None)
        self.assertEqual(args.graphs, [])

    def test_the_build_directory_is_an_argument(self):
        self.assertEqual(self.parse(["-d", "out"]).build_dir, "out")


if __name__ == "__main__":
    unittest.main()
