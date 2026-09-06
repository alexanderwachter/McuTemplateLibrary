# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Checks for the trace line parser, the DOT graph reader and the step
resolution of fsmview.py:  python3 -m unittest tools/fsmview/test_fsmview.py"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fsmview  # noqa: E402

TRAFFIC_LIGHT_DOT = """digraph "traffic_light" {
    // table: traffic_light_table
    rankdir=LR;
    node [shape=box, style=rounded];
    __initial [shape=point];
    "red" [label="red\\ntimeout 2000 ms"];
    "green" [label="green\\ntimeout 6000 ms"];
    "yellow";
    "red" -> "green" [label="timeout" id="red__timeout__green__0"];
    "green" -> "yellow" [label="timeout" id="green__timeout__yellow__1"];
    "green" -> "yellow" [label="pedestrian_button\\n[minimum_green_elapsed]" id="green__pedestrian_button__yellow__2"];
    "yellow" -> "red" [label="timeout" id="yellow__timeout__red__3"];
    "green" -> "green" [label="tick\\n(internal)" id="green__tick__internal_target__4" style=dashed];
    __initial -> "red";
}
"""

WILDCARD_DOT = """digraph "wild" {
    // table: wild_table
    __initial [shape=point];
    "any_state" [style=dashed];
    "a";
    "dead";
    "a" -> "dead" [label="kill" id="a__kill__dead__0"];
    "any_state" -> "dead" [label="kill" id="any_state__kill__dead__1"];
    __initial -> "a";
}
"""


class ParseLine(unittest.TestCase):
    def test_transition_with_zephyr_prefix(self):
        record = fsmview.parse_line(
            "[00:01:02.345,678] <dbg> usbc_fsm: traceTransition: "
            "fsm[sink_table] unattached_snk -(cc_changed)-> attach_wait_snk\n")
        self.assertEqual(record["machine"], "sink_table")
        self.assertEqual(record["from"], "unattached_snk")
        self.assertEqual(record["event"], "cc_changed")
        self.assertEqual(record["to"], "attach_wait_snk")
        self.assertAlmostEqual(record["ts"], 62.345678)
        self.assertFalse(record["raw"].endswith("\n"))

    def test_zephyr_colors_are_stripped(self):
        record = fsmview.parse_line(
            "[00:05:22.013,000] \x1b[0m<inf> mtl_fsm: "
            "fsm[traffic_light_table] green -(timeout)-> yellow\x1b[0m\r\n")
        self.assertEqual(record["to"], "yellow")
        self.assertNotIn("\x1b", record["raw"])

    def test_initial_with_example_prefix(self):
        record = fsmview.parse_line("[  1234ms] fsm[traffic_light_table] -> red")
        self.assertEqual(record["machine"], "traffic_light_table")
        self.assertIsNone(record["from"])
        self.assertIsNone(record["event"])
        self.assertEqual(record["to"], "red")
        self.assertAlmostEqual(record["ts"], 1.234)

    def test_no_prefix_no_timestamp(self):
        record = fsmview.parse_line("fsm[t] a -(e)-> b")
        self.assertEqual((record["from"], record["event"], record["to"]), ("a", "e", "b"))
        self.assertIsNone(record["ts"])

    def test_other_lines_are_ignored(self):
        self.assertIsNone(fsmview.parse_line("[   0ms] lamps: red=true"))
        self.assertIsNone(fsmview.parse_line(""))
        self.assertIsNone(fsmview.parse_line("fsm[t] garbage"))


class GraphReader(unittest.TestCase):
    def setUp(self):
        self.graph = fsmview.Graph("traffic_light", TRAFFIC_LIGHT_DOT)

    def test_identity(self):
        self.assertEqual(self.graph.name, "traffic_light")
        self.assertEqual(self.graph.table, "traffic_light_table")
        self.assertEqual(self.graph.initial, "red")

    def test_states_and_events(self):
        self.assertEqual(self.graph.states, ["red", "green", "yellow"])
        self.assertEqual(self.graph.events, ["pedestrian_button", "tick", "timeout"])

    def test_edges(self):
        internal = next(edge for edge in self.graph.edges if edge["event"] == "tick")
        self.assertEqual(internal["to"], fsmview.INTERNAL_TARGET)
        self.assertEqual(self.graph.edge_ids("green", "timeout", "yellow"),
                         ["green__timeout__yellow__1"])
        self.assertEqual(self.graph.edge_ids("green", "pedestrian_button", "yellow"),
                         ["green__pedestrian_button__yellow__2"])
        self.assertEqual(self.graph.edge_ids("green", "tick", "internal_target"),
                         ["green__tick__internal_target__4"])
        self.assertEqual(self.graph.edge_ids("red", "tick", "red"), [])


class StepResolution(unittest.TestCase):
    def setUp(self):
        self.trace = fsmview.Trace(
            [fsmview.Graph("traffic_light", TRAFFIC_LIGHT_DOT), fsmview.Graph("wild", WILDCARD_DOT)],
            mapping={})

    def step(self, line):
        return self.trace.add_line(line)

    def test_initial_and_transitions(self):
        step = self.step("fsm[traffic_light_table] -> red")
        self.assertEqual((step["i"], step["prev"], step["state"], step["edges"]), (0, None, "red", []))
        step = self.step("fsm[traffic_light_table] red -(timeout)-> green")
        self.assertEqual(step["graph"], "traffic_light")
        self.assertEqual(step["state"], "green")
        self.assertEqual(step["edges"], ["red__timeout__green__0"])
        self.assertEqual(step["warnings"], [])

    def test_internal_transition_stays(self):
        self.step("fsm[traffic_light_table] red -(timeout)-> green")
        step = self.step("fsm[traffic_light_table] green -(tick)-> internal_target")
        self.assertEqual((step["prev"], step["state"]), ("green", "green"))
        self.assertEqual(step["edges"], ["green__tick__internal_target__4"])
        self.assertEqual(step["warnings"], [])

    def test_state_seeded_from_initial_without_initial_line(self):
        step = self.step("fsm[traffic_light_table] green -(timeout)-> yellow")
        self.assertIn("was in 'red'", step["warnings"][0])
        self.assertEqual(step["state"], "yellow")

    def test_any_state_resolves_to_tracked_state(self):
        step = self.step("fsm[wild_table] any_state -(kill)-> dead")
        self.assertEqual((step["prev"], step["state"]), ("a", "dead"))
        self.assertEqual(step["edges"], ["any_state__kill__dead__1"])
        self.assertEqual(step["warnings"], [])

    def test_per_source_fallback_lights_the_wildcard_edge(self):
        self.step("fsm[wild_table] any_state -(kill)-> dead")
        step = self.step("fsm[wild_table] dead -(kill)-> dead")
        self.assertEqual(step["edges"], ["any_state__kill__dead__1"])

    def test_unknown_names_are_flagged(self):
        step = self.step("fsm[traffic_light_table] red -(bogus)-> nowhere")
        self.assertTrue(any("unknown event 'bogus'" in w for w in step["warnings"]))
        self.assertTrue(any("unknown state 'nowhere'" in w for w in step["warnings"]))
        step = self.step("fsm[unknown_table] -> x")
        self.assertIsNone(step["graph"])
        self.assertIn("no graph for machine 'unknown_table'", step["warnings"])

    def test_graph_matching_order(self):
        self.assertEqual(self.trace.graph_for("wild_table").stem, "wild")  # table comment
        self.assertEqual(self.trace.graph_for("traffic_light").stem, "traffic_light")  # name
        mapped = fsmview.Trace(self.trace.graphs, mapping={"other": "wild"})
        self.assertEqual(mapped.graph_for("other").stem, "wild")

    def test_collision_warning(self):
        twice = fsmview.Trace([fsmview.Graph("a", WILDCARD_DOT), fsmview.Graph("b", WILDCARD_DOT)],
                              mapping={})
        self.assertEqual(len(twice.warnings), 1)
        self.assertIn("--map wild_table=", twice.warnings[0])
        resolved = fsmview.Trace(twice.graphs, mapping={"wild_table": "b"})
        self.assertEqual(resolved.warnings, [])
        self.assertEqual(resolved.graph_for("wild_table").stem, "b")


if __name__ == "__main__":
    unittest.main()
