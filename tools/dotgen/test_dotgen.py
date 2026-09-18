# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Checks for the table crawler and the include search of dotgen.py:
python3 -m unittest tools/dotgen/test_dotgen.py"""

import contextlib
import io
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dotgen  # noqa: E402

HEADER = """
// struct commented_out : fsm::transition_table<> {};
/* struct in_block_comment : fsm::transition_table<> {}; */
#include <mtl/StateMachine.hpp>

namespace app {

struct go {};
struct idle {};
struct busy { char const* text = "struct fake : fsm::transition_table<> {"; };

struct plain_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>> {};

namespace inner::deep {
class rebound_table final : public mtl::rebind_t<mtl::typelist<
        fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>>,
    fsm::transition_table> {
public:
    static constexpr int marker = 1;
};
} // namespace inner::deep

template<typename STATE>
struct templated_table : fsm::transition_table<
    fsm::transition<fsm::from<STATE>, fsm::on<go>, fsm::to<busy>>> {};

struct not_a_table : some_base<fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>> {};

} // namespace app

namespace {
struct anonymous_table : fsm::transition_table<
    fsm::transition<fsm::from<app::idle>, fsm::on<app::go>, fsm::to<app::busy>>> {};
}
"""


class Crawler(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.header = Path(self.directory.name) / "tables.hpp"
        self.header.write_text(HEADER, encoding="utf-8")
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.tables = dotgen.find_tables(self.header)
        self.messages = stderr.getvalue()

    def tearDown(self):
        self.directory.cleanup()

    def test_named_tables_with_their_namespaces(self):
        self.assertEqual([table.qualified for table in self.tables],
                         ["app::plain_table", "app::inner::deep::rebound_table", "anonymous_table"])
        self.assertEqual([table.name for table in self.tables],
                         ["plain_table", "rebound_table", "anonymous_table"])

    def test_template_is_skipped_with_a_hint(self):
        self.assertIn("template table 'templated_table' skipped", self.messages)
        self.assertIn("--table", self.messages)

    def test_line_numbers(self):
        lines = HEADER.splitlines()
        for table in self.tables:
            self.assertIn(table.name, lines[table.line - 1])

    def test_generator_names_every_table(self):
        source = dotgen.generator_source(self.tables, Path("/out"))
        self.assertIn(f'#include "{self.header}"', source)
        self.assertIn('fsm::writeDot<app::inner::deep::rebound_table>(out, "rebound_table")', source)
        self.assertIn('std::ofstream out{"/out/anonymous_table.dot"}', source)

    def test_explicit_instantiation_includes_its_header(self):
        table = dotgen.Table("app::templated_table<app::idle>", "idle_table", None, 0)
        source = dotgen.generator_source([table], Path("/out"), [self.header])
        self.assertIn(f'#include "{self.header}"', source)
        self.assertIn('fsm::writeDot<app::templated_table<app::idle>>(out, "idle_table")', source)

    def test_scan_reports_given_headers(self):
        tables, given = dotgen.scan([str(self.header)])
        self.assertEqual(len(tables), 3)
        self.assertEqual(given, [self.header.resolve()])


class IncludeSearch(unittest.TestCase):
    """The headers a table header includes are found in the tree and
    where they live becomes an -I, so they need not be named by hand."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.write("project/tables.hpp",
                   "#include <lib/Units.hpp>\n"
                   '#include "local.hpp"\n'
                   "#include <cstdint>\n"
                   "// #include <commented/Out.hpp>\n")
        self.write("project/local.hpp", "")
        self.write("vendor/libfoo/include/lib/Units.hpp", "#include <deep/Deep.hpp>\n")
        self.write("vendor/libbar/include/deep/Deep.hpp", "")
        self.write("target_libc/include/cstdint", "")  # a freestanding libc must not win
        self.write("build/include/lib/Units.hpp", "")  # build output, never searched
        self.system = self.write("system/cstdint", "").parent

    def tearDown(self):
        self.directory.cleanup()

    def write(self, relative, text):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        return path

    def resolve(self):
        return dotgen.resolve_includes([self.root / "project" / "tables.hpp"],
                                       [self.root / "project"], [self.root], [self.system])

    def test_angle_include_adds_where_the_header_lives(self):
        self.assertIn(self.root / "vendor" / "libfoo" / "include", self.resolve())

    def test_follows_the_found_header_into_its_own_includes(self):
        self.assertIn(self.root / "vendor" / "libbar" / "include", self.resolve())

    def test_headers_the_compiler_has_are_left_to_it(self):
        self.assertNotIn(self.root / "target_libc" / "include", self.resolve())

    def test_quoted_neighbour_and_comment_add_nothing(self):
        self.assertEqual(len(self.resolve()), 2)

    def test_build_directories_are_not_searched(self):
        roots = dotgen.include_roots([self.root])
        self.assertIn(self.root / "vendor" / "libfoo" / "include", roots)
        self.assertNotIn(self.root / "build" / "include", roots)


if __name__ == "__main__":
    unittest.main()
