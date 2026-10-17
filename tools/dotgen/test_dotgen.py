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
import unittest.mock
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

using shared_transitions = fsm::transition_table<
    fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>>;

namespace inner::deep {
class aliased_table final : public app::shared_transitions {
public:
    static constexpr int marker = 1;
};
} // namespace inner::deep

template<typename STATE>
struct templated_table : fsm::transition_table<
    fsm::transition<fsm::from<STATE>, fsm::on<go>, fsm::to<busy>>> {};

struct not_a_table : some_base<fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>> {};
struct holds_an_alias : some_base<shared_transitions> {};

} // namespace app

namespace {
struct anonymous_table : fsm::transition_table<
    fsm::transition<fsm::from<app::idle>, fsm::on<app::go>, fsm::to<app::busy>>> {};
}
"""


NESTED_HEADER = """
#include <mtl/StateMachine.hpp>

namespace app {
struct tick {};
struct low {};
struct high {};

struct inner_table : fsm::transition_table<
    fsm::transition<fsm::from<low>, fsm::on<tick>, fsm::to<high>>> {};

// a composite state is not a table: its sub-table above is, and gets a graph of its own
struct active {
    using submachine = inner_table;
};
struct idle {};

struct outer_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>, fsm::on<tick>, fsm::to<active>>> {};
} // namespace app
"""


class NestedCrawler(unittest.TestCase):
    def test_a_sub_table_is_a_table_of_its_own(self):
        with tempfile.TemporaryDirectory() as directory:
            header = Path(directory) / "nested.hpp"
            header.write_text(NESTED_HEADER, encoding="utf-8")
            tables, has_templates = dotgen.find_tables(header)
        self.assertEqual([table.qualified for table in tables],
                         ["app::inner_table", "app::outer_table"])
        self.assertEqual(has_templates, [])


class Crawler(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.header = Path(self.directory.name) / "tables.hpp"
        self.header.write_text(HEADER, encoding="utf-8")
        self.tables, self.templates = dotgen.find_tables(self.header)

    def tearDown(self):
        self.directory.cleanup()

    def scan_messages(self, instantiated_by_image=False):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            dotgen.scan([str(self.header)], instantiated_by_image=instantiated_by_image)
        return stderr.getvalue()

    def test_named_tables_with_their_namespaces(self):
        self.assertEqual([table.qualified for table in self.tables],
                         ["app::plain_table", "app::inner::deep::aliased_table", "anonymous_table"])
        self.assertEqual([table.name for table in self.tables],
                         ["plain_table", "aliased_table", "anonymous_table"])

    def test_template_is_reported_with_its_line(self):
        [(name, line)] = self.templates
        self.assertEqual(name, "templated_table")
        self.assertIn("struct templated_table", HEADER.splitlines()[line - 1])

    def test_scan_hints_at_the_template_unless_an_image_names_it(self):
        messages = self.scan_messages()
        self.assertIn("template table 'templated_table' skipped", messages)
        self.assertIn("--table", messages)
        self.assertIn("--elf", messages)
        self.assertNotIn("skipped", self.scan_messages(instantiated_by_image=True))

    def test_line_numbers(self):
        lines = HEADER.splitlines()
        for table in self.tables:
            self.assertIn(table.name, lines[table.line - 1])

    def test_generator_names_every_table(self):
        source = dotgen.generator_source(self.tables, Path("/out"))
        self.assertIn(f'#include "{self.header}"', source)
        self.assertIn('fsm::writeDot<app::inner::deep::aliased_table>(out, "aliased_table")', source)
        self.assertIn('std::ofstream out{"/out/anonymous_table.dot"}', source)

    def test_explicit_instantiation_includes_its_header(self):
        table = dotgen.Table("app::templated_table<app::idle>", "idle_table", None, 0)
        source = dotgen.generator_source([table], Path("/out"), [self.header])
        self.assertIn(f'#include "{self.header}"', source)
        self.assertIn('fsm::writeDot<app::templated_table<app::idle>>(out, "idle_table")', source)

    def test_scan_reports_given_headers(self):
        tables, given, template_headers = dotgen.scan([str(self.header)])
        self.assertEqual(len(tables), 3)
        self.assertEqual(given, [self.header.resolve()])

    def test_scan_remembers_template_table_headers(self):
        _, _, template_headers = dotgen.scan([str(self.header)])
        self.assertEqual(template_headers, [self.header.resolve()])

    def test_a_table_derives_from_an_alias_of_another_header(self):
        named = Path(self.directory.name) / "named.hpp"
        named.write_text('#include "tables.hpp"\n'
                         "struct named_elsewhere : app::shared_transitions {};\n",
                         encoding="utf-8")
        alone, _ = dotgen.find_tables(named)
        self.assertEqual(alone, [])
        tables, _, _ = dotgen.scan([self.directory.name])
        self.assertIn("named_elsewhere", [table.name for table in tables])


SYMBOLS = """\
08001234 t void fsm::StateMachine<app::plain_table, fsm::timed<app::Timer&, 1u>, (anonymous namespace)::Policy>::processWithReaction<app::go>(app::go const&)
08001300 t fsm::StateMachine<fsm::internal::nested<app::inner_table, 1, mtl::nil_type>, fsm::timed<app::Timer&, 1u>>::leaveActiveState()
08001400 t void fsm::StateMachine<app::drp::table_for<app::default_timing, (app::preference)0>, app::Driver<app::Tcpc>>::enterState<app::idle, app::busy>()
08001500 t fsm::StateMachine<app::plain_table, fsm::timed<app::Timer&, 1u>, (anonymous namespace)::Policy>::StateMachine()
08001600 t fsm::StateMachine<fsm::internal::nested<app::vcs_table<(app::role)1>, 1, mtl::nil_type>, app::Driver<app::Tcpc>>::StateMachine()
08001700 t void fsm::timed<app::Timer&, 1u>::onEnter<app::busy, fsm::StateMachine<app::other_table, fsm::timed<app::Timer&, 1u>>>(fsm::StateMachine<app::other_table, fsm::timed<app::Timer&, 1u>>&)
08001800 t fsm::StateMachine<(anonymous namespace)::local_table>::StateMachine()
08001900 T z_impl_k_sleep
"""


class ImageTables(unittest.TestCase):
    """The tables of an image are read from its demangled symbols: the
    first argument of every fsm::StateMachine, the nested wrapper of a
    child machine unwrapped, each table once."""

    def setUp(self):
        self.tables = dotgen.machine_tables(SYMBOLS.splitlines())

    def test_root_and_child_tables_each_once(self):
        self.assertEqual(self.tables, [
            "app::plain_table",
            "app::inner_table",
            "app::drp::table_for<app::default_timing, (app::preference)0>",
            "app::vcs_table<(app::role)1>",
            "app::other_table",
            "(anonymous namespace)::local_table",
        ])

    def test_instantiations_get_distinct_names(self):
        self.assertEqual(dotgen.table_name("app::plain_table"), "plain_table")
        self.assertEqual(dotgen.table_name("app::vcs_table<(app::role)1>"), "vcs_table_role_1")
        self.assertEqual(
            dotgen.table_name("app::drp::table_for<app::default_timing, (app::preference)0>"),
            "table_for_default_timing_preference_0")

    def test_nested_argument_lists_stay_balanced(self):
        line = "fsm::StateMachine<app::table<std::pair<int, int>, app::fn<void (*)(int, int)>>, obs>"
        self.assertEqual(dotgen.machine_tables([line]),
                         ["app::table<std::pair<int, int>, app::fn<void (*)(int, int)>>"])

    def test_built_tables_take_the_scanned_header_and_skip_anonymous(self):
        scanned = [dotgen.Table("app::plain_table", "plain_table", Path("/src/tables.hpp"), 30)]
        messages = []
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), \
                unittest.mock.patch.object(dotgen, "image_tables", return_value=self.tables):
            tables = dotgen.built_tables(["image.elf"], "nm", scanned, messages.append, self.fail)
        self.assertEqual([table.qualified for table in tables], self.tables[:-1])
        self.assertEqual(tables[0].header, Path("/src/tables.hpp"))
        self.assertIsNone(tables[2].header)
        self.assertIn("local_table skipped", stderr.getvalue())


class BuildDirectory(unittest.TestCase):
    """A Zephyr build directory answers the arguments: the image and
    the nm from CMakeCache.txt, the trees from the application and the
    module list."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        (self.root / "CMakeCache.txt").write_text(
            "APPLICATION_SOURCE_DIR:PATH=/work/app\n"
            "BYPRODUCT_KERNEL_ELF_NAME:FILEPATH=/work/build/zephyr/zephyr.elf\n"
            "CMAKE_NM:FILEPATH=/sdk/bin/arm-zephyr-eabi-nm\n"
            "CMAKE_NM-ADVANCED:INTERNAL=1\n", encoding="utf-8")
        (self.root / "zephyr_modules.txt").write_text(
            '"usb-typec":"/work/usbc":"/work/usbc/zephyr"\n'
            '"cmsis":"/work/modules/hal/cmsis":"${ZEPHYR_CMSIS_CMAKE_DIR}"\n', encoding="utf-8")

    def tearDown(self):
        self.directory.cleanup()

    def test_the_build_answers_what_is_not_given(self):
        build = dotgen.ZephyrBuild(self.root, self.fail)
        args = dotgen.build_parser().parse_args([])
        build.supply(args)
        self.assertEqual(args.elf, ["/work/build/zephyr/zephyr.elf"])
        self.assertEqual(args.nm, "/sdk/bin/arm-zephyr-eabi-nm")
        self.assertEqual(args.paths, ["/work/app", "/work/usbc", "/work/modules/hal/cmsis"])
        self.assertEqual(args.out, str(self.root / "dot"))

    def test_given_arguments_win(self):
        build = dotgen.ZephyrBuild(self.root, self.fail)
        args = dotgen.build_parser().parse_args(["/work/usbc/include", "-o", "graphs"])
        build.supply(args)
        self.assertEqual(args.paths, ["/work/usbc/include"])
        self.assertEqual(args.out, "graphs")
        self.assertEqual(args.elf, ["/work/build/zephyr/zephyr.elf"])

    def test_no_build_is_refused(self):
        with self.assertRaises(SystemExit):
            dotgen.ZephyrBuild(self.root / "missing")


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
