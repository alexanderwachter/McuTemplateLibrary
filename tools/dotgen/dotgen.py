#!/usr/bin/env python3
# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Write Graphviz DOT graphs of the fsm transition tables in a source tree.

Scans headers for named tables (struct X : fsm::transition_table<...>,
also through mtl::rebind_t), generates a C++ program that includes the
headers and calls fsm::writeDot for every table, builds it with the host
C++ compiler against the library headers, runs it, and writes one
<table>.dot per table - the graphs tools/fsmview loads.

The table headers must compile on the host: keep them free of target
headers (put kernel calls behind a declared function, as
zephyr/samples/traffic_light/src/traffic_light.hpp does). Templated
tables cannot be found by scanning; name an instantiation with --table.

Runs as "west fsm_dotgen" (extension command of the mtl Zephyr module) or directly:
  python3 tools/dotgen/dotgen.py [PATH...] [-I DIR]... [-o DIR] [--table NS::TABLE[=NAME]]...
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

MTL_ROOT = Path(__file__).resolve().parents[2]
HEADER_SUFFIXES = {".hpp", ".h", ".hh", ".hxx"}

COMMENT_RE = re.compile(r"//[^\n]*|/\*.*?\*/", re.DOTALL)
STRING_RE = re.compile(r'"(?:[^"\\\n]|\\.)*"')
# a struct/class head up to its opening brace, base clause included
STRUCT_RE = re.compile(r"\b(struct|class)\s+(\w+)\s*(?:final\s*)?:\s*([^{;]*?)\{", re.DOTALL)
NAMESPACE_RE = re.compile(r"\bnamespace\s+((?:\w+(?:::)?)+|(?=\{))\s*\{")
TEMPLATE_RE = re.compile(r"template\s*<[^;{]*>\s*$", re.DOTALL)


class Table:
    def __init__(self, qualified, name, header, line):
        self.qualified = qualified  # how the generator names the type
        self.name = name            # struct name = machine id and file stem
        self.header = header
        self.line = line


def strip_code(text):
    """Comments and string literals blanked, positions preserved."""
    def blank(match):
        return re.sub(r"[^\n]", " ", match.group(0))
    return STRING_RE.sub(blank, COMMENT_RE.sub(blank, text))


def find_tables(header):
    """Named, non-template transition tables declared in a header."""
    text = strip_code(header.read_text(encoding="utf-8", errors="replace"))
    tables = []
    # namespace tracking: a stack of (name, brace depth at its opening)
    stack = []
    depth = 0
    position = 0
    while True:
        brace = re.compile(r"[{}]").search(text, position)
        if not brace:
            break
        if brace.group(0) == "{":
            head = text[position:brace.end()]
            namespace = NAMESPACE_RE.search(head)
            struct = STRUCT_RE.search(head)
            if namespace and namespace.end() == len(head):
                stack.append((namespace.group(1), depth))
            elif struct and struct.end() == len(head):
                if "fsm::transition_table" in struct.group(3):
                    before = text[:position + struct.start()]
                    if TEMPLATE_RE.search(before[-400:]):
                        line = before.count("\n") + 1
                        print(f"dotgen: {header}:{line}: template table '{struct.group(2)}' "
                              f"skipped - name an instantiation with --table", file=sys.stderr)
                    else:
                        scopes = [name for name, _ in stack if name]
                        qualified = "::".join(scopes + [struct.group(2)])
                        tables.append(Table(qualified, struct.group(2), header,
                                            before.count("\n") + 1))
            depth += 1
        else:
            depth -= 1
            if stack and stack[-1][1] == depth:
                stack.pop()
        position = brace.end()
    return tables


def scan(paths, say=lambda message: None):
    """Tables found under the paths, and the headers named directly (the
    generator includes those even without a found table: --table needs
    the header declaring the template)."""
    headers = []
    given = []
    for path in paths:
        path = Path(path)
        if path.is_dir():
            found = sorted(p for p in path.rglob("*") if p.suffix in HEADER_SUFFIXES)
            say(f"dotgen: scanning {len(found)} header(s) under {path}")
            headers.extend(found)
        elif path.suffix in HEADER_SUFFIXES:
            say(f"dotgen: scanning {path}")
            headers.append(path)
            given.append(path.resolve())
        else:
            print(f"dotgen: {path}: not a header, skipped (tables must live in headers)",
                  file=sys.stderr)
    tables = []
    for header in headers:
        for table in find_tables(header.resolve()):
            say(f"dotgen: {table.header}:{table.line}: {table.qualified}")
            tables.append(table)
    return tables, given


def generator_source(tables, out_dir, headers=()):
    lines = ["#include <mtl/StateMachineDot.hpp>"]
    included = {str(table.header) for table in tables if table.header}
    included.update(str(header) for header in headers)
    for header in sorted(included):
        lines.append(f'#include "{header}"')
    lines += ["#include <fstream>", "#include <iostream>", "", "int main()", "{"]
    for table in tables:
        path = (out_dir / f"{table.name}.dot").as_posix()
        lines += [
            "    {",
            f'        std::ofstream out{{"{path}"}};',
            f'        fsm::writeDot<{table.qualified}>(out, "{table.name}");',
            f'        std::cout << "{path}\\n";',
            "    }",
        ]
    lines += ["    return 0;", "}", ""]
    return "\n".join(lines)


def build_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser(
            description="Write Graphviz DOT graphs of the fsm transition tables in a source tree",
            formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("paths", nargs="*", metavar="PATH", default=["."],
                        help="headers or directories to scan for tables (default: .)")
    parser.add_argument("-I", dest="include_dirs", action="append", default=[], metavar="DIR",
                        help="extra include directory for the table headers")
    parser.add_argument("-D", dest="defines", action="append", default=[], metavar="MACRO[=VALUE]",
                        help="preprocessor definition for the table headers")
    parser.add_argument("-t", "--table", action="append", default=[], metavar="NS::TABLE[=NAME]",
                        help="also render this type (e.g. a template instantiation); "
                             "NAME is the file stem and graph title (default: the last name part)")
    parser.add_argument("-o", "--out", default="dot", metavar="DIR",
                        help="output directory, one <table>.dot per table (default: dot)")
    parser.add_argument("--cxx", default=os.environ.get("CXX") or shutil.which("c++") or "c++",
                        help="host C++ compiler (default: $CXX, or c++)")
    parser.add_argument("--std", default="c++20", help="language standard (default: c++20)")
    parser.add_argument("--keep", action="store_true",
                        help="keep the generated program in the output directory")
    return parser


def run(args, say=print, fail=sys.exit):
    tables, given_headers = scan(args.paths, say)
    for spec in args.table:
        qualified, _, name = spec.partition("=")
        table = Table(qualified, name or qualified.split("<")[0].split("::")[-1], None, 0)
        say(f"dotgen: --table: {table.qualified} as {table.name}")
        tables.append(table)
    if not tables:
        fail("dotgen: no transition table found (tables must be named structs deriving from "
             "fsm::transition_table, declared in headers)")
    names = {}
    for table in tables:
        if table.name in names:
            fail(f"dotgen: two tables named '{table.name}': {names[table.name]} and "
                 f"{table.qualified} - rename one, or pass --table {table.qualified}=<name>")
        names[table.name] = table.qualified

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    include_dirs = [MTL_ROOT / "include"] + [Path(d).resolve() for d in args.include_dirs]
    headers = {table.header for table in tables if table.header} | set(given_headers)
    include_dirs += sorted({header.parent for header in headers})
    if args.table and not headers:
        fail("dotgen: --table needs the header declaring the type: pass the header as PATH")

    with tempfile.TemporaryDirectory(prefix="dotgen-") as temp:
        source = Path(args.keep and out_dir or temp) / "dotgen.cpp"
        say(f"dotgen: generating {source} for {len(tables)} table(s), "
            f"including {len(headers)} header(s)")
        source.write_text(generator_source(tables, out_dir, headers), encoding="utf-8")
        program = Path(temp) / "dotgen"
        command = [args.cxx, f"-std={args.std}"]
        command += [f"-I{directory}" for directory in include_dirs]
        command += [f"-D{define}" for define in args.defines]
        command += [str(source), "-o", str(program)]
        say(f"dotgen: compiling with {args.cxx} -std={args.std} "
            + " ".join(f"-I{directory}" for directory in include_dirs))
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            sys.stderr.write(result.stderr)
            fail(f"dotgen: compiling the generator failed: {' '.join(command)}")
        say(f"dotgen: running the generator, writing into {out_dir}")
        result = subprocess.run([str(program)], capture_output=True, text=True)
        if result.returncode != 0:
            sys.stderr.write(result.stderr)
            fail("dotgen: the generator failed")
        for line in result.stdout.splitlines():
            say(f"dotgen: wrote {line}")
        say(f"dotgen: done, {len(tables)} graph(s) in {out_dir}")


def main(argv=None):
    run(build_parser().parse_args(argv), say=lambda message: print(message, file=sys.stderr))


try:
    from west.commands import WestCommand
except ImportError:  # standalone use without west
    WestCommand = None

if WestCommand is not None:

    class DotGen(WestCommand):
        def __init__(self):
            super().__init__(
                "fsm_dotgen",
                "write Graphviz DOT graphs of fsm transition tables",
                __doc__,
                requires_workspace=False,
            )

        def do_add_parser(self, parser_adder):
            parser = parser_adder.add_parser(
                self.name, help=self.help, description=self.description,
                formatter_class=argparse.RawDescriptionHelpFormatter)
            return build_parser(parser)

        def do_run(self, args, unknown_args):
            run(args, say=self.inf, fail=self.die)


if __name__ == "__main__":
    main()
