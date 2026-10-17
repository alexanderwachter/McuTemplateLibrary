#!/usr/bin/env python3
# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""west fsm_dotgen: the graphs of the state machines in a Zephyr image.

The generic generator, tools/dotgen/dotgen.py, takes the headers to
scan, the image to read the machines from and the nm reading it. In a
west workspace the build directory knows all of that already, so this
command lets it answer every argument not given - its image is the
--elf, its toolchain's nm the --nm, its application and module trees
the PATHs, <build>/dot the output - and a bare "west fsm_dotgen" after
"west build" draws the machines of the image just built:

  west fsm_dotgen [-d BUILD_DIR] [PATH...] [--elf IMAGE] [-o DIR] ...

-d names the build directory; without it the configured one is taken,
as for west build, when it holds a build and neither PATH nor --elf is
given. Every other argument is dotgen.py's.
"""

import argparse
import re
import sys
from pathlib import Path

MTL_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(MTL_ROOT / "tools" / "dotgen"))
import dotgen  # noqa: E402

CACHE_ENTRY_RE = re.compile(r"^(\w+):\w+=(.*)$", re.MULTILINE)
MODULE_LINE_RE = re.compile(r'^"([^"]*)":"([^"]*)":"([^"]*)"$', re.MULTILINE)


class ZephyrBuild:
    """What a Zephyr build directory knows: the image it built, the nm
    of its toolchain, and the application and modules the image was
    built from - the trees declaring its tables."""

    def __init__(self, directory, fail=sys.exit):
        self.directory = Path(directory)
        cache = self.directory / "CMakeCache.txt"
        if not cache.is_file():
            fail(f"dotgen: {self.directory} is no build directory: no CMakeCache.txt in it")
        entries = dict(CACHE_ENTRY_RE.findall(cache.read_text(encoding="utf-8", errors="replace")))
        self.elf = entries.get("BYPRODUCT_KERNEL_ELF_NAME")
        if not self.elf:
            fail(f"dotgen: {self.directory} names no image (BYPRODUCT_KERNEL_ELF_NAME): "
                 f"not a Zephyr build?")
        self.nm = entries.get("CMAKE_NM")
        modules = self.directory / "zephyr_modules.txt"
        module_dirs = []
        if modules.is_file():
            module_dirs = [path for _, path, _ in
                           MODULE_LINE_RE.findall(modules.read_text(encoding="utf-8"))]
        application = entries.get("APPLICATION_SOURCE_DIR")
        self.sources = [path for path in [application] + module_dirs if path]
        self.out = self.directory / "dot"

    def supply(self, args):
        """The build's answers for every argument not given."""
        args.elf = args.elf or [self.elf]
        args.nm = args.nm or self.nm
        args.paths = args.paths or self.sources
        args.out = args.out or str(self.out)


def build_parser(parser):
    parser.add_argument("-d", "--build-dir", metavar="DIR",
                        help="the Zephyr build directory answering what is not given: its "
                             "image is the --elf, its toolchain's nm the --nm, its "
                             "application and modules the PATHs, DIR/dot the output "
                             "(default: the configured build directory, as for west build, "
                             "when it holds a build and neither PATH nor --elf is given)")
    return dotgen.build_parser(parser)


try:
    from west.commands import WestCommand
except ImportError:  # the tests import this file without west
    WestCommand = None

if WestCommand is not None:

    class DotGen(WestCommand):
        def __init__(self):
            super().__init__(
                "fsm_dotgen",
                "write Graphviz DOT graphs of fsm transition tables",
                __doc__ + "\n" + dotgen.__doc__,
                requires_workspace=False,
            )

        def do_add_parser(self, parser_adder):
            parser = parser_adder.add_parser(
                self.name, help=self.help, description=self.description,
                formatter_class=argparse.RawDescriptionHelpFormatter)
            return build_parser(parser)

        def do_run(self, args, unknown_args):
            if args.build_dir is None and not args.elf and not args.paths:
                args.build_dir = self.configured_build_dir()
            if args.build_dir:
                build = ZephyrBuild(args.build_dir, self.die)
                self.inf(f"dotgen: build directory {build.directory}: image {build.elf}, "
                         f"declarations under {', '.join(build.sources)}")
                build.supply(args)
            dotgen.run(args, say=self.inf, fail=self.die)

        def configured_build_dir(self):
            """The build directory west build would use when its
            configured format needs no arguments, if it holds a build."""
            fmt = self.config.get("build.dir-fmt", "build")
            directory = Path("build" if "{" in fmt else fmt)
            return str(directory) if (directory / "CMakeCache.txt").is_file() else None
