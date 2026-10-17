# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Checks for the west fsm_dotgen command's build directory reading:
python3 -m unittest zephyr/scripts/test_fsm_dotgen.py"""

import argparse
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fsm_dotgen  # noqa: E402


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

    def parse(self, arguments):
        return fsm_dotgen.build_parser(argparse.ArgumentParser()).parse_args(arguments)

    def test_the_build_answers_what_is_not_given(self):
        build = fsm_dotgen.ZephyrBuild(self.root, self.fail)
        args = self.parse([])
        build.supply(args)
        self.assertEqual(args.elf, ["/work/build/zephyr/zephyr.elf"])
        self.assertEqual(args.nm, "/sdk/bin/arm-zephyr-eabi-nm")
        self.assertEqual(args.paths, ["/work/app", "/work/usbc", "/work/modules/hal/cmsis"])
        self.assertEqual(args.out, str(self.root / "dot"))

    def test_given_arguments_win(self):
        build = fsm_dotgen.ZephyrBuild(self.root, self.fail)
        args = self.parse(["/work/usbc/include", "-o", "graphs"])
        build.supply(args)
        self.assertEqual(args.paths, ["/work/usbc/include"])
        self.assertEqual(args.out, "graphs")
        self.assertEqual(args.elf, ["/work/build/zephyr/zephyr.elf"])

    def test_no_build_is_refused(self):
        with self.assertRaises(SystemExit):
            fsm_dotgen.ZephyrBuild(self.root / "missing")


if __name__ == "__main__":
    unittest.main()
