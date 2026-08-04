#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import copy
import importlib.util
import json
import logging
import os
import sys
import unittest
from contextlib import redirect_stderr
from contextlib import redirect_stdout
from io import StringIO
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from sdk_py_actions import ptab_ext  # noqa: E402
from sdk_py_actions.cli.context import SdkConfig  # noqa: E402
from sdk_py_actions.cli.context import SdkContext  # noqa: E402
from sdk_py_actions.cli.registry import CommandRegistry  # noqa: E402


class FakeIssue:
    def __init__(self, severity: str) -> None:
        self.severity = severity


class FakePtab:
    chip = "SF32LB52X"

    def __init__(self, partitions: list[dict]) -> None:
        self._partitions = copy.deepcopy(partitions)

    @property
    def partitions(self) -> list[dict]:
        return copy.deepcopy(self._partitions)

    def is_v3(self) -> bool:
        return True

    def get_chip_config(self) -> dict:
        return make_chip_config()


class FakePtabModule:
    @staticmethod
    def parse_size(value):
        if isinstance(value, int):
            return value
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    @staticmethod
    def resolve_region_address(region, offset, chip_config, core=None):
        bases = {
            "mpi1": (0x60000000, 0x10000000),
            "mpi2": (0x62000000, 0x12000000),
            "psram1": (0x60000000, 0x10000000),
            "hpsys_ram": (0x20000000, 0x20000000),
        }
        sbus, cbus = bases.get(region, (0, 0))
        return sbus + offset, cbus + offset

    @staticmethod
    def print_overlay_summary(report, dedupe=True):
        return None


def make_chip_config() -> dict:
    return {
        "memory_info": {
            "mpi1": {"type": "psram", "size": 0x400, "sip": True},
            "mpi2": {"type": "nand", "size": 0x800, "sip": False},
        },
        "mpi": {
            "mpi1": {
                "base": {"offset": 0x60000000, "size": 0x400},
                "xip": {"offset": 0x10000000, "size": 0x400},
            },
            "mpi2": {
                "base": {"offset": 0x62000000, "size": 0x800},
                "xip": {"offset": 0x12000000, "size": 0x800},
            },
        },
        "ram": {
            "hpsys": {
                "ram": {
                    "ram0": {"hcpu": {"offset": 0x20000000, "size": 0x200}},
                    "ram1": {"hcpu": {"offset": 0x20000200, "size": 0x200}},
                }
            }
        },
    }


def make_prepared() -> dict:
    return {
        "base_path": "/tmp/base/ptab.yaml",
        "effective_path": "/tmp/base/ptab.yaml",
        "uses_overlay": False,
        "overlay_paths": {"chip": None, "board": None},
        "report": {"validation": [FakeIssue("warning")]},
    }


def make_partitions() -> list[dict]:
    return [
        {
            "name": "hcpu_flash_code",
            "type": "app",
            "subtype": "factory",
            "region": "mpi2",
            "offset": 0x100,
            "size": 0x100,
            "core": "HCPU",
            "exec": {"region": "psram1", "offset": 0},
        },
        {
            "name": "fs_region",
            "type": "data",
            "subtype": "filesystem",
            "region": "mpi2",
            "offset": 0x300,
            "size": 0x100,
        },
        {
            "name": "overlap_region",
            "type": "data",
            "subtype": "raw",
            "region": "mpi2",
            "offset": 0x380,
            "size": 0x80,
        },
    ]


class PtabExtTests(unittest.TestCase):
    def make_sdk_ctx(self) -> SdkContext:
        return SdkContext(
            project_dir="/tmp/project",
            build_dir="/tmp/project/build",
            env={"SIFLI_SDK_PATH": ROOT},
            logger=logging.getLogger("test-ptab-ext"),
            runner=mock.Mock(),
            config=SdkConfig(),
        )

    def build_usage_data(self) -> dict:
        return ptab_ext.build_ptab_usage_data(
            FakePtabModule,
            make_prepared(),
            FakePtab(make_partitions()),
            make_chip_config(),
            "/tmp/project",
            "board_hcpu",
            "sf32lb52x",
        )

    def test_register_adds_ptab_group_and_commands(self) -> None:
        registry = CommandRegistry(api_version=2)

        ptab_ext.register(registry)

        self.assertIn("ptab", registry._groups)
        self.assertIn("ptab/export", registry._commands)
        self.assertIn("ptab/usage", registry._commands)
        self.assertIn("ptab-export", registry._commands)
        self.assertTrue(registry._commands["ptab-export"].hidden)

    def test_usage_model_counts_storage_exec_gaps_and_overlaps(self) -> None:
        usage = self.build_usage_data()

        exec_entries = [
            entry for entry in usage["partitions"]
            if entry["kind"] == "exec" and entry["name"] == "hcpu_flash_code"
        ]
        self.assertEqual(len(exec_entries), 1)
        self.assertEqual(exec_entries[0]["region"], "psram1")
        self.assertEqual(exec_entries[0]["source_region"], "psram1")
        self.assertEqual(exec_entries[0]["address"], 0x10000000)

        regions = {region["name"]: region for region in usage["regions"]}
        self.assertEqual(regions["mpi2"]["used_bytes"], 0x200)
        self.assertEqual(regions["mpi2"]["free_bytes"], 0x600)
        self.assertEqual(regions["mpi2"]["overlap_count"], 1)
        self.assertEqual(regions["psram1"]["used_bytes"], 0x100)
        self.assertEqual(regions["psram1"]["memory_type"], "psram")

        mpi2_gaps = [gap for gap in usage["gaps"] if gap["region"] == "mpi2"]
        self.assertEqual(
            [(gap["offset"], gap["end_offset"]) for gap in mpi2_gaps],
            [(0x0, 0x100), (0x200, 0x300), (0x400, 0x800)],
        )
        self.assertEqual(len(usage["overlaps"]), 1)
        self.assertEqual(usage["validation"]["warnings"], 1)
        self.assertEqual(usage["metadata"]["chip"], "SF32LB52X")
        self.assertEqual(usage["metadata"]["chip_dir"], "sf32lb52x")

    def test_usage_model_groups_psram_mpi_aliases(self) -> None:
        usage = ptab_ext.build_ptab_usage_data(
            FakePtabModule,
            make_prepared(),
            FakePtab([
                {
                    "name": "psram_alias",
                    "type": "data",
                    "region": "mpi1",
                    "offset": 0x200,
                    "size": 0x100,
                }
            ]),
            make_chip_config(),
            "/tmp/project",
            "board_hcpu",
            "sf32lb52x",
        )

        self.assertEqual(usage["regions"][0]["name"], "psram1")
        self.assertEqual(usage["partitions"][0]["region"], "psram1")
        self.assertEqual(usage["partitions"][0]["source_region"], "mpi1")
        self.assertEqual(usage["regions"][0]["used_bytes"], 0x100)

    def test_usage_model_allows_unknown_capacity_regions(self) -> None:
        usage = ptab_ext.build_ptab_usage_data(
            FakePtabModule,
            make_prepared(),
            FakePtab([
                {
                    "name": "custom",
                    "type": "data",
                    "region": "mystery",
                    "offset": 0,
                    "size": 0x100,
                }
            ]),
            make_chip_config(),
            "/tmp/project",
            "board_hcpu",
            "sf32lb52x",
        )

        self.assertEqual(usage["regions"][0]["name"], "mystery")
        self.assertIsNone(usage["regions"][0]["total_bytes"])
        self.assertIsNone(usage["regions"][0]["free_bytes"])
        self.assertEqual(usage["regions"][0]["used_bytes"], 0x100)

    def test_usage_json_is_machine_readable_stdout(self) -> None:
        ctx = self.make_sdk_ctx()
        fake_ptab = FakePtab(make_partitions())
        context = {
            "ptab_module": FakePtabModule,
            "prepared": make_prepared(),
            "ptab_obj": fake_ptab,
            "chip_config": make_chip_config(),
            "project_dir": "/tmp/project",
            "board": "board_hcpu",
            "chip": "sf32lb52x",
        }

        output = StringIO()
        stderr = StringIO()
        with mock.patch.object(ptab_ext, "_resolve_ptab_context", return_value=context):
            with redirect_stdout(output), redirect_stderr(stderr):
                ptab_ext.ptab_usage_callback(ctx, output_format="json")

        parsed = json.loads(output.getvalue())
        self.assertEqual(parsed["schema_version"], 1)
        self.assertIn("regions", parsed)
        self.assertIn("partitions", parsed)

    def test_layout_and_table_outputs_include_expected_sections(self) -> None:
        if importlib.util.find_spec("rich") is None:
            self.skipTest("rich is not installed in this Python environment")

        usage = self.build_usage_data()

        layout_output = StringIO()
        with redirect_stdout(layout_output):
            ptab_ext.print_ptab_usage_layout(usage)
        self.assertIn("Memory Layout", layout_output.getvalue())
        self.assertIn("Region Summary", layout_output.getvalue())
        self.assertIn("Partition Usage", layout_output.getvalue())

        table_output = StringIO()
        with redirect_stdout(table_output):
            ptab_ext.print_ptab_usage_tables(usage)
        self.assertNotIn("Memory Layout", table_output.getvalue())
        self.assertIn("Region Summary", table_output.getvalue())
        self.assertIn("Partition Usage", table_output.getvalue())


if __name__ == "__main__":
    unittest.main()
