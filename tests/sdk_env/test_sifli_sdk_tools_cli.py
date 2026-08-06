#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from types import SimpleNamespace
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import sifli_sdk_tools  # noqa: E402


class ToolCommandTemplateTests(unittest.TestCase):
    def test_run_cmd_check_output_keeps_input_command_template(self) -> None:
        cmd = ["demo", "--version"]
        tool_dir = os.path.join(tempfile.gettempdir(), "demo-tools", "bin")
        completed = SimpleNamespace(stdout=b"demo 1.0.0\n", stderr=b"")

        with mock.patch("os.path.exists", return_value=True):
            with mock.patch("subprocess.run", return_value=completed) as run:
                output = sifli_sdk_tools.run_cmd_check_output(cmd, extra_paths=[tool_dir])

        self.assertEqual(output, b"demo 1.0.0\n")
        self.assertEqual(cmd, ["demo", "--version"])
        self.assertEqual(run.call_args[0][0][0], os.path.join(tool_dir, "demo"))

    def test_get_version_keeps_tool_command_template(self) -> None:
        tool = sifli_sdk_tools.SiFliSDKTool.from_json(
            {
                "name": "demo",
                "description": "demo tool",
                "export_paths": [["bin"]],
                "export_vars": {},
                "install": "always",
                "info_url": "https://example.com",
                "license": "Apache-2.0",
                "version_cmd": ["demo", "--version"],
                "version_regex": "demo ([0-9.]+)",
                "supported_targets": ["all"],
                "versions": [],
            }
        )

        with mock.patch("sifli_sdk_tools.run_cmd_check_output", return_value=b"demo 1.0.0\n"):
            self.assertEqual(tool.get_version(executable_path="/tmp/demo"), "1.0.0")

        self.assertEqual(tool._current_options.version_cmd, ["demo", "--version"])


class LegacyCommandDisableTests(unittest.TestCase):
    def test_legacy_env_subcommands_fail_with_migration_message(self) -> None:
        legacy_actions = [
            "export",
            "install-python-env",
            "get-install-python-env",
            "check-python-dependencies",
        ]
        for action in legacy_actions:
            with self.subTest(action=action):
                err = io.StringIO()
                with redirect_stderr(err):
                    with self.assertRaises(SystemExit) as cm:
                        sifli_sdk_tools.main([action])
                self.assertEqual(cm.exception.code, 1)
                output = err.getvalue()
                self.assertIn("no longer supported", output)
                self.assertIn("./install.sh", output)

    def test_parse_tools_info_json_accepts_versions_without_status(self) -> None:
        tools_info = {
            "version": 2,
            "tools": [
                {
                    "name": "demo",
                    "description": "demo tool",
                    "export_paths": [["bin"]],
                    "export_vars": {},
                    "install": "always",
                    "info_url": "https://example.com",
                    "license": "Apache-2.0",
                    "version_cmd": ["demo", "--version"],
                    "version_regex": "([0-9.]+)",
                    "supported_targets": ["all"],
                    "versions": [
                        {
                            "name": "1.0.0",
                            "linux-amd64": {
                                "url": "https://example.com/demo-1.0.0.tar.xz",
                                "size": 123,
                                "sha256": "abc",
                            },
                        }
                    ],
                }
            ],
        }

        parsed = sifli_sdk_tools.parse_tools_info_json(tools_info)
        self.assertIn("demo", parsed)
        self.assertIn("1.0.0", parsed["demo"].versions)
        dumped = sifli_sdk_tools.dump_tools_json(parsed)
        self.assertNotIn('"status"', dumped)

    def test_legacy_install_and_download_require_explicit_version(self) -> None:
        temp_root = tempfile.mkdtemp(prefix="sdk-tools-cli-")
        self.addCleanup(lambda: shutil.rmtree(temp_root, ignore_errors=True))
        tools_json_path = os.path.join(temp_root, "tools.json")
        with open(tools_json_path, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "version": 2,
                    "tools": [
                        {
                            "name": "demo",
                            "description": "demo tool",
                            "export_paths": [["bin"]],
                            "export_vars": {},
                            "install": "always",
                            "info_url": "https://example.com",
                            "license": "Apache-2.0",
                            "version_cmd": ["demo", "--version"],
                            "version_regex": "([0-9.]+)",
                            "supported_targets": ["all"],
                            "versions": [
                                {
                                    "name": "1.0.0",
                                    "macos-arm64": {
                                        "url": "https://example.com/demo-1.0.0.tar.xz",
                                        "size": 123,
                                        "sha256": "abc",
                                    },
                                }
                            ],
                        }
                    ],
                },
                f,
            )

        for action in ["install", "download"]:
            with self.subTest(action=action):
                err = io.StringIO()
                with redirect_stderr(err):
                    with self.assertRaises(SystemExit) as cm:
                        sifli_sdk_tools.main([
                            "--tools-json",
                            tools_json_path,
                            "--sifli-sdk-path",
                            ROOT,
                            action,
                            "demo",
                        ])
                self.assertEqual(cm.exception.code, 1)
                self.assertIn("explicit version", err.getvalue())


if __name__ == "__main__":
    unittest.main()
