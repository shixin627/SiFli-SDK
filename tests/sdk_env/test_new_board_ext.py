#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import logging
import os
import sys
import unittest
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from sdk_py_actions import new_board_ext  # noqa: E402
from sdk_py_actions.cli.context import SdkConfig  # noqa: E402
from sdk_py_actions.cli.context import SdkContext  # noqa: E402
from sdk_py_actions.cli.registry import CommandRegistry  # noqa: E402


class NewBoardExtTests(unittest.TestCase):
    def make_sdk_ctx(self) -> SdkContext:
        return SdkContext(
            project_dir="/tmp/project",
            build_dir="/tmp/project/build",
            env={"SIFLI_SDK_PATH": "/tmp/sdk"},
            logger=logging.getLogger("test-new-board-ext"),
            runner=mock.Mock(),
            config=SdkConfig(),
        )

    def test_register_adds_new_board_command(self) -> None:
        registry = CommandRegistry(api_version=2)

        new_board_ext.register(registry)

        self.assertIn("new-board", registry._commands)

    def test_new_board_callback_passes_explicit_output_dir(self) -> None:
        sdk_ctx = self.make_sdk_ctx()

        with mock.patch.object(new_board_ext, "run_new_board") as run_new_board:
            new_board_ext.new_board_callback(
                sdk_ctx,
                config_path="/tmp/new-board.json",
                output_dir="/tmp/output",
            )

        run_new_board.assert_called_once_with(
            config_path="/tmp/new-board.json",
            output_root="/tmp/output",
        )

    def test_new_board_callback_defaults_to_current_working_directory(self) -> None:
        sdk_ctx = self.make_sdk_ctx()

        with mock.patch.object(new_board_ext, "run_new_board") as run_new_board:
            with mock.patch("os.getcwd", return_value="/tmp/current-dir"):
                new_board_ext.new_board_callback(sdk_ctx, config_path=None, output_dir=None)

        run_new_board.assert_called_once_with(
            config_path=None,
            output_root="/tmp/current-dir",
        )


if __name__ == "__main__":
    unittest.main()
