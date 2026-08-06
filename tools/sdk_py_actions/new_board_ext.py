# SPDX-FileCopyrightText: 2025 SiFli Technologies Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""Interactive/new-config board generator for sdk.py."""

from __future__ import annotations

import os
from typing import Optional

import click

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.new_board import run_new_board

EXTENSION_ID = "new_board"
EXTENSION_VERSION = "2.1.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None

def new_board_callback(
    _sdk_ctx: SdkContext,
    config_path: Optional[str] = None,
    output_dir: Optional[str] = None,
) -> None:
    run_new_board(
        config_path=config_path,
        output_root=output_dir or os.getcwd(),
    )


def register(registry: CommandRegistry) -> None:
    registry.command(
        path="new-board",
        callback=new_board_callback,
        help="Create a new board from interactive prompts or a JSON config file.",
        options=[
            {
                "names": ["--config", "-c", "config_path"],
                "help": "Path to a JSON config file for non-interactive board creation.",
                "type": click.Path(exists=True, dir_okay=False, resolve_path=False),
                "default": None,
            },
            {
                "names": ["--output-dir", "-o"],
                "help": "Directory where the board/base directories will be created. Defaults to the current terminal directory.",
                "type": click.Path(exists=False, file_okay=False, dir_okay=True, resolve_path=False),
                "default": None,
            },
        ],
    )
