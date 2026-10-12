# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Shared fixtures of the Python tests."""

from __future__ import annotations

import pytest

#: The variables through which rich forces colour or a terminal. CI runners set ``FORCE_COLOR``, and
#: the tests compare the plain text that a file receives. ``pyproject.toml`` names the fixture in
#: ``usefixtures``, so that every test uses it.
_TERMINAL_VARIABLES = ("FORCE_COLOR", "NO_COLOR", "TTY_COMPATIBLE", "TTY_INTERACTIVE", "COLUMNS", "LINES")


@pytest.fixture
def plain_terminal(monkeypatch: pytest.MonkeyPatch) -> None:
    """Run every test without the variables that change the terminal output."""
    for name in _TERMINAL_VARIABLES:
        monkeypatch.delenv(name, raising=False)
