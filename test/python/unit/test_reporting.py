# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the bridge between a stage and the Python object it reports to.

A stage runs without the GIL and takes it back for every call to the reporter. An exception the
reporter raises has to leave the stage, through the solver where a solve is running, and reach the
caller unchanged, and the core has to work as before afterwards.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from mqt.scpd import pyscpd
from mqt.scpd.chip import load_chip
from mqt.scpd.config import load_config, write_config

CONFIG = Path(__file__).resolve().parents[2] / "fixtures" / "mini" / "config.toml"


class Raising:
    """A reporter that raises an exception when a stage reports progress on one task."""

    def __init__(self, task: str, error: BaseException) -> None:
        """Raise ``error`` on progress of ``task``."""
        self.task = task
        self.error = error
        self.raised = False

    @staticmethod
    def line(level: int, text: str) -> None:
        """Drop a line."""
        del level, text

    @staticmethod
    def entry(level: int, label: str, figures: list[tuple[str, str]]) -> None:
        """Drop an entry."""
        del level, label, figures

    @staticmethod
    def result(figures: list[tuple[str, str]], fails: int | None) -> None:
        """Drop the result."""
        del figures, fails

    def progress(self, task: str, detail: str, done: int, total: int) -> None:
        """Raise the error of the reporter on progress of its task."""
        del detail, done, total
        if task == self.task:
            self.raised = True
            raise self.error


@pytest.fixture(scope="module")
def inputs() -> tuple[bytes, bytes, bytes, bytes]:
    """The classified fixture, its configuration and its first two artifacts.

    Returns:
        The chip, the configuration, the capacity plan and the global routing.
    """
    config = load_config(CONFIG)
    chip = load_chip(config, CONFIG)
    packed = write_config(config)
    capacity = pyscpd.plan_capacity(chip, packed, "test")
    routing = pyscpd.route_global(chip, capacity, packed, "test")
    return chip, packed, capacity, routing


def test_an_interrupt_during_a_solve_reaches_python(inputs: tuple[bytes, bytes, bytes, bytes]) -> None:
    """Ctrl-C while HiGHS solves stops the solve and reaches the caller, and the next solve is as before."""
    chip, packed, capacity, routing = inputs
    untouched = pyscpd.assign(chip, capacity, routing, packed, "test")
    reporter = Raising("solving the assignment", KeyboardInterrupt())

    with pytest.raises(KeyboardInterrupt):
        pyscpd.assign(chip, capacity, routing, packed, "test", reporter, 0)

    assert reporter.raised
    assert pyscpd.assign(chip, capacity, routing, packed, "test") == untouched


def test_an_error_of_the_reporter_leaves_a_stage_unchanged(inputs: tuple[bytes, bytes, bytes, bytes]) -> None:
    """An error the reporter raises outside a solve reaches the caller with its type and message."""
    chip, packed, capacity, _ = inputs

    with pytest.raises(ValueError, match="the reporter broke"):
        pyscpd.plan_capacity(chip, packed, "test", Raising("distance transform", ValueError("the reporter broke")), 0)

    assert pyscpd.plan_capacity(chip, packed, "test") == capacity
