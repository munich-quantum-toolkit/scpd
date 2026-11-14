# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The planning steps on the benchmark chips of planar-superconducting-pd.

Each chip is planned once, with its configuration in the data repository, and the plan is checked
against what the later stages rely on: every launcher of the chip has a slot, and the plan can be
drawn. The tests run when ``MQT_SCPD_BENCHMARKS`` names a clone of the data repository.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING, TypeVar

import pytest

from mqt.scpd.artifacts import read_artifact
from mqt.scpd.chip import decode_chip, role_name
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.planning import PLANNING_STAGES, planning_geometry
from mqt.scpd.plot import layout_svg
from mqt.scpd.run import BlockReporter, RunDirectory
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Sequence

    from mqt.scpd.console import Figure

DATA = Path(os.environ["MQT_SCPD_BENCHMARKS"]) if "MQT_SCPD_BENCHMARKS" in os.environ else None
CHIPS = sorted(path.parent.name for path in DATA.glob("inputs/*/config.toml")) if DATA else []

pytestmark = pytest.mark.skipif(DATA is None, reason="MQT_SCPD_BENCHMARKS names no clone of the benchmark data")

#: The largest SVG a planning step of a benchmark chip may take, in bytes.
SVG_LIMIT = 10_000_000

T = TypeVar("T")


def entries(items: Sequence[T | None] | None) -> list[T]:
    """The entries of a schema list that are there.

    The generated object model types every list as optional, and every entry with it.

    Returns:
        The entries, without the absent ones.
    """
    return [item for item in items or [] if item is not None]


class Fails:
    """A stage block that keeps the fails each stage reports."""

    def __init__(self) -> None:
        """Start with no stage reported."""
        self.counts: list[int | None] = []

    @staticmethod
    def line(level: int, text: str) -> None:
        """Drop a line."""
        del level, text

    @staticmethod
    def entry(level: int, label: str, figures: Sequence[Figure]) -> None:
        """Drop an entry."""
        del level, label, figures

    def result(self, figures: Sequence[Figure], fails: int | None = None) -> None:
        """Keep the fails of the stage."""
        del figures
        self.counts.append(fails)

    @staticmethod
    def progress(task: str, detail: str, done: int, total: int) -> None:
        """Drop the progress."""
        del task, detail, done, total


@pytest.mark.parametrize("chip", CHIPS)
def test_a_chip_plans_into_a_plan_the_later_stages_can_follow(chip: str, tmp_path: Path) -> None:
    """Every step runs, every launcher of the chip has a slot, and every planning stage can be drawn."""
    assert DATA is not None
    directory = RunDirectory(tmp_path / chip)
    config = directory.prepare(DATA / "inputs" / chip / "config.toml")
    chip_bytes = directory.classified_chip(config)
    fails = Fails()
    for step in STEPS:
        directory.run_stage(step, config, chip_bytes, BlockReporter(fails))
    assert fails.counts == [None] * len(STEPS)

    model = decode_chip(chip_bytes)
    plan = read_artifact(directory.artifact("capacity").read_bytes()).output
    assert isinstance(plan, CapacityPlanT)
    launchers = [index for index, port in enumerate(entries(model.ports)) if role_name(port.role) == "launcher"]
    assert sorted(slot.port.index for slot in entries(plan.launchers)) == launchers

    for stage in PLANNING_STAGES:
        geometry = planning_geometry(directory.artifact(stage).read_bytes(), model, stage)
        assert not geometry.is_empty(), stage
        assert len(layout_svg(model, planning=geometry).encode("utf-8")) < SVG_LIMIT, stage
        if HAS_KLAYOUT:
            from mqt.scpd.export import write_layout  # ruff: ignore[import-outside-top-level]  # needs the optional dependency

            summary = write_layout(model, tmp_path / f"{chip}-{stage}.gds", planning=geometry)
            assert summary.path.stat().st_size > 0
