# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The planning steps on the benchmark chips of planar-superconducting-pd.

Each chip is planned once, with its configuration in the data repository, and the plan is checked
against what the later stages rely on: every inner port is reached and every ring port has a
launcher. The tests run when ``MQT_SCPD_BENCHMARKS`` names a clone of the data repository.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING, TypeVar

import pytest

from mqt.scpd.artifacts import read_artifact
from mqt.scpd.chip import decode_chip, role_name
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
from mqt.scpd.flatbuffers.design.AssignedRole import AssignedRole
from mqt.scpd.planning import PLANNING_STAGES, planning_geometry
from mqt.scpd.plot import layout_svg
from mqt.scpd.run import BlockReporter, RunDirectory
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

    from mqt.scpd.console import Figure
    from mqt.scpd.flatbuffers.design.Chip import ChipT

DATA = Path(os.environ["MQT_SCPD_BENCHMARKS"]) if "MQT_SCPD_BENCHMARKS" in os.environ else None
CHIPS = sorted(path.parent.name for path in DATA.glob("inputs/*/config.toml")) if DATA else []

pytestmark = pytest.mark.skipif(DATA is None, reason="MQT_SCPD_BENCHMARKS names no clone of the benchmark data")

#: The objective of the assignment on the chips that are small enough to know it by heart.
OBJECTIVES = {"4q": 5.02, "9q": 15.27}

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


def indices(values: Iterable[int] | None) -> list[int]:
    """The entries of a schema vector of indices, as integers.

    The generated object model reads a vector of numbers into a NumPy array when NumPy is
    importable, and into a list otherwise. A NumPy array has no single truth value, so the vector is
    read into a list, never tested for truth.

    Returns:
        The indices, or none when the vector is absent.
    """
    return [] if values is None else [int(value) for value in values]


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


def check_the_assignment(chip: ChipT, assignment: AssignmentT, launcher_target: int, terminations: int) -> None:
    """Every ring port has a launcher, no two conventional ports share one, and the chains add up.

    A resonator of the assignment is a resonator port of the chip, or a ring port where the wire of
    an inner resonator surfaces; its connection says which.
    """
    ports = entries(chip.ports)
    ring = [entry.index for entry in entries(assignment.ring)]
    connections = entries(assignment.connections)
    assert len(connections) == len(ring)
    resonators = {
        node for node, connection in enumerate(connections) if connection.sourceRole == AssignedRole.ResonatorSource
    }
    assert all(role_name(ports[ring[node]].role) in {"resonator", "bridge_pair"} for node in resonators)
    used = [
        connection.source.index
        for node, connection in enumerate(connections)
        if node not in resonators and connection.source is not None
    ]
    assert len(used) == len(ring) - len(resonators)
    assert len(set(used)) == len(used), "two conventional ports share a launcher"
    feeds = [(feed.x, feed.y) for feed in entries(assignment.feeds)]
    assert len(set(feeds)) == len(feeds), "two ring nodes share a feed point"

    chained: list[int] = []
    ends = {"launcher": 0, "termination": 0}
    for chain in entries(assignment.chains):
        chained.extend(indices(chain.nodes))
        for end in (chain.start, chain.end):
            ends["launcher" if end is not None else "termination"] += 1
    assert sorted(chained) == sorted(resonators), "a resonator is in no chain or in two"
    assert ends == {"launcher": launcher_target, "termination": terminations}


@pytest.mark.parametrize("chip", CHIPS)
def test_a_chip_plans_into_a_plan_the_later_stages_can_follow(chip: str, tmp_path: Path) -> None:
    """Every step runs without a fail, and the plan keeps the rules the later stages rely on."""
    assert DATA is not None
    directory = RunDirectory(tmp_path / chip)
    config = directory.prepare(DATA / "inputs" / chip / "config.toml")
    chip_bytes = directory.classified_chip(config)
    fails = Fails()
    for step in STEPS:
        directory.run_stage(step, config, chip_bytes, BlockReporter(fails))
    assert fails.counts == [None, 0, 0]

    model = decode_chip(chip_bytes)
    assignment = read_artifact(directory.artifact("assign").read_bytes()).output
    assert isinstance(assignment, AssignmentT)
    assert config.stages is not None
    assert config.stages.assignment is not None
    assert config.rules is not None
    check_the_assignment(model, assignment, config.stages.assignment.launcherTarget, config.rules.feedlineTerminations)
    if chip in OBJECTIVES:
        assert assignment.objective == pytest.approx(OBJECTIVES[chip], abs=1e-9)

    capacity = directory.artifact("capacity").read_bytes()
    for stage in PLANNING_STAGES:
        geometry = planning_geometry(directory.artifact(stage).read_bytes(), model, stage, capacity)
        assert not geometry.is_empty(), stage
        assert len(layout_svg(model, planning=geometry).encode("utf-8")) < SVG_LIMIT, stage
        if HAS_KLAYOUT:
            from mqt.scpd.export import write_layout  # ruff: ignore[import-outside-top-level]  # needs the optional dependency

            summary = write_layout(model, tmp_path / f"{chip}-{stage}.gds", planning=geometry)
            assert summary.path.stat().st_size > 0
