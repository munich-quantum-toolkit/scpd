# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The planning steps on the benchmark chips of planar-superconducting-pd.

Each chip is planned once, with the configuration of this repository, and the plan is checked
against what the later stages rely on: every inner port is reached, every ring port has a launcher,
every connection has a corridor, a crossing slot carries one wire, and no two wires meet inside a
partition. The tests run when ``MQT_SCPD_BENCHMARKS`` names a clone of the data repository.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import TYPE_CHECKING, TypeVar

import pytest

from mqt.scpd.artifacts import read_artifact
from mqt.scpd.chip import decode_chip, role_name
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from mqt.scpd.flatbuffers.design.AssignedRole import AssignedRole
from mqt.scpd.planning import PLANNING_STAGES, planning_geometry
from mqt.scpd.plot import layout_svg
from mqt.scpd.run import BlockReporter, RunDirectory
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Sequence

    from mqt.scpd.console import Figure
    from mqt.scpd.flatbuffers.design.Chip import ChipT

DATA = Path(os.environ["MQT_SCPD_BENCHMARKS"]) if "MQT_SCPD_BENCHMARKS" in os.environ else None
CHIPS = sorted(path.parent.name for path in DATA.glob("inputs/*/config.toml")) if DATA else []
REPOSITORY = Path(__file__).resolve().parents[3] / "benchmarks"

pytestmark = pytest.mark.skipif(DATA is None, reason="MQT_SCPD_BENCHMARKS names no clone of the benchmark data")

#: The objective of the assignment on the chips that are small enough to know it by heart.
OBJECTIVES = {"4q": 5.02, "9q": 15.27}

#: The largest SVG a planning step of a benchmark chip may take, in bytes.
SVG_LIMIT = 10_000_000

Point = tuple[float, float]

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


def side(p: Point, q: Point, r: Point) -> int:
    """Which side of the line through ``p`` and ``q`` the point ``r`` lies on.

    The points are layout units in the tens of thousands, so three points on one line come out near
    zero rather than at it, and the sign is taken against the lengths of the two segments.

    Returns:
        1, -1, or 0 for a point on the line.
    """
    cross = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    scale = math.dist(p, q) * math.dist(p, r)
    if abs(cross) <= 1e-9 * scale:
        return 0
    return 1 if cross > 0 else -1


def crosses(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Whether two chords cross each other properly.

    Returns:
        True when each chord has the ends of the other on both of its sides.
    """
    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def overlaps(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Whether two chords run along each other over a length.

    Returns:
        True when they lie on one line and share more than a thousandth of a cell.
    """
    if side(a, b, c) != 0 or side(a, b, d) != 0:
        return False
    axis = 1 if abs(b[0] - a[0]) < abs(b[1] - a[1]) else 0
    first = sorted((a[axis], b[axis]))
    second = sorted((c[axis], d[axis]))
    return min(first[1], second[1]) - max(first[0], second[0]) > 1e-3


def between(a: Point, b: Point, c: Point) -> bool:
    """Whether the point ``c`` lies between the ends of the chord from ``a`` to ``b`` on its line.

    Returns:
        True when it does.
    """
    slack = 1e-9 * math.dist(a, b)
    return (
        min(a[0], b[0]) - slack <= c[0] <= max(a[0], b[0]) + slack
        and min(a[1], b[1]) - slack <= c[1] <= max(a[1], b[1]) + slack
    )


def same(a: Point, b: Point) -> bool:
    """Whether two points are one place, up to the error of the round trip through layout units.

    Returns:
        True when both coordinates agree to a millionth.
    """
    return abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6


def chords(routing: CorridorRoutingT) -> list[tuple[int, int, Point, Point, bool, bool]]:
    """Every chord of every corridor: the wire, its partition, its ends, and whether each end is pinned.

    A wire is pinned at the point it is fed at and at the cell of its target port; a crossing slot
    is no pin.

    Returns:
        The chords.
    """
    drawn = []
    for wire, corridor in enumerate(entries(routing.corridors)):
        if not corridor.partitions or corridor.source is None or corridor.target is None:
            continue
        places = [
            (corridor.source.x, corridor.source.y),
            *((crossing.x, crossing.y) for crossing in entries(corridor.crossings)),
            (corridor.target.x, corridor.target.y),
        ]
        for index, partition in enumerate(corridor.partitions):
            drawn.append((
                wire,
                int(partition),
                places[index],
                places[index + 1],
                index == 0,
                index == len(corridor.partitions) - 1,
            ))
    return drawn


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

    chained: list[int] = []
    ends = {"launcher": 0, "termination": 0}
    for chain in entries(assignment.chains):
        chained.extend(int(node) for node in chain.nodes or [])
        for end in (chain.start, chain.end):
            ends["launcher" if end is not None else "termination"] += 1
    assert sorted(chained) == sorted(resonators), "a resonator is in no chain or in two"
    assert ends == {"launcher": launcher_target, "termination": terminations}


def check_the_corridors(routing: CorridorRoutingT, plan: CapacityPlanT) -> None:
    """Every connection has a corridor, a slot carries one wire, and no two wires meet in a partition."""
    corridors = entries(routing.corridors)
    assert all(corridor.partitions for corridor in corridors), "a connection has no corridor"

    taken = [(crossing.x, crossing.y) for corridor in corridors for crossing in entries(corridor.crossings)]
    assert len(set(taken)) == len(taken), "two wires cross at one slot"

    borders = {tuple(sorted((border.first, border.second))) for border in entries(plan.borders)}
    pockets = {
        (position.x, position.y)
        for slots in entries(routing.slots)
        if slots.pocket
        for position in entries(slots.positions)
    }
    for corridor in corridors:
        partitions = [int(partition) for partition in corridor.partitions or []]
        crossings = entries(corridor.crossings)
        assert len(crossings) + 1 == len(partitions)
        for index, crossing in enumerate(crossings):
            if (crossing.x, crossing.y) not in pockets:
                assert tuple(sorted(partitions[index : index + 2])) in borders, "a wire leaves into no neighbour"

    feeds = [(corridor.source.x, corridor.source.y) for corridor in corridors if corridor.source is not None]
    low_x, high_x = min(x for x, _ in feeds), max(x for x, _ in feeds)
    low_y, high_y = min(y for _, y in feeds), max(y for _, y in feeds)
    assert all(low_x < x < high_x and low_y < y < high_y for x, y in taken), "a wire crosses outside the ring"

    by_partition: dict[int, list[tuple[int, Point, Point, bool, bool]]] = {}
    for wire, partition, start, end, start_pin, end_pin in chords(routing):
        for other, theirs_start, theirs_end, theirs_start_pin, theirs_end_pin in by_partition.get(partition, []):
            if other == wire:
                continue
            assert not crosses(start, end, theirs_start, theirs_end), f"wires {wire} and {other} cross"
            assert not overlaps(start, end, theirs_start, theirs_end), f"wires {wire} and {other} overlap"
            for chord, pins in (
                ((start, end), ((theirs_start, theirs_start_pin), (theirs_end, theirs_end_pin))),
                ((theirs_start, theirs_end), ((start, start_pin), (end, end_pin))),
            ):
                for pin, pinned in pins:
                    on = side(chord[0], chord[1], pin) == 0 and between(chord[0], chord[1], pin)
                    at_end = same(pin, chord[0]) or same(pin, chord[1])
                    assert not (pinned and on and not at_end), f"wires {wire} and {other} share a pin"
        by_partition.setdefault(partition, []).append((wire, start, end, start_pin, end_pin))


@pytest.mark.parametrize("chip", CHIPS)
def test_a_chip_plans_into_a_plan_the_later_stages_can_follow(chip: str, tmp_path: Path) -> None:
    """Every step runs without a fail, and the plan keeps the rules the later stages rely on."""
    assert DATA is not None
    directory = RunDirectory(tmp_path / chip)
    config = directory.prepare(REPOSITORY / chip / "config.toml", DATA / "inputs" / chip / "routing_config.json")
    chip_bytes = directory.classified_chip(config)
    fails = Fails()
    for step in STEPS:
        directory.run_stage(step, config, chip_bytes, BlockReporter(fails))
    assert fails.counts == [None, 0, 0, 0]

    model = decode_chip(chip_bytes)
    plan = read_artifact(directory.artifact("capacity").read_bytes()).output
    assignment = read_artifact(directory.artifact("assign").read_bytes()).output
    routing = read_artifact(directory.artifact("corridor").read_bytes()).output
    assert isinstance(plan, CapacityPlanT)
    assert isinstance(assignment, AssignmentT)
    assert isinstance(routing, CorridorRoutingT)
    assert config.stages is not None
    assert config.stages.assignment is not None
    assert config.rules is not None
    check_the_assignment(model, assignment, config.stages.assignment.launcherTarget, config.rules.feedlineTerminations)
    check_the_corridors(routing, plan)
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
