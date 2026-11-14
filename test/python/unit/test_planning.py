# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the shapes that the planning artifacts describe."""

from __future__ import annotations

import math
from itertools import pairwise
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd.artifacts import read_artifact, write_artifact
from mqt.scpd.chip import decode_chip
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.Bottleneck import BottleneckT
from mqt.scpd.flatbuffers.artifacts.CapacityElement import CapacityElement
from mqt.scpd.flatbuffers.artifacts.CapacityNode import CapacityNodeT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.GridExtent import GridExtentT
from mqt.scpd.flatbuffers.artifacts.Partition import PartitionT
from mqt.scpd.flatbuffers.artifacts.PartitionBorder import PartitionBorderT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.flatbuffers.geometry.Polygon import PolygonT
from mqt.scpd.planning import PlanningError, PlanningGeometry, planning_geometry
from mqt.scpd.run import RunDirectory
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from mqt.scpd.flatbuffers.design.Chip import ChipT

CONFIG = Path(__file__).resolve().parents[2] / "fixtures" / "mini" / "config.toml"

# Every picture is drawn from artifacts read once with lists and once with NumPy arrays.
pytestmark = pytest.mark.usefixtures("vector_type")


@pytest.fixture(scope="module")
def run(tmp_path_factory: pytest.TempPathFactory) -> RunDirectory:
    """The fixture planned through every step.

    Returns:
        The run directory.
    """
    directory = RunDirectory(tmp_path_factory.mktemp("run"))
    config = directory.prepare(CONFIG)
    chip = directory.classified_chip(config)
    for step in STEPS:
        directory.run_stage(step, config, chip)
    return directory


def chip_of(run: RunDirectory) -> ChipT:
    """The chip of the run, decoded.

    Returns:
        The chip.
    """
    return decode_chip(run.classified_chip(run.load()))


def test_the_capacity_picture_holds_the_partitions_and_the_launchers(run: RunDirectory) -> None:
    """A capacity plan draws its partitions, their borders and the launcher slots."""
    geometry = planning_geometry(run.artifact("capacity").read_bytes(), chip_of(run), "capacity")

    assert geometry.partitions
    assert all(len(ring) >= 3 for partition in geometry.partitions for ring in partition)
    assert geometry.borders
    assert len(geometry.launchers) == 4
    # The grid blocks a border of cells along its edge.
    assert geometry.blocked
    assert len(geometry.partition_tones) == len(geometry.partitions)


def test_a_partition_keeps_its_rings_together(run: RunDirectory) -> None:
    """A partition with a hole is one entry of two rings, with one tone."""
    grid = GridExtentT(width=12, height=12, origin=PointT(0.5, 0.5), cellWidth=1.0, cellHeight=1.0)
    holed = PartitionT(
        label=2,
        outlines=[
            PolygonT(vertices=[PointT(0.0, 0.0), PointT(12.0, 0.0), PointT(12.0, 12.0), PointT(0.0, 12.0)]),
            PolygonT(vertices=[PointT(4.0, 4.0), PointT(8.0, 4.0), PointT(8.0, 8.0), PointT(4.0, 8.0)]),
        ],
    )
    plan = CapacityPlanT(
        capacityGrid=grid,
        detailGrid=grid,
        partitions=[holed],
        borders=[],
        bottlenecks=[],
        launchers=[],
        nodes=[],
        chains=[],
    )
    data = write_artifact(ArtifactT(producer="test", outputType=StageOutput.CapacityPlan, output=plan))

    geometry = planning_geometry(data, chip_of(run), "capacity")

    assert [len(partition) for partition in geometry.partitions] == [2]
    assert len(geometry.partition_tones) == 1


def test_a_border_runs_along_its_cell_edges(run: RunDirectory) -> None:
    """Every step of a border run is one whole cell edge, and the runs use every edge of the plan once."""
    raw = run.artifact("capacity").read_bytes()
    plan = read_artifact(raw).output
    assert isinstance(plan, CapacityPlanT)
    width, height = plan.detailGrid.cellWidth, plan.detailGrid.cellHeight

    geometry = planning_geometry(raw, chip_of(run), "capacity")

    steps = [(abs(b[0] - a[0]), abs(b[1] - a[1])) for border in geometry.borders for a, b in pairwise(border)]
    # The corners are joined on three decimals, the precision of the inputs.
    tolerance = 2e-3
    assert all(
        (math.isclose(dx, width, abs_tol=tolerance) and dy < tolerance)
        or (dx < tolerance and math.isclose(dy, height, abs_tol=tolerance))
        for dx, dy in steps
    )
    assert len(steps) == sum(len(border.samples or []) for border in plan.borders or [] if border is not None)


def test_the_pieces_of_a_border_close_together_are_one_run(run: RunDirectory) -> None:
    """Two edges of a border two cells apart join into one run; a piece far away stays a run of its own."""
    grid = GridExtentT(width=20, height=20, origin=PointT(0.5, 0.5), cellWidth=1.0, cellHeight=1.0)
    border = PartitionBorderT(
        first=1,
        second=2,
        samples=[PointT(1.0, 0.5), PointT(1.0, 3.5), PointT(1.0, 12.5)],
        center=PointT(1.0, 5.5),
    )
    plan = CapacityPlanT(
        capacityGrid=grid,
        detailGrid=grid,
        partitions=[],
        borders=[border],
        bottlenecks=[],
        launchers=[],
        nodes=[],
        chains=[],
    )
    data = write_artifact(ArtifactT(producer="test", outputType=StageOutput.CapacityPlan, output=plan))

    geometry = planning_geometry(data, chip_of(run), "capacity")

    assert sorted(geometry.borders) == [[(1.0, 0.0), (1.0, 1.0), (1.0, 3.0), (1.0, 4.0)], [(1.0, 12.0), (1.0, 13.0)]]


def _square(label: int, x0: float, y0: float, x1: float, y1: float) -> PartitionT:
    corners = [PointT(x0, y0), PointT(x1, y0), PointT(x1, y1), PointT(x0, y1)]
    return PartitionT(label=label, outlines=[PolygonT(vertices=corners)])


def test_the_blocked_cells_leave_out_what_is_too_thin_to_block(run: RunDirectory) -> None:
    """A strip of one cell between two partitions is not a blockage; a block of four cells is."""
    grid = GridExtentT(width=12, height=12, origin=PointT(0.5, 0.5), cellWidth=1.0, cellHeight=1.0)
    # Columns 0 to 4 and 6 to 11 are free, except a block of four by four cells at 7 to 10 and 4 to 7.
    partitions = [
        _square(2, 0.0, 0.0, 5.0, 12.0),
        PartitionT(
            label=3,
            outlines=[
                PolygonT(vertices=[PointT(6.0, 0.0), PointT(12.0, 0.0), PointT(12.0, 12.0), PointT(6.0, 12.0)]),
                PolygonT(vertices=[PointT(7.0, 4.0), PointT(11.0, 4.0), PointT(11.0, 8.0), PointT(7.0, 8.0)]),
            ],
        ),
    ]
    plan = CapacityPlanT(
        capacityGrid=grid,
        detailGrid=grid,
        partitions=partitions,
        borders=[],
        bottlenecks=[],
        launchers=[],
        nodes=[],
        chains=[],
    )
    data = write_artifact(ArtifactT(producer="test", outputType=StageOutput.CapacityPlan, output=plan))

    geometry = planning_geometry(data, chip_of(run), "capacity")

    cells = {
        (x, y)
        for ring in geometry.blocked
        for x in range(int(ring[0][0]), int(ring[1][0]))
        for y in range(int(ring[0][1]), int(ring[2][1]))
    }
    assert cells == {(x, y) for x in range(7, 11) for y in range(4, 8)}


def test_the_borders_of_closed_in_partitions_are_kept_apart(run: RunDirectory) -> None:
    """A group of partitions that no border joins to the outermost free cells is closed in."""
    grid = GridExtentT(width=20, height=20, origin=PointT(0.5, 0.5), cellWidth=1.0, cellHeight=1.0)
    partitions = [
        _square(1, 0.0, 0.0, 10.0, 20.0),
        _square(2, 10.0, 0.0, 20.0, 20.0),
        _square(3, 4.0, 4.0, 6.0, 6.0),
        _square(4, 6.0, 4.0, 8.0, 6.0),
    ]
    borders = [
        PartitionBorderT(first=1, second=2, samples=[PointT(10.0, 0.5)], center=PointT(10.0, 0.5)),
        PartitionBorderT(first=3, second=4, samples=[PointT(6.0, 4.5)], center=PointT(6.0, 4.5)),
    ]
    plan = CapacityPlanT(
        capacityGrid=grid,
        detailGrid=grid,
        partitions=partitions,
        borders=borders,
        bottlenecks=[],
        launchers=[],
        nodes=[],
        chains=[],
    )
    data = write_artifact(ArtifactT(producer="test", outputType=StageOutput.CapacityPlan, output=plan))

    geometry = planning_geometry(data, chip_of(run), "capacity")

    assert geometry.borders == [[(10.0, 0.0), (10.0, 1.0)]]
    assert geometry.enclosed_borders == [[(6.0, 4.0), (6.0, 5.0)]]
    # The two open partitions share a border, so their tones differ; the closed-in ones have none.
    first, second, *closed = geometry.partition_tones
    assert first is not None
    assert second is not None
    assert first != second
    assert closed == [None, None]


def test_blocked_cells_alone_are_something_to_draw() -> None:
    """A stage that produced only blocked cells still has a picture."""
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    assert not PlanningGeometry(blocked=[square]).is_empty()


def test_an_artifact_of_another_stage_is_refused(run: RunDirectory) -> None:
    """The artifact has to be the one of the stage it is drawn as."""
    other = write_artifact(ArtifactT(producer="test", outputType=StageOutput.GlobalRouting, output=GlobalRoutingT()))
    with pytest.raises(PlanningError, match="not a capacity plan"):
        planning_geometry(other, chip_of(run), "capacity")
    with pytest.raises(PlanningError, match="not a planning stage"):
        planning_geometry(run.artifact("capacity").read_bytes(), chip_of(run), "detail")


def capacity_plan(*, cyclic: bool = False, chains: list[int] | None = None) -> bytes:
    """A capacity plan with one gate, one keep-out ring and one chain.

    The chain runs from a launcher to the gate, and from the gate to port 1 and to an index that
    names no port.

    Args:
        cyclic: Link port 1 back to the gate, so that the chain is a cycle.
        chains: The nodes the chains start at; node 0, the launcher, when not given.

    Returns:
        The bytes of the artifact.
    """
    nodes = [
        CapacityNodeT(kind=CapacityElement.Launcher, id=0, next=[1]),
        CapacityNodeT(kind=CapacityElement.Bottleneck, id=0, next=[2, 3]),
        CapacityNodeT(kind=CapacityElement.Target, id=1, next=[1] if cyclic else []),
        CapacityNodeT(kind=CapacityElement.Target, id=99, next=[]),
    ]
    grid = GridExtentT(width=1, height=1, origin=PointT(0.5, 0.5), cellWidth=1.0, cellHeight=1.0)
    plan = CapacityPlanT(
        capacityGrid=grid,
        detailGrid=grid,
        partitions=[],
        borders=[],
        bottlenecks=[BottleneckT(from_=PointT(0.0, 0.0), to=PointT(100.0, 0.0), capacity=2)],
        launchers=[],
        nodes=nodes,
        chains=[0] if chains is None else chains,
        portKeepout=[PolygonT(vertices=[PointT(0.0, 0.0), PointT(10.0, 0.0), PointT(0.0, 10.0)])],
    )
    return write_artifact(ArtifactT(producer="test", outputType=StageOutput.CapacityPlan, output=plan))


def test_a_capacity_plan_without_chains_draws_none(run: RunDirectory) -> None:
    """A plan whose chain list is empty draws no chain."""
    geometry = planning_geometry(capacity_plan(chains=[]), chip_of(run), "capacity")

    assert geometry.chains == []
    assert geometry.bottlenecks == [((0.0, 0.0), (100.0, 0.0), 2)]


def test_a_capacity_chain_joins_the_ports_and_the_gates_it_names(run: RunDirectory) -> None:
    """A chain runs from the middle of each gate to the next port.

    A link is left out where one of its nodes has no place on the chip.
    """
    geometry = planning_geometry(capacity_plan(), chip_of(run), "capacity")

    assert geometry.bottlenecks == [((0.0, 0.0), (100.0, 0.0), 2)]
    assert geometry.chains == [((50.0, 0.0), (2000.0, 200.0))]
    assert geometry.keepout == [[(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]]


def test_a_cyclic_capacity_chain_is_drawn_once(run: RunDirectory) -> None:
    """A chain that leads back to a node it passed draws each link once and ends."""
    geometry = planning_geometry(capacity_plan(cyclic=True), chip_of(run), "capacity")

    assert geometry.chains == [((50.0, 0.0), (2000.0, 200.0)), ((2000.0, 200.0), (50.0, 0.0))]
