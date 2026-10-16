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
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
from mqt.scpd.flatbuffers.artifacts.Bottleneck import BottleneckT
from mqt.scpd.flatbuffers.artifacts.CapacityElement import CapacityElement
from mqt.scpd.flatbuffers.artifacts.CapacityNode import CapacityNodeT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.Corridor import CorridorT
from mqt.scpd.flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from mqt.scpd.flatbuffers.artifacts.FeedlineChain import FeedlineChainT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.GridExtent import GridExtentT
from mqt.scpd.flatbuffers.artifacts.Lattice import LatticeT
from mqt.scpd.flatbuffers.artifacts.Partition import PartitionT
from mqt.scpd.flatbuffers.artifacts.PartitionBorder import PartitionBorderT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.design.Connection import ConnectionT
from mqt.scpd.flatbuffers.design.PortRef import PortRefT
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.flatbuffers.geometry.Polygon import PolygonT
from mqt.scpd.planning import Feedline, PlanningError, PlanningGeometry, planning_geometry
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
    assert not geometry.corridors
    # The grid blocks a border of cells along its edge.
    assert geometry.blocked
    assert len(geometry.partition_tones) == len(geometry.partitions)


def test_the_assign_picture_marks_only_the_launcher_slots(run: RunDirectory) -> None:
    """The feed of a conventional port is a launcher slot, and the feed of a resonator lies on its feedline.

    Over the capacity plan, the assign picture marks the slots of the plan and no feed of a resonator.
    """
    capacity = run.artifact("capacity").read_bytes()
    slots = planning_geometry(capacity, chip_of(run), "capacity").launchers

    geometry = planning_geometry(run.artifact("assign").read_bytes(), chip_of(run), "assign", capacity)

    assert sorted(geometry.launchers) == sorted(slots)
    assert all(not set(feedline.terminals) & set(geometry.launchers) for feedline in geometry.feedlines)


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
    """A strip of one cell between two partitions is no blockage; a block of four cells is."""
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


def test_the_global_picture_draws_the_lattice_and_the_selected_edges(run: RunDirectory) -> None:
    """Every lattice edge is drawn, and the edges the circuit selected once more on their own layer."""
    geometry = planning_geometry(solved_routing(), chip_of(run), "global")

    assert geometry.lattice == [((0.0, 0.0), (100.0, 0.0)), ((100.0, 0.0), (100.0, 50.0))]
    assert geometry.inner == [((100.0, 0.0), (100.0, 50.0))]


def test_the_first_lattice_edge_can_be_selected(run: RunDirectory) -> None:
    """Edge 0 is an edge like any other: when the circuit selects it, it is drawn as selected."""
    lattice = LatticeT(points=[PointT(0.0, 0.0), PointT(100.0, 0.0)], edges=[0, 1], selected=[0])
    data = write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.GlobalRouting,
            output=GlobalRoutingT(lattices=[lattice], connections=[], outerRing=[], resonators=[]),
        )
    )

    geometry = planning_geometry(data, chip_of(run), "global")

    assert geometry.lattice == [((0.0, 0.0), (100.0, 0.0))]
    assert geometry.inner == [((0.0, 0.0), (100.0, 0.0))]


def test_the_assignment_picture_draws_a_chord_from_every_ring_port_to_its_feed(run: RunDirectory) -> None:
    """The ring is drawn through its ports, and each port is joined to the point it is fed from."""
    geometry = planning_geometry(run.artifact("assign").read_bytes(), chip_of(run), "assign")

    assert len(geometry.ring) == 5
    assert len(geometry.assignments) == 5
    assert {chord[0] for chord in geometry.assignments} == set(geometry.ring)
    assert not geometry.borders


def test_the_assignment_picture_lies_over_the_plan_of_its_run(run: RunDirectory) -> None:
    """With the capacity plan, the chords are drawn over its partitions and blocked cells."""
    geometry = planning_geometry(
        run.artifact("assign").read_bytes(),
        chip_of(run),
        "assign",
        capacity=run.artifact("capacity").read_bytes(),
    )

    assert geometry.assignments
    assert geometry.borders
    assert geometry.blocked
    assert any(tone is not None for tone in geometry.partition_tones)


def assignment_of(ring: list[int], feeds: list[PointT], chain: FeedlineChainT) -> bytes:
    """An assignment artifact with one feedline chain.

    Returns:
        The bytes of the artifact.
    """
    return write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.Assignment,
            output=AssignmentT(
                connections=[ConnectionT(target=PortRefT(index)) for index in ring],
                ring=[PortRefT(index) for index in ring],
                launchers=[PortRefT(index) for index in ring],
                feeds=feeds,
                chains=[chain],
            ),
        )
    )


def test_a_feedline_runs_from_its_launcher_through_the_feeds_of_its_resonators(run: RunDirectory) -> None:
    """A chain runs from its first launcher over the feed of each resonator to its last launcher.

    Where the chain ends at a termination instead, it ends at the feed of its resonator there. A
    resonator is named by its component where the configuration declares one, else by its label.
    """
    chip = chip_of(run)
    assert chip.ports is not None
    resonator = chip.ports[8]
    assert resonator is not None
    resonator.component = "Q2"
    feeds = [PointT(250.0, 1600.0), PointT(1000.0, 1600.0)]

    between = planning_geometry(
        assignment_of([4, 8], feeds, FeedlineChainT(nodes=[0, 1], start=PortRefT(1), end=PortRefT(2))), chip, "assign"
    )
    stopped = planning_geometry(
        assignment_of([4, 8], feeds, FeedlineChainT(nodes=[0, 1], end=PortRefT(3))), chip, "assign"
    )

    (feedline,) = between.feedlines
    assert feedline.points == [(300.0, 1600.0), (250.0, 1600.0), (1000.0, 1600.0), (2000.0, 200.0)]
    assert feedline.terminals == [(250.0, 1600.0), (1000.0, 1600.0)]
    assert not feedline.terminations
    assert feedline.label == "feedline 1: Chip.port1 → Q1.port0, Q2 → Chip.port2"
    (feedline,) = stopped.feedlines
    assert feedline.points == [(250.0, 1600.0), (1000.0, 1600.0), (700.0, -600.0)]
    assert feedline.terminations == [(250.0, 1600.0)]
    assert feedline.label == "feedline 1: termination → Q1.port0, Q2 → Chip.port3"


def test_a_feedline_through_the_first_resonator_alone_is_drawn(run: RunDirectory) -> None:
    """A chain that names ring node 0 alone runs through the feed of that node."""
    geometry = planning_geometry(
        assignment_of([4], [PointT(250.0, 1600.0)], FeedlineChainT(nodes=[0], start=PortRefT(1))),
        chip_of(run),
        "assign",
    )

    (feedline,) = geometry.feedlines
    assert feedline.terminals == [(250.0, 1600.0)]


def test_a_feedline_alone_is_something_to_draw() -> None:
    """A stage that produced only feedlines still has a picture."""
    assert not PlanningGeometry(feedlines=[Feedline(points=[(0.0, 0.0), (1.0, 0.0)])]).is_empty()


def test_the_corridor_picture_draws_the_feedlines_of_its_assignment(run: RunDirectory) -> None:
    """The wires of the resonators start at the feeds the feedlines pass, so both are drawn together."""
    chip = chip_of(run)
    assignment = run.artifact("assign").read_bytes()
    corridor = run.artifact("corridor").read_bytes()

    assigned = planning_geometry(assignment, chip, "assign")
    routed = planning_geometry(corridor, chip, "corridor", assignment=assignment)

    assert assigned.feedlines
    assert routed.feedlines == assigned.feedlines
    assert not planning_geometry(corridor, chip, "corridor").feedlines
    with pytest.raises(PlanningError, match="not an assignment"):
        planning_geometry(corridor, chip, "corridor", assignment=run.artifact("capacity").read_bytes())


def test_the_corridor_picture_draws_every_route_over_the_partitions(run: RunDirectory) -> None:
    """A corridor runs from its feed over its crossings to its target, over the partitions it uses."""
    geometry = planning_geometry(
        run.artifact("corridor").read_bytes(),
        chip_of(run),
        "corridor",
        capacity=run.artifact("capacity").read_bytes(),
    )

    assert len(geometry.corridors) == 5
    assert all(len(route) >= 2 for route in geometry.corridors)
    assert geometry.slots
    assert geometry.partitions


@pytest.mark.parametrize("partitions", [[0], [0, 1]], ids=["one", "two"])
def test_a_corridor_through_the_first_partition_is_read_and_drawn(run: RunDirectory, partitions: list[int]) -> None:
    """Partition 0 is a partition like any other, so a corridor through it is complete and drawn."""
    crossings = [PointT(50.0, 0.0)] if len(partitions) == 2 else []
    corridor = CorridorT(partitions=partitions, crossings=crossings, source=PointT(0.0, 0.0), target=PointT(100.0, 0.0))
    data = write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.CorridorRouting,
            output=CorridorRoutingT(corridors=[corridor], slots=[]),
        )
    )

    output = read_artifact(data).output
    geometry = planning_geometry(data, chip_of(run), "corridor")

    assert isinstance(output, CorridorRoutingT)
    assert output.corridors is not None
    (read,) = output.corridors
    assert read is not None
    assert read.partitions is not None
    assert [int(partition) for partition in read.partitions] == partitions
    assert geometry.corridors == [[(0.0, 0.0), *((point.x, point.y) for point in crossings), (100.0, 0.0)]]


def test_every_routed_wire_approaches_its_target_port(run: RunDirectory) -> None:
    """A stub runs from where a corridor ends to the port its connection targets, by connection order."""
    chip = chip_of(run)
    corridor = run.artifact("corridor").read_bytes()
    assignment = run.artifact("assign").read_bytes()

    geometry = planning_geometry(corridor, chip, "corridor", assignment=assignment)

    chosen = read_artifact(assignment).output
    assert isinstance(chosen, AssignmentT)
    ports = chip.ports or []
    targets = set()
    for connection in chosen.connections or []:
        assert connection is not None
        assert connection.target is not None
        center = ports[connection.target.index].center
        assert center is not None
        targets.add((center.x, center.y))
    assert len(geometry.stubs) == len(geometry.corridors)
    assert all(stub[0] == route[-1] for stub, route in zip(geometry.stubs, geometry.corridors, strict=True))
    assert all(stub[-1] in targets for stub in geometry.stubs)
    # The last leg runs along the direction of the port.
    by_center = {(port.center.x, port.center.y): port for port in ports if port is not None and port.center is not None}
    for stub in geometry.stubs:
        if len(stub) == 3:
            angle = math.radians(by_center[stub[-1]].orientation)
            dx, dy = stub[1][0] - stub[2][0], stub[1][1] - stub[2][1]
            assert abs(dx * math.sin(angle) - dy * math.cos(angle)) < 1e-6
    # Without the assignment no corridor names its port.
    assert not planning_geometry(corridor, chip, "corridor").stubs


def test_an_artifact_of_another_stage_is_refused(run: RunDirectory) -> None:
    """The artifact has to be the one of the stage it is drawn as."""
    with pytest.raises(PlanningError, match="not a global routing"):
        planning_geometry(run.artifact("capacity").read_bytes(), chip_of(run), "global")
    with pytest.raises(PlanningError, match="no planning stage"):
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
    assert geometry.chains == [((50.0, 0.0), (300.0, 1600.0))]
    assert geometry.keepout == [[(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]]


def test_a_cyclic_capacity_chain_is_drawn_once(run: RunDirectory) -> None:
    """A chain that leads back to a node it passed draws each link once and ends."""
    geometry = planning_geometry(capacity_plan(cyclic=True), chip_of(run), "capacity")

    assert geometry.chains == [((50.0, 0.0), (300.0, 1600.0)), ((300.0, 1600.0), (50.0, 0.0))]


def test_the_global_picture_lies_over_the_blocked_cells_of_its_capacity_plan(run: RunDirectory) -> None:
    """The global stage is drawn over the cells its grid blocked, and keeps the gates and the keep-out."""
    plan = capacity_plan()
    geometry = planning_geometry(run.artifact("global").read_bytes(), chip_of(run), "global", capacity=plan)

    assert geometry.blocked
    assert geometry.blocked == planning_geometry(plan, chip_of(run), "capacity").blocked
    assert geometry.bottlenecks == [((0.0, 0.0), (100.0, 0.0), 2)]
    assert geometry.keepout == [[(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]]


def solved_routing() -> bytes:
    """A global routing with one lattice of two edges, of which the circuit selected the second.

    Returns:
        The bytes of the artifact.
    """
    lattice = LatticeT(
        points=[PointT(0.0, 0.0), PointT(100.0, 0.0), PointT(100.0, 50.0)],
        edges=[0, 1, 1, 2],
        selected=[1],
    )
    return write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.GlobalRouting,
            output=GlobalRoutingT(lattices=[lattice], connections=[], outerRing=[], resonators=[]),
        )
    )


@pytest.mark.parametrize("stage", ["assign", "corridor"])
def test_the_assign_and_corridor_pictures_draw_the_selected_edges_of_the_inner_circuit(
    run: RunDirectory, stage: str
) -> None:
    """The inner wires show on the lattice edges the circuit selected; the rest of the lattice does not."""
    data = run.artifact(stage).read_bytes()

    geometry = planning_geometry(data, chip_of(run), stage, global_routing=solved_routing())

    assert geometry.inner == [((100.0, 0.0), (100.0, 50.0))]
    assert not geometry.lattice
    assert not planning_geometry(data, chip_of(run), stage).inner
    with pytest.raises(PlanningError, match="the global artifact is not a global routing"):
        planning_geometry(data, chip_of(run), stage, global_routing=run.artifact("capacity").read_bytes())


@pytest.mark.parametrize(
    ("stage", "drawn", "expected"),
    [
        ("capacity", "global", "not a capacity plan"),
        ("assign", "capacity", "not an assignment"),
        ("corridor", "capacity", "not a corridor routing"),
    ],
)
def test_every_stage_refuses_the_artifact_of_another(run: RunDirectory, stage: str, drawn: str, expected: str) -> None:
    """Each planning stage draws its own artifact only."""
    with pytest.raises(PlanningError, match=expected):
        planning_geometry(run.artifact(drawn).read_bytes(), chip_of(run), stage)


def test_a_capacity_artifact_has_to_be_a_capacity_plan(run: RunDirectory) -> None:
    """The artifact that a stage is drawn over has to be the capacity plan of the run."""
    with pytest.raises(PlanningError, match="the capacity artifact is not a capacity plan"):
        planning_geometry(
            run.artifact("global").read_bytes(), chip_of(run), "global", capacity=run.artifact("assign").read_bytes()
        )
