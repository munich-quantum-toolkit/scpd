# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the shapes that the planning artifacts describe."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd.artifacts import write_artifact
from mqt.scpd.chip import decode_chip
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
from mqt.scpd.flatbuffers.artifacts.FeedlineChain import FeedlineChainT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.Lattice import LatticeT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.design.PortRef import PortRefT
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.planning import Feedline, PlanningError, PlanningGeometry, planning_geometry
from mqt.scpd.run import RunDirectory
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from mqt.scpd.flatbuffers.design.Chip import ChipT

CONFIG = Path(__file__).resolve().parents[2] / "fixtures" / "mini" / "config.toml"


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
    assert all(len(ring) >= 3 for ring in geometry.partitions)
    assert geometry.borders
    assert len(geometry.launchers) == 4
    assert not geometry.corridors


def test_the_global_picture_draws_the_lattice_and_the_selected_edges(run: RunDirectory) -> None:
    """Every lattice edge is drawn, and the edges the circuit selected once more on their own layer."""
    lattice = LatticeT(
        points=[PointT(0.0, 0.0), PointT(100.0, 0.0), PointT(100.0, 50.0)],
        edges=[0, 1, 1, 2],
        selected=[1],
    )
    data = write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.GlobalRouting,
            output=GlobalRoutingT(lattices=[lattice], connections=[], outerRing=[], resonators=[]),
        )
    )

    geometry = planning_geometry(data, chip_of(run), "global")

    assert geometry.lattice == [((0.0, 0.0), (100.0, 0.0)), ((100.0, 0.0), (100.0, 50.0))]
    assert geometry.inner == [((100.0, 0.0), (100.0, 50.0))]


def test_the_assignment_picture_draws_a_chord_from_every_ring_port_to_its_feed(run: RunDirectory) -> None:
    """The ring is drawn through its ports, and each port is joined to the point it is fed from."""
    geometry = planning_geometry(run.artifact("assign").read_bytes(), chip_of(run), "assign")

    assert len(geometry.ring) == 5
    assert len(geometry.assignments) == 5
    assert {chord[0] for chord in geometry.assignments} == set(geometry.ring)


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
                connections=[],
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


def test_an_artifact_of_another_stage_is_refused(run: RunDirectory) -> None:
    """The artifact has to be the one of the stage it is drawn as."""
    with pytest.raises(PlanningError, match="not a global routing"):
        planning_geometry(run.artifact("capacity").read_bytes(), chip_of(run), "global")
    with pytest.raises(PlanningError, match="no planning stage"):
        planning_geometry(run.artifact("capacity").read_bytes(), chip_of(run), "detail")
