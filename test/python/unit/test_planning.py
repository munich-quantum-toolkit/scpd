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
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.Lattice import LatticeT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.planning import PlanningError, planning_geometry
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
