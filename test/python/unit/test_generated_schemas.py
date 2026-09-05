# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the schema-generated Python modules."""

from __future__ import annotations

import flatbuffers

from mqt.scpd.flatbuffers.config.GridParams import GridParamsT
from mqt.scpd.flatbuffers.config.PortConfig import PortConfigT
from mqt.scpd.flatbuffers.config.PortDetection import PortDetection
from mqt.scpd.flatbuffers.design.AssignedRole import AssignedRole
from mqt.scpd.flatbuffers.design.Chip import ChipT
from mqt.scpd.flatbuffers.design.Connection import ConnectionT
from mqt.scpd.flatbuffers.design.Port import PortT
from mqt.scpd.flatbuffers.design.PortRef import PortRefT
from mqt.scpd.flatbuffers.design.Rotation import Rotation
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.flatbuffers.geometry.Polygon import PolygonT


def test_role_enums_match_the_wire_format() -> None:
    """The numeric enum values are the on-disk format shared with the C++ side."""
    assert UnassignedRole.Unset == 0
    assert UnassignedRole.Launcher == 1
    assert UnassignedRole.Resonator == 2
    assert UnassignedRole.Conventional == 3
    assert UnassignedRole.Coupler == 4
    assert AssignedRole.Unset == 0
    assert AssignedRole.FeedlineSource == 1
    assert AssignedRole.ResonatorSource == 3
    assert AssignedRole.ConventionalTarget == 6
    assert Rotation.Unset == 0
    assert Rotation.R315 == 8
    assert PortDetection.Manual == 0
    assert PortDetection.Auto == 1


def test_chip_round_trips_through_object_api() -> None:
    """A chip built with the object API survives packing and unpacking."""
    chip = ChipT(
        obstacles=[
            PolygonT(
                vertices=[
                    PointT(0.0, 0.0),
                    PointT(1000.0, 0.0),
                    PointT(1000.0, 1000.0),
                ]
            )
        ],
        ports=[
            PortT(
                label="Chip.port0",
                center=PointT(0.0, 500.0),
                orientation=180.0,
                role=UnassignedRole.Launcher,
            ),
            PortT(
                label="Qb1.port0",
                center=PointT(500.0, 500.0),
                orientation=90.0,
                role=UnassignedRole.Resonator,
            ),
        ],
    )

    builder = flatbuffers.Builder()
    builder.Finish(chip.Pack(builder))
    back = ChipT.InitFromPackedBuf(builder.Output())

    assert [port.label for port in back.ports] == ["Chip.port0", "Qb1.port0"]
    resonator = back.ports[1]
    assert resonator.role == UnassignedRole.Resonator
    assert resonator.center is not None
    assert (resonator.center.x, resonator.center.y) == (500.0, 500.0)
    assert len(back.obstacles) == 1
    assert [(vertex.x, vertex.y) for vertex in back.obstacles[0].vertices][1] == (1000.0, 0.0)


def test_connection_source_may_be_absent() -> None:
    """A resonator connection has no source until the Final stage creates the coupler port."""
    connection = ConnectionT(
        target=PortRefT(index=3),
        sourceRole=AssignedRole.ResonatorSource,
        targetRole=AssignedRole.ResonatorTarget,
    )

    builder = flatbuffers.Builder()
    builder.Finish(connection.Pack(builder))
    back = ConnectionT.InitFromPackedBuf(builder.Output())

    assert back.source is None
    assert back.target is not None
    assert back.target.index == 3
    assert back.sourceRole == AssignedRole.ResonatorSource


def test_config_defaults_are_the_documented_defaults() -> None:
    """Absent keys take the defaults that the configuration section documents."""
    ports = PortConfigT()
    assert ports.detection == PortDetection.Manual
    assert ports.sequences is None

    grid = GridParamsT()
    assert (grid.capacityCellsX, grid.capacityCellsY) == (50, 0)
    assert (grid.launcherOffsetX, grid.launcherOffsetY) == (15, 15)
