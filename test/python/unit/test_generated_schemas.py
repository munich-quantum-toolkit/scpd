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
import pytest

from mqt.scpd.flatbuffers.artifacts.Artifact import Artifact, ArtifactT
from mqt.scpd.flatbuffers.artifacts.FinalRouting import FinalRoutingT
from mqt.scpd.flatbuffers.artifacts.Geometry import GeometryT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.artifacts.Wire import WireT
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
from mqt.scpd.flatbuffers.geometry.Arc import ArcT
from mqt.scpd.flatbuffers.geometry.Line import LineT
from mqt.scpd.flatbuffers.geometry.Path import PathT
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.flatbuffers.geometry.Polygon import PolygonT
from mqt.scpd.flatbuffers.geometry.Segment import SegmentT
from mqt.scpd.flatbuffers.geometry.SegmentShape import SegmentShape


def test_role_enums_match_the_wire_format() -> None:
    """The numeric enum values are the on-disk format shared with the C++ side."""
    assert UnassignedRole.Launcher == 0
    assert UnassignedRole.Resonator == 1
    assert UnassignedRole.Conventional == 2
    assert AssignedRole.FeedlineSource == 0
    assert AssignedRole.ResonatorSource == 2
    assert AssignedRole.ConventionalTarget == 5
    assert Rotation.R315 == 7
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


def test_artifact_carries_the_schema_identifier() -> None:
    """A stored artifact starts with the identifier that records the schema version."""
    artifact = ArtifactT(
        producer="mqt-scpd test",
        outputType=StageOutput.GlobalRouting,
        output=GlobalRoutingT(),
    )

    builder = flatbuffers.Builder()
    builder.Finish(artifact.Pack(builder), file_identifier=b"SCP1")
    stored = builder.Output()

    assert Artifact.ArtifactBufferHasIdentifier(stored, 0, size_prefixed=False)
    back = ArtifactT.InitFromPackedBuf(stored)
    assert back.producer == "mqt-scpd test"
    assert back.outputType == StageOutput.GlobalRouting
    assert isinstance(back.output, GlobalRoutingT)


def test_final_routing_keeps_scalar_vectors() -> None:
    """A vector of indices survives packing with zero, one and several elements."""
    for unresolved in ([], [7], [2, 5, 11]):
        artifact = ArtifactT(
            outputType=StageOutput.FinalRouting,
            output=FinalRoutingT(couplers=[], bridges=[], unresolved=list(unresolved)),
        )

        builder = flatbuffers.Builder()
        builder.Finish(artifact.Pack(builder), file_identifier=b"SCP1")
        back = ArtifactT.InitFromPackedBuf(builder.Output())

        assert isinstance(back.output, FinalRoutingT)
        assert list(back.output.unresolved) == unresolved


def test_geometry_artifact_keeps_analytic_segments() -> None:
    """A wire is stored as lines and arcs, never as sampled points."""
    wire = WireT(
        connection=0,
        path=PathT(
            segments=[
                SegmentT(
                    shapeType=SegmentShape.Line,
                    shape=LineT(start=PointT(0.0, 0.0), end=PointT(100.0, 0.0)),
                ),
                SegmentT(
                    shapeType=SegmentShape.Arc,
                    shape=ArcT(center=PointT(100.0, 50.0), radius=50.0, startAngle=0.0, sweep=1.5),
                ),
            ]
        ),
    )
    artifact = ArtifactT(
        outputType=StageOutput.Geometry,
        output=GeometryT(wires=[wire], couplers=[], bridges=[]),
    )

    builder = flatbuffers.Builder()
    builder.Finish(artifact.Pack(builder), file_identifier=b"SCP1")
    back = ArtifactT.InitFromPackedBuf(builder.Output())

    assert isinstance(back.output, GeometryT)
    path = back.output.wires[0].path
    assert path is not None
    segments = path.segments
    assert [segment.shapeType for segment in segments] == [SegmentShape.Line, SegmentShape.Arc]
    assert isinstance(segments[0].shape, LineT)
    assert isinstance(segments[1].shape, ArcT)
    assert segments[1].shape.radius == pytest.approx(50.0)


def test_config_defaults_are_the_documented_defaults() -> None:
    """Absent keys take the defaults that the configuration section documents."""
    ports = PortConfigT()
    assert ports.detection == PortDetection.Manual
    assert ports.sequences is None

    grid = GridParamsT()
    assert (grid.capacityCellsX, grid.capacityCellsY) == (50, 0)
    assert (grid.launcherOffsetX, grid.launcherOffsetY) == (15, 15)
