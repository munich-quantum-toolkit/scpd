# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the JSON view of the artifacts."""

from __future__ import annotations

import json

import flatbuffers
import pytest

from mqt.scpd.artifacts import IDENTIFIER, write_artifact
from mqt.scpd.flatbuffers.artifacts.Artifact import (
    ArtifactAddOutput,
    ArtifactAddOutputType,
    ArtifactAddProducer,
    ArtifactEnd,
    ArtifactStart,
    ArtifactT,
)
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
from mqt.scpd.flatbuffers.artifacts.BorderSlots import BorderSlotsT
from mqt.scpd.flatbuffers.artifacts.Bottleneck import BottleneckT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.Corridor import CorridorT
from mqt.scpd.flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from mqt.scpd.flatbuffers.artifacts.DetailRouting import DetailRoutingT
from mqt.scpd.flatbuffers.artifacts.FeedlineChain import FeedlineChainT
from mqt.scpd.flatbuffers.artifacts.FinalRouting import FinalRoutingT
from mqt.scpd.flatbuffers.artifacts.Geometry import GeometryT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.GridExtent import GridExtentT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.artifacts.Wire import WireT
from mqt.scpd.flatbuffers.design.AssignedRole import AssignedRole
from mqt.scpd.flatbuffers.design.Bridge import BridgeT
from mqt.scpd.flatbuffers.design.Connection import ConnectionT
from mqt.scpd.flatbuffers.design.ConnectionRef import ConnectionRefT
from mqt.scpd.flatbuffers.design.CpwCoupler import CpwCouplerT
from mqt.scpd.flatbuffers.design.Port import PortT
from mqt.scpd.flatbuffers.design.PortRef import PortRefT
from mqt.scpd.flatbuffers.design.Rotation import Rotation
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole
from mqt.scpd.flatbuffers.geometry.Arc import ArcT
from mqt.scpd.flatbuffers.geometry.Line import LineT
from mqt.scpd.flatbuffers.geometry.Path import PathT
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.flatbuffers.geometry.Segment import SegmentT
from mqt.scpd.flatbuffers.geometry.SegmentShape import SegmentShape
from mqt.scpd.inspection import InspectionError, artifact_to_json


def coupler(connection: int) -> CpwCouplerT:
    """Build a coupler that completes one connection.

    Returns:
        The coupler.
    """
    return CpwCouplerT(
        connection=ConnectionRefT(index=connection),
        port=PortT(
            label=f"Coupler{connection}.port0", center=PointT(1.0, 2.0), orientation=90.0, role=UnassignedRole.Coupler
        ),
        center=PointT(1.0, 2.0),
        rotation=Rotation.R90,
        length=200.0,
        height=26.0,
    )


def artifact(output_type: int, output: object) -> bytes:
    """Serialize one stage output behind an artifact root.

    Returns:
        The bytes of the artifact.
    """
    return write_artifact(ArtifactT(producer="mqt-scpd test", outputType=output_type, output=output))


def empty_global() -> GlobalRoutingT:
    """A global routing that carries nothing, but every field the schema requires.

    Returns:
        The output.
    """
    return GlobalRoutingT(lattices=[], connections=[], outerRing=[], resonators=[])


STAGE_OUTPUTS = [
    pytest.param(
        StageOutput.CapacityPlan,
        CapacityPlanT(
            capacityGrid=GridExtentT(width=12, height=9, origin=PointT(1.5, -2.0), cellWidth=10.0, cellHeight=10.0),
            detailGrid=GridExtentT(origin=PointT(1.5, -2.0)),
            partitions=[],
            borders=[],
            bottlenecks=[BottleneckT(from_=PointT(0.0, 0.0), to=PointT(0.0, 30.0), capacity=1)],
            launchers=[],
            nodes=[],
            chains=[],
        ),
        {
            "capacity_grid": {
                "width": 12,
                "height": 9,
                "origin": {"x": 1.5, "y": -2.0},
                "cell_width": 10.0,
                "cell_height": 10.0,
            },
            "detail_grid": {"origin": {"x": 1.5, "y": -2.0}},
            "partitions": [],
            "borders": [],
            "bottlenecks": [{"from": {"x": 0.0, "y": 0.0}, "to": {"x": 0.0, "y": 30.0}, "capacity": 1}],
            "launchers": [],
            "nodes": [],
            "chains": [],
        },
        id="capacity",
    ),
    pytest.param(
        StageOutput.Assignment,
        AssignmentT(
            connections=[
                ConnectionT(
                    target=PortRefT(3), sourceRole=AssignedRole.ResonatorSource, targetRole=AssignedRole.ResonatorTarget
                ),
                ConnectionT(
                    source=PortRefT(1),
                    target=PortRefT(2),
                    sourceRole=AssignedRole.FeedlineSource,
                    targetRole=AssignedRole.FeedlineTarget,
                ),
            ],
            objective=132.68,
            ring=[PortRefT(3), PortRefT(2)],
            launchers=[PortRefT(9), PortRefT(1)],
            feeds=[PointT(120.5, -40.25), PointT(0.0, 0.0)],
            chains=[FeedlineChainT(nodes=[0], start=PortRefT(9))],
        ),
        {
            "connections": [
                {"target": {"index": 3}, "source_role": "ResonatorSource", "target_role": "ResonatorTarget"},
                {
                    "source": {"index": 1},
                    "target": {"index": 2},
                    "source_role": "FeedlineSource",
                    "target_role": "FeedlineTarget",
                },
            ],
            "objective": 132.68,
            "ring": [{"index": 3}, {"index": 2}],
            "launchers": [{"index": 9}, {"index": 1}],
            "feeds": [{"x": 120.5, "y": -40.25}, {"x": 0.0, "y": 0.0}],
            "chains": [{"nodes": [0], "start": {"index": 9}}],
        },
        id="assignment",
    ),
    pytest.param(
        StageOutput.GlobalRouting,
        empty_global(),
        {"lattices": [], "connections": [], "outer_ring": [], "resonators": []},
        id="global",
    ),
    pytest.param(
        StageOutput.CorridorRouting,
        CorridorRoutingT(
            corridors=[
                CorridorT(partitions=[4, 7], crossings=[PointT(10.5, 20.25)], source=PointT(0.5, 1.5)),
                CorridorT(partitions=[], crossings=[]),
            ],
            slots=[BorderSlotsT(border=2, positions=[PointT(10.5, 20.25)])],
        ),
        {
            "corridors": [
                {"partitions": [4, 7], "crossings": [{"x": 10.5, "y": 20.25}], "source": {"x": 0.5, "y": 1.5}},
                {"partitions": [], "crossings": []},
            ],
            "slots": [{"border": 2, "positions": [{"x": 10.5, "y": 20.25}]}],
        },
        id="corridor",
    ),
    pytest.param(StageOutput.DetailRouting, DetailRoutingT(), {}, id="detail"),
    pytest.param(
        StageOutput.FinalRouting,
        FinalRoutingT(
            couplers=[coupler(3)],
            bridges=[BridgeT(center=PointT(5.0, 6.0), rotation=Rotation.R45, width=60.0, height=60.0)],
            unresolved=[ConnectionRefT(7)],
        ),
        {
            "couplers": [
                {
                    "connection": {"index": 3},
                    "port": {
                        "label": "Coupler3.port0",
                        "center": {"x": 1.0, "y": 2.0},
                        "orientation": 90.0,
                        "role": "Coupler",
                    },
                    "center": {"x": 1.0, "y": 2.0},
                    "rotation": "R90",
                    "length": 200.0,
                    "height": 26.0,
                }
            ],
            "bridges": [
                {"center": {"x": 5.0, "y": 6.0}, "rotation": "R45", "width": 60.0, "height": 60.0},
            ],
            "unresolved": [{"index": 7}],
        },
        id="final",
    ),
    pytest.param(
        StageOutput.Geometry,
        GeometryT(
            wires=[
                WireT(
                    connection=ConnectionRefT(0),
                    path=PathT(
                        segments=[
                            SegmentT(
                                shapeType=SegmentShape.Line, shape=LineT(start=PointT(0.0, 0.0), end=PointT(100.0, 0.0))
                            ),
                            SegmentT(
                                shapeType=SegmentShape.Arc,
                                shape=ArcT(center=PointT(100.0, 50.0), radius=50.0, sweep=1.5),
                            ),
                        ]
                    ),
                )
            ],
            couplers=[],
            bridges=[],
        ),
        {
            "wires": [
                {
                    "connection": {"index": 0},
                    "path": {
                        "segments": [
                            {
                                "shape_type": "Line",
                                "shape": {"start": {"x": 0.0, "y": 0.0}, "end": {"x": 100.0, "y": 0.0}},
                            },
                            {
                                "shape_type": "Arc",
                                "shape": {"center": {"x": 100.0, "y": 50.0}, "radius": 50.0, "sweep": 1.5},
                            },
                        ]
                    },
                }
            ],
            "couplers": [],
            "bridges": [],
        },
        id="geometry",
    ),
]


@pytest.mark.parametrize(("output_type", "output", "expected"), STAGE_OUTPUTS)
def test_every_stage_output_renders_the_fields_of_its_schema(output_type: int, output: object, expected: dict) -> None:
    """The JSON of each stage output carries the schema's own field names."""
    document = json.loads(artifact_to_json(artifact(output_type, output)))

    assert document["producer"] == "mqt-scpd test"
    assert document["output_type"] == {value: name for name, value in vars(StageOutput).items()}[output_type]
    assert document["output"] == expected


def test_absent_fields_and_defaults_are_left_out() -> None:
    """A field the artifact does not carry does not appear, so the JSON shows what was written."""
    document = json.loads(
        artifact_to_json(
            artifact(
                StageOutput.Assignment,
                AssignmentT(connections=[], objective=0.0, ring=[], launchers=[], feeds=[], chains=[]),
            )
        )
    )

    assert document["output"] == {"connections": [], "ring": [], "launchers": [], "feeds": [], "chains": []}


def test_bytes_that_are_not_an_artifact_are_refused() -> None:
    """The identifier and the completeness of the artifact are checked before anything is rendered."""
    data = bytearray(artifact(StageOutput.GlobalRouting, empty_global()))
    data[4:8] = b"XXXX"
    with pytest.raises(InspectionError, match="identifier"):
        artifact_to_json(bytes(data))
    with pytest.raises(InspectionError, match="not an artifact"):
        artifact_to_json(b"\xff\xff\xff\xffSCP1")

    # A buffer whose producer is present but empty passes the verifier, and fails the check that
    # every artifact names what wrote it.
    builder = flatbuffers.Builder()
    producer = builder.CreateString("")
    output = empty_global().Pack(builder)
    ArtifactStart(builder)
    ArtifactAddProducer(builder, producer)
    ArtifactAddOutputType(builder, StageOutput.GlobalRouting)
    ArtifactAddOutput(builder, output)
    builder.Finish(ArtifactEnd(builder), file_identifier=IDENTIFIER)
    with pytest.raises(InspectionError, match="producer is empty"):
        artifact_to_json(bytes(builder.Output()))
