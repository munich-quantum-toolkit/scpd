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
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.DetailRouting import DetailRoutingT
from mqt.scpd.flatbuffers.artifacts.FinalRouting import FinalRoutingT
from mqt.scpd.flatbuffers.artifacts.Geometry import GeometryT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingEnd, GlobalRoutingStart, GlobalRoutingT
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


STAGE_OUTPUTS = [
    pytest.param(StageOutput.CapacityPlan, CapacityPlanT(), {}, id="capacity"),
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
        },
        id="assignment",
    ),
    pytest.param(StageOutput.GlobalRouting, GlobalRoutingT(), {}, id="global"),
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
    """The JSON of each of the six stage outputs carries the schema's own field names."""
    document = json.loads(artifact_to_json(artifact(output_type, output)))

    assert document["producer"] == "mqt-scpd test"
    assert document["output_type"] == {value: name for name, value in vars(StageOutput).items()}[output_type]
    assert document["output"] == expected


def test_absent_fields_and_defaults_are_left_out() -> None:
    """A field the artifact does not carry does not appear, so the JSON shows what was written."""
    document = json.loads(
        artifact_to_json(artifact(StageOutput.Assignment, AssignmentT(connections=[], objective=0.0)))
    )

    assert document["output"] == {"connections": []}


def test_bytes_that_are_not_an_artifact_are_refused() -> None:
    """The identifier and the completeness of the artifact are checked before anything is rendered."""
    data = bytearray(artifact(StageOutput.GlobalRouting, GlobalRoutingT()))
    data[4:8] = b"XXXX"
    with pytest.raises(InspectionError, match="identifier"):
        artifact_to_json(bytes(data))
    with pytest.raises(InspectionError, match="not an artifact"):
        artifact_to_json(b"\xff\xff\xff\xffSCP1")

    # A buffer whose producer is present but empty passes the verifier, and fails the check that
    # every artifact names what wrote it.
    builder = flatbuffers.Builder()
    producer = builder.CreateString("")
    GlobalRoutingStart(builder)
    output = GlobalRoutingEnd(builder)
    ArtifactStart(builder)
    ArtifactAddProducer(builder, producer)
    ArtifactAddOutputType(builder, StageOutput.GlobalRouting)
    ArtifactAddOutput(builder, output)
    builder.Finish(ArtifactEnd(builder), file_identifier=IDENTIFIER)
    with pytest.raises(InspectionError, match="producer is empty"):
        artifact_to_json(bytes(builder.Output()))
