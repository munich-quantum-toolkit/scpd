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

import pytest

from mqt.scpd.artifacts import write_artifact
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.DetailRouting import DetailRoutingT
from mqt.scpd.flatbuffers.artifacts.FinalRouting import FinalRoutingT
from mqt.scpd.flatbuffers.artifacts.Geometry import GeometryT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
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
from mqt.scpd.inspection import InspectionError, artifact_from_json, artifact_to_json, from_dict, to_dict


def coupler(connection: int) -> CpwCouplerT:
    """A coupler that completes one connection.

    Returns:
        The coupler.
    """
    return CpwCouplerT(
        connection=ConnectionRefT(index=connection),
        port=PortT(label=f"Coupler{connection}.port0", center=PointT(1.0, 2.0), orientation=90.0, role=UnassignedRole.Coupler),
        center=PointT(1.0, 2.0),
        rotation=Rotation.R90,
        length=200.0,
        height=26.0,
    )


def artifact(output_type: int, output: object) -> ArtifactT:
    """An artifact around one stage output.

    Returns:
        The artifact.
    """
    return ArtifactT(producer="mqt-scpd test", outputType=output_type, output=output)


STAGE_OUTPUTS = [
    pytest.param(StageOutput.CapacityPlan, CapacityPlanT(), id="capacity"),
    pytest.param(
        StageOutput.Assignment,
        AssignmentT(
            connections=[
                ConnectionT(target=PortRefT(3), sourceRole=AssignedRole.ResonatorSource, targetRole=AssignedRole.ResonatorTarget),
                ConnectionT(
                    source=PortRefT(1),
                    target=PortRefT(2),
                    sourceRole=AssignedRole.FeedlineSource,
                    targetRole=AssignedRole.FeedlineTarget,
                ),
            ],
            objective=132.68,
        ),
        id="assignment",
    ),
    pytest.param(StageOutput.GlobalRouting, GlobalRoutingT(), id="global"),
    pytest.param(StageOutput.DetailRouting, DetailRoutingT(), id="detail"),
    pytest.param(
        StageOutput.FinalRouting,
        FinalRoutingT(
            couplers=[coupler(3)],
            bridges=[BridgeT(center=PointT(5.0, 6.0), rotation=Rotation.R45, width=60.0, height=60.0)],
            unresolved=[ConnectionRefT(7)],
        ),
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
                            SegmentT(shapeType=SegmentShape.Line, shape=LineT(start=PointT(0.0, 0.0), end=PointT(100.0, 0.0))),
                            SegmentT(shapeType=SegmentShape.Arc, shape=ArcT(center=PointT(100.0, 50.0), radius=50.0, sweep=1.5)),
                        ]
                    ),
                )
            ],
            couplers=[coupler(0)],
            bridges=[],
        ),
        id="geometry",
    ),
]


@pytest.mark.parametrize(("output_type", "output"), STAGE_OUTPUTS)
def test_every_stage_output_round_trips_through_json(output_type: int, output: object) -> None:
    """The JSON of an artifact rebuilds the same bytes, for each of the six stage outputs."""
    data = write_artifact(artifact(output_type, output))

    text = artifact_to_json(data)
    assert artifact_from_json(text) == data

    document = json.loads(text)
    assert document["producer"] == "mqt-scpd test"
    assert document["outputType"] == {value: name for name, value in vars(StageOutput).items()}[output_type]


def test_enums_and_union_tags_are_spelled_by_name() -> None:
    """The JSON names roles, rotations and shapes instead of numbering them."""
    document = to_dict(artifact(StageOutput.FinalRouting, FinalRoutingT(couplers=[coupler(3)], bridges=[], unresolved=[])))

    assert document["outputType"] == "FinalRouting"
    assert document["output"]["couplers"][0]["rotation"] == "R90"
    assert document["output"]["couplers"][0]["port"]["role"] == "Coupler"
    assert document["output"]["couplers"][0]["port"]["center"] == {"x": 1.0, "y": 2.0}


@pytest.mark.parametrize(
    ("document", "message"),
    [
        ({"producer": "p", "outputType": "Assignment", "output": {"objective": "high"}}, "output.objective must be float"),
        ({"producer": "p", "outputType": "Assignment", "output": {"connections": 3}}, "output.connections must be a list"),
        ({"producer": "p", "outputType": "Sideways", "output": {}}, "outputType must be one of"),
        ({"producer": "p", "output": {}}, "output has no type tag in outputType"),
        ({"producer": "p", "outputType": "GlobalRouting", "output": {}, "extra": 1}, "unknown fields: extra"),
        ([], "artifact must be an object"),
    ],
)
def test_json_that_does_not_fit_the_schema_is_refused(document: object, message: str) -> None:
    """A wrong type, a wrong name and an unknown field are named with their position."""
    with pytest.raises(InspectionError, match=message):
        from_dict(document, ArtifactT)
    with pytest.raises(InspectionError, match="not JSON"):
        artifact_from_json("{")
