# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the checked artifact layer."""

from __future__ import annotations

import flatbuffers
import pytest

from mqt.scpd.artifacts import IDENTIFIER, ArtifactError, read_artifact, write_artifact
from mqt.scpd.flatbuffers.artifacts.Artifact import (
    ArtifactAddOutput,
    ArtifactAddOutputType,
    ArtifactEnd,
    ArtifactStart,
    ArtifactT,
)
from mqt.scpd.flatbuffers.artifacts.Assignment import AssignmentT
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


def wrap(output_type: int, output: object) -> ArtifactT:
    """Put a stage output behind an artifact root that has a producer.

    Returns:
        The artifact.
    """
    return ArtifactT(producer="mqt-scpd test", outputType=output_type, output=output)


def make_coupler(connection: int) -> CpwCouplerT:
    """Build a coupler that completes the given connection with the port it creates.

    Returns:
        The coupler.
    """
    return CpwCouplerT(
        connection=ConnectionRefT(index=connection),
        port=PortT(
            label=f"Coupler{connection}.port0",
            center=PointT(1200.0, 800.0),
            orientation=90.0,
            role=UnassignedRole.Coupler,
        ),
        center=PointT(1200.0, 800.0),
        rotation=Rotation.R90,
        length=200.0,
        height=26.0,
    )


def test_round_trip_keeps_the_producer_and_the_identifier() -> None:
    """A written artifact starts with the identifier and reads back complete."""
    data = write_artifact(wrap(StageOutput.GlobalRouting, GlobalRoutingT()))

    assert data[4:8] == IDENTIFIER
    back = read_artifact(data)
    assert back.producer == "mqt-scpd test"
    assert back.outputType == StageOutput.GlobalRouting
    assert isinstance(back.output, GlobalRoutingT)


def test_final_routing_keeps_the_created_ports_and_the_failures() -> None:
    """A coupler carries the port it creates; the failure list survives with zero, one and several entries."""
    for unresolved in ([], [7], [2, 5, 11]):
        routing = FinalRoutingT(
            couplers=[make_coupler(3)],
            bridges=[BridgeT(center=PointT(50.0, 60.0), rotation=Rotation.R45, width=60.0, height=60.0)],
            unresolved=[ConnectionRefT(index=index) for index in unresolved],
        )

        back = read_artifact(write_artifact(wrap(StageOutput.FinalRouting, routing)))

        assert isinstance(back.output, FinalRoutingT)
        coupler = back.output.couplers[0]
        assert coupler.connection is not None
        assert coupler.connection.index == 3
        assert coupler.port is not None
        assert (coupler.port.label, coupler.port.role) == ("Coupler3.port0", UnassignedRole.Coupler)
        assert [ref.index for ref in back.output.unresolved] == unresolved


def test_geometry_keeps_analytic_segments() -> None:
    """A wire is stored as lines and arcs, never as sampled points."""
    wire = WireT(
        connection=ConnectionRefT(index=0),
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
    geometry = GeometryT(wires=[wire], couplers=[make_coupler(0)], bridges=[])

    back = read_artifact(write_artifact(wrap(StageOutput.Geometry, geometry)))

    assert isinstance(back.output, GeometryT)
    path = back.output.wires[0].path
    assert path is not None
    assert [segment.shapeType for segment in path.segments] == [SegmentShape.Line, SegmentShape.Arc]
    assert isinstance(path.segments[1].shape, ArcT)
    assert path.segments[1].shape.radius == pytest.approx(50.0)


def test_write_rejects_a_missing_producer_or_output() -> None:
    """The checked layer refuses to write what the core would refuse to read."""
    with pytest.raises(ArtifactError, match="producer is missing"):
        write_artifact(ArtifactT(outputType=StageOutput.GlobalRouting, output=GlobalRoutingT()))
    with pytest.raises(ArtifactError, match="output is missing"):
        write_artifact(ArtifactT(producer="mqt-scpd test"))


def test_write_rejects_an_incomplete_component() -> None:
    """A required field deep inside the output is reported with its path."""
    coupler = make_coupler(0)
    coupler.port = None
    routing = FinalRoutingT(couplers=[coupler], bridges=[], unresolved=[])

    with pytest.raises(ArtifactError, match=r"couplers\[0\]: port is missing"):
        write_artifact(wrap(StageOutput.FinalRouting, routing))


def test_read_rejects_a_buffer_without_its_producer() -> None:
    """A buffer the generated builder writes without the producer is not an artifact."""
    builder = flatbuffers.Builder()
    GlobalRoutingStart(builder)
    output = GlobalRoutingEnd(builder)
    ArtifactStart(builder)
    ArtifactAddOutputType(builder, StageOutput.GlobalRouting)
    ArtifactAddOutput(builder, output)
    builder.Finish(ArtifactEnd(builder), file_identifier=IDENTIFIER)

    with pytest.raises(ArtifactError, match="producer is missing"):
        read_artifact(bytes(builder.Output()))


def test_read_rejects_foreign_bytes() -> None:
    """Bytes without the identifier, or too short to decode, are not an artifact."""
    data = bytearray(write_artifact(wrap(StageOutput.GlobalRouting, GlobalRoutingT())))
    data[4:8] = b"XXXX"
    with pytest.raises(ArtifactError, match="identifier"):
        read_artifact(bytes(data))
    with pytest.raises(ArtifactError):
        read_artifact(b"SCP1")


def test_assignment_round_trips() -> None:
    """The connections of an assignment survive with their roles."""
    assignment = AssignmentT(
        connections=[
            ConnectionT(
                target=PortRefT(index=3),
                sourceRole=AssignedRole.ResonatorSource,
                targetRole=AssignedRole.ResonatorTarget,
            )
        ],
        objective=132.68,
    )

    back = read_artifact(write_artifact(wrap(StageOutput.Assignment, assignment)))

    assert isinstance(back.output, AssignmentT)
    assert back.output.connections[0].sourceRole == AssignedRole.ResonatorSource
    assert back.output.objective == pytest.approx(132.68)


def _make_geometry(segment: SegmentT) -> GeometryT:
    """Build a geometry with one wire that carries the given segment.

    Returns:
        The geometry.
    """
    return GeometryT(
        wires=[WireT(connection=ConnectionRefT(index=0), path=PathT(segments=[segment]))],
        couplers=[],
        bridges=[],
    )


def _without_port_label() -> CpwCouplerT:
    coupler = make_coupler(0)
    assert coupler.port is not None
    coupler.port.label = None
    return coupler


def _without_port_center() -> CpwCouplerT:
    coupler = make_coupler(0)
    assert coupler.port is not None
    coupler.port.center = None
    return coupler


INCOMPLETE_ARTIFACTS = [
    pytest.param(
        ArtifactT(producer="p", outputType=StageOutput.Assignment, output=GlobalRoutingT()),
        "output does not match its type tag",
        id="type-tag",
    ),
    pytest.param(
        wrap(StageOutput.Assignment, AssignmentT(connections=[ConnectionT()])),
        r"connections\[0\]: target is missing",
        id="connection-target",
    ),
    pytest.param(
        wrap(StageOutput.FinalRouting, FinalRoutingT(couplers=None, bridges=[], unresolved=[])),
        "couplers is missing",
        id="list",
    ),
    pytest.param(
        wrap(StageOutput.FinalRouting, FinalRoutingT(couplers=[], bridges=[], unresolved=None)),
        "unresolved is missing",
        id="unresolved",
    ),
    pytest.param(
        wrap(
            StageOutput.FinalRouting,
            FinalRoutingT(
                couplers=[CpwCouplerT(port=make_coupler(0).port, center=PointT(0.0, 0.0))], bridges=[], unresolved=[]
            ),
        ),
        r"couplers\[0\]: connection is missing",
        id="coupler-connection",
    ),
    pytest.param(
        wrap(
            StageOutput.FinalRouting,
            FinalRoutingT(
                couplers=[CpwCouplerT(connection=ConnectionRefT(), port=make_coupler(0).port)],
                bridges=[],
                unresolved=[],
            ),
        ),
        r"couplers\[0\]: center is missing",
        id="coupler-center",
    ),
    pytest.param(
        wrap(StageOutput.FinalRouting, FinalRoutingT(couplers=[_without_port_label()], bridges=[], unresolved=[])),
        r"couplers\[0\]: port: label is missing",
        id="port-label",
    ),
    pytest.param(
        wrap(StageOutput.FinalRouting, FinalRoutingT(couplers=[_without_port_center()], bridges=[], unresolved=[])),
        r"couplers\[0\]: port: center is missing",
        id="port-center",
    ),
    pytest.param(
        wrap(StageOutput.FinalRouting, FinalRoutingT(couplers=[], bridges=[BridgeT()], unresolved=[])),
        r"bridges\[0\]: center is missing",
        id="bridge-center",
    ),
    pytest.param(
        wrap(StageOutput.Geometry, GeometryT(wires=[WireT(path=PathT(segments=[]))], couplers=[], bridges=[])),
        r"wires\[0\]: connection is missing",
        id="wire-connection",
    ),
    pytest.param(
        wrap(StageOutput.Geometry, GeometryT(wires=[WireT(connection=ConnectionRefT())], couplers=[], bridges=[])),
        r"wires\[0\]: path is missing",
        id="wire-path",
    ),
    pytest.param(
        wrap(StageOutput.Geometry, _make_geometry(SegmentT(shapeType=SegmentShape.Line))),
        r"wires\[0\]: segment 0: shape is missing",
        id="segment-shape",
    ),
    pytest.param(
        wrap(
            StageOutput.Geometry,
            _make_geometry(SegmentT(shapeType=SegmentShape.Line, shape=LineT(start=PointT(0.0, 0.0)))),
        ),
        r"wires\[0\]: segment 0: end is missing",
        id="line-end",
    ),
    pytest.param(
        wrap(StageOutput.Geometry, _make_geometry(SegmentT(shapeType=SegmentShape.Arc, shape=ArcT(radius=1.0)))),
        r"wires\[0\]: segment 0: center is missing",
        id="arc-center",
    ),
    pytest.param(
        wrap(StageOutput.Geometry, _make_geometry(SegmentT(shapeType=SegmentShape.Line, shape=PointT(0.0, 0.0)))),
        r"wires\[0\]: segment 0: shape does not match its type tag",
        id="segment-tag",
    ),
]


@pytest.mark.parametrize(("artifact", "message"), INCOMPLETE_ARTIFACTS)
def test_write_names_every_missing_required_field(artifact: ArtifactT, message: str) -> None:
    """Each required field the schema declares is reported with its position when it is missing."""
    with pytest.raises(ArtifactError, match=message):
        write_artifact(artifact)


def test_read_rejects_bytes_that_do_not_decode() -> None:
    """A buffer with the identifier but a root offset outside the buffer is not an artifact."""
    with pytest.raises(ArtifactError, match="not an artifact"):
        read_artifact(b"\xff\xff\xff\xff" + IDENTIFIER)
