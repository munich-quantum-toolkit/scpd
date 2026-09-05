# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Checked reading and writing of stage artifacts.

The generated FlatBuffers code neither enforces the required fields when Python writes a buffer nor
verifies a buffer when Python reads one; a buffer without a producer or an output passes through it
silently. The functions here check the identifier and every field the schema marks as required, so
an artifact that Python writes is one that the core accepts, and an artifact that Python reads is
complete. Checking the values themselves, such as a role left unset or a dimension of zero, is the
core's job.
"""

from __future__ import annotations

import struct

import flatbuffers

from .flatbuffers.artifacts.Artifact import Artifact, ArtifactT
from .flatbuffers.artifacts.Assignment import AssignmentT
from .flatbuffers.artifacts.FinalRouting import FinalRoutingT
from .flatbuffers.artifacts.Geometry import GeometryT
from .flatbuffers.artifacts.StageOutput import StageOutput
from .flatbuffers.artifacts.Wire import WireT
from .flatbuffers.design.Bridge import BridgeT
from .flatbuffers.design.Connection import ConnectionT
from .flatbuffers.design.CpwCoupler import CpwCouplerT
from .flatbuffers.design.Port import PortT
from .flatbuffers.geometry.Arc import ArcT
from .flatbuffers.geometry.Line import LineT
from .flatbuffers.geometry.Segment import SegmentT

#: The file identifier that records the schema version, as ``artifacts.fbs`` declares it.
IDENTIFIER = b"SCP1"

_OUTPUT_TYPES: dict[int, type] = {
    StageOutput.Assignment: AssignmentT,
    StageOutput.FinalRouting: FinalRoutingT,
    StageOutput.Geometry: GeometryT,
}


class ArtifactError(ValueError):
    """Bytes that are not a complete artifact of this schema, or an artifact that is not complete."""


def write_artifact(artifact: ArtifactT) -> bytes:
    """Serialize an artifact as a run directory stores it.

    Args:
        artifact: The artifact to serialize.

    Returns:
        The bytes of the artifact, starting with the schema identifier.

    Raises:
        ArtifactError: If a required field is missing.
    """
    problems = _problems(artifact)
    if problems:
        msg = f"artifact is not complete: {'; '.join(problems)}"
        raise ArtifactError(msg)
    builder = flatbuffers.Builder()
    builder.Finish(artifact.Pack(builder), file_identifier=IDENTIFIER)
    return bytes(builder.Output())


def read_artifact(data: bytes) -> ArtifactT:
    """Read an artifact from the bytes a run directory stores.

    Args:
        data: The bytes of the artifact.

    Returns:
        The artifact as a native object.

    Raises:
        ArtifactError: If the bytes do not carry the schema identifier, cannot be decoded, or omit a
            required field.
    """
    if not Artifact.ArtifactBufferHasIdentifier(data, 0, size_prefixed=False):
        msg = f"bytes are not an artifact of this schema: the identifier is not {IDENTIFIER!r}"
        raise ArtifactError(msg)
    try:
        artifact = ArtifactT.InitFromPackedBuf(data)
    except (IndexError, struct.error, UnicodeDecodeError) as error:
        msg = f"bytes are not an artifact of this schema: {error}"
        raise ArtifactError(msg) from error
    problems = _problems(artifact)
    if problems:
        msg = f"artifact is not complete: {'; '.join(problems)}"
        raise ArtifactError(msg)
    return artifact


def _problems(artifact: ArtifactT) -> list[str]:
    """The required fields of an artifact that are missing, as messages."""
    problems: list[str] = []
    if not artifact.producer:
        problems.append("producer is missing")
    if artifact.outputType == StageOutput.NONE or artifact.output is None:
        problems.append("output is missing")
        return problems
    expected = _OUTPUT_TYPES.get(artifact.outputType)
    if expected is not None and not isinstance(artifact.output, expected):
        problems.append("output does not match its type tag")
        return problems
    if isinstance(artifact.output, AssignmentT):
        _check_list(artifact.output.connections, "connections", _connection_problems, problems)
    elif isinstance(artifact.output, FinalRoutingT):
        _check_list(artifact.output.couplers, "couplers", _coupler_problems, problems)
        _check_list(artifact.output.bridges, "bridges", _bridge_problems, problems)
        if artifact.output.unresolved is None:
            problems.append("unresolved is missing")
    elif isinstance(artifact.output, GeometryT):
        _check_list(artifact.output.wires, "wires", _wire_problems, problems)
        _check_list(artifact.output.couplers, "couplers", _coupler_problems, problems)
        _check_list(artifact.output.bridges, "bridges", _bridge_problems, problems)
    return problems


def _check_list(items: list | None, name: str, check, problems: list[str]) -> None:  # noqa: ANN001
    """Report a missing list, then the problems of each of its items with the item's index."""
    if items is None:
        problems.append(f"{name} is missing")
        return
    for index, item in enumerate(items):
        problems.extend(f"{name}[{index}]: {problem}" for problem in check(item))


def _connection_problems(connection: ConnectionT) -> list[str]:
    return ["target is missing"] if connection.target is None else []


def _port_problems(port: PortT) -> list[str]:
    problems = []
    if not port.label:
        problems.append("label is missing")
    if port.center is None:
        problems.append("center is missing")
    return problems


def _coupler_problems(coupler: CpwCouplerT) -> list[str]:
    problems = []
    if coupler.connection is None:
        problems.append("connection is missing")
    if coupler.port is None:
        problems.append("port is missing")
    else:
        problems.extend(f"port: {problem}" for problem in _port_problems(coupler.port))
    if coupler.center is None:
        problems.append("center is missing")
    return problems


def _bridge_problems(bridge: BridgeT) -> list[str]:
    return ["center is missing"] if bridge.center is None else []


def _wire_problems(wire: WireT) -> list[str]:
    problems = []
    if wire.connection is None:
        problems.append("connection is missing")
    if wire.path is None or wire.path.segments is None:
        problems.append("path is missing")
        return problems
    for index, segment in enumerate(wire.path.segments):
        problems.extend(f"segment {index}: {problem}" for problem in _segment_problems(segment))
    return problems


def _segment_problems(segment: SegmentT) -> list[str]:
    shape = segment.shape
    if shape is None:
        return ["shape is missing"]
    if isinstance(shape, LineT):
        return [f"{end} is missing" for end in ("start", "end") if getattr(shape, end) is None]
    if isinstance(shape, ArcT):
        return ["center is missing"] if shape.center is None else []
    return ["shape does not match its type tag"]
