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
from typing import TYPE_CHECKING

import flatbuffers

from .flatbuffers.artifacts.Artifact import Artifact, ArtifactT
from .flatbuffers.artifacts.Assignment import AssignmentT
from .flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from .flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from .flatbuffers.artifacts.FinalRouting import FinalRoutingT
from .flatbuffers.artifacts.Geometry import GeometryT
from .flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from .flatbuffers.artifacts.StageOutput import StageOutput
from .flatbuffers.geometry.Arc import ArcT
from .flatbuffers.geometry.Line import LineT

if TYPE_CHECKING:
    from .flatbuffers.artifacts.BorderSlots import BorderSlotsT
    from .flatbuffers.artifacts.Bottleneck import BottleneckT
    from .flatbuffers.artifacts.CapacityNode import CapacityNodeT
    from .flatbuffers.artifacts.Corridor import CorridorT
    from .flatbuffers.artifacts.FeedlineChain import FeedlineChainT
    from .flatbuffers.artifacts.GridExtent import GridExtentT
    from .flatbuffers.artifacts.Lattice import LatticeT
    from .flatbuffers.artifacts.LauncherSlot import LauncherSlotT
    from .flatbuffers.artifacts.Partition import PartitionT
    from .flatbuffers.artifacts.PartitionBorder import PartitionBorderT
    from .flatbuffers.artifacts.Wire import WireT
    from .flatbuffers.design.Bridge import BridgeT
    from .flatbuffers.design.Connection import ConnectionT
    from .flatbuffers.design.CpwCoupler import CpwCouplerT
    from .flatbuffers.design.Port import PortT
    from .flatbuffers.geometry.Segment import SegmentT

#: The file identifier that records the schema version, as ``artifacts.fbs`` declares it.
IDENTIFIER = b"SCP1"

_OUTPUT_TYPES: dict[int, type] = {
    StageOutput.CapacityPlan: CapacityPlanT,
    StageOutput.GlobalRouting: GlobalRoutingT,
    StageOutput.Assignment: AssignmentT,
    StageOutput.CorridorRouting: CorridorRoutingT,
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
    """Find the required fields of an artifact that are missing.

    Returns:
        One message per missing field, naming the field and its position.
    """
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
    output = artifact.output
    if isinstance(output, CapacityPlanT):
        for grid, name in ((output.capacityGrid, "capacity_grid"), (output.detailGrid, "detail_grid")):
            if grid is None:
                problems.append(f"{name} is missing")
            else:
                problems.extend(f"{name}: {problem}" for problem in _grid_problems(grid))
        _check_list(output.partitions, "partitions", _partition_problems, problems)
        _check_list(output.borders, "borders", _border_problems, problems)
        _check_list(output.bottlenecks, "bottlenecks", _bottleneck_problems, problems)
        _check_list(output.launchers, "launchers", _launcher_problems, problems)
        _check_list(output.nodes, "nodes", _node_problems, problems)
        _require(output.chains, "chains", problems)
    elif isinstance(output, GlobalRoutingT):
        _check_list(output.lattices, "lattices", _lattice_problems, problems)
        _check_list(output.connections, "connections", _connection_problems, problems)
        _require(output.outerRing, "outer_ring", problems)
        _require(output.resonators, "resonators", problems)
    elif isinstance(output, AssignmentT):
        _check_list(output.connections, "connections", _connection_problems, problems)
        _require(output.ring, "ring", problems)
        _require(output.launchers, "launchers", problems)
        _require(output.feeds, "feeds", problems)
        _check_list(output.chains, "chains", _chain_problems, problems)
    elif isinstance(output, CorridorRoutingT):
        _check_list(output.corridors, "corridors", _corridor_problems, problems)
        _check_list(output.slots, "slots", _border_slots_problems, problems)
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


def _require(value: object, name: str, problems: list[str]) -> None:
    """Report a required field that is missing."""
    if value is None:
        problems.append(f"{name} is missing")


def _missing(item: object, fields: tuple[tuple[str, str], ...]) -> list[str]:
    """The required fields of one table that are missing, by the names the schema gives them.

    Returns:
        One message per missing field.
    """
    return [f"{name} is missing" for attribute, name in fields if getattr(item, attribute) is None]


def _grid_problems(grid: GridExtentT) -> list[str]:
    return _missing(grid, (("origin", "origin"),))


def _partition_problems(partition: PartitionT) -> list[str]:
    return _missing(partition, (("outlines", "outlines"),))


def _border_problems(border: PartitionBorderT) -> list[str]:
    return _missing(border, (("samples", "samples"), ("center", "center")))


def _bottleneck_problems(bottleneck: BottleneckT) -> list[str]:
    return _missing(bottleneck, (("from_", "from"), ("to", "to")))


def _launcher_problems(slot: LauncherSlotT) -> list[str]:
    return _missing(slot, (("port", "port"), ("position", "position")))


def _node_problems(node: CapacityNodeT) -> list[str]:
    return _missing(node, (("next", "next"),))


def _lattice_problems(lattice: LatticeT) -> list[str]:
    return _missing(lattice, (("points", "points"), ("edges", "edges"), ("selected", "selected")))


def _chain_problems(chain: FeedlineChainT) -> list[str]:
    return _missing(chain, (("nodes", "nodes"),))


def _corridor_problems(corridor: CorridorT) -> list[str]:
    problems = _missing(corridor, (("partitions", "partitions"), ("crossings", "crossings")))
    if problems:
        return problems
    partitions = len(corridor.partitions or [])
    crossings = len(corridor.crossings or [])
    # A wire crosses one border fewer than it runs through partitions, and a wire without a way
    # names neither.
    if partitions != crossings + 1 and (partitions, crossings) != (0, 0):
        return [f"names {partitions} partitions and {crossings} crossings"]
    return []


def _border_slots_problems(slots: BorderSlotsT) -> list[str]:
    return _missing(slots, (("positions", "positions"),))


def _check_list(items: list | None, name: str, check, problems: list[str]) -> None:  # ruff: ignore[missing-type-function-argument]
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
