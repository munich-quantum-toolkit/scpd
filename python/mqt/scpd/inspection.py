# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The ``inspect`` command: a stage artifact as schema-driven JSON, and back.

Binary does not mean opaque. Every artifact converts to JSON whose keys are the fields of the
schema, with enums and union tags spelled by name, and the JSON converts back into the same
artifact. The field map below mirrors the schemas that the artifacts use; a field added to a schema
is added here.
"""

from __future__ import annotations

import json
from typing import Any

from .artifacts import read_artifact, write_artifact
from .flatbuffers.artifacts.Artifact import ArtifactT
from .flatbuffers.artifacts.Assignment import AssignmentT
from .flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from .flatbuffers.artifacts.DetailRouting import DetailRoutingT
from .flatbuffers.artifacts.FinalRouting import FinalRoutingT
from .flatbuffers.artifacts.Geometry import GeometryT
from .flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from .flatbuffers.artifacts.StageOutput import StageOutput
from .flatbuffers.artifacts.Wire import WireT
from .flatbuffers.design.AssignedRole import AssignedRole
from .flatbuffers.design.Bridge import BridgeT
from .flatbuffers.design.Chip import ChipT
from .flatbuffers.design.Connection import ConnectionT
from .flatbuffers.design.ConnectionRef import ConnectionRefT
from .flatbuffers.design.CpwCoupler import CpwCouplerT
from .flatbuffers.design.DesignRules import DesignRulesT
from .flatbuffers.design.Port import PortT
from .flatbuffers.design.PortRef import PortRefT
from .flatbuffers.design.Rotation import Rotation
from .flatbuffers.design.UnassignedRole import UnassignedRole
from .flatbuffers.geometry.Arc import ArcT
from .flatbuffers.geometry.Line import LineT
from .flatbuffers.geometry.Path import PathT
from .flatbuffers.geometry.Point import PointT
from .flatbuffers.geometry.Polygon import PolygonT
from .flatbuffers.geometry.Segment import SegmentT
from .flatbuffers.geometry.SegmentShape import SegmentShape


class InspectionError(ValueError):
    """JSON that does not describe an artifact."""


# A field is described by its kind: a scalar type, ("enum", class), ("table", class),
# ("list", inner field), or ("union", tag field, {tag: class}).
Field = Any

_POINT: Field = ("table", PointT)
FIELDS: dict[type, dict[str, Field]] = {
    PointT: {"x": float, "y": float},
    PolygonT: {"vertices": ("list", _POINT)},
    LineT: {"start": _POINT, "end": _POINT},
    ArcT: {"center": _POINT, "radius": float, "startAngle": float, "sweep": float},
    SegmentT: {
        "shapeType": ("enum", SegmentShape),
        "shape": ("union", "shapeType", {SegmentShape.Line: LineT, SegmentShape.Arc: ArcT}),
    },
    PathT: {"segments": ("list", ("table", SegmentT))},
    PortRefT: {"index": int},
    ConnectionRefT: {"index": int},
    PortT: {"label": str, "center": _POINT, "orientation": float, "role": ("enum", UnassignedRole)},
    ChipT: {"obstacles": ("list", ("table", PolygonT)), "ports": ("list", ("table", PortT))},
    ConnectionT: {
        "source": ("table", PortRefT),
        "target": ("table", PortRefT),
        "sourceRole": ("enum", AssignedRole),
        "targetRole": ("enum", AssignedRole),
    },
    DesignRulesT: {
        "minWireSpacing": float,
        "minObstacleSpacing": float,
        "minBendRadius": float,
        "minStraightLength": float,
        "targetResonatorLength": float,
        "resonatorLengthTolerance": float,
        "maxFeedlineUtilization": int,
        "feedlineTerminations": int,
    },
    CpwCouplerT: {
        "connection": ("table", ConnectionRefT),
        "port": ("table", PortT),
        "center": _POINT,
        "rotation": ("enum", Rotation),
        "length": float,
        "height": float,
    },
    BridgeT: {"center": _POINT, "rotation": ("enum", Rotation), "width": float, "height": float},
    CapacityPlanT: {},
    AssignmentT: {"connections": ("list", ("table", ConnectionT)), "objective": float},
    GlobalRoutingT: {},
    DetailRoutingT: {},
    FinalRoutingT: {
        "couplers": ("list", ("table", CpwCouplerT)),
        "bridges": ("list", ("table", BridgeT)),
        "unresolved": ("list", ("table", ConnectionRefT)),
    },
    WireT: {"connection": ("table", ConnectionRefT), "path": ("table", PathT)},
    GeometryT: {
        "wires": ("list", ("table", WireT)),
        "couplers": ("list", ("table", CpwCouplerT)),
        "bridges": ("list", ("table", BridgeT)),
    },
    ArtifactT: {
        "producer": str,
        "outputType": ("enum", StageOutput),
        "output": (
            "union",
            "outputType",
            {
                StageOutput.CapacityPlan: CapacityPlanT,
                StageOutput.Assignment: AssignmentT,
                StageOutput.GlobalRouting: GlobalRoutingT,
                StageOutput.DetailRouting: DetailRoutingT,
                StageOutput.FinalRouting: FinalRoutingT,
                StageOutput.Geometry: GeometryT,
            },
        ),
    },
}


def _enum_names(enum: type) -> dict[int, str]:
    return {value: name for name, value in vars(enum).items() if isinstance(value, int) and not name.startswith("_")}


def _to_value(value: Any, kind: Field, owner: Any) -> Any:
    if value is None:
        return None
    if kind in {float, int, str}:
        return value
    tag, *rest = kind
    if tag == "enum":
        return _enum_names(rest[0]).get(value, value)
    if tag == "table":
        return to_dict(value)
    if tag == "list":
        return [_to_value(item, rest[0], owner) for item in value]
    return to_dict(value)


def to_dict(obj: Any) -> dict[str, Any]:
    """Convert an object of the generated model to a JSON-ready dictionary.

    Returns:
        The dictionary, with the schema's field names as keys.
    """
    fields = FIELDS[type(obj)]
    return {name: _to_value(getattr(obj, name), kind, obj) for name, kind in fields.items()}


def _from_value(value: Any, kind: Field, where: str, owner: dict[str, Any]) -> Any:
    if value is None:
        return None
    if kind in {float, int, str}:
        if not isinstance(value, kind) or (kind is not str and isinstance(value, bool)):
            msg = f"{where} must be {kind.__name__}"
            raise InspectionError(msg)
        return value
    tag, *rest = kind
    if tag == "enum":
        names = {name: number for number, name in _enum_names(rest[0]).items()}
        if isinstance(value, str) and value in names:
            return names[value]
        if isinstance(value, int) and value in _enum_names(rest[0]):
            return value
        msg = f"{where} must be one of {sorted(names)}"
        raise InspectionError(msg)
    if tag == "table":
        return from_dict(value, rest[0], where)
    if tag == "list":
        if not isinstance(value, list):
            msg = f"{where} must be a list"
            raise InspectionError(msg)
        return [_from_value(item, rest[0], f"{where}[{i}]", owner) for i, item in enumerate(value)]
    tag_field, classes = rest
    tag_value = owner.get(tag_field)
    names = {name: number for number, name in _enum_names(_union_enum(classes)).items()}
    number = names.get(tag_value, tag_value)
    if number not in classes:
        msg = f"{where} has no type tag in {tag_field}"
        raise InspectionError(msg)
    return from_dict(value, classes[number], where)


def _union_enum(classes: dict[int, type]) -> type:
    return StageOutput if AssignmentT in classes.values() else SegmentShape


def from_dict(data: Any, cls: type, where: str = "artifact") -> Any:
    """Rebuild an object of the generated model from its dictionary.

    Args:
        data: The dictionary, as ``to_dict`` produces it.
        cls: The generated class to build.
        where: How to name the value in messages.

    Returns:
        The object.

    Raises:
        InspectionError: If the dictionary does not fit the schema.
    """
    if not isinstance(data, dict):
        msg = f"{where} must be an object"
        raise InspectionError(msg)
    fields = FIELDS[cls]
    unknown = set(data) - set(fields)
    if unknown:
        msg = f"{where} has unknown fields: {', '.join(sorted(unknown))}"
        raise InspectionError(msg)
    obj = cls()
    for name, kind in fields.items():
        if name in data:
            setattr(obj, name, _from_value(data[name], kind, f"{where}.{name}", data))
    return obj


def artifact_to_json(data: bytes) -> str:
    """Print the bytes of a stage artifact as JSON.

    Returns:
        The JSON text, indented.

    Raises:
        ArtifactError: If the bytes are not a complete artifact.
    """
    return json.dumps(to_dict(read_artifact(data)), indent=2) + "\n"


def artifact_from_json(text: str) -> bytes:
    """Rebuild the bytes of a stage artifact from its JSON.

    Returns:
        The artifact bytes.

    Raises:
        InspectionError: If the text is not JSON that describes an artifact.
        ArtifactError: If the described artifact is not complete.
    """
    try:
        data = json.loads(text)
    except json.JSONDecodeError as error:
        msg = f"not JSON: {error}"
        raise InspectionError(msg) from error
    return write_artifact(from_dict(data, ArtifactT))
