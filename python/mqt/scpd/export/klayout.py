# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The KLayout adapter: GDSII and OASIS from the chip model.

The module needs KLayout and therefore imports it, so importing the module fails without it.
Ask :data:`mqt.scpd.export.HAS_KLAYOUT` before importing when the dependency may be absent;
install ``mqt-scpd[klayout]`` to have it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import klayout.db as kdb

from ..chip import obstacles_of, ports_of, role_name, vertices_of

if TYPE_CHECKING:
    from pathlib import Path

    from ..flatbuffers.design.Chip import ChipT

#: The layer that carries the obstacle polygons.
OBSTACLE_LAYER = (1, 0)

#: The layer of the port labels; the datatype is the port's role value.
PORT_LAYER = 2

#: The file suffixes KLayout writes, with the format each selects.
FORMATS: dict[str, str] = {".gds": "GDS2", ".gds2": "GDS2", ".oas": "OASIS"}

#: The database unit in micrometers: the inputs carry three decimals.
DATABASE_UNIT = 0.001


class ExportError(ValueError):
    """A layout that cannot be written, because the file names no supported format."""


@dataclass(frozen=True)
class ExportSummary:
    """What a written layout holds."""

    path: Path
    format: str
    polygons: int
    ports: int


def write_layout(chip: ChipT, path: Path, *, cell: str = "chip") -> ExportSummary:
    """Write the unrouted chip as GDSII or OASIS, chosen by the file suffix.

    The obstacles go to layer 1 as polygons. Every port becomes a text with its label on layer 2,
    with the role's value as the datatype, so that the roles can be shown and hidden separately.

    Args:
        chip: The classified chip.
        path: The file to write; ``.gds``, ``.gds2`` or ``.oas``.
        cell: The name of the top cell.

    Returns:
        A summary of what was written.

    Raises:
        ExportError: If the suffix names no supported format.
    """
    suffix = path.suffix.lower()
    if suffix not in FORMATS:
        msg = f"{path}: the suffix must be one of {', '.join(FORMATS)}"
        raise ExportError(msg)

    layout = kdb.Layout()
    layout.dbu = DATABASE_UNIT
    top = layout.create_cell(cell)
    obstacle_layer = layout.layer(*OBSTACLE_LAYER)
    polygons = 0
    for polygon in obstacles_of(chip):
        vertices = vertices_of(polygon)
        if len(vertices) < 3:
            continue
        top.shapes(obstacle_layer).insert(kdb.DPolygon([kdb.DPoint(v.x, v.y) for v in vertices]))
        polygons += 1
    ports = 0
    for port in ports_of(chip):
        center = port.center
        if center is None:
            continue
        layer = layout.layer(PORT_LAYER, port.role, role_name(port.role))
        top.shapes(layer).insert(kdb.DText(port.label or "", kdb.DTrans(kdb.DVector(center.x, center.y))))
        ports += 1
    layout.write(str(path))
    return ExportSummary(path, FORMATS[suffix], polygons, ports)
