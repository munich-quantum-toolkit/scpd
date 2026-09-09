# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The KLayout adapter: GDSII and OASIS from the chip model.

KLayout is imported when a file is written, so that the package imports without it. Install
``mqt-scpd[klayout]`` to render layouts.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import starmap
from typing import TYPE_CHECKING

from ..chip import obstacles_of, ports_of, role_name, vertices_of

if TYPE_CHECKING:
    from pathlib import Path

    from ..flatbuffers.design.Chip import ChipT
    from ..planning import PlanningGeometry

#: The layer that carries the obstacle polygons.
OBSTACLE_LAYER = (1, 0)

#: The layer of the port labels; the datatype is the port's role value.
PORT_LAYER = 2

#: The layer each planning shape goes on, with the name KLayout shows it under. They start at 10
#: so that the artwork keeps the layers it has and the overlay can be switched off as a block.
#:
#: The planning stages are not manufacturable geometry and do not claim to be. They are here
#: because a partition border that exists only in an SVG cannot be measured against the artwork it
#: has to respect, and KLayout is where that measuring happens.
PLANNING_LAYERS: dict[str, tuple[int, int, str]] = {
    "keepout": (9, 0, "plan.port-keepout"),
    "partitions": (10, 0, "plan.partition"),
    "borders": (11, 0, "plan.border"),
    "bottlenecks": (12, 0, "plan.bottleneck"),
    "launchers": (13, 0, "plan.launcher"),
    "chains": (14, 0, "plan.chain"),
    "lattice": (15, 0, "plan.lattice"),
    "inner": (16, 0, "plan.inner-circuit"),
    "assignments": (17, 0, "plan.assignment"),
    "ring": (18, 0, "plan.ring"),
    "corridors": (19, 0, "plan.corridor"),
    "slots": (20, 0, "plan.slot"),
}

#: How wide a planning line is drawn, in layout units. A path needs a width to be a shape at all;
#: this one is chosen to be visible beside the artwork and carries no design meaning.
PLANNING_LINE_WIDTH = 20.0

#: The radius of a launcher marker, in layout units.
PLANNING_MARKER_RADIUS = 60.0

#: The file suffixes KLayout writes, with the format each selects.
FORMATS: dict[str, str] = {".gds": "GDS2", ".gds2": "GDS2", ".oas": "OASIS"}

#: The database unit in micrometers: the inputs carry three decimals.
DATABASE_UNIT = 0.001


class ExportError(RuntimeError):
    """A layout that cannot be written."""


@dataclass(frozen=True)
class ExportSummary:
    """What a written layout holds."""

    path: Path
    format: str
    polygons: int
    ports: int
    #: The planning shapes written beside the artwork, zero when none were.
    planning: int = 0


def write_layout(
    chip: ChipT,
    path: Path,
    *,
    cell: str = "chip",
    planning: PlanningGeometry | None = None,
) -> ExportSummary:
    """Write the chip as GDSII or OASIS, chosen by the file suffix.

    The obstacles go to layer 1 as polygons. Every port becomes a text with its label on layer 2,
    with the role's value as the datatype, so that the roles can be shown and hidden separately.
    What a planning stage produced goes on the layers of ``PLANNING_LAYERS``, one per kind of
    shape, above the artwork and switchable off.

    Args:
        chip: The classified chip.
        path: The file to write; ``.gds``, ``.gds2`` or ``.oas``.
        cell: The name of the top cell.
        planning: What a planning stage produced, written beside the artwork.

    Returns:
        A summary of what was written.

    Raises:
        ExportError: If KLayout is not installed or the suffix names no supported format.
    """
    suffix = path.suffix.lower()
    if suffix not in FORMATS:
        msg = f"{path}: the suffix must be one of {', '.join(FORMATS)}"
        raise ExportError(msg)
    try:
        import klayout.db as kdb  # noqa: PLC0415
    except ImportError as error:
        msg = "KLayout is not installed; install mqt-scpd[klayout] to write layouts"
        raise ExportError(msg) from error

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

    shapes = _write_planning(kdb, layout, top, planning) if planning is not None else 0
    layout.write(str(path))
    return ExportSummary(path, FORMATS[suffix], polygons, ports, shapes)


def _write_planning(kdb, layout, top, planning: PlanningGeometry) -> int:  # noqa: ANN001
    """Write the shapes of one planning stage onto their own layers.

    A partition is a polygon and everything else is a path, because that is what each of them
    actually is: an area of the free space, or a line across or along it.

    Returns:
        How many shapes were written.
    """

    def layer(name: str) -> int:
        number, datatype, label = PLANNING_LAYERS[name]
        return layout.layer(number, datatype, label)

    def path(points: list[tuple[float, float]]) -> object:
        return kdb.DPath(list(starmap(kdb.DPoint, points)), PLANNING_LINE_WIDTH)

    written = 0
    for ring in planning.keepout:
        top.shapes(layer("keepout")).insert(kdb.DPolygon(list(starmap(kdb.DPoint, ring))))
        written += 1
    for ring in planning.partitions:
        top.shapes(layer("partitions")).insert(kdb.DPolygon(list(starmap(kdb.DPoint, ring))))
        written += 1
    for border in planning.borders:
        if len(border) >= 2:
            top.shapes(layer("borders")).insert(path(border))
            written += 1
    for first, second, _ in planning.bottlenecks:
        top.shapes(layer("bottlenecks")).insert(path([first, second]))
        written += 1
    for x, y in planning.launchers:
        top.shapes(layer("launchers")).insert(
            kdb.DPolygon(
                kdb.DBox(
                    x - PLANNING_MARKER_RADIUS,
                    y - PLANNING_MARKER_RADIUS,
                    x + PLANNING_MARKER_RADIUS,
                    y + PLANNING_MARKER_RADIUS,
                )
            )
        )
        written += 1
    for name, segments in (
        ("chains", planning.chains),
        ("lattice", planning.lattice),
        ("inner", planning.inner),
        ("assignments", planning.assignments),
    ):
        target = layer(name)
        for first, second in segments:
            if first != second:
                top.shapes(target).insert(path([first, second]))
                written += 1
    if len(planning.ring) >= 2:
        top.shapes(layer("ring")).insert(path([*planning.ring, planning.ring[0]]))
        written += 1
    for route in planning.corridors:
        if len(route) >= 2:
            top.shapes(layer("corridors")).insert(path(route))
            written += 1
    for x, y in planning.slots:
        top.shapes(layer("slots")).insert(
            kdb.DPolygon(
                kdb.DBox(
                    x - PLANNING_MARKER_RADIUS / 3,
                    y - PLANNING_MARKER_RADIUS / 3,
                    x + PLANNING_MARKER_RADIUS / 3,
                    y + PLANNING_MARKER_RADIUS / 3,
                )
            )
        )
        written += 1
    return written
