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
from itertools import starmap
from typing import TYPE_CHECKING

import klayout.db as kdb

from ..chip import obstacles_of, ports_of, role_name, vertices_of

if TYPE_CHECKING:
    from pathlib import Path

    from ..flatbuffers.design.Chip import ChipT
    from ..planning import PlanningGeometry

#: The layer that carries the obstacle polygons, as in the published layouts of planar-superconducting-pd.
OBSTACLE_LAYER = (1, 0)

#: The layer of the port labels; the datatype is the port's role value. The published layouts use layers 1 to 5
#: for the obstacles and the routing result, so the labels stay clear of them.
PORT_LAYER = 10

#: The layer of each kind of planning shape, with the name KLayout shows it under. The layers start
#: at 20, so the artwork and the port labels keep theirs and the overlay can be switched off as a
#: block.
#:
#: The planning shapes are not geometry to manufacture. They are in the file because KLayout is
#: where a partition border is measured against the artwork it has to respect.
PLANNING_LAYERS: dict[str, tuple[int, int, str]] = {
    "keepout": (20, 0, "plan.port-keepout"),
    "partitions": (21, 0, "plan.partition"),
    "borders": (22, 0, "plan.border"),
    "bottlenecks": (23, 0, "plan.bottleneck"),
    "launchers": (24, 0, "plan.launcher"),
    "chains": (25, 0, "plan.chain"),
    "lattice": (26, 0, "plan.lattice"),
    "inner": (27, 0, "plan.inner-circuit"),
    "assignments": (28, 0, "plan.assignment"),
    "ring": (29, 0, "plan.ring"),
    "corridors": (30, 0, "plan.corridor"),
    "slots": (31, 0, "plan.slot"),
    "feedlines": (32, 0, "plan.feedline"),
    "blocked": (33, 0, "plan.blocked"),
    "budgets": (34, 0, "plan.budget"),
    # The routing stages: the copper by kind, the room it keeps, the couplers and the marks of the
    # wires the Final stage left failing. One layer per phase of the Final stage as well, so that
    # one GDS shows all five and each can be switched off on its own.
    "wires": (40, 0, "route.wire"),
    "resonators": (41, 0, "route.resonator"),
    "inner_wires": (42, 0, "route.inner-wire"),
    "feedline_wires": (43, 0, "route.feedline"),
    "clearance": (44, 0, "route.clearance"),
    "couplers": (45, 0, "route.coupler"),
    "failing": (46, 0, "route.failing"),
    "offlength": (47, 0, "route.off-length"),
    "squeezed": (48, 0, "route.squeezed"),
    "phase.pre-feedline-routing": (50, 0, "final.pre-feedline-routing"),
    "phase.pre-feedline-refinement": (51, 0, "final.pre-feedline-refinement"),
    "phase.feedline-insertion": (52, 0, "final.feedline-insertion"),
    "phase.feedline-constrained-routing": (53, 0, "final.feedline-constrained-routing"),
    "phase.feedline-constrained-refinement": (54, 0, "final.feedline-constrained-refinement"),
}

#: The width of a planning line, in layout units. A path needs a width to be a shape at all; this
#: one is visible beside the artwork and carries no design meaning.
PLANNING_LINE_WIDTH = 20.0

#: The half width of a launcher marker, in layout units; a slot marker is a third of it.
PLANNING_MARKER_RADIUS = 60.0

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
    #: The planning shapes written beside the artwork, zero when there were none.
    planning: int = 0


def write_layout(
    chip: ChipT,
    path: Path,
    *,
    cell: str = "chip",
    planning: PlanningGeometry | None = None,
) -> ExportSummary:
    """Write the chip as GDSII or OASIS, chosen by the file suffix.

    The obstacles go to layer 1 as polygons. Every port becomes a text with its label on layer 10,
    with the role's value as the datatype, so that the roles can be shown and hidden separately.
    What a planning stage produced goes on the layers of ``PLANNING_LAYERS``, one per kind of
    shape.

    Args:
        chip: The classified chip.
        path: The file to write; ``.gds``, ``.gds2`` or ``.oas``.
        cell: The name of the top cell.
        planning: What a planning stage produced, written beside the artwork.

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
    shapes = _write_planning(layout, top, planning) if planning is not None else 0
    layout.write(str(path))
    return ExportSummary(path, FORMATS[suffix], polygons, ports, shapes)


def _write_planning(layout: kdb.Layout, top: kdb.Cell, planning: PlanningGeometry) -> int:
    """Write the shapes of one planning stage onto their layers.

    A partition, a keepout and a blocked cell are polygons, because they are areas of the grid. The
    wire budget of a bottleneck is a text at the middle of its line, on a layer of its own. A
    corridor and the approach to its target port are paths on the same layer. Everything else is a
    path or a small square, because it is a line across or along the free space, or a point on it.

    Returns:
        How many shapes were written.
    """

    def layer(name: str) -> int:
        number, datatype, label = PLANNING_LAYERS[name]
        return layout.layer(number, datatype, label)

    def path(points: list[tuple[float, float]]) -> kdb.DPath:
        return kdb.DPath(list(starmap(kdb.DPoint, points)), PLANNING_LINE_WIDTH)

    def square(x: float, y: float, radius: float) -> kdb.DPolygon:
        return kdb.DPolygon(kdb.DBox(x - radius, y - radius, x + radius, y + radius))

    written = 0
    keepout = kdb.Region()
    for ring in planning.keepout:
        keepout ^= kdb.Region(kdb.DPolygon(list(starmap(kdb.DPoint, ring))).to_itype(layout.dbu))
    top.shapes(layer("keepout")).insert(keepout)
    written += keepout.count()
    for ring in planning.blocked:
        top.shapes(layer("blocked")).insert(kdb.DPolygon(list(starmap(kdb.DPoint, ring))))
        written += 1
    for partition in planning.partitions:
        # The rings of a partition are combined even-odd, so a ring inside another one is a hole.
        region = kdb.Region()
        for ring in partition:
            region ^= kdb.Region(kdb.DPolygon(list(starmap(kdb.DPoint, ring))).to_itype(layout.dbu))
        top.shapes(layer("partitions")).insert(region)
        written += 1
    for border in [*planning.borders, *planning.enclosed_borders]:
        if len(border) >= 2:
            top.shapes(layer("borders")).insert(path(border))
            written += 1
    for first, second, budget in planning.bottlenecks:
        top.shapes(layer("bottlenecks")).insert(path([first, second]))
        middle = kdb.DVector((first[0] + second[0]) / 2, (first[1] + second[1]) / 2)
        top.shapes(layer("budgets")).insert(kdb.DText(str(budget), kdb.DTrans(middle)))
        written += 2
    for x, y in planning.launchers:
        top.shapes(layer("launchers")).insert(square(x, y, PLANNING_MARKER_RADIUS))
        written += 1
    for name, segments in (
        ("chains", planning.chains),
        ("lattice", planning.lattice),
        ("inner", planning.inner),
        ("assignments", planning.assignments),
    ):
        for first, second in segments:
            if first != second:
                top.shapes(layer(name)).insert(path([first, second]))
                written += 1
    if len(planning.ring) >= 2:
        top.shapes(layer("ring")).insert(path([*planning.ring, planning.ring[0]]))
        written += 1
    for route in [*planning.corridors, *planning.stubs]:
        if len(route) >= 2:
            top.shapes(layer("corridors")).insert(path(route))
            written += 1
    for name, routes in (
        ("wires", planning.wires),
        ("resonators", planning.resonators),
        ("inner_wires", planning.inner_wires),
        ("feedline_wires", planning.feedline_wires),
    ):
        for route in routes:
            if len(route) >= 2:
                top.shapes(layer(name)).insert(path(route))
                written += 1
    for name, drawn in planning.phases.items():
        key = f"phase.{name}"
        if key not in PLANNING_LAYERS:
            continue
        for route in drawn:
            if len(route) >= 2:
                top.shapes(layer(key)).insert(path(route))
                written += 1
    for ring in planning.couplers:
        top.shapes(layer("couplers")).insert(kdb.DPolygon(list(starmap(kdb.DPoint, ring))))
        written += 1
    for name, marked in (("failing", planning.failing), ("offlength", planning.off_length)):
        for _, _, route in marked:
            if len(route) >= 2:
                top.shapes(layer(name)).insert(path(route))
                written += 1
    for _, _, route, line in planning.squeezed:
        for shape in (route, line):
            if len(shape) >= 2:
                top.shapes(layer("squeezed")).insert(path(shape))
                written += 1
    # The room every wire keeps, as a path one clearance wide: the design rule as the stage
    # converted it onto its grid. Two wires closer than that are two of these that overlap, which
    # a boolean on this layer finds.
    if planning.clearance > 0:
        for route in (*planning.wires, *planning.resonators, *planning.inner_wires, *planning.feedline_wires):
            if len(route) >= 2:
                top.shapes(layer("clearance")).insert(kdb.DPath(list(starmap(kdb.DPoint, route)), planning.clearance))
                written += 1
    for x, y in planning.slots:
        top.shapes(layer("slots")).insert(square(x, y, PLANNING_MARKER_RADIUS / 3))
        written += 1
    for feedline in planning.feedlines:
        if len(feedline.points) >= 2:
            top.shapes(layer("feedlines")).insert(path(feedline.points))
            written += 1
    return written
