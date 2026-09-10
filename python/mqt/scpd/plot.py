# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Per-stage SVG rendering.

``plot`` reads the chip and, from phase 2 on, the artifacts of a run directory, and nothing else;
the core writes no SVG. The layout view draws the obstacles and the ports colored
by role, in layout units and with every vertex of the input, so that the picture shows what the
GDS shows. A tolerance drops vertices for a smaller file when the detail is not needed; the 10 MB
budget of a snapshot applies to the raster views of the later stages.
"""

from __future__ import annotations

from itertools import starmap
from typing import TYPE_CHECKING
from xml.sax.saxutils import escape

from .chip import obstacles_of, ports_of, role_name, vertices_of

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from .flatbuffers.design.Chip import ChipT
    from .planning import PlanningGeometry

#: The stages the command knows, and the phase in which each arrives.
STAGES: dict[str, str] = {
    "layout": "phase 1",
    "capacity": "phase 3",
    "global": "phase 3",
    "assign": "phase 3",
    "corridor": "phase 4",
    "detail": "phase 4",
    "final": "phase 4",
    "aligned": "phase 4",
}

#: The color of each planning layer. They sit above the artwork, so each one is distinct from the
#: obstacle fill and from every role color.
PLANNING_COLORS: dict[str, str] = {
    "partition": "#8ecae6",
    "keepout": "#c77dff",
    "border": "#023047",
    "bottleneck": "#d00000",
    "launcher": "#e63946",
    "chain": "#7b2cbf",
    "lattice": "#adb5bd",
    "inner": "#0077b6",
    "assignment": "#ff7f0e",
    "ring": "#606060",
    "corridor": "#e85d04",
    "slot": "#495057",
    "wire": "#c1121f",
    "innerwire": "#0077b6",
    "clearance": "#c1121f",
}

#: The fill of each role in the port legend. The colors stay apart from the obstacle fill.
ROLE_COLORS: dict[str, str] = {
    "launcher": "#e63946",
    "resonator": "#ffb703",
    "conventional": "#2dc653",
    "bridge_pair": "#9d4edd",
    "coupler": "#ff7f0e",
    "unset": "#9a9a9a",
}

#: The fill and the outline of the obstacle polygons, and the outline of the chip boundary.
OBSTACLE_FILL = "#3d5a80"
OBSTACLE_STROKE = "#1c2b40"


class PlotError(ValueError):
    """A plot that cannot be produced."""


def simplify(points: Sequence[tuple[float, float]], tolerance: float) -> list[tuple[float, float]]:
    """Drop the vertices of a polyline that stay within ``tolerance`` of the line between their neighbors.

    Douglas-Peucker, iterative. The first and last point always survive.

    Returns:
        The kept points, in order.
    """
    if len(points) < 3:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        first, last = stack.pop()
        (ax, ay), (bx, by) = points[first], points[last]
        dx, dy = bx - ax, by - ay
        length = (dx * dx + dy * dy) ** 0.5
        farthest, distance = first, 0.0
        for i in range(first + 1, last):
            px, py = points[i]
            d = (
                abs(dx * (py - ay) - dy * (px - ax)) / length
                if length > 0
                else ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
            )
            if d > distance:
                farthest, distance = i, d
        if distance > tolerance:
            keep[farthest] = True
            stack.extend(((first, farthest), (farthest, last)))
    return [point for point, kept in zip(points, keep, strict=True) if kept]


def _number(value: float) -> str:
    """A coordinate with the three decimals the inputs carry, without trailing zeros.

    Returns:
        The number as text.
    """
    text = f"{value:.3f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def _dedupe(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Drop a vertex that repeats its predecessor, and a closing vertex that repeats the first.

    Returns:
        The remaining vertices, in order.
    """
    kept: list[tuple[float, float]] = []
    for point in points:
        if not kept or point != kept[-1]:
            kept.append(point)
    if len(kept) > 1 and kept[-1] == kept[0]:
        kept.pop()
    return kept


def _path_data(points: Sequence[tuple[float, float]]) -> str:
    """One closed subpath: an absolute move, then relative lines, which keeps the numbers short.

    Returns:
        The path data.
    """
    (x0, y0), *rest = points
    parts = [f"M{_number(x0)} {_number(y0)}"]
    x, y = x0, y0
    for px, py in rest:
        parts.append(f"l{_number(px - x)} {_number(py - y)}")
        x, y = px, py
    return "".join(parts) + "Z"


def _planning_layers(
    planning: PlanningGeometry,
    to_view: Callable[[float, float], tuple[float, float]],
    radius: float,
    font: float,
) -> str:
    """Draw the shapes of one planning stage over the chip.

    Every layer is one group, so a viewer can switch it off, and every shape is drawn in layout
    units like the artwork under it.

    Args:
        planning: What the stage produced.
        to_view: The layout point as the picture's own coordinates.
        radius: The radius a port marker is drawn at.
        font: The size of a gate label, in layout units.

    Returns:
        The SVG elements of the overlay.
    """
    parts: list[str] = []

    def polyline(points: Sequence[tuple[float, float]], *, close: bool = False) -> str:
        moved = list(starmap(to_view, points))
        return (
            _path_data(moved)
            if close
            else "".join(
                (f"M{_number(px)} {_number(py)}" if index == 0 else f"L{_number(px)} {_number(py)}")
                for index, (px, py) in enumerate(moved)
            )
        )

    if planning.keepout:
        # Under everything else: it is what the ports block, not what a stage decided.
        data = "".join(polyline(ring, close=True) for ring in planning.keepout)
        parts.append(f'<g class="l-keepout"><path d="{data}"/></g>')
    if planning.partitions:
        data = "".join(polyline(ring, close=True) for ring in planning.partitions)
        parts.append(f'<g class="l-partition"><path d="{data}"/></g>')
    if planning.borders:
        data = "".join(polyline(border) for border in planning.borders)
        parts.append(f'<g class="l-border"><path d="{data}"/></g>')
    if planning.bottlenecks:
        data = "".join(polyline([first, second]) for first, second, _ in planning.bottlenecks)
        # The wire budget beside the gate it belongs to. A gate whose number is not shown says
        # only that a corridor narrows there, not how much of it the circuit is allowed to use.
        labels = "".join(
            f'<text x="{_number(x)}" y="{_number(y - 0.35 * font)}">{capacity}</text>'
            for first, second, capacity in planning.bottlenecks
            for x, y in [to_view((first[0] + second[0]) / 2, (first[1] + second[1]) / 2)]
        )
        parts.append(f'<g class="l-bottleneck"><path d="{data}"/>{labels}</g>')
    if planning.chains:
        data = "".join(polyline([first, second]) for first, second in planning.chains)
        parts.append(f'<g class="l-chain"><path d="{data}"/></g>')
    if planning.lattice:
        data = "".join(polyline([first, second]) for first, second in planning.lattice)
        parts.append(f'<g class="l-lattice"><path d="{data}"/></g>')
    if planning.inner:
        data = "".join(polyline([first, second]) for first, second in planning.inner)
        parts.append(f'<g class="l-inner"><path d="{data}"/></g>')
    if planning.ring:
        # The ring is a closed cycle, so it is drawn as one.
        parts.append(f'<g class="l-ring"><path d="{polyline(planning.ring, close=True)}"/></g>')
    if planning.assignments:
        data = "".join(polyline([first, second]) for first, second in planning.assignments)
        parts.append(f'<g class="l-assignment"><path d="{data}"/></g>')
    if planning.slots:
        # Every place a wire may cross a border, so a taken slot can be told from a free one.
        ticks = "".join(
            f'<circle cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius * 0.5)}"/>'
            for px, py in planning.slots
            for x, y in [to_view(px, py)]
        )
        parts.append(f'<g class="l-slot">{ticks}</g>')
    if planning.corridors:
        data = "".join(polyline(route) for route in planning.corridors)
        parts.append(f'<g class="l-corridor"><path d="{data}"/></g>')
    if planning.clearance > 0 and (planning.wires or planning.inner_wires):
        # The room every wire is entitled to: a band one clearance wide around it, drawn under
        # the wires themselves. Each wire is its own element and the band is translucent, so two
        # wires closer than the clearance are two bands whose overlap is darker — which is the
        # whole point of drawing it. The width is in layout units, not on screen, because a
        # clearance is a physical distance; it is the design rule as the router converted it to
        # whole cells, not the rule itself, so what is drawn is what is kept.
        bands = "".join(f'<path d="{polyline(wire)}"/>' for wire in [*planning.wires, *planning.inner_wires])
        parts.append(f'<g class="l-clearance">{bands}</g>')
    if planning.wires:
        # The drawn copper, over the plan it followed.
        data = "".join(polyline(wire) for wire in planning.wires)
        parts.append(f'<g class="l-wire"><path d="{data}"/></g>')
    if planning.inner_wires:
        data = "".join(polyline(wire) for wire in planning.inner_wires)
        parts.append(f'<g class="l-innerwire"><path d="{data}"/></g>')
    if planning.launchers:
        circles = "".join(
            f'<circle cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius * 1.4)}"/>'
            for px, py in planning.launchers
            for x, y in [to_view(px, py)]
        )
        parts.append(f'<g class="l-launcher">{circles}</g>')
    return "".join(parts)


def _planning_style(font: float, clearance: float = 0.0) -> str:
    """The stroke of every planning layer, and the type of the capacity labels.

    Args:
        font: The size of a gate label, in layout units.
        clearance: The width of the band drawn around every wire, in layout units. Zero draws none.

    Returns:
        The CSS of the overlay.
    """
    widths = {
        "keepout": 0.6,
        "partition": 0.6,
        "border": 1.4,
        "bottleneck": 1.6,
        "chain": 1.0,
        "lattice": 0.5,
        "inner": 2.0,
        "assignment": 1.2,
        "ring": 1.0,
        "corridor": 1.8,
        "wire": 1.6,
        "innerwire": 1.6,
    }
    style = "".join(
        f"g.l-{name}>path{{fill:none;stroke:{PLANNING_COLORS[name]};stroke-width:{width};"
        f"stroke-linejoin:round;vector-effect:non-scaling-stroke}}"
        for name, width in widths.items()
    )
    style += (
        f"g.l-bottleneck>text{{font:{_number(font)}px sans-serif;fill:{PLANNING_COLORS['bottleneck']};"
        "text-anchor:middle;paint-order:stroke;stroke:#ffffff;stroke-width:0.25em;"
        "stroke-linejoin:round}"
    )
    style += f"g.l-keepout>path{{fill:{PLANNING_COLORS['keepout']};fill-opacity:0.18;fill-rule:evenodd}}"
    style += "g.l-partition>path{fill:none;stroke-dasharray:4 3}"
    style += "g.l-assignment>path{stroke-dasharray:6 4}"
    style += "g.l-corridor>path{stroke-dasharray:5 4}"
    style += (
        f"g.l-launcher>circle{{fill:none;stroke:{PLANNING_COLORS['launcher']};stroke-width:1.6;"
        "vector-effect:non-scaling-stroke}"
    )
    style += f"g.l-slot>circle{{fill:{PLANNING_COLORS['slot']};fill-opacity:0.55;stroke:none}}"
    if clearance > 0:
        style += (
            f"g.l-clearance>path{{fill:none;stroke:{PLANNING_COLORS['clearance']};"
            f"stroke-width:{_number(clearance)};stroke-opacity:0.13;"
            "stroke-linejoin:round;stroke-linecap:round}"
        )
    return style


def layout_svg(
    chip: ChipT,
    *,
    width: int = 2000,
    tolerance: float = 0.0,
    title: str = "",
    planning: PlanningGeometry | None = None,
) -> str:
    """Render the unrouted chip: the obstacles, and the ports colored by role.

    The picture's user space is layout units, so every coordinate of the input survives with its
    three decimals, and a viewer that zooms in sees the geometry the GDS holds. Strokes keep their
    width on screen while zooming.

    Args:
        chip: The classified chip.
        width: The display width in pixels; the height follows the chip's aspect ratio.
        tolerance: Drop the vertices that stay within this distance, in layout units, of the line
            between their neighbors. Zero keeps every vertex.
        title: A caption, such as the configuration's name.
        planning: What a planning stage produced, drawn over the artwork on layers of its own.

    Returns:
        The SVG text.

    Raises:
        PlotError: If the chip has no geometry to draw.
    """
    polygons = [vertices_of(polygon) for polygon in obstacles_of(chip)]
    centers = [(port, port.center) for port in ports_of(chip) if port.center is not None]
    xs = [v.x for vertices in polygons for v in vertices] + [center.x for _, center in centers]
    ys = [v.y for vertices in polygons for v in vertices] + [center.y for _, center in centers]
    if not xs:
        msg = "the chip has no obstacle vertex and no port to draw"
        raise PlotError(msg)
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1e-9)
    pad = 0.02 * span
    view_width = (max_x - min_x) + 2 * pad
    view_height = (max_y - min_y) + 2 * pad
    height = round(width * view_height / view_width)

    def to_view(x: float, y: float) -> tuple[float, float]:
        return x - min_x + pad, max_y - y + pad

    chip_area = (max_x - min_x) * (max_y - min_y)
    obstacles: list[str] = []
    outlines: list[str] = []
    for vertices in polygons:
        points = _dedupe([to_view(v.x, v.y) for v in vertices])
        if tolerance > 0:
            points = simplify(points, tolerance)
        if len(points) < 3:
            continue
        box_area = (max(v.x for v in vertices) - min(v.x for v in vertices)) * (
            max(v.y for v in vertices) - min(v.y for v in vertices)
        )
        # A polygon that spans the chip is its outline, drawn as a frame rather than a fill.
        (outlines if box_area >= 0.9 * chip_area else obstacles).append(_path_data(points))

    # Port markers are physical sizes, so they stay in proportion to the artwork when zooming.
    radius = min(max(span / 1200, 10.0), 60.0)
    ports = [
        f'<circle class="{role_name(port.role)}" cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius)}">'
        f"<title>{escape(port.label or '')}</title></circle>"
        for port, center in centers
        for x, y in [to_view(center.x, center.y)]
    ]
    counts = {name: sum(1 for port, _ in centers if role_name(port.role) == name) for name in ROLE_COLORS}
    font = view_height / 90
    legend = "".join(
        f'<g transform="translate({_number(pad + i * 11 * font)},{_number(view_height - 0.4 * font)})">'
        f'<circle class="{name}" cx="0" cy="{_number(-0.35 * font)}" r="{_number(0.35 * font)}"/>'
        f'<text x="{_number(0.6 * font)}" y="0">{name} ({counts[name]})</text></g>'
        for i, name in enumerate(name for name in ROLE_COLORS if counts[name])
    )
    style = (
        f"path.o{{fill:{OBSTACLE_FILL};stroke:{OBSTACLE_STROKE};stroke-width:0.6;vector-effect:non-scaling-stroke}}"
        f"path.f{{fill:none;stroke:{OBSTACLE_STROKE};stroke-width:1;vector-effect:non-scaling-stroke}}"
        f"text{{font:{_number(font)}px sans-serif;fill:#202020}}"
        + "".join(
            f"circle.{name}{{fill:{color};stroke:#ffffff;stroke-width:0.8;vector-effect:non-scaling-stroke}}"
            for name, color in ROLE_COLORS.items()
        )
    )
    # The gate labels sit inside the picture rather than along its edge, so they are drawn a
    # little smaller than the caption and the legend.
    gate_font = 0.8 * font
    overlay = _planning_layers(planning, to_view, radius, gate_font) if planning is not None else ""
    if overlay:
        style += _planning_style(gate_font, planning.clearance if planning is not None else 0.0)
    caption = f'<text x="{_number(pad)}" y="{_number(1.2 * font)}">{escape(title)}</text>' if title else ""
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {_number(view_width)} {_number(view_height)}">'
        f"<style>{style}</style>"
        f'<rect width="{_number(view_width)}" height="{_number(view_height)}" fill="#ffffff"/>'
        f'<path class="f" d="{"".join(outlines)}"/>'
        f'<path class="o" d="{"".join(obstacles)}"/>'
        f"{''.join(ports)}{overlay}{legend}{caption}</svg>\n"
    )
