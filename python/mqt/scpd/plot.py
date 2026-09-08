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
GDS shows. A tolerance drops vertices for a smaller file when the detail is not needed; the 2 MB
budget of a snapshot applies to the raster views of the later stages.
"""

from __future__ import annotations

from typing import TYPE_CHECKING
from xml.sax.saxutils import escape

from .chip import obstacles_of, ports_of, role_name, vertices_of

if TYPE_CHECKING:
    from collections.abc import Sequence

    from .flatbuffers.design.Chip import ChipT

#: The stages the command knows, and the phase in which each arrives.
STAGES: dict[str, str] = {
    "layout": "phase 1",
    "capacity": "phase 3",
    "detail": "phase 4",
    "final": "phase 4",
    "aligned": "phase 4",
}

#: The fill of each role in the port legend. The colors stay apart from the obstacle fill.
ROLE_COLORS: dict[str, str] = {
    "launcher": "#e63946",
    "resonator": "#ffb703",
    "conventional": "#2dc653",
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
    """A coordinate with the three decimals the inputs carry, without trailing zeros."""
    text = f"{value:.3f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def _dedupe(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Drop a vertex that repeats its predecessor, and a closing vertex that repeats the first."""
    kept: list[tuple[float, float]] = []
    for point in points:
        if not kept or point != kept[-1]:
            kept.append(point)
    if len(kept) > 1 and kept[-1] == kept[0]:
        kept.pop()
    return kept


def _path_data(points: Sequence[tuple[float, float]]) -> str:
    """One closed subpath: an absolute move, then relative lines, which keeps the numbers short."""
    (x0, y0), *rest = points
    parts = [f"M{_number(x0)} {_number(y0)}"]
    x, y = x0, y0
    for px, py in rest:
        parts.append(f"l{_number(px - x)} {_number(py - y)}")
        x, y = px, py
    return "".join(parts) + "Z"


def layout_svg(chip: ChipT, *, width: int = 2000, tolerance: float = 0.0, title: str = "") -> str:
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
    caption = f'<text x="{_number(pad)}" y="{_number(1.2 * font)}">{escape(title)}</text>' if title else ""
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {_number(view_width)} {_number(view_height)}">'
        f"<style>{style}</style>"
        f'<rect width="{_number(view_width)}" height="{_number(view_height)}" fill="#ffffff"/>'
        f'<path class="f" d="{"".join(outlines)}"/>'
        f'<path class="o" d="{"".join(obstacles)}"/>'
        f"{''.join(ports)}{legend}{caption}</svg>\n"
    )
