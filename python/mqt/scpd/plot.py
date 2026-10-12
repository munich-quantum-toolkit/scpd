# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Per-stage SVG rendering.

``plot`` reads the chip and the artifacts of a run; the core writes no SVG. The picture shows the chip
the way a layout tool shows a mask: the metal as a hatched layer with an inner contour on a light
substrate, a millimetre grid, and a frame with rulers, corner marks and a scale bar. Every port is
one marker. A planning stage is drawn over the chip, one group per kind of shape.

The picture is in layout units, which the chip inputs give in micrometres, so every coordinate of the
input survives and a viewer that zooms in sees the geometry the GDS holds. Lines that only guide the
eye keep their width on screen while zooming. Opened on its own in a browser, the picture has a
panel that shows and hides each layer, and it leaves out what is finer than a few screen pixels, so
that a zoomed-out view stays sharp. A tolerance drops vertices for a smaller file when the detail is
not needed.
"""

from __future__ import annotations

import json
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
    "corridor": "phase 3",
    "detail": "phase 4",
    "final": "phase 4",
    "aligned": "phase 4",
}

#: The substrate, as the two ends of a faint diagonal gradient.
SUBSTRATE: tuple[str, str] = ("#f7f7f6", "#efeeeb")
#: The fill of the metal, its edge, and the hatch and inner contour drawn on it.
METAL_FILL = "#7b7b80"
METAL_EDGE = "#4c4c52"
METAL_DETAIL = "#a8a8ad"
#: The fill of every port marker. One color for every role: the role is in the class of a marker
#: and in its tooltip, and the picture is about where the ports are.
PORT_COLOR = "#e34948"
#: The ink of the frame, its labels and the caption.
INK = "#1b1a18"
MUTED = "#55524b"
FRAME = "#b5b1a8"
#: The color of every feedline chain. The chains tell apart by their tooltips, not by color.
FEEDLINE_COLOR = "#2a78d6"

#: The stroke of each planning layer. Feedlines, inner wires and corridors run over the metal, so
#: they get a white casing under them.
PLANNING_COLORS: dict[str, str] = {
    "partition": "#8b929c",
    "border": "#3f4752",
    "bottleneck": "#eb6834",
    "launcher": "#e34948",
    "lattice": "#b3afa6",
    "inner": "#008300",
    "assignment": "#4a3aa7",
    "ring": "#6b675f",
    "corridor": "#2b2b2e",
    "slot": "#55524b",
}

#: The stroke width of each planning layer, in screen pixels.
_PLANNING_WIDTHS: dict[str, float] = {
    "partition": 0.8,
    "border": 1.8,
    "bottleneck": 1.6,
    "lattice": 0.5,
    "inner": 2.0,
    "assignment": 1.0,
    "ring": 1.0,
    "corridor": 3.4,
}

#: The planning layers each stage leaves out of its picture. The keep-out of the port approaches
#: and the capacity chains are left out everywhere. The capacity and corridor pictures show the
#: partitions by their borders alone: an outline traces the keep-out around every port as well. They
#: leave the gates out too.
_HIDDEN: dict[str, frozenset[str]] = {
    "capacity": frozenset({"bottlenecks", "partitions"}),
    "corridor": frozenset({"bottlenecks", "partitions"}),
}

#: The borders under the corridors are only the ground the wires run on, so the corridor picture
#: draws them light and the dark wires stand out.
_CORRIDOR_GROUND = "svg.corridor g.l-border>path{stroke:#c9ccd1;stroke-width:1}"

#: Lengths of the decoration, in layout units (micrometres).
_HOLE_PITCH = 120.0
_GRID_MINOR = 250
_GRID_MAJOR = 1000
_HATCH_PITCH = 36.0
_INSET = 20.0
_INSET_WIDTH = 6.0


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

    The steps run between the rounded points, so that rounding does not add up along the polygon.
    Every subpath winds the same way: all obstacles share one path under the nonzero fill rule, and
    two overlapping polygons of opposite winding would cancel to a hole.

    Returns:
        The path data.
    """
    rounded = [(round(x, 3), round(y, 3)) for x, y in points]
    doubled_area = sum(
        ax * by - bx * ay for (ax, ay), (bx, by) in zip(rounded, rounded[1:] + rounded[:1], strict=False)
    )
    if doubled_area < 0:
        rounded.reverse()
    (x0, y0), *rest = rounded
    parts = [f"M{_number(x0)} {_number(y0)}"]
    x, y = x0, y0
    for px, py in rest:
        parts.append(f"l{_number(px - x)} {_number(py - y)}")
        x, y = px, py
    return "".join(parts) + "Z"


def _rectilinear(points: Sequence[tuple[float, float]], center: tuple[float, float]) -> list[tuple[float, float]]:
    """A polyline turned into one of horizontal and vertical steps.

    Between two points that differ in both coordinates the line turns once. Of the two corners, it
    takes the one farther from the center of the chip, so a feedline keeps to the outside, where its
    launchers are.

    Returns:
        The points of the stepped line, in order.
    """
    stepped: list[tuple[float, float]] = []
    for point in points:
        if stepped and point[0] != stepped[-1][0] and point[1] != stepped[-1][1]:
            last = stepped[-1]
            corners = [(point[0], last[1]), (last[0], point[1])]
            stepped.append(max(corners, key=lambda c: (c[0] - center[0]) ** 2 + (c[1] - center[1]) ** 2))
        stepped.append(point)
    return stepped


def _planning_layers(
    planning: PlanningGeometry,
    to_view: Callable[[float, float], tuple[float, float]],
    radius: float,
    font: float,
    center: tuple[float, float],
    hidden: frozenset[str],
) -> str:
    """Draw the shapes of one planning stage over the chip.

    Every kind of shape is one group, so a viewer can switch it off, and every shape is drawn in
    layout units like the artwork under it.

    Args:
        planning: What the stage produced.
        to_view: The layout point as a point of the picture.
        radius: The radius of a port marker.
        font: The size of a gate label, in layout units.
        center: The center of the chip in the picture, which the feedlines keep away from.
        hidden: The layers the stage leaves out.

    Returns:
        The SVG elements of the overlay.
    """
    parts: list[str] = []

    def polyline(points: Sequence[tuple[float, float]], *, close: bool = False, moved: bool = False) -> str:
        view = list(points) if moved else list(starmap(to_view, points))
        if close:
            return _path_data(view)
        return "".join(f"{'M' if index == 0 else 'L'}{_number(x)} {_number(y)}" for index, (x, y) in enumerate(view))

    def group(name: str, data: str, *, cased: bool = False) -> None:
        casing = f'<path class="casing" d="{data}"/>' if cased else ""
        parts.append(f'<g class="l-{name}">{casing}<path d="{data}"/></g>')

    if planning.partitions and "partitions" not in hidden:
        group("partition", "".join(polyline(ring, close=True) for ring in planning.partitions))
    if planning.borders:
        group("border", "".join(polyline(border) for border in planning.borders))
    if planning.bottlenecks and "bottlenecks" not in hidden:
        data = "".join(polyline([first, second]) for first, second, _ in planning.bottlenecks)
        # The wire budget sits beside its gate: a gate without its number only says that a
        # corridor narrows there, not how many wires may pass.
        labels = "".join(
            f'<text x="{_number(x)}" y="{_number(y - 0.35 * font)}">{capacity}</text>'
            for first, second, capacity in planning.bottlenecks
            for x, y in [to_view((first[0] + second[0]) / 2, (first[1] + second[1]) / 2)]
        )
        parts.append(f'<g class="l-bottleneck"><path d="{data}"/>{labels}</g>')
    if planning.lattice:
        group("lattice", "".join(polyline([first, second]) for first, second in planning.lattice))
    if planning.inner:
        group("inner", "".join(polyline([first, second]) for first, second in planning.inner), cased=True)
    if planning.ring:
        # The ring is a closed cycle, so it is drawn as one.
        group("ring", polyline(planning.ring, close=True))
    if planning.assignments:
        group("assignment", "".join(polyline([first, second]) for first, second in planning.assignments))
    if planning.slots:
        # Every place a wire may cross a border, so a taken slot can be told from a free one.
        ticks = "".join(
            f'<circle cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius * 0.25)}"/>'
            for px, py in planning.slots
            for x, y in [to_view(px, py)]
        )
        parts.append(f'<g class="l-slot">{ticks}</g>')
    if planning.corridors or planning.stubs:
        # A corridor is drawn like a coplanar waveguide, as the feedlines are, and continues along
        # the approach of its target port.
        data = "".join(polyline(route) for route in planning.corridors)
        data += "".join(polyline([start, port]) for start, port in planning.stubs)
        parts.append(
            f'<g class="l-corridor"><path class="casing" d="{data}"/><path d="{data}"/>'
            f'<path class="core" d="{data}"/></g>'
        )
    if planning.launchers:
        circles = "".join(
            f'<circle cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius * 0.7)}"/>'
            for px, py in planning.launchers
            for x, y in [to_view(px, py)]
        )
        parts.append(f'<g class="l-launcher">{circles}</g>')
    if planning.feedlines:
        # Over everything else. A feedline is drawn like a coplanar waveguide: a white casing, the
        # band, and a light center conductor. It runs in horizontal and vertical steps, and it names
        # its launchers and resonators under the pointer.
        side = radius * 0.45
        chains = []
        for feedline in planning.feedlines:
            data = polyline(_rectilinear(list(starmap(to_view, feedline.points)), center), moved=True)
            terminals = "".join(
                f'<circle cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius * 0.35)}"/>'
                for px, py in feedline.terminals
                for x, y in [to_view(px, py)]
            )
            terminations = "".join(
                f'<rect x="{_number(x - side)}" y="{_number(y - side)}" '
                f'width="{_number(2 * side)}" height="{_number(2 * side)}"/>'
                for px, py in feedline.terminations
                for x, y in [to_view(px, py)]
            )
            chains.append(
                f"<g><title>{escape(feedline.label)}</title>"
                f'<path class="casing" d="{data}"/><path d="{data}"/><path class="core" d="{data}"/>'
                f"{terminals}{terminations}</g>"
            )
        parts.append(f'<g class="l-feedline">{"".join(chains)}</g>')
    return "".join(parts)


def _planning_style(font: float) -> str:
    """The stroke of every planning layer, and the type of the gate labels.

    Returns:
        The CSS rules.
    """
    style = "".join(
        f"g.l-{name}>path{{fill:none;stroke:{PLANNING_COLORS[name]};stroke-width:{width};"
        "stroke-linejoin:round;stroke-linecap:round;vector-effect:non-scaling-stroke}"
        for name, width in _PLANNING_WIDTHS.items()
    )
    style += (
        f"g.l-bottleneck>text{{font:600 {_number(font)}px {_FONT};fill:{PLANNING_COLORS['bottleneck']};"
        "text-anchor:middle;paint-order:stroke;stroke:#ffffff;stroke-width:0.25em;stroke-linejoin:round}"
        "g.l-assignment>path{stroke-dasharray:6 4}"
        "g.l-inner>path.casing{stroke:#ffffff;stroke-width:4.6}"
        "g.l-corridor>path.casing{stroke:#ffffff;stroke-width:6.2;stroke-opacity:.9}"
        "g.l-corridor>path.core{stroke:#ffffff;stroke-width:0.9;stroke-opacity:.85}"
        f"g.l-launcher>circle{{fill:none;stroke:{PLANNING_COLORS['launcher']};stroke-width:1.6;"
        "vector-effect:non-scaling-stroke}"
        f"g.l-slot>circle{{fill:{PLANNING_COLORS['slot']};fill-opacity:0.55;stroke:none}}"
        f"g.l-feedline path{{fill:none;stroke:{FEEDLINE_COLOR};stroke-width:4.2;stroke-linejoin:round;"
        "stroke-linecap:round;vector-effect:non-scaling-stroke}"
        "g.l-feedline path.casing{stroke:#ffffff;stroke-width:7}"
        "g.l-feedline path.core{stroke:#ffffff;stroke-width:1.1;stroke-opacity:.9}"
        f"g.l-feedline circle,g.l-feedline rect{{fill:{FEEDLINE_COLOR};stroke:#ffffff;stroke-width:1.2;"
        "vector-effect:non-scaling-stroke}"
    )
    return style


#: The type of every label.
_FONT = "Inter,'SF Pro Text',system-ui,-apple-system,'Segoe UI',sans-serif"
_MONO = "ui-monospace,'SF Mono',Menlo,Consolas,monospace"


def _frame(box: tuple[float, float, float, float], view_height: float, pad: float, font: float, caption: str) -> str:
    """The marks around the chip: corner marks, rulers in millimetres, a scale bar and a die label.

    Args:
        box: The chip in the picture, as left, top, right and bottom.
        view_height: The height of the picture.
        pad: The margin around the chip.
        font: The size of the legend type.
        caption: The text of the die label.

    Returns:
        The SVG group of the marks.
    """
    left, top, right, bottom = box
    tick = 0.35 * pad
    marks = []
    for y in range(0, int(bottom - top) + 1, _GRID_MINOR):
        length = tick * (1.0 if y % _GRID_MAJOR == 0 else 0.5)
        marks.append(
            f"M{_number(left)} {_number(top + y)}H{_number(left - length)}"
            f"M{_number(right)} {_number(top + y)}H{_number(right + length)}"
        )
    labels = "".join(
        f'<text class="ruler" x="{_number(right + 1.15 * tick)}" y="{_number(top + y + 0.2 * font)}">'
        f"{y // _GRID_MAJOR}</text>"
        for y in range(_GRID_MAJOR, int(bottom - top) + 1, _GRID_MAJOR)
    )
    corners = []
    inset, arm, ring = 260.0, 150.0, 55.0
    for cx, cy in (
        (left + inset, top + inset),
        (right - inset, top + inset),
        (left + inset, bottom - inset),
        (right - inset, bottom - inset),
    ):
        corners.append(
            f"M{_number(cx - arm)} {_number(cy)}H{_number(cx + arm)}M{_number(cx)} {_number(cy - arm)}"
            f"V{_number(cy + arm)}M{_number(cx + ring)} {_number(cy)}A{ring} {ring} 0 1 0 {_number(cx - ring)} "
            f"{_number(cy)}A{ring} {ring} 0 1 0 {_number(cx + ring)} {_number(cy)}"
        )
    # The scale bar takes a round length near a tenth of the die.
    length = next((v for v in (1000, 2000, 5000, 10000) if v >= (right - left) / 10), 10000)
    x1 = right - 0.2 * pad
    y = view_height - 0.55 * font
    bar = (
        f'<path class="scale" d="M{_number(x1 - length)} {_number(y)}H{_number(x1)}'
        f"M{_number(x1 - length)} {_number(y - 0.3 * font)}V{_number(y + 0.3 * font)}"
        f"M{_number(x1)} {_number(y - 0.3 * font)}V{_number(y + 0.3 * font)}"
        f'M{_number(x1 - length / 2)} {_number(y - 0.18 * font)}V{_number(y)}"/>'
        f'<text class="ruler" x="{_number(x1 - length - 0.4 * font)}" y="{_number(y + 0.3 * font)}" '
        f'style="text-anchor:end">{length // 1000} mm</text>'
    )
    die = f'<text class="die" x="{_number(right)}" y="{_number(1.5 * font)}">{escape(caption)}</text>'
    return (
        f'<g class="x-marks"><path class="tick" d="{"".join(marks)}"/>'
        f'<path class="corner" d="{"".join(corners)}"/>{labels}{bar}{die}</g>'
    )


#: The layers the panel can show and hide, by the selector of their elements.
_LAYERS: tuple[tuple[str, str, str], ...] = (
    ("Grid", ".x-grid", MUTED),
    ("Substrate", ".x-holes", MUTED),
    ("Metal", ".o,.x-rim,.x-hatch,.x-inset", METAL_FILL),
    ("Ports", ".x-ports", PORT_COLOR),
    ("Partitions", ".l-partition", PLANNING_COLORS["partition"]),
    ("Borders", ".l-border", PLANNING_COLORS["border"]),
    ("Bottlenecks", ".l-bottleneck", PLANNING_COLORS["bottleneck"]),
    ("Lattice", ".l-lattice", PLANNING_COLORS["lattice"]),
    ("Inner wires", ".l-inner", PLANNING_COLORS["inner"]),
    ("Ring", ".l-ring", PLANNING_COLORS["ring"]),
    ("Assignments", ".l-assignment", PLANNING_COLORS["assignment"]),
    ("Slots", ".l-slot", PLANNING_COLORS["slot"]),
    ("Corridors", ".l-corridor", PLANNING_COLORS["corridor"]),
    ("Launcher slots", ".l-launcher", PLANNING_COLORS["launcher"]),
    ("Feedlines", ".l-feedline", FEEDLINE_COLOR),
    ("Frame", ".x-marks", MUTED),
)

#: The script of the layer panel and of the level of detail. It runs where the picture is opened on
#: its own; embedded as an image, the picture shows every layer and no panel.
_SCRIPT = """(function () {
  var NS = "http://www.w3.org/2000/svg", L = %s, f = %s, x0 = %s, y0 = %s;
  var svg = document.documentElement, rowH = 1.5 * f, w = 9.5 * f, open = false;
  function el(name, attrs, parent) {
    var e = document.createElementNS(NS, name);
    for (var k in attrs) e.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(e);
    return e;
  }
  // What is finer than a few screen pixels is left out, so a zoomed-out view stays sharp.
  function load() {
    var m = svg.getScreenCTM(), k = m ? m.a : 1;
    svg.querySelectorAll(".x-holes").forEach(function (n) { n.classList.toggle("load-off", k * %s < 9); });
    svg.querySelectorAll(".x-minor").forEach(function (n) { n.classList.toggle("load-off", k * %s < 7); });
    svg.querySelectorAll(".x-hatch").forEach(function (n) { n.classList.toggle("load-off", k * %s < 4); });
  }
  window.addEventListener("resize", load);
  window.addEventListener("wheel", function () { requestAnimationFrame(load); }, {passive: true});
  if (window.visualViewport) window.visualViewport.addEventListener("resize", load);
  load();
  var panel = el("g", {"class": "x-panel"}, svg);
  var bg = el("rect", {"class": "x-bg", x: x0, y: y0, width: w, height: rowH, rx: 0.4 * f}, panel);
  var head = el("g", {"class": "x-head"}, panel);
  el("rect", {x: x0, y: y0, width: w, height: rowH, "fill-opacity": 0}, head);
  var title = el("text", {x: x0 + 0.5 * f, y: y0 + rowH / 2, "font-weight": 600}, head);
  title.textContent = "Layers \\u25B8";
  var list = el("g", {style: "display:none"}, panel);
  L.forEach(function (layer, i) {
    var y = y0 + rowH * (i + 1), on = true;
    var row = el("g", {"class": "x-row"}, list);
    el("rect", {"class": "x-hit", x: x0, y: y, width: w, height: rowH, "fill-opacity": 0}, row);
    var dot = el("circle", {cx: x0 + 0.75 * f, cy: y + rowH / 2, r: 0.33 * f, fill: layer[2],
                            stroke: "rgba(0,0,0,.45)", "stroke-width": 0.06 * f}, row);
    el("text", {x: x0 + 1.4 * f, y: y + rowH / 2}, row).textContent = layer[0];
    row.addEventListener("click", function () {
      on = !on;
      dot.setAttribute("fill", on ? layer[2] : "none");
      row.setAttribute("class", on ? "x-row" : "x-row off");
      svg.querySelectorAll(layer[1]).forEach(function (n) { n.style.display = on ? "" : "none"; });
    });
  });
  head.addEventListener("click", function () {
    open = !open;
    list.style.display = open ? "" : "none";
    bg.setAttribute("height", rowH * (open ? L.length + 1 : 1));
    title.textContent = "Layers " + (open ? "\\u25BE" : "\\u25B8");
  });
})();"""


def _panel(body: str, x: float, y: float, font: float) -> str:
    """The layer panel for the layers a picture carries.

    Returns:
        The script element, or nothing when the picture carries no layer.
    """
    present = [
        [name, selector, color]
        for name, selector, color in _LAYERS
        if any(f'class="{part.strip().lstrip(".")}' in body for part in selector.split(","))
    ]
    script = _SCRIPT % (
        json.dumps(present),
        _number(font),
        _number(x),
        _number(y),
        _HOLE_PITCH,
        _GRID_MINOR,
        _HATCH_PITCH,
    )
    return f"<script><![CDATA[\n{script}\n]]></script>"


def layout_svg(
    chip: ChipT,
    *,
    width: int = 2000,
    tolerance: float = 0.0,
    title: str = "",
    stage: str = "",
    planning: PlanningGeometry | None = None,
) -> str:
    """Render the chip, its ports, and what a planning stage produced.

    The picture's user space is layout units, so every coordinate of the input survives with its
    three decimals, and a viewer that zooms in sees the geometry the GDS holds.

    Args:
        chip: The classified chip.
        width: The display width in pixels; the height follows the chip's aspect ratio.
        tolerance: Drop the vertices that stay within this distance, in layout units, of the line
            between their neighbors. Zero keeps every vertex.
        title: The name of the chip, such as the directory of its configuration.
        stage: The stage the picture shows, which the caption names. The capacity picture leaves
            out the gates of its plan.
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
    box = (pad, pad, pad + (max_x - min_x), pad + (max_y - min_y))
    center = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
    font = view_height / 90
    # The metal is drawn through references to one path, and a reference does not pass
    # vector-effect on, so its edge and rim are given in layout units of one screen pixel at the
    # size the picture opens at.
    pixel = view_width / width

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
    metal = "".join(obstacles)

    # Port markers are physical sizes, so they stay in proportion to the artwork when zooming.
    radius = 2.2 * min(max(span / 1200, 10.0), 60.0)
    ports = "".join(
        f'<circle class="{role_name(port.role)}" cx="{_number(x)}" cy="{_number(y)}" r="{_number(radius)}">'
        f"<title>{escape(port.label or '')}</title></circle>"
        for port, center_point in centers
        for x, y in [to_view(center_point.x, center_point.y)]
    )
    legend = (
        f'<g class="legend"><rect x="{_number(pad - 0.8 * font)}" y="{_number(view_height - 1.55 * font)}" '
        f'width="{_number(8 * font)}" height="{_number(1.5 * font)}" rx="{_number(0.75 * font)}"/>'
        f'<circle cx="{_number(pad)}" cy="{_number(view_height - 0.8 * font)}" r="{_number(0.35 * font)}"/>'
        f'<text x="{_number(pad + 0.6 * font)}" y="{_number(view_height - 0.45 * font)}">port ({len(centers)})</text>'
        "</g>"
        if centers
        else ""
    )

    # A millimetre grid under the metal: a fine line every 250 um, a stronger one every 1 mm.
    left, top, right, bottom = box
    minor, major = [], []
    for x in range(_GRID_MINOR, int(right - left), _GRID_MINOR):
        (major if x % _GRID_MAJOR == 0 else minor).append(f"M{_number(left + x)} {_number(top)}V{_number(bottom)}")
    for y in range(_GRID_MINOR, int(bottom - top), _GRID_MINOR):
        (major if y % _GRID_MAJOR == 0 else minor).append(f"M{_number(left)} {_number(top + y)}H{_number(right)}")
    pitch, hole, hatch = _number(_HOLE_PITCH), _number(_HOLE_PITCH / 2 - 11), _HATCH_PITCH
    strokes = (
        f"M0 {_number(hatch)}L{_number(hatch)} 0"
        f"M{_number(-hatch / 4)} {_number(hatch / 4)}L{_number(hatch / 4)} {_number(-hatch / 4)}"
        f"M{_number(hatch * 0.75)} {_number(hatch * 1.25)}L{_number(hatch * 1.25)} {_number(hatch * 0.75)}"
    )
    defs = (
        '<linearGradient id="substrate" x1="0" y1="0" x2="1" y2="1">'
        f'<stop offset="0" stop-color="{SUBSTRATE[0]}"/><stop offset="1" stop-color="{SUBSTRATE[1]}"/>'
        "</linearGradient>"
        f'<pattern id="holes" width="{pitch}" height="{pitch}" patternUnits="userSpaceOnUse">'
        f'<rect x="{hole}" y="{hole}" width="22" height="22" rx="4" fill="#000" fill-opacity=".07"/></pattern>'
        f'<pattern id="hatch" width="{_number(hatch)}" height="{_number(hatch)}" patternUnits="userSpaceOnUse">'
        f'<path d="{strokes}" stroke="{METAL_DETAIL}" stroke-width="5"/></pattern>'
        f'<path id="metal" d="{metal}"/>'
        '<clipPath id="metal-clip"><use href="#metal"/></clipPath>'
    )
    hidden = _HIDDEN.get(stage, frozenset())
    overlay = _planning_layers(planning, to_view, radius, 0.8 * font, center, hidden) if planning is not None else ""
    style = (
        f"text{{font:{_number(font)}px {_FONT};fill:{MUTED}}}"
        f".o{{fill:{METAL_FILL};stroke:{METAL_EDGE};stroke-width:{_number(0.9 * pixel)}}}"
        f".x-rim{{fill:none;stroke:#ffffff;stroke-width:{_number(2.4 * pixel)};stroke-linejoin:round}}"
        ".x-hatch{fill:url(#hatch);fill-opacity:.55}"
        f".x-inset path,.x-inset use{{fill:none}}.x-inset .line{{stroke:{METAL_DETAIL};"
        f"stroke-width:{_number(2 * (_INSET + _INSET_WIDTH / 2))}}}"
        f".x-inset .fill{{stroke:{METAL_FILL};stroke-width:{_number(2 * (_INSET - _INSET_WIDTH / 2))}}}"
        f"path.f{{fill:none;stroke:{FRAME};stroke-width:1.4;stroke-dasharray:2 3;vector-effect:non-scaling-stroke}}"
        f".x-grid{{fill:none;stroke:{INK};stroke-width:1;vector-effect:non-scaling-stroke}}"
        ".x-grid{stroke-opacity:.1}.x-minor{stroke-opacity:.04}"
        f".x-ports circle{{fill:{PORT_COLOR};stroke:#ffffff;stroke-width:1.4;vector-effect:non-scaling-stroke}}"
        f".legend rect{{fill:{SUBSTRATE[0]};fill-opacity:.86;stroke:{FRAME};stroke-opacity:.5}}"
        f".legend circle{{fill:{PORT_COLOR}}}"
        f".x-marks path{{fill:none;stroke:{MUTED};stroke-width:1;vector-effect:non-scaling-stroke}}"
        f".x-marks path.corner{{stroke-opacity:.7}}.x-marks path.scale{{stroke:{INK};stroke-width:1.4}}"
        f"text.ruler{{font:500 {_number(0.62 * font)}px {_MONO}}}"
        f"text.die{{font:500 {_number(0.7 * font)}px {_MONO};text-anchor:end;letter-spacing:.12em}}"
        f"text.title{{font:700 {_number(1.5 * font)}px {_FONT};fill:{INK};letter-spacing:-.01em}}"
        f"text.stage{{font:500 {_number(font)}px {_FONT};letter-spacing:.08em}}"
        f"g.x-panel text{{font:500 {_number(0.85 * font)}px {_FONT};fill:{INK};dominant-baseline:central;"
        "user-select:none}g.x-panel g.x-row,g.x-panel g.x-head{cursor:pointer}"
        f"g.x-panel rect.x-bg{{fill:{SUBSTRATE[0]};fill-opacity:.94;stroke:{FRAME};stroke-opacity:.5}}"
        "g.x-panel g.x-row:hover rect.x-hit{fill-opacity:.07}g.x-panel g.x-row.off text{opacity:.45}"
        ".load-off{display:none}"
    )
    if overlay:
        style += _planning_style(0.8 * font) + _CORRIDOR_GROUND
    name = escape(title)
    heading = ""
    if title:
        heading += f'<text class="title" x="{_number(pad)}" y="{_number(1.5 * font)}">{name}</text>'
    if stage:
        offset = (0.95 * len(title) + 0.6) * font if title else 0.0
        heading += (
            f'<text class="stage" x="{_number(pad + offset)}" y="{_number(1.5 * font)}">{escape(stage.upper())}</text>'
        )
    caption = " · ".join(
        part
        for part in (
            "MQT SCPD",
            title.upper(),
            stage.upper(),
            f"{(right - left) / 1000:.1f} \u00d7 {(bottom - top) / 1000:.1f} mm",
        )
        if part
    )
    body = (
        f'<rect width="{_number(view_width)}" height="{_number(view_height)}" fill="url(#substrate)"/>'
        f'<rect class="x-holes" x="{_number(left)}" y="{_number(top)}" width="{_number(right - left)}" '
        f'height="{_number(bottom - top)}" fill="url(#holes)"/>'
        f'<path class="x-grid x-minor" shape-rendering="crispEdges" d="{"".join(minor)}"/>'
        f'<path class="x-grid" shape-rendering="crispEdges" d="{"".join(major)}"/>'
        f'<path class="f" d="{"".join(outlines)}"/>'
        '<use class="x-rim" href="#metal"/><use class="o" href="#metal"/><use class="x-hatch" href="#metal"/>'
        '<g class="x-inset" clip-path="url(#metal-clip)"><use class="line" href="#metal"/>'
        '<use class="fill" href="#metal"/></g>'
        f'<g class="x-ports">{ports}</g>{overlay}'
        f"{_frame(box, view_height, pad, font, caption)}{legend}{heading}"
    )
    # The stage is the class of the picture, so a rule can draw a layer differently in one stage.
    stage_class = f' class="{escape(stage)}"' if stage else ""
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg"{stage_class} width="{width}" height="{height}" '
        f'viewBox="0 0 {_number(view_width)} {_number(view_height)}">'
        f"<style>{style}</style><defs>{defs}</defs>{body}"
        f"{_panel(body, left + 0.6 * font, top + 0.6 * font, font)}</svg>\n"
    )
