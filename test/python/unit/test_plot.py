# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the SVG rendering."""

from __future__ import annotations

import json
import re
from itertools import pairwise, starmap
from pathlib import Path

# The parsed SVG is the output of the code under test, not foreign input.
from xml.etree import ElementTree as ET  # ruff: ignore[suspicious-xml-etree-import]

import pytest

from mqt.scpd.chip import decode_chip, load_chip, obstacles_of, ports_of, vertices_of
from mqt.scpd.config import load_config
from mqt.scpd.flatbuffers.design.Chip import ChipT
from mqt.scpd.flatbuffers.design.Port import PortT
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.flatbuffers.geometry.Polygon import PolygonT
from mqt.scpd.planning import Feedline, PlanningGeometry
from mqt.scpd.plot import FEEDLINE_COLOR, METAL_FILL, PORT_COLOR, PlotError, layout_svg, simplify

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"
SVG = "{http://www.w3.org/2000/svg}"


def test_simplification_keeps_corners_and_drops_the_rest() -> None:
    """Points within the tolerance of the line between their neighbors go; corners stay."""
    straight = [(0.0, 0.0), (1.0, 0.1), (2.0, -0.1), (3.0, 0.0)]
    assert simplify(straight, 0.5) == [(0.0, 0.0), (3.0, 0.0)]

    square = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 5.0)]
    assert simplify(square, 0.5) == [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 5.0)]

    assert simplify([(0.0, 0.0), (1.0, 1.0)], 0.5) == [(0.0, 0.0), (1.0, 1.0)]


def _distinct_vertices(model: ChipT) -> int:
    """The vertices the layout view draws: every input vertex except a repeat of its predecessor.

    Returns:
        The count.
    """
    count = 0
    for polygon in obstacles_of(model):
        points = [(v.x, v.y) for v in vertices_of(polygon)]
        kept = [p for i, p in enumerate(points) if i == 0 or p != points[i - 1]]
        if len(kept) > 1 and kept[-1] == kept[0]:
            kept.pop()
        count += len(kept) if len(kept) >= 3 else 0
    return count


def test_the_layout_view_keeps_every_vertex() -> None:
    """The picture parses as XML, carries one marker per port, and draws every vertex of the input."""
    config_path = FIXTURE / "config.toml"
    model = decode_chip(load_chip(load_config(config_path), config_path))

    svg = layout_svg(model, title="mini")

    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    assert sum(len(subpath) for subpath in _subpaths(svg)) == _distinct_vertices(model)
    style = root.find(f"{SVG}style")
    assert style is not None
    assert style.text is not None
    assert METAL_FILL in style.text
    circles = list(root.iter(f"{SVG}circle"))
    assert len([c for c in circles if c.find(f"{SVG}title") is not None]) == len(ports_of(model))
    classes = {c.get("class") for c in circles}
    assert {"launcher", "resonator", "conventional"} <= classes
    assert root.get("width") == "2000"
    assert any(text.text == "mini" for text in root.iter(f"{SVG}text"))


def test_a_tolerance_trades_vertices_for_size() -> None:
    """With a tolerance the round pad loses most of its vertices; without one it keeps them all."""
    config_path = FIXTURE / "config.toml"
    model = decode_chip(load_chip(load_config(config_path), config_path))

    exact = sum(len(subpath) for subpath in _subpaths(layout_svg(model)))
    coarse = sum(len(subpath) for subpath in _subpaths(layout_svg(model, tolerance=20.0)))

    assert exact == _distinct_vertices(model)
    assert coarse < exact / 2


def test_an_empty_chip_cannot_be_drawn() -> None:
    """A chip without geometry is a PlotError, not a division by zero."""
    with pytest.raises(PlotError, match="no obstacle vertex and no port"):
        layout_svg(ChipT())


def _chip(*polygons: list[tuple[float, float]]) -> ChipT:
    """Build a chip that carries only obstacles.

    Returns:
        The chip.
    """
    return ChipT(obstacles=[PolygonT(vertices=list(starmap(PointT, polygon))) for polygon in polygons])


def _subpaths(svg: str) -> list[list[tuple[float, float]]]:
    """The absolute vertices of every obstacle and outline subpath of a picture.

    Returns:
        One vertex list per subpath, in path order.
    """
    subpaths = []
    for data in re.findall(r'<path (?:class="f"|id="metal") d="([^"]*)"', svg):
        for subpath in filter(None, data.split("Z")):
            x = y = 0.0
            vertices = []
            for command, dx, dy in re.findall(r"([Ml])(-?[\d.]+) (-?[\d.]+)", subpath):
                x, y = (float(dx), float(dy)) if command == "M" else (x + float(dx), y + float(dy))
                vertices.append((x, y))
            subpaths.append(vertices)
    return subpaths


def _doubled_area(vertices: list[tuple[float, float]]) -> float:
    return sum(ax * by - bx * ay for (ax, ay), (bx, by) in zip(vertices, vertices[1:] + vertices[:1], strict=True))


def test_overlapping_obstacles_of_opposite_winding_leave_no_hole() -> None:
    """Every subpath winds the same way, so the nonzero rule fills an overlap instead of cancelling it."""
    clockwise = [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
    counterclockwise = [(5.0, 5.0), (15.0, 5.0), (15.0, 15.0), (5.0, 15.0)]
    assert _doubled_area(clockwise) < 0 < _doubled_area(counterclockwise)

    first, second = _subpaths(layout_svg(_chip(clockwise, counterclockwise)))
    assert (_doubled_area(first) > 0) == (_doubled_area(second) > 0)


def test_relative_steps_do_not_accumulate_rounding() -> None:
    """Steps run between rounded points, so the last vertex lands where the input has it."""
    points = [(i * 1.0004, 0.0) for i in range(1000)] + [(0.0, 1.0)]
    (rebuilt,) = _subpaths(layout_svg(_chip(points)))
    # The first and the last input vertex share their x, so they must share it in the picture.
    assert rebuilt[-1][0] == pytest.approx(rebuilt[0][0], abs=1e-6)
    assert max(x for x, _ in rebuilt) - rebuilt[0][0] == pytest.approx(round(999 * 1.0004, 3), abs=1e-6)


def _mini() -> ChipT:
    config_path = FIXTURE / "config.toml"
    return decode_chip(load_chip(load_config(config_path), config_path))


def test_a_planning_stage_is_drawn_over_the_chip_on_layers_of_its_own() -> None:
    """Every kind of planning shape is one group with its own stroke, and a gate shows its budget."""
    geometry = PlanningGeometry(
        partitions=[[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]],
        bottlenecks=[((0.0, 0.0), (10.0, 0.0), 3)],
        corridors=[[(0.0, 0.0), (50.0, 50.0), (100.0, 0.0)]],
        slots=[(50.0, 50.0)],
    )

    svg = layout_svg(_mini(), planning=geometry)

    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    groups = {group.get("class") for group in root.iter(f"{SVG}g")}
    assert {"l-partition", "l-bottleneck", "l-corridor", "l-slot"} <= groups
    assert any(text.text == "3" for text in root.iter(f"{SVG}text"))
    style = root.find(f"{SVG}style")
    assert style is not None
    assert style.text is not None
    assert "g.l-corridor>path" in style.text


def test_every_feedline_chain_runs_in_steps_in_one_color() -> None:
    """A chain shows its terminals and its termination, names its ports, and runs horizontally and vertically.

    Every chain takes the one feedline color, and a step between two points that differ in both
    coordinates turns at the corner farther from the center of the chip.
    """
    feedlines = [
        Feedline(
            points=[(0.0, 10.0 * index), (50.0, 40.0 + 10.0 * index), (100.0, 10.0 * index)],
            terminals=[(50.0, 40.0 + 10.0 * index)],
            label=f"feedline {index + 1}",
        )
        for index in range(5)
    ]
    feedlines[4].terminations = [(50.0, 80.0)]

    svg = layout_svg(_mini(), planning=PlanningGeometry(feedlines=feedlines))

    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    (layer,) = (group for group in root.iter(f"{SVG}g") if group.get("class") == "l-feedline")
    chains = layer.findall(f"{SVG}g")
    assert [chain.findtext(f"{SVG}title") for chain in chains] == [f"feedline {index}" for index in range(1, 6)]
    assert all(chain.get("stroke") is None for chain in chains)
    style = root.find(f"{SVG}style")
    assert style is not None
    assert style.text is not None
    assert f"g.l-feedline path{{fill:none;stroke:{FEEDLINE_COLOR}" in style.text
    for chain in chains:
        for path in chain.findall(f"{SVG}path"):
            points = [(float(x), float(y)) for x, y in re.findall(r"[ML](-?[\d.]+) (-?[\d.]+)", path.get("d") or "")]
            assert len(points) == 5
            assert all(a[0] == b[0] or a[1] == b[1] for a, b in pairwise(points))
    assert len(layer.findall(f".//{SVG}circle")) == 5
    assert len(layer.findall(f".//{SVG}rect")) == 1


def test_a_planning_stage_without_shapes_changes_nothing() -> None:
    """An empty overlay draws the same picture as none."""
    model = _mini()
    assert layout_svg(model, planning=PlanningGeometry()) == layout_svg(model)


def test_every_port_is_one_marker_of_one_color() -> None:
    """A port keeps its role as the class of its marker, and the legend counts every port once."""
    chip = ChipT(
        ports=[PortT(label="Coupler1_2.port1", center=PointT(0.0, 0.0), role=UnassignedRole.BridgePair)],
        obstacles=[PolygonT(vertices=[PointT(-5.0, -5.0), PointT(5.0, -5.0), PointT(5.0, 5.0)])],
    )

    root = ET.fromstring(layout_svg(chip))  # ruff: ignore[suspicious-xml-element-tree-usage]

    markers = [circle for circle in root.iter(f"{SVG}circle") if circle.find(f"{SVG}title") is not None]
    assert [circle.get("class") for circle in markers] == ["bridge_pair"]
    assert any(text.text == "port (1)" for text in root.iter(f"{SVG}text"))
    style = root.find(f"{SVG}style")
    assert style is not None
    assert style.text is not None
    assert f".x-ports circle{{fill:{PORT_COLOR}" in style.text


def _groups(svg: str) -> set[str | None]:
    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    return {group.get("class") for group in root.iter(f"{SVG}g")}


def test_a_stage_leaves_out_what_its_picture_does_not_need() -> None:
    """The capacity picture shows borders, not outlines and gates; keep-out and chains never show."""
    geometry = PlanningGeometry(
        partitions=[[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]],
        borders=[[(0.0, 50.0), (100.0, 50.0)]],
        bottlenecks=[((0.0, 0.0), (10.0, 0.0), 3)],
        keepout=[[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]],
        chains=[((0.0, 0.0), (10.0, 10.0))],
    )

    capacity = _groups(layout_svg(_mini(), stage="capacity", planning=geometry))
    assert "l-border" in capacity
    assert capacity.isdisjoint({"l-partition", "l-bottleneck", "l-keepout", "l-chain"})

    other = _groups(layout_svg(_mini(), stage="global", planning=geometry))
    assert {"l-partition", "l-bottleneck"} <= other
    assert other.isdisjoint({"l-keepout", "l-chain"})


def test_the_corridors_are_solid_and_the_ground_under_them_light() -> None:
    """A corridor is a solid band with a casing and a core, and continues along the stub to its port.

    The corridor picture draws the borders light.
    """
    geometry = PlanningGeometry(
        borders=[[(0.0, 50.0), (100.0, 50.0)]],
        corridors=[[(0.0, 0.0), (50.0, 50.0)]],
        stubs=[((50.0, 50.0), (50.0, 80.0))],
    )

    svg = layout_svg(_mini(), stage="corridor", planning=geometry)

    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    assert root.get("class") == "corridor"
    (layer,) = (group for group in root.iter(f"{SVG}g") if group.get("class") == "l-corridor")
    paths = layer.findall(f"{SVG}path")
    assert [path.get("class") for path in paths] == ["casing", None, "core"]
    assert all(path.get("d") == paths[0].get("d") for path in paths)
    assert (paths[0].get("d") or "").count("M") == 2
    style = root.find(f"{SVG}style")
    assert style is not None
    assert style.text is not None
    assert "g.l-corridor>path{stroke-dasharray" not in style.text
    assert "svg.corridor g.l-border>path" in style.text


def test_the_panel_lists_the_layers_the_picture_carries() -> None:
    """The layer panel names every layer that is drawn and none that is not."""
    svg = layout_svg(_mini(), title="mini", stage="layout")

    found = re.search(r"L = (\[.*?\]), f = ", svg)
    assert found is not None
    layers = json.loads(found[1])
    names = [name for name, _, _ in layers]
    assert {"Grid", "Metal", "Ports", "Frame"} <= set(names)
    assert "Corridors" not in names
    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    assert any(text.text == "LAYOUT" for text in root.iter(f"{SVG}text"))


def test_a_feedline_turns_at_the_corner_away_from_the_center() -> None:
    """A step that changes both coordinates turns where the line keeps to the outside of the chip."""
    square = _chip(
        [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0), (0.0, 1000.0)], [(400.0, 400.0), (600.0, 400.0), (500.0, 600.0)]
    )
    geometry = PlanningGeometry(feedlines=[Feedline(points=[(0.0, 0.0), (300.0, 100.0)], label="feedline 1")])

    root = ET.fromstring(layout_svg(square, planning=geometry))  # ruff: ignore[suspicious-xml-element-tree-usage]

    (layer,) = (group for group in root.iter(f"{SVG}g") if group.get("class") == "l-feedline")
    # The picture has a margin of 20 and runs y downwards: the corner at (0, 100) lies farther from
    # the center (500, 500) than the one at (300, 0).
    assert layer.findall(f".//{SVG}path")[1].get("d") == "M20 1020L20 920L320 920"
