# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the SVG rendering."""

from __future__ import annotations

import re
from itertools import starmap
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
from mqt.scpd.plot import OBSTACLE_FILL, ROLE_COLORS, PlotError, layout_svg, simplify

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
    paths = {path.get("class"): path.get("d") or "" for path in root.findall(f"{SVG}path")}
    drawn = sum(data.count("M") + data.count("l") for data in paths.values())
    assert drawn == _distinct_vertices(model)
    style = root.find(f"{SVG}style")
    assert style is not None
    assert style.text is not None
    assert OBSTACLE_FILL in style.text
    circles = root.findall(f"{SVG}circle")
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
    for data in re.findall(r'<path class="[of]" d="([^"]*)"', svg):
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


def test_every_feedline_chain_is_drawn_in_a_color_of_its_own() -> None:
    """A chain shows its terminals and its termination and names its ports.

    Its color differs from those of its neighbours along the ring, and the color of the last chain
    differs from that of the first.
    """
    feedlines = [
        Feedline(
            points=[(0.0, 10.0 * index), (50.0, 10.0 * index), (100.0, 10.0 * index)],
            terminals=[(50.0, 10.0 * index)],
            label=f"feedline {index + 1}",
        )
        for index in range(5)
    ]
    feedlines[4].terminations = [(50.0, 40.0)]

    svg = layout_svg(_mini(), planning=PlanningGeometry(feedlines=feedlines))

    root = ET.fromstring(svg)  # ruff: ignore[suspicious-xml-element-tree-usage]
    (layer,) = (group for group in root.iter(f"{SVG}g") if group.get("class") == "l-feedline")
    chains = layer.findall(f"{SVG}g")
    assert [chain.findtext(f"{SVG}title") for chain in chains] == [f"feedline {index}" for index in range(1, 6)]
    colors = [chain.get("stroke") for chain in chains]
    assert all(colors)
    assert all(colors[index] != colors[index + 1] for index in range(4))
    assert colors[4] != colors[0]
    assert len(layer.findall(f".//{SVG}circle")) == 5
    assert len(layer.findall(f".//{SVG}rect")) == 1


def test_a_planning_stage_without_shapes_changes_nothing() -> None:
    """An empty overlay draws the same picture as none."""
    model = _mini()
    assert layout_svg(model, planning=PlanningGeometry()) == layout_svg(model)


def test_a_bridge_port_has_a_colour_of_its_own() -> None:
    """A port that a wire crosses its component through is drawn and counted under its own role."""
    chip = ChipT(
        ports=[PortT(label="Coupler1_2.port1", center=PointT(0.0, 0.0), role=UnassignedRole.BridgePair)],
        obstacles=[PolygonT(vertices=[PointT(-5.0, -5.0), PointT(5.0, -5.0), PointT(5.0, 5.0)])],
    )

    root = ET.fromstring(layout_svg(chip))  # ruff: ignore[suspicious-xml-element-tree-usage]

    assert {circle.get("class") for circle in root.iter(f"{SVG}circle")} == {"bridge_pair"}
    assert any(text.text == "bridge_pair (1)" for text in root.iter(f"{SVG}text"))
    assert "bridge_pair" in ROLE_COLORS
