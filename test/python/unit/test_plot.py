# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the SVG rendering."""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree

import pytest

from mqt.scpd.chip import decode_chip, load_chip, obstacles_of, ports_of, vertices_of
from mqt.scpd.config import load_config
from mqt.scpd.flatbuffers.design.Chip import ChipT
from mqt.scpd.plot import OBSTACLE_FILL, PlotError, layout_svg, simplify

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"
SVG = "{http://www.w3.org/2000/svg}"


def test_simplification_keeps_corners_and_drops_the_rest() -> None:
    """Points within the tolerance of the line between their neighbors go; corners stay."""
    straight = [(0.0, 0.0), (1.0, 0.1), (2.0, -0.1), (3.0, 0.0)]
    assert simplify(straight, 0.5) == [(0.0, 0.0), (3.0, 0.0)]

    square = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 5.0)]
    assert simplify(square, 0.5) == [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 5.0)]

    assert simplify([(0.0, 0.0), (1.0, 1.0)], 0.5) == [(0.0, 0.0), (1.0, 1.0)]


def _distinct_vertices(model: ChipT) -> int:
    """The vertices the layout view draws: every input vertex except a repeat of its predecessor."""
    count = 0
    for polygon in obstacles_of(model):
        points = [(v.x, v.y) for v in vertices_of(polygon)]
        kept = [p for i, p in enumerate(points) if i == 0 or p != points[i - 1]]
        if len(kept) > 1 and kept[-1] == kept[0]:
            kept.pop()
        count += len(kept) if len(kept) >= 3 else 0
    return count


@pytest.mark.parametrize("chip", ["4q", "9q"])
def test_the_layout_view_keeps_every_vertex(chip: str) -> None:
    """The picture parses as XML, carries one marker per port, and draws every vertex of the input."""
    config_path = BENCHMARKS / chip / "config.toml"
    model = decode_chip(load_chip(load_config(config_path), config_path))

    svg = layout_svg(model, title=chip)

    root = ElementTree.fromstring(svg)
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
    assert any(text.text == chip for text in root.iter(f"{SVG}text"))


def test_a_tolerance_trades_vertices_for_size() -> None:
    """With a tolerance the picture shrinks; without one it keeps the input's coordinates."""
    config_path = BENCHMARKS / "9q" / "config.toml"
    model = decode_chip(load_chip(load_config(config_path), config_path))

    exact = layout_svg(model)
    coarse = layout_svg(model, tolerance=20.0)

    assert len(coarse) < len(exact) / 2
    assert len(exact.encode("utf-8")) < 2_000_000


def test_an_empty_chip_cannot_be_drawn() -> None:
    """A chip without geometry is a PlotError, not a division by zero."""
    with pytest.raises(PlotError, match="no obstacle vertex and no port"):
        layout_svg(ChipT())
