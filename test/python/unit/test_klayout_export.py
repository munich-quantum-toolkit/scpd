# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the KLayout adapter. They skip where the optional KLayout dependency is absent."""

from __future__ import annotations

from pathlib import Path

import pytest

from mqt.scpd.chip import decode_chip, load_chip
from mqt.scpd.config import load_config
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.design.Chip import ChipT
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole
from mqt.scpd.planning import Feedline, PlanningGeometry

pytestmark = pytest.mark.skipif(not HAS_KLAYOUT, reason="the export needs KLayout")

if HAS_KLAYOUT:
    import klayout.db as kdb

    from mqt.scpd.export import OBSTACLE_LAYER, PLANNING_LAYERS, PORT_LAYER, ExportError, write_layout

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"


@pytest.mark.parametrize("suffix", [".gds", ".oas"])
def test_the_unrouted_chip_is_written_and_reads_back(tmp_path: Path, suffix: str) -> None:
    """Every obstacle becomes a polygon and every port a text on the layer of its role."""
    config_path = FIXTURE / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))

    summary = write_layout(chip, tmp_path / f"chip{suffix}")

    assert (summary.polygons, summary.ports) == (4, 9)
    layout = kdb.Layout()
    layout.read(str(summary.path))
    top = layout.top_cell()
    assert top.name == "chip"
    obstacles = layout.layer(*OBSTACLE_LAYER)
    assert top.shapes(obstacles).size() == 4
    launchers = layout.layer(PORT_LAYER, UnassignedRole.Launcher)
    labels = {shape.text_string for shape in top.shapes(launchers).each()}
    assert len(labels) == 4
    assert "Chip.port0" in labels
    assert layout.dbu == pytest.approx(0.001)


def test_the_suffix_selects_the_format(tmp_path: Path) -> None:
    """A suffix KLayout would not write is refused before anything is built."""
    with pytest.raises(ExportError, match="suffix must be one of"):
        write_layout(ChipT(), tmp_path / "chip.svg")


def test_a_planning_stage_is_written_on_layers_of_its_own(tmp_path: Path) -> None:
    """Every kind of planning shape goes on its own named layer, from layer 20 on, beside the artwork."""
    config_path = FIXTURE / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))
    geometry = PlanningGeometry(
        partitions=[[[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]]],
        corridors=[[(0.0, 0.0), (50.0, 50.0), (100.0, 0.0)]],
        slots=[(50.0, 50.0)],
    )

    summary = write_layout(chip, tmp_path / "plan.gds", planning=geometry)

    assert summary.planning == 3
    assert min(number for number, _, _ in PLANNING_LAYERS.values()) >= 20
    layout = kdb.Layout()
    layout.read(str(summary.path))
    top = layout.top_cell()
    for name in ("partitions", "corridors", "slots"):
        number, datatype, _ = PLANNING_LAYERS[name]
        assert top.shapes(layout.layer(number, datatype)).size() == 1, name
    assert top.shapes(layout.layer(*OBSTACLE_LAYER)).size() == 4


@pytest.mark.parametrize("suffix", [".gds", ".oas"])
@pytest.mark.parametrize(
    ("layer_name", "area", "hole_empty"),
    [("partitions", 80000.0, True), ("keepout", 80000.0, True), ("blocked", 90000.0, False)],
)
def test_nested_planning_rings_preserve_holes_and_filled_regions(
    tmp_path: Path, suffix: str, layer_name: str, area: float, *, hole_empty: bool
) -> None:
    """Partitions and keepout preserve holes; overlapping blocked rectangles stay filled."""
    config_path = FIXTURE / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))
    outer = [(0.0, 0.0), (300.0, 0.0), (300.0, 300.0), (0.0, 300.0)]
    hole = [(100.0, 100.0), (200.0, 100.0), (200.0, 200.0), (100.0, 200.0)]
    geometry = PlanningGeometry()
    if layer_name == "partitions":
        geometry.partitions = [[outer, hole]]
    elif layer_name == "keepout":
        geometry.keepout = [outer, hole]
    else:
        geometry.blocked = [outer, hole]

    summary = write_layout(chip, tmp_path / f"hole{suffix}", planning=geometry)

    layout = kdb.Layout()
    layout.read(str(summary.path))
    number, datatype, _ = PLANNING_LAYERS[layer_name]
    region = kdb.Region(layout.top_cell().begin_shapes_rec(layout.layer(number, datatype))).merged()
    assert region.count() == 1
    assert region.area() * layout.dbu**2 == pytest.approx(area)
    inside_hole = kdb.Region(kdb.DBox(140.0, 140.0, 160.0, 160.0).to_itype(layout.dbu))
    assert (region & inside_hole).is_empty() == hole_empty


def test_every_feedline_chain_is_one_path_on_the_feedline_layer(tmp_path: Path) -> None:
    """A feedline chain is written as the line from its first launcher to its last."""
    config_path = FIXTURE / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))
    geometry = PlanningGeometry(
        feedlines=[
            Feedline(points=[(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)], terminals=[(50.0, 0.0)]),
            Feedline(points=[(0.0, 50.0), (60.0, 50.0)], terminals=[(60.0, 50.0)], terminations=[(60.0, 50.0)]),
        ]
    )

    summary = write_layout(chip, tmp_path / "feedlines.gds", planning=geometry)

    assert summary.planning == 2
    layout = kdb.Layout()
    layout.read(str(summary.path))
    number, datatype, _ = PLANNING_LAYERS["feedlines"]
    shapes = layout.top_cell().shapes(layout.layer(number, datatype))
    assert shapes.size() == 2
    assert all(shape.is_path() for shape in shapes.each())


def test_the_stubs_the_blocked_cells_and_the_budgets_are_written(tmp_path: Path) -> None:
    """A stub goes on the corridor layer, a blocked cell and a wire budget each on a layer of their own."""
    config_path = FIXTURE / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))
    geometry = PlanningGeometry(
        blocked=[[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]],
        bottlenecks=[((0.0, 0.0), (100.0, 0.0), 3)],
        corridors=[[(0.0, 0.0), (50.0, 50.0)]],
        stubs=[[(50.0, 50.0), (50.0, 500.0), (600.0, 500.0)]],
    )

    summary = write_layout(chip, tmp_path / "plan.gds", planning=geometry)

    assert summary.planning == 5
    layout = kdb.Layout()
    layout.read(str(summary.path))
    top = layout.top_cell()

    def shapes(name: str) -> list[kdb.Shape]:
        number, datatype, _ = PLANNING_LAYERS[name]
        return list(top.shapes(layout.layer(number, datatype)).each())

    assert [shape.is_path() for shape in shapes("corridors")] == [True, True]
    assert [shape.is_polygon() or shape.is_box() for shape in shapes("blocked")] == [True]
    (budget,) = shapes("budgets")
    assert budget.text_string == "3"
    assert (budget.text_dpos.x, budget.text_dpos.y) == pytest.approx((50.0, 0.0))
    assert len({number for number, _, _ in PLANNING_LAYERS.values()}) == len(PLANNING_LAYERS)


def test_the_routing_shapes_get_layers_of_their_own(tmp_path: Path) -> None:
    """The copper, the room it keeps, the phases and the failing wires each go on their own layer."""
    config_path = FIXTURE / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))
    route = [(1000.0, 1000.0), (1400.0, 1000.0), (1400.0, 1600.0)]
    planning = PlanningGeometry(
        wires=[route, [(2000.0, 2000.0), (2500.0, 2000.0)]],
        resonators=[[(300.0, 300.0), (600.0, 300.0)]],
        failing=[("1", ["open"], route), ("f0", ["unrouted"], [])],
        phases={"pre-feedline-routing": [route]},
        clearance=190.0,
    )

    summary = write_layout(chip, tmp_path / "chip.gds", planning=planning)

    layout = kdb.Layout()
    layout.read(str(summary.path))
    top = layout.top_cell()
    assert top.shapes(layout.layer(*PLANNING_LAYERS["wires"])).size() == 2
    assert top.shapes(layout.layer(*PLANNING_LAYERS["resonators"])).size() == 1
    # The one failing wire with a way; the unrouted one has nothing to write.
    assert top.shapes(layout.layer(*PLANNING_LAYERS["failing"])).size() == 1
    assert top.shapes(layout.layer(*PLANNING_LAYERS["phase.pre-feedline-routing"])).size() == 1
    assert top.shapes(layout.layer(*PLANNING_LAYERS["clearance"])).size() == 3
    assert summary.planning == 8
