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
        partitions=[[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]],
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
