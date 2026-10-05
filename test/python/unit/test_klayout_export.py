# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the KLayout adapter. They skip where KLayout has no wheel."""

from __future__ import annotations

from pathlib import Path

import pytest

from mqt.scpd.chip import decode_chip, load_chip
from mqt.scpd.config import load_config
from mqt.scpd.export.klayout import OBSTACLE_LAYER, PLANNING_LAYERS, PORT_LAYER, ExportError, write_layout
from mqt.scpd.flatbuffers.design.Chip import ChipT
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole
from mqt.scpd.planning import PlanningGeometry

kdb = pytest.importorskip("klayout.db")

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"


@pytest.mark.parametrize("suffix", [".gds", ".oas"])
def test_the_unrouted_chip_is_written_and_reads_back(tmp_path: Path, suffix: str) -> None:
    """Every obstacle becomes a polygon and every port a text on the layer of its role."""
    config_path = BENCHMARKS / "4q" / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))

    summary = write_layout(chip, tmp_path / f"chip{suffix}")

    assert (summary.polygons, summary.ports) == (28, 36)
    layout = kdb.Layout()
    layout.read(str(summary.path))
    top = layout.top_cell()
    assert top.name == "chip"
    obstacles = layout.layer(*OBSTACLE_LAYER)
    assert top.shapes(obstacles).size() == 28
    launchers = layout.layer(PORT_LAYER, UnassignedRole.Launcher)
    labels = {shape.text_string for shape in top.shapes(launchers).each()}
    assert len(labels) == 16
    assert "Chip.port0" in labels
    assert layout.dbu == pytest.approx(0.001)


def test_the_failing_wires_get_a_layer_of_their_own(tmp_path: Path) -> None:
    """A wire the Final stage left failing is written on its own layer beside the wire it is."""
    config_path = BENCHMARKS / "4q" / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))
    route = [(1000.0, 1000.0), (1400.0, 1000.0), (1400.0, 1600.0)]
    planning = PlanningGeometry(
        wires=[route, [(2000.0, 2000.0), (2500.0, 2000.0)]],
        failing=[("1", ["open"], route), ("f0", ["unrouted"], [])],
    )

    summary = write_layout(chip, tmp_path / "chip.gds", planning=planning)

    layout = kdb.Layout()
    layout.read(str(summary.path))
    top = layout.top_cell()
    assert top.shapes(layout.layer(*PLANNING_LAYERS["wires"])).size() == 2
    # The one failing wire with a way; the unrouted one has nothing to write.
    assert top.shapes(layout.layer(*PLANNING_LAYERS["failing"])).size() == 1
    assert summary.planning == 3


def test_the_suffix_selects_the_format(tmp_path: Path) -> None:
    """A suffix KLayout would not write is refused before anything is built."""
    with pytest.raises(ExportError, match="suffix must be one of"):
        write_layout(ChipT(), tmp_path / "chip.svg")
