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

pytestmark = pytest.mark.skipif(not HAS_KLAYOUT, reason="the export needs KLayout")

if HAS_KLAYOUT:
    import klayout.db as kdb

    from mqt.scpd.export import OBSTACLE_LAYER, PORT_LAYER, ExportError, write_layout

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


def test_the_suffix_selects_the_format(tmp_path: Path) -> None:
    """A suffix KLayout would not write is refused before anything is built."""
    with pytest.raises(ExportError, match="suffix must be one of"):
        write_layout(ChipT(), tmp_path / "chip.svg")
