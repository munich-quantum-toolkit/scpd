# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Checks of the benchmark chips of planar-superconducting-pd.

The chips live in https://github.com/cda-tum/planar-superconducting-pd. These tests run when
``MQT_SCPD_BENCHMARKS`` names a clone of it, which the ``benchmarks`` nox session and the benchmark
job of CI set, and skip otherwise.
"""

from __future__ import annotations

import csv
import os
from pathlib import Path
from xml.etree import ElementTree as ET  # ruff: ignore[suspicious-xml-etree-import]

import pytest

from mqt.scpd.chip import decode_chip, load_chip, ports_of
from mqt.scpd.config import load_config
from mqt.scpd.doctor import run_doctor
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.plot import layout_svg

DATA = Path(os.environ["MQT_SCPD_BENCHMARKS"]) if "MQT_SCPD_BENCHMARKS" in os.environ else None
CHIPS = sorted(path.parent.name for path in DATA.glob("inputs/*/config.toml")) if DATA else []

pytestmark = pytest.mark.skipif(DATA is None, reason="MQT_SCPD_BENCHMARKS names no clone of the benchmark data")


def _config(chip: str) -> Path:
    assert DATA is not None
    return DATA / "inputs" / chip / "config.toml"


def test_every_benchmark_is_present() -> None:
    """The clone carries the eight chips of the paper."""
    assert CHIPS == ["17q", "21q", "33q", "45q", "4q", "57q", "69q", "9q"]


@pytest.mark.parametrize("chip", CHIPS)
def test_the_configuration_follows_table_one(chip: str) -> None:
    """The configuration obeys the rules for a shipped file and carries the values of Table I."""
    assert DATA is not None
    config = load_config(_config(chip), strict=True)
    with (DATA / "qor" / "qor.csv").open(encoding="utf-8") as file:
        row = next(row for row in csv.DictReader(file) if row["benchmark"] == chip.upper())

    assert config.rules is not None
    assert config.rules.targetResonatorLength == pytest.approx(float(row["resonator_length_d_fix_um"]))
    assert config.rules.maxFeedlineUtilization == int(row["feedline_max_capacity_r_util"])


@pytest.mark.parametrize("chip", CHIPS)
def test_the_doctor_passes(chip: str) -> None:
    """The chip and its configuration fit together."""
    report = run_doctor(_config(chip))
    assert report.ok, report.text()


@pytest.mark.parametrize("chip", CHIPS)
def test_the_layout_view_draws_every_port(chip: str) -> None:
    """The picture parses and carries one marker per port."""
    config_path = _config(chip)
    model = decode_chip(load_chip(load_config(config_path), config_path))

    root = ET.fromstring(layout_svg(model))  # ruff: ignore[suspicious-xml-element-tree-usage]

    markers = [circle for circle in root.iter("{http://www.w3.org/2000/svg}circle") if len(circle)]
    assert len(markers) == len(ports_of(model))


@pytest.mark.skipif(not HAS_KLAYOUT, reason="the export needs KLayout")
@pytest.mark.parametrize("chip", CHIPS)
def test_the_rendered_obstacles_are_the_published_unrouted_layout(chip: str, tmp_path: Path) -> None:
    """The obstacles that render writes cover exactly the polygons of the published unrouted GDS file."""
    import klayout.db as kdb  # ruff: ignore[import-outside-top-level]  # needs the optional dependency

    from mqt.scpd.export import OBSTACLE_LAYER, write_layout  # ruff: ignore[import-outside-top-level]

    assert DATA is not None
    config_path = _config(chip)
    written = write_layout(decode_chip(load_chip(load_config(config_path), config_path)), tmp_path / f"{chip}.gds")

    # A region built from a shape iterator reads its layout lazily, so both layouts stay alive here.
    published, rendered = kdb.Layout(), kdb.Layout()
    published.read(str(DATA / "layouts" / "unrouted" / "gds" / f"{chip}_unrouted.gds"))
    rendered.read(str(written.path))
    # find_layer, not layer: the published file names the layer, and layer() would add a new one.
    regions = [
        kdb.Region(layout.top_cell().begin_shapes_rec(layout.find_layer(*OBSTACLE_LAYER)))
        for layout in (published, rendered)
    ]

    assert not regions[0].is_empty()
    assert (regions[0] ^ regions[1]).is_empty()
