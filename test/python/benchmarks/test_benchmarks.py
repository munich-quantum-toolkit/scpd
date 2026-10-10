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
import tomllib
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


#: The configurations this repository plans the chips with. They are the configurations of the data
#: repository with the keys of the planning stages added; the chip input stays in the data repository
#: and is given with ``--chip``.
REPOSITORY = Path(__file__).resolve().parents[3] / "benchmarks"

#: The keys in which a configuration of this repository may differ from the one of the data
#: repository: the port grouping the planning stages need, the launcher border of the capacity grid
#: and the launcher count of the assignment.
PLANNING_KEYS = {
    "ports.patterns.conventional",
    "ports.patterns.bridge_pair",
    "ports.patterns.component",
    "ports.bridge_pairs",
    "grid.launcher_offset_x",
    "grid.launcher_offset_y",
    "stages.assignment.launcher_target",
}


def _config(chip: str) -> Path:
    assert DATA is not None
    return DATA / "inputs" / chip / "config.toml"


def _chip_input(chip: str) -> Path:
    assert DATA is not None
    return DATA / "inputs" / chip / "routing_config.json"


def _flat(table: dict[str, object], prefix: str = "") -> dict[str, object]:
    flat: dict[str, object] = {}
    for key, value in table.items():
        if isinstance(value, dict):
            flat.update(_flat(value, f"{prefix}{key}."))
        else:
            flat[f"{prefix}{key}"] = value
    return flat


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


def test_the_repository_plans_every_chip() -> None:
    """This repository carries one planning configuration per chip of the data repository."""
    assert sorted(path.parent.name for path in REPOSITORY.glob("*/config.toml")) == CHIPS


@pytest.mark.parametrize("chip", CHIPS)
def test_a_planning_configuration_adds_only_the_planning_keys(chip: str) -> None:
    """A configuration of this repository is the one of the data repository plus the planning keys."""
    with _config(chip).open("rb") as file:
        published = _flat(tomllib.load(file))
    with (REPOSITORY / chip / "config.toml").open("rb") as file:
        planning = _flat(tomllib.load(file))

    differing = {key for key in published.keys() | planning.keys() if published.get(key) != planning.get(key)}
    assert differing <= PLANNING_KEYS


@pytest.mark.parametrize("chip", CHIPS)
def test_a_planning_configuration_follows_table_one(chip: str) -> None:
    """A configuration of this repository obeys the rules for a shipped file and carries Table I."""
    assert DATA is not None
    config = load_config(REPOSITORY / chip / "config.toml", strict=True)
    with (DATA / "qor" / "qor.csv").open(encoding="utf-8") as file:
        row = next(row for row in csv.DictReader(file) if row["benchmark"] == chip.upper())

    assert config.rules is not None
    assert config.rules.targetResonatorLength == pytest.approx(float(row["resonator_length_d_fix_um"]))
    assert config.rules.maxFeedlineUtilization == int(row["feedline_max_capacity_r_util"])
    assert config.stages is not None
    assert config.stages.assignment is not None
    assert config.stages.assignment.launcherTarget > 0


@pytest.mark.parametrize("chip", CHIPS)
def test_the_doctor_passes_on_a_planning_configuration(chip: str) -> None:
    """The planning configuration fits the chip of the data repository, given with --chip."""
    report = run_doctor(REPOSITORY / chip / "config.toml", chip_path=_chip_input(chip))
    assert report.ok, report.text()


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
