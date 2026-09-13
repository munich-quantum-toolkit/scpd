# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the config.toml loader."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from mqt.scpd.config import ConfigError, load_config, parse_config, read_config, shipped_config_problems, write_config

if TYPE_CHECKING:
    from pathlib import Path

MANUAL = """
[chip]
input = "routing_config.json"

[ports.patterns]
launcher = '^Chip\\.port\\d+$'
resonator = '^Qb\\d+\\.port0$'
conventional = '^(Qb\\d+\\.port1|Coupler\\d+_\\d+\\.port[0-4])$'

[ports.sequences]
all_outer = ["Qb1.port1", "Qb1.port0", "Coupler1_2.port3"]
fixed_outer = ["Qb1.port0"]

[design_rules]
min_wire_spacing = 185.0
min_obstacle_spacing = 25.0
min_bend_radius = 50.0
min_straight_length = 100.0
target_resonator_length = 2500
resonator_length_tolerance = 100.0
max_feedline_utilization = 5
feedline_terminations = 1
"""

WITH_GRID = (
    MANUAL
    + """
[grid]
capacity_cells_x = 25
capacity_cells_y = 25
"""
)


def test_manual_configuration_loads_with_the_schema_defaults() -> None:
    """Every section lands in the schema object; absent keys take the schema's defaults."""
    config = parse_config(MANUAL)

    assert config.chipInput == "routing_config.json"
    assert config.ports is not None
    assert config.ports.patterns is not None
    assert config.ports.patterns.resonator == r"^Qb\d+\.port0$"
    assert config.ports.sequences is not None
    assert config.ports.sequences.allOuter == ["Qb1.port1", "Qb1.port0", "Coupler1_2.port3"]
    assert config.ports.sequences.fixedOuter == ["Qb1.port0"]
    assert config.rules is not None
    assert config.rules.targetResonatorLength == pytest.approx(2500.0)
    assert config.rules.maxFeedlineUtilization == 5
    assert config.grid is None


def test_the_grid_section_is_optional_and_partial() -> None:
    """A partial grid section loads with the defaults for the rest."""
    config = parse_config(WITH_GRID)

    assert config.grid is not None
    assert (config.grid.capacityCellsX, config.grid.capacityCellsY) == (25, 25)
    assert (config.grid.launcherOffsetX, config.grid.launcherOffsetY) == (15, 15)


@pytest.mark.parametrize(
    ("text", "message"),
    [
        (
            MANUAL.replace("[ports.patterns]", "[ports]\nsloppy = 1\n\n[ports.patterns]"),
            "[ports] has the unknown key 'sloppy'",
        ),
        (MANUAL + "\n[extra]\nx = 1\n", "unknown section [extra]"),
        (MANUAL.replace("max_feedline_utilization = 5", "max_feedline_utilization = 5.5"), "must be an integer"),
        (MANUAL.replace("min_bend_radius = 50.0\n", ""), "[design_rules] lacks the required key 'min_bend_radius'"),
        (
            MANUAL.split("[ports.sequences]", maxsplit=1)[0] + "[design_rules]" + MANUAL.split("[design_rules]")[1],
            "[ports.sequences] is missing",
        ),
        (MANUAL.replace('[chip]\ninput = "routing_config.json"\n', ""), "[chip] is missing"),
        (MANUAL.replace('fixed_outer = ["Qb1.port0"]', "fixed_outer = [1]"), "must be an array of strings"),
        ("not = [toml", "is not TOML"),
    ],
)
def test_problems_are_named(text: str, message: str) -> None:
    """A shape problem is reported with its section and key."""
    with pytest.raises(ConfigError, match=message.replace("[", r"\[").replace("]", r"\]")):
        parse_config(text)


def test_every_problem_is_reported_at_once() -> None:
    """The loader does not stop at the first problem."""
    text = MANUAL.replace("min_bend_radius = 50.0\n", "").replace('input = "routing_config.json"', "input = 3")
    with pytest.raises(ConfigError) as info:
        parse_config(text)
    assert "[chip] input must be a string" in str(info.value)
    assert "[design_rules] lacks the required key 'min_bend_radius'" in str(info.value)


def test_shipped_configurations_carry_only_what_differs_from_the_defaults() -> None:
    """Strict loading rejects defaults written out, except the design rules."""
    assert shipped_config_problems({"ports": {}}) == []
    parse_config(MANUAL, strict=True)

    problems = shipped_config_problems({"grid": {"capacity_cells_x": 50, "launcher_offset_x": 20}})
    assert problems == ["[grid] capacity_cells_x is set to its default 50; remove it"]
    with pytest.raises(ConfigError, match="capacity_cells_x is set to its default"):
        parse_config(MANUAL + "\n[grid]\ncapacity_cells_x = 50\n", strict=True)


def test_configuration_round_trips_through_the_core_bytes() -> None:
    """What write_config packs, read_config unpacks unchanged."""
    config = parse_config(WITH_GRID)
    back = read_config(write_config(config))

    assert back.chipInput == config.chipInput
    assert back.ports is not None
    assert back.ports.sequences is not None
    assert back.ports.sequences.fixedOuter == ["Qb1.port0"]
    assert back.ports.patterns is not None
    assert back.ports.patterns.resonator == r"^Qb\d+\.port0$"
    assert back.grid is not None
    assert back.grid.capacityCellsY == 25
    assert back.rules is not None
    assert back.rules.feedlineTerminations == 1


def test_load_config_names_the_file(tmp_path: Path) -> None:
    """A file is loaded by path, and a missing file is a ConfigError."""
    path = tmp_path / "config.toml"
    path.write_text(MANUAL, encoding="utf-8")
    assert load_config(path).chipInput == "routing_config.json"

    with pytest.raises(ConfigError, match="cannot read"):
        load_config(tmp_path / "absent.toml")
