# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of loading the chip input through the core."""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path

import pytest

from mqt.scpd.chip import (
    ROLE_NAMES,
    ChipError,
    chip_input_path,
    decode_chip,
    load_chip,
    obstacles_of,
    ports_of,
    role_name,
)
from mqt.scpd.config import load_config
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"


def test_the_fixture_chip_loads_and_classifies() -> None:
    """The fixture input loads with its configuration, and its sequences fit the chip."""
    config_path = FIXTURE / "config.toml"
    config = load_config(config_path)
    assert chip_input_path(config, config_path) == FIXTURE / "routing_config.json"

    chip = decode_chip(load_chip(config, config_path))

    assert len(obstacles_of(chip)) == 4
    ports = ports_of(chip)
    assert Counter(role_name(port.role) for port in ports) == {"launcher": 4, "resonator": 2, "conventional": 3}
    assert ports[0].label == "Chip.port0"
    assert ports[0].center is not None


def test_a_chip_path_replaces_the_configured_input(tmp_path: Path) -> None:
    """A chip named beside the configuration is read instead of the input the file names."""
    config_path = tmp_path / "config.toml"
    config_path.write_text((FIXTURE / "config.toml").read_text(encoding="utf-8"), encoding="utf-8")
    config = load_config(config_path)
    chip_path = FIXTURE / "routing_config.json"

    assert chip_input_path(config, config_path, chip_path) == chip_path
    assert len(ports_of(decode_chip(load_chip(config, config_path, chip_path)))) == 9
    with pytest.raises(ChipError, match="cannot read the chip input"):
        load_chip(config, config_path)


def test_a_chip_that_does_not_fit_its_configuration_is_refused() -> None:
    """A pattern that leaves ports unmatched, and a sequence label the chip lacks, are named."""
    config_path = FIXTURE / "config.toml"
    config = load_config(config_path)
    assert config.ports is not None
    assert config.ports.patterns is not None
    config.ports.patterns.conventional = r"^Q\d+\.port1$"

    with pytest.raises(ChipError, match=re.escape("'C12.port0' matches no role pattern")):
        load_chip(config, config_path)

    config = load_config(config_path)
    assert config.ports is not None
    assert config.ports.sequences is not None
    config.ports.sequences.allOuter.append("Q9.port0")
    with pytest.raises(ChipError, match=re.escape("'Q9.port0' is not a port of the chip")):
        load_chip(config, config_path)

    config = load_config(config_path)
    config.chipInput = "absent.json"
    with pytest.raises(ChipError, match="cannot read the chip input"):
        load_chip(config, config_path)


def test_role_names_follow_the_schema() -> None:
    """Every role has the name the configuration keys use."""
    assert role_name(UnassignedRole.Launcher) == "launcher"
    assert role_name(UnassignedRole.Coupler) == "coupler"
    assert role_name(99) == "unset"


def test_roles_are_named_as_the_configuration_keys_spell_them() -> None:
    """Every role of the schema has the name of its configuration key, and nothing else has one."""
    assert {
        UnassignedRole.Unset: "unset",
        UnassignedRole.Launcher: "launcher",
        UnassignedRole.Resonator: "resonator",
        UnassignedRole.Conventional: "conventional",
        UnassignedRole.Coupler: "coupler",
        UnassignedRole.BridgePair: "bridge_pair",
    } == ROLE_NAMES
    assert role_name(UnassignedRole.BridgePair) == "bridge_pair"
