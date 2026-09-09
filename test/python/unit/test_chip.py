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

from mqt.scpd.chip import ChipError, chip_input_path, decode_chip, load_chip, obstacles_of, ports_of, role_name
from mqt.scpd.config import load_config
from mqt.scpd.flatbuffers.design.UnassignedRole import UnassignedRole

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"


def test_the_four_qubit_chip_loads_and_classifies() -> None:
    """The committed 4-qubit input loads with its shipped configuration."""
    config_path = BENCHMARKS / "4q" / "config.toml"
    config = load_config(config_path)
    assert chip_input_path(config, config_path) == BENCHMARKS / "4q" / "routing_config.json"

    chip = decode_chip(load_chip(config, config_path))

    assert len(obstacles_of(chip)) == 28
    ports = ports_of(chip)
    # Its couplers carry three ports, so one pair of each bridges and the third ends a wire.
    assert Counter(role_name(port.role) for port in ports) == {
        "launcher": 16,
        "resonator": 4,
        "conventional": 8,
        "bridge_pair": 8,
    }
    assert ports[0].label
    assert ports[0].center is not None


def test_the_nine_qubit_chip_loads_with_its_sequences() -> None:
    """The committed 9-qubit input classifies every port and accepts its sequences."""
    config_path = BENCHMARKS / "9q" / "config.toml"
    chip = decode_chip(load_chip(load_config(config_path), config_path))

    assert Counter(role_name(port.role) for port in ports_of(chip)) == {
        "launcher": 24,
        "resonator": 9,
        "conventional": 21,
        "bridge_pair": 48,
    }


def test_a_chip_that_does_not_fit_its_configuration_is_refused() -> None:
    """A pattern that leaves ports unmatched, and a sequence label the chip lacks, are named."""
    config_path = BENCHMARKS / "4q" / "config.toml"
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
    # A name in more than one word is the configuration key, not the schema spelling.
    assert role_name(UnassignedRole.BridgePair) == "bridge_pair"
    assert role_name(99) == "unset"
