# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Loading of ``config.toml`` into the configuration schema.

The loader checks the shape of the file: every section and key it knows, the type of every value,
and nothing it does not know. What the values mean, such as whether a pattern compiles or a sequence
names ports of the chip, is checked by the core. The loaded configuration crosses into the core as
the bytes of the ``config.fbs`` schema.
"""

from __future__ import annotations

import tomllib
from typing import TYPE_CHECKING, Any, TypeVar, cast

import flatbuffers

from .flatbuffers.config.Config import ConfigT
from .flatbuffers.config.GridParams import GridParamsT
from .flatbuffers.config.PortConfig import PortConfigT
from .flatbuffers.config.PortPatterns import PortPatternsT
from .flatbuffers.config.PortSequences import PortSequencesT
from .flatbuffers.design.DesignRules import DesignRulesT

if TYPE_CHECKING:
    from pathlib import Path


class ConfigError(ValueError):
    """A configuration that cannot be loaded. The message names every problem."""


T = TypeVar("T")


#: The keys of ``[design_rules]``, all of them required, with the kind of number each holds.
DESIGN_RULES: dict[str, type] = {
    "min_wire_spacing": float,
    "min_obstacle_spacing": float,
    "min_bend_radius": float,
    "min_straight_length": float,
    "target_resonator_length": float,
    "resonator_length_tolerance": float,
    "max_feedline_utilization": int,
    "feedline_terminations": int,
}

#: The keys of ``[grid]`` with the defaults of the schema, which the loader applies to absent keys.
GRID_DEFAULTS: dict[str, int] = {
    "capacity_cells_x": 50,
    "capacity_cells_y": 0,
    "launcher_offset_x": 15,
    "launcher_offset_y": 15,
}


class _Section:
    """One TOML table, read key by key so that unknown keys can be reported at the end."""

    def __init__(self, name: str, table: dict[str, Any], problems: list[str]) -> None:
        self.name = name
        self.table = table
        self.problems = problems
        self.seen: set[str] = set()

    def take(self, key: str, kind: type[T], *, default: T, required: bool = False) -> T:
        """Read one value, checking its type; report a missing required key or a wrong type.

        Returns:
            The value, or ``default`` when the key is absent or holds the wrong type.
        """
        self.seen.add(key)
        if key not in self.table:
            if required:
                self.problems.append(f"[{self.name}] lacks the required key '{key}'")
            return default
        value = self.table[key]
        if not _is_kind(value, kind):
            self.problems.append(f"[{self.name}] {key} must be {_kind_name(kind)}")
            return default
        return cast("T", float(value) if kind is float else value)

    def subtable(self, key: str) -> dict[str, Any] | None:
        """Read a nested table.

        Returns:
            The table, or None when it is absent or not a table.
        """
        self.seen.add(key)
        value = self.table.get(key)
        if value is None:
            return None
        if not isinstance(value, dict):
            self.problems.append(f"[{self.name}] {key} must be a table")
            return None
        return value

    def finish(self) -> None:
        """Report every key the section does not know."""
        for key in self.table:
            if key not in self.seen:
                self.problems.append(f"[{self.name}] has the unknown key '{key}'")


def _is_kind(value: object, kind: type) -> bool:
    if kind is float:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if kind is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if kind is list:
        return isinstance(value, list) and all(isinstance(item, str) for item in value)
    return isinstance(value, kind)


_KIND_NAMES: dict[type, str] = {float: "a number", int: "an integer", str: "a string", list: "an array of strings"}


def _kind_name(kind: type) -> str:
    return _KIND_NAMES[kind]


def _read_ports(table: dict[str, Any], problems: list[str]) -> PortConfigT:
    ports = _Section("ports", table, problems)
    config = PortConfigT()

    patterns_table = ports.subtable("patterns")
    if patterns_table is None:
        problems.append("[ports.patterns] is missing")
    else:
        patterns = _Section("ports.patterns", patterns_table, problems)
        config.patterns = PortPatternsT()
        config.patterns.launcher = patterns.take("launcher", str, required=True, default="")
        config.patterns.resonator = patterns.take("resonator", str, required=True, default="")
        config.patterns.conventional = patterns.take("conventional", str, required=True, default="")
        patterns.finish()

    sequences_table = ports.subtable("sequences")
    if sequences_table is None:
        problems.append("[ports.sequences] is missing")
    else:
        sequences = _Section("ports.sequences", sequences_table, problems)
        config.sequences = PortSequencesT()
        config.sequences.allOuter = list(sequences.take("all_outer", list, required=True, default=[]))
        config.sequences.fixedOuter = list(sequences.take("fixed_outer", list, required=True, default=[]))
        sequences.finish()
    ports.finish()
    return config


def _read_rules(table: dict[str, Any], problems: list[str]) -> DesignRulesT:
    section = _Section("design_rules", table, problems)
    rules = DesignRulesT()
    rules.minWireSpacing = section.take("min_wire_spacing", float, required=True, default=0.0)
    rules.minObstacleSpacing = section.take("min_obstacle_spacing", float, required=True, default=0.0)
    rules.minBendRadius = section.take("min_bend_radius", float, required=True, default=0.0)
    rules.minStraightLength = section.take("min_straight_length", float, required=True, default=0.0)
    rules.targetResonatorLength = section.take("target_resonator_length", float, required=True, default=0.0)
    rules.resonatorLengthTolerance = section.take("resonator_length_tolerance", float, required=True, default=0.0)
    rules.maxFeedlineUtilization = section.take("max_feedline_utilization", int, required=True, default=0)
    rules.feedlineTerminations = section.take("feedline_terminations", int, required=True, default=0)
    section.finish()
    return rules


def _read_grid(table: dict[str, Any], problems: list[str]) -> GridParamsT:
    section = _Section("grid", table, problems)
    grid = GridParamsT()
    grid.capacityCellsX = section.take("capacity_cells_x", int, default=GRID_DEFAULTS["capacity_cells_x"])
    grid.capacityCellsY = section.take("capacity_cells_y", int, default=GRID_DEFAULTS["capacity_cells_y"])
    grid.launcherOffsetX = section.take("launcher_offset_x", int, default=GRID_DEFAULTS["launcher_offset_x"])
    grid.launcherOffsetY = section.take("launcher_offset_y", int, default=GRID_DEFAULTS["launcher_offset_y"])
    section.finish()
    return grid


def shipped_config_problems(document: dict[str, Any]) -> list[str]:
    """The ways a parsed file breaks the rules for a shipped configuration.

    A shipped configuration carries only what differs from the defaults, with one exception that is
    always written out: every design rule.

    Returns:
        One message per broken rule, empty when the file follows them.
    """
    problems: list[str] = []
    grid = document.get("grid")
    if isinstance(grid, dict):
        problems.extend(
            f"[grid] {key} is set to its default {GRID_DEFAULTS[key]}; remove it"
            for key, value in grid.items()
            if key in GRID_DEFAULTS and value == GRID_DEFAULTS[key]
        )
        if not grid:
            problems.append("[grid] is empty; remove it")
    return problems


def parse_config(text: str, *, source: str = "config.toml", strict: bool = False) -> ConfigT:
    """Parse the text of a ``config.toml``.

    Args:
        text: The TOML text.
        source: How to name the file in messages.
        strict: Also apply the rules for a shipped configuration.

    Returns:
        The configuration, with the schema defaults applied to absent keys.

    Raises:
        ConfigError: If the text is not TOML, or has a problem. The message names every problem.
    """
    try:
        document = tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
        msg = f"{source} is not TOML: {error}"
        raise ConfigError(msg) from error

    problems: list[str] = []
    root = _Section("", document, problems)
    config = ConfigT()

    chip = root.subtable("chip")
    if chip is None:
        problems.append("[chip] is missing")
    else:
        section = _Section("chip", chip, problems)
        config.chipInput = section.take("input", str, required=True, default="")
        section.finish()

    ports = root.subtable("ports")
    if ports is None:
        problems.append("[ports] is missing")
    else:
        config.ports = _read_ports(ports, problems)

    rules = root.subtable("design_rules")
    if rules is None:
        problems.append("[design_rules] is missing")
    else:
        config.rules = _read_rules(rules, problems)

    grid = root.subtable("grid")
    config.grid = _read_grid(grid, problems) if grid is not None else None

    problems.extend(
        f"unknown section [{key}]" for key in document if key not in {"chip", "ports", "design_rules", "grid"}
    )
    if strict:
        problems.extend(shipped_config_problems(document))
    if problems:
        msg = f"{source}: " + "; ".join(problems)
        raise ConfigError(msg)
    return config


def load_config(path: Path, *, strict: bool = False) -> ConfigT:
    """Load a ``config.toml`` file.

    Args:
        path: The file.
        strict: Also apply the rules for a shipped configuration.

    Returns:
        The configuration.

    Raises:
        ConfigError: If the file cannot be read or has a problem.
    """
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        msg = f"cannot read {path}: {error}"
        raise ConfigError(msg) from error
    return parse_config(text, source=str(path), strict=strict)


def write_config(config: ConfigT) -> bytes:
    """Serialize a configuration as the core takes it.

    Returns:
        The bytes of the ``config.fbs`` schema.
    """
    builder = flatbuffers.Builder()
    builder.Finish(config.Pack(builder))
    return bytes(builder.Output())


def read_config(data: bytes) -> ConfigT:
    """Decode a configuration that ``write_config`` produced.

    Returns:
        The configuration, with the sequence labels as text.
    """
    config = ConfigT.InitFromPackedBuf(data, 0)
    # The generated decoder leaves the elements of a string vector as bytes.
    sequences = config.ports.sequences if config.ports is not None else None
    if sequences is not None:
        sequences.allOuter = [_text(label) for label in sequences.allOuter or []]
        sequences.fixedOuter = [_text(label) for label in sequences.fixedOuter or []]
    return config


def _text(label: str | bytes | None) -> str:
    return label.decode("utf-8") if isinstance(label, bytes) else (label or "")
