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

from .flatbuffers.config.AssignmentParams import AssignmentParamsT
from .flatbuffers.config.BridgeRule import BridgeRuleT
from .flatbuffers.config.CapacityParams import CapacityParamsT
from .flatbuffers.config.Config import ConfigT
from .flatbuffers.config.GlobalParams import GlobalParamsT
from .flatbuffers.config.GridParams import GridParamsT
from .flatbuffers.config.PortConfig import PortConfigT
from .flatbuffers.config.PortPatterns import PortPatternsT
from .flatbuffers.config.PortSequences import PortSequencesT
from .flatbuffers.config.SolverParams import SolverParamsT
from .flatbuffers.config.StageParams import StageParamsT
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
    "detail_factor": 30,
}

#: The defaults of the stage sections, by section and key. A shipped configuration carries only
#: what differs from them, the same rule the grid section follows.
STAGE_DEFAULTS: dict[str, dict[str, object]] = {
    "capacity": {"planner": "", "bottleneck_clearance": 1.5},
    "global": {"router": "", "internal_bridges": False},
    "assignment": {"assigner": "", "launcher_target": 0},
    "solver": {"backend": "", "time_limit": 0.0, "relative_gap": 0.0},
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

    def subtables(self, key: str) -> list[dict[str, Any]]:
        """Read an array of nested tables.

        Returns:
            The tables, empty when the key is absent or does not hold an array of tables.
        """
        self.seen.add(key)
        value = self.table.get(key)
        if value is None:
            return []
        if not isinstance(value, list) or not all(isinstance(item, dict) for item in value):
            self.problems.append(f"[{self.name}] {key} must be an array of tables")
            return []
        return cast("list[dict[str, Any]]", value)

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


_KIND_NAMES: dict[type, str] = {
    float: "a number",
    int: "an integer",
    bool: "true or false",
    str: "a string",
    list: "an array of strings",
}


def _kind_name(kind: type) -> str:
    return _KIND_NAMES[kind]


def _read_ports(table: dict[str, Any], problems: list[str]) -> PortConfigT:
    ports = _Section("ports", table, problems)
    config = PortConfigT()
    config.bridgePairs = []

    patterns_table = ports.subtable("patterns")
    if patterns_table is None:
        problems.append("[ports.patterns] is missing")
    else:
        patterns = _Section("ports.patterns", patterns_table, problems)
        config.patterns = PortPatternsT()
        config.patterns.launcher = patterns.take("launcher", str, required=True, default="")
        config.patterns.resonator = patterns.take("resonator", str, required=True, default="")
        config.patterns.conventional = patterns.take("conventional", str, required=True, default="")
        # The component grouping is declared rather than parsed out of a label. It is optional
        # because a chip whose planning stages need no grouping does not have to state one.
        # A bridge port is one a wire crosses the component at rather than ends at. The pattern
        # says which ports those are; [[ports.bridge_pairs]] below says which two of them pair.
        # Both are optional, because a chip whose components carry no crossing declares neither.
        config.patterns.bridgePair = patterns.take("bridge_pair", str, default="")
        config.patterns.component = patterns.take("component", str, default="")
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

    for index, rule_table in enumerate(ports.subtables("bridge_pairs")):
        rule_section = _Section(f"ports.bridge_pairs[{index}]", rule_table, problems)
        rule = BridgeRuleT()
        rule.first = rule_section.take("first", str, required=True, default="")
        rule.second = rule_section.take("second", str, required=True, default="")
        rule_section.finish()
        config.bridgePairs.append(rule)

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
    grid.detailFactor = section.take("detail_factor", int, default=GRID_DEFAULTS["detail_factor"])
    section.finish()
    return grid


def _read_stages(table: dict[str, Any], problems: list[str]) -> StageParamsT:
    """Read the stage sections, applying the defaults to everything a file leaves out.

    Every section is built whether or not the file carries it, so that an absent section behaves
    exactly like one that states only defaults. A stage that reads a parameter therefore never has
    to know whether the file mentioned it.

    Returns:
        The stage parameters.
    """
    section = _Section("stages", table, problems)
    stages = StageParamsT()
    defaults = STAGE_DEFAULTS

    capacity = _Section("stages.capacity", section.subtable("capacity") or {}, problems)
    stages.capacity = CapacityParamsT()
    stages.capacity.planner = capacity.take("planner", str, default="")
    stages.capacity.bottleneckClearance = capacity.take(
        "bottleneck_clearance", float, default=defaults["capacity"]["bottleneck_clearance"]
    )
    capacity.finish()

    router = _Section("stages.global", section.subtable("global") or {}, problems)
    stages.global_ = GlobalParamsT()
    stages.global_.router = router.take("router", str, default="")
    stages.global_.internalBridges = router.take(
        "internal_bridges", bool, default=defaults["global"]["internal_bridges"]
    )
    router.finish()

    assignment = _Section("stages.assignment", section.subtable("assignment") or {}, problems)
    stages.assignment = AssignmentParamsT()
    stages.assignment.assigner = assignment.take("assigner", str, default="")
    stages.assignment.launcherTarget = assignment.take("launcher_target", int, default=0)
    assignment.finish()

    solver = _Section("stages.solver", section.subtable("solver") or {}, problems)
    stages.solver = SolverParamsT()
    stages.solver.backend = solver.take("backend", str, default="")
    stages.solver.timeLimit = solver.take("time_limit", float, default=0.0)
    stages.solver.relativeGap = solver.take("relative_gap", float, default=0.0)
    solver.finish()

    section.finish()
    return stages


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

    stages = document.get("stages")
    if isinstance(stages, dict):
        for name, defaults in STAGE_DEFAULTS.items():
            section = stages.get(name)
            if not isinstance(section, dict):
                continue
            problems.extend(
                f"[stages.{name}] {key} is set to its default {defaults[key]}; remove it"
                for key, value in section.items()
                if key in defaults and value == defaults[key]
            )
            if not section:
                problems.append(f"[stages.{name}] is empty; remove it")
        if not stages:
            problems.append("[stages] is empty; remove it")
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

    config.grid = _read_grid(root.subtable("grid") or {}, problems)

    # The stage sections are always built, so an absent one is the defaults rather than nothing.
    config.stages = _read_stages(root.subtable("stages") or {}, problems)

    known = {"chip", "ports", "design_rules", "grid", "stages"}
    problems.extend(f"unknown section [{key}]" for key in document if key not in known)
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
