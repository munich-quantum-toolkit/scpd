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
from .flatbuffers.config.CorridorParams import CorridorParamsT
from .flatbuffers.config.DebugParams import DebugParamsT
from .flatbuffers.config.DetailParams import DetailParamsT
from .flatbuffers.config.FinalParams import FinalParamsT
from .flatbuffers.config.GlobalParams import GlobalParamsT
from .flatbuffers.config.GridParams import GridParamsT
from .flatbuffers.config.PortConfig import PortConfigT
from .flatbuffers.config.PortPatterns import PortPatternsT
from .flatbuffers.config.PortSequences import PortSequencesT
from .flatbuffers.config.RunParams import RunParamsT
from .flatbuffers.config.SolverParams import SolverParamsT
from .flatbuffers.config.StageParams import StageParamsT
from .flatbuffers.design.DesignRules import DesignRulesT
from .steps import FINAL_PHASES, STEPS

if TYPE_CHECKING:
    from pathlib import Path


class ConfigError(ValueError):
    """A configuration that cannot be loaded. The message names every problem."""


#: Every integer of the schema is a ``uint32``.
INT32_LIMIT = 2**31
UINT32_MAX = 2**32 - 1


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
GRID_DEFAULTS: dict[str, float] = {
    "capacity_cells_x": 50,
    "capacity_cells_y": 0,
    "detail_factor": 30,
    "router_cell_size": 10.0,
}

#: The keys that no stage reads any more, by section. The loader still accepts them, and
#: ``removed_keys`` names each one, so that a configuration can drop them before a later version
#: refuses them.
REMOVED_KEYS: dict[str, tuple[str, ...]] = {"grid": ("launcher_offset_x", "launcher_offset_y")}

#: The keys of each ``[stages.*]`` section with the defaults of the schema. A shipped configuration
#: carries only what differs from them, the rule the grid section follows.
STAGE_DEFAULTS: dict[str, dict[str, object]] = {
    "capacity": {"planner": "", "bottleneck_clearance": 1.5, "crossing_pitch": 165.0},
    "global": {"router": "", "internal_bridges": False},
    "assignment": {"assigner": "", "launcher_target": 0},
    "solver": {"backend": "", "time_limit": 0.0, "relative_gap": 0.0, "threads": 0},
    "corridor": {"router": "", "rounds": 12, "max_relaxation": 4},
    "detail": {
        "router": "",
        "corridor_spacings": 4,
        "rounds": 30,
        "max_relaxation": 30,
        "obstacle_penalty_reach": 185.0,
        "obstacle_penalty": 10,
        "orderings": 16,
    },
    "final": {
        "router": "",
        "corridor_spacings": 11,
        "inner_corridor_spacings": 11,
        "rounds": 30,
        "inner_rounds": 4,
        "max_relaxation": 10,
        "refinement_rounds": 2,
        "feedline_refinement_rounds": 2,
        "refinement_corridor_spacings": 6,
        "bend_penalty_norm": 2.5,
        "wire_proximity_penalty_norm": 0.00125,
        "static_proximity_penalty_norm": 0.00033,
        "obstacle_penalty_reach": 100.0,
        "coupler_length": 200.0,
        "coupler_height": 26.0,
        "repair_trials": 0,
        "coupler_bias": 0.98,
        "coupler_max_shortfall": 0.30,
        "coupler_box_margin_cells": 5,
        "coupler_shift_cells": 20,
        "coupler_lead_margin_cells": 5,
        "launcher_lead": 10.0,
        "coupler_offsets": "",
        "edge_launcher_margin_cells": 6,
        "terminal_slot_cells": 20,
        "chain_end_edge_weight": 0.1,
        "edge_bend_factor": 1.0,
        "chain_length_weight": 0,
        "capacity_check": True,
        "length_point_radius_cells": 40,
        "length_point_band_percent": 10.0,
        "sweep_length_band": 1.0,
        "meander_leg_spacing_cells": 25,
        "meander_start_margin_cells": -1,
        "capacity_margin": 1.06,
        "capacity_margin_from_wires": 3,
        "resonator_passage": 0.8,
        "crossing_reach_cells": 10,
        "crossing_spacing": 1.4142135623730951,
        "slot_pin_cells": 10,
        "escape_straight_cells": 5,
        "escape_stub_margin_cells": 4,
        "bottleneck_wires": 10,
        "bottleneck_rise": 2.0,
        "room_margin_cells": 10,
        "room_channel_reach_cells": 80,
        "detour_factor": 1.5,
        "terminal_stub_extra_cells": 5,
        "global_sat_runs": 8,
        "global_sat_states": 8,
        "global_sat_tries": 200,
        "chain_rule_refusals": 1500,
        "research_ring_members": 2,
        "research_rounds": 2,
        "fence_pairs": 3,
        "halo_reach": 1,
        "rise_reach": 3,
        "rise_peak": 3,
    },
}

#: The keys of ``[debug]``: the pages a run writes under ``<run>/debug``, every one off by default.
#: The key names the page; ``html`` is the index that links them.
DEBUG_PAGES: tuple[str, ...] = (
    "html",
    "pre_feedline_routing_dashboard",
    "pre_feedline_refinement_dashboard",
    "feedline_insertion",
    "local_sat",
    "global_sat",
    "feedline_constrained_routing_dashboard",
    "feedline_constrained_refinement_dashboard",
)

#: The keys of ``[run]`` with the defaults of the schema.
RUN_DEFAULTS: dict[str, object] = {"stop_after": ""}


class _Section:
    """One TOML table, read key by key so that unknown keys can be reported at the end."""

    def __init__(self, name: str, table: dict[str, Any], problems: list[str]) -> None:
        self.name = name
        self.table = table
        self.problems = problems
        self.seen: set[str] = set()

    def take(self, key: str, kind: type[T], *, default: T, required: bool = False, signed: bool = False) -> T:
        """Read one value, checking its type; report a missing required key or a wrong type.

        Args:
            key: The key.
            kind: The kind of value the key holds.
            default: What an absent key reads as.
            required: Whether an absent key is a problem.
            signed: Whether a number may be negative; a count may not.

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
        if kind is int and not signed and not 0 <= value <= UINT32_MAX:
            self.problems.append(f"[{self.name}] {key} must be between 0 and {UINT32_MAX}")
            return default
        if kind is int and signed and not -INT32_LIMIT <= value < INT32_LIMIT:
            self.problems.append(f"[{self.name}] {key} must be between {-INT32_LIMIT} and {INT32_LIMIT - 1}")
            return default
        if kind is float:
            try:
                return cast("T", float(value))
            except OverflowError:
                self.problems.append(f"[{self.name}] {key} is too large for a number")
                return default
        return cast("T", value)

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

    patterns_table = ports.subtable("patterns")
    if patterns_table is None:
        problems.append("[ports.patterns] is missing")
    else:
        patterns = _Section("ports.patterns", patterns_table, problems)
        config.patterns = PortPatternsT()
        config.patterns.launcher = patterns.take("launcher", str, required=True, default="")
        config.patterns.resonator = patterns.take("resonator", str, required=True, default="")
        config.patterns.conventional = patterns.take("conventional", str, required=True, default="")
        # Both are optional. A bridge port is one where a wire crosses a component rather than ends,
        # and [[ports.bridge_pairs]] says which two of them pair. The component pattern declares
        # which part of a label names the component, so no algorithm reads a label itself.
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

    config.bridgePairs = []
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
    grid.capacityCellsX = section.take("capacity_cells_x", int, default=int(GRID_DEFAULTS["capacity_cells_x"]))
    grid.capacityCellsY = section.take("capacity_cells_y", int, default=int(GRID_DEFAULTS["capacity_cells_y"]))
    section.seen.update(REMOVED_KEYS["grid"])
    grid.detailFactor = section.take("detail_factor", int, default=int(GRID_DEFAULTS["detail_factor"]))
    # A length in layout units: the size of a router cell, which the routing stages work on.
    grid.routerCellSize = section.take("router_cell_size", float, default=GRID_DEFAULTS["router_cell_size"])
    section.finish()
    return grid


def _read_stages(table: dict[str, Any], problems: list[str]) -> StageParamsT:
    """Read the stage sections, with the defaults for everything the file leaves out.

    Every section is built whether or not the file carries it, so an absent section behaves exactly
    like one that states only defaults, and a stage never has to know whether the file named it.

    Returns:
        The stage parameters.
    """
    section = _Section("stages", table, problems)
    stages = StageParamsT()

    def part(name: str) -> _Section:
        return _Section(f"stages.{name}", section.subtable(name) or {}, problems)

    def take(sub: _Section, key: str, kind: type[T]) -> T:
        return sub.take(key, kind, default=cast("T", STAGE_DEFAULTS[sub.name.removeprefix("stages.")][key]))

    capacity = part("capacity")
    stages.capacity = CapacityParamsT()
    stages.capacity.planner = take(capacity, "planner", str)
    stages.capacity.bottleneckClearance = take(capacity, "bottleneck_clearance", float)
    stages.capacity.crossingPitch = take(capacity, "crossing_pitch", float)
    capacity.finish()

    inner = part("global")
    stages.global_ = GlobalParamsT()
    stages.global_.router = take(inner, "router", str)
    stages.global_.internalBridges = take(inner, "internal_bridges", bool)
    inner.finish()

    assignment = part("assignment")
    stages.assignment = AssignmentParamsT()
    stages.assignment.assigner = take(assignment, "assigner", str)
    stages.assignment.launcherTarget = take(assignment, "launcher_target", int)
    assignment.finish()

    solver = part("solver")
    stages.solver = SolverParamsT()
    stages.solver.backend = take(solver, "backend", str)
    stages.solver.timeLimit = take(solver, "time_limit", float)
    stages.solver.relativeGap = take(solver, "relative_gap", float)
    stages.solver.threads = take(solver, "threads", int)
    solver.finish()

    corridor = part("corridor")
    stages.corridor = CorridorParamsT()
    stages.corridor.router = take(corridor, "router", str)
    stages.corridor.rounds = take(corridor, "rounds", int)
    stages.corridor.maxRelaxation = take(corridor, "max_relaxation", int)
    corridor.finish()

    detail = part("detail")
    stages.detail = DetailParamsT()
    stages.detail.router = take(detail, "router", str)
    for key, field in (
        ("corridor_spacings", "corridorSpacings"),
        ("rounds", "rounds"),
        ("max_relaxation", "maxRelaxation"),
        ("obstacle_penalty", "obstaclePenalty"),
        ("orderings", "orderings"),
    ):
        setattr(stages.detail, field, take(detail, key, int))
    # A length in layout units, unlike everything else here, because what it bounds is a
    # distance on the chip and not a count of anything.
    stages.detail.obstaclePenaltyReach = take(detail, "obstacle_penalty_reach", float)
    detail.finish()

    final = part("final")
    stages.final = FinalParamsT()
    stages.final.router = take(final, "router", str)
    for key, field in (
        ("corridor_spacings", "corridorSpacings"),
        ("inner_corridor_spacings", "innerCorridorSpacings"),
        ("rounds", "rounds"),
        ("inner_rounds", "innerRounds"),
        ("max_relaxation", "maxRelaxation"),
        ("refinement_rounds", "refinementRounds"),
        ("feedline_refinement_rounds", "feedlineRefinementRounds"),
        ("refinement_corridor_spacings", "refinementCorridorSpacings"),
        ("repair_trials", "repairTrials"),
        ("coupler_box_margin_cells", "couplerBoxMarginCells"),
        ("coupler_shift_cells", "couplerShiftCells"),
        ("coupler_lead_margin_cells", "couplerLeadMarginCells"),
        ("edge_launcher_margin_cells", "edgeLauncherMarginCells"),
        ("terminal_slot_cells", "terminalSlotCells"),
        ("chain_length_weight", "chainLengthWeight"),
        ("length_point_radius_cells", "lengthPointRadiusCells"),
        ("meander_leg_spacing_cells", "meanderLegSpacingCells"),
        ("capacity_margin_from_wires", "capacityMarginFromWires"),
        ("crossing_reach_cells", "crossingReachCells"),
        ("slot_pin_cells", "slotPinCells"),
        ("escape_straight_cells", "escapeStraightCells"),
        ("escape_stub_margin_cells", "escapeStubMarginCells"),
        ("bottleneck_wires", "bottleneckWires"),
        ("room_margin_cells", "roomMarginCells"),
        ("room_channel_reach_cells", "roomChannelReachCells"),
        ("terminal_stub_extra_cells", "terminalStubExtraCells"),
        ("global_sat_runs", "globalSatRuns"),
        ("global_sat_states", "globalSatStates"),
        ("global_sat_tries", "globalSatTries"),
        ("chain_rule_refusals", "chainRuleRefusals"),
        ("research_ring_members", "researchRingMembers"),
        ("research_rounds", "researchRounds"),
        ("fence_pairs", "fencePairs"),
        ("halo_reach", "haloReach"),
        ("rise_reach", "riseReach"),
        ("rise_peak", "risePeak"),
    ):
        setattr(stages.final, field, take(final, key, int))
    # Lengths in layout units and prices against the grid extent. None of them is a cell
    # count: the same count is a different distance on every one of the eight grids.
    for key, field in (
        ("bend_penalty_norm", "bendPenaltyNorm"),
        ("wire_proximity_penalty_norm", "wireProximityPenaltyNorm"),
        ("static_proximity_penalty_norm", "staticProximityPenaltyNorm"),
        ("obstacle_penalty_reach", "obstaclePenaltyReach"),
        ("coupler_length", "couplerLength"),
        ("coupler_height", "couplerHeight"),
        ("coupler_bias", "couplerBias"),
        ("coupler_max_shortfall", "couplerMaxShortfall"),
        ("launcher_lead", "launcherLead"),
        ("chain_end_edge_weight", "chainEndEdgeWeight"),
        ("edge_bend_factor", "edgeBendFactor"),
        ("length_point_band_percent", "lengthPointBandPercent"),
        ("sweep_length_band", "sweepLengthBand"),
        ("capacity_margin", "capacityMargin"),
        ("resonator_passage", "resonatorPassage"),
        ("crossing_spacing", "crossingSpacing"),
        ("bottleneck_rise", "bottleneckRise"),
        ("detour_factor", "detourFactor"),
    ):
        setattr(stages.final, field, take(final, key, float))
    # A margin in cells, or -1 for the coupler length plus four: the one key below zero.
    margin = final.take("meander_start_margin_cells", int, default=-1, signed=True)
    stages.final.meanderStartMarginCells = margin
    stages.final.capacityCheck = take(final, "capacity_check", bool)
    stages.final.couplerOffsets = take(final, "coupler_offsets", str)
    final.finish()

    section.finish()
    return stages


def _read_debug(table: dict[str, Any], problems: list[str]) -> DebugParamsT:
    """Read which debug pages a run writes; every page is off unless the file turns it on.

    Returns:
        The debug parameters.
    """
    section = _Section("debug", table, problems)
    debug = DebugParamsT()
    for key in DEBUG_PAGES:
        setattr(debug, _camel(key), section.take(key, bool, default=False))
    section.finish()
    return debug


def _camel(key: str) -> str:
    """The attribute name the generated object API gives a schema field.

    Returns:
        ``pre_feedline_routing_dashboard`` as ``preFeedlineRoutingDashboard``.
    """
    first, *rest = key.split("_")
    return first + "".join(word.capitalize() for word in rest)


def _read_run(table: dict[str, Any], problems: list[str]) -> RunParamsT:
    """Read how far a run goes; the step must be one this release runs, or a phase of the Final stage.

    Returns:
        The run parameters.
    """
    section = _Section("run", table, problems)
    run = RunParamsT()
    run.stopAfter = section.take("stop_after", str, default="")
    if run.stopAfter and run.stopAfter not in STEPS and run.stopAfter not in FINAL_PHASES:
        named = ", ".join(f"'{step}'" for step in STEPS + FINAL_PHASES)
        problems.append(f"[run] stop_after must be empty or one of {named}")
        run.stopAfter = ""
    section.finish()
    return run


def _toml(value: object) -> str:
    """A default as the file would spell it.

    Returns:
        The value in TOML.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)


def _is_default(value: object, default: object) -> bool:
    """Whether a value written in the file is the default, compared as the key's kind.

    A whole number in a key that holds a number is that number, and a flag is never a number.

    Returns:
        True when the value is the default.
    """
    if isinstance(default, bool) or isinstance(value, bool):
        return isinstance(value, bool) and isinstance(default, bool) and value == default
    if isinstance(default, float):
        return isinstance(value, (int, float)) and value == default
    return type(value) is type(default) and value == default


def _defaults_problems(name: str, section: dict[str, Any], defaults: dict[str, object]) -> list[str]:
    problems = [
        f"[{name}] {key} is set to its default {_toml(defaults[key])}; remove it"
        for key, value in section.items()
        if key in defaults and _is_default(value, defaults[key])
    ]
    if not section:
        problems.append(f"[{name}] is empty; remove it")
    return problems


def removed_keys(document: dict[str, Any]) -> list[str]:
    """The keys of a parsed file that no stage reads any more.

    Returns:
        One message per key, empty when the file carries none.
    """
    messages: list[str] = []
    for name, keys in REMOVED_KEYS.items():
        section = document.get(name)
        if isinstance(section, dict):
            messages.extend(
                f"[{name}] {key} has no effect, and a later version will refuse it; remove it"
                for key in keys
                if key in section
            )
    return messages


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
        for name in sorted(stages):
            section = stages[name]
            if name in STAGE_DEFAULTS and isinstance(section, dict):
                problems.extend(_defaults_problems(f"stages.{name}", section, STAGE_DEFAULTS[name]))
        if not stages:
            problems.append("[stages] is empty; remove it")
    run = document.get("run")
    if isinstance(run, dict):
        problems.extend(_defaults_problems("run", run, RUN_DEFAULTS))
    problems.extend(removed_keys(document))
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
    config.stages = _read_stages(root.subtable("stages") or {}, problems)
    config.run = _read_run(root.subtable("run") or {}, problems)
    config.debug = _read_debug(root.subtable("debug") or {}, problems)

    problems.extend(
        f"unknown section [{key}]"
        for key in document
        if key not in {"chip", "ports", "design_rules", "grid", "stages", "run", "debug"}
    )
    if strict:
        problems.extend(shipped_config_problems(document))
    if problems:
        msg = f"{source}: " + "; ".join(problems)
        raise ConfigError(msg)
    return config


def removed_keys_in(path: Path) -> list[str]:
    """The keys of a ``config.toml`` file that no stage reads any more, as ``removed_keys`` names them.

    Returns:
        One message per key, empty when the file carries none or cannot be read as TOML.
    """
    try:
        document = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError):
        return []
    return removed_keys(document)


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
    except UnicodeDecodeError as error:
        msg = f"cannot read {path}: it is not UTF-8 ({error.reason} at byte {error.start})"
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
