# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the config.toml loader."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from mqt.scpd.config import (
    DESIGN_RULES,
    GRID_DEFAULTS,
    RUN_DEFAULTS,
    STAGE_DEFAULTS,
    ConfigError,
    load_config,
    parse_config,
    read_config,
    removed_keys,
    removed_keys_in,
    shipped_config_problems,
    write_config,
)

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"

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

WITH_BRIDGES = (
    MANUAL.replace(
        "conventional = '^(Qb\\d+\\.port1|Coupler\\d+_\\d+\\.port[0-4])$'",
        "conventional = '^(Qb\\d+\\.port1|Coupler\\d+_\\d+\\.port0)$'\n"
        "bridge_pair = '^Coupler\\d+_\\d+\\.port[1-4]$'\n"
        "component = '^([^.]+)\\.port\\d+$'",
    )
    + """
[[ports.bridge_pairs]]
first = '^(Coupler\\d+_\\d+)\\.port1$'
second = '^(Coupler\\d+_\\d+)\\.port2$'

[[ports.bridge_pairs]]
first = '^(Coupler\\d+_\\d+)\\.port3$'
second = '^(Coupler\\d+_\\d+)\\.port4$'
"""
)

STAGES = """
[stages.capacity]
crossing_pitch = 150.0

[stages.global]
internal_bridges = true

[stages.assignment]
launcher_target = 7

[stages.solver]
backend = "highs"
time_limit = 30
threads = 4

[stages.corridor]
rounds = 6

[run]
stop_after = "assign"
"""

WITH_STAGES = MANUAL + STAGES


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
    assert config.grid.detailFactor == 30


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
        (MANUAL.replace("max_feedline_utilization = 5", "max_feedline_utilization = -5"), "between 0 and 4294967295"),
        (WITH_GRID.replace("capacity_cells_x = 25", "capacity_cells_x = 4294967296"), "between 0 and 4294967295"),
        (MANUAL.replace("min_bend_radius = 50.0", "min_bend_radius = 1" + "0" * 400), "too large for a number"),
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

    problems = shipped_config_problems({"grid": {"capacity_cells_x": 50, "detail_factor": 20}})
    assert problems == ["[grid] capacity_cells_x is set to its default 50; remove it"]
    with pytest.raises(ConfigError, match="capacity_cells_x is set to its default"):
        parse_config(MANUAL + "\n[grid]\ncapacity_cells_x = 50\n", strict=True)


def test_a_removed_key_loads_and_is_named() -> None:
    """A key that no stage reads any more still loads, and the loader names it so it can go."""
    text = WITH_GRID.replace("capacity_cells_x = 25", "capacity_cells_x = 25\nlauncher_offset_x = 0")
    config = parse_config(text)

    assert config.grid is not None
    assert config.grid.capacityCellsX == 25
    message = "[grid] launcher_offset_x has no effect, and a later version will refuse it; remove it"
    assert removed_keys({"grid": {"launcher_offset_x": 0, "capacity_cells_x": 25}}) == [message]
    assert removed_keys({"grid": {"capacity_cells_x": 25}}) == []
    assert shipped_config_problems({"grid": {"launcher_offset_y": 7}}) == [message.replace("_x", "_y")]


def test_removed_keys_in_reads_the_file(tmp_path: Path) -> None:
    """The file variant names the removed keys of a file, and nothing for a file it cannot read."""
    path = tmp_path / "config.toml"
    path.write_text(MANUAL + "\n[grid]\nlauncher_offset_y = 0\n", encoding="utf-8")
    assert removed_keys_in(path) == [
        "[grid] launcher_offset_y has no effect, and a later version will refuse it; remove it"
    ]
    assert removed_keys_in(tmp_path / "absent.toml") == []
    latin = tmp_path / "latin.toml"
    latin.write_bytes("# Gr\u00f6\u00dfe\n".encode("latin-1"))
    assert removed_keys_in(latin) == []


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


def test_the_stage_and_run_sections_default_when_the_file_leaves_them_out() -> None:
    """Every stage section is built, so an absent section behaves like one of defaults."""
    config = parse_config(MANUAL)

    stages = config.stages
    assert stages is not None
    assert stages.capacity is not None
    assert (stages.capacity.planner,) == ("",)
    assert stages.capacity.bottleneckClearance == pytest.approx(1.5)
    assert stages.capacity.crossingPitch == pytest.approx(165.0)
    assert stages.global_ is not None
    assert (stages.global_.router,) == ("",)
    assert stages.global_.internalBridges is False
    assert stages.assignment is not None
    assert stages.assignment.launcherTarget == 0
    assert stages.solver is not None
    assert (stages.solver.backend,) == ("",)
    assert stages.solver.timeLimit == pytest.approx(0.0)
    assert stages.solver.threads == 0
    assert stages.corridor is not None
    assert (stages.corridor.rounds, stages.corridor.maxRelaxation) == (12, 4)
    assert config.run is not None
    assert (config.run.stopAfter,) == ("",)


def test_a_stage_section_sets_what_it_names() -> None:
    """The keys a stage section carries replace the defaults; the rest stay."""
    config = parse_config(WITH_STAGES)

    stages = config.stages
    assert stages is not None
    assert stages.capacity is not None
    assert stages.capacity.crossingPitch == pytest.approx(150.0)
    assert stages.capacity.bottleneckClearance == pytest.approx(1.5)
    assert stages.global_ is not None
    assert stages.global_.internalBridges is True
    assert stages.assignment is not None
    assert stages.assignment.launcherTarget == 7
    assert stages.solver is not None
    assert (stages.solver.backend, stages.solver.timeLimit) == ("highs", pytest.approx(30.0))
    assert stages.solver.threads == 4
    assert stages.corridor is not None
    assert (stages.corridor.rounds, stages.corridor.maxRelaxation) == (6, 4)
    assert config.run is not None
    assert config.run.stopAfter == "assign"


def test_the_bridge_declarations_load() -> None:
    """The bridge pattern, the component pattern and the bridge rules are read in order."""
    config = parse_config(WITH_BRIDGES)

    assert config.ports is not None
    assert config.ports.patterns is not None
    assert config.ports.patterns.bridgePair == r"^Coupler\d+_\d+\.port[1-4]$"
    assert config.ports.patterns.component == r"^([^.]+)\.port\d+$"
    rules = config.ports.bridgePairs
    assert [(rule.first, rule.second) for rule in rules] == [
        (r"^(Coupler\d+_\d+)\.port1$", r"^(Coupler\d+_\d+)\.port2$"),
        (r"^(Coupler\d+_\d+)\.port3$", r"^(Coupler\d+_\d+)\.port4$"),
    ]


def test_the_grid_reads_the_detail_factor() -> None:
    """The detail grid divides each capacity cell; absent, it takes the schema default."""
    default = parse_config(WITH_GRID)
    assert default.grid is not None
    assert default.grid.detailFactor == 30
    config = parse_config(WITH_GRID + "detail_factor = 20\n")
    assert config.grid is not None
    assert config.grid.detailFactor == 20


@pytest.mark.parametrize(
    ("text", "message"),
    [
        (MANUAL + "\n[stages.capacity]\nsloppy = 1\n", "[stages.capacity] has the unknown key 'sloppy'"),
        (MANUAL + "\n[stages.geometry]\nrounds = 3\n", "[stages] has the unknown key 'geometry'"),
        (MANUAL + "\n[stages.global]\ninternal_bridges = 1\n", "internal_bridges must be true or false"),
        (MANUAL + "\n[stages.assignment]\nlauncher_target = -1\n", "between 0 and 4294967295"),
        (MANUAL + "\n[stages.solver]\nthreads = -2\n", "[stages.solver] threads must be between 0 and 4294967295"),
        (MANUAL + "\n[stages.solver]\nthreads = 2.5\n", "[stages.solver] threads must be an integer"),
        (
            MANUAL.replace("[ports.patterns]", "[ports]\nbridge_pairs = 3\n\n[ports.patterns]"),
            "[ports] bridge_pairs must be an array of tables",
        ),
        (
            WITH_BRIDGES.replace("second = '^(Coupler\\d+_\\d+)\\.port2$'\n", ""),
            "[ports.bridge_pairs[0]] lacks the required key 'second'",
        ),
        (
            MANUAL + '\n[run]\nstop_after = "outer"\n',
            "[run] stop_after must be empty or one of 'capacity', 'global', 'assign', 'corridor'",
        ),
        (MANUAL + "\n[run]\nstop = 1\n", "[run] has the unknown key 'stop'"),
    ],
    ids=[
        "unknown-stage-key",
        "unknown-stage",
        "flag-not-boolean",
        "negative-count",
        "negative-threads",
        "fractional-threads",
        "bridge-pairs-not-tables",
        "bridge-rule-without-second",
        "unknown-step",
        "unknown-run-key",
    ],
)
def test_stage_problems_are_named(text: str, message: str) -> None:
    """A problem of a stage, run or bridge section is reported with its section and key."""
    with pytest.raises(ConfigError, match=message.replace("[", r"\[").replace("]", r"\]")):
        parse_config(text)


def test_shipped_stage_sections_carry_only_what_differs_from_the_defaults() -> None:
    """Strict loading rejects a stage or run key written out at its default, and empty sections."""
    parse_config(WITH_STAGES, strict=True)
    assert shipped_config_problems({"stages": {"corridor": {"rounds": 12}, "capacity": {}}}) == [
        "[stages.capacity] is empty; remove it",
        "[stages.corridor] rounds is set to its default 12; remove it",
    ]
    assert shipped_config_problems({"stages": {"global": {"internal_bridges": False}}}) == [
        "[stages.global] internal_bridges is set to its default false; remove it",
    ]
    assert shipped_config_problems({"run": {"stop_after": ""}}) == [
        '[run] stop_after is set to its default ""; remove it',
    ]
    # A whole number in a key that holds a number is that number; a flag is not a number.
    assert shipped_config_problems({"stages": {"solver": {"time_limit": 0}}}) == [
        "[stages.solver] time_limit is set to its default 0.0; remove it",
    ]
    assert shipped_config_problems({"stages": {"assignment": {"launcher_target": False}}}) == []
    assert shipped_config_problems({"stages": {}, "run": {}}) == [
        "[stages] is empty; remove it",
        "[run] is empty; remove it",
    ]


def test_the_stage_and_bridge_sections_round_trip_through_the_core_bytes() -> None:
    """What write_config packs of the newer sections, read_config unpacks unchanged."""
    config = parse_config(WITH_BRIDGES + STAGES)
    back = read_config(write_config(config))

    assert back.ports is not None
    assert config.ports is not None
    assert [(rule.first, rule.second) for rule in back.ports.bridgePairs] == [
        (rule.first, rule.second) for rule in config.ports.bridgePairs
    ]
    assert back.stages is not None
    assert back.stages.assignment is not None
    assert back.stages.assignment.launcherTarget == 7
    assert back.stages.solver is not None
    assert back.stages.solver.backend == "highs"
    assert back.stages.solver.threads == 4
    assert back.run is not None
    assert back.run.stopAfter == "assign"


def test_load_config_names_the_file(tmp_path: Path) -> None:
    """A file is loaded by path, and a missing file is a ConfigError."""
    path = tmp_path / "config.toml"
    path.write_text(MANUAL, encoding="utf-8")
    assert load_config(path).chipInput == "routing_config.json"

    with pytest.raises(ConfigError, match="cannot read"):
        load_config(tmp_path / "absent.toml")


def test_a_file_that_is_not_utf8_is_named(tmp_path: Path) -> None:
    """A configuration in another encoding is a ConfigError that names the file."""
    path = tmp_path / "config.toml"
    path.write_bytes(("# Gr\u00f6\u00dfe\n" + MANUAL).encode("latin-1"))
    with pytest.raises(ConfigError, match=re.escape(f"cannot read {path}: it is not UTF-8")):
        load_config(path)


def test_the_fixture_follows_the_shipped_rules() -> None:
    """The fixture configuration loads under the rules for a shipped file."""
    load_config(FIXTURE / "config.toml", strict=True)


def _toml_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    return str(value)


def test_the_configuration_reference_lists_every_key_with_its_default() -> None:
    """docs/configuration.md has a row for every key the loader reads, with the default the loader applies."""
    reference = (Path(__file__).resolve().parents[3] / "docs" / "configuration.md").read_text(encoding="utf-8")
    rows: dict[tuple[str, str], str] = {}
    section = ""
    for line in reference.splitlines():
        if heading := re.fullmatch(r"## `\[+([a-z_.]+)\]+`", line):
            section = heading[1]
        elif row := re.fullmatch(r"\| `([a-z_]+)` +\| (\S+) +\|.*", line):
            rows[section, row[1]] = row[2]

    expected = {("design_rules", key): "—" for key in DESIGN_RULES}
    expected.update({("grid", key): f"`{_toml_value(value)}`" for key, value in GRID_DEFAULTS.items()})
    for name, defaults in STAGE_DEFAULTS.items():
        expected.update({(f"stages.{name}", key): f"`{_toml_value(value)}`" for key, value in defaults.items()})
    expected.update({("run", key): f"`{_toml_value(value)}`" for key, value in RUN_DEFAULTS.items()})
    assert {key: rows.get(key) for key in expected} == expected


def test_the_coupler_insertion_keys_of_the_final_stage_read_and_default() -> None:
    """The coupler insertion's knobs are keys of `[stages.final]` with the defaults of the schema."""
    config = parse_config(MANUAL)
    assert config.stages is not None
    assert config.stages.final is not None
    final = config.stages.final
    assert final.couplerBias == pytest.approx(0.98)
    assert final.couplerMaxShortfall == pytest.approx(0.30)
    assert final.couplerBoxMarginCells == 5
    assert final.couplerShiftCells == 20
    assert final.couplerLeadMarginCells == 5
    assert final.launcherLead == pytest.approx(10.0)
    assert not final.couplerOffsets
    assert not final.jogOptions
    assert not final.couplerOptionExtension
    assert final.edgeLauncherMarginCells == 6
    assert final.terminalSlotCells == 20
    assert final.chainEndEdgeWeight == pytest.approx(0.1)
    assert final.edgeBendFactor == pytest.approx(1.0)
    assert final.chainLengthWeight == 0
    assert final.capacityCheck is True
    assert final.capacityMargin == pytest.approx(1.06)
    assert final.globalSatTries == 200
    assert final.meanderStartMarginCells == -1
    assert final.meanderRelaxation is True
    assert final.feedlineMeanderRelaxation is True
    assert final.capacityOwnRunBonus is False
    assert final.couplerGridOptions == 0
    assert final.couplerGridOnly is False
    assert final.feedlineLatticeEdges == 0
    assert final.feedlineLatticeDirect is False
    assert final.localRipOwnChainOnly is False
    assert final.capacityCdt is True
    assert final.capacityCdtIncremental is True
    assert final.couplerEntryLength is True
    assert final.feedlineMeanderKeepout is True
    assert final.capacityCheckChains == ""
    assert final.chainDeadPrefixRefusals == 0
    assert final.capacityCdtCanonical is False
    assert final.couplerOptionFixes is False
    assert final.routerGridSnap == 0.0
    assert final.couplerLatticePitch == 0.0
    assert final.couplerLatticeOptions == 32
    assert final.couplerLatticeReach == 0
    assert final.couplerLatticeShortest is False
    assert final.capacityCdtWorld is True
    assert final.couplerLatticeFloor == 0.0
    assert final.couplerLatticeNear == 0.0
    assert final.couplerLatticeChecker is False
    assert final.couplerLatticeReserveStep == 0
    assert final.chainRuleSteps == 0
    assert final.capacityChamberFill == 0.0
    assert final.couplerMeanderSlack is False
    assert final.couplerMeanderSlackChains in ("", None, b"")
    assert final.meanderRipupAlternatives == 0
    assert final.wireLauncherGuard == 0
    assert final.wireLauncherTurnCells == 5
    assert final.chainSearch == "astar"

    config = parse_config(
        MANUAL
        + '\n[stages.final]\ncoupler_box_margin_cells = 20\ncoupler_offsets = "0,1,2,3,4,5,6,7"\n'
        + "chain_end_edge_weight = 1.0\ncapacity_check = false\nchain_rule_refusals = 1\n"
        + 'meander_relaxation = false\njog_options = "20, 40"\ncoupler_option_extension = "0, 10, -5"\n'
        + 'chain_search = "bidirectional"\nmeander_ripup_alternatives = 4\nwire_launcher_guard = 3\n'
    )
    assert config.stages is not None
    assert config.stages.final is not None
    assert config.stages.final.couplerBoxMarginCells == 20
    assert config.stages.final.couplerOffsets == "0,1,2,3,4,5,6,7"
    assert config.stages.final.chainEndEdgeWeight == pytest.approx(1.0)
    back = read_config(write_config(config))
    assert back.stages is not None
    assert back.stages.final is not None
    assert back.stages.final.couplerBoxMarginCells == 20
    assert back.stages.final.couplerOffsets == "0,1,2,3,4,5,6,7"
    assert back.stages.final.chainEndEdgeWeight == pytest.approx(1.0)
    assert back.stages.final.capacityCheck is False
    assert back.stages.final.chainRuleRefusals == 1
    assert back.stages.final.meanderRelaxation is False
    assert back.stages.final.meanderRipupAlternatives == 4
    assert back.stages.final.wireLauncherGuard == 3
    assert back.stages.final.jogOptions == "20, 40"
    assert back.stages.final.couplerOptionExtension == "0, 10, -5"
    assert back.stages.final.chainSearch == "bidirectional"


def test_chain_search_names_the_searches() -> None:
    """`[stages.final] chain_search` is astar or bidirectional, and names the value it refuses."""
    with pytest.raises(ConfigError, match="chain_search: 'dijkstra'"):
        parse_config(MANUAL + '\n[stages.final]\nchain_search = "dijkstra"\n')


@pytest.mark.parametrize("extensions", ["0,x", "0,10.5", "201", "-201", "0,--3"])
def test_coupler_option_extension_names_cells(extensions: str) -> None:
    """`[stages.final] coupler_option_extension` takes whole cells from -200 to 200, and names the entry it refuses."""
    with pytest.raises(ConfigError, match="coupler_option_extension"):
        parse_config(MANUAL + f'\n[stages.final]\ncoupler_option_extension = "{extensions}"\n')


@pytest.mark.parametrize("jogs", ["20,0", "20,x", "-20", "201", "20,4.5"])
def test_jog_options_names_jog_lengths_in_cells(jogs: str) -> None:
    """`[stages.final] jog_options` takes whole jog lengths in cells from 1 to 200, and names the entry it refuses."""
    with pytest.raises(ConfigError, match="jog_options"):
        parse_config(MANUAL + f'\n[stages.final]\njog_options = "{jogs}"\n')


def test_the_debug_section_reads_and_defaults() -> None:
    """`[debug]` names the pages a run writes; every page is off unless the file turns it on."""
    config = parse_config(MANUAL)
    assert config.debug is not None
    assert config.debug.html is False
    assert config.debug.preFeedlineRoutingDashboard is False
    assert config.debug.preFeedlineRefinementDashboard is False
    assert config.debug.feedlineInsertion is False
    assert config.debug.localSat is False
    assert config.debug.globalSat is False
    assert config.debug.feedlineConstrainedRoutingDashboard is False
    assert config.debug.feedlineConstrainedRefinementDashboard is False

    config = parse_config(MANUAL + "\n[debug]\nhtml = true\nlocal_sat = true\n")
    assert config.debug is not None
    assert config.debug.html is True
    assert config.debug.localSat is True
    assert config.debug.globalSat is False
    back = read_config(write_config(config))
    assert back.debug is not None
    assert back.debug.html is True
    assert back.debug.localSat is True

    with pytest.raises(ConfigError, match=r"\[debug\] has the unknown key .search_pictures."):
        parse_config(MANUAL + "\n[debug]\nsearch_pictures = true\n")
