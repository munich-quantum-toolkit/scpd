# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The ``doctor`` command: check a configuration and its chip before a run.

The doctor loads the configuration under the rules for a shipped file, loads and classifies the chip,
prints the classification table so that a wrong pattern is visible in a second, and prints the outer
port ring the run would use, checked label by label against the chip.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from . import pyscpd
from .chip import ChipError, chip_input_path, decode_chip, load_chip, obstacles_of, ports_of, role_name
from .config import ConfigError, load_config, write_config

if TYPE_CHECKING:
    from pathlib import Path

    from .flatbuffers.config.Config import ConfigT
    from .flatbuffers.design.Chip import ChipT


@dataclass
class DoctorReport:
    """What the doctor found."""

    lines: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """Whether the configuration and the chip pass every check."""
        return not self.problems

    def say(self, *lines: str) -> None:
        """Append report lines."""
        self.lines.extend(lines)

    def fail(self, problem: str) -> None:
        """Record a problem, which also appears in the report."""
        self.problems.append(problem)
        self.lines.append(f"PROBLEM: {problem}")

    def text(self) -> str:
        """The whole report.

        Returns:
            The report, one line per entry, ending with the verdict.
        """
        verdict = "doctor: OK" if self.ok else f"doctor: {len(self.problems)} problem(s)"
        return "\n".join([*self.lines, verdict])


def classification_table(chip: ChipT, config: ConfigT) -> list[str]:
    """The roles of the ports as a table: role, pattern, count and the first labels.

    Returns:
        The lines of the table.
    """
    patterns = config.ports.patterns if config.ports is not None else None
    pattern_of = {
        "launcher": patterns.launcher if patterns else "",
        "resonator": patterns.resonator if patterns else "",
        "conventional": patterns.conventional if patterns else "",
        "mating": (patterns.mating or "") if patterns else "",
    }
    counts: Counter[str] = Counter()
    examples: dict[str, list[str]] = {}
    for port in ports_of(chip):
        name = role_name(port.role)
        counts[name] += 1
        examples.setdefault(name, [])
        if len(examples[name]) < 3:
            examples[name].append(port.label or "")
    lines = [f"{'role':<13}{'ports':>6}  pattern / first labels"]
    for name in ("launcher", "resonator", "conventional", "mating"):
        if name not in counts and not pattern_of[name]:
            continue
        lines.append(f"{name:<13}{counts.get(name, 0):>6}  {pattern_of[name]}")
        if examples.get(name):
            lines.append(f"{'':<19}  {', '.join(examples[name])}")
    for name in counts:
        if name not in pattern_of:
            lines.append(f"{name:<13}{counts[name]:>6}")
    return lines


def ring_summary(config: ConfigT) -> list[str]:
    """The configured outer port ring: the size of each sequence and the sequences themselves.

    Returns:
        The lines.
    """
    sequences = config.ports.sequences if config.ports is not None else None
    if sequences is None:
        return ["outer port ring: no sequences"]
    all_outer = [label or "" for label in sequences.allOuter or []]
    fixed_outer = [label or "" for label in sequences.fixedOuter or []]
    return [
        "outer port ring",
        f"all_outer: {len(all_outer)} ports, entering at {all_outer[0] if all_outer else '-'}",
        "  " + ", ".join(all_outer),
        f"fixed_outer: {len(fixed_outer)} ports" + ("" if fixed_outer else "; the configuration pins no port"),
        *(["  " + ", ".join(fixed_outer)] if fixed_outer else []),
    ]


def run_doctor(config_path: Path, *, list_ports: bool = False) -> DoctorReport:
    """Check a configuration and the chip it names.

    Args:
        config_path: The ``config.toml`` to check.
        list_ports: Also list every port with its role.

    Returns:
        The report.
    """
    report = DoctorReport()
    report.say(f"configuration: {config_path}")
    try:
        config = load_config(config_path, strict=True)
    except ConfigError as error:
        report.fail(str(error))
        return report
    report.say(f"chip input: {chip_input_path(config, config_path)}")

    config_bytes = write_config(config)
    for problem in pyscpd.validate_config(config_bytes):
        report.fail(problem)
    if not report.ok:
        return report

    try:
        chip_bytes = load_chip(config, config_path)
    except ChipError as error:
        report.fail(str(error))
        return report
    chip = decode_chip(chip_bytes)
    report.say("", f"obstacles: {len(obstacles_of(chip))}", f"ports: {len(ports_of(chip))}", "")
    report.say(*classification_table(chip, config))
    if list_ports:
        report.say("", *(f"  {port.label or '':<24} {role_name(port.role)}" for port in ports_of(chip)))
    report.say("", *ring_summary(config))
    return report
