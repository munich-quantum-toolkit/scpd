# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Internal bindings of the MQT SCPD core. The command-line interface is the supported product."""

from collections.abc import Callable

def load_chip(chip_json: str, config: bytes) -> bytes:
    """Read the chip input, classify its ports from the configuration's patterns and check the configured port sequences against the chip. Returns the classified chip as bytes of the design schema. Raises ValueError naming every problem."""

def plan_capacity(chip: bytes, config: bytes, producer: str) -> bytes:
    """Run the Capacity stage. Returns 01-capacity.fb as bytes."""

def route_global(chip: bytes, capacity: bytes, config: bytes, producer: str) -> bytes:
    """Run the Global stage. Returns 02-global.fb as bytes."""

def assign(chip: bytes, capacity: bytes, global_: bytes, config: bytes, producer: str) -> bytes:
    """Run the Assignment stage. Returns 03-assign.fb as bytes."""

def route_corridor(
    chip: bytes,
    capacity: bytes,
    assignment: bytes,
    config: bytes,
    producer: str,
    progress: Callable[[str], None] | None = None,
) -> bytes:
    """Run the Corridor stage. Returns 04-corridor.fb as bytes. progress, when given, is called with one line per round while the stage runs."""

def route_detail(
    chip: bytes,
    capacity: bytes,
    global_: bytes,
    assignment: bytes,
    corridor: bytes,
    config: bytes,
    producer: str,
    progress: Callable[[str], None] | None = None,
) -> bytes:
    """Run the Detail stage. Returns 05-detail.fb as bytes. progress, when given, is called with one line per pass while the stage runs."""

def route_final(
    chip: bytes,
    capacity: bytes,
    global_: bytes,
    assignment: bytes,
    detail: bytes,
    config: bytes,
    producer: str,
    progress: Callable[[str], None] | None = None,
) -> bytes:
    """Run the Final stage. Returns 06-final.fb as bytes. progress, when given, is called with one line per round while the stage runs."""

def check_final(chip: bytes, global_: bytes, assignment: bytes, final: bytes, config: bytes) -> str:
    """Check a final routing against the design rules. Returns the text of drc.json."""

def algorithms() -> list[tuple[str, list[str]]]:
    """The implementations this build ships, one list per stage."""

def set_solver(
    solve: Callable[[str, list[str], float, float], tuple[str, float, list[float]]] | None,
) -> None:
    """Register the external solver of this process, or clear it with None. The callable receives the MPS text, the variable names in model order, a time limit and a relative gap, and returns (status, objective, values)."""

def validate_config(config: bytes) -> list[str]:
    """The problems of a configuration that can be seen without the chip, empty when there are none."""
