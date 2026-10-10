# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Internal bindings of the MQT SCPD core. The command-line interface is the supported product."""

def load_chip(chip_json: str, config: bytes) -> bytes:
    """Read the chip input, classify its ports from the configuration's patterns and check the configured port sequences against the chip. Returns the classified chip as bytes of the design schema. Raises ValueError naming every problem."""

def validate_config(config: bytes) -> list[str]:
    """The problems of a configuration that can be seen without the chip, empty when there are none."""

def plan_capacity(
    chip: bytes, config: bytes, producer: str, reporter: object | None = None, verbosity: int = 0
) -> bytes:
    """Run the Capacity stage on a classified chip. The reporter receives what the stage reports, through its methods line, entry, result and progress. Returns 01-capacity.fb as bytes."""

def route_global(
    chip: bytes, capacity: bytes, config: bytes, producer: str, reporter: object | None = None, verbosity: int = 0
) -> bytes:
    """Run the Global stage on a classified chip and its 01-capacity.fb. The reporter receives what the stage reports, as for plan_capacity. Returns 02-global.fb as bytes."""

def assign(
    chip: bytes,
    capacity: bytes,
    global_routing: bytes,
    config: bytes,
    producer: str,
    reporter: object | None = None,
    verbosity: int = 0,
) -> bytes:
    """Run the Assignment stage on a classified chip, its 01-capacity.fb and its 02-global.fb. The reporter receives what the stage reports, as for plan_capacity. Returns 03-assign.fb as bytes."""

def route_corridor(
    chip: bytes,
    capacity: bytes,
    assignment: bytes,
    config: bytes,
    producer: str,
    reporter: object | None = None,
    verbosity: int = 0,
) -> bytes:
    """Run the Corridor stage on a classified chip, its 01-capacity.fb and its 03-assign.fb. The reporter receives what the stage reports, as for plan_capacity. Returns 04-corridor.fb as bytes."""

def algorithms() -> list[tuple[str, list[str]]]:
    """The implementations this build ships, as (stage, names) pairs in pipeline order."""

def set_solver(solve: object | None) -> None:
    """Register the external solver of the process, or clear it with None. The callable receives the MPS text, the variable names in model order, a time limit and a relative gap, and returns (status, objective, values)."""

def solver_info(config: bytes) -> str:
    """The backend a solve with this configuration uses: HiGHS and its version, or the name of the external backend."""

def artifact_to_json(artifact: bytes) -> str:
    """Render a stage artifact as JSON. The field names, the enum names and the union tags come from the schema, so the JSON follows it without a second description of the model. Raises ValueError when the bytes are not a complete artifact of this schema version."""
