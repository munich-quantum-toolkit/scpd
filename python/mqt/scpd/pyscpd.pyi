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

def plan_capacity(chip: bytes, config: bytes, producer: str) -> bytes:
    """Run the Capacity stage. Returns 01-capacity.fb as bytes."""

def route_global(chip: bytes, capacity: bytes, config: bytes, producer: str) -> bytes:
    """Run the Global stage. Returns 02-global.fb as bytes."""

def assign(chip: bytes, capacity: bytes, global_: bytes, config: bytes, producer: str) -> bytes:
    """Run the Assignment stage. Returns 03-assign.fb as bytes."""

def route_corridor(
    chip: bytes, capacity: bytes, assignment: bytes, config: bytes, producer: str, progress: object | None = None
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
    progress: object | None = None,
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
    progress: object | None = None,
    debug: object | None = None,
    verbosity: int = 0,
) -> bytes:
    """Run the Final stage. Returns 06-final.fb as bytes. progress, when given, is called with one line per round while the stage runs, and with verbosity 1 with one line per wire and search as well; debug, when given, is called with the kind and the JSON text of every record a page of the [debug] section asks for."""

class CouplerSession:
    """The Final stage held after its coupler insertion, to move couplers by hand and look at the capacity graph again: a debugging aid."""

    def __init__(
        self,
        chip: bytes,
        global_: bytes,
        assignment: bytes,
        detail: bytes,
        config: bytes,
        progress: object | None = None,
        verbosity: int = 0,
    ) -> None:
        """Run the Final stage up to and with the coupler insertion and keep it. progress, when given, is called with every line the stage says, now and on every later call."""

    def couplers(self) -> str:
        """The couplers as they stand and every option of each, as JSON."""

    def set_option(self, coupler: int, option: int) -> str:
        """Put a coupler on another option and draw its two feedline edges again. Returns JSON with what became of each edge."""

    def graph(self) -> str:
        """The capacity graph of the chip as it stands, as the JSON of final-capacity-graph.json."""

    def insertion_check(self) -> str:
        """The capacity check of the coupler insertion on the chip as it stands, every chain standing. Returns JSON with verdict, lost and graph; under SCPD_SLOT_CHECK (on) the check by crossing slots gives them, adds over, and keeps the check before as previousVerdict and previousLost."""

    def route_feedlines(self) -> str:
        """Run the feedline routing on the chip as it stands. Returns JSON with the fails of every wire and every wire's way. The routing changes the state for good."""

    def set_debug(self, debug: object) -> None:
        """Record what the stage does from now on: debug is called with the kind and the JSON text of every record the [debug] section of the session's configuration asks for."""

def check_final(chip: bytes, global_: bytes, assignment: bytes, final: bytes, config: bytes) -> str:
    """Check a final routing against the design rules. Returns the text of drc.json."""

def algorithms() -> list[tuple[str, list[str]]]:
    """The implementations this build ships, one list per stage."""

def set_solver(solve: object | None) -> None:
    """Register the external solver of this process, or clear it with None. The callable receives the MPS text, the variable names in model order, a time limit and a relative gap, and returns (status, objective, values)."""

def slot_check(graph: str) -> str:
    """Check a capacity graph, the JSON of final-capacity-graph.json, by the crossing slots of its feedline edges. Returns JSON with verdict, lost, over and graph. Raises ValueError when the graph cannot be read."""

def validate_config(config: bytes) -> list[str]:
    """The problems of a configuration that can be seen without the chip, empty when there are none."""
