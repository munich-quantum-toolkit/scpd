# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Internal bindings of the MQT SCPD core. The command-line interface is the supported product."""

from collections.abc import Callable, Sequence

def load_chip(chip_json: str, config: bytes) -> bytes:
    """Read the chip input, classify its ports from the configuration's patterns and check the configured port sequences against the chip. Returns the classified chip as bytes of the design schema. Raises ValueError naming every problem."""

def validate_config(config: bytes) -> list[str]:
    """The problems of a configuration that can be seen without the chip, empty when there are none."""

def plan_problems(chip: bytes, config: bytes, steps: Sequence[str]) -> list[str]:
    """What would stop a run of the given steps on a classified chip, before any stage runs: an unknown implementation name, a detail grid above the cell limit, a ring that the global step refuses, a missing launcher_target or one whose sum with feedline_terminations is odd, launchers listed in the direction of the port ring, fewer than three launchers off one line on a chip with resonators, or ring counts that the assignment refuses. Empty when there is nothing."""

def solving_steps(chip: bytes, config: bytes, steps: Sequence[str]) -> list[str]:
    """The steps of a run that solve a mixed-integer program, and so need the solver backend: the assignment, and the global step where the chip has an inner circuit. The list keeps a global step whose port ring the core refuses, since that step stops the run anyway."""

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

def route_detail(
    chip: bytes,
    capacity: bytes,
    global_: bytes,
    assignment: bytes,
    corridor: bytes,
    config: bytes,
    producer: str,
    reporter: object | None = None,
    verbosity: int = 0,
) -> bytes:
    """Run the Detail stage. Returns 05-detail.fb as bytes. The stage reports through reporter as plan_capacity."""

def route_final(
    chip: bytes,
    capacity: bytes,
    global_: bytes,
    assignment: bytes,
    detail: bytes,
    config: bytes,
    producer: str,
    reporter: object | None = None,
    debug: object | None = None,
    verbosity: int = 0,
) -> bytes:
    """Run the Final stage. Returns 06-final.fb as bytes. The stage reports through reporter as plan_capacity; debug, when given, is called with the kind and the JSON text of every record a page of the [debug] section asks for."""

class CouplerSession:
    """The Final stage held after its coupler insertion, to move couplers by hand and look at the capacity graph again: a debugging aid."""

    def __init__(
        self,
        chip: bytes,
        global_: bytes,
        assignment: bytes,
        detail: bytes,
        config: bytes,
        reporter: object | None = None,
        verbosity: int = 0,
        debug: object | None = None,
    ) -> None:
        """Run the Final stage up to and with the coupler insertion and keep it. The stage reports through reporter as plan_capacity, now and on every later call. debug, when given, is called with the kind and the JSON text of every record the [debug] section of config asks for, from this run, the global SAT search included, and from every later call."""

    def couplers(self) -> str:
        """The couplers as they stand and every option of each, as JSON."""

    def set_option(self, coupler: int, option: int) -> str:
        """Put a coupler on another option and draw its two feedline edges again. Returns JSON with what became of each edge."""

    def graph(self) -> str:
        """The capacity graph of the chip as it stands, as the JSON of final-capacity-graph.json."""

    def insertion_check(self) -> str:
        """The capacity check of the coupler insertion on the chip as it stands, every chain standing. Returns JSON with verdict, lost and graph; under SCPD_SLOT_CHECK (on) the check by crossing slots gives them, adds over, and keeps the check before as previousVerdict and previousLost."""

    def local_check(self, chain: int, options: Sequence[int]) -> str:
        """Run the two local checks of the global SAT search on a chain with its couplers on options, one per waypoint (0 at a fixed waypoint), then put the couplers back. Returns JSON with chain, options, undrawn and, for the checks standing and alone, the verdict, the lost wires and the graph judged."""

    def step_check(self, chain: int, step: int, options: Sequence[int]) -> str:
        """Rebuild the step check of the chain search at edge step of a chain with its couplers on options, one per waypoint (0 at a fixed waypoint): the chain alone, its couplers up to the far end of edge step and its edges 0 to step, drawn as set_option draws them; then put the couplers back. Returns JSON with chain, step, edge, options, verdict, lost, closed, undrawn and the graph judged."""

    def chains(self) -> str:
        """Every chain as it stands, as JSON: per chain the coupler of every waypoint (null at a fixed waypoint), the option every waypoint stands on and its feedline edges by name."""

    def compare_locally(self, runs: int, span: int) -> str:
        """Route the feedlines, then compare the slot check of every feedline alone with a rip-up and re-route of its wires alone, over up to runs runs of options of every segment of span edges of every chain. Returns JSON with the four classes, the runs judged and what the chip as it began came to; the routing changes the state for good, the comparison leaves it as found."""

    def route_feedlines(self) -> str:
        """Run the feedline routing on the chip as it stands. Returns JSON with the fails of every wire and every wire's way. The routing changes the state for good."""

    def set_debug(self, debug: object) -> None:
        """Record what the stage does from now on: debug is called with the kind and the JSON text of every record the [debug] section of the session's configuration asks for."""

def check_final(chip: bytes, global_: bytes, assignment: bytes, final: bytes, config: bytes) -> str:
    """Check a final routing against the design rules. Returns the text of drc.json."""

def slot_check(graph: str) -> str:
    """Check a capacity graph, the JSON of final-capacity-graph.json, by the crossing slots of its feedline edges. Returns JSON with verdict, lost, over and graph. Raises ValueError when the graph cannot be read."""

def algorithms() -> list[tuple[str, list[str]]]:
    """The implementations this build ships, as (stage, names) pairs in pipeline order."""

def set_solver(solve: Callable | None) -> None:
    """Register the external solver of the process, or clear it with None. The callable receives the MPS text, the variable names in model order, a time limit, a relative gap and a thread count, and returns (status, objective, values, bound, gap, nodes): bound and gap are NaN and nodes is None where the solver reports none. Raises TypeError when solve is neither callable nor None. The module clears the solver at interpreter exit."""

def solver_info(config: bytes) -> str:
    """The backend a solve with this configuration uses: HiGHS and its version, or the name of the external backend."""

def artifact_to_json(artifact: bytes) -> str:
    """Render a stage artifact as JSON. The field names, the enum names and the union tags come from the schema, so the JSON follows it without a second description of the model. Raises ValueError when the bytes are not a complete artifact of this schema version."""
