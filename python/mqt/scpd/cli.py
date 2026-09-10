# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The ``mqt-scpd`` command line, which is the supported product.

Every subcommand returns an exit code: 0 when it succeeded, 1 when the input has a problem, and 2
when the arguments are wrong.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import TYPE_CHECKING

from .artifacts import ArtifactError
from .chip import ChipError, classify_chip, decode_chip, load_chip
from .config import ConfigError, load_config, write_config
from .doctor import run_doctor
from .export.klayout import ExportError, write_layout
from .inspection import InspectionError, artifact_to_json
from .planning import FINAL_PHASES, PLANNING_STAGES, PlanningError, planning_geometry
from .plot import STAGES, PlotError, layout_svg
from .run import IMPLEMENTED, RunDirectory, RunError
from .solvers import register as register_external_solver

if TYPE_CHECKING:
    from .flatbuffers.config.Config import ConfigT


def _load(config_path: Path) -> tuple[bytes, ConfigT, Path]:
    """Load the configuration and its chip, for the commands that draw the chip.

    Returns:
        The classified chip bytes, the configuration and its path.
    """
    config = load_config(config_path)
    return load_chip(config, config_path), config, config_path


def command_doctor(args: argparse.Namespace) -> int:
    """Run the doctor and print its report.

    Returns:
        The exit code.
    """
    report = run_doctor(args.config, list_ports=args.ports)
    print(report.text())
    return 0 if report.ok else 1


def _planning_for(args: argparse.Namespace, chip_bytes: bytes, config: ConfigT):  # noqa: ANN202
    """What a planning stage produced, or None for the plain layout view.

    Returns:
        The geometry of the stage, or None when the stage is ``layout``.

    Raises:
        RunError: If the stage is asked for without a run directory, or its artifact is missing.
    """
    if args.stage == "layout":
        return None
    if args.stage not in PLANNING_STAGES:
        msg = f"stage '{args.stage}' arrives with {STAGES[args.stage]}"
        raise RunError(msg)
    if args.run_dir is None:
        msg = f"stage '{args.stage}' is read from a run directory; pass --run-dir"
        raise RunError(msg)
    directory = RunDirectory(args.run_dir)
    artifact = directory.artifact(args.stage)
    if not artifact.is_file():
        msg = f"{artifact} is missing; run `mqt-scpd plan -c ... -o {args.run_dir}` first"
        raise RunError(msg)
    # The global picture is drawn over the gates the circuit had to pay for and the corridor
    # picture over the partitions its wires run through, so the capacity artifact of the same
    # run is read beside them when the run still carries one.
    plan = directory.artifact("capacity") if args.stage in {"global", "corridor", "detail"} else None
    capacity = plan.read_bytes() if plan is not None and plan.is_file() else None
    # The wires are drawn with the clearance the design rules demand around them, so that two
    # wires closer than the rule allows are two bands that overlap.
    spacing = config.rules.minWireSpacing if config.rules is not None else 0.0
    return planning_geometry(
        artifact.read_bytes(),
        decode_chip(chip_bytes),
        args.stage,
        capacity,
        clearance=spacing,
        phase=getattr(args, "phase", None),
    )


def command_plot(args: argparse.Namespace) -> int:
    """Render one stage of a run as SVG.

    Returns:
        The exit code.
    """
    chip_bytes, config, config_path = _load(args.config)
    planning = _planning_for(args, chip_bytes, config)
    svg = layout_svg(
        decode_chip(chip_bytes),
        width=args.width,
        tolerance=args.tolerance,
        title=f"{config_path} ({args.stage})",
        planning=planning,
    )
    args.output.write_text(svg, encoding="utf-8")
    print(f"wrote {args.output} ({len(svg.encode('utf-8')) / 1e6:.2f} MB)")
    return 0


def command_render(args: argparse.Namespace) -> int:
    """Write the chip, and optionally one planning stage, as GDSII or OASIS.

    Returns:
        The exit code.
    """
    chip_bytes, config, _ = _load(args.config)
    planning = _planning_for(args, chip_bytes, config)
    summary = write_layout(decode_chip(chip_bytes), args.output, planning=planning)
    extra = f", {summary.planning} planning shapes" if summary.planning else ""
    print(f"wrote {summary.path} as {summary.format}: {summary.polygons} polygons, {summary.ports} ports{extra}")
    return 0


def command_plan(args: argparse.Namespace) -> int:
    """Run the planning stages into a run directory.

    Returns:
        The exit code.
    """
    # An installed and licensed gurobipy becomes the external backend of this process; without
    # one the linked-in HiGHS is used, and a run works either way.
    register_external_solver()

    directory = RunDirectory(args.output)
    # Running one stage resumes a run, so the inputs are re-copied only when the whole set runs.
    config = (
        directory.load() if args.stage is not None and directory.config.is_file() else directory.prepare(args.config)
    )
    # With --verbose a stage that reports its progress prints one line at a time while it runs.
    # The Final stage is minutes of work on the largest chip, and what it is doing in that time is
    # only useful live.
    def say(line: str) -> None:
        print(line, flush=True)

    for stage in list(IMPLEMENTED) if args.stage is None else [args.stage]:
        result = directory.run_stage(stage, config, say if args.verbose else None)
        print(f"{result.stage:9s} -> {result.path.name} ({result.size} bytes)")
    return 0


def command_drc(args: argparse.Namespace) -> int:
    """Check a run against the design rules without routing it again.

    Returns:
        The exit code: nonzero when an active rule found something.
    """
    from . import pyscpd  # noqa: PLC0415
    from .drc import summarize  # noqa: PLC0415

    directory = RunDirectory(args.run_dir)
    config = directory.load()
    chip = classify_chip(directory.chip.read_text(encoding="utf-8"), config, str(directory.chip))
    text = pyscpd.check_final(
        chip,
        directory.artifact("global").read_bytes(),
        directory.artifact("assign").read_bytes(),
        directory.artifact("final").read_bytes(),
        write_config(config),
    )
    directory.drc.write_text(text, encoding="utf-8")
    active, advisory = summarize(text)
    print(f"wrote {directory.drc}: {active} active findings, {advisory} advisory")
    return 1 if active else 0


def command_list_algorithms(args: argparse.Namespace) -> int:
    """Print the implementations this build ships.

    Returns:
        The exit code.
    """
    del args
    from . import pyscpd  # noqa: PLC0415

    for stage, names in pyscpd.algorithms():
        print(f"{stage + ':':18s}{', '.join(names)}")
    return 0


def command_inspect(args: argparse.Namespace) -> int:
    """Print a stage artifact as JSON.

    Returns:
        The exit code.
    """
    text = artifact_to_json(args.artifact.read_bytes())
    if args.output is None:
        sys.stdout.write(text)
    else:
        args.output.write_text(text, encoding="utf-8")
        print(f"wrote {args.output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """The argument parser of the command line.

    Returns:
        The parser, with one subparser per command.
    """
    parser = argparse.ArgumentParser(prog="mqt-scpd", description="Physical design for superconducting quantum chips.")
    commands = parser.add_subparsers(dest="command", required=True)

    doctor = commands.add_parser("doctor", help="check a configuration and its chip before a run")
    doctor.add_argument("-c", "--config", type=Path, required=True, help="the config.toml to check")
    doctor.add_argument("--ports", action="store_true", help="list every port with its role")
    doctor.set_defaults(run=command_doctor)

    plot = commands.add_parser("plot", help="render a stage as SVG")
    plot.add_argument("-c", "--config", type=Path, required=True, help="the config.toml of the chip")
    plot.add_argument("--stage", choices=list(STAGES), default="layout", help="the stage to render")
    plot.add_argument(
        "--phase",
        choices=list(FINAL_PHASES),
        help="which phase of the final stage to draw; the end state by default",
    )
    plot.add_argument("--run-dir", type=Path, help="the run directory a stage other than layout is read from")
    plot.add_argument("-o", "--output", type=Path, required=True, help="the SVG file to write")
    plot.add_argument("--width", type=int, default=2000, help="the display width of the picture in pixels")
    plot.add_argument(
        "--tolerance",
        type=float,
        default=0.0,
        help="drop vertices within this distance (layout units) of the polygon edge; 0 keeps every vertex",
    )
    plot.set_defaults(run=command_plot)

    render = commands.add_parser("render", help="write the chip, and optionally a stage, as GDSII or OASIS")
    render.add_argument("-c", "--config", type=Path, required=True, help="the config.toml of the chip")
    render.add_argument("--stage", choices=list(STAGES), default="layout", help="the stage to write beside the artwork")
    render.add_argument(
        "--phase",
        choices=list(FINAL_PHASES),
        help="which phase of the final stage to draw; the end state by default",
    )
    render.add_argument("--run-dir", type=Path, help="the run directory a stage other than layout is read from")
    render.add_argument("-o", "--output", type=Path, required=True, help="the .gds or .oas file to write")
    render.set_defaults(run=command_render)

    plan = commands.add_parser("plan", help="run the planning stages into a run directory")
    plan.add_argument("-c", "--config", type=Path, required=True, help="the config.toml to run")
    plan.add_argument("-o", "--output", type=Path, required=True, help="the run directory to write")
    plan.add_argument("--stage", choices=list(IMPLEMENTED), help="run only this stage, resuming the run")
    plan.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="print what a stage is doing while it runs, one line per round",
    )
    plan.set_defaults(run=command_plan)

    drc = commands.add_parser("drc", help="check a run against the design rules")
    drc.add_argument("run_dir", type=Path, help="the run directory to check")
    drc.set_defaults(run=command_drc)

    algorithms = commands.add_parser("list-algorithms", help="list the implementations of every stage")
    algorithms.set_defaults(run=command_list_algorithms)

    inspect = commands.add_parser("inspect", help="print a stage artifact as JSON")
    inspect.add_argument("artifact", type=Path, help="the .fb artifact")
    inspect.add_argument("-o", "--output", type=Path, help="write the JSON to this file instead of stdout")
    inspect.set_defaults(run=command_inspect)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the command line.

    Args:
        argv: The arguments, without the program name; ``None`` takes them from the process.

    Returns:
        The exit code.
    """
    args = build_parser().parse_args(argv)
    try:
        return int(args.run(args))
    except (
        ConfigError,
        ChipError,
        PlotError,
        ExportError,
        ArtifactError,
        InspectionError,
        PlanningError,
        RunError,
        OSError,
    ) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
