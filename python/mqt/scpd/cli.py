# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The ``mqt-scpd`` command line, which is the supported product.

Every subcommand returns an exit code: 0 when it did what it was asked, 1 when the input has a
problem, a stage failed or a run ends with fails, 2 when the arguments are wrong, and 130 when it
was interrupted. What a command writes follows ``docs/terminal_output.md``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import pyscpd
from .artifacts import ArtifactError
from .chip import ChipError, decode_chip, load_chip
from .config import ConfigError, load_config, write_config
from .console import Console, size
from .doctor import run_doctor
from .export import HAS_KLAYOUT
from .inspection import InspectionError, artifact_to_json
from .planning import PLANNING_STAGES, PlanningError, PlanningGeometry, planning_geometry
from .plot import STAGES, PlotError, layout_svg
from .run import BlockReporter, RunDirectory, RunError, steps_until
from .solvers import register as register_external_solver
from .steps import STEPS


def _positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        msg = f"must be positive, not {value}"
        raise argparse.ArgumentTypeError(msg)
    return value


def _non_negative_float(text: str) -> float:
    value = float(text)
    if not value >= 0:  # also rejects nan
        msg = f"must not be negative, not {value}"
        raise argparse.ArgumentTypeError(msg)
    return value


def _subject(config_path: Path) -> str:
    """The name of the chip a configuration plans: its directory, or the file name.

    Returns:
        The name.
    """
    return config_path.parent.name if config_path.name == "config.toml" else config_path.stem


def command_doctor(args: argparse.Namespace) -> int:
    """Run the doctor and print its report.

    Returns:
        The exit code.
    """
    report = run_doctor(args.config, list_ports=args.ports, chip_path=args.chip)
    print(report.text())
    return 0 if report.ok else 1


def command_plan(args: argparse.Namespace) -> int:
    """Run the planning steps into a run directory.

    Returns:
        The exit code.
    """
    console = Console(verbosity=args.verbose, live=False if args.no_progress else None)
    # An installed and licensed gurobipy becomes the external solver of the process; without it
    # the linked-in HiGHS solves, and a run works either way.
    register_external_solver()

    directory = RunDirectory(args.output)
    if args.stage is not None:
        # A resumed step reads the configuration and the chip the run carries, so that it equals
        # the step of an uninterrupted run.
        config = directory.load()
        steps = [args.stage]
        subject = directory.path.name
    else:
        config = directory.prepare(args.config, args.chip)
        configured = config.run.stopAfter if config.run is not None else None
        steps = steps_until(args.stop_after if args.stop_after is not None else configured)
        subject = _subject(args.config)
    chip = directory.classified_chip(config)

    console.header("plan", subject, str(directory.path), [pyscpd.solver_info(write_config(config))])
    last: str | None = None
    for step in steps:
        with console.stage(step) as block:
            try:
                result = directory.run_stage(step, config, chip, BlockReporter(block), args.verbose)
            except KeyboardInterrupt:
                block.failure("interrupted")
                console.closing(stopped_after=last, failed=step)
                return 130
            # A stage that fails in any way ends the run with the reason in its block and the
            # closing line, never with a traceback: the core raises ValueError and RuntimeError,
            # but a solver of the user's own may raise anything.
            except Exception as error:  # ruff: ignore[blind-except]
                block.failure(str(error) or type(error).__name__)
                console.closing(stopped_after=last, failed=step)
                return 1
            block.artifact(result.path.name, result.size)
        last = step
    console.closing(stopped_after=last)
    return 1 if console.run.fails else 0


def _drawn(args: argparse.Namespace) -> tuple[bytes, str, PlanningGeometry | None]:
    """The chip a picture is drawn from, its title, and the planning stage drawn over it.

    The chip of a run directory is the copy the run carries; otherwise it is the chip the
    configuration names.

    Returns:
        The classified chip, the title of the picture, and the planning shapes or None.

    Raises:
        PlotError: If the stage arrives with a later phase.
        RunError: If the artifact of the stage is missing from the run.
    """
    if STAGES[args.stage] not in {"phase 1", "phase 3"}:
        msg = f"stage '{args.stage}' arrives with {STAGES[args.stage]}"
        raise PlotError(msg)
    if args.run_dir is None:
        return load_chip(load_config(args.config), args.config, args.chip), str(args.config), None
    directory = RunDirectory(args.run_dir)
    chip_bytes = directory.classified_chip(directory.load())
    title = f"{directory.path} ({args.stage})"
    if args.stage == "layout":
        return chip_bytes, title, None
    artifact = directory.artifact(args.stage)
    if not artifact.is_file():
        msg = f"{artifact} is missing; run `mqt-scpd plan` up to the {args.stage} step first"
        raise RunError(msg)
    # The global picture is drawn over the gates the circuit paid for, and the corridor picture
    # over the partitions its wires run through.
    plan = directory.artifact("capacity")
    capacity = plan.read_bytes() if args.stage in {"global", "corridor"} and plan.is_file() else None
    geometry = planning_geometry(artifact.read_bytes(), decode_chip(chip_bytes), args.stage, capacity)
    return chip_bytes, title, geometry


def command_plot(args: argparse.Namespace) -> int:
    """Render one stage as SVG.

    Returns:
        The exit code.
    """
    chip_bytes, title, planning = _drawn(args)
    svg = layout_svg(
        decode_chip(chip_bytes), width=args.width, tolerance=args.tolerance, title=title, planning=planning
    )
    args.output.write_text(svg, encoding="utf-8")
    Console().success(f"{args.output}  {size(len(svg.encode('utf-8')))}")
    return 0


def command_render(args: argparse.Namespace) -> int:
    """Write the chip, and a planning stage beside it, as GDSII or OASIS.

    Returns:
        The exit code.
    """
    if not HAS_KLAYOUT:
        Console(file=sys.stderr).failure("render needs KLayout; install it with 'mqt-scpd[klayout]'")
        return 1
    from .export import (  # ruff: ignore[import-outside-top-level]  # needs the optional dependency
        ExportError,
        write_layout,
    )

    chip_bytes, _, planning = _drawn(args)
    try:
        summary = write_layout(decode_chip(chip_bytes), args.output, planning=planning)
    except ExportError as error:
        Console(file=sys.stderr).failure(str(error))
        return 1
    figures = [summary.format, f"{summary.polygons} polygons", f"{summary.ports} ports"]
    if summary.planning:
        figures.append(f"{summary.planning} planning shapes")
    Console().success(f"{summary.path}  {size(summary.path.stat().st_size)} · " + " · ".join(figures))
    return 0


def command_list_algorithms(args: argparse.Namespace) -> int:
    """Print the implementations this build ships, one line per stage.

    Returns:
        The exit code.
    """
    del args
    for stage, names in pyscpd.algorithms():
        print(f"{stage:<18} {', '.join(names)}")
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
        Console().success(f"{args.output}  {size(len(text.encode('utf-8')))}")
    return 0


def _add_chip_option(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--chip",
        type=Path,
        help="the chip input to read instead of the one the configuration names",
    )


def _add_drawing_options(parser: argparse.ArgumentParser) -> None:
    """The options that plot and render share: where the chip comes from, and which stage."""
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("-c", "--config", type=Path, help="the config.toml of the chip")
    source.add_argument("--run-dir", type=Path, help="the run directory to draw the chip and a stage from")
    _add_chip_option(parser)
    parser.add_argument("--stage", choices=list(STAGES), default="layout", help="the stage to draw")


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
    _add_chip_option(doctor)
    doctor.set_defaults(run=command_doctor)

    plan = commands.add_parser("plan", help="run the planning steps into a run directory")
    start = plan.add_mutually_exclusive_group(required=True)
    start.add_argument("-c", "--config", type=Path, help="the config.toml to run from the first step")
    start.add_argument(
        "--stage",
        choices=list(STEPS),
        help="run this one step again, on the configuration and the chip the run carries",
    )
    plan.add_argument("-o", "--output", type=Path, required=True, help="the run directory")
    _add_chip_option(plan)
    plan.add_argument(
        "--stop-after",
        choices=list(STEPS),
        help="the last step to run; overrides [run] stop_after",
    )
    plan.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help="show more: -v the steps of a stage and its solves, -vv one line per item, -vvv the solver log",
    )
    plan.add_argument("--no-progress", action="store_true", help="never show the live progress line")
    plan.set_defaults(run=command_plan)

    plot = commands.add_parser("plot", help="draw a stage as SVG")
    _add_drawing_options(plot)
    plot.add_argument("-o", "--output", type=Path, required=True, help="the SVG file to write")
    plot.add_argument("--width", type=_positive_int, default=2000, help="the display width of the picture in pixels")
    plot.add_argument(
        "--tolerance",
        type=_non_negative_float,
        default=0.0,
        help="drop vertices within this distance (layout units) of the polygon edge; 0 keeps every vertex",
    )
    plot.set_defaults(run=command_plot)

    render = commands.add_parser("render", help="write the chip and a stage as GDSII or OASIS")
    _add_drawing_options(render)
    render.add_argument("-o", "--output", type=Path, required=True, help="the .gds or .oas file to write")
    render.set_defaults(run=command_render)

    algorithms = commands.add_parser("list-algorithms", help="list the implementations of every stage")
    algorithms.set_defaults(run=command_list_algorithms)

    inspect = commands.add_parser("inspect", help="print a stage artifact as JSON")
    inspect.add_argument("artifact", type=Path, help="the .fb artifact")
    inspect.add_argument("-o", "--output", type=Path, help="write the JSON to this file instead of stdout")
    inspect.set_defaults(run=command_inspect)
    return parser


def _check_combinations(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """Refuse the options that argparse cannot rule out on its own; it exits with code 2."""
    if args.command == "plan" and args.stage is not None and (args.chip is not None or args.stop_after is not None):
        parser.error("plan --stage runs one step on the copies the run carries; it takes no --chip or --stop-after")
    if args.command in {"plot", "render"}:
        if args.run_dir is not None and args.chip is not None:
            parser.error(f"{args.command} --run-dir draws the chip the run carries; it takes no --chip")
        if args.stage in PLANNING_STAGES and args.run_dir is None:
            parser.error(f"{args.command} --stage {args.stage} is drawn from a run; pass --run-dir")


def main(argv: list[str] | None = None) -> int:
    """Run the command line.

    Args:
        argv: The arguments, without the program name; ``None`` takes them from the process.

    Returns:
        The exit code.
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    _check_combinations(parser, args)
    try:
        return int(args.run(args))
    except KeyboardInterrupt:
        Console(file=sys.stderr).failure("interrupted")
        return 130
    # The errors of the inputs, and every error the core raises outside a stage, such as a backend
    # name it does not know.
    except (
        ConfigError,
        ChipError,
        PlotError,
        ArtifactError,
        InspectionError,
        PlanningError,
        RunError,
        OSError,
        ValueError,
        RuntimeError,
    ) as error:
        Console(file=sys.stderr).failure(str(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
