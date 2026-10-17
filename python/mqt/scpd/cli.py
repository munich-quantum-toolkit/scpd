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
import contextlib
import os
import sys
from pathlib import Path
from typing import TYPE_CHECKING

from . import pyscpd
from .artifacts import ArtifactError
from .chip import ChipError, decode_chip, load_chip
from .config import ConfigError, load_config, removed_keys_in, write_config
from .console import Console, size, write_out
from .doctor import run_doctor, solver_problem
from .export import HAS_KLAYOUT
from .inspection import InspectionError, artifact_to_json
from .planning import PLANNING_STAGES, PlanningError, PlanningGeometry, planning_geometry
from .plot import STAGES, PlotError, layout_svg
from .run import STAGE_FILES, BlockReporter, RunDirectory, RunError, input_digests, steps_until
from .solvers import register as register_external_solver
from .steps import STEPS

if TYPE_CHECKING:
    from .console import StageBlock
    from .flatbuffers.config.Config import ConfigT


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
    return config_path.resolve().parent.name if config_path.name == "config.toml" else config_path.stem


def _validated(config: ConfigT, config_path: Path) -> ConfigT:
    """Check what the core refuses in a configuration before the chip is loaded, as the doctor does.

    Args:
        config: The loaded configuration.
        config_path: The configuration file, which a problem names.

    Returns:
        The configuration.

    Raises:
        ConfigError: If the core refuses a value of the configuration.
    """
    if problems := pyscpd.validate_config(write_config(config)):
        msg = f"{config_path}: " + "; ".join(problems)
        raise ConfigError(msg)
    return config


def command_doctor(args: argparse.Namespace) -> int:
    """Run the doctor and print its report.

    Returns:
        The exit code.
    """
    report = run_doctor(args.config, list_ports=args.ports, chip_path=args.chip)
    write_out(report.text() + "\n")
    return 0 if report.ok else 1


def command_plan(args: argparse.Namespace) -> int:
    """Run the planning steps into a run directory.

    Returns:
        The exit code.

    Raises:
        ConfigError: If the core refuses a value of the configuration, or if the configuration
            and the chip would stop one of the steps, as ``pyscpd.plan_problems`` and
            ``solver_problem`` report it. The run directory stays as it was.
        BrokenPipeError: If the reader of the output went away, also while a stage wrote.
    """
    console = Console(verbosity=args.verbose, live=False if args.no_progress else None)
    # The core reaches gurobipy only for a configuration that names the gurobi backend; every other
    # configuration solves with the linked-in HiGHS.
    register_external_solver()

    directory = RunDirectory(args.output)
    if args.stage is not None:
        # A resumed step reads the configuration and the chip the run carries, so that it equals
        # the step of an uninterrupted run.
        config = _validated(directory.load(), directory.config)
        steps = [args.stage]
        subject = directory.path.resolve().name
        chip = directory.classified_chip(config)
    else:
        config, chip_input = directory.inputs(args.config, args.chip)
        _validated(config, args.config)
        configured = config.run.stopAfter if config.run is not None else None
        steps = steps_until(args.stop_after if args.stop_after is not None else configured)
        subject = _subject(args.config)
        chip = load_chip(config, args.config, chip_input)
    # Every check runs before a fresh run deletes the artifacts of the run it replaces.
    config_bytes = write_config(config)
    problems = pyscpd.plan_problems(chip, config_bytes, steps)
    if (problem := solver_problem(chip, config_bytes, steps)) is not None:
        problems.append(problem)
    if problems:
        raise ConfigError("; ".join(problems))
    solver = pyscpd.solver_info(config_bytes)
    if args.stage is None:
        directory.prepare(args.config, args.chip)

    console.header("plan", subject, str(directory.path), [solver])
    for warning in removed_keys_in(args.config if args.stage is None else directory.config):
        console.warning(warning)
    last: str | None = None
    for step in steps:
        with console.stage(step) as block:
            try:
                result = directory.run_stage(step, config, chip, BlockReporter(block), args.verbose)
            except KeyboardInterrupt:
                block.failure("interrupted")
                console.closing(stopped_after=last, failed=step)
                return 130
            # A reader that went away while the stage wrote is no failure of the stage. main()
            # handles it as for every other command. Any other broken pipe fails the stage.
            except BrokenPipeError as error:
                if console.reader_gone:
                    raise
                return _stage_failed(console, block, error, last, step)
            # A stage that fails in any way ends the run with the reason in its block and the
            # closing line, never with a traceback: the core raises ValueError and RuntimeError,
            # but a solver of the user's own may raise anything.
            except Exception as error:  # ruff: ignore[blind-except]
                return _stage_failed(console, block, error, last, step)
            block.artifact(result.path.name, result.size)
        last = step
    console.closing(stopped_after=last)
    return 1 if console.run.fails else 0


def _stage_failed(console: Console, block: StageBlock, error: Exception, last: str | None, step: str) -> int:
    """End the run at a stage that failed, with the reason in its block and the closing line.

    Returns:
        The exit code.
    """
    block.failure(str(error) or type(error).__name__)
    console.closing(stopped_after=last, failed=step)
    return 1


def _drawn(args: argparse.Namespace) -> tuple[bytes, str, PlanningGeometry | None]:
    """The chip a picture is drawn from, its name, and the planning stage drawn over it.

    The chip of a run directory is the copy the run carries; otherwise it is the chip the
    configuration names.

    Returns:
        The classified chip, the name of the chip for the caption, and the planning shapes or None.

    Raises:
        PlotError: If the stage arrives with a later phase.
        RunError: If the artifact of the stage is missing from the run, or if it or an earlier
            artifact was made from inputs the run no longer holds.
    """
    if STAGES[args.stage] not in {"phase 1", "phase 3"}:
        msg = f"stage '{args.stage}' arrives with {STAGES[args.stage]}"
        raise PlotError(msg)
    if args.run_dir is None:
        return load_chip(load_config(args.config), args.config, args.chip), _subject(args.config), None
    directory = RunDirectory(args.run_dir)
    config = directory.load()
    chip_bytes = directory.classified_chip(config)
    title = directory.path.resolve().name
    if args.stage == "layout":
        return chip_bytes, title, None
    artifact = directory.artifact(args.stage)
    if not artifact.is_file():
        msg = f"{artifact} is missing; run `mqt-scpd plan` up to the {args.stage} step first"
        raise RunError(msg)
    directory.check_current(args.stage, input_digests(config, chip_bytes))
    # The global picture is drawn over the cells the grid blocked, and the assign and corridor
    # pictures over the partitions their chords and wires run through, with the inner circuit of the
    # global step. The corridor picture also shows the feedlines its resonator wires start at.
    plan = directory.artifact("capacity")
    capacity = plan.read_bytes() if args.stage in {"global", "assign", "corridor"} and plan.is_file() else None
    chosen = directory.artifact("assign")
    assignment = chosen.read_bytes() if args.stage == "corridor" and chosen.is_file() else None
    solved = directory.artifact("global")
    routing = solved.read_bytes() if args.stage in {"assign", "corridor"} and solved.is_file() else None
    geometry = planning_geometry(
        artifact.read_bytes(), decode_chip(chip_bytes), args.stage, capacity, assignment, routing
    )
    return chip_bytes, title, geometry


def command_plot(args: argparse.Namespace) -> int:
    """Render one stage as SVG.

    Returns:
        The exit code.
    """
    chip_bytes, title, planning = _drawn(args)
    svg = layout_svg(
        decode_chip(chip_bytes),
        width=args.width,
        tolerance=args.tolerance,
        title=title,
        stage=args.stage,
        planning=planning,
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
    write_out("".join(f"{stage:<18} {', '.join(names)}\n" for stage, names in pyscpd.algorithms()))
    return 0


def _stale_reason(artifact: Path) -> str | None:
    """Why an artifact of a run directory no longer matches the inputs of the run.

    A step that failed keeps its old artifact without a record, and an input of the run can change
    after a step wrote its artifact. An artifact outside a run directory has no record to check.

    Returns:
        The reason, or None when the artifact is current or lies outside a run directory.
    """
    stage = next((step for step, name in STAGE_FILES.items() if name == artifact.name), None)
    directory = RunDirectory(artifact.parent)
    if stage is None or not directory.config.is_file() or not directory.chip.is_file():
        return None
    try:
        config = directory.load()
        directory.check_current(stage, input_digests(config, directory.classified_chip(config)))
    except (RunError, ConfigError, ChipError) as error:
        return str(error)
    return None


def command_inspect(args: argparse.Namespace) -> int:
    """Print a stage artifact as JSON.

    An artifact of a run directory that no longer matches the inputs of the run still prints, after a
    warning on stderr.

    Returns:
        The exit code.
    """
    if (reason := _stale_reason(args.artifact)) is not None:
        Console(file=sys.stderr).warning(reason)
    text = artifact_to_json(args.artifact.read_bytes())
    if args.output is None:
        write_out(text)
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
        help="show more: -v the steps of a stage and its solves, -vv one line per item, -vvv the log of HiGHS",
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
    except BrokenPipeError:
        # The reader of the output went away, as `head` does once it has its lines. That is no
        # error to report. Python flushes the output once more at exit, so it goes to the null
        # device instead.
        with contextlib.suppress(OSError, ValueError):
            null = os.open(os.devnull, os.O_WRONLY)
            try:
                os.dup2(null, sys.stdout.fileno())
            finally:
                os.close(null)
        return 1
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
