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

from .artifacts import ArtifactError
from .chip import ChipError, decode_chip, load_chip
from .config import ConfigError, load_config
from .doctor import run_doctor
from .export.klayout import ExportError, write_layout
from .inspection import InspectionError, artifact_to_json
from .plot import STAGES, PlotError, layout_svg


def _load(config_path: Path) -> tuple[bytes, Path]:
    """Load the configuration and its chip, for the commands that draw the chip.

    Returns:
        The classified chip bytes and the configuration path.
    """
    config = load_config(config_path)
    return load_chip(config, config_path), config_path


def command_doctor(args: argparse.Namespace) -> int:
    """Run the doctor and print its report.

    Returns:
        The exit code.
    """
    report = run_doctor(args.config, list_ports=args.ports)
    print(report.text())
    return 0 if report.ok else 1


def command_plot(args: argparse.Namespace) -> int:
    """Render one stage of a run as SVG.

    Returns:
        The exit code.
    """
    if args.stage != "layout":
        print(f"stage '{args.stage}' arrives with {STAGES[args.stage]}", file=sys.stderr)
        return 1
    chip_bytes, config_path = _load(args.config)
    svg = layout_svg(decode_chip(chip_bytes), width=args.width, tolerance=args.tolerance, title=str(config_path))
    args.output.write_text(svg, encoding="utf-8")
    print(f"wrote {args.output} ({len(svg.encode('utf-8')) / 1e6:.2f} MB)")
    return 0


def command_render(args: argparse.Namespace) -> int:
    """Write the unrouted chip as GDSII or OASIS.

    Returns:
        The exit code.
    """
    chip_bytes, _ = _load(args.config)
    summary = write_layout(decode_chip(chip_bytes), args.output)
    print(f"wrote {summary.path} as {summary.format}: {summary.polygons} polygons, {summary.ports} ports")
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
    plot.add_argument("-o", "--output", type=Path, required=True, help="the SVG file to write")
    plot.add_argument("--width", type=int, default=2000, help="the display width of the picture in pixels")
    plot.add_argument(
        "--tolerance",
        type=float,
        default=0.0,
        help="drop vertices within this distance (layout units) of the polygon edge; 0 keeps every vertex",
    )
    plot.set_defaults(run=command_plot)

    render = commands.add_parser("render", help="write the unrouted chip as GDSII or OASIS")
    render.add_argument("-c", "--config", type=Path, required=True, help="the config.toml of the chip")
    render.add_argument("-o", "--output", type=Path, required=True, help="the .gds or .oas file to write")
    render.set_defaults(run=command_render)

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
    except (ConfigError, ChipError, PlotError, ExportError, ArtifactError, InspectionError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
