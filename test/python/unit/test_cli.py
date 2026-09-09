# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the command line, through its entry point."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import TYPE_CHECKING

from mqt.scpd.artifacts import write_artifact
from mqt.scpd.cli import main
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput

if TYPE_CHECKING:
    import pytest

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"
CONFIG = str(BENCHMARKS / "4q" / "config.toml")


def test_doctor_reports_and_exits_zero(capsys: pytest.CaptureFixture[str]) -> None:
    """The doctor prints its report and its verdict."""
    assert main(["doctor", "-c", CONFIG]) == 0
    out = capsys.readouterr().out
    assert "doctor: OK" in out
    assert "launcher         16" in out
    assert "all_outer: 12 ports, entering at Q1.port0" in out


def test_plot_writes_the_layout_and_refuses_later_stages(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """The layout stage renders; a stage of a later phase names that phase."""
    output = tmp_path / "layout.svg"
    assert main(["plot", "-c", CONFIG, "--stage", "layout", "-o", str(output), "--width", "800"]) == 0
    assert output.read_text(encoding="utf-8").startswith("<svg")
    assert "wrote" in capsys.readouterr().out

    assert main(["plot", "-c", CONFIG, "--stage", "final", "-o", str(output)]) == 1
    assert "phase 4" in capsys.readouterr().err


def test_render_writes_a_layout_file(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """The unrouted chip is written as GDSII where KLayout is installed, and refused with a hint where not."""
    output = tmp_path / "chip.gds"
    code = main(["render", "-c", CONFIG, "-o", str(output)])
    captured = capsys.readouterr()
    if importlib.util.find_spec("klayout") is None:
        assert code == 1
        assert "mqt-scpd[klayout]" in captured.err
    else:
        assert code == 0
        assert output.stat().st_size > 0
        assert "28 polygons, 36 ports" in captured.out


def test_inspect_prints_an_artifact_as_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """An artifact file prints as JSON to stdout or to a file."""
    artifact = tmp_path / "02-global.fb"
    artifact.write_bytes(
        write_artifact(
            ArtifactT(
                producer="test",
                outputType=StageOutput.GlobalRouting,
                output=GlobalRoutingT(lattices=[], connections=[], outerRing=[], resonators=[]),
            )
        )
    )

    assert main(["inspect", str(artifact)]) == 0
    document = json.loads(capsys.readouterr().out)
    assert document == {
        "producer": "test",
        "outputType": "GlobalRouting",
        "output": {
            "lattices": [],
            "connections": [],
            "outerRing": [],
            "resonators": [],
            "objective": 0.0,
        },
    }

    output = tmp_path / "02-global.json"
    assert main(["inspect", str(artifact), "-o", str(output)]) == 0
    assert json.loads(output.read_text(encoding="utf-8")) == document


def test_problems_exit_with_one(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A missing configuration and bytes that are no artifact end with exit code 1 and a message."""
    assert main(["plot", "-c", str(tmp_path / "absent.toml"), "-o", str(tmp_path / "x.svg")]) == 1
    assert "error: cannot read" in capsys.readouterr().err

    broken = tmp_path / "broken.fb"
    broken.write_bytes(b"not an artifact")
    assert main(["inspect", str(broken)]) == 1
    assert "error:" in capsys.readouterr().err

    assert main(["doctor", "-c", str(tmp_path / "absent.toml")]) == 1
    assert "doctor: 1 problem(s)" in capsys.readouterr().out


def test_plan_fills_a_run_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """`plan` runs the three planning stages and names each artifact it wrote."""
    run = tmp_path / "run"
    assert main(["plan", "-c", str(BENCHMARKS / "4q" / "config.toml"), "-o", str(run)]) == 0

    captured = capsys.readouterr().out
    assert "01-capacity.fb" in captured
    assert "02-global.fb" in captured
    assert "03-assign.fb" in captured
    assert (run / "00-chip.json").is_file()


def test_plan_runs_one_stage_and_resumes(tmp_path: Path) -> None:
    """A single stage continues a run instead of starting one over."""
    run = tmp_path / "run"
    config = str(BENCHMARKS / "4q" / "config.toml")
    assert main(["plan", "-c", config, "-o", str(run)]) == 0
    before = (run / "01-capacity.fb").read_bytes()

    assert main(["plan", "-c", config, "-o", str(run), "--stage", "assign"]) == 0

    assert (run / "01-capacity.fb").read_bytes() == before
    assert (run / "03-assign.fb").is_file()


def test_plotting_a_stage_needs_a_run_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A stage other than layout is read from a run, and the message says so."""
    assert (
        main([
            "plot",
            "-c",
            str(BENCHMARKS / "4q" / "config.toml"),
            "--stage",
            "capacity",
            "-o",
            str(tmp_path / "out.svg"),
        ])
        == 1
    )
    assert "pass --run-dir" in capsys.readouterr().err


def test_plotting_a_stage_that_was_not_run_says_what_to_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A missing artifact names the command that produces it."""
    run = tmp_path / "run"
    run.mkdir()
    assert (
        main([
            "plot",
            "-c",
            str(BENCHMARKS / "4q" / "config.toml"),
            "--stage",
            "capacity",
            "--run-dir",
            str(run),
            "-o",
            str(tmp_path / "out.svg"),
        ])
        == 1
    )
    assert "mqt-scpd plan" in capsys.readouterr().err


def test_plotting_a_stage_of_a_later_phase_says_which(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A stage that has not been built yet is named with the phase it arrives in."""
    assert (
        main([
            "plot",
            "-c",
            str(BENCHMARKS / "4q" / "config.toml"),
            "--stage",
            "final",
            "-o",
            str(tmp_path / "out.svg"),
        ])
        == 1
    )
    assert "phase 4" in capsys.readouterr().err


def test_plot_draws_a_planning_stage_over_the_chip(tmp_path: Path) -> None:
    """`plot --stage` puts what a stage produced on top of the artwork."""
    run = tmp_path / "run"
    config = str(BENCHMARKS / "4q" / "config.toml")
    assert main(["plan", "-c", config, "-o", str(run)]) == 0

    output = tmp_path / "capacity.svg"
    assert main(["plot", "-c", config, "--stage", "capacity", "--run-dir", str(run), "-o", str(output)]) == 0

    svg = output.read_text(encoding="utf-8")
    assert 'class="l-bottleneck"' in svg
    assert "(capacity)" in svg

    # The global picture is drawn over the gates the circuit had to pay for, which the run
    # directory still carries, so the same layer is there.
    picture = tmp_path / "global.svg"
    assert main(["plot", "-c", config, "--stage", "global", "--run-dir", str(run), "-o", str(picture)]) == 0
    assert 'class="l-bottleneck"' in picture.read_text(encoding="utf-8")


def test_list_algorithms_names_one_implementation_per_stage(capsys: pytest.CaptureFixture[str]) -> None:
    """The registry ships one entry per stage, which is what the command exists to show."""
    assert main(["list-algorithms"]) == 0

    captured = capsys.readouterr().out
    assert "capacity-planner: watershed" in captured
    assert "global-router:    hanan-milp" in captured
    assert "assigner:         ordered-milp" in captured
