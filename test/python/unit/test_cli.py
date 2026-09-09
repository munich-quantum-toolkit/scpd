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

import pytest

from mqt.scpd.artifacts import write_artifact
from mqt.scpd.cli import main
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput

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
        assert "207 polygons, 36 ports" in captured.out


def test_inspect_prints_an_artifact_as_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """An artifact file prints as JSON to stdout or to a file."""
    artifact = tmp_path / "03-global.fb"
    artifact.write_bytes(write_artifact(ArtifactT(producer="test", outputType=StageOutput.GlobalRouting, output=GlobalRoutingT())))

    assert main(["inspect", str(artifact)]) == 0
    document = json.loads(capsys.readouterr().out)
    assert document == {"producer": "test", "outputType": "GlobalRouting", "output": {}}

    output = tmp_path / "03-global.json"
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
