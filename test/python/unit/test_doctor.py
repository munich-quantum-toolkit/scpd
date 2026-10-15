# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the doctor."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from mqt.scpd.doctor import run_doctor

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"


def test_the_fixture_passes() -> None:
    """The fixture configuration passes, and the report ends with the verdict."""
    report = run_doctor(FIXTURE / "config.toml")

    assert report.ok, report.text()
    assert report.text().endswith("doctor: OK")


def test_the_report_carries_the_table_the_ring_and_the_ports() -> None:
    """The classification table, the configured ring and the optional port list are printed."""
    report = run_doctor(FIXTURE / "config.toml", list_ports=True)

    text = report.text()
    assert "conventional      3" in text
    assert "all_outer: 5 ports, entering at Q1.port0" in text
    assert "fixed_outer: 3 ports" in text
    assert "  Q1.port0                 resonator" in text


def test_problems_end_the_report_with_a_verdict(tmp_path: Path) -> None:
    """A missing file, a chip that cannot be read and a sequence the chip lacks are problems, not crashes."""
    report = run_doctor(tmp_path / "absent.toml")
    assert not report.ok
    assert report.text().endswith("doctor: 1 problem(s)")

    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    broken = tmp_path / "config.toml"
    broken.write_text(config.replace('input = "routing_config.json"', 'input = "absent.json"'), encoding="utf-8")
    report = run_doctor(broken)
    assert not report.ok
    assert "cannot read the chip input" in report.problems[0]

    wrong_ring = tmp_path / "ring.toml"
    wrong_ring.write_text(
        config.replace(
            'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
        ).replace('"Q1.port0",', '"Q9.port0",', 1),
        encoding="utf-8",
    )
    report = run_doctor(wrong_ring)
    assert not report.ok
    assert "'Q9.port0' is not a port of the chip" in report.problems[0]


def test_a_default_written_out_is_not_a_problem(tmp_path: Path) -> None:
    """The rules for a shipped file are for the benchmarks; a user may write a default out."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    written_out = tmp_path / "config.toml"
    written_out.write_text(
        config.replace(
            'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
        ).replace("capacity_cells_x = 12", "capacity_cells_x = 50"),
        encoding="utf-8",
    )
    report = run_doctor(written_out)
    assert report.ok, report.text()


def test_the_report_names_the_solvers_it_can_reach() -> None:
    """The report says that HiGHS is always there and whether Gurobi can be reached."""
    text = run_doctor(FIXTURE / "config.toml").text()

    assert "solvers" in text
    assert "  highs:  linked in, always available" in text
    assert "  gurobi: " in text


def test_the_table_shows_the_bridge_pattern(tmp_path: Path) -> None:
    """A configuration with a bridge pattern shows it in the table, with the ports it matches."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    bridged = tmp_path / "bridged.toml"
    bridged.write_text(
        config.replace(
            'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
        ).replace("[ports.patterns]\n", "[ports.patterns]\nbridge_pair = '^C\\d+\\.port[12]$'\n"),
        encoding="utf-8",
    )

    report = run_doctor(bridged)

    assert report.ok, report.text()
    assert "bridge_pair       0  ^C\\d+\\.port[12]$" in report.text()


def test_what_would_stop_the_configured_steps_is_a_problem(tmp_path: Path) -> None:
    """A run through the assignment needs a launcher target; a run that stops before it does not."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    config = config.replace(
        'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
    ).replace("launcher_target = 1", "launcher_target = 0")
    missing = tmp_path / "missing.toml"
    missing.write_text(config, encoding="utf-8")

    report = run_doctor(missing)
    assert not report.ok
    assert report.problems == [
        "[stages.assignment] launcher_target is missing; it is a per-chip figure with no default"
    ]

    stopped = tmp_path / "stopped.toml"
    stopped.write_text(config + '\n[run]\nstop_after = "global"\n', encoding="utf-8")
    report = run_doctor(stopped)
    assert report.ok, report.text()


def test_a_solver_backend_the_run_cannot_reach_is_a_problem(monkeypatch: pytest.MonkeyPatch) -> None:
    """An unknown backend, or Gurobi without gurobipy, would stop the run, so the doctor refuses both."""
    monkeypatch.setenv("SCPD_SOLVER", "cplex")
    report = run_doctor(FIXTURE / "config.toml")
    assert report.problems == ["SCPD_SOLVER: 'cplex' is no solver backend; use 'highs' or 'gurobi'"]

    monkeypatch.setenv("SCPD_SOLVER", "gurobi")
    monkeypatch.setitem(sys.modules, "gurobipy", None)
    report = run_doctor(FIXTURE / "config.toml")
    assert len(report.problems) == 1
    assert report.problems[0].startswith("SCPD_SOLVER asks for gurobi, but gurobipy is not installed")


def test_a_solver_problem_names_where_the_backend_name_comes_from(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Without ``SCPD_SOLVER``, the problem names the configuration key that chose the backend."""
    monkeypatch.delenv("SCPD_SOLVER", raising=False)
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    config = config.replace(
        'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
    )
    gurobi = tmp_path / "gurobi.toml"
    gurobi.write_text(config + '\n[stages.solver]\nbackend = "gurobi"\n', encoding="utf-8")
    monkeypatch.setitem(sys.modules, "gurobipy", None)
    problems = run_doctor(gurobi).problems
    assert len(problems) == 1
    assert problems[0].startswith("[stages.solver] backend asks for gurobi, but gurobipy is not installed")


@pytest.mark.parametrize("step", ["capacity", "global"])
def test_a_run_that_solves_nothing_needs_no_solver(monkeypatch: pytest.MonkeyPatch, tmp_path: Path, step: str) -> None:
    """A run that solves nothing never reaches the solver backend, but an unknown backend name is still a problem.

    The fixture's ring carries every routable port, so its global step has no inner circuit to solve.
    """
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    stopped = tmp_path / "config.toml"
    stopped.write_text(
        config.replace('input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"')
        + f'\n[run]\nstop_after = "{step}"\n',
        encoding="utf-8",
    )
    monkeypatch.setenv("SCPD_SOLVER", "gurobi")
    monkeypatch.setitem(sys.modules, "gurobipy", None)
    report = run_doctor(stopped)
    assert report.ok, report.text()

    monkeypatch.setenv("SCPD_SOLVER", "cplex")
    report = run_doctor(stopped)
    assert report.problems == ["SCPD_SOLVER: 'cplex' is no solver backend; use 'highs' or 'gurobi'"]
