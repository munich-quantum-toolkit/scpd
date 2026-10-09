# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the bring-your-own-licence solver path, without gurobipy installed."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd import pyscpd
from mqt.scpd.chip import load_chip
from mqt.scpd.config import load_config, write_config
from mqt.scpd.solvers import SolverUnavailableError, available, register, solve_mps

if TYPE_CHECKING:
    from collections.abc import Generator, Sequence

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini" / "config.toml"


@pytest.fixture
def no_external_solver() -> Generator[None]:
    """Leave the process without an external solver, whatever a test registers."""
    pyscpd.set_solver(None)
    yield
    pyscpd.set_solver(None)


@pytest.fixture
def without_gurobipy(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make gurobipy fail to import, whether or not it is installed."""
    monkeypatch.setitem(sys.modules, "gurobipy", None)


@pytest.mark.usefixtures("without_gurobipy", "no_external_solver")
def test_gurobi_is_unavailable_without_gurobipy() -> None:
    """Without gurobipy the backend says why and registers nothing."""
    usable, why = available()
    assert not usable
    assert "gurobipy is not installed" in why

    assert not register()
    packed = write_config(load_config(FIXTURE))
    assert pyscpd.solver_info(packed).startswith("HiGHS ")


@pytest.mark.usefixtures("without_gurobipy")
def test_solving_without_gurobipy_names_the_way_out() -> None:
    """A solve that has no gurobipy to reach points at the highs backend."""
    with pytest.raises(SolverUnavailableError, match="use the highs backend"):
        solve_mps("NAME empty\nENDATA\n", [])


@pytest.mark.usefixtures("no_external_solver")
def test_a_registered_solver_receives_the_model_as_mps() -> None:
    """The external solver sees MPS text and the variable names, and its status reaches the stage."""
    seen: list[tuple[str, list[str]]] = []

    def refuse(
        mps: str, names: Sequence[str], time_limit: float, relative_gap: float
    ) -> tuple[str, float, list[float]]:
        del time_limit, relative_gap
        seen.append((mps, list(names)))
        return "infeasible", float("nan"), []

    pyscpd.set_solver(refuse)
    config = load_config(FIXTURE)
    assert config.stages is not None
    assert config.stages.solver is not None
    config.stages.solver.backend = "gurobi"
    packed = write_config(config)
    assert pyscpd.solver_info(packed) == "gurobi"

    chip = load_chip(config, FIXTURE)
    capacity = pyscpd.plan_capacity(chip, packed, "test")
    routing = pyscpd.route_global(chip, capacity, packed, "test")
    with pytest.raises(RuntimeError, match="the assignment did not solve: infeasible"):
        pyscpd.assign(chip, capacity, routing, packed, "test")

    assert len(seen) == 1
    mps, names = seen[0]
    assert "COLUMNS" in mps
    assert "flow_0" in names
