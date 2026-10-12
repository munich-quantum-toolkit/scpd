# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the bring-your-own-licence solver path, with gurobipy absent or replaced by a fake."""

from __future__ import annotations

import math
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


@pytest.mark.usefixtures("without_gurobipy")
def test_gurobi_is_unavailable_without_gurobipy() -> None:
    """Without gurobipy the backend says why."""
    usable, why = available()
    assert not usable
    assert "gurobipy is not installed" in why


@pytest.mark.usefixtures("no_external_solver")
def test_a_registered_gurobi_solves_only_when_the_configuration_names_it() -> None:
    """A registered backend leaves a configuration that names no backend with HiGHS."""
    register()
    config = load_config(FIXTURE)
    assert pyscpd.solver_info(write_config(config)).startswith("HiGHS ")

    assert config.stages is not None
    assert config.stages.solver is not None
    config.stages.solver.backend = "gurobi"
    assert pyscpd.solver_info(write_config(config)) == "gurobi"


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


class FakeVariable:
    """A variable of the fake Gurobi model."""

    def __init__(self, name: str, value: float) -> None:
        """Hold the name and the value of a solved variable."""
        self.VarName = name
        self.X = value


class FakeModel:
    """A Gurobi model that answers with a fixed status and fixed values."""

    def __init__(self, gurobi: FakeGurobi, mps: str) -> None:
        """Remember the model text and the module that answers for it."""
        self.gurobi = gurobi
        self.mps = mps
        self.Status = gurobi.status
        self.SolCount = gurobi.solutions
        self.ObjVal = 7.0
        self.Runtime = gurobi.runtime

    def setParam(self, name: str, value: float) -> None:  # ruff: ignore[invalid-function-name]
        """Record a parameter of the solve, and give the status that the solve without dual reductions ends with."""
        self.gurobi.parameters[name] = value
        if name == "DualReductions":
            self.Status = self.gurobi.status_without_dual_reductions

    def optimize(self) -> None:
        """Record that the model was solved."""
        self.gurobi.solved.append(self.mps)

    @staticmethod
    def getVars() -> list[FakeVariable]:  # ruff: ignore[invalid-function-name]
        """The variables, in an order other than the one of the model.

        Returns:
            The variables.
        """
        return [FakeVariable("y", 2.0), FakeVariable("x", 1.0)]


class FakeEnvironment:
    """A Gurobi environment that counts how often it was released."""

    def __init__(self, gurobi: FakeGurobi) -> None:
        """Remember the module that counts the releases."""
        self.gurobi = gurobi

    @staticmethod
    def setParam(name: str, value: float) -> None:  # ruff: ignore[invalid-function-name]
        """Accept a parameter of the environment."""
        del name, value

    def start(self) -> None:
        """Start the environment."""

    def dispose(self) -> None:
        """Count the release."""
        self.gurobi.disposed += 1


class FakeGurobi:
    """The part of gurobipy that the backend uses."""

    GurobiError = RuntimeError

    def __init__(self, status: int = 2, solutions: int = 1, status_without_dual_reductions: int = 3) -> None:
        """Set the status and the number of solutions that every solve ends with."""
        self.status = status
        self.status_without_dual_reductions = status_without_dual_reductions
        self.solutions = solutions
        self.runtime = 0.0
        self.parameters: dict[str, float] = {}
        self.solved: list[str] = []
        self.disposed = 0

    def Env(self, *, empty: bool) -> FakeEnvironment:  # ruff: ignore[invalid-function-name]
        """A new environment.

        Returns:
            The environment.
        """
        assert empty
        return FakeEnvironment(self)

    def read(self, path: str, env: FakeEnvironment) -> FakeModel:
        """The model in the file.

        Returns:
            The model.
        """
        del env
        return FakeModel(self, Path(path).read_text(encoding="utf-8"))


def with_gurobi(monkeypatch: pytest.MonkeyPatch, gurobi: FakeGurobi) -> FakeGurobi:
    """Make ``import gurobipy`` return the fake module.

    Returns:
        The fake module.
    """
    monkeypatch.setitem(sys.modules, "gurobipy", gurobi)
    return gurobi


def test_gurobi_is_available_when_a_licence_can_be_acquired(monkeypatch: pytest.MonkeyPatch) -> None:
    """With gurobipy and a licence the backend is available, and the licence check releases it."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())

    assert available() == (True, "")
    assert gurobi.disposed == 1


def test_gurobi_returns_the_values_in_the_order_of_the_names(monkeypatch: pytest.MonkeyPatch) -> None:
    """Gurobi reads the MPS text, honours the limits and answers by variable name."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())

    status, objective, values = solve_mps("NAME pair\nENDATA\n", ["x", "y", "z"], time_limit=5.0, relative_gap=0.01)

    assert gurobi.solved == ["NAME pair\nENDATA\n"]
    assert gurobi.parameters == {"TimeLimit": 5.0, "MIPGap": 0.01}
    assert (status, objective, values) == ("optimal", 7.0, [1.0, 2.0, 0.0])
    assert gurobi.disposed == 1


@pytest.mark.parametrize(
    ("code", "solutions", "expected"),
    [
        (3, 0, "infeasible"),
        (2, 0, "gurobi stopped without a solution"),
        (7, 1, "gurobi ended with status 7"),
    ],
)
def test_gurobi_without_a_solution_returns_no_values(
    monkeypatch: pytest.MonkeyPatch, code: int, solutions: int, expected: str
) -> None:
    """A solve that ends without a solution returns its status and no values."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(code, solutions))

    status, objective, values = solve_mps("NAME pair\nENDATA\n", ["x"])

    assert status == expected
    assert math.isnan(objective)
    assert values == []
    assert not gurobi.parameters
    assert gurobi.disposed == 1


@pytest.mark.parametrize(("resolved", "expected"), [(3, "infeasible"), (5, "unbounded")])
def test_gurobi_tells_infeasible_from_unbounded(monkeypatch: pytest.MonkeyPatch, resolved: int, expected: str) -> None:
    """A solve that ends with "infeasible or unbounded" repeats without dual reductions and reports which."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(4, 0, resolved))

    status, objective, values = solve_mps("NAME pair\nENDATA\n", ["x"])

    assert status == expected
    assert math.isnan(objective)
    assert values == []
    assert gurobi.parameters == {"DualReductions": 0}
    assert len(gurobi.solved) == 2


def test_gurobi_keeps_to_the_time_limit_over_both_solves(monkeypatch: pytest.MonkeyPatch) -> None:
    """The solve without dual reductions gets only the time that the first solve left."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(4, 0, 3))
    gurobi.runtime = 2.0

    status, _, _ = solve_mps("NAME pair\nENDATA\n", ["x"], time_limit=5.0)

    assert status == "infeasible"
    assert gurobi.parameters == {"TimeLimit": 3.0, "DualReductions": 0}
    assert len(gurobi.solved) == 2
