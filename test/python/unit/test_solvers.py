# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the bring-your-own-license solver path, with gurobipy absent or replaced by a fake."""

from __future__ import annotations

import math
import subprocess
import sys
import textwrap
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd import pyscpd
from mqt.scpd.chip import load_chip
from mqt.scpd.cli import main
from mqt.scpd.config import load_config, write_config
from mqt.scpd.solvers import SolverUnavailableError, available, register, solve_mps

if TYPE_CHECKING:
    from collections.abc import Generator, Sequence

    from mqt.scpd.solvers.gurobipy_backend import SolveAnswer

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
def test_set_solver_refuses_an_object_that_cannot_be_called() -> None:
    """A solver that cannot be called is refused when it is registered, not when a stage solves."""
    with pytest.raises(TypeError):
        pyscpd.set_solver("gurobi")  # ty: ignore[invalid-argument-type]

    config = load_config(FIXTURE)
    assert config.stages is not None
    assert config.stages.solver is not None
    config.stages.solver.backend = "gurobi"
    packed = write_config(config)
    chip = load_chip(config, FIXTURE)
    capacity = pyscpd.plan_capacity(chip, packed, "test")
    routing = pyscpd.route_global(chip, capacity, packed, "test")
    # The refused object left no external solver behind.
    with pytest.raises(RuntimeError, match="no external solver is registered"):
        pyscpd.assign(chip, capacity, routing, packed, "test")


@pytest.mark.usefixtures("no_external_solver")
def test_a_registered_solver_receives_the_model_as_mps() -> None:
    """The external solver sees MPS text and the variable names, and its status reaches the stage."""
    seen: list[tuple[str, list[str]]] = []

    def refuse(mps: str, names: Sequence[str], time_limit: float, relative_gap: float, threads: int) -> SolveAnswer:
        del time_limit, relative_gap, threads
        seen.append((mps, list(names)))
        return "infeasible", float("nan"), [], float("nan"), float("nan"), None

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


class StepLines:
    """A reporter that keeps the lines of the -v level, where a stage reports how a solve ended."""

    def __init__(self) -> None:
        """Start with no line."""
        self.lines: list[str] = []

    def line(self, level: int, text: str) -> None:
        """Keep a line of the -v level."""
        if level == 1:
            self.lines.append(text)

    @staticmethod
    def entry(level: int, label: str, figures: object) -> None:
        """Ignore an entry."""
        del level, label, figures

    @staticmethod
    def result(figures: object, fails: int | None = None) -> None:
        """Ignore the result."""
        del figures, fails

    @staticmethod
    def progress(task: str, detail: str, done: int, total: int) -> None:
        """Ignore the progress."""
        del task, detail, done, total


@pytest.mark.usefixtures("no_external_solver")
def test_a_registered_solver_receives_the_threads_and_reports_its_search() -> None:
    """The callable receives [stages.solver] threads, and the bound, gap and nodes it returns reach the solve line."""
    threads_seen: list[int] = []

    def refuse(mps: str, names: Sequence[str], time_limit: float, relative_gap: float, threads: int) -> SolveAnswer:
        del mps, names, time_limit, relative_gap
        threads_seen.append(threads)
        return "infeasible", float("nan"), [], 4.0, 0.25, 9

    pyscpd.set_solver(refuse)
    config = load_config(FIXTURE)
    assert config.stages is not None
    assert config.stages.solver is not None
    config.stages.solver.backend = "gurobi"
    config.stages.solver.threads = 3
    packed = write_config(config)
    chip = load_chip(config, FIXTURE)
    capacity = pyscpd.plan_capacity(chip, packed, "test")
    routing = pyscpd.route_global(chip, capacity, packed, "test")
    reporter = StepLines()
    with pytest.raises(RuntimeError, match="the assignment did not solve: infeasible"):
        pyscpd.assign(chip, capacity, routing, packed, "test", reporter, 1)

    assert threads_seen == [3]
    assert "gurobi infeasible · bound 4.00 · gap 25.0 % · 9 nodes" in reporter.lines


def run_python(script: str, *arguments: str) -> subprocess.CompletedProcess[str]:
    """Run a script in a new interpreter, so that a crash of the interpreter fails one test only.

    Returns:
        The finished process, with its output.
    """
    return subprocess.run(  # ruff: ignore[subprocess-without-shell-equals-true]  # this interpreter runs a fixed script
        [sys.executable, "-c", textwrap.dedent(script), *arguments],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )


def test_a_solver_that_clears_itself_during_a_solve_ends_the_stage_cleanly() -> None:
    """The core releases a callable that unregisters itself during a solve and keeps the interpreter alive."""
    script = """
        import sys
        from pathlib import Path

        from mqt.scpd import pyscpd
        from mqt.scpd.chip import load_chip
        from mqt.scpd.config import load_config, write_config


        def clear_and_refuse(mps, names, time_limit, relative_gap, threads):
            pyscpd.set_solver(None)
            return "infeasible", 0.0, [], float("nan"), float("nan"), None


        pyscpd.set_solver(clear_and_refuse)
        del clear_and_refuse
        config = load_config(Path(sys.argv[1]))
        config.stages.solver.backend = "gurobi"
        packed = write_config(config)
        chip = load_chip(config, Path(sys.argv[1]))
        capacity = pyscpd.plan_capacity(chip, packed, "test")
        routing = pyscpd.route_global(chip, capacity, packed, "test")
        try:
            pyscpd.assign(chip, capacity, routing, packed, "test")
        except RuntimeError as error:
            print(error)
    """

    finished = run_python(script, str(FIXTURE))

    assert finished.returncode == 0, finished.stderr
    assert "the assignment did not solve: infeasible" in finished.stdout


def test_a_solver_registered_until_exit_lets_the_interpreter_exit_cleanly() -> None:
    """The core releases a callable that it alone holds and that is never cleared before the interpreter shuts down."""
    script = """
        from mqt.scpd import pyscpd

        pyscpd.set_solver(lambda mps, names, time_limit, relative_gap, threads: ("infeasible", 0.0, [], 0.0, 0.0, None))
    """

    finished = run_python(script)

    assert finished.returncode == 0, finished.stderr


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
        self.IsMIP = gurobi.integral
        self.NodeCount = 12.0

    @property
    def ObjBound(self) -> float:  # ruff: ignore[invalid-function-name]
        """The best bound, or the error of Gurobi when the fake has none to give.

        Raises:
            RuntimeError: If the fake has no bound to give, as Gurobi raises GurobiError.
        """
        if self.gurobi.bound is None:
            msg = "Unable to retrieve attribute 'ObjBound'"
            raise RuntimeError(msg)
        return self.gurobi.bound

    @property
    def MIPGap(self) -> float:  # ruff: ignore[invalid-function-name]
        """The relative gap."""
        return self.gurobi.gap

    def setParam(self, name: str, value: float) -> None:  # ruff: ignore[invalid-function-name]
        """Record a parameter of the solve, and give the status that the solve without dual reductions ends with."""
        self.gurobi.parameters[name] = value
        if name == "DualReductions":
            self.Status = self.gurobi.status_without_dual_reductions

    def optimize(self) -> None:
        """Record that the model was solved."""
        self.gurobi.solved.append(self.mps)

    def dispose(self) -> None:
        """Record the release."""
        self.gurobi.released.append("model")

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
        """Start the environment, or fail as Gurobi does without a license.

        Raises:
            RuntimeError: If the fake module has no license.
        """
        if not self.gurobi.licensed:
            msg = "no Gurobi license found"
            raise RuntimeError(msg)

    def dispose(self) -> None:
        """Count the release."""
        self.gurobi.disposed += 1
        self.gurobi.released.append("environment")


class FakeGurobi:
    """The part of gurobipy that the backend uses."""

    GurobiError = RuntimeError

    def __init__(self, status: int = 2, solutions: int = 1, status_without_dual_reductions: int = 3) -> None:
        """Set the status and the number of solutions that every solve ends with."""
        self.status = status
        self.status_without_dual_reductions = status_without_dual_reductions
        self.solutions = solutions
        self.runtime = 0.0
        self.integral = 1
        self.bound: float | None = 6.5
        self.gap = 0.07
        self.licensed = True
        self.parameters: dict[str, float] = {}
        self.solved: list[str] = []
        self.disposed = 0
        self.released: list[str] = []

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


def test_gurobi_is_available_when_a_license_can_be_acquired(monkeypatch: pytest.MonkeyPatch) -> None:
    """With gurobipy and a license the backend is available, and the license check releases it."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())

    assert available() == (True, "")
    assert gurobi.disposed == 1


def test_gurobi_is_unavailable_without_a_license_and_releases_its_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Without a license the backend is unavailable, says why, and still releases the environment."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())
    gurobi.licensed = False

    assert available() == (False, "gurobipy has no usable license (no Gurobi license found)")
    assert gurobi.released == ["environment"]


def test_gurobi_returns_the_values_in_the_order_of_the_names(monkeypatch: pytest.MonkeyPatch) -> None:
    """Gurobi reads the MPS text, honors the limits and answers by variable name."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())

    answer = solve_mps("NAME pair\nENDATA\n", ["x", "y"], time_limit=5.0, relative_gap=0.01, threads=4)

    assert gurobi.solved == ["NAME pair\nENDATA\n"]
    assert gurobi.parameters == {"TimeLimit": 5.0, "MIPGap": 0.01, "Threads": 4}
    assert answer == ("optimal", 7.0, [1.0, 2.0], 6.5, 0.07, 12)
    assert gurobi.released == ["model", "environment"]


def test_gurobi_leaves_the_threads_to_its_default_at_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    """Zero threads set no Threads parameter, so Gurobi picks the count itself."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())

    solve_mps("NAME pair\nENDATA\n", ["x", "y"])

    assert "Threads" not in gurobi.parameters


def test_gurobi_reports_no_search_figures_for_a_linear_program(monkeypatch: pytest.MonkeyPatch) -> None:
    """A linear program has no bound, gap or nodes, as with HiGHS."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())
    gurobi.integral = 0

    _, _, values, bound, gap, nodes = solve_mps("NAME pair\nENDATA\n", ["x", "y"])

    assert values == [1.0, 2.0]
    assert math.isnan(bound)
    assert math.isnan(gap)
    assert nodes is None


def test_gurobi_reports_a_figure_it_cannot_give_as_absent(monkeypatch: pytest.MonkeyPatch) -> None:
    """A bound that Gurobi refuses to give, and an infinite gap, are NaN; the solution stays."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())
    gurobi.bound = None
    gurobi.gap = 1e100

    status, objective, values, bound, gap, nodes = solve_mps("NAME pair\nENDATA\n", ["x", "y"])

    assert (status, objective, values, nodes) == ("optimal", 7.0, [1.0, 2.0], 12)
    assert math.isnan(bound)
    assert math.isnan(gap)


def test_gurobi_without_a_value_for_a_variable_is_an_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """A variable that Gurobi does not know makes the solve an error that names the variable."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())

    status, objective, values, _, _, nodes = solve_mps("NAME pair\nENDATA\n", ["x", "y", "z"])

    assert status == "gurobi returned no value for 1 of 3 variables: z"
    assert math.isnan(objective)
    assert values == []
    assert nodes is None
    assert gurobi.released == ["model", "environment"]


def test_gurobi_without_a_license_releases_its_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    """An environment that cannot start raises the error of Gurobi and is released."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi())
    gurobi.licensed = False

    with pytest.raises(RuntimeError, match="no Gurobi license found"):
        solve_mps("NAME pair\nENDATA\n", ["x"])

    assert gurobi.solved == []
    assert gurobi.released == ["environment"]


def test_an_interrupted_gurobi_solve_raises_keyboard_interrupt(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ctrl-C during a Gurobi solve raises KeyboardInterrupt once the model and the environment are released."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(11, 1))

    with pytest.raises(KeyboardInterrupt):
        solve_mps("NAME pair\nENDATA\n", ["x", "y"])

    assert gurobi.released == ["model", "environment"]


@pytest.mark.usefixtures("no_external_solver")
def test_a_run_interrupted_in_gurobi_ends_with_130(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Ctrl-C during a Gurobi solve ends the run as Ctrl-C during a HiGHS solve does, and writes no artifact."""
    with_gurobi(monkeypatch, FakeGurobi(11, 1))
    monkeypatch.setenv("SCPD_SOLVER", "gurobi")
    run = tmp_path / "run"

    assert main(["plan", "-c", str(FIXTURE), "-o", str(run), "--no-progress"]) == 130

    out = capsys.readouterr().out
    assert "📌 assign\n   ✗ interrupted" in out
    assert not (run / "03-assign.fb").exists()


@pytest.mark.parametrize(
    ("code", "solutions", "expected"),
    [
        (3, 0, "infeasible"),
        (2, 0, "gurobi stopped without a solution"),
        (8, 0, "gurobi stopped without a solution"),
        (17, 1, "gurobi ended with status 17"),
    ],
)
def test_gurobi_without_a_solution_returns_no_values(
    monkeypatch: pytest.MonkeyPatch, code: int, solutions: int, expected: str
) -> None:
    """A solve that ends without a solution returns its status and no values."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(code, solutions))

    status, objective, values, bound, _, nodes = solve_mps("NAME pair\nENDATA\n", ["x"])

    assert status == expected
    assert math.isnan(objective)
    assert values == []
    assert math.isnan(bound)
    assert nodes is None
    assert not gurobi.parameters
    assert gurobi.released == ["model", "environment"]


@pytest.mark.parametrize("code", [7, 8, 9, 10, 13, 15, 16])
def test_gurobi_stopped_at_a_limit_keeps_its_solution(monkeypatch: pytest.MonkeyPatch, code: int) -> None:
    """A limit that stops the search with a solution in hand gives that solution, as with HiGHS."""
    with_gurobi(monkeypatch, FakeGurobi(code, 1))

    answer = solve_mps("NAME pair\nENDATA\n", ["x", "y"])

    assert answer == ("feasible", 7.0, [1.0, 2.0], 6.5, 0.07, 12)


@pytest.mark.parametrize(("resolved", "expected"), [(3, "infeasible"), (5, "unbounded")])
def test_gurobi_tells_infeasible_from_unbounded(monkeypatch: pytest.MonkeyPatch, resolved: int, expected: str) -> None:
    """A solve that ends with "infeasible or unbounded" repeats without dual reductions and reports which."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(4, 0, resolved))

    status, objective, values, _, _, _ = solve_mps("NAME pair\nENDATA\n", ["x"])

    assert status == expected
    assert math.isnan(objective)
    assert values == []
    assert gurobi.parameters == {"DualReductions": 0}
    assert len(gurobi.solved) == 2


def test_gurobi_keeps_to_the_time_limit_over_both_solves(monkeypatch: pytest.MonkeyPatch) -> None:
    """The solve without dual reductions gets only the time that the first solve left."""
    gurobi = with_gurobi(monkeypatch, FakeGurobi(4, 0, 3))
    gurobi.runtime = 2.0

    status, *_ = solve_mps("NAME pair\nENDATA\n", ["x"], time_limit=5.0)

    assert status == "infeasible"
    assert gurobi.parameters == {"TimeLimit": 3.0, "DualReductions": 0}
    assert len(gurobi.solved) == 2
