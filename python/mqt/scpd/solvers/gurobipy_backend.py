# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The Gurobi backend, reached over MPS text.

The core writes the model as MPS text, and this module reads the answer back by variable name.
That is the whole interface. MPS carries bounds, a variable type, linear rows and one linear
objective, which is all that the planning models use. The text carries names and nothing else, so
when this backend and the linked-in one disagree, they disagree about the model and not about the
encoding.

The package never imports ``gurobipy`` when it is imported, so an installation without it works.
"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

__all__ = ["GUROBI_STATUS", "SolveAnswer", "SolverUnavailableError", "available", "register", "solve_mps"]

#: What a solve returns to the core: the status, the objective, one value per variable, the best
#: bound, the relative gap and the branch-and-bound nodes. The bound and the gap are NaN and the
#: nodes are None where Gurobi reports none.
SolveAnswer = tuple[str, float, list[float], float, float, int | None]

#: What each Gurobi status code means in the terms of the core. Every other code is an error, which
#: the core reports and does not raise. Code 4 says "infeasible or unbounded"; the solve repeats
#: without dual reductions to tell the two apart, so that the code does not reach this table. Code
#: 11 says that Ctrl-C stopped the solve, and :func:`solve_mps` raises KeyboardInterrupt for it. A
#: limit that stops the search is "feasible", as for HiGHS: whether a solution exists is decided
#: by whether one came back.
GUROBI_STATUS: dict[int, str] = {
    2: "optimal",
    3: "infeasible",
    5: "unbounded",
    7: "feasible",  # iteration limit
    8: "feasible",  # node limit
    9: "feasible",  # time limit
    10: "feasible",  # solution limit
    13: "feasible",  # suboptimal
    15: "feasible",  # objective limit
    16: "feasible",  # work limit
}


#: The Gurobi status code for "infeasible or unbounded".
INFEASIBLE_OR_UNBOUNDED = 4

#: The Gurobi status code for a solve that Ctrl-C stopped. Gurobi takes the signal itself while it
#: solves, so Python raises no KeyboardInterrupt of its own.
INTERRUPTED = 11

#: The value Gurobi gives for an infinite figure, such as the gap of a zero objective.
GUROBI_INFINITY = 1e100

#: The search figures of a solve that has none: no bound, no gap and no nodes.
NO_FIGURES: tuple[float, float, int | None] = (float("nan"), float("nan"), None)


class SolverUnavailableError(RuntimeError):
    """Gurobi was asked for, but gurobipy is not installed."""


def available() -> tuple[bool, str]:
    """Whether ``gurobipy`` can be imported and a licence acquired.

    Returns:
        Whether Gurobi can be used, and why not when it cannot.
    """
    try:
        import gurobipy  # ruff: ignore[import-outside-top-level]  # ty: ignore[unresolved-import]
    except ImportError as error:
        return False, f"gurobipy is not installed ({error})"
    try:
        environment = gurobipy.Env(empty=True)
        environment.setParam("OutputFlag", 0)
        environment.start()
    except gurobipy.GurobiError as error:  # pragma: no cover - depends on the licence at hand
        return False, f"gurobipy has no usable licence ({error})"
    environment.dispose()
    return True, ""


def _figure(model: object, name: str, gurobi_error: type[Exception]) -> float:
    """Read one figure of a solved model.

    Args:
        model: The Gurobi model.
        name: The name of the attribute.
        gurobi_error: The error that Gurobi raises for an attribute it cannot give.

    Returns:
        The figure, or NaN when Gurobi gives none or an infinite one.
    """
    try:
        value = float(getattr(model, name))
    except (AttributeError, gurobi_error):
        return float("nan")
    return value if math.isfinite(value) and abs(value) < GUROBI_INFINITY else float("nan")


def _search_figures(model: object, gurobi_error: type[Exception]) -> tuple[float, float, int | None]:
    """The best bound, the relative gap and the nodes of a solved mixed-integer program.

    A linear program has no search, so it has no such figures, as with HiGHS.

    Args:
        model: The solved Gurobi model.
        gurobi_error: The error that Gurobi raises for an attribute it cannot give.

    Returns:
        The bound and the gap, or NaN for each figure that Gurobi does not give, and the nodes, or
        None when Gurobi does not give them.
    """
    integral = _figure(model, "IsMIP", gurobi_error)
    if math.isnan(integral) or integral < 1.0:
        return NO_FIGURES
    nodes = _figure(model, "NodeCount", gurobi_error)
    return (
        _figure(model, "ObjBound", gurobi_error),
        _figure(model, "MIPGap", gurobi_error),
        None if math.isnan(nodes) else int(nodes),
    )


def solve_mps(
    mps: str,
    names: Sequence[str],
    time_limit: float = 0.0,
    relative_gap: float = 0.0,
    threads: int = 0,
) -> SolveAnswer:
    """Solve a model that the core wrote as MPS text.

    Args:
        mps: The model as free-form MPS.
        names: The variable names in the order of the model, which is the order of the values.
        time_limit: The seconds the solve may take, or zero for no limit.
        relative_gap: The gap at which the solve may stop, or zero for the default of Gurobi.
        threads: The threads the solve may use, or zero for the default of Gurobi.

    Returns:
        The status, the objective, one value per name of ``names``, the best bound, the relative
        gap and the branch-and-bound nodes. A solve without a solution, and a linear program, give
        a NaN bound and gap and no nodes.

    Raises:
        SolverUnavailableError: If ``gurobipy`` cannot be imported.
        KeyboardInterrupt: If Ctrl-C stopped the solve. The model and the environment are released
            first.
    """
    try:
        import gurobipy  # ruff: ignore[import-outside-top-level]  # ty: ignore[unresolved-import]
    except ImportError as error:
        msg = "gurobipy is not installed; install it and a licence, or use the highs backend"
        raise SolverUnavailableError(msg) from error

    # Gurobi reads a model from a file, so the text goes through one. That takes milliseconds
    # against models of a few thousand variables.
    with tempfile.TemporaryDirectory(prefix="mqt-scpd-") as directory:
        path = Path(directory) / "model.mps"
        path.write_text(mps, encoding="utf-8")

        environment = gurobipy.Env(empty=True)
        try:
            environment.setParam("OutputFlag", 0)
            environment.start()
            model = gurobipy.read(str(path), env=environment)
            # Gurobi frees a model before the environment that holds it.
            try:
                if time_limit > 0.0:
                    model.setParam("TimeLimit", time_limit)
                if relative_gap > 0.0:
                    model.setParam("MIPGap", relative_gap)
                if threads > 0:
                    model.setParam("Threads", threads)
                model.optimize()
                if model.Status == INFEASIBLE_OR_UNBOUNDED:
                    # Presolve stops at this code when it proves that no optimum exists. Without dual
                    # reductions Gurobi says which of the two cases holds, as HiGHS does. The second solve
                    # gets the time the first one left, so that both together keep to the limit.
                    model.setParam("DualReductions", 0)
                    if time_limit > 0.0:
                        model.setParam("TimeLimit", max(time_limit - model.Runtime, 0.0))
                    model.optimize()
                if model.Status == INTERRUPTED:
                    # The finally clauses release the model and the environment before the
                    # interrupt reaches the command line.
                    raise KeyboardInterrupt

                status = GUROBI_STATUS.get(model.Status, f"gurobi ended with status {model.Status}")
                if status not in {"optimal", "feasible"} or model.SolCount == 0:
                    if status in {"optimal", "feasible"}:
                        status = "gurobi stopped without a solution"
                    return status, float("nan"), [], *NO_FIGURES

                # MPS keeps names and nothing else, so the values are matched by name and not by the
                # order in which Gurobi holds the variables.
                by_name = {variable.VarName: variable.X for variable in model.getVars()}
                # The core writes every variable into the MPS text, so a name that Gurobi does not
                # know means that the model and the answer do not belong together.
                if missing := [name for name in names if name not in by_name]:
                    shown = ", ".join(missing[:3]) + (", ..." if len(missing) > 3 else "")
                    message = f"gurobi returned no value for {len(missing)} of {len(names)} variables: {shown}"
                    return message, float("nan"), [], *NO_FIGURES
                # MPS states a minimization. The core writes a maximization with its objective
                # negated and negates the objective and the bound it gets back, so both are returned
                # as the text states them.
                values = [by_name[name] for name in names]
                return status, float(model.ObjVal), values, *_search_figures(model, gurobipy.GurobiError)
            finally:
                model.dispose()
        finally:
            environment.dispose()


def register() -> None:
    """Install this backend as the external solver of the process.

    The core uses the backend only for a configuration that asks for ``gurobi``. Registering it
    imports nothing and acquires no licence: a solve without gurobipy raises
    :class:`SolverUnavailableError`, and a solve without a licence raises the error of Gurobi.
    """
    from .. import pyscpd  # ruff: ignore[import-outside-top-level]

    pyscpd.set_solver(solve_mps)
