# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The bring-your-own-licence Gurobi backend, reached over an MPS round trip.

The core emits the model as MPS text and this reads the answer back by variable name. That is the
whole interface, and it is deliberately narrow: MPS carries bounds, a variable type, linear rows
and one linear objective, which is exactly what the planning models use. Because the round trip
carries names and nothing else, a disagreement between this backend and the linked-in one is a
disagreement about the model rather than about the encoding.

The package never imports ``gurobipy`` at import time, so an installation without it works.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

__all__ = ["GUROBI_STATUS", "SolverUnavailableError", "available", "register", "solve_mps"]

#: What each Gurobi status code means in the core's own terms. Everything else is an error, which
#: the core reports rather than raising.
GUROBI_STATUS: dict[int, str] = {
    2: "optimal",
    3: "infeasible",
    4: "unbounded",
    5: "unbounded",
    9: "feasible",
    11: "feasible",
    13: "feasible",
}


class SolverUnavailableError(RuntimeError):
    """Gurobi was asked for but is not installed, or has no licence for the model."""


def available() -> tuple[bool, str]:
    """Whether ``gurobipy`` can be imported and a licence acquired.

    Returns:
        Whether it is usable, and why not when it is not.
    """
    try:
        import gurobipy  # noqa: PLC0415  # ty: ignore[unresolved-import]
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


def solve_mps(
    mps: str,
    names: Sequence[str],
    time_limit: float = 0.0,
    relative_gap: float = 0.0,
) -> tuple[str, float, list[float]]:
    """Solve a model that the core emitted as MPS text.

    Args:
        mps: The model as free-form MPS.
        names: The variable names in the model's own order, which the values come back in.
        time_limit: Seconds the solve may take, or zero for no limit.
        relative_gap: The gap the solve may stop at, or zero for Gurobi's own default.

    Returns:
        The status, the objective and one value per variable of ``names``.

    Raises:
        SolverUnavailableError: If ``gurobipy`` cannot be imported.
    """
    try:
        import gurobipy  # noqa: PLC0415  # ty: ignore[unresolved-import]
    except ImportError as error:
        msg = "gurobipy is not installed; install it and a licence, or use the highs backend"
        raise SolverUnavailableError(msg) from error

    # Gurobi reads a model from a file, so the text goes through one. The round trip is
    # milliseconds against models of a few thousand variables.
    with tempfile.TemporaryDirectory(prefix="mqt-scpd-") as directory:
        path = Path(directory) / "model.mps"
        path.write_text(mps, encoding="utf-8")

        environment = gurobipy.Env(empty=True)
        environment.setParam("OutputFlag", 0)
        environment.start()
        try:
            model = gurobipy.read(str(path), env=environment)
            if time_limit > 0.0:
                model.setParam("TimeLimit", time_limit)
            if relative_gap > 0.0:
                model.setParam("MIPGap", relative_gap)
            model.optimize()

            status = GUROBI_STATUS.get(model.Status, f"gurobi ended with status {model.Status}")
            if status not in {"optimal", "feasible"} or model.SolCount == 0:
                if status in {"optimal", "feasible"}:
                    status = "gurobi stopped without a solution"
                return status, float("nan"), []

            # The MPS round trip preserves names and nothing else, so the values are matched back
            # by name rather than by the order Gurobi happens to hold them in.
            by_name = {variable.VarName: variable.X for variable in model.getVars()}
            # MPS states a minimization, so the core negates a maximization before writing it and
            # undoes that itself; the objective comes back exactly as the file asked for.
            return status, float(model.ObjVal), [by_name.get(name, 0.0) for name in names]
        finally:
            environment.dispose()


def register() -> bool:
    """Install this backend as the external solver of the process, if it can be used.

    Returns:
        Whether it was installed.
    """
    from .. import pyscpd  # noqa: PLC0415

    usable, _ = available()
    if usable:
        pyscpd.set_solver(solve_mps)
    return usable
