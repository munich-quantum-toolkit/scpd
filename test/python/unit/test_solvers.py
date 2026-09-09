# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The bring-your-own-licence solver path, which is reached over an MPS round trip."""

from __future__ import annotations

import sys

import pytest

from mqt.scpd.solvers import available, register
from mqt.scpd.solvers.gurobipy_backend import GUROBI_STATUS, SolverUnavailableError, solve_mps

#: An MPS file with one column and no rows, which is enough to reach the import.
TRIVIAL_MPS = "NAME t\nROWS\n N  COST\nCOLUMNS\n    x  COST  1\nRHS\nBOUNDS\nENDATA\n"


@pytest.fixture
def without_gurobipy(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make importing gurobipy fail, as it does on an installation without it.

    Binding the name to ``None`` in the module table is what the import machinery reports as an
    ImportError, so this reproduces the real failure rather than a stand-in for it.
    """
    monkeypatch.setitem(sys.modules, "gurobipy", None)


def test_the_status_map_covers_what_the_core_understands() -> None:
    """Every status the map produces is one the core knows how to read back."""
    assert set(GUROBI_STATUS.values()) <= {"optimal", "feasible", "infeasible", "unbounded"}
    assert GUROBI_STATUS[2] == "optimal"


def test_an_installation_without_gurobipy_says_so(without_gurobipy: None) -> None:
    """`doctor` reports why the backend is unreachable rather than failing."""
    del without_gurobipy
    usable, why = available()
    assert not usable
    assert "gurobipy is not installed" in why


def test_registering_without_gurobipy_changes_nothing(without_gurobipy: None) -> None:
    """A run works with the linked-in solver alone, which is the point of vendoring it."""
    del without_gurobipy
    assert register() is False


def test_solving_without_gurobipy_names_the_alternative(without_gurobipy: None) -> None:
    """The message says what to do rather than only what went wrong."""
    del without_gurobipy
    with pytest.raises(SolverUnavailableError, match="use the highs backend"):
        solve_mps(TRIVIAL_MPS, ["x"])
