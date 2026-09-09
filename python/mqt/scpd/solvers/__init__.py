# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Solver backends the user brings themselves.

HiGHS is linked into the core and is always there, so an installation without a commercial licence
is fully functional. Gurobi is reached at run time through ``gurobipy``, which the user installs
and licenses; the core hands it the model as MPS text and reads the solution back by variable name,
so nothing here is a build dependency of the package.
"""

from __future__ import annotations

from .gurobipy_backend import GUROBI_STATUS, available, register

__all__ = ["GUROBI_STATUS", "available", "register"]
