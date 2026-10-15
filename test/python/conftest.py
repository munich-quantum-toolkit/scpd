# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Shared fixtures of the Python tests."""

from __future__ import annotations

import importlib
import pkgutil
from typing import TYPE_CHECKING

import pytest

import mqt.scpd.flatbuffers

if TYPE_CHECKING:
    from collections.abc import Generator

#: The variables through which rich forces colour or a terminal. CI runners set ``FORCE_COLOR``, and
#: the tests compare the plain text that a file receives. ``pyproject.toml`` names the fixture in
#: ``usefixtures``, so that every test uses it.
_TERMINAL_VARIABLES = ("FORCE_COLOR", "NO_COLOR", "TTY_COMPATIBLE", "TTY_INTERACTIVE", "COLUMNS", "LINES")


@pytest.fixture
def plain_terminal(monkeypatch: pytest.MonkeyPatch) -> None:
    """Run every test without the variables that change the terminal output."""
    for name in _TERMINAL_VARIABLES:
        monkeypatch.delenv(name, raising=False)


@pytest.fixture(scope="session")
def default_solver() -> Generator[None]:
    """Run every test with the solver backend that its configuration names.

    ``SCPD_SOLVER`` overrides the configured backend, so a value in the shell would change what the
    tests check. ``pyproject.toml`` names the fixture in ``usefixtures``. The fixture has session
    scope, so it also comes before the module fixtures that plan a run. A test that sets the
    variable itself does so after this fixture.
    """
    with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.delenv("SCPD_SOLVER", raising=False)
        yield


@pytest.fixture(params=["list", "numpy"])
def vector_type(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> str:
    """Decode the numeric vectors of every schema as a list, then as a NumPy array.

    The generated object model reads a vector of numbers into a NumPy array when NumPy is
    importable, and into a list otherwise. Each generated module keeps NumPy in its attribute
    ``np``, so the fixture sets that attribute in every module.

    Returns:
        The name of the type the vectors are read into.
    """
    numpy = None if request.param == "list" else pytest.importorskip("numpy")
    package = mqt.scpd.flatbuffers
    for module in pkgutil.walk_packages(package.__path__, f"{package.__name__}."):
        loaded = importlib.import_module(module.name)
        if hasattr(loaded, "np"):
            monkeypatch.setattr(loaded, "np", numpy)
    return request.param
