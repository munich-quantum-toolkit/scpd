# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Adapters from the geometry model to layout files.

Writing GDSII or OASIS needs KLayout, which is an optional dependency: install
``mqt-scpd[klayout]`` for it. This package therefore imports without it and states through
:data:`HAS_KLAYOUT` whether the adapter below is available. The adapter itself imports KLayout
the ordinary way, because it cannot do anything without it.
"""

# The optional dependency is probed here, so that importing this package states whether the
# adapter below can be imported at all.
# ruff: file-ignore[non-empty-init-module]

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING

try:
    import_module("klayout.db")
except ModuleNotFoundError as error:
    if error.name is not None and not error.name.startswith("klayout"):
        raise
    HAS_KLAYOUT = False
else:
    HAS_KLAYOUT = True

__all__ = ["HAS_KLAYOUT"]

if TYPE_CHECKING or HAS_KLAYOUT:
    from .klayout import (
        DATABASE_UNIT,
        FORMATS,
        OBSTACLE_LAYER,
        PORT_LAYER,
        ExportError,
        ExportSummary,
        write_layout,
    )

    __all__ += [
        "DATABASE_UNIT",
        "FORMATS",
        "OBSTACLE_LAYER",
        "PORT_LAYER",
        "ExportError",
        "ExportSummary",
        "write_layout",
    ]
