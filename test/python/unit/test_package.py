# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the package layout."""

from __future__ import annotations

import mqt.scpd
from mqt.scpd import pyscpd


def test_package_imports() -> None:
    """The package and its native extension import."""
    assert mqt.scpd.__doc__ is not None
    assert pyscpd.__doc__ is not None
