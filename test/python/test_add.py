# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Test the add function."""

from mqt.scpd.pyscpd import add


def test_add() -> None:
    """Test the add function."""
    assert add(1, 2) == 3
