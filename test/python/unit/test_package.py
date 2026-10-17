# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the package layout."""

from __future__ import annotations

from importlib import metadata

import mqt.scpd
from mqt.scpd import pyscpd


def test_package_imports() -> None:
    """The package and its native extension import."""
    assert mqt.scpd.__doc__ is not None
    assert pyscpd.__doc__ is not None


def test_distribution_carries_the_licenses_of_the_linked_libraries() -> None:
    """The extension links HiGHS with its LP reader, FlatBuffers and nlohmann/json, so their licenses go with it."""
    files = metadata.files("mqt-scpd") or []
    licenses = {file.name for file in files if file.parent.name == "licenses"}
    assert {
        "LICENSE",
        "HiGHS-LICENSE.txt",
        "HiGHS-filereaderlp-LICENSE.txt",
        "FlatBuffers-LICENSE.txt",
        "nlohmann_json-LICENSE.txt",
    } <= licenses
