# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""What the two SAT benchmarks share: the chips, the run of a chip to its coupler insertion, and the report.

The measurement itself is :mod:`mqt.scpd.debug.bench`, which ``mqt-scpd sat-bench`` runs as well.

The benchmarks run only with ``MQT_SCPD_SAT_BENCHMARKS`` set, because they take minutes on the small
chips and hours on the large ones. ``MQT_SCPD_SAT_CHIPS`` names the chips to measure, ``4q 9q 17q``
when it is unset. ``MQT_SCPD_BENCHMARKS`` names a clone of the benchmark data; without it the chips
ship in ``benchmarks/`` of the repository. ``MQT_SCPD_SAT_REPORT`` names the directory the tables go
to, the working directory when it is unset.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any

import pytest

from mqt.scpd.debug import bench

if TYPE_CHECKING:
    from mqt.scpd.flatbuffers.config.Config import ConfigT
    from mqt.scpd.run import RunDirectory

REPOSITORY = Path(__file__).resolve().parents[3]
DATA = (
    Path(os.environ["MQT_SCPD_BENCHMARKS"]) / "inputs"
    if "MQT_SCPD_BENCHMARKS" in os.environ
    else REPOSITORY / "benchmarks"
)
CHIPS = os.environ.get("MQT_SCPD_SAT_CHIPS", "4q 9q 17q").split()
REPORT = Path(os.environ.get("MQT_SCPD_SAT_REPORT", "."))

gated = pytest.mark.skipif(
    not os.environ.get("MQT_SCPD_SAT_BENCHMARKS"),
    reason="MQT_SCPD_SAT_BENCHMARKS is not set; the SAT benchmarks take minutes to hours",
)


def plan_to_the_insertion(chip: str, directory: Path) -> tuple[RunDirectory, ConfigT]:
    """Plan a chip of the benchmark data through the Detail stage.

    Returns:
        The run directory and its configuration.
    """
    return bench.plan_to_the_insertion(DATA / chip / "config.toml", directory)


def write_report(name: str, chip: str, table: dict[str, Any]) -> Path:
    """Write the table of one chip as JSON beside the others, with its page, and print its line.

    Returns:
        The JSON file written.
    """
    REPORT.mkdir(parents=True, exist_ok=True)
    path = REPORT / f"sat-{name}-{chip}.json"
    path.write_text(json.dumps(table, indent=1) + "\n", encoding="utf-8")
    path.with_suffix(".html").write_text(bench.page(name, table, live=False), encoding="utf-8")
    print(bench.summary(name, chip, table))
    return path
