# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""What the two SAT benchmarks share: the chips, the run of a chip to its coupler insertion, and the report.

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

from mqt.scpd.run import RunDirectory
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from mqt.scpd.flatbuffers.config.Config import ConfigT

REPOSITORY = Path(__file__).resolve().parents[3]
DATA = (
    Path(os.environ["MQT_SCPD_BENCHMARKS"]) / "inputs"
    if "MQT_SCPD_BENCHMARKS" in os.environ
    else REPOSITORY / "benchmarks"
)
CHIPS = os.environ.get("MQT_SCPD_SAT_CHIPS", "4q 9q 17q").split()
REPORT = Path(os.environ.get("MQT_SCPD_SAT_REPORT", "."))

#: The four classes of a case, in the order the core counts them.
CLASSES = ("satClean", "satFailing", "unsatClean", "unsatFailing")

gated = pytest.mark.skipif(
    not os.environ.get("MQT_SCPD_SAT_BENCHMARKS"),
    reason="MQT_SCPD_SAT_BENCHMARKS is not set; the SAT benchmarks take minutes to hours",
)


def plan_to_the_insertion(chip: str, directory: Path) -> tuple[RunDirectory, ConfigT]:
    """Plan a chip through the coupler insertion, so that a session can start from its artifacts.

    Returns:
        The run directory and its configuration.
    """
    run = RunDirectory(directory)
    config = run.prepare(DATA / chip / "config.toml")
    chip_bytes = run.classified_chip(config)
    for step in STEPS[:-1]:
        run.run_stage(step, config, chip_bytes)
    return run, config


def agreement(classes: dict[str, int]) -> float:
    """The share of the cases on which the check and the routing agree.

    Returns:
        ``(SAT and clean + UNSAT and failing) / all``, or 1.0 without a case.
    """
    total = sum(classes[name] for name in CLASSES)
    return 1.0 if total == 0 else (classes["satClean"] + classes["unsatFailing"]) / total


def write_report(name: str, chip: str, table: dict[str, Any]) -> Path:
    """Write the table of one chip as JSON beside the others and print its line.

    Returns:
        The file written.
    """
    REPORT.mkdir(parents=True, exist_ok=True)
    path = REPORT / f"sat-{name}-{chip}.json"
    path.write_text(json.dumps(table, indent=2) + "\n", encoding="utf-8")
    classes = table["classes"]
    print(
        f"{name} {chip}: {table['cases']} cases · SAT and clean {classes['satClean']} · "
        f"SAT but failing {classes['satFailing']} · UNSAT but clean {classes['unsatClean']} · "
        f"UNSAT and failing {classes['unsatFailing']} · agreement {100 * agreement(classes):.1f} % · "
        f"{table['seconds']:.0f}s"
    )
    return path
