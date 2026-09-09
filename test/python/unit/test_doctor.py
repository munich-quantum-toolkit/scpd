# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the doctor."""

from __future__ import annotations

from pathlib import Path

import pytest

from mqt.scpd.doctor import run_doctor

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"


@pytest.mark.parametrize("chip", ["4q", "9q"])
def test_the_committed_benchmarks_pass(chip: str) -> None:
    """Each committed configuration passes as shipped, and the report ends with the verdict."""
    report = run_doctor(BENCHMARKS / chip / "config.toml")

    assert report.ok, report.text()
    text = report.text()
    assert "outer port ring" in text
    assert text.endswith("doctor: OK")


def test_the_report_carries_the_table_the_ring_and_the_ports() -> None:
    """The classification table, the configured ring and the optional port list are printed."""
    report = run_doctor(BENCHMARKS / "9q" / "config.toml", list_ports=True)

    text = report.text()
    assert "mating           60" in text
    assert "all_outer: 40 ports, entering at Qb1.port1" in text
    assert "fixed_outer: 21 ports" in text
    assert "  Qb1.port0                resonator" in text


def test_problems_end_the_report_with_a_verdict(tmp_path: Path) -> None:
    """A missing file, a chip that cannot be read and a sequence the chip lacks are problems, not crashes."""
    report = run_doctor(tmp_path / "absent.toml")
    assert not report.ok
    assert report.text().endswith("doctor: 1 problem(s)")

    config = (BENCHMARKS / "4q" / "config.toml").read_text(encoding="utf-8")
    broken = tmp_path / "config.toml"
    broken.write_text(config.replace('input = "routing_config.json"', 'input = "absent.json"'), encoding="utf-8")
    report = run_doctor(broken)
    assert not report.ok
    assert "cannot read the chip input" in report.problems[0]

    wrong_ring = tmp_path / "ring.toml"
    wrong_ring.write_text(
        config.replace('input = "routing_config.json"', f'input = "{BENCHMARKS / "4q" / "routing_config.json"}"').replace(
            '"Q1.port0",', '"Q9.port0",', 1
        ),
        encoding="utf-8",
    )
    report = run_doctor(wrong_ring)
    assert not report.ok
    assert "'Q9.port0' is not a port of the chip" in report.problems[0]
