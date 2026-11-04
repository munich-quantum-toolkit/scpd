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

from mqt.scpd.doctor import run_doctor

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"


def test_the_fixture_passes() -> None:
    """The fixture configuration passes, and the report ends with the verdict."""
    report = run_doctor(FIXTURE / "config.toml")

    assert report.ok, report.text()
    assert report.text().endswith("doctor: OK")


def test_the_report_carries_the_table_the_ring_and_the_ports() -> None:
    """The classification table, the configured ring and the optional port list are printed."""
    report = run_doctor(FIXTURE / "config.toml", list_ports=True)

    text = report.text()
    assert "conventional      3" in text
    assert "all_outer: 5 ports, entering at Q1.port0" in text
    assert "fixed_outer: 3 ports" in text
    assert "  Q1.port0                 resonator" in text


def test_problems_end_the_report_with_a_verdict(tmp_path: Path) -> None:
    """A missing file, a chip that cannot be read and a sequence the chip lacks are problems, not crashes."""
    report = run_doctor(tmp_path / "absent.toml")
    assert not report.ok
    assert report.text().endswith("doctor: 1 problem(s)")

    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    broken = tmp_path / "config.toml"
    broken.write_text(config.replace('input = "routing_config.json"', 'input = "absent.json"'), encoding="utf-8")
    report = run_doctor(broken)
    assert not report.ok
    assert "cannot read the chip input" in report.problems[0]

    wrong_ring = tmp_path / "ring.toml"
    wrong_ring.write_text(
        config.replace(
            'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
        ).replace('"Q1.port0",', '"Q9.port0",', 1),
        encoding="utf-8",
    )
    report = run_doctor(wrong_ring)
    assert not report.ok
    assert "'Q9.port0' is not a port of the chip" in report.problems[0]


def test_a_file_that_is_not_utf8_is_a_problem(tmp_path: Path) -> None:
    """A configuration or a chip input in another encoding ends the report with a verdict."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    latin = tmp_path / "latin.toml"
    latin.write_bytes(("# Gr\u00f6\u00dfe\n" + config).encode("latin-1"))
    report = run_doctor(latin)
    assert "it is not UTF-8" in report.problems[0]
    assert report.text().endswith("doctor: 1 problem(s)")

    chip = tmp_path / "chip.json"
    chip.write_bytes(b"\xff" + (FIXTURE / "routing_config.json").read_bytes())
    broken = tmp_path / "config.toml"
    broken.write_text(config.replace('input = "routing_config.json"', 'input = "chip.json"'), encoding="utf-8")
    report = run_doctor(broken)
    assert "it is not UTF-8" in report.problems[0]
    assert report.text().endswith("doctor: 1 problem(s)")


def test_a_default_written_out_is_not_a_problem(tmp_path: Path) -> None:
    """The rules for a shipped file are for the benchmarks; a user may write a default out."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    written_out = tmp_path / "config.toml"
    written_out.write_text(
        config.replace(
            'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
        ).replace("capacity_cells_x = 12", "capacity_cells_x = 50"),
        encoding="utf-8",
    )
    report = run_doctor(written_out)
    assert report.ok, report.text()


def test_the_table_shows_the_bridge_pattern(tmp_path: Path) -> None:
    """A configuration with a bridge pattern shows it in the table, with the ports it matches."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    bridged = tmp_path / "bridged.toml"
    bridged.write_text(
        config.replace(
            'input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"'
        ).replace("[ports.patterns]\n", "[ports.patterns]\nbridge_pair = '^C\\d+\\.port[12]$'\n"),
        encoding="utf-8",
    )

    report = run_doctor(bridged)

    assert report.ok, report.text()
    assert "bridge_pair       0  ^C\\d+\\.port[12]$" in report.text()


def test_what_would_stop_the_configured_steps_is_a_problem(tmp_path: Path) -> None:
    """A planner that this build does not register would stop the capacity step."""
    config = (FIXTURE / "config.toml").read_text(encoding="utf-8")
    unknown = tmp_path / "unknown.toml"
    unknown.write_text(
        config.replace('input = "routing_config.json"', f'input = "{(FIXTURE / "routing_config.json").as_posix()}"')
        + '\n[stages.capacity]\nplanner = "other-planner"\n',
        encoding="utf-8",
    )

    report = run_doctor(unknown)
    assert report.problems == [
        "[stages.capacity] planner 'other-planner' is not a known implementation; the registered ones are watershed"
    ]
