# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The run directory: what it holds, what it refuses, and what resuming a stage invalidates."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from mqt.scpd.run import IMPLEMENTED, STAGE_FILES, RunDirectory, RunError, stages_before

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"


def test_the_stage_order_puts_global_before_the_assignment() -> None:
    """Which outer ports the inner circuit surfaces at is what the assignment consumes."""
    assert IMPLEMENTED == ("capacity", "global", "assign")
    assert list(STAGE_FILES)[:3] == ["capacity", "global", "assign"]
    assert STAGE_FILES["global"] == "02-global.fb"
    assert STAGE_FILES["assign"] == "03-assign.fb"


def test_a_stage_names_the_stages_it_reads() -> None:
    """The stages before one are the ones whose artifacts it needs."""
    assert stages_before("capacity") == []
    assert stages_before("global") == ["capacity"]
    assert stages_before("assign") == ["capacity", "global"]


def test_an_unimplemented_stage_says_which_ones_run() -> None:
    """A stage of a later phase is refused with the list of the ones that exist."""
    with pytest.raises(RunError, match="capacity, global, assign"):
        stages_before("detail")


def test_an_unknown_stage_has_no_artifact(tmp_path: Path) -> None:
    """A name that is no stage at all names no file."""
    with pytest.raises(RunError, match="no stage of the pipeline"):
        RunDirectory(tmp_path).artifact("nonsense")


def test_preparing_copies_both_inputs(tmp_path: Path) -> None:
    """A run carries its own copy of the configuration and the chip, for provenance."""
    run = RunDirectory(tmp_path / "run")
    config = run.prepare(BENCHMARKS / "4q" / "config.toml")

    assert config.chipInput == "routing_config.json"
    assert run.config.is_file()
    assert run.chip.is_file()
    assert run.chip.name == "00-chip.json"
    assert run.chip.read_text(encoding="utf-8") == (BENCHMARKS / "4q" / "routing_config.json").read_text(
        encoding="utf-8"
    )


def test_loading_needs_a_configuration(tmp_path: Path) -> None:
    """A directory without a config.toml is not a run directory."""
    tmp_path.mkdir(exist_ok=True)
    with pytest.raises(RunError, match="not a run directory"):
        RunDirectory(tmp_path).load()


def test_running_a_stage_invalidates_everything_after_it(tmp_path: Path) -> None:
    """The directory is left in one state rather than a mixed one."""
    run = RunDirectory(tmp_path)
    tmp_path.mkdir(exist_ok=True)
    for stage in STAGE_FILES:
        run.artifact(stage).write_bytes(b"stale")

    removed = run.invalidate_after("global")

    assert {path.name for path in removed} == {"03-assign.fb", "04-detail.fb", "05-final.fb", "06-geometry.fb"}
    assert run.artifact("capacity").is_file()
    assert run.artifact("global").is_file()
    assert not run.artifact("assign").is_file()


def test_a_stage_says_which_artifact_it_is_missing(tmp_path: Path) -> None:
    """Resuming without the artifact a stage reads names the stage to run first."""
    run = RunDirectory(tmp_path / "run")
    config = run.prepare(BENCHMARKS / "4q" / "config.toml")

    with pytest.raises(RunError, match="01-capacity.fb is missing; run the capacity stage first"):
        run.run_stage("global", config)


def test_a_run_without_its_chip_copy_is_incomplete(tmp_path: Path) -> None:
    """The chip is read from the run's own copy, so a run stays reproducible after its inputs move."""
    run = RunDirectory(tmp_path / "run")
    config = run.prepare(BENCHMARKS / "4q" / "config.toml")
    run.chip.unlink()

    with pytest.raises(RunError, match="the run directory is incomplete"):
        run.run_stage("capacity", config)


@pytest.mark.parametrize("chip", ["4q", "9q"])
def test_the_planning_stages_fill_a_run_directory(tmp_path: Path, chip: str) -> None:
    """All three stages run on the two chip inputs the repository carries."""
    run = RunDirectory(tmp_path / "run")
    config = run.prepare(BENCHMARKS / chip / "config.toml")

    for stage in IMPLEMENTED:
        result = run.run_stage(stage, config)
        assert result.path.is_file()
        assert result.size > 0
    assert {path.name for path in (tmp_path / "run").iterdir()} == {
        "config.toml",
        "00-chip.json",
        "01-capacity.fb",
        "02-global.fb",
        "03-assign.fb",
    }


def test_resuming_a_stage_reproduces_the_same_artifact(tmp_path: Path) -> None:
    """A stage is deterministic, which is what makes a resumed run equal to an uninterrupted one."""
    run = RunDirectory(tmp_path / "run")
    config = run.prepare(BENCHMARKS / "4q" / "config.toml")
    for stage in IMPLEMENTED:
        run.run_stage(stage, config)

    first = run.artifact("assign").read_bytes()
    kept = tmp_path / "kept.fb"
    shutil.copyfile(run.artifact("assign"), kept)

    run.run_stage("assign", config)

    assert run.artifact("assign").read_bytes() == first
    assert kept.read_bytes() == first
