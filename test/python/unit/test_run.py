# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the run directory: the order of the steps, what a stage reads and writes, and resuming."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd import pyscpd
from mqt.scpd.console import Figure
from mqt.scpd.run import (
    ARTIFACT_INPUTS,
    STAGE_FILES,
    STEP_INPUTS,
    BlockReporter,
    RunDirectory,
    RunError,
    steps_until,
)
from mqt.scpd.steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Sequence

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "mini"
CONFIG = FIXTURE / "config.toml"


class Recorder:
    """A stage block that keeps what it is handed."""

    def __init__(self) -> None:
        """Start with nothing kept."""
        self.lines: list[tuple[int, str]] = []
        self.entries: list[tuple[int, str, list[Figure]]] = []
        self.results: list[tuple[list[Figure], int | None]] = []
        self.tasks: list[str] = []

    def line(self, level: int, text: str) -> None:
        """Keep a line."""
        self.lines.append((level, text))

    def entry(self, level: int, label: str, figures: Sequence[Figure]) -> None:
        """Keep an entry."""
        self.entries.append((level, label, list(figures)))

    def result(self, figures: Sequence[Figure], fails: int | None = None) -> None:
        """Keep the result."""
        self.results.append((list(figures), fails))

    def progress(self, task: str, detail: str, done: int, total: int) -> None:
        """Keep the task of the progress."""
        del detail, done, total
        self.tasks.append(task)


def run_through(directory: RunDirectory, steps: list[str]) -> None:
    """Run the steps in order in a prepared directory."""
    config = directory.load()
    chip = directory.classified_chip(config)
    for step in steps:
        directory.run_stage(step, config, chip)


@pytest.mark.parametrize(
    ("stop_after", "expected"),
    [
        (None, list(STEPS)),
        ("", list(STEPS)),
        ("capacity", ["capacity"]),
        ("assign", ["capacity", "global", "assign"]),
    ],
    ids=["none", "empty", "first", "middle"],
)
def test_a_run_takes_the_steps_up_to_its_stop(stop_after: str | None, expected: list[str]) -> None:
    """A run takes every step up to and including the one it stops after."""
    assert steps_until(stop_after) == expected


def test_a_stop_that_is_no_step_is_refused() -> None:
    """A step name the release does not know names the ones it does."""
    with pytest.raises(RunError, match="capacity, global, assign, corridor"):
        steps_until("outer")


def test_preparing_copies_the_inputs_into_the_run(tmp_path: Path) -> None:
    """The run keeps its own configuration and chip, so a later step reads them from there."""
    elsewhere = tmp_path / "inputs"
    elsewhere.mkdir()
    chip = elsewhere / "chip.json"
    shutil.copyfile(FIXTURE / "routing_config.json", chip)
    directory = RunDirectory(tmp_path / "run")

    directory.prepare(CONFIG, chip)

    assert directory.config.read_bytes() == CONFIG.read_bytes()
    assert directory.chip.read_bytes() == chip.read_bytes()


def test_preparing_again_starts_the_run_afresh(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A run prepared again keeps no artifact of the last one, even when its first step fails."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, list(STEPS))
    (directory.path / "01-capacity.fb.tmp").write_bytes(b"partial")

    def broken(*args: object) -> bytes:
        del args
        msg = "the capacity stage broke"
        raise RuntimeError(msg)

    monkeypatch.setattr(pyscpd, "plan_capacity", broken)
    directory.prepare(CONFIG)
    with pytest.raises(RuntimeError, match="broke"):
        run_through(directory, ["capacity"])

    assert sorted(path.name for path in directory.path.iterdir()) == ["00-chip.json", "config.toml"]


def test_a_run_starts_again_from_its_own_inputs(tmp_path: Path) -> None:
    """A run prepared from the copies it carries keeps them and starts afresh."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity"])

    directory.prepare(directory.config, directory.chip)

    assert directory.config.read_bytes() == CONFIG.read_bytes()
    assert directory.chip.read_bytes() == (FIXTURE / "routing_config.json").read_bytes()
    assert sorted(path.name for path in directory.path.iterdir()) == ["00-chip.json", "config.toml"]


def test_a_run_started_from_its_own_configuration_keeps_its_chip_copy(tmp_path: Path) -> None:
    """A run started from its own config.toml without a chip input takes its chip copy.

    The copied configuration names the chip input relative to the original configuration, which
    does not resolve inside the run directory.
    """
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)

    directory.prepare(directory.config)

    assert directory.chip.read_bytes() == (FIXTURE / "routing_config.json").read_bytes()
    assert sorted(path.name for path in directory.path.iterdir()) == ["00-chip.json", "config.toml"]


def test_preparing_refuses_a_directory_that_holds_a_configuration_of_its_own(tmp_path: Path) -> None:
    """A directory with a config.toml but without the chip copy of a run is not a run, and stays as it was."""
    foreign = tmp_path / "chip"
    foreign.mkdir()
    (foreign / "config.toml").write_text("# a configuration of its own\n", encoding="utf-8")

    with pytest.raises(RunError, match="is not a run directory"):
        RunDirectory(foreign).prepare(CONFIG)

    assert (foreign / "config.toml").read_text(encoding="utf-8") == "# a configuration of its own\n"
    with pytest.raises(RunError, match="is not a run directory"):
        RunDirectory(FIXTURE).prepare(CONFIG)


def test_a_step_writes_its_artifact_and_removes_the_later_ones(tmp_path: Path) -> None:
    """Running a step again leaves no artifact behind that came from the old one."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity", "global"])
    assert directory.artifact("global").is_file()

    run_through(directory, ["capacity"])

    assert directory.artifact("capacity").is_file()
    assert not directory.artifact("global").exists()
    assert sorted(path.name for path in directory.path.iterdir()) == [
        "00-chip.json",
        "01-capacity.fb",
        "01-capacity.inputs.json",
        "config.toml",
    ]


def test_a_step_without_its_input_is_refused(tmp_path: Path) -> None:
    """A step whose input is missing says which step to run first."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)

    with pytest.raises(RunError, match=r"02-global\.fb is missing; run the global step first"):
        run_through(directory, ["capacity", "assign"])


def test_a_step_without_its_input_leaves_the_directory_as_it_was(tmp_path: Path) -> None:
    """A refused step deletes no artifact of a later step and no record."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity", "global", "assign", "corridor"])
    directory.artifact("capacity").unlink()
    before = artifacts_of(directory)

    with pytest.raises(RunError, match=r"01-capacity\.fb is missing; run the capacity step first"):
        run_through(directory, ["global"])

    assert artifacts_of(directory) == before


def test_a_resumed_run_equals_an_uninterrupted_one(tmp_path: Path) -> None:
    """A run continued step by step from its own directory writes the same bytes."""
    whole = RunDirectory(tmp_path / "whole")
    whole.prepare(CONFIG)
    run_through(whole, list(STEPS))

    resumed = RunDirectory(tmp_path / "resumed")
    resumed.prepare(CONFIG)
    run_through(resumed, ["capacity", "global"])
    for step in ("assign", "corridor"):
        run_through(RunDirectory(tmp_path / "resumed"), [step])

    for stage, name in STAGE_FILES.items():
        assert (whole.path / name).read_bytes() == (resumed.path / name).read_bytes(), stage


def edit_config(directory: RunDirectory, old: str, new: str) -> None:
    """Replace one line of the configuration a run carries."""
    text = directory.config.read_text(encoding="utf-8")
    assert old in text
    directory.config.write_text(text.replace(old, new), encoding="utf-8")


def artifacts_of(directory: RunDirectory) -> dict[str, bytes]:
    """The artifacts and records of a run, by file name.

    Returns:
        The bytes of every file a step wrote.
    """
    return {path.name: path.read_bytes() for path in directory.path.glob("0[1-4]-*")}


@pytest.mark.parametrize(
    ("old", "new", "message"),
    [
        (
            "min_straight_length = 100.0",
            "min_straight_length = 300.0",
            (
                r"\[design_rules\] in config\.toml changed after the capacity step wrote 01-capacity\.fb; "
                r"run the capacity step again"
            ),
        ),
        (
            "capacity_cells_x = 12",
            "capacity_cells_x = 13",
            r"\[grid\] in config\.toml changed after the capacity step",
        ),
        (
            "[stages.assignment]",
            "[stages.solver]\nthreads = 1\n\n[stages.assignment]",
            (
                r"\[stages\.solver\] in config\.toml changed after the global step wrote 02-global\.fb; "
                r"run the global step again"
            ),
        ),
        (
            "[stages.assignment]",
            "[stages.capacity]\ncrossing_pitch = 170.0\n\n[stages.assignment]",
            r"\[stages\.capacity\] in config\.toml changed after the capacity step",
        ),
    ],
    ids=["design-rule", "grid", "solver", "pitch"],
)
def test_a_step_refuses_an_earlier_artifact_made_from_other_sections(
    tmp_path: Path, old: str, new: str, message: str
) -> None:
    """A section an earlier step read and that has changed since stops the step, and the run stays as it was."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, list(STEPS))
    before = artifacts_of(directory)
    edit_config(directory, old, new)

    with pytest.raises(RunError, match=message):
        run_through(directory, ["corridor"])

    assert artifacts_of(directory) == before


def test_every_step_has_an_artifact_and_a_record_of_its_inputs() -> None:
    """A step of the release that the run directory does not know could not run or be resumed."""
    assert list(STAGE_FILES) == list(STEPS)
    assert list(STEP_INPUTS) == list(STEPS)
    assert list(ARTIFACT_INPUTS) == list(STEPS)


def test_a_step_without_a_core_call_deletes_nothing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A step that the run directory knows but no core call runs is refused before any artifact goes."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, list(STEPS))
    before = artifacts_of(directory)
    monkeypatch.setitem(STAGE_FILES, "later", "05-later.fb")
    monkeypatch.setitem(STEP_INPUTS, "later", STEP_INPUTS["corridor"])
    monkeypatch.setitem(ARTIFACT_INPUTS, "later", ARTIFACT_INPUTS["corridor"])

    with pytest.raises(RunError, match="no core call runs the later step"):
        run_through(directory, ["later"])

    assert artifacts_of(directory) == before


def test_a_step_refuses_an_earlier_artifact_made_from_another_chip(tmp_path: Path) -> None:
    """A chip copy edited after a step stops the steps after it."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity"])
    chip = json.loads(directory.chip.read_text(encoding="utf-8"))
    chip["obstacles"].append(chip["obstacles"][0])
    directory.chip.write_text(json.dumps(chip), encoding="utf-8")

    with pytest.raises(RunError, match=r"00-chip\.json changed after the capacity step wrote 01-capacity\.fb"):
        run_through(directory, ["global"])


def test_a_step_reruns_after_its_own_section_changed(tmp_path: Path) -> None:
    """A change to a section that only the resumed step reads is the reason to run that step again."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, list(STEPS))
    edit_config(directory, "[stages.assignment]", "[stages.corridor]\nrounds = 2\n\n[stages.assignment]")

    run_through(directory, ["corridor"])

    assert directory.artifact("corridor").is_file()


def test_a_step_resumes_under_another_solver_backend(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """``SCPD_SOLVER`` chooses the backend of one invocation and is not an input that a record holds."""
    monkeypatch.delenv("SCPD_SOLVER", raising=False)
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity", "global"])
    recorded = directory.record("global").read_bytes()
    monkeypatch.setenv("SCPD_SOLVER", "highs")

    run_through(directory, ["assign"])

    assert directory.record("global").read_bytes() == recorded
    record = json.loads(directory.record("assign").read_text(encoding="utf-8"))
    assert sorted(record) == sorted(STEP_INPUTS["assign"])


@pytest.mark.parametrize("record", [None, "not json", "{}"], ids=["missing", "unreadable", "incomplete"])
def test_a_step_refuses_an_earlier_artifact_without_a_record(tmp_path: Path, record: str | None) -> None:
    """An artifact whose record of inputs is missing or broken is not an input of a later step."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity"])
    if record is None:
        directory.record("capacity").unlink()
    else:
        directory.record("capacity").write_text(record, encoding="utf-8")

    with pytest.raises(
        RunError, match=r"01-capacity\.fb carries no record of the inputs it was made from; run the capacity step again"
    ):
        run_through(directory, ["global"])


def test_a_step_that_fails_keeps_the_old_artifact(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A step that raises writes nothing, and the artifact of its last run stays."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity"])
    before = directory.artifact("capacity").read_bytes()

    def broken(*args: object) -> bytes:
        del args
        msg = "the capacity stage broke"
        raise RuntimeError(msg)

    monkeypatch.setattr(pyscpd, "plan_capacity", broken)
    with pytest.raises(RuntimeError, match="broke"):
        run_through(directory, ["capacity"])

    assert directory.artifact("capacity").read_bytes() == before
    assert not list(directory.path.glob("*.tmp"))


def test_a_step_that_fails_removes_the_later_artifacts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A step that raises leaves no artifact of a later step, which came from the step's old artifact."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, list(STEPS))

    def broken(*args: object) -> bytes:
        del args
        msg = "the global stage broke"
        raise RuntimeError(msg)

    monkeypatch.setattr(pyscpd, "route_global", broken)
    with pytest.raises(RuntimeError, match="broke"):
        run_through(directory, ["global"])

    assert sorted(path.name for path in directory.path.glob("*.fb")) == ["01-capacity.fb", "02-global.fb"]


def test_an_artifact_is_replaced_whole_or_not_at_all(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A write that breaks off halfway leaves the old artifact and no partial file."""
    directory = RunDirectory(tmp_path / "run")
    directory.prepare(CONFIG)
    run_through(directory, ["capacity"])
    before = directory.artifact("capacity").read_bytes()
    write_bytes = Path.write_bytes

    def half(self: Path, data: bytes) -> int:
        write_bytes(self, data[: len(data) // 2])
        msg = "the disk is full"
        raise OSError(msg)

    monkeypatch.setattr(Path, "write_bytes", half)
    with pytest.raises(OSError, match="the disk is full"):
        run_through(directory, ["capacity"])
    monkeypatch.undo()

    assert directory.artifact("capacity").read_bytes() == before
    assert not list(directory.path.glob("*.tmp"))


def test_a_block_receives_figures_it_can_lay_out(tmp_path: Path) -> None:
    """What the core reports as (text, tone) pairs reaches the block as figures."""
    directory = RunDirectory(tmp_path / "run")
    config = directory.prepare(CONFIG)
    chip = directory.classified_chip(config)
    block = Recorder()

    result = directory.run_stage("capacity", config, chip, BlockReporter(block), verbosity=1)

    assert result.path == directory.artifact("capacity")
    assert result.size == result.path.stat().st_size
    assert len(block.results) == 1
    figures, fails = block.results[0]
    assert all(isinstance(figure, Figure) for figure in figures)
    assert figures[0].text.endswith("partitions")
    assert fails is None
    assert any(level == 1 for level, _ in block.lines)
    assert block.tasks
