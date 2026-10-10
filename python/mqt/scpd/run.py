# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The run directory: what a step reads, what it writes, and how a run is resumed.

Every step reads the artifacts of the steps before it and writes one artifact, so a run can be
continued from any completed step. Running a step deletes the artifacts of every later step, so
the directory never holds a later artifact beside an earlier one it did not come from. An artifact
is written to a file of its own first and then renamed, so no half-written artifact is left behind.

Python does the file work. The core never opens a file: it takes bytes and returns bytes.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from itertools import starmap
from typing import TYPE_CHECKING, Protocol

from . import pyscpd
from ._version import version as __version__
from .chip import ChipError, chip_input_path, load_chip
from .config import load_config, write_config
from .console import Figure
from .steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

    from .flatbuffers.config.Config import ConfigT

__all__ = ["STAGE_FILES", "BlockReporter", "RunDirectory", "RunError", "StageResult", "steps_until"]


class RunError(RuntimeError):
    """A run directory that cannot be read, or a step whose input is missing."""


#: The artifact each step writes, in the order the steps run. The number is that order, so a listing
#: of the directory reads like the pipeline.
STAGE_FILES: dict[str, str] = {
    "capacity": "01-capacity.fb",
    "global": "02-global.fb",
    "assign": "03-assign.fb",
    "corridor": "04-corridor.fb",
}


def steps_until(stop_after: str | None) -> list[str]:
    """The steps a run takes.

    Args:
        stop_after: The last step to take; None or an empty name takes every step.

    Returns:
        The steps, in order.

    Raises:
        RunError: If the name is no step of this release.
    """
    if not stop_after:
        return list(STEPS)
    if stop_after not in STEPS:
        msg = f"'{stop_after}' is no step of this release; the steps are {', '.join(STEPS)}"
        raise RunError(msg)
    return list(STEPS[: STEPS.index(stop_after) + 1])


class StageBlockLike(Protocol):
    """What a stage reports into: the block of the stage on the console."""

    def line(self, level: int, text: str) -> None:
        """Take a line of free text."""

    def entry(self, level: int, label: str, figures: Sequence[Figure]) -> None:
        """Take an entry, such as one round."""

    def result(self, figures: Sequence[Figure], fails: int | None = None) -> None:
        """Take the result of the stage."""

    def progress(self, task: str, detail: str, done: int, total: int) -> None:
        """Take how far the stage has got."""


class BlockReporter:
    """Hands what the core reports to a stage block.

    The core calls the methods below with figures as (text, tone) pairs; the block takes
    :class:`~mqt.scpd.console.Figure` values.
    """

    def __init__(self, block: StageBlockLike) -> None:
        """Report into a block.

        Args:
            block: The block of the stage.
        """
        self.block = block

    def line(self, level: int, text: str) -> None:
        """Hand a line to the block."""
        self.block.line(level, text)

    def entry(self, level: int, label: str, figures: Sequence[tuple[str, str]]) -> None:
        """Hand an entry to the block."""
        self.block.entry(level, label, list(starmap(Figure, figures)))

    def result(self, figures: Sequence[tuple[str, str]], fails: int | None) -> None:
        """Hand the result to the block."""
        self.block.result(list(starmap(Figure, figures)), fails)

    def progress(self, task: str, detail: str, done: int, total: int) -> None:
        """Hand the progress to the block."""
        self.block.progress(task, detail, done, total)


@dataclass(frozen=True)
class StageResult:
    """What running one step wrote."""

    stage: str
    path: Path
    size: int


class RunDirectory:
    """The directory of one run, and the steps that fill it."""

    def __init__(self, path: Path) -> None:
        """Use a directory, which need not exist yet.

        Args:
            path: The directory.
        """
        self.path = path

    def artifact(self, stage: str) -> Path:
        """The file a step writes.

        Args:
            stage: The step.

        Returns:
            The path, whether or not the file exists.

        Raises:
            RunError: If the name is no step of this release.
        """
        if stage not in STAGE_FILES:
            msg = f"'{stage}' is no step of this release; the steps are {', '.join(STEPS)}"
            raise RunError(msg)
        return self.path / STAGE_FILES[stage]

    @property
    def config(self) -> Path:
        """The copy of the configuration that the run uses."""
        return self.path / "config.toml"

    @property
    def chip(self) -> Path:
        """The copy of the chip input that the run uses."""
        return self.path / "00-chip.json"

    def prepare(self, config_path: Path, chip_path: Path | None = None) -> ConfigT:
        """Create the directory, or start it afresh, and copy the inputs into it.

        Every artifact of an earlier run in the directory is deleted first, so no artifact stays
        beside inputs it was not made from.

        Args:
            config_path: The configuration to run.
            chip_path: A chip input that replaces the configured one, as ``--chip`` gives it.

        Returns:
            The configuration.

        Raises:
            RunError: If the directory holds a config.toml of its own and no chip copy, so that it
                is no run directory.
            ChipError: If the chip input is no file.
        """
        config = load_config(config_path)
        chip = chip_input_path(config, config_path, chip_path)
        if self.config.exists() and not self.chip.exists():
            msg = f"{self.path} is no run directory: it holds a config.toml of its own and no 00-chip.json"
            raise RunError(msg)
        if not chip.is_file():
            msg = f"cannot read the chip input {chip}: there is no such file"
            raise ChipError(msg)
        self.path.mkdir(parents=True, exist_ok=True)
        for name in STAGE_FILES.values():
            (self.path / name).unlink(missing_ok=True)
            (self.path / f"{name}.tmp").unlink(missing_ok=True)
        # The chip comes first: a directory that holds a chip copy and no configuration is never
        # taken for a directory of somebody else's configuration.
        shutil.copyfile(chip, self.chip)
        shutil.copyfile(config_path, self.config)
        return config

    def load(self) -> ConfigT:
        """Read the configuration of an existing run.

        Returns:
            The configuration.

        Raises:
            RunError: If the directory carries no configuration.
        """
        if not self.config.is_file():
            msg = f"{self.path} carries no config.toml; it is not a run directory"
            raise RunError(msg)
        return load_config(self.config)

    def classified_chip(self, config: ConfigT) -> bytes:
        """Classify the chip copy of the run with its configuration.

        Args:
            config: The configuration of the run.

        Returns:
            The classified chip, as the core takes it.
        """
        return load_chip(config, self.config, self.chip)

    def run_stage(
        self,
        stage: str,
        config: ConfigT,
        chip: bytes,
        reporter: BlockReporter | None = None,
        verbosity: int = 0,
    ) -> StageResult:
        """Run one step and write its artifact.

        Args:
            stage: The step.
            config: The configuration of the run.
            chip: The classified chip.
            reporter: Where the step reports what it does, or None.
            verbosity: How many times -v was given.

        Returns:
            What the step wrote.

        Raises:
            RunError: If the name is no step of this release, or an input of the step is missing.
        """  # ruff: ignore[docstring-extraneous-exception]  # artifact() and _read() raise it
        path = self.artifact(stage)
        packed = write_config(config)
        producer = f"mqt-scpd {__version__}"
        level = min(max(verbosity, 0), 3)
        # Each step reads the artifacts of the steps before it, and a missing one stops the step
        # before the core runs.
        if stage == "capacity":
            data = pyscpd.plan_capacity(chip, packed, producer, reporter, level)
        elif stage == "global":
            data = pyscpd.route_global(chip, self._read("capacity"), packed, producer, reporter, level)
        elif stage == "assign":
            capacity, routing = self._read("capacity"), self._read("global")
            data = pyscpd.assign(chip, capacity, routing, packed, producer, reporter, level)
        else:
            capacity, assignment = self._read("capacity"), self._read("assign")
            data = pyscpd.route_corridor(chip, capacity, assignment, packed, producer, reporter, level)
        _write_whole(path, data)
        self.invalidate_after(stage)
        return StageResult(stage=stage, path=path, size=len(data))

    def invalidate_after(self, stage: str) -> list[Path]:
        """Delete the artifact of every step after one.

        Args:
            stage: The step that was just run.

        Returns:
            The files that were removed.
        """
        names = list(STAGE_FILES)
        removed = []
        for later in names[names.index(stage) + 1 :]:
            path = self.artifact(later)
            if path.is_file():
                path.unlink()
                removed.append(path)
        return removed

    def _read(self, stage: str) -> bytes:
        path = self.artifact(stage)
        if not path.is_file():
            msg = f"{path.name} is missing; run the {stage} step first"
            raise RunError(msg)
        return path.read_bytes()


def _write_whole(path: Path, data: bytes) -> None:
    """Write a file under its own name only once all of it is written."""
    partial = path.with_name(path.name + ".tmp")
    try:
        partial.write_bytes(data)
        partial.replace(path)
    finally:
        partial.unlink(missing_ok=True)
