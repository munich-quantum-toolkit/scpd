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

Beside each artifact a step writes a record of the inputs it read: a digest of the classified chip
and of every configuration section the step reads. A step refuses to run on an earlier artifact
whose inputs have changed since, so a resumed step never reads an artifact that the current
configuration would not make.

Python does the file work. The core never opens a file: it takes bytes and returns bytes.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from dataclasses import dataclass
from itertools import starmap
from typing import TYPE_CHECKING, Protocol

from . import pyscpd
from ._version import version as __version__
from .chip import RUN_CHIP, ChipError, chip_input_path, load_chip
from .config import load_config, write_config
from .console import Figure
from .flatbuffers.config.GridParams import GridParamsT
from .steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence
    from pathlib import Path

    from .flatbuffers.config.Config import ConfigT

__all__ = [
    "ARTIFACT_INPUTS",
    "STAGE_FILES",
    "STEP_INPUTS",
    "BlockReporter",
    "RunDirectory",
    "RunError",
    "StageResult",
    "input_digests",
    "steps_until",
]


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


#: The inputs each step reads besides the artifacts of the steps before it: the configuration
#: sections, named as in ``config.toml``, and the chip copy of the run as the configuration classifies it.
STEP_INPUTS: dict[str, tuple[str, ...]] = {
    "capacity": ("[ports]", "[design_rules]", "[grid]", "[stages.capacity]", RUN_CHIP),
    "global": ("[ports]", "[design_rules]", "[grid]", "[stages.global]", "[stages.solver]", RUN_CHIP),
    "assign": ("[ports]", "[design_rules]", "[grid]", "[stages.assignment]", "[stages.solver]", RUN_CHIP),
    "corridor": ("[ports]", "[design_rules]", "[grid]", "[stages.capacity]", "[stages.corridor]", RUN_CHIP),
}


#: The artifacts each step reads, in the order the core takes them.
ARTIFACT_INPUTS: dict[str, tuple[str, ...]] = {
    "capacity": (),
    "global": ("capacity",),
    "assign": ("capacity", "global"),
    "corridor": ("capacity", "assign"),
}


def _plain(value: object) -> object:
    """A value of the configuration schema as JSON data, with its fields in a fixed order.

    Returns:
        The value as dictionaries, lists, text and numbers.
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    if isinstance(value, list):
        return [_plain(item) for item in value]
    if hasattr(value, "__dict__"):
        return {key: _plain(item) for key, item in sorted(vars(value).items())}
    return value


def input_digests(config: ConfigT, chip: bytes) -> dict[str, str]:
    """The SHA-256 digest of every input a step can read.

    ``SCPD_SOLVER`` is no input: it chooses the backend of one invocation, and the run directory
    records only the files of the run. A step that resumes under another backend can therefore give
    another artifact, as ``docs/benchmarks.md`` says.

    Args:
        config: The configuration of the run.
        chip: The classified chip.

    Returns:
        The digest of each configuration section and of the chip, under the names of
        :data:`STEP_INPUTS`.
    """
    stages = config.stages
    sections: dict[str, object] = {
        "[ports]": config.ports,
        "[design_rules]": config.rules,
        # A configuration without [grid] takes the defaults, as one that states them does.
        "[grid]": config.grid if config.grid is not None else GridParamsT(),
        "[stages.capacity]": stages.capacity if stages is not None else None,
        "[stages.global]": stages.global_ if stages is not None else None,
        "[stages.assignment]": stages.assignment if stages is not None else None,
        "[stages.solver]": stages.solver if stages is not None else None,
        "[stages.corridor]": stages.corridor if stages is not None else None,
    }
    digests = {
        name: hashlib.sha256(json.dumps(_plain(section), sort_keys=True).encode("utf-8")).hexdigest()
        for name, section in sections.items()
    }
    digests[RUN_CHIP] = hashlib.sha256(chip).hexdigest()
    return digests


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

    def record(self, stage: str) -> Path:
        """The file in which a step records the inputs its artifact was made from.

        Args:
            stage: The step.

        Returns:
            The path, whether or not the file exists.
        """
        artifact = self.artifact(stage)
        return artifact.with_name(f"{artifact.stem}.inputs.json")

    @property
    def config(self) -> Path:
        """The copy of the configuration that the run uses."""
        return self.path / "config.toml"

    @property
    def chip(self) -> Path:
        """The copy of the chip input that the run uses."""
        return self.path / RUN_CHIP

    def inputs(self, config_path: Path, chip_path: Path | None = None) -> tuple[ConfigT, Path]:
        """Read the configuration of a new run and find its chip input, without changing the directory.

        Args:
            config_path: The configuration to run.
            chip_path: A chip input that replaces the configured one, as ``--chip`` gives it.
                Without one, the configuration of another run takes that run's chip copy, as
                :func:`~mqt.scpd.chip.chip_input_path` finds it.

        Returns:
            The configuration and the chip input.

        Raises:
            RunError: If the path is a file, or if the directory holds a config.toml of its own and
                no chip copy, so that it is no run directory.
            ChipError: If the chip input is no file.
        """
        config = load_config(config_path)
        chip = chip_input_path(config, config_path, chip_path)
        if self.path.exists() and not self.path.is_dir():
            msg = f"{self.path} is no run directory: it is a file"
            raise RunError(msg)
        if self.config.exists() and not self.chip.exists():
            msg = f"{self.path} is no run directory: it holds a config.toml of its own and no 00-chip.json"
            raise RunError(msg)
        if not chip.is_file():
            msg = f"cannot read the chip input {chip}: there is no such file"
            raise ChipError(msg)
        return config, chip

    def prepare(self, config_path: Path, chip_path: Path | None = None) -> ConfigT:
        """Create the directory, or start it afresh, and copy the inputs into it.

        Every artifact of an earlier run in the directory is deleted first, so no artifact stays
        beside inputs it was not made from.

        Args:
            config_path: The configuration to run.
            chip_path: A chip input that replaces the configured one, as for :meth:`inputs`.

        Returns:
            The configuration.
        """
        config, chip = self.inputs(config_path, chip_path)
        self.path.mkdir(parents=True, exist_ok=True)
        for stage in STAGE_FILES:
            for path in (self.artifact(stage), self.record(stage)):
                path.unlink(missing_ok=True)
                path.with_name(f"{path.name}.tmp").unlink(missing_ok=True)
        # The chip comes first: a directory that holds a chip copy and no configuration is never
        # taken for a directory of somebody else's configuration. An input that already is the copy
        # of the run, as when a run is started again from its own files, stays where it is.
        for source, copy in ((chip, self.chip), (config_path, self.config)):
            if not (copy.exists() and copy.samefile(source)):
                shutil.copyfile(source, copy)
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
            RunError: If the name is no step of this release, if an input of the step is missing, or
                if an input of an earlier step has changed since that step wrote its artifact. A
                refused step leaves the directory as it was.
        """
        path = self.artifact(stage)
        if stage not in ARTIFACT_INPUTS:
            msg = f"no core call runs the {stage} step"
            raise RunError(msg)
        digests = input_digests(config, chip)
        self._check_unchanged(stage, digests)
        # Each step reads the artifacts of the steps before it, and a missing one stops the step
        # before anything is deleted.
        inputs = [self._read(earlier) for earlier in ARTIFACT_INPUTS[stage]]
        packed = write_config(config)
        producer = f"mqt-scpd {__version__}"
        level = min(max(verbosity, 0), 3)
        calls: dict[str, Callable[[], bytes]] = {
            "capacity": lambda: pyscpd.plan_capacity(chip, packed, producer, reporter, level),
            "global": lambda: pyscpd.route_global(chip, inputs[0], packed, producer, reporter, level),
            "assign": lambda: pyscpd.assign(chip, inputs[0], inputs[1], packed, producer, reporter, level),
            "corridor": lambda: pyscpd.route_corridor(chip, inputs[0], inputs[1], packed, producer, reporter, level),
        }
        if stage not in calls:
            msg = f"no core call runs the {stage} step"
            raise RunError(msg)
        # The artifacts of the later steps go before the core runs, so that a step that fails leaves
        # none of them beside an input they were not made from.
        self.invalidate_after(stage)
        self.record(stage).unlink(missing_ok=True)
        data = calls[stage]()
        _write_whole(path, data)
        # The record comes after the artifact, so an artifact never carries the record of other inputs.
        record = {name: digests[name] for name in STEP_INPUTS[stage]}
        _write_whole(self.record(stage), (json.dumps(record, indent=2) + "\n").encode("utf-8"))
        return StageResult(stage=stage, path=path, size=len(data))

    def invalidate_after(self, stage: str) -> list[Path]:
        """Delete the artifact and the record of every step after one.

        Args:
            stage: The step whose later steps lose their artifacts.

        Returns:
            The files that were removed.
        """
        names = list(STAGE_FILES)
        removed = []
        for later in names[names.index(stage) + 1 :]:
            for path in (self.artifact(later), self.record(later)):
                if path.is_file():
                    path.unlink()
                    removed.append(path)
        return removed

    def check_current(self, stage: str, digests: dict[str, str]) -> None:
        """Refuse the artifact of a step, and every earlier artifact, when an input changed since.

        A picture of a step draws its artifact over the earlier ones, so each of them has to come
        from the inputs the run holds now. A step that failed keeps its old artifact without a
        record, and that artifact is refused as well.

        Args:
            stage: The step.
            digests: The digests of the inputs the run holds now, as :func:`input_digests` gives them.

        Raises:
            RunError: If one of the artifacts has no readable record, or if an input it records has
                changed.
        """  # ruff: ignore[docstring-extraneous-exception]  # _check_records() raises it
        names = list(STAGE_FILES)
        self._check_records(names[: names.index(stage) + 1], digests)

    def _check_unchanged(self, stage: str, digests: dict[str, str]) -> None:
        """Refuse a step when an earlier artifact was made from inputs that have changed since.

        Raises:
            RunError: If an earlier artifact has no readable record, or if an input it records has
                changed.
        """  # ruff: ignore[docstring-extraneous-exception]  # _check_records() raises it
        names = list(STAGE_FILES)
        self._check_records(names[: names.index(stage)], digests)

    def _check_records(self, stages: Sequence[str], digests: dict[str, str]) -> None:
        """Refuse artifacts made from inputs that have changed since.

        A missing artifact is left to the caller, which names the step to run first.

        Raises:
            RunError: If an artifact has no readable record, or if an input it records has changed.
        """
        for earlier in stages:
            artifact = self.artifact(earlier)
            if not artifact.is_file():
                continue
            try:
                record = json.loads(self.record(earlier).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                record = None
            if not isinstance(record, dict) or set(record) != set(STEP_INPUTS[earlier]):
                msg = f"{artifact.name} carries no record of the inputs it was made from; run the {earlier} step again"
                raise RunError(msg)
            for name in STEP_INPUTS[earlier]:
                if record[name] != digests[name]:
                    where = name if name == RUN_CHIP else f"{name} in config.toml"
                    msg = (
                        f"{where} changed after the {earlier} step wrote {artifact.name}; run the {earlier} step again"
                    )
                    raise RunError(msg)

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
