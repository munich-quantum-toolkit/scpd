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

Beside each artifact a step writes a record of the inputs it read: a digest of the classified chip,
of every configuration section the step reads, and of the artifact itself. A step refuses to run
on an earlier artifact whose bytes or inputs do not match its record.

Python does the file work. The core never opens a file: it takes bytes and returns bytes.
"""

from __future__ import annotations

import hashlib
import json
import os
import stat
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from itertools import starmap
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

if os.name == "nt":
    import msvcrt
else:
    import fcntl

from . import pyscpd
from ._version import version as __version__
from .chip import RUN_CHIP, chip_input_path, load_chip, read_chip_text
from .config import load_config, parse_config, read_config_text, write_config
from .console import Figure
from .flatbuffers.config.GridParams import GridParamsT
from .steps import STEPS

if TYPE_CHECKING:
    from collections.abc import Callable, Generator, Sequence

    from .flatbuffers.config.Config import ConfigT

__all__ = [
    "ARTIFACT_INPUTS",
    "STAGE_FILES",
    "STEP_INPUTS",
    "BlockReporter",
    "RunDirectory",
    "RunError",
    "RunInputs",
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
}


#: The inputs each step reads besides the artifacts of the steps before it: the configuration
#: sections, named as in ``config.toml``, and the chip copy of the run as the configuration classifies it.
STEP_INPUTS: dict[str, tuple[str, ...]] = {
    "capacity": ("[ports]", "[design_rules]", "[grid]", "[stages.capacity]", RUN_CHIP),
}


#: The artifacts each step reads, in the order the core takes them.
ARTIFACT_INPUTS: dict[str, tuple[str, ...]] = {
    "capacity": (),
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
        RunError: If the name is not a step of this release.
    """
    if not stop_after:
        return list(STEPS)
    if stop_after not in STEPS:
        msg = f"'{stop_after}' is not a step of this release; the steps are {', '.join(STEPS)}"
        raise RunError(msg)
    return list(STEPS[: STEPS.index(stop_after) + 1])


class StageBlockLike(Protocol):
    """What a stage reports into: the block of the stage on the console."""

    def line(self, level: int, text: str) -> None:
        """Take a line of free text."""

    def entry(self, level: int, label: str, figures: Sequence[Figure]) -> None:
        """Take an entry, such as one chain."""

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


@dataclass(frozen=True)
class RunInputs:
    """The exact inputs of a fresh run."""

    config: ConfigT
    config_text: str
    chip_path: Path
    chip_text: str


_RUN_MARKER = b"MQT SCPD run directory\n"
_ARTIFACT_DIGEST = "artifact_sha256"


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
            RunError: If the name is not a step of this release.
        """
        if stage not in STAGE_FILES:
            msg = f"'{stage}' is not a step of this release; the steps are {', '.join(STEPS)}"
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

    @property
    def marker(self) -> Path:
        """The file that identifies a directory created by MQT SCPD."""
        return self.path / ".mqt-scpd-run"

    def _check_destination(self) -> None:
        """Refuse to write into a directory that does not belong to a run.

        Raises:
            RunError: If the destination is a file, link, or unrecognized non-empty directory.
        """
        try:
            entry = self.path.lstat()
        except FileNotFoundError:
            return
        junction = os.name == "nt" and entry.st_reparse_tag == stat.IO_REPARSE_TAG_MOUNT_POINT
        if stat.S_ISLNK(entry.st_mode) or junction:
            msg = f"{self.path} is not a run directory: it is a directory link"
            raise RunError(msg)
        if not stat.S_ISDIR(entry.st_mode):
            msg = f"{self.path} is not a run directory: it is a file"
            raise RunError(msg)
        if any(self.path.iterdir()) and not self._owned():
            msg = f"{self.path} is not a run directory: it holds files without a valid {self.marker.name} marker"
            raise RunError(msg)

    def _owned(self) -> bool:
        """Check the run marker before modifying existing files.

        Returns:
            Whether the directory has a regular marker with the expected contents.
        """
        try:
            return not self.marker.is_symlink() and self.marker.is_file() and self.marker.read_bytes() == _RUN_MARKER
        except OSError:
            return False

    def inputs(self, config_path: Path, chip_path: Path | None = None) -> RunInputs:
        """Read the exact inputs for a fresh run without changing its destination.

        Args:
            config_path: The configuration to run.
            chip_path: An optional replacement for the configured chip input.

        Returns:
            The parsed configuration and the source texts to classify and publish.
        """
        self._check_destination()
        config_text = read_config_text(config_path)
        config = parse_config(config_text, source=str(config_path))
        chip = chip_input_path(config, config_path, chip_path)
        return RunInputs(config, config_text, chip, read_chip_text(chip))

    def prepare(self, config_path: Path, chip_path: Path | None = None) -> ConfigT:
        """Start a run directory from input files.

        Returns:
            The parsed configuration.
        """
        snapshot = self.inputs(config_path, chip_path)
        with self.staging(snapshot) as pending:
            self.publish(pending)
        return snapshot.config

    @contextmanager
    def staging(self, snapshot: RunInputs) -> Generator[RunDirectory, None, None]:
        """Create a temporary run from the input snapshot.

        Yields:
            A run directory on the same volume as the destination.
        """
        parent = self.path.absolute().parent
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as staging:
            pending = RunDirectory(Path(staging) / "run")
            pending.path.mkdir()
            pending.chip.write_bytes(snapshot.chip_text.encode("utf-8"))
            pending.config.write_bytes(snapshot.config_text.encode("utf-8"))
            pending.marker.write_bytes(_RUN_MARKER)
            yield pending

    @contextmanager
    def _write_lock(self) -> Generator[None, None, None]:
        """Serialize writers to an existing run.

        Yields:
            Once the run's write lock is held.

        Raises:
            RunError: If another writer holds the lock.
        """
        self._check_destination()
        with self.marker.open("r+b") as marker:
            try:
                if os.name == "nt":
                    # Lock the byte past the contents so readers can check the marker.
                    marker.seek(0, os.SEEK_END)
                    msvcrt.locking(marker.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    fcntl.flock(marker.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                msg = f"cannot write {self.path}: the run is in use"
                raise RunError(msg) from error
            yield

    def publish(self, pending: RunDirectory) -> None:
        """Publish a completed run while preserving unrelated files."""
        self._check_destination()
        if not self.path.exists():
            pending.path.rename(self.path)
            return
        if not self._owned():
            # Claim an existing empty directory without replacing it.
            with self.marker.open("xb") as marker:
                marker.write(_RUN_MARKER)
        sources = [pending.chip, pending.config]
        for stage in STAGE_FILES:
            for source in (pending.artifact(stage), pending.record(stage)):
                sources.extend((source, source.with_suffix(source.suffix + ".tmp")))
        self._publish_files(sources)

    def _publish_files(
        self,
        sources: Sequence[Path],
        *,
        digests: dict[str, str] | None = None,
        prerequisites: dict[str, bytes] | None = None,
    ) -> None:
        """Replace a set of managed files after checking the inputs under the write lock.

        Raises:
            RunError: If a destination is occupied, inputs changed, or rollback is incomplete.
            OSError: If the files cannot be published.
            KeyboardInterrupt: If publication is interrupted.
        """
        targets = [self.path / source.name for source in sources]
        with self._write_lock():
            if digests is not None:
                config = self.load()
                if input_digests(config, self.classified_chip(config)) != digests:
                    msg = f"the inputs of {self.path} changed during planning; run the step again"
                    raise RunError(msg)
                if prerequisites and self._check_records(list(prerequisites), digests, {}) != prerequisites:
                    msg = f"an earlier artifact of {self.path} changed during planning; run the step again"
                    raise RunError(msg)
            for target in targets:
                if target.exists() and not target.is_file():
                    msg = f"cannot replace {target}: it is not a file"
                    raise RunError(msg)
            originals = [target for target in targets if target.is_file() or target.is_symlink()]
            replacements = [
                (source, target) for source, target in zip(sources, targets, strict=True) if source.is_file()
            ]
            previous = Path(tempfile.mkdtemp(prefix=".mqt-scpd-previous-", dir=self.path.absolute().parent))
            saved: list[Path] = []
            installed: list[Path] = []
            try:
                saved.extend(target.replace(previous / target.name) for target in originals)
                for source, target in replacements:
                    installed.append(source.replace(target))
            except (OSError, KeyboardInterrupt):
                self._restore_files(previous, saved, installed)
                raise
            for backup in saved:
                backup.unlink()
            previous.rmdir()

    def _restore_files(self, previous: Path, saved: list[Path], installed: list[Path]) -> None:
        """Restore displaced files, retaining any backup that cannot be put back.

        Raises:
            RunError: If restoration is incomplete.
        """
        backed_up = {backup.name for backup in saved}
        failures: list[OSError] = []
        for target in installed:
            if target.name not in backed_up:
                try:
                    target.unlink(missing_ok=True)
                except OSError as error:
                    failures.append(error)
        for backup in saved:
            try:
                backup.replace(self.path / backup.name)
            except OSError as error:
                failures.append(error)
        if failures:
            msg = f"cannot restore all files of {self.path}; any remaining backups are in {previous}"
            raise RunError(msg) from failures[0]
        previous.rmdir()

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
            RunError: If the name is not a step of this release, if an input of the step is missing, or
                if an input of an earlier step has changed since that step wrote its artifact. A
                refused step leaves the directory as it was.
        """
        self._check_destination()
        path = self.artifact(stage)
        if stage not in ARTIFACT_INPUTS:
            msg = f"no core call runs the {stage} step"
            raise RunError(msg)
        digests = input_digests(config, chip)
        ordered = list(STAGE_FILES)
        verified = self._check_records(ordered[: ordered.index(stage)], digests, {})
        inputs = []
        for earlier in ARTIFACT_INPUTS[stage]:
            if earlier not in verified:
                msg = f"{self.artifact(earlier).name} is missing; run the {earlier} step first"
                raise RunError(msg)
            inputs.append(verified[earlier])
        packed = write_config(config)
        producer = f"mqt-scpd {__version__}"
        level = min(max(verbosity, 0), 2)
        calls: dict[str, Callable[[], bytes]] = {
            "capacity": lambda: pyscpd.plan_capacity(chip, *inputs, packed, producer, reporter, level),
        }
        if stage not in calls:
            msg = f"no core call runs the {stage} step"
            raise RunError(msg)
        data = calls[stage]()
        record = {name: digests[name] for name in STEP_INPUTS[stage]}
        record[_ARTIFACT_DIGEST] = hashlib.sha256(data).hexdigest()
        with tempfile.TemporaryDirectory(dir=self.path.absolute().parent) as staging:
            pending = RunDirectory(Path(staging))
            pending.artifact(stage).write_bytes(data)
            pending.record(stage).write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8", newline="\n")
            sources = [
                file
                for step in ordered[ordered.index(stage) :]
                for file in (pending.artifact(step), pending.record(step))
            ]
            self._publish_files(sources, digests=digests, prerequisites=verified)
        return StageResult(stage=stage, path=path, size=len(data))

    def check_current(self, stage: str, digests: dict[str, str], *, data: bytes | None = None) -> None:
        """Check that the artifact and its preceding stages match their records.

        Args:
            stage: The step.
            digests: The digests of the inputs the run holds now, as :func:`input_digests` gives them.
            data: The artifact bytes the caller will consume, if it already read them.

        Raises:
            RunError: If an artifact has no readable record or its bytes or inputs have changed.
        """  # ruff: ignore[docstring-extraneous-exception]  # _check_records() raises it
        names = list(STAGE_FILES)
        self._check_records(names[: names.index(stage) + 1], digests, {stage: data} if data is not None else {})

    def _check_records(
        self, stages: Sequence[str], digests: dict[str, str], artifacts: dict[str, bytes]
    ) -> dict[str, bytes]:
        """Read and verify the artifacts that are present. The caller reports missing stage inputs.

        Returns:
            The exact bytes that passed verification.

        Raises:
            RunError: If an artifact has no readable record or its bytes or inputs have changed.
        """
        for earlier in stages:
            artifact = self.artifact(earlier)
            if earlier not in artifacts:
                try:
                    artifacts[earlier] = artifact.read_bytes()
                except FileNotFoundError:
                    continue
            try:
                record = json.loads(self.record(earlier).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                record = None
            if not isinstance(record, dict) or set(record) != {*STEP_INPUTS[earlier], _ARTIFACT_DIGEST}:
                msg = f"{artifact.name} carries no record of the inputs it was made from; run the {earlier} step again"
                raise RunError(msg)
            if record[_ARTIFACT_DIGEST] != hashlib.sha256(artifacts[earlier]).hexdigest():
                msg = f"{artifact.name} does not match its input record; run the {earlier} step again"
                raise RunError(msg)
            for name in STEP_INPUTS[earlier]:
                if record[name] != digests[name]:
                    where = name if name == RUN_CHIP else f"{name} in config.toml"
                    msg = (
                        f"{where} changed after the {earlier} step wrote {artifact.name}; run the {earlier} step again"
                    )
                    raise RunError(msg)
        return artifacts
