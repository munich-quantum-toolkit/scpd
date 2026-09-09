# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The run directory: what a stage reads, what it writes, and how a run is resumed.

Every stage reads and writes one artifact, so a run can be continued from any completed stage
instead of from the beginning. Running a stage invalidates everything after it, and the directory
is left in one state rather than a mixed one: the later artifacts are deleted rather than kept
beside a newer earlier one.

File orchestration is Python's job throughout. The core never opens a file; it takes bytes and
returns bytes.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from . import pyscpd
from ._version import version as __version__
from .chip import chip_input_path, classify_chip
from .config import load_config, write_config

if TYPE_CHECKING:
    from .flatbuffers.config.Config import ConfigT

__all__ = ["STAGE_FILES", "RunDirectory", "RunError", "stages_before"]


class RunError(RuntimeError):
    """A run directory that cannot be read, or a stage whose input is missing."""


#: The artifact each stage writes, in pipeline order. The number is the order, so a directory
#: listing reads as the pipeline does.
#:
#: The Global stage comes before the Assignment stage. Which outer ports the inner circuit
#: surfaces at is what the assignment consumes, so the inner circuit has to be solved first; the
#: prototype's own drivers run the two in this order for the same reason.
STAGE_FILES: dict[str, str] = {
    "capacity": "01-capacity.fb",
    "global": "02-global.fb",
    "assign": "03-assign.fb",
    "detail": "04-detail.fb",
    "final": "05-final.fb",
    "geometry": "06-geometry.fb",
}

#: The stages this release implements, in order.
IMPLEMENTED: tuple[str, ...] = ("capacity", "global", "assign")


def stages_before(stage: str) -> list[str]:
    """The stages a stage needs, in order.

    Args:
        stage: The stage to run.

    Returns:
        The names of the stages whose artifacts it reads.

    Raises:
        RunError: If the stage is not one this release implements.
    """
    if stage not in IMPLEMENTED:
        msg = f"'{stage}' is no stage this release runs; it runs {', '.join(IMPLEMENTED)}"
        raise RunError(msg)
    return list(IMPLEMENTED[: IMPLEMENTED.index(stage)])


@dataclass(frozen=True)
class StageResult:
    """What running one stage did."""

    stage: str
    path: Path
    size: int


class RunDirectory:
    """One run's directory, and the stages that fill it."""

    def __init__(self, path: Path) -> None:
        self.path = path

    def artifact(self, stage: str) -> Path:
        """The file a stage writes.

        Args:
            stage: The stage name.

        Returns:
            The path, whether or not it exists.

        Raises:
            RunError: If the name is no stage of the pipeline.
        """
        if stage not in STAGE_FILES:
            msg = f"'{stage}' is no stage of the pipeline"
            raise RunError(msg)
        return self.path / STAGE_FILES[stage]

    @property
    def config(self) -> Path:
        """The configuration copied into the run for provenance."""
        return self.path / "config.toml"

    @property
    def chip(self) -> Path:
        """The chip input copied into the run for provenance."""
        return self.path / "00-chip.json"

    def prepare(self, config_path: Path) -> ConfigT:
        """Create the directory and copy the inputs into it.

        Args:
            config_path: The configuration to run.

        Returns:
            The parsed configuration.
        """
        config = load_config(config_path)
        self.path.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(config_path, self.config)
        shutil.copyfile(chip_input_path(config, config_path).resolve(), self.chip)
        return config

    def load(self) -> ConfigT:
        """The configuration of an existing run.

        Returns:
            The parsed configuration.

        Raises:
            RunError: If the directory carries no configuration.
        """
        if not self.config.is_file():
            msg = f"{self.path} carries no config.toml; it is not a run directory"
            raise RunError(msg)
        return load_config(self.config)

    def invalidate_after(self, stage: str) -> list[Path]:
        """Delete every artifact a stage's output supersedes.

        Args:
            stage: The stage that was just run.

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
            msg = f"{path.name} is missing; run the {stage} stage first"
            raise RunError(msg)
        return path.read_bytes()

    def run_stage(self, stage: str, config: ConfigT) -> StageResult:
        """Run one stage and write its artifact.

        Args:
            stage: The stage to run.
            config: The run configuration.

        Returns:
            What was written.

        Raises:
            RunError: If the stage is unknown, or an artifact it reads is missing.
        """
        stages_before(stage)
        # The chip is read from the copy the run carries, not from wherever the configuration
        # points, so a run stays reproducible after its inputs move.
        try:
            text = self.chip.read_text(encoding="utf-8")
        except OSError as error:
            msg = f"{self.chip} is missing; the run directory is incomplete"
            raise RunError(msg) from error
        chip = classify_chip(text, config, str(self.chip))
        packed = write_config(config)

        if stage == "capacity":
            data = pyscpd.plan_capacity(chip, packed, __version__)
        elif stage == "global":
            data = pyscpd.route_global(chip, self._read("capacity"), packed, __version__)
        else:
            data = pyscpd.assign(chip, self._read("capacity"), self._read("global"), packed, __version__)

        path = self.artifact(stage)
        path.write_bytes(data)
        self.invalidate_after(stage)
        return StageResult(stage=stage, path=path, size=len(data))
