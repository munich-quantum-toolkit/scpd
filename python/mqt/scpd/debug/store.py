# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The record store of a debug run: one compressed JSON-lines file per kind.

A run of the largest chip records tens of thousands of searches, so the
records are written as they come, compressed, and never held in memory
together. ``<run>/debug/data/<kind>.jsonl.gz`` holds every record of one kind
in the order the stage made them.
"""

from __future__ import annotations

import gzip
import json
import shutil
from typing import TYPE_CHECKING, Any, Self

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

#: The directory under ``<run>/debug`` the files go in.
DATA = "data"
SUFFIX = ".jsonl.gz"


def data_dir(debug: Path) -> Path:
    """Where the records of the debug directory ``debug`` are.

    Returns:
        The directory.
    """
    return debug / DATA


class Sink:
    """Where a run hands its records: the callback a stage is given.

    Opening a sink starts the store over, so a run never adds to the records of an earlier run in
    the same directory. The file of a kind is opened when its first record arrives and kept open
    until :meth:`close`.
    """

    def __init__(self, debug: Path) -> None:
        """Start an empty store under ``debug``.

        Args:
            debug: The run's debug directory.
        """
        self.directory = data_dir(debug)
        shutil.rmtree(self.directory, ignore_errors=True)
        self.directory.mkdir(parents=True)
        self.files: dict[str, gzip.GzipFile] = {}
        self.counts: dict[str, int] = {}

    def __call__(self, kind: str, record: str) -> None:
        """Append one record.

        Args:
            kind: The record's kind, which names its file.
            record: The record as JSON text.
        """
        file = self.files.get(kind)
        if file is None:
            file = gzip.GzipFile(self.directory / (kind + SUFFIX), "wb")
            self.files[kind] = file
        file.write(record.encode("utf-8"))
        file.write(b"\n")
        self.counts[kind] = self.counts.get(kind, 0) + 1

    def close(self) -> None:
        """Finish every file."""
        for file in self.files.values():
            file.close()
        self.files.clear()

    def __enter__(self) -> Self:
        """The sink itself.

        Returns:
            The sink.
        """
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Finish every file."""
        self.close()


def kinds(debug: Path) -> list[str]:
    """The kinds the store under ``debug`` has records of, sorted.

    Returns:
        The kinds; empty when there is no store.
    """
    directory = data_dir(debug)
    if not directory.is_dir():
        return []
    return sorted(path.name[: -len(SUFFIX)] for path in directory.glob("*" + SUFFIX))


def read(debug: Path, kind: str) -> Iterator[dict[str, Any]]:
    """The records of one kind, in the order they were written.

    Yields:
        One record at a time; nothing when the kind was never recorded.
    """
    path = data_dir(debug) / (kind + SUFFIX)
    if not path.is_file():
        return
    with gzip.open(path, "rb") as file:
        for line in file:
            if line.strip():
                yield json.loads(line)
