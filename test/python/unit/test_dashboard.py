# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests for the dashboard of a run's searches."""

from __future__ import annotations

import re
from typing import TYPE_CHECKING

from mqt.scpd import dashboard

if TYPE_CHECKING:
    from pathlib import Path

    import pytest

#: The lines of a feedline pass in which one wire finds its way and one does
#: not, each naming its picture as the run saw it.
LOG = (
    "[final]     0.10s  feedline routing: 2 wires, up to 1 rounds, 0 relaxations each way",
    "[final]     0.11s    wire 0 · round 0 forward · normal: found 5 cells · run/debug/found.svg",
    "[final]     0.12s    wire 1 · round 0 forward · normal: no way, relaxing · run/debug/failed.svg",
)

PICTURE = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">'
    '<rect class="ob" x="0" y="0" width="4" height="4"/></svg>'
)


def test_every_attempt_opens_a_viewer_beside_the_page(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """An attempt that found its way gets a viewer as one that failed does, and the links hold from the page."""
    monkeypatch.chdir(tmp_path)
    debug = tmp_path / "run" / "debug"
    debug.mkdir(parents=True)
    (debug / "found.svg").write_text(PICTURE, encoding="utf-8")
    (debug / "failed.svg").write_text(PICTURE, encoding="utf-8")
    log = debug / "run.log"
    log.write_text("\n".join(LOG) + "\n", encoding="utf-8")
    page = debug / "dashboard-feedline.html"

    dashboard.build(log, "feedline", page, debug / "viewers")

    links = re.findall(r'href="([^"]+)"', page.read_text(encoding="utf-8"))
    assert sorted(links) == ["viewers/failed.html", "viewers/found.html"]
    for link in links:
        assert (debug / link).exists()
