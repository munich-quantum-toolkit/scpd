# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The debug pages of a run.

The Final stage records what the pages of the ``[debug]`` section need, one
JSON record at a time, through the callback a run hands it. :mod:`.store`
keeps the records under ``<run>/debug/data``, :mod:`.pages` builds
``<run>/debug/index.html`` and the pages from them, and :mod:`.serve` puts
the capacity graph behind a local server that moves couplers.
"""

from __future__ import annotations

from . import pages, serve, store

__all__ = ["pages", "serve", "store"]
