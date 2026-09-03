from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class DrcRule(object):
  WireClearance = cast(int, ...)
  FeedlineOrthogonality = cast(int, ...)
  WireLoop = cast(int, ...)
  ObstacleClearance = cast(int, ...)
  ComponentOverlap = cast(int, ...)
  MinStraightLength = cast(int, ...)
  ResonatorLength = cast(int, ...)
  MinBendRadius = cast(int, ...)

