from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class Rotation(object):
  R0 = cast(int, ...)
  R45 = cast(int, ...)
  R90 = cast(int, ...)
  R135 = cast(int, ...)
  R180 = cast(int, ...)
  R225 = cast(int, ...)
  R270 = cast(int, ...)
  R315 = cast(int, ...)

