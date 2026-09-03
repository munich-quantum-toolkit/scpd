from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from flatbuffers import table
from mqt.scpd.generated.Arc import Arc
from mqt.scpd.generated.Line import Line
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class SegmentShape(object):
  NONE = cast(int, ...)
  Line = cast(int, ...)
  Arc = cast(int, ...)
def SegmentShapeCreator(union_type: typing.Literal[SegmentShape.NONE, SegmentShape.Line, SegmentShape.Arc], table: table.Table) -> typing.Union[None, Line, Arc]: ...

