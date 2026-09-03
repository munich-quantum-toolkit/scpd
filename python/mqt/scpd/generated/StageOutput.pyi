from __future__ import annotations

import flatbuffers
import numpy as np

import typing
from flatbuffers import table
from mqt.scpd.generated.Assignment import Assignment
from mqt.scpd.generated.CapacityPlan import CapacityPlan
from mqt.scpd.generated.DetailRouting import DetailRouting
from mqt.scpd.generated.FinalRouting import FinalRouting
from mqt.scpd.generated.Geometry import Geometry
from mqt.scpd.generated.GlobalRouting import GlobalRouting
from typing import cast

uoffset: typing.TypeAlias = flatbuffers.number_types.UOffsetTFlags.py_type

class StageOutput(object):
  NONE = cast(int, ...)
  CapacityPlan = cast(int, ...)
  Assignment = cast(int, ...)
  GlobalRouting = cast(int, ...)
  DetailRouting = cast(int, ...)
  FinalRouting = cast(int, ...)
  Geometry = cast(int, ...)
def StageOutputCreator(union_type: typing.Literal[StageOutput.NONE, StageOutput.CapacityPlan, StageOutput.Assignment, StageOutput.GlobalRouting, StageOutput.DetailRouting, StageOutput.FinalRouting, StageOutput.Geometry], table: table.Table) -> typing.Union[None, CapacityPlan, Assignment, GlobalRouting, DetailRouting, FinalRouting, Geometry]: ...

