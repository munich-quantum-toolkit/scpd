# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The chip input, loaded through the core.

The core parses the prototype's ``routing_config.json``, classifies the ports from the patterns of
the configuration, and hands the classified chip back as the bytes of the ``design.fbs`` schema.
Those bytes are what every core function takes; ``decode_chip`` turns them into the generated
object model when Python needs to look inside.
"""

from __future__ import annotations

import re
from typing import TYPE_CHECKING

from . import pyscpd
from .config import write_config
from .flatbuffers.design.Chip import ChipT
from .flatbuffers.design.UnassignedRole import UnassignedRole

if TYPE_CHECKING:
    from pathlib import Path

    from .flatbuffers.config.Config import ConfigT
    from .flatbuffers.design.Port import PortT
    from .flatbuffers.geometry.Point import PointT
    from .flatbuffers.geometry.Polygon import PolygonT


class ChipError(ValueError):
    """A chip input that cannot be loaded, or that does not fit its configuration."""


#: The name of each role, by schema value, as the configuration keys spell it: the enum name in snake
#: case.
ROLE_NAMES: dict[int, str] = {
    value: re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()
    for name, value in vars(UnassignedRole).items()
    if not name.startswith("_") and isinstance(value, int)
}


def chip_input_path(config: ConfigT, config_path: Path, chip_path: Path | None = None) -> Path:
    """The chip input of a run: the one given, else the one the configuration names.

    Args:
        config: The loaded configuration.
        config_path: The configuration file, which the configured chip input is relative to.
        chip_path: A chip input that replaces the configured one, as ``--chip`` gives it.

    Returns:
        The path of the chip input.
    """
    if chip_path is not None:
        return chip_path
    return config_path.parent / (config.chipInput or "")


def load_chip(config: ConfigT, config_path: Path, chip_path: Path | None = None) -> bytes:
    """Load and classify the chip input of a configuration.

    Args:
        config: The loaded configuration.
        config_path: The configuration file, which the configured chip input is relative to.
        chip_path: A chip input that replaces the configured one, as ``--chip`` gives it.

    Returns:
        The classified chip as bytes of the ``design.fbs`` schema.

    Raises:
        ChipError: If the input cannot be read, is not a valid chip input, or does not fit the
            configuration. The message names every problem.
    """
    path = chip_input_path(config, config_path, chip_path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        msg = f"cannot read the chip input {path}: {error}"
        raise ChipError(msg) from error
    try:
        return pyscpd.load_chip(text, write_config(config))
    except ValueError as error:
        msg = f"{path}: {error}"
        raise ChipError(msg) from error


def decode_chip(data: bytes) -> ChipT:
    """Decode the bytes of a classified chip into the generated object model.

    Returns:
        The chip.
    """
    return ChipT.InitFromPackedBuf(data, 0)


def role_name(role: int) -> str:
    """The name of a role as the configuration keys spell it.

    Returns:
        The name, or ``"unset"``.
    """
    return ROLE_NAMES.get(role, "unset")


def ports_of(chip: ChipT) -> list[PortT]:
    """The ports of a chip.

    The generated model types every list element as optional, because a buffer could omit one.
    A chip the core wrote never does; this accessor states that once.

    Returns:
        The ports, in PortRef order.
    """
    return [port for port in chip.ports or [] if port is not None]


def obstacles_of(chip: ChipT) -> list[PolygonT]:
    """The obstacle polygons of a chip.

    Returns:
        The polygons, in input order.
    """
    return [polygon for polygon in chip.obstacles or [] if polygon is not None]


def vertices_of(polygon: PolygonT) -> list[PointT]:
    """The vertices of a polygon.

    Returns:
        The vertices, in input order.
    """
    return [vertex for vertex in polygon.vertices or [] if vertex is not None]
