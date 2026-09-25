# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The ``inspect`` command: a stage artifact as JSON.

Binary does not mean opaque. The core renders an artifact through the type tables that the schema
compiler emits beside the reader, so the JSON carries the schema's own field names, its enum names
and its union tags, and no second description of the model has to be kept in step with it.
"""

from __future__ import annotations

from . import pyscpd


class InspectionError(ValueError):
    """Bytes that are not a stage artifact of this schema version."""


def artifact_to_json(data: bytes) -> str:
    """Render the bytes of a stage artifact as JSON.

    Args:
        data: The bytes of the artifact, as a run directory stores them.

    Returns:
        The artifact as indented JSON, ending in a newline. The keys are the field names of the
        schema, which are snake_case, and enums and union tags are spelled by name.

    Raises:
        InspectionError: If the bytes are not a complete artifact of this schema version.
    """
    try:
        return pyscpd.artifact_to_json(data)
    except ValueError as error:
        raise InspectionError(str(error)) from error
