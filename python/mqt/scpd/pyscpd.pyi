# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Internal bindings of the MQT SCPD core. The command-line interface is the supported product."""

def load_chip(chip_json: str, config: bytes) -> bytes:
    """Read the chip input, classify its ports from the configuration's patterns and check the configured port sequences against the chip. Returns the classified chip as bytes of the design schema. Raises ValueError naming every problem."""

def validate_config(config: bytes) -> list[str]:
    """The problems of a configuration that can be seen without the chip, empty when there are none."""

def artifact_to_json(artifact: bytes) -> str:
    """Render a stage artifact as JSON. The field names, the enum names and the union tags come from the schema, so the JSON follows it without a second description of the model. Raises ValueError when the bytes are not a complete artifact of this schema version."""
