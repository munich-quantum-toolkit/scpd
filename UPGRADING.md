# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

- Stage artifacts use the `SCP2` identifier. The former `SCP1` tables cannot
  store the full Capacity plan. To upgrade a run, use the old `config.toml` and
  `00-chip.json` as inputs to
  `mqt-scpd plan -c <config> --chip <chip> -o <new-dir>`. The new directory has
  the marker and input records this version requires.
- The Capacity stage requires detail grid steps within 10% of each other. Set
  `[grid] capacity_cells_y` to zero to derive it from the chip aspect ratio, or
  set explicit dimensions that produce nearly square cells.
- Remove `[grid] launcher_offset_x` and `launcher_offset_y` from existing
  configurations. The loader accepts these keys with a warning.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
