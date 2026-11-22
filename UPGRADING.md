# Upgrade Guide

See the [changelog](CHANGELOG.md) for the full list of changes.

## [Unreleased]

- To upgrade an `SCP1` run, use its `config.toml` and `00-chip.json` with
  `mqt-scpd plan -c <config> --chip <chip> -o <new-dir>`. This creates `SCP2`
  artifacts with the run marker and input records.
- The Capacity stage requires detail grid steps within 10% of each other. Set
  `[grid] capacity_cells_y` to zero to derive it from the chip aspect ratio, or
  set explicit dimensions that produce nearly square cells.
- Remove `[grid] launcher_offset_x` and `launcher_offset_y` from existing
  configurations. The loader accepts these keys with a warning.

To plan through Assignment, set `[stages.assignment] launcher_target` to the
number of chain ends that need a launcher on your chip. Use
`[run] stop_after = "capacity"` for Capacity alone. `doctor` checks through the
configured stop point, which defaults to Corridor. See the
[configuration reference](docs/configuration.md) for launcher order, ring
counts, and solver settings.

The Linux x86-64 wheels require a processor with the POPCNT instruction.

Keep existing `SCP2` Capacity artifacts. Regenerate earlier `SCP2` Global and
Assignment placeholders from the original configuration and chip. To add
Corridor to an existing run with current artifacts, run
`mqt-scpd plan -o <run-dir> --stage corridor`. Set `[run] stop_after = "assign"`
to keep Assignment as the last stage of a full run. `plan` exits with code 1 if
Corridor leaves any connection unrouted. Rerun Capacity if Corridor reports that
a saved partition covers a blocked cell or that a border sample does not join
its recorded partitions.

Assignment uses a sliding window to enforce the feedline utilization limit. The
bundled HiGHS 1.16.0 enables parallel mixed-integer search by default. Rerunning
Assignment can select a different layout when objective values tie.

Install the optional Gurobi backend with `uv pip install "mqt-scpd[gurobi]"`.

GDS/OASIS planning exports preserve holes in the port approach layer. Run
`mqt-scpd render` again to update exported files.

KLayout 0.30.13 has Python 3.15 wheels on Linux and macOS. On Windows x86-64,
its wheels cover Python through 3.14. KLayout has no Windows ARM64 wheel.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
