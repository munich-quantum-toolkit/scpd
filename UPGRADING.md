# Upgrade Guide

See the [changelog](CHANGELOG.md) for the full list of changes.

## [Unreleased]

- The Capacity stage requires detail grid steps within 10% of each other. Set
  `[grid] capacity_cells_y` to zero to derive it from the chip aspect ratio, or
  set explicit dimensions that produce nearly square cells.

The Linux x86-64 wheels require a processor with the POPCNT instruction.

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
