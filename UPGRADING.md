# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### Planning a chip

`mqt-scpd plan` runs the planning steps of a chip into a run directory:

```console
mqt-scpd plan -c planar-superconducting-pd/inputs/17q/config.toml -o runs/17q
```

A chip whose couplers carry a wire across needs a few keys that a configuration
for `plot` and `render` does not: a `component` pattern, which groups the ports
by component, and a `bridge_pair` pattern with the `[[ports.bridge_pairs]]`
rules that pair those ports. The `component` pattern has to match every
`bridge_pair` port. `plan` checks the implementation names of the stages and the
size of the detail grid before its first stage, and refuses a run whose steps
they would stop; the run directory then stays as it was. `doctor` reports the
same problems for the steps up to `[run] stop_after`, which are all steps when
the key is missing.

GDS/OASIS planning exports preserve holes in the port-keepout layer. Export
existing plans again with `mqt-scpd render` to update these files.

No stage reads `[grid] launcher_offset_x` and `launcher_offset_y` any more: the
capacity step blocks no border of its grid, since the launchers sit on the edge
of the grid. A configuration that sets them still loads, and `doctor` and `plan`
warn about each key. Remove them; a later version refuses them.

The detail grid, `[grid] capacity_cells_x` and `capacity_cells_y` times
`detail_factor`, may have at most 25,000,000 cells. `plan` and `doctor` refuse a
larger grid before the first stage; `docs/configuration.md` explains the limit.
`capacity_cells_x` has to be at least two, and `capacity_cells_y` either `0` or
at least two, since a grid needs two cells along each axis. The capacity step
refuses a chip on which the targets of two ports lie on one cell of the detail
grid; a larger `detail_factor` makes the cells smaller.

Every step writes a record of its inputs beside its artifact, for example
`01-capacity.inputs.json`: a digest of the chip and of each configuration
section the step reads. `plan --stage <step>` refuses a step when the chip or
such a section has changed since an earlier step wrote its artifact, and names
the section and the step to run again. The run directory then stays as it was.
`plot` and `render` with `--run-dir` refuse a stage in the same way, and also an
artifact without a record, which a step that failed leaves behind. `inspect`
prints such an artifact of a run directory after a warning on stderr.

### Output of the commands

`plot`, `render` and `inspect -o` end with a line that starts with `✓`. Every
command except `doctor` reports an error of an input or a stage (exit code 1) on
a line that starts with `✗`. Both lines have the format of
`docs/terminal_output.md`. A script that looked for `wrote` or `error:` looks
for these lines instead. Wrong arguments still print the `argparse` usage
message with `error:` and exit with code 2. `doctor` reports each problem on a
line that starts with `PROBLEM:`, and each warning, such as a removed key, on a
line that starts with `WARNING:`. It ends with `doctor: OK` or
`doctor: <n> problem(s)`; a warning is not a problem. Every command exits with
code 130 on Ctrl-C, and with code 1 and no message when the reader of its output
goes away, as with `| head`.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
