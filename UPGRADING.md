# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### Planning a chip

`mqt-scpd plan` needs a few keys that a configuration for `doctor`, `plot` and
`render` does not:

- `[stages.assignment] launcher_target`: how many feedline chain ends the
  assignment puts on a launcher; every other chain end is a termination. It is a
  figure of the chip and has no default.
- A `component` pattern, which groups the ports by component, and, for a chip
  whose couplers carry a wire across, a `bridge_pair` pattern with the
  `[[ports.bridge_pairs]]` rules that pair those ports. The `component` pattern
  has to match every `bridge_pair` port.

The configurations of the benchmark chips in
[planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd)
carry these keys:

```console
mqt-scpd plan -c planar-superconducting-pd/inputs/17q/config.toml -o runs/17q
```

The assignment expects the chip input to list the launcher ports once around the
chip, against the direction of the port ring, as on the benchmark chips. A chip
with resonators needs at least three launchers that do not all lie on one line.
With fewer, two resonators could share one feed point, and the assignment
refuses the chip. The assignment also refuses a solution that puts two feeds, or
a feed and its launcher, closer together than `[design_rules] min_wire_spacing`.
`launcher_target` plus `[design_rules] feedline_terminations` has to be even,
since every feedline chain has two ends. The counts of the ring set more limits:
the conventional ports of the ring plus `launcher_target` need at most as many
launchers as the chip has, `launcher_target` and `feedline_terminations` are
each at most the resonators of the ring, and the resonators are at most the
feedline chains times `[design_rules] max_feedline_utilization`
(`docs/configuration.md` lists them). The global step refuses a ring that
carries both ends of a bridge. `plan` checks these figures, the ring and the
implementation names of the stages before its first stage, and refuses a run
whose steps they would stop; the run directory then stays as it was. `doctor`
reports the same problems for the steps that `[run] stop_after` configures.

An inner routable port that the `component` pattern does not match ends a wire
of the inner circuit, as a port of an inner qubit does.

The detail grid, `[grid] capacity_cells_x` and `capacity_cells_y` times
`detail_factor`, may have at most 25,000,000 cells. `plan` and `doctor` refuse a
larger grid before the first stage; `docs/configuration.md` explains the limit.

Every step writes a record of its inputs beside its artifact, for example
`01-capacity.inputs.json`: a digest of the chip and of each configuration
section the step reads. `plan --stage <step>` refuses a step when the chip or
such a section has changed since an earlier step wrote its artifact, and names
the section and the step to run again. The run directory then stays as it was.

The planning stages solve with HiGHS unless `[stages.solver] backend` or the
environment variable `SCPD_SOLVER` is `gurobi`. Gurobi needs an installed and
licensed `gurobipy`. `plan` also refuses before its first stage a backend name
that the core does not know, and a Gurobi backend without a usable `gurobipy`
when a step of the run solves. `doctor` reports the same problems for the steps
that `[run] stop_after` configures.

### Output of the commands

`plot`, `render` and `inspect -o` end with a line that starts with `✓`. Every
command except `doctor` reports an error of an input or a stage (exit code 1) on
a line that starts with `✗`. Both lines have the format of
`docs/terminal_output.md`. A script that looked for `wrote` or `error:` looks
for these lines instead. Wrong arguments still print the `argparse` usage
message with `error:` and exit with code 2. `doctor` reports each problem on a
line that starts with `PROBLEM:` and ends with `doctor: OK` or
`doctor: <n> problem(s)`.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
