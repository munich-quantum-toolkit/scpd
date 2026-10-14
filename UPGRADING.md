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
refuses the chip. `launcher_target` plus `[design_rules] feedline_terminations`
has to be even, since every feedline chain has two ends. `plan` checks these
figures and the implementation names of the stages before its first stage, and
refuses a run whose steps they would stop; the run directory then stays as it
was. `doctor` reports the same problems for the steps that `[run] stop_after`
configures.

An inner routable port that the `component` pattern does not match ends a wire
of the inner circuit, as a port of an inner qubit does.

The planning stages solve with HiGHS unless `[stages.solver] backend` or the
environment variable `SCPD_SOLVER` is `gurobi`. Gurobi needs an installed and
licensed `gurobipy`. `plan` also refuses before its first stage a backend name
that the core does not know, and a Gurobi backend without a usable `gurobipy`
when a step of the run solves. `doctor` reports the same problems for the steps
that `[run] stop_after` configures.

### Output of the commands

`plot`, `render` and `inspect -o` end with a line that starts with `✓`, and
every command reports an error on a line that starts with `✗`, both in the
format of `docs/terminal_output.md`. A script that looked for `wrote` or
`error:` looks for these instead.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
