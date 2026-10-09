# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### Planning a chip

`mqt-scpd plan` needs a few keys that a configuration for `doctor`, `plot` and
`render` does not:

- `[stages.assignment] launcher_target`: how many feedline ends the chip gives a
  launcher. It is a figure of the chip and has no default.
- A `component` pattern, which groups the ports by component, and, for a chip
  whose couplers carry a wire across, a `bridge_pair` pattern with the
  `[[ports.bridge_pairs]]` rules that pair those ports.

The configurations in `benchmarks/<chip>/config.toml` carry these keys. Plan a
benchmark chip with the chip input of
[planar-superconducting-pd](https://github.com/cda-tum/planar-superconducting-pd):

```console
mqt-scpd plan -c benchmarks/17q/config.toml \
  --chip planar-superconducting-pd/inputs/17q/routing_config.json -o runs/17q
```

The assignment expects the launcher ports to be numbered against the direction
of the port ring, as on the benchmark chips.

### Output of the commands

`plot`, `render` and `inspect -o` end with a line that starts with `✓`, and
every command reports an error on a line that starts with `✗`, both in the
format of `docs/terminal_output.md`. A script that looked for `wrote` or
`error:` looks for these instead.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
