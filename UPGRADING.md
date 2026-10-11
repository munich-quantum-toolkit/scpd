# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### The run directory numbers seven artifacts

The Corridor stage now stands between the assignment and the detail routing, so
every artifact after `03-assign.fb` moves up by one:

| Before           | Now               |
| ---------------- | ----------------- |
| —                | `04-corridor.fb`  |
| `04-detail.fb`   | `05-detail.fb`    |
| `05-final.fb`    | `06-final.fb`     |
| `06-geometry.fb` | `07-geometry.fb`  |

A run directory written by an earlier build is not read by this one. Run the
pipeline again rather than renaming the files: the stages before the corridor
also changed, because a partition border now reports the number of wires it can
carry instead of its length in cells.

`plot --stage` and `render --stage` take the new name `corridor`; the other
stage names are unchanged.

### The coupler insertion checks the capacity graph by default

The chain search of the coupler insertion refuses every step whose feedline edge
closes the way of an outer wire (`SCPD_CAPACITY_RULE`). It finds the bottlenecks
of each step by profiles along the walls it moves (`SCPD_CAPACITY_FAST=1`), and
it reports the capacity graph of the chip it leaves behind (`SCPD_BOTTLENECKS`).
All three are on by default. With them every benchmark chip ends the feedline
routing with no bad wire, but the insertion takes longer.
`SCPD_CAPACITY_RULE=0`, `SCPD_CAPACITY_FAST=0` and `SCPD_BOTTLENECKS=0` give the
earlier behavior.

### The refinements search a narrower band

Both refinements of the Final stage now redraw a wire within
`refinement_corridor_spacings` wire spacings of its way, 6 by default, instead
of `corridor_spacings`. The routes the refinements leave change. Set
`refinement_corridor_spacings = 0` under `[stages.final]` to give them the
sweep's band again.

### CMake 3.28

Building from source now requires CMake 3.28 or newer.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
