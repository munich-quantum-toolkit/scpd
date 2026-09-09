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

### CMake 3.28

Building from source now requires CMake 3.28 or newer.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
