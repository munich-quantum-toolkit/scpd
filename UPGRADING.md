# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

The routing API introduced in [#134] uses tangent straight leads around each
circular turn. Turn endpoints, primitive identifiers, costs, and selected routes
can differ from earlier versions of that branch. Obtain identifiers through
`MovePrimitives` instead of storing numeric identifiers. Turns now start on
their first tagged cell, including the source-stub boundary. Use `decodePath()`
to read move geometry separately from swept occupancy.

`spliceCouplerDogleg()` accepts a final `candidateAllowed` callback. It checks
complete paths as a hard constraint; the anchor callback remains a preference.
Pass the source primitive tables to standalone `CrossingConstraints::build()`
when checking tagged feedlines, including terminal turns.

[#134]: https://github.com/munich-quantum-toolkit/scpd/pull/134

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
