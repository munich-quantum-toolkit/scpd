# 0006 — Neutral geometry IR with a KLayout adapter

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The deliverable is GDSII or OASIS. The prototype instead hand-wrote SVG in six
unrelated renderers totalling about 2,150 lines, with no shared scene graph or
transform helper.

GDSFactory was considered as the export path because it offers a ready
connection to KLayout.

## Decision

The core emits a neutral geometry intermediate representation. Python adapters
turn it into layout files. `klayout` is the one supported adapter and
dependency; GDSFactory is an optional extra over the same IR.

The IR carries **analytic segments** — lines and circular arcs with exact centre
and radius — not sampled points. Polygonization happens once, in the adapter, at
a configured tolerance.

## Alternatives considered

**GDSFactory as a hard dependency.** Gives a component and PDK ecosystem,
plotting and design-rule checking. Rejected: it sits on KLayout anyway, so it
reaches the same file through a much larger and faster-moving dependency stack,
and its opinions about layout structure are not ones this tool needs.

**A GDS writer in C++.** Fastest at run time and dependency-light, but it means
owning the format, and OASIS support becomes separate work.

**Sampled points in the IR.** What the prototype did. Rejected: it discards
exactness, then spends 419 lines plus a 1,771-line helper block reconstructing
arc structure from the polylines it flattened.

## Consequences

- The core never links a layout library.
- Export backends are independently testable against a fixed IR.
- Bend-radius checking becomes a direct comparison rather than an inference.
- KLayout does **not** give design-rule checking on the exported file. Its
  Python package is used here for GDS/OASIS writing and rendering only, and the
  core owns design-rule checking outright
  ([decision 0022](0022-drc-in-the-core.md)).
