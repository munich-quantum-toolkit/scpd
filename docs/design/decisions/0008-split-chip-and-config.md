# 0008 — Chip description separate from routing configuration

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype had one `routing_config` JSON file with four top-level keys, two
of which were ignored on load. It described geometry only. Tuning lived in C++
parameter structs, environment variables and hard-coded call-site constants, and
the ring order of outer ports was a hand-entered array of about 350 strings
inside each benchmark driver.

## Decision

Two inputs.

- **Chip description**, versioned JSON: geometry, ports, obstacles, technology.
  What to route.
- **Routing configuration**, TOML: algorithm selection and tuning parameters.
  How to route it.

A `mqt-scpd convert` subcommand migrates the prototype's configurations.

## Alternatives considered

**One new unified schema.** Simpler single-file workflow, but leaves geometry
and tuning entangled, which is what made the prototype's parameters
un-reviewable.

**Keep the existing schema.** Would preserve the benchmark inputs untouched, but
the existing schema does not describe half of what a run depends on.

## Consequences

- The hand-entered port ring is **not** input. It is derived from geometry by
  the port-ordering pass. Forcing this distinction into the open is much of the
  value of the split.
- The conversion tool is what preserves the quality baseline across the move.
- Chip descriptions become instance-based rather than flattened, which is
  covered separately in [decision 0016](0016-artifact-formats.md).
