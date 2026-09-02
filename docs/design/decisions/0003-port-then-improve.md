# 0003 — Port first, improve second

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype produces published results: eight benchmarks routed with zero
unresolved wires. A rewrite that changes structure and algorithms at the same
time cannot tell a structural bug from an algorithmic regression.

## Decision

Move the algorithms across largely as they are, restructured but not re-derived,
and keep the published quality reproducible as a baseline. Only then change
algorithmic behaviour.

Cleanups that are forced by the new structure — collapsing six copies of one
driver, deleting dead code, replacing string-typed entity handling — are part of
the port, not algorithmic change. What is deferred is changing
*what the algorithms compute*.

## Alternatives considered

**Redesign freely, treating the prototype as a specification.** Better final
architecture, but no baseline to test against during the most error-prone phase.

**Keep the hot path verbatim, redesign everything else.** Close to what happens
in practice, but drawing the line at a file boundary rather than at "behaviour
versus structure" would have carried the router's known defects across
untouched.

## Consequences

- Some prototype design debt lives on in the first release.
- The benchmark comparison is meaningful throughout the port.
- Phase 3 requires backend agreement on objective values, and phase 4 requires
  zero final failures, before anything is tuned.
