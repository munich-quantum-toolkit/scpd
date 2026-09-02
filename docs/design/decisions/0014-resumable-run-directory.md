# 0014 — Resumable run directory

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The largest benchmark takes 456 seconds, most of it in the last two stages.
Iterating on the detail router should not require re-solving two mixed-integer
programs. Measurement, benchmarking and experiments all want per-stage output.

## Decision

Every stage reads and writes an artifact in a run directory. The CLI can run the
whole pipeline, a single stage, or resume from a completed one.

```bash
mqt-scpd route chip.json -c config.toml -o run/
mqt-scpd detail run/ --router astar-dubins
mqt-scpd final run/
```

Running a stage invalidates every artifact after it; the CLI deletes them rather
than leaving the directory in a mixed state.

## Alternatives considered

**Metrics plus optional debug dumps**, with no resume. Much less schema surface,
but every experiment re-runs the full pipeline.

**In-memory with opaque checkpoints.** One serialization format instead of one
per stage boundary, but the artifacts stop being inspectable, which removes most
of the reason for having them.

## Consequences

- Every stage boundary needs a defined, versioned artifact. That surface is the
  real cost of this decision, and it is what the schema set in
  [decision 0016](0016-artifact-formats.md) makes manageable.
- Serialization time is negligible against a 456-second pipeline.
- Determinism is required of every stage, otherwise resume is not equivalent to
  a straight run. An integration test asserts the equivalence.
