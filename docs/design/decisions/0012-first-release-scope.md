# 0012 — First release contains the full pipeline, one algorithm each

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

A rewrite can be sequenced to deliver breadth first, depth first, or the library
before any user-facing entry point.

## Decision

Version 0.1 is the whole pipeline working end to end — chip description in,
GDSII out, driven by the CLI, tested and documented — with exactly one
implementation per stage. The registry exists and each stage has one entry.

## Alternatives considered

**Skeleton first**, with a naive router, then port the real algorithms into it.
Earliest possible working loop, but a first release that routes badly is not
useful to anyone and invites the structure to be judged on results it was never
going to produce.

**Core library only**, deferring the CLI and export. Focuses on the data model,
but produces something no user can run, which removes the feedback that keeps a
rewrite honest.

## Consequences

- The architecture is proven on the real problem before variety is added.
- Every phase in the roadmap ends at something runnable, which is what makes the
  sequencing testable rather than aspirational.
