# 0005 — Typed stage interfaces with a name registry

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

This is research software; swapping an algorithm or adding a pass is a stated
requirement. The prototype had no seam at all — the pipeline was a flat script
in each of ten benchmark drivers, and stages reached into each other's state.

## Decision

Each stage is an abstract interface with concrete implementations selected by
name. A thin registry maps names to factories, so implementations can be chosen
from configuration or the command line and listed at run time.

Every `run` is `const` and takes its inputs by `const&`, returning a new value.

## Alternatives considered

**A generic pass manager over a shared mutable design database**, in the style
of OpenROAD. Maximum flexibility, but weaker type safety and it invites exactly
the hidden inter-stage coupling this rewrite exists to remove.

**Typed interfaces with no registry.** Marginally simpler. Rejected because
`--assigner=greedy` and `mqt-scpd list-algorithms` are worth thirty lines.

## Consequences

- The first release ships one implementation per stage; the registry has one
  entry each. That is expected, not a smell — the second implementation becomes
  a new file rather than a refactor.
- The `const` signature is the load-bearing part. It removes the by-value chip
  copies, prevents cross-stage mutation, and is what makes both resume and
  future threading possible.
