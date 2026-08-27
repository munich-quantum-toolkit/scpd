# 0013 — Testing without golden metric pins

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype has no test suite. The benchmarks are the tests, and correctness
is read off scraped log lines. MQT requires tests for every change.

The output of a routing run is a large geometry, which makes the obvious
approach — pin the output and assert it does not change — tempting.

## Decision

Four layers, none of them a pinned metric.

| Layer        | Location                   | Content                                                                                                       |
| ------------ | -------------------------- | ------------------------------------------------------------------------------------------------------------- |
| Unit, C++    | `test/<module>/`           | Dubins arc construction, distance transform, point-in-polygon, watershed, model assembly, artifact round-trip |
| Unit, Python | `test/python/unit/`        | Schema validation, chip generator, converter, KLayout export                                                  |
| Property     | `test/python/property/`    | Invariants over generated chips                                                                               |
| Integration  | `test/python/integration/` | CLI end to end on the small benchmarks; resume equals an uninterrupted run                                    |

Properties assert what makes a routing result *valid*, for any input: every net
connected; no wire pair closer than `minWireSpacing`; every arc radius at least
`minBendRadius`; every path endpoint coincident with its declared port; every
resonator length within tolerance of target.

Quality is **tracked, not asserted**. `mqt-scpd benchmark` runs the suite
against a recorded baseline and reports per-chip deltas in failures, angle cost,
wire length and runtime. It fails only on things that are unambiguously wrong —
a nonzero final failure count, or a design-rule violation — never on a metric
moving. It runs nightly or on demand, not in pull-request continuous
integration.

## Alternatives considered

**Golden files pinning routed output or metrics.** Cheapest to write and
directly protects the published baseline. Rejected: it would block exactly the
runtime and quality improvements the rewrite exists to enable, every legitimate
change churns the goldens, and a passing golden test says nothing about why the
output is correct.

**Properties only, with no quality tracking at all.** Robust and never needs
regeneration, but silent quality regressions in wire length or angle cost would
pass unnoticed.

## Consequences

- Improving an algorithm does not require regenerating fixtures.
- Quality regressions are visible in a benchmark report rather than as a test
  failure, which means someone has to read the report. That is the accepted
  trade.
- KLayout design-rule checking on exported GDS provides an independent check of
  the core's own checker.
