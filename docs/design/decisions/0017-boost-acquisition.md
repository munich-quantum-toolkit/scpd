# 0017 — Boost.Polygon through FetchContent or vendored headers

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The prototype uses Boost in exactly one place: `construct_voronoi` inside the
capacity stage's Voronoi graph construction, roughly 150 lines out of 64,000.
Everything else — polygon extraction, distance transform, watershed, medial-axis
pruning — is hand-rolled.

Its build expects a Homebrew-installed Boost at a hard-coded include path.

## Decision

Boost.Polygon is acquired through FetchContent with
`BOOST_INCLUDE_LIBRARIES=polygon`, or by vendoring the headers.
**A user-installed Boost is never required.**

The dependency is kept. Writing a Voronoi implementation is precisely the kind
of work the project's minimalism rules direct us away from when a mature library
exists.

## Alternatives considered

**Require a system Boost.** Rejected: it is what makes the prototype unbuildable
on a machine that is not the original author's.

**Drop Boost and hand-roll Voronoi construction.** Rejected on the same rule
that keeps the dependency.

**Replace it with a single-header Delaunay library**, deriving the Voronoi
diagram as its dual. Genuinely lighter and would remove Boost entirely. Deferred
rather than rejected: converting to the specific edge and cell structure the
watershed code consumes is real work with real risk, and the port-first rule
says not to take it on now. Worth revisiting once the capacity stage is under
test.

## Consequences

- This is the least-proven dependency choice in the design. If
  `BOOST_INCLUDE_LIBRARIES` does not cleanly yield a `Boost::polygon` target,
  vendor the headers rather than reaching for a system package.
- The dependency stays confined to one module, `MQT::ScpdGrid`, which keeps the
  later replacement option open.
