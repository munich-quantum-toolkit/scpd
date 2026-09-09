# 0029 — A feedline end is fed between two launchers

- **Status:** Accepted
- **Date:** 2026-09-09

## Context

The Assignment stage lays the ring of outer ports onto the ring of launcher
slots, and the artifact said which launcher each ring node was given. Drawn,
that put every resonator on a launcher, including the ones that *end* a
feedline run.

That is not where a run actually starts. A feedline is a chain of ports between
two ends. A node in the middle is reached from both sides. A node at an end is
reached from one side only, and the wire that feeds it has to come past the
other — from the stretch between its own launcher and the launcher its
neighbour on that open side sits on. Drawing it on a launcher puts the feed at
a point no wire runs through.

The prototype makes this explicit. After solving,
`OrderedAssignmentGraph::compute_min_overlap_assignment_physical_aware` walks
the resonators with exactly one assigned edge and calls
`CapacityGrid::add_coupler_launcher_ports(previous, own, 1, "first" | "second")`
for each. That interpolates a new launcher slot between the two named ones —
for one port, their midpoint — registers it as a launcher slot of its own and
reassigns the resonator to it. Which neighbour is taken is the side the run
does *not* continue on.

## Decision

The Assignment artifact carries, parallel to its ring, **where each node is
actually fed from**:

```
/// Where each ring node is actually fed from, parallel to `ring`.
feeds: [geometry.Point] (required);
```

For a node in the middle of a run that is its launcher slot's own position. For
a resonator that ends a run it is the midpoint between that slot and the slot
its neighbour on the open side was given; where the two are the same slot there
is nothing to interpolate and the launcher stands.

A point, not a port reference: the slot a feed starts at is not a port of the
chip and inventing one would put a port into the chip that its input never
declared. Everything the later stages need is the position.

## Consequences

Rendering follows the artifact, as [decision 0021](0021-debug-rendering-in-python.md)
requires: the assignment chord is drawn to the feed point, and a feed that is
not on a launcher slot is drawn as a launcher marker of its own, so both the SVG
and the GDS show the derived slots. On the 9-qubit chip all four feedline ends
are fed between launchers; where an end has the same launcher on both sides
there is nothing to interpolate and it stays where it was.

`launchers` stays as it was. It still says which launcher a node belongs to,
which is what the utilisation constraint counts and what the later stages route
to; `feeds` says where the wire starts.
