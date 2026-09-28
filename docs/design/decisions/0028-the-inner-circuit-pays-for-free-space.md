# 0028 — The inner circuit pays for the free space it crosses

- **Status:** Accepted
- **Date:** 2026-09-09

## Context

The Global stage solves the inner circuit as a flow on the Hanan lattice each
capacity chain induces. A lattice edge says a wire *may* run somewhere. It does
not say the corridor it runs through has room for it, and until now nothing
did: the capacity chains were computed, written to the artifact and drawn, but
the model that decides where a wire goes never read them.

The effect was not subtle. Every inner target was served from the nearest port
that happened to be a node of the same lattice, which is usually a port of the
very coupler the target sits on. `Coupler4_5.port1 -> Coupler4_5.port0` is a
wire from a component to itself. The ring the Assignment stage consumes is
built from where the circuit surfaces, so the ring was wrong too — on the
9-qubit chip it came out with 24 ports where the prototype has 30, and none of
the six surfacing coupler ports were among them.

Two further defects kept the constraint from binding even where it existed:

- The bridge coupling was read **per lattice**. A bridge's two ports sit on
  opposite sides of a component's artwork and therefore usually in different
  chambers, so they are nodes of *different* lattices. Requiring both ends in
  one lattice meant the constraint was skipped for exactly the bridges it was
  written for, and a wire could start at a bridge port out of nothing.
- The ring kept a gated port only when the port itself was a source. The source
  is the *inner* end of the bridge; the ring names the outer end. The test
  could never be true.

## Decision

Each capacity chain that describes outer free space carries an integer flow of
its own, alongside the binary flow on the lattices:

- its launcher supplies it, at most one wire per target;
- each of its gates conserves what it takes and passes at most its capacity;
- each of its targets draws one wire;
- where a target is the **outer end of a bridge**, what it draws is exactly
  what the inner circuit sends out through that bridge.

That last clause is the point of the whole record: an inner wire may surface at
an outer port only when the free space behind that port can carry it to a
launcher.

A chain whose targets the inner circuit already owns is left out — constraining
those against themselves would say nothing. The bridge coupling is taken over
all lattices, and a bridge that surfaces on the ring is coupled only through
the chain capacity, because its far end is outside and on no lattice at all.

**A demand exists only where a supply can reach it.** The walk starts at every
launcher of the chain and stops at a gate with no room left; a target it does
not reach draws nothing, and nothing may surface there either. Without this the
model is not merely wrong but unsolvable: 27 of the 73 chains of the 17-qubit
chip have no launcher at all or are cut off from theirs by a shut gate, and
insisting that each of their targets draws a wire made the whole stage
infeasible. The prototype has a local version of the same rule — it checks only
the gates immediately beside a target — which is enough for its own inputs and
not in general.

## Consequences

On the 9-qubit chip the stage now reproduces the prototype's inner circuit: the
same four Hanan grids over the same ports, four of its six connections
identical, and the same 30-port ring and 10 resonators. The assignment's
crossing count moves from 10 to 15, which is the prototype's.

Of the two connections that differ, one is a tie and one is deliberate.

`Qb5.port1` is served from `Coupler4_7.port4` where the prototype uses
`Coupler7_8.port2`. Both cost 2844.9 on the lattice, so the two solutions are
the same price and the solver is free to return either.

`Coupler5_8.port0` is served from `Coupler8_9.port4` where the prototype uses
`Coupler6_9.port4`, and here we are simply cheaper: 1368 against 2438 by true
rectilinear length. The prototype's objective sums `weight * var[u][v]` over
its edge list, and its edges are emitted with `u < v` only, so **every arc that
runs the other way is free**. Its choice costs 798 under that objective and
ours 1333, which is why it prefers the longer wire. We price both directions;
the difference is measured here rather than reproduced.

The chain trees themselves still differ in 15 of 38 cases on the 9-qubit chip,
in how many gates lie between a target and a launcher. That follows from the
bottleneck set, which differs by the deliberate deviations of
[decision 0019](0019-design-rules-in-layout-units.md) and the grid frame, and
is recorded in `GridMetrics`. The target sets of all 38 chains agree exactly.

> **Amended 2026-09-09.** The demand of a target is on the **port**, over every
> lattice at once, not on a lattice node. A chamber border runs between two
> capacity chains, so a target beside one is a node of both, and a demand per
> lattice asked for a second wire to a port that already had one. That second
> wire is not merely waste: it takes supply the chip has, and the target it was
> taken from is then reported unreached. On the 17-qubit chip four targets sit
> in two lattices each; the model built 14 wires for 11 served targets and
> reported 3 unreached, where 13 wires serve 13. This is the same correction
> the bridge coupling needed above, in the one place it had not been made.

