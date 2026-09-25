# 0032 — The coupler couples along the ring, at the target length

- **Status:** Accepted
- **Date:** 2026-09-15

## Context

The Final stage cuts every resonator back to a CPW coupler and drives the
couplers with feedline chains from launcher to launcher. Two things had to be
settled before the first coupler was placed: where along the resonator the
cut is made, and what the coupler's geometry is on the router grid.

The prototype cuts where the way left to the qubit is `meander_length`, a
figure it carries per chip (250 to 600 cells), and it never compares a
finished resonator with `target_resonator_length`; its `lengthSatisfied` is
a stub. Its coupler is an anchor with a body 20 by 3 cells whose long axis
lies along the heading the resonator leaves the anchor on; the resonator
turns a quarter right at the anchor, toward the side the feedline ports are
on, so its arc crosses its own feedline in the cell picture. The feedline
enters one end of the body and leaves the other.

A first port of the geometry here laid the body along the resonator's
straight run *after* the turn — the run that points inward, toward the
qubit — with the feedline along its far edge. Physically sensible, and wrong
for the chains: the feedline then runs across the ring, so every edge from
one coupler to the next has to turn back around the coupler and its
resonator, and on the 17-qubit chip a third of the edges found no way at
all.

## Decision

**The cut is at `target_resonator_length`**, the design rule, less the run
from the way's last cell to the port. `meander_length` stays what it was:
how long the outer routing makes the way so that the cut point exists. The
feedline phase draws every resonator again from its coupler and makes it the
target length exactly, within `resonator_length_tolerance`, with the strict
meander insertion; a way longer than that is a fail, because nothing spliced
in makes a way shorter.

**The coupling run comes before the turn, along the ring.** From the anchor
the resonator runs `coupler_length` cells straight on the heading across the
coupler's orientation, turns a quarter onto the orientation, runs the
straight start and joins what is left of its way. The body spans the
coupling run, `coupler_height` cells across on the side away from the turn,
so the turn never crosses the feedline; the feedline runs along the body's
far edge, across the orientation too — along the ring, where the chains run,
which is the axis the prototype's body has. The feedline may run either way
along the body, which is a third option axis beside the orientation and the
mirror.

**The chains come from the assignment.** The model already has them — a
chord between two resonators, a launcher or a termination at each anchor —
and the artifact now carries them as `Assignment.chains`, each a run of ring
nodes in ring order with the launcher slot of its first and its last anchor.
The prototype gets the same lists as `LAUNCHER_LINE_REQUESTS_` from its own
assignment.

## Consequences

- The coupler's created port is the `ResonatorSource` port, at the anchor;
  its orientation is the heading a wire arrives at the anchor on. The
  artifact carries the body as centre, rotation along the coupling run,
  length and height, which is what the Finalize stage rebuilds it from.
- A resonator and the edges of its own chain run beside each other along
  the coupler on purpose. The clearance rule, the crossing rule and the
  router's fence leave the room around a coupler's anchor open between the
  wires that share it, and nowhere else.
- Every option is priced by how much its two feedline edges turn, the
  prototype's objective, and the room another wire keeps under the body is
  a price rather than a rejection: every wire is drawn again under the
  feedline constraints.
