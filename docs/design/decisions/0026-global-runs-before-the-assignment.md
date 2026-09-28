# 0026 — The Global stage runs before the Assignment stage

- **Status:** Accepted
- **Date:** 2026-09-08

## Context

[Decision 0005](0005-stage-interfaces-and-registry.md) fixed the shape of a
stage but not the order of the six, and the architecture documents listed the
Assignment stage before the Global stage: the assignment would decide which
resonator a launcher feeds, and the global router would then solve the inner
circuit inside that decision.

Implementing the two showed that the prototype runs them the other way round,
and that it has to. `QubitLayoutOptimizer::buildModelInnerCircuit` returns the
outer port sequence the assignment then consumes, and appends to the resonator
list the outer-facing coupler port at which each inner resonator surfaces. All
eight drivers call it before `OrderedAssignmentGraph::from_port_sequences`.

The reason is not incidental. A coupler's artwork has routable ports on
opposite sides, and an inner wire crosses it and leaves through one of them.
Which one it leaves through is a fact about the solved inner circuit, and it
decides two things the assignment cannot work without: which outer ports carry
a wire at all, and which of them a launcher has to feed. An assignment made
before that is an assignment over the wrong ring.

## Decision

The pipeline order is **Capacity, Global, Assignment**, and the run directory
numbers the artifacts accordingly:

```text
01-capacity.fb
02-global.fb
03-assign.fb
```

`IGlobalRouter::run` therefore takes the chip, the capacity plan and the
configuration, and reports the ring; `IAssigner::run` takes the chip, the
capacity plan, the global routing and the configuration.

## Alternatives considered

**Keep the documented order and give the assignment the configured ring.**
Rejected: the configured ring carries both ports of every coupler bridge, so
the assignment would route wires to ports that no inner wire surfaces at, and
would miss the ones it does. The objective would not be comparable with the
prototype's, which is the acceptance criterion for these stages.

**Keep the documented order and iterate the two to a fixpoint.** Rejected as
unfounded: the prototype does not iterate, the port ring is not disputed
between the two stages once the inner circuit is solved, and a fixpoint that no
benchmark needs is machinery on speculation.

**Keep the file names and run them out of order.** Rejected: a run directory
whose `03` is produced before its `02` cannot express "running a stage
invalidates every artifact after it", which is what
[decision 0014](0014-resumable-run-directory.md) rests on.

## Consequences

- `ARCHITECTURE.md` and the pipeline document are corrected rather than the
  code bent to fit them.
- The Global stage is a no-op on a chip whose outer ring is its whole port
  ring — the 4-qubit benchmark. It writes an empty `02-global.fb` and reports
  the configured ring unchanged. That is a valid pipeline state, not a skipped
  stage.
- The Assignment stage reads its ring from `02-global.fb` rather than from the
  configuration, so a run that resumes at the assignment uses the ring the
  inner circuit actually produced.
