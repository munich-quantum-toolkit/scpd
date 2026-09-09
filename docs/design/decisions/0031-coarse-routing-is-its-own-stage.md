# 0031 — Coarse routing is a stage of its own

- **Status:** Accepted
- **Date:** 2026-09-09

## Context

The prototype routes a wire twice. First it walks every assigned connection
through the partitions of the capacity grid, crossing a border only at one of a
fixed set of places; then the detail grid fills in the pixels inside each
partition. Its own benchmark table counts the failures of the two separately, as
`GlobalFail` and `DetailFail`, and times them separately as well.

The pipeline of this repository had no place for the first of the two.
[Decision 0014](0014-resumable-run-directory.md) fixed six stages, and the
coarse pass fits none of them: the Capacity stage runs before the assignment
exists and cannot route what has not been assigned, and folding it into the
Detail stage would hide the step that makes the wire budgets binding inside the
step that draws pixels.

Its name is the second problem. The prototype calls it global routing, which is
also what the field calls a router that assigns nets to coarse regions — but
`Global` in this pipeline is already the stage that solves the inner circuit,
and `route` is the command that runs the whole pipeline.

## Decision

The coarse pass is a stage of its own, called **Corridor**, between the
Assignment and the Detail stage. The run directory therefore numbers seven
artifacts rather than six:

```text
01-capacity.fb   02-global.fb   03-assign.fb
04-corridor.fb   05-detail.fb   06-final.fb   07-geometry.fb
```

`ICorridorRouter::run` takes the chip, the capacity plan, the assignment and the
configuration, and reports one corridor per connection: the partitions the wire
runs through and the slot it crosses each border at.

A wire carries its own identity through the stage. The prototype keeps only the
union of the chords per partition and recovers which wire is which afterwards,
by matching path ends; that costs it a cell at each end of every wire and makes
its threaded pre-routing depend on the order the threads finished in.

## Alternatives considered

**Fold it into the Detail stage.** No renumbering, and the corridors would be
derived state like the grids. Rejected: the two passes fail for different
reasons and are worth measuring apart, and a run that resumes at the detail
router should not re-solve the corridors it already has.

**Call the new stage `global` and rename the inner-circuit stage.** Closest to
both the prototype's vocabulary and the field's. Rejected: it renames an
artifact of a released pipeline, reopens
[decision 0026](0026-global-runs-before-the-assignment.md), and buys a name that
the word `corridor` — already this repository's term for the free space a wire
runs through — carries just as precisely.

## Consequences

- The artifact numbers after `03` all move up by one. This is a breaking change
  to the run directory and is recorded in `UPGRADING.md`.
- `PartitionBorder.budget` becomes what its own comment always claimed: the
  number of wires the border can carry, counted as the crossing slots the stage
  actually uses. Both stages take the slots from one function and one pitch, so
  the budget and what is routed across it cannot disagree.
- That pitch is a stage parameter, `[stages.capacity] crossing_pitch`, and not a
  design rule. It says how finely a border is divided for planning; how far two
  wires keep apart is the detail router's answer and the design-rule check's
  verdict. The prototype divides at its wire spacing less twenty units and
  records no reason for the twenty, and on the 45-qubit chip that difference is
  one wire with a way through and one without.
- A stage that reports what it planned can be drawn. `plot --stage corridor`
  shows every wire's way through the partitions and every slot, taken or free,
  which is the picture that says whether a corridor is congested.
