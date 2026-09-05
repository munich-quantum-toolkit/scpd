# 0022 — The core owns design-rule checking

- **Status:** Accepted
- **Date:** 2026-09-01
- **Amended by:** [0024](0024-wire-loop-is-active.md)

## Context

The roadmap's acceptance criteria required that "KLayout design-rule checking on
the exported GDS agrees with the core's own checker, so the two are not marking
each other's homework."

**There is no KLayout design-rule checking to agree with.** The prototype's
`klayout/` package is a GDS/OASIS exporter plus a viewer panel; not one file in
it mentions DRC. The criterion described a cross-check that had never existed,
and the package's own README asks for the opposite arrangement: once a
`MinClearanceReport` exists as JSON, its violations belong in the viewer, which
wants the core to hand it a structured report it can highlight.

What the prototype does have is two of the eight rules that matter, implemented
twice over: `FinalGrid::verify_min_clearance` (483 lines, router cells) and
`QubitLayout::verify_min_clearance` (301 lines, layout units) check wire
clearance in the two different coordinate systems the two stages work in, and
`verify_crossing_orthogonality` (180 lines) checks that wires cross feedlines at
right angles. The other six rules do not exist. Two of them look as though they
do: obstacle clearance is *marked* as a keepout before every search but never
verified afterwards, and `lengthSatisfied` is assigned `false` at all eight
sites that write it and never computed.

## Decision

**DRCPolice** — `DrcPolice` in a new `MQT::ScpdDrc` library — is the tool's
design-rule checker. It runs after the Final stage and again after Finalize, and
writes a schema-defined `DrcReport` to `drc.json`.

Eight rules. Four are **active**: wire clearance, feedline orthogonality,
obstacle clearance, and resonator length within `resonator_length_tolerance`.
Four are **advisory** — wire loops, component overlap, minimum straight length,
minimum bend radius: compiled, unit-tested, and skipped unless `--drc-all` or
`[drc] all = true` runs them.

[Decision 0024](0024-wire-loop-is-active.md) moves wire loops to the active set,
making the split five and three. The prototype gained a working implementation
after this record was written.

Each rule is written **once**, against a `DrcView` that the calling stage
supplies in its own units — router cells for Final, layout units for Finalize. A
rule converts the design rules into the view's unit through `cells_for` or uses
them directly, so the two spaces cannot drift the way the prototype's two
clearance checkers did.

A violation of an active rule does not abort the run. The pipeline finishes and
the CLI exits nonzero.

## Alternatives considered

**Write KLayout DRC rule decks and require agreement.** What the original
criterion wanted, and a genuinely independent implementation would be worth
having. Rejected as the *criterion*: it is a substantial body of new work the
prototype never had, and only wire clearance and obstacle clearance map cleanly
onto layer-based GDS DRC — orthogonal crossings, resonator length and bend
radius all need information that the flattened polygons no longer carry.
Recorded as possible later work for the two rules that do map.

**Put the checker in `MQT::ScpdIO`.** `ScpdIO` was already listed as owning
"design-rule reporting", and this avoids an eighth module. Rejected: eight rules
across two coordinate spaces is real geometric algorithm, and putting it in the
module that is otherwise pure serialization misdescribes both. It would also
mean a test could not check a rule without linking the artifact layer.

**Put it in `MQT::ScpdPipeline`.** It runs after two stages, so it could live
with them. Rejected: nothing outside the pipeline could then run a check, and
`pipeline` is meant to be the top of the dependency graph rather than a place
algorithms live.

**Ship all eight rules active.** Rejected as dishonest about their maturity. Six
rules are new code with no measured behavior on any benchmark; declaring them
enforced before anyone has seen what they report on a working chip would either
block the port on false positives or, worse, get switched off wholesale the
first time one fired.

**Abort the run at the first violation.** Rejected: it destroys the artifacts
and the plot needed to diagnose the violation. The prototype prints its
clearance report and keeps going, which is the right instinct.

## Consequences

- The module count goes from seven to eight. The graph stays acyclic:
  `drc → grid, design, geometry`, and both `pipeline` and `io` depend on `drc`.
- `mqt-scpd drc run/` re-checks an existing run directory without re-routing,
  and `plot --drc` overlays findings on the SVG — the request the prototype's
  viewer documents, answered from the same report.
- **The advisory rules are a promise to measure, not a place to hide work.**
  Phase 5 requires reporting what each one finds across all eight benchmarks;
  promotion to active is a separate, evidence-backed decision.
- Rule 1 gets *shorter* in the port. The prototype decides whether two wires
  share a component by parsing the component name out of a port label, with a
  special case for synthetic labels that embed another port's name; under
  [decision 0018](0018-port-roles-unassigned-and-assigned.md) that is a field
  comparison.
- `DrcReport` is schema-defined even though it is written as JSON, unlike
  metrics. A violation has to be machine-readable to be overlaid, diffed and
  gated on; the prototype's findings existed only as printed text, which is
  exactly why its viewer could not use them.
