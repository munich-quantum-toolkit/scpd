# Roadmap

The rewrite proceeds in six phases. Each ends at something runnable and
testable, so progress is demonstrable rather than asserted.

The starting point is the FridgeCAD prototype repository. Its algorithms are the
specification, not the source; see [porting notes](porting-notes.md) for what
each part of it does and what should not be carried over.

## Acceptance criteria for the rewrite

The port is complete when all of the following hold:

- All eight benchmark chips route with **zero unresolved wires** in the final
  stage.
- The 69-qubit benchmark completes in **no more than about 460 seconds**, which
  is the prototype's measured time.
- Total feedline angle cost is within an agreed tolerance of the published value
  of 188 on the 69-qubit benchmark.
- All eight benchmarks pass every **active** DRCPolice rule. Each rule has unit
  tests on geometry hand-built to violate it by a known margin and to sit just
  inside it, and an integration test asserts that the Final and Finalize views
  agree on the same design. The independence comes from fixtures authored
  against the rule — KLayout ships no design-rule checking to compare against,
  which is why the core owns the checker at all.
- Peak resident memory on the 69-qubit benchmark is at most about 2 GB, down
  from roughly 7.5 GB.

## Phase 0 — Foundation

Bootstrap the repository structure and the data model. No domain logic.

- Restructure from the single flat `mqt-scpd` target to the eight per-module
  targets described in [ARCHITECTURE.md](../../ARCHITECTURE.md).
- Adapt `AddMQTCoreLibrary.cmake` and `CompilerWarnings.cmake` from MQT Core.
- Write `schemas/*.fbs` and the `nox -s schemas` session that regenerates the
  committed C++ and Python bindings.
- Write the architecture documents and decision records.

**Done when** `uvx nox -s lint` and CI pass, and CI fails if the committed
generated code is stale.

## Phase 1 — Geometry, design model, I/O

The first phase that produces something a user can look at.

- `MQT::ScpdGeometry`, `MQT::ScpdDesign` — now thin: ports, the two role enums,
  design rules, and the parsed configuration — and `MQT::ScpdIO`.
- The `routing_config.json` loader and the `config.toml` loader, both
  validating, and one `config.toml` per benchmark chip.
- Pattern-based role classification, with the classification table printed by
  `mqt-scpd doctor` so a wrong regex is visible before a 456-second run.
- `plot.py` and `mqt-scpd plot`. This lands **now**, not in phase 5: it is the
  instrument for watching phases 2 through 4 make progress, and it is only cheap
  to build because it reads artifacts rather than pipeline internals.
- `export/klayout.py`: the GDS and OASIS adapter.

**Done when** all eight configurations load, `mqt-scpd doctor --config` passes
on every one, `mqt-scpd plot --stage layout` renders each chip under 2 MB,
`mqt-scpd render -o chip.gds` renders the *unrouted* chip, and
`mqt-scpd inspect` round-trips every artifact.

There is no generator and no converter. The benchmark chips are the prototype's
own inputs, unchanged, which is what makes the routing baseline directly
comparable. See
[decision 0020](decisions/0020-legacy-routing-config-as-input.md).

## Phase 2 — Grid and router

- `MQT::ScpdGrid`: rasterization, distance transform, watershed, packed obstacle
  grids.
- `MQT::ScpdRouting`: **one** Dubins A*, ported from the prototype's optimized
  router. The unoptimized twin is not ported.
- Fix the memory model while porting: one shared read-only obstacle grid across
  threads, per-thread scratch only, bit-packed masks where the prototype used a
  byte per cell.
- Fix the dangling-reference defect the prototype documents and works around
  rather than repairing.

**Done when** the prototype's 100-route stress test exists as a GoogleTest, and
peak resident memory on the 69-qubit benchmark is at most about 2 GB.

Memory is addressed here rather than later because it is what caps thread count,
and threading is the phase-6 performance work.

## Phase 3 — Solver and the planning stages

- `MQT::ScpdMilp`: the model abstraction, the HiGHS backend, MPS emission, and
  runtime backend selection.
- Capacity, Assignment and Global stages.
- Precompute `AssignmentInputs` so that model construction no longer calls into
  the capacity grid.

**Done when** the first three stages reproduce the prototype's objective values
on the 4- and 9-qubit benchmarks under **both** HiGHS and gurobipy. Agreement
between the two backends is what proves the abstraction is faithful rather than
merely compiling.

## Phase 4 — Routing stages

- Detail, Final and Finalize stages.
- Collapse the prototype's four live rip-up-and-reroute drivers into one
  parameterized implementation, keeping coupler placement and feedline routing
  as one fixpoint rather than two sequential steps. The coupler optimizer is
  ported as it stands; it is genuine algorithm, not duplication.
- The analytic-segment geometry IR, end to end.
- Validate the two derived values that are no longer literals: the detail-grid
  blockade radius (4–9 instead of a fixed 6) and the straight-start stub (10–11
  instead of a fixed 9).
- `MQT::ScpdDrc` with DRCPolice's four **active** rules — wire clearance,
  feedline orthogonality, obstacle clearance and resonator length — in both
  coordinate views, plus `drc.json` and `mqt-scpd drc`.

**Done when** all eight benchmarks route with zero final failures *and* pass the
four active rules.

DRC lands here rather than in phase 5 because "zero failures" is not a
manufacturability claim on its own: a route can complete and still violate a
clearance. Two of the four rules are ports of prototype code, and the other two
are the ones the prototype never had — obstacle clearance is only ever marked,
never verified, and `lengthSatisfied` is a stub that reads `false` everywhere.

## Phase 5 — Quality, tooling, documentation

- Metrics, `report.py`, `mqt-scpd benchmark`.
- DRCPolice's four **advisory** rules — wire loops, component overlap, minimum
  straight length and minimum bend radius — with `--drc-all` and the `--drc`
  plot overlay. Measure what they report on all eight benchmarks before
  proposing any of them for promotion to active.
- The property and integration test suites.
- User documentation and the CLI reference.

**Done when** the 69-qubit benchmark meets the runtime and angle-cost criteria
above and `uvx nox -s docs` builds clean with warnings as errors.

## Phase 6 and beyond

Not scheduled. Recorded so the interfaces are not compromised for them
prematurely.

- Parallelism in the detail and final routers, which the `const` stage
  interfaces are already shaped for.
- A second implementation of one or more stages, which is what the registry
  exists for.
- Flip-chip and multi-layer support, which requires a layer axis in the data
  model and inter-layer edges in the routing graph. Explicitly out of scope
  until the research direction is settled.

## Risks

1. **The two outer port sequences are hand-maintained input.** `all_outer` runs
   to about 330 entries on the 69-qubit chip, and a typo in it is a silently
   worse assignment rather than a compile error. This is the price of deleting
   the port-ordering subsystem, and it is a smaller risk than the generator it
   replaces — the sequences are copied verbatim from the prototype's own
   drivers, where they were already hand-maintained, just in C++. Mitigation:
   load-time validation that every label exists and matches exactly one role
   pattern, that `fixed_outer ⊆ all_outer`, and `mqt-scpd doctor` printing the
   resolved ring.
2. **HiGHS may be materially slower than Gurobi** on the largest assignment
   model. The mixed-integer stages are roughly 10 percent of wall time, so the
   exposure is bounded. If HiGHS cannot close the 69-qubit model, the honest
   outcome is HiGHS for continuous integration and small chips, Gurobi for
   publication runs, documented plainly rather than papered over.
3. **The eight-way heading assumption is packed into the A\* state index** and
   the flat primitive tables, where headings are packed as
   `(ang << 10) | path_id`, capping path identifiers at 1024. Named constants
   make it visible. Changing it is out of scope for the first release.
4. **Deriving the two grid-cell values changes routing behaviour.** The
   detail-grid blockade and the straight-start stub are literals in the
   prototype and become chip-dependent under
   [decision 0019](decisions/0019-design-rules-in-layout-units.md). A benchmark
   may regress. Mitigation: both are validated in phase 4 against the zero-fail
   baseline, and a chip that genuinely needs a different value gets a documented
   per-chip override — never a reverted rule.
5. **Boost via FetchContent is the least-proven dependency choice.** If
   `BOOST_INCLUDE_LIBRARIES` does not cleanly yield a `Boost::polygon` target,
   vendor the headers rather than requiring a system Boost.
