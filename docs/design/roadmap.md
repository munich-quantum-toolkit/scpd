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
- KLayout design-rule checking on the exported GDS agrees with the core's own
  checker, so the two are not marking each other's homework.
- Peak resident memory on the 69-qubit benchmark is at most about 2 GB, down
  from roughly 7.5 GB.

## Phase 0 — Foundation

Bootstrap the repository structure and the data model. No domain logic.

- Restructure from the single flat `mqt-scpd` target to the seven per-module
  targets described in [ARCHITECTURE.md](../../ARCHITECTURE.md).
- Adapt `AddMQTCoreLibrary.cmake` and `CompilerWarnings.cmake` from MQT Core.
- Write `schemas/*.fbs` and the `nox -s schemas` session that regenerates the
  committed C++ and Python bindings.
- Write the architecture documents and decision records.

**Done when** `uvx nox -s lint` and CI pass, and CI fails if the committed
generated code is stale.

## Phase 1 — Geometry, design model, I/O

The first phase that produces something a user can look at.

- `MQT::ScpdGeometry`, `MQT::ScpdDesign`, `MQT::ScpdIO`.
- `gen.py`: a lattice chip generator driven by short recipes.
- `convert.py`: converts the prototype's `routing_config` files into the new
  instance-based chip description.
- `export/klayout.py`: the GDS and OASIS adapter.

**Done when** `mqt-scpd gen benchmarks/recipes/69q.toml` followed by
`mqt-scpd render -o chip.gds` renders the *unrouted* chip, all eight legacy
configurations convert, and `mqt-scpd inspect` round-trips every artifact.

Converting the legacy configurations first is deliberate: it produces the
reference inputs that validate the generator, which protects the quality
baseline. See risk 1 below.

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
- Collapse the prototype's six near-identical rip-up-and-reroute drivers into
  one parameterized implementation.
- The analytic-segment geometry IR, end to end.

**Done when** all eight benchmarks route with zero final failures.

## Phase 5 — Quality, tooling, documentation

- Metrics, design-rule reporting, `report.py`, `mqt-scpd benchmark`.
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

1. **The generator must reproduce benchmark geometry faithfully**, or the
   baseline that makes "port, then improve" safe is lost. Mitigation: convert
   the legacy configurations in phase 1, commit the converted chips for the four
   smaller benchmarks as reference inputs, and validate the generator against
   them.
2. **HiGHS may be materially slower than Gurobi** on the largest assignment
   model. The mixed-integer stages are roughly 10 percent of wall time, so the
   exposure is bounded. If HiGHS cannot close the 69-qubit model, the honest
   outcome is HiGHS for continuous integration and small chips, Gurobi for
   publication runs, documented plainly rather than papered over.
3. **The eight-way heading assumption is packed into the A\* state index** and
   the flat primitive tables, where headings are packed as
   `(ang << 10) | path_id`, capping path identifiers at 1024. Named constants
   make it visible. Changing it is out of scope for the first release.
4. **Boost via FetchContent is the least-proven dependency choice.** If
   `BOOST_INCLUDE_LIBRARIES` does not cleanly yield a `Boost::polygon` target,
   vendor the headers rather than requiring a system Boost.
