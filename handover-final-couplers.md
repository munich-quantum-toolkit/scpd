# Phase 4, step 3 → the couplers, the feedlines and their refinement

Written for whoever finishes the Final stage. Its first two phases are built,
verified and drawn; the three that follow are specified here, with what the
first two learned and what cost time.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. **Nothing is committed — the user commits
  per phase.** Leave your work in the tree.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only. The Final stage is `include/fiction/layout/FinalGrid.cpp`, 23 413
  lines.
- What is finished: [summary-final-routing.md](summary-final-routing.md).

## Where the stage stands

`06-final.fb` is written. Its five phases each leave a snapshot, and two of
them carry wires:

```text
inner  outer  couplers  feedlines  refined
  ●      ●       ○          ○         ○
```

The three empty ones are yours. They are written out empty rather than left
out, so a reader and a renderer see the same five phases whatever a build
carries — `plot --stage final --phase couplers` already works and draws
nothing.

## What you inherit

`Driver` in `src/pipeline/FinalRouter.cpp` is the one rip-up-and-reroute loop.
The prototype writes it out four times, once per phase, and the four differ
only in their parameters; `Pass` is that parameter set. Phases 4 and 5 are two
more calls to `sweep` and `refine`, over a different wire list.

| Piece | Where |
| --- | --- |
| `Driver::sweep`, `Driver::refine`, `Pass` | `src/pipeline/FinalRouter.cpp` |
| `Field` — copper and clearance per cell, charged and discharged per wire | same file |
| `Scene` — the router grid, the mask with the keepout, the targets | same file |
| `routing::spliceCouplerDogleg`, `buildDogleg`, `CouplerDoglegOptions` | `include/mqt-scpd/routing/CouplerInsertion.hpp` |
| `DubinsRouter::routeOrthogonal`, `buildOrthogonalConstraints`, `setSingleCrossingFeedline`, `crossingAllowedOrthogonal` | `include/mqt-scpd/routing/DubinsRouter.hpp` |
| `drc::checkCells` — wire clearance, wire loop, obstacle clearance | `include/mqt-scpd/drc/Rules.hpp` |

## Phase 3 — coupler insertion

`run_optimized_cpw_coupler_insertion` (`FinalGrid.cpp:16940`). The roadmap says
to port the optimizer as it stands: it is genuine algorithm, not duplication.

- Per resonator, an option set: eight orientation offsets × mirrored × dogleg
  variants, each spliced with `routing::spliceCouplerDogleg` at the point where
  the way left to the qubit port reaches `target_resonator_length`. Reject an
  option whose body leaves the grid or meets an obstacle.
- A chain is launcher → couplers in ring order → launcher, read from
  `Assignment.ring`, `launchers` and `feeds`. Greedy local search per chain: an
  option is scored by the two real feedline edges to its chain neighbours.
- The committed coupler creates the `ResonatorSource` port and completes the
  connection (`design.fbs` `CpwCoupler`). Its body becomes an obstacle;
  re-placing one has to clear the old cells first — the prototype needs
  `remove_cpw_coupler_obstacles` for exactly that, and forgetting it leaves
  obstacles at abandoned positions.
- The footprint is `cells_for` of 200.0 × 26.0 layout units, not `20 × 3`
  cells. `FinalParams` already carries `coupler_length` and `coupler_height`.

## Phase 4 — feedline routing, as one fixpoint

`run_final_routing_feedline_choices_parallel` (`:13863`) over
`run_final_routing_feedline` (`:3535`).

- Route each chain edge with `routeOrthogonal`.
- Then route the regular wires again with the feedlines as orthogonal-crossing
  constraints, so a wire may cross a feedline only at a right angle. The first
  and last edge of a chain are hard obstacles for every wire.
- The repair loop is what makes this one fixpoint and not two steps: when a
  chain does not route it turns an already-placed coupler to one of its stored
  options and re-runs only the wires that touched it. `IFinalRouter` is shaped
  around that — one call does both.
- DRC rule 2, `feedline-orthogonality`, lands here. `CheckedWire.feedline`
  already exists and rule 1 already skips a coupler-to-coupler feedline and
  counts it in `feedlines_skipped`.

## Phase 5 — feedline refinement

`run_final_routing_feedline_refinement_parallel` (`:12391`): `Driver::refine`
over the feedline wires. It is already written — it re-routes every wire of a
list against the centring price and rolls back when the wider constraint does
not route.

## Lessons from phases 1 and 2

**1. Follow the prototype's call graph, and read what it does with a failure.**
The prototype's rip-up leaves a neighbour let go of after a successful search
and never checks what the wire it drew is worth against the wires that come
back. It gets away with it because it holds no clearance against them. Here the
wire is judged **after** everything is put back, and counts as routed only when
its way holds the rule against every other wire. That one change took the
21-qubit chip from ten violations to one. **Do not weaken it in phases 4 and 5.**

**2. The router and the check have to measure the same thing.** The search
keeps nineteen whole cells because a search works in cells; the rule is 185
layout units, which is 18.5 of them. For a while the driver judged a committed
wire by the guard field — nineteen cells — and so chased twenty encounters on
the 17-qubit chip that the check forgave, round after round. A committed way is
judged in layout units now, by `conflictsIn`, and by the same pairwise junction
test `drc::checkCells` makes. Watch for this whenever a new constraint is added:
what the search refuses may be stricter than what the design asks for, and only
the second belongs in a verdict.

**3. Settled means the way it has is legal, not that this attempt found a new
one.** A wire whose search fails and whose way was legal all along is finished.
Returning false there kept eleven wires of the 17-qubit chip open for every
round of the sweep, each costing up to twenty searches and moving nothing.

**4. A search that ignores the rule is not a search that ignores the copper.**
The last resort of the sweep routes the wire with the room of the others
ignored — their copper still solid — and lets go of every wire whose room lies
over that way. It is what finds the wires actually in the way, which the sweep
over ring neighbours cannot: the wire blocking this one need not be a
neighbour.

**5. The rounds stop paying long before they run out.** Measured on the
21-qubit chip: the count of open wires settles by the second round and the
remaining twenty-eight change no byte. The pass stops after four rounds without
progress, which is a factor of four in runtime and no loss at all. Look for the
same before turning any round count up.

**6. Two encounters are not violations, and the router and the check have to
agree on both.** A junction — two terminals within one wire spacing — and two
wires that end on one component. The ports of a component sit closer together
than the wire spacing and each has to be reached, so their approaches converge
and no arrangement holds them apart. The prototype's own `verify_min_clearance`
makes both exemptions. Getting this wrong in either direction costs a day: too
narrow and the 17-qubit chip cannot route at all; too wide and the check passes
things it should not. What is implemented is the narrow form — the exemption is
geometric **and** bound to the component, so a wire may enter a cell another
wire only guards when both end on the same component and the cell is within one
and a half spacings of one of the two terminals.

**7. The straight run out of a source is a fixed place, like the source
itself.** The router adds it to the way *after* it has searched, so no search
ever sees it. A wire drawn across another wire's run is a violation the search
could not have refused. They are charged before the first wire is drawn, and
only the wire being searched for is let out of its own.

**8. Measure with a script, not with the test suite.** `measure_final.py` in
the session scratchpad reads `06-final.fb` back and prints drawn, undrawn,
violations and self-meetings per chip in seconds. Every defect above was found
with it. `mqt-scpd drc <run>` now does the same through the shipped checker.

**9. The build traps.**

- `uv sync` does **not** rebuild the extension when only C++ changed. Use
  `uv pip install --python .venv/bin/python --no-build-isolation --no-deps
  --reinstall-package mqt-scpd -e .` and check behaviour, not the `.so` time.
  **Three changes in a row produced byte-identical output** here, and only one
  of the three was really a no-op.
- `cmake --build --preset release` does not pick up a new source file until the
  glob is re-run; touch the module's `CMakeLists.txt`.
- `mqt-scpd plan --stage <name>` reads the config **from the run directory**,
  not from the one named on the command line. Copy it over before an
  experiment, or the override does nothing.
- `mqt-scpd plan -v` is the fastest way to see what a change did. Every round
  says how many wires it tried, how many settled, how many are still open and
  how many have no way at all; a mechanism that changes nothing shows up as an
  identical trace.
- `uvx nox -s stubs` emits invalid Python for the `global` stage, so
  `python/mqt/scpd/pyscpd.pyi` is maintained by hand.

## What is open

1. **Two chips do not hold the rule everywhere.** The figures are in
   [summary-final-routing.md](summary-final-routing.md). Every wire is drawn on
   all eight; what is left is a handful of pairs that come closer than 185
   layout units, most of them by a few units and one of them by a lot. They are
   in `drc.json` of the run, with the place and the distance.
2. **The meander is not built.** `FinalParams.meander_length` is read and
   converted and nothing uses it yet, so a resonator's way is as long as its
   route makes it. The prototype meanders during the outer routing
   (`dubin_router_opt.hpp:2182`), and the coupler splice point of phase 3
   depends on the way being longer than the target.
3. **`run_final_routing_parralel`'s lenpoint constraint is not ported.** It
   keeps a disc clear around the point where a resonator reaches its target
   length, which is where phase 3 will splice the coupler. Without it the
   coupler's head may have no room. Read `outer_res_lenpoint_clearance` in
   `FinalGrid.hpp` before phase 3.
4. **Nothing is compared against the prototype's own output**, only against its
   formulation as read and the counts in its checked-in run logs.
