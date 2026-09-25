# Phase 4, step 3 → the Final stage: routing and meander complete — handover

Written for whoever builds the **coupler insertion**, phase 3 of the Final
stage. The inner and the outer routing are done and so is the meander: on every
benchmark every wire is drawn, the design rule holds everywhere, and every
resonator is as long as `meander_length` asks. This file says what the stage is
now, what changed to get there and why, what cost time, and what the coupler
insertion starts from.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. The outer routing is committed as `1957fe1`
  ("0 fails on final routing on all benchmarks"), its write-up as `6faeaae`, and
  the meander insertion — everything under *The meander* below, the `length`
  field of `FinalWire`, the per-chip `meander_length` of the benchmarks, the
  tests and the write-ups — as `cd8e597` ("0 fails meander insertion",
  2026-09-15). **The user commits per phase**; leave your work in the tree.
  `artifacts/*/drc.json` are tracked and show as modified after a run that
  changes a report.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only. `include/fiction/layout/FinalGrid.cpp` (23 413 lines) and
  `include/fiction/algorithms/routing/dubin_router_opt.hpp`.
- Write-ups: [summary-final-routing.md](summary-final-routing.md) is the record
  of every measurement, *The meander* first; [user_final.md](user_final.md) is
  the guide to the code and every command;
  [handover-final-couplers.md](handover-final-couplers.md) briefs phases 3 to 5
  as they were planned after the outer routing — read it with *Next* below,
  which says what the meander changed for phase 3.

## Where the pipeline stands

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb  06-final.fb
```

Six of seven artifacts are written. In `06-final.fb` the phases `inner` and
`outer` carry wires, each with its rendered length; `couplers`, `feedlines`,
`refined` are written empty. At the setting every benchmark ships (`rounds = 6`,
`max_relaxation = 5`, `refinement_rounds = 0`, and the prototype's
`meander_length` per chip):

| chip | wires | drawn | open | short | **Fails** | `drc` pairs | `meander_length` | resonators, shortest to longest | final stage |
| ---- | ----: | ----: | ---: | ----: | --------: | ----------: | ---------------: | ------------------------------: | ----------: |
| 4q   |    12 |    12 |    0 |     0 |     **0** |           0 |             2500 |                    2509 –  2536 |      0.04 s |
| 9q   |    36 |    36 |    0 |     0 |     **0** |           0 |             3000 |                    4021 –  5149 |       0.4 s |
| 17q  |    72 |    72 |    0 |     0 |     **0** |           0 |             3000 |                    3007 –  5640 |       0.9 s |
| 21q  |    78 |    78 |    0 |     0 |     **0** |           0 |             4000 |                    6533 – 10061 |       3.7 s |
| 33q  |   118 |   118 |    0 |     0 |     **0** |           0 |             6000 |                    7890 – 13069 |         8 s |
| 45q  |   157 |   157 |    0 |     0 |     **0** |           0 |             6000 |                    8546 – 16163 |        10 s |
| 57q  |   201 |   201 |    0 |     0 |     **0** |           0 |             6000 |                    7979 – 19768 |        31 s |
| 69q  |   241 |   241 |    0 |     0 |     **0** |           0 |             6000 |                   13025 – 27221 |        43 s |

A resonator's length here is its way plus the run from the way's last cell to
the port, in layout units, as `-v` prints it. `artifacts/` was regenerated in
full with the meander on 2026-09-15: every stage, SVG and GDS of `layout`,
`capacity`, `global`, `assign`, `corridor`, `detail`, `final`, the five final
phases as SVG, and `drc.json`. `--stage aligned` cannot be drawn yet ("arrives
with phase 4").

## The stage as it is now

One driver, `Driver` in `src/pipeline/FinalRouter.cpp`, runs the inner pass and
the outer pass with the same loop. Read it in this order:

| Piece                                        | Where                                                   | What it is                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| -------------------------------------------- | ------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Tuning`, `tuningOf`                         | `:85`, `:135`                                           | every knob converted onto the grid once. `spacing` is the rule in cells, unrounded; `meanderLength` the resonator length in cells and `meanderLengthUnits` in layout units; `approach*` the port band's geometry                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `Scene`, `sceneOf`                           | `:201`, `:254`                                          | the mask with the keepout, the port bands, the target of every port                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `Field`                                      | `:435`                                                  | copper and clearance per cell. **Only the count reads it**; no search is fenced by it                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `Wire`, `Pass`                               | `:564`, `:613`                                          | one connection — for a resonator with `anchorGap`, the run from its last cell to the port, and `tooShort`; one pass's parameters                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `Driver::lengthen`, `routing::insertMeander` | `:920`, `include/mqt-scpd/routing/MeanderInsertion.hpp` | the meander: one loop spliced into a straight run of the way, inside the corridor of the search that found it, as near the qubit as it fits. First fit in phase 1, cheapest by the price field in the relaxation and the refinement                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `Driver::sweep`                              | `:1022`                                                 | the rounds; every pass starts with every wire on its Detail way (`seed`, `:1377`)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `Driver::attempt`                            | `:1458`                                                 | **the prototype's two phases and nothing else**: phase 1 fenced by the two ring neighbours; phase 2 the relaxation along the sweep, the wire let go of is crossable, the fence is the last one let go of plus the opposite neighbour, the lane is free and everything outside it priced, the wires let go of are priced at growing distances (`priceLane`, `:1908`), their approaches ten times over (`priceApproaches`, `:1951`). A resonator's way found is lengthened before it is taken, and one without room for its meander is no way. A way found is taken; on failure the wires let go of go back, and a resonator keeps a plain way it found |
| `Driver::fence`, `buildCorridor`             | `:1792`, `:1724`                                        | `mark_obstacles` and `expand_path` of the prototype                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `Driver::meetAt`, `couldMeet`                | `:1622`, `:1827`                                        | the junction: two terminals within one clearance, everything within 1.5 of either exempt. **Geometry alone**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `Driver::conflictsOf`, `failsOf`             | `:1606`, `:1154`                                        | the count: `Fails = unrouted + open + short`, each wire once, by the check's own test and by the sampler's length                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `Driver::sayResonatorLengths`                | `:1190`                                                 | one `-v` line per resonator at the end of the stage, and one over all of them                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `Driver::drawSearch`, `drawGrid`             | `:2406`, `:2543`                                        | the debug pictures; a resonator's shows the way with its meander and what the lengthening said                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `src/pipeline/DebugSvg.hpp`                  |                                                         | the SVG painter                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `src/grid/PortBands.cpp`                     |                                                         | `bandLength`, `bandHalfWidth`, `stampBand`, `digTargetBeyondBand`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `src/drc/Rules.cpp`                          |                                                         | rule 1 wire clearance, rule 3 wire loop. Rule 4 obstacle clearance is out                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Everything the search decides by is in the picture `-d` draws of it, and `-v 1`
says one line per search. Use both before reasoning about a wire.

```bash
.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml -o artifacts/17q -v 1 -d
```

## The meander

A resonator's wire has to be `meander_length` long before the coupler is spliced
into it — longer than `target_resonator_length` (2500), because the coupler is
spliced where the way left to the qubit reaches the target and the wire is fed
on the ring, not at a port. The router draws it as short as it can, so one
meander is spliced into a straight run of it. Built on 2026-09-14 and 15; the
record is *The meander* in
[summary-final-routing.md](summary-final-routing.md).

**What it is.** `routing::insertMeander`
(`include/mqt-scpd/routing/MeanderInsertion.hpp`,
`src/routing/MeanderInsertion.cpp`) is the prototype's `meander_insertion`
(`dubin_router_opt.hpp:2182`) and `meander_insertion_proximity` (`:2325`) as one
function: the cells of the straight runs are walked in pairs — the smallest span
first, and from the qubit end, so that the smallest loop goes as near the qubit
as it fits — each end of a pair is turned across it by one or two moves of the
primitives, and one rectangular loop is built between the turned ends, as deep
as the missing length makes it. That loop is the one shape the prototype ever
accepts (`nMeanders == 1`). Every cell of the piece has to be enterable — the
driver hands it the corridor of the search, the band less the fence — and the
spliced path may not meet itself. Without a price the first fit is taken; with
one every fit is scored by the price of its cells and ten per direction change,
and the cheapest is taken. The length is what `samplePath` renders, and the loop
is built again a little deeper where the rendering falls short.

**Where the loop goes, and why.** The prototype walks the pairs from the source
and takes the first fit, which put every loop at the feed point: on 4q up
against the launcher pads, on 17q within thirty cells of the ring. That end is
where the coupler insertion cuts the way back to the target length — a loop
there is cut in two, and the coupler lands on a leg of it — and where the
feedline runs. The prototype's own pictures show its loops near the coupler only
because its feedline phase routes every resonator again from the coupler and
places the loop again from there; that phase is not built here, so the loop is
put where it survives the cut. On 4q the four loops sit before the qubits; on
17q, where the fan-in before a qubit leaves no room for a loop 26 to 51 cells
deep, they sit on the middle of the way. The user asked for this on 2026-09-15
after seeing the first placement.

**Where it hooks in.** `Driver::lengthen` (`FinalRouter.cpp:920`) calls it with
`meander_length` less `Wire::anchorGap`, the run from the last cell to the port,
and with the search's own price field in the relaxation and the refinement.
`Driver::attempt` lengthens every way a resonator's search finds before it takes
it; a way without room is set aside and counts as no way, so the relaxation goes
on, and when nothing comes of it the wire keeps that plain way, `tooShort`, for
the next round. `Driver::failsOf` measures every drawn resonator with the
sampler and counts one short as a fail; the `Fails:` lines say `short` beside
`unrouted` and `open`, `-v 1` names them and says per search
`meander: before → after cells for required`, `long enough` or
`no room for a meander` with the placements tried, and `-d` draws the way with
its meander. `-v` ends the stage on one line per resonator
(`Driver::sayResonatorLengths`): the way, the run to the port, the two together
against `meander_length`, and one line over all of them. `FinalWire.length` in
the artifact is the rendered length in layout units.

**What was measured.** At the prototype's own figures — `meander_length` 2500 on
4q, 4000 on 21q, 6000 from 33q up, the default 3000 on 9q and 17q, set in every
benchmark's `config.toml` — all eight chips end on `Fails: 0` with no `drc`
pair, in the time the stage took before. Only 4q (4 of 4) and 17q (5 of 17) have
a resonator that needs a loop; from 21q up every resonator is longer than its
figure as drawn, by thousands of units. Under load, at 14000 on 33q, every one
of the 33 resonators gets a loop and the chip still routes clean; at 6000 on 17q
no resonator is short but sixteen wires end within the rule of another, which is
the price of a loop 130 cells deep on the densest grid.

**Seven things are not the prototype's**, each measured or reasoned in the
summary: the pairs from the qubit end; a direct walk over the primitives for the
turned ends instead of the prototype's reflection trick, which names the wrong
move where a heading has two arcs to one exit; the loop's room checked from its
geometry; the rendered length verified and the loop deepened where the rendering
falls short; the spliced path tested for meeting itself; the priced fit scored
on the piece with prefix sums and spliced once, ten times faster; and no "forty
cells short is close enough" exit. The box is the router grid, because the mask
already blocks everything outside the ring of sources.

## What changed on 2026-09-13 to 15, most valuable first

Every item was measured over the eight chips at six rounds and five relaxations.

1. **The meander**, above, with `Fails = unrouted + open + short`, the `-v 1`
   notes per search, the `-v` length lines, `FinalWire.length`, and the per-chip
   `meander_length` in the benchmarks (a shipped config carries only what
   differs from the default; `doctor` enforces it, so 9q and 17q say nothing and
   run at 3000).
2. **The exemption for two wires ending on one component is gone** — from the
   check (`junctionsOf`), the fence (`couldMeet`, `meetAt`) and the count. It
   assumed a component's ports sit closer than the wire spacing; the two ports
   of a qubit sit 909 units apart on 17q, and wire 44 ran 140 units from wire
   43's approach unreported and unfenced. Taking it out took 33q from 18 fails
   with 10 crossings to **0**, and every other chip to 0 as well. The prototype
   forgives such a pair outright (`wires_connected` in `verify_min_clearance`);
   do not bring that back.
3. **The target sits on the first cell the straight-length rule allows.**
   `digTargetBeyondBand` stepped once before its first test, so every target lay
   two cells beyond the band: 120 units along an axis and 127 along a diagonal
   against a rule of 100. Now the band ends before the first cell whose centre
   lies `min_straight_length` from the port's *own* position and the target is
   that cell: 100–110 axial, 100–114 diagonal.
4. **`Tuning::spacing` was never assigned**, so the stage counted no wire as too
   close and its "0 too close" could not agree with `drc`. Set now; the `Fails:`
   line and the check count the same encounters.
5. **Every pass starts on the Detail stage's ways.** The sweep is a rip-up and
   re-route, and a rip-up needs something to rip: seeds are joined on the router
   grid with the straight stub in front and put down with copper and clearance.
   A wire still on its seed at the end is *unrouted* and gets no cells in the
   artifact, as the prototype drops it.
6. **`attempt` is the prototype's `run_final_routing`, verbatim in shape.** The
   field-fenced search, the verdict against the field, the relaxation against
   the sweep, the targeted rip and the rescue are all gone. What the summary
   calls "the four corrections" no longer describes the search.
7. **Rule 4 (obstacle clearance) is out of the check** because its raster did
   not exempt the port approaches the router's mask exempts, so it reported
   every stub out of a port. The keepout is still searched.
8. **`Fails:` after every stage**, `-v 1`, `-d`, and the tests read `rounds`,
   `max_relaxation`, `refinement_rounds` and `meander_length` from each
   benchmark's `config.toml` (`test/pipeline/Benchmarks.hpp`), so the suite runs
   what `plan` runs.

**Measured and not kept** — see the summary for the figures: a verdict by
penetration depth (more fails), a uniform top price on released rooms (more
fails), the approaches of the wires around priced ten times over (kept, but it
changed nothing: the crossings lay 26–510 cells from any terminal), doubling
that zone's width (one fail more on 33q), and the prototype's loop placement
from the source (legal everywhere, wrong where the coupler goes).

## Lessons that cost time

1. **A signed step times an unsigned count wraps.** `step.x * k` with
   `int8_t step.x = -1` and `uint32_t k` walked off the grid on its first step,
   and the fallback hid it for every port facing 180°, 270°, 135° or 315°. Keep
   every such loop in `std::int64_t`. Measure the result of a geometric change
   from the artifact, per wire, before believing it.
2. **Read the log before the theory.** `-v 1` showed within a minute that the
   33q crossings arose in *phase 1* of wire 47 — fenced only by 46 and 48 —
   after 43, 44 and 45 had relaxed into 47's room. No price on the relaxation
   could reach that.
3. **Look at the pictures before believing a placement.** The prototype's pair
   order put every loop at the feed point; `Fails: 0` and every length reached
   said nothing about it, and the artifacts showed it at a glance.
   `plot --stage final --phase outer` is a minute; a wrong placement is a day.
   The corners of a loop show a sawtooth in the plots, as every quarter turn of
   the router does (eleven of them on 17q outside the resonators): that is the
   raster of the primitive, not a defect of the loop, and the exact curves are
   the Finalize stage's.
4. **A shortcut in an exemption is a hole in the rule.** The component shortcut
   was in three places that all agreed with each other, so no test and no count
   could see it. Only the plot, which draws the clearance bands without
   exemptions, showed the overlap.
5. **Read the loop that splices, not the one that prints.** The prototype's
   `meander_insertion_proximity` accepts a way forty cells short as "close
   enough" and, through a stale index, scores a path that is not the
   candidate's. Neither is ported.
6. **A cell of a hand-built path is tagged with the move that *leaves* it, and a
   move's end cell belongs to the next move.** Get this wrong and the sampler
   renders every arc one cell off, which no test of the cells can see; the
   meander test renders the result and compares it with what the insertion
   reported.
7. **The priced variant is cheap only when the score is taken on the piece with
   prefix sums over the rest and the splice is made once, for the best.** Made
   as the prototype makes it, it took ten times as long on a 150-cell run.
8. **On the large chips the resonators are far longer than `meander_length` as
   routed**, so the shipped setting does not exercise the insertion there.
   Measure with a figure above the shortest resonator when the insertion changes
   (`summary-final-routing.md`, *Measured under load*).
9. **The prototype's parallel routing is not its sequential routing.**
   `run_final_routing_parallel` relaxes *both* ways and repeats each round up to
   four times within itself; `run_final_routing` does neither. When the user
   says "exactly the prototype", ask which one.
10. **The price model is the prototype's to the digit**:
    `100·Σ static + 100·wire[end]·(1 + swept)` per primitive, bends in
    hundredths of a cell, norms resolved with `llround` and a floor of 1.
11. **Diagonal bands are narrower than the rule.** `bandHalfWidth` truncates
    `19/√2/2` to 6, so a diagonal band is 13 cells = 183 units wide against
    185. Inherited from the prototype; untouched.
12. **Tooling.** `uv sync` does not rebuild the extension — use
    `uv pip install --python .venv/bin/python --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .`
    after every C++ change. `plan --stage X` reads the configuration from the
    run directory, not from `-c`. `uvx nox -s lint` fails on findings that
    predate this work (`noqa` codes in `cli.py`, `klayout.py`,
    `gurobipy_backend.py`, `test_planning.py`; an import order in `run.py`; one
    `ty` finding in `test_planning.py`), and its hooks
    **rewrite 108 files of the tree** — every Markdown file refilled, every
    `drc.json` re-indented, blank lines before closing namespaces removed — none
    of which the committed tree carries; revert what you did not write before
    reading `git status`. The local `clang-format` (21.1.7) would also re-wrap
    `src/routing/PathGeometry.cpp` and
    `include/mqt-scpd/routing/DubinsRouter.hpp` as committed, so changes there
    are applied by hand. `uvx nox -s schemas -- --check` reports the generated
    code stale until `FinalWire.length` is committed, because it compares
    against the last commit.
13. **`-d` is heavy on a chip that relaxes a lot** (one picture per search, 17q:
    166 pictures, 19 MB), and at the schema's defaults of thirty rounds the
    seeded sweep on 45q once took 109 minutes. Never run the Final tests at
    30/10 on 45q and up; the tests read the shipped 6/5/0.

## Verification

```bash
cmake --build --preset release
uv pip install --python .venv/bin/python --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .
ctest --preset release                                     # see the figure below
uv run --no-sync pytest test/python/unit                   # 141 tests
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  .venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c -v
  .venv/bin/mqt-scpd drc artifacts/$c
done
./build/release/test/routing/mqt-scpd-routing-test --gtest_filter='MeanderInsertion.*'
./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='*Final.ResonatorsReachTheMeanderLength/*'
```

Last full run on 2026-09-15, with the meander as it is: Python 141 of 141;
`ctest --preset release` 368 tests, 366 passed — the ten `MeanderInsertion`
tests and `Final.ResonatorsReachTheMeanderLength` on all eight chips among them.
Of the two that did not: `BenchmarkDetail.DrawsCopperTheRulesAllow/69q` hit
ctest's 1500 s timeout because the run shared the machine with the artifact
regeneration and the stress runs, and passes alone in 96 s; the run of
2026-09-14 before the placement change had 366 of 367 with only the failure
below. That one is `BenchmarkDetail.DrawsCopperTheRulesAllow/57q`,
**open and not of this step**: on 57q the Detail stage's wires start one cell
away from the point the corridor feeds them at ("wire 0 starts at (92, 1244) and
is fed at (1713.19, 38222.16)", and the same for 11, 100, 102, 107, 112, 118,
121 and more, all on the top and bottom rows of the ring). Nothing in
`Scene.cpp`, `CapacityPlanner.cpp`, `CorridorRouter.cpp` or `DetailRouter.cpp`
changed since `fc7e846`, so it was red before the outer routing. It smells like
the feed point of a launcher on 57q not falling on a detail cell centre; start
at `cellAt` in `DetailRouter.cpp` and the launcher slot in `Scene.cpp`.

## What is open

1. **Couplers, feedlines, refinement** — *Next* below, and
   [handover-final-couplers.md](handover-final-couplers.md).
2. **The meander is one loop, and nothing keeps it from the rest of its own
   wire.** One rectangular detour is the prototype's one shape; a loop whose
   legs come within the rule of the wire's own other runs breaks no rule of the
   check, which compares two wires, and the insertion refuses only a loop that
   meets the way.
3. **Rule 4** needs the port approaches exempted in its raster before it can
   come back (`sceneOf`, `keepoutExemptions` is the model).
4. **The refinement** (`Driver::refine`) is ported, lengthens a resonator again,
   and runs zero rounds everywhere; the prototype runs 3 to 15 rounds with the
   two neighbours hard at the clearance and a centring price of 6.
5. **The diagonal band width** (lesson 11).
6. `CheckedWire::components` in the DRC view is filled and read by nothing.
7. `BenchmarkDetail.DrawsCopperTheRulesAllow/57q` (Verification).

## Next: the coupler insertion

Phase 3 of the Final stage: `run_optimized_cpw_coupler_insertion`
(`FinalGrid.cpp:16940`), with `add_cpw_coupler` (`FinalGrid.hpp:521`),
`CPWCoupler` (`FinalGrid.hpp:402`), `CouplerOption` (`FinalGrid.hpp:547`),
`compute_cpw_coupler_insertion_point` (`dubin_router_opt.hpp:9977`),
`mark_cpw_coupler_obstacles` and `remove_cpw_coupler_obstacles`
(`FinalGrid.cpp:22707`, `:22666`). The roadmap says to port the optimizer as it
stands: it is genuine algorithm, not duplication.

**What it has to do.** For every resonator, cut the way at the point where the
way left to the qubit port is the target length, and put a CPW coupler there:
the body 200 × 26 layout units (`FinalParams.coupler_length`, `coupler_height`,
`cells_for` on the router grid — not the prototype's 20 × 3 cells), its readout
port at the cut on the heading the way has there, its two feedline ports on the
far side. The part of the way from the feed point to the cut is discarded. The
coupler's readout port is the `ResonatorSource` port the assignment left absent:
`CpwCoupler.port` carries it, the port list becomes the input ports followed by
the couplers' ports in coupler order (`design.fbs`, `CpwCoupler`), and the
connection is complete. The committed body becomes an obstacle and its clearance
room a charge on the field; re-placing a coupler clears the old cells first,
which the prototype needs `remove_cpw_coupler_obstacles` for and forgets at its
peril. The options per resonator are the prototype's `CouplerOption`: eight
orientation offsets × mirrored × the dogleg variants (`straight_length = 14`, a
second dogleg, an S-jog), each spliced with `routing::spliceCouplerDogleg`
(`include/mqt-scpd/routing/CouplerInsertion.hpp`, calibrated on
`samplePathFromSecond`) and rejected when the body leaves the grid, meets an
obstacle, or takes another wire's room. The chains launcher → couplers in ring
order → launcher come from `Assignment.ring`, `launchers` and `feeds`; the
greedy local search scores an option by the two feedline edges to its chain
neighbours, and the feedline routing of phase 4 turns a placed coupler to
another stored option when a chain does not route — the two phases are one
fixpoint, which is why `IFinalRouter` is one call. The `couplers` snapshot then
carries the resonators cut back to their couplers and `FinalRouting.couplers`
the couplers; `plot --stage final --phase couplers` and GDS layer 26 already
draw the phase.

**What the meander changed for it, and one thing to settle first.**

- Every resonator is at least `meander_length` long, way plus the run to the
  port, and the artifact says how long (`FinalWire.length`; `-v` prints the sum
  per resonator). The cut lands `length + gap − target` from the feed point
  along the way: on 17q at least 500 units in, on 4q at the feed point itself
  because `meander_length` equals the target there, and on the large chips
  thousands of units in, because their resonators are 7 900 to 27 000 units long
  as routed. **So on 21q and up the coupler does not sit on the ring**; the
  feedline of phase 4 has to be routed to it, as the prototype's
  `run_final_routing_feedline` routes it to the couplers wherever they are. Look
  at where the cut lands on 33q before designing the chains.
- **Which length is the target.** The plan (`docs/design/pipeline.md`,
  `handover-final-couplers.md`) says the coupler is spliced where the way left
  to the qubit reaches `target_resonator_length` (2500), and rule 7 of the
  check, `resonator-length`, judges the finished resonator against that figure
  with `resonator_length_tolerance`. The prototype's code cuts at
  `meander_length − target_anchor_offset − kUndershootMargin`
  (`FinalGrid.cpp:17327`), which is the meander figure, not the design rule.
  With `meander_length` at or above the target on every chip, the cut at the
  target keeps the whole loop only if the loop lies within the last target
  length of the way, which the placement at the qubit end makes likely but does
  not promise: a loop 100 cells deep spans 2 000 units of way. Decide which
  figure the cut uses, and check the loop against it with the `-v 1` notes and
  `FinalWire.length` before trusting a splice.
- `Wire::anchorGap` (cells) and `anchorGapUnits` hold the run from the way's
  last cell to the port; `Driver::lengthInUnits` measures a way the way the
  artifact does. Both are what a splice point's "length left to the qubit" is
  measured with.
- The loop is a rectangle of quarter turns 35 to 70 cells wide and 26 to 51
  cells deep; a dogleg spliced onto one of its legs is a legal path and a bad
  coupler. Reject an option whose splice point lies on the loop —
  `insertMeander` does not mark the loop, so mark it, or find it as the runs
  between the two quarter turns that face each other.

**Where it goes here.** A third pass in `DubinsFinalRouter::run`
(`FinalRouter.cpp:2824`), after the outer sweep and its refinement and before
the `couplers` snapshot; the chip is immutable, so the created ports live in the
output. Use `Field` for the room a coupler body takes, the same way a wire's way
is charged, so that the count and the check see it. Write the tests first: every
resonator has a coupler and a `ResonatorSource` port appended in coupler order;
the remaining way from the coupler to the qubit is the target length within
tolerance; no coupler body within the rule of a wire it does not belong to; the
snapshot and the picture. Add the `-v` lines the stage already has the shape of:
one per coupler, where it sits and how long the way left to the qubit is.
