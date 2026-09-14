# Phase 4, step 3 → the Final stage: outer routing complete — handover

Written for whoever builds the **meander insertion**, the next piece of the
Final stage. The inner and the outer routing are done: every wire of every
benchmark is drawn and the design rule holds everywhere. This file says what
the stage is now, what changed to get there and why, what cost time, and
where the meander goes.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. The outer routing is committed as
  `1957fe1` ("0 fails on final routing on all benchmarks"). Not committed:
  this file and two test repairs named under *Verification*. **The user
  commits per phase**; leave your work in the tree. `artifacts/*/drc.json`
  are tracked and show as modified after a run that changes a report.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only. `include/fiction/layout/FinalGrid.cpp` (23 413 lines) and
  `include/fiction/algorithms/routing/dubin_router_opt.hpp`.
- Write-ups: [summary-final-routing.md](summary-final-routing.md) is the
  record of every measurement; [user_final.md](user_final.md) is the guide
  to the code and every command; [handover-final-couplers.md](handover-final-couplers.md)
  briefs the coupler and feedline phases that come after the meander.

## Where the pipeline stands

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb  06-final.fb
```

Six of seven artifacts are written. In `06-final.fb` the phases `inner` and
`outer` carry wires; `couplers`, `feedlines`, `refined` are written empty.
At the setting every benchmark ships (`rounds = 6`, `max_relaxation = 5`,
`refinement_rounds = 0`):

| chip | wires | drawn | open | **Fails** | `drc` pairs | final stage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 | 0 | **0** | 0 |  0.04 s |
| 9q  |  36 |  36 | 0 | **0** | 0 |   0.4 s |
| 17q |  72 |  72 | 0 | **0** | 0 |   1.9 s |
| 21q |  78 |  78 | 0 | **0** | 0 |   6 s |
| 33q | 118 | 118 | 0 | **0** | 0 |   9 s |
| 45q | 157 | 157 | 0 | **0** | 0 |  10 s |
| 57q | 201 | 201 | 0 | **0** | 0 |  30 s |
| 69q | 241 | 241 | 0 | **0** | 0 |  41 s |

`artifacts/` was cleared and regenerated in full on 2026-09-14: every stage,
SVG and GDS of `layout`, `capacity`, `global`, `assign`, `corridor`,
`detail`, `final`, the five final phases as SVG, and `drc.json`.
`--stage aligned` cannot be drawn yet ("arrives with phase 4").

## The stage as it is now

One driver, `Driver` in `src/pipeline/FinalRouter.cpp`, runs the inner pass
and the outer pass with the same loop. Read it in this order:

| Piece | Where | What it is |
| --- | --- | --- |
| `Tuning`, `tuningOf` | `:83`, `:131` | every knob converted onto the grid once. `spacing` is the rule in cells, unrounded; `approach*` the port band's geometry |
| `Scene`, `sceneOf` | `:196`, `:249` | the mask with the keepout, the port bands, the target of every port |
| `Field` | `:430` | copper and clearance per cell. **Only the count reads it**; no search is fenced by it |
| `Wire`, `Pass` | `:559`, `:599` | one connection; one pass's parameters |
| `Driver::sweep` | `:914` | the rounds; every pass starts with every wire on its Detail way (`seed`, `:1177`) |
| `Driver::attempt` | `:1252` | **the prototype's two phases and nothing else**: phase 1 fenced by the two ring neighbours; phase 2 the relaxation along the sweep, the wire let go of is crossable, the fence is the last one let go of plus the opposite neighbour, the lane is free and everything outside it priced, the wires let go of are priced at growing distances (`priceLane`, `:1653`), their approaches ten times over (`priceApproaches`, `:1696`). A way found is taken; on failure the wires let go of go back |
| `Driver::fence`, `buildCorridor` | `:1537`, `:1469` | `mark_obstacles` and `expand_path` of the prototype |
| `Driver::meetAt`, `couldMeet` | `:1368`, `:1572` | the junction: two terminals within one clearance, everything within 1.5 of either exempt. **Geometry alone** |
| `Driver::conflictsOf`, `failsOf` | `:1352`, `:1032` | the count: `Fails = unrouted + open`, by the check's own test |
| `Driver::drawSearch`, `drawGrid` | `:2149`, `:2284` | the debug pictures |
| `src/pipeline/DebugSvg.hpp` | | the SVG painter |
| `src/grid/PortBands.cpp` | | `bandLength`, `bandHalfWidth`, `stampBand`, `digTargetBeyondBand` |
| `src/drc/Rules.cpp` | | rule 1 wire clearance, rule 3 wire loop. Rule 4 obstacle clearance is out |

Everything the search decides by is in the picture `-d` draws of it, and
`-v 1` says one line per search. Use both before reasoning about a wire.

```bash
.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml -o artifacts/17q -v 1 -d
```

## What changed on 2026-09-13 and 14, most valuable first

Every item was measured over 4q–33q at six rounds and five relaxations, and
the 69q run confirms the end state.

1. **The exemption for two wires ending on one component is gone** — from the
   check (`junctionsOf`), the fence (`couldMeet`, `meetAt`) and the count.
   It assumed a component's ports sit closer than the wire spacing; the two
   ports of a qubit sit 909 units apart on 17q, and wire 44 ran 140 units
   from wire 43's approach unreported and unfenced. Taking it out took 33q
   from 18 fails with 10 crossings to **0**, and every other chip to 0 as
   well. The forgiven pass-bys were what the relaxation cascades grew from.
   The prototype forgives such a pair outright (`wires_connected` in
   `verify_min_clearance`); do not bring that back.
2. **The target sits on the first cell the straight-length rule allows.**
   `digTargetBeyondBand` stepped once before its first test, so every target
   lay two cells beyond the band: 120 units along an axis and 127 along a
   diagonal against a rule of 100. Now the band ends before the first cell
   whose centre lies `min_straight_length` from the port's *own* position
   and the target is that cell: 100–110 axial, 100–114 diagonal. Alone it
   took 17q and 21q to 0.
3. **`Tuning::spacing` was never assigned**, so the stage counted no wire as
   too close and its "0 too close" could not agree with `drc`. Set now; the
   `Fails:` line and the check count the same encounters.
4. **Every pass starts on the Detail stage's ways.** The sweep is a rip-up
   and re-route, and a rip-up needs something to rip: seeds are joined on
   the router grid with the straight stub in front and put down with copper
   and clearance. A wire still on its seed at the end is *unrouted* and gets
   no cells in the artifact, as the prototype drops it.
5. **`attempt` is the prototype's `run_final_routing`, verbatim in shape.**
   The field-fenced search, the verdict against the field, the relaxation
   against the sweep, the targeted rip and the rescue are all gone. What the
   summary calls "the four corrections" no longer describes the search.
6. **Rule 4 (obstacle clearance) is out of the check** because its raster
   did not exempt the port approaches the router's mask exempts, so it
   reported every stub out of a port. The keepout is still searched.
7. **`Fails:` after every stage**, `-v 1`, `-d`, and the tests read
   `rounds`, `max_relaxation`, `refinement_rounds` from each benchmark's
   `config.toml` (`test/pipeline/Benchmarks.hpp`), so the suite runs what
   `plan` runs.

**Measured and not kept** — see the summary for the figures: a verdict by
penetration depth (more fails), a uniform top price on released rooms (more
fails), the approaches of the wires around priced ten times over (kept, but
it changed nothing: the crossings lay 26–510 cells from any terminal),
doubling that zone's width (one fail more on 33q). The approach price is
still in; it is harmless and the user asked for it, but nothing measured
earns it.

## Lessons that cost time

1. **A signed step times an unsigned count wraps.** `step.x * k` with
   `int8_t step.x = -1` and `uint32_t k` walked off the grid on its first
   step, and the fallback hid it for every port facing 180°, 270°, 135° or
   315°. Keep every such loop in `std::int64_t`. Measure the result of a
   geometric change from the artifact, per wire, before believing it.
2. **Read the log before the theory.** `-v 1` showed within a minute that
   the 33q crossings arose in *phase 1* of wire 47 — fenced only by 46 and
   48 — after 43, 44 and 45 had relaxed into 47's room. No price on the
   relaxation could reach that. The approach price was built on a plausible
   reading of the pictures and measured neutral.
3. **A shortcut in an exemption is a hole in the rule.** The component
   shortcut was in three places that all agreed with each other, so no test
   and no count could see it. Only the plot, which draws the clearance
   bands without exemptions, showed the overlap. When the check, the count
   and the search share a rule, a single wrong assumption is invisible.
4. **The prototype's parallel routing is not its sequential routing.**
   `run_final_routing_parralel` relaxes *both* ways and repeats each round
   up to four times within itself; `run_final_routing` does neither. When
   the user says "exactly the prototype", ask which one. Neither was needed
   in the end.
5. **The price model is the prototype's to the digit**: `100·Σ static +
   100·wire[end]·(1 + swept)` per primitive, bends in hundredths of a cell,
   norms resolved with `llround` and a floor of 1. If a price seems not to
   bite, it is not the scale — compare bend 7125 against a per-step price
   of 400.
6. **Diagonal bands are narrower than the rule.** `bandHalfWidth` truncates
   `19/√2/2` to 6, so a diagonal band is 13 cells = 183 units wide against
   185, and asymmetric by a cell on the joining strips. Inherited from the
   prototype; untouched; worth a look if a diagonal port ever fails the
   check.
7. **Tooling.** `clang-format` on this machine is 21.1.7, the hook pins
   23.1.0; the diff was nil so far. `uv sync` does not rebuild the
   extension — use `uv pip install --python .venv/bin/python
   --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .` after
   every C++ change, and check behaviour. `plan --stage X` reads the
   configuration from the run directory, not from `-c`. The pinned ruff
   reports four `noqa` comments in `cli.py`, an import blank line and a
   missing `__init__` docstring in `run.py`, and one old test signature; all
   predate this work.
8. **`-d` is heavy on a chip that relaxes a lot** (one picture per search,
   17q: 166 pictures, 19 MB), and at the schema's defaults of thirty rounds
   the seeded sweep on 45q once took 109 minutes. Never run the Final tests
   at 30/10 on 45q and up; the tests read the shipped 6/5/0 now.

## What is open

1. **The meander** — below.
2. **Couplers, feedlines, refinement**: [handover-final-couplers.md](handover-final-couplers.md).
3. **Rule 4** needs the port approaches exempted in its raster before it can
   come back (`sceneOf`, `keepoutExemptions` is the model).
4. **The refinement** (`Driver::refine`) is ported but runs zero rounds
   everywhere and is unmeasured; the prototype runs 3 to 15 rounds with the
   two neighbours hard at the clearance and a centring price of 6.
5. **The diagonal band width** (lesson 6).
6. `CheckedWire::components` in the DRC view is filled and read by nothing.
7. The summary's "four corrections" and "measured" sections are a history
   now; the sections "The seeded sweep", "The search fenced as the
   prototype's" and the two bug findings are current. Keep it that way.

## Next: the meander

A resonator's wire has to be `meander_length` long before the coupler is
spliced into it — longer than `target_resonator_length` (2500), because the
coupler is spliced where the way left to the qubit reaches the target and the
wire is fed on the ring, not at a port. The router draws it as short as it
can, so a serpentine is inserted into a straight run of it.

**What the repo has.** `Wire::resonator` (`FinalRouter.cpp:582`, set from
`AssignedRole::ResonatorTarget`); `FinalParams.meander_length` in layout
units (`schemas/config.fbs:245`, default 3000, read into `Tuning::meanderLength`
as cells at `:161` and used by nothing); `DubinsRouter::freeStripAlong`
(`include/mqt-scpd/routing/DubinsRouter.hpp:212`), the widest obstacle-free
strip along a segment, which is what a serpentine is fitted into;
`routing::samplePath` in `PathGeometry.hpp`, which renders a path's length
the way the prototype's `sample_path` does; `CouplerInsertion.hpp` for the
phase after. Decision 0019: the router's `meander_insertion_params::
min_straight_length` is **not** the design rule of the same name — it is the
straight a meander needs to fit, and keeps its own name.

**What the prototype does** (`dubin_router_opt.hpp:2010` params, `:2182`
`meander_insertion`, `:2325` `meander_insertion_proximity`;
`FinalGrid.cpp:7017` and `:7201` in `process_wire` of `run_final_routing_parralel`):

- After a resonator's search succeeds, in phase 1 and in every relaxation
  level alike, it calls the insertion with
  `meander_length - target_anchor_offset(path)`, where the offset is the gap
  between the sampled path's last point and the port's true position. **A
  resonator whose meander cannot be placed counts as not routed**, and the
  relaxation goes on; the plain way is kept only as the neighbour obstacle.
- `meander_insertion`: sample the path; if it is already long enough, done.
  Otherwise walk the points of its straight segments in steps
  (`distance_from_s_t` from either end), and for each pair of points try
  `compute_meander_between_points` with the extra length needed, inside the
  box `LowerX..UpperX × LowerY..UpperY` — the grid inset by
  `outer_meander_insertion_boundary = 0.07` on each side — with
  `min_clearence`, `min_straight_length = 25` and `min_radius = 5`. The first
  fit is spliced in, the path re-sampled, and the real length printed. The
  `_proximity` variant scores candidate placements by bends and the wire
  price instead of taking the first.
- Lengths are in final-grid cells per layout: 4q 250, 9q the default 300,
  17q 300, 21q 400, 33q to 69q 600. In our layout units that is about 2500,
  3000, 3000, 4000 and 6000. **Our benchmarks carry no `meander_length`, so
  every chip runs at 3000 today** — set it per chip as the prototype does
  before measuring anything.
- The meander lives inside the corridor and the fence of the search that
  produced the way, so it cannot cross a neighbour the way itself could not.
  In the relaxation the `_proximity` variant reads the same price field.

**Where it goes here.** In `Driver::attempt`, once `found` is non-empty and
`wire.resonator` holds, before the way is taken and placed: extend `found`
to the required length or treat the attempt as failed and let the relaxation
continue, exactly as the prototype. The count, the fence and the pictures
need nothing new — a meander is cells of the way like any other — but
`drawSearch` should say the length reached, and the `-v 1` line should say
"meander: N cells, L units" or "no room for the meander". Check the result
with `samplePath` against `meander_length`, with `resonator_length_tolerance`
(100) as the margin, and put that check into `test_final_router.cpp` over
every chip: every resonator's way at least `meander_length` long and every
wire still within the rule. The `Fails:` line should count a resonator that
is short as unrouted.

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
```

Last full run on 2026-09-14, after everything above: Python 141 of 141;
`ctest --preset release` 350 tests in 29 minutes, 347 passed. Of the three
that failed, none is of this step, and two are repaired in the tree:

- `ArtifactSchema.FinalRoutingKeepsTheCreatedPortsAndTheFailures` built a
  `FinalRoutingT` without the `grid` the schema has required since the Final
  stage was added, so the reader refused its own bytes. The test now sets
  one. Repaired.
- `BenchmarkStages.ReportsWhatThePortsOwnApproachesKeepClear` expected the
  capacity keepout to be strictly larger than the reserved cells, which held
  while a launcher swept squares in front of its port; the slot is the
  launcher now and nothing is swept, so on 9q the two sets are equal (740 and
  740). The test says `>=` now, with the reason. Repaired.
- `BenchmarkDetail.DrawsCopperTheRulesAllow/57q` — **open, not of this
  step.** On 57q the Detail stage's wires start one cell away from the point
  the corridor feeds them at: "wire 0 starts at (92, 1244) and is fed at
  (1713.19, 38222.16)", and the same for 11, 100, 102, 107, 112, 118, 121
  and more, all on the top and bottom rows of the ring. Nothing in
  `Scene.cpp`, `CapacityPlanner.cpp`, `CorridorRouter.cpp` or
  `DetailRouter.cpp` changed functionally since `fc7e846`, so it was red
  before this step. The other seven chips pass it. It smells like the feed
  point of a launcher on 57q not falling on a detail cell centre; start at
  `cellAt` in `DetailRouter.cpp` and the launcher slot in `Scene.cpp`.
