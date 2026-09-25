# Phase 4, step 4 → the coupler insertion, the feedlines and the repair

Written for whoever takes the Final stage further. All five phases are built,
drawn and checked; what is left is the routing quality on the chips from 17
qubits up. Read [summary-final-routing.md](summary-final-routing.md), section
*The couplers, the feedlines and the repair*, for what was built and how it
differs from the prototype;
[decision 0032](docs/design/decisions/0032-the-coupler-couples-along-the-ring.md)
for why the coupler is shaped as it is; and [user_final.md](user_final.md) for
how to run it, what every line of the log means and where every symbol sits.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. The outer routing and the meander are
  committed (`1957fe1`, `6faeaae`, `cd8e597`); the couplers, the feedlines,
  the repair and the refinement are **not**. The user commits per phase —
  leave your work in the tree and say what you verified.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only. `run_optimized_cpw_coupler_insertion` is `FinalGrid.cpp:16940`,
  the feedline routing `:10566`, `run_final_routing_feedline_choices_parallel`
  `:13863`, the strict meander `dubin_router_opt.hpp:2500`.

## Where it stands

`06-final.fb` carries all five phases, and each one is a snapshot that can be
drawn, rendered and checked on its own:

```text
inner  outer  couplers  feedlines  refined
  ●      ●       ●          ●         ●
```

`Fails` counts wires and `drc` counts findings, so a pair of wires too close
is two fails and one finding. **Two measurement series, and they are not
comparable**: read the settings above each.

**Series 1, the whole stage, and out of date** — `rounds = 6`,
`max_relaxation = 5`, `refinement_rounds = 0`, `repair_trials = 100`,
`feedline_refinement_rounds = 5`. Both of those last two are zero by default
now, by the user's decision. None of the coupler rules below is in these
figures; they are kept only to show what the repair and the refinement are
worth, which is the difference against series 2 on the same chips. **Measure
this series again before quoting it.**

| chip | **Fails** | `drc` | final stage |
| --- | ---: | ---: | ---: |
| 4q  |  **0** |  0 |    6 s |
| 9q  |  **0** |  0 |   42 s |
| 17q |  **7** |  5 |   99 s |
| 21q | **13** | 12 |  420 s |
| 33q | **49** | 55 | 1299 s |

**Series 2, the sweep alone, and current** — `repair_trials = 0`,
`stop_after = "feedlines"`, so neither the repair nor the refinement runs.
This is what the eight `benchmarks/*/config.toml` carry, because the user
asked for the trials to be switched off so that what the sweep itself leaves
is visible. It is the series to work against while the coupler rules are
being changed.

| chip | baseline | **fails** | N | edges | diagonal | `drc` findings | wires in them |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |   **0** |  **0** |  0 |   5/5 |  0 |   0 |  0 |
| 9q  |   **0** |  **0** |  0 | 10/10 |  1 |   0 |  0 |
| 17q |  **10** |  **6** |  0 | 20/20 |  3 |   5 |  6 |
| 21q |  **14** | **12** |  0 | 26/26 |  3 |  13 | 12 |
| 33q |  **54** | **36** |  0 | 40/40 |  8 |  33 | 36 |
| 45q |  **61** | **59** |  0 | 52/52 |  9 |  71 | 58 |
| 57q |  **77** | **79** |  6 | 66/66 | 14 | 117 | 77 |
| 69q | **112** | **96** | 20 | 81/81 | 12 | 132 | 93 |

`N` is the fallback count explained below. **Every chain edge is drawn on
every chip**, which was not true of the strict fence without the fallback.
Compare *wires* with *fails*, not findings: one wire can sit in several
findings, so on the larger chips the findings outnumber the fails without
anything being wrong.

**The two counts agree, and where they differ the difference is named.** On
17q, 21q and 33q they match wire for wire. On 45q, 57q and 69q the stage is 1,
2 and 3 wires higher, and that is exactly the resonators counted `long` or
`short` — 45q has 2 long, 57q 1 short and 4 long, 69q 1 short and 6 long — for
which rule 7 does not exist on the Final view. Such a resonator is counted by
the check only where it also sits in a clearance pair. There is no unexplained
difference on any chip.

**57q is the one row that loses against its baseline**, 77 to 79. That is what
the clearance between two edges costs on that chip, and it is worth paying:
two fails against a rule that holds physically.

The gain is unevenly spread: 33q loses sixteen fails, 69q sixteen, 45q two,
57q none. On every one of the eight chips every coupler is placed and every
chain edge is drawn. The four named causes, measured on 17q as each went in:
10 at the baseline, 10 with the body rules, 8 with the terminal wall gone, 6
with the clearance between two edges.

**The user's three rules, measured on the finished artifacts** rather than
read out of the router (`check_rules.py` in the session scratchpad reads the
artifact alone, so it agrees with neither the router nor the checker by
construction), each outside the reach of a coupler the two share:

| Rule | Result |
| --- | --- |
| No coupler body covers a foreign resonator | holds on all eight |
| No edge within the clearance of a resonator | holds on all eight |
| No edge within the clearance of another edge | one case, 57q edges 20 and 21 at 100 units against 185 |

**Measure the exemption from the anchor, not from the body.** A coupler's
reach is a disc around the port it creates, which is the anchor the resonator
starts on; the body centre sits about half a coupling run away. An independent
check written against the body centre put that disc off the meeting and
reported 147.7 units for the 57q pair, where the check reports 100.0 — it hid
the real closest approach behind its own exemption and found a farther one.
The stage's `couplerReach()` and `drc::sharedCoupler` both measure from the
anchor; anything that checks them must too.

No two edges overlap anywhere away from a shared coupler, so the rule holds as
the *crossing* rule the user asked for. The one case is a *clearance* kept too
narrowly: 57q edges 20 and 21, consecutive in chain 2 and sharing coupler port
665, run 100 units apart just past that coupler's reach. It is one of the
places the fallback below gave up, which is the price that section names. `mqt-scpd drc` reports the same pair with the same figure,
so nothing is hidden — it is one of 33q's 34 findings. It is the only
edge-against-edge finding on any of the eight.

**Fixed with a fallback, and read why before you touch the fence.** The
obvious fix works on the small chips and breaks the large ones. That second
half was found only after the first had been written down as a success, on
five chips out of eight.

| chip | before the fence | strict fence | strict + fallback |
| --- | ---: | ---: | ---: |
| 33q | 38 | **36**, case gone | **36**, case gone |
| 45q | 59 | 59 | — |
| 57q | 77 | **79** | — |
| 69q | 96 | **98**, only **79 of 81 edges drawn** | — |

The two undrawn edges were the real damage, not the two fails: a chain with a
missing edge leaves its coupler attached to nothing, which is worse than two
edges that run too close. The fallback routes the second edge against the full
clearance first and against the old two-cell copper alone when that finds
nothing, which is the pattern the commit already used. On 4q to 33q it gives
the strict figures with every edge drawn; 45q, 57q and 69q were still running.

**A fallback makes the rule depend on the data**, and that does not go away in
principle: whether two edges keep their distance would depend on whether the
strict route happened to find a way. The insertion counts it, as
`N weighed without the clearance to their neighbour` on its own line, so a
difference between two runs can be told apart from a difference the fallback
caused. **Quote that number whenever you quote a fail count.**

| chip | N | edges drawn |
| --- | ---: | --- |
| 4q … 45q | 0 | all |
| 57q | 6 | 66 of 66 |
| 69q | 20 | 81 of 81 |

**The honest reading: the rule holds everywhere, but it is not enforceable
everywhere, and where it is not, the price is paid in clearance rather than in
chains.** N says how often, per chip.

Up to 45q the fallback never fires, so those rows are directly comparable with
the ones measured before it existed, and 33q giving the same 36 either way is
explained rather than merely observed — the same code path ran. From 57q up it
fires, and on 69q those twenty fallbacks are exactly what keeps the last two
edges drawn: without them the chip ended on 79 of 81. So a low N is not a
better result in itself. On the small chips it means comparability is free; on
69q, `N = 0` would have meant two incomplete chains.

**A fallback is a clearance given up, so check for it.** Every fallback is a
place where two edges were weighed against copper alone, which is where an
edge-against-edge finding can survive. On a chip with `N > 0`, measure rule C
on the artifact rather than assuming it.

**If a chip comes back with edges undrawn, N tells you which bug it is.** With
`N > 0` the fallback fired and was not enough, so the geometry was already
closed before the greedy ran. With `N = 0` the fallback did not fire although
it should have, and the place to look is the *first* edge: `localCost` also
returns infinity when the first edge finds no way, and that one has no
fallback at all — it is routed against the committed edges, not against a
neighbour.

The diagnosis is worth more than the case. The clearance between two edges is enforced in three places and
was right in two of them. `fenceCommittedEdges` fences a committed edge for
the edges routed after it, and `fence()`/`meetAt()` do the same in phase 4 —
but the commit does not always route an edge again. It takes the path the
greedy already chose (`chainEdgePaths_`), and those were produced inside
`localCost` with `ignoreAdjacent = true`, where the neighbouring edge keeps
only its copper dilated by two cells. An edge judged against its neighbour at
two cells' distance was therefore shipped at two cells' distance. The rule now
carries the full clearance stencil there too, exempt within `couplerReach` of
the anchor, which is the same rule as in the other two places. **A rule that
holds on the path that routes is not a rule, if another path can commit.**
Three places carried this one rule and two of them were right, which is trap 1
in a third shape: not the router against the checker, but two paths inside one
stage.

And the second lesson, which cost the first one its happy ending:
**enforcing a rule inside the greedy is not the same as enforcing it on the
result.** `localCost` returns infinity when an edge finds no way, so a
tightening there does not narrow a path, it deletes an *option*. The greedy
then takes a worse one, and by the commit the geometry is set hard enough that
even the fallback route finds nothing. Every tightening in `localCost`
therefore needs a fallback: try it strict, and fall back to the looser rule
rather than lose an edge. The commit in `insertCouplers` already works that
way.

**The stage's `Fails:` line and `mqt-scpd drc` agree on every chip measured** —
on 17q not only in count but wire for wire. Keep it that way: it is the one
property that makes a number worth reading.

Rule 7, `resonator-length`, is the exception and deliberately so. It is named
in the schema and implemented nowhere; `docs/design/pipeline.md` gives it the
Finalize view, because a resonator's true length exists only after the fit.
So the stage counts a resonator too short or too long as a fail and the check
says nothing about it. If that is ever to change, `FinalWire.length` carries
the figure the stage measured and the view can work out the run to the port
itself — but it would be checking an approximation, and the decision belongs
to whoever owns the Finalize stage.

The two counters inside the stage were checked against each other and agree:
`requiredLength()` is the target less `anchorGap`, and `sayResonatorLengths`
measures the way plus `anchorGap` against the same target, so a `short` fail
and a resonator named in the `N off 2500` line are the same wire. On 17q that
is wire 30, at 1908 units against 2500 — not a borderline case but a wire
whose search finds only a short way that the meander cannot lengthen.

## What the coupler insertion is

Phase 3 cuts every resonator back to a CPW coupler and draws the feedline
chains through the couplers.

- **The cut is at `target_resonator_length`** (2500 units on every chip), less
  the run from the way's last cell to the port, within
  `resonator_length_tolerance` (100). `meander_length` stays what it was: how
  long phase 2 makes the way so that the cut point exists at all.
- **The coupling run comes before the turn.** From the anchor the resonator
  runs `coupler_length` (20 cells) straight on the heading across the
  coupler's orientation, quarter-turns onto the orientation, runs the straight
  start and joins what is left of its way. The body spans the coupling run,
  `coupler_height` (3 cells) across on the side away from the turn, so the
  turn never crosses the feedline; the feedline runs along the body's far
  edge, along the ring, where the chains run.
- **The chains come from the assignment** (`Assignment.chains`), read off the
  model's chord, launcher and termination variables. A chain runs in ring
  order from the launcher slot of its first anchor to the slot of its last.
- **Options**: 8 orientations × mirrored × feedline direction × {no second
  dogleg, one of 20 reversed} = 64 per resonator, pre-screened to the 24 that
  turn least. An option is refused when its body, its head or the stub the
  feedline leaves it on lies off the grid, on artwork, on a body already
  placed or in a port's approach; when what is left of the way is longer than
  the target and tolerance allow; when the spliced way meets itself; when the
  body or the head covers a **resonator**; or when the option's own way comes
  back into the body after its turn. Another wire's *room* under it is
  **priced**, not refused, because every wire is drawn again in phase 4 — but
  a resonator's copper is refused outright, by the user's decision.
- **What a body is tested against is the tail, not the whole way.** A
  resonator's way before the cut runs all the way to the ring, and its head is
  exactly what gives way to this coupler; testing the body against that would
  refuse every option. `tailOwner_` marks, per cell, the resonator whose tail
  holds it — from the qubit backwards to `target_resonator_length` plus the
  tolerance, which is the part that survives the cut. Both rules together cost
  options (on 17q the worst resonator went from 18 to 13) and cost no coupler:
  every one is still placed.
- **A coupler may stand on a diagonal.** It never did before, on any chip, and
  the reason was a unit error: `coupler_length` and `coupler_height` were
  turned into cells with `cellsFor()` along an axis, and a diagonal step is
  √2 longer, so a diagonal body came out 41 % too long and twice too wide and
  fell through the `free()` test. `CouplerOption` now carries `run`/`depth` in
  cells of its own orientation (`cellsOn()`, diagonal = `lround(axial / √2)`,
  at least 1), and `couplersOf()` writes `length`/`height` into the artifact
  with `hypot(v.dx · cellWidth, v.dy · cellHeight)` — otherwise the check
  measures a body that is too short and its reach with it. Result: 3 of 17 on
  17q, 14 of 57 on 57q.
- **The greedy**: up to five passes per chain, every coupler takes the option
  whose two edges to its chain neighbours cost least. The cost is the
  prototype's: `10000 × turns + lengths + |Δlength| + 20000 × second dogleg +
  100 × guarded cells`.
- **Edges** are routed free of the crossing rule in a band of 200 cells around
  the chord, with the coupler bodies, the endpoint resonators' cut ways, every
  other resonator inflated by the clearance, the approaches of every port and
  every already-committed edge as obstacles. Two edges that meet at one
  coupler keep no clearance from each other but never cross.

Phase 4 draws every wire again under the feedline constraints and then repairs;
phase 5 refines. Both are the same `Driver` with a different `Pass`.

## Where the pieces are

`Driver` in `src/pipeline/FinalRouter.cpp` is the one rip-up-and-reroute loop.
The prototype writes it out four times, once per phase; `Pass` is the parameter
set that makes the four differ. Line numbers are of the declaration and drift
with every edit — search the name, not the number.

| Piece | Where |
| --- | --- |
| `Driver::insertCouplers`, `tailOwner_` | `FinalRouter.cpp:1733`, section *The couplers* — phase 3 end to end, and the map of which resonator's tail holds a cell |
| `cellsOn`, `Driver::couplerRunOf` | `:168` and `:1694` — a cell count on the coupler's own orientation; a diagonal step is √2 longer |
| `Driver::optionsOf`, `makeOption` | `:2044` and `:2068` — the 64 options, the geometry of one, and every test that refuses it |
| `Driver::edgeWayOf`, `fenceCommittedEdges` | `:2311` and `:2330` — a committed edge is an obstacle for the edges after it, two edges of one coupler exempt only within its reach |
| `Driver::fenceResonators`, `corridorOfEdge`, `routeEdge` | `:2400` and `:2522` — the search of one edge |
| `Driver::angleCostOf`, `localCost`, `heuristicCost` | `:2622` — what an option costs, and the beeline pre-screen |
| `Driver::optimizeChain`, `applyOption`, `sayCoupler` | `:2830` — the greedy, the commit, the log line |
| `Driver::ringWithEdges`, `assignBridges`, `beginPass`, `constrainByFeedlines` | `:2987` — phase 4's wire list and fences |
| `Driver::snapshot`, `restore`, `turnCoupler`, `unsettledBy`, `repair` | `:3139` — the repair |
| `Driver::refine` | `:1496` — phase 5, with the verdict |
| `Driver::failsOf`, `crossesAFeedline`, `sayFails` | `:1322` — the count and the `Fails:` line |
| `routing::CrossingConstraints` | `include/mqt-scpd/routing/CrossingConstraints.hpp` — the crossing rule, one implementation for the search and the check |
| `MeanderOptions.exact`, `tolerance`, `boxFor` | `include/mqt-scpd/routing/MeanderInsertion.hpp` — the strict insertion |
| `CouplerDoglegOptions.leadStraight` | `include/mqt-scpd/routing/CouplerInsertion.hpp` — the coupling run before the turn |
| `Assigner::readBackChains` | `src/pipeline/Assigner.cpp:420` |
| `drc::viewOfFinal`, `checkOrthogonality` | `src/drc/Rules.cpp:275` — rule 2 |
| `FinalParams.repair_trials`, `feedline_refinement_rounds`, `stop_after` | `schemas/config.fbs` |

## How to run it

```bash
cmake --build --preset release
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .

.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml -o artifacts/17q -v 1
.venv/bin/mqt-scpd drc  artifacts/17q
.venv/bin/mqt-scpd plot -c benchmarks/17q/config.toml --stage final \
    --phase couplers --run-dir artifacts/17q -o 17q-couplers.svg
```

`-v 1` names every failing wire and says, for a wire that found nothing, what
stands in its way. `-d` writes one SVG per search into `<run>/debug`, the
failing edge searches included.

**Iterate on one phase with `stop_after`.** It names the last phase to run,
takes the same five names `--phase` takes, and what follows it is not run at
all — so working on the insertion costs seconds, not minutes:

```toml
[stages.final]
stop_after = "couplers"
```

Tests, slowest last:

```bash
build/release/test/routing/mqt-scpd-routing-test
build/release/test/drc/mqt-scpd-drc-test
.venv/bin/python -m pytest test/python/unit -q
build/release/test/pipeline/mqt-scpd-pipeline-test \
    --gtest_filter='EveryChip/Final.*/4q:EveryChip/Final.*/9q:FinalRouter.*'
```

`Final.*` on 17q and up fails exactly as the fails above say, which is the
point of it.

## What to do next

1. **The dense fan-in from 17q up.** This is the whole remaining gap. With the
   cut at the target length the couplers sit deep among the qubit ports, and
   the edges between couplers run through the fan-in of the wires into those
   ports. Read the `in the way:` notes at `-v 1`: what blocks the failing
   wires is their two ring neighbours **at once**, a pocket the relaxation
   never opens because it fences one side while it lets the other go.
   Ideas not yet tried: draw a resonator from its coupler *before* the
   conventional wires around it; give the fan-in its own pass with a wider
   band before the ring is swept. **Read the next section before adding a
   third** — the obvious ones are spent.
2. **The overlaps on 21q come from a promise that is not taken back.** Four of
   its thirteen findings are true overlaps at zero units between neighbouring
   ring wires (7↔8, 28↔30, 63↔65, 65↔66). The mechanism: a wire relaxes, lets
   its neighbour go, finds a way through the neighbour's room and keeps it;
   the neighbour is searched again as `ripped`, finds nothing, and keeps the
   way it had. Letting a wire go is a promise that it moves, and nothing takes
   the promise back when it cannot. Rolling the ripper back is the obvious
   answer and is exactly what trap 2 below warns about, so it was not built —
   think it through before you do.
3. **The self-intersection rejection costs options.** Dropping an option whose
   spliced way meets itself took 21q from 11 fails to 13, because the coupler
   the repair used to turn to is no longer offered. The rejection is right —
   it is what the prototype does, and it removed a real rule-3 finding on 33q
   — but the greedy should be given the room back some other way, for instance
   by repairing the spliced way instead of dropping the option.
4. **Make the repair cheaper.** A trial is a snapshot, a turn and a one-round
   sweep over what the turn unsettles; the snapshot copies every wire, and a
   trial costs one to seven seconds. A restore that puts back only the
   unsettled wires, and trials in parallel as the prototype runs them, would
   let a hundred trials cost what ten cost now. The 17-qubit chip was still
   improving when its hundred were spent.
5. **Measure 45q, 57q and 69q to the end** with the current rules and fill in
   the table in the summary. Budget half an hour a chip and run them in the
   background.
6. **Look at the picture before believing a number.** `--phase couplers` draws
   the coupler bodies; a fail that looks like a routing problem is often a
   coupler that went somewhere silly.

## Measured and rejected

Five things that look obviously right and are not. Each was built, measured
and taken out again; the tree carries none of them. Do not spend the day
again.

| What was tried | Result |
| --- | --- |
| A relaxation level that lets go of **both** ring neighbours and fences nothing of the ring | 21q 14 → 16 fails, though it found a way for 34 searches that had none: the neighbours let go of found no room in turn |
| Coupler options priced by another wire's copper, weighted by kind (60 for conventional, 20 for a resonator, instead of a flat 20) | 17q 10 → 12 |
| Every cell in another wire's room under an option priced fourfold instead of once | 17q 10 → 10 |
| Every option charged for the cells of its two ring neighbours that fall within the rule of its head | 17q 6 → 10 |
| `rounds` 6 → 16 and `max_relaxation` 5 → 20 | 17q 6 → 10, and the pass stops itself after four rounds without progress anyway |
| Closing the cells behind an edge's junction in the corridor, so the next edge cannot run back over them | 4q went to "4 of 5 edges drawn": it takes from the search the room its own last move sweeps |
| Pricing those cells at 127 instead of closing them | Does not bind where the edge has no way out, and pushes 4q's feedline angle cost from 32 to 36 |

The three pricing experiments all say the same thing: the greedy's objective
is ten thousand per eighth turn of the two edge ways, and that figure carries
more than the room a better-placed body wins back. Disturb it and you lose
more at the edges than you gain at the neighbours. If the fan-in is to be
opened by pricing, the price has to leave the edge objective alone.

The refinement of phase 5 belongs here too in a weaker sense: without a
verdict on what it accepts it took 17q from 8 fails to 23 and 21q from 13 to
38. It is kept, with the verdict, and off by default.

## Traps, each of which cost a day

**1. The router and the check must measure the same thing, in both
directions.** Twice this went wrong. First the driver judged a committed wire
by the guard field (19 whole cells) while the rule is 185 layout units (18.5
cells), so it chased encounters the check forgave. Then the crossing rule went
the other way: the router read a feedline's straight runs off its *moves*, and
a move that runs straight before it bends counted as a bend along its whole
length, while the check read the *cells* — so the stage said `Fails: 0` where
`drc` found a wire at 45 degrees beside such a straight lead.
`CrossingConstraints::build` reads the cells on both sides now and takes no
primitives at all. In the same repair: **rule 2 binds on no edge of a chain**,
terminal or not, because the search draws every edge free of the crossing rule
and fences it by every other edge instead.

**1b. Two walls of our own making, and both cost fails.** They are worth
naming because neither showed up as a wrong number — they showed up as wires
that could not route, which looks like a hard chip.

The first was the edges at the launchers. They were hard obstacles for every
wire, which walls off the plane from the chip edge to the first coupler, and
the recorded reason cited no source. The prototype excludes them everywhere
instead, from the crossing rule and from the fence alike
(`FinalGrid.cpp:10677` and `:10788`, `is_first_last_feedline`). Both the
router and rule 1 treat every edge the same now: 17q went from 10 fails to 8.
**Read the prototype before writing down a reason.**

The second was the opposite, a wall that was missing. In
`fenceCommittedEdges`, two edges that share a coupler were separated only by
their copper dilated by two cells, over their whole length, so they could run
forty layout units apart right across the chip — which is what f0 and f1 did
on 17q. They now keep the full clearance, exempt within `couplerReach` of the
waypoint they share, which is the exemption `fence()`/`meetAt()` already made
in phase 4. 17q went from 8 to 6. The pair had never been reported, because
rule 1 exempted any two edges from each other; that is fixed too.

**2. Settled means the way it has is legal, not that this attempt found a new
one.** A wire whose search fails and whose way was legal all along is finished.
Returning false there kept eleven wires of the 17-qubit chip open for every
round, each costing up to twenty searches and moving nothing. But a wire a
neighbour *let go of* must be searched again even so — letting go was a promise
that it moves. That is what `Wire::ripped` is for.

**3. A wire list can grow under you.** `insertCouplers` appends the edges of
the chains to `wires`. A list of indices built before phase 3 and used for the
final count silently counted twelve wires of seventeen. Build it where you use
it (`everyWire()`), never once at the top.

**4. A search that ignores the rule is not a search that ignores the copper.**
The last resort routes the wire with the others' room ignored and their copper
still solid, then lets go of every wire whose room lies over that way. It is
what finds the wires actually in the way, which a sweep over ring neighbours
cannot: the blocker need not be a neighbour.

**5. The refinement needs a verdict.** The prototype's refinement takes what it
finds. Here five rounds without a verdict took 17q from 8 fails to 23 and 21q
from 13 to 38, because a wider way fenced only by the two neighbours and the
edges walks into the room of wires further along. A way found is taken only
when `conflictsIn(found) <= conflictsIn(had)` and it crosses no feedline.

**6. The rounds stop paying long before they run out.** On 21q the count
settles by the second round and the remaining twenty-eight change no byte. The
pass stops after four rounds without progress. Look for the same before turning
any round count up.

**7. Two encounters are not violations.** A junction (two terminals within one
wire spacing) and two wires ending on one component. A third is now added: a
resonator and the edges of its own coupler, within `couplerReach` of the anchor
— that is the coupling, and the fence, the count and the check all leave it
open. Getting any of these wrong in either direction costs a day.

**8. The build traps.**

- `uv sync` does **not** rebuild the extension when only C++ changed. Use
  `uv pip install --python .venv/bin/python --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .`
  and check behaviour, not the `.so` time. Three changes in a row once produced
  byte-identical output and only one was really a no-op.
- `cmake --build --preset release` does not pick up a new source file until the
  glob is re-run; touch the module's `CMakeLists.txt`.
- `mqt-scpd plan --stage <name>` reads the config **from the run directory**,
  not from the one named on the command line. Copy it over first.
- Run `uvx nox -s schemas` after any `.fbs` change, and check that both the C++
  header and the Python module came out with the new field.
- **Do not run `uvx nox -s lint` blindly.** It reformats every committed file
  to 80 columns, and its `typos` hook renames identifiers in code it touches
  (it once renamed `nd` to `and` in `DubinsRouter.cpp`).

## What is open

1. **The fan-in from 17q up** — item 1 above, and the whole *Measured and
   rejected* section, which is what is left of the obvious answers. This is
   the only thing between here and zero fails everywhere.
2. **The repair is slow** and was still improving when its trials ran out.
3. **The insertion is slow on the larger chips**, because every option costs
   two routed edges per greedy pass: 77 s on 21q, 159 s on 33q.
4. **One pipeline test fails on 4q, knowingly.**
   `EveryChip/Final.TheEdgesOfAChainMeetOnlyAtTheCoupler` finds two
   consecutive edges sharing two cells instead of one. The second lies two
   steps inside the forced run-in of the next edge, and the previous edge
   crosses it at step 442 of its 526, so it is not the junction but a
   crossing somewhere along the way. **It is the run-in, not the way.** That
   run-in is fixed the moment the coupler option is chosen, before edge 4 has
   a path at all, so no fence against a neighbour's *way* can see it — which
   is why the edge-clearance fix that cured the look-alike case on 33q leaves
   this one standing. Both ways of forcing it were measured and taken out
   again (see *Measured and rejected*), because a drawn edge is worth more
   than two tidy cells. Fix the geometry of the run-in, or relax the test,
   but do not close those cells.
5. **The tree is in a measuring state, and only the user can end it.** All
   eight `benchmarks/*/config.toml` carry `repair_trials = 0` and
   `stop_after = "feedlines"`. That is not an oversight: it is the user's
   instruction to switch the trials off, written into the files, so turning
   it back is his call and not a tidying job. While it stands,
   `test_the_final_routing_carries_a_snapshot_of_every_phase` fails because
   it expects all five snapshots, and the pipeline tests expect the old fail
   counts from 17q up.

   What to expect when it is reverted: all five phases run again with a
   hundred repair trials, and the fails drop below series 2, because the
   repair addresses exactly what the sweep leaves. Series 2 then has to keep
   its "without the repair" label and a third series goes beside it. Budget
   the time — one pass over all eight chips takes far longer than the runs
   behind series 2, 69q several hours on its own.
6. **Nothing is compared against the prototype's own output**, only against its
   formulation as read and the counts in its checked-in run logs.
