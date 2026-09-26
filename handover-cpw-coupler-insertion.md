# The CPW coupler insertion

Written for whoever takes the coupler insertion further. Two things changed in
this session and both are in *How the option is chosen* below: the option is
now settled either by the greedy as before **or exactly**, by a layered search
over the whole chain, and the router's heuristic gained the bend term it was
missing. Before this, read
[handover-final-couplers.md](handover-final-couplers.md) for the feedline
chains, the repair and the refinement, which this session did not touch, and
know that the geometry described here differs from
[0032](docs/design/decisions/0032-the-coupler-couples-along-the-ring.md).

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. HEAD is **`cf17fb3` ⚡️ Coupler insertion
  basic functionality**. Everything below is **uncommitted** on top of it. The
  user commits per phase — leave the work in the tree and say what you
  verified.

## Where it stands

Measured chip by chip, **one run at a time**, with `stop_after = "couplers"`
and `repair_trials = 0` in all eight benchmarks. **Phase 4 does not run at
these settings**, so a resonator that is short here has not yet had the pass
that redraws and meanders it.

The two figures the insertion is judged by are **the feedline edges it fails
to draw** and **the feedline angle cost**. Fails of the whole stage are not a
criterion for this phase (user, 2026-09-26).

| chip | chains | edges | **not drawn** | angle | insertion | couplers (diagonal) | option costs (from the memo) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q | 1 | 5 | 0 | 16 | 1.6 s | 4 (2) | 320 (198) |
| 9q | 2 | 10 | 0 | 22 | 9.6 s | 9 (5) | 700 (339) |
| 17q | 4 | 20 | 0 | 72 | 19.8 s | 17 (8) | 2111 (1070) |
| 21q | 5 | 26 | 0 | 48 | 39.5 s | 21 (16) | 1660 (800) |
| 33q | 7 | 40 | **2** | 72 | 91.9 s | 33 (21) | 2869 (1302) |
| 45q | 8 | 52 | 0 | 123 | 146.4 s | 45 (33) | 3795 (1888) |
| 57q | 9 | 66 | **6** | 128 | 262.5 s | 57 (38) | 5278 (2435) |
| 69q | 12 | 81 | **6** | 176 | 509.0 s | 69 (38) | 6315 (3064) |

**`refinement_rounds` belongs to this measurement.** It drives the outer
routing's refinement, which settles what way a resonator has *before* the
coupler cuts it back — and therefore whether a place fits at all. 57q and 69q
were once run at 0 while the other six were at 5, and it shows: 57q lost seven
edges at angle 148 instead of six at 128, 69q eight at 184 instead of six at
176. All eight benchmarks carry 5; check it before believing a number.

## How the option is chosen

`insertCouplers` (`src/pipeline/FinalRouter.cpp:1830`) runs five steps.

1. **The couplers.** One per resonator that is feasible and drawn.
   `optionsOf` (`:2177`) builds up to 48 options for it, each the first place
   along the way that fits — see *The option space*. A coupler with no option
   is dropped and counted.
2. **The chains**, from `assignment.chains`: the start launcher, the couplers
   in ring order, the end launcher. A chain of fewer than two waypoints is
   dropped.
3. **The search** over the options — the greedy, or the exact layered search.
   This is the whole of what follows.
4. **The commit** (`:2044`–`:2167`). `applyOption` puts every chosen option in
   place, one wire per chain edge is appended, and each edge takes the way the
   search found for it **only if that way still passes `edgeWayStillOpen`** —
   rebuilt against the corridor as it stands now — otherwise it is routed
   again. Each edge is `place`d as it is committed, so it fences the edges
   after it.
5. **The report**, `==>` line included.

### Both searches price an edge the same way

One chain edge costs `10000 × angleCostOf(way)` — how much it turns, in
eighths, its arrival at the target heading counted (`angleCostOf`, `:3383`) —
and everything if it finds no way. Nothing else is priced. The length,
length-difference and `100 × guarded` terms are still commented out at
`:3640`.

What an edge runs between is a pure function of the option at each of its two
ends: `sourceOf` gives the `out` port of the option at the start, `targetOf`
the `in` port of the option at the end, and the end stub is that option's
coupling run. Everything `corridorOfEdge` (`:3041`) closes is either the same
for the whole chip — the artwork, the launcher stubs, the static proximity —
or read off those same two options: the 250-cell box around the two ports, and
the two endpoint couplers' own resonators.

**With one exception: `fenceCommittedEdges` (`:2952`)**, which closes *every*
edge of *every* chain as it currently stands. That is the one thing that ties
an edge's price to the rest of the chip, and it is what both searches have to
work around.

### The greedy — the default

`optimizeChain` (`:3682`), the prototype's `optimize_chain`. Five passes
(`PASSES = 5`); each pass walks the chain front to back and gives every
coupler the cheapest of its options.

- Every open option is priced by `localCost` (`:3462`), which routes the
  coupler's two edges and adds their angle costs. The pair it routed is
  remembered in `edgeMemo_`, keyed on `(chain, edge, option at from, option at
  to)`.
- An option that loses an edge is **never** taken. Lost edges are counted
  apart from the cost, because an empty way turns nowhere and `angleCostOf`
  scores it zero — on one number, losing an edge would look cheapest of all.
- The test is `cost <= least`, so a tie moves the coupler to the
  highest-indexed tied option and spends another pass.
- **The jogs open per coupler**: a pass in which not one plain option brought
  both edges home sets `jogsUnlocked`, and the passes after it may use the
  second-dogleg options.
- A pass that changes nothing ends the loop.

It is a coordinate descent: a worsening at coupler *i* that would unlock a
larger gain at *i+1* is never taken, and the loop order decides every tie.

### The exact search — `SCPD_CHAIN_DP=1`

`optimizeChainsExact` (`:4208`). **A chain is a trellis**: one layer per
waypoint, one node per option of the coupler standing there, and the step
between two of them is one routed feedline edge. A launcher is a layer of one
node. The cheapest run of options along the chain is then a shortest path, and
it is solved exactly.

```text
   launcher      coupler 1        coupler 2        coupler 3      launcher
      ●      ──▶  o₁ … o₁₆   ──▶  o₁ … o₁₆   ──▶  o₁ … o₁₆   ──▶     ●
                    w₀(·)          w₁(·,·)         w₂(·,·)        w₃(·)
```

**Pricing every pair is what makes that expensive** — an inner edge of a chain
is 16 × 16 = 256 real A\* searches — so the search prices as few as it can.
`routing::solveTrellis` (`src/routing/ChainTrellis.cpp`) does this:

```text
loop:
  solve the trellis over the prices it holds   (one sweep of the layers)
  if every step of the winner is real          -> it is optimal, stop
  else price the first step of it that is not  (one routeEdge) and solve again
```

It is exact however weak the bound is: every step it did **not** price is
carrying a figure that cannot be too high, so nothing can undercut a winner
whose own steps are all real. A weak bound costs evaluations, never
correctness. On the chips it priced between **7 % and 52 %** of the pairs of a
chain — 72 of 1008 on one 9q chain, 410 of 784 on one 17q chain.

**The bound is `turnBound` (`:3428`)**: the fewest eighth turns any way from
one pose to the other can make, answered without routing anything. The walk of
headings runs from the source heading to the target heading; the way's
straight runs add up to the displacement between the poses, so at least one of
them must point within an eighth of the beeline, and the walk therefore passes
through one of the beeline's three nearest headings. The best of those three
detours is the bound. It is **not** the prototype's
`estimate_edge_angle_cost_heuristic`, which pins the walk to the beeline
itself and can read up to two eighths high — harmless where it only throws
candidates away before routing them, fatal here.

**The rounds.** A round freezes `fenceCommittedEdges` and solves each chain
against it exactly, taking each chain's answer into the fence before the next
chain is solved; the rounds repeat until no chain moves. Up to four
(`ROUNDS = 4`). It is therefore **optimal against a frozen fence, not optimal
outright** — the fence is the part no decomposition reaches. Say it that way.

- **Between rounds only what moved is forgotten.** `forgetEdgesNear`
  (`:4110`) drops the remembered edges whose box a way that changed can reach,
  and keeps the rest — an edge sees nothing outside its own 250-cell box, so a
  way that moved elsewhere cannot have changed its price. On 17q the last
  round re-searches **0** of chain 1's 260 pairs and 0 of chain 2's 178.
- **The round that is kept is judged by `stateFaults` (`:4163`) first and by
  cost second** — how many of a state's edges would not survive its own fence,
  which is the commit's own `edgeWayStillOpen` test asked of a whole state.
  Judging by cost alone does not work: round 0 is solved against a fence with
  nothing in it, so it is always the cheapest and always the least real.
- It stops at the first round with **no** faults, at the first round that
  changes nothing, or the first time it stands on a state it has stood on
  before.
- **The jogs open at the wall.** When no run of plain options joins the chain,
  `reachedFromStart` / `reachedFromEnd` name the first layer the chain does
  not reach, and only the two couplers at that step get their jogs. Opening
  them everywhere makes an inner edge 48 × 48 = 2304 pairs.
- A chain the trellis cannot join at all falls back to the greedy, and says so.

**What it buys, measured** (`SCPD_CHAIN_DP=1`, against the greedy on the same
build; 21q and up are unmeasured with it):

| | 4q | 9q | 17q |
| --- | ---: | ---: | ---: |
| greedy, angle | 16 | 22 | 72 |
| exact, angle | 16 | **20** | **70** |
| greedy, insertion | 1.6 s | 9.6 s | 19.8 s |
| exact, insertion | 2.5 s | **5.9 s** | 37.9 s |

Every edge is drawn either way. The angle cost is never worse and twice
better; the clock goes both ways, because 9q settles in two rounds while 17q
never settles and pays all four. **The prototype measured the same trade and
dropped it** — its own full Viterbi cost 244 s against 14 s for its greedy at
100 bends against 99 (`FinalGrid.cpp:18459`). What makes it worth having here
is not the 1–3 % on the angle cost: it is that a price per *option* is a node
weight the trellis takes for free, and that is the term the greedy cannot have
at all. See *What is open*.

### The router's heuristic

`DubinsRouter::setBendLowerBound`, **on by default**. The free search adds
`bendPenalty × cyclic distance from a state's heading to the target's` to its
heuristic — a way has to arrive on the target heading and every eighth turn it
still owes costs one bend penalty. The orthogonal search always had this; the
free one did not. It **halves the states the search expands**. See *Runtime*.

## What a coupler is

A pad, and a lead the resonator leaves it on. Four ports.

```text
        feedline in ●───────────────────────● feedline out     far edge
                    ┌─────────────────────┐
                    │         pad         │  run × depth
                    └─────────────────────┘
     resonator port ●───────────────────────● resonator port   near edge
              (2)   ╰─╮                        (1)
                      │ arc, a quarter turn
                      ╰──────────● insertion point, on the resonator's path
                         lead straight
```

- **Two feedline ports** at the ends of the far edge, both carrying the pad's
  axis. The chain **arrives at `in` and leaves at `out`**, running the pad's
  length in between — that run is the coupling. (`sourceOf` returns `out`,
  `targetOf` returns `in`.)
- **Two resonator ports** at the ends of the near edge, opposite them. The
  first faces along the pad's axis, the second against it. Which one a coupler
  uses is an option.
- **The lead**: from the resonator port a quarter turn onto the coupler's
  orientation, then `leadStraight()` cells straight. The turn direction is
  *derived* — the two ports face opposite ways, so one reaches the orientation
  with the clock and the other against it.

**The orientation of a coupler is the direction the straight after the arc
runs** — the perpendicular from the feedline line to the resonator line. It is
`option.couplerOrientation`; the pad's axis is `option.orientation`, a quarter
turn back.

**The tip of the lead lands on the resonator's path**, not the pad's centre.
The lead is built first, in a frame whose origin is the resonator port; the
port is the place less the whole lead, and the pad hangs off the port.

**The place is chosen per orientation.** Each offset walks the places in order
of length mismatch and stops at the first one *it* fits. No cap: capping at 24
destroyed 4q, where the two dozen places nearest the target all put the pad
through the box edge while places that fitted lay further along the way.

### The option space

**8 offsets × 2 resonator ports = 16 plain options**, and every coupler on 4q,
9q and 17q has all sixteen. The offset counts from **the heading the nearest
launcher faces**, so offset 0 means the same thing everywhere: the resonator
leaves pointing the way the launcher that drives it points.

**Plus a second dogleg, gated.** After the arc and its straight, one more
quarter turn and one more straight of `COUPLER_SECOND_STRAIGHT = 20` cells,
turned each way — the prototype's `second_straight` / `second_reverse`. It
slides the pad sideways off the way it sits on without changing which way the
coupler points. That makes 48 options in all.

They are **not open by default**. Both searches open them only for a coupler
that has shown it needs them, and they differ in how they decide — see *How
the option is chosen*. Opening them everywhere was measured and is worse; the
gate is the whole value.

**A second dogleg turns the lead a second time**, so the heading it finally
meets the path on is not the coupler's orientation. The invariant therefore
checks the **first** arc: the orientation is what the component points the
resonator in, settled by the first arc, and the second is a jog in front of it
that moves where the pad has to sit.

## One length, not two

`meander_length` is **gone** from every benchmark. What the outer routing makes
a resonator's way is `target_resonator_length` itself. It used to be a second
figure carried per chip (2500 to 6000) while every chip asked for a 2500-unit
resonator; what it actually decided was how much slack the way had before the
coupler cut it back.

Targets now: 4q 2500, 9q 2500, 17q 3000, 21q 4000, 6000 on the four largest.

**`couplerPlace` subtracts the lead.** The place is the point where the lead
*and* what is left together come to the figure:

```cpp
const double wanted = std::max(0.0, (target * COUPLER_BIAS) - leadLength());
```

Without it the way left was the target and the lead — a quarter turn and
fourteen cells, about 290 layout units — sat on top. On 21q every resonator
came out 2746–2805 against 2500, over by exactly the lead.

## A feedline may not cross its own resonator

In `corridorOfEdge` beside the feedline fence. For each of an edge's two
endpoints that is a coupler, that coupler's own resonator is closed — **copper
dilated by 2, and the pad's own cells left open**.

Not the clearance, and the geometry says why: `coupler_height = 26` units is a
pad **3 cells** deep, so the feedline port and the resonator lead sit **4 cells
apart** at the coupler. A 19-cell clearance disc would bury the port the edge
has to reach, and running side by side there is the coupling itself.

**It rescued the undrawn edge on 17q** — adding a constraint made more routable,
which reads backwards until you see why: an edge used to shortcut across its
own resonator, the greedy priced that as cheap and aimed the coupler at it, and
the neighbouring edge then had nowhere to go.

## Runtime

Three things were measured this session, in this order. Each number is one run
at a time on the same machine.

**1. Three searches that were the same search.** `corridorOfEdge` discarded
the cells it was handed (`(void)more`), so `routeAgainst`'s clearance fallback,
the second order of `tryOrder`, and the commit's `ignoreAdjacent` retry were
all identical re-searches. Deleting them changed the output **byte for byte —
the fail lists included** — and cost nothing:

| | 4q | 9q | 17q |
| --- | ---: | ---: | ---: |
| before | 3.8 s | 23.7 s | 46.9 s |
| after | 2.1 s | 14.0 s | 25.1 s |

**2. The bend term the free search was missing.** `setBendLowerBound`, on by
default. Angle cost, edges drawn and the whole fail list identical on four
chips; full pipeline runs on 4q and 9q identical too:

| | 4q | 9q | 17q | 21q |
| --- | ---: | ---: | ---: | ---: |
| before | 1.9 s | 13.3 s | 24.1 s | 53.2 s |
| after | 1.7 s | 9.8 s | 20.1 s | 39.5 s |

**3. Our router against the prototype's `dubin_router_opt`.** Two standalone
harnesses, one machine, the same hundred requests as
`test/routing/test_route_stress.cpp`. **In the heuristic mode the pipeline
uses, the two searches are bit-identical** — pops 8 295 612, stale 1 335 804,
expansions 6 959 808, pushes 18 475 516, both — and at that identical work we
are **9 % faster** (11 175 against 12 327 µs/route). Per expanded node we are
faster than they are. What the comparison produced was the bend term above and
nothing else; the details and the three things it ruled out are in
`memory/scpd-router-vs-prototype.md`.

**Where a search's time goes**, on the real chips: **83 % the expansion loop,
15–23 % the distance field**. The field is a backward Dijkstra over the
corridor per search and it earns that back — measured both ways, 17q 23.4 s
with the field against 41.3 s with octile, 9q 13.0 against 14.9. On an *empty*
grid the ranking reverses and the field is pure overhead, which is why a
synthetic router benchmark says the opposite of the truth here.

**A box around the two ends of an edge**, 250 cells of margin, with everything
outside closed. Not a band along a beeline — that is what bent the edges into
shapes the bend penalty could not move, and it is not coming back.

## Reading the log

The output follows the prototype's: a `[Coupler Insertion]` tag, angles in
degrees, and the SVG of each search on the line that made it. `try` lines are
`-v 2`; the rest is `-v 1`. The `==>` line is the one to sweep a setting
against.

The greedy:

```text
[Coupler Insertion]   '4' chain 0 pass 0: holds option 1/16 -> cost 60000
[Coupler Insertion]   try '4' option 2/16: angle 90° (offset 0, resonator port 2)
                      at (612,365) -> cost 20000 · artifacts/4q/debug/final-00076-….svg
[Coupler Insertion]   '10' chain 0 pass 0: no plain option keeps both edges —
                      opening the second-dogleg options
[Coupler Insertion] ACCEPT '4' chain 0 pass 0: angle 90° … -> cost 20000
```

The exact search:

```text
[Coupler Insertion] chain 2 round 0: optimum 180000 over 6 layers,
                    312 of 800 pairs priced, 312 of them searched
[Coupler Insertion] round 0: total 660000, 5 edges would not survive this state
[Coupler Insertion] round 1: every edge survives this state — stopping
[Coupler Insertion] the chains stand on round 1: total 700000, 0 edges it would not keep
```

**`pairs priced` counts steps the trellis asked for; `searched` counts the ones
that were not already remembered.** The gap between them is what
`forgetEdgesNear` saved.

Both end on:

```text
[Coupler Insertion] edge f3 chain 0 (2->3): 280 cells, angle 4, as the greedy chose it
==> coupler insertion: 0 feedline edges NOT drawn, feedline angle cost 16, 1.7s
```

## The pictures

- **`final-coupler-options.svg`** — once, before the search: every option of
  every coupler, the one it starts on in pink.
- **Every edge picture** carries the couplers and the feedlines as they stand.
  A **dashed** feedline is a path the search chose for an edge that has no way
  — it will never be built. Drawn solid, as it was at first, it puts two
  feedlines crossing in a picture whose own edge crosses nothing.

## How to run it

```bash
cmake --build --preset release
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .
cp benchmarks/17q/config.toml artifacts/17q/config.toml     # ← not optional
.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml -o artifacts/17q --stage final -v 1
SCPD_CHAIN_DP=1 .venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml \
    -o artifacts/17q-dp --stage final -v 1
```

**`plan --stage final` reads the config *and the earlier stages' artifacts*
from the run directory.** This cost an hour and a wrong diagnosis said out
loud: three chips were measured with `stop_after = "couplers"` still in
`artifacts/<chip>/config.toml` while `benchmarks/` said `"feedlines"`. Copy the
config over, every time; and a fresh run directory needs `00-chip.json` and
`0[1-5]-*.fb` copied in or the stage refuses to start.

**Measure sequentially.** Running the chips in parallel contends for CPU and
the runtimes in the `==>` line become meaningless.

`uvx nox -s lint` reformats every committed file — do not run it blindly.

## Where the pieces are

| Piece | What it does |
| --- | --- |
| `Driver::insertCouplers` | the five steps: couplers, chains, search, commit, report |
| `Driver::optionsOf` | the 48 candidates, each walking the places for its own fit |
| `Driver::makeOption` | one option's geometry, and every reason to refuse it |
| `Driver::couplerPlace` | the places, by length mismatch, lead subtracted |
| `Coupler::jogsUnlocked` | the gate on the second-dogleg options |
| `Driver::nearestLauncherHeading` | what the offset counts from |
| `Driver::optimizeChain` | **the greedy**: five passes, every open option routed |
| `Driver::localCost` | the greedy's two edges, memoised |
| `Driver::optimizeChainsExact` | **the exact search**: the rounds, the frozen fence, the round it keeps |
| `Driver::solveChain` | one chain, one round: the trellis built and solved |
| `Driver::edgeCost` | one edge priced for a named pair of options, through the memo |
| `Driver::turnBound` | the admissible bound the trellis leans on |
| `Driver::stateFaults` | how many edges of a state would not survive its own fence |
| `Driver::forgetEdgesNear`, `Driver::edgeBox` | what a round has to forget, and what it may keep |
| `routing::solveTrellis` | the search itself, router-free and unit-tested |
| `Driver::corridorOfEdge` | the box, the launcher stubs, the feedline fence, the own-resonator rule |
| `Driver::routeEdge` | one edge, searched |
| `Driver::drawCouplerOptions` | `final-coupler-options.svg` |
| `DubinsRouter::setBendLowerBound` | the bend term in the free search's heuristic |

## What is open

1. **The exact search does not converge on 17q.** It cycles between a state
   costing 660000 with three edges that would not survive it and one costing
   700000 with one. So on 17q the real choice is **angle 66 with two feedline
   edges lost, or angle 70 with all twenty**, and the gate picks 70. Nothing
   yet closes that gap; the cycle is inside chain 0, its own six edges fencing
   each other from round to round, and Gauss-Seidel against Jacobi makes no
   difference to it at all.
2. **The chains of a round are independent and are not run in parallel.**
   Freezing the fence is what made them independent — this is the largest
   runtime lever left, 4× on 17q and 12× on 69q. It needs one router context
   per thread: `router_`, `corridor_`, `box_`, `proximity_`, `stencils_` and
   `frame_` are all shared `Driver` state. The prototype has exactly this as
   `CouplerRouterCtx`.
3. **The bend term is verified identical only to 21q.** 33q and up were run
   with it on and never against it off. Before trusting it there, run one chip
   both ways and compare the angle cost, not only the clock — the prototype
   leaves the same term off by default and says why
   (`dubin_router_opt.hpp:263-286`).
4. **The cost still sees only the feedline.** Length, length difference and
   `100 × guarded` are commented out at `:3640`, so the two resonator ports of
   one coupler are an exact tie. The trellis makes the fix cheap in a way the
   greedy never could: a price per *option* is a node weight, and a node weight
   costs no routing at all — `TrellisProblem::nodeCost` is already there and
   unused. This is the obvious next experiment.
5. **The commit order decides who gets the room.** Edges are committed by
   chain index and the first one there takes the space. Committing by how
   little room an edge has left is the natural fix.
6. **Two fences are dead code.** `fenceResonators` has **no caller** — the
   own-resonator rule was written inline instead, and the two should be
   reconciled. `closeOutsideBox` has none either, switched off on request; the
   comment at its call site says how to switch it on.
7. **The edge picture draws stale prices.** Edge searches attach
   `zeroProximity_`, but `drawSearch` still renders `proximity_`, which holds
   whatever the last sweep search left.
8. **Diagnostics are still in the tree** — the `fence for edge …` and
   `picture …` lines and their counters.
9. **The configs are in a measuring state** and must go back before a commit:
   `repair_trials = 0` and `stop_after = "couplers"` in all eight.
   `test_the_final_routing_carries_a_snapshot_of_every_phase` fails while they
   are in.

## Traps

**1. An empty way turns nowhere.** `angleCostOf` returns 0 for an empty path,
so an option that lost an edge scores *cheaper* than every option that kept
one. Both searches keep the lost count apart from the cost.

**2. A rule that only one path obeys is not a rule.** The commit reuses the
search's path when the endpoints match; for a long time it checked nothing
else, so a way drawn against a chip with no feedlines shipped through the ones
added since. `edgeWayStillOpen` now tests every cell of a kept path against the
corridor that holds at the commit.

**3. A heuristic and a rule that disagree cost you the option.** The feedline's
side of the pad was guessed from the resonator's run direction and the legality
rule then demanded one particular side: 240 of 416 candidates on 17q were built
and thrown away for no geometric reason.

**4. A picture that shows nothing is not proof that nothing is there — and a
picture that shows something is not proof that it is.** The feedline fence was
measured closing 211 464 cells while the picture showed none of it. Measure,
then believe.

**5. Adding a constraint can make more things routable.** Forbidding a feedline
from crossing its own resonator drew the edge that had been undrawn all
session.

**6. Bytes are not time.** Twice now. A per-search `memset` was blamed for the
runtime — 161 GB on 69q by volume, 0.4 % by clock. And the prototype's own
measurement of its bucket queue (93 % of pushes landing in the coarse block,
~48 GB of avoidable traffic) reproduces here at 86–87 % — and raising the fine
block until the coarse share falls to 20 % **changes the runtime not at all**.

**7. A synthetic benchmark can say the exact opposite of the truth.** On an
empty grid the octile heuristic is 18× faster than the distance field, because
the field build is nearly the whole cost. On the real chips the field wins by
1.8×. Measure the router on the workload the pipeline gives it, or do not
measure it.

**8. Rounds of a fixpoint iteration are not comparable by cost.** Round 0 is
solved against a fence with nothing in it, so it is always the cheapest and
always the least real; keeping it cost 9q a feedline edge and 17q two. Compare
them by whether a state survives its own fence, which is the test the commit
applies.

**9. A parameter that is silently ignored is worse than no parameter.**
`corridorOfEdge` took `more`, `ignoreAdjacent` and `pivot` and `(void)`-cast
all three. Three call sites were built on the belief that they did something;
all three were identical re-searches, and one of them made a counter that could
never increment print 0 into every log for weeks.

## Measured and rejected

Do not re-attempt these without new evidence.

- **The second dogleg open to every coupler.** 9q went 0 → 8 fails (all too
  long) and the insertion from 18 s to 61 s. The gate is the whole value.
- **The exact search without the chain's own fence** (`SCPD_CHAIN_DP=2`). It
  converges in one round and finds a better angle cost — 16/19/66 — and
  **loses edges**: 9q 9 of 10, 17q 18 of 20. The self-fence is exactly what
  keeps the edges drawable.
- **Keeping the cheapest round of the exact search** rather than the one that
  survives itself. 9q lost a feedline edge, 17q two. See trap 8.
- **Jacobi rounds** (every chain of a round solved against one frozen fence,
  all written at the end) against Gauss-Seidel: byte-identical results. The
  oscillation is inside a chain, not between chains. Jacobi is what would make
  the chains parallel, so it is worth having back when that is built.
- **Raising the bucket queue's fine block** from 1024 to 4096, 16384 or 65536.
  The coarse share falls from 86 % to 20 %; the runtime does not move.
- **The octile heuristic in the pipeline.** 17q 41.3 s against 23.4 s.
- **The prototype's micro-optimizations.** 8-byte nodes, 16-byte queue
  entries, the swept-cell prefix trie, the visit stamp, the bitmask-and-ctz
  queue — we already have all of them, and per expanded node we are faster.
- **Ranking options by lost edges instead of discarding them.** 17q 20 fails
  against 21 for discarding, but discarding is the safer rule and what the
  user asked for.
- **The box as a hard bound on every wire.** 4q went to 13 fails with nothing
  drawn. Ordinary wires are fed from points on the launcher ring, which lies
  *outside* the box.
- **Reversing the resonator port headings**, and **reversing the search's start
  angle in the feedline pass**.
- **One Dubins A\* per chain through a maze of resonator walls.** Sixteen
  coupler options are sixteen walls, not sixteen doors in one wall: with every
  usable option standing, 1 of 10 chains kept a way through. **The option must
  be settled before the search** — which is what the trellis does, and why it
  is a different idea and not that one again. See
  `memory/scpd-phase-4-chain-maze.md`.
