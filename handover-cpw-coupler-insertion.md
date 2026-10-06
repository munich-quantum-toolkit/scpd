# The CPW coupler insertion

> **This document is one step behind the code.** The option is no longer
> settled by the layered trellis it describes: the default is now a
> best-first search over **prefixes** of coupler options, with no rounds, and
> the commit keeps the ways that search found. What changed and what it is
> worth is in
> [handover-chain-astar.md](handover-chain-astar.md); *Where it stands* and
> *Where the pieces are* below carry the current figures, and everything else
> here still describes the ground all of it stands on — the options, the
> geometry, the bound, the corridor and the traps.

Written for whoever takes the coupler insertion further. **The option is
settled exactly**, by a search over the whole chain (*How the option is
chosen*) — the trellis is behind `SCPD_CHAIN_ASTAR=0` and the greedy behind
`SCPD_CHAIN_DP=0` from there. What made that affordable is the bound: it is now the
*exact* least turning of a way with nothing in its path (*The analytic
bound*), which prices a fraction of the pairs the old one did and made the
layered search **faster than the greedy on seven of the eight chips**, 69q
excepted. The router's heuristic also gained the bend term it was missing.
Before this, read
[handover-final-couplers.md](handover-final-couplers.md) for the feedline
chains, the repair and the refinement, which this session did not touch, and
know that the geometry described here differs from
[0032](docs/design/decisions/0032-the-coupler-couples-along-the-ring.md).

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`. HEAD is **`6226b15` ⚡️ progress coupler
  insertion some fails remain at 45-69Q**. The analytic bound is
  **uncommitted** on top of it; everything else is in. The user commits per
  phase — leave the work in the tree and say what you verified.

## Where it stands

**2026-10-05**: the insertion refuses, in every edge search, a way that
leaves the wires beside it too little room — *The squeeze rule* below —
and closes the coupler-end runs of every settled chain's terminal edges
in its corridor (`guardSettledTerminalRuns`, handover-feedline-routing's
*The terminal edges' coupler runs*). On the day's defaults
(`artifacts/logs/squeeze-reject2`, the feedline pass run after it):

| chip | not drawn | angle | insertion | ways refused | `bad` after the pass |
| --- | ---: | ---: | ---: | ---: | ---: |
| 4q | 0 | 12 | 0.5 s | 0 | 0 |
| 9q | 0 | 20 | 0.6 s | 0 | 0 |
| 17q | 0 | 51 | 8.7 s | 81 | 4 |
| 21q | 0 | 50 | 3.3 s | 0 | 1 |
| 33q | 0 | 76 | 8.8 s | 0 | 0 |
| 45q | 0 | 109 | 29.0 s | 21 | 0 |
| 57q | 0 | 132 | 31.1 s | 63 | 2 |
| 69q | 0 | 180 | 87.5 s | 7 | 1 |

Every edge drawn on every chip, no edge squeezed at the end, and the
feedline pass behind it ends with no open wire anywhere. What follows is
the state the search was handed over on, kept for the measurements.

Measured chip by chip, **one run at a time**, with `stop_after = "couplers"`
and `repair_trials = 0` in all eight benchmarks. **Phase 4 does not run at
these settings**, so a resonator that is short here has not yet had the pass
that redraws and meanders it.

The two figures the insertion is judged by are **the feedline edges it fails
to draw** and **the feedline angle cost**. Fails of the whole stage are not a
criterion for this phase (user, 2026-09-26).

**The prefix search is the default.** The table below is the *trellis* sweep,
kept because it is one sweep of two searches over all eight chips and the
prefix search is measured against it. What the prefix search makes of the same
chips, one sweep on pure defaults with no environment variable set,
2026-09-30:

| chip | not drawn | angle | insertion | `stateFaults` |
| --- | ---: | ---: | ---: | ---: |
| 4q | **0** | 8 | 0.3 s | 0 |
| 9q | **0** | 20 | 0.6 s | 0 |
| 17q | **0** | 47 | 2.0 s | 0 |
| 21q | **0** | 48 | 3.1 s | 0 |
| 33q | **0** | 76 | 7.3 s | 0 |
| 45q | **0** | 105 | 23.6 s | 0 |
| 57q | **0** | 132 | 32.1 s | **0** |
| 69q | **0** | 166 | 70.9 s | **22** |

**No chip loses a feedline edge.** But **69q's zero is not the other seven's
zero**: twenty-two of its eighty-one edges lie across others and count as
drawn only because the commit no longer tests a kept way. For 69q read the 22.
The rest is in [handover-chain-astar.md](handover-chain-astar.md).

**Both halves of the table below come from one sweep** of the trellis and the
greedy, every chip run on its own, so its columns are comparable to each other
and not only to history.

| chip | chains | edges | not drawn | angle | insertion | | greedy: not drawn | angle | insertion |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| 4q | 1 | 5 | **0** | **8** | **0.2 s** | | 0 | 16 | 1.6 s |
| 9q | 2 | 10 | **0** | **20** | **0.5 s** | | 0 | 22 | 9.5 s |
| 17q | 4 | 20 | **0** | **51** | **11.9 s** | | 0 | 72 | 19.5 s |
| 21q | 5 | 26 | **0** | 48 | **2.4 s** | | 0 | 48 | 38.8 s |
| 33q | 7 | 40 | **0** | 76 | **9.3 s** | | **2** | 72 | 90.0 s |
| 45q | 8 | 52 | **0** | **105** | **32.5 s** | | 0 | 123 | 144.4 s |
| 57q | 9 | 66 | **5** | **118** | 2641.0 s | | 6 | 128 | 261.5 s |
| 69q | 12 | 81 | **10** | 173 | 366.0 s | | **6** | 176 | 508.9 s |

The couplers are unchanged by the search: 4/9/17/21/33/45/57/69 of them, with
2/5/8/16/21/33/38/38 on a diagonal.

**Six of the eight draw every edge**, where the greedy loses two on 33q, six
on 57q and six on 69q. And where both draw the same edges the angle cost is
better or level everywhere: 4q **8** against 16, 9q 20 against 22, 17q **51**
against 72, 21q level at 48, 45q **105** against 123.

**Compare the angle cost only where the same number of edges was drawn.** An
edge that was not drawn turns nowhere and `angleCostOf` scores it **zero**
(*Traps*, 1), so a search that draws fewer edges is flattered by this column —
which is why 33q's 76 against 72 is the better result and not the worse one.

**The two that are not there yet, said plainly:**

- **57q draws 61 of 66** against the greedy's 60, at angle 118 against 128 —
  but it costs **2641 s against 262 s**, ten times the greedy, and it is the
  one chip where the second order is a bad bargain.
- **69q is a regression.** 10 edges undrawn against the greedy's 6. It is also
  *fast* — 366 s against the greedy's 509 — and the two go together: chains
  the trellis cannot join are now left as they stand rather than handed to
  the greedy, so 69q gives up quickly and loses what it gave up on.

**Raising the order does not reach these two.** The second order sees exactly
one pair of edges — the two that share a coupler. 57q was measured both with
a width cap that handed its two widest chains back to the first order and
without it: **5 edges undrawn either way**, 122 against 118 on the angle, 810 s
against 2641 s. So the edges that are still lost there are blocked by edges
that are **not** their neighbours, and no amount of predecessor in the node
reaches them. See *What is open*.

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
3. **The search** over the options — the prefix search by default, the
   trellis at `SCPD_CHAIN_ASTAR=0`, the greedy at `SCPD_CHAIN_DP=0` from
   there. The trellis and the greedy are what follows; the prefix search is
   in [handover-chain-astar.md](handover-chain-astar.md).
4. **The commit** (`:2044`–`:2167`). `applyOption` puts every chosen option in
   place, one wire per chain edge is appended, and each edge takes the way the
   search found for it. **Whether that way is tested first is now a switch**:
   `SCPD_CHAIN_KEEP_WAYS` is on by default and takes it as it is, so the
   paragraph below describes what `=0` restores. The endpoints are checked
   either way. What the test is for is trap 2, and giving it up is what makes
   the undrawn count stop reporting the truth — the way it took it
   **only if that way still passes `edgeWayStillOpen`** —
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

### The greedy — `SCPD_CHAIN_DP=0`

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

### The layered trellis — `SCPD_CHAIN_ASTAR=0`

**This was the default and is now the way back.** What replaced it drops the
one assumption it rests on; see [handover-chain-astar.md](handover-chain-astar.md).
Everything in this section still holds for the trellis path.

`optimizeChainsExact` (`:4208`), **on by default since the analytic bound
made it cheaper than the greedy on seven of the eight chips**;
`SCPD_CHAIN_DP=0` is the way back. **A chain is a trellis**: one layer per
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

**The bound is chosen by `SCPD_CHAIN_BOUND` (`:3261`)**, and how sharp it is
*is* the runtime — it decides how much of the trellis has to be searched at
all.

`turnBound` (`:3428`), **`SCPD_CHAIN_BOUND=0`**, is the original: the walk of
headings runs from the source heading to the target heading; the way's
straight runs add up to the displacement between the poses, so at least one of
them must point within an eighth of the beeline, and the walk therefore passes
through one of the beeline's three nearest headings. The best of those three
detours is the bound. It is **not** the prototype's
`estimate_edge_angle_cost_heuristic`, which pins the walk to the beeline
itself and can read up to two eighths high — harmless where it only throws
candidates away before routing them, fatal here.

**`AnalyticDubins::minTurns` (`src/routing/AnalyticDubins.cpp`) is the
default**, `SCPD_CHAIN_BOUND=1`, and it is the *exact* least turning of a way
with nothing in its path — see *The analytic bound* below. It prices a
fraction of the pairs `turnBound` does and is 2.5 to 7.6 times faster over the
chips, at **identical optima**.

**`turnBound` is essentially distance-blind.** It looks at the displacement
only to pick the beeline heading and never asks whether a way that turns that
little can actually cover it. Measured over random pose pairs its mean is
~3.0 eighths at every separation from 5 to 400 cells; the analytic answer runs
4.5 to 6.2 and rises as the poses close up.

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
- **A chain the trellis cannot join at all is left as it stands**, and says
  so. It used to fall back to the greedy here; that is gone (user,
  2026-09-27). A round was then part exact and part coordinate descent, the
  greedy's choice was fenced into every chain solved after it, and the round
  total no longer priced what the search had chosen. Removing it is a gain on
  its own: 4q went from angle 16 to **8**, because the fallback had been
  dragging the state onto a greedy choice that turned twice as much.

**What it buys, measured** (against the greedy on the same
build), **all eight chips, one run at a time**. The exact column is the
analytic bound, which is the default; the bound changes the clock and nothing
else, so the angle cost and the edges drawn are the same under either.

The figures are in *Where it stands*, which is one sweep of both searches over
all eight chips.

**With the analytic bound the exact search is faster than the greedy on seven
of the eight chips**, and by an order of magnitude on 9q, 21q and 33q — which
reverses the trade the prototype measured and dropped (its own full Viterbi
cost 244 s against 14 s for its greedy at 100 bends against 99,
`FinalGrid.cpp:18459`). **69q is the exception**: 949 s against the greedy's
509, because it pays all four rounds and never settles.

**It is not uniformly better on quality either.** 33q is the clear win — all
40 edges where the greedy loses 2, at angle 76 against 72 (which is level per
drawn edge; see *Where it stands*). **57q and 69q are the losses**: 7 edges
undrawn against the greedy's 6 on both, though at angle 114 against 128 and
160 against 176, which is less turning per edge either way. In each case the exact search does not
converge and the round it keeps still carries faults. That is a property of
the rounds, not of the bound — both bounds produce the identical
1280000/1920000/1280000/1900000 cycle on 57q with identical faults 9/11/10/11,
and the identical 18/12/10/12 on 69q.

What is worth having beyond the clock is still that a price per *option* is a
node weight the trellis takes for free, and that is the term the greedy cannot
have at all. See *What is open*.

### The order of the search — `SCPD_CHAIN_ORDER`

**A node carries the option at its own waypoint *and* the one at the waypoint
before it.** That is the second order, and it is **the default**;
`SCPD_CHAIN_ORDER=1` is the way back to a node that carries only its own.

**What it fixes.** At first order a step is priced against a fence that holds
what an *earlier round* committed — inside one solve, that is the other
assignment's way. So the two edges that meet at a coupler never see each
other: the trellis prices a pair that cannot both be built as though it
could, `stateFaults` counts the difference, and the commit is where an edge
is lost. On 17q that was six edges. Carrying the predecessor in the node lets
the step fence the way that predecessor actually takes, and the price of a
step is once again a function of the node at each of its two ends — which is
the property the trellis's exactness rests on (`ChainTrellis.hpp`).

**The recursion is cut after one step, on purpose.** The predecessor's own way
depends on *its* predecessor, and following that back would make a node the
whole prefix of the chain — the exponential the trellis exists to avoid. So
the way fenced is the predecessor's **first-order** way, which is
well-defined, deterministic and already in the memo. It covers the pair that
matters, because two edges can only meet where their 250-cell boxes overlap
and at a shared coupler they always do.

**What it costs.** A layer of `n` options becomes one of `n x n` and the steps
between two layers `n^3`. At sixteen options that is 256 nodes and 4096 real
steps a layer, which solves; with the jogs open it is 2304 nodes, which does
not, so a chain past `SECOND_ORDER_NODES = 600` is solved first order and
says so in the log. The analytic bound is unaffected — it depends on the pair
of options and nothing else, so it is still asked `n x n` times a layer and
the lifted graph is filled from that.

**`edgeMemoKey` carries the predecessor** as a third option field, because one
pair of endpoint options now has a different way under each predecessor. Zero
is the first-order slot — what the greedy writes, and what the second order
reads its own fence out of.

### The router's heuristic

`DubinsRouter::setBendLowerBound`, **on by default**. The free search adds
`bendPenalty × cyclic distance from a state's heading to the target's` to its
heuristic — a way has to arrive on the target heading and every eighth turn it
still owes costs one bend penalty. The orthogonal search always had this; the
free one did not. It **halves the states the search expands**. See *Runtime*.

## The analytic bound

`AnalyticDubins` (`include/mqt-scpd/routing/AnalyticDubins.hpp`,
`src/routing/AnalyticDubins.cpp`) answers **the fewest eighth turns a way of
this bend radius can make from one pose to another when nothing is in its
way** — no obstacles, no corridor, no edge of the chip. Every real way is also
an obstacle-free way, so it can never exceed what the search produces: it is
admissible, **the trellis stays exact**, and it is simply far sharper.

**The problem is small because the move set is.** There are exactly five
primitives per heading — the straight step, the eighth turns to either side
and the quarter turns to either side (`test/routing/test_primitives.cpp:40`
asserts the turn set is `{-2,-1,0,1,2}`). A straight does not turn, so in this
metric it is free and may be repeated at will. What is left is

> minimise the turning of an arc sequence walking the headings from the
> source's to the target's, subject to the displacement the arcs leave over
> lying in the **cone of the headings the way runs straight on**.

Iterative deepening from `headingDistance(source, target)` up to a cap;
`cap + 1` when nothing fits, which is still a lower bound. The constructor
reads the arcs off `MovePrimitives::of`, so the model cannot drift from the
move set the router actually expands if the radius ever changes.

**The cone test is a 256-entry table, exact and O(1).** Every generator is a
multiple of 45°, so a cone's edges lie on headings and membership turns only
on which of **sixteen direction classes** the residual falls in — eight rays
and the eight open sectors between them. The table is built once by
Carathéodory in two dimensions: a member of a cone is already a non-negative
combination of *two* of its generators, so trying every pair and every single
generator settles it exactly. Do not replace this with an angular-hull
shortcut on "spans more than 180°": the cone of two opposite headings is the
line through them, not the half-plane, and that is exactly where such a
shortcut over-accepts.

**Two things that would have broken it silently:**

1. `reconstructSegments` (`src/routing/PathGeometry.cpp:27`) tags an arc with
   its **entry** heading and opens one segment per primitive change, so
   `angleCostOf` sums exactly `headingDistance(entry, exit)` per arc — a
   quarter turn counts two and nothing counts twice. The model has to match
   that, and it does.
2. The straight runs must be relaxed to **real** rather than whole cells, and
   the minimum separation the router keeps between two arcs dropped. Both only
   widen the feasible set, which is what keeps the answer a lower bound.

**Admissibility is measured, not argued, in three places** — the 100
stress-harness requests on an empty grid and through pillars
(`test/routing/test_analytic_dubins.cpp`), some 1500 random pose pairs routed
for real at separations from 15 to 300 cells, and the live audit below. **Not
one violation anywhere.** On random pairs it is *exact* on 81–95 % of them,
not merely below.

**It costs nothing.** 0.038 µs per answer against 11 231 µs per route in the
microbenchmark; 0.5–0.6 µs on real chip poses. The whole layer graph is built
up front and the line says what it cost — 13 632 bounds in 7.3 ms on 17q,
62 264 in 33.6 ms on 57q. The bound's own time is never the question.

**`SCPD_CHAIN_BOUND=2` is the audit.** It prints `turnBound / analytic / real`
for every step the search actually priced and a summary line, and counts any
step where the analytic answer sits above the real one. **Run it before
believing any clock** — it is the only thing that tests admissibility on the
real workload. On 9q: 32 priced steps, mean turnBound 1.75, analytic 2.44,
real 2.94; exact on 27, above the real on 0.

**The trap this set.** On the `test_route_stress` requests the analytic answer
is sharper than `turnBound` on **0 of 100** — those are wide open lattice
cells with opposite headings, where the beeline detour is always realisable.
That fixture says the change is worthless and it is wrong. Judge a bound on
the workload the pipeline gives it, or do not judge it.

**What it bought**, `SCPD_CHAIN_DP=1` with `SCPD_CHAIN_BOUND=0` against `1`,
one run at a time (pairs the search had to price / insertion):

| chip | turnBound | analytic | |
| --- | ---: | ---: | ---: |
| 4q | 264 / 2.5 s | 85 / 0.8 s | 3.1x |
| 9q | 421 / 6.0 s | 50 / 0.9 s | 6.7x |
| 17q | 3212 / 37.9 s | 1322 / 15.0 s | 2.5x |
| 21q | 479 / 13.2 s | 29 / 2.3 s | 5.7x |
| 33q | 854 / 33.8 s | 67 / 5.9 s | 5.7x |
| 45q | — / 149.7 s | — / 19.8 s | 7.6x |
| 57q | — / 530.7 s | — / 235.6 s | 2.3x |
| 69q | — / 1620.3 s | — / 922.4 s | 1.8x |

**Every chain optimum, every round total, every fault count and the round that
is kept are identical under both bounds** — 17q reproduces its whole
660000/700000/660000/700000 cycle, 57q its 1280000/1920000/1280000/1900000,
69q its faults 18/12/10/12 and the round it stands on.
That identity is the gate. A difference would mean the bound is not
admissible, and the exactness of the layered search would be gone with it.

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

### Two refusals the option space did not have

**What is left of the resonator may not run through the pad.** In
`Driver::makeOption`, right after the pad cell set is built. Nothing refused
it before: `free()` prices another wire's copper but exempts the resonator's
own (`owner != resonator.key`), and the self-intersection test asks whether
the path meets *itself*, not whether it meets the body. It rarely rejects an
option outright — `couplerPlace` walks on to the next place, so the coupler
moves rather than disappearing. Measured against the trellis: six chips
unchanged, **57q 5 undrawn → 2 and 2641 s → 357 s**, 69q 10 → 11 undrawn and
366 s → 1023 s.

**And it may not fall more than three tenths below the target length.**
`couplerMaxShortfall()`, `SCPD_COUPLER_MAX_SHORTFALL`. The walk through the
places carries **away from the figure** when a place is refused, and only an
overshoot was ever caught; on 69q that left a resonator of **1455 units
against 6000**, which the meander cannot recover — it lengthens a way, but not
by four times. The figure held against is the **spliced** length — lead, what
is left, and the run to the port — because the lead is absolute (~290 layout
units) and is a twentieth of a 6000-unit target but an eighth of a 2500-unit
one.

It works and it is not free. **At a tenth**, measured: 69q's resonators go
from spanning 1231–6131 units to 6008–6128 and from three below 5400 to none,
but the options over all 69 couplers fall from 3196 to **2726**, no coupler
keeps all 48, **17q loses two edges and goes from 2.1 s to 26.8 s**, and
**4q's angle goes from 8 to 32**. That cost is why the default is **three
tenths** (user, 2026-09-30). At three tenths 4q is back at 8 and 17q back at
zero undrawn and 2.0 s, and the case the rule exists for is still refused by a
wide margin: 1455 units against a floor of 4200.

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
# the prefix search is the default; this is the way back to the trellis, and
# `SCPD_CHAIN_DP=0` from there to the greedy
SCPD_CHAIN_DP=0 .venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml \
    -o artifacts/17q --stage final -v 1
# the bound the layered search leans on: 0 turnBound, 1 analytic (default),
# 2 the audit. Run the audit before believing any clock.
SCPD_CHAIN_BOUND=2 .venv/bin/mqt-scpd plan \
    -c benchmarks/9q/config.toml -o artifacts/9q-dp --stage final -v 1
```

**The run says which switches are in force.** The insertion prints one line
before it searches — `[Coupler Insertion] settings: search prefix A* (default),
each chain on its own chip yes (default), …` — naming each value and whether
it came from the environment or the default. Read it before believing any
sweep: trap 5 below is that a switch which does not take costs an hour, and an
**empty** assignment (`SCPD_CHAIN_SOLO=`) used to count as zero and turn a
default off in silence.

**`plan --stage final` reads the config *and the earlier stages' artifacts*
from the run directory.** This cost an hour and a wrong diagnosis said out
loud: three chips were measured with `stop_after = "couplers"` still in
`artifacts/<chip>/config.toml` while `benchmarks/` said `"feedlines"`. Copy the
config over, every time; and a fresh run directory needs `00-chip.json` and
`0[1-5]-*.fb` copied in or the stage refuses to start.

**Measure sequentially.** Running the chips in parallel contends for CPU and
the runtimes in the `==>` line become meaningless.

**And watch what else the machine is doing.** `artifacts/` grows to **15 GB**,
and every run rewrites the GDS and SVG in it, which sets Spotlight indexing it
again — a loop that does not settle while you are measuring. One 69q run came
back at **5553 s against 922 s** for the identical result (angle 160, 7 edges
undrawn) with `mds_stores` and `mediaanalysisd` taking about one and a half
cores between them. Check it before believing a clock:

```bash
ps -A -o %cpu,comm -r | awk 'NR>1 && ($2 ~ /mds_stores|mediaanalysisd/) {s+=$1} END {print s"%"}'
```

Measuring **CPU time** as well as wall time is the cheap insurance — the
process spends the same cycles whatever else is running, so the two agreeing
is what says a row is clean.

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
| `Driver::optimizeChainsPrefix` | **the default**: one pass over the chains, no rounds |
| `Driver::solveChainAStar` | one chain by the prefix search — see the other handover |
| `Driver::optimizeChainsExact` | **the trellis**: the rounds, the frozen fence, the round it keeps |
| `Driver::solveChain` | one chain, one round: the trellis built and solved |
| `Driver::edgeCost` | one edge priced for a named pair of options, through the memo |
| `Driver::turnBound` | the original admissible bound, `SCPD_CHAIN_BOUND=0` |
| `Driver::boundTurns` | which bound the trellis leans on |
| `routing::AnalyticDubins` | **the analytic bound**: the least turning with nothing in the way, the default; `minTurnsAround` the same with the artwork in the way, `SCPD_CHAIN_BOUND=3`, sound and measured useless (handover-chain-astar, *What is open*) |
| `Driver::measureSqueeze`, `refuseSqueezed`, `reportSqueeze`, `guardSettledTerminalRuns` | **the squeeze rule** and the terminal runs closed in the corridor, 2026-10-05 |
| `Driver::audit` | `SCPD_CHAIN_BOUND=2`: both bounds against the real price of a step |
| `Driver::stateFaults` | how many edges of a state would not survive its own fence — **the figure to read**, and about twice what the commit has to repair |
| `Driver::chainAStar`, `chainSolo`, `chainKeepWays` | the three switches of the prefix path, all on by default |
| `couplerMaxShortfall` | how far below the target an option may leave the resonator, three tenths |
| `envSet`, `envFlag`, `envWhole`, `envReal` | every switch read through one reader; an **empty** value counts as unset |
| `Driver::forgetEdgesNear`, `Driver::edgeBox` | what a round has to forget, and what it may keep |
| `routing::solveTrellis` | the search itself, router-free and unit-tested |
| `Driver::corridorOfEdge` | the box, the launcher stubs, the feedline fence, the own-resonator rule |
| `Driver::routeEdge` | one edge, searched |
| `Driver::drawCouplerOptions` | `final-coupler-options.svg` |
| `DubinsRouter::setBendLowerBound` | the bend term in the free search's heuristic |

## The squeeze rule (2026-10-05)

The one room rule that refuses. `measureSqueeze` reads, from every fifth
cell of a way past its two coupler runs, a straight line to either side at
a right angle until the first artwork cell inside the launcher rectangle
— a qubit or a tunable coupler, never a launcher pad or a CPW coupler
body — and counts the distinct wires whose copper the line crosses; `k`
of them need `clearance + k · pitch` cells, and the worst line of the way
is its shortfall. A resonator whose coupler is not applied yet counts only
on its way as the chosen option cuts it (`wayOfResonator`): its uncut
tail in the field is not a wire in the channel, and reading it as one cost
17q's chain 1 40000 before that was found (user). `refuseSqueezed`, from
`routeEdge` and from the commit's kept way, clears a way with a shortfall
above `SCPD_SQUEEZE_TOLERANCE` (0) as no way (`SCPD_SQUEEZE_REJECT`, on);
an edge the commit then cannot draw is drawn once more with the rule
suspended (`SCPD_SQUEEZE_RECOVER`, on, never needed on the eight chips).
`reportSqueeze` at the end says what came through and marks it in the
artifact (`FinalVerdict.Squeezed`, `note`, `marks`) for the pictures. The
measurements, the diagnostic it grew out of and what it did to the fails
are in handover-feedline-routing's *The squeeze report*: 17q 12 → 4, 69q
4 → 1, 20 → 8 over the eight chips, no open wire left.

## The room rules (2026-10-03/04, calibration only — nothing refuses yet)

The plan is `plan-coupler-room-rules.md`, the task `prompt-coupler-room-rules.md`.
Steps 1 to 3 of it are in the tree, **uncommitted**: the baseline logs, the
geometry library, R0, and the calibration lines that say what each rule
would have refused at the options the search chose. No rule is live, no
switch changes a decision, and the eight chips reproduce the baseline in
every `==>`, `CHECK`, `feedline routing` and `final routing` line (only the
work counts of the chains that reach the 10 s budget differ, as
[handover-chain-astar.md](handover-chain-astar.md) says they may).

**The geometry** is `include/mqt-scpd/routing/RoomRules.hpp`, router-free
and unit-tested (`test/routing/test_room_rules.cpp`, twelve tests):
`straightCells` is the crossing rule's own definition of a straight cell,
and `CrossingConstraints::build` now calls it, so R1 counts exactly the
cells the orthogonal rule accepts; `crossingCapacity` groups them into runs
and `lanesOf(run, pitch, margin) = 1 + ⌊(run − 1 − 2·margin) / pitch⌋`, zero
below; `channelBetween` reads the in-edge backwards past its pad run and the
out-edge forwards past its own and returns the least distance between the
two arms, where it occurs, and `startGap`, the distance where the arms leave
their runs; `supercoverLine`, `sideOf`, `sameSide`.

**R0** is `Driver::bridgersOf`: the `assignBridges` walk, both arcs, the
shorter kept, precomputed into `bridgers_[chain][at]` with one `tell` per
edge. `checkBridgers` runs after `assignBridges` in phase 4 and prints
`feedline routing: CHECK bridges — 0 wires whose bridged edge differs from
the insertion's count`; it is 0 on all eight chips, and the shorter-arc
guard never fired.

**The calibration lines** come from `reportRoom`, after the three checks,
under `SCPD_ROOM_REPORT` (on): one `room R1` line per edge between two
couplers, one `room R2` and one `room R3` line per coupler, and a fourth
`CHECK` line, `CHECK chain channels` (R4). The `settings:` line names
`SCPD_ROOM_PITCH` (clearance + 1 = 20), `SCPD_ROOM_MARGIN` (10),
`SCPD_ROOM_CHANNEL_REACH` (80), `SCPD_ROOM_CHANNEL_COUNT` (1) and
`SCPD_ROOM_REPORT`; a `room rules —` summary line stands before `==>`.
`artifacts/logs/calibration.py <log>` reads the verdicts out of a log,
`artifacts/logs/keylines.sh` strips a log to the lines an arm is judged by,
and `artifacts/logs/run-arm.sh <arm> [ENV=…]` runs the eight chips into
`artifacts/logs/<arm>/`. The baseline is `artifacts/logs/base` and
`artifacts/logs/base-ortho` (`SCPD_ORTHO_CROSSING=1`), the identity proof
`artifacts/logs/control`, the calibration `artifacts/logs/calib2`.

**What the calibration found, at the start values (pitch 20, margin 10,
reach 80).** The four known cases and the healthy chips:

| | 45q coupler 64 (f22/f23) | 57q coupler 8 (f2/f3) | 69q coupler 9 / wire 10 | 69q f5/f6 |
|---|---|---|---|---|
| R1 | room enough (f23: 13 lanes for 2) | room enough (f3: 9 lanes for 2) | — | — |
| R2 | silent: the arms leave the pad 64 apart and never come closer | silent, 62 at the pad | — | — |
| R3 | — | — | **red**: wire 10 lies 1.0 cell from the lead | — |
| R4 | — | — | — | **red**: gap 31 against 60 |

- **R1** flags 0 / 0 / 0 / 3 / 2 / 1 / 3 / 7 edges on 4q … 69q, none of
  them an open pair; it never flags the known cases. Under
  `SCPD_ORTHO_CROSSING` off a plain wire crosses its bridged edge anywhere,
  which is why 21q and 33q pass with 0 open while R1 would refuse edges on
  them.
- **R2** as the plan defines it — the least distance between the two arms —
  cannot see the two in-chain cases. In the artifact (`artifacts/logs/pinch.py`)
  the conflict point of 45q 65/66 lies 22 cells from the **in port** of
  coupler 64 and 22 from its **out port**: wire 65 runs along the pad at the
  clearance (19.8 cells from the pad axis), wire 66 at 38, and the two are
  0.65 cells short of the rule. The arms of f22 and f23 are 64 apart there
  and never come closer. 57q 9/10 is the same picture at coupler 8 (wire 9 at
  20 from the axis, wire 10 at 18.4 from 9). Nothing else is within 60 cells
  of either point in the `couplers` snapshot, so the second wall is not at
  the coupler: these are the lane-trading pairs of plain wires
  `handover-feedline-routing.md` describes, leaning on the pad. R2 with the
  divergence condition (silent when the arms never come closer than where
  they leave the pad — 62 of 69 couplers on 69q) flags 1 / 1 / 1 / 5 couplers
  on 17q / 45q / 57q / 69q, all healthy; most are jog options whose two edges
  run back side by side 19–20 cells apart.
- **"Bends right after the pad"** is not the discriminator either:
  `artifacts/logs/bends.py` shows a third of the edges with two or more
  crossers on 45q and 57q bend as soon as their stub allows, and one of them
  is open.
- **R3** at the chosen options, read as the nearest plain copper to the
  pad and to the lead apart (a wire at the run is where it crosses its
  bridged edge and says nothing). **The lead is the signal and the pad is
  noise.** A plain wire within one cell of a lead: 45q couplers 6 and 104,
  57q 128 and 149, 69q 9, 101, 106 and 212 — eight in all, 69q's coupler 9
  among them, and not one coupler anywhere between 1 and 10 cells. A plain
  wire *in* a pad (0 cells): 8 couplers on 57q, 7 on 69q, none on the
  others — these are seeds the feedline pass redraws around the body, and
  none of them is open. So R3 on the lead at a reach of 1 would move the one
  coupler that is open and seven that are not; R3 on the pad would move
  fifteen couplers for nothing.
- **R4** flags 69q f5/f6 (gap 31 at (2218,4182) against 60), the plan's
  case, and besides it found something the three checks do not look for:
  on 69q the terminal edges of four pairs of neighbouring chains **cross
  each other**, twice each — f32/f33, f46/f47, f60/f61, f73/f74 — two of
  them within three cells of a coupler's in port (`artifacts/logs/fcross.py`).
  `SCPD_EDGE_SEES_CHAINS` is off, so an edge is never routed against another
  chain, and nothing reports a feedline crossing a feedline. 45q and 57q have
  no such pair.

**Where the pieces are.** `Driver::roomPitch/roomMargin/roomChannelReach/
roomChannelCount/roomReport` (the switches), `RoomStats room_`, `ring_`,
`ringPlace_`, `bridgers_`, `chainOfCoupler_`, `liftedR1_/R2_/R3_` (reset per
insertion, read by nothing yet), `bridgersOf`, `checkBridgers`,
`channelAt` (R2's measurement, `CouplerChannel`), `capacityOfEdge` (R1's),
`reportRoom`, `checkChainChannels` (R4).

## What is open

**2026-10-05**: what is open now is in handover-feedline-routing's *What
is not done* and handover-chain-astar's *What is open* — the four 69q
chains at the clock, the eight crossing wires at the halo's edge, the
repair not rerun. The list below is the state of 2026-09-30, kept for
what it says.

**The first two below are closed by the prefix search, and are kept because
they say what it was for.** Items 1 and 2 describe the trellis; on the prefix
path 17q settles in one pass with every edge drawn, and 57q draws all 66. What
is open *now* is at the head of
[handover-chain-astar.md](handover-chain-astar.md)'s own list: seventeen of
69q's edges lie across others because the commit no longer tests them, two of
them because a feedline covers a coupler's own port, and the repair that would
move such a coupler (`repair_trials`) has never been run against this search.

1. **(Closed on the prefix path.) The exact search does not converge on
   17q.** It cycles between a state
   costing 660000 with three edges that would not survive it and one costing
   700000 with one. So on 17q the real choice is **angle 66 with two feedline
   edges lost, or angle 70 with all twenty**, and the gate picks 70. Nothing
   yet closes that gap; the cycle is inside chain 0, its own six edges fencing
   each other from round to round, and Gauss-Seidel against Jacobi makes no
   difference to it at all.
2. **(Closed on the prefix path.) 57q and 69q still lose edges, and the second
   order does not reach them.** 57q draws 61 of 66 and 69q 71 of 81; the greedy draws 60 and 75.
   The second order closes the conflict between the two edges that **share a
   coupler**, and on 17q that was the whole of it (6 undrawn to 0). What is
   left on these two is blocked by edges that are not neighbours — measured:
   57q loses the same 5 edges whether its two widest chains are solved at
   second order or handed back to the first, at 2641 s against 810 s. So a
   wider node is not the answer; what is needed is either a conflict the
   search can see between *any* two edges, or a repair at the commit — rip
   the edge that took the room, lay the one that lost, re-route the first.
   The commit is the one place the fence is real, and it is where the greedy's
   old two-order idea would actually have worked.
3. **The chains are independent and are not run in parallel.** With the rounds
   gone on the prefix path this is the largest runtime lever left, 4× on 17q
   and 12× on 69q. It needs one router context
   per thread: `router_`, `corridor_`, `box_`, `proximity_`, `stencils_` and
   `frame_` are all shared `Driver` state. The prototype has exactly this as
   `CouplerRouterCtx`.
4. **The bend term is verified identical only to 21q.** 33q and up were run
   with it on and never against it off. Before trusting it there, run one chip
   both ways and compare the angle cost, not only the clock — the prototype
   leaves the same term off by default and says why
   (`dubin_router_opt.hpp:263-286`).
5. **The cost still sees only the feedline.** Length, length difference and
   `100 × guarded` are commented out at `:3640`, so the two resonator ports of
   one coupler are an exact tie. The trellis makes the fix cheap in a way the
   greedy never could: a price per *option* is a node weight, and a node weight
   costs no routing at all — `TrellisProblem::nodeCost` is already there and
   unused. This is the obvious next experiment.
6. **The commit order decides who gets the room.** Edges are committed by
   chain index and the first one there takes the space. Committing by how
   little room an edge has left is the natural fix.
7. **Two fences are dead code.** `fenceResonators` has **no caller** — the
   own-resonator rule was written inline instead, and the two should be
   reconciled. `closeOutsideBox` has none either, switched off on request; the
   comment at its call site says how to switch it on.
8. **The edge picture draws stale prices.** Edge searches attach
   `zeroProximity_`, but `drawSearch` still renders `proximity_`, which holds
   whatever the last sweep search left.
9. **Diagnostics are still in the tree** — the `fence for edge …` and
   `picture …` lines and their counters.
10. **The configs are in a measuring state** and must go back before a commit:
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

**9. A saturating value must saturate, or it wraps and wins.** A round whose
chain the trellis could not join set the round total to
`TRELLIS_UNREACHABLE`, and the loop then went on adding the next chains' costs
onto it. On 69q that printed `total 399999` — `UINT64_MAX + 400000` — which
is *below* every honest round and would have taken the `total < bestTotal`
tiebreak. Fixed at `:4421`: once unreachable, stay unreachable. It changed no
chip's outcome, because 69q's faults 18/12/10/12 pick round 2 on the first
criterion and the total never got a say — but it was one tie away from
choosing the least real round on every run.

**10. A parameter that is silently ignored is worse than no parameter.**
`corridorOfEdge` took `more`, `ignoreAdjacent` and `pivot` and `(void)`-cast
all three. Three call sites were built on the belief that they did something;
all three were identical re-searches, and one of them made a counter that could
never increment print 0 into every log for weeks.

**11. An empty environment value is not an unset one.** `SCPD_CHAIN_SOLO=`
left in a shell or a script is a valid pointer to an empty string, and
`std::atoi("")` is zero — so a switch whose default is *on* silently went off
and nothing said so. All nine switches now read through `envSet`, which counts
an empty value as unset, and the insertion prints a `settings:` line naming
each value and where it came from. This is trap 10 again, one level down: the
parameter was not ignored, it was obeyed after being misread.

**12. `stateFaults` is not the commit's number.** It said 17 on 69q where the
commit kept 72 edges, routed 6 again and lost 3 — nine touched. It judges
every edge against every other edge's *searched* way, while the commit lays
them in order, so an edge routed again changes the ground for the ones after
it. Read it as an upper bound on what the commit has to repair.

**13. A count of what was drawn stops being a measure the moment the drawing
stops being tested.** With `SCPD_CHAIN_KEEP_WAYS` on, 69q draws 81 of 81 and
seventeen of them cross other edges. The undrawn count went to zero because
the test went away, not because the geometry improved.

## The coupler box margins (2026-10-06, built, swept and taken out)

Two margins were built on top of the box rule and **removed again at the
user's instruction**. Nothing of them is in the tree: `SCPD_COUPLER_BOX_MARGIN`
and `SCPD_LAUNCHER_FENCE_MARGIN` do not exist, and setting either does nothing
and says nothing. This section is here so the measurement does not go with the
code.

What they attached to are the two switches that *are* in the tree, both
**off**: `SCPD_COUPLER_BOX_TURN`, which draws the box at
`max(launcherStraight, straightStart + BEND_RADIUS) + clearance + 1` — 36 cells
from a launcher cell against the 32 it has — and holds every piece of a coupler
inside it with its own forced straight run and the quarter turn after it; and
`SCPD_LAUNCHER_FENCE_TURN`, which makes the fence `corridorOfEdge` lays on
every other launcher reserve `straightStart + BEND_RADIUS` rather than
`straightStart`. Each margin was that many cells further off the box side than
its rule asks.

**17q to `feedlines`, `bad = unrouted + open + crossing`**, the coupler
margin swept with both turn switches on:

| margin | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| `bad` | 4 | 5 | 4 | 4 | **2** | **1** |
| s | 59 | 56 | 59 | 57 | 51 | 52 |

A threshold and not a slope: the two open wires 31 and 32 fall away at 4 and
two of the three crossings at 5. The baseline with both switches off is `bad` 4
(14, 31, 33, 56) in 38 s, so a margin of 5 was the best figure ever measured at
this stage — `bad` 1, crossing 13 alone, no open wire, and **without**
`SCPD_CROSSING_EXIT_HEADING`. Above 5 is unmeasured, and so is every chip but
17q. The launcher-fence margin was measured only at 1 and 5 and is worse at
both.

**Why it is nevertheless out**, and this is the part worth reading before
building it again. A box refusal in `makeOption` does not discard an option; it
advances the place. `optionsOf` walks every place of the resonator's way, in
order of mismatch to the biased target length, and keeps the **first that
fits** — so the margin's primary effect is to move the cut, and an option is
lost only where no place of the whole way fits for that tuple. Both happen,
measured from margin 0 to 5:

- **28 of 611 options lost** (4.6 %), on the six couplers at the box edge:
  resonator 55 22 → 13, 44 11 → 7, 26 20 → 15, 15 31 → 26, 1 32 → 29,
  11 36 → 34. Resonators 15 and 44 each lose a whole orientation.
- **8 of 17 cuts moved, every one of them the same way**: less way left to the
  qubit, so the resonator gets *shorter* — 1 by 71 units, 11 and 40 by 70, 55
  by 60, 15 and 39 by 50, 30 by 28, 26 by 10. All eight were already short of
  the 3000 the rule asks, so the margin moves them further from it: the places
  the box still allows lie inward, and inward is the qubit's direction.

So the margin buys its `bad` by pushing couplers out of the launcher channel
and a little toward their qubits, and pays in options and in length. **Every
measurement here stopped at `feedlines`**, which is the phase *before* the one
that makes resonators their length, so whether the refinement wins those 71
units back is not known. That is the open question to settle first.

To build it back: two `envWhole` helpers and a third argument at the five
`couplerBox_.holds(...)` call sites in `terminalsInBox` and `makeOption` plus
the fence run in `corridorOfEdge`. `CouplerBox::holds` still carries the
`margin` parameter, unused. Arms: `runs2-m0` … `runs2-m5` for the sweep.

**Do not believe the arms `stub-m0` and `stub-m5`** (`bad` 2 at margin 0 and 5
at margin 5, the opposite ranking). They were measured while the box rule used
`straightStart` as the straight run an edge is forced to make off a *coupler*
port, which is a launcher's figure. The user caught it: `runsOfEdge` gives a
coupler port the pad's own run, `cellsOn(couplerLength, orientation)` — 21
cells axial and 15 diagonal — and `corridorOfEdge` then opens
`max(terminalSlot(), run + BEND_RADIUS)`, 26 at an axial coupler against 20 at
a launcher. Ten cells short at every coupler port, and with the figure
corrected the margin's ranking reverses. `edgeRunOffPort(orientation)` is the
one place that says it now, shared by `terminalsInBox`, `makeOption` and
`checkCouplerBodies`.

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
