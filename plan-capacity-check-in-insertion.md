# A capacity check inside the coupler insertion: cost and methods

Written for the user, who decides what goes into the coupler insertion, and for
whoever builds it.

The question: the capacity analysis after the coupler insertion
(`SCPD_BOTTLENECKS`, see
[handover-bottleneck-analysis.md](handover-bottleneck-analysis.md)) is to
exclude a feedline edge after which the capacity graph no longer carries the
outer wires, before the feedline pass fails on that edge. This plan measures
what such a check costs at each place in the insertion, what can be made
incremental, and which satisfiability checks are faster than the integer flow.
Nothing here is built into the insertion yet.

- Checkout `scpd-phase-4`, branch `phase-4-routing-stages`, HEAD `712cf7a` plus
  the uncommitted work of 2026-10-07/08. Nothing is committed.
- Every measurement:
  `SCPD_BOTTLENECKS=1 SCPD_LAUNCHER_FENCE_TURN=1 SCPD_SEARCH_PICTURES=0`,
  `--stage final`, on an otherwise idle machine. Times are the median of 3 (69q)
  or 5 (17q) runs with the range in brackets. The timing runs use `-v 0` without
  `-d`: without `-v` the stage prints nothing at all.

## What changed during this work (user, 2026-10-08)

The user changed two things in the analysis while the measurements ran. Both are
now the default, and both have a switch back.

1. **Only the bottlenecks the insertion has a hand in cut the chambers.** A
   bottleneck stays when one of its ends lies on a feedline edge, a coupler pad
   or a resonator's lead. Every other bottleneck is left out, and the chambers
   on either side of it are one (`couplerCuts`). `SCPD_BOTTLENECK_ALL=1` keeps
   every bottleneck.
   - 17q: 149 of 214 bottlenecks stay; 255 chambers instead of 321.
   - 69q: 197 of 743 stay; 348 chambers instead of 791.
2. **Each wire is checked alone instead of all wires at once.** A wire has a way
   when the edges open to it that take at least one wire join its source chamber
   to its target chamber (`pipeline::checkEachAlone`, a breadth-first search per
   wire). The verdict is `SAT` when every wire has a way. Otherwise it is
   `UNSAT, maybe not routable`, which names each wire and the edges that take no
   wire on the way it would have past them. `SCPD_CAPACITY_FLOW=1` restores the
   integer flow (`checkCapacity`).

With both switches set, the analysis gives the same 358 log lines as before,
word for word (times aside; checked on 17q).

The check alone is a **necessary** condition and not a sufficient one: two wires
that both need the one place of a gate that takes one wire both pass it. A wire
with no way alone leaves overflow in the flow too.

17q now reads:

```text
CAPACITY GRAPH UNSAT — … maybe not routable: no way alone through edges that
take a wire for 2 (closed at g70 2 at its source, left · f17) · 9 (… f0) ·
11 (… f1 · lead of 11) · 12 (… f2) · 14 (… f2) · 27 (… f8) · 38 (… f11) ·
41 (… f13) · 56 (… f17)
```

All nine are a wire's own source run against a feedline, the open question about
the own stub (handover, *What is open* 1).

## Where a check could sit

All of it is in `src/pipeline/FinalRouter.cpp`. The counts come from the logs of
one run each, `artifacts/logs/cap-rt/17q-base.log` and `69q-base.log`.

| Place                                  | Function                                                                       | 17q          | 69q           |
| -------------------------------------- | ------------------------------------------------------------------------------ | ------------ | ------------- |
| every priced step of the prefix search | `problem.step` in `Driver::solveChainAStar`, through `edgeCost` → `routeEdge`  | 2721         | 501           |
| the last step of a chain               | the same, `at + 1 == edges`: every run of options the search tries to complete | about 20–30  | about 120–170 |
| every complete run                     | `problem.found` in `Driver::solveChainAStar`                                   | 12           | 61            |
| every settled chain                    | after `solveChainAStar` in `Driver::optimizeChainsPrefix`                      | 4 (3 solved) | 12            |
| every committed edge                   | the commit loop in `insertCouplers`                                            | 20           | 81            |
| once, after the commit                 | `reportBottlenecks` (today)                                                    | 1            | 1             |
| the insertion itself, for comparison   |                                                                                | 39.7 s       | 102.3 s       |

The step counts are the `steps routed` of the four and twelve chains. The last
steps are counted from the `fence for edge cK-…` lines of the last layer, less
the lines the commit writes, so they are estimates.

**What the chip looks like at each place.** Only the commit and the end see the
whole chip. While a chain is searched:

- `edges_` is empty, so `wallsAfterInsertion` finds no feedline edge. The ways
  of the chains settled so far are in `chainEdgePaths_`, those of the prefix in
  the search's `laid` map.
- The chains not yet searched stand on their first options.
- A resonator's lead (`wire.arc`) is set only by `applyOption` in the commit.
  Before it, the lead is that of an option not yet applied.
- Under `SCPD_CHAIN_SOLO` (on) the other chains are not fenced at all.

A check inside the search therefore needs a wall builder that reads this state:
the settled chains' ways, the prefix's ways, every chosen option's pad and lead.
It still sees only part of the chip: a gate that only closes with a later chain
is not there yet.

**The baseline is not empty.** 17q has 9 wires with no way alone before any
check, and 69q has 24. A rule "exclude an edge after which the graph is UNSAT"
would exclude every edge. The rule must be differential:
**exclude an edge after which more wires have no way** (or, with the flow, after
which the shortfall rises). That is what the measurements below test.

## What one check costs

`SCPD_CAPACITY_BENCH=<n>` repeats the whole analysis `n` times on the chip as it
stands, one quiet pass each (`capacityPass`). The logs are
`artifacts/logs/cap-rt/17q-bench.log` and `69q-bench.log`.

| Step                           | 17q, s                     | 69q, s                  |
| ------------------------------ | -------------------------- | ----------------------- |
| walls                          | 0.0106 [0.0102..0.0108]    | 0.072 [0.072..0.074]    |
| distance transform             | 0.0635 [0.0629..0.0637]    | 3.368 [3.347..3.402]    |
| Voronoi                        | 0.0417 [0.0410..0.0423]    | 0.277 [0.266..0.283]    |
| axis                           | 0.0057 [0.0056..0.0059]    | 0.057 [0.055..0.059]    |
| cuts (`findBottlenecks`)       | 0.0098 [0.0097..0.0102]    | 0.096 [0.094..0.098]    |
| chambers                       | 0.0207 [0.0202..0.0210]    | 0.294 [0.293..0.300]    |
| stretches and ports            | 0.0012 [0.0011..0.0012]    | 0.0075 [0.0072..0.0079] |
| check, each wire alone         | 0.0001                     | 0.0001                  |
| **in all**                     | **0.154 [0.152..0.154]**   | **4.17 [4.14..4.22]**   |
| check, the flow (same graph)   | 0.190 [0.189..0.193]       | 0.604 [0.598..0.606]    |

The grid is 1425 × 1425 cells on 17q and 5400 × 5400 on 69q.

**Cold start and repetition do not differ.** The first call in the report (walls
0.011, distance 0.063, Voronoi 0.044, axis 0.006, cuts 0.010, chambers 0.022 s)
matches the repeats within their range. HiGHS shows no warm-up either.

**The distance transform is the whole problem on 69q.** The column pass of
`grid::squaredDistanceTransform` scans up to the square root of the best
distance so far, so its cost grows with the open space. A linear transform (the
lower envelope of Felzenszwalb and Huttenlocher), measured on the wall masks of
both chips (`artifacts/logs/cap-rt/edtbench.cpp`):

| Chip | today                   | linear                  | cells that differ |
| ---- | ----------------------- | ----------------------- | ----------------- |
| 17q  | 0.068 [0.068..0.069] s  | 0.013 [0.013..0.014] s  | 0                 |
| 69q  | 3.70 [3.49..3.75] s     | 0.241 [0.232..0.293] s  | 0                 |

With it, one check costs about **0.10 s on 17q and 1.05 s on 69q**, with the
same answer.

## What can be done incrementally

`SCPD_CAPACITY_BENCH` also leaves every drawn feedline edge out once and
compares the analysis without the edge to the one with it.

**How far one edge changes the cuts** (distance from the edge's way to the
middle of every cut that appears or disappears):

- 17q, 19 edges: median 50 cells, at most 156 (f5).
- 69q, 77 edges: median 56 cells, at most 363; three edges above 160.

**Distance, axis and cuts in a window around the edge** (the edge's box grown by
a reach, cuts compared in the middle half of the window):

| Reach     | 17q, s per edge          | cuts agree / only on chip / only in window | 69q, s per edge          | agree / chip / window |
| --------- | ------------------------ | ------------------------------------------ | ------------------------ | --------------------- |
| 120 cells | 0.0077 [0.0039..0.0108]  | 130 / 3 / 6                                | 0.0144 [0.0046..0.1708]  | 224 / 3 / 8           |
| 240 cells | 0.0175 [0.0119..0.0258]  | 231 / 0 / 0                                | 0.0453 [0.0175..0.2494]  | 327 / 2 / 9           |

These times use today's transform; the linear one lowers them further.

- **Distance transform and medial axis in a window.** It saves a factor of about
  8 (17q) to 90 (69q) on the geometry. The window must reach about 240 cells to
  give the same cuts on 17q. On 69q even 240 cells miss 2 cuts and find 9 that
  the whole chip does not have, because one edge changes cuts up to 363 cells
  away. The window border is free space to the transform, so the cuts near it
  are wrong; only its middle may be used.
- **Chambers and edges only near the window.** A chamber is global: one new cut
  can split a chamber that reaches across the chip. The chambers within 120
  cells of an edge hold 5–19 % of the free cells on 17q and up to 20 % on 69q,
  so a local flood saves about 80 %, but only with chamber numbers that stay the
  same from one check to the next. Today they are renumbered by their lowest
  cell on every call, so a new cut renumbers every later chamber.
- **The check.** Alone, it costs 0.1 ms; nothing to save. For the flow, see part
  2: a warm start of HiGHS saved nothing.

An incremental pass would cost about 0.025 s on 17q and 0.1 s on 69q per check.
It is not exact: the window must be large enough and its border handled, and the
chamber numbers must be kept stable.

## Added time, place by place

The insertion takes 39.7 s on 17q and 102.3 s on 69q. "Full" is one whole
analysis per check; "linear" is the same with the linear distance transform;
"window" is the incremental estimate above.

| Place                | Calls 17q / 69q | 17q full | 17q linear | 17q window | 69q full | 69q linear | 69q window |
| -------------------- | --------------- | -------- | ---------- | ---------- | -------- | ---------- | ---------- |
| every step           | 2721 / 501      | +419 s   | +272 s     | +68 s      | +2089 s  | +526 s     | +50 s      |
| last step of a chain | ~25 / ~150      | +4 s     | +2.5 s     | +0.6 s     | +626 s   | +158 s     | +15 s      |
| every complete run   | 12 / 61         | +1.8 s   | +1.2 s     | —          | +254 s   | +64 s      | —          |
| every settled chain  | 4 / 12          | +0.6 s   | +0.4 s     | —          | +50 s    | +12.6 s    | —          |
| every committed edge | 20 / 81         | +3.1 s   | +2.0 s     | —          | +338 s   | +85 s      | —          |

Two things the table does not show:

- **A chain under the clock does not get longer, it gets worse.**
  `SCPD_CHAIN_ASTAR_SECONDS` (10 s) bounds one attempt at one chain. 17q's chain
  1 and 69q's chains 1, 4, 6 and 10 reach it today. A check inside the search
  spends that budget, so those chains price fewer steps and may settle on a
  worse run or none.
- **A rejected run costs more search.** A check that rejects a complete run
  makes the search try the next one, and each of those costs edge searches
  (15–200 ms each) on top of the checks.

## Part 2: faster ways to decide

Measured on the capacity graphs after the change above (17q: 255 chambers, 216
edges, 58 wires; 69q: 348 chambers, 329 edges, 230 wires, 17 of them with no way
in the graph at all). The program is `artifacts/logs/cap-rt/flowbench.cpp`; it
reads the graphs `SCPD_CAPACITY_DUMP` writes. Results:
`artifacts/logs/cap-rt/flowbench-17q-cut.txt`, `flowbench-69q.txt`.

**The reference** is today's flow, `checkCapacity`: on 17q short by 10 on 10
edges, on 69q short by 15 on 8 edges, plus 17 wires with no way at all.

| Method                                                       | 17q, s          | 69q, s          | Answer                                                                 |
| ------------------------------------------------------------ | --------------- | --------------- | ---------------------------------------------------------------------- |
| M0 `checkCapacity` (through `milp::Model`)                   | 0.195           | 0.634           | the reference                                                          |
| M0b the same model, built straight into HiGHS                | 0.004 + 0.176   | 0.012 + 0.586   | the same; building the model is not the cost                           |
| M1b least overflow, no price on arcs                         | 0.120           | 0.456           | the same overflow                                                      |
| M1c decision, hard capacities, no objective                  | 0.013           | 0.042           | infeasible on both, which says nothing new: the baseline is not empty  |
| M1d decision, overflow at most the reference's               | 0.196           | 0.979           | feasible; no faster than optimising                                    |
| M1e decision, overflow at most the reference's − 1           | 0.183           | 0.809           | infeasible, which proves the reference least                           |
| M2a LP relaxation, least overflow                            | 0.044           | 0.167           | bound 10 and 15: **exact on both**                                     |
| M2b LP relaxation, hard capacities                           | 0.009           | 0.032           | infeasible                                                             |
| **A each wire alone** (`checkEachAlone`)                     | **0.00005**     | **0.0001**      | 9 wires (17q), 24 wires (69q); a lower bound                           |
| M3a cut condition, single edges                              | 0.0001          | 0.0003          | lower bound 5 and 14                                                   |
| M3b single edges and single-chamber cuts                     | 0.0002          | 0.0006          | lower bound 9 and 14                                                   |
| M3c and a min cut per wire                                   | 0.004           | 0.020           | lower bound 8 and 14 (packing of disjoint cuts)                        |
| **M5 negotiated congestion** (PathFinder, 40 rounds)         | **0.0003**      | **0.0004**      | overflow 10 and 15 on the reference's edges: **the optimum on both**   |
| M6a HiGHS warm-started with M5                               | 0.177           | 0.601           | no faster than cold                                                    |

**Decomposition.** The graph is not "almost a ring" after the change. On 17q, 99
of 255 chambers have no edge. Dropping chambers with no port and at most one
edge, and folding chambers with no port between two edges, leaves 135 chambers
and 195 edges in one component; the largest biconnected block holds 114
chambers. On 69q, 144 chambers and 308 edges are left in 5 components (the
largest 129 chambers); the largest block holds 63. A split by block leaves most
of the flow in one piece on 17q, and the heuristic is already below a
millisecond, so a split is not worth building.

**CP-SAT, SAT or pseudo-Boolean solvers** were not tried: they need a new
dependency, and the user's change replaced the solver with the check alone. The
heuristic and the LP bound together prove the optimum on both chips in under 0.2
s, so another exact solver has little to gain.

### The verdict that matters: does an edge raise the shortfall?

Each feedline edge is left out and put back. The truth is the reference on both
graphs: the edge raises the shortfall when the reference's overflow plus its
wires with no way at all rises. 17q has 19 edges, 8 of which raise it; 69q has
81 edges, 25 of which raise it.

| Method                                                   | 17q: hit / false / missed | 69q: hit / false / missed | Time per check, 17q / 69q |
| -------------------------------------------------------- | ------------------------- | ------------------------- | ------------------------- |
| more wires with no way alone (`checkEachAlone`)          | 7 / 0 / 1                 | 15 / 1 / 10               | 0.05 ms / 0.1 ms          |
| PathFinder overflow rises (M5)                           | 8 / 0 / 0                 | 25 / 0 / 0                | 0.3 ms / 0.4 ms           |
| PathFinder from the ways before the edge                 | 8 / 0 / 0                 | 25 / 2 / 0                | < 0.1 ms / < 0.1 ms       |
| LP bound rises (M2a)                                     | 8 / 0 / 0                 | 25 / 0 / 0                | 46 ms / 163 ms            |
| exact: overflow at most the one before (M1d)             | 8 / 0 / 0                 | 25 / 2 / 0                | 199 ms / 1144 ms          |
| cut bound (M3c)                                          | 0 / 0 / 8                 | 12 / 0 / 13               | 4 ms / 19 ms              |

- **The check alone misses the wires that compete.** 17q's one miss is f9: two
  wires (31, 32) need the one place of a gate. On 69q, 10 of 25 edges raise the
  shortfall only through competition, for example gate g35 between f73 and f74,
  which takes no wire and five wires need, or a stretch of f55.
- **PathFinder gives the reference's verdict on all 100 edges** at under half a
  millisecond. It is an upper bound: it can claim a rise that the optimum does
  not have, and nothing proves it right except the LP bound (M2a), which met it
  on both chips.
- The "2 false" of the exact budget check and of the warm PathFinder on 69q are
  edges after which one more wire has no way at all while the overflow falls by
  one: the total stays the same, the budget rule sees the overflow only.
- Warm starts save nothing for HiGHS. For PathFinder, carrying the ways over
  from before the edge (42–46 of 55 on 17q carry over) makes it nearly free, but
  it is already cheap from scratch.

## Recommendation

1. **Replace the distance transform with the linear one.** It gives the same
   value in every cell on both chips and takes the analysis on 69q from 4.2 s to
   about 1.05 s, on 17q from 0.154 s to about 0.10 s. It is a change in
   `src/grid/DistanceTransform.cpp` alone and helps the capacity stage too.
2. **Check per settled chain first, as a report.** After each chain in
   `optimizeChainsPrefix`, build the walls of the chip as it stands (settled
   chains, chosen options and their leads), run the analysis, and say which of
   the chain's edges raise the number of wires with no way. Cost: +0.4 s on 17q,
   +13 s on 69q (linear transform). This needs the wall builder for the search
   state and nothing else. It is the place to measure whether the prediction
   holds (handover, *What is open* 4) before anything is excluded.
3. **Then exclude at the last step of the chain.** When a settled chain raises
   the count, search it once more with the check in `problem.step` for the last
   edge only: a run that raises the count is priced unreachable and A\* takes
   the next. Cost while nothing is raised: none. When a chain is searched again:
   its search time plus about 25 (17q) to 150 (69q) checks, so +2.5 s on 17q and
   up to +158 s on 69q, unless the windowed geometry comes first.
4. **Do not check every step.** +272 s on 17q and +526 s on 69q with the linear
   transform, and the chains under the clock lose search instead of gaining
   time. A windowed, incremental analysis brings this to about +68 s / +50 s,
   but needs stable chamber numbers and a window that reaches far enough (363
   cells on 69q), and it still sees only part of the chip.
5. **The check:** `checkEachAlone` as the user set it, and PathFinder beside it
   to catch the wires that compete (0.3–0.4 ms). Keep the flow for the report
   (`SCPD_CAPACITY_FLOW=1`) and as the proof when a decision needs one; the LP
   bound proves PathFinder's answer optimal in 0.05–0.17 s when they agree.

## What is lost

- **The filter** leaves out every bottleneck between other obstacles, such as
  the artwork pockets in front of target ports (17q g33, g72, g73) and the gaps
  between two target runs. The insertion cannot change them, but they still
  limit how many wires pass, and the graph no longer counts them.
- **The check alone** does not see wires that compete for a gate: 1 of 8 raising
  edges on 17q, 10 of 25 on 69q.
- **A check inside the search** sees only the chains settled so far and the
  prefix; a gate that closes only with a later chain is not there.
- **A window** shorter than the reach of the change gives wrong cuts near its
  border: 120 cells disagree on 9 cuts on 17q; 240 cells still disagree on 11 on
  69q.
- **The differential rule** excludes an edge only when it makes things worse. It
  never repairs the 9 (17q) and 24 (69q) wires that have no way before any
  check.

## Decisions for the user

1. **The own stub.** On 17q, 7 of the 8 edges that raise the shortfall do so
   through a 0-cell gap between a wire's own source run and the new feedline. If
   such a gap is too strict for its own wire, the check would exclude good
   edges. This is the open question of 2026-10-08, and it decides whether an
   exclusion rule can be trusted.
2. **Check alone, or alone and PathFinder?** Alone misses 10 of 25 raising edges
   on 69q.
3. **Where:** per settled chain as a report first, then the last step on a
   second search (recommended), or another place from the table.
4. **The linear distance transform:** replace the current one?
5. **69q has 17 wires with no way in the graph at all** (19 20 41 43 44 72 73 74
   75 133 134 135 136 176 177 178 179), with the filter and with every
   bottleneck kept alike: the graph has no way between their two chambers
   whatever the capacities. This needs a look before a rule runs on 69q.

## Code and data of this work

- `src/pipeline/FinalRouter.cpp`:
  - `couplerCuts`, `bottleneckAll` (`SCPD_BOTTLENECK_ALL`), `capacityFlow`
    (`SCPD_CAPACITY_FLOW`), `flowOfAlone`, and `Wall::lead`;
  - `capacityProblemOf` (the graph and demands, shared by the report and the
    bench) and `bottleneckOptionsOf`; `wallsAfterInsertion` can leave one
    feedline edge out;
  - the measurement: `capacityBench` (`SCPD_CAPACITY_BENCH`, 0), `capacityDump`
    (`SCPD_CAPACITY_DUMP`, none), `capacityPass`, `dumpCapacityPass`,
    `benchCapacityCheck`.
- `src/pipeline/CapacityFlow.cpp`, `include/mqt-scpd/pipeline/CapacityFlow.hpp`:
  `checkEachAlone` and `AloneCheck`.
- Tests: `EachAlone.*` (6) in `test/pipeline/test_capacity_flow.cpp`; in
  `FinalRouter.DrawsTheCapacityGraphAfterTheBottlenecks`, every bottleneck edge
  has an end of the insertion (fails with `SCPD_BOTTLENECK_ALL=1`).
- Not in the package (`artifacts/` is ignored by git): `flowbench.cpp`,
  `edtbench.cpp`, the logs, and the dumps `dump-17q-cut/`, `dump-69q/` and
  `dump-17q/` (the graph before the change) in `artifacts/logs/cap-rt/`. Build
  line for `flowbench`, from the checkout:

  ```zsh
  B=build/cp311-abi3-macosx_15_0_arm64/Release
  clang++ -std=c++20 -O3 -DNDEBUG -Include -I$B/src/pipeline/include \
    -I$B/src/milp/include -I$B/_deps/highs-src/highs -I$B/_deps/highs-build \
    -I$B/_deps/nlohmann_json-src/include artifacts/logs/cap-rt/flowbench.cpp \
    $B/src/pipeline/libmqt-scpd-pipeline.a $B/src/milp/libmqt-scpd-milp.a \
    $B/lib/libhighs.a -lz -o artifacts/logs/cap-rt/flowbench
  ./artifacts/logs/cap-rt/flowbench artifacts/logs/cap-rt/dump-17q-cut 5
  ```

- The run that writes the dumps:

  ```zsh
  SCPD_CAPACITY_BENCH=5 SCPD_CAPACITY_DUMP=artifacts/logs/cap-rt/dump-17q-cut \
  SCPD_DEV_OVERLAY=artifacts/dev/py-cap SCPD_BOTTLENECKS=1 \
  SCPD_LAUNCHER_FENCE_TURN=1 SCPD_SEARCH_PICTURES=0 \
  .venv/bin/python artifacts/logs/dev-mqt-scpd.py plan \
    -c benchmarks/17q/config.toml -o artifacts/dev/17q-cap --stage final -v 0
  ```

## Built and measured: the check after every settled chain (2026-10-08)

The user asked for the check after every settled chain and for an evaluation on
17q alone: can the check find feedlines that leave no fail in the feedline
routing?

### What it does

`SCPD_CAPACITY_CHAIN=1` (off by default) calls `checkChainCapacity` after each
chain `optimizeChainsPrefix` settles.

- `stateOf` builds the chip as the chains settled so far leave it: their
  couplers with pads and the leads `applyOption` would give them, and their
  feedline edges, numbered as the commit numbers them. A resonator whose coupler
  does not stand yet is left out.
- `lostWires` runs the analysis on that state (`capacityPass` with `wallsOf`)
  and returns the wires with no way alone.
- The wires that have a way before the chain and none after it are the ones the
  chain closes. The log says `[Capacity Check] chain k: … more with it`.
- When there are any, the chain is searched again. A complete run of options is
  refused at its last step when it closes as many wires, or when more of its
  edges fail the commit's test (`chainFaults`, the per-chain part of
  `stateFaults`). The cheapest run that closes fewer is taken; this repeats up
  to `SCPD_CAPACITY_CHAIN_TRIES` (3) times.

The commit test was added after the first run: a run that closed no wire moved a
coupler to an option whose first edge did not survive the commit
(`SCPD_CHAIN_KEEP_WAYS` is off), and f0 was left undrawn.

One check costs 0.33 s on 17q (two analyses: before and with the chain). A
search again costs the chain's own search time, up to the clock.

### Results on 17q

`--stage final`, `stop_after = "feedlines"`, no repair,
`SCPD_BOTTLENECKS=1 SCPD_SEARCH_PICTURES=0`. `bad` = unrouted + open + crossing
of the final routing. Logs: `artifacts/logs/chain-check/`.

| Run                                     | Predicted at the end (no way alone) | bad (u/o/c)   | Fails                   | Insertion |
| --------------------------------------- | ----------------------------------- | ------------- | ----------------------- | --------- |
| check off                               | 2 9 25 31 38 56                     | **4** (0/0/4) | crossing 14, 31, 33, 56 | 5.7 s     |
| check on                                | 2 9 31 56                           | **4** (0/0/4) | crossing 14, 31, 33, 56 | 78 s      |
| check on, `SCPD_CHAIN_ASTAR_SECONDS=60` | 2 9 31 56                           | **4** (0/0/4) | crossing 14, 31, 33, 56 | 361 s     |
| `SCPD_LAUNCHER_FENCE_TURN=1`, off       | 2 9 11 12 14 27 38 41 56            | **5** (1/2/2) | f7; open 31, 32; 42, 56 | 41 s      |
| `SCPD_LAUNCHER_FENCE_TURN=1`, on        | 2 12 14 27 41 56                    | **5** (1/2/2) | f7; open 31, 32; 42, 56 | 106 s     |

**The check finds no feedline that removes a fail.**

- It moved chains where it could: chain 1 freed 25 and chain 2 freed 38 (for
  more cost). Neither wire failed in the routing.
- Where the prediction was right (31 and 56 without the fence turn), no run of
  the chain leaves the wire a way: 405 runs of chain 1 and 333 runs of chain 3
  were checked and refused in 2 × 60 s. Chain 1's edge f8 and chain 3's edge f17
  must pass in front of the launchers of 31 and 56.
- Chain 0 found no run that leaves wire 9 a way (416 runs in 2 × 60 s), and wire
  9 routes without a fail.

**The prediction does not match the fails.**

| Run                          | Predicted and failed | Predicted, no fail           | Failed, not predicted |
| ---------------------------- | -------------------- | ---------------------------- | --------------------- |
| check off                    | 31, 56               | 2, 9, 25, 38                 | 14, 33                |
| `SCPD_LAUNCHER_FENCE_TURN=1` | 56                   | 2, 9, 11, 12, 14, 27, 38, 41 | f7, 31, 32, 42        |

- Every predicted wire is closed by a gap of 0 wires between its own source run
  (or, for 11, its lead) and a feedline. Most of them route anyway: wire 25 has
  the same 12-cell gaps to f7 as wire 31 has to f8, and 25 routes in round 0.
  This is the open question about the own stub; the analysis cannot tell 25 from
  31.
- Wire 31 fails because its search starts within the clearance of f8
  (`dead on arrival`, way at 19.0 cells). That is a cell-exact test at the
  terminal, not a capacity question.
- 14 and 33 fail crossing the edge they bridge (f2 at 10 cells, f9 at 10 cells).
  The graph gives them a crossing stretch: 14 a stretch of 13 cells near the end
  of f2 that takes one wire.
- With the fence turn, chain 1 runs out of time and is not settled, so the check
  never sees it, and its edge f7 stays undrawn.

### What would have to change first

1. The rule for a gap at a wire's own port run. As it stands, most predictions
   are wrong (4 of 6, 8 of 9). If the own wire may pass, the predictions for 2,
   9, 25, 38 go, and so do those for 31 and 56.
2. A test for the fails that do occur: a terminal whose first cells lie within
   the clearance of a feedline (31), and a bridged crossing with too little
   straight room on the edge (14 on f2).
3. A chain that the search cannot settle in time is not checked at all.
