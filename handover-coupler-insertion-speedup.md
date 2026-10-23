# Coupler insertion speed-up, 2026-10-21 to 2026-10-23

Written for whoever goes on with the run time of the Final stage after
2026-10-23. It covers what made the coupler insertion fast and
deterministic, what was tried and dropped, why 69q did not finish for a day,
and where the time of the Final stage goes now.

Read [handover-global-sat.md](handover-global-sat.md) for the global SAT
search and [handover-slot-check.md](handover-slot-check.md) for the slot
check. [stub-bound-handoff.md](stub-bound-handoff.md) holds the analysis
behind the stub bound.

## State

- Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`.
- The work is the squash of branch `wp11-final` (worktree
  `.claude/worktrees/wp11-final`, 59 commits on `0c18a7b`). It lands in
  two commits:
  - `a5d6640` holds the first part: no clocks, the new defaults up to the
    profile memo, and the router turn bound and the dropped experiments as
    switches that are off.
  - The next commit, staged in the main checkout, holds:
    - the step-check fix;
    - the three exact speed-ups on by default;
    - the refusal limit;
    - the Fails/Bad split;
    - the removal of the dropped experiments;
    - the cleaned changelog.
- Not committed, and not part of the work: `*-final.svg`, `COMMIT_MSG.txt`,
  the `handover-*.md` files and this file.
- Still to do:
  - The changelog entries carry no PR reference or author yet. AGENTS.md
    asks for both once the PR exists.
  - Four switches, off by default, were not judged in this work:
    `SCPD_GLOBAL_SAT_EARLY`, `SCPD_SEGMENT_MEMO`, `SCPD_CHAIN_LENGTH_WEIGHT`
    and `SCPD_CHAIN_LEARNED_BOUND`. `SCPD_CHAIN_LENGTH_WEIGHT` is close to
    the length cost that was dropped and is a candidate for removal.

## Results

Final stage, one chip at a time on the Mac, from a run directory that holds
the stages before it. "Start" is `0c18a7b` with its defaults and the 20 s
clocks (`artifacts/logs/bench-globalsat-both-2026-10-21`). "End" is
`wp11-final` with its defaults and no switch set
(`artifacts/logs/gate-final-2026-10-23`).

| Chip | Start | End | Factor | Insertion start → end | Fails / Bad end | Angle cost start → end | Global SAT end |
|---|---|---|---|---|---|---|---|
| 4q | 6.1 s | 5.4 s | 1.1× | 0.4 → 0.1 s | 2 / 0 | 8 → 8 | – |
| 9q | 11.2 s | 10.1 s | 1.1× | 1.3 → 0.3 s | 0 / 0 | 20 → 20 | after the commit |
| 17q | 59.2 s | 21.8 s | 2.7× | 31.5 → 1.4 s | 4 / 0 | 49 → 53 | after 1 combination |
| 21q | 68.9 s | 51.5 s | 1.3× | 21.9 → 4.7 s | 0 / 0 | 52 → 54 | after 39 combinations |
| 33q | 126.1 s | 73.8 s | 1.7× | 50.8 → 1.6 s | 1 / 0 | 76 → 76 | after the commit |
| 45q | 311.9 s | 143.7 s | 2.2× | 202.3 → 31.2 s | 0 / 0 | 107 → 107 | after 1 combination |
| 57q | 323.3 s | 208.6 s | 1.5× | 130.5 → 13.1 s | 2 / 0 | 134 → 138 | after the commit |
| 69q | 1387.1 s | 530.2 s | 2.6× | 886.1 → 36.5 s | 0 / 0 | 176 → 178 | after 1 combination |
| 4q–57q | 906.6 s | 514.9 s | 1.8× | | | | |
| all eight | 2293.7 s | 1045.1 s | 2.2× | | | | |

- `Fails` counts the resonators of the wrong length (short or long). `Bad`
  counts the wires with no usable way (unrouted, open, crossing a feedline,
  meeting itself). Every Fail above is a short resonator.
- At the start, 69q ended with 3 fails and was globally SAT only because the
  clocks searched four chains again without the capacity rule.
- The end-edge weight and the offsets (below) change results: the angle cost
  rises on 17q, 21q and 57q, and 57q ends with 2 short resonators instead of
  none. The user chose this on 2026-10-22. Every other change keeps the
  result or makes it cheaper.

## What the work contains

### Deterministic insertion

- The coupler insertion reads no clock. Its searches stop on a count or when
  they are done, so every machine gives the same result.
- The global SAT search tries its combinations in a fixed order, with no
  random generator.
- `[Chain Progress]` lines (`SCPD_CHAIN_PROGRESS_STEPS`, 100) show a search
  that grows without end.
- `SCPD_CHAIN_RULE_REFUSALS` (1500, 0 off) is the safety net.
  - A chain whose capacity rule refuses that many steps before any run joins
    it is searched again without the rule. The log says the count.
  - On the eight benchmarks at most 80 refusals come before a first run
    (33q chain 1, before the step-check fix), so the net never acts there.

### New defaults

All of these are on by default. `=0`, or the value named, gives the earlier
path.

| Switch | Default | What it does | Exact? |
|---|---|---|---|
| `SCPD_CHAIN_BOUND` | 4 | Bounds a step by the analytic Dubins answer with the forced end runs folded in (stub bound). `1` gives the earlier bound. | yes |
| `SCPD_SLOT_CHECK_FAST` | on | The faster slot check; same results on all 809 graphs of the golden file. | yes |
| `SCPD_PROFILE_MEMO` | on | The profile cuts keep traces, lines and narrowings of the walls from step check to step check. `SCPD_REUSE_VERIFY=1` checks it. | yes |
| `SCPD_ROUTER_TURN_BOUND` | on | The edge searches of the insertion add bend penalty × `AnalyticDubins::minTurns` to the heuristic and drop moves that cannot end within the turn budget. 45q chain 6: 357 M → 92 M states. | yes |
| `SCPD_EDGE_SEARCH_MEMO` | on | Replays an edge search whose whole input repeats: ends, runs, bend penalty, turn-bound window, edge box, and a 128-bit hash of the corridor in the box. 45q chain 6: 1499 of 1762 searches replayed. `SCPD_EDGE_SEARCH_MEMO_VERIFY=1` checks it. | yes |
| `SCPD_CHECK_ONLY_WINNERS` | on | Leaves out the step checks whose verdict the A* never reads once a cheapest run is in hand. 45q chain 6: 839 checks left out. | yes |
| `SCPD_CHAIN_END_EDGE_WEIGHT` | 0.1 | Prices the edges between a launcher and its coupler at a tenth. | no, user's choice |
| `SCPD_COUPLER_OFFSETS` | 0,1,2,5,6,7 | Leaves out offsets 3 and 4, which were never chosen on 4q–45q. | no, user's choice |
| `SCPD_SECOND_DOGLEG` | off | Opens no second-dogleg option. | no, user's choice |

"Exact" means that `06-final.fb` stays byte-identical apart from the version
string at its end. Compare with that string masked.

### The step-check fix

The local step check of the chain search judged a chip that never exists.
Two faults made it refuse runs that the whole-chip check finds SAT.

1. **Pads on the wrong option** (`b2650d4`).
   - `wallsOf` drew each coupler's pad on `couplers_[i].chosen`, which is still
     the coupler's earlier option during the search. The edges, however, ran to
     the ports of the step's options.
   - On 69q chain 11 (`0 9 0 20 0 31`), the phantom pad of coupler 223 made a
     gate of capacity 1 that wires 219–222 all had to pass.
   - `InsertionState::options` now records every coupler's option, and the
     pass draws pads and the pad tips of mouths on it.
   - The fault had been in the code since 2026-10-09. The 20 s clocks hid it:
     each such chain ran out of time and was searched again without the rule.
2. **A terminal edge took room** (`996d718`).
   - In the check of a chain on its own, a terminal edge was an inflated wall,
     and a gate between it and a near edge held no wire. On 69q chain 2, f13
     passes 15 cells from f16. On chain 8, f54 runs 20 cells beside f55 for 75
     cells.
   - The whole-chip check treats terminal edges as wires, because the feedline
     routing lays them again.
   - A terminal edge now stays a thin wall that closes the chain's pocket. Every
     gate with an end on it takes every wire, and it covers no port.
   - This reverses, for the local check only, the rule of 2026-10-13 that a
     gate at a terminal edge takes what it measures. The user agreed on
     2026-10-23.

With both fixes:
- 69q settles all 12 chains in about 35 s without a refusal.
- 33q, 45q and 57q settle cheaper chains, and the angle cost falls on 45q and
  57q.
- Tests: `TheStepCheckStandsEveryPadWhereTheStepLaysItsEdges` and
  `TheStepCheckLetsEveryWirePassBesideATerminalEdge`.
- "Short by N" in a step line counts the gates and slots over capacity, not
  wires. "Short by 0" with lost wires means those wires have no way at all.

### Measurement lines

`SCPD_STEP_PROFILE=1` prints:
- `[Capacity Split]`, `[Edge Split]` (with states expanded and searches
  replayed), `[Router Classes]`, `[Slot Split]`, `[Saddle Split]`,
  `[Profile Split]` and `[Chambers Split]`;
- per chain, the step checks left out.

`SCPD_OPTION_DUMP=1` writes `option-stats.jsonl`, with how every coupler
option was priced and chosen.

## What was tried and dropped

All measured on 45q unless named. None of these is in the commit.

| Idea | Result | Why dropped |
|---|---|---|
| Weighted A* in the chain search (w 1.05, 1.2) | no gain | the incumbent is found early; the proof costs the time |
| Length cost in the chain price | slower | the user rejected it |
| Lazy step check (check when taken) | small | rejected |
| Pair memo of free-space ways | 189 s against 195 s | not exact under the fence; rejected |
| Free-space bound and prefilter probe | the probe takes about 10 s | not a proved bound |
| Option equivalence classes | small | rejected |
| Multi-target router search (ε 0 and 1) | 213 s against 223 s | small gain, cut-offs taken as final |
| Resumable router search | 228 s against 223 s | slower |
| Pre-check of the bound against the turn budget ("lever A") | 0 of 12 cut-offs | cannot fire by construction |
| Slot-rule cache in the slot check | rule costs 0.1 ms per check | nothing to gain |
| Flat mouth buckets in `closeMouths` (branch `wp7-routing`) | −0.25 s | too small; not merged |
| Rubber-band router in the edge searches (worktree `wp8-rubberband`, uncommitted) | see below | changes results |

The rubber-band router came from the session "Rubberband routing
evaluation". It was tried in `routeEdge` in two modes:
- **Hybrid:** rubber band first, the A* where it finds nothing.
- **Price-exact:** the A* is skipped where the rubber-band turns equal the
  analytic bound.

Both modes price every edge right, but lay another way at the same price on
7 % (45q) to 15 % (17q) of the searches. The capacity checks of later steps
read those ways. On 45q chain 7 rose from 64000 to 86000, and the global SAT
search needed 59 combinations instead of 1. On 21q chain 1 rose from 28000 to
30000. 45q insertion: 77.6 s base, 79.9 s hybrid, 95.0 s price-exact. Only a
cell-for-cell match keeps the results, and proving that needs the A*.

## Where the time goes now

69q, 530 s, from `artifacts/logs/gate-final-2026-10-23/69q.log`:

| Phase | Seconds |
|---|---|
| Outer routing | 41 |
| Outer refinement (10 rounds) | 188 |
| Coupler insertion with global SAT | 37 |
| Feedline routing | 161 |
| Feedline refinement (2 rounds) | 101 |

The coupler insertion is 7 % of the stage on 69q and 22 % on 45q. The next
large levers are in the routing and refinement passes. Analysis of
2026-10-22, all expected exact, estimates for 45q unless named:

1. **Make the `whatBlocks` diagnostic optional** (`attempt`, the feedline
   pass at `-v 1`).
   - Every failed phase-1 search runs 4–5 extra searches to name what
     blocked.
   - About 5 s on 45q, 8 s on 57q, and an estimated 50–70 s on 69q.
2. **Reuse the corridor within one `attempt`.** Phase 1 and every relaxation
   level flood the same corridor again: 4–5 s.
   - The corridor can open cells outside `box_` (`openFixedPlaces`), so copy
     the whole corridor BitGrid, not only the box rows.
3. **A leaner corridor flood:** no `uint32` `seen_` per cell, one
   blocked-or-bodies grid, two frontier vectors. 3–5 s.
4. **A table for the crossing rule in the orthogonal search**
   (`canCrossOrthogonal`). 2–2.5 s.
5. **Free strips of meanders from prefix counts** along a line. 1.5–3 s.
6. **The room price on open row spans only.** 1–2 s.

In the insertion itself, caching the slots of the prefix strokes from check to
check could save at most about 5 s on 45q. Its key must cover the raster
window, the crossing rule and the taken ways nearby.

## How the work was run

- **One heavy job at a time.** Forked replays crashed the Mac four times.
  - Every build, test run and benchmark took the lock in
    `artifacts/logs/speedup-queue/lock.sh` (`take` … `give`).
  - Lid closing puts the Mac to sleep whatever `caffeinate` says, and a run
    across a sleep has wrong timings. `pmset -g log` shows sleeps.
- **The live dashboard** is in `artifacts/logs/speedup-dash`, port 8793:
  - `refresh.py` builds `metrics.json` from every arm log under
    `artifacts/logs/<arm>/<chip>.log`, and from the agents' scratchpad logs;
  - `status.json` holds labels, the reference arm and the commit plan.
- **Reproduce the gate** with `artifacts/logs/speedup-queue/gate_final.sh`:
  - It copies stages 00–05 from `artifacts/<chip>` into a fresh run directory
    and runs `mqt-scpd plan … --stage final -v 1` with no switches.
  - Never give `-c` a `config.toml` that already lies in the output
    directory.
- **Lint.** Run `prek` on the changed files only:
  `uvx prek run --files <files>`. A repo-wide run rewrites many unrelated
  files. `typos` turns the identifier `nd` into `and` and breaks the build.
- **Branches kept for reference:**
  - `wp10-debug69`, commit `937dd27`: debug switches to replay one chain run
    (`SCPD_CHAIN_FORCE`) and dump its step graphs (`SCPD_STEP_GRAPH_DIR`).
  - `wp7-routing`: the flat mouth buckets, and a time split of slot placement
    per stroke.
  - `wp8-rubberband` (uncommitted): the rubber-band router and its
    integration.

## Known failures that are older than this work

- **Tests.** The same tests failed on `0c18a7b` before this work (commit
  `ace63e0`):
  - `FinalRouter.OnlyUnsettledRedrawsOneWire`
  - `FinalRouter.ACrossableTerminalEdgeTakesEveryWire`
  - `CouplerSession.ClosesTheMouthBetweenTwoChainsWithTwoLines`
  - `CouplerSession.ClosesAMouthWhoseLinesWouldRunOverTheSlotsWithOneLine`
- **Lint findings.** These are in files this work did not touch, or are
  older than it:
  - `ty` reports `None` attributes in `artifacts/logs/*.py` and in
    `test/python/unit/test_planning.py:230`;
  - `ruff` reports an import inside a function in `python/mqt/scpd/cli.py`;
  - `rumdl` reports the second `### Changed` heading under `[Unreleased]` in
    the changelog.
