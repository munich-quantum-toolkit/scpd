# Plan: targeted re-search of coupler options after the feedline pass

Approved by the user on 2026-10-04. The task prompt that points here is
`prompt-coupler-repair-search.md`; the start prompt for a fresh session is
`prompt-coupler-repair-start.md`.

For a fresh agent. Repository `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`,
branch `phase-4-routing-stages`, HEAD `63851cf` plus the **uncommitted** room-rule
work of 2026-10-04 (RoomRules library, R0, calibration lines; see *The room
rules* in `handover-cpw-coupler-insertion.md`). The user commits per phase:
leave everything in the working tree. Every new switch is read through
`envFlag`/`envWhole`/`envReal`, has a documented default with the measured
reasoning in its comment, and appears in a `settings:` line. Report to the
user in German; repository content in English. Line numbers below were read
on 2026-10-04 and drift as code is added: grep the identifiers.


## Context

The feedline pass leaves seven wires open in the last round over the eight
benchmarks (45q 2, 57q 1, 69q 4) and, with the orthogonal crossing rule on,
28 (17q 4, 33q 3, 45q 2, 57q 9, 69q 10) plus 22 wires crossing a feedline
against the rule. The room rules measured on 2026-10-04 that **no local
geometric rule at the coupler separates the open pairs from healthy
couplers**: the plan's channel rule misses both in-chain cases (they are
lane pairs of plain wires 0.65 cells short, leaning on the pad), the crossing
capacity rule flags only healthy edges, the own-room rule flags one open and
seven healthy couplers. The user's conclusion: a heuristic does not reach
these; what reaches them is a **trial that changes coupler options and
routes again** — but not the blind `Driver::repair` (turn a coupler near a
fail to one of its two cheapest other options, re-sweep the whole ring,
keep on strictly fewer fails). Instead:

1. **Triage.** After the baseline feedline routing, read every fail, find the
   feedline edges and couplers that can be its cause, and group them into
   connected **segments** of one chain.
2. **Re-search.** Re-open the prefix search for exactly those segments —
   both ends fixed, the rest of the chip frozen — and let the A\* run on
   past its first answer: every complete run of options it reaches, in
   order of cost, is tested by a **local rip-up-and-reroute** of the wires
   the segment touches plus k ring neighbours, under the orthogonal crossing
   rule. The first run whose window ends clean and whose chip is not worse
   is kept.
3. Nothing else is done to the heuristics.

## Decisions taken with the user (2026-10-04)

1. **Regime:** `SCPD_ORTHO_CROSSING=1` becomes the default. The figure is
   open in the last round plus wires crossing a feedline against the rule
   (and unrouted, which is 0 today). Baseline under the rule
   (`artifacts/logs/base-ortho`, 2026-10-03):

   | | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q | all |
   |---|---|---|---|---|---|---|---|---|---|
   | open, last round | 0 | 0 | 4 | 0 | 3 | 2 | 9 | 10 | 28 |
   | open, end | 0 | 0 | 4 | 0 | 4 | 6 | 17 | 24 | 55 |
   | crossing a feedline | 0 | 0 | 4 | 0 | 1 | 2 | 8 | 7 | 22 |
   | fails (incl. lengths) | 4 | 1 | 15 | 2 | 12 | 21 | 47 | 59 | 161 |

   The defaults regime (rule off, 7 / 18 / 0) is measured once as a control
   arm and reported; nothing is tuned for it.
2. **Where:** in place of `Driver::repair` in phase 4, the same run, right
   after the two sweeps (`:11159-11168`); `repair_trials` becomes the budget
   (candidates tested per chip). The search state stays in memory.
3. **Criterion:** a candidate run of options is kept when its local window
   ends with 0 open, 0 crossing, 0 unrouted, and the chip's count of
   (open + crossing + unrouted) over every wire is strictly below what it
   was before the segment (lengths are not a criterion). A full feedline
   pass after all segments is a measured arm, not part of the criterion.
4. **k:** a switch, start 2, sweep 1 / 2 / 3.

## What the code offers (read 2026-10-04; `src/pipeline/FinalRouter.cpp` unless stated)

- **The A\* keeps no state.** `routing::solveChainAStar`
  (`src/routing/ChainSearch.cpp:113-114`) holds arena and queue as locals;
  `ChainSolution` carries `chosen` and counters. `Driver::solveChainAStar`'s
  `laid` map (`:6660`) is local too. Nothing resumes; a re-search is a fresh
  `ChainProblem`. What survives the insertion: `couplers_[c].options`
  (built once, `:2890`; indices stable), `chainEdgePaths_`, `edgeMemo_`
  entries of middle edges (`CHAIN_FRESH_EDGES = 2`, `:4134`), `jogsUnlocked`
  (monotone).
- **A sub-chain is a sanctioned problem.** `ChainSearch.hpp:65`: "a layer of
  one is a fixed waypoint". `width = 1` at both ends of `[lo, hi]`, the real
  widths between, the bound matrix by the same double loop over
  `portOfOption` / `boundTurns` (`:6676-6705`) with indices shifted by `lo`.
  `width == 0` returns unsolved at once (`ChainSearch.cpp:81-86`). The
  deferred pricing: a child is pushed with the bound, priced when popped,
  re-pushed, and expanded only when popped real; a complete real node that
  pops returns `solved` (`ChainSearch.cpp:206-215`); `found` is called for
  every complete run as it is priced (`:185`).
- **The frozen rest is fenced through `prefixFence_`** (`:4765`, consumed by
  `corridorOfEdge` `:4454-4465`): any `const Path*`s. `fenceCommittedEdges`
  (`:4148-4228`) fences this chain's committed edges by `edgeWayOf` (the
  wire's way once drawn) but skips later edges except the neighbour
  (`fenceLaterEdges()` off, `:687`) and other chains unless
  `fenceEverything_` (`:4168-4170`; `SCPD_EDGE_SEES_CHAINS` off). `corridorOfEdge`
  never closes ring wires (`:4378-4381`); the leads and pads it closes are
  read off `couplers_[c].options[chosen]` (`:4475-4501`), so `edgeCost`'s
  temporary `stand()` of `chosen` (`:6184-6192`) fences a candidate option's
  own lead and pad.
- **Un-applying a coupler exists**: `turnCoupler` (`:7718-7751`) lifts the
  two edge wires, `applyOption`s (`:7382-7439`: body bits, `chosen`, lift,
  `arc`, `objective.source`, `startStub`, `fixed.clear()`, `way = option.way`,
  place), rebuilds the edges' objectives from `sourceOf`/`targetOf`,
  `routeEdge`s both, `place`s. `snapshot`/`restore` (`:7673-7714`) roll
  `wires`, `bodies_`, `chosen`, `anchor` and the whole field back and
  rebuild the crossing rule under a feedline pass; they do not touch
  `chainEdgePaths_`, `edgeMemo_`, `jogsUnlocked`.
- **The repair today** (`:7793-7962`): candidates from the coarse flag
  `!drawn || !routed || tooShort || tooLong` (`:7853`; a wire that only
  crosses a feedline nominates nothing), two cheapest untried options by
  `option.cost`, each trial a whole-ring `sweep` with `rounds = 1`, kept on
  strictly fewer `failsOf(every).total()`, then re-applied and re-swept.
- **Fail facts per wire exist**: `failsOf` (`:2239-2302`) fills `openIds`,
  `crossingIds`, `unroutedIds`, `shortIds`, `longIds`, `loopIds`;
  `conflictsIn(way, wire, wires, &blockers, &where)` (`:8671-8745`) gives
  the partner keys and the own-way conflict cell; `whoBlocks` (`:8986`)
  formats them; `crossesAFeedline`/`whereItCrosses` (`:2306`, `:2334`) give
  the crossing cell and the nearest coupler; `whatBlocks` (`:8311-8400`) is
  the ablation classifier. `Wire::bridged` (`:1500`) names the edge a plain
  wire must cross; `bridgers_[chain][at]` (R0) names the plain wires per
  edge; `chainOfCoupler_` maps a coupler to (chain, waypoint).
- **A pass over a subset of members is unsafe as written**: `attempt`
  derives `before`/`after` from `members[(slot ± 1) % total]` and the second
  pair from `± 2` (`:8093-8095`, `:8134-8137`); the relaxation releases
  `members[(slot ± level) % total]` (`:8210`). The safe form is the full
  ring as `members` with only the flagged wires redrawn. Today that is
  impossible: `beginPass` clears `routed` on every member under a feedline
  pass, and the early accept at `:8108` (`isLegal` → "keeps its way, which
  holds") is dead because `forceReroute()` (`:221`) is on.
- **Legality without committing**: `isLegal` (`:8495-8521`) = lifted
  `conflictsOf` + `crossesAFeedline` + length; `conflictsIn(candidate, …)`
  judges a way the wire does not hold yet (as `refine` does, `:2552`).
  `conflictsIn` exempts plain wire vs feedline unconditionally (`:8717`), so
  "open" never means "too close to a feedline"; the feedline side is the
  fence during the search and the crossing mask (`CrossingConstraints`,
  `src/routing/CrossingConstraints.cpp:24-91`: straight runs carry their
  heading over ±10 cells, bends and the first/last 10 cells are
  `CURVE_ZONE`) in `crossesAFeedline`.
- **Orthogonal crossing today**: read at `crossesAFeedline` (`:2308`),
  `rebuildCrossingRule` (`:7596`; terminal edges left out under
  `feedlineLikePrototype()`, `:7589`), `search` (`:9062`, `routeOrthogonal`).
  `constrainByFeedlines` (`:7609-7669`) fences every drawn non-bridged,
  non-terminal edge regardless of the rule and prices every edge 5×. The DRC
  (`src/drc/Rules.cpp:278-331`, `checkOrthogonality`) builds its mask from
  **all** feedlines, terminals included — a divergence from the router that
  matters once the rule is the default.
- **Phase 4 order** (`:11145-11188`): `assignBridges`, `checkBridgers`,
  `ringWithEdges`, `sweep(members, feedlinePass)`, `sweep(inner, …)`,
  `repair(wires, members, everyWire(), feedlinePass)`, snapshot
  `"feedlines"`, `refine` under `runs(4)`. `stop_after = "feedlines"` runs
  the repair. `repair_trials` is 0 in all eight benchmark configs and is
  **not** read by `test/pipeline/Benchmarks.hpp` (tests run the schema
  default 100).
- **Feedline/feedline crossings between chains** exist on 69q (four pairs
  of terminal edges of neighbouring chains, `artifacts/logs/fcross.py`) and
  no check reports them; `waysCross` (`:7977`) is the test
  `checkCouplerCrossings` uses for edges meeting at a coupler.

## Design

### Vocabulary

- **Segment**: one chain `c`, waypoints `lo..hi` of `chains_[c]`
  (`lo < hi`), the waypoints `lo` and `hi` **fixed** on their current option
  (a launcher is fixed anyway), the interior `lo+1..hi-1` re-searched, the
  edges `lo..hi-1` re-routed. A segment with no interior (`hi = lo + 1`)
  re-routes one edge and re-searches nothing — allowed, it is the cheapest
  trial.
- **Window** `W(segment, k)`: the wires the local rip-up-and-reroute redraws —
  the plain wires bridged to a segment edge (`bridgers_[c][at]`,
  `lo ≤ at < hi`), the resonators of waypoints `lo..hi`, the terminal edge
  at a segment end that is a launcher, every wire named by a fail the
  segment was built from, then `k` more ring members beyond each end of
  that set's extent in `members` order (ring neighbours of order k).
- **Figure**: `bad(F) = F.unrouted + F.open + F.crossing` of a `Fails`
  (lengths excluded).

### 1. Triage — `Driver::blameFails`

Input: `failsOf(wires, every)` after the two sweeps. For each wire in
`unroutedIds`, `openIds`, `crossingIds`:

- **open**: `conflictsIn(wire.way, wire, wires, &blockers, &where)` with the
  wire lifted (private, `:8671`; the triage lives inside `Driver`) → every
  partner `p` and the own cell `x`. Blamed edges: the wire's `bridged` edge
  and each partner's; plus every feedline edge whose way comes within
  `couplerReach()` (≈ 62 cells on 45q) of `x` — a new `nearestEdgeTo(cell)`
  / `edgesNear(cell, reach)` factored out of `whereItCrosses`'s inner loop
  (`:2352-2369`, brute force over `edges_` × way, ~25k distances per query,
  negligible); plus the coupler whose pad or lead is within the clearance of
  `x` (`bodies_`, `couplers_[].options[chosen].arc`).
- **crossing**: `whereItCrosses` → the cell (it reports only under the rule,
  which is now the default); `edgesNear(cell, CROSSING_REACH + 1)` → that
  edge and its two couplers.
- **unrouted**: the wire's `bridged` edge and the edges of the two
  resonators that flank it in the ring (as `repair`'s `near` does,
  `:7807-7846`); `deadOnArrival` text for the log.
- **feedline/feedline**: a new `checkFeedlineCrossings` beside
  `checkCouplerCrossings` (`:8843-8872`, same shape: `waysCross` over every
  pair of drawn edges with `edges_[i].chain != edges_[j].chain`, prefiltered
  by `reachOf(a).meets(reachOf(b))` `:6963-6966`; 69q's 81 edges are 3240
  pairs, milliseconds) — called at `:3188` as a fourth guarantee line and
  read here: both edges blamed. Today such a pair surfaces, if at all, as
  "open" on both edges unnamed, because `conflictsIn` does bind
  feedline↔feedline.

Blamed edge `(c, at)` → waypoints `at`, `at+1` of chain `c` marked. Per
chain, maximal runs of marked waypoints form segments; a run is widened by
`SCPD_RESEARCH_GROW` (default 1) waypoints on each side so the fixed ends
have one free coupler between them and the fail; clamped to the chain. Two
runs that touch after widening merge. Each segment carries the fails it was
built from (wire keys) and the fail class; segments are ordered by fails
carried, then chain index. One `tell` per fail
(`fail 65 (open with 66 at (2806,1332)) blames f23 of chain 3 → segment chain 3
waypoints 2..5`), one `say` per chip: `targeted repair: n fails → m segments
(chains …), k fails with no edge within reach`.

Cross-chain fails (two terminal edges of different chains, or a plain wire
between chains with no bridged edge) yield one segment per chain, searched
one after the other; the second sees the first's result.

### 2. The re-search of one segment — `Driver::researchSegment`

Built as `solveChainAStar` is (`:6630-6934`), on the sub-chain:

- `open[lo] = {couplers_[..].chosen}` or `{0}` for a launcher; `open[hi]`
  likewise; interior `open[at] = openOptionsOf(c, at, &feedlines)` with
  `jogsUnlocked` set for the interior couplers when `SCPD_RESEARCH_JOGS` is
  on (default on: the second-dogleg options are exactly the "new/old
  options" the search should explore; the insertion opened them only at
  the wall).
- bounds by `portOfOption`/`boundTurns`, indices shifted by `lo`;
  `problem.budget = SCPD_RESEARCH_SECONDS` (default 20 s).
- `step`: `edgeCost(wires, c, lo + at, …, &fence, &way, /*remember=*/false)`
  — fresh, no memo (`edgeMemoKey` knows nothing of the fence, and the
  phase-3 entries are stale in phase 4). **What the step sees**, verified:
  `edgeCost`'s `stand()` (`:6184-6194`) sets `chosen` at the two ends, and
  `sourceOf/targetOf`, `runsOfEdge`, `portOfOption` and the chain's own
  leads/pads in `corridorOfEdge` (`:4475-4501`, `option.arc`/`option.body`
  of `options[chosen]`) all read `chosen` — the candidate's lead and pad are
  fenced. Three things are the **applied** state and need handling:
  (1) `bodies_` (`:4424`) holds every committed pad, the segment couplers'
  *old* pads included, as hard obstacles — clear the segment couplers'
  committed `option.body` cells out of `bodies_` for the search and put them
  back after (`Snapshot::bodies` is the copy); (2) `wayOfResonator`
  (`:4232-4240`) returns the applied, cut-back way, so the
  `RESONATOR_COPPER` tail stamp (`:4489`) runs along the old way — harmless
  at copper width, but the plan requires `SCPD_EDGE_LEAD_ONLY` on (the
  default); (3) interior couplers that are not this step's ends stand on
  their committed option, the same approximation the insertion lives with.
- **The fence.** `prefixFence_` gets only the segment's laid prefix ways
  (pointers into the `laid` map are stable; pointers into `wires[k].way`
  are not across `restore`). Committed edges come through
  `fenceCommittedEdges` (`:4148-4228`), and it fences **too much**: with
  `edgeWayOf` returning the drawn wire's way, the segment's own edges
  `[lo, hi)` would be fenced with their stale committed ways. Add an
  own-chain exclusion range (`openChain_`, `openLo_`, `openHi_` beside
  `looseChain_`, `:5609`) tested at `:4177`, set around the segment search.
  Set `fenceEverything_ = true` as well (`SCPD_RESEARCH_SEES_CHAINS`,
  default on): other chains' committed edges and this chain's terminal and
  later edges are then fenced regardless of `edgesSeeOtherChains`,
  `terminalEdgesFenceAll`, `fenceLaterEdges` — which is what keeps a
  re-searched terminal edge from crossing its neighbour chain again. Leave
  `soloChain_` and `looseChain_` at `NO_OWNER`; update the comment at
  `:5603-5605` ("set only around `stateFaults`"). The terminal exemptions of
  the fence builder (`:6761-6765`) are chain-relative: for a segment fence
  every laid way, no exemption. The end-pair reorder is **off** in the
  re-search (`pairAtAnEnd` would fire at segment ends, and `redone` breaks
  the cost order the enumeration relies on); say so in the comment.
- **The A\* runs on past its first answer.** `routing::ChainProblem` gets
  an optional `accept(chosen, cost) -> bool` and a `maxAccepts`;
  `ChainSolution` gains `rejected`. In `routing::solveChainAStar` the hook
  sits at the **real** pop of a last-layer node (`ChainSearch.cpp:206-216`):
  `prefixOf(top.node); if (accept && !accept(prefix, price)) { ++rejected;
  continue; }` before `solved = true; return`. Every open entry carries a
  lower bound on the true cost through it, so real last-layer pops arrive
  in non-decreasing cost and the next real pop after a rejection is the
  next-cheapest complete run (modulo `relaid`, which already costs the
  proof). Two holes the review found, both to fix: (1) `out.chosen/cost`
  are recorded at *pricing* time (`:192-196`) — with `accept` set they
  must not be, or `outOfTime` and queue-dry return a refused run; guard
  that block with `!problem.accept`, so `solved` means "an accepted run
  exists" and the budget fallback returns nothing rather than a refused
  run. (2) The clock is read only at the top of the loop (`:144`); an
  `accept` that runs a local sweep takes seconds, so test the budget again
  right after a rejection and bound the number of accepts with
  `maxAccepts`. `found` (`:197-199`) keeps firing at pricing. The readout of
  a run's ways (`laid.find(upto(at+3)).before` else `upto(at+2).way`,
  `:6909-6932`) is factored into `waysOfRun(laid, run)` and used by both the
  accept lambda and the final readout. Unit tests in
  `test/routing/test_chain_search.cpp`: a rejected cheapest run yields the
  next cheapest; accept-all equals today byte for byte; reject-all exhausts
  with `!solved && optimal && rejected == runs`; the budget and `maxAccepts`
  stop it; a run refused is never in `chosen`.
- `accept` is the legality test of §3. The run that passes is written as
  `optimizeChainsPrefix` writes (`chosen` per interior waypoint, the ways
  into `chainEdgePaths_[c][at]`) and left applied (§3 leaves it so).
- The **current** run is a complete run too and will usually be the
  cheapest; it fails the test by construction (its window has the fail), so
  the enumeration moves on. Log every rejected run: `segment chain 3 2..5
  run 6 0 9 (cost 60000): window 7 wires, 1 open (65/66), rejected`.

### 3. The legality test — local rip-up-and-reroute

Given a complete run for the segment:

1. `const auto taken = snapshot(wires);` and `const auto before = bad(failsOf(wires, every))`
   (computed once per segment, not per candidate).
2. **Apply**: for each interior waypoint, `applyOption(wires, coupler, option)`
   (as `turnCoupler` does, `:7718-7751`, but for several couplers before any
   edge is routed); for each segment edge: `lift` its wire, `way = the A*'s
   way` for that edge (`waysOfRun`), objective from `sourceOf`/`targetOf`,
   `fixed.clear()`, **`startStub`/`endStub` recomputed from `runsOfEdge(edge)`**
   (they are set once at creation, `:3092-3097`, and `turnCoupler` never
   recomputes them — a latent bug this inherits), `drawn = true`, `place`.
   `rebuildCrossingRule(wires)` (the edges moved).
3. **Window**, built geometrically, not from flags: the wires of
   `W(segment, k)` as defined, plus every member whose way lies within
   `tuning_.clearance` of a new edge way (`alongDisc` of the new ways
   against `field_.owner`) or crosses one (`waysCross`) — `isLegal`/`failsOf`
   cannot find those, because `conflictsIn` exempts plain wire vs feedline
   both ways (`:8715-8716`). Mark them `routed = false`, `ripped = true`;
   everything else keeps `routed = true`.
4. **Local pass**: `Pass local = pass; local.rounds = SCPD_RESEARCH_ROUNDS
   (default 2, never 1: a wire released late in a round would otherwise never
   be redrawn); local.name = "targeted repair"; local.onlyUnsettled = true;`
   then `sweep(wires, members, local)` over the **full ring** `members`. The
   new `Pass::onlyUnsettled` guards the one clearing loop in `beginPass`
   (`:7545-7547`); the sweep loop already skips `routed` members under
   `keepDrawn` (`:2118`), so only the window and whatever the relaxation
   releases (`:8213-8222`) are redrawn, while the ±1/±2 fences and
   `priceLane` see the true ring neighbours. The round tallies (`:2129-2149`)
   count attempted wires only and may end the pass early on `fails == 0`;
   the verdict comes from `failsOf`, not from them. Terminal edges in the
   window are redrawn by the sweep's `search` (as the pass does today), not
   by `routeEdge`. `verbosity_`/`progress_` are silenced during the trial as
   `repair` does (`:7909-7912`).
5. **Score**: `const auto after = failsOf(wires, every)`; the window is clean
   when no wire of `W` ∪ released is in `unroutedIds`/`openIds`/`crossingIds`
   (a key-level variant of `Fails` is needed; today it holds id strings);
   accept when the window is clean and `bad(after) < before`. Say the verdict
   with the figures.
6. **Reject** → `restore(wires, taken)` (whole `wires` vector, `bodies_`,
   `chosen`, `anchor`, the field rebuilt, the crossing rule); not
   `chainEdgePaths_`, `edgeMemo_`, `jogsUnlocked`, `foreignRoomStale_`
   (left true, forces a rebuild — fine). Any `Wire&`/`Path*` into `wires`
   held across it dangles. **Accept** → keep; `before = bad(after)`; write
   `chainEdgePaths_`.

Cost per test: the window (≈ 6–12 wires) × rounds (2) searches plus one
`failsOf(every)` (conflict counting, no search) plus a `restore` (field
rebuild) on reject — against today's repair trial, a whole-ring sweep.

### 4. Orchestration — `Driver::researchSegments(wires, members, every, pass)`

Replaces the `repair` call at `:11168` when `SCPD_REPAIR_SEARCH` is on
(default on; `=0` is today's repair). Loop over segments in order; a
segment whose fails were already cleared by an earlier accept is skipped
(re-run `failsOf` and re-triage after every accept, cheap). Budget:
`tuning_.repairTrials` legality tests per chip in all (`repair_trials`; the
eight benchmark configs move from 0 to the measured value), plus
`SCPD_RESEARCH_SECONDS` per segment search. After the loop, when
`SCPD_RESEARCH_FINAL_SWEEP` is on (default decided by the sweep, start off),
one full `sweep(members, pass)`, kept only if `bad()` does not rise
(`snapshot`/`restore` around it). One `==> targeted repair:` line: segments
built / searched / accepted, candidates tested / rejected, fails
before → after by class, seconds; plus the settings line.

### 5. The orthogonal rule as the default

- `orthoCrossing()` default `true` (`:829`), comment updated with the
  base-ortho table; the `feedline routing settings:` line already names it.
- Align the DRC with the router: `checkOrthogonality` (`src/drc/Rules.cpp:278-331`)
  builds its mask from every feedline wire, the router (`rebuildCrossingRule`
  `:7589`) leaves terminal edges out under `feedlineLikePrototype()`. The
  artifact's `feedline_edges` carries the ports of each edge, so a terminal
  edge is one whose port is a launcher: skip those in the DRC's mask, with a
  test in `test/drc/test_rules.cpp`. (The user may prefer the other
  direction — terminal edges in the rule everywhere; ask before changing the
  router, the DRC change is the conservative one.)
- `Final.FeedlinesAreDrawnAndCrossedAtRightAngles` (`test/pipeline/test_final_router.cpp:359`)
  then tests what the stage enforces, not what happens to hold.

### Switches (Driver statics beside `chainKeepWays`, every one in the settings line)

| switch | default | meaning |
|---|---|---|
| `SCPD_ORTHO_CROSSING` | **on** (was off) | the crossing rule in search, mask and count |
| `SCPD_REPAIR_SEARCH` | on | the targeted re-search replaces `repair`; `=0` is the old repair |
| `SCPD_RESEARCH_K` | 2 | ring neighbours of the window, each side; sweep 1 / 2 / 3 |
| `SCPD_RESEARCH_GROW` | 1 | free waypoints added at each end of a blamed run; sweep 0 / 1 / 2 |
| `SCPD_RESEARCH_SECONDS` | 20 | A\* budget per segment, whole enumeration |
| `SCPD_RESEARCH_ROUNDS` | 2 | rounds of the local pass |
| `SCPD_RESEARCH_JOGS` | on | second-dogleg options open for the segment's couplers |
| `SCPD_RESEARCH_SEES_CHAINS` | on | other chains' edges fenced during the segment search |
| `SCPD_RESEARCH_FINAL_SWEEP` | off, decided by the sweep | one full pass after all segments, kept if not worse |
| `repair_trials` (config) | 0 → measured value in the benchmarks | legality tests per chip |

## Implementation steps

Each step ends with a build of both trees, the install, and 4q / 9q / 17q
diffed against the base-ortho logs with the new switch off (byte-identical
`==>`, `CHECK`, `feedline routing`, `final routing` lines, `keylines.sh`).

1. **Baseline.** `SCPD_ORTHO_CROSSING` default on; DRC alignment (§5); rerun
   all eight into `artifacts/logs/base-ortho2` (the installed build now
   carries the room-rule report lines); confirm 28 / 55 / 22. Also one arm
   with the old `repair` at `repair_trials = 20` under the rule, as the
   figure the new repair is measured against (`artifacts/logs/repair-old`).
2. **`checkFeedlineCrossings`** in the insertion's checks (fourth CHECK line)
   and the `edgesNear` helper; confirm the four 69q pairs are named; 45q/57q
   green.
3. **`Pass::onlyUnsettled`** in `beginPass` and `sweep`'s loop; a test in
   `test/pipeline/test_final_router.cpp` style on 4q: flag one wire, sweep,
   only that wire's way changes.
4. **`accept` in `routing::solveChainAStar`** + `ChainSolution::rejected` +
   the four unit tests in `test_chain_search.cpp`.
5. **`blameFails`** with its `tell` lines, run on 45q / 57q / 69q under the
   rule: the segments must contain couplers 64 (45q), 8 (57q), 9 (69q) and
   the terminal pairs of the 69q crossings; report how many segments and
   how wide.
6. **`researchSegment`** (the sub-problem, the own-chain exclusion range in
   `fenceCommittedEdges`, `fenceEverything_`, the pads cleared out of
   `bodies_`, no reorder, no memo, `waysOfRun`) and the legality test
   (apply with recomputed stubs, geometric window, `onlyUnsettled` sweep,
   key-level fails, `restore`); first on 17q (4 open under the rule, 15 s a
   run) with `repair_trials = 8`.
7. **`researchSegments`** in phase 4 behind `SCPD_REPAIR_SEARCH`; the `==>`
   line and the settings line.
8. **The sweep**: k = 1 / 2 / 3; grow 0 / 1 / 2; `repair_trials` 8 / 20 /
   40; final sweep on / off; sees-chains on / off; each arm all eight chips,
   one at a time, into `artifacts/logs/<arm>/`; then one arm with the rule
   off for the record.
9. Defaults, switch comments with the figures, `handover-feedline-routing.md`
   (*Where it stands*, switches, what is left) and a new
   `handover-targeted-repair.md` (triage, the enumeration, the local pass,
   the traps); benchmark configs' `repair_trials`; `Benchmarks.hpp` reads
   `repair_trials` so the pipeline tests run what the CLI runs.
10. Tests and `ctest --test-dir build/release -E 'EveryChip/(Final|SpacedFinal)' -j 6`
    (four pre-existing failures, listed in `handover-feedline-routing.md`).

## Measurement protocol

- One chip: `cp benchmarks/<chip>/config.toml artifacts/<chip>/config.toml && <ENV> .venv/bin/mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage final -v 1 | tee artifacts/logs/<arm>/<chip>.log`;
  `artifacts/logs/run-arm.sh <arm> [ENV=…]` does the eight in order, with
  the Spotlight figure per chip and CPU time (`/usr/bin/time -p`). Check
  Spotlight before the large chips; it ran at 120 % during the 2026-10-04
  control arms without changing a result, but the seconds are only
  comparable at 0 %.
- Read per log: `==> coupler insertion`, the four `CHECK` lines,
  `targeted repair:` lines and `==> targeted repair`, the last
  `feedline routing round` and `N failed:` lines (of the pass and of the
  local passes, which are silenced — only the summary is printed),
  `final routing: … open … crossing`, `is open in the room of`, and the
  DRC's `feedline-orthogonality` findings in `plan`'s DRC summary.
- Acceptance per arm, all eight chips, one at a time, rule on: every edge
  drawn, the four CHECK lines green, `bad()` per chip not above base-ortho
  and the sum below it; 4q / 9q / 21q stay at 0; report seconds (the
  insertion's, the repair's, the stage's) and the lengths beside.
- A default is set only by a value that passes alone and together on all
  eight; ties by the sum of `bad()`, then by repair seconds.

## Risks and pitfalls

- **The window grows through the relaxation** (`:8217` releases ring
  neighbours of a redrawn wire); with `rounds = 2` that is bounded, but the
  score must count the released wires too, or a candidate wins by pushing
  its fail one wire further. `bad(after) < before` over `every` covers it.
- **`restore` rebuilds the whole field** (`:7700`), ~2 M cells on 69q, per
  rejected candidate. With 20–40 tests per chip that is seconds, not
  minutes; measure it in the `==>` line.
- **The current run is the cheapest and always rejected** — one wasted
  test per segment unless the enumeration skips the run equal to the
  current `chosen`; do skip it inside `accept` (compare the run to `chosen`,
  return false without a sweep, count it apart).
- **Edge stubs are stale after a turn**: `startStub`/`endStub` of an edge
  wire are set once (`:3092-3097`) and `turnCoupler` never recomputes them;
  recompute from `runsOfEdge` whenever an option changes (also fixes the old
  repair).
- **Hidden pads during the step**: forgetting to clear the segment
  couplers' committed pads out of `bodies_` makes every candidate edge
  route around pads that will not be there; the symptom is a cost that never
  beats the current run.
- **Monotonicity**: without the reorder, `f` is monotone and the complete
  runs pop in cost order; with `relaid` they may not. Keep the reorder off
  here and say so.
- **`jogsUnlocked` is monotone**: opening the jogs for a segment's couplers
  leaves them open for any later search of that chain in the same run; the
  insertion is over by then, so nothing else reads them — note it.
- **Option indices**: `couplers_[c].options` is never rebuilt, so `chosen`,
  `edgeMemoKey` and the run indices stay consistent; do not call `optionsOf`
  again (the resonator's `wire.way` is the cut-back way, `coupler.outerWay`
  the original).
- **`conflictsIn` exempts plain wire vs feedline**: a candidate that leaves
  a plain wire inside an edge's clearance is "legal" to `isLegal`; the
  fence keeps the search from drawing such a way, but a wire that was not
  redrawn keeps whatever it had. Include every wire whose way lies within
  the clearance of a segment's new edge ways in the window (an
  `alongDisc` over the new ways against `field_.owner`).
- **Terminal edges fence nobody** under `feedlineLikePrototype()` (`:7630`)
  and are not in the crossing mask (`:7589`): a plain wire may cross a
  terminal edge anywhere. A re-searched terminal edge is therefore judged
  only by `checkFeedlineCrossings` and the window's own fails.
- **Determinism**: the A\* budget makes a segment's answer machine-dependent
  (handover-chain-astar, trap 0); `SCPD_RESEARCH_SECONDS=0` for a run that
  must reproduce.
- **`stop_after`**: the targeted repair runs inside phase 4, so
  `stop_after = "feedlines"` includes it; `SCPD_REPAIR_SEARCH=0` with
  `repair_trials = 0` is today's behaviour exactly.

## Verification

- Unit: `test_chain_search.cpp` (accept/rejected), `test_final_router.cpp`
  (`onlyUnsettled` redraws one wire; `FeedlinesAreDrawnAndCrossedAtRightAngles`
  under the rule), `test_rules.cpp` (terminal edges out of the DRC mask).
- Identity: every new switch off → base-ortho byte-identical on the eight
  chips (`keylines.sh`).
- End to end: the sweep of §Measurement; 17q is the first chip to watch
  (4 open under the rule, 15 s a run); the 69q cross-chain crossings must be
  gone or named.
- `ctest` as in step 10.
