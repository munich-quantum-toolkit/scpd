# The targeted repair

Written for whoever takes the feedline stage further. Phase 4 of the Final
stage used to end on a blind repair: turn a coupler near a fail to one of
its two cheapest other options, sweep the whole ring again, keep the turn on
strictly fewer fails. That repair nominated no coupler for a wire that only
crossed a feedline, had never been run against the prefix search, and was
switched off in every benchmark (`repair_trials = 0`). This document is about
what replaced it on 2026-10-04: a **targeted re-search of coupler options**,
built on the plan the user approved that day
(`plan-coupler-repair-search.md`, the task `prompt-coupler-repair-search.md`).

Read [handover-feedline-routing.md](handover-feedline-routing.md) first for
the feedline pass this sits behind and the two ways of counting "open";
[handover-chain-astar.md](handover-chain-astar.md) for the prefix search the
re-search is a sub-problem of; and *The room rules* in
[handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md) for
why a trial that moves couplers is needed at all — no local geometric rule
at the coupler separates the open pairs from healthy couplers.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`, HEAD **`63851cf`** plus the uncommitted room-rule
  work of 2026-10-04 plus everything here, **all uncommitted** — the user
  commits per phase.
- The regime is the orthogonal crossing rule **on** (`SCPD_ORTHO_CROSSING`,
  default flipped 2026-10-04). The figure a chip is judged by is
  `bad = unrouted + open + crossing`, counted with every wire down at the end
  of the stage; lengths are not a criterion (user, 2026-10-04).
- Run one chip: `cp benchmarks/45q/config.toml artifacts/45q/config.toml &&
  .venv/bin/mqt-scpd plan -c benchmarks/45q/config.toml -o artifacts/45q
  --stage final -v 1`. The repair runs when `repair_trials > 0` in the run
  directory's `config.toml` — that copy is what `plan --stage final` reads,
  not the `-c` path.

## What it does

Three mechanisms, in the order they run after the two sweeps of phase 4
(`Driver::repairFeedlines` → `researchSegments`):

1. **The triage** (`Driver::blameFails`). Every fail of `failsOf(wires,
   every)` — open, crossing, unrouted — is traced to the chain edges and
   couplers that can be its cause, a blamed edge marks its two waypoints, a
   blamed coupler its one, and per chain the maximal runs of marked
   waypoints become **segments**: widened by `SCPD_RESEARCH_GROW` waypoints
   at each end so the fixed ends have one free coupler between them and the
   fail, clamped to the chain, merged where they touch. Ordered by fails
   carried, then by chain. One `tell` per fail says what it blames, one per
   segment says what it was built from, one `say` sums the chip up:
   `targeted repair: 8 fails → 4 segments (chains 7 3 6 0), 0 fails with no
   edge or coupler within reach`.
2. **The re-search of a segment** (`Driver::researchSegment`). The prefix
   search (`routing::solveChainAStar`) on the sub-chain `lo..hi`: both ends
   fixed on the option they stand on, the interior couplers open (their
   second-dogleg options too, `SCPD_RESEARCH_JOGS`), the rest of the chip
   frozen and fenced (`SCPD_RESEARCH_SEES_CHAINS`). The search runs on past
   its first answer: every complete run of options it reaches, in order of
   cost, is offered to `ChainProblem::accept`, and the first run that passes
   the legality test is the answer. Nothing is remembered and the end-pair
   reorder is off, so the runs really do pop cheapest first.
3. **The legality test** (`Driver::tryRun`). The interior couplers take the
   run's options, the segment's edges take the search's ways, the crossing
   rule is rebuilt, a **window** of ring wires is flagged, and one silenced
   `sweep` over the whole ring under `Pass::onlyUnsettled` redraws the
   window and nothing else. The verdict is `failsOf` over every wire: the
   run is **kept** when no wire the pass touched is unrouted, open or
   crossing (the window is clean) **and** `bad()` over the chip is strictly
   below what it was before the segment. Refused, the chip is put back with
   `restore`.

After an accept the fails are read again and the triage rebuilt on the new
state, so a segment whose fails were cleared is not searched and one whose
fails remain is searched against the chip as it now stands. The budget is
`repair_trials` legality tests per chip, in all. One `==>` line ends it:

```
==> targeted repair: 3 segments built, 1 searched, 0 accepted (0 re-triages); 8 legality tests of 8 (8 refused, 0 the current run), 47 edge searches; unrouted 0 -> 0, open 4 -> 4, crossing 4 -> 4 (bad 8 -> 8), 5.7s
```

### What a fail blames

- **Open** (`conflictsIn` with the wire lifted names every partner and the
  own cell of each conflict): the wire's bridged edge and each partner's,
  the edge itself where either is a feedline, every edge whose way comes
  within `couplerReach()` (≈ 62 cells on 45q) of the conflict cell
  (`edgesNear`), and every coupler whose lead or pad lies within the
  clearance of it. The lane pairs of 45q and 57q lean on a pad; 69q's
  resonator 9 lies one cell from its own lead.
- **Crossing** (`crossingCellOf`, the first cell that breaks the rule outside
  the wire's own coupler): every edge within `CROSSING_REACH·√2 + 1` of the
  cell. The halo is a square, so its corner is 14 cells away in the
  distance `edgesNear` measures; at a reach of 11, 17q's wire 2 blamed
  nothing.
- **Unrouted**: an edge blames itself, a resonator its coupler, a plain wire
  its bridged edge and the couplers of the two resonators that flank it in
  the ring.
- **Two edges of different chains that cross** (`feedlineCrossingPairs`,
  the fourth `CHECK` line): both edges, each as a fail of its own wire.
- **Every fail also blames the edges within the clearance of its search
  start** — the source moved along its heading by the stub, where the router
  begins. A wire whose first cells are closed is *dead on arrival* in every
  round whatever else the fail says: 17q's wire 31 crosses f9 and is blamed
  on f9, but it finds no way because f8 lies 19 cells from its start, in
  every candidate of every segment that leaves f8 where it is.

A fail that blames nothing is counted and said (`3 fails with no edge or
coupler within reach` on 57q: the lane pair 161/162, far from every edge).
The targeted repair cannot reach it by design — nothing at a coupler causes
it.

### The window

`W(segment, k)`: the plain wires bridged to a segment edge (`bridgers_`, R0
of the room rules), the resonators of the segment's waypoints, a terminal
edge among its edges, every wire the segment's fails named, every ring
member whose way lies within the clearance of a new edge way or crosses one
(`alongDisc` over the new ways against `field_.owner`, `waysCross`) — which
`isLegal` could never see, because `conflictsIn` exempts plain wire against
feedline both ways — and then `SCPD_RESEARCH_K` ring members on either side
of each of those. Flagged `routed = false, ripped = true`; **every other
member is held `routed = true`**, the fails outside the window included, so
the window is what the pass redraws and nothing else. The sweep is over the
**whole ring**, because `attempt` reads its neighbours off `members[slot ±
1]` and the relaxation releases `members[slot ± level]`: a pass over a
subset of members would fence against the wrong wires. The verdict counts
the window and whatever the relaxation released and redrew (the members
whose way changed), so a candidate cannot win by pushing its fail one wire
further.

### What the search sees, and what it does not

- `edgeCost` stands the two ends of a step on the options asked about, and
  `sourceOf`/`targetOf`, `runsOfEdge`, `portOfOption` and the chain's own
  leads and pads in `corridorOfEdge` all read `chosen`: the candidate's lead
  and pad are fenced.
- `prefixFence_` holds the segment's own laid ways, every one of them.
  `fenceCommittedEdges` holds the rest of the chip: the segment's own edges
  are left open through `openChain_`/`openLo_`/`openHi_` (they are the ones
  being laid again, and `edgeWayOf` would fence them with the stale ways
  their wires still hold), and under `SCPD_RESEARCH_SEES_CHAINS`
  `fenceEverything_` fences every other chain's edge and this chain's
  terminal and later edges whatever the insertion's own switches say.
- Three things are the **applied** state during the search: the interior
  couplers' committed pads are hidden out of `bodies_` (`setPads`) and put
  back afterwards; the resonator tail stamped along `wayOfResonator` is the
  applied, cut-back way (harmless at copper width); an interior coupler that
  is not this step's end stands on its committed option — the approximation
  the insertion lives with too.
- **The current run** is enumerated like any other and refused without a
  test when its edges come out on the ways they already hold; with other
  ways it is a candidate — the same couplers with the edges laid against the
  frozen chip, which is what a terminal pair that crosses its neighbour
  chain needs.

## The search and the count disagree at the halo's edge

Found on 17q while reading why no candidate's window was ever clean: wire
31, redrawn in a candidate's local pass (`relax 1 … found 342 cells`), is
counted `crossing at (303,213), 10 cells from edge f9` at the end of the
same pass. The search returned a way the count refuses. Over the eight chips
of `base-ortho2`, **12 of the 22 crossing wires found a way in their last
search and are crossing by the count alone**, every one of them "10 cells
from edge": 17q 33; 33q 72; 45q 13, 132; 57q 15, 51, 77, 134, 156; 69q 167,
182, 186. The other ten kept a way no search drew (dead on arrival, or no
way in any round).

The mechanism, read off 33q's wire 72 in the artifact: its way runs
`(1288,706,4) (1287,706,4) (1287,707,4) (1287,706,3) (1286,707,3)` — the
cell (1287,706) twice, first as a cell swept by a move entered heading north
(4), then as the **state cell** of the next move, recorded with the heading
the wire leaves it on (3, diagonal). f24 runs west (2) ten cells away, so
the cell is the halo's corner; heading 4 is orthogonal to 2 and allowed,
heading 3 is not. `DubinsRouter::expandOrthogonal` tests every cell a move
sweeps, the end cell included, **with the heading the move entered on**
(`cellOk` uses `current.heading`); `reconstruct` records the end cell with
the exit heading; `crossingCellOf` and the DRC read that. A wire that
crosses a feedline straight and turns on the last cell of the halo passes
the search and fails the check.

`SCPD_CROSSING_EXIT_HEADING` (off) makes the search test the end cell with
the exit heading too (`DubinsRouter::setOrthogonalExitCheck`): the search
then refuses what the count refuses. It changes the regime's figures — a
wire that used to find a way that crosses now finds another or none — so it
is a measured arm and not a default — see *The exit heading, measured* in
*Where it stands*: `bad` 77 → 61 over the eight chips by the regime alone,
no chip worse, and 17q's repair from nothing accepted to every segment
accepted.

## The switches

Every one is read through `envFlag`/`envWhole`/`envReal`, defaults to what
was measured last, and appears in the `targeted repair settings:` line that
`repairFeedlines` prints before the repair, with where each value came from.

| switch | default | |
|---|---|---|
| `SCPD_ORTHO_CROSSING` | **on** (was off) | the crossing rule in search, mask and count; the regime since 2026-10-04 |
| `SCPD_CROSSING_EXIT_HEADING` | off | the orthogonal search tests a move's end cell with its exit heading, as the count reads it; see *The search and the count disagree* |
| `SCPD_BRIDGE_CHECK` | on | a way found that does not cross the wire's bridged edge is refused as no way, in the pass and in the repair's local passes alike; see *The bridge check* in handover-feedline-routing.md |
| `SCPD_RESEARCH_VERBOSE` | off | the local passes of the legality tests speak: every `wire … · round …` line of every candidate's window |
| `SCPD_REPAIR_SEARCH` | on | the targeted re-search; `=0` is the blind `repair` of before |
| `SCPD_RESEARCH_K` | 2 | ring members redrawn beyond each window member; sweep 1 / 2 / 3 |
| `SCPD_RESEARCH_GROW` | 1 | free waypoints added at each end of a blamed run; sweep 0 / 1 / 2 |
| `SCPD_RESEARCH_SECONDS` | 20 | the clock of one segment's search, legality tests included; 0 is no limit |
| `SCPD_RESEARCH_ROUNDS` | 2 | rounds of the local pass; never 1, or a wire released late is never redrawn |
| `SCPD_RESEARCH_JOGS` | on | second-dogleg options open for the segment's couplers |
| `SCPD_RESEARCH_SEES_CHAINS` | on | the rest of the chip fenced during the segment search |
| `SCPD_RESEARCH_FINAL_SWEEP` | off | one full pass after the segments, kept if `bad()` does not rise |
| `repair_trials` (config) | 0 in the benchmarks | legality tests per chip — the budget |
| `SCPD_PROBE_ONLY_UNSETTLED` | unset | a test seam: flag one ring wire, sweep under `onlyUnsettled`, say what moved, put everything back |

With `SCPD_REPAIR_SEARCH=0` the stage reproduces `artifacts/logs/base-ortho2`
byte for byte in every `==>`, `CHECK`, `feedline routing` and `final
routing` line (`artifacts/logs/keylines.sh`; verified on 4q / 9q / 17q,
2026-10-04). With it on and `repair_trials = 0` it reproduces them too —
the triage reports and decides nothing.

## Where it stands

### 17q, the first chip (2026-10-04)

17q has 4 open and 4 crossing wires under the rule (`bad` 8): the lane pair
18/19 at chain 0, the lane pair 32/33 and the dead-on-arrival 31 at chain 1,
and wire 2 crossing f17. The triage builds three segments — chain 1
waypoints 1..5 (interior couplers '26' '30' '34'), chain 0 waypoints 3..6
('18' '20'), chain 3 waypoints 0..3 ('55' '1') — with 0 fails out of
reach. Every run below is `artifacts/logs/<arm>/17q.log`, `-v 1`, the
default switches unless named:

| arm | exit heading | trials | accepted | bad | open | crossing | repair |
|---|---|---|---|---|---|---|---|
| `base-ortho2` | off | 0 | — | 8 | 4 | 4 | — |
| `research-17q` | off | 8 | 0 of 1 searched | 8 | 4 | 4 | 5.7 s |
| `r17-t40` | off | 40 | 0 of 1 | 8 | 4 | 4 | 42 s |
| `r17-t40-g2` (grow 2) | off | 40 | 0 of 1 | 8 | 4 | 4 | 41 s |
| `r17-t40-k3` (k 3) | off | 40 | 0 of 1 | 8 | 4 | 4 | 32 s |
| `exit-17q` | **on** | 0 | — | 6 | 4 | 2 | — |
| `exit-r17-t40` | **on** | 40 (14 used) | **3 of 3** | **0** | **0** | **0** | 9.3 s |

Without the exit heading tested, every candidate's window ends with wire 31
or 32 crossing f9 "10 cells from edge": the local pass redraws them and the
count refuses what the search returned, so no window is ever clean, though
candidates `0 0 0 40` and `0 0 12 31` of chain 1 take `bad` from 8 to 4.
With it, the regime alone takes 17q from 8 to 6 (32 and 33 stay open but no
longer "cross"), and the repair closes the rest: chain 1 on options
`0 0 12 31` after 12 tests, chain 0 on `0 12 30` and chain 3 on `39 1 0 27`
after one test each, 14 legality tests in 9.3 s. Lengths are not the figure
and are not read (user, 2026-10-04).

### The eight chips (2026-10-04)

Every arm all eight chips, one at a time, on the installed build, `-v 1`,
the defaults unless named; `bad` per chip with `(open/crossing)` beside it
(unrouted is 0 everywhere), the sum, and the wall seconds of the whole
stage over the eight chips. Every edge of every chip is drawn in every arm;
`CHECK feedline room`, `coupler crossings` and `resonator crossings` are
green everywhere, `CHECK feedline crossings` is red on 69q at the end of the
**insertion** in every arm (the four terminal pairs) and green after the
pass, where the repair reads it.

| arm | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q | sum | seconds |
|---|---|---|---|---|---|---|---|---|---|---|
| `base-ortho2` — rule on, no repair | 0 | 0 | 8 (4/4) | 0 | 5 (4/1) | 8 (6/2) | 25 (17/8) | 31 (24/7) | **77** | 2287 |
| `repair-old` — the blind repair, 20 trials | 0 | 0 | 8 (4/4) | 0 | 5 (4/1) | 8 (6/2) | 23 (18/5) | 29 (22/7) | **73** | 3445 |
| `research` — k 2, grow 1, 20 trials | 0 | 0 | 8 (4/4) | 0 | 4 (4/0) | 7 (6/1) | 24 (17/7) | 26 (21/5) | **69** | 2568 |
| `exit-base` — exit heading, no repair | 0 | 0 | 6 (4/2) | 0 | 4 (4/0) | 6 (6/0) | 19 (17/2) | 26 (23/3) | **61** | 2186 |
| `research-exit` — exit heading, 20 trials | 0 | 0 | **0** | 0 | 4 (4/0) | 6 (6/0) | 19 (17/2) | 22 (20/2) | **51** | 2514 |

No arm is above the baseline on any chip; 4q, 9q and 21q stay at 0 in every
arm. The blind repair's 73 is bought with lengths (it keys on every fail,
lengths included) and 1160 s more. What the repair did per chip, with the
seconds of the repair and of the insertion beside it:

| arm | chip | segments built / searched / accepted | tests used | repair s | insertion s |
|---|---|---|---|---|---|
| `research` | 17q | 3 / 1 / 0 | 20 of 20 | 14 | 4.7 |
| `research` | 33q | 2 / 3 / 1 | 7 of 20 | 57 | 9.3 |
| `research` | 45q | 4 / 7 / 1 | 20 of 20 | 129 | 38.8 |
| `research` | 57q | 6 / 8 / 1 | 20 of 20 | 138 | 61.1 |
| `research` | 69q | 5 / 8 / 1 | 6 of 20 | 147 | 116.7 |
| `research-exit` | 17q | 3 / 3 / 3 | 14 of 20 | 9 | 4.7 |
| `research-exit` | 33q | 1 / 1 / 0 | 2 of 20 | 20 | 9.8 |
| `research-exit` | 45q | 3 / 3 / 0 | 11 of 20 | 77 | 36.1 |
| `research-exit` | 57q | 3 / 3 / 0 | 8 of 20 | 67 | 61.0 |
| `research-exit` | 69q | 5 / 8 / 1 | 5 of 20 | 144 | 117.4 |

What the two arms with the repair say:

- **Where a crossing was the fail, the repair reaches it.** Without the
  exit heading it closed 33q's 72, one of 45q's two, one of 57q's eight and
  two of 69q's seven, each by one accepted segment — and 69q's accept took
  three open wires with it (24 → 21). With the exit heading on, the regime
  alone turns most of the count-only crossings into nothing (22 → 7 over
  the eight chips), and 17q's three segments all pass.
- **The lane pairs of plain wires stay.** 33q's 9–12, 45q's 65/66, 116/117
  and 133/136, 57q's eight pairs and 69q's twenty open wires are what is
  left in every arm with the repair: candidates for their segments were
  refused with the pair still open in the window (the local pass redraws the
  two and they trade the lane again), or the segment ran out of its 20 s
  before a candidate was tested (69q's chains 0 and 9, six and five edges,
  reached no complete run in 20 s; 33q's chain 0 tested three). On 69q and
  33q the clock, not the trials, was the budget — 6 and 7 of 20 tests used.
- **The two fails the triage cannot reach** (57q 161/162, 69q 175/176) blame
  nothing and are in no segment.
- **After an accept, a segment searched before without one and unchanged
  since is skipped** (added after the arms; `artifacts/logs/final-69q-exit`):
  69q's repair falls from 144 s to 88 s at the same result, and the
  post-repair `feedline routing: CHECK feedline crossings` is green — the
  four pairs of the insertion are gone once the pass has redrawn the
  terminal edges.

### The exit heading, measured

`exit-base` against `base-ortho2` is the regime change alone: crossing 22
→ 7 over the eight chips (17q 4 → 2, 33q 1 → 0, 45q 2 → 0, 57q 8 → 2, 69q
7 → 3), open 55 → 54 (69q 24 → 23), `bad` 77 → 61, and the stage 100 s
faster. Twelve count-only crossings were the prediction; fifteen crossings
went, because a search that may not turn on the halo's last cell also lays
the wire elsewhere. No chip is worse. It is **off by default** until the
user decides; everything the repair reaches on 17q it reaches only under
it.

## Reading the log

At `-v 1`, in order:

- `coupler insertion: CHECK feedline crossings — N pairs of edges of
  different chains cross: f32 (chain 4) x f33 (chain 5) · …` — the fourth
  guarantee line, red on 69q with the four pairs `fcross.py` found.
- `targeted repair settings: re-search yes (default), k 2 (default), …` —
  every switch and where it came from.
- `targeted repair: fail 65 (open with 66 at (2856,1286)) blames f23 (chain
  3 edge 3)` — one per fail; `blames nothing within reach` is the fail the
  repair cannot see.
- `targeted repair: segment chain 3 waypoints 2..5 (3 edges, interior
  couplers '64' '67') from fails 65 (…), 66 (…)` — couplers are named by
  their resonator in quotes, as the `room` lines name them, never by index.
- `targeted repair: 8 fails → 4 segments (chains 7 3 6 0), 0 fails with no
  edge or coupler within reach` — the chip's triage in one line.
- `targeted repair:   chain 3 … : every edge drawn on options 0 0 29
  launcher -> cost 60000` — a complete run reached (the search's `found`).
- `targeted repair:   chain 3 … run 0 0 29 launcher (cost 60000): window 14
  wires, 7 moved, no way for 32, 31 in the last round; in it 0 unrouted, 2
  open, 3 crossing (…); bad 8 -> 8 (…): rejected` — one legality test. `no
  way for` names the window wires the local pass could not redraw: they kept
  what they had, and that is why they still fail.
- `targeted repair: chain 3 … — ACCEPTED options … at cost …` or `nothing
  accepted, out of legality tests` / `out of time` / `every run refused` /
  `no run of options joins it` — one per segment searched.
- `==> targeted repair: …` — the chip.

`artifacts/logs/repair-summary.py <log>…` prints exactly these lines plus
`bad` of the final line.

## Tools

- `artifacts/logs/run-arm.sh <arm> [ENV=VAL …]` — the eight chips, one at a
  time, on the **installed** binding; `REPAIR_TRIALS=N` sets
  `repair_trials` in the run directory's copy of the config, `CHIPS="45q
  57q"` restricts the chips.
- `artifacts/logs/dev-sync.sh` and `dev-run.sh <arm> <chip> [ENV=VAL …]` —
  the same chip on the binding the **build tree** holds, through an overlay
  package under `artifacts/dev/py` (`OVERLAY=py2` for a second one) and a run
  directory of its own under `artifacts/dev/<chip>`. The venv is an editable
  install whose scikit-build finder maps `mqt.scpd.pyscpd` to the installed
  binding whatever `PYTHONPATH` says; `dev-mqt-scpd.py` takes that finder
  out and puts the overlay first. This is how a build can be tried while an
  arm runs on the installed binding — **never install into `.venv` while an
  arm is running**, a later chip of the arm would run another build.
- `artifacts/logs/repair-summary.py`, `keylines.sh`, `calibration.py`,
  `fcross.py`, `pinch.py` as before.

## The traps

- **The venv is an editable install.** `PYTHONPATH` does not override the
  binding; see *Tools*. A dev run that silently used the installed binding
  looks exactly like a run of the new build with no effect.
- **Couplers have two names.** The `room` lines, `sayCoupler` and this
  repair name a coupler by its resonator (`coupler '64'`); `couplers_`
  indexes it (20 on 45q). The first version of the triage printed indices
  and read as though coupler 64 was not in its segment.
- **The crossing halo is a square.** A crossing cell lies up to
  `CROSSING_REACH·√2` from the edge's straight run in Euclidean distance;
  a blame reach of `CROSSING_REACH + 1` misses the corners.
- **A wire that keeps failing in every candidate is dead on arrival**, not
  short of room: read the `no way for …` names in the verdict line, then the
  `dead on arrival` line of that wire in the base pass. The edge at its start
  is the one to move, and if that edge leaves a *fixed* end of the segment
  (17q's f8 leaves waypoint 2, the segment's `lo`), no run of the segment can
  move it — `SCPD_RESEARCH_GROW=2` or the start blame widening the segment
  can.
- **`restore` replaces the whole wire list.** Any `Wire&` or `Path*` into
  `wires` held across a refused candidate dangles; the lambdas in
  `researchSegment` hold the vector, never an element.
- **`maxAccepts` counts refusals, the current run included.** The search is
  given `trialsLeft + 1` so that the one untested refusal of the current run
  does not eat a test; the lambda itself stops testing at `trialsLeft`.
- **`snapshot` after `setPads`.** The snapshot a refused candidate is put
  back to has the interior pads hidden, which is the search's own ground;
  the pads come back once, when the segment is done.
- **The wall-clock budgets** (`SCPD_RESEARCH_SECONDS`, and the insertion's
  `SCPD_CHAIN_ASTAR_SECONDS`) make a run machine-dependent. Spotlight sat at
  100–120 % during most of the 2026-10-04 arms (`artifacts/` is rewritten
  by every run); the seconds columns are indicative only.

## What is not done

- **Three decisions are the user's** (2026-10-04): whether
  `SCPD_CROSSING_EXIT_HEADING` becomes the default — it changes the regime's
  figures, and without it the targeted repair accepts nothing on 17q; the
  criterion — a window clean of unrouted, open and crossing wires, where a
  relaxed one ("no new fail in the window, `bad` strictly lower") would take
  17q's chain 1 from 8 to 4 even under the inconsistent rule; and the sweep
  of the plan's step 8 (k 1 / 2 / 3, grow 0 / 1 / 2, `repair_trials` 8 / 20
  / 40, final sweep, sees-chains), of which only the arms in *Where it
  stands* were run. `repair_trials` stays at 0 in the eight benchmark
  configs until the sweep names its value.
- **Fails the triage cannot reach**: a lane pair of plain wires with no edge
  within `couplerReach` and no coupler within the clearance — 57q 161/162,
  69q 175/176 — blames nothing and lies in no segment. Nothing at a coupler
  causes them; the one-sided relaxation of the pass is their mechanism
  (handover-feedline-routing, *What is not done*).
- **The stubs are outside the rule.** `assemble` prepends and appends the
  straight stubs unchecked; a port whose run lies inside a halo at a
  non-right angle is a crossing no search can refuse and no repair can
  mend. None of the 22 crossing wires of the baseline is one.
- **The work counts of the budget-limited chains** (45q chain 6, 57q chains
  0, 2 and 4, 69q chain 8) differ from run to run and from the load on the
  machine, as handover-chain-astar's trap 0 says; every arm of 2026-10-04 ran
  with Spotlight at 100–140 % and, for part of the day, two runs side by
  side. The figures held; the seconds are indicative.
- **The `EveryChip/Final` and `SpacedFinal` suites** were not run (hours);
  `Benchmarks.hpp` now reads `repair_trials`, so they would run the stage as
  the CLI runs it. The quick suite (`-E 'EveryChip/(Final|SpacedFinal)'`)
  passed 393 of 396 on 2026-10-04 with the three pre-existing Detail-stage
  failures (`BenchmarkDetail.DrawsCopperTheRulesAllow/9q`, `/57q`,
  `SpacedDetail.KeepTheWireSpacing/9q`);
  `FinalRouter.StartsOnTheDetailWaysAndReportsItsFails` passes again now
  that the tests run the benchmark's `repair_trials`. New tests:
  `ChainSearch.ARefusedCheapestRunYieldsTheNextCheapest`,
  `AcceptingEveryRunChangesNothing`, `RefusingEveryRunExhaustsTheChain`,
  `TheTrialBoundStopsTheRefusals`, `TheClockIsReadAfterARefusal`;
  `FinalRouter.OnlyUnsettledRedrawsOneWire`;
  `FeedlineOrthogonality.ATerminalEdgeIsNotInTheRule`;
  `DubinsRouter.TheExitHeadingCheckMakesTheSearchAgreeWithTheCount`.
- **The DRC alignment took the conservative direction**: terminal edges out
  of the DRC's crossing mask, as the router has them. The other direction —
  terminal edges in the rule everywhere, router and DRC alike — was not
  measured.
