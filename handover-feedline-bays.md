# Bays: the feedline routing settled a stretch of the ring at a time

Written for whoever builds this. It replaces the feedline routing's whole-ring
sweep and its trial repair with a procedure that cuts the ring into **bays**,
settles each on its own, and then swaps coupler options where a bay fails —
systematically, in cost order, from data the coupler insertion already
produces.

Read [handover-final-routing.md](handover-final-routing.md) for the feedline
phase as it stands and [handover-chain-astar.md](handover-chain-astar.md) for
the coupler-option search this leans on. This document assumes both.

**Nothing here is built yet. Step 1 is a measurement, and it may tell you not
to build the rest.**

- Reference checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch `phase-4-routing-stages`, HEAD **`435ca78` Coupler insertion done now
  feedline routing**.
- **The tree is not clean.** `src/pipeline/FinalRouter.cpp` carries about 540
  uncommitted lines — the `feedlineLikePrototype` switch and what hangs off it
  — and all eight benchmark configs carry an uncommitted `corridor_spacings`
  per chip, `stop_after = "feedlines"`, and on 17q
  `resonator_length_tolerance = 200.0`. **Every line number below is against
  that working tree, not against `435ca78`.** Check `git diff` before trusting
  one.
- The user commits per phase. Leave the work in the tree and say what you
  verified.

### Work in your own checkout. Do not change this one.

`scpd-phase-4` is the reference and must stay as it is found. Earlier phases
each got a checkout of their own — `scpd-phase-1`, `scpd-phase-2`,
`scpd-phase-3` — and this is the same arrangement.

**Copy the tree. Do not `git worktree add`.** A worktree checks out `435ca78`
and would drop the 540 uncommitted lines that every line number in this
document refers to.

```bash
cd ~/Documents/GitHub
cp -a scpd-phase-4 scpd-feedline-bays          # uncommitted work included
cd scpd-feedline-bays
rm -rf artifacts/*/debug                        # ~16 GB of old debug pictures
git status --short                              # must match what is listed above
cmake --preset release && cmake --build --preset release
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .
```

Two things the copy needs afterwards:

- **Reinstall the Python package** as shown, or `.venv/bin/mqt-scpd` still runs
  the extension built in `scpd-phase-4` and every measurement you take is of
  the wrong tree. This is the one mistake that wastes a whole day.
- **`artifacts/` comes with the copy**, which is what lets `plan --stage final`
  resume from `05-detail.fb` instead of re-running the earlier stages. Keep it,
  but drop the `debug/` folders: they are about 16 GB, they are pictures of old
  searches, and Spotlight reindexing them has cost a measurement a factor of
  six before.

Report your results against `scpd-phase-4`'s numbers, and hand the work back as
a diff or a branch in your own checkout.

## Why

Phase 3 now settles the coupler options with an exact prefix A\*
(`Driver::optimizeChainsPrefix`, `solveChainAStar`) and draws **every** feedline
edge on all eight chips. Phase 4 then redraws the ring under the feedline
constraints and mends what fails with `Driver::repair` (`:5964`) — a trial loop
that turns one coupler at a time and keeps a turn only when it leaves strictly
fewer fails.

Two things are wrong with that loop.

**It is uninformed.** It cannot say which coupler caused a failure, so it
guesses. It orders its guesses by `CouplerOption::cost`, which the greedy writes
(`:4325`) and the A\* never does — so on the current default path that sort runs
on a constant and the order is `optionsOf`'s order and nothing else.

**It is blind to the failures that now matter.** Its predicate at `:6019` is
`!drawn || !routed || tooShort || tooLong`. A wire that crosses a feedline at the
wrong angle nominates **no candidate coupler at all**, because `failsOf`
computes `open` and `crossing` as locals (`:1577`, `:1588`) and never stores
them on the wire.

A bay makes the failure attributable **by construction**: settle one stretch of
the ring with everything else standing, and whatever fails, failed there. No
blame grid, no conflict minimisation.

## What was decided

With the user, 2026-10-01:

- The procedure **may redraw the inner coupler-to-coupler chain edges**. Today
  only `turnCoupler` redraws them; `ringWithEdges` (`:5662`) keeps them out of
  the sweep entirely.
- **Length failures are out of scope.** `tooShort` and `tooLong` are reported,
  never counted, never a reason to act, never part of a score. Phase 4 cannot
  change a length anyway — `feedlineMeander()` (`:231`) is off by default — so
  an off-target resonator belongs to another pass.
- **Inner circuit wires are never touched**, and phase 4's second sweep over
  them (`:8683`) stays exactly as it is.
- **Step 1 gates the rest.**

### What counts as a failure

Four things: **F1** a wire has no way, **F2** it crosses a feedline other than
at a right angle, **F3** it lies in the room around a feedline's bend, and
**open** it breaks the clearance rule against another wire.

`Fails::total()` (`:1552`) returns `failing`, which counts a wire once for *any*
fault, **length included** — so it is the wrong score here. Add a sibling
accessor that counts a wire only when it fails on something other than
`tooShort`/`tooLong`, and judge every candidate by that. Keep printing the
length columns; they decide nothing.

### Inner wires

`ringWithEveryEdge` (step 3) is built from `outer`, so an inner wire is **not a
member** and no mask can select one. Nothing in the new code attempts them. They
stand as obstacles, like the artwork — which is what makes a bay sweep honest
rather than optimistic.

**But inner fails do not score a candidate.** Today `repair` measures itself on
`everyWire()` (`:8691`), so a fail in the inner circuit can veto a coupler turn
the ring needed. Score on what the procedure can affect — the ring members and
the chain edges — with one guard: reject a candidate that *raises* the inner
fail count. Then inner wires can never block an improvement and never be
damaged by one.

### The word

`segment` is taken: `src/pipeline/Assigner.cpp` uses it for the ring arc between
two launcher slots. `zone` is taken too: `CrossingConstraints` has curve, pin
and straight zones. **`Bay`** is free in `src/` and `include/`. One term, one
meaning.

## Step 1 — Measure. No code.

All eight configs already carry `stop_after = "feedlines"` and
`repair_trials = 0`, so **phase 4 runs and the repair does not**. Phase 4 has
never been run against the A\* feedlines: every log kept so far predates that
config change and holds no `feedline routing` line at all. This is the cleanest
baseline there will ever be, and it costs one loop.

```bash
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  cp benchmarks/$c/config.toml artifacts/$c-dp/config.toml
  /usr/bin/time -l .venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml \
      -o artifacts/$c-dp --stage final -v 1 > /tmp/feedlines-$c.log 2>&1
done
```

Sequentially, and watch what else the machine is doing — `artifacts/` is about
17 GB and Spotlight reindexing it has cost a measurement a factor of six
before. Measure CPU time beside wall time; the two agreeing is what says a row
is clean.

`-v 1` already yields everything, from code that exists:

| line | source | what it gives |
| --- | --- | --- |
| `feedline routing: … \| Fails: N` | `sayFails` `:1773` | the per-type counts |
| the per-type id lists | `:1789` | the whole inventory by name |
| `<id> is open in the room of <ids>` | `whoBlocks` `:6771` | the blockers, by name |
| `<id> crosses at (x,y), K cells from edge <id>…` | `whereItCrosses` `:1659` | the offending cell **and the nearest edge** |
| `the ring the feedline pass sweeps: …` | `beginPass` `:5733` | the ring order, so the bay structure is derivable from the log alone |

### The rule that decides whether to build anything

Let `F` be the wires failing on **F1, F2, F3 or open** — length excluded.

**Stop. The machinery is unnecessary if:**

- **`F ≤ 2` on every chip.** The A\* feedlines are simply routable. Switch
  phase 5 back on, run the DRC, and finish.
- **Feedline-edge `open` dominates on 69q.** Its 22 `stateFaults` are
  edge-against-edge overlaps that phase 3 leaves behind under
  `SCPD_CHAIN_KEEP_WAYS`, and `conflictsIn` (`:6707`) does bind edge against
  edge. One run at `SCPD_CHAIN_KEEP_WAYS=0` settles it. Redrawing inner edges
  is allowed here, so this is mendable — but if it is *all* of `F`, the cheaper
  fix is in phase 3 and this plan is the wrong tool.
- **The implicated couplers have no untried options.** Step 2 prints
  `chosen/options` to check exactly this. With nothing to enumerate, step 7 is
  dead on arrival.

**Build if** `F ≥ 10` on at least two chips, the failures clump — more than one
per implicated edge — and the implicated couplers carry unused options.

Report the short and long columns in the same table for information. They will
not be zero, and they are not a reason to build or not to build.

## Step 2 — The instrument. About thirty lines.

- **`Wire` (`:845`), beside `tooLong` (`:918`)**: add `bool open` and
  `bool crossing`. Step 6 needs the same two fields, so this is not
  scaffolding.
- **`failsOf` (`:1562`)**: assign them where the locals already are.
- **`whyItFails(const Wire&) -> WhyFailed`**, beside `failsOf`: separate F1, F2
  and F3. The F2/F3 split needs no change to `CrossingConstraints`:
  `maskAt(x, y) == CURVE_ZONE` is F3, otherwise `!allowed(...)` is F2. Both are
  public, and `router_.crossingConstraints()` is already called at `:1632`.
  Reuse the own-coupler exemption from `crossesAFeedline` (`:1639`) word for
  word.
- **`edgeNearest(wires, point, span&)`**: extract the loop that already sits
  inside `whereItCrosses` (`:1677`) and call it from there.
- One `-v 1` line per failing wire: the type, the nearest edge, the coupler,
  and its `chosen/options` count.

**Verify:** re-run step 1's loop. The counts must not move, and the new line
must appear once per failing wire. Group the log by edge in `awk`; do not put
aggregation in the router yet.

## Step 3 — The bay table. No change in behaviour.

`assignBridges` (`:5685`) **already computes the ring decomposition**: for each
non-terminal edge, the cyclic ring arc between its two couplers, stamped onto
`wire.bridged`. Factor that walk out and build on it. Do not write a second
one.

New, beside `assignBridges`:

- `arcOfEdge(wires, placeOf, edge)` — the ring places of an edge's two
  couplers, shared with `assignBridges`.
- `struct Bay` — the two couplers, their ring places, their chain waypoints
  (`chain`, `at`, which step 7 enumerates over), the joining edge or
  `NO_OWNER`, a `Span box`, and the wire keys.
- `baysOf(wires, outer, members)` — type **A** from the edges' arcs, one inner
  edge each; type **B** from the ring-adjacent pairs left over at a chain
  boundary, where no feedline joins the two couplers and the wires pass between
  the terminal edges without crossing one.
- `boxOfBay(wires, bay)` — the union of `reachOf(way)` (`:5227`, public) over
  the bay's edge, both resonators and the conventional wires in the arc, plus
  `edgeBox` (`:5179`) so the box covers where the edge's *search* may roam and
  not only where it runs now.
- `inBay(wires, members, bay)` — one byte per slot, set where
  `reachOf(wire.way).meets(bay.box)`. **Every member whose way enters the box**,
  not the wires that must cross it: that picks up the two resonators, the
  conventional wires of the arc and any `bridged` partner that happens to lie
  there.
- `ringWithEveryEdge(wires, outer)` — `ringWithEdges` (`:5662`) with its two
  `terminal` tests dropped, so the inner edges are members and can be redrawn.

**Do not use `boxOf` (`:7560`)** for a bay box. It takes `pass.reach`, which is
798 cells on 17q against a grid about 1500 across, so every bay would hold every
wire. `reachOf` is the right tool and it is already public.

**Assert the partition**: every ring place covered once, and say the bay count
and the A-to-B split. Note that a chain which ends in a *termination* has no
launcher waypoint, so "the first and last edge of every chain is terminal" is
false.

**Verify:** cross step 2's per-edge grouping against the bay table on 17q. If
the failures do not clump in a few bays, this is the wrong cut, and the rest of
this document needs rethinking before it is written in code.

## Step 4 — The mask on the sweep. Provably inert.

`sweep` (`:1412`) already **is** the bay-local rip-up and reroute. What it lacks
is a way to say "attempt only these". Add a defaulted
`const std::vector<std::uint8_t>* attempts = nullptr` to `sweep`, `beginPass`
(`:5720`) and `attempt` (`:6259`), indexed by **slot in `members`**. Four edit
sites:

1. `:1437`, the per-slot loop — skip a masked-out slot.
2. `:1455`, the round tally — count masked-in members only, or the early exit
   never fires.
3. `:5727`, **`beginPass` sets `routed = false` for every member** — mask it.
   Without this the first bay marks the whole ring unsettled, only its own
   wires are drawn again, and everything else reads as open for the rest of the
   stage. This is the edit most likely to be missed, because `beginPass` looks
   like setup.
4. `:6373`, the relaxation — it walks `(slot ± level) % total` and sets
   `routed = false` **without lifting the wire**, which is a promise to redraw
   it. Masked out, the promise is never kept: the wire keeps its way, the wire
   being drawn settles across it, and it is left overlapped. **Skip masked-out
   slots and do not charge them against `level`.**

**Keep `members` the full ring and select with the mask.** A short bay-only list
would make `laneOf` (`:7254`) and `fence(±1, ±2)` (`:6300`) wrap inside the bay
and free the wrong region. It also keeps `laneOf`, `startsAtLauncher` (`:7192`)
and `boxOf` private and untouched, so **nothing needs an accessibility change**
as long as the new code goes in as `Driver` members before `:6130`.

**Verify:** run all eight with the switch absent and compare
`artifacts/*/06-final.fb` against the tree. Any difference is a masking bug, not
a result.

## Step 5 — The bay walk.

`sweepBays(wires, members, bays, pass)` — per bay in ring order: rebuild
`bay.attempts`, call the masked `sweep` with `verbosity_` and `progress_` saved
and zeroed the way `repair` does at `:6085`, and one `say` line per bay. Use a
scope guard for that save; `repair`'s version is not exception-safe and has no
early return, while `sweepBays` will have several. A run that throws mid-walk
would otherwise leave the driver mute for the rest of the stage.

Then **one global `failsOf`** as the verdict. Bay boxes overlap, so a wire
redrawn in bay 7 can break what bay 3 settled, and a sum over bays is not a
number. Log the walk order, or two runs that differ cannot be compared.

Gate it: `feedlineBays()` through `envFlag("SCPD_FEEDLINE_BAYS", false)`,
following the house pattern at `:87`–`:111`, and add it to the `settings:` line
(`:2288`) so a stale export shows in the log.

**Verify:** fails and seconds against the old path, per chip. Expect the
failures to **move** rather than vanish — the walk alone cannot change a
coupler. And expect less speedup than the bay count suggests: `pass.reach` is
chip-scale and `constrainByFeedlines` (`:5780`) fences every edge of every
chain per attempt, so only the number of attempts falls, not the cost of one.

## Step 6 — The plumbing.

**The alternates are already computed and thrown away.** `problem.found`
(`:4931`) fires for **every complete run of options whose every edge is drawn**,
with its cost — and only prints it. Push each `(run, cost)` into a new
`ChainAnswer::alternates` (`:4511`) and keep it per chain in
`optimizeChainsPrefix` (`:5523`). That is the k-best list, free.

- `boundsOf(chain, open)` — **extract** the bound build at `:4894`–`:4917` word
  for word, including the `boundPairs_`/`boundNanos_` accounting, and call it
  from `solveChainAStar`. Recompute rather than cache: it costs microseconds,
  and a cached member would sit outside `Snapshot` (`:5845`).
- `turnCouplers(wires, turns)` — `turnCoupler` (`:5889`) for several couplers at
  once: one `snapshot`, lift and clear the union of `edgeIn`/`edgeOut`,
  `applyOption` each, `routeEdge` each distinct edge, `restore` and return false
  on the first empty way. Then rewrite `turnCoupler` as a one-element call, so
  there is one body and not two.
- **Mend `edgeWayOf` (`:3280`).** It falls back to `chainEdgePaths_` whenever
  the edge's wire is not drawn, and `chainEdgePaths_` is neither in `Snapshot`
  nor written by `turnCoupler`. So `fenceCommittedEdges` fences against an
  insertion-time path that no longer corresponds to anything. Return an empty
  path when the wire exists and is undrawn. This is a live fault today; step 7
  routes far more edges and would meet it far more often.

**Verify:** run the **old** path with the repair on. Add
`tuning.repairTrials` through `envWhole` at `:410` so the eight configs and the
test fixtures need no edit, then `SCPD_REPAIR_TRIALS=100`. The results must not
move, because `turnCoupler` is now a one-element call.

## Step 7 — Grouped enumeration.

- `componentsOf(bays, failing)` — union-find over the failing bays, **connected
  when a wire touches both**. That relation also catches a conflict across two
  chains, because a conventional wire can touch bays of different chains. Walk
  in bay-index order and sort each component, or `IsDeterministic`
  (`test/pipeline/test_final_router.cpp:410`) fails.
- Enumerate the assignments to the waypoints a component spans, the rest held,
  ordered by the summed analytic bound from `boundsOf`. The bound is a lower
  bound on the real angle cost, so cheapest-first is admissible and a budget can
  cut the tail off. Prefer combinations that appear in `alternates`. **Never
  order by `CouplerOption::cost`** — see the traps.
- Per candidate: `turnCouplers`; work out which bays are dirty — `reachOf` of
  the old and the new way per redrawn edge, `Span::meets` against each
  `bay.box`, which is `forgetEdgesNear`'s test (`:5205`) applied to bays;
  rebuild those bays' boxes and masks; run `sweepBays` over them plus the
  component's own bays and their ring neighbours; score over the ring members
  and the chain edges with the length-excluding count; reject a candidate that
  raises the inner fail count; `restore` between candidates.
- Take the first candidate with no failures. Otherwise keep the best **by
  fails, then by angle** — the two-level rule `stateFaults` already uses.
  Exhausting a component's candidates is a **structural** verdict, and saying so
  is the one thing the current repair cannot do.
- Caps through `envWhole`: candidates per component, waypoints per component.

**Keep the bay table a local owned by the caller, never a `Driver` member.**
`restore` (`:5864`) rebuilds `field_` and the crossing rule from `wires` alone,
so any new driver state survives a rejected trial untouched — and a bay's box
and mask are computed from ways a rejected candidate moved and `restore` moved
back.

## Step 8 — The default, the tests, the write-up.

Flip `SCPD_FEEDLINE_BAYS` to true only once it beats the old path on all eight
chips. `uvx nox -s lint` reformats every committed file, so do not run it
blindly.

Tests belong in `test/pipeline/test_final_router.cpp`. A function-local static
cannot be toggled in-process, so test the **pieces**: `baysOf` partitions the
ring, `inBay` catches a wire whose way enters the box and not one that misses
it, `whyItFails` separates a curve-zone cell from a wrong-heading cell. The
guards on the default path are `FeedlinesAreDrawnAndCrossedAtRightAngles`
(`:359`), `TheEdgesOfAChainMeetOnlyAtTheCoupler` (`:329`) and `IsDeterministic`
(`:410`).

## Where the work lands

Everything is in **`src/pipeline/FinalRouter.cpp`**: the switches at
`:87`–`:258`, `Wire` at `:845`, `sweep`/`beginPass`/`attempt` at
`:1412`/`:5720`/`:6259`, `failsOf`/`whereItCrosses` at `:1562`/`:1659`,
`ChainAnswer` at `:4511`, `problem.found` at `:4931`, the bound build at
`:4894`, the spatial kit at `:5165`–`:5256`, `assignBridges`/`ringWithEdges` at
`:5662`–`:5718`, `snapshot`/`restore`/`turnCoupler`/`repair` at
`:5845`–`:6128`, and the phase-4 call site at `:8672`.

Also `test/pipeline/test_final_router.cpp`, and
`include/mqt-scpd/routing/CrossingConstraints.hpp` read-only, for `maskAt` and
`CURVE_ZONE`.

## Traps

**1. `beginPass` unsettles the whole ring.** `:5727` sets `routed = false` for
every member. Under a mask that is ignored, the first bay marks every ring wire
unsettled and the rest of the stage reads them as open.

**2. The relaxation releases wires it will never redraw.** `:6373` sets
`routed = false` without lifting — a promise. Masked out, the promise is broken
silently and shows up as an open fail in a bay nobody was working on.

**3. `restore` rebuilds from `wires` alone.** `:5864` reassigns the wires, the
bodies, the couplers' `chosen` and `anchor`, then rebuilds `field_` from nothing
and the crossing rule. Anything else on the driver survives a rejected trial.
Add no driver state.

**4. `chainEdgePaths_` is outside `Snapshot`** and goes stale the moment an edge
loses its way. See step 6.

**5. `CouplerOption::cost` is dead on the A\* path.** It is written only by the
greedy (`:4325`, `:4348`, `:4372`), so with `SCPD_CHAIN_ASTAR` on it stays at
`uint32` max for every option of every coupler. `repair`'s "up to two untried
options, the cheapest first" (`:6055`) is therefore a `stable_sort` on a
constant. Do not read that as evidence a cost ordering was ever measured, and
do not reuse it.

**6. A feedline edge is never a crossing fail.** `crossesAFeedline` returns
false at once for `wire.feedline` (`:1632`). An edge drawn straight through
another chain's edge reports nothing from the crossing rule; only `conflictsOf`
sees it, as `open`. Do not read a quiet crossing column as a clean bay.

**7. Bay boxes overlap, so the walk is order-dependent.** A wire in three bays
is drawn three times and the last bay wins. That is why the verdict is one
global `failsOf` and never a sum.

**8. Build the member list once.** Interleaving the inner edges changes ring
adjacency for every wire after the insertion point, which changes `fence(±1,
±2)` and `laneOf` for wires that are in no bay's mask. Two bays swept on lists
built at different moments would disagree about the same geometry.

**9. `failsOf` writes while it counts.** It sets `wire.tooShort` and
`wire.tooLong` as a side effect (`:1585`). `whatBlocks` (`:6469`) mutates the
crossing rule and `bodies_` and restores them, and its own comment says it once
changed the result it was there to explain.

**10. Six environment switches change the rules underneath all of this.**
`SCPD_FEEDLINE_PROTOTYPE` (on) governs the ±2 fence, the per-round band growth,
`laneOf`, and whether terminal edges are in the crossing rule and the fences at
all; `SCPD_ORTHO_CROSSING` (on) must be all-or-nothing across build, search and
count; `SCPD_FEEDLINE_FORCE_REROUTE` (on) makes the "keep its way" shortcut in
`attempt` dead; `SCPD_FEEDLINE_MEANDER` (**off**) means the phase cannot change
a length; and `SCPD_CHAIN_SOLO` and `SCPD_CHAIN_KEEP_WAYS` (both on) are what
leave 69q with 22 edge-against-edge overlaps for phase 4 to find. Read the
`settings:` line of a run before believing any number.
