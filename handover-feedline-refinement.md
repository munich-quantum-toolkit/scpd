# The feedline refinement, and the lengths

Written for whoever takes the fifth phase of the Final stage further. The
phase is `refined`, it runs after the feedline routing on its finished
state, and it is where the resonators are finally made their length: until
2026-10-05 the stage routed and did not read a length, and the meander was
switched off in every feedline pass. It is on in both of them now, roughly
in the sweep and exactly here.

The figure is **two-part, and both parts stand in every table**:
`bad = unrouted + open + crossing` may not rise, and `short + long` is what
the phase lowers. Over the eight benchmarks it stands at **bad 2 and
short 6, no long**, from bad 8 and short 102 where the fourth phase left it
— see *Where it stands*. Never quote the `Fails:` sum of the log line; it
counts things that are nobody's figure.

Read [handover-feedline-routing.md](handover-feedline-routing.md) for the
stage this one ends, [handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md)
and [handover-chain-astar.md](handover-chain-astar.md) for the ground the
chains stand on, and *The meander* in
[summary-final-routing.md](summary-final-routing.md) for how the insertion
is built and what the user corrected in it on 2026-09-15.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch `phase-4-routing-stages`, HEAD **`624130f` coupler of minor fails
  remain feedline refinement next**, which carries the whole of 2026-10-05
  up to the squeeze rule. Everything in this document is **uncommitted** on
  top of it; the user commits per phase.
- **The phase is the default since 2026-10-05** (user). Three things make
  it so: `feedline_refinement_rounds` is **2** in `schemas/config.fbs` and
  in `python/mqt/scpd/config.py`, `SCPD_FEEDLINE_MEANDER` is **on**, and
  `stop_after` is `"refined"` in all eight benchmark configs. The configs
  do **not** repeat `feedline_refinement_rounds` — the doctor refuses a key
  set to its own default, which is how the flip was caught.
  `artifacts/logs/default1` is the arm a plain
  `artifacts/logs/run-arm.sh <arm>` produces, and it reproduces `sweep6`
  chip for chip.
- The arms named `sweep6` in this document were measured with
  `STOP_AFTER=refined REFINE_ROUNDS=2 … SCPD_FEEDLINE_MEANDER=1` before the
  flip; `run-arm.sh` still takes `REFINE_ROUNDS=` for a sweep over the
  rounds.

## Where it stands

**2026-10-05 evening**, `artifacts/logs/sweep6` against the fourth phase's
own state (`artifacts/logs/squeeze-reject2`, the arm this phase was handed
over on). Both arms are the eight chips one at a time on the installed
binding, `repair_trials = 0`, `max_relaxation = 8`, 69q at five rounds.

| chip | phase 4: bad · short/long | **phase 5**: bad · short/long | s | of that phase 5 |
|---|---|---|---|---|
| 4q | 0 · 4/0 | **0** · **0**/0 | 4 | 0.7 |
| 9q | 0 · 2/0 | **0** · **0**/0 | 14 | 5.2 |
| 17q | 4 · 7/0 | **1** · **3**/0 | 46 | 8.5 |
| 21q | 1 · 3/0 | **0** · **1**/0 | 76 | 24.3 |
| 33q | 0 · 10/0 | **0** · **1**/0 | 85 | 27.5 |
| 45q | 0 · 13/0 | **0** · **0**/0 | 211 | 54.0 |
| 57q | 2 · 32/0 | **0** · **0**/0 | 288 | 92.4 |
| 69q | 1 · 31/0 | **1** · **1**/0 | 661 | 124.3 |
| **all** | **8** · **102**/0 | **2** · **6**/0 | | |

Five chips are clean on both figures. What is left:

- **Two crossing wires**, 17q's 31 and 69q's 15, and they are the same case
  the fourth phase left: a plain wire crossing a chain edge ten cells from
  it, the halo's edge where the search and the count disagree. The one
  lever on them is `SCPD_CROSSING_EXIT_HEADING`, which is off and is the
  user's decision — see *The search and the count disagree at the halo's
  edge* in [handover-targeted-repair.md](handover-targeted-repair.md). The
  refinement **closed three of 17q's four** (14, 33, 56) and three of the
  eight over the chips, which no room rule had reached before: the
  re-route with the room price moves a wire off the halo's edge.
- **Six short resonators**: 17q 1, 30 and 44, one each on 21q and 33q,
  69q's 3. None of them is a question of room — see *What is not done*.
- **No long anywhere**, on any chip, in any arm measured.

`artifacts/logs/bad-table.py <arm> …` prints this table; it carries the
short and long columns and the runtime of the fifth phase on its own since
2026-10-05.

## What the phase does, and where it differs from the prototype

`Driver::refine` (`src/pipeline/FinalRouter.cpp`, search for `void refine(`)
is both refinements — the outer one after the outer routing and this one —
and the `Pass::refinement` flag is what tells the two apart. One attempt per
wire, no rip-up, a rollback onto the way the wire had. The prototype is
`run_final_routing_feedline_refinement_parallel`
(`FinalGrid.cpp:12391`, its head in `FinalGrid.hpp:989`).

| the prototype | here |
|---|---|
| corridor = the backup way widened by `ref_corridor_expansion` **200** | `pass.reach` = `corridor_spacings · clearance` = **798** on every benchmark |
| ring neighbours **i ± 1 … i ± 4** hard at `ref_min_clearance` 19 | `SCPD_REFINE_FENCE_PAIRS` pairs, **3** |
| `compute_corridor_proximity_decay` on the fully marked corridor | `priceRoom`, after the constraints since `SCPD_REFINE_PRICE_LAST` |
| `compute_proximity_grid`, 5× near any routed feedline, stamped on top | `priceTheEdges`, the same figure and the same order |
| takes whatever it finds; rollback only on "no way" | takes it only when `conflictsIn` does not rise and it crosses no feedline |
| `meander_insertion_proximity_strict`, failure → rollback | `lengthen` with `exact`, the same |
| **fallback**: one more route with the soft field zeroed, for `!fitted` | `SCPD_REFINE_FALLBACK`, the same, `fitted` set on the commit rather than on the meander |
| `fitted[i]`: has this resonator ever been at its length | a local vector over the pass |
| `FG_REFINE_MEMO`, a quiescence skip | not built |
| five rounds, hard in the code | `feedline_refinement_rounds`, 2 in every measurement here |

Two things in that table read backwards until you check them:

- **The prototype does not widen its refinement corridor, it narrows it.**
  200 against the 800 of its own phase 1 (`(round + 1) · outer_expansion`,
  `FinalGrid.cpp:11695`). Ours runs on the full sweep width, 798 — four
  times the prototype's. Room is not what this phase is short of, and the
  4-qubit measurement below says so in one number.
- **`ref_refinement_rounds` (2) is a dead parameter in that function**: it
  runs a hard-coded five.

## The two passes ask different questions

`SCPD_FEEDLINE_MEANDER` used to switch the meander in **both** feedline
passes at once, which is why the lengths were off in this phase: the sweep
is judged by whether a resonator can be routed at all, the refinement by
whether it can be made its length, and one switch cannot say that.

- **`SCPD_REFINE_MEANDER`** (on) lengthens in the refinement whatever the
  sweep does.
- **`SCPD_SWEEP_LENGTH_BAND`** (5) is how wide the sweep's length band is,
  as a multiple of `resonator_length_tolerance`. This is the prototype's
  own arrangement and not a knob it tunes: its sweep calls
  `meander_insertion_proximity` and its refinement
  `meander_insertion_proximity_strict` (`FinalGrid.cpp:11809` against
  `:13570`), so the sweep aims at *at least* the length and nothing there
  can be too long. **The figure the stage is judged by is unchanged**:
  `failsOf` counts short and long against the tolerance itself.

Why the band matters, measured on 17q (`artifacts/logs/refine-probe`):

| 17q | bad · short/long | s |
|---|---|---|
| no meander in either pass, no refinement | 4 · 7/0 | 21.5 |
| refinement only, no lengths in it | 1 · 9/**1** | 29.8 |
| meander in the sweep at band 1, no refinement | **6** · 3/0 | 72.2 |
| meander in the sweep at band 1 + refinement | 3 · 3/0 | 83.6 |
| **meander in the sweep at band 5 + refinement** | **1** · **3**/0 | 46 |

The second row is why the refinement may not run without lengths: a
clearance-maximising way is a detour, and a detour overshoots — wire 5 came
out 3218 against 3000 ± 200, the only `long` ever measured here. The third
row is the narrow band's price, the open pair 30/31, and the last row is
that pair gone. The user's reading on 2026-10-05 — *roughly in the sweep,
finely in the refinement* — is what the two rows separate.

## The 4-qubit meander that could not be placed

The user found it: a resonator that stays short on a chip with four qubits
and room everywhere. It is worth writing down because the diagnosis was the
whole of the fix, and because the first guess — the corridor — was wrong.

`MeanderResult::Refusals` now counts every refusal by its reason, and the
stage's line names them with the budget the pairs are drawn from:

```
no room for a meander: 122 of 243 cells, 21420 placements tried over 70
straight cells less 35 reserved, widest span 34 of the 35 a loop needs
  — 16660 the two ends do not face each other,
    4760 no pair far enough apart for the legs
```

**Not one refusal for want of room**, no closed cell, no self-crossing, no
box. The 16660 are the head × tail cross product being filtered; every one
of the 4760 real placements fell at `run = span − 2 · BEND_RADIUS ≥
minStraightLength`. The way is 111 cells, 70 of them on a straight run; the
start margin reserved the first 35 and a loop needs 35 cells of span
between its two ends. **Impossible by one cell**, in every round, at every
relaxation.

The margin was `couplerLength + straightStart + 4`. What it is for is to
keep the loop out of the run the resonator couples along, which is the
coupler's own length; the `straightStart` in it was the second straight run
after the lead, and `SCPD_RESONATOR_STUB` took that away on 2026-10-03
without this figure following. It is `couplerLength + 4` = 24 now. 4q ends
with **no failing wire and no resonator off its length**; wire 4 goes 126 →
243 cells, 2508 units against 2500 ± 100. On 17q the change is neutral —
bad 1 and short 3 either way, a different three.

Three switches came out of it: `SCPD_MEANDER_MARGIN_STUB` (off) puts the
stub term back, `SCPD_MEANDER_START_MARGIN` sets the margin outright, and
`SCPD_MEANDER_LEG_SPACING` (25, the prototype's `min_straight_length`,
which is **not** the design rule of that name) is what the two legs of one
loop keep between them. The floor that means anything there is the wire
clearance, 19: two legs of one wire are not a pair the design-rule check
looks at, so nothing below it is defensible on its own.

## The checks, and what they found

The four checks of the coupler insertion run at the end of `insertCouplers`
and the crossing one again after the repair. **The refinement moves
feedline edges** — 7 of 20 on 17q — and nothing looked at the result:
`failsOf` sees a conventional wire crossing an edge, not an edge crossing an
edge, and nothing at all re-measures the room around a coupler lead. All
four are said again at the end of the phase under `SCPD_REFINE_CHECKS`
(on), and the first one at the end of phase 4 as well, so that a pair can
be blamed on the pass that made it.

On `sweep6`: coupler crossings, resonator crossings and feedline crossings
are **green on all eight chips**, 69q included. `CHECK feedline room` is
green on 9q, 17q, 57q and 69q and **red on 4q, 21q, 33q and 45q** — one
pair on 4q (f0 vs 4), two on 21q (f6 vs 15, f16 vs 43), one each on 33q
(f17 vs 49) and 45q (f25 vs 69) — where the insertion's own line is green
on every one of them. So a pass after the insertion walks a feedline edge
into the clearance of a coupler lead. **Which pass is not yet separated**:
the line at the end of phase 4 was added after `sweep6` ran, so the first
arm that says it is the next one. See *What is not done*.

## What the control arms say

Measured on the eight chips (`ctl-nosweep`) and on 17q and 69q, the two
chips that still carry a fail (`ctl-*`), each against `default1`:

| arm | what it changes | 17q: bad · short | 69q: bad · short | 17q s | 69q s |
|---|---|---|---|---|---|
| `default1` | the defaults | 1 · 3 | 1 · 1 | 45 | 677 |
| `ctl-pairs1` | one pair of ring neighbours fenced, not three | 1 · **4** | 1 · **8** | 50 | 858 |
| `ctl-pricefirst` | the room price before the constraints | 1 · 3 | 1 · 1 | 46 | **627** |
| `ctl-rounds5` | five rounds, as the prototype runs | 1 · 3 | 1 · 1 | **62** | **871** |
| `ctl-nofallback` | no second search without the room price | 1 · 3 | 1 · 1 | 44 | **602** |

- **`SCPD_REFINE_FENCE_PAIRS` = 3 is the one that earns its keep.** With
  one pair 69q goes from one short resonator to eight and 17q from three to
  four, and the run is a quarter slower — a loose fence wastes searches on
  ways the accept test then refuses, exactly as the switch's comment
  predicted.
- **The room price's order changes nothing** on either chip, and building
  it before the constraints is 7 % faster on 69q. It is on because it is
  the prototype's order and because the chamfer not seeing the feedline
  walls is wrong on the face of it; **no measurement supports it.**
- **The prototype's five rounds buy nothing** over two and cost 38 % on
  17q and 29 % on 69q.
- **The fallback buys nothing** and costs 11 % on 69q. It was built for a
  detour with no slack left, and once the sweep makes the lengths the ways
  the refinement sees are already near their target, so the case is rare.
  **This is the first candidate to switch off.**

And over the eight chips, what the sweep's meander is worth once the
refinement makes the lengths properly (`ctl-nosweep`,
`SCPD_FEEDLINE_MEANDER=0`): `bad` **2** either way, short **10** against
**6**, and 17q 31 s against 45 s, 69q 616 s against 677 s. So the sweep's
meander is worth four resonators for about a third more time on the small
chips and a tenth on the large ones. The refinement alone already reaches
bad 2 and short 10, where the fourth phase alone leaves bad 8 and short
102.

## Where the room-check pairs come from

The room check now says its line at the end of the insertion, at the end
of phase 4 and at the end of phase 5, which settles the question *What is
not done* asked. On `default1`, pairs closer than the rule:

| chip | insertion | after phase 4 | after phase 5 |
|---|---|---|---|
| 4q | 0 | **1** | 1 |
| 21q | 0 | **2** | 2 |
| 33q | 0 | **2** | 1 |
| 45q | 0 | 0 | **1** |
| 57q | 0 | **1** | 0 |
| 9q, 17q, 69q | 0 | 0 | 0 |

**Six of the seven are the sweep's**, and the refinement closes two of
them and makes one. So this is a defect of the **fourth** phase, standing
unseen since that pass began redrawing the chain edges: the insertion
fences every coupler lead at the clearance when it draws an edge, and the
pass that redraws the same edge does not reproduce that fence. The
refinement only revealed it by having a check. It is not this phase's to
fix, and it is the first thing to look at in the fourth.

## 17q's wire 31, diagnosed and not fixed

The one `bad` wire left on 17q, read on 2026-10-06. The diagnosis is here
because it cost four arms and because the obvious answer is the wrong one;
**no fix for it is in the tree** — one was built, measured and reverted at
the user's instruction.

The count says `31 crosses at (316,67), 11 cells from edge f8`, which reads
like the halo's-edge case of `SCPD_CROSSING_EXIT_HEADING`. It is not.

| 17q arm | bad · short | what it says |
|---|---|---|
| the defaults (`default1`) | 1 · 3 | 31 crossing |
| `SCPD_CROSSING_EXIT_HEADING=1` (`ctl-exit`) | 1 · 3 | **no change.** The switch is in force (the settings line says `the exit heading tested yes (env)`) and 31 is *dead on arrival* in every relaxation of every round, so it never had a way to refuse |
| `SCPD_BRIDGE_CHECK=0` (`diag-nobridge`) | 1 · 3 | no change; no way of 31's was ever refused for missing its bridge |
| `SCPD_ORTHO_CROSSING=0` (`diag-noortho`) | **0** · 2 | 31 routes, and the chip is clean |

**Wire 31 never finds a way at all.** It keeps the seed the Detail stage
drew, and that seed crosses f8 at an angle. Two things hold it:

1. **Its search cannot take its first step.** `dead on arrival: the search
   starts at (361,57) heading (0,1) after a stub of 11, and 3 of its first
   4 cells are closed: (361,58) … within the clearance of f8 (way at 19.0
   cells)` — all three at **exactly 19.0 cells**. `tuning_.clearance` is
   the whole cells the rule spans, 19, while the rule is
   `min_wire_spacing / cell` = 185 / 9.86 = **18.76**, and `conflictsIn`
   and the design-rule check both measure the rule itself. So the fence is
   a quarter of a cell stricter than every judge, and those three cells are
   closed to the search and acceptable to everyone who looks. Fencing by
   the rule instead of the disc **removes the dead-on-arrival entirely**
   (measured, `artifacts/logs/fence-rule`), and 31 still finds no way: the
   phase-1 verdict becomes `in the way: the feedlines`.
2. **Then the orthogonal crossing rule holds it.** With the rule off the
   wire routes and 17q ends at `bad` 0. It is enclosed by chain edges and
   has exactly one door — `constrainByFeedlines` fences every drawn edge
   but the one `assignBridges` gave it — and that door cannot be taken at
   a right angle from where the wire starts.

**Caveat on the last row**: `diag-noortho` and `diag-nobridge` were run on
a binding that carried the rule-exact fence, which is no longer in the
tree. The `bad` 0 therefore stands on fence-by-the-rule **and** the
crossing rule off together; neither has been measured alone on the tree as
it is.

What this says for whoever picks it up: the lever is not the exit heading,
and it is not the bridge check. It is the fence's rounding and the one-door
bridge assignment, in that order.

## The switches

| switch | default | |
|---|---|---|
| `SCPD_FEEDLINE_MEANDER` | **on** | the sweep makes lengths too, since 2026-10-05; `=0` leaves it routing and the refinement makes them alone |
| `SCPD_REFINE_MEANDER` | **on** | the refinement makes lengths whatever the sweep does |
| `SCPD_SWEEP_LENGTH_BAND` | **5** | the sweep's length band, as a multiple of `resonator_length_tolerance`; 1 is the band the sweep had, measured on 17q above |
| `SCPD_REFINE_FALLBACK` | **on** | the prototype's second search with the room price zeroed, for a resonator never at its length. Measured useless and 11 % of 69q's run (`ctl-nofallback`); on only because it is the prototype's |
| `SCPD_REFINE_PRICE_LAST` | **on** | `priceRoom` after `constrainByFeedlines`, so the chamfer sees the feedline walls; `=0` is the order the phase was written with and is identical in every judged line on 17q and 69q (`ctl-pricefirst`), 7 % faster |
| `SCPD_REFINE_FENCE_PAIRS` | **3** | pairs of ring neighbours fenced; 1 is what the phase had and takes 69q from one short resonator to eight (`ctl-pairs1`), 4 is the prototype. **The one switch here with a measured case for it** |
| `SCPD_REFINE_CHECKS` | **on** | the four checks again at the end of the phase, and the room check at the end of phase 4 |
| `SCPD_MEANDER_MARGIN_STUB` | **off** | put the straight start back into the meander's start margin — see *The 4-qubit meander* |
| `SCPD_MEANDER_START_MARGIN` | unset | the margin outright, in cells, over the two above |
| `SCPD_MEANDER_LEG_SPACING` | **25** | what the two legs of a loop keep between them |
| `SCPD_COUPLER_BIAS` | **1.0** | where the coupler cuts the way, as a share of the target length; below 1 it cuts further up and hands the difference to the meander |
| `feedline_refinement_rounds` | **2** | the rounds of the phase. The prototype's 5 is identical in every judged line and costs 38 % on 17q and 29 % on 69q (`ctl-rounds5`) |

The phase prints a `feedline refinement settings:` line naming the first
six, and the insertion's `lead … cells straight` line now names the bias.

## Where the pieces are

- `src/pipeline/FinalRouter.cpp` — `refine` is the phase; `lengthensIn`
  decides whether a pass lengthens; `lengthen` wraps the insertion and
  takes the band; `priceTheEdges` is the 5× feedline price, split out of
  `constrainByFeedlines` so the refinement can stamp it after the room
  price; `meanderStartMargin`, `meanderLegSpacing`, `couplerBias`,
  `sweepLengthBand`, `refineMeander`, `refineFallback`, `refinePriceLast`,
  `refineFencePairs`, `refineChecks` are the switches.
- `include/mqt-scpd/routing/MeanderInsertion.hpp` and
  `src/routing/MeanderInsertion.cpp` — the insertion, with
  `MeanderResult::Refusals`, `said()` and the `straightCells` / `reserved`
  / `widestSpan` budget.
- `python/mqt/scpd/plot.py` — the clearance band is drawn at 0.26 opacity
  since 2026-10-06 (user), where 0.13 made a single band invisible at the
  zoom of a whole chip and only the overlaps read, so the picture looked as
  if the room were drawn at the crossings alone. The band covers every
  wire, the chain edges included — `geometry.inner_wires` carries them.
- `python/mqt/scpd/planning.py` — `LENGTH_VERDICTS` and
  `PlanningGeometry.off_length`; `plot.py` draws them amber and dashed on
  `l-offlength` under `l-failing`, `export/klayout.py` on layer 31
  `final.off-length`. The marks are on the **end state** only: the phase
  snapshots carry no verdict, so `--phase refined` shows none.
- `artifacts/logs/` — `run-arm.sh` takes `REFINE_ROUNDS=` now;
  `bad-table.py` prints short, long and the phase's own runtime. The arms
  of 2026-10-05 evening are `sweep6` (the state above), `base6` (phase 4
  alone on the same binding), `sweep6-m35` (the old start margin),
  `sweep6-b95` (the bias at 0.95) and `ref6` (the refinement without the
  sweep's meander); `refine-probe/` holds the 17q and 4q single-chip runs
  the two tables above are from.

## What is not done

- **The red `CHECK feedline room` on four chips is not attributed.** Green
  at the insertion, red after the refinement, and the line that separates
  the sweep from the refinement was added after `sweep6` ran. Read it on
  the next arm first; if the sweep is the one that does it, this is a
  defect of the fourth phase that has stood unseen since the pass began
  redrawing edges, and the refinement only revealed it.
- **The fence is stricter than every judge.** `closeRoomOf` closes a disc
  of `tuning_.clearance` whole cells where `conflictsIn` and the DRC
  measure `tuning_.spacing`, the rule unrounded — 19 against 18.76 on every
  benchmark. A rule-exact fence was built, measured on 17q (it removes the
  dead-on-arrival that pins wire 31) and **reverted**, so nothing of it is
  in the tree. It is cheap to rebuild: `stencilOf` taking a real radius, one
  cached stencil of `tuning_.spacing`, and `closeRoomOf` choosing between
  them. What it does to the other seven chips has never been measured.
- **The six short resonators are not understood one by one.** None of them
  is a question of room in the sense the 4-qubit one was — that case is
  closed — but no one has read the refusal breakdown for 17q 1, 30, 44 or
  69q 3. The line now says exactly what to look at.
- **`SCPD_COUPLER_BIAS` has never had a `long` to fix.** It was built for
  them and no arm has produced one; the 0.95 arm is a control measurement
  with nothing to control. If the band ever lets a way out long, this is
  the lever.
- **The fallback has not paid for itself.** On 17q under the sweep's
  meander it took 1 of 5 searches in one round and 0 of 3 in the next; on
  4q it took none. It is the prototype's and it is cheap, but the case it
  was built for — a detour with no slack left — is rare once the sweep
  makes the lengths, because the ways the refinement sees are already
  near their target.
- **`SCPD_REFINE_FENCE_PAIRS` and `SCPD_REFINE_PRICE_LAST` have no control
  arm.** Both are on in every measurement here and both are reasoned from
  the prototype rather than measured. They are one arm each.
- **The rounds are 2, the prototype runs 5.** Nobody has measured what a
  third, fourth and fifth round buy. The phase costs 0.7 s on 4q and
  124 s on 69q at two rounds, which is a fifth of that chip's run.
- **The outer refinement now carries `Pass::refinement`** and therefore
  `SCPD_REFINE_FENCE_PAIRS`, `SCPD_REFINE_PRICE_LAST` and the fallback as
  well, where it used to fence one pair and price before the constraints.
  It is not a feedline pass, so the checks and `SCPD_REFINE_MEANDER` do not
  reach it, but the fence and the price order do. That change is in every
  arm here and has not been measured on its own.
- **Three of the four switches reasoned from the prototype buy nothing**,
  measured — see *What the control arms say*. `SCPD_REFINE_PRICE_LAST`,
  `SCPD_REFINE_FALLBACK` and a third, fourth and fifth round all leave
  17q and 69q on exactly the same figures, and two of them cost runtime.
  They are on because they are the prototype's and because off is not
  better either; nothing here argues for them.

## Tests

**`test_the_final_routing_carries_a_snapshot_of_every_phase` passes again.**
It had been failing since 2026-10-04 because the benchmarks stopped after
`feedlines` and it expects all five phases; `stop_after = "refined"` is
what settles it. The flip also broke `test_doctor`'s four cases for a
while — the doctor refuses a configuration that sets a key to its own
default, so `feedline_refinement_rounds = 2` had to come **out** of the
benchmark configs again once the schema carried it.

`.venv/bin/python -m pytest test/python/unit`: **145 of 146 pass**, and
the one that fails is the 9q Detail defect this phase does not touch
(`test_the_detail_routing_carries_a_drawn_wire_per_connection`, wire 0
starts where it is not fed). Before the phase it was 143 of 145 with two
failures. The two new tests are
`test_the_resonators_off_their_length_are_marked_on_a_layer_of_their_own`
and the length layer in
`test_the_final_routing_marks_the_wires_the_stage_left_failing`.

**`ctest` has not been run on this tree.** It is owed, and
`test/pipeline/Benchmarks.hpp` reads `feedline_refinement_rounds` from the
config text, so the suites that run the whole Final stage now run five
phases rather than four.
