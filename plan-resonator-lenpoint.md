# Plan: the resonator length-point clearance in the outer routing and the refinement

Asked for by the user on 2026-10-04: *analyse the prototype's mechanism that
gives a resonator a larger clearance to its neighbours at the place where it
reaches the resonator length, to ease the coupler insertion, and plan how to
add it to the rip-up-and-reroute of the outer routing and to the refinement.
Do not build it yet.* Later the same day the user asked for it to be built,
with three changes to the plan below: **only** the outer routing and the
outer refinement (no feedline pass), **hard** everywhere — no second try
without the constraint — and the prototype's figures as they are: `k = 40`
and a plain band of ±10 % of the target length around the point where the
remaining length is the target length less the anchor gap, no derivation
from the pad and no lead subtracted. It is in the tree under `SCPD_LENPOINT_K` /
`SCPD_LENPOINT_PCT` / `SCPD_LENPOINT_REPORT`; see *The length-point
clearance* in `handover-feedline-routing.md` for what was built and
measured. The rest of this document is the analysis and the plan as
approved.

Repository `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
`phase-4-routing-stages`, the uncommitted work of 2026-10-04 on top
(`handover-targeted-repair.md`). The prototype is
`/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/FinalGrid.cpp`
with its parameters in `FinalGrid.hpp`; line numbers were read on
2026-10-04.


## What the prototype does

The mechanism is the **RRR-Lenpoint-Constraint** of
`run_final_routing_parralel` (`FinalGrid.cpp:6193-6600`, the outer routing)
and of the clearance refinement inside the same function (`:7620-7760`). Its
parameters are `FinalGridParams::outer_res_lenpoint_clearance = 40` cells
(`FinalGrid.hpp:855`, the radius `k`; 0 is off) and
`outer_res_lenpoint_band_pct = 10.0` (`:863`, the half-width of the band in
per cent of the target length); `FG_RES_LENPT_K` and `FG_RES_LENPT_PCT`
override them from the environment.

**The point.** For every resonator the *length point* is the cell of its way
at which the remaining length to the target end equals the length the
coupler insertion aims at. It is measured **backwards from the target end**
over the sampled geometry (`res_length_marks`, `:6317`): `want =
meander_length − anchor − kLenPointUndershoot`, where `anchor` is the gap
from the way's last sample to the true port position and the undershoot
(3.0) mirrors the insertion's "aim slightly low" margin. Measured backwards
because `compute_cpw_coupler_insertion_point` scores every candidate by the
remaining length to the path's end and throws the launcher side away: a
resonator of about target length has the point at its start, a long one far
into its middle. Before a wire has a routed path (round 0), the point is
read off the coarse predecessor path by the same backward walk
(`res_length_marks_cells`, `:6268`) — without that seed the first wire of a
round would route unconstrained and never be touched again.

**The band, not the point.** The point moves with every reroute and every
meander, so what is protected is the stretch of the way between the
`(1 − pct)` and the `(1 + pct)` marks of `want`: `LenMarks{lo, pt, hi, band}`
(`:6255`), the band being the real run of cells between the marks, duplicates
removed so that `mark_obstacles` can interpolate between them.

**Two stamps, applied wherever `mark_obstacles` fences a neighbour**
(`:6406-6560`; phase 1 at `:6984-6997`, every relaxation level at
`:7095-7161`, the refinement at `:7720-7740`):

1. `lenpt_stamp_band(buf, wire_idx, own_idx, k)`: when a **neighbour** that
   is fenced is a resonator, its band is inflated by radius `k` into the
   corridor buffer instead of `min_dist_wires` — the wire being routed keeps
   `k` from the stretch on which the neighbour's coupler will sit.
2. `lenpt_stamp_opposite(buf, wire_idx, res_id, k)`: when the wire being
   routed **is itself a resonator**, its own band cannot be stamped (it lies
   on its own path), so the two marks are projected onto each fenced
   neighbour's path — the nearest cell to `lo`, the nearest to `hi` — and
   the neighbour's stretch between the two projections is inflated by `k`:
   exactly the part of the neighbour that runs alongside the resonator's
   length-point region.

Both stamps skip every band cell within `k + 2` of an endpoint of the wire
being routed (`lenpt_covers_terminal_of`, `:6392`): the *same-port
siblings*, wires that hang on the port at which a resonator reaches its
length, would otherwise be unroutable by construction. The band falls into
runs around the skipped cells, stamped one by one.

**Soft, not hard.** A wire is routed first *with* the constraint; if it ends
unrouted, it is routed again without it (`:7305-7320`: *"the constraint may
cost quality, never a wire"*). Only the second failure counts against the
round's failure budget. In the refinement the existing rollback to the
backup path does the same job: a reroute refused by the constraint leaves
the wire as it was (`:7719`).

**Where it applies.** The outer routing's phase 1 (both ring neighbours),
every relaxation level (the ripped wire and the neighbour on the other
side, unless that one was itself ripped), and the clearance refinement of
the same function (both ring neighbours, after `mark_obstacles` and before
`attach_corridor_grid`, "or the corridor build overwrites it all"). The
separate `run_final_routing_resonator_refinement_parallel` (`:7916`) does
**not** stamp it; the feedline passes do not either — the couplers stand by
then.

**What it reports.** At the end of the outer routing the points are recorded
(`FINAL_RES_LENPOINTS_`, `FINAL_RES_LENBANDS_`, drawn by `svg_overlay` with
the keep-out radius) and a `[LENPT]` report (`:7440-7600`) lists per
resonator the nearest other wires to its point, capped, with a histogram at
fixed edges 20 / 25 / 30 / 40 / 60 / 100 cells and the count of points whose
nearest wire is closer than `k` ("violations"). Fixed edges so that runs at
different `k`, and the `k = 0` baseline, compare directly. Thread safety is
by construction: a wire only touches neighbours inside the guard radius, on
which no second thread works.

**Why it exists** (the comment at `:6193-6215`): the coupler's head — the
arc and the lead-out — sticks out sideways beside the path at that point and
needs room; a foreign wire too close there makes the coupler and feedline
phase fail *"at a place nobody can clean up any more"*. That is the room our
room rules measured on 2026-10-04 and found missing after the fact (R3: a
plain wire within one cell of a lead at eight couplers; the lane pairs
leaning on a pad).


## What we have, and what differs

- **The place is the same figure.** `Driver::couplerPlace` (`FinalRouter.cpp`,
  grep it) orders the cells of a resonator's way by
  `|left − wanted|` with `left = overall − lengthAt[i]` from
  `routing::reconstructSegments` (the rendered length, measured from the
  target end exactly as the prototype measures) and
  `wanted = max(0, targetLength − anchorGap) · COUPLER_BIAS − leadLength()`.
  The band is therefore the cells with `left ∈ [wanted·(1 − pct),
  wanted·(1 + pct)]`, built from the very function the insertion uses, so
  the constraint protects where the pad will be centred — including the bias
  and the lead, which the prototype approximates with its undershoot of 3.
- **The outer routing lengthens resonators to `meander_length`**, which is
  `target_resonator_length` by default (`Tuning::meanderLength`), so a
  resonator's way is about the target length and its length point lies
  near its **start**, on the ring side — where the plain wires of the ring
  run beside it and where the pad has to fit. A resonator the Detail stage
  drew long has the point further in.
- **Our fences are discs around ways, not a buffer the search owns.**
  `attempt` closes `fence(wire, {&before, &after})` in phase 1 and the
  prototype's four-wire fence at every relaxation level (`:8129-8140`,
  `:8246-8261`), `refine` closes `fence(wire, {&before, &after})` (`:2509`).
  `closeRoomOf(wire, other, cells)` is the primitive: `alongDisc(cells,
  stencilFor(tuning_.clearance), …)` into `corridor_`, with the meeting
  exemption of `meetAt`. A stamp at radius `k` is the same call with
  `stencilFor(k)` over a sub-path — nothing new in kind.
- **No seed problem.** `seed` puts every wire down on the Detail stage's way
  before the first search, and `couplerPlace`-style marks can be read off
  any way the wire has, drawn or seeded; `reconstructSegments` needs a way
  of this grid's cells, which the seeded way is.
- **Our refinement is one function**, `refine`, used for the outer
  refinement (phase 2) and the feedline refinement (phase 5). The plan adds
  the stamp to the outer use; under the feedline constraints the couplers
  stand and the insertion is over, so it would protect nothing there.
- **The feedline pass redraws every ring wire again** (phase 4). Room won in
  the outer routing survives into the insertion (phase 3), which is the
  point; whether it survives phase 4 does not matter for the insertion, and
  the targeted repair is what mends phase 4.
- **Same-port siblings exist here too**: the conventional wires into a
  qubit's other ports end within the clearance of the resonator's target.
  `meetAt` already exempts the meeting of two wires whose terminals lie
  within the rule of each other; the stamp has to make the same exemption
  around the routed wire's own ends, as the prototype's `k + 2` guard does.
- **The prototype's `k = 40` is on a grid of the same pitch** (about ten
  layout units a cell, as ours), so 40 cells is about two clearances. Our
  geometry says what the pad needs: `couplerReach()` =
  `couplerLength + couplerHeight + 2·BEND_RADIUS + 1.5·clearance` ≈ 62 cells
  on 45q is the reach of the whole coupler; the room a neighbour has to leave
  beside the way at the length point is the pad's depth plus the clearance
  plus the turn, `couplerHeight + clearance + BEND_RADIUS`, and the pad may
  go to either side. That figure, not 40, is the default to start from, with
  40 as one arm of the sweep.


## The plan

Every piece behind a switch read through `envFlag`/`envWhole`/`envReal`,
default documented with the measured reasoning, named in a `settings:`
line; the switches off reproduce `artifacts/logs/base-ortho2` byte for byte
in the judged lines. Report in German, repository content in English.

### 1. The marks — `Driver::lengthMarksOf(wire)`

A `LengthMarks{lo, pt, hi, band}` per resonator, factored from
`couplerPlace`: `reconstructSegments` over the wire's current way (drawn or
seeded), `wanted` exactly as `couplerPlace` computes it, `left` per cell;
`pt` the cell nearest `wanted`, `band` every cell with
`left ∈ [wanted·(1 − pct), wanted·(1 + pct)]` in way order, consecutive
duplicates removed; `lo`/`hi` its ends. `pct` is `SCPD_LENPOINT_PCT`
(default 10, the prototype's). Computed on demand from the neighbour's
current way, never cached: the way moves at every reroute and every
meander, and the stamp is read at the moment of fencing, as the prototype
does ("kein geteilter Zustand"). Cost: one `reconstructSegments` per
neighbour per search — the same the insertion pays per `couplerPlace`.

### 2. The two stamps — `Driver::closeLengthBand`, `Driver::closeAlongside`

- `closeLengthBand(wire, other, k)`: `other` is a resonator being fenced;
  `alongDisc(marks.band, stencilFor(k), close)` into `corridor_`, skipping
  every band cell within `k + 2` of `wire.objective.source` or `.target`
  (the same-port guard), and respecting `meetAt` as `closeRoomOf` does.
- `closeAlongside(wire, other, k)`: `wire` is a resonator being routed;
  project `marks.lo` and `marks.hi` of `wire` onto `other.way` (nearest
  cell each), inflate the stretch of `other.way` between the two
  projections by `stencilFor(k)`, same guard.

Both are no-ops at `k = 0` and when the resonator has no marks (a way of
fewer than two cells). `k` is `SCPD_LENPOINT_K` with default
`couplerHeight + clearance + BEND_RADIUS` in cells (computed, said in the
settings line in cells); `=0` is off.

### 3. Where they are called

- **`attempt`, phase 1**: after `fence(wire, {&before, &after})` and before
  `constrainByFeedlines` — for each of `before`/`after` that is a resonator,
  `closeLengthBand`; if `wire` is a resonator, `closeAlongside` with each
  neighbour. Only while `!pass.feedlines`: under the feedline constraints
  the couplers stand (see above). The prototype's second pair of fences
  under the feedline rule does not apply here.
- **`attempt`, every relaxation level**: after the level's `fence(…)` calls
  and `fenceFixedPlaces`, for the wires that level fences (the ripped wire's
  far-side neighbour and `before`/`after` as the branch has them): the same
  two stamps. A ripped wire is not stamped — it is meant to be crossable,
  as the prototype says of `ripped_flag`.
- **`refine`**: after `fence(wire, {&before, &after})` and before
  `priceRoom()`, the same two stamps, only in the outer refinement
  (`!pass.feedlines`), under `SCPD_LENPOINT_REFINE` (default on: the
  prototype turned it back on because the refinement had been levelling the
  room the rounds had won).
- The `isLegal` early accept (`forceReroute` off) does not see the stamp;
  with `forceReroute` on (default) every wire is searched, so the stamp
  binds every round.

### 4. Soft, as the prototype has it — `SCPD_LENPOINT_FALLBACK`

In `attempt`, when the search with the stamp ends with no way after the
relaxation, run the whole attempt once more without the stamp (default on).
The simplest form: a flag `lengthPointActive_` the stamps read, `attempt`
run twice from `sweep`'s loop (`won = attempt(…) || (fallback &&
attempt(… without))`), the second run only when the first failed. The round
record counts a wire as failed only after the second run. `=0` lets the
constraint cost a wire, which is what the user's bridge check does on
purpose and which the sweep should measure here too. In `refine` the
fallback is the existing "kept its way".

### 5. The report — the length-point line

At the end of the outer routing (after `refine`, before the insertion) one
line per chip in the prototype's shape: `outer routing: LENPT k=N band=±P%:
M resonators, nearest other wire to the length point min X mean Y, below k
Z, histogram <20:a 20–25:b 25–30:c 30–40:d 40–60:e 60–100:f ≥100:g`, with
fixed edges so that arms compare, and at `-v 1` one line per resonator
naming its three nearest wires with distances, the same-port siblings left
out as the stamp leaves them out. The same line again at the end of the
feedline pass is the insertion's input as phase 4 leaves it. Written as a
`say` under `SCPD_LENPOINT_REPORT` (default on, report only), the way the
room rules' lines are.

### 6. Measurement

Arms over all eight chips, one at a time, `stop_after = "feedlines"` and
`repair_trials = 0` first, then with the targeted repair at its measured
budget:

| arm | switches |
|---|---|
| identity | `SCPD_LENPOINT_K=0` → `base-ortho2` byte for byte in the judged lines |
| default | `k` from the geometry, pct 10, fallback on, refine on |
| k 40 | the prototype's figure |
| pct 5 | the user's 2026-08-31 figure in the prototype's comment |
| hard | fallback off: the constraint may cost a wire |
| no refine | the stamp in the rounds only |

Read per arm: the LENPT line (min, mean, below-k, histogram) before the
insertion; the insertion's `==>` line (edges not drawn, angle), the four
`CHECK` lines, the room lines' R3 figures (plain wires within 0/5/10/19
cells of a pad or lead), the number of options per coupler; the outer
routing's `final`-style line after phase 2 (unrouted, open — the stamp can
only add fails there, and the arm must say how many); the feedline pass's
`bad` per chip; the seconds of the outer routing and of the insertion.
Acceptance: the outer routing loses no wire on any chip (fallback on), the
R3 near-lead count and the lane pairs leaning on a pad fall, `bad` after
the feedline pass does not rise on any chip, and the sum falls. A default is
set only by a value that holds alone and together on all eight.

### 7. Tests

- `test_final_router.cpp`: the length-point line is printed; a probe in the
  style of `SCPD_PROBE_ONLY_UNSETTLED` is not needed — the marks are a pure
  function of a way and can be unit-tested once `lengthMarksOf` is factored
  so that a test can hand it a way and a target length (a small
  `routing::` helper, `lengthBand(segments, wanted, pct)`, in the style of
  `RoomRules.hpp`'s router-free geometry, with tests in
  `test_room_rules.cpp`: the band of a straight way, the band of a way
  shorter than `wanted` collapses to the start, duplicates removed, the
  same-port guard).
- Identity: `SCPD_LENPOINT_K=0` on 4q / 9q / 17q against `base-ortho2`.

### 8. Risks and open questions

- **The outer routing may lose wires to the stamp** on the crowded chips; the
  fallback is what the prototype needed for that, and the hard arm measures
  what it costs here.
- **The band is wide on long resonators**: 10 % of 6000 units is 60 cells of
  way each side of the point, inflated by `k`; on a resonator the Detail
  stage drew far longer than the target the band sits deep in the chip
  where the ring is dense. `pct 5` is the arm for that.
- **The lengthening moves the point.** `lengthen` meanders a resonator after
  its search; the band read off the neighbour's way is the meandered one,
  which is right, but a resonator's own `closeAlongside` is computed from
  its pre-search way and its meander may put the point elsewhere. The
  prototype has the same approximation.
- **`meetAt` and the guard**: two exemptions around the ends must agree, or
  a wire into a qubit's second port is fenced away from the resonator's
  target. The guard of `k + 2` is the prototype's; ours should be the larger
  of that and the meeting radius `1.5 · clearance`.
- **Where the pad goes is not known at routing time** (either side of the
  way, eight orientations), so the stamp is a disc, not a half-plane; it
  asks twice the room the pad needs. The insertion's option count per
  coupler is the figure that says whether that was worth it.
- **Nothing in the feedline pass**: by then the couplers stand and the lead
  and pad are fenced themselves (`fenceFixed`, `bodies_`). The stamp there
  would only steal room.
