# Plan: feedline-room rules in the coupler insertion (hard exclusion)

Approved by the user on 2026-10-03. The task prompt that points here is
`prompt-coupler-room-rules.md`; the start prompt for a fresh session is
`prompt-coupler-room-start.md`.

For a fresh agent. Repository `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`,
branch `phase-4-routing-stages`, HEAD `63851cf` "feedline routing 7 fails
remain", tree clean (2026-10-03). The user commits per phase: leave all work in
the working tree, never commit. Every new rule goes behind an `SCPD_*` switch
(read once through `envFlag`/`envWhole`/`envReal`, `src/pipeline/FinalRouter.cpp:87-111`)
with a documented default and the measured reasoning in its comment, and is
printed in the insertion's `settings:` line. Length fails (short/long) are
explicitly not a target. Report to the user in German; repository content in
English. Line numbers below were verified at this HEAD; grep the identifiers
once code is added.

## Context

The Final stage routes the ring (outer routing + refinement), then
`Driver::insertCouplers` (`FinalRouter.cpp:2769`) places one CPW coupler per
resonator and routes the feedline chains through them — a prefix A* over
coupler options, `solveChainAStar` (`:5863`) — and the feedline pass then
re-routes every ring wire around the chains. Two facts drive this plan:

- The insertion routes its edges against artwork, pads, leads, resonator
  tails and launcher stubs, but **never against the ring wires**, and never
  asks whether the shape it leaves behind has room for them.
- The feedline pass re-routes **only the terminal edges** (`ringWithEdges`
  `:6711`); the edges between couplers keep the insertion's ways to the end
  of the stage. A pinch between two such edges can only be fixed here.

After the 2026-10-03 fixes (`SCPD_RESONATOR_STUB` off, `SCPD_FENCE_FIXED` on)
the pass leaves, one run at a time, no environment variable set:

| open | 45q | 57q | 69q | small chips |
|---|---|---|---|---|
| in the last round (the user's figure) | 2 (116, 65) | 1 (10) | 4 (176, 19, 9, f0) | 0 |
| at the end of the stage (pairs) | 6 (65/66, 116/117, 133/136) | 3 (9/10/11) | 9 (9/10/f1, 19/20, 176/177, 203/206) | 0 |

Four of the nine remaining conflicts are pinches the chain geometry makes at a
coupler (measured from `artifacts/<chip>/06-final.fb` and the `is open in the
room of` lines; one cell is 9.92 units, the clearance 19 cells, the rule 18.65):

- **45q 65/66** (ring `… 63 64(r) 65 66 67(r) …`): conflict at (2806,1332),
  22 cells from f22 **and** 22 from f23 — the in- and out-edge of coupler 64 —
  resonator 64's head 27 cells away. Both edges leave the pad and turn back
  the same way; the channel between them is ~44 cells where two plain wires
  need 3 × 19.
- **57q 9/10** (ring `… 7 8(r) 9 10 11(r) …`): at (1218,3610), f2 at 20 and
  f3 at 30, the edges of coupler 8; the same shape.
- **69q 9/10/f1** (ring `… 8 f0[F] 9(r) 10 11(r) …`): the insertion laid
  resonator 9's lead one cell from wire 10's way; 10 cannot leave once the
  lead is fenced, and 9 is dead on arrival in every round.
- **69q 19/20** (ring `… 18(r) f5[L] 19 20 f6[F] 21(r) …`): two plain wires
  between the terminal edges of two different chains (f6 at 41, f4 at 56, f5
  at 68, head of 18 at 34). Cross-chain; this step only measures it.

The other five (45q 116/117, 133/136; 57q 10/11; 69q 176/177, 203/206) have no
chain edge within 80 cells: border or lattice pinches, not the insertion's.

The prototype (`/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/FinalGrid.cpp`)
has no such rule. Its only trace is the commented-out `kSameAngleCorridorPenalty`
(`:18378-18425`): "two chain-adjacent couplers both landed on 135°, squeezing
out five plain feedline wires that needed to pass between them" — the same
failure, sketched as a penalty and abandoned. Its chain router keeps the two
edges of one coupler only 2 cells apart (`:18225-18234`); its bridge map puts
every conventional port between two resonators on one edge with no capacity
check (`CapacityGrid.cpp:7727`). Nothing to port; this is ours.

## Decisions taken with the user (2026-10-03)

1. **Hard exclusion, no cost term.** A refused option or edge does not exist
   for the search; nothing is added to `edgeCost`'s price.
2. **Stand-down ladder.** While a chain stays joinable the rules are hard.
   When no run of options joins a chain even with the second-dogleg options
   opened, the rules are lifted for that chain only, one at a time, each lift
   logged loudly. "All feedline edges drawn" stays a hard criterion.
   Precedent: `openOptionsOf` `:5559-5565`, "the guard stands down".
3. **Measure both crossing regimes.** Acceptance against today's defaults
   (`SCPD_ORTHO_CROSSING` off, `:829`). Every arm is also run with
   `SCPD_ORTHO_CROSSING=1` and reported; the default stays off until the user
   decides.
4. **In-chain rules first.** Cross-chain channels get a check line at the
   commit, modelled on `checkFeedlineRoom` (`:8129`); solving them is later.

## Acceptance

Per chip, defaults, one run at a time, all eight chips
(`.venv/bin/mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage final -v 1`):

- `==> coupler insertion: 0 feedline edges NOT drawn` (`:3197`), the three
  `CHECK` lines green, `0 unrouted` in the `final routing:` line.
- Open in the **last round** of the feedline pass ≤ 2 / 1 / 4 on 45q / 57q /
  69q (the last `feedline routing round N …: … open N | Fails: N` line and
  the `N failed: …` line after the pass), open at the **end** ≤ 6 / 3 / 9
  (`final routing: … open` and the `is open in the room of` lines), the five
  small chips at 0 by both figures with every edge drawn — any change on a
  small chip is a defect in a rule, not a result.
- Fails and insertion seconds are reported beside it; neither is the
  criterion. Today: fails 18 / 29 / 45, insertion 36 s / 61 s / 117 s, chains
  at the 10 s budget (`best in the time given`): 45q chain 6, 57q chain 4,
  69q chains 1 and 10 (1, 4 and 10 in the `-d` run).

## Where everything is (`src/pipeline/FinalRouter.cpp` unless stated)

| piece | where |
|---|---|
| `insertCouplers`: `approaches_` `:2794-2833`, `tailOwner_` `:2842-2865` (never read), `conventional_` `:2869-2875` (never read), couplers + `optionsOf` `:2878-2909` (a coupler with no option is dropped `:2889-2894`), chains `:2913-2964` (`chain->nodes` are ring slots, `couplerOfWire` maps wire key → coupler), `chainEdgePaths_`/`edgeWireOf_` `:2966-2972`, `settings:` `:2982-2998`, search `:3001-3010`, commit `:3013-3128` (keep-or-reroute `:3082-3088`), checks `:3132-3134`, summary `:3143-3155`, `==>` `:3197` | `:2769-3200` |
| `struct CouplerOption` (orientation = pad axis, offset, run, depth, way, anchor, centre, secondPort, secondStraight/Reverse, couplerOrientation, resonatorIn/Out, arcEnd, arc, body, in, out, cost, `guarded` — priced, read nowhere, `:5224`), `Coupler`, `Waypoint`, `Edge` | `:2602-2659`, `:2662`, `:2686`, `:2696` |
| `optionsOf`: 8 offsets × 2 ports × 3 jog variants, each walking the `couplerPlace` (`:3444`) places nearest the target length until one fits (`:3239-3262`); a `refuse` in `makeOption` skips the **place**, the walk moves off-length, `couplerMaxShortfall` (`:160`, 0.30) is the backstop | `:3205-3284` |
| `makeOption`, `refuse` `:3591`, the `free()` lambda `:3794-3811` (hard on `scene_.blocked` and `approaches_`; `bodies_`, other owners' copper and `field_.guarded` only priced), applied to the body `:3823`, the far-edge run `:3895`, the lead `:3991`; other refusals `:3606 :3637 :3653 :3703 :3752 :3778 :3791 :3824 :3830 :3851 :3881 :3890 :3896 :3946 :3982 :3985 :3992 :4005 :4025` | `:3586-4029` |
| `openOptionsOf` (jog gate, body-on-feedline filter, the stand-down) | `:5530-5570` |
| `optimizeChainsPrefix` (one pass; an unjoined chain is "left as it stands" `:6561-6575`; owns a non-const `wires`) | `:6554-6608` |
| `solveChainAStar` (two goes `:5934`, `laid` cleared per go `:5903`, jogs opened at the wall `:6098-6118`) and its `step` lambda (fence from `laid` `:5984-6002`, `CHAIN_FRESH_EDGES = 2` `:4053`, `edgeCost` `:6020`, end-pair reorder `:6043-6076`, `return real` on UNREACHABLE `:6078-6083`, `laid[key] = …` `:6084-6086`) | `:5863-6167`, `:5957-6088` |
| `routing::solveChainAStar` (an unreachable step is skipped `:167-170`) | `src/routing/ChainSearch.cpp:75-242` |
| `edgeCost` (memo `edgeMemo_` keyed `chain<<56 | from<<48 | ahead<<32 | optionFrom<<16 | optionTo` `:9929-9938`; `TRELLIS_UNREACHABLE` for an empty way `:5458`, else `10000 × angleCostOf`) | `:5410-5462` |
| `routeEdge` (its way **includes** the forced stubs: `DubinsRouter::straightStub` tags them with the straight primitive, `src/routing/DubinsRouter.cpp:794-811`), `corridorOfEdge` (artwork, `EDGE_BOX_MARGIN = 250`, `fenceCommittedEdges` `:4067`, `prefixFence_`, own leads/pads, launcher stubs, slots `max(terminalSlot(), run + BEND_RADIUS)` `:4521-4524`), `runsOfEdge` `:2729` | `:4865-4960`, `:4276-4549` |
| commit `applyOption` `:6615`, `edgeWayStillOpen` `:4852`, `stateFaults` `:6283`, checks `checkFeedlineRoom` `:8129` / `checkCouplerCrossings` `:8077` / `checkResonatorCrossings` `:8030`, `waysCross` `:7977` | |
| `assignBridges`: the wires that must cross each non-terminal edge = the non-resonator ring wires strictly between the two couplers' resonators, walking the ring **forward**; runs in phase 4 (`:10382`); ring order = `assignment.connections` order = `wires` order (`wiresOf` `:10033`), `outer` = every non-inner key `:10281` | `:6734-6764` |
| `Wire` (resonator, feedline, terminal, bridged, couplerAtSource/Target, fixed, arc, startStub, endStub, slot, key, objective, way, drawn) | `:1431-1509` |
| `Field` (`owner` `:1315`, `guarded` `:1309`), `conflictsIn` `:7903`, `meetAt` `:7774`, `couldMeet` `:8485`, `sharedCoupler` `:7809`, `couplerReach()` `:7889` (≈ 62 cells on 45q), `stencilFor` `:9075`, `alongDisc` `:8521`, `closeRoomOf` `:8454`, `fence` `:8419` | |
| `routing::reconstructSegments` → `PathSegment{heading, primitive, straight, cells, lengthAt}`; `MovePrimitives::isStraight`; `BEND_RADIUS = 5` `:119` | `src/routing/PathGeometry.cpp:27-73`, `include/mqt-scpd/routing/Path.hpp:61-73`, `Primitives.hpp:90` |
| the crossing rule: `CrossingConstraints::build` — a cell is **straight** when it steps to the next distinct cell along its own heading (`:56-71`); straight cells carry their heading over a square of `expandRadius = CROSSING_REACH = 10` (`:9959`); bend cells and the first/last 10 cells are `CURVE_ZONE` (`:97-105`); `rebuildCrossingRule` `:6811`; `constrainByFeedlines` `:6842` (every drawn edge fenced except `wire.bridged` `:6850`, 5× price band); `crossesAFeedline` `:2305`; DRC `FeedlineOrthogonality` `src/drc/Rules.cpp:273-330` | `src/routing/CrossingConstraints.cpp:23-110` |
| `Span`/`edgeBox`/`reachOf`, `forgetEdgesNear` (the trellis's purge; not usable here) | `:6190-6260` |
| the repair (`repair` `:7018`, `turnCoupler` `:6951` → `routeEdge` `:6974`, `unsettledBy` `:6989`); `repair_trials = 0` in all eight configs | |
| libraries are globbed: a new `src/routing/RoomRules.cpp` is picked up by `cmake/AddMQTScpdLibrary.cmake:59-61`, a new `test/routing/test_room_rules.cpp` by `test/routing/CMakeLists.txt`; export with `MQT_SCPD_ROUTING_EXPORT` as `CrossingConstraints.hpp:40` | |
| figures: `coupler_length = 200`, `coupler_height = 26` units (`schemas/config.fbs:273-274`) → pad run 21 / depth 3 cells on 45q; 69q's config has `refinement_rounds = 10`, the other seven 5 (do not change it mid-sweep) | |

There is no path-to-path distance helper; distances are asked through the
field or the stencils. The geometry below goes into a small router-free
library so it can be unit-tested (`Driver` is in an anonymous namespace).

## The rules

Shared switches (Driver statics beside `chainKeepWays` `:4820`):

| switch | type | default | meaning |
|---|---|---|---|
| `SCPD_ROOM_PITCH` | cells | `clearance + 1` (20) | centre-to-centre pitch of parallel wires; sweep 19 / 20 |
| `SCPD_ROOM_MARGIN` | cells | `CROSSING_REACH` (10) | dead length at both ends of a straight run; sweep 0 / 10 / 15 |

Every refusal prints a `[Coupler Insertion]` line through `tell` (-v 1) with
its figures, capped at 20 per chain and rule, and feeds a counter.

### R0 — the crossers per edge (no switch; the input to R1, R2, R4)

Hoist the walk of `assignBridges` into a new `Driver::bridgersOf(wires, chain, from)`
and precompute `bridgers_[chain][at]` right after the chains are built
(`:2964`): the `!resonator && feasible` ring wires whose slot lies strictly
between the slots of the two couplers' resonators, walking forward from the
`from` resonator (exactly `:6752-6764`; ring size `assignment.connections.size()`).
Terminal edges have none. **Direction guard:** compute both arcs, keep the
shorter, and `tell` when they differ (the ring lines show chain nodes in
increasing ring order today, so the forward arc is the short one; the guard is
cheap insurance). Keep the keys, not only the count. One `tell` per edge:
`chain c edge k->k+1: n plain wires must cross it (ids)`. In phase 4, after
`assignBridges` (`:10382`), one check line: `N wires whose bridged edge differs
from the insertion's count` — must be 0 on all eight chips.

### R1 — crossing capacity of an edge (`SCPD_ROOM_CROSSING`, default decided by the sweep, start off)

For an edge's routed way (any pair of options), the crossings it can host:

1. Drop the first `slotFrom` and last `slotTo` cells of the way,
   `slot = max(terminalSlot(), run + BEND_RADIUS)` — the figures
   `corridorOfEdge` `:4521-4524` and `checkFeedlineRoom` `:8161-8173` use.
   These are the pad runs and the stubs the way carries; nothing crosses
   there, and the slot (≥ 20) also covers the 10-cell pin zones.
2. A cell is **straight** by the crossing rule's own definition
   (`CrossingConstraints.cpp:56-71`): it steps to the next distinct cell along
   its own heading. Factor that test into the new library (`straightCells`)
   and let `CrossingConstraints::build` call it, so R1 counts exactly the
   straights the orthogonal rule will accept.
3. Group consecutive straight cells into runs of length `L`:
   `lanes(L) = (L − 1 − 2·margin) < 0 ? 0 : 1 + ⌊(L − 1 − 2·margin) / pitch⌋`
   (margin 10, pitch 20: L = 21 → 1, 41 → 2, 61 → 3).
4. `capacity = Σ lanes`; **refuse when `capacity < |bridgers_[chain][at]|`.**

**Attach** in `edgeCost` after the way is known (`:5455`), whether it came from
the memo or from `routeEdge`, and **evaluate on every call**: the verdict is a
pure function of the way, O(length), so the memo keeps the real way and no
purge is needed when the rule is lifted. Return `TRELLIS_UNREACHABLE`;
`*wayOut` still carries the way. Both `edgeCost` calls of the end-pair
reorder (`:6053`, `:6061`) pass through it. The commit (`:3063-3128`) and
`turnCoupler` use `routeEdge`, never `edgeCost`, so a chosen way is never
re-refused; count committed edges below capacity for the summary only.
Log: `chain c edge k->k+1 options a/b: n plain wires must cross it (ids); its
straights carry m lane(s) over r run(s) of L1 L2 … cells (pitch p, margin g)
— refused (R1)`.

**Flag.** With `SCPD_ORTHO_CROSSING` off, phase 4 lets a plain wire cross its
bridged edge anywhere (`constrainByFeedlines` `:6850`, `conflictsIn` and the
DRC exempt the pair). R1 then models a rule the pass does not apply; expect
its gain under `SCPD_ORTHO_CROSSING=1`, and sweep `margin = 0` under ortho-off.
If ortho-off shows no gain, R1 stays off by default until the crossing rule
comes back.

### R2 — the channel at a coupler (`SCPD_ROOM_CHANNEL`, default decided by the sweep; `SCPD_ROOM_CHANNEL_REACH` = T cells, default 80, sweep 40 / 80 / 120; `SCPD_ROOM_CHANNEL_COUNT` 1 = current ways (default) | 2 = ring count)

**When.** In `step` after the reorder block (`:6076`) and before `laid[...] =`
(`:6084`), whenever `at ≥ 1`, `!points[at].fixed`, `real != UNREACHABLE`, R2
on and not lifted for this chain. Coupler `c = points[at].coupler`, option
`opt = couplers_[c].options[open[at][prefix[at]]]` (read the geometry off the
option; do not touch `chosen`). In-edge way: `before.empty() ? *wayOfEdge(at−1) : before`
(the reorder may have replaced it); out-edge way: `way`.

**Measured.**
1. In-arm: walk the in-edge backwards from its last cell, skip `opt.run + 1`
   cells (the collinear pad run — the in-edge's `endStub`), take the next `T`
   cells `a_1 … a_T`. Out-arm: skip the first `opt.run + 1` cells of the
   out-edge, take up to `2T`.
2. `gap = min_i min_j |a_i − b_j|` (Euclidean, cells), and the pair `(a*, b*)`
   where it occurs; ~80 × 300 distances per step, microseconds beside one
   edge search. If every distance grows with the index the arms diverge and
   the rule is silent.
3. `m`: for each `a_i` walk the supercover line to its nearest `b_j`, read
   `field_.owner` over its cells, count distinct owners that are plain ring
   wires (`conventional_[owner] == 1`), excluding `couplers_[c].wire`; add 1
   if any cell of `opt.way` past `opt.arc.size()` lies on one of those lines
   (the resonator's own continuation in the channel). Mode 2 (if mode 1
   reports `m = 0` at the known pinches): `m = |bridgers(at)| + |bridgers(at−1)|`
   when the arms lie on the same side, else mode 1.
4. Same-side flag, for the log and the calibration: `across = headingVector(turned(opt.orientation, −2))`
   (the far-edge side, `:3682-3683`); the arms are on the same side when
   `sign(dot(a_K − opt.centre, across)) == sign(dot(b_K − opt.centre, across))`
   for `K = min(T, 30)` cells past each stub — the U the data describes.

**Refuse when `m ≥ 1` and `gap < (m + 1) · pitch`** (45q coupler 64: gap 44,
m = 2 → needs 60; 57q coupler 8: ~50 against 60). Return `TRELLIS_UNREACHABLE`
from `step` without inserting into `laid`. Not memoised (it depends on the
prefix); the edge memo entry stays valid. Prefix search only (the trellis and
the greedy never see the pair under its predecessor); say so in the comment.
Log: `chain c coupler 'w' option o: channel between edge k−1 and edge k narrows
to g cells at (x,y), d cells past the pad, same side; m wires in it (ids) need
(m+1)·p — refused (R2)`.

### R3 — the option's own room (`SCPD_ROOM_OPTION`, default decided by the sweep; `SCPD_ROOM_OPTION_REACH` = R cells, sweep 0 / 5 / 10 / 19; `SCPD_ROOM_OPTION_MIN` = 8; `SCPD_ROOM_OPTION_TAILS` default off)

In `makeOption`'s `free()` (`:3794-3811`), before the pricing lines, with a
`bool roomRule = true` parameter threaded through `optionsOf(wire, roomRule)`:
`plainWithin(cell, R)` = a cell owned by a plain ring wire other than the
resonator (`field_.owner != NO_OWNER && != resonator.key && conventional_[owner]`),
for `R > 0` over `stencilFor(R).full` around the cell (for `R = 19` pre-test
`field_.guarded`); refuse with `blocker = "a plain wire's room"`. Under
`SCPD_ROOM_OPTION_TAILS` also `tailWithin(cell, R)` over `tailOwner_`
(`:2842-2865`), finally giving it a reader: `blocker = "another resonator's tail"`.
`free()` already guards the body, the far-edge run and the lead, and the
`refuse(...)` strings print `blocker`.

What it does: the refusal skips **this place for this orientation**, the walk
goes on to the next place by length mismatch, bounded by the overshoot
refusal (`:3636`) and the 30 % shortfall rule (`:3650-3657`); so R3 trades room
for length. **Build-time stand-down:** at `:2888`, if a coupler ends with fewer
than `SCPD_ROOM_OPTION_MIN` options under the rule, rebuild it with
`optionsOf(wire, false)`, say so (`coupler for resonator w: n options under the
room rule, below 8 — the rule stands down for it`), count it. Count the
couplers that moved more than 100 units off the figure (`sayCoupler` `:6675-6702`
prints the spliced length). 69q resonator 9 is the case it is for; expect the
option count per coupler to fall steeply above R = 10, and the small chips to
react first (the shortfall history: 17q lost two edges at 10 %).

### R4 — report only: `checkChainChannels` (`SCPD_ROOM_REPORT`, default on)

Modelled on `checkFeedlineRoom` `:8129-8215`, called after `:3134`. Walk `outer`
in ring order; for every maximal run of plain wires between two resonators
whose couplers belong to different chains A and B, take the last edge of A
and the first edge of B (`edgeWireOf_`), measure the gap between the two
committed ways (R2's helper, both pad runs excluded, T = whole way), `m` = the
plain wires in the run; also re-measure every same-chain coupler channel on
the committed ways. One `say` line: `coupler insertion: CHECK chain channels —
k of n channels between chains tighter than (m+1)·pitch, j of c coupler
channels tighter: chain 2 f5 / chain 3 f6: 2 wires (19 20), gap 41 cells at
(2274,4122) against 60; …`. 69q 19/20 must appear. Nothing is refused.

## The stand-down ladder (`SCPD_ROOM_LADDER`, default on; `=0` leaves an unjoined chain as it stands, for measuring how often the ladder is needed)

Lives in `optimizeChainsPrefix` (`:6556-6575`, which owns a non-const `wires`
— the R3 rebuild needs it; `solveChainAStar` takes `const`):

| rung | lifted for this chain | notes |
|---|---|---|
| 0 | nothing (plain options, then the jogs at the wall — the existing two goes) | |
| 1 | R2 | |
| 2 | R1 | no purge needed (R1 is re-evaluated per call) |
| 3 | R3 | rebuild this chain's couplers' options with `roomRule = false`, reset their `chosen` to 0, purge this chain's memo entries (`std::erase_if(edgeMemo_, key >> 56 == chain)`; the option indices change) |

Per rung: restore the chain's couplers' `jogsUnlocked` to the snapshot taken
before rung 0, so that "lifted" reproduces today's search exactly; run
`solveChainAStar`; climb on both verdicts (`no run of options joins the
chain` and `out of time …`) and say which: `[Coupler Insertion] chain c: <verdict>
— the <channel|crossing-capacity|own-room> rule stands down for this chain
(n steps routed, attempt k)`. State: `liftedR1_/liftedR2_/liftedR3_` vectors
sized `chains_.size()`, reset at `:2966`; nothing else reads them (not the
commit, not `stateFaults`), so a lift cannot leak; `laid` is cleared inside
`solveChainAStar` already. Assert no coupler is in two chains (`:2941-2955`).
Each rung restarts the 10 s budget per go, so a hopeless chain may spend up to
80 s; the chain's result line (`:6587-6595`) says the attempt count. After rung
3 a chain is left as it stands, as today.

## Logging and the `settings:` line

- `struct RoomStats { r1Asked, r1Refused, r2Asked, r2Refused, r3Refused (places), r3StoodDown (couplers), r3Moved (couplers > 100 units off), liftedR1/R2/R3 (chain ids), committedBelowCapacity, tightChannelsAfterCommit }`,
  a Driver member reset at the top of `insertCouplers`.
- One `say` line before the `==>` line (`:3197`): `coupler insertion: room rules
  — R1 refused a/b edges, R2 refused c/d channels, R3 refused e places (f
  couplers built without it, g moved > 100 units); stood down on chains
  R2[…] R1[…] R3[…]; committed: h edges below capacity, i channels tighter
  than (m+1)·pitch`.
- Extend the `settings:` line (`:2982-2998`) with every new switch and its
  origin, as the five existing ones (`from(name)` `:2983-2985`).
- Every switch comment carries the default, the measured reasoning and the
  sweep table, as `couplerMaxShortfall` (`:137-165`) does.

## Implementation steps

Each step ends with: `cmake --build --preset release` (the test tree),
`cmake --build build/cp311-abi3-macosx_15_0_arm64/Release --parallel`, the
install below, then 4q / 9q / 17q with the new switch **off**, diffed against
the baseline after stripping timestamps
(`sed -E 's/^\[final\] +[0-9.]+s  //; s/, [0-9.]+s$//'`): the `==>` line, the
three `CHECK` lines, every `chain N:` line and the `feedline routing` /
`final routing` lines must be byte-identical.

1. **Baseline logs.** Run all eight chips at defaults into a log directory
   outside `artifacts/<chip>` (e.g. `artifacts/logs/base/`), and once more
   with `SCPD_ORTHO_CROSSING=1`. These are the comparison set for every diff.
2. **Geometry library**, router-free and unit-tested:
   `include/mqt-scpd/routing/RoomRules.hpp` + `src/routing/RoomRules.cpp` with
   `straightCells(path)`, `crossingCapacity(path, skipHead, skipTail, pitch, margin) → {lanes, runs}`,
   `channelBetween(a, aSkipTail, b, bSkipHead, reach) → {gap, at, bt}`,
   `supercoverLine(p, q, visit)`, `sameSide(...)`; tests in
   `test/routing/test_room_rules.cpp`. Let `CrossingConstraints::build` call
   `straightCells` (behaviour-preserving; the orthogonality tests prove it).
3. **R0 and calibration, nothing live.** `bridgersOf`/`bridgers_` with its
   `tell` lines, `RoomStats`, `settings:` extension, summary line, the lifted
   flags. Then compute R1's capacity and R2's channel **after the search, at
   the chosen options**, printing one line per edge (`crossers n, capacity m`)
   and per coupler (`channel g, inside m, needs (m+1)·p, same side?`) with the
   verdict each would give. Run 45q, 57q, 69q and the small chips: the `==>`
   and final lines must be identical to the baseline; the four known cases
   (45q coupler 64 / f22–f23, 57q coupler 8 / f2–f3, 69q coupler 9 / wire 10,
   69q f5/f6 via R4) must be flagged; count how many healthy couplers and
   edges are flagged too, and tune `reach`, `pitch`, `margin` on these lines
   **before** any rule is live — three runs instead of a sweep.
4. **R1** in `edgeCost` + the R4 check line + the phase-4 bridgers check.
5. **R2** in `step`.
6. **R3** in `makeOption` / `optionsOf` + the build-time stand-down.
7. **The ladder** in `optimizeChainsPrefix`. Prove it with `SCPD_ROOM_PITCH=200`
   on 4q: stand-down lines appear and the run ends byte-identical to the
   baseline ("lifted" = today).
8. Optional: R1 inside `turnCoupler` (the repair is off in every config).
9. **The sweep**, defaults, switch comments, handover updates.
10. **Tests and ctest.**

Build and install after every source change, never while a run is going:

```bash
cmake --build build/cp311-abi3-macosx_15_0_arm64/Release --parallel
cp build/cp311-abi3-macosx_15_0_arm64/Release/bindings/pyscpd.abi3.so \
   .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so
codesign -s - --force .venv/lib/python3.13/site-packages/mqt/scpd/pyscpd.abi3.so
```

## Measurement protocol

- One chip: `cp benchmarks/<chip>/config.toml artifacts/<chip>/config.toml &&
  <ENV> .venv/bin/mqt-scpd plan -c benchmarks/<chip>/config.toml -o artifacts/<chip> --stage final -v 1 2>&1 | tee artifacts/logs/<arm>/<chip>.log`.
  Sequentially, all eight, no `-d`, no `-v 2`. Before each large chip check
  Spotlight: `ps -A -o %cpu,comm -r | awk 'NR>1 && ($2 ~ /mds_stores|mediaanalysisd/) {s+=$1} END {print s"%"}'`;
  measure CPU time beside wall time. One arm ≈ 25 min (45q 3 min, 57q 4.6,
  69q 9.5, the rest under 6 together).
- Arms: `base` (all off; must reproduce 2/1/4 and 6/3/9); R3 alone at
  R = 0 / 5 / 10 / 19; R1 alone at (pitch, margin) = (20,10), (20,0), (19,10);
  R2 alone at T = 40 / 80 / 120; all three together with the ladder at the
  chosen values; then `base`, each rule at its chosen value and all-together
  with `SCPD_ORTHO_CROSSING=1`. For the final table re-run 45q / 57q / 69q with
  `SCPD_CHAIN_ASTAR_SECONDS=0` (chains at the budget vary run to run), or
  twice at 10 s and report both.
- Read per log: `==> coupler insertion` (NOT drawn, angle, seconds); the three
  `CHECK` lines and `CHECK chain channels`; `N of N chains settled …, K edges
  would not survive`; per chain `optimum | best found … | best in the time
  given` and every stand-down line; the `room rules` summary; the last
  `feedline routing round` line and `N failed:`; `final routing: … open` and
  `is open in the room of`; the `coupler for resonator … options` lines and
  the resonator spans from `sayCoupler`.
- Defaults: a rule goes on by default only if it passes the acceptance alone
  and together on all eight chips; a parameter takes the value with the
  fewest open in the last round over the three large chips, ties broken by
  end-open, then by the fewest refusals. `SCPD_ORTHO_CROSSING` stays off; its
  tables are reported.
- Handover updates: `handover-feedline-routing.md` (switch table, *Where it
  stands* rows per arm, *What is left, by cause*, traps: option renumbering,
  ladder budget); `handover-cpw-coupler-insertion.md` (a section "The room
  rules": definitions, formulas, attachment points, the only-terminal-edges
  fact; *Where the pieces are* rows for `bridgersOf`, `RoomRules`,
  `checkChainChannels`, `RoomStats`).

## Risks and pitfalls

- **False refusals on the small chips.** 4q has one chain of five edges with
  one to three crossers each; short straights would refuse everything and
  the ladder would lift it loudly. Step 3's calibration lines and the small
  chips' rows per arm (must equal `base`) settle it.
- **R1 under ortho-off** models a rule the pass does not apply (see the flag).
- **R0 direction.** A chain running against ring order would count nearly
  the whole ring; the shorter-arc guard and the phase-4 check line must
  report 0 differences on all eight chips.
- **R2's `m` from current ways** over- or under-predicts who ends in the
  channel (the relaxation moves wires by up to `reach`); log `m` and the ids
  at every refusal; switch to count mode 2 if the known pinches report 0.
- **The option walk moves off-length under R3**; report the spans and the
  `r3Moved` counter; 17q and 4q react first.
- **Joinability and budget.** Refusing edges shrinks the prefix search; a
  chain at the 10 s budget may fall to the ladder; count rungs; consider
  `SCPD_CHAIN_ASTAR_SECONDS=30` as a measured arm if rungs are reached only by
  the clock.
- **Memo.** R1 is re-evaluated per call (no empty ways stored); the R3 rebuild
  renumbers options and must purge the chain's memo; `forgetEdgesNear` does
  not help here.
- **The end-pair reorder**: evaluate R2 only after it, on `before`/`way`.
- **Runtime.** R2 is O(T × |out-arm|) per step and R1 O(|way|) per call —
  negligible beside an edge search; R3 multiplies `couplerPlace` walks (the
  option build is ~10 s on 45q, ~35 s on 69q today, the gap between the
  `couplers sit inside` and `settings:` lines). Watch the `==>` seconds.
- **Pitch rounding**: the rule is 18.65 cells; pitch 20 is conservative by
  1–2 cells per lane; the (19,10) arm measures what that costs.
- **Housekeeping**: Spotlight on `artifacts/` (debug dirs 16 MB–9.5 GB per
  chip; remove `artifacts/<chip>/debug` before any `-d` run, the CLI does not
  clear it); copy the benchmark config into `artifacts/<chip>/` every time;
  `-v 2` quadruples a 69q run; 69q keeps `refinement_rounds = 10`; the `in
  the way` line never fences the second pair of neighbours — read the `dead
  on arrival` line first when a wire fails everywhere; an editor with
  `FinalRouter.cpp` open can write a stale buffer back.

## Tests and verification

- `test/routing/test_room_rules.cpp` (built like `test_coupler_insertion.cpp`
  with `MovePrimitives(5)`/`buildDogleg`): a 61-cell straight with margin 10
  and pitch 20 → 3 lanes; two 30-cell straights around a quarter turn → 2,
  with margin 15 → 0; `skipHead`/`skipTail` exclude the stubs; an arc alone
  → 0; two parallel straights 44 apart → gap 44 at the right indices; a U
  from two doglegs → the narrowest gap at the closed end; `sameSide` signs;
  supercover lines hit every cell between two points. In
  `test/routing/test_chain_search.cpp` add
  `AStepRefusedForItsPredecessorPairIsRoutedAround` (a `conflict` reading
  `prefix[at−1], prefix[at], to` — R2's shape) beside
  `APrefixThatBlocksItselfCostsAnother`.
- Ladder proof: `SCPD_ROOM_PITCH=200` on 4q (stand-down lines, then a run
  byte-identical to the baseline).
- End to end: the sweep; the `base` arm byte-identical to today's logs in the
  `==>`, `CHECK` and final lines.
- `cmake --build --preset release && ctest --test-dir build/release -E 'EveryChip/(Final|SpacedFinal)' -j 6`
  (231 s today; the excluded suites run the full stage for hours). Four
  failures pre-exist and are not this work's:
  `FinalRouter.StartsOnTheDetailWaysAndReportsItsFails`,
  `EveryChip/BenchmarkDetail.DrawsCopperTheRulesAllow/9q` and `/57q`,
  `EveryChip/SpacedDetail.KeepTheWireSpacing/9q`.

## What to report to the user

Per arm the two open figures per chip (last round first), edges drawn, the
checks, the refusal counters and the rungs used, insertion seconds; which
defaults were set and why; what the four known pinches became; what remains
(the border and lattice pairs, the cross-chain channels from R4) — in German,
tables for numbers, no commits.
