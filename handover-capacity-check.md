# The capacity check in the coupler insertion, and the coupler page

Written for whoever takes the capacity check further. It covers the work of
2026-10-08:

- the two changes the user made to the analysis;
- the measurements of what a check inside the insertion costs;
- the check after every settled chain, and why it finds nothing on 17q;
- the debugging page that moves couplers by hand.

Read [handover-bottleneck-analysis.md](handover-bottleneck-analysis.md) first:
it describes the walls, the port runs with their slots, the cuts, the chambers,
the graph and the page this builds on. The full measurements and the reasoning
behind the recommendation are in
[plan-capacity-check-in-insertion.md](plan-capacity-check-in-insertion.md).

## State

**2026-10-09: the check is a rule of the chain search now.** How the coupler
insertion uses it — the chip of a step, the demands, the length rule, the
refusal, the fallback, the clock, the log and the measurements on every chip —
is in *How the insertion uses the capacity check* of
[handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md).
Sections 7 and 8 below are the history and the details.

- Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`, HEAD `712cf7a`.
- **Everything below is uncommitted.** The user commits per phase.
- Files of this work:
  - `src/pipeline/FinalRouter.cpp`, `src/pipeline/CapacityFlow.cpp`;
  - `include/mqt-scpd/pipeline/CapacityFlow.hpp`,
    `include/mqt-scpd/pipeline/CouplerSession.hpp` (new);
  - `bindings/bindings.cpp`, `python/mqt/scpd/pyscpd.pyi` (regenerated);
  - `python/mqt/scpd/capacityview.py`, `capacityview.html`, `cli.py`, `run.py`;
  - `test/pipeline/test_capacity_flow.cpp`,
    `test/pipeline/test_final_router.cpp`,
    `test/python/unit/test_capacityview.py`;
  - `src/grid/DistanceTransform.cpp` (linear),
    `test/grid/test_distance_transform.cpp`;
  - `include/mqt-scpd/routing/CrossingConstraints.hpp`,
    `src/routing/CrossingConstraints.cpp` (`allowedArriving`),
    `src/drc/Rules.cpp`, `test/drc/test_rules.cpp`;
  - `CHANGELOG.md`, `handover-bottleneck-analysis.md`,
    `plan-capacity-check-in-insertion.md`.
- Not in the package (`artifacts/` is ignored by git): the measurement programs
  and logs in `artifacts/logs/cap-rt/` and `artifacts/logs/chain-check/`, the
  run directories `artifacts/dev/17q-chk`, `17q-cap`, `69q-cap`, and the
  overlays `artifacts/dev/py-cg` and `py-cap`.

## 1. Two changes to the analysis (user, 2026-10-08)

Both are the default now. Each has a switch back.

### Only the cuts the insertion has a hand in

A bottleneck cuts the chambers only when one of its ends lies on a feedline
edge, a coupler pad or a resonator's lead (`couplerCuts`, after
`findBottlenecks`). Every other bottleneck is left out, and the chambers on
either side of it are one.

- A lead is a port run with `Wall::lead` set: `markRun` sets it on the two walls
  of a resonator's source run. In the log a lead is still a wall of the kind
  `stub`, so `artwork–stub` and `stub–stub` cuts remain in the count; their stub
  end is a lead.
- The `BOTTLENECKS` line says how many cuts were left out.
- `SCPD_BOTTLENECK_ALL=1` keeps every cut.

17q: 149 of 214 cuts stay, 255 chambers instead of 321. 69q: 197 of 743 stay.

### The wires routed one after another (user, 2026-10-08 evening)

`pipeline::checkInTurn` (`CapacityFlow.cpp`) replaces the check of each wire
alone. It routes the outer wires one after another, in wire order, and counts
the load on every edge:

- Each wire takes its way with the fewest edges over the edges open to it that
  still take a wire. An edge takes as many wires as its capacity, less the ways
  routed before it that pass it.
- A wire that finds no such way takes its way with the fewest edges whatever the
  capacities. The edges it overfills have overflow.
- A plain wire crosses exactly the feedline edges it is prescribed, each once,
  and no other (`FlowDemand::crossings`). Its prescribed edges are the drawn
  edges between two couplers that it bridges (`bridgers_`). The search carries
  the prescribed edges crossed so far in its state; a step back into the chamber
  an edge was entered from is no step. A prescribed edge with no stretch leaves
  the wire no way. Resonators cross nothing.
- The answer is a `FlowCheck`, so the page and the JSON show loads and overflow
  as for the flow. The JSON names each wire's prescribed edges (`crosses`), and
  the page shows them as "has to cross".

The verdict is `SAT` when every wire has a way and no edge has overflow.
Otherwise the line reads
`UNSAT — … short by N wires on M edges, routed one after another in wire order`
and names each overfilled edge with the wires through it; wires with no way at
all are named apart.

- Overflow says only that this order does not fit. Another order may.
- `SCPD_CAPACITY_FLOW=1` restores the integer flow `checkCapacity` (HiGHS). The
  flow does not hold a wire to its prescribed edges.

17q, both margins 5, full run (`artifacts/logs/in-turn/17q.log`): UNSAT, short
by 6 wires on 6 edges. Four are a wire leaving its own launcher stub through a
gap to a feedline that holds no wire: 2 (g143, f18), 9 (g49, f0), 38 (g33, f11),
56 (g74, f17). Two are the launcher row in front of f8 and f9: 29 passes in
front of 31's stub (g2, g1) before 31 is routed, and then 31 and 32 overfill g1
and g80.

### Port runs as two thin walls (user, 2026-10-08 evening)

A port run is no longer a solid band. `markRun` draws only the edge of the band
of half the clearance to either side of the run's middle line: two walls one
cell thick, 4-connected, where the band's sides lay, and the band's end at the
port. The end at the terminal is open. The free cells between the walls are the
slot (`Walls::slots`); `SLOT_DEPTH` is gone. The outer edge of a run lies where
it lay, so a gap between two runs has the same length as before.

- The cuts in front of a terminal now end at the two wall ends, not on the
  band's end face beside a 3-cell slot. All 76 cuts between a launcher run and a
  feedline end within half a cell of a wall end; before, 43 ended on the face.
- No cut joins the two walls of one run: its line would cross the slot, which
  `findBottlenecks` refuses. So `couplerCuts` needs no rule for it.
- The end at the port must be closed. Open, the slot joined the room behind the
  run through the row of free cells between each launcher run and the border: 85
  cuts had a way round them, and a terminal lay in the chamber behind its run
  rather than the one in front of it.

17q, both margins 5 (`artifacts/logs/thin-stubs/17q.log`):

| Line                   | Solid band                    | Two thin walls                |
| ---------------------- | ----------------------------- | ----------------------------- |
| `BOTTLENECKS`          | 144 kept, 63 left out         | 142 kept, 71 left out         |
| `CAPACITY GRAPH`       | 308 chambers, 144 bottlenecks | 298 chambers, 142 bottlenecks |
| stretches              | 75, 152 crossings             | 75, 149 crossings             |
| `CAPACITY GRAPH UNSAT` | short by 6 wires on 6 edges   | short by 3 wires on 3 edges   |

The own launcher-run gaps of 2, 9 and 38 are gone: the narrowest place in front
of each is a slanted line from a wall end to the feedline, 19.7 to 22.6 cells
long, which holds one wire. The face cut of the solid band ran from beside the
slot to the feedline, 17 to 18.4 cells, and held none. 56 stays short: f17 lies
17 to 17.5 cells from both wall ends (g84, g141). So do 31 and 32 in front of f8
and f9 (g90, g10).

## 2. What a check inside the insertion costs

Summary only; the tables are in the plan.

| Step (median)                | 17q     | 69q    |
| ---------------------------- | ------- | ------ |
| walls                        | 0.011 s | 0.07 s |
| distance transform           | 0.064 s | 3.37 s |
| Voronoi + axis + cuts        | 0.057 s | 0.43 s |
| chambers                     | 0.021 s | 0.29 s |
| check, each wire alone       | 0.1 ms  | 0.1 ms |
| **analysis in all**          | 0.154 s | 4.17 s |
| check, the flow (same graph) | 0.19 s  | 0.60 s |

- **The distance transform is the cost on 69q.** The column pass of
  `grid::squaredDistanceTransform` scans as far as the best distance so far, so
  it grows with open space. A linear transform (Felzenszwalb–Huttenlocher) gives
  the same value in every cell and takes 0.013 s on 17q and 0.24 s on 69q
  (`artifacts/logs/cap-rt/edtbench.cpp`). **Not built in**; the user decides.
- One feedline edge changes cuts up to 156 cells (17q) and 363 cells (69q) from
  its way. A window of 240 cells around the edge gives the same cuts on 17q, not
  always on 69q.
- PathFinder (negotiated congestion, `flowbench.cpp`) answered, on all 100 edges
  of 17q and 69q, whether an edge raises the flow's shortfall exactly as the
  flow did, in under 0.5 ms. The check alone missed 1 of 8 such edges on 17q and
  10 of 25 on 69q: the wires that compete for a gate.

### The measurement switches

| Variable              | Default | What it does                                                                                                                   |
| --------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `SCPD_CAPACITY_BENCH` | 0       | `benchCapacityCheck`: repeats the analysis this often, leaves every feedline edge out once, times windows of 120 and 240 cells |
| `SCPD_CAPACITY_DUMP`  | none    | a directory: every flow problem of the bench as JSON, for `flowbench`                                                          |

Both act only with `SCPD_BOTTLENECKS=1`. The bench uses `capacityPass`, a quiet
run of the whole analysis built from the same helpers as the report
(`capacityProblemOf`, `bottleneckOptionsOf`, `wallsOf`).

## 3. The check after every settled chain

`SCPD_CAPACITY_CHAIN=1` (off by default) calls `checkChainCapacity` after each
chain `optimizeChainsPrefix` settles.

### How it works

1. `stateOf` builds the chip as the chains settled so far leave it:
   - their couplers stand, with the pads and with the leads `applyOption` would
     give them;
   - their feedline edges are drawn, with the names the commit gives them;
   - a resonator whose coupler does not stand yet is left out.
2. `lostWires` runs the analysis on that state and returns the wires with no way
   or with overflow on their way.
3. The wires with a way before the chain and none with it are the ones the chain
   closes. The log says
   `[Capacity Check] chain k: n wires without a way before it, m more with it: …`.
4. When the chain closes any, it is searched again (`chainCheck_` is active). At
   the last step of a run of options, `problem.step` refuses the run when either
   holds:
   - it closes as many wires as the run it would replace;
   - more of its edges fail the commit's test than of that run (`runFaults`,
     built on `chainFaults`, the per-chain part of `stateFaults`).

   The cheapest run that closes fewer is taken. This repeats up to
   `SCPD_CAPACITY_CHAIN_TRIES` (3) times. A search that finds no such run leaves
   the chain as it was.

**Why the commit test is part of it.** `SCPD_CHAIN_KEEP_WAYS` is off, so the
commit checks every way again (`edgeWayStillOpen`). In the first version a
search again moved chain 0's first coupler to an option whose edge f0 did not
survive that check, and the commit then found no way for f0.

### What it gave on 17q

`--stage final`, `stop_after = "feedlines"`, no repair. `bad` is unrouted +
open + crossing of the final routing. Logs in `artifacts/logs/chain-check/`;
section *Built and measured* of the plan has the full table.

| Run                                     | bad   | Fails                   |
| --------------------------------------- | ----- | ----------------------- |
| check off                               | 4     | crossing 14, 31, 33, 56 |
| check on (10 s per search)              | 4     | the same                |
| check on, `SCPD_CHAIN_ASTAR_SECONDS=60` | 4     | the same                |
| `SCPD_LAUNCHER_FENCE_TURN=1`, off / on  | 5 / 5 | f7; open 31, 32; 42, 56 |

**It finds no feedline that removes a fail.**

- Where it moved a chain (25 and 38 freed), those wires had not failed.
- Where the prediction was right (31, 56), no run of the chain leaves the wire a
  way: 405 and 333 runs refused in 2 × 60 s. Chain 1's f8 and chain 3's f17 must
  pass in front of the launchers of 31 and 56.
- Most predictions are 0-wire gaps between a wire's own source run and a
  feedline, and most of those wires route. Wire 25 has the same 12-cell gaps to
  f7 as 31 has to f8; 25 routes, 31 is dead on arrival.
- 14 and 33 fail crossing the edge they bridge; the graph gives them a crossing
  stretch there. They are not predicted.

### Reproduce

```zsh
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
# once: the run directory with the inputs of a full 17q run
mkdir -p artifacts/dev/17q-chk artifacts/logs/chain-check
cp artifacts/17q/0[0-5]* artifacts/dev/17q-chk/
sed 's/^stop_after    = "couplers"/stop_after    = "feedlines"/' \
  benchmarks/17q/config.toml > artifacts/dev/17q-chk/config.toml

run() {  # run <log name> [ENV=VAL ...]
  local name=$1; shift
  /usr/bin/time -p env "$@" SCPD_DEV_OVERLAY=artifacts/dev/py-cg \
    SCPD_BOTTLENECKS=1 SCPD_SEARCH_PICTURES=0 \
    .venv/bin/python artifacts/logs/dev-mqt-scpd.py plan \
    -c artifacts/dev/17q-chk/config.toml -o artifacts/dev/17q-chk \
    --stage final -v 1 > artifacts/logs/chain-check/$name.log 2>&1
}
run 17q-off
run 17q-on SCPD_CAPACITY_CHAIN=1
run 17q-on-60s SCPD_CAPACITY_CHAIN=1 SCPD_CHAIN_ASTAR_SECONDS=60
```

The inputs in `artifacts/dev/17q-chk` came from the full run in `artifacts/17q`
of 2026-10-08 11:29. A search that reaches its clock can end elsewhere on
another machine; `SCPD_CHAIN_ASTAR_SECONDS=0` removes the clock.

## 4. The coupler page

A debugging aid the user asked for: move a coupler by hand and look at the
capacity graph again.

### Run it

```zsh
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
SCPD_DEV_OVERLAY=artifacts/dev/py-cg SCPD_SEARCH_PICTURES=0 \
.venv/bin/python artifacts/logs/dev-mqt-scpd.py couplers artifacts/dev/17q-chk
```

Then open `http://127.0.0.1:8765/`.

- The run directory needs the artifacts 00–05 and a `config.toml`.
- Start-up runs the inner and outer routing and the insertion: about 15 s on
  17q.
- `--port N` picks another port. `-v` prints every line of the stage, `-v 1` one
  line per search as well. Without `-v` the terminal shows the session lines and
  the `CAPACITY GRAPH` lines.
- The stage's switches apply as in a run of `plan`. `SCPD_BOTTLENECKS` is not
  needed.
- Installed, the same command is `mqt-scpd couplers <run dir>`.

### Use it

- The **Couplers** tab lists every coupler: chain, waypoint, resonator, the
  option it stands on, how many the chain search offers, its two feedline edges.
- Picking a coupler, in the list or by its pad on the chip, zooms to it and
  draws every option with pad and lead:
  - pink: the option it stands on;
  - blue: an option the chain search offers (`openOptionsOf`);
  - dashed: one the search does not offer (shortfall rule, body guard, jogs not
    open).

  A pad's tooltip names orientation, offset, port, jog and the resonator length
  left.
- A click on another option, on the chip or in the option table, moves the
  coupler. The page loads again with the new graph and keeps the coupler and the
  zoom.
- The log at the top of the tab shows the last move. An edge with no way reads
  `NO WAY` in red, followed by the verdict line.
- `#coupler=8` in the address opens that coupler.

Moves add up within one server. To undo one, put the coupler back on its old
option, or start the server again.

### What a move does

`Driver::setCouplerOption`:

1. `applyOption`, as the commit applies an option: the pad, the lead, the
   resonator's way.
2. Both feedline edges of the coupler stand down first, so that neither is
   routed around the other's way to the old ports.
3. Each is routed as the commit routes an edge, but against **every** edge on
   the chip (`fenceEverything_`), with the squeeze rule and its recovery. The
   way goes into `chainEdgePaths_` too.
4. `capacityGraphJson` runs `analyseCapacity` with `graphOnly_` (no pictures)
   and a debug sink that keeps only `final-capacity-graph.json`.

Other edges that now run through the new pad are not routed again. The graph
shows them; the page does not move them.

### How it is built

| Piece                                           | Where                                          | What                                                                                                                                                                                                |
| ----------------------------------------------- | ---------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pipeline::CouplerSession`                      | `CouplerSession.hpp`, end of `FinalRouter.cpp` | runs the stage to the insertion (`innerPassOf`, `routeTheRing`, `insertCouplers`) and keeps scene, tuning, wires and driver; `couplers()`, `setOption()`, `graph()`                                 |
| `Driver::couplersJson`                          | `FinalRouter.cpp`                              | couplers and options as JSON; pads as four corners (`padCorners`)                                                                                                                                   |
| `Driver::setCouplerOption`, `capacityGraphJson` | `FinalRouter.cpp`                              | see above                                                                                                                                                                                           |
| `pyscpd.CouplerSession`                         | `bindings.cpp`                                 | the binding; it keeps its own reference to the progress callable (`keepSaying`)                                                                                                                     |
| `RunDirectory.coupler_session`                  | `run.py`                                       | opens a session from a run directory                                                                                                                                                                |
| `capacityview.CouplerServer`, `serve`           | `capacityview.py`                              | `GET /` the page, `GET /graph.json`, `GET /couplers.json`, `POST /option` with `{"coupler", "option"}`; a bad request answers 400                                                                   |
| `capacityview.page_text`                        | `capacityview.py`                              | fills the template with the graph and, for the server, the session data                                                                                                                             |
| the Couplers tab                                | `capacityview.html`                            | a second script after the page's own; it wraps `build` and `pick`, keeps the picked coupler, the tab and the view in `sessionStorage` (`capacity-graph-session`), and reloads the page after a move |
| `command_couplers`                              | `cli.py`                                       | the `couplers` command                                                                                                                                                                              |

Phases 1 and 2 of `DubinsFinalRouter::run` now use the same helpers as the
session (`innerPassOf`, `outerPassOf`, `refinementPassOf`, `routeTheRing`), so
the two cannot drift apart.

## 5. More room in front of the launchers (user, 2026-10-08)

`SCPD_EDGE_LAUNCHER_MARGIN` (cells, default 5 since 2026-10-08 afternoon)
lengthens the run a feedline edge keeps closed in front of every launcher but
its own (`edgeLauncherMargin` in `corridorOfEdge`). The run is `straightStart`,
plus the bend radius under `SCPD_LAUNCHER_FENCE_TURN`, plus this margin; the
clearance disc lies around the whole of it. So an edge passes the launcher stubs
further off.

- It acts only in the corridor of a feedline edge's search. That corridor serves
  the searches and the commit of the insertion, the commit's test of a way
  (`edgeWayStillOpen`, `stateFaults`), the coupler page, and the repair's
  searches of an edge in phase 4 (the targeted re-search and `turnCoupler`). The
  coupler box, the wires' own routing and the capacity analysis do not see it.
- It is read at every corridor, not once per process, so a test can set it.
- With a margin the stage says
  `a feedline edge keeps N cells closed in front of every launcher but its own …`;
  without one it says nothing, and the log is as before.
- Measured on 4q: without a margin, edge f1 passes 21 cells from launcher
  port9's run of 11 cells and runs through the 41 cells a margin of 30 closes;
  with the margin every edge keeps at least the clearance (19.03 cells) from
  that longer run. The test `FinalRouter.AnEdgeKeepsTheLauncherMarginClear`
  holds both. Not measured on the other chips or on `bad`.

## 6. A smaller box for the couplers (user, 2026-10-08)

`SCPD_COUPLER_BOX_MARGIN` (cells, default 5 since 2026-10-08 afternoon) shrinks
the coupler box by this margin on every side, for the couplers alone
(`couplerBoxMargin`, through the `margin` parameter `CouplerBox::holds` already
had). The four tests of `makeOption` use it: the pad's body, the feedline's run
along the pad beyond its ports, the resonator's lead, and the turn after the
lead (the last only under `SCPD_COUPLER_BOX_TURN`). An option that leaves the
smaller box is refused, and `couplerPlace` walks on to the next place on the
resonator, as for the box without the margin. `terminalsInBox`, which has no
caller, uses the margin too.

- An edge between two couplers keeps the whole box (`SCPD_EDGE_IN_BOX`). The
  fence of the edges in front of the launchers is `SCPD_EDGE_LAUNCHER_MARGIN`.
  The capacity analysis and the box check of the report (`checkCouplerBodies`)
  measure against the box without the margin.
- It is read at every test, so a test can set it.
- With a margin the stage says
  `a coupler keeps N cells more off every side of that box … x … y …` after the
  line `couplers sit inside …`; without one the log is as before.
- Measured on 4q (`stop_after = "couplers"`): without a margin a pad cell lies 1
  cell from a side of the box; with 20 cells the nearest lies 28 cells off. The
  insertion then drew all 5 edges and settled its chain, where without the
  margin one edge stayed undrawn. Test:
  `FinalRouter.ACouplerKeepsTheBoxMarginClear`. Not measured on the other chips
  or on `bad`.

Both margins are 5 cells by default since the afternoon of 2026-10-08, at the
user's instruction. What that gives on 17q, and the earlier sweep with the turn
switches on, is in *The coupler box margins* of
[handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md): with
every other switch at its default, `bad` goes from 4 to 5 and chain 1 no longer
settles. The four runs of the evaluation in section 3 were made with both
margins at 0, before the change.

## 7. The check after every step (user, 2026-10-08 night)

`SCPD_CAPACITY_STEP=1` (off by default) runs the capacity analysis after every
step of the chain search that routed its edge, report only.

- The chip is `stateOf` with a prefix: the chains settled before
  (`stepSettled_`, set by `optimizeChainsPrefix`), and the chain being searched
  with its couplers up to the far end of this edge and its edges up to this one.
  A coupler after the prefix does not stand, so its resonator is no demand.
- `capacityVerdictOf` gives the wires with no way (or a way over an overfilled
  edge, routed in turn) and the shortfall; `lostWires` uses it too.
- Each step says, at `-v 1`:
  `[Capacity Step] chain 1 f7 (1->2) on options launcher 0 4: UNSAT — 1 wire without a way (27), short by 1; this edge closes 27; 0.107s`.
  "Closes" is against the prefix before it, or against the chip without the
  chain for a first edge.
- Each searched chain says how many steps were checked and how long they took,
  and each feedline edge of the run it settles on says what the check said when
  the search laid it:
  `[Capacity Step] chain 3 f17: UNSAT — 1 wire without a way (56), short by 1, closes 56`.
- The edges are named as the commit names them (`f<slot>`).

**Cost.** One check is one whole analysis: about 0.105 s on 17q, after the
distance transform became linear (64 ms → 17 ms; the 139 lines of the analysis
are word for word the same). The rest is Voronoi 42 ms, chambers 21 ms, cuts 21
ms, walls 8 ms. The checks spend the chain search's clock: on 17q chain 1 routed
781 steps instead of 2157 before its 10 s ran out. Chains 0, 2 and 3 settled on
the same runs.

17q (margins 5, `artifacts/logs/cap-step/17q.log`): 234 steps checked in 25 s.
Every edge of chains 0 and 2 is SAT. In chain 3, f17 closes wire 56 and f18, f19
keep it closed. Chain 1 ran out of time; 16 of its 116 checked steps close a
wire, for example f7 on options 4 or 8 closes 27 or 26.

## 8. The capacity rule replaces the squeeze rule (user, 2026-10-09)

The squeeze rule is gone (see *The squeeze rule* in
handover-cpw-coupler-insertion.md). In its place the check of section 7 refuses:

- `SCPD_CAPACITY_RULE`, off since 2026-10-10 (user; on before): a step whose
  edge closes a wire (a wire with a way through the graph the prefix leaves, and
  none with this edge) is refused as an edge with no way; the A* takes another
  option. The step line ends `— refused (SCPD_CAPACITY_RULE)`, and the chain
  line counts the refusals.
- A chain that no run joins under the rule is searched once more without it, and
  says so (`out of time before …` or
  `no run of options joins the chain without closing a wire`). The second search
  checks steps only under `SCPD_CAPACITY_STEP`.
- The checks' time is added to the chain's budget (`problem.budget`, which the
  A* reads on every round), so a checked chain prices as many steps as an
  unchecked one.
- The rule acts in the chain search only. The commit, the targeted repair and
  the coupler session route edges without it.

**The crossing count** (2026-10-09, user): 27, 33 and 56 were counted crossing
on the last cell of the band (`CROSSING_REACH` 10), where they arrive straight
and the next move starts turned. `CrossingConstraints::allowedArriving` judges a
cell on the heading it arrives with as well; the Final stage's count,
`whereItCrosses` and the DRC use it.

17q (both margins at their defaults unless named, 10 s per chain,
`artifacts/logs/caprule/*-x.log`):

| Arm   | Graph | Feedline routing            |
| ----- | ----- | --------------------------- |
| box 5 | SAT   | open 10, 11, f0; crossing – |
| box 8 | SAT   | open 10, 11, f0; crossing – |

Chain 0 settles on `20 3 0 12 27`. The squeeze rule refused it (f1 leaves 21
cells to the artwork beside wire 11); the graph routes 11 round another way, and
the feedline pass then leaves 10, 11 and f0 open. The graph is a necessary
condition only. Before the rule, with the squeeze rule and box 8, chain 0 sat on
`0 0 0 12 27` at the same cost.

### The resonator length rule (user, 2026-10-09)

Resonator 11 had a way through the graph round the whole chip (34 bottlenecks)
where its coupler's lead stood on the far side of f1: legal in the graph, far
longer than a resonator can be. Now:

- `FlowDemand::longest` = target length − run to the port (`anchorGap`) − length
  of the lead + tolerance (`longestWayOf`), for resonators only.
- `checkInTurn` routes such a demand on its shortest way by the least length
  (`shortestWay`: Dijkstra over edge, chamber left into, crossings so far): from
  the start terminal to the line of the first edge, from line to line
  (`betweenLines`), and from the last line to the end terminal. The lines are
  the bottleneck's line and a stretch from its first cell to its last
  (`FlowEdge::from`/`to`). A first version measured through the middles of the
  edges; on a wide bottleneck that is a detour, and resonators 18 and 20 were
  refused although they route.
- A way longer than `longest` is no way; `FlowCheck::tooLong` says so, the wire
  line reads
  `no way within its length: the shortest runs N cells, it may run M`, and the
  verdict names the wire apart.

17q, both margins as named, 10 s per chain (`artifacts/logs/caplen/*-2.log`, run
directories `artifacts/dev/17q-len-b5`, `-b8`):

| Arm   | Chains                         | Graph | Feedline routing               |
| ----- | ------------------------------ | ----- | ------------------------------ |
| box 5 | all four settle under the rule | SAT   | unrouted –, open –, crossing – |
| box 8 | all four settle under the rule | SAT   | unrouted –, open –, crossing – |

Pictures: `artifacts/sat/17q-length-box5-final.svg` and
`17q-length-box5-capacity-graph.svg`/`.html`.

## 9. The fast check: profile cuts (2026-10-10)

`SCPD_CAPACITY_FAST` (0 by default) changes how a check of the chain search
finds its bottlenecks. Everything after the bottlenecks (chambers, graph,
`checkInTurn`, the rule) is unchanged.

### Why

One check was a whole analysis of the chip. Measured on 17q over 241 checks of
the chain search (this cloud machine, about three times slower than the Mac):

| Step                                                       | ms per check |  Share |
| ---------------------------------------------------------- | -----------: | -----: |
| Voronoi (`construct_voronoi` 97, filter 34, sites 16)      |          146 |   30 % |
| cuts (`findBottlenecks`, then `couplerCuts` drops a third) |          109 |   23 % |
| chambers (flood 78, blocked moves 11)                      |           91 |   19 % |
| distance transform                                         |           60 |   13 % |
| axis (`rasterizeMedialAxis`)                               |           33 |    7 % |
| walls (port runs 13, border 8.5, feedlines 4, artwork 3.6) |           31 |    6 % |
| stretches and ports                                        |          3.3 |  < 1 % |
| `checkInTurn`                                              |         0.16 | 0.03 % |
| **one check**                                              |      **490** |        |

More than 99 % is geometry of the whole chip, done again at every check.

### How it works

`grid::profileCuts` (`src/grid/ProfileCuts.cpp`) needs no medial axis:

- **Static fields.** `fastCapacityOf` builds the walls that stay through an
  insertion once (`wallsOf(…, staticOnly)`: border, artwork, the port runs of
  the plain wires and of every resonator's target), and two feature transforms
  of them (`grid::nearestBlocked`): one of the walls inside the feedline ring
  (artwork, runs at targets), one of those outside (border, runs at sources).
  Two fields let a cell of a feedline see a static wall on each side.
- **Dynamic walls** (feedline edges, pads, leads) are traced (Moore tracing) and
  walked along. The gap from every outline cell to each field is a profile; a
  bottleneck is a persistent minimum: the gap rises by
  `2 × SCPD_BOTTLENECK_RISE` on both sides before it falls below, a stretch of
  one width counts once at its middle, and a change of the wall across counts as
  a rise unless the gap falls there.
- **Two dynamic walls.** The gap is the distance between their lines less both
  radii: a feedline edge is its middle line inflated by `wallInflation`, a pad
  or a lead its outline. This is how two feedline edges are cut.
- **Mouths.** Every port run's slot is closed by a cut from each flank's open
  end to the nearest dynamic wall ahead, unless a static wall is nearer
  (`mouthOf`). The medial axis does this with its slot rules; a profile has no
  minimum there.
- **Lines and wedges.** A cut must cross only free cells and no slot. Where two
  walls touch, or both touch a third dynamic wall near the cut (two edges of a
  chain at their coupler), the wedge between them is no bottleneck. Cuts between
  the same two walls with both ends within a room pitch are one.

`SCPD_CAPACITY_FAST=2` runs the medial axis (which decides) and the profiles,
and logs every step where the wires they close differ (`[Capacity Fast]`), and
per chain how often they agree. At 1 the report after the insertion
(`SCPD_BOTTLENECKS`, `final-capacity-graph.*`, `final-bottlenecks.svg`) is built
from the profile cuts too, with no medial axis to draw. With either mode on,
every chain logs the time of each step of the fast pass.

Also in this work, with the same results:

- `grid::chambersOf` labels the chambers in one pass with a union-find kept in
  the labels (root = lowest cell), numbered in the order a flood would give;
  `bottleneckMoves` looks only at cells within two cells of a cut's line (49 → 6
  ms for 145 cuts). Tests:
  `Chambers.LabelEveryCellAsAFloodFromTheLowestCellWould`,
  `Chambers.ABottleneckRefusesEveryStepAcrossItsLine`.
- The capacity pass hands its wall and chamber labels back for the next pass
  (`giveBack`); the labelling keeps its scratch per thread.

### Measured on 17q (2026-10-10)

Cloud machine, `stop_after = "feedlines"`, no repair, `SCPD_CAPACITY_RULE=1`.
Measured with a C++ driver outside the package that plans once and runs the
Final stage, reading `benchmarks/17q/config.toml` as
`test/pipeline/Benchmarks.hpp` does; the Python CLI was not used.

| Arm                                             | Check per step | Insertion | Whole Final stage | Final routing                                    |
| ----------------------------------------------- | -------------- | --------: | ----------------: | ------------------------------------------------ |
| no rule                                         | –              |       8 s |              84 s | open 18, 19                                      |
| rule, medial axis                               | 0.40-0.51 s    |     227 s |             306 s | open 18, 19                                      |
| rule, profiles                                  | 0.063-0.144 s  |      85 s |             164 s | open 18, 19, same short/long, same angle cost 48 |
| rule, profiles, after the second round below    | 0.017-0.057 s  |      58 s |             133 s | the same                                         |
| rule, medial axis, after the second round below | 0.34-0.45 s    |     187 s |             261 s | the same                                         |

The fast pass per check after the second round: walls 7-13 ms, profiles 12-24
ms, chambers 5-15 ms, stretches 1-5 ms, `checkInTurn` 0.3 ms.

**The second round** (same day):

- `grid::chambersOf` labels by runs: a row's free cells that no cut touches are
  one run and are joined as a whole to the runs above it they overlap
  (diagonally only at a run's ends, past the corner rule); a cell with a refused
  step is joined step by step as before. 17q: 45 → 6 ms per call. Test:
  `Chambers.LabelEveryCellAsAFloodFromTheLowestCellWould`, now 20 grids with 30
  cuts each.
- `wallsOf` copies the border and the artwork from `baseWalls` (built once) and
  lays every port run from `runStampOf`, the run's cells by geometry, kept while
  the run's line stays the same (only the leads change). A cell another wall
  holds is skipped as before.
- The Moore trace of `grid::profileCuts` stops where its step out of the start
  repeats its first step; the old stop could miss and trace a wall several times
  (one outline of 19 500 cells on 17q). Test:
  `ProfileCuts.CutsAThinWallToTheWallOnEachSide`.
- A mouth of a static run is searched only as far as the static wall ahead of
  it.

Proof that the walls and chambers are unchanged: the medial-axis run with the
rule after the second round gives every one of the 386 steps of the run before
it the same verdict; it checks 32 steps more in the same time.

Agreement (`SCPD_CAPACITY_FAST=2`, 407 steps): the two refuse the same steps on
404 and close exactly the same wires on 327. The three refusals that differ are
in chain 1: f6 on option 33 (the profiles close 20 21 22, the axis none) and f8
on options … 29 4 (the axis closes 30, the profiles none). The profiles cost
0.060, 0.091, 0.101 and 0.150 s per check in chains 0 to 3, the medial axis
0.34-0.45 s.

Tests: `ctest -E EveryChip` 408 of 410 after the second round.
`FinalRouter.OnlyUnsettledRedrawsOneWire` fails as before;
`CouplerInsertion.ASecondDoglegOffersAnotherPlacement` (anchor 580 against 581)
fails too, in code this work does not touch.

Cut for cut the profiles do not reproduce the axis (on the 17q end state about
90 of 145 cuts match within a room pitch): the slot rules and the persistence
along an axis and along a wall differ. The verdict is what the rule uses.

### What is open

1. **The remaining time** is the profiles (12-24 ms; `collectWalls` scans the
   whole grid, the dynamic walls could come from the wall builder), the feedline
   edges in the walls (each one's disc stamped again per check) and the chambers
   (an incremental flood of only the chambers the step's walls touch: on the 17q
   end state a new edge touches 11 chambers in the median, 7.8 % of the free
   cells).
2. **The three differing refusals** were not looked into.
3. **Larger chips** were not run: 69q was stopped (user). `collectWalls` scans
   the whole grid per check; on 69q the dynamic walls should come from the wall
   builder instead.
4. **The default** stays the medial axis until the user decides.

## 10. An open chain prescribes no crossing (user, 2026-10-10)

On 33q the rule made the result worse: 2 open wires, angle cost 80, four chains
searched again without the rule, 609 s. Without the rule: 0 bad wires, angle
cost 76. The step graphs (`SCPD_CAPACITY_STEP_GRAPHS=1`) showed why (user):
every edge the check called closing lay on a chain the search had laid only up
to that edge.

The check prescribed every drawn edge between couplers to the plain wires that
bridge it (`bridgers_`), the edges of the chain under search too. That chain is
open at its end, so both sides of a new edge are one chamber, and
`crossingStretches` lays no stretch where both sides are one chamber. A
prescribed edge with no stretch leaves the wire no way. 12 of the 13 steps that
closed a wire on the 33q run without the rule were this; source and target
mostly lay in the same chamber.

The fix: an edge of a chain that stands only in part (`Edge::open`, set by
`stateOf` when fewer than all waypoints stand) is prescribed to no wire. It is
still a wall, and its stretches, where it has any, stay open to its bridgers. A
pocket that a half-laid chain closes against other walls is still found at the
step that closes it. Test:
`FinalRouter.TheStepCheckPrescribesNoCrossingOfAnOpenChain` (9q; it fails
without the fix on chain 0 f2 and f3).

33q, rule on, profiles, sequential, CLI, `stop_after = "feedlines"`:

| Arm                  | Bad wires | Angle cost | Chains again without the rule | Insertion | Whole run |
| -------------------- | --------- | ---------: | ----------------------------- | --------: | --------: |
| rule, before the fix | 2 (open)  |         80 | 1, 3, 4, 6                    |         – |     609 s |
| no rule              | 0         |         76 | –                             |    58.8 s |     346 s |
| rule, with the fix   | 0         |         76 | –                             |    47.8 s |     370 s |

With the fix every feedline edge of the seven chains is SAT, and the final
routing is the one without the rule (short: 1, 9, 13, 22, 31, 49, 80, 87, 92,
102). The rule still refuses three steps: chain 1 f6 on options 6 and 9 (wires
18 and 19 pass the pad and the lead of coupler 20 through cuts of 9-13 cells,
which take no wire) and chain 6 f35 on options 0 0 (eight wires). Whether those
cuts are right is open.

## 11. A gap at a terminal edge is no limit (user, 2026-10-11)

On 57q the rule refused every step of chain 8, and four chains on 69q refused
every step from their first edge on. A diagnosis run on 57q
(`SCPD_CAPACITY_STEP_GRAPHS=1`, `-v 1`) showed:

- No edge of the 446 steps that closed a wire meets another feedline edge (the
  step line names any it meets: `its edge meets …`).
- Every closure was capacity, not a missing way: a bottleneck full.
- 10 of the 17 refused steps written out had a terminal edge (from or to a
  launcher) at the full bottleneck. Chain 8: its first edge f62 runs beside f61,
  the last edge of chain 7; the gap f62 · f61 takes 1-3 wires and five (178-182)
  need it, and the gaps of 9-13 cells between f62 and the pad or lead of coupler
  183 take none.
- The other 7: chain 1 f14 · artwork (10 wires on 9 places), chain 4 f33 ·
  artwork near Qb57 (2 on 1; f33 ends with no way, the one unrouted wire of 57q)
  and chain 6, lead of 160 · f54 (9-12 cells, none).

The feedline routing lays the terminal edges again, so a gap at one is no limit
(user). `capacityProblemOf` gives a bottleneck with an end on a terminal edge
the capacity of every wire; the edge stays a wall, and no wire is prescribed to
cross it. The graph JSON marks such a bottleneck `"terminal": true`. Test:
`FinalRouter.ATerminalEdgeTakesEveryWire` (9q; it fails without the change).

Open: the gaps of 9-13 cells between a feedline edge and the pad or lead of the
coupler beside it (33q chain 1 f6, 57q chain 6 f54) take no wire. Whether that
is right is the user's call.

## 12. The end pair the other way round in the commit (user, 2026-10-11)

On 57q chain 4 ran out of time with the rule and again without it, and was
committed as it stood. The commit lays the edges front to back, and f33 (its
edge 1) found no way after f32: the one unrouted wire. The chain search lays an
end pair the other way round when the later edge finds no way (`mayReorder` in
`solveChainAStar`; it fired 15 times on chain 4), the commit did not.

The commit now does the same for edge 1 and the last edge of a chain: it takes
the predecessor up, lays the edge first, and routes the predecessor again
against it; when either finds no way, both go back as they were. The log says
`no way after f…, but both fit with f… laid first`.

57q, rule on, profiles, terminal gaps open: 0 bad wires (was 1), 0 edges not
drawn (was 1, f33), angle cost 138 (was 136; f32 is routed again), graph SAT,
675 s. No test yet: none of the chips the tests run (4q, 9q) commits a chain the
search left as it stood.

## 13. Every wire may cross a terminal edge (user, 2026-10-11)

On 69q the rule refused every option of the first edge of chains 1, 4, 7, 9 and
11, and the final graph had no way for 14 wires (19, 20 · 72-75 · 133-136 ·
176-179). The diagnosis run (`SCPD_CAPACITY_STEP_GRAPHS=1`, `-v 1`) showed one
cause: the first edge of a chain meets the last edge of the chain before it at
the next launcher (`its edge meets f5` and the like; chain 4's f26 and f25 run
13 cells apart, and their walls of 9.5 cells each merge). Between them lie the
chip ports of plain wires, in a pocket of three chambers with no bottleneck out:
where two walls touch there is no gap to cut. The feedline routing lays the
terminal edges again and every one of those wires was drawn in the end; only 183
and 184 stayed open.

The fix (user): a terminal edge is crossable. `crossingStretches` gives it
stretches as any edge, open to every wire (`users` empty) and taking every wire.
No wire is prescribed one, so a way may cross it or not. The graph JSON marks
such a stretch `"terminal": true`. Tests:
`FinalRouter.ATerminalEdgeTakesEveryWire` (9q) checks the stretches as well;
`FinalRouter.DrawsTheCapacityGraphAfterTheBottlenecks` lets a way cross a
terminal edge beside its prescribed ones.

## 14. Feedlines with a reflector (user, 2026-10-11)

On 45q the final graph had no way for wires 57 and 58 (Chip.port44/43 to
Coupler24_28). Both are prescribed to cross f20, the first edge of chain 3.
Chain 3 has one launcher: its other end is a reflector, so its first point is a
coupler and f20 runs between two couplers. The chain stays open at the reflector
when it stands whole, so f20 has chamber 55 on both sides (153 of its 158 cells)
and `crossingStretches` laid no stretch: a prescribed edge with no stretch, no
way. The rule then refused every last edge f25 of the chain (with it the chain
stands whole and the prescription holds), and the chain ran out of time twice
and was committed as it stood.

The user's reading: f20 is taken for a terminal edge. In the capacity graph it
was not (`Edge::terminal` asks for a launcher), but the chain search and the
commit took the first and the last edge of a chain for terminal edges: the
fences in `fenceCommittedEdges` and the prefix search, `pairAtAnEnd`,
`CHAIN_FRESH_EDGES` and the commit's end-pair reorder. All of them now ask
`atLauncher(chain, at)`. A chain with two launchers is unchanged.

In the capacity graph an edge of a chain with a reflector end keeps the places
where both sides are one chamber, as a stretch from that chamber back into it.
`checkInTurn` takes such an edge as a step that crosses it (both searches
skipped an edge back into the chamber they came from). The flow of
`SCPD_CAPACITY_FLOW` already took it. The insertion logs every such chain:
`chain 3: one launcher, a reflector at its first end`.

Tests: `InTurn.AWireCrossesAFeedlineWithOneChamberOnBothSides`,
`InTurn.AWireOfFixedLengthCrossesAFeedlineWithOneChamberOnBothSides` (both fail
without the change). No test chip (4q, 9q) has a reflector.

## All switches of this work

| Variable                    | Default | What it does                                                                                                              |
| --------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------- |
| `SCPD_BOTTLENECK_ALL`       | off     | on: every cut, not only those with an end on a feedline edge, a pad or a lead                                             |
| `SCPD_CAPACITY_FLOW`        | off     | on: the integer flow instead of the wires routed one after another                                                        |
| `SCPD_CAPACITY_CHAIN`       | off     | on: the check after every settled chain, and the search again                                                             |
| `SCPD_CAPACITY_STEP`        | off     | on: the analysis after every step of the chain search, report only (section 7)                                            |
| `SCPD_CAPACITY_RULE`        | off     | the chain search refuses a step whose edge closes a wire (section 8)                                                      |
| `SCPD_CAPACITY_STEP_GRAPHS` | off     | on: the graph the check saw at every edge of a settled chain, as `step-c<chain>-f<edge>-capacity-graph.json` (section 10) |
| `SCPD_CAPACITY_CHAIN_TRIES` | 3       | how often one chain is searched again                                                                                     |
| `SCPD_COUPLER_BOX_MARGIN`   | 5       | cells the couplers keep further in from every side of the coupler box (section 6)                                         |
| `SCPD_EDGE_LAUNCHER_MARGIN` | 5       | cells added to the run a feedline edge keeps closed in front of every other launcher (section 5)                          |
| `SCPD_CAPACITY_FAST`        | 0       | 1: the bottlenecks of a check by profiles (section 9); 2: both, the axis decides, log the differences                     |
| `SCPD_CAPACITY_BENCH`       | 0       | measurement only (section 2)                                                                                              |
| `SCPD_CAPACITY_DUMP`        | none    | measurement only (section 2)                                                                                              |

## Tests

- `test/pipeline/test_capacity_flow.cpp`: `InTurn.*`, eleven tests.
- `test/pipeline/test_final_router.cpp`:
  - `FinalRouter.DrawsTheCapacityGraphAfterTheBottlenecks` also checks that
    every bottleneck edge has an end on a feedline edge, a pad or a lead. It
    fails with `SCPD_BOTTLENECK_ALL=1`, as it should. It also checks that every
    way crosses exactly the feedline edges its wire is prescribed.
  - `FinalRouter.TheCapacityCheckLeavesAChainThatClosesNoWayAsItIs`, on the
    nine-qubit fixture: the four-qubit fixture settles no chain.
  - `CouplerSession.MovesACouplerAndDrawsItsEdgesAgain`.
- `test/python/unit/test_capacityview.py`: the session data on the page, the
  server with a stand-in session (`FakeSession`), a real session on 4q.

Last state:

- `ctest -E EveryChip`: 378 of 379. Only
  `FinalRouter.OnlyUnsettledRedrawsOneWire` fails, as before this work.
- Python: 153 of 154.
  `test_planning.py::test_the_detail_routing_carries_a_drawn_wire_per_connection`
  fails: the ends of the Detail wires do not match the Corridor plan. This work
  does not touch that code; whether it failed before was not checked.
- The `EveryChip` benchmark tests were not run.

## Pitfalls

1. **Keep the overlay current.** The user runs on `artifacts/dev/py-cg`. After a
   C++ change: build `mqt-scpd-bindings`, then
   `OVERLAY=py-cg zsh artifacts/logs/dev-sync.sh`. A run on an old overlay
   showed the old cuts and looked like a bug in the filter.
2. **Do not sync an overlay while a process has its binding loaded**, such as a
   running coupler server: `dev-sync.sh` copies over the `.so`. To change only
   the page, copy `capacityview.html` into the overlay alone.
3. **Without `-v` the stage prints nothing.** For timing runs use `-v 0`: the
   progress lines, without the per-search lines.
4. **The lint hooks rewrite files that are not yours.** `typos` changes
   "vermilion" in `FinalRouter.cpp`, `trailing-whitespace` a blank line in it,
   and `rumdl` reflows all of `CHANGELOG.md`. Copy the files before
   `prek run --files …` and put them back; format C++ with
   `clang-format --lines` on the changed hunks only.
5. **The stub generator.** It wrote the keyword `global` as a parameter name,
   which made the stub invalid; the bindings now name it `global_`. It also
   writes the progress and debug callbacks as `object` where the stub had
   `Callable` types. Do not edit the `.pyi` by hand.
6. **`SCPD_CHAIN_KEEP_WAYS` is off.** The commit checks every way of the search
   again. Anything that changes a chain's options after its search has to pass
   the same test (`chainFaults`).

## What is open

1. **The own stub.** Most predictions are 0-wire gaps at a wire's own port run,
   and most of those wires route. If the own wire may pass such a gap, the false
   predictions go, and so do the true ones for 31 and 56. The user decides. With
   the port runs as two thin walls, 56 is the only such gap left on 17q.
2. **A prediction of the fails that do occur.** On 17q they are a terminal whose
   first cells lie within the clearance of a feedline (31, dead on arrival at
   f8) and bridged crossings with too little straight edge (14 on f2, 33 on f9).
   Neither is a capacity question.
3. **The cost of a check on the large chips.** One check is one whole analysis
   of the chip: 0.1 s on 17q, 0.5 s on 21q, about 1.5–2 s on 33q–69q. The linear
   distance transform is in (`src/grid/DistanceTransform.cpp`); what is left is
   the Voronoi, the axis, the cuts and the chambers of the whole chip. A check
   that updates the graph only around the new edge is the next step (§2 and
   `artifacts/logs/cap-fast/`).
4. **Wires that compete.** The wires routed in turn see them in one order only;
   PathFinder would try other orders, at under half a millisecond. Not built
   into the stage.
5. **69q has 17 wires with no way in the graph at all**, with and without the
   filter. Not looked into.
6. **The coupler page moves one coupler at a time** and reroutes only its two
   edges. Rerouting the edges the new pad blocks, or a whole chain, is not
   built.
