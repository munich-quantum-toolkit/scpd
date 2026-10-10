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
  - `src/grid/DistanceTransform.cpp` (linear), `test/grid/test_distance_transform.cpp`;
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

- A lead is a port run with `Wall::lead` set: `markRun` sets it on the two
  walls of a resonator's source run. In the log a lead is still a wall of the
  kind `stub`, so `artwork–stub` and `stub–stub` cuts remain in the count; their
  stub end is a lead.
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
- A wire that finds no such way takes its way with the fewest edges whatever
  the capacities. The edges it overfills have overflow.
- A plain wire crosses exactly the feedline edges it is prescribed, each once,
  and no other (`FlowDemand::crossings`). Its prescribed edges are the drawn
  edges between two couplers that it bridges (`bridgers_`). The search carries
  the prescribed edges crossed so far in its state; a step back into the
  chamber an edge was entered from is no step. A prescribed edge with no
  stretch leaves the wire no way. Resonators cross nothing.
- The answer is a `FlowCheck`, so the page and the JSON show loads and
  overflow as for the flow. The JSON names each wire's prescribed edges
  (`crosses`), and the page shows them as "has to cross".

The verdict is `SAT` when every wire has a way and no edge has overflow.
Otherwise the line reads `UNSAT — … short by N wires on M edges, routed one
after another in wire order` and names each overfilled edge with the wires
through it; wires with no way at all are named apart.

- Overflow says only that this order does not fit. Another order may.
- `SCPD_CAPACITY_FLOW=1` restores the integer flow `checkCapacity` (HiGHS).
  The flow does not hold a wire to its prescribed edges.

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
  band's end face beside a 3-cell slot. All 76 cuts between a launcher run and
  a feedline end within half a cell of a wall end; before, 43 ended on the face.
- No cut joins the two walls of one run: its line would cross the slot, which
  `findBottlenecks` refuses. So `couplerCuts` needs no rule for it.
- The end at the port must be closed. Open, the slot joined the room behind the
  run through the row of free cells between each launcher run and the border:
  85 cuts had a way round them, and a terminal lay in the chamber behind its
  run rather than the one in front of it.

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
17 to 17.5 cells from both wall ends (g84, g141). So do 31 and 32 in front of
f8 and f9 (g90, g10).

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
  with its couplers up to the far end of this edge and its edges up to this
  one. A coupler after the prefix does not stand, so its resonator is no
  demand.
- `capacityVerdictOf` gives the wires with no way (or a way over an
  overfilled edge, routed in turn) and the shortfall; `lostWires` uses it too.
- Each step says, at `-v 1`:
  `[Capacity Step] chain 1 f7 (1->2) on options launcher 0 4: UNSAT — 1 wire without a way (27), short by 1; this edge closes 27; 0.107s`.
  "Closes" is against the prefix before it, or against the chip without the
  chain for a first edge.
- Each searched chain says how many steps were checked and how long they
  took, and each feedline edge of the run it settles on says what the check
  said when the search laid it: `[Capacity Step] chain 3 f17: UNSAT — 1 wire
  without a way (56), short by 1, closes 56`.
- The edges are named as the commit names them (`f<slot>`).

**Cost.** One check is one whole analysis: about 0.105 s on 17q, after the
distance transform became linear (64 ms → 17 ms; the 139 lines of the analysis
are word for word the same). The rest is Voronoi 42 ms, chambers 21 ms, cuts
21 ms, walls 8 ms. The checks spend the chain search's clock: on 17q chain 1
routed 781 steps instead of 2157 before its 10 s ran out. Chains 0, 2 and 3
settled on the same runs.

17q (margins 5, `artifacts/logs/cap-step/17q.log`): 234 steps checked in 25 s.
Every edge of chains 0 and 2 is SAT. In chain 3, f17 closes wire 56 and f18,
f19 keep it closed. Chain 1 ran out of time; 16 of its 116 checked steps close
a wire, for example f7 on options 4 or 8 closes 27 or 26.

## 8. The capacity rule replaces the squeeze rule (user, 2026-10-09)

The squeeze rule is gone (see *The squeeze rule* in
handover-cpw-coupler-insertion.md). In its place the check of section 7 refuses:

- `SCPD_CAPACITY_RULE`, off since 2026-10-10 (user; on before): a step whose
  edge closes a wire (a wire with a
  way through the graph the prefix leaves, and none with this edge) is refused
  as an edge with no way; the A* takes another option. The step line ends
  `— refused (SCPD_CAPACITY_RULE)`, and the chain line counts the refusals.
- A chain that no run joins under the rule is searched once more without it,
  and says so (`out of time before …` or `no run of options joins the chain
  without closing a wire`). The second search checks steps only under
  `SCPD_CAPACITY_STEP`.
- The checks' time is added to the chain's budget (`problem.budget`, which
  the A* reads on every round), so a checked chain prices as many steps as an
  unchecked one.
- The rule acts in the chain search only. The commit, the targeted repair and
  the coupler session route edges without it.

**The crossing count** (2026-10-09, user): 27, 33 and 56 were counted
crossing on the last cell of the band (`CROSSING_REACH` 10), where they arrive
straight and the next move starts turned. `CrossingConstraints::allowedArriving`
judges a cell on the heading it arrives with as well; the Final stage's count,
`whereItCrosses` and the DRC use it.

17q (both margins at their defaults unless named, 10 s per chain,
`artifacts/logs/caprule/*-x.log`):

| Arm | Graph | Feedline routing |
| --- | --- | --- |
| box 5 | SAT | open 10, 11, f0; crossing – |
| box 8 | SAT | open 10, 11, f0; crossing – |

Chain 0 settles on `20 3 0 12 27`. The squeeze rule refused it (f1 leaves 21
cells to the artwork beside wire 11); the graph routes 11 round another way,
and the feedline pass then leaves 10, 11 and f0 open. The graph is a necessary
condition only. Before the rule, with the squeeze rule and box 8, chain 0 sat
on `0 0 0 12 27` at the same cost.

### The resonator length rule (user, 2026-10-09)

Resonator 11 had a way through the graph round the whole chip (34 bottlenecks)
where its coupler's lead stood on the far side of f1: legal in the graph, far
longer than a resonator can be. Now:

- `FlowDemand::longest` = target length − run to the port (`anchorGap`) −
  length of the lead + tolerance (`longestWayOf`), for resonators only.
- `checkInTurn` routes such a demand on its shortest way by the least length
  (`shortestWay`: Dijkstra over edge, chamber left into, crossings so far):
  from the start terminal to the line of the first edge, from line to line
  (`betweenLines`), and from the last line to the end terminal. The lines are
  the bottleneck's line and a stretch from its first cell to its last
  (`FlowEdge::from`/`to`). A first version measured through the middles of
  the edges; on a wide bottleneck that is a detour, and resonators 18 and 20
  were refused although they route.
- A way longer than `longest` is no way; `FlowCheck::tooLong` says so, the
  wire line reads `no way within its length: the shortest runs N cells, it
  may run M`, and the verdict names the wire apart.

17q, both margins as named, 10 s per chain (`artifacts/logs/caplen/*-2.log`,
run directories `artifacts/dev/17q-len-b5`, `-b8`):

| Arm | Chains | Graph | Feedline routing |
| --- | --- | --- | --- |
| box 5 | all four settle under the rule | SAT | unrouted –, open –, crossing – |
| box 8 | all four settle under the rule | SAT | unrouted –, open –, crossing – |

Pictures: `artifacts/sat/17q-length-box5-final.svg` and
`17q-length-box5-capacity-graph.svg`/`.html`.

## All switches of this work

| Variable                    | Default | What it does                                                                                     |
| --------------------------- | ------- | ------------------------------------------------------------------------------------------------ |
| `SCPD_BOTTLENECK_ALL`       | off     | on: every cut, not only those with an end on a feedline edge, a pad or a lead                    |
| `SCPD_CAPACITY_FLOW`        | off     | on: the integer flow instead of the wires routed one after another                               |
| `SCPD_CAPACITY_CHAIN`       | off     | on: the check after every settled chain, and the search again                                    |
| `SCPD_CAPACITY_STEP`        | off     | on: the analysis after every step of the chain search, report only (section 7)                   |
| `SCPD_CAPACITY_RULE`        | off     | the chain search refuses a step whose edge closes a wire (section 8)                             |
| `SCPD_CAPACITY_CHAIN_TRIES` | 3       | how often one chain is searched again                                                            |
| `SCPD_COUPLER_BOX_MARGIN`   | 5       | cells the couplers keep further in from every side of the coupler box (section 6)                |
| `SCPD_EDGE_LAUNCHER_MARGIN` | 5       | cells added to the run a feedline edge keeps closed in front of every other launcher (section 5) |
| `SCPD_CAPACITY_BENCH`       | 0       | measurement only (section 2)                                                                     |
| `SCPD_CAPACITY_DUMP`        | none    | measurement only (section 2)                                                                     |

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
   predictions go, and so do the true ones for 31 and 56. The user decides.
   With the port runs as two thin walls, 56 is the only such gap left on 17q.
2. **A prediction of the fails that do occur.** On 17q they are a terminal whose
   first cells lie within the clearance of a feedline (31, dead on arrival at
   f8) and bridged crossings with too little straight edge (14 on f2, 33 on f9).
   Neither is a capacity question.
3. **The cost of a check on the large chips.** One check is one whole analysis
   of the chip: 0.1 s on 17q, 0.5 s on 21q, about 1.5–2 s on 33q–69q. The
   linear distance transform is in (`src/grid/DistanceTransform.cpp`); what is
   left is the Voronoi, the axis, the cuts and the chambers of the whole chip.
   A check that updates the graph only around the new edge is the next step
   (§2 and `artifacts/logs/cap-fast/`).
4. **Wires that compete.** The wires routed in turn see them in one order only;
   PathFinder would try other orders, at under half a millisecond. Not built
   into the stage.
5. **69q has 17 wires with no way in the graph at all**, with and without the
   filter. Not looked into.
6. **The coupler page moves one coupler at a time** and reroutes only its two
   edges. Rerouting the edges the new pad blocks, or a whole chain, is not
   built.
