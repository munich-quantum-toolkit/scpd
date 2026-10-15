# The capacity graph SAT check against the feedline routing

Written for whoever takes the capacity check of the coupler insertion further.
It covers 2026-10-13 to 2026-10-15: the experiment that sets the check's
verdict against the feedline routing on every chip, the rules that came out of
31 review rounds with the user, what the check gets right and wrong per chip,
the open problems by class, and a measurement of the older, more optimistic
capacity rule the user asked for last.

Read [handover-capacity-check.md](handover-capacity-check.md) first for the
check inside the insertion and the profile cuts, and
[handover-bottleneck-analysis.md](handover-bottleneck-analysis.md) for the
walls, the port runs with their slots, the cuts and the chambers. This document
only says what changed since.

## State

- Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`, HEAD `ce1d5b4` (2026-10-13), everything here
  **uncommitted**; the user commits per phase.
- Changed for this work: `src/pipeline/FinalRouter.cpp` (the check),
  `src/pipeline/CapacityFlow.cpp` and its header (the demand router),
  `src/grid/Bottlenecks.cpp` and its header (`uncrossedBottlenecks`),
  `test/pipeline/test_final_router.cpp`, `test/pipeline/test_capacity_flow.cpp`,
  `test/grid/test_bottlenecks.cpp`, `CHANGELOG.md`. The rest of the working
  tree (bindings, dashboard, chambers, profile cuts, crossing constraints) is
  the user's earlier work on this branch.
- **A lint run damaged the tree on 2026-10-14.** `uvx nox -s lint` runs
  `prek run --all-files`; its self-correcting hooks (license headers, prettier,
  ruff format, typos) rewrote 134 files that had no uncommitted changes. The
  typos hook renamed `nd` to `and` in `src/routing/DubinsRouter.cpp` and broke
  the build. The 62 source and test files among them were restored from HEAD.
  The other 72 (final SVGs, `artifacts/*/drc.json`, old scripts under
  `artifacts/logs/`, `prompt-*.md`) still carry the hooks' changes; their list
  is `artifacts/logs/sat-vs-feedline-2026-10-13/round29/hook-touched-files.txt`
  and `xargs git checkout -- < that file` reverts them. The user wants the
  function fixed before any lint work; do not run the full lint mid-experiment.
  Format own lines with the hook's binary:
  `git clang-format --binary ~/.cache/prek/hooks/python-l3u9P7bS1jVobIyZCrso/bin/clang-format -f -- <files>`.
- `uv sync --inexact --no-dev --no-build-isolation-package mqt-scpd` keeps the
  old extension after C++ edits; add `--reinstall-package mqt-scpd` (about five
  minutes) before any experiment script.
- Tests that cover this work, all green on 2026-10-15:
  `./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='InTurn.*:CapacityFlow.*:CouplerSession.*:*Capacity*'`
  (56 tests, plus `CouplerSession.ClosesEveryMouthThoughALeadStandsBeforeTheTip`,
  which plans 33q and takes six minutes) and
  `./build/release/test/grid/mqt-scpd-grid-test --gtest_filter='Bottlenecks.*'`.
  `EveryConnectionIsDrawn/17q` fails with this tree (f13 finds no way); the
  user parked it.

## The experiment

Everything lives in `artifacts/logs/sat-vs-feedline-2026-10-13/`.

- `experiment.py runs/<chip> round<N>/results/<chip> --walks 10 --per-verdict 12 --workers W`:
  phase A walks the coupler options at random in forked children and records
  the check's verdict of every complete state; phase B replays the base state,
  12 SAT and 12 UNSAT states, runs the check and the feedline routing on each,
  and stores `cases/caseNN.json` with the verdict, the lost wires, the local
  verdicts per chain and the routing's fails, and `caseNN-graph.json` with the
  capacity graph. Workers: 6 up to 21q, 4 on 33q, at most 3 on 45q and 57q and 2
  on 69q; ten forked 69q routings rebooted the Mac twice.
- `report.py round<N>/results -o round<N>/index.html --round "Runde N"` writes
  the page with one card per case and the graph pages under `graphs/`;
  `serve.py --dir round<N>` serves it on port 8766 and collects the user's
  feedback per case in `round<N>/feedback.json`. The four counts per chip:
  SAT0 and UNSATbad agree, SATbad is the check too lax, UNSAT0 the check too
  strict. `bad = unrouted + open + crossing`; a short resonator is no fail.
- `run_round<N>.sh` is each round's script; `run_round31.sh` and
  `run_round31_chain.sh` (the normal 21q run first) are the latest. Round 31 is
  current and served.
- Probes, all `python <probe> round<N> <chip> <case>` unless said otherwise,
  all needing the rebuilt package: `recheck.py round<N> <chip>` reruns the check
  on every stored case of a chip and prints one line per case and the counts;
  `graphdiff.py` rechecks one case and saves `<case>-graph-recheck.json`;
  `lostprobe.py` says why wires have no way; `stretchprobe.py ... [edge]...`
  lists the stretches; `flowprobe.py ... <wire>...` compares with the MILP
  (`SCPD_CAPACITY_FLOW=1`); `mouthprobe.py` prints the check's summary with the
  mouth notes; `gatecross.py <graph.json>...` counts gates that meet in the
  middle; `graphpage.py <graph.json> <out.html>` builds a graph page;
  `crop.py <round> <chip> <case> <graph.json> <cx> <cy> <R> <out.png>` renders a
  close-up with the routed wires and the gates; `crosses.py <results_dir> <case>
  <wire>...` says where a routed wire crosses its feedline edge;
  `addcases.py runs/<chip> round<N>/results/<chip> --verdict UNSAT --bad 0 --try 40`
  routes more walk states and keeps the ones that come out as wanted;
  `addstate.py runs/<chip> round<N>/results/<chip> <other_round_case.json>`
  adds one stored state of another round.

## The check as it stands

The check keeps the design of [handover-capacity-check.md](handover-capacity-check.md)
and adds, in the order the rounds brought them:

1. **Two stages** (user, round 17). Every chain is checked alone first
   (`SCPD_LOCAL_CHECK`, on): its edges as walls, the other chains absent. On a
   local UNSAT the option search goes on without the rule, as before. Then the
   chip is checked whole with the **terminal edges as wires**
   (`SCPD_TERMINAL_DEMANDS`, on): the edge from a launcher to its coupler is no
   wall but a demand from the launcher's slot to the coupler's slot.
2. **Terminal runs.** Each terminal edge keeps a run at its launcher (11
   straight cells) and at its coupler (21 axial or 15 diagonal steps plus
   `SCPD_TERMINAL_STUB_MARGIN`, 5, times the step length), marked as two flank
   walls with a slot between. The slot's mouth, tip to tip, is a gate of
   capacity 1 for the edge alone (`slotGates`), and the slot's chamber is erased
   from every other gate. The user counted the diagonal stubs and corrected both
   the lengths and the step length.
3. **Mouths between chains** (`mouthGates`, user, rounds 18 to 21). Between the
   ending edge of chain k and the starting edge of chain k+1, in chain order,
   the outer lines between the two coupler runs' slots close the mouth
   (`mouthLinesOf`): the lines from a tip of one run to a tip of the other that
   have the other two tips on one side, none within half a clearance of a tip
   that is not its own end. Two lines where the runs stand end to end, one
   between the two nearest tips where they stand side by side (21q f20/f21,
   17q f5/f6, f19/f0: the two lines ran over the slots and over each other).
   Every line is cut at the walls it crosses (`cutAtWalls`), each piece
   re-checked on its own raster line; no piece joins the two flanks of one run;
   no piece joins a run to the lead of its own coupler's resonator, but a line
   from the other run's tip may end on that lead (33q: lead 20 stands between
   f5 and f6, and refusing the line left the mouth open). A mouth piece of
   capacity 0 at the flank of its own terminal edge takes that edge.
4. **No gate meets another in the middle** (`grid::uncrossedBottlenecks`, user,
   2026-10-14). Two gates whose lines cross more than 1.5 cells from the ends of
   both, or one ending within 1.5 cells of the other's middle, lie beside the
   chambers on both sides of each other and let a wire over one of them unpaid.
   The slot's gate stays over the mouth's, the mouth's over the cut, the
   shorter of two cuts over the longer; each gate is set against the survivors
   in that order. `dropCrossingGates` applies it after the cuts, mouths and
   slots are assembled, in `analyseCapacity` and `capacityPass`.
5. **Notes.** `mouthGates` returns the gates and one note per mouth it could
   not close; the notes go into the check's summary (`graph.summary` in the
   JSON, the "Zusammenfassung" on the page), so an open mouth is visible.
6. **Crossing places** (`crossingStretches`). A cell of a feedline edge is a
   place when the band across it (`crossingReach` 11 cells each way) is free of
   other walls and port runs and the routing's crossing rule allows it; under
   `SCPD_STRETCH_BY_CLEARANCE` (on) no place lies within 11 cells of the edge's
   ends along it; under `SCPD_CROSSING_RECTANGLES` (on, user, round 23) the
   bands of the cells half a clearance along the edge either way have to be
   clear as well; under `SCPD_CROSSING_ROOM` (off) room past the band is asked
   too. Consecutive places with the same two chambers are one stretch; a place
   with the same chamber on both sides is none unless the chain is reflected.
   Under `SCPD_CROSSING_POINTS` (on) the stretch's capacity is the number of
   **points** on it, placed from the edge's start a lane spacing apart
   (`SCPD_CROSSING_SPACING` times the clearance, √2 by default, user, round
   16) and clear of leads and terminal runs (`SCPD_POINTS_CLEAR_OF_LEADS`).
   Without points the capacity is `(length − 11) / clearance + 1`, and without
   the clearance rule `length / pitch + 1`, the oldest form.
8. **The round switch** (user, 2026-10-15). `SCPD_CHECK_ROUND=N` sets every
   rule to the state it had after round N: the two stages from 17, the rip-up
   from 22, the rectangles from 24, the detour over the gates that take a wire
   from 25, the cut at walls from 27, the crossing rule from 30; each rule's
   own switch (`SCPD_LOCAL_CHECK`, `SCPD_TERMINAL_DEMANDS`, `SCPD_RIP_UP`,
   `SCPD_CROSSING_RECTANGLES`, `SCPD_DETOUR_TAKERS`, `SCPD_CUT_GATES_AT_WALLS`,
   `SCPD_UNCROSSED_GATES`) still overrides. The mouth rules of rounds 30 and
   31 have no switch; they matter only with the terminal edges as wires.
   `SCPD_CHECK_ROUND=16` is the check of round 16, the last before the two
   stages; the test `ChecksAsItDidInRoundSixteenUnderTheRoundSwitch` holds it.
7. **The demand router** (`checkInTurn`, `CapacityFlow.cpp`). Demands are routed
   one after another on the way with the fewest edges within the capacities; a
   demand without a `longest` takes its shortest way within the capacities when
   it is no longer than `SCPD_DETOUR_FACTOR` (1.5) times its shortest way over
   the gates that take a wire at all. After the first sweep the demands that
   overfill an edge are ripped up and routed again, backward then forward, with
   a relaxation that moves the demands off the overfilled edge, at most eight
   sweeps (user, round 21); `FlowCheck.sweeps` counts them. The MILP
   (`checkCapacity`, `SCPD_CAPACITY_FLOW=1`) is the reference.

## Where it stands (round 31, 2026-10-15)

| Chip | SAT0 | SATbad | UNSAT0 | UNSATbad | Base | Walk states complete / SAT |
|---|---|---|---|---|---|---|
| 4q | 13 | 0 | 0 | 10 | SAT | 36 / 26 |
| 9q | 13 | 0 | 0 | 0 | SAT | 161 / 161 |
| 17q | 12 | 0 | 2 | 11 | UNSAT | 126 / 28 |
| 21q | 0 | 0 | 1 | 13 | UNSAT | 310 / 0 |
| 33q | 5 | 8 | 5 | 7 | SAT | 456 / 141 |

The 21q UNSAT0 is `case13`, the routable state of round 29 added by
`addstate.py`; 40 more UNSAT states of the round 31 walks all route badly. On
4q to 21q the rounds since 16 removed every SATbad case. The normal final stage
of 21q (`run_round31_chain.sh`, log `round31/21q-normal.log`) now ends with
**2 open wires, 57 and f21**, where the run of 2026-10-11 ended with 0 bad:
the insertion finds no SAT state under the current rules and commits one the
routing cannot do.

The check costs 0.4 to 5 s per state on 33q, well under the routing.

## What is wrong, by class

1. **Too few crossing places** (21q every case, 33q cases 16 and 20). The
   router's crossing rule keeps 10 cells from an edge's ends and off its bends.
   The check's place needs, under the rectangle rule, the band of every cell
   within half a clearance along the edge to be allowed by that rule as well,
   so the first place lies 20 cells from an end or a bend, and the end zone
   adds its own 11. On 21q `case13` the router crosses f4 ten cells before its
   end and f23 fourteen cells after its start, where the check has no place;
   its stretches there are 7 and 2 cells long and take one wire where the
   router lays two. On 21q f11 has no place at all, so chain 2 is UNSAT alone
   in every state, and on 33q f7 (103 cells, two bends) has none in two cases.
   This is why 21q has no SAT state. The user's reading: the capacity estimate
   of a straight segment is too pessimistic. The likely fix: the rectangle's
   neighbour cells need to be free of walls only (the band without the rule),
   the rule applies to the crossing cell itself, and the end zone is the rule's
   10 cells, not 11 more. Not built; the user asked for the measurement below
   instead.
2. **The terminal edges' paths are no obstacle** (33q SATbad, 8 cases, always
   18 and 19). Two resonators from launchers between f5's and f6's launcher
   have to pass along f5's terminal edge into the chip; that corridor holds one
   wire, the check lets four through the mouth (capacity 14). A terminal edge
   is a demand, its routed path divides no chamber. Proposal, the user's to
   decide: the routed path of a terminal edge counts as a wall when the cuts
   are measured, the edge stays a demand and a user of those cuts.
3. **A pocket behind a chosen stretch** (33q cases 17 and 19). Wire 12 crosses
   f2 into a small chamber between f2 and f3 at their coupler and leaves it
   only through a gate of 18.9 cells (capacity 0); the router crosses f2
   elsewhere. The chamber structure at a coupler where two edges meet.
4. **17q cases 17 and 18**: lost at the gates between f18 and the artwork; the
   user called it an inaccuracy and let it stand. **21q**: the pocket in front
   of 56's target run, where 56, 57 and f21 need three ways and the two gates
   there take one each; check and routing agree that it fails.

## The older capacity rule, measured (user, 2026-10-15)

The user's reading of the 21q cases: the capacity estimate of a straight
segment of a feedline edge is too pessimistic. Measured with `recheck.py` on
the stored cases of round 31 (14 on 21q with `case13`, 25 on 33q and 17q), the
counts SAT0 / SATbad / UNSAT0 / UNSATbad, the rules switched by environment:

| Variant | Switches | 21q | 33q | 17q |
|---|---|---|---|---|
| as it stands | all on | 0 / 0 / 1 / 13 | 5 / 8 / 5 / 7 | 12 / 0 / 2 / 11 |
| 1 old capacity formula | `SCPD_STRETCH_BY_CLEARANCE=0` | 0 / 0 / 1 / 13 | 5 / 8 / 5 / 7 | 12 / 0 / 2 / 11 |
| 2 and no crossing points | + `SCPD_CROSSING_POINTS=0` | 0 / 0 / 1 / 13 | 5 / 8 / 5 / 7 | **14 / 0 / 0 / 11** |
| 3 and no rectangles (the old method) | + `SCPD_CROSSING_RECTANGLES=0` | 1 / 5 / 0 / 8 | **9 / 8 / 1 / 7** | 12 / 0 / 2 / 11 |
| 4 no rectangles alone | `SCPD_CROSSING_RECTANGLES=0` | 0 / 3 / 1 / 10 | stopped at 15 of 25 | not run |
| 5 round 16 | `SCPD_CHECK_ROUND=16` | 0 / 3 / 1 / 10 | stopped at 18 of 25 | 9 / 8 / 5 / 3 |

Variant 1 with points on changes nothing: with points the capacity is the
number of points, the formula only decides where places lie near the ends.
Variant 2 resolves both 17q UNSAT0 (cases 17 and 18, the gates at f18) with no
new SATbad and leaves 21q and 33q as they are. Variant 3 is the only one that
makes the routable 21q `case13` SAT, at the price of five SATbad (the base,
cases 01, 03, 08, 11: 57/f21 and 62/63); on 33q it resolves four of five
UNSAT0 with no new SATbad; on 17q it falls back to two UNSAT0. Variant 4 is
laxer in the wrong places on 21q (three SATbad) and leaves `case13` UNSAT. No
variant is better everywhere; the rectangle is where the pessimism sits, and
dropping it whole is too lax.

The user stopped the last three runs on 2026-10-15; their partial lines are
in `round31/variants/33q-round16.txt` and `33q-norect.txt`, one line per
case, and `17q-norect.txt` never ran. `round31/variants/run_rest.sh`, `run_norect_rest.sh` and
`run_round16.sh` are the scripts; a variant of 21q takes about 25 minutes, 33q
and 17q about eight each, three in parallel are fine.

The full run of the capacity suites with the round switch
(`InTurn.*:CapacityFlow.*:GapCapacity.*:CouplerSession.*:*Capacity*`, 60
tests, and the grid `Bottlenecks.*:BottleneckCapacity.*`, 16 tests) was green
at the end, 2026-10-15.

## The tests

Three layers, all GoogleTest unless said otherwise, all in the test tree of the
component that owns the behaviour.

- **The demand router**, `test/pipeline/test_capacity_flow.cpp`, suites
  `InTurn`, `CapacityFlow` and `GapCapacity`. Small graphs built by hand
  (`nearAndFar`, a few chambers and edges) with one property per test: a wire
  crosses every feedline it is prescribed once, a feedline with nowhere to
  cross leaves no way, a wire of fixed length takes its shortest way measured
  centre to centre and refuses a way longer than it may run, a wire whose short
  way is full goes no further than its detour, the rip-up takes an earlier wire
  off a full gate for one with no other way, and the detour is measured against
  the gates that take a wire. Runs in a second:
  `./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='InTurn.*:CapacityFlow.*:GapCapacity.*'`.
- **The gates**, `test/grid/test_bottlenecks.cpp`, suite `Bottlenecks`.
  `DropsTheLowerOfTwoThatMeetInTheMiddleOfEither` draws lines on a 40-cell grid
  and checks the precedence (slot over mouth over cut, shorter over longer),
  the touch rule, that two lines from one end both stay, and that a gate meeting
  only a gate that went itself stays:
  `./build/release/test/grid/mqt-scpd-grid-test --gtest_filter='Bottlenecks.*'`.
- **The check on the benchmark chips**, `test/pipeline/test_final_router.cpp`,
  suite `CouplerSession`. Each test plans a chip from `test/pipeline/Benchmarks.hpp`
  (`fourQubit()`, `nineQubit()`, `seventeenQubit()`, or `benchmarkOf("33q")`
  for any chip directory; `plan(benchmark)` runs the capacity, global,
  assignment and detail stages), builds a `CouplerSession`, and reads
  `session.insertionCheck()` as JSON: `verdict`, `lost`, `local[]` with one
  verdict per chain, and `graph` with `edges` (`kind` bottleneck or stretch,
  `line`, `ends`, `chambers`, `capacity`, `load`, `through`, `mouth`, `slot`,
  `points`), `wires`, `summary`, the `raster` (run-length, 0 free, 1 to 5 the
  wall kinds, 6 and up a chamber) and `clearance`. `session.routeFeedlines()`
  gives the routed wires with their `path` for tests of the routing. The
  properties tested, by name:
  `ChecksEveryChainOnItsOwnFirst`, `SearchesEveryChainAgainstItselfAlone`
  (the two stages), `CarriesTheTerminalEdgesAsWiresInTheCheckOfTheChip` (the
  terminal demands, the stub lengths with and without the margin),
  `ClosesTheSlotOfEveryTerminalEdgesRunForTheEdgeAlone`,
  `KeepsTheGatesAtTheRunsOfTheTerminalEdges`,
  `ClosesTheMouthBetweenTwoChainsWithTwoLines` (none on 4q, two lines on 9q,
  no flank to lead, no terminal edge lost),
  `ClosesAMouthWhoseLinesWouldRunOverTheSlotsWithOneLine` (no tip with two
  mouth ends, no mouth line within half a clearance of a foreign tip),
  `ClosesEveryMouthThoughALeadStandsBeforeTheTip` (17q and 33q, no "no mouth"
  note in the summary), `CutsEveryGateAtTheWallsItCrosses` (every gate's
  raster line free between its ends), `LetsNoGateMeetAnotherInTheMiddle`
  (every pair of gates on 4q, 9q, 17q), `KeepsTheGapsBetweenALeadAndItsEdges`,
  `KeepsTheCrossingPointsClearOfTheTerminalEdgesRuns`,
  `KeepsARectangleClearAroundEveryCrossing`,
  `PacksTheCrossingsOfAnEdgeALaneSpacingApart`,
  `DrawsTheCapacityGraphAfterTheBottlenecks`. A 4q test takes seconds, 9q and
  17q about a minute each, 33q six minutes (the planning); the filter
  `'CouplerSession.*'` runs them all in about ten minutes.
- **The method of the rounds.** Every rule came in red first: a property on
  the JSON that the current graph breaks (a crossing pair, a mouth over a slot,
  a missing mouth), then the rule, then the graph of the failing round's case
  checked directly with `graphdiff.py` or `recheck.py` before a new round. For a
  case the fixtures do not hold, `benchmarkOf("<chip>")` plans that chip in
  the test; the planned state is the planner's default, which can differ from
  the experiment's base in where a rule bites (33q: the test lost the mouth at
  f27/f28, the experiment at f5/f6).
- **The Python side.** `test/python/unit/test_dashboard.py` covers the
  dashboard of the earlier session; the probes under the experiment folder are
  tools, not tests, and run only with the rebuilt package:
  `uv run --no-sync python <probe> ...`.

## The traps

- Chambers are separated by walls and by the gate lines; a gate beside more
  than two chambers lets a wire hop between two chambers on one side of it.
  The crossing rule removes the cases from crossing gates; gates that fan out
  from one tip still share wedge chambers.
- `cutAtWalls` walks the raster line of the whole line; a piece's own raster
  line runs through other cells, so every piece is re-cut on its own line.
- A gate's ends after cutting sit on wall cells next to a tip, not on the tip;
  the touch rule (1.5 cells) tolerates that.
- The check is quiet: `progress_` is nulled during a check, `say` and `tell`
  print nothing there. Diagnostics go into the JSON (`summary`, `local`) or
  the probes.
- `set_option` on a coupler option that is not open is refused; replay a state
  as `addstate.py` does, from the base, only the options that differ.

## What is not done

- Class 1 above, the fix to the places; then round 32 for 4q to 33q, and the
  normal 21q run back to 0 bad.
- The user's decision on class 2, and the chamber structure of class 3.
- 45q, 57q and 69q were last run in rounds up to 16; not rerun with the
  rules since.
- The lint reverts listed under State, `UPGRADING.md`, and the parked
  `EveryConnectionIsDrawn/17q`.
