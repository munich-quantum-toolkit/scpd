# The slot check of the coupler insertion

Written for whoever makes the slot check fast and builds it into the chain
search of the coupler insertion. It covers 2026-10-16 to 2026-10-19: what the
check does, every rule and switch it has, how well it agrees with the feedline
routing, how fast it is, the experiment around it, and the plan for the next
two steps.

Read [handover-capacity-graph-sat-check.md](handover-capacity-graph-sat-check.md)
first for the review rounds before 2026-10-16, and
[handover-capacity-check.md](handover-capacity-check.md) for the check by
stretches inside the insertion.

## State

- Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`, HEAD `0d8eba0`. Everything below is
  **uncommitted**.
- Files of this work: `src/pipeline/SlotCheck.cpp`,
  `include/mqt-scpd/pipeline/SlotCheck.hpp`, `src/pipeline/FinalRouter.cpp`
  (the end verdict, the insertion fixes, the meeting fix),
  `bindings/bindings.cpp` (`slot_check`), `test/pipeline/test_slot_check.cpp`,
  `test/pipeline/test_final_router.cpp`, `CHANGELOG.md`.
- Tests: `./build/release/test/pipeline/mqt-scpd-pipeline-test --gtest_filter='SlotCheck.*'`
  (13 tests, green) and `--gtest_filter='CouplerSession.*'` (25 tests; two
  fail since before this work:
  `ClosesTheMouthBetweenTwoChainsWithTwoLines` and
  `ClosesAMouthWhoseLinesWouldRunOverTheSlotsWithOneLine`; they describe the
  mouths before commit `0aa06c7` and wait to be brought up to date).
- `uvx nox -s lint` has not run. It rewrote 134 unrelated files once (see the
  handover above); format own lines with `git clang-format` only.
- After a C++ edit, rebuild the extension with
  `uv sync --inexact --no-dev --no-build-isolation-package mqt-scpd --reinstall-package mqt-scpd`;
  without `--reinstall-package` the old extension stays.
- **The Mac crashes under load.** Forked replays of 45q with three workers
  rebooted it on 2026-10-19, as 69q with two did before. Replay 45q, 57q and
  69q with one worker, 33q with two, and run nothing heavy beside them.

## Where the check sits

1. The insertion builds the capacity graph of the chip as it stands: walls,
   port runs, chambers, gates, mouths and the stretches of every feedline edge
   (`capacityPass`, `FinalRouter.cpp`). The check by stretches judges it
   (`checkInTurn`).
2. Under `SCPD_SLOT_CHECK` (on) the insertion hands the graph to
   `slotCheck(const SlotGraph&)` (`SlotCheck.cpp`) as a typed structure
   (`slotGraphOf`, `slotGraphOfPass` in `FinalRouter.cpp`), with no JSON on
   the way. The check replaces the stretches by crossing slots and routes
   every wire again. Its verdict is the verdict of the whole chip in
   `writeCapacityGraph` and in `insertionCheckJson` (the coupler session).
3. Under `SCPD_SLOT_CHECK_STEPS` (on) the chain search uses it too: every
   call of `capacityVerdictOf` runs the slot check, which covers each step
   of a chain search, the check of every chain on its own and the global
   check. A wire is lost when the check gives it no way or its way passes
   an edge or self-bottleneck that is over (`SlotCheckResult::crowded`). When
   the check throws `invalid_argument`, the check by stretches decides.
4. The graph JSON is written only for a page: under `--debug`, for the
   coupler session and for `pyscpd.slot_check(graph_json) -> json`, which
   reads the JSON through the adapter `slotGraphOf(std::string_view)`.

An edge of an open chain is prescribed to no wire, but its stretches stay
open to the wires that bridge it (`bridgers_`). Its slots stay open to the
same wires: they may cross there without a prescription. Without this rule
17q refused 4776 of 5488 steps and left f0 undrawn.

## What the check does, in order

`Checker::run` (`SlotCheck.cpp`):

1. **Read the graph.** The raster (run lengths: 0 cut line, 1 to 5 walls,
   6 + c chamber c) becomes a `Grid`; strokes, wires, gates and stretches are
   read (`readStrokesAndWires`).
2. **Gates.** Every bottleneck keeps its line and its chambers. Its capacity
   is counted again (see *Capacity* below) unless the check by stretches gave
   it another (a mouth, an own run's passage). A gate too narrow for a
   resonator at its own run keeps capacity 0 for it (`SCPD_RESONATOR_PASSAGE`).
   A plain gate on the line of a mouth between the same two chambers becomes
   that mouth (`markMouthTwins`, `SCPD_MOUTH_TWINS`).
3. **Sides.** A gate joins only a chamber on one side of its line to one on
   the other side, sampled a cell and a half off the line (`addSides`).
4. **Slots.** `SlotPlacer` lays crossing slots on every feedline stroke: a
   rectangle on one straight piece, the crossing reach to either side of the
   middle line and one clearance along it, covering only free cells and the
   edge's own wall, no port run's slot and no other slot. It needs a line
   across the edge that the router's crossing rule allows
   (`SCPD_SLOT_CROSSING_RULE`): the rule is
   `routing::CrossingConstraints`, rebuilt from the feedline strokes without
   the terminal edges, with the router's reach (the graph's
   `crossingReach` less one). Slots are laid from both ends; the end that
   gives more counts. A slot is an edge of capacity 1 between the chambers
   on its two sides, open to the wires prescribed to cross that feedline.
5. **Self-bottlenecks** (`S …`): a free line between two places of one
   feedline edge with more slots behind it than it holds wires, and a free
   line from a slot corner to the open end of a stub that closes a room. A
   wire passes one when the count of its slots and ends behind the line is
   odd.
6. **Owners** (`findOwners`): a gate with an end on a wire's own port run
   holds that wire half a clearance more when the run's middle line, carried
   on, meets the gate (`limitOf`).
7. **Mouths** (`findMouths`): the mouths between chains, named by their
   terminal edges; which plain wires belong to a mouth by their launcher's
   place round the chip. A wire with a prescription passes no mouth; a
   terminal edge and the plain wires of a mouth pass that mouth and no
   other; a mouth piece of unknown mouth bars nobody.
8. **Ways.** Per wire a Dijkstra over states (edge, chamber, layer), the
   layer holding the prescribed feedlines crossed and the own mouth passed
   (`shortest`). A way may be at most 1.5 times its shortest way, both
   pulled tight.
9. **Negotiation.** PathFinder for at most 60 rounds: present and history
   costs on edges that are over. SAT when every wire has a way and no edge
   and no self-bottleneck is over; UNSAT otherwise, with the lost wires and
   the edges over.
10. **Output.** The graph again as JSON, with slots, loads, ways, `verdict`,
    `summary` and `crossSlots`.

## Capacity

A gap of `L` cells across which the wires cross:

- `L` is counted across the wires' heading: `L cos(phi)`, `phi` the angle
  between the line's normal and the nearest of the eight headings, at most
  22.5 degrees (`headingShare`, `SCPD_GATE_HEADING`).
- `n = floor(L cos(phi) / c)` with `c` the clearance in cells (19 on every
  chip; the rule itself is 18.5 to 18.8 cells).
- From three wires on (`SCPD_CAPACITY_MARGIN_FROM` 3) the clearance counts
  1.06 times (`SCPD_CAPACITY_MARGIN`): `floor(L cos(phi) / (1.06 c))`. A gap
  of three may drop to two this way; the user wants it so in the C++ check.
- A gate at a wire's own run: `max(n, floor((L cos(phi) - c + k c / 2) / (1.06 c)) + 1)`
  for `k` own ends (`limitOf`); here the factor holds for any count.

## Switches

All read from the environment, all on by default. `=0` (or the old value)
gives the behaviour before.

| Switch | Default | Where | What |
|---|---|---|---|
| `SCPD_SLOT_CHECK` | on | FinalRouter | the slot check gives the end verdict |
| `SCPD_SLOT_CHECK_STEPS` | on | FinalRouter | the slot check judges every step of the chain search and every chain on its own |
| `SCPD_CAPACITY_MARGIN` | 1.06 | SlotCheck | factor on the clearance from `…_FROM` wires on |
| `SCPD_CAPACITY_MARGIN_FROM` | 3 | SlotCheck | fewest wires the factor applies to |
| `SCPD_GATE_HEADING` | on | SlotCheck | count a line across the wires' heading |
| `SCPD_RESONATOR_PASSAGE` | 0.8 | SlotCheck | narrowest own-run passage for a resonator, in clearances |
| `SCPD_SLOT_CROSSING_RULE` | on | SlotCheck | a slot needs a line the router may cross on |
| `SCPD_MOUTH_TWINS` | on | SlotCheck | a gate on a mouth's line is that mouth |
| `SCPD_EDGE_SLOT_CLEARANCE` | on | FinalRouter | two edges of one coupler keep the clearance in the slots outside their meeting; a coupler session lets an edge that breaks it stand down |
| `SCPD_MEET_AT_THE_MIDDLE` | on | FinalRouter | the fence judges a meeting at the middle of a pair, as the count of fails does |
| `SCPD_TERMINAL_SLOT_CLEARANCE` | on | FinalRouter | a terminal edge keeps the full clearance from its neighbour in its slots |
| `SCPD_RELEASED_EDGES_GONE` | on | FinalRouter | a released terminal edge loses its fence only |

The last four change the insertion and the feedline routing, not the check;
they came out of cases where the routing failed for a reason the check could
not see.

## How well it agrees with the feedline routing

Rounds 53m, 54m and 55m are the cases of rounds 53 to 55 routed again with
the routing as it stands (`run_meet.sh`); a case counts when the insertion
draws every edge.

| Round | Cases | Right | SAT, bad > 0 | UNSAT, 0 bad |
|---|---|---|---|---|
| 53m | 136 | 129 | 0 | 7 |
| 54m | 111 | 107 | 0 | 4 |
| 55m | 125 | 117 | 0 | 8 |
| all | 372 | 353 (95 %) | **0** | 19 |

Every wrong verdict is too careful. Nine cases of 9q could not be replayed
(their options no longer exist), 28 cases are no longer complete under
`SCPD_EDGE_SLOT_CLEARANCE` (8 of 13 on 57q of round 55m).

## How fast it is

Measured on 2026-10-19 over every graph of rounds 53m to 55m
(`timing_cpp.py`), one call of `pyscpd.slot_check`, median:

| Chip | SAT | UNSAT | graph JSON |
|---|---|---|---|
| 4q | 0.11 s | 0.16 s | 0.2 MB |
| 17q | 0.19 s | 0.23 s | 0.8 MB |
| 33q | 0.20 s | 0.29 s | 1.4 MB |
| 45q | 0.27 s | 0.40 s | 1.9 MB |
| 57q | 0.35 s | 0.54 s | 2.3 MB |
| 69q | – | 0.60 s | 2.8 MB |

The chain search checks every step with the check by stretches today
(benchmark logs of `meet-middle-2026-10-18`, `[Capacity Step]` lines):

| Chip | Step checks | Time in them | Per check | Insertion |
|---|---|---|---|---|
| 17q | 187 | 1.5 s | 8 ms | 5.6 s |
| 21q | 2261 | 28.6 s | 13 ms | 76 s |
| 33q | 576 | 10.8 s | 19 ms | 33 s |
| 45q | 1417 | 31.4 s | 22 ms | 109 s |
| 57q | 1008 | 27.0 s | 27 ms | 100 s |
| 69q | 2451 | 111 s | 45 ms | 374 s |

Since then the check got faster, each step checked against a golden file
of 809 graphs (`golden_slotcheck.py`, verdict, lost wires, edges over, loads,
ways and slots; 0 differ after every step):

- the switches read once per check (`Settings`);
- squared distances instead of `hypot`, and the crossing rule tested before
  a slot's cells are gathered;
- tables for the owner limits, the owners and the shortest tight ways;
- the stub cells collected while the raster is decoded, and the second room
  fill stopped at the size of the first;
- the Dijkstra over flat state arrays instead of maps;
- the typed `SlotGraph` instead of JSON, and no output graph without a page.

A 69q UNSAT graph went from 0.60 s to 0.13 s a call. 4q still takes about
88 ms; most of it is the room fills of the self-bottlenecks (394 fills,
17 million cells). The typed and the JSON way judge alike on every replayed
step of 4q, 9q and 17q (`typed_vs_json.py`).

## The experiment

Everything lives in `artifacts/logs/sat-vs-feedline-2026-10-13/` (git
ignores it).

- `experiment.py`, `replay_all.py` (with `--resume`), `rejudge_offline.py`:
  make, replay and judge cases; `report.py` builds a round's page
  (`--cdt-graphs`, `--cdt-label` put the prototype beside the C++ check);
  `serve.py --dir <round> --port <p>` serves it.
- `run_meet.sh`, `rejudge_m.sh`: rounds 53m to 55m. Their pages are served on
  ports 8770, 8771 and 8772; the rounds before on 8767 (53), 8768 (54) and
  8766 (55).
- `ksweep_cpp.py`: the C++ check at several factors over every case.
- `timing_cpp.py`: the timing table above.
- `caselog.py`, `dashall.py`: the routing log and the feedline routing
  dashboard of one case.
- `newcase.py`: replace one case of a round by another complete state of the
  same walk verdict.

## The CDT prototype

`cdt/` holds a second way to build the graph: a constrained Delaunay
triangulation over the wall outlines, with the same decision on top and an
ILP (HiGHS) that gives the verdict. Its best form, z1r1 (no allowance at the
cut ends, cuts between two static obstacles unbounded, the crossing rule,
`cos(phi)`, factor 1.06 from three and never below three) gets 333 of 372
right, with no case SAT that fails; with the end allowances of the C++ check
it gets 339, with two such cases. It is slower (0.4 to 7 s a call in Python)
and judges worse than the C++ check, so the C++ check goes into the
insertion. `cdt/README.md` and `cdt/PROGRESS.md` hold its stages.

## Next steps

1. **Faster steps.** Between two steps of one chain, lay the slots again
   only on the edges that changed and start the negotiation from the ways of
   the step before; cap the room fills.
2. **The global failures.** 45q passes with the slot check in the steps;
   69q still ends with g223 and g225 over (chain 9, wires 183 and 184)
   although chain 9 found its options without a closed wire. Table:
   `local_global.py <arm> ...` in the experiment folder.

## Traps

- The switches of `SlotCheck.cpp` are read once per check (`readSettings`),
  not at start-up, so one process can still compare settings.
- `test/pipeline/zz_profile_slot_check.cpp` is a profiling harness
  (`SCPD_PROFILE_GRAPH`); delete it before the work is committed.
- The prototype's `cdt/py.sh` has no `highspy`; `cdt/ilp.sh` has.
- Scratch files under `/private/tmp` vanish when the Mac restarts; keep
  helpers in the experiment folder.
- A coupler option index of a stored case may point elsewhere after the
  options change; `replay_all.py` records `replayMatches` per case.
