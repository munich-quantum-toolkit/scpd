# Local and global SAT in the coupler insertion

Written for whoever goes on with the capacity check of the coupler insertion
after 2026-10-21. It covers 2026-10-20 and 2026-10-21:

- the escape rules of the slot check;
- the local test (the local slot check against the local rip-up);
- the global rounds built from locally SAT chain states;
- the global SAT search in the normal run, which makes all eight benchmarks
  globally SAT;
- the faster slot check behind `SCPD_SLOT_CHECK_FAST`;
- how to run every test again.

Read [handover-slot-check.md](handover-slot-check.md) for the slot check
itself and [handover-local-sat.md](handover-local-sat.md) for the local
comparison as it stood on 2026-10-19.

## State

- Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`.
- HEAD `0c18a7b` ("improved local/global sat check") holds the global SAT
  search, the fast slot check and the switch. Commit `ba2b707` before it holds
  the escape rules and the local test.
- Not committed: `*-final.svg`, `COMMIT_MSG.txt`, this file and the two older
  handovers.
- Still to do:
  - `CHANGELOG.md` and `UPGRADING.md`: no entry yet for the fence fix, the
    global SAT search and `SCPD_SLOT_CHECK_FAST`.
  - `uvx nox -s lint` has not run. It once rewrote 134 unrelated files; format
    only your own lines with `git clang-format`.
  - Tests for the global SAT search.
- Tests: `SlotCheck.*` 18/18 green, including `FastCheckFindsTheSame`;
  `test/routing/test_crossing_constraints.cpp` 6/6 green. See *Known test
  failures* for the rest of the pipeline tests.

## Results

### Benchmarks, normal run, all defaults

Arm `artifacts/logs/bench-globalsat-both-2026-10-21`.

| Chip | Angle cost | Global | Bad wires | Time | Global SAT search | Length fails |
|---|---|---|---|---|---|---|
| 4q | 8 | SAT | 0 | 6 s | not needed | short 6, 9 |
| 9q | 20 | SAT | 0 | 11 s | not needed | – |
| 17q | 49 | SAT | 0 | 59 s | chain 0 alone, 8th combination, 25.5 s | short 1, 26, 44, 55 |
| 21q | 52 | SAT | 0 | 69 s | not needed | – |
| 33q | 76 | SAT | 0 | 126 s | not needed | short 80 |
| 45q | 107 | SAT | 0 | 312 s | chain 6 alone, 1st combination, 81.8 s | long 51 |
| 57q | 134 | SAT | 0 | 323 s | not needed | – |
| 69q | 176 | SAT | 0 | 1387 s | chain 9 alone, 1st combination, 238.4 s | short 96; long 3, 111 |

- "Bad wires" are unrouted, open or crossing wires. There are none.
- Every length fail was there before the global SAT search
  (`bench-relaxboth-2026-10-21`). They belong to the length tuning of the
  feedline routing, not to the capacity check.
- Before the global SAT search, 45q was UNSAT (g135). 69q was UNSAT (g222,
  g224) with open wires 183 and 184.

### Runtime with `SCPD_SLOT_CHECK_FAST=1`

Arm `artifacts/logs/bench-slotfast-2026-10-21`, compared by `phase_times.py`.

| Chip | Final stage, old | fast | Factor |
|---|---:|---:|---:|
| 4q | 6.1 s | 5.7 s | 1.06× |
| 9q | 11.2 s | 10.6 s | 1.05× |
| 17q | 59.2 s | 44.2 s | 1.34× |
| 21q | 68.8 s | 59.6 s | 1.16× |
| 33q | 126.1 s | 106.8 s | 1.18× |
| 45q | 311.9 s | 245.7 s | 1.27× |
| 57q | 323.3 s | 275.1 s | 1.18× |
| 69q | 1387.1 s | 1155.6 s | 1.20× |

Phases of 69q:

| Phase | old | fast |
|---|---:|---:|
| outer routing | 37.3 s | 37.3 s |
| outer refinement | 168.4 s | 168.8 s |
| chain search | 674.4 s | 492.8 s |
| ... of it step checks | 392.4 s | 210.5 s |
| global SAT search | 238.5 s | 188.1 s |
| ... of it slot checks | 88.5 s | 31.0 s |
| feedline routing | 163.6 s | 160.5 s |
| feedline refinement | 101.6 s | 105.2 s |

- One slot check alone is 9 to 14 times faster: 6.7 ms on 4q, 17 ms on 33q and
  29 ms on 45q, against 92, 204 and 281 ms before.
- The outputs are byte-equal on 284 files, from 140 graphs of 4q to 33q and two
  of 45q.
- In the benchmarks, every result line without times is equal on 4q to 33q.
  On 45q, 57q and 69q the chain search checks a few more steps in its time
  budget (45q chain 2: 438 instead of 433). The chains, the angle costs, the
  global SAT state and the routing are still the same.
- The switch is off by default.
- What is left is not the slot check:
  - the edge searches and the stretch pass (`capacityPass`) of every step in
    the chain search;
  - the segment search in the global SAT search;
  - the routing phases.

  An incremental check would start there (see *Next steps*).

## The check chain of the coupler insertion

1. **Chain search.** Every step is checked by the slot check
   (`SCPD_SLOT_CHECK_STEPS`). Only the wires of the chain's own feedline are
   routed (`SCPD_LOCAL_SAT_WIRES`, `chainWires`); every terminal edge stays in
   the graph.
2. **Commit.** Every chain is laid on the chip.
3. **Whole-chip check** (`wholeChipLost`). This is the check of
   `insertionCheckJson`: every chain standing and the terminal edges as wires.
   A chip with an undrawn edge counts as UNSAT.
4. **Global SAT search** (`searchGlobalSat`, `SCPD_GLOBAL_SAT_SEARCH`, on). It
   runs only when step 3 says UNSAT.

### The global SAT search

1. **Which chains to search.** `wholeChipLost(wires, &touching)` reads the
   edges over capacity (`SlotCheckResult::over`). It finds their entries in
   the `SlotGraph` the check read, by name, and takes every `f<k>` from the
   name, `feedline` and `ends` of each entry. The chains that own those
   feedline edges are the **focus**, with the waypoints at both ends of each
   edge. Example: 45q g135 lies between f41 and the artwork near Qb36. f41 is
   chain 6, edge 3→4, so the focus is chain 6 at waypoints 3 and 4.
2. **Collect states** (`collectChainStates`). Per chain, the segment search of
   the targeted repair (`researchSegment`) yields up to `SCPD_GLOBAL_SAT_RUNS`
   (8) runs of options per segment, through the collector `segmentCollect_`.
   - Other chains: only the two end segments. Only states whose first or last
     coupler moves are kept, up to `SCPD_GLOBAL_SAT_STATES` (8), new end pairs
     first.
   - Focus chains: also every segment that holds a marked waypoint. Every
     locally SAT state is kept, with no end filter and no limit. States that
     move a marked waypoint come first.
   - A run with an edge that has no way is no state.
3. **Two local checks.** A state is kept when either check finds it SAT.
   - **With the neighbours standing.** This is the check of the local test
     (`slotCheckNow`): every other chain stands as the commit left it, the
     terminal edges are wires, and only the chain's own wires and terminal
     edges are routed and judged. Such a state can stand on its own; 45q
     needs it.
   - **Chain alone.** Only the chain stands and its own wires are routed. Such
     a state may need its neighbours to move as well; 17q needs it.

   States the first check finds SAT come first.
4. **Combine.**
   - Every combination starts from the snapshot of the commit (`snapshot`,
     `restore`, `chainEdgePaths_`).
   - The couplers move as the coupler session moves them (`setCouplerOption`).
     Every edge must be drawn, and the whole chip is checked as in step 3.
   - First every state of a focus chain alone, the others as committed. Then
     random combinations: 100 %, 75 % or 50 % of the chains on another state,
     focus chains always.
   - The first SAT combination stays. Without one, the chip goes back to the
     commit.
5. **Budgets.**
   - The collection has its own limit, `SCPD_GLOBAL_SAT_COLLECT_SECONDS` (600).
     Chains not reached keep their state.
   - The combinations have `SCPD_GLOBAL_SAT_SECONDS` (60) and
     `SCPD_GLOBAL_SAT_TRIES` (200), counted from the end of the collection.

The log prints:

- per chain, runs, segments, locally SAT states (how many with the
  neighbours standing), kept states, end pairs, seconds and seconds in slot
  checks;
- at verbosity 1, every state of a focus chain with its options and the wires
  it loses;
- every combination with what it loses.

### How the search got there (45q)

1. Round 59 found 90 SAT whole-chip states of 45q. Every SAT state moves
   coupler 36 (chain 6, waypoint 4) to option 7 or 2. The ends of chain 6 stay
   6/12.
2. The first search in the normal run kept only states with **other ends**,
   from the two end segments only. That excluded this move twice: chain 6 has
   8 waypoints, its end segments are 0..3 and 4..7, and the move keeps the
   ends.
3. Searching all segments at the edge over found the move. The limit of 8
   states then cut it off: the states of segments 0 to 2 came first.
4. Without the limit, the collector's local check still refused it. That check
   stood the chain alone, without the terminal edges as wires, and judged
   every lost wire. The local test stands every chain and judges only the
   feedline's own wires.
5. `replay_round59.py` replayed round 59 on today's build. The commit options
   are identical. Chain 6 alone is SAT; the same cases without chain 6 are
   UNSAT.
6. With the check of the local test, 45q became SAT, but 17q fell back to
   UNSAT. 17q needs several chains to move together, and the stricter check
   refuses such states. Both checks together solve both chips.

## Rules of the slot check changed in this period (defaults)

| Switch | Default | What |
|---|---|---|
| `SCPD_SLOT_ESCAPE` | on | A slot is placed only where both of its ends have a way out (`EscapeWays`). A way out is a router turn (`MovePrimitives(5)`) or a straight. |
| `SCPD_ESCAPE_STRAIGHT` | 5 | Cells of the straight way out. |
| `SCPD_ESCAPE_QUARTER_TURNS` | on | 90° turns count as ways out. Banning them did not help the 45q case. |
| `SCPD_ESCAPE_STUB_MARGIN` | 4 | Extra cells a way out keeps from stub walls, beyond clearance minus inflation. 4 flipped a 45q SAT-bad case to UNSAT; 1 to 3 do not. |
| `SCPD_ESCAPE_APART_MARGIN` | 0 | Extra cells between two ways out. |
| `SCPD_ESCAPE_COUPLER_RUNS` | off | Rule tried and dropped: a way out may not cross the run between the two feedline ports of a coupler. |
| `SCPD_SLOT_PIN_CELLS` | 10 | Cells at each end of an edge where no slot lies (the router's pin zone, `CrossingConstraints::PIN_CELLS`). 5 was too optimistic: the router measures the middle of the crossing, the check the side of the slot. |
| `SCPD_SLOT_STUB_GAP` | 0 | Extra gap between a slot and a stub. |
| `SCPD_OWN_RUN_BONUS` | off | The gate capacity bonus of half a clearance for a wire's own port run. Removed: it made 69q g222 hold 5 where the router passes 4. |
| `SCPD_RELAX_BOTH` | on | Feedline routing: a wire that fails every relaxation level in two rounds in a row gets one more level that releases both ring neighbours and fences slot ±2 (69q chain 10, wires 206/207). |

Changes without a switch:

- **Fence rule.** A feedline edge is fenced only by edges of its own chain. This
  covers `setCouplerOption` and the segment search. `fenceEverything_` no
  longer fences other chains. On 69q, round 59 then had 50 of 80 complete
  states instead of 7.
- **Segment search clearance.** `optionClearOf` keeps the coupler options of a
  segment clear of the edges laid before them.
- **The local test's plain mode** (`SCPD_LOCAL_AGREEMENT_PLAIN`) keeps the
  terminal edges in the window.

The cap graph pages draw these layers:

- the escape ways;
- the escape inflation (9.5 cells to each side);
- the stub walls of the ways out;
- the coupler runs (red hatch);
- the terminal edges.

## The local test

The test compares the local slot check of a feedline with a local
rip-up-and-reroute of its wires. It runs in phase 4 in place of the repair.
For the definitions (window, local SAT, rip-up) see
[handover-local-sat.md](handover-local-sat.md).

### Result with today's defaults

Arm `local-nobonus-2026-10-21`: the stored runs of `local-sat-pin5` (4q to
33q) and `local-sat-pin10` (45q to 69q), judged again. Page on port 8788.

| Chip | Runs | SAT, clean | SAT, rip-up fails | UNSAT, clean | UNSAT, fails | Agree |
|---|---|---|---|---|---|---|
| 4q | 24 | 24 | 0 | 0 | 0 | 100 % |
| 9q | 48 | 48 | 0 | 0 | 0 | 100 % |
| 17q | 96 | 45 | 0 | 6 | 45 | 94 % |
| 21q | 128 | 118 | 0 | 5 | 5 | 96 % |
| 33q | 208 | 200 | 0 | 3 | 5 | 99 % |
| 45q | 283 | 254 | 0 | 22 | 7 | 92 % |
| 57q | 384 | 382 | 0 | 0 | 2 | 100 % |
| 69q | 450 | 421 | 0 | 11 | 18 | 98 % |

- **No SAT-bad case on any chip:** the local check is never optimistic.
- The UNSAT-clean cases are where it is too strict. 45q has the most (22).
  The stub margin of 4 is a likely cause: it flipped one 45q SAT-bad case and
  may have made others UNSAT.

### Running it

All scripts are in `artifacts/logs/sat-vs-feedline-2026-10-13/`. Git ignores
that folder. Call them by absolute path: a background shell does not keep the
working directory.

```zsh
L=/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4/artifacts/logs
E=$L/sat-vs-feedline-2026-10-13

# 1. The runs: 8 runs of options per segment, one chip at a time.
#    Every run is written to $L/<arm>/cases/<chip>/.
$E/run_agree.sh local-mytest-2026-10-22 8 4q 9q 17q 21q 33q 45q 57q 69q

# 2. Or judge stored runs again under new switches; the rip-up stays as stored.
cd $E && ../../../.venv/bin/python side/rejudge_local.py \
  ../local-nobonus-2026-10-21/cases/45q ../local-mytest-2026-10-22/cases/45q
#    With a side build instead of the installed one (see Traps):
../../../.venv/bin/python side/with_side.py side/rejudge_local.py <from> <to>

# 3. The table, against another arm.
python3 local_agreement.py local-nobonus-2026-10-21 local-mytest-2026-10-22

# 4. The page, with the pictures after the outer routing and the coupler phase.
$E/stage_pictures.sh local-mytest-2026-10-22       # while artifacts/<chip> holds the run
../../../.venv/bin/python local_report.py local-mytest-2026-10-22 --round "my test"
../../../.venv/bin/python local_serve.py --dir ../local-mytest-2026-10-22/page \
  --port 8793 --host 0.0.0.0
```

- **One case only:** `SCPD_LOCAL_AGREEMENT_ONLY=<chain>:<lo>:<options>`, for
  example `10:1:6 5 4 27`. The run loops and the accept step skip every other
  run.
- **Plain wires only:** `SCPD_LOCAL_AGREEMENT_PLAIN=1`. Resonators leave the
  window; terminal edges stay.
- **Graphs:** `local_serve.py` builds a feedline's own capacity graph when the
  page asks for it. The feedline routing dashboards of single cases come from
  `dashlocal.py` and are served from `dash-local/` on port 8777.

## The global test (rounds from local SAT states)

`ends_experiment.py`:

1. Holds the Final stage at the end of the coupler insertion
   (`coupler_session`).
2. Takes every locally SAT run of the local test per chain, grouped by the
   options of the chain's first and last coupler.
3. Draws whole-chip states, most chains on other end options.
4. Checks them until half are SAT and half UNSAT, or until it has checked
   four times `--cases`.
5. Routes the chosen cases.

```zsh
cd $E
for chip in 45q 69q; do
  ../../../.venv/bin/python ends_experiment.py ../../$chip \
    ../local-nobonus-2026-10-21/cases/$chip round60/results/$chip --cases 20 --workers 1
done
../../../.venv/bin/python report.py round60/results -o round60/index.html --round round60
../../../.venv/bin/python serve.py --dir round60 --port 8794 --host 0.0.0.0
```

- `ends.json` per chip holds whether half and half was reached and why not.
  The page shows it.
- **Replay a round on the current build:** `replay_round59.py <run_dir>
  round59/results/<chip> <out> --chain 6 --cases case001 case002`. It checks
  each case whole, with only the given chain's moves, and without them. Set
  `SCPD_GLOBAL_SAT_SEARCH=0`, so that the session holds the chip as the
  commit leaves it.

Round 59 (`round59`, page on 8792), whole-chip states checked:

| Chip | Checked | Complete | SAT | UNSAT |
|---|---|---|---|---|
| 4q | 18 | 16 | 16 | 0 |
| 9q | 120 | 120 | 120 | 0 |
| 17q | 136 | 70 | 20 | 50 |
| 21q | 76 | 76 | 56 | 20 |
| 33q | 160 | 160 | 160 | 0 |
| 45q | 120 | 103 | 90 | 13 |
| 57q | 120 | 120 | 120 | 0 |
| 69q | 80 | 50 | 42 | 8 |

Round 59 routed 69q with the fence fix: 9 of 10 SAT cases came out clean. All
8 UNSAT cases failed, as did the base case (183, 184). case001 is SAT with bad
wires 185 to 187.

## Benchmarks

```zsh
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
L=artifacts/logs
# All eight chips one after the other (or CHIPS="17q 45q"), switches as ENV=VALUE.
CHIPS="17q 45q 4q 9q 21q 33q 57q 69q" $L/run-arm.sh bench-mytest-2026-10-22 SCPD_SLOT_CHECK_FAST=1
# Angle cost, undrawn edges, chains locally SAT, global verdict, bad wires, time, against another arm.
cd $L/sat-vs-feedline-2026-10-13 && python3 bench_night.py bench-mytest-2026-10-22 bench-globalsat-both-2026-10-21
# Seconds of every phase of the Final stage, two arms side by side.
python3 phase_times.py bench-globalsat-both-2026-10-21 bench-mytest-2026-10-22
```

- `run-arm.sh` runs `plan --stage final` only and reuses the earlier stages.
  It writes `<arm>/<chip>.log` and `<arm>/DONE`.
- `REPAIR_TRIALS`, `STOP_AFTER`, `MAX_RELAXATION` and `REFINE_ROUNDS` change
  the run directory's copy of the config.
- Useful log lines: `grep "\[Global SAT\]"` and `grep "final routing:"`.

## Open points and next steps

1. **CHANGELOG, UPGRADING, lint, tests for the global SAT search.** See
   *State*.
2. **The collector's check with the neighbours standing is still stricter than
   the whole-chip check** in one known case. On 45q the states of chain 6,
   segment 3 (coupler 36 → 7) lose wire 123 there, but the whole chip with
   that move is SAT (`replay_round59.py`). The search did not need them; find
   out why 123 is lost.
3. **Runtime of the global SAT search.** On 69q the collection takes about
   190 s, and the first combination is SAT after 1 s. Almost all of it is the
   segment search (`researchSegment`), not the slot check. A focus chain could
   be searched before the other chains, and the search could check each focus
   state as soon as it is found.
4. **Incremental work in the chain search** (agreed with the user as the next
   speed-up). First measure per step how the time splits between the stretch
   pass, `slotGraphOfPass` and the slot check. Then cache per feedline edge the
   slots, ways out and room fills, keyed by the edge's geometry and the
   obstacles near it. Keep byte equality as the condition, as the fast check
   did.
5. **Make `SCPD_SLOT_CHECK_FAST` the default** once the user agrees, and
   remove the old path.
6. **The local check is pessimistic on 45q** (22 UNSAT-clean). Check the stub
   margin of 4 against them.
7. **Round 59 69q case001**, SAT with bad wires 185 to 187.
8. **Length fails** of the feedline routing (table above). They are not part
   of this work.

## Known test failures

The full `mqt-scpd-pipeline-test` did not run for this handover. Earlier in
the work, four FinalRouter tests failed on this dirty branch, and the user
agreed to leave them for now. Two `CouplerSession` tests describe the mouths
before commit `0aa06c7`: `ClosesTheMouthBetweenTwoChainsWithTwoLines` and
`ClosesAMouthWhoseLinesWouldRunOverTheSlotsWithOneLine`. Run the full suite
once and record the names.

## Pages on the local network

The Mac serves them on `0.0.0.0`; in the WLAN use `http://192.168.178.152:<port>`.

| Port | Page |
|---|---|
| 8774 | start page of every page (`artifacts/logs/launcher`) |
| 8788 | local test with today's defaults (`local-nobonus-2026-10-21`) |
| 8792 | round 59, global cases from local SAT states |
| 8791 | local test, pin cells 10 (`local-sat-pin10-2026-10-20`) |
| 8789 | round 58 |
| 8783 | local test, escape clearance (`local-escape-clear-2026-10-20`) |
| 8777 | feedline routing dashboards of single local cases (`dash-local`) |

## Traps

- **The Mac crashes under load.** Run one heavy job at a time, and 45q and
  larger with one worker. Forked replays rebooted the Mac four times.
- **Rebuild the extension after a C++ edit:**
  `uv sync --inexact --no-dev --no-build-isolation-package mqt-scpd
  --reinstall-package mqt-scpd`. Without `--reinstall-package` the old
  extension stays. A running benchmark loads the new extension at its next
  chip, so do not reinstall under it.
- **Side builds.** Build with `ninja -C build/cp311-abi3-macosx_15_0_arm64/Release
  bindings/pyscpd.abi3.so`, then `rm` the old `side/pyscpd.abi3.so` and `cp`
  the new one. A `cp` over the old file in place got the process killed by
  code signing.
- **Absolute script paths.** `with_side.py` needs the script by absolute
  path; with a relative one, `local_report.py` wrote into the wrong folder.
- **zsh.** A glob such as `*.log` that matches nothing is an error and ends a
  loop; use `setopt NULL_GLOB`. Use `env A=B cmd` for quoted assignments.
- **A copied run directory** needs `routing_config.json` beside
  `config.toml`.
- **`timeout` does not exist on this Mac.**
- **Full pipeline test.** The whole `mqt-scpd-pipeline-test` takes many
  minutes; filter it (`--gtest_filter='SlotCheck.*'`).
- **The worktree of the speed-up agent** is
  `.claude/worktrees/agent-abe16eec4af5ec644`, with its own `.venv`.
  Measurements and the byte comparison harness are in the session scratchpad
  (`bench/check.sh`, `golden_slotcheck.py`); copy them out if they matter.
