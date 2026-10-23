# The local slot check against the local rip-up

Written for whoever goes on with the capacity check of the coupler insertion
after 2026-10-19. It covers the evening of 2026-10-19: the slot check moved
into every step of the chain search, the comparison of the local check with
the local rip-up-and-reroute, the fixes that comparison found, and a global
stress round built from the local cases. Read
[handover-slot-check.md](handover-slot-check.md) first for the slot check
itself.

## State

- Checkout `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`, branch
  `phase-4-routing-stages`, HEAD `b097c70`. Everything below is
  **uncommitted**.
- Files: `src/pipeline/SlotCheck.cpp`, `include/mqt-scpd/pipeline/SlotCheck.hpp`,
  `src/pipeline/FinalRouter.cpp`, `test/pipeline/test_slot_check.cpp`,
  `test/pipeline/test_final_router.cpp`, `CHANGELOG.md`.
- Tests: `./build/release/test/pipeline/mqt-scpd-pipeline-test
  --gtest_filter='SlotCheck.*:*KeepsAnEdgeClear*:*JudgesTheGraphInMemory*:*KeepsTheEdgesAtACoupler*'`,
  18 tests, green.
- Still to do before a commit:
  - delete `test/pipeline/zz_profile_slot_check.cpp` (a profiling harness);
  - record the golden file again (`golden_slotcheck.py record`): even slots and
    the slot that leads back into its chamber changed verdicts on purpose;
  - `uvx nox -s lint` has not run; format own lines with `git clang-format`.
- After a C++ edit: `uv sync --inexact --no-dev --no-build-isolation-package
  mqt-scpd --reinstall-package mqt-scpd`. A job that runs from `.venv` loads
  the new extension at its next chip, so do not reinstall under a running
  benchmark; `side/with_side.py` and `side/side_cli.py` in the experiment
  folder load a build of `build/cp311-abi3-*/Release/bindings/pyscpd.abi3.so`
  copied next to them instead.
- The Mac: one heavy job at a time; 45q and larger with one worker.

## What changed in the code

| Switch | Default | Where | What |
|---|---|---|---|
| `SCPD_SLOT_CHECK_STEPS` | on | FinalRouter | every step of the chain search, every chain on its own and the global check judged by the slot check |
| `SCPD_LOCAL_SAT_WIRES` | on | FinalRouter | the slot check of a step routes and judges the wires of the chain's feedline alone (`chainWires`), every terminal edge kept |
| `SCPD_EVEN_SLOTS` | on | SlotCheck | the slots of a straight piece spread evenly, same count per run of places where a slot fits |
| `SCPD_OWN_TERMINAL_STUB_GUARD` | on | FinalRouter | an edge's corridor closes the coupler run of the terminal edge of its own chain (`guardOwnTerminalRuns`) |
| `SCPD_STATIC_GAP_BONUS` | 0 | SlotCheck | cells added to a gate from a feedline edge to artwork or the border; see *Bonus* |
| `SCPD_LOCAL_AGREEMENT=N` | 0 | FinalRouter | phase 4 compares instead of repairing; see *The comparison* |

Without a switch:

- **Slots open to bridging wires.** An edge of an open chain is prescribed to
  no wire, but its stretches stay open to the wires that bridge it; its slots
  now do too (`addSlots`). Without this 17q refused 4776 of 5488 steps.
- **A slot that leads back into its chamber.** A slot of a chain with a
  reflector end may have one chamber on both sides. The search built moves
  only between two different chambers, so such a slot was never usable
  (move building in `Checker`). On 45q, chain 3 checked alone, wires 57 and 58
  were prescribed to cross f20 and had no way at all: 209 refused steps and
  the chain search ran out of time. Now: 253 steps, all SAT.
- `tryRun` is split into `applyRun`, `windowOfRun` and `rerouteWindow`, so
  that the repair and the comparison share them.

## The comparison (`SCPD_LOCAL_AGREEMENT`)

Phase 4 runs it instead of the repair. Every chain is cut into overlapping
segments of three edges (`SCPD_LOCAL_AGREEMENT_SPAN`); the segment search of
the targeted repair yields up to N runs of options per segment; each run is
applied, judged and put back (`agreeRun`, `compareLocally`).

- **The wires of a feedline** (`feedlineWindow`): with a launcher at both ends,
  the ring of the feedline pass from its starting to its ending terminal edge,
  every plain wire and resonator between them. The ring holds terminal edges
  only, so a feedline with a reflector end gets the resonators of its couplers,
  the wires that bridge its edges and its terminal edge.
- **Local SAT of a feedline**: the slot check of the chip with that feedline's
  wires alone plus every terminal edge (they name the mouths; without them a
  feedline's own terminal edge lost its way).
- **Local rip-up of a feedline**: only its wires are routed again, four rounds
  with `SCPD_RESEARCH_ROUNDS=4`, every other wire off the canvas
  (`rerouteWindow` with `ignoreOutside`).
- The rip-up is run again only for the segment's feedline, for feedlines that
  share a wire with it, and for feedlines whose verdict differs from the chip
  as it began. In practice only the segment's feedline was ever run again, so
  every number below counts one verdict per run.
- `SCPD_LOCAL_AGREEMENT_DIR` writes every run as JSON (feedlines, rounds of
  the rip-up, ways, pads) and the slot check's graph for every run with a
  disagreeing feedline.

### Result, 2026-10-19

Arm `artifacts/logs/local-agreement-2026-10-19`, 8 runs per segment, page on
port 8773 (`local_serve.py`). 45q to 69q ran with the fix for slots that lead
back into their chamber; 4q to 33q ran before it.

| Chip | Runs | SAT, clean | SAT, rip-up fails | UNSAT, clean | UNSAT, fails | Agree |
|---|---|---|---|---|---|---|
| 4q | 24 | 24 | 0 | 0 | 0 | 100 % |
| 9q | 48 | 48 | 0 | 0 | 0 | 100 % |
| 17q | 96 | 47 | 0 | 4 | 45 | 96 % |
| 21q | 128 | 116 | 2 | 3 | 7 | 96 % |
| 33q | 208 | 200 | 0 | 2 | 6 | 99 % |
| 45q | 283 | 268 | 4 | 6 | 5 | 96 % |
| 57q | 384 | 378 | 2 | 2 | 2 | 99 % |
| 69q | 452 | 420 | 10 | 11 | 11 | 95 % |
| all | 1623 | 1501 | 18 | 28 | 76 | 97 % |

## The disagreements left

**SAT, rip-up fails (18).** Nearly all are two neighbouring wires that the
rip-up lays too close; with four rounds they take turns, only one of the two
fits at a time (`rounds` in the run's JSON):

- 21q chain 4, segment 0..3, options `launcher 7 5 3` and `launcher 7 11 3`:
  wires 59 and 60 at the two slots of f22, one on its diagonal piece, one on
  its vertical piece; the rip-up lays both through the first slot.
- 45q chain 7, segment 1..4, `6 1 2 37`, `6 1 5 37`: wires 132 and 133.
- 57q chain 7, segment 3..6, `15 35 35 launcher`, `15 35 36 launcher`: 171, 172.
- 69q chain 4, segment 4..7: 92, 93. 69q chain 9, segments 3..6 and 4..7
  (nine runs): 186/187 and 191/192.
- 45q chain 1, segment 1..4, `6 2 30 3`, `6 3 30 3`: wire 25 ends **crossing**
  after the first round; not a pair of neighbours.

**UNSAT, clean (28).** 17q: wires 11 and 40, the same wires the global check
of 17q loses. 69q chains 8 and 9: wires 163 to 168 and 180 to 184, the region
where 69q fails globally. 45q chains 5 and 6: 104 to 109, 116 to 122.

## Benchmarks with the changes

Arm `local-sat-2026-10-19` against `even-slots-2026-10-19` (both with the
stub guard and even slots):

| Chip | Steps (refused) | Chains out of time | Global | Bad | Time |
|---|---|---|---|---|---|
| 17q | 188 (8) → 188 (8) | none → none | UNSAT → UNSAT (11, 40) | 0 → 0 | 51 s → 52 s |
| 45q | 2242 (366) → 972 (139) | chain 3 → none | SAT → SAT | 0 → 0 | 321 s → 219 s |

All eight chips with the stub guard (`slot-stubguard-2026-10-19`) equal the
arm before it; with even slots the slot check takes about twice as long per
step (every place of an edge is examined, not every nineteenth), and 33q and
21q take 7 to 14 s longer. The other six chips were not run with
`SCPD_LOCAL_SAT_WIRES` yet.

## Global check

The 372 cases of rounds 53m to 55m (`switch_sweep.py`, pages on 8770 to 8772):
355 right, 0 SAT with bad wires, 17 UNSAT with none. Even slots made 33q case
15 (53m) and 57q case 10 (54m) right.

**Round 56c**, global cases built from the local runs (`combo_experiment.py`,
`run_combos.sh`, page on 8775): a random set of chains, each on one run of
options or two that do not overlap, checked and routed.

| Chip | Cases | SAT, 0 bad | SAT, bad | UNSAT, 0 bad | UNSAT, bad |
|---|---|---|---|---|---|
| 4q | 17 | 17 | 0 | 0 | 0 |
| 9q | 51 | 51 | 0 | 0 | 0 |
| 17q | 61 | 1 | 0 | 60 | 0 |
| 21q | 61 | 35 | 0 | 7 | 19 |
| 33q | 61 | 45 | 0 | 1 | 15 |
| 45q | 51 | 34 | 1 | 6 | 10 |
| 57q | 51 | 45 | 1 | 2 | 3 |
| 69q | 31 | 0 | 3 | 0 | 28 |

The five SAT cases with bad wires each hold one of the local runs above that
were SAT with a failing rip-up: 45q case 15 (132, 133), 57q case 38 (171, 172),
69q cases 1 to 3 (186, 187). 17q is UNSAT in 112 of 113 combinations, routed
clean in all 60: the global check loses wires 11 and 40.

## Bonus

`SCPD_STATIC_GAP_BONUS` over the 372 cases: 2 to 4 cells change nothing; 5 and
6 make 7 cases of 45q (round 55m, cases 02 to 11) SAT with bad wires 70 and 71,
because gate g160 (96.3 cells, f23 to the artwork near Qb41) then holds 5
wires and the router passes 4; 7 and 8 also make 17q cases wrong. Pages under
`sat-vs-feedline-2026-10-13/bonus/b5` and `b7`, the flipped gates in
`bonus/flips.txt`. It stays at 0.

## Next steps

1. The neighbouring pairs: count two slots, or two crossings of a gate, that
   lie one clearance apart with a margin, and run the comparison again on 21q,
   45q, 57q and 69q.
2. Wire 25 on 45q: why the rip-up crosses a feedline it may not cross.
3. The global check of 17q (wires 11 and 40), then 69q (g223, g225).
4. Benchmarks of all eight chips with `SCPD_LOCAL_SAT_WIRES`.

## Tools

In `artifacts/logs/sat-vs-feedline-2026-10-13/` (git ignores it):

- `run_agree.sh <arm> <runs> [chips]`, `local_agreement.py <arm>`
  (`--runs <arm>/<chip>` lists every disagreeing feedline), `local_report.py`,
  `local_serve.py` (builds a feedline's own graph when it is asked for).
- `combo_experiment.py`, `run_combos.sh <round> <local arm>`.
- `switch_sweep.py <switch> <values>`: the 372 cases under several values.
- `bonus_pages.sh`, `side/bonus_cases.py`.
- `artifacts/logs/watch_agree.sh`, `watch_bench.sh`, `rebuild_loop.sh`,
  `run-arm.sh`; start them by absolute path, a background command does not keep
  the working directory.
- Start page of every page: port 8774 (`artifacts/logs/launcher`).

## Traps

- A background shell started with a relative path fails silently with exit
  code 127; three runs were lost that way.
- `local_report.py` and the page count only feedlines judged in the run; a
  feedline that keeps the rip-up of the chip as it began counts nowhere.
- The JSON adapter and the typed path of the slot check both carry `users` of
  stretches now; a graph written before 2026-10-19 lacks them.
- Old rounds 2 to 51 and their dashboards were deleted on 2026-10-19 to free
  54 GB; round 52 stays for the golden test.
