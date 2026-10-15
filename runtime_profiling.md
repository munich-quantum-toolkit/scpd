# Runtime and results of every benchmark chip, to the end of the feedline refinement

Written for the user, 2026-10-13. Branch `phase-4-routing-stages`, HEAD
`9a6ef83` plus the uncommitted changes listed under *Changes in the tree*.

## Summary

- Every chip ends the feedline refinement with **0 bad wires**: 0 unrouted,
  0 open and 0 wires that cross a feedline. Every run exits with code 0.
- The results match the runs of 2026-10-11 (`9a6ef83`): the same short counts,
  the same resonators off their length and the same remaining finding on 69q.
- 45q is 40 s slower and 69q 60 s slower than on 2026-10-11. Almost all of the
  extra time is in the coupler insertion. The likely cause is the chain budget
  of 20 s instead of 10 s; this cause is not measured.
- The Final stage takes more than 85 % of every run. On 69q the refinement
  (169 s) and the coupler insertion (232 s) take the most time.

## Changes in the tree

- `src/pipeline/CorridorRouter.cpp`: a chord drawn over the end of another
  chord is refused at every end, not only at pinned ends. The `fromPin` and
  `toPin` flags are gone.
- `src/pipeline/FinalRouter.cpp`: `SCPD_CHAIN_ASTAR_SECONDS` defaults to 20 s
  instead of 10 s.
- `benchmarks/17q/config.toml`: `repair_trials` and `stop_after` are commented
  out. The 17q run used a copy with `repair_trials = 0` and
  `stop_after = "refined"`, the same as every other chip.

## Setup

- Package rebuilt from the tree with
  `uv sync --inexact --no-dev --no-build-isolation-package mqt-scpd --reinstall-package mqt-scpd`.
- `mqt-scpd plan -c <config> -o <run> -v 1` for each chip, with each chip's
  `benchmarks/<chip>/config.toml` (`stop_after = "refined"`,
  `repair_trials = 0`). No `SCPD_*` variable set.
- One chip at a time on the Mac (Apple silicon, macOS 15.7.7). No other chip ran
  beside it, because the chain search has a wall-clock budget.
- Each log line carries the wall-clock seconds since the start of the run. A
  stage's time runs from the line that writes the previous stage's artifact to
  the line that writes its own. A phase of the Final stage runs from its first
  log line to the first log line of the next phase.
- Scripts and data, in `artifacts/logs/runtime-2026-10-13/` (ignored by git):
  `run_all.sh` runs the chips; `parse.py <dir>` prints the stage times, the
  phase times and the results; `<chip>.log` holds each run's log;
  `17q-config.toml` is the 17q configuration that the run used.

## Results at the end of the feedline refinement

`bad = unrouted + open + crossing`. Short and long are not counted yet. The
checks at the end are feedline room, coupler crossings, resonator crossings,
feedline crossings and chain crossings.

| Chip | Wires drawn | Unrouted | Open | Crossing | **Bad** | Short | Long | Resonators off length | Checks at the end |
| ---- | ----------: | -------: | ---: | -------: | ------: | ----: | ---: | --------------------: | ----------------------------- |
| 4q   |       17/17 |        0 |    0 |        0 |   **0** |     1 |    0 |                   1/4 | all green                     |
| 9q   |       46/46 |        0 |    0 |        0 |   **0** |     0 |    0 |                   0/9 | all green                     |
| 17q  |       92/92 |        0 |    0 |        0 |   **0** |     3 |    0 |                  3/17 | all green                     |
| 21q  |     104/104 |        0 |    0 |        0 |   **0** |     1 |    0 |                  1/21 | all green                     |
| 33q  |     158/158 |        0 |    0 |        0 |   **0** |     1 |    0 |                  1/33 | all green                     |
| 45q  |     209/209 |        0 |    0 |        0 |   **0** |     0 |    0 |                  0/45 | all green                     |
| 57q  |     267/267 |        0 |    0 |        0 |   **0** |     0 |    0 |                  0/57 | all green                     |
| 69q  |     322/322 |        0 |    0 |        0 |   **0** |     1 |    0 |                  1/69 | feedline room: 1 (f46 vs 132) |

## Results of the phases before

| Chip | Outer routing, fails in round 0 | Chains | Chains at the time limit | Chains searched again without the capacity rule | Feedline edges not drawn | Feedline crossings after the coupler insertion | Channels between chains too narrow | Feedline routing, fails in round 0 | Short after feedline routing | Feedline room after feedline routing |
| ---- | ---: | -: | -: | -: | -: | -: | -: | -: | -: | -: |
| 4q   |    0 |  1 |  0 |  0 |  0 |  0 |  0 |  2 |  1 |  1 |
| 9q   |    0 |  2 |  0 |  0 |  0 |  0 |  0 |  0 |  2 |  0 |
| 17q  |    2 |  4 |  0 |  0 |  0 |  0 |  0 |  3 |  5 |  0 |
| 21q  |    6 |  5 |  0 |  0 |  0 |  0 |  0 |  1 |  2 |  0 |
| 33q  |    9 |  7 |  0 |  0 |  0 |  0 |  1 |  1 | 10 |  1 |
| 45q  |    7 |  8 |  1 |  0 |  0 |  0 |  0 |  4 | 15 |  0 |
| 57q  |   26 |  9 |  1 |  0 |  0 |  0 |  1 |  2 | 29 |  1 |
| 69q  |   22 | 12 |  3 |  1 |  0 |  3 |  7 |  8 | 29 |  0 |

- 69q: the capacity graph after the coupler insertion is UNSAT. One edge is
  short by one wire. The feedline routing removes the 3 feedline crossings
  between chains: f5 × f6, f46 × f47 and f60 × f61.
- The feedline refinement removes the feedline/lead pair that is too close on
  4q, 33q and 57q. On 69q the refinement leaves one pair too close: f46 and
  resonator 132 at (3321,1561).

## Runtime per stage (seconds)

| Chip | Capacity | Global | Assign | Corridor | Detail |  Final | **Total** | 2026-10-11 |
| ---- | -------: | -----: | -----: | -------: | -----: | -----: | --------: | ---------: |
| 4q   |      0.1 |    0.0 |    0.0 |      0.0 |    0.0 |    3.0 |   **3.2** |          4 |
| 9q   |      0.3 |    0.0 |    0.0 |      0.0 |    0.2 |    8.4 |   **9.0** |         10 |
| 17q  |      1.1 |    0.0 |    0.2 |      0.1 |    0.3 |   34.2 |  **36.0** |         40 |
| 21q  |      2.1 |    0.0 |    0.3 |      0.2 |    0.5 |   41.8 |  **44.9** |         49 |
| 33q  |      6.2 |    0.0 |    1.8 |      0.7 |    1.6 |   66.8 |  **77.1** |         83 |
| 45q  |     10.9 |    0.0 |    1.3 |      2.0 |    4.6 |  193.5 | **212.2** |        172 |
| 57q  |     21.6 |    0.0 |    3.5 |      4.1 |    9.1 |  207.2 | **245.5** |        243 |
| 69q  |     53.4 |    0.1 |   15.4 |     10.9 |   16.5 |  606.9 | **703.1** |        643 |

## Runtime per phase of the Final stage (seconds)

The feedline routing includes the inner routing under the feedlines, which
takes less than 0.1 s on every chip. 4q has no inner wires.

| Chip | Inner | Outer | Refinement | Coupler insertion | Feedline routing | Repair | Feedline refinement |
| ---- | ----: | ----: | ---------: | ----------------: | ---------------: | -----: | ------------------: |
| 4q   |     – |   0.1 |        0.6 |               0.5 |              1.3 |    0.0 |                 0.5 |
| 9q   |   0.0 |   0.5 |        3.1 |               1.1 |              1.0 |    0.0 |                 2.6 |
| 17q  |   0.0 |   1.8 |        3.2 |               6.3 |             18.4 |    0.0 |                 4.4 |
| 21q  |   0.0 |   7.6 |       11.9 |               4.0 |              6.8 |    0.1 |                11.3 |
| 33q  |   0.0 |   6.6 |       20.9 |              11.5 |              6.4 |    0.1 |                20.9 |
| 45q  |   0.0 |  25.9 |       30.0 |              59.7 |             46.1 |    0.1 |                31.1 |
| 57q  |   0.0 |  41.9 |       40.6 |              54.6 |             19.6 |    0.2 |                49.6 |
| 69q  |   0.0 |  38.2 |      169.3 |             232.1 |             76.3 |    0.4 |                89.4 |

Against the logs of 2026-10-11, the coupler insertion grew from 41 s to 60 s
on 45q and from 150 s to 232 s on 69q. The feedline routing of 45q grew from
26 s to 46 s.
