# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### The profile cuts keep their work on the walls between step checks

The step checks of the coupler insertion now keep the outline traces, the
simplified lines and the narrowings between two walls from one pass to the next
(`SCPD_PROFILE_MEMO`, on), and gather the walls they walk from the cells the
pass drew. The bottlenecks, and so every result, stay the same; on 45q the chain
search takes 78.0 s instead of 97.0 s. Over the whole insertion of 45q the memo
made 959 traces and 15 528 pairs of walls. `SCPD_PROFILE_MEMO=0` gives the
earlier behavior.

### The coupler insertion prices end edges at a tenth and offers six offsets

Two defaults of the coupler insertion changed. An edge between a launcher and
its coupler costs a tenth of its turns in the chain searches
(`SCPD_CHAIN_END_EDGE_WEIGHT=0.1`; the feedline routing draws these edges
again), and the option generator turns a pad by the offsets 0, 1, 2, 5, 6 and 7
only (`SCPD_COUPLER_OFFSETS=0,1,2,5,6,7`; offsets 3 and 4 turn the pad 135°
and 180° against the way and were never chosen). Together they cut the Final
stage of 4q to 57q from 1843 s to 634 s, and every chip ends globally SAT. The
results change: other couplers and feedline edges are chosen, 57q ends with 2
short wires (40, 183) instead of none, and the feedline angle cost rises on 17q
(49 → 53), 21q (52 → 54) and 57q (134 → 140).

`SCPD_CHAIN_END_EDGE_WEIGHT=1 SCPD_COUPLER_OFFSETS=0,1,2,3,4,5,6,7` restores the
earlier behavior.

### The coupler insertion opens no second dogleg

A coupler of the coupler insertion now stands on an option without a second
dogleg. The chain search, the greedy, the trellis, the segment search and the
blind repair never offer an option with one. `SCPD_SECOND_DOGLEG=1` restores the
earlier behavior, which opens a coupler's second-dogleg options when a chain
does not reach past it on plain options and in the segment search
(`SCPD_RESEARCH_JOGS`, which now acts only under `SCPD_SECOND_DOGLEG=1`).

### The coupler insertion has no time limits

The coupler insertion now gives the same result on every machine and at every
speed. Its searches stop on a count or when they are done, never on the clock:

| Switch                                      | Now                                                                                       |
| ------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `SCPD_CHAIN_ASTAR_SECONDS`                  | removed; the chain search runs to its optimum                                             |
| `SCPD_RESEARCH_SECONDS`                     | removed; the segment search ends after `repair_trials` tests or when it saw every run     |
| `SCPD_LOCAL_AGREEMENT_SECONDS`              | removed; `SCPD_LOCAL_AGREEMENT` bounds the comparison                                     |
| `SCPD_CAPACITY_SECONDS`                     | removed; the flow check (`SCPD_CAPACITY_FLOW=1`) has no limit                             |
| `SCPD_GLOBAL_SAT_SECONDS`                   | removed; `SCPD_GLOBAL_SAT_TRIES` (200) bounds the combinations                            |
| `SCPD_GLOBAL_SAT_COLLECT_SECONDS`           | removed; the collection takes every chain a stage needs                                   |
| `SCPD_CHAIN_WALL_SECONDS`                   | removed                                                                                   |
| `SCPD_CHAIN_RULE_FIRST_RUN_SECONDS`         | removed                                                                                   |
| `SCPD_CHAIN_END_PROBE`, `_AFTER`, `_BUDGET` | removed; the probe only cut the clock of the rule search                                  |
| —                                           | `SCPD_CHAIN_PROGRESS_STEPS` (100): a `[Chain Progress]` line every that many priced steps |

The log no longer says `best in the time given` or `out of time`. A chain search
that grows without end shows in its `[Chain Progress]` lines. The global SAT
search tries its combinations in a fixed order and has no random stage, so its
second stage tries other combinations than before.

### The run directory numbers seven artifacts

The Corridor stage now stands between the assignment and the detail routing, so
every artifact after `03-assign.fb` moves up by one:

| Before           | Now               |
| ---------------- | ----------------- |
| —                | `04-corridor.fb`  |
| `04-detail.fb`   | `05-detail.fb`    |
| `05-final.fb`    | `06-final.fb`     |
| `06-geometry.fb` | `07-geometry.fb`  |

A run directory written by an earlier build is not read by this one. Run the
pipeline again rather than renaming the files: the stages before the corridor
also changed, because a partition border now reports the number of wires it can
carry instead of its length in cells.

`plot --stage` and `render --stage` take the new name `corridor`; the other
stage names are unchanged.

### The coupler insertion checks the capacity graph by default

The chain search of the coupler insertion refuses every step whose feedline edge
closes the way of an outer wire (`SCPD_CAPACITY_RULE`). It finds the bottlenecks
of each step by profiles along the walls it moves (`SCPD_CAPACITY_FAST=1`), and
it reports the capacity graph of the chip it leaves behind (`SCPD_BOTTLENECKS`).
All three are on by default. With them every benchmark chip ends the feedline
routing with no bad wire, but the insertion takes longer.
`SCPD_CAPACITY_RULE=0`, `SCPD_CAPACITY_FAST=0` and `SCPD_BOTTLENECKS=0` give the
earlier behavior.

### The crossing slots judge the chip after the coupler insertion

The verdict on the whole chip after the coupler insertion now comes from the
check by crossing slots (`SCPD_SLOT_CHECK`, on). The log line
`CAPACITY GRAPH SAT` or `CAPACITY GRAPH UNSAT` and `final-capacity-graph.json`
carry its verdict; the line of the check by stretches now reads
`CAPACITY GRAPH by the stretches, before the slot check:`, and the graph keeps
that verdict as `previousVerdict`. In the graph the slot edges (`□ …`) take the
place of the stretches (`× …`), and the self-bottlenecks (`S …`) join the
gates. `CouplerSession.insertion_check` returns the slot check's `verdict`,
`lost`, `over` and `graph`, and the earlier answer as `previousVerdict` and
`previousLost`. The chain search keeps its own check. `SCPD_SLOT_CHECK=0` gives
the earlier behavior.

### The chain search prices its steps by the stub bound

The layered search of the coupler insertion now bounds every step by the
analytic answer with the forced end runs folded in (`SCPD_CHAIN_BOUND=4`). The
bound stays a lower bound, so the search stays exact and the chains it settles
cost the same; it is sharper, so the search prices fewer steps and the
insertion is faster. A chain the old bound ran out of time on may now be
solved, which can lower the stage's fail count. `SCPD_CHAIN_BOUND=1` gives the
earlier bound.
### The global SAT search collects the chains at an edge over first

The global SAT search of the coupler insertion now collects the states of the
chains at an edge over first and tries each state on its own before it collects
another chain (`SCPD_GLOBAL_SAT_LAZY`, on). It stops at the same combination as
before, so the chip it leaves does not change; it collects fewer chains and
says how many it left out. Only a binding `SCPD_GLOBAL_SAT_COLLECT_SECONDS` can
make the two orders differ, because the lazy order reaches the chains at an
edge over first. Set `SCPD_GLOBAL_SAT_LAZY=0` to collect every chain first.

### The faster slot check is the default

`slotCheck` and `slotCheckJson` now run the faster check of
`SlotCheckFast.cpp` by default. It returns the same results as the plain
check, down to the JSON graph, so nothing changes but the time. Set
`SCPD_SLOT_CHECK_FAST=0` to run the plain check as before.

### The refinements search a narrower band

Both refinements of the Final stage now redraw a wire within
`refinement_corridor_spacings` wire spacings of its way, 6 by default, instead
of `corridor_spacings`. The routes the refinements leave change. Set
`refinement_corridor_spacings = 0` under `[stages.final]` to give them the
sweep's band again.

### CMake 3.28

Building from source now requires CMake 3.28 or newer.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
