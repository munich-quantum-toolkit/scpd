# Global SAT: conflicts between feedline ends

Status: design approved 2026-10-30, not implemented. Code today:
`searchGlobalSat` and `collectChainStates` in `src/pipeline/FinalRouter.cpp`,
`namedChains` and `nextCombinationRound` in `src/pipeline/CombinationOrder.cpp`.

## Goal

After the commit of the coupler insertion every chain is SAT on its own, but the
whole chip can still be UNSAT: a bottleneck between two feedlines holds fewer
wires than must pass it. The global SAT search is meant to find other options
for the chains so that the whole chip is SAT. Today it rarely does: of the 128
runs under `planar-superconducting-pd/runs/` that were UNSAT after the commit,
16 ended SAT and 119 ended with "no SAT combination".

The case that fails and must work: **33q_dense with the snap options**
(`coupler_snap_pitch = 50`, offsets `0,1,2,6,7`, `coupler_merge = false`,
bench label `snap-contact`). One bottleneck is over after the commit, g380
"f33 at its coupler, right · 103 at its target, right" (5 wires pass, 6 must:
98–103). f33 is the terminal edge at the **end of chain 5**; next to it,
g377 "f34 at its coupler · f33 at its coupler" carries the same wires, f34
being the terminal edge at the **start of chain 6**. The fix needs both ends to
move. Today:

1. The focus skips terminal edges (`!edge.terminal`), so there is none and
   stage 1 does not run.
2. `namedChains` names chain 5 alone (the feedline name f33 at the gate); chain
   6 is never named, so never collected and never moved.
3. Chain 5's states are the 8 cheapest runs of its two end segments: 6 locally
   SAT, all only with the chain alone, 3 distinct end pairs; every
   combination fails at the same bottleneck (g375–g378 as the passes name it).
4. Round 2 finds no new chain to name and the search ends after 6
   combinations.

Success: 33q_dense with the snap options ends SAT (search and report); the
eight standard chips (4q, 9q, 17q, 21q, 33q, 45q, 33q_dense, 45q_dense, bench
label `contact`) keep their verdicts, edges and angle costs (the search runs
only where the commit is UNSAT, among them 45q_dense).

## Model

A conflict lies between **two feedlines** (chains). A chain has two ends: its
**start**, the first coupler (the first waypoint that is not fixed), and its
**end**, the last coupler. A conflict is a pair of chain ends, such as "end of
chain 5 ↔ start of chain 6". It is resolved by other options for **the two
competing ends**; the other ends of the two chains stay.

## 1. Conflict pairs

For every bottleneck over its capacity in a failed whole-chip check, each of
its two end walls is mapped to a chain end:

| wall name | chain end |
|---|---|
| `fN at its coupler, …`, `fN at its launcher, …` (a terminal edge) | the chain of fN; the end the edge belongs to (its first edge: start; its last edge: end) |
| `fN` (an inner edge) | the chain of fN; the end nearer the edge by waypoint index (`from < size / 2`: start, else end) |
| `lead of R`, `pad of R`, `R at its source/target` with R a resonator | the chain of R's coupler; the end nearer that coupler's waypoint |
| anything else (a plain wire's port, the artwork, the border) | none |

The two chain ends make the pair. When only one end wall maps to a chain end,
or both map to the same chain, the partner is **the end coupler of another
chain nearest the middle of the gate** (by pad centre, ties to the lower
chain index). A pair is stored with the lower chain first; pairs repeat across
gates and are kept once, in the order of their first gate.

Lost wires without a bottleneck over keep today's naming (`namedChains`).

`conflictPairs` is a pure function in `CombinationOrder` (header
`include/mqt-scpd/pipeline/CombinationOrder.hpp`): it takes, per bottleneck
over, its two end names and its middle, and a map of the chip (per feedline
name its chain, its waypoint pair and whether it is terminal; per resonator its
chain and waypoint; per chain its size and its two end couplers' pad centres)
and returns the pairs. It reads nothing of the router.

The whole-chip check (`wholeChipLost`) already knows the bottlenecks over
(`SlotCheckResult::over`, the `SlotGraph` edges with `ends`, `from`, `to`);
it returns their end names and middles alongside what it returns now.

## 2. States at a chain end

For every chain end in a conflict pair:

- The candidates are the open options of the end coupler
  (`openOptionsOf(chain, waypoint)`), without the option it stands on,
  ordered by the distance of their pad centre from the gate's middle, the
  farthest first, ties by option index; at most `global_sat_end_options`
  of them.
- For each candidate option `o`, the end segment (the waypoints from the end
  coupler to the launcher or termination at that end, widened inward to span 3
  as today) is searched by `researchSegment` with the end coupler **pinned** to
  `o`, one accepted run (the cheapest). The run is checked locally
  (`checkChainLocally`); a run SAT with the other chains standing or with the
  chain alone is a state of the chain, tagged with the end and `o`.
- The states of an end come in order: SAT with the others standing first,
  then SAT alone; within each, the order of the candidates.

`Segment` gets a field `pinned` (waypoint index → option index). In
`researchSegment` an interior layer whose waypoint is pinned offers that one
option instead of `openOptionsOf`; a pin on an option that is not open is
dropped (and logged).

A chain's state 0 stays the committed state. The end states are added to
`chainStates_` after it; the states `collectChainStates` collects today stay
for the conflict-directed stage.

## 3. Combinations

A new first stage, **pair rounds**, replaces stage 1:

- Round 1 starts from the commit. For each conflict pair (A, B), in order:
  A alone on each of its end states, then B alone on each of its end states,
  then A and B together on every pair of their end states, ordered by the sum
  of the two state ranks, then A's rank, then B's.
- Each combination is tried as today (`tryPick`: back to the commit, couplers
  moved by `setCouplerOption`, every edge drawn, `wholeChipLost`). The first
  SAT combination stays on the chip.
- When a round has no SAT combination, the next starts from its failed
  combination with the fewest lost wires and bottlenecks over (the earliest on
  a tie) and takes the conflict pairs that check names whose ends that
  combination does not move yet; the pairs' states are collected when first
  needed. The rounds end when no failed combination names a new pair.
- Then the conflict-directed stage 2 runs as today on what is left of the
  budget.

`global_sat_tries` bounds all combinations together; the order is fixed and
depends on nothing but the chip.

## 4. Configuration, log, tests

Configuration (`[stages.final]`, new fields at the end of `FinalParams`):

- `global_sat_end_options: uint32 = 16` — how many options per competing
  end the pair rounds try; 0 switches the pair rounds off (the search as
  today).

Log (`say`):

- `global SAT: g380: end of chain 5 (waypoint 5) ↔ start of chain 6 (waypoint 1)`
  per conflict pair;
- `global SAT: chain 5 end: 14 options tried, 9 locally SAT (4 with the other
  chains standing); 1.2s` per chain end;
- `global SAT: pair round 1: chains 5 and 6; N combinations` per round.

Tests:

- `test/pipeline/test_combination_order.cpp` (where `namedChains` and
  `nextCombinationRound` are tested): `conflictPairs` on a synthetic chip — a terminal-edge
  stub against a plain wire's port takes the nearest other chain end; two
  feedline walls make the pair directly; an inner edge maps to its nearer end;
  pairs repeat once.
- the order of a pair round's combinations (singles of A, singles of B, then
  pairs by rank sum), as a pure function `pairRoundPicks` in
  `CombinationOrder` beside `nextCombinationRound`.
- a pipeline test: a segment with a pinned interior waypoint reaches runs on
  that option only.

Measurement: bench labels `snap-pairs` (6 snap chips) and `pairs` (8 standard
chips) against `snap-contact` and `contact`; the verdicts, edges and angle
costs, and the seconds of the global SAT search.

## Out of scope

Conflicts in the middle of a chain (a bottleneck between inner couplers of two
chains) are mapped to the nearer ends like any other; moving inner couplers is
left to stage 2. Joint search of two chains' ends as one problem is not done.
