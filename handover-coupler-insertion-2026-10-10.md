# Coupler insertion: backup of 2026-10-10 and what it holds

Written for whoever picks the coupler insertion up again. The working tree of
`scpd-phase-4` was reset to the last commit on 2026-10-10 (user). This file says
where the state before the reset is kept, how to bring it back, and what it
contains.

- Repository: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, HEAD `f132d5e` "capacity flow check works
  but to slow". The tree is at this commit again.
- Nothing of the backup is committed.

## Where the backup is

Directory:
`/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4-backups/2026-10-10-coupler-insertion/`

| File                              | What it is                                             |
| --------------------------------- | ------------------------------------------------------ |
| `HEAD.txt`                        | the commit the backup is a change against (`f132d5e…`) |
| `status.txt`                      | `git status --short` before the reset                  |
| `working-tree.patch`              | `git diff --binary HEAD`: every tracked change         |
| `files.txt`, `files.tar.gz`       | the 20 changed or untracked files, whole               |
| `allchips-slide/`, `dbg-offset0/` | the logs and scripts of the measurements below         |
| this file                         | the handover                                           |

There is also a stash in the repository: `git stash list` shows
`backup coupler insertion 2026-10-10`. The reset was done with that stash, so
the stash holds the same tracked changes as the patch.

To bring the state back, on a clean tree at `f132d5e`:

```console
git stash apply          # the stash, or
git apply --binary working-tree.patch
```

`plan-capacity-step-runtime.md` was untracked and was left in the tree; it is
also in `files.tar.gz`.

## What the backup contains

Two pieces of work, both uncommitted before the reset.

1. **The per-step capacity check kept from the step before** (2026-10-09,
   earlier sessions). Grid library: `grid::addWallsToDistanceTransform`,
   `grid::CellWindow`, `IncrementalMedialAxis`, `BottleneckField`,
   `ChamberField`, with tests; `Driver::StepAnalysis` in `FinalRouter.cpp`; the
   port band wall, the plateau cut, `SCPD_CAPACITY_SOLO`. It is documented in
   the backed-up `handover-capacity-check.md` §9 and
   `handover-cpw-coupler-insertion.md`.
2. **The coupler insertion changes of 2026-10-10**, below.

## The changes of 2026-10-10

### Why 4q had no option on offset 0

Offset 0 points the resonator the way the launcher nearest the *place* points,
into the chip. The pad and the feedline then hang 28 cells from the insertion
point towards the chip edge. The outer routing leaves the length points of Q1
and Q4 outside the coupler box (Q4 at (187,52); the box with its margin of 5
starts at y = 88). The way then turns along the box edge. The deepest place of
the length window lies 22 cells inside the box for Q4 and 26 for Q1. So every
place of the window refused offset 0 with "the body leaves the box".
`SCPD_EDGE_IN_BOX` and `SCPD_CAPACITY_RULE` act after the options are built and
change nothing here.

A note on the measurements: `SCPD_COUPLER_MAX_SHORTFALLS` (with an S) is not
read by the code; the switch is `SCPD_COUPLER_MAX_SHORTFALL`.

Offset 0 is counted from the launcher nearest the place, so it can change along
one way. On 4q, Q1 and Q3 switch launcher at the end of their window. This was
not changed.

### `SCPD_COUPLER_SLIDE`, on: slide up the way

`optionsOf` walks the places in order of mismatch, as before. When no place fits
for an offset, port and jog, it walks the places further towards the qubit than
the wanted one, the nearest first, and `makeOption` waives the shortfall rule
for them (`pastTheShortfall`). A place of the window always wins. With it every
4q coupler has all six options on offset 0.

### `SCPD_COUPLER_JOGS`, on: the jogs from the start

`jogsUnlocked` is set when the coupler is built. Before, a coupler opened its
second-dogleg options only when a chain found no run of plain options, and on 4q
no chain fails. The edge from the right launcher (row y ≈ 411) to Q2's coupler
needs a quarter turn and then the pad's run of 21 cells: 26 cells. The plain
option leaves 25, and the edge takes a detour of two eighths. The jog option
leaves 45.

### Defaults changed (user)

`SCPD_CAPACITY_RULE` and `SCPD_EDGE_IN_BOX` are **off** by default. Both
switches, and `SCPD_COUPLER_SLIDE` and `SCPD_COUPLER_JOGS`, are read at every
call, so that a test can set them; all four appear in the settings line
`[Coupler Insertion] settings:`.

### Measured on 4q

| Variant                     | angle cost | Fails   |
| --------------------------- | ---------- | ------- |
| before (slide and jogs off) | 18         | 0       |
| slide and jogs on           | **8**      | 0       |
| jogs off                    | 14         | 3 short |
| slide off                   | 14         | 0       |

The same 8 with the capacity rule and the edge box on.

### Measured on all chips

New defaults, `stop_after = "couplers"`, `SCPD_BOTTLENECKS=1` (report only),
`-d`; logs in `allchips-slide/`. No capacity mechanism refused anything: no
`[Capacity Step]` line, no rule refusal, no chain check in any log. v3 is the
run of 2026-10-09 with the old defaults (capacity rule and edge box on).

| Chip | not drawn | angle | insertion | v3: not drawn / angle / insertion |
| ---- | --------- | ----- | --------- | --------------------------------- |
| 4q   | 0         | 8     | 0.8 s     | 2 / 12 / 27.1 s                   |
| 9q   | 0         | 20    | 1.5 s     | 0 / 20 / 2.1 s                    |
| 17q  | 0         | 61    | 6.5 s     | 0 / 49 / 32.2 s                   |
| 21q  | 0         | 54    | 15.1 s    | 0 / 52 / 35.9 s                   |
| 33q  | 0         | 74    | 18.6 s    | 0 / 92 / 197.6 s                  |
| 45q  | 0         | 118   | 56.0 s    | 0 / 122 / 209.1 s                 |
| 57q  | 1         | 126   | 50.5 s    | 0 / 144 / 701.3 s                 |
| 69q  | 1         | 166   | 121.4 s   | 1 / 180 / 844.8 s                 |

### Open problem: two edges of one chain cross on 17q

In that run f0 and f2 of chain 0 share the cell (1342,719). Three things
together:

1. The jog moved coupler `'11'` (Qb17) from (1340,725) to (1340,755), next to
   the right launcher.
2. The prefix search lays f0 while the later couplers stand on other options. In
   the commit, `'11'` stands on its jog and its lead, with the clearance, lies
   on f0's way. The commit routes f0 again: 497 cells, angle 20 (v3: 6). That is
   where 61 against 49 comes from.
3. The detour crosses f2. Nothing forbids it: under the rule of 2026-10-02
   (`SCPD_TERMINAL_EDGES_FENCE_ALL`, off) the first and last edge of a chain
   fence only their neighbour. No check reported it, because
   `checkCouplerCrossings` asks only about edges that meet at a coupler and
   `checkFeedlineCrossings` only about different chains.

Also: the prefix search routes only the two edges at each end of a chain against
the prefix (`CHAIN_FRESH_EDGES = 2`). The edges in between come from a memo
keyed by the option pair, so `SCPD_TERMINAL_EDGES_FENCE_ALL=1` alone does not
change the search.

The user decided: coming close is allowed, crossing is not. In the backup, in
three parts:

- **Lines.** `crossLines()` runs at the end of `corridorOfEdge`, after the slots
  are opened again. It closes, one cell wide on either side (`stencilFor(1)`),
  the own-chain terminal edges that `fenceCommittedEdges` leaves open
  (`chainLines_`) and every prefix edge that does not meet the edge being priced
  (`prefixLines_`, set by `solveChainAStar`'s step).
- **Memo.** `edgeCost` holds a remembered way against the corridor of the prefix
  it is asked under (`standsInCorridor`) and routes it again when it does not
  fit; `memoStale_` counts these in the insertion line.
- **Prefix check.** `standsOnThePrefix` refuses a step whose coupler, on the
  option asked for, holds a cell of an earlier prefix edge in its lead plus the
  clearance, its pad or its resonator. The two slots of each earlier edge are
  left out. The cells per option are kept in `optionRoom_`.
- **Check.** `checkChainCrossings` ("CHECK chain crossings") reports pairs of
  one chain that do not meet at a coupler and cross or share a cell.

17q with all of it: 0 crossings, 0 edges not drawn, angle 49, every check green.
**But chain 0 now runs out of its 10 s** ("out of time before any run of options
joined the chain — leaving it as it stands", 781 steps), so the 49 is the commit
of the old state, not a search answer. The next step is to count how many steps
the prefix check refuses and whether it is too strict, for example because a
coupler's provisional resonator way crosses earlier edges. None of the three
parts was measured on the other chips.

### Tests in the backup

`test/pipeline/test_final_router.cpp`:

- `CouplerSession.EveryCouplerOnFourQubitsHasAnOptionOnOffsetZero`: with the
  slide no 4q coupler lacks offset 0; without it one does.
- `FinalRouter.TheFourQubitChainMakesFourQuarterTurns`: angle cost 8 with the
  jogs, more without.
- `TheCapacityRuleLetsNoSettledEdgeCloseAWire` and
  `KeepsTheEdgesBetweenCouplersInsideTheCouplerBox` set their switch to 1
  themselves, now that the defaults are off.

The first two were green on the slide and jog code. After the change of defaults
and the crossing fix the test binary was **not run again**, and
`checkChainCrossings` has no test yet.

### Documentation in the backup

`CHANGELOG.md` (first bullet, without a PR reference and author yet),
`handover-cpw-coupler-insertion.md` ("Sliding up the way, and the jogs from the
start"), `handover-capacity-check.md` (the defaults).

### Traps met on the way

- `uvx nox -s lint` runs `prek --all-files` and rewrote 70 files that were clean
  (`drc.json`, scripts, docs); they were checked out again. Run it with
  `SKIP=clang-format` and look at `git status` afterwards. `ty` fails on old
  scripts in `artifacts/logs/` and on `test_planning.py`.
- `typos` changes "vermilion" in `FinalRouter.cpp`; set it back.
- `clang-format` only on one's own hunks: the hook binary
  `~/.cache/prek/hooks/python-Pi7X7G2Q9spAl1MrcS7Y/bin/clang-format` (v23.1.0)
  with `--lines`.
- Overlays: `artifacts/dev/py-slide` (slide and jogs, old capacity defaults at
  first, then the new defaults), `artifacts/dev/py-lines` (with the crossing
  fix). `artifacts/dev/py-dbg` holds the temporary dumps `SCPD_DBG_OFFSETS` and
  `SCPD_DBG_NOBOX` that are in no source. Never sync an overlay while a run on
  it is going.
- `CouplerSession` (`artifacts/logs/dbg-offset0`, scripts `session.py` and
  `force.py` in the scratchpad of that session) sets a coupler on an option and
  draws its two edges again; that is how the missing room of f0 on 4q was found.
