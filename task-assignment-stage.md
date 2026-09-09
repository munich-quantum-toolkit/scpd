# Task — fix the Assignment stage, and make three rules mandatory on all eight benchmarks

You are picking this up cold. Everything you need is below; read the orientation
first, then the task.

---

## 0. Orientation

**Repository:** `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-3`
**Branch:** `phase-3-solver-and-planning-stages`, based on `a126097`.
**Everything is uncommitted and stays that way** — the user commits per phase. Do
not commit, do not push. Leave the work in the tree and report what you verified.

**Prototype for comparison:** `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD`.
It is the C++ program this project is a port of. It is read-only: never write
into that tree.

### Read before you start

- `AGENTS.md` — the project's rules. They are strict and they are enforced by the
  linter and by review. In particular: tests for every change; comments and prose
  in the style of Orwell's six rules and ASD-STE100; no scaffolding or diagnostic
  code left behind; match the style of neighbouring files.
- `summary-stage-3.md` — the running handover. It records what phase 3 delivers,
  what was measured, the deliberate deviations from the prototype, and a "Traps
  that cost hours" section you should take seriously.
- `docs/design/pipeline.md` — the Assignment section, and the Capacity/Global
  ones for context.
- `docs/design/decisions/` — 0025 (the ring is manual input), 0026 (Global runs
  before Assignment), 0028 (the inner circuit pays for free space), 0029 (a
  feedline end is fed between launchers), 0030 (bridge pairs are declared).
  0029 is the one you will amend.

### Build, test, run

```bash
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-3
cmake --build build/dev && ctest --test-dir build/dev    # 254 tests today
uvx nox -s tests                                         # 120 tests x 4 Python versions
uvx nox -s lint                                          # must pass before you hand back
```

The CLI lives in the nox test environment:

```bash
export PATH="$PWD/.nox/tests-3-13/bin:$PATH"
mqt-scpd plan -c benchmarks/9q/config.toml -o artifacts/9q --stage capacity
mqt-scpd plan -c benchmarks/9q/config.toml -o artifacts/9q --stage global
mqt-scpd plan -c benchmarks/9q/config.toml -o artifacts/9q --stage assign
mqt-scpd plot -c benchmarks/9q/config.toml --stage assign --run-dir artifacts/9q -o artifacts/9q/9q-assign.svg
```

`--stage X` runs **only** that stage and resumes the run; there is no "up to X"
flag. `artifacts/` is not gitignored and is deleted before a commit.

### The trap that will cost you an hour if you skip it

**`mqt-scpd` runs the *installed* Python extension, not `build/dev`.** After any
C++ change you must reinstall, or you will be debugging a stale binary and
drawing wrong conclusions from it. This happened in the previous session:

```bash
for d in ~/.cache/uv/archive-v0/*/; do [ -f "$d/mqt/scpd/pyscpd.abi3.so" ] && rm -rf "$d"; done
uv pip install --python .nox/tests-3-13/bin/python \
    --no-build-isolation --no-deps --reinstall-package mqt-scpd -e .
```

`uv sync` alone does **not** rebuild the extension: it restores a cached archive
keyed by the package version, which does not move while the tree is uncommitted.
Always confirm behaviourally, never by a fresh `.so` timestamp. `ctest` runs
against `build/dev` and needs only `cmake --build`.

`uvx nox -s schemas -- --check` reports "stale" locally because the regenerated
code is uncommitted. That is expected and not your problem. If you change a
`.fbs`, run `uvx nox -s schemas` to regenerate.

---

## 1. Context — why this task exists

Capacity and Global are now correct on all eight benchmarks: every inner target
is served, one wire per target, one gate per narrowing. Assignment is the last
planning stage and **has not been re-run since the outer ring changed** on 17Q
and 69Q.

Reading it against the prototype shows it is wrong in ways the current tests
cannot see. There is exactly one assignment test, `AssignsTheFourQubitChip`
(`test/pipeline/test_benchmark_stages.cpp:501`), and its central assertion —
`connections.size() == 2` — pins the defect rather than the behaviour.

Three rules are to hold on **every** benchmark and to be checked in ctest:

1. **Every ring node is reached.** Every port of the outer ring the Global stage
   reported carries a connection in the assignment.
2. **A launcher feeds conventional ports, at most one each.** After the
   resonators are moved off onto the segments between launchers, a launcher slot
   carries at most one conventional port and no resonator at all.
3. **The ring keeps its cyclic order.** A launcher that comes cyclically after
   another is assigned to a port that comes cyclically after that other
   launcher's port.

---

## 2. How the stage works today

Files: `include/mqt-scpd/pipeline/Assigner.hpp` (62 lines),
`src/pipeline/Assigner.cpp` (413 lines). Registered as `"ordered-milp"` in
`src/pipeline/Registry.cpp`.

`assignmentInputs(chip, capacity, global)` (`Assigner.cpp:352-407`) turns the
solved stages into a plain value, `AssignmentInputs`:

- `ring` — `global.outer_ring` verbatim, in the Global stage's order. **Not** the
  configured `all_outer`: the inner circuit decides which coupler ports surface.
- `isResonator` — membership in `global.resonators`, not the chip's role.
- `launchers` / `launcherPosition` — from `capacity.launchers`, in plan order.
  That order is the index space the flow modulo works in.
- `nearestLauncher` — plain Euclidean nearest launcher slot. The prototype ran a
  graph search over the capacity grid *inside the model builder*; making it a
  value is what let the solver be abstracted. Do not undo that.
- `edgeWeight` — **dead**: filled with ones at `:405` and read nowhere. The real
  chord weights are recomputed in `ringEdgesOf`.

`ringEdgesOf` (`Assigner.cpp:57-79`) reduces the ring to its resonators
(`anchors`) and joins consecutive ones by a chord whose weight is the number of
conventional ports between them. That weight is the objective coefficient.

The model (`build`, `Assigner.cpp:130-271`):

| constraint | line | meaning |
| --- | --- | --- |
| `degree_i == 2` | 168-180 | each anchor has two of {chord in, chord out, launcher end, extra termination} — this carves the resonator cycle into open feedline chains |
| `flow_start` and `flow_step_*` | 192-204 | a monotone potential: pinned at the top, falls by at least one at every conventional port, and at a resonator only when it takes a launcher |
| `starts_*`, `util_*` | 206-237 | a counter along each chain, reset at its start, bounded by `max_feedline_utilization` |
| `launcher_target`, `termination_target` | 245-246 | **equalities**: exactly `launcher_target` anchors end at a launcher and exactly `feedline_terminations` end otherwise |
| `gap_*`, `wrap_*` | 248-267 | the proximity tie-break at `PROXIMITY_WEIGHT = 0.01` |

Objective: `Σ chord weight · edge + 0.01 · Σ gap`. The integer part is the
crossing count.

`launcher_target` is therefore the number of chain **ends** fed by a launcher, so
`(launcher_target + feedline_terminations) / 2` chains. The numbers are
consistent with that on every chip — 9Q: `(3+1)/2 = 2` chains for 10 resonators
at utilization 5; 69Q: `(24+0)/2 = 12` chains for 70 at 6.

`readBack` (`Assigner.cpp:282-347`) writes the artifact:

- `launchers` — parallel to `ring`, the launcher **port** at
  `flow mod |launchers|`, one entry per ring node.
- `feeds` — parallel to `ring`; the launcher slot position, except that a
  resonator ending a chain gets the midpoint to its neighbour's slot (decision
  0029).
- `connections` — **only** the anchors whose `launcher` binary is set.

Schema: `schemas/artifacts.fbs`, table `Assignment` (lines 140-155).

### Where the three rules stand

| rule | today | evidence |
| --- | --- | --- |
| 1 | **fails** | `Assigner.cpp:333-346` emits one connection per launcher-terminated anchor, `launcher_target` of them. On 4Q that is 2 connections for a 12-port ring. The prototype emits a request for **every** port (`OrderedAssignmentGraph.cpp:997-1000`). |
| 2 | **fails** | Only a chain *end* is moved off its launcher (`Assigner.cpp:311-331`). Every other resonator keeps the slot's own position, so a slot carries several nodes and at least one resonator. |
| 3 | probably holds | The potential is monotone. Two things could break it and neither is tested — see §4. |

---

## 3. What the prototype does about rule 2 — read this carefully

`/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD/include/fiction/layout/OrderedAssignmentGraph.cpp:792-843`,
the pass tagged `"zero"`. It groups the ports by the launcher index the MILP gave
them and, **for every group that contains a resonator at all** — not only groups
of more than one —

```cpp
uint32_t flow_min = node_flow;                              // the group's own launcher
uint32_t flow_max = (flow_min - 1 + max_flow) % max_flow;   // the NEXT launcher along
std::vector<std::string> new_launcher =
    capacity_grid.add_coupler_launcher_ports(launcher_labels_[flow_min],
                                             launcher_labels_[flow_max],
                                             occurs_resonators, "zero");
```

and hands the new points to that group's resonators **in ring order**
(`occurence_indices` is ascending node index and `counter` walks it in order).

The geometry is `CapacityGrid.cpp:7736-7843` — a straight line between the two
launcher slot points, divided evenly:

```cpp
double divisor = static_cast<double>(amount + 1);
double ratio   = static_cast<double>(i + 1) / divisor;
double exact_x = x0 + ratio * (x1 - x0);
double exact_y = y0 + ratio * (y1 - y0);
```

So `n` resonators land at `1/(n+1) … n/(n+1)` of the way, **never on either
end**, and the one earliest along the ring lands nearest its own launcher. The
next launcher is `flow_min - 1` because the potential *decreases* along the ring
walk.

Two consequences worth having clear before you write code:

- What is left standing on a real launcher slot is exactly the **conventional**
  ports. That is rule 2's first half.
- The potential falls strictly at every conventional port
  (`flow_step_*` with `-1.0`), so no two conventional ports share a slot. That is
  rule 2's second half, and rule 3 is the same monotone walk read differently.
  There is no explicit uniqueness constraint in either program; it is emergent.

Our decision 0029 is this same interpolation with `amount == 1`, applied to chain
ends only. It is a special case, and this task generalises it.

Ignore these dead ends in the prototype, they are not worth porting:
`NodeKind::Choice` (never constructed), `Node::position` (never set), the
`// Aufteilung in 2 * amount Segmente` comment (stale, contradicts its own code),
`max_index` at `:807` (unused), and the "`fake_endpoints` is ignored" claim in
`OrderedAssignmentGraph.hpp:176` (it is used, at `:698`).

---

## 4. The work

### Step 1 — measure the baseline first

The stage has not run since the ring moved, so **before changing anything**, run
all eight benchmarks through capacity, global and assign and record per chip:

- does it solve at all, and how long does assign take;
- `connections.size()` against `ring.size()`;
- how many ring nodes share each launcher slot, and how many resonators sit on
  one;
- whether the sequence of launcher slots along the ring is already cyclically
  monotone.

That table says which rules fail where and whether trap A below is real. Report
it; the rest of the work is guesswork without it. Note that 45Q was once seen to
take minutes under a variant formulation, so time the run.

### Step 2 — a connection per ring node (rule 1)

In `readBack`, replace the loop over anchors with a loop over the ring. Every
node gets a `ConnectionT` whose `target` is that port:

- a **resonator** keeps today's shape — `source` absent, because the port that
  feeds it is the coupler the Final stage inserts, with `ResonatorSource` /
  `ResonatorTarget`;
- a **conventional** port is fed from the launcher slot it was given, so `source`
  is `inputs.launchers[slotOf[index]]`, with `FeedlineSource` / `FeedlineTarget`
  — the roles `schemas/design.fbs:37-45` gives the launcher-terminated ends of a
  feedline chain.

`connections.size()` becomes `ring.size()` on every chip.

### Step 3 — move every resonator onto the segment (rule 2)

Replace the end-only midpoint (`Assigner.cpp:311-331`) with the prototype's
`"zero"` pass:

- group the ring indices by `slotOf`, keeping ring order within a group;
- for a group with `n > 0` resonators, take `here = launcherPosition[slot]` and
  `next = launcherPosition[(slot + launchers - 1) % launchers]`, and give the
  `i`-th resonator of the group `here + (i + 1) / (n + 1) * (next - here)`;
- conventional ports keep `launcherPosition[slot]`.

Get these right, both are named in the prototype's code: the next launcher is the
slot **below** by index, because the potential falls along the ring; and the ring
order within a group decides the order along the segment, which is what keeps
wires from crossing at sub-launcher scale. Where the two slots coincide there is
nothing to interpolate and today's guard (`Assigner.cpp:324-326`) carries over.

**Drop the prototype's endpoint refinement** (`OrderedAssignmentGraph.cpp:957-996`,
tags `"first"` and `"second"`). It exists to move a chain end off a launcher,
which the `"zero"` pass now does for every resonator, and a second overwrite
would put a feed back onto a point another node already uses — against rule 2.
Say so in the amendment to decision 0029.

### Step 4 — two traps to settle before trusting rules 2 and 3

Both were found by reading and neither is covered by a test.

- **Trap A, the anchor counter** (`Assigner.cpp:193-203`). `anchor` starts at `0`
  and is pre-incremented, so the first resonator is charged to
  `variables.launcher[1]`. That is right only when `inputs.ring[0]` is itself a
  resonator. 4Q's ring begins at `Q1.port0`, a resonator, but the ring is now the
  Global stage's own and this must be checked per chip rather than assumed. If a
  ring can start on a conventional port, every launcher constraint is shifted by
  one anchor and rule 3 is not guaranteed. Check it in step 1.
- **Trap B, the open-side selection** (`Assigner.cpp:322-323`). It picks the side
  the run *arrives* from, while the comment two lines above it and decision 0029
  both say the side the run does *not* continue on. Step 3 deletes this branch,
  so the question resolves itself — but do not carry the same mistake into the
  new code.

### Step 5 — documentation

| File | Change |
| --- | --- |
| `schemas/artifacts.fbs` | the `feeds` comment now describes every resonator, not only a chain end |
| `docs/design/decisions/0029-a-feedline-end-is-fed-between-launchers.md` | append an `> **Amended <date>.**` block, the way 0018, 0027 and 0028 do: the interpolation is for every resonator, at `1/(n+1) … n/(n+1)`, and the endpoint refinement is dropped |
| `docs/design/pipeline.md` | the Assignment section: what a connection is now, and the two rules the monotone walk gives |
| `CHANGELOG.md` | one entry per defect fixed, under `## [Unreleased] / ### Added`, in the existing style with `([#114]) ([**@FeldmeierMichael**])` |
| `summary-stage-3.md` | what this session did, the new figures, and what is left open |

---

## 5. The tests

All three rules, on **all eight benchmarks**, in ctest, mandatory. This is what
the user asked for explicitly.

### Fixtures for the five missing chips

`test/pipeline/test_benchmark_stages.cpp` covers 4Q, 9Q and 17Q. Its `load()`
builds each configuration by hand and reads only the two port sequences out of
the shipped TOML, "because a test that built its own ring would not be testing
the ring the chip actually ships with" (`:64-67`).

The same argument applies to the scalars, and hand-copying numbers into five more
fixtures would be five more places to drift. **Add a `readScalar(text, key)`
beside the existing `readArray`**, and let `load()` read `capacity_cells_x`,
`launcher_offset_x`, `launcher_target`, `max_feedline_utilization` and
`feedline_terminations` from the file. The role patterns stay parameters, because
4Q's naming (`Q1`, `C12`, coupler ports 0-2) differs from the other seven
(`Qb1`, `Coupler1_2`, ports 0-4) and that difference is the point of them.

The per-chip figures, for checking your fixture reads them right:

| chip | capacity_cells_x | launcher_offset | launcher_target | utilization | terminations | ring | resonators |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 4q | 12 | 5 | 2 | 4 | 0 | 12 | 4 |
| 9q | 12 | 5 | 3 | 5 | 1 | 30 | 10 |
| 17q | 25 | 20 | 7 | 5 | 1 | 58 | 20 |
| 21q | ? | ? | 10 | 5 | 0 | 70 | 22 |
| 33q | ? | ? | 14 | 5 | 0 | 110 | 34 |
| 45q | ? | ? | 15 | 6 | 1 | 150 | 46 |
| 57q | ? | ? | 18 | 7 | 0 | 190 | 58 |
| 69q | ? | ? | 24 | 6 | 0 | 230 | 70 |

(The `?` are in each `benchmarks/<chip>/config.toml`; read them, do not guess.)

### One test per chip, one pipeline run

A `TEST_P` over the eight chip names, so each chip is its own ctest entry with
its own name and its own failure message, while capacity, global and assign run
**once** per chip:

```cpp
TEST_P(BenchmarkAssignment, FollowsTheRulesOfTheRing) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto plan   = capacityPlanners().make("watershed")->run(...);
  const auto global = globalRouters().make("hanan-milp")->run(...);
  const auto assign = assigners().make("ordered-milp")->run(...);
  expectEveryRingNodeIsReached(benchmark, global, assign);
  expectALauncherFeedsOneConventionalPort(benchmark, plan, global, assign);
  expectTheRingKeepsItsCyclicOrder(plan, assign);
}
```

The three checks, as free functions in the test file:

1. **Every ring node is reached.** `connections.size() == ring.size()`, and the
   set of `connection->target` covers `assignment.ring` exactly.
2. **A launcher feeds one conventional port.** For every ring node whose `feeds`
   point equals some `plan.launchers[*].position` — a launcher slot, not an
   interpolated point — count it against that slot. Every slot's count is at most
   one, and no such node is in `global.resonators`. Conversely, every resonator's
   feed is on **no** launcher slot.
3. **Cyclic order.** Walk `assignment.launchers` along the ring and map each to
   its index in `plan.launchers`. The sequence must be cyclically monotone: it
   never rises except at exactly one wrap. That is the user's statement — a
   launcher later in the cycle is given a port later in the cycle — and it is
   what the monotone potential is supposed to produce.

`AssignsTheFourQubitChip` shrinks to what remains chip-specific about 4Q (its
ring is its whole port set and it has no inner circuit); its
`connections.size() == 2` assertion is replaced by rule 1.

### Cost, and what to do if it is too slow

Capacity alone is about 93 s over the eight chips, 48 s of it 69Q; global adds a
second or two; assign is unmeasured since the ring changed. ctest goes from 11 s
to a few minutes and runs on Ubuntu, macOS and Windows in CI. The user chose this
knowingly. If assign turns out to cost far more than capacity on the large chips,
report the measured times and ask before trimming anything — do not quietly drop
a chip.

---

## 6. Verification and reporting

```bash
cmake --build build/dev && ctest --test-dir build/dev
for d in ~/.cache/uv/archive-v0/*/; do [ -f "$d/mqt/scpd/pyscpd.abi3.so" ] && rm -rf "$d"; done
uvx nox -s tests
uvx nox -s lint
```

Then all eight benchmarks end to end, and the renderings regenerated so the
chords can be looked at:

```bash
export PATH="$PWD/.nox/tests-3-13/bin:$PATH"
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  for s in capacity global assign; do
    mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c --stage $s
  done
  for s in capacity global assign; do
    mqt-scpd plot   -c benchmarks/$c/config.toml --stage $s --run-dir artifacts/$c -o artifacts/$c/$c-$s.svg
    mqt-scpd render -c benchmarks/$c/config.toml --stage $s --run-dir artifacts/$c -o artifacts/$c/$c-$s.gds
  done
done
```

Report, per chip: assign time, objective, the crossing count (the integer part),
connections against ring length, the number of distinct feed points, and whether
each of the three rules holds. Say plainly which figures moved and why. If a rule
cannot be made to hold on some chip, say which chip and what blocks it rather
than weakening the test.

Nothing is committed. Leave `artifacts/` in place for the user to look at and say
so; it is deleted before a commit, together with any stray `*.svg` / `*.gds` at
the repository root.
