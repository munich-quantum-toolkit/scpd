# Phase 4, step 3 → the Final stage — handover

Written for whoever implements the **Final** stage. The Detail stage is done,
verified and drawn, and it holds the design rule everywhere. This file says what
is finished, what the ground truth is, and — most of all — **what cost time and
should not cost it twice.**

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, based on `8ef300a`. **Nothing is committed —
  the user commits per phase.** Leave your work in the tree.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only. The Final stage is `include/fiction/layout/FinalGrid.cpp`, and it
  is **23 413 lines** — an order of magnitude more than `DetailedGrid.cpp`.
- What is finished: [summary-stage-4.md](summary-stage-4.md) (Corridor),
  [summary-detail-routing.md](summary-detail-routing.md) (Detail). The two
  earlier briefings, [handover-detail-routing.md](handover-detail-routing.md)
  and [handover-detail-clearance.md](handover-detail-clearance.md), are history
  now: **both of their requirements are met.**

## Where the pipeline stands

Five of seven artifacts are written, and the sixth is yours:

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb  →  06-final.fb
```

| chip | connections | drawn | inner | cells | bends | longest | detail | clearance |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 |   0/0 |    550 |   41 |  64 |  0.38 s | held |
| 9q  |  30 |  30 |   6/6 |   2644 |  116 |  98 |  0.09 s | held |
| 17q |  58 |  58 | 14/14 |   7891 |  553 | 202 |  0.33 s | held |
| 21q |  70 |  70 |   8/8 |  12454 |  717 | 221 |  0.60 s | held |
| 33q | 110 | 110 |   8/8 |  25916 | 1346 | 308 |  1.63 s | held |
| 45q | 150 | 150 |   7/7 |  42869 | 2744 | 396 |  4.58 s | held |
| 57q | 190 | 190 | 11/11 |  71230 | 4555 | 575 |  9.66 s | held |
| 69q | 230 | 230 | 11/11 | 111345 | 5478 | 700 | 15.50 s | held |

**968 of 968 connections and 65 of 65 inner connections, and no two wires
anywhere within the design rule.** 314 of 314 ctest cases, 129 Python tests.

## What the Final stage has to do

`docs/design/pipeline.md` § Final. The prototype's live sequence:

```text
inner routing        stubs inside each unit cell
outer routing        the resonator and conventional wires
coupler insertion    CPW couplers instantiated; ResonatorSource ports created
feedline routing     the launcher-to-launcher chains, with rip-up repair
feedline refinement
```

with a clearance check and a wire-loop check after every routing pass. Two
things the design document already fixes and you should not re-litigate:

- **Coupler placement and feedline routing are one fixpoint, not two steps.**
  The repair loop re-orients an already-placed coupler when that is what lets a
  chain route. `IFinalRouter` has to be shaped around that.
- **The obstacle keepout is baked into the raster mask**, not checked
  afterwards. Every cell a search may enter already satisfies it.

Your input is the Detail stage's cell paths. They are a **centre line on a
grid**, not geometry: your job is to turn them into curvature-constrained
copper with real widths, and the clearance question comes back in a form the
cell grid could not express.

---

# Lessons learned — read this part twice

These are the things that cost a day each. Every one of them is measured, not
believed.

## 1. A rule in layout units is not a rule in cells, and the check must use the same one

`min_wire_spacing` is 185 layout units. The detail cell is 19 units on the
17-qubit chip and 40 on the 9-qubit one. A router that works in cells **cannot**
keep 185: what it keeps is `ceil(185 / cell) - 1` cells, which is 4 to 9 cells
and 153 to 182 layout units.

| chip | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cell (layout units) | 19.2 | 39.6 | 19.0 | 39.9 | 38.4 | 36.4 | 31.7 | 36.0 |
| rule in cells | 9 | 4 | 9 | 4 | 4 | 5 | 5 | 5 |
| what that is | 173 | 158 | 171 | 160 | 154 | 182 | 158 | 180 |

I first wrote the check in exact layout units — forbid a cell when
`(dx·cellW)² + (dy·cellH)² < 185²`. It is arithmetically right and it is the
wrong check: it asks the grid to tell apart two answers that are the same
drawing, and it reported three failures on the 57-qubit chip that were pairs of
wires 5.66 cells apart against a rule of 5. **Convert the rule once, enforce the
converted figure, and check the converted figure.**

For your stage the conversion is different — the final grid is finer, and
`cells_for(min_wire_spacing, final)` **is** 19 there — but the discipline is the
same: one conversion, used by the router, the test and the picture alike.
`docs/design/data-model.md` § clearance has the table of every place a clearance
value comes from.

## 2. Never quote a distance as a cell count

Every knob of this stage used to be a cell count, and a cell count is not a
distance. `corridor_half_width = 40` cells is 760 layout units on one grid and
1600 on another — four wire spacings there and eight here, from the same
literal. `obstacle_penalty_radius = 6` cells is 114 units and 240 units.

They are now `corridor_spacings` (4 wire spacings) and `obstacle_penalty_reach`
(185 layout units), converted through the grid at the point of use. **Prices** —
what a step costs, what a penalty costs — are quoted per *step*, against the ten
a step along an axis costs, and that is already grid-independent: a way of a
given length is twice as many steps on the fine grid as on the coarse one, so a
price per step is the same fraction of it either way. A price per *cell of a
clearance disc* is not: the disc of one wire spacing holds 298 cells on the
finest grid and 83 on the coarsest.

Do the same. `FinalGridParams` in the prototype is full of cell counts.

## 3. A plan's crossings are a coarse route, not a constraint

The Corridor stage writes, for each wire, the partitions it passes through and a
crossing point on each border between them. The previous version of the Detail
stage treated those crossings as **hard**: it claimed the cells, cut the wire
into pieces, and made each piece begin and end exactly there.

Measured: the crossings a plan names sit **13 to 28 layout units apart** on the
benchmark chips, against a rule of 185. No arrangement that runs through them
can hold the rule. That single decision accounted for most of the 457 failures I
started from.

The corridor is a guide. Seed with it, then draw the wire again from end to end
and let it cross where it can. The same will be true of the Detail stage's cell
paths for you: they are where a wire roughly goes, not where its centre line has
to be to the nanometre.

## 4. The fixed places are the ones that must never lose their clearance

A wire has two places it cannot be moved off: the point the assignment feeds it
at, and the cell of its target port. Everything between them is negotiable.

The bug that cost the most: while a wire is lifted off the canvas to be
re-routed, its **two fixed places must keep their clearance charged**. Only the
wire currently being drawn is let out of its own two places, and only for the
length of its own search. Otherwise a wire drawn while another one is lifted
settles within a wire spacing of where that other one has to return to — and
that is a violation no later round can undo, because neither wire can move the
place. Fixing this alone took the 17-qubit chip from 4 failures to 0 and the
whole set from 457 to 175.

Measured beforehand, and worth knowing: **the fixed places themselves are far
enough apart.** The minimum distance between any two endpoints is 192.8 to 543.8
layout units across the eight chips, all ≥ 185. The plan is feasible; only the
router was in the way.

## 5. Count a wire's violations with the wire off the field

The mirror image of lesson 4. After re-routing a wire I re-charged its two ends
and *then* counted how many of its cells lay within the rule of another wire.
Its own two ends are within the rule of the cells beside them, so every wire was
too close to itself, no wire was ever marked finished, and every sweep re-routed
every wire. It still converged — which is why it was easy to miss — but it
wasted most of the runtime and it hid how many sweeps were really needed.

**Lift the wire, count, then put it down.** `recount()` in
`src/pipeline/DetailRouter.cpp` is the pattern.

## 6. Follow the prototype, and read its call graph rather than its functions

This is the biggest one, and it is the user's own correction to me.

I spent hours on mechanisms of my own — a sequential "spread" pass that lays
every wire down in turn, a PathFinder-style congestion history, an
accept-only-if-better test on every swap. They took the failures from 457 to
175. Then I threw all of it away, implemented `detailed_cross_boundary_routing`
as the prototype actually writes it, and the same measurement gave **46** — and
it ran three times faster. The remaining 46 came down with the prototype's own
parameters turned up, and the last 3 with a mechanism the prototype describes in
a comment.

But **read what runs, not what is written**. The prototype's defects that matter:

- `astar_in_corridor` has **no occupancy test at all**. Two wires are kept apart
  only by the disc `build_corridor` cuts around the *two* neighbours in the
  ring. That is why the prototype's own pictures show wires touching.
- `for (back_count = 0; back_count <= 0 && !success; ++back_count)`
  (`DetailedGrid.cpp:1587`) runs its body **exactly once**. The comment above it
  describes an escalation into the opposite sweep direction that never happens.
  Implementing what the comment says took the last three failures on the
  57-qubit chip to zero. **Look for more of these in `FinalGrid.cpp`.**
- `DetailedGridParams::I` and `neighbor_path_proximity_penalty` are read in two
  passes and used in neither: both calls that look as though they use them pass
  `corridor_polygon_penalty`.
- `run_inner_routing_requests` wraps its search in a relaxation that cannot do
  anything — what it rips changes neither the mask nor the costs.
- Its `DetailFail 0` is scraped from the **last round** of the cross-boundary
  pass, and a wire with no path is skipped before it can be counted. Its own
  logs start that pass at 37 failures on the 69-qubit chip. **Do not treat its
  published zero as the bar.**

## 7. Its parameters were tuned for a weaker rule

`rounds = 8` and `max_relaxation = 10` are the prototype's, and they are enough
for the clearance it holds — to two wires, which settles in a couple of sweeps.
Holding the rule against every wire is a harder question:

| | fails over the eight chips |
| --- | ---: |
| rounds 30, relaxation 30, with back-rip | **0** |
| rounds 30, relaxation 30, no back-rip | 3 |
| rounds 8, relaxation 10 | 46 |
| rounds 60 or 100 | 11 → no further gain without back-rip |

More sweeps stop helping at 30. When a number stops paying, stop turning it and
find the missing mechanism instead — that is how the back-rip was found.

## 8. Make the global constraint a field on the canvas, not a mask per search

The prototype rebuilds an `allowed` mask per search: the box around the wire's
own way, minus a disc around two neighbours. Cutting a disc for **every** wire
that way costs the whole chip's copper per attempt — on the 69-qubit chip,
111 345 cells × 83 disc cells, per search, per wire, per round.

Instead the canvas carries `guard_[cell]`: how many wire cells lie within the
rule of that cell. A wire charges the disc around each of its cells when it is
put down and discharges it when taken off; a search that must hold the rule
refuses any cell with `guard_ > 0`. **The rule against 240 wires then costs what
the rule against two cost**, the test is one comparison, and "ripping" a wire is
exactly "discharge its clearance, leave its copper". You will want the same
structure for whatever the Final stage's global constraint turns out to be.

## 9. Measure before you build, and keep the measurement runnable

The single most useful thing I made was a 90-line script that reads
`05-detail.fb` back and counts violations. It gave a number after every change,
in two seconds, without running the test suite. Build yours first.

```bash
# artifacts, then the number
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  rm -f artifacts/$c/05-detail.fb
  .venv/bin/mqt-scpd plan -c benchmarks/$c/config.toml -o artifacts/$c --stage detail
done
```

A second thing worth the ten minutes: a debug switch on the stage that prints an
**ASCII map** of the neighbourhood of a violation — obstacles, each wire by a
letter, the charged clearance, free space. Two of the three real bugs above were
found by looking at one. It is temporary code; take it out again.

## 10. The traps that are still traps

1. **The corridor stage writes a cell as the layout point of its corner-frame
   centre.** Reading one back is `floor(onGrid(toCell(p)))`, never
   `roundToCell`. Rounding moves the whole feed-point rectangle one cell, and
   111 of the 230 wires of the 69-qubit chip then cannot take their first step.
   `cellAt` in `DetailRouter.cpp` is the one conversion; use it.
2. **The diagonal-crossing rule must ask whether *one* wire holds both cells
   beside the step.** Refusing whenever either corner is taken also refuses the
   harmless squeeze between two different wires, and that costs connections.
3. **`uv sync` does not rebuild the extension when only C++ changed.** Use
   `uv pip install --python .venv/bin/python --no-build-isolation --no-deps
   --reinstall-package mqt-scpd -e .` and check behaviour, not the `.so`
   timestamp. `cmake --build --preset release` alone updates ctest but **not**
   the Python module.
4. **`uvx ruff check` rewrites every `# noqa: <code>` into
   `# ruff: ignore[<name>]` across the whole repository.** Revert every file you
   do not own. `uvx nox -s stubs` still emits invalid Python for the `global`
   stage, so `python/mqt/scpd/pyscpd.pyi` is maintained by hand.
5. **Long runs belong in the background.** The whole ctest suite is 14 minutes;
   the 69-qubit detail cases alone are 87 seconds each.
6. **After any `.fbs` change, run `uvx nox -s schemas`** and update
   `python/mqt/scpd/config.py` (`STAGE_DEFAULTS` **and** `_read_stages`) in the
   same commit — the Python side does not follow the schema by itself.

## 11. What was built, measured and thrown away

Do not rebuild these without a new measurement.

| mechanism | verdict |
| --- | --- |
| A sequential pass laying every wire down in turn from a clean canvas | Better on two chips, worse on two others, and it loses wires; the prototype's sweep beats it |
| A PathFinder congestion history over the rounds | Made it worse (4 → 5 unresolved on 17q); the rip-up already does what it is for |
| Accepting a swap only when it strictly lowers the conflict count | Blocks the lateral moves the next round needs; `<=` beat `<`, and the prototype's "no test at all" beat both |
| Naming the wires in the way by a priced search instead of ring order | Helped a lot in my own version, and is unnecessary once the sweep is the prototype's |
| A price for coming within the rule, as a last resort so a wire is drawn anyway | Not needed: a wire that finds nothing keeps the way it had, which is the prototype's answer and is always legal |
| The obstacle proximity penalty | Neither better nor worse on any chip with the clearance in force. Kept, because it keeps copper off the artwork where there is room |
| The seed's rescue passes (`placeWhatIsLeft`, `drawWhatIsLeftWhole`) | **Kept — still earn their place.** Without them 45q loses 5 connections and 69q 1 |

## Verification

```bash
cmake --build --preset release && ctest --preset release   # 314 tests
uv run --no-sync pytest test/python/unit                   # 129 tests
uvx nox -s schemas                                         # after any .fbs change
```

The eight benchmarks and the pictures:

```bash
for c in 4q 9q 17q 21q 33q 45q 57q 69q; do
  mqt-scpd plan   -c benchmarks/$c/config.toml -o artifacts/$c --stage detail
  mqt-scpd plot   -c benchmarks/$c/config.toml --stage detail --run-dir artifacts/$c \
                  -o artifacts/$c/$c-detail.svg
  mqt-scpd render -c benchmarks/$c/config.toml --stage detail --run-dir artifacts/$c \
                  -o artifacts/$c/$c-detail.gds
done
```

`plot --stage detail` draws every wire with a band of the clearance the router
keeps, so two wires closer than that are two bands whose overlap is darker;
`render --stage detail` writes the same band on GDS layer 23 `plan.clearance`,
where a boolean finds the overlaps exactly.

## What is open for you

1. **There is no design-rule stage.** The clearance is checked in the Detail
   stage's own tests and drawn in its pictures. Phase 5's DRC has to make the
   same check on finished geometry, where a wire is a polygon with a width and
   not a run of cells — and `docs/design/pipeline.md` § Design-rule checking
   already writes down the rule set it must implement.
2. **The corridor is a guide and no longer a constraint.** A wire may use a
   partition its corridor did not name, so the capacity stage's wire budgets are
   a plan rather than a bound.
3. **`[stages.capacity] crossing_pitch` is 165 against a wire spacing of 185.**
   It no longer blocks anything, because the crossings are a seed — but it is
   still a number saying a border carries more wires than the rule allows, and
   the budgets follow from it.
4. **Nothing has been compared against the prototype's own output**, only
   against its formulation as read and its published counts. A run of FridgeCAD
   on 9Q side by side would still be worth more than another test.
