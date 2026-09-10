# Phase 4, step 2 → step 3 — handover

Written for whoever picks the Detail stage up next. The stage is implemented,
verified and drawn; what it does **not** yet do is hold the wire clearance, and
that is now a hard requirement rather than an open question. Read
[summary-detail-routing.md](summary-detail-routing.md) for the stage itself;
this file says what is done, what is binding, and what was already measured so
that it is not measured twice.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, based on `8ef300a`. **Nothing is
  committed — the user commits.** Leave your work in the tree.
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`),
  read only.
- Step 1 (Corridor): [summary-stage-4.md](summary-stage-4.md). Step 2's
  briefing: [handover-detail-routing.md](handover-detail-routing.md).

## The one thing that is not done

**Every minimum clearance must hold. A wire that comes closer to another than
the design rule allows is a failure, exactly like a wire that was not drawn.**
That is the standing requirement from here on, and the stage does not meet it.

What the stage holds today, measured on the written artifacts:

| chip | clearance it keeps | pairs inside it | of which ring neighbours | cells inside it |
| --- | ---: | ---: | ---: | ---: |
| 4q, 9q, 17q, 21q, 33q | 154–173 | **0** | 0 | 0 |
| 45q | 182 |  9 |  6 |  675 of  43163 |
| 57q | 158 | 14 | 11 |  930 of  71611 |
| 69q | 180 |  6 |  4 |  618 of 111726 |

Two independent reasons, both verified:

1. **The clearance is only ever enforced between a wire and two others** — the
   wire before it and the wire after it in the ring. That is `build_corridor`'s
   `other_paths` in the prototype and `clearAround` in `buildBand` here. Against
   the other 227 wires the only rule is "no shared cell", which is one cell.
   Three of the nine offending pairs on the 45-qubit chip are such non-neighbour
   pairs.
2. **It is only applied in the cross-boundary pass.** The per-partition pass
   that draws the pieces has no clearance at all, and a wire whose
   cross-boundary re-route fails keeps that piece-built path. This is the larger
   effect: switch the relaxation off, so that far more wires keep their
   piece-built path, and the offending pairs go from 9/14/6 to **63/108/128**
   while six connections of the 45-qubit chip go unrouted as well.

### What is already measured about fixing it

**Enforcing the clearance against every wire, not just two, still draws every
connection.** In `buildBand`, replacing the two `clearAround` calls with one per
wire was tried on the three chips that fail today: 45q, 57q and 69q each drew
all of their connections, at roughly twice the runtime (the whole test,
including the four earlier stages, went from 15/32/79 s to 30/62/134 s). That is
the encouraging half.

The discouraging half, and the reason this is not simply done:

- **That run says the connections were drawn, not that the clearance held.** The
  per-partition pass still has none, so a wire that keeps its piece-built path
  still violates it. Carrying the clearance into the per-partition pass as well
  is the part that has *not* been tried, and it is the part that will hurt: a
  piece is drawn between two crossings the corridor fixed, and those crossings
  sit one crossing pitch apart.
- **The crossing pitch is smaller than the wire spacing, so the plan itself
  cannot be realised with the rule respected.** `[stages.capacity]
  crossing_pitch` is 165 layout units and `min_wire_spacing` is 185. Two wires
  that cross one border at adjacent slots are 165 apart at that point **by
  construction**. Until that number changes, no detail router can hold 185 there
  — and the border budgets that follow from the pitch get smaller when it does.
  This is the corridor stage's own open question 1
  ([summary-stage-4.md](summary-stage-4.md)), and it is now blocking.

So the work is a pair: raise the pitch to at least the spacing in the Capacity
stage and re-run the Corridor stage, then enforce the clearance everywhere in
the Detail stage. Doing only the second will fail at the crossings; doing only
the first changes nothing on its own.

### The clearance is a cell count, and the picture draws that

The rule is a length; the router works in cells. The conversion is the
prototype's own, `ceil(min_wire_spacing / cell) - 1` cells
(`DetailedGrid.cpp:1455`), and it is 4 to 9 cells on the eight benchmarks — 153.5
to 182.0 layout units against a rule of 185. **A router on this grid cannot keep
185 at all**, and everything above is measured against what it can keep.
`blockadeRadius` in [src/pipeline/DetailRouter.cpp](src/pipeline/DetailRouter.cpp)
derives it; `planning.blockade` in
[python/mqt/scpd/planning.py](python/mqt/scpd/planning.py) converts it back for
the picture. Never read the literal 6 the prototype also carries: it is right on
none of the eight.

## What the stage does today

Eight-connected A\* on the detail grid, costs `{10, 14}`, octile heuristic — the
prototype's figures. Four passes, and one rescue between the second and the
third:

1. **Inside each partition.** A corridor cuts its wire into one piece per
   partition it names; each piece is searched for inside that partition alone,
   between the two crossings the plan gave it. The pieces of one partition are
   tried in up to `orderings` orders, and the order that draws the most wins.
   **This is the only pass that knows about partitions**, exactly as in the
   prototype: `astar_in_cell` takes the label grid, `astar_in_corridor` does not.
2. **Joined by concatenation**, in corridor order. The corridor already says
   which piece is whose, so there is no reconstruction pass and no cell is lost
   at either end.
3. *(rescue)* A wire the pieces could not join is searched for once more from
   end to end over the free space; where that finds nothing, the wires actually
   in its way are taken out and drawn again after it, and a displaced wire may
   displace in turn one level deep.
4. **Re-drawn inside a band**, `corridor_half_width` cells around the way it
   has, with the clearance kept to its two ring neighbours; on failure the
   prototype's relaxation lets go of the wire ahead one place further each time
   and steers with two prices instead of confining.
5. **The inner circuit last**, over the whole free space with the ring as hard
   blockers.

**No two wires share a cell**, and no two cross between cells: a diagonal step
past a corner both of whose cells belong to *one* wire is refused, which is the
only crossing that shares no cell.

### The figures

| chip | connections | drawn | inner | cells | bends | longest | detail | SVG |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 |   0/0 |    550 |   14 |  64 | 0.01 s | 0.17 MB |
| 9q  |  30 |  30 |   6/6 |   2647 |  121 |  98 | 0.02 s | 0.35 MB |
| 17q |  58 |  58 | 14/14 |   7893 |  524 | 205 | 0.17 s | 1.06 MB |
| 21q |  70 |  70 |   8/8 |  12464 |  782 | 221 | 0.22 s | 0.93 MB |
| 33q | 110 | 110 |   8/8 |  25991 | 1667 | 308 | 0.96 s | 1.74 MB |
| 45q | 150 | 150 |   7/7 |  42927 | 3423 | 397 | 1.91 s | 2.67 MB |
| 57q | 190 | 190 | 11/11 |  71254 | 6575 | 575 | 3.37 s | 4.42 MB |
| 69q | 230 | 230 | 11/11 | 111407 | 9822 | 702 | 6.35 s | 6.44 MB |

All 968 connections and all 65 inner ones are drawn. **306 of 306 ctest cases
pass**, 129 Python tests. Runtime 0.01 to 6.35 s against the prototype's 0.08 to
46.9. Two runs of the same input produce the same bytes. Every SVG is inside the
10 MB budget; the GDS files are 0.06 to 2.03 MB. The pictures are current, in
`artifacts/<chip>/<chip>-detail.svg` and `.gds`.

## What the prototype's own zero means

Its table reports `DetailFail` 0 on all eight, and that is a **weaker claim**
than this stage makes. Do not treat it as the bar.

- The number is scraped by `scripts/run_layout_benchmarks.py:21` from
  `[Boundary Optimize] … Failed N` of the **last round** of the cross-boundary
  pass. A wire with no path is skipped there before it can be counted
  (`if (wire.path.size() < 2) continue;`), and a failure there only means the
  wire kept the way it already had, because `wire.path` is replaced on success
  alone. Its own logs start that pass at 37 failures on the 69-qubit chip and
  reach 0 in the second round.
- Its per-partition pass does route everything — 4747 of 4747 pieces on the
  69-qubit chip, 0 reconstruction failures — because **occupancy is not an
  obstacle there at all.** `astar_in_cell` has no occupancy test: a cell another
  wire holds costs 50 to run over, and the crossing test kept beside it is only
  built when some wire pixel is reachable from both ends, so where none is, it
  does not run either.
- Ported into this stage, that rule also draws every connection — and costs 57
  cells carrying two wires across 32 pairs (17q 4, 21q 2, 33q 10, 45q 24, 57q
  13, 69q 4). The rule this stage keeps instead costs nothing.

Its `DetailedGridParams::I` is read in two passes and used in neither, and so is
`neighbor_path_proximity_penalty`: both calls that look as though they use it
pass `corridor_polygon_penalty`. The `back_count` loop at `DetailedGrid.cpp:1587`
runs its body exactly once. `run_inner_routing_requests` wraps its search in a
relaxation that cannot do anything, because what it rips changes neither the
mask nor the costs.

## The traps that cost time

1. **The corridor stage writes a cell as the layout point of its corner-frame
   centre.** Reading one back is `floor(onGrid(toCell(p)))`, never
   `roundToCell`. Rounding moves the whole feed-point rectangle one cell, and
   111 of the 230 wires of the 69-qubit chip then cannot take their first step.
   `cellAt` in `DetailRouter.cpp` is the one conversion; use it.
2. **The diagonal rule must ask whether *one* wire holds both cells beside the
   step.** Refusing whenever either corner is taken also refuses the harmless
   squeeze between two different wires, and that costs connections.
3. **Partitions belong in the first pass only.** Carrying them into the later
   passes left two connections unrouted and three times the clearance
   violations. The corridor tells the pieces where to go; once the pieces do not
   fit, it has said all it has to say.
4. **A rule the prototype applies where failing is free is not the same rule
   applied where failing is fatal.** Its launcher clamp lives in
   `astar_in_corridor` only. Applied to the per-partition pass it is fatal.
5. **`uv sync` does not rebuild the extension when only C++ changed.** Use
   `uv pip install --python .venv/bin/python --no-build-isolation --no-deps
   --reinstall-package mqt-scpd -e .` and check behaviour, not the `.so`
   timestamp.
6. **`uvx ruff check` rewrites every `# noqa: <code>` into
   `# ruff: ignore[<name>]` across the whole repository.** Revert every file you
   do not own and map the comments back in the ones you do.
   `uvx nox -s stubs` still emits invalid Python for the `global` stage, so
   `python/mqt/scpd/pyscpd.pyi` is maintained by hand.
7. **Long runs belong in the background.** The 69-qubit detail test alone is
   79 s and the whole suite 11 minutes.

## What was built, measured and thrown away

Do not rebuild these without a new measurement.

| mechanism | verdict |
| --- | --- |
| The prototype's sweep over the wires beside one in the ring | Places no further connection on any chip; turned a 26 s run into more than 20 minutes |
| A price for crossing where a displaced wire used to be | Places nothing at a price of 10, loses two connections at 40 |
| A repeat pass offering the leftovers the room the others left | Places nothing once the rest is in place |
| The clamp keeping a wire inside the rectangle the feed points span | Changes not one cell: without it, no wire of the eight chips puts a single cell outside it, because the blocked border strip and the launcher sweeps already do |

And what does earn its place, ablated at the final state — connections left over
of 968: as shipped **0**; without the displacement chain 2; without the price
for running beside another wire 3; without the targeted rip-up 8; without the
end-to-end rescue 11.

## The files

| file | what |
| --- | --- |
| `schemas/artifacts.fbs` | `DetailWire`, and `DetailRouting` with `grid`, `wires`, `inner` |
| `schemas/config.fbs` | `DetailParams`, and `detail` in `StageParams` |
| `include/mqt-scpd/pipeline/Stages.hpp` | `IDetailRouter` |
| `include/mqt-scpd/pipeline/DetailRouter.hpp`, `src/pipeline/DetailRouter.cpp` | the stage, `makePixelAStarRouter()` |
| `include/mqt-scpd/pipeline/Registry.hpp`, `src/pipeline/Registry.cpp` | `detailRouters()`, default `"pixel-astar"` |
| `src/io/Artifacts.cpp` | a `case` in `validate`: a wire's cells step by at most one |
| `bindings/bindings.cpp`, `python/mqt/scpd/pyscpd.pyi` | `route_detail(...)` |
| `python/mqt/scpd/{run,planning,plot,artifacts,inspection,config}.py`, `export/klayout.py` | the stage, the read-back, the layers, the clearance band |
| `test/pipeline/test_detail_router.cpp` | the invariants over all eight chips |
| `test/io/test_artifacts_schema.cpp`, `test/python/unit/*` | the schema and the Python side |

`docs/design/pipeline.md` has the stage's write-up and
`docs/design/data-model.md` the derived clearance; `CHANGELOG.md` is under
`[Unreleased]`.

## Verification

```bash
cmake --build --preset release && ctest --preset release   # 306 tests
uv run --no-sync pytest test/python/unit                   # 129 tests
uvx nox -s schemas                                         # after any .fbs change
```

The eight benchmarks, and the pictures:

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
where a boolean finds the overlaps exactly. **The picture is how a clearance
failure is seen; the number is how it is counted, and there is no design-rule
check yet — writing one is part of the job above.**

## What else is open

1. **The corridor is a guide and no longer a constraint.** Only the first pass
   is confined to the partitions the plan names. A wire may therefore end up
   using a partition its corridor did not name, and the wire budgets the
   capacity stage counted are a plan rather than a bound. Whether that matters
   is a question for the capacity model and for the design-rule check.
2. **`digTargetBeyondBand` is still called by nothing, and still not needed.**
   The corridor stage adds a port's way out of its own pocket to the partition
   graph; this stage never had to dig it, and every wire that crosses a pocket
   is drawn. Measured, not assumed.
3. **Nothing has been compared against the prototype's own output**, only
   against its formulation as read, its published failure counts and its own
   logs. A run of FridgeCAD on 9Q side by side would still be worth more than
   another test.
