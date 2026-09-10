# Phase 4, step 2 — the Detail stage

What was built, what was measured, what is deliberately different from the
prototype, and what is still open. Step 1 is
[summary-stage-4.md](summary-stage-4.md); the briefing this answers is
[handover-detail-routing.md](handover-detail-routing.md), and the clearance
requirement that followed it is
[handover-detail-clearance.md](handover-detail-clearance.md) — **that
requirement is now met.**

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, based on `8ef300a`. **Nothing is committed.**
- Prototype: `/Users/michaelfeldmeier/Documents/GitHub/FridgeCAD` (`0c5d6d9`)

## What this step delivers

The **Detail** stage: every wire of the plan drawn cell by cell on the detail
grid, and no two of them closer than the design rule. The run directory fills
five of its seven artifacts:

```text
01-capacity.fb  02-global.fb  03-assign.fb  04-corridor.fb  05-detail.fb
```

`DetailRouting` carries the detail grid, one path per connection of the
assignment in its order, and one per connection of the global stage in its
order. A wire that was not drawn carries no cells, so a reader counts the
failures rather than being handed a list that can disagree with the paths beside
it.

## The model

Eight-connected A\* on the detail grid, costs `{10, 14}`, octile heuristic —
the prototype's own figures. The prototype's four passes, in its order:

1. **`detailed_pre_routing` — inside each partition.** A corridor cuts its wire
   into one piece per partition it names; every piece is searched for inside
   that partition alone, between the two crossings the plan gave it. The pieces
   of one partition are tried in up to `orderings` orders and the order that
   draws the most of them is kept. **This is the only pass that knows about
   partitions**, exactly as in the prototype: `astar_in_cell` takes the label
   grid, `astar_in_corridor` does not.
2. **`reconstruct_wires` — joined by concatenation.** The corridor already says
   which piece belongs to which wire, so there is no reconstruction pass and no
   cell is lost at either end.
3. **`detailed_cross_boundary_routing` — every wire drawn again, end to end.**
   Sweeps over the wire list alternating direction; a wire that has been
   re-routed is left alone. For a wire that has not, two phases: the band
   `corridor_spacings` wire spacings around the way it has, and then a
   relaxation that lets go of the wires ahead of it one place further along each
   time and then the wires behind it, steering with two prices instead of
   confining. A wire that finds nothing keeps the way it had.
4. **The inner circuit.** Its connections have no corridor, so they are seeded
   over whatever free space the ring has left — and from there they are wires
   like any other in pass 3.

**Everything before pass 3 is a seed.** The corridor stage's route says which
partitions a wire passes through and roughly where; it is not where the copper
has to run, and pass 3 is free to move a wire off it entirely.

### The one correction

**The clearance is held against every wire, not against two.** The prototype's
`astar_in_corridor` knows nothing about where the other wires are: what keeps
them apart is `build_corridor` alone, which cuts a disc of the wire spacing
around the ways of the two wires beside this one in the ring. Against the other
two hundred the only rule is that no cell is shared, which is one cell.

Cutting the disc for every wire per search would cost the whole chip's copper
per attempt, so it is not in the mask: the canvas carries a field of how many
wire cells lie within the rule of each cell, every wire charges its own when it
is put down and discharges it when it is taken off, and a search that must hold
the rule cannot enter a charged cell. Letting go of a wire is then exactly what
the prototype means by ripping it — its clearance stops standing in the way and
it is marked as a wire still to be drawn — with its copper left where it is, so
no two wires ever share a cell while the sweep is running.

**Two wires never share a cell**, and never cross between cells either: a
diagonal step past a corner both of whose cells belong to *one* wire is refused,
because that is exactly when two eight-connected paths cross without sharing a
cell. Two *different* wires there are two wires passing close, which is a
clearance question and not a short.

## What the figures say

At the shipped defaults, all eight chips:

| chip | connections | drawn | inner | cells | bends | longest | detail | SVG | GDS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4q  |  12 |  12 |   0/0 |    550 |   41 |  64 |  0.38 s | 0.17 MB | 0.07 MB |
| 9q  |  30 |  30 |   6/6 |   2644 |  116 |  98 |  0.09 s | 0.35 MB | 0.15 MB |
| 17q |  58 |  58 | 14/14 |   7891 |  553 | 202 |  0.33 s | 1.06 MB | 0.43 MB |
| 21q |  70 |  70 |   8/8 |  12454 |  717 | 221 |  0.60 s | 0.92 MB | 0.42 MB |
| 33q | 110 | 110 |   8/8 |  25916 | 1346 | 308 |  1.63 s | 1.73 MB | 0.69 MB |
| 45q | 150 | 150 |   7/7 |  42869 | 2744 | 396 |  4.58 s | 2.65 MB | 1.03 MB |
| 57q | 190 | 190 | 11/11 |  71230 | 4555 | 575 |  9.66 s | 4.34 MB | 1.57 MB |
| 69q | 230 | 230 | 11/11 | 111345 | 5478 | 700 | 15.50 s | 6.27 MB | 2.17 MB |

**All 968 connections are drawn, all 65 of the inner circuit, and no two wires
anywhere on any of the eight chips come within the design rule of each other.**
Every SVG is inside the 10 MB budget. Runtime is 0.09 to 15.5 seconds against
the prototype's own 0.08 to 46.9 for this stage. Two runs of the same input
produce the same bytes.

**314 of 314 ctest cases pass**, and 129 Python tests. The checks over every
chip: every wire is eight-connected and free of obstacles, does not meet itself,
no two share a cell or cross between cells, the first and last cell of a wire
are the point the assignment feeds it at and the cell of its target port, every
connection is drawn, and **no two wires come within the rule**.

## The rule, on a grid

`min_wire_spacing` is a length in layout units and the router works in cells, so
the rule is converted, and the conversion is the prototype's own: the rule spans
`ceil(spacing / cell)` cells and what is kept clear is one less. Two wires are
far enough apart when the distance between their cells is more than that many.

| chip | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cells | 9 | 4 | 9 | 4 | 4 | 5 | 5 | 5 |
| layout units | 173 | 158 | 171 | 160 | 154 | 182 | 158 | 180 |

The conversion is where the rule stops being 185, and it cannot be anything
else: a cell is 19 layout units across on the 17-qubit chip and 40 on the
9-qubit one, so where a wire runs is only known to half a cell in the first
place. The check is made against the converted figure, because a check the grid
cannot pass is a check of the grid and not of the copper.

## What each part is worth

Measured over the eight chips as places where the design rule does not hold:

| | fails |
| --- | ---: |
| as shipped | **0** |
| without letting go of the wires *behind* a wire as well as ahead of it | 3 |
| at the prototype's own 8 sweeps and 10 relaxations instead of 30 and 30 | 46 |
| holding the clearance to two ring neighbours, and to the plan's crossings | 457 |

And as connections left undrawn, over 17q, 45q and 69q:

| | undrawn |
| --- | ---: |
| as shipped | **0** |
| without the seed's rescue passes (`placeWhatIsLeft`, `drawWhatIsLeftWhole`) | 6 |

The obstacle penalty is on, as in the prototype. Measured with the clearance in
force it is neither better nor worse on any chip; what it buys is a wire that
keeps off the artwork where there is room to.

## Every figure is normalised to the grid

Nothing here is a cell count standing for a distance. A price is quoted per
**step**, against the ten a step along an axis costs; a reach is quoted as a
length or as a multiple of the wire spacing and converted through the grid.

- `corridor_spacings` is 4 wire spacings. The prototype's `K` is 40 *cells* —
  760 layout units on the 17-qubit grid and 1600 on the 9-qubit one, four
  spacings there and eight here.
- `obstacle_penalty_reach` is 185 layout units. The prototype's is 6 *cells* —
  114 units on the one chip and 240 on the other.
- `rounds` and `max_relaxation` are 30, where the prototype's are 8 and 10.

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

`docs/design/pipeline.md` has the stage's write-up and `CHANGELOG.md` is under
`[Unreleased]`.

## What is still open

1. **The corridor is a guide and no longer a constraint.** A wire may end up
   using a partition its corridor did not name, so the wire budgets the capacity
   stage counted are a plan rather than a bound. Whether that matters is a
   question for the capacity model.
2. **`[stages.capacity] crossing_pitch` is 165 and the wire spacing is 185.** It
   no longer blocks anything here, because the crossings are a seed; it is still
   a number that says a border carries more wires than the rule allows.
3. **There is no design-rule stage.** The clearance is checked in this stage's
   own test suite and drawn in its pictures. Phase 5's DRC has to make the same
   check on the finished geometry, where a wire is a polygon and not a run of
   cells.
4. **Nothing has been compared against the prototype's own output**, only
   against its formulation as read and its published counts.
