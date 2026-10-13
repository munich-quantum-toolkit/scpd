# Plan: the capacity check of a chain-search step from the step before it

Built on 2026-10-09 as planned; the state of the code and the measurements
are in handover-capacity-check.md, section 9.

Written for the user and for whoever takes the capacity check further. The
task is `prompt-capacity-step-runtime.md`: make the check that follows every
step of the chain search (`SCPD_CAPACITY_RULE`) cheap enough to run on every
chip every day, with the same verdict.

- Checkout `scpd-phase-4`, branch `phase-4-routing-stages`, HEAD `f132d5e`.
  Everything is uncommitted; the user commits per phase.
- The measured state before this work is in
  handover-cpw-coupler-insertion.md, *What it costs* and *Measured on every
  chip (2026-10-09)*: one check is one whole analysis of the chip, 0.1 s on
  17q and about 2 s on 69q, and a chain checks hundreds of steps.

## What the check depends on

A check analyses the chip of a step: the chains settled before, and the
prefix of the chain being searched — its couplers up to the far end of the
edge being priced, their pads and leads, and its edges up to this one. The
chip of a step **differs from the chip of the step before it by additions
only**: one coupler's pad and lead, and one feedline edge. (When the step
lays the pair before it the other way round, the chip differs from the one
two steps back by two edges and two couplers, still additions only.)

Each stage of the analysis is a function of the stage before it:

| Stage                       | Reads                                       | Where it can change after an addition           |
| --------------------------- | ------------------------------------------- | ----------------------------------------------- |
| walls (mask, owner, slots)  | the state                                   | the cells the new walls take                    |
| distance transform          | mask                                        | the cells whose nearest wall is now a new one   |
| medial axis (Voronoi edges) | the boundary cells                          | the edges that pass where the distance changed  |
| axis raster and adjacency   | the edges, mask                             | the cells of the edges that changed             |
| cuts (saddles, forks)       | axis, distance, slots                       | every candidate whose walks read a changed cell |
| chambers                    | mask, cuts                                  | the chambers that touch a changed cell or cut   |
| stretches, ports, demands   | walls, chambers, wires                      | everywhere; cheap (8 ms on 69q)                 |
| the check in turn           | the graph                                   | cheap (0.1 ms)                                  |

## The design

The check keeps a **working state** of the whole analysis — walls, distance,
axis, cut records, chambers — for the chip of one step, and a **patch** per
step that was not refused: what the step changed, with the old values, so
the state can be moved to the step's parent and back. A refused step leaves
no patch. The state moves between steps by undoing and applying patches along
the search tree, so a check costs the work of its own additions and nothing
of the chip around them.

Each stage is updated exactly:

1. **Walls.** `wallsOf` is drawn once more inside the window around the new
   walls and compared cell by cell with the working walls; walls are matched
   by name. The patch holds the cells that differ.
2. **Distance.** The distance transform of the new wall cells alone, in a
   window, taken as the minimum with the old field. The set of cells whose
   distance fell is star-shaped towards the new walls, so when no cell on the
   window's border fell, no cell outside it did; the window grows until that
   holds. Exact.
3. **Medial axis.** The Voronoi diagram changes only where the distance fell
   (region A). The edges of the diagram are kept with their two site cells.
   The edges that pass A are removed; the diagram of the sites in a window
   around A (A widened by the largest clearance in it) is built, and its
   edges that pass A are added. An added edge is exact when the clearance
   disc of each of its two vertices lies inside the window; when one does
   not, the window grows. The raster and the adjacency are kept with counts
   per cell and per pair, so removing an edge's cells and adding another's
   gives the raster `rasterizeMedialAxis` would draw.
4. **Cuts.** `findBottlenecks` is split into its units: the saddle test of
   a cell with its rise walks, the cut from a saddle (shores, descents,
   line), and the fork rule of a junction. Each unit records the box of the
   cells it read. After a change, every unit whose box meets the box of the
   changed cells is run again; the others cannot have changed. The global
   steps — the order, the pair dedupe, `narrowestOfEveryPlateau`,
   `couplerCuts` — run on the whole list, which is small.
5. **Chambers.** A chamber touched by a change is flooded again from the
   cells beside the change, all pieces at once, one step each in turn; a
   piece that runs out of cells is split off and gets a new label, and when
   one piece per old chamber is left running, it is the rest of that chamber
   and keeps its label unexplored. So a wall that cuts a small pocket off a
   chamber of a million cells costs the pocket. A removed cut joins two
   chambers; the piece that ran out is relabelled to the other.
6. **Graph and check.** As before, on the working walls and chambers.

Switches: `SCPD_CAPACITY_INCREMENTAL` (on; 0 analyses the whole chip at every
step as before) and `SCPD_CAPACITY_VERIFY` (off; on runs both at every step
and logs where cuts, chambers or the verdict differ).

## What is measured

As in `prompt-capacity-step-runtime.md`: every chip with the rule, the time
per check before and after, chains settled, fallbacks, the graph's verdict,
`bad`, the insertion's and the run's time; and, under the verify switch, how
often the step-wise state and the whole analysis disagree.
