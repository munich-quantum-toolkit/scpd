# Routing

Two stages route a chip after the planning stages: the Detail stage draws every
wire cell by cell, and the Final stage draws the wires again on a finer grid
with turns of the bend radius, places the couplers, and routes the feedlines.
This page describes both stages as they stand, what each phase of the Final
stage does, how to read a run, and every key of their configuration.

## Overview

| Stage  | Artifact       | Reads                                                                                         | Writes                                                                                             |
| ------ | -------------- | --------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| detail | `05-detail.fb` | The partitions of the capacity plan, the inner connections, the assignment and the corridors  | One eight-connected path per connection of the assignment and per inner connection                 |
| final  | `06-final.fb`  | The inner connections, the assignment with its ring and feedline chains, and the detail paths | The wires, the inner wires, the feedline edges, the couplers, one snapshot per phase, the verdicts |

The Detail stage takes a corridor, which names the partitions a wire passes and
where it crosses from one to the next, and turns it into a path on the detail
grid. The corridor is a seed, not a bound: the crossings of a plan can lie
closer than the wire spacing, so the stage may move a wire off its corridor.

The Final stage takes every detail path as the seed of a search on the router
grid. Every capacity cell is cut into router cells no wider than
`[grid] router_cell_size`, which is about ten layout units on the benchmark
chips, so the wire spacing is nineteen cells there. The obstacle mask carries
`min_obstacle_spacing`, so every cell the search may enter already keeps the
obstacle clearance. Everything outside the ring of launcher slots is blocked.
Every length of the configuration is converted onto the grid once, at the start
of the stage; no step converts a figure a second time.

## Detail Routing

The stage runs an eight-connected A\* on the detail grid. A step along an axis
costs 10, a step along a diagonal 14, and the octile distance is the heuristic.
It draws every wire in five passes.

1. **Inside each partition.** A corridor cuts its wire into one piece per
   partition it names, and each piece is searched for inside that partition
   alone, between the two crossings the plan gave it. The pieces of one
   partition are tried in up to `orderings` orders, and the order that draws the
   most pieces is kept. Only this pass reads the partitions.
2. **Joined.** The pieces of a wire are joined in corridor order.
3. **The rescue.** A wire the pieces could not join is searched for once more
   from end to end over the free space. Where that finds nothing, the wires in
   its way are taken out and drawn again after it, one level deep.
4. **Across the borders.** Every wire is drawn again from end to end in up to
   `rounds` sweeps that alternate direction. A wire that holds the rule is left
   alone. A wire that does not is searched in a band `corridor_spacings` wire
   spacings to either side of the way it has, with every cell within the rule of
   another wire cut out. Where the band holds nothing, the relaxation lets go of
   the wires ahead of it in the sweep, one more each time, and then the wires
   behind it, up to `max_relaxation`. Letting go of a wire opens its clearance;
   its copper stays until it is drawn again. The relaxed search is priced rather
   than confined: cells outside the ring of the wire's own ends and its
   neighbours' ways cost more. A wire that finds no way keeps the way it had,
   and the next round asks again.
5. **The inner circuit.** The inner connections have no corridor. They are drawn
   last over the free space the ring leaves.

The clearance is `min_wire_spacing` converted onto the grid, and the stage holds
it against every wire, not only against the two beside a wire in the ring. The
grid counts how many wires guard each cell; a wire charges its room when it is
put down and gives it up when it is taken off, so a search cannot enter a cell
another wire guards. A wire's two ends keep their room while the wire is off the
grid. No two wires share a cell, and no two cross between cells. A step beside
an obstacle costs `obstacle_penalty` more, falling to nothing at
`obstacle_penalty_reach` layout units.

The stage reports, per round, how many wires are still to settle, and at the end
how many wires it drew, how many are unrouted and how many lie within the rule
of another. A fail is a wire without a way or a wire within the rule of another,
counted with every wire down. A wire the stage could not draw carries no cells
in the artifact.

## Final Routing

The Final stage runs five phases. Each phase changes what the phase before it
produced, so each one leaves its own snapshot in the artifact, and `plot` and
`render` draw the one they are asked for. `[run] stop_after` names the last
phase to run; the phases after it do not run, and the stage still counts what it
came to and writes its artifact.

| Phase                             | What runs                                                                                                                | What the snapshot holds                                                                |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------- |
| `pre-feedline-routing`            | The inner routing, the outer routing and the outer recovery                                                              | Every ring wire and every inner wire with its way and its rendered length; no couplers |
| `pre-feedline-refinement`         | The refinement of the outer routing                                                                                      | The same wires, moved into the room they have                                          |
| `feedline-insertion`              | The coupler insertion: the chain search with its step checks, the commit, the whole-chip check and the global SAT search | Every resonator cut back to its coupler, the coupler bodies and the feedline edges     |
| `feedline-constrained-routing`    | The bridges, the feedline routing, the inner routing under the feedlines, the targeted repair                            | Every wire drawn again under the feedline constraints                                  |
| `feedline-constrained-refinement` | The refinement under the feedline constraints                                                                            | The same, with every resonator at its target length                                    |

One driver runs every routing pass. A pass sweeps its wires in rounds that
alternate direction. For each wire it offers a way inside a band around the way
the wire has, lets go of the wires beside it one at a time when it finds none,
and puts them back when nothing came of it. A way counts as found only when it
holds the rule against every wire with every wire down; a way that exists only
because a neighbour was lifted leaves the wire open, and the next round tries
again.

**Pre-feedline routing.** The inner routing draws the wires of the inner circuit
inside the unit cells, in `inner_rounds` rounds and a band of
`inner_corridor_spacings` wire spacings. The outer routing draws the ring. Every
wire starts on its detail path and is searched in a band `corridor_spacings`
wire spacings to either side of it, fenced by its two ring neighbours. The
relaxation lets go of one neighbour after another along the sweep, up to
`max_relaxation`, and then against the sweep. It prices rather than forbids:
everything outside the lane between the two neighbours' ways costs more, and so
does the halo of every wire let go of. A plain wire keeps
`length_point_radius_cells` from the band where a resonator reaches its target
length, because the coupler insertion puts the pad there; the band is
`length_point_band_percent` of the target length to either side of that point. A
resonator's way is made `target_resonator_length` long: one rectangular loop is
spliced into a straight run of it, as deep as the missing length asks, as near
the qubit as it fits. A way without room for its loop is no way. The outer
recovery ends the phase with one more pass over the wires that still fail, with
the length-point bands off.

**Pre-feedline refinement.** Every ring wire is routed again in
`refinement_rounds` rounds with a price on the room it leaves, so that a wire
with room to spare moves into the middle of it. The band is
`refinement_corridor_spacings` wire spacings, `fence_pairs` ring neighbours to
either side are hard, and nothing is ripped up. A way is taken only when the
wire's conflicts do not rise.

**Feedline insertion.** One coupler per drawn resonator, and the feedline chains
through them. The next section describes it in depth.

**Feedline-constrained routing.** Every plain wire between two couplers is told
its bridge, the one feedline edge it crosses. The feedline routing sweeps the
ring with the edges of the chains as members, every wire again under the
feedline constraints, with the meander in the sweep. The inner routing runs
again under the feedlines. The targeted repair re-searches the coupler options
around the wires that still fail.

**Feedline-constrained refinement.** The refinement runs again under the
feedline constraints, in `feedline_refinement_rounds` rounds, and makes every
resonator its target length within `resonator_length_tolerance`. A way is taken
only when the wire's conflicts do not rise and it crosses no feedline. Four
checks run at the end of the insertion and again at the end of this phase: no
feedline within the rule of a coupler lead, no two edges that meet at a coupler
cross, no feedline crosses a resonator, and no two edges of different chains
cross.

## The Coupler Insertion

### A coupler and its options

A coupler is a pad of `coupler_length` by `coupler_height` layout units and a
lead. Two feedline ports sit at the ends of the far edge; the chain arrives at
one and leaves at the other, and the run along the pad between them is the
coupling. Two resonator ports sit at the ends of the near edge. The lead runs
from a resonator port through a quarter turn onto the coupler's orientation and
then straight for `min_straight_length` plus `coupler_lead_margin_cells` cells.
The tip of the lead lands on the resonator's way. The port a coupler creates
joins the port list of the chip, so the connection of the resonator is complete.

An option of a coupler is one orientation, one resonator port and one jog. The
orientations are the offsets of `coupler_offsets`, counted in eighth turns from
the heading of the nearest launcher. A jog adds a second quarter turn and a
straight run after the lead, which moves the pad sideways off the way. For each
option the insertion walks the cells of the resonator's way in order of how near
the place leaves the length `coupler_bias` times the target, and keeps the first
place that fits. A place is refused when the pad or the lead leaves the coupler
box less `coupler_box_margin_cells`, after a shift of up to
`coupler_shift_cells` into it; when it lies on artwork, on a placed pad or in a
port's approach; when the rest of the way runs through the pad or meets itself;
when the pad covers the tail of another resonator; or when the spliced length
falls more than `coupler_max_shortfall` of the target below it. The coupler box
is the rectangle the launcher slots span, less the launchers' straight runs plus
`launcher_lead`.

### The chains and the chain search

A chain comes from the assignment: a launcher, the couplers in ring order, and a
launcher or a reflector at the far end. One feedline edge runs between two
waypoints of a chain. Settling a chain means choosing one option per coupler so
that every edge has a way.

The chain search is a best-first search over **prefixes** of options. A node is
the options chosen for the first `k` waypoints. Its price is the real price of
the edges inside the prefix, each routed against the committed chip and the ways
the earlier edges of the same prefix have laid. The heuristic is the cheapest
run of lower bounds from the node to the end of the chain, where the bound of a
step is the least turning a way of the bend radius can make from one pose to
another with nothing in its way. The bound never exceeds a real way, so the
search returns the optimum. A step is routed when the search pops it, and a node
whose bound reaches the cheapest complete run found so far is dropped unpriced.

An edge costs its turns: one eighth turn costs 10000. An edge at a launcher is
weighted by `chain_end_edge_weight`, the bend price of an edge is
`edge_bend_factor` times the outer routing's, and `chain_length_weight` prices
each cell of length. The two edges at each end of a chain are routed again for
every prefix; the edges between are remembered by the pair of options at their
two ends. When the second edge of an end pair finds no way, the pair is laid the
other way round: this edge first, then its predecessor again against it. Each
chain is searched without the edges of the other chains.

The corridor of an edge closes the artwork, everything outside a box around the
edge's two ports, every coupler pad, every lead inflated by the clearance, every
resonator past its lead as an uncrossable line, the straight run of every
launcher but the edge's own plus `edge_launcher_margin_cells`, the chain's own
edges, and the coupler runs of the terminal edges of every chain settled before
plus `terminal_stub_extra_cells`. It opens a straight slot of
`terminal_slot_cells` at each of the edge's two ports, and nothing else.

### The step check

With `capacity_check` on, every step that routes its edge is followed by a check
of the chip as the step leaves it: the chains settled before on their options,
this chain's couplers up to the far end of the edge, and the prefix's edges up
to this one. A step whose edge loses a wire is refused, and the search takes the
next option. After `chain_rule_refusals` refused steps without a complete run,
the chain is searched again without the rule, because a chain left on its first
options is worse than one that loses a wire. The check judges only the wires of
the chain's own feedline; every terminal edge stays in the graph.

### The commit

The commit applies the chosen options: the pad becomes an obstacle, the lead and
the rest of the way become the resonator, and one wire per edge joins the wire
list in chain order with the way the search found. Where the second edge of an
end pair finds no way at the commit, the pair is laid the other way round, as in
the search.

### The capacity graph

The capacity check asks whether every outer wire still has a way through the
gaps the chip leaves. It builds a graph of the free space.

- **Walls.** The border, the artwork, the feedline edges inflated by half the
  clearance, the pads and the leads. A port's straight run is two thin walls
  with a slot between them, closed at the port and open at the far end.
- **Bottlenecks** come from **profile cuts**. The walls that stay through an
  insertion are built once, with a distance field of the walls inside the
  feedline ring and one of the walls outside it. The outline of every wall the
  insertion adds is walked, and the gap from each outline cell to each field and
  to the other added walls is a profile. A bottleneck is a persistent minimum of
  a profile: the gap rises by `bottleneck_rise` on both sides before it falls
  below. Only a gap of at most `bottleneck_wires` wires counts. A cut crosses
  free cells only and no slot. A port run's slot is closed by a cut from each
  flank to the nearest wall ahead, its **mouth**.
- **Chambers** are the pieces the walls and the cuts leave of the free space.
- **Stretches** are the cells of a feedline edge where a wire may cross it: the
  band `crossing_reach_cells` to either side is free, the crossing rule allows
  the cell, and the cell lies past the pin cells at the edge's ends. A stretch
  takes as many wires as it holds crossing points `crossing_spacing` clearances
  apart.
- **Mouths between chains** close the room between the last edge of one chain
  and the first edge of the next with lines between the tips of their terminal
  runs.
- **Gates.** A cut of `L` cells across the wires' heading takes
  `floor(L cos φ / c)` wires, with `c` the clearance and `φ` the angle to the
  nearest heading. From `capacity_margin_from_wires` wires on, the clearance
  counts `capacity_margin` times. A gate at a wire's own port run takes that
  wire half a clearance more; a gate narrower than `resonator_passage`
  clearances at a resonator's own run takes no resonator. A gate at a terminal
  edge takes every wire, and every wire may cross a terminal edge, because the
  feedline routing lays the terminal edges again.

Every outer wire whose two ends lie in chambers is a demand. A plain wire must
cross exactly the edges it bridges, each once, and no other. A resonator crosses
nothing and may run at most its target length less the run to the port and the
lead, plus the tolerance. A way may be up to `detour_factor` times the shortest.

### The judge

The demands are routed one after another in wire order, each on its way with the
fewest edges over the gates that still take a wire, so that every gate carries a
load. The **slot check** then gives the verdict. It replaces the stretches by
**slots**: a rectangle on a straight piece of a feedline edge, the crossing
reach to either side and one clearance along the edge, on free cells only,
spread evenly along the piece, none within `slot_pin_cells` of an end. A slot
takes one wire between the chambers on its two sides. A slot is placed only
where both of its ends have an **escape**: a turn of the router or a straight
run of `escape_straight_cells` that keeps `escape_stub_margin_cells` from the
walls of a port run. Each wire takes a shortest way over the slots and gates it
may use, and a negotiation raises the price of every slot and gate that is over
until the wires settle. The chip is **SAT** when every wire has a way and no
slot and no gate is over, **UNSAT** otherwise, with the lost wires and the gates
over named.

### The whole-chip check and the global SAT search

After the commit the chip is checked whole, with every chain standing and the
terminal edges as demands. A chip with an undrawn edge is UNSAT. When the check
says UNSAT, the global SAT search looks for another set of options.

1. The chains that own the gates over capacity are the **focus**, with the
   waypoints at both ends of each such edge marked.
2. Per chain, the segment search of the targeted repair yields up to
   `global_sat_runs` runs of options per segment. For a chain outside the focus,
   only the two end segments are searched and only states that move an end
   coupler are kept, up to `global_sat_states`. For a focus chain every segment
   with a marked waypoint is searched and every state is kept.
3. A state is **locally SAT** when either of two checks finds it SAT: with the
   other chains standing as the commit left them, or with the chain alone.
4. **Stage 1** tries every locally SAT state of a focus chain alone.
5. **Stage 2** tries combinations of states over several chains, the focus
   chains always among them, up to `global_sat_tries` combinations in all. Each
   combination starts from the commit, moves the couplers, draws every edge
   again and checks the chip whole.

The first SAT combination stays. Without one, the chip goes back to the commit.

## The Feedline Routing

### The crossing rule

A plain wire may cross a feedline edge only at a right angle, within
`crossing_reach_cells` of the edge's straight runs, outside the pin cells at the
edge's ends and outside the room around a bend. The search, the obstacle mask
and the count all read the same rule. A terminal edge is outside the rule: it is
a hard obstacle for every wire, and the clearance rule checks it. Each plain
wire between two couplers is told its bridge, the one edge it may cross; every
other edge is fenced, and a way that does not cross the wire's bridge is refused
as no way.

### The ring, the fences and the prices

The pass sweeps the ring of the outer routing with the edges of the chains as
members. A wire is fenced by `fence_pairs` ring neighbours to either side, hard
at the clearance, and by the fixed places of every wire: its lead, the run out
of its source and the run into its target. The coupler run of a terminal edge is
closed to every other wire's search plus `terminal_stub_extra_cells`.

The relaxation lets go of one neighbour after another along the sweep, up to
`max_relaxation`, and then against the sweep. The search is priced inside the
**lane**: the polygon from the wire's source along one neighbour's way to its
target and back along the other's. Everything outside the lane costs more. A
wire let go of keeps a **halo** of `halo_reach` clearances at a flat price, and
past it a price that rises to `rise_peak` times the flat price at `rise_reach`
clearances. A wire that fails every level in two rounds in a row gets one more
level that lets go of both neighbours at once.

### The meander

A resonator is lengthened by a meander spliced in between two straight cells of
its way: each end turns onto a heading across the pair, and between the two
turned ends a rectangular loop runs out, across and back, its run across at
least `meander_leg_spacing_cells` long. Where one loop would reach past the room
it may use, the meander takes three, five or more shallower loops side by side,
the fewest that fit. Of all placements that fit, the insertion takes the one
whose cells cost least in the search's price field, with a price of 100 per
direction change, as the prototype's strict insertion does. When the pairs a
hundred cells apart along the way leave nothing, every pair up to 150 cells
apart is tried as well. A loop along the axes comes first; only when none fits
does the loop run along the other diagonal of a diagonal way.

Before the couplers, a resonator's way is made long enough from where it enters
the coupler box onwards, since the coupler is spliced in there and whatever lies
before it is cut off; the meander stays inside the coupler box and behind that
cell. The coupler insertion then keeps the feedline edges out of the box of
every such meander, grown by the wire clearance and cut down to the coupler box,
except around the two ends of an edge. The coupler itself leaves `coupler_bias`
times the target to run to the qubit, a little below the target, because the
meander can lengthen a way and never shorten it.

Under the feedline constraints the sweep holds a resonator to within
`sweep_length_band` times `resonator_length_tolerance` of the target, and the
refinement to the tolerance itself. The first `meander_start_margin_cells`
straight cells of the way, by default the straight run of the lead off the
coupler, are left alone. A loop that adds more than the missing length is taken
when the length it lands on lies within the tolerance. When the way a resonator
finds has no room for a meander, it keeps the way it had if that way holds every
rule, or takes a meander into that way when only its length is wrong.

A resonator that reaches its qubit through a bridge carries its length on the
inner wire beyond the bridge: that wire gets the meander and the target length,
and the wire from the coupler to the bridge only feeds it.

### The targeted repair

The repair runs after the sweep on the wires that still fail.

1. **The triage.** Every unrouted, open or crossing wire blames the edges and
   couplers within reach of its conflict. Per chain, the runs of blamed
   waypoints become segments, widened by one free coupler at each end.
2. **The re-search.** The chain search runs on a segment with both ends fixed
   and the couplers between them open, the rest of the chip fenced. Every
   complete run of options it reaches is offered in order of cost.
3. **The legality test.** The couplers take the run's options and the edges the
   search's ways. A window of ring wires is drawn again in `research_rounds`
   rounds: the wires that bridge a segment edge, the resonators of the segment,
   the wires the fails named, every wire whose way lies within the clearance of
   a new edge, and `research_ring_members` ring members to either side of each.
   The run is kept when no wire of the window is unrouted, open or crossing and
   the bad wires of the chip fall. Otherwise the chip is put back.

The budget is `repair_trials` legality tests per chip; zero turns the repair
off. After an accepted run the triage is built again on the new state.

## Reading a Run

The Final stage ends every pass on one line and the stage on one over every
wire. The line counts the wires drawn and the wires with each verdict, with
every wire down. A conflict between two wires counts both wires.

| Verdict    | Meaning                                                                                  |
| ---------- | ---------------------------------------------------------------------------------------- |
| `unrouted` | The wire has no way of its own: it is still on its seed or not on the grid               |
| `open`     | Its way comes within the wire clearance of another wire                                  |
| `crossing` | It crosses a feedline edge other than at a right angle, or enters the room around a bend |
| `short`    | A resonator shorter than `target_resonator_length` less the tolerance                    |
| `long`     | A resonator longer than `target_resonator_length` plus the tolerance                     |

The line also names a wire whose way meets itself; such a wire is carried in the
artifact and named in its tooltip. `Bad` is `unrouted + open + crossing`: the
wires with no usable way. `Fails` is `short + long`: the resonators off their
length. `0 bad wires` means every connection and every feedline edge has a way
that holds the clearance and the crossing rule. `SAT` on the whole-chip check
means every outer wire has a way through the capacity graph with no slot and no
gate over capacity. The two are different claims: the graph is a necessary
condition, and the routing is the result.

A benchmark run is accepted when every connection and every feedline edge is
drawn, `Bad` is 0, `Fails` is 0, the whole-chip check is SAT, and `mqt-scpd drc`
reports no active finding. `Bad` counts wires and the check counts findings, so
a pair of wires too close is two bad wires and one finding.

The end state of `06-final.fb` carries the verdict of every wire.
`plot --stage final` draws a wire with an unrouted, open or crossing bit in red
and a resonator off its length in amber, and names the verdicts in the tooltip;
`render` writes them on layers of their own. `--phase <name>` draws the snapshot
of one phase; the snapshots carry no verdict, because they are not judged.

## Configuration

Every key below is optional. Every length is in layout units unless the key says
`cells`; a cell is a router cell for `[stages.final]` and a detail cell for
`[stages.detail]`. The wire clearance of both stages is `min_wire_spacing`
converted onto the grid, and no key sets it.

### `[stages.detail]`

| Key                      | Default | Meaning                                                                                                                 |
| ------------------------ | ------- | ----------------------------------------------------------------------------------------------------------------------- |
| `router`                 | `""`    | The implementation of the stage; empty takes `pixel-astar`                                                              |
| `corridor_spacings`      | `4`     | How far to either side of its own way a wire may move when it is drawn again across the borders, in wire spacings       |
| `rounds`                 | `30`    | The most sweeps over the wires, in turn forward and backward                                                            |
| `max_relaxation`         | `30`    | How many neighbouring wires a wire without a way may let go of, along the sweep first and then against it               |
| `obstacle_penalty_reach` | `185.0` | How far the price of running beside an obstacle reaches; `0.0` turns the price off                                      |
| `obstacle_penalty`       | `10`    | What a step right beside an obstacle costs, against the 10 a step along an axis costs; it falls to nothing at the reach |
| `orderings`              | `16`    | How many orders the pieces of one partition are tried in; `1` keeps the given order                                     |

### `[stages.final]`

The routing passes:

| Key                             | Default   | Meaning                                                                                                              |
| ------------------------------- | --------- | -------------------------------------------------------------------------------------------------------------------- |
| `router`                        | `""`      | The implementation of the stage; empty takes `dubins`                                                                |
| `corridor_spacings`             | `11`      | The band to either side of its way a ring wire may move in, in wire spacings                                         |
| `inner_corridor_spacings`       | `11`      | The same for the inner circuit                                                                                       |
| `refinement_corridor_spacings`  | `6`       | The band of both refinements, in wire spacings; `0` takes `corridor_spacings`                                        |
| `rounds`                        | `30`      | The most sweeps of the outer routing and the feedline routing                                                        |
| `inner_rounds`                  | `4`       | The most sweeps of the inner routing                                                                                 |
| `max_relaxation`                | `10`      | How many neighbouring wires a wire without a way may let go of                                                       |
| `refinement_rounds`             | `2`       | The rounds of the pre-feedline refinement; `0` skips it                                                              |
| `feedline_refinement_rounds`    | `2`       | The rounds of the feedline-constrained refinement; `0` skips it                                                      |
| `bend_penalty_norm`             | `2.5`     | What one eighth turn costs, in hundredths of a cell per unit of the grid's width plus height                         |
| `wire_proximity_penalty_norm`   | `0.00125` | What running beside another wire costs, on the same scale                                                            |
| `static_proximity_penalty_norm` | `0.00033` | What running beside an obstacle costs, on the same scale                                                             |
| `obstacle_penalty_reach`        | `100.0`   | How far the price of running beside an obstacle reaches; `0.0` turns the price off                                   |
| `fence_pairs`                   | `3`       | How many ring neighbours to either side a wire is fenced against in the feedline routing and the refinements         |
| `halo_reach`                    | `1`       | The halo of a wire let go of, in clearances                                                                          |
| `rise_reach`                    | `3`       | Where the rising price past the halo reaches its peak, in clearances                                                 |
| `rise_peak`                     | `3`       | The peak of the rising price, as a factor on the flat price                                                          |
| `length_point_radius_cells`     | `40`      | What a wire keeps from the band where a resonator reaches its target length, in cells; `0` turns it off              |
| `length_point_band_percent`     | `10.0`    | The half-width of that band, as a percentage of the target length                                                    |
| `sweep_length_band`             | `1.0`     | The band of lengths the feedline sweep accepts, as a multiple of `resonator_length_tolerance`                        |
| `meander_leg_spacing_cells`     | `25`      | What the two legs of a meander loop keep between them, in cells                                                      |
| `meander_start_margin_cells`    | `-1`      | How many straight cells at the start of a resonator the meander leaves alone; negative takes the lead's straight run |

The couplers and the chain search:

| Key                          | Default | Meaning                                                                                                                                                |
| ---------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `coupler_length`             | `200.0` | The run of the pad the resonator couples along                                                                                                         |
| `coupler_height`             | `26.0`  | How far the feedline lies from the resonator across the pad                                                                                            |
| `coupler_offsets`            | `""`    | The orientations of an option, as a comma list of eighth turns `0` to `7` off the nearest launcher's heading; empty takes every offset but `3` and `4` |
| `coupler_bias`               | `0.98`  | Where the coupler cuts the way, as the share of the target length left to run from the coupler to the qubit                                            |
| `coupler_max_shortfall`      | `0.30`  | How far below the target length a coupler may leave the resonator, as a share of it                                                                    |
| `coupler_box_margin_cells`   | `5`     | What a coupler keeps off every side of the coupler box, in cells                                                                                       |
| `coupler_shift_cells`        | `20`    | How far an option that leaves the box is moved into it before the place is given up, in cells                                                          |
| `coupler_lead_margin_cells`  | `5`     | The straight run of the lead beyond `min_straight_length`, in cells                                                                                    |
| `launcher_lead`              | `10.0`  | How much further than its straight run a launcher keeps the coupler box away                                                                           |
| `edge_launcher_margin_cells` | `6`     | What a feedline edge keeps closed in front of every launcher but its own, beyond the launcher's run, in cells                                          |
| `terminal_slot_cells`        | `20`    | The straight slot a feedline edge keeps open at a port, in cells                                                                                       |
| `terminal_stub_extra_cells`  | `5`     | How far the coupler run of a terminal edge is guarded past the pad, in cells                                                                           |
| `chain_end_edge_weight`      | `0.1`   | The weight of an edge at a launcher in the chain search, against `1` for every other edge                                                              |
| `edge_bend_factor`           | `1.0`   | The bend price of a feedline edge, as a factor on `bend_penalty_norm`                                                                                  |
| `chain_length_weight`        | `0`     | The price of one cell of an edge's length in the chain search; `0` prices length not at all                                                            |
| `repair_trials`              | `100`   | How many legality tests the targeted repair may run per chip; `0` turns the repair off                                                                 |
| `research_ring_members`      | `2`     | How many ring members beside each window wire the repair draws again                                                                                   |
| `research_rounds`            | `2`     | The rounds of the repair's local pass                                                                                                                  |

The capacity check:

| Key                          | Default  | Meaning                                                                                                                               |
| ---------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `capacity_check`             | `true`   | Whether the insertion runs the step checks, the whole-chip check and the global SAT search; off, the chains are settled by cost alone |
| `chain_rule_refusals`        | `1500`   | How many steps the capacity rule may refuse before a run of options joins a chain; then the chain is searched again without the rule  |
| `bottleneck_wires`           | `10`     | The widest gap, in wires, that is still a bottleneck                                                                                  |
| `bottleneck_rise`            | `2.0`    | The least rise of a gap profile on both sides of a minimum, in cells                                                                  |
| `crossing_reach_cells`       | `10`     | The band beside a feedline edge where a wire may cross only at a right angle, in cells                                                |
| `crossing_spacing`           | `1.414…` | The spacing of the crossing points of a stretch, in clearances                                                                        |
| `slot_pin_cells`             | `10`     | The cells at each end of an edge where no slot lies                                                                                   |
| `escape_straight_cells`      | `5`      | The straight run a slot's escape may take, in cells                                                                                   |
| `escape_stub_margin_cells`   | `4`      | What an escape keeps from the walls of a port run, in cells                                                                           |
| `capacity_margin`            | `1.06`   | The factor on the clearance a gate counts with, from `capacity_margin_from_wires` wires on                                            |
| `capacity_margin_from_wires` | `3`      | The fewest wires the factor applies to                                                                                                |
| `resonator_passage`          | `0.8`    | The narrowest gate a resonator passes at its own port run, in clearances                                                              |
| `detour_factor`              | `1.5`    | How much longer than its shortest way a demand may run through the graph                                                              |
| `room_margin_cells`          | `10`     | The dead length at both ends of a straight run in the room report, in cells                                                           |
| `room_channel_reach_cells`   | `80`     | The reach of the channel rule in the room report, in cells                                                                            |
| `global_sat_runs`            | `8`      | The runs of options the global SAT search collects per segment                                                                        |
| `global_sat_states`          | `8`      | The locally SAT states it keeps per chain outside the focus                                                                           |
| `global_sat_tries`           | `200`    | The whole-chip combinations it tries                                                                                                  |

## Debug Pages

A run writes debug pages when its `config.toml` has a `[debug]` section with a
page turned on, or when `mqt-scpd plan` runs with `-d`, which turns every page
on. The Final stage then records what the pages need, one JSON record at a time,
and the run keeps the records under `<run>/debug/data/<kind>.jsonl.gz`. The
pages are built from the records when the stage ends and link from
`<run>/debug/index.html`; `mqt-scpd debug <run>` builds them again from the
records, and `mqt-scpd debug <run> --serve` runs the stage to the end of its
coupler insertion and serves the capacity graph on a local page whose tab moves
a coupler to another option. The core writes no file.

| Key                                         | Page                                                                                                                                               |
| ------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `html`                                      | `index.html`: the links, the lines the run said and what every phase came to                                                                       |
| `pre_feedline_routing_dashboard`            | A table of the pre-feedline routing: a row per wire, a column per round and relaxation level; a click draws the way the search found over the grid |
| `pre_feedline_refinement_dashboard`         | The same table for the pre-feedline refinement                                                                                                     |
| `feedline_insertion`                        | The coupler options and the capacity graph of the whole-chip check, on the page the graph is laid out on                                           |
| `local_sat`                                 | Every step check of the chain search: the chain, the step, the options, the verdict and the wires it closes                                        |
| `global_sat`                                | The global SAT search: the states collected per chain, every combination with its verdict, and the result                                          |
| `feedline_constrained_routing_dashboard`    | The table for the feedline-constrained routing                                                                                                     |
| `feedline_constrained_refinement_dashboard` | The table for the feedline-constrained refinement                                                                                                  |

Every key is off by default. A run records only the kinds the pages it writes
need: `grid` and `search` for a dashboard, `coupler-options` and
`capacity-graph` for the insertion page, `step-check`, `global-sat` and `phase`.
The records are plain JSON, one per line, so a script can read them without the
pages.

## SAT Benchmarks

Two benchmarks measure how well the capacity check predicts the routing. The
**local** benchmark takes a feedline, applies a run of options to its segment,
and compares the slot check of that feedline alone with a rip-up and re-route of
its wires alone. The **global** benchmark takes a whole-chip state and compares
the whole-chip check with the feedline routing of the chip. Each case falls into
one of four classes:

| Class                | The check says | The routing ends |
| -------------------- | -------------- | ---------------- |
| SAT and clean        | SAT            | with no bad wire |
| SAT but failing      | SAT            | with a bad wire  |
| UNSAT but clean      | UNSAT          | with no bad wire |
| UNSAT and failing    | UNSAT          | with a bad wire  |

The agreement is `(SAT and clean + UNSAT and failing) / all`. A case in the
second class is the check too lax; a case in the third is the check too strict.
The benchmarks live in `test/python/benchmarks/` and run with
`MQT_SCPD_SAT_BENCHMARKS=1 uvx nox -s sat_benchmarks`. They take hours on the
large chips, so the ordinary test suite leaves them out.
