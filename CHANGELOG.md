<!-- Entries in each category are sorted by merge time, with the latest PRs appearing first. -->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on a mixture of [Keep a Changelog] and [Common Changelog].
This project adheres to [Semantic Versioning], with the exception that minor
releases may include breaking changes.

## [Unreleased]

### Added

- ✨ Move a coupler option that leaves the coupler box into it, by at most
  `SCPD_COUPLER_SHIFT` cells along each axis (20 by default; `=0` places every
  option on its place). The option is moved by what its cell outside the box
  lacks and built again until it fits. The resonator runs from the moved tip
  of the lead in a straight line to the first cell of its way that lies at
  least a bend radius in front of the tip; that line must stay off the
  artwork, the ports' approaches and the coupler's own feedline run. On 4q all
  four couplers now stand on offset 0 near their target length, and the chain
  makes four quarter turns (angle cost 8 against 18); the other benchmarks
  keep their angle cost.
- ✨ Find the bottlenecks of the chip the coupler insertion leaves, build the
  capacity graph they make and check whether it carries every outer wire, report
  only (`SCPD_BOTTLENECKS`). The medial axis is built over the border, the
  artwork, the coupler pads, the feedline edges and the port runs of every outer
  wire, a port run being the rule's straight length from a feed point or a
  target port, or a resonator's lead as it stands; feedline edges are inflated
  by half the clearance. A port run is two walls one cell thick, the edge of the
  band of half the clearance to either side of it with square ends, open at the
  terminal's end, and the free cells between them are the slot where its
  terminal lies: a fork of the medial axis with an arm into a slot ends a
  narrowing (`BottleneckOptions::slots`), no cut crosses a slot, two cuts are
  one narrowing only between the same walls (`BottleneckOptions::wallOf`), and
  the cuts from the ends of the two walls close the terminal's own chamber,
  which its wire starts or ends in; every other arm of the fork in front of a
  slot is cut at its narrowest cell up to the next fork unless the search
  already cuts it, and two cuts between the same two walls with ends near each
  other are one narrowing. Every cell of a stretch at one clearance now passes
  the minimum rise and is taken to one middle, so a stretch that ends at a slot
  keeps its cut whichever way the grid numbers it. `findBottlenecks` traces each
  saddle of the axis to the two obstacles across it, and a bottleneck holds the
  length between the walls over the wire clearance (`wiresThroughGap`). The
  bottlenecks cut the free space into chambers (`grid::chambersOf`, with
  `bottleneckMoves` moved out of the capacity stage), and a plain wire crosses
  only the edge between two couplers it is prescribed, on the stretches where
  `CrossingConstraints` lets a wire through at a right angle, as many as fit a
  wire pitch apart; no wire crosses a terminal edge and no resonator any edge.
  Only the bottlenecks with an end on a feedline edge, a coupler pad or a
  resonator's lead cut the chambers (`SCPD_BOTTLENECK_ALL=1` keeps every one).
  `checkInTurn` routes the wires one after another in wire order, each from the
  chambers of its source port to those of its target port over the edges that
  still take a wire; a plain wire crosses exactly the feedline edges it is
  prescribed, each once, and a wire with no such way takes its shortest way and
  overfills the edges on it. The log says `CAPACITY GRAPH SAT` when no edge
  carries more wires than it takes and `UNSAT` with each overfilled edge and the
  wires through it. `SCPD_CAPACITY_FLOW=1` routes every wire at once instead
  (`checkCapacity`), as an integer multi-commodity flow with the least overflow
  (`SCPD_CAPACITY_SECONDS`), which also says `UNKNOWN` and names each edge the
  graph is short on with the wires there. `SCPD_CAPACITY_CHAIN=1` checks the
  graph after every chain the coupler insertion settles, against the chains
  before it, and searches a chain that closes a wire's way again
  (`SCPD_CAPACITY_CHAIN_TRIES`), refusing a run of options that closes as many
  or fails the commit's test on more edges. With `SCPD_CAPACITY_RULE=1`
  (off by default) every step of the chain search is checked on the chains
  settled before with the prefix standing on top, and a step whose edge
  closes a wire is refused.
  A resonator's way through the graph is the one whose least length — from
  line to line of the edges it passes — is shortest, and it is no way when
  that is longer than the resonator's target length less its lead allows;
  a chain no run of options joins under the rule is searched once more
  without it, and the time of the checks does not count against the chain's
  clock. `SCPD_CAPACITY_STEP=1` reports the check without the rule: each step
  says at `-v 1` whether the graph carries every wire and which wires its
  edge closes, and each feedline edge of a settled chain says it once. `mqt-scpd couplers
  <run>` holds the Final stage after the coupler insertion (`CouplerSession`)
  and serves the page on a local port with a Couplers tab: every option of a
  coupler is drawn
  on the chip, and a click puts the coupler on one, draws its two feedline
  edges again and shows the capacity graph of the chip as it then stands. The
  bindings name the global routing argument `global_`, as the stubs do.
  `SCPD_EDGE_LAUNCHER_MARGIN` (cells, 5) lengthens the run a feedline edge's
  search keeps closed in front of every launcher but its own, so that the edges
  of the coupler insertion and of the repair pass the launcher stubs further
  off; nothing else sees it. `SCPD_COUPLER_BOX_MARGIN` (cells, 5) holds every
  coupler's pad, its feedline runs, its lead and the turn after it that much
  further in from each side of the coupler box; the edges between couplers keep
  the whole box. `-d` draws
  `final-bottlenecks.svg` and `final-capacity-graph.svg`
  (`SCPD_BOTTLENECK_WIRES`, `SCPD_BOTTLENECK_RISE`) and writes the graph as
  `final-capacity-graph.json`, which `plan` lays out on the page
  `final-capacity-graph.html` (`mqt.scpd.capacityview`): the chip and an
  abstract view of the graph side by side, every edge with its load and
  capacity, the target and source ports, the way the flow sends each wire, and
  tables of the edges, the feedline edges and the wires;
  `SCPD_SEARCH_PICTURES=0` keeps a debug run to the pictures of the whole chip.
  `BottleneckOptions::minimumRise` drops the minima that the steps of a raster
  wall make and keeps one cut of a stretch of even width.
- ✨ Build the fifth phase of the Final stage, the refinement of the
  feedline routing, and make every resonator its target length in it. The
  phase draws every wire of the ring once more with a price on the room it
  leaves, fences three pairs of ring neighbours, and keeps the way a wire
  had when the new one is worse; a resonator is lengthened by the meander
  there, under `SCPD_REFINE_MEANDER` (on) rather than the sweep's
  `SCPD_FEEDLINE_MEANDER`, because the two passes ask different questions.
  Where a meander finds no room the search is made once more with the room
  price taken off and the hard corridor standing, for a resonator that has
  never reached its length — the prototype's own fallback
  (`SCPD_REFINE_FALLBACK`, measured to buy nothing and kept only because it
  is the prototype's). Three pairs of ring neighbours rather than one is
  the one figure here with a measured case for it: with one pair the
  69-qubit benchmark ends with eight resonators off their length instead of
  one. The room price is built after the feedline
  constraints, so the chamfer to the middle of a channel sees the walls of
  that channel (`SCPD_REFINE_PRICE_LAST`), and the four checks of the
  coupler insertion are said again at the end of the phase, because the
  refinement moves feedline edges and nothing looked at the result
  (`SCPD_REFINE_CHECKS`). **The phase is the default**:
  `feedline_refinement_rounds` is 2, `SCPD_FEEDLINE_MEANDER` is on, and the
  eight benchmark configs run to `stop_after = "refined"`. Over those eight
  the stage ends at `bad` 2 instead of 8 and 6 resonators off their target
  length instead of 102, no long anywhere and five chips clean on both
  figures, for about twice the runtime.
- ✨ Make a resonator its length in the feedline sweep as well, within a
  band five times `resonator_length_tolerance` wide
  (`SCPD_SWEEP_LENGTH_BAND`), and leave what is left to the refinement:
  the prototype aims roughly in its sweep (`meander_insertion_proximity`)
  and exactly in its refinement (`meander_insertion_proximity_strict`). The
  figure the stage is judged by is unchanged — short and long are counted
  against the tolerance itself.
- ✨ Say why a meander found no room rather than only how many placements
  were tried: `MeanderResult::Refusals` counts each refusal by its reason —
  the ends not facing each other, no pair far enough apart for the legs,
  no depth, no room beside the way, a closed cell, a self-crossing, a
  rendering off the tolerance — and the stage's line names them with the
  budget the pairs were drawn from.
- ✨ Mark the resonators the Final stage left off their target length in
  the picture, on a layer of their own under the failing one:
  `planning.off_length` from the `Short` and `Long` verdicts, amber and
  dashed in `plot --stage final`, layer 31 `final.off-length` in `render`.
- 🔧 Make the place the coupler cuts a resonator's way a switch,
  `SCPD_COUPLER_BIAS` (1.0): a way the coupler leaves longer than the
  target is a `long` verdict nothing in the stage can mend, and cutting at
  less than the target hands the difference to the meander.
- ✨ Close the run a terminal feedline edge has to make at its coupler to
  every other wire: the run into the first coupler of a chain and the run
  out of its last, five cells beyond it, inflated by the wire clearance, in
  every search of the feedline pass but the edge's own, and in the coupler
  insertion for every chain already settled. The launcher end was walled by
  the port band already; the coupler end was open to every wire that is not
  a ring neighbour of the edge, and a wire drawn across it left the edge no
  way. `SCPD_TERMINAL_STUB_GUARD` (on) and `SCPD_TERMINAL_STUB_EXTRA` (5).
  On the 69-qubit benchmark the feedline routing ends with 10 failing wires
  instead of 14, and the two terminal edges that ended open now hold.
- ✨ Fence three pairs of ring neighbours in the feedline pass rather than
  the prototype's two, in phase 1 and at every relaxation level, the wires
  let go of never among them: `SCPD_FEEDLINE_FENCE_PAIRS` (3). With the
  terminal runs closed the 69-qubit benchmark ends with 5 failing wires
  instead of 10, and the pass is a third faster.
- ✨ Measure, at the end of the coupler insertion, the room every feedline
  edge leaves beside it for the wires that have to pass between it and the
  nearest qubit or tunable coupler, and mark the edges that leave too
  little: `FinalWire.verdict` carries `Squeezed`, `note` the figures and
  `marks` the line measured; `plot --stage final --phase couplers` draws
  them in magenta with the line and the note, `render` on layer 30
  `final.squeezed` (`SCPD_SQUEEZE_REPORT`, `SCPD_SQUEEZE_STEP`,
  `SCPD_SQUEEZE_REACH`). On the 17- and 69-qubit benchmarks every marked
  edge carries a failing wire of the feedline routing and every open wire
  lies on a marked edge.
- ⚡️ Tell every edge search of the chain A* how many turns its way may
  still make and leave its prefix able to beat the cheapest complete run
  priced so far, and drop a prefix whose bound already reaches that run
  without pricing it (`SCPD_CHAIN_STEP_BUDGET`; `DubinsRouter::setMaxTurns`).
  Exact: the same runs of options chain for chain, a tenth off the
  17-qubit insertion.
- 🔧 Price a wire let go of in a relaxation of the feedline pass within one
  wire clearance rather than three, `SCPD_HALO_REACH` 1: the prototype's
  own disc. On the 69-qubit benchmark one more pair of wires closes.
- ✨ Carry the Final stage's verdict against every wire in its artifact and
  mark the failing wires in the picture. `FinalWire.verdict` holds the bits
  of the new `FinalVerdict` — unrouted, open, crossing, short, long, meeting
  itself — as the stage's own count sets them on the end state; the phase
  snapshots are not judged and carry none. `mqt-scpd plot --stage final`
  draws every unrouted, open or crossing wire in red over the picture, names
  it and its verdicts in a tooltip and counts them in the legend, and
  `render` writes them on a layer of their own (`final.failing`). The two
  length verdicts are carried and named but do not mark a wire.

- ✨ Insert a CPW coupler on every resonator of the Final stage and route
  the feedline chains through them, the prototype's
  `run_optimized_cpw_coupler_insertion`. A resonator's way is cut where the
  way left to the qubit port is `target_resonator_length`, and the coupler
  sits there: the way runs the coupling length straight from the anchor,
  turns a quarter and joins what is left; the body spans that run and the
  feedline runs along its far edge, along the ring. Every orientation,
  mirrored or not, with the feedline either way along the body and with
  or without a second dogleg is an option, and a greedy local search per
  chain takes for each coupler the option whose two feedline edges to its
  chain neighbours turn least; an edge already chosen or drawn stands in
  the way of every edge routed after it, the edge into a coupler and
  the edge out of it never cross, and every edge keeps the wire clearance
  from every resonator but its own coupler's. The chains come from the assignment, which
  now carries them (`Assignment.chains`); the coupler carries the
  `ResonatorSource` port the assignment left absent, and the artifact
  carries the edges (`FinalRouting.feedlines`, `feedline_edges`) and the
  couplers, from the `couplers` phase on ([#118]) ([**@FeldmeierMichael**])
- ✨ Route every wire again under the feedline constraints, the prototype's
  feedline routing: the edges of the chains are fenced, a wire crosses an
  edge between two couplers at a right angle only and a conventional wire
  crosses the one edge that spans its launcher, once; the edges at the
  launchers are hard for every wire. A resonator is drawn again from its
  coupler and made `target_resonator_length` exactly, within
  `resonator_length_tolerance`, by the strict meander insertion
  (`MeanderOptions.exact`). A wire whose way already holds every rule is
  settled without a search ([#118]) ([**@FeldmeierMichael**])
- ✨ Repair the feedline routing by turning couplers, the prototype's
  `run_final_routing_feedline_choices`: while fails are left, a coupler near
  them is turned to another of its options, what that unsettles is drawn
  again, and the turn is kept when it leaves strictly fewer fails, up to
  `repair_trials` times ([#118]) ([**@FeldmeierMichael**])
- ✨ Refine the feedline routing, the fifth phase of the Final stage: every
  wire of the ring and every edge at a launcher is drawn again against the
  centring price under the feedline constraints, for
  `feedline_refinement_rounds` rounds, zero by default for now, the prototype's
  `run_final_routing_feedline_refinement_parallel` ([#118])
  ([**@FeldmeierMichael**])
- ✨ Count a resonator too long, a wire crossing a feedline other than at a
  right angle, a wire that meets itself and an edge of a chain without a
  way as fails; every `Fails:`
  line says so, `-v 1` names them, and `-v` says where every coupler sits
  and how long every resonator is against the target ([#118])
  ([**@FeldmeierMichael**])
- ✨ Treat every edge of a feedline chain alike in the design-rule check: the
  conventional and inner wires cross any of them, at a launcher or between two
  couplers, and rule 1 binds instead between an edge and a resonator and
  between two edges, outside the reach of a coupler the two share. The edges
  at the launchers used to be checked against every wire, which walled off the
  plane from the chip edge to the first coupler; the prototype excludes them
  everywhere (`is_first_last_feedline`) ([#118]) ([**@FeldmeierMichael**])
- ✨ Check feedline orthogonality, rule 2 of the design-rule check, by the
  router's own crossing test (`routing::CrossingConstraints`, shared by the
  search and the check, reading a straight run off the cells rather than
  the moves, so that the straight lead of a move that bends counts as the
  straight run it is), and leave a resonator and the edges of its own
  chain alone near the coupler in rules 1 and 2, where they run beside each
  other on purpose. `mqt-scpd drc` and the tests read one view of a final
  routing (`drc::viewOfFinal`), the feedline edges and couplers included
  ([#118]) ([**@FeldmeierMichael**])
- ✨ Stop the Final stage after one of its five phases with
  `[stages.final] stop_after`, which takes the same five names `--phase`
  takes; the phases after it are not run, and asking a stopped run for a
  phase it does not hold is an error rather than a picture of the end state
  ([#118]) ([**@FeldmeierMichael**])
- ✨ Draw the coupler bodies in the pictures of the Final stage ([#118])
  ([**@FeldmeierMichael**])
- ✨ Lengthen every resonator of the Final stage to `meander_length`. After
  a resonator's search has found a way, one meander is spliced into a
  straight run of it: the two ends of the run are turned across it by one
  or two moves of the primitives, and one rectangular loop is built between
  them, as deep as the missing length makes it. The loop may enter what the
  search could enter and nothing else, so it cannot cross a neighbour the
  way itself could not. Phase 1 takes the first placement that fits; the
  relaxation and the refinement take the cheapest by the search's own price
  field, as the prototype's `meander_insertion` and
  `meander_insertion_proximity` do. The smallest loop goes as near the qubit
  as it fits, because the source end is where the coupler is spliced in and
  the feedline runs. A way without room for its meander is no way, so the
  relaxation goes on, and a resonator left short keeps the way it found and
  is tried again in the next round ([#117]) ([**@FeldmeierMichael**])
- ✨ Count a resonator too short as a fail. The `Fails:` line of every pass
  and of the stage says `short` beside `unrouted` and `open`, `-v 1` names
  the wires, and every search of a resonator says what its lengthening came
  to: the length before and after, or that there was no room and how many
  placements were tried. `-v` ends the stage on the length of every
  resonator — the way, the run to the port, and the two together against
  `meander_length` ([#117]) ([**@FeldmeierMichael**])
- ✨ Carry the length of every wire in the Final stage's artifact.
  `FinalWire.length` is the rendered length of the way in layout units, the
  exact curves of its bends and not the cells they sweep, so that a reader
  can check a resonator against `meander_length` without the primitives
  ([#117]) ([**@FeldmeierMichael**])
- ✨ Set `meander_length` per benchmark to the prototype's own figure: 2500
  layout units on the 4-qubit chip, 4000 on the 21-qubit chip, 6000 from
  the 33-qubit chip up, and the default of 3000 on the 9- and 17-qubit
  chips ([#117]) ([**@FeldmeierMichael**])
- ✨ Add the Final stage: every wire of the plan drawn again as
  curvature-constrained copper over the Dubins primitives of one bend radius,
  on a grid whose cell is about ten layout units. The stage runs five phases —
  the inner circuit, the ring, the couplers, the feedline chains and their
  refinement — and leaves a snapshot after each, so that one artifact carries
  what every phase drew ([#117]) ([**@FeldmeierMichael**])
- ✨ Run every routing phase from one driver. The prototype writes the same
  rip-up-and-reroute loop out four times, once per phase, and the four differ
  only in their parameters ([#117]) ([**@FeldmeierMichael**])
- ✨ Count the wire spacing between **every** pair of wires the Final stage
  draws. A search is fenced in as the prototype's is — by the two ring
  neighbours inflated by the clearance, and by nothing else — while the fails
  of a pass are counted on a field that carries how many wires guard each
  cell, so the count against two hundred wires costs what the count against
  two cost ([#117]) ([**@FeldmeierMichael**])
- ✨ Block everything between the chip outline and the rectangle the launcher
  slots stand on. A wire that enters that strip comes back in somewhere else
  and has gone around the sources of the wires beside it, which is a crossing
  the plan never allowed; where the edge of the routable space is, is derived
  from the sources rather than given as a cell count ([#117])
  ([**@FeldmeierMichael**])
- ✨ Bake the obstacle keepout into the router grid's mask, so that every cell
  a search may enter satisfies `min_obstacle_spacing` by construction. The
  distance is measured exactly, from the cell to the polygon edge in layout
  units, and the prototype's `FG_OBSTACLE_INFLATE` override is not carried
  over ([#117]) ([**@FeldmeierMichael**])
- ✨ Add the design-rule check in the cell view: wire clearance and wire loop,
  each written once and called by the stage's own tests, so that what the
  stage is judged by and what the checker reports cannot drift apart ([#117])
  ([**@FeldmeierMichael**])
- ✨ Report what a routing stage is doing while it runs: `mqt-scpd plan -v`
  prints one line per round of the Corridor, Detail and Final stages, with how
  many wires were tried, how many settled, how many are unrouted and how many
  open. It is a callback and not a report, because on the largest chip the
  Final stage is minutes of work and what it is doing is only useful live;
  nothing is printed from the core ([#117]) ([**@FeldmeierMichael**])
- ✨ End every routing stage on a `Fails:` line. A fail is a wire without a
  way of its own or a wire whose way comes within the rule of another, counted
  with every wire down and by the design-rule check's own test, so the last
  line of a stage says what `mqt-scpd drc` will find ([#117])
  ([**@FeldmeierMichael**])
- ♻️ Fence a search of the Final stage as the prototype's `run_final_routing`
  does, and by nothing else: the band around the way the wire has, less the
  two ring neighbours inflated by the clearance; then the relaxation along
  the sweep, which lets go of one more wire ahead per level — a wire let go
  of is no obstacle, because it is drawn again afterwards — fences with the
  last wire let go of and the neighbour on the other side, prices everything
  outside the lane between the two ring neighbours and the wires let go of at
  growing distances, and takes the way it finds. The verdict against the
  field, the relaxation against the sweep, the targeted rip and the rescue
  are gone ([#117]) ([**@FeldmeierMichael**])
- 🐛 Forgive a clearance encounter only where two wires actually meet. Two
  wires that end on one component were forgiven near their ports by the
  check, the count and the router's fence alike, on the assumption that a
  component's ports sit closer together than the wire spacing; the two ports
  of a qubit can sit nine hundred units apart, and a wire passed the other's
  approach at 140 units unreported. A junction is now geometry alone, two
  terminals within the clearance of each other ([#117])
  ([**@FeldmeierMichael**])
- 🐛 Route a wire to the first cell the straight-length rule allows. The
  target of a port sat two cells beyond the port's band, and the band ran to
  the rule's length, so the straight run into a port exceeded
  `min_straight_length` by twenty units along an axis and thirty along a
  diagonal. The band now ends on the cell before the first cell whose centre
  lies the rule's length from the port's own position, and the dig takes
  that cell rather than the one after it, so the excess is less than one
  step. With it the 17- and 21-qubit chips route with no fail ([#117])
  ([**@FeldmeierMichael**])
- ✨ Price the approaches of the wires around a relaxed search ten times over.
  A wire let go of is crossable, but the straight run out of its source and
  the run into its target are the two places it cannot be drawn anywhere
  else; a way through either took them for good. Each is priced as a port
  band is shaped, the straight length long and two wire clearances wide
  ([#117]) ([**@FeldmeierMichael**])
- ✨ Start every pass of the Final stage from the ways the Detail stage drew.
  The sweep is a rip-up and re-route, and a rip-up needs something to rip:
  every wire is put down on its Detail way, joined on the router grid with its
  straight stub in front, before the first search, and a wire that finds no
  way of its own keeps that seed. No wire is ever without a way; a wire still
  on its seed when the rounds end is unrouted, and the artifact carries no
  cells for it, as the prototype drops the path of a wire it could not route
  ([#117]) ([**@FeldmeierMichael**])
- 🐛 Count a committed way of the Final stage by the rule in layout units.
  The figure was declared and never set, so the stage never counted a wire
  too close to another and its "too close" count could not agree with
  `mqt-scpd drc` ([#117]) ([**@FeldmeierMichael**])
- ✨ Say what every search of the Final stage came to: `mqt-scpd plan -v 1`
  adds one line per wire and search — found in phase 1, or which neighbour
  each relaxation level let go of and whether it helped — names the picture
  of the search when `-d` is on, and ends every pass on which wires are
  unrouted, which open, and what each round came to wire by wire ([#117])
  ([**@FeldmeierMichael**])
- ✨ Draw what the Final stage looks at: `mqt-scpd plan -d` writes the router
  grid — artwork and keepout, the obstacle halo, every port as the chip
  carries it, every seed, every source and target with its heading — and
  then one picture per search into
  `<run>/debug/`, named after the pass, the round, the wire and the kind of
  search, showing the band, the fence, the prices, the neighbours, the wires
  let go of and the way found. Like the progress it is a callback, so the
  core writes no file ([#117]) ([**@FeldmeierMichael**])
- ✨ Render the Final stage: `plot --stage final --phase <name>` draws one
  phase, and `render --stage final` writes all five on layers of their own, so
  one GDS shows every phase ([#117]) ([**@FeldmeierMichael**])

- ✨ Add the Detail stage: every wire of the plan is drawn cell by cell on the
  detail grid, eight-connected, inside the partitions its corridor names, and no
  two wires share a cell ([#116]) ([**@FeldmeierMichael**])
- 🐛 Refuse the one crossing that shares no cell: a diagonal step past a corner
  both of whose cells belong to one wire. The prototype's occupancy is a set of
  pixels and never records the lines between them, so two eight-connected paths
  can cross there without either noticing ([#116]) ([**@FeldmeierMichael**])
- ✨ Draw a wire the per-partition pieces could not join in one search from the
  point it is fed at to its target. A piece has to begin and end exactly at the
  crossings the plan names, and where two wires both have to pass a narrow place
  neither order fits; what is binding is which partitions the wire runs through,
  not where along a border it crosses ([#116]) ([**@FeldmeierMichael**])
- ✨ Derive the clearance the detail stage keeps between two wires from
  `min_wire_spacing` on the detail grid. The prototype carries the same quantity
  twice, computed in one pass and as the literal 6 in the other, and the two
  disagree on every benchmark ([#116]) ([**@FeldmeierMichael**])
- ✨ Render the Detail stage: `plot --stage detail` draws every wire as the
  polyline of its bends, and `render --stage detail` writes the same on layers
  of its own ([#116]) ([**@FeldmeierMichael**])
- ✨ Draw the clearance a wire is entitled to: a band around every routed wire,
  translucent in the SVG and a path on a layer of its own in the GDS, so that two
  wires closer than the clearance are two bands that overlap. The band is
  `min_wire_spacing` as the router converts it to whole cells, not the rule
  itself, because a router that works in cells cannot keep a distance the cells
  do not divide ([#116]) ([**@FeldmeierMichael**])
- ✨ Hold the wire spacing between **every** pair of wires the Detail stage
  draws, the inner circuit included, rather than between a wire and the two
  beside it in the ring. The prototype cuts its clearance out of the search mask
  and can only afford to do it for two neighbours; here the canvas carries a
  field of how many wire cells lie within the rule of each cell, so the rule
  against 240 wires costs what the rule against two cost. Over the eight
  benchmarks this takes the places where the rule does not hold from 457 to zero
  ([#116]) ([**@FeldmeierMichael**])
- 🐛 Keep a wire's two fixed places charged with their clearance while the wire
  is off the canvas. The point the assignment feeds a wire at and the cell of
  its target port are where it has to be, and a wire drawn while another one was
  lifted could settle within a wire spacing of where that other one had to
  return to — a violation no later round can undo, because neither wire can move
  the place ([#116]) ([**@FeldmeierMichael**])
- 🐛 Count how close a wire runs to another with the wire itself off the
  clearance field. Its own two ends are within the rule of the cells beside
  them, so a wire that was charged before the count was always too close to
  itself, and no wire was ever finished ([#116]) ([**@FeldmeierMichael**])
- ✨ Treat the Corridor stage's crossings as a seed and not as a constraint: a
  wire is drawn again from the point it is fed at to its target and crosses
  where it can. The crossings a plan names sit as little as 13 layout units
  apart on the benchmark chips, and no arrangement that runs through them can
  hold a rule of 185 ([#116]) ([**@FeldmeierMichael**])
- ✨ Let go of the wires behind a wire as well as ahead of it when its re-route
  fails. The prototype's `back_count` loop describes the escalation and its
  `back_count <= 0` runs the body exactly once; running it is what takes the
  last three places on the 57-qubit chip ([#116]) ([**@FeldmeierMichael**])
- ✨ Relax the corridor a wire is re-routed in, letting go of the wire ahead of
  it one place further along each time and steering the search with the price of
  leaving the room between its two neighbours rather than forbidding it. Without
  it the clearance around those two covers the way most wires have, and they keep
  the corners their pieces met at ([#116]) ([**@FeldmeierMichael**])
- ✅ Check, over every benchmark chip, that every connection is drawn and that no
  two wires come within the design rule of each other — the inner circuit
  included. A wire that runs too close counts exactly as a wire that was never
  drawn ([#116]) ([**@FeldmeierMichael**])
- ✨ Add the Corridor stage: every assigned connection is routed through the
  partitions before any pixel is drawn, crossing a border only at a slot of its
  own and never meeting another wire inside a partition — neither crossing it
  nor running along it ([#115]) ([**@FeldmeierMichael**])
- ✨ Confine the coarse routing to the ring the ports feed from: a border is a
  place to cross only strictly inside the rectangle the feed points span, so no
  wire leaves through the ring and comes back in behind another port ([#115])
  ([**@FeldmeierMichael**])
- 🐛 Refuse a wire that runs over a point another wire is pinned to: the point
  it is fed at, or the cell of its target port. Neither can be moved aside, and
  the crossing test cannot see the case at all — a chord that stops on another
  one never reaches its far side — so a wire could run down a whole line of
  feed points with no crossing reported ([#115]) ([**@FeldmeierMichael**])
- ✨ Render the Corridor stage: `plot --stage corridor` draws every wire's way
  through the partitions and every crossing slot, taken or free, and
  `render --stage corridor` writes the same on layers of its own ([#115])
  ([**@FeldmeierMichael**])
- ✨ Fill the cells of a partition back in from its outlines, so a routing stage
  can work inside one without growing the watershed a second time ([#115])
  ([**@FeldmeierMichael**])
- 🐛 Report the wire budget of a partition border as the number of wires it can
  carry. It was the length of the border in detail cells, which is what its own
  comment already denied ([#115]) ([**@FeldmeierMichael**])
- ✨ Let a target that no partition border reaches leave along its own port's
  approach. The capacity grid does not open the band it stamps, so a port the
  band and the artwork close around sat in a pocket no wire could leave
  ([#115]) ([**@FeldmeierMichael**])
- ✨ Add `[stages.capacity] crossing_pitch`: how finely a partition border is
  divided into places a wire may cross. It is a planning figure and not a
  clearance rule ([#115]) ([**@FeldmeierMichael**])

### Fixed

- 🐛 Keep the edges of one chain from crossing in the coupler insertion. The
  first and last edge of a chain fence every edge of it again
  (`SCPD_TERMINAL_EDGES_FENCE_ALL`, on by default); on 69q chain 2's f15
  had run over f13. The slots at an edge's two ends no longer open the
  copper of an earlier edge of its chain, and `CHECK chain crossings` says
  after the insertion and after the feedline pass how many pairs of one
  chain cross.
- 🐛 Test a remembered edge of the prefix search against the prefix that
  stands and route it again when the prefix runs into it; the memo key
  holds the pair of options but not the prefix, so a way found under one
  prefix was priced under another. A step that may lay its end pair the
  other way round gets no step budget, so a budget cut-off no longer starts
  the reorder and marks the answer as not optimal.
- 🐛 Refuse a coupler place whose way left over, with the lead, is longer
  than the target length; the overshoot test left the lead out. Refuse an
  option whose feedline run, as long as the edge search forces it, meets the
  artwork or a port's approach. Break ties of the nearest launcher by port
  number and of the places by their index, keep the greedy on the first of
  equal options, put the fence flag back where it was after the commit's
  test, and keep the bend penalty of an edge inside 16 bits.
- 🐛 Let the meander reach the near end of a resonator's way again. The
  cells it leaves alone at the start were `coupler_length` plus the
  straight start plus four, where the straight start is the second run
  after the coupler's lead that `SCPD_RESONATOR_STUB` took away on
  2026-10-03 — the margin did not follow. On the 4-qubit benchmark that
  reserved 35 of the 70 straight cells of a 111-cell way while a loop needs
  35 cells of span between its two ends, so every one of the 4760
  placements tried was refused for want of span and not one for want of
  room. The margin is `coupler_length + 4` now
  (`SCPD_MEANDER_MARGIN_STUB`, `SCPD_MEANDER_START_MARGIN`,
  `SCPD_MEANDER_LEG_SPACING`), and the chip ends with no failing wire and
  no resonator off its length.

### Changed

- 🐛 Judge a cell of a wire by the crossing rule on the heading it arrives
  with as well as its own (`CrossingConstraints::allowedArriving`), in the
  Final stage's count and in the design-rule check: the search tests every
  cell of a move on the heading the move starts on, so a wire that crossed a
  feedline straight and turned on the last cell of the band was counted
  crossing although it kept the right angle for the whole band.
- ⚡ Compute `grid::squaredDistanceTransform` in time linear in the cells, as
  the lower envelope of one parabola per row (Felzenszwalb and Huttenlocher).
  The values are the same; on the 17-qubit chip the bottleneck analysis spends
  17 ms on it instead of 64 ms.
- ♻️ Quote every figure of the Detail stage against the grid rather than in
  cells of it. `[stages.detail] corridor_half_width`, 40 cells, becomes
  `corridor_spacings`, 4 wire spacings; `obstacle_penalty_radius`, 6 cells,
  becomes `obstacle_penalty_reach`, 185 layout units. A cell is 19 layout units
  on the 17-qubit grid and 40 on the 9-qubit one, so the same cell count stood
  for two different distances ([#116]) ([**@FeldmeierMichael**])
- ♻️ Raise `[stages.detail] rounds` to 30 and `max_relaxation` to 30, from the
  prototype's 8 and 10. Its figures are enough for the clearance it holds — to
  two wires — and holding it against every wire takes more sweeps to settle: at
  8 and 10 the eight benchmarks leave 46 places where the rule does not hold
  ([#116]) ([**@FeldmeierMichael**])
- ✨ Add `mqt-scpd plan`, the resumable run directory, and
  `mqt-scpd list-algorithms` ([#114]) ([**@FeldmeierMichael**])
- ✨ Render the planning stages through the existing commands:
  `plot --stage capacity|global|assign` draws them over the chip as SVG, and
  `render --stage` writes the same content to GDSII or OASIS on layers of its
  own ([#114]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdPipeline`: the stage interfaces, the name registry, and the
  Capacity, Global and Assignment stages ([#114]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdMilp`: the solver-neutral model, the linked-in HiGHS backend,
  MPS emission and the runtime backend selection ([#114])
  ([**@FeldmeierMichael**])
- ✨ Add the bring-your-own-licence Gurobi backend, reached through `gurobipy`
  over an MPS round trip ([#114]) ([**@FeldmeierMichael**])
- ✨ Add the capacity layer of `MQT::ScpdGrid`: the medial axis over
  Boost.Polygon, bottleneck detection, partition extraction and the per-cell
  wire budgets ([#114]) ([**@FeldmeierMichael**])
- ✨ Prune the bottlenecks to the gates the capacity chains cross, and derive
  the partitions from the chambers those gates carve out ([#114])
  ([**@FeldmeierMichael**])
- ✨ Keep the inner circuit's Hanan lattice off the chip artwork ([#114])
  ([**@FeldmeierMichael**])
- ✨ Make the inner circuit pay for the free space it crosses: each capacity
  chain constrains the flow, so a wire may surface at an outer port only where
  the chain behind it carries one to a launcher ([#114])
  ([**@FeldmeierMichael**])
- ✨ Declare which two ports of a component a wire crosses between: the new
  `bridge_pair` role names the ports, and one `[[ports.bridge_pairs]]` rule per
  crossing pairs them. The two declarations are checked against each other at
  load ([#114]) ([**@FeldmeierMichael**])
- 🐛 Give an inner target one wire over every lattice at once, instead of one
  per lattice that carries it. A target beside a chamber border is a node of two
  lattices, and the second wire took the supply another target then had to do
  without ([#114]) ([**@FeldmeierMichael**])
- ✨ Shut a bridge whose far end the outer ring does not name, so no inner wire
  crosses a component the assignment never sees.
  `[stages.global] internal_bridges` grants the crossing ([#114])
  ([**@FeldmeierMichael**])
- ✨ Draw the wire budget of every gate beside it in the SVG, in the capacity
  and the global picture alike ([#114]) ([**@FeldmeierMichael**])
- 🐛 Report one gate per narrowing. The saddle search returns a line at every
  cell of a plateau of equal clearance, so one narrow place came back several
  times over, and a chamber that two of those lines led out of was credited with
  twice the room the chip has ([#114]) ([**@FeldmeierMichael**])
- 🐛 Keep the gates of a chamber that several corridors meet. A gate counted as
  hidden unless *every* cell beside it saw it unobstructed, which is true of no
  gate that has a neighbour, so such a chamber lost all of its gates and the
  free space beyond it fell out of the plan ([#114]) ([**@FeldmeierMichael**])
- ✨ Report what the ports' own approaches keep clear in the capacity plan, and
  draw it on a layer of its own. The port bands and the launcher sweeps are
  obstacles the chip input does not carry, so no picture could show them
  ([#114]) ([**@FeldmeierMichael**])
- 🐛 Carry a port's component through `mqt-scpd inspect`, which dropped it on
  the way to JSON and back ([#114]) ([**@FeldmeierMichael**])
- ✨ Give a bridge port twice the forward approach, worked out from the
  component's own port pairs rather than from its label ([#114])
  ([**@FeldmeierMichael**])
- 🐛 Give every port of the outer ring a connection in the assignment. Only the
  resonators that ended a feedline carried one, so the artifact named
  `launcher_target` ports of the ring and said nothing about the rest ([#114])
  ([**@FeldmeierMichael**])
- 🐛 Feed every resonator from a point between two launchers, not only one that
  ends a feedline. What stands on a launcher slot is now a conventional port and
  nothing else; the resonators a launcher was given divide the segment to the
  next launcher along evenly, in ring order ([#114]) ([**@FeldmeierMichael**])
- 🐛 Run the assignment's ordering potential over one turn of the launcher ring.
  Over the ring's own length the walk could turn twice, which put two
  conventional ports on one launcher on five of the eight benchmark chips and
  broke the cyclic order of the ring ([#114]) ([**@FeldmeierMichael**])
- 🐛 Charge a resonator's ordering step to its own launcher. The step was
  charged to the next resonator along whenever the ring opened on a conventional
  port, which is four of the eight benchmark chips ([#114])
  ([**@FeldmeierMichael**])
- ✨ Carry where each ring node is fed from in the assignment as `feeds`, so
  both renderers draw the chord to the point the wire starts at ([#114])
  ([**@FeldmeierMichael**])
- ✨ Add the component a port belongs to, declared by one more configured
  pattern, and print the grouping in `mqt-scpd doctor` ([#114])
  ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdRouting`: the curvature-constrained A\* over Dubins
  primitives, the path geometry and its sampler, the self-intersection guard and
  the coupler dogleg insertion ([#113]) ([**@FeldmeierMichael**])
- ✨ Add `MQT::ScpdGrid`: the grid metrics and the rule-to-cell conversion, the
  obstacle rasterization with the design-rule keepout, the distance transform,
  the watershed partitioning and the port keep-out bands ([#113])
  ([**@FeldmeierMichael**])
- ✨ Add the three grid coordinate types of the data model to the geometry
  schema ([#113]) ([**@FeldmeierMichael**])
- ✨ Add the `mqt-scpd` command line with `doctor`, `plot`, `render` and
  `inspect`: the `config.toml` and chip loaders, the port-role classification,
  the layout SVG, the KLayout adapter and the JSON view of the artifacts
  ([#105]) ([**@FeldmeierMichael**])
- ✨ Add the eight benchmark configurations and their chip inputs ([#105])
  ([**@FeldmeierMichael**])
- 👷 Fail CI when the committed schema-generated code is stale ([#98])
  ([**@FeldmeierMichael**])
- ✨ Add semantic validation of the data model in the core and a checked
  artifact read and write layer in C++ and Python ([#98])
  ([**@FeldmeierMichael**])
- ✨ Add the FlatBuffers schemas of the data model, the committed C++ and Python
  code generated from them, and the `nox -s schemas` session ([#98])
  ([**@FeldmeierMichael**])
- 🏗️ Split the core into the eight per-module CMake targets of the architecture
  ([#98]) ([**@FeldmeierMichael**])
- 🐍 Start building CPython 3.15 wheels ([#67]) ([**@denialhaag**])
- ✨ Set up the repository ([#1]) ([**@denialhaag**])

### Changed

- 💥 Run the Global stage before the Assignment stage, and number the artifacts
  `02-global.fb` and `03-assign.fb` accordingly; which outer port an inner wire
  surfaces at is what the assignment's ring is made of ([#114])
  ([**@FeldmeierMichael**])
- ♻️ Build the grid and stage sections of a configuration whether or not the
  file carries them, so that an absent section is the defaults rather than
  nothing for the stage that reads it ([#114]) ([**@FeldmeierMichael**])
- ♻️ Make `[ports.sequences]` required and drop the `detection` and
  `start_component` keys of the configuration schema; the outer port ring is
  configuration only ([#105]) ([**@FeldmeierMichael**])
- 💥 Drop support for x86 macOS and stop publishing the respective wheels
  ([#89]) ([**@denialhaag**])
- ⬆️ Raise the macOS deployment target to 13.3 to enable `std::format` in libc++
  ([#89]) ([**@denialhaag**])
- 💥 Require Python 3.11 or newer ([#89]) ([**@denialhaag**])
- ⬆️ Update `nanobind` to version 3.0.1 ([#83]) ([**@denialhaag**])

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd

<!-- PR links -->

[#116]: https://github.com/munich-quantum-toolkit/scpd/pull/116
[#118]: https://github.com/munich-quantum-toolkit/scpd/pull/118
[#117]: https://github.com/munich-quantum-toolkit/scpd/pull/117
[#115]: https://github.com/munich-quantum-toolkit/scpd/pull/115
[#114]: https://github.com/munich-quantum-toolkit/scpd/pull/114
[#113]: https://github.com/munich-quantum-toolkit/scpd/pull/113
[#105]: https://github.com/munich-quantum-toolkit/scpd/pull/105
[#98]: https://github.com/munich-quantum-toolkit/scpd/pull/98
[#89]: https://github.com/munich-quantum-toolkit/scpd/pull/89
[#83]: https://github.com/munich-quantum-toolkit/scpd/pull/83
[#67]: https://github.com/munich-quantum-toolkit/scpd/pull/67
[#1]: https://github.com/munich-quantum-toolkit/scpd/pull/1

<!-- Contributor -->

[**@denialhaag**]: https://github.com/denialhaag
[**@FeldmeierMichael**]: https://github.com/FeldmeierMichael

<!-- General links -->

[Keep a Changelog]: https://keepachangelog.com/en/1.1.0/
[Common Changelog]: https://common-changelog.org
[Semantic Versioning]: https://semver.org/spec/v2.0.0.html
