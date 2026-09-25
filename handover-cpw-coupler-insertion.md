# The CPW coupler insertion

Written for whoever takes the coupler insertion further. The geometry was
rebuilt from the ground up in this session: what a coupler *is* on the grid,
where it sits, which way it faces and how its resonator leaves it are all
different from what [0032](docs/design/decisions/0032-the-coupler-couples-along-the-ring.md)
describes. Read this before that decision, and read
[handover-final-couplers.md](handover-final-couplers.md) for the feedline
chains, the repair and the refinement, which this session did not touch.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch: `phase-4-routing-stages`, HEAD at `cd8e597`. **Nothing of the
  coupler insertion is committed** — 55 files are modified in the tree. The
  user commits per phase; leave the work in the tree and say what you
  verified.

## Where it stands

Measured with the stage stopping after the feedlines and the repair off
(`repair_trials = 0`, `stop_after = "feedlines"`), which is what the config
files in the tree currently say.

| chip | **Fails** | open | short | long | crossing | edges drawn |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 4q  | **0**  | 0 | 0  | 0  | 0 | 5 of 5 |
| 9q  | **8**  | 0 | 3  | 5  | 0 | 10 of 10 |
| 17q | **22** | 4 | 3  | 12 | 5 | 20 of 20 |

**4q is clean end to end** — seventeen wires, no fail of any kind, every
resonator within 100 units of the 2500 the rule asks. It was never clean
before this session.

21q and up are **not measured**. They were measured earlier in the session
at settings that no longer exist; do not carry those numbers forward.

Every chain edge is drawn on all three chips, and no wire crosses a feedline
on 4q or 9q. What is left is length: twelve resonators on 17q are **too
long**, which is the fail nothing repairs — the meander can only lengthen a
way, never shorten one.

## What a coupler is now

A pad, and a lead that the resonator leaves it on. Four ports, not one.

```text
        feedline in ●───────────────────────● feedline out     far edge
                    ┌─────────────────────┐
                    │         pad         │  run × depth
                    └─────────────────────┘
     resonator port ●───────────────────────● resonator port   near edge
              (2)   ╰─╮                        (1)
                      │ arc, a quarter turn
                      ╰──────────● insertion point, on the resonator's path
                         lead straight
```

- **The pad** spans `coupler_length` along its orientation and
  `coupler_height` across it, in cells that carry the design figure on that
  heading — fewer cells along a diagonal, because a diagonal step is longer.
- **Two feedline ports** at the ends of the far edge, both carrying the pad's
  axis. The chain arrives at one and leaves from the other. (Both
  `sourceOf` and `targetOf` return `option.out`, so in fact the two edges
  meet on *one* cell and `option.in` is never used by the chain — see *What
  is open*.)
- **Two resonator ports** at the ends of the near edge, opposite the feedline
  pair. The first faces along the pad's axis, the second against it. Which
  one a coupler uses is an option, not a constant.
- **The lead** is the component's own: from the resonator port a quarter
  turn onto the coupler's orientation, then `leadStraight()` cells straight.
  Which way the arc turns is *derived*, not set — the two ports face opposite
  ways, so one reaches the orientation with the clock and the other against
  it.

**The orientation of a coupler is the direction the straight after the arc
runs.** Not the pad's long axis. Geometrically it is the perpendicular from
the line between the feedline ports to the line between the resonator ports.
It is stored on the option as `couplerOrientation`; the pad's axis is
`orientation` and stands a quarter turn back from it.

### Where the coupler sits

**The tip of the lead lands on the resonator's path.** Not the pad's centre —
that was the model before and it is gone. The lead is built first, in a frame
whose origin is the resonator port; the port is then the place less the whole
lead, and the pad hangs off the port.

**The place is chosen per orientation.** Each of the eight offsets walks the
places in order of how well they match the biased target length and stops at
the first one *it* fits — pad, feedline run and lead all inside the box. A
place that is perfect on length but puts the pad through the box edge is not
that orientation's place; the next one along the way is tried. Settling one
place for all eight was what cost the options.

### The option space

**8 offsets × 2 resonator ports = 16 options per coupler**, and on 4q, 9q and
17q every coupler has all sixteen. The offset counts from **the heading the
nearest launcher faces**, so offset 0 means the same thing on every coupler:
the resonator leaves pointing the way the launcher that drives it points.

Counting the offset from the heading the resonator's own way happened to hold
at the place — which is what it used to do — made offset 0 a different thing
on every coupler, because that heading is a property of the routing and moves
whenever the outer routing moves.

## Where the pieces are

Line numbers drift; search the name.

| Piece | What it does |
| --- | --- |
| `Driver::optionsOf` | the 16 candidates, each walking the places for its own fit |
| `Driver::makeOption` | one option's whole geometry, and every reason to refuse it |
| `Driver::couplerPlace` | the candidate places, sorted by length mismatch alone |
| `Driver::leadStraight` | the straight after the arc, never below `min_straight_length` |
| `Driver::nearestLauncherHeading` | what the offset counts from |
| `Driver::optimizeChain` | the greedy: every option routed, no pre-screen |
| `Driver::localCost` | two edges routed, both orders tried, memoised |
| `Driver::edgeMemoKey`, `edgeMemo_` | the cache, keyed on the option pair at an edge's two ends |
| `Driver::fenceCommittedEdges` | every active feedline, inflated by the clearance |
| `Driver::drawCouplerOptions` | `final-coupler-options.svg`: every option of every coupler |

## How to run it

```bash
cmake --build --preset release
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .

.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml -o artifacts/17q -v 1
```

**`plan --stage final` reads the config from the run directory.** This cost
an hour in this session and a wrong diagnosis in front of the user: three
chips were measured with `stop_after = "couplers"` still sitting in
`artifacts/<chip>/config.toml` while `benchmarks/<chip>/config.toml` said
`"feedlines"`, so phase 4 never ran and every resonator stood at its lead
length. **Copy the config over first**, every time:

```bash
cp benchmarks/17q/config.toml artifacts/17q/config.toml
.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml -o artifacts/17q --stage final -v 1
```

`-v 2` explains every one of the 16 candidates per coupler: which place it
took, or which check refused it. `-d` writes the pictures.

## The pictures

- **`final-coupler-options.svg`** — drawn once, before the greedy. Every
  option of every coupler: pad, lead and feedline port in pale violet, the
  option the coupler starts on over them in pink. For "is the right one
  there, and is it the one we begin with".
- **Every edge picture** now also carries the couplers and the feedlines as
  they stand at that moment, under the wires of the search.

**A picture that shows nothing is not proof that nothing is there.** The
feedline fence was measured closing 211 464 cells while the picture showed
none of it, because most of it lies over artwork the ground layer has already
drawn dark. Measure before believing a picture, and believe the measurement.

## What is open

1. **The length, and it is the whole remaining gap.** `couplerPlace` picks
   the place by how much of the *old* way is left from there, but the search
   now starts at the **lead's end**, a whole lead further on. The figure the
   place is chosen by is therefore systematically not the length the search
   has to draw. Twelve resonators on 17q come out too long, which nothing
   downstream repairs. **This is one line: subtract the lead from `wanted`.**
   It was not done because it wants measuring, not guessing.
2. **Two fences are dead code.** `fenceResonators` has **no caller** —
   resonators are invisible to the insertion's edge searches. `closeOutsideBox`
   has none either, switched off on request; the comment at the call site says
   how to switch it back on. Both are written, both look like rules that hold,
   and neither does. A third, `fenceCommittedEdges`, was in the same state at
   the start of this session and cost 16 open fails on 17q until it was hung
   back in.
3. **The greedy's cost sees only the feedline.** `localCost` is
   `10000 × angleCostOf` for the two edges and nothing else; the terms for
   length, length difference and `100 × guarded` are commented out. Nothing
   about the resonator enters the decision, so the two resonator ports of one
   coupler are an exact tie and the loop order decides between them. The
   sixteen options are therefore more room to choose wrongly, not less.
4. **`option.in` is never used.** Both ends of the chain pin on `option.out`,
   so the feedline does not run the length of the pad as the geometry says it
   should. Worth settling before anyone reasons from the picture.
5. **Diagnostics are still in the tree** — the `fence for edge …` and
   `picture …` lines and their counters. They cost nothing at `-v 0` but are
   not part of the finished stage.
6. **The configs are in a measuring state** and must go back before a commit:
   `repair_trials = 0` and `stop_after = "feedlines"` in all eight
   benchmarks.

## Traps

**1. An empty way turns nowhere.** `angleCostOf` returns 0 for an empty path,
so an option that loses an edge scored *cheaper* than every option that kept
one. `localCost` now reports how many edges it lost beside the cost, and
`optimizeChain` discards any option that loses one. This was a real defect,
found by the user from a picture.

**2. A rule that only one path obeys is not a rule.** The commit reuses the
path the greedy chose whenever the endpoints still match, and for a long time
it checked nothing else — so a way drawn against a chip with no feedlines on
it shipped straight through the ones added since. `edgeWayStillOpen` now
tests every cell of a kept path against the corridor that holds at the
commit. The same shape of bug cost a clearance violation on 33q earlier.

**3. A heuristic and a rule that disagree cost you the option, not the
heuristic.** The feedline's side of the pad was guessed from the resonator's
run direction, and the legality rule then demanded one particular side: five
of every eight offsets were built and thrown away for no geometric reason —
240 of 416 candidates on 17q. Deriving the side from the rule made every
offset legal.

**4. Caching the wrong key caches nothing.** A memo keyed on the option with
a generation bumped on every move scored **0 hits of 1360**: every move
invalidates everything, and the greedy asks each question once per pass.
Keyed on the pair of options at an edge's two ends it scores about half —
671 of 1360 on 17q — and halves the insertion's runtime with byte-identical
fails. The key deliberately leaves out the state of the other chains; an
entry can outlive the ground it was found on, and the fails are what says
whether that is affordable.

**5. The place cap.** Walking the places per orientation with a cap of 24
destroyed 4q outright — all 24 places nearest the target length put the pad
through the box edge, and every coupler was given up on while places that
fitted lay further along. There is no cap now.

## Measured and rejected

- **Ranking options by lost edges instead of discarding them.** 17q 20 fails
  against 21 for discarding, but the user asked for discarding and it is the
  safer rule: an option that loses an edge is never weighed.
- **The box as a hard bound on every wire.** 4q went to 13 fails with nothing
  drawn, 9q to 31 with 28 unrouted. Ordinary wires are fed from points on the
  launcher ring, which lies *outside* the box, and `openFixedPlaces` reopens
  only the stub — not the `clearance + 1` cells beyond it. The box can bind
  feedline edges; it cannot bind wires fed from outside it.
- **Dropping the `leaves` heuristic while keeping the old option space.** More
  options, worse results — 9q 7 → 8, 17q 21 → 23 with four edges undrawn,
  because the freed options are the ones that send a resonator across its own
  feedline and the greedy cannot tell.
- **Reversing the resonator port headings** (9q: 7 → 8, 17q: two edges lost)
  and **reversing the search's start angle in the feedline pass** (every
  resonator on 4q found nothing: the arc is a fixed head, and a reversed start
  makes the wire double back on it).
