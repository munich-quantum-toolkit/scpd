# A\* over coupler option prefixes

Written for whoever takes this further. It replaces the per-chain trellis of
the coupler insertion with a best-first search over **prefixes** of coupler
options, so that every feedline edge is routed against the ways the options
before it have already laid. Read
[handover-cpw-coupler-insertion.md](handover-cpw-coupler-insertion.md) first —
this document assumes its vocabulary (chains, options, the analytic bound, the
rounds, `stateFaults`) and describes only what changes.

- Checkout: `/Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4`
- Branch `phase-4-routing-stages`, HEAD **`9d20ddc` ⚡️ Until 57q all feedlines
  drawn**. Everything below is **uncommitted** on top of it, the pad-crossing
  filter in `Driver::makeOption` included. The user commits per phase — the
  work is in the tree and this says what was verified.
- **It is the default.** `SCPD_CHAIN_ASTAR=0` is the way back to the trellis,
  and `SCPD_CHAIN_DP=0` from there to the greedy.

## Why

The trellis rests on one assumption, which `ChainTrellis.hpp` states:

> *the price of a step depends on the choice at both of its ends and on
> nothing else*

**It is false.** Two feedline edges that meet at a coupler block each other,
so a step's price depends on what was laid before it. The trellis prices a
pair that cannot both be built as though it could, `stateFaults` counts the
difference, and the commit is where an edge is lost.

Carrying the predecessor in the node — the second order — closed that for the
two edges that **share** a coupler, and on 17q that was the whole of it: six
undrawn edges to none. It never reached 57q and 69q. 57q lost the same five
edges whether its widest chains were solved at second order or handed back to
the first, at 2641 s against 810 s for the privilege, so what was lost there
was blocked by edges that are **not** neighbours, and no width of node reaches
that.

This is that approximation dropped. A node is a prefix, each of its edges is
routed against the ways the prefix has laid, and a prefix that blocks itself
costs everything and the search takes another combination.

## Where it stands

Defaults, one run per chip, `stop_after = "couplers"`, `repair_trials = 0`,
`--stage final`. The pad filter is in for both halves.

| chip | not drawn | angle | insertion | | trellis: not drawn / angle / insertion |
| --- | ---: | ---: | ---: | --- | --- |
| 4q | **0** | **8** | 0.3 s | | 0 / 8 / 0.3 s |
| 9q | **0** | **20** | 0.6 s | | 0 / 20 / 0.6 s |
| 17q | **0** | **47** | **2.1 s** | | 0 / 51 / 11.2 s |
| 21q | **0** | **48** | 2.9 s | | 0 / 48 / 2.9 s |
| 33q | **0** | **76** | **7.2 s** | | 0 / 76 / 10.6 s |
| 45q | **0** | **105** | **28.1 s** | | 0 / 105 / 36.0 s |
| 57q | **0** | **130** | **200.7 s** | | **2** / 133 / 356.8 s |
| 69q | **2** | 187 | **240.5 s** | | **11** / 162 / 1023.3 s |

**57q draws every edge**, which nothing before it did — the trellis lost two
and the greedy six. **69q loses two of eighty-one** where the trellis lost
eleven and the greedy six, in a quarter of the time.

**Read 69q's angle column with care.** An edge that was not drawn turns
nowhere and `angleCostOf` scores it zero, so the trellis's 162 is over 70
drawn edges and our 187 over 79 — 2.31 an edge against 2.37. Per drawn edge
the two are level; what changed is how many are there at all.

**Every chain of six of the eight chips is a proved optimum.** 57q's chain 4
reads `best in the time given` and 69q's chain 8 `best found, an end pair laid
the other way round`; 69q's chain 2 has no answer at all. Everything else is
settled exactly.

**`stateFaults` is zero on seven of the eight chips.** That is the measurement
to look at, and it is what the prefix fence buys: the state the search hands
over survives its own fence entire, so the commit keeps every way the search
found and there is nothing for a second pass to correct. Only 69q carries
faults, and only because one of its chains cannot be joined.

## The search

`routing::solveChainAStar` (`include/mqt-scpd/routing/ChainSearch.hpp`,
`src/routing/ChainSearch.cpp`), router-free and unit-tested alone, exactly as
`routing::solveTrellis` beside it.

**A node is a prefix** — the options chosen for layers `0..k`. It is held in
an arena as `{parent, choice, layer, price, real}`, so the run of choices is
read by walking parents and no node copies one.

**The price** of a node is the real routed price of the edges in its prefix,
each routed against a corridor holding everything committed elsewhere **plus
the ways of the earlier edges of this same prefix**.

**The heuristic** is `rest[layer][choice]`: the cheapest run of analytic
bounds (`routing::AnalyticDubins`) from that choice to the end of the chain,
settled by one backward pass over the bound matrices — `O(L n²)` answers of a
function that costs half a microsecond. It is the exact optimum of the
relaxation in which no step blocks any other, so it is a lower bound under
*any* prefix, and A\* with it returns the true optimum.

**A step is routed when the search reaches it, not when it is generated.** A
child is pushed carrying the analytic bound for its own step; when it is
popped, that step is routed for real, its price and `f` are corrected and it
goes back on the queue. Only a node popped with its own step already real is
expanded. The placeholder is a lower bound, so this changes nothing about the
answer, and it means a step is routed exactly when the search commits to
looking at it. `solveTrellis` defers its pricing the same way.

**Ties break on the deeper prefix, then on the older node.** Neither can bear
on the price of the answer; they are there so that two runs of one chain
answer the same, and preferring the deeper one dives at the end of the chain
rather than spreading across a layer.

**Nothing is merged.** The node count is exponential in the worst case, and
the edge cache and the time limit below are the answer to that.

## There are no rounds

`Driver::optimizeChainsPrefix` is one pass over the chains, each solved
against the ways the chains before it settled on. `optimizeChainsExact` and
its four rounds stay for the trellis and are untouched.

The rounds exist because the trellis prices a chain against a fence it cannot
see all of: its answer may not survive its own fence, so a later round has
something to correct, and the round that is kept has to be chosen by
`stateFaults`. None of that applies here, because the prefix search routes
every edge against the ways the options before it laid.

**That is measured, not argued.** Under the rounds, seven of the eight chips
reported **zero faults after round 0** and the loop stopped there of its own
accord; only 69q ever paid for a second round, where it changed nothing and
cost 200 s. `stateFaults` is still reported once, at the end, because it is
the number this whole approach is judged by — an edge counted there is an edge
the commit routes again from scratch — but it decides nothing.

**What the rounds were worth on 69q, said plainly.** They gave a chain that
ran out of time a second attempt, and at a 60 s budget that was worth three
edges. Raising the budget to 180 s buys the same three back in one pass and
more cheaply, which is why the rounds could go — but if 69q ever loses edges
to the clock again, this is the thing that was taken away.

## The edge cache

**The two edges at each end of a chain are routed again for every prefix. The
ones in between are remembered by the pair of options at their two ends**
(`CHAIN_FRESH_EDGES = 2`, user 2026-09-28), through `edgeMemo_`.

The pair of options is not the whole truth on this path — the prefix fence is
part of the question — so a remembered entry outlives the ground it was found
on, exactly as `edgeMemo_` already says of the greedy's entries. The ends of a
chain are where that costs too much to accept: an edge at a launcher leaves
the terminal on one heading and cannot yield, and the edges beside it want the
same ground.

**It is the difference between finishing and not.** Without it, one chain of
nine waypoints on 57q had routed **24 897 edges in 1 400 s** and was not done;
**14 400 of those were one edge**, asked for the same 256 pairs of options
over and over. With it, 45q fell from 67.0 s to 27.7 s and 57q finished at
all. The small chips reuse nothing, because a chain of five or six edges is
all ends and no middle.

## The other order, for the pair at either end

The prefix lays its edges front to back, so the edge before this one took the
room first and this one has to work around it. **At the ends of a chain that
order decides whether both fit at all** (user, 2026-09-28): an edge at a
launcher leaves the terminal on one heading and cannot yield, and the edge
beside it wants the same ground.

So when the step into the second edge of either end pair finds no way, it is
tried the other way round: **this edge first, then its predecessor again
against it.** If both come home, the pair is kept in that order. The
predecessor's new way hangs off *this* prefix's key, because the shorter
prefix is shared by every choice that extends it and only this choice needed
its predecessor moved.

**It costs the proof, and the search says so.** The price of the prefix
changes with the predecessor's way, which `ChainProblem::step` carries back as
`redone`; a prefix can then be *cheaper* than the prefix it extends, and that
is the one thing A\* rests on not happening. `ChainSolution::relaid` records
it and clears `optimal`, and the log then reads `best found, an end pair laid
the other way round` rather than `optimum`.

**What it buys, measured on 69q:** it fires 41 times, and chain 8 comes home
only through it. It does **not** rescue 69q's chain 2, which exhausts every
run of options in 52 s: there really is no sequence that joins that chain.
On 57q it fires on chain 4 and on the smaller chips a handful of times,
changing no chip's outcome.

## The time limit

**`SCPD_CHAIN_ASTAR_SECONDS` bounds one attempt at one chain, 180 s by
default.** When it runs out the search hands back **the cheapest complete run
of options it has reached** — every edge of that run drawn, and drawn against
the run itself — and says so: `ChainSolution::outOfTime` is set and the log
reads `best in the time given`. Zero is no limit.

**180 s and not 60 s, and the two chips disagree about it.** At 60 s, 69q's
chains 2 and 5 both ran over and, with no second pass to recover them, it lost
**five** edges instead of two — for 5 s saved. Chain 5 settles in 79 s. 57q
pays the other way: its chain 4 reaches the limit whatever it is, so 180 s
costs it **115 s for nothing at all** — 200.7 s against 86.1 s, at the same 66
edges and the same angle 130. Edges drawn is the criterion, so 180 s stands;
`SCPD_CHAIN_ASTAR_SECONDS=60` is the setting for a quick 57q.

**It is spent per attempt, not per chain.** A chain whose plain options do not
reach opens its jogs and searches again, and the second attempt starts the
clock over, so such a chain can spend twice this.

**`-v 1` says when a run of options comes home.** Every time the search
completes a run whose every feedline edge is drawn, it prints the run and what
it costs:

```text
[Coupler Insertion]   chain 0: every feedline edge drawn on options
                      launcher 18 3 0 12 32 launcher -> cost 180000
[Coupler Insertion]   chain 0: every feedline edge drawn on options
                      launcher 18 3 0 12 41 launcher -> cost 160000
```

They do not arrive cheapest first: the end of a chain is reached in order of
the *bound* on what a run costs, and its last step is priced only once it is
reached. The cheapest is kept. **A chain that never prints this line is a
chain the search never joined**, which is the quickest way to read a failure.

**Two failures, and they are not the same failure.** A chain that comes back
unsolved reports either `no run of options joins the chain` — the search saw
every run there is, which is a verdict about the geometry — or `out of time
before any run of options joined the chain`, which is a verdict about the
clock. `ChainSolution::outOfTime` is what separates them, and it has to be its
own field: `optimal` will not do, because a reordered end pair clears that too
and a chain that had merely reordered then read as one that had run out of
time. Calling the two by one name hid a 69q chain that had spent 74 s against
a 60 s budget for a whole sweep.

## Where the pieces are

| Piece | What it does |
| --- | --- |
| `routing::ChainProblem` | `width`, `bound(layer, from, to)`, `step(prefix, to, redone)`, `budget`, `found` |
| `routing::ChainSolution` | `chosen`, `cost`, `expansions`, `routed`, `reached`, `solved`, `optimal`, `outOfTime`, `relaid` |
| `routing::solveChainAStar` | the search: the arena, the heuristic, the queue, the budget |
| `Driver::optimizeChainsPrefix` | **one pass over the chains, no rounds** |
| `Driver::solveChainAStar` | one chain: the bounds built, `step` supplied, the end-pair reorder, the winner's ways read back |
| `Driver::chainAStar` | the `SCPD_CHAIN_ASTAR` gate, on by default |
| `Driver::chainAStarBudget` | `SCPD_CHAIN_ASTAR_SECONDS`, 180 s, per attempt |
| `Driver::CHAIN_FRESH_EDGES` | how many edges at each end are never remembered |
| `Driver::prefixFence_` | the ways the caller has already laid, which `corridorOfEdge` closes |
| `Driver::openOptionsOf` | which options a waypoint offers, factored out of `solveChain` |
| `Driver::portOfOption` | the port an edge leaves a waypoint by or arrives at it by |

- **`prefixFence_`** replaces the single `aheadFence_` the second order added:
  a list of ways that `corridorOfEdge` closes with
  `alongDisc(..., stencilFor(tuning_.clearance), ...)` immediately after
  `fenceCommittedEdges`, exactly where `aheadFence_` was closed. The
  second-order trellis puts one way in it and the prefix search every edge of
  the prefix.
- **`edgeCost` takes the fence and a `remember` flag**, and `remember` is
  false for the four edges at the ends of a chain and for both routes of a
  reorder. `edgeMemoKey` and its `ahead` field are otherwise untouched, so the
  trellis reads and writes what it always did.
- **The winner's ways** come out of a `std::map` from prefix to
  `{way, before}`, filled as `step` prices each one. `std::map` because the
  key is a vector and because its values never move — the fence holds
  pointers into it.

## The tests

`test/routing/test_chain_search.cpp`, sixteen of them, on the `Table` fixture
of `test_chain_trellis.cpp` with one thing added: a `conflict` that reads the
whole prefix. With no conflict the two searches solve the same problem and
must agree; with one they do not, and that is the reason this search exists.

Two are that reason written down as something that runs.

- **`APrefixThatBlocksItselfCostsAnother`.** The table discounts exactly the
  three steps of one run of choices and charges ten for every other step, so
  that run costs 3 and no other costs less than 21 — and it is the one run the
  conflict forbids. The prefix search answers 21. The trellis answers 3, names
  that run, and the run it names cannot be built at all.
- **`BeatsTheTrellisOnRandomChainsThatBlockThemselves`.** Three hundred small
  random chains with a random rule that makes pairs of choices refuse to stand
  together, each brute-forced over every assignment. The prefix search returns
  the brute-force optimum on every one. The trellis, priced honestly, is above
  it on more than a tenth of the chains it solves at all.

The rest hold the contract: an exact bound prices exactly `L-1` steps, a zero
bound gives the same price and strictly more steps, an unbuildable step is
routed round, a layer nothing reaches is named by `reached`, a chain of one
layer prices nothing, ties break the same way twice, a `redone` price is
counted and clears `optimal`, a budget of a nanosecond claims nothing, no
budget means no limit, every complete run is reported and the cheapest kept,
and on chains where nothing blocks the two searches agree.

## How to run it

```bash
cmake --build --preset release
./build/release/test/routing/mqt-scpd-routing-test \
    --gtest_filter='ChainSearch.*:ChainTrellis.*:AnalyticDubins.*'
uv pip install --python .venv/bin/python --no-build-isolation --no-deps \
    --reinstall-package mqt-scpd -e .

cp benchmarks/17q/config.toml artifacts/17q-dp/config.toml     # ← not optional
.venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml \
    -o artifacts/17q-dp --stage final -v 1
# no time limit, for a chip small enough to prove every chain
SCPD_CHAIN_ASTAR_SECONDS=0 .venv/bin/mqt-scpd plan \
    -c benchmarks/17q/config.toml -o artifacts/17q-dp --stage final -v 1
# 60 s instead of 180 s: 57q in 86 s rather than 201 s, at the same 66 edges
SCPD_CHAIN_ASTAR_SECONDS=60 .venv/bin/mqt-scpd plan \
    -c benchmarks/57q/config.toml -o artifacts/57q-dp --stage final -v 1
# the way back to the trellis, and from there to the greedy
SCPD_CHAIN_ASTAR=0 .venv/bin/mqt-scpd plan -c benchmarks/17q/config.toml \
    -o artifacts/17q-dp --stage final -v 1
```

Report per chip: **feedline edges NOT drawn, angle cost, insertion seconds,
and `steps routed` per chain**, and for every chain whether it read `optimum`,
`best found, an end pair laid the other way round`, or `best in the time
given`.

## What is open

1. **69q loses edges, and one of its chains has no answer at all.** Chain 2
   exhausts every run of options in 52 s: the search saw them all and none
   joins the chain. That is not something a better search reaches — it needs
   either more options at those couplers or a repair at the commit: rip the
   edge that took the room, lay the one that lost, re-route the first.
2. **Merging, measured against the unmerged answer.** The natural first rule
   is the second-order equivalence: prefixes that agree on `(layer, choice
   here, choice before)` are interchangeable for the future, because an edge
   sees only a 250-cell box (`EDGE_BOX_MARGIN`) and at a shared coupler it is
   the immediately preceding edge that reaches into it. That collapses the
   node count to `n²` a layer while the prices still carry the true prefix
   fence. Doing it *after* the unmerged version exists is the whole point: the
   approximation error can then be measured against an exact reference
   instead of assumed. The exact rule, if the second order loses something, is
   geometric: key a node by the prefix's fenced cells that fall inside the
   union of the *remaining* edges' boxes.
3. **`CHAIN_FRESH_EDGES = 2` was not swept.** On 57q's chain 4 the two dearest
   edges by far are the **last two** — 14 400 and 7 200 routes of 24 897 — and
   those are precisely the ones the rule never remembers. Raising the reuse to
   them would collapse that chain; whether it costs an edge is the measurement
   nobody has made.
4. **A chain is still fenced by its own edges as the pass before left them.**
   `looseChain_` is only set under `SCPD_CHAIN_DP=2`, so
   `fenceCommittedEdges` closes this chain's other edges on top of the prefix
   fence. With one pass those are empty for every chain, so it costs nothing
   today — but it would bite the moment a second pass came back.
5. **The chains are independent and are not run in parallel.** With the rounds
   gone this is the largest runtime lever left. It needs one router context
   per thread, since `router_`, `corridor_`, `box_`, `proximity_`, `stencils_`
   and `frame_` are all shared `Driver` state.
6. **The cost still sees only the feedline.** Length, length difference and
   `100 × guarded` are commented out, so the two resonator ports of one
   coupler are an exact tie.

## Traps that cost time

1. **Measure sequentially, and watch what else the machine is doing.**
   `artifacts/` grows to 15 GB and every run rewrites the GDS and SVG in it,
   which sets Spotlight indexing it again. One 69q run came back at **5553 s
   against 922 s** for the identical result. Check with
   `ps -A -o %cpu,comm -r | awk '$2 ~ /mds_stores|mediaanalysisd/ {s+=$1} END {print s}'`,
   and measure CPU time beside wall time — the two agreeing is what says a row
   is clean.
2. **Use `-v 1`.** `-v 2` alone made a 69q run take four times as long.
3. **`plan --stage final` reads the earlier stages' artifacts out of the run
   directory**, so its numbers are not a full pipeline run's. Compare only
   like with like, and copy the config over every time.
4. **The pipeline tests are not a quick check.** `test/pipeline/Benchmarks.hpp`
   reads `refinement_rounds` from the config but **not** `repair_trials` or
   `stop_after`, so `EveryChip/Final.*` runs the full stage at the schema
   default `repair_trials = 100` — over two hours, unfinished.
5. **A chain that says it cannot be joined may only have run out of time.**
   That message was ambiguous for a whole sweep and hid a 69q chain that had
   spent 74 s against a 60 s budget. It now names which of the two it was.
6. **An editor with `FinalRouter.cpp` open can write a stale buffer back over
   your change.** If a switch does not take, check the source before the
   logic.
7. **The configs are in a measuring state** and must go back before a commit:
   `repair_trials = 0` and `stop_after = "couplers"` in all eight.
   `test_the_final_routing_carries_a_snapshot_of_every_phase` fails while they
   are in.

## The pad filter (uncommitted, and keep it)

In `Driver::makeOption`, right after the pad cell set is built: an option is
refused when **what is left of the resonator runs through its own pad**.
Nothing refused it before — `free()` prices another wire's copper but exempts
the resonator's own (`owner != resonator.key`), and the self-intersection test
asks whether the path meets *itself*, not whether it meets the body.

It does not usually reject an option outright; `couplerPlace` walks on to the
next place along the resonator, so the coupler moves rather than disappearing.
Measured against the trellis: six chips unchanged, **57q 5 undrawn → 2 and
2641 s → 357 s**, 69q 10 → 11 undrawn and 366 s → 1023 s. An option whose pad
is crossed by its own resonator is not buildable, whatever a metric says
somewhere.
