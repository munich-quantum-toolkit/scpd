/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/FinalRouter.hpp"

#include "DebugSvg.hpp"
#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/PortBands.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/pipeline/CapacityFlow.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/CouplerSession.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/routing/AnalyticDubins.hpp"
#include "mqt-scpd/routing/ChainSearch.hpp"
#include "mqt-scpd/routing/ChainTrellis.hpp"
#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/MeanderInsertion.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/RoomRules.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <queue>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbc = flatbuffers::config;
namespace fbd = flatbuffers::design;
namespace fbg = flatbuffers::geometry;

using routing::Heading;
using routing::MovePrimitives;
using routing::Path;
using routing::PathPoint;
using routing::RoutingObjective;

/// No wire holds this cell.
constexpr std::uint32_t NO_OWNER = std::numeric_limits<std::uint32_t>::max();

/// Whether an environment switch carries a value at all.
///
/// **An empty value is not a value.** `SCPD_CHAIN_SOLO=` left in a shell or a
/// script is a leftover, not a decision, and `std::atoi("")` is zero — so
/// read as a number it turns a default **off** and says nothing about having
/// done so. Every switch below goes through this.
[[nodiscard]] inline const char* envSet(const char* name) {
  const char* value = std::getenv(name);
  return (value != nullptr && *value != '\0') ? value : nullptr;
}

/// An on/off switch from the environment, `fallback` when it carries no
/// value.
[[nodiscard]] inline bool envFlag(const char* name, const bool fallback) {
  const char* value = envSet(name);
  return value != nullptr ? std::atoi(value) != 0 : fallback;
}

/// A whole-number switch from the environment, `fallback` when it carries no
/// value.
[[nodiscard]] inline int envWhole(const char* name, const int fallback) {
  const char* value = envSet(name);
  return value != nullptr ? std::atoi(value) : fallback;
}

/// A real-number switch from the environment, `fallback` when it carries no
/// value.
[[nodiscard]] inline double envReal(const char* name, const double fallback) {
  const char* value = envSet(name);
  return value != nullptr ? std::atof(value) : fallback;
}

/// The bend radius of the primitives, in cells.
///
/// It is a property of the move set and not a knob: the primitive tables, the
/// swept-cell tries and the search state are all built for one radius, and the
/// design rule it has to clear is `min_bend_radius`, which is 50 layout units
/// against a router cell of about ten. The prototype uses five everywhere.
constexpr std::uint8_t BEND_RADIUS = 5;

/// How wide a resonator's copper stands in the way of a feedline edge, in
/// cells, past the lead that carries the clearance (user, 2026-10-03).
///
/// Two cells says only one thing, and it is the thing that matters there: an
/// edge may lie beside a resonator far from any coupler — they have nothing
/// to do with each other, the feedline pass draws the resonator again, and
/// fencing the whole length at the clearance is what left edge f1 of chain 0
/// on 69q without a way — but it may not run **across** it.
constexpr std::uint32_t RESONATOR_COPPER = 2;

/// How much of `target_resonator_length` is left to run from the centre of
/// the coupler to the qubit, as a share of that figure.
///
/// **Short of the design figure on purpose**, and the reason is that the
/// meander can only lengthen a way, never shorten one: a way the coupler
/// leaves longer than the target is a `long` verdict nothing in the stage
/// can mend, where one left short is what the meander is for. Cutting at
/// `bias · target` puts the coupler that much further up the way and hands
/// the rest to the meander.
///
/// A switch since 2026-10-05 (user), because the feedline sweep makes
/// lengths again from then on and the `long` fails are what it cannot fix.
/// `couplerPlace` subtracts the lead from the figure, so the place is where
/// the lead and what is left together come to `bias · target`.
[[nodiscard]] inline double couplerBias() {
  static const auto bias =
      std::clamp(envReal("SCPD_COUPLER_BIAS", 1.0), 0.5, 1.0);
  return bias;
}

/// How far below `target_resonator_length` a coupler may leave the
/// resonator, as a share of that figure. Three tenths by default, so a chip
/// asking for 6000 accepts 4200 and refuses 4199.
///
/// **A tenth was measured and was too tight.** It caught what it was for —
/// 69q's 1455-unit resonator — but it also took 15 % of every coupler's
/// options away, left no coupler on 69q with all 48, and cost 17q two
/// feedline edges and ten times its insertion time (2.1 s against 26.8 s).
/// Three tenths refuses the case that matters by a wide margin, 1455 against
/// a floor of 4200, and leaves far more of the option space standing
/// (user, 2026-09-30).
///
/// **Why an option needs refusing for this at all.** `couplerPlace` offers
/// every cell of the way, ordered by how near it leaves the design figure,
/// and each orientation walks that list until one place *fits*. A place that
/// fails a legality rule is skipped, and the walk carries on down the
/// list — away from the figure. Nothing stopped it: only an overshoot was
/// refused, and any undershoot at all was accepted. On 69q that put a
/// coupler far enough along a resonator to leave **1455 units of a 6000-unit
/// resonator**, which no later phase recovers: the meander can lengthen a
/// way, but not by four times.
///
/// `SCPD_COUPLER_MAX_SHORTFALL` sets it, as a share and not a percentage.
[[nodiscard]] inline double couplerMaxShortfall() {
  static const double share = [] {
    return std::clamp(envReal("SCPD_COUPLER_MAX_SHORTFALL", 0.30), 0.0, 1.0);
  }();
  return share;
}

/// Whether the sweep under the feedline constraints takes the shape the
/// prototype's `run_final_routing_feedline` gives it, rather than the shape
/// of the outer routing it otherwise shares every line with.
///
/// The prototype's own comment says the feedline pass reroutes every wire
/// "genau wie run_final_routing()" and only adds the orthogonal crossing
/// constraints. Its code says five more things (`FinalGrid.cpp:4254`,
/// `:4262`, `:4437`, `:4612`, `:10677`), and they are what this switch
/// turns on:
///
/// 1. **The band grows with the round**, `(r + 1) * expansion`, where the
///    outer routing holds it at `expansion`. This is the escape valve a
///    wire has when its own band is the thing hemming it in.
/// 2. **Four neighbours are fenced, not two**: `i ± 1` and `i ± 2`.
/// 3. **The lane polygon is built from other paths than the two
///    neighbours** — see `Driver::laneOf`, which is the whole of it.
/// 4. **The relaxation prices its discs at one clearance**, where the outer
///    routing grows them by the level.
/// 5. **The crossing rule is built from the edges between couplers only**,
///    the terminal ones left out.
///
/// And it drops the one thing that is ours and not the prototype's: the
/// ten-fold price on the approaches of the wires around this one.
[[nodiscard]] inline bool feedlineLikePrototype() {
  static const bool on = envFlag("SCPD_FEEDLINE_PROTOTYPE", true);
  return on;
}

/// Whether the straight runs at a coupler are measured along the heading
/// they run on rather than along an axis. `=0` restores the figures that
/// stood before, for measuring against them.
[[nodiscard]] inline bool couplerStubs() {
  static const bool on = envFlag("SCPD_COUPLER_STUBS", true);
  return on;
}

/// Whether every wire of the feedline pass is drawn again at least once,
/// rather than keeping a way that already holds every rule.
///
/// The way a resonator holds when the pass begins is not a way this stage
/// drew. It is the coupler's lead, and behind the lead's tip the route the
/// outer routing left, spliced on at the insertion point. The two meet at
/// the right cell but not on the same heading, so the join is a kink — a
/// corner no search would ever have returned, and one the curvature rule
/// does not admit. `isLegal` does not look for it: it asks after the room
/// against other wires, the crossing rule and the length, and a kinked way
/// answers all three. So the wire was settled on its first round and never
/// searched, and the kink stood to the end of the run. Measured on 17q,
/// every one of the five resonators on a diagonal coupler kept its way in
/// all six rounds (user, 2026-09-30).
///
/// Forcing the draw costs the searches the shortcut saved. `=0` is the way
/// back.
[[nodiscard]] inline bool forceReroute() {
  static const bool on = envFlag("SCPD_FEEDLINE_FORCE_REROUTE", true);
  return on;
}

/// Whether a resonator is meandered in the feedline **sweep**.
///
/// **On since 2026-10-05** (user), where it had been off since
/// 2026-09-30. It was off because in the exact regime the only tool is the
/// meander, which adds, so a way that comes out too long can only be
/// refused as no way — and a resonator refused in the sweep goes back on
/// its seed and costs the whole round. What answers that is not switching
/// the insertion off but giving the sweep a wide band and leaving the rest
/// to the refinement, which is the prototype's own arrangement: see
/// `sweepLengthBand`.
///
/// Measured over the eight benchmarks (`artifacts/logs/sweep6` against
/// `base6`): the resonators off their target length go from 102 to 6 and
/// `bad` from 8 to 2, for about twice the runtime. `=0` leaves the sweep
/// routing alone, as it was; the refinement then makes the lengths by
/// itself under `SCPD_REFINE_MEANDER`.
[[nodiscard]] inline bool feedlineMeander() {
  static const bool on = envFlag("SCPD_FEEDLINE_MEANDER", true);
  return on;
}

/// How many cells at the start of a resonator's way the meander leaves
/// alone, and what the two legs of its loop keep between them.
///
/// Both were derived rather than chosen, and on the 4-qubit chip the two
/// together made the insertion arithmetically impossible: wire 4's way is
/// 111 cells, of which 70 lie on a straight run; the margin reserved the
/// first 35 of them and a loop needs `legSpacing + 2 · BEND_RADIUS` = 35
/// cells of span between its two ends, which the 34 left over cannot give.
/// Every one of the 4760 placements it tried was refused for that and for
/// nothing else — not one for want of room, a closed cell or a crossing
/// (`artifacts/logs/refine-probe/4q-diag.log`).
///
/// **The margin is `couplerLength + 4` now**, where it was
/// `couplerLength + straightStart + 4`. What it is for is to keep the loop
/// out of the run the resonator couples along, which is the coupler's own
/// length; the straight start was the second run after the lead, and
/// `SCPD_RESONATOR_STUB` took that away on 2026-10-03 without this figure
/// following. The four cells are the prototype's own start margin.
[[nodiscard]] inline std::uint32_t meanderStartMargin(
    const std::uint32_t couplerLength, const std::uint32_t straightStart) {
  static const auto asked = envWhole("SCPD_MEANDER_START_MARGIN", -1);
  if (asked >= 0) {
    return static_cast<std::uint32_t>(asked);
  }
  static const bool withStub = envFlag("SCPD_MEANDER_MARGIN_STUB", false);
  return couplerLength + (withStub ? straightStart : 0U) + 4U;
}

/// What the two legs of a loop keep between them, in cells.
///
/// The prototype's `min_straight_length`, which is **not** the design rule
/// of that name (decision 0019 keeps the two apart): the rule is 100 layout
/// units, about ten cells, and the figure here is what holds the two legs of
/// one resonator apart. 25 is the prototype's. The floor that means
/// something is the wire clearance, 19 cells — two legs of one wire are not
/// a pair the design-rule check looks at, so nothing below the clearance is
/// defensible on its own.
[[nodiscard]] inline std::uint32_t meanderLegSpacing() {
  static const auto cells = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_MEANDER_LEG_SPACING", 25), 4, 200));
  return cells;
}

/// How wide the length band of the feedline **sweep** is, as a multiple of
/// `resonator_length_tolerance`.
///
/// **5** (user, 2026-10-05): the meander goes back on in the feedline pass,
/// and it aims roughly rather than exactly there; the refinement tightens
/// what is left. This is the prototype's own arrangement, and it is not a
/// knob it has to tune: its sweep calls `meander_insertion_proximity` and
/// its refinement `meander_insertion_proximity_strict`
/// (`FinalGrid.cpp:11809` against `:13570`), so the sweep aims at *at
/// least* the length and nothing there can be too long, where the strict
/// insertion aims at the length itself.
///
/// What it buys: in the exact regime the only tool is the meander, which
/// adds, so a way that comes out long can only be refused as no way. A
/// resonator refused in the sweep goes back on its seed and the whole round
/// is spent on it. A wide band lets the sweep place a loop that is roughly
/// right and leaves the last few hundred units to the phase whose business
/// they are.
///
/// The band the **stage is judged by** is not this one: `failsOf` counts
/// short and long against `resonator_length_tolerance` itself, whatever the
/// sweep allowed itself. `=1` is the band the sweep had.
[[nodiscard]] inline double sweepLengthBand() {
  static const auto band =
      std::max(1.0, envReal("SCPD_SWEEP_LENGTH_BAND", 5.0));
  return band;
}

/// Whether the **refinement** of the feedline routing makes a resonator its
/// length, whatever `SCPD_FEEDLINE_MEANDER` says about the sweep.
///
/// **On** (2026-10-05). The fifth phase is where the lengths are finally
/// read, and the two passes want different answers: the sweep is judged by
/// whether a resonator can be routed at all, the refinement by whether it
/// can be made its length. One switch over both passes could not say that,
/// and the cost of the sweep's meander is not small — measured on 17q, the
/// sweep with the meander takes 21.5 s to 72.2 s and opens the 30/31 pair,
/// for 4 of the 7 short resonators; the refinement closes 4 of them in
/// 11 s more and opens nothing (`artifacts/logs/refine-probe`).
///
/// `=0` leaves the refinement routing alone, as it was while
/// `feedline_refinement_rounds` was 0.
[[nodiscard]] inline bool refineMeander() {
  static const bool on = envFlag("SCPD_REFINE_MEANDER", true);
  return on;
}

/// Whether a refinement attempt whose meander found no room is searched
/// once more with the room price taken off.
///
/// **On**, and it is the prototype's own fallback
/// (`FinalGrid.cpp:13600`): the clearance-maximising way is a detour, and a
/// detour is long — long enough that the meander has no slack left and in
/// the exact regime long enough to be refused outright. Zeroing the wire
/// proximity field leaves the hard corridor standing and gives a direct way
/// with room to pad. Only for a resonator that has never reached its length
/// in this phase (the prototype's `fitted`): one that has will roll back
/// onto a way that is still at its length, so a second search buys nothing.
///
/// What it is for, measured on 17q before it existed: of 34 resonator
/// attempts in two refinement rounds, **12 were refused as too long for
/// their target** and 15 found no room for a loop.
[[nodiscard]] inline bool refineFallback() {
  static const bool on = envFlag("SCPD_REFINE_FALLBACK", true);
  return on;
}

/// Whether the refinement's room price is built **after** the feedline
/// constraints rather than before them.
///
/// **On**. `priceRoom` is the chamfer distance to the middle of the channel
/// — the prototype's `compute_corridor_proximity_decay` — and it reads the
/// corridor. Built before `constrainByFeedlines`, it reads a corridor in
/// which the fences of the chain edges and the terminal edges' coupler runs
/// are not yet closed, so the field pulls a wire into the middle of a
/// channel whose feedline walls it cannot see. The prototype computes the
/// decay on the fully marked buffer. `=0` is the order the phase was
/// written with.
[[nodiscard]] inline bool refinePriceLast() {
  static const bool on = envFlag("SCPD_REFINE_PRICE_LAST", true);
  return on;
}

/// How many pairs of ring neighbours the refinement fences.
///
/// The prototype marks **i ± 1 … i ± 4** as hard obstacles at
/// `ref_min_clearance`, which is the design rule itself
/// (`FinalGrid.cpp:13410`); the phase was written with one pair, where the
/// feedline sweep fences three (`SCPD_FEEDLINE_FENCE_PAIRS`). A way taken
/// here is tested against every other wire afterwards, so a loose fence
/// does not break the rule — it wastes searches on ways that are then
/// refused.
[[nodiscard]] inline std::uint32_t refineFencePairs() {
  static const auto pairs = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_REFINE_FENCE_PAIRS", 3), 1, 8));
  return pairs;
}

/// Whether the four checks of the coupler insertion are said again at the
/// end of the refinement.
///
/// **On**. They run at the end of `insertCouplers` and the crossing check
/// once more after the repair, but the refinement **moves feedline edges**
/// — 7 of 20 on 17q — and nothing looked at the result: `failsOf` sees a
/// conventional wire crossing an edge and not an edge crossing an edge.
[[nodiscard]] inline bool refineChecks() {
  static const bool on = envFlag("SCPD_REFINE_CHECKS", true);
  return on;
}

/// Whether a price stamped onto the field adds to what is already there
/// rather than replacing it.
///
/// The lane fill lays the ground — the price outside the polygon, nothing
/// inside — and every disc stamped after it used to **overwrite**. Both
/// carry the same figure, so a cell outside the lane and a cell in the halo
/// of the wire just let go of came to exactly the same 7, and the search had
/// nothing to tell them apart: it took the shortest way through, because
/// every way through cost the same (user, 2026-10-01).
///
/// Adding them keeps the ground and raises it where something else is near,
/// so the halo of a released wire is dearer than open ground, and dearer
/// again where it meets a feedline. Clamped at `CEILING`.
[[nodiscard]] inline bool addTheprices() {
  static const bool on = envFlag("SCPD_PRICE_ADD", true);
  return on;
}

/// Whether a resonator between two conventional wires has its two
/// neighbours cut to the stretch that runs alongside it.
///
/// **This is not the prototype's.** Its case distinction corrects the lane
/// only where the neighbour is itself a feedline or a resonator; where both
/// neighbours are conventional it leaves them whole. A resonator is the one
/// wire that makes that a bad lane: it begins at its **coupler**, out in the
/// fan-in, and ends at its qubit, while the two neighbours begin and end at
/// their own ports somewhere else entirely. The polygon then closes with two
/// long straight jumps — coupler to the start of one neighbour, qubit to the
/// end of the other — and the region it frees is not the room the resonator
/// has but a wedge cutting across the ring (user, 2026-10-01).
///
/// Cut to the stretch between the cells nearest the resonator's own two
/// ends, the jumps are short and the lane is the corridor the wire actually
/// runs in. `=0` restores the prototype's shape.
[[nodiscard]] inline bool trimTheLane() {
  static const bool on = envFlag("SCPD_LANE_TRIM", true);
  return on;
}

/// What the halo of a wire let go of costs, as a multiple of the wire price.
///
/// One was the figure the lane fill already lays on open ground, and once the
/// prices add up that is enough to tell the halo from the ground beside it.
///
/// **Raising it was measured and made things worse** (33q, the crossing rule
/// off): 13 fails at one, 16 at two, 20 at three, 15 at four, and the open
/// wires went the wrong way, 2 -> 6 -> 8 -> 4. A wire let go of is drawn
/// again a moment later, so pricing its room steers the wire being drawn off
/// a lane that is about to be free and onto one that the next wire then
/// wants. The figure is not monotone either, which is the mark of a knob
/// that shuffles the order rather than one that reaches a cause
/// (user, 2026-10-01).
/// How far the halo of a wire reaches, as a multiple of the wire clearance,
/// and how its price falls over that reach.
///
/// **A flat disc of one clearance is the shape that fails.** Measured on a
/// 33q search: of the 77 587 cells the band lets a wire enter, 59 424 — more
/// than three quarters — carry the same price. A constant over nearly
/// everything is not a steer; it adds the same to every way and leaves only
/// the length to decide, which is why the search takes the shortest way and
/// looks past the room its neighbour needs. And the disc is a step: at its
/// rim nothing opposes a crossing, so the cheapest way is often over the
/// wire, along the cheap side and back over, which is the one shape a
/// rip-up and re-route must not be taught (user, 2026-10-01).
///
/// A reach of several clearances with the price falling over it fixes both:
/// far out it is cheap, so the field stops blanketing, and the peak sits on
/// the wire, so a crossing pays the most exactly where it crosses — twice,
/// for a way that comes back.
///
/// The prototype has no such halo in its feedline pass. It has the decay
/// (`compute_proximity_grid_decay`) and uses it in the resonator refinement
/// at a reach of 60 cells and a peak of 35, which is where these figures
/// come from, but nothing of the sort steers the sweep there. This is ours.
///
/// `SCPD_HALO_REACH` is the reach in clearances, `SCPD_HALO_DECAY` the shape:
/// 0 flat, as it was, 1 linear to nothing at the rim, 2 exponential.
///
/// **1 since 2026-10-05** (user), which with the decay off is the
/// prototype's own disc of one clearance around a wire let go of. Measured
/// on 69q with the terminal stub guard and three fenced pairs: reach 3
/// `bad` 5 (183/184 and 190/191 open, 191 crossing), reach 1 `bad` 4
/// (183/184 open, 191 and 15 crossing at the halo's edge), 550 s against
/// 576. `=3` is the reach that stood from 2026-10-01 to 2026-10-05.
[[nodiscard]] inline std::uint32_t haloReach() {
  static const auto reach = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_HALO_REACH", 1), 1, 12));
  return reach;
}
/// **Off by default.** A falling halo is a mitigation, not a cure: it is
/// still a price per cell, and a price per cell can always be paid once and
/// recouped on the other side. What it cannot express is "do not cross",
/// which is the thing actually wanted (user, 2026-10-01).
[[nodiscard]] inline int haloDecay() {
  static const auto shape = std::clamp(envWhole("SCPD_HALO_DECAY", 0), 0, 2);
  return shape;
}

/// The most a cell of the wire proximity may hold.
///
/// The layer is sixteen bits now, but the search adds `PENALTY_SCALE * value`
/// into a `uint32_t` cost that also carries the whole way behind it, so the
/// value needs a roof. Four thousand is 400 000 a cell, about twenty-seven
/// bends on a 33-qubit grid, and even a way made entirely of such cells
/// stays an order of magnitude below where the cost would wrap.
constexpr std::uint32_t CEILING = 4000;

/// The toll for crossing a wire that was let go of, and how wide the band
/// that collects it is.
///
/// A price over an area cannot say "do not cross": a way pays it on the far
/// side and earns it back there, and the dearer the area the more attractive
/// it becomes to hop the wire, run on the cheap side and hop back — the one
/// shape a rip-up and re-route must not be taught (user, 2026-10-01).
///
/// A toll says it. Laid on the wire's own centre line and a cell or two
/// either side, it is paid by a way that crosses and by no way that does
/// not, and a way that comes back pays it twice. The search charges
/// `PENALTY_SCALE * value * (1 + swept)`, so 127 on 33q is about 12 700 for
/// the crossing move against 14 950 for a bend: one crossing costs about a
/// bend, two cost two, and going round is cheaper than almost anything else.
///
/// The band has to be wider than one cell. A way may step diagonally from
/// one side of a one-cell line to the other without ever standing on it, and
/// would cross for nothing.
///
/// `SCPD_CROSS_TOLL` is the value, 0 for none; `SCPD_CROSS_TOLL_WIDTH` the
/// radius of the band in cells.
/// **127 was the ceiling and it was too low.** A byte could buy at most
/// 12 700, where one bend on a 33-qubit grid is 14 950 — so the dearest toll
/// expressible was still cheaper than a single corner, and widening the band
/// changed nothing: measured at widths 2, 5, 10 and 19 the ways crossed the
/// released wire exactly 48 times each, and the fails did not move. The
/// layer is sixteen bits now and the toll is given in the same units as the
/// rest, so 150 is about one bend and 1500 about ten (user, 2026-10-01).
[[nodiscard]] inline std::uint32_t crossToll() {
  static const auto toll = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_CROSS_TOLL", 150), 0,
                 static_cast<int>(CEILING)));
  return toll;
}
[[nodiscard]] inline std::uint32_t crossTollWidth() {
  static const auto wide = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_CROSS_TOLL_WIDTH", 2), 1, 10));
  return wide;
}

[[nodiscard]] inline std::uint32_t releasedFactor() {
  static const auto factor = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_RELEASED_FACTOR", 1), 1, 18));
  return factor;
}

/// Whether the wires let go of in a relaxation keep a priced halo. On by
/// default; `=0` is a diagnostic that takes the halo away so a picture shows
/// what is left.
///
/// `stampDisc(way, clearance, price)` paints a band two clearances across —
/// 39 cells on 33q — around each released wire. At a zoom where the wire's
/// own line is off the crop, that band reads as a free-standing block with a
/// rounded edge, which is what a disc stamp leaves. It is a **price**, not a
/// wall: on 33q 7, which is 700 against the 14950 of one bend, so it can
/// make a way lose on cost but can never shut one out (user, 2026-10-01).
[[nodiscard]] inline bool priceTheReleased() {
  static const bool on = envFlag("SCPD_PRICE_RELEASED", true);
  return on;
}

/// Whether the approach band of every routable port is kept in the obstacle
/// mask. On by default, which is what a run does; `=0` is a diagnostic.
///
/// The band is the prototype's `add_fixed_port_obstacles`: a strip one wire
/// spacing wide, `min_straight_length` out of the port and three times that
/// back behind it. On 33q that is 41 by 19 cells on an axis and 29 by 13 on
/// a diagonal, and a diagonal one is the lozenge that shows up in the
/// pictures beside a port.
///
/// It is worth being clear about what it is, because it does not look like
/// what it is: it is **a wall**, stamped into `scene.blocked`, not a price.
/// No search enters it and no tuning of the proximity gets a wire through
/// it; it also carries the full static proximity, because the halo is grown
/// from the same mask. In a picture it is drawn as "artwork and keepout",
/// which is why it reads as copper (user, 2026-10-01).
///
/// Turning it off restores the mask to what it was before the bands, the
/// target cell each one digs excepted — those are kept, so the objectives
/// are unchanged and only the walls are gone.
[[nodiscard]] inline bool portBands() {
  static const bool on = envFlag("SCPD_PORT_BANDS", true);
  return on;
}

/// Whether the approaches of the wires around the one being drawn are
/// priced at ten times the wire price.
///
/// Ours, not the prototype's, and already off under the feedline
/// constraints — `priceLane` returns before it on the prototype path. This
/// switch reaches the one place it still acts, the outer routing, so that
/// what it is worth can be measured without changing what a run does.
/// Default on, which is the behaviour that stands.
///
/// What it paints is a rectangle at **both** ends of every ring neighbour
/// and every wire let go of: a port band long, two wire clearances wide, at
/// `10 x wire price`. On 33q that is 10 by 37 cells at 70, and 70 is 70
/// cells of detour for every cell entered, against 149.5 for a whole bend —
/// by a distance the strongest price in the field.
[[nodiscard]] inline bool priceTheApproaches() {
  static const bool on = envFlag("SCPD_PRICE_APPROACHES", true);
  return on;
}

/// How much further than its own stub a launcher keeps the coupler box away,
/// in layout units (user, 2026-10-02).
///
/// The box is the room the launcher stubs leave open, and it used to be drawn
/// on the stub itself: the straight run of `min_straight_length` the wire off
/// a launcher is forced to make, inflated by the clearance, plus the one cell
/// that makes it the first open cell rather than the last closed one. A
/// coupler is allowed anywhere inside that, so a coupler may sit with its
/// feedline port one cell outside a stub's clearance — and the edge into it
/// then has the stub pressed against its flank for its whole approach. Those
/// are the edges that fail: not because they cross anything, but because
/// there is no room beside the launcher for the run they have to make.
///
/// Ten units on a rule of a hundred, so the box is drawn on a straight run of
/// a hundred and ten. `SCPD_LAUNCHER_LEAD` sets it; `=0` is the figure that
/// stood before.
[[nodiscard]] inline double launcherLead() {
  static const double lead = std::max(0.0, envReal("SCPD_LAUNCHER_LEAD", 10.0));
  return lead;
}

/// Whether the coupler box reserves the quarter turn a wire has to make
/// after its straight run, and how much margin it keeps on top, in cells
/// (user, 2026-10-06).
///
/// The box was drawn tangent to the launcher stubs: `launcherStraight +
/// clearance + 1` from every launcher cell. A wire leaving that launcher is
/// forced straight for `straightStart`, so between the end of its own stub
/// and a feedline edge sitting on the box edge there are
/// `launcherStraight + 1 - straightStart` cells — two on every benchmark.
/// It takes `BEND_RADIUS` to turn out of that channel, so the wire cannot
/// turn at all: it may only cross the edge straight, and if that edge is not
/// its bridge it is fenced and the wire is dead on arrival.
///
/// That is not a corner case, because `edgeInBox` makes the same
/// rectangle the hard corridor bound of every non-terminal edge search: an
/// edge runs *on* the box edge whenever that is the straight line between
/// its two couplers. On 17q f8 is `y` ≡ 77 ≡ `minY` for 396 cells and f2 is
/// `x` ≡ 1347 ≡ `maxX` for 273, and wire 31 — whose launcher sits behind f8
/// — is the wire that cannot leave.
///
/// So the box would keep clear the launcher's own run **or** that run plus a
/// quarter turn, whichever is longer, and then the clearance, the one cell
/// and the margin. On the benchmarks `straightStart + BEND_RADIUS` = 16 is
/// the longer of the two against `launcherStraight` 12, so the reach grows
/// from 32 to 36 + margin.
///
/// **Off (user, 2026-10-06), and it only means anything together with
/// `SCPD_LAUNCHER_FENCE_TURN`.** That pairing is the whole measurement, and
/// each half on its own buys nothing — 17q to `feedlines`, `bad =
/// unrouted + open + crossing`:
///
///   | box turn | launcher fence | bad |
///   | off | off | 4 (14, 31, 33, 56) |
///   | **on** | off | 4 (30, 31 open + 31, 56) |
///   | off | **on** | 4 |
///   | **on** | **on** | **2** (31, 56), and no open wire |
///
/// Why it is off: with the margin taken out again (below) the pair leaves 17q
/// at `bad` **4** — the figure it started from, with other wires (31 and 32
/// open plus 27 and 56 crossing, against 14, 31, 33 and 56 crossing) — and
/// costs 45 % of the run, 54 s against 38. So for that chip it is a trade and
/// not a gain, and the geometry it corrects has to earn its keep somewhere
/// else before it is the default. **What it corrects is still real** and the
/// check says so: the box side was 32 cells from a launcher cell where the
/// wire off it needs 11 straight + 5 turn + 19 clearance = 35, and five of
/// 17q's seventeen couplers sat with their feedline run on that side.
///
/// The reason the box alone failed is that, while no edge was held to it
/// (`edgeInBox` came later), it was not what pinned an *edge* beside a
/// launcher: what the edge searches kept clear is the fence `corridorOfEdge`
/// lays on every launcher that is not their own. That fence was one cell
/// *tighter* than the box, so
/// pushing the box in moved the couplers and the edge hugged the fence
/// instead — on 17q wire 31 stayed dead on arrival against f8 at 18.0 cells
/// where it had been 19.0. Correct both and the channel is the figure the
/// wire needs on both sides of it.
///
/// **A margin on top was built, swept and taken out again** (user,
/// 2026-10-06). It kept a coupler that many cells further off the box side
/// than the rule asks. On 17q to `feedlines` it was a threshold and not a
/// slope — `bad` 4, 5, 4, 4, 2, 1 at margins 0 to 5, the two open wires 31
/// and 32 falling away at 4 and two of three crossings at 5 — so the best
/// figure measured at this stage, `bad` 1, stood on a margin of 5. The rule
/// here is what the geometry asks for and nothing beyond it; whoever wants
/// the margin back wants `holds`'s own `margin` parameter, which is unused,
/// at the four call sites in `terminalsInBox` and `makeOption`.
[[nodiscard]] inline bool couplerBoxTurn() {
  static const bool on = envFlag("SCPD_COUPLER_BOX_TURN", false);
  return on;
}

/// Whether the fence an edge search keeps around every launcher reserves the
/// quarter turn the wire off that launcher has to make, and the margin on
/// top, in cells (user, 2026-10-06).
///
/// `corridorOfEdge` closes, around every launcher but the one the edge docks
/// on, the cell, the straight run of `straightStart` the wire off it is
/// forced to make, and the clearance around both. The first cell an edge may
/// then use is `straightStart + clearance` away — and the wire whose room
/// that is needs `BEND_RADIUS` more, because at the end of its forced
/// straight it has to *turn* to go anywhere but across the edge. If that
/// edge is not the wire's bridge it is fenced at the clearance and the wire
/// has no first step at all.
///
/// On 17q that is wire 31: its launcher is (361,46), its stub ends at
/// (361,57), f8 runs at y=77, and (361,58) is at exactly 19.0 cells —
/// closed to the search and acceptable to every judge. The fence reserves
/// 11 + 19 = 30 cells where the wire needs 11 + 5 + 19 = 35.
///
/// **Off (user, 2026-10-06)**, with `SCPD_COUPLER_BOX_TURN`, which is the
/// only switch it means anything beside — the table there is the
/// measurement for both. On its own it removes wire 31's dead on arrival and
/// changes no judged figure; with the box and no margin the pair leaves 17q
/// at the `bad` it started from and costs 45 % of the run.
[[nodiscard]] inline bool launcherFenceTurn() {
  static const bool on = envFlag("SCPD_LAUNCHER_FENCE_TURN", false);
  return on;
}

/// How many cells further in from every side of the coupler box a coupler
/// has to stay: `SCPD_COUPLER_BOX_MARGIN`, 5 (user, 2026-10-08; `=0` is the
/// box as it stood). The pad, the feedline's run along it beyond the ports,
/// the resonator's lead and the turn after it (`makeOption`) are held to the
/// box shrunk by this margin, so that the couplers stand further off the
/// launcher stubs.
///
/// It acts on the places of the couplers and nowhere else: an edge between
/// two couplers still has the whole box (`SCPD_EDGE_IN_BOX`), and the
/// edges' fence in front of the launchers is `SCPD_EDGE_LAUNCHER_MARGIN`.
/// Read at every test, so that a run can set it for one chip.
[[nodiscard]] inline std::int64_t couplerBoxMargin() {
  return static_cast<std::int64_t>(
      std::clamp(envWhole("SCPD_COUPLER_BOX_MARGIN", 5), 0, 500));
}

/// How many cells further than the run of `launcherFenceTurn` a feedline
/// edge keeps closed in front of every launcher: `SCPD_EDGE_LAUNCHER_MARGIN`,
/// 5 (user, 2026-10-08; `=0` is the fence as it stood). The run off the
/// launcher is lengthened by this margin, with the clearance disc around the
/// whole of it, so that an edge passes the launcher stubs further off.
///
/// It acts in the corridor of a feedline edge's search (`corridorOfEdge`)
/// and nowhere else: the searches and the commit of the coupler insertion,
/// the commit's test of a way, and the repair's searches of an edge. The
/// coupler box, the wires' own routing and the capacity analysis do not see
/// it. Read at every corridor, so that a run can set it for one chip.
[[nodiscard]] inline std::int64_t edgeLauncherMargin() {
  return static_cast<std::int64_t>(
      std::clamp(envWhole("SCPD_EDGE_LAUNCHER_MARGIN", 5), 0, 500));
}

/// Whether an edge between two couplers stays inside the coupler box, the
/// rectangle the launcher stubs leave open and every coupler stands in:
/// `SCPD_EDGE_IN_BOX`, **on** (user, 2026-10-07). Everything outside the
/// box is closed to the edge's search, a hard bound and not a price. The
/// starting and ending edges of a chain are exempt: they come from a
/// launcher on the border and have to cross the room between the stubs to
/// reach their first coupler.
[[nodiscard]] inline bool edgeInBox() {
  static const bool on = envFlag("SCPD_EDGE_IN_BOX", true);
  return on;
}

/// Whether an edge of a chain keeps the full clearance from the resonator of
/// the coupler it serves (user, 2026-10-02).
///
/// It kept two cells of copper before, which says only that the edge may not
/// run *through* the resonator. Two cells is far short of the rule, so the
/// insertion was free to lay an edge right alongside a resonator — and the
/// resonator is then the wire that pays: when the feedline pass draws it
/// again it has to hold `min_wire_spacing` to that edge, which is a wire
/// that no longer moves. The room was taken before the wire that needs it
/// was ever asked.
///
/// So the resonator is stamped with the clearance stencil like every other
/// obstacle. What stays open is what the edge cannot do without: the
/// straight run it is forced to make off its source and the one it is forced
/// to arrive on, which are the runs `startStraightLength` and
/// `endStraightLength` hold it to. At the coupler those runs lie along the
/// pad, four cells from the resonator's lead — inside the clearance by a
/// long way, and the coupling is exactly that closeness.
///
/// `=0` restores the two cells of copper.
[[nodiscard]] inline bool edgeKeepsResonatorClear() {
  static const bool on = envFlag("SCPD_EDGE_RESONATOR_CLEARANCE", true);
  return on;
}

/// Whether a coupler body and a feedline are kept out of one another (user,
/// 2026-10-02).
///
/// Two halves of one rule, and both were missing:
///
/// 1. **No edge runs through a body.** The bodies used to be closed in the
///    corridor and were opened again when the band was; since then an edge
///    has been free to cross any pad on the chip, its own two included. The
///    bodies that stand are closed again here, and so are the two the edge
///    is being drawn between, which are not in `bodies_` yet while a chain
///    is still searching itself. The feedline ports sit one cell outside the
///    pad by construction, so closing it costs the edge nothing it needs.
/// 2. **No body is laid across an edge.** `makeOption` refuses a body that
///    meets the artwork or a port's approach and only *prices* one that
///    meets a wire, at a figure a turn outweighs. A feedline is not a wire
///    that is drawn again afterwards — it is the thing the resonators are
///    being attached to — so an option whose pad covers one is closed
///    instead of priced, in `openOptionsOf`.
///
/// Where the guard would leave a coupler no option at all it is not applied:
/// a chain with an empty layer has no answer, and a pad on a feedline is
/// still better than no coupler.
///
/// `=0` is the behaviour that stood.
[[nodiscard]] inline bool guardTheBodies() {
  static const bool on = envFlag("SCPD_BODY_GUARD", true);
  return on;
}

/// Whether an edge of one chain is routed against the feedlines of the
/// others.
///
/// **Off** (user, 2026-10-02): one feedline at a time, and a conflict with
/// another feedline is not this search's business. The chains are settled
/// one after another, so fencing them against each other makes the order
/// decide who gets the room — the first chain has the run of the chip and
/// the last works around everything — and the late chains are where the
/// edges without a way were. The feedline pass draws every edge again
/// afterwards, against all of them at once and with the rip-up to move
/// them; that is where a crossing between two chains belongs.
///
/// `chainSolo` already said this for the prefix search and measured it
/// there. This is the same thing said for the commit and for every other
/// search of an edge.
///
/// What still stands: the edge's **own** chain, every resonator the corridor
/// closes, the launcher stubs, the bodies and the artwork. It is only the
/// other chains' copper that is opened.
///
/// `=1` fences them again, which is the behaviour that stood.
[[nodiscard]] inline bool edgesSeeOtherChains() {
  static const bool on = envFlag("SCPD_EDGE_SEES_CHAINS", false);
  return on;
}

/// Whether the first and the last edge of a chain are an obstacle to every
/// edge of that chain, or only to the one beside them (user, 2026-10-02).
///
/// **Only to the one beside them.** A terminal edge has the least freedom of
/// any edge on the chip: it leaves a launcher on the launcher's heading and
/// cannot yield, so it takes the room it takes and the rest of the chain has
/// to live with it. That is right for its neighbour, which shares the ground
/// at the launcher and really does have to work around it. For an edge four
/// waypoints along it is not a conflict at all — it is a wall standing where
/// the two were never going to meet, and the chain pays for it in bends.
///
/// The prototype goes the other way and hard-fences the terminal edges for
/// everyone (`forbidden_paths_start_end`). This is the opposite reading, and
/// it is the one being measured.
///
/// `stateFaults` is exempt: its question is what the commit will find on the
/// whole chip, so it fences the terminal edges for everything.
///
/// `=1` fences them for the whole chain, which is the behaviour that stood.
[[nodiscard]] inline bool terminalEdgesFenceAll() {
  static const bool on = envFlag("SCPD_TERMINAL_EDGES_FENCE_ALL", false);
  return on;
}

/// Whether a chain edge keeps the clearance from **every coupler lead** on
/// the chip, and not only from the two it is drawn between (user,
/// 2026-10-02).
///
/// **The lead and not the whole resonator** (user, 2026-10-02). The lead is
/// the quarter turn a resonator leaves its coupler on and the straight run
/// after it — `option.arc`, which is the head of the resonator's way — and
/// it is the stretch that belongs to the coupler. Past it the resonator is
/// a wire like any other, drawn again in the feedline pass and free to be
/// crossed there.
///
/// Fencing the whole resonator was measured and it blocks edges that are
/// plainly routable: on 69q edge f1 of chain 0 runs from (1835,4440) to
/// (1904,4308), and its own resonator crosses that strip at x≈1874 — a
/// hundred and thirty cells from the coupler, where the two have nothing to
/// do with each other. No slot reaches it, because a slot runs along the
/// terminal's heading and the room the edge wants there is sideways.
///
/// No exemption: every lead binds on every edge, its own two included.
///
/// What makes it affordable is that it is not stamped per search.
/// `ensureForeignRoom` builds one grid of bits for every lead outside the
/// chain being routed and the corridor reads it in the pass it already
/// makes over the grid; this chain's leads move while it is being solved,
/// so those are stamped per edge.
///
/// `=0` is the behaviour that stood.
[[nodiscard]] inline bool edgeKeepsEveryResonatorClear() {
  static const bool on = envFlag("SCPD_EDGE_ALL_RESONATORS", true);
  return on;
}

/// How long the slot is that is freed again at each terminal of a chain
/// edge, in cells (user, 2026-10-02).
///
/// The construction this belongs to has three steps and no fourth:
///
/// 1. Everything within the clearance of a resonator — its way, which
///    carries the coupler's lead as its head — is closed.
/// 2. A straight slot of this many cells is freed again at the edge's two
///    terminals, so that a feedline can dock on the coupler.
/// 3. **No other exemption.** The coupling zone around an anchor used to be
///    one, and it is gone: the slot is the whole of what the coupling needs.
///
/// Twenty is the prototype's figure, where it is the same number twice —
/// `ending_straight_length = 20` at a coupler, and `free_terminal_stub(...,
/// 20)` re-opening exactly the stub the router is then held to
/// (`FinalGrid.cpp:4963`, `:20160`). Its comment says why it has to exist at
/// all: *"without freeing a short straight lead-in/lead-out stub at each
/// terminal, that inflation can seal off the corridor immediately at the
/// port and the search finds no route at all."*
///
/// The artwork is not freed with it. A forced run that lies on copper is an
/// edge that cannot be built, and opening it would hide that rather than
/// mend it.
/// Whether only the coupler's lead is fenced, rather than the whole
/// resonator hanging off it (user, 2026-10-02).
///
/// The lead is the quarter turn and the straight after it, `option.arc`,
/// which is the stretch that belongs to the coupler; past it a resonator is
/// a wire like any other.
///
/// **On by default** (user, 2026-10-03), because it is the only setting
/// under which **every feedline edge of every chip is drawn**: 81 of 81 on
/// 69q, where fencing the whole resonator leaves edge f1 of chain 0 without
/// a way — its own resonator crosses the strip it needs a hundred and
/// thirty cells from the coupler, where the two have nothing to do with
/// each other, and no terminal slot reaches that, because a slot runs along
/// the heading and the room wanted there is sideways.
///
/// **What it costs**, measured over the eight chips against fencing the
/// whole resonator, both with `SCPD_FENCE_LATER_EDGES=0`:
///
///                 edges       open   fails   check
///     lead        all drawn    100     199      34
///     whole       1 missing     52     160       0
///
/// Almost all of it falls on 69q, which trades its last edge for 37 open
/// wires, 17 -> 54. The reason is not the insertion but what follows it: a
/// feedline laid across a resonator far from any coupler is a wire the
/// feedline pass then has to draw around, and often cannot.
///
/// So this is a deliberate choice of *every edge drawn* over *fewest open
/// wires*, and `=0` is the other one.
/// Whether an edge of a chain is fenced against the edges of its own chain
/// that come **after** it (user, 2026-10-02).
///
/// The prefix search prices edge `k` against the edges `0..k-1` and against
/// nothing else, because the later ones do not exist yet when it is priced.
/// The commit then routes edge `k` against all of them — the later ones
/// stand on the ways the search found for them, and `edgeWayOf` hands them
/// out. So a run of options the search proved complete can be one the
/// commit cannot build, and `stateFaults` is the count of exactly that.
///
/// Seen on 45q under `SCPD_EDGE_LEAD_ONLY`: chain 7 settled on a complete
/// run, `stateFaults` said one edge would not survive, and edge f47 — the
/// third of seven — came back with no way. In the search it was fenced
/// against edge 1 alone; at the commit against 1, 3, 4 and 5.
///
/// **Off by default** (user, 2026-10-03). Measured: under `fenceTheLeadOnly`
/// it is what gets 45q its last edge back, 51 of 52 -> 52 of 52. Under the
/// whole-resonator fence it changes nothing at all on any of the eight
/// chips, because `stateFaults` is zero there — the fault it mends only
/// shows once the obstacles are loosened enough for the search to prefer a
/// different run of options of the same price.
///
/// Off, the commit reproduces the conditions the search answered under, so
/// what the search proved is what gets built. What it gives up is that two
/// edges of one chain may then be drawn too close to each other, which the
/// feedline pass has to settle — the same bargain `chainSolo` makes across
/// chains.
[[nodiscard]] inline bool fenceLaterEdges() {
  static const bool on = envFlag("SCPD_FENCE_LATER_EDGES", false);
  return on;
}

[[nodiscard]] inline bool fenceTheLeadOnly() {
  static const bool on = envFlag("SCPD_EDGE_LEAD_ONLY", true);
  return on;
}

/// Whether the feedline pass starts a resonator at `COUPLER_LEAD_STRAIGHT`
/// rather than at the tip of the whole lead (user, 2026-10-03).
///
/// The lead the insertion builds is `leadStraight()` cells of straight run
/// — 14, the prototype's figure for a resonator, plus `couplerLeadMargin`.
/// The margin is there for the insertion's sake alone: it buys the splice
/// room to be a turn instead of a corner, so that the way it leaves behind
/// is one a search could have returned.
///
/// It has no business being there afterwards. `applyOption` used to hand
/// the feedline pass the whole lead as the wire's fixed head and start its
/// search at the tip, so the resonator was held straight for 14 **and** the
/// margin before it was allowed to turn — room the rule never asked for and
/// the wire beside it then could not use.
///
/// Trimmed, the head is the turn and 14, and the margin's cells go back to
/// the search. `=0` keeps the whole lead fixed.
[[nodiscard]] inline bool trimTheLead() {
  static const bool on = envFlag("SCPD_FEEDLINE_LEAD_TRIM", true);
  return on;
}

/// Whether the feedline pass holds a resonator to a second straight run
/// after its lead — the rule's `min_straight_length` once more — before it
/// may turn (user, 2026-10-03).
///
/// The lead is a quarter turn and `max(COUPLER_LEAD_STRAIGHT, straightStart)`
/// cells of straight, 14 on every chip here, and the search starts at its
/// tip. The prototype starts its resonator at the end of the turn with
/// `start_straight_length = 14` (`FinalGrid.cpp:10658`): the 14 are its stub
/// **and** its lead, one straight run, and nothing is forced beyond them.
/// `applyOption` used to set `startStub` to the rule's run on top of the
/// lead, so the search began 14 + 11 = 25 cells after the turn.
///
/// Those eleven cells belong to no way until the resonator has been drawn
/// once: the way the insertion splices turns off five cells after the tip,
/// and a neighbour fenced by that way may settle inside the clearance of
/// where the search has to begin. On 45q wire 3 did exactly that in round 0
/// — 38.9 cells from 4's stub end before, 17.0 after, 21.2 from 4's way all
/// the while — and wire 4 never found a way again, in any round, at any
/// relaxation; 130 did the same to 131. See `fenceFixed`, which closes the
/// other half of this.
///
/// Measured over the eight chips, one run at a time, 2026-10-03, open wires
/// at the end of the stage (the five small chips have none under any
/// setting):
///
/// | | 45q | 57q | 69q | all |
/// |---|---|---|---|---|
/// | the second run, ways fenced (before) | 12 | 19 | 20 | 51 |
/// | no second run | 8 | 11 | 15 | 34 |
/// | the second run, fixed places fenced | 8 | 3 | 19 | 30 |
/// | **no second run, fixed places fenced** | **6** | **3** | **9** | **18** |
///
/// The two halves need each other: without the run the search starts where
/// the neighbours were fenced, but a wire let go of still gives its head
/// away; with the head fenced the run is still forced, and 4 on 45q finds a
/// way in round 0 only to lose it to 5's relaxation. Together they close
/// every resonator that was open at its head on 45q and 57q; what is left
/// is pairs of plain wires trading one lane (45q 65/66, 133/136; 57q
/// 9/10/11) and, on 69q, resonator 9, whose lead the insertion laid across
/// wire 10 with no room left for 10 to go. Fails on the three large chips
/// went 24/46/49 to 18/29/45; 17q gained two short resonators (7 to 9),
/// which the meander is for.
///
/// Off, which is the prototype's figure. `=1` restores the second run.
[[nodiscard]] inline bool resonatorStub() {
  static const bool on = envFlag("SCPD_RESONATOR_STUB", false);
  return on;
}

/// Whether a wire's fixed places — a resonator's lead, the straight run out
/// of a source, the run into a target — keep their clearance in every
/// fence, whether the wire is fenced or let go of (user, 2026-10-03).
///
/// `fence` closes the clearance around a wire's **way**. The fixed places
/// are not always in it: a resonator the insertion has just spliced holds
/// its lead and the old tail, and the straight run its next search must
/// make lies in neither. A neighbour drawn against the way alone may then
/// settle inside that run's clearance, and the resonator's next search is
/// over before it starts — its first cell is closed (see `deadOnArrival`).
///
/// And a wire let go of in the relaxation is fenced by nothing at all, so a
/// relaxed neighbour may be drawn straight through its head: 132 on 45q came
/// within 6 cells of 131's resonator port that way, 13 cells having been the
/// figure the insertion left. The prototype keeps exactly these cells hard
/// for a ripped resonator — `resonator_head_cells`, the arc and twenty cells
/// beyond it (`FinalGrid.cpp:12100`) — and this is that rule, for every wire
/// and for the runs at the launchers too.
///
/// Measured over the eight chips, 2026-10-03, with and without the second
/// run of `resonatorStub` — the table is there. Alone it takes 57q from 19
/// open wires to 3 and 45q's 131/132 apart, and leaves 69q at 19 against
/// 20; with the second run gone as well the three chips read 6, 3 and 9
/// against 12, 19 and 20.
///
/// On. `=0` fences the ways alone, as before.
[[nodiscard]] inline bool fenceFixed() {
  static const bool on = envFlag("SCPD_FENCE_FIXED", true);
  return on;
}

[[nodiscard]] inline std::uint32_t terminalSlot() {
  static const auto cells = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_TERMINAL_SLOT", 20), 0, 200));
  return cells;
}

/// Whether the orthogonal crossing rule binds at all. `=1` puts it back,
/// `=0` lifts it entirely.
///
/// Three things go with it, and all three have to go together or the run
/// contradicts itself:
///
/// 1. **The rule is not built.** `rebuildCrossingRule` clears the router's
///    constraints instead of filling them, so no cell remembers a heading
///    and no end zone is closed.
/// 2. **The search is the free one.** A wire under the feedline constraints
///    is searched with `route` rather than `routeOrthogonal`, so it is not
///    paying for a rule that is not there.
/// 3. **The fails do not count it.** `crossesAFeedline` says no for every
///    wire, so the `crossing` column reads 0 and the stage's total is the
///    room and the length alone.
///
/// What it leaves standing: the edges are still fenced and still priced.
/// Only the *angle* at which a wire may pass one stops being a rule.
///
/// **On by default** since 2026-10-04 (user). It was set aside while the
/// rip-up itself was the work (2026-10-02), so that what the sweep did
/// could be read without it; the rip-up is read, and the rule is the right
/// rule — a wire that crosses a feedline other than at a right angle is a
/// crossing no air bridge can be built over. The figure the stage is judged
/// by under it is open in the last round plus wires crossing a feedline
/// (plus unrouted), and the baseline it is measured against is
/// `artifacts/logs/base-ortho` (2026-10-03), eight chips one after another:
///
///   | | 4q | 9q | 17q | 21q | 33q | 45q | 57q | 69q | all |
///   | open, last round     | 0 | 0 |  4 | 0 | 3 | 2 |  9 | 10 | 28 |
///   | open, end of stage   | 0 | 0 |  4 | 0 | 4 | 6 | 17 | 24 | 55 |
///   | crossing a feedline  | 0 | 0 |  4 | 0 | 1 | 2 |  8 |  7 | 22 |
///   | fails, lengths incl. | 4 | 1 | 15 | 2 | 12 | 21 | 47 | 59 | 161 |
///
/// With the rule off the same build leaves 7 open in the last round, 18 at
/// the end and 0 crossing (`artifacts/logs/base`); most of what the rule
/// adds is wires grazing the ten-cell halo of an edge rather than crossing
/// one. `=0` is that regime, measured once as a control arm and tuned for
/// nowhere.
[[nodiscard]] inline bool orthoCrossing() {
  static const bool on = envFlag("SCPD_ORTHO_CROSSING", true);
  return on;
}

/// Whether the orthogonal search tests the cell a move ends on with the
/// heading it leaves on, as the count and the DRC read that cell
/// (`DubinsRouter::setOrthogonalExitCheck`). `SCPD_CROSSING_EXIT_HEADING`,
/// **off** until measured: the search as it stands lets a wire cross a
/// feedline straight and turn on the last cell of the halo, and the count
/// then calls it crossing — 12 of the 22 crossing wires of the 2026-10-04
/// baseline found a way in their last search and are "crossing" by that
/// cell alone, every one "10 cells from edge". On, the search refuses what
/// the count refuses, so a window of the targeted repair can be clean where
/// today it cannot.
///
/// **Measured 2026-10-04** (`artifacts/logs/exit-base` against
/// `base-ortho2`, eight chips, no repair): crossing 22 → 7, open 55 → 54,
/// `bad` 77 → 61, no chip worse, the stage 100 s faster over the eight.
/// With the targeted repair at 20 tests: `bad` 69 → 51, 17q 8 → 0, where
/// without it the repair accepts nothing on 17q. Still off: whether the
/// regime's rule changes is the user's decision.
[[nodiscard]] inline bool crossingExitHeading() {
  static const bool on = envFlag("SCPD_CROSSING_EXIT_HEADING", false);
  return on;
}

/// Whether a conventional wire that has to cross a chain edge is held to
/// crossing **that** edge: `SCPD_BRIDGE_CHECK`, **on** (user, 2026-10-04).
///
/// `assignBridges` tells every plain wire between two couplers which edge
/// it may cross, and `constrainByFeedlines` fences every other edge but the
/// terminal ones. A search can still come home without crossing its bridge
/// — round the end of the chain through a terminal edge, which fences
/// nobody, or through a gap the fences leave — and such a way is not a
/// detour but a wire on the wrong side of the feedline. Taken, it stands in
/// the rounds that follow and the wires around it are drawn against it: on
/// 33q wires 9 and 11 did exactly that and the lane pairs 9–12 never
/// settled. Under this switch a way found that does not cross the wire's
/// bridge is refused as no way, in phase 1 and at every relaxation level,
/// so the wire fails rather than poisons the rip-up. `=0` takes any way the
/// search finds, as before.
///
/// **Measured 2026-10-04** (`artifacts/logs/bridge-check` against
/// `base-ortho2`, eight chips, `stop_after = "feedlines"`, no repair):
/// `bad = unrouted + open + crossing` 77 → 71, open 55 → 50, crossing
/// 22 → 21, no chip worse, 103 ways refused over the eight chips, 4q / 9q /
/// 21q at 0 throughout; 33q's bundle 9–12 shrinks to 12/13, 57q and 69q
/// lose two open wires each, and the CPU time falls on six of eight chips.
[[nodiscard]] inline bool bridgeCheck() {
  static const bool on = envFlag("SCPD_BRIDGE_CHECK", true);
  return on;
}

/// How many pairs of ring neighbours the feedline pass fences:
/// `SCPD_FEEDLINE_FENCE_PAIRS`, **3** (user, 2026-10-05). The prototype
/// fences two (`additional_paths_1`). In phase 1 the pairs are `slot ± k`
/// for `k = 1..pairs`; at a relaxation level they are the `k`-th wire
/// beyond the one let go of and the `k`-th behind the wire being drawn,
/// the released wires never — they are the ones meant to stay crossable —
/// and a pair that would wrap round onto the wire or a released one is
/// left out.
///
/// **Measured 2026-10-05 on 69q** (5 rounds, relaxation 8, the terminal
/// stub guard on): 2 pairs `bad` 10, 3 pairs **5**, 4 pairs 5 — three and
/// four end on the same wires in every round (183/184, 190/191) and the
/// pass is a third faster, round 0 failing 10 wires instead of 17. 17q is
/// unchanged at 12. `=2` is the prototype's fence.
[[nodiscard]] inline std::uint32_t feedlineFencePairs() {
  static const auto pairs = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_FEEDLINE_FENCE_PAIRS", 3), 1, 8));
  return pairs;
}

/// Whether the fence beyond the wire let go of runs on to the next
/// conventional wire: `SCPD_FENCE_TO_CONVENTIONAL`, **off**.
///
/// **Measured 2026-10-05 on 69q** (6 rounds, relaxation 8, against
/// `full-r8`): the walk fired in 47 of 1195 relaxation searches, changed
/// the outcome of 4 — all in the pair 180–184 beside f60/f61, which stays
/// open either way — and ended on the same failed wires in every round and
/// the same `bad` 16. The hypothesis below is not what the rounds trade
/// on (user). Kept as a switch so the arm can be run again; `=1` turns it
/// on.
///
/// At a relaxation level the prototype fences the two wires beyond the one
/// it let go of and the two behind the wire being drawn, and leaves the
/// released one crossable. The two beyond are what bounds the room the
/// released wire gives up — and a resonator does not bound it: it starts at
/// its coupler out in the fan-in, not at the border, so a resonator in one
/// of those two slots leaves the room open on that side, and the wire drawn
/// into it settles where the next conventional wire runs. On 69q one round
/// fewer ended with fewer fails for exactly that reason (user). Under this
/// switch the fence beyond the released wire walks on until it has closed a
/// conventional wire — a resonator is closed and passed, a feedline edge is
/// passed and left to `constrainByFeedlines`, which fences every edge but
/// the bridged and the terminal ones — and never fewer than the prototype's
/// two. `=0` fences the two alone, as the prototype does.
[[nodiscard]] inline bool fenceToConventional() {
  static const bool on = envFlag("SCPD_FENCE_TO_CONVENTIONAL", false);
  return on;
}

/// Whether the straight run a terminal edge has to make at its coupler is
/// closed to every other wire of the feedline pass:
/// `SCPD_TERMINAL_STUB_GUARD`, **on** (user, 2026-10-05).
///
/// A terminal edge runs from a launcher to the first coupler of its chain,
/// or from the last coupler to a launcher, and the prototype fences it for
/// nobody (`forbidden_feedline_paths` leaves `is_first_last_feedline` out).
/// Its launcher end is walled by the port band in the obstacle mask; its
/// coupler end is not a port of the chip and has no band. So the run the
/// edge has to arrive on at the first coupler — the target of a starting
/// edge — and the run it has to leave the last coupler on — the source of
/// an ending edge — were open to every wire that is not a ring neighbour of
/// the edge, and a wire drawn across them leaves the edge no way when it is
/// drawn again: 69q's f5 and f6, 17q's f0, f1 and f12 end open there.
///
/// Under this switch `constrainByFeedlines` closes that run, inflated by
/// the clearance, in every search but the edge's own — drawn or not, since
/// the run follows from the coupler alone — and `SCPD_TERMINAL_STUB_EXTRA`
/// (5) cells beyond it, so that the bend into the run has room as well
/// (user). The meeting at the coupler stays open as `closeRoomOf` leaves
/// it, so the resonator and the next edge of the same coupler keep their
/// exits. And the coupler insertion closes the same runs of every chain
/// already settled in the corridor of every later edge search
/// (`guardSettledTerminalRuns`), where `SCPD_EDGE_SEES_CHAINS` otherwise
/// lets a chain ignore the others: that is where 69q's f5 and f6 crossed,
/// and a terminal edge that finds no way afterwards keeps that crossing.
/// `=0` leaves the runs open in both places, as before.
[[nodiscard]] inline bool terminalStubGuard() {
  static const bool on = envFlag("SCPD_TERMINAL_STUB_GUARD", true);
  return on;
}
[[nodiscard]] inline std::uint32_t terminalStubExtra() {
  static const auto extra = static_cast<std::uint32_t>(
      std::clamp(envWhole("SCPD_TERMINAL_STUB_EXTRA", 5), 0, 100));
  return extra;
}

/// Whether the feedline pass prices the room outside the lane polygon in a
/// relaxation: `SCPD_FEEDLINE_LANE`, **on**. The polygon is the prototype's
/// `compute_corridor_polygon_proximity` — the lane between the two
/// neighbours' ways (`laneOf`), everything outside it at the wire price, so
/// a relaxed wire stays in its lane rather than wandering where the release
/// opened room. `=0` leaves the polygon out of the feedline pass; the halos
/// and the toll of the wires let go of stay. Added 2026-10-04 to measure
/// what the polygon is worth (user).
[[nodiscard]] inline bool feedlineLane() {
  static const bool on = envFlag("SCPD_FEEDLINE_LANE", true);
  return on;
}

/// Whether the outer routing prices the room outside the lane polygon in a
/// relaxation: `SCPD_OUTER_LANE`, **on** (user, 2026-10-04 evening, after
/// an afternoon without it). The polygon between the two neighbours' ways is
/// the ground of the outer routing's relaxation — everything outside it at
/// the wire price, the lane free. `=0` leaves it out and the relaxation is
/// priced by `outerPenalty` alone; measured on 57q under the hard
/// length-point clearance: polygon and per-level halos 23 open, halos alone
/// 12, and 0 once the band of the wire let go of went with it — see *The
/// length-point clearance* in handover-feedline-routing.md.
[[nodiscard]] inline bool outerLane() {
  static const bool on = envFlag("SCPD_OUTER_LANE", true);
  return on;
}

/// How the outer routing prices the wires it let go of in a relaxation:
/// `SCPD_OUTER_PENALTY`. **0 (default)** is a halo of one clearance more
/// per level, as the outer routing has always had it — the wire released
/// at level `l` priced within `(l + 1) · clearance`. 1 is the feedline
/// pass's passing penalty: every released wire priced within
/// `SCPD_HALO_REACH · clearance`, the same radius at every level, and the
/// crossing toll (`SCPD_CROSS_TOLL`) on the released way itself. 2 prices
/// them not at all. Measured on 57q's outer routing under the hard
/// length-point clearance (handover-feedline-routing.md, *The length-point
/// clearance*): with the polygon 18 / 19 / 28 open for 0 / 1 / 2, without
/// it 0 / 35 / —. The feedline pass is not touched by this; it prices as
/// `priceTheReleased` says.
[[nodiscard]] inline int outerPenalty() {
  static const int mode = std::clamp(envWhole("SCPD_OUTER_PENALTY", 0), 0, 2);
  return mode;
}

/// How many places the insertion will try before it gives a coupler up. The
/// first is the one the bias names; the rest are what a coupler whose every
/// orientation is refused there falls back on.

// --------------------------------------------------------------- The tuning

/// Every knob of the stage, already converted onto the grid it runs on.
///
/// Nothing here is a cell count that stands for a distance. What the
/// configuration carries is a length in layout units or a price, and the
/// conversion happens once, here, so that no call site can convert it a
/// second time or differently.
struct Tuning {
  /// What the stage keeps between two wires, in cells. The rule spans this
  /// many whole cells, so the search keeps a little more than the rule asks.
  std::uint32_t clearance = 0;
  /// The rule itself, in cells but not rounded. What the search keeps is the
  /// whole number above it; what a committed wire is judged by is this, which
  /// is the same figure the design-rule check uses.
  double spacing = 0.0;
  /// The straight run a wire leaves its source on, in cells.
  std::uint32_t straightStart = 0;
  /// The rule `min_straight_length` itself, in cells of the smaller cell
  /// side and not rounded: the length of a port run in the bottleneck
  /// analysis, the same along an axis and along a diagonal.
  double straightLength = 0.0;
  /// The same run, lengthened by `launcherLead`, in cells: what the coupler
  /// box keeps clear of a launcher. Never shorter than `straightStart`.
  std::uint32_t launcherStraight = 0;
  /// How far to either side of its own way a wire may be moved, in cells.
  std::uint32_t reach = 0;
  std::uint32_t innerReach = 0;
  std::uint32_t rounds = 0;
  std::uint32_t innerRounds = 0;
  std::uint32_t maxRelaxation = 0;
  std::uint32_t refinementRounds = 0;
  std::uint32_t feedlineRefinementRounds = 0;
  /// How long a resonator's way is made before the coupler is spliced in, in
  /// cells. Zero switches the meander off.
  double meanderLength = 0.0;
  /// The same in layout units, for the lines that report it.
  double meanderLengthUnits = 0.0;
  /// How long a resonator is when the stage is done: the design rule, in
  /// cells and in layout units, with the tolerance the rule allows. The
  /// coupler is spliced where the way left to the qubit is this long, and
  /// the way drawn again from the coupler is made this long exactly.
  double targetLength = 0.0;
  double targetLengthUnits = 0.0;
  double lengthTolerance = 0.0;
  double lengthToleranceUnits = 0.0;
  /// The coupler body, in cells: the run the resonator couples along and
  /// how far the feedline lies from it.
  std::uint32_t couplerLength = 0;
  std::uint32_t couplerHeight = 0;
  /// How many times a placed coupler may be turned to another of its
  /// options while the feedlines do not route.
  std::uint32_t repairTrials = 0;
  /// The last of the five phases to run, by the name of its snapshot, or
  /// empty for all five. A stop is for looking at one phase's work: the
  /// stage still counts what it came to and still writes its artifact.
  std::string stopAfter;
  /// The price of one eighth turn, in hundredths of a cell.
  std::uint16_t bendPenalty = 0;
  std::uint8_t wireProximityPenalty = 0;
  std::uint8_t staticProximityPenalty = 0;
  /// How far the price of running beside an obstacle reaches, in cells.
  std::uint32_t obstacleReach = 0;
  /// The approach of a terminal, in the geometry of a port's band: the
  /// straight length in steps and the half width in cells, along an axis and
  /// along a diagonal, where a step is longer and a strip narrower by the
  /// square root of two.
  std::uint32_t approachAxial = 0;
  std::uint32_t approachDiagonal = 0;
  std::uint32_t approachHalfAxial = 0;
  std::uint32_t approachHalfDiagonal = 0;
};

/// A price quoted against the extent of the grid, resolved to an absolute one.
///
/// A bend costs a fixed amount while the cost of a way grows with the grid, so
/// the same bend price is relatively cheaper on a bigger chip. The prototype
/// had to retune three penalties per chip until it quoted them this way, and
/// the spread it had hand-tuned then collapsed from 3.75 to about 2.
[[nodiscard]] std::uint32_t resolved(const double norm,
                                     const grid::GridMetrics& router) {
  const auto extent = static_cast<double>(router.width) + router.height;
  return static_cast<std::uint32_t>(std::lround(norm * extent));
}

/// A figure in cells along an axis, as cells along a heading.
///
/// Every figure of the tuning is a length in layout units converted once, on
/// the axis with the shorter cell step. A diagonal step is longer by the
/// square root of two, so the same length is fewer cells along it, and a
/// body laid out in axial cells on a diagonal would be half again as long as
/// the design rule asks.
[[nodiscard]] std::uint32_t cellsOn(const std::uint32_t axial,
                                    const routing::Heading heading) {
  if (!routing::isDiagonal(heading)) {
    return axial;
  }
  return std::max(1U, static_cast<std::uint32_t>(std::lround(
                          static_cast<double>(axial) / std::numbers::sqrt2)));
}

[[nodiscard]] Tuning tuningOf(const ConfigT& config,
                              const grid::GridMetrics& router) {
  const fbc::FinalParamsT defaults;
  const auto& params =
      (config.stages != nullptr && config.stages->final != nullptr)
          ? *config.stages->final
          : defaults;
  const auto& rules = *config.rules;

  Tuning tuning;
  // The one conversion of the design rule. `cells_for` spans at least the
  // rule, so nineteen cells of about ten layout units is 190 against a rule of
  // 185; the prototype writes the nineteen out as a literal on every one of
  // its eight grids and records no reason for it.
  tuning.clearance =
      std::max(1U, grid::cellsFor(rules.min_wire_spacing, router));
  // The rule itself, unrounded. A committed way is judged by this and not by
  // the whole cells the search keeps, so that the verdict is the one the
  // design-rule check gives: `checkClearance` measures a pair in cells times
  // the smaller cell side, and the same side is the unit here.
  tuning.spacing =
      rules.min_wire_spacing / std::min(router.cellWidth, router.cellHeight);
  tuning.straightStart = grid::cellsFor(rules.min_straight_length, router);
  tuning.straightLength =
      rules.min_straight_length / std::min(router.cellWidth, router.cellHeight);
  tuning.launcherStraight =
      grid::cellsFor(rules.min_straight_length + launcherLead(), router);
  tuning.reach = std::max(1U, params.corridor_spacings * tuning.clearance);
  tuning.innerReach =
      std::max(1U, params.inner_corridor_spacings * tuning.clearance);
  tuning.rounds = std::max(1U, params.rounds);
  tuning.innerRounds = std::max(1U, params.inner_rounds);
  tuning.maxRelaxation = params.max_relaxation;
  tuning.refinementRounds = params.refinement_rounds;
  tuning.feedlineRefinementRounds = params.feedline_refinement_rounds;
  // How long the outer routing makes a resonator's way is the design rule
  // itself, not a figure of its own. It used to be `meander_length`, carried
  // per chip and set to anything from 2500 to 6000 while every chip asked
  // for a 2500-unit resonator; what it actually decided was how much slack
  // the way had over the figure before the coupler cut it back. One number
  // for one thing: the way is made the length the rule asks for.
  tuning.meanderLength =
      rules.target_resonator_length <= 0.0
          ? 0.0
          : rules.target_resonator_length /
                std::min(router.cellWidth, router.cellHeight);
  tuning.meanderLengthUnits = std::max(0.0, rules.target_resonator_length);
  const auto unit = std::min(router.cellWidth, router.cellHeight);
  tuning.targetLength = std::max(0.0, rules.target_resonator_length) / unit;
  tuning.targetLengthUnits = std::max(0.0, rules.target_resonator_length);
  tuning.lengthTolerance =
      std::max(0.0, rules.resonator_length_tolerance) / unit;
  tuning.lengthToleranceUnits = std::max(0.0, rules.resonator_length_tolerance);
  tuning.couplerLength =
      std::max(1U, grid::cellsFor(params.coupler_length, router));
  tuning.couplerHeight =
      std::max(1U, grid::cellsFor(params.coupler_height, router));
  tuning.repairTrials = params.repair_trials;
  tuning.stopAfter = params.stop_after;
  tuning.bendPenalty = static_cast<std::uint16_t>(
      std::min<std::uint32_t>(std::numeric_limits<std::uint16_t>::max(),
                              resolved(params.bend_penalty_norm, router)));
  // The search adds a penalty per swept cell, so it has to stay well below
  // the 127 the router accepts or one cell of it would outweigh a whole bend.
  tuning.wireProximityPenalty =
      static_cast<std::uint8_t>(std::min<std::uint32_t>(
          127U, resolved(params.wire_proximity_penalty_norm, router)));
  tuning.staticProximityPenalty =
      static_cast<std::uint8_t>(std::min<std::uint32_t>(
          127U, resolved(params.static_proximity_penalty_norm, router)));
  tuning.obstacleReach = grid::cellsFor(params.obstacle_penalty_reach, router);
  tuning.approachAxial =
      grid::bandLength(rules.min_straight_length, router, false);
  tuning.approachDiagonal =
      grid::bandLength(rules.min_straight_length, router, true);
  tuning.approachHalfAxial =
      grid::bandHalfWidth(rules.min_wire_spacing, router, false);
  tuning.approachHalfDiagonal =
      grid::bandHalfWidth(rules.min_wire_spacing, router, true);
  return tuning;
}

// ---------------------------------------------------------------- The scene

/// The grid the stage searches on, and what it may not enter.
///
/// None of it is an artifact. It is rebuilt from the chip and the
/// configuration, as every grid of a run is, which is what keeps a resumed run
/// equal to an uninterrupted one.
struct Scene {
  grid::GridMetrics router;
  /// The chip artwork, its obstacle keepout, the border and the ports' own
  /// approaches. Every cell a search may enter satisfies the obstacle rule
  /// because this mask already holds it.
  grid::BitGrid blocked;
  /// The artwork alone: the qubits, the couplers and the launchers as
  /// `rasterizeObstacles` lays them on the grid, with the launcher cells
  /// freed. None of what `blocked` adds on top of it — not the block over
  /// everything outside the launchers, not the approach band of every port.
  /// A feedline edge is drawn against this and the launcher stubs and
  /// nothing else.
  grid::BitGrid components;
  /// The cell each routable port is reached at, by port index.
  std::unordered_map<std::uint32_t, std::size_t> targetCell;
  /// The heading a wire has when it arrives at each port, by port index.
  std::unordered_map<std::uint32_t, Heading> arrival;
  /// The cell each launcher stands on, by port index: where a feedline
  /// chain starts and ends. Freed in the mask, because a wire has to be
  /// able to stand where it starts.
  std::unordered_map<std::uint32_t, std::size_t> launcherCell;
  /// The heading a wire leaves each launcher on, by port index. An outer
  /// wire is fed from a point on the ring and carries no port for its
  /// launcher, so this is the only place the launcher's own direction is
  /// kept.
  std::unordered_map<std::uint32_t, routing::Heading> launcherHeading;
};

/// Block everything between the edge of the chip and the ring the wires are
/// fed from.
///
/// Every wire starts on the ring of launcher slots, which is a rectangle set
/// in from the chip outline. What lies outside it is free space that no wire
/// has any business in: a wire that enters it comes back in somewhere else and
/// has gone *around* the sources of the wires beside it rather than past them,
/// which is a crossing the plan never allowed. The capacity grid keeps the
/// same strip clear through the configured launcher offset; here the strip is
/// derived, because where the sources are is a fact of the chip and not a
/// figure to tune.
void blockOutsideTheSources(const ChipT& chip, Scene& scene) {
  std::uint32_t lowX = scene.router.width - 1;
  std::uint32_t lowY = scene.router.height - 1;
  std::uint32_t highX = 0;
  std::uint32_t highY = 0;
  bool found = false;
  for (const auto& port : chip.ports) {
    if (port->role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    const auto cell = scene.router.clampToCell(port->center);
    lowX = std::min(lowX, cell.x());
    highX = std::max(highX, cell.x());
    lowY = std::min(lowY, cell.y());
    highY = std::max(highY, cell.y());
    found = true;
  }
  if (!found || lowX > highX || lowY > highY) {
    return;
  }
  for (std::uint32_t y = 0; y < scene.router.height; ++y) {
    const bool outsideRow = y < lowY || y > highY;
    for (std::uint32_t x = 0; x < scene.router.width; ++x) {
      if (outsideRow || x < lowX || x > highX) {
        scene.blocked.setCell(x, y, true);
      }
    }
  }
}

[[nodiscard]] Scene sceneOf(const ChipT& chip, const ConfigT& config,
                            const grid::GridMetrics& capacity) {
  const auto& rules = *config.rules;

  Scene scene;
  scene.router = grid::routerGrid(capacity, config.grid->router_cell_size);

  const design::BridgeRules noRules;
  const auto& bridgeRules =
      config.ports != nullptr ? config.ports->bridge_pairs : noRules;
  const auto bridging = design::bridgingPorts(chip, bridgeRules);

  // The obstacle clearance cannot be expressed in a search step: the router
  // knows a wire only by its centre line and may enter any free cell. So it is
  // baked into the mask instead, as an exact distance in layout units from the
  // cell to the polygon edge rather than a dilation of the finished raster —
  // a dilation would carry the half cell the fill convention differs by, and
  // would widen the edge seam a second time.
  //
  // What the keepout would otherwise seal is the ports themselves: every port
  // sits at the foot of a small polygon of its own. Each keeps a corridor out
  // along its orientation, as long as the approach its band reserves.
  grid::RasterOptions options;
  options.keepout = rules.min_obstacle_spacing;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!design::isRoutable(port.role) &&
        port.role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    const auto step = grid::orientationStep(port.orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const double reach =
        (bridging[index] ? 2.0 : 1.0) * rules.min_straight_length;
    const double length = step.diagonal() ? reach / std::sqrt(2.0) : reach;
    options.keepoutExemptions.push_back(
        {.from = port.center,
         .to = {port.center.x() + (step.x * length),
                port.center.y() + (step.y * length)},
         .halfWidth = 0.5 * rules.min_wire_spacing});
  }

  const auto raster = grid::rasterizeObstacles(chip, scene.router, options);
  scene.blocked = raster.blocked;
  // Kept before anything else is stamped into the mask: the artwork on its
  // own, which is all a feedline edge has to stay out of.
  scene.components = raster.blocked;
  blockOutsideTheSources(chip, scene);

  // A launcher's slot is the launcher, as the capacity scene has it: the
  // cell its centre rounds to, freed so that a feedline can stand on it. The
  // keepout exemption above keeps the run out of the pad open.
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (port.role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    if (routing::headingOfOrientation(port.orientation) >=
        routing::NUM_HEADINGS) {
      continue;
    }
    const auto cell = scene.router.clampToCell(port.center);
    const auto slot = scene.router.index(cell.x(), cell.y());
    scene.blocked.set(slot, false);
    scene.components.set(slot, false);
    scene.launcherCell.emplace(index, slot);
    scene.launcherHeading.emplace(
        index, routing::reverse(routing::headingOfOrientation(port.orientation)));
  }

  // The approach of every routable port, as the prototype's
  // `add_fixed_port_obstacles` stamps it: a strip one wire spacing wide that
  // runs the minimum straight length out of the port and further back behind
  // it, so that a wire reaches the port along its orientation and no other
  // wire runs over the approach. The target is the cell just beyond it.
  //
  // What the mask held before the bands, so that `portBands` can put it back
  // and leave the targets the bands dug standing.
  const auto withoutBands = scene.blocked;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!design::isRoutable(port.role)) {
      continue;
    }
    const auto step = grid::orientationStep(port.orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const auto heading = routing::headingOfOrientation(port.orientation);
    if (heading >= routing::NUM_HEADINGS) {
      continue;
    }
    // The target is the first cell along the step whose centre lies the
    // straight length or more from the port itself — the port's own
    // position, not the centre of the cell it falls on, which can be half a
    // cell off — and the band ends on the cell before it. So the straight
    // run into the port exceeds the rule by less than one step: on a grid of
    // ten-unit cells by up to ten units along an axis and fourteen along a
    // diagonal, where a step count of whole band lengths put it twenty to
    // thirty units over.
    const auto reach =
        (bridging[index] ? 2.0 : 1.0) * rules.min_straight_length;
    const auto centre = scene.router.clampToCell(port.center);
    // Signed throughout: a step of -1 times an unsigned count wraps around,
    // and the walk would leave the grid on its first step.
    std::uint32_t steps = 0;
    for (std::int64_t k = 1; steps == 0; ++k) {
      const auto x = static_cast<std::int64_t>(centre.x()) + (step.x * k);
      const auto y = static_cast<std::int64_t>(centre.y()) + (step.y * k);
      if (x < 0 || y < 0 || x >= scene.router.width ||
          y >= scene.router.height) {
        break;
      }
      const auto at =
          scene.router.toLayout(static_cast<double>(x), static_cast<double>(y));
      if (std::hypot(at.x() - port.center.x(), at.y() - port.center.y()) >=
          reach) {
        steps = static_cast<std::uint32_t>(k);
      }
    }
    const grid::PortBand band{
        .center = centre,
        .step = step,
        .forward = steps == 0
                       ? grid::bandLength(reach, scene.router, step.diagonal())
                       : steps - 1,
        .backward = grid::bandLength(3.0 * rules.min_straight_length,
                                     scene.router, step.diagonal()),
        .halfWidth = grid::bandHalfWidth(rules.min_wire_spacing, scene.router,
                                         step.diagonal())};
    const auto stamped = grid::stampBand(scene.blocked, band);
    // The router grid keeps no mask from before the bands, so the target is
    // dug rather than placed: the walk frees the band cells it crosses, which
    // is the way out of the port's own approach. This is the one caller
    // `digTargetBeyondBand` was written for.
    if (const auto target =
            grid::digTargetBeyondBand(scene.blocked, stamped, step)) {
      scene.targetCell.emplace(index, *target);
      scene.arrival.emplace(index, heading);
    }
  }
  if (!portBands()) {
    scene.blocked = withoutBands;
  }
  return scene;
}

// ------------------------------------------------------------ The clearance

/// The offsets of one clearance disc, and what enters it on each step.
///
/// Charging the disc around every cell of a wire costs the disc for every
/// cell, and the disc of nineteen cells holds eleven hundred of them. Two
/// consecutive cells of a path differ by one step, so all but a leading edge
/// of the disc is already charged: the full disc is stamped once, at the first
/// cell, and one edge per step after that. The prototype does the same, and
/// what it buys is a factor of thirty.
struct Stencil {
  using Offsets = std::vector<std::pair<std::int32_t, std::int32_t>>;
  Offsets full;
  /// The offsets that enter the disc on a step, indexed `[dy + 1][dx + 1]`.
  std::array<std::array<Offsets, 3>, 3> edge;
};

[[nodiscard]] Stencil stencilOf(const std::uint32_t radius) {
  Stencil stencil;
  const auto reach = static_cast<std::int32_t>(radius);
  const auto limit = reach * reach;
  const auto inside = [limit, reach](const std::int32_t dx,
                                     const std::int32_t dy) {
    return dx >= -reach && dx <= reach && dy >= -reach && dy <= reach &&
           ((dx * dx) + (dy * dy)) <= limit;
  };
  for (std::int32_t dy = -reach; dy <= reach; ++dy) {
    for (std::int32_t dx = -reach; dx <= reach; ++dx) {
      if (inside(dx, dy)) {
        stencil.full.emplace_back(dx, dy);
      }
    }
  }
  for (std::int32_t sy = -1; sy <= 1; ++sy) {
    for (std::int32_t sx = -1; sx <= 1; ++sx) {
      if (sx == 0 && sy == 0) {
        continue;
      }
      auto& edge = stencil.edge[static_cast<std::size_t>(sy + 1)]
                               [static_cast<std::size_t>(sx + 1)];
      for (const auto& [dx, dy] : stencil.full) {
        if (!inside(dx + sx, dy + sy)) {
          edge.emplace_back(dx, dy);
        }
      }
    }
  }
  return stencil;
}

/// The copper of the chip, and the room every wire keeps around it.
///
/// Two fields over the router grid. `owner` says which wire holds a cell;
/// `guard` counts how many wires keep their clearance over a cell. A wire
/// charges its guard when it is put down and gives it up when it is taken
/// off.
///
/// The field is what a wire is *judged* by, not what its search is fenced in
/// by. The search keeps the prototype's two ring neighbours, inflated by the
/// clearance, and nothing else (`Driver::fence`); the fails of a pass are
/// counted on this field against every other wire (`Driver::conflictsIn`).
/// Charged once per move, the count against two hundred wires costs what the
/// count against two cost.
class Field {
public:
  Field(const grid::GridMetrics& router, const std::uint32_t clearance)
      : stencil_(stencilOf(clearance)), width_(router.width),
        height_(router.height), guard_(router.cells(), 0),
        owner_(router.cells(), NO_OWNER), stamp_(router.cells(), 0) {}

  [[nodiscard]] bool guarded(const std::size_t cell) const {
    return guard_[cell] != 0;
  }
  [[nodiscard]] std::uint16_t guardCount(const std::size_t cell) const {
    return guard_[cell];
  }
  [[nodiscard]] std::uint32_t owner(const std::size_t cell) const {
    return owner_[cell];
  }

  /// Take the copper of a wire, without its room.
  void occupy(const Path& path, const std::uint32_t wire) {
    for (const auto& point : path) {
      if (point.x < width_ && point.y < height_) {
        owner_[index(point.x, point.y)] = wire;
      }
    }
  }

  /// Give up the copper of a wire. A cell another wire has since taken is
  /// left alone, which cannot happen while no two wires share a cell but
  /// costs nothing to say.
  void vacate(const Path& path, const std::uint32_t wire) {
    for (const auto& point : path) {
      if (point.x < width_ && point.y < height_) {
        const auto cell = index(point.x, point.y);
        if (owner_[cell] == wire) {
          owner_[cell] = NO_OWNER;
        }
      }
    }
  }

  /// Charge the room a wire keeps, or give it up again.
  void charge(const Path& path) { walk(path, 1); }
  void discharge(const Path& path) { walk(path, -1); }

  /// The same for the places a wire cannot be moved off.
  void chargeFixed(const Path& fixed) { walk(fixed, 1); }
  void dischargeFixed(const Path& fixed) { walk(fixed, -1); }

private:
  [[nodiscard]] std::size_t index(const std::uint32_t x,
                                  const std::uint32_t y) const {
    return (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_)) + x;
  }

  /// Apply a change to every cell within the rule of the path, once each.
  ///
  /// The stamp is what makes it once each: the leading edges of the steps
  /// overlap wherever a path turns, and a path that comes back on itself
  /// covers the same cells twice. Charging a cell twice for one wire would
  /// leave it guarded after the wire is gone.
  void walk(const Path& path, const int by) {
    if (path.empty()) {
      return;
    }
    beginPass();
    bool started = false;
    std::int64_t lastX = 0;
    std::int64_t lastY = 0;
    for (const auto& point : path) {
      const auto x = static_cast<std::int64_t>(point.x);
      const auto y = static_cast<std::int64_t>(point.y);
      if (started) {
        const auto sx = x - lastX;
        const auto sy = y - lastY;
        if (sx == 0 && sy == 0) {
          continue;
        }
        if (sx >= -1 && sx <= 1 && sy >= -1 && sy <= 1) {
          stamp(stencil_.edge[static_cast<std::size_t>(sy + 1)]
                             [static_cast<std::size_t>(sx + 1)],
                x, y, by);
          lastX = x;
          lastY = y;
          continue;
        }
      }
      stamp(stencil_.full, x, y, by);
      started = true;
      lastX = x;
      lastY = y;
    }
  }

  void beginPass() {
    if (++pass_ == 0) {
      std::ranges::fill(stamp_, 0);
      pass_ = 1;
    }
  }

  void stamp(const Stencil::Offsets& offsets, const std::int64_t cx,
             const std::int64_t cy, const int by) {
    for (const auto& [dx, dy] : offsets) {
      const auto x = cx + dx;
      const auto y = cy + dy;
      if (x < 0 || y < 0 || x >= width_ || y >= height_) {
        continue;
      }
      const auto cell = static_cast<std::size_t>((y * width_) + x);
      if (stamp_[cell] == pass_) {
        continue;
      }
      stamp_[cell] = pass_;
      guard_[cell] = static_cast<std::uint16_t>(guard_[cell] + by);
    }
  }

  Stencil stencil_;
  std::int64_t width_;
  std::int64_t height_;
  std::vector<std::uint16_t> guard_;
  std::vector<std::uint32_t> owner_;
  std::vector<std::uint32_t> stamp_;
  std::uint32_t pass_ = 0;
};

// ----------------------------------------------------------------- The wires

/// One connection of the plan, and the way it has.
struct Wire {
  RoutingObjective objective;
  /// The way it has: the Detail stage's cells at first, joined on this grid
  /// and put down like any other way, its own after it has been routed. The
  /// corridor of every search is a band around this, and a wire whose search
  /// finds nothing keeps it.
  Path way;
  /// Whether the way was drawn by this stage. A way that was not is a seed:
  /// it holds copper and room on the canvas, but it is not curvature-
  /// constrained and knows nothing of this grid's keepout, so a wire still
  /// on it when the rounds end is unrouted and the artifact carries no cells
  /// for it.
  bool drawn = false;
  /// Whether it has been routed in the pass running now.
  bool routed = false;
  /// Whether its room is charged into the field.
  bool placed = false;
  /// Whether only its fixed places are charged, which is what a wire without
  /// a way of its own holds until it is drawn.
  bool endsOnly = false;
  /// The places it cannot be moved off: the straight run it leaves its source
  /// on, which its port's orientation fixes, and the cell of its target port.
  Path fixed;
  bool resonator = false;
  /// The run from the cell its way ends on to the port itself, in cells. The
  /// sampler measures a way to its last cell, and the wire runs on to the
  /// port from there; a resonator's length counts both.
  double anchorGap = 0.0;
  /// The same run in layout units, for the lines that report it.
  double anchorGapUnits = 0.0;
  /// Whether its way is shorter than a resonator's has to be: drawn, but
  /// without room for the meander that would make it long enough.
  bool tooShort = false;
  bool inner = false;
  /// Its place in the artifact: the connection of the assignment, or of the
  /// global stage for an inner wire.
  std::uint32_t slot = 0;
  /// Its place in the wire list, which is what the field records per cell.
  std::uint32_t key = 0;
  /// Whether both of its ends lie on the grid.
  bool feasible = false;
  /// The ports its two ends are, where they are ports at all. A resonator is
  /// fed on the segment between two launchers, so its source is a point and
  /// no port stands there.
  std::uint32_t sourcePort = NO_OWNER;
  std::uint32_t targetPort = NO_OWNER;
  /// Whether it is an edge of a feedline chain, and which.
  bool feedline = false;
  std::uint32_t edge = NO_OWNER;
  /// A feedline edge that starts or ends at a launcher: a hard obstacle for
  /// every wire.
  bool terminal = false;
  /// The couplers at its ends: a resonator's own, a feedline edge's two, by
  /// their index in the coupler list. Where two wires share one, the rule
  /// does not bind near it.
  std::uint32_t couplerAtSource = NO_OWNER;
  std::uint32_t couplerAtTarget = NO_OWNER;
  /// The straight run it leaves its source on and arrives at its target on,
  /// in cells, where the pass's own figure does not apply: a resonator
  /// drawn from its coupler runs the coupling length straight, a feedline
  /// edge runs through the coupler body it arrives at.
  std::optional<std::uint32_t> startStub;
  std::uint32_t endStub = 0;
  /// The head of a resonator drawn from its coupler: the coupling run from
  /// the anchor and the quarter turn after it, up to the source of the
  /// search. It is put in front of every way the search finds, and it is a
  /// fixed place.
  Path arc;
  /// The feedline edge a conventional wire may cross, at a right angle: the
  /// edge of the chain that spans its launcher. Everything else is fenced.
  std::uint32_t bridged = NO_OWNER;
  /// Whether the way is longer than a resonator may be, in the feedline
  /// phase, where no meander can help it.
  bool tooLong = false;
  /// Whether a neighbour's relaxation let go of it since it was last drawn:
  /// then it is drawn again even where its way holds, because letting it
  /// go was a promise that it moves.
  bool ripped = false;
};

/// What one pass of the driver does.
struct Pass {
  /// What to call it in the progress lines.
  std::string name;
  std::uint32_t rounds = 1;
  std::uint32_t maxRelaxation = 0;
  /// The band around a wire's own way, in cells.
  std::uint32_t reach = 0;
  /// The straight run out of the source, in cells.
  std::uint32_t straightStart = 0;
  /// Whether a wire that already has a drawn way is left alone.
  bool keepDrawn = true;
  /// Whether the feedlines constrain the pass: a wire crosses a feedline
  /// at a right angle only, the edges of the chains are fenced, and a
  /// resonator is made its target length exactly.
  bool feedlines = false;
  /// Whether the pass redraws only the members already flagged
  /// `routed = false` and leaves every other member on the way it holds,
  /// standing as the fences and lanes of the ones redrawn. Off, a pass under
  /// the feedline constraints clears `routed` on every member in
  /// `beginPass`, so every member is drawn again.
  ///
  /// The targeted repair's local rip-up-and-reroute is what it is for
  /// (2026-10-04): the window of a candidate run of coupler options is
  /// flagged, the **whole ring** is swept, and only the window and whatever
  /// the relaxation releases are redrawn — while the ±1/±2 fences and the
  /// lane see the true ring neighbours. A pass over a subset of `members`
  /// could not give that: `attempt` reads its neighbours off
  /// `members[slot ± 1]`, and the relaxation releases `members[slot ± level]`.
  /// Needs `keepDrawn`, which is what makes the sweep skip a routed member.
  bool onlyUnsettled = false;
  /// Whether this is a refinement pass — one attempt per wire, no rip-up,
  /// a rollback onto the way the wire had. `refine` sets nothing itself; the
  /// flag is what tells `lengthensIn` that `SCPD_REFINE_MEANDER` applies
  /// and the checks at the end of the pass which name to say.
  bool refinement = false;
};

// ------------------------------------------------------------ Joining cells

/// The heading of one step between two neighbouring cells, or the heading the
/// first cell has when the two are one place.
[[nodiscard]] Heading headingOfStep(const PathPoint& from,
                                    const PathPoint& to) {
  const int dx = (to.x > from.x) ? 1 : ((to.x < from.x) ? -1 : 0);
  const int dy = (to.y > from.y) ? 1 : ((to.y < from.y) ? -1 : 0);
  for (Heading heading = 0; heading < routing::NUM_HEADINGS; ++heading) {
    const auto step = routing::headingVector(heading);
    if (step.dx == dx && step.dy == dy) {
      return heading;
    }
  }
  return from.heading;
}

/// Extend a way to a cell by the eight-connected steps of the line between
/// its last cell and that one, so that no two consecutive cells of the way
/// are more than one step apart. A cell the way already ends on adds nothing.
void connect(Path& way, const PathPoint& to) {
  auto x = static_cast<std::int64_t>(way.back().x);
  auto y = static_cast<std::int64_t>(way.back().y);
  const auto targetX = static_cast<std::int64_t>(to.x);
  const auto targetY = static_cast<std::int64_t>(to.y);
  const auto dx = std::abs(targetX - x);
  const auto dy = -std::abs(targetY - y);
  const std::int64_t sx = x < targetX ? 1 : -1;
  const std::int64_t sy = y < targetY ? 1 : -1;
  auto error = dx + dy;
  while (x != targetX || y != targetY) {
    const auto doubled = 2 * error;
    if (doubled > dy) {
      error += dy;
      x += sx;
    }
    if (doubled < dx) {
      error += dx;
      y += sy;
    }
    way.push_back({.x = static_cast<std::uint32_t>(x),
                   .y = static_cast<std::uint32_t>(y),
                   .heading = 0,
                   .primitive = 0});
  }
}

// -------------------------------------------------------- The debug pictures

/// Where a search stood when its picture was taken.
struct DebugFrame {
  /// The pass, by its first word: "inner", "outer", "refinement".
  std::string pass;
  std::uint32_t round = 0;
  bool forward = true;
  /// "normal" for phase 1, "relax N" for the relaxation, "refine".
  std::string kind = "normal";
  /// The wires whose inflated ways fence the search.
  std::vector<std::uint32_t> fence;
  /// The wires let go of, which the search may cross.
  std::vector<std::uint32_t> ripped;
  /// The two ring neighbours, which bound the lane.
  std::uint32_t before = NO_OWNER;
  std::uint32_t after = NO_OWNER;
  /// The three ways the lane polygon was actually built from, as
  /// `Driver::laneOf` made them. The picture used to draw the polygon from
  /// the raw ring neighbours instead, which is not the polygon the pricing
  /// used: past a feedline or a run of resonators the walk goes further out
  /// (see `laneOf`), and the shape on the screen then had nothing to do with
  /// the shape the search paid by (user, 2026-10-01).
  Path laneBefore;
  Path laneAfter;
  Path laneCurrent;
  const std::vector<Wire>* wires = nullptr;
};

/// The first word of a pass name, for a file name.
[[nodiscard]] std::string passTag(const std::string_view name) {
  const auto space = name.find(' ');
  return std::string(space == std::string_view::npos ? name
                                                     : name.substr(0, space));
}

/// A wire as its picture names it: the ring index, or "i" and the index of
/// the inner circuit.
[[nodiscard]] std::string wireId(const Wire& wire) {
  if (wire.feedline) {
    return std::format("f{}", wire.slot);
  }
  return wire.inner ? std::format("i{}", wire.slot)
                    : std::format("{}", wire.slot);
}

/// A value of a price field as one of eight levels, zero for none.
[[nodiscard]] int levelOf(const int value, const int most) {
  if (value <= 0) {
    return 0;
  }
  return 1 + std::min(7, (7 * value) / std::max(1, most));
}

/// The eight shades of a field, by the values it actually holds.
///
/// A ramp drawn against the largest value in the picture tells the reader
/// nothing once the largest is far from the rest. The prices add up now, so
/// somewhere in a view they reach the 127 the router accepts, and against
/// that every ordinary price is flat: 7 and 14 both come out
/// `1 + 7*14/127 = 1`, and the halo of the wire just let go of — the one
/// thing the addition was for — is the same shade as open ground
/// (user, 2026-10-01).
///
/// Ranking instead gives each value that is really there a shade of its own,
/// and ties the picture to the field rather than to its outlier.
class Shades {
public:
  void gather(const int value) {
    if (value > 0) {
      seen_.push_back(value);
    }
  }
  void settle() {
    std::ranges::sort(seen_);
    const auto twice = std::ranges::unique(seen_);
    seen_.erase(twice.begin(), twice.end());
  }
  [[nodiscard]] int of(const int value) const {
    if (value <= 0 || seen_.empty()) {
      return 0;
    }
    const auto found = std::ranges::lower_bound(seen_, value);
    if (found == seen_.end()) {
      return 8;
    }
    // The low end gets the shades. Spreading the ranks evenly over eight
    // would put the first three values in one shade again where seventeen of
    // them are present, and the distinctions that matter — open ground
    // against the halo of one wire, against two — are all down there. High
    // is high: everything from the eighth value up shares the darkest.
    const auto rank = static_cast<int>(found - seen_.begin());
    return 1 + std::min(7, rank);
  }
  [[nodiscard]] std::size_t count() const { return seen_.size(); }
  [[nodiscard]] std::string say() const {
    std::string out;
    for (const auto value : seen_) {
      out += (out.empty() ? "" : ", ") + std::to_string(value);
    }
    return out;
  }

private:
  std::vector<int> seen_;
};

/// The look of the debug pictures: one class per thing, so that a layer can
/// be switched off in the file. Every fill is translucent, so that what lies
/// under it stays visible.
constexpr std::string_view DEBUG_STYLE =
    // The palette is Okabe-Ito, which is built to stay apart under every
    // common colour vision deficiency, plus one violet (#5D3A9B) that is a
    // known safe partner for orange. What it rules out is the red/green
    // opposition the pictures used to lean on: the band was green and the
    // wire price red, the way found green and the target red, and for a
    // red-green blind reader those were one colour each (user, 2026-10-01).
    //
    // Sixteen things are drawn in the search picture, and no sixteen colours
    // stay apart. So colour carries the four **area** fills, which never
    // share an outline, and every **line** carries a dash signature of its
    // own on top of its colour:
    //
    //   solid thick      the way it found        solid         a feedline
    //   dotted fine      the way it had          long dash     let go of
    //   dash-dot         the coupler box         fine dotted   the fence
    //
    // And the two ends differ in shape as well: the source is a circle, the
    // target a square.
    ".bg{fill:#fff}"
    // --- the four area fills: dark neutral, orange, sky blue, violet-pink
    ".ob{fill:#333a40;fill-opacity:.85}"
    ".co{fill:#56B4E9;fill-opacity:.26}"
    ".s1{fill:#E69F00;fill-opacity:.10}.s2{fill:#E69F00;fill-opacity:.15}"
    ".s3{fill:#E69F00;fill-opacity:.20}.s4{fill:#E69F00;fill-opacity:.25}"
    ".s5{fill:#E69F00;fill-opacity:.31}.s6{fill:#E69F00;fill-opacity:.37}"
    ".s7{fill:#E69F00;fill-opacity:.44}.s8{fill:#E69F00;fill-opacity:.52}"
    ".p1{fill:#CC79A7;fill-opacity:.10}.p2{fill:#CC79A7;fill-opacity:.15}"
    ".p3{fill:#CC79A7;fill-opacity:.20}.p4{fill:#CC79A7;fill-opacity:.25}"
    ".p5{fill:#CC79A7;fill-opacity:.31}.p6{fill:#CC79A7;fill-opacity:.37}"
    ".p7{fill:#CC79A7;fill-opacity:.44}.p8{fill:#CC79A7;fill-opacity:.52}"
    // --- the fence: closed room, so a neutral fine dotted wash
    ".fz{fill:none;stroke:#333a40;stroke-opacity:.34;stroke-linecap:round;"
    "stroke-dasharray:1 3}"
    // --- the feedlines: vermillion, solid when drawn, short-dashed when not
    ".fw{fill:none;stroke:#D55E00;stroke-width:2}"
    ".fwg{fill:none;stroke:#D55E00;stroke-width:1.4;stroke-opacity:.5;"
    "stroke-dasharray:4 3}"
    // --- the wires around: green solid, violet-pink long-dashed, grey dotted
    ".nb{fill:none;stroke:#009E73;stroke-width:1.6}"
    ".rp{fill:none;stroke:#CC79A7;stroke-width:1.6;stroke-dasharray:9 5}"
    ".ow{fill:none;stroke:#6b6b6b;stroke-width:1;stroke-dasharray:1.5 2.5}"
    // --- the way it found: the thickest line in the picture
    ".fd{fill:none;stroke:#0072B2;stroke-width:3}"
    ".sd{fill:none;stroke:#8a8a8a;stroke-width:.9;stroke-opacity:.8;"
    "stroke-dasharray:3 3}"
    // --- the two ends: circle against square, blue against vermillion
    ".src{fill:#0072B2;stroke:#fff;stroke-width:.4}"
    ".tgt{fill:#D55E00;stroke:#fff;stroke-width:.4}"
    // --- the ports: green, as the neighbours, which they belong to
    ".prt{fill:none;stroke:#009E73;stroke-width:1.2}"
    ".prx{fill:#009E73;stroke:#fff;stroke-width:.3}"
    ".dst{fill:none;stroke:#009E73;stroke-width:.8;stroke-dasharray:2 2}"
    ".pra{fill:none;stroke:#009E73;stroke-width:1}"
    ".prl{font-family:monospace;fill:#007a5a}"
    // --- the couplers: violet, the box dash-dotted, the chosen one solid
    ".cbx{fill:none;stroke:#5D3A9B;stroke-width:1.4;stroke-dasharray:10 4 2 4}"
    ".opt{fill:none;stroke:#CC79A7;stroke-width:.8;stroke-opacity:.55}"
    ".opl{fill:none;stroke:#CC79A7;stroke-width:.8;stroke-opacity:.4}"
    ".ops{fill:#CC79A7;fill-opacity:.55;stroke:none}"
    ".ocp{fill:none;stroke:#5D3A9B;stroke-width:2.2}"
    ".ocl{fill:none;stroke:#5D3A9B;stroke-width:2.2}"
    ".ocs{fill:#5D3A9B;stroke:#fff;stroke-width:.4}"
    ".ar{stroke:#1a1a1a;stroke-width:1;fill:none}"
    ".lane{fill:none;stroke:#5D3A9B;stroke-width:1.2;stroke-dasharray:8 4;"
    "stroke-opacity:.9}"
    ".lb{font-family:monospace;fill:#111}"
    ".lg{fill:#fff;fill-opacity:.94;stroke:#6b6b6b;stroke-width:.5}"
    ".lgs{fill:#fff;stroke:#9a9a9a;stroke-width:.35}"
    // --- the bottleneck picture: the obstacles by kind, the medial axis,
    // and the lines by how many wires they hold — vermilion solid, orange
    // dashed, bluish green dotted
    ".wb{fill:#d9d9d9}"
    ".wc{fill:#5D3A9B;fill-opacity:.85}"
    ".wst{fill:none;stroke:#56B4E9;stroke-width:2.4}"
    ".wsz{fill:#56B4E9;fill-opacity:.35}"
    ".wfz{fill:#333a40;fill-opacity:.28}"
    ".wfl{fill:none;stroke:#111;stroke-width:2.4}"
    ".wax{fill:#0072B2;fill-opacity:.35}"
    ".b0{fill:none;stroke:#D55E00;stroke-width:3}"
    ".b1{fill:none;stroke:#E69F00;stroke-width:2.4;stroke-dasharray:6 3}"
    ".b2{fill:none;stroke:#009E73;stroke-width:1.4;stroke-dasharray:2 2}"
    ".bsd{fill:#111}"
    ".btx{font-family:monospace;fill:#a14400}"
    // --- the capacity graph: eight pale chamber fills, the load of an edge
    // as vermilion (over), orange (full) or bluish green (room)
    ".k0{fill:#E69F00;fill-opacity:.16}.k1{fill:#56B4E9;fill-opacity:.18}"
    ".k2{fill:#009E73;fill-opacity:.16}.k3{fill:#F0E442;fill-opacity:.22}"
    ".k4{fill:#0072B2;fill-opacity:.14}.k5{fill:#D55E00;fill-opacity:.12}"
    ".k6{fill:#CC79A7;fill-opacity:.18}.k7{fill:#5D3A9B;fill-opacity:.12}"
    ".gl{fill:none;stroke-width:2.4}.gl-over{stroke:#D55E00;stroke-width:3.4}"
    ".gl-full{stroke:#E69F00;stroke-dasharray:6 3}"
    ".gl-room{stroke:#009E73;stroke-width:1.4;stroke-dasharray:2 2}"
    ".xs{fill:none;stroke-width:7;stroke-opacity:.55;stroke-linecap:round}"
    ".xs-over{stroke:#D55E00}.xs-full{stroke:#E69F00}.xs-room{stroke:#56B4E9}"
    ".ge{fill:none;stroke:#333a40;stroke-width:.8;stroke-opacity:.75}"
    ".gex{stroke-dasharray:4 3}"
    ".gt{font-family:monospace}.gt-over{fill:#a14400;font-weight:bold}"
    ".gt-full{fill:#7a5200}.gt-room{fill:#00664a}"
    ".node{fill:#fff;stroke:#111;stroke-width:1.2}"
    ".nt{font-family:monospace;fill:#111}"
    ".pm{fill:#111;stroke:none}";

/// A port as the chip carries it, on the router grid: the cell its centre
/// falls on, the step its orientation takes, and its label. Not the cell a
/// wire is routed to, which lies beyond the port's band.
struct PortMark {
  std::int64_t x = 0;
  std::int64_t y = 0;
  /// The exact position, in fractional cells: the integer `i` is the centre
  /// of cell `i`, so `x` and `y` are these rounded.
  double fx = 0.0;
  double fy = 0.0;
  grid::Step step;
  std::uint32_t index = 0;
  std::string label;
};

[[nodiscard]] std::vector<PortMark> portMarksOf(const ChipT& chip,
                                                const Scene& scene) {
  std::vector<PortMark> marks;
  marks.reserve(chip.ports.size());
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    const auto cell = scene.router.clampToCell(port.center);
    const auto exact = scene.router.toCell(port.center);
    marks.push_back({.x = cell.x(),
                     .y = cell.y(),
                     .fx = exact.x(),
                     .fy = exact.y(),
                     .step = grid::orientationStep(port.orientation),
                     .index = index,
                     .label = port.label});
  }
  return marks;
}

/// The cells of a way, for the painter.
[[nodiscard]] std::vector<debug::Cell> cellsOf(const Path& way) {
  std::vector<debug::Cell> cells;
  cells.reserve(way.size());
  for (const auto& point : way) {
    cells.emplace_back(point.x, point.y);
  }
  return cells;
}

// -------------------------------------------------------------- The driver

/// The router grid of a run, and every search the stage makes on it.
///
/// One driver runs every routing phase. The prototype writes the loop out four
/// times — once for the inner circuit, once for the ring, once for the wires
/// under the feedline constraints and once for the refinement — and the four
/// differ only in their parameters.
class Driver {
public:
  Driver(const Scene& scene, const Tuning& tuning, Progress progress = {},
         Debug debug = {}, const std::uint32_t verbosity = 0)
      : scene_(scene), tuning_(tuning), progress_(std::move(progress)),
        debug_(std::move(debug)), verbosity_(verbosity),
        primitives_(std::make_shared<const MovePrimitives>(BEND_RADIUS)),
        analytic_(*primitives_),
        scratch_(scene.router.width, scene.router.height),
        router_(primitives_, scratch_,
                {.startStraightLength = 0,
                 .endStraightLength = 0,
                 .minRadius = BEND_RADIUS,
                 .bendPenalty = tuning.bendPenalty}),
        field_(scene.router, tuning.clearance),
        corridor_(scene.router.width, scene.router.height),
        proximity_(scene.router.cells(), 0), seen_(scene.router.cells(), 0),
        bodies_(scene.router.width, scene.router.height),
        approaches_(scene.router.width, scene.router.height) {
    router_.attachObstacles(&scene_.blocked);
    router_.setOrthogonalExitCheck(crossingExitHeading());
    // The price of running beside an obstacle. It forbids nothing — the
    // keepout in the mask already does that — and keeps copper off the artwork
    // wherever there is room for it.
    router_.computeStaticProximity(tuning_.obstacleReach,
                                   tuning_.staticProximityPenalty);
    router_.attachWireProximity(&proximity_);
    // The distance field, not the octile distance. It costs a backward
    // Dijkstra over the corridor per search — 15 to 23 % of the time a
    // search takes — and it earns it back on a fenced corridor, where it
    // stops the search flooding and proves unreachable cells before they are
    // expanded. Measured both ways on the real chips: 17q 23.4 s with the
    // field against 41.3 s with octile, 9q 13.0 against 14.9, same angle
    // cost. On an *empty* grid the ranking reverses and the field is pure
    // overhead — which is why a synthetic benchmark says the opposite.
    router_.setHeuristic(routing::Heuristic::DistanceField);
  }

  [[nodiscard]] const Field& field() const { return field_; }
  [[nodiscard]] const Tuning& tuning() const { return tuning_; }

  /// Say one line, when anybody is listening. The seconds since the driver
  /// was built are on every line, so a slow round is visible as one.
  void say(const std::string& line) const {
    if (!progress_) {
      return;
    }
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began_)
            .count();
    progress_(std::format("[final] {:8.2f}s  {}", seconds, line));
  }

  /// Say one line of detail: what one wire's search came to. Only at the
  /// second level of verbosity, where a run says one line per search.
  void tell(const std::string& line) const {
    if (verbosity_ >= 1) {
      say("  " + line);
    }
  }

  /// Say one line of the coupler insertion's own reasoning: every option a
  /// coupler had, whether it survived being built and what the greedy
  /// priced it at. Only at the third level of verbosity — it is 64 lines
  /// per coupler, which is one chip under a lens and not a run's log.
  void explain(const std::string& line) const {
    if (verbosity_ >= 2) {
      say("    " + line);
    }
  }
  [[nodiscard]] bool explaining() const { return verbosity_ >= 2; }

  /// The picture of the last search, for the line about it, when there is one.
  [[nodiscard]] std::string picture() const {
    return lastPicture_.empty() ? std::string{} : " · " + lastPicture_;
  }

  /// What the grid and the rules came to, once, before anything is drawn.
  void sayTheSetting() const {
    say(std::format(
        "grid {}x{} cells of {:.2f} layout units | clearance {} cells for a "
        "rule of {:.2f} | stub {} cells | band {} cells | bend {} | wire "
        "price {} | obstacle price {} over {} cells | {}",
        scene_.router.width, scene_.router.height,
        std::min(scene_.router.cellWidth, scene_.router.cellHeight),
        tuning_.clearance, tuning_.spacing, tuning_.straightStart,
        tuning_.reach, tuning_.bendPenalty, tuning_.wireProximityPenalty,
        tuning_.staticProximityPenalty, tuning_.obstacleReach,
        tuning_.meanderLength > 0.0
            ? std::format("resonators {:.0f} cells long", tuning_.meanderLength)
            : std::string("no meander")));
  }

  /// Whether a wire has to reach a length: a resonator, while the meander is
  /// switched on; every resonator once it is drawn from its coupler.
  /// Whether a wire is lengthened in this pass: a resonator that needs a
  /// length, and — under the feedline constraints — only while the meander
  /// is switched on there. The refinement has its own switch, because the
  /// fifth phase is where the lengths are read and the sweep's meander is
  /// dear: see `feedlineMeander` and `refineMeander`.
  [[nodiscard]] bool lengthensIn(const Wire& wire, const Pass& pass) const {
    return needsLength(wire) &&
           (!pass.feedlines || feedlineMeander() ||
            (pass.refinement && refineMeander()));
  }

  [[nodiscard]] bool needsLength(const Wire& wire) const {
    return wire.resonator && (exact_ || tuning_.meanderLength > 0.0);
  }

  /// How long a resonator's way has to be, in cells: `meander_length` in the
  /// outer routing and the target length once the coupler is in, each less
  /// the run from the cell the way ends on to the port itself, which the
  /// wire covers without cells of its own.
  [[nodiscard]] double requiredLength(const Wire& wire) const {
    const auto figure = exact_ ? tuning_.targetLength : tuning_.meanderLength;
    return std::max(0.0, figure - wire.anchorGap);
  }

  /// The figure a resonator is measured against, in layout units, and the
  /// tolerance around it: none in the outer routing, the rule's once the
  /// coupler is in.
  [[nodiscard]] double requiredUnits() const {
    return exact_ ? tuning_.targetLengthUnits : tuning_.meanderLengthUnits;
  }
  [[nodiscard]] double toleranceUnits() const {
    return exact_ ? tuning_.lengthToleranceUnits : 0.0;
  }

  /// Switch the length a resonator is made to: at least `meander_length`
  /// before the coupler, the target length exactly after it.
  void aimAtTarget(const bool exact) { exact_ = exact; }

  /// The length of a way as the sampler renders it, in cells.
  [[nodiscard]] double lengthOf(const Path& way) const {
    return routing::renderedLength(*primitives_, way);
  }

  /// The same in layout units, with the two sides of a cell told apart.
  [[nodiscard]] double lengthInUnits(const Path& way) const {
    if (way.empty()) {
      return 0.0;
    }
    Path copy = way;
    std::vector<routing::PathSegment> segments;
    const auto points =
        routing::samplePath(*primitives_, copy, copy.front(), segments);
    double total = 0.0;
    for (std::size_t at = 1; at < points.size(); ++at) {
      total += std::hypot(
          (points[at].x() - points[at - 1].x()) * scene_.router.cellWidth,
          (points[at].y() - points[at - 1].y()) * scene_.router.cellHeight);
    }
    return total;
  }

  /// What lengthening a way came to, and the words for it.
  struct Lengthened {
    bool reached = false;
    bool tooLong = false;
    std::string note;
  };

  /// Lengthen a resonator's way to what it needs: the prototype's meander
  /// insertion, the first placement that fits in phase 1 and the cheapest by
  /// the search's own price field in the relaxation and the refinement. The
  /// meander may enter what the search could enter — the band less the
  /// fence — and nothing else, so it cannot cross a neighbour the way itself
  /// could not.
  /// @param band How wide the length band is, as a multiple of
  ///   `resonator_length_tolerance`: one in the refinement, which aims at
  ///   the length itself, and `sweepLengthBand` in the sweep, which aims
  ///   roughly. See `sweepLengthBand`.
  [[nodiscard]] Lengthened lengthen(const Wire& wire, Path& way,
                                    const bool priced,
                                    const double band = 1.0) {
    const auto required = requiredLength(wire);
    routing::MeanderOptions options{.width = scene_.router.width,
                                    .height = scene_.router.height,
                                    .box = {.minX = 0,
                                            .maxX = scene_.router.width - 1,
                                            .minY = 0,
                                            .maxY = scene_.router.height - 1}};
    if (exact_) {
      // The prototype's strict insertion: the length exactly, within the
      // rule's tolerance, in the widest free strip along the pair, and
      // never in the run the resonator couples along.
      options.exact = true;
      options.tolerance = band * tuning_.lengthTolerance;
      options.startMargin =
          meanderStartMargin(tuning_.couplerLength, tuning_.straightStart);
      options.minStraightLength = meanderLegSpacing();
      options.boxFor = [this](const PathPoint& a, const PathPoint& b) {
        return router_.freeStripAlong(a, b);
      };
    }
    const auto enterable = [this](const std::uint32_t x,
                                  const std::uint32_t y) {
      return x < scene_.router.width && y < scene_.router.height &&
             !corridor_.testCell(x, y);
    };
    routing::CellPrice price;
    if (priced) {
      price = [this](const std::uint32_t x, const std::uint32_t y) {
        const auto cell =
            (static_cast<std::size_t>(y) * scene_.router.width) + x;
        return static_cast<std::uint32_t>(router_.staticPenalty(cell)) +
               proximity_[cell];
      };
    }
    const auto made = routing::insertMeander(*primitives_, way, required,
                                             enterable, options, price);
    Lengthened result{.reached = made.reached, .tooLong = made.tooLong};
    if (made.tooLong) {
      result.note =
          std::format("too long for its target: {:.0f} of {:.0f} cells",
                      made.lengthBefore, required);
    } else if (!made.reached) {
      // What stood in the way, not only how many placements were counted: a
      // loop refused for want of room is a different problem from one no
      // pair of the way lies far enough apart to carry. See
      // `MeanderResult::Refusals`.
      std::string said;
      for (const auto& [reason, times] : made.refusals.said()) {
        said += std::format("{}{} {}", said.empty() ? "" : ", ", times, reason);
      }
      result.note = std::format(
          "no room for a meander: {:.0f} of {:.0f} cells, {} placements "
          "tried over {} straight cells less {} reserved, widest span {} of "
          "the {} a loop needs — {}",
          made.lengthBefore, required, made.candidates, made.straightCells,
          made.reserved, made.widestSpan,
          options.minStraightLength + (2U * primitives_->minRadius()), said);
    } else if (made.inserted) {
      result.note = std::format("meander: {:.0f} → {:.0f} cells for {:.0f}",
                                made.lengthBefore, made.lengthAfter, required);
    } else {
      result.note = std::format("long enough: {:.0f} of {:.0f} cells",
                                made.lengthBefore, required);
    }
    return result;
  }

  /// Put a wire's way down: its copper, and the room it keeps around it. The
  /// way may be one this stage drew or the seed the wire started on; both
  /// hold copper, because both are where the wire runs.
  void place(Wire& wire) {
    if (wire.way.empty()) {
      return;
    }
    if (wire.endsOnly) {
      field_.dischargeFixed(wire.fixed);
      wire.endsOnly = false;
    }
    if (!wire.placed) {
      field_.occupy(wire.way, wire.key);
      field_.charge(wire.way);
      wire.placed = true;
    }
  }

  /// Take a wire off the canvas altogether, for the length of its own search.
  /// Only the wire being drawn is let out of its own two places.
  void lift(Wire& wire) {
    if (wire.placed) {
      field_.discharge(wire.way);
      wire.placed = false;
    } else if (wire.endsOnly) {
      field_.dischargeFixed(wire.fixed);
    }
    wire.endsOnly = false;
    if (!wire.way.empty()) {
      field_.vacate(wire.way, wire.key);
    }
  }

  /// Sweep the wires, drawing each one again from the way it has.
  ///
  /// The sweep is a rip-up and re-route, and a rip-up needs something to rip:
  /// every wire of the pass starts on the way the Detail stage drew, put down
  /// with its copper and its room before the first search, and a wire is
  /// taken off the canvas, offered a way of its own, and put back on the way
  /// it had when it finds none. So no wire is ever without a way, and what a
  /// round sees of the wires it has not reached yet is where they run rather
  /// than empty space. The prototype starts the same way: its `global_paths`
  /// begin as the detailed routing's paths, and a wire that fails keeps its
  /// entry.
  ///
  /// Rounds alternate direction. A wire that has been routed is left alone.
  /// A round ends when every wire has been routed, not when no attempt
  /// failed. The two are not the same: a wire routed early in a round can be
  /// let go of by a wire further along, and letting it go is a promise that it
  /// will be drawn again. Stopping on the failure count alone leaves it on the
  /// canvas with its room uncharged, and the next wire that comes past settles
  /// inside it.
  ///
  /// What a round reports, and what the pass ends on, is its fails: the
  /// wires still on their seed, for which this stage has found no way; the
  /// wires with a way of their own that is not settled against every other
  /// wire; and the resonators whose way is too short for want of room for
  /// their meander. The first are unrouted, the second open, the third
  /// short, and all three count.
  ///
  /// @returns How many wires are unrouted, open or short when the pass ends.
  std::uint32_t sweep(std::vector<Wire>& wires,
                      const std::vector<std::uint32_t>& members,
                      const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    if (total == 0) {
      // A pass with nothing to draw still ends on its summary, so that every
      // pass of every run reads the same way.
      sayFails(pass.name, {}, 0);
      return 0;
    }
    fixPlaces(wires, members, pass);
    seed(wires, members, pass);
    beginPass(wires, members, pass);
    rounds_.clear();
    bridgeRefusals_ = 0;
    auto fewest = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t stale = 0;
    say(std::format("{}: {} wires, up to {} rounds, {} relaxations each way",
                    pass.name, total, pass.rounds, pass.maxRelaxation));
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      const bool forward = (round % 2) == 0;
      frame_.pass = passTag(pass.name);
      frame_.round = round;
      frame_.forward = forward;
      rounds_.push_back({.forward = forward});
      std::uint32_t tried = 0;
      std::uint32_t won = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
        const auto slot = forward ? at : (total - 1 - at);
        auto& wire = wires[members[slot]];
        if (!wire.feasible || (wire.routed && pass.keepDrawn)) {
          continue;
        }
        ++tried;
        won += attempt(wires, members, slot, forward, pass) ? 1 : 0;
      }
      // Unrouted is a wire with no way of its own yet; open is a wire whose
      // own way is not settled — let go of by a wire that relaxed past it and
      // not drawn again yet; short is a resonator that kept a way without
      // room for its meander. All three are fails, and a round with none
      // ends the pass.
      std::uint32_t unrouted = 0;
      std::uint32_t open = 0;
      std::uint32_t tooShort = 0;
      for (const auto member : members) {
        const auto& wire = wires[member];
        if (!wire.feasible) {
          continue;
        }
        unrouted += wire.drawn ? 0 : 1;
        tooShort += (wire.drawn && (wire.tooShort || wire.tooLong)) ? 1 : 0;
        open += (wire.drawn && !wire.tooShort && !wire.tooLong && !wire.routed)
                    ? 1
                    : 0;
      }
      const auto fails = unrouted + open + tooShort;
      say(std::format("{} round {} {}: tried {}, routed {}, unrouted {}, "
                      "open {}, short {} | Fails: {}",
                      pass.name, round, forward ? "forward " : "backward",
                      tried, won, unrouted, open, tooShort, fails));
      if (fails == 0) {
        break;
      }
      // A round that leaves as many fails as the best round before it has
      // moved nothing that the next round can use. The rounds are there for
      // a wire to take room a later one has not claimed yet, and once that
      // has stopped happening they only cost searches: measured on the
      // 21-qubit chip, the count settles by the second round and the
      // remaining twenty-eight change no byte of the result.
      stale = fails < fewest ? 0 : stale + 1;
      fewest = std::min(fewest, fails);
      if (stale >= STALE_ROUNDS) {
        say(std::format("{}: {} rounds without progress, stopping", pass.name,
                        stale));
        break;
      }
    }

    // Everything goes back down, so that the field says what the canvas holds
    // and the next phase starts from the truth.
    for (const auto member : members) {
      place(wires[member]);
    }
    const auto fails = failsOf(wires, members);
    sayFails(pass.name, fails, total);
    if (pass.feedlines && bridgeCheck()) {
      say(std::format("{}: {} way{} refused for not crossing the wire's "
                      "bridge (SCPD_BRIDGE_CHECK)",
                      pass.name, bridgeRefusals_,
                      bridgeRefusals_ == 1 ? "" : "s"));
    }
    // The second level of verbosity ends the pass on what every round came
    // to, wire by wire: how many found a way in phase 1, how many only after
    // relaxation, and which found none.
    if (verbosity_ >= 1) {
      for (std::size_t round = 0; round < rounds_.size(); ++round) {
        const auto& record = rounds_[round];
        std::string failed;
        for (const auto& id : record.failed) {
          failed += (failed.empty() ? " " : ", ") + id;
        }
        say(std::format(
            "{} round {} {}: {} found a way in phase 1, {} after "
            "relaxation, {} failed{}{}",
            pass.name, round, record.forward ? "forward " : "backward",
            record.normal, record.relaxed, record.failed.size(),
            record.failed.empty() ? "" : ":" + failed,
            record.tooShort == 0 ? ""
                                 : std::format(", {} kept a way too short for "
                                               "its meander",
                                               record.tooShort)));
      }
    }
    return fails.total();
  }

  /// What a set of wires comes to, counted with every wire down.
  struct Fails {
    /// Wires with a way of their own.
    std::uint32_t drawn = 0;
    /// Wires without one: still on their seed, or not on the grid at all.
    std::uint32_t unrouted = 0;
    /// Wires whose own way comes within the rule of another wire.
    std::uint32_t open = 0;
    /// Resonators whose way is shorter than `meander_length` asks, or than
    /// the target length allows once the coupler is in.
    std::uint32_t tooShort = 0;
    /// Resonators whose way is longer than the target length allows, once
    /// the coupler is in.
    std::uint32_t tooLong = 0;
    /// Wires that cross a feedline other than at a right angle, or enter
    /// the room around a feedline's bend.
    std::uint32_t crossing = 0;
    /// Wires that cross or touch themselves: a resonator whose kept way
    /// runs back over the head its coupler put in front of it.
    std::uint32_t loops = 0;
    /// Of the unrouted, the edges of the feedline chains.
    std::uint32_t feedlinesUnrouted = 0;
    /// Wires with any of the above, each counted once.
    std::uint32_t failing = 0;
    /// Which ones, for the second level of verbosity.
    std::vector<std::string> unroutedIds;
    std::vector<std::string> openIds;
    std::vector<std::string> shortIds;
    std::vector<std::string> longIds;
    std::vector<std::string> crossingIds;
    std::vector<std::string> loopIds;
    /// The same three the targeted repair reads, by wire key rather than by
    /// name, so that a fail can be looked up without parsing its id.
    std::vector<std::uint32_t> unroutedKeys;
    std::vector<std::uint32_t> openKeys;
    std::vector<std::uint32_t> crossingKeys;
    /// The verdict against every wire of the list, by key, as the bits of
    /// `FinalVerdict`: zero where nothing is held against a wire, and for
    /// every wire outside the members counted. This is what the artifact
    /// carries on its end state, so a picture can mark the wires the stage
    /// left failing by the stage's own count.
    std::vector<std::uint8_t> verdicts;
    [[nodiscard]] std::uint32_t total() const { return failing; }
    /// The figure the targeted repair is judged by: unrouted, open and
    /// crossing, the lengths left out (user, 2026-10-04).
    [[nodiscard]] std::uint32_t bad() const {
      return unrouted + open + crossing;
    }
  };

  /// Count the fails of a set of wires, by the same test the design-rule
  /// check makes and with every wire down — so what it says is what the
  /// check will find, not what the sweep happened to leave open. Each wire
  /// is counted with itself lifted off the canvas, because a wire is always
  /// within the rule of itself. A resonator's length is measured here as
  /// well, by the sampler that measures it for the artifact, so that a way
  /// too short is a fail whatever the sweep believed about it.
  [[nodiscard]] Fails failsOf(std::vector<Wire>& wires,
                              const std::vector<std::uint32_t>& members) {
    Fails fails;
    fails.verdicts.assign(wires.size(), 0);
    const auto hold = [&fails](const Wire& wire, const fba::FinalVerdict bit) {
      if (wire.key < fails.verdicts.size()) {
        fails.verdicts[wire.key] |= static_cast<std::uint8_t>(bit);
      }
    };
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.drawn) {
        ++fails.unrouted;
        ++fails.failing;
        fails.feedlinesUnrouted += wire.feedline ? 1 : 0;
        fails.unroutedIds.push_back(wireId(wire));
        fails.unroutedKeys.push_back(wire.key);
        hold(wire, fba::FinalVerdict::Unrouted);
        continue;
      }
      ++fails.drawn;
      lift(wire);
      const bool open = conflictsOf(wire, wires) != 0;
      if (open && verbosity_ >= 1) {
        say(std::format("  {} is open in the room of {}", wireId(wire),
                        whoBlocks(wire, wires)));
      }
      place(wire);
      const auto length = needsLength(wire) ? lengthOf(wire.way) : 0.0;
      const auto required = requiredLength(wire);
      const auto tolerance = exact_ ? tuning_.lengthTolerance : 0.0;
      wire.tooShort = needsLength(wire) && length < required - tolerance;
      wire.tooLong =
          needsLength(wire) && exact_ && length > required + tolerance;
      const bool crossing = feedlinePass_ && crossesAFeedline(wire, wires);
      if (crossing && verbosity_ >= 1) {
        say(std::format("  {} crosses at {}", wireId(wire),
                        whereItCrosses(wire, wires)));
      }
      // A way that meets itself is rule 3 of the check; a search never
      // returns one, but a resonator that keeps the way its coupler cut
      // may hold one, and the count says so.
      const bool loop = routing::pathSelfIntersects(
          wire.way, scene_.router.width, scene_.router.height, loopScratch_);
      if (open) {
        ++fails.open;
        fails.openIds.push_back(wireId(wire));
        fails.openKeys.push_back(wire.key);
        hold(wire, fba::FinalVerdict::Open);
      }
      if (wire.tooShort) {
        ++fails.tooShort;
        fails.shortIds.push_back(wireId(wire));
        hold(wire, fba::FinalVerdict::Short);
      }
      if (wire.tooLong) {
        ++fails.tooLong;
        fails.longIds.push_back(wireId(wire));
        hold(wire, fba::FinalVerdict::Long);
      }
      if (crossing) {
        ++fails.crossing;
        fails.crossingIds.push_back(wireId(wire));
        fails.crossingKeys.push_back(wire.key);
        hold(wire, fba::FinalVerdict::Crossing);
      }
      if (loop) {
        ++fails.loops;
        fails.loopIds.push_back(wireId(wire));
        hold(wire, fba::FinalVerdict::Loop);
      }
      if (open || wire.tooShort || wire.tooLong || crossing || loop) {
        ++fails.failing;
      }
    }
    return fails;
  }

  /// Whether a wire crosses a feedline other than at a right angle, by the
  /// test the search runs on every move: the constraints of every edge
  /// between two couplers, with the cells around the wire's own coupler
  /// left out, because the edges of its own chain pin there on purpose.
  [[nodiscard]] bool crossesAFeedline(const Wire& wire,
                                      const std::vector<Wire>& wires) const {
    (void)wires;
    return crossingCellOf(wire).has_value();
  }

  /// The first cell of a wire's way that breaks the crossing rule, outside
  /// the reach of its own coupler; nothing when the way holds the rule, when
  /// the rule is off, or when the wire is a feedline edge, which the rule is
  /// built from and never binds.
  [[nodiscard]] std::optional<PathPoint>
  crossingCellOf(const Wire& wire) const {
    if (wire.feedline || !wire.drawn || !orthoCrossing()) {
      return std::nullopt;
    }
    const auto& constraints = router_.crossingConstraints();
    if (constraints.empty()) {
      return std::nullopt;
    }
    const auto own =
        wire.couplerAtSource != NO_OWNER
            ? std::optional<PathPoint>(couplers_[wire.couplerAtSource].anchor)
            : std::nullopt;
    for (std::size_t at = 0; at < wire.way.size(); ++at) {
      const auto& point = wire.way[at];
      if (own.has_value() &&
          std::hypot(static_cast<double>(point.x) - own->x,
                     static_cast<double>(point.y) - own->y) <= couplerReach()) {
        continue;
      }
      // On the heading the way arrives with as well, which is what the
      // search tested: see `CrossingConstraints::allowedArriving`.
      const auto arrived = at == 0 ? point.heading : wire.way[at - 1].heading;
      if (!constraints.allowedArriving(point.x, point.y, point.heading,
                                       arrived)) {
        return point;
      }
    }
    return std::nullopt;
  }

  /// Where a wire first breaks the crossing rule, and which coupler stands
  /// nearest that cell — the question being whether the breaks gather at
  /// couplers of one kind.
  [[nodiscard]] std::string
  whereItCrosses(const Wire& wire, const std::vector<Wire>& wires) const {
    const auto& constraints = router_.crossingConstraints();
    const auto own =
        wire.couplerAtSource != NO_OWNER
            ? std::optional<PathPoint>(couplers_[wire.couplerAtSource].anchor)
            : std::nullopt;
    for (std::size_t at = 0; at < wire.way.size(); ++at) {
      const auto& point = wire.way[at];
      if (own.has_value() &&
          std::hypot(static_cast<double>(point.x) - own->x,
                     static_cast<double>(point.y) - own->y) <= couplerReach()) {
        continue;
      }
      const auto arrived = at == 0 ? point.heading : wire.way[at - 1].heading;
      if (constraints.allowedArriving(point.x, point.y, point.heading,
                                      arrived)) {
        continue;
      }
      // Which edge it is crossing: the drawn chain edge with a cell nearest
      // the offending one.
      const auto [nearest, span] = nearestEdgeTo(wires, point);
      if (nearest == edges_.size()) {
        return std::format("({},{})", point.x, point.y);
      }
      // The couplers at that edge's two ends, and how each stands.
      const auto& chain = chains_[edges_[nearest].chain];
      const auto describe = [this](const Waypoint& point) {
        if (point.coupler == NO_OWNER) {
          return std::string("launcher");
        }
        const auto& option =
            couplers_[point.coupler].options[couplers_[point.coupler].chosen];
        return std::format(
            "coupler {} ({})", point.coupler,
            (!option.way.empty() &&
             routing::isDiagonal(option.way.front().heading))
                ? "diagonal"
                : "axial");
      };
      return std::format(
          "({},{}), {:.0f} cells from edge {}, which runs {} -> {}", point.x,
          point.y, span, wireId(wires[edges_[nearest].wire]),
          describe(chain[edges_[nearest].from]),
          describe(chain[edges_[nearest].to]));
    }
    return "nowhere";
  }

  /// How far a cell lies from a drawn chain edge: the least distance from
  /// the cell to any cell of the edge's way. Brute force over the way —
  /// a few hundred cells — which is what every caller can afford.
  [[nodiscard]] static double distanceToWay(const PathPoint& cell,
                                            const Path& way) {
    double least = std::numeric_limits<double>::max();
    for (const auto& at : way) {
      least = std::min(least, std::hypot(static_cast<double>(cell.x) - at.x,
                                         static_cast<double>(cell.y) - at.y));
    }
    return least;
  }

  /// The drawn chain edge with a cell nearest a cell, by edge index, and how
  /// far that cell is; `edges_.size()` when no edge is drawn. The first edge
  /// in edge order wins a tie, as the loop this was factored out of had it.
  [[nodiscard]] std::pair<std::size_t, double>
  nearestEdgeTo(const std::vector<Wire>& wires, const PathPoint& cell) const {
    std::size_t nearest = edges_.size();
    double span = std::numeric_limits<double>::max();
    for (std::size_t at = 0; at < edges_.size(); ++at) {
      const auto& edge = wires[edges_[at].wire];
      if (!edge.drawn) {
        continue;
      }
      const auto far = distanceToWay(cell, edge.way);
      if (far < span) {
        span = far;
        nearest = at;
      }
    }
    return {nearest, span};
  }

  /// Every drawn chain edge whose way comes within `reach` cells of a cell,
  /// by edge index with its distance, nearest first. Brute force over
  /// `edges_` × way — some 25 000 distances a query on 45q — which is
  /// nothing next to one search.
  [[nodiscard]] std::vector<std::pair<std::size_t, double>>
  edgesNear(const std::vector<Wire>& wires, const PathPoint& cell,
            const double reach) const {
    std::vector<std::pair<std::size_t, double>> near;
    for (std::size_t at = 0; at < edges_.size(); ++at) {
      const auto& edge = wires[edges_[at].wire];
      if (!edge.drawn) {
        continue;
      }
      const auto far = distanceToWay(cell, edge.way);
      if (far <= reach) {
        near.emplace_back(at, far);
      }
    }
    std::ranges::stable_sort(near, {}, [](const auto& entry) {
      return entry.second;
    });
    return near;
  }

  /// Which switches the outer routing runs under, said once before it.
  void sayOuterSettings() const {
    const auto from = [](const char* name) {
      return envSet(name) != nullptr ? "env" : "default";
    };
    say(std::format(
        "outer routing settings: length-point clearance k {} cells ({}), "
        "band ±{:.0f}% of the target length ({}), report {} ({}); hard — a "
        "wire that cannot be routed with it fails, recovered without it at "
        "the end {} ({}); relaxation priced by the {} ({}), the released "
        "wires {} ({})",
        lengthPointK(), from("SCPD_LENPOINT_K"), lengthPointPct() * 100.0,
        from("SCPD_LENPOINT_PCT"), lengthPointReport() ? "yes" : "no",
        from("SCPD_LENPOINT_REPORT"), lengthPointRecovery() ? "yes" : "no",
        from("SCPD_LENPOINT_RECOVERY"),
        outerLane() ? (outerPenalty() == 2 ? "lane polygon alone"
                                           : "lane polygon and the passing penalty")
                    : (outerPenalty() == 2 ? "nothing — no polygon, no penalty"
                                           : "passing penalty alone"),
        from("SCPD_OUTER_LANE"),
        outerPenalty() == 2   ? "not priced"
        : outerPenalty() == 1 ? "at the feedline pass's flat halo and toll"
                              : "at one clearance more per level",
        from("SCPD_OUTER_PENALTY")));
  }

  /// The recovery at the end of the outer routing: the wires the rounds left
  /// unrouted or open, searched once more in a pass over the ring with the
  /// length-point bands off. `beginPass` clears `routed` on no member of an
  /// outer pass and the sweep skips a routed member under `keepDrawn`, so
  /// only the failing wires are drawn; every other wire stands as the fence
  /// of the ones redrawn. Nothing happens when nothing failed.
  void recoverWithoutBands(std::vector<Wire>& wires,
                           const std::vector<std::uint32_t>& members,
                           const Pass& pass) {
    if (!lengthPointRecovery() || lengthPointK() == 0) {
      return;
    }
    std::uint32_t failing = 0;
    for (const auto member : members) {
      const auto& wire = wires[member];
      failing += (wire.feasible && (!wire.drawn || !wire.routed)) ? 1 : 0;
    }
    if (failing == 0) {
      say(std::format("{}: nothing to recover without the length-point "
                      "clearance",
                      pass.name));
      return;
    }
    Pass recovery = pass;
    recovery.name = "outer recovery";
    lengthPointActive_ = false;
    const auto left = sweep(wires, members, recovery);
    lengthPointActive_ = true;
    say(std::format("{}: recovery without the length-point clearance "
                    "(SCPD_LENPOINT_RECOVERY): {} wire{} failing before it, "
                    "{} after",
                    pass.name, failing, failing == 1 ? "" : "s", left));
  }

  /// The length-point report, the prototype's `[LENPT]` line: for every
  /// resonator the nearest other wire of the ring to its length point, the
  /// same-port siblings left out as the stamp leaves them out, capped at a
  /// hundred cells; the least, the mean, how many lie below `k`, and a
  /// histogram at fixed edges so that arms at different `k` and the `k = 0`
  /// baseline compare directly. One `tell` per resonator at `-v 1`.
  void sayLengthPoints(const std::vector<Wire>& wires,
                       const std::vector<std::uint32_t>& members) const {
    if (!lengthPointReport()) {
      return;
    }
    constexpr double CAP = 100.0;
    const auto k = lengthPointK();
    const double guard = std::max(static_cast<double>(k) + 2.0,
                                  1.5 * static_cast<double>(tuning_.clearance));
    struct Hit {
      std::uint32_t wire = 0;
      double nearest = CAP;
      std::string names;
    };
    std::vector<Hit> hits;
    std::uint32_t unknown = 0;
    for (const auto member : members) {
      const auto& wire = wires[member];
      if (!wire.resonator || !wire.feasible) {
        continue;
      }
      const auto marks = lengthMarksOf(wire);
      if (!marks.valid) {
        ++unknown;
        continue;
      }
      const auto& point = wire.way[marks.pt];
      std::vector<std::pair<double, std::uint32_t>> near;
      for (const auto other : members) {
        if (other == member || wires[other].way.empty()) {
          continue;
        }
        const auto& o = wires[other];
        bool sibling = false;
        for (const auto* const end : {&o.objective.source, &o.objective.target}) {
          sibling = sibling ||
                    std::hypot(static_cast<double>(point.x) - end->x,
                               static_cast<double>(point.y) - end->y) <= guard;
        }
        if (sibling) {
          continue;
        }
        const auto far = distanceToWay(point, o.way);
        if (far < CAP) {
          near.emplace_back(far, other);
        }
      }
      std::ranges::sort(near);
      Hit hit{.wire = member, .nearest = near.empty() ? CAP : near.front().first,
              .names = {}};
      for (std::size_t n = 0; n < near.size() && n < 3; ++n) {
        hit.names += std::format("{}{} at {:.1f}", n == 0 ? "" : ", ",
                                 wireId(wires[near[n].second]), near[n].first);
      }
      if (verbosity_ >= 1) {
        tell(std::format("length point of {} at ({},{}), band {} cells, "
                         "nearest: {}",
                         wireId(wire), point.x, point.y,
                         marks.hi - marks.lo + 1,
                         hit.names.empty() ? std::string("none within 100")
                                           : hit.names));
      }
      hits.push_back(std::move(hit));
    }
    double sum = 0.0;
    double least = CAP;
    std::uint32_t below = 0;
    const std::array<double, 6> edges{20.0, 25.0, 30.0, 40.0, 60.0, 100.0};
    std::array<std::uint32_t, 7> buckets{};
    for (const auto& hit : hits) {
      sum += hit.nearest;
      least = std::min(least, hit.nearest);
      below += hit.nearest < static_cast<double>(k) ? 1 : 0;
      std::size_t b = 0;
      while (b < edges.size() && hit.nearest >= edges[b]) {
        ++b;
      }
      ++buckets[b];
    }
    say(std::format(
        "outer routing: LENPT k={} band=±{:.0f}%: {} length points ({} "
        "unknown), nearest other wire min {:.1f} mean {:.1f} cells, {} below "
        "k; histogram <20:{} 20-25:{} 25-30:{} 30-40:{} 40-60:{} 60-100:{} "
        ">=100:{}",
        k, lengthPointPct() * 100.0, hits.size(), unknown,
        hits.empty() ? -1.0 : least,
        hits.empty() ? -1.0 : sum / static_cast<double>(hits.size()), below,
        buckets[0], buckets[1], buckets[2], buckets[3], buckets[4],
        buckets[5], buckets[6]));
  }

  /// Say how long every resonator is, in layout units: its way as the
  /// sampler renders it, the run from the way's last cell to the port, and
  /// the two together against `meander_length`. One line per resonator, in
  /// the order of the ring.
  void sayResonatorLengths(const std::vector<Wire>& wires,
                           const std::vector<std::uint32_t>& members) const {
    if (tuning_.meanderLength <= 0.0) {
      return;
    }
    std::uint32_t resonators = 0;
    std::uint32_t short_ = 0;
    double shortest = std::numeric_limits<double>::max();
    double longest = 0.0;
    for (const auto member : members) {
      const auto& wire = wires[member];
      if (!wire.resonator || !wire.feasible) {
        continue;
      }
      ++resonators;
      const auto label = wire.targetPort < ports_.size()
                             ? ports_[wire.targetPort].label
                             : std::string("?");
      if (!wire.drawn) {
        ++short_;
        say(std::format("resonator {} to {}: not drawn", wireId(wire), label));
        continue;
      }
      const auto way = lengthInUnits(wire.way);
      const auto total = way + wire.anchorGapUnits;
      const auto figure = requiredUnits();
      const auto tolerance = toleranceUnits();
      const bool enough = total >= figure - tolerance;
      const bool over = exact_ && total > figure + tolerance;
      short_ += (enough && !over) ? 0 : 1;
      shortest = std::min(shortest, total);
      longest = std::max(longest, total);
      say(std::format("resonator {} to {}: {:.0f} units of way + {:.0f} to "
                      "the port = {:.0f} units, {} {:.0f}{}",
                      wireId(wire), label, way, wire.anchorGapUnits, total,
                      over ? "over" : (enough ? "meets" : "short of"), figure,
                      exact_ ? std::format(" within {:.0f}", tolerance) : ""));
    }
    if (resonators == 0) {
      return;
    }
    say(std::format(
        "resonators: {} in all, {:.0f} to {:.0f} units, {} off "
        "{:.0f}{}",
        resonators, shortest, longest, short_, requiredUnits(),
        exact_ ? std::format(" by more than {:.0f}", toleranceUnits()) : ""));
  }

  /// The summary line of a pass, or of the stage.
  void sayFails(const std::string& name, const Fails& fails,
                const std::uint32_t total) const {
    say(std::format(
        "{}: {} of {} drawn, {} unrouted{}, {} open, {} short{}{}{} "
        "| Fails: {}",
        name, fails.drawn, total, fails.unrouted,
        fails.feedlinesUnrouted == 0
            ? std::string{}
            : std::format(" ({} feedline edges)", fails.feedlinesUnrouted),
        fails.open, fails.tooShort,
        exact_ ? std::format(", {} long", fails.tooLong) : std::string{},
        feedlinePass_ ? std::format(", {} crossing a feedline", fails.crossing)
                      : std::string{},
        fails.loops == 0 ? std::string{}
                         : std::format(", {} meeting itself", fails.loops),
        fails.total()));
    if (verbosity_ >= 1 && fails.total() != 0) {
      const auto join = [](const std::vector<std::string>& ids) {
        std::string joined;
        for (const auto& id : ids) {
          joined += (joined.empty() ? "" : ", ") + id;
        }
        return joined.empty() ? std::string("-") : joined;
      };
      say(std::format("{}: unrouted: {} · open: {} · short: {} · long: {} · "
                      "crossing: {} · meeting itself: {}",
                      name, join(fails.unroutedIds), join(fails.openIds),
                      join(fails.shortIds), join(fails.longIds),
                      join(fails.crossingIds), join(fails.loopIds)));
    }
  }

  /// Route every wire again with a price on the room it leaves, so that a
  /// wire that has room to spare moves into the middle of it. A wire that
  /// cannot be routed under the wider constraint keeps the way it had.
  void refine(std::vector<Wire>& wires,
              const std::vector<std::uint32_t>& members, const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    if (pass.rounds != 0) {
      say(std::format("{}: {} wires, {} rounds", pass.name, total,
                      pass.rounds));
      if (pass.refinement && pass.feedlines) {
        say(std::format("{} settings: meander {} (SCPD_REFINE_MEANDER {}, "
                        "SCPD_FEEDLINE_MEANDER {}), fallback {}, {} pair{} "
                        "fenced, the room price {} the feedline constraints, "
                        "checks {}",
                        pass.name,
                        (refineMeander() || feedlineMeander()) ? "on" : "off",
                        refineMeander() ? 1 : 0, feedlineMeander() ? 1 : 0,
                        refineFallback() ? "on" : "off", refineFencePairs(),
                        refineFencePairs() == 1 ? "" : "s",
                        refinePriceLast() ? "after" : "before",
                        refineChecks() ? "on" : "off"));
      }
    }
    // Has this resonator ever been at its length in this pass? The
    // prototype's `fitted`, and the one thing the fallback is gated on: a
    // resonator that has reached it rolls back onto a way that is still at
    // its length, so a second search buys nothing. See `refineFallback`.
    std::vector<std::uint8_t> fitted(wires.size(), 0);
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      std::uint32_t fallbacksTried = 0;
      std::uint32_t fallbacksTaken = 0;
      const bool forward = (round % 2) == 0;
      frame_.pass = passTag(pass.name);
      frame_.round = round;
      frame_.forward = forward;
      std::uint32_t moved = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
        const auto slot = forward ? at : (total - 1 - at);
        auto& wire = wires[members[slot]];
        if (!wire.drawn || !wire.feasible) {
          continue;
        }
        const auto had = wire.way;
        const auto& before = wires[members[(slot + total - 1) % total]];
        const auto& after = wires[members[(slot + 1) % total]];
        lift(wire);
        buildCorridor(wire, pass.reach);
        // The ring neighbours, `refineFencePairs` pairs of them: the pair
        // beside the wire first, so that one pair is the fence the phase was
        // written with, cell for cell.
        for (std::uint32_t pair = 1; pair <= refineFencePairs(); ++pair) {
          if (2 * pair >= total) {
            break;
          }
          fence(wire, {&wires[members[(slot + total - pair) % total]]});
          fence(wire, {&wires[members[(slot + pair) % total]]});
        }
        // The outer refinement keeps the length-point clearance too, or it
        // would level the room the rounds won (the prototype turned it back
        // on there for that reason, `FinalGrid.cpp:7628`). Nothing under
        // the feedline constraints.
        closeLengthBands(wire, {&before, &after}, pass);
        // The chamfer to the middle of the channel reads the corridor, so it
        // is built after everything that closes a cell of it — the fences of
        // the chain edges and the terminal edges' coupler runs included. See
        // `refinePriceLast`.
        if (!refinePriceLast()) {
          priceRoom();
        }
        constrainByFeedlines(wire, wires, pass);
        if (refinePriceLast()) {
          priceRoom();
          // `priceRoom` fills the field, so the 5× discs around the chain
          // edges `constrainByFeedlines` stamped are gone with it; they go
          // back on top, which is the order the prototype stamps them in
          // (`compute_corridor_proximity_decay`, then
          // `compute_proximity_grid`).
          priceTheEdges(wire, wires, pass);
        }
        frame_.wires = &wires;
        frame_.before = before.key;
        frame_.after = after.key;
        frame_.kind = "refine";
        frame_.fence = {before.key, after.key};
        frame_.ripped.clear();
        auto found =
            search(wire, pass.straightStart, true, !lengthensIn(wire, pass));
        std::string note;
        // Whether the way in hand is at this resonator's length, which is
        // what `fitted` records once the way is actually taken.
        bool atLength = false;
        if (lengthensIn(wire, pass)) {
          // A resonator is lengthened here as it is in the relaxation, and
          // keeps the way it had when the wider way has no room for it.
          std::string said;
          bool reached = true;
          if (!found.empty()) {
            const auto made = lengthen(wire, found, true);
            said = made.note;
            note = " · " + said;
            reached = made.reached;
          }
          drawSearch(wire, found, said);
          if (!reached) {
            found.clear();
          }
          // The prototype's fallback: the way that was found is a detour
          // around the neighbours and a detour has no slack, so the search
          // is made once more with the room price taken off and the hard
          // corridor standing. Only while this resonator has never been at
          // its length. See `refineFallback`.
          if (found.empty() && refineFallback() && fitted[wire.key] == 0) {
            ++fallbacksTried;
            // Nothing reads the price again in this iteration, and the next
            // wire's `priceRoom` fills it from scratch, so it is zeroed
            // rather than saved and put back: a grid-sized copy per failed
            // resonator is 24 MB on the 69-qubit chip. The prototype does
            // not restore it either.
            std::ranges::fill(proximity_, 0);
            auto again = search(wire, pass.straightStart, true, false);
            std::string saidAgain("no way");
            bool reachedAgain = false;
            if (!again.empty()) {
              const auto made = lengthen(wire, again, true);
              saidAgain = made.note;
              reachedAgain = made.reached;
            }
            drawSearch(wire, again, saidAgain);
            if (reachedAgain) {
              found = std::move(again);
              ++fallbacksTaken;
            }
            note += std::format(" · without the room price: {}", saidAgain);
          }
          atLength = !found.empty();
        }
        tell(std::format("wire {} · round {} {} · refine: {}{}{}", wireId(wire),
                         round, forward ? "forward" : "backward",
                         found.empty()
                             ? std::string("kept its way")
                             : std::format("found {} cells", found.size()),
                         note, picture()));
        // The prototype's refinement takes what it finds, and falls back to
        // the way it had when the wider constraint does not route. Here a
        // way found is taken only when it is not worse than the way the
        // wire had, counted against every other wire with the wire lifted,
        // and when it crosses no feedline other than at a right angle:
        // under the feedline constraints a wider way is fenced by the two
        // neighbours and the edges alone, and a way that takes another
        // wire's room is not wider, it is wrong.
        if (!found.empty()) {
          const auto before = conflictsIn(had, wire, wires);
          const auto after = conflictsIn(found, wire, wires);
          bool crossing = false;
          if (feedlinePass_) {
            wire.way = found;
            crossing = crossesAFeedline(wire, wires);
          }
          if (after > before || crossing) {
            tell(std::format("wire {} · refine: the way found is worse, {} against {} "
                             "conflicting cells{}; kept its way",
                             wireId(wire), after, before,
                             crossing ? ", or crosses a feedline" : ""));
            found.clear();
          }
        }
        wire.way = found.empty() ? had : found;
        moved += found.empty() ? 0 : 1;
        // Only a way that is taken makes a resonator fitted. A way at its
        // length that the test above refused leaves the wire on the way it
        // had, which this pass has not measured, so the fallback is offered
        // again next round — one search more than the prototype pays, and
        // the conservative way round.
        if (atLength && !found.empty()) {
          fitted[wire.key] = 1;
        }
        wire.drawn = true;
        place(wire);
      }
      std::uint32_t crowded = 0;
      for (const auto member : members) {
        auto& wire = wires[member];
        if (!wire.drawn) {
          continue;
        }
        lift(wire);
        crowded += conflictsOf(wire, wires) == 0 ? 0 : 1;
        place(wire);
      }
      say(std::format("{} round {} {}: moved {} of {}, {} too close to "
                      "another{}",
                      pass.name, round, forward ? "forward " : "backward",
                      moved, total, crowded,
                      fallbacksTried == 0
                          ? std::string{}
                          : std::format(", {} of {} searches without the room "
                                        "price took their length",
                                        fallbacksTaken, fallbacksTried)));
    }
    // The guarantees of the insertion, said again on what the refinement
    // left: it moves feedline edges, and the insertion's lines were about
    // the edges as it drew them. See `refineChecks`.
    if (pass.rounds != 0 && pass.feedlines && pass.refinement &&
        refineChecks()) {
      static_cast<void>(checkFeedlineRoom(wires, pass.name));
      static_cast<void>(checkCouplerCrossings(wires, pass.name));
      static_cast<void>(checkResonatorCrossings(wires, pass.name));
      static_cast<void>(checkFeedlineCrossings(wires, pass.name));
    }
  }

  // ---------------------------------------------------------- The couplers

  /// One way a coupler can sit on its resonator: the prototype's
  /// `CouplerOption`, with the geometry this stage builds from it.
  ///
  /// The resonator's way is cut where the way left to the qubit is the
  /// target length and a dogleg is put in front: from the anchor the way
  /// runs the coupling length straight on the heading across the coupler's
  /// orientation, turns a quarter onto the orientation, runs the straight
  /// start and joins the way. The body spans the coupling run,
  /// `couplerHeight` cells across on the side away from the turn, so that
  /// the turn never crosses the feedline, and the feedline runs along its
  /// far edge, across the orientation too: along the ring, where the chains
  /// run, as the prototype's body lies. The feedline may run either way
  /// along it.
  struct CouplerOption {
    /// The orientation the pad takes: the heading the way holds at the
    /// place, turned by the offset.
    Heading orientation = 0;
    /// Which of the eight turns off that heading this option is.
    Heading offset = 0;

    /// The coupling run and the depth of the body, in cells on this
    /// orientation: the design figure in layout units, which along a diagonal
    /// is fewer cells than along an axis because a diagonal step is longer by
    /// the square root of two.
    std::uint32_t run = 0;
    std::uint32_t depth = 0;
    /// The resonator's way from the anchor to the qubit.
    Path way;
    /// The anchor: the resonator's port on the pad, where its mandatory
    /// quarter turn off the coupler begins.
    PathPoint anchor;
    /// The centre of the pad, which is the cell the placement rule names.
    PathPoint centre;
    /// Which of the two resonator ports the resonator leaves from.
    bool secondPort = false;
    /// The second dogleg, the prototype's `second_straight` /
    /// `second_reverse`: after the arc and its straight, one more quarter
    /// turn and one more straight before the lead meets the path. Zero for
    /// none. It is what lets a coupler slide sideways off the way it sits
    /// on — the same pad, reached from one side or the other.
    std::uint32_t secondStraight = 0;
    bool secondReverse = false;
    /// **The coupler's orientation**: the direction the straight run takes
    /// after the arc off the resonator port. Not the pad's long axis, which
    /// is `orientation` and is the axis the feedline runs along — this is
    /// the direction the coupler points the resonator in, and it is what the
    /// legality rule binds and what the layout is drawn on.
    Heading couplerOrientation = 0;
    /// The coupler's two resonator ports: the ends of the near edge,
    /// parallel to the feedline pair and opposite it. The resonator leaves
    /// the first with the clock and would leave the second against it.
    PathPoint resonatorIn;
    PathPoint resonatorOut;
    /// Where the quarter turn ends, on the orientation: the source of every
    /// search of the resonator from then on.
    PathPoint arcEnd;
    /// The cells of the coupling run and the turn, the arc end excluded.
    Path arc;
    /// The cells of the body.
    std::vector<std::size_t> body;
    /// Where the feedline arrives and where it leaves, with its heading.
    PathPoint in;
    PathPoint out;
    /// What the option cost the last time it was priced: the two edges to
    /// its chain neighbours.
    std::uint32_t cost = std::numeric_limits<std::uint32_t>::max();
    /// How many cells of the body and the run lie in the room of another
    /// wire. Not a rejection — every wire is drawn again under the feedline
    /// constraints — but a price.
    std::uint32_t guarded = 0;
  };

  /// A coupler: the resonator it belongs to and every way it can sit.
  struct Coupler {
    std::uint32_t wire = NO_OWNER;
    std::vector<CouplerOption> options;
    std::size_t chosen = 0;
    /// The anchor of the chosen option, for the meeting exemption.
    PathPoint anchor;
    /// The resonator's way before the cut, which every option is cut from.
    Path outerWay;
    /// Whether the options with a second dogleg are open to this coupler.
    ///
    /// They are not, to begin with. A jog slides the pad sideways and there
    /// are two of them per offset and port, so opening them everywhere
    /// trebles what the greedy has to route and hands it choices it cannot
    /// judge — its cost measures the feedline's turning and knows nothing of
    /// the resonator. So a coupler gets them only once it has shown it needs
    /// them: a pass in which not one of its plain options brought both of
    /// its feedline edges home unlocks them for the passes after.
    bool jogsUnlocked = false;
    /// The feedline edges into and out of it, by wire key.
    std::uint32_t edgeIn = NO_OWNER;
    std::uint32_t edgeOut = NO_OWNER;
  };

  /// A point of a feedline chain: a launcher, or a coupler with its options.
  struct Waypoint {
    bool fixed = false;
    std::uint32_t coupler = NO_OWNER;
    std::uint32_t port = NO_OWNER;
    /// The launcher as a source and as a target.
    PathPoint asSource;
    PathPoint asTarget;
  };

  /// One edge of a chain, and the feedline wire that draws it.
  struct Edge {
    std::uint32_t chain = 0;
    std::size_t from = 0;
    std::size_t to = 0;
    std::uint32_t wire = NO_OWNER;
    bool terminal = false;
  };

  [[nodiscard]] const std::vector<Coupler>& couplers() const {
    return couplers_;
  }
  [[nodiscard]] const std::vector<Edge>& edges() const { return edges_; }

  /// What `reportSqueeze` found on one edge: the figures in words and the
  /// worst line it measured, from the edge's cell to the wall it hit.
  struct Squeeze {
    std::string note;
    PathPoint from;
    PathPoint to;
  };
  [[nodiscard]] const std::unordered_map<std::uint32_t, Squeeze>&
  squeezed() const {
    return squeezed_;
  }
  [[nodiscard]] const std::vector<std::vector<Waypoint>>& chains() const {
    return chains_;
  }

  /// The straight run a feedline holds along a coupler's body: the coupling
  /// run of the option that coupler stands on, which is fewer cells along a
  /// diagonal than along an axis. The straight start where there is no
  /// coupler, so that a launcher keeps the run the rule asks for.
  [[nodiscard]] std::uint32_t couplerRunOf(const std::uint32_t coupler) const {
    if (coupler == NO_OWNER || coupler >= couplers_.size()) {
      return tuning_.straightStart;
    }
    const auto& found = couplers_[coupler];
    return found.options[found.chosen].run;
  }

  /// The straight runs an edge has no choice about: the one it leaves its
  /// source on and the one it arrives at its target on. `routeEdge` hands
  /// these to the search and `corridorOfEdge` has to leave them open, so
  /// they are said once here rather than twice.
  [[nodiscard]] std::pair<std::uint32_t, std::uint32_t>
  runsOfEdge(const Edge& edge) const {
    const auto& chain = chains_[edge.chain];
    return {(couplerStubs() && !chain[edge.from].fixed)
                ? couplerRunOf(chain[edge.from].coupler)
                : tuning_.straightStart,
            chain[edge.to].fixed ? tuning_.straightStart
                                 : couplerRunOf(chain[edge.to].coupler)};
  }

  /// The source of a waypoint as the edge that leaves it sees it, and the
  /// target as the edge that reaches it sees it.
  [[nodiscard]] PathPoint sourceOf(const Waypoint& point) const {
    if (point.fixed) {
      return point.asSource;
    }
    const auto& coupler = couplers_[point.coupler];
    return coupler.options[coupler.chosen].out;
  }
  [[nodiscard]] PathPoint targetOf(const Waypoint& point) const {
    if (point.fixed) {
      return point.asTarget;
    }
    const auto& coupler = couplers_[point.coupler];
    // The end the chain arrives at, not the one it leaves on: between the two
    // it runs the length of the pad, which is what couples it.
    return coupler.options[coupler.chosen].in;
  }

  /// Phase 3: put a coupler on every resonator and route the feedline chains
  /// through them.
  ///
  /// The prototype's `run_optimized_cpw_coupler_insertion`. For every
  /// resonator the options are built: the pad along the way at the places the
  /// biased target names, rejected when the pad or the feedline's run along
  /// it leaves the grid or meets the artwork. Then
  /// every chain is optimized by a greedy local search: a coupler takes the
  /// option whose two feedline edges to its chain neighbours turn least,
  /// until no coupler improves or five passes are done. The chosen options
  /// are committed — the resonator cut back to its coupler, the body an
  /// obstacle, the edges drawn — and what came of it is said.
  void insertCouplers(std::vector<Wire>& wires, const AssignmentT& assignment,
                      const ChipT& chip) {
    const auto began = std::chrono::steady_clock::now();
    couplers_.clear();
    chains_.clear();
    edges_.clear();
    familyStats_ = {};
    familyViolations_ = 0;
    learnedRaised_ = 0;
    learnedAbove_ = 0;
    budgetCutoffs_ = 0;
    stepBudget_ = routing::TRELLIS_UNREACHABLE;
    room_ = RoomStats{};
    bodies_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    aimAtTarget(true);
    couplerBox_ = couplerBoxOf();
    say(std::format(
        "couplers sit inside x {}..{} y {}..{}, which is what the "
        "launcher stubs leave open at a run of {} cells inflated "
        "by {} ({} cells of lead over the stub itself); a place "
        "is taken where one of the eight orientations puts both "
        "feedline ports, their runs and the {} cells of the turn "
        "after them inside it; an edge between two couplers {} "
        "(SCPD_EDGE_IN_BOX)",
        couplerBox_.minX, couplerBox_.maxX, couplerBox_.minY, couplerBox_.maxY,
        couplerBoxTurn() ? std::max(tuning_.launcherStraight,
                                    tuning_.straightStart + BEND_RADIUS)
                         : tuning_.launcherStraight,
        tuning_.clearance, tuning_.launcherStraight - tuning_.straightStart,
        couplerBoxTurn() ? std::uint32_t{BEND_RADIUS} : 0U,
        edgeInBox() ? "stays inside it as well" : "may leave it"));
    if (couplerBoxMargin() > 0) {
      say(std::format(
          "a coupler keeps {} cells more off every side of that box "
          "(SCPD_COUPLER_BOX_MARGIN): x {}..{} y {}..{}",
          couplerBoxMargin(), couplerBox_.minX + couplerBoxMargin(),
          couplerBox_.maxX - couplerBoxMargin(),
          couplerBox_.minY + couplerBoxMargin(),
          couplerBox_.maxY - couplerBoxMargin()));
    }
    if (edgeLauncherMargin() > 0) {
      say(std::format(
          "a feedline edge keeps {} cells closed in front of every launcher "
          "but its own: the run of {} cells, {} cells of margin "
          "(SCPD_EDGE_LAUNCHER_MARGIN), and the clearance around them",
          static_cast<std::int64_t>(tuning_.straightStart) +
              (launcherFenceTurn() ? static_cast<std::int64_t>(BEND_RADIUS)
                                   : 0) +
              edgeLauncherMargin(),
          static_cast<std::int64_t>(tuning_.straightStart) +
              (launcherFenceTurn() ? static_cast<std::int64_t>(BEND_RADIUS)
                                   : 0),
          edgeLauncherMargin()));
    }

    // The approaches of every port, inflated by the clearance: no coupler
    // may sit in one and no feedline edge may run through one, because the
    // wire into that port has nowhere else to go. So the approach reaches
    // from the port out past the cell the search ends on by the straight
    // start it has to arrive on. A resonator's feed is a point on the ring
    // and not a port, so only its target counts.
    approaches_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    {
      const auto& stencil = stencilFor(tuning_.clearance);
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      // Only the straight run the wire has to arrive on. The crossing band
      // used to be added here as well, to keep a feedline far enough from a
      // port that the wire into it could still cross at a right angle — but
      // the insertion has no orthogonal crossing rule, so that band was a
      // rule from another phase enforced by geometry in this one.
      const auto beyond = tuning_.straightStart;
      for (const auto& wire : wires) {
        if (!wire.feasible || wire.feedline || wire.fixed.empty()) {
          continue;
        }
        Path places = wire.resonator ? Path{} : wire.fixed;
        // The cells before the target, where the search arrives on the
        // port's heading: back from the target against that heading.
        const auto v = routing::headingVector(wire.objective.target.heading);
        for (std::int64_t k = 0; k <= beyond; ++k) {
          // `k` is signed on purpose: a heading step is a signed byte, and an
          // unsigned `k` would compute `k * v.dx` unsigned, turning a step of
          // -1 into four billion and cutting the approach short after one cell
          // on every heading that runs left or down.
          const auto x = static_cast<std::int64_t>(wire.objective.target.x) - (k * v.dx);
          const auto y = static_cast<std::int64_t>(wire.objective.target.y) - (k * v.dy);
          if (x < 0 || y < 0 || x >= width || y >= scene_.router.height) {
            break;
          }
          places.push_back({.x = static_cast<std::uint32_t>(x),
                            .y = static_cast<std::uint32_t>(y),
                            .heading = wire.objective.target.heading,
                            .primitive = 0});
        }
        const auto in = router_.straightStub(wire.objective.target, true, tuning_.straightStart);
        places.insert(places.end(), in.begin(), in.end());
        alongDisc(places, stencil, [&](const std::int64_t x, const std::int64_t y) {
          approaches_.set(static_cast<std::size_t>((y * width) + x), true);
        });
      }
    }

    // The resonators as they will stand: a coupler cuts every resonator back
    // to the target length, so what survives of another resonator's way is
    // its tail — the run from the qubit back along the way until the target
    // length and the tolerance are spent. No coupler body and no coupler
    // head may cut through that, whoever it belongs to. The head is priced
    // by what lies under it and the body was too; both are refused here,
    // because neither ever moves again once the option is taken.
    tailOwner_.assign(scene_.router.cells(), NO_OWNER);
    for (const auto& wire : wires) {
      if (!wire.resonator || !wire.feasible || !wire.drawn || wire.way.empty()) {
        continue;
      }
      const auto keep = std::max(0.0, tuning_.targetLength - wire.anchorGap) +
                        tuning_.lengthTolerance;
      double run = 0.0;
      for (auto at = wire.way.size(); at-- > 0;) {
        const auto& point = wire.way[at];
        if (point.x < scene_.router.width && point.y < scene_.router.height) {
          tailOwner_[scene_.router.index(point.x, point.y)] = wire.key;
        }
        if (at == 0) {
          break;
        }
        const auto& step = wire.way[at - 1];
        run += (step.x != point.x && step.y != point.y) ? std::numbers::sqrt2
                                                        : 1.0;
        if (run > keep) {
          break;
        }
      }
    }

    // Which wires have little room to give way: everything of the ring that
    // is not a resonator and not a feedline. Read by the option price.
    conventional_.assign(wires.size(), 0);
    for (const auto& wire : wires) {
      if (wire.key < conventional_.size()) {
        conventional_[wire.key] =
            static_cast<std::uint8_t>(!wire.resonator && !wire.feedline ? 1 : 0);
      }
    }

    // The couplers, one per drawn resonator, with their options.
    couplerOfWire_.clear();
    auto& couplerOfWire = couplerOfWire_;
    std::uint32_t withoutOptions = 0;
    for (auto& wire : wires) {
      if (!wire.resonator || !wire.feasible || !wire.drawn) {
        continue;
      }
      Coupler coupler;
      coupler.wire = wire.key;
      coupler.outerWay = wire.way;
      coupler.options = optionsOf(wire);
      if (coupler.options.empty()) {
        ++withoutOptions;
        tell(std::format("coupler for resonator {}: no option fits",
                         wireId(wire)));
        continue;
      }
      std::string where;
      for (Heading heading = 0; heading < routing::NUM_HEADINGS; ++heading) {
        if (std::ranges::any_of(coupler.options,
                                [heading](const CouplerOption& option) {
                                  return option.orientation == heading;
                                })) {
          where += (where.empty() ? "" : " ") + std::to_string(heading);
        }
      }
      tell(std::format("coupler for resonator {}: {} options, orientations {}",
                       wireId(wire), coupler.options.size(), where));
      couplerOfWire.emplace(wire.key,
                            static_cast<std::uint32_t>(couplers_.size()));
      couplers_.push_back(std::move(coupler));
    }

    // The chains, from the assignment: launcher, couplers in ring order,
    // launcher; an end that is a termination has no launcher.
    for (const auto& chain : assignment.chains) {
      std::vector<Waypoint> points;
      const auto launcher =
          [&](const fbd::PortRef& port) -> std::optional<Waypoint> {
        const auto index = port.index();
        const auto cell = scene_.launcherCell.find(index);
        if (index >= chip.ports.size() || cell == scene_.launcherCell.end()) {
          return std::nullopt;
        }
        const auto heading =
            routing::headingOfOrientation(chip.ports[index]->orientation);
        const auto place = scene_.router.cell(cell->second);
        Waypoint point;
        point.fixed = true;
        point.port = index;
        point.asSource = {.x = place.x(),
                          .y = place.y(),
                          .heading = routing::reverse(heading),
                          .primitive = 0};
        point.asTarget = {
            .x = place.x(), .y = place.y(), .heading = heading, .primitive = 0};
        return point;
      };
      if (chain->start != nullptr) {
        if (const auto point = launcher(*chain->start)) {
          points.push_back(*point);
        }
      }
      for (const auto node : chain->nodes) {
        const auto found = std::ranges::find_if(wires, [&](const Wire& wire) {
          return !wire.inner && wire.slot == node;
        });
        if (found == wires.end()) {
          continue;
        }
        const auto coupler = couplerOfWire.find(found->key);
        if (coupler == couplerOfWire.end()) {
          continue;
        }
        Waypoint point;
        point.coupler = coupler->second;
        points.push_back(point);
      }
      if (chain->end != nullptr) {
        if (const auto point = launcher(*chain->end)) {
          points.push_back(*point);
        }
      }
      if (points.size() >= 2) {
        chains_.push_back(std::move(points));
      }
    }

    chainEdgePaths_.assign(chains_.size(), {});
    edgeWireOf_.assign(chains_.size(), {});
    for (std::size_t chain = 0; chain < chains_.size(); ++chain) {
      const auto edges = chains_[chain].size() - 1;
      chainEdgePaths_[chain].assign(edges, Path{});
      edgeWireOf_[chain].assign(edges, NO_OWNER);
    }
    liftedR1_.assign(chains_.size(), false);
    liftedR2_.assign(chains_.size(), false);
    liftedR3_.assign(chains_.size(), false);
    chainOfCoupler_.assign(couplers_.size(), {NO_OWNER, 0});
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      for (std::size_t at = 0; at < chains_[chain].size(); ++at) {
        const auto& point = chains_[chain][at];
        if (point.fixed) {
          continue;
        }
        if (chainOfCoupler_[point.coupler].first != NO_OWNER) {
          throw std::logic_error(std::format(
              "coupler {} stands in chains {} and {}", point.coupler,
              chainOfCoupler_[point.coupler].first, chain));
        }
        chainOfCoupler_[point.coupler] = {chain, at};
      }
    }

    // R0: the ring as the feedline pass will sweep it — every wire that is
    // not inner, in the order the assignment feeds them, which is the order
    // `wires` holds them in before the edges are appended — and for every
    // edge between two couplers the plain wires that must cross it.
    ring_.clear();
    ringPlace_.clear();
    for (const auto& wire : wires) {
      if (!wire.inner && !wire.feedline) {
        ringPlace_.emplace(wire.slot, static_cast<std::uint32_t>(ring_.size()));
        ring_.push_back(wire.key);
      }
    }
    bridgers_.assign(chains_.size(), {});
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto edges = chains_[chain].size() - 1;
      bridgers_[chain].assign(edges, {});
      for (std::size_t at = 0; at < edges; ++at) {
        bridgers_[chain][at] = bridgersOf(wires, chain, at);
        if (!bridgers_[chain][at].empty()) {
          tell(std::format("[Coupler Insertion] chain {} edge {}->{}: {} plain "
                           "wire{} must cross it ({})",
                           chain, at, at + 1, bridgers_[chain][at].size(),
                           bridgers_[chain][at].size() == 1 ? "" : "s",
                           namesOf(wires, bridgers_[chain][at])));
        }
      }
    }

    drawCouplerOptions(wires);

    // **Which switches are actually in force.** Trap 5 of the handover is
    // that a switch which does not take costs an hour before anyone reads
    // the source; the cure is for the run to say what it is doing. Each one
    // names its value and where the value came from, so a stale export or
    // an empty assignment is visible in the log instead of being inferred
    // from the results.
    {
      const auto from = [](const char* name) {
        return envSet(name) != nullptr ? "environment" : "default";
      };
      say(std::format(
          "[Coupler Insertion] settings: search {} ({}), each chain on its "
          "own chip {} ({}), commit keeps the search's ways {} ({}), "
          "{:.0f}s a chain ({}), shortfall {:.0f}% ({}); room: pitch {} "
          "({}), margin {} ({}), channel reach {} ({}), channel count {} "
          "({}), report {} ({})",
          chainAStar() ? "prefix A*"
                       : (exactChainSearch() != 0 ? "trellis" : "greedy"),
          from("SCPD_CHAIN_ASTAR"), chainSolo() ? "yes" : "no",
          from("SCPD_CHAIN_SOLO"), chainKeepWays() ? "yes" : "no",
          from("SCPD_CHAIN_KEEP_WAYS"),
          static_cast<double>(chainAStarBudget().count()) / 1e9,
          from("SCPD_CHAIN_ASTAR_SECONDS"), couplerMaxShortfall() * 100.0,
          from("SCPD_COUPLER_MAX_SHORTFALL"), roomPitch(),
          from("SCPD_ROOM_PITCH"), roomMargin(), from("SCPD_ROOM_MARGIN"),
          roomChannelReach(), from("SCPD_ROOM_CHANNEL_REACH"),
          roomChannelCount() == 2 ? "ring" : "ways",
          from("SCPD_ROOM_CHANNEL_COUNT"), roomReport() ? "yes" : "no",
          from("SCPD_ROOM_REPORT")));
    }

    // The search for the options.
    std::uint32_t passes = 0;
    if (chainAStar()) {
      passes = optimizeChainsPrefix(wires);
    } else if (exactChainSearch()) {
      passes = optimizeChainsExact(wires);
    } else {
      for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
        passes = std::max(passes, optimizeChain(wires, chain));
      }
    }

    // The commit: the chosen options, then the edges.
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      applyOption(wires, index, couplers_[index].chosen);
    }
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& points = chains_[chain];
      for (std::size_t at = 0; at + 1 < points.size(); ++at) {
        Edge edge{.chain = chain,
                  .from = at,
                  .to = at + 1,
                  .wire = static_cast<std::uint32_t>(wires.size()),
                  .terminal = points[at].fixed || points[at + 1].fixed};
        Wire wire;
        wire.key = edge.wire;
        wire.slot = static_cast<std::uint32_t>(edges_.size());
        wire.feedline = true;
        wire.feasible = true;
        wire.terminal = edge.terminal;
        wire.edge = wire.slot;
        // Both ends the same way round: the edge that leaves a coupler runs
        // along its body exactly as the edge that arrives on it does, and
        // `couplerRunOf` is that run — the straight start where there is no
        // coupler, so a launcher keeps the run the rule asks for. The start
        // used to take the axial `straightStart` whatever stood there, which
        // made the two edges of one coupler disagree about its own body.
        wire.startStub = couplerStubs()
                             ? couplerRunOf(points[at].coupler)
                             : tuning_.straightStart;
        wire.endStub = points[at + 1].fixed
                           ? tuning_.straightStart
                           : couplerRunOf(points[at + 1].coupler);
        wire.couplerAtSource = points[at].coupler;
        wire.couplerAtTarget = points[at + 1].coupler;
        wire.sourcePort = points[at].port;
        wire.targetPort = points[at + 1].port;
        wire.objective.source = sourceOf(points[at]);
        wire.objective.target = targetOf(points[at + 1]);
        if (points[at].coupler != NO_OWNER) {
          couplers_[points[at].coupler].edgeOut = edge.wire;
        }
        if (points[at + 1].coupler != NO_OWNER) {
          couplers_[points[at + 1].coupler].edgeIn = edge.wire;
        }
        edgeWireOf_[chain][at] = edge.wire;
        wires.push_back(std::move(wire));
        edges_.push_back(edge);
      }
    }
    // The edges as the greedy chose them, where they still run between the
    // ports the couplers ended on; the rest routed now, against everything
    // committed before them.
    std::uint32_t drawn = 0;
    std::uint32_t angle = 0;
    for (const auto& edge : edges_) {
      auto& wire = wires[edge.wire];
      lastPicture_.clear();
      const auto& chosen = chainEdgePaths_[edge.chain][edge.from];
      // A path the greedy chose is kept only when it still runs between the
      // right two ports **and** still lies in the corridor that holds now.
      // The endpoints alone are not enough: the greedy drew it against the
      // chains as they stood then, with the edges of the coupler it was
      // weighing left out, and every edge committed since has closed ground
      // under it. Kept without this test, such a path ships straight through
      // a launcher's stub or another chain — visible in the debug picture as
      // a way crossing closed white, which no search would ever have
      // returned.
      // The endpoints are identity and are always asked. Whether the way
      // still lies in the corridor that holds now is the *rule*, and
      // `chainKeepWays` is where that rule is given up: the way the search
      // found is taken as it is, crossings and all.
      const bool runsRight = !chosen.empty() &&
                             chosen.front().samePlace(wire.objective.source) &&
                             chosen.back().samePlace(wire.objective.target);
      const bool fits =
          runsRight && (chainKeepWays() ||
                        edgeWayStillOpen(wires, wire.objective, edge, chosen));
      wire.way = fits ? chosen : routeEdge(wires, wire.objective, edge);
      wire.drawn = !wire.way.empty();
      // A way the greedy chose and the commit kept was drawn by no search of
      // this loop, so it has no picture of its own. Draw it against the
      // ground it stands on, or every line of the log would link the last
      // search the greedy happened to make.
      if (debug_ && fits) {
        corridorOfEdge(wires, wire.objective, edge);
        Wire pretend;
        pretend.objective = wire.objective;
        pretend.feedline = true;
        pretend.slot = wire.slot;
        frame_.wires = &wires;
        frame_.pass = "edge";
        frame_.round = edge.chain;
        frame_.forward = true;
        frame_.kind = std::format("c{}-{}to{}-kept", edge.chain, edge.from,
                                  edge.to);
        frame_.fence.clear();
        frame_.ripped.clear();
        frame_.before = NO_OWNER;
        frame_.after = NO_OWNER;
        drawSearch(pretend, wire.way,
                   std::format("chain {} {}->{}, as the greedy chose it",
                               edge.chain, edge.from, edge.to));
      }
      if (wire.drawn) {
        ++drawn;
        angle += angleCostOf(wire.way, wire.objective.target.heading);
        place(wire);
      }
      tell(std::format(
          "[Coupler Insertion] edge f{} chain {} ({}->{}): {}{}", wire.slot,
          edge.chain, edge.from, edge.to,
          wire.drawn
              ? std::format("{} cells, angle {}{}", wire.way.size(),
                            angleCostOf(wire.way, wire.objective.target.heading),
                            fits ? ", as the greedy chose it" : ", routed again")
              : std::string("NO WAY"),
          picture()));
    }
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      sayCoupler(wires, index);
    }
    static_cast<void>(checkFeedlineRoom(wires));
    static_cast<void>(checkCouplerBodies(wires));
    static_cast<void>(checkCouplerCrossings(wires));
    static_cast<void>(checkResonatorCrossings(wires));
    static_cast<void>(checkFeedlineCrossings(wires));
    reportRoom(wires);
    reportSqueeze(wires);
    reportBottlenecks(wires);
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    const auto diagonal = std::ranges::count_if(
        couplers_, [](const Coupler& coupler) {
          return routing::isDiagonal(
              coupler.options[coupler.chosen].way.front().heading);
        });
    say(std::format(
        "coupler insertion: {} couplers on {} resonators ({} on a diagonal), "
        "{} chains, {} of {} edges drawn, feedline angle cost {}, {} {}, "
        "{} of {} option costs from the memo, {:.1f}s",
        couplers_.size(), couplers_.size() + withoutOptions, diagonal,
        chains_.size(), drawn, edges_.size(), angle, passes,
        // Both searches fill `passes`, so the word has to say which one ran:
        // a log that calls an exact round a greedy pass is a log that answers
        // the wrong question when somebody asks which search they got.
        chainAStar()         ? "chains settled"
        : exactChainSearch() ? "exact rounds"
                             : "greedy passes",
        memoHits_, memoHits_ + memoMisses_, seconds));
    // The two figures the insertion is judged by, on a line of their own so
    // that a sweep over settings can be read off the log without counting.
    // Whether the ways the insertion leaves behind are ways a search could
    // have returned. A resonator spliced onto its coupler's lead is the one
    // way in the stage that no router drew, so it is the one that can hold a
    // corner no primitive makes; `couplerLeadMargin` is what buys the join
    // room to be a turn instead.
    std::uint32_t kinked = 0;
    std::uint32_t kinks = 0;
    for (const auto& coupler : couplers_) {
      const auto found = kinksIn(wires[coupler.wire].way);
      kinked += found == 0 ? 0 : 1;
      kinks += found;
    }
    say(std::format("coupler insertion: lead {} cells straight, {} of margin, "
                    "cut at {:.2f} of the target length; {} of {} resonators "
                    "keep a way with a corner no primitive makes, {} corners "
                    "in all",
                    leadStraight(), couplerLeadMargin(), couplerBias(), kinked,
                    couplers_.size(), kinks));
    const auto missing = static_cast<std::uint32_t>(edges_.size()) - drawn;
    if (boundPairs_ > 0) {
      say(std::format(
          "coupler insertion: the layer graph took {} bound{} in {:.1f} ms "
          "({:.3f} us each), bound {}",
          boundPairs_, boundPairs_ == 1 ? "" : "s",
          static_cast<double>(boundNanos_) / 1e6,
          static_cast<double>(boundNanos_) / 1000.0 /
              static_cast<double>(boundPairs_),
          chainBound() == 0   ? "turnBound"
          : chainBound() == 3 ? "analytic around the artwork"
                              : "analytic"));
    }
    if (chainBound() == 3) {
      say(std::format("coupler insertion: the family bound tried {} ways "
                      "against the artwork, raised the bound by {} eighth "
                      "turns in all, ran out of its path budget on {} pairs, "
                      "{} bounds above a real price (SCPD_CHAIN_BOUND=3)",
                      familyStats_.paths, familyStats_.raised,
                      familyStats_.budgetOut, familyViolations_));
    }
    if (chainStepBudget()) {
      say(std::format("coupler insertion: the step budget cut off {} edge "
                      "searches (SCPD_CHAIN_STEP_BUDGET)",
                      budgetCutoffs_));
    }
    if (chainLearnedBound()) {
      say(std::format("coupler insertion: the learned bound stood above the "
                      "analytic one on {} bounds asked and above the real "
                      "price on {} steps priced (SCPD_CHAIN_LEARNED_BOUND)",
                      learnedRaised_, learnedAbove_));
    }
    if (auditSteps_ > 0) {
      say(std::format(
          "coupler insertion: audit over {} priced steps — mean turnBound "
          "{:.2f}, "
          "analytic {:.2f}, real {:.2f}; sharper on {}, exact on {}, above the "
          "real on {}",
          auditSteps_, static_cast<double>(auditTurnBound_) / auditSteps_,
          static_cast<double>(auditAnalytic_) / auditSteps_,
          static_cast<double>(auditReal_) / auditSteps_, auditSharper_,
          auditExact_, auditViolations_));
    }
    {
      const auto chainsOf = [](const std::vector<std::uint32_t>& chains) {
        std::string out;
        for (const auto chain : chains) {
          out += (out.empty() ? "" : " ") + std::to_string(chain);
        }
        return out;
      };
      say(std::format(
          "coupler insertion: room rules — R1 refused {}/{} edges, R2 "
          "refused {}/{} channels, R3 refused {} places ({} couplers built "
          "without it, {} moved > 100 units); stood down on chains R2[{}] "
          "R1[{}] R3[{}]; committed: {} edges below capacity, {} of {} "
          "coupler channels tighter than (m+1)·pitch ({} by along-runners), "
          "{} of {} channels between chains, couplers with a plain wire "
          "within 0/5/10/19 cells: {}/{}/{}/{}",
          room_.r1Refused, room_.r1Asked, room_.r2Refused, room_.r2Asked,
          room_.r3Refused, room_.r3StoodDown, room_.r3Moved,
          chainsOf(room_.liftedR2), chainsOf(room_.liftedR1),
          chainsOf(room_.liftedR3), room_.committedBelowCapacity,
          room_.committedTightChannels, room_.couplerChannels,
          room_.committedTightAlong, room_.crossChainTight,
          room_.crossChainChannels, room_.r3Near[0], room_.r3Near[1],
          room_.r3Near[2], room_.r3Near[3]));
    }
    say(std::format("==> coupler insertion: {} feedline edge{} NOT drawn, "
                    "feedline angle cost {}, {:.1f}s",
                    missing, missing == 1 ? "" : "s", angle, seconds));
  }

  /// Every option of a coupler on a resonator's way: the prototype's sweep
  /// over the orientation offsets and the mirror, with the direction of the
  /// feedline and one second dogleg on top.
  [[nodiscard]] std::vector<CouplerOption> optionsOf(Wire& resonator) {
    std::vector<CouplerOption> options;
    // The resonator's own room must not stand in the way of its own body.
    lift(resonator);
    // The place is settled **per orientation**, not once for all of them.
    // Every orientation gets the point of the way whose mismatch to the
    // biased target is least *among the points where that orientation
    // actually fits* — where its pad, its feedline run and its resonator
    // lead all stay inside the box. A point that is perfect on length but
    // puts the pad through the box edge is not the place for that
    // orientation; the next point along the way is tried, and the one after
    // that.
    //
    // Settling it once for all eight was what cost the options: one
    // orientation's perfect point is another's impossible one, and the
    // whole coupler was built at the first point that suited *anybody*.
    const auto places = couplerPlace(resonator);
    for (Heading offset = 0; offset < routing::NUM_HEADINGS; ++offset) {
      for (const bool secondPort : {false, true}) {
       // The second dogleg, as the prototype sweeps it: none, or one of
       // twenty turned each way. It slides the pad sideways off the way it
       // sits on without changing which way the coupler points, which is
       // the one freedom the offsets and the two ports do not give.
       for (const auto& [secondStraight, secondReverse] :
            {std::pair{0U, false}, std::pair{COUPLER_SECOND_STRAIGHT, true},
             std::pair{COUPLER_SECOND_STRAIGHT, false}}) {
        std::string why;
        std::size_t tried = 0;
        bool got = false;
        // Every place, in order of mismatch, until one fits. No cap: the
        // cap was what broke 4q, where the two dozen places nearest the
        // target length all put the pad through the box edge and the
        // coupler was given up on while places that fitted lay further
        // along the way.
        for (const auto& spot : places) {
          ++tried;
          auto option =
              makeOption(resonator, spot, secondPort, secondStraight,
                         secondReverse, offset, explaining() ? &why : nullptr);
          if (!option.has_value()) {
            continue;
          }
          explain(std::format(
              "[Coupler Insertion]   '{}' offset {} port {}{}: option {} at "
              "place {} of {} ({:.0f} of {:.0f} cells to the port), "
              "angle {}°, centre ({},{}), {} priced cells",
              wireId(resonator), offset, secondPort ? 2 : 1,
              secondStraight == 0
                  ? std::string{}
                  : std::format(" jog {}{}", secondStraight,
                                secondReverse ? " reversed" : ""),
              options.size(), tried - 1, places.size(), spot.centreLength,
              spot.wanted, degreesOfHeading(option->couplerOrientation),
              option->centre.x, option->centre.y, option->guarded));
          options.push_back(std::move(*option));
          got = true;
          break;
        }
        if (!got && explaining()) {
          explain(std::format(
              "[Coupler Insertion]   '{}' offset {} port {}{}: no place of "
              "{} tried fits — last refusal: {}",
              wireId(resonator), offset, secondPort ? 2 : 1,
              secondStraight == 0
                  ? std::string{}
                  : std::format(" jog {}{}", secondStraight,
                                secondReverse ? " reversed" : ""),
              tried, why.empty() ? std::string("none offered") : why));
        }
       }
      }
    }
    if (places.empty() && explaining()) {
      explain(std::format(
          "resonator {}: no cell of its way carries the biased target",
          wireId(resonator)));
    }
    place(resonator);
    return options;
  }

  /// The rectangle a coupler may sit in.
  ///
  /// Every launcher closes a stub for the wire that leaves it: the cell, the
  /// straight run it needs and the clearance around both. A coupler placed
  /// beyond one of those stubs puts the chain's feedline on the far side of
  /// it, and the edge into that coupler has to come back around the stub —
  /// which is where the detours came from. The stubs of all the launchers
  /// together leave one open rectangle in the middle of the chip, and inside
  /// it no stub stands between a coupler and the ring.
  ///
  /// The rectangle is the innermost of the stub ends: of each launcher the
  /// point its stub reaches furthest in the direction it faces, projected
  /// onto the axis it bounds.
  struct CouplerBox {
    std::int64_t minX = 0;
    std::int64_t maxX = 0;
    std::int64_t minY = 0;
    std::int64_t maxY = 0;
    [[nodiscard]] bool holds(const std::int64_t x, const std::int64_t y,
                             const std::int64_t margin = 0) const {
      return x >= minX + margin && x <= maxX - margin &&
             y >= minY + margin && y <= maxY - margin;
    }
    [[nodiscard]] bool empty() const { return maxX < minX || maxY < minY; }
  };

  /// The room a chain's edge has no choice about beyond a coupler's feedline
  /// port: the straight run it is forced to make off it and the quarter turn
  /// at the end of that run, in cells, exactly as `runsOfEdge` and
  /// `corridorOfEdge` reckon it.
  ///
  /// `max(terminalSlot(), run + BEND_RADIUS)` with `run` the pad's own run on
  /// this orientation — 26 cells axial and 20 diagonal on the benchmarks,
  /// against the 16 a launcher's `straightStart + BEND_RADIUS` comes to. With
  /// `couplerBoxTurn` off it is the figure the box had, `straightStart`.
  [[nodiscard]] std::int64_t edgeRunOffPort(const Heading orientation) const {
    if (!couplerBoxTurn()) {
      return static_cast<std::int64_t>(tuning_.straightStart);
    }
    const auto run = couplerStubs()
                         ? cellsOn(tuning_.couplerLength, orientation)
                         : tuning_.straightStart;
    return static_cast<std::int64_t>(
        std::max(terminalSlot(), run + static_cast<std::uint32_t>(BEND_RADIUS)));
  }

  /// Whether a coupler centred on a cell, in one orientation, keeps both of
  /// its feedline ports and the straight run the chain is forced to make off
  /// each of them inside the box.
  ///
  /// The real coordinates, not a margin around the centre: how far the ports
  /// lie from the centre and in which direction is the orientation's own
  /// business, and a pad lying along the box's long side reaches nowhere
  /// near as far across it as one lying athwart. A single figure for all
  /// eight is pessimistic by the difference, and it cost places that were
  /// perfectly good.
  ///
  /// The run along the far edge is a straight line and the box is convex, so
  /// its two ends decide it.
  [[nodiscard]] bool terminalsInBox(const std::int64_t cx,
                                    const std::int64_t cy,
                                    const Heading orientation,
                                    const Heading leaves) const {
    const std::int64_t halfRun =
        cellsOn(tuning_.couplerLength, orientation) / 2;
    const std::int64_t halfDepth =
        cellsOn(tuning_.couplerHeight, orientation) / 2;
    const auto along = routing::headingVector(orientation);
    // The edge the feedline runs along, picked as `makeOption` picks it: the
    // one away from the side the resonator leaves on.
    const auto leaving = routing::headingVector(leaves);
    auto across = routing::headingVector(routing::turned(orientation, 2));
    if (((across.dx * leaving.dx) + (across.dy * leaving.dy)) > 0) {
      across = routing::headingVector(routing::turned(orientation, -2));
    }
    const auto edgeX = cx + ((halfDepth + 1) * across.dx);
    const auto edgeY = cy + ((halfDepth + 1) * across.dy);
    // The straight run off the port, and the quarter turn the edge has to
    // make at the end of it: a run that ends on the box edge leaves the
    // curve after it nowhere but in a launcher's stub.
    //
    // **The run off a coupler port is not `straightStart`.** The two ends of
    // a chain's edge are forced straight by different figures, and
    // `runsOfEdge` is where they are said: `straightStart` at a launcher, the
    // pad's own run — `cellsOn(couplerLength, orientation)`, 21 cells axial
    // and 15 diagonal — at a coupler. `corridorOfEdge` then leaves
    // `max(terminalSlot(), run + BEND_RADIUS)` open at each terminal, 26 at a
    // coupler against 20 at a launcher. A box drawn on `straightStart` is ten
    // cells short of the room the edge off this port has no choice about, so
    // the same figure is used here. See `edgeRunOffPort`.
    const std::int64_t reach = halfRun + edgeRunOffPort(orientation);
    for (const std::int64_t end : {-reach, reach}) {
      if (!couplerBox_.holds(edgeX + (end * along.dx), edgeY + (end * along.dy),
                             couplerBoxMargin())) {
        return false;
      }
    }
    return true;
  }

  /// Whether any of the eight orientations fits at a cell. A place needs one
  /// — the greedy is free to take another and reach past the box, as it was
  /// before the box existed.
  [[nodiscard]] bool anyOrientationFits(const std::int64_t cx,
                                        const std::int64_t cy,
                                        const Heading leaves) const {
    for (Heading offset = 0; offset < routing::NUM_HEADINGS; ++offset) {
      const auto orientation =
          static_cast<Heading>((leaves + offset) % routing::NUM_HEADINGS);
      if (terminalsInBox(cx, cy, orientation, leaves)) {
        return true;
      }
    }
    return false;
  }

  /// The rectangle the launcher stubs leave open, from the scene's launchers.
  [[nodiscard]] CouplerBox couplerBoxOf() const {
    CouplerBox box{.minX = 0,
                   .maxX = static_cast<std::int64_t>(scene_.router.width) - 1,
                   .minY = 0,
                   .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    // How far a stub reaches from its launcher: the straight run the wire
    // needs, the clearance the stub is inflated by, and one cell more.
    //
    // The one cell matters. The stub's places run to `straightStart` and the
    // clearance disc carries them `clearance` further, so `straightStart +
    // clearance` is the **last closed** cell, not the first open one. A box
    // drawn on it is tangent to every stub, and a coupler port on its edge
    // has its straight run grazing the disc — which is why a path left such
    // a port straight and then swung wide instead of running on.
    //
    // The run here is `launcherStraight`, not `straightStart`: the box keeps
    // the launchers a little more room than their own stubs take, so that an
    // edge into a coupler at the box edge has somewhere to run. See
    // `launcherLead`.
    //
    // And the launcher's run alone is not what the wire off it needs: it is
    // forced straight for `straightStart` and then has to *turn* to leave
    // the channel, which takes `BEND_RADIUS` more. The longer of the two
    // runs is what the box keeps clear — see `couplerBoxTurn`.
    const auto run = couplerBoxTurn()
                         ? std::max(tuning_.launcherStraight,
                                    tuning_.straightStart + BEND_RADIUS)
                         : tuning_.launcherStraight;
    const auto reach =
        static_cast<std::int64_t>(run + tuning_.clearance) + 1;
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      const auto v = routing::headingVector(found->second);
      const auto endX = static_cast<std::int64_t>(place.x()) + (reach * v.dx);
      const auto endY = static_cast<std::int64_t>(place.y()) + (reach * v.dy);
      // A stub that faces along an axis bounds that axis, on the side it
      // faces from. A diagonal one bounds both.
      if (v.dx > 0) {
        box.minX = std::max(box.minX, endX);
      } else if (v.dx < 0) {
        box.maxX = std::min(box.maxX, endX);
      }
      if (v.dy > 0) {
        box.minY = std::max(box.minY, endY);
      } else if (v.dy < 0) {
        box.maxY = std::min(box.maxY, endY);
      }
    }
    return box;
  }

  /// Where a coupler sits on a resonator: one cell of the way, and what the
  /// way still measures from there to the port.
  struct CouplerPlace {
    /// The index into the way the coupler's centre takes.
    std::size_t at = 0;
    /// That cell, for the box the coupler has to sit in, and the heading
    /// the way holds there, which decides which edge the feedline takes.
    std::int64_t x = 0;
    std::int64_t y = 0;
    Heading leaves = 0;
    /// What the way from there to the port came to, in cells, against what
    /// was asked of it.
    double centreLength = 0.0;
    double wanted = 0.0;
  };

  /// The place the coupler takes on a resonator: the point of the way where
  /// what is left to the qubit is `target_resonator_length` times the bias.
  ///
  /// This replaces the prototype's splice. There is no dogleg and no turn to
  /// build: the body is a pad centred on that one point, the way is cut
  /// there, and the orientation is the option's to choose. What is left of
  /// the way is short of the design figure by the bias, which is the room the
  /// meander needs to make it exact afterwards — the meander can only
  /// lengthen a way, never shorten one.
  [[nodiscard]] std::vector<CouplerPlace>
  couplerPlace(const Wire& resonator) const {
    if (resonator.way.size() < 2) {
      return {};
    }
    const auto segmented =
        routing::reconstructSegments(*primitives_, resonator.way);
    if (segmented.segments.empty() ||
        segmented.segments.back().lengthAt.empty()) {
      return {};
    }
    const auto overall = segmented.segments.back().lengthAt.back();
    const double target =
        std::max(0.0, tuning_.targetLength - resonator.anchorGap);
    // The component's own lead runs in front of what is left, so the place
    // is the point where the lead **and** the rest together come to the
    // figure. Without this the way left is the target and the lead — a
    // quarter turn and fourteen cells, about 290 layout units — sits on top
    // of it: on 21q every resonator came out at 2746 to 2805 against a
    // target of 2500, over by exactly the lead, and nothing downstream
    // shortens a way.
    const double wanted =
        std::max(0.0, (target * couplerBias()) - leadLength());
    // Where each cell of the way sits, so a cell of a segment can be named as
    // an index into the way again.
    std::unordered_map<std::uint64_t, std::size_t> indexOf;
    indexOf.reserve(resonator.way.size() * 2);
    for (std::size_t r = 0; r < resonator.way.size(); ++r) {
      const auto key = (static_cast<std::uint64_t>(resonator.way[r].x) << 32U) |
                       resonator.way[r].y;
      indexOf.emplace(key, r);
    }
    std::vector<CouplerPlace> places;
    for (const auto& segment : segmented.segments) {
      for (std::size_t i = 0; i < segment.cells.size(); ++i) {
        const auto left = overall - segment.lengthAt[i];
        const auto key =
            (static_cast<std::uint64_t>(segment.cells[i].x) << 32U) |
            segment.cells[i].y;
        const auto found = indexOf.find(key);
        if (found == indexOf.end() || found->second + 1 >= resonator.way.size()) {
          continue;
        }
        places.push_back(
            CouplerPlace{.at = found->second,
                         .x = static_cast<std::int64_t>(segment.cells[i].x),
                         .y = static_cast<std::int64_t>(segment.cells[i].y),
                         .leaves = segment.cells[i].heading,
                         .centreLength = left,
                         .wanted = wanted});
      }
    }
    // Nearest the asked-for length first, and nothing else. Which places
    // fit is no longer a property of the list: every orientation walks it
    // and stops at the first place *it* fits, so the order only has to say
    // which place is best on length. The list used to be sorted by whether
    // any of the eight orientations fitted, which put one orientation's
    // impossible point ahead of another's perfect one.
    std::ranges::sort(places, [](const CouplerPlace& a, const CouplerPlace& b) {
      return std::abs(a.centreLength - a.wanted) <
             std::abs(b.centreLength - b.wanted);
    });
    return places;
  }

  /// Where a resonator reaches its target length, on the way it has now: the
  /// index of that cell (`pt`) and the band of indices `lo..hi` whose
  /// remaining length to the target end lies within `lengthPointPct` of the
  /// figure. The figure is the target length less the anchor gap, as the
  /// prototype's `meander_length - anchor` is — plainly ±10 % of the target
  /// length around that point, nothing derived (user, 2026-10-04). The pad
  /// the insertion then centres lies `leadLength()` nearer the target end,
  /// well inside the band. The length is measured as the insertion measures
  /// it: by `reconstructSegments` on a drawn way, over the cells' polyline on
  /// a seeded one, which has no primitives of this grid yet. Backwards from
  /// the target end, because the insertion scores every place by what is
  /// left to the qubit. A way shorter than the band's near end collapses
  /// the band onto the point, as the prototype collapses it onto the front.
  struct LengthMarks {
    bool valid = false;
    std::size_t lo = 0;
    std::size_t pt = 0;
    std::size_t hi = 0;
    double wanted = 0.0;
  };

  [[nodiscard]] LengthMarks lengthMarksOf(const Wire& wire) const {
    LengthMarks marks;
    if (!wire.resonator || wire.way.size() < 2) {
      return marks;
    }
    const auto& way = wire.way;
    std::vector<double> at(way.size(), 0.0);
    if (wire.drawn) {
      const auto segmented = routing::reconstructSegments(*primitives_, way);
      std::unordered_map<std::uint64_t, std::size_t> indexOf;
      indexOf.reserve(way.size() * 2);
      const auto keyOf = [](const PathPoint& point) {
        return (static_cast<std::uint64_t>(point.x) << 32U) | point.y;
      };
      for (std::size_t r = 0; r < way.size(); ++r) {
        indexOf.emplace(keyOf(way[r]), r);
      }
      std::vector<double> known(way.size(), -1.0);
      for (const auto& segment : segmented.segments) {
        for (std::size_t i = 0; i < segment.cells.size(); ++i) {
          const auto found = indexOf.find(keyOf(segment.cells[i]));
          if (found != indexOf.end()) {
            known[found->second] = segment.lengthAt[i];
          }
        }
      }
      // The cells an arc sweeps carry no length of their own; they take the
      // last length known before them.
      double last = 0.0;
      for (std::size_t r = 0; r < way.size(); ++r) {
        if (known[r] >= 0.0) {
          last = known[r];
        }
        at[r] = last;
      }
    } else {
      for (std::size_t r = 1; r < way.size(); ++r) {
        at[r] = at[r - 1] +
                std::hypot(static_cast<double>(way[r].x) - way[r - 1].x,
                           static_cast<double>(way[r].y) - way[r - 1].y);
      }
    }
    const double overall = at.back();
    marks.wanted = std::max(0.0, tuning_.targetLength - wire.anchorGap);
    const double pct = lengthPointPct();
    const double low = marks.wanted * (1.0 - pct);
    const double high = marks.wanted * (1.0 + pct);
    std::size_t lo = way.size();
    std::size_t hi = 0;
    double nearest = std::numeric_limits<double>::max();
    for (std::size_t r = 0; r < way.size(); ++r) {
      const double left = overall - at[r];
      if (left >= low && left <= high) {
        lo = std::min(lo, r);
        hi = std::max(hi, r);
      }
      const double off = std::abs(left - marks.wanted);
      if (off < nearest) {
        nearest = off;
        marks.pt = r;
      }
    }
    if (lo == way.size()) {
      lo = marks.pt;
      hi = marks.pt;
    }
    marks.lo = lo;
    marks.hi = hi;
    marks.valid = true;
    return marks;
  }

  /// The straight run of the component's resonator lead, in cells. The
  /// coupler is drawn with fourteen; a wire still owes the design rule its
  /// own straight, so the lead is never shorter than that. On a grid of
  /// about ten layout units a cell the rule asks eleven.
  /// How long the component's lead is, in cells: the quarter turn and the
  /// straight after it. Built once on a cardinal heading; a lead on a
  /// diagonal is a little longer, and the place is picked by this one figure
  /// all the same, because the place is settled before the orientation.
  [[nodiscard]] double leadLength() const {
    if (leadLength_ < 0.0) {
      try {
        leadLength_ =
            routing::buildDogleg(*primitives_, 0, 1, leadStraight()).cost;
      } catch (const std::logic_error&) {
        leadLength_ = leadStraight();
      }
    }
    return leadLength_;
  }
  mutable double leadLength_ = -1.0;

  /// A heading in degrees, the way the prototype prints an angle: it
  /// counts eighths and multiplies by 45, so a log of ours can be read
  /// beside a log of its.
  [[nodiscard]] static int degreesOfHeading(const Heading heading) {
    return static_cast<int>(heading) * 45;
  }

  [[nodiscard]] std::uint32_t leadStraight() const {
    return std::max(COUPLER_LEAD_STRAIGHT, tuning_.straightStart) +
           couplerLeadMargin();
  }
  static constexpr std::uint32_t COUPLER_LEAD_STRAIGHT = 14;

  /// How many cells of straight run the coupler's lead keeps **beyond** what
  /// the rule asks, so that the resonator spliced onto it can be drawn
  /// again.
  ///
  /// The insertion splices a way together out of three pieces: the quarter
  /// turn off the coupler, the straight run after it, and the route the
  /// outer stage had left from the insertion point on. It checks that the
  /// three do not meet themselves, and nothing else — in particular not that
  /// the join is a corner any Dubins primitive could have made. Where the
  /// straight ends on one heading and the old route carries on at another,
  /// the way has a kink, and a kink is a shape no search will ever return.
  /// The wire is then placed on a way it cannot be given again: every
  /// neighbour settles against that shape, and when the wire is asked to
  /// draw itself the only legal ways left are the ones the neighbours now
  /// stand in (user, 2026-10-01).
  ///
  /// The margin buys the join room to be a turn rather than a corner.
  /// `SCPD_COUPLER_LEAD_MARGIN` sets it; `=0` is the figure that stood
  /// before.
  [[nodiscard]] static std::uint32_t couplerLeadMargin() {
    static const auto margin = static_cast<std::uint32_t>(
        std::max(0, envWhole("SCPD_COUPLER_LEAD_MARGIN", 5)));
    return margin;
  }

  /// How many corners of a way no primitive could have made: two cells in a
  /// row whose headings are more than one eighth apart. A way the router
  /// returned holds none.
  [[nodiscard]] static std::uint32_t kinksIn(const Path& way) {
    std::uint32_t kinks = 0;
    for (std::size_t at = 0; at + 1 < way.size(); ++at) {
      if (routing::headingDistance(way[at].heading, way[at + 1].heading) > 1) {
        ++kinks;
      }
    }
    return kinks;
  }

  /// The straight of the second dogleg, in cells. The prototype's twenty.
  static constexpr std::uint32_t COUPLER_SECOND_STRAIGHT = 20;

  /// One option at the place the coupler takes: the body as a pad centred on
  /// that cell, turned by one of the eight offsets from the heading the way
  /// holds there. Nothing, where it does not fit.
  [[nodiscard]] std::optional<CouplerOption>
  makeOption(const Wire& resonator, const CouplerPlace& place,
             const bool secondPort, const std::uint32_t secondStraight,
             const bool secondReverse,
             const Heading offset, std::string* why = nullptr) const {
    const auto refuse = [&why](std::string reason)
        -> std::optional<CouplerOption> {
      if (why != nullptr) {
        *why = std::move(reason);
      }
      return std::nullopt;
    };
    CouplerOption option;
    option.offset = offset;
    // The way is cut at the place. Nothing is spliced in front of it: the
    // resonator simply runs out of the pad it ends on.
    option.way.assign(resonator.way.begin() +
                          static_cast<std::ptrdiff_t>(place.at),
                      resonator.way.end());
    if (option.way.size() < 2) {
      return refuse("what is left of the way is too short to draw");
    }
    // The place is where the **tip of the lead** has to land — the end of
    // the arc and the straight after it. Neither the pad's centre nor the
    // resonator's port is this cell any more; both are worked back from it
    // once the lead is built.
    // The orientation: the heading the way holds at the place, turned by the
    // offset. This is the one thing an option chooses, and it is what lets
    // the coupler face the chain rather than inherit the direction the
    // resonator happens to run in — without it the pad on a diagonal run
    // takes a diagonal approach the chain cannot make, and edges go
    // undrawn.
    // At offset 0 the coupler's own orientation — the direction the
    // straight after the mandatory arc runs — is the heading the nearest
    // launcher faces, so the resonator leaves the coupler pointing the way
    // that launcher points. The pad's axis is a quarter turn back from
    // that, because the arc turns a quarter from the port onto the
    // orientation.
    const auto facing = routing::turned(
        nearestLauncherHeading(place.x, place.y), -2);
    option.orientation =
        static_cast<Heading>((facing + offset) % routing::NUM_HEADINGS);
    option.run = cellsOn(tuning_.couplerLength, option.orientation);
    option.depth = cellsOn(tuning_.couplerHeight, option.orientation);
    // What is left runs to the qubit. Only an overshoot is refused: nothing
    // spliced in later makes a way shorter, while a way short of the figure
    // is what the meander is for, and is priced by how much it has to add.
    const double target =
        std::max(0.0, tuning_.targetLength - resonator.anchorGap);
    const auto left = lengthOf(option.way);
    if (left > target + tuning_.lengthTolerance) {
      return refuse("what is left of the way is longer than the target");
    }
    // **And it may not be far shorter either.**
    //
    // The figure to hold against is what the resonator will actually
    // measure: the lead the component leaves the pad on, what is left to the
    // qubit, and the run from the last cell to the port. `left` is the
    // middle term alone — the lead is spliced on at the end of this function
    // — so the lead has to be added back, and it matters that it is: it is
    // an absolute length, about 290 layout units, which is a twentieth of a
    // 6000-unit target and an eighth of a 2500-unit one. Measured without
    // it, the same share would refuse on the small chips the very place
    // `couplerPlace` aims at.
    const auto whole = left + leadLength() + resonator.anchorGap;
    const auto least = tuning_.targetLength * (1.0 - couplerMaxShortfall());
    if (whole < least) {
      return refuse(std::format(
          "what is left of the resonator is {:.0f} against a target of "
          "{:.0f}, past the {:.0f}% a coupler may take off",
          whole, tuning_.targetLength, couplerMaxShortfall() * 100.0));
    }
    if (left < target - tuning_.lengthTolerance) {
      option.guarded +=
          static_cast<std::uint32_t>(target - tuning_.lengthTolerance - left);
    }

    // The body: a pad centred on the anchor, the run along the orientation
    // and the depth across it. In cells: a cell is in the body when its
    // projections onto the two step vectors lie within half the run and half
    // the depth. The feedline runs along the far edge, parallel to the run.
    const auto along = routing::headingVector(option.orientation);
    // Which long edge of the pad the feedline runs along: the one away from
    // the side the resonator leaves on, so that the resonator never has to
    // cross its own feedline to get to its qubit. The pad straddles the
    // centre, so both edges exist either way; what this picks is the pair of
    // feedline ports, and picking the near one put every resonator on the
    // same side as the chain that drives it.
    // The side the feedline takes is not guessed any more, it is derived.
    // It used to be picked by a heuristic — the edge away from the side the
    // resonator happened to leave on — and the legality rule below then
    // demanded one particular side, so five of the eight offsets were built
    // and thrown away again for no geometric reason: on 4q the rule alone
    // refused forty of sixty-four candidates, on 17q two hundred and forty
    // of four hundred and sixteen. Taking the side the rule asks for makes
    // every offset legal and leaves the rule an invariant.
    const auto acrossHeading = routing::turned(option.orientation, -2);
    const auto across = routing::headingVector(acrossHeading);

    // The coupler's orientation. Think of the pad as a rectangle with the
    // line between its two feedline ports along one long edge and the line
    // between its two resonator ports along the other: the orientation is
    // the perpendicular from the feedline line to the resonator line. It is
    // the direction the coupler points its resonator in, and the straight
    // run after the arc lies on it.
    option.couplerOrientation = routing::reverse(acrossHeading);

    // Which options are legal. Both feedline edges meet this coupler at the
    // same angle — one arrives on it, the other leaves on it — and that
    // angle is the pad's own axis, which both feedline ports carry. The
    // coupler has to point its resonator a quarter turn with it. Which long
    // edge the feedline takes is decided above from the heading the way
    // holds at the place, so half the offsets come out pointing the other
    // way: those are the options that would send the resonator across its
    // own feedline, and they are refused here rather than priced.
    if (const auto legal = routing::turned(option.orientation, 2);
        option.couplerOrientation != legal) {
      return refuse(std::format(
          "the coupler points its resonator on {} where the feedline angle "
          "{} makes {} the only legal one",
          option.couplerOrientation, option.orientation, legal));
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const std::int64_t halfRun = option.run / 2;
    const std::int64_t halfDepth = option.depth / 2;

    // Where the coupler sits. What has to land on the resonator's path is
    // not the pad's centre but the **tip of the straight run after the
    // arc** — the insertion point is the end of the component's own lead,
    // not the middle of its pad. So the lead is built first, in a frame
    // whose origin is the resonator port, and the port is the place less
    // the whole lead; the pad then hangs off that port.
    //
    // The port the resonator leaves on, and which way its arc turns: the
    // two ports face opposite ways along the pad, so one reaches the
    // coupler's orientation with the clock and the other against it.
    option.secondPort = secondPort;
    const auto portHeading = secondPort
                                 ? routing::reverse(option.orientation)
                                 : option.orientation;
    option.secondStraight = secondStraight;
    option.secondReverse = secondReverse;
    routing::DoglegGeometry lead;
    routing::DoglegGeometry jog;
    Heading afterFirst = 0;
    try {
      const auto turnSign =
          routing::turned(portHeading, 2) == option.couplerOrientation ? 1 : -1;
      lead = routing::buildDogleg(*primitives_, portHeading, turnSign,
                                  leadStraight());
      afterFirst = lead.tip.heading;
      // The second dogleg, when the option asks for one: another quarter
      // turn off the orientation and another straight. It turns the lead a
      // second time, so the heading it finally meets the path on is **not**
      // the coupler's orientation any more — the orientation is what the
      // component points the resonator in, which is settled by the first
      // arc, and the second is a jog in front of it that moves where the
      // pad has to sit for that lead to land on the way. Reversed it jogs
      // one way, with the first turn the other.
      if (secondStraight > 0) {
        jog = routing::buildDogleg(*primitives_, afterFirst,
                                   secondReverse ? -turnSign : turnSign,
                                   secondStraight);
      }
    } catch (const std::logic_error&) {
      return refuse("the primitives hold no quarter turn for the lead");
    }
    // Where the whole lead ends, as an offset from the port: the first
    // dogleg, and the second laid on its tip.
    const auto tipX =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.x)) +
        (secondStraight > 0
             ? static_cast<std::int64_t>(static_cast<std::int32_t>(jog.tip.x))
             : 0);
    const auto tipY =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.y)) +
        (secondStraight > 0
             ? static_cast<std::int64_t>(static_cast<std::int32_t>(jog.tip.y))
             : 0);
    const auto portX = place.x - tipX;
    const auto portY = place.y - tipY;
    // The port is one end of the near edge; the centre lies half a depth
    // across from it and half a run back along the pad, on the side the
    // other port is.
    const std::int64_t towards = secondPort ? 1 : -1;
    const auto originX = portX + ((halfDepth + 1) * across.dx) +
                         (towards * halfRun * along.dx);
    const auto originY = portY + ((halfDepth + 1) * across.dy) +
                         (towards * halfRun * along.dy);
    if (originX < 0 || originY < 0 || originX >= scene_.router.width ||
        originY >= scene_.router.height) {
      return refuse("the pad leaves the grid");
    }
    option.centre = {.x = static_cast<std::uint32_t>(originX),
                     .y = static_cast<std::uint32_t>(originY),
                     .heading = option.orientation,
                     .primitive = 0};
    const std::int64_t alongNorm =
        (along.dx * along.dx) + (along.dy * along.dy);
    const std::int64_t acrossNorm =
        (across.dx * across.dx) + (across.dy * across.dy);
    const auto reach = halfRun + halfDepth + 1;
    if (originX - reach < 0 || originY - reach < 0 ||
        originX + reach >= width || originY + reach >= height) {
      return refuse("the body leaves the grid");
    }
    const char* blocker = "";
    const auto free = [&](const std::size_t cell) {
      if (scene_.blocked.test(cell)) {
        blocker = "artwork";
        return false;
      }
      if (approaches_.test(cell)) {
        blocker = "a port's approach";
        return false;
      }
      // A body already placed is not a refusal, and neither is another wire's
      // copper: every wire is drawn again under the feedline constraints.
      // Both are priced far above a wire's room.
      option.guarded += bodies_.test(cell) ? 20 : 0;
      const auto owner = field_.owner(cell);
      option.guarded += (owner != NO_OWNER && owner != resonator.key) ? 20 : 0;
      option.guarded += field_.guarded(cell) ? 1 : 0;
      return true;
    };
    for (std::int64_t y = originY - reach; y <= originY + reach; ++y) {
      for (std::int64_t x = originX - reach; x <= originX + reach; ++x) {
        const auto dx = x - originX;
        const auto dy = y - originY;
        const auto onRun = (dx * along.dx) + (dy * along.dy);
        const auto onDepth = (dx * across.dx) + (dy * across.dy);
        if (std::abs(onRun) > halfRun * alongNorm ||
            std::abs(onDepth) > halfDepth * acrossNorm) {
          continue;
        }
        // The pad's own copper. Nothing tested this against the box. The
        // run is on the pad's far edge, so an axial body whose run is inside
        // is inside too — but a diagonal one turns its corners out, and the
        // rule is about the copper and not about the four axial cases.
        if (!couplerBox_.holds(x, y, couplerBoxMargin())) {
          return refuse(
              std::format("the body leaves the box at ({},{})", x, y));
        }
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (!free(cell)) {
          return refuse(std::format("the body meets {}", blocker));
        }
        option.body.push_back(cell);
      }
    }
    if (option.body.empty()) {
      return refuse("the body has no cells");
    }
    const std::unordered_set<std::size_t> pad(option.body.begin(),
                                              option.body.end());

    // **What is left of the resonator may not run back through the pad.**
    //
    // The way left runs from the insertion point to the qubit, and the pad
    // hangs off the far end of the component's lead, so a way that re-enters
    // it has folded back over the coupler it is being given. Nothing refused
    // that until now: `free` above prices another wire's copper but exempts
    // the resonator's own (`owner != resonator.key`), and the
    // self-intersection test below asks whether the path meets *itself*, not
    // whether it meets the body. The option is not buildable, and it is the
    // two feedline edges that have to reach the pad which pay for it — the
    // pad they must arrive at has the resonator lying across it.
    if (std::ranges::any_of(option.way, [&](const PathPoint& cell) {
          return pad.contains(static_cast<std::size_t>(
              (static_cast<std::int64_t>(cell.y) * width) +
              static_cast<std::int64_t>(cell.x)));
        })) {
      return refuse("what is left of the resonator runs through the pad");
    }

    // Where the feedline meets it: the two ends of the far edge, a port pair
    // whose line is parallel to the pad. The chain arrives at one end, runs
    // the length of the pad — which is the coupling — and leaves at the
    // other. Both carry the pad's orientation, so the feedline runs straight
    // along it and the body drawn in the layout is parallel to the pair.
    const auto edgeX = originX + ((halfDepth + 1) * across.dx);
    const auto edgeY = originY + ((halfDepth + 1) * across.dy);
    const auto endOf = [&](const std::int64_t sign) {
      return PathPoint{
          .x = static_cast<std::uint32_t>(edgeX + (sign * halfRun * along.dx)),
          .y = static_cast<std::uint32_t>(edgeY + (sign * halfRun * along.dy)),
          .heading = option.orientation,
          .primitive = 0};
    };
    option.in = endOf(-1);
    option.out = endOf(1);
    // The pair itself, and the stub the chain runs straight on beyond each
    // end, have to be free: the edge that arrives has nowhere else to come
    // from and the edge that leaves nowhere else to go. These cells are the
    // feedline's terminal stubs, and the resonator's own head is let through
    // them below — they are what the coupling is made of.
    //
    // Both ports, the run the chain is forced to make off each of them and
    // the quarter turn at the end of that run have to be in the box. The
    // place was chosen so that at least one orientation manages it; the ones
    // that do not are refused here, or the greedy would take a port past a
    // launcher's stub and the edge into it would have to come back around —
    // the detour the box exists to stop. The run is a straight line and the
    // box is convex, so its two ends decide it, and these are the same two
    // ends `terminalsInBox` tested when the place was picked.
    {
      const std::int64_t beyond = halfRun + edgeRunOffPort(option.orientation);
      for (const std::int64_t end : {-beyond, beyond}) {
        const auto x = edgeX + (end * along.dx);
        const auto y = edgeY + (end * along.dy);
        if (!couplerBox_.holds(x, y, couplerBoxMargin())) {
          return refuse(std::format(
              "the feedline's run along the pad leaves the box at ({},{})", x,
              y));
        }
      }
    }
    std::unordered_set<std::size_t> stubs;
    for (std::int64_t k = -halfRun - tuning_.straightStart;
         k <= halfRun + tuning_.straightStart; ++k) {
      const auto x = edgeX + (k * along.dx);
      const auto y = edgeY + (k * along.dy);
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return refuse("the feedline's run along the pad leaves the grid");
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (!free(cell)) {
        return refuse(
            std::format("the feedline's run along the pad meets {}", blocker));
      }
      stubs.insert(cell);
    }

    // The resonator ports: the two ends of the near edge, parallel to the
    // feedline pair and opposite it, so a coupler carries four ports in
    // all. Which way the resonator turns off the pad is the difference
    // between them — with the clock off the first, against it off the
    // second. The two are mirror images: each faces outward along the pad,
    // away from the other, and each turns the way that brings its run to the
    // same heading — the first with the clock off the orientation, the
    // second against it off the reverse. Both therefore satisfy the rule
    // below, and which one a coupler takes is an option, not a constant. On
    // 9q one coupler of nine wants the second, and a fixed choice gave it
    // the first.
    const auto nearX = originX - ((halfDepth + 1) * across.dx);
    const auto nearY = originY - ((halfDepth + 1) * across.dy);
    const auto resonatorPort = [&](const std::int64_t sign) {
      return std::pair{nearX + (sign * halfRun * along.dx),
                       nearY + (sign * halfRun * along.dy)};
    };
    {
      const auto [firstX, firstY] = resonatorPort(1);
      option.resonatorIn = {.x = static_cast<std::uint32_t>(firstX),
                            .y = static_cast<std::uint32_t>(firstY),
                            .heading = option.orientation,
                            .primitive = 0};
      const auto [otherX, otherY] = resonatorPort(-1);
      option.resonatorOut = {.x = static_cast<std::uint32_t>(otherX),
                             .y = static_cast<std::uint32_t>(otherY),
                             .heading = routing::reverse(option.orientation),
                             .primitive = 0};
    }
    const auto& leaveFrom = secondPort ? option.resonatorOut
                                       : option.resonatorIn;

    // The head of the resonator: the lead of the component itself, already
    // built above because the pad was placed from it. It is fixed — it
    // belongs to the coupler and not to the routing — and the search that
    // draws the rest of the resonator starts on its end, which is the
    // insertion point on the path.
    // The rule that ties the coupler to its chain. Both feedline edges meet
    // this coupler at the same angle — the one edge arrives on it, the
    // other leaves on it — and that angle is what the feedline ports carry.
    // The straight run has to have come out on the orientation; if it did
    // not, the turn and the orientation disagree and the option is not the
    // component.
    if (lead.tip.heading != option.couplerOrientation) {
      return refuse("the first arc does not reach the coupler's "
                    "orientation");
    }
    // The tip is not part of the arc: the search begins on it and puts the
    // arc in front of what it finds, so a tip in both would put one cell in
    // the way twice and the self-intersection test would throw it away.
    //
    // Both pieces of the lead, walked as one: the first dogleg from the
    // port, and the second laid on its tip. The second's cells are offset
    // by the first's tip, which is how `buildDogleg` hands them back — in a
    // frame whose origin is the turn it starts on.
    std::vector<std::pair<PathPoint, bool>> leadCells;
    leadCells.reserve(lead.path.size() + jog.path.size());
    for (const auto& move : lead.path) {
      leadCells.emplace_back(move, false);
    }
    for (const auto& move : jog.path) {
      leadCells.emplace_back(move, true);
    }
    const auto jogX =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.x));
    const auto jogY =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.y));

    option.arc.clear();
    option.arc.reserve(leadCells.size());
    PathPoint tip{};
    for (std::size_t step = 0; step < leadCells.size(); ++step) {
      const auto& [move, onJog] = leadCells[step];
      const auto x =
          portX + (onJog ? jogX : 0) +
          static_cast<std::int64_t>(static_cast<std::int32_t>(move.x));
      const auto y =
          portY + (onJog ? jogY : 0) +
          static_cast<std::int64_t>(static_cast<std::int32_t>(move.y));
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return refuse("the resonator's lead leaves the grid");
      }
      if (!couplerBox_.holds(x, y, couplerBoxMargin())) {
        return refuse(std::format(
            "the resonator's lead leaves the box at ({},{})", x, y));
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      // The lead runs off the pad's own near edge, so the pad is not in its
      // way; everything else is.
      if (!pad.contains(cell) && !free(cell)) {
        return refuse(std::format("the resonator's lead meets {}", blocker));
      }
      const PathPoint here{.x = static_cast<std::uint32_t>(x),
                           .y = static_cast<std::uint32_t>(y),
                           .heading = move.heading,
                           .primitive = move.primitive};
      if (step + 1 < leadCells.size()) {
        option.arc.push_back(here);
      } else {
        tip = here;
      }
    }
    if (option.arc.empty()) {
      return refuse("the lead has no cells");
    }
    option.anchor = leaveFrom;
    option.arcEnd = tip;
    // **The resonator's own turn.** The lead already carries the straight
    // run the rule asks for off the pad — it is the quarter turn and
    // `max(COUPLER_LEAD_STRAIGHT, straightStart)` — and the search begins
    // on its tip. The first thing that search may want to do is bend, and a
    // tip on the box edge leaves it no room to, exactly as a feedline port
    // there leaves its edge none. So the `BEND_RADIUS` cells beyond the tip,
    // on the heading the tip holds, are in the rule too.
    if (couplerBoxTurn()) {
      const auto onward = routing::headingVector(tip.heading);
      // The lead already carries the resonator's own straight run. What
      // comes after it is the second one the pass may hold it to —
      // `SCPD_RESONATOR_STUB`, off by default — and then the bend.
      const auto beyondTip =
          static_cast<std::int64_t>(BEND_RADIUS) +
          (resonatorStub() ? static_cast<std::int64_t>(tuning_.straightStart)
                           : 0);
      for (std::int64_t k = 1; k <= beyondTip; ++k) {
        const auto x = static_cast<std::int64_t>(tip.x) + (k * onward.dx);
        const auto y = static_cast<std::int64_t>(tip.y) + (k * onward.dy);
        if (!couplerBox_.holds(x, y, couplerBoxMargin())) {
          return refuse(std::format(
              "the turn after the resonator's lead leaves the box at ({},{})",
              x, y));
        }
      }
    }
    // The way the option carries is the lead **and what is left to the
    // qubit**: the arc, the straight after it, and the old route onward
    // from the insertion point.
    //
    // The two join without a jump, which they did not always. While the
    // pad sat on the path and the lead hung off it, the lead ended nowhere
    // near the route and stitching them made a way that read as drawn and
    // measured as nonsense; the way was the lead alone for that reason.
    // Now the lead's tip *is* the insertion point, so the join is exact —
    // and a resonator that finds nothing under the feedline constraints
    // keeps a way that is a whole resonator rather than a stub.
    Path spliced = option.arc;
    spliced.push_back(tip);
    spliced.insert(spliced.end(), option.way.begin() + 1, option.way.end());
    if (routing::pathSelfIntersects(spliced, scene_.router.width,
                                    scene_.router.height, loopScratch_)) {
      return refuse("the spliced way meets itself");
    }
    option.way = std::move(spliced);
    return option;
  }

  /// The corridor of a feedline edge: everything free within the reach of
  /// the line between its two ends, less the bodies and the ways of the
  /// couplers it runs between. The prototype's `expand_path` of 150 cells
  /// around the two endpoints.
  /// The way an edge of a chain has now: its wire's, once it has one, or
  /// what the greedy last chose for it.
  [[nodiscard]] const Path& edgeWayOf(const std::vector<Wire>& wires,
                                      const std::uint32_t chain,
                                      const std::size_t at) const {
    const auto key = edgeWireOf_[chain][at];
    if (key != NO_OWNER && wires[key].drawn) {
      return wires[key].way;
    }
    return chainEdgePaths_[chain][at];
  }

  /// How far past the two ends of an edge its search may roam, in cells.
  static constexpr std::uint32_t EDGE_BOX_MARGIN = 250;

  /// How many edges at each end of a chain the prefix search routes again for
  /// every prefix rather than remembering by the pair of options at their two
  /// ends. The edges in between are remembered — see `Driver::solveChainAStar`.
  static constexpr std::size_t CHAIN_FRESH_EDGES = 2;

  /// Every edge of every chain but the one being routed stands in its way,
  /// inflated by the clearance — the edges that share a coupler with it
  /// included, right up to the port they share.
  ///
  /// Nothing is exempt. The direct neighbours of a path were once the one
  /// pair of feedlines nothing closed, and an edge weighed against them ran
  /// through them.
  ///
  /// What it fences is `edgeWayOf`: the edge's wire once it has one, else
  /// what the greedy last chose for it. So this is the one thing in
  /// `corridorOfEdge` that is **not** a function of the two endpoint options
  /// alone — it is what ties an edge's price to every other edge on the chip.
  void fenceCommittedEdges(const std::vector<Wire>& wires, const Edge& edge) {
    // While a chain is searching itself, its own edges may be asked to stand
    // out of the way. What they hold is the answer of the round before, and
    // fencing an edge with the last round's idea of its own neighbours is
    // what makes the rounds a walk that can circle rather than a descent.
    // The commit fences the chain fully and re-routes what no longer fits,
    // so nothing ships through a way this leaves open.
    const bool skipOwn = looseChain_ == edge.chain;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    std::size_t fenced = 0;
    std::size_t ways = 0;
    std::size_t neighbours = 0;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      if (chain != edge.chain) {
        // Another chain's feedline. It stands open unless something asks
        // for it: `edgesSeeOtherChains` is the standing answer and says no,
        // `soloChain_` says no for the prefix search whatever that answers,
        // and `fenceEverything_` overrides both for the one diagnostic that
        // has to see the whole chip.
        const bool fenceIt =
            fenceEverything_ ||
            (edgesSeeOtherChains() && soloChain_ == NO_OWNER);
        if (!fenceIt) {
          continue;
        }
      }
      const auto lastEdge = chainEdgePaths_[chain].size() - 1;
      for (std::size_t at = 0; at < chainEdgePaths_[chain].size(); ++at) {
        if (chain == edge.chain && (at == edge.from || skipOwn)) {
          continue;
        }
        // The segment the targeted repair is re-searching stands open: its
        // edges are the ones being laid again, and `edgeWayOf` would fence
        // them with the stale ways their wires still hold (2026-10-04).
        if (chain == openChain_ && at >= openLo_ && at < openHi_) {
          continue;
        }
        // A terminal edge of this edge's own chain stands open unless the
        // two are neighbours — **and the rule does not run both ways**
        // (user, 2026-10-03). A terminal edge has the least freedom on the
        // chip and the rest of the chain can work around it; that is why it
        // is let off for the edges far from it. The edge itself gets no
        // such licence: when a terminal edge is the one being drawn, every
        // edge of its chain is in its way. See `terminalEdgesFenceAll`.
        const bool drawingATerminal =
            edge.from == 0 || edge.from == lastEdge;
        if (chain == edge.chain && !fenceEverything_ &&
            !terminalEdgesFenceAll() && !drawingATerminal &&
            (at == 0 || at == lastEdge) && at + 1 != edge.from &&
            at != edge.from + 1) {
          continue;
        }
        // An edge of this chain that comes after this one stands open, so
        // that the commit asks what the search answered — **except the one
        // next to it**, which shares a coupler with it (user, 2026-10-03).
        // Two edges that meet at a coupler may not cross each other, and
        // skipping every later edge skipped that one too: the edge leaving
        // a coupler and the edge arriving at it were drawn through one
        // another. See `fenceLaterEdges`.
        if (chain == edge.chain && !fenceEverything_ && !fenceLaterEdges() &&
            at > edge.from && at != edge.from + 1) {
          continue;
        }
        // Counted only for the diagnostic: an edge that meets this one at a
        // coupler is fenced by the full clearance like every other feedline.
        // The disc that used to be left open there was the one place two
        // feedlines could come closer than the rule allows.
        const bool adjacent = chain == edge.chain &&
                              (at + 1 == edge.from || at == edge.from + 1);
        neighbours += adjacent ? 1 : 0;
        const auto& way = edgeWayOf(wires, chain, at);
        if (way.empty()) {
          continue;
        }
        ++ways;
        alongDisc(way, stencil, [&](const std::int64_t x, const std::int64_t y) {
          corridor_.set(static_cast<std::size_t>((y * width) + x), true);
          ++fenced;
        });
      }
    }
    tell(std::format("fence for edge c{}-{}to{}: {} ways ({} neighbours), "
                     "{} cells closed",
                     edge.chain, edge.from, edge.to, ways, neighbours,
                     fenced));
  }

  /// The way a resonator stands on: its own once it has been drawn from its
  /// coupler, and what its coupler's chosen option carries before that.
  [[nodiscard]] const Path& wayOfResonator(const Wire& wire) const {
    const auto owner = couplerOfWire_.find(wire.key);
    if (owner != couplerOfWire_.end() &&
        !(wire.couplerAtSource == owner->second && wire.drawn)) {
      const auto& own = couplers_[owner->second];
      return own.options[own.chosen].way;
    }
    return wire.way;
  }

  /// The room of the coupler leads that do **not** hang off the chain being
  /// routed, as one grid of bits. See `edgeKeepsEveryResonatorClear`.
  ///
  /// Why it is built per chain rather than per edge: a lead moves when its
  /// coupler takes another option, and while a chain is solved the only
  /// couplers that move are its own. Everything else stands, so one grid
  /// serves every edge search of that solve — thousands of them on 69q.
  void ensureForeignRoom(const std::vector<Wire>& wires,
                         const std::uint32_t chain) {
    if (!edgeKeepsEveryResonatorClear()) {
      return;
    }
    if (!foreignRoomStale_ && foreignRoomChain_ == chain &&
        foreignRoom_.size() == static_cast<std::size_t>(scene_.router.width) *
                                   scene_.router.height) {
      return;
    }
    foreignRoom_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    // Which couplers belong to the chain being routed, so their resonators
    // can be left out.
    std::unordered_set<std::uint32_t> mine;
    if (chain < chains_.size()) {
      for (const auto& point : chains_[chain]) {
        if (!point.fixed) {
          mine.insert(point.coupler);
        }
      }
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      if (mine.contains(index)) {
        continue;
      }
      const auto& coupler = couplers_[index];
      const auto& option = coupler.options[coupler.chosen];
      const auto& way = wayOfResonator(wires[coupler.wire]);
      const auto close = [&](const std::int64_t x, const std::int64_t y) {
        foreignRoom_.set(static_cast<std::size_t>((y * width) + x), true);
      };
      // The lead carries the clearance.
      alongDisc(fenceTheLeadOnly() ? option.arc : way, stencil, close);
      // What follows it carries only its own copper: not crossable, but no
      // room kept. See `RESONATOR_COPPER`.
      if (fenceTheLeadOnly()) {
        alongDisc(way, stencilFor(RESONATOR_COPPER), close,
                  option.arc.empty() ? 0 : option.arc.size() - 1);
      }
      // And the pad itself, copper only.
      //
      // `bodies_` is empty for the whole of the option search — it is filled
      // by `applyOption`, and that runs at the commit — so the search was
      // solving against a chip with no coupler pads on it at all, while the
      // commit builds on one with every pad standing. On 45q that cost edge
      // f47 of chain 7: a tight turn between two neighbouring couplers that
      // the search found and the commit could not build, which is what
      // `stateFaults` was reporting as the one edge that would not survive
      // (user, 2026-10-02).
      if (guardTheBodies()) {
        for (const auto cell : option.body) {
          foreignRoom_.set(cell, true);
        }
      }
    }
    foreignRoomChain_ = chain;
    foreignRoomStale_ = false;
  }

  /// Every resonator stands in the way of an edge, inflated by the
  /// clearance: a feedline comes no closer to a resonator than the rule
  /// allows, except to the resonators of the two couplers the edge runs
  /// between, within the couplers' reach, where the coupling itself is the
  /// closeness. A resonator's way is the way it has once it is cut back to
  /// its coupler, or its coupler's chosen option while the options are
  /// still being weighed.
  void fenceResonators(const std::vector<Wire>& wires, const Edge& edge) {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto& chain = chains_[edge.chain];
    std::vector<std::pair<std::uint32_t, PathPoint>> exempt;
    for (const auto at : {edge.from, edge.to}) {
      if (!chain[at].fixed) {
        const auto& coupler = couplers_[chain[at].coupler];
        exempt.emplace_back(coupler.wire, coupler.options[coupler.chosen].anchor);
      }
    }
    for (const auto& wire : wires) {
      if (!wire.resonator || !wire.feasible) {
        continue;
      }
      const Path* way = &wire.way;
      const auto coupler = couplerOfWire_.find(wire.key);
      if (coupler != couplerOfWire_.end() &&
          !(wire.couplerAtSource == coupler->second && wire.drawn)) {
        const auto& own = couplers_[coupler->second];
        way = &own.options[own.chosen].way;
      }
      const PathPoint* anchor = nullptr;
      for (const auto& [key, at] : exempt) {
        if (key == wire.key) {
          anchor = &at;
        }
      }
      const auto reach = couplerReach();
      alongDisc(*way, stencil, [&](const std::int64_t x, const std::int64_t y) {
        if (anchor != nullptr &&
            std::hypot(static_cast<double>(x) - anchor->x,
                       static_cast<double>(y) - anchor->y) <= reach) {
          return;
        }
        corridor_.set(static_cast<std::size_t>((y * width) + x), true);
      });
    }
  }

  void corridorOfEdge(const std::vector<Wire>& wires,
                      const RoutingObjective& objective, const Edge& edge) {
    // The corridor of an edge holds nothing back but the artwork. A band
    // around the beeline, the bodies, the resonators, the committed edges and
    // every port's approach used to be closed here, and it was the band and
    // the fences that bent the edges into their shapes: the search took the
    // turns the corridor left it, not the ones it would have chosen, and
    // raising the bend penalty sixteenfold changed not one point of the angle
    // cost because there was no straighter way left open to move to.
    //
    // **The artwork has to be stamped in here.** The router holds an obstacle
    // grid of its own, but it reads it only to price proximity and to measure
    // a free strip; what actually closes a cell to a search is the corridor
    // and nothing else. Starting from an empty corridor therefore opens the
    // artwork, and the edges ran straight through the launcher pads.
    //
    // `components` and not `blocked`: the artwork alone. `blocked` carries
    // two more things an edge has no reason to obey — everything outside the
    // rectangle the launchers span, and the approach band of every port —
    // and obeying them is what bent the edges before.
    //
    // Every wire an edge passes is drawn again under the feedline constraints
    // afterwards, so none of them is closed either. Nor are the stubs at the
    // edge's own two ends, nor the cells a caller hands in as `more`: the
    // artwork and the launcher terminals below, and nothing else.
    ensureForeignRoom(wires, edge.chain);
    corridor_ = scene_.components;
    // A box around the two ends, and everything outside it closed.
    //
    // Not a band along a beeline — that was what bent the edges into shapes
    // the bend penalty could not move, and it is not coming back. This is a
    // rectangle spanning the source and the target with a wide margin, so
    // an edge may still take any shape inside it; what it may not do is
    // wander the whole chip. Without it the search explores a grid of two
    // million cells to draw an edge three hundred long, and on 17q one edge
    // search cost 51 ms — the insertion's whole runtime is these searches.
    // The prototype bounds the same search the same way, with `expand_path`
    // around the path it already has.
    const auto margin = static_cast<std::int64_t>(EDGE_BOX_MARGIN);
    const auto lowX = std::min<std::int64_t>(objective.source.x, objective.target.x);
    const auto highX = std::max<std::int64_t>(objective.source.x, objective.target.x);
    const auto lowY = std::min<std::int64_t>(objective.source.y, objective.target.y);
    const auto highY = std::max<std::int64_t>(objective.source.y, objective.target.y);
    box_ = {.minX = std::max<std::int64_t>(0, lowX - margin),
            .minY = std::max<std::int64_t>(0, lowY - margin),
            .maxX = std::min<std::int64_t>(
                static_cast<std::int64_t>(scene_.router.width) - 1, highX + margin),
            .maxY = std::min<std::int64_t>(
                static_cast<std::int64_t>(scene_.router.height) - 1, highY + margin)};
    {
      const auto w = static_cast<std::int64_t>(scene_.router.width);
      const auto h = static_cast<std::int64_t>(scene_.router.height);
      // Every coupler body that stands and the room of every resonator
      // outside this chain, closed in the same pass: see `guardTheBodies`
      // and `edgeKeepsEveryResonatorClear`. Two bit tests a cell on a loop
      // that already walks the grid, against two more walks of it.
      const bool guard = guardTheBodies();
      const grid::BitGrid* foreign =
          (edgeKeepsEveryResonatorClear() && !foreignRoom_.empty())
              ? &foreignRoom_
              : nullptr;
      // The coupler box, for an edge between two couplers: see `edgeInBox`.
      const bool inBox = edgeInBox() && !edge.terminal && !couplerBox_.empty();
      for (std::int64_t y = 0; y < h; ++y) {
        const bool outsideRow = y < box_.minY || y > box_.maxY;
        for (std::int64_t x = 0; x < w; ++x) {
          const auto cell = static_cast<std::size_t>((y * w) + x);
          if (outsideRow || x < box_.minX || x > box_.maxX ||
              (inBox && !couplerBox_.holds(x, y))) {
            corridor_.set(cell, true);
          } else if ((guard && bodies_.test(cell)) ||
                     (foreign != nullptr && foreign->test(cell))) {
            corridor_.set(cell, true);
          }
        }
      }
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto& chain = chains_[edge.chain];

    // Every feedline that stands now is an obstacle to the one being drawn.
    // This is the one fence the corridor kept when the rest were opened: two
    // edges of the same chain are wires like any other pair and owe each
    // other the clearance, and an edge drawn through a chain that is already
    // there is not a way the stage can keep. The edges that meet this one at
    // a coupler are the exception, and only within that coupler's reach,
    // where they pin on the same cell on purpose.
    fenceCommittedEdges(wires, edge);

    // The edges the caller is standing on, which nothing above can close.
    //
    // `fenceCommittedEdges` reaches only what an earlier round committed,
    // and inside one solve that is the *other* assignment's ways — so
    // without this two edges of the chain being searched never see each
    // other, the search prices a pair that cannot both be built as if it
    // could, and the commit is where one of them is lost. This is that hole
    // closed: the ways the caller has already laid, fenced at the full
    // clearance like every other feedline. `prefixFence_` says which they
    // are.
    if (prefixFence_ != nullptr) {
      for (const Path* laid : *prefixFence_) {
        if (laid == nullptr || laid->empty()) {
          continue;
        }
        alongDisc(*laid, stencilFor(tuning_.clearance),
                  [&](const std::int64_t x, const std::int64_t y) {
                    corridor_.set(static_cast<std::size_t>((y * width) + x),
                                  true);
                  });
      }
    }

    // The leads of this chain's own couplers, the two at this edge's ends
    // included.
    //
    // `ensureForeignRoom` holds every other coupler's lead and leaves out
    // this chain's, because those move while it is solved. Between the two,
    // **every** lead on the chip stands in the way of every edge, inflated
    // by the clearance, with no exemption anywhere — see `terminalSlot`,
    // which is the one thing that opens again and the only thing that does.
    if (edgeKeepsEveryResonatorClear()) {
      const auto& stencil = stencilFor(tuning_.clearance);
      for (std::size_t at = 0; at < chain.size(); ++at) {
        if (chain[at].fixed) {
          continue;
        }
        const auto& coupler = couplers_[chain[at].coupler];
        const auto& option = coupler.options[coupler.chosen];
        const auto& way = wayOfResonator(wires[coupler.wire]);
        const auto close = [&](const std::int64_t x, const std::int64_t y) {
          corridor_.set(static_cast<std::size_t>((y * width) + x), true);
        };
        alongDisc(fenceTheLeadOnly() ? option.arc : way, stencil, close);
        if (fenceTheLeadOnly()) {
          alongDisc(way, stencilFor(RESONATOR_COPPER), close,
                    option.arc.empty() ? 0 : option.arc.size() - 1);
        }
        // The pad with it, copper only, and every pad of the chain rather
        // than the two at this edge's ends: these are the ones `bodies_`
        // does not hold while the chain is still searching itself.
        if (guardTheBodies()) {
          for (const auto cell : option.body) {
            corridor_.set(cell, true);
          }
        }
      }
    }

    // The launcher terminals: the one thing an edge may not run through. A
    // wire leaves its launcher on the launcher's heading and has nowhere else
    // to go, so the cell, the straight run it needs off it and the clearance
    // around the whole of it are closed.
    //
    // **Every** launcher that carries a wire, not only the ends of the
    // chains: on 4q the chains use two of sixteen, and the other fourteen are
    // the sources of the conventional wires. Closing only the chain ends let
    // the edges run right past those fourteen.
    //
    // Every launcher but the one this edge is being routed to. A chain edge
    // has at most one launcher end — the other is a coupler — so this is the
    // one terminal it has to be able to reach, and every other launcher on
    // the chip is closed to it with its clearance disc like any obstacle.
    // Opening both ends was the wider rule and had nothing to open: an edge
    // between two couplers has no launcher end at all, and one with two
    // would be a chain without a coupler, which the insertion never builds.
    std::unordered_set<std::uint32_t> mine;
    if (chain[edge.to].fixed) {
      mine.insert(chain[edge.to].port);
    } else if (chain[edge.from].fixed) {
      mine.insert(chain[edge.from].port);
    }
    const auto& stencil = stencilFor(tuning_.clearance);
    // The straight run the wire off a launcher is forced to make, and the
    // quarter turn at the end of it: the run alone leaves the wire able to
    // cross the edge and nothing else — see `launcherFenceTurn`. And the
    // margin of `edgeLauncherMargin` beyond.
    const auto launcherRun =
        static_cast<std::int64_t>(tuning_.straightStart) +
        (launcherFenceTurn() ? static_cast<std::int64_t>(BEND_RADIUS) : 0) +
        edgeLauncherMargin();
    const auto close = [&](const PathPoint& at) {
      // The straight run off the terminal, on the heading the wire leaves it
      // on, and the clearance around it.
      const auto v = routing::headingVector(at.heading);
      Path places;
      for (std::int64_t k = 0; k <= launcherRun; ++k) {
        const auto x = static_cast<std::int64_t>(at.x) + (k * v.dx);
        const auto y = static_cast<std::int64_t>(at.y) + (k * v.dy);
        if (x < 0 || y < 0 || x >= width || y >= height) {
          break;
        }
        places.push_back({.x = static_cast<std::uint32_t>(x),
                          .y = static_cast<std::uint32_t>(y),
                          .heading = at.heading,
                          .primitive = 0});
      }
      alongDisc(places, stencil,
                [&](const std::int64_t x, const std::int64_t y) {
                  corridor_.set(static_cast<std::size_t>((y * width) + x),
                                true);
                });
    };
    for (const auto& [port, slot] : scene_.launcherCell) {
      if (mine.contains(port)) {
        continue;
      }
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      close({.x = place.x(),
             .y = place.y(),
             .heading = found->second,
             .primitive = 0});
    }

    // The runs the terminal edges of every chain already settled have to
    // make at their couplers — into the first coupler, out of the last —
    // and `terminalStubExtra` cells beyond, inflated by the clearance,
    // whatever `edgesSeeOtherChains` says: an edge of a later chain drawn
    // across them is a crossing of two chains the feedline pass cannot
    // undo, because a terminal edge that finds no way keeps its way. 69q's
    // f5 and f6 crossed at the insertion that way. See `terminalStubGuard`.
    if (terminalStubGuard()) {
      guardSettledTerminalRuns(edge);
    }

    // **The slot**, and the only thing that opens again — last, so that it
    // is the last word.
    //
    // Everything within the clearance of a resonator is closed above, the
    // coupler's lead with it, and that seals the corridor at the port the
    // edge has to dock on. A straight run of `terminalSlot` cells is freed
    // at each of the two terminals and nothing else is: no coupling zone,
    // no exemption around an anchor. The prototype's `free_terminal_stub`
    // is this, down to the twenty.
    //
    // The artwork stays closed. A forced run that lies on copper is an edge
    // that cannot be built, and opening it would hide that rather than mend
    // it.
    //
    // Only where something above closes them: with the guards off this is
    // the corridor that stood.
    if (edgeKeepsResonatorClear() || guardTheBodies() ||
        edgeKeepsEveryResonatorClear()) {
      // At least the slot, and never less than the run the search is held
      // to plus the room one turn needs.
      //
      // In the prototype the slot and the run are one number: it holds the
      // router to 20 and frees 20. Here the run at a coupler is the pad's
      // own length, because the feedline lies along the whole pad — 21
      // cells on 45q — and after it the edge still has to bend away. A slot
      // of 20 left it standing at the end of the pad inside the resonator's
      // clearance, and six edges of 45q and fourteen of 57q came back with
      // no way at all.
      //
      // **Measured on 45q**, slot against edges drawn: 20 -> 46 of 52, 25 ->
      // 52, and 30, 35, 40, 60, 100 all bit-identical to 25. So what is
      // missing at 20 is a handful of cells to turn in, not room to route
      // in, and `BEND_RADIUS` is that figure by name.
      const auto [startRun, endRun] = runsOfEdge(edge);
      const auto turn = static_cast<std::uint32_t>(BEND_RADIUS);
      const auto slotFrom = std::max(terminalSlot(), startRun + turn);
      const auto slotTo = std::max(terminalSlot(), endRun + turn);
      std::size_t opened = 0;
      const auto reopen = [&](const Path& run) {
        for (const auto& point : run) {
          if (point.x >= scene_.router.width ||
              point.y >= scene_.router.height) {
            continue;
          }
          const auto cell = scene_.router.index(point.x, point.y);
          if (scene_.components.test(cell) || !corridor_.test(cell)) {
            continue;
          }
          corridor_.set(cell, false);
          ++opened;
        }
      };
      reopen(router_.straightStub(objective.source, false, slotFrom));
      reopen(router_.straightStub(objective.target, true, slotTo));
      if (opened != 0) {
        tell(std::format("edge c{}-{}to{}: {} cells of its slots ({} and {} "
                         "cells) were closed and are open again",
                         edge.chain, edge.from, edge.to, opened, slotFrom,
                         slotTo));
      }
    }
  }

  /// What a bend costs an edge of a chain. A feedline is judged by how much
  /// it turns — that is the whole objective the greedy optimises — so an edge
  /// may pay more for a bend than a wire that is only trying to get there.
  /// Read from the environment so a sweep over it costs a rebuild of nothing.
  ///
  /// At 1.0 this **is** the outer routing's own figure, 7125 on 17q and
  /// 27000 on 69q; the insertion adds nothing to it. The prototype routes
  /// its coupler edges at a flat 4000 (`FinalGrid.cpp:4963`), so ours is
  /// between two and seven times what it pays, and the factor may now go
  /// below one to meet it (user, 2026-10-03).
  [[nodiscard]] std::uint16_t edgeBendPenalty() const {
    static const double factor = [] {
      return std::clamp(envReal("SCPD_EDGE_BEND_FACTOR", 1.0), 0.01, 16.0);
    }();
    return std::max(static_cast<std::uint16_t>(1),
                     static_cast<std::uint16_t>(factor * tuning_.bendPenalty));
  }

  /// Whether the chain's options are settled by the exact layered search
  /// rather than by the greedy. Read once, like the bend factor beside it,
  /// so a run costs a rebuild of nothing.
  ///
  /// An environment switch and not a config key on purpose: `plan --stage
  /// final` reads its config out of the run directory, so an A/B on a key
  /// turns on remembering to copy the file over and is confounded the moment
  /// it is forgotten.
  ///
  /// **On by default**, and `SCPD_CHAIN_DP=0` is the way back to the greedy.
  /// It was off while it cost more than it bought; the analytic bound
  /// (`chainBound` below) turned that around. One sweep of both searches over
  /// all eight chips, a run at a time: the exact search is **faster on seven
  /// of them** — 9q 1.0 s against 9.5, 21q 2.5 against 38.8, 33q 6.4 against
  /// 90.0 — and where both draw the same edges its angle cost is better on
  /// three and level on two, never worse.
  ///
  /// **What it costs, said plainly.** 69q is still slower than the greedy,
  /// 949 s against 509. And 57q and 69q each draw one feedline edge fewer
  /// than the greedy does, 7 undrawn against 6, because neither converges and
  /// the round each keeps still carries faults. 33q is the other way round —
  /// all 40 edges where the greedy loses 2. See *Where it stands* and *What
  /// is open* in `handover-cpw-coupler-insertion.md`.
  [[nodiscard]] static int exactChainSearch() {
    static const int mode = [] {
      return envWhole("SCPD_CHAIN_DP", 1);
    }();
    return mode;
  }

  /// Which bound the layered search leans on, on the same reasoning as
  /// `exactChainSearch` above: 0 is `turnBound`, 1 the analytic answer, 2
  /// the analytic answer with every step it priced reported beside both,
  /// and 3 the analytic answer **around the artwork** — every family of
  /// ways of `k` turns tried against the artwork and the edge box, the
  /// bound raised where all of them are blocked
  /// (`AnalyticDubins::minTurnsAround`, user, 2026-10-05) — with the same
  /// audit. `SCPD_CHAIN_FAMILY_PATHS` (20000) bounds the ways tried per
  /// pair; past it the family is left unsettled.
  [[nodiscard]] static int chainBound() {
    static const int mode = [] {
      return envWhole("SCPD_CHAIN_BOUND", 1);
    }();
    return mode;
  }

  /// The fewest eighth turns an edge between two poses can make, by whichever
  /// bound is switched on. Both are lower bounds on `angleCostOf`, so the
  /// layered search is exact either way; the analytic one is simply sharper,
  /// and a sharper bound is fewer steps priced.
  [[nodiscard]] std::uint32_t boundTurns(const PathPoint& from,
                                         const PathPoint& to) const {
    if (chainBound() == 0) {
      return turnBound(from, to);
    }
    if (chainBound() == 3) {
      return boundTurnsAround(from, to);
    }
    return analytic_.minTurns(from, to);
  }

  [[nodiscard]] static std::uint64_t familyPathBudget() {
    static const auto paths = static_cast<std::uint64_t>(
        std::clamp(envWhole("SCPD_CHAIN_FAMILY_PATHS", 20000), 100, 100000000));
    return paths;
  }

  /// The analytic bound with the artwork in the way: the box is the one
  /// `corridorOfEdge` keeps, the runs at the two ends are left at zero,
  /// which is below what any edge is forced to make.
  [[nodiscard]] std::uint32_t boundTurnsAround(const PathPoint& from,
                                               const PathPoint& to) const {
    const auto margin = static_cast<std::int64_t>(EDGE_BOX_MARGIN);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const routing::AnalyticDubins::Box box{
        .minX = std::max<std::int64_t>(0, std::min<std::int64_t>(from.x, to.x) - margin),
        .minY = std::max<std::int64_t>(0, std::min<std::int64_t>(from.y, to.y) - margin),
        .maxX = std::min<std::int64_t>(width - 1, std::max<std::int64_t>(from.x, to.x) + margin),
        .maxY = std::min<std::int64_t>(height - 1, std::max<std::int64_t>(from.y, to.y) + margin)};
    const routing::AnalyticDubins::Blocked blocked =
        [this, width](const std::int64_t x, const std::int64_t y) {
          return scene_.components.test(static_cast<std::size_t>((y * width) + x));
        };
    return analytic_.minTurnsAround(from, to, blocked, box, 0, 0,
                                    familyPathBudget(), &familyStats_);
  }

  /// A round's total, or the word for a round holding a chain that nothing
  /// joins — which is not a number and must not print as one.
  [[nodiscard]] static std::string spell(const std::uint64_t total) {
    return total == routing::TRELLIS_UNREACHABLE ? std::string("unreachable")
                                                 : std::to_string(total);
  }

  /// Report one priced step against both bounds. `SCPD_CHAIN_BOUND=2` only.
  ///
  /// How sharp the bound is decides how much of the trellis has to be
  /// searched at all, so it is the figure to look at before any clock. Here
  /// it is measured against the truth — the real price of a step the search
  /// did go on to pay for — rather than guessed at from the runtime.
  void audit(const std::uint32_t chain, const std::size_t layer,
             const PathPoint& from, const PathPoint& to,
             const std::uint64_t real) {
    if (real == routing::TRELLIS_UNREACHABLE) {
      ++auditUnreachable_;
      return;
    }
    const auto truth = static_cast<std::uint32_t>(real / 10000ULL);
    const auto loose = turnBound(from, to);
    const auto sharp = analytic_.minTurns(from, to);
    if (chainBound() == 3) {
      const auto around = boundTurnsAround(from, to);
      if (around > truth) {
        ++familyViolations_;
        say(std::format("[Coupler Insertion]   AUDIT chain {} step {}: the "
                        "family bound {} is ABOVE the real {} — not admissible",
                        chain, layer, around, truth));
      }
    }
    ++auditSteps_;
    auditTurnBound_ += loose;
    auditAnalytic_ += sharp;
    auditReal_ += truth;
    auditSharper_ += (sharp > loose) ? 1U : 0U;
    auditExact_ += (sharp == truth) ? 1U : 0U;
    if (sharp > truth) {
      // The one thing that would break the exactness of the layered search.
      ++auditViolations_;
      say(std::format(
          "[Coupler Insertion]   AUDIT chain {} step {}: analytic {} is ABOVE "
          "the real {} — the bound is not admissible",
          chain, layer, sharp, truth));
    }
    tell(std::format(
        "[Coupler Insertion]   audit chain {} step {}: turnBound {}, "
        "analytic {}, real {}",
        chain, layer, loose, sharp, truth));
  }

  /// What the bound cost and how sharp it was, over the whole insertion.
  std::uint64_t boundPairs_ = 0;
  /// What the family bound did, summed over the insertion.
  mutable routing::AnalyticDubins::AroundStats familyStats_;
  std::uint32_t familyViolations_ = 0;
  /// The learned bound: how often it stood above the analytic one when a
  /// bound was asked, and how often a real price came in below it.
  std::uint64_t learnedRaised_ = 0;
  std::uint64_t learnedAbove_ = 0;
  /// The step budget of the chain A*: what the step being priced may cost,
  /// handed from the search to `routeEdge`; how many searches it cut off,
  /// and whether the last one was.
  std::uint64_t stepBudget_ = routing::TRELLIS_UNREACHABLE;
  std::uint64_t budgetCutoffs_ = 0;
  bool lastCutOff_ = false;
  std::uint64_t boundNanos_ = 0;
  std::uint32_t auditSteps_ = 0;
  std::uint32_t auditUnreachable_ = 0;
  std::uint32_t auditSharper_ = 0;
  std::uint32_t auditExact_ = 0;
  std::uint32_t auditViolations_ = 0;
  std::uint64_t auditTurnBound_ = 0;
  std::uint64_t auditAnalytic_ = 0;
  std::uint64_t auditReal_ = 0;

  /// The ways the caller has already laid, for the corridor to close along
  /// with everything committed, or null. Set around a single `routeEdge`
  /// call and cleared again.
  ///
  /// `fenceCommittedEdges` can only close what an earlier round committed,
  /// so inside one solve it holds the *other* assignment's ways. This is
  /// the hole that leaves: the ways of the edges the caller is standing on
  /// right now. The second-order trellis puts one way in it — its
  /// predecessor's — and `solveChainAStar` puts every edge of the prefix it
  /// is extending.
  const std::vector<const Path*>* prefixFence_ = nullptr;

  /// How much of the chain a node of the layered search carries: 1 is the
  /// option at its own waypoint, 2 that option **and** the one before it.
  ///
  /// At second order a step may fence the way its predecessor takes, which
  /// is what lets the search see two edges of one chain blocking each other
  /// instead of discovering it at the commit.
  ///
  /// **It applies to every chain, however wide.** It was once given up for a
  /// layer past a width cap, and the cap was doing the damage: on 57q the
  /// two chains whose jogs had opened — 48 options against 16 — were the
  /// ones handed back to the first order, and they were where the edges were
  /// lost. A layer of `n` options costs `n x n` nodes and the step matrix
  /// between two layers `n^2 x n^2`, so the widest chains are dear; that is
  /// the price of seeing the blockage at all.
  [[nodiscard]] static int chainOrder() {
    static const int order = [] {
      return envWhole("SCPD_CHAIN_ORDER", 2);
    }();
    return order;
  }

  /// Whether a chain is settled by the prefix search rather than by the
  /// trellis, on the same reasoning as `exactChainSearch` above: an
  /// environment switch and not a config key, because `plan --stage final`
  /// reads its config out of the run directory.
  ///
  /// **On by default**, and `SCPD_CHAIN_ASTAR=0` is the way back to the
  /// trellis. What it changes is the one assumption the trellis rests on —
  /// see `solveChainAStar` — and what that buys is the edges: 57q draws
  /// every one of its 66 where the trellis lost two and the greedy six, and
  /// it does so in a quarter of the time. It also needs no rounds; see
  /// `optimizeChainsPrefix`.
  /// Whether the chain search raises the bound of a pair of options to the
  /// cheapest price that pair has cost in this solve so far:
  /// `SCPD_CHAIN_LEARNED_BOUND`, **off** until measured (user, 2026-10-05).
  /// The analytic bound knows no obstacle, and on 17q the last step of
  /// chain 3 was bound 3 against real 5 on every one of ten priced runs —
  /// the whole proof was refuting prefixes the first pricing had already
  /// told the price of. A prefix only adds fences, so a pair cannot cost
  /// less under one prefix than under none; but the price seen was under
  /// *some* prefix, and the edge search minimises bends with proximity and
  /// length rather than turns alone, so this is a heuristic, not a proof:
  /// `learnedAbove_` counts the steps whose real price came in below the
  /// learned bound, which is how far from admissible it was.
  /// Whether an edge search of the chain A* is told how many turns its way
  /// may make and still leave the prefix able to beat the cheapest complete
  /// run priced so far: `SCPD_CHAIN_STEP_BUDGET`, **on** (user,
  /// 2026-10-05). The budget is what the run has left — its cost less the
  /// prefix's price less the bound on the rest — in eighth-turns, and the
  /// router drops every move that would take a way past it
  /// (`setMaxTurns`), so a step whose way needs more turns ends as no way
  /// for that prefix instead of being searched to the end. Exact: the
  /// prefix could not have won, and the complete run is in the queue. A way
  /// cut off is not put in the greedy's memo, which knows no budget.
  [[nodiscard]] static bool chainStepBudget() {
    static const bool on = envFlag("SCPD_CHAIN_STEP_BUDGET", true);
    return on;
  }

  [[nodiscard]] static bool chainLearnedBound() {
    static const bool on = envFlag("SCPD_CHAIN_LEARNED_BOUND", false);
    return on;
  }

  [[nodiscard]] static bool chainAStar() {
    static const bool on = [] {
      return envFlag("SCPD_CHAIN_ASTAR", true);
    }();
    return on;
  }

  /// How long the prefix search may spend on one chain before it settles for
  /// the cheapest complete run of options it has reached — every edge of that
  /// run drawn, and drawn against the run itself. `SCPD_CHAIN_ASTAR_SECONDS`
  /// sets it; zero is no limit and then the answer is always the optimum.
  ///
  /// Without one the search is exponential in the worst case, and it is not
  /// a theoretical worst case: on 57q one chain of nine waypoints had routed
  /// 24 897 edges in 1 400 s and was not done. Six of the eight chips never
  /// reach this limit — their whole insertion is under 70 s.
  ///
  /// **It is spent per attempt, not per chain.** A chain whose plain options
  /// do not reach opens its jogs and searches again, and the second attempt
  /// starts the clock over, so such a chain can spend twice this.
  ///
  /// **Ten seconds, because the answer arrives early and the rest is the
  /// proof.** 57q's chain 4 is the only chain on any chip that has ever
  /// reached the limit. It comes home at **220000 either way** — after 202
  /// edge searches at 10 s, after 3209 at 180 s — so the extra 170 s bought
  /// no better run of options, only the certainty that none exists. 57q
  /// still draws all 66 edges at `stateFaults` 0 (user, 2026-09-30).
  ///
  /// The cost is the word in the log: a chain that reaches the limit reports
  /// `best in the time given` and `ChainSolution::optimal` is false, so the
  /// search no longer claims an optimum it has not proved.
  [[nodiscard]] static std::chrono::nanoseconds chainAStarBudget() {
    static const auto budget = [] {
      const double seconds = envReal("SCPD_CHAIN_ASTAR_SECONDS", 10.0);
      return std::chrono::nanoseconds(
          static_cast<std::int64_t>(std::max(0.0, seconds) * 1e9));
    }();
    return budget;
  }

  /// Whether a chain is searched as though the other chains were not there:
  /// `fenceCommittedEdges` then closes none of their edges.
  ///
  /// **On by default**, and `SCPD_CHAIN_SOLO=0` is the way back.
  ///
  /// The chains are settled one after another, so fencing them against each
  /// other makes the order decide who gets the room: the first chain has the
  /// run of the chip and the last has to work around everything. That is
  /// what strangled the late chains. It moves the conflict to the commit,
  /// where an edge that no longer fits is routed again from scratch — and
  /// the commit turns out to repair those better than the fence avoided
  /// them.
  ///
  /// **Measured on 69q, one run each.** Three chains that no run of options
  /// joined — 2, 5 and 11 — all reach their optimum once the other chains
  /// stand open, and chain 5 does it in **8 edge searches against 713**.
  /// Every column improves: 78 of 81 edges drawn against 76, angle 164
  /// against 176, `stateFaults` 17 against 20, and 83 s against 192 s. The
  /// commit still has the cross-chain conflicts to settle; it settles them.
  ///
  /// What it costs is the resonators: the couplers pick other places without
  /// the fence, so 69q spans 5806 to 6130 units against 6008 to 6128, and
  /// nine rather than six sit more than 100 off the figure. All of them stay
  /// inside `couplerMaxShortfall`, so the meander has more to add and
  /// nothing is beyond it.
  ///
  /// It bears only on the prefix search — the commit and `stateFaults`
  /// always fence everything, or their test would mean nothing.
  [[nodiscard]] static bool chainSolo() {
    static const bool on = [] {
      return envFlag("SCPD_CHAIN_SOLO", true);
    }();
    return on;
  }

  /// Whether the commit takes the ways the prefix search found as they are,
  /// rather than testing each against the chip as it then stands and routing
  /// again what no longer fits.
  ///
  /// **Off by default** since 2026-10-02; `SCPD_CHAIN_KEEP_WAYS=1` keeps
  /// them again.
  ///
  /// **What it gives up, said plainly.** The test it drops is the one that
  /// stops a way drawn against one chip from shipping through another, and
  /// with `chainSolo` the ways are drawn against a chip with nothing on it —
  /// so they will cross each other. What the stage reports as drawn is then
  /// no longer what it can build: `stateFaults` is the figure that says how
  /// many of the drawn edges do not survive the chip they are drawn on, and
  /// under this switch it is the figure to read, not the undrawn count.
  ///
  /// **And it was not a figure of speech.** `checkFeedlineRoom` caught it on
  /// 69q: edge f73 of chain 10, kept as the search found it, ran inside the
  /// rule of resonator 218 at (1366,3548) — ten cells from that resonator's
  /// own coupler, which belongs to another edge entirely. One pair, and it
  /// is the only one on the eight chips. Routed again instead, the check is
  /// green and the stage ends on the same numbers it ended on before: 81 of
  /// 81 edges, 24 open, 55 fails. The kept way was not a better way, only
  /// an untested one.
  ///
  /// The endpoints are still checked. That is not a rule but an identity: a
  /// way that does not run between the ports the couplers ended on is a
  /// different edge, not a stale one.
  [[nodiscard]] static bool chainKeepWays() {
    static const bool on = [] {
      return envFlag("SCPD_CHAIN_KEEP_WAYS", false);
    }();
    return on;
  }

  // ---------------------------------------------- The length-point clearance
  //
  // The coupler insertion puts a resonator's pad where what is left of the
  // way to the qubit is the target length (`couplerPlace`). The pad and the
  // lead stick out beside the way there and need room, and a plain wire
  // that the outer routing laid too close to that place is what the
  // insertion and the feedline pass then fail on, at a spot nobody can
  // clean up any more (the room rules of 2026-10-04: R3, the lane pairs
  // leaning on a pad). The prototype's answer is its RRR-Lenpoint-Constraint
  // (`FinalGrid.cpp:6193-6600`, `:7620-7760`): in the outer routing and its
  // refinement every fence of a resonator inflates the stretch of its way
  // around that point by a larger radius, and a resonator being routed keeps
  // the same radius from the stretch of each neighbour that runs alongside
  // its own point. This is that mechanism, for the outer routing and the
  // outer refinement **only** — under the feedline constraints the couplers
  // stand and the lead and pad are fenced themselves — and **hard**: a wire
  // that cannot be routed with the clearance fails (user, 2026-10-04); the
  // prototype's second try without it is not built. Plan:
  // `plan-resonator-lenpoint.md`.

  /// The radius a wire keeps from a resonator's length-point band, in cells:
  /// `SCPD_LENPOINT_K`, **40**, the prototype's flat figure on a grid of the
  /// same pitch (user, 2026-10-04: as the prototype has it, no derivation
  /// from the pad's geometry); 0 is off. A first version took
  /// `couplerHeight + clearance + BEND_RADIUS` = 27 on the benchmarks;
  /// `artifacts/logs/lenpoint-k27` holds that arm for 4q–45q.
  ///
  /// **Measured 2026-10-04** at 40 (`artifacts/logs/lenpoint40` against
  /// `bridge-check`, eight chips, `stop_after = "feedlines"`, no repair):
  /// `bad` after the feedline routing 71 → 56 (33q 4 → 0, 45q 8 → 5, 69q
  /// 28 → 16), the stage 2400 s → 1354 s; but the outer routing no longer
  /// ends at 0 fails everywhere — 17q leaves resonator 30 unrouted (bad
  /// 8 → 12), 57q ends with 23 open and loses edge f4 of chain 0 — and the
  /// feedline angle moves with the layout (4q 8 → 12, 45q 111 → 109). See
  /// *The length-point clearance* in handover-feedline-routing.md.
  [[nodiscard]] static std::uint32_t lengthPointK() {
    static const auto k = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_LENPOINT_K", 40), 0, 200));
    return k;
  }

  /// The half-width of the band around the length point, as a share of the
  /// target length: `SCPD_LENPOINT_PCT`, 10 (per cent), the prototype's. The
  /// point moves with every reroute and every meander, so the stretch of the
  /// way between `(1 - pct)` and `(1 + pct)` of the figure is what is kept
  /// clear, not the point.
  [[nodiscard]] static double lengthPointPct() {
    static const double share =
        std::clamp(envReal("SCPD_LENPOINT_PCT", 10.0), 0.0, 100.0) / 100.0;
    return share;
  }

  /// Whether the outer routing ends on the length-point report — per chip
  /// the nearest other wire to every resonator's length point, with a
  /// histogram at fixed edges so arms at different `k` compare:
  /// `SCPD_LENPOINT_REPORT`, on. Report only.
  [[nodiscard]] static bool lengthPointReport() {
    static const bool on = envFlag("SCPD_LENPOINT_REPORT", true);
    return on;
  }

  /// Whether the outer routing ends on a recovery of its fails without the
  /// length-point clearance: `SCPD_LENPOINT_RECOVERY`, on (user,
  /// 2026-10-04). The clearance is hard inside the rounds; a wire it leaves
  /// unrouted or open is then searched once more in a pass over the ring
  /// with the bands off — the failing wires only, everything else stands.
  /// 17q's resonator 30 is the case: its own band inflates both plain
  /// neighbours by `k`, and the lane between them is too narrow for it.
  [[nodiscard]] static bool lengthPointRecovery() {
    static const bool on = envFlag("SCPD_LENPOINT_RECOVERY", true);
    return on;
  }

  // --------------------------------------------------- The targeted repair
  //
  // Phase 4 leaves wires open that no sweep closes: lane-trading pairs of
  // plain wires leaning on a coupler pad, a resonator whose lead the
  // insertion laid one cell from a neighbour, terminal edges of two chains
  // crossing. The room rules measured on 2026-10-04 that no local geometric
  // rule at the coupler separates these from healthy couplers; what reaches
  // them is a trial that changes coupler options and routes again. The
  // targeted repair is that trial done with aim: every fail is traced to the
  // chain segments that can be its cause (`blameFails`), each segment is
  // re-searched with the prefix search on its own — both ends fixed, the
  // rest of the chip frozen — and every complete run of options the search
  // reaches is tested by a local rip-up-and-reroute of the wires the segment
  // touches before it is taken. Plan: `plan-coupler-repair-search.md`.

  /// Whether phase 4's repair is the targeted re-search of coupler options
  /// (`SCPD_REPAIR_SEARCH`, **on**) or the blind `repair` of before (`=0`):
  /// turn a coupler near a fail to one of its two cheapest other options,
  /// sweep the whole ring again, keep on strictly fewer fails. The blind
  /// repair nominates no coupler for a wire that only crosses a feedline,
  /// and it has never been run against the prefix search; `=0` with
  /// `repair_trials = 0` is the stage exactly as it stood on 2026-10-04.
  ///
  /// **Measured 2026-10-04**, eight chips one at a time, 20 legality tests
  /// a chip, `bad = unrouted + open + crossing` summed over the chips
  /// (`artifacts/logs/base-ortho2`, `repair-old`, `research`,
  /// `research-exit`; handover-targeted-repair.md, *Where it stands*):
  /// no repair 77, the blind repair 73 (and 1160 s slower, buying its four
  /// with lengths), this re-search 69 at k 2 / grow 1, and 51 with
  /// `SCPD_CROSSING_EXIT_HEADING=1` — where 17q goes from 8 to 0. No chip is
  /// worse than the baseline in any arm. What stays are the lane pairs of
  /// plain wires; what goes are the crossings.
  [[nodiscard]] static bool repairSearch() {
    static const bool on = envFlag("SCPD_REPAIR_SEARCH", true);
    return on;
  }

  /// How many free waypoints a blamed run of waypoints is widened by at each
  /// end before it becomes a segment: `SCPD_RESEARCH_GROW`, 1. At 1 the two
  /// fixed ends of a segment have one coupler between them and the fail
  /// that is free to move; at 0 a blamed edge is re-routed between two
  /// couplers that stay where they are, and a blamed coupler alone yields no
  /// segment at all. Sweep 0 / 1 / 2.
  [[nodiscard]] static std::size_t researchGrow() {
    static const auto grow = static_cast<std::size_t>(
        std::clamp(envWhole("SCPD_RESEARCH_GROW", 1), 0, 10));
    return grow;
  }

  /// How many ring members on each side of a candidate's window are redrawn
  /// with it: `SCPD_RESEARCH_K`, 2. The window itself is the plain wires
  /// bridged to a segment edge, the resonators of its waypoints, a terminal
  /// edge at a launcher end, the wires the fails named, and every member
  /// whose way lies within the clearance of a new edge way or crosses one;
  /// `k` is how far beyond that the rip-up reaches along the ring. Sweep
  /// 1 / 2 / 3 (user, 2026-10-04).
  [[nodiscard]] static std::uint32_t researchK() {
    static const auto k = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_RESEARCH_K", 2), 0, 20));
    return k;
  }

  /// The clock of one segment's search, the whole enumeration of its runs
  /// **and their legality tests** included: `SCPD_RESEARCH_SECONDS`, 20 s;
  /// zero is no limit. A legality test is a local sweep and takes seconds,
  /// so the search reads the clock again after every refusal. As every
  /// wall-clock budget, it makes the answer machine-dependent
  /// (handover-chain-astar, trap 0); a run that has to reproduce to the
  /// digit sets 0 and pays for it in time.
  ///
  /// **Twenty seconds is the binding budget on the large chips** (measured
  /// 2026-10-04, `artifacts/logs/research`): on 69q the segments of chains 0
  /// and 9 — six and five edges, 8229 and 6299 option pairs — reach no
  /// complete run at all in 20 s (288 and 293 edge searches), and on 33q
  /// chain 0 tests three candidates before the clock stops it; 6 of 20
  /// legality tests were used on 69q, 7 on 33q. The trials are the budget
  /// the user set; this clock is what keeps a wide segment from eating them
  /// all, and the sweep over it (or over `SCPD_RESEARCH_GROW`, which sets
  /// the width) is still owed.
  [[nodiscard]] static std::chrono::nanoseconds researchBudget() {
    static const auto budget = [] {
      const double seconds = envReal("SCPD_RESEARCH_SECONDS", 20.0);
      return std::chrono::nanoseconds(
          static_cast<std::int64_t>(std::max(0.0, seconds) * 1e9));
    }();
    return budget;
  }

  /// How many rounds the local rip-up-and-reroute sweeps: `SCPD_RESEARCH_ROUNDS`,
  /// 2. Not 1 by default: a wire the relaxation releases late in a round
  /// would never be redrawn, and the window would end with a wire let go of
  /// and left as it was.
  [[nodiscard]] static std::uint32_t researchRounds() {
    static const auto rounds = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_RESEARCH_ROUNDS", 2), 1, 10));
    return rounds;
  }

  /// Whether the second-dogleg options are open for the couplers a segment
  /// re-searches: `SCPD_RESEARCH_JOGS`, on. The jogs are exactly the
  /// "new/old options" a re-search should explore — a pad slid sideways
  /// off the way it sits on — and the insertion opened them only where a
  /// chain hit a wall. `jogsUnlocked` is monotone, so a coupler re-searched
  /// here keeps its jogs open for any later search of its chain in the same
  /// run; the insertion is over by then and nothing else reads them.
  [[nodiscard]] static bool researchJogs() {
    static const bool on = envFlag("SCPD_RESEARCH_JOGS", true);
    return on;
  }

  /// Whether the other chains' edges, and this chain's edges outside the
  /// segment, are fenced while a segment is re-searched:
  /// `SCPD_RESEARCH_SEES_CHAINS`, on — `fenceEverything_` for the length of
  /// the search, whatever `edgesSeeOtherChains`, `terminalEdgesFenceAll` and
  /// `fenceLaterEdges` say. The insertion searches each chain on its own
  /// chip (`chainSolo`) and lets the commit settle the conflicts; here the
  /// rest of the chip is frozen and is exactly what a re-searched edge must
  /// not cross — the terminal pairs of neighbouring chains on 69q cross
  /// because nothing ever fenced them against each other.
  [[nodiscard]] static bool researchSeesChains() {
    static const bool on = envFlag("SCPD_RESEARCH_SEES_CHAINS", true);
    return on;
  }

  /// Whether one full pass over the ring follows the segments, kept only
  /// when the figure does not rise: `SCPD_RESEARCH_FINAL_SWEEP`, off until
  /// the sweep decides it. It is a measured arm, not part of the criterion
  /// (user, 2026-10-04).
  [[nodiscard]] static bool researchFinalSweep() {
    static const bool on = envFlag("SCPD_RESEARCH_FINAL_SWEEP", false);
    return on;
  }

  /// Whether the local passes of the legality tests speak: `SCPD_RESEARCH_VERBOSE`,
  /// off. A test is silenced like a trial of the blind repair, so a chip's
  /// log stays readable; with this on every `wire … · round …` line of
  /// every candidate's window is printed, with its `dead on arrival` and
  /// `in the way` verdicts — the way to read why a window wire keeps
  /// failing. Diagnostic only; it changes no decision.
  [[nodiscard]] static bool researchVerbose() {
    static const bool on = envFlag("SCPD_RESEARCH_VERBOSE", false);
    return on;
  }

  // ------------------------------------------------------- The room rules
  //
  // The insertion routes every feedline edge against the artwork, the pads,
  // the leads and the resonator tails, and never against the ring wires that
  // must cross it or run beside it afterwards. The feedline pass then draws
  // the terminal edges again but keeps every edge between two couplers as
  // the insertion laid it, so a pinch between two such edges — the two edges
  // of one coupler bending back the same way with 44 cells between them
  // where two plain wires need three times the pitch — can be mended here or
  // nowhere. The room rules measure the room an option leaves on the
  // geometry `routing/RoomRules.hpp` holds, and each refuses hard: a refused
  // option or edge does not exist for the search, nothing is priced (user,
  // 2026-10-03). Every rule has a switch, and the calibration lines
  // `reportRoom` prints at the chosen options say what each would have
  // refused whether or not it is on.

  /// The centre-to-centre pitch of two plain wires running side by side, in
  /// cells: `SCPD_ROOM_PITCH`, the clearance plus one by default — 20 on
  /// every benchmark. The design rule is 18.65 cells, so 20 is conservative
  /// by one to two cells a lane. Read by every room rule and by the
  /// calibration lines.
  [[nodiscard]] std::uint32_t roomPitch() const {
    static const int set = envWhole("SCPD_ROOM_PITCH", 0);
    return set > 0 ? static_cast<std::uint32_t>(set) : tuning_.clearance + 1;
  }

  /// The dead length at both ends of a straight run, in cells, where
  /// nothing can cross it: `SCPD_ROOM_MARGIN`, `CROSSING_REACH` (10) by
  /// default, which is the reach of the orthogonal crossing rule around a
  /// bend and at the ends of an edge. With `SCPD_ORTHO_CROSSING` off a plain
  /// wire may cross its bridged edge anywhere, and 0 is the margin that
  /// models that regime.
  [[nodiscard]] static std::uint32_t roomMargin() {
    static const auto cells = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_ROOM_MARGIN", CROSSING_REACH), 0, 200));
    return cells;
  }

  /// How far behind the pad the channel rule follows each edge, in cells:
  /// `SCPD_ROOM_CHANNEL_REACH`, 80 by default. The in-edge is followed this
  /// far back from its pad run and the out-edge twice as far forward from
  /// its own.
  [[nodiscard]] static std::size_t roomChannelReach() {
    static const auto cells = static_cast<std::size_t>(
        std::clamp(envWhole("SCPD_ROOM_CHANNEL_REACH", 80), 1, 2000));
    return cells;
  }

  /// How the channel rule counts the wires that have to pass through a
  /// coupler's channel: `SCPD_ROOM_CHANNEL_COUNT`. 1 (default) reads the
  /// ring wires' ways as they stand along the line across the channel; 2
  /// takes the ring count — the wires that must cross the two edges of the
  /// coupler — when both arms lie on one side of the pad, and 1 otherwise.
  [[nodiscard]] static int roomChannelCount() {
    static const int mode =
        std::clamp(envWhole("SCPD_ROOM_CHANNEL_COUNT", 1), 1, 2);
    return mode;
  }

  /// Whether the insertion says, at the chosen options, what every room
  /// rule would have refused, and checks the channels between chains:
  /// `SCPD_ROOM_REPORT`, on. Report only; nothing is refused by it.
  [[nodiscard]] static bool roomReport() {
    static const bool on = envFlag("SCPD_ROOM_REPORT", true);
    return on;
  }

  /// Whether the insertion measures, at its end, the room every edge leaves
  /// beside it for the wires that have to pass between it and the nearest
  /// qubit or coupler: `SCPD_SQUEEZE_REPORT`, on (user, 2026-10-05). Report
  /// only — nothing is refused by it — and what marks an edge `Squeezed` in
  /// the artifact, so that a picture of the `couplers` phase shows where
  /// the feedline pass is going to run out of room before it has run. See
  /// `reportSqueeze`.
  [[nodiscard]] static bool squeezeReport() {
    static const bool on = envFlag("SCPD_SQUEEZE_REPORT", true);
    return on;
  }
  /// Every how many cells of an edge the room beside it is measured:
  /// `SCPD_SQUEEZE_STEP`, 5.
  [[nodiscard]] static std::uint32_t squeezeStep() {
    static const auto step = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_SQUEEZE_STEP", 5), 1, 100));
    return step;
  }
  /// How far from an edge an obstacle still counts as the wall of its
  /// channel, in cells: `SCPD_SQUEEZE_REACH`, 300. Beyond it the room is
  /// open and nothing is measured.
  [[nodiscard]] static std::uint32_t squeezeReach() {
    static const auto reach = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_SQUEEZE_REACH", 300), 10, 5000));
    return reach;
  }

  /// Whether a debug run draws a picture of every search:
  /// `SCPD_SEARCH_PICTURES`, on. Off keeps the pictures of the whole chip
  /// alone, which a run of thousands of searches otherwise buries.
  [[nodiscard]] static bool searchPictures() {
    static const bool on = envFlag("SCPD_SEARCH_PICTURES", true);
    return on;
  }

  /// Whether the insertion looks for the bottlenecks of the chip it leaves
  /// behind and checks the capacity graph they make: `SCPD_BOTTLENECKS`,
  /// **off** (user, 2026-10-07). Report only. See `reportBottlenecks`.
  ///
  /// Read at every insertion rather than once, so that a run can switch it
  /// on for one chip without the process remembering it.
  [[nodiscard]] static bool bottleneckReport() {
    return envFlag("SCPD_BOTTLENECKS", false);
  }
  /// How many wires the widest gap `reportBottlenecks` still reports would
  /// hold: `SCPD_BOTTLENECK_WIRES`, 10 (user, 2026-10-07).
  [[nodiscard]] static std::uint32_t bottleneckWires() {
    static const auto wires = static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_BOTTLENECK_WIRES", 10), 1, 100));
    return wires;
  }
  /// How far `wallsAfterInsertion` inflates a port run or a feedline
  /// edge: half the wire clearance, in cells (user, 2026-10-07).
  [[nodiscard]] double wallInflation() const {
    return 0.5 * static_cast<double>(tuning_.clearance);
  }
  /// The cells within `wallInflation` of a cell, as offsets.
  [[nodiscard]] std::vector<std::pair<std::int64_t, std::int64_t>>
  inflationStencil() const {
    const auto radius = wallInflation();
    const auto reach = static_cast<std::int64_t>(std::floor(radius));
    std::vector<std::pair<std::int64_t, std::int64_t>> offsets;
    for (std::int64_t dy = -reach; dy <= reach; ++dy) {
      for (std::int64_t dx = -reach; dx <= reach; ++dx) {
        if (static_cast<double>((dx * dx) + (dy * dy)) <= radius * radius) {
          offsets.emplace_back(dx, dy);
        }
      }
    }
    return offsets;
  }

  /// How long the capacity check of `reportCapacityGraph` may search, in
  /// seconds: `SCPD_CAPACITY_SECONDS`, 30.
  [[nodiscard]] static double capacitySeconds() {
    static const auto seconds =
        std::clamp(envReal("SCPD_CAPACITY_SECONDS", 30.0), 1.0, 3600.0);
    return seconds;
  }
  /// `grid::BottleneckOptions::minimumRise` for `reportBottlenecks`, in
  /// cells: `SCPD_BOTTLENECK_RISE`, 2. Zero keeps every local minimum of
  /// the clearance along the axis.
  [[nodiscard]] static double bottleneckRise() {
    static const auto cells =
        std::clamp(envReal("SCPD_BOTTLENECK_RISE", 2.0), 0.0, 1000.0);
    return cells;
  }

  /// What the room rules did in one insertion, for the summary line.
  struct RoomStats {
    std::uint32_t r1Asked = 0;
    std::uint32_t r1Refused = 0;
    std::uint32_t r2Asked = 0;
    std::uint32_t r2Refused = 0;
    /// Places the option's own-room rule refused.
    std::uint32_t r3Refused = 0;
    /// Couplers built without the own-room rule, having too few options
    /// under it.
    std::uint32_t r3StoodDown = 0;
    /// Couplers that ended more than 100 units off the design figure.
    std::uint32_t r3Moved = 0;
    /// The chains each rule was lifted for.
    std::vector<std::uint32_t> liftedR1;
    std::vector<std::uint32_t> liftedR2;
    std::vector<std::uint32_t> liftedR3;
    /// At the commit: edges whose straights carry fewer lanes than wires
    /// must cross them, and coupler channels narrower than the wires in
    /// them need.
    std::uint32_t committedBelowCapacity = 0;
    std::uint32_t committedTightChannels = 0;
    /// The same, counting only the wires that run along the channel.
    std::uint32_t committedTightAlong = 0;
    std::uint32_t couplerChannels = 0;
    /// At the commit: couplers whose chosen option has a plain wire's copper
    /// within 0 / 5 / 10 / 19 cells of its pad, run or lead — what R3 would
    /// refuse at each reach.
    std::array<std::uint32_t, 4> r3Near{};
    /// The channels between two chains, and how many are too narrow.
    std::uint32_t crossChainChannels = 0;
    std::uint32_t crossChainTight = 0;
  };
  RoomStats room_;
  /// The edges `reportSqueeze` found leaving too little room, by wire key,
  /// with the figures in words — what the artifact carries as `Squeezed`
  /// and `note` on every snapshot from the coupler insertion on.
  std::unordered_map<std::uint32_t, Squeeze> squeezed_;

  /// The outer ring in ring order — every wire that is not inner, in the
  /// order the assignment feeds them — and the ring place of every slot.
  /// What `assignBridges` walks, held here so the insertion can walk it
  /// before the chains are settled.
  std::vector<std::uint32_t> ring_;
  std::unordered_map<std::uint32_t, std::uint32_t> ringPlace_;
  /// R0: for every chain and edge, the plain ring wires that must cross
  /// that edge. Empty for a terminal edge. See `bridgersOf`.
  std::vector<std::vector<std::vector<std::uint32_t>>> bridgers_;
  /// The chain and waypoint every coupler stands at.
  std::vector<std::pair<std::uint32_t, std::size_t>> chainOfCoupler_;
  /// Which room rules are lifted for a chain: the stand-down ladder's
  /// state, one flag per chain. Nothing but the rules reads them.
  std::vector<bool> liftedR1_;
  std::vector<bool> liftedR2_;
  std::vector<bool> liftedR3_;

  /// R0: the plain ring wires that must cross one edge of a chain.
  ///
  /// The walk `assignBridges` makes, done once here so the room rules can
  /// ask before the chain is settled: the ring wires that are not
  /// resonators, are feasible, and whose slot lies strictly between the
  /// slots of the two couplers' resonators, walking the ring forward from
  /// the `from` resonator. A terminal edge has none. Both arcs are walked
  /// and the shorter kept — a chain whose nodes ran against ring order
  /// would otherwise count nearly the whole ring — and since
  /// `assignBridges` itself only walks forward, the line says when the two
  /// differ; `checkBridgers` in phase 4 must then report 0.
  [[nodiscard]] std::vector<std::uint32_t>
  bridgersOf(const std::vector<Wire>& wires, const std::uint32_t chain,
             const std::size_t at) const {
    std::vector<std::uint32_t> none;
    const auto& points = chains_[chain];
    if (at + 1 >= points.size() || points[at].fixed || points[at + 1].fixed ||
        ring_.empty()) {
      return none;
    }
    const auto ring = static_cast<std::uint32_t>(ring_.size());
    const auto from =
        ringPlace_.find(wires[couplers_[points[at].coupler].wire].slot);
    const auto to =
        ringPlace_.find(wires[couplers_[points[at + 1].coupler].wire].slot);
    if (from == ringPlace_.end() || to == ringPlace_.end()) {
      return none;
    }
    const auto walk = [&](const std::uint32_t a, const std::uint32_t b) {
      std::vector<std::uint32_t> keys;
      for (auto p = (a + 1) % ring; p != b; p = (p + 1) % ring) {
        const auto& wire = wires[ring_[p]];
        if (!wire.resonator && wire.feasible) {
          keys.push_back(wire.key);
        }
      }
      return keys;
    };
    auto forward = walk(from->second, to->second);
    auto backward = walk(to->second, from->second);
    if (backward.size() < forward.size()) {
      tell(std::format("[Coupler Insertion] chain {} edge {}->{}: the ring "
                       "runs the other way round it ({} wires forward, {} "
                       "back); the shorter arc is taken",
                       chain, at, at + 1, forward.size(), backward.size()));
      return backward;
    }
    return forward;
  }

  /// The names of a list of wires, for a line.
  [[nodiscard]] static std::string
  namesOf(const std::vector<Wire>& wires,
          const std::vector<std::uint32_t>& keys) {
    std::string out;
    for (const auto key : keys) {
      out += (out.empty() ? "" : " ") + wireId(wires[key]);
    }
    return out.empty() ? std::string("-") : out;
  }

  /// After `assignBridges`: whether the edge every plain wire was told to
  /// cross is the edge the insertion counted it against (R0). A difference
  /// means the room rules were fed a wrong count; it must be 0 on every
  /// chip.
  void checkBridgers(const std::vector<Wire>& wires) const {
    std::unordered_map<std::uint32_t, std::uint32_t> expected;
    for (std::size_t chain = 0; chain < bridgers_.size(); ++chain) {
      for (std::size_t at = 0; at < bridgers_[chain].size(); ++at) {
        for (const auto key : bridgers_[chain][at]) {
          expected[key] = edgeWireOf_[chain][at];
        }
      }
    }
    std::uint32_t differ = 0;
    std::string named;
    for (const auto key : ring_) {
      const auto& wire = wires[key];
      if (wire.resonator || !wire.feasible) {
        continue;
      }
      const auto found = expected.find(key);
      const auto want = found == expected.end() ? NO_OWNER : found->second;
      if (want == wire.bridged) {
        continue;
      }
      ++differ;
      if (differ <= 12) {
        named += std::format(
            "{}{} crosses {} where the insertion counted it on {}",
            named.empty() ? ": " : " · ", wireId(wire),
            wire.bridged == NO_OWNER ? std::string("no edge")
                                     : wireId(wires[wire.bridged]),
            want == NO_OWNER ? std::string("no edge") : wireId(wires[want]));
      }
    }
    say(std::format("feedline routing: CHECK bridges — {} wire{} whose "
                    "bridged edge differs from the insertion's count{}",
                    differ, differ == 1 ? "" : "s",
                    differ == 0 ? "; the check is GREEN" : named));
  }

  /// The channel a coupler's two edges leave behind its pad, measured.
  struct CouplerChannel {
    routing::Channel channel;
    /// The plain wires whose ways lie in the channel, by key.
    std::vector<std::uint32_t> inside;
    /// Those of them that run along the channel rather than across it.
    std::vector<std::uint32_t> along;
    /// Whether the arms come closer than where they leave their pad runs.
    /// Arms that run on or apart leave no channel to measure.
    bool narrows = false;
    /// Whether the resonator's own way runs through the channel.
    bool resonatorInside = false;
    /// Whether both arms lie on the same side of the pad.
    bool sameSide = false;
    /// How many wires must pass through: `m`.
    std::uint32_t need = 0;
  };

  /// R2's measurement: how close the edge arriving at a pad and the edge
  /// leaving it come to each other behind the pad, and how many plain wires
  /// lie between them there.
  ///
  /// Both pad runs are skipped (`run + 1` cells); the in-edge is followed
  /// `reach` cells back and the out-edge twice as far forward. `m` is read
  /// off the field: for every cell of the in-arm, the cells on the line to
  /// its nearest out-arm cell name their owners, and every distinct plain
  /// ring wire among them counts once — plus one when the resonator's own
  /// way past its lead lies on one of those lines. With
  /// `roomChannelCount() == 2` and both arms on one side, `m` is instead
  /// the ring count: the wires that must cross the two edges.
  [[nodiscard]] CouplerChannel
  channelAt(const std::vector<Wire>& wires, const CouplerOption& option,
            const std::uint32_t resonatorKey, const Path& in, const Path& out,
            const std::size_t reach, const std::uint32_t ringCount) const {
    CouplerChannel found;
    const std::size_t skip = option.run + 1;
    found.channel =
        routing::channelBetween(in, skip, reach, out, skip, 2 * reach);
    if (!found.channel.measured) {
      return found;
    }
    found.narrows = found.channel.gap < found.channel.startGap - 0.5;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto aEnd = in.size() - skip;
    const auto aBegin = aEnd > reach ? aEnd - reach : std::size_t{0};
    const auto bBegin = skip;
    const auto bEnd = std::min(out.size(), bBegin + (2 * reach));
    std::unordered_set<std::size_t> lineCells;
    std::set<std::uint32_t> owners;
    // A wire runs along the channel when its cells on the lines keep within
    // an eighth of the arm's heading or its reverse; one that crosses the
    // channel meets the lines at a right angle. Half a pitch of such cells
    // makes a wire an along-runner.
    std::unordered_map<std::uint32_t, std::unordered_map<std::size_t, Heading>>
        headingOf;
    std::unordered_map<std::uint32_t, std::uint32_t> parallel;
    for (std::size_t i = aBegin; i < aEnd; ++i) {
      std::size_t nearest = bBegin;
      auto least = std::numeric_limits<double>::infinity();
      for (std::size_t j = bBegin; j < bEnd; ++j) {
        const auto dx =
            static_cast<double>(in[i].x) - static_cast<double>(out[j].x);
        const auto dy =
            static_cast<double>(in[i].y) - static_cast<double>(out[j].y);
        const auto d = (dx * dx) + (dy * dy);
        if (d < least) {
          least = d;
          nearest = j;
        }
      }
      routing::supercoverLine(
          in[i].x, in[i].y, out[nearest].x, out[nearest].y,
          [&](const std::int64_t x, const std::int64_t y) {
            if (x < 0 || y < 0 || x >= width || y >= height) {
              return;
            }
            const auto cell = static_cast<std::size_t>((y * width) + x);
            lineCells.insert(cell);
            const auto owner = field_.owner(cell);
            if (owner != NO_OWNER && owner != resonatorKey &&
                owner < conventional_.size() && conventional_[owner] != 0) {
              owners.insert(owner);
              auto known = headingOf.find(owner);
              if (known == headingOf.end()) {
                std::unordered_map<std::size_t, Heading> cells;
                for (const auto& point : wires[owner].way) {
                  if (point.x < scene_.router.width &&
                      point.y < scene_.router.height) {
                    cells[scene_.router.index(point.x, point.y)] =
                        point.heading;
                  }
                }
                known = headingOf.emplace(owner, std::move(cells)).first;
              }
              const auto heading = known->second.find(cell);
              if (heading != known->second.end() &&
                  routing::headingDistance(heading->second, in[i].heading) !=
                      2) {
                ++parallel[owner];
              }
            }
          });
    }
    found.inside.assign(owners.begin(), owners.end());
    for (const auto owner : found.inside) {
      const auto count = parallel.find(owner);
      if (count != parallel.end() && count->second >= roomPitch() / 2) {
        found.along.push_back(owner);
      }
    }
    for (std::size_t at = option.arc.size(); at < option.way.size(); ++at) {
      const auto& cell = option.way[at];
      if (cell.x < scene_.router.width && cell.y < scene_.router.height &&
          lineCells.contains(scene_.router.index(cell.x, cell.y))) {
        found.resonatorInside = true;
        break;
      }
    }
    // Which side of the pad each arm lies on, read `K` cells past the stubs
    // — the far-edge side is where `across` points.
    const auto across =
        routing::headingVector(routing::turned(option.orientation, -2));
    const auto K = std::min<std::size_t>(reach, 30);
    const auto aK = aEnd > K ? std::max(aBegin, aEnd - K) : aBegin;
    const auto bK = std::min(bEnd - 1, bBegin + K - 1);
    found.sameSide = routing::sameSide(option.centre, across, in[aK], out[bK]);
    found.need = static_cast<std::uint32_t>(owners.size()) +
                 (found.resonatorInside ? 1U : 0U);
    if (roomChannelCount() == 2 && found.sameSide) {
      found.need = ringCount;
    }
    return found;
  }

  /// The crossing capacity of one edge's way with the pad runs and stubs at
  /// its two ends dropped: `max(terminalSlot(), run + BEND_RADIUS)` at each
  /// end, the figures `corridorOfEdge` and `checkFeedlineRoom` use.
  [[nodiscard]] routing::CrossingCapacity
  capacityOfEdge(const Edge& edge, const Path& way) const {
    const auto [startRun, endRun] = runsOfEdge(edge);
    const auto turn = static_cast<std::uint32_t>(BEND_RADIUS);
    return routing::crossingCapacity(
        way, std::max(terminalSlot(), startRun + turn),
        std::max(terminalSlot(), endRun + turn), roomPitch(), roomMargin());
  }

  /// What a list of run lengths reads as.
  [[nodiscard]] static std::string
  runsOf(const std::vector<std::uint32_t>& runs) {
    std::string out;
    for (const auto run : runs) {
      out += (out.empty() ? "" : " ") + std::to_string(run);
    }
    return out.empty() ? std::string("-") : out;
  }

  /// The room figures at the chosen options, after the commit: what R1
  /// would say of every edge between two couplers and what R2 would say of
  /// every coupler's channel, one line each, and the channels between
  /// chains (R4). Nothing here refuses; it is how the rules are calibrated
  /// before one of them is live, and what the summary line counts.
  void reportRoom(const std::vector<Wire>& wires) {
    if (!roomReport()) {
      return;
    }
    const auto pitch = roomPitch();
    // R1 at the chosen options.
    for (const auto& edge : edges_) {
      if (edge.terminal || edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.drawn) {
        continue;
      }
      const auto& need = bridgers_[edge.chain][edge.from];
      const auto capacity = capacityOfEdge(edge, wire.way);
      const bool below = capacity.lanes < need.size();
      if (below) {
        ++room_.committedBelowCapacity;
      }
      // The same runs at no margin, which is the figure for the regime
      // without the orthogonal crossing rule, where a wire may cross its
      // bridged edge anywhere.
      std::uint32_t loose = 0;
      for (const auto run : capacity.runs) {
        loose += routing::lanesOf(run, pitch, 0);
      }
      tell(std::format(
          "[Coupler Insertion] room R1 chain {} edge {}->{} (f{}): {} plain "
          "wire{} must cross it ({}); its straights carry {} lane{} over {} "
          "run{} of {} cells (pitch {}, margin {}; {} at margin 0) — {}",
          edge.chain, edge.from, edge.to, wire.slot, need.size(),
          need.size() == 1 ? "" : "s", namesOf(wires, need), capacity.lanes,
          capacity.lanes == 1 ? "" : "s", capacity.runs.size(),
          capacity.runs.size() == 1 ? "" : "s", runsOf(capacity.runs), pitch,
          roomMargin(), loose, below ? "R1 would refuse" : "room enough"));
    }
    // R2 at the chosen options.
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      const auto& coupler = couplers_[index];
      if (coupler.edgeIn == NO_OWNER || coupler.edgeOut == NO_OWNER) {
        continue;
      }
      const auto& in = wires[coupler.edgeIn];
      const auto& out = wires[coupler.edgeOut];
      if (!in.drawn || !out.drawn) {
        continue;
      }
      const auto& option = coupler.options[coupler.chosen];
      const auto [chain, at] = chainOfCoupler_[index];
      if (chain == NO_OWNER) {
        continue;
      }
      const auto ringCount = static_cast<std::uint32_t>(
          (at >= 1 ? bridgers_[chain][at - 1].size() : 0) +
          (at < bridgers_[chain].size() ? bridgers_[chain][at].size() : 0));
      const auto found = channelAt(wires, option, coupler.wire, in.way, out.way,
                                   roomChannelReach(), ringCount);
      if (!found.channel.measured) {
        continue;
      }
      ++room_.couplerChannels;
      if (!found.narrows) {
        tell(std::format(
            "[Coupler Insertion] room R2 chain {} coupler '{}' option {}: "
            "the arms of f{} and f{} do not come closer than at the pad "
            "({:.1f} cells, {}) — silent",
            chain, wireId(wires[coupler.wire]), coupler.chosen, in.slot,
            out.slot, found.channel.startGap,
            found.sameSide ? "same side" : "other sides"));
        continue;
      }
      const auto needs = (found.need + 1) * pitch;
      const bool tight = found.need >= 1 && found.channel.gap < needs;
      const auto alongNeed = static_cast<std::uint32_t>(found.along.size()) +
                             (found.resonatorInside ? 1U : 0U);
      const bool tightAlong =
          alongNeed >= 1 && found.channel.gap < (alongNeed + 1) * pitch;
      if (tight) {
        ++room_.committedTightChannels;
      }
      if (tightAlong) {
        ++room_.committedTightAlong;
      }
      tell(std::format(
          "[Coupler Insertion] room R2 chain {} coupler '{}' option {}: "
          "channel between f{} and f{} narrows to {:.1f} cells at ({},{}), "
          "{} cells past the pad (from {:.1f} at the pad), {}; {} wire{} in "
          "it ({}{}), {} running along it ({}); need {} — {}; along-runners "
          "alone need {} — {}",
          chain, wireId(wires[coupler.wire]), coupler.chosen, in.slot, out.slot,
          found.channel.gap, in.way[found.channel.at].x,
          in.way[found.channel.at].y, found.channel.aDepth,
          found.channel.startGap, found.sameSide ? "same side" : "other sides",
          found.need, found.need == 1 ? "" : "s", namesOf(wires, found.inside),
          found.resonatorInside ? " + its own resonator" : "",
          found.along.size(), namesOf(wires, found.along), needs,
          tight ? "R2 would refuse" : "room enough", (alongNeed + 1) * pitch,
          tightAlong ? "would refuse" : "room enough"));
    }
    // R3 at the chosen options: how close the nearest plain wire's copper
    // lies to the pad, the feedline's run along it and the lead.
    {
      const std::uint32_t farthest = 19;
      const auto& stencil = stencilFor(farthest);
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      const auto height = static_cast<std::int64_t>(scene_.router.height);
      for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
        const auto& coupler = couplers_[index];
        const auto [chain, at] = chainOfCoupler_[index];
        if (chain == NO_OWNER) {
          continue;
        }
        const auto& option = coupler.options[coupler.chosen];
        // The option's own cells, each with the part it belongs to.
        std::vector<std::pair<std::size_t, const char*>> own;
        for (const auto cell : option.body) {
          own.emplace_back(cell, "pad");
        }
        for (const auto& cell : option.arc) {
          if (cell.x < scene_.router.width && cell.y < scene_.router.height) {
            own.emplace_back(scene_.router.index(cell.x, cell.y), "lead");
          }
        }
        const auto along = routing::headingVector(option.orientation);
        const auto reach = static_cast<std::int64_t>(option.run / 2) +
                           static_cast<std::int64_t>(tuning_.straightStart);
        const std::int64_t midX =
            (static_cast<std::int64_t>(option.in.x) + option.out.x) / 2;
        const std::int64_t midY =
            (static_cast<std::int64_t>(option.in.y) + option.out.y) / 2;
        for (std::int64_t k = -reach; k <= reach; ++k) {
          const auto x = midX + (k * along.dx);
          const auto y = midY + (k * along.dy);
          if (x >= 0 && y >= 0 && x < width && y < height) {
            own.emplace_back(static_cast<std::size_t>((y * width) + x), "run");
          }
        }
        // The nearest plain wire to the pad and the lead, and apart from it
        // the nearest to the feedline's run along the pad: a plain wire
        // may cross the run's stubs — that is where it crosses its bridged
        // edge — so a wire at the run says nothing about the room, and the
        // counts below are over the pad and the lead alone.
        struct Nearest {
          double distance = std::numeric_limits<double>::infinity();
          std::uint32_t who = NO_OWNER;
          const char* part = "";
          std::int64_t x = 0;
          std::int64_t y = 0;
        };
        Nearest own_;
        Nearest run_;
        for (const auto& [cell, name] : own) {
          Nearest& best = name[0] == 'r' ? run_ : own_;
          const auto cx = static_cast<std::int64_t>(cell % scene_.router.width);
          const auto cy = static_cast<std::int64_t>(cell / scene_.router.width);
          for (const auto& [dx, dy] : stencil.full) {
            const auto d =
                std::sqrt(static_cast<double>((dx * dx) + (dy * dy)));
            if (d >= best.distance) {
              continue;
            }
            const auto x = cx + dx;
            const auto y = cy + dy;
            if (x < 0 || y < 0 || x >= width || y >= height) {
              continue;
            }
            const auto owner =
                field_.owner(static_cast<std::size_t>((y * width) + x));
            if (owner == NO_OWNER || owner == coupler.wire ||
                owner >= conventional_.size() || conventional_[owner] == 0) {
              continue;
            }
            best = {
                .distance = d, .who = owner, .part = name, .x = cx, .y = cy};
          }
        }
        const auto said = [&](const Nearest& found, const char* what) {
          if (found.who == NO_OWNER) {
            return std::format("no plain wire within {} cells of its {}",
                               farthest, what);
          }
          return std::format("plain wire {} lies {:.1f} cells from its {} at "
                             "({},{})",
                             wireId(wires[found.who]), found.distance,
                             found.part, found.x, found.y);
        };
        if (own_.who != NO_OWNER) {
          room_.r3Near[0] += own_.distance <= 0.0 ? 1 : 0;
          room_.r3Near[1] += own_.distance <= 5.0 ? 1 : 0;
          room_.r3Near[2] += own_.distance <= 10.0 ? 1 : 0;
          room_.r3Near[3] += own_.distance <= 19.0 ? 1 : 0;
        }
        tell(std::format(
            "[Coupler Insertion] room R3 chain {} coupler '{}' option {}: {}"
            "{}; {}",
            chain, wireId(wires[coupler.wire]), coupler.chosen,
            said(own_, "pad or lead"),
            own_.who == NO_OWNER
                ? std::string{}
                : std::format(
                      " — R3 would refuse at a reach of {} or more",
                      static_cast<std::uint32_t>(std::ceil(own_.distance))),
            said(run_, "run")));
      }
    }
    checkChainChannels(wires);
  }

  /// The room a way leaves beside it, measured as `reportSqueeze` says:
  /// the worst of the straight lines from every `squeezeStep`-th cell past
  /// `skipStart` and before `skipEnd`, to either side at a right angle, to
  /// the first artwork cell inside the launcher rectangle. `selfKey` is the
  /// wire whose own copper is not a wire in the channel. Returns the
  /// shortfall in cells, zero when nothing is short, and what was found.
  [[nodiscard]] std::pair<double, Squeeze>
  measureSqueeze(const std::vector<Wire>& wires, const Path& way,
                 const std::size_t skipStart, const std::size_t skipEnd,
                 const std::uint32_t selfKey) const {
    double worst = 0.0;
    Squeeze found;
    if (way.size() < 3 || skipStart + skipEnd >= way.size()) {
      return {worst, found};
    }
    const auto pitch = roomPitch();
    const auto clearance = tuning_.clearance;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto reach = squeezeReach();
    // The rectangle the launcher cells span: the artwork inside it is the
    // circuit, the artwork outside it the launcher pads.
    std::int64_t inMinX = width;
    std::int64_t inMinY = height;
    std::int64_t inMaxX = -1;
    std::int64_t inMaxY = -1;
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto place = scene_.router.cell(slot);
      inMinX = std::min<std::int64_t>(inMinX, place.x());
      inMaxX = std::max<std::int64_t>(inMaxX, place.x());
      inMinY = std::min<std::int64_t>(inMinY, place.y());
      inMaxY = std::max<std::int64_t>(inMaxY, place.y());
    }
    const auto inside = [&](const std::int64_t x, const std::int64_t y) {
      return x > inMinX && x < inMaxX && y > inMinY && y < inMaxY;
    };
    // A resonator whose coupler is not applied yet holds its whole outer
    // way in the field, the tail past the anchor included — copper the cut
    // takes away. Measured at search time that tail counted as a wire in
    // the channel, and on 17q a chain whose edges were squeezed by nothing
    // at the end paid 40000 to keep clear of its own resonators' tails
    // (user, 2026-10-05). So such a resonator counts only where the cell
    // lies on its way as the chosen option cuts it, `wayOfResonator`.
    std::unordered_map<std::uint32_t, std::unordered_set<std::size_t>> cut;
    const auto onCutWay = [&](const std::uint32_t owner, const std::size_t index) {
      const auto coupler = couplerOfWire_.find(owner);
      if (coupler == couplerOfWire_.end()) {
        return true;
      }
      const auto& wire = wires[owner];
      if (wire.couplerAtSource == coupler->second && wire.drawn) {
        return true;
      }
      auto found = cut.find(owner);
      if (found == cut.end()) {
        std::unordered_set<std::size_t> cells;
        for (const auto& point : wayOfResonator(wire)) {
          if (point.x < scene_.router.width && point.y < scene_.router.height) {
            cells.insert(scene_.router.index(point.x, point.y));
          }
        }
        found = cut.emplace(owner, std::move(cells)).first;
      }
      return found->second.contains(index);
    };
    for (std::size_t at = skipStart; at + skipEnd < way.size();
         at += squeezeStep()) {
      const auto& cell = way[at];
      for (const int side : {2, -2}) {
        const auto v = routing::headingVector(routing::turned(cell.heading, side));
        if (v.dx == 0 && v.dy == 0) {
          continue;
        }
        const bool diagonal = v.dx != 0 && v.dy != 0;
        std::unordered_set<std::uint32_t> owners;
        bool wall = false;
        std::int64_t x = cell.x;
        std::int64_t y = cell.y;
        std::uint32_t steps = 0;
        const auto noteOwner = [&](const std::int64_t ox, const std::int64_t oy) {
          const auto index = static_cast<std::size_t>((oy * width) + ox);
          const auto owner = field_.owner(index);
          if (owner != NO_OWNER && owner != selfKey && owner < wires.size() &&
              onCutWay(owner, index)) {
            owners.insert(owner);
          }
        };
        for (std::uint32_t s = 1; s <= reach; ++s) {
          x += v.dx;
          y += v.dy;
          if (!inside(x, y)) {
            break;
          }
          const auto index = static_cast<std::size_t>((y * width) + x);
          if (scene_.components.test(index) && !bodies_.test(index)) {
            wall = true;
            steps = s;
            break;
          }
          noteOwner(x, y);
          if (diagonal) {
            // A way may pass between two diagonal cells without standing
            // on either; the two axis neighbours of the step catch it.
            noteOwner(x - v.dx, y);
            noteOwner(x, y - v.dy);
          }
        }
        if (!wall || owners.empty()) {
          continue;
        }
        const auto k = static_cast<std::uint32_t>(owners.size());
        const double length =
            static_cast<double>(steps) * (diagonal ? std::numbers::sqrt2 : 1.0);
        const double need = static_cast<double>(clearance) +
                            static_cast<double>(k) * static_cast<double>(pitch);
        const double shortfall = need - length;
        if (shortfall > worst) {
          worst = shortfall;
          std::vector<std::uint32_t> keys(owners.begin(), owners.end());
          std::ranges::sort(keys);
          found.from = cell;
          found.to = {.x = static_cast<std::uint32_t>(x),
                      .y = static_cast<std::uint32_t>(y),
                      .heading = cell.heading,
                      .primitive = 0};
          found.note = std::format(
              "squeezed at ({},{}): {} wire{} ({}) in {:.0f} cells to the "
              "artwork, need {:.0f} ({} + {} x {})",
              cell.x, cell.y, k, k == 1 ? "" : "s", namesOf(wires, keys),
              length, need, clearance, k, pitch);
        }
      }
    }
    return {worst, found};
  }

  /// The room every edge leaves beside it, measured at the end of the
  /// insertion (user, 2026-10-05): from every `squeezeStep`-th cell of the
  /// edge, past its two runs at the couplers, a straight line to either
  /// side at a right angle to the edge's heading, as far as the first cell
  /// of the artwork **inside the chip** — the wall of the channel: a qubit
  /// or a tunable coupler, never a launcher pad, which lies outside the
  /// rectangle the launcher cells span, and never a CPW coupler body the
  /// insertion put there (user, 2026-10-05: the room in question is the
  /// room between the feedlines and the circuit, not the room at the
  /// border). A line that leaves that rectangle measures nothing. Every
  /// distinct wire whose copper the line crosses is a wire that has to pass
  /// through that channel, and `k` of them need `clearance + k · pitch`
  /// cells: the feedline's own exclusion zone and one pitch each. A line
  /// that finds no wall within `squeezeReach`, or no wire, measures nothing.
  /// The worst line of an edge is its figure; an edge whose worst line is
  /// short of what its wires need is `Squeezed`, said at `-v 1` and kept in
  /// `squeezed_` for the artifact. This is the bottleneck the feedline pass
  /// fails in afterwards, read before it has run and without the repair's
  /// trials. Report only: no search refuses a way for it.
  void reportSqueeze(const std::vector<Wire>& wires) {
    squeezed_.clear();
    if (!squeezeReport()) {
      return;
    }
    std::vector<std::string> marked;
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.drawn) {
        continue;
      }
      const auto [worst, found] = measureSqueeze(
          wires, wire.way,
          static_cast<std::size_t>(startStubOf(wire, tuning_.straightStart) +
                                   BEND_RADIUS),
          static_cast<std::size_t>(wire.endStub + BEND_RADIUS), wire.key);
      if (worst > 0.0) {
        marked.push_back(std::format("f{} ({})", wire.slot, found.note));
        tell(std::format("[Coupler Insertion] squeeze chain {} edge {}->{} "
                         "(f{}): {}, short by {:.0f}, the line ends at ({},{})",
                         edge.chain, edge.from, edge.to, wire.slot, found.note,
                         worst, found.to.x, found.to.y));
        squeezed_[wire.key] = found;
      }
    }
    std::string list;
    for (const auto& one : marked) {
      list += (list.empty() ? "" : " · ") + one;
    }
    say(std::format("coupler insertion: SQUEEZE — {} of {} edges leave too "
                    "little room beside them for the wires that have to "
                    "pass (pitch {}, clearance {}, step {}, reach {}){}{}",
                    marked.size(), edges_.size(), roomPitch(),
                    tuning_.clearance, squeezeStep(), squeezeReach(),
                    marked.empty() ? "" : ": ", list));
  }

  /// What an obstacle of `reportBottlenecks` is.
  enum class WallKind : std::uint8_t {
    Border,
    Artwork,
    Coupler,
    Stub,
    Feedline
  };

  [[nodiscard]] static std::string_view kindName(const WallKind kind) {
    switch (kind) {
    case WallKind::Border:
      return "border";
    case WallKind::Artwork:
      return "artwork";
    case WallKind::Coupler:
      return "coupler pad";
    case WallKind::Stub:
      return "stub";
    case WallKind::Feedline:
      return "feedline";
    }
    return "?";
  }

  /// One obstacle of `reportBottlenecks`: what it is, how the log names it,
  /// and the wire it belongs to — the feedline edge, the resonator of a pad,
  /// the wire a stub or a lead is the end of — or NO_OWNER.
  struct Wall {
    WallKind kind = WallKind::Artwork;
    std::string name;
    std::uint32_t wire = NO_OWNER;
    /// A flank of a resonator's lead: the stub at its coupler.
    bool lead = false;
  };

  /// A port run of the bottleneck analysis: the middle line of the copper
  /// a wire has no choice about at one end, in fractional cells, from the
  /// port to the terminal — the end the free routing starts from.
  using PortRun = std::vector<std::pair<double, double>>;

  /// The run a wire leaves its source on. A plain wire's is the rule's
  /// straight length from the point the assignment feeds it at. A
  /// resonator's is the lead it already has — the coupling run and the
  /// quarter turn off its coupler, to where its search starts — with
  /// nothing added.
  [[nodiscard]] PortRun sourceRunOf(const Wire& wire) const {
    const auto& source = wire.objective.source;
    if (wire.couplerAtSource != NO_OWNER && !wire.arc.empty()) {
      PortRun run;
      for (const auto& point : wire.arc) {
        run.emplace_back(point.x, point.y);
      }
      if (!wire.arc.back().samePlace(source)) {
        run.emplace_back(source.x, source.y);
      }
      if (run.size() == 1) {
        run.push_back(run.front());
      }
      return run;
    }
    return straightRun(static_cast<double>(source.x),
                       static_cast<double>(source.y), source.heading);
  }

  /// The run a wire arrives at its target port on: the rule's straight
  /// length from the port itself. The router's target cell already lies
  /// past the port's band, that length out, so the run is measured from
  /// the port and not from that cell.
  [[nodiscard]] PortRun targetRunOf(const Wire& wire) const {
    const auto& target = wire.objective.target;
    const auto out = routing::reverse(target.heading);
    if (wire.targetPort < ports_.size()) {
      return straightRun(ports_[wire.targetPort].fx, ports_[wire.targetPort].fy,
                         out);
    }
    return straightRun(static_cast<double>(target.x),
                       static_cast<double>(target.y), out);
  }

  /// `straightLength` from a point on a heading.
  [[nodiscard]] PortRun straightRun(const double x, const double y,
                                    const Heading heading) const {
    const auto v = routing::headingVector(heading);
    const auto norm =
        std::hypot(static_cast<double>(v.dx), static_cast<double>(v.dy));
    const auto length = tuning_.straightLength;
    return {{x, y},
            {x + (static_cast<double>(v.dx) / norm * length),
             y + (static_cast<double>(v.dy) / norm * length)}};
  }

  /// The obstacles `reportBottlenecks` builds the medial axis over: the
  /// mask, the obstacle every blocked cell belongs to, and the lines of
  /// the stubs, leads and feedlines for the picture, which a mask one cell
  /// wide shows too thin to read. With them the slots of the port runs —
  /// the free cells between the two walls of each — and per outer wire the
  /// cell its terminal lies on at each end, source first: a cell of the
  /// slot.
  struct Walls {
    grid::BitGrid mask;
    std::vector<std::uint32_t> of;
    std::vector<Wall> list;
    std::vector<std::pair<std::uint32_t, std::vector<debug::Cell>>> strokes;
    std::vector<std::size_t> slots;
    std::unordered_map<std::uint32_t, std::array<std::size_t, 2>> terminals;
  };

  /// One bottleneck as `reportBottlenecks` measured it: its length in
  /// cells, how many wires it holds by the rule `reportSqueeze` uses, and
  /// which wires cross it on the ways they have now.
  struct MeasuredBottleneck {
    grid::Bottleneck gate;
    double cells = 0.0;
    std::uint32_t holds = 0;
    std::vector<std::uint32_t> crossing;
  };

  /// The obstacles of the chip as the insertion leaves it (user,
  /// 2026-10-07): everything outside the rectangle the launcher cells span
  /// and the outermost ring of cells (the border), the artwork inside that
  /// rectangle, the coupler pads, the port runs every outer wire has no
  /// choice about, and every feedline edge drawn. A port run
  /// (`sourceRunOf`, `targetRunOf`) is the rule's straight length from a
  /// plain wire's feed point or from a target port, and a resonator's lead
  /// as it stands. The port runs and the feedline edges are copper that
  /// keeps a clearance of its own, so each keeps half the clearance to
  /// either side (user, 2026-10-07); a gap between two of them has already
  /// given up a whole clearance. A port run is the two sides of a band with
  /// square ends, two walls one cell thick with a slot between them where
  /// its terminal lies (user, 2026-10-08); a feedline edge is inflated by a
  /// disc. The room a later obstacle keeps takes only free cells, a
  /// feedline's own line any cell. The ways of the other wires are
  /// no obstacle: they are what has to pass through. The feedline edge
  /// whose wire is `without` is left out.
  [[nodiscard]] Walls
  wallsAfterInsertion(const std::vector<Wire>& wires,
                      const std::uint32_t without = NO_OWNER) const {
    return wallsOf(wires, edges_, nullptr, without);
  }

  /// The obstacles of `wallsAfterInsertion` for a state of the insertion:
  /// the feedline edges `edges` (their ways in `wires`), and the pads of
  /// the couplers `placed` marks, every coupler when it is null.
  [[nodiscard]] Walls wallsOf(const std::vector<Wire>& wires,
                              const std::vector<Edge>& edges,
                              const std::vector<bool>* placed,
                              const std::uint32_t without) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    Walls walls{
        .mask = grid::BitGrid(scene_.router.width, scene_.router.height),
        .of = std::vector<std::uint32_t>(scene_.router.cells(), NO_OWNER),
        .list = {},
        .strokes = {}};
    const auto add = [&walls](const WallKind kind, std::string name,
                              const std::uint32_t wire) {
      walls.list.push_back(
          {.kind = kind, .name = std::move(name), .wire = wire});
      return static_cast<std::uint32_t>(walls.list.size() - 1);
    };
    const auto mark = [&](const std::int64_t x, const std::int64_t y,
                          const std::uint32_t wall) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      walls.mask.set(cell);
      walls.of[cell] = wall;
    };
    // A line through cells as an obstacle, inflated by `wallInflation`: the
    // line takes its cells whatever holds them, the disc around each cell
    // only the free ones.
    const auto disc = inflationStencil();
    const auto markThick = [&](const std::vector<debug::Cell>& points,
                               const std::uint32_t wall) {
      std::vector<std::size_t> line;
      for (std::size_t at = 0; at < points.size(); ++at) {
        const auto [x, y] = points[at];
        const auto [px, py] = at == 0 ? points[at] : points[at - 1];
        for (const auto cell : grid::lineCells(
                 px, py, x, y, scene_.router.width, scene_.router.height)) {
          line.push_back(cell);
        }
      }
      for (const auto cell : line) {
        const auto cx = static_cast<std::int64_t>(cell) % width;
        const auto cy = static_cast<std::int64_t>(cell) / width;
        for (const auto& [dx, dy] : disc) {
          const auto x = cx + dx;
          const auto y = cy + dy;
          if (x >= 0 && y >= 0 && x < width && y < height &&
              walls.of[static_cast<std::size_t>((y * width) + x)] == NO_OWNER) {
            mark(x, y, wall);
          }
        }
      }
      for (const auto cell : line) {
        walls.mask.set(cell);
        walls.of[cell] = wall;
      }
      walls.strokes.emplace_back(wall, points);
    };

    std::int64_t inMinX = 0;
    std::int64_t inMinY = 0;
    std::int64_t inMaxX = width - 1;
    std::int64_t inMaxY = height - 1;
    if (!scene_.launcherCell.empty()) {
      inMinX = width;
      inMinY = height;
      inMaxX = -1;
      inMaxY = -1;
      for (const auto& [port, slot] : scene_.launcherCell) {
        const auto place = scene_.router.cell(slot);
        inMinX = std::min<std::int64_t>(inMinX, place.x());
        inMaxX = std::max<std::int64_t>(inMaxX, place.x());
        inMinY = std::min<std::int64_t>(inMinY, place.y());
        inMaxY = std::max<std::int64_t>(inMaxY, place.y());
      }
    }
    const auto border = add(WallKind::Border, "border", NO_OWNER);
    for (std::int64_t y = 0; y < height; ++y) {
      for (std::int64_t x = 0; x < width; ++x) {
        if (x <= 0 || y <= 0 || x >= width - 1 || y >= height - 1 ||
            x < inMinX || x > inMaxX || y < inMinY || y > inMaxY) {
          mark(x, y, border);
        }
      }
    }

    // The artwork inside the launcher cells is one obstacle: the qubits and
    // the couplers between them touch. `endName` names a cell of it by the
    // nearest port.
    const auto artwork = add(WallKind::Artwork, "artwork", NO_OWNER);
    for (std::size_t cell = 0; cell < walls.of.size(); ++cell) {
      if (scene_.components.test(cell) && walls.of[cell] == NO_OWNER) {
        walls.mask.set(cell);
        walls.of[cell] = artwork;
      }
    }

    for (std::size_t index = 0; index < couplers_.size(); ++index) {
      const auto& coupler = couplers_[index];
      if (coupler.chosen >= coupler.options.size() ||
          coupler.wire >= wires.size() ||
          (placed != nullptr && !(*placed)[index])) {
        continue;
      }
      const auto pad = add(
          WallKind::Coupler,
          std::format("pad of {}", wireId(wires[coupler.wire])), coupler.wire);
      for (const auto cell : coupler.options[coupler.chosen].body) {
        walls.mask.set(cell);
        walls.of[cell] = pad;
      }
    }

    // The port runs (user, 2026-10-08). A run keeps the band of half the
    // clearance to either side of its middle line, with square ends: a
    // cell whose nearest point on the line is one of its ends from beyond
    // is not in it. The run is the edge of that band and nothing more: two
    // walls one cell thick, left and right of the line, made of the cells
    // of the band with a neighbour, diagonal ones included, outside it to
    // the side or beyond the port end. So each wall is 4-connected and lies
    // where the band's edge lies, and a gap between two runs keeps its
    // length. The end at the port is closed because the port's own copper
    // lies there; open, the space between the walls would join the room
    // behind the run through the cell or two between the band and the port
    // (on 17q a row of cells between every launcher run and the border).
    // The end at the terminal is open. The cells between the two walls are
    // free and are the slot the terminal lies in: the medial axis runs into
    // it, and no bottleneck crosses it. A wall takes only free cells, and a
    // run of no length keeps no room.
    const auto half = wallInflation();
    const auto markRun = [&](const PortRun& run, const std::string& name,
                             const std::uint32_t wire, const bool lead) {
      const auto left = add(WallKind::Stub, name + ", left", wire);
      const auto right = add(WallKind::Stub, name + ", right", wire);
      walls.list[left].lead = lead;
      walls.list[right].lead = lead;
      double total = 0.0;
      double x0 = run.front().first;
      double x1 = x0;
      double y0 = run.front().second;
      double y1 = y0;
      for (std::size_t k = 0; k < run.size(); ++k) {
        const auto [px, py] = run[k];
        if (k > 0) {
          total += std::hypot(px - run[k - 1].first, py - run[k - 1].second);
        }
        x0 = std::min(x0, px);
        x1 = std::max(x1, px);
        y0 = std::min(y0, py);
        y1 = std::max(y1, py);
      }
      // Where each cell of a box around the band lies. The box reaches a
      // cell past the band on every side, so every neighbour of a band cell
      // is in it; it is not clipped to the grid.
      enum class Place : std::uint8_t {
        Aside,
        BeyondPort,
        BeyondEnd,
        Left,
        Right
      };
      const auto reach = half + 1.0;
      const auto fromX = static_cast<std::int64_t>(std::floor(x0 - reach));
      const auto fromY = static_cast<std::int64_t>(std::floor(y0 - reach));
      const auto boxWidth =
          static_cast<std::int64_t>(std::ceil(x1 + reach)) - fromX + 1;
      const auto boxHeight =
          static_cast<std::int64_t>(std::ceil(y1 + reach)) - fromY + 1;
      std::vector<Place> places(static_cast<std::size_t>(boxWidth * boxHeight),
                                Place::Aside);
      const auto placeAt = [&](const std::int64_t x, const std::int64_t y) {
        return places[static_cast<std::size_t>(((y - fromY) * boxWidth) +
                                               (x - fromX))];
      };
      for (std::size_t at = 0; total > 0.0 && at < places.size(); ++at) {
        const auto cx = static_cast<double>(
            fromX + (static_cast<std::int64_t>(at) % boxWidth));
        const auto cy = static_cast<double>(
            fromY + (static_cast<std::int64_t>(at) / boxWidth));
        // The nearest point of the line: how far, on which side, and
        // whether it is an end seen from beyond.
        double best = std::numeric_limits<double>::max();
        double side = 0.0;
        auto beyond = Place::Aside;
        for (std::size_t k = 0; k + 1 < run.size(); ++k) {
          const auto ax = run[k].first;
          const auto ay = run[k].second;
          const auto dx = run[k + 1].first - ax;
          const auto dy = run[k + 1].second - ay;
          const auto squared = (dx * dx) + (dy * dy);
          const auto raw = squared > 0.0
                               ? (((cx - ax) * dx) + ((cy - ay) * dy)) / squared
                               : 0.0;
          const auto t = std::clamp(raw, 0.0, 1.0);
          const auto distance =
              std::hypot(cx - (ax + (t * dx)), cy - (ay + (t * dy)));
          if (distance < best) {
            best = distance;
            side = (dx * (cy - ay)) - (dy * (cx - ax));
            beyond = k == 0 && raw < 0.0                ? Place::BeyondPort
                     : k + 2 == run.size() && raw > 1.0 ? Place::BeyondEnd
                                                        : Place::Aside;
          }
        }
        if (beyond != Place::Aside) {
          places[at] = beyond;
        } else if (best <= half) {
          places[at] = side < 0.0 ? Place::Right : Place::Left;
        }
      }
      for (std::int64_t y = fromY + 1; y + 1 < fromY + boxHeight; ++y) {
        for (std::int64_t x = fromX + 1; x + 1 < fromX + boxWidth; ++x) {
          const auto place = placeAt(x, y);
          if ((place != Place::Left && place != Place::Right) || x < 0 ||
              y < 0 || x >= width || y >= height) {
            continue;
          }
          const auto cell = static_cast<std::size_t>((y * width) + x);
          if (walls.of[cell] != NO_OWNER) {
            continue;
          }
          bool edge = false;
          for (std::int64_t dy = -1; dy <= 1 && !edge; ++dy) {
            for (std::int64_t dx = -1; dx <= 1 && !edge; ++dx) {
              const auto next = placeAt(x + dx, y + dy);
              edge = next == Place::Aside || next == Place::BeyondPort;
            }
          }
          if (edge) {
            mark(x, y, place == Place::Right ? right : left);
          } else {
            walls.slots.push_back(cell);
          }
        }
      }
      std::vector<debug::Cell> line;
      for (const auto& [px, py] : run) {
        line.emplace_back(std::lround(px), std::lround(py));
      }
      walls.strokes.emplace_back(left, std::move(line));
      // The terminal: on the line, a cell short of its end, in the slot
      // between the two walls.
      const auto& end = run.back();
      const auto& before = run[run.size() - 2];
      const auto length =
          std::max(1.0e-9, std::hypot(end.first - before.first,
                                      end.second - before.second));
      const auto tx = std::clamp<std::int64_t>(
          std::lround(end.first - ((end.first - before.first) / length)), 0,
          width - 1);
      const auto ty = std::clamp<std::int64_t>(
          std::lround(end.second - ((end.second - before.second) / length)), 0,
          height - 1);
      return static_cast<std::size_t>((ty * width) + tx);
    };
    for (const auto& wire : wires) {
      if (!wire.feasible || wire.inner || wire.feedline) {
        continue;
      }
      const auto resonator =
          wire.couplerAtSource != NO_OWNER && !wire.arc.empty();
      const auto source =
          markRun(sourceRunOf(wire),
                  resonator ? std::format("lead of {}", wireId(wire))
                            : std::format("{} at its source", wireId(wire)),
                  wire.key, resonator);
      const auto target =
          markRun(targetRunOf(wire),
                  std::format("{} at its target", wireId(wire)), wire.key,
                  false);
      walls.terminals.emplace(wire.key, std::array{source, target});
    }

    for (const auto& edge : edges) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.drawn || wire.way.empty() || edge.wire == without) {
        continue;
      }
      markThick(cellsOf(wire.way),
                add(WallKind::Feedline, wireId(wire), edge.wire));
    }
    // A feedline's own line takes any cell, so a slot keeps only the cells
    // still free.
    std::erase_if(walls.slots, [&walls](const std::size_t cell) {
      return walls.mask.test(cell);
    });
    return walls;
  }

  /// How the log names the obstacle at one end of a bottleneck: by its
  /// wall, and a cell of the artwork by the nearest port that is not a
  /// launcher, since the artwork is one piece.
  [[nodiscard]] std::string endName(const Walls& walls,
                                    const std::size_t cell) const {
    const auto& wall = walls.list[walls.of[cell]];
    if (wall.kind != WallKind::Artwork) {
      return wall.name;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto x = static_cast<std::int64_t>(cell) % width;
    const auto y = static_cast<std::int64_t>(cell) / width;
    const PortMark* nearest = nullptr;
    std::int64_t best = std::numeric_limits<std::int64_t>::max();
    for (const auto& port : ports_) {
      if (scene_.launcherCell.contains(port.index)) {
        continue;
      }
      const auto dx = port.x - x;
      const auto dy = port.y - y;
      const auto squared = (dx * dx) + (dy * dy);
      if (squared < best) {
        best = squared;
        nearest = &port;
      }
    }
    return nearest == nullptr ? wall.name
                              : std::format("artwork near {}", nearest->label);
  }

  /// Whether the capacity graph keeps every bottleneck: `SCPD_BOTTLENECK_ALL`,
  /// off. Off keeps only those `couplerCuts` keeps (user, 2026-10-08).
  [[nodiscard]] static bool bottleneckAll() {
    return envFlag("SCPD_BOTTLENECK_ALL", false);
  }
  /// Whether the capacity graph is checked by the integer flow of
  /// `checkCapacity`: `SCPD_CAPACITY_FLOW`, off. Off routes the wires one
  /// after another with `checkInTurn` (user, 2026-10-08).
  [[nodiscard]] static bool capacityFlow() {
    return envFlag("SCPD_CAPACITY_FLOW", false);
  }

  /// The bottlenecks the coupler insertion has a hand in (user,
  /// 2026-10-08): those with an end on a feedline edge, on a coupler pad or
  /// on the lead a resonator leaves its coupler on. A bottleneck between
  /// other obstacles — the artwork, the border, the port runs of the plain
  /// wires — is there whatever the insertion does, and is left out, so the
  /// chambers on either side of it are one. Every bottleneck is kept under
  /// `bottleneckAll`. No bottleneck joins the two walls of one port run:
  /// its line would cross the slot between them, and `grid::findBottlenecks`
  /// refuses every line that crosses a slot.
  [[nodiscard]] static std::vector<grid::Bottleneck>
  couplerCuts(const std::vector<Wall>& list,
              const std::span<const std::uint32_t> wallOf,
              std::vector<grid::Bottleneck> gates) {
    if (bottleneckAll()) {
      return gates;
    }
    const auto counts = [&](const std::size_t cell) {
      const auto& wall = list[wallOf[cell]];
      return wall.kind == WallKind::Feedline ||
             wall.kind == WallKind::Coupler || wall.lead;
    };
    std::erase_if(gates, [&](const grid::Bottleneck& gate) {
      return !counts(gate.first) && !counts(gate.second);
    });
    return gates;
  }

  /// The options `reportBottlenecks` searches the bottlenecks with: gaps up
  /// to `bottleneckWires` wires, one narrowing per wire pitch, the rise of
  /// `bottleneckRise`, and the slots and walls of `wallsAfterInsertion`.
  [[nodiscard]] grid::BottleneckOptions
  bottleneckOptionsOf(const std::span<const std::size_t> slots,
                      const std::span<const std::uint32_t> wallOf) const {
    const auto pitch = static_cast<double>(roomPitch());
    const auto clearance = static_cast<double>(tuning_.clearance);
    const auto half = static_cast<double>(bottleneckWires()) * clearance / 2.0;
    const auto step =
        std::min(scene_.router.cellWidth, scene_.router.cellHeight);
    return {.maximumSquaredClearance =
                static_cast<std::uint32_t>(std::floor(half * half)) + 1,
            .sameNarrowing = pitch * step,
            .minimumRise = bottleneckRise(),
            .slots = slots,
            .wallOf = wallOf};
  }

  /// The bottlenecks of the chip the insertion leaves behind (user,
  /// 2026-10-07), report only. The medial axis is built over the obstacles
  /// of `wallsAfterInsertion` — a Voronoi diagram over their boundary
  /// cells, as the capacity stage builds it — and `grid::findBottlenecks`
  /// finds the cells of the axis where the clearance has a local minimum
  /// or the edge of a plateau of one, and traces each of them down to the
  /// two obstacle cells across it. Every such line is a bottleneck, and the
  /// two obstacles at its ends say what it lies between: a feedline and a
  /// launcher stub, a feedline and a qubit, two feedlines.
  ///
  /// Only gaps up to what `bottleneckWires` wires need are reported, and a
  /// local minimum counts only where the clearance rises by
  /// `bottleneckRise` cells on both sides: the steps of a raster wall are
  /// minima a cell deep. Per bottleneck the length gives how many wires it
  /// holds — `wiresThroughGap`, the length between the inflated walls over
  /// the wire clearance — and the ways the wires have now give which cross
  /// it.
  /// Those ways are the Detail stage's or the insertion's and the feedline
  /// pass moves them, so the count says where to look, not what will fail.
  void reportBottlenecks(const std::vector<Wire>& wires) {
    if (!bottleneckReport()) {
      return;
    }
    analyseCapacity(wires);
    benchCapacityCheck(wires);
  }

  /// The analysis of `reportBottlenecks`, whatever `SCPD_BOTTLENECKS`
  /// says: the bottlenecks, the capacity graph and its check, with their
  /// lines, pictures and data.
  void analyseCapacity(const std::vector<Wire>& wires) {
    const auto began = std::chrono::steady_clock::now();
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    // The time of every step, for the line that says what the analysis
    // cost.
    auto lapStart = began;
    std::string laps;
    const auto lap = [&](const std::string_view what) {
      const auto now = std::chrono::steady_clock::now();
      laps +=
          std::format("{}{} {:.3f}", laps.empty() ? "" : ", ", what,
                      std::chrono::duration<double>(now - lapStart).count());
      lapStart = now;
    };
    const auto walls = wallsAfterInsertion(wires);
    lap("walls");
    const auto distance = grid::squaredDistanceTransform(walls.mask);
    lap("distance");
    const auto edges = grid::medialAxis(walls.mask);
    lap("voronoi");
    const auto axis =
        grid::rasterizeMedialAxis(walls.mask, scene_.router, edges);
    lap("axis");
    const auto clearance = static_cast<double>(tuning_.clearance);
    const auto widest = static_cast<double>(bottleneckWires()) * clearance;
    const auto found =
        grid::findBottlenecks(walls.mask, axis, distance, scene_.router,
                              bottleneckOptionsOf(walls.slots, walls.of));
    const auto gates = couplerCuts(walls.list, walls.of, found);
    lap("saddles");

    std::vector<MeasuredBottleneck> measured;
    measured.reserve(gates.size());
    std::map<std::string, std::uint32_t> pairs;
    std::uint32_t overfull = 0;
    for (const auto& gate : gates) {
      MeasuredBottleneck one{.gate = gate};
      const auto x0 = static_cast<std::int64_t>(gate.first) % width;
      const auto y0 = static_cast<std::int64_t>(gate.first) / width;
      const auto x1 = static_cast<std::int64_t>(gate.second) % width;
      const auto y1 = static_cast<std::int64_t>(gate.second) / width;
      one.cells = std::hypot(static_cast<double>(x1 - x0),
                             static_cast<double>(y1 - y0));
      const auto& a = walls.list[walls.of[gate.first]];
      const auto& b = walls.list[walls.of[gate.second]];
      one.holds = wiresThroughGap(one.cells, clearance);

      // The wires on the line between the two ends, the ends' own wires
      // aside. A way may pass between two cells of the line that touch at a
      // corner, so a diagonal step looks at the two cells beside it too.
      std::set<std::uint32_t> owners;
      const auto note = [&](const std::size_t cell) {
        const auto owner = field_.owner(cell);
        if (owner < wires.size() && !wires[owner].feedline && owner != a.wire &&
            owner != b.wire) {
          owners.insert(owner);
        }
      };
      const auto line = grid::lineCells(x0, y0, x1, y1, scene_.router.width,
                                        scene_.router.height);
      for (std::size_t at = 1; at + 1 < line.size(); ++at) {
        note(line[at]);
        const auto px = static_cast<std::int64_t>(line[at - 1]) % width;
        const auto py = static_cast<std::int64_t>(line[at - 1]) / width;
        const auto cx = static_cast<std::int64_t>(line[at]) % width;
        const auto cy = static_cast<std::int64_t>(line[at]) / width;
        if (px != cx && py != cy) {
          note(static_cast<std::size_t>((py * width) + cx));
          note(static_cast<std::size_t>((cy * width) + px));
        }
      }
      one.crossing.assign(owners.begin(), owners.end());
      if (one.crossing.size() > one.holds) {
        ++overfull;
      }
      auto low = kindName(a.kind);
      auto high = kindName(b.kind);
      if (high < low) {
        std::swap(low, high);
      }
      ++pairs[std::format("{}–{}", low, high)];
      measured.push_back(std::move(one));
    }

    // Every bottleneck alike, the tightest first: the fewest wires to
    // spare.
    std::vector<std::size_t> order(measured.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    const auto spare = [&measured](const std::size_t at) {
      return static_cast<std::int64_t>(measured[at].holds) -
             static_cast<std::int64_t>(measured[at].crossing.size());
    };
    std::ranges::stable_sort(
        order, [&](const std::size_t left, const std::size_t right) {
          return spare(left) != spare(right)
                     ? spare(left) < spare(right)
                     : measured[left].cells < measured[right].cells;
        });
    for (const auto at : order) {
      const auto& one = measured[at];
      tell(std::format(
          "[Bottleneck] {} · {}: {:.1f} cells from ({},{}) to ({},{}), holds "
          "{}, "
          "crossed now by {} ({})",
          endName(walls, one.gate.first), endName(walls, one.gate.second),
          one.cells, static_cast<std::int64_t>(one.gate.first) % width,
          static_cast<std::int64_t>(one.gate.first) / width,
          static_cast<std::int64_t>(one.gate.second) % width,
          static_cast<std::int64_t>(one.gate.second) / width, one.holds,
          one.crossing.size(), namesOf(wires, one.crossing)));
    }

    std::string paired;
    for (const auto& [pair, count] : pairs) {
      paired +=
          std::format("{}{} {}", paired.empty() ? "" : " · ", pair, count);
    }
    std::array<std::uint32_t, 5> kinds{};
    for (const auto& wall : walls.list) {
      ++kinds[static_cast<std::size_t>(wall.kind)];
    }
    std::size_t axisCells = 0;
    for (const auto cell : axis.cells) {
      axisCells += cell == grid::AxisCell::None ? 0 : 1;
    }
    say(std::format(
        "coupler insertion: BOTTLENECKS — {} in all ({}){}; {} "
        "of them hold fewer wires than cross them now; walls: {} feedline "
        "edges, {} coupler pads, {} stubs and leads, the artwork, the border; "
        "axis {} cells; gaps up to {} wires ({:.0f} cells = {} x {:.0f}), "
        "a gap holding its length over the clearance, port runs walled "
        "and feedlines inflated {:.1f} cells out; rise {:.1f} cells; {:.2f}s "
        "({})",
        measured.size(), paired.empty() ? "-" : paired,
        bottleneckAll()
            ? std::string()
            : std::format(", {} more left out with no end on a feedline "
                          "edge, a coupler pad or a resonator lead",
                          found.size() - gates.size()),
        overfull,
        kinds[static_cast<std::size_t>(WallKind::Feedline)],
        kinds[static_cast<std::size_t>(WallKind::Coupler)],
        kinds[static_cast<std::size_t>(WallKind::Stub)], axisCells,
        bottleneckWires(), widest, bottleneckWires(), clearance,
        wallInflation(), bottleneckRise(),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count(),
        (lap("measure"), laps)));
    if (!graphOnly_) {
      drawBottlenecks(wires, walls, axis, measured);
    }
    reportCapacityGraph(wires, walls, gates, measured);
  }

  /// One edge of the capacity graph of `reportCapacityGraph`: a bottleneck
  /// between the chambers beside it, or a stretch of a feedline edge that a
  /// wire may cross between the chambers on its two sides.
  struct GraphEdge {
    bool crossing = false;
    /// The bottleneck by its place in the measured list, or the feedline
    /// edge by its wire key.
    std::uint32_t id = 0;
    std::vector<std::uint32_t> chambers;
    /// How many wires it takes: the bottleneck's `holds`, or how many
    /// crossings a wire pitch apart the stretch has room for.
    std::uint32_t capacity = 0;
    /// The wires that may use it. Every wire may pass a bottleneck; a
    /// stretch of an edge between couplers only the wires that bridge it.
    std::optional<std::vector<std::uint32_t>> users;
    /// A stretch: its first and last cell on the edge's way, and its length
    /// in cells.
    std::size_t first = 0;
    std::size_t last = 0;
    double length = 0.0;
    /// The wires whose way through the graph takes it.
    std::vector<std::uint32_t> through;
  };

  /// One outer wire of the capacity graph: the cells its two ends leave
  /// from, as `portsOf` gives them, the chambers each lies in, and its
  /// demand in the flow; none when a port lies in no chamber.
  struct GraphWire {
    std::uint32_t wire = 0;
    std::array<PathPoint, 2> ends{};
    std::vector<std::uint32_t> from;
    std::vector<std::uint32_t> to;
    std::optional<std::uint32_t> demand;
    /// The feedline edges it is prescribed to cross, by their wire key.
    std::vector<std::uint32_t> crosses;
  };

  /// The stretches of every drawn feedline edge where a wire may cross it,
  /// with the chambers on either side, by the rule the feedline pass holds
  /// a wire to (user, 2026-10-07). The rule is built as
  /// `rebuildCrossingRule` builds it — the same edges, `CROSSING_REACH` —
  /// and a cell of an edge is a place to cross when the straight line
  /// across it at a right angle, `CROSSING_REACH + 1` cells to each side,
  /// is one `CrossingConstraints::allowed` lets a wire run and runs into no
  /// obstacle but the edge itself. So a crossing keeps clear of the edge's
  /// bends and of its first and last ten cells, as the search does.
  ///
  /// Consecutive places with the same two chambers are one stretch, and a
  /// stretch of `length` cells takes `length / pitch + 1` crossings: two
  /// wires crossing side by side keep the wire spacing. A plain wire crosses
  /// the one edge it is prescribed and no other (user, 2026-10-07): an edge
  /// between couplers is open only to the plain wires that bridge it
  /// (`bridgers_`, which `checkBridgers` holds equal to `assignBridges`). No
  /// wire is prescribed a terminal edge, so a terminal edge has no stretch,
  /// and no resonator crosses a feedline at all.
  [[nodiscard]] std::vector<GraphEdge>
  crossingStretches(const std::vector<Wire>& wires,
                    const std::vector<Edge>& edges, const Walls& walls,
                    const grid::Chambers& chambers) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto pitch = static_cast<double>(roomPitch());
    std::vector<Path> crossable;
    for (const auto& edge : edges) {
      const auto& wire = wires[edge.wire];
      if (wire.drawn && !(edge.terminal && feedlineLikePrototype())) {
        crossable.push_back(wire.way);
      }
    }
    routing::CrossingConstraints rule;
    if (orthoCrossing()) {
      rule.build(scene_.router.width, scene_.router.height, crossable, {},
                 CROSSING_REACH);
    }
    // The obstacle every drawn edge is, to let a crossing line run over its
    // own edge.
    std::unordered_map<std::uint32_t, std::uint32_t> wallOfEdge;
    for (std::uint32_t wall = 0; wall < walls.list.size(); ++wall) {
      if (walls.list[wall].kind == WallKind::Feedline) {
        wallOfEdge.emplace(walls.list[wall].wire, wall);
      }
    }
    const auto reach = static_cast<std::int64_t>(CROSSING_REACH) + 1;
    // How far past the band a side may still be the edge's own inflation.
    const auto beyond =
        static_cast<std::int64_t>(std::ceil(wallInflation())) + 1;

    std::vector<GraphEdge> stretches;
    for (const auto& edge : edges) {
      const auto& wire = wires[edge.wire];
      const auto own = wallOfEdge.find(edge.wire);
      if (edge.terminal || !wire.drawn || wire.way.empty() ||
          own == wallOfEdge.end()) {
        continue;
      }
      const std::optional<std::vector<std::uint32_t>> users =
          bridgers_[edge.chain][edge.from];
      const auto straight = routing::straightCells(wire.way);

      // The two chambers a crossing at a cell joins, or nothing where no
      // wire may cross there.
      const auto sidesAt = [&](const std::size_t at)
          -> std::optional<std::pair<std::uint32_t, std::uint32_t>> {
        const auto& point = wire.way[at];
        if (orthoCrossing() && !straight[at]) {
          return std::nullopt;
        }
        const auto across = routing::turned(point.heading, 2);
        const auto v = routing::headingVector(across);
        for (std::int64_t t = -reach; t <= reach; ++t) {
          const auto x = static_cast<std::int64_t>(point.x) + (t * v.dx);
          const auto y = static_cast<std::int64_t>(point.y) + (t * v.dy);
          if (x < 0 || y < 0 || x >= width || y >= height) {
            return std::nullopt;
          }
          const auto cell = static_cast<std::size_t>((y * width) + x);
          if (walls.mask.test(cell) && walls.of[cell] != own->second) {
            return std::nullopt;
          }
          if (!rule.allowed(static_cast<std::uint32_t>(x),
                            static_cast<std::uint32_t>(y), across)) {
            return std::nullopt;
          }
        }
        const auto chamberAt = [&](const std::int64_t side) {
          for (std::int64_t t = reach; t <= reach + beyond; ++t) {
            const auto x =
                static_cast<std::int64_t>(point.x) + (side * t * v.dx);
            const auto y =
                static_cast<std::int64_t>(point.y) + (side * t * v.dy);
            if (x < 0 || y < 0 || x >= width || y >= height) {
              break;
            }
            const auto cell = static_cast<std::size_t>((y * width) + x);
            if (chambers.of[cell] != grid::NO_CHAMBER) {
              return chambers.of[cell];
            }
            if (walls.mask.test(cell) && walls.of[cell] != own->second) {
              break;
            }
          }
          return grid::NO_CHAMBER;
        };
        const auto left = chamberAt(1);
        const auto right = chamberAt(-1);
        if (left == grid::NO_CHAMBER || right == grid::NO_CHAMBER ||
            left == right) {
          return std::nullopt;
        }
        return std::pair{std::min(left, right), std::max(left, right)};
      };

      std::optional<std::pair<std::uint32_t, std::uint32_t>> open;
      std::size_t first = 0;
      const auto close = [&](const std::size_t last) {
        double length = 0.0;
        for (std::size_t at = first + 1; at <= last; ++at) {
          length += std::hypot(
              static_cast<double>(wire.way[at].x) - wire.way[at - 1].x,
              static_cast<double>(wire.way[at].y) - wire.way[at - 1].y);
        }
        stretches.push_back(
            {.crossing = true,
             .id = edge.wire,
             .chambers = {open->first, open->second},
             .capacity =
                 static_cast<std::uint32_t>(std::floor(length / pitch)) + 1,
             .users = users,
             .first = first,
             .last = last,
             .length = length,
             .through = {}});
      };
      for (std::size_t at = 0; at < wire.way.size(); ++at) {
        const auto sides = sidesAt(at);
        if (open.has_value() && sides != open) {
          close(at - 1);
          open.reset();
        }
        if (sides.has_value() && !open.has_value()) {
          open = sides;
          first = at;
        }
      }
      if (open.has_value()) {
        close(wire.way.size() - 1);
      }
    }
    return stretches;
  }

  /// The cells a wire's terminals lie on, source first — cells of the slots
  /// between the walls of its port runs — each with the way on from there.
  [[nodiscard]] std::array<std::pair<PathPoint, routing::HeadingVector>, 2>
  portsOf(const Walls& walls, const Wire& wire) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto found = walls.terminals.find(wire.key);
    const auto onward = [](const PortRun& run) {
      const auto& end = run.back();
      const auto& before = run[run.size() - 2];
      return routing::HeadingVector{
          .dx =
              static_cast<std::int8_t>((end.first > before.first + 0.5)   ? 1
                                       : (end.first < before.first - 0.5) ? -1
                                                                          : 0),
          .dy = static_cast<std::int8_t>((end.second > before.second + 0.5) ? 1
                                         : (end.second < before.second - 0.5)
                                             ? -1
                                             : 0)};
    };
    const auto pointOf = [width](const std::size_t cell) {
      return PathPoint{.x = static_cast<std::uint32_t>(
                           static_cast<std::int64_t>(cell) % width),
                       .y = static_cast<std::uint32_t>(
                           static_cast<std::int64_t>(cell) / width),
                       .heading = 0,
                       .primitive = 0};
    };
    if (found == walls.terminals.end()) {
      return {std::pair{wire.objective.source, routing::HeadingVector{}},
              std::pair{wire.objective.target, routing::HeadingVector{}}};
    }
    return {std::pair{pointOf(found->second[0]), onward(sourceRunOf(wire))},
            std::pair{pointOf(found->second[1]), onward(targetRunOf(wire))}};
  }

  /// The chambers a terminal lies in. A terminal lies in the slot between
  /// the two walls of its port run, and the slot in one chamber: the room
  /// in front of the run's open end, which the bottlenecks from the ends of
  /// the two walls close off (user, 2026-10-07). A terminal on the line of a bottleneck lies in every
  /// chamber beside it; one on an obstacle is taken on along its way, and
  /// then to the nearest cell around it, a few cells at most; no chamber
  /// when there is none.
  [[nodiscard]] std::vector<std::uint32_t>
  chambersOfPort(const grid::Chambers& chambers, const Walls& walls,
                 const PathPoint& port,
                 const routing::HeadingVector& on) const {
    // Past the inflation of the port run, and a few cells more.
    const auto LOOK = static_cast<std::int64_t>(std::ceil(wallInflation())) + 6;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto inside = [&](const std::int64_t x, const std::int64_t y) {
      return x >= 0 && y >= 0 && x < width && y < height;
    };
    const auto cellAt = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    // The chambers of a free cell: its own, or those of the cells around it
    // when it lies on a line.
    const auto of = [&](const std::int64_t x, const std::int64_t y) {
      std::vector<std::uint32_t> found;
      if (!inside(x, y) || walls.mask.test(cellAt(x, y))) {
        return found;
      }
      if (const auto own = chambers.of[cellAt(x, y)]; own != grid::NO_CHAMBER) {
        found.push_back(own);
        return found;
      }
      for (std::size_t step = 0; step < grid::STEP_DX.size(); ++step) {
        const auto nx = x + grid::STEP_DX[step];
        const auto ny = y + grid::STEP_DY[step];
        if (inside(nx, ny) && chambers.of[cellAt(nx, ny)] != grid::NO_CHAMBER) {
          found.push_back(chambers.of[cellAt(nx, ny)]);
        }
      }
      std::ranges::sort(found);
      const auto repeated = std::ranges::unique(found);
      found.erase(repeated.begin(), repeated.end());
      return found;
    };
    const auto px = static_cast<std::int64_t>(port.x);
    const auto py = static_cast<std::int64_t>(port.y);
    for (std::int64_t k = 0; k <= LOOK; ++k) {
      if (auto found = of(px + (k * on.dx), py + (k * on.dy)); !found.empty()) {
        return found;
      }
    }
    for (std::int64_t ring = 1; ring <= LOOK; ++ring) {
      for (std::int64_t dy = -ring; dy <= ring; ++dy) {
        for (std::int64_t dx = -ring; dx <= ring; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != ring) {
            continue;
          }
          if (auto found = of(px + dx, py + dy); !found.empty()) {
            return found;
          }
        }
      }
    }
    return {};
  }

  /// The flow problem of `reportCapacityGraph`: the graph's edges, the
  /// demands and what the report and the picture need of both.
  struct CapacityProblem {
    /// The bottleneck edges first, then the stretches.
    std::vector<GraphEdge> graph;
    std::size_t gateEdges = 0;
    /// The bottlenecks with one chamber on both sides.
    std::uint32_t roundabout = 0;
    std::vector<FlowEdge> flowEdges;
    std::vector<FlowDemand> demands;
    /// The wire of every demand.
    std::vector<std::uint32_t> wireOf;
    /// The outer wires with a terminal in no chamber.
    std::vector<std::uint32_t> walled;
    std::uint32_t outer = 0;
    std::vector<std::uint32_t> portsIn;
    std::vector<std::pair<PathPoint, std::uint32_t>> portMarks;
    std::vector<GraphWire> outerWires;
  };

  /// How long a resonator's way through the capacity graph may be, in
  /// cells (user, 2026-10-09): the target length less the run to the port
  /// and less the lead it already has, with the tolerance. Nothing for a
  /// plain wire, which may be any length.
  [[nodiscard]] std::optional<double> longestWayOf(const Wire& wire) const {
    if (!wire.resonator) {
      return std::nullopt;
    }
    return std::max(0.0, tuning_.targetLength - wire.anchorGap -
                             (wire.arc.empty() ? 0.0 : lengthOf(wire.arc)) +
                             tuning_.lengthTolerance);
  }

  /// The edges and demands of the capacity graph over the chambers the
  /// bottlenecks cut: see `reportCapacityGraph`.
  [[nodiscard]] CapacityProblem
  capacityProblemOf(const std::vector<Wire>& wires,
                    const std::vector<Edge>& edges, const Walls& walls,
                    const std::vector<MeasuredBottleneck>& measured,
                    const grid::Chambers& chambers) const {
    CapacityProblem problem;
    auto& graph = problem.graph;
    for (std::uint32_t gate = 0; gate < measured.size(); ++gate) {
      if (chambers.beside[gate].size() < 2) {
        ++problem.roundabout;
        continue;
      }
      graph.push_back({.crossing = false,
                       .id = gate,
                       .chambers = chambers.beside[gate],
                       .capacity = measured[gate].holds,
                       .users = std::nullopt,
                       .first = 0,
                       .last = 0,
                       .length = measured[gate].cells,
                       .through = {}});
    }
    problem.gateEdges = graph.size();
    for (auto& stretch : crossingStretches(wires, edges, walls, chambers)) {
      graph.push_back(std::move(stretch));
    }

    // The feedline edges each plain wire is prescribed to cross (user,
    // 2026-10-08): every drawn edge between two couplers that it bridges
    // (`bridgers_`). Its way takes exactly one stretch of each and no other;
    // an edge with no stretch leaves it no way.
    std::unordered_set<std::uint32_t> drawnFeedlines;
    for (const auto& wall : walls.list) {
      if (wall.kind == WallKind::Feedline) {
        drawnFeedlines.insert(wall.wire);
      }
    }
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> prescribed;
    for (const auto& edge : edges) {
      if (edge.terminal || !drawnFeedlines.contains(edge.wire)) {
        continue;
      }
      for (const auto key : bridgers_[edge.chain][edge.from]) {
        prescribed[key].push_back(edge.wire);
      }
    }
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> stretchesOf;
    for (std::uint32_t index = 0; index < graph.size(); ++index) {
      if (graph[index].crossing) {
        stretchesOf[graph[index].id].push_back(index);
      }
    }

    // One demand per outer wire whose two ports lie in chambers.
    std::unordered_map<std::uint32_t, std::uint32_t> demandOf;
    problem.portsIn.assign(chambers.count, 0);
    for (const auto& wire : wires) {
      if (!wire.feasible || wire.inner || wire.feedline) {
        continue;
      }
      ++problem.outer;
      const auto ends = portsOf(walls, wire);
      auto from =
          chambersOfPort(chambers, walls, ends[0].first, ends[0].second);
      auto to = chambersOfPort(chambers, walls, ends[1].first, ends[1].second);
      problem.portMarks.emplace_back(ends[0].first, wire.key);
      problem.portMarks.emplace_back(ends[1].first, wire.key);
      const auto mine = prescribed.find(wire.key);
      problem.outerWires.push_back(
          {.wire = wire.key,
           .ends = {ends[0].first, ends[1].first},
           .from = from,
           .to = to,
           .demand = std::nullopt,
           .crosses = mine != prescribed.end() ? mine->second
                                               : std::vector<std::uint32_t>{}});
      if (from.empty() || to.empty()) {
        problem.walled.push_back(wire.key);
        continue;
      }
      ++problem.portsIn[from.front()];
      ++problem.portsIn[to.front()];
      problem.outerWires.back().demand =
          static_cast<std::uint32_t>(problem.demands.size());
      demandOf.emplace(wire.key,
                       static_cast<std::uint32_t>(problem.demands.size()));
      problem.wireOf.push_back(wire.key);
      std::vector<std::vector<std::uint32_t>> crossings;
      for (const auto feedline : problem.outerWires.back().crosses) {
        const auto found = stretchesOf.find(feedline);
        crossings.push_back(found != stretchesOf.end()
                                ? found->second
                                : std::vector<std::uint32_t>{});
      }
      const auto pointOf = [](const PathPoint& point) {
        return FlowPoint{.x = static_cast<double>(point.x),
                         .y = static_cast<double>(point.y)};
      };
      problem.demands.push_back({.from = std::move(from),
                                 .to = std::move(to),
                                 .crossings = std::move(crossings),
                                 .longest = longestWayOf(wire),
                                 .start = pointOf(ends[0].first),
                                 .end = pointOf(ends[1].first)});
    }
    problem.flowEdges.reserve(graph.size());
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    for (const auto& edge : graph) {
      FlowEdge flow{
          .chambers = edge.chambers, .capacity = edge.capacity, .users = {}};
      // The line a way passes the edge on: the bottleneck's line, or the
      // stretch from its first cell to its last.
      if (edge.crossing) {
        const auto& way = wires[edge.id].way;
        flow.from = {.x = static_cast<double>(way[edge.first].x),
                     .y = static_cast<double>(way[edge.first].y)};
        flow.to = {.x = static_cast<double>(way[edge.last].x),
                   .y = static_cast<double>(way[edge.last].y)};
      } else {
        const auto& gate = measured[edge.id].gate;
        const auto a = static_cast<std::int64_t>(gate.first);
        const auto b = static_cast<std::int64_t>(gate.second);
        flow.from = {.x = static_cast<double>(a % width),
                     .y = static_cast<double>(a / width)};
        flow.to = {.x = static_cast<double>(b % width),
                   .y = static_cast<double>(b / width)};
      }
      if (edge.users.has_value()) {
        flow.users = std::vector<std::uint32_t>{};
        for (const auto key : *edge.users) {
          if (const auto found = demandOf.find(key); found != demandOf.end()) {
            flow.users->push_back(found->second);
          }
        }
      }
      problem.flowEdges.push_back(std::move(flow));
    }
    return problem;
  }

  /// The capacity graph of the chip the insertion leaves behind, and whether
  /// it carries every outer wire at once (user, 2026-10-07), report only.
  ///
  /// The bottlenecks cut the free space into chambers, as they cut it in
  /// the capacity stage: a walk over the cells that never steps across a
  /// bottleneck's line (`grid::chambersOf`). A chamber is a node. A
  /// bottleneck between two or more chambers is an edge every wire may
  /// take, and `holds` wires — `wiresThroughGap` — fit
  /// through; one with the same chamber on both sides has a way round it
  /// and separates nothing. The feedlines are walls, and the stretches of
  /// `crossingStretches` are the edges across them, each open to the wires
  /// the feedline pass lets cross there.
  ///
  /// Every outer wire is a demand from the chamber its source terminal
  /// lies in to the chamber of its target terminal, each in the slot of
  /// its port run, crossing exactly the feedline edges it is prescribed.
  /// `checkInTurn` routes them one after another in wire order, each over
  /// the edges that still take a wire (user, 2026-10-08); under
  /// `SCPD_CAPACITY_FLOW`, `checkCapacity` routes all of them at once as an
  /// integer multi-commodity flow with the least overflow. An overflow of
  /// zero says the graph carries every wire; an edge with overflow is where
  /// it does not, and the wires sent through it are the ones that compete
  /// there. A wire the graph has no way for at all is said apart. The check
  /// is necessary and not sufficient: the graph knows nothing of the order
  /// of wires inside a chamber, of turns or of lengths.
  void reportCapacityGraph(const std::vector<Wire>& wires, const Walls& walls,
                           const std::vector<grid::Bottleneck>& gates,
                           const std::vector<MeasuredBottleneck>& measured) {
    const auto began = std::chrono::steady_clock::now();
    auto lapStart = began;
    std::string laps;
    const auto lap = [&](const std::string_view what) {
      const auto now = std::chrono::steady_clock::now();
      laps +=
          std::format("{}{} {:.3f}", laps.empty() ? "" : ", ", what,
                      std::chrono::duration<double>(now - lapStart).count());
      lapStart = now;
    };
    const auto chambers = grid::chambersOf(walls.mask, gates, scene_.router);
    lap("chambers");

    auto problem = capacityProblemOf(wires, edges_, walls, measured, chambers);
    auto& graph = problem.graph;
    const auto roundabout = problem.roundabout;
    const auto gateEdges = problem.gateEdges;
    const auto& demands = problem.demands;
    const auto& wireOf = problem.wireOf;
    const auto& walled = problem.walled;
    const auto outer = problem.outer;
    for (std::size_t index = gateEdges; index < graph.size(); ++index) {
      const auto& stretch = graph[index];
      const auto& way = wires[stretch.id].way;
      tell(std::format(
          "[Capacity] stretch {} cells {}..{} from ({},{}) to ({},{}), "
          "{:.0f} cells, chambers c{} and c{}, takes {}, open to {}",
          wireId(wires[stretch.id]), stretch.first, stretch.last,
          way[stretch.first].x, way[stretch.first].y, way[stretch.last].x,
          way[stretch.last].y, stretch.length, stretch.chambers[0],
          stretch.chambers[1], stretch.capacity,
          stretch.users.has_value() ? namesOf(wires, *stretch.users)
                                    : std::string("every wire")));
    }
    lap("stretches and ports");
    FlowCheck check;
    if (capacityFlow()) {
      milp::SolveOptions options;
      options.timeLimit = capacitySeconds();
      check =
          checkCapacity(chambers.count, problem.flowEdges, demands, options);
      lap("flow");
    } else {
      check = checkInTurn(chambers.count, problem.flowEdges, demands);
      lap("in turn");
    }

    std::vector<std::uint32_t> noWay;
    for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
      if (!check.ways[demand].has_value()) {
        noWay.push_back(wireOf[demand]);
        continue;
      }
      for (const auto index : *check.ways[demand]) {
        graph[index].through.push_back(wireOf[demand]);
      }
    }

    const auto nameOf = [&](const GraphEdge& edge) {
      if (!edge.crossing) {
        const auto& gate = gates[edge.id];
        return std::format("g{} {} · {}", edge.id, endName(walls, gate.first),
                           endName(walls, gate.second));
      }
      return std::format("× {} cells {}..{}", wireId(wires[edge.id]),
                         edge.first, edge.last);
    };
    std::vector<std::vector<std::uint32_t>> crossesOf(demands.size());
    for (const auto& one : problem.outerWires) {
      if (one.demand.has_value()) {
        crossesOf[*one.demand] = one.crosses;
      }
    }
    for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
      const auto& way = check.ways[demand];
      std::string said;
      if (way.has_value()) {
        for (const auto index : *way) {
          const auto& edge = graph[index];
          said += std::format("{}{} ({}/{})", said.empty() ? "" : " → ",
                              nameOf(edge), edge.through.size(), edge.capacity);
        }
      }
      const auto& feedlines = crossesOf[demand];
      tell(std::format(
          "[Capacity] {}{}: {}", wireId(wires[wireOf[demand]]),
          feedlines.empty()
              ? std::string()
              : std::format(", crossing {}", namesOf(wires, feedlines)),
          !way.has_value()
              ? (demand < check.tooLong.size() && check.tooLong[demand]
                     ? std::format("no way within its length: the shortest "
                                   "runs {:.0f} cells, it may run {:.0f}",
                                   check.lengths[demand],
                                   demands[demand].longest.value_or(0.0))
                     : std::string("no way from its source chamber to its "
                                   "target chamber"))
          : way->empty() ? std::string("source and target in one chamber")
                         : said));
    }

    std::uint32_t crossings = 0;
    std::set<std::uint32_t> crossed;
    std::string overList;
    std::uint32_t overEdges = 0;
    for (std::uint32_t index = 0; index < graph.size(); ++index) {
      const auto& edge = graph[index];
      if (edge.crossing) {
        crossings += edge.capacity;
        crossed.insert(edge.id);
      }
      if (index < check.overflow.size() && check.overflow[index] > 0) {
        ++overEdges;
        const auto one = std::format(
            "{} +{} ({}/{}: {})", nameOf(edge), check.overflow[index],
            edge.through.size(), edge.capacity, namesOf(wires, edge.through));
        tell(std::format("[Capacity] over: {}", one));
        overList += (overList.empty() ? "" : " · ") + one;
      }
    }
    const auto counted = std::format(
        "coupler insertion: CAPACITY GRAPH — {} chambers, {} bottlenecks "
        "between two or more of them ({} with a way round them), {} stretches "
        "to cross on {} feedline edges ({} crossings in all)",
        chambers.count, gateEdges, roundabout, graph.size() - gateEdges,
        crossed.size(), crossings);
    say(counted);
    // The verdict. SAT: every outer wire has a way and the ways keep every
    // edge within what it takes. UNSAT: a wire has no way at all, a port
    // lies in no chamber, the wires routed one after another overfill an
    // edge, or the least overflow the solver proved is above zero. UNKNOWN:
    // the time limit stopped the solver with overflow left and no proof
    // that it is the least.
    const auto proven = check.status == milp::SolveStatus::Optimal;
    const auto answered = proven || check.status == milp::SolveStatus::Feasible;
    const bool lost = !noWay.empty() || !walled.empty();
    const auto verdict = lost                 ? "UNSAT"
                         : !answered          ? "UNKNOWN"
                         : check.shortBy == 0 ? "SAT"
                         : proven             ? "UNSAT"
                                              : "UNKNOWN";
    const auto resonators = static_cast<std::uint32_t>(
        std::ranges::count_if(wires, [](const Wire& wire) {
          return wire.feasible && !wire.inner && !wire.feedline &&
                 wire.resonator;
        }));
    std::string why;
    if (!noWay.empty()) {
      // The wires refused only for their length apart from the rest.
      std::vector<std::uint32_t> tooLong;
      std::vector<std::uint32_t> nowhere;
      for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
        if (check.ways[demand].has_value()) {
          continue;
        }
        (demand < check.tooLong.size() && check.tooLong[demand] ? tooLong
                                                                : nowhere)
            .push_back(wireOf[demand]);
      }
      if (!nowhere.empty()) {
        why += std::format("; no way in the graph for {}",
                           namesOf(wires, nowhere));
      }
      if (!tooLong.empty()) {
        why += std::format("; no way within its length for {}",
                           namesOf(wires, tooLong));
      }
    }
    if (!walled.empty()) {
      why +=
          std::format("; a port in no chamber for {}", namesOf(wires, walled));
    }
    if (!answered) {
      why += "; the solver gave no answer";
    } else if (check.shortBy > 0) {
      why += std::format(
          "; short by {} wire{} on {} edge{}{}: {}", check.shortBy,
          check.shortBy == 1 ? "" : "s", overEdges, overEdges == 1 ? "" : "s",
          !capacityFlow() ? ", routed one after another in wire order"
          : proven        ? ""
                          : ", not proven least before the time limit",
          overList);
    }
    const auto judged = std::format(
        "coupler insertion: CAPACITY GRAPH {} — {} outer wires, {} plain and "
        "{} "
        "resonators from their coupler port, each plain wire crossing "
        "exactly the feedline edges it is prescribed{}; {:.2f}s ({})",
        verdict, outer, outer - resonators, resonators,
        !why.empty() ? why
        : capacityFlow()
            ? std::string("; every one has a way within the capacities")
            : std::string("; every one has a way within the capacities, "
                          "routed one after another in wire order"),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count(),
        (lap("report"), laps));
    say(judged);
    if (!graphOnly_) {
      drawCapacityGraph(wires, walls, gates, chambers, graph, problem.portsIn,
                        problem.portMarks);
    }
    writeCapacityGraph(wires, walls, gates, measured, chambers, graph,
                       problem.outerWires, check, verdict, {counted, judged});
  }

  /// How many times `benchCapacityCheck` runs the whole capacity analysis
  /// to time it: `SCPD_CAPACITY_BENCH`, 0. Zero runs no measurement. It
  /// measures what a capacity check inside the insertion would cost and
  /// changes nothing the stage does.
  [[nodiscard]] static std::uint32_t capacityBench() {
    return static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_CAPACITY_BENCH", 0), 0, 1000));
  }
  /// The directory `benchCapacityCheck` writes the flow problems to as
  /// JSON: `SCPD_CAPACITY_DUMP`, none.
  [[nodiscard]] static std::string capacityDump() {
    const char* value = envSet("SCPD_CAPACITY_DUMP");
    return value == nullptr ? std::string() : std::string(value);
  }

  /// One quiet run of the whole capacity analysis and the time of each
  /// step, in the order of `CAPACITY_STEPS`.
  struct CapacityPass {
    Walls walls;
    std::vector<grid::Bottleneck> gates;
    grid::Chambers chambers;
    CapacityProblem problem;
    FlowCheck check;
    /// The outer wires with no way: a terminal in no chamber, or no way in
    /// the graph (alone, or at all under the flow).
    std::uint32_t unrouted = 0;
    std::array<double, 9> seconds{};
  };
  static constexpr std::array<std::string_view, 9> CAPACITY_STEPS{
      "walls",   "distance", "voronoi",             "axis", "saddles",
      "measure", "chambers", "stretches and ports", "check"};

  /// The analysis of `reportBottlenecks` and `reportCapacityGraph` without
  /// a line of log, with the feedline edge of wire `without` left out.
  [[nodiscard]] CapacityPass capacityPass(const std::vector<Wire>& wires,
                                          const std::uint32_t without) const {
    return capacityPass(wires, edges_, nullptr, without);
  }

  /// `capacityPass` for a state of the insertion, as `wallsOf` takes it.
  [[nodiscard]] CapacityPass capacityPass(const std::vector<Wire>& wires,
                                          const std::vector<Edge>& edges,
                                          const std::vector<bool>* placed,
                                          const std::uint32_t without) const {
    CapacityPass pass;
    auto mark = std::chrono::steady_clock::now();
    std::size_t step = 0;
    const auto lap = [&] {
      const auto now = std::chrono::steady_clock::now();
      pass.seconds.at(step++) =
          std::chrono::duration<double>(now - mark).count();
      mark = now;
    };
    pass.walls = wallsOf(wires, edges, placed, without);
    lap();
    const auto distance = grid::squaredDistanceTransform(pass.walls.mask);
    lap();
    const auto medial = grid::medialAxis(pass.walls.mask);
    lap();
    const auto axis =
        grid::rasterizeMedialAxis(pass.walls.mask, scene_.router, medial);
    lap();
    pass.gates = couplerCuts(
        pass.walls.list, pass.walls.of,
        grid::findBottlenecks(
            pass.walls.mask, axis, distance, scene_.router,
            bottleneckOptionsOf(pass.walls.slots, pass.walls.of)));
    lap();
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<MeasuredBottleneck> measured;
    measured.reserve(pass.gates.size());
    for (const auto& gate : pass.gates) {
      MeasuredBottleneck one{.gate = gate};
      one.cells = std::hypot(
          static_cast<double>((static_cast<std::int64_t>(gate.second) % width) -
                              (static_cast<std::int64_t>(gate.first) % width)),
          static_cast<double>((static_cast<std::int64_t>(gate.second) / width) -
                              (static_cast<std::int64_t>(gate.first) / width)));
      one.holds = wiresThroughGap(one.cells,
                                  static_cast<double>(tuning_.clearance));
      measured.push_back(std::move(one));
    }
    lap();
    pass.chambers =
        grid::chambersOf(pass.walls.mask, pass.gates, scene_.router);
    lap();
    pass.problem =
        capacityProblemOf(wires, edges, pass.walls, measured, pass.chambers);
    lap();
    if (capacityFlow()) {
      milp::SolveOptions options;
      options.timeLimit = capacitySeconds();
      pass.check = checkCapacity(pass.chambers.count, pass.problem.flowEdges,
                                 pass.problem.demands, options);
    } else {
      pass.check = checkInTurn(pass.chambers.count, pass.problem.flowEdges,
                               pass.problem.demands);
    }
    lap();
    pass.unrouted = static_cast<std::uint32_t>(
        pass.problem.walled.size() +
        static_cast<std::size_t>(std::ranges::count_if(
            pass.check.ways, [](const auto& way) { return !way.has_value(); })));
    return pass;
  }

  /// A flow problem of `capacityPass` as JSON, for solvers outside the
  /// stage: the chambers with the lowest cell of each, the edges with a key
  /// that names the same edge in another pass, the demands, and the answer
  /// of `checkCapacity`.
  void dumpCapacityPass(const std::vector<Wire>& wires,
                        const CapacityPass& pass, const std::string& name,
                        const double flowSeconds) const {
    const auto directory = capacityDump();
    if (directory.empty()) {
      return;
    }
    using nlohmann::json;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<std::int64_t> lowest(pass.chambers.count, -1);
    for (std::size_t cell = 0; cell < pass.chambers.of.size(); ++cell) {
      const auto chamber = pass.chambers.of[cell];
      if (chamber != grid::NO_CHAMBER && lowest[chamber] < 0) {
        lowest[chamber] = static_cast<std::int64_t>(cell);
      }
    }
    json edges = json::array();
    for (std::size_t index = 0; index < pass.problem.graph.size(); ++index) {
      const auto& edge = pass.problem.graph[index];
      std::string key;
      if (edge.crossing) {
        key = std::format("s {} {} {}", wireId(wires[edge.id]), edge.first,
                          edge.last);
      } else {
        const auto& gate = pass.gates[edge.id];
        const auto low = std::min(gate.first, gate.second);
        const auto high = std::max(gate.first, gate.second);
        key = std::format("g {} {} {} {}", static_cast<std::int64_t>(low) % width,
                          static_cast<std::int64_t>(low) / width,
                          static_cast<std::int64_t>(high) % width,
                          static_cast<std::int64_t>(high) / width);
      }
      const auto& flow = pass.problem.flowEdges[index];
      edges.push_back(
          {{"key", key},
           {"chambers", flow.chambers},
           {"capacity", flow.capacity},
           {"users", flow.users.has_value() ? json(*flow.users) : json(nullptr)},
           {"overflow", pass.check.overflow[index]}});
    }
    json demands = json::array();
    for (std::size_t demand = 0; demand < pass.problem.demands.size();
         ++demand) {
      const auto& wanted = pass.problem.demands[demand];
      const auto& way = pass.check.ways[demand];
      demands.push_back(
          {{"wire", wireId(wires[pass.problem.wireOf[demand]])},
           {"from", wanted.from},
           {"to", wanted.to},
           {"way", way.has_value() ? json(*way) : json(nullptr)}});
    }
    json walled = json::array();
    for (const auto key : pass.problem.walled) {
      walled.push_back(wireId(wires[key]));
    }
    const json data{{"chambers", pass.chambers.count},
                    {"lowest", lowest},
                    {"width", width},
                    {"edges", edges},
                    {"demands", demands},
                    {"walled", walled},
                    {"status", static_cast<int>(pass.check.status)},
                    {"shortBy", pass.check.shortBy},
                    {"flowSeconds", flowSeconds}};
    std::ofstream out(std::filesystem::path(directory) / (name + ".json"));
    out << data.dump();
  }

  /// What a capacity check inside the insertion would cost
  /// (`SCPD_CAPACITY_BENCH`), measurement only. Three parts, one line of
  /// log each:
  ///
  /// - the whole analysis repeated on the chip as it stands, step by step,
  ///   and the flow alone repeated on the one graph;
  /// - every drawn feedline edge left out once: what the edge changes in
  ///   the overflow, how many cuts it adds or removes and how far from its
  ///   way they lie, and how many cells the chambers near it hold;
  /// - the medial axis and the cuts recomputed in a window around every
  ///   edge alone, at two reaches, and how many of the cuts in the middle
  ///   of the window agree with those of the whole chip.
  ///
  /// With `SCPD_CAPACITY_DUMP` every flow problem is also written out.
  void benchCapacityCheck(const std::vector<Wire>& wires) {
    const auto repeats = capacityBench();
    if (repeats == 0) {
      return;
    }
    const auto spread = [](std::vector<double> values) {
      std::ranges::sort(values);
      const auto middle = values.size() / 2;
      const auto median = values.size() % 2 == 1
                              ? values[middle]
                              : 0.5 * (values[middle - 1] + values[middle]);
      return std::format("{:.4f} [{:.4f}..{:.4f}]", median, values.front(),
                         values.back());
    };
    const auto since = [](const std::chrono::steady_clock::time_point began) {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           began)
          .count();
    };

    // The whole analysis, again and again on the same chip.
    std::vector<std::vector<double>> times(CAPACITY_STEPS.size() + 1);
    std::optional<CapacityPass> full;
    for (std::uint32_t round = 0; round < repeats; ++round) {
      auto pass = capacityPass(wires, NO_OWNER);
      double total = 0.0;
      for (std::size_t step = 0; step < CAPACITY_STEPS.size(); ++step) {
        times[step].push_back(pass.seconds.at(step));
        total += pass.seconds.at(step);
      }
      times.back().push_back(total);
      if (!full.has_value()) {
        full = std::move(pass);
      }
    }
    std::string steps;
    for (std::size_t step = 0; step < CAPACITY_STEPS.size(); ++step) {
      steps += std::format("{}{} {}", steps.empty() ? "" : ", ",
                           CAPACITY_STEPS.at(step), spread(times[step]));
    }
    std::uint32_t demanded = 0;
    for (const auto& edge : full->problem.flowEdges) {
      demanded += edge.capacity;
    }
    say(std::format(
        "coupler insertion: CAPACITY BENCH — the whole analysis {} times, "
        "median [min..max] seconds: {}; in all {}; {} chambers, {} edges, {} "
        "demands, {} wires with no way, short by {} ({})",
        repeats, steps, spread(times.back()), full->chambers.count,
        full->problem.flowEdges.size(), full->problem.demands.size(),
        full->unrouted, full->check.shortBy,
        capacityFlow() ? "the flow" : "the wires in turn"));
    milp::SolveOptions options;
    options.timeLimit = capacitySeconds();
    std::vector<double> flows;
    std::vector<double> turns;
    for (std::uint32_t round = 0; round < repeats; ++round) {
      auto began = std::chrono::steady_clock::now();
      static_cast<void>(checkCapacity(full->chambers.count,
                                      full->problem.flowEdges,
                                      full->problem.demands, options));
      flows.push_back(since(began));
      began = std::chrono::steady_clock::now();
      static_cast<void>(checkInTurn(full->chambers.count,
                                    full->problem.flowEdges,
                                    full->problem.demands));
      turns.push_back(since(began));
    }
    say(std::format("coupler insertion: CAPACITY BENCH — the check alone on "
                    "one graph {} times: the flow {} s, the wires in turn {} s",
                    repeats, spread(flows), spread(turns)));
    dumpCapacityPass(wires, *full, "full", times[8].front());

    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto keyOf = [](const grid::Bottleneck& gate) {
      return std::pair{std::min(gate.first, gate.second),
                       std::max(gate.first, gate.second)};
    };
    std::set<std::pair<std::size_t, std::size_t>> fullKeys;
    for (const auto& gate : full->gates) {
      fullKeys.insert(keyOf(gate));
    }
    std::vector<std::size_t> chamberCells(full->chambers.count, 0);
    std::size_t freeCells = 0;
    for (const auto chamber : full->chambers.of) {
      if (chamber != grid::NO_CHAMBER) {
        ++chamberCells[chamber];
        ++freeCells;
      }
    }
    // The way's box, grown by `reach` and kept on the grid.
    struct Box {
      std::int64_t x0, y0, x1, y1;
    };
    const auto boxOf = [&](const Path& way, const std::int64_t reach) {
      Box box{width, height, -1, -1};
      for (const auto& point : way) {
        box.x0 = std::min<std::int64_t>(box.x0, point.x);
        box.y0 = std::min<std::int64_t>(box.y0, point.y);
        box.x1 = std::max<std::int64_t>(box.x1, point.x);
        box.y1 = std::max<std::int64_t>(box.y1, point.y);
      }
      return Box{std::max<std::int64_t>(0, box.x0 - reach),
                 std::max<std::int64_t>(0, box.y0 - reach),
                 std::min<std::int64_t>(width - 1, box.x1 + reach),
                 std::min<std::int64_t>(height - 1, box.y1 + reach)};
    };
    const auto inside = [width](const Box& box, const std::size_t cell) {
      const auto x = static_cast<std::int64_t>(cell) % width;
      const auto y = static_cast<std::int64_t>(cell) / width;
      return x >= box.x0 && x <= box.x1 && y >= box.y0 && y <= box.y1;
    };

    // Every drawn feedline edge left out once.
    static constexpr std::int64_t NEAR = 120;
    for (const auto& edge : edges_) {
      const auto& wire = wires[edge.wire];
      if (!wire.drawn || wire.way.empty()) {
        continue;
      }
      const auto began = std::chrono::steady_clock::now();
      const auto without = capacityPass(wires, edge.wire);
      const auto seconds = since(began);
      std::set<std::pair<std::size_t, std::size_t>> withoutKeys;
      for (const auto& gate : without.gates) {
        withoutKeys.insert(keyOf(gate));
      }
      // How far a cut that differs lies from the edge's way, from the
      // middle of its line.
      const auto away = [&](const std::pair<std::size_t, std::size_t>& key) {
        const auto mx = 0.5 * static_cast<double>(
                                  (static_cast<std::int64_t>(key.first) % width) +
                                  (static_cast<std::int64_t>(key.second) % width));
        const auto my = 0.5 * static_cast<double>(
                                  (static_cast<std::int64_t>(key.first) / width) +
                                  (static_cast<std::int64_t>(key.second) / width));
        double best = std::numeric_limits<double>::max();
        for (const auto& point : wire.way) {
          best = std::min(best, std::hypot(mx - point.x, my - point.y));
        }
        return best;
      };
      std::uint32_t added = 0;
      std::uint32_t removed = 0;
      double farthest = 0.0;
      for (const auto& key : fullKeys) {
        if (!withoutKeys.contains(key)) {
          ++added;
          farthest = std::max(farthest, away(key));
        }
      }
      for (const auto& key : withoutKeys) {
        if (!fullKeys.contains(key)) {
          ++removed;
          farthest = std::max(farthest, away(key));
        }
      }
      // The chambers of the whole chip with a cell near the edge: what a
      // walk that redraws the chambers around the edge has to cover.
      const auto near = boxOf(wire.way, NEAR);
      std::set<std::uint32_t> touched;
      for (auto y = near.y0; y <= near.y1; ++y) {
        for (auto x = near.x0; x <= near.x1; ++x) {
          const auto chamber =
              full->chambers.of[static_cast<std::size_t>((y * width) + x)];
          if (chamber != grid::NO_CHAMBER) {
            touched.insert(chamber);
          }
        }
      }
      std::size_t touchedCells = 0;
      for (const auto chamber : touched) {
        touchedCells += chamberCells[chamber];
      }
      say(std::format(
          "coupler insertion: CAPACITY BENCH f{} ({} cells) — without it "
          "{} wires with no way and short by {}, with it {} and {}; it adds "
          "{} cuts and removes {}, the "
          "farthest {:.0f} cells from its way; {} of {} chambers within {} "
          "cells of it, {} of {} free cells; the analysis without it {:.3f} s "
          "(check {:.3f} s)",
          wire.slot, wire.way.size(), without.unrouted, without.check.shortBy,
          full->unrouted, full->check.shortBy, added, removed, farthest,
          touched.size(),
          full->chambers.count, NEAR, touchedCells, freeCells, seconds,
          without.seconds[8]));
      dumpCapacityPass(wires, without, std::format("without-f{}", wire.slot),
                       without.seconds[8]);
    }

    // The axis and the cuts again in a window around each edge alone.
    std::unordered_set<std::size_t> slotCells(full->walls.slots.begin(),
                                              full->walls.slots.end());
    for (const auto reach : {std::int64_t{120}, std::int64_t{240}}) {
      std::vector<double> windowTimes;
      std::uint32_t agree = 0;
      std::uint32_t wholeOnly = 0;
      std::uint32_t windowOnly = 0;
      std::size_t windowCells = 0;
      for (const auto& edge : edges_) {
        const auto& wire = wires[edge.wire];
        if (!wire.drawn || wire.way.empty()) {
          continue;
        }
        const auto box = boxOf(wire.way, reach);
        const auto core = boxOf(wire.way, reach / 2);
        const auto cw = static_cast<std::uint32_t>(box.x1 - box.x0 + 1);
        const auto ch = static_cast<std::uint32_t>(box.y1 - box.y0 + 1);
        windowCells += static_cast<std::size_t>(cw) * ch;
        const auto began = std::chrono::steady_clock::now();
        grid::BitGrid mask(cw, ch);
        std::vector<std::uint32_t> of(static_cast<std::size_t>(cw) * ch,
                                      NO_OWNER);
        std::vector<std::size_t> slots;
        for (std::int64_t y = 0; y < ch; ++y) {
          for (std::int64_t x = 0; x < cw; ++x) {
            const auto cell =
                static_cast<std::size_t>(((box.y0 + y) * width) + box.x0 + x);
            const auto local = static_cast<std::size_t>((y * cw) + x);
            if (full->walls.mask.test(cell)) {
              mask.set(local);
              of[local] = full->walls.of[cell];
            }
            if (slotCells.contains(cell)) {
              slots.push_back(local);
            }
          }
        }
        auto metrics = scene_.router;
        metrics.width = cw;
        metrics.height = ch;
        const auto distance = grid::squaredDistanceTransform(mask);
        const auto medial = grid::medialAxis(mask);
        const auto axis = grid::rasterizeMedialAxis(mask, metrics, medial);
        const auto gates = couplerCuts(
            full->walls.list, of,
            grid::findBottlenecks(mask, axis, distance, metrics,
                                  bottleneckOptionsOf(slots, of)));
        windowTimes.push_back(since(began));
        const auto global = [&](const std::size_t local) {
          return static_cast<std::size_t>(
              ((box.y0 + (static_cast<std::int64_t>(local) / cw)) * width) +
              box.x0 + (static_cast<std::int64_t>(local) % cw));
        };
        std::set<std::pair<std::size_t, std::size_t>> inWindow;
        for (const auto& gate : gates) {
          const grid::Bottleneck mapped{.first = global(gate.first),
                                        .second = global(gate.second),
                                        .saddle = 0};
          if (inside(core, mapped.first) && inside(core, mapped.second)) {
            inWindow.insert(keyOf(mapped));
          }
        }
        for (const auto& key : fullKeys) {
          if (inside(core, key.first) && inside(core, key.second)) {
            if (inWindow.contains(key)) {
              ++agree;
            } else {
              ++wholeOnly;
            }
          }
        }
        for (const auto& key : inWindow) {
          windowOnly += fullKeys.contains(key) ? 0 : 1;
        }
      }
      say(std::format(
          "coupler insertion: CAPACITY BENCH windows of {} cells around each "
          "edge — distance, axis and cuts {} s per edge, {:.0f} cells per "
          "window against {} on the chip; in the middle half of the windows "
          "{} cuts agree with the whole chip's, {} only on the chip, {} only "
          "in the window",
          reach, spread(windowTimes),
          static_cast<double>(windowCells) /
              static_cast<double>(std::max<std::size_t>(1, windowTimes.size())),
          full->walls.mask.size(), agree, wholeOnly, windowOnly));
    }
  }

  /// R4, report only: the channels between the terminal edges of two
  /// chains. Walking the ring, every maximal run of plain wires between two
  /// resonators whose couplers belong to different chains lies between the
  /// last edge of the one chain and the first edge of the other; the gap
  /// between those two committed ways, both pad runs excluded, is measured
  /// against what the run's wires need.
  void checkChainChannels(const std::vector<Wire>& wires) {
    const auto pitch = roomPitch();
    const auto n = ring_.size();
    std::string named;
    const auto couplerOf =
        [&](const Wire& wire) -> std::optional<std::uint32_t> {
      if (!wire.resonator) {
        return std::nullopt;
      }
      const auto found = couplerOfWire_.find(wire.key);
      if (found == couplerOfWire_.end()) {
        return std::nullopt;
      }
      return found->second;
    };
    std::size_t start = n;
    for (std::size_t p = 0; p < n; ++p) {
      if (couplerOf(wires[ring_[p]]).has_value()) {
        start = p;
        break;
      }
    }
    if (start < n) {
      std::vector<std::uint32_t> between;
      auto last = start;
      for (std::size_t k = 1; k <= n; ++k) {
        const auto p = (start + k) % n;
        const auto& wire = wires[ring_[p]];
        const auto b = couplerOf(wire);
        if (!b.has_value()) {
          if (!wire.resonator && wire.feasible) {
            between.push_back(wire.key);
          }
          continue;
        }
        const auto a = *couplerOf(wires[ring_[last]]);
        const auto [chainA, atA] = chainOfCoupler_[a];
        const auto [chainB, atB] = chainOfCoupler_[*b];
        if (chainA != NO_OWNER && chainB != NO_OWNER && chainA != chainB &&
            !between.empty()) {
          const auto outKey = couplers_[a].edgeOut;
          const auto inKey = couplers_[*b].edgeIn;
          if (outKey != NO_OWNER && inKey != NO_OWNER && wires[outKey].drawn &&
              wires[inKey].drawn) {
            ++room_.crossChainChannels;
            // Reversed, so that the pad run of each lies where
            // `channelBetween` skips: the out-edge's at its start, the
            // in-edge's at its end.
            const Path first(wires[outKey].way.rbegin(),
                             wires[outKey].way.rend());
            const Path second(wires[inKey].way.rbegin(),
                              wires[inKey].way.rend());
            const auto runA = couplers_[a].options[couplers_[a].chosen].run;
            const auto runB = couplers_[*b].options[couplers_[*b].chosen].run;
            const auto channel = routing::channelBetween(
                first, runA + 1, first.size(), second, runB + 1, second.size());
            const auto needs =
                (static_cast<std::uint32_t>(between.size()) + 1) * pitch;
            if (channel.measured && channel.gap < needs) {
              ++room_.crossChainTight;
              if (room_.crossChainTight <= 12) {
                named += std::format(
                    "{}chain {} f{} / chain {} f{}: {} wire{} ({}), gap "
                    "{:.1f} cells at ({},{}) against {}",
                    named.empty() ? ": " : " · ", chainA, wires[outKey].slot,
                    chainB, wires[inKey].slot, between.size(),
                    between.size() == 1 ? "" : "s", namesOf(wires, between),
                    channel.gap, first[channel.at].x, first[channel.at].y,
                    needs);
              }
            }
          }
        }
        between.clear();
        last = p;
      }
    }
    say(std::format(
        "coupler insertion: CHECK chain channels — {} of {} channel{} between "
        "chains tighter than (m+1)·pitch, {} of {} coupler channel{} tighter{}",
        room_.crossChainTight, room_.crossChainChannels,
        room_.crossChainChannels == 1 ? "" : "s", room_.committedTightChannels,
        room_.couplerChannels, room_.couplerChannels == 1 ? "" : "s",
        (room_.crossChainTight == 0 && room_.committedTightChannels == 0)
            ? "; the check is GREEN"
            : named));
  }

  /// The chain being searched on its own, or NO_OWNER. Set around a single
  /// `solveChainAStar` call and cleared again, so nothing the commit does
  /// ever sees it.
  std::uint32_t soloChain_ = NO_OWNER;
  /// Whether `analyseCapacity` writes only the data and draws no picture:
  /// set by `capacityGraphJson`.
  bool graphOnly_ = false;
  /// The capacity check of `checkChainCapacity` while a chain is searched
  /// again: the chain, the chains settled before it, the wires that had no
  /// way before it, how many more wires a run of options may leave without
  /// one, and how many of its edges may fail the commit's test
  /// (`chainFaults`). A run that leaves `limit` or more wires, or fails on
  /// more edges than `faults`, is refused at its last step.
  struct ChainCheck {
    bool active = false;
    std::uint32_t chain = NO_OWNER;
    const std::vector<bool>* settled = nullptr;
    std::set<std::uint32_t> before;
    std::size_t limit = 0;
    std::uint32_t faults = 0;
    std::uint32_t checks = 0;
    std::uint32_t refused = 0;
  };
  ChainCheck chainCheck_;
  /// The chains settled before the one being searched, while
  /// `capacityStep` or `capacityRule` checks every step of its search; null
  /// otherwise. With `stepRefuses_` a step whose edge closes a wire is
  /// refused.
  const std::vector<bool>* stepSettled_ = nullptr;
  bool stepRefuses_ = false;
  /// The room of the resonators outside one chain, and which chain that is.
  /// `foreignRoomStale_` is set wherever a resonator can have moved.
  grid::BitGrid foreignRoom_;
  std::uint32_t foreignRoomChain_ = NO_OWNER;
  bool foreignRoomStale_ = true;
  /// Set around `stateFaults`, and around a segment search of the targeted
  /// repair under `researchSeesChains`: fence every chain against every
  /// other, whatever `edgesSeeOtherChains` says.
  bool fenceEverything_ = false;
  /// The segment the targeted repair is re-searching — chain and the edges
  /// `[openLo_, openHi_)` — which `fenceCommittedEdges` leaves open, or
  /// NO_OWNER. Set around `researchSegment` and cleared again.
  std::uint32_t openChain_ = NO_OWNER;
  std::size_t openLo_ = 0;
  std::size_t openHi_ = 0;

  /// The chain that is being searched and is therefore not fenced by its own
  /// edges, or NO_OWNER. Only `SCPD_CHAIN_DP=2` ever sets it.
  std::uint32_t looseChain_ = NO_OWNER;

  /// Whether every cell of a way the greedy chose is still open in the
  /// corridor this edge would be searched in now.
  ///
  /// The corridor is rebuilt for the test, which is what makes it honest:
  /// it holds the artwork, the launcher terminals and every feedline that
  /// stands, inflated by the wire clearance. A way that fails this is a way
  /// the search would refuse, and keeping it would put a crossing in the
  /// layout that nothing downstream looks at again.
  [[nodiscard]] bool edgeWayStillOpen(const std::vector<Wire>& wires,
                                      const RoutingObjective& objective,
                                      const Edge& edge, const Path& way) {
    corridorOfEdge(wires, objective, edge);
    return std::ranges::none_of(way, [this](const PathPoint& point) {
      return point.x >= scene_.router.width || point.y >= scene_.router.height ||
             corridor_.test(scene_.router.index(point.x, point.y));
    });
  }

  /// Route one feedline edge, free of any crossing rule, as the prototype
  /// routes its chain edges: the room of every wire is priced, nothing but
  /// the artwork and the couplers is in the way.
  [[nodiscard]] Path routeEdge(const std::vector<Wire>& wires,
                               const RoutingObjective& objective,
                               const Edge& edge) {
    lastPicture_.clear();
    corridorOfEdge(wires, objective, edge);
    // Nothing another wire holds is priced here. An edge is drawn against
    // the artwork and the couplers alone; every wire it passes is drawn
    // again under the feedline constraints afterwards, and the price on its
    // copper and its room only bought the edge a detour it did not need.
    // The static penalty on the artwork stays, which is what `usePenalty`
    // still carries into the search.
    //
    // The field is all zeroes, and it used to be rewritten — a byte per cell
    // of the whole grid — before every edge search. On 69q that is 29
    // million bytes a search and about 160 GB over an insertion, to say each
    // time what it said before. A field that is always zero is filled once.
    if (zeroProximity_.size() != proximity_.size()) {
      zeroProximity_.assign(proximity_.size(), 0);
    }
    
    // The approaches of every port used to cost the most a cell can, because
    // a feedline beside a port leaves the wire into it no way to cross at a
    // right angle. That is the crossing rule of the feedline phase, and the
    // insertion does not carry it: the corridor already keeps an edge out of
    // an approach, and nothing here prices an angle.
    const auto& chain = chains_[edge.chain];
    const auto [startRun, endRun] = runsOfEdge(edge);
    router_.setParams({.startStraightLength = startRun,
                       .endStraightLength = endRun,
                       .minRadius = BEND_RADIUS,
                       .bendPenalty = edgeBendPenalty()});
    router_.setSingleCrossingFeedline(nullptr, 0, 1);
    router_.attachCorridor(&corridor_);
    router_.attachWireProximity(&zeroProximity_);
    // The turns this way may make and still let its prefix win, when the
    // chain search said: see `chainStepBudget`.
    if (stepBudget_ != routing::TRELLIS_UNREACHABLE) {
      router_.setMaxTurns(static_cast<std::int16_t>(
          std::min<std::uint64_t>(stepBudget_ / 10000ULL, 255ULL)));
    }
    auto found = router_.route(objective, true);
    lastCutOff_ = stepBudget_ != routing::TRELLIS_UNREACHABLE &&
                  found.empty() && router_.lastSearchCutOff();
    budgetCutoffs_ += lastCutOff_ ? 1 : 0;
    router_.setMaxTurns(-1);
    // One picture per search, as the outer routing draws one: what the
    // corridor left open, what the search paid for and the way it took. The
    // ones that found nothing are the point of it, but a way that came out
    // crooked is only readable beside the ground it was found on.
    if (debug_) {
      Wire pretend;
      pretend.objective = objective;
      pretend.feedline = true;
      pretend.slot = static_cast<std::uint32_t>(edge.from);
      frame_.wires = &wires;
      frame_.pass = "edge";
      frame_.round = edge.chain;
      frame_.forward = true;
      frame_.kind = std::format("c{}-{}to{}", edge.chain, edge.from, edge.to);
      frame_.fence.clear();
      frame_.ripped.clear();
      frame_.before = NO_OWNER;
      frame_.after = NO_OWNER;
      drawSearch(pretend, found,
                 std::format("chain {} {}->{}{}", edge.chain, edge.from,
                             edge.to, found.empty() ? ", no way" : ""));
    }
    if (found.empty() && verbosity_ >= 1) {
      const auto describe = [&](const PathPoint& point, const bool isTarget) {
        const auto moved = router_.sanitize(point, isTarget);
        const bool inGrid =
            moved.x < scene_.router.width && moved.y < scene_.router.height;
        const auto cell = inGrid ? scene_.router.index(moved.x, moved.y) : 0;
        return std::format(
            "({}, {}) heading {} -> ({}, {}){}", point.x, point.y,
            point.heading, moved.x, moved.y,
            !inGrid ? " off the grid"
            : corridor_.test(cell)
                ? " outside the corridor"
                : (scene_.blocked.test(cell) ? " on artwork" : " free"));
      };
      std::string ways;
      for (const auto at : {edge.from, edge.to}) {
        const auto& point = chain[at];
        if (point.fixed) {
          continue;
        }
        const auto& coupler = couplers_[point.coupler];
        const auto& option = coupler.options[coupler.chosen];
        const auto& wire = wires[coupler.wire];
        ways += std::format(
            " | waypoint {} way ({},{})..({},{}) {} cells, outer "
            "({},{})..({},{}) {} cells, target ({},{})",
            at, option.way.front().x, option.way.front().y, option.way.back().x,
            option.way.back().y, option.way.size(), coupler.outerWay.front().x,
            coupler.outerWay.front().y, coupler.outerWay.back().x,
            coupler.outerWay.back().y, coupler.outerWay.size(),
            wire.objective.target.x, wire.objective.target.y);
      }
      tell(std::format(
          "edge of chain {} from {} to {}: no way; source {}, target {}{}",
          edge.chain, edge.from, edge.to, describe(objective.source, false),
          describe(objective.target, true), ways));
    }
    return found;
  }

  /// What an edge's way costs the chain search: ten thousand an eighth
  /// turn, as it always did, and `SCPD_CHAIN_LENGTH_WEIGHT` per cell of
  /// the way, **0** by default. The idea (user, 2026-10-05) was that the
  /// length tells two runs with the same turns apart, where the exact
  /// search keeps whichever it reached first. The bounds (`turnBound`, the
  /// analytic one) count turns alone and stay admissible, but they then
  /// sit below the true cost by the whole length, and an A* whose bound is
  /// loose prunes nothing: **measured at weight 1** (`artifacts/logs/length`
  /// against `squeeze-reject2`), 17q's chain 0 expanded 252 prefixes
  /// instead of 43 and the insertion took 20.4 s instead of 8.7 for `bad`
  /// 4 → 3; on 69q the insertion took 276 s instead of 88, four chains
  /// that had settled ran out of their clock, the angle cost went 180 →
  /// 246, one edge was not drawn and `bad` went 1 → 2. The switch stays
  /// for a bound that counts length too; at 0 the objective is as it was.
  [[nodiscard]] static std::uint64_t lengthWeight() {
    static const auto weight = static_cast<std::uint64_t>(
        std::clamp(envWhole("SCPD_CHAIN_LENGTH_WEIGHT", 0), 0, 1000));
    return weight;
  }
  [[nodiscard]] std::uint64_t wayCostOf(const Path& way,
                                        const Heading targetHeading) const {
    if (way.empty()) {
      return 0;
    }
    return (10000ULL * angleCostOf(way, targetHeading)) +
           (lengthWeight() * static_cast<std::uint64_t>(way.size()));
  }

  /// How much a way turns, in eighths, its arrival at the target counted:
  /// the prototype's `compute_segment_angle_cost`, which is what its coupler
  /// optimizer minimizes.
  [[nodiscard]] std::uint32_t angleCostOf(const Path& way,
                                          const Heading target) const {
    if (way.empty()) {
      return 0;
    }
    const auto segmented = routing::reconstructSegments(*primitives_, way);
    std::uint32_t cost = 0;
    Heading last = way.front().heading;
    bool first = true;
    for (const auto& segment : segmented.segments) {
      if (!first) {
        cost += routing::headingDistance(last, segment.heading);
      }
      first = false;
      last = segment.heading;
    }
    cost += routing::headingDistance(last, target);
    return cost;
  }

  /// The fewest eighth turns any way from one pose to another can make —
  /// a lower bound on `angleCostOf`, answered without routing anything.
  ///
  /// `angleCostOf` is the length of a walk on the ring of eight headings:
  /// it starts at the heading the way leaves the source on, steps once per
  /// straight run, and ends by paying the difference to the target heading.
  /// The stubs the router is given pin both ends of that walk, so it runs
  /// from the source heading to the target heading whatever the way does in
  /// between.
  ///
  /// A way's straight runs add up to the displacement between the two poses.
  /// Dotted against the beeline — the heading whose step points most nearly
  /// along that displacement — the sum is positive, so at least one run must
  /// point within an eighth of the beeline. The walk therefore passes
  /// through one of the beeline's three nearest headings, and it can be no
  /// shorter than the best of those three detours. Obstacles only lengthen
  /// it, so this holds for whatever way the search comes back with — even
  /// though the search minimises length and bends together rather than
  /// bends alone.
  ///
  /// It is **not** the prototype's `estimate_edge_angle_cost_heuristic`,
  /// which pins the walk to the beeline itself and adds two where the poses
  /// are close. That is the tighter figure and it is used there to throw
  /// candidates away before routing them, where guessing high costs nothing.
  /// Here a figure that is too high would lose the answer.
  [[nodiscard]] static std::uint32_t turnBound(const PathPoint& from,
                                               const PathPoint& to) {
    const auto dx = static_cast<std::int64_t>(to.x) -
                    static_cast<std::int64_t>(from.x);
    const auto dy = static_cast<std::int64_t>(to.y) -
                    static_cast<std::int64_t>(from.y);
    if (dx == 0 && dy == 0) {
      return routing::headingDistance(from.heading, to.heading);
    }
    routing::Heading beeline = 0;
    std::int64_t most = std::numeric_limits<std::int64_t>::min();
    for (routing::Heading h = 0; h < routing::NUM_HEADINGS; ++h) {
      const auto v = routing::headingVector(h);
      const auto dot = (static_cast<std::int64_t>(v.dx) * dx) +
                       (static_cast<std::int64_t>(v.dy) * dy);
      if (dot > most) {
        most = dot;
        beeline = h;
      }
    }
    auto best = routing::NUM_HEADINGS;
    for (int eighth = -1; eighth <= 1; ++eighth) {
      const auto through = routing::turned(beeline, eighth);
      const auto detour = routing::headingDistance(from.heading, through) +
                          routing::headingDistance(through, to.heading);
      best = detour < best ? detour : best;
    }
    return best;
  }

  /// What one option of a coupler costs: its two edges routed, each priced
  /// by how much it turns, plus their lengths and the difference between
  /// them, plus a penalty for a second dogleg. An option whose edge finds
  /// no way, or whose body meets a neighbour's, costs everything.
  [[nodiscard]] std::uint32_t localCost(const std::vector<Wire>& wires,
                                        const std::uint32_t chain,
                                        const std::size_t at,
                                        const std::size_t option,
                                        Path* firstOut = nullptr,
                                        Path* secondOut = nullptr,
                                        std::uint32_t* lostOut = nullptr) {

    constexpr auto INFINITE = std::numeric_limits<std::uint32_t>::max();
    auto& points = chains_[chain];
    auto& coupler = couplers_[points[at].coupler];
    const auto kept = coupler.chosen;
    coupler.chosen = option;
    const auto& candidate = coupler.options[option];
    std::uint32_t cost = 0;
    std::uint32_t before = 0;
    std::uint32_t after = 0;
    Path first;
    const auto overlaps = [&](const Waypoint& other) {
      if (other.fixed) {
        return false;
      }
      const auto& body = couplers_[other.coupler]
                             .options[couplers_[other.coupler].chosen]
                             .body;
      std::unordered_set<std::size_t> cells(body.begin(), body.end());
      return std::ranges::any_of(candidate.body, [&](const std::size_t cell) {
        return cells.contains(cell);
      });
    };
    const auto restore = [&]() { coupler.chosen = kept; };
    // How many of the two edges found no way. Kept apart from the cost on
    // purpose: an edge that finds nothing is an empty way, and an empty way
    // turns nowhere — `angleCostOf` scores it zero, the cheapest an edge can
    // be. Weighed on one number, an option that loses an edge beats every
    // option that keeps one, and the chain ships with a coupler nothing
    // reaches. The greedy compares the pair instead.
    std::uint32_t lost = 0;
    const auto report = [&](const std::uint32_t value) {
      if (lostOut != nullptr) {
        *lostOut = lost;
      }
      return value;
    };
    const bool hasBefore = at > 0;
    const bool hasAfter = at + 1 < points.size();
    if ((hasBefore && overlaps(points[at - 1])) ||
        (hasAfter && overlaps(points[at + 1]))) {
      restore();
      lost = 2;
      return report(INFINITE);
    }

    const auto edgeOf = [&](const std::size_t from, const std::size_t to) {
      return Edge{.chain = chain,
                  .from = from,
                  .to = to,
                  .terminal = points[from].fixed || points[to].fixed};
    };
    const auto objectiveOf = [&](const std::size_t from, const std::size_t to) {
      return RoutingObjective{.source = sourceOf(points[from]),
                              .target = targetOf(points[to])};
    };

    // One edge of this coupler, routed on its own.
    //
    // It used to take the sibling edge's freshly found way and fence it —
    // copper two cells wide, the clearance on top, the clearance dropped
    // again when that left no way. None of it ever reached the search:
    // `corridorOfEdge` discarded the cells it was handed. What actually
    // keeps the two edges of one coupler apart is `fenceCommittedEdges`,
    // which closes the sibling's last chosen way like every other edge on
    // the chip. So the price of an edge is a function of the two options at
    // its ends and the frozen state of every other edge — never of the
    // order the two edges of one coupler are routed in.
    const auto routeOne = [&](const std::size_t from, const std::size_t to) {
      return routeEdge(wires, objectiveOf(from, to), edgeOf(from, to));
    };

    // Both edges of this coupler. Neither sees the other, so there is no
    // order to choose: the two searches were tried both ways round for a
    // long time and returned the same two paths every time, because the
    // corridor each is built in does not depend on the other.
    struct Attempt {
      Path first;
      Path second;
      std::uint32_t lost = 0;
      std::uint32_t cost = 0;
    };
    const auto routeBoth = [&]() {
      Attempt out;
      if (hasBefore) {
        out.first = routeOne(at - 1, at);
      }
      if (hasAfter) {
        out.second = routeOne(at, at + 1);
      }
      if (hasBefore) {
        out.lost += out.first.empty() ? 1 : 0;
        out.cost += wayCostOf(out.first, objectiveOf(at - 1, at).target.heading);
      }
      if (hasAfter) {
        out.lost += out.second.empty() ? 1 : 0;
        out.cost += wayCostOf(out.second, objectiveOf(at, at + 1).target.heading);
      }
      return out;
    };

    // Both edges already known for this pair of options: nothing to route.
    // This is where the greedy stops repeating itself — it weighs the same
    // pair once per pass and per neighbour, and the answer cannot have
    // changed while neither coupler moved.
    const auto remembered = [&](const std::size_t from, Path& into) {
      const auto found = edgeMemo_.find(edgeMemoKey(chain, from));
      if (found == edgeMemo_.end()) {
        return false;
      }
      into = found->second;
      return true;
    };
    {
      Attempt known;
      bool all = true;
      if (hasBefore) {
        all = all && remembered(at - 1, known.first);
      }
      if (hasAfter) {
        all = all && remembered(at, known.second);
      }
      if (all) {
        ++memoHits_;
        if (hasBefore) {
          known.lost += known.first.empty() ? 1 : 0;
          known.cost += wayCostOf(known.first,
                                  objectiveOf(at - 1, at).target.heading);
        }
        if (hasAfter) {
          known.lost += known.second.empty() ? 1 : 0;
          known.cost += wayCostOf(known.second,
                                  objectiveOf(at, at + 1).target.heading);
        }
        first = std::move(known.first);
        lost = known.lost;
        cost = known.cost;
        before = static_cast<std::uint32_t>(first.size());
        after = static_cast<std::uint32_t>(known.second.size());
        if (secondOut != nullptr) {
          *secondOut = known.second;
        }
        if (firstOut != nullptr) {
          *firstOut = first;
        }
        restore();
        return report(cost);
      }
    }
    ++memoMisses_;

    auto taken = routeBoth();
    if (hasBefore) {
      edgeMemo_[edgeMemoKey(chain, at - 1)] = taken.first;
    }
    if (hasAfter) {
      edgeMemo_[edgeMemoKey(chain, at)] = taken.second;
    }
    first = std::move(taken.first);
    lost = taken.lost;
    cost = taken.cost;
    before = static_cast<std::uint32_t>(first.size());
    after = static_cast<std::uint32_t>(taken.second.size());
    if (secondOut != nullptr) {
      *secondOut = taken.second;
    }
    if (firstOut != nullptr) {
      *firstOut = first;
    }
    // cost += before + after;
    // cost += before > after ? before - after : after - before;
    // cost += 100 * candidate.guarded;
    restore();
    return report(cost);
  }


  /// What a cell of the head or the body costs an option, in the units the
  /// greedy multiplies by a hundred — where one turn of a feedline edge
  /// costs ten thousand.
  ///
  /// The head and the body never move again once an option is taken: the
  /// head is the resonator's fixed places and the body is an obstacle to
  /// every search after it. So whatever stands beside them is what has to
  /// give way, and it is the wire beside them that fails when it cannot.
  /// Measured on the 17-qubit chip: every pair of wires the check found too
  /// close was a resonator and its ring neighbour, within a dozen cells of
  /// that resonator's own coupler.
  ///
  /// A conventional wire costs more than a resonator under them, because a
  /// resonator is drawn again from its coupler and may meander its way
  /// around, while a conventional wire runs from a fixed port to a fixed
  /// port and has only the band around its way.
  static constexpr std::uint32_t UNDER_RESONATOR = 20;
  static constexpr std::uint32_t UNDER_CONVENTIONAL = 60;
  static constexpr std::uint32_t IN_THE_ROOM = 1;


  /// The greedy local search over one chain: the prototype's
  /// `optimize_chain`. Up to five passes; in each, every coupler of the
  /// chain takes the cheapest of its options, its own included, with the
  /// neighbours as they stand.
  ///
  /// **Every** option is routed. The prototype pre-screens them by an
  /// estimate of what their edges will turn and routes only the cheapest
  /// two dozen; here the estimate is gone, because an estimate that ranks
  /// an option it never routes can only drop the one that would have won.
  /// It costs what it costs: a coupler weighs up to sixty-four options and
  /// each is two routed edges.
  ///
  /// @returns How many passes improved something.
  [[nodiscard]] std::uint32_t optimizeChain(const std::vector<Wire>& wires,
                                            const std::uint32_t chain) {
    constexpr std::uint32_t PASSES = 5;
    constexpr auto INFINITE = std::numeric_limits<std::uint32_t>::max();
    auto& points = chains_[chain];
    std::uint32_t improved = 0;
    for (std::uint32_t pass = 0; pass < PASSES; ++pass) {
      bool moved = false;
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (points[at].fixed) {
          continue;
        }
        auto& coupler = couplers_[points[at].coupler];
        if (coupler.options.size() <= 1) {
          Path first;
          Path second;
          std::uint32_t lost = 0;
          coupler.options.front().cost =
              localCost(wires, chain, at, 0, &first, &second, &lost);
          if (at > 0) {
            chainEdgePaths_[chain][at - 1] = first;
          }
          if (at + 1 < points.size()) {
            chainEdgePaths_[chain][at] = second;
          }
          continue;
        }
        const auto current = coupler.chosen;
        auto best = current;
        Path bestFirst;
        Path bestSecond;
        // An option is eligible only when both its edges found a way.
        // One that loses an edge is discarded outright, never weighed: an
        // empty way turns nowhere, so `angleCostOf` scores it zero and it
        // would otherwise look cheaper than every option that keeps both.
        // Among the eligible ones the cheapest wins, and any eligible option
        // beats a coupler sitting on one that lost an edge.
        std::uint32_t leastLost = 0;
        auto least = localCost(wires, chain, at, current, &bestFirst,
                               &bestSecond, &leastLost);
        coupler.options[current].cost = least;
        explain(std::format(
            "[Coupler Insertion]   '{}' chain {} pass {}: holds option "
            "{}/{} -> {}",
            wireId(wires[coupler.wire]), chain, pass, current + 1,
            coupler.options.size(),
            leastLost > 0 ? std::format("{} edge(s) unroutable", leastLost)
                          : std::format("cost {}", least)));
        bool plainFits = leastLost == 0 &&
                         coupler.options[current].secondStraight == 0;
        for (std::size_t option = 0; option < coupler.options.size();
             ++option) {
          if (option == current) {
            continue;
          }
          if (coupler.options[option].secondStraight > 0 &&
              !coupler.jogsUnlocked) {
            continue;
          }
          Path first;
          Path second;
          std::uint32_t lost = 0;
          const auto cost =
              localCost(wires, chain, at, option, &first, &second, &lost);
          coupler.options[option].cost = cost;
          if (explaining()) {
            const auto& candidate = coupler.options[option];
            explain(std::format(
                "[Coupler Insertion]   try '{}' option {}/{}: angle {}° "
                "(offset {}, resonator port {}) at ({},{}) -> {}{}",
                wireId(wires[coupler.wire]), option + 1,
                coupler.options.size(),
                degreesOfHeading(candidate.couplerOrientation),
                candidate.offset, candidate.secondPort ? 2 : 1,
                candidate.centre.x, candidate.centre.y,
                lost > 0 ? std::format("{} edge(s) unroutable", lost)
                         : std::format("cost {}", cost),
                picture()));
          }
          plainFits = plainFits ||
                      (lost == 0 && coupler.options[option].secondStraight == 0);
          const bool better =
              lost == 0 &&
              (leastLost > 0 || (cost != INFINITE && cost <= least));
          if (better) {
            leastLost = lost;
            least = cost;
            best = option;
            bestFirst = std::move(first);
            bestSecond = std::move(second);
          }
        }
        // What the coupler's edges are from now on, for every edge routed
        // after them.
        if (least != INFINITE) {
          if (at > 0 && !bestFirst.empty()) {
            chainEdgePaths_[chain][at - 1] = bestFirst;
          }
          if (at + 1 < points.size() && !bestSecond.empty()) {
            chainEdgePaths_[chain][at] = bestSecond;
          }
        }
        // Not one plain option brought both edges home: open the jogs, and
        // let the next pass use them. Opening them in this pass would mean
        // routing them against a chain that is still moving under them.
        if (!plainFits && !coupler.jogsUnlocked) {
          coupler.jogsUnlocked = true;
          moved = true;
          tell(std::format(
              "[Coupler Insertion]   '{}' chain {} pass {}: no plain option "
              "keeps both edges — opening the second-dogleg options",
              wireId(wires[coupler.wire]), chain, pass));
        }
        if (best != current) {
          coupler.chosen = best;
          moved = true;
          const auto& won = coupler.options[best];
          tell(std::format(
              "[Coupler Insertion] ACCEPT '{}' chain {} pass {}: angle {}° "
              "(offset {}, resonator port {}) at ({},{}) -> cost {}",
              wireId(wires[coupler.wire]), chain, pass,
              degreesOfHeading(won.couplerOrientation), won.offset,
              won.secondPort ? 2 : 1, won.centre.x, won.centre.y, least));
        }
      }
      if (!moved) {
        break;
      }
      ++improved;
    }
    return improved;
  }

  /// What one edge of a chain costs with a named option at each of its ends,
  /// routed if it has to be. The price the greedy puts on it, for the one
  /// edge: ten thousand an eighth turn, and everything if there is no way.
  ///
  /// It stands the two couplers on the options asked about for as long as it
  /// takes to answer, because everything the question needs — the two ports,
  /// the end stub, the two resonators the corridor closes — is read off
  /// `chosen`. Both are put back before it returns.
  ///
  /// `fence` holds the ways the caller has already laid, which the corridor
  /// closes on top of everything committed. `remember` is false for a caller
  /// whose fence is a whole prefix: a way found against one prefix is worth
  /// nothing under another, and the memo key cannot tell them apart.
  [[nodiscard]] std::uint64_t
  edgeCost(const std::vector<Wire>& wires, const std::uint32_t chain,
           const std::size_t from, const std::size_t optionFrom,
           const std::size_t optionTo, const std::size_t aheadOption = 0,
           const std::vector<const Path*>* fence = nullptr,
           Path* wayOut = nullptr, const bool remember = true) {
    auto& points = chains_[chain];
    const auto stand = [&](const std::size_t at, const std::size_t option) {
      if (points[at].fixed) {
        return std::size_t{0};
      }
      auto& coupler = couplers_[points[at].coupler];
      const auto kept = coupler.chosen;
      coupler.chosen = option;
      return kept;
    };
    const auto keptFrom = stand(from, optionFrom);
    const auto keptTo = stand(from + 1, optionTo);

    const RoutingObjective objective{.source = sourceOf(points[from]),
                                     .target = targetOf(points[from + 1])};
    const Edge edge{.chain = chain,
                    .from = from,
                    .to = from + 1,
                    .terminal =
                        points[from].fixed || points[from + 1].fixed};

    const auto key = edgeMemoKey(chain, from, aheadOption);
    const auto found = remember ? edgeMemo_.find(key) : edgeMemo_.end();
    Path way;
    if (found != edgeMemo_.end()) {
      ++memoHits_;
      way = found->second;
    } else {
      ++memoMisses_;
      prefixFence_ = fence;
      way = routeEdge(wires, objective, edge);
      prefixFence_ = nullptr;
      if (remember && !lastCutOff_) {
        edgeMemo_[key] = way;
      }
    }

    stand(from, keptFrom);
    stand(from + 1, keptTo);
    if (wayOut != nullptr) {
      *wayOut = way;
    }
    if (way.empty()) {
      return routing::TRELLIS_UNREACHABLE;
    }
    return wayCostOf(way, objective.target.heading);
  }

  /// What one round of the exact search made of one chain: the options it
  /// would have it stand on, and the way it found for each of its edges.
  /// Nothing is written while a round runs — see `optimizeChainsExact`.
  struct ChainAnswer {
    bool solved = false;
    std::uint64_t cost = routing::TRELLIS_UNREACHABLE;
    std::uint32_t evaluations = 0;
    /// Of those, the ones that were not already remembered — the searches.
    std::uint32_t routed = 0;
    /// How many prefixes the prefix search grew children from. Zero on the
    /// trellis, which has no prefixes.
    std::uint32_t expansions = 0;
    /// Placeholders the search dropped unpriced against its incumbent.
    std::uint32_t pruned = 0;
    /// Whether the budget stopped the search rather than the search settling
    /// the chain — the one thing that separates a chain with no answer from
    /// one there was no time to answer.
    bool outOfTime = false;
    /// Whether the search proved this the cheapest run of options there is.
    /// False when the prefix search ran out of time, or when it reordered a
    /// pair of edges at an end of the chain; either way this is then only
    /// the cheapest run it reached.
    bool optimal = false;
    /// Whether a pair of edges at an end of the chain had to be laid the
    /// other way round. That is the reason `optimal` is false when the
    /// search did not run out of time.
    bool relaid = false;
    std::uint64_t pairs = 0;
    std::vector<std::size_t> chosen;
    std::vector<Path> ways;
  };

  /// The copper of every feedline that stands, bar one chain's: the cells a
  /// coupler body may not be laid across. See `guardTheBodies`.
  ///
  /// The chain being solved is left out because its own edges are about to
  /// be routed again — what they hold is the answer of the round before, and
  /// closing options against it would hold this round to the last one's
  /// shape. Built once per solve, not once per layer: a pass over the ways
  /// of the chip against a grid of bits.
  [[nodiscard]] grid::BitGrid feedlineCellsBut(const std::uint32_t chain) const {
    grid::BitGrid cells(scene_.router.width, scene_.router.height);
    if (!guardTheBodies()) {
      return cells;
    }
    for (std::uint32_t other = 0; other < chainEdgePaths_.size(); ++other) {
      if (other == chain) {
        continue;
      }
      for (const auto& way : chainEdgePaths_[other]) {
        for (const auto& point : way) {
          if (point.x < scene_.router.width &&
              point.y < scene_.router.height) {
            cells.set(scene_.router.index(point.x, point.y), true);
          }
        }
      }
    }
    return cells;
  }

  /// Which options a waypoint offers a search: a launcher offers the one it
  /// is, a coupler its plain options, and its jogs only once it has shown it
  /// needs them.
  ///
  /// And, where `feedlines` is given, only the options whose body is clear
  /// of every feedline that stands — `guardTheBodies` says why. Where that
  /// leaves nothing the coupler keeps all of them: a layer with no node has
  /// no answer, and the chain would be lost rather than moved.
  [[nodiscard]] std::vector<std::uint32_t>
  openOptionsOf(const std::uint32_t chain, const std::size_t at,
                const grid::BitGrid* feedlines = nullptr) const {
    std::vector<std::uint32_t> open;
    const auto& point = chains_[chain][at];
    if (point.fixed) {
      open.push_back(0);
      return open;
    }
    const auto& coupler = couplers_[point.coupler];
    std::vector<std::uint32_t> all;
    for (std::size_t option = 0; option < coupler.options.size(); ++option) {
      if (coupler.options[option].secondStraight > 0 &&
          !coupler.jogsUnlocked) {
        continue;
      }
      all.push_back(static_cast<std::uint32_t>(option));
      if (feedlines != nullptr &&
          std::ranges::any_of(coupler.options[option].body,
                              [&](const std::size_t cell) {
                                return feedlines->test(cell);
                              })) {
        continue;
      }
      open.push_back(static_cast<std::uint32_t>(option));
    }
    if (open.empty() && !all.empty()) {
      tell(std::format("[Coupler Insertion]   chain {} waypoint {}: every "
                       "option lies on a feedline; the guard stands down",
                       chain, at));
      return all;
    }
    if (feedlines != nullptr && open.size() != all.size()) {
      tell(std::format("[Coupler Insertion]   chain {} waypoint {}: {} of {} "
                       "options closed, their body on a feedline",
                       chain, at, all.size() - open.size(), all.size()));
    }
    return open;
  }

  /// The port an edge leaves a waypoint by, or arrives at it by, with the
  /// coupler standing there on a named option. A launcher has one port each
  /// way whatever is asked.
  [[nodiscard]] PathPoint portOfOption(const std::uint32_t chain,
                                       const std::size_t at,
                                       const std::size_t option,
                                       const bool leaving) const {
    const auto& point = chains_[chain][at];
    if (point.fixed) {
      return leaving ? point.asSource : point.asTarget;
    }
    const auto& chosen = couplers_[point.coupler].options[option];
    return leaving ? chosen.out : chosen.in;
  }

  /// The cheapest run of coupler options along one chain, exactly, against
  /// the chip as it stands — one round of the layered search that replaces
  /// the greedy's coordinate descent.
  ///
  /// A chain is a trellis: a layer per waypoint, a node per option of the
  /// coupler standing there, and the step between two of them one routed
  /// feedline edge. The price of a step is what its two options make of it
  /// and nothing else — every other thing `corridorOfEdge` closes is either
  /// the same for the whole chip or read off those two options — with one
  /// exception, `fenceCommittedEdges`, which closes every other edge on the
  /// chip as it currently stands. Freeze that and the trellis is exact.
  ///
  /// `routing::solveTrellis` prices a step only when the answer turns on it,
  /// and the bound is what lets it leave the rest alone — so how sharp the
  /// bound is *is* the runtime. `boundTurns` picks it; the analytic answer is
  /// the default and prices a fraction of the pairs `turnBound` does.
  [[nodiscard]] ChainAnswer solveChain(const std::vector<Wire>& wires,
                                       const std::uint32_t chain) {
    ChainAnswer out;
    const auto routedBefore = memoMisses_;
    auto& points = chains_[chain];
    const auto layers = points.size();
    if (layers < 2) {
      return out;
    }

    std::vector<std::vector<std::uint32_t>> open;
    routing::TrellisResult answer;
    // Whether a node of the search carries the option of the waypoint before
    // it as well as its own. Settled per go, because widening the options is
    // what can put it out of reach.
    bool second = false;
    {
      // Two goes at most: the plain options, then the jogs opened at the
      // wall if the plain ones did not reach.
      const auto feedlines = feedlineCellsBut(chain);
      for (int go = 0; go < 2; ++go) {
        open.clear();
        for (std::size_t at = 0; at < layers; ++at) {
          open.push_back(openOptionsOf(chain, at, &feedlines));
        }
        const auto portOf = [&](const std::size_t at, const std::uint32_t node,
                                const bool leaving) {
          return portOfOption(chain, at, open[at][node], leaving);
        };

        // **Second order**: a node is the option at its own waypoint *and*
        // the one at the waypoint before it. That is what lets a step fence
        // the way its own predecessor takes — without it the two edges that
        // meet at a coupler never see each other inside a solve, the trellis
        // prices a pair that cannot both be built as though it could, and
        // the commit is where one of them is lost.
        //
        // It costs width: a layer of `n` options becomes one of `n x n`, and
        // the step matrix between two layers `n^2 x n^2`, of which only the
        // steps whose two nodes agree about the option they share are steps
        // at all. Every chain pays it — giving it up on the wide ones was
        // measured giving up exactly the chains that needed it.
        // `SCPD_CHAIN_ORDER=1` turns it off everywhere.
        second = chainOrder() >= 2;

        // How many nodes each layer holds, and how to read one back.
        std::vector<std::size_t> wide(layers);
        wide[0] = open[0].size();
        for (std::size_t at = 1; at < layers; ++at) {
          wide[at] =
              second ? open[at - 1].size() * open[at].size() : open[at].size();
        }
        constexpr auto NO_AHEAD = std::numeric_limits<std::size_t>::max();
        const auto hereOf = [&](const std::size_t at, const std::size_t node) {
          return (second && at > 0) ? node % open[at].size() : node;
        };
        const auto aheadOf = [&](const std::size_t at, const std::size_t node) {
          return (second && at > 0) ? node / open[at].size() : NO_AHEAD;
        };

        routing::TrellisProblem problem;
        for (const auto nodes : wide) {
          problem.width.push_back(static_cast<std::uint32_t>(nodes));
        }

        // The layer graph, built before the search rather than during it.
        // Every step of it is priced by the bound, which answers in a
        // fraction of a microsecond against the tens of milliseconds a real
        // edge search costs, so the whole graph is affordable where even one
        // extra search is not. This is the only place the bound is asked —
        // `solveTrellis` fills its prices from it once — so the time spent
        // here is the whole of what the bound costs.
        const auto beganBound = std::chrono::steady_clock::now();
        // The analytic answer depends on the pair of *options* and nothing
        // else, so it is asked once per option pair and the lifted graph is
        // filled from that: raising the order multiplies the nodes, never
        // the geometry.
        std::vector<std::vector<std::uint64_t>> turns(layers - 1);
        for (std::size_t at = 0; at + 1 < layers; ++at) {
          const auto n = open[at].size();
          const auto m = open[at + 1].size();
          turns[at].resize(n * m);
          for (std::size_t c = 0; c < n; ++c) {
            const auto leaving = portOf(at, static_cast<std::uint32_t>(c), true);
            for (std::size_t d = 0; d < m; ++d) {
              const auto arriving =
                  portOf(at + 1, static_cast<std::uint32_t>(d), false);
              turns[at][(c * m) + d] = 10000ULL * boundTurns(leaving, arriving);
            }
          }
          boundPairs_ += n * m;
        }
        boundNanos_ += static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - beganBound)
                .count());

        // A step of the lifted graph is a step only when the node it lands
        // on agrees about where it came from; everything else is not a step
        // at all and carries the price of one that cannot be built.
        std::vector<std::vector<std::uint64_t>> bounds(layers - 1);
        for (std::size_t at = 0; at + 1 < layers; ++at) {
          bounds[at].assign(wide[at] * wide[at + 1],
                            routing::TRELLIS_UNREACHABLE);
          const auto m = open[at + 1].size();
          for (std::size_t i = 0; i < wide[at]; ++i) {
            const auto c = hereOf(at, i);
            for (std::size_t j = 0; j < wide[at + 1]; ++j) {
              if (second && aheadOf(at + 1, j) != c) {
                continue;
              }
              bounds[at][(i * wide[at + 1]) + j] =
                  turns[at][(c * m) + hereOf(at + 1, j)];
            }
          }
        }

        problem.bound = [&](const std::size_t at, const std::uint32_t i,
                            const std::uint32_t j) {
          return bounds[at][(static_cast<std::size_t>(i) * wide[at + 1]) + j];
        };
        problem.evaluate = [&](const std::size_t at, const std::uint32_t i,
                               const std::uint32_t j) {
          const auto c = hereOf(at, i);
          const auto d = hereOf(at + 1, j);
          if (second && aheadOf(at + 1, j) != c) {
            return routing::TRELLIS_UNREACHABLE;
          }
          // The way the predecessor takes, priced **first order**. Carrying
          // its own fence back as well would make a node the whole prefix of
          // the chain, which is the exponential the trellis exists to avoid;
          // one step back is what buys the pair that matters — the two edges
          // that share a coupler — at a width that can still be solved.
          Path ahead;
          std::size_t aheadCode = 0;
          if (const auto p = aheadOf(at, i); p != NO_AHEAD) {
            static_cast<void>(edgeCost(wires, chain, at - 1, open[at - 1][p],
                                       open[at][c], 0, nullptr, &ahead));
            aheadCode = points[at - 1].fixed ? 0 : open[at - 1][p] + 1;
          }
          const std::vector<const Path*> laid{&ahead};
          const auto real =
              edgeCost(wires, chain, at, open[at][c], open[at + 1][d],
                       aheadCode, ahead.empty() ? nullptr : &laid);
          if (chainBound() >= 2) {
            audit(chain, at, portOf(at, static_cast<std::uint32_t>(c), true),
                  portOf(at + 1, static_cast<std::uint32_t>(d), false), real);
          }
          return real;
        };
        answer = routing::solveTrellis(problem);
        if (answer.solved) {
          break;
        }
        // The wall: the first layer nothing reaches going forward, and the
        // last nothing reaches coming back. The step that shut the chain
        // runs between the two couplers there, so those are the ones whose
        // choice is widened — never the whole chain, which trebles what
        // every round has to route and was measured worse.
        bool widened = false;
        const auto unlock = [&](const std::size_t at) {
          if (points[at].fixed) {
            return;
          }
          auto& coupler = couplers_[points[at].coupler];
          if (coupler.jogsUnlocked) {
            return;
          }
          coupler.jogsUnlocked = true;
          widened = true;
          tell(std::format(
              "[Coupler Insertion]   '{}' chain {}: the chain does not reach "
              "past it on plain options — opening the second-dogleg options",
              wireId(wires[coupler.wire]), chain));
        };
        for (std::size_t at = 0; at < layers; ++at) {
          if (!answer.reachedFromStart[at]) {
            unlock(at);
            if (at > 0) {
              unlock(at - 1);
            }
            break;
          }
        }
        for (std::size_t at = layers; at > 0; --at) {
          if (!answer.reachedFromEnd[at - 1]) {
            unlock(at - 1);
            if (at < layers) {
              unlock(at);
            }
            break;
          }
        }
        if (!widened) {
          break;
        }
      }
    }

    out.evaluations = answer.evaluations;
    out.routed = static_cast<std::uint32_t>(memoMisses_ - routedBefore);
    for (std::size_t at = 0; at + 1 < layers; ++at) {
      out.pairs +=
          static_cast<std::uint64_t>(open[at].size()) * open[at + 1].size();
    }
    if (!answer.solved) {
      return out;
    }
    out.solved = true;
    out.optimal = true;
    out.cost = answer.cost;
    out.chosen.assign(layers, 0);
    for (std::size_t at = 0; at < layers; ++at) {
      // A second-order node carries its predecessor as well; the option of
      // this waypoint is the low half of it.
      const auto node = static_cast<std::size_t>(answer.chosen[at]);
      const auto here =
          (second && at > 0) ? node % open[at].size() : node;
      out.chosen[at] = points[at].fixed ? 0 : open[at][here];
    }
    // Every step of the winner was priced, so every one of its ways is in
    // the memo of this round. Read them out rather than route them again —
    // under the predecessor the winner stands on, which is the key the way
    // was remembered under.
    out.ways.assign(layers - 1, Path{});
    const auto kept = standOn(chain, out.chosen);
    for (std::size_t at = 0; at + 1 < layers; ++at) {
      const auto ahead = (second && at > 0) ? optionAt(chain, at - 1) : 0;
      const auto found = edgeMemo_.find(edgeMemoKey(chain, at, ahead));
      if (found != edgeMemo_.end()) {
        out.ways[at] = found->second;
      }
    }
    static_cast<void>(standOn(chain, kept));
    return out;
  }

  /// The cheapest run of coupler options along one chain, exactly, with
  /// every edge routed against the ways the options before it have laid.
  ///
  /// `solveChain` above is far cheaper and rests on an assumption that is
  /// false here: that the price of a step turns on the option at each of its
  /// two ends and on nothing else. Two feedline edges that meet at a coupler
  /// block each other, so the trellis prices a pair that cannot both be
  /// built as though it could, `stateFaults` counts the difference, and the
  /// commit is where an edge is lost. Carrying the predecessor in the node
  /// closes that for the two edges that **share** a coupler — on 17q six
  /// undrawn edges to none — and it does not reach 57q and 69q, where what
  /// is lost is blocked by edges that are not neighbours. No width of node
  /// reaches those; only pricing a step against the whole prefix does.
  ///
  /// So a node here is a **prefix**, and `routing::solveChainAStar` walks
  /// the prefixes best-first. Its heuristic is the cheapest run of analytic
  /// bounds from a layer to the end of the chain — the same relaxation the
  /// trellis fills its price array from, run backwards. It never exceeds
  /// what a real edge costs under any prefix, so the answer is the true
  /// optimum against a fence that is real.
  ///
  /// **Nothing is remembered.** A way is worth only what the prefix it was
  /// routed against makes it worth, and with no merging no prefix comes
  /// round twice. `edgeMemo_` stays for the trellis.
  /// What one step of one prefix of the chain search laid.
  struct Laid {
    /// The way of the edge into the prefix's last layer.
    Path way;
    /// A way for the edge **before** it, when the two would only both fit
    /// with this one laid first — see the reorder in `solveChainAStar`'s
    /// `step`. Empty otherwise. It hangs off this key and not the shorter one
    /// because the shorter prefix is shared by every choice that extends it,
    /// and only this choice needed its predecessor moved.
    Path before;
  };

  /// The ways of a complete run of choices, read out of what the search laid
  /// under the prefix each edge was priced under — which is the run itself.
  /// Where a pair at an end of the chain was reordered, the earlier edge's
  /// way is the replacement the later one left behind. One way per edge,
  /// empty where nothing was laid.
  [[nodiscard]] static std::vector<Path>
  waysOfRun(const std::map<std::vector<std::uint32_t>, Laid>& laid,
            const std::span<const std::uint32_t> run) {
    std::vector<Path> ways(run.empty() ? 0 : run.size() - 1);
    const std::vector<std::uint32_t> whole(run.begin(), run.end());
    const auto upto = [&](const std::size_t take) {
      return std::vector<std::uint32_t>(
          whole.begin(), whole.begin() + static_cast<std::ptrdiff_t>(take));
    };
    for (std::size_t at = 0; at + 1 < whole.size(); ++at) {
      if (at + 3 <= whole.size()) {
        const auto moved = laid.find(upto(at + 3));
        if (moved != laid.end() && !moved->second.before.empty()) {
          ways[at] = moved->second.before;
          continue;
        }
      }
      const auto found = laid.find(upto(at + 2));
      if (found != laid.end()) {
        ways[at] = found->second.way;
      }
    }
    return ways;
  }

  [[nodiscard]] ChainAnswer solveChainAStar(const std::vector<Wire>& wires,
                                            const std::uint32_t chain) {
    ChainAnswer out;
    const auto routedBefore = memoMisses_;
    auto& points = chains_[chain];
    const auto layers = points.size();
    if (layers < 2) {
      return out;
    }

    std::vector<std::vector<std::uint32_t>> open;
    routing::ChainSolution answer;
    // What every step the search has priced laid (`Laid`), by the prefix it
    // was priced under: the key is the run of node indices from layer 0.
    // This is what `step` reads a prefix's fence out of, and it is the one
    // thing a node of the search needs that the search itself knows nothing
    // about.
    //
    // `std::map` for two reasons: the key is a vector, and its values never
    // move, so the fence may hold pointers into it.
    std::map<std::vector<std::uint32_t>, Laid> laid;

    // The capacity check of every step (`capacityStep`): the verdict on the
    // chip each priced step leaves, under the same key as `laid`, with the
    // wires it closes against the prefix before it; and the verdict on the
    // chip with none of this chain standing, which a first edge is compared
    // with. The feedline edges are named as the commit names them.
    struct StepCheck {
      CapacityVerdict verdict;
      std::vector<std::uint32_t> closed;
    };
    std::map<std::vector<std::uint32_t>, StepCheck> checked;
    CapacityVerdict unchained;
    std::uint32_t stepChecks = 0;
    std::uint32_t stepsCarrying = 0;
    std::uint32_t stepsClosing = 0;
    std::uint32_t stepsRefused = 0;
    double stepSeconds = 0.0;
    std::uint32_t firstSlot = 0;
    for (std::uint32_t before = 0; before < chain; ++before) {
      firstSlot += chains_[before].empty()
                       ? 0U
                       : static_cast<std::uint32_t>(chains_[before].size() - 1);
    }
    if (stepSettled_ != nullptr) {
      unchained = capacityVerdictOf(stateOf(wires, *stepSettled_));
    }

    // Two goes at most: the plain options, then the jogs opened at the wall
    // if the plain ones did not reach.
    const auto feedlines = feedlineCellsBut(chain);
    for (int go = 0; go < 2; ++go) {
      open.clear();
      for (std::size_t at = 0; at < layers; ++at) {
        open.push_back(openOptionsOf(chain, at, &feedlines));
      }
      laid.clear();
      checked.clear();

      // The bounds, built before the search as the trellis builds its price
      // array. The analytic answer depends on the pair of options and
      // nothing else, so the whole graph costs microseconds against the tens
      // of milliseconds one real edge search costs.
      const auto beganBound = std::chrono::steady_clock::now();
      std::vector<std::vector<std::uint64_t>> turns(layers - 1);
      for (std::size_t at = 0; at + 1 < layers; ++at) {
        const auto n = open[at].size();
        const auto m = open[at + 1].size();
        turns[at].resize(n * m);
        for (std::size_t c = 0; c < n; ++c) {
          const auto leaving = portOfOption(chain, at, open[at][c], true);
          for (std::size_t d = 0; d < m; ++d) {
            const auto arriving =
                portOfOption(chain, at + 1, open[at + 1][d], false);
            turns[at][(c * m) + d] = 10000ULL * boundTurns(leaving, arriving);
          }
        }
        boundPairs_ += n * m;
      }
      boundNanos_ += static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - beganBound)
              .count());

      routing::ChainProblem problem;
      for (const auto& choices : open) {
        problem.width.push_back(static_cast<std::uint32_t>(choices.size()));
      }
      // The cheapest a pair of options has cost in this solve, by layer and
      // pair: see `chainLearnedBound`.
      std::unordered_map<std::uint64_t, std::uint64_t> learned;
      const auto pairKey = [](const std::size_t at, const std::uint32_t i,
                              const std::uint32_t j) {
        return (static_cast<std::uint64_t>(at) << 40U) |
               (static_cast<std::uint64_t>(i) << 20U) | j;
      };
      problem.bound = [&](const std::size_t at, const std::uint32_t i,
                          const std::uint32_t j) {
        auto bound =
            turns[at][(static_cast<std::size_t>(i) * open[at + 1].size()) + j];
        if (chainLearnedBound()) {
          const auto known = learned.find(pairKey(at, i, j));
          if (known != learned.end() && known->second > bound) {
            ++learnedRaised_;
            bound = known->second;
          }
        }
        return bound;
      };
      problem.budget = chainAStarBudget();
      // Every time a run of options comes back with all of its feedline
      // edges drawn. This is what the budget falls back on, so say when one
      // arrives and what it costs: a chain that never prints this line is
      // one the search never joined.
      problem.found = [&](const std::span<const std::uint32_t> run,
                          const std::uint64_t cost) {
        std::string options;
        for (std::size_t at = 0; at < run.size(); ++at) {
          options += std::format("{}{}", at == 0 ? "" : " ",
                                 points[at].fixed
                                     ? std::string("launcher")
                                     : std::to_string(open[at][run[at]]));
        }
        tell(std::format("[Coupler Insertion]   chain {}: every feedline edge "
                         "drawn on options {} -> cost {}",
                         chain, options, cost));
      };
      problem.step = [&](const std::span<const std::uint32_t> prefix,
                         const std::uint32_t j, std::int64_t& redone) {
        redone = 0;
        const auto at = prefix.size() - 1;
        const auto edges = layers - 1;
        std::vector<std::uint32_t> key(prefix.begin(), prefix.end());
        const auto upto = [&](const std::size_t take) {
          return std::vector<std::uint32_t>(
              key.begin(), key.begin() + static_cast<std::ptrdiff_t>(take));
        };
        // The way edge `e` of this prefix stands on. Every edge of a prefix
        // has one: the search prices a prefix's own step before it ever
        // grows a child from it. A pair that was reordered left the
        // replacement for its earlier edge on the later one's key, so that
        // is what counts where it exists.
        const auto wayOfEdge = [&](const std::size_t e) -> const Path* {
          if (e + 3 <= key.size()) {
            const auto moved = laid.find(upto(e + 3));
            if (moved != laid.end() && !moved->second.before.empty()) {
              return &moved->second.before;
            }
          }
          const auto found = laid.find(upto(e + 2));
          return (found != laid.end() && !found->second.way.empty())
                     ? &found->second.way
                     : nullptr;
        };
        std::vector<const Path*> fence;
        fence.reserve(at);
        for (std::size_t e = 0; e < at; ++e) {
          // The chain's first edge is an obstacle to the edge beside it and
          // to nothing further along — `terminalEdgesFenceAll`. The last
          // edge cannot appear here at all: this fence holds the edges
          // *before* the one being priced.
          //
          // Unless the edge being priced is itself a terminal one, which
          // sees its whole chain whatever its distance from it.
          const bool drawingATerminal = at == 0 || at + 1 == edges;
          if (e == 0 && at != 1 && !drawingATerminal &&
              !terminalEdgesFenceAll()) {
            continue;
          }
          if (const Path* laidWay = wayOfEdge(e); laidWay != nullptr) {
            fence.push_back(laidWay);
          }
        }

        // Whether this edge is routed again for every prefix, or remembered
        // by the pair of options at its two ends.
        //
        // The pair is not the whole truth on this path — the prefix fence is
        // part of the question — so an entry outlives the ground it was
        // found on, exactly as `edgeMemo_` says of the greedy's entries. The
        // ends of a chain are where that costs too much to accept: an edge
        // at a launcher has one way off the terminal and the edges beside it
        // crowd the same room, so the two edges at each end are always
        // routed against the prefix that is actually standing
        // (`CHAIN_FRESH_EDGES`, user 2026-09-28). Everything in between is
        // remembered, and the search stops paying for the same pair of
        // options once per prefix that reaches it.
        const bool fresh =
            at < CHAIN_FRESH_EDGES || at + CHAIN_FRESH_EDGES >= edges;
        Path way;
        stepBudget_ = chainStepBudget() ? problem.stepBudget
                                        : routing::TRELLIS_UNREACHABLE;
        auto real =
            edgeCost(wires, chain, at, open[at][prefix[at]], open[at + 1][j],
                     0, fence.empty() ? nullptr : &fence, &way, !fresh);
        stepBudget_ = routing::TRELLIS_UNREACHABLE;
        if (chainBound() >= 2) {
          audit(chain, at, portOfOption(chain, at, open[at][prefix[at]], true),
                portOfOption(chain, at + 1, open[at + 1][j], false), real);
        }

        // **The other order, for the pair at either end of the chain.**
        //
        // The prefix lays its edges front to back, so the edge before this
        // one took the room first and this one has to work around it. At the
        // ends of a chain that order decides whether both fit at all: an
        // edge at a launcher leaves the terminal on one heading and cannot
        // yield, and the edge beside it wants the same ground. Laid the
        // other way round — this one first, then its predecessor again
        // against it — the pair can come home where it could not (user,
        // 2026-09-28).
        //
        // The predecessor's new way hangs off *this* key, because the prefix
        // it belongs to is shared by every choice that extends it and only
        // this choice needed it moved. And the price of the prefix changes
        // with it, which is what `redone` carries back.
        Path before;
        const bool pairAtAnEnd = at == 1 || (at + 1 == edges && at >= 1);
        if (real == routing::TRELLIS_UNREACHABLE && pairAtAnEnd &&
            !fence.empty()) {
          const Path* stood = fence.back();
          const auto wasTurning = wayCostOf(
              *stood,
              portOfOption(chain, at, open[at][prefix[at]], false).heading);
          std::vector<const Path*> without(fence.begin(), fence.end() - 1);
          Path mine;
          const auto first = edgeCost(
              wires, chain, at, open[at][prefix[at]], open[at + 1][j], 0,
              without.empty() ? nullptr : &without, &mine, false);
          if (first != routing::TRELLIS_UNREACHABLE) {
            auto against = without;
            against.push_back(&mine);
            Path again;
            const auto nowTurning =
                edgeCost(wires, chain, at - 1, open[at - 1][prefix[at - 1]],
                         open[at][prefix[at]], 0, &against, &again, false);
            if (nowTurning != routing::TRELLIS_UNREACHABLE) {
              real = first;
              way = std::move(mine);
              before = std::move(again);
              redone = static_cast<std::int64_t>(nowTurning) -
                       static_cast<std::int64_t>(wasTurning);
              tell(std::format(
                  "[Coupler Insertion]   chain {} edge {}: no way after edge "
                  "{}, but both fit with edge {} laid first — edge {} redone "
                  "at {} instead of {}",
                  chain, at, at - 1, at, at - 1, nowTurning, wasTurning));
            }
          }
        }

        if (real == routing::TRELLIS_UNREACHABLE) {
          // The prefix blocks itself here, in either order. Nothing that
          // extends it is a chain, and the search takes another combination
          // — which is the whole of what the trellis cannot do.
          return real;
        }
        // A run of options complete with this step, while the chain is
        // searched again under the capacity check: refused when it leaves
        // as many wires without a way as the run that was searched again
        // (`checkChainCapacity`).
        if (chainCheck_.active && chainCheck_.chain == chain &&
            at + 1 == edges) {
          std::vector<std::size_t> options(layers, 0);
          for (std::size_t layer = 0; layer <= at; ++layer) {
            options[layer] = open[layer][prefix[layer]];
          }
          options[at + 1] = open[at + 1][j];
          std::vector<const Path*> ways(edges, nullptr);
          for (std::size_t e = 0; e < at; ++e) {
            ways[e] = wayOfEdge(e);
          }
          if (!before.empty()) {
            ways[at - 1] = &before;
          }
          ways[at] = &way;
          ++chainCheck_.checks;
          const auto lost = lostWires(
              stateOf(wires, *chainCheck_.settled, chain, &options, &ways));
          const auto added = static_cast<std::size_t>(
              std::ranges::count_if(lost, [&](const std::uint32_t key) {
                return !chainCheck_.before.contains(key);
              }));
          if (added >= chainCheck_.limit ||
              runFaults(wires, chain, options, ways) > chainCheck_.faults) {
            ++chainCheck_.refused;
            return routing::TRELLIS_UNREACHABLE;
          }
        }
        // The capacity check of this step: the chip the chains settled
        // before this one leave, with this prefix standing on top — its
        // couplers up to this edge's far end and its edges up to this one.
        // Under `capacityRule` a step whose edge closes a wire is refused
        // here, before anything of it is remembered.
        if (stepSettled_ != nullptr) {
          const auto began = std::chrono::steady_clock::now();
          std::vector<std::size_t> options(layers, 0);
          std::string named;
          for (std::size_t layer = 0; layer <= at + 1; ++layer) {
            options[layer] = open[layer][layer <= at ? prefix[layer] : j];
            named += std::format("{}{}", layer == 0 ? "" : " ",
                                 points[layer].fixed
                                     ? std::string("launcher")
                                     : std::to_string(options[layer]));
          }
          std::vector<const Path*> ways(edges, nullptr);
          for (std::size_t e = 0; e < at; ++e) {
            ways[e] = wayOfEdge(e);
          }
          if (!before.empty()) {
            ways[at - 1] = &before;
          }
          ways[at] = &way;
          auto verdict = capacityVerdictOf(
              stateOf(wires, *stepSettled_, chain, &options, &ways, at + 2));
          const auto parent = checked.find(key);
          const auto& was = (at == 0 || parent == checked.end())
                                ? unchained
                                : parent->second.verdict;
          std::vector<std::uint32_t> closed;
          std::ranges::set_difference(verdict.lost, was.lost,
                                      std::back_inserter(closed));
          const auto spent = std::chrono::steady_clock::now() - began;
          const auto seconds = std::chrono::duration<double>(spent).count();
          // The check's time is given back to the clock: the budget is for
          // routing, and a chain checked step by step prices as many steps
          // as one that is not.
          if (problem.budget.count() > 0) {
            problem.budget +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(spent);
          }
          ++stepChecks;
          stepsCarrying += verdict.lost.empty() ? 1U : 0U;
          stepsClosing += closed.empty() ? 0U : 1U;
          stepSeconds += seconds;
          const bool refused = stepRefuses_ && !closed.empty();
          stepsRefused += refused ? 1U : 0U;
          const std::vector<std::uint32_t> lost(verdict.lost.begin(),
                                                verdict.lost.end());
          tell(std::format(
              "[Capacity Step] chain {} f{} ({}->{}) on options {}: {}{}; "
              "{}{}; {:.3f}s",
              chain, firstSlot + at, at, at + 1, named,
              lost.empty() ? std::string("SAT") : std::string("UNSAT"),
              lost.empty()
                  ? std::string()
                  : std::format(" — {} wire{} without a way ({}), short by {}",
                                lost.size(), lost.size() == 1 ? "" : "s",
                                namesOf(wires, lost), verdict.shortBy),
              closed.empty()
                  ? std::string("this edge closes no wire")
                  : std::format("this edge closes {}", namesOf(wires, closed)),
              refused ? " — refused (SCPD_CAPACITY_RULE)" : "", seconds));
          if (refused) {
            return routing::TRELLIS_UNREACHABLE;
          }
          auto mine = key;
          mine.push_back(j);
          checked[std::move(mine)] = StepCheck{.verdict = std::move(verdict),
                                               .closed = std::move(closed)};
        }
        if (chainLearnedBound()) {
          auto& known = learned[pairKey(at, prefix[at], j)];
          if (known == 0) {
            known = real;
          } else {
            learnedAbove_ += known > real ? 1 : 0;
            known = std::min(known, real);
          }
        }
        key.push_back(j);
        laid[std::move(key)] = Laid{.way = std::move(way),
                                    .before = std::move(before)};
        return real;
      };

      answer = routing::solveChainAStar(problem);
      if (answer.solved) {
        break;
      }
      // The wall: the deepest layer a prefix whose every step is real ever
      // got to. The step out of it is what shut the chain, so those are the
      // two couplers whose choice is widened — never the whole chain, which
      // trebles what every round has to route and was measured worse.
      bool widened = false;
      const auto unlock = [&](const std::size_t at) {
        if (at >= layers || points[at].fixed) {
          return;
        }
        auto& coupler = couplers_[points[at].coupler];
        if (coupler.jogsUnlocked) {
          return;
        }
        coupler.jogsUnlocked = true;
        widened = true;
        tell(std::format(
            "[Coupler Insertion]   '{}' chain {}: no prefix reaches past it "
            "on plain options — opening the second-dogleg options",
            wireId(wires[coupler.wire]), chain));
      };
      unlock(answer.reached);
      unlock(answer.reached + 1);
      if (!widened) {
        break;
      }
    }

    // On this path a step is priced exactly when it is routed, so the two
    // figures the trellis keeps apart are one and the same.
    out.evaluations = answer.routed;
    out.routed = static_cast<std::uint32_t>(memoMisses_ - routedBefore);
    out.expansions = answer.expansions;
    out.pruned = answer.pruned;
    out.outOfTime = answer.outOfTime;
    if (stepSettled_ != nullptr) {
      const std::vector<std::uint32_t> lost(unchained.lost.begin(),
                                            unchained.lost.end());
      say(std::format(
          "[Capacity Step] chain {}: {} steps checked in {:.1f}s ({:.3f}s "
          "each), {} of them SAT, {} closed a wire, {} refused for it; "
          "without the chain {} wire{} without a way ({})",
          chain, stepChecks, stepSeconds,
          stepChecks == 0 ? 0.0 : stepSeconds / stepChecks, stepsCarrying,
          stepsClosing, stepsRefused, lost.size(), lost.size() == 1 ? "" : "s",
          namesOf(wires, lost)));
      // Every feedline edge of the run the chain settles on, with what the
      // check said when the search laid it.
      for (std::size_t e = 0; answer.solved && e + 1 < layers; ++e) {
        const std::vector<std::uint32_t> run(
            answer.chosen.begin(),
            answer.chosen.begin() + static_cast<std::ptrdiff_t>(e + 2));
        const auto found = checked.find(run);
        if (found == checked.end()) {
          say(std::format("[Capacity Step] chain {} f{}: not checked", chain,
                          firstSlot + e));
          continue;
        }
        const auto& one = found->second;
        const std::vector<std::uint32_t> edgeLost(one.verdict.lost.begin(),
                                                  one.verdict.lost.end());
        say(std::format(
            "[Capacity Step] chain {} f{}: {}{}", chain, firstSlot + e,
            edgeLost.empty()
                ? std::string("SAT")
                : std::format("UNSAT — {} wire{} without a way ({}), short "
                              "by {}",
                              edgeLost.size(), edgeLost.size() == 1 ? "" : "s",
                              namesOf(wires, edgeLost), one.verdict.shortBy),
            one.closed.empty()
                ? std::string(", closes no wire")
                : std::format(", closes {}", namesOf(wires, one.closed))));
      }
    }
    for (std::size_t at = 0; at + 1 < layers; ++at) {
      out.pairs +=
          static_cast<std::uint64_t>(open[at].size()) * open[at + 1].size();
    }
    if (!answer.solved) {
      return out;
    }
    out.solved = true;
    out.optimal = answer.optimal;
    out.relaid = answer.relaid;
    out.cost = answer.cost;
    out.chosen.assign(layers, 0);
    for (std::size_t at = 0; at < layers; ++at) {
      out.chosen[at] = points[at].fixed ? 0 : open[at][answer.chosen[at]];
    }
    // The winner's ways, read out of `laid` under the prefix each was routed
    // against — which is the run of choices the winner itself is. Where a
    // pair at an end of the chain was reordered, the earlier edge's way is
    // the replacement the later one left behind.
    out.ways = waysOfRun(laid, answer.chosen);
    return out;
  }

  /// Stand a whole chain on a run of options and hand back the run it was
  /// standing on. Used to read a chain's ways out of the memo without
  /// leaving anything moved.
  [[nodiscard]] std::vector<std::size_t>
  standOn(const std::uint32_t chain, const std::vector<std::size_t>& options) {
    auto& points = chains_[chain];
    std::vector<std::size_t> kept(points.size(), 0);
    for (std::size_t at = 0; at < points.size(); ++at) {
      if (points[at].fixed) {
        continue;
      }
      auto& coupler = couplers_[points[at].coupler];
      kept[at] = coupler.chosen;
      coupler.chosen = options[at];
    }
    return kept;
  }

  /// A rectangle that may hang off the grid on either side, which
  /// `routing::CellBox` cannot: both of the things compared below are the
  /// grown span of something near an edge of the chip.
  struct Span {
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = 0;
    std::int64_t maxY = 0;

    [[nodiscard]] bool meets(const Span& other) const {
      return other.minX <= maxX && other.maxX >= minX && other.minY <= maxY &&
             other.maxY >= minY;
    }
  };

  /// The box an edge is searched in, for a named option at each of its ends
  /// — the same rectangle `corridorOfEdge` closes everything outside of.
  [[nodiscard]] Span edgeBox(const std::uint32_t chain,
                                         const std::size_t from,
                                         const std::size_t optionFrom,
                                         const std::size_t optionTo) const {
    const auto source = portOfOption(chain, from, optionFrom, true);
    const auto target = portOfOption(chain, from + 1, optionTo, false);
    const std::int64_t margin = EDGE_BOX_MARGIN;
    return {.minX = std::min<std::int64_t>(source.x, target.x) - margin,
            .minY = std::min<std::int64_t>(source.y, target.y) - margin,
            .maxX = std::max<std::int64_t>(source.x, target.x) + margin,
            .maxY = std::max<std::int64_t>(source.y, target.y) + margin};
  }

  /// Forget every remembered edge whose corridor one of these ways can have
  /// changed, and keep the rest.
  ///
  /// A round's answer only moves an edge's price where it moves a way that
  /// edge could see. Everything outside an edge's box is closed to it
  /// whatever happens, so a way that changed matters to it only if what the
  /// way closes — itself, spread by the clearance — reaches into that box.
  /// The test is on bounding boxes, so it forgets more than it has to and
  /// never less.
  ///
  /// Throwing the whole memo away at the head of every round is the safe
  /// thing and it is what the rounds cost: on 17q three of four rounds
  /// re-priced chains that had not moved and could not move.
  void forgetEdgesNear(const std::vector<Span>& moved) {
    if (moved.empty()) {
      return;
    }
    std::erase_if(edgeMemo_, [&](const auto& entry) {
      const auto key = entry.first;
      const auto chain = static_cast<std::uint32_t>((key >> 56U) & 0xFFU);
      const auto from = static_cast<std::size_t>((key >> 48U) & 0xFFU);
      const auto encodedFrom = static_cast<std::size_t>((key >> 16U) & 0xFFFFU);
      const auto encodedTo = static_cast<std::size_t>(key & 0xFFFFU);
      if (chain >= chains_.size() || from + 1 >= chains_[chain].size()) {
        return true;
      }
      const auto box = edgeBox(chain, from,
                               encodedFrom == 0 ? 0 : encodedFrom - 1,
                               encodedTo == 0 ? 0 : encodedTo - 1);
      return std::ranges::any_of(
          moved, [&](const Span& what) { return what.meets(box); });
    });
  }

  /// Where a way that changed reaches, the clearance it keeps included.
  [[nodiscard]] Span reachOf(const Path& way) const {
    const std::int64_t spread = tuning_.clearance;
    Span box{.minX = std::numeric_limits<std::int64_t>::max(),
             .minY = std::numeric_limits<std::int64_t>::max(),
             .maxX = std::numeric_limits<std::int64_t>::min(),
             .maxY = std::numeric_limits<std::int64_t>::min()};
    for (const auto& point : way) {
      box.minX = std::min<std::int64_t>(box.minX, point.x);
      box.minY = std::min<std::int64_t>(box.minY, point.y);
      box.maxX = std::max<std::int64_t>(box.maxX, point.x);
      box.maxY = std::max<std::int64_t>(box.maxY, point.y);
    }
    box.minX -= spread;
    box.minY -= spread;
    box.maxX += spread;
    box.maxY += spread;
    return box;
  }

  /// How many edges of the chip would not survive the state it is standing
  /// on: their way is empty, or it no longer lies in the corridor that holds
  /// once every other edge of that same state is fenced.
  ///
  /// This is the commit's own test — `edgeWayStillOpen` — asked of a whole
  /// state at once, and it is what a round has to be judged by. A round
  /// whose answer scores zero here is one the commit will keep entire; a
  /// round that scores above zero will have edges routed again from
  /// scratch, and those are the ones that come back with no way at all.
  ///
  /// It builds a corridor per edge and searches for nothing, so it costs a
  /// few dozen corridor builds and not one A*.
  [[nodiscard]] std::uint32_t stateFaults(const std::vector<Wire>& wires) {
    std::uint32_t faults = 0;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      faults += chainFaults(wires, chain);
    }
    return faults;
  }

  /// `chainFaults` of a chain standing on a run of options and ways for a
  /// moment; the options and ways it stood on before are put back.
  [[nodiscard]] std::uint32_t runFaults(const std::vector<Wire>& wires,
                                        const std::uint32_t chain,
                                        const std::vector<std::size_t>& options,
                                        const std::vector<const Path*>& ways) {
    const auto& points = chains_[chain];
    std::vector<std::size_t> kept(points.size(), 0);
    for (std::size_t at = 0; at < points.size(); ++at) {
      if (!points[at].fixed) {
        auto& coupler = couplers_[points[at].coupler];
        kept[at] = coupler.chosen;
        coupler.chosen = options[at];
      }
    }
    auto keptWays = chainEdgePaths_[chain];
    for (std::size_t at = 0; at < ways.size(); ++at) {
      chainEdgePaths_[chain][at] = ways[at] != nullptr ? *ways[at] : Path{};
    }
    const auto faults = chainFaults(wires, chain);
    chainEdgePaths_[chain] = std::move(keptWays);
    for (std::size_t at = 0; at < points.size(); ++at) {
      if (!points[at].fixed) {
        couplers_[points[at].coupler].chosen = kept[at];
      }
    }
    return faults;
  }

  /// The edges of one chain that would not survive the state, as
  /// `stateFaults` counts them.
  [[nodiscard]] std::uint32_t chainFaults(const std::vector<Wire>& wires,
                                          const std::uint32_t chain) {
    // The one place that fences every chain against every other whatever
    // the searches do. This is the commit's test asked of a whole state,
    // and the commit lays all the chains on one chip — so with the fence
    // open it would answer zero by construction and say nothing.
    fenceEverything_ = true;
    std::uint32_t faults = 0;
    const auto& points = chains_[chain];
    for (std::size_t at = 0; at + 1 < points.size(); ++at) {
      const auto& way = chainEdgePaths_[chain][at];
      if (way.empty()) {
        ++faults;
        continue;
      }
      const RoutingObjective objective{.source = sourceOf(points[at]),
                                       .target = targetOf(points[at + 1])};
      const Edge edge{.chain = chain,
                      .from = at,
                      .to = at + 1,
                      .terminal = points[at].fixed || points[at + 1].fixed};
      if (!way.front().samePlace(objective.source) ||
          !way.back().samePlace(objective.target) ||
          !edgeWayStillOpen(wires, objective, edge, way)) {
        ++faults;
      }
    }
    fenceEverything_ = false;
    return faults;
  }

  /// The exact search over every chain, in rounds.
  ///
  /// The one thing that stops a chain's price being a function of its own
  /// two options is the fence of every other edge on the chip. A round
  /// freezes that fence, solves each chain against it exactly, and takes
  /// the answer into the fence before the next chain is solved; the rounds
  /// repeat until no chain moves.
  ///
  /// Freezing the fence for a whole round instead — every chain solved
  /// against the ways as they stood when the round began, none written back
  /// until all are done — makes the chains of a round independent of each
  /// other, which is what would make them parallel. It was measured and it
  /// oscillates: on 17q chain 0 flipped between two optima round after
  /// round, and the walk ended on whichever half the last round left.
  ///
  /// **Optimal against a frozen fence**, not optimal outright — the fence is
  /// the part no decomposition reaches.
  ///
  /// @returns How many rounds changed something.
  [[nodiscard]] std::uint32_t optimizeChainsExact(std::vector<Wire>& wires) {
    constexpr std::uint32_t ROUNDS = 4;
    std::uint32_t changed = 0;

    // A round is solved against the round before it, so the rounds are a
    // walk and not a descent: a chain can be driven off its own optimum by a
    // neighbour and back onto it, round after round, and stop on whichever
    // half of the flip the last round happened to be. Two things follow.
    //
    // Every whole-chip state the walk has stood on is kept, and the walk
    // stops the moment it stands on one twice — from there it only repeats
    // itself, and on 17q that is the difference between stopping at round 2
    // and paying out all four.
    //
    // And the state the search leaves behind is the best round it stood on,
    // judged **first** by how many of its edges survive its own fence and
    // only then by what it costs.
    //
    // Judging by cost alone does not work and was measured: round 0 is
    // solved against a fence with nothing in it, so it is always the
    // cheapest and always the least real, and keeping it cost 9q a feedline
    // edge and 17q two. Whether a state survives itself is the thing that
    // is comparable between rounds, because it is the test the commit
    // actually applies.
    const auto stateNow = [&]() {
      std::vector<std::size_t> state;
      state.reserve(couplers_.size());
      for (const auto& coupler : couplers_) {
        state.push_back(coupler.chosen);
      }
      return state;
    };
    std::vector<std::vector<std::size_t>> seen;
    std::vector<std::size_t> bestState;
    std::vector<std::vector<Path>> bestWays;
    auto bestFaults = std::numeric_limits<std::uint32_t>::max();
    auto bestTotal = routing::TRELLIS_UNREACHABLE;
    std::uint32_t bestRound = 0;

    // The ways as the round before left them, so the round can say which of
    // them moved and forget only what those reach.
    auto wasBefore = chainEdgePaths_;
    for (std::uint32_t round = 0; round < ROUNDS; ++round) {
      if (round > 0) {
        std::vector<Span> moved;
        for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
          for (std::size_t at = 0; at < chainEdgePaths_[chain].size(); ++at) {
            const auto& now = chainEdgePaths_[chain][at];
            const auto& then = wasBefore[chain][at];
            if (now.size() == then.size() &&
                std::ranges::equal(now, then, [](const PathPoint& a,
                                                 const PathPoint& b) {
                  return a.samePlace(b);
                })) {
              continue;
            }
            if (!then.empty()) {
              moved.push_back(reachOf(then));
            }
            if (!now.empty()) {
              moved.push_back(reachOf(now));
            }
          }
        }
        forgetEdgesNear(moved);
        wasBefore = chainEdgePaths_;
      }

      bool moved = false;
      std::uint64_t total = 0;
      for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
        // Solved against the ways as they stand **now**, the chains already
        // seen in this round included. Solving every chain of a round
        // against one frozen fence and writing them all at the end is the
        // other way round, and it makes the chains of a round independent —
        // which is what would make them parallel. It also oscillates: on
        // 17q chain 0 flipped between two optima round after round and the
        // search stopped on whichever half the last round left. Taking each
        // chain's answer into the fence at once settles it.
        looseChain_ = exactChainSearch() >= 2 ? chain : NO_OWNER;
        const auto answer = chainAStar() ? solveChainAStar(wires, chain)
                                         : solveChain(wires, chain);
        looseChain_ = NO_OWNER;
        // Once a chain of the round cannot be joined at all, the round's
        // total is unreachable and must *stay* unreachable. Adding the next
        // chain's cost onto TRELLIS_UNREACHABLE wraps it — on 69q an
        // unsolved chain plus 400000 printed as `total 399999`, which then
        // undercuts every honest round in the `total < bestTotal` tiebreak
        // below and makes the least real round look the cheapest.
        total = (!answer.solved || total == routing::TRELLIS_UNREACHABLE)
                    ? routing::TRELLIS_UNREACHABLE
                    : total + answer.cost;
        if (!answer.solved) {
          // Nothing the trellis can offer joins this chain up.
          //
          // **The chain is left as it stands.** It used to fall back to the
          // greedy here, which is not wanted (user, 2026-09-27): a round is
          // then part exact and part coordinate descent, the greedy's choice
          // is fenced into every chain solved after it, and the round total
          // no longer prices what the search actually chose. Leaving the
          // chain alone keeps the round one method's answer, and the edges
          // that cannot be drawn are counted by `stateFaults` and reported
          // by the commit — which is the honest outcome rather than a
          // quietly patched one.
          // Two different things, and calling them by one name reads as a
          // geometric verdict where it is only a clock. `optimal` on an
          // unsolved answer says the search saw every run of options there
          // is; without it, the search simply ran out of time.
          say(std::format(
              "[Coupler Insertion] chain {} round {}: {} — leaving it as it "
              "stands ({} steps routed)",
              chain, round,
              answer.optimal
                  ? "no run of options joins the chain"
                  : "out of time before any run of options joined the chain",
              answer.routed));
          continue;
        }
        auto& points = chains_[chain];
        for (std::size_t at = 0; at < points.size(); ++at) {
          if (points[at].fixed) {
            continue;
          }
          auto& coupler = couplers_[points[at].coupler];
          moved = moved || answer.chosen[at] != coupler.chosen;
          coupler.chosen = answer.chosen[at];
        }
        for (std::size_t at = 0; at + 1 < points.size(); ++at) {
          if (!answer.ways[at].empty()) {
            chainEdgePaths_[chain][at] = answer.ways[at];
          }
        }
        say(chainAStar()
                ? std::format("[Coupler Insertion] chain {} round {}: {} {} "
                              "over {} layers, {} prefixes expanded, {} "
                              "steps routed",
                              chain, round,
                              answer.optimal   ? "optimum"
                              : answer.relaid  ? "best found, an end pair "
                                                 "laid the other way round"
                                               : "best in the time given",
                              answer.cost, points.size(), answer.expansions,
                              answer.routed) +
                      (answer.pruned == 0
                           ? std::string{}
                           : std::format(", {} left unpriced", answer.pruned))
                : std::format("[Coupler Insertion] chain {} round {}: optimum "
                              "{} over {} layers, {} of {} pairs priced, {} "
                              "of them searched",
                              chain, round, answer.cost, points.size(),
                              answer.evaluations, answer.pairs,
                              answer.routed));
      }
      const auto faults = stateFaults(wires);
      if (faults < bestFaults ||
          (faults == bestFaults && total < bestTotal)) {
        bestFaults = faults;
        bestTotal = total;
        bestState = stateNow();
        bestWays = chainEdgePaths_;
        bestRound = round;
      }
      say(std::format("[Coupler Insertion] round {}: total {}, {} edge{} "
                      "would not survive this state",
                      round, spell(total), faults, faults == 1 ? "" : "s"));
      if (faults == 0) {
        // A state the commit keeps entire. No later round can better it on
        // the thing that is judged first, so there is nothing to pay for.
        say(std::format("[Coupler Insertion] round {}: every edge survives "
                        "this state — stopping",
                        round));
        break;
      }
      if (!moved) {
        say(std::format("[Coupler Insertion] round {}: nothing moved — the "
                        "chains stand on their own fence",
                        round));
        break;
      }
      ++changed;
      const auto state = stateNow();
      if (std::ranges::find(seen, state) != seen.end()) {
        say(std::format("[Coupler Insertion] round {}: back on a state it has "
                        "already stood on — stopping",
                        round));
        break;
      }
      seen.push_back(state);
    }

    if (!bestState.empty()) {
      for (std::size_t index = 0; index < couplers_.size(); ++index) {
        couplers_[index].chosen = bestState[index];
      }
      chainEdgePaths_ = bestWays;
      say(std::format("[Coupler Insertion] the chains stand on round {}: "
                      "total {}, {} edge{} it would not keep",
                      bestRound, spell(bestTotal), bestFaults,
                      bestFaults == 1 ? "" : "s"));
    }
    return changed;
  }

  /// The prefix search over every chain, once. **There are no rounds.**
  ///
  /// The rounds of `optimizeChainsExact` exist because the trellis prices a
  /// chain against a fence it cannot see all of: its answer may not survive
  /// its own fence, so a later round has something to correct, and the round
  /// that is kept has to be chosen by `stateFaults`. None of that applies
  /// here. The prefix search routes every edge against the ways the options
  /// before it laid, so the state it hands over survives itself.
  ///
  /// That is measured and not argued. Under the rounds, **seven of the eight
  /// chips reported zero faults after round 0** and the loop stopped there of
  /// its own accord; only 69q ever paid for a second round, where it changed
  /// nothing and cost 200 s. The rounds are gone from this path (user,
  /// 2026-09-28).
  ///
  /// What is left is one pass over the chains, each solved against the ways
  /// the chains before it settled on — the same Gauss-Seidel order the
  /// commit lays them in.
  ///
  /// @returns How many chains it settled.
  [[nodiscard]] std::uint32_t optimizeChainsPrefix(std::vector<Wire>& wires) {
    std::uint32_t settled = 0;
    // The chains settled so far, for the capacity check.
    std::vector<bool> standing(chains_.size(), false);
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      soloChain_ = chainSolo() ? chain : NO_OWNER;
      stepSettled_ =
          (capacityStep() || capacityRule()) ? &standing : nullptr;
      stepRefuses_ = capacityRule();
      auto answer = solveChainAStar(wires, chain);
      if (!answer.solved && stepRefuses_) {
        say(std::format("[Coupler Insertion] chain {}: {} ({} steps routed) "
                        "— searched again without the capacity rule",
                        chain,
                        answer.outOfTime
                            ? "out of time before a run of options joined "
                              "the chain without closing a wire"
                            : "no run of options joins the chain without "
                              "closing a wire",
                        answer.routed));
        stepRefuses_ = false;
        stepSettled_ = capacityStep() ? &standing : nullptr;
        answer = solveChainAStar(wires, chain);
      }
      stepSettled_ = nullptr;
      stepRefuses_ = false;
      soloChain_ = NO_OWNER;
      auto& points = chains_[chain];
      if (!answer.solved) {
        // **The chain is left as it stands**, and the edges it cannot draw
        // are reported by the commit. Two different reasons, and calling
        // them by one name reads as a verdict about the geometry where it is
        // only a clock.
        say(std::format(
            "[Coupler Insertion] chain {}: {} — leaving it as it stands "
            "({} steps routed)",
            chain,
            answer.outOfTime
                ? "out of time before any run of options joined the chain"
                : "no run of options joins the chain",
            answer.routed));
        continue;
      }
      ++settled;
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (!points[at].fixed) {
          couplers_[points[at].coupler].chosen = answer.chosen[at];
        }
      }
      for (std::size_t at = 0; at + 1 < points.size(); ++at) {
        if (!answer.ways[at].empty()) {
          chainEdgePaths_[chain][at] = answer.ways[at];
        }
      }
      say(std::format("[Coupler Insertion] chain {}: {} {} over {} layers, "
                      "{} prefixes expanded, {} steps routed",
                      chain,
                      answer.optimal  ? "optimum"
                      : answer.relaid ? "best found, an end pair laid the "
                                        "other way round"
                                      : "best in the time given",
                      answer.cost, points.size(), answer.expansions,
                      answer.routed));
      if (capacityChain()) {
        checkChainCapacity(wires, chain, standing);
      }
      standing[chain] = true;
    }
    // What the commit is about to find, said once. It decides nothing — with
    // one pass there is no second state to prefer — but it is the number
    // this whole approach is judged by, so it belongs in the log: an edge
    // counted here is an edge the commit will route again from scratch.
    const auto faults = stateFaults(wires);
    say(std::format("[Coupler Insertion] {} of {} chains settled{}, {} edge{} "
                    "would not survive this state",
                    settled, chains_.size(),
                    chainSolo() ? " each on its own chip" : "", faults,
                    faults == 1 ? "" : "s"));
    return settled;
  }

  /// Whether every step of the chain search is followed by the capacity
  /// check of the chip as that step leaves it: `SCPD_CAPACITY_STEP`, off
  /// (user, 2026-10-08). On its own it reports and the search takes no
  /// notice of it; `capacityRule` checks every step as well and refuses. The
  /// steps say at `-v 1` whether the graph still carries every wire, and
  /// every feedline edge of a settled chain says it once. The checks spend
  /// the time of the chain search's clock, so a chain that runs into
  /// `SCPD_CHAIN_ASTAR_SECONDS` prices fewer steps with them.
  [[nodiscard]] static bool capacityStep() {
    return envFlag("SCPD_CAPACITY_STEP", false);
  }

  /// Whether the chain search refuses a step whose edge closes a wire the
  /// prefix before it left a way: `SCPD_CAPACITY_RULE`, **on** (user,
  /// 2026-10-09). "Closes" is what the capacity check of `capacityStep`
  /// says: a wire with a way through the capacity graph of the chip the
  /// prefix leaves, routed in turn, and none once this edge stands. The
  /// search then takes another option, as it does for an edge with no way.
  /// A chain that no run of options joins under the rule is searched once
  /// more without it: a chain left on its first options is worse than one
  /// that closes a wire.
  [[nodiscard]] static bool capacityRule() {
    return envFlag("SCPD_CAPACITY_RULE", true);
  }

  /// Whether the capacity graph is checked after every settled chain
  /// (`checkChainCapacity`): `SCPD_CAPACITY_CHAIN`, off.
  [[nodiscard]] static bool capacityChain() {
    return envFlag("SCPD_CAPACITY_CHAIN", false);
  }
  /// How often `checkChainCapacity` searches one chain again:
  /// `SCPD_CAPACITY_CHAIN_TRIES`, 3.
  [[nodiscard]] static std::uint32_t capacityChainTries() {
    return static_cast<std::uint32_t>(
        std::clamp(envWhole("SCPD_CAPACITY_CHAIN_TRIES", 3), 0, 20));
  }

  /// A state of the insertion as the capacity analysis reads it: the
  /// wires, with a feedline wire for every edge drawn; those edges; and the
  /// couplers that stand.
  struct InsertionState {
    std::vector<Wire> wires;
    std::vector<Edge> edges;
    std::vector<bool> placed;
  };

  /// The insertion with the chains `in` standing on the options and ways
  /// they settled on, and chain `chain`, when `options` is given, on
  /// `options` (one per waypoint) and `ways` (one per edge; an edge with no
  /// way is not drawn). Of that chain only the first `standing` waypoints
  /// stand, as a prefix of the chain search leaves it. A standing
  /// coupler has its pad and its resonator the lead `applyOption` gives
  /// it; a resonator whose coupler does not stand yet is left out, since
  /// its lead is not known. The feedline edges are numbered as the commit
  /// numbers them, so they carry the names the commit gives them.
  [[nodiscard]] InsertionState
  stateOf(const std::vector<Wire>& wires, const std::vector<bool>& in,
          const std::uint32_t chain = NO_OWNER,
          const std::vector<std::size_t>* options = nullptr,
          const std::vector<const Path*>* ways = nullptr,
          const std::size_t standing =
              std::numeric_limits<std::size_t>::max()) const {
    InsertionState state{.wires = wires,
                         .edges = {},
                         .placed = std::vector<bool>(couplers_.size(), false)};
    std::uint32_t slot = 0;
    for (std::uint32_t at = 0; at < chains_.size(); ++at) {
      const auto& points = chains_[at];
      const bool given = at == chain && options != nullptr;
      if (!in[at] && !given) {
        slot +=
            points.empty() ? 0U : static_cast<std::uint32_t>(points.size() - 1);
        continue;
      }
      for (std::size_t point = 0; point < points.size(); ++point) {
        if (points[point].fixed || (given && point >= standing)) {
          continue;
        }
        const auto index = points[point].coupler;
        const auto& coupler = couplers_[index];
        const auto& option =
            coupler.options[given ? (*options)[point] : coupler.chosen];
        state.placed[index] = true;
        // The lead as `applyOption` cuts it.
        auto& wire = state.wires[coupler.wire];
        const auto spare =
            (trimTheLead() && option.arc.size() > couplerLeadMargin() + 1)
                ? static_cast<std::size_t>(couplerLeadMargin())
                : std::size_t{0};
        wire.arc.assign(option.arc.begin(),
                        option.arc.end() - static_cast<std::ptrdiff_t>(spare));
        wire.objective.source =
            spare == 0 || wire.arc.empty()
                ? option.arcEnd
                : PathPoint{.x = wire.arc.back().x,
                            .y = wire.arc.back().y,
                            .heading = option.arcEnd.heading,
                            .primitive = wire.arc.back().primitive};
        wire.couplerAtSource = index;
      }
      for (std::size_t from = 0; from + 1 < points.size(); ++from) {
        const Path* way = given && ways != nullptr ? (*ways)[from]
                                                   : &chainEdgePaths_[at][from];
        Wire feed;
        feed.key = static_cast<std::uint32_t>(state.wires.size());
        feed.slot = slot++;
        feed.feedline = true;
        feed.feasible = true;
        feed.terminal = points[from].fixed || points[from + 1].fixed;
        if (way != nullptr) {
          feed.way = *way;
        }
        feed.drawn = !feed.way.empty();
        state.edges.push_back({.chain = at,
                               .from = from,
                               .to = from + 1,
                               .wire = feed.key,
                               .terminal = feed.terminal});
        state.wires.push_back(std::move(feed));
      }
    }
    for (std::size_t index = 0; index < couplers_.size(); ++index) {
      if (!state.placed[index] && couplers_[index].wire < wires.size()) {
        state.wires[couplers_[index].wire].feasible = false;
      }
    }
    return state;
  }

  /// What the capacity graph of a state says: the outer wires it has no
  /// way for, and by how many wires its edges are short.
  struct CapacityVerdict {
    std::set<std::uint32_t> lost;
    std::uint32_t shortBy = 0;
  };

  /// The outer wires the capacity graph of a state has no way for: a
  /// terminal in no chamber, no way at all, or a way over an edge with
  /// overflow, routed in turn (`checkInTurn`) or, under
  /// `SCPD_CAPACITY_FLOW`, by the flow.
  [[nodiscard]] std::set<std::uint32_t>
  lostWires(const InsertionState& state) const {
    return capacityVerdictOf(state).lost;
  }

  /// `lostWires`, and the shortfall of the edges with it.
  [[nodiscard]] CapacityVerdict
  capacityVerdictOf(const InsertionState& state) const {
    const auto pass =
        capacityPass(state.wires, state.edges, &state.placed, NO_OWNER);
    CapacityVerdict verdict{.lost = {}, .shortBy = pass.check.shortBy};
    auto& lost = verdict.lost;
    lost.insert(pass.problem.walled.begin(), pass.problem.walled.end());
    for (std::size_t demand = 0; demand < pass.check.ways.size(); ++demand) {
      const auto& way = pass.check.ways[demand];
      const bool over =
          way.has_value() &&
          std::ranges::any_of(*way, [&](const std::uint32_t index) {
            return pass.check.overflow[index] > 0;
          });
      if (!way.has_value() || over) {
        lost.insert(pass.problem.wireOf[demand]);
      }
    }
    return verdict;
  }

  /// The capacity check after a settled chain (user, 2026-10-08). The
  /// chip as the chains settled so far leave it is compared with the chip
  /// with this chain standing as well: the wires that have a way through
  /// the capacity graph before it and none after it are the ones its
  /// couplers and feedline edges close. When there are any, the chain is
  /// searched again, and a complete run of options is refused at its last
  /// step when it closes as many, or when more of its edges fail the
  /// commit's test than of the run it replaces (`chainFaults`); the
  /// cheapest run that closes fewer is taken. That is repeated
  /// `capacityChainTries` times or until the chain closes none. A search that
  /// finds no such run leaves the chain as it was.
  void checkChainCapacity(const std::vector<Wire>& wires,
                          const std::uint32_t chain,
                          const std::vector<bool>& settled) {
    const auto began = std::chrono::steady_clock::now();
    const auto before = lostWires(stateOf(wires, settled));
    auto with = settled;
    with[chain] = true;
    const auto closed = [&] {
      std::vector<std::uint32_t> keys;
      for (const auto key : lostWires(stateOf(wires, with))) {
        if (!before.contains(key)) {
          keys.push_back(key);
        }
      }
      return keys;
    };
    auto lost = closed();
    say(std::format(
        "[Capacity Check] chain {}: {} wire{} without a way before "
        "it, {} more with it{}{} ({:.2f}s)",
        chain, before.size(), before.size() == 1 ? "" : "s", lost.size(),
        lost.empty() ? "" : ": ",
        lost.empty() ? std::string() : namesOf(wires, lost),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count()));
    auto& points = chains_[chain];
    for (std::uint32_t attempt = 0;
         attempt < capacityChainTries() && !lost.empty(); ++attempt) {
      const auto again = std::chrono::steady_clock::now();
      chainCheck_ = {.active = true,
                     .chain = chain,
                     .settled = &settled,
                     .before = before,
                     .limit = lost.size(),
                     .faults = chainFaults(wires, chain),
                     .checks = 0,
                     .refused = 0};
      soloChain_ = chainSolo() ? chain : NO_OWNER;
      const auto answer = solveChainAStar(wires, chain);
      soloChain_ = NO_OWNER;
      chainCheck_.active = false;
      const auto seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - again)
                               .count();
      if (!answer.solved) {
        say(std::format(
            "[Capacity Check] chain {}: searched again, {} run of options "
            "closes fewer than {} — the chain stays as it was ({} runs "
            "checked, {} refused, {} steps routed, {:.2f}s)",
            chain, answer.outOfTime ? "out of time before any" : "no",
            lost.size(), chainCheck_.checks, chainCheck_.refused, answer.routed,
            seconds));
        break;
      }
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (!points[at].fixed) {
          couplers_[points[at].coupler].chosen = answer.chosen[at];
        }
      }
      for (std::size_t at = 0; at + 1 < points.size(); ++at) {
        if (!answer.ways[at].empty()) {
          chainEdgePaths_[chain][at] = answer.ways[at];
        }
      }
      lost = closed();
      say(std::format(
          "[Capacity Check] chain {}: searched again, {} {} — {} more without "
          "a way{}{} ({} runs checked, {} refused, {} steps routed, {:.2f}s)",
          chain,
          answer.optimal  ? "optimum"
          : answer.relaid ? "best found, an end pair laid the other way round"
                          : "best in the time given",
          answer.cost, lost.size(), lost.empty() ? "" : ": ",
          lost.empty() ? std::string() : namesOf(wires, lost),
          chainCheck_.checks, chainCheck_.refused, answer.routed, seconds));
    }
  }

  /// The pad of an option as the four corners of the rectangle its body
  /// cells span along its orientation, half a cell out.
  [[nodiscard]] std::vector<std::array<double, 2>>
  padCorners(const CouplerOption& option) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto v = routing::headingVector(option.orientation);
    const auto norm =
        std::hypot(static_cast<double>(v.dx), static_cast<double>(v.dy));
    const auto ax = static_cast<double>(v.dx) / norm;
    const auto ay = static_cast<double>(v.dy) / norm;
    const auto bx = -ay;
    const auto by = ax;
    auto alongLow = std::numeric_limits<double>::max();
    auto alongHigh = std::numeric_limits<double>::lowest();
    auto acrossLow = std::numeric_limits<double>::max();
    auto acrossHigh = std::numeric_limits<double>::lowest();
    for (const auto cell : option.body) {
      const auto x =
          static_cast<double>(static_cast<std::int64_t>(cell) % width);
      const auto y =
          static_cast<double>(static_cast<std::int64_t>(cell) / width);
      alongLow = std::min(alongLow, (x * ax) + (y * ay));
      alongHigh = std::max(alongHigh, (x * ax) + (y * ay));
      acrossLow = std::min(acrossLow, (x * bx) + (y * by));
      acrossHigh = std::max(acrossHigh, (x * bx) + (y * by));
    }
    if (option.body.empty()) {
      return {};
    }
    std::vector<std::array<double, 2>> corners;
    for (const auto& [a, b] : {std::pair{alongLow - 0.5, acrossLow - 0.5},
                               std::pair{alongHigh + 0.5, acrossLow - 0.5},
                               std::pair{alongHigh + 0.5, acrossHigh + 0.5},
                               std::pair{alongLow - 0.5, acrossHigh + 0.5}}) {
      corners.push_back({(a * ax) + (b * bx), (a * ay) + (b * by)});
    }
    return corners;
  }

  /// The couplers as they stand, with every option of each, as JSON for the
  /// page that moves couplers (`CouplerSession`). Per coupler: its index,
  /// its resonator, its chain and waypoint, the option it stands on and its
  /// two feedline edges. Per option: the pad as four corners, the two
  /// feedline ports, the lead, the length of resonator left in layout
  /// units, and whether the chain search offers it (`openOptionsOf`: the
  /// shortfall rule, the body guard, the jogs).
  [[nodiscard]] std::string couplersJson(const std::vector<Wire>& wires) const {
    using nlohmann::json;
    const auto point = [](const PathPoint& at) {
      return json::array({at.x, at.y});
    };
    const auto edgeName = [&wires](const std::uint32_t key) {
      return key < wires.size() ? json(wireId(wires[key])) : json(nullptr);
    };
    json list = json::array();
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& points = chains_[chain];
      const auto feedlines = feedlineCellsBut(chain);
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (points[at].fixed) {
          continue;
        }
        const auto index = points[at].coupler;
        const auto& coupler = couplers_[index];
        const auto open = openOptionsOf(chain, at, &feedlines);
        json options = json::array();
        for (std::size_t k = 0; k < coupler.options.size(); ++k) {
          const auto& option = coupler.options[k];
          json lead = json::array();
          for (const auto& cell : option.arc) {
            lead.push_back(point(cell));
          }
          options.push_back(
              {{"index", k},
               {"open", std::ranges::find(
                            open, static_cast<std::uint32_t>(k)) != open.end()},
               {"orientation", option.orientation},
               {"offset", option.offset},
               {"secondPort", option.secondPort},
               {"jog", option.secondStraight},
               {"centre", point(option.centre)},
               {"in", point(option.in)},
               {"out", point(option.out)},
               {"pad", padCorners(option)},
               {"lead", lead},
               {"resonator", std::lround(lengthInUnits(option.way))}});
        }
        list.push_back({{"index", index},
                        {"resonator", wireId(wires[coupler.wire])},
                        {"chain", chain},
                        {"at", at},
                        {"chosen", coupler.chosen},
                        {"jogs", coupler.jogsUnlocked},
                        {"edgeIn", edgeName(coupler.edgeIn)},
                        {"edgeOut", edgeName(coupler.edgeOut)},
                        {"options", options}});
      }
    }
    return json{{"couplers", list}}.dump();
  }

  /// Put a coupler on another option after the insertion and draw its two
  /// feedline edges again, for the page that moves couplers
  /// (`CouplerSession`). The option is applied as the commit applies it
  /// (`applyOption`); both edges stand down first, so that neither is
  /// routed around the other's way to the old ports, and are then routed
  /// as the commit routes an edge, against every edge on the chip
  /// (`fenceEverything_`). Returns
  /// JSON: per edge whether it was drawn and its length in cells.
  ///
  /// @throws std::invalid_argument when the coupler or the option does not
  /// exist.
  [[nodiscard]] std::string setCouplerOption(std::vector<Wire>& wires,
                                             const std::uint32_t index,
                                             const std::size_t option) {
    if (index >= couplers_.size() ||
        option >= couplers_[index].options.size()) {
      throw std::invalid_argument(
          std::format("coupler {} has no option {}", index, option));
    }
    applyOption(wires, index, option);
    const auto& coupler = couplers_[index];
    std::vector<std::uint32_t> keys;
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key < wires.size()) {
        keys.push_back(key);
      }
    }
    for (const auto key : keys) {
      auto& wire = wires[key];
      const auto& edge = edges_[wire.slot];
      if (wire.drawn) {
        lift(wire);
      }
      wire.drawn = false;
      wire.way.clear();
      chainEdgePaths_[edge.chain][edge.from].clear();
    }
    using nlohmann::json;
    json drawn = json::array();
    for (const auto key : keys) {
      auto& wire = wires[key];
      const auto edge = edges_[wire.slot];
      const auto& points = chains_[edge.chain];
      wire.objective.source = sourceOf(points[edge.from]);
      wire.objective.target = targetOf(points[edge.to]);
      wire.startStub = couplerStubs() ? couplerRunOf(points[edge.from].coupler)
                                      : tuning_.straightStart;
      wire.endStub = points[edge.to].fixed
                         ? tuning_.straightStart
                         : couplerRunOf(points[edge.to].coupler);
      fenceEverything_ = true;
      auto way = routeEdge(wires, wire.objective, edge);
      fenceEverything_ = false;
      wire.way = std::move(way);
      wire.drawn = !wire.way.empty();
      if (wire.drawn) {
        place(wire);
      }
      chainEdgePaths_[edge.chain][edge.from] = wire.way;
      drawn.push_back({{"edge", wireId(wire)},
                       {"drawn", wire.drawn},
                       {"cells", wire.way.size()}});
      say(std::format("[Coupler Session] '{}' on option {}: edge {} {}",
                      wireId(wires[coupler.wire]), option, wireId(wire),
                      wire.drawn
                          ? std::format("drawn, {} cells", wire.way.size())
                          : std::string("NO WAY")));
    }
    return json{{"coupler", index}, {"option", option}, {"edges", drawn}}
        .dump();
  }

  /// The capacity graph of the chip as it stands, as the JSON
  /// `writeCapacityGraph` writes, without a picture
  /// (`CouplerSession`). The lines of the analysis are said as ever.
  [[nodiscard]] std::string capacityGraphJson(const std::vector<Wire>& wires) {
    std::string captured;
    auto kept = std::move(debug_);
    debug_ = [&captured](const std::string_view name,
                         const std::string_view content) {
      if (name == "final-capacity-graph.json") {
        captured = content;
      }
      return std::string(name);
    };
    graphOnly_ = true;
    try {
      analyseCapacity(wires);
    } catch (...) {
      graphOnly_ = false;
      debug_ = std::move(kept);
      throw;
    }
    graphOnly_ = false;
    debug_ = std::move(kept);
    return captured;
  }

  /// Put a coupler's option in place: the resonator's way is cut back to
  /// the coupler, its search from now on starts at the end of the turn on
  /// the coupling run, and the body becomes an obstacle. The body it had
  /// before is cleared first, which the prototype needs
  /// `remove_cpw_coupler_obstacles` for.
  void applyOption(std::vector<Wire>& wires, const std::uint32_t index,
                   const std::size_t option) {
    auto& coupler = couplers_[index];
    auto& wire = wires[coupler.wire];
    if (coupler.anchor.x != 0 || coupler.anchor.y != 0 ||
        wire.couplerAtSource == index) {
      const auto& before = coupler.options[coupler.chosen];
      for (const auto cell : before.body) {
        bodies_.set(cell, false);
      }
    }
    coupler.chosen = option;
    const auto& chosen = coupler.options[option];
    lift(wire);
    // The lead, less the margin the insertion needed and the routing does
    // not: see `trimTheLead`. What is cut off goes back to the search,
    // which starts where the kept head ends.
    const auto spare =
        (trimTheLead() && chosen.arc.size() > couplerLeadMargin() + 1)
            ? static_cast<std::size_t>(couplerLeadMargin())
            : std::size_t{0};
    wire.arc.assign(chosen.arc.begin(),
                    chosen.arc.end() - static_cast<std::ptrdiff_t>(spare));
    wire.objective.source =
        spare == 0 || wire.arc.empty()
            ? chosen.arcEnd
            : PathPoint{.x = wire.arc.back().x,
                        .y = wire.arc.back().y,
                        .heading = chosen.arcEnd.heading,
                        .primitive = wire.arc.back().primitive};
    // The run is a length in layout units, and `straightStart` counts the
    // cells that span it **along an axis** (`cellsFor`, which says so). The
    // router walks that many cells along the heading, and a diagonal step
    // spans the square root of two, so a resonator leaving a diagonal
    // coupler was held to a run half again as long as the rule asks — 155
    // units against 100 on 17q. `cellsOn` is the conversion the coupler's
    // own run and body already go through.
    //
    // Whether there is such a run at all is `resonatorStub`: the lead is
    // already the rule's length, and the prototype forces nothing beyond it.
    wire.startStub = !resonatorStub() ? 0U
                     : couplerStubs() ? cellsOn(tuning_.straightStart,
                                                chosen.arcEnd.heading)
                                      : tuning_.straightStart;
    wire.fixed.clear();
    wire.couplerAtSource = index;
    wire.routed = false;
    wire.tooShort = false;
    wire.tooLong = false;
    coupler.anchor = chosen.anchor;
    wire.way = chosen.way;
    wire.drawn = true;
    foreignRoomStale_ = true;
    for (const auto cell : chosen.body) {
      bodies_.set(cell, true);
    }
    place(wire);
  }

  /// Say where a coupler sits and what is left of its resonator.
  void sayCoupler(const std::vector<Wire>& wires,
                  const std::uint32_t index) const {
    const auto& coupler = couplers_[index];
    const auto& wire = wires[coupler.wire];
    const auto& option = coupler.options[coupler.chosen];
    const auto label = wire.targetPort < ports_.size()
                           ? ports_[wire.targetPort].label
                           : std::string("?");
    const auto way = lengthInUnits(wire.way);
    tell(std::format(
        "[Coupler Insertion] Coupler for '{} -> {}' inserted at ({}, {}) "
        "with angle {}°, resonator leaves on {}° ({}), search starts on "
        "{}°, offset {} of the nearest launcher, resonator "
        "port {} at ({},{}), "
        "guarded {}, {:.0f} units of way + "
        "{:.0f} to the port = {:.0f}, target {:.0f}",
        wireId(wire), label, option.centre.x, option.centre.y,
        degreesOfHeading(option.couplerOrientation),
        option.way.empty() ? 0 : degreesOfHeading(option.way.front().heading),
        (!option.way.empty() && routing::isDiagonal(option.way.front().heading))
            ? "diagonal"
            : "axial",
        degreesOfHeading(option.arcEnd.heading), option.offset,
        option.secondPort ? 2 : 1, option.anchor.x, option.anchor.y,
        option.guarded, way,
        wire.anchorGapUnits, way + wire.anchorGapUnits,
        tuning_.targetLengthUnits));
  }

  // --------------------------------------------- Under the feedline
  // constraints

  /// The sweep of a pass under the feedline constraints, in ring order: the
  /// edge of a chain that arrives at a coupler comes right before that
  /// coupler's resonator, the edge that leaves it right after, as the
  /// prototype builds its ring of objectives.
  [[nodiscard]] std::vector<std::uint32_t>
  ringWithEdges(const std::vector<Wire>& wires,
                const std::vector<std::uint32_t>& outer) const {
    std::vector<std::uint32_t> members;
    for (const auto key : outer) {
      const auto& wire = wires[key];
      const auto coupler = wire.couplerAtSource;
      if (coupler != NO_OWNER && couplers_[coupler].edgeIn != NO_OWNER &&
          wires[couplers_[coupler].edgeIn].terminal) {
        members.push_back(couplers_[coupler].edgeIn);
      }
      members.push_back(key);
      if (coupler != NO_OWNER && couplers_[coupler].edgeOut != NO_OWNER &&
          wires[couplers_[coupler].edgeOut].terminal) {
        members.push_back(couplers_[coupler].edgeOut);
      }
    }
    return members;
  }

  /// Tell every conventional wire which edge of a chain it may cross: the
  /// edge between the two couplers whose resonators stand on either side of
  /// it in the ring. It crosses that one at a right angle and no other.
  void assignBridges(std::vector<Wire>& wires,
                     const std::vector<std::uint32_t>& outer) const {
    const auto ring = static_cast<std::uint32_t>(outer.size());
    if (ring == 0) {
      return;
    }
    std::unordered_map<std::uint32_t, std::uint32_t> placeOf;
    for (std::uint32_t at = 0; at < ring; ++at) {
      placeOf.emplace(wires[outer[at]].slot, at);
    }
    for (const auto& edge : edges_) {
      if (edge.terminal) {
        continue;
      }
      const auto& points = chains_[edge.chain];
      const auto from =
          placeOf.find(wires[couplers_[points[edge.from].coupler].wire].slot);
      const auto to =
          placeOf.find(wires[couplers_[points[edge.to].coupler].wire].slot);
      if (from == placeOf.end() || to == placeOf.end()) {
        continue;
      }
      for (auto at = (from->second + 1) % ring; at != to->second;
           at = (at + 1) % ring) {
        auto& wire = wires[outer[at]];
        if (!wire.resonator) {
          wire.bridged = edge.wire;
        }
      }
    }
  }

  /// What a pass under the feedline constraints starts with: every wire of
  /// it drawn again, and the crossing rule built from the edges between
  /// couplers.
  void beginPass(std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members, const Pass& pass) {
    // Every wire is placed again here, so a resonator may stand anywhere.
    foreignRoomStale_ = true;
    feedlinePass_ = pass.feedlines;
    if (!pass.feedlines) {
      router_.clearOrthogonalConstraints();
      return;
    }
    if (!pass.onlyUnsettled) {
      for (const auto member : members) {
        wires[member].routed = false;
      }
    }
    rebuildCrossingRule(wires);
    {
      const auto origin = [](const char* name) {
        return envSet(name) != nullptr ? "env" : "default";
      };
      say(std::format("feedline routing settings: a resonator runs the "
                      "rule's straight again after its lead {} ({}), fixed "
                      "places fenced {} ({}), crossing rule {} ({}), the "
                      "exit heading tested {} ({}), a way that misses the "
                      "wire's bridge refused {} ({}), lane polygon priced {} "
                      "({}), the terminal edges' coupler runs guarded {} "
                      "({}) by {} cells more ({})",
                      resonatorStub() ? "yes" : "no",
                      origin("SCPD_RESONATOR_STUB"),
                      fenceFixed() ? "yes" : "no", origin("SCPD_FENCE_FIXED"),
                      orthoCrossing() ? "on" : "off",
                      origin("SCPD_ORTHO_CROSSING"),
                      crossingExitHeading() ? "yes" : "no",
                      origin("SCPD_CROSSING_EXIT_HEADING"),
                      bridgeCheck() ? "yes" : "no", origin("SCPD_BRIDGE_CHECK"),
                      feedlineLane() ? "yes" : "no",
                      origin("SCPD_FEEDLINE_LANE"),
                      terminalStubGuard() ? "yes" : "no",
                      origin("SCPD_TERMINAL_STUB_GUARD"), terminalStubExtra(),
                      origin("SCPD_TERMINAL_STUB_EXTRA")));
    }
    if (feedlinePass_ && verbosity_ >= 1) {
      std::string order;
      for (const auto member : members) {
        const auto& wire = wires[member];
        order += (order.empty() ? "" : " ") + wireId(wire) +
                 (wire.feedline
                      ? std::format("[{}{}]", startsAtLauncher(wire) ? "F" : "",
                                    endsAtLauncher(wire) ? "L" : "")
                      : (wire.resonator ? "(r)" : ""));
      }
      say(std::format("the ring the feedline pass sweeps: {}", order));
    }
}

  /// The crossing rule from the edges between couplers as they are drawn
  /// now. Built when a pass under the feedline constraints starts and
  /// whenever a snapshot is put back, so that the rule the count and the
  /// searches use is never the one of a trial thrown away.
  void rebuildCrossingRule(const std::vector<Wire>& wires) {
    // Every edge of a chain, the terminal ones included: a wire that may
    // pass a terminal edge has to cross it at a right angle like any other,
    // or the crossing is one no air bridge can be built over.
    std::vector<Path> crossable;
    for (const auto& edge : edges_) {
      const auto& wire = wires[edge.wire];
      // The prototype builds the rule from the edges between couplers and
      // leaves the terminal ones out of it (`is_first_last_feedline`,
      // `FinalGrid.cpp:10677`): a terminal edge runs from the border to the
      // first coupler, and a rule over it closes the plane from outside in.
      if (edge.terminal && feedlineLikePrototype()) {
        continue;
      }
      if (wire.drawn) {
        crossable.push_back(wire.way);
      }
    }
    if (!orthoCrossing()) {
      router_.clearOrthogonalConstraints();
      return;
    }
    router_.buildOrthogonalConstraints(crossable, {}, CROSSING_REACH);
  }

  /// Fence a search by the feedlines and price the room around them, as
  /// the prototype's feedline routing does: every edge inflated by the
  /// clearance stands in the way, except the edge a conventional wire
  /// bridges, which it may cross once at a right angle, and the edges of a
  /// resonator's own coupler, which the meeting leaves open. The room
  /// around every edge costs five times the wire price.
  void constrainByFeedlines(const Wire& wire, const std::vector<Wire>& wires,
                            const Pass& pass) {
    if (!pass.feedlines) {
      router_.setSingleCrossingFeedline(nullptr, 0, 1);
      return;
    }
    for (const auto& edge : edges_) {
      const auto& other = wires[edge.wire];
      // The run a terminal edge has to make at its coupler, closed whether
      // or not the edge is drawn: see `terminalStubGuard`.
      if (edge.terminal && other.key != wire.key && terminalStubGuard()) {
        guardTerminalStub(wire, other);
      }
      if (!other.drawn || other.key == wire.key || other.key == wire.bridged) {
        continue;
      }
      // The first and last edge of a chain run from a launcher on the border
      // to the first coupler, which is deep in the fan-in. Fenced, such an
      // edge cuts the plane in two from the border inward, and every wire on
      // the far side of it has to go around the coupler at its end.
      //
      // The prototype leaves them out of the fence of **every** wire, not
      // only of the ones that may cross them: `forbidden_feedline_paths` is
      // built with `if (feedline.is_first_last_feedline) continue;` in both
      // its branches (`FinalGrid.cpp:3730`, `:3744`), the resonators' branch
      // included. Fencing a resonator by them was ours.
      if (edge.terminal && (feedlineLikePrototype() ||
                            (!wire.resonator && !wire.feedline))) {
        continue;
      }
      fence(wire, {&other});
    }
    // A feedline edge keeps the clearance from every resonator; the meeting
    // at its own coupler stays open.
    if (wire.feedline) {
      for (const auto& other : wires) {
        if (other.resonator && other.drawn && other.key != wire.key) {
          fence(wire, {&other});
        }
      }
    }
    // The bridged edge is crossed under the crossing rule alone, at a right
    // angle within its band, as the prototype crosses it: the one-crossing
    // overlay the router offers asks for a straight run on the far side
    // that a port beside the feedline cannot give.
    router_.setSingleCrossingFeedline(nullptr, 0, 1);
    priceTheEdges(wire, wires, pass);
  }

  /// The room around an edge, five times the price of the room around any
  /// other wire, as the prototype prices `all_feedline_paths`
  /// (`FinalGrid.cpp:11733`). Inflated by the clearance, which is the
  /// radius `compute_proximity_grid` is handed there.
  ///
  /// **The terminal edges are priced with the rest.** The prototype leaves
  /// them out of this list and closes them hard instead
  /// (`forbidden_paths_start_end`, `FinalGrid.cpp:11747`), so under it they
  /// need no price. Here they are drawn again like any other edge, and a
  /// wire that is neither fenced by them nor made to pay for their room
  /// treats a launcher run as free ground (user, 2026-09-30).
  ///
  /// Its own method because the refinement builds the room price after the
  /// constraints and so has to stamp these again on top of it: see
  /// `refinePriceLast`.
  void priceTheEdges(const Wire& wire, const std::vector<Wire>& wires,
                     const Pass& pass) {
    if (!pass.feedlines) {
      return;
    }
    const auto strong = static_cast<std::uint8_t>(
        std::min<std::uint32_t>(127U, 5U * tuning_.wireProximityPenalty));
    for (const auto& edge : edges_) {
      const auto& other = wires[edge.wire];
      if (other.drawn && other.key != wire.key) {
        stampDisc(other.way, tuning_.clearance, strong);
      }
    }
  }

  /// Close, in the corridor of an edge search, the coupler-end runs of the
  /// terminal edges of every **other** chain whose edges are committed:
  /// the run into the first coupler and the run out of the last, each with
  /// `terminalStubExtra` cells beyond it and the clearance around the
  /// whole. The chain being searched is left out — its own terminal edges
  /// are fenced by `fenceCommittedEdges` and `prefixFence_` as the rules
  /// there say, and its couplers may still move. See `terminalStubGuard`.
  void guardSettledTerminalRuns(const Edge& edge) {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto close = [&](const Path& cells) {
      alongDisc(cells, stencil,
                [&](const std::int64_t x, const std::int64_t y) {
                  corridor_.set(static_cast<std::size_t>((y * width) + x),
                                true);
                });
    };
    for (std::uint32_t other = 0; other < chains_.size(); ++other) {
      if (other == edge.chain || other >= chainEdgePaths_.size()) {
        continue;
      }
      const auto& points = chains_[other];
      const auto& ways = chainEdgePaths_[other];
      if (points.size() < 3 || ways.size() + 1 != points.size()) {
        continue;
      }
      // The first edge, committed, from a launcher into the first coupler.
      if (points.front().fixed && !points[1].fixed && !ways.front().empty()) {
        const auto run = couplerRunOf(points[1].coupler);
        close(router_.straightStub(targetOf(points[1]), true,
                                   run + terminalStubExtra()));
      }
      // The last edge, committed, out of the last coupler to a launcher.
      const auto last = points.size() - 2;
      if (points.back().fixed && !points[last].fixed && !ways.back().empty()) {
        const auto run = couplerRunOf(points[last].coupler);
        close(router_.straightStub(sourceOf(points[last]), false,
                                   run + terminalStubExtra()));
      }
    }
  }

  /// Close the straight run a terminal edge makes at its coupler — into the
  /// first coupler for an edge that starts at a launcher, out of the last
  /// for one that ends at a launcher — and `terminalStubExtra` cells beyond
  /// it, inflated by the clearance, to the search of `wire`. The launcher
  /// end needs nothing: the port band walls it. See `terminalStubGuard`.
  void guardTerminalStub(const Wire& wire, const Wire& edge) {
    Path cells;
    if (startsAtLauncher(edge)) {
      const auto in = router_.straightStub(edge.objective.target, true,
                                           edge.endStub + terminalStubExtra());
      cells.insert(cells.end(), in.begin(), in.end());
    }
    if (endsAtLauncher(edge)) {
      const auto out = router_.straightStub(
          edge.objective.source, false,
          startStubOf(edge, tuning_.straightStart) + terminalStubExtra());
      cells.insert(cells.end(), out.begin(), out.end());
    }
    if (cells.empty()) {
      return;
    }
    closeRoomOf(wire, edge, cells);
    openFixedPlaces(wire);
  }

  /// A test seam and nothing else. `SCPD_PROBE_ONLY_UNSETTLED=<slot>` flags
  /// that one wire of the ring, sweeps the whole ring once under
  /// `Pass::onlyUnsettled` with no relaxation, says how many wires were
  /// tried and whose ways moved, and puts everything back — so with the
  /// variable set the stage ends on exactly what it ends on without it.
  /// `FinalRouter.OnlyUnsettledRedrawsOneWire` reads the line; the `Driver`
  /// is not reachable from a test any other way. Read from the environment
  /// on every call, not once, so a test may set it and unset it again.
  void probeOnlyUnsettled(std::vector<Wire>& wires,
                          const std::vector<std::uint32_t>& members,
                          const Pass& pass) {
    const int slot = envWhole("SCPD_PROBE_ONLY_UNSETTLED", -1);
    if (slot < 0) {
      return;
    }
    const auto taken = snapshot(wires);
    std::vector<Path> before;
    before.reserve(members.size());
    std::uint32_t flagged = 0;
    for (const auto member : members) {
      auto& wire = wires[member];
      before.push_back(wire.way);
      if (!wire.feedline && !wire.inner &&
          wire.slot == static_cast<std::uint32_t>(slot)) {
        wire.routed = false;
        ++flagged;
      }
    }
    Pass local = pass;
    local.rounds = 1;
    local.maxRelaxation = 0;
    local.name = "probe";
    local.onlyUnsettled = true;
    const auto verbosity = verbosity_;
    verbosity_ = 0;
    const auto quiet = progress_;
    progress_ = {};
    sweep(wires, members, local);
    progress_ = quiet;
    verbosity_ = verbosity;
    const auto tried = rounds_.empty()
                           ? 0U
                           : rounds_.back().normal + rounds_.back().relaxed +
                                 static_cast<std::uint32_t>(
                                     rounds_.back().failed.size());
    std::uint32_t changed = 0;
    std::string moved;
    for (std::size_t at = 0; at < members.size(); ++at) {
      const auto& now = wires[members[at]].way;
      const bool same =
          now.size() == before[at].size() &&
          std::ranges::equal(now, before[at],
                             [](const PathPoint& a, const PathPoint& b) {
                               return a.samePlace(b) && a.heading == b.heading;
                             });
      if (!same) {
        ++changed;
        moved += (moved.empty() ? "" : ", ") + wireId(wires[members[at]]);
      }
    }
    say(std::format("probe onlyUnsettled: wire {} flagged ({} member{}), {} "
                    "wire{} tried, {} way{} moved{}",
                    slot, flagged, flagged == 1 ? "" : "s", tried,
                    tried == 1 ? "" : "s", changed, changed == 1 ? "" : "s",
                    moved.empty() ? std::string{} : ": " + moved));
    restore(wires, taken);
  }

  // ------------------------------------------------------------- The repair

  /// The state a trial of the repair changes, and puts back.
  struct Snapshot {
    std::vector<Wire> wires;
    std::vector<std::size_t> chosen;
    std::vector<PathPoint> anchors;
    grid::BitGrid bodies;
  };

  [[nodiscard]] Snapshot snapshot(const std::vector<Wire>& wires) const {
    Snapshot taken{
        .wires = wires, .chosen = {}, .anchors = {}, .bodies = bodies_};
    for (const auto& coupler : couplers_) {
      taken.chosen.push_back(coupler.chosen);
      taken.anchors.push_back(coupler.anchor);
    }
    return taken;
  }

  /// Put a snapshot back, and the field with it: everything is charged
  /// again from what the wires hold.
  void restore(std::vector<Wire>& wires, const Snapshot& taken) {
    wires = taken.wires;
    bodies_ = taken.bodies;
    for (std::size_t index = 0; index < couplers_.size(); ++index) {
      couplers_[index].chosen = taken.chosen[index];
      couplers_[index].anchor = taken.anchors[index];
    }
    field_ = Field(scene_.router, tuning_.clearance);
    for (auto& wire : wires) {
      wire.placed = false;
      wire.endsOnly = false;
      if (!wire.way.empty()) {
        place(wire);
      } else if (!wire.fixed.empty()) {
        field_.chargeFixed(wire.fixed);
        wire.endsOnly = true;
      }
    }
    if (feedlinePass_) {
      rebuildCrossingRule(wires);
    }
  }

  /// Turn a placed coupler to another of its options and draw its two
  /// edges again. Nothing is changed when an edge finds no way.
  [[nodiscard]] bool turnCoupler(std::vector<Wire>& wires,
                                 const std::uint32_t index,
                                 const std::size_t option) {
    auto& coupler = couplers_[index];
    const auto taken = snapshot(wires);
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key != NO_OWNER) {
        lift(wires[key]);
        wires[key].way.clear();
        wires[key].drawn = false;
      }
    }
    applyOption(wires, index, option);
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key == NO_OWNER) {
        continue;
      }
      auto& wire = wires[key];
      const auto& edge = edges_[wire.edge];
      const auto& points = chains_[edge.chain];
      wire.objective.source = sourceOf(points[edge.from]);
      wire.objective.target = targetOf(points[edge.to]);
      wire.fixed.clear();
      wire.way = routeEdge(wires, wire.objective, edge);
      if (wire.way.empty()) {
        restore(wires, taken);
        return false;
      }
      wire.drawn = true;
      wire.routed = false;
      place(wire);
    }
    return true;
  }

  /// The wires a turned coupler unsettles: its resonator, its two edges,
  /// and every wire of the ring between it and the next resonator on
  /// either side. The prototype's dirty walk.
  [[nodiscard]] std::vector<std::uint32_t>
  unsettledBy(const std::vector<Wire>& wires,
              const std::vector<std::uint32_t>& members,
              const std::uint32_t index) const {
    const auto& coupler = couplers_[index];
    std::vector<std::uint32_t> dirty{coupler.wire};
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key != NO_OWNER) {
        dirty.push_back(key);
      }
    }
    const auto total = members.size();
    const auto found = std::ranges::find(members, coupler.wire);
    if (found == members.end() || total < 2) {
      return dirty;
    }
    const auto at = static_cast<std::size_t>(found - members.begin());
    for (const int direction : {1, -1}) {
      for (std::size_t step = 1; step < total; ++step) {
        const auto place =
            (at + total + (static_cast<std::size_t>(direction) * step)) % total;
        const auto& wire = wires[members[place]];
        if (wire.resonator) {
          break;
        }
        dirty.push_back(wire.key);
      }
    }
    return dirty;
  }

  /// Phase 4's repair: while the feedlines leave fails, turn a coupler near
  /// them to another of its options and route what that unsettles again;
  /// keep the turn that leaves strictly fewer fails. The prototype's
  /// `run_final_routing_feedline_choices_parallel`, one trial at a time.
  ///
  /// @returns The fails the repair ends on.
  std::uint32_t repair(std::vector<Wire>& wires,
                       const std::vector<std::uint32_t>& members,
                       const std::vector<std::uint32_t>& every,
                       const Pass& pass) {
    auto current = failsOf(wires, every);
    std::uint32_t trials = 0;
    std::set<std::pair<std::uint32_t, std::size_t>> tried;
    Pass again = pass;
    again.rounds = 1;
    again.name = "feedline repair";
    while (current.total() > 0 && trials < tuning_.repairTrials) {
      // The couplers near the fails: of a failing resonator or edge, and
      // of the nearest resonators on either side of a failing wire, with
      // their chain neighbours.
      std::vector<std::uint32_t> candidates;
      const auto add = [&](const std::uint32_t index) {
        if (index != NO_OWNER &&
            std::ranges::find(candidates, index) == candidates.end()) {
          candidates.push_back(index);
        }
      };
      const auto near = [&](const Wire& wire) {
        if (wire.feedline) {
          add(wire.couplerAtSource);
          add(wire.couplerAtTarget);
          return;
        }
        if (wire.couplerAtSource != NO_OWNER) {
          add(wire.couplerAtSource);
          return;
        }
        const auto found = std::ranges::find(members, wire.key);
        if (found == members.end()) {
          return;
        }
        const auto at = static_cast<std::size_t>(found - members.begin());
        const auto total = members.size();
        for (const int direction : {1, -1}) {
          for (std::size_t step = 1; step < total; ++step) {
            const auto place =
                (at + total + (static_cast<std::size_t>(direction) * step)) %
                total;
            const auto& other = wires[members[place]];
            if (other.resonator) {
              add(other.couplerAtSource);
              break;
            }
          }
        }
      };
      for (const auto key : every) {
        const auto& wire = wires[key];
        if (!wire.feasible) {
          continue;
        }
        const bool failing =
            !wire.drawn || !wire.routed || wire.tooShort || wire.tooLong;
        if (failing) {
          near(wire);
        }
      }
      const auto direct = candidates;
      for (const auto index : direct) {
        for (const auto& points : chains_) {
          for (std::size_t at = 0; at < points.size(); ++at) {
            if (points[at].coupler != index) {
              continue;
            }
            if (at > 0) {
              add(points[at - 1].coupler);
            }
            if (at + 1 < points.size()) {
              add(points[at + 1].coupler);
            }
          }
        }
      }
      // A wave: up to two untried options per candidate, the cheapest first.
      struct Trial {
        std::uint32_t coupler = 0;
        std::size_t option = 0;
        std::uint32_t fails = 0;
      };
      std::vector<Trial> wave;
      for (const auto index : candidates) {
        const auto& coupler = couplers_[index];
        std::vector<std::size_t> options;
        for (std::size_t option = 0; option < coupler.options.size();
             ++option) {
          if (option != coupler.chosen && !tried.contains({index, option})) {
            options.push_back(option);
          }
        }
        std::ranges::stable_sort(options, {}, [&](const std::size_t option) {
          return coupler.options[option].cost;
        });
        for (std::size_t k = 0; k < 2 && k < options.size(); ++k) {
          wave.push_back({.coupler = index,
                          .option = options[k],
                          .fails = current.total()});
        }
      }
      if (wave.empty()) {
        say("feedline repair: nothing left to try");
        break;
      }
      const auto taken = snapshot(wires);
      for (auto& trial : wave) {
        if (trials >= tuning_.repairTrials) {
          break;
        }
        ++trials;
        tried.insert({trial.coupler, trial.option});
        if (!turnCoupler(wires, trial.coupler, trial.option)) {
          trial.fails = std::numeric_limits<std::uint32_t>::max();
          tell(std::format(
              "repair trial {}: coupler {} option {}: its edges find no way",
              trials, trial.coupler, trial.option));
          continue;
        }
        for (const auto key : unsettledBy(wires, members, trial.coupler)) {
          wires[key].routed = false;
        }
        const auto verbosity = verbosity_;
        verbosity_ = 0;
        const auto quiet = progress_;
        progress_ = {};
        sweep(wires, members, again);
        progress_ = quiet;
        verbosity_ = verbosity;
        trial.fails = failsOf(wires, every).total();
        tell(std::format(
            "repair trial {}: coupler {} option {}: {} fails against {}",
            trials, trial.coupler, trial.option, trial.fails, current.total()));
        restore(wires, taken);
      }
      const auto best = std::ranges::min_element(wave, {}, &Trial::fails);
      if (best == wave.end() || best->fails >= current.total()) {
        say(std::format(
            "feedline repair: {} trials, none with fewer than {} fails", trials,
            current.total()));
        if (trials >= tuning_.repairTrials) {
          break;
        }
        continue;
      }
      if (!turnCoupler(wires, best->coupler, best->option)) {
        continue;
      }
      for (const auto key : unsettledBy(wires, members, best->coupler)) {
        wires[key].routed = false;
      }
      sweep(wires, members, again);
      current = failsOf(wires, every);
      say(std::format(
          "feedline repair: coupler {} turned to option {} after {} trials | "
          "Fails: {}",
          best->coupler, best->option, trials, current.total()));
    }
    // Everything down again, so that the field says what the canvas holds.
    for (auto& wire : wires) {
      place(wire);
    }
    return current.total();
  }

  // --------------------------------------------------- The targeted repair

  /// One segment of one chain the targeted repair re-searches: waypoints
  /// `lo..hi` of `chains_[chain]`, `lo < hi`, the two ends **fixed** on the
  /// option they stand on (a launcher is fixed anyway), the interior
  /// `lo+1..hi-1` re-searched and the edges `lo..hi-1` re-routed. A segment
  /// with no interior (`hi == lo + 1`) re-routes one edge and re-searches
  /// nothing — allowed, it is the cheapest trial there is.
  struct Segment {
    std::uint32_t chain = 0;
    std::size_t lo = 0;
    std::size_t hi = 0;
    /// The fails it was built from, by wire key, and what each was.
    std::vector<std::uint32_t> fails;
    std::vector<std::string> because;
  };

  /// A coupler as the rest of the log names it: by its resonator, in
  /// quotes, as the `room` lines and `sayCoupler` do — never by its index.
  [[nodiscard]] std::string couplerName(const std::vector<Wire>& wires,
                                        const std::uint32_t coupler) const {
    if (coupler == NO_OWNER || coupler >= couplers_.size()) {
      return "launcher";
    }
    return std::format("'{}'", wireId(wires[couplers_[coupler].wire]));
  }

  [[nodiscard]] std::string segmentName(const std::vector<Wire>& wires,
                                        const Segment& segment) const {
    std::string interior;
    for (std::size_t at = segment.lo + 1; at < segment.hi; ++at) {
      const auto& point = chains_[segment.chain][at];
      interior += std::format("{}{}", interior.empty() ? "" : " ",
                              point.fixed ? std::string("launcher")
                                          : couplerName(wires, point.coupler));
    }
    return std::format("chain {} waypoints {}..{} ({} edge{}, interior "
                       "coupler{} {})",
                       segment.chain, segment.lo, segment.hi,
                       segment.hi - segment.lo,
                       segment.hi - segment.lo == 1 ? "" : "s",
                       segment.hi - segment.lo <= 2 ? "" : "s",
                       interior.empty() ? "none" : interior);
  }

  /// **The triage**: every fail of the feedline routing traced to the chain
  /// edges and couplers that can be its cause, and those grouped into the
  /// segments of one chain the re-search works on. Nothing here decides
  /// anything about geometry; it only says where to look.
  ///
  /// - An **open** wire names its partners through `conflictsIn`, the own
  ///   cell of each conflict included. Blamed: the wire's bridged edge and
  ///   each partner's (and the edge itself where either is a feedline),
  ///   every edge whose way comes within `couplerReach` of the conflict
  ///   cell, and every coupler whose lead or pad lies within the clearance
  ///   of it — the lane pairs of 45q and 57q lean on a pad, 69q's resonator
  ///   9 lies one cell from its own lead (handover-cpw-coupler-insertion,
  ///   *The room rules*).
  /// - A **crossing** wire names the cell where it breaks the rule; blamed
  ///   is every edge within `CROSSING_REACH + 1` of it, which is the edge
  ///   whose rule it broke.
  /// - An **unrouted** wire blames its bridged edge and the couplers of the
  ///   two resonators that flank it in the ring, as the blind repair's
  ///   `near` did; a resonator blames its own coupler, an edge itself.
  /// - Two **edges of different chains that cross** (`feedlineCrossingPairs`)
  ///   blame both edges, each counted as a fail of its own wire.
  /// - Every open, crossing or unrouted wire also blames the edges within
  ///   the clearance of its **search start**: a wire whose first cells are
  ///   closed is dead on arrival in every round, whatever else the fail
  ///   says.
  ///
  /// A blamed edge marks its two waypoints, a blamed coupler its one. Per
  /// chain the maximal runs of marked waypoints are widened by
  /// `researchGrow` at each end, clamped to the chain, merged where they
  /// touch, and become the segments; a segment carries the fails that marked
  /// its waypoints. Segments are ordered by fails carried, then by chain.
  /// Cross-chain fails yield one segment per chain, searched one after the
  /// other, so the second sees the first's result.
  ///
  /// One `tell` per fail and per segment, one `say` per chip.
  [[nodiscard]] std::vector<Segment>
  blameFails(std::vector<Wire>& wires, const std::vector<std::uint32_t>& members,
             const Fails& fails) {
    std::vector<std::vector<std::vector<std::uint32_t>>> marks(chains_.size());
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      marks[chain].assign(chains_[chain].size(), {});
    }
    std::map<std::uint32_t, std::string> because;
    std::set<std::uint32_t> unblamed;
    const auto note = [&](const std::uint32_t key, const std::string& what) {
      auto& text = because[key];
      text += (text.empty() ? "" : "; ") + what;
    };
    const auto mark = [&](const std::uint32_t chain, const std::size_t at,
                          const std::uint32_t fail) {
      auto& list = marks[chain][at];
      if (std::ranges::find(list, fail) == list.end()) {
        list.push_back(fail);
      }
    };
    const auto add = [](std::string& named, const std::string& name) {
      if (named.find(name) == std::string::npos) {
        named += (named.empty() ? "" : ", ") + name;
      }
    };
    const auto blameEdge = [&](const std::size_t index, const std::uint32_t fail,
                               std::string& named) {
      if (index >= edges_.size()) {
        return;
      }
      const auto& edge = edges_[index];
      mark(edge.chain, edge.from, fail);
      mark(edge.chain, edge.to, fail);
      add(named, std::format("{} (chain {} edge {})", wireId(wires[edge.wire]),
                             edge.chain, edge.from));
    };
    const auto blameCoupler = [&](const std::uint32_t coupler,
                                  const std::uint32_t fail, std::string& named) {
      if (coupler == NO_OWNER || coupler >= chainOfCoupler_.size()) {
        return;
      }
      const auto [chain, at] = chainOfCoupler_[coupler];
      if (chain == NO_OWNER) {
        return;
      }
      mark(chain, at, fail);
      add(named, std::format("coupler {} (chain {} waypoint {})",
                             couplerName(wires, coupler), chain, at));
    };
    // The edge a wire key stands for, or none.
    const auto edgeOfWire = [&](const std::uint32_t key) {
      return (key != NO_OWNER && key < wires.size() && wires[key].feedline)
                 ? static_cast<std::size_t>(wires[key].edge)
                 : edges_.size();
    };
    // Every coupler whose lead or pad lies within the clearance of a cell.
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto couplersNear = [&](const PathPoint& cell, const std::uint32_t fail,
                                  std::string& named) {
      const auto limit = static_cast<double>(tuning_.clearance);
      for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
        const auto& option = couplers_[index].options[couplers_[index].chosen];
        bool near = distanceToWay(cell, option.arc) <= limit;
        for (std::size_t at = 0; !near && at < option.body.size(); ++at) {
          const auto body = static_cast<std::int64_t>(option.body[at]);
          near = std::hypot(static_cast<double>(body % width) - cell.x,
                            static_cast<double>(body / width) - cell.y) <=
                 limit;
        }
        if (near) {
          blameCoupler(index, fail, named);
        }
      }
    };
    // The edges within the clearance of a wire's search start — the source
    // moved along its heading by the stub, where the router begins. A wire
    // whose first cells are closed is dead on arrival in every round, so an
    // edge standing there is its cause whatever else the fail says: 17q's
    // wire 31 crosses f9 and is blamed on f9, but it finds no way because
    // f8 lies 19 cells from its start (`deadOnArrival`, 2026-10-04).
    const auto blameAtTheStart = [&](const Wire& wire, const std::uint32_t fail,
                                     std::string& named) {
      const auto stub = static_cast<std::int64_t>(
          startStubOf(wire, tuning_.straightStart));
      const auto& source = wire.objective.source;
      const auto v = routing::headingVector(source.heading);
      const auto x = static_cast<std::int64_t>(source.x) + (v.dx * stub);
      const auto y = static_cast<std::int64_t>(source.y) + (v.dy * stub);
      if (x < 0 || y < 0 || x >= width ||
          y >= static_cast<std::int64_t>(scene_.router.height)) {
        return;
      }
      const PathPoint start{.x = static_cast<std::uint32_t>(x),
                            .y = static_cast<std::uint32_t>(y),
                            .heading = source.heading,
                            .primitive = 0};
      for (const auto& [edge, far] :
           edgesNear(wires, start, static_cast<double>(tuning_.clearance) + 1.0)) {
        if (edges_[edge].wire != wire.key) {
          blameEdge(edge, fail, named);
        }
      }
    };
    const auto said = [&](const std::uint32_t key, const std::string& what,
                          const std::string& named) {
      if (named.empty()) {
        unblamed.insert(key);
      }
      tell(std::format("targeted repair: fail {} ({}) blames {}",
                       wireId(wires[key]), what,
                       named.empty() ? std::string("nothing within reach")
                                     : named));
    };

    for (const auto key : fails.openKeys) {
      auto& wire = wires[key];
      std::vector<std::uint32_t> blockers;
      std::vector<PathPoint> where;
      lift(wire);
      conflictsIn(wire.way, wire, wires, &blockers, &where);
      place(wire);
      std::string named;
      std::string with;
      if (wire.feedline) {
        blameEdge(wire.edge, key, named);
      }
      blameEdge(edgeOfWire(wire.bridged), key, named);
      for (std::size_t at = 0; at < blockers.size(); ++at) {
        const auto& partner = wires[blockers[at]];
        with += std::format("{}{}", with.empty() ? "" : " and ", wireId(partner));
        if (partner.feedline) {
          blameEdge(partner.edge, key, named);
        }
        blameEdge(edgeOfWire(partner.bridged), key, named);
        if (at < where.size()) {
          const auto& cell = where[at];
          with += std::format(" at ({},{})", cell.x, cell.y);
          for (const auto& [edge, far] : edgesNear(wires, cell, couplerReach())) {
            blameEdge(edge, key, named);
          }
          couplersNear(cell, key, named);
        }
      }
      blameAtTheStart(wire, key, named);
      const auto what = std::format("open with {}", with.empty() ? "-" : with);
      note(key, what);
      said(key, what, named);
    }
    for (const auto key : fails.crossingKeys) {
      const auto& wire = wires[key];
      std::string named;
      std::string what = "crossing nowhere";
      if (const auto cell = crossingCellOf(wire); cell.has_value()) {
        what = std::format("crossing at ({},{})", cell->x, cell->y);
        // The rule's halo is a square of `CROSSING_REACH` cells around each
        // straight cell, so its corner lies `CROSSING_REACH * sqrt 2` away
        // in the distance `edgesNear` measures; a cell in the corner of the
        // halo blamed nothing at a reach of 11 (17q's wire 2, 2026-10-04).
        for (const auto& [edge, far] : edgesNear(
                 wires, *cell,
                 (static_cast<double>(CROSSING_REACH) * std::numbers::sqrt2) +
                     1.0)) {
          blameEdge(edge, key, named);
        }
      }
      blameAtTheStart(wire, key, named);
      note(key, what);
      said(key, what, named);
    }
    for (const auto key : fails.unroutedKeys) {
      const auto& wire = wires[key];
      std::string named;
      if (wire.feedline) {
        blameEdge(wire.edge, key, named);
      } else if (wire.couplerAtSource != NO_OWNER) {
        blameCoupler(wire.couplerAtSource, key, named);
      } else {
        blameEdge(edgeOfWire(wire.bridged), key, named);
        const auto found = std::ranges::find(members, key);
        if (found != members.end()) {
          const auto at = static_cast<std::size_t>(found - members.begin());
          const auto total = members.size();
          for (const int direction : {1, -1}) {
            for (std::size_t step = 1; step < total; ++step) {
              const auto place =
                  (at + total + (static_cast<std::size_t>(direction) * step)) %
                  total;
              const auto& other = wires[members[place]];
              if (other.resonator) {
                blameCoupler(other.couplerAtSource, key, named);
                break;
              }
            }
          }
        }
      }
      blameAtTheStart(wire, key, named);
      note(key, "unrouted");
      said(key, "unrouted", named);
    }
    for (const auto& [i, j] : feedlineCrossingPairs(wires)) {
      for (const auto [mine, other] : {std::pair{i, j}, std::pair{j, i}}) {
        const auto key = edges_[mine].wire;
        std::string named;
        blameEdge(mine, key, named);
        blameEdge(other, key, named);
        const auto what = std::format("crosses {} of chain {}",
                                      wireId(wires[edges_[other].wire]),
                                      edges_[other].chain);
        note(key, what);
        said(key, what, named);
      }
    }

    // The segments: maximal runs of marked waypoints per chain, widened,
    // clamped, merged where they touch after widening.
    const auto grow = researchGrow();
    std::vector<Segment> segments;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto size = chains_[chain].size();
      std::vector<std::pair<std::size_t, std::size_t>> runs;
      for (std::size_t at = 0; at < size;) {
        if (marks[chain][at].empty()) {
          ++at;
          continue;
        }
        auto end = at;
        while (end + 1 < size && !marks[chain][end + 1].empty()) {
          ++end;
        }
        const auto lo = at >= grow ? at - grow : 0;
        const auto hi = std::min(size - 1, end + grow);
        if (!runs.empty() && runs.back().second >= lo) {
          runs.back().second = std::max(runs.back().second, hi);
        } else {
          runs.emplace_back(lo, hi);
        }
        at = end + 1;
      }
      for (const auto& [lo, hi] : runs) {
        Segment segment{.chain = chain, .lo = lo, .hi = hi, .fails = {},
                        .because = {}};
        for (std::size_t at = lo; at <= hi; ++at) {
          for (const auto fail : marks[chain][at]) {
            if (std::ranges::find(segment.fails, fail) == segment.fails.end()) {
              segment.fails.push_back(fail);
              segment.because.push_back(because[fail]);
            }
          }
        }
        if (hi <= lo) {
          tell(std::format("targeted repair: chain {} waypoint {} is blamed "
                           "alone and the segment is widened by nothing — no "
                           "edge to route, dropped",
                           chain, lo));
          continue;
        }
        segments.push_back(std::move(segment));
      }
    }
    std::ranges::stable_sort(segments, [](const Segment& a, const Segment& b) {
      if (a.fails.size() != b.fails.size()) {
        return a.fails.size() > b.fails.size();
      }
      if (a.chain != b.chain) {
        return a.chain < b.chain;
      }
      return a.lo < b.lo;
    });
    std::string chainsNamed;
    for (const auto& segment : segments) {
      std::string from;
      for (std::size_t at = 0; at < segment.fails.size(); ++at) {
        from += std::format("{}{} ({})", at == 0 ? "" : ", ",
                            wireId(wires[segment.fails[at]]),
                            segment.because[at]);
      }
      tell(std::format("targeted repair: segment {} from fails {}",
                       segmentName(wires, segment), from));
      chainsNamed += std::format("{}{}", chainsNamed.empty() ? "" : " ",
                                 segment.chain);
    }
    say(std::format("targeted repair: {} fail{} → {} segment{} (chains {}), {} "
                    "fail{} with no edge or coupler within reach",
                    because.size(), because.size() == 1 ? "" : "s",
                    segments.size(), segments.size() == 1 ? "" : "s",
                    chainsNamed.empty() ? "-" : chainsNamed, unblamed.size(),
                    unblamed.size() == 1 ? "" : "s"));
    return segments;
  }

  /// Phase 4's repair, dispatched: the targeted re-search of coupler options
  /// under `repairSearch`, the blind `repair` of before without it. Says
  /// which switches are in force first, as the insertion says its own.
  void repairFeedlines(std::vector<Wire>& wires,
                       const std::vector<std::uint32_t>& members,
                       const std::vector<std::uint32_t>& every,
                       const Pass& pass) {
    {
      const auto from = [](const char* name) {
        return envSet(name) != nullptr ? "env" : "default";
      };
      say(std::format(
          "targeted repair settings: re-search {} ({}), k {} ({}), grow {} "
          "({}), {:.0f}s a segment ({}), {} rounds ({}), jogs {} ({}), sees "
          "chains {} ({}), final sweep {} ({}), budget {} legality tests "
          "(repair_trials), crossing rule {} ({})",
          repairSearch() ? "yes" : "no", from("SCPD_REPAIR_SEARCH"),
          researchK(), from("SCPD_RESEARCH_K"), researchGrow(),
          from("SCPD_RESEARCH_GROW"),
          static_cast<double>(researchBudget().count()) / 1e9,
          from("SCPD_RESEARCH_SECONDS"), researchRounds(),
          from("SCPD_RESEARCH_ROUNDS"), researchJogs() ? "on" : "off",
          from("SCPD_RESEARCH_JOGS"), researchSeesChains() ? "yes" : "no",
          from("SCPD_RESEARCH_SEES_CHAINS"),
          researchFinalSweep() ? "on" : "off",
          from("SCPD_RESEARCH_FINAL_SWEEP"), tuning_.repairTrials,
          orthoCrossing() ? "on" : "off", from("SCPD_ORTHO_CROSSING")) +
          (researchVerbose() ? ", the local passes speak (env)" : ""));
    }
    if (!repairSearch()) {
      static_cast<void>(repair(wires, members, every, pass));
      return;
    }
    researchSegments(wires, members, every, pass);
  }

  /// Hide or show the pads of some couplers in `bodies_`, at the option each
  /// stands on. A segment's interior couplers are hidden for the length of
  /// its search: `corridorOfEdge` closes `bodies_` as hard obstacles, and
  /// the pads that will not be there once the couplers move would otherwise
  /// make every candidate edge route around them — the symptom being a cost
  /// that never beats the current run.
  void setPads(const std::vector<std::uint32_t>& couplers, const bool standing) {
    for (const auto index : couplers) {
      const auto& coupler = couplers_[index];
      for (const auto cell : coupler.options[coupler.chosen].body) {
        bodies_.set(cell, standing);
      }
    }
  }

  /// Every drawn edge's cells but a segment's own, for the option guard of
  /// `openOptionsOf`: an option whose pad lies on a feedline that stands is
  /// closed. `feedlineCellsBut` reads the insertion's ways; here the edges
  /// have wires, and the pass may have moved the terminal ones.
  [[nodiscard]] grid::BitGrid
  standingCellsBut(const std::vector<Wire>& wires, const std::uint32_t chain,
                   const std::size_t lo, const std::size_t hi) const {
    grid::BitGrid cells(scene_.router.width, scene_.router.height);
    if (!guardTheBodies()) {
      return cells;
    }
    for (const auto& edge : edges_) {
      if (edge.chain == chain && edge.from >= lo && edge.from < hi) {
        continue;
      }
      const auto& way = edgeWayOf(wires, edge.chain, edge.from);
      for (const auto& point : way) {
        if (point.x < scene_.router.width && point.y < scene_.router.height) {
          cells.set(scene_.router.index(point.x, point.y), true);
        }
      }
    }
    return cells;
  }

  /// What one segment's re-search came to.
  struct Research {
    bool accepted = false;
    /// Legality tests run — the budget's unit.
    std::uint32_t trials = 0;
    /// Runs refused after a test, and runs refused without one because they
    /// were the current run with the same ways.
    std::uint32_t rejected = 0;
    std::uint32_t skippedCurrent = 0;
    /// Edge searches of the prefix search.
    std::uint32_t routed = 0;
    std::uint32_t runs = 0;
    bool outOfTime = false;
    bool outOfTrials = false;
    bool exhausted = false;
  };

  /// **The re-search of one segment**: the prefix search on the sub-chain
  /// `lo..hi`, both ends fixed on the option they stand on, the interior
  /// couplers open, and every complete run of options it reaches tested by
  /// the local rip-up-and-reroute of `tryRun` before it is taken. Built as
  /// `solveChainAStar` is, with what differs said here:
  ///
  /// - **No memo and no reorder.** `edgeMemoKey` knows nothing of the fence
  ///   and the phase-3 entries are stale in phase 4, so every step is a
  ///   fresh search; and the end-pair reorder would fire at the segment
  ///   ends and break the cost order the enumeration rests on — a `redone`
  ///   price makes a prefix cheaper than its parent, and then the runs no
  ///   longer pop cheapest first.
  /// - **The fence.** `prefixFence_` holds the segment's own laid ways, every
  ///   one of them; `fenceCommittedEdges` holds the rest of the chip, with
  ///   the segment's own edges left open (`openChain_`) and, under
  ///   `researchSeesChains`, every other chain's edge and this chain's
  ///   terminal and later edges fenced whatever the insertion's switches
  ///   say. `soloChain_` and `looseChain_` stay at NO_OWNER.
  /// - **What the step sees.** `edgeCost` stands the two ends on the options
  ///   asked about, and `sourceOf`/`targetOf`, `runsOfEdge`, `portOfOption`
  ///   and the chain's own leads and pads in `corridorOfEdge` read `chosen`,
  ///   so the candidate's lead and pad are fenced. Three things are the
  ///   **applied** state: the interior couplers' committed pads in `bodies_`
  ///   are hidden for the search (`setPads`); the resonator tail stamped
  ///   along `wayOfResonator` is the applied, cut-back way, harmless at
  ///   copper width; and an interior coupler that is not this step's end
  ///   stands on its committed option — the approximation the insertion
  ///   lives with too.
  /// - **The current run** is enumerated like any other and refused without
  ///   a test when its edges come out on the ways they already hold; with
  ///   other ways it is a candidate like any other — the same couplers with
  ///   the edges laid against the frozen chip, which is what a terminal pair
  ///   that crosses its neighbour chain needs.
  ///
  /// `badNow` is the chip's figure before the segment and is moved to the
  /// accepted candidate's. Leaves the accepted run applied, or the chip as
  /// it was.
  [[nodiscard]] Research
  researchSegment(std::vector<Wire>& wires,
                  const std::vector<std::uint32_t>& members,
                  const std::vector<std::uint32_t>& every, const Pass& pass,
                  const Segment& segment, const std::uint32_t trialsLeft,
                  std::uint32_t& badNow) {
    Research out;
    const auto chain = segment.chain;
    auto& points = chains_[chain];
    const auto lo = segment.lo;
    const auto hi = segment.hi;
    const auto layers = hi - lo + 1;
    const auto edges = hi - lo;
    const auto name = segmentName(wires, segment);

    // The run the segment stands on, by option index per layer.
    std::vector<std::uint32_t> current(layers, 0);
    std::vector<std::uint32_t> interior;
    for (std::size_t k = 0; k < layers; ++k) {
      const auto& point = points[lo + k];
      if (point.fixed) {
        continue;
      }
      current[k] = static_cast<std::uint32_t>(couplers_[point.coupler].chosen);
      if (k > 0 && k + 1 < layers) {
        interior.push_back(point.coupler);
      }
    }
    if (researchJogs()) {
      for (const auto index : interior) {
        couplers_[index].jogsUnlocked = true;
      }
    }

    // The options each layer offers: the ends their current option, the
    // interior everything the guard leaves open.
    const auto standing = standingCellsBut(wires, chain, lo, hi);
    std::vector<std::vector<std::uint32_t>> open(layers);
    for (std::size_t k = 0; k < layers; ++k) {
      if (k == 0 || k + 1 == layers) {
        open[k] = {current[k]};
      } else {
        open[k] = openOptionsOf(chain, lo + k, &standing);
      }
    }

    // The bounds, as the insertion builds them, indices shifted by `lo`.
    std::vector<std::vector<std::uint64_t>> turns(edges);
    std::uint64_t pairs = 0;
    for (std::size_t e = 0; e < edges; ++e) {
      const auto n = open[e].size();
      const auto m = open[e + 1].size();
      turns[e].resize(n * m);
      for (std::size_t c = 0; c < n; ++c) {
        const auto leaving = portOfOption(chain, lo + e, open[e][c], true);
        for (std::size_t d = 0; d < m; ++d) {
          const auto arriving =
              portOfOption(chain, lo + e + 1, open[e + 1][d], false);
          turns[e][(c * m) + d] = 10000ULL * boundTurns(leaving, arriving);
        }
      }
      pairs += n * m;
    }

    routing::ChainProblem problem;
    for (const auto& choices : open) {
      problem.width.push_back(static_cast<std::uint32_t>(choices.size()));
    }
    problem.bound = [&](const std::size_t e, const std::uint32_t i,
                        const std::uint32_t j) {
      return turns[e][(static_cast<std::size_t>(i) * open[e + 1].size()) + j];
    };
    problem.budget = researchBudget();
    // One more than the tests left: the current run may be refused once
    // without a test, and that refusal counts in the search's own tally.
    problem.maxAccepts = trialsLeft + 1;

    std::map<std::vector<std::uint32_t>, Laid> laid;
    const auto runText = [&](const std::span<const std::uint32_t> run) {
      std::string options;
      for (std::size_t k = 0; k < run.size(); ++k) {
        options += std::format("{}{}", k == 0 ? "" : " ",
                               points[lo + k].fixed
                                   ? std::string("launcher")
                                   : std::to_string(open[k][run[k]]));
      }
      return options;
    };
    problem.found = [&](const std::span<const std::uint32_t> run,
                        const std::uint64_t cost) {
      ++out.runs;
      tell(std::format("targeted repair:   {}: every edge drawn on options {} "
                       "-> cost {}",
                       name, runText(run), cost));
    };
    problem.step = [&](const std::span<const std::uint32_t> prefix,
                       const std::uint32_t j, std::int64_t& redone) {
      redone = 0;
      const auto at = prefix.size() - 1;
      std::vector<std::uint32_t> key(prefix.begin(), prefix.end());
      // Every way the prefix has laid is fenced, no exemption: the terminal
      // exemptions of the insertion's fence are chain-relative, and a
      // segment is not a chain.
      std::vector<const Path*> fence;
      fence.reserve(at);
      for (std::size_t e = 0; e < at; ++e) {
        const auto found = laid.find(std::vector<std::uint32_t>(
            key.begin(), key.begin() + static_cast<std::ptrdiff_t>(e + 2)));
        if (found != laid.end() && !found->second.way.empty()) {
          fence.push_back(&found->second.way);
        }
      }
      Path way;
      const auto real =
          edgeCost(wires, chain, lo + at, open[at][prefix[at]], open[at + 1][j],
                   0, fence.empty() ? nullptr : &fence, &way, false);
      if (real == routing::TRELLIS_UNREACHABLE) {
        return real;
      }
      key.push_back(j);
      laid[std::move(key)] = Laid{.way = std::move(way), .before = {}};
      return real;
    };

    // The state the segment starts from, with the interior pads hidden: a
    // refused candidate is put back to exactly the search's own ground.
    setPads(interior, false);
    const auto taken = snapshot(wires);
    fenceEverything_ = researchSeesChains();
    openChain_ = chain;
    openLo_ = lo;
    openHi_ = hi;
    std::uint32_t trials = 0;

    problem.accept = [&](const std::span<const std::uint32_t> run,
                         const std::uint64_t cost) {
      const auto ways = waysOfRun(laid, run);
      // The current run on the ways it already holds: nothing to test.
      bool same = true;
      for (std::size_t k = 0; same && k < layers; ++k) {
        same = open[k][run[k]] == current[k];
      }
      for (std::size_t e = 0; same && e < edges; ++e) {
        const auto& held = wires[edgeWireOf_[chain][lo + e]].way;
        same = held.size() == ways[e].size() &&
               std::ranges::equal(held, ways[e],
                                  [](const PathPoint& a, const PathPoint& b) {
                                    return a.samePlace(b);
                                  });
      }
      if (same) {
        ++out.skippedCurrent;
        tell(std::format("targeted repair:   {} run {} (cost {}): the current "
                         "run on the ways it holds, not tested",
                         name, runText(run), cost));
        return false;
      }
      if (trials >= trialsLeft) {
        out.outOfTrials = true;
        return false;
      }
      ++trials;
      const bool kept = tryRun(wires, members, every, pass, segment, open, run,
                               ways, cost, taken, badNow);
      if (!kept) {
        ++out.rejected;
        setPads(interior, false);
      }
      return kept;
    };

    const auto answer = routing::solveChainAStar(problem);

    openChain_ = NO_OWNER;
    openLo_ = 0;
    openHi_ = 0;
    fenceEverything_ = false;
    prefixFence_ = nullptr;
    // The pads back — the accepted run's, or the ones the chip stood on.
    setPads(interior, true);
    foreignRoomStale_ = true;

    out.trials = trials;
    out.routed = answer.routed;
    out.accepted = answer.solved;
    out.outOfTime = answer.outOfTime;
    out.outOfTrials = out.outOfTrials || answer.outOfTrials;
    out.exhausted = !answer.solved && answer.optimal;
    say(std::format(
        "targeted repair: {} — {} after {} run{} reached, {} tested, {} "
        "refused, {} the current run; {} edge searches over {} option "
        "pair{}{}",
        name,
        answer.solved      ? std::format("ACCEPTED options {} at cost {}",
                                         runText(answer.chosen), answer.cost)
        : out.outOfTrials  ? "nothing accepted, out of legality tests"
        : answer.outOfTime ? "nothing accepted, out of time"
        : answer.optimal   ? "nothing accepted, every run refused"
                           : "nothing accepted, no run of options joins it",
        out.runs, out.runs == 1 ? "" : "s", trials, out.rejected,
        out.skippedCurrent, answer.routed, pairs, pairs == 1 ? "" : "s",
        answer.expansions == 0 ? std::string{}
                               : std::format(", {} prefixes expanded",
                                             answer.expansions)));
    return out;
  }

  /// **The legality test** of one candidate run of options for a segment: a
  /// local rip-up-and-reroute, judged on the figure.
  ///
  /// 1. The interior couplers take the run's options (`applyOption`, as
  ///    `turnCoupler` does, but for every coupler before any edge is laid);
  ///    the segment's edges take the ways the search found, objectives from
  ///    `sourceOf`/`targetOf`, stubs **recomputed** from `runsOfEdge` — they
  ///    are set once at creation and `turnCoupler` never recomputed them, a
  ///    latent bug this does not inherit; the crossing rule is rebuilt.
  /// 2. The window: the plain wires bridged to a segment edge
  ///    (`bridgers_`), the resonators of its waypoints, a terminal edge among
  ///    its edges, every wire the fails named, every member whose way lies
  ///    within the clearance of a new edge way or crosses one — which
  ///    `isLegal` cannot see, because `conflictsIn` exempts plain wire
  ///    against feedline both ways — and `researchK` ring members beyond
  ///    each of those. Flagged `routed = false, ripped = true`; every other
  ///    member is held `routed = true`, the fails outside the window
  ///    included, so the window is what the pass redraws and nothing else.
  /// 3. One silenced `sweep` over the **whole ring** under
  ///    `Pass::onlyUnsettled`, `researchRounds` rounds, so the fences and the
  ///    lane see the true ring neighbours. Terminal edges in the window are
  ///    drawn by the sweep's `search`, as the pass draws them.
  /// 4. The verdict from `failsOf` over every wire, not from the round
  ///    tallies: the window is clean when no wire the pass touched — the
  ///    window and whatever the relaxation released and redrew — is
  ///    unrouted, open or crossing; the run is kept when the window is clean
  ///    and `bad()` over every wire is strictly below what it was before the
  ///    segment. Lengths are no criterion (user, 2026-10-04).
  /// 5. Refused: `restore` to the segment's start — the whole wire list,
  ///    `bodies_`, `chosen`, the anchors, the field rebuilt, the crossing
  ///    rule; not `chainEdgePaths_`, `edgeMemo_`, `jogsUnlocked`. Kept: the
  ///    edges' ways go into `chainEdgePaths_` and `badNow` moves.
  [[nodiscard]] bool tryRun(std::vector<Wire>& wires,
                            const std::vector<std::uint32_t>& members,
                            const std::vector<std::uint32_t>& every,
                            const Pass& pass, const Segment& segment,
                            const std::vector<std::vector<std::uint32_t>>& open,
                            const std::span<const std::uint32_t> run,
                            const std::vector<Path>& ways,
                            const std::uint64_t cost, const Snapshot& taken,
                            std::uint32_t& badNow) {
    const auto chain = segment.chain;
    const auto& points = chains_[chain];
    const auto lo = segment.lo;
    const auto hi = segment.hi;
    const auto edges = hi - lo;
    std::string options;
    for (std::size_t k = 0; k < run.size(); ++k) {
      options += std::format("{}{}", k == 0 ? "" : " ",
                             points[lo + k].fixed
                                 ? std::string("launcher")
                                 : std::to_string(open[k][run[k]]));
    }
    const auto label = std::format("targeted repair:   {} run {} (cost {})",
                                   segmentName(wires, segment), options, cost);

    // 1. Apply.
    for (std::size_t k = 1; k + 1 < run.size(); ++k) {
      if (!points[lo + k].fixed) {
        applyOption(wires, points[lo + k].coupler, open[k][run[k]]);
      }
    }
    for (std::size_t e = 0; e < edges; ++e) {
      auto& wire = wires[edgeWireOf_[chain][lo + e]];
      const auto& edge = edges_[wire.edge];
      lift(wire);
      wire.way = ways[e];
      wire.objective.source = sourceOf(points[edge.from]);
      wire.objective.target = targetOf(points[edge.to]);
      wire.fixed.clear();
      const auto [startRun, endRun] = runsOfEdge(edge);
      wire.startStub = startRun;
      wire.endStub = endRun;
      wire.drawn = !wire.way.empty();
      wire.routed = wire.drawn;
      wire.tooShort = false;
      wire.tooLong = false;
      place(wire);
    }
    rebuildCrossingRule(wires);

    // 2. The window.
    std::set<std::uint32_t> window;
    for (std::size_t e = 0; e < edges; ++e) {
      if (lo + e < bridgers_[chain].size()) {
        for (const auto key : bridgers_[chain][lo + e]) {
          window.insert(key);
        }
      }
      const auto key = edgeWireOf_[chain][lo + e];
      if (edges_[wires[key].edge].terminal) {
        window.insert(key);
      }
    }
    for (std::size_t k = 0; k < run.size(); ++k) {
      if (!points[lo + k].fixed) {
        window.insert(couplers_[points[lo + k].coupler].wire);
      }
    }
    for (const auto key : segment.fails) {
      window.insert(key);
    }
    {
      const std::unordered_set<std::uint32_t> isMember(members.begin(),
                                                       members.end());
      const auto& stencil = stencilFor(tuning_.clearance);
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      for (std::size_t e = 0; e < edges; ++e) {
        const auto& way = wires[edgeWireOf_[chain][lo + e]].way;
        alongDisc(way, stencil, [&](const std::int64_t x, const std::int64_t y) {
          const auto owner =
              field_.owner(static_cast<std::size_t>((y * width) + x));
          if (owner != NO_OWNER && owner < wires.size() &&
              isMember.contains(owner)) {
            window.insert(owner);
          }
        });
        for (const auto member : members) {
          if (!window.contains(member) && !wires[member].way.empty() &&
              waysCross(way, wires[member].way)) {
            window.insert(member);
          }
        }
      }
    }
    {
      const auto k = researchK();
      const auto total = members.size();
      std::vector<std::uint32_t> more;
      for (std::size_t at = 0; at < total; ++at) {
        if (!window.contains(members[at])) {
          continue;
        }
        for (std::size_t step = 1; step <= k && step < total; ++step) {
          more.push_back(members[(at + step) % total]);
          more.push_back(members[(at + total - step) % total]);
        }
      }
      window.insert(more.begin(), more.end());
    }
    std::vector<std::uint32_t> flagged;
    std::vector<Path> had;
    had.reserve(members.size());
    for (const auto member : members) {
      auto& wire = wires[member];
      had.push_back(wire.way);
      if (window.contains(member)) {
        wire.routed = false;
        wire.ripped = true;
        flagged.push_back(member);
      } else {
        wire.routed = true;
      }
    }

    // 3. The local pass, silenced unless asked to speak.
    Pass local = pass;
    local.rounds = researchRounds();
    local.name = "targeted repair";
    local.onlyUnsettled = true;
    const auto verbosity = verbosity_;
    const auto quiet = progress_;
    if (!researchVerbose()) {
      verbosity_ = 0;
      progress_ = {};
    } else {
      say(label + ": the local pass");
    }
    sweep(wires, members, local);
    const auto after = failsOf(wires, every);
    progress_ = quiet;
    verbosity_ = verbosity;
    // Which window wires found no way in the last round of the local pass
    // and kept what they had: a wire that fails the window because it was
    // never redrawn reads differently from one redrawn into a fail.
    std::string noWay;
    if (!rounds_.empty()) {
      for (const auto& id : rounds_.back().failed) {
        noWay += (noWay.empty() ? "" : ", ") + id;
      }
    }

    // 4. The verdict.
    std::set<std::uint32_t> touched(flagged.begin(), flagged.end());
    std::uint32_t moved = 0;
    for (std::size_t at = 0; at < members.size(); ++at) {
      const auto& now = wires[members[at]].way;
      const bool same =
          now.size() == had[at].size() &&
          std::ranges::equal(now, had[at],
                             [](const PathPoint& a, const PathPoint& b) {
                               return a.samePlace(b);
                             });
      if (!same) {
        ++moved;
        touched.insert(members[at]);
      }
    }
    std::uint32_t unrouted = 0;
    std::uint32_t openInWindow = 0;
    std::uint32_t crossing = 0;
    std::string named;
    const auto count = [&](const std::vector<std::uint32_t>& keys,
                           std::uint32_t& figure, const char* what) {
      for (const auto key : keys) {
        if (touched.contains(key)) {
          ++figure;
          named += std::format("{}{} {}", named.empty() ? "" : ", ",
                               wireId(wires[key]), what);
        }
      }
    };
    count(after.unroutedKeys, unrouted, "unrouted");
    count(after.openKeys, openInWindow, "open");
    count(after.crossingKeys, crossing, "crossing");
    const bool clean = unrouted + openInWindow + crossing == 0;
    const bool better = after.bad() < badNow;
    tell(std::format("{}: window {} wire{}, {} moved{}; in it {} unrouted, {} "
                     "open, {} crossing{}; bad {} -> {} ({} unrouted, {} open, "
                     "{} crossing): {}",
                     label, flagged.size(), flagged.size() == 1 ? "" : "s",
                     moved,
                     noWay.empty() ? std::string{}
                                   : ", no way for " + noWay + " in the last round",
                     unrouted, openInWindow, crossing,
                     named.empty() ? std::string{} : " (" + named + ")", badNow,
                     after.bad(), after.unrouted, after.open, after.crossing,
                     clean && better    ? "ACCEPTED"
                     : clean            ? "rejected, the chip is not better"
                     : better           ? "rejected, the window is not clean"
                                        : "rejected"));
    if (clean && better) {
      badNow = after.bad();
      for (std::size_t e = 0; e < edges; ++e) {
        chainEdgePaths_[chain][lo + e] = wires[edgeWireOf_[chain][lo + e]].way;
      }
      return true;
    }

    // 5. Refused.
    restore(wires, taken);
    foreignRoomStale_ = true;
    return false;
  }

  /// **The targeted repair over one chip**, in place of the blind `repair`:
  /// the triage, then every segment re-searched in order under the budget
  /// of `repair_trials` legality tests. After an accept the fails are read
  /// again and the triage rebuilt on the new state — a segment whose fails
  /// an earlier accept cleared is then not there to be searched, and one
  /// whose fails remain is searched against the chip as it now stands. The
  /// figure can only fall with an accept, so the loop ends. Then, under
  /// `researchFinalSweep`, one full pass over the ring, kept if the figure
  /// does not rise. One `==>` line says what came of it.
  void researchSegments(std::vector<Wire>& wires,
                        const std::vector<std::uint32_t>& members,
                        const std::vector<std::uint32_t>& every,
                        const Pass& pass) {
    const auto began = std::chrono::steady_clock::now();
    auto fails = failsOf(wires, every);
    const auto before = fails;
    auto segments = blameFails(wires, members, fails);
    const auto built = segments.size();
    auto badNow = fails.bad();
    std::uint32_t trials = 0;
    std::uint32_t accepted = 0;
    std::uint32_t searched = 0;
    std::uint32_t rejected = 0;
    std::uint32_t skipped = 0;
    std::uint32_t routed = 0;
    std::uint32_t retriaged = 0;
    // The segments searched without an accept, by chain, span and the fails
    // they carried. After an accept the triage is rebuilt, and a segment
    // that comes back unchanged would be searched again against a chip that
    // changed somewhere else — on 33q that spent a second 20 s on chain 0
    // for the same three refusals. Skipped, and said; a segment whose fails
    // changed is searched again.
    std::set<std::tuple<std::uint32_t, std::size_t, std::size_t,
                        std::vector<std::uint32_t>>>
        fruitless;
    std::size_t at = 0;
    while (at < segments.size() && badNow > 0) {
      if (trials >= tuning_.repairTrials) {
        say(std::format("targeted repair: the budget of {} legality test{} is "
                        "spent, {} segment{} left unsearched",
                        tuning_.repairTrials,
                        tuning_.repairTrials == 1 ? "" : "s",
                        segments.size() - at,
                        segments.size() - at == 1 ? "" : "s"));
        break;
      }
      const Segment segment = segments[at];
      const auto key = std::tuple{segment.chain, segment.lo, segment.hi,
                                  segment.fails};
      if (fruitless.contains(key)) {
        tell(std::format("targeted repair: {} searched before without an "
                         "accept and unchanged since, skipped",
                         segmentName(wires, segment)));
        ++at;
        continue;
      }
      ++searched;
      const auto result =
          researchSegment(wires, members, every, pass, segment,
                          tuning_.repairTrials - trials, badNow);
      trials += result.trials;
      rejected += result.rejected;
      skipped += result.skippedCurrent;
      routed += result.routed;
      if (!result.accepted) {
        fruitless.insert(key);
        ++at;
        continue;
      }
      ++accepted;
      ++retriaged;
      // The triage again on the new state, its fail lines silenced; the
      // summary says what remains.
      const auto verbosity = verbosity_;
      verbosity_ = 0;
      fails = failsOf(wires, every);
      segments = blameFails(wires, members, fails);
      verbosity_ = verbosity;
      at = 0;
    }
    if (researchFinalSweep()) {
      const auto taken = snapshot(wires);
      const auto verbosity = verbosity_;
      verbosity_ = 0;
      const auto quiet = progress_;
      progress_ = {};
      Pass full = pass;
      full.name = "targeted repair final sweep";
      sweep(wires, members, full);
      const auto swept = failsOf(wires, every);
      progress_ = quiet;
      verbosity_ = verbosity;
      if (swept.bad() <= badNow) {
        say(std::format("targeted repair: the final sweep is kept, bad {} -> {}",
                        badNow, swept.bad()));
        badNow = swept.bad();
      } else {
        say(std::format("targeted repair: the final sweep is dropped, bad {} "
                        "-> {}",
                        badNow, swept.bad()));
        restore(wires, taken);
        foreignRoomStale_ = true;
      }
    }
    // Everything down again, so that the field says what the canvas holds.
    for (auto& wire : wires) {
      place(wire);
    }
    const auto verbosity = verbosity_;
    verbosity_ = 0;
    const auto after = failsOf(wires, every);
    verbosity_ = verbosity;
    // The fourth guarantee as the stage leaves it: the pass redraws the
    // terminal edges and the repair moves couplers, so the insertion's line
    // is not the last word on it. Only here, under the switch, so that
    // `SCPD_REPAIR_SEARCH=0` reproduces the stage's lines exactly.
    static_cast<void>(checkFeedlineCrossings(wires, "feedline routing"));
    // And the first guarantee on what the sweep left, so that a pair the
    // refinement's own line reports can be told from one the sweep made:
    // the insertion's line is about the edges as the insertion drew them,
    // and every pass after it draws them again. See `refineChecks`.
    if (refineChecks()) {
      static_cast<void>(checkFeedlineRoom(wires, "feedline routing"));
    }
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    say(std::format(
        "==> targeted repair: {} segment{} built, {} searched, {} accepted ({} "
        "re-triage{}); {} legality test{} of {} ({} refused, {} the current "
        "run), {} edge searches; unrouted {} -> {}, open {} -> {}, crossing {} "
        "-> {} (bad {} -> {}), {:.1f}s",
        built, built == 1 ? "" : "s", searched, accepted, retriaged,
        retriaged == 1 ? "" : "s", trials, trials == 1 ? "" : "s",
        tuning_.repairTrials, rejected, skipped, routed, before.unrouted,
        after.unrouted, before.open, after.open, before.crossing,
        after.crossing, before.bad(), after.bad(), seconds));
  }

private:
  /// How many rounds without progress end a pass.
  static constexpr std::uint32_t STALE_ROUNDS = 4;

  /// How many ways the pass running now refused because they did not cross
  /// the wire's bridge (`bridgeCheck`). Reset per sweep, said at its end.
  std::uint32_t bridgeRefusals_ = 0;

  /// Whether the length-point bands are stamped at all. Cleared for the
  /// recovery pass at the end of the outer routing (`recoverWithoutBands`)
  /// and set again after it.
  bool lengthPointActive_ = true;

  /// Charge the places no wire may be moved off, before anything is drawn.
  ///
  /// A wire has more of them than its two ends. The straight run it leaves its
  /// source on is fixed by the orientation of the port that feeds it: the
  /// search never sees those cells, because the router adds the run to the way
  /// after it has searched. So a wire drawn across another wire's run is a
  /// violation the search could not have refused and no later round can undo.
  /// They are charged here, before the first wire is drawn, and only the wire
  /// being searched for is let out of its own.
  void fixPlaces(std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members, const Pass& pass) {
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.feasible || !wire.fixed.empty()) {
        continue;
      }
      const auto stub = startStubOf(wire, pass.straightStart);
      wire.fixed = wire.arc;
      const auto out = router_.straightStub(wire.objective.source, false, stub);
      wire.fixed.insert(wire.fixed.end(), out.begin(), out.end());
      if (wire.endStub > 0) {
        const auto in =
            router_.straightStub(wire.objective.target, true, wire.endStub);
        wire.fixed.insert(wire.fixed.end(), in.begin(), in.end());
      } else {
        wire.fixed.push_back(wire.objective.target);
      }
      if (!wire.placed) {
        field_.chargeFixed(wire.fixed);
        wire.endsOnly = true;
      }
    }
  }

  /// Put every wire of a pass down on the way the Detail stage drew, before
  /// anything is searched.
  ///
  /// What a wire starts on is the Detail stage's cells joined on this grid,
  /// with the straight run out of its source spliced in front: that run is
  /// part of every way the search will ever return, and the room around it
  /// has to stand from the first search on or a neighbour settles across it.
  /// A wire that has a way of its own from an earlier pass, or that is down
  /// already, is left alone; a wire the Detail stage did not draw keeps only
  /// its fixed places charged, as `fixPlaces` left them.
  void seed(std::vector<Wire>& wires, const std::vector<std::uint32_t>& members,
            const Pass& pass) {
    std::uint32_t seeded = 0;
    std::uint32_t feasible = 0;
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.feasible) {
        continue;
      }
      ++feasible;
      if (wire.drawn || wire.placed || wire.way.empty()) {
        continue;
      }
      wire.way = seededWay(wire);
      place(wire);
      ++seeded;
    }
    say(std::format("{}: {} of {} wires start on the way the Detail stage "
                    "drew",
                    pass.name, seeded, feasible));
  }

  /// The Detail stage's cells as a way on this grid.
  ///
  /// The cells come from a grid three to four times coarser, so consecutive
  /// ones lie apart; each pair is joined by the eight-connected steps of the
  /// line between them, which is what makes the result a way the field can
  /// hold like any other. In front of it runs the straight stub out of the
  /// source, which `fixed` holds; the seed cells the stub already covers are
  /// skipped, and the way ends on the target cell whatever the last seed cell
  /// was. The headings are those of the steps.
  [[nodiscard]] static Path seededWay(const Wire& wire) {
    Path way;
    if (wire.fixed.size() >= 2) {
      way.assign(wire.fixed.begin(), wire.fixed.end() - 1);
    } else {
      way.push_back(wire.objective.source);
    }
    const auto& source = wire.objective.source;
    const auto covered = static_cast<double>(way.size() - 1);
    auto first = wire.way.begin();
    while (first != wire.way.end() &&
           std::hypot(static_cast<double>(first->x) - source.x,
                      static_cast<double>(first->y) - source.y) <= covered) {
      ++first;
    }
    for (auto point = first; point != wire.way.end(); ++point) {
      connect(way, *point);
    }
    connect(way, wire.objective.target);
    for (std::size_t at = 0; at + 1 < way.size(); ++at) {
      way[at].heading = headingOfStep(way[at], way[at + 1]);
      way[at].primitive = 0;
    }
    way.back().heading = wire.objective.target.heading;
    way.back().primitive = 0;
    return way;
  }

  /// One wire's turn: the prototype's two phases, and nothing else.
  ///
  /// Phase 1 offers the wire a way inside the band around the way it has,
  /// with its two ring neighbours standing in the way as obstacles inflated
  /// by the clearance. Phase 2 is the relaxation, along the sweep as the
  /// prototype's: each level lets go of one more of the wires ahead — a wire
  /// let go of is no obstacle at all, because it is drawn again afterwards —
  /// and fences the search with the last wire let go of and the neighbour on
  /// the other side. The lane between the two ring neighbours is free and
  /// everything outside it priced, and the wires let go of are priced at
  /// growing distances, so the way is pushed away from where they run rather
  /// than drawn over it. A way found is taken as it is; when none is found,
  /// the wires let go of go back to what they were.
  ///
  /// A resonator's way found is not a way taken until it is long enough.
  /// The meander goes in after each search, the first placement that fits in
  /// phase 1 and the cheapest in the relaxation, and a way without room for
  /// it counts as no way, so the relaxation goes on; when nothing comes of
  /// it the wire keeps that way, short as it is, as the prototype keeps it.
  ///
  /// The rule against every other wire is not what the search keeps. It is
  /// what the fails are counted by when the round is over.
  bool attempt(std::vector<Wire>& wires,
               const std::vector<std::uint32_t>& members,
               const std::uint32_t slot, const bool forward, const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    auto& wire = wires[members[slot]];
    const auto& before = wires[members[(slot + total - 1) % total]];
    const auto& after = wires[members[(slot + 1) % total]];

    // Under the feedline constraints a wire starts on the way it has, and a
    // way that holds every rule already is settled: the rule against every
    // other wire, the crossing rule, and a resonator's length. Only a wire
    // whose way does not hold is searched for — unless `forceReroute` is on,
    // which is what draws the kink out of a spliced way. See it for why a
    // kinked way passes `isLegal`.
    // No lane until one is priced: the frame outlives a wire, and a picture
    // of the next one must not show the last one's polygon.
    frame_.laneBefore.clear();
    frame_.laneAfter.clear();
    frame_.laneCurrent.clear();
    const bool ripped = wire.ripped;
    wire.ripped = false;
    if (pass.feedlines && !forceReroute() && wire.drawn && !ripped &&
        isLegal(wire, wires)) {
      wire.routed = true;
      wire.tooShort = false;
      wire.tooLong = false;
      place(wire);
      tell(std::format("wire {} · round {} {}: keeps its way, which holds",
                       wireId(wire), frame_.round,
                       forward ? "forward" : "backward"));
      return true;
    }

    // The wire is lifted off the canvas for the length of its own search, so
    // that what is put down afterwards is the one way it has.
    lift(wire);

    // Phase 1: the band around the way it has, fenced by its two neighbours,
    // and nothing priced — except, under the feedline constraints, the
    // chains: fenced, and priced around.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    // The prototype's feedline pass fences the second pair as well
    // (`additional_paths_1`, `FinalGrid.cpp:4262`), where its outer routing
    // fences only the first. A ring whose order the chain edges have cut is
    // no longer planar, and one neighbour either side no longer says where
    // a wire may run.
    if (pass.feedlines && feedlineLikePrototype() && total > 4) {
      // And the pairs beyond it as `feedlineFencePairs` says, 2 by default.
      for (std::uint32_t k = 2; k <= feedlineFencePairs() && 2 * k < total;
           ++k) {
        fence(wire, {&wires[members[(slot + total - k) % total]],
                     &wires[members[(slot + k) % total]]});
      }
    }
    // The length-point clearance of the outer routing: see `lengthPointK`.
    closeLengthBands(wire, {&before, &after}, pass);
    std::ranges::fill(proximity_, 0);
    constrainByFeedlines(wire, wires, pass);
    frame_.wires = &wires;
    frame_.before = before.key;
    frame_.after = after.key;
    frame_.kind = "normal";
    frame_.fence = {before.key, after.key};
    frame_.ripped.clear();
    const bool lengthened = lengthensIn(wire, pass);
    auto found = search(wire, pass.straightStart, true, !lengthened);
    const auto id = wireId(wire);
    const auto where = std::format("round {} {}", frame_.round,
                                   forward ? "forward" : "backward");
    RoundRecord* record = rounds_.empty() ? nullptr : &rounds_.back();
    // A wire that has to cross a chain edge has to cross **its** edge; a way
    // that comes home on the wrong side of the feedline is refused as no
    // way, so that the wire fails here rather than standing in the way of
    // every wire drawn after it. See `bridgeCheck`.
    const auto crossesItsBridge = [&](const Path& way) {
      if (!pass.feedlines || !bridgeCheck() || wire.bridged == NO_OWNER ||
          wire.bridged >= wires.size()) {
        return true;
      }
      const auto& bridge = wires[wire.bridged];
      return !bridge.drawn || bridge.way.empty() || waysCross(way, bridge.way);
    };
    const auto refuseTheDetour = [&](const std::string& phase) {
      if (found.empty() || crossesItsBridge(found)) {
        return;
      }
      tell(std::format("wire {} · {} · {}: found {} cells that do not cross "
                       "its bridge {} — refused as no way",
                       id, where, phase, found.size(),
                       wireId(wires[wire.bridged])));
      ++bridgeRefusals_;
      found.clear();
    };
    refuseTheDetour("normal");
    // The last way found that could not be made long enough, and what the
    // lengthening said. A way found is kept only when it is long enough.
    Path plain;
    bool plainTooLong = false;
    std::string note;
    const auto lengthenFound = [&](const bool priced) {
      note.clear();
      if (!lengthened) {
        return;
      }
      std::string said;
      if (!found.empty()) {
        const auto made =
            lengthen(wire, found, priced,
                     pass.refinement ? 1.0 : sweepLengthBand());
        said = made.note;
        note = " · " + said;
        if (!made.reached) {
          plain = std::move(found);
          plainTooLong = made.tooLong;
          found.clear();
        }
      }
      // The picture of the search, taken here rather than in `search` so
      // that it shows the way with its meander, or the way that had no room
      // for one.
      drawSearch(wire, found.empty() && !said.empty() ? plain : found, said);
    };
    // What a search came to, in words: the way found and what the
    // lengthening made of it, or a way found and left short, or nothing.
    const auto outcome = [&]() {
      if (!found.empty()) {
        return std::format("found {} cells{}", found.size(), note);
      }
      if (!note.empty()) {
        return std::format("found {} cells{}", plain.size(), note);
      }
      return std::string("no way");
    };
    lengthenFound(false);
    if (!found.empty()) {
      tell(std::format("wire {} · {} · normal: {}{}", id, where, outcome(),
                       picture()));
      if (record != nullptr) {
        ++record->normal;
      }
    } else {
      // Asked before `whatBlocks`, which puts the corridor back without the
      // second pair of neighbours and would misreport an open start.
      const auto arrival = verbosity_ >= 1
                               ? deadOnArrival(wire, wires, pass.straightStart)
                               : std::string{};
      tell(std::format("wire {} · {} · normal: {}, relaxing{}{}{}", id, where,
                       outcome(), picture(),
                       verbosity_ >= 1 && pass.feedlines
                           ? whatBlocks(wire, wires, members, before, after, pass)
                           : std::string{},
                       arrival));
    }

    // Phase 2: the relaxation.
    std::vector<std::pair<std::uint32_t, bool>> released;
    for (std::uint32_t level = 1;
         found.empty() && total > 1 && level <= pass.maxRelaxation; ++level) {
      const auto step = forward ? (slot + level) % total
                                : (slot + total - (level % total)) % total;
      if (step == slot) {
        continue;
      }
      auto& ripped = wires[members[step]];
      released.emplace_back(ripped.key, ripped.routed);
      ripped.routed = false;

      buildCorridor(wire, pass.reach);
      // What the fence actually closed, so that the picture and the log can
      // say it. They used to name `{ripped, before/after}` whatever the
      // fence did, and since the prototype's rule came in that is wrong on
      // both counts: the wire let go of is **not** fenced — it is the one
      // thing meant to stay crossable — and two of the four that are fenced
      // were never named. A reader then sees a closed band drawn over a
      // crossable wire and no band over two closed ones (user, 2026-10-01).
      std::vector<std::uint32_t> closed;
      if (pass.feedlines && feedlineLikePrototype() && total > 4) {
        // The prototype fences the wire *after* the one it let go of, not
        // the one it let go of — that one is meant to be crossable — and the
        // second pair with it (`FinalGrid.cpp:4430`).
        const auto onward = [&](const std::uint32_t at) {
          return forward ? (at + 1) % total : (at + total - 1) % total;
        };
        const auto backward = [&](const std::uint32_t at) {
          return forward ? (at + total - 1) % total : (at + 1) % total;
        };
        const auto firstOn = onward(step);
        const auto secondOn = onward(firstOn);
        // `feedlineFencePairs` pairs: the k-th beyond the released wire and
        // the k-th behind this one, stopping before a pair wraps round.
        auto on = step;
        auto back = slot;
        for (std::uint32_t k = 1; k <= feedlineFencePairs(); ++k) {
          on = onward(on);
          back = backward(back);
          if (on == slot || back == step || on == back) {
            break;
          }
          // Never a wire let go of at this level or one before: those are
          // the ones meant to stay crossable (user, 2026-10-05).
          for (const auto at : {on, back}) {
            const auto& other = wires[members[at]];
            const bool ripped = std::ranges::any_of(
                released, [&](const auto& r) { return r.first == other.key; });
            if (ripped || other.key == wire.key) {
              continue;
            }
            fence(wire, {&other});
            if (std::ranges::find(closed, other.key) == closed.end()) {
              closed.push_back(other.key);
            }
          }
        }
        // Beyond the two, on to the next conventional wire: a resonator
        // does not bound the room the released wire gave up. See
        // `fenceToConventional`.
        if (fenceToConventional()) {
          const auto conventional = [&](const std::uint32_t at) {
            const auto& other = wires[members[at]];
            return !other.resonator && !other.feedline;
          };
          bool bounded = conventional(firstOn) || conventional(secondOn);
          auto at = onward(secondOn);
          for (std::uint32_t walked = 0;
               !bounded && walked + 4 < total && at != slot && at != step;
               ++walked, at = onward(at)) {
            const auto& other = wires[members[at]];
            if (other.feedline) {
              continue;
            }
            fence(wire, {&other});
            if (other.key != wire.key &&
                std::ranges::find(closed, other.key) == closed.end()) {
              closed.push_back(other.key);
            }
            bounded = conventional(at);
          }
        }
      } else {
        fence(wire, {&ripped, forward ? &before : &after});
        closed = {ripped.key, (forward ? before : after).key};
        // Only the neighbour on the far side keeps its length-point band at
        // a relaxation level. The wire let go of at this level is fenced by
        // its way and nothing more: it is drawn again afterwards and its
        // length point moves with it, so its band is let go of with it
        // (user, 2026-10-04) — which is also what the prototype ends up
        // doing, its stamp of that wire's band landing before the corridor
        // buffer is reset (`FinalGrid.cpp:7095-7102`). The wires let go of
        // at the levels before are crossable and get no band either.
        closeLengthBands(wire, {forward ? &before : &after}, pass);
      }
      fenceFixedPlaces(wire, wires, released);
      priceLane(wires, members, slot, forward, level);
      constrainByFeedlines(wire, wires, pass);
      frame_.kind = std::format("relax {}", level);
      frame_.fence = closed;
      frame_.ripped.push_back(ripped.key);
      found = search(wire, pass.straightStart, true, !lengthened);
      refuseTheDetour(std::format("relax {}", level));
      lengthenFound(true);
      std::string fenced;
      for (const auto key : closed) {
        fenced += (fenced.empty() ? "" : ", ") + wireId(wires[key]);
      }
      tell(std::format("wire {} · relax {}: let go of {}, fence {} · {}{}{}", id,
                       level, wireId(ripped),
                       fenced.empty() ? std::string("nothing") : fenced,
                       outcome(), picture(),
                       found.empty() && verbosity_ >= 1
                           ? deadOnArrival(wire, wires, pass.straightStart)
                           : std::string{}));
    }

    if (found.empty()) {
      // The rollback: every wire let go of is exactly what it was.
      std::string names;
      for (const auto& [key, routed] : released) {
        wires[key].routed = routed;
        names += (names.empty() ? "" : ", ") + wireId(wires[key]);
      }
      // A resonator whose way was found but could not be made long enough
      // keeps that way: it is what the wires around it see, and the wire is
      // tried again in the next round.
      if (!plain.empty()) {
        wire.way = std::move(plain);
        wire.drawn = true;
        wire.tooShort = !plainTooLong;
        wire.tooLong = plainTooLong;
      }
      place(wire);
      wire.routed = false;
      tell(std::format("wire {} · {}: no way after {} relaxations; {} back as "
                       "they were{}",
                       id, where, released.size(),
                       names.empty() ? std::string("nothing") : names,
                       wire.tooShort ? "; keeps a way too short for its meander"
                                     : ""));
      if (record != nullptr) {
        record->failed.push_back(id);
        record->tooShort += wire.tooShort ? 1 : 0;
      }
      return false;
    }
    if (record != nullptr && !released.empty()) {
      ++record->relaxed;
    }
    for (const auto& [key, routed] : released) {
      wires[key].ripped = true;
    }
    wire.way = found;
    wire.drawn = true;
    wire.routed = true;
    wire.tooShort = false;
    wire.tooLong = false;
    place(wire);
    return true;
  }

  /// What stands in the way of a wire that found nothing under the feedline
  /// constraints, for the second level of verbosity: the search again with
  /// one constraint left out at a time — the ring neighbours, the feedline
  /// fences, the crossing rule, the coupler bodies — and with all of them
  /// left out. Restores what the search had.
  [[nodiscard]] std::string whatBlocks(const Wire& wire, std::vector<Wire>& wires,
                                       const std::vector<std::uint32_t>& members,
                                       const Wire& before, const Wire& after,
                                       const Pass& pass) {
    const auto attempt = [&](const bool neighbours, const bool feedlines,
                             const bool crossing, const bool bodies) {
      const auto kept = bodies_;
      if (!bodies) {
        bodies_ = grid::BitGrid(scene_.router.width, scene_.router.height);
      }
      buildCorridor(wire, pass.reach);
      if (neighbours) {
        fence(wire, {&before, &after});
      }
      std::ranges::fill(proximity_, 0);
      if (feedlines) {
        for (const auto& edge : edges_) {
          const auto& other = wires[edge.wire];
          if (other.drawn && other.key != wire.key && other.key != wire.bridged) {
            fence(wire, {&other});
          }
        }
      }
      if (!crossing) {
        router_.clearOrthogonalConstraints();
      }
      const bool found = !search(wire, pass.straightStart, true, false).empty();
      if (!crossing) {
        // Only the rule goes back. `beginPass` would do it, but it also
        // rebuilds the field and puts **every** wire down again — the one
        // this attempt has lifted included — and the attempt goes on
        // believing it is still off the canvas. That made the second level
        // of verbosity change the result it was there to explain: the same
        // settings came to 6 open at `-v` and 4 at `-v 1`.
        rebuildCrossingRule(wires);
      }
      bodies_ = kept;
      return found;
    };
    std::string blocked;
    const auto note = [&](const std::string& what, const bool found) {
      if (found) {
        blocked += (blocked.empty() ? "" : ", ") + what;
      }
    };
    note("the neighbours", attempt(false, true, true, true));
    note("the feedlines", attempt(true, false, true, true));
    note("the crossing rule", attempt(true, true, false, true));
    note("the coupler bodies", attempt(true, true, true, false));
    if (blocked.empty()) {
      blocked = attempt(false, false, false, false) ? "several together" : "the band itself";
    }
    // What the search had, put back.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    std::ranges::fill(proximity_, 0);
    constrainByFeedlines(wire, wires, pass);
    return " · in the way: " + blocked;
  }

  /// Whether a search was over before it began, for the log of one that
  /// found nothing: the cell it starts on — the source moved along its
  /// heading by the straight run — and the cells a first step can reach,
  /// which of them are closed, and what lies within the clearance of each.
  /// Empty when the start is open.
  ///
  /// A search that fails this way fails whatever else is loosened, and it is
  /// what 45q's wire 4 did in every round: its start stood 17 cells from wire
  /// 3, inside 3's clearance, with its own seed way open end to end and a
  /// strip twenty cells wide beside it (2026-10-03). The `in the way` line
  /// cannot see this — it never fences the second pair of neighbours — and
  /// the pictures draw the closed cells white and unnamed.
  [[nodiscard]] std::string deadOnArrival(const Wire& wire,
                                          const std::vector<Wire>& wires,
                                          const std::uint32_t straightStart) {
    const auto stub = static_cast<std::int64_t>(startStubOf(wire, straightStart));
    const auto& source = wire.objective.source;
    const auto v = routing::headingVector(source.heading);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const std::int64_t sx = static_cast<std::int64_t>(source.x) + (v.dx * stub);
    const std::int64_t sy = static_cast<std::int64_t>(source.y) + (v.dy * stub);
    // The start, the cell straight ahead and the two beside that one: the
    // least any first move sweeps.
    const std::int64_t px = -v.dy;
    const std::int64_t py = v.dx;
    const std::array<std::pair<std::int64_t, std::int64_t>, 4> first{
        {{sx, sy},
         {sx + v.dx, sy + v.dy},
         {sx + v.dx + px, sy + v.dy + py},
         {sx + v.dx - px, sy + v.dy - py}}};
    std::string closed;
    std::uint32_t shut = 0;
    for (const auto& [x, y] : first) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        ++shut;
        closed += std::format("{}({},{}) off the grid", closed.empty() ? "" : "; ", x, y);
        continue;
      }
      if (!corridor_.test(static_cast<std::size_t>((y * width) + x))) {
        continue;
      }
      ++shut;
      closed += std::format("{}({},{}) closed, within the clearance of {}",
                            closed.empty() ? "" : "; ", x, y,
                            whatIsNear(wire, wires, x, y));
    }
    if (shut == 0) {
      return {};
    }
    return std::format(" · dead on arrival: the search starts at ({},{}) "
                       "heading ({},{}) after a stub of {}, and {} of its "
                       "first {} cells are closed: {}",
                       sx, sy, v.dx, v.dy, stub, shut, first.size(), closed);
  }

  /// What lies within the clearance of a cell, by name: the artwork, a
  /// coupler body, and every wire whose way or fixed places reach it. Not
  /// every wire named is fenced in the search at hand; the line says what
  /// is there, the fences say what is closed.
  [[nodiscard]] std::string whatIsNear(const Wire& wire,
                                       const std::vector<Wire>& wires,
                                       const std::int64_t x,
                                       const std::int64_t y) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cell = static_cast<std::size_t>((y * width) + x);
    std::string names;
    if (scene_.blocked.test(cell)) {
      names = "artwork or keepout";
    }
    if (bodies_.test(cell)) {
      names += std::string(names.empty() ? "" : ", ") + "a coupler body";
    }
    const auto limit = static_cast<double>(tuning_.clearance);
    for (const auto& other : wires) {
      if (other.key == wire.key) {
        continue;
      }
      auto least = std::numeric_limits<double>::max();
      const char* what = "";
      for (const Path* const cells : {&other.way, &other.fixed}) {
        for (const auto& point : *cells) {
          const auto far = std::hypot(static_cast<double>(point.x) - static_cast<double>(x),
                                      static_cast<double>(point.y) - static_cast<double>(y));
          if (far < least) {
            least = far;
            what = cells == &other.way ? "way" : "fixed places";
          }
        }
      }
      if (least <= limit) {
        names += std::format("{}{} ({} at {:.1f} cells)",
                             names.empty() ? "" : ", ", wireId(other), what,
                             least);
      }
    }
    return names.empty() ? std::string("nothing named: the band, a port band or a body")
                         : names;
  }

  /// Whether the way a wire has holds every rule the pass judges by: the
  /// rule against every other wire, the crossing rule, and the length of a
  /// resonator.
  [[nodiscard]] bool isLegal(Wire& wire, const std::vector<Wire>& wires) {
    if (!wire.drawn || wire.way.empty()) {
      return false;
    }
    const bool placed = wire.placed;
    lift(wire);
    const bool open = conflictsOf(wire, wires) != 0;
    if (placed) {
      place(wire);
    }
    if (open || crossesAFeedline(wire, wires)) {
      return false;
    }
    if (needsLength(wire)) {
      const auto length = lengthOf(wire.way);
      const auto required = requiredLength(wire);
      const auto tolerance = exact_ ? tuning_.lengthTolerance : 0.0;
      if (length < required - tolerance ||
          (exact_ && length > required + tolerance)) {
        return false;
      }
    }
    return true;
  }

  /// How many cells of a way lie in the room of another wire.
  ///
  /// The wire itself has to be off the canvas when this runs. A cell where
  /// the two wires meet is not counted, because the rule does not bind there
  /// and the design-rule check makes the same exemption.
  [[nodiscard]] std::uint32_t conflictsOf(const Wire& wire,
                                          const std::vector<Wire>& wires) {
    return conflictsIn(wire.way, wire, wires);
  }

  /// Whether two wires meet at a place, so that the rule does not bind there.
  ///
  /// They meet when a terminal of one sits within the rule of a terminal of
  /// the other: two ports that close together have to be reached by two
  /// wires whose approaches converge, and no arrangement holds them apart.
  /// What belongs to the meeting is everything within one and a half rules
  /// of either terminal. This is the design-rule check's own test, and it is
  /// geometry alone: that two wires end on one component says nothing about
  /// where its ports are — the two ports of a qubit can sit nine hundred
  /// units apart, and a wire passing the other's approach there is as much a
  /// violation as anywhere else.
  [[nodiscard]] bool meetAt(const Wire& one, const Wire& two, const double x,
                            const double y) const {
    const auto link = static_cast<double>(tuning_.clearance);
    const auto reach = 1.5 * link;
    // A resonator and the feedline edges of its own coupler run beside each
    // other along the coupler on purpose: that is the coupling. Everything
    // within the coupler's reach of its anchor belongs to that meeting.
    if (const auto shared = sharedCoupler(one, two); shared != NO_OWNER) {
      const auto& anchor = couplers_[shared].anchor;
      if (std::hypot(x - anchor.x, y - anchor.y) <= couplerReach()) {
        return true;
      }
    }
    const std::array<const PathPoint*, 2> mine{&one.objective.source,
                                               &one.objective.target};
    const std::array<const PathPoint*, 2> theirs{&two.objective.source,
                                                 &two.objective.target};
    for (const auto* const a : mine) {
      for (const auto* const b : theirs) {
        const auto apart = std::hypot(static_cast<double>(a->x) - b->x,
                                      static_cast<double>(a->y) - b->y);
        if (apart > link) {
          continue;
        }
        if (std::hypot(x - a->x, y - a->y) <= reach ||
            std::hypot(x - b->x, y - b->y) <= reach) {
          return true;
        }
      }
    }
    return false;
  }

  /// The coupler two wires share, or none: a resonator and an edge of its
  /// own coupler's chain, or the two edges that meet at one coupler.
  [[nodiscard]] static std::uint32_t sharedCoupler(const Wire& one,
                                                   const Wire& two) {
    for (const auto a : {one.couplerAtSource, one.couplerAtTarget}) {
      if (a == NO_OWNER) {
        continue;
      }
      if (a == two.couplerAtSource || a == two.couplerAtTarget) {
        return a;
      }
    }
    return NO_OWNER;
  }

  /// How far the meeting at a coupler reaches from its anchor, in cells:
  /// the body and the clearance around it.
  /// How far from a coupler two edges of its own chain may be within the
  /// rule of each other.
  ///
  /// They pin on one cell — the coupler's feedline port — so around that
  /// cell the rule cannot bind, and the disc of the rule itself is what it
  /// takes to let them both stand there. `couplerReach` is three times as
  /// far and is the wrong figure here: it is the reach of the coupler's
  /// *geometry*, the run and the depth and the turns, and on 17q it left
  /// sixty-two cells of every neighbouring edge fenced by two cells of
  /// copper instead of nineteen of clearance — the thin fence visible at
  /// the start of every edge in the pictures.
  /// The heading the launcher nearest a cell faces.
  ///
  /// What a coupler's orientation is counted from. The heading the
  /// resonator's way happens to hold at the insertion point is a property
  /// of the routing, not of the chip: it changes whenever the outer routing
  /// changes, and offset 0 then means something different on every coupler.
  /// The nearest launcher's own heading is a fact of the chip, so offset 0
  /// is the same thing everywhere — the resonator leaves its coupler the
  /// way the launcher that drives it points.
  [[nodiscard]] Heading nearestLauncherHeading(const std::int64_t x,
                                               const std::int64_t y) const {
    Heading heading = 0;
    double nearest = std::numeric_limits<double>::max();
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      const auto distance = std::hypot(static_cast<double>(place.x()) - x,
                                       static_cast<double>(place.y()) - y);
      if (distance < nearest) {
        nearest = distance;
        heading = found->second;
      }
    }
    return heading;
  }

  [[nodiscard]] double couplerMeeting() const {
    return static_cast<double>(tuning_.clearance);
  }

  [[nodiscard]] double couplerReach() const {
    return static_cast<double>(tuning_.couplerLength + tuning_.couplerHeight +
                               (2 * BEND_RADIUS)) +
           (1.5 * static_cast<double>(tuning_.clearance));
  }

  /// The same for a way the wire does not have yet.
  ///
  /// The guard is only the first question. What the search keeps is nineteen
  /// whole cells and what the rule asks for is 185 layout units, which is
  /// 18.5 of them, so a cell the search refuses is not always a cell the rule
  /// refuses. A committed wire is judged by the rule, in layout units, which
  /// is the figure the design-rule check uses — otherwise the rounds chase
  /// encounters that nothing will ever report.
  [[nodiscard]] std::uint32_t
  conflictsIn(const Path& way, const Wire& wire,
              const std::vector<Wire>& wires,
              std::vector<std::uint32_t>* blockers = nullptr,
              std::vector<PathPoint>* where = nullptr) {
    // An edge of a chain is crossed by the wires between its couplers on
    // purpose, so the rule does not bind between it and a conventional or
    // an inner wire. Against a resonator it binds: a feedline keeps the
    // clearance from every resonator but its own coupler's, and there only
    // within the coupler's reach. Against another edge it binds too, so
    // that no feedline is ever drawn along another.
    const bool crossable = wire.feedline;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto limit = tuning_.spacing * tuning_.spacing;
    const auto& stencil = stencilFor(tuning_.clearance);
    std::uint32_t found = 0;
    for (const auto& point : way) {
      if (point.x >= scene_.router.width || point.y >= scene_.router.height) {
        continue;
      }
      const auto cell =
          (static_cast<std::size_t>(point.y) * scene_.router.width) + point.x;
      if (!field_.guarded(cell)) {
        continue;
      }
      // Somebody guards it. Whether the rule is broken is a question of how
      // far away that wire's copper really is, and of whether the two meet
      // there — which is the question the design-rule check asks, pair by
      // pair, and it has to be asked the same way here.
      for (const auto& [dx, dy] : stencil.full) {
        if (static_cast<double>((dx * dx) + (dy * dy)) > limit) {
          continue;
        }
        const auto x = static_cast<std::int64_t>(point.x) + dx;
        const auto y = static_cast<std::int64_t>(point.y) + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        const auto owner =
            field_.owner(static_cast<std::size_t>((y * width) + x));
        if (owner == NO_OWNER || owner == wire.key || owner >= wires.size()) {
          continue;
        }
        const auto& other = wires[owner];
        if ((crossable && !other.resonator && !other.feedline) ||
            (other.feedline && !wire.resonator && !wire.feedline)) {
          continue;
        }
        if (meetAt(wire, wires[owner], 0.5 * (point.x + x),
                   0.5 * (point.y + y))) {
          continue;
        }
        ++found;
        if (blockers != nullptr &&
            std::ranges::find(*blockers, owner) == blockers->end()) {
          blockers->push_back(owner);
          if (where != nullptr) {
            where->push_back(point);
          }
        }
        break;
      }
    }
    return found;
  }

  /// Whether two ways cross: they share a cell, or they are two diagonal
  /// steps through one unit square the other way round.
  ///
  /// The second case shares no cell at all — a step from (x,y) to
  /// (x+1,y+1) and one from (x+1,y) to (x,y+1) pass through each other
  /// without ever standing on the same pixel — and a test that only
  /// compares cells reports such a pair as clear.
  [[nodiscard]] static bool waysCross(const Path& one, const Path& two) {
    std::unordered_set<std::uint64_t> cells;
    cells.reserve(one.size() * 2);
    const auto cellKey = [](const PathPoint& at) {
      return (static_cast<std::uint64_t>(at.x) << 32U) | at.y;
    };
    for (const auto& at : one) {
      cells.insert(cellKey(at));
    }
    for (const auto& at : two) {
      if (cells.contains(cellKey(at))) {
        return true;
      }
    }
    // The diagonals of each unit square a way steps across, by the square's
    // lower-left corner and which way the step leans.
    const auto diagonals = [](const Path& way) {
      std::unordered_set<std::uint64_t> seen;
      for (std::size_t at = 0; at + 1 < way.size(); ++at) {
        const auto fromX = static_cast<std::int64_t>(way[at].x);
        const auto fromY = static_cast<std::int64_t>(way[at].y);
        const auto dx = static_cast<std::int64_t>(way[at + 1].x) - fromX;
        const auto dy = static_cast<std::int64_t>(way[at + 1].y) - fromY;
        if (dx == 0 || dy == 0 || dx * dx != 1 || dy * dy != 1) {
          continue;
        }
        const auto cornerX = std::min(fromX, fromX + dx);
        const auto cornerY = std::min(fromY, fromY + dy);
        const std::uint64_t lean = (dx * dy > 0) ? 1U : 0U;
        seen.insert((static_cast<std::uint64_t>(cornerX) << 33U) |
                    (static_cast<std::uint64_t>(cornerY) << 1U) | lean);
      }
      return seen;
    };
    const auto mine = diagonals(one);
    if (mine.empty()) {
      return false;
    }
    for (const auto key : diagonals(two)) {
      if (mine.contains(key ^ 1U)) {
        return true;
      }
    }
    return false;
  }

  /// Whether any feedline edge crosses a resonator.
  ///
  /// Past its lead a resonator keeps no room from a feedline — see
  /// `RESONATOR_COPPER` — but it is not crossable, and that is what this
  /// counts. The lead itself is left out: an edge docks on its coupler
  /// there, through the terminal slot, and `checkFeedlineRoom` is what has
  /// something to say about that stretch.
  [[nodiscard]] std::uint32_t
  checkResonatorCrossings(const std::vector<Wire>& wires,
                          const std::string_view stage =
                              "coupler insertion") {
    std::uint32_t pairs = 0;
    std::string named;
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& feedline = wires[edge.wire];
      if (!feedline.feedline || feedline.way.empty()) {
        continue;
      }
      for (const auto& coupler : couplers_) {
        const auto& option = coupler.options[coupler.chosen];
        const auto& way = wayOfResonator(wires[coupler.wire]);
        if (way.size() <= option.arc.size()) {
          continue;
        }
        const Path tail(way.begin() +
                            static_cast<std::ptrdiff_t>(option.arc.size()),
                        way.end());
        if (!waysCross(feedline.way, tail)) {
          continue;
        }
        ++pairs;
        if (pairs <= 12) {
          named += std::format("{}f{} x {}", named.empty() ? "" : " · ",
                               feedline.slot, wireId(wires[coupler.wire]));
        }
      }
    }
    say(std::format("{}: CHECK resonator crossings — {} "
                    "feedline{} cross a resonator{}{}",
                    stage, pairs, pairs == 1 ? "" : "s",
                    pairs == 0 ? "; the check is GREEN" : ": ", named));
    return pairs;
  }

  /// Whether any two edges that meet at a coupler cross each other.
  ///
  /// They pin on the same coupler on purpose, so no clearance binds between
  /// them there and `conflictsIn` says nothing about the pair. Crossing is
  /// still not allowed, and with `fenceLaterEdges` off it became possible:
  /// the skip that opens the later edges of a chain opened the neighbour
  /// with them, and the edge leaving a coupler was drawn through the edge
  /// arriving at it (user, 2026-10-03).
  [[nodiscard]] std::uint32_t checkCouplerCrossings(
      const std::vector<Wire>& wires,
      const std::string_view stage = "coupler insertion") {
    std::uint32_t pairs = 0;
    std::string named;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& edges = edgeWireOf_[chain];
      for (std::size_t at = 0; at + 1 < edges.size(); ++at) {
        if (edges[at] >= wires.size() || edges[at + 1] >= wires.size()) {
          continue;
        }
        const auto& one = wires[edges[at]];
        const auto& two = wires[edges[at + 1]];
        if (one.way.empty() || two.way.empty() ||
            !waysCross(one.way, two.way)) {
          continue;
        }
        ++pairs;
        if (pairs <= 12) {
          named += std::format("{}f{} x f{} (chain {})",
                               named.empty() ? "" : " · ", one.slot, two.slot,
                               chain);
        }
      }
    }
    say(std::format("{}: CHECK coupler crossings — {} pair{} "
                    "of edges meeting at a coupler cross{}{}",
                    stage, pairs, pairs == 1 ? "" : "s",
                    pairs == 0 ? "; the check is GREEN" : ": ", named));
    return pairs;
  }

  /// Whether every committed coupler lies wholly inside the box, measured on
  /// all three pieces of copper it puts on the chip.
  ///
  /// **The body alone is the wrong half.** The feedline runs along the pad's
  /// *far* edge, `halfDepth + 1` cells across the centre, so the body always
  /// lies inside the run and a check on the body says GREEN while the run
  /// sits on the box edge. On 17q that is five couplers of seventeen — 3 and
  /// 4 with their run on `maxX` 1347, 8 on `minY` 77, 12 on `minX` 77, 16
  /// four cells off `maxY` — and the edges f2, f8, f12/f13 and f17 then run
  /// the length of the chip along that line. So the check measures:
  ///
  /// - the **body**, `couplerLength` along the orientation by
  ///   `couplerHeight` across it, as `option.body` holds it;
  /// - the **run**, the pad's two feedline ports and the straight run the
  ///   chain is forced to make off each of them plus its turn — the pad's own
  ///   run at a coupler and not `straightStart`, see `edgeRunOffPort`;
  /// - the **lead**, the resonator's own quarter turn off the near edge and
  ///   the straight run after it, `option.arc`;
  /// - the **turn**, the `BEND_RADIUS` cells beyond the lead's tip on the
  ///   heading it holds — the room the resonator's search needs to bend at
  ///   all, which is the same thing the run's straight stubs need.
  ///
  /// Per piece: how many couplers reach outside, the worst overshoot in
  /// cells, and — so that the line carries a measured figure when it is
  /// green — how far the piece that comes nearest a side of the box still
  /// stays off it. A slack of 0 is a piece *on* the box edge: legal, and one
  /// cell from not being. The box side is the first cell a launcher's stub
  /// and its clearance leave open, so a coupler there keeps the design rule
  /// from that stub by exactly one cell and leaves the wire off that
  /// launcher nothing to turn in — see `launcherFenceTurn`.
  ///
  /// Said once, at the end of the insertion: a coupler does not move again —
  /// the feedline passes redraw edges and resonators, never a pad — so a
  /// second line in the refinement would re-print this one rather than
  /// re-measure it.
  [[nodiscard]] std::uint32_t checkCouplerBodies(
      const std::vector<Wire>& wires,
      const std::string_view stage = "coupler insertion") {
    /// One piece of copper, measured against the box.
    struct Piece {
      std::string_view what;
      std::uint32_t outside = 0;
      std::int64_t worst = 0;
      std::string named;
      std::int64_t tightest = std::numeric_limits<std::int64_t>::max();
      std::string closest;
    };
    std::array<Piece, 4> pieces{Piece{.what = "body"}, Piece{.what = "run"},
                                Piece{.what = "lead"}, Piece{.what = "turn"}};
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    // How far a cell reaches past a side of the box; negative inside. The
    // slack reported is to the box side itself, which is the figure a
    // picture shows: zero is a piece *on* the edge, which the rule allows.
    const auto past = [this](const std::int64_t x, const std::int64_t y) {
      return std::max({couplerBox_.minX - x, x - couplerBox_.maxX,
                       couplerBox_.minY - y, y - couplerBox_.maxY});
    };
    const auto measure = [&](Piece& piece, const std::string& id,
                             const std::vector<std::pair<std::int64_t,
                                                         std::int64_t>>& at) {
      std::int64_t over = 0;
      std::int64_t overX = 0;
      std::int64_t overY = 0;
      for (const auto& [x, y] : at) {
        const auto out = past(x, y);
        if (out > over) {
          over = out;
          overX = x;
          overY = y;
        }
        if (-past(x, y) < piece.tightest) {
          piece.tightest = -past(x, y);
          piece.closest = std::format("'{}' at ({},{})", id, x, y);
        }
      }
      if (over <= 0) {
        return;
      }
      ++piece.outside;
      piece.worst = std::max(piece.worst, over);
      if (piece.outside <= 8) {
        piece.named +=
            std::format("{}'{}' by {} cell{} at ({},{})",
                        piece.named.empty() ? "" : " · ", id, over,
                        over == 1 ? "" : "s", overX, overY);
      }
    };

    for (const auto& coupler : couplers_) {
      if (coupler.wire >= wires.size() ||
          coupler.chosen >= coupler.options.size()) {
        continue;
      }
      const auto& option = coupler.options[coupler.chosen];
      const auto id = wireId(wires[coupler.wire]);

      std::vector<std::pair<std::int64_t, std::int64_t>> at;
      at.reserve(option.body.size());
      for (const auto cell : option.body) {
        at.emplace_back(static_cast<std::int64_t>(cell) % width,
                        static_cast<std::int64_t>(cell) / width);
      }
      measure(pieces[0], id, at);

      // The run: the line through the two feedline ports, carried
      // `straightStart` past each of them on the orientation. Built the way
      // `makeOption` builds it, off the ports rather than off the centre, so
      // that a diagonal orientation is measured where its copper is.
      at.clear();
      const auto along = routing::headingVector(option.orientation);
      const auto fromX = static_cast<std::int64_t>(option.in.x);
      const auto fromY = static_cast<std::int64_t>(option.in.y);
      // The span from port to port, read off the two ports themselves.
      // **Not `option.run`**: that is `cellsOn(couplerLength, orientation)`
      // and the ports sit at `± cellsOn(...) / 2` of the centre, so on an odd
      // run the two differ by a cell and the check reported a cell of
      // overshoot that `makeOption` had never allowed (17q coupler '40').
      const auto span = std::max(
          std::abs(static_cast<std::int64_t>(option.out.x) - fromX),
          std::abs(static_cast<std::int64_t>(option.out.y) - fromY));
      // The same figure `terminalsInBox` and `corridorOfEdge` use, which at
      // a coupler port is the pad's own run and not `straightStart` — see
      // `edgeRunOffPort`.
      const auto off = edgeRunOffPort(option.orientation);
      for (std::int64_t k = -off; k <= span + off; ++k) {
        at.emplace_back(fromX + (k * along.dx), fromY + (k * along.dy));
      }
      measure(pieces[1], id, at);

      at.clear();
      at.reserve(option.arc.size());
      for (const auto& step : option.arc) {
        at.emplace_back(static_cast<std::int64_t>(step.x),
                        static_cast<std::int64_t>(step.y));
      }
      measure(pieces[2], id, at);

      // The quarter turn the resonator's search may want off the lead's tip.
      at.clear();
      const auto onward = routing::headingVector(option.arcEnd.heading);
      const auto beyondTip =
          static_cast<std::int64_t>(BEND_RADIUS) +
          (resonatorStub() ? static_cast<std::int64_t>(tuning_.straightStart)
                           : 0);
      for (std::int64_t k = 1; k <= beyondTip; ++k) {
        at.emplace_back(
            static_cast<std::int64_t>(option.arcEnd.x) + (k * onward.dx),
            static_cast<std::int64_t>(option.arcEnd.y) + (k * onward.dy));
      }
      measure(pieces[3], id, at);
    }

    std::uint32_t outside = 0;
    std::string line;
    for (auto& piece : pieces) {
      outside += piece.outside;
      if (piece.closest.empty()) {
        piece.tightest = 0;
        piece.closest = "nothing";
      }
      line += std::format("{}{} {} outside (worst {}, tightest {} off, {}){}",
                          line.empty() ? "" : "; ", piece.outside, piece.what,
                          piece.worst, piece.tightest, piece.closest,
                          piece.named.empty() ? "" : ": " + piece.named);
    }
    say(std::format("{}: CHECK coupler bodies — of {} coupler{} against the "
                    "box x {}..{} y {}..{}: {}{}",
                    stage, couplers_.size(), couplers_.size() == 1 ? "" : "s",
                    couplerBox_.minX, couplerBox_.maxX, couplerBox_.minY,
                    couplerBox_.maxY, line,
                    outside == 0 ? "; the check is GREEN" : ""));
    return outside;
  }

  /// The pairs of drawn edges of **different** chains whose ways cross, by
  /// edge index, in edge order. Two edges of one chain are
  /// `checkCouplerCrossings`'s business where they meet at a coupler and the
  /// clearance rule's elsewhere; two edges of different chains have no
  /// business meeting at all. Prefiltered by the boxes the two ways span, so
  /// 69q's 81 edges are 3240 box tests and a handful of `waysCross`.
  [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>>
  feedlineCrossingPairs(const std::vector<Wire>& wires) const {
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    std::vector<Span> boxes(edges_.size());
    std::vector<bool> drawn(edges_.size(), false);
    for (std::size_t at = 0; at < edges_.size(); ++at) {
      const auto key = edges_[at].wire;
      if (key < wires.size() && !wires[key].way.empty()) {
        boxes[at] = reachOf(wires[key].way);
        drawn[at] = true;
      }
    }
    for (std::size_t i = 0; i < edges_.size(); ++i) {
      if (!drawn[i]) {
        continue;
      }
      for (std::size_t j = i + 1; j < edges_.size(); ++j) {
        if (!drawn[j] || edges_[j].chain == edges_[i].chain ||
            !boxes[i].meets(boxes[j]) ||
            !waysCross(wires[edges_[i].wire].way, wires[edges_[j].wire].way)) {
          continue;
        }
        pairs.emplace_back(i, j);
      }
    }
    return pairs;
  }

  /// **The fourth guarantee** (2026-10-04): no feedline crosses a feedline
  /// of another chain. The three checks before it look at leads, couplers
  /// and resonators, and `final routing: … crossing a feedline` counts plain
  /// wires only, so on 69q the terminal edges of four pairs of neighbouring
  /// chains — f32/f33, f46/f47, f60/f61, f73/f74 — crossed each other twice
  /// each and no line said so (`artifacts/logs/fcross.py`, R4 of the room
  /// rules). `SCPD_EDGE_SEES_CHAINS` is off, so the insertion never routes
  /// an edge against another chain; this is where that shows. The same
  /// `waysCross` the coupler check uses, which finds the two diagonals
  /// through one unit square as well as a shared cell.
  ///
  /// Said twice: at the end of the insertion, where the four 69q pairs
  /// stand, and at the end of phase 4 after the repair (`feedline routing:`),
  /// where the pass has redrawn the terminal edges — the line a chip is
  /// judged by is the second.
  ///
  /// @returns How many pairs of edges of different chains cross.
  [[nodiscard]] std::uint32_t
  checkFeedlineCrossings(const std::vector<Wire>& wires,
                         const std::string_view stage =
                             "coupler insertion") const {
    const auto pairs = feedlineCrossingPairs(wires);
    std::string named;
    for (std::size_t k = 0; k < pairs.size() && k < 12; ++k) {
      const auto& one = edges_[pairs[k].first];
      const auto& two = edges_[pairs[k].second];
      named += std::format("{}{} (chain {}) x {} (chain {})",
                           named.empty() ? "" : " · ",
                           wireId(wires[one.wire]), one.chain,
                           wireId(wires[two.wire]), two.chain);
    }
    say(std::format("{}: CHECK feedline crossings — {} pair{} of edges of "
                    "different chains cross{}{}",
                    stage, pairs.size(), pairs.size() == 1 ? "" : "s",
                    pairs.empty() ? "; the check is GREEN" : ": ", named));
    return static_cast<std::uint32_t>(pairs.size());
  }

  /// **The check the coupler insertion is handed over on**: no feedline may
  /// come closer to what the corridor fences than the design rule allows.
  ///
  /// It measures **what is actually routed against**, not a rule of its own
  /// (user, 2026-10-03). That is three things and nothing else, the same
  /// three `corridorOfEdge` builds:
  ///
  /// 1. What is fenced — every coupler's lead under `fenceTheLeadOnly`, or
  ///    every resonator's whole way without it.
  /// 2. Inflated by the design rule, taken unrounded in layout units as
  ///    `conflictsIn` takes it, so a cell the whole-cell stencil refuses is
  ///    not always a cell the rule refuses.
  /// 3. Less the terminal slots, which are the one thing the corridor opens
  ///    again and are opened here by the same figures — see `terminalSlot`.
  ///
  /// So a green line means the insertion built what it said it would, and a
  /// red one means an edge sits somewhere the corridor had closed. It is no
  /// longer a second opinion about the design rule; `checkClearance` in the
  /// DRC is that, and it has the whole chip to look at rather than this
  /// stage's own fence.
  ///
  /// @returns How many feedline/coupler pairs break it.
  [[nodiscard]] std::uint32_t
  checkFeedlineRoom(std::vector<Wire>& wires,
                    const std::string_view stage = "coupler insertion") {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    // What the corridor closes, by coupler, so a pair can be named.
    std::unordered_map<std::size_t, std::uint32_t> fenced;
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      const auto& coupler = couplers_[index];
      const auto& option = coupler.options[coupler.chosen];
      const Path& path = fenceTheLeadOnly()
                             ? option.arc
                             : wayOfResonator(wires[coupler.wire]);
      for (const auto& point : path) {
        if (point.x < scene_.router.width && point.y < scene_.router.height) {
          fenced.emplace(scene_.router.index(point.x, point.y), index);
        }
      }
    }
    const auto limit = tuning_.spacing * tuning_.spacing;
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto turn = static_cast<std::uint32_t>(BEND_RADIUS);
    std::uint32_t pairs = 0;
    std::string named;
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.feedline || wire.way.empty()) {
        continue;
      }
      // The slots this edge docks in, by the figures the corridor used.
      std::unordered_set<std::size_t> slot;
      const auto [startRun, endRun] = runsOfEdge(edge);
      const auto keep = [&](const Path& run) {
        for (const auto& point : run) {
          if (point.x < scene_.router.width &&
              point.y < scene_.router.height) {
            slot.insert(scene_.router.index(point.x, point.y));
          }
        }
      };
      keep(router_.straightStub(wire.objective.source, false,
                                std::max(terminalSlot(), startRun + turn)));
      keep(router_.straightStub(wire.objective.target, true,
                                std::max(terminalSlot(), endRun + turn)));
      // One pair per coupler, however many cells of the way reach it.
      std::unordered_set<std::uint32_t> told;
      for (const auto& point : wire.way) {
        if (point.x >= scene_.router.width ||
            point.y >= scene_.router.height) {
          continue;
        }
        if (slot.contains(scene_.router.index(point.x, point.y))) {
          continue;
        }
        for (const auto& [dx, dy] : stencil.full) {
          if (static_cast<double>((dx * dx) + (dy * dy)) > limit) {
            continue;
          }
          const auto x = static_cast<std::int64_t>(point.x) + dx;
          const auto y = static_cast<std::int64_t>(point.y) + dy;
          if (x < 0 || y < 0 || x >= width || y >= height) {
            continue;
          }
          const auto found =
              fenced.find(static_cast<std::size_t>((y * width) + x));
          if (found == fenced.end() || !told.insert(found->second).second) {
            continue;
          }
          ++pairs;
          if (pairs <= 12) {
            named += std::format("{}f{} vs {} at ({},{})",
                                 named.empty() ? "" : " · ", wire.slot,
                                 wireId(wires[couplers_[found->second].wire]),
                                 point.x, point.y);
          }
        }
      }
    }
    say(std::format(
        "{}: CHECK feedline room — {} feedline/{} pair{} "
        "closer than the rule{}{}",
        stage, pairs, fenceTheLeadOnly() ? "lead" : "resonator",
        pairs == 1 ? "" : "s",
        pairs == 0 ? "; the check is GREEN" : ": ", named));
    return pairs;
  }

  /// Which wires an open wire lies in the room of, by name. The wire has to
  /// be off the canvas when this runs, as `conflictsIn` asks.
  [[nodiscard]] std::string whoBlocks(const Wire& wire,
                                      const std::vector<Wire>& wires) {
    std::vector<std::uint32_t> blockers;
    std::vector<PathPoint> where;
    conflictsIn(wire.way, wire, wires, &blockers, &where);
    std::string named;
    for (std::size_t at = 0; at < blockers.size(); ++at) {
      const auto& other = wires[blockers[at]];
      std::string note;
      if (at < where.size()) {
        const auto& cell = where[at];
        const auto far = [&cell](const PathPoint& end) {
          return std::hypot(static_cast<double>(cell.x) - end.x,
                            static_cast<double>(cell.y) - end.y);
        };
        const auto fromSource = far(wire.objective.source);
        const auto fromTarget = far(wire.objective.target);
        const auto stub = static_cast<double>(
            std::max(startStubOf(wire, tuning_.straightStart), wire.endStub));
        note = std::format(
            " at ({},{}), {:.0f} from its source and {:.0f} from its target, "
            "stub {:.0f}{}",
            cell.x, cell.y, fromSource, fromTarget, stub,
            std::min(fromSource, fromTarget) <= stub ? " — IN ITS OWN STUB"
                                                     : "");
      }
      named += (named.empty() ? "" : "; ") + wireId(other) + note;
    }
    return named.empty() ? std::string("-") : named;
  }

  /// The search itself, with the stubs of this pass. Its picture is taken
  /// here unless the caller has more to put into it.
  [[nodiscard]] Path search(const Wire& wire, const std::uint32_t straightStart,
                            const bool usePenalty, const bool draw = true) {
    router_.setParams({.startStraightLength = startStubOf(wire, straightStart),
                       .endStraightLength = wire.endStub,
                       .minRadius = BEND_RADIUS,
                       .bendPenalty = tuning_.bendPenalty});
    // The field this search is priced by, said every time rather than once.
    //
    // `routeEdge` hands the router a field of zeroes — an edge of a chain is
    // searched without a price — and it was never handed back. The router
    // keeps a pointer, so from the first edge of the coupler insertion
    // onward every later search read zeroes: the lane, the halos, the bands
    // along the feedlines and the toll were all written into a field nobody
    // looked at. It is why four sweeps over the prices moved nothing and why
    // a toll of twenty-seven bends a cell left the ways exactly where they
    // were (user, 2026-10-01).
    router_.attachWireProximity(&proximity_);
    router_.attachCorridor(&corridor_);
    lastPicture_.clear();
    // The crossing rule does not bind around a resonator's own coupler,
    // where the edges of its chain pin and run beside it on purpose.
    std::vector<std::uint32_t> exempt;
    if (feedlinePass_ && wire.couplerAtSource != NO_OWNER) {
      const auto& anchor = couplers_[wire.couplerAtSource].anchor;
      const auto reach = static_cast<std::int64_t>(std::ceil(couplerReach()));
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      for (std::int64_t dy = -reach; dy <= reach; ++dy) {
        for (std::int64_t dx = -reach; dx <= reach; ++dx) {
          const auto x = static_cast<std::int64_t>(anchor.x) + dx;
          const auto y = static_cast<std::int64_t>(anchor.y) + dy;
          if (x < 0 || y < 0 || x >= width || y >= scene_.router.height ||
              std::hypot(static_cast<double>(dx), static_cast<double>(dy)) >
                  couplerReach()) {
            continue;
          }
          exempt.push_back(static_cast<std::uint32_t>((y * width) + x));
        }
      }
    }
    router_.setCrossingExemption(exempt);
    // A wire under the feedline constraints crosses a feedline at a right
    // angle only; a feedline edge itself is searched free, as the
    // prototype searches it.
    auto found = feedlinePass_ && !wire.feedline && orthoCrossing()
                     ? router_.routeOrthogonal(wire.objective, usePenalty)
                     : router_.route(wire.objective, usePenalty);
    // A resonator drawn from its coupler leaves it on a fixed quarter turn
    // that no search sees; the way it has is the turn and what was found.
    if (!found.empty() && !wire.arc.empty()) {
      Path whole = wire.arc;
      whole.insert(whole.end(), found.begin(), found.end());
      // The search never saw the head, so a way that comes back over it is
      // a way that meets itself, which the router's own guard would have
      // refused: no way, by the same test.
      if (routing::pathSelfIntersects(whole, scene_.router.width,
                                      scene_.router.height, loopScratch_)) {
        found.clear();
      } else {
        found = std::move(whole);
      }
    }
    if (draw) {
      drawSearch(wire, found, {});
    }
    return found;
  }

  /// The straight run a wire leaves its source on, in cells: its own figure
  /// where it has one, the pass's for a wire fed from the ring, and none
  /// for a resonator fed from the segment between two launchers.
  [[nodiscard]] static std::uint32_t
  startStubOf(const Wire& wire, const std::uint32_t straightStart) {
    if (wire.startStub.has_value()) {
      return *wire.startStub;
    }
    return wire.resonator ? 0U : straightStart;
  }

  /// The cells a search may enter: the free space within `reach` steps of the
  /// way the wire has.
  ///
  /// This is the prototype's `expand_path`: a walk over the cells that are not
  /// artwork, so the band follows the way rather than boxing it and does not
  /// reach around an obstacle. It knows nothing of other wires. What fences a
  /// search in is `fence`, and nothing else.
  void buildCorridor(const Wire& wire, const std::uint32_t asked) {
    // The prototype widens the band of the feedline pass by one band per
    // round (`FinalGrid.cpp:4254`), where its outer routing holds it at one.
    // A wire that found nothing in round 0 is offered more room in round 1
    // rather than the same room again, which is what stops the sweep from
    // repeating itself.
    const std::uint32_t reach =
        (feedlinePass_ && feedlineLikePrototype())
            ? (frame_.round + 1U) * asked
            : asked;
    blockOnBodies_ = !wire.feedline;
    corridor_.fill(true);
    box_ = {};
    if (wire.way.empty()) {
      return;
    }
    // The band cannot leave the box the way spans grown by the reach, so
    // neither the walk nor anything that reads the band afterwards has to
    // touch the rest of the grid. On the 21-qubit chip a short wire's box is
    // a fortieth of it.
    box_ = boxOf(wire, reach);
    ++pass_;
    if (pass_ == 0) {
      std::ranges::fill(seen_, 0);
      pass_ = 1;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    queue_.clear();
    depth_.clear();

    const auto visit = [&](const std::int64_t x, const std::int64_t y,
                           const std::uint32_t depth) {
      if (x < box_.minX || y < box_.minY || x > box_.maxX || y > box_.maxY) {
        return;
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      // A coupler body stands in the way of a wire that has to go around it,
      // but not of a feedline: the feedline is what the coupler is there to
      // meet, and it runs along the body on purpose.
      if (seen_[cell] == pass_ || scene_.blocked.test(cell) ||
          (blockOnBodies_ && bodies_.test(cell))) {
        return;
      }
      seen_[cell] = pass_;
      queue_.push_back(cell);
      depth_.push_back(depth);
      corridor_.set(cell, false);
    };

    for (const auto& point : wire.way) {
      visit(point.x, point.y, 0);
    }
    for (std::size_t head = 0; head < queue_.size(); ++head) {
      const auto depth = depth_[head];
      if (depth >= reach) {
        continue;
      }
      const auto cell = queue_[head];
      const auto x = static_cast<std::int64_t>(cell % scene_.router.width);
      const auto y = static_cast<std::int64_t>(cell / scene_.router.width);
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          if (dx != 0 || dy != 0) {
            visit(x + dx, y + dy, depth + 1);
          }
        }
      }
    }
    openFixedPlaces(wire);
  }

  /// Fence a search in: the ways of these wires, inflated by the clearance,
  /// are closed to it.
  ///
  /// This is the prototype's `mark_obstacles` with `min_dist_wires`: the disc
  /// of the clearance around every cell of a way, stamped by its leading
  /// edge. Where the wire and a fence wire meet — a terminal of one within
  /// the rule of a terminal of the other — the disc leaves the meeting open,
  /// which is the design-rule check's own
  /// exemption in its narrow form: the fence wire's copper itself stays
  /// closed. The wire's own fixed places are opened again last, because it
  /// has to be able to stand on them.
  void fence(const Wire& wire, std::initializer_list<const Wire*> others) {
    for (const Wire* const other : others) {
      if (other == &wire) {
        continue;
      }
      closeRoomOf(wire, *other, other->way);
      // The fixed places with it, where they are not in the way: the lead
      // and the straight run of a resonator the insertion has just spliced,
      // which its next search has to make and no neighbour may take. See
      // `fenceFixed`.
      if (fenceFixed()) {
        closeRoomOf(wire, *other, other->fixed);
      }
    }
    openFixedPlaces(wire);
  }

  /// Keep the fixed places of the wires let go of closed while everything
  /// else of them is crossable — the prototype's `ripped_heads`, for every
  /// wire. See `fenceFixed`. Nothing happens with the switch off.
  void fenceFixedPlaces(const Wire& wire,
                        const std::vector<Wire>& wires,
                        const std::vector<std::pair<std::uint32_t, bool>>&
                            released) {
    if (!fenceFixed()) {
      return;
    }
    for (const auto& [key, routed] : released) {
      closeRoomOf(wire, wires[key], wires[key].fixed);
    }
    openFixedPlaces(wire);
  }

  /// Close the clearance around some cells of another wire — its way or its
  /// fixed places — except where the two wires meet.
  void closeRoomOf(const Wire& wire, const Wire& other, const Path& cells) {
    if (cells.empty()) {
      return;
    }
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const bool meeting = couldMeet(wire, other);
    alongDisc(cells, stencil, [&](const std::int64_t x, const std::int64_t y) {
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (meeting && field_.owner(cell) != other.key &&
          meetAt(wire, other, static_cast<double>(x),
                 static_cast<double>(y))) {
        return;
      }
      corridor_.set(cell, true);
    });
  }

  /// Close some cells of another wire by the length-point radius, except
  /// where the two meet and except within reach of the routed wire's own
  /// **fixed places** — its ends, the straight run out of its source and the
  /// run into its target. The prototype guards the two ends alone
  /// (`lenpt_covers_terminal_of`, `k + 2`) and falls back to a search without
  /// the constraint when a wire fails; here the constraint is hard, and the
  /// first version with the ends alone left 17q's wires 2 and 31 dead on
  /// arrival in every round of the outer routing: the length point of the
  /// resonator beside them lies 24 cells from their launcher stub, and a
  /// band cell 30 cells from the source still closed, inflated by `k`, the
  /// cells the search has to step onto behind the stub. A wire's fixed
  /// places are where it has to be; no band may close them or the cells a
  /// first move out of them sweeps.
  void closeBandCells(const Wire& wire, const Wire& other, const Path& cells,
                      const std::uint32_t k) {
    if (cells.empty() || k == 0) {
      return;
    }
    const double guard = std::max(static_cast<double>(k) + 2.0,
                                  1.5 * static_cast<double>(tuning_.clearance));
    const auto nearAnEnd = [&](const PathPoint& cell) {
      for (const auto* const end :
           {&wire.objective.source, &wire.objective.target}) {
        if (std::hypot(static_cast<double>(cell.x) - end->x,
                       static_cast<double>(cell.y) - end->y) <= guard) {
          return true;
        }
      }
      return !wire.fixed.empty() && distanceToWay(cell, wire.fixed) <= guard;
    };
    Path kept;
    kept.reserve(cells.size());
    for (const auto& cell : cells) {
      if (!nearAnEnd(cell)) {
        kept.push_back(cell);
      }
    }
    if (kept.empty()) {
      return;
    }
    const auto& stencil = stencilFor(k);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const bool meeting = couldMeet(wire, other);
    alongDisc(kept, stencil, [&](const std::int64_t x, const std::int64_t y) {
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (meeting && field_.owner(cell) != other.key &&
          meetAt(wire, other, static_cast<double>(x),
                 static_cast<double>(y))) {
        return;
      }
      corridor_.set(cell, true);
    });
  }

  /// The length-point clearance of the outer routing, for one search: for
  /// every fenced neighbour that is a resonator, its band inflated by
  /// `lengthPointK` instead of the clearance; and when the wire being
  /// routed is itself a resonator, the stretch of each fenced neighbour that
  /// runs alongside its own band — its two marks projected onto the
  /// neighbour's way, the way between the projections — inflated by the
  /// same radius. Nothing under the feedline constraints, nothing at `k = 0`.
  /// The wire's own fixed places are opened again afterwards, as `fence`
  /// opens them.
  void closeLengthBands(const Wire& wire,
                        std::initializer_list<const Wire*> others,
                        const Pass& pass) {
    const auto k = lengthPointK();
    if (pass.feedlines || k == 0 || !lengthPointActive_) {
      return;
    }
    for (const Wire* const other : others) {
      if (other == nullptr || other == &wire || other->way.empty()) {
        continue;
      }
      // One zone per neighbour (user, 2026-10-04): a resonator neighbour is
      // stamped with its own band and nothing else; the stretch alongside
      // the routed resonator's band is stamped only on a neighbour that has
      // no band of its own. A plain wire beside a routed resonator used to
      // carry both and showed two inflated zones in the pictures.
      if (other->resonator) {
        const auto marks = lengthMarksOf(*other);
        if (marks.valid) {
          closeBandCells(wire, *other,
                         Path(other->way.begin() +
                                  static_cast<std::ptrdiff_t>(marks.lo),
                              other->way.begin() +
                                  static_cast<std::ptrdiff_t>(marks.hi + 1)),
                         k);
        }
        continue;
      }
      if (wire.resonator) {
        const auto marks = lengthMarksOf(wire);
        if (marks.valid) {
          const auto& lo = wire.way[marks.lo];
          const auto& hi = wire.way[marks.hi];
          std::size_t atLo = 0;
          std::size_t atHi = 0;
          double bestLo = std::numeric_limits<double>::max();
          double bestHi = std::numeric_limits<double>::max();
          for (std::size_t n = 0; n < other->way.size(); ++n) {
            const auto& cell = other->way[n];
            const auto dLo = std::hypot(static_cast<double>(cell.x) - lo.x,
                                        static_cast<double>(cell.y) - lo.y);
            const auto dHi = std::hypot(static_cast<double>(cell.x) - hi.x,
                                        static_cast<double>(cell.y) - hi.y);
            if (dLo < bestLo) {
              bestLo = dLo;
              atLo = n;
            }
            if (dHi < bestHi) {
              bestHi = dHi;
              atHi = n;
            }
          }
          const auto from = std::min(atLo, atHi);
          const auto to = std::max(atLo, atHi);
          closeBandCells(
              wire, *other,
              Path(other->way.begin() + static_cast<std::ptrdiff_t>(from),
                   other->way.begin() + static_cast<std::ptrdiff_t>(to + 1)),
              k);
        }
      }
    }
    openFixedPlaces(wire);
  }

  /// A wire's own fixed places are always enterable. They are where the wire
  /// has to be, and the search is the one pass that may stand on them.
  void openFixedPlaces(const Wire& wire) {
    for (const auto& point : wire.fixed) {
      if (point.x < scene_.router.width && point.y < scene_.router.height) {
        corridor_.setCell(point.x, point.y, false);
      }
    }
  }

  /// Whether two wires meet anywhere: a terminal of one within the rule of a
  /// terminal of the other. The cheap half of `meetAt`, asked once per fence
  /// wire before every cell is asked.
  [[nodiscard]] bool couldMeet(const Wire& one, const Wire& two) const {
    // Two wires on one coupler meet there whatever their terminals do.
    //
    // This is the cheap gate `fence` asks once per fence wire before it asks
    // `meetAt` of every cell, and it used to ask about terminals alone. So
    // the coupler exemption `meetAt` carries was never reached from the
    // fence: the resonator of a coupler had its whole clearance disc closed
    // against the edge that leaves that same coupler, the edge's own start
    // stub included, and the edge had no first move to make. On 33q it cost
    // every one of the seven last edges — f5, f11, f17, f22, f27, f33, f39 —
    // which failed their search in all six rounds, each one fenced by the
    // resonator standing beside it in the ring (user, 2026-10-01).
    //
    // The rule is not being given up: `meetAt` still decides cell by cell,
    // and it opens only what lies within the coupler's reach of its anchor.
    if (sharedCoupler(one, two) != NO_OWNER) {
      return true;
    }
    const auto link = static_cast<double>(tuning_.clearance);
    for (const auto* const a : {&one.objective.source, &one.objective.target}) {
      for (const auto* const b :
           {&two.objective.source, &two.objective.target}) {
        if (std::hypot(static_cast<double>(a->x) - b->x,
                       static_cast<double>(a->y) - b->y) <= link) {
          return true;
        }
      }
    }
    return false;
  }

  /// Apply something to every cell of a disc around every cell of a way, once
  /// per step: the full disc at the first cell and the leading edge of the
  /// disc at each step after it, as the clearance field charges its own.
  /// Cells off the grid are skipped.
  template <typename Apply>
  void alongDisc(const Path& way, const Stencil& stencil, Apply&& apply,
                 const std::size_t from = 0) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto stamp = [&](const Stencil::Offsets& offsets,
                           const std::int64_t cx, const std::int64_t cy) {
      for (const auto& [dx, dy] : offsets) {
        const auto x = cx + dx;
        const auto y = cy + dy;
        if (x >= 0 && y >= 0 && x < width && y < height) {
          apply(x, y);
        }
      }
    };
    bool started = false;
    std::int64_t lastX = 0;
    std::int64_t lastY = 0;
    for (std::size_t step = from; step < way.size(); ++step) {
      const auto& point = way[step];
      const auto x = static_cast<std::int64_t>(point.x);
      const auto y = static_cast<std::int64_t>(point.y);
      if (started) {
        const auto sx = x - lastX;
        const auto sy = y - lastY;
        if (sx == 0 && sy == 0) {
          continue;
        }
        if (sx >= -1 && sx <= 1 && sy >= -1 && sy <= 1) {
          stamp(stencil.edge[static_cast<std::size_t>(sy + 1)]
                            [static_cast<std::size_t>(sx + 1)],
                x, y);
          lastX = x;
          lastY = y;
          continue;
        }
      }
      stamp(stencil.full, x, y);
      started = true;
      lastX = x;
      lastY = y;
    }
  }

  /// The price of leaving the lane between a wire's two ring neighbours.
  ///
  /// This is the prototype's `compute_corridor_polygon_proximity`, and it is
  /// what steers the relaxation. The lane is the closed polygon that runs from
  /// the wire's own source along the way of the neighbour before it, back to
  /// the wire's own target, and along the way of the neighbour after it. Every
  /// cell outside that polygon is priced; nothing is forbidden. So a wire that
  /// has to leave its lane may, and a wire that need not, does not.
  ///
  /// On top of it the wires further ahead along the sweep are priced at
  /// growing distances, so that a wire let go of is pushed away rather than
  /// walked over.
  ///
  /// And on top of that, not in the prototype, the approaches of the wires
  /// around this one cost ten times the price. A wire let go of is
  /// crossable, but the straight run it leaves its source on and the run it
  /// arrives at its target on are the two places it cannot be drawn anywhere
  /// else: a way through either takes them for good, and the wire let go of
  /// has no way back. The ring neighbours are fenced already; their
  /// approaches are priced all the same, so that a way that has to pass them
  /// passes wide.
  void priceLane(const std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members,
                 const std::uint32_t slot, const bool forward,
                 const std::uint32_t level) {
    const auto total = static_cast<std::uint32_t>(members.size());
    const auto& wire = wires[members[slot]];
    const auto& before = wires[members[(slot + total - 1) % total]];
    const auto& after = wires[members[(slot + 1) % total]];

    // The prototype's feedline pass builds the polygon from other ways than
    // the two neighbours (`Driver::laneOf`), prices the discs of the wires it
    // let go of at one clearance rather than one per level
    // (`FinalGrid.cpp:4612` against `:3151`), and knows nothing of the
    // ten-fold price on the approaches, which is ours.
    const bool proto = feedlinePass_ && feedlineLikePrototype();

    const auto price = tuning_.wireProximityPenalty;
    std::ranges::fill(proximity_, 0);
    if (proto) {
      const auto lane = laneOf(wires, members, slot);
      frame_.laneBefore = lane.before;
      frame_.laneAfter = lane.after;
      frame_.laneCurrent = lane.current;
      if (!lane.rule.empty()) {
        tell(std::format("    lane {} · {}before {}, after {}, own {} cells",
                         wireId(wire), lane.rule, lane.before.size(),
                         lane.after.size(), lane.current.size()));
      }
      if (feedlineLane()) {
        fillLane(lane.current, lane.before, lane.after, price, 0);
      }
    } else if (outerLane()) {
      // The outer routing kept the order it had: the ground first, the lane
      // freed after, so a degenerate polygon leaves everything priced.
      std::ranges::fill(proximity_, price);
      fillLane(wire.way, before.way, after.way, price, 0);
      frame_.laneBefore = before.way;
      frame_.laneAfter = after.way;
      frame_.laneCurrent = wire.way;
    } else {
      // No polygon: the passing penalty below is the whole price of the
      // relaxation. See `outerLane`.
      frame_.laneBefore.clear();
      frame_.laneAfter.clear();
      frame_.laneCurrent.clear();
    }

    const auto neighbour =
        forward ? (slot + 1) % total : (slot + total - 1) % total;
    // The feedline pass's passing penalty, or the outer routing's own under
    // `outerPenalty` = 1: flat halos and the toll instead of halos that grow
    // by the level.
    const bool flat = proto || outerPenalty() == 1;
    // Under the feedline constraints `priceTheReleased` decides; in the
    // outer routing `outerPenalty` does, and 2 prices nothing.
    const bool priced = proto ? priceTheReleased() : (outerPenalty() != 2 && priceTheReleased());
    for (std::uint32_t step = 0; priced && step <= level; ++step) {
      const auto at = forward ? (neighbour + step) % total
                              : (neighbour + total - step) % total;
      const auto& released = wires[members[at]].way;
      stampHalo(released,
                flat ? haloReach() * tuning_.clearance
                     : (step + 1) * tuning_.clearance,
                static_cast<std::uint8_t>(std::min<std::uint32_t>(
                    127U, releasedFactor() * price)));
      // The toll goes on last, so it stands whatever the halo left: it is a
      // charge for the one move that crosses, not a share of the room.
      if (flat && crossToll() > 0) {
        stampDisc(released, crossTollWidth(),
                  static_cast<std::uint16_t>(crossToll()));
      }
    }

    if (proto || !priceTheApproaches()) {
      return;
    }

    const auto strong =
        static_cast<std::uint8_t>(std::min<std::uint32_t>(127U, 10U * price));
    std::vector<std::uint32_t> around{before.key, after.key};
    for (std::uint32_t step = 0; step <= level; ++step) {
      const auto at = forward ? (neighbour + step) % total
                              : (neighbour + total - step) % total;
      around.push_back(wires[members[at]].key);
    }
    std::ranges::sort(around);
    const auto twice = std::ranges::unique(around);
    around.erase(twice.begin(), twice.end());
    for (const auto key : around) {
      if (key != wire.key) {
        priceApproaches(wires[key], strong);
      }
    }
  }

  /// Price the two approaches of a wire: the straight run out of its source
  /// along the heading it leaves on, and the run into its target along the
  /// heading it arrives on, each as long as a port's band and twice as wide
  /// as the wire clearance, in the band's own geometry.
  void priceApproaches(const Wire& other, const std::uint16_t price) {
    if (!other.feasible) {
      return;
    }
    priceApproach(other.objective.source, other.objective.source.heading, true,
                  price);
    priceApproach(other.objective.target, other.objective.target.heading, false,
                  price);
  }

  /// One approach: from `end` along `heading` where the wire leaves there,
  /// against it where the wire arrives there. A price only ever rises.
  void priceApproach(const PathPoint& end, const Heading heading,
                     const bool leaving, const std::uint16_t price) {
    const auto v = routing::headingVector(heading);
    if (v.dx == 0 && v.dy == 0) {
      return;
    }
    const std::int64_t dirX = leaving ? v.dx : -v.dx;
    const std::int64_t dirY = leaving ? v.dy : -v.dy;
    const bool diagonal = v.dx != 0 && v.dy != 0;
    const auto length =
        diagonal ? tuning_.approachDiagonal : tuning_.approachAxial;
    // Twice the band's half width: two clearances across, not one.
    const auto half =
        2 * static_cast<std::int64_t>(diagonal ? tuning_.approachHalfDiagonal
                                               : tuning_.approachHalfAxial);
    const std::int64_t perpX = -dirY;
    const std::int64_t perpY = dirX;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto strip = [&](const std::int64_t cx, const std::int64_t cy) {
      for (std::int64_t t = -half; t <= half; ++t) {
        const auto x = cx + (t * perpX);
        const auto y = cy + (t * perpY);
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        auto& cell = proximity_[static_cast<std::size_t>((y * width) + x)];
        cell = std::max(cell, price);
      }
    };
    std::int64_t prevY = end.y;
    for (std::uint32_t k = 0; k <= length; ++k) {
      const auto cx = static_cast<std::int64_t>(end.x) + (dirX * k);
      const auto cy = static_cast<std::int64_t>(end.y) + (dirY * k);
      // A diagonal run touches its last cell only at a corner; the joining
      // cell closes the gap, as the band's stamp closes its own.
      if (diagonal && k > 0) {
        strip(cx, prevY);
      }
      strip(cx, cy);
      prevY = cy;
    }
  }

  /// The three ways the lane polygon is built from.
  struct Lane {
    Path before;
    Path after;
    Path current;
    /// Which of the prototype's rules fired, for the log.
    std::string rule;
  };

  /// Whether a feedline edge has a launcher at its source, or at its target.
  /// The prototype's `first_feedlines` and `last_feedlines`.
  [[nodiscard]] bool startsAtLauncher(const Wire& wire) const {
    if (!wire.feedline || wire.edge >= edges_.size()) {
      return false;
    }
    const auto& edge = edges_[wire.edge];
    return edge.chain < chains_.size() &&
           edge.from < chains_[edge.chain].size() &&
           chains_[edge.chain][edge.from].fixed;
  }
  [[nodiscard]] bool endsAtLauncher(const Wire& wire) const {
    if (!wire.feedline || wire.edge >= edges_.size()) {
      return false;
    }
    const auto& edge = edges_[wire.edge];
    return edge.chain < chains_.size() && edge.to < chains_[edge.chain].size() &&
           chains_[edge.chain][edge.to].fixed;
  }

  /// Keep the stretch of a way that runs alongside another: from the cell
  /// nearest that one's first to the cell nearest its last, ordered so that
  /// the kept stretch runs the same way round.
  static void trimAlong(Path& way, const Path& current) {
    if (way.size() < 2 || current.empty()) {
      return;
    }
    const auto nearest = [&way](const PathPoint& to) {
      std::size_t best = 0;
      auto least = std::numeric_limits<std::uint32_t>::max();
      for (std::size_t at = 0; at < way.size(); ++at) {
        const auto span = static_cast<std::uint32_t>(
            std::sqrt(std::pow(static_cast<double>(way[at].x) - to.x, 2) +
                      std::pow(static_cast<double>(way[at].y) - to.y, 2)));
        if (span < least) {
          least = span;
          best = at;
        }
      }
      return best;
    };
    const auto head = nearest(current.front());
    const auto tail = nearest(current.back());
    const auto lo = std::min(head, tail);
    const auto hi = std::max(head, tail);
    if (hi <= lo) {
      return;
    }
    Path kept(way.begin() + static_cast<std::ptrdiff_t>(lo),
              way.begin() + static_cast<std::ptrdiff_t>(hi) + 1);
    if (head > tail) {
      std::ranges::reverse(kept);
    }
    way = std::move(kept);
  }

  /// Cut a way after the cell of it that lies nearest a point.
  static void cutAfterNearest(Path& way, const PathPoint& to) {
    if (way.empty()) {
      return;
    }
    // The prototype truncates: `uint32_t dist = std::sqrt(...)`
    // (`FinalGrid.cpp:4484`). Whole cells, and the first of a tie wins,
    // which is not the same index a real-valued distance picks.
    std::size_t best = 0;
    std::uint32_t least = std::numeric_limits<std::uint32_t>::max();
    for (std::size_t at = 0; at < way.size(); ++at) {
      const auto span = static_cast<std::uint32_t>(
          std::sqrt(std::pow(static_cast<double>(way[at].x) - to.x, 2) +
                    std::pow(static_cast<double>(way[at].y) - to.y, 2)));
      if (span < least) {
        least = span;
        best = at;
      }
    }
    way.erase(way.begin() + static_cast<std::ptrdiff_t>(best) + 1, way.end());
  }

  /// What the lane polygon is drawn between, as the prototype draws it
  /// (`FinalGrid.cpp:4437`-`:4592`).
  ///
  /// The polygon of the outer routing runs from the wire's first cell along
  /// its predecessor, to its last cell, and back along its successor. That
  /// is the room the wire has — and it is the right room only while the two
  /// wires beside it are conventional wires of the ring, which is what the
  /// outer routing's members always are.
  ///
  /// In the feedline pass they need not be. A feedline edge runs *across*
  /// the ring rather than beside it, so a polygon closed on one bounds
  /// nothing; and a resonator beside a resonator is not a wall either,
  /// because it is the thing the wire is trying to get past. Either way the
  /// fill then frees a region that is not the wire's room, and the search is
  /// steered into somebody else's.
  ///
  /// So the walk goes outward until it finds a way that does bound the room:
  /// past a feedline to the wire beyond it, past a run of resonators to the
  /// first conventional wire after them — that one cut at the cell nearest
  /// the resonator's own source, because only the part of it that runs
  /// alongside is a wall.
  [[nodiscard]] Lane laneOf(const std::vector<Wire>& wires,
                            const std::vector<std::uint32_t>& members,
                            const std::uint32_t slot) const {
    const auto total = static_cast<std::int64_t>(members.size());
    const auto step = [&](const std::int64_t k) -> const Wire& {
      const auto at =
          ((static_cast<std::int64_t>(slot) + k) % total + total) % total;
      return wires[members[static_cast<std::size_t>(at)]];
    };
    Lane lane{
        .before = step(-1).way, .after = step(1).way, .current = step(0).way};
    const auto& wire = step(0);

    // A feedline edge of its own. A terminal edge takes the way beyond the
    // neighbour it would be bounded by, and a last edge is run the other way
    // round so that the polygon closes the way it was drawn.
    if (wire.feedline) {
      if (startsAtLauncher(wire)) {
        lane.after = step(2).way;
        lane.rule += "first feedline: after <- +2; ";
      }
      if (endsAtLauncher(wire)) {
        lane.before = step(-2).way;
        std::ranges::reverse(lane.current);
        lane.rule += "last feedline: before <- -2, own way reversed; ";
      }
      return lane;
    }

    // The successor.
    const auto& next = step(1);
    if (next.feedline) {
      if (wire.resonator) {
        lane.after = step(2).way;
        lane.rule += "after is a feedline, this a resonator: after <- +2; ";
      } else {
        const auto& beyond = step(2).way;
        lane.after.insert(lane.after.end(), beyond.begin(), beyond.end());
        lane.rule += "after is a feedline, this conventional: after += +2; ";
      }
    } else if (next.resonator) {
      // The prototype walks on from `i + 2` while it stands on a resonator
      // and stops on the first that is not one — or on the wire itself,
      // having gone the whole way round, and it uses that one too
      // (`FinalGrid.cpp:4465`). No guard, so none here.
      std::int64_t k = 2;
      while (k < total && step(k).resonator) {
        ++k;
      }
      auto beyond = step(k).way;
      if (endsAtLauncher(step(k))) {
        std::ranges::reverse(beyond);
      } else {
        cutAfterNearest(beyond, next.objective.source);
      }
      lane.after.insert(lane.after.begin(), beyond.begin(), beyond.end());
      lane.rule += std::format("after is a resonator: +{} prepended{}; ", k,
                               endsAtLauncher(step(k)) ? " reversed" : " cut");
    }

    // The predecessor, the same the other way round.
    const auto& prev = step(-1);
    if (prev.feedline) {
      if (wire.resonator) {
        lane.before = step(-2).way;
        lane.rule += "before is a feedline, this a resonator: before <- -2; ";
      } else {
        std::ranges::reverse(lane.before);
        const auto& beyond = step(-2).way;
        lane.before.insert(lane.before.end(), beyond.begin(), beyond.end());
        lane.rule += "before is a feedline, this conventional: reversed, += -2; ";
      }
    } else if (prev.resonator) {
      std::int64_t k = -2;
      while (k > -total && step(k).resonator) {
        --k;
      }
      // The before side never reverses, whatever it stops on: the
      // prototype's `last_feedlines` test is on the after side only
      // (`FinalGrid.cpp:4497` against `:4568`).
      auto beyond = step(k).way;
      cutAfterNearest(beyond, prev.objective.source);
      lane.before.insert(lane.before.begin(), beyond.begin(), beyond.end());
      lane.rule += std::format("before is a resonator: {} prepended cut; ", k);
    }

    // The case the prototype has no rule for: a resonator with a plain wire
    // on either side. Neither branch above fired, so both neighbours are
    // still whole, and the polygon would close over the two long jumps from
    // the coupler and from the qubit. See `trimTheLane`.
    if (trimTheLane() && wire.resonator && !next.feedline && !next.resonator &&
        !prev.feedline && !prev.resonator) {
      const auto was = lane.before.size() + lane.after.size();
      trimAlong(lane.before, lane.current);
      trimAlong(lane.after, lane.current);
      lane.rule += std::format(
          "resonator between two plain wires: both cut to its own span, "
          "{} -> {} cells; ",
          was, lane.before.size() + lane.after.size());
    }
    return lane;
  }

  /// Clear the price inside the lane, by a scanline fill of its polygon.
  /// `outside` is the price the ground takes, `inside` the price the lane
  /// keeps. The ground is painted **here**, not by the caller, and only once
  /// the polygon is known to be one: the prototype's
  /// `compute_corridor_polygon_proximity` returns before its own
  /// `std::fill` on an empty way or a degenerate polygon
  /// (`FinalGrid.cpp:15756`, `:15773`), which leaves the grid as the caller
  /// zeroed it. Painting first and freeing after inverts that case — every
  /// cell priced where the prototype prices none.
  void fillLane(const Path& current, const Path& before, const Path& after,
                const std::uint16_t outside, const std::uint16_t inside) {
    if (current.empty() || outside == 0) {
      return;
    }
    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(before.size() + after.size() + 2);
    const auto add = [&polygon](const PathPoint& point) {
      polygon.emplace_back(static_cast<double>(point.x) + 0.5,
                           static_cast<double>(point.y) + 0.5);
    };
    add(current.front());
    for (const auto& point : before) {
      add(point);
    }
    add(current.back());
    for (const auto& point : std::ranges::reverse_view(after)) {
      add(point);
    }
    if (polygon.size() < 3) {
      return;
    }
    std::ranges::fill(proximity_, outside);

    double minY = polygon.front().second;
    double maxY = minY;
    for (const auto& [x, y] : polygon) {
      minY = std::min(minY, y);
      maxY = std::max(maxY, y);
    }
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto firstRow =
        std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(minY)));
    const auto lastRow = std::min<std::int64_t>(
        height - 1, static_cast<std::int64_t>(std::ceil(maxY)));

    std::vector<double> crossings;
    for (std::int64_t y = firstRow; y <= lastRow; ++y) {
      const auto scan = static_cast<double>(y) + 0.5;
      crossings.clear();
      for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size();
           j = i++) {
        const auto [xi, yi] = polygon[i];
        const auto [xj, yj] = polygon[j];
        if ((yi > scan) != (yj > scan)) {
          crossings.push_back(xi + ((scan - yi) * (xj - xi) / (yj - yi)));
        }
      }
      std::ranges::sort(crossings);
      for (std::size_t at = 0; at + 1 < crossings.size(); at += 2) {
        const auto from = std::max<std::int64_t>(
            0, static_cast<std::int64_t>(std::ceil(crossings[at] - 0.5)));
        const auto to = std::min<std::int64_t>(
            width - 1,
            static_cast<std::int64_t>(std::floor(crossings[at + 1] - 0.5)));
        for (std::int64_t x = from; x <= to; ++x) {
          proximity_[static_cast<std::size_t>((y * width) + x)] = inside;
        }
      }
    }
  }

  /// Price a disc of a radius around every cell of a way: added to what the
  /// cell already holds, or set outright where `addTheprices` is off. See it
  /// for why adding is the default.
  void stampDisc(const Path& way, const std::uint32_t radius,
                 const std::uint16_t price) {
    if (way.empty() || radius == 0 || price == 0) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const bool add = addTheprices();
    alongDisc(way, stencilFor(radius),
              [&](const std::int64_t x, const std::int64_t y) {
                auto& cell = proximity_[static_cast<std::size_t>((y * width) + x)];
                cell = add ? static_cast<std::uint16_t>(std::min<std::uint32_t>(
                                 CEILING, static_cast<std::uint32_t>(cell) + price))
                           : price;
              });
  }

  /// A halo around a way: the full price on it, falling to nothing at the
  /// reach. See `haloReach` for why the sweep wants this and not a disc.
  ///
  /// Laid as nested rings from the rim inwards, each adding only what it is
  /// dearer than the ring outside it. The sum telescopes, so a cell ends on
  /// the ground it had plus the price of the ring it really sits in, and no
  /// cell is counted twice however many rings contain it.
  void stampHalo(const Path& way, const std::uint32_t reach,
                 const std::uint16_t peak) {
    if (way.empty() || reach == 0 || peak == 0) {
      return;
    }
    if (haloDecay() == 0) {
      stampDisc(way, reach, peak);
      return;
    }
    const auto priceAt = [&](const std::uint32_t ring) -> double {
      if (ring == 0 || ring > reach) {
        return 0.0;
      }
      const auto at = static_cast<double>(ring);
      const auto far = static_cast<double>(reach);
      const auto share = haloDecay() == 2 ? std::exp(-(at - 1.0) / (far / 3.0))
                                          : 1.0 - ((at - 1.0) / far);
      return static_cast<double>(peak) * std::max(0.0, share);
    };
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const bool add = addTheprices();
    for (std::uint32_t ring = reach; ring >= 1; --ring) {
      const auto step = static_cast<std::int32_t>(
          std::lround(priceAt(ring) - priceAt(ring + 1)));
      if (step > 0) {
        alongDisc(way, stencilFor(ring),
                  [&](const std::int64_t x, const std::int64_t y) {
                    auto& cell =
                        proximity_[static_cast<std::size_t>((y * width) + x)];
                    const auto was = static_cast<std::int32_t>(cell);
                    cell = static_cast<std::uint16_t>(std::clamp(
                        add ? was + step : std::max(was, step), 0,
                        static_cast<std::int32_t>(CEILING)));
                  });
      }
      if (ring == 1) {
        break;
      }
    }
  }

  [[nodiscard]] const Stencil& stencilFor(const std::uint32_t radius) {
    const auto found = stencils_.find(radius);
    if (found != stencils_.end()) {
      return found->second;
    }
    return stencils_.emplace(radius, stencilOf(radius)).first->second;
  }

  /// The price that pulls a wire into the middle of the room it has.
  ///
  /// Two chamfer passes give every free cell of the corridor its distance to
  /// the nearest wall; the cells that are a local maximum of that distance are
  /// the middle of the channel, and two more passes give every cell its
  /// distance to the middle. What a cell costs is then the share of the way
  /// from the middle to the wall it stands at, so the middle is free and the
  /// wall is the full price. This is the prototype's
  /// `compute_corridor_proximity_decay`, which is what its refinement steers
  /// by.
  void priceRoom() {
    const auto price = tuning_.wireProximityPenalty;
    std::ranges::fill(proximity_, price);
    if (price == 0) {
      return;
    }
    if (box_.empty()) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    constexpr std::int32_t FAR = 1000000;
    wall_.assign(scene_.router.cells(), 0);
    middle_.assign(scene_.router.cells(), FAR);

    const auto lowX = box_.minX;
    const auto lowY = box_.minY;
    const auto highX = box_.maxX;
    const auto highY = box_.maxY;
    for (std::int64_t y = lowY; y <= highY; ++y) {
      for (std::int64_t x = lowX; x <= highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        const bool border = y == lowY || x == lowX || y == highY || x == highX;
        wall_[cell] = (corridor_.test(cell) || border) ? 0 : FAR;
      }
    }
    const auto step = static_cast<std::size_t>(width);
    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        wall_[cell] = std::min(
            {wall_[cell], wall_[cell - step - 1] + 1, wall_[cell - step] + 1,
             wall_[cell - step + 1] + 1, wall_[cell - 1] + 1});
      }
    }
    for (std::int64_t y = highY - 1; y > lowY; --y) {
      for (std::int64_t x = highX - 1; x > lowX; --x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        wall_[cell] = std::min(
            {wall_[cell], wall_[cell + 1] + 1, wall_[cell + step - 1] + 1,
             wall_[cell + step] + 1, wall_[cell + step + 1] + 1});
      }
    }

    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        bool peak = true;
        for (std::int64_t dy = -1; dy <= 1 && peak; ++dy) {
          for (std::int64_t dx = -1; dx <= 1; ++dx) {
            const auto other = static_cast<std::size_t>(
                static_cast<std::int64_t>(cell) + (dy * width) + dx);
            if (wall_[other] > wall_[cell]) {
              peak = false;
              break;
            }
          }
        }
        if (peak) {
          middle_[cell] = 0;
        }
      }
    }
    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        middle_[cell] =
            std::min({middle_[cell], middle_[cell - step - 1] + 1,
                      middle_[cell - step] + 1, middle_[cell - step + 1] + 1,
                      middle_[cell - 1] + 1});
      }
    }
    for (std::int64_t y = highY - 1; y > lowY; --y) {
      for (std::int64_t x = highX - 1; x > lowX; --x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        middle_[cell] = std::min(
            {middle_[cell], middle_[cell + 1] + 1, middle_[cell + step - 1] + 1,
             middle_[cell + step] + 1, middle_[cell + step + 1] + 1});
      }
    }

    for (std::size_t cell = 0; cell < proximity_.size(); ++cell) {
      const auto dw = wall_[cell];
      const auto dc = middle_[cell];
      if (dw == 0 || dc >= FAR) {
        proximity_[cell] = price;
        continue;
      }
      proximity_[cell] = static_cast<std::uint8_t>(
          ((dc * price) + ((dc + dw) / 2)) / (dc + dw));
    }
  }

  /// The box a band can reach, so that nothing sweeps the whole grid.
  struct Box {
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = -1;
    std::int64_t maxY = -1;
    [[nodiscard]] bool empty() const { return maxX < minX || maxY < minY; }
  };

  [[nodiscard]] Box boxOf(const Wire& wire, const std::uint32_t reach) const {
    Box box{.minX = std::numeric_limits<std::int64_t>::max(),
            .minY = std::numeric_limits<std::int64_t>::max(),
            .maxX = std::numeric_limits<std::int64_t>::min(),
            .maxY = std::numeric_limits<std::int64_t>::min()};
    const auto stretch = [&box](const PathPoint& point) {
      box.minX = std::min(box.minX, static_cast<std::int64_t>(point.x));
      box.maxX = std::max(box.maxX, static_cast<std::int64_t>(point.x));
      box.minY = std::min(box.minY, static_cast<std::int64_t>(point.y));
      box.maxY = std::max(box.maxY, static_cast<std::int64_t>(point.y));
    };
    for (const auto& point : wire.way) {
      stretch(point);
    }
    stretch(wire.objective.source);
    stretch(wire.objective.target);
    for (const auto& point : wire.fixed) {
      stretch(point);
    }
    const auto grown = static_cast<std::int64_t>(reach);
    box.minX = std::max<std::int64_t>(0, box.minX - grown);
    box.minY = std::max<std::int64_t>(0, box.minY - grown);
    box.maxX =
        std::min<std::int64_t>(scene_.router.width - 1, box.maxX + grown);
    box.maxY =
        std::min<std::int64_t>(scene_.router.height - 1, box.maxY + grown);
    return box;
  }

  // --- The debug pictures --------------------------------------------------

  /// The font and the markers of a picture, from how wide it is, so that a
  /// picture of a whole chip and one of a single band both read.
  struct Scale {
    double font = 8.0;
    double marker = 3.0;
  };
  [[nodiscard]] static Scale scaleOf(const debug::View& view) {
    const auto width = static_cast<double>(view.width());
    return {.font = std::clamp(width / 70.0, 4.0, 28.0),
            .marker = std::clamp(width / 160.0, 1.5, 8.0)};
  }

  /// The room a legend of so many lines needs above the view.
  [[nodiscard]] static double headroomFor(const Scale& scale,
                                          const std::size_t lines) {
    return scale.font * ((static_cast<double>(lines) * 1.4) + 1.6);
  }

  /// The longest line a legend will hold, in characters, so that its box is
  /// wide enough for the text rather than a fixed sixty ems that the longer
  /// entries ran out of.
  [[nodiscard]] static std::size_t
  widest(const std::vector<std::string>& title,
         const std::vector<std::pair<std::string, std::string>>& entries) {
    std::size_t most = 0;
    for (const auto& line : title) {
      most = std::max(most, line.size());
    }
    for (const auto& [cls, what] : entries) {
      most = std::max(most, what.size() + 3);
    }
    return most;
  }

  /// The two ends of a wire: a marker each, the heading it leaves or arrives
  /// on as a line the length of the clearance, and its name.
  void drawEnds(debug::Svg& svg, const Wire& wire, const Scale& scale) const {
    const auto reach = static_cast<double>(tuning_.clearance);
    const auto& source = wire.objective.source;
    const auto& target = wire.objective.target;
    const auto leaving = routing::headingVector(source.heading);
    const auto arriving = routing::headingVector(target.heading);
    svg.line(svg.centreX(source.x), svg.centreY(source.y),
             svg.centreX(source.x) + (leaving.dx * reach),
             svg.centreY(source.y) - (leaving.dy * reach), "ar");
    svg.line(svg.centreX(target.x) - (arriving.dx * reach),
             svg.centreY(target.y) + (arriving.dy * reach),
             svg.centreX(target.x), svg.centreY(target.y), "ar");
    // Shape, not only colour: the source is a circle and the target a
    // square, so the two are told apart with no hue at all.
    svg.circle(source.x, source.y, scale.marker, "src");
    svg.square(target.x, target.y, scale.marker, "tgt", "target");
    const auto id = wireId(wire);
    svg.text(svg.centreX(source.x) + scale.marker,
             svg.centreY(source.y) - scale.marker, id, "lb", scale.font);
    svg.text(svg.centreX(target.x) + scale.marker,
             svg.centreY(target.y) - scale.marker, id, "lb", scale.font);
  }

  /// A legend in the headroom above the view, so that it covers no cell: a
  /// title, then one line per thing drawn, with the swatch it is drawn in.
  void drawLegend(
      debug::Svg& svg, const Scale& scale,
      const std::vector<std::string>& title,
      const std::vector<std::pair<std::string, std::string>>& entries) const {
    const auto& view = svg.view();
    const auto rows = static_cast<double>(title.size() + entries.size());
    // The legend is set in whatever size keeps it inside the picture. At the
    // chip's own font the longest entry runs past the right edge of a small
    // grid — on 4q the box came out 740 units wide against a picture of 696
    // — and the text was simply cut off there.
    const auto room = static_cast<double>(view.width()) - (2.0 * scale.font);
    const auto chars = 0.62 * static_cast<double>(widest(title, entries));
    const auto font =
        std::max(0.25 * scale.font,
                 std::min(scale.font, chars > 0.0 ? room / chars : scale.font));
    const auto x = debug::Svg::left(view.minX) + font;
    const auto tall = font * ((rows * 1.4) + 0.6);
    // Hung from the top edge of the view rather than stood on the ceiling:
    // the headroom a caller reserves is a guess, and a legend that outgrows
    // it used to come down over the chip. Growing upward instead, it leaves
    // the picture rather than covering it, and a legend cut off at the top
    // says plainly that the reservation was too small.
    auto y = svg.top(view.maxY) - tall;
    const auto wide = std::max(font * 24.0, chars * font);
    svg.box(x - (0.5 * font), y - (0.5 * font), wide, tall, "lg");
    for (const auto& line : title) {
      svg.text(x, y + font, line, "lb", font);
      y += 1.4 * font;
    }
    for (const auto& [cls, what] : entries) {
      // A frame under every swatch. Four of the things drawn are translucent
      // fills at a tenth of an opacity, and a swatch of one on white is not
      // visibly anything; against the frame it reads as a tint.
      svg.box(x, y + (0.15 * font), font, font, "lgs");
      svg.box(x, y + (0.15 * font), font, font, cls);
      svg.text(x + (1.6 * font), y + font, what, "lb", font);
      y += 1.4 * font;
    }
  }

  /// Every port whose cell lies in the view, as the chip carries it: the
  /// square on the port's cell, a dot at its exact position, the line along
  /// its orientation, the index beside it and the label as a tooltip.
  void drawPorts(debug::Svg& svg, const Scale& scale) const {
    const auto& view = svg.view();
    const auto reach = static_cast<double>(tuning_.clearance);
    for (const auto& port : ports_) {
      if (port.x < view.minX || port.x > view.maxX || port.y < view.minY ||
          port.y > view.maxY) {
        continue;
      }
      svg.square(port.x, port.y, scale.marker, "prt",
                 std::format("port {} · {} · exactly at cell ({:.2f}, {:.2f})",
                             port.index, port.label, port.fx, port.fy));
      svg.dot(debug::Svg::atX(port.fx), svg.atY(port.fy), 0.6 * scale.marker,
              "prx");
      if (port.step.x != 0 || port.step.y != 0) {
        svg.line(debug::Svg::atX(port.fx), svg.atY(port.fy),
                 debug::Svg::atX(port.fx) + (port.step.x * reach),
                 svg.atY(port.fy) - (port.step.y * reach), "pra");
      }
      svg.text(svg.centreX(port.x) - scale.marker,
               svg.centreY(port.y) + (2.2 * scale.marker),
               std::format("p{}", port.index), "prl", 0.8 * scale.font);
    }
  }

  /// For every wire whose end lies in the view: a dashed line from the exact
  /// position of its port to the cell it is routed to, beyond the port's
  /// band, and the distance between the two in layout units and in cells.
  void drawPortDistances(debug::Svg& svg, const Scale& scale,
                         const std::vector<Wire>& wires) const {
    const auto& view = svg.view();
    const auto inside = [&view](const PathPoint& point) {
      return point.x >= view.minX && point.x <= view.maxX &&
             point.y >= view.minY && point.y <= view.maxY;
    };
    const auto annotate = [&](const PathPoint& end, const std::uint32_t port) {
      if (port >= ports_.size() || !inside(end)) {
        return;
      }
      const auto& mark = ports_[port];
      const auto dx = static_cast<double>(end.x) - mark.fx;
      const auto dy = static_cast<double>(end.y) - mark.fy;
      const auto cells = std::hypot(dx, dy);
      const auto units = std::hypot(dx * scene_.router.cellWidth,
                                    dy * scene_.router.cellHeight);
      const auto x0 = debug::Svg::atX(mark.fx);
      const auto y0 = svg.atY(mark.fy);
      const auto x1 = svg.centreX(end.x);
      const auto y1 = svg.centreY(end.y);
      svg.line(x0, y0, x1, y1, "dst");
      svg.text((0.5 * (x0 + x1)) + scale.marker,
               (0.5 * (y0 + y1)) - scale.marker,
               std::format("{:.1f} u · {:.1f} cells", units, cells), "prl",
               0.8 * scale.font);
    };
    for (const auto& wire : wires) {
      if (!wire.feasible) {
        continue;
      }
      annotate(wire.objective.target, wire.targetPort);
      if (wire.inner) {
        annotate(wire.objective.source, wire.sourcePort);
      }
    }
  }

  /// The obstacles and the price of running beside them, over a view.
  /// The rectangle the launcher stubs leave open, which is where a coupler
  /// may sit. Drawn on every search picture, because an edge that detours is
  /// usually an edge whose coupler sits outside it.
  void drawCouplerBox(debug::Svg& svg) const {
    if (couplerBox_.empty()) {
      return;
    }
    const auto x0 = debug::Svg::atX(static_cast<double>(couplerBox_.minX) - 0.5);
    const auto x1 = debug::Svg::atX(static_cast<double>(couplerBox_.maxX) + 0.5);
    const auto y0 = svg.atY(static_cast<double>(couplerBox_.maxY) + 0.5);
    const auto y1 = svg.atY(static_cast<double>(couplerBox_.minY) - 0.5);
    svg.box(x0, y0, x1 - x0, y1 - y0, "cbx");
  }

  void drawGround(debug::Svg& svg, const debug::View& view) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellOf = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    const auto& halo = router_.staticProximity();
    const int haloMost = std::max<int>(1, tuning_.staticProximityPenalty);
    static_cast<void>(view);
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return halo.empty() ? 0 : levelOf(halo[cellOf(x, y)], haloMost);
        },
        [](const int level) { return std::format("s{}", level); });
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return scene_.blocked.test(cellOf(x, y)) ? 1 : 0;
        },
        [](const int) { return std::string("ob"); });
  }

  /// A picture of one search: what it may enter, what fences it, what it
  /// pays, the wires around it, and what it found. Taken right after the
  /// search, so it shows exactly what the router was given; for a resonator
  /// the way carries its meander and the note says what the lengthening
  /// came to.
  void drawSearch(const Wire& wire, const Path& found,
                  const std::string& note) {
    if (!debug_ || !searchPictures() || box_.empty() ||
        frame_.wires == nullptr) {
      return;
    }
    const auto& wires = *frame_.wires;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellOf = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    const debug::View view{.minX = box_.minX,
                           .minY = box_.minY,
                           .maxX = box_.maxX,
                           .maxY = box_.maxY};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 21));
    svg.style(DEBUG_STYLE);

    // The ground: the halo, the artwork, what the search may enter, and
    // what it pays for entering it.
    drawGround(svg, view);
    drawCouplerBox(svg);
    {
      std::size_t shut = 0;
      std::size_t art = 0;
      for (std::int64_t y = box_.minY; y <= box_.maxY; ++y) {
        for (std::int64_t x = box_.minX; x <= box_.maxX; ++x) {
          const auto cell = cellOf(x, y);
          shut += corridor_.test(cell) ? 1 : 0;
          art += scene_.components.test(cell) ? 1 : 0;
        }
      }
      tell(std::format("picture {}: {} cells closed in the corridor, {} of "
                       "them artwork",
                       frame_.kind, shut, art));
    }
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return corridor_.test(cellOf(x, y)) ? 0 : 1;
        },
        [](const int) { return std::string("co"); });
    // The levels are scaled to the dearest cell in the box, so that a price
    // ten times the ordinary one is drawn ten times as dark and not the same.
    Shades prices;
    for (std::int64_t y = box_.minY; y <= box_.maxY; ++y) {
      for (std::int64_t x = box_.minX; x <= box_.maxX; ++x) {
        prices.gather(proximity_[cellOf(x, y)]);
      }
    }
    prices.settle();
    priceShades_ = prices.say();
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return prices.of(proximity_[cellOf(x, y)]);
        },
        [](const int level) { return std::format("p{}", level); });

    // The couplers and the feedlines as they stand right now, whatever the
    // search is. A picture of one edge used to show the ground and that
    // edge alone, so what the edge had to get past was not in it: the pads
    // it runs between and the chains already drawn. Both are drawn under
    // the wires of the search, so they never hide them.
    for (const auto& coupler : couplers_) {
      if (coupler.chosen >= coupler.options.size()) {
        continue;
      }
      const auto& option = coupler.options[coupler.chosen];
      const auto along = routing::headingVector(option.orientation);
      const auto across =
          routing::headingVector(routing::turned(option.orientation, -2));
      const std::int64_t halfRun = option.run / 2;
      const std::int64_t halfDepth = option.depth / 2;
      const auto cx = static_cast<std::int64_t>(option.centre.x);
      const auto cy = static_cast<std::int64_t>(option.centre.y);
      std::vector<debug::Cell> pad;
      for (const auto& [a, d] : {std::pair{-1, -1}, std::pair{1, -1},
                                 std::pair{1, 1}, std::pair{-1, 1}}) {
        pad.emplace_back(
            cx + (a * halfRun * along.dx) + (d * halfDepth * across.dx),
            cy + (a * halfRun * along.dy) + (d * halfDepth * across.dy));
      }
      svg.polygon(pad, "ocp");
      if (!option.arc.empty()) {
        auto lead = cellsOf(option.arc);
        lead.emplace_back(option.arcEnd.x, option.arcEnd.y);
        svg.polyline(lead, "ocl");
      }
    }
    // Over the chains and not over `edges_`: that list is filled at the
    // commit, so during the greedy — which is where most of these pictures
    // are taken — it is empty, and the feedlines went undrawn in every one
    // of them. `edgeWayOf` gives the drawn wire's way once there is one and
    // what the greedy last chose before that, so this holds in both.
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      for (std::size_t at = 0; at < chainEdgePaths_[chain].size(); ++at) {
        const auto key = edgeWireOf_[chain][at];
        if (key == wire.key) {
          continue;
        }
        const auto& way = edgeWayOf(wires, chain, at);
        if (way.empty()) {
          continue;
        }
        // A drawn edge and a path the greedy chose are not the same thing
        // and must not look the same. Once the commit has been past an edge
        // and it found nothing, what is left in `chainEdgePaths_` is a way
        // that will never be built — and drawn like the rest it shows a
        // chip that never existed. On 17q this put two feedlines crossing in
        // the picture of an edge whose own way crosses nothing: the crossing
        // was against the abandoned path of an edge that had already failed.
        const bool drawn = key != NO_OWNER && key < wires.size() &&
                           wires[key].drawn;
        svg.polyline(cellsOf(way), drawn ? "fw" : "fwg");
      }
    }

    // The wires around it: the lane between the ring neighbours, the
    // neighbours, the fence, the wires let go of, and its own way.
    const auto nameOf = [&wires](const std::uint32_t key) {
      return key < wires.size() ? wireId(wires[key]) : std::string("-");
    };
    const auto wayOf = [&wires](const std::uint32_t key) -> const Path& {
      static const Path none;
      return key < wires.size() ? wires[key].way : none;
    };
    // The lane polygon, exactly as `fillLane` was handed it: everything
    // outside it is what the search pays the wire price for, and everything
    // inside is free. Drawn from what the pricing recorded where there is
    // one, and from the two ring neighbours where there is not.
    if (!frame_.laneCurrent.empty()) {
      const auto& current = frame_.laneCurrent;
      const auto& before = frame_.laneBefore;
      const auto& after = frame_.laneAfter;
      std::vector<debug::Cell> lane;
      lane.emplace_back(current.front().x, current.front().y);
      for (const auto& point : before) {
        lane.emplace_back(point.x, point.y);
      }
      lane.emplace_back(current.back().x, current.back().y);
      for (const auto& point : std::ranges::reverse_view(after)) {
        lane.emplace_back(point.x, point.y);
      }
      // A hairline is invisible on a picture three thousand cells wide, and
      // this is the one outline that says what the whole price field means.
      svg.polygon(lane, "lane",
                  std::format("stroke-width=\"{:.1f}\"",
                              std::max(2.0, 0.9 * scale.marker)));
    }
    for (const auto key : {frame_.before, frame_.after}) {
      svg.polyline(cellsOf(wayOf(key)), "nb");
    }
    const auto band =
        std::format("stroke-width=\"{}\"", (2 * tuning_.clearance) + 1);
    std::string fenced;
    for (const auto key : frame_.fence) {
      svg.polyline(cellsOf(wayOf(key)), "fz", band);
      svg.polyline(cellsOf(wayOf(key)), "fw");
      fenced += (fenced.empty() ? "" : ",") + nameOf(key);
    }
    std::string ripped;
    for (const auto key : frame_.ripped) {
      svg.polyline(cellsOf(wayOf(key)), "rp");
      ripped += (ripped.empty() ? "" : ",") + nameOf(key);
    }
    svg.polyline(cellsOf(wire.way), "ow");
    svg.polyline(cellsOf(found), "fd");
    drawPorts(svg, scale);
    drawPortDistances(svg, scale, wires);
    drawEnds(svg, wire, scale);

    const auto result = found.empty()
                            ? std::string("no way")
                            : std::format("found {} cells", found.size());
    drawLegend(
        svg, scale,
        {std::format("{} · round {} {} · wire {} · {} · {}{}", frame_.pass,
                     frame_.round, frame_.forward ? "forward" : "backward",
                     wireId(wire), frame_.kind, result,
                     note.empty() ? "" : " · " + note),
         std::format("fence {} · ripped {} · lane between {} and {} · box x "
                     "{}..{} y {}..{}",
                     fenced.empty() ? "-" : fenced,
                     ripped.empty() ? "-" : ripped, nameOf(frame_.before),
                     nameOf(frame_.after), box_.minX, box_.maxX, box_.minY,
                     box_.maxY),
         std::format("clearance {} cells · band {} cells · wire price {} · "
                     "obstacle price {} over {} cells · bend {}",
                     tuning_.clearance, tuning_.reach,
                     tuning_.wireProximityPenalty,
                     tuning_.staticProximityPenalty, tuning_.obstacleReach,
                     tuning_.bendPenalty)},
        {{"ob", "artwork and keepout: closed"},
         {"s8", "obstacle halo: priced, darker is dearer"},
         {"co", "band: what the search may enter"},
         {"lane", "the lane: the polygon the price is measured against — "
                  "everything outside it is priced, everything inside is "
                  "free. Built from the ways on either side, which past a "
                  "feedline or a run of resonators are not the two "
                  "neighbours but what lies beyond them"},
         {"p8", std::format("wire price, light to dark over the values this "
                            "picture holds: {}. Outside the lane, around the "
                            "wires let go of, and around the feedlines — and "
                            "they add up where they meet",
                            priceShades_.empty() ? "none" : priceShades_)},
         {"fz", "fence: the clearance around the fence wires, closed"},
         {"nb", "ring neighbours"},
         {"rp", "let go of: drawn again afterwards; crossable unless it is "
                "the fence"},
         {"ow", "the way it had"},
         {"fd", "the way it found"},
         {"src", "source, line = heading it leaves on"},
         {"tgt", "target, line = heading it arrives on"},
         {"prt", "port as the chip carries it: square = its cell, dot = exact "
                 "position, dashed = distance to the target beyond its band"},
         {"cbx", "where a coupler may sit: what the launcher stubs leave open"},
         {"ocp", "the couplers as they stand: pad and resonator lead"},
         {"fw", "the feedlines as they stand: drawn"},
         {"fwg", "what the greedy chose for an edge that has no way: dashed, "
                 "and never built"}});

    std::string kind = frame_.kind;
    std::erase(kind, ' ');
    lastPicture_ =
        debug_(std::format("final-{:05}-{}-r{}{}-w{}-{}.svg", ++pictures_,
                           frame_.pass, frame_.round,
                           frame_.forward ? 'f' : 'b', wireId(wire), kind),
               svg.finish());
  }

public:
  /// A picture of the whole router grid before anything is drawn: the
  /// artwork, the price of running beside it, every port as the chip carries
  /// it, every wire's two ends with the heading it leaves and arrives on, and
  /// the way the Detail stage drew.
  void attachPorts(std::vector<PortMark> ports) { ports_ = std::move(ports); }

  /// One picture of every coupler option there is, for reading with the
  /// eye what the numbers only count: the outer wires as they stand, and on
  /// every coupler each option it may take — the pad, the resonator lead
  /// off it and the feedline port it offers. The option the coupler starts
  /// on is drawn over the others in a second colour, so that "is the right
  /// one there at all, and is it the one we begin with" is one glance.
  ///
  /// Drawn once, before the greedy runs.
  void drawCouplerOptions(const std::vector<Wire>& wires) {
    if (!debug_) {
      return;
    }
    const debug::View view{
        .minX = 0,
        .minY = 0,
        .maxX = static_cast<std::int64_t>(scene_.router.width) - 1,
        .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 10));
    svg.style(DEBUG_STYLE);
    drawGround(svg, view);
    drawCouplerBox(svg);

    // Every outer wire as it stands going in.
    for (const auto& wire : wires) {
      if (wire.feasible && !wire.inner && !wire.feedline && !wire.way.empty()) {
        svg.polyline(cellsOf(wire.way), "sd");
      }
    }

    // The pad of one option, as the four corners of its rectangle.
    const auto padOf = [this](const CouplerOption& option) {
      const auto along = routing::headingVector(option.orientation);
      const auto across =
          routing::headingVector(routing::turned(option.orientation, -2));
      const std::int64_t halfRun = option.run / 2;
      const std::int64_t halfDepth = option.depth / 2;
      const auto cx = static_cast<std::int64_t>(option.centre.x);
      const auto cy = static_cast<std::int64_t>(option.centre.y);
      std::vector<debug::Cell> corners;
      for (const auto& [a, d] : {std::pair{-1, -1}, std::pair{1, -1},
                                 std::pair{1, 1}, std::pair{-1, 1}}) {
        corners.emplace_back(
            cx + (a * halfRun * along.dx) + (d * halfDepth * across.dx),
            cy + (a * halfRun * along.dy) + (d * halfDepth * across.dy));
      }
      return corners;
    };

    std::uint32_t options = 0;
    for (const auto& coupler : couplers_) {
      for (std::size_t at = 0; at < coupler.options.size(); ++at) {
        const auto& option = coupler.options[at];
        const bool starts = at == coupler.chosen;
        ++options;
        svg.polygon(padOf(option), starts ? "ocp" : "opt");
        if (!option.arc.empty()) {
          auto lead = cellsOf(option.arc);
          lead.emplace_back(option.arcEnd.x, option.arcEnd.y);
          svg.polyline(lead, starts ? "ocl" : "opl");
        }
        svg.circle(option.out.x, option.out.y, starts ? 2.2 : 1.2,
                   starts ? "ocs" : "ops");
      }
    }

    drawPorts(svg, scale);
    drawLegend(
        svg, scale,
        {std::format("coupler options · {} couplers · {} options in all · "
                     "before the greedy",
                     couplers_.size(), options),
         std::format("pad {}x{} cells · lead {} straight · clearance {} cells",
                     tuning_.couplerLength, tuning_.couplerHeight,
                     leadStraight(), tuning_.clearance)},
        {{"ob", "artwork and keepout"},
         {"sd", "the outer wires as they stand"},
         {"cbx", "the box the launcher stubs leave open"},
         {"opt", "an option's pad"},
         {"opl", "that option's resonator lead, port to insertion point"},
         {"ops", "that option's feedline port"},
         {"ocp", "the option the coupler starts on: pad, lead and port"},
         {"prt", "port as the chip carries it"}});
    const auto where = debug_("final-coupler-options.svg", svg.finish());
    if (!where.empty()) {
      say(std::format("the coupler options: {}", where));
    }
  }

  /// The obstacles of `wallsAfterInsertion` in a picture: the border, the
  /// artwork, the pads, the walls of the port runs and the inflation of the
  /// feedlines as areas, and the lines of the port runs and feedlines on
  /// top.
  void drawWalls(debug::Svg& svg, const Walls& walls) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          const auto cell = static_cast<std::size_t>((y * width) + x);
          if (!walls.mask.test(cell)) {
            return 0;
          }
          switch (walls.list[walls.of[cell]].kind) {
          case WallKind::Border:
            return 1;
          case WallKind::Artwork:
            return 2;
          case WallKind::Coupler:
            return 3;
          case WallKind::Stub:
            return 4;
          case WallKind::Feedline:
            return 5;
          }
          return 0;
        },
        [](const int value) {
          static constexpr std::array<std::string_view, 5> CLASSES = {
              "wb", "ob", "wc", "wsz", "wfz"};
          return std::string(CLASSES[static_cast<std::size_t>(value - 1)]);
        });
    for (const auto& [wall, points] : walls.strokes) {
      svg.polyline(points,
                   walls.list[wall].kind == WallKind::Feedline ? "wfl" : "wst");
    }
  }

  /// The capacity graph of `reportCapacityGraph` as data, for the page that
  /// `plan --debug` builds from it (`final-capacity-graph.html`). Cells are
  /// grid cells, y up. It holds
  ///
  /// - the chambers and the walls as one raster, run-length coded row by
  ///   row: 0 a free cell in no chamber (the line of a bottleneck), 1 to 5
  ///   a wall by its kind (`wallKinds`), 6 + c a cell of chamber c;
  /// - the lines of the port runs and the feedline edges (`strokes`);
  /// - a node per chamber, and every edge with the chambers it joins, what
  ///   it takes, the wires the flow sends through it and its overflow; a
  ///   bottleneck also with its two ends and the wires its line crosses
  ///   now, a stretch with its feedline and the wires it is open to;
  /// - the cells of the slots between the walls of the port runs, where the
  ///   terminals lie;
  /// - the bottlenecks with one chamber on both sides (`aside`);
  /// - every outer wire with its source and its target port, the cells its
  ///   ends leave from, the chambers they lie in, its way through the graph
  ///   as edge indices and the way it has now.
  void writeCapacityGraph(const std::vector<Wire>& wires, const Walls& walls,
                          const std::vector<grid::Bottleneck>& gates,
                          const std::vector<MeasuredBottleneck>& measured,
                          const grid::Chambers& chambers,
                          const std::vector<GraphEdge>& graph,
                          const std::vector<GraphWire>& outer,
                          const FlowCheck& check,
                          const std::string_view verdict,
                          const std::vector<std::string>& summary) {
    if (!debug_) {
      return;
    }
    using nlohmann::json;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellAt = [width](const std::size_t cell) {
      return json::array({static_cast<std::int64_t>(cell) % width,
                          static_cast<std::int64_t>(cell) / width});
    };
    const auto names = [&wires](const std::vector<std::uint32_t>& keys) {
      auto list = json::array();
      for (const auto key : keys) {
        list.push_back(wireId(wires[key]));
      }
      return list;
    };
    // A way as the cells it turns at, its two ends included.
    const auto corners = [](const Path& way, const std::size_t first,
                            const std::size_t last) {
      auto list = json::array();
      for (auto at = first; at <= last && at < way.size(); ++at) {
        if (at != first && at != last) {
          const auto& before = way[at - 1];
          const auto& here = way[at];
          const auto& after = way[at + 1];
          const auto inX = static_cast<std::int64_t>(here.x) - before.x;
          const auto inY = static_cast<std::int64_t>(here.y) - before.y;
          const auto outX = static_cast<std::int64_t>(after.x) - here.x;
          const auto outY = static_cast<std::int64_t>(after.y) - here.y;
          if (inX == outX && inY == outY) {
            continue;
          }
        }
        list.push_back(json::array({way[at].x, way[at].y}));
      }
      return list;
    };

    auto raster = json::array();
    {
      int previous = -1;
      std::size_t run = 0;
      for (std::size_t cell = 0; cell < chambers.of.size(); ++cell) {
        int code = 0;
        if (walls.mask.test(cell)) {
          code = walls.of[cell] < walls.list.size()
                     ? 1 + static_cast<int>(walls.list[walls.of[cell]].kind)
                     : 1 + static_cast<int>(WallKind::Artwork);
        } else if (chambers.of[cell] != grid::NO_CHAMBER) {
          code = 6 + static_cast<int>(chambers.of[cell]);
        }
        if (code != previous && run > 0) {
          raster.push_back(previous);
          raster.push_back(run);
          run = 0;
        }
        previous = code;
        ++run;
      }
      if (run > 0) {
        raster.push_back(previous);
        raster.push_back(run);
      }
    }
    auto wallKinds = json::array();
    for (const auto kind :
         {WallKind::Border, WallKind::Artwork, WallKind::Coupler,
          WallKind::Stub, WallKind::Feedline}) {
      wallKinds.push_back(kindName(kind));
    }
    auto strokes = json::array();
    for (const auto& [wall, points] : walls.strokes) {
      auto line = json::array();
      for (const auto& [x, y] : points) {
        line.push_back(json::array({x, y}));
      }
      strokes.push_back({{"name", walls.list[wall].name},
                         {"kind", kindName(walls.list[wall].kind)},
                         {"points", std::move(line)}});
    }

    auto slotCells = json::array();
    for (const auto cell : walls.slots) {
      slotCells.push_back(cellAt(cell));
    }

    std::vector<std::size_t> size(chambers.count, 0);
    for (const auto chamber : chambers.of) {
      if (chamber != grid::NO_CHAMBER) {
        ++size[chamber];
      }
    }
    const auto node = chamberNodes(chambers);
    auto nodes = json::array();
    for (std::uint32_t chamber = 0; chamber < chambers.count; ++chamber) {
      nodes.push_back(
          {{"chamber", chamber},
           {"at", json::array({node[chamber].first, node[chamber].second})},
           {"cells", size[chamber]}});
    }

    auto edges = json::array();
    for (std::size_t index = 0; index < graph.size(); ++index) {
      const auto& edge = graph[index];
      json one{{"index", index},
               {"kind", edge.crossing ? "stretch" : "bottleneck"},
               {"chambers", edge.chambers},
               {"capacity", edge.capacity},
               {"load", edge.through.size()},
               {"overflow",
                index < check.overflow.size() ? check.overflow[index] : 0U},
               {"length", edge.length},
               {"through", names(edge.through)},
               {"users", edge.users.has_value() ? names(*edge.users) : json()}};
      if (edge.crossing) {
        one["name"] = std::format("× {} cells {}..{}", wireId(wires[edge.id]),
                                  edge.first, edge.last);
        one["feedline"] = wireId(wires[edge.id]);
        one["line"] = corners(wires[edge.id].way, edge.first, edge.last);
      } else {
        const auto& gate = gates[edge.id];
        one["name"] = std::format("g{}", edge.id);
        one["ends"] = json::array(
            {endName(walls, gate.first), endName(walls, gate.second)});
        one["line"] = json::array({cellAt(gate.first), cellAt(gate.second)});
        one["crossingNow"] = names(measured[edge.id].crossing);
      }
      edges.push_back(std::move(one));
    }
    auto aside = json::array();
    for (std::size_t gate = 0; gate < gates.size(); ++gate) {
      if (chambers.beside[gate].size() >= 2) {
        continue;
      }
      aside.push_back(
          {{"name", std::format("g{}", gate)},
           {"chambers", chambers.beside[gate]},
           {"ends", json::array({endName(walls, gates[gate].first),
                                 endName(walls, gates[gate].second)})},
           {"line", json::array({cellAt(gates[gate].first),
                                 cellAt(gates[gate].second)})},
           {"length", measured[gate].cells},
           {"capacity", measured[gate].holds}});
    }

    // The launcher a plain wire starts at, which its wire does not name:
    // the launcher port nearest the start of its way.
    const auto launcherAt = [this](const PathPoint& end) -> std::uint32_t {
      std::uint32_t nearest = NO_OWNER;
      std::int64_t best = std::numeric_limits<std::int64_t>::max();
      for (const auto& port : ports_) {
        if (!scene_.launcherCell.contains(port.index)) {
          continue;
        }
        const auto dx = port.x - static_cast<std::int64_t>(end.x);
        const auto dy = port.y - static_cast<std::int64_t>(end.y);
        if ((dx * dx) + (dy * dy) < best) {
          best = (dx * dx) + (dy * dy);
          nearest = port.index;
        }
      }
      return nearest;
    };
    // A port: what the chip calls it, where it is, the way a wire leaves it
    // or arrives at it, and the cell the graph places it by.
    const auto portOf = [&](const std::uint32_t index, const PathPoint& end,
                            const PathPoint& leaves,
                            const std::vector<std::uint32_t>& in,
                            const std::string_view otherwise) {
      const auto v = routing::headingVector(end.heading);
      json port{{"at", json::array({end.x, end.y})},
                {"heading", json::array({v.dx, v.dy})},
                {"cell", json::array({leaves.x, leaves.y})},
                {"chambers", in}};
      if (index < ports_.size()) {
        port["label"] = ports_[index].label;
        port["port"] = json::array({ports_[index].fx, ports_[index].fy});
      } else {
        port["label"] = otherwise;
        port["port"] = json::array({end.x, end.y});
      }
      return port;
    };
    auto outerWires = json::array();
    for (const auto& one : outer) {
      const auto& wire = wires[one.wire];
      json entry{
          {"name", wireId(wire)},
          {"resonator", wire.resonator},
          {"source",
           wire.couplerAtSource != NO_OWNER
               ? portOf(wire.sourcePort, wire.objective.source, one.ends[0],
                        one.from, "coupler port")
               : portOf(wire.sourcePort < ports_.size()
                            ? wire.sourcePort
                            : launcherAt(wire.objective.source),
                        wire.objective.source, one.ends[0], one.from, "")},
          {"target", portOf(wire.targetPort, wire.objective.target, one.ends[1],
                            one.to, "")},
          {"path", wire.way.empty()
                       ? json::array()
                       : corners(wire.way, 0, wire.way.size() - 1)}};
      if (wire.couplerAtSource == NO_OWNER &&
          wire.sourcePort >= ports_.size()) {
        // The launcher port names the start; the wire starts at its cell.
        entry["source"]["port"] = entry["source"]["at"];
      }
      entry["crosses"] = json::array();
      for (const auto feedline : one.crosses) {
        entry["crosses"].push_back(wireId(wires[feedline]));
      }
      if (!one.demand.has_value()) {
        entry["status"] = "port in no chamber";
        entry["way"] = json();
      } else if (const auto& way = check.ways[*one.demand]; !way.has_value()) {
        entry["status"] = "no way";
        entry["way"] = json();
      } else {
        entry["status"] = way->empty() ? "one chamber" : "routed";
        entry["way"] = *way;
      }
      outerWires.push_back(std::move(entry));
    }

    const json data{{"width", scene_.router.width},
                    {"height", scene_.router.height},
                    {"cellSize", scene_.router.cellWidth},
                    {"clearance", tuning_.clearance},
                    {"straightLength", tuning_.straightLength},
                    {"pitch", roomPitch()},
                    {"crossingReach", CROSSING_REACH + 1},
                    {"inflation", wallInflation()},
                    {"verdict", verdict},
                    {"summary", summary},
                    {"wallKinds", std::move(wallKinds)},
                    {"raster", std::move(raster)},
                    {"strokes", std::move(strokes)},
                    {"slots", std::move(slotCells)},
                    {"nodes", std::move(nodes)},
                    {"edges", std::move(edges)},
                    {"aside", std::move(aside)},
                    {"wires", std::move(outerWires)}};
    const auto where = debug_("final-capacity-graph.json", data.dump());
    if (!where.empty()) {
      say(std::format("the capacity graph as data: {}", where));
    }
  }

  /// Where the capacity graph puts the node of each chamber: the cell of
  /// it nearest the mean of its cells.
  [[nodiscard]] std::vector<debug::Cell>
  chamberNodes(const grid::Chambers& chambers) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<double> sumX(chambers.count, 0.0);
    std::vector<double> sumY(chambers.count, 0.0);
    std::vector<double> cells(chambers.count, 0.0);
    for (std::size_t cell = 0; cell < chambers.of.size(); ++cell) {
      const auto chamber = chambers.of[cell];
      if (chamber != grid::NO_CHAMBER) {
        sumX[chamber] +=
            static_cast<double>(static_cast<std::int64_t>(cell) % width);
        sumY[chamber] +=
            static_cast<double>(static_cast<std::int64_t>(cell) / width);
        cells[chamber] += 1.0;
      }
    }
    std::vector<debug::Cell> node(chambers.count, {0, 0});
    std::vector<double> best(chambers.count,
                             std::numeric_limits<double>::max());
    for (std::size_t cell = 0; cell < chambers.of.size(); ++cell) {
      const auto chamber = chambers.of[cell];
      if (chamber == grid::NO_CHAMBER) {
        continue;
      }
      const auto x = static_cast<std::int64_t>(cell) % width;
      const auto y = static_cast<std::int64_t>(cell) / width;
      const auto dx = static_cast<double>(x) - (sumX[chamber] / cells[chamber]);
      const auto dy = static_cast<double>(y) - (sumY[chamber] / cells[chamber]);
      const auto apart = (dx * dx) + (dy * dy);
      if (apart < best[chamber]) {
        best[chamber] = apart;
        node[chamber] = {x, y};
      }
    }
    return node;
  }

  /// The picture of `reportCapacityGraph`. The chambers are filled, two
  /// chambers an edge joins never in one colour; every chamber carries a
  /// node with its number and how many ports lie in it. A bottleneck is
  /// drawn as its line, coloured by how many wires the walk sent through it
  /// against what it takes, and joined to the nodes of its chambers; a
  /// stretch to cross is drawn along its feedline and joined the same way,
  /// dashed. The ports are dots, with the wire and its chamber on hover.
  void drawCapacityGraph(
      const std::vector<Wire>& wires, const Walls& walls,
      const std::vector<grid::Bottleneck>& gates,
      const grid::Chambers& chambers, const std::vector<GraphEdge>& graph,
      const std::vector<std::uint32_t>& portsIn,
      const std::vector<std::pair<PathPoint, std::uint32_t>>& portMarks) {
    if (!debug_) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const debug::View view{
        .minX = 0,
        .minY = 0,
        .maxX = width - 1,
        .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 13));
    svg.style(DEBUG_STYLE);

    // Greedy colours over the chambers, so that an edge never joins two of
    // one colour while eight colours last.
    constexpr int COLOURS = 8;
    std::vector<int> colour(chambers.count, 0);
    {
      std::vector<std::vector<std::uint32_t>> next(chambers.count);
      for (const auto& edge : graph) {
        for (const auto a : edge.chambers) {
          for (const auto b : edge.chambers) {
            if (a != b) {
              next[a].push_back(b);
            }
          }
        }
      }
      for (std::uint32_t chamber = 0; chamber < chambers.count; ++chamber) {
        std::array<bool, COLOURS> taken{};
        for (const auto other : next[chamber]) {
          if (other < chamber) {
            taken[static_cast<std::size_t>(colour[other])] = true;
          }
        }
        const auto free = std::ranges::find(taken, false);
        colour[chamber] = free == taken.end()
                              ? static_cast<int>(chamber % COLOURS)
                              : static_cast<int>(free - taken.begin());
      }
    }
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          const auto chamber =
              chambers.of[static_cast<std::size_t>((y * width) + x)];
          return chamber == grid::NO_CHAMBER ? 0 : 1 + colour[chamber];
        },
        [](const int value) { return std::format("k{}", value - 1); });
    drawWalls(svg, walls);

    const auto node = chamberNodes(chambers);

    const auto load = [](const GraphEdge& edge) {
      return edge.through.size() > edge.capacity    ? "over"
             : edge.through.size() == edge.capacity ? "full"
                                                    : "room";
    };
    for (const auto& edge : graph) {
      debug::Cell middle{0, 0};
      if (edge.crossing) {
        const auto& way = wires[edge.id].way;
        std::vector<debug::Cell> along;
        for (auto at = edge.first; at <= edge.last; ++at) {
          along.emplace_back(way[at].x, way[at].y);
        }
        if (along.size() == 1) {
          along.push_back(along.front());
        }
        svg.polyline(along, std::format("xs xs-{}", load(edge)));
        const auto& centre = way[(edge.first + edge.last) / 2];
        middle = {centre.x, centre.y};
      } else {
        const auto& gate = gates[edge.id];
        const debug::Cell from{static_cast<std::int64_t>(gate.first) % width,
                               static_cast<std::int64_t>(gate.first) / width};
        const debug::Cell to{static_cast<std::int64_t>(gate.second) % width,
                             static_cast<std::int64_t>(gate.second) / width};
        svg.line(from, to, std::format("gl gl-{}", load(edge)),
                 std::format("g{} · {} · {}: {:.1f} cells, takes {}, {} sent "
                             "through ({})",
                             edge.id, endName(walls, gate.first),
                             endName(walls, gate.second), edge.length,
                             edge.capacity, edge.through.size(),
                             namesOf(wires, edge.through)));
        middle = {(from.first + to.first) / 2, (from.second + to.second) / 2};
      }
      for (const auto chamber : edge.chambers) {
        svg.polyline({node[chamber], middle}, edge.crossing ? "ge gex" : "ge");
      }
      if (!edge.through.empty() || edge.crossing) {
        svg.text(svg.centreX(middle.first) + (0.6 * scale.marker),
                 svg.centreY(middle.second) - (0.6 * scale.marker),
                 std::format("{}/{}", edge.through.size(), edge.capacity),
                 std::format("gt gt-{}", load(edge)), 0.6 * scale.font);
      }
    }
    for (const auto& [port, key] : portMarks) {
      const auto chamber = chambers.of[static_cast<std::size_t>(
          (static_cast<std::int64_t>(port.y) * width) +
          static_cast<std::int64_t>(port.x))];
      svg.square(port.x, port.y, 0.35 * scale.marker, "pm",
                 std::format("port of {} · chamber {}", wireId(wires[key]),
                             chamber == grid::NO_CHAMBER
                                 ? std::string("-")
                                 : std::to_string(chamber)));
    }
    for (std::uint32_t chamber = 0; chamber < chambers.count; ++chamber) {
      svg.circle(node[chamber].first, node[chamber].second, 0.9 * scale.marker,
                 "node");
      svg.text(svg.centreX(node[chamber].first) + scale.marker,
               svg.centreY(node[chamber].second) + (0.35 * scale.font),
               portsIn[chamber] == 0
                   ? std::format("c{}", chamber)
                   : std::format("c{} · {}p", chamber, portsIn[chamber]),
               "nt", 0.65 * scale.font);
    }

    std::size_t overfull = 0;
    for (const auto& edge : graph) {
      overfull += edge.through.size() > edge.capacity ? 1 : 0;
    }
    drawLegend(
        svg, scale,
        {std::format(
             "capacity graph after the coupler insertion · {} chambers · "
             "{} edges · {} carry more wires than they take",
             chambers.count, graph.size(), overfull),
         std::format(
             "every outer wire routed at once from the chambers of its "
             "source port to those of its target port, least overflow "
             "first · a crossing keeps {} cells clear on each side, and two "
             "crossings {} cells apart",
             CROSSING_REACH + 1, roomPitch())},
        {{"k0", "a chamber: free space no bottleneck cuts; c<n> · <k>p = its "
                "number and how many ports lie in it"},
         {"wfl", "feedline edge"},
         {"wst", "port run: the rule's straight length from a port — a "
                 "plain wire's feed point, the end of a resonator's turn "
                 "off its coupler, a target port"},
         {"wsz", "the two walls of a port run, half the clearance to "
                 "either side"},
         {"wfz", "feedline edge inflated by half the clearance"},
         {"gl gl-over", "edge the flow sends more wires through than it "
                        "takes; sent/takes beside it"},
         {"gl gl-full", "bottleneck that takes exactly the wires it is sent"},
         {"gl gl-room", "bottleneck with room to spare"},
         {"xs xs-room",
          "stretch of a feedline a wire may cross, the edge to the "
          "chambers on its two sides dashed"},
         {"ge", "edge of the graph: node, the middle of its bottleneck or "
                "stretch, node"},
         {"node", "node: a chamber"},
         {"pm", "port: where a wire leaves its source stub or enters its "
                "target run"}});
    const auto where = debug_("final-capacity-graph.svg", svg.finish());
    if (!where.empty()) {
      say(std::format("the capacity graph: {}", where));
    }
  }

  /// The picture of `reportBottlenecks`: the obstacles by kind, the medial
  /// axis over them, the ways the outer wires have now, and every
  /// bottleneck as the line between its two obstacle cells with a dot on
  /// the saddle it was found from. The line's colour says how many wires
  /// it holds, and one crossed by more wires than it holds carries
  /// "holds/crossed" beside it, whatever its ends are. The tooltip names
  /// the two obstacles.
  void drawBottlenecks(const std::vector<Wire>& wires, const Walls& walls,
                       const grid::MedialAxis& axis,
                       const std::vector<MeasuredBottleneck>& measured) {
    if (!debug_) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const debug::View view{
        .minX = 0,
        .minY = 0,
        .maxX = width - 1,
        .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 13));
    svg.style(DEBUG_STYLE);
    const auto cellOf = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    drawWalls(svg, walls);
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return axis.onAxis(cellOf(x, y)) ? 1 : 0;
        },
        [](const int) { return std::string("wax"); });
    for (const auto& wire : wires) {
      if (wire.feasible && !wire.inner && !wire.feedline && !wire.way.empty()) {
        svg.polyline(cellsOf(wire.way), "sd");
      }
    }

    std::size_t labelled = 0;
    for (const auto& one : measured) {
      const debug::Cell from{static_cast<std::int64_t>(one.gate.first) % width,
                             static_cast<std::int64_t>(one.gate.first) / width};
      const debug::Cell to{static_cast<std::int64_t>(one.gate.second) % width,
                           static_cast<std::int64_t>(one.gate.second) / width};
      const auto* const cls = one.holds == 0   ? "b0"
                              : one.holds == 1 ? "b1"
                                               : "b2";
      svg.line(from, to, cls,
               std::format("{} · {}: {:.1f} cells, holds {}, crossed now by {} "
                           "({})",
                           endName(walls, one.gate.first),
                           endName(walls, one.gate.second), one.cells,
                           one.holds, one.crossing.size(),
                           namesOf(wires, one.crossing)));
      svg.circle(static_cast<std::int64_t>(one.gate.saddle) % width,
                 static_cast<std::int64_t>(one.gate.saddle) / width,
                 0.45 * scale.marker, "bsd");
      if (one.crossing.size() > one.holds) {
        ++labelled;
        svg.text(0.5 * (svg.centreX(from.first) + svg.centreX(to.first)) +
                     scale.marker,
                 0.5 * (svg.centreY(from.second) + svg.centreY(to.second)),
                 std::format("{}/{}", one.holds, one.crossing.size()), "btx",
                 0.7 * scale.font);
      }
    }

    drawLegend(
        svg, scale,
        {std::format("bottlenecks after the coupler insertion · {} in all, {} "
                     "crossed by more wires than they hold",
                     measured.size(), labelled),
         std::format("medial axis over the obstacles · gaps up to {} wires · "
                     "rise {:.1f} cells · a gap of L cells between the "
                     "walls holds L / {} wires, rounded down · port runs "
                     "walled and feedlines inflated {:.1f} cells out",
                     bottleneckWires(), bottleneckRise(), tuning_.clearance,
                     wallInflation())},
        {{"wb", "border: outside the launcher cells"},
         {"ob", "artwork"},
         {"wc", "coupler pad"},
         {"wst", "port run: the rule's straight length from a port — a "
                 "plain wire's feed point, the end of a resonator's turn "
                 "off its coupler, a target port"},
         {"wsz", "the two walls of a port run, half the clearance to "
                 "either side"},
         {"wfz", "feedline edge inflated by half the clearance"},
         {"wfl", "feedline edge"},
         {"wax", "medial axis"},
         {"sd", "the ways the outer wires have now"},
         {"b0", "bottleneck that holds no wire"},
         {"b1", "bottleneck that holds one wire"},
         {"b2", "bottleneck that holds two or more"},
         {"bsd", "saddle: the cell of the axis it was found from"},
         {"btx", "holds/crossed now, where more wires cross than it holds"}});
    const auto where = debug_("final-bottlenecks.svg", svg.finish());
    if (!where.empty()) {
      say(std::format("the bottlenecks: {}", where));
    }
  }

  void drawGrid(const std::vector<Wire>& wires) {
    if (!debug_) {
      return;
    }
    const debug::View view{
        .minX = 0,
        .minY = 0,
        .maxX = static_cast<std::int64_t>(scene_.router.width) - 1,
        .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 8));
    svg.style(DEBUG_STYLE);
    drawGround(svg, view);
    for (const auto& wire : wires) {
      if (wire.feasible) {
        svg.polyline(cellsOf(wire.way), "sd");
      }
    }
    // The ports themselves, before the wire ends, which lie a band beyond
    // them.
    drawPorts(svg, scale);
    for (const auto& wire : wires) {
      if (wire.feasible) {
        drawEnds(svg, wire, scale);
      }
    }
    drawLegend(
        svg, scale,
        {std::format(
             "final routing · grid {}x{} cells of {:.2f} layout "
             "units · {} wires",
             scene_.router.width, scene_.router.height,
             std::min(scene_.router.cellWidth, scene_.router.cellHeight),
             wires.size()),
         std::format("clearance {} cells · stub {} cells · band {} cells · "
                     "wire price {} · obstacle price {} over {} cells · "
                     "bend {}",
                     tuning_.clearance, tuning_.straightStart, tuning_.reach,
                     tuning_.wireProximityPenalty,
                     tuning_.staticProximityPenalty, tuning_.obstacleReach,
                     tuning_.bendPenalty)},
        {{"ob", "artwork and keepout: closed to every search"},
         {"s8", "obstacle halo: priced, darker is dearer"},
         {"sd", "the way the Detail stage drew, the seed of every search"},
         {"prt", "port as the chip carries it: square = its cell, dot = exact "
                 "position, line = orientation, p<n> = its index, label on "
                 "hover"},
         {"src", "source, line = heading it leaves on, number = wire"},
         {"tgt",
          "target beyond the port's band, line = heading it arrives on"}});
    const auto where = debug_("final-grid.svg", svg.finish());
    if (!where.empty()) {
      say(std::format("the grid: {}", where));
    }
  }

private:
  const Scene& scene_;
  Tuning tuning_;
  Progress progress_;
  Debug debug_;
  std::uint32_t verbosity_ = 0;
  DebugFrame frame_;
  std::uint32_t pictures_ = 0;
  /// The ports as the chip carries them, for the pictures.
  std::vector<PortMark> ports_;
  /// Where the picture of the last search went, for the line about it.
  std::string lastPicture_;
  /// What each round of the pass running now came to, per wire.
  struct RoundRecord {
    bool forward = true;
    std::uint32_t normal = 0;
    std::uint32_t relaxed = 0;
    /// Of the failed, the resonators that kept a way too short.
    std::uint32_t tooShort = 0;
    std::vector<std::string> failed;
  };
  std::vector<RoundRecord> rounds_;
  std::chrono::steady_clock::time_point began_ =
      std::chrono::steady_clock::now();
  std::shared_ptr<const MovePrimitives> primitives_;
  routing::AnalyticDubins analytic_;
  routing::SearchScratch scratch_;
  routing::DubinsRouter router_;
  Field field_;
  grid::BitGrid corridor_;
  std::vector<std::uint16_t> proximity_;
  std::vector<std::uint32_t> seen_;
  std::vector<std::size_t> queue_;
  std::vector<std::uint32_t> depth_;
  std::vector<std::int32_t> wall_;
  std::vector<std::int32_t> middle_;
  std::unordered_map<std::uint32_t, Stencil> stencils_;
  Box box_;
  std::uint32_t pass_ = 0;
  mutable routing::PathLoopScratch loopScratch_;
  /// Whether a resonator is made the target length exactly, once its
  /// coupler is in, rather than at least `meander_length`.
  bool exact_ = false;
  /// Whether the pass running now is under the feedline constraints.
  bool feedlinePass_ = false;
  /// The prices the last picture actually held, for its legend: a ramp means
  /// nothing without the figures it stands for.
  std::string priceShades_;
  /// The cells the coupler bodies take.
  grid::BitGrid bodies_;
  /// Whether the corridor being built closes the coupler bodies. A feedline
  /// runs along a body on purpose, so for a feedline it does not.
  bool blockOnBodies_ = true;
  /// One byte per wire: whether it is a conventional wire, which the option
  /// price weighs heavier than a resonator.
  std::vector<std::uint8_t> conventional_;
  /// One entry per cell: the resonator whose surviving way holds it, or
  /// nobody. No coupler may be built over one.
  std::vector<std::uint32_t> tailOwner_;
  /// A price field of nothing, for the searches that price nothing. Filled
  /// once and never written again.
  std::vector<std::uint16_t> zeroProximity_;

  /// The way one edge of a chain was found to take, kept by the pair of
  /// coupler options at its two ends.
  ///
  /// What an edge runs between is those two options and nothing else: move
  /// either coupler and the edge is a different question, leave both and it
  /// is the same one. The greedy asks it again and again — every option of
  /// every coupler is weighed against both neighbours once per pass, and a
  /// pair it has already routed comes back on the next pass and the next.
  ///
  /// The pair is the whole key. What else could bear on the answer — the
  /// other chains, fencing this one — is deliberately left out: taking it
  /// in would make every entry stale on every move and there would be
  /// nothing left to keep. The price is that an entry can outlive the
  /// ground it was found on; the fails say what that costs.
  /// The greedy leaves the staleness above standing. The exact search does
  /// not: at the head of every round it forgets the entries whose corridor
  /// a way that moved could have changed, and keeps the rest — see
  /// `forgetEdgesNear`.
  std::unordered_map<std::uint64_t, Path> edgeMemo_;
  std::uint64_t memoHits_ = 0;
  std::uint64_t memoMisses_ = 0;

  /// The option a waypoint stands on; a launcher has none and counts zero.
  [[nodiscard]] std::size_t optionAt(const std::uint32_t chain,
                                     const std::size_t at) const {
    const auto& point = chains_[chain][at];
    return point.fixed ? 0 : couplers_[point.coupler].chosen + 1;
  }

  /// `ahead` is the option the waypoint **before** this edge stands on, plus
  /// one, and zero for an edge that has no predecessor or is being priced
  /// first order. It belongs in the key because the second-order search
  /// fences the way that predecessor takes, so one pair of endpoint options
  /// can have two different ways under two different predecessors. Zero is
  /// therefore the first-order slot, which is what the greedy uses and what
  /// the second order reads its own fence out of.
  [[nodiscard]] std::uint64_t edgeMemoKey(const std::uint32_t chain,
                                          const std::size_t from,
                                          const std::size_t ahead = 0) const {
    return (static_cast<std::uint64_t>(chain & 0xFFU) << 56U) |
           (static_cast<std::uint64_t>(from & 0xFFU) << 48U) |
           (static_cast<std::uint64_t>(ahead & 0xFFFFU) << 32U) |
           (static_cast<std::uint64_t>(optionAt(chain, from) & 0xFFFFU)
            << 16U) |
           static_cast<std::uint64_t>(optionAt(chain, from + 1) & 0xFFFFU);
  }

  /// The approaches of every port, inflated by the clearance, where no
  /// coupler may sit.
  grid::BitGrid approaches_;
  /// The rectangle the launcher stubs leave open, which a coupler sits in.
  CouplerBox couplerBox_;
  std::vector<Coupler> couplers_;
  /// The coupler of every resonator that has one, by wire key.
  std::unordered_map<std::uint32_t, std::uint32_t> couplerOfWire_;
  std::vector<std::vector<Waypoint>> chains_;
  std::vector<Edge> edges_;
  /// The way of every edge of every chain as the greedy last chose it, per
  /// chain in edge order, until the edge has a wire of its own. What is
  /// committed stands in the way of every edge routed after it.
  std::vector<std::vector<Path>> chainEdgePaths_;
  /// The wire of every edge of every chain once it is committed, `NO_OWNER`
  /// before.
  std::vector<std::vector<std::uint32_t>> edgeWireOf_;
  /// How far the crossing rule reaches from a feedline's straight run, in
  /// cells: the prototype's `expand_radius`.
  static constexpr int CROSSING_REACH = 10;
};

// -------------------------------------------------------- Building the wires

/// The cell of the router grid a cell of the detail grid falls on.
[[nodiscard]] PathPoint onRouter(const grid::GridMetrics& detail,
                                 const grid::GridMetrics& router,
                                 const fbg::DCoord& cell) {
  const auto place = detail.toLayout(cell.x(), cell.y());
  const auto found = router.clampToCell(place);
  return {.x = found.x(), .y = found.y(), .heading = 0, .primitive = 0};
}

/// The way the Detail stage drew, on the grid this stage draws on.
///
/// It is a seed and nothing else. The Detail stage's cells are a centre line
/// on a grid three to four times coarser, so what comes back is a chain of
/// cells with gaps; every use of it here is a set of places to spread from or
/// a polygon to price against, and neither needs the chain to be unbroken.
[[nodiscard]] Path seedOf(const grid::GridMetrics& detail,
                          const grid::GridMetrics& router,
                          const std::vector<fbg::DCoord>& path) {
  Path way;
  way.reserve(path.size());
  for (const auto& cell : path) {
    const auto point = onRouter(detail, router, cell);
    if (way.empty() || !point.samePlace(way.back())) {
      way.push_back(point);
    }
  }
  return way;
}

[[nodiscard]] grid::GridMetrics metricsOf(const fba::GridExtentT& extent) {
  return {.width = extent.width,
          .height = extent.height,
          .origin = extent.origin,
          .cellWidth = extent.cell_width,
          .cellHeight = extent.cell_height};
}

/// Every wire the stage has to draw: the ring first, in the order the
/// assignment holds it, and the inner circuit after it.
[[nodiscard]] std::vector<Wire> wiresOf(const ChipT& chip,
                                        const GlobalRoutingT& global,
                                        const AssignmentT& assignment,
                                        const DetailRoutingT& detail,
                                        const Scene& scene) {
  const auto detailGrid = metricsOf(*detail.grid);
  std::vector<Wire> wires;
  wires.reserve(assignment.connections.size() + global.connections.size());

  const auto place =
      [&](const std::uint32_t port, const bool leaving,
          PathPoint& into) { // NOLINT(bugprone-easily-swappable-parameters)
        const auto cell = scene.targetCell.find(port);
        const auto heading = scene.arrival.find(port);
        if (cell == scene.targetCell.end() || heading == scene.arrival.end()) {
          return false;
        }
        const auto found = scene.router.cell(cell->second);
        into = {.x = found.x(),
                .y = found.y(),
                .heading = leaving ? routing::reverse(heading->second)
                                   : heading->second,
                .primitive = 0};
        return true;
      };

  for (std::uint32_t index = 0; index < assignment.connections.size();
       ++index) {
    const auto& connection = *assignment.connections[index];
    Wire wire;
    wire.slot = index;
    wire.key = static_cast<std::uint32_t>(wires.size());
    wire.resonator =
        connection.target_role == fbd::AssignedRole::ResonatorTarget;
    if (index < detail.wires.size()) {
      wire.way = seedOf(detailGrid, scene.router, detail.wires[index]->path);
    }

    // A wire starts where the assignment feeds it, which is a point on the
    // ring and not a port: a resonator is fed on the segment between two
    // launchers, so no port stands there. The heading it leaves with is the
    // reverse of the one a wire arrives at its launcher with.
    const bool fed =
        index < assignment.feeds.size() && index < assignment.launchers.size();
    if (fed) {
      const auto launcher = assignment.launchers[index].index();
      if (launcher < chip.ports.size()) {
        const auto heading =
            routing::headingOfOrientation(chip.ports[launcher]->orientation);
        const auto cell = scene.router.clampToCell(assignment.feeds[index]);
        wire.objective.source = {.x = cell.x(),
                                 .y = cell.y(),
                                 .heading = routing::reverse(heading),
                                 .primitive = 0};
        wire.feasible = heading < routing::NUM_HEADINGS;
      }
    }
    wire.targetPort = connection.target.index();
    if (!place(connection.target.index(), false, wire.objective.target)) {
      wire.feasible = false;
    }
    // A resonator's length runs to the port itself, and the way ends on the
    // cell beyond the port's band, so the gap between the two is part of
    // the length. The port's exact position is in fractional cells, as the
    // sampler measures.
    if (wire.resonator && wire.feasible &&
        connection.target.index() < chip.ports.size()) {
      const auto exact =
          scene.router.toCell(chip.ports[connection.target.index()]->center);
      const auto dx = static_cast<double>(wire.objective.target.x) - exact.x();
      const auto dy = static_cast<double>(wire.objective.target.y) - exact.y();
      wire.anchorGap = std::hypot(dx, dy);
      wire.anchorGapUnits =
          std::hypot(dx * scene.router.cellWidth, dy * scene.router.cellHeight);
    }
    wires.push_back(std::move(wire));
  }

  for (std::uint32_t index = 0; index < global.connections.size(); ++index) {
    const auto& connection = *global.connections[index];
    Wire wire;
    wire.inner = true;
    wire.slot = index;
    wire.key = static_cast<std::uint32_t>(wires.size());
    if (index < detail.inner.size()) {
      wire.way = seedOf(detailGrid, scene.router, detail.inner[index]->path);
    }
    wire.targetPort = connection.target.index();
    if (connection.source != nullptr) {
      wire.sourcePort = connection.source->index();
    }
    wire.feasible =
        connection.source != nullptr &&
        place(connection.source->index(), true, wire.objective.source) &&
        place(connection.target.index(), false, wire.objective.target);
    wires.push_back(std::move(wire));
  }
  return wires;
}

// ------------------------------------------------------------- The artifact

/// How long a way is, in layout units.
using Measure = std::function<double(const Path&)>;

/// What the insertion says about an edge, by wire key: the `Squeezed` bit,
/// the note behind it and the line it measured. See `Driver::reportSqueeze`.
using Notes = std::unordered_map<std::uint32_t, Driver::Squeeze>;

[[nodiscard]] std::unique_ptr<fba::FinalWireT>
wireOf(const Path& way, const double length, const std::uint8_t verdict = 0,
       const Driver::Squeeze* squeeze = nullptr) {
  auto drawn = std::make_unique<fba::FinalWireT>();
  drawn->path.reserve(way.size());
  for (const auto& point : way) {
    drawn->path.emplace_back(point.x, point.y, point.heading);
  }
  drawn->length = length;
  drawn->verdict = static_cast<fba::FinalVerdict>(verdict);
  if (squeeze != nullptr) {
    drawn->note = squeeze->note;
    drawn->marks.emplace_back(squeeze->from.x, squeeze->from.y,
                              squeeze->from.heading);
    drawn->marks.emplace_back(squeeze->to.x, squeeze->to.y,
                              squeeze->to.heading);
  }
  return drawn;
}

/// The couplers as the artifact carries them.
using CouplerList = std::vector<std::unique_ptr<fbd::CpwCouplerT>>;

[[nodiscard]] CouplerList copyOf(const CouplerList& couplers) {
  CouplerList copy;
  copy.reserve(couplers.size());
  for (const auto& coupler : couplers) {
    copy.push_back(std::make_unique<fbd::CpwCouplerT>(*coupler));
  }
  return copy;
}

/// The state of every wire, as one phase left it. `verdicts` is what the
/// stage holds against each wire by key, as `failsOf` counted it, and only
/// the end state is handed one: a phase snapshot is not judged and carries
/// zero on every wire.
[[nodiscard]] std::unique_ptr<fba::FinalPhaseT>
snapshotOf(std::string name, const std::vector<Wire>& wires,
           const std::uint32_t ring, const std::size_t inner,
           const Measure& measure, const std::vector<std::uint32_t>& edges = {},
           const CouplerList& couplers = {},
           const std::vector<std::uint8_t>* verdicts = nullptr,
           const Notes* notes = nullptr) {
  const auto verdictOf = [&](const std::uint32_t key) -> std::uint8_t {
    std::uint8_t bits =
        verdicts != nullptr && key < verdicts->size() ? (*verdicts)[key] : 0;
    if (notes != nullptr && notes->contains(key)) {
      bits |= static_cast<std::uint8_t>(fba::FinalVerdict::Squeezed);
    }
    return bits;
  };
  const auto noteOf = [notes](const std::uint32_t key) -> const Driver::Squeeze* {
    if (notes == nullptr) {
      return nullptr;
    }
    const auto found = notes->find(key);
    return found == notes->end() ? nullptr : &found->second;
  };
  const auto entryOf = [&](const Wire& wire) {
    return wire.drawn ? wireOf(wire.way, measure(wire.way), verdictOf(wire.key),
                               noteOf(wire.key))
                      : wireOf({}, 0.0, verdictOf(wire.key), noteOf(wire.key));
  };
  auto phase = std::make_unique<fba::FinalPhaseT>();
  phase->name = std::move(name);
  for (const auto key : edges) {
    phase->feedlines.push_back(entryOf(wires[key]));
  }
  phase->couplers = copyOf(couplers);
  phase->wires.reserve(ring);
  for (std::uint32_t at = 0; at < ring; ++at) {
    phase->wires.push_back(wireOf({}, 0.0));
  }
  phase->inner.reserve(inner);
  for (std::size_t at = 0; at < inner; ++at) {
    phase->inner.push_back(wireOf({}, 0.0));
  }
  for (const auto& wire : wires) {
    if (wire.feedline) {
      continue;
    }
    auto& into = wire.inner ? phase->inner : phase->wires;
    if (wire.slot < into.size()) {
      // An undrawn wire stays an empty entry, as before, and carries the
      // verdict against it — `Unrouted`, when the end state is judged.
      into[wire.slot] = entryOf(wire);
    }
  }
  return phase;
}

/// The degrees a heading stands for, the inverse of `headingOfOrientation`.
[[nodiscard]] double degreesOf(const Heading heading) {
  switch (heading & 7U) {
  case 2:
    return 0.0;
  case 1:
    return 45.0;
  case 0:
    return 90.0;
  case 7:
    return 135.0;
  case 6:
    return 180.0;
  case 5:
    return 225.0;
  case 4:
    return 270.0;
  default:
    return 315.0;
  }
}

/// The port a waypoint of a chain stands for: the launcher, or the port
/// the coupler created, which follows the chip's own ports in coupler order.
[[nodiscard]] fbd::PortRef portOf(const Driver& driver, const ChipT& chip,
                                  const std::uint32_t chain,
                                  const std::size_t at) {
  const auto& point = driver.chains()[chain][at];
  if (point.fixed) {
    return fbd::PortRef(point.port);
  }
  return fbd::PortRef(static_cast<std::uint32_t>(chip.ports.size()) +
                      point.coupler);
}

/// The couplers as they stand, for the artifact: the connection each
/// completes, the port it creates, and its body in layout units.
[[nodiscard]] CouplerList couplersOf(const Driver& driver,
                                     const std::vector<Wire>& wires,
                                     const ChipT& chip, const Scene& scene) {
  CouplerList list;
  for (std::uint32_t index = 0; index < driver.couplers().size(); ++index) {
    const auto& coupler = driver.couplers()[index];
    const auto& option = coupler.options[coupler.chosen];
    const auto& wire = wires[coupler.wire];
    auto made = std::make_unique<fbd::CpwCouplerT>();
    made->connection = fbd::ConnectionRef(wire.slot);
    made->port = std::make_unique<fbd::PortT>();
    made->port->label =
        std::format("{}.coupler", wire.targetPort < chip.ports.size()
                                      ? chip.ports[wire.targetPort]->label
                                      : std::to_string(index));
    made->port->center =
        scene.router.toLayout(option.anchor.x, option.anchor.y);
    // A wire arrives at the port against the heading the resonator leaves
    // its anchor on.
    made->port->orientation =
        degreesOf(routing::reverse(option.way.front().heading));
    made->port->role = fbd::UnassignedRole::Coupler;
    // The body is a pad centred on the anchor and turned to the option's
    // orientation — not to the heading the resonator happens to arrive on.
    // Drawing it on the arrival heading is what put the couplers at the
    // wrong angle in the picture while their cells sat right.
    const auto along = routing::headingVector(option.orientation);
    const auto across =
        routing::headingVector(routing::turned(option.orientation, 2));
    (void)across;
    // On the pad's own centre, which is the place the coupler was put at.
    // Not on the anchor: that is the resonator port on the pad's near edge,
    // and a body drawn on it hangs half a pad off where the coupler sits.
    made->center = scene.router.toLayout(option.centre.x, option.centre.y);
    // The rectangle is drawn on its own long axis, which is the axis the
    // feedline runs along. The coupler's orientation stands perpendicular to
    // it and is not what a rectangle is rotated by.
    made->rotation = static_cast<fbd::Rotation>(
        1 + static_cast<int>(degreesOf(option.orientation) / 45.0));
    // In layout units, by the step the cells are counted in: a diagonal step
    // is the diagonal of a cell, so a body that is fewer cells along a
    // diagonal is the length the design rule asks for all the same.
    const auto step = [&scene](const routing::HeadingVector v) {
      return std::hypot(v.dx * scene.router.cellWidth,
                        v.dy * scene.router.cellHeight);
    };
    made->length = option.run * step(along);
    made->height =
        option.depth *
        step(routing::headingVector(routing::turned(option.orientation, 2)));
    list.push_back(std::move(made));
  }
  return list;
}

/// The final router of the first release.
/// The pass of phase 1, the inner circuit.
[[nodiscard]] Pass innerPassOf(const Tuning& tuning) {
  return {.name = "inner routing",
          .rounds = tuning.innerRounds,
          .maxRelaxation = tuning.maxRelaxation,
          .reach = tuning.innerReach,
          .straightStart = 0,
          .keepDrawn = true};
}

/// The pass of phase 2, the ring.
[[nodiscard]] Pass outerPassOf(const Tuning& tuning) {
  return {.name = "outer routing",
          .rounds = tuning.rounds,
          .maxRelaxation = tuning.maxRelaxation,
          .reach = tuning.reach,
          .straightStart = tuning.straightStart,
          .keepDrawn = true};
}

/// The refinement that ends phase 2.
[[nodiscard]] Pass refinementPassOf(const Tuning& tuning) {
  return {.name = "refinement",
          .rounds = tuning.refinementRounds,
          .maxRelaxation = 0,
          .reach = tuning.reach,
          .straightStart = tuning.straightStart,
          .keepDrawn = false,
          .refinement = true};
}

/// Phase 2 of the stage: the ring, against the inner circuit and against
/// itself, then its refinement.
void routeTheRing(Driver& driver, std::vector<Wire>& wires,
                  const std::vector<std::uint32_t>& outer,
                  const Tuning& tuning) {
  driver.sayOuterSettings();
  const auto outerPass = outerPassOf(tuning);
  driver.sweep(wires, outer, outerPass);
  driver.recoverWithoutBands(wires, outer, outerPass);
  driver.refine(wires, outer, refinementPassOf(tuning));
  driver.sayLengthPoints(wires, outer);
}

class DubinsFinalRouter final : public IFinalRouter {
public:
  [[nodiscard]] FinalRoutingT
  run(const ChipT& chip, const CapacityPlanT& /*capacity*/,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const DetailRoutingT& detail, const ConfigT& config,
      const Progress& progress, const Debug& debug,
      const std::uint32_t verbosity) const override {
    if (config.grid == nullptr || config.rules == nullptr) {
      throw std::invalid_argument(
          "the configuration carries no grid section or no design rules");
    }
    if (detail.grid == nullptr) {
      throw std::invalid_argument("the detail routing carries no grid");
    }
    const auto scene = buildScene(chip, config);
    const auto tuning =
        tuningOf(config, grid::routerGrid(scene.capacity,
                                          config.grid->router_cell_size));
    const auto field = sceneOf(chip, config, scene.capacity);

    auto wires = wiresOf(chip, global, assignment, detail, field);
    const auto ring = static_cast<std::uint32_t>(assignment.connections.size());
    std::vector<std::uint32_t> outer;
    std::vector<std::uint32_t> inner;
    for (const auto& wire : wires) {
      (wire.inner ? inner : outer).push_back(wire.key);
    }

    Driver driver(field, tuning, progress, debug, verbosity);
    driver.attachPorts(portMarksOf(chip, field));
    driver.drawGrid(wires);
    const Measure measure = [&driver](const Path& way) {
      return driver.lengthInUnits(way);
    };

    FinalRoutingT routing;
    auto extent = std::make_unique<fba::GridExtentT>();
    extent->width = field.router.width;
    extent->height = field.router.height;
    extent->origin = field.router.origin;
    extent->cell_width = field.router.cellWidth;
    extent->cell_height = field.router.cellHeight;
    routing.grid = std::move(extent);

    // Where the run ends. `stop_after` names the last phase to run, by the
    // name of its snapshot; what follows it is not run at all, which is what
    // a stop is worth over a picture of a finished run. An unknown name runs
    // every phase, because the loader has already refused it.
    static constexpr std::array<std::string_view, 5> PHASES{
        "inner", "outer", "couplers", "feedlines", "refined"};
    std::size_t lastPhase = PHASES.size() - 1;
    for (std::size_t at = 0; at < PHASES.size(); ++at) {
      if (tuning.stopAfter == PHASES[at]) {
        lastPhase = at;
      }
    }
    const auto runs = [&lastPhase](const std::size_t phase) {
      return phase <= lastPhase;
    };

    // Phase 1. The inner circuit runs inside the unit cells, where the room is
    // narrow and there is nothing to steer around, so it takes no straight
    // stubs and a band a fraction of the ring's.
    driver.sayTheSetting();
    driver.sweep(wires, inner, innerPassOf(tuning));
    routing.phases.push_back(
        snapshotOf("inner", wires, ring, global.connections.size(), measure));

    // Phase 2. The ring, against the inner circuit and against itself.
    if (runs(1)) {
      routeTheRing(driver, wires, outer, tuning);
      routing.phases.push_back(
          snapshotOf("outer", wires, ring, global.connections.size(), measure));
    }

    // What the tail writes, whatever the run stops after: the edges of the
    // chains and the couplers as they stand. Every wire is counted from the
    // list as it is at the time, because the coupler insertion appends the
    // edges of the chains to it.
    std::vector<std::uint32_t> edges;
    const auto couplersNow = [&]() {
      return couplersOf(driver, wires, chip, field);
    };
    const auto everyWire = [&wires]() {
      std::vector<std::uint32_t> all(wires.size());
      std::iota(all.begin(), all.end(), 0U);
      return all;
    };

    // Phase 3. The couplers, and the feedline chains through them.
    if (runs(2)) {
      driver.insertCouplers(wires, assignment, chip);
      for (const auto& edge : driver.edges()) {
        edges.push_back(edge.wire);
      }
      routing.phases.push_back(snapshotOf("couplers", wires, ring,
                                          global.connections.size(), measure,
                                          edges, couplersNow(), nullptr,
                                          &driver.squeezed()));
    }

    // Phase 4. Every wire again under the feedline constraints, in the ring
    // with the edges of the chains, then the inner circuit; and the repair,
    // which turns a coupler while the feedlines leave fails.
    if (runs(3)) {
      driver.assignBridges(wires, outer);
      driver.checkBridgers(wires);
      const auto members = driver.ringWithEdges(wires, outer);
      const Pass feedlinePass{.name = "feedline routing",
                              .rounds = tuning.rounds,
                              .maxRelaxation = tuning.maxRelaxation,
                              .reach = tuning.reach,
                              .straightStart = tuning.straightStart,
                              .keepDrawn = true,
                              .feedlines = true};
      driver.sweep(wires, members, feedlinePass);
      driver.sweep(wires, inner,
                   {.name = "inner routing under the feedlines",
                    .rounds = tuning.innerRounds,
                    .maxRelaxation = tuning.maxRelaxation,
                    .reach = tuning.innerReach,
                    .straightStart = 0,
                    .keepDrawn = true,
                    .feedlines = true});
      driver.probeOnlyUnsettled(wires, members, feedlinePass);
      driver.repairFeedlines(wires, members, everyWire(), feedlinePass);
      routing.phases.push_back(snapshotOf("feedlines", wires, ring,
                                          global.connections.size(), measure,
                                          edges, couplersNow(), nullptr,
                                          &driver.squeezed()));

      // Phase 5. The refinement of the feedline routing.
      if (runs(4)) {
        driver.refine(wires, members,
                      {.name = "feedline refinement",
                       .rounds = tuning.feedlineRefinementRounds,
                       .maxRelaxation = 0,
                       .reach = tuning.reach,
                       .straightStart = tuning.straightStart,
                       .keepDrawn = false,
                       .feedlines = true,
                       .refinement = true});
        routing.phases.push_back(snapshotOf("refined", wires, ring,
                                            global.connections.size(), measure,
                                            edges, couplersNow(), nullptr,
                                            &driver.squeezed()));
      }
    }

    // What the stage came to, over every wire of the plan and counted with
    // every wire down, so that the last line of a run says what the
    // design-rule check will find.
    if (lastPhase + 1 < PHASES.size()) {
      driver.say(std::format("the stage stops after {}, as stop_after asks",
                             PHASES[lastPhase]));
    }
    driver.sayResonatorLengths(wires, outer);
    const auto fails = driver.failsOf(wires, everyWire());
    driver.sayFails("final routing", fails,
                    static_cast<std::uint32_t>(wires.size()));

    // The end state carries the verdict against every wire, so that a
    // picture of the run marks what the last line counted.
    auto last = snapshotOf("", wires, ring, global.connections.size(), measure,
                           edges, couplersNow(), &fails.verdicts,
                           &driver.squeezed());
    routing.wires = std::move(last->wires);
    routing.inner = std::move(last->inner);
    routing.feedlines = std::move(last->feedlines);
    routing.couplers = std::move(last->couplers);
    for (const auto& edge : driver.edges()) {
      auto described = std::make_unique<fba::FeedlineEdgeT>();
      described->chain = edge.chain;
      described->from = portOf(driver, chip, edge.chain, edge.from);
      described->to = portOf(driver, chip, edge.chain, edge.to);
      described->terminal = edge.terminal;
      routing.feedline_edges.push_back(std::move(described));
    }
    // A `ConnectionRef` indexes the connections of the assignment, so only a
    // wire of the ring can be named here. An inner wire that was not drawn is
    // an empty entry in `inner`, which is what a reader counts.
    for (const auto& wire : wires) {
      if (!wire.drawn && !wire.inner) {
        routing.unresolved.emplace_back(wire.slot);
      }
    }
    return routing;
  }
};

} // namespace

std::unique_ptr<IFinalRouter> makeDubinsFinalRouter() {
  return std::make_unique<DubinsFinalRouter>();
}

/// What a `CouplerSession` holds: the scene, the tuning and the wires the
/// driver works on, and the driver, which refers to the first two.
struct CouplerSession::State {
  State(Scene scene, Tuning tuning)
      : field(std::move(scene)), tuning(std::move(tuning)) {}
  Scene field;
  Tuning tuning;
  std::vector<Wire> wires;
  std::unique_ptr<Driver> driver;
};

CouplerSession::CouplerSession(const ChipT& chip, const GlobalRoutingT& global,
                               const AssignmentT& assignment,
                               const DetailRoutingT& detail,
                               const ConfigT& config, const Progress& progress,
                               const std::uint32_t verbosity) {
  if (config.grid == nullptr || config.rules == nullptr) {
    throw std::invalid_argument(
        "the configuration carries no grid section or no design rules");
  }
  if (detail.grid == nullptr) {
    throw std::invalid_argument("the detail routing carries no grid");
  }
  const auto scene = buildScene(chip, config);
  state_ = std::make_unique<State>(
      sceneOf(chip, config, scene.capacity),
      tuningOf(config, grid::routerGrid(scene.capacity,
                                        config.grid->router_cell_size)));
  auto& wires = state_->wires;
  wires = wiresOf(chip, global, assignment, detail, state_->field);
  std::vector<std::uint32_t> outer;
  std::vector<std::uint32_t> inner;
  for (const auto& wire : wires) {
    (wire.inner ? inner : outer).push_back(wire.key);
  }
  state_->driver = std::make_unique<Driver>(state_->field, state_->tuning,
                                            progress, Debug{}, verbosity);
  auto& driver = *state_->driver;
  driver.attachPorts(portMarksOf(chip, state_->field));
  driver.sayTheSetting();
  driver.sweep(wires, inner, innerPassOf(state_->tuning));
  routeTheRing(driver, wires, outer, state_->tuning);
  driver.insertCouplers(wires, assignment, chip);
}

CouplerSession::~CouplerSession() = default;

std::string CouplerSession::couplers() const {
  return state_->driver->couplersJson(state_->wires);
}

std::string CouplerSession::setOption(const std::uint32_t coupler,
                                      const std::uint32_t option) {
  return state_->driver->setCouplerOption(state_->wires, coupler, option);
}

std::string CouplerSession::graph() {
  return state_->driver->capacityGraphJson(state_->wires);
}

} // namespace mqt::scpd::pipeline
