/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/MeanderInsertion.hpp"

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// The most fingers a meander has.
constexpr uint32_t MAX_FINGERS = 15;

/// The widest pair, in cells of the straight runs, the fallback sweep over
/// every cell tries: the prototype's `kSmallDetourMaxSpan`.
constexpr std::int64_t SMALL_SPAN = 150;

/// How many placements the insertion builds per alternative asked for before
/// it stops looking for more. Most of the next cheapest placements start
/// inside the box of the meander taken and are skipped unbuilt; the rest are
/// built, and some of them still overlap it.
constexpr uint32_t MAX_ALTERNATIVE_BUILDS = 16;

/// A cell with a heading and the move that leaves it, in signed arithmetic,
/// so that a candidate off the grid is a number and not a wrapped one.
struct Spot {
  int64_t x = 0;
  int64_t y = 0;
  Heading heading = 0;
  uint16_t primitive = 0;
};

/// A run of spots that replaces part of a path.
using Piece = std::vector<Spot>;

/// A box in signed arithmetic, both bounds included.
struct Box {
  int64_t minX = 0;
  int64_t maxX = -1;
  int64_t minY = 0;
  int64_t maxY = -1;
  [[nodiscard]] bool holds(const int64_t x, const int64_t y) const {
    return x >= minX && x <= maxX && y >= minY && y <= maxY;
  }
};

/// Where a move from a spot ends, on the heading it exits with.
Spot after(const Spot& at, const Primitive& move) {
  return {.x = at.x + move.dx,
          .y = at.y + move.dy,
          .heading = move.exitHeading,
          .primitive = 0};
}

/// Append the cells a move sweeps from a spot, each tagged with the move,
/// the move's end excluded: the end is where the next move starts, and the
/// tag of a cell is the move that leaves it. This is how the router writes
/// its own paths.
void appendMove(Piece& into, const Spot& at, const Primitive& move) {
  const Spot end = after(at, move);
  into.push_back(
      {.x = at.x, .y = at.y, .heading = at.heading, .primitive = move.id});
  for (const CellOffset& offset : move.swept) {
    const Spot cell{.x = at.x + offset.dx,
                    .y = at.y + offset.dy,
                    .heading = at.heading,
                    .primitive = move.id};
    if ((offset.dx == 0 && offset.dy == 0) ||
        (cell.x == end.x && cell.y == end.y)) {
      continue;
    }
    if (into.back().x == cell.x && into.back().y == cell.y) {
      continue;
    }
    into.push_back(cell);
  }
}

/// Append a straight run of so many steps from a spot along its heading, and
/// move the spot to the end of the run.
void appendStraight(Piece& into, Spot& at, const MovePrimitives& primitives,
                    const int64_t steps) {
  const HeadingVector v = headingVector(at.heading);
  const uint16_t straight = primitives.straight(at.heading);
  for (int64_t k = 0; k < steps; ++k) {
    into.push_back(
        {.x = at.x, .y = at.y, .heading = at.heading, .primitive = straight});
    at.x += v.dx;
    at.y += v.dy;
  }
}

/// One or two moves that take a point from the heading it has onto a heading
/// across the path, or a point across the path onto the heading the path
/// continues on.
struct Chain {
  /// Its cells, from the first to the one before its end.
  Piece cells;
  /// Where it starts, on the heading it starts with.
  Spot from;
  /// Where it ends, on the heading it ends with.
  Spot to;
  double cost = 0.0;
};

using Targets = std::array<bool, NUM_HEADINGS>;

/// Every chain that leaves a point of the path and ends on one of the target
/// headings inside the box: each single move that does, and the cheapest
/// pair of moves per target heading. The prototype's
/// `generate_sanitized_coordinates_multi`, forward half.
std::vector<Chain> chainsFrom(const MovePrimitives& primitives,
                              const Spot& from, const Targets& target,
                              const Box& box) {
  std::vector<Chain> chains;
  std::array<std::optional<Chain>, NUM_HEADINGS> bestPair;
  for (const Primitive& first : primitives.of(from.heading)) {
    Chain one;
    one.from = from;
    appendMove(one.cells, from, first);
    one.to = after(from, first);
    one.cost = first.cost;
    if (!box.holds(one.to.x, one.to.y)) {
      continue;
    }
    for (const Primitive& second : primitives.of(one.to.heading)) {
      if (!target[second.exitHeading]) {
        continue;
      }
      const Spot end = after(one.to, second);
      if (!box.holds(end.x, end.y)) {
        continue;
      }
      const double cost = first.cost + second.cost;
      auto& best = bestPair[end.heading];
      if (best.has_value() && best->cost <= cost) {
        continue;
      }
      Chain two = one;
      appendMove(two.cells, one.to, second);
      two.to = end;
      two.cost = cost;
      best = std::move(two);
    }
    if (target[one.to.heading]) {
      chains.push_back(std::move(one));
    }
  }
  for (auto& best : bestPair) {
    if (best.has_value()) {
      chains.push_back(std::move(*best));
    }
  }
  return chains;
}

/// Every chain that starts on one of the target headings inside the box and
/// ends at a point of the path on the heading the path has there: each single
/// move that does, and the cheapest pair of moves per target heading. The
/// backward half of the same.
std::vector<Chain> chainsInto(const MovePrimitives& primitives, const Spot& to,
                              const Targets& target, const Box& box) {
  std::vector<Chain> chains;
  std::array<std::optional<Chain>, NUM_HEADINGS> bestPair;
  for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
    if (!target[heading]) {
      continue;
    }
    for (const Primitive& first : primitives.of(heading)) {
      if (first.exitHeading == to.heading) {
        const Spot start{.x = to.x - first.dx,
                         .y = to.y - first.dy,
                         .heading = heading,
                         .primitive = 0};
        if (box.holds(start.x, start.y)) {
          Chain one;
          one.from = start;
          one.to = to;
          one.cost = first.cost;
          appendMove(one.cells, start, first);
          chains.push_back(std::move(one));
        }
      }
      for (const Primitive& second : primitives.of(first.exitHeading)) {
        if (second.exitHeading != to.heading) {
          continue;
        }
        const Spot middle{.x = to.x - second.dx,
                          .y = to.y - second.dy,
                          .heading = first.exitHeading,
                          .primitive = 0};
        const Spot start{.x = middle.x - first.dx,
                         .y = middle.y - first.dy,
                         .heading = heading,
                         .primitive = 0};
        if (!box.holds(middle.x, middle.y) || !box.holds(start.x, start.y)) {
          continue;
        }
        const double cost = first.cost + second.cost;
        auto& best = bestPair[heading];
        if (best.has_value() && best->cost <= cost) {
          continue;
        }
        Chain two;
        two.from = start;
        two.to = to;
        two.cost = cost;
        appendMove(two.cells, start, first);
        appendMove(two.cells, middle, second);
        best = std::move(two);
      }
    }
  }
  for (auto& best : bestPair) {
    if (best.has_value()) {
      chains.push_back(std::move(*best));
    }
  }
  return chains;
}

/// The quarter turn of the primitives from one heading onto the next but
/// one. A cardinal heading holds one, which ends a radius along each of the
/// two; a diagonal heading holds one as well, added on top of its eighth
/// turns.
const Primitive* quarterTurn(const MovePrimitives& primitives,
                             const Heading from, const Heading to) {
  for (const Primitive& move : primitives.of(from)) {
    if (move.exitHeading == to) {
      return &move;
    }
  }
  return nullptr;
}

/// A meander from a point on a heading to a point on the opposite heading
/// beside it: out along the heading, a quarter turn, across, a quarter turn,
/// and back, `fingers` times side by side, each finger as deep as the first
/// and joined to the next by a run across at the foot of the two. Its depth
/// is what the wanted length makes it. Where even the flattest meander is
/// longer than wanted, it is built flat when it overshoots by no more than
/// `slack`.
///
/// The heading may be cardinal or diagonal. With `u` the step out, `v` the
/// step across and `D` the moves of the quarter turns, the meander joins the
/// two points when `b - a` is the legs along `u`, the runs along `v` and the
/// turns, which fixes the length of the runs across and the difference of
/// the first and the last leg.
///
/// The prototype's `compute_meander_with_sanitized_coordinates_*` with an
/// odd number of meanders, which is the shape its strict insertion accepts,
/// and its `construct_*_meander_path`. The prototype builds the meander
/// along the axes only.
std::optional<Piece>
buildLoop(const MovePrimitives& primitives, const Spot& a, const Spot& b,
          const double wanted, const double slack, const Box& box,
          const uint32_t safety, const uint32_t minStraight,
          const uint32_t fingers, MeanderResult::Refusals& why) {
  if (b.heading != reverse(a.heading)) {
    ++why.headingsApart;
    return std::nullopt;
  }
  const HeadingVector along = headingVector(a.heading);
  const int64_t dx = b.x - a.x;
  const int64_t dy = b.y - a.y;
  // The side b lies on, across a's heading.
  Heading across = a.heading;
  for (const int eighths : {2, -2}) {
    const Heading candidate = turned(a.heading, eighths);
    const HeadingVector v = headingVector(candidate);
    if ((dx * v.dx) + (dy * v.dy) > 0) {
      across = candidate;
    }
  }
  if (across == a.heading) {
    ++why.tooClose;
    return std::nullopt;
  }
  // The turns at the head of a finger, from the heading out onto the one
  // across and from there back, and at its foot the other way round.
  const Primitive* first = quarterTurn(primitives, a.heading, across);
  const Primitive* second = quarterTurn(primitives, across, b.heading);
  const Primitive* third = quarterTurn(primitives, b.heading, across);
  const Primitive* fourth = quarterTurn(primitives, across, a.heading);
  if (first == nullptr || second == nullptr ||
      (fingers > 1 && (third == nullptr || fourth == nullptr))) {
    ++why.headingsApart;
    return std::nullopt;
  }
  const HeadingVector v = headingVector(across);
  // One step along a heading: one cell on the axes, two on a diagonal, in
  // squared length; the out and the across step have the same.
  const int64_t norm = (along.dx * along.dx) + (along.dy * along.dy);
  const double stepLength =
      primitives.cost(a.heading, primitives.straight(a.heading));
  // The heads of the fingers and the feet between them.
  const auto heads = static_cast<int64_t>((fingers + 1) / 2);
  const int64_t feet = heads - 1;
  const auto runs = static_cast<int64_t>(fingers);
  // What the straight runs have to cover once every turn is made.
  int64_t restX = dx - (heads * (first->dx + second->dx));
  int64_t restY = dy - (heads * (first->dy + second->dy));
  double turns = static_cast<double>(heads) * (first->cost + second->cost);
  if (feet > 0) {
    restX -= feet * (third->dx + fourth->dx);
    restY -= feet * (third->dy + fourth->dy);
    turns += static_cast<double>(feet) * (third->cost + fourth->cost);
  }
  const int64_t alongRest = (restX * along.dx) + (restY * along.dy);
  const int64_t acrossRest = (restX * v.dx) + (restY * v.dy);
  if (alongRest % norm != 0 || acrossRest % norm != 0) {
    ++why.noJoin;
    return std::nullopt;
  }
  // How many steps the first leg is longer than the last, and the steps of
  // every run across together, shared out as evenly as they go.
  const int64_t forward = alongRest / norm;
  const int64_t across0 = acrossRest / norm;
  const int64_t narrowest = across0 < 0 ? -1 : across0 / runs;
  if (narrowest < 0 || static_cast<double>(narrowest) * stepLength <
                           static_cast<double>(minStraight)) {
    ++why.tooClose;
    return std::nullopt;
  }
  // The meander is the first leg, the legs between, the last leg, the runs
  // and the turns long. The legs between are as long as the first and the
  // last is the first less the forward difference, so each step deeper adds
  // one step per leg. Rounded up, so that the length is not undershot.
  const auto legs = static_cast<double>(fingers + 1);
  double out =
      std::ceil((((wanted - turns) / stepLength) -
                 static_cast<double>(across0) + static_cast<double>(forward)) /
                legs);
  // The flattest meander has no leg shorter than nothing.
  const auto flattest = static_cast<double>(std::max<int64_t>(0, forward));
  if (out < flattest) {
    if (legs * stepLength * (flattest - out) > slack) {
      ++why.noDepth;
      return std::nullopt;
    }
    out = flattest;
  }
  const auto outSteps = static_cast<int64_t>(out);
  const int64_t back = outSteps - forward;
  // The far edge of the meander, `safety` cells short of the box: the end of
  // the first turn, measured along the heading out. On a diagonal the end
  // of the last head is a corner of its own, and a meander of several
  // fingers reaches back past its start at their feet.
  const auto margin = static_cast<double>(safety);
  const auto beyond = [&](const double x, const double y) {
    return x < static_cast<double>(box.minX) + margin ||
           x > static_cast<double>(box.maxX) - margin ||
           y < static_cast<double>(box.minY) + margin ||
           y > static_cast<double>(box.maxY) - margin;
  };
  const auto alongOf = [&](const Primitive& turn) {
    return static_cast<double>((turn.dx * along.dx) + (turn.dy * along.dy)) /
           static_cast<double>(norm);
  };
  const auto at = [&](const Spot& from, const double depth) {
    return std::pair{static_cast<double>(from.x) + (depth * along.dx),
                     static_cast<double>(from.y) + (depth * along.dy)};
  };
  const auto [farX, farY] =
      at(a, static_cast<double>(outSteps) + alongOf(*first));
  bool outside = beyond(farX, farY);
  if (!outside && isDiagonal(a.heading)) {
    const auto [backX, backY] =
        at(b, static_cast<double>(back) - alongOf(*second));
    outside = beyond(backX, backY);
  }
  if (!outside && feet > 0) {
    const auto [footX, footY] =
        at(a, alongOf(*first) + alongOf(*second) + alongOf(*third));
    outside = beyond(footX, footY);
  }
  if (outside) {
    ++why.outsideBox;
    return std::nullopt;
  }

  Piece loop;
  Spot spot = a;
  appendStraight(loop, spot, primitives, outSteps);
  for (int64_t run = 0; run < runs; ++run) {
    const bool head = run % 2 == 0;
    const Primitive& onto = head ? *first : *third;
    const Primitive& off = head ? *second : *fourth;
    appendMove(loop, spot, onto);
    spot = after(spot, onto);
    appendStraight(loop, spot, primitives,
                   (across0 / runs) + (run < across0 % runs ? 1 : 0));
    appendMove(loop, spot, off);
    spot = after(spot, off);
    appendStraight(loop, spot, primitives, run + 1 == runs ? back : outSteps);
  }
  if (spot.x != b.x || spot.y != b.y) {
    ++why.noJoin;
    return std::nullopt;
  }
  return loop;
}

/// A cell of a straight run of the path: where it is, the rendered length up
/// to it, and its place in the path.
struct Straight {
  Spot spot;
  double lengthAt = 0.0;
  std::size_t index = 0;
};

} // namespace

MeanderResult insertMeander(const MovePrimitives& primitives, Path& path,
                            const double required,
                            const CellPredicate& enterable,
                            const MeanderOptions& options,
                            const CellPrice& price) {
  MeanderResult result;
  result.required = required;
  if (path.size() < 2) {
    return result;
  }
  std::vector<PathSegment> segments;
  const auto samples = samplePath(primitives, path, path.front(), segments);
  result.lengthBefore = polylineLength(samples);
  result.lengthAfter = result.lengthBefore;
  const double tolerance =
      options.exact ? std::max(0.0, options.tolerance) : 0.0;
  if (options.exact && result.lengthBefore > required + tolerance) {
    // Longer than it may be: nothing spliced in makes a path shorter.
    result.tooLong = true;
    return result;
  }
  if (result.lengthBefore >= required - tolerance) {
    result.reached = true;
    return result;
  }
  double wanted = required;
  if (!options.exact &&
      wanted - result.lengthBefore < options.smallDeficitMargin) {
    wanted += options.smallDeficitMargin;
  }
  const double deficit = wanted - result.lengthBefore;

  // The cells of the straight runs. The sampler has just cut the path into
  // its segments, so each cell is found again in the path in order. A run
  // of one straight step counts as well: it is the cell an arc ends on when
  // another arc or the end of the path follows, and the segments call it
  // straight only from its second cell on.
  std::vector<Straight> straights;
  std::size_t cursor = 0;
  for (const PathSegment& segment : segments) {
    if (!segment.straight &&
        !primitives.isStraight(segment.heading, segment.primitive)) {
      continue;
    }
    for (std::size_t i = 0; i < segment.cells.size(); ++i) {
      const PathPoint& cell = segment.cells[i];
      while (cursor < path.size() && !(path[cursor] == cell)) {
        ++cursor;
      }
      if (cursor >= path.size()) {
        break;
      }
      straights.push_back({.spot = {.x = cell.x,
                                    .y = cell.y,
                                    .heading = cell.heading,
                                    .primitive = cell.primitive},
                           .lengthAt = segment.lengthAt[i],
                           .index = cursor});
    }
  }
  const std::size_t count = straights.size();
  if (count < 2) {
    return result;
  }
  const std::size_t step = std::max<std::size_t>(1, count / 100);
  std::size_t start = count > options.startMargin ? options.startMargin : 0;
  while (start < count && straights[start].index < options.keepBefore) {
    ++start;
  }
  result.straightCells = static_cast<uint32_t>(count);
  result.reserved = static_cast<uint32_t>(start);
  // The widest span any pair of the cells left over spans, which is the
  // room a loop has to be built in: measured here so that an insertion
  // that never built one says why in one figure.
  for (std::size_t a = start; a < count; ++a) {
    for (std::size_t b = a + 1; b < count; ++b) {
      const auto dx = static_cast<int64_t>(straights[b].spot.x) -
                      static_cast<int64_t>(straights[a].spot.x);
      const auto dy = static_cast<int64_t>(straights[b].spot.y) -
                      static_cast<int64_t>(straights[a].spot.y);
      const auto apart =
          static_cast<uint32_t>(std::max(std::abs(dx), std::abs(dy)));
      result.widestSpan = std::max(result.widestSpan, apart);
    }
  }
  const Box box{.minX = options.box.minX,
                .maxX = options.box.maxX,
                .minY = options.box.minY,
                .maxY = options.box.maxY};

  // The chains of a cell depend on which headings a pair reaches out along,
  // and every cell is the near end of many pairs and the far end of many
  // more, so they are built once per cell and target set.
  using Chains = std::vector<Chain>;
  std::vector<std::array<std::optional<Chains>, 6>> heads(count);
  std::vector<std::array<std::optional<Chains>, 6>> tails(count);
  // The box of a pair: the box of the options, cut down to what the caller
  // allows between the two cells of the pair. With a box per pair the
  // chains of a cell differ per pair too, so they are not cached then.
  Box pairBox = box;
  const auto boxOfPair = [&](const Straight& g0, const Straight& g1) {
    if (!options.boxFor) {
      return box;
    }
    const auto strip = options.boxFor({.x = static_cast<uint32_t>(g0.spot.x),
                                       .y = static_cast<uint32_t>(g0.spot.y),
                                       .heading = g0.spot.heading,
                                       .primitive = g0.spot.primitive},
                                      {.x = static_cast<uint32_t>(g1.spot.x),
                                       .y = static_cast<uint32_t>(g1.spot.y),
                                       .heading = g1.spot.heading,
                                       .primitive = g1.spot.primitive});
    return Box{.minX = std::max<int64_t>(box.minX, strip.minX),
               .maxX = std::min<int64_t>(box.maxX, strip.maxX),
               .minY = std::max<int64_t>(box.minY, strip.minY),
               .maxY = std::min<int64_t>(box.maxY, strip.maxY)};
  };
  Chains uncachedHeads;
  Chains uncachedTails;
  const auto headsOf = [&](const std::size_t at, const std::size_t key,
                           const Targets& target) -> const Chains& {
    if (options.boxFor) {
      uncachedHeads =
          chainsFrom(primitives, straights[at].spot, target, pairBox);
      return uncachedHeads;
    }
    auto& slot = heads[at][key];
    if (!slot.has_value()) {
      slot = chainsFrom(primitives, straights[at].spot, target, box);
    }
    return *slot;
  };
  const auto tailsOf = [&](const std::size_t at, const std::size_t key,
                           const Targets& target) -> const Chains& {
    if (options.boxFor) {
      uncachedTails =
          chainsInto(primitives, straights[at].spot, target, pairBox);
      return uncachedTails;
    }
    auto& slot = tails[at][key];
    if (!slot.has_value()) {
      slot = chainsInto(primitives, straights[at].spot, target, box);
    }
    return *slot;
  };

  // The head, the meander and the tail as one run of cells, or nothing when
  // the meander does not fit or a cell of it may not be entered.
  const auto assemble = [&](const Chain& head, const Chain& tail,
                            const double loopLength,
                            const uint32_t fingers) -> std::optional<Piece> {
    const auto loop = buildLoop(
        primitives, head.to, tail.from, loopLength, tolerance, pairBox,
        options.safety, options.minStraightLength, fingers, result.refusals);
    if (!loop.has_value()) {
      return std::nullopt;
    }
    Piece piece;
    piece.reserve(head.cells.size() + loop->size() + tail.cells.size());
    const Piece& body = *loop;
    for (const Piece* const part : {&head.cells, &body, &tail.cells}) {
      for (const Spot& spot : *part) {
        if (!box.holds(spot.x, spot.y) ||
            !enterable(static_cast<uint32_t>(spot.x),
                       static_cast<uint32_t>(spot.y))) {
          ++result.refusals.blocked;
          return std::nullopt;
        }
        piece.push_back(spot);
      }
    }
    return piece;
  };

  // The fewest fingers a meander between two turned ends fits with: one
  // loop first, as deep as the length makes it, and more of them, each
  // shallower, when the deep one leaves the box or runs into a closed cell.
  // More fingers only narrow the runs across and add turns, so the count
  // stops growing once a meander is too narrow or too long at its flattest.
  // The prototype takes the fewest that fit its box as well.
  const auto fewestFingers = [&](const Chain& head, const Chain& tail,
                                 const double loopLength)
      -> std::optional<std::pair<Piece, uint32_t>> {
    for (uint32_t fingers = 1; fingers <= MAX_FINGERS; fingers += 2) {
      const auto refused = result.refusals;
      auto piece = assemble(head, tail, loopLength, fingers);
      if (piece.has_value()) {
        return std::pair{std::move(*piece), fingers};
      }
      if (result.refusals.tooClose != refused.tooClose ||
          result.refusals.noDepth != refused.noDepth ||
          result.refusals.headingsApart != refused.headingsApart) {
        break;
      }
    }
    return std::nullopt;
  };

  // The path with a piece in place of the cells between a pair, or nothing
  // when the spliced path meets itself.
  PathLoopScratch scratch;
  const auto splice = [&](const Straight& g0, const Straight& g1,
                          const Piece& piece) -> Path {
    Path spliced;
    spliced.reserve(path.size() + piece.size());
    spliced.insert(spliced.end(), path.begin(),
                   path.begin() + static_cast<std::ptrdiff_t>(g0.index));
    for (const Spot& spot : piece) {
      spliced.push_back({.x = static_cast<uint32_t>(spot.x),
                         .y = static_cast<uint32_t>(spot.y),
                         .heading = spot.heading,
                         .primitive = spot.primitive});
    }
    spliced.insert(spliced.end(),
                   path.begin() + static_cast<std::ptrdiff_t>(g1.index),
                   path.end());
    if (pathSelfIntersects(spliced, options.width, options.height, scratch)) {
      ++result.refusals.selfCrossing;
      return {};
    }
    return spliced;
  };

  // Build a placement: the meander is built for the length it should have,
  // and where the rendering falls short of the requirement by a fraction of
  // a cell per bend, it is built again that much deeper. The path itself is
  // left as it is.
  const auto build =
      [&](const Straight& g0, const Straight& g1, const Chain& head,
          const Chain& tail, double loopLength,
          const uint32_t fingers) -> std::optional<MeanderResult::Alternative> {
    // The best rendering that met the requirement, and whether any meander
    // was built at all.
    Path best;
    double bestAfter = 0.0;
    CellBox bestBox{.minX = 1, .maxX = 0, .minY = 1, .maxY = 0};
    bool built = false;
    const auto meets = [&](const double after) {
      return options.exact ? std::abs(after - required) <= tolerance
                           : after >= required;
    };
    // Where the rendering lands within a cell of the requirement the
    // meander is right; in the exact mode it is built again shallower as
    // well as deeper, so that it lands within the tolerance and not merely
    // above. A meander built again may no longer fit, for a flat one cannot
    // be made shallower; the best one so far is kept.
    const int attempts = options.exact ? 6 : 4;
    for (int attempt = 0; attempt < attempts; ++attempt) {
      const auto piece = assemble(head, tail, loopLength, fingers);
      Path candidate = piece.has_value() ? splice(g0, g1, *piece) : Path{};
      if (candidate.empty()) {
        break;
      }
      built = true;
      const double after = renderedLength(primitives, candidate);
      if (meets(after) &&
          (best.empty() ||
           std::abs(after - required) < std::abs(bestAfter - required))) {
        best = std::move(candidate);
        bestAfter = after;
        bestBox = {
            .minX = UINT32_MAX, .maxX = 0, .minY = UINT32_MAX, .maxY = 0};
        for (const Spot& spot : *piece) {
          bestBox.minX = std::min(bestBox.minX, static_cast<uint32_t>(spot.x));
          bestBox.maxX = std::max(bestBox.maxX, static_cast<uint32_t>(spot.x));
          bestBox.minY = std::min(bestBox.minY, static_cast<uint32_t>(spot.y));
          bestBox.maxY = std::max(bestBox.maxY, static_cast<uint32_t>(spot.y));
        }
      }
      if (options.exact) {
        if (std::abs(after - required) <= 1.0) {
          break;
        }
        loopLength += required - after;
        continue;
      }
      if (after >= required) {
        break;
      }
      loopLength += (required - after) + 1.0;
    }
    if (best.empty()) {
      if (built) {
        ++result.refusals.offLength;
      }
      return std::nullopt;
    }
    return MeanderResult::Alternative{
        .path = std::move(best), .lengthAfter = bestAfter, .meander = bestBox};
  };

  // Take a placement: build it and splice it into the path.
  const auto take = [&](const Straight& g0, const Straight& g1,
                        const Chain& head, const Chain& tail,
                        const double loopLength, const uint32_t fingers) {
    auto made = build(g0, g1, head, tail, loopLength, fingers);
    if (!made.has_value()) {
      return false;
    }
    path = std::move(made->path);
    result.reached = true;
    result.inserted = true;
    result.lengthAfter = made->lengthAfter;
    result.meander = made->meander;
    return true;
  };

  // Whether two meander boxes lie apart: they share at most a quarter of
  // the smaller one. The loop on the other side of the same run shares only
  // the cells along the run with the loop on this side, and is apart.
  const auto apart = [](const CellBox& a, const CellBox& b) {
    const auto cells = [](const CellBox& box) {
      return static_cast<uint64_t>(box.maxX - box.minX + 1) *
             (box.maxY - box.minY + 1);
    };
    const uint32_t minX = std::max(a.minX, b.minX);
    const uint32_t maxX = std::min(a.maxX, b.maxX);
    const uint32_t minY = std::max(a.minY, b.minY);
    const uint32_t maxY = std::min(a.maxY, b.maxY);
    if (minX > maxX || minY > maxY) {
      return true;
    }
    const auto shared =
        static_cast<uint64_t>(maxX - minX + 1) * (maxY - minY + 1);
    return 4 * shared <= std::min(cells(a), cells(b));
  };

  // What the cells of the path cost and where its steps change direction,
  // summed up front, so that a candidate is scored by what it adds and what
  // it takes out, without the whole path being walked for each.
  std::vector<double> priceUpTo;
  std::vector<uint32_t> bendsUpTo;
  if (price) {
    priceUpTo.assign(path.size() + 1, 0.0);
    bendsUpTo.assign(path.size() + 1, 0);
    for (std::size_t at = 0; at < path.size(); ++at) {
      priceUpTo[at + 1] = priceUpTo[at] + price(path[at].x, path[at].y);
      bendsUpTo[at + 1] = bendsUpTo[at];
      if (at >= 2 &&
          (static_cast<int64_t>(path[at].x) - path[at - 1].x !=
               static_cast<int64_t>(path[at - 1].x) - path[at - 2].x ||
           static_cast<int64_t>(path[at].y) - path[at - 1].y !=
               static_cast<int64_t>(path[at - 1].y) - path[at - 2].y)) {
        ++bendsUpTo[at + 1];
      }
    }
  }
  // The score of the spliced path: the price of every cell, and a bend
  // price per direction change. The changes at the seams are counted on the
  // piece with its neighbours on either side.
  const auto scoreOf = [&](const Straight& g0, const Straight& g1,
                           const Piece& piece) {
    double total =
        priceUpTo[g0.index] + (priceUpTo.back() - priceUpTo[g1.index]);
    Path seam;
    seam.reserve(piece.size() + 4);
    for (std::size_t back = std::min<std::size_t>(2, g0.index); back > 0;
         --back) {
      seam.push_back(path[g0.index - back]);
    }
    for (const Spot& spot : piece) {
      total +=
          price(static_cast<uint32_t>(spot.x), static_cast<uint32_t>(spot.y));
      seam.push_back({.x = static_cast<uint32_t>(spot.x),
                      .y = static_cast<uint32_t>(spot.y),
                      .heading = spot.heading,
                      .primitive = spot.primitive});
    }
    for (std::size_t ahead = 0; ahead < 2 && g1.index + ahead < path.size();
         ++ahead) {
      seam.push_back(path[g1.index + ahead]);
    }
    const std::size_t suffixFrom = std::min(path.size(), g1.index + 2);
    const auto bends = bendsUpTo[g0.index] +
                       (bendsUpTo.back() - bendsUpTo[suffixFrom]) +
                       countBends(seam);
    return total + (options.bendPrice * bends);
  };

  // A placement that fits, by what it takes to build it again: the pair,
  // the headings it reaches out along, the two turned ends by their place
  // in the lists of the pair, and the number of fingers.
  struct Fit {
    std::size_t from = 0;
    std::size_t to = 0;
    Targets target{};
    std::size_t key = 0;
    std::size_t head = 0;
    std::size_t tail = 0;
    double loopLength = 0.0;
    uint32_t fingers = 1;
    double score = 0.0;
  };
  std::vector<Fit> fits;

  // One sweep over the pairs, `stride` cells apart and spanning at most
  // `widest`, with a meander along the axes or, as the last resort, along
  // the diagonals; whether it took one.
  //
  // The pairs are tried by their span, the smallest first, and within one
  // span from the target end of the path, so that the smallest meander goes
  // as near the target as it fits. The source end is where the coupler
  // insertion cuts the way back to the target length and where the feedline
  // runs, so a meander there is cut in two or in the feedline's way. The
  // prototype walks from the source; its later phases place the meander
  // again from the coupler.
  //
  // Without a price the first placement that fits is taken. With one, every
  // placement that fits is scored and the cheapest is taken, as in the
  // prototype's strict insertion.
  const auto first = static_cast<std::int64_t>(start);
  const auto last = static_cast<std::int64_t>(count) - 1;
  const auto sweep = [&](const std::int64_t stride, const std::int64_t widest,
                         const bool diagonal) {
    fits.clear();
    for (std::int64_t span = stride; span <= std::min(widest, last - first);
         span += stride) {
      for (std::int64_t i = last; i - span >= first; i -= stride) {
        const std::int64_t j = i - span;
        const Straight& g0 = straights[static_cast<std::size_t>(j)];
        const Straight& g1 = straights[static_cast<std::size_t>(i)];
        // A meander reaches out across the pair: across a pair that lies
        // along x it runs along y, and the other way round. Across a pair
        // that lies along a diagonal, the smaller of its two distances at
        // least half the larger, it may run along the other diagonal.
        const int64_t apartX = std::abs(g1.spot.x - g0.spot.x);
        const int64_t apartY = std::abs(g1.spot.y - g0.spot.y);
        Targets target{};
        std::size_t key = 0;
        if (diagonal) {
          if (apartX == 0 || apartY == 0 ||
              2 * std::min(apartX, apartY) < std::max(apartX, apartY)) {
            continue;
          }
          const bool sameSign =
              (g1.spot.x - g0.spot.x > 0) == (g1.spot.y - g0.spot.y > 0);
          target[sameSign ? 3 : 1] = true;
          target[sameSign ? 7 : 5] = true;
          key = sameSign ? 5U : 4U;
        } else {
          if (apartX >= apartY) {
            target[0] = true;
            target[4] = true;
            key |= 1U;
          }
          if (apartX <= apartY) {
            target[2] = true;
            target[6] = true;
            key |= 2U;
          }
        }
        // What replaces the cells between the pair has to be as long as
        // they were, plus the deficit.
        const double lengthToAdd = (g1.lengthAt - g0.lengthAt) + deficit;
        pairBox = boxOfPair(g0, g1);
        if (pairBox.maxX < pairBox.minX || pairBox.maxY < pairBox.minY) {
          continue;
        }
        const Chains& headChains =
            headsOf(static_cast<std::size_t>(j), key, target);
        const Chains& tailChains =
            tailsOf(static_cast<std::size_t>(i), key, target);
        for (std::size_t h = 0; h < headChains.size(); ++h) {
          const Chain& head = headChains[h];
          for (std::size_t t = 0; t < tailChains.size(); ++t) {
            const Chain& tail = tailChains[t];
            ++result.candidates;
            if (tail.from.heading != reverse(head.to.heading)) {
              ++result.refusals.headingsApart;
              continue;
            }
            const double loopLength = lengthToAdd - head.cost - tail.cost;
            const auto fit = fewestFingers(head, tail, loopLength);
            if (!fit.has_value()) {
              continue;
            }
            ++result.fits;
            if (!price) {
              if (take(g0, g1, head, tail, loopLength, fit->second)) {
                return true;
              }
              continue;
            }
            fits.push_back({.from = static_cast<std::size_t>(j),
                            .to = static_cast<std::size_t>(i),
                            .target = target,
                            .key = key,
                            .head = h,
                            .tail = t,
                            .loopLength = loopLength,
                            .fingers = fit->second,
                            .score = scoreOf(g0, g1, fit->first)});
          }
        }
      }
    }
    std::ranges::stable_sort(fits, {}, &Fit::score);
    const auto buildFit = [&](const Fit& fit) {
      const Straight& g0 = straights[fit.from];
      const Straight& g1 = straights[fit.to];
      pairBox = boxOfPair(g0, g1);
      const Chain head = headsOf(fit.from, fit.key, fit.target)[fit.head];
      const Chain tail = tailsOf(fit.to, fit.key, fit.target)[fit.tail];
      return build(g0, g1, head, tail, fit.loopLength, fit.fingers);
    };
    std::optional<MeanderResult::Alternative> chosen;
    std::size_t at = 0;
    for (; at < fits.size() && !chosen.has_value(); ++at) {
      chosen = buildFit(fits[at]);
    }
    if (!chosen.has_value()) {
      return false;
    }
    // The alternatives: the next cheapest placements whose meanders lie
    // apart from the one taken and from each other. A placement that reaches
    // out to the same side as one taken, over more than half of the same
    // stretch of the path, overlaps it and is not built.
    struct Taken {
      CellBox box;
      std::size_t from = 0;
      std::size_t to = 0;
      Heading out = 0;
    };
    const auto takenOf = [&](const Fit& fit, const CellBox& box) {
      return Taken{
          .box = box,
          .from = straights[fit.from].index,
          .to = straights[fit.to].index,
          .out = headsOf(fit.from, fit.key, fit.target)[fit.head].to.heading};
    };
    std::vector<Taken> taken{takenOf(fits[at - 1], chosen->meander)};
    const auto sameStretch = [&](const Fit& fit) {
      const std::size_t from = straights[fit.from].index;
      const std::size_t to = straights[fit.to].index;
      const Heading out =
          headsOf(fit.from, fit.key, fit.target)[fit.head].to.heading;
      return std::ranges::any_of(taken, [&](const Taken& one) {
        if (one.out != out) {
          return false;
        }
        const std::size_t low = std::max(from, one.from);
        const std::size_t high = std::min(to, one.to);
        return high > low &&
               2 * (high - low) > std::min(to - from, one.to - one.from);
      });
    };
    uint32_t builds = 0;
    for (; at < fits.size() &&
           result.alternatives.size() < options.alternatives &&
           builds < MAX_ALTERNATIVE_BUILDS * options.alternatives;
         ++at) {
      const Fit& fit = fits[at];
      if (sameStretch(fit)) {
        continue;
      }
      ++builds;
      auto made = buildFit(fit);
      if (!made.has_value() ||
          std::ranges::any_of(taken, [&](const Taken& one) {
            return !apart(one.box, made->meander);
          })) {
        continue;
      }
      taken.push_back(takenOf(fit, made->meander));
      result.alternatives.push_back(std::move(*made));
    }
    path = std::move(chosen->path);
    result.reached = true;
    result.inserted = true;
    result.lengthAfter = chosen->lengthAfter;
    result.meander = chosen->meander;
    return true;
  };

  // The coarse sweep first, about a hundred cells of the path to choose
  // from; where it finds nothing, every cell, with pairs no wider than
  // `SMALL_SPAN`, so that a small meander between close cells is tried as
  // well, as in the prototype's fallback. A meander along the diagonals is
  // the last resort: the layout keeps its meanders on the axes.
  const auto stride = static_cast<std::int64_t>(step);
  for (const bool diagonal : {false, true}) {
    if (sweep(stride, last - first, diagonal) ||
        (stride > 1 && sweep(1, SMALL_SPAN, diagonal))) {
      return result;
    }
  }
  return result;
}

std::vector<std::pair<const char*, uint32_t>>
MeanderResult::Refusals::said() const {
  const std::array<std::pair<const char*, uint32_t>, 8> all{{
      {"the two ends do not face each other", headingsApart},
      {"no pair far enough apart for the legs", tooClose},
      {"the stretch it replaces is already longer", noDepth},
      {"no room beside the way for the loop", outsideBox},
      {"the loop does not come back to the pair", noJoin},
      {"a cell of the loop is closed", blocked},
      {"the way would meet itself", selfCrossing},
      {"the rendering never landed in the tolerance", offLength},
  }};
  std::vector<std::pair<const char*, uint32_t>> found;
  for (const auto& one : all) {
    if (one.second != 0) {
      found.push_back(one);
    }
  }
  std::ranges::stable_sort(found, std::ranges::greater{},
                           &std::pair<const char*, uint32_t>::second);
  return found;
}

} // namespace mqt::scpd::routing
