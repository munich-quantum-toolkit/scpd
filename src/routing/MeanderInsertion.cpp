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

/// The quarter turn of the primitives from one cardinal heading onto the
/// next but one, the one that ends a radius along each of the two.
const Primitive* quarterTurn(const MovePrimitives& primitives,
                             const Heading from, const Heading to) {
  const auto radius = static_cast<int64_t>(primitives.minRadius());
  const HeadingVector in = headingVector(from);
  const HeadingVector out = headingVector(to);
  for (const Primitive& move : primitives.of(from)) {
    if (move.exitHeading == to && move.dx == radius * (in.dx + out.dx) &&
        move.dy == radius * (in.dy + out.dy)) {
      return &move;
    }
  }
  return nullptr;
}

/// One rectangular loop from a point on a cardinal heading to a point on the
/// opposite heading beside it: out along the heading, a quarter turn, across
/// to the other point, a quarter turn, and back to it. Its depth is what the
/// wanted length makes it.
///
/// The prototype's `compute_meander_with_sanitized_coordinates_*` with one
/// meander, which is the one shape its insertion ever accepts, and its
/// `construct_*_meander_path`.
std::optional<Piece> buildLoop(const MovePrimitives& primitives, const Spot& a,
                               const Spot& b, const double wanted,
                               const Box& box, const uint32_t safety,
                               const uint32_t minStraight) {
  if (isDiagonal(a.heading) || b.heading != reverse(a.heading)) {
    return std::nullopt;
  }
  const auto radius = static_cast<int64_t>(primitives.minRadius());
  const HeadingVector along = headingVector(a.heading);
  const int64_t dx = b.x - a.x;
  const int64_t dy = b.y - a.y;
  // How far b lies along a's heading, and how far across it.
  const int64_t forward = (dx * along.dx) + (dy * along.dy);
  Heading across = a.heading;
  int64_t span = 0;
  for (const int eighths : {2, -2}) {
    const Heading candidate = turned(a.heading, eighths);
    const HeadingVector v = headingVector(candidate);
    const int64_t side = (dx * v.dx) + (dy * v.dy);
    if (side > 0) {
      across = candidate;
      span = side;
    }
  }
  const int64_t run = span - (2 * radius);
  if (span <= 0 || run < static_cast<int64_t>(minStraight)) {
    return std::nullopt;
  }
  const Primitive* first = quarterTurn(primitives, a.heading, across);
  const Primitive* second = quarterTurn(primitives, across, b.heading);
  if (first == nullptr || second == nullptr) {
    return std::nullopt;
  }
  // The loop is out + first turn + run + second turn + back long, and back
  // is out less the forward distance. Rounded up, so that the length is not
  // undershot.
  const double out =
      std::ceil((wanted - static_cast<double>(run) - first->cost -
                 second->cost + static_cast<double>(forward)) /
                2.0);
  if (out < 0.0) {
    return std::nullopt;
  }
  const auto outSteps = static_cast<int64_t>(out);
  const int64_t back = outSteps - forward;
  if (back < 0) {
    return std::nullopt;
  }
  // The far edge of the loop, `safety` cells short of the box.
  const int64_t farX = a.x + ((outSteps + radius) * along.dx);
  const int64_t farY = a.y + ((outSteps + radius) * along.dy);
  const auto margin = static_cast<int64_t>(safety);
  if (farX < box.minX + margin || farX > box.maxX - margin ||
      farY < box.minY + margin || farY > box.maxY - margin) {
    return std::nullopt;
  }

  Piece loop;
  Spot at = a;
  appendStraight(loop, at, primitives, outSteps);
  appendMove(loop, at, *first);
  at = after(at, *first);
  appendStraight(loop, at, primitives, run);
  appendMove(loop, at, *second);
  at = after(at, *second);
  appendStraight(loop, at, primitives, back);
  if (at.x != b.x || at.y != b.y) {
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
  if (result.lengthBefore >= required) {
    result.reached = true;
    return result;
  }
  double wanted = required;
  if (wanted - result.lengthBefore < options.smallDeficitMargin) {
    wanted += options.smallDeficitMargin;
  }
  const double deficit = wanted - result.lengthBefore;

  // The cells of the straight runs. The sampler has just cut the path into
  // its segments, so each cell is found again in the path in order.
  std::vector<Straight> straights;
  std::size_t cursor = 0;
  for (const PathSegment& segment : segments) {
    if (!segment.straight) {
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
  const std::size_t start =
      count > options.startMargin ? options.startMargin : 0;
  const Box box{.minX = options.box.minX,
                .maxX = options.box.maxX,
                .minY = options.box.minY,
                .maxY = options.box.maxY};

  // The chains of a cell depend on which headings a pair reaches out along,
  // and every cell is the near end of many pairs and the far end of many
  // more, so they are built once per cell and target set.
  using Chains = std::vector<Chain>;
  std::vector<std::array<std::optional<Chains>, 4>> heads(count);
  std::vector<std::array<std::optional<Chains>, 4>> tails(count);
  const auto headsOf = [&](const std::size_t at, const std::size_t key,
                           const Targets& target) -> const Chains& {
    auto& slot = heads[at][key];
    if (!slot.has_value()) {
      slot = chainsFrom(primitives, straights[at].spot, target, box);
    }
    return *slot;
  };
  const auto tailsOf = [&](const std::size_t at, const std::size_t key,
                           const Targets& target) -> const Chains& {
    auto& slot = tails[at][key];
    if (!slot.has_value()) {
      slot = chainsInto(primitives, straights[at].spot, target, box);
    }
    return *slot;
  };

  // The head, the loop and the tail as one run of cells, or nothing when the
  // loop does not fit or a cell of it may not be entered.
  const auto assemble = [&](const Chain& head, const Chain& tail,
                            const double loopLength) -> std::optional<Piece> {
    const auto loop = buildLoop(primitives, head.to, tail.from, loopLength, box,
                                options.safety, options.minStraightLength);
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
          return std::nullopt;
        }
        piece.push_back(spot);
      }
    }
    return piece;
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
      return {};
    }
    return spliced;
  };

  // Take a placement: the loop is built for the length it should have, and
  // where the rendering falls short of the requirement by a fraction of a
  // cell per bend, it is built again that much deeper.
  const auto take = [&](const Straight& g0, const Straight& g1,
                        const Chain& head, const Chain& tail,
                        double loopLength) {
    Path candidate;
    double after = 0.0;
    for (int attempt = 0; attempt < 4; ++attempt) {
      const auto piece = assemble(head, tail, loopLength);
      candidate = piece.has_value() ? splice(g0, g1, *piece) : Path{};
      if (candidate.empty()) {
        return false;
      }
      after = renderedLength(primitives, candidate);
      if (after >= required) {
        break;
      }
      loopLength += (required - after) + 1.0;
    }
    if (after < required) {
      return false;
    }
    path = std::move(candidate);
    result.reached = true;
    result.inserted = true;
    result.lengthAfter = after;
    return true;
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

  struct Fit {
    std::size_t from = 0;
    std::size_t to = 0;
    const Chain* head = nullptr;
    const Chain* tail = nullptr;
    double loopLength = 0.0;
    double score = 0.0;
  };
  std::vector<Fit> fits;

  // The pairs are tried by their span, the smallest first, and within one
  // span from the target end of the path, so that the smallest loop goes as
  // near the target as it fits. The source end is where the coupler
  // insertion cuts the way back to the target length and where the feedline
  // runs, so a loop there is cut in two or in the feedline's way. The
  // prototype walks from the source; its later phases place the loop again
  // from the coupler.
  const auto first = static_cast<std::int64_t>(start);
  const auto last = static_cast<std::int64_t>(count) - 1;
  const auto stride = static_cast<std::int64_t>(step);
  for (std::int64_t span = stride; span <= last - first; span += stride) {
    for (std::int64_t i = last; i - span >= first; i -= stride) {
      const std::int64_t j = i - span;
      const Straight& g0 = straights[static_cast<std::size_t>(j)];
      const Straight& g1 = straights[static_cast<std::size_t>(i)];
      // A loop reaches out across the pair: across a pair that lies along
      // x it runs along y, and the other way round.
      const int64_t apartX = std::abs(g1.spot.x - g0.spot.x);
      const int64_t apartY = std::abs(g1.spot.y - g0.spot.y);
      Targets target{};
      std::size_t key = 0;
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
      // What replaces the cells between the pair has to be as long as they
      // were, plus the deficit.
      const double lengthToAdd = (g1.lengthAt - g0.lengthAt) + deficit;
      for (const Chain& head :
           headsOf(static_cast<std::size_t>(j), key, target)) {
        for (const Chain& tail :
             tailsOf(static_cast<std::size_t>(i), key, target)) {
          ++result.candidates;
          if (tail.from.heading != reverse(head.to.heading)) {
            continue;
          }
          const double loopLength = lengthToAdd - head.cost - tail.cost;
          if (!price) {
            if (take(g0, g1, head, tail, loopLength)) {
              result.fits = 1;
              return result;
            }
            continue;
          }
          const auto piece = assemble(head, tail, loopLength);
          if (!piece.has_value()) {
            continue;
          }
          fits.push_back({.from = static_cast<std::size_t>(j),
                          .to = static_cast<std::size_t>(i),
                          .head = &head,
                          .tail = &tail,
                          .loopLength = loopLength,
                          .score = scoreOf(g0, g1, *piece)});
        }
      }
    }
  }

  result.fits = static_cast<uint32_t>(fits.size());
  std::ranges::stable_sort(fits, {}, &Fit::score);
  for (const Fit& fit : fits) {
    if (take(straights[fit.from], straights[fit.to], *fit.head, *fit.tail,
             fit.loopLength)) {
      return result;
    }
  }
  return result;
}

} // namespace mqt::scpd::routing
