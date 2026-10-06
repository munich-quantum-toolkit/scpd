/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CouplerInsertion.hpp"

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <map>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// The quarter turn of a heading toward an exit heading whose curve ends
/// closest to the direction of the exit heading. The primitives come in
/// ascending identifier order, so of equally close turns the one of lowest
/// identifier wins.
const Primitive* quarterTurnTo(const MovePrimitives& primitives,
                               const Heading from, const Heading exit) {
  const HeadingVector e = headingVector(exit);
  const Primitive* best = nullptr;
  double bestError = 0.0;
  for (const Primitive& p : primitives.of(from)) {
    if (p.exitHeading != exit) {
      continue;
    }
    // The angle between the last piece of the curve and the exit heading.
    double error = std::numbers::pi;
    if (p.samples.size() >= 2) {
      const Point& a = p.samples[p.samples.size() - 2];
      const Point& b = p.samples.back();
      const double tx = b.x() - a.x();
      const double ty = b.y() - a.y();
      error = std::abs(
          std::atan2((tx * e.dy) - (ty * e.dx), (tx * e.dx) + (ty * e.dy)));
    }
    if (best == nullptr || error < bestError) {
      best = &p;
      bestError = error;
    }
  }
  return best;
}

/// The length of one straight step of a heading, in cells.
double stepLength(const Heading heading) {
  return isDiagonal(heading) ? std::numbers::sqrt2 : 1.0;
}

/// The length of a move as samplePath() renders it: the curve of its samples,
/// pulled onto the end of the move.
double renderedMoveLength(const MovePrimitives& primitives,
                          const Heading heading, const Primitive& move) {
  // The move starts far enough from the origin that its end has no negative
  // coordinate.
  constexpr int32_t origin = 1 << 16;
  const auto at = [](const int32_t offset) {
    return static_cast<uint32_t>(origin + offset);
  };
  const Path path{
      {.x = at(0), .y = at(0), .heading = heading, .primitive = move.id},
      {.x = at(move.dx),
       .y = at(move.dy),
       .heading = move.exitHeading,
       .primitive = primitives.straight(move.exitHeading)}};
  return renderedLength(primitives, path);
}

uint64_t cellKey(const uint32_t x, const uint32_t y) {
  return (static_cast<uint64_t>(x) << 32U) | y;
}

/// The signed value of an offset that wraps around in an unsigned field.
int64_t signedOffset(const uint32_t offset) {
  return static_cast<int64_t>(static_cast<int32_t>(offset));
}

/// Whether two cells touch only at a corner.
bool isDiagonalStep(const PathPoint& a, const PathPoint& b) {
  const int64_t dx = static_cast<int64_t>(a.x) - static_cast<int64_t>(b.x);
  const int64_t dy = static_cast<int64_t>(a.y) - static_cast<int64_t>(b.y);
  return (dx == 1 || dx == -1) && (dy == 1 || dy == -1);
}

/// A part of a coupler's dogleg: its cells, relative to its start, the offset
/// of its end, where the next part starts, and its length. A primitive ends at
/// its offset Primitive::dx, Primitive::dy, which need not be its last swept
/// cell.
struct DoglegPiece {
  Path cells;
  int64_t endX = 0;
  int64_t endY = 0;
  /// The length of the part as samplePath() renders it, in cells.
  double length = 0.0;
};

/// The swept cells of a primitive that leaves a heading, tagged with the
/// heading and the primitive.
DoglegPiece pieceOf(const MovePrimitives& primitives,
                    const Primitive& primitive, const Heading heading) {
  DoglegPiece piece;
  piece.endX = primitive.dx;
  piece.endY = primitive.dy;
  piece.length = renderedMoveLength(primitives, heading, primitive);
  for (const CellOffset& move : primitive.swept) {
    piece.cells.push_back(
        {.x = static_cast<uint32_t>(static_cast<int32_t>(move.dx)),
         .y = static_cast<uint32_t>(static_cast<int32_t>(move.dy)),
         .heading = heading,
         .primitive = primitive.id});
  }
  return piece;
}

} // namespace

DoglegGeometry buildDogleg(const MovePrimitives& primitives,
                           const Heading entry, const int turnSign,
                           const uint32_t straightLength) {
  if (turnSign != 1 && turnSign != -1) {
    throw std::invalid_argument("a dogleg turns one way or the other");
  }
  const Heading exit = turned(entry, 2 * turnSign);
  const Primitive* turn = quarterTurnTo(primitives, entry, exit);
  if (turn == nullptr) {
    throw std::logic_error(
        "the primitives hold no quarter turn for this heading");
  }
  const DoglegPiece turnPiece = pieceOf(primitives, *turn, entry);
  DoglegGeometry result;
  result.path = turnPiece.cells;
  result.cost = turnPiece.length +
                (static_cast<double>(straightLength) * stepLength(exit));
  result.tip = {.x = static_cast<uint32_t>(static_cast<int32_t>(turn->dx)),
                .y = static_cast<uint32_t>(static_cast<int32_t>(turn->dy)),
                .heading = exit,
                .primitive = primitives.straight(exit)};
  const Path run = straightRun(primitives, result.tip, straightLength);
  result.path.insert(result.path.end(), run.begin() + 1, run.end());
  result.tip = run.back();
  return result;
}

std::optional<CouplerSplice> spliceCouplerDogleg(
    const MovePrimitives& primitives, const double targetLength, Path& path,
    const uint32_t width, const uint32_t height, Heading orientation,
    const CouplerDoglegOptions& options,
    const std::function<bool(uint32_t, uint32_t)>& anchorAllowed) {
  if (orientation >= NUM_HEADINGS) {
    throw std::invalid_argument("a coupler orientation is a heading");
  }
  if (path.empty()) {
    return std::nullopt;
  }

  struct PathOption {
    double length = 0.0;
    std::vector<DoglegPiece> pieces;
  };
  std::map<uint16_t, std::vector<PathOption>> optionsByHeading;

  // The mandatory dogleg, and a second one when asked for.
  const Heading orientationStart = turned(orientation, 2);
  if (options.mirrored) {
    orientation = reverse(orientation);
  }
  const int firstTurn = options.mirrored ? 1 : -1;
  const DoglegGeometry dogleg = buildDogleg(primitives, orientationStart,
                                            firstTurn, options.straightLength);
  std::vector<DoglegPiece> prefix;
  if (options.leadStraight > 0) {
    // The straight run before the turn: the origin and one cell per step, on
    // the heading the turn starts on, tagged with the straight move. The run
    // ends on the cell the turn starts from.
    DoglegPiece lead;
    lead.cells =
        straightRun(primitives, {.x = 0, .y = 0, .heading = orientationStart},
                    options.leadStraight);
    const HeadingVector v = headingVector(orientationStart);
    lead.endX = static_cast<int64_t>(options.leadStraight) * v.dx;
    lead.endY = static_cast<int64_t>(options.leadStraight) * v.dy;
    lead.length = static_cast<double>(options.leadStraight) *
                  stepLength(orientationStart);
    prefix.push_back(std::move(lead));
  }
  const auto pieceOfDogleg = [](const DoglegGeometry& geometry) {
    DoglegPiece piece;
    piece.cells = geometry.path;
    piece.endX = signedOffset(geometry.tip.x);
    piece.endY = signedOffset(geometry.tip.y);
    piece.length = geometry.cost;
    return piece;
  };
  prefix.push_back(pieceOfDogleg(dogleg));
  Heading searchHeading = orientation;
  if (options.secondStraightLength > 0) {
    const int secondTurn = options.secondTurnReverse ? -firstTurn : firstTurn;
    const DoglegGeometry second = buildDogleg(
        primitives, orientation, secondTurn, options.secondStraightLength);
    prefix.push_back(pieceOfDogleg(second));
    searchHeading = second.tip.heading;
  }
  double prefixLength = 0.0;
  for (const DoglegPiece& piece : prefix) {
    prefixLength += piece.length;
  }
  optionsByHeading[searchHeading].push_back(
      {.length = prefixLength, .pieces = prefix});

  // Mirrored moves render to the same length up to rounding. The margin
  // keeps rounding from deciding whether a continuation is shorter than the
  // first one of its heading.
  constexpr double roundingMargin = 1e-9;
  const auto addOption = [&](const Heading target, const double length,
                             const std::vector<DoglegPiece>& pieces) {
    auto it = optionsByHeading.find(target);
    if (it == optionsByHeading.end() ||
        length < it->second.front().length - roundingMargin) {
      optionsByHeading[target].push_back({.length = length, .pieces = pieces});
    }
  };
  const auto with = [&](std::initializer_list<DoglegPiece> extra) {
    std::vector<DoglegPiece> v = prefix;
    for (const DoglegPiece& p : extra) {
      v.push_back(p);
    }
    return v;
  };

  // Every one- and two-primitive continuation of the dogleg.
  for (const Primitive& first : primitives.of(searchHeading)) {
    const Heading intermediate = first.exitHeading;
    const DoglegPiece pieceOne = pieceOf(primitives, first, searchHeading);
    const double lengthOne = prefixLength + pieceOne.length;
    addOption(intermediate, lengthOne, with({pieceOne}));

    for (const Primitive& second : primitives.of(intermediate)) {
      const DoglegPiece pieceTwo = pieceOf(primitives, second, intermediate);
      addOption(second.exitHeading, lengthOne + pieceTwo.length,
                with({pieceOne, pieceTwo}));
    }
  }

  // The candidates: every cell of a straight run, with every option of its
  // heading, scored by the length mismatch of the path that would remain.
  // The lengths are those of the rendered path, so the length from a cell to
  // the end is the rendered length of the whole path minus the rendered
  // length up to the cell.
  struct Candidate {
    PathPoint cell;
    double mismatch = 0.0;
    double signedDiff = 0.0;
    const PathOption* option = nullptr;
    std::size_t splitIndex = 0;
  };
  std::vector<Candidate> candidates;
  Path rendered = path;
  std::vector<PathSegment> segments;
  const double overallLength = polylineLength(
      samplePath(primitives, rendered, rendered.front(), segments));
  // A cell of a straight run is a copy of a point of the path, and the
  // segments keep the order of the path. A cursor through the path therefore
  // finds the candidate's own point, also where an earlier point lies on the
  // same cell.
  std::size_t own = 0;
  for (const PathSegment& segment : segments) {
    if (!segment.straight()) {
      continue;
    }
    const auto found = optionsByHeading.find(segment.heading);
    if (found == optionsByHeading.end()) {
      continue;
    }
    for (std::size_t i = 0; i < segment.cells.size(); ++i) {
      const PathPoint& cell = segment.cells[i];
      while (own < path.size() && path[own] != cell) {
        ++own;
      }
      for (const PathOption& option : found->second) {
        const double diff =
            (overallLength - segment.lengthAt[i] + option.length) -
            targetLength;
        candidates.push_back({.cell = cell,
                              .mismatch = std::abs(diff),
                              .signedDiff = diff,
                              .option = &option,
                              .splitIndex = own});
      }
    }
  }

  // The indices of the points on each cell: pairs of a cell key and an index
  // in ascending order, so that the indices of one cell are adjacent and
  // ascend.
  std::vector<std::pair<uint64_t, std::size_t>> cellIndex;
  cellIndex.reserve(path.size());
  for (std::size_t r = 0; r < path.size(); ++r) {
    cellIndex.emplace_back(cellKey(path[r].x, path[r].y), r);
  }
  std::ranges::sort(cellIndex);
  const auto indicesOf = [&](const uint32_t x, const uint32_t y) {
    return std::ranges::equal_range(cellIndex, cellKey(x, y), {},
                                    &std::pair<uint64_t, std::size_t>::first);
  };

  // Simulate a candidate's dogleg against the remaining path. Each piece
  // starts at the end of the piece before it, and the end of the last piece
  // is the candidate's cell, where the dogleg joins the path. The cells are
  // signed, so that a dogleg reaching past the edge of the grid is rejected.
  const auto gridWidth = static_cast<int64_t>(width);
  const auto gridHeight = static_cast<int64_t>(height);
  const auto tryCandidate = [&](const Candidate& cand, Path& out) {
    int64_t endX = 0;
    int64_t endY = 0;
    for (const DoglegPiece& piece : cand.option->pieces) {
      endX += piece.endX;
      endY += piece.endY;
    }
    int64_t x0 = static_cast<int64_t>(cand.cell.x) - endX;
    int64_t y0 = static_cast<int64_t>(cand.cell.y) - endY;
    Path simulated;
    std::size_t lastPieceStart = 0;
    for (const DoglegPiece& piece : cand.option->pieces) {
      lastPieceStart = simulated.size();
      for (const PathPoint& move : piece.cells) {
        const int64_t x = x0 + signedOffset(move.x);
        const int64_t y = y0 + signedOffset(move.y);
        if (x < 0 || y < 0 || x >= gridWidth || y >= gridHeight) {
          return false;
        }
        const PathPoint point{.x = static_cast<uint32_t>(x),
                              .y = static_cast<uint32_t>(y),
                              .heading = move.heading,
                              .primitive = move.primitive};
        // A piece that starts on the last cell of the piece before it takes
        // that cell over. A turn then keeps its start cell, as in a routed
        // path, and no two consecutive points of the dogleg share a cell.
        if (lastPieceStart > 0 && simulated.size() == lastPieceStart &&
            simulated.back().samePlace(point)) {
          simulated.back() = point;
          --lastPieceStart;
          continue;
        }
        simulated.push_back(point);
      }
      x0 += piece.endX;
      y0 += piece.endY;
    }
    // Only the last piece may meet the path, and only on the cell where the
    // dogleg joins it; the last piece can sweep a cell past its end. Two
    // diagonal steps can also cross inside a 2 by 2 block without sharing a
    // cell, so no diagonal step of the dogleg, including the step onto the
    // joining cell, may cross one of the path. A step of the path crosses
    // the diagonal step from a to b where it joins the two other cells of
    // their block.
    const std::size_t split = cand.splitIndex;
    for (std::size_t p = 0; p < simulated.size(); ++p) {
      const PathPoint& a = simulated[p];
      const PathPoint& b =
          p + 1 < simulated.size() ? simulated[p + 1] : cand.cell;
      const bool joins = p >= lastPieceStart && a.samePlace(cand.cell);
      if (!joins) {
        const auto onCell = indicesOf(a.x, a.y);
        if (!onCell.empty() && onCell.back().second >= split) {
          return false;
        }
      }
      if (isDiagonalStep(a, b)) {
        const PathPoint other{.x = b.x, .y = a.y};
        for (const auto& entry : indicesOf(a.x, b.y)) {
          const std::size_t r = entry.second;
          if ((r >= split && r + 1 < path.size() &&
               path[r + 1].samePlace(other)) ||
              (r > split && path[r - 1].samePlace(other))) {
            return false;
          }
        }
      }
    }
    out = std::move(simulated);
    return true;
  };

  // The rule that picks a candidate, in the order of the candidates: the
  // first collision-free one whose anchor the filter allows, or else the
  // first collision-free one.
  struct Choice {
    const Candidate* candidate = nullptr;
    Path cells;
    bool inAllowedArea = true;
  };
  const auto choose = [&]() -> std::optional<Choice> {
    std::optional<Choice> fallback;
    for (const Candidate& cand : candidates) {
      Path simulated;
      if (!tryCandidate(cand, simulated)) {
        continue;
      }
      if (!anchorAllowed || simulated.empty() ||
          anchorAllowed(simulated.front().x, simulated.front().y)) {
        return Choice{.candidate = &cand,
                      .cells = std::move(simulated),
                      .inAllowedArea = true};
      }
      if (!fallback) {
        fallback = Choice{.candidate = &cand,
                          .cells = std::move(simulated),
                          .inAllowedArea = false};
      }
    }
    return fallback;
  };
  const auto byMismatch = [](const Candidate& a, const Candidate& b) {
    return a.mismatch < b.mismatch;
  };

  // The best achievable mismatch, which scales the undershoot preference.
  // The sorts are stable, so among candidates of equal mismatch the one
  // earliest along the path wins, whatever the standard library.
  std::ranges::stable_sort(candidates, byMismatch);
  const std::optional<Choice> nearest = choose();
  if (!nearest) {
    return std::nullopt;
  }
  const double best = nearest->candidate->mismatch;
  // Prefer a slight undershoot over an equal overshoot: an overshoot only
  // loses to an undershoot within about twice the best mismatch of the
  // target, so a hard, blocked-in resonator is not dragged far away.
  for (Candidate& cand : candidates) {
    cand.mismatch =
        cand.signedDiff <= 0.0 ? -cand.signedDiff : cand.signedDiff + best;
  }
  std::ranges::stable_sort(candidates, byMismatch);
  const std::optional<Choice> chosen = choose();
  if (!chosen) {
    return std::nullopt;
  }

  const Candidate& winner = *chosen->candidate;
  path.erase(path.begin(),
             path.begin() + static_cast<std::ptrdiff_t>(winner.splitIndex));
  path.insert(path.begin(), chosen->cells.begin(), chosen->cells.end());
  return CouplerSplice{.anchor = {.x = path.front().x,
                                  .y = path.front().y,
                                  .heading = winner.cell.heading,
                                  .primitive = winner.cell.primitive},
                       .inAllowedArea = chosen->inAllowedArea};
}

} // namespace mqt::scpd::routing
