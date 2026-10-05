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
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// The primitive of a heading with a given exit heading, the one of lowest
/// identifier when several exist.
const Primitive* primitiveTo(const MovePrimitives& primitives,
                             const Heading from, const Heading exit) {
  for (const Primitive& p : primitives.of(from)) {
    if (p.exitHeading == exit) {
      return &p;
    }
  }
  return nullptr;
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

/// Whether the step from @p c to @p d crosses the diagonal step from @p a to
/// @p b inside their 2 by 2 block, that is, whether it joins the two other
/// cells of the block.
bool crossesDiagonalStep(const PathPoint& a, const PathPoint& b,
                         const PathPoint& c, const PathPoint& d) {
  return (c.x == a.x && c.y == b.y && d.x == b.x && d.y == a.y) ||
         (c.x == b.x && c.y == a.y && d.x == a.x && d.y == b.y);
}

/// A part of a coupler's dogleg: its cells, relative to its start, and the
/// offset of its end, where the next part starts. A primitive ends at its
/// offset Primitive::dx, Primitive::dy, which need not be its last swept cell.
struct DoglegPiece {
  Path cells;
  int64_t endX = 0;
  int64_t endY = 0;
};

/// The swept cells of a primitive that leaves a heading, tagged with the
/// heading and the primitive.
DoglegPiece pieceOf(const Primitive& primitive, const Heading heading) {
  DoglegPiece piece;
  piece.endX = primitive.dx;
  piece.endY = primitive.dy;
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
  DoglegGeometry result;
  result.tip = {.x = 0, .y = 0, .heading = exit, .primitive = 0};

  const Primitive* turn = primitiveTo(primitives, entry, exit);
  if (turn == nullptr) {
    throw std::logic_error(
        "the primitives hold no quarter turn for this heading");
  }
  result.cost = turn->cost;
  result.tip.primitive = turn->id;
  result.path = pieceOf(*turn, entry).cells;
  result.tip.x = static_cast<uint32_t>(static_cast<int32_t>(turn->dx));
  result.tip.y = static_cast<uint32_t>(static_cast<int32_t>(turn->dy));

  const uint16_t straight = primitives.straight(exit);
  const HeadingVector v = headingVector(exit);
  for (uint32_t s = 0; s < straightLength; ++s) {
    result.tip.x =
        static_cast<uint32_t>(static_cast<int32_t>(result.tip.x) + v.dx);
    result.tip.y =
        static_cast<uint32_t>(static_cast<int32_t>(result.tip.y) + v.dy);
    result.tip.primitive = straight;
    result.path.push_back(result.tip);
  }
  result.cost += static_cast<double>(straightLength);
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
  double initialCost = dogleg.cost;
  std::vector<DoglegPiece> prefix;
  if (options.leadStraight > 0) {
    // The straight run before the turn: the origin and one cell per step, on
    // the heading the turn starts on, tagged with the straight move. The run
    // ends on the cell the turn starts from.
    DoglegPiece lead;
    const HeadingVector v = headingVector(orientationStart);
    const uint16_t straight = primitives.straight(orientationStart);
    for (uint32_t s = 0; s <= options.leadStraight; ++s) {
      lead.cells.push_back(
          {.x = static_cast<uint32_t>(static_cast<int32_t>(s) * v.dx),
           .y = static_cast<uint32_t>(static_cast<int32_t>(s) * v.dy),
           .heading = orientationStart,
           .primitive = straight});
    }
    lead.endX = static_cast<int64_t>(options.leadStraight) * v.dx;
    lead.endY = static_cast<int64_t>(options.leadStraight) * v.dy;
    prefix.push_back(std::move(lead));
    initialCost += static_cast<double>(options.leadStraight);
  }
  const auto pieceOfDogleg = [](const DoglegGeometry& geometry) {
    DoglegPiece piece;
    piece.cells = geometry.path;
    piece.endX = signedOffset(geometry.tip.x);
    piece.endY = signedOffset(geometry.tip.y);
    return piece;
  };
  prefix.push_back(pieceOfDogleg(dogleg));
  Heading searchHeading = orientation;
  if (options.secondStraightLength > 0) {
    const int secondTurn = options.secondTurnReverse ? -firstTurn : firstTurn;
    const DoglegGeometry second = buildDogleg(
        primitives, orientation, secondTurn, options.secondStraightLength);
    prefix.push_back(pieceOfDogleg(second));
    initialCost += second.cost;
    searchHeading = second.tip.heading;
  }
  optionsByHeading[searchHeading].push_back(
      {.length = initialCost, .pieces = prefix});

  const auto addOption = [&](const Heading target, const double cost,
                             const std::vector<DoglegPiece>& pieces) {
    auto it = optionsByHeading.find(target);
    if (it == optionsByHeading.end() || cost < it->second.front().length) {
      optionsByHeading[target].push_back({.length = cost, .pieces = pieces});
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
    const DoglegPiece pieceOne = pieceOf(first, searchHeading);
    const double costOne = initialCost + first.cost;
    addOption(intermediate, costOne, with({pieceOne}));

    for (const Primitive& second : primitives.of(intermediate)) {
      addOption(second.exitHeading, costOne + second.cost,
                with({pieceOne, pieceOf(second, intermediate)}));
    }
  }

  // The candidates: every cell of a straight run, with every option of its
  // heading, scored by the length mismatch of the path that would remain.
  struct Candidate {
    PathPoint cell;
    double mismatch = 0.0;
    double signedDiff = 0.0;
    const PathOption* option = nullptr;
    std::size_t splitIndex = 0;
  };
  std::vector<Candidate> candidates;
  const SegmentedPath segmented = reconstructSegments(primitives, path);
  if (segmented.segments.empty()) {
    return std::nullopt;
  }
  const double overallLength = segmented.segments.back().lengthAt.back();
  std::unordered_map<uint64_t, std::size_t> firstIndexOf;
  firstIndexOf.reserve(path.size() * 2);
  for (std::size_t r = 0; r < path.size(); ++r) {
    firstIndexOf.emplace(cellKey(path[r].x, path[r].y), r);
  }
  for (const PathSegment& segment : segmented.segments) {
    if (!segment.straight) {
      continue;
    }
    const auto found = optionsByHeading.find(segment.heading);
    if (found == optionsByHeading.end()) {
      continue;
    }
    for (std::size_t i = 0; i < segment.cells.size(); ++i) {
      const PathPoint& cell = segment.cells[i];
      const auto it = firstIndexOf.find(cellKey(cell.x, cell.y));
      const std::size_t routingIndex =
          (it != firstIndexOf.end()) ? it->second : path.size();
      for (const PathOption& option : found->second) {
        const double diff =
            (overallLength - segment.lengthAt[i] + option.length) -
            targetLength;
        candidates.push_back({.cell = cell,
                              .mismatch = std::abs(diff),
                              .signedDiff = diff,
                              .option = &option,
                              .splitIndex = routingIndex});
      }
    }
  }

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
    // joining cell, may cross one of the path.
    for (std::size_t p = 0; p < simulated.size(); ++p) {
      const PathPoint& a = simulated[p];
      const PathPoint& b =
          p + 1 < simulated.size() ? simulated[p + 1] : cand.cell;
      const bool joins = p >= lastPieceStart && a.samePlace(cand.cell);
      const bool diagonal = isDiagonalStep(a, b);
      for (std::size_t r = cand.splitIndex; r < path.size(); ++r) {
        if (!joins && a.samePlace(path[r])) {
          return false;
        }
        if (diagonal && r + 1 < path.size() &&
            crossesDiagonalStep(a, b, path[r], path[r + 1])) {
          return false;
        }
      }
    }
    out = std::move(simulated);
    return true;
  };
  const auto allowed = [&](const Path& simulated) {
    return !anchorAllowed || simulated.empty() ||
           anchorAllowed(simulated.front().x, simulated.front().y);
  };

  // The best achievable mismatch, which scales the undershoot preference.
  // The sorts are stable, so among candidates of equal mismatch the one
  // earliest along the path wins, whatever the standard library.
  std::ranges::stable_sort(candidates,
                           [](const Candidate& a, const Candidate& b) {
                             return a.mismatch < b.mismatch;
                           });
  double best = 0.0;
  {
    Path discard;
    bool got = false;
    for (const Candidate& cand : candidates) {
      if (tryCandidate(cand, discard) && allowed(discard)) {
        best = cand.mismatch;
        got = true;
        break;
      }
    }
    if (!got) {
      for (const Candidate& cand : candidates) {
        if (tryCandidate(cand, discard)) {
          best = cand.mismatch;
          break;
        }
      }
    }
  }
  // Prefer a slight undershoot over an equal overshoot: an overshoot only
  // loses to an undershoot within about twice the best mismatch of the
  // target, so a hard, blocked-in resonator is not dragged far away.
  for (Candidate& cand : candidates) {
    cand.mismatch =
        cand.signedDiff <= 0.0 ? -cand.signedDiff : cand.signedDiff + best;
  }
  std::ranges::stable_sort(candidates,
                           [](const Candidate& a, const Candidate& b) {
                             return a.mismatch < b.mismatch;
                           });

  bool found = false;
  bool foundFallback = false;
  Path chosen;
  Path fallback;
  std::size_t chosenSplit = 0;
  std::size_t fallbackSplit = 0;
  PathPoint chosenCell;
  PathPoint fallbackCell;
  for (const Candidate& cand : candidates) {
    Path simulated;
    if (!tryCandidate(cand, simulated)) {
      continue;
    }
    if (!allowed(simulated)) {
      if (!foundFallback) {
        foundFallback = true;
        fallback = std::move(simulated);
        fallbackSplit = cand.splitIndex;
        fallbackCell = cand.cell;
      }
      continue;
    }
    found = true;
    chosen = std::move(simulated);
    chosenSplit = cand.splitIndex;
    chosenCell = cand.cell;
    break;
  }
  bool inAllowedArea = true;
  if (!found && foundFallback) {
    found = true;
    inAllowedArea = false;
    chosen = std::move(fallback);
    chosenSplit = fallbackSplit;
    chosenCell = fallbackCell;
  }
  if (!found) {
    return std::nullopt;
  }

  path.erase(path.begin(),
             path.begin() + static_cast<std::ptrdiff_t>(chosenSplit));
  path.insert(path.begin(), chosen.begin(), chosen.end());
  return CouplerSplice{.anchor = {.x = path.front().x,
                                  .y = path.front().y,
                                  .heading = chosenCell.heading,
                                  .primitive = chosenCell.primitive},
                       .inAllowedArea = inAllowedArea};
}

} // namespace mqt::scpd::routing
