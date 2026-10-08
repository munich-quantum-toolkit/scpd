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
#include "mqt-scpd/routing/SelfIntersection.hpp"

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

/// The first quarter turn toward the requested heading.
const Primitive* quarterTurnTo(const MovePrimitives& primitives,
                               const Heading from, const Heading exit) {
  for (const Primitive& move : primitives.of(from)) {
    if (move.exitHeading == exit) {
      return &move;
    }
  }
  return nullptr;
}

/// The length of one straight step of a heading, in cells.
double stepLength(const Heading heading) {
  return isDiagonal(heading) ? std::numbers::sqrt2 : 1.0;
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
/// of its end, where the next part starts, and its length.
struct DoglegPiece {
  Path cells;
  int64_t endX = 0;
  int64_t endY = 0;
  /// The length of the part as samplePath() renders it, in cells.
  double length = 0.0;
};

/// The swept cells of a primitive, including its start and end, tagged with
/// the entry heading and the primitive.
DoglegPiece pieceOf(const Primitive& primitive, const Heading heading) {
  DoglegPiece piece;
  piece.endX = primitive.dx;
  piece.endY = primitive.dy;
  piece.length = polylineLength(primitive.samples);
  const auto add = [&](const CellOffset& move) {
    piece.cells.push_back(
        {.x = static_cast<uint32_t>(static_cast<int32_t>(move.dx)),
         .y = static_cast<uint32_t>(static_cast<int32_t>(move.dy)),
         .heading = heading,
         .primitive = primitive.id});
  };
  for (const CellOffset& move : primitive.swept) {
    add(move);
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
  const DoglegPiece turnPiece = pieceOf(*turn, entry);
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
    const uint32_t width, const uint32_t height, const Heading couplerHeading,
    const CouplerDoglegOptions& options,
    const std::function<bool(uint32_t, uint32_t)>& anchorAllowed,
    const std::function<bool(const Path&)>& candidateAllowed) {
  if (couplerHeading >= NUM_HEADINGS) {
    throw std::invalid_argument("the coupler heading lies outside 0 to 7");
  }
  if (!std::isfinite(targetLength) || targetLength < 0.0) {
    throw std::invalid_argument("the target length is negative or not finite");
  }
  if (path.empty()) {
    return std::nullopt;
  }

  struct PathOption {
    double length = 0.0;
    std::vector<DoglegPiece> pieces;
  };
  std::map<uint16_t, std::vector<PathOption>> optionsByHeading;

  // The mandatory dogleg, and a second one when asked for. The lead and the
  // first turn start on the heading across the coupler heading.
  const Heading across = turned(couplerHeading, 2);
  const Heading afterFirstTurn =
      options.mirrored ? reverse(couplerHeading) : couplerHeading;
  const int firstTurn = options.mirrored ? 1 : -1;
  const DoglegGeometry dogleg =
      buildDogleg(primitives, across, firstTurn, options.straightLength);
  std::vector<DoglegPiece> prefix;
  if (options.leadStraight > 0) {
    // The straight run before the turn: the origin and one cell per step, on
    // the heading the turn starts on, tagged with the straight move. The run
    // ends on the cell the turn starts from.
    DoglegPiece lead;
    lead.cells = straightRun(primitives, {.x = 0, .y = 0, .heading = across},
                             options.leadStraight);
    const HeadingVector v = headingVector(across);
    lead.endX = static_cast<int64_t>(options.leadStraight) * v.dx;
    lead.endY = static_cast<int64_t>(options.leadStraight) * v.dy;
    lead.length =
        static_cast<double>(options.leadStraight) * stepLength(across);
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
  Heading searchHeading = afterFirstTurn;
  if (options.secondStraightLength > 0) {
    const int secondTurn = options.secondTurnReverse ? -firstTurn : firstTurn;
    const DoglegGeometry second = buildDogleg(
        primitives, afterFirstTurn, secondTurn, options.secondStraightLength);
    prefix.push_back(pieceOfDogleg(second));
    searchHeading = second.tip.heading;
  }
  double prefixLength = 0.0;
  for (const DoglegPiece& piece : prefix) {
    prefixLength += piece.length;
  }
  optionsByHeading[searchHeading].push_back(
      {.length = prefixLength, .pieces = prefix});

  const auto addOption = [&](const Heading target, const double length,
                             const std::vector<DoglegPiece>& pieces) {
    optionsByHeading[target].push_back({.length = length, .pieces = pieces});
  };
  const auto with = [&](std::initializer_list<DoglegPiece> extra) {
    std::vector<DoglegPiece> v = prefix;
    for (const DoglegPiece& p : extra) {
      v.push_back(p);
    }
    return v;
  };

  // Every one- and two-primitive continuation of the dogleg. The primitives
  // come in ascending identifier order, so the options of each heading follow
  // the lexicographic order of their primitive identifiers.
  for (const Primitive& first : primitives.of(searchHeading)) {
    const Heading intermediate = first.exitHeading;
    const DoglegPiece pieceOne = pieceOf(first, searchHeading);
    const double lengthOne = prefixLength + pieceOne.length;
    addOption(intermediate, lengthOne, with({pieceOne}));

    for (const Primitive& second : primitives.of(intermediate)) {
      const DoglegPiece pieceTwo = pieceOf(second, intermediate);
      addOption(second.exitHeading, lengthOne + pieceTwo.length,
                with({pieceOne, pieceTwo}));
    }
  }

  // The candidates: every cell of a straight run, with every option of its
  // heading, scored by the length mismatch of the path that would remain.
  // The lengths are those of the rendered path, so the length from a cell to
  // the end is the rendered length of the whole path minus the rendered
  // length up to the cell. The mismatches and the charges are whole numbers
  // of billionths of a cell, so that two lengths that differ only by rounding
  // give equal charges, unless a half billionth lies between them.
  constexpr double stepsPerCell = 1e9;
  struct Candidate {
    PathPoint cell;
    double mismatch = 0.0;
    double signedDiff = 0.0;
    double charge = 0.0;
    const PathOption* option = nullptr;
    std::size_t splitIndex = 0;
    /// The rank of the candidate: by its cell along the path, and on one cell
    /// by the order of the options.
    std::size_t order = 0;
    std::optional<bool> feasible = std::nullopt;
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
  for (std::size_t s = 0; s < segments.size(); ++s) {
    const PathSegment& segment = segments[s];
    // PathSegment::straight() reads false for a straight run of one cell, so
    // the tag decides. A run of one cell at the end of the path, such as the
    // end of a last turn, counts only with a point after it.
    if (!primitives.isStraight(segment.heading, segment.primitive) ||
        (!segment.straight() && s + 1 == segments.size())) {
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
            std::round(((overallLength - segment.lengthAt[i] + option.length) -
                        targetLength) *
                       stepsPerCell);
        candidates.push_back({.cell = cell,
                              .mismatch = std::abs(diff),
                              .signedDiff = diff,
                              .option = &option,
                              .splitIndex = own,
                              .order = candidates.size()});
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
  PathLoopScratch loopScratch;
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
    // The candidate's own point follows the dogleg. A last dogleg point on
    // the same cell would hide the own point from samplePath(), which keeps
    // the first of two consecutive points on one cell. The own point can be
    // the start of the arc of the next turn (see Path), so it stays and the
    // dogleg point goes. The collision test below still covers the step onto
    // the candidate's cell.
    if (simulated.size() > 1 && simulated.back().samePlace(cand.cell)) {
      simulated.pop_back();
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
    // Include the join when checking the new prefix against itself. The
    // remaining path was checked above; its clearance belongs to the caller.
    simulated.push_back(cand.cell);
    if (pathSelfIntersects(simulated, width, height, loopScratch)) {
      return false;
    }
    simulated.pop_back();
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
    for (Candidate& cand : candidates) {
      if (cand.feasible == false) {
        continue;
      }
      Path simulated;
      if (!tryCandidate(cand, simulated)) {
        cand.feasible = false;
        continue;
      }
      if (!cand.feasible.has_value() && candidateAllowed) {
        Path complete = simulated;
        complete.insert(complete.end(),
                        path.begin() +
                            static_cast<std::ptrdiff_t>(cand.splitIndex),
                        path.end());
        cand.feasible = candidateAllowed(complete);
        if (!*cand.feasible) {
          continue;
        }
      }
      cand.feasible = true;
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
  // Each sort key ends on the rank, which differs between any two
  // candidates. The order of equal mismatches and of equal charges therefore
  // follows the path, whatever the sort algorithm of the standard library.
  const auto byMismatch = [](const Candidate& a, const Candidate& b) {
    if (a.mismatch != b.mismatch) {
      return a.mismatch < b.mismatch;
    }
    return a.order < b.order;
  };
  const auto byCharge = [](const Candidate& a, const Candidate& b) {
    if (a.charge != b.charge) {
      return a.charge < b.charge;
    }
    return a.order < b.order;
  };

  // The best achievable mismatch, which scales the undershoot preference.
  std::ranges::sort(candidates, byMismatch);
  const std::optional<Choice> nearest = choose();
  if (!nearest) {
    return std::nullopt;
  }
  const double best = nearest->candidate->mismatch;
  // Prefer a slight undershoot over an equal overshoot: an overshoot only
  // loses to an undershoot within about twice the best mismatch of the
  // target, so a hard, blocked-in resonator is not dragged far away.
  for (Candidate& cand : candidates) {
    cand.charge =
        cand.signedDiff <= 0.0 ? -cand.signedDiff : cand.signedDiff + best;
  }
  std::ranges::sort(candidates, byCharge);
  const std::optional<Choice> chosen = choose();
  if (!chosen) {
    return std::nullopt;
  }

  // The insertion is the only step that can fail, and a failed insertion
  // leaves the path as it was. The erasure after it cannot fail.
  const auto split =
      path.begin() + static_cast<std::ptrdiff_t>(chosen->candidate->splitIndex);
  const auto spliced =
      path.insert(split, chosen->cells.begin(), chosen->cells.end());
  path.erase(path.begin(), spliced);
  return CouplerSplice{.anchor = path.front(),
                       .inAllowedArea = chosen->inAllowedArea};
}

} // namespace mqt::scpd::routing
