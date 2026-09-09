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
const Primitive* primitiveTo(const MovePrimitives& primitives, const Heading from,
                             const Heading exit) {
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

} // namespace

DoglegGeometry buildDogleg(const MovePrimitives& primitives, const Heading entry,
                           const int turnSign, const uint32_t straightLength) {
  if (turnSign != 1 && turnSign != -1) {
    throw std::invalid_argument("a dogleg turns one way or the other");
  }
  const Heading exit = turned(entry, 2 * turnSign);
  DoglegGeometry result;
  result.tip = {.x = 0, .y = 0, .heading = exit, .primitive = 0};

  const Primitive* turn = primitiveTo(primitives, entry, exit);
  if (turn == nullptr) {
    throw std::logic_error("the primitives hold no quarter turn for this heading");
  }
  result.cost = turn->cost;
  result.tip.primitive = turn->id;
  PathPoint current{.x = 0, .y = 0, .heading = entry, .primitive = turn->id};
  for (const CellOffset& move : turn->swept) {
    current.x = static_cast<uint32_t>(static_cast<int32_t>(move.dx));
    current.y = static_cast<uint32_t>(static_cast<int32_t>(move.dy));
    result.path.push_back(current);
  }
  result.tip.x = current.x;
  result.tip.y = current.y;

  const uint16_t straight = primitives.straight(exit);
  const HeadingVector v = headingVector(exit);
  for (uint32_t s = 0; s < straightLength; ++s) {
    result.tip.x = static_cast<uint32_t>(static_cast<int32_t>(result.tip.x) + v.dx);
    result.tip.y = static_cast<uint32_t>(static_cast<int32_t>(result.tip.y) + v.dy);
    result.tip.primitive = straight;
    result.path.push_back(result.tip);
  }
  result.cost += static_cast<double>(straightLength);
  return result;
}

std::optional<CouplerSplice> spliceCouplerDogleg(
    const MovePrimitives& primitives, const double targetLength, Path& path,
    Heading orientation, const CouplerDoglegOptions& options,
    const std::function<bool(uint32_t, uint32_t)>& anchorAllowed) {
  if (orientation >= NUM_HEADINGS) {
    throw std::invalid_argument("a coupler orientation is a heading");
  }
  if (path.empty()) {
    return std::nullopt;
  }

  struct PathOption {
    PathPoint end;
    double length = 0.0;
    std::vector<Path> pieces;
  };
  std::map<uint16_t, std::vector<PathOption>> optionsByHeading;

  // The mandatory dogleg, and a second one when asked for.
  const Heading orientationStart = turned(orientation, 2);
  if (options.mirrored) {
    orientation = reverse(orientation);
  }
  const int firstTurn = options.mirrored ? 1 : -1;
  DoglegGeometry dogleg = buildDogleg(primitives, orientationStart, firstTurn, options.straightLength);
  PathPoint tip = dogleg.tip;
  double initialCost = dogleg.cost;
  std::vector<Path> prefix = {dogleg.path};
  Heading searchHeading = orientation;
  if (options.secondStraightLength > 0) {
    const int secondTurn = options.secondTurnReverse ? -firstTurn : firstTurn;
    DoglegGeometry second = buildDogleg(primitives, orientation, secondTurn, options.secondStraightLength);
    prefix.push_back(second.path);
    initialCost += second.cost;
    tip = second.tip;
    searchHeading = second.tip.heading;
  }
  optionsByHeading[searchHeading].push_back({tip, initialCost, prefix});

  const auto addOption = [&](const Heading target, const PathPoint& end, const double cost,
                             const std::vector<Path>& pieces) {
    auto it = optionsByHeading.find(target);
    if (it == optionsByHeading.end() || cost < it->second.front().length) {
      optionsByHeading[target].push_back({end, cost, pieces});
    }
  };
  const auto with = [&](std::initializer_list<Path> extra) {
    std::vector<Path> v = prefix;
    for (const Path& p : extra) {
      v.push_back(p);
    }
    return v;
  };

  // Every one- and two-primitive continuation of the dogleg.
  for (const Primitive& first : primitives.of(searchHeading)) {
    const Heading intermediate = first.exitHeading;
    Path pathOne;
    PathPoint current{.x = 0, .y = 0, .heading = searchHeading, .primitive = first.id};
    for (const CellOffset& move : first.swept) {
      current.x = static_cast<uint32_t>(static_cast<int32_t>(move.dx));
      current.y = static_cast<uint32_t>(static_cast<int32_t>(move.dy));
      pathOne.push_back(current);
    }
    const double costOne = initialCost + first.cost;
    const PathPoint tipOne{.x = tip.x + current.x, .y = tip.y + current.y,
                           .heading = intermediate, .primitive = first.id};
    addOption(intermediate, tipOne, costOne, with({pathOne}));

    for (const Primitive& second : primitives.of(intermediate)) {
      Path combined;
      PathPoint c2{.x = 0, .y = 0, .heading = intermediate, .primitive = second.id};
      for (const CellOffset& move : second.swept) {
        c2.x = static_cast<uint32_t>(static_cast<int32_t>(move.dx));
        c2.y = static_cast<uint32_t>(static_cast<int32_t>(move.dy));
        combined.push_back(c2);
      }
      const PathPoint tipTwo{.x = tipOne.x + c2.x, .y = tipOne.y + c2.y,
                             .heading = second.exitHeading, .primitive = second.id};
      addOption(second.exitHeading, tipTwo, costOne + second.cost, with({pathOne, combined}));
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
      const std::size_t routingIndex = (it != firstIndexOf.end()) ? it->second : path.size();
      for (const PathOption& option : found->second) {
        const double diff = (overallLength - segment.lengthAt[i] + option.length) - targetLength;
        candidates.push_back({cell, std::abs(diff), diff, &option, routingIndex});
      }
    }
  }

  // Simulate a candidate's dogleg against the remaining path.
  const auto tryCandidate = [&](const Candidate& cand, Path& out) {
    Path simulated;
    uint32_t x0 = cand.cell.x;
    uint32_t y0 = cand.cell.y;
    for (const Path& piece : cand.option->pieces) {
      for (const PathPoint& move : piece) {
        simulated.push_back({.x = x0 + move.x, .y = y0 + move.y, .heading = move.heading, .primitive = move.primitive});
      }
      if (!simulated.empty()) {
        x0 = simulated.back().x;
        y0 = simulated.back().y;
      }
    }
    if (!simulated.empty()) {
      const uint32_t offsetX = cand.cell.x - simulated.back().x;
      const uint32_t offsetY = cand.cell.y - simulated.back().y;
      for (PathPoint& point : simulated) {
        point.x += offsetX;
        point.y += offsetY;
      }
    }
    // The last point must meet the path; every other point must not.
    for (std::size_t p = 0; p + 1 < simulated.size(); ++p) {
      for (std::size_t r = cand.splitIndex; r < path.size(); ++r) {
        if (simulated[p].samePlace(path[r])) {
          return false;
        }
      }
    }
    out = std::move(simulated);
    return true;
  };
  const auto allowed = [&](const Path& simulated) {
    return !anchorAllowed || simulated.empty() || anchorAllowed(simulated.front().x, simulated.front().y);
  };

  // The best achievable mismatch, which scales the undershoot preference.
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.mismatch < b.mismatch; });
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
    cand.mismatch = cand.signedDiff <= 0.0 ? -cand.signedDiff : cand.signedDiff + best;
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.mismatch < b.mismatch; });

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

  path.erase(path.begin(), path.begin() + static_cast<std::ptrdiff_t>(chosenSplit));
  path.insert(path.begin(), chosen.begin(), chosen.end());
  return CouplerSplice{.anchor = {.x = path.front().x,
                                 .y = path.front().y,
                                 .heading = chosenCell.heading,
                                 .primitive = chosenCell.primitive},
                       .inAllowedArea = inAllowedArea};
}

} // namespace mqt::scpd::routing
