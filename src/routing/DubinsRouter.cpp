/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/DubinsRouter.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/BucketQueue.hpp"
#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// The step costs of the search, in hundredths of a cell.
constexpr uint32_t STRAIGHT_COST = 100;
constexpr uint32_t DIAGONAL_COST = 141;
constexpr uint32_t PENALTY_SCALE = 100;

/// Start loading a node record the next pop will need, so that the
/// record is in the cache by the time the expansion reads it.
inline void prefetchForWrite([[maybe_unused]] const void* address) {
#if defined(__GNUC__) || defined(__clang__)
  __builtin_prefetch(address, 1, 3);
#endif
}

constexpr uint32_t octile(const uint32_t x0, const uint32_t y0,
                          const uint32_t x1, const uint32_t y1) {
  const uint32_t dx = x0 > x1 ? x0 - x1 : x1 - x0;
  const uint32_t dy = y0 > y1 ? y0 - y1 : y1 - y0;
  const uint32_t minD = std::min(dx, dy);
  const uint32_t maxD = std::max(dx, dy);
  return ((maxD - minD) * STRAIGHT_COST) + (minD * DIAGONAL_COST);
}

/// The sign of the cross product of a direction with the vector from a
/// center to a point: 1 when the point is left of the direction, -1 right, 0
/// on the line.
int sideSign(const int64_t dx, const int64_t dy, const int64_t px,
             const int64_t py, const int64_t cx, const int64_t cy) {
  const int64_t cross = (dx * (py - cy)) - (dy * (px - cx));
  return (cross > 0 ? 1 : 0) - (cross < 0 ? 1 : 0);
}

/// Refuses a static proximity penalty that does not fit the seven low bits
/// of the working byte.
void checkProximityPenalty(const uint8_t penalty) {
  if (penalty > 127U) {
    throw std::invalid_argument("a proximity penalty is at most 127");
  }
}

/// Refuses a source or a target whose heading is not one of the eight
/// headings of the router grid.
void checkHeadings(const RoutingObjective& objective) {
  if (objective.source.heading >= NUM_HEADINGS ||
      objective.target.heading >= NUM_HEADINGS) {
    throw std::invalid_argument(
        "the source and the target of a route need a heading from 0 to 7");
  }
}

} // namespace

DubinsRouter::DubinsRouter(std::shared_ptr<const MovePrimitives> primitives,
                           SearchScratch& scratch, const SearchParams params)
    : movePrimitives(std::move(primitives)), searchScratch(&scratch),
      searchParams(params), gridWidth(scratch.width()),
      gridHeight(scratch.height()) {
  if (movePrimitives == nullptr) {
    throw std::invalid_argument("a router needs primitives");
  }
  if (movePrimitives->minRadius() < MIN_BEND_RADIUS ||
      movePrimitives->minRadius() > MAX_BEND_RADIUS) {
    throw std::invalid_argument(
        "a router accepts a bend radius from 2 to 23 cells");
  }
  if (gridWidth == 0 || gridHeight == 0 || gridWidth >= 65536U ||
      gridHeight >= 65536U) {
    throw std::invalid_argument(
        "a router grid needs 1 to 65535 cells per axis");
  }
  if (searchParams.minRadius != movePrimitives->minRadius()) {
    throw std::invalid_argument(
        "the bend radius must be the one the primitives were built with");
  }
  staticPenalties.assign(cells(), 0);
  buildTables(searchParams.bendPenalty);
}

std::size_t DubinsRouter::heldBytes() const {
  const auto capacityBytes = [](const auto& container) {
    return container.capacity() * sizeof(container[0]);
  };
  std::size_t bytes = sizeof(DubinsRouter) + capacityBytes(staticPenalties) +
                      capacityBytes(packedGrid) + capacityBytes(crossingRules) +
                      capacityBytes(exemptCells) +
                      capacityBytes(crossingSideCells) +
                      capacityBytes(distanceField) + loopScratch.heldBytes() +
                      open.heldBytes() + constraints.heldBytes();
  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    bytes +=
        capacityBytes(trie[heading]) + capacityBytes(triePrimitives[heading]);
  }
  for (const std::vector<uint32_t>& bucket : fieldBuckets) {
    bytes += capacityBytes(bucket);
  }
  return bytes;
}

void DubinsRouter::setParams(const SearchParams& params) {
  if (params.minRadius != movePrimitives->minRadius()) {
    throw std::invalid_argument(
        "the bend radius must be the one the primitives were built with");
  }
  if (searchParams.bendPenalty != params.bendPenalty) {
    buildTables(params.bendPenalty);
  }
  // Last, so that a failed rebuild keeps the previous parameters with the
  // tables they belong to.
  searchParams = params;
}

// --- Grids -----------------------------------------------------------------

void DubinsRouter::attachObstacles(const grid::BitGrid* obstacles) {
  if (obstacles != nullptr &&
      (obstacles->width() != gridWidth || obstacles->height() != gridHeight)) {
    throw std::invalid_argument(
        "the obstacle grid does not match the router grid");
  }
  obstacleMask = obstacles;
}

void DubinsRouter::attachCorridor(const grid::BitGrid* corridor) {
  attachCorridorUnpacked(corridor);
  keepPacked = true;
  rebuildPacked();
}

void DubinsRouter::attachCorridorUnpacked(const grid::BitGrid* corridor) {
  if (corridor != nullptr &&
      (corridor->width() != gridWidth || corridor->height() != gridHeight)) {
    throw std::invalid_argument("the corridor does not match the router grid");
  }
  corridorMask = corridor;
  keepPacked = false;
  packedValid = false;
}

bool DubinsRouter::corridorBlocked(const uint32_t x, const uint32_t y) const {
  return corridorMask != nullptr && corridorMask->testCell(x, y);
}

void DubinsRouter::rebuildPacked() {
  packedValid = false;
  if (corridorMask == nullptr || !keepPacked) {
    return;
  }
  const std::size_t n = cells();
  packedGrid.resize(n);
  // Local views, so that the stores of bytes do not make the loop read the
  // vectors again.
  const std::span<const uint64_t> words(corridorMask->words());
  const std::span<const uint8_t> penalties(staticPenalties);
  const std::span<uint8_t> packed(packedGrid);
  // A word of the corridor mask holds 64 cells. A clear word copies their
  // penalties, and a full word sets the high bit, which marks a cell outside
  // the corridor, in all of them. Only a mixed word is read bit by bit.
  for (std::size_t word = 0; word < words.size(); ++word) {
    const std::size_t begin = word * 64;
    const std::size_t end = std::min(begin + 64, n);
    const uint64_t bits = words[word];
    if (bits == 0) {
      for (std::size_t i = begin; i < end; ++i) {
        packed[i] = penalties[i];
      }
    } else if (bits == ~uint64_t{0}) {
      for (std::size_t i = begin; i < end; ++i) {
        packed[i] = static_cast<uint8_t>(penalties[i] | 0x80U);
      }
    } else {
      for (std::size_t i = begin; i < end; ++i) {
        const auto outside =
            static_cast<uint8_t>(((bits >> (i - begin)) & 1U) << 7U);
        packed[i] = static_cast<uint8_t>(outside | penalties[i]);
      }
    }
  }
  packedValid = true;
}

void DubinsRouter::setStaticProximity(std::vector<uint8_t> penalty) {
  if (penalty.size() != cells()) {
    throw std::invalid_argument(
        "the proximity grid does not match the router grid");
  }
  std::ranges::for_each(penalty, checkProximityPenalty);
  staticPenalties = std::move(penalty);
  rebuildPacked();
}

void DubinsRouter::computeStaticProximity(const uint32_t distance,
                                          const uint8_t penalty) {
  const bool grows = obstacleMask != nullptr && distance != 0 && penalty != 0;
  if (grows) {
    checkProximityPenalty(penalty);
  }
  std::ranges::fill(staticPenalties, 0);
  if (grows) {
    computeStaticProximityWindow(
        distance, penalty,
        {.minX = 0, .maxX = gridWidth - 1, .minY = 0, .maxY = gridHeight - 1});
  }
  rebuildPacked();
}

void DubinsRouter::computeStaticProximityWindow(const uint32_t distance,
                                                const uint8_t penalty,
                                                CellBox window) {
  const bool noCell = obstacleMask == nullptr || window.minX > window.maxX ||
                      window.minY > window.maxY || window.minX >= gridWidth ||
                      window.minY >= gridHeight;
  if (!noCell && distance != 0) {
    checkProximityPenalty(penalty);
  }
  packedValid = false;
  if (noCell) {
    return;
  }
  window.maxX = std::min(window.maxX, gridWidth - 1);
  window.maxY = std::min(window.maxY, gridHeight - 1);
  for (uint32_t y = window.minY; y <= window.maxY; ++y) {
    std::fill(
        staticPenalties.begin() +
            static_cast<std::ptrdiff_t>(
                (static_cast<std::size_t>(y) * gridWidth) + window.minX),
        staticPenalties.begin() +
            static_cast<std::ptrdiff_t>(
                (static_cast<std::size_t>(y) * gridWidth) + window.maxX + 1),
        0);
  }
  if (distance == 0 || penalty == 0) {
    return;
  }

  // The halo holds the obstacles that reach into the window: a growth of
  // distance layers never needs to expand past it. The far bounds stop at the
  // edge of the grid, so that a large distance does not wrap them.
  const uint32_t haloMinX = window.minX > distance ? window.minX - distance : 0;
  const auto haloMaxX = static_cast<uint32_t>(
      std::min<uint64_t>(uint64_t{window.maxX} + distance, gridWidth - 1));
  const uint32_t haloMinY = window.minY > distance ? window.minY - distance : 0;
  const auto haloMaxY = static_cast<uint32_t>(
      std::min<uint64_t>(uint64_t{window.maxY} + distance, gridHeight - 1));
  const uint32_t haloWidth = haloMaxX - haloMinX + 1;
  const std::size_t haloSize =
      static_cast<std::size_t>(haloWidth) * (haloMaxY - haloMinY + 1);
  // The growth keeps its own memory, so that a router does not hold a byte
  // per cell between two growths.
  std::vector<uint8_t> visited(haloSize, 0);
  std::vector<uint32_t> front;
  std::vector<uint32_t> next;

  const auto local = [&](const uint32_t x, const uint32_t y) {
    return (static_cast<std::size_t>(y - haloMinY) * haloWidth) +
           (x - haloMinX);
  };
  const auto mark = [&](const uint32_t x, const uint32_t y,
                        const uint8_t value) {
    if (x >= window.minX && x <= window.maxX && y >= window.minY &&
        y <= window.maxY) {
      staticPenalties[(static_cast<std::size_t>(y) * gridWidth) + x] = value;
    }
  };
  for (uint32_t y = haloMinY; y <= haloMaxY; ++y) {
    for (uint32_t x = haloMinX; x <= haloMaxX; ++x) {
      if (obstacleMask->testCell(x, y) && visited[local(x, y)] == 0) {
        visited[local(x, y)] = 1;
        mark(x, y, penalty);
        front.push_back((y * gridWidth) + x);
      }
    }
  }
  static constexpr std::array<int8_t, 4> DXS = {1, -1, 0, 0};
  static constexpr std::array<int8_t, 4> DYS = {0, 0, 1, -1};
  // The decay is computed in 64 bits, so that no distance wraps it.
  const uint64_t decayDivisor = uint64_t{distance} + 1U;
  // The halo bounds, signed like the coordinates of a neighbor.
  const auto signedMinX = static_cast<int64_t>(haloMinX);
  const auto signedMinY = static_cast<int64_t>(haloMinY);
  const auto signedMaxX = static_cast<int64_t>(haloMaxX);
  const auto signedMaxY = static_cast<int64_t>(haloMaxY);
  for (uint32_t d = 1; d <= distance && !front.empty(); ++d) {
    next.clear();
    const uint64_t decayNumerator =
        uint64_t{penalty} * (uint64_t{distance} - d + 1U);
    const auto layerPenalty = static_cast<uint8_t>(
        std::max<uint64_t>(1U, decayNumerator / decayDivisor));
    for (const uint32_t current : front) {
      const uint32_t cx = current % gridWidth;
      const uint32_t cy = current / gridWidth;
      for (std::size_t i = 0; i < DXS.size(); ++i) {
        const int64_t nx = static_cast<int64_t>(cx) + DXS[i];
        const int64_t ny = static_cast<int64_t>(cy) + DYS[i];
        if (nx < signedMinX || ny < signedMinY || nx > signedMaxX ||
            ny > signedMaxY) {
          continue;
        }
        const auto ux = static_cast<uint32_t>(nx);
        const auto uy = static_cast<uint32_t>(ny);
        if (visited[local(ux, uy)] != 0) {
          continue;
        }
        visited[local(ux, uy)] = 1;
        mark(ux, uy, layerPenalty);
        next.push_back((uy * gridWidth) + ux);
      }
    }
    front.swap(next);
  }
}

void DubinsRouter::attachWireProximity(const std::vector<uint8_t>* penalty) {
  if (penalty != nullptr && penalty->size() != cells()) {
    throw std::invalid_argument(
        "the wire proximity grid does not match the router grid");
  }
  wireProximity = penalty;
}

uint8_t DubinsRouter::wirePenalty(const std::size_t index) const {
  return wireProximity != nullptr ? (*wireProximity)[index] : 0;
}

// --- Crossing constraints ----------------------------------------------------

void DubinsRouter::holdCrossingRules() {
  if (crossingRules.empty()) {
    crossingRules.assign(cells(), 0);
  }
}

void DubinsRouter::releaseUnusedCrossingRules() {
  if (constraints.empty() && exemptCells.empty() && singleCrossing == nullptr) {
    crossingRules = std::vector<uint8_t>();
  }
}

void DubinsRouter::clearOrthogonalConstraints() {
  constraints.clear();
  releaseUnusedCrossingRules();
  for (uint8_t& rules : crossingRules) {
    rules &= static_cast<uint8_t>(~CONSTRAINED);
  }
}

void DubinsRouter::buildOrthogonalConstraints(
    const std::vector<Path>& feedlines, const std::vector<bool>& skip,
    const int expandRadius) {
  // Every allocation runs before the first change, so a failed call keeps the
  // previous constraints and their rule bits.
  CrossingConstraints built;
  built.build(gridWidth, gridHeight, feedlines, skip, expandRadius);
  // The search reads from the rule byte of a cell whether the constraints
  // constrain it, so that it calls them only for such a cell.
  holdCrossingRules();
  static_assert(std::is_nothrow_move_assignable_v<CrossingConstraints>);
  constraints = std::move(built);
  const std::span<const uint8_t> masks = constraints.cellMasks();
  for (std::size_t index = 0; index < masks.size(); ++index) {
    uint8_t& rules = crossingRules[index];
    rules = masks[index] != 0U ? static_cast<uint8_t>(rules | CONSTRAINED)
                               : static_cast<uint8_t>(rules & ~CONSTRAINED);
  }
}

void DubinsRouter::setSingleCrossingFeedline(const Path* feedline,
                                             const int straightRadius,
                                             const int curveRadius) {
  singleCrossing = feedline;
  singleCrossingStraightRadius = straightRadius;
  singleCrossingCurveRadius = curveRadius;
  releaseUnusedCrossingRules();
}

void DubinsRouter::beginSingleCrossingOverlay(const PathPoint& source,
                                              const PathPoint& target) {
  if (singleCrossing == nullptr || singleCrossingStraightRadius <= 0 ||
      singleCrossing->size() < 2) {
    return;
  }
  holdCrossingRules();

  const int64_t straightRadius = singleCrossingStraightRadius;
  const int64_t curveRadius = singleCrossingCurveRadius;
  const SegmentedPath segmented =
      reconstructSegments(*movePrimitives, *singleCrossing);
  const int64_t sx = source.x;
  const int64_t sy = source.y;
  const int64_t tx = target.x;
  const int64_t ty = target.y;
  const auto w = static_cast<int64_t>(gridWidth);
  const auto h = static_cast<int64_t>(gridHeight);

  // The far side of a segment is the side its line puts the target on,
  // provided the source lies on the other side. A segment with both ends
  // on one side is one the wire has no business crossing; it gets no rule.
  const auto farSideOf = [&](const int64_t cx, const int64_t cy,
                             const int64_t dx, const int64_t dy) {
    const int ss = sideSign(dx, dy, sx, sy, cx, cy);
    const int st = sideSign(dx, dy, tx, ty, cx, cy);
    if (st == 0 || ss == st) {
      return 0;
    }
    return st;
  };
  const auto mark = [&](const int64_t cx, const int64_t cy, const int64_t dx,
                        const int64_t dy, const int farSign,
                        const uint8_t value, const int64_t radius) {
    const bool isCurve = (value & SIDE_STRAIGHT) == 0U;
    for (int64_t y = std::max<int64_t>(0, cy - radius);
         y <= std::min<int64_t>(h - 1, cy + radius); ++y) {
      for (int64_t x = std::max<int64_t>(0, cx - radius);
           x <= std::min<int64_t>(w - 1, cx + radius); ++x) {
        if (sideSign(dx, dy, x, y, cx, cy) != farSign) {
          continue;
        }
        uint8_t& rules =
            crossingRules[(static_cast<std::size_t>(y) * gridWidth) +
                          static_cast<std::size_t>(x)];
        const auto m = static_cast<uint8_t>(rules & SIDE_BITS);
        if (m == 0U) {
          // The list grows before the bits change, so a failed allocation
          // leaves no bit that endSingleCrossingOverlay() does not clear.
          crossingSideCells.push_back(static_cast<uint32_t>((y * w) + x));
          rules |= value;
        } else if ((m & SIDE_STRAIGHT) != 0U && m != value) {
          // The straight zones of two segments with different exit headings
          // meet near a turn of the feedline, so the cell is blocked. A curve
          // zone over a straight zone keeps the rule of the straight zone.
          if (!isCurve) {
            rules = static_cast<uint8_t>((rules & ~SIDE_BITS) | SIDE_FAR);
          }
        }
      }
    }
  };

  // Straight zones: a step may enter a cell of the zone only on the heading
  // that leaves the feedline at a right angle, toward the far side.
  for (const PathSegment& segment : segmented.segments) {
    if (!segment.straight()) {
      continue;
    }
    const HeadingVector d = headingVector(segment.heading);
    const PathPoint& mid = segment.cells[segment.cells.size() / 2];
    const int farSign = farSideOf(mid.x, mid.y, d.dx, d.dy);
    if (farSign == 0) {
      continue;
    }
    // The exit run of a crossing leaves the feedline at a right angle,
    // toward the far side. Of the two perpendiculars of the segment, the
    // one two eighths back points to the positive side.
    const Heading away = (farSign > 0) ? turned(segment.heading, -2)
                                       : turned(segment.heading, 2);
    const auto value = static_cast<uint8_t>(SIDE_FAR | SIDE_STRAIGHT | away);
    for (const PathPoint& c : segment.cells) {
      mark(c.x, c.y, d.dx, d.dy, farSign, value, straightRadius);
    }
  }

  // Curve zones and the head zone: never enterable on the far side.
  const auto blockRun = [&](const std::span<const PathPoint> run) {
    for (std::size_t k = 0; k < run.size(); ++k) {
      const PathPoint& a = run[k];
      int64_t dx = 0;
      int64_t dy = 0;
      if (k + 1 < run.size()) {
        dx = static_cast<int64_t>(run[k + 1].x) - a.x;
        dy = static_cast<int64_t>(run[k + 1].y) - a.y;
      } else if (k > 0) {
        dx = static_cast<int64_t>(a.x) - run[k - 1].x;
        dy = static_cast<int64_t>(a.y) - run[k - 1].y;
      }
      if (dx == 0 && dy == 0) {
        const HeadingVector d = headingVector(a.heading);
        // A signed step of -1, 0 or 1.
        // NOLINTNEXTLINE(bugprone-signed-char-misuse)
        dx = static_cast<int64_t>(d.dx);
        // A signed step of -1, 0 or 1.
        // NOLINTNEXTLINE(bugprone-signed-char-misuse)
        dy = static_cast<int64_t>(d.dy);
      }
      const int farSign = farSideOf(a.x, a.y, dx, dy);
      if (farSign == 0) {
        continue;
      }
      mark(a.x, a.y, dx, dy, farSign, SIDE_FAR, curveRadius);
    }
  };
  // A curve zone surrounds each run of points under one move that is not a
  // straight run of two or more cells. The zone centers on the last point of
  // the run, which for a turn is the last cell its arc sweeps.
  const Path& feedline = *singleCrossing;
  std::size_t runBegin = 0;
  for (std::size_t k = 1; k <= feedline.size(); ++k) {
    const PathPoint& first = feedline[runBegin];
    if (k < feedline.size() && feedline[k].heading == first.heading &&
        feedline[k].primitive == first.primitive) {
      continue;
    }
    const PathPoint& last = feedline[k - 1];
    bool longStraight = false;
    if (movePrimitives->isStraight(first.heading, first.primitive)) {
      for (std::size_t r = runBegin + 1; r < k && !longStraight; ++r) {
        longStraight = !feedline[r].samePlace(first);
      }
    }
    if (!longStraight) {
      blockRun(std::span<const PathPoint>(&last, 1));
    }
    runBegin = k;
  }
  blockRun(std::span<const PathPoint>(feedline).first(
      std::min<std::size_t>(10, feedline.size())));
}

void DubinsRouter::endSingleCrossingOverlay() {
  for (const uint32_t index : crossingSideCells) {
    crossingRules[index] &= static_cast<uint8_t>(~SIDE_BITS);
  }
  crossingSideCells.clear();
}

void DubinsRouter::setCrossingExemption(const std::vector<uint32_t>& cells) {
  // Every allocation runs before the first change, so a failed call keeps the
  // previous exemption.
  exemptCells.reserve(cells.size());
  if (!cells.empty()) {
    holdCrossingRules();
  }
  for (const uint32_t index : exemptCells) {
    crossingRules[index] &= static_cast<uint8_t>(~EXEMPT);
  }
  exemptCells.clear();
  for (const uint32_t index : cells) {
    if (index < crossingRules.size() && (crossingRules[index] & EXEMPT) == 0U) {
      crossingRules[index] |= EXEMPT;
      exemptCells.push_back(index);
    }
  }
  releaseUnusedCrossingRules();
}

bool DubinsRouter::canCrossOrthogonal(const uint32_t x, const uint32_t y,
                                      const Heading heading) const {
  if (x >= gridWidth || y >= gridHeight) {
    return false;
  }
  if (!crossingRules.empty()) {
    const uint8_t rules =
        crossingRules[(static_cast<std::size_t>(y) * gridWidth) + x];
    if ((rules & EXEMPT) != 0U) {
      return true;
    }
    if ((rules & SIDE_FAR) != 0U &&
        ((rules & SIDE_STRAIGHT) == 0U ||
         (rules & SIDE_HEADING) != (heading & SIDE_HEADING))) {
      return false;
    }
  }
  return constraints.allowed(x, y, heading);
}

bool DubinsRouter::canTurnOrthogonal(const uint32_t x, const uint32_t y) const {
  if (x >= gridWidth || y >= gridHeight) {
    return false;
  }
  if (!crossingRules.empty()) {
    const uint8_t rules =
        crossingRules[(static_cast<std::size_t>(y) * gridWidth) + x];
    if ((rules & EXEMPT) != 0U) {
      return true;
    }
    if ((rules & SIDE_FAR) != 0U) {
      return false;
    }
  }
  return constraints.turnAllowed(x, y);
}

bool DubinsRouter::stubCrossingAllowed(const PathPoint& from,
                                       const uint32_t length) const {
  const HeadingVector v = headingVector(from.heading);
  for (int64_t step = 1; step <= int64_t{length}; ++step) {
    const auto x =
        static_cast<uint32_t>(static_cast<int64_t>(from.x) + (v.dx * step));
    const auto y =
        static_cast<uint32_t>(static_cast<int64_t>(from.y) + (v.dy * step));
    if (!canCrossOrthogonal(x, y, from.heading)) {
      return false;
    }
  }
  return true;
}

bool DubinsRouter::crossingAllowedOrthogonal(const uint32_t x, const uint32_t y,
                                             const Heading heading) const {
  return canCrossOrthogonal(x, y, heading);
}

bool DubinsRouter::turnAllowedOrthogonal(const uint32_t x,
                                         const uint32_t y) const {
  return canTurnOrthogonal(x, y);
}

uint8_t DubinsRouter::constraintMaskAt(const uint32_t x,
                                       const uint32_t y) const {
  return constraints.maskAt(x, y);
}

// --- Search tables ---------------------------------------------------------

void DubinsRouter::buildTables(const uint32_t bend) {
  const auto w = static_cast<int32_t>(gridWidth);

  struct TempNode {
    int8_t dx = 0;
    int8_t dy = 0;
    uint16_t primitives = 0;
    uint8_t completes = NO_PRIMITIVE;
    std::vector<uint32_t> children;
  };

  // Every allocation runs before the first change, so a failed call keeps the
  // previous tables.
  std::array<std::vector<TrieNode>, NUM_HEADINGS> builtTrie;
  std::array<std::vector<TriePrimitive>, NUM_HEADINGS> builtPrimitives;
  std::array<uint32_t, NUM_HEADINGS> builtNegativeX{};
  std::array<uint32_t, NUM_HEADINGS> builtPositiveX{};
  std::array<uint32_t, NUM_HEADINGS> builtNegativeY{};
  std::array<uint32_t, NUM_HEADINGS> builtPositiveY{};
  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    auto& tprims = builtPrimitives[heading];
    auto& tflat = builtTrie[heading];
    std::vector<TempNode> nodes;
    std::vector<uint32_t> roots;
    int32_t negativeX = 0;
    int32_t positiveX = 0;
    int32_t negativeY = 0;
    int32_t positiveY = 0;

    const auto prims = movePrimitives->of(static_cast<Heading>(heading));
    // The primitives of every radius from MIN_BEND_RADIUS to MAX_BEND_RADIUS
    // fit the tables, so the checks of this loop do not fire. They guard the
    // layout of the tables if the radius limits change.
    if (prims.size() > MAX_PRIMITIVES_PER_HEADING) {
      throw std::invalid_argument(
          "a heading has more primitives than the search tables hold");
    }
    for (std::size_t li = 0; li < prims.size(); ++li) {
      const Primitive& p = prims[li];
      TriePrimitive tp;
      tp.endDx = p.dx;
      tp.endDy = p.dy;
      tp.exit = p.exitHeading & 7U;
      tp.id = p.id;
      tp.sweptCount = static_cast<uint16_t>(p.swept.size());
      // The move cost in hundredths of a cell: rounded to single precision
      // first, then scaled and truncated. The rounding is part of the cost
      // definition, so a change to it can change the path a search returns.
      const auto moveCost = static_cast<uint32_t>(
          static_cast<double>(static_cast<float>(p.cost)) * 100.0);
      tp.costBend =
          moveCost +
          (headingDistance(static_cast<Heading>(heading), tp.exit) * bend);

      negativeX = std::max<int32_t>(negativeX, -p.dx);
      positiveX = std::max<int32_t>(positiveX, p.dx);
      negativeY = std::max<int32_t>(negativeY, -p.dy);
      positiveY = std::max<int32_t>(positiveY, p.dy);
      for (const CellOffset& c : p.swept) {
        negativeX = std::max<int32_t>(negativeX, -c.dx);
        positiveX = std::max<int32_t>(positiveX, c.dx);
        negativeY = std::max<int32_t>(negativeY, -c.dy);
        positiveY = std::max<int32_t>(positiveY, c.dy);
      }

      // The normalized cell sequence: the origin first, the end last.
      std::vector<CellOffset> cellsOfMove;
      const bool sweepsOrigin = !p.swept.empty() && p.swept.front().dx == 0 &&
                                p.swept.front().dy == 0;
      if (!sweepsOrigin) {
        cellsOfMove.push_back({.dx = 0, .dy = 0});
      }
      for (const CellOffset& c : p.swept) {
        cellsOfMove.push_back(c);
      }
      bool appended = false;
      if (cellsOfMove.back().dx != p.dx || cellsOfMove.back().dy != p.dy) {
        cellsOfMove.push_back({.dx = p.dx, .dy = p.dy});
        appended = true;
      }
      tp.sweepsOrigin = sweepsOrigin ? 1 : 0;
      tp.endExtra = appended ? 0 : 1;
      if (cellsOfMove.size() > MAX_TRIE_DEPTH) {
        throw std::invalid_argument(
            "a primitive sweeps more cells than the search tables hold");
      }

      uint32_t parent = 0xFFFFFFFFU;
      for (std::size_t k = 1; k < cellsOfMove.size(); ++k) {
        const CellOffset& c = cellsOfMove[k];
        if (c.dx < -127 || c.dx > 127 || c.dy < -127 || c.dy > 127) {
          throw std::invalid_argument(
              "a primitive reaches further than the search tables hold");
        }
        const auto dx = static_cast<int8_t>(c.dx);
        const auto dy = static_cast<int8_t>(c.dy);
        uint32_t child = 0xFFFFFFFFU;
        const auto& siblings =
            (parent == 0xFFFFFFFFU) ? roots : nodes[parent].children;
        for (const uint32_t candidate : siblings) {
          if (nodes[candidate].dx == dx && nodes[candidate].dy == dy) {
            child = candidate;
            break;
          }
        }
        if (child == 0xFFFFFFFFU) {
          child = static_cast<uint32_t>(nodes.size());
          TempNode node;
          node.dx = dx;
          node.dy = dy;
          nodes.push_back(node);
          ((parent == 0xFFFFFFFFU) ? roots : nodes[parent].children)
              .push_back(child);
        }
        nodes[child].primitives |= static_cast<uint16_t>(1U << li);
        if (k + 1 == cellsOfMove.size()) {
          if (nodes[child].completes != NO_PRIMITIVE) {
            throw std::invalid_argument(
                "two primitives of a heading end on the same cell");
          }
          nodes[child].completes = static_cast<uint8_t>(li);
        }
        parent = child;
      }
      tprims.push_back(tp);
    }

    builtNegativeX[heading] = static_cast<uint32_t>(negativeX);
    builtPositiveX[heading] = static_cast<uint32_t>(positiveX);
    builtNegativeY[heading] = static_cast<uint32_t>(negativeY);
    builtPositiveY[heading] = static_cast<uint32_t>(positiveY);

    // Emit the trie in preorder with the subtree size as skip.
    const auto emit = [&](const auto& self, const uint32_t n,
                          const uint32_t depth) -> uint32_t {
      const auto mine = static_cast<uint32_t>(tflat.size());
      TrieNode node;
      node.dx = nodes[n].dx;
      node.dy = nodes[n].dy;
      node.linear = (static_cast<int32_t>(nodes[n].dy) * w) + nodes[n].dx;
      node.primitives = nodes[n].primitives;
      node.completes = nodes[n].completes;
      node.depth = static_cast<uint8_t>(depth);
      tflat.push_back(node);
      uint32_t size = 1;
      for (const uint32_t child : nodes[n].children) {
        size += self(self, child, depth + 1);
      }
      tflat[mine].skip = static_cast<uint16_t>(size);
      return size;
    };
    tflat.reserve(nodes.size());
    for (const uint32_t root : roots) {
      emit(emit, root, 0);
    }
  }
  static_assert(std::is_nothrow_swappable_v<decltype(trie)> &&
                std::is_nothrow_swappable_v<decltype(triePrimitives)>);
  trie.swap(builtTrie);
  triePrimitives.swap(builtPrimitives);
  marginNegativeX = builtNegativeX;
  marginPositiveX = builtPositiveX;
  marginNegativeY = builtNegativeY;
  marginPositiveY = builtPositiveY;
}

// --- Distance field --------------------------------------------------------

void DubinsRouter::beginDistanceField(const uint32_t tx, const uint32_t ty) {
  const std::size_t n = cells();
  if (distanceField.size() != n) {
    distanceField.assign(n, 0);
    fieldStamp = 0;
  }
  // Stamp 0 means never written; the stamps run 1 to 1023, then the field
  // is cleared once.
  if (++fieldStamp >= (1U << (32U - FIELD_DISTANCE_BITS))) {
    std::ranges::fill(distanceField, 0);
    fieldStamp = 1;
  }
  for (auto& bucket : fieldBuckets) {
    bucket.clear();
  }
  distanceField[(static_cast<std::size_t>(ty) * gridWidth) + tx] =
      fieldStamp << FIELD_DISTANCE_BITS;
  fieldBuckets[0].push_back((ty << 16U) | tx);
  fieldPending = 1;
  fieldScan = 0;
  fieldSettled = STRAIGHT_COST;
}

uint32_t DubinsRouter::growDistanceField(const std::size_t cell) {
  // The growth is Dijkstra's algorithm with steps of STRAIGHT_COST and
  // DIAGONAL_COST. A bucket holds one distance: the distances that wait in
  // the buckets run from the scan distance to at most DIAGONAL_COST above
  // it, fewer than FIELD_BUCKETS. Every bucket below the scan distance d is
  // drained, so each later step starts from d or more, adds STRAIGHT_COST or
  // more, and is clamped to FIELD_DISTANCE_MASK at most. It writes at least
  // min(d + STRAIGHT_COST, FIELD_DISTANCE_MASK), and a stamped distance at or
  // below that bound is final. With the buckets empty, every stamped
  // distance is final, and a cell without the stamp cannot reach the target.
  const uint32_t stamp = fieldStamp;
  const uint32_t stampHigh = stamp << FIELD_DISTANCE_BITS;
  const grid::BitGrid& corridor = *corridorMask;
  // Local copies, so that the stores to the field and to the buckets do not
  // make the loop read the members again.
  const std::span<uint32_t> field(distanceField);
  std::size_t pending = fieldPending;
  uint32_t d = fieldScan;
  // An older stamp makes the difference wrap above FIELD_DISTANCE_MASK.
  while (pending != 0 && field[cell] - stampHigh >
                             std::min(d + STRAIGHT_COST, FIELD_DISTANCE_MASK)) {
    // A waiting entry lies at most DIAGONAL_COST above d.
    while (fieldBuckets[d % FIELD_BUCKETS].empty()) {
      ++d;
    }
    auto& bucket = fieldBuckets[d % FIELD_BUCKETS];
    while (!bucket.empty()) {
      const uint32_t packed = bucket.back();
      bucket.pop_back();
      --pending;
      const uint32_t x = packed & 0xFFFFU;
      const uint32_t y = packed >> 16U;
      const std::size_t index = (static_cast<std::size_t>(y) * gridWidth) + x;
      if ((field[index] & FIELD_DISTANCE_MASK) != d) {
        continue;
      }
      const uint32_t x0 = (x == 0) ? 0 : x - 1;
      const uint32_t x1 = (x + 1 >= gridWidth) ? gridWidth - 1 : x + 1;
      const uint32_t y0 = (y == 0) ? 0 : y - 1;
      const uint32_t y1 = (y + 1 >= gridHeight) ? gridHeight - 1 : y + 1;
      for (uint32_t ny = y0; ny <= y1; ++ny) {
        const std::size_t row = static_cast<std::size_t>(ny) * gridWidth;
        for (uint32_t nx = x0; nx <= x1; ++nx) {
          if (nx == x && ny == y) {
            continue;
          }
          const std::size_t nIndex = row + nx;
          if (corridor.test(nIndex)) {
            continue;
          }
          const uint32_t weight =
              (nx != x && ny != y) ? DIAGONAL_COST : STRAIGHT_COST;
          const uint32_t nextDistance =
              std::min(d + weight, FIELD_DISTANCE_MASK);
          const uint32_t nv = field[nIndex];
          if ((nv >> FIELD_DISTANCE_BITS) == stamp &&
              (nv & FIELD_DISTANCE_MASK) <= nextDistance) {
            continue;
          }
          field[nIndex] = stampHigh | nextDistance;
          fieldBuckets[nextDistance % FIELD_BUCKETS].push_back((ny << 16U) |
                                                               nx);
          ++pending;
        }
      }
    }
    ++d;
  }
  fieldPending = pending;
  fieldScan = d;
  fieldSettled = pending == 0
                     ? FIELD_DISTANCE_MASK
                     : std::min(d + STRAIGHT_COST, FIELD_DISTANCE_MASK);
  return field[cell];
}

// --- States and paths ------------------------------------------------------

PathPoint DubinsRouter::unpackState(const uint32_t index) const {
  const auto heading = static_cast<Heading>(index & 7U);
  const uint32_t cell = index >> 3U;
  return {.x = cell % gridWidth,
          .y = cell / gridWidth,
          .heading = heading,
          .primitive = 0};
}

uint32_t DubinsRouter::parentState(const uint32_t index) const {
  const SearchNode& node = searchScratch->at(index);
  const auto parentHeading = static_cast<Heading>(node.parentHeading & 7U);
  const Primitive* primitive =
      movePrimitives->find(parentHeading, node.primitive);
  const int32_t dx = primitive != nullptr ? primitive->dx : 0;
  const int32_t dy = primitive != nullptr ? primitive->dy : 0;
  const PathPoint c = unpackState(index);
  const auto px = static_cast<uint32_t>(static_cast<int32_t>(c.x) - dx);
  const auto py = static_cast<uint32_t>(static_cast<int32_t>(c.y) - dy);
  return stateIndex(px, py, parentHeading);
}

Path DubinsRouter::reconstruct(const uint32_t goalIndex,
                               const uint32_t startIndex) const {
  Path path;
  uint32_t current = goalIndex;
  path.push_back(unpackState(current));
  while (current != startIndex) {
    const SearchNode& node = searchScratch->at(current);
    const uint32_t parentIndex = parentState(current);
    const uint16_t edge = node.primitive;
    PathPoint parent = unpackState(parentIndex);
    if (const Primitive* primitive =
            movePrimitives->find(parent.heading, edge)) {
      std::vector<PathPoint> segment;
      segment.reserve(primitive->swept.size());
      for (const CellOffset& off : primitive->swept) {
        segment.push_back({.x = static_cast<uint32_t>(
                               static_cast<int64_t>(parent.x) + off.dx),
                           .y = static_cast<uint32_t>(
                               static_cast<int64_t>(parent.y) + off.dy),
                           .heading = parent.heading,
                           .primitive = edge});
      }
      std::ranges::reverse(segment);
      for (const PathPoint& c : segment) {
        if (!c.samePlace(path.back())) {
          path.push_back(c);
        }
      }
    }
    parent.primitive = edge;
    current = parentIndex;
    if (!parent.samePlace(path.back())) {
      path.push_back(parent);
    }
  }
  std::ranges::reverse(path);
  // The goal state itself belongs to the target stub.
  if (!path.empty()) {
    path.pop_back();
  }
  return path;
}

PathPoint DubinsRouter::sanitize(const PathPoint point,
                                 const bool isTarget) const {
  const int64_t length =
      isTarget ? static_cast<int64_t>(searchParams.endStraightLength)
               : -static_cast<int64_t>(searchParams.startStraightLength);
  const HeadingVector v = headingVector(point.heading);
  // The source moves along its heading, the target back along its heading.
  // The moved coordinate is exact in 64 bits. A coordinate outside the range
  // of a cell coordinate becomes the largest one, so that a stub of up to
  // 2^32 - 1 cells cannot wrap the point back into the grid.
  const auto moved = [](const uint32_t coordinate, const int64_t offset) {
    const int64_t value = static_cast<int64_t>(coordinate) + offset;
    constexpr auto largest = std::numeric_limits<uint32_t>::max();
    return std::in_range<uint32_t>(value) ? static_cast<uint32_t>(value)
                                          : largest;
  };
  return {.x = moved(point.x, -(v.dx * length)),
          .y = moved(point.y, -(v.dy * length)),
          .heading = static_cast<Heading>(point.heading & 7U),
          .primitive = 0};
}

Path DubinsRouter::straightStub(const PathPoint point, const bool isTarget,
                                const uint32_t length) const {
  PathPoint start = point;
  start.heading = static_cast<Heading>(point.heading & 7U);
  if (!isTarget) {
    return straightRun(*movePrimitives, start, length);
  }
  // A target stub runs backward from the target: the run from the far end,
  // reversed.
  const HeadingVector v = headingVector(start.heading);
  start.x = static_cast<uint32_t>(static_cast<int64_t>(start.x) -
                                  (v.dx * static_cast<int64_t>(length)));
  start.y = static_cast<uint32_t>(static_cast<int64_t>(start.y) -
                                  (v.dy * static_cast<int64_t>(length)));
  Path stub = straightRun(*movePrimitives, start, length);
  std::ranges::reverse(stub);
  return stub;
}

Path DubinsRouter::assemble(const Path& searched, const PathPoint& source,
                            const PathPoint& target) const {
  const Path head =
      straightStub(source, false, searchParams.startStraightLength);
  Path tail = straightStub(target, true, searchParams.endStraightLength);
  std::ranges::reverse(tail);
  Path path;
  path.reserve(head.size() + searched.size() + tail.size());
  path.insert(path.end(), head.begin(), head.end());
  // The head ends on the cell the search started from, so that cell would
  // otherwise appear twice and every consumer would have to step over a
  // move of no length.
  auto first = searched.begin();
  if (first != searched.end() && first->samePlace(path.back())) {
    ++first;
  }
  path.insert(path.end(), first, searched.end());
  // The tail starts on the cell the search ended at. Without a searched
  // path, the search started there too, and the head already ends on that
  // cell.
  auto tailFirst = tail.begin();
  if (searched.empty()) {
    ++tailFirst;
  }
  path.insert(path.end(), tailFirst, tail.end());
  return path;
}

template <typename Search> Path DubinsRouter::guarded(const Search& search) {
  Path path = search();
  if (path.empty()) {
    return path;
  }
  if (!pathSelfIntersects(path, gridWidth, gridHeight, loopScratch)) {
    return path;
  }
  // The same empty path a search that finds nothing returns: every caller
  // handles it, and a silent short is worse than a visible failure.
  ++loopGuardRejectionCount;
  return {};
}

// --- The free search -------------------------------------------------------

Path DubinsRouter::route(const RoutingObjective& objective,
                         const bool usePenalty, const bool onlyStraight) {
  if (corridorMask == nullptr) {
    throw std::logic_error("route needs an attached corridor");
  }
  if (usePenalty && wireProximity == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  checkHeadings(objective);
  RoutingObjective moved;
  moved.source = sanitize(objective.source, false);
  moved.target = sanitize(objective.target, true);
  // A stub is a straight run, so its cells lie in the grid when its two ends
  // do.
  if (!inGrid(objective.source) || !inGrid(objective.target) ||
      !inGrid(moved.source) || !inGrid(moved.target)) {
    return {};
  }
  searchScratch->beginSearch();
  return guarded([&] {
    if (packedValid) {
      return usePenalty
                 ? searchFree<true, true>(moved, objective.source,
                                          objective.target, onlyStraight)
                 : searchFree<true, false>(moved, objective.source,
                                           objective.target, onlyStraight);
    }
    return usePenalty
               ? searchFree<false, true>(moved, objective.source,
                                         objective.target, onlyStraight)
               : searchFree<false, false>(moved, objective.source,
                                          objective.target, onlyStraight);
  });
}

template <bool PACKED, bool USE_PENALTY>
Path DubinsRouter::searchFree(const RoutingObjective& objective,
                              const PathPoint& source, const PathPoint& target,
                              const bool onlyStraight) {
  open.clear();
  // The start meets the corridor test that every cell a move enters meets.
  // A goal outside the corridor cannot be reached, so the search does not
  // run.
  const auto outside = [&](const PathPoint& point) {
    const std::size_t linear =
        (static_cast<std::size_t>(point.y) * gridWidth) + point.x;
    if constexpr (PACKED) {
      return (packedGrid[linear] & 0x80U) != 0U;
    } else {
      return corridorMask->test(linear);
    }
  };
  if (outside(objective.source) || outside(objective.target)) {
    return {};
  }
  const std::span<const uint8_t> blockMap =
      PACKED ? std::span<const uint8_t>(packedGrid)
             : std::span<const uint8_t>();
  const std::span<const uint8_t> penaltyMap =
      PACKED ? std::span<const uint8_t>(packedGrid)
             : std::span<const uint8_t>(staticPenalties);
  constexpr uint8_t blockMask = PACKED ? uint8_t{0x80} : uint8_t{0xFF};
  constexpr uint8_t penaltyMask = PACKED ? uint8_t{0x7F} : uint8_t{0xFF};
  const std::span<const uint8_t> wireMap =
      wireProximity != nullptr ? std::span<const uint8_t>(*wireProximity)
                               : std::span<const uint8_t>();

  for (uint32_t a = 0; a < NUM_HEADINGS; ++a) {
    bendBoundByHeading[a] = bendLowerBound
                                ? headingDistance(static_cast<Heading>(a),
                                                  objective.target.heading) *
                                      searchParams.bendPenalty
                                : 0U;
  }

  const bool useField = activeHeuristic == Heuristic::DistanceField;
  if (useField) {
    beginDistanceField(objective.target.x, objective.target.y);
    growDistanceField(
        (static_cast<std::size_t>(objective.source.y) * gridWidth) +
        objective.source.x);
  }

  const uint32_t startIndex = stateIndex(objective.source.x, objective.source.y,
                                         objective.source.heading);
  searchScratch->setStart(startIndex);
  open.push({.x = static_cast<uint16_t>(objective.source.x),
             .y = static_cast<uint16_t>(objective.source.y),
             .heading = objective.source.heading,
             .f = 0,
             .g = 0});

  while (!open.empty()) {
    const QueueEntry current = open.pop();
    if (const QueueEntry* next = open.peek()) {
      const uint32_t nextIndex = stateIndex(next->x, next->y, next->heading);
      if (nextIndex < searchScratch->size()) {
        prefetchForWrite(&searchScratch->at(nextIndex));
      }
    }
    const uint32_t currentIndex =
        stateIndex(current.x, current.y, current.heading);
    SearchNode& node = searchScratch->at(currentIndex);
    if (node.iteration == searchScratch->iteration() && node.closed != 0U) {
      continue;
    }
    if (current.g > node.g) {
      continue;
    }
    node.closed = 1;

    if (current.x == objective.target.x && current.y == objective.target.y &&
        current.heading == objective.target.heading) {
      return assemble(reconstruct(currentIndex, startIndex), source, target);
    }
    expandFree<PACKED, USE_PENALTY>(current, objective, onlyStraight, blockMap,
                                    penaltyMap, blockMask, penaltyMask, wireMap,
                                    useField);
  }
  return {};
}

template <bool PACKED, bool USE_PENALTY>
void DubinsRouter::expandFree(
    const QueueEntry& current, const RoutingObjective& objective,
    const bool onlyStraight, const std::span<const uint8_t> blockMap,
    const std::span<const uint8_t> penaltyMap, const uint8_t blockMask,
    const uint8_t penaltyMask, const std::span<const uint8_t> wireMap,
    const bool useField) {
  const uint32_t heading = current.heading & 7U;
  const auto& tprims = triePrimitives[heading];
  // Local views, so that the stores of the expansion do not make the walk
  // read the vectors again.
  const std::span<const TrieNode> flat(trie[heading]);
  const uint32_t cx = current.x;
  const uint32_t cy = current.y;
  const std::size_t currentLinear =
      (static_cast<std::size_t>(cy) * gridWidth) + cx;
  const std::span<const uint32_t> field(distanceField);
  const uint32_t fieldHigh = fieldStamp << FIELD_DISTANCE_BITS;
  const grid::BitGrid* corridor = corridorMask;

  const auto outside = [&](const std::size_t index) {
    if constexpr (PACKED) {
      return (blockMap[index] & blockMask) != 0U;
    } else {
      return corridor->test(index);
    }
  };

  const bool fastBounds = (cx >= marginNegativeX[heading]) &&
                          (cx + marginPositiveX[heading] < gridWidth) &&
                          (cy >= marginNegativeY[heading]) &&
                          (cy + marginPositiveY[heading] < gridHeight);

  // The candidates: every primitive whose end is in the grid and in the
  // corridor, allowed, not dominated and, with a field, able to reach the
  // target.
  uint16_t alive = 0;
  // The largest entry of an end cell less the current stamp. An entry with
  // an older stamp wraps above FIELD_DISTANCE_MASK.
  uint32_t farthest = 0;
  std::array<uint32_t, MAX_PRIMITIVES_PER_HEADING> endField{};
  std::array<uint32_t, MAX_PRIMITIVES_PER_HEADING> endIndex{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endX{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endY{};
  for (std::size_t i = 0; i < tprims.size(); ++i) {
    const TriePrimitive& p = tprims[i];
    const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + p.endDx);
    const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + p.endDy);
    if (nx >= gridWidth || ny >= gridHeight) {
      continue;
    }
    if (onlyStraight && (p.exit & 1U) != 0U) {
      continue;
    }
    const std::size_t linear = (static_cast<std::size_t>(ny) * gridWidth) + nx;
    if (outside(linear)) {
      continue;
    }
    const uint32_t index = stateIndex(nx, ny, static_cast<Heading>(p.exit));
    const SearchNode& n = searchScratch->at(index);
    if (n.iteration == searchScratch->iteration()) {
      if (n.closed != 0U || current.g + p.costBend >= n.g) {
        continue;
      }
    }
    if (useField) {
      const uint32_t hv = field[linear];
      farthest = std::max(farthest, hv - fieldHigh);
      endField[i] = hv & FIELD_DISTANCE_MASK;
    }
    endIndex[i] = index;
    endX[i] = static_cast<uint16_t>(nx);
    endY[i] = static_cast<uint16_t>(ny);
    alive |= static_cast<uint16_t>(1U << i);
  }
  // An end cell without a final distance makes the field grow. The rare
  // growth runs after the loop above, and the hint lets the compiler keep the
  // values of the walk below in registers.
  if (farthest > fieldSettled) [[unlikely]] {
    for (std::size_t i = 0; i < tprims.size(); ++i) {
      if ((alive & (1U << i)) == 0) {
        continue;
      }
      const std::size_t linear =
          (static_cast<std::size_t>(endY[i]) * gridWidth) + endX[i];
      uint32_t hv = field[linear];
      if (hv - fieldHigh > fieldSettled) {
        hv = growDistanceField(linear);
      }
      if ((hv >> FIELD_DISTANCE_BITS) != fieldStamp) {
        // No walk through the corridor leads from the end cell to the target.
        // The cells of every move form a chain of neighbors (see
        // Primitive::swept), so no path of moves leads there either.
        alive &= static_cast<uint16_t>(~(1U << i));
      } else {
        endField[i] = hv & FIELD_DISTANCE_MASK;
      }
    }
  }
  if (alive == 0) {
    return;
  }

  const uint32_t penaltyCurrent =
      USE_PENALTY ? (penaltyMap[currentLinear] & penaltyMask) : 0U;
  // Only the instantiations that add the penalties write it.
  // NOLINTNEXTLINE(misc-const-correctness)
  std::array<uint32_t, MAX_TRIE_DEPTH + 2> penaltyAcc{};

  const auto nodeCount = static_cast<uint32_t>(flat.size());
  uint32_t i = 0;
  while (i < nodeCount) {
    const TrieNode tn = flat[i];
    if ((alive & tn.primitives) == 0) {
      i += tn.skip;
      continue;
    }
    std::size_t cell = 0;
    if (fastBounds) {
      cell = static_cast<std::size_t>(
          static_cast<std::ptrdiff_t>(currentLinear) + tn.linear);
    } else {
      const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + tn.dx);
      const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + tn.dy);
      if (nx >= gridWidth || ny >= gridHeight) {
        i += tn.skip;
        continue;
      }
      cell = (static_cast<std::size_t>(ny) * gridWidth) + nx;
    }
    if (outside(cell)) {
      i += tn.skip;
      continue;
    }
    if constexpr (USE_PENALTY) {
      penaltyAcc[tn.depth + 1U] =
          penaltyAcc[tn.depth] + (penaltyMap[cell] & penaltyMask);
    }
    if (tn.completes != NO_PRIMITIVE && (alive & (1U << tn.completes)) != 0) {
      const TriePrimitive& p = tprims[tn.completes];
      const uint32_t index = endIndex[tn.completes];
      // Only the instantiations that add the penalties write it.
      // NOLINTNEXTLINE(misc-const-correctness)
      uint32_t penaltyTerm = 0;
      if constexpr (USE_PENALTY) {
        uint32_t sum = penaltyAcc[tn.depth + 1U];
        if (p.sweepsOrigin != 0U) {
          sum += penaltyCurrent;
        }
        const std::size_t linear =
            (static_cast<std::size_t>(endY[tn.completes]) * gridWidth) +
            endX[tn.completes];
        if (p.endExtra != 0U) {
          sum += static_cast<uint32_t>(penaltyMap[linear] & penaltyMask);
        }
        penaltyTerm =
            PENALTY_SCALE * (sum + (static_cast<uint32_t>(wireMap[linear]) *
                                    (1U + p.sweptCount)));
      }
      const uint32_t tentative = current.g + p.costBend + penaltyTerm;
      SearchNode& n = searchScratch->at(index);
      const bool seen = n.iteration == searchScratch->iteration();
      if (!seen || (n.closed == 0U && tentative < n.g)) {
        if (!seen) {
          n.iteration = searchScratch->iteration();
          n.closed = 0;
        }
        n.g = tentative;
        n.parentHeading = heading & 7U;
        n.primitive = p.id & 0x3FFU;
        uint32_t h = 0;
        if (useField) {
          h = endField[tn.completes];
        } else {
          h = octile(endX[tn.completes], endY[tn.completes], objective.target.x,
                     objective.target.y);
        }
        h += bendBoundByHeading[p.exit & 7U];
        const uint32_t f = tentative + h;
        open.push({.x = endX[tn.completes],
                   .y = endY[tn.completes],
                   .heading = static_cast<uint8_t>(p.exit),
                   .f = f,
                   .g = tentative});
      }
    }
    ++i;
  }
}

// --- The orthogonal search -------------------------------------------------

Path DubinsRouter::routeOrthogonal(const RoutingObjective& objective,
                                   const bool usePenalty,
                                   const bool onlyStraight) {
  if (corridorMask == nullptr) {
    throw std::logic_error("routeOrthogonal needs an attached corridor");
  }
  if (usePenalty && wireProximity == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  checkHeadings(objective);
  RoutingObjective moved;
  moved.source = sanitize(objective.source, false);
  moved.target = sanitize(objective.target, true);
  // A stub is a straight run, so its cells lie in the grid when its two ends
  // do.
  if (!inGrid(objective.source) || !inGrid(objective.target) ||
      !inGrid(moved.source) || !inGrid(moved.target)) {
    return {};
  }
  searchScratch->beginSearch();
  // The overlay ends on every exit from this function, an exception
  // included.
  const auto endOverlay = [](DubinsRouter* router) {
    router->endSingleCrossingOverlay();
  };
  const std::unique_ptr<DubinsRouter, decltype(endOverlay)> overlayEnd(
      this, endOverlay);
  beginSingleCrossingOverlay(objective.source, objective.target);
  // The stubs are fixed, so their cells meet the crossing test of a straight
  // step before the search runs. The last step into the target tests the
  // target. The source meets the same test on its own heading, as if a
  // straight step entered it, so both ends of the wire meet the test.
  if (!canCrossOrthogonal(objective.source.x, objective.source.y,
                          objective.source.heading) ||
      !stubCrossingAllowed(objective.source,
                           searchParams.startStraightLength) ||
      !stubCrossingAllowed(moved.target, searchParams.endStraightLength)) {
    return {};
  }
  return guarded([&] {
    return usePenalty ? searchOrthogonal<true>(moved, objective.source,
                                               objective.target, onlyStraight)
                      : searchOrthogonal<false>(moved, objective.source,
                                                objective.target, onlyStraight);
  });
}

template <bool USE_PENALTY>
Path DubinsRouter::searchOrthogonal(const RoutingObjective& objective,
                                    const PathPoint& source,
                                    const PathPoint& target,
                                    const bool onlyStraight) {
  open.clear();
  // The start meets the corridor test that every cell a move enters meets.
  // A goal outside the corridor cannot be reached, so the search does not
  // run.
  if (corridorMask->testCell(objective.source.x, objective.source.y) ||
      corridorMask->testCell(objective.target.x, objective.target.y)) {
    return {};
  }
  const uint32_t startIndex = stateIndex(objective.source.x, objective.source.y,
                                         objective.source.heading);
  searchScratch->setStart(startIndex);
  open.push({.x = static_cast<uint16_t>(objective.source.x),
             .y = static_cast<uint16_t>(objective.source.y),
             .heading = objective.source.heading,
             .f = 0,
             .g = 0});
  while (!open.empty()) {
    const QueueEntry current = open.pop();
    if (const QueueEntry* next = open.peek()) {
      const uint32_t nextIndex = stateIndex(next->x, next->y, next->heading);
      if (nextIndex < searchScratch->size()) {
        prefetchForWrite(&searchScratch->at(nextIndex));
      }
    }
    const uint32_t currentIndex =
        stateIndex(current.x, current.y, current.heading);
    SearchNode& node = searchScratch->at(currentIndex);
    if (node.iteration == searchScratch->iteration() && node.closed != 0U) {
      continue;
    }
    if (current.g > node.g) {
      continue;
    }
    node.closed = 1;
    if (current.x == objective.target.x && current.y == objective.target.y &&
        current.heading == objective.target.heading) {
      return assemble(reconstruct(currentIndex, startIndex), source, target);
    }
    expandOrthogonal<USE_PENALTY>(current, objective, onlyStraight);
  }
  return {};
}

template <bool USE_PENALTY>
void DubinsRouter::expandOrthogonal(const QueueEntry& current,
                                    const RoutingObjective& objective,
                                    const bool onlyStraight) {
  const uint32_t heading = current.heading & 7U;
  const auto& tprims = triePrimitives[heading];
  // Local views, so that the stores of the expansion do not make the walk
  // read the vectors again.
  const std::span<const TrieNode> flat(trie[heading]);
  const std::span<const uint8_t> rules(crossingRules);
  const std::span<const uint8_t> penalties(staticPenalties);
  const std::span<const uint8_t> wires =
      wireProximity != nullptr ? std::span<const uint8_t>(*wireProximity)
                               : std::span<const uint8_t>();
  const uint32_t cx = current.x;
  const uint32_t cy = current.y;
  const std::size_t currentLinear =
      (static_cast<std::size_t>(cy) * gridWidth) + cx;
  const grid::BitGrid& corridor = *corridorMask;

  // The rule byte of a cell tells whether any crossing rule applies there.
  // A cell without a rule passes the crossing tests of a straight step and
  // of a turn, so only a ruled cell runs them.
  const auto ruled = [&](const std::size_t cell) {
    return !rules.empty() && (rules[cell] & RULED) != 0U;
  };
  // The heading of an arc changes along it, so a turn may touch only cells
  // that no crossing rule constrains, from its start to its end. That test
  // is stricter than the one of a straight step, so a cell that passes it
  // needs no other crossing test.
  const auto crossingOk = [&](const uint32_t x, const uint32_t y,
                              const bool turn) {
    return turn ? canTurnOrthogonal(x, y)
                : canCrossOrthogonal(x, y, static_cast<Heading>(heading));
  };
  const bool turnMayStart = !ruled(currentLinear) || canTurnOrthogonal(cx, cy);
  const bool fastBounds = (cx >= marginNegativeX[heading]) &&
                          (cx + marginPositiveX[heading] < gridWidth) &&
                          (cy >= marginNegativeY[heading]) &&
                          (cy + marginPositiveY[heading] < gridHeight);

  uint16_t alive = 0;
  uint16_t turns = 0;
  std::array<uint32_t, MAX_PRIMITIVES_PER_HEADING> endIndex{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endX{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endY{};
  for (std::size_t i = 0; i < tprims.size(); ++i) {
    const TriePrimitive& p = tprims[i];
    const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + p.endDx);
    const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + p.endDy);
    if (nx >= gridWidth || ny >= gridHeight) {
      continue;
    }
    if (onlyStraight && (p.exit & 1U) != 0U) {
      continue;
    }
    const bool turn = p.exit != heading;
    if (turn && !turnMayStart) {
      continue;
    }
    const std::size_t linear = (static_cast<std::size_t>(ny) * gridWidth) + nx;
    if (corridor.test(linear) || (ruled(linear) && !crossingOk(nx, ny, turn))) {
      continue;
    }
    const uint32_t index = stateIndex(nx, ny, static_cast<Heading>(p.exit));
    const SearchNode& n = searchScratch->at(index);
    if (n.iteration == searchScratch->iteration()) {
      if (n.closed != 0U || current.g + p.costBend >= n.g) {
        continue;
      }
    }
    endIndex[i] = index;
    endX[i] = static_cast<uint16_t>(nx);
    endY[i] = static_cast<uint16_t>(ny);
    alive |= static_cast<uint16_t>(1U << i);
    if (turn) {
      turns |= static_cast<uint16_t>(1U << i);
    }
  }
  if (alive == 0) {
    return;
  }

  const uint32_t penaltyCurrent = USE_PENALTY ? penalties[currentLinear] : 0U;
  // Only the instantiations that add the penalties write it.
  // NOLINTNEXTLINE(misc-const-correctness)
  std::array<uint32_t, MAX_TRIE_DEPTH + 2> penaltyAcc{};
  const auto nodeCount = static_cast<uint32_t>(flat.size());
  uint32_t i = 0;
  while (i < nodeCount) {
    const TrieNode tn = flat[i];
    if ((alive & tn.primitives) == 0) {
      i += tn.skip;
      continue;
    }
    std::size_t cell = 0;
    if (fastBounds) {
      cell = static_cast<std::size_t>(
          static_cast<std::ptrdiff_t>(currentLinear) + tn.linear);
    } else {
      const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + tn.dx);
      const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + tn.dy);
      if (nx >= gridWidth || ny >= gridHeight) {
        i += tn.skip;
        continue;
      }
      cell = (static_cast<std::size_t>(ny) * gridWidth) + nx;
    }
    if (corridor.test(cell)) {
      i += tn.skip;
      continue;
    }
    if (ruled(cell)) {
      const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + tn.dx);
      const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + tn.dy);
      // A cell closed to the turns that sweep it can still be open to the
      // straight step that shares it.
      const auto turning = static_cast<uint16_t>(alive & tn.primitives & turns);
      if (turning == 0 || !crossingOk(nx, ny, true)) {
        alive &= static_cast<uint16_t>(~turning);
        if ((alive & tn.primitives) == 0 || !crossingOk(nx, ny, false)) {
          i += tn.skip;
          continue;
        }
      }
    }
    if constexpr (USE_PENALTY) {
      penaltyAcc[tn.depth + 1U] = penaltyAcc[tn.depth] + penalties[cell];
    }
    if (tn.completes != NO_PRIMITIVE && (alive & (1U << tn.completes)) != 0) {
      const TriePrimitive& p = tprims[tn.completes];
      const std::size_t linear =
          (static_cast<std::size_t>(endY[tn.completes]) * gridWidth) +
          endX[tn.completes];
      // Only the instantiations that add the penalties write it.
      // NOLINTNEXTLINE(misc-const-correctness)
      uint32_t penaltyTerm = 0;
      if constexpr (USE_PENALTY) {
        uint32_t sum = penaltyAcc[tn.depth + 1U];
        if (p.sweepsOrigin != 0U) {
          sum += penaltyCurrent;
        }
        if (p.endExtra != 0U) {
          sum += penalties[linear];
        }
        penaltyTerm = (PENALTY_SCALE * sum) +
                      (PENALTY_SCALE * static_cast<uint32_t>(wires[linear]) *
                       (1U + p.sweptCount));
      }
      const uint32_t tentative = current.g + p.costBend + penaltyTerm;
      SearchNode& n = searchScratch->at(endIndex[tn.completes]);
      const bool seen = n.iteration == searchScratch->iteration();
      if (!seen || (n.closed == 0U && tentative < n.g)) {
        if (!seen) {
          n.iteration = searchScratch->iteration();
          n.closed = 0;
        }
        n.g = tentative;
        n.parentHeading = heading & 7U;
        n.primitive = p.id & 0x3FFU;
        uint32_t h = octile(endX[tn.completes], endY[tn.completes],
                            objective.target.x, objective.target.y);
        // The unavoidable turning toward the target heading is a lower bound
        // on the bend penalties still to pay.
        h += headingDistance(static_cast<Heading>(p.exit),
                             objective.target.heading) *
             searchParams.bendPenalty;
        const uint32_t f = tentative + h;
        open.push({.x = endX[tn.completes],
                   .y = endY[tn.completes],
                   .heading = static_cast<uint8_t>(p.exit),
                   .f = f,
                   .g = tentative});
      }
    }
    ++i;
  }
}

// --- Free strip ------------------------------------------------------------

CellBox DubinsRouter::freeStripAlong(const PathPoint from,
                                     const PathPoint to) const {
  if (obstacleMask == nullptr) {
    return {
        .minX = 0, .maxX = gridWidth - 1, .minY = 0, .maxY = gridHeight - 1};
  }
  const grid::BitGrid& obstacles = *obstacleMask;
  const auto w = static_cast<int64_t>(gridWidth);
  const auto h = static_cast<int64_t>(gridHeight);
  // An end off the grid moves to the nearest cell of the grid. The grid has
  // at most 65535 cells per axis, so every product below fits 64 bits.
  const auto fromX = static_cast<int64_t>(std::min(from.x, gridWidth - 1));
  const auto fromY = static_cast<int64_t>(std::min(from.y, gridHeight - 1));
  const auto toX = static_cast<int64_t>(std::min(to.x, gridWidth - 1));
  const auto toY = static_cast<int64_t>(std::min(to.y, gridHeight - 1));
  const int64_t dx = toX - fromX;
  const int64_t dy = toY - fromY;
  const int64_t lengthSquared = (dx * dx) + (dy * dy);
  if (lengthSquared == 0) {
    const auto x = static_cast<uint32_t>(fromX);
    const auto y = static_cast<uint32_t>(fromY);
    return {.minX = x, .maxX = x, .minY = y, .maxY = y};
  }
  const double length = std::sqrt(static_cast<double>(lengthSquared));
  const double inverseLength = 1.0 / length;
  // The unit normal in fixed point, so that the edges of every offset are
  // computed with integers.
  constexpr int32_t shift = 16;
  constexpr int64_t one = int64_t{1} << shift;
  const auto nx = static_cast<int64_t>(
      std::llround(-static_cast<double>(dy) * inverseLength * one));
  const auto ny = static_cast<int64_t>(
      std::llround(static_cast<double>(dx) * inverseLength * one));
  const auto roundFixed = [](const int64_t v) {
    return (v + (one >> 1)) >> shift;
  };
  const auto edgeAt = [&](const int64_t offset, int64_t& x0, int64_t& y0,
                          int64_t& x1, int64_t& y1) {
    x0 = roundFixed((fromX << shift) + (nx * offset));
    y0 = roundFixed((fromY << shift) + (ny * offset));
    x1 = roundFixed((toX << shift) + (nx * offset));
    y1 = roundFixed((toY << shift) + (ny * offset));
  };

  // The center of a cell lies at the offset cross / length from the line of
  // the segment, along the normal (-dy, dx) / length, where cross is an exact
  // integer. Growth step k on a side covers the cells whose offset on that
  // side lies in (k - 1, k] and whose projection falls on the segment. Each
  // cell has one step, so every cell of the strip is tested once.
  const auto stepOf = [&](const int64_t x, const int64_t y,
                          const int64_t side) {
    const int64_t cross = side * ((dx * (y - fromY)) - (dy * (x - fromX)));
    return cross <= 0 ? int64_t{0}
                      : static_cast<int64_t>(
                            std::ceil(static_cast<double>(cross) / length));
  };
  const auto alongSegment = [&](const int64_t x, const int64_t y) {
    const int64_t along = (dx * (x - fromX)) + (dy * (y - fromY));
    return along >= 0 && along <= lengthSquared;
  };
  // Whether the edge of a growth step lies on the grid and the cells of the
  // step are free.
  const auto stepFree = [&](const int64_t step, const int64_t side) {
    int64_t x0 = 0;
    int64_t y0 = 0;
    int64_t x1 = 0;
    int64_t y1 = 0;
    edgeAt(side * step, x0, y0, x1, y1);
    if (std::min(x0, x1) < 0 || std::max(x0, x1) >= w || std::min(y0, y1) < 0 ||
        std::max(y0, y1) >= h) {
      return false;
    }
    const double nearOffset = static_cast<double>(side * (step - 1)) * length;
    const double farOffset = static_cast<double>(side * step) * length;
    // The rows between the edge of the previous step and the edge of this
    // one.
    const double nearShift = nearOffset * static_cast<double>(dx) /
                             static_cast<double>(lengthSquared);
    const double farShift = farOffset * static_cast<double>(dx) /
                            static_cast<double>(lengthSquared);
    const int64_t firstRow =
        std::max<int64_t>(0, static_cast<int64_t>(std::floor(
                                 static_cast<double>(std::min(fromY, toY)) +
                                 std::min(nearShift, farShift))));
    const int64_t lastRow =
        std::min<int64_t>(h - 1, static_cast<int64_t>(std::ceil(
                                     static_cast<double>(std::max(fromY, toY)) +
                                     std::max(nearShift, farShift))));
    for (int64_t y = firstRow; y <= lastRow; ++y) {
      // The columns of the row that the step can cover, one cell wider on
      // either side; stepOf() and alongSegment() decide each cell exactly.
      const auto rowY = static_cast<double>(y - fromY);
      const auto originX = static_cast<double>(fromX);
      double low = 0.0;
      auto high = static_cast<double>(w - 1);
      // Narrows the columns to those between two offsets from fromX.
      const auto narrow = [&](const double a, const double b) {
        low = std::max(low, originX + std::min(a, b));
        high = std::min(high, originX + std::max(a, b));
      };
      if (dx != 0) {
        narrow((-static_cast<double>(dy) * rowY) / static_cast<double>(dx),
               (static_cast<double>(lengthSquared) -
                (static_cast<double>(dy) * rowY)) /
                   static_cast<double>(dx));
      }
      if (dy != 0) {
        const double straight = static_cast<double>(dx) * rowY;
        narrow((straight - nearOffset) / static_cast<double>(dy),
               (straight - farOffset) / static_cast<double>(dy));
      }
      const int64_t firstColumn =
          std::max<int64_t>(0, static_cast<int64_t>(std::floor(low)) - 1);
      const int64_t lastColumn =
          std::min<int64_t>(w - 1, static_cast<int64_t>(std::ceil(high)) + 1);
      for (int64_t x = firstColumn; x <= lastColumn; ++x) {
        if (stepOf(x, y, side) == step && alongSegment(x, y) &&
            obstacles.testCell(static_cast<uint32_t>(x),
                               static_cast<uint32_t>(y))) {
          return false;
        }
      }
    }
    return true;
  };

  int64_t positive = 0;
  int64_t negative = 0;
  bool growPositive = true;
  bool growNegative = true;
  while (growPositive || growNegative) {
    if (growPositive) {
      if (stepFree(positive + 1, 1)) {
        ++positive;
      } else {
        growPositive = false;
      }
    }
    if (growNegative) {
      if (stepFree(negative + 1, -1)) {
        ++negative;
      } else {
        growNegative = false;
      }
    }
  }
  int64_t c1x = 0;
  int64_t c1y = 0;
  int64_t c2x = 0;
  int64_t c2y = 0;
  int64_t c3x = 0;
  int64_t c3y = 0;
  int64_t c4x = 0;
  int64_t c4y = 0;
  edgeAt(positive, c1x, c1y, c2x, c2y);
  edgeAt(-negative, c4x, c4y, c3x, c3y);
  const auto clampX = [&](const int64_t v) {
    return static_cast<uint32_t>(std::clamp<int64_t>(v, 0, w - 1));
  };
  const auto clampY = [&](const int64_t v) {
    return static_cast<uint32_t>(std::clamp<int64_t>(v, 0, h - 1));
  };
  return {.minX = clampX(std::min({c1x, c2x, c3x, c4x})),
          .maxX = clampX(std::max({c1x, c2x, c3x, c4x})),
          .minY = clampY(std::min({c1y, c2y, c3y, c4y})),
          .maxY = clampY(std::max({c1y, c2y, c3y, c4y}))};
}

} // namespace mqt::scpd::routing
