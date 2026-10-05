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
#include <cstdlib>
#include <memory>
#include <span>
#include <stdexcept>
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

} // namespace

DubinsRouter::DubinsRouter(std::shared_ptr<const MovePrimitives> primitives,
                           SearchScratch& scratch, const SearchParams params)
    : movePrimitives(std::move(primitives)), searchScratch(&scratch),
      searchParams(params), gridWidth(scratch.width()),
      gridHeight(scratch.height()) {
  if (movePrimitives == nullptr) {
    throw std::invalid_argument("a router needs primitives");
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
  buildTables();
}

std::size_t DubinsRouter::heldBytes() const {
  const auto capacityBytes = [](const auto& container) {
    return container.capacity() * sizeof(container[0]);
  };
  std::size_t bytes =
      sizeof(DubinsRouter) + capacityBytes(staticPenalties) +
      capacityBytes(packedGrid) + capacityBytes(exempt) +
      capacityBytes(exemptCells) + capacityBytes(crossingSide) +
      capacityBytes(crossingSideCells) + capacityBytes(distanceField) +
      capacityBytes(proximityVisited) + capacityBytes(proximityFrontA) +
      capacityBytes(proximityFrontB) + capacityBytes(loopScratch.xs) +
      capacityBytes(loopScratch.ys) + open.heldBytes();
  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    bytes +=
        capacityBytes(trie[heading]) + capacityBytes(triePrimitives[heading]);
  }
  for (const std::vector<uint32_t>& bucket : fieldBuckets) {
    bytes += capacityBytes(bucket);
  }
  if (!constraints.empty()) {
    bytes += cells();
  }
  return bytes;
}

void DubinsRouter::setParams(const SearchParams& params) {
  if (params.minRadius != movePrimitives->minRadius()) {
    throw std::invalid_argument(
        "the bend radius must be the one the primitives were built with");
  }
  const bool bendChanged = searchParams.bendPenalty != params.bendPenalty;
  searchParams = params;
  if (bendChanged) {
    tablesValid = false;
    buildTables();
  }
}

// --- Grids -----------------------------------------------------------------

void DubinsRouter::attachObstacles(const grid::BitGrid* obstacles) {
  if (obstacles != nullptr && obstacles->size() != cells()) {
    throw std::invalid_argument(
        "the obstacle grid does not match the router grid");
  }
  obstacleMask = obstacles;
}

void DubinsRouter::attachCorridor(const grid::BitGrid* corridor) {
  attachCorridorUnpacked(corridor);
  rebuildPacked();
}

void DubinsRouter::attachCorridorUnpacked(const grid::BitGrid* corridor) {
  if (corridor != nullptr && corridor->size() != cells()) {
    throw std::invalid_argument("the corridor does not match the router grid");
  }
  corridorMask = corridor;
  packedValid = false;
  fieldValid = false;
}

bool DubinsRouter::corridorBlocked(const uint32_t x, const uint32_t y) const {
  return corridorMask != nullptr && corridorMask->testCell(x, y);
}

void DubinsRouter::rebuildPacked() {
  packedValid = false;
  if (corridorMask == nullptr) {
    return;
  }
  packedGrid.resize(cells());
  const std::size_t n = cells();
  for (std::size_t i = 0; i < n; ++i) {
    packedGrid[i] = static_cast<uint8_t>(
        (corridorMask->test(i) ? 0x80U : 0x00U) | staticPenalties[i]);
  }
  packedValid = true;
}

void DubinsRouter::setStaticProximity(std::vector<uint8_t> penalty) {
  if (penalty.size() != cells()) {
    throw std::invalid_argument(
        "the proximity grid does not match the router grid");
  }
  for (const uint8_t value : penalty) {
    if (value > 127U) {
      throw std::invalid_argument("a proximity penalty is at most 127");
    }
  }
  staticPenalties = std::move(penalty);
  rebuildPacked();
}

void DubinsRouter::computeStaticProximity(const uint32_t distance,
                                          const uint8_t penalty) {
  std::ranges::fill(staticPenalties, 0);
  packedValid = false;
  if (obstacleMask == nullptr || distance == 0 || penalty == 0) {
    rebuildPacked();
    return;
  }
  computeStaticProximityWindow(
      distance, penalty,
      {.minX = 0, .maxX = gridWidth - 1, .minY = 0, .maxY = gridHeight - 1});
  rebuildPacked();
}

void DubinsRouter::computeStaticProximityWindow(const uint32_t distance,
                                                const uint8_t penalty,
                                                CellBox window) {
  packedValid = false;
  if (obstacleMask == nullptr || window.minX > window.maxX ||
      window.minY > window.maxY) {
    return;
  }
  window.maxX = std::min(window.maxX, gridWidth - 1);
  window.maxY = std::min(window.maxY, gridHeight - 1);
  window.minX = std::min(window.minX, window.maxX);
  window.minY = std::min(window.minY, window.maxY);
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
  if (penalty > 127U) {
    throw std::invalid_argument("a proximity penalty is at most 127");
  }

  // The halo holds the obstacles that reach into the window: a growth of
  // distance layers never needs to expand past it.
  const uint32_t haloMinX = window.minX > distance ? window.minX - distance : 0;
  const uint32_t haloMaxX = std::min(window.maxX + distance, gridWidth - 1);
  const uint32_t haloMinY = window.minY > distance ? window.minY - distance : 0;
  const uint32_t haloMaxY = std::min(window.maxY + distance, gridHeight - 1);
  const uint32_t haloWidth = haloMaxX - haloMinX + 1;
  const std::size_t haloSize =
      static_cast<std::size_t>(haloWidth) * (haloMaxY - haloMinY + 1);
  proximityVisited.assign(haloSize, 0);
  proximityFrontA.clear();
  proximityFrontB.clear();

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
      if (obstacleMask->testCell(x, y) && proximityVisited[local(x, y)] == 0) {
        proximityVisited[local(x, y)] = 1;
        mark(x, y, penalty);
        proximityFrontA.push_back((y * gridWidth) + x);
      }
    }
  }
  static constexpr std::array<int8_t, 4> DXS = {1, -1, 0, 0};
  static constexpr std::array<int8_t, 4> DYS = {0, 0, 1, -1};
  // The divisor is 64-bit, so that the largest distance does not wrap it to
  // zero.
  const uint64_t decayDivisor = static_cast<uint64_t>(distance) + 1U;
  // The halo bounds, signed like the coordinates of a neighbor.
  const auto signedMinX = static_cast<int64_t>(haloMinX);
  const auto signedMinY = static_cast<int64_t>(haloMinY);
  const auto signedMaxX = static_cast<int64_t>(haloMaxX);
  const auto signedMaxY = static_cast<int64_t>(haloMaxY);
  for (uint32_t d = 1; d <= distance && !proximityFrontA.empty(); ++d) {
    proximityFrontB.clear();
    const uint32_t decayNumerator =
        static_cast<uint32_t>(penalty) * (distance - d + 1U);
    const auto decayed = static_cast<uint32_t>(decayNumerator / decayDivisor);
    const auto layerPenalty =
        static_cast<uint8_t>(std::max<uint32_t>(1U, decayed));
    for (const uint32_t current : proximityFrontA) {
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
        if (proximityVisited[local(ux, uy)] != 0) {
          continue;
        }
        proximityVisited[local(ux, uy)] = 1;
        mark(ux, uy, layerPenalty);
        proximityFrontB.push_back((uy * gridWidth) + ux);
      }
    }
    proximityFrontA.swap(proximityFrontB);
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

void DubinsRouter::clearOrthogonalConstraints() { constraints.clear(); }

void DubinsRouter::buildOrthogonalConstraints(const std::vector<Path>& wires,
                                              const std::vector<bool>& skip,
                                              const int expandRadius) {
  constraints.build(gridWidth, gridHeight, wires, skip, expandRadius);
}

void DubinsRouter::setSingleCrossingFeedline(const Path* feedline,
                                             const int straightRadius,
                                             const int curveRadius) {
  singleCrossing = feedline;
  singleCrossingStraightRadius = straightRadius;
  singleCrossingCurveRadius = curveRadius;
}

bool DubinsRouter::beginSingleCrossingOverlay(const PathPoint& source,
                                              const PathPoint& target) {
  if (singleCrossing == nullptr || singleCrossingStraightRadius <= 0 ||
      singleCrossing->size() < 2) {
    return false;
  }
  if (crossingSide.size() != cells()) {
    crossingSide.assign(cells(), 0);
  }
  endSingleCrossingOverlay();

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
        uint8_t& m = crossingSide[(static_cast<std::size_t>(y) * gridWidth) +
                                  static_cast<std::size_t>(x)];
        if (m == 0U) {
          m = value;
          crossingSideCells.push_back(static_cast<uint32_t>((y * w) + x));
        } else if ((m & SIDE_STRAIGHT) != 0U && m != value) {
          // Two straights with different exit headings, a bend: block. A
          // curve over a straight's corridor: the corridor wins.
          if (!isCurve) {
            m = SIDE_FAR;
          }
        }
      }
    }
  };

  // Straight zones: the exit run travels away from the feedline, toward the
  // far side; the state heading that travels that way is its reverse.
  for (const PathSegment& segment : segmented.segments) {
    if (!segment.straight || segment.cells.empty()) {
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

  // Curve zones and the pin zone: never enterable on the far side.
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
  // straight run of two or more cells. The zone centres on the last point of
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
  crossingSideActive = !crossingSideCells.empty();
  return crossingSideActive;
}

void DubinsRouter::endSingleCrossingOverlay() {
  for (const uint32_t index : crossingSideCells) {
    crossingSide[index] = 0;
  }
  crossingSideCells.clear();
  crossingSideActive = false;
}

void DubinsRouter::setCrossingExemption(const std::vector<uint32_t>& cells) {
  for (const uint32_t index : exemptCells) {
    exempt[index] = 0;
  }
  exemptCells.clear();
  if (cells.empty()) {
    return;
  }
  if (exempt.size() != this->cells()) {
    exempt.assign(this->cells(), 0);
  }
  for (const uint32_t index : cells) {
    if (index < exempt.size() && exempt[index] == 0U) {
      exempt[index] = 1;
      exemptCells.push_back(index);
    }
  }
}

bool DubinsRouter::canCrossOrthogonal(const uint32_t x, const uint32_t y,
                                      const Heading heading) const {
  if (x >= gridWidth || y >= gridHeight) {
    return false;
  }
  const std::size_t index = (static_cast<std::size_t>(y) * gridWidth) + x;
  if (!exemptCells.empty() && exempt[index] != 0U) {
    return true;
  }
  if (crossingSideActive) {
    const uint8_t side = crossingSide[index];
    if (side != 0U) {
      if ((side & SIDE_STRAIGHT) == 0U) {
        return false;
      }
      if ((side & 7U) != (heading & 7U)) {
        return false;
      }
    }
  }
  return constraints.allowed(x, y, heading);
}

bool DubinsRouter::crossingAllowedOrthogonal(const uint32_t x, const uint32_t y,
                                             const Heading heading) const {
  return canCrossOrthogonal(x, y, heading);
}

uint8_t DubinsRouter::constraintMaskAt(const uint32_t x,
                                       const uint32_t y) const {
  return constraints.maskAt(x, y);
}

// --- Search tables
// -------------------------------------------------------------

void DubinsRouter::buildTables() {
  if (tablesValid) {
    return;
  }
  const uint32_t bend = searchParams.bendPenalty;
  const auto w = static_cast<int32_t>(gridWidth);

  struct TempNode {
    int8_t dx = 0;
    int8_t dy = 0;
    uint16_t primitives = 0;
    uint8_t completes = NO_PRIMITIVE;
    std::vector<uint32_t> children;
  };

  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    auto& tprims = triePrimitives[heading];
    auto& tflat = trie[heading];
    tprims.clear();
    tflat.clear();
    std::vector<TempNode> nodes;
    std::vector<uint32_t> roots;
    int32_t left = 0;
    int32_t right = 0;
    int32_t up = 0;
    int32_t down = 0;

    const auto prims = movePrimitives->of(static_cast<Heading>(heading));
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
      // definition, so a change to it can change the way a search returns.
      const auto moveCost = static_cast<uint32_t>(
          static_cast<double>(static_cast<float>(p.cost)) * 100.0);
      tp.costBend =
          moveCost +
          (headingDistance(static_cast<Heading>(heading), tp.exit) * bend);

      left = std::max<int32_t>(left, -p.dx);
      right = std::max<int32_t>(right, p.dx);
      up = std::max<int32_t>(up, -p.dy);
      down = std::max<int32_t>(down, p.dy);
      for (const CellOffset& c : p.swept) {
        left = std::max<int32_t>(left, -c.dx);
        right = std::max<int32_t>(right, c.dx);
        up = std::max<int32_t>(up, -c.dy);
        down = std::max<int32_t>(down, c.dy);
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

    marginLeft[heading] = static_cast<uint32_t>(left);
    marginRight[heading] = static_cast<uint32_t>(right);
    marginUp[heading] = static_cast<uint32_t>(up);
    marginDown[heading] = static_cast<uint32_t>(down);

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
  tablesValid = true;
}

// --- Distance field
// --------------------------------------------------------------

void DubinsRouter::buildDistanceField(const uint32_t tx, const uint32_t ty) {
  fieldValid = false;
  if (corridorMask == nullptr || tx >= gridWidth || ty >= gridHeight) {
    return;
  }
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
  const uint32_t stampHigh = fieldStamp << FIELD_DISTANCE_BITS;
  for (auto& bucket : fieldBuckets) {
    bucket.clear();
  }
  const grid::BitGrid& corridor = *corridorMask;
  uint32_t* field = distanceField.data();

  // The search reads and writes the field through a raw pointer into its
  // flat table.
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  field[(static_cast<std::size_t>(ty) * gridWidth) + tx] = stampHigh;
  fieldBuckets[0].push_back((ty << 16U) | tx);
  std::size_t pending = 1;
  uint32_t d = 0;
  while (pending != 0) {
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
          if ((nv >> FIELD_DISTANCE_BITS) == fieldStamp &&
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
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  fieldValid = true;
}

// --- States and paths
// ----------------------------------------------------------

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
  return {.x = static_cast<uint32_t>(static_cast<int64_t>(point.x) -
                                     (v.dx * length)),
          .y = static_cast<uint32_t>(static_cast<int64_t>(point.y) -
                                     (v.dy * length)),
          .heading = static_cast<Heading>(point.heading & 7U),
          .primitive = 0};
}

Path DubinsRouter::straightStub(const PathPoint point, const bool isTarget,
                                const uint32_t length) const {
  const int64_t signedLength =
      isTarget ? static_cast<int64_t>(length) : -static_cast<int64_t>(length);
  const HeadingVector v = headingVector(point.heading);
  const uint16_t straight = movePrimitives->straight(point.heading);
  Path stub;
  for (int64_t i = 0; i <= std::abs(signedLength); ++i) {
    const int64_t step = (signedLength < 0) ? i : -i;
    stub.push_back({.x = static_cast<uint32_t>(static_cast<int64_t>(point.x) +
                                               (v.dx * step)),
                    .y = static_cast<uint32_t>(static_cast<int64_t>(point.y) +
                                               (v.dy * step)),
                    .heading = static_cast<Heading>(point.heading & 7U),
                    .primitive = straight});
  }
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
  // The tail starts on the cell the search ended at. Without a searched way,
  // the search started there too, and the head already ends on that cell.
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

// --- The free search
// -------------------------------------------------------------

Path DubinsRouter::route(const RoutingObjective& objective,
                         const bool usePenalty, const bool onlyStraight) {
  if (corridorMask == nullptr) {
    throw std::logic_error("route needs an attached corridor");
  }
  if (usePenalty && wireProximity == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  RoutingObjective moved;
  moved.source = sanitize(objective.source, false);
  moved.target = sanitize(objective.target, true);
  if (!inGrid(moved.source) || !inGrid(moved.target)) {
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
  buildTables();
  open.clear();
  // The start meets the corridor test that every cell a move enters meets.
  const std::size_t startLinear =
      (static_cast<std::size_t>(objective.source.y) * gridWidth) +
      objective.source.x;
  if constexpr (PACKED) {
    if ((packedGrid[startLinear] & 0x80U) != 0U) {
      return {};
    }
  } else if (corridorMask->test(startLinear)) {
    return {};
  }
  const uint8_t* blockMap = PACKED ? packedGrid.data() : nullptr;
  const uint8_t* penaltyMap =
      PACKED ? packedGrid.data() : staticPenalties.data();
  constexpr uint8_t blockMask = PACKED ? uint8_t{0x80} : uint8_t{0xFF};
  constexpr uint8_t penaltyMask = PACKED ? uint8_t{0x7F} : uint8_t{0xFF};
  const uint8_t* wireMap =
      wireProximity != nullptr ? wireProximity->data() : nullptr;

  for (uint32_t a = 0; a < NUM_HEADINGS; ++a) {
    bendBoundByHeading[a] = bendLowerBound
                                ? headingDistance(static_cast<Heading>(a),
                                                  objective.target.heading) *
                                      searchParams.bendPenalty
                                : 0U;
  }

  const bool useField = activeHeuristic == Heuristic::DistanceField;
  if (useField) {
    buildDistanceField(objective.target.x, objective.target.y);
  } else {
    fieldValid = false;
  }
  const bool fieldReady = useField && fieldValid;

  const uint32_t startIndex = stateIndex(objective.source.x, objective.source.y,
                                         objective.source.heading);
  searchScratch->setStart(startIndex);
  open.push({.x = static_cast<uint16_t>(objective.source.x),
             .y = static_cast<uint16_t>(objective.source.y),
             .heading = objective.source.heading,
             .primitive = 0,
             .f = 0,
             .g = 0},
            0);

  while (!open.empty()) {
    const QueueEntry current = open.pop();
    if (const QueueEntry* next = open.peek()) {
      const uint32_t nextIndex = stateIndex(next->x, next->y, next->heading);
      if (nextIndex < searchScratch->size()) {
        // The prefetch takes the address of the record in the flat table.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        prefetchForWrite(searchScratch->data() + nextIndex);
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
                                    fieldReady);
  }
  return {};
}

template <bool PACKED, bool USE_PENALTY>
void DubinsRouter::expandFree(const QueueEntry& current,
                              const RoutingObjective& objective,
                              const bool onlyStraight, const uint8_t* blockMap,
                              const uint8_t* penaltyMap,
                              const uint8_t blockMask,
                              const uint8_t penaltyMask, const uint8_t* wireMap,
                              const bool useField) {
  const uint32_t heading = current.heading & 7U;
  const auto& tprims = triePrimitives[heading];
  const auto& tflat = trie[heading];
  const uint32_t cx = current.x;
  const uint32_t cy = current.y;
  const std::size_t currentLinear =
      (static_cast<std::size_t>(cy) * gridWidth) + cx;
  const uint32_t* field = distanceField.data();
  const grid::BitGrid* corridor = corridorMask;

  const auto blocked = [&](const std::size_t index) {
    if constexpr (PACKED) {
      // The test reads the packed working grid through a raw pointer.
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      return (blockMap[index] & blockMask) != 0U;
    } else {
      return corridor->test(index);
    }
  };

  const bool fastBounds =
      (cx >= marginLeft[heading]) && (cx + marginRight[heading] < gridWidth) &&
      (cy >= marginUp[heading]) && (cy + marginDown[heading] < gridHeight);

  // The candidates: every primitive whose end is in the grid, allowed, not
  // blocked, not dominated and, with a field, able to reach the target.
  uint16_t alive = 0;
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
    if (blocked(linear)) {
      continue;
    }
    const uint32_t index = stateIndex(nx, ny, static_cast<Heading>(p.exit));
    const SearchNode& n = searchScratch->at(index);
    if (n.iteration == searchScratch->iteration()) {
      if (n.closed != 0U || current.g + p.costBend >= n.g) {
        continue;
      }
    }
    uint32_t hRaw = 0;
    if (useField) {
      // The heuristic reads the distance field through a raw pointer.
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      const uint32_t hv = field[linear];
      if ((hv >> FIELD_DISTANCE_BITS) != fieldStamp) {
        continue; // Provably unable to reach the target.
      }
      hRaw = hv & FIELD_DISTANCE_MASK;
    }
    endField[i] = hRaw;
    endIndex[i] = index;
    endX[i] = static_cast<uint16_t>(nx);
    endY[i] = static_cast<uint16_t>(ny);
    alive |= static_cast<uint16_t>(1U << i);
  }
  if (alive == 0) {
    return;
  }

  // The trie walk reads the trie and the penalty grids through raw pointers
  // into their flat tables.
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const uint32_t penaltyCurrent =
      USE_PENALTY ? (penaltyMap[currentLinear] & penaltyMask) : 0U;
  // Only the instantiations that add the penalties write it.
  // NOLINTNEXTLINE(misc-const-correctness)
  std::array<uint32_t, MAX_TRIE_DEPTH + 2> penaltyAcc{};

  const TrieNode* flat = tflat.data();
  const auto nodeCount = static_cast<uint32_t>(tflat.size());
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
    if (blocked(cell)) {
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
          sum += penaltyMap[linear] & penaltyMask;
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
                   .primitive = p.id,
                   .f = f,
                   .g = tentative},
                  f);
      }
    }
    ++i;
  }
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

// --- The orthogonal search
// ------------------------------------------------------

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
  RoutingObjective moved;
  moved.source = sanitize(objective.source, false);
  moved.target = sanitize(objective.target, true);
  if (!inGrid(moved.source) || !inGrid(moved.target)) {
    return {};
  }
  searchScratch->beginSearch();
  const bool overlay =
      beginSingleCrossingOverlay(objective.source, objective.target);
  Path path = guarded([&] {
    return usePenalty ? searchOrthogonal<true>(moved, objective.source,
                                               objective.target, onlyStraight)
                      : searchOrthogonal<false>(moved, objective.source,
                                                objective.target, onlyStraight);
  });
  if (overlay) {
    endSingleCrossingOverlay();
  }
  return path;
}

template <bool USE_PENALTY>
Path DubinsRouter::searchOrthogonal(const RoutingObjective& objective,
                                    const PathPoint& source,
                                    const PathPoint& target,
                                    const bool onlyStraight) {
  buildTables();
  open.clear();
  // The start meets the corridor test that every cell a move enters meets.
  // The crossing rules judge the heading a cell is entered on, and the search
  // does not enter its start.
  if (corridorMask->testCell(objective.source.x, objective.source.y)) {
    return {};
  }
  const uint32_t startIndex = stateIndex(objective.source.x, objective.source.y,
                                         objective.source.heading);
  searchScratch->setStart(startIndex);
  open.push({.x = static_cast<uint16_t>(objective.source.x),
             .y = static_cast<uint16_t>(objective.source.y),
             .heading = objective.source.heading,
             .primitive = 0,
             .f = 0,
             .g = 0},
            0);
  while (!open.empty()) {
    const QueueEntry current = open.pop();
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
  const auto& tflat = trie[heading];
  const uint32_t cx = current.x;
  const uint32_t cy = current.y;
  const std::size_t currentLinear =
      (static_cast<std::size_t>(cy) * gridWidth) + cx;
  const grid::BitGrid& corridor = *corridorMask;

  const auto cellOk = [&](const uint32_t x, const uint32_t y) {
    if (corridor.testCell(x, y)) {
      return false;
    }
    return canCrossOrthogonal(x, y, static_cast<Heading>(heading));
  };
  const bool fastBounds =
      (cx >= marginLeft[heading]) && (cx + marginRight[heading] < gridWidth) &&
      (cy >= marginUp[heading]) && (cy + marginDown[heading] < gridHeight);

  uint16_t alive = 0;
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
    if (!cellOk(nx, ny)) {
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
  }
  if (alive == 0) {
    return;
  }

  const uint32_t penaltyCurrent =
      USE_PENALTY ? staticPenalties[currentLinear] : 0U;
  // Only the instantiations that add the penalties write it.
  // NOLINTNEXTLINE(misc-const-correctness)
  std::array<uint32_t, MAX_TRIE_DEPTH + 2> penaltyAcc{};
  const TrieNode* flat = tflat.data();
  const auto nodeCount = static_cast<uint32_t>(tflat.size());
  uint32_t i = 0;
  while (i < nodeCount) {
    // The walk reads the trie through a raw pointer into its flat table.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const TrieNode tn = flat[i];
    if ((alive & tn.primitives) == 0) {
      i += tn.skip;
      continue;
    }
    const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + tn.dx);
    const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + tn.dy);
    if (!fastBounds && (nx >= gridWidth || ny >= gridHeight)) {
      i += tn.skip;
      continue;
    }
    if (!cellOk(nx, ny)) {
      i += tn.skip;
      continue;
    }
    const std::size_t cell = (static_cast<std::size_t>(ny) * gridWidth) + nx;
    if constexpr (USE_PENALTY) {
      penaltyAcc[tn.depth + 1U] = penaltyAcc[tn.depth] + staticPenalties[cell];
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
          sum += staticPenalties[linear];
        }
        penaltyTerm =
            (PENALTY_SCALE * sum) +
            (PENALTY_SCALE * static_cast<uint32_t>(wirePenalty(linear)) *
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
                   .primitive = p.id,
                   .f = f,
                   .g = tentative},
                  f);
      }
    }
    ++i;
  }
}

// --- Free strip
// -----------------------------------------------------------------

CellBox DubinsRouter::freeStripAlong(const PathPoint from,
                                     const PathPoint to) const {
  if (obstacleMask == nullptr) {
    return {
        .minX = 0, .maxX = gridWidth - 1, .minY = 0, .maxY = gridHeight - 1};
  }
  const grid::BitGrid& obstacles = *obstacleMask;
  const auto w = static_cast<int64_t>(gridWidth);
  const auto h = static_cast<int64_t>(gridHeight);
  const auto edgeFree = [&](const int64_t x0, const int64_t y0,
                            const int64_t x1, const int64_t y1) {
    if (std::min(x0, x1) < 0 || std::max(x0, x1) >= w || std::min(y0, y1) < 0 ||
        std::max(y0, y1) >= h) {
      return false;
    }
    const int64_t dx = std::abs(x1 - x0);
    const int64_t dy = -std::abs(y1 - y0);
    const int64_t sx = (x0 < x1) ? 1 : -1;
    const int64_t sy = (y0 < y1) ? 1 : -1;
    int64_t err = dx + dy;
    int64_t x = x0;
    int64_t y = y0;
    while (true) {
      if (obstacles.testCell(static_cast<uint32_t>(x),
                             static_cast<uint32_t>(y))) {
        return false;
      }
      if (x == x1 && y == y1) {
        break;
      }
      const int64_t e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
    }
    return true;
  };

  const double dx = static_cast<double>(to.x) - static_cast<double>(from.x);
  const double dy = static_cast<double>(to.y) - static_cast<double>(from.y);
  const double length2 = (dx * dx) + (dy * dy);
  if (length2 == 0.0) {
    return {.minX = from.x, .maxX = from.x, .minY = from.y, .maxY = from.y};
  }
  const double inverseLength = 1.0 / std::sqrt(length2);
  // The unit normal in fixed point, so that the edges of every offset are
  // computed with integers.
  constexpr int32_t shift = 16;
  constexpr int64_t one = int64_t{1} << shift;
  const auto nx = static_cast<int64_t>(std::llround(-dy * inverseLength * one));
  const auto ny = static_cast<int64_t>(std::llround(dx * inverseLength * one));
  const int64_t fromX = static_cast<int64_t>(from.x) << shift;
  const int64_t fromY = static_cast<int64_t>(from.y) << shift;
  const int64_t toX = static_cast<int64_t>(to.x) << shift;
  const int64_t toY = static_cast<int64_t>(to.y) << shift;
  const auto roundFixed = [](const int64_t v) {
    return (v + (one >> 1)) >> shift;
  };
  const auto edgeAt = [&](const int64_t offset, int64_t& x0, int64_t& y0,
                          int64_t& x1, int64_t& y1) {
    x0 = roundFixed(fromX + (nx * offset));
    y0 = roundFixed(fromY + (ny * offset));
    x1 = roundFixed(toX + (nx * offset));
    y1 = roundFixed(toY + (ny * offset));
  };
  int64_t positive = 0;
  int64_t negative = 0;
  bool growPositive = true;
  bool growNegative = true;
  int64_t x0 = 0;
  int64_t y0 = 0;
  int64_t x1 = 0;
  int64_t y1 = 0;
  while (growPositive || growNegative) {
    if (growPositive) {
      edgeAt(positive + 1, x0, y0, x1, y1);
      if (edgeFree(x0, y0, x1, y1)) {
        ++positive;
      } else {
        growPositive = false;
      }
    }
    if (growNegative) {
      edgeAt(-(negative + 1), x0, y0, x1, y1);
      if (edgeFree(x0, y0, x1, y1)) {
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
