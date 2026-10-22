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
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
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
  return (cross > 0) - (cross < 0);
}

} // namespace

DubinsRouter::DubinsRouter(std::shared_ptr<const MovePrimitives> primitives,
                           SearchScratch& scratch, const SearchParams params)
    : primitives_(std::move(primitives)), scratch_(scratch), params_(params),
      width_(scratch.width()), height_(scratch.height()) {
  if (primitives_ == nullptr) {
    throw std::invalid_argument("a router needs primitives");
  }
  if (width_ == 0 || height_ == 0 || width_ >= 65536U || height_ >= 65536U) {
    throw std::invalid_argument(
        "a router grid needs 1 to 65535 cells per axis");
  }
  if (params_.minRadius != primitives_->minRadius()) {
    throw std::invalid_argument(
        "the bend radius must be the one the primitives were built with");
  }
  static_.assign(cells(), 0);
  buildTables();
}

void DubinsRouter::setParams(const SearchParams& params) {
  if (params.minRadius != primitives_->minRadius()) {
    throw std::invalid_argument(
        "the bend radius must be the one the primitives were built with");
  }
  const bool bendChanged = params_.bendPenalty != params.bendPenalty;
  params_ = params;
  if (bendChanged) {
    tablesValid_ = false;
    buildTables();
  }
}

// --- The analytic turn bound -------------------------------------------------

void DubinsRouter::setTurnLowerBound(const bool on) {
  turnLowerBound_ = on;
  if (on && analytic_ == nullptr) {
    analytic_ = std::make_unique<AnalyticDubins>(*primitives_);
    // The model's ways turn at most `cap()` eighths, so they hold at most
    // that many turning moves; no partial sum of their moves reaches
    // further on either axis than this.
    int32_t longest = 0;
    for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
      for (const Primitive& p : primitives_->of(heading)) {
        if (p.exitHeading != heading) {
          longest = std::max(
              {longest, std::abs(int32_t{p.dx}), std::abs(int32_t{p.dy})});
        }
      }
    }
    turnReach_ = static_cast<int32_t>(analytic_->cap()) * longest;
  }
}

uint32_t DubinsRouter::minTurnsTo(const uint32_t x, const uint32_t y,
                                  const Heading heading,
                                  const PathPoint& goal) {
  const int32_t dx = static_cast<int32_t>(goal.x) - static_cast<int32_t>(x);
  const int32_t dy = static_cast<int32_t>(goal.y) - static_cast<int32_t>(y);
  const auto goalHeading = static_cast<Heading>(goal.heading & 7U);
  if (std::abs(dx) <= TURN_WINDOW && std::abs(dy) <= TURN_WINDOW) {
    constexpr auto SPAN = static_cast<std::size_t>((2 * TURN_WINDOW) + 1);
    auto& window = turnWindow_[goalHeading];
    if (window.empty()) {
      window.assign(SPAN * SPAN * NUM_HEADINGS, TURN_UNKNOWN);
    }
    const std::size_t at =
        ((((static_cast<std::size_t>(dy + TURN_WINDOW) * SPAN) +
           static_cast<std::size_t>(dx + TURN_WINDOW)) *
          NUM_HEADINGS) +
         (heading & 7U));
    if (window[at] == TURN_UNKNOWN) {
      const PathPoint from{.x = x, .y = y, .heading = heading, .primitive = 0};
      window[at] = static_cast<uint8_t>(analytic_->minTurns(from, goal));
    }
    return window[at];
  }
  // Far from the goal, `AnalyticDubins::minTurns` reads the displacement
  // only through the direction class of the displacement less a partial
  // sum of turning moves, which is at most `reach` on either axis. The
  // class is set by the signs of the four forms x, y, x + y and x - y, and
  // a partial sum moves each form by at most `2 * reach`. A form whose
  // value is above that keeps its sign; a form within it is "near". Beyond
  // `4 * reach` on either axis at most one form is near. So two
  // displacements with the same near form and value, and the same signs of
  // the other forms, have the same answer. The far table holds one answer
  // per such class, computed at a representative displacement.
  const int32_t reach = turnReach_;
  if (4 * reach >= TURN_WINDOW) {
    const PathPoint from{.x = x, .y = y, .heading = heading, .primitive = 0};
    return analytic_->minTurns(from, goal);
  }
  const int32_t near = 2 * reach;
  const int32_t values = (2 * near) + 1;
  const int32_t out = (2 * near) + 1;
  const std::array<int32_t, 4> forms{dx, dy, dx + dy, dx - dy};
  int32_t slot = -1;
  int32_t ox = 0;
  int32_t oy = 0;
  for (int32_t form = 0; form < 4 && slot < 0; ++form) {
    const int32_t c = forms[static_cast<std::size_t>(form)];
    if (std::abs(c) > near) {
      continue;
    }
    // The side of the line the displacement lies on, along the line.
    const int32_t side = form == 0 ? (dy > 0 ? 1 : -1) : (dx > 0 ? 1 : -1);
    const int32_t far = side * ((4 * reach) + 1);
    switch (form) {
    case 0:
      ox = c;
      oy = far;
      break;
    case 1:
      ox = far;
      oy = c;
      break;
    case 2:
      ox = far;
      oy = c - far;
      break;
    default:
      ox = far;
      oy = far - c;
      break;
    }
    slot = (((form * 2) + (side > 0 ? 1 : 0)) * values) + (c + near);
  }
  if (slot < 0) {
    // No form is near: the displacement lies inside one of the eight open
    // sectors, far from its edges.
    static constexpr std::array<std::array<int32_t, 2>, 8> SECTORS{{{2, 1},
                                                                    {1, 2},
                                                                    {-1, 2},
                                                                    {-2, 1},
                                                                    {-2, -1},
                                                                    {-1, -2},
                                                                    {1, -2},
                                                                    {2, -1}}};
    const auto sign = [](const int32_t v) { return (v > 0) - (v < 0); };
    for (int32_t k = 0; k < 8; ++k) {
      const auto& v = SECTORS[static_cast<std::size_t>(k)];
      if (sign(v[0]) == sign(dx) && sign(v[1]) == sign(dy) &&
          sign(v[0] + v[1]) == sign(dx + dy) &&
          sign(v[0] - v[1]) == sign(dx - dy)) {
        ox = v[0] * out;
        oy = v[1] * out;
        slot = (8 * values) + k;
        break;
      }
    }
  }
  auto& table = turnFarTable_[goalHeading];
  if (table.empty()) {
    table.assign(static_cast<std::size_t>((8 * values) + 8) * NUM_HEADINGS,
                 TURN_UNKNOWN);
  }
  const std::size_t at =
      (static_cast<std::size_t>(slot) * NUM_HEADINGS) + (heading & 7U);
  if (table[at] == TURN_UNKNOWN) {
    // Lifted so that both ends stay inside the unsigned coordinates.
    constexpr int32_t LIFT = 1 << 20U;
    const PathPoint from{.x = static_cast<uint32_t>(LIFT),
                         .y = static_cast<uint32_t>(LIFT),
                         .heading = heading,
                         .primitive = 0};
    const PathPoint to{.x = static_cast<uint32_t>(LIFT + ox),
                       .y = static_cast<uint32_t>(LIFT + oy),
                       .heading = goalHeading,
                       .primitive = 0};
    table[at] = static_cast<uint8_t>(analytic_->minTurns(from, to));
  }
  return table[at];
}

uint32_t DubinsRouter::turnBoundOf(const uint32_t x, const uint32_t y,
                                   const Heading heading) {
  uint32_t least = UINT32_MAX;
  for (const PathPoint& goal : turnGoals_) {
    least = std::min(least, minTurnsTo(x, y, heading, goal));
  }
  return least;
}

// --- Grids -----------------------------------------------------------------

void DubinsRouter::attachObstacles(const grid::BitGrid* obstacles) {
  if (obstacles != nullptr && obstacles->size() != cells()) {
    throw std::invalid_argument(
        "the obstacle grid does not match the router grid");
  }
  obstacles_ = obstacles;
}

void DubinsRouter::attachCorridor(const grid::BitGrid* corridor) {
  attachCorridorUnpacked(corridor);
  rebuildPacked();
}

void DubinsRouter::attachCorridorUnpacked(const grid::BitGrid* corridor) {
  if (corridor != nullptr && corridor->size() != cells()) {
    throw std::invalid_argument("the corridor does not match the router grid");
  }
  corridor_ = corridor;
  packedValid_ = false;
  fieldValid_ = false;
}

bool DubinsRouter::corridorBlocked(const uint32_t x, const uint32_t y) const {
  return corridor_ != nullptr && corridor_->testCell(x, y);
}

void DubinsRouter::rebuildPacked() {
  packedValid_ = false;
  if (corridor_ == nullptr) {
    return;
  }
  const std::size_t n = cells();
  const auto& words = corridor_->words();
  const bool whole = packedStale_ || packed_.size() != n ||
                     packedWords_.size() != words.size();
  packed_.resize(n);
  packedWords_.resize(words.size());
  // One word of the corridor at a time, and only the words that differ from
  // the corridor packed last: every other cell holds what it would be
  // packed to.
  for (std::size_t at = 0; at < words.size(); ++at) {
    const uint64_t word = words[at];
    if (!whole && word == packedWords_[at]) {
      continue;
    }
    packedWords_[at] = word;
    const std::size_t first = at * 64;
    const std::size_t last = std::min(n, first + 64);
    for (std::size_t i = first; i < last; ++i) {
      packed_[i] = static_cast<uint8_t>((((word >> (i - first)) & 1U) << 7U) |
                                        static_[i]);
    }
  }
  packedStale_ = false;
  packedValid_ = true;
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
  static_ = std::move(penalty);
  packedStale_ = true;
  rebuildPacked();
}

void DubinsRouter::computeStaticProximity(const uint32_t distance,
                                          const uint8_t penalty) {
  std::fill(static_.begin(), static_.end(), 0);
  packedValid_ = false;
  packedStale_ = true;
  if (obstacles_ == nullptr || distance == 0 || penalty == 0) {
    rebuildPacked();
    return;
  }
  computeStaticProximityWindow(
      distance, penalty,
      {.minX = 0, .maxX = width_ - 1, .minY = 0, .maxY = height_ - 1});
  rebuildPacked();
}

void DubinsRouter::computeStaticProximityWindow(const uint32_t distance,
                                                const uint8_t penalty,
                                                CellBox window) {
  packedValid_ = false;
  packedStale_ = true;
  if (obstacles_ == nullptr || window.minX > window.maxX ||
      window.minY > window.maxY) {
    return;
  }
  window.maxX = std::min(window.maxX, width_ - 1);
  window.maxY = std::min(window.maxY, height_ - 1);
  window.minX = std::min(window.minX, window.maxX);
  window.minY = std::min(window.minY, window.maxY);
  for (uint32_t y = window.minY; y <= window.maxY; ++y) {
    std::fill(static_.begin() +
                  static_cast<std::ptrdiff_t>(
                      (static_cast<std::size_t>(y) * width_) + window.minX),
              static_.begin() +
                  static_cast<std::ptrdiff_t>(
                      (static_cast<std::size_t>(y) * width_) + window.maxX + 1),
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
  const uint32_t haloMaxX = std::min(window.maxX + distance, width_ - 1);
  const uint32_t haloMinY = window.minY > distance ? window.minY - distance : 0;
  const uint32_t haloMaxY = std::min(window.maxY + distance, height_ - 1);
  const uint32_t haloWidth = haloMaxX - haloMinX + 1;
  const std::size_t haloSize =
      static_cast<std::size_t>(haloWidth) * (haloMaxY - haloMinY + 1);
  proximityVisited_.assign(haloSize, 0);
  proximityFrontA_.clear();
  proximityFrontB_.clear();

  const auto local = [&](const uint32_t x, const uint32_t y) {
    return (static_cast<std::size_t>(y - haloMinY) * haloWidth) +
           (x - haloMinX);
  };
  const auto mark = [&](const uint32_t x, const uint32_t y,
                        const uint8_t value) {
    if (x >= window.minX && x <= window.maxX && y >= window.minY &&
        y <= window.maxY) {
      static_[(static_cast<std::size_t>(y) * width_) + x] = value;
    }
  };
  for (uint32_t y = haloMinY; y <= haloMaxY; ++y) {
    for (uint32_t x = haloMinX; x <= haloMaxX; ++x) {
      if (obstacles_->testCell(x, y) && proximityVisited_[local(x, y)] == 0) {
        proximityVisited_[local(x, y)] = 1;
        mark(x, y, penalty);
        proximityFrontA_.push_back((y * width_) + x);
      }
    }
  }
  static constexpr int8_t DXS[4] = {1, -1, 0, 0};
  static constexpr int8_t DYS[4] = {0, 0, 1, -1};
  for (uint32_t d = 1; d <= distance && !proximityFrontA_.empty(); ++d) {
    proximityFrontB_.clear();
    const uint32_t decayed =
        (static_cast<uint32_t>(penalty) * (distance - d + 1U)) /
        (distance + 1U);
    const auto layerPenalty =
        static_cast<uint8_t>(std::max<uint32_t>(1U, decayed));
    for (const uint32_t current : proximityFrontA_) {
      const uint32_t cx = current % width_;
      const uint32_t cy = current / width_;
      for (int i = 0; i < 4; ++i) {
        const int64_t nx = static_cast<int64_t>(cx) + DXS[i];
        const int64_t ny = static_cast<int64_t>(cy) + DYS[i];
        if (nx < haloMinX || ny < haloMinY || nx > haloMaxX || ny > haloMaxY) {
          continue;
        }
        const auto ux = static_cast<uint32_t>(nx);
        const auto uy = static_cast<uint32_t>(ny);
        if (proximityVisited_[local(ux, uy)] != 0) {
          continue;
        }
        proximityVisited_[local(ux, uy)] = 1;
        mark(ux, uy, layerPenalty);
        proximityFrontB_.push_back((uy * width_) + ux);
      }
    }
    proximityFrontA_.swap(proximityFrontB_);
  }
}

void DubinsRouter::attachWireProximity(const std::vector<uint16_t>* penalty) {
  if (penalty != nullptr && penalty->size() != cells()) {
    throw std::invalid_argument(
        "the wire proximity grid does not match the router grid");
  }
  wire_ = penalty;
}

uint16_t DubinsRouter::wirePenalty(const std::size_t index) const {
  return wire_ != nullptr ? (*wire_)[index] : 0;
}

// --- Crossing constraints ----------------------------------------------------

void DubinsRouter::clearOrthogonalConstraints() { constraints_.clear(); }

void DubinsRouter::buildOrthogonalConstraints(const std::vector<Path>& wires,
                                              const std::vector<bool>& skip,
                                              const int expandRadius) {
  constraints_.build(width_, height_, wires, skip, expandRadius);
}

void DubinsRouter::setSingleCrossingFeedline(const Path* feedline,
                                             const int straightRadius,
                                             const int curveRadius) {
  singleCrossing_ = feedline;
  singleCrossingStraightRadius_ = straightRadius;
  singleCrossingCurveRadius_ = curveRadius;
}

bool DubinsRouter::beginSingleCrossingOverlay(const PathPoint& source,
                                              const PathPoint& target) {
  if (singleCrossing_ == nullptr || singleCrossingStraightRadius_ <= 0 ||
      singleCrossing_->size() < 2) {
    return false;
  }
  if (crossingSide_.size() != cells()) {
    crossingSide_.assign(cells(), 0);
  }
  endSingleCrossingOverlay();

  const int64_t straightRadius = singleCrossingStraightRadius_;
  const int64_t curveRadius = singleCrossingCurveRadius_;
  const SegmentedPath segmented =
      reconstructSegments(*primitives_, *singleCrossing_);
  const int64_t sx = source.x;
  const int64_t sy = source.y;
  const int64_t tx = target.x;
  const int64_t ty = target.y;
  const auto w = static_cast<int64_t>(width_);
  const auto h = static_cast<int64_t>(height_);

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
        uint8_t& m = crossingSide_[(static_cast<std::size_t>(y) * width_) +
                                   static_cast<std::size_t>(x)];
        if (m == 0U) {
          m = value;
          crossingSideCells_.push_back(static_cast<uint32_t>((y * w) + x));
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
  const auto blockRun = [&](const std::vector<PathPoint>& run) {
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
        dx = d.dx;
        dy = d.dy;
      }
      const int farSign = farSideOf(a.x, a.y, dx, dy);
      if (farSign == 0) {
        continue;
      }
      mark(a.x, a.y, dx, dy, farSign, SIDE_FAR, curveRadius);
    }
  };
  for (const PathSegment& segment : segmented.segments) {
    if (!segment.straight) {
      blockRun(segment.cells);
    }
  }
  const std::vector<PathPoint> head(
      singleCrossing_->begin(),
      singleCrossing_->begin() +
          static_cast<std::ptrdiff_t>(
              std::min<std::size_t>(10, singleCrossing_->size())));
  blockRun(head);
  crossingSideActive_ = !crossingSideCells_.empty();
  return crossingSideActive_;
}

void DubinsRouter::endSingleCrossingOverlay() {
  for (const uint32_t index : crossingSideCells_) {
    crossingSide_[index] = 0;
  }
  crossingSideCells_.clear();
  crossingSideActive_ = false;
}

void DubinsRouter::setCrossingExemption(const std::vector<uint32_t>& cells) {
  for (const uint32_t index : exemptCells_) {
    exempt_[index] = 0;
  }
  exemptCells_.clear();
  if (cells.empty()) {
    return;
  }
  if (exempt_.size() != this->cells()) {
    exempt_.assign(this->cells(), 0);
  }
  for (const uint32_t index : cells) {
    if (index < exempt_.size() && exempt_[index] == 0U) {
      exempt_[index] = 1;
      exemptCells_.push_back(index);
    }
  }
}

bool DubinsRouter::canCrossOrthogonal(const uint32_t x, const uint32_t y,
                                      const Heading heading) const {
  if (x >= width_ || y >= height_) {
    return false;
  }
  const std::size_t index = (static_cast<std::size_t>(y) * width_) + x;
  if (!exemptCells_.empty() && exempt_[index] != 0U) {
    return true;
  }
  if (crossingSideActive_) {
    const uint8_t side = crossingSide_[index];
    if (side != 0U) {
      if ((side & SIDE_STRAIGHT) == 0U) {
        return false;
      }
      if ((side & 7U) != (heading & 7U)) {
        return false;
      }
    }
  }
  return constraints_.allowed(x, y, heading);
}

bool DubinsRouter::crossingAllowedOrthogonal(const uint32_t x, const uint32_t y,
                                             const Heading heading) const {
  return canCrossOrthogonal(x, y, heading);
}

uint8_t DubinsRouter::constraintMaskAt(const uint32_t x,
                                       const uint32_t y) const {
  return constraints_.maskAt(x, y);
}

// --- Search tables
// -------------------------------------------------------------

void DubinsRouter::buildTables() {
  if (tablesValid_) {
    return;
  }
  const uint32_t bend = params_.bendPenalty;
  const auto w = static_cast<int32_t>(width_);

  struct TempNode {
    int8_t dx = 0;
    int8_t dy = 0;
    uint16_t primitives = 0;
    uint8_t completes = NO_PRIMITIVE;
    std::vector<uint32_t> children;
  };

  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    auto& tprims = triePrimitives_[heading];
    auto& tflat = trie_[heading];
    tprims.clear();
    tflat.clear();
    std::vector<TempNode> nodes;
    std::vector<uint32_t> roots;
    int32_t left = 0;
    int32_t right = 0;
    int32_t up = 0;
    int32_t down = 0;

    const auto prims = primitives_->of(static_cast<Heading>(heading));
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
      // The move cost as the prototype tabulated it: rounded to float first.
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
        cellsOfMove.push_back({0, 0});
      }
      for (const CellOffset& c : p.swept) {
        cellsOfMove.push_back(c);
      }
      bool appended = false;
      if (cellsOfMove.back().dx != p.dx || cellsOfMove.back().dy != p.dy) {
        cellsOfMove.push_back({p.dx, p.dy});
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

    marginLeft_[heading] = static_cast<uint32_t>(left);
    marginRight_[heading] = static_cast<uint32_t>(right);
    marginUp_[heading] = static_cast<uint32_t>(up);
    marginDown_[heading] = static_cast<uint32_t>(down);

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
  tablesValid_ = true;
}

// --- Distance field
// --------------------------------------------------------------

void DubinsRouter::buildDistanceField(const uint32_t tx, const uint32_t ty) {
  fieldValid_ = false;
  if (corridor_ == nullptr || tx >= width_ || ty >= height_) {
    return;
  }
  buildDistanceField(std::vector<uint32_t>{(ty << 16U) | tx});
}

void DubinsRouter::buildDistanceField(const std::vector<uint32_t>& seeds) {
  fieldValid_ = false;
  if (corridor_ == nullptr || seeds.empty()) {
    return;
  }
  const std::size_t n = cells();
  if (field_.size() != n) {
    field_.assign(n, 0);
    fieldStamp_ = 0;
  }
  // Stamp 0 means never written; the stamps run 1 to 1023, then the field
  // is cleared once.
  if (++fieldStamp_ >= (1U << (32U - FIELD_DISTANCE_BITS))) {
    std::fill(field_.begin(), field_.end(), 0);
    fieldStamp_ = 1;
  }
  const uint32_t stampHigh = fieldStamp_ << FIELD_DISTANCE_BITS;
  for (auto& bucket : fieldBuckets_) {
    bucket.clear();
  }
  const grid::BitGrid& corridor = *corridor_;
  uint32_t* field = field_.data();

  std::size_t pending = 0;
  for (const uint32_t seed : seeds) {
    const std::size_t index =
        (static_cast<std::size_t>(seed >> 16U) * width_) + (seed & 0xFFFFU);
    if (field[index] == stampHigh) {
      continue;
    }
    field[index] = stampHigh;
    fieldBuckets_[0].push_back(seed);
    ++pending;
  }
  uint32_t d = 0;
  while (pending != 0) {
    auto& bucket = fieldBuckets_[d % FIELD_BUCKETS];
    while (!bucket.empty()) {
      const uint32_t packed = bucket.back();
      bucket.pop_back();
      --pending;
      const uint32_t x = packed & 0xFFFFU;
      const uint32_t y = packed >> 16U;
      const std::size_t index = (static_cast<std::size_t>(y) * width_) + x;
      if ((field[index] & FIELD_DISTANCE_MASK) != d) {
        continue;
      }
      const uint32_t x0 = (x == 0) ? 0 : x - 1;
      const uint32_t x1 = (x + 1 >= width_) ? width_ - 1 : x + 1;
      const uint32_t y0 = (y == 0) ? 0 : y - 1;
      const uint32_t y1 = (y + 1 >= height_) ? height_ - 1 : y + 1;
      for (uint32_t ny = y0; ny <= y1; ++ny) {
        const std::size_t row = static_cast<std::size_t>(ny) * width_;
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
          uint32_t nd = d + weight;
          if (nd > FIELD_DISTANCE_MASK) {
            nd = FIELD_DISTANCE_MASK;
          }
          const uint32_t nv = field[nIndex];
          if ((nv >> FIELD_DISTANCE_BITS) == fieldStamp_ &&
              (nv & FIELD_DISTANCE_MASK) <= nd) {
            continue;
          }
          field[nIndex] = stampHigh | nd;
          fieldBuckets_[nd % FIELD_BUCKETS].push_back((ny << 16U) | nx);
          ++pending;
        }
      }
    }
    ++d;
  }
  fieldValid_ = true;
}

// --- States and paths
// ----------------------------------------------------------

PathPoint DubinsRouter::unpackState(const uint32_t index) const {
  const auto heading = static_cast<Heading>(index & 7U);
  const uint32_t cell = index >> 3U;
  return {.x = cell % width_,
          .y = cell / width_,
          .heading = heading,
          .primitive = 0};
}

uint32_t DubinsRouter::parentState(const uint32_t index) const {
  const SearchNode& node = scratch_.at(index);
  const auto parentHeading = static_cast<Heading>(node.parentHeading & 7U);
  const Primitive* primitive = primitives_->find(parentHeading, node.primitive);
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
    const SearchNode& node = scratch_.at(current);
    const uint32_t parentIndex = parentState(current);
    const uint16_t edge = node.primitive;
    PathPoint parent = unpackState(parentIndex);
    if (const Primitive* primitive = primitives_->find(parent.heading, edge)) {
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
      std::reverse(segment.begin(), segment.end());
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
  std::reverse(path.begin(), path.end());
  // The goal state itself belongs to the target stub.
  if (!path.empty()) {
    path.pop_back();
  }
  return path;
}

PathPoint DubinsRouter::sanitize(const PathPoint point,
                                 const bool isTarget) const {
  const int64_t length =
      isTarget ? static_cast<int64_t>(params_.endStraightLength)
               : -static_cast<int64_t>(params_.startStraightLength);
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
  const uint16_t straight = primitives_->straight(point.heading);
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

Path DubinsRouter::assemble(Path searched, const PathPoint& source,
                            const PathPoint& target) const {
  if (searched.empty()) {
    return searched;
  }
  const Path head = straightStub(source, false, params_.startStraightLength);
  Path tail = straightStub(target, true, params_.endStraightLength);
  std::reverse(tail.begin(), tail.end());
  Path path;
  path.reserve(head.size() + searched.size() + tail.size());
  path.insert(path.end(), head.begin(), head.end());
  // The head ends on the cell the search started from, so that cell would
  // otherwise appear twice and every consumer would have to step over a
  // move of no length.
  auto first = searched.begin();
  if (!path.empty() && first != searched.end() &&
      first->samePlace(path.back())) {
    ++first;
  }
  path.insert(path.end(), first, searched.end());
  path.insert(path.end(), tail.begin(), tail.end());
  return path;
}

template <typename Search> Path DubinsRouter::guarded(Search&& search) {
  Path path = search();
  if (path.empty()) {
    return path;
  }
  if (!pathSelfIntersects(path, width_, height_, loopScratch_)) {
    return path;
  }
  // The same empty path a search that finds nothing returns: every caller
  // handles it, and a silent short is worse than a visible failure.
  ++loopGuardRejections_;
  return {};
}

// --- The free search
// -------------------------------------------------------------

Path DubinsRouter::route(const RoutingObjective& objective,
                         const bool usePenalty, const bool onlyStraight) {
  if (corridor_ == nullptr) {
    throw std::logic_error("route needs an attached corridor");
  }
  if (usePenalty && wire_ == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  lastCost_ = 0;
  expansions_ = 0;
  RoutingObjective moved;
  moved.source = sanitize(objective.source, false);
  moved.target = sanitize(objective.target, true);
  if (!inGrid(moved.source) || !inGrid(moved.target)) {
    return {};
  }
  scratch_.beginSearch();
  return guarded([&] {
    if (packedValid_) {
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
  open_.clear();
  const uint8_t* blockMap = PACKED ? packed_.data() : nullptr;
  const uint8_t* penaltyMap = PACKED ? packed_.data() : static_.data();
  constexpr uint8_t blockMask = PACKED ? uint8_t{0x80} : uint8_t{0xFF};
  constexpr uint8_t penaltyMask = PACKED ? uint8_t{0x7F} : uint8_t{0xFF};
  const uint16_t* wireMap = wire_ != nullptr ? wire_->data() : nullptr;

  for (uint32_t a = 0; a < NUM_HEADINGS; ++a) {
    bendLb_[a] = bendLowerBound_
                     ? headingDistance(static_cast<Heading>(a),
                                       objective.target.heading) *
                           params_.bendPenalty
                     : 0U;
  }
  turnGoals_.clear();
  if (turnLowerBound_) {
    turnGoals_.push_back(objective.target);
  }

  const bool useField = heuristic_ == Heuristic::DistanceField;
  if (useField) {
    buildDistanceField(objective.target.x, objective.target.y);
  } else {
    fieldValid_ = false;
  }
  const bool fieldReady = useField && fieldValid_;

  const uint32_t startIndex = stateIndex(objective.source.x, objective.source.y,
                                         objective.source.heading);
  scratch_.setStart(startIndex);
  open_.push({.x = static_cast<uint16_t>(objective.source.x),
              .y = static_cast<uint16_t>(objective.source.y),
              .heading = objective.source.heading,
              .primitive = 0,
              .f = 0,
              .g = 0},
             0);

  cutOff_ = false;
  while (!open_.empty()) {
    const QueueEntry current = open_.pop();
    if (const QueueEntry* next = open_.peek()) {
      const uint32_t nextIndex = stateIndex(next->x, next->y, next->heading);
      if (nextIndex < scratch_.size()) {
        prefetchForWrite(scratch_.data() + nextIndex);
      }
    }
    const uint32_t currentIndex =
        stateIndex(current.x, current.y, current.heading);
    SearchNode& node = scratch_.at(currentIndex);
    if (node.iteration == scratch_.iteration() && node.closed != 0U) {
      continue;
    }
    if (current.g > node.g) {
      continue;
    }
    node.closed = 1;
    ++expansions_;

    if (current.x == objective.target.x && current.y == objective.target.y &&
        current.heading == objective.target.heading) {
      lastCost_ = current.g;
      return assemble(reconstruct(currentIndex, startIndex), source, target);
    }
    expandFree<PACKED, USE_PENALTY>(current, objective, onlyStraight, blockMap,
                                    penaltyMap, blockMask, penaltyMask, wireMap,
                                    fieldReady);
  }
  return {};
}

template <bool PACKED, bool USE_PENALTY, bool PARK>
void DubinsRouter::expandFree(const QueueEntry& current,
                              const RoutingObjective& objective,
                              const bool onlyStraight, const uint8_t* blockMap,
                              const uint8_t* penaltyMap,
                              const uint8_t blockMask,
                              const uint8_t penaltyMask,
                              const uint16_t* wireMap, const bool useField) {
  const uint32_t heading = current.heading & 7U;
  // Whether a move that exits on `exit` keeps the way within `maxTurns_`;
  // a move that would not is dropped and said so. See `setMaxTurns`.
  const auto turnsAfter = [&](const uint8_t exit) {
    return static_cast<uint32_t>(current.turns) +
           headingDistance(static_cast<Heading>(heading),
                           static_cast<Heading>(exit & 7U));
  };
  const auto withinTurns = [&](const uint8_t exit) {
    if (maxTurns_ < 0 || turnsAfter(exit) <= static_cast<uint32_t>(maxTurns_)) {
      return true;
    }
    cutOff_ = true;
    return false;
  };
  const auto& tprims = triePrimitives_[heading];
  const auto& tflat = trie_[heading];
  const uint32_t cx = current.x;
  const uint32_t cy = current.y;
  const std::size_t currentLinear =
      (static_cast<std::size_t>(cy) * width_) + cx;
  const uint32_t* field = field_.data();
  const grid::BitGrid* corridor = corridor_;

  const auto blocked = [&](const std::size_t index) {
    if constexpr (PACKED) {
      return (blockMap[index] & blockMask) != 0U;
    } else {
      return corridor->test(index);
    }
  };

  const bool fastBounds =
      (cx >= marginLeft_[heading]) && (cx + marginRight_[heading] < width_) &&
      (cy >= marginUp_[heading]) && (cy + marginDown_[heading] < height_);

  // The candidates: every primitive whose end is in the grid, allowed, not
  // blocked, not dominated and, with a field, able to reach the target.
  uint16_t alive = 0;
  // Under PARK, the candidates the field proves unable to reach the target,
  // kept aside for a later target instead of dropped.
  uint16_t parked = 0;
  std::array<uint32_t, MAX_PRIMITIVES_PER_HEADING> endField{};
  std::array<uint32_t, MAX_PRIMITIVES_PER_HEADING> endIndex{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endX{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endY{};
  for (std::size_t i = 0; i < tprims.size(); ++i) {
    const TriePrimitive& p = tprims[i];
    const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + p.endDx);
    const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + p.endDy);
    if (nx >= width_ || ny >= height_) {
      continue;
    }
    if (onlyStraight && (p.exit & 1U) != 0U) {
      continue;
    }
    const std::size_t linear = (static_cast<std::size_t>(ny) * width_) + nx;
    if (blocked(linear)) {
      continue;
    }
    const uint32_t index = stateIndex(nx, ny, static_cast<Heading>(p.exit));
    const SearchNode& n = scratch_.at(index);
    if (n.iteration == scratch_.iteration()) {
      if (n.closed != 0U || current.g + p.costBend >= n.g) {
        continue;
      }
    }
    uint32_t hRaw = 0;
    if (useField) {
      const uint32_t hv = field[linear];
      if ((hv >> FIELD_DISTANCE_BITS) != fieldStamp_) {
        // Provably unable to reach the target.
        if constexpr (!PARK) {
          continue;
        }
        parked |= static_cast<uint16_t>(1U << i);
      } else {
        hRaw = hv & FIELD_DISTANCE_MASK;
      }
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

  uint32_t penaltyCurrent = 0;
  if constexpr (USE_PENALTY) {
    penaltyCurrent = penaltyMap[currentLinear] & penaltyMask;
  }
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
      if (nx >= width_ || ny >= height_) {
        i += tn.skip;
        continue;
      }
      cell = (static_cast<std::size_t>(ny) * width_) + nx;
    }
    if (blocked(cell)) {
      i += tn.skip;
      continue;
    }
    if constexpr (USE_PENALTY) {
      penaltyAcc[tn.depth + 1U] =
          penaltyAcc[tn.depth] + (penaltyMap[cell] & penaltyMask);
    }
    if (tn.completes != NO_PRIMITIVE && (alive & (1U << tn.completes)) != 0 &&
        withinTurns(tprims[tn.completes].exit)) {
      const TriePrimitive& p = tprims[tn.completes];
      const uint32_t index = endIndex[tn.completes];
      uint32_t penaltyTerm = 0;
      if constexpr (USE_PENALTY) {
        uint32_t sum = penaltyAcc[tn.depth + 1U];
        if (p.sweepsOrigin != 0U) {
          sum += penaltyCurrent;
        }
        const std::size_t linear =
            (static_cast<std::size_t>(endY[tn.completes]) * width_) +
            endX[tn.completes];
        if (p.endExtra != 0U) {
          sum += penaltyMap[linear] & penaltyMask;
        }
        penaltyTerm =
            PENALTY_SCALE * (sum + (static_cast<uint32_t>(wireMap[linear]) *
                                    (1U + p.sweptCount)));
      }
      const uint32_t tentative = current.g + p.costBend + penaltyTerm;
      SearchNode& n = scratch_.at(index);
      const bool seen = n.iteration == scratch_.iteration();
      // The turns the reached state still owes at least, when the search
      // bounds them (`setTurnLowerBound`). A move whose way cannot end
      // within the turn bound is dropped as one that exceeds it.
      const bool improves = !(seen && (n.closed != 0U || tentative >= n.g));
      const uint32_t owed =
          (improves && !turnGoals_.empty())
              ? turnBoundOf(endX[tn.completes], endY[tn.completes],
                            static_cast<Heading>(p.exit & 7U))
              : 0U;
      const bool overBound =
          maxTurns_ >= 0 &&
          turnsAfter(p.exit) + owed > static_cast<uint32_t>(maxTurns_);
      if (improves && overBound) {
        cutOff_ = true;
      }
      if (improves && !overBound) {
        if (!seen) {
          n.iteration = scratch_.iteration();
          n.closed = 0;
        }
        n.g = tentative;
        n.parentHeading = heading & 7U;
        n.primitive = p.id & 0x3FFU;
        uint32_t h = 0;
        if (useField) {
          h = endField[tn.completes];
        } else if (multiGoals_.empty()) {
          h = octile(endX[tn.completes], endY[tn.completes], objective.target.x,
                     objective.target.y);
        } else {
          h = UINT32_MAX;
          for (const PathPoint& goal : multiGoals_) {
            h = std::min(h, octile(endX[tn.completes], endY[tn.completes],
                                   goal.x, goal.y));
          }
        }
        h += std::max(bendLb_[p.exit & 7U], owed * params_.bendPenalty);
        const uint32_t f = tentative + h;
        const QueueEntry entry{.x = endX[tn.completes],
                               .y = endY[tn.completes],
                               .heading = static_cast<uint8_t>(p.exit),
                               .turns = static_cast<uint8_t>(std::min<uint32_t>(
                                   255U, turnsAfter(p.exit))),
                               .primitive = p.id,
                               .f = f,
                               .g = tentative};
        if (PARK && ((parked >> tn.completes) & 1U) != 0U) {
          parked_.push_back(entry);
          parked_.back().f = PARKED;
        } else {
          open_.push(entry, f);
        }
      }
    }
    ++i;
  }
}

// --- The multi-target search
// ---------------------------------------------------

std::vector<DubinsRouter::MultiRoute>
DubinsRouter::routeMulti(const PathPoint& source,
                         const std::vector<MultiTarget>& targets,
                         const bool usePenalty) {
  if (corridor_ == nullptr) {
    throw std::logic_error("routeMulti needs an attached corridor");
  }
  if (usePenalty && wire_ == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  std::vector<MultiRoute> out(targets.size());
  expansions_ = 0;
  const PathPoint start = sanitize(source, false);
  std::vector<PathPoint> goals;
  goals.reserve(targets.size());
  for (const MultiTarget& one : targets) {
    goals.push_back(sanitize(one.target, true));
  }
  if (!inGrid(start)) {
    return out;
  }
  scratch_.beginSearch();
  if (packedValid_) {
    if (usePenalty) {
      searchMulti<true, true>(source, start, targets, goals, out);
    } else {
      searchMulti<true, false>(source, start, targets, goals, out);
    }
  } else if (usePenalty) {
    searchMulti<false, true>(source, start, targets, goals, out);
  } else {
    searchMulti<false, false>(source, start, targets, goals, out);
  }
  return out;
}

template <bool PACKED, bool USE_PENALTY>
void DubinsRouter::searchMulti(const PathPoint& source, const PathPoint& start,
                               const std::vector<MultiTarget>& targets,
                               const std::vector<PathPoint>& goals,
                               std::vector<MultiRoute>& out) {
  buildTables();
  open_.clear();
  cutOff_ = false;
  const uint8_t* blockMap = PACKED ? packed_.data() : nullptr;
  const uint8_t* penaltyMap = PACKED ? packed_.data() : static_.data();
  constexpr uint8_t blockMask = PACKED ? uint8_t{0x80} : uint8_t{0xFF};
  constexpr uint8_t penaltyMask = PACKED ? uint8_t{0x7F} : uint8_t{0xFF};
  const uint16_t* wireMap = wire_ != nullptr ? wire_->data() : nullptr;

  // The targets still to settle. A goal off the grid has no way, as in
  // route().
  std::vector<std::size_t> pending;
  std::vector<uint32_t> seeds;
  for (std::size_t k = 0; k < goals.size(); ++k) {
    if (inGrid(goals[k])) {
      pending.push_back(k);
      seeds.push_back((goals[k].y << 16U) | goals[k].x);
    }
  }
  if (pending.empty()) {
    return;
  }

  // The heuristic: the least of its terms over every target (see
  // `routeMulti`).
  for (uint32_t a = 0; a < NUM_HEADINGS; ++a) {
    uint32_t least = UINT32_MAX;
    for (const std::size_t k : pending) {
      least = std::min(
          least, headingDistance(static_cast<Heading>(a), goals[k].heading));
    }
    bendLb_[a] = bendLowerBound_ ? least * params_.bendPenalty : 0U;
  }
  turnGoals_.clear();
  if (turnLowerBound_) {
    for (const std::size_t k : pending) {
      turnGoals_.push_back(goals[k]);
    }
  }
  const bool useField = heuristic_ == Heuristic::DistanceField;
  if (useField) {
    buildDistanceField(seeds);
  } else {
    fieldValid_ = false;
  }
  const bool fieldReady = useField && fieldValid_;
  if (!fieldReady) {
    for (const std::size_t k : pending) {
      multiGoals_.push_back(goals[k]);
    }
  }
  // `expandFree` reads the objective's target for the octile heuristic
  // alone, and `multiGoals_` stands in for it here.
  const RoutingObjective objective{.source = start,
                                   .target = goals[pending[0]]};
  const bool onlyStraight = false;

  const uint32_t startIndex = stateIndex(start.x, start.y, start.heading);
  scratch_.setStart(startIndex);
  open_.push({.x = static_cast<uint16_t>(start.x),
              .y = static_cast<uint16_t>(start.y),
              .heading = start.heading,
              .primitive = 0,
              .f = 0,
              .g = 0},
             0);

  while (!open_.empty() && !pending.empty()) {
    const QueueEntry current = open_.pop();
    const uint32_t currentIndex =
        stateIndex(current.x, current.y, current.heading);
    SearchNode& node = scratch_.at(currentIndex);
    if (node.iteration == scratch_.iteration() && node.closed != 0U) {
      continue;
    }
    if (current.g > node.g) {
      continue;
    }
    node.closed = 1;
    ++expansions_;

    for (auto it = pending.begin(); it != pending.end();) {
      const PathPoint& goal = goals[*it];
      if (current.x != goal.x || current.y != goal.y ||
          current.heading != goal.heading) {
        ++it;
        continue;
      }
      out[*it] = settle(currentIndex, startIndex, current.turns, source,
                        targets[*it].target, goal, targets[*it].own);
      it = pending.erase(it);
    }
    if (pending.empty()) {
      break;
    }
    expandFree<PACKED, USE_PENALTY>(current, objective, onlyStraight, blockMap,
                                    penaltyMap, blockMask, penaltyMask, wireMap,
                                    fieldReady);
  }
  for (const std::size_t k : pending) {
    out[k].outcome =
        cutOff_ ? MultiRoute::Outcome::CutOff : MultiRoute::Outcome::NoWay;
  }
  multiGoals_.clear();
}

DubinsRouter::MultiRoute
DubinsRouter::settle(const uint32_t goalIndex, const uint32_t startIndex,
                     const uint32_t turns, const PathPoint& source,
                     const PathPoint& target, const PathPoint& goal,
                     const grid::BitGrid* own) {
  MultiRoute route;
  const Path searched = reconstruct(goalIndex, startIndex);
  route.turns = turns;
  route.cost = scratch_.at(goalIndex).g;
  // Whether every cell the search tested for the way is open in `own`: the
  // cells of `searched` after its first, which is the start state the search
  // never tests, and the goal cell, which `reconstruct` leaves to the stub.
  const auto openIn = [&] {
    for (std::size_t i = 1; i < searched.size(); ++i) {
      if (own->test((static_cast<std::size_t>(searched[i].y) * width_) +
                    searched[i].x)) {
        return false;
      }
    }
    return !own->test((static_cast<std::size_t>(goal.y) * width_) + goal.x);
  };
  // A goal on the start state leaves nothing searched, which route()
  // returns as no way.
  if (searched.empty()) {
    route.outcome = MultiRoute::Outcome::NoWay;
  } else if (own != nullptr && !openIn()) {
    route.outcome = MultiRoute::Outcome::LeftOwnCorridor;
  } else {
    route.outcome = MultiRoute::Outcome::Found;
  }
  route.way = assemble(searched, source, target);
  // The loop guard of route(), without its count: a caller that wants the
  // way searches route() again, which counts it.
  if (route.outcome == MultiRoute::Outcome::Found &&
      pathSelfIntersects(route.way, width_, height_, loopScratch_)) {
    route.outcome = MultiRoute::Outcome::Looped;
  }
  return route;
}

// --- The resumable search
// -------------------------------------------------------

DubinsRouter::MultiRoute
DubinsRouter::routeResumable(PausedSearch& search, const PathPoint& source,
                             const PathPoint& target, const grid::BitGrid* own,
                             const bool usePenalty,
                             const std::vector<PathPoint>& family) {
  if (corridor_ == nullptr) {
    throw std::logic_error("routeResumable needs an attached corridor");
  }
  if (usePenalty && wire_ == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  expansions_ = 0;
  resumeSplit_ = {};
  const PathPoint goal = sanitize(target, true);
  if (!search.started_) {
    search.source_ = source;
    search.family_.clear();
    for (const PathPoint& one : family) {
      if (const PathPoint moved = sanitize(one, true); inGrid(moved)) {
        search.family_.push_back(moved);
      }
    }
    if (search.family_.empty()) {
      search.family_.push_back(goal);
    }
  }
  if (!inGrid(sanitize(search.source_, false)) || !inGrid(goal)) {
    return {};
  }
  if (packedValid_) {
    return usePenalty ? searchResumable<true, true>(search, target, goal, own)
                      : searchResumable<true, false>(search, target, goal, own);
  }
  return usePenalty ? searchResumable<false, true>(search, target, goal, own)
                    : searchResumable<false, false>(search, target, goal, own);
}

template <bool PACKED, bool USE_PENALTY>
DubinsRouter::MultiRoute
DubinsRouter::searchResumable(PausedSearch& search, const PathPoint& target,
                              const PathPoint& goal, const grid::BitGrid* own) {
  buildTables();
  open_.clear();
  parked_.clear();
  const uint8_t* blockMap = PACKED ? packed_.data() : nullptr;
  const uint8_t* penaltyMap = PACKED ? packed_.data() : static_.data();
  constexpr uint8_t blockMask = PACKED ? uint8_t{0x80} : uint8_t{0xFF};
  constexpr uint8_t penaltyMask = PACKED ? uint8_t{0x7F} : uint8_t{0xFF};
  const uint16_t* wireMap = wire_ != nullptr ? wire_->data() : nullptr;

  // The heuristic: of this call's target alone, or the least over the
  // family.
  const bool least = resumeOrder_ == ResumeOrder::Minimum;
  const std::vector<PathPoint> aims =
      least ? search.family_ : std::vector<PathPoint>{goal};
  for (uint32_t a = 0; a < NUM_HEADINGS; ++a) {
    uint32_t turns = UINT32_MAX;
    for (const PathPoint& aim : aims) {
      turns = std::min(turns,
                       headingDistance(static_cast<Heading>(a), aim.heading));
    }
    bendLb_[a] = bendLowerBound_ ? turns * params_.bendPenalty : 0U;
  }
  // A resumed search serves targets it does not know yet, so it bounds
  // nothing by the turns to one goal (`setTurnLowerBound`).
  turnGoals_.clear();
  const bool useField = heuristic_ == Heuristic::DistanceField;
  if (useField) {
    std::vector<uint32_t> seeds;
    for (const PathPoint& aim : aims) {
      seeds.push_back((aim.y << 16U) | aim.x);
    }
    buildDistanceField(seeds);
  } else {
    fieldValid_ = false;
  }
  const bool fieldReady = useField && fieldValid_;
  if (least && !fieldReady) {
    multiGoals_ = aims;
  }
  // The key of an entry for this call, or PARKED where the field proves it
  // unable to reach the target.
  const auto keyOf = [&](const QueueEntry& entry) {
    uint32_t h = 0;
    if (fieldReady) {
      const uint32_t hv =
          field_[(static_cast<std::size_t>(entry.y) * width_) + entry.x];
      if ((hv >> FIELD_DISTANCE_BITS) != fieldStamp_) {
        return PARKED;
      }
      h = hv & FIELD_DISTANCE_MASK;
    } else if (least) {
      h = UINT32_MAX;
      for (const PathPoint& aim : aims) {
        h = std::min(h, octile(entry.x, entry.y, aim.x, aim.y));
      }
    } else {
      h = octile(entry.x, entry.y, goal.x, goal.y);
    }
    return entry.g + h + bendLb_[entry.heading & 7U];
  };
  // Under Offset, what the keys of the last call are lowered by, if the
  // field reaches the last goal cell.
  bool offset = resumeOrder_ == ResumeOrder::Offset && search.started_;
  uint32_t delta = 0;
  if (offset) {
    const PathPoint& last = search.lastGoal_;
    uint32_t apart = 0;
    if (fieldReady) {
      const uint32_t hv =
          field_[(static_cast<std::size_t>(last.y) * width_) + last.x];
      offset = (hv >> FIELD_DISTANCE_BITS) == fieldStamp_;
      apart = hv & FIELD_DISTANCE_MASK;
    } else {
      apart = octile(last.x, last.y, goal.x, goal.y);
    }
    delta =
        apart + (bendLowerBound_ ? headingDistance(last.heading, goal.heading) *
                                       params_.bendPenalty
                                 : 0U);
  }
  const PathPoint start = sanitize(search.source_, false);
  const RoutingObjective objective{.source = start, .target = goal};
  const uint32_t goalIndex = stateIndex(goal.x, goal.y, goal.heading);
  const auto secondsSince =
      [](const std::chrono::steady_clock::time_point from) {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             from)
            .count();
      };

  // The eighth-turns of the way to a closed state, along its parents.
  const auto turnsTo = [&](uint32_t index) {
    uint32_t turns = 0;
    while (index != search.startIndex_) {
      const SearchNode& node = scratch_.at(index);
      turns += headingDistance(static_cast<Heading>(node.parentHeading & 7U),
                               static_cast<Heading>(index & 7U));
      index = parentState(index);
    }
    return turns;
  };

  scratch_.beginSearch();
  const uint16_t now = scratch_.iteration();
  if (!search.started_) {
    search.started_ = true;
    search.startIndex_ = stateIndex(start.x, start.y, start.heading);
    scratch_.setStart(search.startIndex_);
    cutOff_ = false;
    open_.push({.x = static_cast<uint16_t>(start.x),
                .y = static_cast<uint16_t>(start.y),
                .heading = start.heading,
                .primitive = 0,
                .f = 0,
                .g = 0},
               0);
  } else {
    cutOff_ = search.cutOff_;
    auto lap = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < search.indices_.size(); ++i) {
      SearchNode node = search.nodes_[i];
      node.iteration = now;
      scratch_.at(search.indices_[i]) = node;
    }
    resumeSplit_.restore = secondsSince(lap);
    const SearchNode& reached = scratch_.at(goalIndex);
    if (reached.iteration == now && reached.closed != 0U) {
      // An earlier call closed this target's goal state on its way: the
      // answer is there, and the paused search stays as it was.
      multiGoals_.clear();
      return settle(goalIndex, search.startIndex_, turnsTo(goalIndex),
                    search.source_, target, goal, own);
    }
    // Every entry left, back on the open list: a parked one keyed for this
    // call or parked again, an open one at its key under Minimum, lowered
    // by `delta` under Offset, and keyed for this call under Target.
    lap = std::chrono::steady_clock::now();
    for (QueueEntry entry : search.pending_) {
      if (entry.f == PARKED || (!least && !offset)) {
        entry.f = keyOf(entry);
      } else if (offset) {
        entry.f = entry.f > entry.g + delta ? entry.f - delta : entry.g;
      }
      if (entry.f == PARKED) {
        parked_.push_back(entry);
      } else {
        open_.push(entry, entry.f);
      }
    }
    resumeSplit_.rekey = secondsSince(lap);
  }

  MultiRoute out;
  bool found = false;
  std::vector<uint32_t> closed;
  while (!open_.empty()) {
    const QueueEntry current = open_.pop();
    const uint32_t currentIndex =
        stateIndex(current.x, current.y, current.heading);
    SearchNode& node = scratch_.at(currentIndex);
    if (node.iteration == now && node.closed != 0U) {
      continue;
    }
    if (current.g > node.g) {
      continue;
    }
    if (offset) {
      // A key lowered by `delta` is set right before the state is taken.
      QueueEntry exact = current;
      exact.f = keyOf(current);
      if (exact.f == PARKED) {
        parked_.push_back(exact);
        continue;
      }
      if (exact.f > current.f) {
        open_.push(exact, exact.f);
        ++resumeSplit_.reinserts;
        continue;
      }
    }
    node.closed = 1;
    ++expansions_;
    closed.push_back(currentIndex);
    if (currentIndex == goalIndex) {
      out = settle(goalIndex, search.startIndex_, turnsTo(goalIndex),
                   search.source_, target, goal, own);
      found = true;
    }
    // The goal state too, so that a later target loses no way through it.
    expandFree<PACKED, USE_PENALTY, true>(current, objective, false, blockMap,
                                          penaltyMap, blockMask, penaltyMask,
                                          wireMap, fieldReady);
    if (found) {
      break;
    }
  }
  if (!found) {
    out.outcome =
        cutOff_ ? MultiRoute::Outcome::CutOff : MultiRoute::Outcome::NoWay;
  }

  // Pause: the entries still to expand, the best of each state, and every
  // state reached.
  const auto paused = std::chrono::steady_clock::now();
  search.cutOff_ = cutOff_;
  search.lastGoal_ = goal;
  multiGoals_.clear();
  std::vector<QueueEntry> pending = std::move(parked_);
  parked_.clear();
  while (!open_.empty()) {
    pending.push_back(open_.pop());
  }
  std::vector<QueueEntry> kept;
  kept.reserve(pending.size());
  for (const QueueEntry& entry : pending) {
    const SearchNode& state =
        scratch_.at(stateIndex(entry.x, entry.y, entry.heading));
    if (state.closed == 0U && entry.g == state.g) {
      kept.push_back(entry);
    }
  }
  const auto indexOf = [&](const QueueEntry& entry) {
    return stateIndex(entry.x, entry.y, entry.heading);
  };
  std::ranges::sort(kept, {}, indexOf);
  const auto [first, last] = std::ranges::unique(kept, {}, indexOf);
  kept.erase(first, last);
  std::vector<uint32_t> indices = std::move(search.indices_);
  indices.insert(indices.end(), closed.begin(), closed.end());
  for (const QueueEntry& entry : kept) {
    indices.push_back(indexOf(entry));
  }
  indices.push_back(search.startIndex_);
  std::ranges::sort(indices);
  indices.erase(std::ranges::unique(indices).begin(), indices.end());
  search.nodes_.resize(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    search.nodes_[i] = scratch_.at(indices[i]);
  }
  search.indices_ = std::move(indices);
  search.pending_ = std::move(kept);
  resumeSplit_.pause = secondsSince(paused);
  return out;
}

// --- The orthogonal search
// ------------------------------------------------------

Path DubinsRouter::routeOrthogonal(const RoutingObjective& objective,
                                   const bool usePenalty,
                                   const bool onlyStraight) {
  if (corridor_ == nullptr) {
    throw std::logic_error("routeOrthogonal needs an attached corridor");
  }
  if (usePenalty && wire_ == nullptr) {
    throw std::logic_error(
        "routing with penalties needs an attached wire proximity");
  }
  RoutingObjective moved;
  moved.source = sanitize(objective.source, false);
  moved.target = sanitize(objective.target, true);
  if (!inGrid(moved.source) || !inGrid(moved.target)) {
    return {};
  }
  scratch_.beginSearch();
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
  open_.clear();
  const uint32_t startIndex = stateIndex(objective.source.x, objective.source.y,
                                         objective.source.heading);
  scratch_.setStart(startIndex);
  open_.push({.x = static_cast<uint16_t>(objective.source.x),
              .y = static_cast<uint16_t>(objective.source.y),
              .heading = objective.source.heading,
              .primitive = 0,
              .f = 0,
              .g = 0},
             0);
  cutOff_ = false;
  while (!open_.empty()) {
    const QueueEntry current = open_.pop();
    const uint32_t currentIndex =
        stateIndex(current.x, current.y, current.heading);
    SearchNode& node = scratch_.at(currentIndex);
    if (node.iteration == scratch_.iteration() && node.closed != 0U) {
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
  // Whether a move that exits on `exit` keeps the way within `maxTurns_`;
  // a move that would not is dropped and said so. See `setMaxTurns`.
  const auto turnsAfter = [&](const uint8_t exit) {
    return static_cast<uint32_t>(current.turns) +
           headingDistance(static_cast<Heading>(heading),
                           static_cast<Heading>(exit & 7U));
  };
  const auto withinTurns = [&](const uint8_t exit) {
    if (maxTurns_ < 0 || turnsAfter(exit) <= static_cast<uint32_t>(maxTurns_)) {
      return true;
    }
    cutOff_ = true;
    return false;
  };
  const auto& tprims = triePrimitives_[heading];
  const auto& tflat = trie_[heading];
  const uint32_t cx = current.x;
  const uint32_t cy = current.y;
  const std::size_t currentLinear =
      (static_cast<std::size_t>(cy) * width_) + cx;
  const grid::BitGrid& corridor = *corridor_;

  const auto cellOk = [&](const uint32_t x, const uint32_t y) {
    if (corridor.testCell(x, y)) {
      return false;
    }
    return canCrossOrthogonal(x, y, static_cast<Heading>(heading));
  };
  const bool fastBounds =
      (cx >= marginLeft_[heading]) && (cx + marginRight_[heading] < width_) &&
      (cy >= marginUp_[heading]) && (cy + marginDown_[heading] < height_);

  uint16_t alive = 0;
  std::array<uint32_t, MAX_PRIMITIVES_PER_HEADING> endIndex{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endX{};
  std::array<uint16_t, MAX_PRIMITIVES_PER_HEADING> endY{};
  for (std::size_t i = 0; i < tprims.size(); ++i) {
    const TriePrimitive& p = tprims[i];
    const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + p.endDx);
    const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + p.endDy);
    if (nx >= width_ || ny >= height_) {
      continue;
    }
    if (onlyStraight && (p.exit & 1U) != 0U) {
      continue;
    }
    // The end cell is recorded with the heading the move leaves on, so under
    // `exitCheck_` it has to pass the rule on that heading too — see
    // `setOrthogonalExitCheck`.
    if (!cellOk(nx, ny) ||
        (exitCheck_ &&
         !canCrossOrthogonal(nx, ny, static_cast<Heading>(p.exit)))) {
      continue;
    }
    const uint32_t index = stateIndex(nx, ny, static_cast<Heading>(p.exit));
    const SearchNode& n = scratch_.at(index);
    if (n.iteration == scratch_.iteration()) {
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

  uint32_t penaltyCurrent = 0;
  if constexpr (USE_PENALTY) {
    penaltyCurrent = static_[currentLinear];
  }
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
    const auto nx = static_cast<uint32_t>(static_cast<int32_t>(cx) + tn.dx);
    const auto ny = static_cast<uint32_t>(static_cast<int32_t>(cy) + tn.dy);
    if (!fastBounds && (nx >= width_ || ny >= height_)) {
      i += tn.skip;
      continue;
    }
    if (!cellOk(nx, ny)) {
      i += tn.skip;
      continue;
    }
    const std::size_t cell = (static_cast<std::size_t>(ny) * width_) + nx;
    if constexpr (USE_PENALTY) {
      penaltyAcc[tn.depth + 1U] = penaltyAcc[tn.depth] + static_[cell];
    }
    if (tn.completes != NO_PRIMITIVE && (alive & (1U << tn.completes)) != 0 &&
        withinTurns(tprims[tn.completes].exit)) {
      const TriePrimitive& p = tprims[tn.completes];
      const std::size_t linear =
          (static_cast<std::size_t>(endY[tn.completes]) * width_) +
          endX[tn.completes];
      uint32_t penaltyTerm = 0;
      if constexpr (USE_PENALTY) {
        uint32_t sum = penaltyAcc[tn.depth + 1U];
        if (p.sweepsOrigin != 0U) {
          sum += penaltyCurrent;
        }
        if (p.endExtra != 0U) {
          sum += static_[linear];
        }
        penaltyTerm =
            (PENALTY_SCALE * sum) +
            (PENALTY_SCALE * static_cast<uint32_t>(wirePenalty(linear)) *
             (1U + p.sweptCount));
      }
      const uint32_t tentative = current.g + p.costBend + penaltyTerm;
      SearchNode& n = scratch_.at(endIndex[tn.completes]);
      const bool seen = n.iteration == scratch_.iteration();
      if (!(seen && (n.closed != 0U || tentative >= n.g))) {
        if (!seen) {
          n.iteration = scratch_.iteration();
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
             params_.bendPenalty;
        const uint32_t f = tentative + h;
        open_.push({.x = endX[tn.completes],
                    .y = endY[tn.completes],
                    .heading = static_cast<uint8_t>(p.exit),
                    .turns = static_cast<uint8_t>(
                        std::min<uint32_t>(255U, turnsAfter(p.exit))),
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
  return freeStrip(from, to, nullptr);
}

CellBox DubinsRouter::freeStripAlong(const PathPoint from, const PathPoint to,
                                     FreeStripMemo& memo) const {
  return freeStrip(from, to, &memo);
}

CellBox DubinsRouter::freeStrip(const PathPoint from, const PathPoint to,
                                FreeStripMemo* const memo) const {
  if (obstacles_ == nullptr) {
    return {.minX = 0, .maxX = width_ - 1, .minY = 0, .maxY = height_ - 1};
  }
  const grid::BitGrid& obstacles = *obstacles_;
  const auto w = static_cast<int64_t>(width_);
  const auto h = static_cast<int64_t>(height_);
  const auto edgeFree = [&](const int64_t x0, const int64_t y0,
                            const int64_t x1, const int64_t y1) {
    if (std::min(x0, x1) < 0 || std::max(x0, x1) >= w || std::min(y0, y1) < 0 ||
        std::max(y0, y1) >= h) {
      return false;
    }
    int64_t dx = std::abs(x1 - x0);
    int64_t dy = -std::abs(y1 - y0);
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
  if (memo != nullptr && (from.x == to.x || from.y == to.y)) {
    // On an axis-parallel segment the normal is a whole cell, so the edge at
    // an offset is the segment moved by that many cells, and it is free
    // until a cell of it is blocked or leaves the grid.
    const int64_t stepX = nx / one;
    const int64_t stepY = ny / one;
    const int64_t alongX = (to.x > from.x) - (to.x < from.x);
    const int64_t alongY = (to.y > from.y) - (to.y < from.y);
    const auto depthAt = [&](const int64_t x, const int64_t y, const int64_t sx,
                             const int64_t sy) {
      const auto side = static_cast<uint64_t>((sx + 1) * 3 + (sy + 1));
      const auto key =
          (((static_cast<uint64_t>(y) * width_) + static_cast<uint64_t>(x))
           << 4U) |
          side;
      const auto known = memo->depths.find(key);
      if (known != memo->depths.end()) {
        return static_cast<int64_t>(known->second);
      }
      int64_t depth = 0;
      for (int64_t px = x + sx, py = y + sy;
           px >= 0 && px < w && py >= 0 && py < h &&
           !obstacles.testCell(static_cast<uint32_t>(px),
                               static_cast<uint32_t>(py));
           px += sx, py += sy) {
        ++depth;
      }
      memo->depths.emplace(key, static_cast<uint32_t>(depth));
      return depth;
    };
    positive = std::numeric_limits<int64_t>::max();
    negative = std::numeric_limits<int64_t>::max();
    for (int64_t x = from.x, y = from.y;; x += alongX, y += alongY) {
      positive = std::min(positive, depthAt(x, y, stepX, stepY));
      negative = std::min(negative, depthAt(x, y, -stepX, -stepY));
      if (x == to.x && y == to.y) {
        break;
      }
    }
    growPositive = false;
    growNegative = false;
  }
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
