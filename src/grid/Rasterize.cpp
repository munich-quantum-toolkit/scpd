/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Rasterize.hpp"

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

namespace {

using flatbuffers::design::ChipT;
using flatbuffers::geometry::PolygonT;

/// lineCells() accepts two ends that lie fewer than this many cells apart
/// along each axis. Then twice the error term of the walk fits into int64_t.
constexpr uint64_t LINE_SPAN_LIMIT = uint64_t{1} << 61U;

/// The margin around the grid, in cells, within which fillPolygon() keeps the
/// ends of an edge. An end further out moves along its edge onto the margin,
/// so that its rounded cell fits into int64_t and the edge stays shorter than
/// LINE_SPAN_LIMIT.
constexpr double EDGE_MARGIN = 576460752303423488.0; // 2^59

/// The quotient and the remainder of a division of whole numbers.
struct Division {
  uint64_t quotient = 0;
  uint64_t remainder = 0;
};

/// Divides @p factor times @p steps by @p length without overflow.
/// @pre @p factor and @p steps are at most @p length, and @p length is
/// positive and below 2^62.
Division divideProduct(const uint64_t factor, const uint64_t steps,
                       const uint64_t length) {
  if (steps == 0 || factor <= std::numeric_limits<uint64_t>::max() / steps) {
    const uint64_t product = factor * steps;
    return {.quotient = product / length, .remainder = product % length};
  }
  // Long multiplication over the bits of steps. The remainder stays below
  // length, so no sum overflows.
  Division result;
  for (unsigned bit = 64; bit-- > 0;) {
    result.quotient <<= 1U;
    result.remainder <<= 1U;
    if (result.remainder >= length) {
      result.remainder -= length;
      ++result.quotient;
    }
    if (((steps >> bit) & 1U) != 0) {
      result.remainder += factor;
      if (result.remainder >= length) {
        result.remainder -= length;
        ++result.quotient;
      }
    }
  }
  return result;
}

/// The distance between two coordinates, without overflow.
uint64_t span(const int64_t from, const int64_t to) {
  return from <= to ? static_cast<uint64_t>(to) - static_cast<uint64_t>(from)
                    : static_cast<uint64_t>(from) - static_cast<uint64_t>(to);
}

/// Moves each end of the segment from @p a to @p b that lies outside a box
/// along the segment onto the box. An end inside the box keeps its value.
/// @return @c false when the segment misses the box.
bool clipToBox(Point& a, Point& b, const Point low, const Point high) {
  const auto inside = [&](const Point point) {
    return point.x() >= low.x() && point.x() <= high.x() &&
           point.y() >= low.y() && point.y() <= high.y();
  };
  if (inside(a) && inside(b)) {
    return true;
  }
  // Liang and Barsky, in half units, so that the difference of two finite
  // coordinates cannot overflow. The parameter t runs from a to b, and the
  // parameter u = 1 - t runs from b to a. Each one is measured from its own
  // end, so each stays accurate near that end.
  const double ax = a.x() / 2.0;
  const double ay = a.y() / 2.0;
  const double bx = b.x() / 2.0;
  const double by = b.y() / 2.0;
  const double dx = bx - ax;
  const double dy = by - ay;
  double t0 = 0.0;
  double t1 = 1.0;
  double u0 = 0.0;
  double u1 = 1.0;
  // Keeps the part of the segment where p * t <= q.
  const auto keep = [](const double p, const double q, double& first,
                       double& last) {
    if (p == 0.0) {
      return q >= 0.0;
    }
    const double t = q / p;
    if (p < 0.0) {
      first = std::max(first, t);
    } else {
      last = std::min(last, t);
    }
    return first <= last;
  };
  const double lowX = low.x() / 2.0;
  const double lowY = low.y() / 2.0;
  const double highX = high.x() / 2.0;
  const double highY = high.y() / 2.0;
  if (!keep(-dx, ax - lowX, t0, t1) || !keep(dx, highX - ax, t0, t1) ||
      !keep(-dy, ay - lowY, t0, t1) || !keep(dy, highY - ay, t0, t1) ||
      !keep(dx, bx - lowX, u0, u1) || !keep(-dx, highX - bx, u0, u1) ||
      !keep(dy, by - lowY, u0, u1) || !keep(-dy, highY - by, u0, u1)) {
    return false;
  }
  // The point at parameter t from a, which is u from b. It comes from the
  // nearer end, so that a coordinate far larger than the box cannot cancel.
  // When both ends lie that far off the box, the point is inexact, and the
  // clamp keeps it on the box.
  const auto pointAt = [&](const double t, const double u) {
    const bool fromA = t <= 0.5;
    const double x = fromA ? ax + (t * dx) : bx - (u * dx);
    const double y = fromA ? ay + (t * dy) : by - (u * dy);
    return Point(2.0 * std::clamp(x, lowX, highX),
                 2.0 * std::clamp(y, lowY, highY));
  };
  if (t0 > 0.0) {
    a = pointAt(t0, u1);
  }
  if (u0 > 0.0) {
    b = pointAt(t1, u0);
  }
  return true;
}

/// The share of the largest coordinate that a row interval of
/// columnsNear() adds to its reach. The rounding errors of the interval and
/// of distanceToSegment() grow with the coordinates, and this share exceeds
/// them many times over.
constexpr double ROUNDING_SHARE = 1e-12;

/// The columns of the cells in a row whose center can lie within a reach of
/// a segment.
///
/// A point within @p reach of the segment lies within @p reach, along each
/// axis, of a point of the segment. So the columns cover the part of the
/// segment within @p reach of the row along y, widened by @p reach along x.
///
/// @param grid The grid of the row.
/// @param a One end of the segment, in layout units.
/// @param b The other end of the segment, in layout units.
/// @param reach The distance to cover, in layout units.
/// @param row The row.
/// @param first The first column of the window to search.
/// @param last The last column of the window to search. It may lie left of
/// @p first, and then the window is empty.
/// @return The first and the last column, clamped onto [@p first, @p last].
/// The first column exceeds the last when no cell center of the row lies
/// within @p reach. When the ends of the segment or their difference are not
/// finite, or @p reach is not a number, the function returns every column
/// from @p first to @p last.
std::pair<int64_t, int64_t> columnsNear(const GridMetrics& grid, const Point a,
                                        const Point b, const double reach,
                                        const int64_t row, const int64_t first,
                                        const int64_t last) {
  const double dx = b.x() - a.x();
  const double dy = b.y() - a.y();
  if (first > last || !std::isfinite(dx) || !std::isfinite(dy)) {
    return {first, last};
  }
  const double y = grid.toLayout(0.0, static_cast<double>(row)).y();
  const double below = y - reach;
  const double above = y + reach;
  if (std::max(a.y(), b.y()) < below || std::min(a.y(), b.y()) > above) {
    return {first, first - 1};
  }
  // The parameters along the segment where it enters and leaves the band
  // within reach of the row. The point at a parameter comes from the nearer
  // end, so that a coordinate far larger than the grid cannot cancel.
  double enter = 0.0;
  double leave = 1.0;
  if (dy != 0.0) {
    const double atBelow = (below - a.y()) / dy;
    const double atAbove = (above - a.y()) / dy;
    enter = std::clamp(std::min(atBelow, atAbove), 0.0, 1.0);
    leave = std::clamp(std::max(atBelow, atAbove), 0.0, 1.0);
  }
  const auto xAt = [&](const double t) {
    return t <= 0.5 ? a.x() + (t * dx) : b.x() - ((1.0 - t) * dx);
  };
  const double left = std::min(xAt(enter), xAt(leave)) - reach;
  const double right = std::max(xAt(enter), xAt(leave)) + reach;
  const double from = std::ceil((left - grid.origin.x()) / grid.cellWidth);
  const double to = std::floor((right - grid.origin.x()) / grid.cellWidth);
  if (std::isnan(from) || std::isnan(to)) {
    return {first, last};
  }
  return {static_cast<int64_t>(std::clamp(from, static_cast<double>(first),
                                          static_cast<double>(last) + 1.0)),
          static_cast<int64_t>(std::clamp(to, static_cast<double>(first) - 1.0,
                                          static_cast<double>(last)))};
}

/// Blocks every free cell within the keepout of an obstacle edge, then frees
/// the cells of the keepout that a port corridor reaches. A second mask marks
/// the cells of the keepout, so that a port corridor never frees a cell that
/// the polygons block after the island rule.
void blockKeepout(const ChipT& chip, const GridMetrics& grid,
                  const RasterOptions& options, BitGrid& blocked,
                  std::size_t& keepoutCells, std::size_t& exemptedCells) {
  BitGrid keepout(grid.width, grid.height);
  const BoundingBox box = grid.box();
  const double gridSize = std::max({std::fabs(box.minX), std::fabs(box.minY),
                                    std::fabs(box.maxX), std::fabs(box.maxY)});
  const double cellStep = std::max(grid.cellWidth, grid.cellHeight);

  // Visit the cells of a window around an edge, given in layout units. The
  // window is clamped onto the grid before the cast to an integer. std::fmax
  // and std::fmin drop a NaN, so a NaN bound widens the window to the grid,
  // and the distance test decides. In each row, only the columns within
  // reach of the edge take the distance test. The reach adds a cell step and
  // a share of the coordinates to the half width, so that no rounding error
  // drops a cell that the distance test keeps.
  const auto forEachCellNear = [&](const Point a, const Point b,
                                   const double halfWidth, auto&& visit) {
    const Point ca = grid.toCell(a);
    const Point cb = grid.toCell(b);
    const double extraX = std::ceil(halfWidth / grid.cellWidth) + 1.0;
    const double extraY = std::ceil(halfWidth / grid.cellHeight) + 1.0;
    const double lowX = std::floor(std::min(ca.x(), cb.x())) - extraX;
    const double highX = std::ceil(std::max(ca.x(), cb.x())) + extraX;
    const double lowY = std::floor(std::min(ca.y(), cb.y())) - extraY;
    const double highY = std::ceil(std::max(ca.y(), cb.y())) + extraY;
    const auto width = static_cast<double>(grid.width);
    const auto height = static_cast<double>(grid.height);
    const auto x0 =
        static_cast<int64_t>(std::fmin(std::fmax(lowX, 0.0), width));
    const auto x1 =
        static_cast<int64_t>(std::fmax(std::fmin(highX, width - 1.0), -1.0));
    const auto y0 =
        static_cast<int64_t>(std::fmin(std::fmax(lowY, 0.0), height));
    const auto y1 =
        static_cast<int64_t>(std::fmax(std::fmin(highY, height - 1.0), -1.0));
    const double size =
        std::max({gridSize, std::fabs(a.x()), std::fabs(a.y()),
                  std::fabs(b.x()), std::fabs(b.y()), halfWidth});
    const double reach = halfWidth + cellStep + (ROUNDING_SHARE * size);
    for (int64_t y = y0; y <= y1; ++y) {
      const auto [first, last] = columnsNear(grid, a, b, reach, y, x0, x1);
      for (int64_t x = first; x <= last; ++x) {
        const Point center =
            grid.toLayout(static_cast<double>(x), static_cast<double>(y));
        if (distanceToSegment(center, a, b) <= halfWidth) {
          visit(grid.index(static_cast<uint32_t>(x), static_cast<uint32_t>(y)));
        }
      }
    }
  };

  for (const auto& obstacle : chip.obstacles) {
    const auto* const polygon = obstacle.get();
    if (polygon == nullptr || polygon->vertices.size() < 2) {
      continue;
    }
    const auto& vertices = polygon->vertices;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      const Point a = vertices[i];
      const Point b = vertices[(i + 1) % vertices.size()];
      forEachCellNear(a, b, options.keepout, [&](const std::size_t index) {
        if (!blocked.test(index)) {
          blocked.set(index);
          keepout.set(index);
          ++keepoutCells;
        }
      });
    }
  }
  for (const auto& portCorridor : options.keepoutExemptions) {
    forEachCellNear(portCorridor.from, portCorridor.to, portCorridor.halfWidth,
                    [&](const std::size_t index) {
                      if (keepout.test(index)) {
                        keepout.set(index, false);
                        blocked.set(index, false);
                        ++exemptedCells;
                      }
                    });
  }
}

} // namespace

namespace {

/// The length of the vector (@p dx, @p dy).
///
/// IEEE 754 rounds each multiplication, the addition and the square root
/// correctly, and the build turns floating-point contraction off. The result
/// is therefore the same on every platform. It is not always the correctly
/// rounded length, because the squares round before they are added.
/// std::hypot gives no promise of either kind: its last bit may differ
/// between math libraries, and the keepout compares the distance with its
/// limit exactly. The sum of squares is a
/// normal number for every length from about 1.5e-154 to 1.3e154, a range
/// that holds every distance on a chip. Outside that range the sum overflows
/// or loses digits, and std::hypot takes over.
double vectorLength(const double dx, const double dy) {
  const double squared = (dx * dx) + (dy * dy);
  if (std::isnormal(squared)) {
    return std::sqrt(squared);
  }
  return std::hypot(dx, dy);
}

/// The distance from a point to a segment whose squared length overflows.
/// The terms are taken in half units and divided by the longer axis of the
/// segment, so each stays finite. The nearest point comes from the end it lies
/// nearer to, so that a coordinate far larger than the distance cannot cancel.
double distanceToLongSegment(const Point point, const Point from,
                             const Point to) {
  const double fx = from.x() / 2.0;
  const double fy = from.y() / 2.0;
  const double tx = to.x() / 2.0;
  const double ty = to.y() / 2.0;
  const double px = point.x() / 2.0;
  const double py = point.y() / 2.0;
  const double dx = tx - fx;
  const double dy = ty - fy;
  const double scale = std::max(std::fabs(dx), std::fabs(dy));
  const double ux = dx / scale;
  const double uy = dy / scale;
  const double length2 = (ux * ux) + (uy * uy);
  // The parameter of the nearest point measured from either end.
  const double fromStart = std::clamp(
      ((((px - fx) / scale) * ux) + (((py - fy) / scale) * uy)) / length2, 0.0,
      1.0);
  const double fromEnd = std::clamp(
      ((((tx - px) / scale) * ux) + (((ty - py) / scale) * uy)) / length2, 0.0,
      1.0);
  const bool nearStart = fromStart <= fromEnd;
  const double cx = nearStart ? fx + (fromStart * dx) : tx - (fromEnd * dx);
  const double cy = nearStart ? fy + (fromStart * dy) : ty - (fromEnd * dy);
  return 2.0 * vectorLength(px - cx, py - cy);
}

} // namespace

double distanceToSegment(const Point point, const Point from, const Point to) {
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  const double length2 = (dx * dx) + (dy * dy);
  if (!std::isfinite(length2)) {
    return distanceToLongSegment(point, from, to);
  }
  if (length2 <= 1e-18) {
    return vectorLength(point.x() - from.x(), point.y() - from.y());
  }
  // The parameter of the nearest point measured from either end. The nearest
  // point comes from the end it lies nearer to, so that a coordinate far
  // larger than the distance cannot cancel.
  const double fromStart = std::clamp(
      (((point.x() - from.x()) * dx) + ((point.y() - from.y()) * dy)) / length2,
      0.0, 1.0);
  const double fromEnd = std::clamp(
      (((to.x() - point.x()) * dx) + ((to.y() - point.y()) * dy)) / length2,
      0.0, 1.0);
  const bool nearStart = fromStart <= fromEnd;
  const double cx =
      nearStart ? from.x() + (fromStart * dx) : to.x() - (fromEnd * dx);
  const double cy =
      nearStart ? from.y() + (fromStart * dy) : to.y() - (fromEnd * dy);
  return vectorLength(point.x() - cx, point.y() - cy);
}

std::vector<std::size_t> lineCells(const int64_t x0, const int64_t y0,
                                   const int64_t x1, const int64_t y1,
                                   const uint32_t width,
                                   const uint32_t height) {
  const uint64_t dx = span(x0, x1);
  const uint64_t dy = span(y0, y1);
  if (dx >= LINE_SPAN_LIMIT || dy >= LINE_SPAN_LIMIT) {
    throw std::invalid_argument("the ends of a line lie too far apart");
  }
  std::vector<std::size_t> cells;
  if (std::max(x0, x1) < 0 || std::max(y0, y1) < 0 ||
      std::cmp_greater_equal(std::min(x0, x1), width) ||
      std::cmp_greater_equal(std::min(y0, y1), height)) {
    return cells;
  }

  // The walk takes one step along the major axis per cell, and a step along
  // the minor axis when its error term asks for one. After k steps, the
  // offset along the minor axis is minor * k / length, rounded half up.
  const bool alongX = dx >= dy;
  const uint64_t length = std::max(dx, dy);
  const uint64_t minor = std::min(dx, dy);
  const int64_t sx = (x0 < x1) ? 1 : -1;
  const int64_t sy = (y0 < y1) ? 1 : -1;
  const auto offset = [&](const int64_t steps) -> int64_t {
    if (length == 0) {
      return 0;
    }
    const Division division =
        divideProduct(minor, static_cast<uint64_t>(steps), length);
    const bool up = division.remainder >= length - division.remainder;
    return static_cast<int64_t>(division.quotient) + (up ? 1 : 0);
  };

  // The steps whose cell lies on the grid along the major axis.
  const int64_t major0 = alongX ? x0 : y0;
  const int64_t majorStep = alongX ? sx : sy;
  const int64_t majorLast = static_cast<int64_t>(alongX ? width : height) - 1;
  const auto steps = static_cast<int64_t>(length);
  const int64_t majorFirst =
      std::max<int64_t>(0, majorStep > 0 ? -major0 : major0 - majorLast);
  const int64_t majorEnd =
      std::min<int64_t>(steps, majorStep > 0 ? majorLast - major0 : major0);

  // Of those, the steps whose cell lies on the grid along the minor axis.
  // The minor offset never decreases, so a binary search finds both ends.
  const int64_t minor0 = alongX ? y0 : x0;
  const int64_t minorStep = alongX ? sy : sx;
  const int64_t minorLast = static_cast<int64_t>(alongX ? height : width) - 1;
  const int64_t lowOffset = minorStep > 0 ? -minor0 : minor0 - minorLast;
  const int64_t highOffset = minorStep > 0 ? minorLast - minor0 : minor0;
  // The first step in [from, to] whose offset reaches the value, or to + 1.
  const auto firstReaching = [&](int64_t from, int64_t to,
                                 const int64_t value) {
    while (from <= to) {
      const int64_t middle = from + ((to - from) / 2);
      if (offset(middle) >= value) {
        to = middle - 1;
      } else {
        from = middle + 1;
      }
    }
    return from;
  };
  const int64_t first = firstReaching(majorFirst, majorEnd, lowOffset);
  const int64_t last = firstReaching(first, majorEnd, highOffset + 1) - 1;
  if (first > last) {
    return cells;
  }

  // The cell and the error term that the walk from the first end reaches
  // after the steps off the grid.
  const auto a = static_cast<int64_t>(dx);
  const auto b = static_cast<int64_t>(dy);
  int64_t err = a - b;
  int64_t minorSteps = 0;
  if (length > 0) {
    const Division division =
        divideProduct(minor, static_cast<uint64_t>(first), length);
    const auto remainder = static_cast<int64_t>(division.remainder);
    const bool up = division.remainder >= length - division.remainder;
    minorSteps = static_cast<int64_t>(division.quotient) + (up ? 1 : 0);
    err += alongX ? (up ? a : 0) - remainder : remainder - (up ? b : 0);
  }
  int64_t x = x0 + (sx * (alongX ? first : minorSteps));
  int64_t y = y0 + (sy * (alongX ? minorSteps : first));
  cells.reserve(static_cast<std::size_t>(last - first) + 1);
  for (int64_t step = first;; ++step) {
    cells.push_back((static_cast<std::size_t>(y) * width) +
                    static_cast<std::size_t>(x));
    if (step == last) {
      break;
    }
    const int64_t e2 = 2 * err;
    if (e2 >= -b) {
      err -= b;
      x += sx;
    }
    if (e2 <= a) {
      err += a;
      y += sy;
    }
  }
  return cells;
}

void fillPolygon(BitGrid& mask, const GridMetrics& grid,
                 const PolygonT& polygon) {
  const auto& vertices = polygon.vertices;
  if (vertices.empty()) {
    return;
  }
  std::vector<Point> nodes;
  nodes.reserve(vertices.size());
  auto lowX = static_cast<double>(grid.width);
  double highX = 0.0;
  auto lowY = static_cast<double>(grid.height);
  double highY = 0.0;
  for (const auto& vertex : vertices) {
    const Point node = grid.toCell(vertex);
    if (!std::isfinite(node.x()) || !std::isfinite(node.y())) {
      throw std::invalid_argument("a polygon vertex must be finite");
    }
    nodes.push_back(node);
    lowX = std::min(lowX, std::floor(node.x()));
    highX = std::max(highX, std::ceil(node.x()));
    lowY = std::min(lowY, std::floor(node.y()));
    highY = std::max(highY, std::ceil(node.y()));
  }
  // The window is the polygon's own box, so a polygon cannot flood the grid.
  // It is clamped onto the grid before the cast to an integer.
  const auto minX = static_cast<int64_t>(std::max(lowX, 0.0));
  const auto minY = static_cast<int64_t>(std::max(lowY, 0.0));
  const auto maxX = static_cast<int64_t>(
      std::min(highX, static_cast<double>(grid.width) - 1.0));
  const auto maxY = static_cast<int64_t>(
      std::min(highY, static_cast<double>(grid.height) - 1.0));

  // The area: every cell whose center lies inside, by ray casting along each
  // row. A cell center lies inside when an odd number of edges cross its row
  // right of it. The edge test is half-open, so a vertex on the row counts
  // for one of its two edges only.
  if (nodes.size() >= 3) {
    // The first column of the window at or right of a crossing.
    const auto firstColumnFrom = [&](const double crossing) {
      return static_cast<int64_t>(std::ceil(std::clamp(
          crossing, static_cast<double>(minX), static_cast<double>(maxX + 1))));
    };
    std::vector<double> crossings;
    for (int64_t y = minY; y <= maxY; ++y) {
      const auto py = static_cast<double>(y);
      crossings.clear();
      for (std::size_t i = 0, j = nodes.size() - 1; i < nodes.size(); j = i++) {
        const double xi = nodes[i].x();
        const double yi = nodes[i].y();
        const double xj = nodes[j].x();
        const double yj = nodes[j].y();
        if ((yi > py) != (yj > py)) {
          // From the end nearer to the row, so that a coordinate far larger
          // than the grid cannot cancel.
          const bool fromI = std::fabs(py - yi) <= std::fabs(py - yj);
          const double crossing =
              fromI ? ((xj - xi) * (py - yi) / (yj - yi)) + xi
                    : ((xi - xj) * (py - yj) / (yi - yj)) + xj;
          // A crossing that is not a number lies right of no cell center.
          if (!std::isnan(crossing)) {
            crossings.push_back(crossing);
          }
        }
      }
      std::ranges::sort(crossings);
      // The cells from crossing k - 1 up to crossing k have every crossing
      // from k on right of them.
      const std::size_t count = crossings.size();
      for (std::size_t k = 0; k <= count; ++k) {
        if ((count - k) % 2 == 0) {
          continue;
        }
        const int64_t from =
            (k == 0) ? minX : firstColumnFrom(crossings[k - 1]);
        const int64_t to =
            (k == count) ? maxX : firstColumnFrom(crossings[k]) - 1;
        for (int64_t x = from; x <= to; ++x) {
          mask.setCell(static_cast<uint32_t>(x), static_cast<uint32_t>(y));
        }
      }
    }
  }

  // The edges: a line of cells between the rounded vertex cells, so that the
  // outline has no gap where the area test misses a thin polygon. An end far
  // off the grid first moves along its edge onto the margin.
  const Point low(-EDGE_MARGIN, -EDGE_MARGIN);
  const Point high(static_cast<double>(grid.width) + EDGE_MARGIN,
                   static_cast<double>(grid.height) + EDGE_MARGIN);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    Point a = nodes[i];
    Point b = nodes[(i + 1) % nodes.size()];
    if (!clipToBox(a, b, low, high)) {
      continue;
    }
    for (const std::size_t index : lineCells(
             std::llround(a.x()), std::llround(a.y()), std::llround(b.x()),
             std::llround(b.y()), grid.width, grid.height)) {
      mask.set(index);
    }
  }
}

void removeIslands(BitGrid& mask) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  if (width < 3 || height < 3) {
    return;
  }
  const BitGrid before = mask;
  for (uint32_t y = 1; y + 1 < height; ++y) {
    for (uint32_t x = 1; x + 1 < width; ++x) {
      if (!before.testCell(x, y)) {
        continue;
      }
      int freeNeighbors = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!before.testCell(
                  static_cast<uint32_t>(static_cast<int64_t>(x) + dx),
                  static_cast<uint32_t>(static_cast<int64_t>(y) + dy))) {
            ++freeNeighbors;
          }
        }
      }
      if (freeNeighbors >= 7) {
        mask.setCell(x, y, false);
      }
    }
  }
}

void blockBorder(BitGrid& mask, const uint32_t alongX, const uint32_t alongY) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  // The columns and the rows to block at each edge. Neither count exceeds the
  // grid, so no index below wraps, whatever the size of the grid.
  const uint32_t columns = std::min(alongX, width);
  const uint32_t rows = std::min(alongY, height);
  for (uint32_t y = 0; y < height; ++y) {
    if (y < rows || y >= height - rows) {
      for (uint32_t x = 0; x < width; ++x) {
        mask.setCell(x, y);
      }
      continue;
    }
    for (uint32_t x = 0; x < columns; ++x) {
      mask.setCell(x, y);
      mask.setCell(width - 1 - x, y);
    }
  }
}

RasterizedObstacles rasterizeObstacles(const ChipT& chip,
                                       const GridMetrics& grid,
                                       const RasterOptions& options) {
  RasterizedObstacles result;
  result.blocked = BitGrid(grid.width, grid.height);
  BitGrid& blocked = result.blocked;

  for (const auto& obstacle : chip.obstacles) {
    if (obstacle != nullptr) {
      fillPolygon(blocked, grid, *obstacle);
    }
  }
  removeIslands(blocked);

  if (options.keepout > 0.0) {
    blockKeepout(chip, grid, options, blocked, result.keepoutCells,
                 result.exemptedCells);
  }
  blockBorder(blocked, options.borderX, options.borderY);
  return result;
}

} // namespace mqt::scpd::grid
