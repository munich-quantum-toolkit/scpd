/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Voronoi.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"

#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

/// Boost's Voronoi construction is exact over integers, so the sites are
/// placed on a grid ten times finer than the cells. The factor divides out of
/// every vertex again; it exists so that two sites one cell apart are ten
/// units apart and the arithmetic has room.
constexpr int SITE_SCALE = 10;

/// How far a walk over the boundary cells may travel and still call two sites
/// the same wall. Four steps is what separates the two sides of a corridor
/// from two neighbors on the same side of one.
constexpr int SAME_WALL_STEPS = 4;

/// A diagram needs three sites before it has an edge at all.
constexpr std::size_t MINIMUM_SITES = 3;

/// The eight steps around a cell.
constexpr std::array<int, 8> NEIGHBOR_DX = {-1, 0, 1, -1, 1, -1, 0, 1};
constexpr std::array<int, 8> NEIGHBOR_DY = {-1, -1, -1, 0, 0, 1, 1, 1};

/// The obstacle cells that touch free space, which are the sites of the
/// diagram. A cell in the middle of an obstacle contributes nothing to the
/// shape of the free space around it.
struct Sites {
  std::vector<boost::polygon::point_data<int>> points;
  std::vector<int> x;
  std::vector<int> y;
  /// The site at a cell, or -1.
  std::vector<int> at;
};

/// Whether a position is blocked. Everything outside the grid counts as
/// blocked, so the border of the grid is a wall like any other.
bool blockedAt(const BitGrid& blocked, const int64_t x, const int64_t y) {
  if (x < 0 || y < 0 || x >= static_cast<int64_t>(blocked.width()) ||
      y >= static_cast<int64_t>(blocked.height())) {
    return true;
  }
  return blocked.test((static_cast<std::size_t>(y) * blocked.width()) + static_cast<std::size_t>(x));
}

Sites collectSites(const BitGrid& blocked) {
  Sites sites;
  sites.at.assign(blocked.size(), -1);
  const auto width = static_cast<int64_t>(blocked.width());
  const auto height = static_cast<int64_t>(blocked.height());

  for (int64_t y = 0; y < height; ++y) {
    for (int64_t x = 0; x < width; ++x) {
      if (!blockedAt(blocked, x, y)) {
        continue;
      }
      // Only a cell with a free four-neighbor is on a wall's surface.
      if (blockedAt(blocked, x - 1, y) && blockedAt(blocked, x + 1, y) &&
          blockedAt(blocked, x, y - 1) && blockedAt(blocked, x, y + 1)) {
        continue;
      }
      sites.at[(static_cast<std::size_t>(y) * blocked.width()) + static_cast<std::size_t>(x)] =
          static_cast<int>(sites.points.size());
      sites.points.emplace_back(static_cast<int>(x) * SITE_SCALE, static_cast<int>(y) * SITE_SCALE);
      sites.x.push_back(static_cast<int>(x));
      sites.y.push_back(static_cast<int>(y));
    }
  }
  return sites;
}

/// Whether two sites lie on the same wall, decided by a bounded walk over the
/// boundary cells between them.
class SameWall {
public:
  SameWall(const Sites& sites, const BitGrid& blocked)
      : sites_(sites), blocked_(blocked), stamp_(sites.points.size(), 0) {}

  [[nodiscard]] bool operator()(const int from, const int to) {
    if (from == to) {
      return true;
    }
    ++current_;
    queue_.clear();
    queue_.emplace_back(from, 0);
    stamp_[static_cast<std::size_t>(from)] = current_;

    for (std::size_t head = 0; head < queue_.size(); ++head) {
      const auto [site, depth] = queue_[head];
      if (site == to) {
        return true;
      }
      if (depth == SAME_WALL_STEPS) {
        continue;
      }
      for (std::size_t step = 0; step < NEIGHBOR_DX.size(); ++step) {
        const auto nx = sites_.x[static_cast<std::size_t>(site)] + NEIGHBOR_DX[step];
        const auto ny = sites_.y[static_cast<std::size_t>(site)] + NEIGHBOR_DY[step];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(blocked_.width()) ||
            ny >= static_cast<int>(blocked_.height())) {
          continue;
        }
        const auto next = sites_.at[(static_cast<std::size_t>(ny) * blocked_.width()) +
                                    static_cast<std::size_t>(nx)];
        if (next < 0 || stamp_[static_cast<std::size_t>(next)] == current_) {
          continue;
        }
        stamp_[static_cast<std::size_t>(next)] = current_;
        queue_.emplace_back(next, depth + 1);
      }
    }
    return false;
  }

private:
  const Sites& sites_;
  const BitGrid& blocked_;
  std::vector<int> stamp_;
  int current_ = 0;
  std::vector<std::pair<int, int>> queue_;
};

/// Whether the straight segment between two points stays in free space. The
/// segment is sampled at one point per cell of its length, which is what
/// keeps a diagonal from stepping over a one-cell wall.
bool freeSegment(const BitGrid& blocked, const double ax, const double ay, const double bx,
                 const double by) {
  const auto length = std::hypot(bx - ax, by - ay);
  const auto steps = std::max(1, static_cast<int>(std::ceil(length)));
  for (int step = 0; step <= steps; ++step) {
    const auto t = static_cast<double>(step) / steps;
    if (blockedAt(blocked, std::lround(ax + (t * (bx - ax))),
                  std::lround(ay + (t * (by - ay))))) {
      return false;
    }
  }
  return true;
}

} // namespace

std::vector<MedialEdge> medialAxis(const BitGrid& blocked) {
  std::vector<MedialEdge> edges;
  if (blocked.empty()) {
    return edges;
  }

  const auto sites = collectSites(blocked);
  if (sites.points.size() < MINIMUM_SITES) {
    return edges;
  }

  boost::polygon::voronoi_diagram<double> diagram;
  boost::polygon::construct_voronoi(sites.points.begin(), sites.points.end(), &diagram);

  SameWall sameWall(sites, blocked);
  for (const auto& edge : diagram.edges()) {
    if (!edge.is_primary() || !edge.is_finite()) {
      continue;
    }
    const auto* from = edge.vertex0();
    const auto* to = edge.vertex1();
    if (from == nullptr || to == nullptr) {
      continue;
    }

    // Every edge appears twice, once from each side. Keeping the one whose
    // own cell has the lower site index emits it exactly once.
    const auto near = static_cast<int>(edge.cell()->source_index());
    const auto far = static_cast<int>(edge.twin()->cell()->source_index());
    if (near > far) {
      continue;
    }
    if (sameWall(near, far)) {
      continue;
    }

    const auto x0 = from->x() / SITE_SCALE;
    const auto y0 = from->y() / SITE_SCALE;
    const auto x1 = to->x() / SITE_SCALE;
    const auto y1 = to->y() / SITE_SCALE;
    if (!freeSegment(blocked, x0, y0, x1, y1)) {
      continue;
    }
    edges.push_back({x0, y0, x1, y1});
  }
  return edges;
}

MedialAxis rasterizeMedialAxis(const BitGrid& blocked, const GridMetrics& grid,
                               const std::vector<MedialEdge>& edges) {
  if (blocked.width() != grid.width || blocked.height() != grid.height) {
    throw std::invalid_argument(
        std::format("the mask is {}x{} cells and the grid {}x{}", blocked.width(),
                    blocked.height(), grid.width, grid.height));
  }

  MedialAxis axis;
  axis.cells.assign(grid.cells(), AxisCell::None);
  if (edges.empty()) {
    return axis;
  }

  for (const auto& edge : edges) {
    // The endpoints are truncated rather than rounded, because that is what
    // decides which cell a vertex exactly between two cells belongs to, and
    // rounding moves the whole axis by half a cell against the mask it was
    // derived from.
    const auto cells = lineCells(static_cast<int64_t>(edge.x0), static_cast<int64_t>(edge.y0),
                                 static_cast<int64_t>(edge.x1), static_cast<int64_t>(edge.y1),
                                 grid.width, grid.height);
    std::size_t previous = grid.cells();
    for (const auto cell : cells) {
      if (blocked.test(cell)) {
        continue;
      }
      axis.cells[cell] = AxisCell::Path;
      auto& neighbors = axis.neighbors[cell];
      if (previous != grid.cells() && previous != cell) {
        if (std::ranges::find(neighbors, previous) == neighbors.end()) {
          neighbors.push_back(previous);
        }
        auto& before = axis.neighbors[previous];
        if (std::ranges::find(before, cell) == before.end()) {
          before.push_back(cell);
        }
      }
      previous = cell;
    }
  }

  for (const auto& [cell, neighbors] : axis.neighbors) {
    if (neighbors.size() > 2) {
      axis.cells[cell] = AxisCell::Junction;
    } else if (neighbors.size() == 1) {
      axis.cells[cell] = AxisCell::Endpoint;
    }
  }
  return axis;
}

} // namespace mqt::scpd::grid
