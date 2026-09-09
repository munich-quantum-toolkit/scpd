/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/SelfIntersection.hpp"

#include "mqt-scpd/routing/Path.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace mqt::scpd::routing {

namespace {

uint64_t cellKey(const int32_t x, const int32_t y) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32U) |
         static_cast<uint32_t>(y);
}

} // namespace

void rasterizePathCells(const Path& path, const uint32_t width,
                        const uint32_t height, std::vector<int32_t>& xs,
                        std::vector<int32_t>& ys) {
  const auto w = static_cast<int32_t>(width);
  const auto h = static_cast<int32_t>(height);
  xs.clear();
  ys.clear();
  const auto push = [&](const int32_t x, const int32_t y) {
    if (x < 0 || y < 0 || x >= w || y >= h) {
      return;
    }
    if (!xs.empty() && xs.back() == x && ys.back() == y) {
      return;
    }
    xs.push_back(x);
    ys.push_back(y);
  };
  if (path.empty()) {
    return;
  }
  if (path.size() == 1) {
    push(static_cast<int32_t>(path[0].x), static_cast<int32_t>(path[0].y));
    return;
  }
  for (std::size_t k = 0; k + 1 < path.size(); ++k) {
    const auto x1 = static_cast<int32_t>(path[k].x);
    const auto y1 = static_cast<int32_t>(path[k].y);
    const auto x2 = static_cast<int32_t>(path[k + 1].x);
    const auto y2 = static_cast<int32_t>(path[k + 1].y);
    const int32_t dx = x2 - x1;
    const int32_t dy = y2 - y1;
    const int32_t steps = std::max(std::abs(dx), std::abs(dy));
    const double divisor = steps != 0 ? steps : 1;
    const double xInc = dx / divisor;
    const double yInc = dy / divisor;
    double x = x1;
    double y = y1;
    for (int32_t v = 0; v <= steps; ++v) {
      push(static_cast<int32_t>(std::lround(x)),
           static_cast<int32_t>(std::lround(y)));
      x += xInc;
      y += yInc;
    }
  }
}

uint32_t scanCellsForSelfIntersection(PathLoopScratch& scratch,
                                      std::vector<PathLoopHit>* hits,
                                      const bool stopAtFirst) {
  const auto& cx = scratch.xs;
  const auto& cy = scratch.ys;
  scratch.spurRevisitsIgnored = 0;
  scratch.maxSpurDistance = 0;
  if (cx.size() < 3) {
    return 0;
  }
  scratch.seenCell.clear();
  scratch.seenDiagonal.clear();
  uint32_t found = 0;

  for (std::size_t j = 0; j < cx.size(); ++j) {
    const uint64_t key = cellKey(cx[j], cy[j]);
    bool hit = false;
    PathLoopKind kind = PathLoopKind::Revisit;
    std::size_t first = j;

    // A diagonal crossing on the step that arrives at j.
    if (j > 0) {
      const int32_t dx = cx[j] - cx[j - 1];
      const int32_t dy = cy[j] - cy[j - 1];
      if (dx != 0 && dy != 0) {
        const uint64_t blockKey =
            cellKey(std::min(cx[j], cx[j - 1]), std::min(cy[j], cy[j - 1]));
        const uint8_t bit = (dx * dy > 0) ? 0x1U : 0x2U;
        const std::size_t slot = (bit == 0x1U) ? 0U : 1U;
        auto& seen = scratch.seenDiagonal[blockKey];
        if ((seen.mask & static_cast<uint8_t>(~bit)) != 0) {
          hit = true;
          kind = PathLoopKind::DiagonalCross;
          first = seen.at[1U - slot];
        } else {
          seen.mask |= bit;
          seen.at[slot] = j;
        }
      }
    }

    // A plain revisit, unless the two visits are close enough to be the
    // heading-change re-emission, in which case the newer visit replaces the
    // older one so that a chain of them cannot add up to a long-range hit.
    if (!hit) {
      const auto it = scratch.seenCell.find(key);
      if (it == scratch.seenCell.end()) {
        scratch.seenCell.emplace(key, j);
      } else if (j - it->second <= PATH_LOOP_SPUR_WINDOW) {
        ++scratch.spurRevisitsIgnored;
        scratch.maxSpurDistance = std::max(scratch.maxSpurDistance, j - it->second);
        it->second = j;
      } else {
        hit = true;
        kind = PathLoopKind::Revisit;
        first = it->second;
      }
    }

    if (!hit) {
      continue;
    }
    if (hits != nullptr) {
      hits->push_back({.kind = kind,
                       .x = static_cast<uint32_t>(cx[j]),
                       .y = static_cast<uint32_t>(cy[j]),
                       .firstIndex = first,
                       .secondIndex = j});
    }
    ++found;
    if (stopAtFirst) {
      return found;
    }
    scratch.seenCell.clear();
    scratch.seenDiagonal.clear();
    scratch.seenCell.emplace(key, j);
  }
  return found;
}

uint32_t findPathSelfIntersections(const Path& path, const uint32_t width,
                                   const uint32_t height,
                                   PathLoopScratch& scratch,
                                   std::vector<PathLoopHit>* hits) {
  rasterizePathCells(path, width, height, scratch.xs, scratch.ys);
  return scanCellsForSelfIntersection(scratch, hits, false);
}

bool pathSelfIntersects(const Path& path, const uint32_t width,
                        const uint32_t height, PathLoopScratch& scratch,
                        PathLoopHit* first) {
  rasterizePathCells(path, width, height, scratch.xs, scratch.ys);
  std::vector<PathLoopHit> one;
  const uint32_t n =
      scanCellsForSelfIntersection(scratch, first != nullptr ? &one : nullptr, true);
  if (n != 0 && first != nullptr && !one.empty()) {
    *first = one.front();
  }
  return n != 0;
}

} // namespace mqt::scpd::routing
