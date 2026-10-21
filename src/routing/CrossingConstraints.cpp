/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CrossingConstraints.hpp"

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/RoomRules.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

void CrossingConstraints::build(const uint32_t width, const uint32_t height,
                                const std::vector<Path>& feedlines,
                                const std::vector<bool>& skip,
                                const int expandRadius,
                                const std::size_t pinCells) {
  width_ = width;
  height_ = height;
  masks_.assign(static_cast<std::size_t>(width) * height, 0);
  const auto w = static_cast<int64_t>(width);
  const auto h = static_cast<int64_t>(height);
  const auto blockAround = [&](const int64_t cx, const int64_t cy,
                               const int radius) {
    for (int64_t dy = -radius; dy <= radius; ++dy) {
      for (int64_t dx = -radius; dx <= radius; ++dx) {
        const int64_t nx = cx + dx;
        const int64_t ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        masks_[(static_cast<std::size_t>(ny) * width) +
               static_cast<std::size_t>(nx)] = CURVE_ZONE;
      }
    }
  };
  // Within the radius, a disc: a square reaches a radius times the root of
  // two along its diagonals, and binds a wire that keeps the clearance from
  // the feedline there (user, 2026-10-13). Per heading, the front of the
  // disc: its offsets that the disc one step back does not hold. A cell
  // one step on along a straight run adds only its front to the discs
  // marked before, which mark the same bit.
  const auto reach = static_cast<int64_t>(expandRadius) * expandRadius;
  const auto inDisc = [&](const int64_t dx, const int64_t dy) {
    return (dx * dx) + (dy * dy) <= reach;
  };
  std::vector<std::pair<int64_t, int64_t>> disc;
  std::array<std::vector<std::pair<int64_t, int64_t>>, NUM_HEADINGS> fronts;
  for (int64_t dy = -expandRadius; dy <= expandRadius; ++dy) {
    for (int64_t dx = -expandRadius; dx <= expandRadius; ++dx) {
      if (!inDisc(dx, dy)) {
        continue;
      }
      disc.emplace_back(dx, dy);
      for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
        const auto step = headingVector(heading);
        if (!inDisc(dx + step.dx, dy + step.dy)) {
          fronts[heading].emplace_back(dx, dy);
        }
      }
    }
  }
  for (std::size_t k = 0; k < feedlines.size(); ++k) {
    if (k < skip.size() && skip[k]) {
      continue;
    }
    const Path& wire = feedlines[k];
    if (wire.empty()) {
      continue;
    }
    // A cell that steps to the next distinct cell along its own heading is
    // on a straight run. `straightCells` is that definition, shared with the
    // coupler insertion's room rules so that the two count the same cells.
    const std::vector<bool> straight = straightCells(wire);
    // The last straight cell marked, whose disc the next cell's disc on the
    // same heading, one step on, covers but for its front.
    const PathPoint* last = nullptr;
    for (std::size_t at = 0; at < wire.size(); ++at) {
      if (!straight[at]) {
        continue;
      }
      const PathPoint& cell = wire[at];
      const auto heading = static_cast<Heading>(cell.heading & 7U);
      const auto bit = static_cast<uint8_t>(1U << heading);
      const auto step = headingVector(heading);
      const bool onward = last != nullptr && last->heading == cell.heading &&
                          static_cast<int64_t>(cell.x) - last->x == step.dx &&
                          static_cast<int64_t>(cell.y) - last->y == step.dy;
      last = &cell;
      for (const auto& [dx, dy] : onward ? fronts[heading] : disc) {
        const int64_t nx = static_cast<int64_t>(cell.x) + dx;
        const int64_t ny = static_cast<int64_t>(cell.y) + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        uint8_t& mask = masks_[(static_cast<std::size_t>(ny) * width) +
                               static_cast<std::size_t>(nx)];
        if (mask != CURVE_ZONE) {
          mask |= bit;
        }
      }
    }
    for (std::size_t at = 0; at < wire.size(); ++at) {
      if (!straight[at]) {
        blockAround(wire[at].x, wire[at].y, 1);
      }
    }
    // The pin zones at both ends may not be crossed.
    for (std::size_t i = 0; i < pinCells && i < wire.size(); ++i) {
      blockAround(wire[i].x, wire[i].y, 1);
    }
    if (wire.size() > pinCells) {
      for (std::size_t i = wire.size() - pinCells; i < wire.size(); ++i) {
        blockAround(wire[i].x, wire[i].y, 1);
      }
    }
  }
}

void CrossingConstraints::clear() {
  masks_.clear();
  width_ = 0;
  height_ = 0;
}

bool CrossingConstraints::allowed(const uint32_t x, const uint32_t y,
                                  const Heading heading) const {
  if (masks_.empty()) {
    return true;
  }
  if (x >= width_ || y >= height_) {
    return false;
  }
  const uint8_t mask = masks_[(static_cast<std::size_t>(y) * width_) + x];
  if (mask == 0U) {
    return true;
  }
  for (Heading wireHeading = 0; wireHeading < NUM_HEADINGS; ++wireHeading) {
    if ((mask & static_cast<uint8_t>(1U << wireHeading)) == 0U) {
      continue;
    }
    if (!isOrthogonal(wireHeading, heading)) {
      return false;
    }
  }
  return true;
}

bool CrossingConstraints::allowedArriving(const uint32_t x, const uint32_t y,
                                          const Heading heading,
                                          const Heading arrivedWith) const {
  return allowed(x, y, heading) || allowed(x, y, arrivedWith);
}

uint8_t CrossingConstraints::maskAt(const uint32_t x, const uint32_t y) const {
  if (masks_.empty() || x >= width_ || y >= height_) {
    return 0;
  }
  return masks_[(static_cast<std::size_t>(y) * width_) + x];
}

} // namespace mqt::scpd::routing
