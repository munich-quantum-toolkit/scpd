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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::routing {

void CrossingConstraints::build(const uint32_t width, const uint32_t height,
                                const std::vector<Path>& feedlines,
                                const std::vector<bool>& skip,
                                const int expandRadius) {
  width_ = width;
  height_ = height;
  masks_.assign(static_cast<std::size_t>(width) * height, 0);
  const auto w = static_cast<int64_t>(width);
  const auto h = static_cast<int64_t>(height);
  const auto blockAround = [&](const int64_t cx, const int64_t cy, const int radius) {
    for (int64_t dy = -radius; dy <= radius; ++dy) {
      for (int64_t dx = -radius; dx <= radius; ++dx) {
        const int64_t nx = cx + dx;
        const int64_t ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        masks_[(static_cast<std::size_t>(ny) * width) + static_cast<std::size_t>(nx)] = CURVE_ZONE;
      }
    }
  };
  for (std::size_t k = 0; k < feedlines.size(); ++k) {
    if (k < skip.size() && skip[k]) {
      continue;
    }
    const Path& wire = feedlines[k];
    if (wire.empty()) {
      continue;
    }
    // A cell that steps to the next cell along its own heading is on a
    // straight run. The router lists a cell again where its heading changes
    // on it, so the step to test is the one to the next cell somewhere
    // else; a cell listed twice is no bend of its own.
    std::vector<bool> straight(wire.size(), false);
    for (std::size_t at = 0; at < wire.size(); ++at) {
      const PathPoint& cell = wire[at];
      std::size_t ahead = at + 1;
      while (ahead < wire.size() && wire[ahead].samePlace(cell)) {
        ++ahead;
      }
      if (ahead >= wire.size()) {
        continue;
      }
      const HeadingVector v = headingVector(cell.heading);
      const PathPoint& next = wire[ahead];
      straight[at] =
          static_cast<int64_t>(next.x) - static_cast<int64_t>(cell.x) == v.dx &&
          static_cast<int64_t>(next.y) - static_cast<int64_t>(cell.y) == v.dy;
    }
    for (std::size_t at = 0; at < wire.size(); ++at) {
      if (!straight[at]) {
        continue;
      }
      const PathPoint& cell = wire[at];
      const auto bit = static_cast<uint8_t>(1U << (cell.heading & 7U));
      for (int64_t dy = -expandRadius; dy <= expandRadius; ++dy) {
        for (int64_t dx = -expandRadius; dx <= expandRadius; ++dx) {
          const int64_t nx = static_cast<int64_t>(cell.x) + dx;
          const int64_t ny = static_cast<int64_t>(cell.y) + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
            continue;
          }
          uint8_t& mask = masks_[(static_cast<std::size_t>(ny) * width) + static_cast<std::size_t>(nx)];
          if (mask != CURVE_ZONE) {
            mask |= bit;
          }
        }
      }
    }
    for (std::size_t at = 0; at < wire.size(); ++at) {
      if (!straight[at]) {
        blockAround(wire[at].x, wire[at].y, 1);
      }
    }
    // The pin zones at both ends may not be crossed.
    for (std::size_t i = 0; i < 10 && i < wire.size(); ++i) {
      blockAround(wire[i].x, wire[i].y, 1);
    }
    if (wire.size() > 10) {
      for (std::size_t i = wire.size() - 10; i < wire.size(); ++i) {
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

uint8_t CrossingConstraints::maskAt(const uint32_t x, const uint32_t y) const {
  if (masks_.empty() || x >= width_ || y >= height_) {
    return 0;
  }
  return masks_[(static_cast<std::size_t>(y) * width_) + x];
}

} // namespace mqt::scpd::routing
