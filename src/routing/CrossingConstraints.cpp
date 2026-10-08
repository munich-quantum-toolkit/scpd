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

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/**
 * @brief Marks the points of a feedline that lie on a turn.
 *
 * A run of points under one tag whose next point has another heading is a
 * turn. The turn covers the run and the point after it, which is the end of
 * the arc. The turn after the first run of the feedline also covers the point
 * before its run, which is the start of the arc where the search began with
 * the turn (see Path).
 *
 * @param feedline The feedline.
 * @return One flag per point of @p feedline, set for a point on a turn.
 */
std::vector<bool> turnPoints(const Path& feedline) {
  std::vector<bool> onTurn(feedline.size(), false);
  const auto sameTag = [&](const std::size_t a, const std::size_t b) {
    return feedline[a].heading == feedline[b].heading &&
           feedline[a].primitive == feedline[b].primitive;
  };
  std::size_t firstRunEnd = 1;
  while (firstRunEnd < feedline.size() && sameTag(firstRunEnd, 0)) {
    ++firstRunEnd;
  }
  std::size_t begin = 0;
  while (begin < feedline.size()) {
    std::size_t end = begin + 1;
    while (end < feedline.size() && sameTag(end, begin)) {
      ++end;
    }
    if (end < feedline.size() &&
        feedline[end].heading != feedline[begin].heading) {
      const std::size_t from = begin == firstRunEnd ? begin - 1 : begin;
      for (std::size_t at = from; at <= end; ++at) {
        onTurn[at] = true;
      }
    }
    begin = end;
  }
  return onTurn;
}

} // namespace

void CrossingConstraints::build(const uint32_t width, const uint32_t height,
                                const std::vector<Path>& feedlines,
                                const std::vector<bool>& skip,
                                const int expandRadius) {
  std::vector<uint8_t> builtMasks(static_cast<std::size_t>(width) * height, 0);
  const auto w = static_cast<int64_t>(width);
  const auto h = static_cast<int64_t>(height);
  const auto closeAround = [&](const int64_t cx, const int64_t cy,
                               const int radius) {
    for (int64_t dy = -radius; dy <= radius; ++dy) {
      for (int64_t dx = -radius; dx <= radius; ++dx) {
        const int64_t nx = cx + dx;
        const int64_t ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        builtMasks[(static_cast<std::size_t>(ny) * width) +
                   static_cast<std::size_t>(nx)] = CURVE_ZONE;
      }
    }
  };
  for (std::size_t k = 0; k < feedlines.size(); ++k) {
    if (k < skip.size() && skip[k]) {
      continue;
    }
    const Path& feedline = feedlines[k];
    if (feedline.empty()) {
      continue;
    }
    // A cell that steps to the next cell along its own heading is on a
    // straight run. A path can list a cell twice in a row, so the step to
    // test is the one to the next cell somewhere else.
    std::vector<bool> straight(feedline.size(), false);
    for (std::size_t at = 0; at < feedline.size(); ++at) {
      const PathPoint& cell = feedline[at];
      std::size_t ahead = at + 1;
      while (ahead < feedline.size() && feedline[ahead].samePlace(cell)) {
        ++ahead;
      }
      if (ahead >= feedline.size()) {
        continue;
      }
      const HeadingVector v = headingVector(cell.heading);
      const PathPoint& next = feedline[ahead];
      straight[at] =
          static_cast<int64_t>(next.x) - static_cast<int64_t>(cell.x) == v.dx &&
          static_cast<int64_t>(next.y) - static_cast<int64_t>(cell.y) == v.dy;
    }
    for (std::size_t at = 0; at < feedline.size(); ++at) {
      if (!straight[at]) {
        continue;
      }
      const PathPoint& cell = feedline[at];
      const auto bit = static_cast<uint8_t>(1U << (cell.heading & 7U));
      for (int64_t dy = -expandRadius; dy <= expandRadius; ++dy) {
        for (int64_t dx = -expandRadius; dx <= expandRadius; ++dx) {
          const int64_t nx = static_cast<int64_t>(cell.x) + dx;
          const int64_t ny = static_cast<int64_t>(cell.y) + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
            continue;
          }
          uint8_t& mask = builtMasks[(static_cast<std::size_t>(ny) * width) +
                                    static_cast<std::size_t>(nx)];
          if (mask != CURVE_ZONE) {
            mask |= bit;
          }
        }
      }
    }
    // Every cell of a turn closes the cells around it, even where the turn
    // sweeps cells straight ahead: its rendered curve bends from the start
    // of the arc.
    const std::vector<bool> onTurn = turnPoints(feedline);
    for (std::size_t at = 0; at < feedline.size(); ++at) {
      if (!straight[at] || onTurn[at]) {
        closeAround(feedline[at].x, feedline[at].y, 1);
      }
    }
    // The end zones, the first and last ten cells, close the cells around
    // them.
    for (std::size_t i = 0; i < 10 && i < feedline.size(); ++i) {
      closeAround(feedline[i].x, feedline[i].y, 1);
    }
    if (feedline.size() > 10) {
      for (std::size_t i = feedline.size() - 10; i < feedline.size(); ++i) {
        closeAround(feedline[i].x, feedline[i].y, 1);
      }
    }
  }
  masks = std::move(builtMasks);
  gridWidth = width;
  gridHeight = height;
}

void CrossingConstraints::clear() {
  masks = std::vector<uint8_t>();
  gridWidth = 0;
  gridHeight = 0;
}

bool CrossingConstraints::allowed(const uint32_t x, const uint32_t y,
                                  const Heading heading) const {
  if (masks.empty()) {
    return true;
  }
  if (x >= gridWidth || y >= gridHeight) {
    return false;
  }
  const uint8_t mask = masks[(static_cast<std::size_t>(y) * gridWidth) + x];
  if (mask == 0U) {
    return true;
  }
  const auto step = static_cast<Heading>(heading & 7U);
  for (Heading wireHeading = 0; wireHeading < NUM_HEADINGS; ++wireHeading) {
    if ((mask & static_cast<uint8_t>(1U << wireHeading)) == 0U) {
      continue;
    }
    if (!isOrthogonal(wireHeading, step)) {
      return false;
    }
  }
  return true;
}

bool CrossingConstraints::turnAllowed(const uint32_t x,
                                      const uint32_t y) const {
  if (masks.empty()) {
    return true;
  }
  if (x >= gridWidth || y >= gridHeight) {
    return false;
  }
  return masks[(static_cast<std::size_t>(y) * gridWidth) + x] == 0U;
}

} // namespace mqt::scpd::routing
