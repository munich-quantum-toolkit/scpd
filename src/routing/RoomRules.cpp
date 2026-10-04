/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/RoomRules.hpp"

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <vector>

namespace mqt::scpd::routing {

std::vector<bool> straightCells(const Path& path) {
  std::vector<bool> straight(path.size(), false);
  for (std::size_t at = 0; at < path.size(); ++at) {
    const PathPoint& cell = path[at];
    std::size_t ahead = at + 1;
    while (ahead < path.size() && path[ahead].samePlace(cell)) {
      ++ahead;
    }
    if (ahead >= path.size()) {
      continue;
    }
    const HeadingVector v = headingVector(cell.heading);
    const PathPoint& next = path[ahead];
    straight[at] =
        static_cast<int64_t>(next.x) - static_cast<int64_t>(cell.x) == v.dx &&
        static_cast<int64_t>(next.y) - static_cast<int64_t>(cell.y) == v.dy;
  }
  return straight;
}

uint32_t lanesOf(const uint32_t run, const uint32_t pitch,
                 const uint32_t margin) {
  const auto usable =
      static_cast<int64_t>(run) - 1 - (2 * static_cast<int64_t>(margin));
  if (usable < 0) {
    return 0;
  }
  if (pitch == 0) {
    return 1;
  }
  return 1 + static_cast<uint32_t>(usable / static_cast<int64_t>(pitch));
}

CrossingCapacity crossingCapacity(const Path& path, const std::size_t skipHead,
                                  const std::size_t skipTail,
                                  const uint32_t pitch, const uint32_t margin) {
  CrossingCapacity out;
  if (path.size() <= skipHead + skipTail) {
    return out;
  }
  const auto straight = straightCells(path);
  const auto end = path.size() - skipTail;
  uint32_t run = 0;
  const auto close = [&]() {
    if (run > 0) {
      out.runs.push_back(run);
      out.lanes += lanesOf(run, pitch, margin);
      run = 0;
    }
  };
  for (std::size_t at = skipHead; at < end; ++at) {
    if (straight[at]) {
      ++run;
    } else {
      close();
    }
  }
  close();
  return out;
}

Channel channelBetween(const Path& a, const std::size_t aSkipTail,
                       const std::size_t aTake, const Path& b,
                       const std::size_t bSkipHead, const std::size_t bTake) {
  Channel out;
  if (a.size() <= aSkipTail || b.size() <= bSkipHead) {
    return out;
  }
  // The first arm, from the cell before the skipped run backwards.
  const auto aEnd = a.size() - aSkipTail;
  const auto aBegin = aEnd > aTake ? aEnd - aTake : std::size_t{0};
  // The second arm, from the cell after the skipped run forwards.
  const auto bBegin = bSkipHead;
  const auto bEnd = std::min(b.size(), bBegin + bTake);
  if (aBegin >= aEnd || bBegin >= bEnd) {
    return out;
  }
  auto least = std::numeric_limits<double>::infinity();
  for (std::size_t i = aEnd; i-- > aBegin;) {
    const auto ax = static_cast<double>(a[i].x);
    const auto ay = static_cast<double>(a[i].y);
    for (std::size_t j = bBegin; j < bEnd; ++j) {
      const auto dx = ax - static_cast<double>(b[j].x);
      const auto dy = ay - static_cast<double>(b[j].y);
      const auto d = std::sqrt((dx * dx) + (dy * dy));
      if (d < least) {
        least = d;
        out.at = i;
        out.bt = j;
      }
    }
  }
  out.measured = true;
  out.gap = least;
  {
    const auto dx =
        static_cast<double>(a[aEnd - 1].x) - static_cast<double>(b[bBegin].x);
    const auto dy =
        static_cast<double>(a[aEnd - 1].y) - static_cast<double>(b[bBegin].y);
    out.startGap = std::sqrt((dx * dx) + (dy * dy));
  }
  out.aDepth = aEnd - 1 - out.at;
  out.bDepth = out.bt - bBegin;
  return out;
}

void supercoverLine(const int64_t x0, const int64_t y0, const int64_t x1,
                    const int64_t y1,
                    const std::function<void(int64_t, int64_t)>& visit) {
  int64_t x = x0;
  int64_t y = y0;
  const int64_t dx = std::abs(x1 - x0);
  const int64_t dy = std::abs(y1 - y0);
  const int64_t xstep = x1 >= x0 ? 1 : -1;
  const int64_t ystep = y1 >= y0 ? 1 : -1;
  visit(x, y);
  const int64_t ddx = 2 * dx;
  const int64_t ddy = 2 * dy;
  if (ddx >= ddy) {
    int64_t error = dx;
    int64_t errorPrevious = dx;
    for (int64_t i = 0; i < dx; ++i) {
      x += xstep;
      error += ddy;
      if (error > ddx) {
        y += ystep;
        error -= ddx;
        if (error + errorPrevious < ddx) {
          visit(x, y - ystep);
        } else if (error + errorPrevious > ddx) {
          visit(x - xstep, y);
        } else {
          visit(x, y - ystep);
          visit(x - xstep, y);
        }
      }
      visit(x, y);
      errorPrevious = error;
    }
  } else {
    int64_t error = dy;
    int64_t errorPrevious = dy;
    for (int64_t i = 0; i < dy; ++i) {
      y += ystep;
      error += ddx;
      if (error > ddy) {
        x += xstep;
        error -= ddy;
        if (error + errorPrevious < ddy) {
          visit(x - xstep, y);
        } else if (error + errorPrevious > ddy) {
          visit(x, y - ystep);
        } else {
          visit(x - xstep, y);
          visit(x, y - ystep);
        }
      }
      visit(x, y);
      errorPrevious = error;
    }
  }
}

int sideOf(const PathPoint& centre, const HeadingVector across,
           const PathPoint& point) {
  const auto dx =
      static_cast<int64_t>(point.x) - static_cast<int64_t>(centre.x);
  const auto dy =
      static_cast<int64_t>(point.y) - static_cast<int64_t>(centre.y);
  const auto dot = (dx * across.dx) + (dy * across.dy);
  return dot > 0 ? 1 : (dot < 0 ? -1 : 0);
}

bool sameSide(const PathPoint& centre, const HeadingVector across,
              const PathPoint& a, const PathPoint& b) {
  const auto sa = sideOf(centre, across, a);
  return sa != 0 && sa == sideOf(centre, across, b);
}

} // namespace mqt::scpd::routing
