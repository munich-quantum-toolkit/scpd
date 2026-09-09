/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Watershed.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <span>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

namespace {

constexpr double INFINITE = std::numeric_limits<double>::infinity();
/// Two arrival times closer than this are a tie, decided by the seed index.
constexpr double TIE_EPSILON = 1e-6;
constexpr std::size_t NO_CELL = std::numeric_limits<std::size_t>::max();

struct FrontEntry {
  std::size_t cell;
  double time;
  bool operator>(const FrontEntry& other) const { return time > other.time; }
};

} // namespace

PartitionLabel runWatershed(const BitGrid& blocked,
                            const std::span<const std::size_t> seeds,
                            std::vector<PartitionLabel>& labels,
                            PartitionLabel nextLabel) {
  const uint32_t width = blocked.width();
  const uint32_t height = blocked.height();
  const std::size_t total = blocked.size();
  if (total == 0 || labels.size() < total) {
    return nextLabel;
  }
  const PartitionLabel firstLabel = nextLabel;

  std::vector<double> time(total, INFINITE);
  std::vector<PartitionLabel> tentativeLabel(total, LABEL_NONE);
  std::vector<uint32_t> tentativeSeed(total, std::numeric_limits<uint32_t>::max());
  std::vector<bool> finalized(total, false);
  std::priority_queue<FrontEntry, std::vector<FrontEntry>, std::greater<>> front;

  const auto isBarrier = [&](const std::size_t cell) {
    return blocked.test(cell) ||
           (labels[cell] != LABEL_NONE && labels[cell] < firstLabel);
  };

  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const std::size_t seed = seeds[i];
    if (seed >= total || blocked.test(seed) || labels[seed] != LABEL_NONE) {
      continue;
    }
    time[seed] = 0.0;
    labels[seed] = nextLabel;
    tentativeLabel[seed] = nextLabel;
    tentativeSeed[seed] = static_cast<uint32_t>(i);
    finalized[seed] = true;
    front.push({seed, 0.0});
    ++nextLabel;
  }

  // The arrival time of a cell from its finalized four-neighbors, and the
  // neighbor it arrives from.
  const auto solve = [&](const uint32_t x,
                         const uint32_t y) -> std::pair<double, std::size_t> {
    double a = INFINITE;
    std::size_t aCell = NO_CELL;
    double b = INFINITE;
    std::size_t bCell = NO_CELL;
    const auto consider = [&](const std::size_t n, double& best,
                              std::size_t& bestCell) {
      if (!blocked.test(n) && finalized[n] && time[n] < best) {
        best = time[n];
        bestCell = n;
      }
    };
    const std::size_t row = static_cast<std::size_t>(y) * width;
    if (x > 0) {
      consider(row + x - 1, a, aCell);
    }
    if (x + 1 < width) {
      consider(row + x + 1, a, aCell);
    }
    if (y > 0) {
      consider(row - width + x, b, bCell);
    }
    if (y + 1 < height) {
      consider(row + width + x, b, bCell);
    }
    if (aCell == NO_CELL && bCell == NO_CELL) {
      return {INFINITE, NO_CELL};
    }
    if (aCell == NO_CELL) {
      return {b + 1.0, bCell};
    }
    if (bCell == NO_CELL) {
      return {a + 1.0, aCell};
    }
    const double diff = a - b;
    const double discriminant = 2.0 - (diff * diff);
    if (discriminant < 0.0) {
      return (a <= b) ? std::make_pair(a + 1.0, aCell)
                      : std::make_pair(b + 1.0, bCell);
    }
    const double solved = (a + b + std::sqrt(discriminant)) * 0.5;
    if (solved < std::max(a, b)) {
      return (a <= b) ? std::make_pair(a + 1.0, aCell)
                      : std::make_pair(b + 1.0, bCell);
    }
    if (std::fabs(a - b) < TIE_EPSILON) {
      return (tentativeSeed[aCell] <= tentativeSeed[bCell])
                 ? std::make_pair(solved, aCell)
                 : std::make_pair(solved, bCell);
    }
    return (a <= b) ? std::make_pair(solved, aCell)
                    : std::make_pair(solved, bCell);
  };

  const auto relax = [&](const std::size_t cell) {
    if (finalized[cell] || isBarrier(cell)) {
      return;
    }
    const auto [arrival, from] =
        solve(static_cast<uint32_t>(cell % width),
              static_cast<uint32_t>(cell / width));
    if (from == NO_CELL) {
      return;
    }
    bool take = false;
    if (time[cell] == INFINITE || arrival < time[cell] - TIE_EPSILON) {
      take = true;
    } else if (std::fabs(arrival - time[cell]) <= TIE_EPSILON &&
               tentativeSeed[from] < tentativeSeed[cell]) {
      take = true;
    }
    if (take) {
      time[cell] = arrival;
      tentativeLabel[cell] = tentativeLabel[from];
      tentativeSeed[cell] = tentativeSeed[from];
      front.push({cell, arrival});
    }
  };

  const auto relaxNeighbors = [&](const std::size_t cell) {
    const auto x = static_cast<uint32_t>(cell % width);
    const auto y = static_cast<uint32_t>(cell / width);
    if (x > 0) {
      relax(cell - 1);
    }
    if (x + 1 < width) {
      relax(cell + 1);
    }
    if (y > 0) {
      relax(cell - width);
    }
    if (y + 1 < height) {
      relax(cell + width);
    }
  };

  for (const std::size_t seed : seeds) {
    if (seed < total && finalized[seed]) {
      relaxNeighbors(seed);
    }
  }
  while (!front.empty()) {
    const FrontEntry current = front.top();
    front.pop();
    const std::size_t cell = current.cell;
    if (finalized[cell] || current.time > time[cell] + TIE_EPSILON) {
      continue;
    }
    finalized[cell] = true;
    labels[cell] = tentativeLabel[cell];
    relaxNeighbors(cell);
  }
  return nextLabel;
}

void smoothPartitionBorders(const BitGrid& blocked,
                            std::vector<PartitionLabel>& labels,
                            const PartitionLabel firstLabel, const int radius,
                            const int iterations) {
  const uint32_t width = blocked.width();
  const uint32_t height = blocked.height();
  const std::size_t total = blocked.size();
  if (total == 0 || labels.size() < total) {
    return;
  }
  const auto ofThisRun = [&](const std::size_t cell) {
    return labels[cell] >= firstLabel;
  };
  static constexpr int64_t DX4[4] = {1, -1, 0, 0};
  static constexpr int64_t DY4[4] = {0, 0, 1, -1};

  for (int iteration = 0; iteration < iterations; ++iteration) {
    std::vector<PartitionLabel> next = labels;
    bool changed = false;
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const std::size_t cell = (static_cast<std::size_t>(y) * width) + x;
        if (blocked.test(cell) || !ofThisRun(cell)) {
          continue;
        }
        const PartitionLabel own = labels[cell];
        bool border = false;
        for (int k = 0; k < 4 && !border; ++k) {
          const int64_t nx = static_cast<int64_t>(x) + DX4[k];
          const int64_t ny = static_cast<int64_t>(y) + DY4[k];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const std::size_t n = (static_cast<std::size_t>(ny) * width) +
                                static_cast<std::size_t>(nx);
          if (blocked.test(n)) {
            continue;
          }
          if (!ofThisRun(n) || labels[n] != own) {
            border = true;
          }
        }
        if (!border) {
          continue;
        }
        std::map<PartitionLabel, int> counts;
        int valid = 0;
        for (int64_t oy = -radius; oy <= radius; ++oy) {
          for (int64_t ox = -radius; ox <= radius; ++ox) {
            const int64_t nx = static_cast<int64_t>(x) + ox;
            const int64_t ny = static_cast<int64_t>(y) + oy;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
              continue;
            }
            const std::size_t n = (static_cast<std::size_t>(ny) * width) +
                                  static_cast<std::size_t>(nx);
            if (blocked.test(n) || !ofThisRun(n)) {
              continue;
            }
            ++counts[labels[n]];
            ++valid;
          }
        }
        if (valid == 0) {
          continue;
        }
        PartitionLabel best = own;
        int bestCount = 0;
        for (const auto& [label, count] : counts) {
          if (count > bestCount) {
            bestCount = count;
            best = label;
          }
        }
        // Only a clear majority moves a cell, so the pass cannot oscillate
        // and does not eat real corners.
        if (best != own && bestCount * 2 > valid) {
          next[cell] = best;
          changed = true;
        }
      }
    }
    labels.swap(next);
    if (!changed) {
      break;
    }
  }
}

} // namespace mqt::scpd::grid
