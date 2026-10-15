/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Chambers.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::size_t FIRST_DIAGONAL_STEP = 4;

/// The direction that undoes each of the eight steps.
constexpr std::array<std::size_t, 8> REVERSE_STEP = {1, 0, 3, 2, 7, 6, 5, 4};

/// Whether two segments cross, strictly.
bool segmentsCrossStrictly(const double ax, const double ay, const double bx,
                           const double by, const double cx, const double cy,
                           const double dx, const double dy) {
  const auto side = [](const double px, const double py, const double qx,
                       const double qy, const double rx, const double ry) {
    return ((qx - px) * (ry - py)) - ((qy - py) * (rx - px));
  };
  const auto first = side(ax, ay, bx, by, cx, cy);
  const auto second = side(ax, ay, bx, by, dx, dy);
  const auto third = side(cx, cy, dx, dy, ax, ay);
  const auto fourth = side(cx, cy, dx, dy, bx, by);
  return ((first > 0 && second < 0) || (first < 0 && second > 0)) &&
         ((third > 0 && fourth < 0) || (third < 0 && fourth > 0));
}

/// Whether a point lies on a segment.
bool pointOnSegment(const double px, const double py, const double x0,
                    const double y0, const double x1, const double y1) {
  constexpr double EPSILON = 1.0e-6;
  if (std::abs(((py - y0) * (x1 - x0)) - ((px - x0) * (y1 - y0))) > EPSILON) {
    return false;
  }
  return px >= std::min(x0, x1) - EPSILON && px <= std::max(x0, x1) + EPSILON &&
         py >= std::min(y0, y1) - EPSILON && py <= std::max(y0, y1) + EPSILON;
}

} // namespace

std::vector<std::vector<BlockedMove>>
bottleneckMoves(const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid) {
  std::vector<std::vector<BlockedMove>> moves(bottlenecks.size());
  const auto width = static_cast<std::int64_t>(grid.width);
  const auto height = static_cast<std::int64_t>(grid.height);

  for (std::size_t index = 0; index < bottlenecks.size(); ++index) {
    const auto& gate = bottlenecks[index];
    const auto x0 = static_cast<double>(gate.first % grid.width) + 0.5;
    const auto y0 = static_cast<double>(gate.first / grid.width) + 0.5;
    const auto x1 = static_cast<double>(gate.second % grid.width) + 0.5;
    const auto y1 = static_cast<double>(gate.second / grid.width) + 0.5;

    const auto minX = std::max<std::int64_t>(
        0, static_cast<std::int64_t>(std::floor(std::min(x0, x1))) - 1);
    const auto maxX = std::min<std::int64_t>(
        width - 1, static_cast<std::int64_t>(std::ceil(std::max(x0, x1))) + 1);
    const auto minY = std::max<std::int64_t>(
        0, static_cast<std::int64_t>(std::floor(std::min(y0, y1))) - 1);
    const auto maxY = std::min<std::int64_t>(
        height - 1, static_cast<std::int64_t>(std::ceil(std::max(y0, y1))) + 1);

    // Only a cell within two cells of the line can lie on it or step
    // across it: per row, the cells beside the part of the line within two
    // rows of it.
    constexpr double NEAR = 2.0;
    auto& blocked = moves[index];
    for (std::int64_t y = minY; y <= maxY; ++y) {
      const auto ay = static_cast<double>(y) + 0.5;
      double from = 0.0;
      double to = 1.0;
      if (y1 != y0) {
        const auto a = (ay - NEAR - y0) / (y1 - y0);
        const auto b = (ay + NEAR - y0) / (y1 - y0);
        from = std::max(0.0, std::min(a, b));
        to = std::min(1.0, std::max(a, b));
      } else if (std::abs(ay - y0) > NEAR) {
        continue;
      }
      if (from > to) {
        continue;
      }
      const auto xa = x0 + (from * (x1 - x0));
      const auto xb = x0 + (to * (x1 - x0));
      const auto firstX = std::max(minX, static_cast<std::int64_t>(std::floor(
                                             std::min(xa, xb) - NEAR - 1.0)));
      const auto lastX = std::min(
          maxX, static_cast<std::int64_t>(std::ceil(std::max(xa, xb) + NEAR)));
      for (std::int64_t x = firstX; x <= lastX; ++x) {
        const auto cell = grid.index(static_cast<std::uint32_t>(x),
                                     static_cast<std::uint32_t>(y));
        const auto ax = static_cast<double>(x) + 0.5;
        const auto ay = static_cast<double>(y) + 0.5;
        const auto onLine = pointOnSegment(ax, ay, x0, y0, x1, y1) &&
                            cell != gate.first && cell != gate.second;

        for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
          const auto nx = x + STEP_DX[step];
          const auto ny = y + STEP_DY[step];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const auto neighbor = grid.index(static_cast<std::uint32_t>(nx),
                                           static_cast<std::uint32_t>(ny));
          if (!onLine) {
            // Each edge is decided once, from its lower cell.
            if (cell > neighbor) {
              continue;
            }
            if (!segmentsCrossStrictly(ax, ay, static_cast<double>(nx) + 0.5,
                                       static_cast<double>(ny) + 0.5, x0, y0,
                                       x1, y1)) {
              continue;
            }
          }
          blocked.push_back({.cell = cell, .direction = step});
          blocked.push_back(
              {.cell = neighbor, .direction = REVERSE_STEP[step]});
        }
      }
    }
  }
  return moves;
}

Chambers chambersOf(const BitGrid& blocked,
                    const std::vector<Bottleneck>& bottlenecks,
                    const GridMetrics& grid) {
  Chambers chambers;
  chambersOf(blocked, bottlenecks, grid, chambers);
  return chambers;
}

void chambersOf(const BitGrid& blocked,
                const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid, Chambers& chambers) {
  if (blocked.width() != grid.width || blocked.height() != grid.height) {
    throw std::invalid_argument(std::format(
        "the mask is {}x{} cells and the grid {}x{}", blocked.width(),
        blocked.height(), grid.width, grid.height));
  }
  const auto width = static_cast<std::int64_t>(grid.width);
  const auto height = static_cast<std::int64_t>(grid.height);
  const auto moves = bottleneckMoves(bottlenecks, grid);

  // The directions a gate refuses out of each cell, one bit each. The bits
  // are kept per thread from call to call and cleared where they were set:
  // a grid of millions of cells is labelled many times, and fresh memory
  // costs more than the labelling itself.
  thread_local std::vector<std::uint8_t> refused;
  if (refused.size() != grid.cells()) {
    refused.assign(grid.cells(), 0);
  }
  for (const auto& gate : moves) {
    for (const auto& move : gate) {
      refused[move.cell] |= static_cast<std::uint8_t>(1U << move.direction);
    }
  }
  // A cell every one of whose steps is refused lies on a line.
  const auto onALine = [&](const std::int64_t x, const std::int64_t y) {
    const auto cell = static_cast<std::size_t>((y * width) + x);
    for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
      const auto nx = x + STEP_DX[step];
      const auto ny = y + STEP_DY[step];
      if (nx >= 0 && ny >= 0 && nx < width && ny < height &&
          (refused[cell] & (1U << step)) == 0U) {
        return false;
      }
    }
    return true;
  };

  // The chambers are the connected pieces of the free space, labelled by
  // runs: a row's free cells that no gate touches are one run, since every
  // step between two of them is allowed. A cell with a refused step is a
  // run of its own and is joined to the cells before it step by step, as
  // a flood would. A run is joined to the run before it in its row and to
  // every run of the row above it overlaps: straight up from any of its
  // cells, and diagonally from its two ends past the corner rule; a
  // diagonal step from inside a run reaches nothing a straight one does
  // not. The union-find over the runs keeps the lowest run as the root,
  // so numbering the roots in row order numbers the pieces in the order of
  // their lowest cells, which is the order a flood from every unlabelled
  // cell in turn would give.
  struct Run {
    std::int64_t from = 0;
    std::int64_t to = 0;
    bool single = false;
  };
  thread_local std::vector<Run> runs;
  thread_local std::vector<std::uint32_t> parent;
  thread_local std::vector<std::size_t> rowStart;
  runs.clear();
  parent.clear();
  rowStart.assign(static_cast<std::size_t>(height) + 1, 0);
  const auto find = [](std::uint32_t run) {
    while (parent[run] != run) {
      parent[run] = parent[parent[run]];
      run = parent[run];
    }
    return run;
  };
  const auto join = [&find](const std::uint32_t a, const std::uint32_t b) {
    const auto rootA = find(a);
    const auto rootB = find(b);
    if (rootA < rootB) {
      parent[rootB] = rootA;
    } else if (rootB < rootA) {
      parent[rootA] = rootB;
    }
  };
  const auto free = [&](const std::int64_t x, const std::int64_t y) {
    return !blocked.test(static_cast<std::size_t>((y * width) + x));
  };
  // The run of row `y` that holds column `x`, if any.
  const auto runAt = [&](const std::int64_t x,
                         const std::int64_t y) -> std::optional<std::uint32_t> {
    const auto first = runs.begin() + static_cast<std::ptrdiff_t>(rowStart[y]);
    const auto last =
        runs.begin() + static_cast<std::ptrdiff_t>(rowStart[y + 1]);
    const auto found = std::upper_bound(
        first, last, x,
        [](const std::int64_t v, const Run& r) { return v < r.from; });
    if (found == first || std::prev(found)->to < x) {
      return std::nullopt;
    }
    return static_cast<std::uint32_t>(std::prev(found) - runs.begin());
  };
  for (std::int64_t y = 0; y < height; ++y) {
    rowStart[y] = runs.size();
    for (std::int64_t x = 0; x < width;) {
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (blocked.test(cell)) {
        ++x;
        continue;
      }
      if (refused[cell] != 0U) {
        if (!onALine(x, y)) {
          runs.push_back({.from = x, .to = x, .single = true});
        }
        ++x;
        continue;
      }
      auto to = x;
      while (to + 1 < width && free(to + 1, y) &&
             refused[static_cast<std::size_t>((y * width) + to + 1)] == 0U) {
        ++to;
      }
      runs.push_back({.from = x, .to = to, .single = false});
      x = to + 1;
    }
    rowStart[y + 1] = runs.size();
    for (auto index = rowStart[y]; index < rowStart[y + 1]; ++index) {
      parent.push_back(static_cast<std::uint32_t>(index));
    }
    for (auto index = rowStart[y]; index < rowStart[y + 1]; ++index) {
      const auto run = runs[index];
      const auto me = static_cast<std::uint32_t>(index);
      if (run.single) {
        // Step by step, as a flood: left, above, above left, above right.
        const auto cell = static_cast<std::size_t>((y * width) + run.from);
        for (const auto step : std::array<std::size_t, 4>{3, 1, 7, 6}) {
          const auto nx = run.from + STEP_DX[step];
          const auto ny = y + STEP_DY[step];
          if (nx < 0 || ny < 0 || nx >= width ||
              (refused[cell] & (1U << step)) != 0U) {
            continue;
          }
          if (step >= FIRST_DIAGONAL_STEP && !free(nx, y) &&
              !free(run.from, ny)) {
            continue;
          }
          if (const auto other = runAt(nx, ny)) {
            join(me, *other);
          }
        }
        continue;
      }
      // The cell left of the run is no cell of a plain run: a single cell,
      // a line cell or a wall.
      if (run.from > 0) {
        if (const auto other = runAt(run.from - 1, y)) {
          join(me, *other);
        }
      }
      if (y == 0) {
        continue;
      }
      const auto above = y - 1;
      for (auto other = rowStart[above]; other < rowStart[y]; ++other) {
        const auto& up = runs[other];
        if (up.to < run.from - 1 || up.from > run.to + 1) {
          continue;
        }
        const bool straight = up.to >= run.from && up.from <= run.to;
        const bool leftCorner =
            up.to == run.from - 1 &&
            (free(run.from - 1, y) || free(run.from, above));
        const bool rightCorner = up.from == run.to + 1 &&
                                 (free(run.to + 1, y) || free(run.to, above));
        if (straight || leftCorner || rightCorner) {
          join(me, static_cast<std::uint32_t>(other));
        }
      }
    }
  }
  chambers.count = 0;
  chambers.of.resize(grid.cells());
  chambers.beside.clear();
  thread_local std::vector<std::uint32_t> labelOf;
  labelOf.assign(runs.size(), NO_CHAMBER);
  for (std::size_t index = 0; index < runs.size(); ++index) {
    const auto root = find(static_cast<std::uint32_t>(index));
    labelOf[index] = root == index ? chambers.count++ : labelOf[root];
  }
  for (std::int64_t y = 0; y < height; ++y) {
    auto* row = chambers.of.data() + (y * width);
    std::int64_t x = 0;
    for (auto index = rowStart[y]; index < rowStart[y + 1]; ++index) {
      const auto& run = runs[index];
      std::fill(row + x, row + run.from, NO_CHAMBER);
      std::fill(row + run.from, row + run.to + 1, labelOf[index]);
      x = run.to + 1;
    }
    std::fill(row + x, row + width, NO_CHAMBER);
  }

  // The chambers beside each gate. A cell beside a gate's line that lies on
  // the line of another gate is in no chamber; the chamber behind that line
  // is the one the gate opens onto, a cell or two further away from it
  // (user, 2026-10-13). Without the look through, two gates whose lines
  // touch each see one side only: the one whose sides are then one chamber
  // is no edge of the graph, and its line still parts the chambers beyond.
  constexpr int LOOK_THROUGH = 3;
  chambers.beside.resize(moves.size());
  for (std::size_t gate = 0; gate < moves.size(); ++gate) {
    auto& beside = chambers.beside[gate];
    for (const auto& move : moves[gate]) {
      auto x = static_cast<std::int64_t>(move.cell) % width;
      auto y = static_cast<std::int64_t>(move.cell) / width;
      auto at = move.cell;
      for (int k = 0; k < LOOK_THROUGH && chambers.of[at] == NO_CHAMBER &&
                      !blocked.test(at);
           ++k) {
        x -= STEP_DX[move.direction];
        y -= STEP_DY[move.direction];
        if (x < 0 || y < 0 || x >= width || y >= height) {
          break;
        }
        at = static_cast<std::size_t>((y * width) + x);
      }
      if (chambers.of[at] != NO_CHAMBER) {
        beside.push_back(chambers.of[at]);
      }
    }
    std::ranges::sort(beside);
    const auto repeated = std::ranges::unique(beside);
    beside.erase(repeated.begin(), repeated.end());
  }
  for (const auto& gate : moves) {
    for (const auto& move : gate) {
      refused[move.cell] = 0;
    }
  }
}

} // namespace mqt::scpd::grid
