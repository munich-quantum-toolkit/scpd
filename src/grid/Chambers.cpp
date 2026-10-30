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
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <vector>

namespace mqt::scpd::grid {
namespace {

/// The index of the first diagonal step in STEP_DX and STEP_DY.
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
  constexpr double epsilon = 1.0e-6;
  if (std::abs(((py - y0) * (x1 - x0)) - ((px - x0) * (y1 - y0))) > epsilon) {
    return false;
  }
  return px >= std::min(x0, x1) - epsilon && px <= std::max(x0, x1) + epsilon &&
         py >= std::min(y0, y1) - epsilon && py <= std::max(y0, y1) + epsilon;
}

} // namespace

void bottleneckMovesOf(const Bottleneck& gate, const GridMetrics& grid,
                       std::vector<BlockedMove>& blocked) {
  const auto width = static_cast<std::int64_t>(grid.width);
  const auto height = static_cast<std::int64_t>(grid.height);
  const std::size_t firstRow = gate.first / grid.width;
  const std::size_t secondRow = gate.second / grid.width;
  const auto x0 = static_cast<double>(gate.first % grid.width) + 0.5;
  const auto y0 = static_cast<double>(firstRow) + 0.5;
  const auto x1 = static_cast<double>(gate.second % grid.width) + 0.5;
  const auto y1 = static_cast<double>(secondRow) + 0.5;

  const auto minX = std::max<std::int64_t>(
      0, static_cast<std::int64_t>(std::floor(std::min(x0, x1))) - 1);
  const auto maxX = std::min<std::int64_t>(
      width - 1, static_cast<std::int64_t>(std::ceil(std::max(x0, x1))) + 1);
  const auto minY = std::max<std::int64_t>(
      0, static_cast<std::int64_t>(std::floor(std::min(y0, y1))) - 1);
  const auto maxY = std::min<std::int64_t>(
      height - 1, static_cast<std::int64_t>(std::ceil(std::max(y0, y1))) + 1);

  blocked.clear();
  for (std::int64_t y = minY; y <= maxY; ++y) {
    // Only a cell near the line has a step that crosses it or lies on it: a
    // step reaches one cell up or down, so the cells of a row that matter
    // are those within two of where the line runs between a row and a half
    // above and below. A gate across many rows looks at a few cells of each,
    // not at its whole box, in the order the box would give them.
    auto fromX = minX;
    auto toX = maxX;
    if (y0 != y1) {
      const auto rowY = static_cast<double>(y) + 0.5;
      const auto at = [&](const double ty) {
        const auto t = std::clamp((ty - y0) / (y1 - y0), 0.0, 1.0);
        return x0 + (t * (x1 - x0));
      };
      const auto a = at(rowY - 1.5);
      const auto b = at(rowY + 1.5);
      fromX = std::max(
          minX, static_cast<std::int64_t>(std::floor(std::min(a, b))) - 2);
      toX = std::min(
          maxX, static_cast<std::int64_t>(std::ceil(std::max(a, b))) + 2);
    }
    for (std::int64_t x = fromX; x <= toX; ++x) {
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
                                     static_cast<double>(ny) + 0.5, x0, y0, x1,
                                     y1)) {
            continue;
          }
        }
        blocked.push_back({.cell = cell, .direction = step});
        blocked.push_back({.cell = neighbor, .direction = REVERSE_STEP[step]});
      }
    }
  }
}

std::vector<std::vector<BlockedMove>>
bottleneckMoves(const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid) {
  std::vector<std::vector<BlockedMove>> moves(bottlenecks.size());
  for (std::size_t index = 0; index < bottlenecks.size(); ++index) {
    bottleneckMovesOf(bottlenecks[index], grid, moves[index]);
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

namespace {

/// The labelling of `chambersOf`, with the moves of every gate and the steps
/// they refuse out of each cell, as a bit per direction and as a bit per
/// cell, already there.
void labelChambers(const BitGrid& blocked,
                   const std::vector<const std::vector<BlockedMove>*>& moves,
                   const std::vector<std::uint8_t>& refused,
                   const std::vector<std::uint64_t>& refusedBits,
                   const GridMetrics& grid, Chambers& chambers) {
  const auto width = static_cast<std::int64_t>(grid.width);
  const auto height = static_cast<std::int64_t>(grid.height);
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
  // The first cell in [from, to) whose bit `wordAt` sets, or `to`: the
  // rows are scanned 64 cells at a time.
  const auto& blockedWords = blocked.words();
  const auto firstSet = [](const auto& wordAt, std::size_t from,
                           const std::size_t to) {
    while (from < to) {
      const auto word = from >> 6U;
      const auto bits = wordAt(word) >> (from & 63U);
      if (bits != 0U) {
        return std::min<std::size_t>(
            from + static_cast<std::size_t>(std::countr_zero(bits)), to);
      }
      from = (word + 1) << 6U;
    }
    return to;
  };
  const auto freeWord = [&](const std::size_t word) {
    return ~blockedWords[word];
  };
  const auto stopWord = [&](const std::size_t word) {
    return blockedWords[word] | refusedBits[word];
  };
  for (std::int64_t y = 0; y < height; ++y) {
    rowStart[y] = runs.size();
    const auto rowFirst = static_cast<std::size_t>(y * width);
    const auto rowEnd = rowFirst + static_cast<std::size_t>(width);
    for (auto cell = firstSet(freeWord, rowFirst, rowEnd); cell < rowEnd;
         cell = firstSet(freeWord, cell, rowEnd)) {
      const auto x = static_cast<std::int64_t>(cell - rowFirst);
      if (refused[cell] != 0U) {
        if (!onALine(x, y)) {
          runs.push_back({.from = x, .to = x, .single = true});
        }
        ++cell;
        continue;
      }
      // A plain run: up to the next cell that is a wall or has a refused
      // step.
      const auto end = firstSet(stopWord, cell, rowEnd);
      runs.push_back({.from = x,
                      .to = static_cast<std::int64_t>(end - rowFirst) - 1,
                      .single = false});
      cell = end;
    }
    rowStart[y + 1] = runs.size();
    for (auto index = rowStart[y]; index < rowStart[y + 1]; ++index) {
      parent.push_back(static_cast<std::uint32_t>(index));
    }
    // The runs of a row are sorted and apart, so the first run above that
    // may touch a run of this row only moves on along the row; the run
    // left of a cell is the one before it, when it ends there.
    auto upFirst = y > 0 ? rowStart[y - 1] : rowStart[y];
    const auto leftOf = [&](const std::size_t index,
                            const std::int64_t x) -> std::optional<std::uint32_t> {
      if (index > rowStart[y] && runs[index - 1].to == x - 1) {
        return static_cast<std::uint32_t>(index - 1);
      }
      return std::nullopt;
    };
    const auto aboveAt = [&](const std::int64_t x) -> std::optional<std::uint32_t> {
      for (auto other = upFirst; other < rowStart[y] && runs[other].from <= x;
           ++other) {
        if (runs[other].to >= x) {
          return static_cast<std::uint32_t>(other);
        }
      }
      return std::nullopt;
    };
    for (auto index = rowStart[y]; index < rowStart[y + 1]; ++index) {
      const auto run = runs[index];
      const auto me = static_cast<std::uint32_t>(index);
      if (y > 0) {
        while (upFirst < rowStart[y] && runs[upFirst].to < run.from - 1) {
          ++upFirst;
        }
      }
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
          if (const auto other =
                  ny == y ? leftOf(index, run.from) : aboveAt(nx)) {
            join(me, *other);
          }
        }
        continue;
      }
      // The cell left of the run is no cell of a plain run: a single cell,
      // a line cell or a wall.
      if (const auto other = leftOf(index, run.from)) {
        join(me, *other);
      }
      if (y == 0) {
        continue;
      }
      const auto above = y - 1;
      for (auto other = upFirst;
           other < rowStart[y] && runs[other].from <= run.to + 1; ++other) {
        const auto& up = runs[other];
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
    for (const auto& move : *moves[gate]) {
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
      // A gate's moves see the same two or three chambers over and over.
      if (const auto chamber = chambers.of[at];
          chamber != NO_CHAMBER && std::ranges::find(beside, chamber) == beside.end()) {
        beside.push_back(chamber);
      }
    }
    std::ranges::sort(beside);
    const auto repeated = std::ranges::unique(beside);
    beside.erase(repeated.begin(), repeated.end());
  }
}

void checkFits(const BitGrid& blocked, const GridMetrics& grid) {
  if (blocked.width() != grid.width || blocked.height() != grid.height) {
    throw std::invalid_argument(std::format(
        "the mask is {}x{} cells and the grid {}x{}", blocked.width(),
        blocked.height(), grid.width, grid.height));
  }
}

} // namespace

void chambersOf(const BitGrid& blocked,
                const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid, Chambers& chambers) {
  checkFits(blocked, grid);
  const auto moves = bottleneckMoves(bottlenecks, grid);

  // The directions a gate refuses out of each cell, one bit each. The bits
  // are kept per thread from call to call and cleared where they were set:
  // a grid of millions of cells is labelled many times, and fresh memory
  // costs more than the labelling itself.
  // The same cells as a bit each, for the scan of the rows below.
  thread_local std::vector<std::uint8_t> refused;
  thread_local std::vector<std::uint64_t> refusedBits;
  if (refused.size() != grid.cells()) {
    refused.assign(grid.cells(), 0);
    refusedBits.assign((grid.cells() + 63) / 64, 0);
  }
  for (const auto& gate : moves) {
    for (const auto& move : gate) {
      refused[move.cell] |= static_cast<std::uint8_t>(1U << move.direction);
      refusedBits[move.cell >> 6U] |= std::uint64_t{1} << (move.cell & 63U);
    }
  }
  std::vector<const std::vector<BlockedMove>*> of;
  of.reserve(moves.size());
  for (const auto& gate : moves) {
    of.push_back(&gate);
  }
  labelChambers(blocked, of, refused, refusedBits, grid, chambers);
  for (const auto& gate : moves) {
    for (const auto& move : gate) {
      refused[move.cell] = 0;
      refusedBits[move.cell >> 6U] = 0;
    }
  }
}

void chamberCacheUpdate(const std::vector<Bottleneck>& bottlenecks,
                        const GridMetrics& grid, ChamberCache& cache,
                        std::vector<const std::vector<BlockedMove>*>& moves) {
  const auto cells = grid.cells();
  if (cache.width != grid.width || cache.height != grid.height ||
      cache.refused.size() != cells) {
    cache.entries.clear();
    cache.applied.clear();
    cache.refused.assign(cells, 0);
    cache.refusedBits.assign((cells + 63) / 64, 0);
    cache.width = grid.width;
    cache.height = grid.height;
  }
  const auto clock = ++cache.clock;
  const auto keyOf = [&](const Bottleneck& gate) {
    return (static_cast<std::uint64_t>(gate.first) * cells) + gate.second;
  };
  const auto width = static_cast<std::int64_t>(grid.width);

  // The moves of every gate, from the cache or traced now. The entries of
  // an unordered map stay where they are as it grows.
  moves.clear();
  moves.reserve(bottlenecks.size());
  std::vector<std::uint64_t> keys;
  keys.reserve(bottlenecks.size());
  for (const auto& gate : bottlenecks) {
    const auto key = keyOf(gate);
    auto [found, fresh] = cache.entries.try_emplace(key);
    auto& entry = found->second;
    if (fresh) {
      bottleneckMovesOf(gate, grid, entry.moves);
      entry.minX = width;
      entry.minY = static_cast<std::int64_t>(grid.height);
      for (const auto& move : entry.moves) {
        const auto x = static_cast<std::int64_t>(move.cell) % width;
        const auto y = static_cast<std::int64_t>(move.cell) / width;
        entry.minX = std::min(entry.minX, x);
        entry.minY = std::min(entry.minY, y);
        entry.maxX = std::max(entry.maxX, x);
        entry.maxY = std::max(entry.maxY, y);
      }
      ++cache.computed;
    } else {
      ++cache.reused;
    }
    entry.used = clock;
    moves.push_back(&entry.moves);
    keys.push_back(key);
  }
  std::ranges::sort(keys);
  keys.erase(std::ranges::unique(keys).begin(), keys.end());

  // The refused steps of the last call, changed to those of this one: the
  // cells of a gate that went are cleared, the gates that stay and reach
  // into one of them are laid on again, and the gates that came are laid on.
  std::vector<std::uint64_t> gone;
  std::vector<std::uint64_t> came;
  std::ranges::set_difference(cache.applied, keys, std::back_inserter(gone));
  std::ranges::set_difference(keys, cache.applied, std::back_inserter(came));
  const auto layOn = [&](const ChamberCache::Entry& entry) {
    for (const auto& move : entry.moves) {
      cache.refused[move.cell] |=
          static_cast<std::uint8_t>(1U << move.direction);
      cache.refusedBits[move.cell >> 6U] |= std::uint64_t{1}
                                            << (move.cell & 63U);
    }
  };
  if (!gone.empty()) {
    std::vector<const ChamberCache::Entry*> cleared;
    for (const auto key : gone) {
      const auto& entry = cache.entries.at(key);
      for (const auto& move : entry.moves) {
        cache.refused[move.cell] = 0;
        cache.refusedBits[move.cell >> 6U] &=
            ~(std::uint64_t{1} << (move.cell & 63U));
      }
      cleared.push_back(&entry);
    }
    for (const auto key : keys) {
      if (std::ranges::binary_search(came, key)) {
        continue;
      }
      const auto& entry = cache.entries.at(key);
      const auto touches = std::ranges::any_of(
          cleared, [&](const ChamberCache::Entry* other) {
            return entry.minX <= other->maxX && other->minX <= entry.maxX &&
                   entry.minY <= other->maxY && other->minY <= entry.maxY;
          });
      if (touches) {
        layOn(entry);
      }
    }
  }
  for (const auto key : came) {
    layOn(cache.entries.at(key));
  }
  cache.added += came.size();
  cache.removed += gone.size();
  cache.applied = std::move(keys);

  // The gates no call has used lately are forgotten.
  std::erase_if(cache.entries, [&](const auto& item) {
    return clock - item.second.used >= cache.keep;
  });
}

void chambersOfCached(const BitGrid& blocked,
                      const std::vector<Bottleneck>& bottlenecks,
                      const GridMetrics& grid, Chambers& chambers,
                      ChamberCache& cache) {
  checkFits(blocked, grid);
  thread_local std::vector<const std::vector<BlockedMove>*> moves;
  chamberCacheUpdate(bottlenecks, grid, cache, moves);
  labelChambers(blocked, moves, cache.refused, cache.refusedBits, grid,
                chambers);
}

} // namespace mqt::scpd::grid
