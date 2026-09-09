/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Partitions.hpp"

#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Watershed.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <map>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

/// How far apart two samples of one border may be and still be one run of it.
/// A sample sits on a cell edge, so neighbours are one cell apart and the
/// diagonal of that is the largest step a run makes.
constexpr double ADJACENT_SAMPLE_SQUARED = 2.25;


/// One edge of a cell, as the two corners it runs between. Corners are whole
/// numbers in cell coordinates, so they key exactly.
struct Corner {
  std::int32_t x = 0;
  std::int32_t y = 0;
  [[nodiscard]] bool operator==(const Corner&) const = default;
};

struct CornerHash {
  [[nodiscard]] std::size_t operator()(const Corner& corner) const noexcept {
    return (static_cast<std::size_t>(static_cast<std::uint32_t>(corner.x))
            << 32U) ^
           static_cast<std::uint32_t>(corner.y);
  }
};

/// The key of an unordered pair of labels.
std::uint32_t borderKey(const PartitionLabel a, const PartitionLabel b) {
  return (static_cast<std::uint32_t>(std::min(a, b)) << 16U) | std::max(a, b);
}

/// The number of coarse cells one axis of the detail grid holds, or nothing
/// when the detail grid is not a whole refinement.
std::optional<std::pair<std::uint32_t, std::uint32_t>>
refinement(const GridMetrics& detail, const GridMetrics& coarse) {
  if (coarse.width == 0 || coarse.height == 0 ||
      detail.width % coarse.width != 0 || detail.height % coarse.height != 0) {
    return std::nullopt;
  }
  return std::pair{detail.width / coarse.width, detail.height / coarse.height};
}

} // namespace

Point PartitionBorder::center() const {
  if (samples.empty()) {
    return {};
  }
  double x = 0.0;
  double y = 0.0;
  for (const auto& sample : samples) {
    x += sample.x();
    y += sample.y();
  }
  return {x / static_cast<double>(samples.size()),
          y / static_cast<double>(samples.size())};
}

Partitions extractPartitions(const BitGrid& blocked,
                             const std::span<const PartitionLabel> labels,
                             const GridMetrics& grid) {
  if (blocked.width() != grid.width || blocked.height() != grid.height ||
      labels.size() != grid.cells()) {
    throw std::invalid_argument(std::format(
        "the mask is {}x{} cells and the labels {}, on a grid of {}x{}",
        blocked.width(), blocked.height(), labels.size(), grid.width,
        grid.height));
  }

  Partitions partitions;
  if (grid.cells() == 0) {
    return partitions;
  }

  /// The label of a cell, with an obstacle and everything off the grid
  /// counting as no label at all.
  const auto labelAt = [&](const std::int64_t x,
                           const std::int64_t y) -> PartitionLabel {
    if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(grid.width) ||
        y >= static_cast<std::int64_t>(grid.height)) {
      return LABEL_NONE;
    }
    const auto index = (static_cast<std::size_t>(y) * grid.width) +
                       static_cast<std::size_t>(x);
    return blocked.test(index) ? LABEL_NONE : labels[index];
  };

  // The outline edges of each label, and the borders between labels. Both
  // are keyed so that the result does not depend on the order the cells were
  // visited in, only on which cells carry which label.
  std::map<PartitionLabel, std::vector<std::pair<Corner, Corner>>> outlineEdges;
  std::map<std::uint32_t, PartitionBorder> borders;
  std::unordered_set<std::uint64_t> latticeKeys;

  const auto addBorder = [&](const PartitionLabel a, const PartitionLabel b,
                             const double x, const double y) {
    if (a == LABEL_NONE || b == LABEL_NONE || a == b) {
      return;
    }
    auto& border = borders[borderKey(a, b)];
    border.first = std::min(a, b);
    border.second = std::max(a, b);
    border.samples.emplace_back(x, y);

    // A sample lies on a half-cell position, so doubling makes the key exact.
    const auto qx = static_cast<std::uint32_t>(std::llround(x * 2.0));
    const auto qy = static_cast<std::uint32_t>(std::llround(y * 2.0));
    if (latticeKeys.insert((static_cast<std::uint64_t>(qx) << 32U) | qy)
            .second) {
      partitions.lattice.emplace_back(x, y);
    }
  };

  for (std::uint32_t y = 0; y < grid.height; ++y) {
    for (std::uint32_t x = 0; x < grid.width; ++x) {
      const auto index = grid.index(x, y);
      const auto label = labels[index];
      if (label == LABEL_NONE || blocked.test(index)) {
        continue;
      }
      const auto ix = static_cast<std::int64_t>(x);
      const auto iy = static_cast<std::int64_t>(y);

      // Only the right and the lower neighbor are consulted, so every border
      // between two cells is counted once rather than from both sides.
      if (const auto right = labelAt(ix + 1, iy);
          right != LABEL_NONE && right != label) {
        addBorder(label, right, static_cast<double>(x + 1),
                  static_cast<double>(y) + 0.5);
      }
      if (const auto below = labelAt(ix, iy + 1);
          below != LABEL_NONE && below != label) {
        addBorder(label, below, static_cast<double>(x) + 0.5,
                  static_cast<double>(y + 1));
      }

      // The outline runs where the neighbor is anything else at all, an
      // obstacle included. Each edge is emitted with the partition on its
      // left, so that the walk below closes the ring in one direction.
      const auto cx = static_cast<std::int32_t>(x);
      const auto cy = static_cast<std::int32_t>(y);
      auto& edges = outlineEdges[label];
      if (labelAt(ix, iy - 1) != label) {
        edges.emplace_back(Corner{cx, cy}, Corner{cx + 1, cy});
      }
      if (labelAt(ix + 1, iy) != label) {
        edges.emplace_back(Corner{cx + 1, cy}, Corner{cx + 1, cy + 1});
      }
      if (labelAt(ix, iy + 1) != label) {
        edges.emplace_back(Corner{cx + 1, cy + 1}, Corner{cx, cy + 1});
      }
      if (labelAt(ix - 1, iy) != label) {
        edges.emplace_back(Corner{cx, cy + 1}, Corner{cx, cy});
      }
    }
  }

  for (auto& [key, border] : borders) {
    partitions.borders.push_back(std::move(border));
  }

  for (auto& [label, edges] : outlineEdges) {
    std::unordered_map<Corner, std::vector<Corner>, CornerHash> outgoing;
    outgoing.reserve(edges.size());
    for (const auto& [from, to] : edges) {
      outgoing[from].push_back(to);
    }

    // Each ring is walked by following one outgoing edge per corner and
    // consuming it, so a corner where four cells meet contributes to two
    // rings rather than joining them into one.
    while (!outgoing.empty()) {
      auto start = outgoing.begin()->first;
      auto current = outgoing.begin()->second.back();
      outgoing.begin()->second.pop_back();
      if (outgoing.begin()->second.empty()) {
        outgoing.erase(outgoing.begin());
      }

      std::vector<Point> ring;
      ring.emplace_back(start.x, start.y);
      ring.emplace_back(current.x, current.y);

      std::size_t guard = 0;
      while (!(current == start)) {
        const auto next = outgoing.find(current);
        if (next == outgoing.end() || next->second.empty()) {
          break;
        }
        const auto step = next->second.back();
        next->second.pop_back();
        if (next->second.empty()) {
          outgoing.erase(next);
        }
        ring.emplace_back(step.x, step.y);
        current = step;
        if (++guard > edges.size() + 4) {
          break;
        }
      }

      // Three corners and the repeat of the first are the least a closed
      // outline can have.
      if (ring.size() >= 4) {
        partitions.outlines.push_back(
            {.label = label, .ring = std::move(ring)});
      }
    }
  }

  std::ranges::sort(partitions.lattice, [](const Point& a, const Point& b) {
    return std::pair{a.y(), a.x()} < std::pair{b.y(), b.x()};
  });
  return partitions;
}

std::vector<std::size_t> freeCellSeeds(const BitGrid& blocked,
                                       const GridMetrics& detail,
                                       const GridMetrics& coarse) {
  const auto factors = refinement(detail, coarse);
  if (!factors.has_value()) {
    throw std::invalid_argument(
        std::format("a detail grid of {}x{} cells is no whole refinement of a "
                    "coarse grid of {}x{}",
                    detail.width, detail.height, coarse.width, coarse.height));
  }
  if (blocked.width() != detail.width || blocked.height() != detail.height) {
    throw std::invalid_argument(std::format(
        "the mask is {}x{} cells and the detail grid {}x{}", blocked.width(),
        blocked.height(), detail.width, detail.height));
  }
  const auto [cellWidth, cellHeight] = *factors;

  std::vector<std::size_t> seeds;
  for (std::uint32_t cy = 0; cy < coarse.height; ++cy) {
    for (std::uint32_t cx = 0; cx < coarse.width; ++cx) {
      const auto x0 = cx * cellWidth;
      const auto y0 = cy * cellHeight;
      bool clear = true;
      for (std::uint32_t y = y0; y < y0 + cellHeight && clear; ++y) {
        for (std::uint32_t x = x0; x < x0 + cellWidth; ++x) {
          if (blocked.test(detail.index(x, y))) {
            clear = false;
            break;
          }
        }
      }
      if (clear) {
        seeds.push_back(
            detail.index(x0 + (cellWidth / 2), y0 + (cellHeight / 2)));
      }
    }
  }
  return seeds;
}

std::vector<CellCapacity> cellCapacities(const BitGrid& blocked,
                                         const GridMetrics& detail,
                                         const GridMetrics& coarse,
                                         const double wirePitch) {
  const auto factors = refinement(detail, coarse);
  if (!factors.has_value()) {
    throw std::invalid_argument(
        std::format("a detail grid of {}x{} cells is no whole refinement of a "
                    "coarse grid of {}x{}",
                    detail.width, detail.height, coarse.width, coarse.height));
  }
  if (wirePitch <= 0.0) {
    throw std::invalid_argument(
        std::format("a wire pitch of {} is not positive", wirePitch));
  }
  const auto [cellWidth, cellHeight] = *factors;

  // How many wires a fully free edge of a capacity cell carries. The shorter
  // side decides, so a cell that is wide and flat is budgeted by its height.
  const auto full = std::min(coarse.cellWidth, coarse.cellHeight) / wirePitch;

  const auto budget = [&](const std::uint32_t free, const std::uint32_t of) {
    const auto fraction = static_cast<double>(free) / static_cast<double>(of);
    return static_cast<std::uint16_t>(fraction * full);
  };

  std::vector<CellCapacity> capacities(coarse.cells());
  for (std::uint32_t cy = 0; cy < coarse.height; ++cy) {
    for (std::uint32_t cx = 0; cx < coarse.width; ++cx) {
      const auto x0 = cx * cellWidth;
      const auto y0 = cy * cellHeight;
      const auto x1 = x0 + cellWidth - 1;
      const auto y1 = y0 + cellHeight - 1;

      std::uint32_t south = 0;
      std::uint32_t north = 0;
      for (std::uint32_t x = x0; x <= x1; ++x) {
        south += blocked.test(detail.index(x, y0)) ? 0U : 1U;
        north += blocked.test(detail.index(x, y1)) ? 0U : 1U;
      }
      std::uint32_t west = 0;
      std::uint32_t east = 0;
      for (std::uint32_t y = y0; y <= y1; ++y) {
        west += blocked.test(detail.index(x0, y)) ? 0U : 1U;
        east += blocked.test(detail.index(x1, y)) ? 0U : 1U;
      }

      capacities[coarse.index(cx, cy)] = {.south = budget(south, cellWidth),
                                          .north = budget(north, cellWidth),
                                          .west = budget(west, cellHeight),
                                          .east = budget(east, cellHeight)};
    }
  }
  return capacities;
}

std::vector<PartitionLabel>
rasterizePartitions(const std::span<const PartitionOutline> outlines,
                    const GridMetrics& grid) {
  std::vector<PartitionLabel> labels(grid.cells(), LABEL_NONE);
  if (grid.width == 0 || grid.height == 0) {
    return labels;
  }

  // The rings of one label are filled together, so a hole ring cancels the
  // outer ring that encloses it. Grouping them keeps that per label rather
  // than across the whole grid.
  std::map<PartitionLabel, std::vector<const PartitionOutline*>> byLabel;
  for (const auto& outline : outlines) {
    if (outline.label != LABEL_NONE) {
      byLabel[outline.label].push_back(&outline);
    }
  }

  std::vector<double> crossings;
  for (const auto& [label, rings] : byLabel) {
    for (std::uint32_t y = 0; y < grid.height; ++y) {
      // A ring runs along cell corners, so the row of cells at index y is
      // the strip between the corner lines y and y + 1, tested at its middle.
      const auto scan = static_cast<double>(y) + 0.5;
      crossings.clear();
      for (const auto* ring : rings) {
        const auto& points = ring->ring;
        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
          const auto& from = points[i];
          const auto& to = points[i + 1];
          // A ring is drawn on the grid, so an edge is either horizontal —
          // and never crossed — or vertical over exactly one cell.
          if ((from.y() > scan) == (to.y() > scan)) {
            continue;
          }
          const auto t = (scan - from.y()) / (to.y() - from.y());
          crossings.push_back(from.x() + (t * (to.x() - from.x())));
        }
      }
      if (crossings.empty()) {
        continue;
      }
      std::ranges::sort(crossings);
      for (std::size_t i = 0; i + 1 < crossings.size(); i += 2) {
        const auto first =
            static_cast<std::int64_t>(std::ceil(crossings[i] - 0.5));
        const auto last =
            static_cast<std::int64_t>(std::floor(crossings[i + 1] - 0.5));
        for (auto x = std::max<std::int64_t>(first, 0);
             x <= std::min<std::int64_t>(
                      last, static_cast<std::int64_t>(grid.width) - 1);
             ++x) {
          labels[grid.index(static_cast<std::uint32_t>(x), y)] = label;
        }
      }
    }
  }
  return labels;
}

std::vector<Point> borderSlots(const PartitionBorder& border, const GridMetrics& grid,
                               const double spacing) {
  if (spacing <= 0.0) {
    throw std::invalid_argument(std::format("a wire spacing of {} is no distance", spacing));
  }
  if (border.samples.empty()) {
    return {};
  }

  // The samples come in the order the grid was scanned, which is not the order
  // the border runs in. Walking it is what makes the spacing a distance along
  // the border rather than a distance to everything taken so far: a border
  // that bends back towards itself would otherwise lose the slots on its
  // return leg, and those are the ones a wire coming from that side needs.
  std::vector<bool> walked(border.samples.size(), false);
  const auto step = [&](const std::size_t from) {
    auto nearest = border.samples.size();
    auto best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < border.samples.size(); ++i) {
      if (walked[i]) {
        continue;
      }
      const auto dx = border.samples[i].x() - border.samples[from].x();
      const auto dy = border.samples[i].y() - border.samples[from].y();
      if (const auto squared = (dx * dx) + (dy * dy); squared < best) {
        best = squared;
        nearest = i;
      }
    }
    // Two samples of one border are cell edges that touch, so a step is about
    // one cell. A longer one means the border is in pieces, and the walk
    // starts again on the next of them.
    return std::pair{nearest, best <= ADJACENT_SAMPLE_SQUARED};
  };

  const auto layout = [&](const std::size_t index) {
    return grid.toLayout(border.samples[index].x(), border.samples[index].y());
  };

  std::vector<Point> slots;
  auto current = std::size_t{0};
  walked[0] = true;
  slots.push_back(border.samples[0]);
  auto last = layout(0);
  while (true) {
    const auto [next, adjacent] = step(current);
    if (next == border.samples.size()) {
      break;
    }
    walked[next] = true;
    const auto here = layout(next);
    // The first sample of a new piece is a slot of its own; along one piece a
    // sample becomes a slot once it is one spacing past the last.
    if (!adjacent || std::hypot(here.x() - last.x(), here.y() - last.y()) >= spacing) {
      slots.push_back(border.samples[next]);
      last = here;
    }
    current = next;
  }
  return slots;
}

} // namespace mqt::scpd::grid
