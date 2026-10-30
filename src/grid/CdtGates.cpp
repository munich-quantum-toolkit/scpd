/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/CdtGates.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"

#include <CDT.h>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <numeric>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

namespace {

/// A well-mixed 64-bit hash of a 64-bit value (SplitMix64).
std::uint64_t splitMix(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

/// Gates in an order that depends on the gates alone: by their two wall
/// cells, the lower first, then by their saddle.
void sortCanonically(std::vector<Bottleneck>& gates) {
  std::ranges::sort(gates, [](const Bottleneck& a, const Bottleneck& b) {
    const auto ka = std::tuple{std::min(a.first, a.second),
                               std::max(a.first, a.second), a.saddle};
    const auto kb = std::tuple{std::min(b.first, b.second),
                               std::max(b.first, b.second), b.saddle};
    return ka < kb;
  });
}

/// The four directions of a step along the outline: +x, +y, -x, -y.
constexpr std::array<int, 4> DIR_X = {1, 0, -1, 0};
constexpr std::array<int, 4> DIR_Y = {0, 1, 0, -1};

/// One edge between a free cell and a wall cell, directed so that the free
/// cell lies to its left. Its corners are points of the corner lattice:
/// corner (x, y) is the lower left corner of cell (x, y).
struct Crack {
  std::uint64_t from = 0;
  std::uint64_t cell = 0;
  std::uint8_t dir = 0;
  std::uint32_t wall = 0;
  bool used = false;
};

struct Corner {
  double x = 0.0;
  double y = 0.0;
};

/// The groups a vertex of the outline touches: at most three.
struct Groups {
  std::array<std::uint32_t, 3> of{};
  std::uint8_t count = 0;

  void add(const std::uint32_t group) {
    for (std::uint8_t k = 0; k < count; ++k) {
      if (of[k] == group) {
        return;
      }
    }
    if (count < of.size()) {
      of[count++] = group;
    }
  }
  [[nodiscard]] bool meets(const Groups& other) const {
    for (std::uint8_t a = 0; a < count; ++a) {
      for (std::uint8_t b = 0; b < other.count; ++b) {
        if (of[a] == other.of[b]) {
          return true;
        }
      }
    }
    return false;
  }
};

/// The cosine of the angle between a line and the nearest of the eight
/// headings a wire takes: the share of the line's length a crossing bundle
/// of wires sees.
double headingShareOf(const double dx, const double dy) {
  if (std::hypot(dx, dy) < 1e-9) {
    return 1.0;
  }
  constexpr double EIGHTH = std::numbers::pi / 4.0;
  const auto angle = std::atan2(dx, -dy);
  const auto folded = std::fmod(std::fmod(angle, EIGHTH) + EIGHTH, EIGHTH);
  return std::cos(std::min(folded, EIGHTH - folded));
}

/// Douglas-Peucker on one run of the outline, its two ends kept.
void simplifyRun(const std::vector<Corner>& in, const double tolerance,
                 std::vector<Corner>& out) {
  const auto count = in.size();
  if (count <= 2) {
    out.insert(out.end(), in.begin(), in.end());
    return;
  }
  std::vector<bool> keep(count, false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<std::size_t, std::size_t>> stack{{0, count - 1}};
  while (!stack.empty()) {
    const auto [lo, hi] = stack.back();
    stack.pop_back();
    const auto& a = in[lo];
    const auto& b = in[hi];
    const auto dx = b.x - a.x;
    const auto dy = b.y - a.y;
    const auto length = std::hypot(dx, dy);
    double worst = -1.0;
    std::size_t at = lo;
    for (auto k = lo + 1; k < hi; ++k) {
      const auto px = in[k].x - a.x;
      const auto py = in[k].y - a.y;
      const auto distance = length < 1e-12 ? std::hypot(px, py)
                                           : std::abs((dx * py) - (dy * px)) /
                                                 length;
      if (distance > worst) {
        worst = distance;
        at = k;
      }
    }
    if (worst > tolerance) {
      keep[at] = true;
      stack.emplace_back(lo, at);
      stack.emplace_back(at, hi);
    }
  }
  for (std::size_t k = 0; k < count; ++k) {
    if (keep[k]) {
      out.push_back(in[k]);
    }
  }
}

/// A union-find over indices that keeps, per set, the index the order
/// `less` ranks first.
struct MinSets {
  std::vector<std::uint32_t> parent;
  std::vector<std::uint32_t> least;

  explicit MinSets(const std::size_t count) : parent(count), least(count) {
    std::iota(parent.begin(), parent.end(), 0U);
    std::iota(least.begin(), least.end(), 0U);
  }
  std::uint32_t find(std::uint32_t at) {
    while (parent[at] != at) {
      parent[at] = parent[parent[at]];
      at = parent[at];
    }
    return at;
  }
  template <typename Less>
  void join(const std::uint32_t a, const std::uint32_t b, const Less& less) {
    const auto ra = find(a);
    const auto rb = find(b);
    if (ra == rb) {
      return;
    }
    parent[ra] = rb;
    if (less(least[ra], least[rb])) {
      least[rb] = least[ra];
    }
  }
};

} // namespace

std::vector<Bottleneck> cdtGates(const std::uint32_t width,
                                 const std::uint32_t height,
                                 const BitGrid& blocked,
                                 const std::span<const std::uint32_t> owner,
                                 const std::span<const CdtWall> walls,
                                 const std::uint32_t border,
                                 const CdtGateOptions& options,
                                 CdtGateStats* stats) {
  const auto w = static_cast<std::int64_t>(width);
  const auto h = static_cast<std::int64_t>(height);
  const auto stride = static_cast<std::uint64_t>(width) + 1U;
  const auto wallAt = [&](const std::int64_t x, const std::int64_t y) {
    return x < 0 || y < 0 || x >= w || y >= h ||
           blocked.test(static_cast<std::size_t>((y * w) + x));
  };
  const auto labelAt = [&](const std::int64_t x, const std::int64_t y) {
    if (x < 0 || y < 0 || x >= w || y >= h) {
      return border;
    }
    const auto label = owner[static_cast<std::size_t>((y * w) + x)];
    return label < walls.size() ? label : border;
  };
  const auto wallOf = [&](const std::uint32_t label) -> const CdtWall& {
    static const CdtWall OUTSIDE{
        .group = std::numeric_limits<std::uint32_t>::max(),
        .fixed = true,
        .target = false};
    return label < walls.size() ? walls[label] : OUTSIDE;
  };

  auto mark = std::chrono::steady_clock::now();
  std::array<double, 5> seconds{};
  const auto lap = [&](const std::size_t step) {
    const auto now = std::chrono::steady_clock::now();
    seconds.at(step) += std::chrono::duration<double>(now - mark).count();
    mark = now;
  };
  // The kind of every group: static, and a target run.
  constexpr auto NONE = std::numeric_limits<std::uint32_t>::max();
  std::vector<std::uint8_t> groupFixed(walls.size(), 1);
  std::vector<std::uint8_t> groupTarget(walls.size(), 0);
  for (const auto& wall : walls) {
    if (wall.group < walls.size()) {
      groupFixed[wall.group] = wall.fixed ? 1 : 0;
      groupTarget[wall.group] = wall.target ? 1 : 0;
    }
  }
  const auto fixedGroup = [&](const std::uint32_t group) {
    return group >= groupFixed.size() || groupFixed[group] != 0;
  };
  const auto targetGroup = [&](const std::uint32_t group) {
    return group < groupTarget.size() && groupTarget[group] != 0;
  };
  const auto allFixed = [&](const Groups& groups) {
    for (std::uint8_t g = 0; g < groups.count; ++g) {
      if (!fixedGroup(groups.of[g])) {
        return false;
      }
    }
    return true;
  };

  // 1. The outline: every edge between a free cell and a wall cell, the
  // free cell to its left. The free cells next to a wall are found a word
  // of the raster at a time.
  // The cracks of a cell lie together, in the order of their sides, from
  // the first of them on: `firstOf`, kept from call to call on the thread.
  constexpr auto NO_CRACK = std::numeric_limits<std::uint32_t>::max();
  thread_local std::vector<std::uint32_t> firstOf;
  thread_local std::vector<Crack> cracks;
  const auto cellCount = static_cast<std::size_t>(width) * height;
  if (firstOf.size() != cellCount) {
    firstOf.assign(cellCount, NO_CRACK);
  }
  cracks.clear();
  {
    const auto& words = blocked.words();
    const auto cells = static_cast<std::uint64_t>(width) * height;
    const auto bitsAt = [&](const std::int64_t offset) -> std::uint64_t {
      // 64 cells from `offset` on; cells outside the grid are walls.
      if (offset < 0 || static_cast<std::uint64_t>(offset) + 64U > cells) {
        std::uint64_t out = 0;
        for (int k = 0; k < 64; ++k) {
          const auto at = offset + k;
          if (at < 0 || static_cast<std::uint64_t>(at) >= cells ||
              blocked.test(static_cast<std::size_t>(at))) {
            out |= std::uint64_t{1} << k;
          }
        }
        return out;
      }
      const auto word = static_cast<std::size_t>(offset) / 64U;
      const auto shift = static_cast<unsigned>(offset % 64);
      if (shift == 0) {
        return words[word];
      }
      return (words[word] >> shift) | (words[word + 1] << (64U - shift));
    };
    // The first and the last cell of every row: their neighbour to the
    // side lies outside the grid, which the shifted words do not see (they
    // read the row before or after). A window of a larger grid has free cells
    // there.
    std::vector<std::uint64_t> sides((cells + 63) / 64, 0);
    for (std::uint64_t y = 0; y < height; ++y) {
      for (const auto cell : {y * width, (y * width) + width - 1}) {
        sides[cell / 64] |= std::uint64_t{1} << (cell % 64);
      }
    }
    for (std::uint64_t base = 0; base < cells; base += 64) {
      const auto at = static_cast<std::int64_t>(base);
      auto here = bitsAt(at);
      if (base + 64U > cells) {
        here |= ~std::uint64_t{0} << (cells - base);
      }
      const auto free = ~here;
      if (free == 0) {
        continue;
      }
      const auto near = bitsAt(at - 1) | bitsAt(at + 1) | bitsAt(at - w) |
                        bitsAt(at + w) | sides[base / 64];
      for (auto bits = free & near; bits != 0; bits &= bits - 1) {
        const auto cell = base + static_cast<std::uint64_t>(std::countr_zero(bits));
        const auto x = static_cast<std::int64_t>(cell % width);
        const auto y = static_cast<std::int64_t>(cell / width);
        const auto corner = [&](const std::int64_t cx, const std::int64_t cy) {
          return (static_cast<std::uint64_t>(cy) * stride) +
                 static_cast<std::uint64_t>(cx);
        };
        firstOf[cell] = static_cast<std::uint32_t>(cracks.size());
        if (wallAt(x, y - 1)) {
          cracks.push_back({corner(x, y), cell, 0, labelAt(x, y - 1), false});
        }
        if (wallAt(x + 1, y)) {
          cracks.push_back({corner(x + 1, y), cell, 1, labelAt(x + 1, y), false});
        }
        if (wallAt(x, y + 1)) {
          cracks.push_back({corner(x + 1, y + 1), cell, 2, labelAt(x, y + 1), false});
        }
        if (wallAt(x - 1, y)) {
          cracks.push_back({corner(x, y + 1), cell, 3, labelAt(x - 1, y), false});
        }
      }
    }
  }
  // The crack of one side of a cell, if there is one.
  const auto crackAt = [&](const std::int64_t x, const std::int64_t y,
                           const std::uint8_t side) -> std::size_t {
    if (x < 0 || y < 0 || x >= w || y >= h) {
      return cracks.size();
    }
    const auto cell = static_cast<std::uint64_t>((y * w) + x);
    const auto first = firstOf[cell];
    if (first == NO_CRACK) {
      return cracks.size();
    }
    for (auto k = static_cast<std::size_t>(first);
         k < cracks.size() && k < first + 4U && cracks[k].cell == cell; ++k) {
      if (cracks[k].dir == side) {
        return k;
      }
    }
    return cracks.size();
  };

  lap(0);
  // 2. The loops of the outline, each kept as its corners where it turns or
  // changes walls, with the wall of the piece from each on. At a corner two
  // walls touch diagonally the loop turns left, so that two free cells that
  // touch at a corner only are apart, as `chambersOf` has them.
  struct Loop {
    std::vector<Corner> points;
    std::vector<std::uint32_t> walls;
  };
  std::vector<Loop> loops;
  for (std::size_t start = 0; start < cracks.size(); ++start) {
    if (cracks[start].used) {
      continue;
    }
    Loop loop;
    auto at = start;
    int lastDir = -1;
    std::uint32_t lastWall = std::numeric_limits<std::uint32_t>::max();
    while (!cracks[at].used) {
      auto& crack = cracks[at];
      crack.used = true;
      if (crack.dir != lastDir || crack.wall != lastWall) {
        loop.points.push_back(
            {static_cast<double>(crack.from % stride),
             static_cast<double>(crack.from / stride)});
        loop.walls.push_back(crack.wall);
        lastDir = crack.dir;
        lastWall = crack.wall;
      }
      // From the end of a crack the outline turns left along the same
      // cell, runs on along the next cell, or turns right along the cell
      // diagonally ahead, in that order.
      const auto d = crack.dir;
      const auto cx = static_cast<std::int64_t>(crack.cell % width);
      const auto cy = static_cast<std::int64_t>(crack.cell / width);
      const auto right = static_cast<std::uint8_t>((d + 3U) % 4U);
      auto next = crackAt(cx, cy, static_cast<std::uint8_t>((d + 1U) % 4U));
      if (next == cracks.size()) {
        next = crackAt(cx + DIR_X[d], cy + DIR_Y[d], d);
      }
      if (next == cracks.size()) {
        next = crackAt(cx + DIR_X[d] + DIR_X[right], cy + DIR_Y[d] + DIR_Y[right], right);
      }
      if (next == cracks.size()) {
        break;
      }
      at = next;
    }
    if (loop.points.size() >= 3) {
      loops.push_back(std::move(loop));
    }
  }

  // Where a gate may end: within the widest gap of a wall that moves, by
  // squares of BUCKET cells. A static outline outside them is kept as the
  // simplification leaves it, not cut into pieces: no gate ends on it.
  const auto clearance = options.clearance;
  constexpr std::int64_t BUCKET = 32;
  const auto nearX = (w / BUCKET) + 1;
  const auto nearY = (h / BUCKET) + 1;
  std::vector<std::uint8_t> moving(static_cast<std::size_t>(nearX * nearY), 0);
  for (const auto& crack : cracks) {
    if (!wallOf(crack.wall).fixed) {
      const auto x = static_cast<std::int64_t>(crack.cell % width) / BUCKET;
      const auto y = static_cast<std::int64_t>(crack.cell / width) / BUCKET;
      moving[static_cast<std::size_t>((y * nearX) + x)] = 1;
    }
  }
  std::vector<std::uint8_t> near(moving.size(), 0);
  {
    const auto reachCells = (static_cast<double>(options.widest) * clearance *
                             options.margin) +
                            (2.0 * clearance);
    const auto r = static_cast<std::int64_t>(
        std::ceil(reachCells / static_cast<double>(BUCKET)));
    for (std::int64_t y = 0; y < nearY; ++y) {
      for (std::int64_t x = 0; x < nearX; ++x) {
        if (moving[static_cast<std::size_t>((y * nearX) + x)] == 0) {
          continue;
        }
        for (auto ny = std::max<std::int64_t>(y - r, 0);
             ny <= std::min(y + r, nearY - 1); ++ny) {
          for (auto nx = std::max<std::int64_t>(x - r, 0);
               nx <= std::min(x + r, nearX - 1); ++nx) {
            near[static_cast<std::size_t>((ny * nearX) + nx)] = 1;
          }
        }
      }
    }
  }
  const auto isNear = [&](const Corner& p) {
    const auto x = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(p.x) / BUCKET, 0, nearX - 1);
    const auto y = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(p.y) / BUCKET, 0, nearY - 1);
    return near[static_cast<std::size_t>((y * nearX) + x)] != 0;
  };

  // 3. Every loop as runs of one wall each, simplified, and cut to pieces no
  // longer than a clearance; every vertex with the groups it touches.
  std::vector<Corner> vertex;
  std::vector<Groups> groupsOf;
  std::vector<Groups> wallsAt;   // the wall labels a vertex touches
  std::vector<std::uint32_t> pieceWall;   // the wall of the piece from a vertex
  std::vector<std::uint32_t> pieceNext;   // the vertex the piece runs to
  std::vector<std::pair<std::size_t, std::size_t>> loopSpan;
  {
    std::vector<Corner> run;
    std::vector<Corner> simple;
    for (const auto& loop : loops) {
      const auto count = loop.points.size();
      // A loop of static walls whose box lies far from every wall that
      // moves ends no gate, and nothing inside it does: it is left out.
      if (std::ranges::all_of(loop.walls, [&](const auto wall) {
            return wallOf(wall).fixed;
          })) {
        auto lo = loop.points.front();
        auto hi = lo;
        for (const auto& p : loop.points) {
          lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
          hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        }
        bool close = false;
        const auto x0 = std::clamp<std::int64_t>(static_cast<std::int64_t>(lo.x) / BUCKET, 0, nearX - 1);
        const auto x1 = std::clamp<std::int64_t>(static_cast<std::int64_t>(hi.x) / BUCKET, 0, nearX - 1);
        const auto y0 = std::clamp<std::int64_t>(static_cast<std::int64_t>(lo.y) / BUCKET, 0, nearY - 1);
        const auto y1 = std::clamp<std::int64_t>(static_cast<std::int64_t>(hi.y) / BUCKET, 0, nearY - 1);
        for (auto y = y0; y <= y1 && !close; ++y) {
          for (auto x = x0; x <= x1 && !close; ++x) {
            close = near[static_cast<std::size_t>((y * nearX) + x)] != 0;
          }
        }
        if (!close) {
          continue;
        }
      }
      // Start at a change of walls, or anywhere in a loop of one wall.
      std::size_t first = 0;
      for (std::size_t k = 0; k < count; ++k) {
        if (loop.walls[k] != loop.walls[(k + count - 1) % count]) {
          first = k;
          break;
        }
      }
      const bool oneWall = std::ranges::all_of(
          loop.walls, [&](const auto wall) { return wall == loop.walls[0]; });
      std::vector<std::pair<Corner, std::uint32_t>> kept;
      std::size_t k = 0;
      while (k < count) {
        const auto wall = loop.walls[(first + k) % count];
        run.clear();
        auto end = k;
        while (end < count && loop.walls[(first + end) % count] == wall &&
               (!oneWall || end < (count / 2) + 1 || k > 0)) {
          run.push_back(loop.points[(first + end) % count]);
          ++end;
        }
        // The run ends where the next one begins.
        run.push_back(loop.points[(first + end) % count]);
        simple.clear();
        simplifyRun(run, options.simplify, simple);
        for (std::size_t s = 0; s + 1 < simple.size(); ++s) {
          kept.emplace_back(simple[s], wall);
        }
        k = end;
      }
      const auto begin = vertex.size();
      for (std::size_t s = 0; s < kept.size(); ++s) {
        const auto& [a, wall] = kept[s];
        const auto& b = kept[(s + 1) % kept.size()].first;
        const auto length = std::hypot(b.x - a.x, b.y - a.y);
        const bool cut = !wallOf(wall).fixed || isNear(a) || isNear(b);
        const auto pieces =
            cut ? std::max<std::size_t>(
                      1, static_cast<std::size_t>(std::ceil(length / clearance)))
                : std::size_t{1};
        for (std::size_t p = 0; p < pieces; ++p) {
          const auto t = static_cast<double>(p) / static_cast<double>(pieces);
          vertex.push_back({a.x + (t * (b.x - a.x)), a.y + (t * (b.y - a.y))});
          Groups groups;
          groups.add(wallOf(wall).group);
          groupsOf.push_back(groups);
          Groups labels;
          labels.add(wall);
          wallsAt.push_back(labels);
          pieceWall.push_back(wall);
        }
      }
      const auto end = vertex.size();
      for (auto v = begin; v < end; ++v) {
        pieceNext.push_back(static_cast<std::uint32_t>(v + 1 < end ? v + 1 : begin));
      }
      // A vertex where two walls meet touches both.
      for (auto v = begin; v < end; ++v) {
        const auto before = v > begin ? v - 1 : end - 1;
        groupsOf[v].add(wallOf(pieceWall[before]).group);
        wallsAt[v].add(pieceWall[before]);
      }
      loopSpan.emplace_back(begin, end);
    }
  }

  for (const auto& crack : cracks) {
    firstOf[crack.cell] = NO_CRACK;
  }
  lap(1);
  // 4. The foot points: for every vertex and every other obstacle whose
  // outline passes within `footReach` clearances, the nearest point of that
  // outline, unless both are static.
  const auto reach = options.footReach * clearance;
  std::vector<std::vector<double>> feet(vertex.size());
  {
    const auto bucket = std::max(reach, 1.0);
    const auto bucketsX = static_cast<std::int64_t>(std::ceil((static_cast<double>(width) + 2.0) / bucket)) + 1;
    const auto bucketsY = static_cast<std::int64_t>(std::ceil((static_cast<double>(height) + 2.0) / bucket)) + 1;
    std::vector<std::vector<std::uint32_t>> buckets(
        static_cast<std::size_t>(bucketsX * bucketsY));
    const auto bucketOf = [&](const double v) {
      return std::clamp<std::int64_t>(static_cast<std::int64_t>(std::floor(v / bucket)), 0,
                                      std::max(bucketsX, bucketsY) - 1);
    };
    for (std::uint32_t v = 0; v < vertex.size(); ++v) {
      const auto& a = vertex[v];
      const auto& b = vertex[pieceNext[v]];
      const auto x0 = std::min(bucketOf(std::min(a.x, b.x)), bucketsX - 1);
      const auto x1 = std::min(bucketOf(std::max(a.x, b.x)), bucketsX - 1);
      const auto y0 = std::min(bucketOf(std::min(a.y, b.y)), bucketsY - 1);
      const auto y1 = std::min(bucketOf(std::max(a.y, b.y)), bucketsY - 1);
      for (auto by = y0; by <= y1; ++by) {
        for (auto bx = x0; bx <= x1; ++bx) {
          buckets[static_cast<std::size_t>((by * bucketsX) + bx)].push_back(v);
        }
      }
    }
    struct Best {
      std::uint32_t group;
      double distance;
      std::uint32_t piece;
      double t;
    };
    std::vector<Best> best;
    for (std::uint32_t v = 0; v < vertex.size(); ++v) {
      const auto& p = vertex[v];
      const auto& mine = groupsOf[v];
      const bool mineStatic = allFixed(mine);
      best.clear();
      const auto cx = std::min(bucketOf(p.x), bucketsX - 1);
      const auto cy = std::min(bucketOf(p.y), bucketsY - 1);
      for (auto by = std::max<std::int64_t>(cy - 1, 0); by <= std::min(cy + 1, bucketsY - 1); ++by) {
        for (auto bx = std::max<std::int64_t>(cx - 1, 0); bx <= std::min(cx + 1, bucketsX - 1); ++bx) {
          for (const auto s : buckets[static_cast<std::size_t>((by * bucketsX) + bx)]) {
            const auto group = wallOf(pieceWall[s]).group;
            bool same = false;
            for (std::uint8_t k = 0; k < mine.count; ++k) {
              same = same || mine.of[k] == group;
            }
            if (same || (mineStatic && fixedGroup(group))) {
              continue;
            }
            const auto& a = vertex[s];
            const auto& b = vertex[pieceNext[s]];
            const auto dx = b.x - a.x;
            const auto dy = b.y - a.y;
            const auto squared = (dx * dx) + (dy * dy);
            if (squared < 1e-12) {
              continue;
            }
            const auto t = (((p.x - a.x) * dx) + ((p.y - a.y) * dy)) / squared;
            if (t <= 0.02 || t >= 0.98) {
              continue;
            }
            const auto distance =
                std::hypot(a.x + (t * dx) - p.x, a.y + (t * dy) - p.y);
            if (distance > reach) {
              continue;
            }
            auto found = std::ranges::find_if(
                best, [&](const Best& one) { return one.group == group; });
            if (found == best.end()) {
              best.push_back({group, distance, s, t});
            } else if (distance < found->distance) {
              *found = {group, distance, s, t};
            }
          }
        }
      }
      for (const auto& one : best) {
        feet[one.piece].push_back(one.t);
      }
    }
  }

  lap(2);
  // 5. The vertices and constraints of the triangulation: every loop with
  // its foot points inserted.
  std::vector<CDT::V2d<double>> points;
  std::vector<Groups> pointGroups;
  std::vector<Groups> pointWalls;
  std::vector<CDT::Edge> edges;
  for (const auto& [begin, end] : loopSpan) {
    const auto first = static_cast<CDT::VertInd>(points.size());
    for (auto v = begin; v < end; ++v) {
      points.push_back(CDT::V2d<double>(vertex[v].x, vertex[v].y));
      pointGroups.push_back(groupsOf[v]);
      pointWalls.push_back(wallsAt[v]);
      auto& ts = feet[v];
      std::ranges::sort(ts);
      const auto& a = vertex[v];
      const auto& b = vertex[pieceNext[v]];
      double last = -1.0;
      for (const auto t : ts) {
        if (t - last < 1e-3) {
          continue;
        }
        last = t;
        points.push_back(
            CDT::V2d<double>(a.x + (t * (b.x - a.x)), a.y + (t * (b.y - a.y))));
        Groups groups;
        groups.add(wallOf(pieceWall[v]).group);
        pointGroups.push_back(groups);
        Groups labels;
        labels.add(pieceWall[v]);
        pointWalls.push_back(labels);
      }
    }
    const auto last = static_cast<CDT::VertInd>(points.size());
    for (auto p = first; p < last; ++p) {
      edges.emplace_back(p, p + 1 < last ? p + 1 : first);
    }
  }
  const auto duplicates = CDT::RemoveDuplicatesAndRemapEdges(points, edges);
  {
    std::vector<Groups> merged(points.size());
    std::vector<Groups> mergedWalls(points.size());
    for (std::size_t k = 0; k < duplicates.mapping.size(); ++k) {
      auto& into = merged[duplicates.mapping[k]];
      for (std::uint8_t g = 0; g < pointGroups[k].count; ++g) {
        into.add(pointGroups[k].of[g]);
      }
      auto& intoWalls = mergedWalls[duplicates.mapping[k]];
      for (std::uint8_t g = 0; g < pointWalls[k].count; ++g) {
        intoWalls.add(pointWalls[k].of[g]);
      }
    }
    pointGroups = std::move(merged);
    pointWalls = std::move(mergedWalls);
  }
  std::erase_if(edges, [](const CDT::Edge& edge) { return edge.v1() == edge.v2(); });
  // For tests: the points in another order, their edges with them.
  if (options.shuffle != 0) {
    std::vector<CDT::VertInd> order(points.size());
    std::iota(order.begin(), order.end(), CDT::VertInd{0});
    std::uint64_t state = options.shuffle;
    for (auto k = order.size(); k > 1; --k) {
      state = splitMix(state);
      std::swap(order[k - 1], order[state % k]);
    }
    std::vector<CDT::VertInd> where(points.size());
    for (std::size_t k = 0; k < order.size(); ++k) {
      where[order[k]] = static_cast<CDT::VertInd>(k);
    }
    const auto permuted = [&](auto& list) {
      auto copy = list;
      for (std::size_t k = 0; k < order.size(); ++k) {
        list[k] = copy[order[k]];
      }
    };
    permuted(points);
    permuted(pointGroups);
    permuted(pointWalls);
    for (auto& edge : edges) {
      edge = CDT::Edge(where[edge.v1()], where[edge.v2()]);
    }
    std::ranges::reverse(edges);
  }
  // Canonical: every point moved by its own deterministic amount, so that the
  // triangulation is the one Delaunay triangulation of the moved points.
  auto placed = points;
  if (options.canonical) {
    constexpr double JITTER = 1e-4;
    for (auto& p : placed) {
      const auto hx = splitMix(std::bit_cast<std::uint64_t>(p.x) ^
                               splitMix(std::bit_cast<std::uint64_t>(p.y)));
      const auto unit = [](const std::uint64_t bits) {
        return (static_cast<double>(bits & 0xffffffU) / 16777215.0) - 0.5;
      };
      p.x += unit(hx) * JITTER;
      p.y += unit(hx >> 24U) * JITTER;
    }
  }
  CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::Auto,
                                 CDT::IntersectingConstraintEdges::TryResolve,
                                 0.0);
  cdt.insertVertices(placed);
  cdt.insertEdges(edges);
  // The free triangles, at an odd depth of constraints from the outside, as
  // `eraseOuterTrianglesAndHoles` finds them, but without erasing the others
  // and renumbering what is left, which took as long again as the
  // triangulation. The super-triangle stays, its three vertices first: a
  // vertex of the input is its index plus three.
  // Whether an edge is a constraint: a piece of an outline. The pieces of
  // the input are looked up by their ends (an end has at most a few); a
  // vertex the triangulation added where constraints crossed asks the
  // triangulation's own set.
  constexpr std::size_t SUPER = 3;
  const auto inputVertices = points.size() + SUPER;
  std::vector<std::array<CDT::VertInd, 4>> pieceEnds(
      inputVertices, {CDT::noVertex, CDT::noVertex, CDT::noVertex, CDT::noVertex});
  for (const auto& edge : edges) {
    const auto a = edge.v1() + SUPER;
    const auto b = edge.v2() + SUPER;
    for (const auto [from, to] : {std::pair{a, b}, std::pair{b, a}}) {
      for (auto& slot : pieceEnds[from]) {
        if (slot == CDT::noVertex || slot == to) {
          slot = to;
          break;
        }
      }
    }
  }
  // Where a vertex is, unmoved: a point of the input where it was given, a
  // point the triangulation added where it put it.
  const auto at = [&](const CDT::VertInd v) -> const CDT::V2d<double>& {
    return v >= SUPER && v < inputVertices ? points[v - SUPER] : cdt.vertices[v];
  };
  const auto isPiece = [&](const CDT::VertInd a, const CDT::VertInd b) {
    if (a >= inputVertices || b >= inputVertices) {
      return cdt.fixedEdges.contains(CDT::Edge(a, b));
    }
    return std::ranges::find(pieceEnds[a], b) != pieceEnds[a].end();
  };
  // The free triangles: an odd count of outlines from the outside, found by
  // a walk that pays one for crossing a piece and nothing else.
  std::vector<std::uint8_t> freeTriangle(cdt.triangles.size(), 0);
  {
    constexpr auto UNSEEN = std::numeric_limits<std::uint32_t>::max();
    std::vector<std::uint32_t> depth(cdt.triangles.size(), UNSEEN);
    std::vector<CDT::TriInd> now;
    std::vector<CDT::TriInd> later;
    for (std::size_t t = 0; t < cdt.triangles.size(); ++t) {
      const auto& v = cdt.triangles[t].vertices;
      if (v[0] < SUPER || v[1] < SUPER || v[2] < SUPER) {
        depth[t] = 0;
        now.push_back(static_cast<CDT::TriInd>(t));
      }
    }
    std::uint32_t level = 0;
    while (!now.empty()) {
      for (std::size_t at = 0; at < now.size(); ++at) {
        const auto t = now[at];
        const auto& triangle = cdt.triangles[t];
        for (std::size_t k = 0; k < 3; ++k) {
          const auto across = triangle.neighbors[k];
          if (across == CDT::noNeighbor || depth[across] != UNSEEN) {
            continue;
          }
          if (isPiece(triangle.vertices[k], triangle.vertices[(k + 1) % 3])) {
            later.push_back(across);
          } else {
            depth[across] = level;
            now.push_back(across);
          }
        }
      }
      now.clear();
      ++level;
      for (const auto t : later) {
        if (depth[t] == UNSEEN) {
          depth[t] = level;
          now.push_back(t);
        }
      }
      later.clear();
    }
    for (std::size_t t = 0; t < cdt.triangles.size(); ++t) {
      freeTriangle[t] = depth[t] != UNSEEN && depth[t] % 2 == 1 ? 1 : 0;
    }
  }
  pointGroups.insert(pointGroups.begin(), 3, Groups{});
  pointWalls.insert(pointWalls.begin(), 3, Groups{});
  lap(3);
  const auto original = pointGroups.size();
  // A vertex the triangulation added where two constraints crossed takes the
  // groups of the nearest original vertex it is joined to.
  pointGroups.resize(cdt.vertices.size());
  pointWalls.resize(cdt.vertices.size());
  if (cdt.vertices.size() > original) {
    std::vector<double> nearest(cdt.vertices.size(), std::numeric_limits<double>::max());
    for (const auto& triangle : cdt.triangles) {
      for (std::size_t k = 0; k < 3; ++k) {
        const auto a = triangle.vertices[k];
        const auto b = triangle.vertices[(k + 1) % 3];
        for (const auto [added, old] : {std::pair{a, b}, std::pair{b, a}}) {
          if (added >= original && old < original) {
            const auto d = CDT::distance(cdt.vertices[added], cdt.vertices[old]);
            if (d < nearest[added]) {
              nearest[added] = d;
              pointGroups[added] = pointGroups[old];
              pointWalls[added] = pointWalls[old];
            }
          }
        }
      }
    }
  }

  // 6. The candidates: every edge between two free triangles whose ends lie
  // on two different obstacles, not both static.
  std::vector<std::uint8_t> staticOf(cdt.vertices.size());
  std::vector<std::uint8_t> targetOf(cdt.vertices.size());
  std::vector<std::uint32_t> sideOf(cdt.vertices.size());
  for (std::size_t v = 0; v < cdt.vertices.size(); ++v) {
    const auto& groups = pointGroups[v];
    bool dynamic = false;
    bool onlyTarget = groups.count > 0;
    std::uint32_t side = NONE;
    std::uint32_t any = NONE;
    for (std::uint8_t g = 0; g < groups.count; ++g) {
      const auto group = groups.of[g];
      any = std::min(any, group);
      if (!fixedGroup(group)) {
        dynamic = true;
        side = std::min(side, group);
        onlyTarget = onlyTarget && targetGroup(group);
      }
    }
    staticOf[v] = dynamic ? 0 : 1;
    targetOf[v] = dynamic && onlyTarget ? 1 : 0;
    sideOf[v] = dynamic ? side : any;
  }
  const auto widest = options.widest;
  const auto wiresIn = [&](const double cells) -> std::uint32_t {
    if (cells < clearance) {
      return 0;
    }
    const auto plain = static_cast<std::uint32_t>(std::floor(cells / clearance));
    if (plain < options.marginFrom) {
      return plain;
    }
    return static_cast<std::uint32_t>(std::floor(cells / (clearance * options.margin)));
  };
  struct Candidate {
    CDT::VertInd a;
    CDT::VertInd b;
    std::uint32_t cap;
    /// The length of the edge: of the candidates of a basin the shortest is
    /// its gate.
    double width;
    std::uint64_t pair;
    /// Its two ends, the lower first: what tells two candidates of one width
    /// apart when the gates are canonical.
    std::array<double, 4> key;
  };
  std::vector<Candidate> candidates;
  std::vector<std::array<std::uint32_t, 3>> onTriangle(
      cdt.triangles.size(), {std::numeric_limits<std::uint32_t>::max(),
                             std::numeric_limits<std::uint32_t>::max(),
                             std::numeric_limits<std::uint32_t>::max()});
  for (std::size_t t = 0; t < cdt.triangles.size(); ++t) {
    const auto& triangle = cdt.triangles[t];
    for (std::size_t k = 0; k < 3; ++k) {
      const auto across = triangle.neighbors[k];
      if (freeTriangle[t] == 0 || across == CDT::noNeighbor || across < t ||
          freeTriangle[across] == 0) {
        continue;
      }
      const auto a = triangle.vertices[k];
      const auto b = triangle.vertices[(k + 1) % 3];
      const auto& ga = pointGroups[a];
      const auto& gb = pointGroups[b];
      if (ga.count == 0 || gb.count == 0 || ga.meets(gb)) {
        continue;
      }
      const bool quietA = staticOf[a] != 0 || targetOf[a] != 0;
      const bool quietB = staticOf[b] != 0 || targetOf[b] != 0;
      if (quietA && quietB && !(targetOf[a] != 0 && targetOf[b] != 0)) {
        continue;
      }
      const auto& pa = at(a);
      const auto& pb = at(b);
      const auto length = std::hypot(pb.x - pa.x, pb.y - pa.y);
      const auto across_ = length * headingShareOf(pb.x - pa.x, pb.y - pa.y);
      const auto cap = wiresIn(across_);
      if (cap >= widest) {
        continue;
      }
      const auto lo = std::min(sideOf[a], sideOf[b]);
      const auto hi = std::max(sideOf[a], sideOf[b]);
      const auto index = static_cast<std::uint32_t>(candidates.size());
      const bool aFirst = pa.x != pb.x ? pa.x < pb.x : pa.y < pb.y;
      const auto& p0 = aFirst ? pa : pb;
      const auto& p1 = aFirst ? pb : pa;
      candidates.push_back({a, b, cap, length,
                            (static_cast<std::uint64_t>(lo) << 32U) | hi,
                            {p0.x, p0.y, p1.x, p1.y}});
      for (const auto on : {t, static_cast<std::size_t>(across)}) {
        auto& slots = onTriangle[on];
        for (auto& slot : slots) {
          if (slot == std::numeric_limits<std::uint32_t>::max()) {
            slot = index;
            break;
          }
        }
      }
    }
  }

  // 7. The basins: per capacity, the candidates joined to their neighbours
  // of the same channel that hold at least one wire and no more; a
  // candidate is a gate when it is the narrowest of its set. The candidates
  // of no wire form their own sets.
  const auto narrower = [&](const std::uint32_t a, const std::uint32_t b) {
    if (candidates[a].width != candidates[b].width) {
      return candidates[a].width < candidates[b].width;
    }
    if (options.canonical && candidates[a].key != candidates[b].key) {
      return candidates[a].key < candidates[b].key;
    }
    return a < b;
  };
  std::vector<std::vector<std::uint32_t>> neighbours(candidates.size());
  for (const auto& slots : onTriangle) {
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        const auto a = slots[i];
        const auto b = slots[j];
        if (i == j || a == std::numeric_limits<std::uint32_t>::max() ||
            b == std::numeric_limits<std::uint32_t>::max() ||
            candidates[a].pair != candidates[b].pair) {
          continue;
        }
        // Two candidates of one channel are neighbours unless the third edge
        // of their triangle, between their two ends that are not shared,
        // spans the mouth of a port run: it joins the run's two flanks, two
        // walls of one group. The terminal lies in that mouth, so the gaps
        // on either side of it are no gates in a row, and each keeps a gate
        // of its own.
        const auto& one = candidates[a];
        const auto& two = candidates[b];
        const auto shared =
            (one.a == two.a || one.a == two.b) ? one.a : one.b;
        const auto x = one.a == shared ? one.b : one.a;
        const auto y = two.a == shared ? two.b : two.a;
        // Along an outline (the back of a run, where its flanks meet) the
        // edge is no mouth.
        bool mouth = false;
        const bool along = isPiece(x, y);
        for (std::uint8_t i = 0; i < pointWalls[x].count && !mouth && !along;
             ++i) {
          for (std::uint8_t j = 0; j < pointWalls[y].count && !mouth; ++j) {
            const auto wx = pointWalls[x].of[i];
            const auto wy = pointWalls[y].of[j];
            mouth = wx != wy && wallOf(wx).group == wallOf(wy).group;
          }
        }
        if (!mouth) {
          neighbours[a].push_back(b);
        }
      }
    }
  }
  std::vector<std::uint32_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0U);
  std::ranges::stable_sort(order, [&](const auto a, const auto b) {
    return candidates[a].cap < candidates[b].cap;
  });
  MinSets sets(candidates.size());
  std::vector<bool> added(candidates.size(), false);
  std::vector<std::uint32_t> gates;
  std::size_t contacts = 0;
  constexpr double NARROWEST = 3.0;
  for (std::size_t from = 0; from < order.size();) {
    const auto cap = candidates[order[from]].cap;
    auto to = from;
    while (to < order.size() && candidates[order[to]].cap == cap) {
      added[order[to]] = true;
      ++to;
    }
    for (auto k = from; k < to; ++k) {
      const auto e = order[k];
      for (const auto f : neighbours[e]) {
        const auto other = candidates[f].cap;
        if (added[f] && ((cap == 0) == (other == 0))) {
          sets.join(e, f, narrower);
        }
      }
    }
    for (auto k = from; k < to; ++k) {
      const auto e = order[k];
      if (sets.least[sets.find(e)] != e) {
        continue;
      }
      if (cap == 0) {
        if (candidates[e].width < NARROWEST) {
          continue;
        }
        ++contacts;
      }
      gates.push_back(e);
    }
    from = to;
  }

  // 8. Each gate as the two wall cells of its obstacles nearest its ends.
  const auto wallCellNear = [&](const CDT::V2d<double>& p, const Groups& groups) {
    const auto px = static_cast<std::int64_t>(std::floor(p.x));
    const auto py = static_cast<std::int64_t>(std::floor(p.y));
    double best = std::numeric_limits<double>::max();
    std::size_t found = std::numeric_limits<std::size_t>::max();
    double bestAny = std::numeric_limits<double>::max();
    std::size_t any = std::numeric_limits<std::size_t>::max();
    for (std::int64_t y = py - 3; y <= py + 2; ++y) {
      for (std::int64_t x = px - 3; x <= px + 2; ++x) {
        if (x < 0 || y < 0 || x >= w || y >= h || !wallAt(x, y)) {
          continue;
        }
        const auto cell = static_cast<std::size_t>((y * w) + x);
        const auto d = std::hypot((static_cast<double>(x) + 0.5) - p.x,
                                  (static_cast<double>(y) + 0.5) - p.y);
        const auto group = wallOf(labelAt(x, y)).group;
        bool mine = false;
        for (std::uint8_t g = 0; g < groups.count; ++g) {
          mine = mine || groups.of[g] == group;
        }
        if (mine && d < best) {
          best = d;
          found = cell;
        }
        if (d < bestAny) {
          bestAny = d;
          any = cell;
        }
      }
    }
    return found != std::numeric_limits<std::size_t>::max() ? found : any;
  };
  std::vector<Bottleneck> out;
  out.reserve(gates.size());
  for (const auto e : gates) {
    const auto& one = candidates[e];
    const auto& pa = at(one.a);
    const auto& pb = at(one.b);
    const auto first = wallCellNear(pa, pointGroups[one.a]);
    const auto second = wallCellNear(pb, pointGroups[one.b]);
    if (first == std::numeric_limits<std::size_t>::max() ||
        second == std::numeric_limits<std::size_t>::max() || first == second) {
      continue;
    }
    const auto mx = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor((pa.x + pb.x) / 2.0)), 0, w - 1);
    const auto my = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor((pa.y + pb.y) / 2.0)), 0, h - 1);
    out.push_back({.first = first,
                   .second = second,
                   .saddle = static_cast<std::size_t>((my * w) + mx)});
  }
  if (options.canonical) {
    sortCanonically(out);
  }
  lap(4);
  if (stats != nullptr) {
    stats->seconds = seconds;
    stats->vertices = cdt.vertices.size();
    stats->constraints = edges.size();
    stats->triangles = static_cast<std::size_t>(
        std::ranges::count(freeTriangle, std::uint8_t{1}));
    stats->candidates = candidates.size();
    stats->gates = out.size();
    stats->contacts = contacts;
  }
  return out;
}

std::vector<Bottleneck> cdtGatesCached(
    const std::uint32_t width, const std::uint32_t height, const BitGrid& blocked,
    const std::span<const std::uint32_t> owner, const std::span<const CdtWall> walls,
    const std::uint32_t border, const CdtGateOptions& options, CdtGateCache& cache,
    CdtGateStats* stats) {
  const auto cells = static_cast<std::size_t>(width) * height;
  if (cache.width != width || cache.height != height) {
    cache.entries.clear();
    cache.width = width;
    cache.height = height;
  }
  // The kind of every cell, the free ones by the words of the mask.
  std::vector<std::uint8_t> kind(cells, 0);
  {
    const auto& words = blocked.words();
    for (std::size_t word = 0; word < words.size(); ++word) {
      for (auto bits = words[word]; bits != 0; bits &= bits - 1) {
        const auto cell =
            (word * 64U) + static_cast<std::size_t>(std::countr_zero(bits));
        if (cell >= cells) {
          break;
        }
        const auto label = owner[cell];
        const auto& wall = label < walls.size() ? walls[label] : walls[border];
        kind[cell] = wall.fixed ? 1 : (wall.target ? 3 : 2);
      }
    }
  }
  // The box of the cells that differ from an entry; empty when none do.
  struct Box {
    std::int64_t x0 = std::numeric_limits<std::int64_t>::max();
    std::int64_t y0 = std::numeric_limits<std::int64_t>::max();
    std::int64_t x1 = -1;
    std::int64_t y1 = -1;
    [[nodiscard]] bool empty() const { return x1 < x0; }
    [[nodiscard]] double area() const {
      return empty() ? 0.0
                     : static_cast<double>(x1 - x0 + 1) *
                           static_cast<double>(y1 - y0 + 1);
    }
  };
  const auto boxAgainst = [&](const CdtGateCache::Entry& entry) {
    Box box;
    for (std::uint32_t y = 0; y < height; ++y) {
      const auto* now = kind.data() + (static_cast<std::size_t>(y) * width);
      const auto* was = entry.kind.data() + (static_cast<std::size_t>(y) * width);
      if (std::memcmp(now, was, width) == 0) {
        continue;
      }
      std::uint32_t lo = 0;
      while (now[lo] == was[lo]) {
        ++lo;
      }
      std::uint32_t hi = width - 1;
      while (now[hi] == was[hi]) {
        --hi;
      }
      box.x0 = std::min<std::int64_t>(box.x0, lo);
      box.x1 = std::max<std::int64_t>(box.x1, hi);
      box.y0 = std::min<std::int64_t>(box.y0, y);
      box.y1 = y;
    }
    return box;
  };
  CdtGateCache::Entry* best = nullptr;
  Box changed;
  for (auto& entry : cache.entries) {
    const auto box = boxAgainst(entry);
    if (best == nullptr || box.area() < changed.area()) {
      best = &entry;
      changed = box;
    }
    if (box.empty()) {
      break;
    }
  }
  const auto remember = [&](std::vector<Bottleneck> gates) {
    if (cache.entries.size() >= cache.capacity) {
      auto oldest = std::ranges::min_element(
          cache.entries, {}, [](const auto& entry) { return entry.used; });
      cache.entries.erase(oldest);
    }
    if (options.canonical) {
      sortCanonically(gates);
    }
    cache.entries.push_back(
        {.kind = std::move(kind), .gates = gates, .used = ++cache.clock});
    return gates;
  };
  if (best != nullptr && changed.empty()) {
    ++cache.reused;
    best->used = ++cache.clock;
    if (stats != nullptr) {
      *stats = CdtGateStats{};
      stats->gates = best->gates.size();
    }
    return best->gates;
  }
  const auto grow = static_cast<std::int64_t>(
      std::ceil((static_cast<double>(options.widest) * options.clearance *
                 options.margin) +
                options.clearance));
  const auto w = static_cast<std::int64_t>(width);
  const auto h = static_cast<std::int64_t>(height);
  Box inner{.x0 = changed.x0 - grow, .y0 = changed.y0 - grow,
            .x1 = changed.x1 + grow, .y1 = changed.y1 + grow};
  Box window{.x0 = std::max<std::int64_t>(0, inner.x0 - grow),
             .y0 = std::max<std::int64_t>(0, inner.y0 - grow),
             .x1 = std::min<std::int64_t>(w - 1, inner.x1 + grow),
             .y1 = std::min<std::int64_t>(h - 1, inner.y1 + grow)};
  if (best == nullptr || window.area() > 0.6 * static_cast<double>(cells)) {
    ++cache.whole;
    return remember(
        cdtGates(width, height, blocked, owner, walls, border, options, stats));
  }
  // The window as a raster of its own; what lies outside it is a wall of a
  // static label of its own, so that no gate of the window ends there but
  // near the window's edge, outside the inner box.
  const auto ww = static_cast<std::uint32_t>(window.x1 - window.x0 + 1);
  const auto wh = static_cast<std::uint32_t>(window.y1 - window.y0 + 1);
  BitGrid part(ww, wh);
  std::vector<std::uint32_t> partOwner(static_cast<std::size_t>(ww) * wh);
  for (std::uint32_t y = 0; y < wh; ++y) {
    const auto from = (static_cast<std::size_t>(window.y0) + y) * width +
                      static_cast<std::size_t>(window.x0);
    std::memcpy(partOwner.data() + (static_cast<std::size_t>(y) * ww),
                owner.data() + from, ww * sizeof(std::uint32_t));
    for (std::uint32_t x = 0; x < ww; ++x) {
      if (kind[from + x] != 0) {
        part.setCell(x, y, true);
      }
    }
  }
  std::vector<CdtWall> partWalls(walls.begin(), walls.end());
  const auto frame = static_cast<std::uint32_t>(partWalls.size());
  partWalls.push_back({.group = frame, .fixed = true, .target = false});
  // A window that reaches the grid's edge meets the border there.
  const auto outside = window.x0 == 0 && window.y0 == 0 && window.x1 == w - 1 &&
                               window.y1 == h - 1
                           ? border
                           : frame;
  auto found =
      cdtGates(ww, wh, part, partOwner, partWalls, outside, options, stats);
  const auto middleIn = [&](const Bottleneck& gate, const std::int64_t dx,
                            const std::int64_t dy, const std::int64_t stride) {
    const auto ax = static_cast<std::int64_t>(gate.first) % stride + dx;
    const auto ay = static_cast<std::int64_t>(gate.first) / stride + dy;
    const auto bx = static_cast<std::int64_t>(gate.second) % stride + dx;
    const auto by = static_cast<std::int64_t>(gate.second) / stride + dy;
    const auto mx = (ax + bx) / 2;
    const auto my = (ay + by) / 2;
    return mx >= inner.x0 && mx <= inner.x1 && my >= inner.y0 && my <= inner.y1;
  };
  std::vector<Bottleneck> gates;
  for (const auto& gate : best->gates) {
    if (!middleIn(gate, 0, 0, w)) {
      gates.push_back(gate);
    }
  }
  const auto toGrid = [&](const std::size_t cell) {
    const auto x = static_cast<std::int64_t>(cell % ww) + window.x0;
    const auto y = static_cast<std::int64_t>(cell / ww) + window.y0;
    return static_cast<std::size_t>((y * w) + x);
  };
  for (const auto& gate : found) {
    if (middleIn(gate, window.x0, window.y0, ww)) {
      gates.push_back({.first = toGrid(gate.first),
                       .second = toGrid(gate.second),
                       .saddle = toGrid(gate.saddle)});
    }
  }
  ++cache.windowed;
  cache.windowCells += static_cast<std::uint64_t>(ww) * wh;
  best->used = ++cache.clock;
  if (stats != nullptr) {
    stats->gates = gates.size();
  }
  return remember(std::move(gates));
}

} // namespace mqt::scpd::grid
