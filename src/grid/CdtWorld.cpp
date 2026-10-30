/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/CdtWorld.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/CdtGates.hpp"
#include "mqt-scpd/grid/Chambers.hpp"

#include <CDT.h>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <unordered_set>
#include <vector>

namespace mqt::scpd::grid {

namespace {

using u32 = std::uint32_t;
using u64 = std::uint64_t;
using Clock = std::chrono::steady_clock;
constexpr u32 NONE = std::numeric_limits<u32>::max();
/// The vertices of artem's super-triangle come first.
constexpr u32 SUPER = 3;
/// How far a foot point on a constraint already in place lies off it, on its
/// free side, in cells: far above the rounding of the coordinates (about
/// 1e-13 cells) and far below the perturbation (1e-4 cells).
constexpr double FOOT_OFFSET = 1e-6;
/// How near a constraint a vertex counts as lying on it, relative to the
/// constraint's length (CDT's `minDistToConstraintEdge`): exactly on it.
/// The vertices CDT adds where constraints cross are put exactly onto the
/// row or column being inserted as they come (`Snapper`), so that the rest
/// of the constraint meets the vertices on its line exactly.
constexpr double CONSTRAINT_TOLERANCE = 0.0;
/// How near a piece a foot of `FOOT_OFFSET` counts as lying on it.
constexpr double HAIR_REACH = 1e-4;
/// A gap of no wire narrower than this is no contact (`cdtGates`).
constexpr double NARROWEST = 3.0;

double msSince(const Clock::time_point from, const Clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

/// A well-mixed 64-bit hash of a 64-bit value (SplitMix64), as `cdtGates`
/// has it.
u64 splitMix(u64 value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

struct XY {
  double x = 0.0;
  double y = 0.0;
  [[nodiscard]] bool operator==(const XY&) const = default;
};

/**
 * @brief Where the world puts a point: moved by a deterministic amount that
 * makes the triangulation of a set of points and constraints the only one,
 * and keeps every row and column of the grid a line.
 *
 * A point (x, y) goes to (X + s (Y - c), Y), where X depends on x alone and
 * Y on y alone: each moved by a hash of its own bits (two different hashes)
 * and rounded to a dyadic step 2^-q, and s = 2^-k a shear about the middle
 * row c. Step and shear follow from the size of the grid so that
 * X + s (Y - c) takes no more than the 53 bits of a double: every placed
 * coordinate is exact.
 *
 * - The points of one column of the grid (one x) lie on one line, exactly,
 *   and so do those of one row. Two walls that touch along a line of the
 *   grid (two objects, or an object and a static wall) have outlines on one
 *   line: a corner of one on a piece of the other lies on that piece, and
 *   the triangulation splits it there. Moved apart by a hash of both
 *   coordinates, the corner would lie a hair off the piece, and the piece
 *   from it would cross the other at an angle the triangulation cannot
 *   always resolve (`InvalidEdgeSplitVertex`).
 * - Moving x and y breaks the ties of four points on one circle, but for
 *   the rectangles of the grid, which stay rectangles; the shear makes them
 *   parallelograms, on no circle. X and Y take different hashes, so that a
 *   set symmetric about a diagonal does not stay so.
 */
struct Placement {
  double step = 0.0;
  double inverse = 0.0;
  double shear = 0.0;
  double middle = 0.0;
  /// The range x and y are moved in, centred on them.
  double jitter = 0.0;

  void fit(const std::uint32_t width, const std::uint32_t height) {
    // A coordinate takes m bits above the point. X + s (Y - c) spans m + q + k
    // bits; the shear moves a point by no more than 2^-10 cells at the top
    // and the bottom row (k = m - 1 + 10), which leaves q = 44 - 2m bits for
    // the step (22 on a grid of 2000 cells: 2048 values of the move).
    const auto span = static_cast<std::uint64_t>(std::max(width, height)) + 2U;
    const auto m = static_cast<int>(std::bit_width(span));
    const int q = std::max(44 - (2 * m), 12);
    const int k = 53 - m - q;
    step = std::ldexp(1.0, -q);
    inverse = std::ldexp(1.0, q);
    shear = std::ldexp(1.0, -k);
    middle = std::ldexp(1.0, m - 1);
    jitter = std::ldexp(1.0, -11);
  }
  [[nodiscard]] double snap(const double v) const {
    return std::nearbyint(v * inverse) * step;
  }
  [[nodiscard]] double moved(const double v, const u64 seed) const {
    const auto clean = v == 0.0 ? 0.0 : v;
    const auto bits = splitMix(std::bit_cast<u64>(clean) ^ seed);
    const auto unit = (static_cast<double>(bits & 0xffffffU) / 16777215.0) - 0.5;
    return snap(clean + (unit * jitter));
  }
  [[nodiscard]] XY of(const XY p) const {
    // Two seeds for the two hashes (any two do; these give `DISABLED_Steps`
    // the gates of `cdtGates` as often as the old move of both coordinates).
    const auto x = moved(p.x, 0x63cfc62a2b097592ULL);
    const auto y = moved(p.y, 0xd35b78b9930a2bdULL);
    return {x + (shear * (y - middle)), y};
  }
  /// The X of a placed point: what every point of its column shares.
  [[nodiscard]] double columnOf(const double x, const double y) const {
    return x - (shear * (y - middle));
  }
  /// The point of column X at about height y, exactly on the column.
  [[nodiscard]] XY onColumn(const double column, const double y) const {
    const auto ys = snap(y);
    return {column + (shear * (ys - middle)), ys};
  }
};

/// CDT's point locator for the states: the walk to a new point starts at
/// the point inserted before it. The points of a layer come in the order
/// of its loops, each within a clearance of the one before, so the walk is
/// a few triangles long; CDT's own locator (a KD-tree of every vertex,
/// asked for the nearest one per point, copied with every state) costs more
/// than it saves here.
struct LastPointLocator {
  CDT::VertInd last = 0;
  bool ready = false;
  void initialize(const std::vector<CDT::V2d<double>>& points) {
    ready = !points.empty();
    last = 0;
  }
  void addPoint(const CDT::VertInd i, const std::vector<CDT::V2d<double>>&) {
    last = i;
  }
  [[nodiscard]] CDT::VertInd nearPoint(const CDT::V2d<double>&,
                                       const std::vector<CDT::V2d<double>>&) const {
    return last;
  }
  [[nodiscard]] CDT::VertInd size() const { return ready ? CDT::VertInd(1) : CDT::VertInd(0); }
  [[nodiscard]] bool empty() const { return !ready; }
};
using Tri = CDT::Triangulation<double, LastPointLocator>;

/// A grid over a box that holds a value per cell, allocated in chunks of
/// CHUNK by CHUNK cells where cells are written: the stamp of a thin
/// diagonal object costs its cells, not its box.
template <class T> struct ChunkedGrid {
  static constexpr std::int64_t CHUNK = 16;
  std::int64_t x0 = 0;
  std::int64_t y0 = 0;
  std::int64_t cw = 0;
  std::int64_t ch = 0;
  std::vector<u32> chunkOf;
  std::vector<T> data;
  T blank{};
  void reset(const std::int64_t bx0, const std::int64_t by0, const std::int64_t bx1,
             const std::int64_t by1, const T empty) {
    x0 = bx0;
    y0 = by0;
    cw = bx1 < bx0 ? 0 : ((bx1 - bx0) / CHUNK) + 1;
    ch = by1 < by0 ? 0 : ((by1 - by0) / CHUNK) + 1;
    chunkOf.assign(static_cast<std::size_t>(cw * ch), NONE);
    data.clear();
    blank = empty;
  }
  [[nodiscard]] bool inside(const std::int64_t x, const std::int64_t y) const {
    return x >= x0 && y >= y0 && x < x0 + (cw * CHUNK) && y < y0 + (ch * CHUNK);
  }
  [[nodiscard]] T get(const std::int64_t x, const std::int64_t y) const {
    if (!inside(x, y)) {
      return blank;
    }
    const auto c = chunkOf[static_cast<std::size_t>((((y - y0) / CHUNK) * cw) + ((x - x0) / CHUNK))];
    if (c == NONE) {
      return blank;
    }
    return data[(static_cast<std::size_t>(c) * CHUNK * CHUNK) +
                static_cast<std::size_t>((((y - y0) % CHUNK) * CHUNK) + ((x - x0) % CHUNK))];
  }
  T& at(const std::int64_t x, const std::int64_t y) {
    auto& c = chunkOf[static_cast<std::size_t>((((y - y0) / CHUNK) * cw) + ((x - x0) / CHUNK))];
    if (c == NONE) {
      c = static_cast<u32>(data.size() / (CHUNK * CHUNK));
      data.resize(data.size() + (CHUNK * CHUNK), blank);
    }
    return data[(static_cast<std::size_t>(c) * CHUNK * CHUNK) +
                static_cast<std::size_t>((((y - y0) % CHUNK) * CHUNK) + ((x - x0) % CHUNK))];
  }
};

/// Puts the vertex CDT adds where the constraint it is inserting crosses
/// another exactly onto that constraint's row or column (where it has one),
/// the moment it is added. CDT computes the crossing in floating point, a
/// few ulps off the row or column; the rest of the constraint, walked from
/// that vertex, would then run a hair beside every vertex that lies exactly
/// on the line, cross the pieces that end there at their very end, and CDT
/// could not resolve that crossing ("computed split vertex is invalid").
/// `canonicalSteiner` places the vertex canonically afterwards; this keeps
/// the insertion itself on the line.
struct Snapper final : CDT::ICallbackHandler {
  Tri* cdt = nullptr;
  const Placement* placement = nullptr;
  CDT::Edge edge{0, 0};
  bool haveEdge = false;
  void onAddEdgeStart(const CDT::Edge& e) override {
    edge = e;
    haveEdge = true;
  }
  void onAddVertexStart(const CDT::VertInd v,
                        const CDT::AddVertexType::Enum type) override {
    if (type != CDT::AddVertexType::FixedEdgesIntersection || !haveEdge ||
        v >= cdt->vertices.size() || edge.v1() >= cdt->vertices.size() ||
        edge.v2() >= cdt->vertices.size()) {
      return;
    }
    const auto& a = cdt->vertices[edge.v1()];
    const auto& b = cdt->vertices[edge.v2()];
    auto& p = cdt->vertices[v];
    if (a.y == b.y) {
      p.y = a.y;
      return;
    }
    const auto column = placement->columnOf(a.x, a.y);
    if (column == placement->columnOf(b.x, b.y)) {
      // On the column exactly: at a height on the placement's grid, so that
      // the shear is exact, as for every point placed on it.
      const auto on = placement->onColumn(column, p.y);
      p.x = on.x;
      p.y = on.y;
    }
  }
};

/// Sets a triangulation's callback handler for a scope.
struct SnapScope {
  Tri& cdt;
  Snapper snapper;
  SnapScope(Tri& t, const Placement& placement) : cdt(t) {
    snapper.cdt = &t;
    snapper.placement = &placement;
    cdt.setCallbackHandler(&snapper);
  }
  ~SnapScope() { cdt.setCallbackHandler(nullptr); }
  SnapScope(const SnapScope&) = delete;
  SnapScope& operator=(const SnapScope&) = delete;
};

/// The internal walls or groups a point touches. Six are room for every
/// point a raster makes; the list stays in the order they were added.
struct Groups {
  std::array<u32, 6> of{};
  std::uint8_t count = 0;

  void add(const u32 group) {
    for (std::uint8_t k = 0; k < count; ++k) {
      if (of[k] == group) {
        return;
      }
    }
    if (count < of.size()) {
      of[count++] = group;
    }
  }
  void add(const Groups& other) {
    for (std::uint8_t k = 0; k < other.count; ++k) {
      add(other.of[k]);
    }
  }
  [[nodiscard]] bool has(const u32 group) const {
    for (std::uint8_t k = 0; k < count; ++k) {
      if (of[k] == group) {
        return true;
      }
    }
    return false;
  }
  [[nodiscard]] bool meets(const Groups& other) const {
    for (std::uint8_t k = 0; k < count; ++k) {
      if (other.has(of[k])) {
        return true;
      }
    }
    return false;
  }
};

/// The cosine of the angle between a line and the nearest of the eight
/// headings a wire takes (`cdtGates`).
double headingShareOf(const double dx, const double dy) {
  if (std::hypot(dx, dy) < 1e-9) {
    return 1.0;
  }
  constexpr double EIGHTH = std::numbers::pi / 4.0;
  const auto angle = std::atan2(dx, -dy);
  const auto folded = std::fmod(std::fmod(angle, EIGHTH) + EIGHTH, EIGHTH);
  return std::cos(std::min(folded, EIGHTH - folded));
}

/// Douglas-Peucker on one run of the outline, its two ends kept
/// (`cdtGates`).
void simplifyRun(const std::vector<XY>& in, const double tolerance,
                 std::vector<XY>& out) {
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
      const auto distance = length < 1e-12
                                ? std::hypot(px, py)
                                : std::abs((dx * py) - (dy * px)) / length;
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
/// `less` ranks first (`cdtGates`).
struct MinSets {
  std::vector<u32> parent;
  std::vector<u32> least;

  explicit MinSets(const std::size_t count) : parent(count), least(count) {
    std::iota(parent.begin(), parent.end(), 0U);
    std::iota(least.begin(), least.end(), 0U);
  }
  u32 find(u32 at) {
    while (parent[at] != at) {
      parent[at] = parent[parent[at]];
      at = parent[at];
    }
    return at;
  }
  template <typename Less>
  void join(const u32 a, const u32 b, const Less& less) {
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

/// One corner of an outline: where it is, the wall of the piece from it on
/// and the wall of the piece before it (internal wall ids).
struct Corner {
  double x = 0.0;
  double y = 0.0;
  u32 wall = 0;
  u32 prev = 0;
};
using Loop = std::vector<Corner>;

constexpr std::array<int, 4> DIR_X = {1, 0, -1, 0};
constexpr std::array<int, 4> DIR_Y = {0, 1, 0, -1};

/// The outline of the walls in a box of the grid, as steps 1 to 3 of
/// `cdtGates` find it: every edge between a free cell and a wall cell, the
/// free cell to its left, joined into loops that turn left where two walls
/// touch at a corner, each run of one wall simplified, and every piece cut
/// to no more than a clearance. `isWall` and `labelAt` take any cell; the
/// loops run inside the box.
template <class IsWall, class LabelAt>
std::vector<Loop> outlineOf(const std::int64_t bx0, const std::int64_t by0,
                            const std::int64_t bx1, const std::int64_t by1,
                            const double simplify, const double clearance,
                            const IsWall& isWall, const LabelAt& labelAt,
                            const std::vector<std::int64_t>* free = nullptr) {
  // `free`: the free cells next to the walls, as indices of the box in row
  // order, when the caller knows them. Only those can have a crack, and in
  // that order they give the cracks the scan of the whole box gives; a long
  // thin wall then costs its own cells, not its box.
  const auto bw = bx1 - bx0 + 1;
  const auto bh = by1 - by0 + 1;
  struct Crack {
    std::int64_t fx;
    std::int64_t fy;
    std::int64_t cx;
    std::int64_t cy;
    std::uint8_t dir;
    u32 wall;
    bool used;
  };
  std::vector<u32> firstOf(free == nullptr ? static_cast<std::size_t>(bw * bh) : 0,
                           NONE);
  // With `free`: the first crack of a cell, kept where the free cells are.
  ChunkedGrid<u32> firstOfFree;
  if (free != nullptr) {
    firstOfFree.reset(bx0, by0, bx1, by1, NONE);
  }
  std::vector<Crack> cracks;
  const auto crackCell = [&](const std::int64_t x, const std::int64_t y,
                             const std::size_t /*slot*/) {
      if (isWall(x, y)) {
        return;
      }
      const auto first = static_cast<u32>(cracks.size());
      if (isWall(x, y - 1)) {
        cracks.push_back({x, y, x, y, 0, labelAt(x, y - 1), false});
      }
      if (isWall(x + 1, y)) {
        cracks.push_back({x + 1, y, x, y, 1, labelAt(x + 1, y), false});
      }
      if (isWall(x, y + 1)) {
        cracks.push_back({x + 1, y + 1, x, y, 2, labelAt(x, y + 1), false});
      }
      if (isWall(x - 1, y)) {
        cracks.push_back({x, y + 1, x, y, 3, labelAt(x - 1, y), false});
      }
      if (cracks.size() > first) {
        if (free == nullptr) {
          firstOf[static_cast<std::size_t>(((y - by0) * bw) + (x - bx0))] = first;
        } else {
          firstOfFree.at(x, y) = first;
        }
      }
  };
  if (free == nullptr) {
    for (auto y = by0; y <= by1; ++y) {
      for (auto x = bx0; x <= bx1; ++x) {
        crackCell(x, y, 0);
      }
    }
  } else {
    for (std::size_t k = 0; k < free->size(); ++k) {
      const auto at = (*free)[k];
      crackCell(bx0 + (at % bw), by0 + (at / bw), k);
    }
  }
  const auto crackAt = [&](const std::int64_t x, const std::int64_t y,
                           const std::uint8_t side) -> std::size_t {
    if (x < bx0 || y < by0 || x > bx1 || y > by1) {
      return cracks.size();
    }
    u32 first = NONE;
    if (free == nullptr) {
      first = firstOf[static_cast<std::size_t>(((y - by0) * bw) + (x - bx0))];
    } else {
      first = firstOfFree.get(x, y);
    }
    if (first == NONE) {
      return cracks.size();
    }
    for (auto k = static_cast<std::size_t>(first);
         k < cracks.size() && k < first + 4U && cracks[k].cx == x &&
         cracks[k].cy == y;
         ++k) {
      if (cracks[k].dir == side) {
        return k;
      }
    }
    return cracks.size();
  };
  struct RawLoop {
    std::vector<XY> points;
    std::vector<u32> walls;
  };
  std::vector<RawLoop> raw;
  for (std::size_t start = 0; start < cracks.size(); ++start) {
    if (cracks[start].used) {
      continue;
    }
    RawLoop loop;
    auto at = start;
    int lastDir = -1;
    u32 lastWall = NONE;
    while (!cracks[at].used) {
      auto& crack = cracks[at];
      crack.used = true;
      if (crack.dir != lastDir || crack.wall != lastWall) {
        loop.points.push_back({static_cast<double>(crack.fx),
                               static_cast<double>(crack.fy)});
        loop.walls.push_back(crack.wall);
        lastDir = crack.dir;
        lastWall = crack.wall;
      }
      const auto d = crack.dir;
      const auto cx = crack.cx;
      const auto cy = crack.cy;
      const auto right = static_cast<std::uint8_t>((d + 3U) % 4U);
      auto next = crackAt(cx, cy, static_cast<std::uint8_t>((d + 1U) % 4U));
      if (next == cracks.size()) {
        next = crackAt(cx + DIR_X[d], cy + DIR_Y[d], d);
      }
      if (next == cracks.size()) {
        next = crackAt(cx + DIR_X[d] + DIR_X[right],
                       cy + DIR_Y[d] + DIR_Y[right], right);
      }
      if (next == cracks.size()) {
        break;
      }
      at = next;
    }
    if (loop.points.size() >= 3) {
      raw.push_back(std::move(loop));
    }
  }
  std::vector<Loop> out;
  std::vector<XY> run;
  std::vector<XY> simple;
  for (const auto& loop : raw) {
    const auto count = loop.points.size();
    std::size_t first = 0;
    for (std::size_t k = 0; k < count; ++k) {
      if (loop.walls[k] != loop.walls[(k + count - 1) % count]) {
        first = k;
        break;
      }
    }
    const bool oneWall = std::ranges::all_of(
        loop.walls, [&](const auto wall) { return wall == loop.walls[0]; });
    std::vector<std::pair<XY, u32>> kept;
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
      run.push_back(loop.points[(first + end) % count]);
      simple.clear();
      simplifyRun(run, simplify, simple);
      for (std::size_t s = 0; s + 1 < simple.size(); ++s) {
        kept.emplace_back(simple[s], wall);
      }
      k = end;
    }
    Loop pieces;
    for (std::size_t s = 0; s < kept.size(); ++s) {
      const auto& [a, wall] = kept[s];
      const auto& b = kept[(s + 1) % kept.size()].first;
      const auto length = std::hypot(b.x - a.x, b.y - a.y);
      const auto cuts = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(length / clearance)));
      for (std::size_t p = 0; p < cuts; ++p) {
        const auto t = static_cast<double>(p) / static_cast<double>(cuts);
        pieces.push_back({a.x + (t * (b.x - a.x)), a.y + (t * (b.y - a.y)),
                          wall, NONE});
      }
    }
    for (std::size_t v = 0; v < pieces.size(); ++v) {
      pieces[v].prev = pieces[(v + pieces.size() - 1) % pieces.size()].wall;
    }
    if (pieces.size() >= 3) {
      out.push_back(std::move(pieces));
    }
  }
  return out;
}

/// The pieces of a set of loops by squares of the grid, for the foot
/// points: a piece is in every square its box meets.
struct PieceIndex {
  double bucket = 1.0;
  std::int64_t nx = 1;
  std::int64_t ny = 1;
  /// Per square, the pieces as (loop, index of its first corner).
  std::vector<std::vector<std::pair<u32, u32>>> squares;

  void build(const std::vector<Loop>& loops, const double size,
             const std::uint32_t width, const std::uint32_t height) {
    bucket = std::max(size, 1.0);
    nx = static_cast<std::int64_t>(
             std::ceil((static_cast<double>(width) + 2.0) / bucket)) +
         1;
    ny = static_cast<std::int64_t>(
             std::ceil((static_cast<double>(height) + 2.0) / bucket)) +
         1;
    squares.assign(static_cast<std::size_t>(nx * ny), {});
    for (u32 l = 0; l < loops.size(); ++l) {
      const auto& loop = loops[l];
      for (u32 v = 0; v < loop.size(); ++v) {
        const auto& a = loop[v];
        const auto& b = loop[(v + 1) % loop.size()];
        const auto x0 = column(std::min(a.x, b.x));
        const auto x1 = column(std::max(a.x, b.x));
        const auto y0 = row(std::min(a.y, b.y));
        const auto y1 = row(std::max(a.y, b.y));
        for (auto y = y0; y <= y1; ++y) {
          for (auto x = x0; x <= x1; ++x) {
            squares[static_cast<std::size_t>((y * nx) + x)].emplace_back(l, v);
          }
        }
      }
    }
  }
  [[nodiscard]] std::int64_t column(const double v) const {
    return std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(v / bucket)), 0, nx - 1);
  }
  [[nodiscard]] std::int64_t row(const double v) const {
    return std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(v / bucket)), 0, ny - 1);
  }
};

/// A foot point: on the piece from corner `index` of loop `loop`, at `t`.
struct Foot {
  u32 loop = 0;
  u32 index = 0;
  double t = 0.0;
};

/// The candidate edges of a state: an edge between two free triangles whose
/// ends lie on two obstacles.
struct Candidate {
  u32 a = 0;
  u32 b = 0;
  u32 t1 = 0;
  u32 t2 = 0;
  u32 cap = 0;
  double width = 0.0;
  /// The obstacle pair, by the ranks of the two groups.
  u64 pairLo = 0;
  u64 pairHi = 0;
  /// The gate cells, once measured.
  u32 first = 0;
  u32 second = 0;
  u32 saddle = 0;
  std::uint8_t gate = 0;
  std::uint8_t measured = 0;
  std::uint8_t valid = 0;
  std::uint8_t cuts = 0;
  /// Whether it cuts the chambers as they are labelled now.
  std::uint8_t sep = 0;
  /// Replaced by a later evaluation of its edge.
  std::uint8_t dead = 0;

  [[nodiscard]] bool samePair(const Candidate& other) const {
    return pairLo == other.pairLo && pairHi == other.pairHi;
  }
};

/// The key of a chamber: the lowest centroid (y, then x) of its triangles.
using ChamberKey = std::pair<double, double>;

struct PointHash {
  std::size_t operator()(const std::pair<u64, u64>& p) const noexcept {
    return static_cast<std::size_t>(splitMix(p.first ^ splitMix(p.second)));
  }
};

} // namespace

/// A state: the triangulation of the base with a set of objects, and what
/// was found on it.
struct CdtWorld::State {
  Tri cdt{CDT::VertexInsertionOrder::AsProvided,
          CDT::IntersectingConstraintEdges::TryResolve, CONSTRAINT_TOLERANCE};
  /// Per vertex of the triangulation, its point of the world (NONE: a point
  /// of the super-triangle or one the triangulation added where two
  /// constraints crossed).
  std::vector<u32> gidOf;
  /// Per point of the world, its vertex (NONE: not in this state).
  std::vector<u32> vidOf;
  /// A triangle at every vertex.
  std::vector<u32> vertTri;
  /// Per vertex, whether it ends a piece of a constraint that is not one of
  /// the layers' edges: a constraint split where a point of another layer
  /// lies on it, or where two constraints cross.
  std::vector<std::uint8_t> splitEnd;
  /// Per object of the world, whether it is in the state.
  std::vector<std::uint8_t> present;
  /// The objects of the state by square of the world's `OBJ_SQUARE` cells:
  /// per square the first node of its list (NONE: none), the nodes being
  /// (object, next). An object stands in every square within a cell of one
  /// of its cells. A child state copies the lists and pushes its own objects
  /// at the heads.
  std::vector<u32> squareHead;
  std::vector<std::pair<u32, u32>> squareNodes;
  /// The objects of the state, sorted.
  std::vector<u32> objects;
  std::vector<std::uint8_t> freeT;
  /// Per edge of a triangle (three per triangle), its candidate or NONE.
  std::vector<u32> slot;
  std::vector<Candidate> pool;
  std::size_t deadCandidates = 0;
  /// The candidates that are gates (measured, valid or not).
  std::vector<u32> gateList;
  /// Per triangle, its chamber (an internal id; NONE: not free).
  std::vector<u32> chamber;
  std::vector<ChamberKey> chamberKey;
  std::vector<std::uint8_t> chamberAlive;
  /// Per internal chamber id, its canonical number (NO_CHAMBER: dead).
  std::vector<u32> chamberRank;
  CdtWorld::Result result;
  CdtStepStats stats;
};

namespace {

/// States no one holds any more, kept to be written over: copying a parent
/// into a state that held a triangulation before reuses its memory, the
/// nodes of the constraint set above all, instead of allocating them anew.
struct StatePool {
  std::vector<std::unique_ptr<CdtWorld::State>> spare;
  std::size_t limit = 4;
};

/// The deleter of a state: back to the pool while the pool is there and
/// has room.
struct Recycle {
  std::weak_ptr<StatePool> pool;
  void operator()(CdtWorld::State* state) const {
    if (const auto keep = pool.lock(); keep && keep->spare.size() < keep->limit) {
      keep->spare.emplace_back(state);
      return;
    }
    std::default_delete<CdtWorld::State>{}(state);
  }
};

} // namespace

struct CdtWorld::Impl {
  // ---------------------------------------------------------------- input
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  CdtGateOptions options;
  /// Where the points go (`Placement`).
  Placement placement;
  BitGrid staticMask;
  std::vector<u32> staticOwner;
  /// The static walls: labels 0 to S - 1, then the outside wall S.
  u32 staticWalls = 0;
  u32 outsideWall = 0;
  u32 borderWall = 0;

  // ---------------------------------------------------------------- walls
  std::vector<u32> wallGroup;
  std::vector<u32> wallObject;
  std::vector<std::uint8_t> wallCuts;
  std::vector<std::uint8_t> groupFixed;
  std::vector<std::uint8_t> groupTarget;
  /// What orders the groups: a function of the walls' labels and the
  /// objects' keys, so the order does not depend on when an object came.
  std::vector<u64> groupRank;

  // ---------------------------------------------------------------- points
  struct Entry {
    u32 layer = 0;
    Groups groups;
    Groups walls;
  };
  std::vector<XY> gpos;
  std::vector<XY> gplaced;
  std::vector<std::vector<Entry>> entries;
  /// The walls whose cells touch a point (on its corner, edge or inside).
  std::vector<std::vector<u32>> touch;
  /// Per point, whether it is a foot laid a hair off a piece already in
  /// place (`layOffsetFeet`).
  std::vector<std::uint8_t> hair;
  /// The constraints at a point: (other point, layer).
  std::vector<std::vector<std::pair<u32, u32>>> constraintsAt;
  std::unordered_map<std::pair<u64, u64>, u32, PointHash> gidIndex;
  /// The points by squares of GID_SQUARE cells (the squares of the
  /// objects, `OBJ_SQUARE`, so that an object relates to the points of its
  /// own squares).
  static constexpr std::int64_t GID_SQUARE = 8;
  std::int64_t gidNx = 1;
  std::int64_t gidNy = 1;
  std::vector<std::vector<u32>> gidSquares;
  /// The objects by squares of OBJ_SQUARE cells: an object is listed in
  /// every square within OBJ_REACH cells of one of its cells, so that the
  /// objects whose stamp, outline or near field can hold a point are found
  /// in the point's square alone, and not by asking every object of a state.
  static constexpr std::int64_t OBJ_SQUARE = 8;
  static constexpr std::int64_t OBJ_REACH = 3;
  std::int64_t objNx = 1;
  std::int64_t objNy = 1;
  std::vector<std::vector<u32>> squareObjects;
  std::vector<std::uint8_t> squareMark;
  /// Per object, the squares within one cell of its cells: those a state
  /// lists it in (`State::squareHead`); and those within OBJ_REACH, as
  /// `squareObjects` lists it.
  std::vector<std::vector<u32>> objectSquares;
  std::vector<std::vector<u32>> reachSquares;
  [[nodiscard]] std::span<const u32> squaresOf(const u32 o) const {
    return reachSquares[o];
  }

  /// The groups and walls of the vertices of the state being built, as
  /// `groupsAt` found them: a vertex is asked for by every edge and gate at
  /// it, many times over. Valid for one state from `beginGroupsCache` on,
  /// once its points stand and its objects are known.
  mutable std::vector<Groups> groupsCache;
  mutable std::vector<Groups> wallsCache;
  mutable std::vector<u32> groupsStamp;
  mutable u32 groupsEpoch = 0;
  mutable const State* groupsState = nullptr;
  void beginGroupsCache(const State& s) const {
    ++groupsEpoch;
    groupsState = &s;
    const auto nv = s.cdt.vertices.size();
    if (groupsCache.size() < nv) {
      groupsCache.resize(nv);
      wallsCache.resize(nv);
      groupsStamp.resize(nv, 0);
    }
  }
  /// Lists `o` in the squares of a state.
  void listInSquares(State& s, const u32 o) const {
    if (s.squareHead.size() != squareObjects.size()) {
      s.squareHead.assign(squareObjects.size(), NONE);
    }
    for (const auto sq : objectSquares[o]) {
      s.squareNodes.emplace_back(o, s.squareHead[sq]);
      s.squareHead[sq] = static_cast<u32>(s.squareNodes.size() - 1);
    }
  }
  /// Calls `f(object)` for every object of a state listed in the square of
  /// a cell.
  template <class F>
  void presentNear(const State& s, const std::int64_t x, const std::int64_t y,
                   const F& f) const {
    const auto sq = squareOf(x, y);
    if (sq >= s.squareHead.size()) {
      return;
    }
    for (auto node = s.squareHead[sq]; node != NONE; node = s.squareNodes[node].second) {
      f(s.squareNodes[node].first);
    }
  }
  [[nodiscard]] std::size_t squareOf(const std::int64_t x, const std::int64_t y) const {
    const auto sx = std::clamp<std::int64_t>(x / OBJ_SQUARE, 0, objNx - 1);
    const auto sy = std::clamp<std::int64_t>(y / OBJ_SQUARE, 0, objNy - 1);
    return static_cast<std::size_t>((sy * objNx) + sx);
  }
  [[nodiscard]] std::span<const u32> objectsNear(const std::int64_t x,
                                                 const std::int64_t y) const {
    return squareObjects[squareOf(x, y)];
  }
  [[nodiscard]] std::span<const u32> objectsNear(const double x, const double y) const {
    return objectsNear(static_cast<std::int64_t>(std::floor(x)),
                       static_cast<std::int64_t>(std::floor(y)));
  }

  // ---------------------------------------------------------------- layers
  /// A layer: the points and constraints that are in a state when its
  /// objects are (none: the base; one: an object; two: the foot points
  /// between two objects).
  struct Layer {
    std::array<u32, 2> objects{NONE, NONE};
    std::vector<u32> gids;
    std::vector<std::pair<u32, u32>> edges;
  };
  std::vector<Layer> layers;

  /// An outline as layers keep it: the corners before the foot points, and
  /// per piece the points it was cut at, from t = 0 on, so that a foot
  /// point on it can be put on the piece as the triangulation has it.
  struct Outline {
    std::vector<Loop> loops;
    std::vector<std::vector<std::vector<std::pair<double, u32>>>> splits;
    PieceIndex index;
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = -1.0;
    double y1 = -1.0;
  };
  Outline baseOutline;
  /// The corners of the base outline by square of CORNER_SQUARE cells, as
  /// (loop, index): `feetOf` the static corners on an object looks at the
  /// corners near the object alone, not at every corner of the chip.
  static constexpr std::int64_t CORNER_SQUARE = 32;
  std::int64_t cornerNx = 1;
  std::int64_t cornerNy = 1;
  std::vector<std::vector<std::pair<u32, u32>>> baseCorners;
  void indexBaseCorners() {
    cornerNx = (static_cast<std::int64_t>(width) / CORNER_SQUARE) + 2;
    cornerNy = (static_cast<std::int64_t>(height) / CORNER_SQUARE) + 2;
    baseCorners.assign(static_cast<std::size_t>(cornerNx * cornerNy), {});
    for (u32 l = 0; l < baseOutline.loops.size(); ++l) {
      const auto& loop = baseOutline.loops[l];
      for (u32 v = 0; v < loop.size(); ++v) {
        const auto sx = std::clamp<std::int64_t>(
            static_cast<std::int64_t>(std::floor(loop[v].x)) / CORNER_SQUARE, 0, cornerNx - 1);
        const auto sy = std::clamp<std::int64_t>(
            static_cast<std::int64_t>(std::floor(loop[v].y)) / CORNER_SQUARE, 0, cornerNy - 1);
        baseCorners[static_cast<std::size_t>((sy * cornerNx) + sx)].emplace_back(l, v);
      }
    }
  }
  /// The feet of the static corners near `onto` on it (`feetOf` of the
  /// base loops, by the corner index).
  [[nodiscard]] std::vector<Foot> baseFeetOn(const Outline& onto) const {
    const auto reach = options.footReach * options.clearance;
    std::vector<Loop> near(1);
    const auto sx0 = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(onto.x0 - reach)) / CORNER_SQUARE, 0, cornerNx - 1);
    const auto sx1 = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(onto.x1 + reach)) / CORNER_SQUARE, 0, cornerNx - 1);
    const auto sy0 = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(onto.y0 - reach)) / CORNER_SQUARE, 0, cornerNy - 1);
    const auto sy1 = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(onto.y1 + reach)) / CORNER_SQUARE, 0, cornerNy - 1);
    for (auto sy = sy0; sy <= sy1; ++sy) {
      for (auto sx = sx0; sx <= sx1; ++sx) {
        for (const auto& [l, v] : baseCorners[static_cast<std::size_t>((sy * cornerNx) + sx)]) {
          near[0].push_back(baseOutline.loops[l][v]);
        }
      }
    }
    return feetOf(near, onto);
  }

  // ---------------------------------------------------------------- objects
  struct Object {
    u64 key = 0;
    u32 firstWall = 0;
    u32 wallCount = 0;
    u32 layer = 0;
    /// The box of its cells and its stamp in it: per cell its wall plus
    /// one, zero where it has none.
    std::int64_t x0 = 0;
    std::int64_t y0 = 0;
    std::int64_t x1 = -1;
    std::int64_t y1 = -1;
    ChunkedGrid<u32> stamp;
    Outline outline;
    /// Its outline as segments of placed points, and their box, for the
    /// inside test.
    std::vector<std::array<double, 4>> segments;
    double sx0 = 0.0;
    double sy0 = 0.0;
    double sx1 = -1.0;
    double sy1 = -1.0;
    /// The points within one cell of its cells.
    std::vector<u32> near;
    /// The layers of foot points between it and another object.
    std::vector<u32> pairs;

    [[nodiscard]] u32 wallAt(const std::int64_t x, const std::int64_t y) const {
      if (x < x0 || y < y0 || x > x1 || y > y1) {
        return NONE;
      }
      const auto v = stamp.get(x, y);
      return v == 0 ? NONE : firstWall + v - 1;
    }
  };
  std::vector<Object> objects;
  std::unordered_map<u64, u32> objectOfKey;

  // ---------------------------------------------------------------- level 0
  /// The triangulation of the static walls alone, with its free triangles
  /// by parity, for the inside test of the static outlines; and per square
  /// of HINT cells a triangle to start the walk from (its indices hold in
  /// every state, whose triangulations only grow from the base's).
  Tri level0{CDT::VertexInsertionOrder::AsProvided,
             CDT::IntersectingConstraintEdges::TryResolve, CONSTRAINT_TOLERANCE};
  std::vector<std::uint8_t> level0Free;
  static constexpr std::int64_t HINT = 16;
  std::int64_t hintNx = 1;
  std::int64_t hintNy = 1;
  std::vector<u32> hint;

  std::shared_ptr<State> baseState;
  std::shared_ptr<StatePool> pool = std::make_shared<StatePool>();

  /// A copy of a state, written over a spare one when there is one.
  [[nodiscard]] std::shared_ptr<State> copyOf(const State& parent) const {
    std::unique_ptr<State> made;
    if (!pool->spare.empty()) {
      made = std::move(pool->spare.back());
      pool->spare.pop_back();
      *made = parent;
    } else {
      made = std::make_unique<State>(parent);
    }
    return {made.release(), Recycle{pool}};
  }

  // ---------------------------------------------------------------- cache
  struct Cached {
    std::vector<u32> objects;
    StatePtr state;
    u64 used = 0;
  };
  std::vector<Cached> cache;
  std::size_t cacheSize = 8;
  u64 clock = 0;
  CdtWorldCacheStats cacheStats;
  BuildStats buildStats;

  // ================================================================ walls

  [[nodiscard]] u32 staticWallOf(const std::uint32_t label) const {
    return label < staticWalls ? label : borderWall;
  }
  [[nodiscard]] u32 groupOfWall(const u32 wall) const { return wallGroup[wall]; }
  [[nodiscard]] bool fixedGroup(const u32 group) const {
    return groupFixed[group] != 0;
  }
  [[nodiscard]] bool targetGroup(const u32 group) const {
    return groupTarget[group] != 0;
  }

  [[nodiscard]] static bool presentIn(const State& s, const u32 object) {
    return object < s.present.size() && s.present[object] != 0;
  }
  [[nodiscard]] bool layerPresent(const State& s, const u32 layer) const {
    const auto& [a, b] = layers[layer].objects;
    return (a == NONE || presentIn(s, a)) && (b == NONE || presentIn(s, b));
  }
  [[nodiscard]] bool wallPresent(const State& s, const u32 wall) const {
    const auto object = wallObject[wall];
    return object == NONE || presentIn(s, object);
  }

  /// Every wall of a cell in a state: the static one, then those of the
  /// objects whose stamps hold it. Cells outside the grid are the border's.
  template <class F>
  void wallsOfCell(const State& s, const std::int64_t x, const std::int64_t y,
                   const F& f) const {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      f(borderWall);
      return;
    }
    const auto cell = static_cast<std::size_t>((y * width) + x);
    if (staticMask.test(cell)) {
      f(staticWallOf(staticOwner[cell]));
    }
    presentNear(s, x, y, [&](const u32 o) {
      const auto wall = objects[o].wallAt(x, y);
      if (wall != NONE) {
        f(wall);
      }
    });
  }
  [[nodiscard]] bool wallCell(const State& s, const std::int64_t x,
                              const std::int64_t y) const {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return true;
    }
    if (staticMask.test(static_cast<std::size_t>((y * width) + x))) {
      return true;
    }
    bool wall = false;
    presentNear(s, x, y, [&](const u32 o) {
      wall = wall || objects[o].wallAt(x, y) != NONE;
    });
    return wall;
  }

  /// The cells whose square lies within `r` of a point.
  template <class F>
  static void cellsNear(const double px, const double py, const double r,
                        const F& f) {
    const auto x0 = static_cast<std::int64_t>(std::floor(px - r - 1e-9));
    const auto x1 = static_cast<std::int64_t>(std::floor(px + r + 1e-9));
    const auto y0 = static_cast<std::int64_t>(std::floor(py - r - 1e-9));
    const auto y1 = static_cast<std::int64_t>(std::floor(py + r + 1e-9));
    for (auto y = y0; y <= y1; ++y) {
      for (auto x = x0; x <= x1; ++x) {
        const double dx = std::max({static_cast<double>(x) - px, 0.0,
                                    px - static_cast<double>(x + 1)});
        const double dy = std::max({static_cast<double>(y) - py, 0.0,
                                    py - static_cast<double>(y + 1)});
        if (std::hypot(dx, dy) <= r + 1e-9) {
          f(x, y);
        }
      }
    }
  }

  // ================================================================ points

  /// The point at a placed position, made if new, with the groups and walls
  /// of one layer.
  u32 pointAt(const XY unmoved, const XY placed, const u32 layer,
              const Groups& groups, const Groups& walls) {
    const std::pair key{std::bit_cast<u64>(placed.x),
                        std::bit_cast<u64>(placed.y)};
    auto [it, fresh] = gidIndex.try_emplace(key, static_cast<u32>(gpos.size()));
    const auto g = it->second;
    if (fresh) {
      gpos.push_back(unmoved);
      gplaced.push_back(placed);
      entries.emplace_back();
      touch.emplace_back();
      hair.push_back(0);
      constraintsAt.emplace_back();
      registerPoint(g);
    }
    auto& list = entries[g];
    auto found = std::ranges::find_if(
        list, [&](const Entry& entry) { return entry.layer == layer; });
    if (found == list.end()) {
      list.push_back({layer, groups, walls});
      layers[layer].gids.push_back(g);
    } else {
      found->groups.add(groups);
      found->walls.add(walls);
    }
    return g;
  }

  /// A new point: the walls whose cells touch it, and the objects it lies
  /// within a cell of.
  void registerPoint(const u32 g) {
    const auto p = gpos[g];
    auto& list = touch[g];
    const auto add = [&](const u32 wall) {
      if (std::ranges::find(list, wall) == list.end()) {
        list.push_back(wall);
      }
    };
    cellsNear(p.x, p.y, 0.0, [&](const std::int64_t x, const std::int64_t y) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        add(borderWall);
      } else if (const auto cell = static_cast<std::size_t>((y * width) + x);
                 staticMask.test(cell)) {
        add(staticWallOf(staticOwner[cell]));
      }
    });
    for (const auto o : objectsNear(p.x, p.y)) {
      relate(g, o);
    }
    const auto sx = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(p.x)) / GID_SQUARE, 0, gidNx - 1);
    const auto sy = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(p.y)) / GID_SQUARE, 0, gidNy - 1);
    gidSquares[static_cast<std::size_t>((sy * gidNx) + sx)].push_back(g);
  }

  /// A point and an object: the walls of its cells that touch the point, and
  /// whether the point lies within a cell of them.
  /// Milliseconds of `addObject` by section, summed (for profiling).
  std::array<double, 10> addSeconds{};
  /// The objects whose `near` took a point since it was last put in order.
  std::vector<std::uint8_t> nearDirty;
  std::vector<u32> nearTouched;

  void relate(const u32 g, const u32 o) {
    auto& object = objects[o];
    const auto p = gpos[g];
    if (p.x < static_cast<double>(object.x0) - 2.0 ||
        p.y < static_cast<double>(object.y0) - 2.0 ||
        p.x > static_cast<double>(object.x1) + 3.0 ||
        p.y > static_cast<double>(object.y1) + 3.0) {
      return;
    }
    auto& list = touch[g];
    cellsNear(p.x, p.y, 0.0, [&](const std::int64_t x, const std::int64_t y) {
      const auto wall = object.wallAt(x, y);
      if (wall != NONE && std::ranges::find(list, wall) == list.end()) {
        list.push_back(wall);
      }
    });
    bool near = false;
    cellsNear(p.x, p.y, 1.0, [&](const std::int64_t x, const std::int64_t y) {
      near = near || object.wallAt(x, y) != NONE;
    });
    if (near) {
      // Only a list that took a point out of order is put in order again
      // (`addObject`): a new point has the highest index there is, so a list
      // that takes new points stays in order as it grows.
      if (!object.near.empty() && g <= object.near.back()) {
        if (nearDirty.size() <= o) {
          nearDirty.resize(objects.size(), 0);
        }
        if (nearDirty[o] == 0) {
          nearDirty[o] = 1;
          nearTouched.push_back(o);
        }
      }
      object.near.push_back(g);
    }
  }

  // ================================================================ outlines

  /// The foot points of the corners of `from` on the pieces of `onto`, as
  /// step 4 of `cdtGates` finds them: per corner and per obstacle of `onto`
  /// within `footReach` clearances, the nearest point of its outline, unless
  /// both are static or of one obstacle.
  [[nodiscard]] std::vector<Foot> feetOf(const std::vector<Loop>& from,
                                         const Outline& onto) const {
    std::vector<Foot> feet;
    const auto reach = options.footReach * options.clearance;
    struct Best {
      u32 group;
      double distance;
      Foot foot;
    };
    std::vector<Best> best;
    for (const auto& loop : from) {
      for (const auto& p : loop) {
        if (p.x < onto.x0 - reach || p.y < onto.y0 - reach ||
            p.x > onto.x1 + reach || p.y > onto.y1 + reach) {
          continue;
        }
        Groups mine;
        mine.add(groupOfWall(p.wall));
        mine.add(groupOfWall(p.prev));
        bool mineStatic = true;
        for (std::uint8_t g = 0; g < mine.count; ++g) {
          mineStatic = mineStatic && fixedGroup(mine.of[g]);
        }
        best.clear();
        const auto cx = onto.index.column(p.x);
        const auto cy = onto.index.row(p.y);
        for (auto by = std::max<std::int64_t>(cy - 1, 0);
             by <= std::min(cy + 1, onto.index.ny - 1); ++by) {
          for (auto bx = std::max<std::int64_t>(cx - 1, 0);
               bx <= std::min(cx + 1, onto.index.nx - 1); ++bx) {
            for (const auto& [l, v] :
                 onto.index.squares[static_cast<std::size_t>(
                     (by * onto.index.nx) + bx)]) {
              const auto& piece = onto.loops[l];
              const auto& a = piece[v];
              const auto& b = piece[(v + 1) % piece.size()];
              const auto group = groupOfWall(a.wall);
              if (mine.has(group) || (mineStatic && fixedGroup(group))) {
                continue;
              }
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
                best.push_back({group, distance, {l, v, t}});
              } else if (distance < found->distance ||
                         (distance == found->distance &&
                          std::tie(l, v) < std::tie(found->foot.loop,
                                                    found->foot.index))) {
                *found = {group, distance, {l, v, t}};
              }
            }
          }
        }
        for (const auto& one : best) {
          feet.push_back(one.foot);
        }
      }
    }
    return feet;
  }

  /// The loops of an outline as constraints of a layer, every piece cut at
  /// its foot points (`splits` of the outline are filled).
  void layLoops(const u32 layer, Outline& outline, const std::vector<Foot>& feet) {
    auto& lay = layers[layer];
    std::vector<std::vector<std::vector<double>>> at(outline.loops.size());
    for (std::size_t l = 0; l < outline.loops.size(); ++l) {
      at[l].resize(outline.loops[l].size());
    }
    for (const auto& foot : feet) {
      at[foot.loop][foot.index].push_back(foot.t);
    }
    outline.splits.assign(outline.loops.size(), {});
    for (std::size_t l = 0; l < outline.loops.size(); ++l) {
      const auto& loop = outline.loops[l];
      auto& splits = outline.splits[l];
      splits.resize(loop.size());
      std::vector<u32> chain;
      for (std::size_t v = 0; v < loop.size(); ++v) {
        const auto& a = loop[v];
        const auto& b = loop[(v + 1) % loop.size()];
        Groups groups;
        groups.add(groupOfWall(a.wall));
        groups.add(groupOfWall(a.prev));
        Groups walls;
        walls.add(a.wall);
        walls.add(a.prev);
        const XY p{a.x, a.y};
        const auto g = pointAt(p, placement.of(p), layer, groups, walls);
        chain.push_back(g);
        splits[v].emplace_back(0.0, g);
        auto& ts = at[l][v];
        std::ranges::sort(ts);
        double last = -1.0;
        for (const auto t : ts) {
          if (t - last < 1e-3) {
            continue;
          }
          last = t;
          Groups fg;
          fg.add(groupOfWall(a.wall));
          Groups fw;
          fw.add(a.wall);
          const XY q{a.x + (t * (b.x - a.x)), a.y + (t * (b.y - a.y))};
          const auto f = pointAt(q, placement.of(q), layer, fg, fw);
          chain.push_back(f);
          splits[v].emplace_back(t, f);
        }
      }
      std::vector<u32> clean;
      for (const auto g : chain) {
        if (clean.empty() || clean.back() != g) {
          clean.push_back(g);
        }
      }
      while (clean.size() > 1 && clean.front() == clean.back()) {
        clean.pop_back();
      }
      if (clean.size() < 3) {
        continue;
      }
      for (std::size_t k = 0; k < clean.size(); ++k) {
        const auto a = clean[k];
        const auto b = clean[(k + 1) % clean.size()];
        lay.edges.emplace_back(a, b);
        constraintsAt[a].emplace_back(b, layer);
        constraintsAt[b].emplace_back(a, layer);
      }
    }
  }

  /// The foot points of a layer that lie on constraints already in place:
  /// each a point of its own, a hair off the piece on its free side, on the
  /// piece as the triangulation has it (between the points it was cut at).
  void layOffsetFeet(const u32 layer, const Outline& onto,
                     const std::vector<Foot>& feet) {
    std::vector<Foot> sorted = feet;
    std::ranges::sort(sorted, [](const Foot& a, const Foot& b) {
      return std::tie(a.loop, a.index, a.t) < std::tie(b.loop, b.index, b.t);
    });
    double last = -1.0;
    u32 lastLoop = NONE;
    u32 lastIndex = NONE;
    for (const auto& foot : sorted) {
      if (foot.loop == lastLoop && foot.index == lastIndex &&
          foot.t - last < 1e-3) {
        continue;
      }
      last = foot.t;
      lastLoop = foot.loop;
      lastIndex = foot.index;
      const auto& loop = onto.loops[foot.loop];
      const auto& a = loop[foot.index];
      const auto& b = loop[(foot.index + 1) % loop.size()];
      const auto& cuts = onto.splits[foot.loop][foot.index];
      // The stretch of the piece between the points it was cut at.
      std::size_t k = 0;
      while (k + 1 < cuts.size() && cuts[k + 1].first <= foot.t) {
        ++k;
      }
      const auto t0 = cuts[k].first;
      const auto g0 = cuts[k].second;
      const auto t1 = k + 1 < cuts.size() ? cuts[k + 1].first : 1.0;
      const auto g1 = k + 1 < cuts.size()
                          ? cuts[k + 1].second
                          : onto.splits[foot.loop][(foot.index + 1) % loop.size()]
                                .front()
                                .second;
      const auto& p0 = gplaced[g0];
      const auto& p1 = gplaced[g1];
      const auto dx = p1.x - p0.x;
      const auto dy = p1.y - p0.y;
      const auto length = std::hypot(dx, dy);
      if (length < 1e-9 || t1 - t0 < 1e-12) {
        continue;
      }
      const auto s = (foot.t - t0) / (t1 - t0);
      const XY unmoved{a.x + (foot.t * (b.x - a.x)),
                          a.y + (foot.t * (b.y - a.y))};
      // A hair off the piece, by an amount of its own: feet at one distance
      // from a piece, placed symmetrically on it, would lie with its two ends
      // on one circle.
      const auto offset =
          FOOT_OFFSET *
          (1.0 + (static_cast<double>(
                      splitMix(std::bit_cast<u64>(unmoved.x) ^
                               splitMix(std::bit_cast<u64>(unmoved.y))) >>
                      11U) *
                  0x1.0p-53));
      const XY placed{p0.x + (s * dx) - (dy / length * offset),
                         p0.y + (s * dy) + (dx / length * offset)};
      Groups groups;
      groups.add(groupOfWall(a.wall));
      Groups walls;
      walls.add(a.wall);
      hair[pointAt(unmoved, placed, layer, groups, walls)] = 1;
    }
  }

  void boxOf(Outline& outline) const {
    outline.x0 = std::numeric_limits<double>::max();
    outline.y0 = std::numeric_limits<double>::max();
    outline.x1 = std::numeric_limits<double>::lowest();
    outline.y1 = std::numeric_limits<double>::lowest();
    for (const auto& loop : outline.loops) {
      for (const auto& p : loop) {
        outline.x0 = std::min(outline.x0, p.x);
        outline.y0 = std::min(outline.y0, p.y);
        outline.x1 = std::max(outline.x1, p.x);
        outline.y1 = std::max(outline.y1, p.y);
      }
    }
    outline.index.build(outline.loops, options.footReach * options.clearance,
                        width, height);
  }

  // ================================================================ build

  void build(const BitGrid& mask, std::span<const std::uint32_t> owner,
             std::span<const CdtWall> walls, const std::uint32_t border,
             std::span<const std::uint8_t> cuts) {
    if (mask.width() != width || mask.height() != height ||
        owner.size() != static_cast<std::size_t>(width) * height) {
      throw std::invalid_argument("CdtWorld: the static walls do not fit the grid");
    }
    if (!(options.clearance > 0.0)) {
      throw std::invalid_argument("CdtWorld: the clearance must be positive");
    }
    const auto began = Clock::now();
    placement.fit(width, height);
    staticMask = mask;
    staticOwner.assign(owner.begin(), owner.end());
    staticWalls = static_cast<u32>(walls.size());
    outsideWall = staticWalls;
    borderWall = border < staticWalls ? border : outsideWall;
    // The static walls and groups: a group past the list is the outside's.
    wallGroup.resize(staticWalls + 1);
    wallObject.assign(staticWalls + 1, NONE);
    wallCuts.assign(staticWalls + 1, 1);
    groupFixed.assign(staticWalls + 1, 1);
    groupTarget.assign(staticWalls + 1, 0);
    groupRank.resize(staticWalls + 1);
    std::iota(groupRank.begin(), groupRank.end(), u64{0});
    for (u32 label = 0; label < staticWalls; ++label) {
      const auto& wall = walls[label];
      const auto group = wall.group < staticWalls ? wall.group : outsideWall;
      wallGroup[label] = group;
      if (group != outsideWall) {
        groupFixed[group] = wall.fixed ? 1 : 0;
        groupTarget[group] = wall.target ? 1 : 0;
      }
      wallCuts[label] = cuts.empty() || (label < cuts.size() && cuts[label] != 0)
                            ? 1
                            : 0;
    }
    wallGroup[outsideWall] = outsideWall;
    gidNx = (static_cast<std::int64_t>(width) / GID_SQUARE) + 1;
    gidNy = (static_cast<std::int64_t>(height) / GID_SQUARE) + 1;
    gidSquares.assign(static_cast<std::size_t>(gidNx * gidNy), {});
    objNx = (static_cast<std::int64_t>(width) / OBJ_SQUARE) + 1;
    objNy = (static_cast<std::int64_t>(height) / OBJ_SQUARE) + 1;
    squareObjects.assign(static_cast<std::size_t>(objNx * objNy), {});
    squareMark.assign(squareObjects.size(), 0);

    // The outline of the static walls, every piece cut.
    const auto w = static_cast<std::int64_t>(width);
    const auto h = static_cast<std::int64_t>(height);
    baseOutline.loops = outlineOf(
        0, 0, w - 1, h - 1, options.simplify, options.clearance,
        [&](const std::int64_t x, const std::int64_t y) {
          return x < 0 || y < 0 || x >= w || y >= h ||
                 staticMask.test(static_cast<std::size_t>((y * w) + x));
        },
        [&](const std::int64_t x, const std::int64_t y) {
          if (x < 0 || y < 0 || x >= w || y >= h) {
            return borderWall;
          }
          return staticWallOf(staticOwner[static_cast<std::size_t>((y * w) + x)]);
        });
    boxOf(baseOutline);
    indexBaseCorners();
    const auto outlined = Clock::now();
    buildStats.outline = msSince(began, outlined);
    // The foot points among the static walls, as constraints of the base.
    const auto feet = feetOf(baseOutline.loops, baseOutline);
    buildStats.feet = msSince(outlined, Clock::now());
    layers.push_back({});
    layLoops(0, baseOutline, feet);
    buildStats.baseVertices = layers[0].gids.size();
    buildStats.baseConstraints = layers[0].edges.size();

    const auto based = Clock::now();
    buildLevel0();
    baseState = scratch({});
    buildStats.base = msSince(based, Clock::now());
    buildStats.points = gpos.size();
  }

  /// The triangulation of the static walls, the parity of its triangles and
  /// the hints of the walk.
  void buildLevel0() {
    std::vector<CDT::V2d<double>> points;
    std::vector<u32> vid(gpos.size(), NONE);
    for (const auto g : layers[0].gids) {
      vid[g] = static_cast<u32>(points.size());
      points.emplace_back(gplaced[g].x, gplaced[g].y);
    }
    std::vector<CDT::Edge> edges;
    for (const auto& [a, b] : layers[0].edges) {
      edges.emplace_back(vid[a], vid[b]);
    }
    level0.insertVertices(points);
    level0.insertEdges(edges);
    const auto n = level0.triangles.size();
    std::vector<u32> depth(n, NONE);
    std::vector<u32> now;
    std::vector<u32> later;
    for (u32 t = 0; t < n; ++t) {
      const auto& v = level0.triangles[t].vertices;
      if (v[0] < SUPER || v[1] < SUPER || v[2] < SUPER) {
        depth[t] = 0;
        now.push_back(t);
      }
    }
    // Crossing a constraint that k outlines share flips the parity k times.
    const auto crossings = [&](const CDT::Edge& edge) -> u32 {
      if (!level0.fixedEdges.contains(edge)) {
        return 0;
      }
      const auto found = level0.overlapCount.find(edge);
      return 1U + (found == level0.overlapCount.end()
                       ? 0U
                       : static_cast<u32>(found->second));
    };
    u32 level = 0;
    while (!now.empty()) {
      for (std::size_t i = 0; i < now.size(); ++i) {
        const auto t = now[i];
        const auto& tr = level0.triangles[t];
        for (std::size_t k = 0; k < 3; ++k) {
          const auto o = tr.neighbors[k];
          if (o == CDT::noNeighbor || depth[o] != NONE) {
            continue;
          }
          const auto c = crossings(CDT::Edge(tr.vertices[k], tr.vertices[(k + 1) % 3]));
          if (c % 2 == 1) {
            later.push_back(o);
          } else {
            depth[o] = level;
            now.push_back(o);
          }
        }
      }
      now.clear();
      ++level;
      for (const auto t : later) {
        if (depth[t] == NONE) {
          depth[t] = level;
          now.push_back(t);
        }
      }
      later.clear();
    }
    level0Free.resize(n);
    for (u32 t = 0; t < n; ++t) {
      level0Free[t] = depth[t] != NONE && depth[t] % 2 == 1 ? 1 : 0;
    }
    hintNx = (static_cast<std::int64_t>(width) / HINT) + 1;
    hintNy = (static_cast<std::int64_t>(height) / HINT) + 1;
    hint.resize(static_cast<std::size_t>(hintNx * hintNy));
    u32 t = 0;
    for (std::int64_t y = 0; y < hintNy; ++y) {
      for (std::int64_t x = 0; x < hintNx; ++x) {
        t = locate(level0, (static_cast<double>(x) + 0.5) * HINT,
                   (static_cast<double>(y) + 0.5) * HINT, t);
        hint[static_cast<std::size_t>((y * hintNx) + x)] = t;
      }
    }
  }

  /// The triangle of a triangulation that holds a point, by a walk from a
  /// triangle.
  static u32 locate(const Tri& cdt, const double x,
                    const double y, u32 t) {
    const auto& tris = cdt.triangles;
    const auto& vs = cdt.vertices;
    const CDT::V2d<double> p(x, y);
    for (std::size_t it = 0; it < 1000000; ++it) {
      const auto& tr = tris[t];
      bool moved = false;
      for (std::size_t j = 0; j < 3; ++j) {
        const auto k = (j + it) % 3;
        if (CDT::orient2D(p, vs[tr.vertices[k]], vs[tr.vertices[(k + 1) % 3]]) <
                0.0 &&
            tr.neighbors[k] != CDT::noNeighbor) {
          t = tr.neighbors[k];
          moved = true;
          break;
        }
      }
      if (!moved) {
        return t;
      }
    }
    return t;
  }
  [[nodiscard]] u32 hintAt(const double x, const double y) const {
    const auto hx = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(x)) / HINT, 0, hintNx - 1);
    const auto hy = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor(y)) / HINT, 0, hintNy - 1);
    return hint[static_cast<std::size_t>((hy * hintNx) + hx)];
  }
  [[nodiscard]] bool staticFreeAt(const double x, const double y) const {
    return level0Free[locate(level0, x, y, hintAt(x, y))] != 0;
  }

  // ================================================================ objects

  u32 addObject(const CdtObject& spec) {
    if (const auto found = objectOfKey.find(spec.key); found != objectOfKey.end()) {
      return found->second;
    }
    if (spec.cells.size() != spec.wallOf.size()) {
      throw std::invalid_argument("CdtWorld: an object needs a wall per cell");
    }
    auto lapAt = Clock::now();
    const auto addLap = [&](const int slot) {
      const auto now = Clock::now();
      if (slot > 0) {
        addSeconds[static_cast<std::size_t>(slot - 1)] += msSince(lapAt, now);
      }
      lapAt = now;
    };
    const auto o = static_cast<u32>(objects.size());
    Object object;
    object.key = spec.key;
    object.wallCount = static_cast<u32>(spec.walls.size());
    object.firstWall = static_cast<u32>(wallGroup.size());
    // Its walls and groups: a group per wall index, ranked by the key.
    const auto firstGroup = static_cast<u32>(groupFixed.size());
    for (u32 k = 0; k < object.wallCount; ++k) {
      groupFixed.push_back(1);
      groupTarget.push_back(0);
      groupRank.push_back((u64{1} << 63U) |
                          (splitMix(spec.key ^ splitMix(u64{k} + 1U)) >> 1U));
    }
    for (u32 k = 0; k < object.wallCount; ++k) {
      const auto& wall = spec.walls[k];
      const auto local = wall.group < object.wallCount ? wall.group : k;
      wallGroup.push_back(firstGroup + local);
      wallObject.push_back(o);
      wallCuts.push_back(spec.cuts.empty() || (k < spec.cuts.size() && spec.cuts[k] != 0)
                             ? 1
                             : 0);
      groupFixed[firstGroup + local] = wall.fixed ? 1 : 0;
      groupTarget[firstGroup + local] = wall.target ? 1 : 0;
    }
    addLap(1);
    // Its stamp in its box.
    object.x0 = std::numeric_limits<std::int64_t>::max();
    object.y0 = std::numeric_limits<std::int64_t>::max();
    object.x1 = std::numeric_limits<std::int64_t>::min();
    object.y1 = std::numeric_limits<std::int64_t>::min();
    for (const auto cell : spec.cells) {
      if (cell >= static_cast<std::size_t>(width) * height) {
        throw std::invalid_argument("CdtWorld: an object has a cell off the grid");
      }
      const auto x = static_cast<std::int64_t>(cell % width);
      const auto y = static_cast<std::int64_t>(cell / width);
      object.x0 = std::min(object.x0, x);
      object.y0 = std::min(object.y0, y);
      object.x1 = std::max(object.x1, x);
      object.y1 = std::max(object.y1, y);
    }
    if (spec.cells.empty()) {
      object.x0 = object.y0 = 0;
      object.x1 = object.y1 = -1;
    }
    object.stamp.reset(object.x0, object.y0, object.x1, object.y1, 0);
    for (std::size_t k = 0; k < spec.cells.size(); ++k) {
      const auto x = static_cast<std::int64_t>(spec.cells[k] % width);
      const auto y = static_cast<std::int64_t>(spec.cells[k] / width);
      if (spec.wallOf[k] >= object.wallCount) {
        throw std::invalid_argument("CdtWorld: an object cell names no wall of it");
      }
      object.stamp.at(x, y) = spec.wallOf[k] + 1;
    }
    object.layer = static_cast<u32>(layers.size());
    layers.push_back({.objects = {o, NONE}});
    objectOfKey.emplace(spec.key, o);
    objects.push_back(std::move(object));
    auto& added = objects[o];
    addLap(2);
    {
      // The squares within OBJ_REACH of its cells, and those within one
      // cell. A cell that lies deep inside its square (farther than the
      // reach from every side) marks that square alone; the cells come in
      // row order, so a run of cells in one square is looked at once.
      std::vector<std::size_t> marked;
      const auto squaresWithin = [&](const std::int64_t reach, std::vector<std::size_t>& out) {
        // The cells come in row order: a run of cells in one row marks the
        // squares within reach of the run at once.
        const auto mark = [&](const std::int64_t xa, const std::int64_t xb, const std::int64_t y) {
          const auto sx0 = std::clamp<std::int64_t>((xa - reach) / OBJ_SQUARE, 0, objNx - 1);
          const auto sx1 = std::clamp<std::int64_t>((xb + reach) / OBJ_SQUARE, 0, objNx - 1);
          const auto sy0 = std::clamp<std::int64_t>((y - reach) / OBJ_SQUARE, 0, objNy - 1);
          const auto sy1 = std::clamp<std::int64_t>((y + reach) / OBJ_SQUARE, 0, objNy - 1);
          for (auto ty = sy0; ty <= sy1; ++ty) {
            for (auto tx = sx0; tx <= sx1; ++tx) {
              const auto sq = static_cast<std::size_t>((ty * objNx) + tx);
              if (squareMark[sq] == 0) {
                squareMark[sq] = 1;
                out.push_back(sq);
              }
            }
          }
        };
        std::int64_t runY = -1;
        std::int64_t runA = 0;
        std::int64_t runB = -2;
        for (const auto cell : spec.cells) {
          const auto x = static_cast<std::int64_t>(cell % width);
          const auto y = static_cast<std::int64_t>(cell / width);
          if (y == runY && x == runB + 1) {
            runB = x;
            continue;
          }
          if (runY >= 0) {
            mark(runA, runB, runY);
          }
          runY = y;
          runA = runB = x;
        }
        if (runY >= 0) {
          mark(runA, runB, runY);
        }
        for (const auto sq : out) {
          squareMark[sq] = 0;
        }
        std::ranges::sort(out);
      };
      squaresWithin(OBJ_REACH, marked);
      reachSquares.resize(objects.size());
      reachSquares[o].assign(marked.begin(), marked.end());
      for (const auto sq : marked) {
        squareObjects[sq].push_back(o);
      }
      marked.clear();
      squaresWithin(1, marked);
      objectSquares.resize(objects.size());
      objectSquares[o].assign(marked.begin(), marked.end());
    }
    addLap(3);
    // The points already there that it touches or lies near: those of the
    // point squares its own squares (within OBJ_REACH of its cells) meet,
    // not those of its whole box, which a long diagonal edge fills with
    // points it never comes near.
    if (!spec.cells.empty()) {
      static_assert(GID_SQUARE == OBJ_SQUARE);
      for (const auto sq : squaresOf(o)) {
        for (const auto g : gidSquares[sq]) {
          relate(g, o);
        }
      }
    }

    addLap(4);
    // Its outline, on its own stamp, from the free cells next to its cells.
    std::vector<std::int64_t> freeNext;
    {
      const auto bx0 = added.x0 - 1;
      const auto by0 = added.y0 - 1;
      const auto bw = added.x1 - added.x0 + 3;
      freeNext.reserve(spec.cells.size() * 2);
      for (const auto cell : spec.cells) {
        const auto x = static_cast<std::int64_t>(cell % width);
        const auto y = static_cast<std::int64_t>(cell / width);
        for (const auto& [dx, dy] : {std::pair{0, -1}, std::pair{1, 0},
                                     std::pair{0, 1}, std::pair{-1, 0}}) {
          if (objects[o].wallAt(x + dx, y + dy) == NONE) {
            freeNext.push_back(((y + dy - by0) * bw) + (x + dx - bx0));
          }
        }
      }
      std::ranges::sort(freeNext);
      freeNext.erase(std::unique(freeNext.begin(), freeNext.end()), freeNext.end());
    }
    added.outline.loops =
        spec.cells.empty()
            ? std::vector<Loop>{}
            : outlineOf(
                  added.x0 - 1, added.y0 - 1, added.x1 + 1, added.y1 + 1,
                  options.simplify, options.clearance,
                  [&](const std::int64_t x, const std::int64_t y) {
                    return objects[o].wallAt(x, y) != NONE;
                  },
                  [&](const std::int64_t x, const std::int64_t y) {
                    return objects[o].wallAt(x, y);
                  },
                  &freeNext);
    addLap(5);
    boxOf(added.outline);
    addLap(6);
    // The foot points on it: of the static corners and of its own.
    auto feet = baseFeetOn(added.outline);
    {
      const auto own = feetOf(added.outline.loops, added.outline);
      feet.insert(feet.end(), own.begin(), own.end());
    }
    layLoops(added.layer, objects[o].outline, feet);
    addLap(7);
    // Its corners' foot points on the static outline.
    layOffsetFeet(added.layer, baseOutline,
                  feetOf(objects[o].outline.loops, baseOutline));
    addLap(8);
    // The polygon of its outline.
    {
      auto& self = objects[o];
      self.sx0 = std::numeric_limits<double>::max();
      self.sy0 = std::numeric_limits<double>::max();
      self.sx1 = std::numeric_limits<double>::lowest();
      self.sy1 = std::numeric_limits<double>::lowest();
      for (const auto& [a, b] : layers[self.layer].edges) {
        const auto& p = gplaced[a];
        const auto& q = gplaced[b];
        self.segments.push_back({p.x, p.y, q.x, q.y});
        self.sx0 = std::min({self.sx0, p.x, q.x});
        self.sy0 = std::min({self.sy0, p.y, q.y});
        self.sx1 = std::max({self.sx1, p.x, q.x});
        self.sy1 = std::max({self.sy1, p.y, q.y});
      }
    }
    addLap(9);
    // The foot points between it and the objects near it are laid when the
    // two first stand in one state (`ensurePairs`): an object has many near
    // it that it never stands with, such as the other versions of itself.
    for (const auto touched : nearTouched) {
      auto& other = objects[touched];
      std::ranges::sort(other.near);
      other.near.erase(std::unique(other.near.begin(), other.near.end()),
                       other.near.end());
      nearDirty[touched] = 0;
    }
    nearTouched.clear();
    addLap(10);
    return o;
  }

  [[nodiscard]] static bool insidePolygon(const Object& object, const double x,
                                          const double y) {
    if (x < object.sx0 || x > object.sx1 || y < object.sy0 || y > object.sy1) {
      return false;
    }
    bool in = false;
    for (const auto& s : object.segments) {
      if ((s[1] > y) != (s[3] > y) &&
          x < ((s[2] - s[0]) * (y - s[1]) / (s[3] - s[1])) + s[0]) {
        in = !in;
      }
    }
    return in;
  }

  // ================================================================ state queries

  /// Where a vertex is, unmoved: a point of the world where it was given, a
  /// point the triangulation added where it put it.
  [[nodiscard]] XY at(const State& s, const u32 v) const {
    const auto g = v < s.gidOf.size() ? s.gidOf[v] : NONE;
    if (g != NONE) {
      return gpos[g];
    }
    return {s.cdt.vertices[v].x, s.cdt.vertices[v].y};
  }

  /// The groups and walls of a vertex in a state.
  void groupsAt(const State& s, const u32 v, Groups& groups, Groups& walls) const {
    const bool cached = &s == groupsState && v < groupsStamp.size();
    if (cached && groupsStamp[v] == groupsEpoch) {
      groups = groupsCache[v];
      walls = wallsCache[v];
      return;
    }
    groupsAtUncached(s, v, groups, walls);
    if (cached) {
      groupsCache[v] = groups;
      wallsCache[v] = walls;
      groupsStamp[v] = groupsEpoch;
    }
  }
  void groupsAtUncached(const State& s, const u32 v, Groups& groups,
                        Groups& walls) const {
    groups = {};
    walls = {};
    if (v < SUPER) {
      return;
    }
    const auto g = s.gidOf[v];
    if (g == NONE) {
      // A point the triangulation added: the walls whose cells lie within a
      // cell of it.
      const auto& p = s.cdt.vertices[v];
      cellsNear(p.x, p.y, 1.0, [&](const std::int64_t x, const std::int64_t y) {
        wallsOfCell(s, x, y, [&](const u32 wall) {
          groups.add(groupOfWall(wall));
          walls.add(wall);
        });
      });
      return;
    }
    for (const auto& entry : entries[g]) {
      if (layerPresent(s, entry.layer)) {
        groups.add(entry.groups);
        walls.add(entry.walls);
      }
    }
    for (const auto wall : touch[g]) {
      if (wallPresent(s, wall)) {
        groups.add(groupOfWall(wall));
        walls.add(wall);
      }
    }
  }

  [[nodiscard]] bool isConstraint(const State& s, const u32 va, const u32 vb) const {
    const auto ga = s.gidOf[va];
    const auto gb = s.gidOf[vb];
    if (ga != NONE && gb != NONE) {
      for (const auto& [other, layer] : constraintsAt[ga]) {
        if (other == gb && layerPresent(s, layer)) {
          return true;
        }
      }
      // A piece of a constraint between two points of the world: the
      // constraint was split where a point lies on it.
      return s.splitEnd[va] != 0 && s.splitEnd[vb] != 0 &&
             s.cdt.fixedEdges.contains(CDT::Edge(va, vb));
    }
    return s.cdt.fixedEdges.contains(CDT::Edge(va, vb));
  }

  /// Marks the ends of every piece of a constraint (`splitEnd`).
  static void markSplitEnds(State& s) {
    s.splitEnd.resize(s.cdt.vertices.size(), 0);
    for (const auto& [edge, originals] : s.cdt.pieceToOriginals) {
      s.splitEnd[edge.v1()] = 1;
      s.splitEnd[edge.v2()] = 1;
    }
  }

  /// Whether a triangle is free: its centroid in a free cell of the raster,
  /// outside the static outlines and outside every object's.
  [[nodiscard]] std::uint8_t classify(const State& s, const u32 t) const {
    const auto& v = s.cdt.triangles[t].vertices;
    if (v[0] < SUPER || v[1] < SUPER || v[2] < SUPER) {
      return 0;
    }
    const auto& vs = s.cdt.vertices;
    const auto cx = (vs[v[0]].x + vs[v[1]].x + vs[v[2]].x) / 3.0;
    const auto cy = (vs[v[0]].y + vs[v[1]].y + vs[v[2]].y) / 3.0;
    const auto x = static_cast<std::int64_t>(std::floor(cx));
    const auto y = static_cast<std::int64_t>(std::floor(cy));
    if (wallCell(s, x, y)) {
      return 0;
    }
    if (!staticFreeAt(cx, cy)) {
      return 0;
    }
    bool inside = false;
    presentNear(s, x, y, [&](const u32 o) {
      inside = inside || insidePolygon(objects[o], cx, cy);
    });
    if (inside) {
      return 0;
    }
    // The sliver between a foot laid a hair off a piece and that piece: the
    // foot touches the wall, as the gate that ends on it does. Free, the
    // sliver would join the two sides of the gate under its end (whether
    // its centroid lies in a free cell depends on which way the move of the
    // piece's ends took it).
    for (std::size_t k = 0; k < 3; ++k) {
      const auto f = v[k];
      const auto g = f < s.gidOf.size() ? s.gidOf[f] : NONE;
      if (g == NONE || hair[g] == 0) {
        continue;
      }
      const auto p = v[(k + 1) % 3];
      const auto q = v[(k + 2) % 3];
      const auto dx = vs[q].x - vs[p].x;
      const auto dy = vs[q].y - vs[p].y;
      const auto length = std::hypot(dx, dy);
      const auto off = std::abs((dx * (vs[f].y - vs[p].y)) - (dy * (vs[f].x - vs[p].x)));
      if (length > 0.0 && off < HAIR_REACH * length && isConstraint(s, p, q)) {
        return 0;
      }
    }
    return 1;
  }

  [[nodiscard]] u32 wiresIn(const double cells) const {
    const auto clearance = options.clearance;
    if (cells < clearance) {
      return 0;
    }
    const auto plain = static_cast<u32>(std::floor(cells / clearance));
    if (plain < options.marginFrom) {
      return plain;
    }
    return static_cast<u32>(std::floor(cells / (clearance * options.margin)));
  }

  /// The two ends of a candidate, the lower first: what tells two
  /// candidates of one width apart.
  [[nodiscard]] std::array<double, 4> keyOf(const State& s,
                                            const Candidate& c) const {
    const auto pa = at(s, c.a);
    const auto pb = at(s, c.b);
    return {pa.x, pa.y, pb.x, pb.y};
  }

  /// The candidate of an edge, per step 6 of `cdtGates`, or NONE.
  u32 evaluate(State& s, const u32 t, const std::size_t k) const {
    const auto& tr = s.cdt.triangles[t];
    const auto across = tr.neighbors[k];
    if (s.freeT[t] == 0 || across == CDT::noNeighbor || s.freeT[across] == 0) {
      return NONE;
    }
    const auto a = tr.vertices[k];
    const auto b = tr.vertices[(k + 1) % 3];
    Groups ga;
    Groups wa;
    Groups gb;
    Groups wb;
    groupsAt(s, a, ga, wa);
    groupsAt(s, b, gb, wb);
    if (ga.count == 0 || gb.count == 0 || ga.meets(gb)) {
      return NONE;
    }
    // A constraint is a wall's edge, never a gap.
    if (isConstraint(s, a, b)) {
      return NONE;
    }
    struct Kind {
      bool fixed;
      bool target;
      u64 side;
    };
    const auto kindOf = [&](const Groups& groups) {
      bool dynamic = false;
      bool onlyTarget = groups.count > 0;
      u64 side = std::numeric_limits<u64>::max();
      u64 any = std::numeric_limits<u64>::max();
      for (std::uint8_t g = 0; g < groups.count; ++g) {
        const auto group = groups.of[g];
        any = std::min(any, groupRank[group]);
        if (!fixedGroup(group)) {
          dynamic = true;
          side = std::min(side, groupRank[group]);
          onlyTarget = onlyTarget && targetGroup(group);
        }
      }
      return Kind{.fixed = !dynamic,
                  .target = dynamic && onlyTarget,
                  .side = dynamic ? side : any};
    };
    const auto ka = kindOf(ga);
    const auto kb = kindOf(gb);
    const bool quietA = ka.fixed || ka.target;
    const bool quietB = kb.fixed || kb.target;
    if (quietA && quietB && !(ka.target && kb.target)) {
      return NONE;
    }
    // The lower end first, so that the gate's two cells come in an order
    // that does not depend on which of its triangles found the edge; and its
    // width and heading from the lower end, so that they do not either (the
    // heading of the edge turned round may differ in the last bit, and a
    // width of a whole number of wires then holds one fewer).
    const auto pa = at(s, a);
    const auto pb = at(s, b);
    const bool aFirst = pa.x != pb.x ? pa.x < pb.x : pa.y < pb.y;
    const auto& lo = aFirst ? pa : pb;
    const auto& hi = aFirst ? pb : pa;
    const auto length = std::hypot(hi.x - lo.x, hi.y - lo.y);
    const auto cap = wiresIn(length * headingShareOf(hi.x - lo.x, hi.y - lo.y));
    if (cap >= options.widest) {
      return NONE;
    }
    Candidate c;
    c.a = aFirst ? a : b;
    c.b = aFirst ? b : a;
    c.t1 = t;
    c.t2 = across;
    c.cap = cap;
    c.width = length;
    c.pairLo = std::min(ka.side, kb.side);
    c.pairHi = std::max(ka.side, kb.side);
    s.pool.push_back(c);
    return static_cast<u32>(s.pool.size() - 1);
  }

  /// Evaluates the edge k of triangle t into both of its slots.
  void evaluateInto(State& s, const u32 t, const std::size_t k) const {
    const auto c = evaluate(s, t, k);
    const auto old = s.slot[(t * 3) + k];
    if (old != NONE && old != c && s.pool[old].dead == 0) {
      s.pool[old].dead = 1;
      s.pool[old].gate = 0;
      ++s.deadCandidates;
    }
    s.slot[(t * 3) + k] = c;
    const auto o = s.cdt.triangles[t].neighbors[k];
    if (o != CDT::noNeighbor) {
      const auto& to = s.cdt.triangles[o];
      for (std::size_t j = 0; j < 3; ++j) {
        if (to.neighbors[j] == t) {
          const auto other = s.slot[(o * 3) + j];
          if (other != NONE && other != c && other != old &&
              s.pool[other].dead == 0) {
            s.pool[other].dead = 1;
            s.pool[other].gate = 0;
            ++s.deadCandidates;
          }
          s.slot[(o * 3) + j] = c;
        }
      }
    }
  }

  /// Whether the edge between two candidates of one triangle spans the mouth
  /// of a port run (step 7 of `cdtGates`).
  [[nodiscard]] bool isMouth(const State& s, const Candidate& one,
                             const Candidate& two) const {
    const auto shared = (one.a == two.a || one.a == two.b) ? one.a : one.b;
    const auto x = one.a == shared ? one.b : one.a;
    const auto y = two.a == shared ? two.b : two.a;
    if (isConstraint(s, x, y)) {
      return false;
    }
    Groups gx;
    Groups wx;
    Groups gy;
    Groups wy;
    groupsAt(s, x, gx, wx);
    groupsAt(s, y, gy, wy);
    for (std::uint8_t i = 0; i < wx.count; ++i) {
      for (std::uint8_t j = 0; j < wy.count; ++j) {
        if (wx.of[i] != wy.of[j] &&
            groupOfWall(wx.of[i]) == groupOfWall(wy.of[j])) {
          return true;
        }
      }
    }
    return false;
  }

  /// The basins of the channels the seeds belong to, each channel taken
  /// whole (step 7 of `cdtGates`, ties broken by the ends); the members are
  /// collected in `touched`.
  void basinsFrom(State& s, const std::vector<u32>& seeds,
                  std::vector<u32>& touched, CdtStepStats& stats) const {
    std::vector<u32> mark(s.pool.size(), 0);
    std::vector<u32> local(s.pool.size(), 0);
    u32 epoch = 1;
    std::vector<u32> component;
    std::vector<u32> stack;
    std::vector<std::vector<u32>> neighbours;
    std::vector<std::array<double, 4>> keys;
    for (const auto seed : seeds) {
      if (seed == NONE || mark[seed] == epoch || s.pool[seed].dead != 0) {
        continue;
      }
      component.clear();
      stack.assign(1, seed);
      mark[seed] = epoch;
      while (!stack.empty()) {
        const auto e = stack.back();
        stack.pop_back();
        local[e] = static_cast<u32>(component.size());
        component.push_back(e);
        const auto& ce = s.pool[e];
        for (const auto tri : {ce.t1, ce.t2}) {
          for (std::size_t j = 0; j < 3; ++j) {
            const auto f = s.slot[(tri * 3) + j];
            if (f == NONE || f == e || mark[f] == epoch ||
                !s.pool[f].samePair(ce) || isMouth(s, ce, s.pool[f])) {
              continue;
            }
            mark[f] = epoch;
            stack.push_back(f);
          }
        }
      }
      const auto n = component.size();
      neighbours.assign(n, {});
      for (std::size_t i = 0; i < n; ++i) {
        const auto& ce = s.pool[component[i]];
        for (const auto tri : {ce.t1, ce.t2}) {
          for (std::size_t j = 0; j < 3; ++j) {
            const auto f = s.slot[(tri * 3) + j];
            if (f == NONE || f == component[i] || !s.pool[f].samePair(ce) ||
                isMouth(s, ce, s.pool[f])) {
              continue;
            }
            neighbours[i].push_back(local[f]);
          }
        }
      }
      keys.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        keys[i] = keyOf(s, s.pool[component[i]]);
      }
      const auto narrower = [&](const u32 a, const u32 b) {
        const auto& ca = s.pool[component[a]];
        const auto& cb = s.pool[component[b]];
        if (ca.width != cb.width) {
          return ca.width < cb.width;
        }
        if (keys[a] != keys[b]) {
          return keys[a] < keys[b];
        }
        return a < b;
      };
      std::vector<u32> order(n);
      std::iota(order.begin(), order.end(), 0U);
      std::ranges::stable_sort(order, [&](const u32 a, const u32 b) {
        return s.pool[component[a]].cap < s.pool[component[b]].cap;
      });
      MinSets sets(n);
      std::vector<bool> added(n, false);
      std::vector<std::uint8_t> gate(n, 0);
      for (std::size_t from = 0; from < n;) {
        const auto cap = s.pool[component[order[from]]].cap;
        auto to = from;
        while (to < n && s.pool[component[order[to]]].cap == cap) {
          added[order[to]] = true;
          ++to;
        }
        for (auto k = from; k < to; ++k) {
          const auto e = order[k];
          for (const auto f : neighbours[e]) {
            const auto other = s.pool[component[f]].cap;
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
          if (cap == 0 && s.pool[component[e]].width < NARROWEST) {
            continue;
          }
          gate[e] = 1;
        }
        from = to;
      }
      for (std::size_t i = 0; i < n; ++i) {
        s.pool[component[i]].gate = gate[i];
        touched.push_back(component[i]);
      }
      ++stats.channels;
      stats.channelCandidates += n;
    }
  }

  /// The wall cell of one of `groups` nearest a point, else the nearest
  /// wall cell (step 8 of `cdtGates`).
  [[nodiscard]] std::size_t wallCellNear(const State& s, const XY p,
                                         const Groups& groups) const {
    const auto px = static_cast<std::int64_t>(std::floor(p.x));
    const auto py = static_cast<std::int64_t>(std::floor(p.y));
    double best = std::numeric_limits<double>::max();
    auto found = std::numeric_limits<std::size_t>::max();
    double bestAny = std::numeric_limits<double>::max();
    auto any = std::numeric_limits<std::size_t>::max();
    for (std::int64_t y = py - 3; y <= py + 2; ++y) {
      for (std::int64_t x = px - 3; x <= px + 2; ++x) {
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        bool wall = false;
        bool mine = false;
        wallsOfCell(s, x, y, [&](const u32 w) {
          wall = true;
          mine = mine || groups.has(groupOfWall(w));
        });
        if (!wall) {
          continue;
        }
        const auto cell = static_cast<std::size_t>((y * width) + x);
        const auto d = std::hypot((static_cast<double>(x) + 0.5) - p.x,
                                  (static_cast<double>(y) + 0.5) - p.y);
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
  }

  /// The gate cells of a candidate, and whether it cuts.
  void measure(const State& s, Candidate& c) const {
    Groups ga;
    Groups wa;
    Groups gb;
    Groups wb;
    groupsAt(s, c.a, ga, wa);
    groupsAt(s, c.b, gb, wb);
    const auto pa = at(s, c.a);
    const auto pb = at(s, c.b);
    const auto first = wallCellNear(s, pa, ga);
    const auto second = wallCellNear(s, pb, gb);
    c.measured = 1;
    c.valid = first != std::numeric_limits<std::size_t>::max() &&
                      second != std::numeric_limits<std::size_t>::max() &&
                      first != second
                  ? 1
                  : 0;
    c.cuts = 0;
    if (c.valid == 0) {
      return;
    }
    const auto w = static_cast<std::int64_t>(width);
    const auto h = static_cast<std::int64_t>(height);
    const auto mx = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor((pa.x + pb.x) / 2.0)), 0, w - 1);
    const auto my = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(std::floor((pa.y + pb.y) / 2.0)), 0, h - 1);
    c.first = static_cast<u32>(first);
    c.second = static_cast<u32>(second);
    c.saddle = static_cast<u32>((my * w) + mx);
    for (const auto cell : {first, second}) {
      wallsOfCell(s, static_cast<std::int64_t>(cell % width),
                  static_cast<std::int64_t>(cell / width), [&](const u32 wall) {
                    c.cuts = c.cuts != 0 || wallCuts[wall] != 0 ? 1 : 0;
                  });
    }
  }

  [[nodiscard]] static bool separates(const Candidate& c) {
    return c.gate != 0 && c.valid != 0 && c.cuts != 0;
  }
  [[nodiscard]] bool separatesEdge(const State& s, const u32 t,
                                   const std::size_t k) const {
    const auto c = s.slot[(t * 3) + k];
    if (c != NONE && s.pool[c].sep != 0) {
      return true;
    }
    const auto& v = s.cdt.triangles[t].vertices;
    return isConstraint(s, v[k], v[(k + 1) % 3]);
  }

  [[nodiscard]] static ChamberKey centroidKey(const State& s, const u32 t) {
    const auto& v = s.cdt.triangles[t].vertices;
    const auto& vs = s.cdt.vertices;
    return {(vs[v[0]].y + vs[v[1]].y + vs[v[2]].y) / 3.0,
            (vs[v[0]].x + vs[v[1]].x + vs[v[2]].x) / 3.0};
  }

  /// One chamber grown from a triangle: the free triangles reached without
  /// crossing a separating edge. Those already labelled are other chambers
  /// it meets (to be merged).
  void grow(State& s, const u32 start, const u32 id, std::vector<u32>& members,
            std::vector<u32>& merged, ChamberKey& key) const {
    std::vector<u32> stack{start};
    members.assign(1, start);
    merged.clear();
    s.chamber[start] = id;
    key = centroidKey(s, start);
    while (!stack.empty()) {
      const auto u = stack.back();
      stack.pop_back();
      const auto& tr = s.cdt.triangles[u];
      for (std::size_t k = 0; k < 3; ++k) {
        const auto o = tr.neighbors[k];
        if (o == CDT::noNeighbor || s.freeT[o] == 0 || s.chamber[o] == id ||
            separatesEdge(s, u, k)) {
          continue;
        }
        if (s.chamber[o] != NONE) {
          merged.push_back(s.chamber[o]);
          continue;
        }
        s.chamber[o] = id;
        stack.push_back(o);
        members.push_back(o);
        key = std::min(key, centroidKey(s, o));
      }
    }
  }

  void chambersFull(State& s) const {
    const auto n = static_cast<u32>(s.cdt.triangles.size());
    s.chamber.assign(n, NONE);
    s.chamberKey.clear();
    s.chamberAlive.clear();
    std::vector<u32> members;
    std::vector<u32> merged;
    for (u32 t = 0; t < n; ++t) {
      if (s.freeT[t] == 0 || s.chamber[t] != NONE) {
        continue;
      }
      const auto id = static_cast<u32>(s.chamberKey.size());
      ChamberKey key;
      grow(s, t, id, members, merged, key);
      s.chamberKey.push_back(key);
      s.chamberAlive.push_back(1);
    }
  }

  // ================================================================ states

  /// Puts every point the triangulation added from `from` on at the
  /// intersection of its two original constraints, computed from their
  /// lines in a fixed order. The triangulation computes it from the piece of
  /// a constraint it meets, which depends on what was inserted before: the
  /// place differs in the last bits, and the state would not be the same
  /// whichever way it was built.
  ///
  /// A line is a row of the grid (one placed y), a column (one X, see
  /// `Placement`) or another line by the ends of its constraint. Constraints
  /// of one row or column (two walls that touch along it) are one line,
  /// whichever of them the triangulation names at the point: which it names
  /// depends on the order they came in. A point on a row or a column is put
  /// on it exactly, so that a constraint along it that comes later finds the
  /// point on its way instead of crossing a piece a hair off it.
  void canonicalSteiner(State& s, const u32 from) const {
    auto& cdt = s.cdt;
    const auto nv = static_cast<u32>(cdt.vertices.size());
    using Segment = std::array<double, 4>;
    std::vector<Segment> segments;
    for (u32 v = from; v < nv; ++v) {
      if (v < s.gidOf.size() && s.gidOf[v] != NONE) {
        continue;
      }
      segments.clear();
      const auto t0 = cdt.VertTrisInternal()[v];
      if (t0 == CDT::noNeighbor) {
        continue;
      }
      auto t = t0;
      for (int guard = 0; guard < 4096; ++guard) {
        const auto& tr = cdt.triangles[t];
        for (const auto u : tr.vertices) {
          if (u == v) {
            continue;
          }
          const CDT::Edge edge(v, u);
          if (!cdt.fixedEdges.contains(edge)) {
            continue;
          }
          const auto found = cdt.pieceToOriginals.find(edge);
          if (found == cdt.pieceToOriginals.end()) {
            continue;
          }
          for (const auto& original : found->second) {
            const auto& p = cdt.vertices[original.v1()];
            const auto& q = cdt.vertices[original.v2()];
            const bool pFirst = p.x != q.x ? p.x < q.x : p.y < q.y;
            Segment segment = pFirst ? Segment{p.x, p.y, q.x, q.y}
                                     : Segment{q.x, q.y, p.x, p.y};
            if (std::ranges::find(segments, segment) == segments.end()) {
              segments.push_back(segment);
            }
          }
        }
        const auto next = tr.next(v).first;
        if (next == CDT::noNeighbor || next == t0) {
          break;
        }
        t = next;
      }
      if (segments.size() < 2) {
        continue;
      }
      // The lines: (kind, key, ends), kind 0 a row, 1 a column, 2 any other.
      using Line = std::pair<int, Segment>;
      std::vector<Line> lines;
      for (const auto& g : segments) {
        Line line{2, g};
        if (g[1] == g[3]) {
          line = {0, {g[1], 0.0, 0.0, 0.0}};
        } else if (const auto column = placement.columnOf(g[0], g[1]);
                   column == placement.columnOf(g[2], g[3])) {
          line = {1, {column, 0.0, 0.0, 0.0}};
        }
        if (std::ranges::find(lines, line) == lines.end()) {
          lines.push_back(line);
        }
      }
      if (lines.size() < 2) {
        continue;
      }
      std::ranges::sort(lines);
      const auto& [ka, a] = lines[0];
      const auto& [kb, b] = lines[1];
      if (ka == kb && ka != 2) {
        continue;
      }
      if (ka == 0 && kb == 1) {
        // A row and a column: their crossing, exactly.
        cdt.vertices[v] = {b[0] + (placement.shear * (a[0] - placement.middle)), a[0]};
      } else if (ka == 0) {
        // A row and another line: on the row.
        const auto y = a[0];
        const auto along = (y - b[1]) / (b[3] - b[1]);
        cdt.vertices[v] = {b[0] + (along * (b[2] - b[0])), y};
      } else if (ka == 1) {
        // A column and another line: on the column.
        const auto c0 = placement.columnOf(b[0], b[1]);
        const auto c1 = placement.columnOf(b[2], b[3]);
        const auto along = (a[0] - c0) / (c1 - c0);
        const auto p = placement.onColumn(a[0], b[1] + (along * (b[3] - b[1])));
        cdt.vertices[v] = {p.x, p.y};
      } else {
        cdt.vertices[v] = CDT::detail::intersectionPosition(
            CDT::V2d<double>(a[0], a[1]), CDT::V2d<double>(a[2], a[3]),
            CDT::V2d<double>(b[0], b[1]), CDT::V2d<double>(b[2], b[3]));
      }
    }
  }

  /// The pairs of objects checked for foot points between them, by the two
  /// indices (the lower in the high half).
  std::unordered_set<u64> pairsDone;

  /// **The foot points between two objects that stand together**: for every
  /// object of `fresh` and every object `present` near it, the layer of the
  /// foot points of each one's corners on the other, laid once, the first
  /// time the two stand in one state. What a pair lays depends on the pair
  /// alone, the newer object's corners first as when it was added.
  void ensurePairs(std::span<const u32> fresh,
                   const std::vector<std::uint8_t>& present) const {
    auto& self = *const_cast<Impl*>(this);
    const auto reach = options.footReach * options.clearance;
    // Only the objects that are present: `present` is the state's, and the
    // objects of the world are many more.
    std::vector<u32> presentList;
    for (u32 q = 0; q < present.size(); ++q) {
      if (present[q] != 0) {
        presentList.push_back(q);
      }
    }
    for (const auto o : fresh) {
      for (const auto q : presentList) {
        if (q == o) {
          continue;
        }
        const auto hi = std::max(o, q);
        const auto lo = std::min(o, q);
        if (!self.pairsDone.insert((u64{lo} << 32U) | hi).second) {
          continue;
        }
        const auto& a = objects[hi].outline;
        const auto& b = objects[lo].outline;
        if (a.loops.empty() || b.loops.empty() || a.x1 + reach < b.x0 ||
            b.x1 + reach < a.x0 || a.y1 + reach < b.y0 || b.y1 + reach < a.y0) {
          continue;
        }
        const auto layer = static_cast<u32>(layers.size());
        self.layers.push_back({.objects = {lo, hi}});
        self.layOffsetFeet(layer, self.objects[lo].outline,
                           feetOf(objects[hi].outline.loops, objects[lo].outline));
        self.layOffsetFeet(layer, self.objects[hi].outline,
                           feetOf(objects[lo].outline.loops, objects[hi].outline));
        if (self.layers[layer].gids.empty()) {
          self.layers.pop_back();
          continue;
        }
        self.objects[hi].pairs.push_back(layer);
        self.objects[lo].pairs.push_back(layer);
      }
    }
    for (const auto touched : self.nearTouched) {
      auto& other = self.objects[touched];
      std::ranges::sort(other.near);
      other.near.erase(std::unique(other.near.begin(), other.near.end()),
                       other.near.end());
      self.nearDirty[touched] = 0;
    }
    self.nearTouched.clear();
  }

  [[nodiscard]] std::shared_ptr<State> scratch(std::span<const u32> list) const {
    const auto began = Clock::now();
    auto s = std::make_shared<State>();
    s->present.assign(objects.size(), 0);
    for (const auto o : list) {
      s->present.at(o) = 1;
    }
    for (u32 o = 0; o < objects.size(); ++o) {
      if (s->present[o] != 0) {
        listInSquares(*s, o);
      }
    }
    ensurePairs(list, s->present);
    for (u32 o = 0; o < objects.size(); ++o) {
      if (s->present[o] != 0) {
        s->objects.push_back(o);
      }
    }
    s->vidOf.assign(gpos.size(), NONE);
    std::vector<CDT::V2d<double>> points;
    std::vector<u32> order;
    for (u32 l = 0; l < layers.size(); ++l) {
      if (!layerPresent(*s, l)) {
        continue;
      }
      for (const auto g : layers[l].gids) {
        if (s->vidOf[g] != NONE) {
          continue;
        }
        s->vidOf[g] = static_cast<u32>(SUPER + points.size());
        points.emplace_back(gplaced[g].x, gplaced[g].y);
        order.push_back(g);
      }
    }
    std::vector<CDT::Edge> edges;
    for (u32 l = 0; l < layers.size(); ++l) {
      if (!layerPresent(*s, l)) {
        continue;
      }
      for (const auto& [a, b] : layers[l].edges) {
        edges.emplace_back(s->vidOf[a] - SUPER, s->vidOf[b] - SUPER);
      }
    }
    std::ranges::sort(edges, [](const CDT::Edge& x, const CDT::Edge& y) {
      return std::pair{x.v1(), x.v2()} < std::pair{y.v1(), y.v2()};
    });
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    s->cdt.insertVertices(points);
    {
      const SnapScope snapping(s->cdt, placement);
      s->cdt.insertEdges(edges);
    }
    // The point locator is built on the next insertion: once here, so that
    // the first state built from this one does not pay for it.
    s->cdt.insertVertices(std::vector<CDT::V2d<double>>{});
    s->gidOf.assign(s->cdt.vertices.size(), NONE);
    for (std::size_t k = 0; k < order.size(); ++k) {
      s->gidOf[SUPER + k] = order[k];
    }
    canonicalSteiner(*s, static_cast<u32>(SUPER + order.size()));
    markSplitEnds(*s);
    beginGroupsCache(*s);
    const auto n = static_cast<u32>(s->cdt.triangles.size());
    s->vertTri.assign(s->cdt.vertices.size(), NONE);
    for (u32 t = 0; t < n; ++t) {
      for (const auto v : s->cdt.triangles[t].vertices) {
        s->vertTri[v] = t;
      }
    }
    s->freeT.resize(n);
    for (u32 t = 0; t < n; ++t) {
      s->freeT[t] = classify(*s, t);
    }
    s->slot.assign(static_cast<std::size_t>(n) * 3, NONE);
    for (u32 t = 0; t < n; ++t) {
      for (std::size_t k = 0; k < 3; ++k) {
        const auto o = s->cdt.triangles[t].neighbors[k];
        if (o != CDT::noNeighbor && o < t) {
          continue;
        }
        evaluateInto(*s, t, k);
      }
    }
    std::vector<u32> seeds;
    for (u32 c = 0; c < s->pool.size(); ++c) {
      seeds.push_back(c);
    }
    std::vector<u32> touched;
    basinsFrom(*s, seeds, touched, s->stats);
    for (u32 c = 0; c < s->pool.size(); ++c) {
      auto& cand = s->pool[c];
      if (cand.gate != 0) {
        measure(*s, cand);
        s->gateList.push_back(c);
      }
      cand.sep = separates(cand) ? 1 : 0;
    }
    chambersFull(*s);
    output(*s);
    s->stats.scratch = true;
    s->stats.objects = s->objects.size();
    s->stats.total = msSince(began, Clock::now());
    return s;
  }

  static void walkAround(const State& s, const u32 v, std::vector<u32>& out) {
    if (v >= s.vertTri.size()) {
      return;
    }
    const auto t0 = s.vertTri[v];
    if (t0 == NONE) {
      return;
    }
    auto t = t0;
    for (int guard = 0; guard < 4096; ++guard) {
      out.push_back(t);
      const auto next = s.cdt.triangles[t].next(v).first;
      if (next == CDT::noNeighbor || next == t0) {
        break;
      }
      t = next;
    }
  }

  /// Prints what two constraints that CDT could not resolve are: their
  /// ends, the points of the world they stand on and the layers and
  /// objects those belong to (MQT_SCPD_CDT_DEBUG_REFUSALS).
  void describeRefusal(const State& s, const u32 base, const std::vector<u32>& added,
                       const CDT::Edge& e1, const CDT::Edge& e2,
                       const char* what) const {
    std::fprintf(stderr, "CdtWorld refusal: %s\n", what);
    const auto describe = [&](const CDT::Edge& e) {
      for (const auto v : {e.v1() + SUPER, e.v2() + SUPER}) {
        const double x = v < s.cdt.vertices.size() ? s.cdt.vertices[v].x : -1.0;
        const double y = v < s.cdt.vertices.size() ? s.cdt.vertices[v].y : -1.0;
        u32 g = NONE;
        if (v >= base && v - base < added.size()) {
          g = added[v - base];
        } else if (v < s.gidOf.size()) {
          g = s.gidOf[v];
        }
        std::fprintf(stderr, "  vertex %u at (%.17g, %.17g) gid %u%s", v, x, y, g,
                     v >= base ? " (new)" : "");
        if (g != NONE) {
          std::fprintf(stderr, " unmoved (%.17g, %.17g) hair %d layers:", gpos[g].x, gpos[g].y,
                       static_cast<int>(hair[g]));
          for (const auto& entry : entries[g]) {
            const auto& lay = layers[entry.layer];
            std::fprintf(stderr, " %u(obj %d,%d)", entry.layer,
                         lay.objects[0] == NONE ? -1 : static_cast<int>(lay.objects[0]),
                         lay.objects[1] == NONE ? -1 : static_cast<int>(lay.objects[1]));
          }
          std::fprintf(stderr, " touch:");
          for (const auto w : touch[g]) {
            std::fprintf(stderr, " w%u(obj %d)", w,
                         wallObject[w] == NONE ? -1 : static_cast<int>(wallObject[w]));
          }
        }
        std::fprintf(stderr, "\n");
      }
    };
    std::fprintf(stderr, " edge 1:\n");
    describe(e1);
    std::fprintf(stderr, " edge 2:\n");
    describe(e2);
    // Which layers hold either edge, by the points of the world.
    const auto gidOfVertex = [&](const u32 v) {
      if (v >= base && v - base < added.size()) {
        return added[v - base];
      }
      return v < s.gidOf.size() ? s.gidOf[v] : NONE;
    };
    for (const auto& e : {e1, e2}) {
      const auto a = gidOfVertex(e.v1() + SUPER);
      const auto b = gidOfVertex(e.v2() + SUPER);
      std::fprintf(stderr, " edge (%u,%u) = gids (%u,%u) in layers:", e.v1(), e.v2(), a, b);
      for (u32 l = 0; l < layers.size(); ++l) {
        for (const auto& [x, y] : layers[l].edges) {
          if ((x == a && y == b) || (x == b && y == a)) {
            std::fprintf(stderr, " %u(obj %d,%d)%s", l,
                         layers[l].objects[0] == NONE ? -1 : static_cast<int>(layers[l].objects[0]),
                         layers[l].objects[1] == NONE ? -1 : static_cast<int>(layers[l].objects[1]),
                         layerPresent(s, l) ? "" : "[absent]");
          }
        }
      }
      std::fprintf(stderr, "\n");
    }
    // Every vertex near the first edge.
    {
      const auto& pa = s.cdt.vertices[e1.v1() + SUPER];
      const auto& pb = s.cdt.vertices[e1.v2() + SUPER];
      const auto x0 = std::min(pa.x, pb.x) - 0.05;
      const auto x1 = std::max(pa.x, pb.x) + 0.05;
      const auto y0 = std::min(pa.y, pb.y) - 0.05;
      const auto y1 = std::max(pa.y, pb.y) + 0.05;
      std::fprintf(stderr, " vertices near edge 1:\n");
      for (u32 v = SUPER; v < s.cdt.vertices.size(); ++v) {
        const auto& q = s.cdt.vertices[v];
        if (q.x < x0 || q.x > x1 || q.y < y0 || q.y > y1) {
          continue;
        }
        const auto g = gidOfVertex(v);
        std::fprintf(stderr, "  v%u (%.17g, %.17g) gid %u", v, q.x, q.y, g);
        if (g != NONE) {
          std::fprintf(stderr, " unmoved (%.17g, %.17g) hair %d layers:", gpos[g].x, gpos[g].y,
                       static_cast<int>(hair[g]));
          for (const auto& entry : entries[g]) {
            std::fprintf(stderr, " %u", entry.layer);
          }
        } else {
          std::fprintf(stderr, " (steiner)");
        }
        std::fprintf(stderr, "\n");
      }
    }
    // Inverted or flat triangles of the parent's triangulation.
    std::size_t bad = 0;
    for (u32 t = 0; t < s.cdt.triangles.size(); ++t) {
      const auto& v = s.cdt.triangles[t].vertices;
      const auto& p0 = s.cdt.vertices[v[0]];
      const auto& p1 = s.cdt.vertices[v[1]];
      const auto& p2 = s.cdt.vertices[v[2]];
      const auto cross = ((p1.x - p0.x) * (p2.y - p0.y)) - ((p1.y - p0.y) * (p2.x - p0.x));
      if (cross <= 0.0) {
        ++bad;
        if (bad <= 5) {
          std::fprintf(stderr, " triangle %u not counter-clockwise (%.3g): (%.6f,%.6f) (%.6f,%.6f) (%.6f,%.6f) gids %u %u %u\n",
                       t, cross, p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, gidOfVertex(v[0]),
                       gidOfVertex(v[1]), gidOfVertex(v[2]));
        }
      }
    }
    std::fprintf(stderr, " %zu of %zu triangles not counter-clockwise\n", bad,
                 s.cdt.triangles.size());
  }

  [[nodiscard]] std::shared_ptr<State> extend(const State& parent,
                                              std::span<const u32> list) const {
    std::vector<u32> fresh;
    for (const auto o : list) {
      if (o >= objects.size()) {
        throw std::out_of_range("CdtWorld: no such object");
      }
      if (!presentIn(parent, o)) {
        fresh.push_back(o);
      }
    }
    std::ranges::sort(fresh);
    fresh.erase(std::unique(fresh.begin(), fresh.end()), fresh.end());
    const auto c0 = Clock::now();
    auto s = copyOf(parent);
    s->stats = {};
    s->result = {};
    const auto c1 = Clock::now();
    auto& st = s->stats;
    st.copy = msSince(c0, c1);
    s->present.resize(objects.size(), 0);
    for (const auto o : fresh) {
      s->present[o] = 1;
      listInSquares(*s, o);
    }
    ensurePairs(fresh, s->present);
    st.pairs = msSince(c1, Clock::now());
    s->objects.clear();
    for (u32 o = 0; o < objects.size(); ++o) {
      if (s->present[o] != 0) {
        s->objects.push_back(o);
      }
    }
    // The layers that come in: the new objects', and those of the foot
    // points between a new object and one that is there.
    std::vector<u32> incoming;
    for (const auto o : fresh) {
      incoming.push_back(objects[o].layer);
      for (const auto l : objects[o].pairs) {
        if (layerPresent(*s, l)) {
          incoming.push_back(l);
        }
      }
    }
    std::ranges::sort(incoming);
    incoming.erase(std::unique(incoming.begin(), incoming.end()), incoming.end());
    s->vidOf.resize(gpos.size(), NONE);
    std::vector<CDT::V2d<double>> points;
    std::vector<u32> added;
    const auto base = static_cast<u32>(s->cdt.vertices.size());
    for (const auto l : incoming) {
      for (const auto g : layers[l].gids) {
        if (s->vidOf[g] != NONE) {
          continue;
        }
        s->vidOf[g] = base + static_cast<u32>(points.size());
        points.emplace_back(gplaced[g].x, gplaced[g].y);
        added.push_back(g);
      }
    }
    // Every edge once: two layers that stand together may share a piece
    // (two objects whose cells overlap have the same outline there), and
    // one the parent already has is not inserted again. A piece inserted
    // twice is harmless where it is whole, but where the first insertion
    // split it at a crossing, the second runs a hair past the split vertex
    // and CDT cannot resolve the crossing again.
    std::vector<CDT::Edge> edges;
    for (const auto l : incoming) {
      for (const auto& [a, b] : layers[l].edges) {
        bool parentHas = false;
        for (const auto& [other, layer] : constraintsAt[a]) {
          if (other == b && layerPresent(parent, layer)) {
            parentHas = true;
            break;
          }
        }
        if (!parentHas) {
          edges.emplace_back(s->vidOf[a] - SUPER, s->vidOf[b] - SUPER);
        }
      }
    }
    std::ranges::sort(edges, [](const CDT::Edge& x, const CDT::Edge& y) {
      return std::pair{x.v1(), x.v2()} < std::pair{y.v1(), y.v2()};
    });
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    const auto cv = Clock::now();
    if (!points.empty()) {
      s->cdt.insertVertices(points);
    }
    const auto ce = Clock::now();
    st.vertices = msSince(cv, ce);
    if (!edges.empty()) {
      try {
        const SnapScope snapping(s->cdt, placement);
        s->cdt.insertEdges(edges);
      } catch (const CDT::InvalidEdgeSplitVertex& error) {
        if (std::getenv("MQT_SCPD_CDT_DEBUG_REFUSALS") != nullptr) {
          describeRefusal(*s, base, added, error.e1(), error.e2(), error.what());
        }
        throw;
      } catch (const CDT::IntersectingConstraintsError& error) {
        if (std::getenv("MQT_SCPD_CDT_DEBUG_REFUSALS") != nullptr) {
          describeRefusal(*s, base, added, error.e1(), error.e2(), error.what());
        }
        throw;
      }
    }
    const auto c2 = Clock::now();
    st.edges = msSince(ce, c2);
    st.insert = msSince(c1, c2);

    // The patch: the triangles the insertion changed, those around every
    // point near a new object (its groups change, and a triangle inside it
    // may not change and still stop being free), and their neighbours.
    const auto nv = s->cdt.vertices.size();
    s->gidOf.resize(nv, NONE);
    for (std::size_t k = 0; k < added.size(); ++k) {
      s->gidOf[base + k] = added[k];
    }
    canonicalSteiner(*s, static_cast<u32>(base + added.size()));
    markSplitEnds(*s);
    beginGroupsCache(*s);
    st.newVertices = added.size();
    st.steinerVertices = nv - base - added.size();
    const auto pn = static_cast<u32>(parent.cdt.triangles.size());
    const auto n = static_cast<u32>(s->cdt.triangles.size());
    std::vector<u32> dirty;
    for (u32 t = 0; t < pn; ++t) {
      if (std::memcmp(&s->cdt.triangles[t], &parent.cdt.triangles[t],
                      sizeof(CDT::Triangle)) != 0) {
        dirty.push_back(t);
      }
    }
    for (u32 t = pn; t < n; ++t) {
      dirty.push_back(t);
    }
    st.changedTriangles = dirty.size();
    s->vertTri.resize(nv, NONE);
    for (const auto t : dirty) {
      for (const auto v : s->cdt.triangles[t].vertices) {
        s->vertTri[v] = t;
      }
    }
    s->freeT.resize(n, 0);
    s->slot.resize(static_cast<std::size_t>(n) * 3, NONE);
    s->chamber.resize(n, NONE);
    std::vector<std::uint8_t> inC(n, 0);
    std::vector<u32> patch = dirty;
    for (const auto t : patch) {
      inC[t] = 1;
    }
    std::vector<u32> around;
    const auto take = [&](const u32 v) {
      around.clear();
      walkAround(*s, v, around);
      for (const auto t : around) {
        if (inC[t] == 0) {
          inC[t] = 1;
          patch.push_back(t);
        }
      }
    };
    for (const auto o : fresh) {
      for (const auto g : objects[o].near) {
        if (g < s->vidOf.size() && s->vidOf[g] != NONE) {
          take(s->vidOf[g]);
        }
      }
    }
    // The points the triangulation added take the walls within a cell.
    for (u32 v = SUPER; v < nv; ++v) {
      if (s->gidOf[v] != NONE) {
        continue;
      }
      const auto& p = s->cdt.vertices[v];
      for (const auto o : fresh) {
        const auto& object = objects[o];
        if (p.x >= static_cast<double>(object.x0) - 2.0 &&
            p.y >= static_cast<double>(object.y0) - 2.0 &&
            p.x <= static_cast<double>(object.x1) + 3.0 &&
            p.y <= static_cast<double>(object.y1) + 3.0) {
          take(v);
          break;
        }
      }
    }
    {
      const auto upto = patch.size();
      for (std::size_t i = 0; i < upto; ++i) {
        for (const auto o : s->cdt.triangles[patch[i]].neighbors) {
          if (o != CDT::noNeighbor && inC[o] == 0) {
            inC[o] = 1;
            patch.push_back(o);
          }
        }
      }
    }
    const auto c3 = Clock::now();
    st.patch = msSince(c2, c3);
    st.reclassified = patch.size();

    // Classify; remember the chambers the patch held.
    std::vector<u32> oldChambers;
    for (const auto t : patch) {
      if (s->chamber[t] != NONE) {
        oldChambers.push_back(s->chamber[t]);
      }
      s->freeT[t] = classify(*s, t);
    }
    const auto c4 = Clock::now();
    st.classify = msSince(c3, c4);

    // The candidates on every edge of the patch.
    const auto poolBefore = static_cast<u32>(s->pool.size());
    for (const auto t : patch) {
      const auto& tr = s->cdt.triangles[t];
      for (std::size_t k = 0; k < 3; ++k) {
        const auto o = tr.neighbors[k];
        if (o != CDT::noNeighbor && inC[o] != 0 && o < t) {
          continue;
        }
        evaluateInto(*s, t, k);
      }
    }
    std::vector<u32> seeds;
    for (const auto t : patch) {
      for (std::size_t k = 0; k < 3; ++k) {
        seeds.push_back(s->slot[(t * 3) + k]);
        const auto o = s->cdt.triangles[t].neighbors[k];
        if (o != CDT::noNeighbor && inC[o] == 0) {
          for (std::size_t j = 0; j < 3; ++j) {
            seeds.push_back(s->slot[(o * 3) + j]);
          }
        }
      }
    }
    const auto c5 = Clock::now();
    st.candidates = msSince(c4, c5);
    std::vector<u32> touched;
    basinsFrom(*s, seeds, touched, st);
    const auto c6 = Clock::now();
    st.basins = msSince(c5, c6);

    // Measure the gates that came, and those whose cells the new objects may
    // have changed: an end within four cells of one's box.
    for (const auto c : touched) {
      auto& cand = s->pool[c];
      if (cand.gate != 0 && cand.measured == 0) {
        measure(*s, cand);
      }
    }
    const auto nearNew = [&](const double x, const double y) {
      for (const auto o : fresh) {
        const auto& object = objects[o];
        if (x >= static_cast<double>(object.x0) - 4.0 &&
            y >= static_cast<double>(object.y0) - 4.0 &&
            x <= static_cast<double>(object.x1) + 5.0 &&
            y <= static_cast<double>(object.y1) + 5.0) {
          return true;
        }
      }
      return false;
    };
    for (const auto c : parent.gateList) {
      auto& cand = s->pool[c];
      if (cand.dead != 0 || cand.gate == 0) {
        continue;
      }
      const auto pa = at(*s, cand.a);
      const auto pb = at(*s, cand.b);
      if (nearNew(pa.x, pa.y) || nearNew(pb.x, pb.y)) {
        measure(*s, cand);
        touched.push_back(c);
      }
    }
    std::vector<u32> flipped;
    for (const auto c : touched) {
      auto& cand = s->pool[c];
      const std::uint8_t sep = separates(cand) ? 1 : 0;
      if (cand.sep != sep) {
        if (c < poolBefore) {
          flipped.push_back(cand.t1);
          flipped.push_back(cand.t2);
        }
        cand.sep = sep;
      }
    }
    {
      // The gates now: the parent's that still are, and those that came.
      std::vector<u32> gates;
      gates.reserve(parent.gateList.size() + 64);
      for (const auto c : parent.gateList) {
        const auto& cand = s->pool[c];
        if (cand.dead == 0 && cand.gate != 0) {
          gates.push_back(c);
        }
      }
      for (const auto c : touched) {
        const auto& cand = s->pool[c];
        if (cand.dead == 0 && cand.gate != 0) {
          gates.push_back(c);
        }
      }
      std::ranges::sort(gates);
      gates.erase(std::unique(gates.begin(), gates.end()), gates.end());
      s->gateList = std::move(gates);
    }
    const auto c7 = Clock::now();
    st.measure = msSince(c6, c7);

    // The chambers: those the patch or a gate that came or went touched are
    // labelled again, merging with the untouched ones they now meet.
    for (const auto t : flipped) {
      if (t < s->chamber.size() && s->chamber[t] != NONE) {
        oldChambers.push_back(s->chamber[t]);
      }
    }
    std::vector<std::uint8_t> hit(s->chamberKey.size(), 0);
    for (const auto c : oldChambers) {
      hit[c] = 1;
      s->chamberAlive[c] = 0;
    }
    std::vector<u32> reset;
    for (u32 t = 0; t < n; ++t) {
      const auto c = s->chamber[t];
      if ((c != NONE && hit[c] != 0) || inC[t] != 0) {
        s->chamber[t] = NONE;
        reset.push_back(t);
      }
    }
    st.relabelled = reset.size();
    std::vector<u32> members;
    std::vector<u32> merged;
    for (const auto t : reset) {
      if (s->freeT[t] == 0 || s->chamber[t] != NONE) {
        continue;
      }
      const auto id = static_cast<u32>(s->chamberKey.size());
      ChamberKey key;
      grow(*s, t, id, members, merged, key);
      if (merged.empty()) {
        s->chamberKey.push_back(key);
        s->chamberAlive.push_back(1);
        continue;
      }
      std::ranges::sort(merged);
      merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
      const auto keep = merged.front();
      for (const auto u : members) {
        s->chamber[u] = keep;
      }
      s->chamberKey.push_back(key);
      s->chamberAlive.push_back(0);
      auto& kept = s->chamberKey[keep];
      kept = std::min(kept, key);
      if (merged.size() > 1) {
        for (std::size_t k = 1; k < merged.size(); ++k) {
          kept = std::min(kept, s->chamberKey[merged[k]]);
          s->chamberAlive[merged[k]] = 0;
        }
        for (u32 u = 0; u < n; ++u) {
          if (s->chamber[u] != NONE &&
              std::ranges::binary_search(merged, s->chamber[u])) {
            s->chamber[u] = keep;
          }
        }
      }
    }
    const auto c8 = Clock::now();
    st.chambers = msSince(c7, c8);
    compact(*s);
    output(*s);
    const auto c9 = Clock::now();
    st.output = msSince(c8, c9);
    st.total = msSince(c0, c9);
    st.objects = s->objects.size();
    return s;
  }

  /// Drops the dead candidates and chambers once they outnumber the live
  /// ones: the indices change, nothing else does.
  static void compact(State& s) {
    if (s.deadCandidates > 4096 && s.deadCandidates * 2 > s.pool.size()) {
      std::vector<u32> newIndex(s.pool.size(), NONE);
      std::vector<Candidate> kept;
      kept.reserve(s.pool.size() - s.deadCandidates);
      for (u32 c = 0; c < s.pool.size(); ++c) {
        if (s.pool[c].dead == 0) {
          newIndex[c] = static_cast<u32>(kept.size());
          kept.push_back(s.pool[c]);
        }
      }
      for (auto& c : s.slot) {
        if (c != NONE) {
          c = newIndex[c];
        }
      }
      for (auto& c : s.gateList) {
        c = newIndex[c];
      }
      s.pool = std::move(kept);
      s.deadCandidates = 0;
    }
    std::size_t alive = 0;
    for (const auto a : s.chamberAlive) {
      alive += a;
    }
    if (s.chamberKey.size() > 4096 && s.chamberKey.size() > 4 * alive) {
      std::vector<u32> newIndex(s.chamberKey.size(), NONE);
      std::vector<ChamberKey> keys;
      for (u32 c = 0; c < s.chamberKey.size(); ++c) {
        if (s.chamberAlive[c] != 0) {
          newIndex[c] = static_cast<u32>(keys.size());
          keys.push_back(s.chamberKey[c]);
        }
      }
      for (auto& c : s.chamber) {
        if (c != NONE) {
          c = newIndex[c];
        }
      }
      s.chamberKey = std::move(keys);
      s.chamberAlive.assign(s.chamberKey.size(), 1);
    }
  }

  /// The result of a state: the gates in canonical order, the chambers
  /// numbered by their keys, and the chambers beside every gate.
  void output(State& s) const {
    std::vector<u32> live;
    for (u32 c = 0; c < s.chamberKey.size(); ++c) {
      if (s.chamberAlive[c] != 0) {
        live.push_back(c);
      }
    }
    std::ranges::sort(live, [&](const u32 a, const u32 b) {
      return s.chamberKey[a] < s.chamberKey[b];
    });
    s.chamberRank.assign(s.chamberKey.size(), NO_CHAMBER);
    for (u32 k = 0; k < live.size(); ++k) {
      s.chamberRank[live[k]] = k;
    }
    auto& r = s.result;
    r = {};
    r.chambers = static_cast<std::uint32_t>(live.size());
    std::vector<u32> gates;
    for (const auto c : s.gateList) {
      if (s.pool[c].valid != 0) {
        gates.push_back(c);
      }
    }
    std::ranges::sort(gates, [&](const u32 a, const u32 b) {
      const auto& x = s.pool[a];
      const auto& y = s.pool[b];
      const auto kx = std::tuple{std::min(x.first, x.second),
                                 std::max(x.first, x.second), x.saddle};
      const auto ky = std::tuple{std::min(y.first, y.second),
                                 std::max(y.first, y.second), y.saddle};
      if (kx != ky) {
        return kx < ky;
      }
      return keyOf(s, x) < keyOf(s, y);
    });
    const auto n = gates.size();
    r.gates.reserve(n);
    r.cuts.reserve(n);
    r.capacity.reserve(n);
    r.width.reserve(n);
    r.ends.reserve(n);
    r.beside.reserve(n);
    for (const auto c : gates) {
      const auto& cand = s.pool[c];
      r.gates.push_back(
          {.first = cand.first, .second = cand.second, .saddle = cand.saddle});
      r.cuts.push_back(cand.cuts);
      r.capacity.push_back(cand.cap);
      r.width.push_back(cand.width);
      const auto pa = at(s, cand.a);
      const auto pb = at(s, cand.b);
      r.ends.push_back({pa.x, pa.y, pb.x, pb.y});
      const auto rankOf = [&](const u32 t) {
        const auto ch = s.chamber[t];
        return ch == NONE ? NO_CHAMBER : s.chamberRank[ch];
      };
      auto one = rankOf(cand.t1);
      auto two = rankOf(cand.t2);
      if (two < one) {
        std::swap(one, two);
      }
      r.beside.push_back({one, one == two ? NO_CHAMBER : two});
    }
  }

  // ================================================================ cache

  /// The entry of the cache whose objects are the largest subset of `want`,
  /// or null; the most recently used among equals.
  [[nodiscard]] Cached* largestSubset(const std::vector<u32>& want) {
    Cached* best = nullptr;
    for (auto& entry : cache) {
      if (entry.objects.size() > want.size() ||
          !std::ranges::includes(want, entry.objects)) {
        continue;
      }
      if (best == nullptr || entry.objects.size() > best->objects.size() ||
          (entry.objects.size() == best->objects.size() &&
           entry.used > best->used)) {
        best = &entry;
      }
    }
    return best;
  }

  /// Puts a state into the cache, the least recently used making room.
  Cached* remember(std::vector<u32> objects, StatePtr state) {
    if (cacheSize == 0) {
      return nullptr;
    }
    if (cache.size() >= cacheSize) {
      cache.erase(std::ranges::min_element(
          cache, {}, [](const Cached& entry) { return entry.used; }));
    }
    cache.push_back({.objects = std::move(objects), .state = std::move(state),
                     .used = ++clock});
    return &cache.back();
  }

  /// **The state of a set of objects through the cache.** A state of the
  /// set is the answer. Else the state is built from the cached state whose
  /// objects are the largest subset (the base if none) — but first, where a
  /// state of the cache shares more objects with the set than that subset
  /// has (a sibling: the same chain a step on, with one coupler on another
  /// option), the common part is built and kept as a state of its own, and
  /// the set is built from it. The siblings that follow then find the common
  /// part and insert only what is theirs. The common part costs what it
  /// would have cost on the way anyway, up to `SYNTH_SLACK` objects.
  static constexpr std::size_t SYNTH_SLACK = 2;

  StatePtr cached(std::span<const u32> list) {
    std::vector<u32> want(list.begin(), list.end());
    std::ranges::sort(want);
    want.erase(std::unique(want.begin(), want.end()), want.end());
    Cached* best = largestSubset(want);
    if (best != nullptr && best->objects.size() == want.size()) {
      ++cacheStats.hits;
      best->used = ++clock;
      return best->state;
    }
    const auto bestSize = best != nullptr ? best->objects.size() : 0;
    // A sibling that shares more than the best subset: its common part.
    std::vector<u32> common;
    std::vector<u32> share;
    std::size_t shared = bestSize;
    for (const auto& entry : cache) {
      if (entry.objects.size() <= shared) {
        continue;
      }
      common.clear();
      std::ranges::set_intersection(entry.objects, want, std::back_inserter(common));
      if (common.size() > shared && common.size() < want.size()) {
        shared = common.size();
        share = common;
      }
    }
    if (shared > bestSize) {
      Cached* shareParent = largestSubset(share);
      const auto shareParentSize =
          shareParent != nullptr ? shareParent->objects.size() : 0;
      const auto direct = want.size() - bestSize;
      const auto viaShare = (share.size() - shareParentSize) + (want.size() - share.size());
      if (viaShare <= direct + SYNTH_SLACK) {
        StatePtr parent = shareParent != nullptr ? shareParent->state : baseState;
        if (shareParent != nullptr) {
          shareParent->used = ++clock;
        }
        cacheStats.inserted += share.size() - shareParentSize;
        ++cacheStats.synthesized;
        StatePtr made = extend(*parent, share);
        best = remember(share, made);
        if (best == nullptr) {
          // No cache: build the set from the common part directly.
          cacheStats.inserted += want.size() - share.size();
          return extend(*made, want);
        }
      }
    }
    StatePtr parent = best != nullptr ? best->state : baseState;
    if (best != nullptr) {
      ++cacheStats.extended;
      best->used = ++clock;
    } else {
      ++cacheStats.fromBase;
    }
    cacheStats.inserted += want.size() - (best != nullptr ? best->objects.size() : 0);
    StatePtr made = extend(*parent, want);
    remember(want, made);
    return made;
  }
};

// ================================================================ public

namespace {

/// An object's key and walls from the labels a raster gives them, without
/// its cells.
CdtObject objectHeadOf(const std::uint64_t key,
                       const std::span<const CdtWall> walls,
                       const std::span<const std::uint32_t> labels,
                       const std::span<const std::uint8_t> cuts) {
  CdtObject object;
  object.key = key;
  // A wall's group is the first of the labels whose wall has that group.
  const auto groupValue = [&](const std::uint32_t label) {
    return label < walls.size() ? walls[label].group : label;
  };
  std::unordered_map<std::uint32_t, std::uint32_t> groupLocal;
  for (std::uint32_t k = 0; k < labels.size(); ++k) {
    groupLocal.try_emplace(groupValue(labels[k]), k);
  }
  for (const auto label : labels) {
    CdtWall wall = label < walls.size() ? walls[label] : CdtWall{};
    wall.group = groupLocal.at(groupValue(label));
    object.walls.push_back(wall);
    object.cuts.push_back(
        cuts.empty() || (label < cuts.size() && cuts[label] != 0) ? 1 : 0);
  }
  return object;
}

} // namespace

CdtObject cdtObjectOf(const std::uint64_t key, const BitGrid& mask,
                      const std::span<const std::uint32_t> owner,
                      const std::span<const CdtWall> walls,
                      const std::span<const std::uint32_t> labels,
                      const std::span<const std::uint8_t> cuts) {
  auto object = objectHeadOf(key, walls, labels, cuts);
  std::unordered_map<std::uint32_t, std::uint32_t> local;
  for (std::uint32_t k = 0; k < labels.size(); ++k) {
    local.try_emplace(labels[k], k);
  }
  const auto& words = mask.words();
  const auto cells = mask.size();
  for (std::size_t word = 0; word < words.size(); ++word) {
    for (auto bits = words[word]; bits != 0; bits &= bits - 1) {
      const auto cell =
          (word * 64U) + static_cast<std::size_t>(std::countr_zero(bits));
      if (cell >= cells) {
        break;
      }
      const auto found = local.find(owner[cell]);
      if (found != local.end()) {
        object.cells.push_back(cell);
        object.wallOf.push_back(found->second);
      }
    }
  }
  return object;
}

CdtWorld::CdtWorld(const std::uint32_t width, const std::uint32_t height,
                   const BitGrid& staticMask,
                   const std::span<const std::uint32_t> staticOwner,
                   const std::span<const CdtWall> walls,
                   const std::uint32_t border, const CdtGateOptions& options,
                   const std::span<const std::uint8_t> cuts,
                   const std::size_t cacheSize)
    : impl_(std::make_unique<Impl>()) {
  impl_->width = width;
  impl_->height = height;
  impl_->options = options;
  impl_->cacheSize = cacheSize;
  impl_->build(staticMask, staticOwner, walls, border, cuts);
}

CdtWorld::~CdtWorld() = default;
CdtWorld::CdtWorld(CdtWorld&&) noexcept = default;
CdtWorld& CdtWorld::operator=(CdtWorld&&) noexcept = default;

std::array<double, 10> CdtWorld::addSeconds() const { return impl_->addSeconds; }

std::uint32_t CdtWorld::add(const CdtObject& object) {
  return impl_->addObject(object);
}

std::optional<std::uint32_t> CdtWorld::find(const std::uint64_t key) const {
  const auto found = impl_->objectOfKey.find(key);
  if (found == impl_->objectOfKey.end()) {
    return std::nullopt;
  }
  return found->second;
}

std::size_t CdtWorld::objectCount() const { return impl_->objects.size(); }

CdtWorld::StatePtr CdtWorld::base() const { return impl_->baseState; }

CdtWorld::StatePtr CdtWorld::with(const StatePtr& parent,
                                  const std::span<const std::uint32_t> objects) const {
  const auto& from = parent != nullptr ? *parent : *impl_->baseState;
  bool any = false;
  for (const auto o : objects) {
    any = any || !Impl::presentIn(from, o);
  }
  if (!any && parent != nullptr) {
    return parent;
  }
  return impl_->extend(from, objects);
}

CdtWorld::StatePtr
CdtWorld::fromScratch(const std::span<const std::uint32_t> objects) const {
  return impl_->scratch(objects);
}

CdtWorld::StatePtr CdtWorld::state(const std::span<const std::uint32_t> objects) {
  return impl_->cached(objects);
}

CdtWorld::StatePtr CdtWorld::state(const BitGrid& mask,
                                   const std::span<const std::uint32_t> owner,
                                   const std::span<const CdtWall> walls,
                                   const std::span<const CdtObjectRef> refs,
                                   const std::span<const std::uint8_t> cuts) {
  std::vector<std::uint32_t> list;
  std::vector<std::size_t> missing;
  for (std::size_t k = 0; k < refs.size(); ++k) {
    if (const auto found = find(refs[k].key); found.has_value()) {
      list.push_back(*found);
    } else {
      missing.push_back(k);
    }
  }
  if (!missing.empty()) {
    // One pass over the raster for every new object.
    std::vector<std::uint32_t> refOf(walls.size() + 1, NONE);
    std::vector<std::uint32_t> localOf(walls.size() + 1, NONE);
    std::vector<CdtObject> made(missing.size());
    for (std::size_t m = 0; m < missing.size(); ++m) {
      const auto& ref = refs[missing[m]];
      made[m] = objectHeadOf(ref.key, walls, ref.labels, cuts);
      for (std::uint32_t k = 0; k < ref.labels.size(); ++k) {
        const auto label = std::min<std::size_t>(ref.labels[k], walls.size());
        refOf[label] = static_cast<std::uint32_t>(m);
        localOf[label] = k;
      }
    }
    const auto& words = mask.words();
    const auto cells = mask.size();
    for (std::size_t word = 0; word < words.size(); ++word) {
      for (auto bits = words[word]; bits != 0; bits &= bits - 1) {
        const auto cell = (word * 64U) + static_cast<std::size_t>(std::countr_zero(bits));
        if (cell >= cells) {
          break;
        }
        const auto label = std::min<std::size_t>(owner[cell], walls.size());
        const auto m = refOf[label];
        if (m != NONE) {
          made[m].cells.push_back(cell);
          made[m].wallOf.push_back(localOf[label]);
        }
      }
    }
    for (auto& object : made) {
      list.push_back(add(object));
    }
  }
  return impl_->cached(list);
}

std::string CdtWorld::describeState(const State& state) const {
  const auto& cdt = state.cdt;
  std::size_t dead = 0;
  for (const auto& c : state.pool) {
    dead += c.dead;
  }
  std::size_t pieces = 0;
  for (const auto& [piece, originals] : cdt.pieceToOriginals) {
    pieces += originals.size();
  }
  char buffer[512];
  std::snprintf(buffer, sizeof buffer,
                "vertices %zu, triangles %zu, fixedEdges %zu, pieceToOriginals %zu (%zu originals), "
                "overlapCount %zu, vertTris %zu; gidOf %zu, vidOf %zu, present %zu, objects %zu, "
                "squareHead %zu, squareNodes %zu, freeT %zu, slot %zu, pool %zu (%zu dead), gateList %zu, "
                "chamber %zu, chamberKey %zu, result gates %zu",
                cdt.vertices.size(), cdt.triangles.size(), cdt.fixedEdges.size(),
                cdt.pieceToOriginals.size(), pieces, cdt.overlapCount.size(),
                cdt.VertTrisInternal().size(), state.gidOf.size(), state.vidOf.size(),
                state.present.size(), state.objects.size(), state.squareHead.size(),
                state.squareNodes.size(), state.freeT.size(), state.slot.size(),
                state.pool.size(), dead, state.gateList.size(), state.chamber.size(),
                state.chamberKey.size(), state.result.gates.size());
  return buffer;
}

const CdtWorld::Result& CdtWorld::result(const State& state) const {
  return state.result;
}

const std::vector<std::uint32_t>& CdtWorld::objectsOf(const State& state) const {
  return state.objects;
}

const CdtStepStats& CdtWorld::statsOf(const State& state) const {
  return state.stats;
}

const CdtWorldCacheStats& CdtWorld::cacheStats() const {
  return impl_->cacheStats;
}

const CdtWorld::BuildStats& CdtWorld::buildStats() const {
  return impl_->buildStats;
}

void CdtWorld::chambers(const State& state, Chambers& into) const {
  const auto& im = *impl_;
  const auto w = static_cast<std::int64_t>(im.width);
  const auto h = static_cast<std::int64_t>(im.height);
  const auto cells = static_cast<std::size_t>(im.width) * im.height;
  // The walls of the state as one raster.
  BitGrid walls = im.staticMask;
  for (const auto o : state.objects) {
    const auto& object = im.objects[o];
    for (auto y = object.y0; y <= object.y1; ++y) {
      for (auto x = object.x0; x <= object.x1; ++x) {
        if (object.wallAt(x, y) != NONE) {
          walls.set(static_cast<std::size_t>((y * w) + x));
        }
      }
    }
  }
  into.of.assign(cells, NO_CHAMBER);
  const auto& cdt = state.cdt;
  const auto& vs = cdt.vertices;
  for (std::uint32_t t = 0; t < cdt.triangles.size(); ++t) {
    if (state.freeT[t] == 0) {
      continue;
    }
    const auto ch = state.chamber[t];
    if (ch == NONE || state.chamberRank[ch] == NO_CHAMBER) {
      continue;
    }
    const auto rank = state.chamberRank[ch];
    const auto& v = cdt.triangles[t].vertices;
    const auto& a = vs[v[0]];
    const auto& b = vs[v[1]];
    const auto& c = vs[v[2]];
    // Every row of cell centres the triangle spans: the span of x where the
    // row meets it, a hair wider so that a centre on a shared edge is in
    // both triangles and takes the lower number.
    const auto lo = std::min({a.y, b.y, c.y});
    const auto hi = std::max({a.y, b.y, c.y});
    const auto y0 = std::max<std::int64_t>(0, static_cast<std::int64_t>(std::ceil(lo - 0.5 - 1e-9)));
    const auto y1 = std::min<std::int64_t>(h - 1, static_cast<std::int64_t>(std::floor(hi - 0.5 + 1e-9)));
    for (auto y = y0; y <= y1; ++y) {
      const auto py = static_cast<double>(y) + 0.5;
      double xl = std::numeric_limits<double>::max();
      double xr = std::numeric_limits<double>::lowest();
      for (const auto& [p, q] : {std::pair{&a, &b}, std::pair{&b, &c}, std::pair{&c, &a}}) {
        const auto ylo = std::min(p->y, q->y);
        const auto yhi = std::max(p->y, q->y);
        if (py < ylo - 1e-9 || py > yhi + 1e-9) {
          continue;
        }
        if (yhi - ylo < 1e-12) {
          xl = std::min({xl, p->x, q->x});
          xr = std::max({xr, p->x, q->x});
          continue;
        }
        const auto x = p->x + ((py - p->y) * (q->x - p->x) / (q->y - p->y));
        xl = std::min(xl, x);
        xr = std::max(xr, x);
      }
      if (xr < xl) {
        continue;
      }
      const auto x0 = std::max<std::int64_t>(0, static_cast<std::int64_t>(std::ceil(xl - 0.5 - 1e-9)));
      const auto x1 = std::min<std::int64_t>(w - 1, static_cast<std::int64_t>(std::floor(xr - 0.5 + 1e-9)));
      for (auto x = x0; x <= x1; ++x) {
        const auto cell = static_cast<std::size_t>((y * w) + x);
        if (!walls.test(cell) && rank < into.of[cell]) {
          into.of[cell] = rank;
        }
      }
    }
  }
  into.count = state.result.chambers;
  into.beside.clear();
  into.beside.reserve(state.result.beside.size());
  for (const auto& [one, two] : state.result.beside) {
    auto& list = into.beside.emplace_back();
    for (const auto c : {one, two}) {
      if (c != NO_CHAMBER) {
        list.push_back(c);
      }
    }
  }
}

std::uint32_t CdtWorld::chamberAt(const State& state, const std::uint32_t x,
                                  const std::uint32_t y) const {
  const auto& im = *impl_;
  if (x >= im.width || y >= im.height ||
      im.wallCell(state, static_cast<std::int64_t>(x), static_cast<std::int64_t>(y))) {
    return NO_CHAMBER;
  }
  const auto px = static_cast<double>(x) + 0.5;
  const auto py = static_cast<double>(y) + 0.5;
  const auto t = Impl::locate(state.cdt, px, py, im.hintAt(px, py));
  if (state.freeT[t] == 0 || state.chamber[t] == NONE) {
    return NO_CHAMBER;
  }
  return state.chamberRank[state.chamber[t]];
}

std::vector<std::array<double, 7>>
CdtWorld::canonicalTriangles(const State& state) const {
  std::vector<std::array<double, 7>> out;
  const auto& cdt = state.cdt;
  for (std::uint32_t t = 0; t < cdt.triangles.size(); ++t) {
    if (state.freeT[t] == 0) {
      continue;
    }
    std::array<std::pair<double, double>, 3> corners;
    for (std::size_t k = 0; k < 3; ++k) {
      const auto& p = cdt.vertices[cdt.triangles[t].vertices[k]];
      corners[k] = {p.y, p.x};
    }
    std::ranges::sort(corners);
    const auto ch = state.chamber[t];
    out.push_back({corners[0].first, corners[0].second, corners[1].first,
                   corners[1].second, corners[2].first, corners[2].second,
                   ch == NONE ? -1.0 : static_cast<double>(state.chamberRank[ch])});
  }
  std::ranges::sort(out);
  return out;
}

} // namespace mqt::scpd::grid
