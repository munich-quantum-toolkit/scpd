/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/ProfileCuts.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

/// The eight steps around a cell, in turning order.
constexpr std::array<int, 8> TURN_DX = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr std::array<int, 8> TURN_DY = {0, 1, 1, 1, 0, -1, -1, -1};

constexpr double INFINITE = std::numeric_limits<double>::infinity();

/// The free cell a trace starts from: left of the first cell of a piece.
constexpr std::size_t START_FROM = 4;

/// How far a simplified line may stray from the cells it stands for.
constexpr double SIMPLIFY_TOLERANCE = 1.0;

using Point = std::pair<double, double>;

/// A dynamic wall: its outlines, and a line its gap to another dynamic wall
/// is measured from.
struct DynamicWall {
  std::uint32_t wall = NO_WALL;
  std::vector<std::size_t> cells;
  /// Each closed outline: the cells of the wall beside a free cell, in
  /// order around the wall.
  std::vector<std::vector<std::size_t>> outlines;
  /// The line, dense (a point per cell) and simplified.
  std::vector<Point> dense;
  std::vector<Point> simple;
  bool closed = false;
  double radius = 0.0;
  double minX = INFINITE;
  double minY = INFINITE;
  double maxX = -INFINITE;
  double maxY = -INFINITE;
};

/// A gap along a wall: its width at every place, and which wall and which
/// point across it is measured to.
struct Profile {
  std::vector<double> gap;
  std::vector<std::uint32_t> partner;
  bool closed = false;
};

/// The places along a profile where the gap has a persistent minimum: the
/// gap rises by `rise` on both sides before it falls below, and a stretch of
/// one width counts once, at its middle. A place without a gap ends a walk
/// as a rise does, and so does a change of the wall across where the gap
/// does not fall; the end of an open profile ends it without one.
std::vector<std::size_t> narrowings(const Profile& profile, const double rise) {
  const auto& gap = profile.gap;
  const auto n = gap.size();
  std::vector<std::size_t> found;
  if (n < 3) {
    return found;
  }
  const auto step = [&](const std::size_t at, const bool forward,
                        std::size_t& next) {
    if (forward) {
      if (at + 1 < n) {
        next = at + 1;
        return true;
      }
      next = 0;
      return profile.closed;
    }
    if (at > 0) {
      next = at - 1;
      return true;
    }
    next = n - 1;
    return profile.closed;
  };
  // The blocks a walk may pass in one step: a block of `BLOCK` places to
  // one wall, every gap known, between its lowest and its highest gap.
  constexpr std::size_t BLOCK = 64;
  const auto blocks = (n + BLOCK - 1) / BLOCK;
  std::vector<double> lowest(blocks, INFINITE);
  std::vector<double> highest(blocks, -INFINITE);
  std::vector<std::uint32_t> wallOfBlock(blocks, NO_WALL);
  for (std::size_t block = 0; block < blocks; ++block) {
    const auto from = block * BLOCK;
    const auto to = std::min(n, from + BLOCK);
    bool uniform = to - from == BLOCK;
    for (auto at = from; at < to && uniform; ++at) {
      uniform = std::isfinite(gap[at]) && profile.partner[at] != NO_WALL &&
                profile.partner[at] == profile.partner[from];
      lowest[block] = std::min(lowest[block], gap[at]);
      highest[block] = std::max(highest[block], gap[at]);
    }
    if (uniform) {
      wallOfBlock[block] = profile.partner[from];
    }
  }
  // Whether the gap rises on one side of a stretch of one width that ends
  // at `end`: walking away, the first place without a gap is a rise, the
  // first to another wall a rise unless it is narrower, the first narrower
  // a fall, the first `rise` wider a rise. A block between the two passes
  // whole.
  const auto rises = [&](const std::size_t end, const bool forward,
                         const double width, const std::uint32_t partner) {
    const auto passes = [&](const std::size_t block) {
      return wallOfBlock[block] == partner && lowest[block] >= width - 1e-9 &&
             highest[block] < width + rise;
    };
    std::size_t here = end;
    for (std::size_t walked = 1; walked < n;) {
      std::size_t next = 0;
      if (!step(here, forward, next)) {
        return false;
      }
      const auto block = next / BLOCK;
      const bool atBlockStart =
          forward ? next % BLOCK == 0 : (next % BLOCK == BLOCK - 1);
      if (atBlockStart && walked + BLOCK < n && passes(block)) {
        here = forward ? next + BLOCK - 1 : next + 1 - BLOCK;
        walked += BLOCK;
        continue;
      }
      here = next;
      ++walked;
      const auto other = gap[here];
      if (!std::isfinite(other)) {
        return true;
      }
      if (profile.partner[here] != partner) {
        return other >= width;
      }
      if (other < width - 1e-9) {
        return false;
      }
      if (other >= width + rise) {
        return true;
      }
    }
    return false;
  };
  // Every stretch of one width to one wall is judged once, from its ends.
  std::vector<bool> taken(n, false);
  for (std::size_t at = 0; at < n; ++at) {
    if (!std::isfinite(gap[at]) || taken[at]) {
      continue;
    }
    const auto width = gap[at];
    const auto partner = profile.partner[at];
    const auto same = [&](const std::size_t other) {
      return std::isfinite(gap[other]) && profile.partner[other] == partner &&
             std::abs(gap[other] - width) < 1e-9;
    };
    std::size_t first = at;
    std::size_t last = at;
    std::size_t length = 1;
    for (std::size_t next = 0; length < n && step(first, false, next) &&
                               !taken[next] && same(next);) {
      first = next;
      ++length;
    }
    for (std::size_t next = 0;
         length < n && step(last, true, next) && !taken[next] && same(next);) {
      last = next;
      ++length;
    }
    for (std::size_t k = 0; k < length; ++k) {
      taken[(first + k) % n] = true;
    }
    if (length == n || !rises(first, false, width, partner) ||
        !rises(last, true, width, partner)) {
      continue;
    }
    found.push_back((first + ((length - 1) / 2)) % n);
  }
  return found;
}

/// Douglas-Peucker simplification of a line.
std::vector<Point> simplify(const std::vector<Point>& line) {
  if (line.size() < 3) {
    return line;
  }
  std::vector<bool> keep(line.size(), false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<std::size_t, std::size_t>> pending{
      {0, line.size() - 1}};
  while (!pending.empty()) {
    const auto [from, to] = pending.back();
    pending.pop_back();
    const auto [ax, ay] = line[from];
    const auto [bx, by] = line[to];
    const auto length = std::hypot(bx - ax, by - ay);
    double worst = 0.0;
    std::size_t at = from;
    for (auto k = from + 1; k < to; ++k) {
      const auto [px, py] = line[k];
      const auto off =
          length > 0.0
              ? std::abs(((bx - ax) * (ay - py)) - ((ax - px) * (by - ay))) /
                    length
              : std::hypot(px - ax, py - ay);
      if (off > worst) {
        worst = off;
        at = k;
      }
    }
    if (worst > SIMPLIFY_TOLERANCE) {
      keep[at] = true;
      pending.emplace_back(from, at);
      pending.emplace_back(at, to);
    }
  }
  std::vector<Point> kept;
  for (std::size_t k = 0; k < line.size(); ++k) {
    if (keep[k]) {
      kept.push_back(line[k]);
    }
  }
  return kept;
}

class Search {
public:
  Search(const ProfileScene& scene, const ProfileOptions& options)
      : scene_(scene), options_(options),
        width_(static_cast<std::int64_t>(scene.width)),
        height_(static_cast<std::int64_t>(scene.height)) {}

  std::vector<Bottleneck> run() {
    collectWalls();
    traced_ = BitGrid(scene_.width, scene_.height);
    for (auto& wall : walls_) {
      traceOutlines(wall);
    }
    drawLines();
    for (const auto& wall : walls_) {
      for (const auto& outline : wall.outlines) {
        for (std::size_t field = 0; field < scene_.fields.size(); ++field) {
          againstField(wall, outline, *scene_.fields[field]);
        }
      }
    }
    for (const auto& one : walls_) {
      for (const auto& other : walls_) {
        if (&one != &other) {
          againstWall(one, other);
        }
      }
    }
    closeMouths();
    dropWedges();
    return oneCutPerNarrowing();
  }

private:
  struct Cut {
    std::size_t first = 0;
    std::size_t second = 0;
    double length = 0.0;
  };

  [[nodiscard]] std::int64_t xOf(const std::size_t cell) const {
    return static_cast<std::int64_t>(cell) % width_;
  }
  [[nodiscard]] std::int64_t yOf(const std::size_t cell) const {
    return static_cast<std::int64_t>(cell) / width_;
  }
  [[nodiscard]] bool inside(const std::int64_t x, const std::int64_t y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }
  [[nodiscard]] std::size_t cellAt(const std::int64_t x,
                                   const std::int64_t y) const {
    return static_cast<std::size_t>((y * width_) + x);
  }
  [[nodiscard]] std::uint32_t wallAt(const std::size_t cell) const {
    return scene_.wallOf[cell];
  }
  [[nodiscard]] bool dynamic(const std::uint32_t wall) const {
    return wall != NO_WALL && wall < scene_.roles.size() &&
           scene_.roles[wall] == ProfileRole::Dynamic;
  }
  [[nodiscard]] double distance(const std::size_t a,
                                const std::size_t b) const {
    return std::hypot(static_cast<double>(xOf(a) - xOf(b)),
                      static_cast<double>(yOf(a) - yOf(b)));
  }

  void collectWalls() {
    std::vector<std::size_t> index(scene_.roles.size(), NO_INDEX);
    for (std::size_t cell = 0; cell < scene_.wallOf.size(); ++cell) {
      const auto wall = wallAt(cell);
      if (!dynamic(wall)) {
        continue;
      }
      if (index[wall] == NO_INDEX) {
        index[wall] = walls_.size();
        walls_.push_back({.wall = wall});
      }
      walls_[index[wall]].cells.push_back(cell);
    }
    indexOf_ = std::move(index);
  }

  /// The outlines of a wall, one per piece: Moore tracing from the first
  /// cell of every piece, which has no cell of the wall to its left.
  void traceOutlines(DynamicWall& wall) {
    const auto own = [&](const std::int64_t x, const std::int64_t y) {
      return inside(x, y) && wallAt(cellAt(x, y)) == wall.wall;
    };
    for (const auto start : wall.cells) {
      const auto sx = xOf(start);
      const auto sy = yOf(start);
      if (traced_.test(start) || own(sx - 1, sy)) {
        continue;
      }
      // Moore tracing: scan the neighbours clockwise from the free cell
      // last looked at; the first cell of the wall is the next step, and the
      // free cell before it is where the next scan starts. The trace ends
      // where the step out of the start would repeat its first step.
      std::vector<std::size_t> loop;
      auto x = sx;
      auto y = sy;
      std::size_t from = START_FROM;
      std::optional<std::size_t> second;
      const auto limit = (8 * wall.cells.size()) + 16;
      for (std::size_t steps = 0; steps < limit; ++steps) {
        const auto here = cellAt(x, y);
        std::optional<std::size_t> turn;
        for (std::size_t tried = 1; tried <= 8 && !turn; ++tried) {
          const auto next = (from + tried) % 8;
          if (own(x + TURN_DX[next], y + TURN_DY[next])) {
            turn = next;
          }
        }
        if (!turn) {
          loop.push_back(here);
          traced_.set(here);
          break;
        }
        const auto nx = x + TURN_DX[*turn];
        const auto ny = y + TURN_DY[*turn];
        if (second && here == start && cellAt(nx, ny) == *second) {
          break;
        }
        loop.push_back(here);
        traced_.set(here);
        // The free cell before the step, seen from the new place.
        const auto before = (*turn + 7) % 8;
        const auto bx = x + TURN_DX[before];
        const auto by = y + TURN_DY[before];
        x = nx;
        y = ny;
        from = 0;
        for (std::size_t k = 0; k < 8; ++k) {
          if (x + TURN_DX[k] == bx && y + TURN_DY[k] == by) {
            from = k;
          }
        }
        if (!second) {
          second = cellAt(x, y);
        }
      }
      // The outline is the cells beside a free cell.
      std::vector<std::size_t> outline;
      for (const auto cell : loop) {
        bool open = false;
        for (std::size_t k = 0; k < 8 && !open; k += 2) {
          const auto nx = xOf(cell) + TURN_DX[k];
          const auto ny = yOf(cell) + TURN_DY[k];
          open = inside(nx, ny) && wallAt(cellAt(nx, ny)) == NO_WALL;
        }
        if (open && (outline.empty() || outline.back() != cell)) {
          outline.push_back(cell);
        }
      }
      if (outline.size() >= 3) {
        wall.outlines.push_back(std::move(outline));
      }
    }
  }

  /// The line of every dynamic wall: the given middle line of an inflated
  /// wall, else its longest outline, closed.
  void drawLines() {
    for (const auto& line : scene_.lines) {
      if (line.wall >= indexOf_.size() || indexOf_[line.wall] == NO_INDEX ||
          line.points.size() < 2) {
        continue;
      }
      auto& wall = walls_[indexOf_[line.wall]];
      wall.radius = line.radius;
      for (std::size_t k = 0; k < line.points.size(); ++k) {
        const auto [x, y] = line.points[k];
        const auto [px, py] = k == 0 ? line.points[k] : line.points[k - 1];
        for (const auto cell :
             lineCells(std::lround(px), std::lround(py), std::lround(x),
                       std::lround(y), scene_.width, scene_.height)) {
          const Point point{static_cast<double>(xOf(cell)),
                            static_cast<double>(yOf(cell))};
          if (wall.dense.empty() || wall.dense.back() != point) {
            wall.dense.push_back(point);
          }
        }
      }
    }
    for (auto& wall : walls_) {
      if (wall.dense.empty()) {
        const auto longest = std::ranges::max_element(
            wall.outlines, {},
            [](const auto& outline) { return outline.size(); });
        if (longest == wall.outlines.end()) {
          continue;
        }
        for (const auto cell : *longest) {
          wall.dense.emplace_back(static_cast<double>(xOf(cell)),
                                  static_cast<double>(yOf(cell)));
        }
        wall.closed = true;
      }
      wall.simple = simplify(wall.dense);
      if (wall.closed && !wall.dense.empty()) {
        wall.simple.push_back(wall.dense.front());
      }
      for (const auto& [x, y] : wall.dense) {
        wall.minX = std::min(wall.minX, x - wall.radius);
        wall.minY = std::min(wall.minY, y - wall.radius);
        wall.maxX = std::max(wall.maxX, x + wall.radius);
        wall.maxY = std::max(wall.maxY, y + wall.radius);
      }
    }
  }

  /// The cut from a cell of one wall towards a cell of another: along the
  /// line between them, the last cell of the first wall and the first cell
  /// of the second, with only free cells between, none of them a slot.
  [[nodiscard]] std::optional<Cut>
  cutBetween(const std::int64_t fromX, const std::int64_t fromY,
             const std::uint32_t one, const std::int64_t toX,
             const std::int64_t toY, const std::uint32_t other) const {
    const auto cells =
        lineCells(fromX, fromY, toX, toY, scene_.width, scene_.height);
    std::size_t at = 0;
    if (cells.empty() || wallAt(cells.front()) != one) {
      return std::nullopt;
    }
    while (at + 1 < cells.size() && wallAt(cells[at + 1]) == one) {
      ++at;
    }
    const auto first = cells[at];
    for (++at; at < cells.size(); ++at) {
      const auto wall = wallAt(cells[at]);
      if (wall == other) {
        const auto length = distance(first, cells[at]);
        if (length < options_.narrowest || length > options_.widest) {
          return std::nullopt;
        }
        return Cut{.first = first, .second = cells[at], .length = length};
      }
      if (wall != NO_WALL ||
          (scene_.slots != nullptr && scene_.slots->test(cells[at]))) {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  /// The gap from every cell of an outline to the nearest static wall of a
  /// field.
  void againstField(const DynamicWall& wall,
                    const std::vector<std::size_t>& outline,
                    const NearestBlocked& field) {
    Profile profile{.gap = std::vector<double>(outline.size(), INFINITE),
                    .partner =
                        std::vector<std::uint32_t>(outline.size(), NO_WALL),
                    .closed = true};
    std::vector<std::size_t> site(outline.size(), NO_NEAREST);
    const auto widest = options_.widest * options_.widest;
    const auto narrowest = options_.narrowest * options_.narrowest;
    for (std::size_t at = 0; at < outline.size(); ++at) {
      const auto cell = outline[at];
      const auto nearest = field.cell[cell];
      const auto squared = static_cast<double>(field.squared[cell]);
      if (nearest == NO_NEAREST || squared > widest || squared < narrowest) {
        continue;
      }
      const auto partner = wallAt(nearest);
      if (partner == NO_WALL || dynamic(partner)) {
        continue;
      }
      profile.gap[at] = std::sqrt(squared);
      profile.partner[at] = partner;
      site[at] = nearest;
    }
    for (const auto at : narrowings(profile, options_.minimumRise)) {
      const auto cell = outline[at];
      if (const auto cut =
              cutBetween(xOf(cell), yOf(cell), wall.wall, xOf(site[at]),
                         yOf(site[at]), profile.partner[at])) {
        cuts_.push_back(*cut);
      }
    }
  }

  /// The gap from every point of one dynamic wall's line to the line of
  /// another: their distance less both radii.
  void againstWall(const DynamicWall& one, const DynamicWall& other) {
    const auto reach = options_.widest;
    if (one.dense.empty() || other.simple.size() < 2 ||
        other.minX > one.maxX + reach || other.maxX < one.minX - reach ||
        other.minY > one.maxY + reach || other.maxY < one.minY - reach) {
      return;
    }
    const auto n = one.dense.size();
    Profile profile{.gap = std::vector<double>(n, INFINITE),
                    .partner = std::vector<std::uint32_t>(n, NO_WALL),
                    .closed = one.closed};
    std::vector<Point> foot(n);
    const auto radii = one.radius + other.radius;
    bool any = false;
    for (std::size_t at = 0; at < n; ++at) {
      const auto [px, py] = one.dense[at];
      if (px < other.minX - reach - one.radius ||
          px > other.maxX + reach + one.radius ||
          py < other.minY - reach - one.radius ||
          py > other.maxY + reach + one.radius) {
        continue;
      }
      double best = INFINITE;
      Point near{0.0, 0.0};
      for (std::size_t k = 0; k + 1 < other.simple.size(); ++k) {
        const auto [ax, ay] = other.simple[k];
        const auto [bx, by] = other.simple[k + 1];
        const auto dx = bx - ax;
        const auto dy = by - ay;
        const auto squared = (dx * dx) + (dy * dy);
        const auto t =
            squared > 0.0
                ? std::clamp((((px - ax) * dx) + ((py - ay) * dy)) / squared,
                             0.0, 1.0)
                : 0.0;
        const auto qx = ax + (t * dx);
        const auto qy = ay + (t * dy);
        const auto d = std::hypot(px - qx, py - qy);
        if (d < best) {
          best = d;
          near = {qx, qy};
        }
      }
      const auto gap = best - radii;
      if (gap >= options_.narrowest && gap <= options_.widest) {
        profile.gap[at] = gap;
        profile.partner[at] = other.wall;
        foot[at] = near;
        any = true;
      }
    }
    if (!any) {
      return;
    }
    for (const auto at : narrowings(profile, options_.minimumRise)) {
      const auto [px, py] = one.dense[at];
      const auto [qx, qy] = foot[at];
      if (const auto cut =
              cutBetween(std::lround(px), std::lround(py), one.wall,
                         std::lround(qx), std::lround(qy), other.wall)) {
        cuts_.push_back(*cut);
      }
    }
  }

  /// A cut from the open end of every flank to the nearest wall ahead:
  /// the nearest dynamic outline cell, unless a static wall lies nearer,
  /// and for a dynamic flank the nearest static wall as well.
  void closeMouths() {
    const auto across = (width_ + BUCKET - 1) / BUCKET;
    const auto down = (height_ + BUCKET - 1) / BUCKET;
    buckets_.assign(static_cast<std::size_t>(across * down), {});
    for (const auto& wall : walls_) {
      for (const auto& outline : wall.outlines) {
        for (const auto cell : outline) {
          buckets_[static_cast<std::size_t>(((yOf(cell) / BUCKET) * across) +
                                            (xOf(cell) / BUCKET))]
              .push_back(cell);
        }
      }
    }
    for (const auto& mouth : scene_.mouths) {
      const auto tx = xOf(mouth.tip);
      const auto ty = yOf(mouth.tip);
      const auto run =
          mouth.flank < scene_.runs.size() ? scene_.runs[mouth.flank] : NO_WALL;
      const auto ownRun = [&](const std::uint32_t wall) {
        return wall == mouth.flank ||
               (run != NO_WALL && wall < scene_.runs.size() &&
                scene_.runs[wall] == run);
      };
      const auto ahead = [&](const std::size_t cell) {
        return (static_cast<double>(xOf(cell) - tx) * mouth.towardX) +
                   (static_cast<double>(yOf(cell) - ty) * mouth.towardY) >
               0.0;
      };
      std::optional<Cut> best;
      const auto consider = [&](const std::size_t cell) {
        const auto wall = wallAt(cell);
        if (wall == NO_WALL || ownRun(wall) || !ahead(cell)) {
          return;
        }
        const auto length = distance(mouth.tip, cell);
        if (length > options_.widest || (best && length >= best->length)) {
          return;
        }
        if (!dynamic(wall) && !dynamic(mouth.flank)) {
          return;
        }
        if (dynamic(wall) && !dynamic(mouth.flank) &&
            length > mouth.staticClearance) {
          return;
        }
        if (const auto cut =
                cutBetween(tx, ty, mouth.flank, xOf(cell), yOf(cell), wall)) {
          best = cut;
        }
      };
      // The nearest outline cell ahead of every dynamic wall within reach:
      // the widest gap, and for a static flank no farther than the static
      // wall ahead of it.
      std::vector<std::pair<std::uint32_t, std::pair<double, std::size_t>>>
          nearest;
      const auto radius =
          dynamic(mouth.flank)
              ? options_.widest
              : std::min(options_.widest, mouth.staticClearance);
      const auto reach = static_cast<std::int64_t>(
          std::ceil(radius / static_cast<double>(BUCKET)));
      for (auto by = std::max<std::int64_t>(0, (ty / BUCKET) - reach);
           by <= std::min(down - 1, (ty / BUCKET) + reach); ++by) {
        for (auto bx = std::max<std::int64_t>(0, (tx / BUCKET) - reach);
             bx <= std::min(across - 1, (tx / BUCKET) + reach); ++bx) {
          for (const auto cell :
               buckets_[static_cast<std::size_t>((by * across) + bx)]) {
            if (!ahead(cell)) {
              continue;
            }
            const auto d = distance(mouth.tip, cell);
            if (d > radius) {
              continue;
            }
            const auto wall = wallAt(cell);
            auto found = std::ranges::find(
                nearest, wall, [](const auto& entry) { return entry.first; });
            if (found == nearest.end()) {
              nearest.push_back({wall, {d, cell}});
            } else if (d < found->second.first) {
              found->second = {d, cell};
            }
          }
        }
      }
      for (const auto& [wall, best] : nearest) {
        consider(best.second);
      }
      if (dynamic(mouth.flank)) {
        for (const auto* field : scene_.fields) {
          if (field->cell[mouth.tip] != NO_NEAREST) {
            consider(field->cell[mouth.tip]);
          }
        }
      }
      if (best) {
        cuts_.push_back(*best);
      }
    }
  }

  /// Where two walls touch, the gap between them only widens away from the
  /// contact, and a minimum in the wedge is no bottleneck; so too where
  /// both touch a third dynamic wall, as two edges of a chain do at their
  /// coupler.
  void dropWedges() {
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::size_t>>
        contacts;
    for (const auto& wall : walls_) {
      for (const auto& outline : wall.outlines) {
        for (const auto cell : outline) {
          for (std::size_t k = 0; k < 8; ++k) {
            const auto nx = xOf(cell) + TURN_DX[k];
            const auto ny = yOf(cell) + TURN_DY[k];
            if (!inside(nx, ny)) {
              continue;
            }
            const auto other = wallAt(cellAt(nx, ny));
            if (other == NO_WALL || other == wall.wall) {
              continue;
            }
            auto& at = contacts[std::minmax(other, wall.wall)];
            if (at.empty() || at.back() != cell) {
              at.push_back(cell);
            }
          }
        }
      }
    }
    std::map<std::uint32_t, std::vector<std::uint32_t>> touching;
    for (const auto& [pair, cells] : contacts) {
      touching[pair.first].push_back(pair.second);
      touching[pair.second].push_back(pair.first);
    }
    const auto touchNear = [&](const std::uint32_t a, const std::uint32_t b,
                               const Cut& cut) {
      const auto found = contacts.find(std::minmax(a, b));
      if (found == contacts.end()) {
        return false;
      }
      const auto reach = (3.0 * cut.length) + 5.0;
      return std::ranges::any_of(found->second, [&](const std::size_t cell) {
        return distance(cell, cut.first) <= reach;
      });
    };
    std::erase_if(cuts_, [&](const Cut& cut) {
      const auto a = wallAt(cut.first);
      const auto b = wallAt(cut.second);
      if (touchNear(a, b, cut)) {
        return true;
      }
      for (const auto third : touching[a]) {
        if (third != b && dynamic(third) && touchNear(a, third, cut) &&
            touchNear(b, third, cut)) {
          return true;
        }
      }
      return false;
    });
  }

  /// Cuts between the same two walls whose ends lie within `sameNarrowing`
  /// of each other on both walls are one narrowing: the shortest stays.
  std::vector<Bottleneck> oneCutPerNarrowing() {
    std::ranges::sort(cuts_, {}, &Cut::length);
    std::vector<Cut> kept;
    const auto close = [&](const std::size_t a, const std::size_t b) {
      return distance(a, b) <= options_.sameNarrowing;
    };
    for (const auto& cut : cuts_) {
      const auto a = wallAt(cut.first);
      const auto b = wallAt(cut.second);
      const bool repeated = std::ranges::any_of(kept, [&](const Cut& other) {
        const auto c = wallAt(other.first);
        const auto d = wallAt(other.second);
        return (a == c && b == d && close(cut.first, other.first) &&
                close(cut.second, other.second)) ||
               (a == d && b == c && close(cut.first, other.second) &&
                close(cut.second, other.first));
      });
      if (!repeated) {
        kept.push_back(cut);
      }
    }
    std::vector<Bottleneck> result;
    result.reserve(kept.size());
    for (const auto& cut : kept) {
      const auto mx = (xOf(cut.first) + xOf(cut.second)) / 2;
      const auto my = (yOf(cut.first) + yOf(cut.second)) / 2;
      result.push_back(
          {.first = cut.first, .second = cut.second, .saddle = cellAt(mx, my)});
    }
    return result;
  }

  static constexpr std::size_t NO_INDEX = static_cast<std::size_t>(-1);

  const ProfileScene& scene_;
  const ProfileOptions& options_;
  std::int64_t width_;
  std::int64_t height_;
  std::vector<DynamicWall> walls_;
  std::vector<std::size_t> indexOf_;
  /// The cells a trace has passed.
  BitGrid traced_;
  /// The outline cells of the dynamic walls in square buckets, for the
  /// mouths.
  static constexpr std::int64_t BUCKET = 32;
  std::vector<std::vector<std::size_t>> buckets_;
  std::vector<Cut> cuts_;
};

} // namespace

std::vector<Bottleneck> profileCuts(const ProfileScene& scene,
                                    const ProfileOptions& options) {
  const auto cells = static_cast<std::size_t>(scene.width) * scene.height;
  if (scene.wallOf.size() != cells) {
    throw std::invalid_argument(
        std::format("the wall labels have {} cells, the grid {}x{}",
                    scene.wallOf.size(), scene.width, scene.height));
  }
  for (const auto* field : scene.fields) {
    if (field == nullptr || field->cell.size() != cells ||
        field->squared.size() != cells) {
      throw std::invalid_argument("a static field does not cover the grid");
    }
  }
  return Search(scene, options).run();
}

} // namespace mqt::scpd::grid
