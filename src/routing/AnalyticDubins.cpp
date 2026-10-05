/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/AnalyticDubins.hpp"

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// How many masks of the eight headings there are, and how many directions a
/// displacement can be classified into: one per heading and one per sector
/// between two neighbouring headings.
constexpr std::uint32_t MASKS = 256;
constexpr std::uint32_t CLASSES = 2 * NUM_HEADINGS;

struct Vector {
  std::int64_t x = 0;
  std::int64_t y = 0;
};

constexpr Vector vectorOf(const Heading heading) {
  const HeadingVector v = headingVector(heading);
  return {.x = v.dx, .y = v.dy};
}

/// Whether `r` is a non-negative combination of `u` and `v`, strictly
/// positive in both when `strict`. Cramer's rule, kept in integers.
constexpr bool spannedBy(const Vector u, const Vector v, const Vector r,
                         const bool strict) {
  const std::int64_t det = (u.x * v.y) - (u.y * v.x);
  if (det == 0) {
    return false;
  }
  // r = a * u + b * v, with a = numA / det and b = numB / det.
  const std::int64_t numA = (r.x * v.y) - (r.y * v.x);
  const std::int64_t numB = (u.x * r.y) - (u.y * r.x);
  if (det > 0) {
    return strict ? (numA > 0 && numB > 0) : (numA >= 0 && numB >= 0);
  }
  return strict ? (numA < 0 && numB < 0) : (numA <= 0 && numB <= 0);
}

/// Whether `r` runs along `u` and points the same way.
constexpr bool alongside(const Vector u, const Vector r) {
  return ((u.x * r.y) - (u.y * r.x)) == 0 && ((u.x * r.x) + (u.y * r.y)) > 0;
}

/// Which of the sixteen directions a displacement points in: an even class
/// for a displacement along a heading, the odd class after it for one
/// strictly between that heading and the next. The cone of any set of
/// headings has its edges on headings, so every displacement of one class
/// lies in it or none does — which is what lets the answer be tabulated.
std::uint32_t classOf(const Vector r) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    if (alongside(vectorOf(h), r)) {
      return 2U * h;
    }
  }
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    if (spannedBy(vectorOf(h), vectorOf(turned(h, 1)), r, true)) {
      return (2U * h) + 1U;
    }
  }
  return 0;
}

/// A representative displacement of a direction class.
constexpr Vector representative(const std::uint32_t direction) {
  const auto h = static_cast<Heading>(direction / 2U);
  const Vector u = vectorOf(h);
  if (direction % 2U == 0U) {
    return u;
  }
  // The two neighbours sum to a displacement strictly between them.
  const Vector v = vectorOf(turned(h, 1));
  return {.x = u.x + v.x, .y = u.y + v.y};
}

/// For every set of headings, which direction classes its cone holds.
///
/// A way may run straight along any heading it passes through, for as long as
/// it likes, so what its straight runs can add up to is exactly the cone of
/// those headings. In two dimensions a member of a cone is already a
/// non-negative combination of two of its generators, so trying every pair
/// and every single generator settles it exactly.
const std::array<std::uint32_t, MASKS>& coneTable() {
  static const std::array<std::uint32_t, MASKS> table = [] {
    std::array<std::uint32_t, MASKS> built{};
    for (std::uint32_t mask = 0; mask < MASKS; ++mask) {
      std::uint32_t holds = 0;
      for (std::uint32_t direction = 0; direction < CLASSES; ++direction) {
        const Vector r = representative(direction);
        bool in = false;
        for (Heading a = 0; a < NUM_HEADINGS && !in; ++a) {
          if ((mask & (1U << a)) == 0U) {
            continue;
          }
          if (alongside(vectorOf(a), r)) {
            in = true;
            break;
          }
          for (Heading b = 0; b < NUM_HEADINGS && !in; ++b) {
            if (b == a || (mask & (1U << b)) == 0U) {
              continue;
            }
            in = spannedBy(vectorOf(a), vectorOf(b), r, false);
          }
        }
        if (in) {
          holds |= 1U << direction;
        }
      }
      built[mask] = holds;
    }
    return built;
  }();
  return table;
}

} // namespace

AnalyticDubins::AnalyticDubins(const MovePrimitives& primitives,
                               const std::uint32_t cap)
    : cap_(cap) {
  for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
    for (const Primitive& p : primitives.of(heading)) {
      const auto turn = headingDistance(heading, p.exitHeading);
      if (turn == 0) {
        // The straight step does not turn, so it is free and unbounded here;
        // the cone of the headings stands for every straight run at once.
        continue;
      }
      arcs_[heading].push_back({.exit = p.exitHeading,
                                .dx = p.dx,
                                .dy = p.dy,
                                .turn = turn,
                                .swept = p.swept});
    }
  }
  (void)coneTable();
}

bool AnalyticDubins::reaches(const std::int64_t dx, const std::int64_t dy,
                             const std::uint32_t headings) {
  if (dx == 0 && dy == 0) {
    return true;
  }
  const auto direction = classOf({.x = dx, .y = dy});
  return (coneTable()[headings & (MASKS - 1U)] & (1U << direction)) != 0U;
}

bool AnalyticDubins::within(const Heading heading, const std::int64_t dx,
                            const std::int64_t dy, const Heading target,
                            const std::uint32_t budget,
                            const std::uint32_t headings) const {
  if (heading == target && reaches(dx, dy, headings)) {
    return true;
  }
  if (budget == 0) {
    return false;
  }
  for (const Arc& arc : arcs_[heading]) {
    if (arc.turn > budget) {
      continue;
    }
    const auto left = budget - arc.turn;
    // Whatever else it does, it still owes the turn onto the target heading.
    if (headingDistance(arc.exit, target) > left) {
      continue;
    }
    if (within(arc.exit, dx - arc.dx, dy - arc.dy, target, left,
               headings | (1U << arc.exit))) {
      return true;
    }
  }
  return false;
}

namespace {

/// The enumeration of one family: the ways of one arc sequence, as
/// `AnalyticDubins::minTurnsAround` describes them.
struct Family {
  const AnalyticDubins::Blocked& blocked;
  const AnalyticDubins::Box& box;
  std::uint64_t budget;
  std::uint64_t paths = 0;
  bool budgetOut = false;
  /// A family of this level had too many runs to walk: the level cannot be
  /// proved blocked, whatever the other families say.
  bool unsettled = false;

  [[nodiscard]] bool inBox(const std::int64_t x, const std::int64_t y) const {
    return x >= box.minX && x <= box.maxX && y >= box.minY && y <= box.maxY;
  }
  [[nodiscard]] bool closed(const std::int64_t x, const std::int64_t y) const {
    return !inBox(x, y) || blocked(x, y);
  }

  /// Whether one concrete way — the runs `lambda` between the arcs of
  /// `seq`, from `start` — sweeps only open cells.
  template <typename ArcT>
  [[nodiscard]] bool open(const std::int64_t startX, const std::int64_t startY,
                          const Heading source,
                          const std::vector<const ArcT*>& seq,
                          const std::vector<std::int64_t>& lambda) {
    ++paths;
    std::int64_t x = startX;
    std::int64_t y = startY;
    Heading heading = source;
    for (std::size_t i = 0; i <= seq.size(); ++i) {
      const auto v = headingVector(heading);
      for (std::int64_t t = 0; t < lambda[i]; ++t) {
        x += v.dx;
        y += v.dy;
        if (closed(x, y)) {
          return false;
        }
      }
      if (i == seq.size()) {
        break;
      }
      const ArcT& arc = *seq[i];
      for (const auto& cell : arc.swept) {
        if (closed(x + cell.dx, y + cell.dy)) {
          return false;
        }
      }
      x += arc.dx;
      y += arc.dy;
      heading = arc.exit;
    }
    return true;
  }
};

constexpr std::int64_t det(const std::int64_t ux, const std::int64_t uy,
                           const std::int64_t wx, const std::int64_t wy) {
  return (ux * wy) - (uy * wx);
}

} // namespace

std::uint32_t AnalyticDubins::minTurnsAround(
    const PathPoint& from, const PathPoint& to, const Blocked& blocked,
    const Box& box, const std::uint32_t minFirstRun,
    const std::uint32_t minLastRun, const std::uint64_t pathBudget,
    AroundStats* stats, const std::uint32_t maxRaise) const {
  const auto dx =
      static_cast<std::int64_t>(to.x) - static_cast<std::int64_t>(from.x);
  const auto dy =
      static_cast<std::int64_t>(to.y) - static_cast<std::int64_t>(from.y);
  const auto source = static_cast<Heading>(from.heading & 7U);
  const auto target = static_cast<Heading>(to.heading & 7U);
  const auto floor = minTurns(from, to);
  if (floor > cap_) {
    return floor;
  }
  // No run can be longer than the box in either direction.
  const auto longest = std::max<std::int64_t>(
      0, std::max(box.maxX - box.minX, box.maxY - box.minY));
  Family family{.blocked = blocked, .box = box, .budget = pathBudget};

  // Every sequence of arcs turning exactly `k` in all from the source
  // heading to the target heading.
  std::vector<const Arc*> seq;
  std::vector<std::int64_t> lambda;
  bool found = false;
  const auto tryFamily = [&](const std::int64_t rx, const std::int64_t ry) {
    // Headings of the runs: the source, then each arc's exit.
    const auto runs = seq.size() + 1;
    std::vector<HeadingVector> v(runs);
    std::vector<std::int64_t> lo(runs, 0);
    v[0] = headingVector(source);
    for (std::size_t i = 0; i < seq.size(); ++i) {
      v[i + 1] = headingVector(seq[i]->exit);
    }
    lo[0] = std::max<std::int64_t>(lo[0], minFirstRun);
    lo[runs - 1] = std::max<std::int64_t>(lo[runs - 1], minLastRun);
    // Two runs on headings that are not parallel are solved for; the rest
    // are enumerated. Prefer the last two.
    std::size_t p = runs;
    std::size_t q = runs;
    for (std::size_t b = runs; b-- > 0 && p == runs;) {
      for (std::size_t a = b; a-- > 0;) {
        if (det(v[a].dx, v[a].dy, v[b].dx, v[b].dy) != 0) {
          p = a;
          q = b;
          break;
        }
      }
    }
    std::vector<std::size_t> free;
    for (std::size_t i = 0; i < runs; ++i) {
      if (i != p && i != q) {
        free.push_back(i);
      }
    }
    if (free.size() > 2) {
      // Three arcs or more: too many runs to walk. The level stays
      // unsettled; another family of it may still find an open way.
      family.unsettled = true;
      return false;
    }
    lambda.assign(runs, 0);
    const auto startX = static_cast<std::int64_t>(from.x);
    const auto startY = static_cast<std::int64_t>(from.y);
    // Recursive enumeration of the free runs, the solved pair last.
    const std::function<bool(std::size_t, std::int64_t, std::int64_t)> walk =
        [&](const std::size_t f, const std::int64_t leftX,
            const std::int64_t leftY) -> bool {
      if (family.paths >= family.budget) {
        family.budgetOut = true;
        return false;
      }
      if (f == free.size()) {
        if (p == runs) {
          // Every heading parallel: the last free run was the last run, and
          // nothing is left to solve — the displacement has to be covered.
          return leftX == 0 && leftY == 0 &&
                 family.open(startX, startY, source, seq, lambda);
        }
        const auto d = det(v[p].dx, v[p].dy, v[q].dx, v[q].dy);
        const auto na = det(leftX, leftY, v[q].dx, v[q].dy);
        const auto nb = det(v[p].dx, v[p].dy, leftX, leftY);
        if (na % d != 0 || nb % d != 0) {
          return false;
        }
        const auto a = na / d;
        const auto b = nb / d;
        if (a < lo[p] || b < lo[q] || a > longest || b > longest) {
          return false;
        }
        lambda[p] = a;
        lambda[q] = b;
        return family.open(startX, startY, source, seq, lambda);
      }
      const auto i = free[f];
      for (std::int64_t n = lo[i]; n <= longest; ++n) {
        lambda[i] = n;
        if (walk(f + 1, leftX - (n * v[i].dx), leftY - (n * v[i].dy))) {
          return true;
        }
        if (family.budgetOut) {
          return false;
        }
      }
      return false;
    };
    if (p == runs) {
      // All runs parallel: enumerate all but the last, which is solved.
      free.clear();
      for (std::size_t i = 0; i + 1 < runs; ++i) {
        free.push_back(i);
      }
      const std::function<bool(std::size_t, std::int64_t, std::int64_t)>
          walkParallel = [&](const std::size_t f, const std::int64_t leftX,
                             const std::int64_t leftY) -> bool {
        if (family.paths >= family.budget) {
          family.budgetOut = true;
          return false;
        }
        if (f == free.size()) {
          const auto& u = v[runs - 1];
          const auto n = (u.dx != 0) ? leftX / u.dx : leftY / u.dy;
          if (n < lo[runs - 1] || n > longest || (n * u.dx) != leftX ||
              (n * u.dy) != leftY) {
            return false;
          }
          lambda[runs - 1] = n;
          return family.open(startX, startY, source, seq, lambda);
        }
        const auto i = free[f];
        for (std::int64_t n = lo[i]; n <= longest; ++n) {
          lambda[i] = n;
          if (walkParallel(f + 1, leftX - (n * v[i].dx),
                           leftY - (n * v[i].dy))) {
            return true;
          }
          if (family.budgetOut) {
            return false;
          }
        }
        return false;
      };
      return walkParallel(0, rx, ry);
    }
    return walk(0, rx, ry);
  };

  // Depth-first over the arc sequences of exactly `k` turns.
  const std::function<void(Heading, std::int64_t, std::int64_t, std::uint32_t)>
      sequences = [&](const Heading heading, const std::int64_t rx,
                      const std::int64_t ry, const std::uint32_t left) {
        if (found || family.budgetOut) {
          return;
        }
        if (left == 0) {
          if (heading == target && tryFamily(rx, ry)) {
            found = true;
          }
          return;
        }
        for (const Arc& arc : arcs_[heading]) {
          if (arc.turn > left || headingDistance(arc.exit, target) > left - arc.turn) {
            continue;
          }
          seq.push_back(&arc);
          sequences(arc.exit, rx - arc.dx, ry - arc.dy, left - arc.turn);
          seq.pop_back();
          if (found || family.budgetOut) {
            return;
          }
        }
      };

  const auto most = std::min<std::uint32_t>(cap_, floor + maxRaise);
  auto turns = floor;
  bool proved = true;
  for (; turns <= most; ++turns) {
    found = false;
    family.unsettled = false;
    seq.clear();
    sequences(source, dx, dy, turns);
    if (found) {
      break;
    }
    if (family.budgetOut || family.unsettled) {
      // Not every way of this level was tried: nothing above it is proved.
      proved = false;
      break;
    }
  }
  // Past the families looked at, every one of them was blocked: the truth
  // is above them, and the answer says so and no more.
  const auto answer = std::min<std::uint32_t>(turns, cap_ + 1);
  if (stats != nullptr) {
    stats->paths += family.paths;
    stats->raised += answer - floor;
    stats->budgetOut += proved ? 0 : 1;
  }
  return answer;
}

std::uint32_t AnalyticDubins::minTurns(const PathPoint& from,
                                       const PathPoint& to) const {
  const auto dx =
      static_cast<std::int64_t>(to.x) - static_cast<std::int64_t>(from.x);
  const auto dy =
      static_cast<std::int64_t>(to.y) - static_cast<std::int64_t>(from.y);
  const auto source = static_cast<Heading>(from.heading & 7U);
  const auto target = static_cast<Heading>(to.heading & 7U);

  // A way walks the ring of headings from one to the other, so it can never
  // turn less than that; there is nothing to look for below it.
  for (auto turns = headingDistance(source, target); turns <= cap_; ++turns) {
    if (within(source, dx, dy, target, turns, 1U << source)) {
      return turns;
    }
  }
  // Nothing was feasible within the cap, so the truth is above it.
  return cap_ + 1;
}

} // namespace mqt::scpd::routing
