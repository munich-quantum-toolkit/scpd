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

#include <array>
#include <cstdint>

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
      arcs_[heading].push_back(
          {.exit = p.exitHeading, .dx = p.dx, .dy = p.dy, .turn = turn});
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
