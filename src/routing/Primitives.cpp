/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/Primitives.hpp"

#include "mqt-scpd/routing/Heading.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <numbers>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

// The generation below builds the moves that leave heading 0, the canonical
// cardinal heading, and the moves that leave heading 7, the canonical
// diagonal heading, and rotates and mirrors them onto the others. Every
// rounding step is part of the definition of the tables: the swept cells and
// the costs depend on it. Where a value is a whole number in exact
// arithmetic, as the end of a move is, the generation computes it from whole
// numbers. A truncated floating-point value could fall one below it, and
// whether it does depends on the compiler and the math library.

namespace mqt::scpd::routing {

namespace {

constexpr double PI = std::numbers::pi;
constexpr double DEGREES = 180.0 / PI;

using IntCells = std::vector<std::array<int32_t, 2>>;
using Samples = std::vector<std::array<double, 2>>;
using Vector16 = std::array<int16_t, 2>;

/// The maps of one heading while the tables are being built.
struct HeadingTables {
  std::map<uint32_t, uint16_t> exit;
  std::map<uint32_t, Vector16> vector;
  std::map<uint32_t, IntCells> swept;
  std::map<uint32_t, double> cost;
  std::map<uint32_t, Samples> samples;
};

IntCells invert(const IntCells& cells, const bool dimX, const bool dimY,
                const bool permutate) {
  IntCells out;
  out.reserve(cells.size());
  for (const auto& c : cells) {
    if (permutate) {
      out.push_back({dimY ? -c[1] : c[1], dimX ? -c[0] : c[0]});
    } else {
      out.push_back({dimX ? -c[0] : c[0], dimY ? -c[1] : c[1]});
    }
  }
  return out;
}

Samples invert(const Samples& points, const bool dimX, const bool dimY,
               const bool permutate) {
  Samples out;
  out.reserve(points.size());
  for (const auto& p : points) {
    if (permutate) {
      out.push_back({dimY ? -p[1] : p[1], dimX ? -p[0] : p[0]});
    } else {
      out.push_back({dimX ? -p[0] : p[0], dimY ? -p[1] : p[1]});
    }
  }
  return out;
}

Vector16 invert(const Vector16& v, const bool dimX, const bool dimY,
                const bool permutate) {
  if (!permutate) {
    return {static_cast<int16_t>(dimX ? -v[0] : v[0]),
            static_cast<int16_t>(dimY ? -v[1] : v[1])};
  }
  return {static_cast<int16_t>(dimY ? -v[1] : v[1]),
          static_cast<int16_t>(dimX ? -v[0] : v[0])};
}

IntCells rotate(const IntCells& cells, const double angleDegrees) {
  IntCells out;
  out.reserve(cells.size());
  const double radians = angleDegrees * (PI / 180.0);
  const double c = std::cos(-radians);
  const double s = std::sin(-radians);
  for (const auto& p : cells) {
    const double x =
        (static_cast<double>(p[0]) * c) - (static_cast<double>(p[1]) * s);
    const double y =
        (static_cast<double>(p[0]) * s) + (static_cast<double>(p[1]) * c);
    out.push_back({static_cast<int32_t>(std::lround(x)),
                   static_cast<int32_t>(std::lround(y))});
  }
  return out;
}

Samples rotate(const Samples& points, const double angleDegrees) {
  Samples out;
  out.reserve(points.size());
  const double radians = angleDegrees * (PI / 180.0);
  const double c = std::cos(-radians);
  const double s = std::sin(-radians);
  for (const auto& p : points) {
    out.push_back({(p[0] * c) - (p[1] * s), (p[0] * s) + (p[1] * c)});
  }
  return out;
}

Vector16 rotate(const Vector16& v, const double angleDegrees) {
  const double radians = angleDegrees * (PI / 180.0);
  const double c = std::cos(-radians);
  const double s = std::sin(-radians);
  const double x =
      (static_cast<double>(v[0]) * c) - (static_cast<double>(v[1]) * s);
  const double y =
      (static_cast<double>(v[0]) * s) + (static_cast<double>(v[1]) * c);
  return {static_cast<int16_t>(std::lround(x)),
          static_cast<int16_t>(std::lround(y))};
}

std::array<double, 2> rotate(const std::array<double, 2> p,
                             const double angleDegrees) {
  const double radians = angleDegrees * (PI / 180.0);
  const double c = std::cos(-radians);
  const double s = std::sin(-radians);
  return {(p[0] * c) - (p[1] * s), (p[0] * s) + (p[1] * c)};
}

/// Ends the samples of a curve exactly at its end. The spacing accumulates in
/// floating point, so the last sample can fall just short of the end or the
/// end can drop out. A last sample within half a spacing of the end moves
/// onto it; otherwise the end is added.
void endSamplesAt(Samples& samples, const std::array<double, 2> end) {
  const std::array<double, 2>& last = samples.back();
  if (std::hypot(last[0] - end[0], last[1] - end[1]) <
      MovePrimitives::SAMPLE_SPACING / 2.0) {
    samples.back() = end;
  } else {
    samples.push_back(end);
  }
}

/// The eight-way heading of a canonical angle in degrees: 0 is heading 0,
/// 45 is heading 1, and so on.
uint16_t headingOfDegrees(const double degrees) {
  return static_cast<uint16_t>(std::lround(degrees / 45.0) % 8);
}

// ---------------------------------------------------------------------------
// Cardinal headings.
// ---------------------------------------------------------------------------

/// The swept cells, the cost and the samples of one arc that leaves the
/// canonical cardinal heading. A straight lead along the heading comes
/// before the arc. The swept cells of the arc start from the lead's end
/// rounded to a whole cell toward the start, as in the research prototype.
/// The samples start the arc at the exact end of the lead.
IntCells arcCellsCardinal(const double radius, const Vector16 vector,
                          double& cost, Samples& samples) {
  IntCells cells;
  const double height = std::sqrt(
      (radius * radius) - ((vector[0] - radius) * (vector[0] - radius)));
  const double offset = vector[1] + height;
  cost = (radius * std::acos(1.0 - (vector[0] / radius))) - offset;

  for (int32_t y = 0; y >= static_cast<int32_t>(std::floor(offset)); --y) {
    cells.push_back({0, y});
  }
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double ys = 0.0; ys <= -offset; ys += MovePrimitives::SAMPLE_SPACING) {
    samples.push_back({0.0, -ys});
  }
  for (int32_t xp = 0; xp <= vector[0]; ++xp) {
    const double y0 = std::sqrt((radius * radius) -
                                ((radius - xp - 0.5) * (radius - xp - 0.5)));
    const double y1 =
        std::sqrt((radius * radius) - ((radius - xp) * (radius - xp)));
    for (auto yp = static_cast<int32_t>(std::floor(std::min(y0, y1)));
         yp <= static_cast<int32_t>(std::ceil(std::max(y0, y1))); ++yp) {
      cells.push_back({xp, static_cast<int32_t>(-yp + std::ceil(offset))});
    }
  }
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double xs = 0.0; xs <= vector[0]; xs += MovePrimitives::SAMPLE_SPACING) {
    samples.push_back(
        {xs, -std::sqrt((radius * radius) - ((radius - xs) * (radius - xs))) +
                 offset});
  }
  endSamplesAt(samples, {static_cast<double>(vector[0]),
                         static_cast<double>(vector[1])});
  cells.erase(std::ranges::unique(cells).begin(), cells.end());
  return cells;
}

void generateCardinal(const uint32_t radius,
                      std::array<HeadingTables, 8>& tables) {
  const auto r = static_cast<double>(radius);
  const auto signedRadius = static_cast<int>(radius);
  std::vector<std::array<double, 2>> ranges;
  std::map<uint32_t, Vector16> vectorOf;
  for (uint32_t i = 1; i <= radius; ++i) {
    const double y =
        std::sqrt((r * r) - (static_cast<double>(radius - i) * (radius - i)));
    const double yUp = std::ceil(y);
    const double yDown = std::floor(y);
    const double angleMin = std::atan((radius - i) / y) * DEGREES;
    for (int j = static_cast<int>(yUp); j <= signedRadius && j <= yDown + 2;
         ++j) {
      const double angleMax =
          std::atan(static_cast<double>(radius - i) / j) * DEGREES;
      ranges.push_back({angleMin, angleMax});
      vectorOf[static_cast<uint32_t>(ranges.size() - 1)] = {
          static_cast<int16_t>(i), static_cast<int16_t>(-j)};
      if ((angleMax <= 45.0 && angleMin >= 45.0) ||
          (angleMax <= 0.0 && angleMin >= 0.0)) {
        break;
      }
    }
  }

  HeadingTables canonical;
  std::set<uint32_t> keys;
  for (uint32_t i = 0; i < ranges.size(); ++i) {
    for (int angle = 0; angle <= 90; angle += 45) {
      if (angle <= ranges[i][0] && angle >= ranges[i][1]) {
        keys.insert(i);
        canonical.exit[i] = headingOfDegrees(90.0 - angle);
        canonical.vector[i] = vectorOf[i];
        double cost = 0.0;
        Samples samples;
        canonical.swept[i] = arcCellsCardinal(r, vectorOf[i], cost, samples);
        canonical.cost[i] = cost;
        canonical.samples[i] = samples;
      }
    }
  }

  const auto n = static_cast<uint32_t>(ranges.size());
  for (uint16_t heading = 0; heading < 8; heading += 2) {
    const double degrees = heading * 45.0;
    HeadingTables& t = tables[heading];
    for (const uint32_t idx : keys) {
      const uint16_t exit = canonical.exit[idx];
      const Heading turnedOne = turned(static_cast<Heading>(heading), exit);
      const Heading turnedOther = turned(static_cast<Heading>(heading), -exit);
      t.cost[idx] = canonical.cost[idx];
      t.cost[n + idx] = canonical.cost[idx];
      t.exit[idx] = turnedOther;
      t.exit[n + idx] = turnedOne;
      t.samples[idx] = rotate(canonical.samples[idx], degrees);
      t.samples[n + idx] =
          rotate(invert(canonical.samples[idx], true, false, false), degrees);
      t.vector[idx] = rotate(canonical.vector[idx], degrees);
      t.vector[n + idx] =
          rotate(invert(canonical.vector[idx], true, false, false), degrees);
      t.swept[idx] = rotate(canonical.swept[idx], degrees);
      t.swept[n + idx] =
          rotate(invert(canonical.swept[idx], true, false, false), degrees);
    }
    const uint32_t straightId = n * 2;
    Samples straightSamples;
    // The floating-point accumulation is part of the table definition.
    // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
    for (double ys = 0.0; ys <= 1.0; ys += MovePrimitives::SAMPLE_SPACING) {
      straightSamples.push_back(
          rotate(std::array<double, 2>{0.0, -ys}, degrees));
    }
    t.samples[straightId] = straightSamples;
    t.cost[straightId] = 1.0;
    t.exit[straightId] = heading;
    const Vector16 step = rotate(Vector16{0, -1}, degrees);
    t.vector[straightId] = step;
    t.swept[straightId] = {{step[0], step[1]}};
  }
}

// ---------------------------------------------------------------------------
// Diagonal headings.
// ---------------------------------------------------------------------------

uint32_t roundUpSqrt2(const double x) {
  if (x <= 0.0001) {
    return 0;
  }
  double v = 0.0;
  for (uint32_t i = 0;; ++i) {
    if (x <= v) {
      return i;
    }
    v += std::numbers::sqrt2;
  }
}

uint32_t roundDownSqrt2(const double x) {
  if (x <= 0.0001) {
    return 0;
  }
  double v = 0.0;
  for (uint32_t i = 0;; ++i) {
    if (x <= v) {
      return i == 0 ? 0 : i - 1;
    }
    v += std::numbers::sqrt2;
  }
}

uint32_t roundUpSqrt2Offset(const double x) {
  if (std::isnan(x) || x <= 0.0001) {
    return 0;
  }
  double value = std::numbers::sqrt2 / 2.0;
  for (uint32_t i = 0;; ++i) {
    if (x <= value) {
      return i;
    }
    value += std::numbers::sqrt2;
  }
}

// The canonical diagonal moves are built in the diagonal frame, whose axes run
// along the diagonals: its x axis toward heading 5, its y axis toward
// heading 7, the direction of travel. A move there ends i half diagonals
// across and j diagonals ahead, plus half a diagonal for an odd i.

/// Converts a point of the diagonal frame to grid cells.
std::array<double, 2> fromDiagonalFrame(const double x, const double y) {
  constexpr double half = std::numbers::sqrt2 / 2.0;
  return {(x + y) * half, (x - y) * half};
}

/// The end of a canonical diagonal move in grid cells. In exact arithmetic
/// the end is a whole cell, so whole numbers give it exactly.
Vector16 diagonalEnd(const uint32_t i, const uint32_t j) {
  const auto halfI = static_cast<int32_t>(i / 2);
  const auto odd = static_cast<int32_t>(i % 2);
  const auto ahead = static_cast<int32_t>(j);
  return {static_cast<int16_t>(ahead + halfI + odd),
          static_cast<int16_t>(halfI - ahead)};
}

IntCells arcCellsDiagonal(const double radius, const Vector16 vector,
                          const std::array<double, 2> ranges, double& cost,
                          Samples& samples) {
  IntCells result;
  const double ai = vector[0] * (std::numbers::sqrt2 / 2.0);
  const double height =
      std::sqrt((radius * radius) - ((ai - radius) * (ai - radius)));
  double ay = vector[1] * std::numbers::sqrt2;
  if (vector[0] % 2 == 1) {
    ay -= std::numbers::sqrt2 / 2.0;
  }
  const double offset = ay + height;
  cost = (radius * std::acos(1.0 - (vector[0] / radius))) - offset;

  // The straight lead in the diagonal frame runs whole diagonals ahead, which
  // are whole cells on the grid.
  int32_t lead = 0;
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double yp = 0.0; yp >= std::floor(offset); yp -= std::numbers::sqrt2) {
    result.push_back({lead, -lead});
    ++lead;
  }
  for (int32_t xp = 0; xp <= vector[0]; ++xp) {
    const double ax = xp * (std::numbers::sqrt2 / 2.0);
    const double y0 = std::sqrt((radius * radius) -
                                ((radius - ax - (std::numbers::sqrt2 / 2.0)) *
                                 (radius - ax - (std::numbers::sqrt2 / 2.0))));
    const double y1 =
        std::sqrt((radius * radius) - ((radius - ax) * (radius - ax)));
    const auto upper = static_cast<int32_t>(roundUpSqrt2(std::max(y0, y1)));
    for (auto yp = static_cast<int32_t>(roundDownSqrt2(std::min(y0, y1)));
         yp < upper; ++yp) {
      double y = static_cast<double>(-yp) * std::numbers::sqrt2;
      if (xp % 2 == 1) {
        y -= std::numbers::sqrt2 / 2.0;
      }
      // In exact arithmetic neither coordinate is a whole number: up to
      // MAX_BEND_RADIUS, each stays more than 0.003 away from one. A rounding
      // error of the steps before therefore cannot change the truncation.
      const auto cell = fromDiagonalFrame(ax, -(y + offset));
      result.push_back(
          {static_cast<int32_t>(cell[0]), static_cast<int32_t>(cell[1])});
    }
  }
  // The straight lead, then the arc.
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double ys = 0.0; ys <= -offset; ys += MovePrimitives::SAMPLE_SPACING) {
    samples.push_back(fromDiagonalFrame(0.0, ys));
  }
  if ((ranges[1] <= 45.0 && ranges[0] >= 45.0) ||
      (ranges[1] <= 0.0 && ranges[0] >= 0.0)) {
    // The floating-point accumulation is part of the table definition.
    // NOLINTBEGIN(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
    for (double xs = 0.0; xs <= vector[0];
         xs += MovePrimitives::SAMPLE_SPACING) {
      const double ax = xs * (std::numbers::sqrt2 / 2.0);
      const double circle =
          std::sqrt((radius * radius) - ((radius - ax) * (radius - ax)));
      samples.push_back(fromDiagonalFrame(ax, circle - offset));
    }
    // NOLINTEND(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  }
  const Vector16 end = diagonalEnd(static_cast<uint32_t>(vector[0]),
                                   static_cast<uint32_t>(-vector[1]));
  endSamplesAt(samples,
               {static_cast<double>(end[0]), static_cast<double>(end[1])});
  result.erase(std::ranges::unique(result).begin(), result.end());
  return result;
}

void generateDiagonal(const uint32_t radius,
                      std::array<HeadingTables, 8>& tables) {
  const auto r = static_cast<double>(radius);
  std::vector<std::array<double, 2>> ranges;
  std::map<uint32_t, Vector16> angleOf;
  std::map<uint32_t, Vector16> coordinateOf;
  const auto xUpper =
      static_cast<uint32_t>(std::floor(r / (std::numbers::sqrt2 / 2.0)));
  for (uint32_t i = 1; i <= xUpper; ++i) {
    const double ai = i * (std::numbers::sqrt2 / 2.0);
    const double y = std::sqrt((r * r) - ((r - ai) * (r - ai)));
    uint32_t yRounded = roundUpSqrt2(y);
    if (i % 2 == 1) {
      yRounded = roundUpSqrt2Offset(y);
    }
    const double angleMin =
        std::atan((r - ai) / std::sqrt((r * r) - ((r - ai) * (r - ai)))) *
        DEGREES;
    const uint32_t yUpper = roundUpSqrt2(r);
    const double offset = (i % 2 == 1) ? 1.0 : 0.0;
    for (uint32_t j = yRounded; (j + offset < yUpper) && (j <= yRounded + 5);
         ++j) {
      const double angleMax =
          std::atan((r - static_cast<double>(i)) / j) * DEGREES;
      ranges.push_back({angleMin, angleMax});
      const auto idx = static_cast<uint32_t>(ranges.size() - 1);
      coordinateOf[idx] = diagonalEnd(i, j);
      angleOf[idx] = {static_cast<int16_t>(i), static_cast<int16_t>(j)};
      if ((angleMin <= 45.0 && angleMax >= 45.0) ||
          (angleMin <= 0.0 && angleMax >= 0.0)) {
        break;
      }
    }
  }

  const std::array<double, 3> angleValues = {0.0, 45.0, 90.0};
  std::array<bool, 3> found = {false, false, false};
  HeadingTables canonical;
  std::set<uint32_t> keys;
  for (uint32_t i = 0; i < ranges.size(); ++i) {
    const double upper = ranges[i][0];
    const double lower = ranges[i][1];
    for (std::size_t a = 0; a < angleValues.size(); ++a) {
      if (found[a]) {
        continue;
      }
      const double angle = angleValues[a];
      if (angle <= upper && angle >= lower) {
        if (angle == 0.0 && coordinateOf[i][0] == 0 &&
            coordinateOf[i][1] == 0) {
          continue;
        }
        keys.insert(i);
        canonical.exit[i] = headingOfDegrees(90.0 - angle);
        canonical.vector[i] = coordinateOf[i];
        Vector16 look = angleOf[i];
        look[1] = static_cast<int16_t>(-look[1]);
        double cost = 0.0;
        Samples samples;
        canonical.swept[i] =
            arcCellsDiagonal(r, look, ranges[i], cost, samples);
        canonical.cost[i] = cost;
        canonical.samples[i] = samples;
        found[a] = true;
      }
    }
  }

  const auto n = static_cast<uint32_t>(ranges.size());
  for (uint16_t heading = 1; heading < 8; heading += 2) {
    const double degrees = (heading * 45.0) - 315.0;
    HeadingTables& t = tables[heading];
    for (const uint32_t idx : keys) {
      const uint16_t exit = canonical.exit.at(idx);
      const Heading turnedOne = turned(static_cast<Heading>(heading), exit);
      const Heading turnedOther = turned(static_cast<Heading>(heading), -exit);
      t.cost[idx] = canonical.cost.at(idx);
      t.cost[n + idx] = canonical.cost.at(idx);
      t.exit[idx] = turnedOther;
      t.exit[n + idx] = turnedOne;
      t.samples[idx] = rotate(
          invert(canonical.samples.at(idx), false, false, false), degrees);
      t.samples[n + idx] =
          rotate(invert(canonical.samples.at(idx), true, true, true), degrees);
      t.vector[idx] = rotate(coordinateOf.at(idx), degrees);
      t.vector[n + idx] =
          rotate(invert(coordinateOf.at(idx), true, true, true), degrees);
      t.swept[idx] =
          rotate(invert(canonical.swept.at(idx), false, false, false), degrees);
      t.swept[n + idx] =
          rotate(invert(canonical.swept.at(idx), true, true, true), degrees);
    }
    const uint32_t straightId = n * 2;
    Samples straightSamples;
    // The floating-point accumulation is part of the table definition.
    // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
    for (double y = 0.0; y <= 1.0; y += MovePrimitives::SAMPLE_SPACING) {
      straightSamples.push_back(rotate(std::array<double, 2>{y, -y}, degrees));
    }
    t.samples[straightId] = straightSamples;
    t.cost[straightId] = std::numbers::sqrt2;
    t.exit[straightId] = heading;
    const Vector16 step = rotate(Vector16{1, -1}, degrees);
    t.vector[straightId] = step;
    t.swept[straightId] = {{step[0], step[1]}};
  }
}

/// The quarter turns that leave a diagonal heading as arcs of a full right
/// angle. The canonical diagonal tables hold no such turn: at some radii they
/// hold no move to the quarter-turn heading, and at others, such as 10 and 13
/// cells, an arc of 72 to 77 degrees that ends on it. The identifiers lie
/// well above every other identifier.
void generateDiagonalQuarterTurns(const uint32_t radiusIn,
                                  std::array<HeadingTables, 8>& tables) {
  const auto radius = static_cast<double>(radiusIn);
  constexpr int arcSamples = 180;
  constexpr uint32_t clockwiseId = 900;
  constexpr uint32_t counterClockwiseId = 901;

  const auto direction = [](const Heading d) -> std::array<double, 2> {
    const HeadingVector v = headingVector(d);
    constexpr double invSqrt2 = 0.70710678118654752440;
    if (isDiagonal(d)) {
      return {v.dx * invSqrt2, v.dy * invSqrt2};
    }
    return {static_cast<double>(v.dx), static_cast<double>(v.dy)};
  };
  const auto arcPoint = [](const std::array<double, 2>& u0, const int turnSign,
                           const double r,
                           const double theta) -> std::array<double, 2> {
    if (turnSign < 0) {
      return {
          r * ((u0[1] * (std::cos(theta) - 1.0)) + (u0[0] * std::sin(theta))),
          r * ((u0[0] * (1.0 - std::cos(theta))) + (u0[1] * std::sin(theta)))};
    }
    return {
        r * ((u0[1] * (1.0 - std::cos(theta))) + (u0[0] * std::sin(theta))),
        r * ((u0[0] * (std::cos(theta) - 1.0)) + (u0[1] * std::sin(theta)))};
  };

  for (Heading entry = 1; entry <= 7; entry = static_cast<Heading>(entry + 2)) {
    const auto u0 = direction(entry);
    for (const int turnSign : {-1, 1}) {
      const Heading exit = turned(entry, 2 * turnSign);
      const uint32_t id = (turnSign > 0) ? clockwiseId : counterClockwiseId;

      IntCells cells;
      int32_t lastX = 0;
      int32_t lastY = 0;
      bool first = true;
      for (int s = 1; s <= arcSamples; ++s) {
        const double theta = (PI / 2.0) * (static_cast<double>(s) / arcSamples);
        const auto [px, py] = arcPoint(u0, turnSign, radius, theta);
        const auto gx = static_cast<int32_t>(std::lround(px));
        const auto gy = static_cast<int32_t>(std::lround(py));
        if (!first && gx == lastX && gy == lastY) {
          continue;
        }
        cells.push_back({gx, gy});
        lastX = gx;
        lastY = gy;
        first = false;
      }

      Samples samples;
      const double arcLength = (PI / 2.0) * radius;
      const int fine =
          std::max(2, static_cast<int>(std::ceil(
                          arcLength / MovePrimitives::SAMPLE_SPACING)));
      samples.push_back({0.0, 0.0});
      for (int s = 1; s <= fine; ++s) {
        const double theta = (PI / 2.0) * (static_cast<double>(s) / fine);
        samples.push_back(arcPoint(u0, turnSign, radius, theta));
      }

      HeadingTables& t = tables[entry];
      t.exit[id] = exit;
      t.vector[id] = {static_cast<int16_t>(lastX), static_cast<int16_t>(lastY)};
      t.swept[id] = std::move(cells);
      t.cost[id] = std::ceil((PI / 2.0) * radius);
      t.samples[id] = std::move(samples);
    }
  }
}

} // namespace

MovePrimitives::MovePrimitives(const uint32_t minRadius)
    : bendRadius(minRadius) {
  if (minRadius == 0) {
    throw std::invalid_argument("the bend radius must be at least one cell");
  }
  if (minRadius > MAX_BEND_RADIUS) {
    throw std::invalid_argument(
        "the bend radius needs more primitive identifiers than a state holds");
  }
  std::array<HeadingTables, 8> tables;
  generateCardinal(minRadius, tables);
  generateDiagonal(minRadius, tables);
  generateDiagonalQuarterTurns(minRadius, tables);

  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    indexOf[heading].fill(-1);
    const HeadingTables& t = tables[heading];
    bool haveStraight = false;
    for (const auto& [id, vector] : t.vector) {
      // A primitive is a move with an end and an exit heading.
      if (id >= MAX_PRIMITIVE_ID || !t.exit.contains(id)) {
        continue;
      }
      Primitive primitive;
      primitive.id = static_cast<uint16_t>(id);
      primitive.exitHeading = static_cast<Heading>(t.exit.at(id) & 7U);
      primitive.dx = vector[0];
      primitive.dy = vector[1];
      primitive.cost = t.cost.contains(id) ? t.cost.at(id) : 0.0;
      if (t.swept.contains(id)) {
        for (const auto& c : t.swept.at(id)) {
          primitive.swept.push_back({.dx = static_cast<int16_t>(c[0]),
                                     .dy = static_cast<int16_t>(c[1])});
        }
      }
      if (t.samples.contains(id)) {
        for (const auto& s : t.samples.at(id)) {
          primitive.samples.emplace_back(s[0], s[1]);
        }
      }
      if (primitive.exitHeading == heading && !haveStraight) {
        straightIds[heading] = primitive.id;
        haveStraight = true;
      }
      indexOf[heading][id] = static_cast<int16_t>(byHeading[heading].size());
      byHeading[heading].push_back(std::move(primitive));
    }
  }
}

const Primitive* MovePrimitives::find(const Heading heading,
                                      const uint32_t id) const {
  if (id >= MAX_PRIMITIVE_ID) {
    return nullptr;
  }
  const int16_t index = indexOf[heading & 7U][id];
  if (index < 0) {
    return nullptr;
  }
  return &byHeading[heading & 7U][static_cast<std::size_t>(index)];
}

double MovePrimitives::cost(const Heading heading, const uint32_t id) const {
  const Primitive* primitive = find(heading, id);
  return primitive != nullptr ? primitive->cost : 0.0;
}

bool MovePrimitives::isStraight(const Heading heading,
                                const uint32_t id) const {
  const Primitive* primitive = find(heading, id);
  // An identifier the heading lacks reads as exit heading zero, so it counts
  // as straight for heading zero only.
  const Heading exit = primitive != nullptr ? primitive->exitHeading : 0;
  return exit == (heading & 7U);
}

} // namespace mqt::scpd::routing
