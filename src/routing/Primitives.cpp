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
#include <compare>
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
// whether it does depends on the compiler and the math library. The test of a
// candidate move is exact too: whether its turn angles hold an eighth or a
// quarter turn is decided from whole numbers (see TurnAngles). A candidate
// can end exactly on the boundary of an eighth turn, and a floating-point
// angle there would depend on the last bit of the arc tangent.

namespace mqt::scpd::routing {

namespace {

constexpr double PI = std::numbers::pi;

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

/// The exit heading of an eighth turn, in eighth turns from the heading the
/// turn leaves.
constexpr uint16_t EIGHTH_TURN = 1;
/// The exit heading of a quarter turn, in eighth turns from the heading the
/// turn leaves.
constexpr uint16_t QUARTER_TURN = 2;

/// The turn angles of a candidate move against an eighth and a quarter turn,
/// decided in exact arithmetic.
///
/// A candidate ends at a whole cell: it runs straight along its heading and
/// then along an arc of the bend radius that turns toward the end. Its turn
/// angles form a range with two ends. The arc angle is the angle by which the
/// arc has turned where it reaches the column of the end cell. The end angle
/// is the angle by which a turn about the center of the arc reaches the end
/// cell. Each member compares one of the two angles with 45 or 90 degrees.
/// The comparisons use whole numbers only, so a candidate on the boundary,
/// such as an end angle of exactly 45 degrees, does not depend on rounding.
struct TurnAngles {
  /// The arc angle against 45 degrees.
  std::strong_ordering arcToEighth;
  /// The end angle against 45 degrees.
  std::strong_ordering endToEighth;
  /// The arc angle against 90 degrees.
  std::strong_ordering arcToQuarter;
  /// The end angle against 90 degrees.
  std::strong_ordering endToQuarter;

  /// Whether the range from the arc angle up to the end angle holds 45
  /// degrees, both ends included.
  [[nodiscard]] bool holdEighth() const {
    return arcToEighth <= 0 && endToEighth >= 0;
  }
  /// Whether the range from the arc angle up to the end angle holds 90
  /// degrees, both ends included.
  [[nodiscard]] bool holdQuarter() const {
    return arcToQuarter <= 0 && endToQuarter >= 0;
  }
};

// ---------------------------------------------------------------------------
// Cardinal headings.
// ---------------------------------------------------------------------------

/// The turn angles of a candidate that leaves the canonical cardinal heading
/// and ends @p across cells to the side and @p ahead cells ahead.
///
/// The center of the arc lies @p radius cells to the side of the start, so
/// @c a = radius - across columns separate the end cell from the center. The
/// arc reaches the column of the end cell where the tangent of 90 degrees
/// less the arc angle is a / sqrt(radius^2 - a^2). The tangent of 90 degrees
/// less the end angle is a / ahead.
TurnAngles cardinalTurnAngles(const int64_t radius, const int64_t across,
                              const int64_t ahead) {
  const int64_t a = radius - across;
  return {.arcToEighth = radius * radius <=> 2 * a * a,
          .endToEighth = ahead <=> a,
          .arcToQuarter = 0 <=> a,
          .endToQuarter = 0 <=> a};
}

/// The swept cells, the cost and the samples of one arc that leaves the
/// canonical cardinal heading. A straight part along the heading comes
/// before the arc. The swept cells of the arc start from the end of the
/// straight part rounded to a whole cell toward the start. The samples start
/// the arc at the exact end of the straight part.
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
  // The candidates in the order of their identifiers. The candidates of one
  // column run ahead from the arc and stop at the first one whose turn angles
  // hold an eighth or a quarter turn.
  std::vector<TurnAngles> candidates;
  std::map<uint32_t, Vector16> vectorOf;
  for (uint32_t i = 1; i <= radius; ++i) {
    const double y =
        std::sqrt((r * r) - (static_cast<double>(radius - i) * (radius - i)));
    const double yUp = std::ceil(y);
    const double yDown = std::floor(y);
    for (int j = static_cast<int>(yUp); j <= signedRadius && j <= yDown + 2;
         ++j) {
      const TurnAngles angles = cardinalTurnAngles(radius, i, j);
      candidates.push_back(angles);
      vectorOf[static_cast<uint32_t>(candidates.size() - 1)] = {
          static_cast<int16_t>(i), static_cast<int16_t>(-j)};
      if (angles.holdEighth() || angles.holdQuarter()) {
        break;
      }
    }
  }

  // Every candidate whose turn angles hold an eighth or a quarter turn
  // becomes a move. No candidate holds both: a quarter turn ends in the
  // column of the center, an eighth turn at least radius / sqrt(2) columns
  // before it.
  HeadingTables canonical;
  std::set<uint32_t> keys;
  for (uint32_t i = 0; i < candidates.size(); ++i) {
    const bool quarter = candidates[i].holdQuarter();
    if (!quarter && !candidates[i].holdEighth()) {
      continue;
    }
    keys.insert(i);
    canonical.exit[i] = quarter ? QUARTER_TURN : EIGHTH_TURN;
    canonical.vector[i] = vectorOf[i];
    double cost = 0.0;
    Samples samples;
    canonical.swept[i] = arcCellsCardinal(r, vectorOf[i], cost, samples);
    canonical.cost[i] = cost;
    canonical.samples[i] = samples;
  }

  const auto n = static_cast<uint32_t>(candidates.size());
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

/// The turn angles of a candidate that leaves the canonical diagonal heading
/// and ends @p across half diagonals to the side and @p ahead diagonals ahead,
/// plus half a diagonal for an odd @p across.
///
/// The arc angle is the true angle of the arc. The center of the arc lies
/// @p radius cells to the side of the start, and the arc reaches the column of
/// the end cell
/// @p across / sqrt(2) cells to the side. The arc angle is therefore below 45
/// degrees while (radius + across)^2 < 2 radius^2, and below 90 degrees
/// while across^2 < 2 radius^2. The end angle reads the counts of half
/// diagonals and of diagonals as if they were cells: the tangent of 90
/// degrees less the end angle is (radius - across) / ahead.
TurnAngles diagonalTurnAngles(const int64_t radius, const int64_t across,
                              const int64_t ahead) {
  const int64_t a = radius - across;
  return {.arcToEighth =
              (radius + across) * (radius + across) <=> 2 * radius * radius,
          .endToEighth = ahead <=> a,
          .arcToQuarter = across * across <=> 2 * radius * radius,
          .endToQuarter = 0 <=> a};
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
                          double& cost, Samples& samples) {
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

  // The straight part in the diagonal frame runs whole diagonals ahead, which
  // are whole cells on the grid.
  int32_t ahead = 0;
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double yp = 0.0; yp >= std::floor(offset); yp -= std::numbers::sqrt2) {
    result.push_back({ahead, -ahead});
    ++ahead;
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
  // The straight part, then the arc.
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double ys = 0.0; ys <= -offset; ys += MovePrimitives::SAMPLE_SPACING) {
    samples.push_back(fromDiagonalFrame(0.0, ys));
  }
  // The floating-point accumulation is part of the table definition.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter,bugprone-float-loop-counter)
  for (double xs = 0.0; xs <= vector[0]; xs += MovePrimitives::SAMPLE_SPACING) {
    const double ax = xs * (std::numbers::sqrt2 / 2.0);
    const double circle =
        std::sqrt((radius * radius) - ((radius - ax) * (radius - ax)));
    samples.push_back(fromDiagonalFrame(ax, circle - offset));
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
  // The candidates in the order of their identifiers. The candidates of one
  // column run ahead from the arc and stop at the first one whose end angle
  // is at most, and whose arc angle at least, an eighth or a quarter turn.
  std::vector<TurnAngles> candidates;
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
    const uint32_t yUpper = roundUpSqrt2(r);
    const double offset = (i % 2 == 1) ? 1.0 : 0.0;
    for (uint32_t j = yRounded; (j + offset < yUpper) && (j <= yRounded + 5);
         ++j) {
      const TurnAngles angles = diagonalTurnAngles(radius, i, j);
      candidates.push_back(angles);
      const auto idx = static_cast<uint32_t>(candidates.size() - 1);
      coordinateOf[idx] = diagonalEnd(i, j);
      angleOf[idx] = {static_cast<int16_t>(i), static_cast<int16_t>(j)};
      if ((angles.arcToEighth >= 0 && angles.endToEighth <= 0) ||
          (angles.arcToQuarter >= 0 && angles.endToQuarter <= 0)) {
        break;
      }
    }
  }

  // The first candidate whose turn angles hold a quarter turn, and that does
  // not end on its start, becomes the quarter turn. The first candidate whose
  // turn angles hold an eighth turn becomes the eighth turn. No candidate
  // holds both: a quarter turn ends at least radius half diagonals to the
  // side, an eighth turn at most radius (sqrt(2) - 1) half diagonals.
  bool foundQuarter = false;
  bool foundEighth = false;
  HeadingTables canonical;
  std::set<uint32_t> keys;
  for (uint32_t i = 0; i < candidates.size(); ++i) {
    const bool quarter = !foundQuarter && candidates[i].holdQuarter() &&
                         (coordinateOf[i][0] != 0 || coordinateOf[i][1] != 0);
    const bool eighth = !foundEighth && candidates[i].holdEighth();
    if (!quarter && !eighth) {
      continue;
    }
    keys.insert(i);
    canonical.exit[i] = quarter ? QUARTER_TURN : EIGHTH_TURN;
    canonical.vector[i] = coordinateOf[i];
    Vector16 look = angleOf[i];
    look[1] = static_cast<int16_t>(-look[1]);
    double cost = 0.0;
    Samples samples;
    canonical.swept[i] = arcCellsDiagonal(r, look, cost, samples);
    canonical.cost[i] = cost;
    canonical.samples[i] = samples;
    foundQuarter = foundQuarter || quarter;
    foundEighth = foundEighth || eighth;
  }

  const auto n = static_cast<uint32_t>(candidates.size());
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

/// The unit vector of a heading.
std::array<double, 2> direction(const Heading heading) {
  const HeadingVector v = headingVector(heading);
  constexpr double invSqrt2 = 0.70710678118654752440;
  if (isDiagonal(heading)) {
    return {v.dx * invSqrt2, v.dy * invSqrt2};
  }
  return {static_cast<double>(v.dx), static_cast<double>(v.dy)};
}

/// The point at which an arc has turned by @p theta radians. The arc starts at
/// the origin along the unit vector @p u0 and has the radius @p r. A positive
/// @p turnSign turns clockwise, in the sense of turned().
std::array<double, 2> arcPoint(const std::array<double, 2>& u0,
                               const int turnSign, const double r,
                               const double theta) {
  if (turnSign < 0) {
    return {
        r * ((u0[1] * (std::cos(theta) - 1.0)) + (u0[0] * std::sin(theta))),
        r * ((u0[0] * (1.0 - std::cos(theta))) + (u0[1] * std::sin(theta)))};
  }
  return {r * ((u0[1] * (1.0 - std::cos(theta))) + (u0[0] * std::sin(theta))),
          r * ((u0[0] * (std::cos(theta) - 1.0)) + (u0[1] * std::sin(theta)))};
}

/// An arc of the bend radius that turns by a whole number of eighth turns.
struct ExactTurn {
  /// The cells the arc passes, from its start to the cell nearest its end.
  IntCells cells;
  /// The cell nearest the end of the arc.
  Vector16 end{};
  /// Points at equal angles along the arc, from the origin to its end.
  Samples samples;
  /// The length of the arc rounded up to whole cells.
  double cost = 0.0;
};

/// Builds an arc of the radius @p radius that leaves @p entry and turns by
/// @p eighths eighth turns, clockwise for a positive @p turnSign in the sense
/// of turned().
///
/// The cells round 90 points per eighth turn, at equal angles, to the
/// nearest cell. The samples lie at equal angles, at most
/// MovePrimitives::SAMPLE_SPACING apart along the arc.
ExactTurn exactTurn(const Heading entry, const int turnSign,
                    const double radius, const int eighths) {
  const auto u0 = direction(entry);
  const double sweep = eighths * (PI / 4.0);
  const int arcSamples = 90 * eighths;
  ExactTurn turn;

  // The cells start with the start of the arc, as those of every other turn
  // do. Up to a radius of 80 cells, the first point rounds onto that cell; at
  // a larger radius it rounds onto a neighbor.
  turn.cells = {{0, 0}};
  int32_t lastX = 0;
  int32_t lastY = 0;
  for (int s = 1; s <= arcSamples; ++s) {
    const double theta = sweep * (static_cast<double>(s) / arcSamples);
    const auto [px, py] = arcPoint(u0, turnSign, radius, theta);
    const auto gx = static_cast<int32_t>(std::lround(px));
    const auto gy = static_cast<int32_t>(std::lround(py));
    if (gx == lastX && gy == lastY) {
      continue;
    }
    turn.cells.push_back({gx, gy});
    lastX = gx;
    lastY = gy;
  }
  turn.end = {static_cast<int16_t>(lastX), static_cast<int16_t>(lastY)};

  const double arcLength = sweep * radius;
  const int fine = std::max(
      2,
      static_cast<int>(std::ceil(arcLength / MovePrimitives::SAMPLE_SPACING)));
  turn.samples.push_back({0.0, 0.0});
  for (int s = 1; s <= fine; ++s) {
    const double theta = sweep * (static_cast<double>(s) / fine);
    turn.samples.push_back(arcPoint(u0, turnSign, radius, theta));
  }
  turn.cost = std::ceil(arcLength);
  return turn;
}

/// Adds an exact turn to the tables of a heading under the identifier @p id.
void addExactTurn(HeadingTables& t, const uint32_t id, const Heading exit,
                  ExactTurn turn) {
  t.exit[id] = exit;
  t.vector[id] = turn.end;
  t.swept[id] = std::move(turn.cells);
  t.cost[id] = turn.cost;
  t.samples[id] = std::move(turn.samples);
}

/// The quarter turns that leave a diagonal heading as arcs of a full right
/// angle. The canonical diagonal tables hold no such turn: at some radii they
/// hold no move to the quarter-turn heading, and at others, such as 10 and 13
/// cells, an arc of 72 to 77 degrees that ends on it. The identifiers are 900
/// and 901. Up to a radius of 80 cells, they lie above every other identifier.
/// At the radii of 81 and of 84 to 90 cells, other moves of a diagonal heading
/// have higher identifiers, but no other move has 900 or 901.
///
/// @throws std::logic_error If another move of a diagonal heading already has
/// the identifier of an exact quarter turn.
void generateDiagonalQuarterTurns(const uint32_t radiusIn,
                                  std::array<HeadingTables, 8>& tables) {
  const auto radius = static_cast<double>(radiusIn);
  constexpr uint32_t clockwiseId = 900;
  constexpr uint32_t counterClockwiseId = 901;

  for (Heading entry = 1; entry <= 7; entry = static_cast<Heading>(entry + 2)) {
    for (const int turnSign : {-1, 1}) {
      const uint32_t id = (turnSign > 0) ? clockwiseId : counterClockwiseId;
      HeadingTables& t = tables[entry];
      if (t.vector.contains(id)) {
        throw std::logic_error(
            "a diagonal move has the identifier of an exact quarter turn");
      }
      addExactTurn(t, id, turned(entry, 2 * turnSign),
                   exactTurn(entry, turnSign, radius, 2));
    }
  }
}

/// The eighth turns, as arcs of 45 degrees, of every heading that holds no
/// eighth turn to a side. Whether a heading holds one depends on how the arc
/// of the radius rounds onto the cells (see TurnAngles). The identifiers are
/// 902 for a clockwise and 903 for a counterclockwise turn, in the sense of
/// turned(). Up to a radius of 80 cells, they lie above every other
/// identifier. At the radii of 81 and of 84 to 90 cells, other moves of a
/// diagonal heading have higher identifiers, but no other move has 902 or
/// 903.
///
/// @throws std::logic_error If another move of the heading already has the
/// identifier of an exact eighth turn.
void generateExactEighthTurns(const uint32_t radiusIn,
                              std::array<HeadingTables, 8>& tables) {
  const auto radius = static_cast<double>(radiusIn);
  constexpr uint32_t clockwiseId = 902;
  constexpr uint32_t counterClockwiseId = 903;

  for (Heading entry = 0; entry < 8; ++entry) {
    for (const int turnSign : {-1, 1}) {
      const Heading exit = turned(entry, turnSign);
      HeadingTables& t = tables[entry];
      // A move counts when the constructor keeps it as a primitive.
      if (std::ranges::any_of(t.exit, [&t, exit](const auto& idAndExit) {
            const auto& [id, moveExit] = idAndExit;
            return id < MovePrimitives::MAX_PRIMITIVE_ID &&
                   t.vector.contains(id) && (moveExit & 7U) == exit;
          })) {
        continue;
      }
      const uint32_t id = (turnSign > 0) ? clockwiseId : counterClockwiseId;
      if (t.vector.contains(id)) {
        throw std::logic_error(
            "a move has the identifier of an exact eighth turn");
      }
      addExactTurn(t, id, exit, exactTurn(entry, turnSign, radius, 1));
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
  generateExactEighthTurns(minRadius, tables);

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
