/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace mqt::scpd::routing {

/**
 * @brief The number of headings of the router grid.
 *
 * The router moves in eight directions. The search state index and the
 * primitive tables depend on this count.
 */
inline constexpr uint32_t NUM_HEADINGS = 8;

/**
 * @brief A heading of the router grid, in the range 0..7.
 *
 * A heading is the direction of travel of a wire. Heading 0 travels toward
 * negative y. With y pointing up, the headings continue clockwise in eighth
 * turns: heading 2 travels toward negative x, heading 4 toward positive y and
 * heading 6 toward positive x. The even headings are the cardinal headings,
 * which run along an axis. The odd headings are the diagonal headings between
 * them.
 */
using Heading = uint8_t;

/**
 * @brief The unit step of a heading along each axis, in cells.
 */
struct HeadingVector {
  /// The step along x: -1, 0 or 1.
  int8_t dx = 0;
  /// The step along y: -1, 0 or 1.
  int8_t dy = 0;
};

/**
 * @brief Computes the unit step of a heading.
 * @param heading The heading. Only its three low bits are read, so a value
 * above 7 is taken modulo 8.
 * @return The step along each axis. A cardinal heading steps along one axis,
 * a diagonal heading along both.
 */
[[nodiscard]] constexpr HeadingVector headingVector(const Heading heading) {
  switch (heading & 7U) {
  case 0:
    return {.dx = 0, .dy = -1};
  case 1:
    return {.dx = -1, .dy = -1};
  case 2:
    return {.dx = -1, .dy = 0};
  case 3:
    return {.dx = -1, .dy = 1};
  case 4:
    return {.dx = 0, .dy = 1};
  case 5:
    return {.dx = 1, .dy = 1};
  case 6:
    return {.dx = 1, .dy = 0};
  default:
    return {.dx = 1, .dy = -1};
  }
}

/**
 * @brief Reports whether a heading runs along a diagonal.
 * @param heading The heading to test.
 * @return @c true for an odd heading, @c false for a cardinal heading.
 */
[[nodiscard]] constexpr bool isDiagonal(const Heading heading) {
  return (heading & 1U) != 0U;
}

/**
 * @brief Computes the opposite heading.
 * @param heading The heading to reverse.
 * @return The heading four eighth turns from @p heading, in the range 0..7.
 */
[[nodiscard]] constexpr Heading reverse(const Heading heading) {
  return static_cast<Heading>((heading + 4U) & 7U);
}

/**
 * @brief Turns a heading clockwise by a number of eighth turns.
 * @param heading The heading to turn.
 * @param eighths The number of eighth turns. A negative value turns
 * counter-clockwise.
 * @return The turned heading, in the range 0..7.
 */
[[nodiscard]] constexpr Heading turned(const Heading heading,
                                       const int eighths) {
  return static_cast<Heading>(
      (((static_cast<int>(heading) + eighths) % 8) + 8) % 8);
}

/**
 * @brief Counts the eighth turns between two headings.
 *
 * The count takes the shorter way around, so it does not depend on the
 * direction of the turn.
 *
 * @param a The first heading.
 * @param b The second heading.
 * @pre @p a and @p b lie in the range 0..7.
 * @return The number of eighth turns, in the range 0..4.
 */
[[nodiscard]] constexpr uint32_t headingDistance(const Heading a,
                                                 const Heading b) {
  const uint32_t raw =
      (a > b) ? static_cast<uint32_t>(a - b) : static_cast<uint32_t>(b - a);
  return raw < NUM_HEADINGS - raw ? raw : NUM_HEADINGS - raw;
}

/**
 * @brief Reports whether two headings cross at a right angle.
 * @param a The first heading.
 * @param b The second heading.
 * @pre @p a and @p b lie in the range 0..7.
 * @return @c true when the two headings are two eighth turns apart.
 */
[[nodiscard]] constexpr bool isOrthogonal(const Heading a, const Heading b) {
  return headingDistance(a, b) == 2;
}

/**
 * @brief Converts the orientation of a port to a heading.
 *
 * The orientation is the direction the port faces, in degrees, as the chip
 * input carries it. The result is the heading a wire has when it arrives at
 * the port, so it points opposite to the orientation: 0 degrees gives
 * heading 2, and 90 degrees gives heading 0. A wire that leaves the port has
 * the reverse heading.
 *
 * @param degrees The orientation of the port. A value outside [0, 360) is
 * folded into that range first.
 * @return The heading when the orientation is an exact multiple of 45
 * degrees, and @c NUM_HEADINGS for any other orientation, an infinite one and
 * NaN.
 */
[[nodiscard]] inline Heading headingOfOrientation(const double degrees) {
  // The remainder is exact, so a multiple of 45 folds onto a multiple of 45 at
  // any magnitude. Adding a full turn to a negative remainder is exact for a
  // multiple of 45; any other remainder has no heading, rounded or not.
  double folded = std::fmod(degrees, 360.0);
  if (folded < 0.0) {
    folded += 360.0;
  }
  if (folded == 0.0) {
    return 2;
  }
  if (folded == 45.0) {
    return 1;
  }
  if (folded == 90.0) {
    return 0;
  }
  if (folded == 135.0) {
    return 7;
  }
  if (folded == 180.0) {
    return 6;
  }
  if (folded == 225.0) {
    return 5;
  }
  if (folded == 270.0) {
    return 4;
  }
  if (folded == 315.0) {
    return 3;
  }
  return static_cast<Heading>(NUM_HEADINGS);
}

} // namespace mqt::scpd::routing
