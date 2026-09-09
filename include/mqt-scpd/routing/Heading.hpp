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

#include <cstdint>

namespace mqt::scpd::routing {

/// The eight-way headings of the router grid. The assumption is packed into
/// the search state index and the primitive tables, so it is named here and
/// nowhere spelled as a literal.
inline constexpr uint32_t NUM_HEADINGS = 8;

/// A heading 0..7. Heading 0 travels toward negative y and the headings
/// continue clockwise in eighth turns when y points up: 2 travels toward
/// negative x, 4 toward positive y, 6 toward positive x.
using Heading = uint8_t;

/// The unit step of a heading along each axis, in cells.
struct HeadingVector {
  int8_t dx = 0;
  int8_t dy = 0;
};

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

/// Whether a heading runs along a diagonal.
[[nodiscard]] constexpr bool isDiagonal(const Heading heading) {
  return (heading & 1U) != 0U;
}

/// The opposite heading.
[[nodiscard]] constexpr Heading reverse(const Heading heading) {
  return static_cast<Heading>((heading + 4U) & 7U);
}

/// The heading a number of eighth turns clockwise from another.
[[nodiscard]] constexpr Heading turned(const Heading heading, const int eighths) {
  return static_cast<Heading>(((static_cast<int>(heading) + eighths) % 8 + 8) % 8);
}

/// The number of eighth turns between two headings, 0..4.
[[nodiscard]] constexpr uint32_t headingDistance(const Heading a, const Heading b) {
  const uint32_t raw = (a > b) ? static_cast<uint32_t>(a - b)
                               : static_cast<uint32_t>(b - a);
  return raw < NUM_HEADINGS - raw ? raw : NUM_HEADINGS - raw;
}

/// Whether two headings cross at a right angle.
[[nodiscard]] constexpr bool isOrthogonal(const Heading a, const Heading b) {
  return headingDistance(a, b) == 2;
}

/// The heading of a port whose orientation, in degrees as the chip input
/// carries it, points along one of the eight directions: the heading a wire
/// has when it arrives at the port. A wire that leaves the port has the
/// reverse heading. Returns NUM_HEADINGS for any other orientation.
[[nodiscard]] constexpr Heading headingOfOrientation(const double degrees) {
  // Fold into [0, 360) exactly for the multiples of 45 the inputs carry.
  double folded = degrees;
  while (folded < 0.0) {
    folded += 360.0;
  }
  while (folded >= 360.0) {
    folded -= 360.0;
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
