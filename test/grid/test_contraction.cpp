/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"

#include <gtest/gtest.h>

namespace {

using mqt::scpd::grid::distanceToSegment;
using mqt::scpd::grid::Point;

TEST(Contraction, TheGridLibraryRoundsEachSquareBeforeTheSum) {
  // The squares of 0.84 and 1.12 round on their own, and the root of their
  // sum is the double nearest 1.4. A fused multiply-add keeps one square
  // exact, and the root becomes the next double above. A cell at this
  // distance then falls outside a keepout of 1.4. Clang contracts by default
  // where the target has a fused multiply-add, so on arm64 this test fails
  // when the grid library is compiled without -ffp-contract=off.
  EXPECT_EQ(
      distanceToSegment(Point(0.84, 1.12), Point(0.0, 0.0), Point(0.0, 0.0)),
      1.4);
}

} // namespace
