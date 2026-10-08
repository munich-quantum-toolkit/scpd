/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

TEST(RouteGeometry, ATurnOntoADiagonalPreservesTheBendRadiusAtTheJoin) {
  using namespace mqt::scpd;
  using namespace mqt::scpd::routing;
  const grid::BitGrid obstacles(120, 120);
  const grid::BitGrid outsideCorridor(120, 120);
  auto primitives = std::make_shared<MovePrimitives>(5);
  SearchScratch scratch(120, 120);
  DubinsRouter router(primitives, scratch);
  router.attachObstacles(&obstacles);
  router.attachCorridor(&outsideCorridor);
  router.setParams(
      {.startStraightLength = 10, .endStraightLength = 10, .minRadius = 5});
  const auto moves = primitives->of(0);
  const auto turn = std::ranges::find_if(
      moves, [](const Primitive& move) { return move.exitHeading == 7; });
  ASSERT_NE(turn, moves.end());
  Path path =
      router.route({.source = {.x = 40, .y = 40, .heading = 0},
                    .target = {.x = static_cast<uint32_t>(50 + turn->dx),
                               .y = static_cast<uint32_t>(20 + turn->dy),
                               .heading = 7}});
  ASSERT_FALSE(path.empty());
  std::vector<PathSegment> segments;
  const auto points = samplePath(*primitives, path, path.front(), segments);
  ASSERT_GT(points.size(), 2U);
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double ax = points[i].x() - points[i - 1].x();
    const double ay = points[i].y() - points[i - 1].y();
    const double bx = points[i + 1].x() - points[i].x();
    const double by = points[i + 1].y() - points[i].y();
    const double cross = std::abs((ax * by) - (ay * bx));
    if (cross < 1e-10) {
      continue;
    }
    const double radius = std::hypot(ax, ay) * std::hypot(bx, by) *
                          std::hypot(ax + bx, ay + by) / (2.0 * cross);
    EXPECT_GE(radius, 5.0 - 1e-6) << "sample " << i;
  }
}

} // namespace
