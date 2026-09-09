/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

const MovePrimitives PRIMITIVES(5);

/// A straight run of a heading from a cell.
Path straightRun(const uint32_t x0, const uint32_t y0, const Heading heading,
                 const uint32_t steps) {
  Path path;
  const HeadingVector v = headingVector(heading);
  const uint16_t id = PRIMITIVES.straight(heading);
  for (uint32_t i = 0; i <= steps; ++i) {
    path.push_back({.x = static_cast<uint32_t>(x0 + (v.dx * i)),
                    .y = static_cast<uint32_t>(y0 + (v.dy * i)),
                    .heading = heading,
                    .primitive = id});
  }
  return path;
}

TEST(PathGeometry, AStraightRunIsOneSegment) {
  const Path path = straightRun(100, 100, 6, 20);
  const SegmentedPath segmented = reconstructSegments(PRIMITIVES, path);
  ASSERT_EQ(segmented.segments.size(), 1U);
  const PathSegment& segment = segmented.segments.front();
  EXPECT_TRUE(segment.straight);
  EXPECT_EQ(segment.heading, 6);
  EXPECT_EQ(segment.steps(), 21U);
  EXPECT_EQ(segment.lengthAt.size(), 21U);
  EXPECT_DOUBLE_EQ(segmented.nominalLength, 21.0);
}

TEST(PathGeometry, AHeadingChangeStartsANewSegment) {
  Path path = straightRun(100, 100, 6, 10);
  const Path second = straightRun(111, 100, 4, 10);
  path.insert(path.end(), second.begin(), second.end());
  const SegmentedPath segmented = reconstructSegments(PRIMITIVES, path);
  ASSERT_EQ(segmented.segments.size(), 2U);
  EXPECT_EQ(segmented.segments[0].heading, 6);
  EXPECT_EQ(segmented.segments[1].heading, 4);
  EXPECT_TRUE(segmented.segments[0].straight);
  EXPECT_TRUE(segmented.segments[1].straight);
  // The lengths accumulate across the segments.
  EXPECT_GT(segmented.segments[1].lengthAt.back(), segmented.segments[0].lengthAt.back());
}

TEST(PathGeometry, ATurnPrimitiveStaysOneStepAtItsRealEnd) {
  // A turn lists every cell it sweeps under one tag; the segment must end
  // on the last of them, not on the first.
  const Primitive* quarter = nullptr;
  for (const Primitive& p : PRIMITIVES.of(6)) {
    if (headingDistance(6, p.exitHeading) == 2) {
      quarter = &p;
      break;
    }
  }
  ASSERT_NE(quarter, nullptr);
  Path path = straightRun(100, 100, 6, 5);
  for (const CellOffset& c : quarter->swept) {
    path.push_back({.x = static_cast<uint32_t>(105 + c.dx),
                    .y = static_cast<uint32_t>(100 + c.dy),
                    .heading = 6,
                    .primitive = quarter->id});
  }
  const SegmentedPath segmented = reconstructSegments(PRIMITIVES, path);
  ASSERT_EQ(segmented.segments.size(), 2U);
  const PathSegment& turn = segmented.segments[1];
  EXPECT_FALSE(turn.straight);
  EXPECT_EQ(turn.steps(), 1U);
  EXPECT_EQ(turn.cells[0].x, static_cast<uint32_t>(105 + quarter->swept.back().dx));
  EXPECT_EQ(turn.cells[0].y, static_cast<uint32_t>(100 + quarter->swept.back().dy));
}

TEST(PathGeometry, SamplingRendersTheStraightRunAtItsRealLength) {
  Path path = straightRun(100, 100, 6, 20);
  std::vector<PathSegment> segments;
  const std::vector<Point> samples = samplePath(PRIMITIVES, path, path.front(), segments);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.front().x(), 100.0);
  EXPECT_DOUBLE_EQ(samples.back().x(), 120.0);
  EXPECT_DOUBLE_EQ(samples.back().y(), 100.0);
  EXPECT_NEAR(polylineLength(samples), 20.0, 1e-9);
  // The rendered length is what the segments report, so a caller that
  // targets a length reads the real one.
  EXPECT_NEAR(segments.back().lengthAt.back(), 20.0, 1e-9);
}

TEST(PathGeometry, SamplingSnapsEveryStepOntoItsGridCell) {
  const Primitive* quarter = nullptr;
  for (const Primitive& p : PRIMITIVES.of(0)) {
    if (p.exitHeading == 6) {
      quarter = &p;
      break;
    }
  }
  ASSERT_NE(quarter, nullptr);
  Path path = straightRun(50, 80, 0, 10);
  const uint32_t turnX = 50;
  const uint32_t turnY = 70;
  for (const CellOffset& c : quarter->swept) {
    path.push_back({.x = static_cast<uint32_t>(turnX + c.dx),
                    .y = static_cast<uint32_t>(turnY + c.dy),
                    .heading = 0,
                    .primitive = quarter->id});
  }
  const Path after = straightRun(turnX + 5, turnY - 5, 6, 10);
  path.insert(path.end(), after.begin(), after.end());

  std::vector<PathSegment> segments;
  const std::vector<Point> samples = samplePath(PRIMITIVES, path, path.front(), segments);
  ASSERT_FALSE(samples.empty());
  // The rendering ends exactly on the last grid cell, so it never drifts.
  EXPECT_NEAR(samples.back().x(), static_cast<double>(path.back().x), 1e-9);
  EXPECT_NEAR(samples.back().y(), static_cast<double>(path.back().y), 1e-9);
  // The two straight runs plus the arc, which renders at its true length
  // rather than as the chord across its ends.
  const double length = polylineLength(samples);
  EXPECT_NEAR(length, 20.0 + (std::numbers::pi / 2.0 * 5.0), 0.05);
}

TEST(PathGeometry, SamplingFromTheSecondPointLeavesTheCallersPathAlone) {
  Path path = straightRun(100, 100, 6, 20);
  std::vector<PathSegment> a;
  std::vector<PathSegment> b;
  std::vector<std::pair<std::size_t, bool>> bounds;
  Path copy = path;
  const std::vector<Point> full = samplePath(PRIMITIVES, copy, copy.front(), a);
  const std::vector<Point> second =
      samplePathFromSecond(PRIMITIVES, path, path.front(), b, &bounds);
  // The first point is the start the rendering is measured from, so both
  // forms render the same polyline; this one does not touch the path it is
  // given, which is what a caller that reads the path afterwards needs.
  EXPECT_EQ(path.size(), 21U);
  EXPECT_NEAR(polylineLength(full), polylineLength(second), 1e-9);
  ASSERT_FALSE(bounds.empty());
  EXPECT_EQ(bounds.size(), b.size());
  EXPECT_TRUE(bounds.front().second);
}

TEST(PathGeometry, BendsAreDirectionChanges) {
  EXPECT_EQ(countBends(straightRun(10, 10, 6, 20)), 0U);
  Path path = straightRun(10, 10, 6, 10);
  const Path second = straightRun(21, 10, 4, 10);
  path.insert(path.end(), second.begin(), second.end());
  EXPECT_EQ(countBends(path), 1U);
  EXPECT_EQ(countBends(Path{}), 0U);
}

TEST(PathGeometry, ARepeatedCellRendersNothing) {
  Path path = straightRun(100, 100, 6, 5);
  // The state re-emission of a heading change: the same cell twice.
  path.push_back({.x = 105, .y = 100, .heading = 7, .primitive = PRIMITIVES.straight(7)});
  const Path after = straightRun(106, 99, 7, 5);
  path.insert(path.end(), after.begin(), after.end());
  std::vector<PathSegment> segments;
  const std::vector<Point> samples = samplePath(PRIMITIVES, path, path.front(), segments);
  EXPECT_NEAR(samples.back().x(), static_cast<double>(path.back().x), 1e-9);
  EXPECT_NEAR(samples.back().y(), static_cast<double>(path.back().y), 1e-9);
  EXPECT_NEAR(polylineLength(samples), 5.0 + (6.0 * std::numbers::sqrt2), 0.01);
}

} // namespace
