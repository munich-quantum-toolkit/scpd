/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <numbers>
#include <type_traits>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::geometry;

/// Pack a native object into a buffer and unpack it again.
template <typename NativeT> NativeT roundTrip(const NativeT& object) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(NativeT::TableType::Pack(builder, &object));

  flatbuffers::Verifier verifier(builder.GetBufferPointer(), builder.GetSize());
  EXPECT_TRUE(verifier.VerifyBuffer<typename NativeT::TableType>(nullptr));

  NativeT result;
  flatbuffers::GetRoot<typename NativeT::TableType>(builder.GetBufferPointer())
      ->UnPackTo(&result);
  return result;
}

TEST(GeometrySchema, PointIsAnInlineStruct) {
  // The layout-space point is stored inline, without indirection.
  static_assert(std::is_trivially_copyable_v<Point>);
  static_assert(sizeof(Point) == 2 * sizeof(double));

  const Point point(1.5, -2.5);
  EXPECT_DOUBLE_EQ(point.x(), 1.5);
  EXPECT_DOUBLE_EQ(point.y(), -2.5);
}

TEST(GeometrySchema, PolygonRoundTripsThroughBuffer) {
  PolygonT polygon;
  polygon.vertices = {Point(0.0, 0.0), Point(100.0, 0.0), Point(100.0, 50.0),
                      Point(0.0, 50.0)};

  const PolygonT back = roundTrip(polygon);
  EXPECT_EQ(back, polygon);
  ASSERT_EQ(back.vertices.size(), 4U);
  EXPECT_EQ(back.vertices[2], Point(100.0, 50.0));
}

TEST(GeometrySchema, PathKeepsLineAndArcSegmentsAnalytic) {
  LineT line;
  line.start = Point(0.0, 0.0);
  line.end = Point(100.0, 0.0);

  ArcT arc;
  arc.centre = Point(100.0, 50.0);
  arc.radius = 50.0;
  arc.start_angle = -std::numbers::pi / 2;
  arc.sweep = std::numbers::pi / 2;

  PathT path;
  path.segments.push_back(std::make_unique<SegmentT>());
  path.segments.back()->shape.Set(line);
  path.segments.push_back(std::make_unique<SegmentT>());
  path.segments.back()->shape.Set(arc);

  const PathT back = roundTrip(path);
  EXPECT_EQ(back, path);
  ASSERT_EQ(back.segments.size(), 2U);

  EXPECT_EQ(back.segments[0]->shape.type, SegmentShape::Line);
  const auto* backLine = back.segments[0]->shape.AsLine();
  ASSERT_NE(backLine, nullptr);
  EXPECT_EQ(backLine->end, Point(100.0, 0.0));

  EXPECT_EQ(back.segments[1]->shape.type, SegmentShape::Arc);
  const auto* backArc = back.segments[1]->shape.AsArc();
  ASSERT_NE(backArc, nullptr);
  EXPECT_EQ(backArc->centre, Point(100.0, 50.0));
  EXPECT_DOUBLE_EQ(backArc->radius, 50.0);
  EXPECT_DOUBLE_EQ(backArc->sweep, std::numbers::pi / 2);
}

/// Build a Line table the way a foreign writer could, with or without start.
std::vector<std::uint8_t> lineBuffer(const bool withStart) {
  flatbuffers::FlatBufferBuilder builder;
  const Point start(0.0, 0.0);
  const Point end(100.0, 0.0);
  const auto table = builder.StartTable();
  if (withStart) {
    builder.AddStruct(Line::VT_START, &start);
  }
  builder.AddStruct(Line::VT_END, &end);
  builder.Finish(flatbuffers::Offset<Line>(builder.EndTable(table)));
  const auto* begin = builder.GetBufferPointer();
  return {begin, std::next(begin, builder.GetSize())};
}

TEST(GeometrySchema, VerifierRejectsALineWithoutItsStart) {
  // The model holds the endpoints by value, so a buffer that omits one must
  // fail verification rather than unpack to the origin.
  const auto complete = lineBuffer(true);
  flatbuffers::Verifier completeVerifier(complete.data(), complete.size());
  EXPECT_TRUE(completeVerifier.VerifyBuffer<Line>(nullptr));

  const auto incomplete = lineBuffer(false);
  flatbuffers::Verifier incompleteVerifier(incomplete.data(),
                                           incomplete.size());
  EXPECT_FALSE(incompleteVerifier.VerifyBuffer<Line>(nullptr));
}

TEST(GeometrySchema, SegmentShapesAreLineAndArcOnly) {
  EXPECT_STREQ(EnumNameSegmentShape(SegmentShape::Line), "Line");
  EXPECT_STREQ(EnumNameSegmentShape(SegmentShape::Arc), "Arc");
  EXPECT_EQ(static_cast<std::size_t>(SegmentShape::MAX), 2U);
}

} // namespace
