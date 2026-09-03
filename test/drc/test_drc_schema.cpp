/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::drc;
using mqt::scpd::flatbuffers::geometry::Point;

TEST(DrcSchema, EightRulesInTableOrder) {
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::WireClearance), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::FeedlineOrthogonality), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::WireLoop), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::ObstacleClearance), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::ComponentOverlap), 4U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::MinStraightLength), 5U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::ResonatorLength), 6U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::MinBendRadius), 7U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::MAX), 7U);
  EXPECT_STREQ(EnumNameDrcRule(DrcRule::WireLoop), "WireLoop");
}

TEST(DrcSchema, VerifierRejectsAFindingWithoutItsLocation) {
  for (const bool withLocation : {true, false}) {
    flatbuffers::FlatBufferBuilder builder;
    const auto wires = builder.CreateVector(std::vector<std::uint32_t>{3});
    const Point location(1.0, 2.0);
    const auto table = builder.StartTable();
    builder.AddOffset(DrcFinding::VT_WIRES, wires);
    if (withLocation) {
      builder.AddStruct(DrcFinding::VT_LOCATION, &location);
    }
    builder.Finish(flatbuffers::Offset<DrcFinding>(builder.EndTable(table)));

    flatbuffers::Verifier verifier(builder.GetBufferPointer(),
                                   builder.GetSize());
    EXPECT_EQ(verifier.VerifyBuffer<DrcFinding>(nullptr), withLocation);
  }
}

TEST(DrcSchema, ReportRoundTripsFindings) {
  DrcReportT report;
  report.stage = DrcStage::Finalize;
  report.feedlines_skipped = 12;

  auto clearance = std::make_unique<DrcFindingT>();
  clearance->rule = DrcRule::WireClearance;
  clearance->severity = DrcSeverity::Active;
  clearance->wires = {3, 8};
  clearance->location = Point(1234.5, 678.9);
  clearance->measured = 1.0;
  clearance->limit = 185.0;
  clearance->clearance_kind = ClearanceKind::Short;
  clearance->message = "wires 3 and 8 touch";
  report.findings.push_back(std::move(clearance));

  auto radius = std::make_unique<DrcFindingT>();
  radius->rule = DrcRule::MinBendRadius;
  radius->severity = DrcSeverity::Advisory;
  radius->wires = {5};
  radius->measured = 40.0;
  radius->limit = 50.0;
  report.findings.push_back(std::move(radius));

  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(DrcReport::Pack(builder, &report));
  flatbuffers::Verifier verifier(builder.GetBufferPointer(), builder.GetSize());
  ASSERT_TRUE(VerifyDrcReportBuffer(verifier));

  DrcReportT back;
  GetDrcReport(builder.GetBufferPointer())->UnPackTo(&back);
  EXPECT_EQ(back, report);
  ASSERT_EQ(back.findings.size(), 2U);
  EXPECT_EQ(back.findings[0]->clearance_kind, ClearanceKind::Short);
  EXPECT_EQ(back.findings[0]->wires, (std::vector<std::uint32_t>{3, 8}));
  EXPECT_EQ(back.findings[1]->severity, DrcSeverity::Advisory);
  EXPECT_EQ(back.stage, DrcStage::Finalize);
  EXPECT_EQ(back.feedlines_skipped, 12U);
}

} // namespace
