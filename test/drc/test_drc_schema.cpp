/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/drc/Validation.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::drc;
using mqt::scpd::drc::validate;
using mqt::scpd::flatbuffers::design::ConnectionRef;
using mqt::scpd::flatbuffers::geometry::Point;

/// A report of the Final stage with one clearance finding and one advisory
/// finding, valid throughout.
std::unique_ptr<DrcReportT> makeReport(const DrcStage stage) {
  auto report = std::make_unique<DrcReportT>();
  report->stage = stage;
  report->feedlines_skipped = 12;

  auto clearance = std::make_unique<DrcFindingT>();
  clearance->rule = DrcRule::WireClearance;
  clearance->severity = DrcSeverity::Active;
  clearance->wires = {ConnectionRef(3), ConnectionRef(8)};
  clearance->location = Point(1234.5, 678.9);
  clearance->measured = 1.0;
  clearance->limit = 185.0;
  clearance->clearance_kind = ClearanceKind::Short;
  clearance->message = "wires 3 and 8 touch";
  report->findings.push_back(std::move(clearance));

  auto radius = std::make_unique<DrcFindingT>();
  radius->rule = DrcRule::MinBendRadius;
  radius->severity = DrcSeverity::Advisory;
  radius->wires = {ConnectionRef(5)};
  radius->location = Point(10.0, 20.0);
  radius->measured = 40.0;
  radius->limit = 50.0;
  report->findings.push_back(std::move(radius));
  return report;
}

TEST(DrcSchema, EightRulesInTableOrderAfterUnset) {
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::Unset), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::WireClearance), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::FeedlineOrthogonality), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::WireLoop), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::ObstacleClearance), 4U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::ComponentOverlap), 5U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::MinStraightLength), 6U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::ResonatorLength), 7U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::MinBendRadius), 8U);
  EXPECT_EQ(static_cast<std::uint8_t>(DrcRule::MAX), 8U);
  EXPECT_STREQ(EnumNameDrcRule(DrcRule::WireLoop), "WireLoop");
}

TEST(DrcSchema, VerifierRejectsAFindingWithoutItsLocation) {
  for (const bool withLocation : {true, false}) {
    flatbuffers::FlatBufferBuilder builder;
    const auto wires = builder.CreateVectorOfStructs(
        std::vector<ConnectionRef>{ConnectionRef(3)});
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

TEST(DrcSchema, ReportsRootHoldsOneReportPerStage) {
  // drc.json holds the reports of both checked stages behind one root.
  DrcReportsT reports;
  reports.reports.push_back(makeReport(DrcStage::Final));
  reports.reports.push_back(makeReport(DrcStage::Finalize));
  EXPECT_TRUE(validate(reports).empty());

  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(DrcReports::Pack(builder, &reports));
  flatbuffers::Verifier verifier(builder.GetBufferPointer(), builder.GetSize());
  ASSERT_TRUE(VerifyDrcReportsBuffer(verifier));

  DrcReportsT back;
  GetDrcReports(builder.GetBufferPointer())->UnPackTo(&back);
  EXPECT_EQ(back, reports);
  ASSERT_EQ(back.reports.size(), 2U);
  EXPECT_EQ(back.reports[0]->stage, DrcStage::Final);
  EXPECT_EQ(back.reports[1]->stage, DrcStage::Finalize);
  ASSERT_EQ(back.reports[0]->findings.size(), 2U);
  const auto& clearance = *back.reports[0]->findings[0];
  EXPECT_EQ(clearance.clearance_kind, ClearanceKind::Short);
  ASSERT_EQ(clearance.wires.size(), 2U);
  EXPECT_EQ(clearance.wires[1].index(), 8U);
  EXPECT_EQ(back.reports[0]->findings[1]->severity, DrcSeverity::Advisory);
  EXPECT_EQ(back.reports[0]->feedlines_skipped, 12U);
}

TEST(DrcSchema, ValidationRejectsUnsetFieldsAndMismatchedKinds) {
  DrcReportsT reports;
  reports.reports.push_back(makeReport(DrcStage::Unset));
  auto& findings = reports.reports[0]->findings;
  findings[0]->clearance_kind = ClearanceKind::Unset;
  findings[1]->rule = DrcRule::Unset;
  findings[1]->severity = DrcSeverity::Unset;
  findings[1]->wires.clear();
  findings[1]->limit = 0.0;
  findings[1]->clearance_kind = ClearanceKind::NearMiss;

  const std::string kindSet =
      std::string("report 0: finding 1 clearance kind is set for a rule ") +
      "other than clearance";
  EXPECT_EQ(validate(reports),
            (std::vector<std::string>{
                "report 0: stage is unset",
                "report 0: finding 0 clearance kind is unset",
                "report 0: finding 1 rule is unset",
                "report 0: finding 1 severity is unset",
                "report 0: finding 1 names no wire",
                "report 0: finding 1 limit must be positive", kindSet}));
}

} // namespace
