/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/generated/artifacts_generated.hpp"
#include "mqt-scpd/generated/design_generated.hpp"
#include "mqt-scpd/generated/geometry_generated.hpp"

#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::generated;

/// Serialize an artifact as a run directory would store it.
std::vector<std::uint8_t> serialize(const ArtifactT& artifact) {
  flatbuffers::FlatBufferBuilder builder;
  FinishArtifactBuffer(builder, Artifact::Pack(builder, &artifact));
  const auto* begin = builder.GetBufferPointer();
  return {begin, std::next(begin, builder.GetSize())};
}

/// Verify and unpack a stored artifact.
ArtifactT deserialize(const std::vector<std::uint8_t>& bytes) {
  flatbuffers::Verifier verifier(bytes.data(), bytes.size());
  EXPECT_TRUE(VerifyArtifactBuffer(verifier));
  EXPECT_TRUE(ArtifactBufferHasIdentifier(bytes.data()));

  ArtifactT result;
  GetArtifact(bytes.data())->UnPackTo(&result);
  return result;
}

template <typename OutputT> ArtifactT wrap(OutputT output) {
  ArtifactT artifact;
  artifact.producer = "mqt-scpd test";
  artifact.output.Set(std::move(output));
  return artifact;
}

TEST(ArtifactSchema, IdentifierRecordsTheSchemaVersion) {
  EXPECT_STREQ(ArtifactIdentifier(), "SCP1");
  EXPECT_STREQ(ArtifactExtension(), "fb");
}

TEST(ArtifactSchema, EveryStageOutputIsAnArtifact) {
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::CapacityPlan), "CapacityPlan");
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::Assignment), "Assignment");
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::GlobalRouting),
               "GlobalRouting");
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::DetailRouting),
               "DetailRouting");
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::FinalRouting), "FinalRouting");
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::Geometry), "Geometry");
  EXPECT_EQ(static_cast<std::uint8_t>(StageOutput::MAX), 6U);
}

TEST(ArtifactSchema, CapacityPlanRoundTrips) {
  CapacityPlanT plan;
  plan.partitions.push_back(std::make_unique<PartitionT>());
  plan.partitions.back()->id = 0;
  plan.partitions.back()->cells = {GCoord(0, 0), GCoord(1, 0)};
  plan.partitions.push_back(std::make_unique<PartitionT>());
  plan.partitions.back()->id = 1;
  plan.partitions.back()->cells = {GCoord(2, 0)};
  plan.borders.push_back(std::make_unique<BorderT>());
  plan.borders.back()->first = 0;
  plan.borders.back()->second = 1;
  plan.borders.back()->budget = 3;
  plan.borders.back()->bottleneck = true;
  plan.chains.push_back(std::make_unique<CapacityChainT>());
  plan.chains.back()->ports = {PortRef(4), PortRef(5)};
  plan.chains.back()->partitions = {0, 1};

  const ArtifactT back = deserialize(serialize(wrap(plan)));
  EXPECT_EQ(back.output.type, StageOutput::CapacityPlan);
  ASSERT_NE(back.output.AsCapacityPlan(), nullptr);
  EXPECT_EQ(*back.output.AsCapacityPlan(), plan);
  EXPECT_TRUE(back.output.AsCapacityPlan()->borders[0]->bottleneck);
}

TEST(ArtifactSchema, AssignmentRoundTrips) {
  AssignmentT assignment;
  assignment.connections.push_back(std::make_unique<ConnectionT>());
  assignment.connections.back()->target = PortRef(3);
  assignment.connections.back()->source_role = AssignedRole::ResonatorSource;
  assignment.connections.back()->target_role = AssignedRole::ResonatorTarget;
  assignment.feedlines.push_back(std::make_unique<FeedlineT>());
  assignment.feedlines.back()->source = PortRef(0);
  assignment.feedlines.back()->target = PortRef(1);
  assignment.feedlines.back()->resonators = {PortRef(3)};
  assignment.objective = 132.68;

  const ArtifactT back = deserialize(serialize(wrap(assignment)));
  ASSERT_NE(back.output.AsAssignment(), nullptr);
  EXPECT_EQ(*back.output.AsAssignment(), assignment);
  EXPECT_DOUBLE_EQ(back.output.AsAssignment()->objective, 132.68);
}

TEST(ArtifactSchema, EmptyGlobalRoutingIsAValidArtifact) {
  // A chip with no inner circuit writes 03-global.fb empty, not skipped.
  const GlobalRoutingT routing;

  const ArtifactT back = deserialize(serialize(wrap(routing)));
  EXPECT_EQ(back.output.type, StageOutput::GlobalRouting);
  ASSERT_NE(back.output.AsGlobalRouting(), nullptr);
  EXPECT_TRUE(back.output.AsGlobalRouting()->routes.empty());
}

TEST(ArtifactSchema, DetailAndFinalRoutingKeepGridCoordinates) {
  DetailRoutingT detail;
  detail.wires.push_back(std::make_unique<PixelPathT>());
  detail.wires.back()->connection = 2;
  detail.wires.back()->pixels = {DCoord(10, 10), DCoord(11, 10)};

  const ArtifactT backDetail = deserialize(serialize(wrap(detail)));
  ASSERT_NE(backDetail.output.AsDetailRouting(), nullptr);
  EXPECT_EQ(*backDetail.output.AsDetailRouting(), detail);

  FinalRoutingT final;
  final.wires.push_back(std::make_unique<CellPathT>());
  final.wires.back()->connection = 2;
  final.wires.back()->states = {RCoord(10, 10, 0), RCoord(11, 11, 1)};
  final.couplers.push_back(std::make_unique<CpwCouplerT>());
  final.couplers.back()->port = PortRef(9);
  final.couplers.back()->rotation = Rotation::R180;
  final.bridges.push_back(std::make_unique<BridgeT>());
  final.bridges.back()->centre = Point(50.0, 60.0);
  final.unresolved = {7};

  const ArtifactT backFinal = deserialize(serialize(wrap(final)));
  ASSERT_NE(backFinal.output.AsFinalRouting(), nullptr);
  EXPECT_EQ(*backFinal.output.AsFinalRouting(), final);
  EXPECT_EQ(backFinal.output.AsFinalRouting()->wires[0]->states[1].heading(),
            1U);
  EXPECT_EQ(backFinal.output.AsFinalRouting()->unresolved,
            std::vector<std::uint32_t>{7});
}

TEST(ArtifactSchema, GeometryKeepsAnalyticSegments) {
  GeometryT geometry;
  geometry.wires.push_back(std::make_unique<WireT>());
  geometry.wires.back()->connection = 0;
  geometry.wires.back()->path = std::make_unique<PathT>();
  geometry.wires.back()->path->segments.push_back(std::make_unique<SegmentT>());
  ArcT arc;
  arc.centre = Point(0.0, 50.0);
  arc.radius = 50.0;
  arc.sweep = std::numbers::pi;
  geometry.wires.back()->path->segments.back()->shape.Set(arc);

  const ArtifactT back = deserialize(serialize(wrap(geometry)));
  ASSERT_NE(back.output.AsGeometry(), nullptr);
  EXPECT_EQ(*back.output.AsGeometry(), geometry);
  const auto* backArc =
      back.output.AsGeometry()->wires[0]->path->segments[0]->shape.AsArc();
  ASSERT_NE(backArc, nullptr);
  EXPECT_DOUBLE_EQ(backArc->radius, 50.0);
}

TEST(ArtifactSchema, VerifierRejectsForeignAndTruncatedBuffers) {
  std::vector<std::uint8_t> bytes = serialize(wrap(GlobalRoutingT{}));

  // The identifier sits at byte offset 4, after the root offset.
  std::vector<std::uint8_t> foreign = bytes;
  constexpr std::array<std::uint8_t, 4> foreignIdentifier{'X', 'X', 'X', 'X'};
  std::ranges::copy(foreignIdentifier, std::next(foreign.begin(), 4));
  flatbuffers::Verifier foreignVerifier(foreign.data(), foreign.size());
  EXPECT_FALSE(ArtifactBufferHasIdentifier(foreign.data()));
  EXPECT_FALSE(VerifyArtifactBuffer(foreignVerifier));

  bytes.resize(bytes.size() / 2);
  flatbuffers::Verifier truncatedVerifier(bytes.data(), bytes.size());
  EXPECT_FALSE(VerifyArtifactBuffer(truncatedVerifier));
}

} // namespace
