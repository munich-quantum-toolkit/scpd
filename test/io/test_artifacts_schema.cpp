/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/buffer.h>
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

using namespace mqt::scpd::flatbuffers::artifacts;
using namespace mqt::scpd::flatbuffers::design;
using namespace mqt::scpd::flatbuffers::geometry;

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

TEST(ArtifactSchema, EveryStageOutputRoundTripsThroughTheRoot) {
  // The outputs of the stages that are not implemented yet are empty tables.
  // Each still travels behind the Artifact root with its own tag.
  const ArtifactT capacity = deserialize(serialize(wrap(CapacityPlanT{})));
  EXPECT_EQ(capacity.output.type, StageOutput::CapacityPlan);
  EXPECT_NE(capacity.output.AsCapacityPlan(), nullptr);
  EXPECT_EQ(capacity.producer, "mqt-scpd test");

  const ArtifactT detail = deserialize(serialize(wrap(DetailRoutingT{})));
  EXPECT_EQ(detail.output.type, StageOutput::DetailRouting);
  EXPECT_NE(detail.output.AsDetailRouting(), nullptr);
  EXPECT_EQ(detail.output.AsCapacityPlan(), nullptr);
}

TEST(ArtifactSchema, AssignmentRoundTrips) {
  AssignmentT assignment;
  assignment.connections.push_back(std::make_unique<ConnectionT>());
  assignment.connections.back()->target = PortRef(3);
  assignment.connections.back()->source_role = AssignedRole::ResonatorSource;
  assignment.connections.back()->target_role = AssignedRole::ResonatorTarget;
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
  EXPECT_NE(back.output.AsGlobalRouting(), nullptr);
}

TEST(ArtifactSchema, FinalRoutingKeepsComponentsAndFailures) {
  FinalRoutingT final;
  final.couplers.push_back(std::make_unique<CpwCouplerT>());
  final.couplers.back()->port = PortRef(9);
  final.couplers.back()->rotation = Rotation::R180;
  final.bridges.push_back(std::make_unique<BridgeT>());
  final.bridges.back()->center = Point(50.0, 60.0);
  final.unresolved = {7};

  const ArtifactT back = deserialize(serialize(wrap(final)));
  ASSERT_NE(back.output.AsFinalRouting(), nullptr);
  EXPECT_EQ(*back.output.AsFinalRouting(), final);
  EXPECT_EQ(back.output.AsFinalRouting()->couplers[0]->rotation,
            Rotation::R180);
  EXPECT_EQ(back.output.AsFinalRouting()->unresolved,
            std::vector<std::uint32_t>{7});
}

TEST(ArtifactSchema, GeometryKeepsAnalyticSegments) {
  GeometryT geometry;
  geometry.wires.push_back(std::make_unique<WireT>());
  geometry.wires.back()->connection = 0;
  geometry.wires.back()->path = std::make_unique<PathT>();
  geometry.wires.back()->path->segments.push_back(std::make_unique<SegmentT>());
  ArcT arc;
  arc.center = Point(0.0, 50.0);
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

TEST(ArtifactSchema, VerifierRejectsAnArtifactWithoutItsProducer) {
  // Provenance is part of the contract: an artifact that does not record the
  // version that wrote it is not a valid artifact.
  for (const bool withProducer : {true, false}) {
    flatbuffers::FlatBufferBuilder builder;
    const auto producer = builder.CreateString("mqt-scpd test");
    const auto output = CreateGlobalRouting(builder);
    const auto table = builder.StartTable();
    if (withProducer) {
      builder.AddOffset(Artifact::VT_PRODUCER, producer);
    }
    builder.AddElement<std::uint8_t>(
        Artifact::VT_OUTPUT_TYPE,
        static_cast<std::uint8_t>(StageOutput::GlobalRouting), 0);
    builder.AddOffset(Artifact::VT_OUTPUT, output);
    builder.Finish(flatbuffers::Offset<Artifact>(builder.EndTable(table)),
                   ArtifactIdentifier());

    flatbuffers::Verifier verifier(builder.GetBufferPointer(),
                                   builder.GetSize());
    EXPECT_EQ(VerifyArtifactBuffer(verifier), withProducer);
  }
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
