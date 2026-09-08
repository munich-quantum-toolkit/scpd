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
#include "mqt-scpd/io/Artifacts.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::artifacts;
using namespace mqt::scpd::flatbuffers::design;
using namespace mqt::scpd::flatbuffers::geometry;
using mqt::scpd::io::readArtifact;
using mqt::scpd::io::validate;
using mqt::scpd::io::writeArtifact;

template <typename OutputT> ArtifactT wrap(OutputT output) {
  ArtifactT artifact;
  artifact.producer = "mqt-scpd test";
  artifact.output.Set(std::move(output));
  return artifact;
}

/// Pack an artifact without the checks of writeArtifact, as a foreign or a
/// defective writer would.
std::vector<std::uint8_t> packUnchecked(const ArtifactT& artifact) {
  flatbuffers::FlatBufferBuilder builder;
  FinishArtifactBuffer(builder, Artifact::Pack(builder, &artifact));
  const auto* begin = builder.GetBufferPointer();
  return {begin, std::next(begin, builder.GetSize())};
}

/// A coupler that completes the given connection with the port it creates.
std::unique_ptr<CpwCouplerT> makeCoupler(const std::uint32_t connection) {
  auto coupler = std::make_unique<CpwCouplerT>();
  coupler->connection = ConnectionRef(connection);
  coupler->port = std::make_unique<PortT>();
  coupler->port->label = "Coupler" + std::to_string(connection) + ".port0";
  coupler->port->center = Point(1200.0, 800.0);
  coupler->port->orientation = 90.0;
  coupler->port->role = UnassignedRole::Coupler;
  coupler->center = Point(1200.0, 800.0);
  coupler->rotation = Rotation::R90;
  coupler->length = 200.0;
  coupler->height = 26.0;
  return coupler;
}

std::unique_ptr<BridgeT> makeBridge() {
  auto bridge = std::make_unique<BridgeT>();
  bridge->center = Point(50.0, 60.0);
  bridge->rotation = Rotation::R45;
  bridge->width = 60.0;
  bridge->height = 60.0;
  return bridge;
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
  const ArtifactT capacity = readArtifact(writeArtifact(wrap(CapacityPlanT{})));
  EXPECT_EQ(capacity.output.type, StageOutput::CapacityPlan);
  EXPECT_NE(capacity.output.AsCapacityPlan(), nullptr);
  EXPECT_EQ(capacity.producer, "mqt-scpd test");

  const ArtifactT global = readArtifact(writeArtifact(wrap(GlobalRoutingT{})));
  EXPECT_EQ(global.output.type, StageOutput::GlobalRouting);
  EXPECT_NE(global.output.AsGlobalRouting(), nullptr);

  const ArtifactT detail = readArtifact(writeArtifact(wrap(DetailRoutingT{})));
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

  const ArtifactT back = readArtifact(writeArtifact(wrap(assignment)));
  ASSERT_NE(back.output.AsAssignment(), nullptr);
  EXPECT_EQ(*back.output.AsAssignment(), assignment);
  EXPECT_DOUBLE_EQ(back.output.AsAssignment()->objective, 132.68);
}

TEST(ArtifactSchema, FinalRoutingKeepsTheCreatedPortsAndTheFailures) {
  // The coupler carries the port it creates and the connection it completes,
  // so a reloaded run can rebuild the grown port list without the stage.
  FinalRoutingT final;
  final.couplers.push_back(makeCoupler(3));
  final.bridges.push_back(makeBridge());
  final.unresolved = {ConnectionRef(7)};

  const ArtifactT back = readArtifact(writeArtifact(wrap(final)));
  ASSERT_NE(back.output.AsFinalRouting(), nullptr);
  const auto& routing = *back.output.AsFinalRouting();
  EXPECT_EQ(routing, final);
  ASSERT_EQ(routing.couplers.size(), 1U);
  EXPECT_EQ(routing.couplers[0]->connection.index(), 3U);
  ASSERT_NE(routing.couplers[0]->port, nullptr);
  EXPECT_EQ(routing.couplers[0]->port->role, UnassignedRole::Coupler);
  EXPECT_EQ(routing.couplers[0]->port->label, "Coupler3.port0");
  ASSERT_EQ(routing.unresolved.size(), 1U);
  EXPECT_EQ(routing.unresolved[0].index(), 7U);
}

TEST(ArtifactSchema, GeometryKeepsAnalyticSegments) {
  GeometryT geometry;
  geometry.wires.push_back(std::make_unique<WireT>());
  geometry.wires.back()->connection = ConnectionRef(0);
  geometry.wires.back()->path = std::make_unique<PathT>();
  geometry.wires.back()->path->segments.push_back(std::make_unique<SegmentT>());
  ArcT arc;
  arc.center = Point(0.0, 50.0);
  arc.radius = 50.0;
  arc.sweep = std::numbers::pi;
  geometry.wires.back()->path->segments.back()->shape.Set(arc);
  geometry.couplers.push_back(makeCoupler(0));
  geometry.bridges.push_back(makeBridge());

  const ArtifactT back = readArtifact(writeArtifact(wrap(geometry)));
  ASSERT_NE(back.output.AsGeometry(), nullptr);
  EXPECT_EQ(*back.output.AsGeometry(), geometry);
  const auto* backArc =
      back.output.AsGeometry()->wires[0]->path->segments[0]->shape.AsArc();
  ASSERT_NE(backArc, nullptr);
  EXPECT_DOUBLE_EQ(backArc->radius, 50.0);
}

TEST(ArtifactSchema, WriteArtifactRejectsAnIncompleteArtifact) {
  ArtifactT artifact;
  EXPECT_EQ(validate(artifact), (std::vector<std::string>{
                                    "producer is empty", "output is missing"}));
  EXPECT_THROW(static_cast<void>(writeArtifact(artifact)),
               std::invalid_argument);

  artifact.producer = "mqt-scpd test";
  EXPECT_THROW(static_cast<void>(writeArtifact(artifact)),
               std::invalid_argument);

  artifact.output.Set(GlobalRoutingT{});
  EXPECT_TRUE(validate(artifact).empty());
  EXPECT_NO_THROW(static_cast<void>(writeArtifact(artifact)));
}

TEST(ArtifactSchema, ReadArtifactRejectsAnUnsetRole) {
  // A scalar field always reads as a value, so an absent role decodes as
  // Unset. The verifier accepts the buffer; the semantic check rejects it.
  AssignmentT assignment;
  assignment.connections.push_back(std::make_unique<ConnectionT>());
  assignment.connections.back()->target = PortRef(1);
  const auto bytes = packUnchecked(wrap(assignment));

  EXPECT_THROW(static_cast<void>(readArtifact(bytes)), std::invalid_argument);

  ArtifactT unpacked;
  GetArtifact(bytes.data())->UnPackTo(&unpacked);
  EXPECT_EQ(validate(unpacked),
            (std::vector<std::string>{"connection 0: source role is unset",
                                      "connection 0: target role is unset"}));
}

TEST(ArtifactSchema, WriteArtifactRejectsIncompleteComponents) {
  FinalRoutingT final;
  final.couplers.push_back(makeCoupler(0));
  final.couplers.back()->port.reset();
  final.bridges.push_back(makeBridge());
  final.bridges.back()->width = 0.0;

  const ArtifactT artifact = wrap(std::move(final));
  EXPECT_EQ(validate(artifact),
            (std::vector<std::string>{"coupler 0: port is missing",
                                      "bridge 0: width must be positive"}));
  EXPECT_THROW(static_cast<void>(writeArtifact(artifact)),
               std::invalid_argument);
}

TEST(ArtifactSchema, ReadArtifactRejectsAnArtifactWithoutItsProducer) {
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
    const auto* begin = builder.GetBufferPointer();
    const std::vector<std::uint8_t> bytes(begin,
                                          std::next(begin, builder.GetSize()));

    if (withProducer) {
      EXPECT_EQ(readArtifact(bytes).producer, "mqt-scpd test");
    } else {
      EXPECT_THROW(static_cast<void>(readArtifact(bytes)),
                   std::invalid_argument);
    }
  }
}

TEST(ArtifactSchema, ReadArtifactRejectsForeignAndTruncatedBuffers) {
  std::vector<std::uint8_t> bytes = writeArtifact(wrap(GlobalRoutingT{}));

  // The identifier sits at byte offset 4, after the root offset.
  std::vector<std::uint8_t> foreign = bytes;
  constexpr std::array<std::uint8_t, 4> foreignIdentifier{'X', 'X', 'X', 'X'};
  std::ranges::copy(foreignIdentifier, std::next(foreign.begin(), 4));
  EXPECT_FALSE(ArtifactBufferHasIdentifier(foreign.data()));
  EXPECT_THROW(static_cast<void>(readArtifact(foreign)), std::invalid_argument);

  bytes.resize(bytes.size() / 2);
  EXPECT_THROW(static_cast<void>(readArtifact(bytes)), std::invalid_argument);
}

} // namespace
