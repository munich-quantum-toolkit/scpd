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
  EXPECT_STREQ(EnumNameStageOutput(StageOutput::CorridorRouting),
               "CorridorRouting");
  EXPECT_EQ(static_cast<std::uint8_t>(StageOutput::MAX), 7U);
}

/// A grid that carries every field the schema requires of one.
std::unique_ptr<GridExtentT> emptyGrid() {
  auto grid = std::make_unique<GridExtentT>();
  grid->origin = Point(0.0, 0.0);
  return grid;
}

/// A capacity plan that carries nothing, but carries every field the schema
/// requires. A stage that produced nothing still writes a complete artifact.
CapacityPlanT emptyPlan() {
  CapacityPlanT plan;
  plan.capacity_grid = std::make_unique<GridExtentT>();
  plan.detail_grid = std::make_unique<GridExtentT>();
  return plan;
}

TEST(ArtifactSchema, EveryStageOutputRoundTripsThroughTheRoot) {
  // The outputs of the stages that are not implemented yet are empty tables.
  // Each still travels behind the Artifact root with its own tag.
  const ArtifactT capacity = readArtifact(writeArtifact(wrap(emptyPlan())));
  EXPECT_EQ(capacity.output.type, StageOutput::CapacityPlan);
  EXPECT_NE(capacity.output.AsCapacityPlan(), nullptr);
  EXPECT_EQ(capacity.producer, "mqt-scpd test");

  const ArtifactT global = readArtifact(writeArtifact(wrap(GlobalRoutingT{})));
  EXPECT_EQ(global.output.type, StageOutput::GlobalRouting);
  EXPECT_NE(global.output.AsGlobalRouting(), nullptr);

  DetailRoutingT drawn;
  drawn.grid = emptyGrid();
  const ArtifactT detail = readArtifact(writeArtifact(wrap(std::move(drawn))));
  EXPECT_EQ(detail.output.type, StageOutput::DetailRouting);
  EXPECT_NE(detail.output.AsDetailRouting(), nullptr);
  EXPECT_EQ(detail.output.AsCapacityPlan(), nullptr);
}

/// The two routing outputs count their cells on a grid, so an artifact
/// without one is refused before it is written rather than failing the
/// verifier of every reader afterwards.
TEST(ArtifactSchema, WriteArtifactRejectsARoutingWithoutItsGrid) {
  EXPECT_EQ(validate(wrap(DetailRoutingT{})),
            (std::vector<std::string>{"grid is missing"}));
  EXPECT_THROW(static_cast<void>(writeArtifact(wrap(DetailRoutingT{}))),
               std::invalid_argument);
  EXPECT_EQ(validate(wrap(FinalRoutingT{})),
            (std::vector<std::string>{"grid is missing"}));
  EXPECT_THROW(static_cast<void>(writeArtifact(wrap(FinalRoutingT{}))),
               std::invalid_argument);
}

TEST(ArtifactSchema, CorridorRoutingRoundTrips) {
  CorridorRoutingT routing;
  auto corridor = std::make_unique<CorridorT>();
  corridor->partitions = {4, 7, 9};
  corridor->crossings.emplace_back(10.5, 20.25);
  corridor->crossings.emplace_back(30.75, 40.0);
  corridor->source = std::make_unique<Point>(0.5, 1.5);
  corridor->target = std::make_unique<Point>(50.0, 60.0);
  routing.corridors.push_back(std::move(corridor));
  // A connection that found no way carries no partitions, which is how a
  // reader counts the failures.
  routing.corridors.push_back(std::make_unique<CorridorT>());
  auto slots = std::make_unique<BorderSlotsT>();
  slots->border = 2;
  slots->positions.emplace_back(10.5, 20.25);
  routing.slots.push_back(std::move(slots));

  const ArtifactT back = readArtifact(writeArtifact(wrap(std::move(routing))));

  ASSERT_EQ(back.output.type, StageOutput::CorridorRouting);
  const auto& read = *back.output.AsCorridorRouting();
  ASSERT_EQ(read.corridors.size(), 2U);
  EXPECT_EQ(read.corridors[0]->partitions,
            (std::vector<std::uint32_t>{4, 7, 9}));
  EXPECT_EQ(read.corridors[0]->crossings.size(), 2U);
  EXPECT_EQ(read.corridors[0]->source->x(), 0.5);
  EXPECT_TRUE(read.corridors[1]->partitions.empty());
  ASSERT_EQ(read.slots.size(), 1U);
  EXPECT_EQ(read.slots[0]->border, 2U);
}

TEST(ArtifactSchema, RefusesACorridorThatNamesTheWrongNumberOfPartitions) {
  // A crossing carries a wire from one partition into the next, so a corridor
  // names one more partition than it has crossings.
  CorridorRoutingT routing;
  auto corridor = std::make_unique<CorridorT>();
  corridor->partitions = {4, 7};
  corridor->source = std::make_unique<Point>(0.0, 0.0);
  corridor->target = std::make_unique<Point>(1.0, 0.0);
  routing.corridors.push_back(std::move(corridor));

  const auto problems = validate(wrap(std::move(routing)));

  ASSERT_EQ(problems.size(), 1U);
  EXPECT_EQ(problems[0], "corridor 0 names 2 partitions and 0 crossings");
}

TEST(ArtifactSchema, RefusesARoutedCorridorWithoutItsEnds) {
  // A way runs from the feed to the target, so a corridor with partitions
  // has both ends.
  CorridorRoutingT routing;
  auto corridor = std::make_unique<CorridorT>();
  corridor->partitions = {4};
  routing.corridors.push_back(std::move(corridor));
  corridor = std::make_unique<CorridorT>();
  corridor->partitions = {4};
  corridor->source = std::make_unique<Point>(0.0, 0.0);
  routing.corridors.push_back(std::move(corridor));

  EXPECT_EQ(validate(wrap(std::move(routing))),
            (std::vector<std::string>{"corridor 0 has a way but no source",
                                      "corridor 0 has a way but no target",
                                      "corridor 1 has a way but no target"}));
}

/// A connection of the assignment to a resonator, with both of its roles.
std::unique_ptr<ConnectionT> resonatorConnection() {
  auto connection = std::make_unique<ConnectionT>();
  connection->source_role = AssignedRole::ResonatorSource;
  connection->target_role = AssignedRole::ResonatorTarget;
  return connection;
}

TEST(ArtifactSchema, RefusesCrossingsOfACorridorWithoutPartitions) {
  // A corridor without partitions is a connection without a way, so it
  // crosses no border either.
  CorridorRoutingT routing;
  auto corridor = std::make_unique<CorridorT>();
  corridor->crossings.emplace_back(1.0, 2.0);
  routing.corridors.push_back(std::move(corridor));

  const auto problems = validate(wrap(std::move(routing)));

  ASSERT_EQ(problems.size(), 1U);
  EXPECT_EQ(problems[0], "corridor 0 names 0 partitions and 1 crossing");
}

TEST(ArtifactSchema, RefusesAnAssignmentWhoseListsDoNotFollowTheRing) {
  // The connections, the launchers and the feeds hold one entry per ring
  // node, and the Corridor stage reads them by the index of the node.
  AssignmentT assignment;
  assignment.ring = {PortRef(1), PortRef(2)};
  assignment.connections.push_back(resonatorConnection());
  assignment.connections.back()->target = PortRef(1);
  assignment.launchers = {PortRef(0), PortRef(3)};
  assignment.feeds.emplace_back(0.0, 0.0);
  assignment.feeds.emplace_back(1.0, 0.0);
  assignment.feeds.emplace_back(2.0, 0.0);

  EXPECT_EQ(validate(wrap(std::move(assignment))),
            (std::vector<std::string>{"1 connection for 2 ring nodes",
                                      "3 feeds for 2 ring nodes"}));
}

TEST(ArtifactSchema, RefusesAFeedlineChainThatLeavesTheRing) {
  // A chain names ring nodes, so each of them has to be a node of the ring.
  AssignmentT assignment;
  assignment.connections.push_back(resonatorConnection());
  assignment.connections.back()->target = PortRef(3);
  assignment.ring.emplace_back(3);
  assignment.launchers.emplace_back(9);
  assignment.feeds.emplace_back(0.0, 0.0);
  auto chain = std::make_unique<FeedlineChainT>();
  chain->nodes = {0, 1};
  assignment.chains.push_back(std::move(chain));

  const auto problems = validate(wrap(std::move(assignment)));

  ASSERT_EQ(problems.size(), 1U);
  EXPECT_EQ(problems[0], "chain 0 names ring node 1 of 1");
}

TEST(ArtifactSchema,
     RefusesAnAssignmentWhoseChainsOrTargetsDisagreeWithTheRing) {
  // Connection i reaches ring node i, a chain runs through one resonator at
  // least, and one feedline drives each resonator.
  AssignmentT assignment;
  assignment.ring = {PortRef(1), PortRef(2), PortRef(5)};
  for (const std::uint32_t port : {1U, 4U, 5U}) {
    assignment.connections.push_back(resonatorConnection());
    assignment.connections.back()->target = PortRef(port);
  }
  assignment.launchers = {PortRef(0), PortRef(0), PortRef(3)};
  assignment.feeds = {Point(0.0, 0.0), Point(1.0, 0.0), Point(2.0, 0.0)};
  for (const auto& nodes :
       std::vector<std::vector<std::uint32_t>>{{0, 1}, {}, {1, 2, 2}}) {
    auto chain = std::make_unique<FeedlineChainT>();
    chain->nodes = nodes;
    assignment.chains.push_back(std::move(chain));
  }

  EXPECT_EQ(validate(wrap(std::move(assignment))),
            (std::vector<std::string>{
                "connection 1 targets port 4, not port 2 of ring node 1",
                "chain 1 runs through no ring node",
                "chains 0 and 2 both name ring node 1",
                "chain 2 names ring node 2 twice"}));
}

TEST(ArtifactSchema, RefusesAGlobalRoutingThatAPictureCannotDraw) {
  // A picture draws a lattice by index, and the Assignment stage reads the
  // role of each connection.
  GlobalRoutingT routing;
  routing.connections.push_back(resonatorConnection());
  routing.connections.push_back(std::make_unique<ConnectionT>());
  routing.connections.back()->target = PortRef(1);
  auto odd = std::make_unique<LatticeT>();
  odd->points = {Point(0.0, 0.0), Point(1.0, 0.0)};
  odd->edges = {0, 1, 0};
  routing.lattices.push_back(std::move(odd));
  auto outside = std::make_unique<LatticeT>();
  outside->points = {Point(0.0, 0.0), Point(1.0, 0.0)};
  outside->edges = {0, 2};
  outside->selected = {1};
  routing.lattices.push_back(std::move(outside));

  EXPECT_EQ(
      validate(wrap(std::move(routing))),
      (std::vector<std::string>{
          "connection 1: source role is unset",
          "connection 1: target role is unset",
          "lattice 0 holds 3 edge indices, not two per edge",
          "lattice 1 names point 2 of 2", "lattice 1 selects edge 1 of 1"}));
}

TEST(ArtifactSchema, RefusesACapacityChainThatLeavesItsNodesOrIsNoTree) {
  // The later stages walk a chain by index, so an index outside the nodes, a
  // link back to an earlier node or a second path to one node would make the
  // walk read past the end, never stop or count a subtree twice.
  // Each target node names a port of its own.
  std::uint32_t port = 0;
  const auto node = [&port](const std::vector<std::uint32_t>& next) {
    auto entry = std::make_unique<CapacityNodeT>();
    entry->kind = CapacityElement::Target;
    entry->id = port++;
    entry->next = next;
    return entry;
  };
  const auto launcher = [] {
    auto entry = std::make_unique<CapacityNodeT>();
    entry->kind = CapacityElement::Launcher;
    return entry;
  };

  CapacityPlanT shared = emptyPlan();
  shared.nodes.push_back(node({2}));
  shared.nodes.push_back(node({2}));
  shared.nodes.push_back(launcher());
  shared.chains = {0, 1};
  // Two chains may share a subtree without a target.
  EXPECT_TRUE(validate(wrap(std::move(shared))).empty());

  CapacityPlanT diamond = emptyPlan();
  diamond.nodes.push_back(node({1, 2}));
  diamond.nodes.push_back(node({2}));
  diamond.nodes.push_back(launcher());
  diamond.chains = {0};
  // One chain may not reach a node twice: the walk would count it twice.
  EXPECT_EQ(
      validate(wrap(std::move(diamond))),
      (std::vector<std::string>{"capacity chain 0 reaches node 2 twice"}));

  CapacityPlanT outside = emptyPlan();
  outside.nodes.push_back(node({3}));
  outside.chains = {2};
  EXPECT_EQ(
      validate(wrap(std::move(outside))),
      (std::vector<std::string>{"capacity node 0 links to node 3 of 1",
                                "capacity chain 0 starts at node 2 of 1"}));

  CapacityPlanT cycle = emptyPlan();
  cycle.nodes.push_back(node({1}));
  cycle.nodes.push_back(node({0}));
  cycle.chains = {0};
  EXPECT_EQ(validate(wrap(std::move(cycle))),
            (std::vector<std::string>{
                "capacity nodes link in a cycle: 2 of 2 nodes lie on or "
                "behind one"}));

  CapacityPlanT loop = emptyPlan();
  loop.nodes.push_back(node({0}));
  loop.chains = {0};
  EXPECT_EQ(validate(wrap(std::move(loop))),
            (std::vector<std::string>{
                "capacity nodes link in a cycle: 1 of 1 node lies on or "
                "behind one"}));
}

TEST(ArtifactSchema, RefusesATargetOnTwoCapacityChains) {
  // The Global stage asks one wire of each target over its chain, so a
  // target on two chains, or twice on one, would ask for its wire twice.
  const auto target = [](const std::uint32_t port,
                         const std::vector<std::uint32_t>& next) {
    auto entry = std::make_unique<CapacityNodeT>();
    entry->kind = CapacityElement::Target;
    entry->id = port;
    entry->next = next;
    return entry;
  };

  CapacityPlanT sharedTarget = emptyPlan();
  sharedTarget.nodes.push_back(target(0, {2}));
  sharedTarget.nodes.push_back(target(1, {2}));
  sharedTarget.nodes.push_back(target(2, {}));
  sharedTarget.chains = {0, 1};
  EXPECT_EQ(
      validate(wrap(std::move(sharedTarget))),
      (std::vector<std::string>{"capacity chains 0 and 1 both reach port 2"}));

  CapacityPlanT twoNodes = emptyPlan();
  twoNodes.nodes.push_back(target(0, {}));
  twoNodes.nodes.push_back(target(0, {}));
  twoNodes.chains = {0, 1};
  EXPECT_EQ(
      validate(wrap(std::move(twoNodes))),
      (std::vector<std::string>{"capacity chains 0 and 1 both reach port 0"}));

  CapacityPlanT twice = emptyPlan();
  twice.nodes.push_back(target(0, {1}));
  twice.nodes.push_back(target(0, {}));
  twice.chains = {0};
  EXPECT_EQ(
      validate(wrap(std::move(twice))),
      (std::vector<std::string>{"capacity chain 0 reaches port 0 twice"}));
}

TEST(ArtifactSchema, RefusesAGateNodeThatDisagreesWithItsBottleneck) {
  // The Global stage reads the capacity of a gate from each of its nodes, so
  // a node that says more than the bottleneck would let more wires through.
  CapacityPlanT plan = emptyPlan();
  auto bottleneck = std::make_unique<BottleneckT>();
  bottleneck->capacity = 2;
  plan.bottlenecks.push_back(std::move(bottleneck));
  for (const std::uint32_t capacity : {2U, 3U}) {
    auto gate = std::make_unique<CapacityNodeT>();
    gate->kind = CapacityElement::Bottleneck;
    gate->id = 0;
    gate->capacity = capacity;
    plan.nodes.push_back(std::move(gate));
    plan.chains.push_back(static_cast<std::uint32_t>(plan.nodes.size() - 1));
  }
  EXPECT_EQ(validate(wrap(std::move(plan))),
            (std::vector<std::string>{"capacity node 1 lets 3 wires through "
                                      "bottleneck 0, which lets 2 wires "
                                      "through"}));
}

TEST(ArtifactSchema, RefusesAGateNodeWithoutItsBottleneck) {
  // The Global stage reads the capacity of a gate from its bottleneck, so a
  // gate has to name one of the plan.
  CapacityPlanT plan = emptyPlan();
  plan.bottlenecks.push_back(std::make_unique<BottleneckT>());
  auto gate = std::make_unique<CapacityNodeT>();
  gate->kind = CapacityElement::Bottleneck;
  gate->id = 1;
  plan.nodes.push_back(std::move(gate));
  plan.chains.push_back(0);

  EXPECT_EQ(
      validate(wrap(std::move(plan))),
      (std::vector<std::string>{"capacity node 0 names bottleneck 1 of 1"}));
}

TEST(ArtifactSchema, RefusesAMissingEntryOfEveryOutput) {
  // Writing an artifact packs every entry of a vector of tables, so a missing
  // entry is refused before the write reads it.
  CapacityPlanT plan = emptyPlan();
  plan.partitions.push_back(std::make_unique<PartitionT>());
  plan.partitions.back()->outlines.push_back(nullptr);
  plan.partitions.push_back(nullptr);
  plan.borders.push_back(nullptr);
  plan.bottlenecks.push_back(nullptr);
  plan.launchers.push_back(nullptr);
  plan.port_keepout.push_back(nullptr);
  plan.nodes.push_back(nullptr);
  EXPECT_EQ(validate(wrap(std::move(plan))),
            (std::vector<std::string>{
                "partition 1: missing", "partition 0 outline 0: missing",
                "border 0: missing", "bottleneck 0: missing",
                "launcher slot 0: missing", "port keepout 0: missing",
                "capacity node 0: missing"}));

  AssignmentT assignment;
  assignment.chains.push_back(nullptr);
  EXPECT_EQ(validate(wrap(std::move(assignment))),
            (std::vector<std::string>{"chain 0: missing"}));

  GlobalRoutingT global;
  global.lattices.push_back(nullptr);
  EXPECT_EQ(validate(wrap(std::move(global))),
            (std::vector<std::string>{"lattice 0: missing"}));

  CorridorRoutingT corridors;
  corridors.slots.push_back(nullptr);
  corridors.corridors.push_back(nullptr);
  EXPECT_EQ(validate(wrap(std::move(corridors))),
            (std::vector<std::string>{"border slots 0: missing",
                                      "corridor 0: missing"}));

  CapacityPlanT unwritable = emptyPlan();
  unwritable.bottlenecks.push_back(nullptr);
  EXPECT_THROW(static_cast<void>(writeArtifact(wrap(std::move(unwritable)))),
               std::invalid_argument);

  // The two grids are required tables, which the write cannot leave out.
  CapacityPlanT withoutGrids = emptyPlan();
  withoutGrids.capacity_grid.reset();
  withoutGrids.detail_grid.reset();
  EXPECT_EQ(validate(wrap(std::move(withoutGrids))),
            (std::vector<std::string>{"capacity grid: missing",
                                      "detail grid: missing"}));
}

TEST(ArtifactSchema, RefusesAPortTheChipDoesNotHave) {
  // An artifact names ports by index, so one made from another chip can name
  // a port past the end of this chip's port list.
  ChipT chip;
  chip.ports.push_back(std::make_unique<PortT>());
  chip.ports.push_back(std::make_unique<PortT>());

  GlobalRoutingT routing;
  routing.outer_ring = {PortRef(0), PortRef(2)};
  routing.resonators = {PortRef(1)};
  routing.connections.push_back(std::make_unique<ConnectionT>());
  routing.connections.back()->source = std::make_unique<PortRef>(5);
  routing.connections.back()->target = PortRef(1);
  EXPECT_EQ(validate(wrap(std::move(routing)), chip),
            (std::vector<std::string>{"connection 0 source names port 5 of 2",
                                      "ring node 1 names port 2 of 2"}));

  AssignmentT assignment;
  assignment.ring = {PortRef(1)};
  assignment.launchers = {PortRef(3)};
  auto chain = std::make_unique<FeedlineChainT>();
  chain->start = std::make_unique<PortRef>(0);
  chain->end = std::make_unique<PortRef>(4);
  assignment.chains.push_back(std::move(chain));
  EXPECT_EQ(validate(wrap(std::move(assignment)), chip),
            (std::vector<std::string>{"launcher 0 names port 3 of 2",
                                      "chain 0 names port 4 of 2"}));

  CapacityPlanT plan;
  auto slot = std::make_unique<LauncherSlotT>();
  slot->port = PortRef(2);
  plan.launchers.push_back(std::move(slot));
  auto target = std::make_unique<CapacityNodeT>();
  target->kind = CapacityElement::Target;
  target->id = 9;
  plan.nodes.push_back(std::move(target));
  auto gate = std::make_unique<CapacityNodeT>();
  gate->kind = CapacityElement::Bottleneck;
  gate->id = 9;
  plan.nodes.push_back(std::move(gate));
  // A gate names a bottleneck and no port.
  EXPECT_EQ(validate(wrap(std::move(plan)), chip),
            (std::vector<std::string>{"launcher slot 0 names port 2 of 2",
                                      "capacity node 0 names port 9 of 2"}));

  GlobalRoutingT fitting;
  fitting.outer_ring = {PortRef(0), PortRef(1)};
  EXPECT_TRUE(validate(wrap(std::move(fitting)), chip).empty());
}

TEST(ArtifactSchema, AssignmentRoundTrips) {
  AssignmentT assignment;
  assignment.connections.push_back(std::make_unique<ConnectionT>());
  assignment.connections.back()->target = PortRef(3);
  assignment.connections.back()->source_role = AssignedRole::ResonatorSource;
  assignment.connections.back()->target_role = AssignedRole::ResonatorTarget;
  assignment.objective = 132.68;
  assignment.ring.emplace_back(3);
  assignment.launchers.emplace_back(9);
  // A feed that is not on its launcher: the point a feedline end starts at.
  assignment.feeds.emplace_back(120.5, -40.25);
  auto chain = std::make_unique<FeedlineChainT>();
  chain->nodes = {0};
  chain->start = std::make_unique<PortRef>(9);
  assignment.chains.push_back(std::move(chain));

  const ArtifactT back = readArtifact(writeArtifact(wrap(assignment)));
  ASSERT_NE(back.output.AsAssignment(), nullptr);
  EXPECT_EQ(*back.output.AsAssignment(), assignment);
  EXPECT_DOUBLE_EQ(back.output.AsAssignment()->objective, 132.68);
}

TEST(ArtifactSchema, FinalRoutingKeepsTheCreatedPortsAndTheFailures) {
  // The coupler carries the port it creates and the connection it completes,
  // so a reloaded run can rebuild the grown port list without the stage.
  FinalRoutingT final;
  final.grid = emptyGrid();
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
  assignment.ring = {PortRef(1)};
  assignment.launchers = {PortRef(0)};
  assignment.feeds.emplace_back(0.0, 0.0);
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
  final.grid = emptyGrid();
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
    // Every required vector is present, so that the producer is the only
    // thing the artifact lacks. A null vector would leave the field out,
    // which is a different problem.
    const std::vector<::flatbuffers::Offset<Lattice>> noLattices;
    const std::vector<::flatbuffers::Offset<Connection>> noConnections;
    const std::vector<PortRef> noPorts;
    const auto output = CreateGlobalRoutingDirect(
        builder, &noLattices, &noConnections, &noPorts, &noPorts);
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

TEST(ArtifactSchema, RefusesAnOutputTypeOfANewerSchema) {
  // A newer schema may append an output type under the same identifier. The
  // verifier lets it pass, so the read has to refuse it.
  flatbuffers::FlatBufferBuilder builder;
  const auto producer = builder.CreateString("mqt-scpd newer");
  const flatbuffers::Offset<void> output(
      builder.EndTable(builder.StartTable()));
  // The tag is written as a plain byte, because the enum of this schema has
  // no value for it.
  const auto start = builder.StartTable();
  builder.AddOffset(Artifact::VT_OUTPUT, output);
  builder.AddOffset(Artifact::VT_PRODUCER, producer);
  builder.AddElement<std::uint8_t>(
      Artifact::VT_OUTPUT_TYPE, static_cast<std::uint8_t>(StageOutput::MAX) + 1,
      0);
  FinishArtifactBuffer(builder,
                       flatbuffers::Offset<Artifact>(builder.EndTable(start)));
  const std::vector<std::uint8_t> bytes(
      builder.GetBufferPointer(),
      std::next(builder.GetBufferPointer(), builder.GetSize()));

  try {
    static_cast<void>(readArtifact(bytes));
    FAIL() << "the read accepted an unknown output type";
  } catch (const std::invalid_argument& error) {
    EXPECT_STREQ(error.what(), "artifact is not valid: output type 8 is "
                               "unknown to this version of mqt-scpd");
  }
}

} // namespace
