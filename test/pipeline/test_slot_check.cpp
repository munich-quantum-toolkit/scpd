/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The check by crossing slots on small capacity graphs drawn by hand.

#include "mqt-scpd/pipeline/SlotCheck.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using nlohmann::json;

constexpr int FREE = 0;
constexpr int ARTWORK = 2;
constexpr int STUB = 4;
constexpr int FEEDLINE = 5;
/// The raster code of chamber c is CHAMBER + c.
constexpr int CHAMBER = 6;

/// A capacity graph in the form of `final-capacity-graph.json`, drawn cell
/// by cell.
struct Drawing {
  std::int64_t width;
  std::int64_t height;
  double clearance;
  std::vector<int> code;
  json strokes = json::array();
  json edges = json::array();
  json wires = json::array();

  Drawing(const std::int64_t w, const std::int64_t h, const double clear)
      : width(w), height(h), clearance(clear),
        code(static_cast<std::size_t>(w * h), CHAMBER) {}

  /// Sets every cell of the rectangle from (x0, y0) to (x1, y1), both
  /// included.
  void fill(const std::int64_t x0, const std::int64_t y0, const std::int64_t x1,
            const std::int64_t y1, const int value) {
    for (auto y = y0; y <= y1; ++y) {
      for (auto x = x0; x <= x1; ++x) {
        code[static_cast<std::size_t>((y * width) + x)] = value;
      }
    }
  }

  void wire(const std::string& name, const std::pair<int, int> from,
            const int fromChamber, const std::pair<int, int> to,
            const int toChamber, const std::vector<std::string>& crosses) {
    wires.push_back(
        {{"name", name},
         {"source",
          {{"cell", {from.first, from.second}}, {"chambers", {fromChamber}}}},
         {"target",
          {{"cell", {to.first, to.second}}, {"chambers", {toChamber}}}},
         {"crosses", crosses},
         {"terminal", false},
         {"resonator", false}});
  }

  [[nodiscard]] std::string text() const {
    json raster = json::array();
    for (std::size_t at = 0; at < code.size();) {
      auto end = at;
      while (end < code.size() && code[end] == code[at]) {
        ++end;
      }
      raster.push_back(code[at]);
      raster.push_back(end - at);
      at = end;
    }
    return json{
        {"width", width},
        {"height", height},
        {"clearance", clearance},
        {"inflation", clearance / 2},
        {"crossingReach", 11},
        {"wallKinds", {"border", "artwork", "coupler pad", "stub", "feedline"}},
        {"raster", std::move(raster)},
        {"slots", json::array()},
        {"strokes", strokes},
        {"nodes", {{{"chamber", 0}}, {{"chamber", 1}}}},
        {"edges", edges},
        {"wires", wires},
        {"verdict", "SAT"},
        {"summary", json::array()}}
        .dump();
  }
};

/// A chip of 260 by 240 cells with clearance 19: chamber 0 above, chamber
/// 1 below, and between them the feedline edge f1 along row 170 from x 60
/// to 100 inside artwork. One crossing slot fits on f1, at x 71, the first
/// place past the ten cells at the edge's end where the router may cross;
/// its exits lie at (71, 157) and (71, 183).
Drawing feedlineChip() {
  Drawing chip(260, 240, 19.0);
  chip.fill(0, 180, 259, 239, CHAMBER + 1);
  chip.fill(0, 161, 259, 179, ARTWORK);
  chip.fill(60, 161, 100, 179, FEEDLINE);
  json points = json::array();
  for (int x = 60; x <= 100; ++x) {
    points.push_back({x, 170});
  }
  chip.strokes.push_back(
      {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
  return chip;
}

/// Adds a stub of `owner` shaped as an L: from (80, top) left to (40, top)
/// and down to (40, 160). Its open end at (80, top) and the corner of the
/// slot at (80.5, 159) close a room round the slot's upper exit.
void addStub(Drawing& chip, const std::string& owner, const int top) {
  chip.fill(40, top, 80, top, STUB);
  chip.fill(40, top, 40, 160, STUB);
  json points = json::array();
  for (int x = 80; x >= 40; --x) {
    points.push_back({x, top});
  }
  for (int y = top + 1; y <= 160; ++y) {
    points.push_back({40, y});
  }
  chip.strokes.push_back({{"name", "lead of " + owner + ", left"},
                          {"kind", "stub"},
                          {"points", std::move(points)}});
}

/// A chip of 40 by 40 cells: chamber 0 above row 20, chamber 1 below, and
/// along row 20 artwork with a cut of ten cells, the gate g0.
Drawing gateChip(const int capacity) {
  Drawing chip(40, 40, 4.0);
  chip.fill(0, 21, 39, 39, CHAMBER + 1);
  chip.fill(0, 20, 39, 20, ARTWORK);
  chip.fill(15, 20, 24, 20, FREE);
  chip.edges.push_back({{"kind", "bottleneck"},
                        {"name", "g0"},
                        {"line", {{14, 20}, {25, 20}}},
                        {"ends", {"artwork", "artwork"}},
                        {"chambers", {0, 1}},
                        {"capacity", capacity},
                        {"users", nullptr},
                        {"length", 11.0},
                        {"mouth", false},
                        {"slot", false},
                        {"terminal", false}});
  return chip;
}

[[nodiscard]] json edgeNamed(const json& graph, const std::string& name) {
  for (const auto& edge : graph.at("edges")) {
    if (edge.at("name") == name) {
      return edge;
    }
  }
  return {};
}

[[nodiscard]] bool summarySays(const json& graph, const std::string& line) {
  for (const auto& said : graph.at("summary")) {
    if (said == line) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] json wireNamed(const json& graph, const std::string& name) {
  for (const auto& wire : graph.at("wires")) {
    if (wire.at("name") == name) {
      return wire;
    }
  }
  return {};
}

TEST(SlotCheck, CrossingPrescriptionTakesTheSlot) {
  auto chip = feedlineChip();
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_TRUE(result.lost.empty());
  EXPECT_TRUE(result.over.empty());
  const auto slot = edgeNamed(graph, "□ f1 1");
  ASSERT_FALSE(slot.is_null());
  EXPECT_EQ(slot.at("kind"), "stretch");
  EXPECT_EQ(slot.at("load"), 1);
  EXPECT_EQ(wireNamed(graph, "1").at("way"), json::array({slot.at("index")}));
  EXPECT_EQ(graph.at("verdict"), "SAT");
}

/// A slot needs a line across its edge that the feedline routing's crossing
/// rule allows (`SCPD_SLOT_CROSSING_RULE`). On an edge of 21 cells the one
/// straight piece a slot fits on has its middle on the last of the first
/// ten cells' neighbours, where the router may not cross, so the edge holds
/// no slot and the wire prescribed to cross it has no way.
TEST(SlotCheck, LaysNoSlotWhereTheRouterMayNotCross) {
  Drawing chip(260, 240, 19.0);
  chip.fill(0, 180, 259, 239, CHAMBER + 1);
  chip.fill(0, 161, 259, 179, ARTWORK);
  chip.fill(60, 161, 80, 179, FEEDLINE);
  json points = json::array();
  for (int x = 60; x <= 80; ++x) {
    points.push_back({x, 170});
  }
  chip.strokes.push_back(
      {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.lost, std::vector<std::string>{"1"});
  const auto& edges = graph.at("crossSlots").at("edges");
  ASSERT_EQ(edges.size(), 1U);
  EXPECT_TRUE(edges[0].at("slots").empty());
  bool ruled = false;
  for (const auto& place : edges[0].at("rejected")) {
    ruled = ruled || place.at("why") == "crossing rule";
  }
  EXPECT_TRUE(ruled) << edges[0].at("rejected");
}

/// A gate on the line of a mouth between the same two chambers is that
/// mouth (`SCPD_MOUTH_TWINS`): a wire prescribed to cross a feedline edge
/// passes no mouth, so it passes the plain twin of the mouth neither. Below
/// f1, chamber 2 lies behind an artwork wall that only the mouth g0 and its
/// twin g1 cross.
TEST(SlotCheck, LetsNoWirePastAMouthByItsTwin) {
  auto chip = feedlineChip();
  chip.fill(200, 180, 259, 239, CHAMBER + 2);
  chip.fill(199, 180, 199, 239, ARTWORK);
  for (const auto& [name, mouth] :
       {std::pair{"g0", true}, std::pair{"g1", false}}) {
    chip.edges.push_back({{"kind", "bottleneck"},
                          {"name", name},
                          {"line", {{199, 195}, {199, 205}}},
                          {"ends",
                           {"f1 at its coupler, left", "f2 at its coupler, right"}},
                          {"chambers", {1, 2}},
                          {"capacity", 3},
                          {"users", nullptr},
                          {"length", 10.0},
                          {"mouth", mouth},
                          {"slot", false},
                          {"terminal", false}});
  }
  chip.wire("1", {100, 60}, 0, {230, 220}, 2, {"f1"});
  auto graph = json::parse(chip.text());
  graph["nodes"].push_back({{"chamber", 2}});

  const auto result = slotCheck(graph.dump());

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.lost, std::vector<std::string>{"1"});
  EXPECT_EQ(edgeNamed(json::parse(result.graph), "g1").at("mouth"), true);
}

TEST(SlotCheck, WireWithoutPrescriptionCannotCross) {
  auto chip = feedlineChip();
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.lost, std::vector<std::string>{"1"});
  EXPECT_EQ(wireNamed(graph, "1").at("status"), "no way");
  EXPECT_TRUE(summarySays(graph, "no way in the graph for 1"))
      << graph.at("summary");
}

TEST(SlotCheck, WireWithAPortInNoChamberIsLost) {
  auto chip = feedlineChip();
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});
  chip.wires[0]["target"]["chambers"] = json::array();

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.lost, std::vector<std::string>{"1"});
  EXPECT_TRUE(summarySays(graph, "a port in no chamber for 1"))
      << graph.at("summary");
}

TEST(SlotCheck, GateOverCapacityIsUnsat) {
  auto chip = gateChip(1);
  chip.wire("1", {20, 5}, 0, {20, 35}, 1, {});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  const auto result = slotCheck(chip.text());

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_TRUE(result.lost.empty());
  EXPECT_EQ(result.over, std::vector<std::string>{"g0"});
  const auto gate = edgeNamed(json::parse(result.graph), "g0");
  EXPECT_EQ(gate.at("load"), 2);
  EXPECT_EQ(gate.at("overflow"), 1);
}

/// A gate is counted across the heading the wires cross it on
/// (`SCPD_GATE_HEADING`): its line lies 21.8 degrees off the nearest of the
/// eight headings' normals, so two wires that would fit 8.6 cells straight
/// across take 8.6 / cos(21.8) = 9.3 of it at that angle, and the gate
/// holds one.
TEST(SlotCheck, CountsAGateAcrossTheHeadingTheWiresCrossItOn) {
  auto chip = gateChip(2);
  chip.edges[0]["line"] = {{14, 20}, {24, 16}};
  chip.edges[0]["length"] = 8.6;
  chip.wire("1", {20, 5}, 0, {20, 35}, 1, {});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  const auto result = slotCheck(chip.text());

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.over, std::vector<std::string>{"g0"});
  EXPECT_EQ(edgeNamed(json::parse(result.graph), "g0").at("capacity"), 1);
}

TEST(SlotCheck, GateWithRoomForBothIsSat) {
  auto chip = gateChip(2);
  chip.wire("1", {20, 5}, 0, {20, 35}, 1, {});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  const auto result = slotCheck(chip.text());

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_EQ(edgeNamed(json::parse(result.graph), "g0").at("load"), 2);
}

TEST(SlotCheck, SlotAndStubShareALineForOneWire) {
  // The line from the slot's corner to the stub's open end is 19 cells
  // long and takes one wire. Wire 7 starts in the room behind it and wire
  // 2 crosses f1 at the slot whose exit lies in that room: both pass it.
  auto chip = feedlineChip();
  addStub(chip, "7", 140);
  chip.wire("7", {60, 150}, 0, {200, 50}, 0, {});
  chip.wire("2", {200, 60}, 0, {200, 220}, 1, {"f1"});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_TRUE(result.lost.empty());
  EXPECT_EQ(result.over,
            std::vector<std::string>{"□ f1 1 to stub (80, 140) 2/1"});
  const auto line = edgeNamed(graph, "S □ f1 1 to stub (80, 140)");
  ASSERT_FALSE(line.is_null());
  EXPECT_TRUE(line.at("selfBottleneck").get<bool>());
  EXPECT_EQ(line.at("capacity"), 1);
  EXPECT_EQ(line.at("through"), json::array({"2", "7"}));
  EXPECT_EQ(line.at("endsInRoom"), json::array({"7"}));
}

TEST(SlotCheck, StubTakesItsOwnWire) {
  // The line from the slot's corner to the stub's open end is 14 cells
  // long, less than a clearance, yet the stub's own wire always leaves the
  // stub: the line takes wire 7.
  auto chip = feedlineChip();
  addStub(chip, "7", 145);
  chip.wire("7", {60, 152}, 0, {200, 50}, 0, {});

  const auto result = slotCheck(chip.text());
  const auto line =
      edgeNamed(json::parse(result.graph), "S □ f1 1 to stub (80, 145)");

  EXPECT_EQ(result.verdict, "SAT");
  ASSERT_FALSE(line.is_null());
  EXPECT_EQ(line.at("capacity"), 1);
  EXPECT_EQ(line.at("load"), 1);
}

TEST(SlotCheck, StubOfAnotherWireTakesNoWireThroughANarrowLine) {
  auto chip = feedlineChip();
  addStub(chip, "9", 145);
  chip.wire("7", {60, 152}, 0, {200, 50}, 0, {});

  const auto result = slotCheck(chip.text());

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.over,
            std::vector<std::string>{"□ f1 1 to stub (80, 145) 1/0"});
}

TEST(SlotCheck, RejectsAGraphWithoutRaster) {
  EXPECT_THROW((void)slotCheck(R"({"width": 2, "height": 2})"),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::pipeline
