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
/// to 100 inside artwork. One crossing slot fits on f1, between the ten
/// cells at either end where the router may not cross; spread over that
/// room it lies in the middle, at x 80, and its exits lie at (80, 157) and
/// (80, 183).
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
/// and down to (40, 160). Its open end at (80, top), above the slot, and
/// the slot's corner at (70.5, 159) close a room round the slot's upper
/// exit.
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

/// The result counts the crossing slots of every feedline edge by its name,
/// as many as the page draws: the chain search reads them against the wires
/// that must cross the edge (`chain_slot_blacklist`).
TEST(SlotCheck, CountsTheCrossingSlotsOfEveryFeedlineEdge) {
  Drawing chip(260, 240, 19.0);
  chip.fill(0, 180, 259, 239, CHAMBER + 1);
  chip.fill(0, 161, 259, 179, ARTWORK);
  chip.fill(60, 161, 170, 179, FEEDLINE);
  json points = json::array();
  for (int x = 60; x <= 170; ++x) {
    points.push_back({x, 170});
  }
  chip.strokes.push_back(
      {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});
  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  ASSERT_TRUE(result.slots.contains("f1"));
  EXPECT_GT(result.slots.at("f1"), 0U);
  EXPECT_EQ(result.slots.at("f1"),
            graph.at("crossSlots").at("edges").at(0).at("slots").size());
}

/// The slots of a straight piece are spread evenly over it, with equal gaps
/// between them and to the two ends. On f1 from x 60 to 170 they lie as far
/// from the one end as from the other.
TEST(SlotCheck, SpreadsTheSlotsOfAStraightPieceEvenly) {
  Drawing chip(260, 240, 19.0);
  chip.fill(0, 180, 259, 239, CHAMBER + 1);
  chip.fill(0, 161, 259, 179, ARTWORK);
  chip.fill(60, 161, 170, 179, FEEDLINE);
  json points = json::array();
  for (int x = 60; x <= 170; ++x) {
    points.push_back({x, 170});
  }
  chip.strokes.push_back(
      {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});
  const auto graph = json::parse(slotCheck(chip.text()).graph);
  std::vector<double> spread;
  for (const auto& slot :
       graph.at("crossSlots").at("edges").at(0).at("slots")) {
    spread.push_back(slot.at("at").at(0).get<double>());
  }
  std::ranges::sort(spread);

  ASSERT_GE(spread.size(), 3U);
  EXPECT_NEAR(spread.front() - 60.0, 170.0 - spread.back(), 1.0);
  for (std::size_t k = 2; k < spread.size(); ++k) {
    EXPECT_NEAR(spread[k] - spread[k - 1], spread[1] - spread[0], 1.0);
  }
}

/// A slot goes only where each of its ends has a way out: one of the
/// router's turns and five cells on, every cell free. On f1 from x 60 to
/// 170, artwork from x 60 to 110 a cell below the lower ends of the slots
/// leaves no turn there, so the slots lie right of it.
TEST(SlotCheck, PlacesASlotOnlyWhereItsEndsHaveAWayOut) {
  Drawing chip(260, 240, 19.0);
  chip.fill(0, 180, 259, 239, CHAMBER + 1);
  chip.fill(0, 161, 259, 179, ARTWORK);
  chip.fill(60, 161, 170, 179, FEEDLINE);
  chip.fill(60, 183, 110, 210, ARTWORK);
  json points = json::array();
  for (int x = 60; x <= 170; ++x) {
    points.push_back({x, 170});
  }
  chip.strokes.push_back(
      {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});
  const auto slotsOf = [](const json& graph) {
    std::vector<double> xs;
    for (const auto& slot :
         graph.at("crossSlots").at("edges").at(0).at("slots")) {
      xs.push_back(slot.at("at").at(0).get<double>());
    }
    std::ranges::sort(xs);
    return xs;
  };

  const auto escaping = json::parse(slotCheck(chip.text()).graph);

  ASSERT_FALSE(slotsOf(escaping).empty());
  EXPECT_GT(slotsOf(escaping).front(), 110.0);
  // Every end of a slot placed shows the way out its placement chose.
  ASSERT_EQ(escaping.at("escapes").size(), 2 * slotsOf(escaping).size());
  for (const auto& end : escaping.at("escapes")) {
    EXPECT_FALSE(end.at("line").is_null()) << end;
  }
  EXPECT_EQ(escaping.at("verdict"), "SAT");
}

/// A slot needs a line across its edge that the feedline routing's crossing
/// rule allows. On an edge of 21 cells the one
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

/// With fewer cells closed at the ends of an edge (`slotPinCells`),
/// the middle of the edge of 21 cells lies outside them, and its slot takes
/// the wire.
TEST(SlotCheck, LaysASlotNearerTheEndsWithFewerPinCells) {
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

  auto text = json::parse(chip.text());
  text["slotPinCells"] = 5;
  const auto result = slotCheck(text.dump());

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_TRUE(result.lost.empty());
}

/// A gate on the line of a mouth between the same two chambers is that
/// mouth: a wire prescribed to cross a feedline edge
/// passes no mouth, so it passes the plain twin of the mouth neither. Below
/// f1, chamber 2 lies behind an artwork wall that only the mouth g0 and its
/// twin g1 cross.
TEST(SlotCheck, LetsNoWirePastAMouthByItsTwin) {
  auto chip = feedlineChip();
  chip.fill(200, 180, 259, 239, CHAMBER + 2);
  chip.fill(199, 180, 199, 239, ARTWORK);
  for (const auto& [name, mouth] :
       {std::pair{"g0", true}, std::pair{"g1", false}}) {
    chip.edges.push_back(
        {{"kind", "bottleneck"},
         {"name", name},
         {"line", {{199, 195}, {199, 205}}},
         {"ends", {"f1 at its coupler, left", "f2 at its coupler, right"}},
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

/// A wire end with a label, as the Final stage writes one.
[[nodiscard]] json endAt(const std::pair<int, int> cell, const int chamber,
                         const std::string& label) {
  return {{"cell", {cell.first, cell.second}},
          {"chambers", {chamber}},
          {"label", label}};
}

/// A wire of a mouth passes that mouth, but a wire whose two ends lie in one
/// chamber passes no gate at all: the mouth line runs through that chamber
/// where no gate cuts it. Wire 1's launcher lies between the launchers of
/// the terminal edges f1 and f2 round the middle of the chip, so it belongs
/// to their mouth g0; g0 holds no wire, and wire 1 needs none of it.
TEST(SlotCheck, AWireOfAMouthWithBothEndsInOneChamberTakesNoGate) {
  Drawing chip(200, 200, 19.0);
  chip.fill(0, 0, 199, 39, CHAMBER + 1);
  chip.fill(0, 40, 199, 40, ARTWORK);
  chip.fill(90, 40, 110, 40, FREE);
  chip.edges.push_back(
      {{"kind", "bottleneck"},
       {"name", "g0"},
       {"line", {{89, 40}, {111, 40}}},
       {"ends", {"f1 at its coupler, left", "f2 at its coupler, right"}},
       {"chambers", {0, 1}},
       {"capacity", 0},
       {"users", nullptr},
       {"length", 22.0},
       {"mouth", true},
       {"slot", false},
       {"terminal", false}});
  const auto add = [&](const std::string& name, const json& source,
                       const json& target, const bool terminal) {
    chip.wires.push_back({{"name", name},
                          {"source", source},
                          {"target", target},
                          {"crosses", json::array()},
                          {"terminal", terminal},
                          {"resonator", false}});
  };
  add("f1", endAt({60, 100}, 0, "coupler port"),
      endAt({10, 100}, 0, "Chip.port1"), true);
  add("f2", endAt({100, 190}, 0, "Chip.port2"),
      endAt({100, 150}, 0, "coupler port"), true);
  add("1", endAt({40, 170}, 0, "Chip.port3"), endAt({60, 150}, 0, "Qb1.port1"),
      false);

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_EQ(result.lost, std::vector<std::string>{});
  EXPECT_EQ(wireNamed(graph, "1").at("status"), "routed");
  EXPECT_EQ(wireNamed(graph, "1").at("way"), json::array());
}

/// A wire's own mouth binds it only where the mouth separates its two ends.
/// Wire 1 belongs to the mouth g0 of f1 and f2, which holds no wire, but
/// the gate g1 beside it joins the wire's two chambers without crossing the
/// mouth line: the wire's port lies on the near side of it, and the wire
/// takes g1.
TEST(SlotCheck, AWireOfAMouthThatDoesNotSeparateItsEndsPassesElsewhere) {
  Drawing chip(200, 200, 19.0);
  chip.fill(0, 0, 199, 39, CHAMBER + 1);
  chip.fill(0, 40, 199, 40, ARTWORK);
  chip.fill(90, 40, 110, 40, FREE);
  chip.fill(150, 40, 170, 40, FREE);
  const auto gate = [&](const std::string& name, const int x0, const int x1,
                        const json& ends, const int capacity,
                        const bool mouth) {
    chip.edges.push_back({{"kind", "bottleneck"},
                          {"name", name},
                          {"line", {{x0, 40}, {x1, 40}}},
                          {"ends", ends},
                          {"chambers", {0, 1}},
                          {"capacity", capacity},
                          {"users", nullptr},
                          {"length", static_cast<double>(x1 - x0)},
                          {"mouth", mouth},
                          {"slot", false},
                          {"terminal", false}});
  };
  gate("g0", 89, 111, {"f1 at its coupler, left", "f2 at its coupler, right"},
       0, true);
  gate("g1", 149, 171, {"artwork", "artwork"}, 1, false);
  const auto add = [&](const std::string& name, const json& source,
                       const json& target, const bool terminal) {
    chip.wires.push_back({{"name", name},
                          {"source", source},
                          {"target", target},
                          {"crosses", json::array()},
                          {"terminal", terminal},
                          {"resonator", false}});
  };
  add("f1", endAt({60, 100}, 0, "coupler port"),
      endAt({10, 100}, 0, "Chip.port1"), true);
  add("f2", endAt({100, 190}, 0, "Chip.port2"),
      endAt({100, 150}, 0, "coupler port"), true);
  add("1", endAt({40, 170}, 0, "Chip.port3"), endAt({160, 20}, 1, "Qb1.port1"),
      false);

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.lost, std::vector<std::string>{});
  EXPECT_EQ(wireNamed(graph, "1").at("way"),
            json::array({edgeNamed(graph, "g1").at("index")}));
}

/// A gate with both ends on one feedline edge lies along that edge's own
/// run: the terminal edge routed as a wire passes it, however narrow it is
/// for any other wire. Here g0 holds no wire by its length, f1 passes it,
/// and wire 2 still has no way.
TEST(SlotCheck, AGateAlongATerminalEdgeLetsThatEdgePass) {
  auto chip = gateChip(0);
  chip.edges[0]["ends"] = {"f1 at its coupler, right",
                           "f1 at its launcher, right"};
  chip.wires.push_back({{"name", "f1"},
                        {"source", endAt({20, 5}, 0, "Chip.port1")},
                        {"target", endAt({20, 35}, 1, "coupler port")},
                        {"crosses", json::array()},
                        {"terminal", true},
                        {"resonator", false}});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.lost, std::vector<std::string>{"2"});
  EXPECT_EQ(wireNamed(graph, "f1").at("way"),
            json::array({edgeNamed(graph, "g0").at("index")}));
}

/// A gate from one side of a feedline edge to the other lies across that
/// edge, not along it: the terminal edge routed as a wire takes its room
/// like any wire. Here g0 holds one wire, f1 and wire 2 both need it.
TEST(SlotCheck, AGateAcrossATerminalEdgeCountsThatEdge) {
  auto chip = gateChip(1);
  chip.edges[0]["ends"] = {"f1 at its launcher, left",
                           "f1 at its launcher, right"};
  chip.wires.push_back({{"name", "f1"},
                        {"source", endAt({20, 5}, 0, "Chip.port1")},
                        {"target", endAt({20, 35}, 1, "coupler port")},
                        {"crosses", json::array()},
                        {"terminal", true},
                        {"resonator", false}});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  const auto result = slotCheck(chip.text());

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.over, std::vector<std::string>{"g0"});
  EXPECT_EQ(edgeNamed(json::parse(result.graph), "g0").at("load"), 2);
}

/// A way out of a slot keeps its clearance from the stub of a port at a
/// qubit or a coupler, but not from the stub of a launcher: a wire that
/// crosses the edge may leave its slot along a launcher's straight run. A
/// stub of wire 2 lies a few cells under f1 from x 60 to 110; at a
/// launcher it leaves the slots there, at a qubit it pushes them past it.
TEST(SlotCheck, AWayOutMayRunBesideALauncherStub) {
  const auto lowestSlot = [](const std::string& label) {
    Drawing chip(260, 240, 19.0);
    chip.fill(0, 180, 259, 239, CHAMBER + 1);
    chip.fill(0, 161, 259, 179, ARTWORK);
    chip.fill(60, 161, 170, 179, FEEDLINE);
    chip.fill(60, 195, 110, 195, STUB);
    json points = json::array();
    for (int x = 60; x <= 170; ++x) {
      points.push_back({x, 170});
    }
    chip.strokes.push_back(
        {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
    chip.strokes.push_back({{"name", "2 at its source, left"},
                            {"kind", "stub"},
                            {"points", {{110, 195}, {60, 195}}}});
    chip.wires.push_back({{"name", "2"},
                          {"source", endAt({110, 195}, 1, label)},
                          {"target", endAt({200, 60}, 0, "coupler port")},
                          {"crosses", {"f1"}},
                          {"terminal", false},
                          {"resonator", false}});
    const auto graph = json::parse(slotCheck(chip.text()).graph);
    double lowest = 1e9;
    for (const auto& slot :
         graph.at("crossSlots").at("edges").at(0).at("slots")) {
      lowest = std::min(lowest, slot.at("at").at(0).get<double>());
    }
    return lowest;
  };

  EXPECT_LT(lowestSlot("Chip.port1"), 110.0);
  EXPECT_GT(lowestSlot("Qb1.port1"), 110.0);
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

/// An edge of an open chain is prescribed to no wire, but its stretches
/// stay open to the wires that bridge it, and so do its slots: wire 1
/// bridges f1 and crosses it without a prescription; wire 2 does not
/// bridge f1 and has no way.
TEST(SlotCheck, WireThatBridgesAnEdgeCrossesItWithoutPrescription) {
  auto chip = feedlineChip();
  chip.edges.push_back({{"kind", "stretch"},
                        {"name", "× f1 cells 0..40"},
                        {"feedline", "f1"},
                        {"chambers", {0, 1}},
                        {"capacity", 1},
                        {"users", {"1"}}});
  chip.wire("1", {200, 60}, 0, {200, 220}, 1, {});
  chip.wire("2", {220, 60}, 0, {220, 220}, 1, {});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "UNSAT");
  EXPECT_EQ(result.lost, std::vector<std::string>{"2"});
  const auto slot = edgeNamed(graph, "□ f1 1");
  ASSERT_FALSE(slot.is_null());
  EXPECT_EQ(slot.at("users"), json::array({"1"}));
  EXPECT_EQ(wireNamed(graph, "1").at("way"), json::array({slot.at("index")}));
}

/// An edge of a chain with a reflector end may have one chamber on both
/// sides; its slots then lead from that chamber back into it, and a wire
/// prescribed to cross the edge crosses at one of them. Here f1 lies in
/// chamber 0 alone, and wire 1 starts and ends in chamber 0.
TEST(SlotCheck, CrossesASlotThatLeadsBackIntoItsChamber) {
  Drawing chip(260, 240, 19.0);
  chip.fill(60, 161, 100, 179, FEEDLINE);
  json points = json::array();
  for (int x = 60; x <= 100; ++x) {
    points.push_back({x, 170});
  }
  chip.strokes.push_back(
      {{"name", "f1"}, {"kind", "feedline"}, {"points", std::move(points)}});
  chip.edges.push_back({{"kind", "stretch"},
                        {"name", "× f1 cells 0..40"},
                        {"feedline", "f1"},
                        {"chambers", {0, 0}},
                        {"capacity", 1},
                        {"users", {"1"}}});
  chip.wire("1", {200, 60}, 0, {200, 220}, 0, {"f1"});

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_TRUE(result.lost.empty());
  const auto slot = edgeNamed(graph, "□ f1 1");
  ASSERT_FALSE(slot.is_null());
  EXPECT_EQ(slot.at("chambers"), json::array({0, 0}));
  EXPECT_EQ(wireNamed(graph, "1").at("way"), json::array({slot.at("index")}));
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
  EXPECT_EQ(result.crowded, (std::vector<std::string>{"1", "2"}));
  const auto gate = edgeNamed(json::parse(result.graph), "g0");
  EXPECT_EQ(gate.at("load"), 2);
  EXPECT_EQ(gate.at("overflow"), 1);
}

/// A gate is counted across the heading the wires cross it on
/// across the wires' heading: its line lies 21.8 degrees off the nearest of the
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

/// A standing wire, as a terminal edge of another chain in a local check, is
/// not routed: it takes no room of a gate, is not lost and is not shown.
TEST(SlotCheck, AStandingWireTakesNoRoom) {
  auto chip = gateChip(1);
  chip.wire("1", {20, 5}, 0, {20, 35}, 1, {});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});
  chip.wires[1]["standing"] = true;

  const auto result = slotCheck(chip.text());
  const auto graph = json::parse(result.graph);

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_TRUE(result.lost.empty());
  EXPECT_EQ(edgeNamed(graph, "g0").at("load"), 1);
  ASSERT_EQ(graph.at("wires").size(), 1U);
  EXPECT_EQ(graph.at("wires")[0].at("name"), "1");
}

TEST(SlotCheck, GateWithRoomForBothIsSat) {
  auto chip = gateChip(2);
  chip.wire("1", {20, 5}, 0, {20, 35}, 1, {});
  chip.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  const auto result = slotCheck(chip.text());

  EXPECT_EQ(result.verdict, "SAT");
  EXPECT_EQ(edgeNamed(json::parse(result.graph), "g0").at("load"), 2);
}

/// A graph whose raster is a view over wall and chamber buffers
/// (`SlotGraph::raster`) is judged as the same graph with its codes written
/// out.
TEST(SlotCheck, ARasterViewIsJudgedAsTheCodes) {
  auto feedline = feedlineChip();
  feedline.wire("1", {200, 60}, 0, {200, 220}, 1, {"f1"});
  addStub(feedline, "9", 145);
  feedline.wire("7", {60, 152}, 0, {200, 50}, 0, {});
  auto gate = gateChip(1);
  gate.wire("1", {20, 5}, 0, {20, 35}, 1, {});
  gate.wire("2", {22, 5}, 0, {22, 35}, 1, {});

  for (const auto& text : {feedline.text(), gate.text()}) {
    const auto coded = slotGraphOf(text);
    // The buffers of the view, read off the codes: walls 0 to 4 for the
    // codes 1 to 5, and chamber c for the code 6 + c.
    const std::vector<std::uint16_t> wallCodes{1, 2, 3, 4, 5};
    constexpr std::uint32_t NONE = 0xFFFFFFFFU;
    std::vector<std::uint64_t> mask((coded.codes.size() + 63) / 64, 0);
    std::vector<std::uint32_t> wallOf(coded.codes.size(), 0);
    std::vector<std::uint32_t> chamberOf(coded.codes.size(), NONE);
    for (std::size_t cell = 0; cell < coded.codes.size(); ++cell) {
      const auto code = coded.codes[cell];
      if (code >= 1 && code <= 5) {
        mask[cell / 64] |= std::uint64_t{1} << (cell % 64);
        wallOf[cell] = code - 1U;
      } else if (code >= 6) {
        chamberOf[cell] = code - 6U;
      }
    }
    auto viewed = coded;
    viewed.codes.clear();
    viewed.raster = SlotGraph::RasterView{.wallMask = mask.data(),
                                          .wallOf = wallOf.data(),
                                          .wallCodes = wallCodes.data(),
                                          .walls = wallCodes.size(),
                                          .otherWall = 2,
                                          .chamberOf = chamberOf.data(),
                                          .noChamber = NONE};
    for (std::size_t cell = 0; cell < coded.codes.size(); ++cell) {
      ASSERT_EQ(viewed.raster->code(cell), coded.codes[cell]) << cell;
    }
    const auto quickCoded = slotCheck(coded);
    const auto quickViewed = slotCheck(viewed);
    EXPECT_EQ(quickViewed.verdict, quickCoded.verdict);
    EXPECT_EQ(quickViewed.lost, quickCoded.lost);
    EXPECT_EQ(quickViewed.over, quickCoded.over);
    EXPECT_EQ(quickViewed.crowded, quickCoded.crowded);
  }
}

TEST(SlotCheck, RejectsAGraphWithoutRaster) {
  EXPECT_THROW((void)slotCheck(R"({"width": 2, "height": 2})"),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::pipeline
