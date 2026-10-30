/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The persistent, canonical CDT of the gates and chambers: objects inserted
// into a base state give the gates of `cdtGates`, and a state is the same
// whichever way it was built.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/CdtGates.hpp"
#include "mqt-scpd/grid/CdtWorld.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t FREE = std::numeric_limits<std::uint32_t>::max();

/// A wall raster built by hand: rectangles and thick lines, each a label.
struct Raster {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  BitGrid mask;
  std::vector<std::uint32_t> owner;
  std::vector<CdtWall> walls;

  Raster(const std::uint32_t w, const std::uint32_t h)
      : width(w), height(h), mask(w, h),
        owner(static_cast<std::size_t>(w) * h, FREE) {}

  void cell(const std::int64_t x, const std::int64_t y, const std::uint32_t label) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return;
    }
    const auto at = static_cast<std::size_t>((y * width) + x);
    mask.set(at);
    owner[at] = label;
  }
  std::uint32_t wall(const CdtWall& kind) {
    walls.push_back(kind);
    return static_cast<std::uint32_t>(walls.size() - 1);
  }
  /// A rectangle of cells, both corners included.
  void rect(const std::uint32_t label, const std::int64_t x0, const std::int64_t y0,
            const std::int64_t x1, const std::int64_t y1) {
    for (auto y = y0; y <= y1; ++y) {
      for (auto x = x0; x <= x1; ++x) {
        cell(x, y, label);
      }
    }
  }
  /// A line between two points, every cell within `half` of it.
  void line(const std::uint32_t label, const double ax, const double ay,
            const double bx, const double by, const double half) {
    const auto x0 = static_cast<std::int64_t>(std::floor(std::min(ax, bx) - half - 1));
    const auto x1 = static_cast<std::int64_t>(std::ceil(std::max(ax, bx) + half + 1));
    const auto y0 = static_cast<std::int64_t>(std::floor(std::min(ay, by) - half - 1));
    const auto y1 = static_cast<std::int64_t>(std::ceil(std::max(ay, by) + half + 1));
    const auto dx = bx - ax;
    const auto dy = by - ay;
    const auto squared = (dx * dx) + (dy * dy);
    for (auto y = y0; y <= y1; ++y) {
      for (auto x = x0; x <= x1; ++x) {
        const auto px = static_cast<double>(x) + 0.5;
        const auto py = static_cast<double>(y) + 0.5;
        const auto t = std::clamp((((px - ax) * dx) + ((py - ay) * dy)) / squared, 0.0, 1.0);
        if (std::hypot(ax + (t * dx) - px, ay + (t * dy) - py) <= half) {
          cell(x, y, label);
        }
      }
    }
  }
  /// The raster without the cells of some labels.
  [[nodiscard]] std::pair<BitGrid, std::vector<std::uint32_t>>
  without(const std::set<std::uint32_t>& labels) const {
    BitGrid m(width, height);
    std::vector<std::uint32_t> o(owner.size(), FREE);
    for (std::size_t c = 0; c < owner.size(); ++c) {
      if (mask.test(c) && !labels.contains(owner[c])) {
        m.set(c);
        o[c] = owner[c];
      }
    }
    return {m, o};
  }
};

/// Everything a state comes to, for comparing two states.
struct Snapshot {
  std::vector<std::tuple<std::size_t, std::size_t, std::size_t>> gates;
  std::vector<std::uint8_t> cuts;
  std::vector<std::uint32_t> capacity;
  std::vector<std::array<std::uint32_t, 2>> beside;
  std::uint32_t chambers = 0;
  std::vector<std::array<double, 7>> triangles;
  std::vector<std::uint32_t> of;

  [[nodiscard]] bool operator==(const Snapshot&) const = default;
};

/// What differs between two snapshots, for the message of a failure.
std::string diffOf(const Snapshot& a, const Snapshot& b) {
  std::string out;
  const auto line = [&](const std::string& text) { out += text + "\n"; };
  if (a.gates != b.gates) {
    line("gates " + std::to_string(a.gates.size()) + " vs " + std::to_string(b.gates.size()));
    std::set<std::tuple<std::size_t, std::size_t, std::size_t>> sa(a.gates.begin(), a.gates.end());
    std::set<std::tuple<std::size_t, std::size_t, std::size_t>> sb(b.gates.begin(), b.gates.end());
    for (const auto& g : sa) {
      if (!sb.contains(g)) {
        line("  only first: " + std::to_string(std::get<0>(g)) + "," + std::to_string(std::get<1>(g)));
      }
    }
    for (const auto& g : sb) {
      if (!sa.contains(g)) {
        line("  only second: " + std::to_string(std::get<0>(g)) + "," + std::to_string(std::get<1>(g)));
      }
    }
  }
  if (a.cuts != b.cuts) {
    line("cuts differ");
  }
  if (a.capacity != b.capacity) {
    line("capacity differs");
  }
  if (a.beside != b.beside) {
    line("beside differs");
  }
  if (a.chambers != b.chambers) {
    line("chambers " + std::to_string(a.chambers) + " vs " + std::to_string(b.chambers));
  }
  if (a.triangles != b.triangles) {
    std::size_t geometry = 0;
    std::set<std::array<double, 6>> ta;
    std::set<std::array<double, 6>> tb;
    for (const auto& t : a.triangles) {
      ta.insert({t[0], t[1], t[2], t[3], t[4], t[5]});
    }
    for (const auto& t : b.triangles) {
      tb.insert({t[0], t[1], t[2], t[3], t[4], t[5]});
    }
    for (const auto& t : ta) {
      geometry += tb.contains(t) ? 0 : 1;
    }
    for (const auto& t : tb) {
      geometry += ta.contains(t) ? 0 : 1;
    }
    line("free triangles " + std::to_string(a.triangles.size()) + " vs " +
         std::to_string(b.triangles.size()) + ", " + std::to_string(geometry) +
         " differ in their corners");
    int shown = 0;
    for (const auto& t : ta) {
      if (!tb.contains(t) && shown++ < 4) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "  only first: (%.6f,%.6f) (%.6f,%.6f) (%.6f,%.6f)", t[1], t[0], t[3], t[2], t[5], t[4]);
        line(buf);
      }
    }
    shown = 0;
    for (const auto& t : tb) {
      if (!ta.contains(t) && shown++ < 4) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "  only second: (%.6f,%.6f) (%.6f,%.6f) (%.6f,%.6f)", t[1], t[0], t[3], t[2], t[5], t[4]);
        line(buf);
      }
    }
  }
  if (a.of != b.of) {
    line("cells differ");
  }
  return out;
}

#define EXPECT_SAME(a, b) \
  do { \
    const auto& sa_ = (a); \
    const auto& sb_ = (b); \
    EXPECT_TRUE(sa_ == sb_) << diffOf(sa_, sb_); \
  } while (false)

Snapshot snapshotOf(const CdtWorld& world, const CdtWorld::State& state,
                    const bool cells = true) {
  Snapshot s;
  const auto& r = world.result(state);
  for (const auto& gate : r.gates) {
    s.gates.emplace_back(gate.first, gate.second, gate.saddle);
  }
  s.cuts = r.cuts;
  s.capacity = r.capacity;
  s.beside = r.beside;
  s.chambers = r.chambers;
  s.triangles = world.canonicalTriangles(state);
  if (cells) {
    Chambers chambers;
    world.chambers(state, chambers);
    s.of = std::move(chambers.of);
  }
  return s;
}

/// A scene with every kind of thing a step brings: a border and two static
/// blocks, a static pad, a port run of two flanks (one group, not static),
/// and as objects two thick lines that cross each other, run over the pad
/// and pass the run, a pad of their own, a short stub, and a pad with its
/// lead: a static wall and the two flanks of a run, one object.
struct Mixed {
  Raster raster{220, 180};
  std::vector<std::vector<std::uint32_t>> objects;

  Mixed() {
    auto& r = raster;
    const auto border = r.wall({.group = 0, .fixed = true});
    for (std::uint32_t x = 0; x < r.width; ++x) {
      r.cell(x, 0, border);
      r.cell(x, r.height - 1, border);
    }
    for (std::uint32_t y = 0; y < r.height; ++y) {
      r.cell(0, y, border);
      r.cell(r.width - 1, y, border);
    }
    const auto block = r.wall({.group = 1, .fixed = true});
    r.rect(block, 30, 30, 70, 60);
    const auto other = r.wall({.group = 2, .fixed = true});
    r.rect(other, 140, 110, 190, 150);
    const auto pad = r.wall({.group = 3, .fixed = true});
    r.rect(pad, 95, 20, 115, 40);
    const auto left = r.wall({.group = 4, .fixed = false});
    const auto right = r.wall({.group = 4, .fixed = false});
    r.rect(left, 160, 30, 161, 70);
    r.rect(right, 172, 30, 173, 70);
    r.rect(left, 160, 70, 173, 71);
    // The objects.
    const auto feedA = r.wall({.group = 6, .fixed = false});
    r.line(feedA, 20.3, 100.7, 200.2, 80.1, 4.5);
    const auto feedB = r.wall({.group = 7, .fixed = false});
    r.line(feedB, 80.4, 170.0, 125.6, 12.3, 4.5);
    const auto padB = r.wall({.group = 8, .fixed = true});
    r.rect(padB, 40, 120, 62, 142);
    const auto stub = r.wall({.group = 9, .fixed = false});
    r.line(stub, 130.0, 60.0, 150.0, 60.0, 1.5);
    const auto padC = r.wall({.group = 10, .fixed = true});
    r.rect(padC, 110, 140, 126, 156);
    const auto leadLeft = r.wall({.group = 11, .fixed = false});
    const auto leadRight = r.wall({.group = 11, .fixed = false});
    r.rect(leadLeft, 114, 110, 115, 139);
    r.rect(leadRight, 121, 110, 122, 139);
    objects = {{feedA}, {feedB}, {padB}, {stub}, {padC, leadLeft, leadRight}};
  }
  [[nodiscard]] std::set<std::uint32_t> objectLabels() const {
    std::set<std::uint32_t> out;
    for (const auto& labels : objects) {
      out.insert(labels.begin(), labels.end());
    }
    return out;
  }
};

CdtGateOptions mixedOptions() {
  return {.clearance = 6.0, .widest = 6, .margin = 1.06, .marginFrom = 3};
}

struct MixedWorld {
  Mixed scene;
  CdtWorld world;
  std::vector<std::uint32_t> ids;

  static CdtWorld make(const Mixed& scene) {
    const auto [mask, owner] = scene.raster.without(scene.objectLabels());
    return {scene.raster.width, scene.raster.height, mask, owner,
            scene.raster.walls, 0, mixedOptions()};
  }
  MixedWorld() : world(make(scene)) {
    for (const auto& labels : scene.objects) {
      ids.push_back(world.add(cdtObjectOf(1000U + labels.front(), scene.raster.mask,
                                          scene.raster.owner, scene.raster.walls,
                                          labels)));
    }
  }
};

TEST(CdtWorld, AFeedlineMakesAGateToTheStaticWall) {
  // A border, a static block at y 10 to 20 and, as the object, a bar at y
  // 33 to 36 that moves: the one gap narrow enough lies between the two.
  Raster r(80, 60);
  const auto border = r.wall({.group = 0, .fixed = true});
  const auto block = r.wall({.group = 1, .fixed = true});
  const auto bar = r.wall({.group = 2, .fixed = false});
  for (std::uint32_t y = 0; y < r.height; ++y) {
    for (std::uint32_t x = 0; x < r.width; ++x) {
      if (x == 0 || y == 0 || x == r.width - 1 || y == r.height - 1) {
        r.cell(x, y, border);
      }
    }
  }
  r.rect(block, 20, 10, 40, 20);
  r.rect(bar, 20, 33, 40, 36);
  const CdtGateOptions options{.clearance = 5.0, .widest = 3, .margin = 1.06, .marginFrom = 3};
  const auto [mask, owner] = r.without({bar});
  CdtWorld world(r.width, r.height, mask, owner, r.walls, 0, options);
  EXPECT_TRUE(world.result(*world.base()).gates.empty());
  const std::array labels{bar};
  const auto id = world.add(cdtObjectOf(7, r.mask, r.owner, r.walls, labels));
  const std::array ids{id};
  const auto state = world.with(world.base(), ids);
  const auto& result = world.result(*state);
  ASSERT_EQ(result.gates.size(), 1U);
  const auto& gate = result.gates.front();
  const auto a = r.owner[gate.first];
  const auto b = r.owner[gate.second];
  EXPECT_TRUE((a == block && b == bar) || (a == bar && b == block));
  const auto dy = std::abs(static_cast<long>(gate.first / r.width) -
                           static_cast<long>(gate.second / r.width));
  const auto dx = std::abs(static_cast<long>(gate.first % r.width) -
                           static_cast<long>(gate.second % r.width));
  EXPECT_EQ(dy, 13);
  EXPECT_LE(dx, 1);
  EXPECT_EQ(result.cuts.front(), 1U);
  EXPECT_EQ(result.capacity.front(), 2U);
  // The free space runs round the gate: one chamber on both of its sides.
  ASSERT_EQ(result.beside.size(), 1U);
  EXPECT_EQ(result.beside.front()[1], NO_CHAMBER);
  // As `cdtGates` finds it on the whole raster.
  CdtGateOptions canonical = options;
  canonical.canonical = true;
  const auto whole = cdtGates(r.width, r.height, r.mask, r.owner, r.walls, 0, canonical);
  ASSERT_EQ(whole.size(), 1U);
  EXPECT_EQ(std::minmax(whole.front().first, whole.front().second),
            std::minmax(gate.first, gate.second));
}

TEST(CdtWorld, AGateCutsAPocketIntoAChamberOfItsOwn) {
  // A static pocket open to the right, its mouth narrowed by the object: the
  // gate across the mouth parts the pocket from the rest, as `chambersOf`
  // parts them.
  Raster r(120, 80);
  const auto border = r.wall({.group = 0, .fixed = true});
  const auto pocket = r.wall({.group = 1, .fixed = true});
  const auto plug = r.wall({.group = 2, .fixed = false});
  for (std::uint32_t y = 0; y < r.height; ++y) {
    for (std::uint32_t x = 0; x < r.width; ++x) {
      if (x == 0 || y == 0 || x == r.width - 1 || y == r.height - 1) {
        r.cell(x, y, border);
      }
    }
  }
  r.rect(pocket, 20, 20, 70, 23);
  r.rect(pocket, 20, 50, 70, 53);
  r.rect(pocket, 20, 20, 23, 53);
  r.rect(plug, 60, 24, 70, 41);
  const CdtGateOptions options{.clearance = 5.0, .widest = 4, .margin = 1.06, .marginFrom = 3};
  const auto [mask, owner] = r.without({plug});
  CdtWorld world(r.width, r.height, mask, owner, r.walls, 0, options);
  const std::array labels{plug};
  const std::array ids{world.add(cdtObjectOf(3, r.mask, r.owner, r.walls, labels))};
  const auto state = world.with(world.base(), ids);
  const auto& result = world.result(*state);
  ASSERT_FALSE(result.gates.empty());
  Chambers mine;
  world.chambers(*state, mine);
  const auto grid = GridMetrics::fit({.minX = 0.0, .minY = 0.0,
                                      .maxX = static_cast<double>(r.width - 1),
                                      .maxY = static_cast<double>(r.height - 1)},
                                     r.width, r.height);
  const auto theirs = chambersOf(r.mask, result.gates, grid);
  EXPECT_EQ(mine.count, theirs.count);
  // The same chambers beside every gate, the numbers mapped by the cells
  // both have.
  std::map<std::uint32_t, std::uint32_t> toTheirs;
  for (std::size_t c = 0; c < mine.of.size(); ++c) {
    if (mine.of[c] != NO_CHAMBER && theirs.of[c] != NO_CHAMBER) {
      toTheirs.emplace(mine.of[c], theirs.of[c]);
    }
  }
  ASSERT_EQ(mine.beside.size(), theirs.beside.size());
  for (std::size_t g = 0; g < mine.beside.size(); ++g) {
    std::vector<std::uint32_t> mapped;
    for (const auto c : mine.beside[g]) {
      mapped.push_back(toTheirs.at(c));
    }
    std::ranges::sort(mapped);
    EXPECT_EQ(mapped, theirs.beside[g]) << "gate " << g;
  }
  EXPECT_TRUE(std::ranges::any_of(mine.beside, [](const auto& b) { return b.size() == 2; }));
  // The cell in the pocket and the cell outside it lie in two chambers.
  const auto inside = mine.of[(30 * r.width) + 40];
  const auto outside = mine.of[(70 * r.width) + 100];
  EXPECT_NE(inside, NO_CHAMBER);
  EXPECT_NE(outside, NO_CHAMBER);
  EXPECT_NE(inside, outside);
  EXPECT_EQ(world.chamberAt(*state, 40, 30), inside);
  EXPECT_EQ(world.chamberAt(*state, 100, 70), outside);
  EXPECT_EQ(world.chamberAt(*state, 21, 21), NO_CHAMBER);
}

TEST(CdtWorld, AGateThatEndsOnAFootCutsWhereverTheSceneLies) {
  // The pocket of the test before, at many places of the grid: the gate
  // across the mouth ends on a foot laid a hair off the static wall, and
  // the sliver between the two must not join the pocket to the rest,
  // whichever way the move of the points takes the wall.
  std::size_t parted = 0;
  std::size_t scenes = 0;
  for (std::int64_t dy = 0; dy < 6; ++dy) {
    for (std::int64_t dx = 0; dx < 6; ++dx) {
      Raster r(130, 90);
      const auto border = r.wall({.group = 0, .fixed = true});
      const auto pocket = r.wall({.group = 1, .fixed = true});
      const auto plug = r.wall({.group = 2, .fixed = false});
      for (std::uint32_t y = 0; y < r.height; ++y) {
        for (std::uint32_t x = 0; x < r.width; ++x) {
          if (x == 0 || y == 0 || x == r.width - 1 || y == r.height - 1) {
            r.cell(x, y, border);
          }
        }
      }
      r.rect(pocket, 20 + dx, 20 + dy, 70 + dx, 23 + dy);
      r.rect(pocket, 20 + dx, 50 + dy, 70 + dx, 53 + dy);
      r.rect(pocket, 20 + dx, 20 + dy, 23 + dx, 53 + dy);
      r.rect(plug, 60 + dx, 24 + dy, 70 + dx, 41 + dy);
      const CdtGateOptions options{.clearance = 5.0, .widest = 4, .margin = 1.06, .marginFrom = 3};
      const auto [mask, owner] = r.without({plug});
      CdtWorld world(r.width, r.height, mask, owner, r.walls, 0, options);
      const std::array labels{plug};
      const std::array ids{world.add(cdtObjectOf(3, r.mask, r.owner, r.walls, labels))};
      const auto state = world.with(world.base(), ids);
      const auto inside = world.chamberAt(*state, static_cast<std::uint32_t>(40 + dx),
                                          static_cast<std::uint32_t>(30 + dy));
      const auto outside = world.chamberAt(*state, static_cast<std::uint32_t>(100 + dx),
                                           static_cast<std::uint32_t>(70 + dy));
      ++scenes;
      parted += inside != NO_CHAMBER && outside != NO_CHAMBER && inside != outside ? 1 : 0;
      EXPECT_SAME(snapshotOf(world, *state), snapshotOf(world, *world.fromScratch(ids)));
    }
  }
  EXPECT_EQ(parted, scenes);
}

TEST(CdtWorld, AGateOfWallsThatDoNotCutPartsNothing) {
  // The pocket of the test before, its plug a wall whose gates do not cut
  // the chambers (`couplerCuts` of the router): the gate is there, the
  // pocket is one chamber with the rest.
  Raster r(120, 80);
  const auto border = r.wall({.group = 0, .fixed = true});
  const auto pocket = r.wall({.group = 1, .fixed = true});
  const auto plug = r.wall({.group = 2, .fixed = false});
  for (std::uint32_t y = 0; y < r.height; ++y) {
    for (std::uint32_t x = 0; x < r.width; ++x) {
      if (x == 0 || y == 0 || x == r.width - 1 || y == r.height - 1) {
        r.cell(x, y, border);
      }
    }
  }
  r.rect(pocket, 20, 20, 70, 23);
  r.rect(pocket, 20, 50, 70, 53);
  r.rect(pocket, 20, 20, 23, 53);
  r.rect(plug, 60, 24, 70, 41);
  const CdtGateOptions options{.clearance = 5.0, .widest = 4, .margin = 1.06, .marginFrom = 3};
  const auto [mask, owner] = r.without({plug});
  const std::vector<std::uint8_t> cuts{1, 0, 0};
  CdtWorld world(r.width, r.height, mask, owner, r.walls, 0, options, cuts);
  const std::array labels{plug};
  const std::array ids{world.add(cdtObjectOf(3, r.mask, r.owner, r.walls, labels, cuts))};
  const auto state = world.with(world.base(), ids);
  const auto& result = world.result(*state);
  ASSERT_FALSE(result.gates.empty());
  EXPECT_TRUE(std::ranges::none_of(result.cuts, [](const auto c) { return c != 0; }));
  EXPECT_EQ(world.chamberAt(*state, 40, 30), world.chamberAt(*state, 100, 70));
  for (const auto& beside : result.beside) {
    EXPECT_EQ(beside[1], NO_CHAMBER);
  }
}

TEST(CdtWorld, InsertedOneByOneIsInsertedAllAtOnce) {
  MixedWorld mixed;
  auto& world = mixed.world;
  const auto& ids = mixed.ids;
  ASSERT_EQ(ids.size(), 5U);
  const auto scratch = world.fromScratch(ids);
  const auto expected = snapshotOf(world, *scratch);
  EXPECT_FALSE(expected.gates.empty());
  EXPECT_GT(expected.chambers, 1U);
  // No gate joins the two flanks of the lead, one obstacle.
  const auto& lead = mixed.scene.objects.back();
  for (const auto& [first, second, saddle] : expected.gates) {
    const auto a = mixed.scene.raster.owner[first];
    const auto b = mixed.scene.raster.owner[second];
    EXPECT_FALSE(a != b && std::ranges::find(lead, a) != lead.end() &&
                 std::ranges::find(lead, b) != lead.end() && a != lead.front() &&
                 b != lead.front());
  }
  // One by one, forwards and backwards.
  auto state = world.base();
  for (const auto id : ids) {
    const std::array one{id};
    state = world.with(state, one);
  }
  EXPECT_SAME(snapshotOf(world, *state), expected);
  state = world.base();
  for (auto k = ids.size(); k-- > 0;) {
    const std::array one{ids[k]};
    state = world.with(state, one);
  }
  EXPECT_SAME(snapshotOf(world, *state), expected);
  // Every prefix, built by one step from the one before, is that prefix
  // from scratch.
  state = world.base();
  for (std::size_t k = 0; k < ids.size(); ++k) {
    const std::array one{ids[k]};
    state = world.with(state, one);
    const std::vector prefix(ids.begin(), ids.begin() + static_cast<std::ptrdiff_t>(k + 1));
    SCOPED_TRACE("prefix " + std::to_string(k + 1));
    EXPECT_SAME(snapshotOf(world, *state), snapshotOf(world, *world.fromScratch(prefix)));
  }
}

TEST(CdtWorld, TheOrderOfTheObjectsDoesNotMatter) {
  MixedWorld mixed;
  auto& world = mixed.world;
  auto order = mixed.ids;
  std::ranges::sort(order);
  const auto expected = snapshotOf(world, *world.fromScratch(order));
  std::size_t tried = 0;
  do {
    auto state = world.base();
    // Two at once, then one by one.
    const std::array two{order[0], order[1]};
    state = world.with(state, two);
    for (std::size_t k = 2; k < order.size(); ++k) {
      const std::array one{order[k]};
      state = world.with(state, one);
    }
    SCOPED_TRACE("order " + std::to_string(tried));
    auto e = expected;
    if (tried % 6 != 0) {
      e.of.clear();
    }
    EXPECT_SAME(snapshotOf(world, *state, tried % 6 == 0), e);
    ++tried;
  } while (std::ranges::next_permutation(order).found);
  EXPECT_EQ(tried, 120U);
}

TEST(CdtWorld, TheCacheDoesNotChangeTheResult) {
  // Every subset of the objects, asked of two worlds whose caches have seen
  // different states: the same as from scratch.
  MixedWorld one;
  MixedWorld two;
  const auto n = one.ids.size();
  std::vector<std::uint32_t> masks(std::size_t{1} << n);
  std::iota(masks.begin(), masks.end(), 0U);
  const auto subset = [&](const std::vector<std::uint32_t>& ids, const std::uint32_t mask) {
    std::vector<std::uint32_t> out;
    for (std::size_t k = 0; k < n; ++k) {
      if ((mask >> k) & 1U) {
        out.push_back(ids[k]);
      }
    }
    return out;
  };
  // The first world asks the largest sets first, the second the smallest.
  std::vector<std::uint32_t> down = masks;
  std::ranges::reverse(down);
  for (const auto mask : down) {
    const auto ids = subset(one.ids, mask);
    const auto cached = one.world.state(ids);
    SCOPED_TRACE("mask " + std::to_string(mask));
    EXPECT_SAME(snapshotOf(one.world, *cached, false),
                snapshotOf(one.world, *one.world.fromScratch(ids), false));
  }
  for (const auto mask : masks) {
    const auto ids = subset(two.ids, mask);
    const auto cached = two.world.state(ids);
    SCOPED_TRACE("mask " + std::to_string(mask));
    EXPECT_SAME(snapshotOf(two.world, *cached, false),
                snapshotOf(one.world, *one.world.state(subset(one.ids, mask)), false));
  }
  EXPECT_GT(two.world.cacheStats().extended, 0U);
}

TEST(CdtWorld, BlocksOnALatticeGiveOneStateWhateverTheWay) {
  // Blocks of one size on a lattice, each an object of its own: the corners
  // of four blocks around a gap lie on one circle, the case where the order
  // of insertion would pick the diagonals without the perturbation. Random
  // sets of blocks, through the cache in random order, against scratch.
  constexpr std::uint32_t SIDE = 160;
  Raster r(SIDE, SIDE);
  const auto border = r.wall({.group = 0, .fixed = true});
  for (std::uint32_t k = 0; k < SIDE; ++k) {
    r.cell(k, 0, border);
    r.cell(k, SIDE - 1, border);
    r.cell(0, k, border);
    r.cell(SIDE - 1, k, border);
  }
  std::vector<std::uint32_t> blocks;
  for (std::uint32_t by = 0; by < 5; ++by) {
    for (std::uint32_t bx = 0; bx < 5; ++bx) {
      const auto label = r.wall({.group = static_cast<std::uint32_t>(r.walls.size()), .fixed = false});
      r.rect(label, 12 + (bx * 30), 12 + (by * 30), 29 + (bx * 30), 29 + (by * 30));
      blocks.push_back(label);
    }
  }
  const CdtGateOptions options{.clearance = 5.0, .widest = 4, .margin = 1.06, .marginFrom = 3};
  const auto [mask, owner] = r.without({blocks.begin(), blocks.end()});
  CdtWorld world(r.width, r.height, mask, owner, r.walls, 0, options, {}, 4);
  std::vector<std::uint32_t> ids;
  for (const auto label : blocks) {
    const std::array labels{label};
    ids.push_back(world.add(cdtObjectOf(label, r.mask, r.owner, r.walls, labels)));
  }
  std::uint64_t seed = 12345;
  const auto next = [&] {
    seed = (seed * 6364136223846793005ULL) + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(seed >> 33U);
  };
  std::vector<std::uint32_t> grown;
  for (int round = 0; round < 40; ++round) {
    // A fresh set every fifth round, else the last one with a block more, as
    // a chain search grows a prefix.
    if (round % 5 == 0) {
      grown.clear();
      for (const auto id : ids) {
        if (next() % 3 == 0) {
          grown.push_back(id);
        }
      }
    } else {
      const auto id = ids[next() % ids.size()];
      if (std::ranges::find(grown, id) == grown.end()) {
        grown.push_back(id);
      }
    }
    auto set = grown;
    // Shuffled: the order the cache is asked in does not matter either.
    for (auto k = set.size(); k > 1; --k) {
      std::swap(set[k - 1], set[next() % k]);
    }
    SCOPED_TRACE("round " + std::to_string(round));
    EXPECT_SAME(snapshotOf(world, *world.state(set), round % 8 == 0),
                snapshotOf(world, *world.fromScratch(set), round % 8 == 0));
  }
  EXPECT_GT(world.cacheStats().extended, 0U);
}

TEST(CdtWorld, ARasterNamesItsObjectsByKey) {
  // `state` with the raster: the objects are added the first time their
  // keys come, and the labels they carry may differ from call to call.
  MixedWorld mixed;
  const auto& scene = mixed.scene;
  const auto [mask, owner] = scene.raster.without(scene.objectLabels());
  CdtWorld world(scene.raster.width, scene.raster.height, mask, owner,
                 scene.raster.walls, 0, mixedOptions());
  std::vector<CdtObjectRef> refs;
  for (const auto& labels : scene.objects) {
    refs.push_back({.key = 1000U + labels.front(), .labels = labels});
  }
  const auto state = world.state(scene.raster.mask, scene.raster.owner,
                                 scene.raster.walls, refs);
  EXPECT_EQ(world.objectCount(), 5U);
  EXPECT_SAME(snapshotOf(world, *state), snapshotOf(mixed.world, *mixed.world.fromScratch(mixed.ids)));
  // Again, with the same keys: no object is added, the cache answers.
  const auto again = world.state(scene.raster.mask, scene.raster.owner,
                                 scene.raster.walls, refs);
  EXPECT_EQ(world.objectCount(), 5U);
  EXPECT_EQ(again, state);
}

TEST(CdtWorld, TheGatesAreThoseOfCdtGatesOnTheWholeRaster) {
  // On the mixed scene, every gate of `cdtGates` (canonical) has one of the
  // world within two cells at both ends, and the other way round, but for a
  // few where outlines overlap instead of being joined.
  MixedWorld mixed;
  const auto& r = mixed.scene.raster;
  const auto state = mixed.world.fromScratch(mixed.ids);
  const auto& mine = mixed.world.result(*state).gates;
  auto options = mixedOptions();
  options.canonical = true;
  const auto theirs = cdtGates(r.width, r.height, r.mask, r.owner, r.walls, 0, options);
  const auto near = [&](const Bottleneck& a, const Bottleneck& b) {
    const auto d = [&](const std::size_t p, const std::size_t q) {
      return std::hypot(static_cast<double>(p % r.width) - static_cast<double>(q % r.width),
                        static_cast<double>(p / r.width) - static_cast<double>(q / r.width));
    };
    return std::min(std::max(d(a.first, b.first), d(a.second, b.second)),
                    std::max(d(a.first, b.second), d(a.second, b.first))) <= 2.0;
  };
  std::size_t found = 0;
  for (const auto& gate : theirs) {
    found += std::ranges::any_of(mine, [&](const Bottleneck& g) { return near(g, gate); }) ? 1 : 0;
  }
  std::printf("mixed scene: %zu gates of cdtGates, %zu of the world, %zu matched\n",
              theirs.size(), mine.size(), found);
  EXPECT_GE(static_cast<double>(found), 0.8 * static_cast<double>(theirs.size()));
}

// ---------------------------------------------------------------- touching

/// A world whose objects touch one another or the static walls: every
/// subset of them through the cache, every subset from scratch, and the
/// whole set inserted one by one in several orders, all the same, and none
/// may throw.
void expectOneStateWhateverTheWay(const Raster& r,
                                  const std::vector<std::vector<std::uint32_t>>& objects,
                                  const CdtGateOptions& options, const std::size_t orders = 24) {
  std::set<std::uint32_t> moving;
  for (const auto& labels : objects) {
    moving.insert(labels.begin(), labels.end());
  }
  const auto [mask, owner] = r.without(moving);
  CdtWorld world(r.width, r.height, mask, owner, r.walls, 0, options, {}, 6);
  std::vector<std::uint32_t> ids;
  for (std::size_t k = 0; k < objects.size(); ++k) {
    ids.push_back(world.add(cdtObjectOf(500U + k, r.mask, r.owner, r.walls, objects[k])));
  }
  std::uint64_t seed = 4242;
  const auto next = [&] {
    seed = (seed * 6364136223846793005ULL) + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(seed >> 33U);
  };
  // The subsets: all of them for a few objects, else random ones.
  std::vector<std::vector<std::uint32_t>> sets;
  if (ids.size() <= 6) {
    for (std::uint32_t m = 0; m < (1U << ids.size()); ++m) {
      std::vector<std::uint32_t> set;
      for (std::size_t k = 0; k < ids.size(); ++k) {
        if (((m >> k) & 1U) != 0) {
          set.push_back(ids[k]);
        }
      }
      sets.push_back(set);
    }
  } else {
    for (int round = 0; round < 40; ++round) {
      std::vector<std::uint32_t> set;
      for (const auto id : ids) {
        if (next() % 3 != 0) {
          set.push_back(id);
        }
      }
      sets.push_back(set);
    }
    sets.push_back(ids);
  }
  for (std::size_t k = 0; k < sets.size(); ++k) {
    auto set = sets[k];
    for (auto j = set.size(); j > 1; --j) {
      std::swap(set[j - 1], set[next() % j]);
    }
    SCOPED_TRACE("set " + std::to_string(k));
    CdtWorld::StatePtr cached;
    CdtWorld::StatePtr whole;
    ASSERT_NO_THROW(cached = world.state(set));
    ASSERT_NO_THROW(whole = world.fromScratch(set));
    EXPECT_SAME(snapshotOf(world, *cached, k % 7 == 0), snapshotOf(world, *whole, k % 7 == 0));
  }
  // The whole set, one by one.
  const auto expected = snapshotOf(world, *world.fromScratch(ids), false);
  auto order = ids;
  for (std::size_t round = 0; round < orders; ++round) {
    for (auto j = order.size(); j > 1; --j) {
      std::swap(order[j - 1], order[next() % j]);
    }
    auto state = world.base();
    for (const auto id : order) {
      const std::array one{id};
      ASSERT_NO_THROW(state = world.with(state, one));
    }
    SCOPED_TRACE("order " + std::to_string(round));
    EXPECT_SAME(snapshotOf(world, *state, false), expected);
  }
}

/// A raster with a border of label 0.
Raster bordered(const std::uint32_t width, const std::uint32_t height) {
  Raster r(width, height);
  const auto border = r.wall({.group = 0, .fixed = true});
  for (std::uint32_t x = 0; x < width; ++x) {
    r.cell(x, 0, border);
    r.cell(x, height - 1, border);
  }
  for (std::uint32_t y = 0; y < height; ++y) {
    r.cell(0, y, border);
    r.cell(width - 1, y, border);
  }
  return r;
}

std::uint32_t newWall(Raster& r, const bool fixed) {
  return r.wall({.group = static_cast<std::uint32_t>(r.walls.size()), .fixed = fixed});
}

// Clearance 7: a piece of 20 cells is cut into three, at points off the
// grid, so that the cuts of two outlines along one line do not meet.
const CdtGateOptions TOUCHING{.clearance = 7.0, .widest = 4, .margin = 1.06, .marginFrom = 3};

TEST(CdtWorld, TwoObjectsThatShareAnEdge) {
  auto r = bordered(100, 80);
  const auto a = newWall(r, false);
  const auto b = newWall(r, true);
  r.rect(a, 20, 20, 39, 39);
  r.rect(b, 40, 20, 59, 39);
  // And a static block the two face, for gates.
  r.rect(newWall(r, true), 20, 50, 59, 60);
  expectOneStateWhateverTheWay(r, {{a}, {b}}, TOUCHING);
}

TEST(CdtWorld, TwoObjectsThatShareAPartOfAnEdge) {
  // The second block slid along the side of the first, a cell at a time: a
  // corner of each lies on a piece of the other. A third under both, slid
  // the other way: three outlines along one row.
  for (std::int64_t shift = 1; shift < 20; ++shift) {
    auto r = bordered(100, 100);
    const auto a = newWall(r, false);
    const auto b = newWall(r, false);
    const auto c = newWall(r, true);
    r.rect(a, 20, 20, 39, 39);
    r.rect(b, 40, 20 + shift, 59, 39 + shift);
    r.rect(c, 40 - shift, 40 + shift, 59 - (shift / 2), 51 + shift);
    SCOPED_TRACE("shift " + std::to_string(shift));
    expectOneStateWhateverTheWay(r, {{a}, {b}, {c}}, TOUCHING, 4);
  }
}

TEST(CdtWorld, AnObjectThatStandsOnAnotherInAT) {
  // A bar on the long side of another, and one at its end: the corners of
  // the one lie inside a piece of the other.
  auto r = bordered(110, 90);
  const auto base = newWall(r, false);
  const auto stem = newWall(r, false);
  const auto end = newWall(r, true);
  const auto cap = newWall(r, false);
  r.rect(base, 15, 40, 74, 47);
  r.rect(stem, 37, 48, 44, 75);
  r.rect(end, 75, 30, 82, 57);
  r.rect(cap, 31, 20, 50, 39);
  expectOneStateWhateverTheWay(r, {{base}, {stem}, {end}, {cap}}, TOUCHING);
}

TEST(CdtWorld, ObjectsThatTouchAtACorner) {
  auto r = bordered(110, 110);
  const auto a = newWall(r, false);
  const auto b = newWall(r, false);
  const auto c = newWall(r, true);
  const auto d = newWall(r, false);
  r.rect(a, 20, 20, 39, 39);
  r.rect(b, 40, 40, 59, 59);
  r.rect(c, 60, 20, 79, 39);  // touches b at (60, 40)
  r.rect(d, 20, 60, 39, 79);  // touches b at (40, 60)
  expectOneStateWhateverTheWay(r, {{a}, {b}, {c}, {d}}, TOUCHING);
}

TEST(CdtWorld, ObjectsThatTouchTheStaticWalls) {
  // Objects along a static block, on it in a T, at its corner, and one that
  // overlaps it and shares the line of its side; slid along it.
  for (std::int64_t shift = 0; shift < 12; ++shift) {
    auto r = bordered(130, 110);
    const auto block = newWall(r, true);
    r.rect(block, 30, 30, 89, 49);
    const auto top = newWall(r, false);
    const auto side = newWall(r, false);
    const auto corner = newWall(r, true);
    const auto over = newWall(r, false);
    r.rect(top, 33 + (3 * shift), 18, 50 + (3 * shift), 29);
    r.rect(side, 90, 22 + shift, 101, 47 + shift);
    r.rect(corner, 18, 50, 29, 61);
    r.rect(over, 45 + (2 * shift), 40, 60 + (2 * shift), 70);
    SCOPED_TRACE("shift " + std::to_string(shift));
    expectOneStateWhateverTheWay(r, {{top}, {side}, {corner}, {over}}, TOUCHING, 4);
  }
}

TEST(CdtWorld, ObjectsThatTouchAlongADiagonal) {
  // Bands of 45 and of 26.6 degrees side by side: their outlines meet on a
  // staircase, simplified by each on its own.
  for (std::int64_t width = 4; width < 10; ++width) {
  auto r = bordered(160, 160);
  const auto a = newWall(r, false);
  const auto b = newWall(r, false);
  const auto c = newWall(r, false);
  const auto d = newWall(r, false);
  for (std::int64_t y = 10; y < 70; ++y) {
    for (std::int64_t x = 10; x < 150; ++x) {
      const auto diagonal = x - y;
      if (diagonal >= 10 && diagonal < 10 + width) {
        r.cell(x, y, a);
      } else if (diagonal >= 10 + width && diagonal < 17 + width) {
        r.cell(x, y, b);
      }
    }
  }
  for (std::int64_t y = 80; y < 150; ++y) {
    for (std::int64_t x = 10; x < 150; ++x) {
      const auto slope = (2 * (y - 80)) - (x - 10);
      if (slope >= -20 && slope < -20 + (2 * width)) {
        r.cell(x, y, c);
      } else if (slope >= -20 + (2 * width) && slope < 6) {
        r.cell(x, y, d);
      }
    }
  }
  SCOPED_TRACE("width " + std::to_string(width));
  expectOneStateWhateverTheWay(r, {{a}, {b}, {c}, {d}}, TOUCHING, 4);
  }
}

TEST(CdtWorld, ABrickWallOfObjects) {
  // Rows of bricks of random sizes laid against one another, some static:
  // every kind of contact along the rows and between them. Random sets of
  // the bricks, through the cache, against scratch.
  auto r = bordered(220, 200);
  std::uint64_t seed = 99;
  const auto next = [&](const std::uint32_t below) {
    seed = (seed * 6364136223846793005ULL) + 1442695040888963407ULL;
    return static_cast<std::int64_t>((seed >> 33U) % below);
  };
  std::vector<std::vector<std::uint32_t>> objects;
  std::int64_t y = 15;
  while (y < 170) {
    const auto rowHeight = 5 + next(9);
    std::int64_t x = 12 + next(10);
    while (x < 190) {
      const auto brick = 6 + next(18);
      const bool fixed = next(5) == 0;
      const auto label = newWall(r, fixed);
      r.rect(label, x, y, std::min<std::int64_t>(x + brick - 1, 205), y + rowHeight - 1);
      if (!(fixed && next(2) == 0)) {
        objects.push_back({label});
      }
      // Bricks touch, or leave a gap of a few cells now and then.
      x += brick + (next(4) == 0 ? 3 + next(6) : 0);
    }
    // Rows touch, or leave a channel.
    y += rowHeight + (next(3) == 0 ? 6 + next(8) : 0);
  }
  ASSERT_GT(objects.size(), 20U);
  expectOneStateWhateverTheWay(r, objects, TOUCHING, 4);
}

// ---------------------------------------------------------------- benchmark

/// The raster of MQT_SCPD_CDT_BENCH (format of `test_cdt_gates.cpp`).
struct Bench {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  BitGrid mask;
  std::vector<std::uint32_t> owner;
  std::vector<CdtWall> walls;
};

bool loadBench(Bench& bench) {
  const char* path = std::getenv("MQT_SCPD_CDT_BENCH");
  if (path == nullptr) {
    return false;
  }
  std::ifstream in(path, std::ios::binary);
  std::array<std::uint32_t, 3> head{};
  in.read(reinterpret_cast<char*>(head.data()), sizeof(head));
  bench.width = head[0];
  bench.height = head[1];
  const auto cells = static_cast<std::size_t>(head[0]) * head[1];
  std::vector<std::uint8_t> mask(cells);
  in.read(reinterpret_cast<char*>(mask.data()), static_cast<std::streamsize>(cells));
  bench.owner.resize(cells);
  in.read(reinterpret_cast<char*>(bench.owner.data()),
          static_cast<std::streamsize>(cells * sizeof(std::uint32_t)));
  std::vector<std::uint32_t> raw(static_cast<std::size_t>(head[2]) * 3);
  in.read(reinterpret_cast<char*>(raw.data()),
          static_cast<std::streamsize>(raw.size() * sizeof(std::uint32_t)));
  for (std::size_t k = 0; k < head[2]; ++k) {
    bench.walls.push_back({.group = raw[(k * 3)],
                           .fixed = raw[(k * 3) + 1] != 0,
                           .target = raw[(k * 3) + 2] != 0});
  }
  bench.mask = BitGrid(bench.width, bench.height);
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (mask[cell] != 0) {
      bench.mask.set(cell);
    }
  }
  return in.good();
}

double quantile(std::vector<double> values, const double q) {
  if (values.empty()) {
    return 0.0;
  }
  std::ranges::sort(values);
  if (q <= 0.5) {
    return values[(values.size() - 1) / 2];
  }
  const auto at = static_cast<std::size_t>(std::ceil(q * static_cast<double>(values.size()))) - 1;
  return values[std::min(values.size() - 1, at)];
}

/// How two chamber rasters pair: each chamber with the one of the other
/// that shares most of its cells, both ways.
struct Pairing {
  std::size_t refCount = 0;
  std::size_t newCount = 0;
  std::size_t mutual = 0;
  std::size_t cellsAgree = 0;
  std::size_t cellsBoth = 0;
  std::size_t refOnly = 0;
  std::size_t newOnly = 0;
};

Pairing pairChambers(const BitGrid& mask, const std::vector<std::uint32_t>& ref,
                     const std::vector<std::uint32_t>& mine) {
  Pairing p;
  std::unordered_map<std::uint64_t, std::size_t> overlap;
  std::unordered_map<std::uint32_t, std::size_t> refSize;
  std::unordered_map<std::uint32_t, std::size_t> newSize;
  for (std::size_t c = 0; c < ref.size(); ++c) {
    if (mask.test(c)) {
      continue;
    }
    const auto r = ref[c];
    const auto m = mine[c];
    if (r != NO_CHAMBER) {
      ++refSize[r];
    }
    if (m != NO_CHAMBER) {
      ++newSize[m];
    }
    if (r != NO_CHAMBER && m != NO_CHAMBER) {
      ++overlap[(static_cast<std::uint64_t>(r) << 32U) | m];
      ++p.cellsBoth;
    } else if (r != NO_CHAMBER) {
      ++p.refOnly;
    } else if (m != NO_CHAMBER) {
      ++p.newOnly;
    }
  }
  std::unordered_map<std::uint32_t, std::pair<std::uint32_t, std::size_t>> bestOfRef;
  std::unordered_map<std::uint32_t, std::pair<std::uint32_t, std::size_t>> bestOfNew;
  for (const auto& [k, count] : overlap) {
    const auto r = static_cast<std::uint32_t>(k >> 32U);
    const auto m = static_cast<std::uint32_t>(k & 0xffffffffU);
    auto& br = bestOfRef[r];
    if (count > br.second) {
      br = {m, count};
    }
    auto& bn = bestOfNew[m];
    if (count > bn.second) {
      bn = {r, count};
    }
  }
  p.refCount = refSize.size();
  p.newCount = newSize.size();
  for (const auto& [r, best] : bestOfRef) {
    if (bestOfNew[best.first].first == r) {
      ++p.mutual;
      p.cellsAgree += best.second;
    }
  }
  return p;
}

// The world on the raster of MQT_SCPD_CDT_BENCH, its feedlines (the walls
// that move, of 300 cells or more) as the objects; run by hand:
//
// MQT_SCPD_CDT_BENCH=<file> mqt-scpd-grid-test
//   --gtest_also_run_disabled_tests --gtest_filter='CdtWorld.DISABLED_Steps'
//
// The steps are those of the prototype: a chain (prefix k-1 to k, the
// feedlines in label order) and siblings (prefix k to k + j for k in 0, 6,
// 12, 18, 24). Every step five times; the median per step, of those the
// median and the 95th percentile. Every step is checked against the same
// objects from scratch; the last state against `cdtGates` (canonical) and
// `chambersOf` on the whole raster.
TEST(CdtWorld, DISABLED_Steps) {
  Bench bench;
  if (!loadBench(bench)) {
    GTEST_SKIP() << "MQT_SCPD_CDT_BENCH names no file";
  }
  const auto cells = static_cast<std::size_t>(bench.width) * bench.height;
  std::vector<std::size_t> size(bench.walls.size(), 0);
  for (std::size_t c = 0; c < cells; ++c) {
    if (bench.mask.test(c) && bench.owner[c] < bench.walls.size()) {
      ++size[bench.owner[c]];
    }
  }
  std::vector<std::uint32_t> feedlines;
  for (std::uint32_t label = 0; label < bench.walls.size(); ++label) {
    if (!bench.walls[label].fixed && size[label] >= 300) {
      feedlines.push_back(label);
    }
  }
  const std::set<std::uint32_t> moving(feedlines.begin(), feedlines.end());
  BitGrid staticMask(bench.width, bench.height);
  std::vector<std::uint32_t> staticOwner(cells, FREE);
  for (std::size_t c = 0; c < cells; ++c) {
    if (bench.mask.test(c) && !moving.contains(bench.owner[c])) {
      staticMask.set(c);
      staticOwner[c] = bench.owner[c];
    }
  }
  const CdtGateOptions options{.clearance = 19.0, .widest = 10, .canonical = true};
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  CdtWorld world(bench.width, bench.height, staticMask, staticOwner, bench.walls, 0, options);
  const auto t1 = Clock::now();
  std::vector<std::uint32_t> ids;
  for (const auto label : feedlines) {
    const std::array labels{label};
    ids.push_back(world.add(cdtObjectOf(label, bench.mask, bench.owner, bench.walls, labels)));
  }
  const auto t2 = Clock::now();
  const auto& build = world.buildStats();
  const auto& base = world.result(*world.base());
  std::printf("world: %u x %u cells, %zu walls, %zu objects (feedlines); build %.1f ms "
              "(outline %.1f, feet %.1f, base state %.1f), %zu base points, %zu "
              "constraints; base: %zu gates, %u chambers; objects added in %.1f ms "
              "(%.2f ms each, cells found by a scan of the raster each)\n",
              bench.width, bench.height, bench.walls.size(), ids.size(),
              std::chrono::duration<double, std::milli>(t1 - t0).count(),
              build.outline, build.feet, build.base, build.baseVertices,
              build.baseConstraints, base.gates.size(), base.chambers,
              std::chrono::duration<double, std::milli>(t2 - t1).count(),
              std::chrono::duration<double, std::milli>(t2 - t1).count() /
                  static_cast<double>(ids.size()));

  // The prefixes of the chain.
  std::vector<CdtWorld::StatePtr> prefixes{world.base()};
  for (const auto id : ids) {
    const std::array one{id};
    prefixes.push_back(world.with(prefixes.back(), one));
  }
  struct Step {
    std::size_t parent;
    std::size_t object;
    bool chain;
  };
  std::vector<Step> steps;
  for (std::size_t k = 0; k < ids.size(); ++k) {
    steps.push_back({k, k, true});
  }
  for (const std::size_t k : {0UL, 6UL, 12UL, 18UL, 24UL}) {
    for (std::size_t j = k + 1; j < ids.size(); ++j) {
      steps.push_back({k, j, false});
    }
  }
  constexpr int REPEATS = 5;
  std::vector<std::vector<CdtStepStats>> runs(steps.size());
  for (int round = 0; round < REPEATS; ++round) {
    for (std::size_t i = 0; i < steps.size(); ++i) {
      const std::array one{ids[steps[i].object]};
      const auto child = world.with(prefixes[steps[i].parent], one);
      runs[i].push_back(world.statsOf(*child));
    }
  }
  const auto field = [&](double CdtStepStats::*member) {
    std::vector<double> perStep;
    for (const auto& run : runs) {
      std::vector<double> values;
      for (const auto& one : run) {
        values.push_back(one.*member);
      }
      perStep.push_back(quantile(values, 0.5));
    }
    return std::pair{quantile(perStep, 0.5), quantile(perStep, 0.95)};
  };
  std::printf("\nper step (%zu: %zu chain, %zu siblings; median of %d each), ms:\n",
              steps.size(), ids.size(), steps.size() - ids.size(), REPEATS);
  const std::array<std::pair<const char*, double CdtStepStats::*>, 10> fields{{
      {"copy", &CdtStepStats::copy},
      {"insert", &CdtStepStats::insert},
      {"patch", &CdtStepStats::patch},
      {"classify", &CdtStepStats::classify},
      {"candidates", &CdtStepStats::candidates},
      {"basins", &CdtStepStats::basins},
      {"measure", &CdtStepStats::measure},
      {"chambers", &CdtStepStats::chambers},
      {"output", &CdtStepStats::output},
      {"total", &CdtStepStats::total},
  }};
  for (const auto& [name, member] : fields) {
    const auto [median, p95] = field(member);
    std::printf("  %-11s median %7.3f  p95 %7.3f\n", name, median, p95);
  }
  {
    const auto count = [&](std::size_t CdtStepStats::*member) {
      std::vector<double> values;
      for (const auto& run : runs) {
        values.push_back(static_cast<double>(run.front().*member));
      }
      return std::pair{quantile(values, 0.5), quantile(values, 0.95)};
    };
    const std::array<std::pair<const char*, std::size_t CdtStepStats::*>, 7> counts{{
        {"new points", &CdtStepStats::newVertices},
        {"Steiner points", &CdtStepStats::steinerVertices},
        {"changed triangles", &CdtStepStats::changedTriangles},
        {"reclassified triangles", &CdtStepStats::reclassified},
        {"channels recomputed", &CdtStepStats::channels},
        {"candidates in them", &CdtStepStats::channelCandidates},
        {"triangles relabelled", &CdtStepStats::relabelled},
    }};
    for (const auto& [name, member] : counts) {
      const auto [median, p95] = count(member);
      std::printf("  %-24s median %7.0f  p95 %7.0f\n", name, median, p95);
    }
  }

  // Through the cache: the steps in the order a chain search asks them.
  {
    CdtWorld cachedWorld(bench.width, bench.height, staticMask, staticOwner, bench.walls, 0, options);
    for (const auto label : feedlines) {
      const std::array labels{label};
      cachedWorld.add(cdtObjectOf(label, bench.mask, bench.owner, bench.walls, labels));
    }
    std::vector<double> totals;
    for (const auto& step : steps) {
      std::vector<std::uint32_t> set(ids.begin(), ids.begin() + static_cast<std::ptrdiff_t>(step.parent));
      (void)cachedWorld.state(set);
      set.push_back(ids[step.object]);
      const auto a = Clock::now();
      (void)cachedWorld.state(set);
      totals.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a).count());
    }
    const auto& cs = cachedWorld.cacheStats();
    std::printf("\nthrough the cache (8 states): median %.3f, p95 %.3f ms a step; hits %zu, "
                "extended %zu, from the base %zu, objects inserted %zu\n",
                quantile(totals, 0.5), quantile(totals, 0.95), cs.hits, cs.extended,
                cs.fromBase, cs.inserted);
  }

  // Inserted one by one is inserted all at once: every step.
  std::size_t equal = 0;
  double scratchMs = 0.0;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    const std::array one{ids[steps[i].object]};
    const auto child = world.with(prefixes[steps[i].parent], one);
    std::vector<std::uint32_t> set(ids.begin(), ids.begin() + static_cast<std::ptrdiff_t>(steps[i].parent));
    set.push_back(ids[steps[i].object]);
    const auto a = Clock::now();
    const auto whole = world.fromScratch(set);
    scratchMs += std::chrono::duration<double, std::milli>(Clock::now() - a).count();
    const auto mine = snapshotOf(world, *child, i % 10 == 0);
    const auto theirs = snapshotOf(world, *whole, i % 10 == 0);
    if (mine == theirs) {
      ++equal;
    } else {
      std::printf("  step %zu (object %zu, %s): gates %zu/%zu, chambers %u/%u, triangles %s, cells %s\n",
                  i, steps[i].object, steps[i].chain ? "chain" : "sibling",
                  mine.gates.size(), theirs.gates.size(), mine.chambers, theirs.chambers,
                  mine.triangles == theirs.triangles ? "same" : "differ",
                  mine.of == theirs.of ? "same" : "differ");
    }
  }
  {
    auto state = world.base();
    for (auto k = ids.size(); k-- > 0;) {
      const std::array one{ids[k]};
      state = world.with(state, one);
    }
    const bool backwards = snapshotOf(world, *state) == snapshotOf(world, *prefixes.back());
    std::printf("\nincremental == from scratch: %zu of %zu steps (from scratch %.2f ms a state); "
                "all inserted backwards == forwards: %s\n",
                equal, steps.size(), scratchMs / static_cast<double>(steps.size()),
                backwards ? "yes" : "NO");
    EXPECT_EQ(equal, steps.size());
    EXPECT_TRUE(backwards);
  }

  // The last state against `cdtGates` on the whole raster.
  const auto& last = *prefixes.back();
  const auto& result = world.result(last);
  std::vector<Bottleneck> reference;
  double referenceMs = 0.0;
  for (int round = 0; round < 3; ++round) {
    const auto a = Clock::now();
    reference = cdtGates(bench.width, bench.height, bench.mask, bench.owner, bench.walls, 0, options);
    referenceMs += std::chrono::duration<double, std::milli>(Clock::now() - a).count() / 3.0;
  }
  const auto xy = [&](const std::size_t cell) {
    return std::pair{static_cast<double>(cell % bench.width), static_cast<double>(cell / bench.width)};
  };
  const auto lengthOf = [&](const Bottleneck& g) {
    const auto [ax, ay] = xy(g.first);
    const auto [bx, by] = xy(g.second);
    return std::hypot(bx - ax, by - ay);
  };
  const auto capOf = [&](const Bottleneck& g) {
    const auto cellsAcross = lengthOf(g);
    if (cellsAcross < options.clearance) {
      return 0U;
    }
    const auto plain = static_cast<std::uint32_t>(std::floor(cellsAcross / options.clearance));
    return plain < options.marginFrom
               ? plain
               : static_cast<std::uint32_t>(std::floor(cellsAcross / (options.clearance * options.margin)));
  };
  const auto endDistance = [&](const Bottleneck& a, const Bottleneck& b) {
    const auto d = [&](const std::size_t p, const std::size_t q) {
      const auto [px, py] = xy(p);
      const auto [qx, qy] = xy(q);
      return std::hypot(px - qx, py - qy);
    };
    return std::min(std::max(d(a.first, b.first), d(a.second, b.second)),
                    std::max(d(a.first, b.second), d(a.second, b.first)));
  };
  const auto pairOf = [&](const Bottleneck& g) {
    const auto group = [&](const std::size_t cell) {
      const auto label = bench.owner[cell];
      return label < bench.walls.size() ? bench.walls[label].group : FREE;
    };
    const auto x = group(g.first);
    const auto y = group(g.second);
    return std::pair{std::min(x, y), std::max(x, y)};
  };
  const auto feedEnd = [&](const Bottleneck& g) {
    return moving.contains(bench.owner[g.first]) || moving.contains(bench.owner[g.second]);
  };
  const auto match = [&](const bool loose) {
    std::vector<bool> used(result.gates.size(), false);
    std::size_t hit = 0;
    std::size_t hitFeed = 0;
    std::size_t feed = 0;
    for (const auto& gate : reference) {
      feed += feedEnd(gate) ? 1 : 0;
      double best = std::numeric_limits<double>::max();
      std::size_t at = used.size();
      for (std::size_t j = 0; j < result.gates.size(); ++j) {
        const auto& mine = result.gates[j];
        if (used[j] || capOf(mine) != capOf(gate) || (loose && pairOf(mine) != pairOf(gate))) {
          continue;
        }
        const auto d = endDistance(mine, gate);
        if (d <= (loose ? options.clearance : 2.0) && d < best) {
          best = d;
          at = j;
        }
      }
      if (at < used.size()) {
        used[at] = true;
        ++hit;
        hitFeed += feedEnd(gate) ? 1 : 0;
      }
    }
    return std::tuple{hit, hitFeed, feed};
  };
  const auto [strict, strictFeed, feedGates] = match(false);
  const auto [loose, looseFeed, feedGates2] = match(true);
  (void)feedGates2;
  std::printf("\nlast state (all %zu feedlines) against cdtGates(canonical) on the whole raster "
              "(%.2f ms, %zu gates); the world %zu gates:\n"
              "  strict (both end cells within 2 cells, same capacity by cells): %zu (%.1f %%), "
              "with a feedline end %zu of %zu (%.1f %%)\n"
              "  loose (same obstacle pair and capacity, ends within a clearance): %zu (%.1f %%), "
              "with a feedline end %zu of %zu (%.1f %%)\n",
              ids.size(), referenceMs, reference.size(), result.gates.size(), strict,
              100.0 * static_cast<double>(strict) / static_cast<double>(reference.size()),
              strictFeed, feedGates,
              100.0 * static_cast<double>(strictFeed) / static_cast<double>(feedGates), loose,
              100.0 * static_cast<double>(loose) / static_cast<double>(reference.size()),
              looseFeed, feedGates,
              100.0 * static_cast<double>(looseFeed) / static_cast<double>(feedGates));

  // The chambers against `chambersOf`.
  const auto grid = GridMetrics::fit({.minX = 0.0, .minY = 0.0,
                                      .maxX = static_cast<double>(bench.width - 1),
                                      .maxY = static_cast<double>(bench.height - 1)},
                                     bench.width, bench.height);
  Chambers theirs;
  double theirsMs = 0.0;
  for (int round = 0; round < 3; ++round) {
    const auto a = Clock::now();
    chambersOf(bench.mask, reference, grid, theirs);
    theirsMs += std::chrono::duration<double, std::milli>(Clock::now() - a).count() / 3.0;
  }
  Chambers mine;
  double mineMs = 0.0;
  for (int round = 0; round < 3; ++round) {
    const auto a = Clock::now();
    world.chambers(last, mine);
    mineMs += std::chrono::duration<double, std::milli>(Clock::now() - a).count() / 3.0;
  }
  Chambers ofMyGates;
  chambersOf(bench.mask, result.gates, grid, ofMyGates);
  const auto p1 = pairChambers(bench.mask, theirs.of, mine.of);
  const auto p2 = pairChambers(bench.mask, ofMyGates.of, mine.of);
  std::size_t freeCells = 0;
  std::size_t unlabelled = 0;
  for (std::size_t c = 0; c < cells; ++c) {
    if (!bench.mask.test(c)) {
      ++freeCells;
      unlabelled += mine.of[c] == NO_CHAMBER ? 1 : 0;
    }
  }
  // `chamberAt`, cell by cell, against the raster of `chambers`.
  std::size_t probes = 0;
  std::size_t agree = 0;
  const auto a = Clock::now();
  for (std::uint32_t y = 3; y < bench.height; y += 17) {
    for (std::uint32_t x = 5; x < bench.width; x += 13) {
      ++probes;
      agree += world.chamberAt(last, x, y) == mine.of[(static_cast<std::size_t>(y) * bench.width) + x] ? 1 : 0;
    }
  }
  const auto probeUs = std::chrono::duration<double, std::micro>(Clock::now() - a).count() /
                       static_cast<double>(probes);
  std::printf("\nchambers: chambersOf(raster, cdtGates) %u chambers in %.2f ms; the world %u "
              "(%zu with cells), its full raster in %.2f ms (%zu of %zu free cells in none)\n"
              "  paired with chambersOf(cdtGates): %zu pairs of %zu/%zu, cells in pairs %.2f %% of "
              "those both cover\n"
              "  paired with chambersOf(the world's gates): %zu pairs of %zu/%zu, cells %.2f %%\n"
              "  chamberAt: %zu probes, %zu agree with the raster, %.2f us a probe\n",
              theirs.count, theirsMs, mine.count, p1.newCount, mineMs, unlabelled, freeCells,
              p1.mutual, p1.refCount, p1.newCount,
              100.0 * static_cast<double>(p1.cellsAgree) / static_cast<double>(p1.cellsBoth),
              p2.mutual, p2.refCount, p2.newCount,
              100.0 * static_cast<double>(p2.cellsAgree) / static_cast<double>(p2.cellsBoth),
              probes, agree, probeUs);
}


// Every wall of MQT_SCPD_CDT_BENCH but the border and the artwork (labels 0
// and 1) as an object of its own: pads, port runs, stubs and feedlines, as
// the router makes them, touching one another where their cells meet. Random
// sets of them, through the cache and one by one, against scratch; no state
// may throw. Run by hand:
//
// MQT_SCPD_CDT_BENCH=<file> mqt-scpd-grid-test
//   --gtest_also_run_disabled_tests --gtest_filter='CdtWorld.DISABLED_TouchingObjects'
TEST(CdtWorld, DISABLED_TouchingObjects) {
  Bench bench;
  if (!loadBench(bench)) {
    GTEST_SKIP() << "MQT_SCPD_CDT_BENCH names no file";
  }
  const auto cells = static_cast<std::size_t>(bench.width) * bench.height;
  std::vector<std::size_t> size(bench.walls.size(), 0);
  for (std::size_t c = 0; c < cells; ++c) {
    if (bench.mask.test(c) && bench.owner[c] < bench.walls.size()) {
      ++size[bench.owner[c]];
    }
  }
  std::vector<std::uint32_t> labels;
  for (std::uint32_t label = 2; label < bench.walls.size(); ++label) {
    if (size[label] > 0) {
      labels.push_back(label);
    }
  }
  const std::set<std::uint32_t> moving(labels.begin(), labels.end());
  BitGrid staticMask(bench.width, bench.height);
  std::vector<std::uint32_t> staticOwner(cells, FREE);
  for (std::size_t c = 0; c < cells; ++c) {
    if (bench.mask.test(c) && !moving.contains(bench.owner[c])) {
      staticMask.set(c);
      staticOwner[c] = bench.owner[c];
    }
  }
  const CdtGateOptions options{.clearance = 19.0, .widest = 10, .canonical = true};
  using Clock = std::chrono::steady_clock;
  CdtWorld world(bench.width, bench.height, staticMask, staticOwner, bench.walls, 0, options,
                 {}, 16);
  std::vector<std::uint32_t> ids;
  const auto t0 = Clock::now();
  for (const auto label : labels) {
    const std::array one{label};
    ids.push_back(world.add(cdtObjectOf(label, bench.mask, bench.owner, bench.walls, one)));
  }
  const auto addMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  std::map<std::string, std::size_t> errors;
  std::size_t states = 0;
  std::size_t failed = 0;
  std::size_t compared = 0;
  std::size_t equal = 0;
  std::vector<double> stepMs;
  const auto attempt = [&](const auto& make) -> CdtWorld::StatePtr {
    ++states;
    try {
      return make();
    } catch (const std::exception& error) {
      ++failed;
      std::string what = error.what();
      if (const auto at = what.find(" at "); at != std::string::npos) {
        what.resize(at);
      }
      ++errors[what.substr(0, 60)];
      return nullptr;
    }
  };
  const auto check = [&](const CdtWorld::StatePtr& state, const std::vector<std::uint32_t>& set) {
    if (state == nullptr) {
      return;
    }
    const auto whole = attempt([&] { return world.fromScratch(set); });
    if (whole == nullptr) {
      return;
    }
    ++compared;
    const auto mine = snapshotOf(world, *state, false);
    const auto theirs = snapshotOf(world, *whole, false);
    if (mine == theirs) {
      ++equal;
    } else {
      ADD_FAILURE() << "a state of " << set.size() << " objects differs from scratch:\n"
                    << diffOf(mine, theirs);
    }
  };
  std::uint64_t seed = 977;
  const auto next = [&] {
    seed = (seed * 6364136223846793005ULL) + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(seed >> 33U);
  };
  // All of them at once.
  check(attempt([&] { return world.fromScratch(ids); }), ids);
  // Random sets through the cache, each half of the objects; every fourth
  // checked against scratch.
  for (int round = 0; round < 48; ++round) {
    std::vector<std::uint32_t> set;
    for (const auto id : ids) {
      if (next() % 2 == 0) {
        set.push_back(id);
      }
    }
    const auto a = Clock::now();
    const auto state = attempt([&] { return world.state(set); });
    stepMs.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a).count());
    if (round % 4 == 0) {
      check(state, set);
    }
  }
  // A chain in a random order: one object at a time, every 16th prefix
  // against scratch.
  auto order = ids;
  for (auto k = order.size(); k > 1; --k) {
    std::swap(order[k - 1], order[next() % k]);
  }
  // An object whose step throws is left out, and the chain goes on.
  auto state = world.base();
  std::vector<std::uint32_t> prefix;
  for (std::size_t k = 0; k < order.size(); ++k) {
    const std::array one{order[k]};
    const auto a = Clock::now();
    const auto grown = attempt([&] { return world.with(state, one); });
    stepMs.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a).count());
    if (grown == nullptr) {
      continue;
    }
    state = grown;
    prefix.push_back(order[k]);
    if (k % 16 == 15 || k + 1 == order.size()) {
      check(state, prefix);
    }
  }
  std::printf("touching objects: %zu objects (added in %.1f ms), %zu states, %zu threw; "
              "%zu of %zu compared equal to scratch; a state %.3f ms median, %.3f p95\n",
              ids.size(), addMs, states, failed, equal, compared, quantile(stepMs, 0.5),
              quantile(stepMs, 0.95));
  for (const auto& [what, count] : errors) {
    std::printf("  %4zu x %s\n", count, what.c_str());
  }
  EXPECT_EQ(failed, 0U);
  EXPECT_EQ(equal, compared);
}

} // namespace
} // namespace mqt::scpd::grid
