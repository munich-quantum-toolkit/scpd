/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The bottlenecks of a constrained Delaunay triangulation of the free space,
// on a raster whose one narrow gap the test knows.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/CdtGates.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <limits>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t WIDTH = 80;
constexpr std::uint32_t HEIGHT = 60;
constexpr std::uint32_t FREE = std::numeric_limits<std::uint32_t>::max();

/// A border ring (label 0), a block (label 1) at y 10 to 20 and a bar (label
/// 2) at y 33 to 36, both from x 20 to 40: a gap of 12 cells between them.
struct Scene {
  BitGrid blocked{WIDTH, HEIGHT};
  std::vector<std::uint32_t> owner =
      std::vector<std::uint32_t>(static_cast<std::size_t>(WIDTH) * HEIGHT, FREE);

  Scene() {
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      for (std::uint32_t x = 0; x < WIDTH; ++x) {
        std::uint32_t label = FREE;
        if (x == 0 || y == 0 || x == WIDTH - 1 || y == HEIGHT - 1) {
          label = 0;
        } else if (x >= 20 && x <= 40 && y >= 10 && y <= 20) {
          label = 1;
        } else if (x >= 20 && x <= 40 && y >= 33 && y <= 36) {
          label = 2;
        }
        if (label != FREE) {
          blocked.setCell(x, y, true);
          owner[(static_cast<std::size_t>(y) * WIDTH) + x] = label;
        }
      }
    }
  }
};

/// A clearance of 5 cells and gaps under 3 wires: only the gap between the
/// block and the bar is narrow enough.
CdtGateOptions narrowOnly() {
  return {.clearance = 5.0, .widest = 3, .margin = 1.06, .marginFrom = 3};
}

std::uint32_t labelOf(const Scene& scene, const std::size_t cell) {
  return scene.owner[cell];
}

TEST(CdtGates, CutsTheGapBetweenAMovingWallAndAStaticOneOnce) {
  const Scene scene;
  const std::vector<CdtWall> walls{{.group = 0, .fixed = true, .target = false},
                                   {.group = 1, .fixed = true, .target = false},
                                   {.group = 2, .fixed = false, .target = false}};
  CdtGateStats stats;
  const auto gates = cdtGates(WIDTH, HEIGHT, scene.blocked, scene.owner, walls,
                              0, narrowOnly(), &stats);
  ASSERT_EQ(gates.size(), 1U);
  const auto& gate = gates.front();
  const auto a = labelOf(scene, gate.first);
  const auto b = labelOf(scene, gate.second);
  EXPECT_TRUE((a == 1 && b == 2) || (a == 2 && b == 1));
  const auto dy = std::abs(static_cast<long>(gate.first / WIDTH) -
                           static_cast<long>(gate.second / WIDTH));
  const auto dx = std::abs(static_cast<long>(gate.first % WIDTH) -
                           static_cast<long>(gate.second % WIDTH));
  EXPECT_EQ(dy, 13);
  EXPECT_LE(dx, 1);
  EXPECT_GT(stats.triangles, 0U);
}

TEST(CdtGates, LeavesTheGapBetweenTwoStaticWallsAlone) {
  const Scene scene;
  const std::vector<CdtWall> walls{{.group = 0, .fixed = true, .target = false},
                                   {.group = 1, .fixed = true, .target = false},
                                   {.group = 2, .fixed = true, .target = false}};
  EXPECT_TRUE(
      cdtGates(WIDTH, HEIGHT, scene.blocked, scene.owner, walls, 0, narrowOnly())
          .empty());
}

TEST(CdtGates, LeavesTheGapWithinOneGroupAlone) {
  // The block and the bar are one obstacle that moves: no gate joins them,
  // while the block keeps its gate to the border below it.
  const Scene scene;
  const std::vector<CdtWall> walls{{.group = 0, .fixed = true, .target = false},
                                   {.group = 1, .fixed = false, .target = false},
                                   {.group = 1, .fixed = false, .target = false}};
  const auto gates =
      cdtGates(WIDTH, HEIGHT, scene.blocked, scene.owner, walls, 0, narrowOnly());
  EXPECT_FALSE(gates.empty());
  for (const auto& gate : gates) {
    const auto a = labelOf(scene, gate.first);
    const auto b = labelOf(scene, gate.second);
    EXPECT_FALSE((a == 1 && b == 2) || (a == 2 && b == 1));
  }
}

TEST(CdtGates, LeavesAGapOfTheTargetRunToAStaticWallAlone) {
  const Scene scene;
  const std::vector<CdtWall> walls{{.group = 0, .fixed = true, .target = false},
                                   {.group = 1, .fixed = true, .target = false},
                                   {.group = 2, .fixed = false, .target = true}};
  EXPECT_TRUE(
      cdtGates(WIDTH, HEIGHT, scene.blocked, scene.owner, walls, 0, narrowOnly())
          .empty());
}

// A timing of `cdtGates` on a wall raster written to a file: width, height and
// the count of walls as three uint32, a byte per cell for the mask, a uint32
// per cell for the label, and per wall its group, whether it is fixed and
// whether it is a target run as three uint32. Run by hand:
/// Blocks of one size on a lattice, each a moving wall of its own, inside a
/// static border: every gap between two of them is as wide as the next, and
/// the corners of four blocks around a gap lie on one circle, so the order
/// the triangulation meets the points in decides its diagonals and which of
/// the equal gaps is the narrowest of a basin.
struct Blocks {
  static constexpr std::uint32_t SIDE = 160;
  BitGrid blocked{SIDE, SIDE};
  std::vector<std::uint32_t> owner = std::vector<std::uint32_t>(static_cast<std::size_t>(SIDE) * SIDE, FREE);
  std::vector<CdtWall> walls{{.group = 0, .fixed = true}};

  Blocks() {
    for (std::uint32_t y = 0; y < SIDE; ++y) {
      for (std::uint32_t x = 0; x < SIDE; ++x) {
        if (x == 0 || y == 0 || x == SIDE - 1 || y == SIDE - 1) {
          blocked.setCell(x, y, true);
          owner[(static_cast<std::size_t>(y) * SIDE) + x] = 0;
        }
      }
    }
    for (std::uint32_t by = 0; by < 5; ++by) {
      for (std::uint32_t bx = 0; bx < 5; ++bx) {
        const auto label = static_cast<std::uint32_t>(walls.size());
        walls.push_back({.group = label, .fixed = false});
        for (std::uint32_t y = 12 + (by * 30); y < 30 + (by * 30); ++y) {
          for (std::uint32_t x = 12 + (bx * 30); x < 30 + (bx * 30); ++x) {
            blocked.setCell(x, y, true);
            owner[(static_cast<std::size_t>(y) * SIDE) + x] = label;
          }
        }
      }
    }
  }
};

std::vector<std::tuple<std::size_t, std::size_t, std::size_t>>
keysOf(const std::vector<Bottleneck>& gates) {
  std::vector<std::tuple<std::size_t, std::size_t, std::size_t>> keys;
  for (const auto& gate : gates) {
    keys.emplace_back(gate.first, gate.second, gate.saddle);
  }
  return keys;
}

TEST(CdtGates, CanonicalGatesDoNotDependOnTheOrderOfThePoints) {
  const Blocks scene;
  CdtGateOptions options{.clearance = 5.0, .widest = 4, .margin = 1.06, .marginFrom = 3, .canonical = true};
  const auto first = keysOf(cdtGates(Blocks::SIDE, Blocks::SIDE, scene.blocked, scene.owner, scene.walls, 0, options));
  ASSERT_FALSE(first.empty());
  for (const std::uint32_t seed : {1U, 2U, 3U, 17U, 4711U}) {
    options.shuffle = seed;
    EXPECT_EQ(keysOf(cdtGates(Blocks::SIDE, Blocks::SIDE, scene.blocked, scene.owner, scene.walls, 0, options)), first)
        << "seed " << seed;
  }
}

// MQT_SCPD_CDT_BENCH=<file> mqt-scpd-grid-test
//   --gtest_also_run_disabled_tests --gtest_filter='*DISABLED_Timing*'
TEST(CdtGates, DISABLED_Timing) {
  const char* path = std::getenv("MQT_SCPD_CDT_BENCH");
  if (path == nullptr) {
    GTEST_SKIP() << "MQT_SCPD_CDT_BENCH names no file";
  }
  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in.good());
  std::array<std::uint32_t, 3> head{};
  in.read(reinterpret_cast<char*>(head.data()), sizeof(head));
  const auto width = head[0];
  const auto height = head[1];
  const auto cells = static_cast<std::size_t>(width) * height;
  std::vector<std::uint8_t> mask(cells);
  in.read(reinterpret_cast<char*>(mask.data()), static_cast<std::streamsize>(cells));
  std::vector<std::uint32_t> owner(cells);
  in.read(reinterpret_cast<char*>(owner.data()),
          static_cast<std::streamsize>(cells * sizeof(std::uint32_t)));
  std::vector<std::uint32_t> raw(static_cast<std::size_t>(head[2]) * 3);
  in.read(reinterpret_cast<char*>(raw.data()),
          static_cast<std::streamsize>(raw.size() * sizeof(std::uint32_t)));
  BitGrid blocked(width, height);
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (mask[cell] != 0) {
      blocked.set(cell);
    }
  }
  std::vector<CdtWall> walls;
  for (std::size_t k = 0; k < head[2]; ++k) {
    walls.push_back({.group = raw[(k * 3)],
                     .fixed = raw[(k * 3) + 1] != 0,
                     .target = raw[(k * 3) + 2] != 0});
  }
  const CdtGateOptions options{.clearance = 19.0, .widest = 10, .canonical = std::getenv("MQT_SCPD_CDT_CANONICAL") != nullptr};
  const char* times = std::getenv("MQT_SCPD_CDT_TIMES");
  const auto rounds = times != nullptr ? std::atoi(times) : 20;
  std::array<double, 5> seconds{};
  CdtGateStats stats;
  std::size_t gates = 0;
  const auto began = std::chrono::steady_clock::now();
  for (int round = 0; round < rounds; ++round) {
    gates = cdtGates(width, height, blocked, owner, walls, 0, options, &stats).size();
    for (std::size_t step = 0; step < seconds.size(); ++step) {
      seconds.at(step) += stats.seconds.at(step);
    }
  }
  const auto total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  std::printf("cdtGates: %zu gates, %zu vertices, %zu triangles; %.2f ms a call "
              "(outline %.2f, loops %.2f, feet %.2f, triangulation %.2f, basins %.2f)\n",
              gates, stats.vertices, stats.triangles, 1000.0 * total / rounds,
              1000.0 * seconds[0] / rounds, 1000.0 * seconds[1] / rounds,
              1000.0 * seconds[2] / rounds, 1000.0 * seconds[3] / rounds,
              1000.0 * seconds[4] / rounds);
}

// The incremental `cdtGatesCached` against `cdtGates` on the raster of
// MQT_SCPD_CDT_BENCH with the cells of one moving wall freed: the gates both
// find, and how long each took.
TEST(CdtGates, DISABLED_Incremental) {
  const char* path = std::getenv("MQT_SCPD_CDT_BENCH");
  if (path == nullptr) {
    GTEST_SKIP() << "MQT_SCPD_CDT_BENCH names no file";
  }
  std::ifstream in(path, std::ios::binary);
  std::array<std::uint32_t, 3> head{};
  in.read(reinterpret_cast<char*>(head.data()), sizeof(head));
  const auto width = head[0];
  const auto height = head[1];
  const auto cells = static_cast<std::size_t>(width) * height;
  std::vector<std::uint8_t> mask(cells);
  in.read(reinterpret_cast<char*>(mask.data()), static_cast<std::streamsize>(cells));
  std::vector<std::uint32_t> owner(cells);
  in.read(reinterpret_cast<char*>(owner.data()),
          static_cast<std::streamsize>(cells * sizeof(std::uint32_t)));
  std::vector<std::uint32_t> raw(static_cast<std::size_t>(head[2]) * 3);
  in.read(reinterpret_cast<char*>(raw.data()),
          static_cast<std::streamsize>(raw.size() * sizeof(std::uint32_t)));
  std::vector<CdtWall> walls;
  for (std::size_t k = 0; k < head[2]; ++k) {
    walls.push_back({.group = raw[(k * 3)],
                     .fixed = raw[(k * 3) + 1] != 0,
                     .target = raw[(k * 3) + 2] != 0});
  }
  const auto gridOf = [&](const std::vector<std::uint8_t>& m) {
    BitGrid grid(width, height);
    for (std::size_t cell = 0; cell < cells; ++cell) {
      if (m[cell] != 0) {
        grid.set(cell);
      }
    }
    return grid;
  };
  // The moving wall with the most cells, from the label given or the largest.
  std::map<std::uint32_t, std::size_t> size;
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (mask[cell] != 0 && owner[cell] < walls.size() && !walls[owner[cell]].fixed) {
      ++size[owner[cell]];
    }
  }
  const char* pick = std::getenv("MQT_SCPD_CDT_FREE");
  std::uint32_t freed = pick != nullptr ? static_cast<std::uint32_t>(std::atoi(pick)) : 0;
  if (pick == nullptr) {
    std::size_t most = 0;
    for (const auto& [label, count] : size) {
      if (count > most) {
        most = count;
        freed = label;
      }
    }
  }
  auto changed = mask;
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (owner[cell] == freed) {
      changed[cell] = 0;
    }
  }
  const CdtGateOptions options{.clearance = 19.0, .widest = 10, .canonical = std::getenv("MQT_SCPD_CDT_CANONICAL") != nullptr};
  CdtGateCache cache;
  const auto before = gridOf(mask);
  const auto after = gridOf(changed);
  (void)cdtGatesCached(width, height, before, owner, walls, 0, options, cache);
  const auto t0 = std::chrono::steady_clock::now();
  CdtGateStats inc;
  const auto fast = cdtGatesCached(width, height, after, owner, walls, 0, options, cache, &inc);
  const auto t1 = std::chrono::steady_clock::now();
  const auto full = cdtGates(width, height, after, owner, walls, 0, options);
  const auto t2 = std::chrono::steady_clock::now();
  std::set<std::pair<std::size_t, std::size_t>> a;
  std::set<std::pair<std::size_t, std::size_t>> b;
  for (const auto& g : fast) {
    a.emplace(std::min(g.first, g.second), std::max(g.first, g.second));
  }
  for (const auto& g : full) {
    b.emplace(std::min(g.first, g.second), std::max(g.first, g.second));
  }
  std::size_t both = 0;
  for (const auto& g : a) {
    both += b.contains(g) ? 1 : 0;
  }
  // Where the gates of one answer but not the other lie: their middles,
  // against the box of the freed cells.
  std::int64_t x0 = width, y0 = height, x1 = -1, y1 = -1;
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (owner[cell] == freed) {
      x0 = std::min<std::int64_t>(x0, cell % width);
      x1 = std::max<std::int64_t>(x1, cell % width);
      y0 = std::min<std::int64_t>(y0, cell / width);
      y1 = std::max<std::int64_t>(y1, cell / width);
    }
  }
  const auto report = [&](const char* what, const auto& mine, const auto& theirs) {
    std::size_t count = 0;
    for (const auto& g : mine) {
      if (theirs.contains(g)) {
        continue;
      }
      const auto mx = static_cast<std::int64_t>((g.first % width + g.second % width) / 2);
      const auto my = static_cast<std::int64_t>((g.first / width + g.second / width) / 2);
      const auto dx = std::max<std::int64_t>({0, x0 - mx, mx - x1});
      const auto dy = std::max<std::int64_t>({0, y0 - my, my - y1});
      if (count++ < 12) {
        std::printf("  %s: middle (%lld,%lld), %lld/%lld off the freed box, length %.0f\n", what,
                    static_cast<long long>(mx), static_cast<long long>(my), static_cast<long long>(dx),
                    static_cast<long long>(dy),
                    std::hypot(static_cast<double>(g.first % width) - static_cast<double>(g.second % width),
                               static_cast<double>(g.first / width) - static_cast<double>(g.second / width)));
      }
    }
  };
  const auto grow = static_cast<std::int64_t>(std::ceil((10.0 * 19.0 * 1.06) + 19.0));
  const auto inside = [&](const auto& set) {
    std::size_t in = 0;
    for (const auto& g : set) {
      const auto mx = static_cast<std::int64_t>((g.first % width + g.second % width) / 2);
      const auto my = static_cast<std::int64_t>((g.first / width + g.second / width) / 2);
      in += (mx >= x0 - grow && mx <= x1 + grow && my >= y0 - grow && my <= y1 + grow) ? 1 : 0;
    }
    return in;
  };
  std::printf("  inside the inner box: incremental %zu, whole %zu; outside: %zu, %zu\n", inside(a),
              inside(b), a.size() - inside(a), b.size() - inside(b));
  report("only incremental", a, b);
  report("only whole", b, a);
  std::printf("freed wall %u (%zu cells): incremental %zu gates in %.2f ms (window %llu cells), "
              "whole %zu gates in %.2f ms, %zu in both\n",
              freed, size[freed], fast.size(),
              std::chrono::duration<double>(t1 - t0).count() * 1000.0,
              static_cast<unsigned long long>(cache.windowCells), full.size(),
              std::chrono::duration<double>(t2 - t1).count() * 1000.0, both);
}

/// The raster of MQT_SCPD_CDT_BENCH: its mask, owners and walls.
struct BenchRaster {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::uint8_t> mask;
  std::vector<std::uint32_t> owner;
  std::vector<CdtWall> walls;

  [[nodiscard]] BitGrid grid(const std::vector<std::uint8_t>& m) const {
    BitGrid out(width, height);
    for (std::size_t cell = 0; cell < m.size(); ++cell) {
      if (m[cell] != 0) {
        out.set(cell);
      }
    }
    return out;
  }
};

bool loadBench(BenchRaster& raster) {
  const char* path = std::getenv("MQT_SCPD_CDT_BENCH");
  if (path == nullptr) {
    return false;
  }
  std::ifstream in(path, std::ios::binary);
  std::array<std::uint32_t, 3> head{};
  in.read(reinterpret_cast<char*>(head.data()), sizeof(head));
  raster.width = head[0];
  raster.height = head[1];
  const auto cells = static_cast<std::size_t>(head[0]) * head[1];
  raster.mask.resize(cells);
  in.read(reinterpret_cast<char*>(raster.mask.data()), static_cast<std::streamsize>(cells));
  raster.owner.resize(cells);
  in.read(reinterpret_cast<char*>(raster.owner.data()),
          static_cast<std::streamsize>(cells * sizeof(std::uint32_t)));
  std::vector<std::uint32_t> raw(static_cast<std::size_t>(head[2]) * 3);
  in.read(reinterpret_cast<char*>(raw.data()),
          static_cast<std::streamsize>(raw.size() * sizeof(std::uint32_t)));
  for (std::size_t k = 0; k < head[2]; ++k) {
    raster.walls.push_back({.group = raw[(k * 3)],
                            .fixed = raw[(k * 3) + 1] != 0,
                            .target = raw[(k * 3) + 2] != 0});
  }
  return in.good();
}

// `chambersOf` on the raster of MQT_SCPD_CDT_BENCH with its CDT gates, and
// with the cells of one moving wall freed (MQT_SCPD_CDT_FREE, or every
// 50th): how long each labelling takes.
TEST(CdtGates, DISABLED_ChamberTiming) {
  BenchRaster raster;
  if (!loadBench(raster)) {
    GTEST_SKIP() << "MQT_SCPD_CDT_BENCH names no file";
  }
  const auto grid = GridMetrics::fit({.minX = 0.0,
                                      .minY = 0.0,
                                      .maxX = static_cast<double>(raster.width - 1),
                                      .maxY = static_cast<double>(raster.height - 1)},
                                     raster.width, raster.height);
  const CdtGateOptions options{.clearance = 19.0, .widest = 10, .canonical = std::getenv("MQT_SCPD_CDT_CANONICAL") != nullptr};
  const auto before = raster.grid(raster.mask);
  const auto gates = cdtGates(raster.width, raster.height, before, raster.owner,
                              raster.walls, 0, options);
  Chambers chambers;
  constexpr int ROUNDS = 40;
  const auto t0 = std::chrono::steady_clock::now();
  for (int round = 0; round < ROUNDS; ++round) {
    chambersOf(before, gates, grid, chambers);
  }
  const auto t1 = std::chrono::steady_clock::now();
  std::printf("chambersOf: %u x %u cells, %zu gates, %u chambers, %.3f ms a call\n",
              raster.width, raster.height, gates.size(), chambers.count,
              1000.0 * std::chrono::duration<double>(t1 - t0).count() / ROUNDS);

  // One moving wall freed every other call, as a step check that tries a
  // wire and the next that does not: the gates from the incremental CDT,
  // the chambers from the cache against from scratch.
  std::vector<std::uint32_t> freedWalls;
  if (const char* pick = std::getenv("MQT_SCPD_CDT_FREE"); pick != nullptr) {
    freedWalls.push_back(static_cast<std::uint32_t>(std::atoi(pick)));
  } else {
    for (std::uint32_t label = 0; label < raster.walls.size(); label += 50) {
      if (!raster.walls[label].fixed) {
        freedWalls.push_back(label);
      }
    }
  }
  CdtGateCache gateCache;
  ChamberCache chamberCache;
  Chambers cached;
  double fresh = 0.0;
  double kept = 0.0;
  int calls = 0;
  for (const auto label : freedWalls) {
    auto changed = raster.mask;
    for (std::size_t cell = 0; cell < changed.size(); ++cell) {
      if (raster.owner[cell] == label) {
        changed[cell] = 0;
      }
    }
    for (const auto* m : {&changed, &raster.mask, &changed}) {
      const auto mask = raster.grid(*m);
      const auto these = cdtGatesCached(raster.width, raster.height, mask, raster.owner,
                                        raster.walls, 0, options, gateCache);
      const auto a = std::chrono::steady_clock::now();
      chambersOfCached(mask, these, grid, cached, chamberCache);
      const auto b = std::chrono::steady_clock::now();
      chambersOf(mask, these, grid, chambers);
      const auto c = std::chrono::steady_clock::now();
      kept += std::chrono::duration<double>(b - a).count();
      fresh += std::chrono::duration<double>(c - b).count();
      ++calls;
      ASSERT_EQ(cached.of, chambers.of) << "wall " << label;
      ASSERT_EQ(cached.count, chambers.count) << "wall " << label;
      ASSERT_EQ(cached.beside, chambers.beside) << "wall " << label;
    }
  }
  std::printf("chambersOfCached: %d calls over %zu freed walls, %.3f ms a call against %.3f "
              "from scratch; gates reused %zu, traced %zu, laid on %zu, taken off %zu\n",
              calls, freedWalls.size(), 1000.0 * kept / calls, 1000.0 * fresh / calls,
              chamberCache.reused, chamberCache.computed, chamberCache.added,
              chamberCache.removed);
}

} // namespace
} // namespace mqt::scpd::grid
