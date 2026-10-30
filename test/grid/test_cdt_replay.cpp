/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The replay benchmark of the CDT world: the passes of a coupler insertion,
// as `worldGatesOf` of the Final router wrote them under
// MQT_SCPD_CDT_REPLAY=<file>, played again in-process, by several
// strategies, with the time of every pass and the gates of every pass
// checked between the strategies and, every so often, against a state built
// from scratch. Run by hand:
//
// MQT_SCPD_CDT_REPLAY=<file> [MQT_SCPD_CDT_STRATEGIES=pinned,cache]
//   [MQT_SCPD_CDT_CACHE=32] [MQT_SCPD_CDT_ORACLE_EVERY=50]
//   [MQT_SCPD_CDT_LIMIT=<passes>] [MQT_SCPD_CDT_CHAIN=<chain>]
//   mqt-scpd-grid-test --gtest_also_run_disabled_tests
//   --gtest_filter='CdtWorld.DISABLED_Replay'

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
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::grid {
namespace {

struct ReplayPass {
  std::uint32_t chain = 0;
  std::vector<std::uint32_t> ids;
  /// The gates the router got, or nothing where the world refused the pass.
  std::optional<std::uint32_t> gates;
};

struct Replay {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  BitGrid mask;
  std::vector<std::uint32_t> owner;
  std::vector<CdtWall> statics;
  std::vector<std::uint8_t> cuts;
  CdtGateOptions options;
  std::vector<CdtObject> objects;
  std::vector<ReplayPass> passes;
};

struct Reader {
  std::ifstream in;
  template <class T> T get() {
    T value{};
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return value;
  }
  template <class T> void getInto(std::vector<T>& out, const std::size_t n) {
    out.resize(n);
    in.read(reinterpret_cast<char*>(out.data()),
            static_cast<std::streamsize>(n * sizeof(T)));
  }
};

bool loadReplay(const char* path, Replay& r) {
  Reader rd{std::ifstream(path, std::ios::binary)};
  if (!rd.in) {
    return false;
  }
  std::array<char, 4> magic{};
  rd.in.read(magic.data(), 4);
  if (std::string_view(magic.data(), 4) != "CDTR") {
    return false;
  }
  if (rd.get<std::uint32_t>() != 1) {
    return false;
  }
  r.width = rd.get<std::uint32_t>();
  r.height = rd.get<std::uint32_t>();
  const auto nWords = rd.get<std::uint64_t>();
  std::vector<std::uint64_t> words;
  rd.getInto(words, nWords);
  r.mask = BitGrid(r.width, r.height);
  for (std::size_t w = 0; w < words.size(); ++w) {
    for (auto bits = words[w]; bits != 0; bits &= bits - 1) {
      const auto cell = (w * 64U) + static_cast<std::size_t>(std::countr_zero(bits));
      if (cell < static_cast<std::size_t>(r.width) * r.height) {
        r.mask.set(cell);
      }
    }
  }
  rd.getInto(r.owner, static_cast<std::size_t>(r.width) * r.height);
  const auto nStatics = rd.get<std::uint32_t>();
  for (std::uint32_t k = 0; k < nStatics; ++k) {
    CdtWall wall;
    wall.group = rd.get<std::uint32_t>();
    wall.fixed = rd.get<std::uint8_t>() != 0;
    wall.target = rd.get<std::uint8_t>() != 0;
    r.statics.push_back(wall);
    r.cuts.push_back(rd.get<std::uint8_t>());
  }
  r.options.clearance = rd.get<double>();
  r.options.widest = rd.get<std::uint32_t>();
  r.options.margin = rd.get<double>();
  r.options.marginFrom = rd.get<std::uint32_t>();
  r.options.simplify = rd.get<double>();
  r.options.footReach = rd.get<double>();
  r.options.canonical = true;
  while (rd.in.peek() != std::ifstream::traits_type::eof()) {
    const auto type = rd.get<std::uint8_t>();
    if (!rd.in) {
      break;
    }
    if (type == 1) {
      CdtObject object;
      object.key = rd.get<std::uint64_t>();
      const auto nWalls = rd.get<std::uint32_t>();
      for (std::uint32_t w = 0; w < nWalls; ++w) {
        CdtWall wall;
        wall.group = rd.get<std::uint32_t>();
        wall.fixed = rd.get<std::uint8_t>() != 0;
        wall.target = rd.get<std::uint8_t>() != 0;
        object.walls.push_back(wall);
        object.cuts.push_back(rd.get<std::uint8_t>());
      }
      const auto nCells = rd.get<std::uint64_t>();
      object.cells.resize(nCells);
      for (auto& cell : object.cells) {
        cell = static_cast<std::size_t>(rd.get<std::uint64_t>());
      }
      rd.getInto(object.wallOf, nCells);
      r.objects.push_back(std::move(object));
    } else if (type == 2) {
      ReplayPass pass;
      pass.chain = rd.get<std::uint32_t>();
      const auto n = rd.get<std::uint32_t>();
      rd.getInto(pass.ids, n);
      r.passes.push_back(std::move(pass));
    } else if (type == 3) {
      const auto gates = rd.get<std::uint32_t>();
      if (!r.passes.empty()) {
        r.passes.back().gates = gates;
      }
    } else {
      return false;
    }
  }
  return true;
}

/// What one pass gave: the gates that cut (as the router keeps them), with
/// their capacity.
struct Gates {
  std::vector<std::array<std::uint64_t, 4>> list;
  bool refused = false;
  bool operator==(const Gates&) const = default;
};

Gates gatesOf(const CdtWorld& world, const CdtWorld::State& state) {
  Gates out;
  const auto& r = world.result(state);
  for (std::size_t g = 0; g < r.gates.size(); ++g) {
    if (r.cuts[g] != 0) {
      out.list.push_back({r.gates[g].first, r.gates[g].second, r.gates[g].saddle,
                          r.capacity[g]});
    }
  }
  return out;
}

struct Strategy {
  std::string name;
  std::vector<double> ms;
  std::vector<Gates> gates;
  CdtStepStats laps;
  std::size_t refused = 0;
  std::size_t mismatches = 0;
  std::size_t inserted = 0;
};

double percentile(std::vector<double> values, const double p) {
  if (values.empty()) {
    return 0.0;
  }
  std::ranges::sort(values);
  const auto at = std::min(values.size() - 1,
                           static_cast<std::size_t>(p * static_cast<double>(values.size())));
  return values[at];
}

std::uint32_t envOr(const char* name, const std::uint32_t fallback) {
  const char* v = std::getenv(name);
  return v == nullptr || *v == '\0' ? fallback : static_cast<std::uint32_t>(std::atoi(v));
}

} // namespace

// The chambers of the passes: what the raster labelling (`chambersOfCached`,
// as the router runs it) costs against the world's chambers per triangle,
// how far the two agree, and what a lookup in the world costs. Run by hand:
//
// MQT_SCPD_CDT_REPLAY=<file> [MQT_SCPD_CDT_EVERY=10] [MQT_SCPD_CDT_LIMIT=<n>]
//   mqt-scpd-grid-test --gtest_also_run_disabled_tests
//   --gtest_filter='CdtWorld.DISABLED_ReplayChambers'
TEST(CdtWorld, DISABLED_ReplayChambers) {
  const char* path = std::getenv("MQT_SCPD_CDT_REPLAY");
  if (path == nullptr || *path == '\0') {
    GTEST_SKIP() << "MQT_SCPD_CDT_REPLAY names no file";
  }
  Replay replay;
  ASSERT_TRUE(loadReplay(path, replay)) << "cannot read " << path;
  const auto every = std::max(1U, envOr("MQT_SCPD_CDT_EVERY", 10));
  const auto limit = envOr("MQT_SCPD_CDT_LIMIT", 0);
  using Clock = std::chrono::steady_clock;
  const auto ms = [](const Clock::time_point a, const Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  CdtWorld world(replay.width, replay.height, replay.mask, replay.owner, replay.statics, 0,
                 replay.options, replay.cuts, 64);
  for (const auto& object : replay.objects) {
    ASSERT_EQ(world.add(object), world.objectCount() - 1);
  }
  GridMetrics grid;
  grid.width = replay.width;
  grid.height = replay.height;
  const auto cells = static_cast<std::size_t>(replay.width) * replay.height;
  const auto width = static_cast<std::int64_t>(replay.width);
  ChamberCache cache;
  Chambers raster;
  Chambers ofWorld;
  std::vector<std::uint32_t> filled(cells, NO_CHAMBER);
  std::mt19937 rng(11);
  struct Sums {
    double rasterMs = 0, movesMs = 0, fillMs = 0, worldMs = 0, randomNs = 0, coherentNs = 0;
    std::size_t passes = 0, gates = 0, besideSame = 0, besideDiff = 0, countSame = 0;
    std::size_t samples = 0, bothChamber = 0, rasterLineOnly = 0, worldNoneOnly = 0;
    std::size_t pairs = 0, pairsAgree = 0;
    std::size_t rasterChambers = 0, worldChambers = 0;
    std::size_t rasterOneWorldTwo = 0, rasterTwoWorldOne = 0, worldSliverSide = 0;
    std::size_t bothTwoSame = 0, bothTwoDiffer = 0, worldEmptyChambers = 0;
    std::size_t materialSides = 0, materialChambers = 0, materialSame = 0;
  } sum;
  std::size_t shown = 0;
  std::size_t played = 0;
  for (std::size_t k = 0; k < replay.passes.size(); k += every) {
    const auto& pass = replay.passes[k];
    if (limit != 0 && played >= limit) {
      break;
    }
    ++played;
    CdtWorld::StatePtr state;
    try {
      state = world.state(pass.ids);
    } catch (const std::exception&) {
      continue;
    }
    const auto& result = world.result(*state);
    std::vector<Bottleneck> gates;
    for (std::size_t g = 0; g < result.gates.size(); ++g) {
      if (result.cuts[g] != 0) {
        gates.push_back(result.gates[g]);
      }
    }
    BitGrid mask = replay.mask;
    for (const auto o : pass.ids) {
      for (const auto cell : replay.objects[o].cells) {
        mask.set(cell);
      }
    }
    // The raster labelling as the router runs it: moves cached by gate.
    const auto a = Clock::now();
    chambersOfCached(mask, gates, grid, raster, cache);
    const auto b = Clock::now();
    const auto moves = bottleneckMoves(gates, grid);
    const auto c = Clock::now();
    std::fill(filled.begin(), filled.end(), NO_CHAMBER);
    const auto d = Clock::now();
    world.chambers(*state, ofWorld);
    const auto e = Clock::now();
    sum.rasterMs += ms(a, b);
    sum.movesMs += ms(b, c);
    sum.fillMs += ms(c, d);
    sum.worldMs += ms(d, e);
    ++sum.passes;
    sum.gates += gates.size();
    sum.rasterChambers += raster.count;
    sum.worldChambers += result.chambers;
    sum.countSame += raster.count == result.chambers ? 1 : 0;
    // The chambers beside each cutting gate: the world lists the two of
    // its own numbering, the raster its own; compare by whether the gate
    // separates two chambers at all and, where it does, whether the same
    // cells lie in them (through the rasterised world chambers).
    // Every world chamber as the raster label most of its cells carry, and
    // its cells; a chamber of no cell is a sliver between walls.
    std::vector<std::map<std::uint32_t, std::size_t>> tally(result.chambers);
    std::vector<std::size_t> worldArea(result.chambers, 0);
    for (std::size_t cell = 0; cell < cells; ++cell) {
      const auto w = ofWorld.of[cell];
      if (w != NO_CHAMBER && w < result.chambers) {
        ++worldArea[w];
        if (raster.of[cell] != NO_CHAMBER) {
          ++tally[w][raster.of[cell]];
        }
      }
    }
    std::vector<std::size_t> rasterArea(raster.count, 0);
    for (const auto r : raster.of) {
      if (r != NO_CHAMBER && r < raster.count) {
        ++rasterArea[r];
      }
    }
    constexpr std::size_t MATERIAL = 300;
    std::vector<std::uint32_t> rasterOfWorld(result.chambers, NO_CHAMBER);
    for (std::uint32_t w = 0; w < result.chambers; ++w) {
      if (worldArea[w] == 0) {
        ++sum.worldEmptyChambers;
      }
      std::size_t best = 0;
      for (const auto& [r, n] : tally[w]) {
        if (n > best) {
          best = n;
          rasterOfWorld[w] = r;
        }
      }
    }
    std::size_t at = 0;
    for (std::size_t g = 0; g < result.gates.size(); ++g) {
      if (result.cuts[g] == 0) {
        continue;
      }
      const auto& rb = raster.beside[at];
      std::vector<std::uint32_t> wb;
      for (const auto ch : result.beside[g]) {
        if (ch != NO_CHAMBER) {
          wb.push_back(ch);
        }
      }
      const bool rTwo = rb.size() >= 2;
      const bool wTwo = wb.size() >= 2;
      {
        // The same question over the chambers that could hold a wire: those
        // of MATERIAL cells or more.
        std::vector<std::uint32_t> rm;
        for (const auto r : rb) {
          if (rasterArea[r] >= MATERIAL) {
            rm.push_back(r);
          }
        }
        std::vector<std::uint32_t> wm;
        for (const auto w : wb) {
          if (worldArea[w] >= MATERIAL) {
            wm.push_back(rasterOfWorld[w]);
          }
        }
        std::ranges::sort(wm);
        wm.erase(std::unique(wm.begin(), wm.end()), wm.end());
        if ((rm.size() >= 2) != (wm.size() >= 2)) {
          ++sum.materialSides;
          if (sum.materialSides <= 8) {
            std::printf("  MATERIAL pass %zu gate (%lld,%lld)-(%lld,%lld) width %.1f: raster {", k,
                        static_cast<long long>(result.gates[g].first % width),
                        static_cast<long long>(result.gates[g].first / width),
                        static_cast<long long>(result.gates[g].second % width),
                        static_cast<long long>(result.gates[g].second / width), result.width[g]);
            for (const auto r : rb) {
              std::printf(" %u(%zu)", r, rasterArea[r]);
            }
            std::printf(" } world {");
            for (const auto w : wb) {
              std::printf(" %u(%zu->%u)", w, worldArea[w], rasterOfWorld[w]);
            }
            std::printf(" }\n");
          }
        } else if (rm.size() >= 2 && rm != wm) {
          ++sum.materialChambers;
        } else {
          ++sum.materialSame;
        }
      }
      if (rTwo == wTwo) {
        ++sum.besideSame;
        if (rTwo) {
          std::vector<std::uint32_t> mapped;
          for (const auto w : wb) {
            mapped.push_back(rasterOfWorld[w]);
          }
          std::ranges::sort(mapped);
          mapped.erase(std::unique(mapped.begin(), mapped.end()), mapped.end());
          if (mapped == rb) {
            ++sum.bothTwoSame;
          } else {
            ++sum.bothTwoDiffer;
          }
        }
      } else {
        ++sum.besideDiff;
        if (wTwo) {
          ++sum.rasterOneWorldTwo;
          if (worldArea[wb[0]] == 0 || worldArea[wb[1]] == 0) {
            ++sum.worldSliverSide;
          }
        } else {
          ++sum.rasterTwoWorldOne;
          // Where does the world join what the raster parts? The cells of the
          // world chamber that carry the raster label fewer of them carry,
          // next to cells with the other label: the leak, drawn.
          if (sum.rasterTwoWorldOne <= 0 && !wb.empty()) {
            const auto w = wb[0];
            std::printf("  LEAK pass %zu gate (%lld,%lld)-(%lld,%lld): world chamber %u covers raster",
                        k, static_cast<long long>(result.gates[g].first % width),
                        static_cast<long long>(result.gates[g].first / width),
                        static_cast<long long>(result.gates[g].second % width),
                        static_cast<long long>(result.gates[g].second / width), w);
            for (const auto& [r, cnt] : tally[w]) {
              std::printf(" %u(%zu)", r, cnt);
            }
            std::printf("\n");
            const auto major = rasterOfWorld[w];
            std::optional<std::size_t> leak;
            for (std::size_t cell = 0; cell < cells && !leak; ++cell) {
              if (ofWorld.of[cell] != w || raster.of[cell] == NO_CHAMBER || raster.of[cell] == major) {
                continue;
              }
              const auto x = static_cast<std::int64_t>(cell % replay.width);
              const auto y = static_cast<std::int64_t>(cell / replay.width);
              for (std::int64_t dy = -3; dy <= 3 && !leak; ++dy) {
                for (std::int64_t dx = -3; dx <= 3; ++dx) {
                  const auto nx = x + dx;
                  const auto ny = y + dy;
                  if (nx < 0 || ny < 0 || nx >= width || ny >= static_cast<std::int64_t>(replay.height)) {
                    continue;
                  }
                  const auto n = static_cast<std::size_t>((ny * width) + nx);
                  if (raster.of[n] == major && ofWorld.of[n] == w) {
                    leak = cell;
                    break;
                  }
                }
              }
            }
            if (leak) {
              const auto lx = static_cast<std::int64_t>(*leak % replay.width);
              const auto ly = static_cast<std::int64_t>(*leak / replay.width);
              std::printf("  leak near (%lld,%lld); window: # wall, . raster line, letters raster "
                          "chambers (upper case where the world has chamber %u), ' ' other\n",
                          static_cast<long long>(lx), static_cast<long long>(ly), w);
              std::map<std::uint32_t, char> letter;
              for (std::int64_t dy = -8; dy <= 8; ++dy) {
                std::string row = "    ";
                for (std::int64_t dx = -14; dx <= 14; ++dx) {
                  const auto nx = lx + dx;
                  const auto ny = ly + dy;
                  if (nx < 0 || ny < 0 || nx >= width || ny >= static_cast<std::int64_t>(replay.height)) {
                    row += ' ';
                    continue;
                  }
                  const auto n = static_cast<std::size_t>((ny * width) + nx);
                  if (mask.test(n)) {
                    row += '#';
                    continue;
                  }
                  const auto r = raster.of[n];
                  char ch = '.';
                  if (r != NO_CHAMBER) {
                    if (!letter.contains(r)) {
                      letter[r] = static_cast<char>('a' + (letter.size() % 26));
                    }
                    ch = letter[r];
                    if (ofWorld.of[n] == w) {
                      ch = static_cast<char>(ch - 'a' + 'A');
                    }
                  } else if (ofWorld.of[n] == w) {
                    ch = ':';
                  }
                  row += ch;
                }
                std::printf("%s\n", row.c_str());
              }
              for (const auto& [r, ch] : letter) {
                std::printf("    %c = raster %u\n", ch, r);
              }
            }
          }
        }
        if (shown < 0) {
          ++shown;
          const auto& gate = result.gates[g];
          std::printf("  pass %zu gate (%lld,%lld)-(%lld,%lld) width %.1f: raster beside {", k,
                      static_cast<long long>(gate.first % width), static_cast<long long>(gate.first / width),
                      static_cast<long long>(gate.second % width), static_cast<long long>(gate.second / width),
                      result.width[g]);
          for (const auto r : rb) {
            std::printf(" %u", r);
          }
          std::printf(" }, world beside {");
          for (const auto w : wb) {
            std::printf(" %u(%zu cells, raster %u)", w, worldArea[w], rasterOfWorld[w]);
          }
          std::printf(" }\n");
        }
      }
      ++at;
    }
    // Samples of free cells: where both give a chamber, do consecutive
    // samples agree on same-or-different?
    std::uniform_int_distribution<std::size_t> pick(0, cells - 1);
    std::uint32_t lastR = NO_CHAMBER;
    std::uint32_t lastW = NO_CHAMBER;
    bool haveLast = false;
    std::vector<std::size_t> freeCells;
    for (int s = 0; s < 4000; ++s) {
      const auto cell = pick(rng);
      if (mask.test(cell)) {
        continue;
      }
      freeCells.push_back(cell);
      ++sum.samples;
      const auto r = raster.of[cell];
      const auto w = ofWorld.of[cell];
      if (r == NO_CHAMBER && w != NO_CHAMBER) {
        ++sum.rasterLineOnly;
      } else if (r != NO_CHAMBER && w == NO_CHAMBER) {
        ++sum.worldNoneOnly;
      } else if (r != NO_CHAMBER) {
        ++sum.bothChamber;
        if (haveLast) {
          ++sum.pairs;
          if ((r == lastR) == (w == lastW)) {
            ++sum.pairsAgree;
          }
        }
        lastR = r;
        lastW = w;
        haveLast = true;
      }
    }
    // Lookups in the world: random free cells, and a coherent walk along a
    // row of cells.
    {
      const auto f = Clock::now();
      std::size_t sink = 0;
      for (const auto cell : freeCells) {
        sink += world.chamberAt(*state, static_cast<std::uint32_t>(cell % replay.width),
                                static_cast<std::uint32_t>(cell / replay.width));
      }
      const auto g = Clock::now();
      const auto y = static_cast<std::uint32_t>(replay.height / 2);
      for (std::uint32_t x = 0; x < std::min<std::uint32_t>(replay.width, 2000); ++x) {
        sink += world.chamberAt(*state, x, y);
      }
      const auto h = Clock::now();
      sum.randomNs += 1e6 * ms(f, g) / std::max<std::size_t>(1, freeCells.size());
      sum.coherentNs += 1e6 * ms(g, h) / std::min<std::uint32_t>(replay.width, 2000);
      EXPECT_GE(sink, 0U);
    }
  }
  const auto n = static_cast<double>(std::max<std::size_t>(1, sum.passes));
  std::printf("chambers over %zu passes (%u x %u = %zu cells), %.1f cutting gates per pass:\n",
              sum.passes, replay.width, replay.height, cells, static_cast<double>(sum.gates) / n);
  std::printf("  raster chambersOfCached %.3f ms (moves alone %.3f ms; a fill of the cell array %.3f ms); "
              "world.chambers %.3f ms\n",
              sum.rasterMs / n, sum.movesMs / n, sum.fillMs / n, sum.worldMs / n);
  std::printf("  world.chamberAt: %.0f ns per random free cell, %.0f ns per cell along a row\n",
              sum.randomNs / n, sum.coherentNs / n);
  std::printf("  chambers: raster %.1f, world %.1f per pass, the same count in %zu of %zu passes\n",
              static_cast<double>(sum.rasterChambers) / n, static_cast<double>(sum.worldChambers) / n,
              sum.countSame, sum.passes);
  std::printf("  gates separating two chambers: same answer %zu (of these, both two and the same "
              "chambers %zu, other chambers %zu), different %zu (raster one, world two: %zu, of which "
              "a world side is a sliver of no cell %zu; raster two, world one: %zu); world chambers of "
              "no cell %.1f per pass\n",
              sum.besideSame, sum.bothTwoSame, sum.bothTwoDiffer, sum.besideDiff,
              sum.rasterOneWorldTwo, sum.worldSliverSide, sum.rasterTwoWorldOne,
              static_cast<double>(sum.worldEmptyChambers) / n);
  std::printf("  over chambers of %zu cells or more: gates with the same answer %zu, a different "
              "number of sides %zu, two sides but other chambers %zu\n",
              std::size_t{300}, sum.materialSame, sum.materialSides, sum.materialChambers);
  std::printf("  free cells sampled %zu: both in a chamber %zu, on a raster line only %zu, in no world "
              "chamber only %zu; consecutive pairs agreeing on same/different chamber %zu of %zu\n",
              sum.samples, sum.bothChamber, sum.rasterLineOnly, sum.worldNoneOnly, sum.pairsAgree,
              sum.pairs);
}

TEST(CdtWorld, DISABLED_Replay) {
  const char* path = std::getenv("MQT_SCPD_CDT_REPLAY");
  if (path == nullptr || *path == '\0') {
    GTEST_SKIP() << "MQT_SCPD_CDT_REPLAY names no file";
  }
  Replay replay;
  ASSERT_TRUE(loadReplay(path, replay)) << "cannot read " << path;
  const auto cacheSize = envOr("MQT_SCPD_CDT_CACHE", 32);
  const auto oracleEvery = envOr("MQT_SCPD_CDT_ORACLE_EVERY", 50);
  const auto limit = envOr("MQT_SCPD_CDT_LIMIT", 0);
  const auto onlyChain = envOr("MQT_SCPD_CDT_CHAIN", std::numeric_limits<std::uint32_t>::max());
  std::string strategies = "pinned,cache";
  if (const char* s = std::getenv("MQT_SCPD_CDT_STRATEGIES"); s != nullptr && *s != '\0') {
    strategies = s;
  }
  std::vector<ReplayPass> passes;
  for (const auto& pass : replay.passes) {
    if (onlyChain != std::numeric_limits<std::uint32_t>::max() && pass.chain != onlyChain) {
      continue;
    }
    passes.push_back(pass);
    if (limit != 0 && passes.size() >= limit) {
      break;
    }
  }
  std::printf("replay %s: %u x %u, %zu objects, %zu passes (%zu played), %zu refused by the router\n",
              path, replay.width, replay.height, replay.objects.size(), replay.passes.size(),
              passes.size(),
              static_cast<std::size_t>(std::ranges::count_if(
                  replay.passes, [](const ReplayPass& p) { return !p.gates.has_value(); })));
  using Clock = std::chrono::steady_clock;

  std::vector<Strategy> results;
  std::stringstream names(strategies);
  std::string name;
  while (std::getline(names, name, ',')) {
    Strategy st{.name = name};
    const auto t0 = Clock::now();
    CdtWorld world(replay.width, replay.height, replay.mask, replay.owner, replay.statics, 0,
                   replay.options, replay.cuts, cacheSize);
    const auto t1 = Clock::now();
    // Every object is added before the passes, as the router adds it the
    // first time it is seen; the ids of the replay are the world's.
    for (const auto& object : replay.objects) {
      ASSERT_EQ(world.add(object), world.objectCount() - 1);
    }
    const auto t2 = Clock::now();
    const auto addMs = world.addSeconds();
    std::printf("[%s] world built in %.1f ms, %zu objects added in %.1f ms (walls %.0f, stamp %.0f, "
                "squares %.0f, points near %.0f, outline %.0f, index %.0f, feet %.0f, corner feet %.0f, "
                "polygon %.0f, pairs %.0f)\n",
                name.c_str(), std::chrono::duration<double, std::milli>(t1 - t0).count(),
                replay.objects.size(),
                std::chrono::duration<double, std::milli>(t2 - t1).count(), addMs[0], addMs[1],
                addMs[2], addMs[3], addMs[4], addMs[5], addMs[6], addMs[7], addMs[8], addMs[9]);
    // The router's pinned state per chain.
    std::uint32_t pinnedChain = std::numeric_limits<std::uint32_t>::max();
    std::vector<std::uint32_t> pinnedIds;
    CdtWorld::StatePtr pinned;
    std::size_t oracleChecked = 0;
    for (std::size_t k = 0; k < passes.size(); ++k) {
      const auto& pass = passes[k];
      Gates gates;
      const auto a = Clock::now();
      CdtWorld::StatePtr state;
      try {
        if (name == "pinned") {
          if (pinnedChain != pass.chain || !pinned) {
            pinnedChain = pass.chain;
            pinnedIds = pass.ids;
            pinned = world.with(world.base(), pinnedIds);
          } else {
            std::vector<std::uint32_t> common;
            std::ranges::set_intersection(pinnedIds, pass.ids, std::back_inserter(common));
            if (common.size() != pinnedIds.size()) {
              pinnedIds = std::move(common);
              pinned = world.with(world.base(), pinnedIds);
            }
          }
          st.inserted += pass.ids.size() - pinnedIds.size();
          state = world.with(pinned, pass.ids);
        } else if (name == "cache") {
          state = world.state(pass.ids);
        } else if (name == "scratch") {
          state = world.fromScratch(pass.ids);
        } else {
          FAIL() << "unknown strategy " << name;
        }
        gates = gatesOf(world, *state);
      } catch (const std::exception& error) {
        gates.refused = true;
        ++st.refused;
        if (st.refused <= 3) {
          std::printf("[%s] pass %zu (chain %u) refused: %s\n", name.c_str(), k, pass.chain,
                      error.what());
        }
      }
      const auto b = Clock::now();
      st.ms.push_back(std::chrono::duration<double, std::milli>(b - a).count());
      if (state) {
        const auto& s = world.statsOf(*state);
        st.laps.copy += s.copy;
        st.laps.insert += s.insert;
        st.laps.pairs += s.pairs;
        st.laps.vertices += s.vertices;
        st.laps.edges += s.edges;
        st.laps.patch += s.patch;
        st.laps.classify += s.classify;
        st.laps.candidates += s.candidates;
        st.laps.basins += s.basins;
        st.laps.measure += s.measure;
        st.laps.chambers += s.chambers;
        st.laps.output += s.output;
        st.laps.newVertices += s.newVertices;
        st.laps.changedTriangles += s.changedTriangles;
        st.laps.reclassified += s.reclassified;
      }
      if (!gates.refused && oracleEvery != 0 && k % oracleEvery == 0 && name != "scratch") {
        try {
          const auto oracle = world.fromScratch(pass.ids);
          ++oracleChecked;
          if (gatesOf(world, *oracle) != gates) {
            ++st.mismatches;
            if (st.mismatches <= 3) {
              std::printf("[%s] pass %zu (chain %u): gates differ from scratch (%zu vs %zu)\n",
                          name.c_str(), k, pass.chain, gates.list.size(),
                          gatesOf(world, *oracle).list.size());
            }
          }
        } catch (const std::exception&) {
          // The oracle refused what the strategy built: nothing to compare.
        }
      }
      st.gates.push_back(std::move(gates));
      if (state && k + 1 == passes.size()) {
        std::printf("[%s] last state: %s\n", name.c_str(), world.describeState(*state).c_str());
      }
    }
    const auto n = static_cast<double>(std::max<std::size_t>(st.ms.size(), 1));
    const auto total = std::accumulate(st.ms.begin(), st.ms.end(), 0.0);
    const auto& cache = world.cacheStats();
    std::printf("[%s] %zu passes: mean %.3f ms, median %.3f, p95 %.3f, max %.3f, sum %.1f ms; "
                "%zu refused, %zu of %zu oracle checks differ; cache hits %zu extended %zu "
                "fromBase %zu, %.1f objects inserted per pass\n",
                name.c_str(), st.ms.size(), total / n, percentile(st.ms, 0.5),
                percentile(st.ms, 0.95), percentile(st.ms, 1.0), total, st.refused,
                st.mismatches, oracleChecked, cache.hits, cache.extended, cache.fromBase,
                static_cast<double>(name == "cache" ? cache.inserted : st.inserted) / n);
    std::printf("[%s] laps ms per pass: copy %.3f insert %.3f (pairs %.3f, vertices %.3f, "
                "edges %.3f) patch %.3f classify %.3f "
                "candidates %.3f basins %.3f measure %.3f chambers %.3f output %.3f; %.0f new "
                "vertices, %.0f changed triangles, %.0f reclassified per pass\n",
                name.c_str(), st.laps.copy / n, st.laps.insert / n, st.laps.pairs / n,
                st.laps.vertices / n, st.laps.edges / n, st.laps.patch / n,
                st.laps.classify / n, st.laps.candidates / n, st.laps.basins / n,
                st.laps.measure / n, st.laps.chambers / n, st.laps.output / n,
                static_cast<double>(st.laps.newVertices) / n,
                static_cast<double>(st.laps.changedTriangles) / n,
                static_cast<double>(st.laps.reclassified) / n);
    EXPECT_EQ(st.mismatches, 0U) << name << ": gates differ from a state built from scratch";
    results.push_back(std::move(st));
  }
  // Every strategy gives the same gates for every pass it did not refuse.
  for (std::size_t s = 1; s < results.size(); ++s) {
    std::size_t differ = 0;
    for (std::size_t k = 0; k < passes.size(); ++k) {
      const auto& a = results[0].gates[k];
      const auto& b = results[s].gates[k];
      if (!a.refused && !b.refused && a != b) {
        ++differ;
        if (differ <= 3) {
          std::printf("pass %zu (chain %u): %s gives %zu gates, %s gives %zu\n", k,
                      passes[k].chain, results[0].name.c_str(), a.list.size(),
                      results[s].name.c_str(), b.list.size());
        }
      }
    }
    EXPECT_EQ(differ, 0U) << results[0].name << " and " << results[s].name << " differ";
  }
}

} // namespace mqt::scpd::grid
