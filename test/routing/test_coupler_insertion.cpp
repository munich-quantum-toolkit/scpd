/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "../SplitMix.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using mqt::scpd::test::SplitMix;
using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint32_t WIDTH = 1200;
constexpr uint32_t HEIGHT = 1200;

/// The move primitives of bend radius 5, built once for every test.
const MovePrimitives& primitives() {
  static const MovePrimitives PRIMITIVES(5);
  return PRIMITIVES;
}

/// A straight run of a heading from a cell, one step per cell.
Path straightRun(const uint32_t x0, const uint32_t y0, const Heading heading,
                 const uint32_t steps) {
  return routing::straightRun(primitives(),
                              {.x = x0, .y = y0, .heading = heading}, steps);
}

/// Appends a quarter turn in the format of a routed path. The last point of
/// the path becomes the start of the arc, the other swept cells follow, and
/// the path ends on the end of the arc, tagged with the straight step of the
/// exit heading.
void appendTurn(Path& path, const Heading from, const Heading to) {
  const PathPoint start = path.back();
  path.pop_back();
  for (const Primitive& p : primitives().of(from)) {
    if (p.exitHeading == to) {
      const PathPoint end{.x = static_cast<uint32_t>(start.x + p.dx),
                          .y = static_cast<uint32_t>(start.y + p.dy),
                          .heading = to,
                          .primitive = primitives().straight(to)};
      for (const CellOffset& c : p.swept) {
        path.push_back({.x = static_cast<uint32_t>(start.x + c.dx),
                        .y = static_cast<uint32_t>(start.y + c.dy),
                        .heading = from,
                        .primitive = p.id});
      }
      if (path.back().samePlace(end)) {
        path.pop_back();
      }
      path.push_back(end);
      return;
    }
  }
}

/// A hairpin: three hundred cells east along y = 300, a U-turn toward
/// positive y, and three hundred cells back west ten cells beside the way out.
Path hairpin() {
  Path path = straightRun(400, 300, 6, 300);
  appendTurn(path, 6, 4);
  appendTurn(path, 4, 2);
  const PathPoint end = path.back();
  const Path back = straightRun(end.x, end.y, 2, 300);
  path.insert(path.end(), back.begin() + 1, back.end());
  return path;
}

/// A dogleg moved to where none of its cells has a negative coordinate.
Path placed(const Path& dogleg) {
  constexpr int32_t origin = 1000;
  Path path;
  for (const PathPoint& point : dogleg) {
    path.push_back(
        {.x = static_cast<uint32_t>(origin + static_cast<int32_t>(point.x)),
         .y = static_cast<uint32_t>(origin + static_cast<int32_t>(point.y)),
         .heading = point.heading,
         .primitive = point.primitive});
  }
  return path;
}

/// The angle between a piece of a polyline and a heading, in degrees.
double degreesOff(const Point& from, const Point& to, const Heading heading) {
  const HeadingVector v = headingVector(heading);
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  return std::abs(
             std::atan2((dx * v.dy) - (dy * v.dx), (dx * v.dx) + (dy * v.dy))) *
         180.0 / std::numbers::pi;
}

/// Whether a path occupies a cell twice other than in two consecutive points
/// or in a spur that steps off a cell and back onto it. A primitive whose last
/// swept cell lies past its end leaves such a spur, as in a routed path.
bool touchesItself(const Path& path) {
  std::map<std::pair<uint32_t, uint32_t>, std::size_t> lastVisit;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto [it, fresh] = lastVisit.try_emplace({path[i].x, path[i].y}, i);
    if (!fresh) {
      if (it->second + 2 < i) {
        return true;
      }
      it->second = i;
    }
  }
  return false;
}

/// The number of points at the end of a spliced path that equal the points at
/// the end of the path before the splice.
std::size_t sharedEnd(const Path& original, const Path& spliced) {
  std::size_t n = 0;
  while (n < original.size() && n < spliced.size() &&
         original[original.size() - 1 - n] == spliced[spliced.size() - 1 - n]) {
    ++n;
  }
  return n;
}

/// The index of a straight point of a spliced path from which on the path
/// equals the path before the splice. The last points of a connection can
/// equal points of the path before the splice, so the shared end can start
/// on a turn. The first straight point of the shared end is then a point of
/// the straight run the connection joins.
std::size_t junctionOf(const MovePrimitives& table, const Path& original,
                       const Path& spliced) {
  std::size_t at = spliced.size() - sharedEnd(original, spliced);
  while (at < spliced.size() &&
         !table.isStraight(spliced[at].heading, spliced[at].primitive)) {
    ++at;
  }
  return at;
}

/// The largest step along either axis between two consecutive points of a
/// path, up to the point at index @p last.
int64_t largestStep(const Path& path, const std::size_t last) {
  int64_t largest = 0;
  for (std::size_t i = 1; i <= last && i < path.size(); ++i) {
    const int64_t dx =
        static_cast<int64_t>(path[i].x) - static_cast<int64_t>(path[i - 1].x);
    const int64_t dy =
        static_cast<int64_t>(path[i].y) - static_cast<int64_t>(path[i - 1].y);
    largest = std::max({largest, std::abs(dx), std::abs(dy)});
  }
  return largest;
}

/// The rendered length of a spliced path minus the rendered lengths of its two
/// parts: up to the junction, and the path before the splice from the
/// junction on. It is zero when the dogleg leaves the rendering of the
/// remaining path as it was, and infinite without a junction.
double lengthOffParts(const MovePrimitives& table, const Path& original,
                      const Path& spliced) {
  const std::size_t junction = junctionOf(table, original, spliced);
  if (junction >= spliced.size()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto shared = static_cast<std::ptrdiff_t>(spliced.size() - junction);
  const Path front(spliced.begin(),
                   spliced.begin() + static_cast<std::ptrdiff_t>(junction) + 1);
  const Path rest(original.end() - shared, original.end());
  return renderedLength(table, spliced) - renderedLength(table, front) -
         renderedLength(table, rest);
}

TEST(CouplerInsertion, ADoglegTurnsOnceAndThenRunsStraight) {
  const DoglegGeometry dogleg = buildDogleg(primitives(), 6, -1, 12);
  // A quarter turn against the clock from heading 6 ends on heading 4.
  EXPECT_EQ(dogleg.tip.heading, 4);
  EXPECT_EQ(dogleg.tip.primitive, primitives().straight(4));
  EXPECT_NEAR(dogleg.cost, (std::numbers::pi / 2.0 * 5.0) + 12.0, 0.5);
  ASSERT_FALSE(dogleg.path.empty());
  EXPECT_EQ(dogleg.path.back().x, dogleg.tip.x);
  EXPECT_EQ(dogleg.path.back().y, dogleg.tip.y);
  // The straight run leaves along the exit heading.
  const HeadingVector v = headingVector(4);
  const PathPoint& afterTurn = dogleg.path[dogleg.path.size() - 13];
  EXPECT_EQ(static_cast<int64_t>(dogleg.tip.x) - afterTurn.x, v.dx * 12);
  EXPECT_EQ(static_cast<int64_t>(dogleg.tip.y) - afterTurn.y, v.dy * 12);

  // Without a straight step, the tip is the end of the turn. Its tag names a
  // move of its own heading, the straight step, as for any other tip.
  const DoglegGeometry clockwise = buildDogleg(primitives(), 6, 1, 0);
  EXPECT_EQ(clockwise.tip.heading, 0);
  EXPECT_EQ(clockwise.tip.primitive, primitives().straight(0));
  const Primitive* turn =
      primitives().find(6, clockwise.path.front().primitive);
  ASSERT_NE(turn, nullptr);
  EXPECT_EQ(clockwise.tip.x, static_cast<uint32_t>(turn->dx));
  EXPECT_EQ(clockwise.tip.y, static_cast<uint32_t>(turn->dy));
  EXPECT_THROW(static_cast<void>(buildDogleg(primitives(), 6, 0, 4)),
               std::invalid_argument);
}

TEST(CouplerInsertion, ADoglegCostsTheLengthOfItsRenderedPath) {
  // A diagonal step is the square root of two cells long, and the turn counts
  // the curve samplePath() draws for it. The rendering starts the turn on the
  // first point of the path, so that point must be the start of the turn at
  // every radius.
  for (uint32_t radius = 1; radius <= MovePrimitives::MAX_BEND_RADIUS;
       ++radius) {
    const MovePrimitives table(radius);
    for (Heading entry = 0; entry < NUM_HEADINGS; ++entry) {
      for (const int sign : {-1, 1}) {
        for (const uint32_t run : {0U, 14U}) {
          const DoglegGeometry dogleg = buildDogleg(table, entry, sign, run);
          EXPECT_NEAR(dogleg.cost, renderedLength(table, placed(dogleg.path)),
                      1e-9)
              << "radius " << radius << ", entry " << static_cast<int>(entry)
              << ", turn " << sign << ", run " << run;
        }
      }
    }
  }
}

TEST(CouplerInsertion, TheTurnOfADoglegEndsAlongItsRun) {
  // At some radii a diagonal heading holds an arc of 72 to 77 degrees beside
  // the exact quarter turn. The dogleg takes the exact quarter turn, so its
  // rendered curve meets the straight run without a kink: over its last
  // tenth of a cell, the curve runs along the run.
  for (const uint32_t radius : {5U, 10U, 13U, 16U}) {
    const MovePrimitives table(radius);
    for (Heading entry = 1; entry < NUM_HEADINGS; entry += 2) {
      for (const int sign : {-1, 1}) {
        const DoglegGeometry dogleg = buildDogleg(table, entry, sign, 4);
        const Primitive* turn =
            table.find(entry, dogleg.path.front().primitive);
        ASSERT_NE(turn, nullptr);
        Path path = placed(dogleg.path);
        const double endX = path.front().x + turn->dx;
        const double endY = path.front().y + turn->dy;
        std::vector<PathSegment> segments;
        const std::vector<Point> points =
            samplePath(table, path, path.front(), segments);
        // The rendering of the turn ends on the end cell of the turn.
        const auto atEnd = std::ranges::find_if(points, [&](const Point& p) {
          return std::hypot(p.x() - endX, p.y() - endY) < 1e-9;
        });
        ASSERT_NE(atEnd, points.end());
        const auto end = static_cast<std::size_t>(atEnd - points.begin());
        ASSERT_GT(end, 0U);
        std::size_t from = end - 1;
        while (from > 0 &&
               std::hypot(points[end].x() - points[from].x(),
                          points[end].y() - points[from].y()) < 0.1) {
          --from;
        }
        EXPECT_LT(
            degreesOff(points[from], points[end], turned(entry, 2 * sign)), 3.0)
            << "radius " << radius << ", entry " << static_cast<int>(entry)
            << ", turn " << sign;
      }
    }
  }

  // At a radius of five cells, each side holds one quarter turn.
  for (Heading entry = 0; entry < NUM_HEADINGS; ++entry) {
    for (const int sign : {-1, 1}) {
      const Heading exit = turned(entry, 2 * sign);
      const auto quarters = std::ranges::count_if(
          primitives().of(entry),
          [&](const Primitive& p) { return p.exitHeading == exit; });
      EXPECT_EQ(quarters, 1) << static_cast<int>(entry) << " " << sign;
    }
  }
}

TEST(CouplerInsertion, ADoglegStepsFromCellToNeighboringCell) {
  // The splice tests the cells of a dogleg against the rest of the path. A
  // dogleg that skipped a cell, such as the end of its turn, would let the
  // path cross it there unseen. At every radius the dogleg lists the end of
  // its turn. At the radii that buildDogleg() names, it skips no cell.
  for (uint32_t radius = 1; radius <= MovePrimitives::MAX_BEND_RADIUS;
       ++radius) {
    const bool chain = radius <= 24 || (radius >= 26 && radius <= 32) ||
                       (radius >= 37 && radius <= 40) || radius == 50;
    const MovePrimitives table(radius);
    for (Heading entry = 0; entry < NUM_HEADINGS; ++entry) {
      for (const int sign : {-1, 1}) {
        const DoglegGeometry dogleg = buildDogleg(table, entry, sign, 3);
        const Primitive* turn =
            table.find(entry, dogleg.path.front().primitive);
        ASSERT_NE(turn, nullptr);
        const auto end =
            std::ranges::find_if(dogleg.path, [&](const PathPoint& point) {
              return point.samePlace({.x = static_cast<uint32_t>(turn->dx),
                                      .y = static_cast<uint32_t>(turn->dy)});
            });
        EXPECT_NE(end, dogleg.path.end())
            << "radius " << radius << ", entry " << static_cast<int>(entry)
            << ", turn " << sign;
        if (!chain) {
          continue;
        }
        for (std::size_t i = 1; i < dogleg.path.size(); ++i) {
          const auto dx =
              static_cast<int32_t>(dogleg.path[i].x - dogleg.path[i - 1].x);
          const auto dy =
              static_cast<int32_t>(dogleg.path[i].y - dogleg.path[i - 1].y);
          EXPECT_LE(std::max(std::abs(dx), std::abs(dy)), 1)
              << "radius " << radius << ", entry " << static_cast<int>(entry)
              << ", turn " << sign << ", point " << i;
        }
      }
    }
  }
}

TEST(CouplerInsertion, TheSpliceLandsWhereTheRestOfThePathHitsTheTarget) {
  // A long straight resonator: the coupler is spliced so that the path from
  // it to the far end is the target length.
  Path path = straightRun(400, 300, 6, 300);
  ASSERT_GT(renderedLength(primitives(), path), 250.0);

  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(primitives(), 150.0, path, WIDTH, HEIGHT, 0);
  ASSERT_TRUE(splice.has_value());
  EXPECT_TRUE(splice->inAllowedArea);
  EXPECT_EQ(path.front().x, splice->anchor.x);
  EXPECT_EQ(path.front().y, splice->anchor.y);

  // The path now runs from the coupler to the unchanged far end, and its
  // length is the target within the step the splice can place it on.
  EXPECT_EQ(path.back().x, 700U);
  EXPECT_EQ(path.back().y, 300U);
  EXPECT_NEAR(renderedLength(primitives(), path), 150.0, 1.0);
}

TEST(CouplerInsertion, TheSplicePrefersAnUndershoot) {
  Path path = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(primitives(), 150.0, path, WIDTH, HEIGHT, 0);
  ASSERT_TRUE(splice.has_value());
  // A resonator that is a little short can be lengthened by a meander; one
  // that is too long cannot be shortened, so the splice never overshoots by
  // more than half the step it had to choose between.
  EXPECT_LE(renderedLength(primitives(), path), 150.5);
}

TEST(CouplerInsertion, OfEqualChargesTheCandidateEarliestAlongThePathWins) {
  // Three candidates on this diagonal resonator give the same length. Two of
  // them join the resonator a cell after the first, with connections one
  // diagonal step longer. The first and one of the two later ones put the
  // anchor on the same cell, the third two cells away. All three overshoot
  // the target by the same length, so their charges are equal, and the first
  // wins whatever the rounding of their lengths.
  const MovePrimitives table(10);
  const Path resonator =
      routing::straightRun(table, {.x = 120, .y = 120, .heading = 5}, 120);
  const CouplerDoglegOptions options{.straightLength = 9, .mirrored = true};
  constexpr double target = 192.2;
  Path first = resonator;
  const std::optional<CouplerSplice> winner =
      spliceCouplerDogleg(table, target, first, WIDTH, HEIGHT, 7, options);
  ASSERT_TRUE(winner.has_value());
  EXPECT_GT(renderedLength(table, first), target);

  // With the anchor of the winner forbidden, the third candidate wins. It
  // gives the same length and keeps one point less of the resonator.
  Path third = resonator;
  const std::optional<CouplerSplice> other = spliceCouplerDogleg(
      table, target, third, WIDTH, HEIGHT, 7, options,
      [&](const uint32_t x, const uint32_t y) {
        return x != winner->anchor.x || y != winner->anchor.y;
      });
  ASSERT_TRUE(other.has_value());
  EXPECT_TRUE(other->inAllowedArea);
  EXPECT_NEAR(renderedLength(table, third), renderedLength(table, first), 1e-9);
  EXPECT_EQ(sharedEnd(resonator, first), sharedEnd(resonator, third) + 1);
}

TEST(CouplerInsertion, TheRenderedSpliceMeetsItsTarget) {
  // The splice measures the remaining path and the dogleg as samplePath()
  // renders them. The candidates of a straight run lie one step apart, so the
  // best one misses the target by at most half a step. An overshoot is that
  // best one, and an undershoot wins only within twice its miss. A diagonal
  // step is the square root of two cells long. Every coupler heading is tested
  // whose dogleg does not end against the resonator, where only a dogleg at
  // an end of the resonator fits.
  for (const Heading heading : {Heading{6}, Heading{7}}) {
    const double step = isDiagonal(heading) ? std::numbers::sqrt2 : 1.0;
    for (Heading couplerHeading = 0; couplerHeading < NUM_HEADINGS;
         ++couplerHeading) {
      for (const bool mirrored : {false, true}) {
        if ((mirrored ? reverse(couplerHeading) : couplerHeading) ==
            reverse(heading)) {
          continue;
        }
        for (const uint32_t lead : {0U, 8U}) {
          for (int eighth = 0; eighth <= 16; ++eighth) {
            const double target = 150.0 + (0.125 * eighth);
            Path path = straightRun(400, 400, heading, 300);
            ASSERT_TRUE(spliceCouplerDogleg(
                            primitives(), target, path, WIDTH, HEIGHT,
                            couplerHeading,
                            {.leadStraight = lead, .mirrored = mirrored})
                            .has_value());
            const double miss = renderedLength(primitives(), path) - target;
            EXPECT_LE(miss, (step / 2.0) + 1e-9)
                << static_cast<int>(heading) << " "
                << static_cast<int>(couplerHeading) << " " << mirrored << " "
                << lead << " " << target;
            EXPECT_GE(miss, -step - 1e-9)
                << static_cast<int>(heading) << " "
                << static_cast<int>(couplerHeading) << " " << mirrored << " "
                << lead << " " << target;
          }
        }
      }
    }
  }
}

TEST(CouplerInsertion, TheSpliceCutsAtTheCandidatesOwnPoint) {
  // This route turns from east onto a diagonal with an eighth turn that
  // sweeps a cell past its end. The end of the turn is a point of the turn
  // and, two points later, the first point of the diagonal run. A splice onto
  // that run cell keeps the run from the second point on, so the spliced path
  // neither turns on the spot nor steps back.
  constexpr uint32_t width = 600;
  constexpr uint32_t height = 400;
  auto shared = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(width, height);
  const grid::BitGrid corridor(width, height);
  DubinsRouter router(shared, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&corridor);
  Path path = router.route(
      {.source = {.x = 50, .y = 100, .heading = 6, .primitive = 0},
       .target = {.x = 500, .y = 139, .heading = 6, .primitive = 0}});
  ASSERT_FALSE(path.empty());

  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(*shared, 100.0, path, width, height, 0);
  ASSERT_TRUE(splice.has_value());
  // The heading changes four times: in the quarter turn of the dogleg, in the
  // eighth and the quarter turn that join the dogleg to the diagonal run, and
  // where the route leaves the diagonal.
  EXPECT_EQ(countBends(path), 4U);
  Path rendered = path;
  std::vector<PathSegment> segments;
  const std::vector<Point> points =
      samplePath(*shared, rendered, rendered.front(), segments);
  std::size_t previous = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x() - points[i - 1].x();
    const double dy = points[i].y() - points[i - 1].y();
    if (std::hypot(dx, dy) < 1e-9) {
      continue;
    }
    if (previous > 0) {
      const double px = points[previous].x() - points[previous - 1].x();
      const double py = points[previous].y() - points[previous - 1].y();
      EXPECT_GT((px * dx) + (py * dy), 0.0)
          << "reversal at (" << points[i - 1].x() << ", " << points[i - 1].y()
          << ")";
    }
    previous = i;
  }
  EXPECT_NEAR(polylineLength(points), 100.0, std::numbers::sqrt2);
}

TEST(CouplerInsertion, ASpliceOnTheStubOfAFirstTurnKeepsTheStartOfItsArc) {
  // Where the search of a routed path begins with a turn, the last cell of
  // the source stub is the start of the arc and keeps the straight tag of the
  // stub (see Path). Each path here is such a path: twelve diagonal steps of
  // stub, then an exact quarter turn, which ends the path. The stub is the
  // only straight run, and a target of zero puts the splice on its last cell.
  // The point there must stay the point before the arc, so that the turn
  // renders from it: the spliced path renders as long as its two parts.
  for (Heading stub = 1; stub < NUM_HEADINGS; stub += 2) {
    for (const uint16_t id : {900, 901}) {
      const Primitive* turn = primitives().find(stub, id);
      ASSERT_NE(turn, nullptr);
      Path path = straightRun(300, 300, stub, 12);
      const PathPoint start = path.back();
      const PathPoint end{
          .x = static_cast<uint32_t>(static_cast<int32_t>(start.x) + turn->dx),
          .y = static_cast<uint32_t>(static_cast<int32_t>(start.y) + turn->dy),
          .heading = turn->exitHeading,
          .primitive = primitives().straight(turn->exitHeading)};
      for (std::size_t k = 1; k < turn->swept.size(); ++k) {
        const PathPoint cell{
            .x = static_cast<uint32_t>(static_cast<int32_t>(start.x) +
                                       turn->swept[k].dx),
            .y = static_cast<uint32_t>(static_cast<int32_t>(start.y) +
                                       turn->swept[k].dy),
            .heading = stub,
            .primitive = id};
        if (!cell.samePlace(end)) {
          path.push_back(cell);
        }
      }
      path.push_back(end);
      // The path from the last stub cell on.
      const Path rest(path.begin() + 12, path.end());

      for (Heading couplerHeading = 0; couplerHeading < NUM_HEADINGS;
           ++couplerHeading) {
        for (const bool mirrored : {false, true}) {
          Path spliced = path;
          ASSERT_TRUE(
              spliceCouplerDogleg(primitives(), 0.0, spliced, WIDTH, HEIGHT,
                                  couplerHeading,
                                  {.straightLength = 3, .mirrored = mirrored})
                  .has_value());
          ASSERT_GT(spliced.size(), rest.size());
          const std::size_t junction = spliced.size() - rest.size();
          EXPECT_TRUE(std::equal(rest.begin(), rest.end(),
                                 spliced.begin() +
                                     static_cast<std::ptrdiff_t>(junction)));
          EXPECT_FALSE(spliced[junction - 1].samePlace(spliced[junction]))
              << static_cast<int>(stub) << " " << id << " "
              << static_cast<int>(couplerHeading) << " " << mirrored;
          const Path front(spliced.begin(),
                           spliced.begin() +
                               static_cast<std::ptrdiff_t>(junction) + 1);
          EXPECT_NEAR(renderedLength(primitives(), spliced),
                      renderedLength(primitives(), front) +
                          renderedLength(primitives(), rest),
                      1e-9)
              << static_cast<int>(stub) << " " << id << " "
              << static_cast<int>(couplerHeading) << " " << mirrored;
        }
      }
    }
  }
}

TEST(CouplerInsertion, AnAllowedAreaMovesTheCouplerAlongItsResonator) {
  Path unrestricted = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> free =
      spliceCouplerDogleg(primitives(), 150.0, unrestricted, WIDTH, HEIGHT, 0);
  ASSERT_TRUE(free.has_value());

  // The window excludes where the coupler would otherwise land, so it
  // slides along the resonator instead of the placement being lost.
  Path restricted = straightRun(400, 300, 6, 300);
  const uint32_t forbidden = free->anchor.x;
  const std::optional<CouplerSplice> moved =
      spliceCouplerDogleg(primitives(), 150.0, restricted, WIDTH, HEIGHT, 0, {},
                          [&](const uint32_t x, const uint32_t) {
                            return x < forbidden - 20 || x > forbidden + 20;
                          });
  ASSERT_TRUE(moved.has_value());
  EXPECT_TRUE(moved->inAllowedArea);
  EXPECT_TRUE(moved->anchor.x < forbidden - 20 ||
              moved->anchor.x > forbidden + 20);
}

TEST(CouplerInsertion, AnUnreachableWindowStillPlacesTheCoupler) {
  Path path = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(primitives(), 150.0, path, WIDTH, HEIGHT, 0, {},
                          [](uint32_t, uint32_t) { return false; });
  // The best collision-free placement is used anyway, and the caller is
  // told that it is outside the window, which is a named miss rather than a
  // failed run.
  ASSERT_TRUE(splice.has_value());
  EXPECT_FALSE(splice->inAllowedArea);
}

TEST(CouplerInsertion, ASecondDoglegOffersAnotherPlacement) {
  Path single = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> one = spliceCouplerDogleg(
      primitives(), 150.0, single, WIDTH, HEIGHT, 0, {.straightLength = 14});
  ASSERT_TRUE(one.has_value());

  // The S-jog turns back on itself, so it reaches across the resonator and
  // puts the coupler somewhere the single dogleg cannot reach.
  Path jogged = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> jog =
      spliceCouplerDogleg(primitives(), 150.0, jogged, WIDTH, HEIGHT, 0,
                          {.straightLength = 14,
                           .secondStraightLength = 10,
                           .secondTurnReverse = true});
  ASSERT_TRUE(jog.has_value());
  EXPECT_NE(jog->anchor.x, one->anchor.x);
  EXPECT_NEAR(renderedLength(primitives(), jogged), 150.0, 1.0);

  // The hook turns twice the same way, so its second straight run lies
  // along the resonator: on a straight wire it lands the coupler exactly
  // where the single dogleg does, only further along the wire.
  Path hooked = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> hook =
      spliceCouplerDogleg(primitives(), 150.0, hooked, WIDTH, HEIGHT, 0,
                          {.straightLength = 14,
                           .secondStraightLength = 10,
                           .secondTurnReverse = false});
  ASSERT_TRUE(hook.has_value());
  EXPECT_EQ(hook->anchor.x, one->anchor.x);
  EXPECT_EQ(hook->anchor.y, one->anchor.y);
}

TEST(CouplerInsertion, TheMirroredDoglegTurnsTheOtherWay) {
  Path plain = straightRun(400, 300, 6, 300);
  Path mirrored = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> a = spliceCouplerDogleg(
      primitives(), 150.0, plain, WIDTH, HEIGHT, 0, {.mirrored = false});
  const std::optional<CouplerSplice> b = spliceCouplerDogleg(
      primitives(), 150.0, mirrored, WIDTH, HEIGHT, 0, {.mirrored = true});
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  // The two doglegs leave the resonator on opposite sides.
  EXPECT_NE(a->anchor.y, b->anchor.y);
}

TEST(CouplerInsertion,
     TheLeadRunsStraightFromTheAnchorAcrossTheCouplerHeading) {
  constexpr uint32_t lead = 8;
  Path path = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> splice = spliceCouplerDogleg(
      primitives(), 150.0, path, WIDTH, HEIGHT, 0, {.leadStraight = lead});
  ASSERT_TRUE(splice.has_value());
  ASSERT_GT(path.size(), lead);

  // The resonator couples along the lead: straight cells from the anchor,
  // at a right angle to the direction the readout port faces.
  const Heading across = path.front().heading;
  EXPECT_TRUE(isOrthogonal(across, 0));
  const HeadingVector v = headingVector(across);
  for (uint32_t i = 0; i < lead; ++i) {
    EXPECT_EQ(path[i].heading, across) << i;
    EXPECT_TRUE(primitives().isStraight(across, path[i].primitive)) << i;
    if (i > 0) {
      EXPECT_EQ(static_cast<int64_t>(path[i].x) - path[i - 1].x, v.dx) << i;
      EXPECT_EQ(static_cast<int64_t>(path[i].y) - path[i - 1].y, v.dy) << i;
    }
  }
  // The lead is part of the resonator, so the length still meets the target.
  EXPECT_NEAR(renderedLength(primitives(), path), 150.0, 1.0);
}

TEST(CouplerInsertion, TheFirstTurnStartsTheLeadLengthFromTheAnchor) {
  double lengthWithoutLead = 0.0;
  for (const uint32_t lead : {0U, 1U, 8U}) {
    Path path = straightRun(400, 300, 6, 300);
    const std::optional<CouplerSplice> splice = spliceCouplerDogleg(
        primitives(), 150.0, path, WIDTH, HEIGHT, 0, {.leadStraight = lead});
    ASSERT_TRUE(splice.has_value()) << lead;
    EXPECT_EQ(path.front().x, splice->anchor.x) << lead;
    EXPECT_EQ(path.front().y, splice->anchor.y) << lead;

    // The first point tagged with a turn is the cell the turn starts from. It
    // lies the lead's number of steps from the anchor, along the lead.
    std::size_t turn = 0;
    while (turn < path.size() &&
           primitives().isStraight(path[turn].heading, path[turn].primitive)) {
      ++turn;
    }
    ASSERT_LT(turn, path.size()) << lead;
    const Heading across = path.front().heading;
    EXPECT_EQ(path[turn].heading, across) << lead;
    const HeadingVector v = headingVector(across);
    EXPECT_EQ(static_cast<int64_t>(path[turn].x) - path.front().x,
              static_cast<int64_t>(v.dx) * lead)
        << lead;
    EXPECT_EQ(static_cast<int64_t>(path[turn].y) - path.front().y,
              static_cast<int64_t>(v.dy) * lead)
        << lead;

    // The splice charges the lead its rendered length, a cell per step of
    // this cardinal lead. The anchor moves along the cardinal resonator by
    // whole cells, so the rendered length is the same for every lead.
    const double length = renderedLength(primitives(), path);
    if (lead == 0) {
      lengthWithoutLead = length;
    } else {
      EXPECT_NEAR(length, lengthWithoutLead, 1e-9) << lead;
    }
  }
}

TEST(CouplerInsertion, AConnectionThroughAnEighthTurnRendersWithoutAKink) {
  // The connection from the dogleg onto a diagonal resonator is an eighth
  // turn from a cardinal heading, and onto a cardinal resonator an eighth turn
  // from a diagonal heading. An eighth turn ends before its last swept cell,
  // or on a cell it does not sweep. The next primitive and the resonator
  // start at the end of the turn, so the rendered path neither jumps nor
  // turns sharply where they meet.
  for (const auto& [heading, couplerHeading] :
       {std::pair<Heading, Heading>{7, 0}, std::pair<Heading, Heading>{6, 7}}) {
    Path path = straightRun(600, 600, heading, 300);
    ASSERT_TRUE(spliceCouplerDogleg(primitives(), 150.0, path, WIDTH, HEIGHT,
                                    couplerHeading)
                    .has_value());
    bool eighthTurn = false;
    for (const PathPoint& point : path) {
      const Primitive* primitive =
          primitives().find(point.heading, point.primitive);
      ASSERT_NE(primitive, nullptr);
      eighthTurn = eighthTurn ||
                   headingDistance(point.heading, primitive->exitHeading) == 1;
    }
    ASSERT_TRUE(eighthTurn) << static_cast<int>(heading);

    Path rendered = path;
    std::vector<PathSegment> segments;
    const std::vector<Point> points =
        samplePath(primitives(), rendered, rendered.front(), segments);
    ASSERT_GT(points.size(), 2U);
    // A jump is a gap of more than one and a half cells between two samples.
    // Along an arc the direction changes by about a degree per sample; a
    // change of more than 45 degrees is a kink or a reversal.
    const double minimumCosine = std::cos(std::numbers::pi / 4.0);
    std::size_t previous = 0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      const double dx = points[i].x() - points[i - 1].x();
      const double dy = points[i].y() - points[i - 1].y();
      EXPECT_LT(std::hypot(dx, dy), 1.5)
          << static_cast<int>(heading) << " " << i;
      if (std::hypot(dx, dy) < 1e-9) {
        continue;
      }
      if (previous > 0) {
        const double px = points[previous].x() - points[previous - 1].x();
        const double py = points[previous].y() - points[previous - 1].y();
        const double cosine =
            ((px * dx) + (py * dy)) / (std::hypot(px, py) * std::hypot(dx, dy));
        EXPECT_GT(cosine, minimumCosine)
            << static_cast<int>(heading) << " at (" << points[i - 1].x() << ", "
            << points[i - 1].y() << ")";
      }
      previous = i;
    }
  }
}

TEST(CouplerInsertion, ASplicedPathStepsFromCellToNeighboringCell) {
  // The splice tests the cells of the dogleg and its connection against the
  // rest of the path, so the spliced path must not skip a cell. Some turns of
  // a connection do not sweep their end: the eighth turns off a diagonal
  // heading at a radius of five cells, and the arcs of 72 to 77 degrees at 10
  // and 16 cells. A straight step after such a turn starts from its end. The
  // targets put the splice all along the diagonal resonator.
  for (const uint32_t radius : {5U, 10U, 16U}) {
    const MovePrimitives table(radius);
    const Path resonator =
        routing::straightRun(table, {.x = 600, .y = 600, .heading = 7}, 200);
    const double length = renderedLength(table, resonator);
    for (Heading couplerHeading = 0; couplerHeading < NUM_HEADINGS;
         ++couplerHeading) {
      for (const bool mirrored : {false, true}) {
        for (CouplerDoglegOptions options :
             {CouplerDoglegOptions{.straightLength = 3},
              CouplerDoglegOptions{.leadStraight = 8,
                                   .straightLength = 3,
                                   .secondStraightLength = 2}}) {
          options.mirrored = mirrored;
          for (int step = 0; step < 12; ++step) {
            const double target =
                length * (0.1 + (1.1 * static_cast<double>(step) / 12.0));
            Path path = resonator;
            ASSERT_TRUE(spliceCouplerDogleg(table, target, path, WIDTH, HEIGHT,
                                            couplerHeading, options)
                            .has_value());
            EXPECT_EQ(largestStep(path, path.size()), 1)
                << "radius " << radius << ", coupler heading "
                << static_cast<int>(couplerHeading) << ", mirrored " << mirrored
                << ", lead " << options.leadStraight << ", target " << target;
          }
        }
      }
    }
  }
}

TEST(CouplerInsertion, ADoglegNeverCrossesADiagonalResonator) {
  // Two diagonal steps can cross inside a 2 by 2 block while all four cells
  // stay distinct. The lead of this mirrored dogleg runs at a right angle to
  // the diagonal resonator and meets it in such a block. The splice must
  // leave a path that does not cross itself, or leave the path unchanged.
  PathLoopScratch scratch;
  Path atEdge = straightRun(400, 300, 7, 300);
  ASSERT_FALSE(pathSelfIntersects(atEdge, WIDTH, HEIGHT, scratch));
  const Path before = atEdge;
  if (spliceCouplerDogleg(primitives(), 126.0, atEdge, WIDTH, HEIGHT, 7,
                          {.leadStraight = 8, .mirrored = true})
          .has_value()) {
    EXPECT_FALSE(pathSelfIntersects(atEdge, WIDTH, HEIGHT, scratch));
  } else {
    EXPECT_EQ(atEdge, before);
  }

  // Every dogleg shape on every diagonal resonator, far from the edges.
  for (Heading heading = 1; heading < NUM_HEADINGS; heading += 2) {
    for (Heading couplerHeading = 0; couplerHeading < NUM_HEADINGS;
         ++couplerHeading) {
      for (const bool mirrored : {false, true}) {
        for (const uint32_t lead : {0U, 8U}) {
          for (CouplerDoglegOptions options :
               {CouplerDoglegOptions{},
                CouplerDoglegOptions{.secondStraightLength = 10,
                                     .secondTurnReverse = true},
                CouplerDoglegOptions{.secondStraightLength = 10}}) {
            options.leadStraight = lead;
            options.mirrored = mirrored;
            Path path = straightRun(600, 600, heading, 300);
            const std::optional<CouplerSplice> splice =
                spliceCouplerDogleg(primitives(), 126.0, path, WIDTH, HEIGHT,
                                    couplerHeading, options);
            ASSERT_TRUE(splice.has_value());
            EXPECT_FALSE(pathSelfIntersects(path, WIDTH, HEIGHT, scratch))
                << "heading " << static_cast<int>(heading)
                << ", coupler heading " << static_cast<int>(couplerHeading)
                << ", mirrored " << mirrored << ", lead " << lead
                << ", second run " << options.secondStraightLength
                << ", reverse " << options.secondTurnReverse;
          }
        }
      }
    }
  }
}

TEST(CouplerInsertion, ADoglegStaysInsideTheGrid) {
  // Each resonator ends on an edge of the grid. With this coupler heading the
  // dogleg runs back across the resonator, so only a dogleg at the end of the
  // resonator is collision-free, and its lead reaches past the end.
  struct Edge {
    uint32_t x;
    uint32_t y;
    Heading heading;
    Heading couplerHeading;
  };
  for (const Edge edge :
       {Edge{.x = 400, .y = 300, .heading = 0, .couplerHeading = 4},
        Edge{.x = 799, .y = 899, .heading = 4, .couplerHeading = 0},
        Edge{.x = 899, .y = 400, .heading = 6, .couplerHeading = 2},
        Edge{.x = 300, .y = 799, .heading = 2, .couplerHeading = 6}}) {
    const int heading = edge.heading;
    Path atEdge = straightRun(edge.x, edge.y, edge.heading, 300);
    const PathPoint end = atEdge.back();
    ASSERT_TRUE(end.x == 0 || end.y == 0 || end.x == WIDTH - 1 ||
                end.y == HEIGHT - 1)
        << heading;
    const Path before = atEdge;
    EXPECT_FALSE(spliceCouplerDogleg(primitives(), 150.0, atEdge, WIDTH, HEIGHT,
                                     edge.couplerHeading, {.leadStraight = 8})
                     .has_value())
        << heading;
    EXPECT_EQ(atEdge, before) << heading;

    // One cell further in, the dogleg fits and reaches the edge.
    const HeadingVector v = headingVector(edge.heading);
    Path inside =
        straightRun(static_cast<uint32_t>(static_cast<int64_t>(edge.x) - v.dx),
                    static_cast<uint32_t>(static_cast<int64_t>(edge.y) - v.dy),
                    edge.heading, 300);
    ASSERT_TRUE(spliceCouplerDogleg(primitives(), 150.0, inside, WIDTH, HEIGHT,
                                    edge.couplerHeading, {.leadStraight = 8})
                    .has_value())
        << heading;
    bool reachesEdge = false;
    for (const PathPoint& point : inside) {
      EXPECT_LT(point.x, WIDTH) << heading;
      EXPECT_LT(point.y, HEIGHT) << heading;
      reachesEdge = reachesEdge || (v.dx != 0 && point.x == end.x) ||
                    (v.dy != 0 && point.y == end.y);
    }
    EXPECT_TRUE(reachesEdge) << heading;
  }
}

TEST(CouplerInsertion, TheDoglegNeverRunsIntoTheRestOfTheResonator) {
  // A target of 450 cells puts the splice on the way out, since the U-turn
  // and the way back are shorter. The way back runs ten cells beside it, on
  // the side the plain dogleg takes, closer than the dogleg reaches. Every
  // plain dogleg on the way out would cross the way back, so the splice
  // moves onto the way back and falls short of the target instead.
  Path plain = hairpin();
  ASSERT_FALSE(touchesItself(plain));
  const std::optional<CouplerSplice> blocked =
      spliceCouplerDogleg(primitives(), 450.0, plain, WIDTH, HEIGHT, 0);
  ASSERT_TRUE(blocked.has_value());
  EXPECT_FALSE(touchesItself(plain));
  EXPECT_EQ(blocked->anchor.heading, 2);
  EXPECT_LT(renderedLength(primitives(), plain), 447.0);

  // The mirrored dogleg takes the free side and lands on the way out.
  Path mirrored = hairpin();
  const std::optional<CouplerSplice> free = spliceCouplerDogleg(
      primitives(), 450.0, mirrored, WIDTH, HEIGHT, 0, {.mirrored = true});
  ASSERT_TRUE(free.has_value());
  EXPECT_FALSE(touchesItself(mirrored));
  EXPECT_EQ(free->anchor.heading, 6);
  EXPECT_NEAR(renderedLength(primitives(), mirrored), 450.0, 1.0);
}

TEST(CouplerInsertion, ADoglegOfALargerRadiusNeverRunsIntoTheResonator) {
  // At a radius of ten cells, the resonator runs toward positive x and y,
  // then toward positive y, negative x and negative y, past the cells where a
  // dogleg spliced onto its first run turns. Whatever the target, the spliced
  // path must not cross itself.
  const MovePrimitives table(10);
  Path path;
  PathPoint state{.x = 300, .y = 300, .heading = 5, .primitive = 0};
  // Appends a move from the state in the format of a routed path.
  const auto move = [&](const uint16_t id) {
    const Primitive* primitive = table.find(state.heading, id);
    ASSERT_NE(primitive, nullptr);
    PathPoint start = state;
    start.primitive = id;
    if (!path.empty() && path.back().samePlace(start)) {
      path.back() = start;
    } else {
      path.push_back(start);
    }
    for (const CellOffset& c : primitive->swept) {
      const PathPoint cell{
          .x = static_cast<uint32_t>(static_cast<int32_t>(state.x) + c.dx),
          .y = static_cast<uint32_t>(static_cast<int32_t>(state.y) + c.dy),
          .heading = state.heading,
          .primitive = id};
      if (!path.back().samePlace(cell)) {
        path.push_back(cell);
      }
    }
    state.x =
        static_cast<uint32_t>(static_cast<int32_t>(state.x) + primitive->dx);
    state.y =
        static_cast<uint32_t>(static_cast<int32_t>(state.y) + primitive->dy);
    state.heading = primitive->exitHeading;
  };
  const auto straight = [&](const uint32_t steps) {
    for (uint32_t k = 0; k < steps; ++k) {
      move(table.straight(state.heading));
    }
  };
  // The turns of the resonator: an eighth turn from heading 5 onto heading 4,
  // and quarter turns from heading 4 onto 2 and from heading 2 onto 0.
  const auto turnTo = [&](const Heading exit) {
    for (const Primitive& p : table.of(state.heading)) {
      if (p.exitHeading == exit) {
        move(p.id);
        return;
      }
    }
    FAIL() << "no turn onto heading " << static_cast<int>(exit);
  };
  straight(4);
  turnTo(4);
  straight(3);
  turnTo(2);
  straight(2);
  turnTo(0);
  straight(45);
  path.push_back({.x = state.x,
                  .y = state.y,
                  .heading = state.heading,
                  .primitive = table.straight(state.heading)});

  PathLoopScratch scratch;
  ASSERT_FALSE(pathSelfIntersects(path, 1000, 1000, scratch));
  const double length = renderedLength(table, path);
  const double dogleg = buildDogleg(table, 7, -1, 14).cost;
  for (int half = 0; half <= 20; ++half) {
    const double target = length + dogleg - 10.0 + (0.5 * half);
    Path spliced = path;
    if (spliceCouplerDogleg(table, target, spliced, 1000, 1000, 5)
            .has_value()) {
      EXPECT_FALSE(pathSelfIntersects(spliced, 1000, 1000, scratch)) << target;
    }
  }
}

TEST(CouplerInsertion, ASpliceOntoRoutesOfALargerRadiusStaysWhole) {
  // Routes at a radius of ten cells around a block, each with couplers
  // spliced on in several shapes and at several targets. Up to the point
  // where it joins the route, the spliced path steps from cell to neighboring
  // cell. Where the route does not cross itself, neither does the spliced
  // path. It renders as long as its two parts, so the length the splice aims
  // at is the length drawn.
  constexpr uint32_t width = 400;
  constexpr uint32_t height = 400;
  auto shared = std::make_shared<const MovePrimitives>(10);
  const MovePrimitives& table = *shared;
  SearchScratch scratch(width, height);
  grid::BitGrid corridor(width, height);
  for (uint32_t y = 170; y < 230; ++y) {
    for (uint32_t x = 180; x < 220; ++x) {
      corridor.setCell(x, y);
    }
  }
  DubinsRouter router(shared, scratch,
                      {.startStraightLength = 12,
                       .endStraightLength = 12,
                       .minRadius = 10,
                       .bendPenalty = 500});
  router.attachCorridor(&corridor);
  SplitMix random(10);
  const auto below = [&](const uint32_t n) {
    return static_cast<uint32_t>(random.next() % n);
  };

  PathLoopScratch loops;
  int splices = 0;
  for (int request = 0; request < 12; ++request) {
    const Path path =
        router.route({.source = {.x = 40 + below(80),
                                 .y = 40 + below(320),
                                 .heading = static_cast<Heading>(below(8)),
                                 .primitive = 0},
                      .target = {.x = 280 + below(80),
                                 .y = 40 + below(320),
                                 .heading = static_cast<Heading>(below(8)),
                                 .primitive = 0}});
    if (path.empty()) {
      continue;
    }
    const bool clean = !pathSelfIntersects(path, width, height, loops);
    const double length = renderedLength(table, path);
    for (Heading couplerHeading = 0; couplerHeading < NUM_HEADINGS;
         ++couplerHeading) {
      for (const bool mirrored : {false, true}) {
        for (CouplerDoglegOptions options :
             {CouplerDoglegOptions{.straightLength = 3},
              CouplerDoglegOptions{.leadStraight = 8,
                                   .straightLength = 14,
                                   .secondStraightLength = 3,
                                   .secondTurnReverse = true}}) {
          options.mirrored = mirrored;
          for (const double share : {0.3, 0.6, 0.9}) {
            Path spliced = path;
            if (!spliceCouplerDogleg(table, length * share, spliced, width,
                                     height, couplerHeading, options)
                     .has_value()) {
              continue;
            }
            ++splices;
            const std::size_t junction = junctionOf(table, path, spliced);
            ASSERT_LT(junction, spliced.size());
            ASSERT_GT(junction, 0U);
            EXPECT_EQ(largestStep(spliced, junction), 1)
                << "request " << request << ", coupler heading "
                << static_cast<int>(couplerHeading) << ", mirrored " << mirrored
                << ", lead " << options.leadStraight << ", share " << share;
            if (clean) {
              EXPECT_FALSE(pathSelfIntersects(spliced, width, height, loops))
                  << "request " << request << ", coupler heading "
                  << static_cast<int>(couplerHeading) << ", mirrored "
                  << mirrored << ", lead " << options.leadStraight << ", share "
                  << share;
            }
            EXPECT_NEAR(lengthOffParts(table, path, spliced), 0.0, 1e-9)
                << "request " << request << ", coupler heading "
                << static_cast<int>(couplerHeading) << ", mirrored " << mirrored
                << ", lead " << options.leadStraight << ", share " << share;
          }
        }
      }
    }
  }
  EXPECT_GT(splices, 500);
}

TEST(CouplerInsertion, APathWithoutAStraightRunHasNoPlaceForACoupler) {
  // A coupler sits on a straight run. One point, or a single turn, has none,
  // so there is no candidate and the path stays as it was.
  Path single = straightRun(400, 300, 6, 0);
  const Path singleBefore = single;
  EXPECT_FALSE(
      spliceCouplerDogleg(primitives(), 150.0, single, WIDTH, HEIGHT, 0)
          .has_value());
  EXPECT_EQ(single, singleBefore);

  Path turn = straightRun(400, 300, 6, 0);
  appendTurn(turn, 6, 4);
  turn.erase(turn.begin());
  const Path turnBefore = turn;
  ASSERT_FALSE(turn.empty());
  EXPECT_FALSE(spliceCouplerDogleg(primitives(), 150.0, turn, WIDTH, HEIGHT, 0)
                   .has_value());
  EXPECT_EQ(turn, turnBefore);
}

TEST(CouplerInsertion, ThereIsNothingToSpliceOntoAnEmptyPath) {
  Path empty;
  EXPECT_FALSE(spliceCouplerDogleg(primitives(), 150.0, empty, WIDTH, HEIGHT, 0)
                   .has_value());
  Path path = straightRun(400, 300, 6, 10);
  EXPECT_THROW(static_cast<void>(spliceCouplerDogleg(primitives(), 150.0, path,
                                                     WIDTH, HEIGHT, 9)),
               std::invalid_argument);
}

} // namespace
