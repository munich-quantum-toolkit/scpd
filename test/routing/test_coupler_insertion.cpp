/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

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
  Path path;
  const HeadingVector v = headingVector(heading);
  const uint16_t id = primitives().straight(heading);
  for (uint32_t i = 0; i <= steps; ++i) {
    path.push_back({.x = static_cast<uint32_t>(x0 + (v.dx * i)),
                    .y = static_cast<uint32_t>(y0 + (v.dy * i)),
                    .heading = heading,
                    .primitive = id});
  }
  return path;
}

/// Appends the cells a quarter turn sweeps from the last point of a path.
void appendTurn(Path& path, const Heading from, const Heading to) {
  const PathPoint start = path.back();
  for (const Primitive& p : primitives().of(from)) {
    if (p.exitHeading == to) {
      for (const CellOffset& c : p.swept) {
        path.push_back({.x = static_cast<uint32_t>(start.x + c.dx),
                        .y = static_cast<uint32_t>(start.y + c.dy),
                        .heading = from,
                        .primitive = p.id});
      }
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
  path.insert(path.end(), back.begin(), back.end());
  return path;
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

TEST(CouplerInsertion, ADoglegTurnsOnceAndThenRunsStraight) {
  const DoglegGeometry dogleg = buildDogleg(primitives(), 6, -1, 12);
  // A quarter turn against the clock from heading 6 ends on heading 4.
  EXPECT_EQ(dogleg.tip.heading, 4);
  EXPECT_NEAR(dogleg.cost, (std::numbers::pi / 2.0 * 5.0) + 12.0, 0.5);
  ASSERT_FALSE(dogleg.path.empty());
  EXPECT_EQ(dogleg.path.back().x, dogleg.tip.x);
  EXPECT_EQ(dogleg.path.back().y, dogleg.tip.y);
  // The straight run leaves along the exit heading.
  const HeadingVector v = headingVector(4);
  const PathPoint& afterTurn = dogleg.path[dogleg.path.size() - 13];
  EXPECT_EQ(static_cast<int64_t>(dogleg.tip.x) - afterTurn.x, v.dx * 12);
  EXPECT_EQ(static_cast<int64_t>(dogleg.tip.y) - afterTurn.y, v.dy * 12);

  const DoglegGeometry clockwise = buildDogleg(primitives(), 6, 1, 0);
  EXPECT_EQ(clockwise.tip.heading, 0);
  EXPECT_THROW(static_cast<void>(buildDogleg(primitives(), 6, 0, 4)),
               std::invalid_argument);
}

TEST(CouplerInsertion, TheSpliceLandsWhereTheRestOfThePathHitsTheTarget) {
  // A long straight resonator: the coupler is spliced so that the path from
  // it to the far end is the target length.
  Path path = straightRun(400, 300, 6, 300);
  const double before = reconstructSegments(primitives(), path).nominalLength;
  ASSERT_GT(before, 250.0);

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
  const double after = reconstructSegments(primitives(), path).nominalLength;
  EXPECT_NEAR(after, 150.0, 3.0);
}

TEST(CouplerInsertion, TheSplicePrefersAnUndershoot) {
  Path path = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(primitives(), 150.0, path, WIDTH, HEIGHT, 0);
  ASSERT_TRUE(splice.has_value());
  const double after = reconstructSegments(primitives(), path).nominalLength;
  // A resonator that is a little short can be lengthened by a meander; one
  // that is too long cannot be shortened, so the splice never overshoots by
  // more than the step it had to choose between.
  EXPECT_LE(after, 151.0);
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
  EXPECT_NEAR(reconstructSegments(primitives(), jogged).nominalLength, 150.0,
              3.0);

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

TEST(CouplerInsertion, TheLeadRunsStraightFromTheAnchorAcrossTheOrientation) {
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
  EXPECT_NEAR(reconstructSegments(primitives(), path).nominalLength, 150.0,
              3.0);
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

    // The splice charges the lead its number of steps, and the path has that
    // many. The sum of the primitive costs is therefore the same for every
    // lead.
    const double length = reconstructSegments(primitives(), path).nominalLength;
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
  for (const auto& [heading, orientation] :
       {std::pair<Heading, Heading>{7, 0}, std::pair<Heading, Heading>{6, 7}}) {
    Path path = straightRun(600, 600, heading, 300);
    ASSERT_TRUE(spliceCouplerDogleg(primitives(), 150.0, path, WIDTH, HEIGHT,
                                    orientation)
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
    for (Heading orientation = 0; orientation < NUM_HEADINGS; ++orientation) {
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
            const std::optional<CouplerSplice> splice = spliceCouplerDogleg(
                primitives(), 126.0, path, WIDTH, HEIGHT, orientation, options);
            ASSERT_TRUE(splice.has_value());
            EXPECT_FALSE(pathSelfIntersects(path, WIDTH, HEIGHT, scratch))
                << "heading " << static_cast<int>(heading) << ", orientation "
                << static_cast<int>(orientation) << ", mirrored " << mirrored
                << ", lead " << lead << ", second run "
                << options.secondStraightLength << ", reverse "
                << options.secondTurnReverse;
          }
        }
      }
    }
  }
}

TEST(CouplerInsertion, ADoglegStaysInsideTheGrid) {
  // Each resonator ends on an edge of the grid. With this orientation the
  // dogleg runs back across the resonator, so only a dogleg at the end of the
  // resonator is collision-free, and its lead reaches past the end.
  struct Edge {
    uint32_t x;
    uint32_t y;
    Heading heading;
    Heading orientation;
  };
  for (const Edge edge :
       {Edge{.x = 400, .y = 300, .heading = 0, .orientation = 4},
        Edge{.x = 799, .y = 899, .heading = 4, .orientation = 0},
        Edge{.x = 899, .y = 400, .heading = 6, .orientation = 2},
        Edge{.x = 300, .y = 799, .heading = 2, .orientation = 6}}) {
    const int heading = edge.heading;
    Path atEdge = straightRun(edge.x, edge.y, edge.heading, 300);
    const PathPoint end = atEdge.back();
    ASSERT_TRUE(end.x == 0 || end.y == 0 || end.x == WIDTH - 1 ||
                end.y == HEIGHT - 1)
        << heading;
    const Path before = atEdge;
    EXPECT_FALSE(spliceCouplerDogleg(primitives(), 150.0, atEdge, WIDTH, HEIGHT,
                                     edge.orientation, {.leadStraight = 8})
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
                                    edge.orientation, {.leadStraight = 8})
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
  EXPECT_LT(reconstructSegments(primitives(), plain).nominalLength, 447.0);

  // The mirrored dogleg takes the free side and lands on the way out.
  Path mirrored = hairpin();
  const std::optional<CouplerSplice> free = spliceCouplerDogleg(
      primitives(), 450.0, mirrored, WIDTH, HEIGHT, 0, {.mirrored = true});
  ASSERT_TRUE(free.has_value());
  EXPECT_FALSE(touchesItself(mirrored));
  EXPECT_EQ(free->anchor.heading, 6);
  EXPECT_NEAR(reconstructSegments(primitives(), mirrored).nominalLength, 450.0,
              3.0);
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
