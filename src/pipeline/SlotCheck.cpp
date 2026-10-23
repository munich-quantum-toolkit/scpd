/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The slot check: footprints and ways out shared between places, room fills
// over words of 64 cells, and memos in the way search.

#include "mqt-scpd/pipeline/SlotCheck.hpp"

#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <format>
#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// The check repeats the arithmetic of its Python model operation by
// operation, so that both judge a graph alike: ties between ways of equal
// length break the same way only when every sum rounds the same. The build
// compiles this file without fused multiply-add for that reason.

namespace mqt::scpd::pipeline {
namespace {

using nlohmann::json;

/// How many rounds the negotiated routing runs at most.
constexpr int ROUNDS = 60;
/// How many times the length of its shortest way a wire's way may run.
constexpr double DETOUR = 1.5;
/// How many points of a feedline edge to either side of a slot count as
/// the slot's own straight piece, whose wall the slot may cover.
constexpr std::ptrdiff_t OWN_WINDOW = 40;
/// How many sweeps `tautLength` makes at most.
constexpr int TAUT_SWEEPS = 40;
/// How many cells past the crossing reach `sideChamber` looks.
constexpr std::int64_t SIDE_STEPS = 14;
/// The code of a stub wall in the raster.
constexpr int STUB = 4;
/// The code of a feedline wall in the raster.
constexpr int FEEDLINE = 5;
/// The first code of a chamber in the raster.
constexpr int CHAMBER = 6;

/// A point of the chip, in cells.
struct Point {
  double x = 0.0;
  double y = 0.0;
};

[[nodiscard]] Point pointOf(const json& value) {
  return {value.at(0).get<double>(), value.at(1).get<double>()};
}

/// A number as JSON: a whole number as an integer, else as a real number.
[[nodiscard]] json numberJson(const double value) {
  if (std::nearbyint(value) == value && std::abs(value) < 9.0e15) {
    return static_cast<std::int64_t>(value);
  }
  return value;
}

[[nodiscard]] json pointJson(const Point& point) {
  return json::array({numberJson(point.x), numberJson(point.y)});
}

/// A computed point as JSON: always as real numbers.
[[nodiscard]] json realJson(const Point& point) {
  return json::array({point.x, point.y});
}

/// A real number rounded to the nearest whole number, half to even, as
/// Python's `round` does.
[[nodiscard]] std::int64_t rounded(const double value) {
  return static_cast<std::int64_t>(std::nearbyint(value));
}

/// A real number rounded to one decimal, half to even on its exact value,
/// as Python's `round(value, 1)` does.
[[nodiscard]] double roundedTenth(const double value) {
  const auto text = std::format("{:.1f}", value);
  return std::stod(text);
}

/// `count` numbers from 0 to 1, evenly apart, as `numpy.linspace` computes
/// them: the index times the step, the last one exactly 1.
[[nodiscard]] std::vector<double> unitSteps(const std::size_t count) {
  std::vector<double> steps(count, 0.0);
  if (count > 1) {
    const double step = 1.0 / static_cast<double>(count - 1);
    for (std::size_t index = 0; index < count; ++index) {
      steps[index] = static_cast<double>(index) * step;
    }
    steps.back() = 1.0;
  }
  return steps;
}

/// Whether a JSON value counts as true in Python: not null, not false, not
/// zero, not empty.
[[nodiscard]] bool truthy(const json& value) {
  if (value.is_null()) {
    return false;
  }
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.is_number()) {
    return value.get<double>() != 0.0;
  }
  if (value.is_string()) {
    return !value.get_ref<const std::string&>().empty();
  }
  return !value.empty();
}

/// Whether a key of a JSON object counts as true in Python.
[[nodiscard]] bool truthy(const json& object, const char* key) {
  const auto found = object.find(key);
  return found != object.end() && truthy(*found);
}

/// A key of a JSON object as text: empty when it is missing or no text.
[[nodiscard]] std::string textOf(const json& object, const char* key) {
  const auto found = object.find(key);
  if (found == object.end()) {
    return {};
  }
  if (found->is_string()) {
    return found->get<std::string>();
  }
  return found->is_null() ? std::string("None") : found->dump();
}

/// Whether a character counts as white space in a regular expression.
[[nodiscard]] bool blank(const char letter) {
  return std::isspace(static_cast<unsigned char>(letter)) != 0;
}

/// The name of the wire a wall belongs to, by the wall's name: what
/// `(?:lead of )?([^\s,]+)` takes from its start. With `atIts`, only when
/// ` at its` or `,` follows the name, as `(?: at its|,)` asks.
[[nodiscard]] std::optional<std::string> runName(const std::string_view name,
                                                 const bool atIts) {
  const auto from = [&](const std::size_t start) -> std::optional<std::string> {
    auto end = start;
    while (end < name.size() && name[end] != ',' && !blank(name[end])) {
      ++end;
    }
    if (end == start) {
      return std::nullopt;
    }
    if (atIts) {
      const auto rest = name.substr(end);
      if (!rest.starts_with(" at its") && !rest.starts_with(",")) {
        return std::nullopt;
      }
    }
    return std::string(name.substr(start, end - start));
  };
  constexpr std::string_view LEAD = "lead of ";
  if (name.starts_with(LEAD)) {
    if (auto found = from(LEAD.size())) {
      return found;
    }
  }
  return from(0);
}

/// The terminal edge an end of a mouth gate names: `f<number>` where the
/// end reads `f<number> at its coupler`.
[[nodiscard]] std::optional<std::string>
couplerEnd(const std::string_view end) {
  if (!end.starts_with('f')) {
    return std::nullopt;
  }
  std::size_t digits = 1;
  while (digits < end.size() &&
         std::isdigit(static_cast<unsigned char>(end[digits])) != 0) {
    ++digits;
  }
  if (digits == 1 || !end.substr(digits).starts_with(" at its coupler")) {
    return std::nullopt;
  }
  return std::string(end.substr(0, digits));
}

/// The wall an end of a gate lies on: its name without the side of a lead
/// or a pad (`, left` or `, right`).
[[nodiscard]] std::string wallOf(const std::string_view end) {
  for (const std::string_view side : {"left", "right"}) {
    if (!end.ends_with(side)) {
      continue;
    }
    auto rest = end.substr(0, end.size() - side.size());
    while (!rest.empty() && blank(rest.back())) {
      rest.remove_suffix(1);
    }
    if (!rest.empty() && rest.back() == ',') {
      rest.remove_suffix(1);
      return std::string(rest);
    }
  }
  return std::string(end);
}

/// The number of a terminal edge's name, `f<number>`.
[[nodiscard]] std::int64_t numberOf(const std::string& name) {
  std::int64_t number = 0;
  const auto* first = name.data() + std::min<std::size_t>(1, name.size());
  const auto* last = name.data() + name.size();
  const auto [end, error] = std::from_chars(first, last, number);
  if (error != std::errc() || end != last) {
    throw std::invalid_argument(
        std::format("slot check: the terminal edge '{}' has no number", name));
  }
  return number;
}

/// The cells of the chip by what they are: 0 a free cell in no chamber, 1
/// to 5 a wall by its kind, 6 + c a cell of chamber c.
struct Grid {
  std::int64_t width = 0;
  std::int64_t height = 0;
  /// The codes of the graph's cells, row by row (`SlotGraph::codes`), or
  /// nothing when `view` gives them (`SlotGraph::raster`).
  const std::uint16_t* code = nullptr;
  const SlotGraph::RasterView* view = nullptr;
  std::size_t size = 0;
  /// The cells of the stubs' walls, row by row (`SlotGraph::stubCells`).
  const std::vector<std::size_t>* stubCells = nullptr;

  [[nodiscard]] bool inside(const std::int64_t x, const std::int64_t y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }
  [[nodiscard]] std::size_t cell(const std::int64_t x,
                                 const std::int64_t y) const {
    return static_cast<std::size_t>((y * width) + x);
  }
  /// The code of cell number `index`.
  [[nodiscard]] std::uint16_t codeAt(const std::size_t index) const {
    return code != nullptr ? code[index] : view->code(index);
  }
  [[nodiscard]] int at(const std::int64_t x, const std::int64_t y) const {
    return codeAt(cell(x, y));
  }
  /// Whether a wire may pass the cell: a chamber or a cut line.
  [[nodiscard]] bool open(const std::int64_t x, const std::int64_t y) const {
    const auto value = at(x, y);
    return value >= CHAMBER || value == 0;
  }
};

[[nodiscard]] Grid gridOf(const SlotGraph& graph) {
  if (graph.codes.empty() && graph.raster.has_value()) {
    return {.width = graph.width,
            .height = graph.height,
            .code = nullptr,
            .view = &*graph.raster,
            .size = static_cast<std::size_t>(graph.width * graph.height),
            .stubCells = &graph.stubCells};
  }
  if (std::cmp_not_equal(graph.codes.size(), graph.width * graph.height)) {
    throw std::invalid_argument(
        "slot check: the raster does not cover the chip");
  }
  return {.width = graph.width,
          .height = graph.height,
          .code = graph.codes.data(),
          .view = nullptr,
          .size = graph.codes.size(),
          .stubCells = &graph.stubCells};
}

/// A port run or a feedline edge of the graph.
struct Stroke {
  std::string name;
  std::string kind;
  std::vector<Point> points;
};

/// One end of a wire.
struct WireEnd {
  Point cell;
  std::vector<std::int64_t> chambers;
  std::string label;
};

/// A wire of the graph.
struct Wire {
  std::string name;
  WireEnd source;
  WireEnd target;
  std::vector<std::string> crosses;
  bool terminal = false;
  bool resonator = false;
};

/// A cell of the raster.
using Cell = std::pair<std::int64_t, std::int64_t>;

/// A way out of one end of a crossing slot: one of the router's turns from
/// the heading across the edge, then `escapeStraight` cells on.
struct EscapeWay {
  /// The cell the way starts from, the router's turn it takes and how many
  /// cells it runs straight after it, which give the cells it covers
  /// (`EscapeWays::cellsOf`) and its curve for the page (`curveOf`).
  Cell start{};
  const routing::Primitive* move = nullptr;
  std::int64_t straight = 0;
  /// The bounds of the cells, for `EscapeWays::apart`.
  std::int64_t minX = 0;
  std::int64_t maxX = 0;
  std::int64_t minY = 0;
  std::int64_t maxY = 0;
};

/// The ways out of the two ends of a crossing slot, the end to the left of
/// the slot's direction first.
using EscapeOptions = std::array<std::vector<EscapeWay>, 2>;

/// The ways out of the two ends of a crossing slot as `EscapeOptions` holds
/// them, by reference: the same slot on the opposite direction refers to
/// the same ways with its two ends swapped.
using EscapeChoice = std::array<const std::vector<EscapeWay>*, 2>;

/// An edge of the check's graph: a bottleneck of the input, or a crossing
/// slot.
struct Edge {
  bool stretch = false;
  std::string name;
  /// The feedline edge a slot crosses.
  std::string feedline;
  std::vector<std::int64_t> chambers;
  /// The chambers on the two sides of a bottleneck's line, when both sides
  /// have one.
  std::optional<std::array<std::vector<std::int64_t>, 2>> sides;
  std::int64_t capacity = 0;
  /// The wires that may use it; every wire when nothing.
  std::optional<std::vector<std::string>> users;
  Point from;
  Point to;
  std::vector<std::string> ends;
  bool mouth = false;
  double length = 0.0;
  /// The place of a slot on its feedline edge.
  Point at;
  /// The unit step of a slot's feedline edge.
  Point direction;
  /// The corners of a slot's rectangle.
  std::vector<Point> corners;
  /// The ways out of a slot's two ends its placement chose
  ///, or none.
  std::vector<EscapeWay> ways;
  /// Every way out of a slot's two ends.
  EscapeOptions options;
  json out;
};

/// A crossing slot on a feedline edge.
struct Slot {
  Point at;
  std::array<Point, 4> corners;
  Point direction;
  /// The ways out of its two ends, the end to the left of `direction`
  /// first, or none.
  std::vector<EscapeWay> ways;
  /// Every way out of its two ends.
  EscapeOptions options;
};

/// A place on a feedline edge where no slot fits, and why.
struct Refused {
  Point at;
  std::string why;
  std::array<Point, 4> corners;
};

/// The slots of one feedline edge.
struct EdgeSlots {
  std::string name;
  double length = 0.0;
  std::vector<Slot> slots;
  std::int64_t stretches = 0;
  std::vector<Refused> refused;
  /// Whether the slots are spread evenly, or laid one after the other
  /// because a spread gave another number.
  bool spread = false;
};

/// A line that a group of slots shares: a self-bottleneck.
struct Group {
  /// The slot edges behind the line, ascending.
  std::vector<std::size_t> slots;
  std::int64_t capacity = 0;
  std::string name;
  std::array<std::array<std::int64_t, 2>, 2> line{};
  std::string feedline;
  /// The wires with one end behind the line.
  std::vector<std::string> enders;
};

[[nodiscard]] double dot(const Point& a, const Point& b) {
  return (a.x * b.x) + (a.y * b.y);
}

[[nodiscard]] Point minus(const Point& a, const Point& b) {
  return {a.x - b.x, a.y - b.y};
}

[[nodiscard]] double distance(const Point& a, const Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

/// The length of the shortest line from `start` to `end` that touches every
/// segment in turn. Each touch point moves, in turn, to the point of its
/// segment nearest the line between its two neighbours, until the points
/// stand still.
[[nodiscard]] double
tautLength(const Point& start, const Point& end,
           const std::vector<std::pair<Point, Point>>& segments) {
  std::vector<Point> points;
  points.reserve(segments.size());
  for (const auto& [c, d] : segments) {
    points.push_back({(c.x + d.x) / 2.0, (c.y + d.y) / 2.0});
  }
  for (int sweep = 0; sweep < TAUT_SWEEPS; ++sweep) {
    double moved = 0.0;
    for (std::size_t i = 0; i < segments.size(); ++i) {
      const auto& [c, d] = segments[i];
      const auto a = i > 0 ? points[i - 1] : start;
      const auto b = i + 1 < segments.size() ? points[i + 1] : end;
      const auto u = minus(d, c);
      const auto uu = dot(u, u);
      Point next;
      if (uu < 1e-12) {
        next = c;
      } else {
        // Mirror b to a's side of the segment's line: the best point on
        // that line lies where a meets the mirror image.
        const auto root = std::sqrt(uu);
        const Point normal{-u.y / root, u.x / root};
        const auto da = dot(minus(a, c), normal);
        const auto db = dot(minus(b, c), normal);
        const auto mirrored = da * db > 0 ? Point{b.x - ((2 * db) * normal.x),
                                                  b.y - ((2 * db) * normal.y)}
                                          : b;
        const auto dd = dot(minus(mirrored, c), normal);
        double q = 0.0;
        if (std::abs(da - dd) < 1e-12) {
          q = (dot(minus(a, c), u) + dot(minus(mirrored, c), u)) / 2.0 / uu;
        } else {
          const auto share = da / (da - dd);
          const Point x{a.x + ((mirrored.x - a.x) * share),
                        a.y + ((mirrored.y - a.y) * share)};
          q = dot(minus(x, c), u) / uu;
        }
        auto clamped = q > 0.0 ? q : 0.0;
        clamped = clamped < 1.0 ? clamped : 1.0;
        next = {c.x + (clamped * u.x), c.y + (clamped * u.y)};
      }
      const auto step = distance(next, points[i]);
      moved = step > moved ? step : moved;
      points[i] = next;
    }
    if (moved < 0.05) {
      break;
    }
  }
  double length = 0.0;
  auto previous = start;
  for (const auto& point : points) {
    length += distance(point, previous);
    previous = point;
  }
  return length + distance(end, previous);
}

/// The cells at either end of an edge between couplers that the crossing
/// rule of the slots closes by default; see `slotPinCellsOfEnvironment`.
constexpr std::size_t SLOT_PIN_CELLS = routing::CrossingConstraints::PIN_CELLS;

/// The cells a way out of a crossing slot keeps from a stub's line beyond a
/// clearance by default; see `escapeStubMarginOfEnvironment`.
constexpr double ESCAPE_STUB_MARGIN = 4.0;

/// The switches of the check as one check reads them: from the environment
/// once at its start (`readSettings`), not at every gate, which cost a tenth
/// of a check on 69q. Each check reads them again, so one process can still
/// compare several settings.
struct Settings {
  double margin = 1.06;
  std::int64_t marginFrom = 3;
  double resonatorPassage = 0.8;
  std::size_t slotPinCells = SLOT_PIN_CELLS;
  std::int64_t escapeStraight = 5;
  double escapeStubMargin = ESCAPE_STUB_MARGIN;
};

/// The settings of the running check, from the graph it judges.
[[nodiscard]] Settings& settings() {
  thread_local Settings current;
  return current;
}

[[nodiscard]] double capacityMargin() { return settings().margin; }
[[nodiscard]] std::int64_t marginFrom() { return settings().marginFrom; }
[[nodiscard]] double resonatorPassage() { return settings().resonatorPassage; }
[[nodiscard]] std::size_t slotPinCells() { return settings().slotPinCells; }
[[nodiscard]] double escapeStubMargin() { return settings().escapeStubMargin; }
[[nodiscard]] std::int64_t escapeStraight() {
  return settings().escapeStraight;
}

// The rules of the check. A gap is measured across the heading the wires
// cross it on; a slot needs a crossing line the router's rule allows; a
// plain gate on a mouth line between the same chambers counts as the
// mouth; the slots of a straight piece are spread evenly; a slot goes only
// where each of its ends has a way out, a quarter turn allowed; an escape
// keeps no clearance from coupler runs and a gate at the wire's own run
// holds it no more than any other.
[[nodiscard]] constexpr bool gateHeading() { return true; }
[[nodiscard]] constexpr bool slotCrossingRule() { return true; }
[[nodiscard]] constexpr bool mouthTwins() { return true; }
[[nodiscard]] constexpr bool evenSlots() { return true; }
[[nodiscard]] constexpr bool slotEscape() { return true; }
[[nodiscard]] constexpr bool escapeQuarterTurns() { return true; }
[[nodiscard]] constexpr bool escapeCouplerRuns() { return false; }
[[nodiscard]] constexpr bool ownRunBonus() { return false; }
[[nodiscard]] constexpr double staticGapBonus() { return 0.0; }
[[nodiscard]] constexpr double slotReachMargin() { return 0.0; }
[[nodiscard]] constexpr double slotStubGap() { return 0.0; }
[[nodiscard]] constexpr double escapeApartMargin() { return 0.0; }

/// How many wires a gap of `cells` holds: `floor(cells / (clearance *
/// margin))`, none when the gap is narrower than that; the margin applies
/// to a gap that holds `marginFrom` wires or more without it.
[[nodiscard]] std::int64_t wiresIn(const double cells, const double clear,
                                   const double margin) {
  const auto plain =
      cells >= clear ? static_cast<std::int64_t>(std::floor(cells / clear)) : 0;
  if (plain < marginFrom()) {
    return plain;
  }
  const auto taken = clear * margin;
  return cells >= taken ? static_cast<std::int64_t>(std::floor(cells / taken))
                        : 0;
}

/// The share of a line from a to b that wires crossing it on the best of
/// the eight headings can use: cos(phi), phi the angle between the line's
/// normal and the nearest heading, at most 22.5 degrees; 1 under
/// `gateHeading` off or for a line of no length.
[[nodiscard]] double headingShare(const Point& a, const Point& b) {
  const auto dx = b.x - a.x;
  const auto dy = b.y - a.y;
  if (!gateHeading() || std::hypot(dx, dy) < 1e-9) {
    return 1.0;
  }
  constexpr double EIGHTH = std::numbers::pi / 4.0;
  // The normal's angle folded into one eighth turn: the distance to the
  // nearest heading.
  const auto angle = std::atan2(dx, -dy);
  const auto folded = std::fmod(std::fmod(angle, EIGHTH) + EIGHTH, EIGHTH);
  return std::cos(std::min(folded, EIGHTH - folded));
}

/// The heading of a unit step on the grid, or nothing for a step of zero.
[[nodiscard]] std::optional<routing::Heading> headingOfStep(const int dx,
                                                            const int dy) {
  for (routing::Heading heading = 0; heading < routing::NUM_HEADINGS;
       ++heading) {
    const auto v = routing::headingVector(heading);
    if (v.dx == dx && v.dy == dy) {
      return heading;
    }
  }
  return std::nullopt;
}

/// The sign of a number: -1, 0 or 1.
[[nodiscard]] int signOf(const double value) {
  return value > 1e-9 ? 1 : (value < -1e-9 ? -1 : 0);
}

/// The router's bend radius in cells, as the Final stage's `BEND_RADIUS`.
constexpr std::uint32_t ESCAPE_RADIUS = 5;

/// The distance from a point to the segment from a to b.
[[nodiscard]] double distanceToSegment(const Point& p, const Point& a,
                                       const Point& b) {
  const auto dx = b.x - a.x;
  const auto dy = b.y - a.y;
  const auto length = (dx * dx) + (dy * dy);
  const auto t =
      length == 0.0
          ? 0.0
          : std::clamp((((p.x - a.x) * dx) + ((p.y - a.y) * dy)) / length, 0.0,
                       1.0);
  return std::hypot(p.x - (a.x + (t * dx)), p.y - (a.y + (t * dy)));
}

/// How far apart the ports of one coupler may lie, in clearances: the run
/// along a pad and the turn into it.
constexpr double COUPLER_RUN_CLEARANCES = 4.0;

/// The runs of the feedlines through their couplers: for two consecutive
/// feedline edges `f<k>` and `f<k+1>`, a feedline stroke or a terminal edge
/// each, the line between the end of the one and the end of the other that
/// lie nearest, when they lie no further apart than a coupler's ports do.
[[nodiscard]] std::vector<std::array<Point, 2>>
couplerRunsOf(const SlotGraph& graph) {
  std::map<std::int64_t, std::array<Point, 2>> ends;
  const auto numberOf =
      [](const std::string& name) -> std::optional<std::int64_t> {
    if (name.size() < 2 || name[0] != 'f') {
      return std::nullopt;
    }
    std::int64_t value = 0;
    const auto [rest, error] =
        std::from_chars(name.data() + 1, name.data() + name.size(), value);
    if (error != std::errc{} || rest != name.data() + name.size()) {
      return std::nullopt;
    }
    return value;
  };
  for (const auto& stroke : graph.strokes) {
    if (stroke.kind != "feedline" || stroke.points.empty()) {
      continue;
    }
    if (const auto k = numberOf(stroke.name); k.has_value()) {
      const auto& first = stroke.points.front();
      const auto& last = stroke.points.back();
      ends[*k] = {Point{first[0], first[1]}, Point{last[0], last[1]}};
    }
  }
  for (const auto& wire : graph.wires) {
    if (!wire.terminal) {
      continue;
    }
    if (const auto k = numberOf(wire.name);
        k.has_value() && !ends.contains(*k)) {
      ends[*k] = {Point{wire.source.cell[0], wire.source.cell[1]},
                  Point{wire.target.cell[0], wire.target.cell[1]}};
    }
  }
  std::vector<std::array<Point, 2>> runs;
  const auto most = COUPLER_RUN_CLEARANCES * graph.clearance;
  for (const auto& [k, mine] : ends) {
    const auto next = ends.find(k + 1);
    if (next == ends.end()) {
      continue;
    }
    std::array<Point, 2> best{};
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& a : mine) {
      for (const auto& b : next->second) {
        if (const auto d = distance(a, b); d < nearest) {
          nearest = d;
          best = {a, b};
        }
      }
    }
    if (nearest <= most) {
      runs.push_back(best);
    }
  }
  return runs;
}

/// The ways out of the ends of crossing slots. An end
/// lies the crossing reach across the edge from the slot's point. A way
/// counts when every cell of it is free and a clearance from the line of
/// every stub, as the router keeps a wire from a resonator's lead and a port
/// run. The raster holds a port run as the two walls of its band, half a
/// clearance to either side of its line, so a way keeps the clearance less
/// that half from the walls; a full clearance from the walls kept it one and
/// a half from the line, and on 21q f11 lost the slot that wire 28 crossed
/// at in the rip-up. A way keeps no clearance from the run of a feedline
/// through a coupler.
class EscapeWays {
public:
  /// A way out that a placed slot takes, and the end it leaves.
  struct Taken {
    Point end;
    EscapeWay way;
  };

  /// The ways out taken so far, by the place of their ends: the ways taken
  /// at ends near a point are found without a pass over every way.
  class TakenList {
  public:
    explicit TakenList(const double near = 1.0)
        : near_(near), side_(near + 1.0) {}

    void push(Taken taken) {
      squares_[keyOf(squareOf(taken.end.x), squareOf(taken.end.y))].push_back(
          items_.size());
      items_.push_back(std::move(taken));
    }
    void append(const TakenList& other) {
      for (const auto& taken : other.items_) {
        push(taken);
      }
    }
    /// Adds the way of every taken end at most `near` from `end` to `into`.
    void near(const Point& end, std::vector<const EscapeWay*>& into) const {
      // A short list is quicker read whole than by its squares.
      if (items_.size() <= SHORT) {
        for (const auto& taken : items_) {
          if (distance(taken.end, end) <= near_) {
            into.push_back(&taken.way);
          }
        }
        return;
      }
      const auto x = squareOf(end.x);
      const auto y = squareOf(end.y);
      for (auto sy = y - 1; sy <= y + 1; ++sy) {
        for (auto sx = x - 1; sx <= x + 1; ++sx) {
          const auto found = squares_.find(keyOf(sx, sy));
          if (found == squares_.end()) {
            continue;
          }
          for (const auto index : found->second) {
            if (distance(items_[index].end, end) <= near_) {
              into.push_back(&items_[index].way);
            }
          }
        }
      }
    }

  private:
    /// How many ways a list holds at most to be read whole.
    static constexpr std::size_t SHORT = 24;
    // A square a little wider than `near_`: two ends at most `near_` apart
    // lie in one square or in two that touch, whatever the rounding.
    [[nodiscard]] std::int64_t squareOf(const double value) const {
      return static_cast<std::int64_t>(std::floor(value / side_));
    }
    [[nodiscard]] static std::uint64_t keyOf(const std::int64_t x,
                                             const std::int64_t y) {
      return (static_cast<std::uint64_t>(x) << 32U) ^
             static_cast<std::uint64_t>(y & 0xFFFFFFFF);
    }

    double near_;
    double side_;
    std::vector<Taken> items_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> squares_;
  };

  EscapeWays(const SlotGraph& graph, const Grid& grid)
      : grid_(grid), clear_(graph.clearance),
        stubClear_(std::max(0.0, graph.clearance - graph.inflation) +
                   escapeStubMargin()),
        reach_(graph.crossingReach + slotReachMargin()),
        near_(clear_ + escapeApartMargin() +
              (2.0 * ((2.0 * ESCAPE_RADIUS) +
                      static_cast<double>(escapeStraight()) + 2.0))),
        bucket_(static_cast<std::int64_t>(std::ceil(clear_))),
        bucketsX_((grid.width / std::max<std::int64_t>(bucket_, 1)) + 3),
        bucketsY_((grid.height / std::max<std::int64_t>(bucket_, 1)) + 3),
        tilesX_((grid.width + TILE - 1) / TILE),
        tiles_(static_cast<std::size_t>(tilesX_ *
                                        ((grid.height + TILE - 1) / TILE)),
               NO_TILE) {
    // The stubs' cells by square of a clearance, the squares one apart from
    // the chip's edges so that the squares around every cell exist.
    const auto squares = static_cast<std::size_t>(bucketsX_ * bucketsY_);
    stubsFrom_.assign(squares + 1, 0);
    if (grid.stubCells != nullptr && bucket_ > 0) {
      for (const auto cell : *grid.stubCells) {
        ++stubsFrom_[squareOf(static_cast<std::int64_t>(cell) % grid.width,
                              static_cast<std::int64_t>(cell) / grid.width) +
                     1];
      }
      for (std::size_t square = 0; square < squares; ++square) {
        stubsFrom_[square + 1] += stubsFrom_[square];
      }
      stubs_.resize(stubsFrom_.back());
      auto next = stubsFrom_;
      for (const auto cell : *grid.stubCells) {
        const auto x = static_cast<std::int64_t>(cell) % grid.width;
        const auto y = static_cast<std::int64_t>(cell) / grid.width;
        stubs_[next[squareOf(x, y)]++] = {x, y};
      }
    }
    if (escapeCouplerRuns()) {
      runs_ = couplerRunsOf(graph);
    }
  }

  /// Two ends this near can have ways out within a clearance of each other.
  [[nodiscard]] double near() const { return near_; }

  /// The end of a slot on one side of its edge: 0 left of `direction`, 1
  /// right of it.
  [[nodiscard]] Point endOf(const Point& at, const Point& direction,
                            const std::size_t side) const {
    const auto sign = side == 0 ? 1.0 : -1.0;
    return {at.x - (direction.y * sign * reach_),
            at.y + (direction.x * sign * reach_)};
  }

  /// Every way out of the two ends of a slot.
  [[nodiscard]] EscapeOptions waysOf(const Point& at,
                                     const Point& direction) const {
    EscapeOptions both;
    for (std::size_t side = 0; side < 2; ++side) {
      const auto sign = side == 0 ? 1.0 : -1.0;
      const auto heading = headingOfStep(signOf(-direction.y * sign),
                                         signOf(direction.x * sign));
      if (!heading.has_value()) {
        continue;
      }
      const auto end = endOf(at, direction, side);
      const auto x0 = rounded(end.x);
      const auto y0 = rounded(end.y);
      for (const auto& move : primitives().of(*heading)) {
        if (move.exitHeading == *heading) {
          continue;
        }
        const auto turn = (move.exitHeading + 8U - *heading) % 8U;
        if (!escapeQuarterTurns() && (turn == 2U || turn == 6U)) {
          continue;
        }
        const auto onward = routing::headingVector(move.exitHeading);
        const auto straight = escapeStraight();
        const auto straightCell = [&](const std::int64_t step) {
          return Cell{x0 + move.dx + (step * onward.dx),
                      y0 + move.dy + (step * onward.dy)};
        };
        bool free = std::ranges::all_of(move.swept, [&](const auto& offset) {
          return freeAt(x0 + offset.dx, y0 + offset.dy);
        });
        for (std::int64_t step = 1; free && step <= straight; ++step) {
          const auto cell = straightCell(step);
          free = freeAt(cell.first, cell.second);
        }
        if (!free) {
          continue;
        }
        EscapeWay way{.start = {x0, y0}, .move = &move, .straight = straight};
        bool first = true;
        forEachCell(way, [&](const std::int64_t x, const std::int64_t y) {
          way.minX = first ? x : std::min(way.minX, x);
          way.maxX = first ? x : std::max(way.maxX, x);
          way.minY = first ? y : std::min(way.minY, y);
          way.maxY = first ? y : std::max(way.maxY, y);
          first = false;
        });
        both.at(side).push_back(way);
      }
    }
    return both;
  }

  /// Whether every cell of one way lies a clearance from every cell of the
  /// other.
  [[nodiscard]] bool apart(const EscapeWay& a, const EscapeWay& b) const {
    const auto keep = clear_ + escapeApartMargin();
    if (cellCount(a) == 0 || cellCount(b) == 0) {
      return true;
    }
    // Two ways whose bounds lie that far apart keep it cell by cell.
    const auto gapX = static_cast<double>(
        std::max<std::int64_t>({0, a.minX - b.maxX, b.minX - a.maxX}));
    const auto gapY = static_cast<double>(
        std::max<std::int64_t>({0, a.minY - b.maxY, b.minY - a.maxY}));
    if ((gapX * gapX) + (gapY * gapY) >= keep * keep) {
      return true;
    }
    cellsOf(a, cellsA_);
    cellsOf(b, cellsB_);
    for (const auto& [ax, ay] : cellsA_) {
      for (const auto& [bx, by] : cellsB_) {
        const auto dx = static_cast<double>(ax - bx);
        const auto dy = static_cast<double>(ay - by);
        if ((dx * dx) + (dy * dy) < keep * keep) {
          return false;
        }
      }
    }
    return true;
  }

  /// Call `visit` with every cell a way out covers, in order: the cells the
  /// router's turn sweeps, then the straight cells after it.
  template <typename Visit>
  static void forEachCell(const EscapeWay& way, Visit&& visit) {
    if (way.move == nullptr) {
      return;
    }
    const auto& [x0, y0] = way.start;
    const auto& move = *way.move;
    for (const auto& offset : move.swept) {
      visit(x0 + offset.dx, y0 + offset.dy);
    }
    const auto onward = routing::headingVector(move.exitHeading);
    for (std::int64_t step = 1; step <= way.straight; ++step) {
      visit(x0 + move.dx + (step * onward.dx),
            y0 + move.dy + (step * onward.dy));
    }
  }
  /// How many cells a way out covers.
  [[nodiscard]] static std::size_t cellCount(const EscapeWay& way) {
    return way.move == nullptr
               ? 0
               : way.move->swept.size() +
                     static_cast<std::size_t>(
                         std::max<std::int64_t>(way.straight, 0));
  }
  /// The cells a way out covers, in `into`.
  static void cellsOf(const EscapeWay& way, std::vector<Cell>& into) {
    into.clear();
    forEachCell(way, [&](const std::int64_t x, const std::int64_t y) {
      into.emplace_back(x, y);
    });
  }

  /// The curve of a way out, for the page: the router's turn, then the
  /// straight cells after it.
  [[nodiscard]] static std::vector<Point> curveOf(const EscapeWay& way) {
    std::vector<Point> curve;
    if (way.move == nullptr) {
      return curve;
    }
    const auto& [x0, y0] = way.start;
    const auto& move = *way.move;
    for (const auto& sample : move.samples) {
      curve.push_back({static_cast<double>(x0) + sample.x(),
                       static_cast<double>(y0) + sample.y()});
    }
    const auto onward = routing::headingVector(move.exitHeading);
    curve.push_back(
        {static_cast<double>(x0 + move.dx + (escapeStraight() * onward.dx)),
         static_cast<double>(y0 + move.dy + (escapeStraight() * onward.dy))});
    return curve;
  }

  /// For each end of a slot the first of its ways out that keeps a clearance
  /// from every way taken at an end near it; nothing when an end has none.
  [[nodiscard]] std::optional<std::vector<EscapeWay>>
  choose(const Slot& slot, const EscapeChoice& ways,
         const std::initializer_list<const TakenList*> taken) const {
    std::vector<EscapeWay> chosen;
    std::vector<const EscapeWay*> others;
    for (std::size_t side = 0; side < 2; ++side) {
      const auto end = endOf(slot.at, slot.direction, side);
      others.clear();
      for (const auto* list : taken) {
        list->near(end, others);
      }
      const auto fits = [&](const EscapeWay& way) {
        return std::ranges::all_of(
            others, [&](const EscapeWay* other) { return apart(*other, way); });
      };
      const auto found = std::ranges::find_if(*ways.at(side), fits);
      if (found == ways.at(side)->end()) {
        return std::nullopt;
      }
      chosen.push_back(*found);
    }
    return chosen;
  }

  /// The ways a placed slot takes, added to a list.
  void take(const Slot& slot, TakenList& into) const {
    for (std::size_t side = 0; side < slot.ways.size(); ++side) {
      into.push({.end = endOf(slot.at, slot.direction, side),
                 .way = slot.ways[side]});
    }
  }

private:
  /// The side of a square of cells whose `freeAt` is kept once found.
  static constexpr std::int64_t TILE = 64;
  static constexpr auto TILE_CELLS = static_cast<std::size_t>(TILE * TILE);
  /// A square of cells with nothing kept yet.
  static constexpr std::int32_t NO_TILE = -1;

  [[nodiscard]] static const routing::MovePrimitives& primitives() {
    static const routing::MovePrimitives table(ESCAPE_RADIUS);
    return table;
  }
  /// The square of a cell in `stubsFrom_`.
  [[nodiscard]] std::size_t squareOf(const std::int64_t x,
                                     const std::int64_t y) const {
    return static_cast<std::size_t>(((y / bucket_) + 1) * bucketsX_ +
                                    ((x / bucket_) + 1));
  }
  /// Whether a cell lies within a clearance of the run of a feedline
  /// through a coupler.
  [[nodiscard]] bool nearRun(const std::int64_t x, const std::int64_t y) const {
    const Point cell{static_cast<double>(x), static_cast<double>(y)};
    return std::ranges::any_of(runs_, [&](const auto& run) {
      return distanceToSegment(cell, run[0], run[1]) < clear_;
    });
  }

  /// Whether a cell lies nearer than `stubClear_` to a stub's cell in its
  /// square of a clearance or in one around it.
  [[nodiscard]] bool nearStub(const std::int64_t x,
                              const std::int64_t y) const {
    if (stubs_.empty()) {
      return false;
    }
    const auto middle = squareOf(x, y);
    for (const auto row : {middle - static_cast<std::size_t>(bucketsX_), middle,
                           middle + static_cast<std::size_t>(bucketsX_)}) {
      for (auto at = stubsFrom_[row - 1]; at < stubsFrom_[row + 2]; ++at) {
        const auto& [sx, sy] = stubs_[at];
        const auto dx = static_cast<double>(sx - x);
        const auto dy = static_cast<double>(sy - y);
        if ((dx * dx) + (dy * dy) < stubClear_ * stubClear_) {
          return true;
        }
      }
    }
    return false;
  }
  /// Whether a way out may cover a cell, kept per cell once found.
  [[nodiscard]] bool freeAt(const std::int64_t x, const std::int64_t y) const {
    if (!grid_.inside(x, y)) {
      return false;
    }
    auto& tile =
        tiles_[static_cast<std::size_t>(((y / TILE) * tilesX_) + (x / TILE))];
    if (tile == NO_TILE) {
      tile = static_cast<std::int32_t>(known_.size() / TILE_CELLS);
      known_.resize(known_.size() + TILE_CELLS, UNKNOWN);
    }
    auto& known =
        known_[(static_cast<std::size_t>(tile) * TILE_CELLS) +
               static_cast<std::size_t>(((y % TILE) * TILE) + (x % TILE))];
    if (known == UNKNOWN) {
      const auto value = grid_.at(x, y);
      known =
          (value == 0 || value >= CHAMBER) && !nearStub(x, y) && !nearRun(x, y)
              ? FREE
              : TAKEN;
    }
    return known == FREE;
  }

  /// What `known_` holds for a cell.
  static constexpr std::uint8_t UNKNOWN = 0;
  static constexpr std::uint8_t FREE = 1;
  static constexpr std::uint8_t TAKEN = 2;

  const Grid& grid_;
  double clear_;
  /// How far a way out keeps from the wall of a stub.
  double stubClear_;
  double reach_;
  double near_;
  /// The stubs' cells by square of a clearance: square s holds
  /// `stubs_[stubsFrom_[s]]` up to `stubs_[stubsFrom_[s + 1]]`.
  std::int64_t bucket_;
  std::int64_t bucketsX_;
  std::int64_t bucketsY_;
  std::vector<std::size_t> stubsFrom_;
  std::vector<Cell> stubs_;
  /// `freeAt` per cell once found, in squares of TILE cells: per square
  /// the number of its block in `known_`, or NO_TILE.
  std::int64_t tilesX_;
  mutable std::vector<std::int32_t> tiles_;
  mutable std::vector<std::uint8_t> known_;
  /// The runs of the feedlines through the couplers.
  std::vector<std::array<Point, 2>> runs_;
  /// The cells of the two ways `apart` compares.
  mutable std::vector<Cell> cellsA_;
  mutable std::vector<Cell> cellsB_;
};

/// Whether a gate runs from a feedline edge to a static obstacle: one end
/// on a feedline edge (`f<n>`, at its coupler or launcher too), the other
/// on artwork or the chip's border.
[[nodiscard]] bool feedlineToStatic(const std::vector<std::string>& ends) {
  const auto feedline = [](const std::string& end) {
    return end.size() >= 2 && end[0] == 'f' &&
           std::isdigit(static_cast<unsigned char>(end[1])) != 0;
  };
  const auto fixed = [](const std::string& end) {
    return end.starts_with("artwork") || end.starts_with("border");
  };
  return ends.size() == 2 && ((feedline(ends[0]) && fixed(ends[1])) ||
                              (fixed(ends[0]) && feedline(ends[1])));
}
/// Take the settings of one check from the graph it judges.
void useSettings(const SlotGraph& graph) {
  settings() = Settings{.margin = graph.capacityMargin,
                        .marginFrom = graph.capacityMarginFromWires,
                        .resonatorPassage = graph.resonatorPassage,
                        .slotPinCells = graph.slotPinCells,
                        .escapeStraight = graph.escapeStraight,
                        .escapeStubMargin = graph.escapeStubMargin};
}

/// How far the ends of two lines may lie apart for them to be one line.
constexpr double MOUTH_TWIN_CELLS = 2.0;

/// Whether the segments p-q and c-d cross at a point inside both.
[[nodiscard]] bool crossesLine(const Point& p, const Point& q, const Point& c,
                               const Point& d) {
  const auto side = [](const Point& a, const Point& b, const Point& x) {
    return ((b.x - a.x) * (x.y - a.y)) - ((b.y - a.y) * (x.x - a.x));
  };
  return side(p, q, c) * side(p, q, d) < 0 && side(c, d, p) * side(c, d, q) < 0;
}

/// Whether a point lies in a polygon, by a ray to the right.
[[nodiscard]] bool inPolygon(const Point& q,
                             const std::vector<Point>& polygon) {
  bool inside = false;
  const auto count = polygon.size();
  for (std::size_t m = 0; m < count; ++m) {
    const auto& [x1, y1] = polygon[m];
    const auto& [x2, y2] = polygon[(m + 1) % count];
    if ((y1 > q.y) != (y2 > q.y)) {
      const auto xc = x1 + (((q.y - y1) * (x2 - x1)) / (y2 - y1));
      if (xc > q.x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

/// The chamber beside a slot on one side: the first chamber cell from the
/// crossing reach on out, along the unit normal (nx, ny).
[[nodiscard]] std::optional<
    std::pair<std::int64_t, std::array<std::int64_t, 2>>>
sideChamber(const Grid& grid, const Point& at, const double nx, const double ny,
            const double reach) {
  const auto first = static_cast<std::int64_t>(reach);
  for (auto step = first; step < first + SIDE_STEPS; ++step) {
    const auto x = rounded(at.x + (static_cast<double>(step) * nx));
    const auto y = rounded(at.y + (static_cast<double>(step) * ny));
    if (grid.inside(x, y) && grid.at(x, y) >= CHAMBER) {
      return std::pair{static_cast<std::int64_t>(grid.at(x, y) - CHAMBER),
                       std::array{x, y}};
    }
  }
  return std::nullopt;
}

/// The chambers that touch a gate's line: samples along the line, two
/// cells or more from its ends, a cell and a half to either side. A sample
/// whose step off the line crosses the line of another gate in `others`
/// lies beyond that gate and counts for no side.
[[nodiscard]] std::array<std::set<std::int64_t>, 2>
lineSides(const Grid& grid, const Edge& edge,
          const std::vector<std::pair<Point, Point>>& others) {
  std::array<std::set<std::int64_t>, 2> found;
  const auto& [ax, ay] = edge.from;
  const auto& [bx, by] = edge.to;
  const auto length = std::hypot(bx - ax, by - ay);
  if (length < 3.0) {
    return found;
  }
  const auto nx = -(by - ay) / length;
  const auto ny = (bx - ax) / length;
  const auto count = static_cast<std::size_t>(
      std::max<std::int64_t>(5, static_cast<std::int64_t>(length)));
  for (const auto t : unitSteps(count)) {
    if (t * length < 2.0 || (1 - t) * length < 2.0) {
      continue;
    }
    const Point p{ax + (t * (bx - ax)), ay + (t * (by - ay))};
    for (std::size_t which = 0; which < 2; ++which) {
      const auto side = which == 0 ? 1.5 : -1.5;
      const auto x = rounded(p.x + (side * nx));
      const auto y = rounded(p.y + (side * ny));
      const Point q{static_cast<double>(x), static_cast<double>(y)};
      if (std::ranges::any_of(others, [&](const auto& other) {
            return crossesLine(p, q, other.first, other.second);
          })) {
        continue;
      }
      if (grid.inside(x, y) && grid.at(x, y) >= CHAMBER) {
        found[which].insert(grid.at(x, y) - CHAMBER);
      }
    }
  }
  return found;
}

/// What the cells of the chip are, as bits: a row of the chip is a run of
/// words of 64 cells, bit b of word w for cell 64 w + b. A word is read from
/// the raster when first asked for, so a check reads only the cells near
/// its slots and lines.
class RasterBits {
public:
  /// What 64 cells of a row are.
  struct Word {
    /// A chamber or a cut line, which a wire may pass.
    std::uint64_t open;
    /// A stub's wall.
    std::uint64_t stub;
    /// A feedline's wall.
    std::uint64_t feedline;
    /// A cell of a port run's slot.
    std::uint64_t portSlot;
  };

  RasterBits(const Grid& grid, const int stubCode, const int feedlineCode,
             const std::vector<std::array<std::int64_t, 2>>& portSlots)
      : grid_(grid), stubCode_(stubCode), feedlineCode_(feedlineCode),
        words_(static_cast<std::size_t>((grid.width + 63) / 64)),
        known_(words_ * static_cast<std::size_t>(grid.height), 0),
        cache_(std::make_unique_for_overwrite<Word[]>(known_.size())),
        portSlots_(known_.size(), 0) {
    for (const auto& [x, y] : portSlots) {
      if (grid.inside(x, y)) {
        portSlots_[wordOf(x, y)] |= bitOf(x);
      }
    }
  }

  /// Whether a cell inside the chip is a cell of a port run's slot.
  [[nodiscard]] bool portSlotAt(const std::int64_t x,
                                const std::int64_t y) const {
    return (portSlots_[wordOf(x, y)] & bitOf(x)) != 0;
  }

  /// How many words a row takes.
  [[nodiscard]] std::size_t words() const { return words_; }

  /// Word `index` of the chip, row by row.
  [[nodiscard]] const Word& at(const std::size_t index) {
    if (known_[index] == 0) {
      Word word{
          .open = 0, .stub = 0, .feedline = 0, .portSlot = portSlots_[index]};
      const auto y = static_cast<std::int64_t>(index / words_);
      const auto first = static_cast<std::int64_t>(index % words_) * 64;
      const auto count = static_cast<unsigned>(
          std::min<std::int64_t>(64, grid_.width - first));
      const auto base = grid_.cell(first, y);
      for (unsigned b = 0; b < count; ++b) {
        const auto value = static_cast<int>(grid_.codeAt(base + b));
        word.open |= static_cast<std::uint64_t>(value >= CHAMBER || value == 0)
                     << b;
        word.stub |= static_cast<std::uint64_t>(value == stubCode_) << b;
        word.feedline |= static_cast<std::uint64_t>(value == feedlineCode_)
                         << b;
      }
      // A free cell is neither wall, whatever the codes of the walls.
      word.stub &= ~word.open;
      word.feedline &= ~word.open;
      cache_[index] = word;
      known_[index] = 1;
    }
    return cache_[index];
  }

  /// The 64 cells of row y from cell x >= 0 on; cells past the row are none.
  [[nodiscard]] Word span(const std::int64_t x, const std::int64_t y) {
    const auto row = static_cast<std::size_t>(y) * words_;
    const auto w = static_cast<std::size_t>(x / 64);
    const auto shift = static_cast<unsigned>(x % 64);
    const auto& low = at(row + w);
    if (shift == 0) {
      return low;
    }
    Word out{.open = low.open >> shift,
             .stub = low.stub >> shift,
             .feedline = low.feedline >> shift,
             .portSlot = low.portSlot >> shift};
    if (w + 1 < words_) {
      const auto& high = at(row + w + 1);
      out.open |= high.open << (64U - shift);
      out.stub |= high.stub << (64U - shift);
      out.feedline |= high.feedline << (64U - shift);
      out.portSlot |= high.portSlot << (64U - shift);
    }
    return out;
  }

private:
  [[nodiscard]] std::size_t wordOf(const std::int64_t x,
                                   const std::int64_t y) const {
    return (static_cast<std::size_t>(y) * words_) +
           static_cast<std::size_t>(x / 64);
  }
  [[nodiscard]] static std::uint64_t bitOf(const std::int64_t x) {
    return std::uint64_t{1} << static_cast<unsigned>(x % 64);
  }

  const Grid& grid_;
  int stubCode_;
  int feedlineCode_;
  std::size_t words_;
  /// Per word: whether `cache_` holds it.
  std::vector<std::uint8_t> known_;
  std::unique_ptr<Word[]> cache_;
  /// The cells of the port runs' slots, every word known from the start.
  std::vector<std::uint64_t> portSlots_;
};

/// The cells a crossing slot covers, as offsets from a cell, in the order
/// `SlotPlacer::examine` samples them. A slot at a point of whole numbers
/// covers the same offsets from its point as every slot at a point of the
/// same parity on the same step of an edge, so the placement finds them
/// once per step and parity (`SlotPlacer::footprintOf`).
struct Footprint {
  /// The offsets, never one twice in a row.
  std::vector<std::array<std::int64_t, 2>> cells;
  /// Per offset: whether a stub's wall there leaves the slot free.
  std::vector<std::uint8_t> stubFree;
  /// Per offset: how many points along a straight edge the point nearest to
  /// the cell lies from the slot's point, which `SlotPlacer::examine` tests
  /// first for the edge's own wall; empty for a footprint of one point.
  std::vector<std::int64_t> nearestStep;
  /// Per offset: whether that point lies near enough for the edge's own
  /// wall to leave the slot free there, and how many points from the slot's
  /// point the furthest such point lies.
  std::vector<std::uint8_t> nearOwn;
  std::int64_t ownSteps = 0;
  /// The bounds of the offsets, and a bit per offset row by row.
  std::int64_t minX = 0;
  std::int64_t maxX = 0;
  std::int64_t minY = 0;
  std::int64_t maxY = 0;
  std::size_t words = 0;
  std::vector<std::uint64_t> bits;
  /// When a row takes one word: per row, the bits of the offsets where a
  /// stub's wall leaves the slot free and where the edge's own wall does.
  std::vector<std::uint64_t> stubFreeBits;
  std::vector<std::uint64_t> nearOwnBits;

  /// Fill the bounds and the bits from the offsets.
  void index() {
    if (cells.empty()) {
      return;
    }
    const auto [left, right] = std::ranges::minmax(
        cells, {}, [](const auto& cell) { return cell[0]; });
    const auto [low, high] = std::ranges::minmax(
        cells, {}, [](const auto& cell) { return cell[1]; });
    minX = left[0];
    maxX = right[0];
    minY = low[1];
    maxY = high[1];
    words = static_cast<std::size_t>((maxX - minX) / 64) + 1;
    bits.assign(static_cast<std::size_t>(maxY - minY + 1) * words, 0);
    for (const auto& [dx, dy] : cells) {
      const auto column = static_cast<std::size_t>(dx - minX);
      bits[(static_cast<std::size_t>(dy - minY) * words) + (column / 64)] |=
          std::uint64_t{1} << (column % 64);
    }
    if (words == 1) {
      stubFreeBits.assign(bits.size(), 0);
      nearOwnBits.assign(bits.size(), 0);
      for (std::size_t index = 0; index < cells.size(); ++index) {
        const auto& [dx, dy] = cells[index];
        const auto row = static_cast<std::size_t>(dy - minY);
        const auto bit = std::uint64_t{1} << static_cast<unsigned>(dx - minX);
        if (stubFree[index] != 0) {
          stubFreeBits[row] |= bit;
        }
        if (!nearOwn.empty() && nearOwn[index] != 0) {
          nearOwnBits[row] |= bit;
        }
      }
    }
  }
  /// Whether the footprint covers an offset.
  [[nodiscard]] bool covers(const std::int64_t dx,
                            const std::int64_t dy) const {
    if (dx < minX || dx > maxX || dy < minY || dy > maxY) {
      return false;
    }
    const auto column = static_cast<std::size_t>(dx - minX);
    return ((bits[(static_cast<std::size_t>(dy - minY) * words) +
                  (column / 64)] >>
             (column % 64)) &
            1U) != 0;
  }
};

/// The crossing slots of every feedline edge of the graph. Slots are laid
/// from either end of the edge, each at the first place past the one
/// before where it fits; the end that gives more slots counts. The same
/// number is spread evenly (`placeEvenly`). A slot fits a place only when
/// each of its ends
/// has a way out a clearance from the ways out of the slots placed before
/// (`EscapeWays::choose`), so that a slot moves on past places where its
/// wire could not get away.
struct SlotPlacer {
  const SlotGraph& graph;
  const Grid& grid;
  /// Whether the places where no slot fits, and why, are wanted (for the
  /// page); `Placement::refused` stays empty otherwise.
  bool explain = false;
  double clearance = 0.0;
  double reach = 0.0;
  double along = 0.0;
  double inflation = 0.0;
  int feedlineCode = FEEDLINE;
  int stubCode = 0;
  std::vector<std::string> wallKinds;
  std::set<std::pair<std::int64_t, std::int64_t>> portSlotsOutside;
  /// The cells of the chip as bits, read as `examine` asks for them.
  mutable std::optional<RasterBits> bits;
  /// The feedline routing's crossing rule over the edges between couplers
  /// (`slotCrossingRule`), and how far it reaches.
  routing::CrossingConstraints rule;
  int ruleReach = 0;
  /// Per heading and mask of the rule: whether a wire on the heading may
  /// enter a cell with that mask.
  std::array<std::array<std::uint8_t, 256>, routing::NUM_HEADINGS> ruleAllows{};
  /// The ways out of the slots, and those the slots of
  /// the edges placed so far take.
  std::optional<EscapeWays> escapes;
  EscapeWays::TakenList taken;

  SlotPlacer(const SlotGraph& graphIn, const Grid& gridIn, const bool explainIn)
      : graph(graphIn), grid(gridIn), explain(explainIn),
        clearance(graphIn.clearance),
        reach(graphIn.crossingReach + slotReachMargin()), along(clearance),
        inflation(graphIn.inflation), wallKinds(graphIn.wallKinds) {
    if (slotEscape()) {
      escapes.emplace(graphIn, gridIn);
      taken = EscapeWays::TakenList(escapes->near());
    }
    const auto kind = std::ranges::find(wallKinds, "feedline");
    if (kind == wallKinds.end()) {
      throw std::invalid_argument("slot check: no feedline wall kind");
    }
    feedlineCode = 1 + static_cast<int>(kind - wallKinds.begin());
    if (const auto stub = std::ranges::find(wallKinds, "stub");
        stub != wallKinds.end()) {
      stubCode = 1 + static_cast<int>(stub - wallKinds.begin());
    }
    for (const auto& [x, y] : graph.portSlots) {
      if (!grid.inside(x, y)) {
        portSlotsOutside.emplace(x, y);
      }
    }
    bits.emplace(grid, stubCode, feedlineCode, graph.portSlots);
  }

  /// Whether a slot with `footprint` from `origin` on fits, row by row of 64
  /// cells: true when no cell refuses it, false when a cell refuses it
  /// whatever the edge's own wall, nothing when only cells of a feedline's
  /// wall might refuse it, which the search of `examine` decides. `straight`
  /// says that the edge's own wall leaves the slot free where the footprint
  /// says so. Nothing, too, for a footprint wider than a word or reaching
  /// past the chip.
  [[nodiscard]] std::optional<bool> quickly(const Footprint& footprint,
                                            const Cell& origin,
                                            const bool straight) const {
    const auto& [ox, oy] = origin;
    if (footprint.stubFreeBits.empty() || ox + footprint.minX < 0 ||
        ox + footprint.maxX >= grid.width || oy + footprint.minY < 0 ||
        oy + footprint.maxY >= grid.height) {
      return std::nullopt;
    }
    bool unsure = false;
    for (std::size_t row = 0; row < footprint.bits.size(); ++row) {
      const auto cells = footprint.bits[row];
      const auto stubFree = footprint.stubFreeBits[row];
      const auto nearOwn = straight ? footprint.nearOwnBits[row] : 0;
      const auto word =
          bits->span(ox + footprint.minX,
                     oy + footprint.minY + static_cast<std::int64_t>(row));
      if ((cells & ((word.portSlot & ~stubFree) |
                    (~word.open & ~(word.stub & stubFree) & ~word.feedline))) !=
          0) {
        return false;
      }
      unsure = unsure || (cells & word.feedline & ~nearOwn) != 0;
    }
    if (unsure) {
      return std::nullopt;
    }
    return true;
  }

  /// The step of the edge at point k, or nothing at a bend.
  [[nodiscard]] static std::optional<Point>
  heading(const std::vector<Point>& points, const std::size_t k) {
    if (k == 0 || k + 1 >= points.size()) {
      return std::nullopt;
    }
    const auto a = minus(points[k], points[k - 1]);
    const auto b = minus(points[k + 1], points[k]);
    if (a.x == b.x && a.y == b.y) {
      return a;
    }
    return std::nullopt;
  }

  struct Placement {
    std::vector<Slot> placed;
    /// The places where no slot fits, under `explain`.
    std::vector<Refused> refused;
    double length = 0.0;
  };

  /// What a slot centred on point k of a line comes to, other slots aside:
  /// no slot (`skip`: no straight run of a slot's length there), a refused
  /// place, or a slot and the cells it covers: `footprint` from cell
  /// (`x`, `y`) on.
  struct Examined {
    enum class Kind { Skip, Refused, Fits } kind = Kind::Skip;
    std::string why;
    Slot slot{};
    const Footprint* footprint = nullptr;
    std::int64_t x = 0;
    std::int64_t y = 0;
  };

  /// What one direction of a feedline edge found at each of its points,
  /// examined once.
  using Examinations = std::vector<std::optional<Examined>>;

  /// The ways out of the slots at the points of one feedline edge, found
  /// once for both directions: per point of the edge as given, the
  /// directions of a slot there and the ways out for each. A point has at
  /// most two: the direction of the edge as given and the opposite one,
  /// which differ at a bend.
  using WaysAt = std::vector<std::vector<std::pair<Point, EscapeOptions>>>;

  /// The footprint of a slot at point `at` on `step` of an edge, and the
  /// cell its offsets start from. A point of whole numbers takes the
  /// footprint of its step and parity, found at the point (0 or 1, 0 or 1)
  /// of the same parity, with offsets from the point; any other point gets
  /// a footprint of its own, with offsets from cell (0, 0).
  ///
  /// The rounding of a sample of the slot shifts with the point: the sums
  /// of a step along the axes are exact, and the rounding of halves to even
  /// keeps its parity; the samples of any other step lie further than a
  /// millionth of a cell from a half, which the error of a sum near the
  /// chip's size does not bridge. A step whose samples come nearer to a half
  /// gives every point a footprint of its own.
  [[nodiscard]] std::pair<const Footprint*, Cell>
  footprintOf(const Point& at, const Point& step, const double ux,
              const double uy) const {
    constexpr double WHOLE_LIMIT = 1048576.0;
    const auto whole = [&](const double value) {
      return std::nearbyint(value) == value && std::abs(value) < WHOLE_LIMIT;
    };
    if (whole(at.x) && whole(at.y)) {
      const auto x = static_cast<std::int64_t>(at.x);
      const auto y = static_cast<std::int64_t>(at.y);
      const auto px = x & 1;
      const auto py = y & 1;
      const auto key = std::tuple{step.x, step.y, px, py};
      auto found = footprints_.find(key);
      if (found == footprints_.end()) {
        auto made = sample({static_cast<double>(px), static_cast<double>(py)},
                           ux, uy, true);
        if (made.has_value()) {
          made->cells.shrink_to_fit();
          nearest(*made, step, (inflation + 1.5) * (inflation + 1.5));
          made->index();
        }
        found = footprints_.emplace(key, std::move(made)).first;
      }
      if (found->second.has_value()) {
        return {&*found->second, Cell{x, y}};
      }
    }
    auto& own = ownFootprints_.emplace_back(*sample(at, ux, uy, false));
    own.index();
    return {&own, Cell{0, 0}};
  }

  /// The footprint of a slot at `at`, with offsets from the cell (0, 0) when
  /// `shared` is false and from `at` itself when it is true; nothing when it
  /// is to be shared and a sample comes too near a half.
  [[nodiscard]] std::optional<Footprint> sample(const Point& at,
                                                const double ux,
                                                const double uy,
                                                const bool shared) const {
    constexpr double HALF_MARGIN = 1e-6;
    const auto& [cx, cy] = at;
    const auto nx = -uy;
    const auto ny = ux;
    const auto half = along / 2.0;
    const auto unit = [](const double value) {
      return value == 0.0 || std::abs(value) == 1.0;
    };
    const auto halves = [](const double value) {
      return std::nearbyint(value * 2.0) == value * 2.0;
    };
    const bool exact = unit(ux) && unit(uy) && halves(half) && halves(reach);
    const auto clearOfHalf = [&](const double value) {
      return std::abs(value - std::nearbyint(value)) < 0.5 - HALF_MARGIN;
    };
    // Stubs count over the middle of the rectangle alone.
    const auto stubAlong = half - slotStubGap() + 1e-9;
    Footprint out;
    const auto ox = shared ? rounded(cx) : 0;
    const auto oy = shared ? rounded(cy) : 0;
    for (double t = -half; t <= half + 1e-9; t += 0.5) {
      for (double s = -reach; s <= reach + 1e-9; s += 0.5) {
        if (shared && !exact &&
            (!clearOfHalf((t * ux) + (s * nx)) ||
             !clearOfHalf((t * uy) + (s * ny)))) {
          return std::nullopt;
        }
        const std::array cell{rounded(cx + (t * ux) + (s * nx)) - ox,
                              rounded(cy + (t * uy) + (s * ny)) - oy};
        // Half-cell samples fall on the same cell twice in a row; once is
        // enough, and the order of the first ones stays.
        if (out.cells.empty() || out.cells.back() != cell) {
          out.cells.push_back(cell);
        }
      }
    }
    out.stubFree.reserve(out.cells.size());
    for (const auto& [dx, dy] : out.cells) {
      out.stubFree.push_back(
          std::abs(((static_cast<double>(dx + ox) - cx) * ux) +
                   ((static_cast<double>(dy + oy) - cy) * uy)) > stubAlong
              ? 1
              : 0);
    }
    return out;
  }

  /// Per offset of a footprint, the number of steps along a straight edge
  /// on `step` to the point nearest the cell, and whether that point lies
  /// within `ownReach`, squared, of the cell.
  static void nearest(Footprint& footprint, const Point& step,
                      const double ownReach) {
    footprint.nearestStep.reserve(footprint.cells.size());
    footprint.nearOwn.reserve(footprint.cells.size());
    for (const auto& [dx, dy] : footprint.cells) {
      std::int64_t best = 0;
      auto bestApart = std::numeric_limits<double>::infinity();
      for (auto j = -OWN_WINDOW; j <= OWN_WINDOW; ++j) {
        const auto ex =
            static_cast<double>(dx) - (static_cast<double>(j) * step.x);
        const auto ey =
            static_cast<double>(dy) - (static_cast<double>(j) * step.y);
        if (const auto apart = (ex * ex) + (ey * ey); apart < bestApart) {
          bestApart = apart;
          best = j;
        }
      }
      footprint.nearestStep.push_back(best);
      const bool near = bestApart <= ownReach;
      footprint.nearOwn.push_back(near ? 1 : 0);
      if (near) {
        footprint.ownSteps = std::max(footprint.ownSteps, std::abs(best));
      }
    }
  }

  /// A slot at `at` across an edge on the unit step (ux, uy), with no way
  /// out yet.
  [[nodiscard]] Slot slotAt(const Point& at, const double ux,
                            const double uy) const {
    const auto& [cx, cy] = at;
    const auto nx = -uy;
    const auto ny = ux;
    const auto half = along / 2.0;
    std::array<Point, 4> corners{};
    constexpr std::array<std::pair<double, double>, 4> SIGNS{
        {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}};
    for (std::size_t corner = 0; corner < 4; ++corner) {
      const auto [a, b] = SIGNS[corner];
      corners[corner] = {cx + ((a * half) * ux) + ((b * reach) * nx),
                         cy + ((a * half) * uy) + ((b * reach) * ny)};
    }
    return {.at = at,
            .corners = corners,
            .direction = {ux, uy},
            .ways = {},
            .options = {}};
  }

  [[nodiscard]] Examined examine(const std::vector<Point>& line,
                                 const std::size_t here) const {
    Examined out;
    const auto count = static_cast<std::ptrdiff_t>(line.size());
    const auto k = static_cast<std::ptrdiff_t>(here);
    // The slot sits on one straight piece: every step of the edge it
    // covers along the edge has the same direction; it may reach the
    // point where the piece bends.
    auto step = heading(line, here);
    if (!step.has_value() && k + 1 < count) {
      step = minus(line[here + 1], line[here]);
    }
    if (!step.has_value()) {
      return out;
    }
    const auto norm = std::hypot(step->x, step->y);
    const auto ux = step->x / norm;
    const auto uy = step->y / norm;
    const auto halfSteps =
        static_cast<std::ptrdiff_t>(std::ceil((along / 2.0 / norm) - 1e-9));
    if (k - halfSteps < 0 || k + halfSteps >= count) {
      return out;
    }
    for (auto q = k - halfSteps; q < k + halfSteps; ++q) {
      const auto piece = minus(line[static_cast<std::size_t>(q) + 1],
                               line[static_cast<std::size_t>(q)]);
      if (piece.x != step->x || piece.y != step->y) {
        return out;
      }
    }
    const auto& [cx, cy] = line[here];
    out.slot = slotAt(line[here], ux, uy);
    out.kind = Examined::Kind::Refused;
    if (!rule.empty() && !crossable(cx, cy, *step)) {
      out.why = "crossing rule";
      return out;
    }
    const auto [footprint, origin] = footprintOf(line[here], *step, ux, uy);
    out.footprint = footprint;
    out.x = origin.first;
    out.y = origin.second;

    // The edge's own straight piece: the points of the edge within
    // OWN_WINDOW of the slot's.
    const auto first =
        static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, k - OWN_WINDOW));
    const auto stop =
        static_cast<std::size_t>(std::min(count, k + OWN_WINDOW + 1));
    const auto ownReach = (inflation + 1.5) * (inflation + 1.5);
    // Whether the edge runs straight on `step` over the points nearest the
    // cells of the footprint, all inside the window: a cell of the edge's
    // own wall is near its nearest point then exactly when the footprint
    // says so.
    const auto own = footprint->ownSteps;
    bool straight = !footprint->nearOwn.empty() && own <= OWN_WINDOW &&
                    k - own >= 0 && k + own < count;
    for (auto j = -own; straight && j <= own; ++j) {
      const auto& point = line[static_cast<std::size_t>(k + j)];
      straight = point.x == cx + (static_cast<double>(j) * step->x) &&
                 point.y == cy + (static_cast<double>(j) * step->y);
    }
    // Row by row first: a cell by cell pass only where a feedline's wall
    // needs the search, or for the cell that refuses the slot, which only
    // the page shows.
    if (const auto fits = quickly(*footprint, origin, straight);
        fits.has_value() && (*fits || !explain)) {
      out.kind = *fits ? Examined::Kind::Fits : Examined::Kind::Refused;
      return out;
    }
    const auto& cells = footprint->cells;
    for (std::size_t index = 0; index < cells.size(); ++index) {
      const auto x = origin.first + cells[index][0];
      const auto y = origin.second + cells[index][1];
      const bool in = grid.inside(x, y);
      const bool stubFree = footprint->stubFree[index] != 0;
      if (!stubFree &&
          (in ? bits->portSlotAt(x, y) : portSlotsOutside.contains({x, y}))) {
        out.why = "slot of a port run";
        return out;
      }
      const int value = in ? std::min(grid.at(x, y), CHAMBER) : 1;
      if (value == CHAMBER || value == 0 || (value == stubCode && stubFree)) {
        continue;
      }
      // The edge's own wall only of the straight piece the slot sits on,
      // not of another arm of the edge bent back beside it.
      if (value == feedlineCode) {
        const auto nearPoint = [&](const std::size_t p) {
          const auto dx = static_cast<double>(x) - line[p].x;
          const auto dy = static_cast<double>(y) - line[p].y;
          return (dx * dx) + (dy * dy) <= ownReach;
        };
        if (straight && footprint->nearOwn[index] != 0) {
          continue;
        }
        // The point nearest the cell on a straight piece first: the cell
        // is near some point of the window, and on the slot's own piece
        // near that one.
        if (!footprint->nearestStep.empty()) {
          const auto p = k + footprint->nearestStep[index];
          if (std::cmp_greater_equal(p, first) && std::cmp_less(p, stop) &&
              nearPoint(static_cast<std::size_t>(p))) {
            continue;
          }
        }
        // From the slot's own point outward: a cell of the slot is near
        // a point close to it along the edge, so the search stops soon.
        bool near = false;
        for (std::size_t offset = 0; !near; ++offset) {
          const bool below = here >= first + offset;
          const bool above = here + offset < stop;
          if (!below && !above) {
            break;
          }
          near = (below && nearPoint(here - offset)) ||
                 (above && offset > 0 && nearPoint(here + offset));
        }
        if (near) {
          continue;
        }
      }
      out.why = value >= 1 && value <= 5
                    ? wallKinds.at(static_cast<std::size_t>(value - 1))
                    : std::string("wall");
      return out;
    }
    out.kind = Examined::Kind::Fits;
    return out;
  }

  /// What `examine` found at point k of a line, examined once.
  [[nodiscard]] const Examined& examined(const std::vector<Point>& line,
                                         Examinations& memo,
                                         const std::size_t k) const {
    if (!memo[k].has_value()) {
      memo[k] = examine(line, k);
    }
    return *memo[k];
  }

  /// What `examine` finds at point k of `line`, the edge `forward` run the
  /// other way, examined once. At a point inside a straight piece the slot
  /// covers the same samples on either direction, keeps the same crossing
  /// rule and is near the same points of the edge, so it comes to what the
  /// forward slot comes to, mirrored; only the first cell that refuses it
  /// can differ, which `explain` asks for.
  [[nodiscard]] const Examined& examinedBack(const std::vector<Point>& line,
                                             Examinations& memo,
                                             const std::size_t k,
                                             const std::vector<Point>& forward,
                                             Examinations& forwardMemo) const {
    if (memo[k].has_value()) {
      return *memo[k];
    }
    const auto step = heading(line, k);
    if (!step.has_value()) {
      memo[k] = examine(line, k);
      return *memo[k];
    }
    const auto& mirror = examined(forward, forwardMemo, line.size() - 1 - k);
    if (mirror.kind == Examined::Kind::Skip ||
        (mirror.kind == Examined::Kind::Refused && explain)) {
      memo[k] =
          mirror.kind == Examined::Kind::Skip ? Examined{} : examine(line, k);
      return *memo[k];
    }
    // The direction as `examine` finds it, zeros of their own sign.
    const auto norm = std::hypot(step->x, step->y);
    memo[k] = Examined{.kind = mirror.kind,
                       .why = mirror.why,
                       .slot = slotAt(line[k], step->x / norm, step->y / norm),
                       .footprint = mirror.footprint,
                       .x = mirror.x,
                       .y = mirror.y};
    return *memo[k];
  }

  /// Whether two slots that fit share a cell.
  [[nodiscard]] static bool overlap(const Examined& a, const Examined& b) {
    const auto& fa = *a.footprint;
    const auto& fb = *b.footprint;
    const auto shiftX = a.x - b.x;
    const auto shiftY = a.y - b.y;
    if (fa.cells.empty() || fb.cells.empty() || fa.maxX + shiftX < fb.minX ||
        fa.minX + shiftX > fb.maxX || fa.maxY + shiftY < fb.minY ||
        fa.minY + shiftY > fb.maxY) {
      return false;
    }
    return std::ranges::any_of(fa.cells, [&](const auto& cell) {
      return fb.covers(cell[0] + shiftX, cell[1] + shiftY);
    });
  }

  /// The ways out of a slot at point k of the edge as given (`ways`) on
  /// `direction`, found once. The same slot on the opposite direction has
  /// the same ways out with its two ends swapped.
  [[nodiscard]] EscapeChoice waysAt(WaysAt& ways, const std::size_t k,
                                    const Point& at,
                                    const Point& direction) const {
    auto& known = ways[k];
    for (const auto& [towards, options] : known) {
      if (towards.x == direction.x && towards.y == direction.y) {
        return {options.data(), options.data() + 1};
      }
      if (towards.x == -direction.x && towards.y == -direction.y) {
        return {options.data() + 1, options.data()};
      }
    }
    // Two directions at most: no entry moves once taken.
    known.reserve(2);
    const auto& [towards, options] =
        known.emplace_back(direction, escapes->waysOf(at, direction));
    return {options.data(), options.data() + 1};
  }

  /// The distance along a line to each of its points.
  [[nodiscard]] static std::vector<double>
  distancesAlong(const std::vector<Point>& line) {
    std::vector<double> at{0.0};
    for (std::size_t k = 1; k < line.size(); ++k) {
      at.push_back(at.back() + distance(line[k], line[k - 1]));
    }
    return at;
  }

  /// Slots laid from the start of the line, each at the first place past
  /// the one before where it fits. When the line runs against the edge as
  /// given, `forward` is the edge as given, whose points `ways` counts.
  [[nodiscard]] Placement place(const std::vector<Point>& line,
                                Examinations& memo, WaysAt& ways,
                                const std::vector<Point>* forward = nullptr,
                                Examinations* forwardMemo = nullptr) {
    const bool reversed = forward != nullptr;
    Placement result;
    std::vector<const Examined*> placed;
    EscapeWays::TakenList mine(escapes.has_value() ? escapes->near() : 1.0);
    double last = -1e9;
    const auto distanceAt = distancesAlong(line);
    for (std::size_t here = 0; here < line.size(); ++here) {
      if (distanceAt[here] - last < along - 1e-9) {
        continue;
      }
      const auto& found =
          reversed ? examinedBack(line, memo, here, *forward, *forwardMemo)
                   : examined(line, memo, here);
      if (found.kind == Examined::Kind::Skip) {
        continue;
      }
      auto kind = found.kind;
      std::string why;
      if (kind == Examined::Kind::Fits &&
          std::ranges::any_of(placed, [&](const Examined* other) {
            return overlap(found, *other);
          })) {
        kind = Examined::Kind::Refused;
        why = "another slot";
      }
      Slot slot = found.slot;
      if (kind == Examined::Kind::Fits && escapes.has_value()) {
        const auto options =
            waysAt(ways, reversed ? line.size() - 1 - here : here, slot.at,
                   slot.direction);
        auto chosen = escapes->choose(slot, options, {&taken, &mine});
        if (chosen.has_value()) {
          slot.ways = std::move(*chosen);
          slot.options = {*options[0], *options[1]};
          escapes->take(slot, mine);
        } else {
          kind = Examined::Kind::Refused;
          why = "no way out";
        }
      }
      if (kind == Examined::Kind::Refused) {
        if (explain) {
          result.refused.push_back({line[here],
                                    why.empty() ? found.why : std::move(why),
                                    found.slot.corners});
        }
        continue;
      }
      result.placed.push_back(std::move(slot));
      placed.push_back(&found);
      last = distanceAt[here];
    }
    result.length = distanceAt.back();
    return result;
  }

  /// Slots spread evenly: on every run of points where a
  /// slot fits, one after the other on one straight piece, as many slots as
  /// laid one after the other there, with equal gaps between them and to
  /// the two ends of the room they cover. Nothing when the line gets fewer
  /// than `wanted` slots this way.
  [[nodiscard]] std::optional<Placement>
  placeEvenly(const std::vector<Point>& line, const std::size_t wanted,
              Examinations& memo, WaysAt& ways) {
    Placement result;
    const auto distanceAt = distancesAlong(line);
    result.length = distanceAt.back();
    std::vector<const Examined*> found;
    found.reserve(line.size());
    for (std::size_t here = 0; here < line.size(); ++here) {
      found.push_back(&examined(line, memo, here));
      if (explain && found.back()->kind == Examined::Kind::Refused) {
        result.refused.push_back(
            {line[here], found.back()->why, found.back()->slot.corners});
      }
    }
    // The runs: consecutive points where a slot fits, with one direction.
    std::vector<std::pair<std::size_t, std::size_t>> runs;
    for (std::size_t here = 0; here < line.size(); ++here) {
      if (found[here]->kind != Examined::Kind::Fits) {
        continue;
      }
      const auto& direction = found[here]->slot.direction;
      if (!runs.empty() && runs.back().second + 1 == here) {
        const auto& before = found[runs.back().second]->slot.direction;
        if (before.x == direction.x && before.y == direction.y) {
          runs.back().second = here;
          continue;
        }
      }
      runs.emplace_back(here, here);
    }
    const auto near = escapes.has_value() ? escapes->near() : 1.0;
    // The slots of one try at a run: their points and the ways out they
    // take.
    struct Try {
      std::vector<std::size_t> at;
      EscapeWays::TakenList taken;
      std::map<std::size_t, std::vector<EscapeWay>> ways;
    };
    // The points of the slots of this line's earlier runs, and the ways out
    // they take.
    std::vector<std::size_t> placed;
    EscapeWays::TakenList mine(near);
    // Whether the slot at a point shares no cell with the slots placed
    // before this run or with the try's, and gets away from them; it joins
    // the try.
    const auto free = [&](const std::size_t point, Try& tried) {
      const auto& slotHere = *found[point];
      const auto meets = [&](const std::size_t other) {
        return overlap(slotHere, *found[other]);
      };
      if (std::ranges::any_of(placed, meets) ||
          std::ranges::any_of(tried.at, meets)) {
        return false;
      }
      if (escapes.has_value()) {
        const auto& slot = slotHere.slot;
        const auto options = waysAt(ways, point, slot.at, slot.direction);
        auto chosen =
            escapes->choose(slot, options, {&taken, &mine, &tried.taken});
        if (!chosen.has_value()) {
          return false;
        }
        Slot placedSlot = slot;
        placedSlot.ways = std::move(*chosen);
        escapes->take(placedSlot, tried.taken);
        tried.ways[point] = std::move(placedSlot.ways);
      }
      tried.at.push_back(point);
      return true;
    };
    // The slots of a run laid one after the other from its start.
    const auto packed = [&](const std::size_t from, const std::size_t to) {
      Try tried{.at = {}, .taken = EscapeWays::TakenList(near), .ways = {}};
      double last = -1e9;
      for (auto point = from; point <= to; ++point) {
        if (distanceAt[point] - last < along - 1e-9) {
          continue;
        }
        if (free(point, tried)) {
          last = distanceAt[point];
        }
      }
      return tried;
    };
    // The points nearest to n centres a pitch apart, centred on the run, or
    // nothing when they leave the run or two slots share a cell.
    const auto spread = [&](const std::size_t from, const std::size_t to,
                            const std::size_t n,
                            const double pitch) -> std::optional<Try> {
      const auto s = distanceAt[from];
      const auto e = distanceAt[to];
      const auto first =
          ((s + e) / 2.0) - (pitch * static_cast<double>(n - 1) / 2.0);
      if (first < s - 1e-9 ||
          first + (pitch * static_cast<double>(n - 1)) > e + 1e-9) {
        return std::nullopt;
      }
      Try tried{.at = {}, .taken = EscapeWays::TakenList(near), .ways = {}};
      auto point = from;
      for (std::size_t slot = 0; slot < n; ++slot) {
        const auto target = first + (pitch * static_cast<double>(slot));
        while (point < to && std::abs(distanceAt[point + 1] - target) <=
                                 std::abs(distanceAt[point] - target)) {
          ++point;
        }
        if ((!tried.at.empty() && tried.at.back() == point) ||
            !free(point, tried)) {
          return std::nullopt;
        }
      }
      return tried;
    };
    for (const auto& [from, to] : runs) {
      // As many as laid one after the other, spread with equal gaps between
      // them and to the ends of the room they cover: the room is the
      // centres' span and a clearance. When that pitch lets two slots share
      // a cell, the pitch grows by half a cell at a time and the gaps at
      // the ends shrink alike.
      auto tried = packed(from, to);
      const auto n = tried.at.size();
      if (n > 0) {
        const auto room = (distanceAt[to] - distanceAt[from]) + along;
        const auto gap = (room - (static_cast<double>(n) * along)) /
                         static_cast<double>(n + 1);
        for (auto pitch = gap + along;; pitch += 0.5) {
          if (auto even = spread(from, to, n, pitch); even.has_value()) {
            tried = std::move(*even);
            break;
          }
          if (n == 1 || pitch * static_cast<double>(n - 1) >
                            distanceAt[to] - distanceAt[from]) {
            break;
          }
        }
      }
      for (const auto point : tried.at) {
        result.placed.push_back(found[point]->slot);
        if (escapes.has_value()) {
          result.placed.back().ways = std::move(tried.ways[point]);
          const auto options = waysAt(ways, point, found[point]->slot.at,
                                      found[point]->slot.direction);
          result.placed.back().options = {*options[0], *options[1]};
        }
        placed.push_back(point);
      }
      mine.append(tried.taken);
    }
    if (result.placed.size() < wanted) {
      return std::nullopt;
    }
    return result;
  }

  /// Whether a wire may cross the edge at a cell on the line across it,
  /// as the router's crossing rule judges every cell the wire enters there:
  /// the line through the cell, the rule's reach and one more to either
  /// side, on the heading across a step of the edge.
  [[nodiscard]] bool crossable(const double cx, const double cy,
                               const Point& step) const {
    const auto dx = signOf(-step.y);
    const auto dy = signOf(step.x);
    const auto across = headingOfStep(dx, dy);
    if (!across.has_value()) {
      return true;
    }
    const auto& allowed = ruleAllows[*across];
    const auto x0 = static_cast<std::int64_t>(std::lround(cx));
    const auto y0 = static_cast<std::int64_t>(std::lround(cy));
    for (int t = -(ruleReach + 1); t <= ruleReach + 1; ++t) {
      const auto x = x0 + (t * dx);
      const auto y = y0 + (t * dy);
      if (x < 0 || y < 0 || !grid.inside(x, y) ||
          allowed[rule.maskAt(static_cast<std::uint32_t>(x),
                              static_cast<std::uint32_t>(y))] == 0) {
        return false;
      }
    }
    return true;
  }

  /// The router's crossing rule, rebuilt from the feedline strokes of the
  /// graph as the feedline routing builds it: every edge between couplers,
  /// the terminal edges left out, with the
  /// crossing reach of the router, one less than the graph's. A cell's
  /// heading is the step into it, the first cell's the step out of it, so
  /// that a cell is on a straight run when it leaves as it came.
  void buildRule(const std::vector<Stroke>& strokes) {
    std::set<std::string> terminal;
    for (const auto& wire : graph.wires) {
      if (wire.terminal) {
        terminal.insert(wire.name);
      }
    }
    std::vector<routing::Path> edges;
    for (const auto& stroke : strokes) {
      if (stroke.kind != "feedline" || terminal.contains(stroke.name)) {
        continue;
      }
      routing::Path way;
      for (const auto& point : stroke.points) {
        const auto x = static_cast<std::uint32_t>(std::lround(point.x));
        const auto y = static_cast<std::uint32_t>(std::lround(point.y));
        if (!way.empty() && way.back().x == x && way.back().y == y) {
          continue;
        }
        routing::PathPoint cell{.x = x, .y = y};
        if (!way.empty()) {
          cell.heading =
              headingOfStep(signOf(static_cast<double>(x) - way.back().x),
                            signOf(static_cast<double>(y) - way.back().y))
                  .value_or(way.back().heading);
        }
        way.push_back(cell);
      }
      if (way.size() >= 2) {
        way.front().heading = way[1].heading;
      }
      edges.push_back(std::move(way));
    }
    ruleReach = static_cast<int>(std::lround(graph.crossingReach)) - 1;
    rule.build(static_cast<std::uint32_t>(grid.width),
               static_cast<std::uint32_t>(grid.height), edges, {}, ruleReach,
               slotPinCells());
    // `CrossingConstraints::allowed` per heading and mask: every heading of
    // a straight run in the mask at a right angle to the heading.
    for (routing::Heading heading = 0; heading < routing::NUM_HEADINGS;
         ++heading) {
      for (unsigned mask = 0; mask < 256; ++mask) {
        bool ok = true;
        for (routing::Heading wire = 0; wire < routing::NUM_HEADINGS; ++wire) {
          if ((mask & (1U << wire)) != 0 &&
              !routing::isOrthogonal(wire, heading)) {
            ok = false;
          }
        }
        ruleAllows[heading][mask] = ok ? 1 : 0;
      }
    }
  }

  [[nodiscard]] std::vector<EdgeSlots> run(const std::vector<Stroke>& strokes) {
    if (slotCrossingRule()) {
      buildRule(strokes);
    }
    std::map<std::string, std::int64_t> stretchCapacity;
    for (const auto& edge : graph.edges) {
      if (edge.stretch) {
        stretchCapacity[edge.feedline] += edge.capacity;
      }
    }
    std::vector<EdgeSlots> edges;
    for (const auto& stroke : strokes) {
      if (stroke.kind != "feedline") {
        continue;
      }
      const auto count = stroke.points.size();
      Examinations forwardFound(count);
      Examinations backwardFound(count);
      WaysAt ways(count);
      auto forward = place(stroke.points, forwardFound, ways);
      const std::vector<Point> reversed(stroke.points.rbegin(),
                                        stroke.points.rend());
      auto backward =
          place(reversed, backwardFound, ways, &stroke.points, &forwardFound);
      auto& greedy =
          backward.placed.size() > forward.placed.size() ? backward : forward;
      std::optional<Placement> even;
      if (evenSlots()) {
        even = placeEvenly(stroke.points, greedy.placed.size(), forwardFound,
                           ways);
      }
      auto& best = even.has_value() ? *even : greedy;
      if (escapes.has_value()) {
        for (const auto& slot : best.placed) {
          escapes->take(slot, taken);
        }
      }
      const auto found = stretchCapacity.find(stroke.name);
      edges.push_back(
          {.name = stroke.name,
           .length = best.length,
           .slots = std::move(best.placed),
           .stretches = found == stretchCapacity.end() ? 0 : found->second,
           .refused = std::move(best.refused),
           .spread = even.has_value()});
    }
    return edges;
  }

  /// The footprints of the slots by step and parity (`footprintOf`), or
  /// nothing for a step whose samples come too near a half.
  mutable std::map<std::tuple<double, double, std::int64_t, std::int64_t>,
                   std::optional<Footprint>>
      footprints_;
  /// The footprints of slots at points of their own.
  mutable std::deque<Footprint> ownFootprints_;
};

/// The limits the ways out of the crossing slots set (`slotEscape`): for
/// every two slots with an end of the one near an end of the other, and no
/// pair of ways out of those ends a clearance apart, a group of the two that
/// takes one wire. An end with no way out limits nothing; the placement
/// gives every slot a way out at each end, and the ways out of its ends
/// (`Edge::options`).
[[nodiscard]] std::vector<Group> escapeLimits(const SlotGraph& graph,
                                              const Grid& grid,
                                              const std::vector<Edge>& edges,
                                              json* show = nullptr) {
  const EscapeWays escapes(graph, grid);
  struct End {
    std::size_t slot;
    std::size_t side;
    Point at;
    const std::vector<EscapeWay>* ways;
  };
  std::vector<End> ends;
  for (std::size_t k = 0; k < edges.size(); ++k) {
    const auto& edge = edges[k];
    if (!edge.stretch || edge.corners.empty()) {
      continue;
    }
    for (std::size_t side = 0; side < 2; ++side) {
      ends.push_back({.slot = k,
                      .side = side,
                      .at = escapes.endOf(edge.at, edge.direction, side),
                      .ways = &edge.options.at(side)});
    }
  }
  std::set<std::pair<std::size_t, std::size_t>> limited;
  std::vector<Group> groups;
  const auto near = escapes.near();
  for (std::size_t i = 0; i < ends.size(); ++i) {
    for (std::size_t j = i + 1; j < ends.size(); ++j) {
      const auto& a = ends[i];
      const auto& b = ends[j];
      // Two ends further apart along an axis than `near` lie further apart.
      if (a.slot == b.slot || a.ways->empty() || b.ways->empty() ||
          std::abs(a.at.x - b.at.x) > near ||
          std::abs(a.at.y - b.at.y) > near || distance(a.at, b.at) > near) {
        continue;
      }
      const auto pair = std::minmax(a.slot, b.slot);
      if (limited.contains(pair)) {
        continue;
      }
      // As long as one way out of the one end and one of the other keep a
      // clearance apart, the two wires get away together.
      const bool together = std::ranges::any_of(*a.ways, [&](const auto& wa) {
        return std::ranges::any_of(
            *b.ways, [&](const auto& wb) { return escapes.apart(wa, wb); });
      });
      if (together) {
        continue;
      }
      limited.insert(pair);
      groups.push_back({.slots = {pair.first, pair.second},
                        .capacity = 1,
                        .name = std::format("X {} / {}", edges[pair.first].name,
                                            edges[pair.second].name),
                        .line = {{{rounded(a.at.x), rounded(a.at.y)},
                                  {rounded(b.at.x), rounded(b.at.y)}}},
                        .feedline = edges[pair.first].feedline,
                        .enders = {}});
    }
  }
  // For the page: every end of a slot, the way out its placement chose,
  // and whether the slot is limited.
  if (show != nullptr) {
    std::set<std::size_t> held;
    for (const auto& [first, second] : limited) {
      held.insert(first);
      held.insert(second);
    }
    *show = json::array();
    for (const auto& end : ends) {
      const auto& chosen = edges[end.slot].ways;
      json cells = nullptr;
      json line = nullptr;
      if (end.side < chosen.size()) {
        cells = json::array();
        EscapeWays::forEachCell(
            chosen[end.side], [&](const std::int64_t x, const std::int64_t y) {
              cells.push_back(json::array({x, y}));
            });
        line = json::array();
        for (const auto& point : EscapeWays::curveOf(chosen[end.side])) {
          line.push_back(realJson(point));
        }
      }
      show->push_back({{"slot", edges[end.slot].name},
                       {"at", json::array({end.at.x, end.at.y})},
                       {"ways", end.ways->size()},
                       {"way", std::move(cells)},
                       {"line", std::move(line)},
                       {"limited", held.contains(end.slot)}});
    }
  }
  return groups;
}

/// The bottlenecks between two places of one feedline edge: a free line
/// from the edge's surface back to its surface, past which the edge holds
/// more crossing slots than the line takes wires. A slot is past the line
/// when one of its two ends lies in the room the line and the edge
/// enclose. Every wire that crosses at one of those slots passes the line
/// on that side, so the slots share what the line takes.
[[nodiscard]] std::vector<Group>
selfBottlenecks(const SlotGraph& graph, const Grid& grid,
                const std::vector<Stroke>& strokes,
                const std::vector<Edge>& edges) {
  const auto clear = graph.clearance;
  const auto margin = capacityMargin();
  const auto r = graph.inflation;
  const auto reach = graph.crossingReach + slotReachMargin() + 2;
  std::vector<Group> groups;
  std::map<std::vector<std::size_t>, std::size_t> index;
  for (const auto& stroke : strokes) {
    if (stroke.kind != "feedline") {
      continue;
    }
    const auto& pts = stroke.points;
    const auto n = pts.size();
    if (n < 4) {
      continue;
    }
    std::vector<std::size_t> mine;
    for (std::size_t k = 0; k < edges.size(); ++k) {
      if (edges[k].stretch && edges[k].feedline == stroke.name) {
        mine.push_back(k);
      }
    }
    if (mine.size() < 2) {
      continue;
    }
    // Per slot of `mine`: the point of the edge nearest it, and its exits.
    std::vector<std::size_t> at(edges.size());
    std::vector<std::array<Point, 2>> exits(edges.size());
    for (const auto k : mine) {
      const auto& c0 = edges[k].at;
      // A slot sits on a point of its edge: the first point there is the
      // nearest one.
      std::size_t closest = 0;
      const auto own = std::ranges::find_if(pts, [&](const Point& point) {
        return point.x == c0.x && point.y == c0.y;
      });
      if (own != pts.end()) {
        closest = static_cast<std::size_t>(own - pts.begin());
      } else {
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t p = 0; p < n; ++p) {
          const auto apart = distance(pts[p], c0);
          if (apart < best) {
            best = apart;
            closest = p;
          }
        }
      }
      at[k] = closest;
      auto direction = minus(pts[std::min(n - 1, closest + 1)],
                             pts[closest == 0 ? 0 : closest - 1]);
      const auto norm = std::max(1e-9, std::hypot(direction.x, direction.y));
      direction = {direction.x / norm, direction.y / norm};
      const Point normal{-direction.y, direction.x};
      exits[k] = {Point{c0.x + (reach * normal.x), c0.y + (reach * normal.y)},
                  Point{c0.x - (reach * normal.x), c0.y - (reach * normal.y)}};
    }
    std::vector<double> along{0.0};
    for (std::size_t p = 1; p < n; ++p) {
      along.push_back(along.back() + distance(pts[p], pts[p - 1]));
    }
    // Whether a line cuts a slot's way across the edge between the
    // rectangle and an exit, where a wire runs once it is across: not
    // inside the rectangle, which is the crossing itself (33q case 02 of
    // the review: f4 0..53 cuts the first slot of f4 inside its rectangle).
    const auto cutsPastRectangle = [&](const std::array<Point, 2>& ends,
                                       const Point& from, const Point& to) {
      const Point mid{(ends[0].x + ends[1].x) / 2.0,
                      (ends[0].y + ends[1].y) / 2.0};
      return std::ranges::any_of(ends, [&](const Point& end) {
        const Point rim{end.x + ((mid.x - end.x) * (2.0 / reach)),
                        end.y + ((mid.y - end.y) * (2.0 / reach))};
        return crossesLine(rim, end, from, to);
      });
    };
    // The points of the slots along the edge, in order: the slots between
    // two places are counted without a pass over the slots.
    std::vector<std::size_t> slotPoints;
    slotPoints.reserve(mine.size());
    for (const auto k : mine) {
      slotPoints.push_back(at[k]);
    }
    std::ranges::sort(slotPoints);
    std::vector<Point> polygon;
    for (std::size_t i = 0; i < n; ++i) {
      for (auto j = i + 1; j < n; ++j) {
        // A pair whose places lie far apart along an axis, or near along
        // the edge, fails the test below: the distance is no shorter than
        // the gap along an axis and at least one cell counts.
        if (along[j] - along[i] < 1.5 ||
            std::abs(pts[j].x - pts[i].x) - (2 * r) >= 6 * clear ||
            std::abs(pts[j].y - pts[i].y) - (2 * r) >= 6 * clear) {
          continue;
        }
        const auto d = distance(pts[j], pts[i]);
        if (!(along[j] - along[i] >= 1.5 * std::max(d, 1.0) &&
              d - (2 * r) < 6 * clear)) {
          continue;
        }
        // Only the slots between the two places can lie past the line.
        const auto between =
            std::distance(std::ranges::upper_bound(slotPoints, i),
                          std::ranges::lower_bound(slotPoints, j));
        if (between < 1) {
          continue;
        }
        const auto gap = (d - (2 * r)) * headingShare(pts[i], pts[j]);
        const auto capacity = wiresIn(gap, clear, margin);
        if (between <= capacity) {
          continue;
        }
        // The line between the two surfaces, facing each other, must run
        // free. The edge's own wall is no obstacle: the line's ends lie on
        // its surface, and rounding puts the first and last samples into
        // it.
        const auto norm = std::max(1e-9, d);
        const Point u{(pts[j].x - pts[i].x) / norm,
                      (pts[j].y - pts[i].y) / norm};
        const Point a{pts[i].x + (r * u.x), pts[i].y + (r * u.y)};
        const Point b{pts[j].x - (r * u.x), pts[j].y - (r * u.y)};
        bool free = true;
        const auto span = std::hypot(b.x - a.x, b.y - a.y);
        const auto steps = static_cast<std::int64_t>(span > 2 ? span : 2.0);
        const auto samples = unitSteps(static_cast<std::size_t>(steps + 1));
        for (std::size_t s = 1; s + 1 < samples.size(); ++s) {
          const auto t = samples[s];
          const Point q{a.x + (t * (b.x - a.x)), a.y + (t * (b.y - a.y))};
          const auto x = rounded(q.x);
          const auto y = rounded(q.y);
          if (!grid.inside(x, y)) {
            free = false;
            break;
          }
          const auto value = grid.at(x, y);
          const bool nearEnd =
              std::min(t, 1 - t) * static_cast<double>(steps) <= 4.0;
          if (value >= 1 && value <= 5) {
            bool ownWall = false;
            if (value == FEEDLINE && nearEnd) {
              ownWall = std::ranges::any_of(pts, [&](const Point& point) {
                return distance(point, q) <= r + 1.5;
              });
            }
            if (!ownWall) {
              free = false;
              break;
            }
          }
        }
        if (!free) {
          continue;
        }
        // A slot is past the line when its end on one side of the edge lies
        // in the room the line and the edge enclose, or when the line cuts
        // the slot's way across the edge from exit to exit: a wire crossing
        // there passes the line once (45q case 06 of the review: the exit
        // of the slot of f46 below its bend lies 1.5 cells past the line
        // from f46 to the end of f46, 14 cells from surface to surface). The
        // slot's rectangle has to lie wholly between the line's two ends
        // along the edge: a slot at an end of the line has its way beside
        // the line's end (33q case 02: the slot at the end of f4 0..85).
        const auto apart = distance(pts[j], pts[i]);
        const auto unit = std::max(1e-9, apart);
        const Point u0{(pts[j].x - pts[i].x) / unit,
                       (pts[j].y - pts[i].y) / unit};
        polygon.clear();
        polygon.push_back({pts[i].x + (r * u0.x), pts[i].y + (r * u0.y)});
        polygon.insert(polygon.end(),
                       pts.begin() + static_cast<std::ptrdiff_t>(i),
                       pts.begin() + static_cast<std::ptrdiff_t>(j) + 1);
        polygon.push_back({pts[j].x - (r * u0.x), pts[j].y - (r * u0.y)});
        std::vector<std::size_t> inside;
        const Point lineFrom = polygon.front();
        const Point lineTo = polygon.back();
        for (const auto k : mine) {
          if (i < at[k] && at[k] < j &&
              (std::ranges::any_of(
                   exits[k],
                   [&](const Point& q) { return inPolygon(q, polygon); }) ||
               (along[at[k]] - along[i] >= clear / 2.0 &&
                along[j] - along[at[k]] >= clear / 2.0 &&
                cutsPastRectangle(exits[k], lineFrom, lineTo)))) {
            inside.push_back(k);
          }
        }
        if (std::cmp_less_equal(inside.size(), capacity) || inside.empty()) {
          continue;
        }
        std::ranges::sort(inside);
        const auto known = index.find(inside);
        if (known != index.end() &&
            capacity >= groups[known->second].capacity) {
          continue;
        }
        Group group{.slots = inside,
                    .capacity = capacity,
                    .name = std::format("{} {}..{}", stroke.name, i, j),
                    .line = {{{rounded(a.x), rounded(a.y)},
                              {rounded(b.x), rounded(b.y)}}},
                    .feedline = stroke.name,
                    .enders = {}};
        if (known != index.end()) {
          groups[known->second] = std::move(group);
        } else {
          index.emplace(inside, groups.size());
          groups.push_back(std::move(group));
        }
      }
    }
  }
  return groups;
}

/// The fills of `slotStubBottlenecks` from the two sides of a line, over the
/// free cells of the chip as bits: a row of the chip is a run of 64-bit
/// words, bit b of word w for cell 64 w + b. A word of free cells (a chamber
/// or a cut line) is read from the raster when a fill first asks for it.
class SideFill {
public:
  /// How much a fill took: the number of cells, and whether that is the
  /// whole piece.
  struct Filled {
    std::size_t cells = 0;
    bool whole = false;
  };

  SideFill(const Grid& grid, RasterBits& bits)
      : grid_(grid), bits_(bits), words_(bits.words()),
        line_(words_ * static_cast<std::size_t>(grid.height), 0),
        seen_{std::vector<std::uint64_t>(line_.size(), 0),
              std::vector<std::uint64_t>(line_.size(), 0)} {}

  /// Begin a new line: forget the last line and its fills.
  void clear() {
    for (const auto word : touched_) {
      line_[word] = 0;
      seen_[0][word] = 0;
      seen_[1][word] = 0;
    }
    touched_.clear();
  }

  /// Make a cell part of the line, which no fill crosses.
  void block(const std::int64_t x, const std::int64_t y) {
    const auto word = wordOf(x, y);
    line_[word] |= bitOf(x);
    touched_.push_back(word);
  }

  /// Whether a cell is free and not on the line.
  [[nodiscard]] bool open(const std::int64_t x, const std::int64_t y) {
    const auto word = wordOf(x, y);
    return ((bits_.at(word).open & ~line_[word]) & bitOf(x)) != 0;
  }

  /// Whether the fill of side `which` took a cell.
  [[nodiscard]] bool filled(const std::size_t which, const std::int64_t x,
                            const std::int64_t y) const {
    return (seen_[which][wordOf(x, y)] & bitOf(x)) != 0;
  }

  /// The free cells 4-connected to (x0, y0) that are not on the line, taken
  /// by side `which` row by row. The fill stops once it holds `most` cells,
  /// or when side 1 meets a cell of side 0: the two sides are one piece then.
  [[nodiscard]] Filled fill(const std::int64_t x0, const std::int64_t y0,
                            const std::size_t which, const std::size_t most) {
    auto& self = seen_[which];
    const auto* other = which == 1 ? &seen_[0] : nullptr;
    Filled out;
    pending_.assign(1, {x0, y0});
    while (!pending_.empty()) {
      const auto [x, y] = pending_.back();
      pending_.pop_back();
      // The cells a fill may take in word w of row y: free, off the line,
      // not taken yet (the other side's cells among them).
      const auto takeable = [&](const std::int64_t r, const std::size_t w) {
        const auto word = (static_cast<std::size_t>(r) * words_) + w;
        return bits_.at(word).open & ~line_[word] & ~self[word];
      };
      const auto w0 = static_cast<std::size_t>(x / 64);
      const auto b0 = static_cast<unsigned>(x % 64);
      if (((takeable(y, w0) >> b0) & 1U) == 0) {
        continue;
      }
      // The run of takeable cells through x, word by word.
      auto right = x;
      for (std::size_t w = w0, b = b0;; ++w, b = 0) {
        const auto ones =
            static_cast<std::size_t>(std::countr_one(takeable(y, w) >> b));
        right = static_cast<std::int64_t>((w * 64) + b + ones) - 1;
        if (b + ones < 64 || w + 1 >= words_) {
          break;
        }
      }
      auto left = x;
      for (std::size_t w = w0, b = b0;; --w, b = 63) {
        const auto ones = static_cast<std::size_t>(
            std::countl_one(takeable(y, w) << (63U - b)));
        left = static_cast<std::int64_t>((w * 64) + b + 1 - ones);
        if (ones < b + 1 || w == 0) {
          break;
        }
      }
      if (other != nullptr && any(*other, y, left, right)) {
        return out;
      }
      set(self, y, left, right);
      out.cells += static_cast<std::size_t>(right - left + 1);
      if (out.cells >= most) {
        return out;
      }
      for (const auto r : {y - 1, y + 1}) {
        if (r < 0 || r >= grid_.height) {
          continue;
        }
        bool before = false;
        for (auto w = static_cast<std::size_t>(left / 64);
             w <= static_cast<std::size_t>(right / 64); ++w) {
          const auto mask = rangeMask(w, left, right);
          const auto cells = takeable(r, w) & mask;
          if (other != nullptr &&
              ((*other)[(static_cast<std::size_t>(r) * words_) + w] & mask) !=
                  0) {
            return out;
          }
          // The first cell of every run of takeable cells.
          auto starts = cells & ~((cells << 1U) | (before ? 1U : 0U));
          before = (cells >> 63U) != 0;
          while (starts != 0) {
            const auto bit = std::countr_zero(starts);
            pending_.emplace_back((static_cast<std::int64_t>(w) * 64) + bit, r);
            starts &= starts - 1;
          }
        }
      }
    }
    out.whole = true;
    return out;
  }

private:
  [[nodiscard]] std::size_t wordOf(const std::int64_t x,
                                   const std::int64_t y) const {
    return (static_cast<std::size_t>(y) * words_) +
           static_cast<std::size_t>(x / 64);
  }
  [[nodiscard]] static std::uint64_t bitOf(const std::int64_t x) {
    return std::uint64_t{1} << static_cast<unsigned>(x % 64);
  }
  /// The bits of the cells of word w in [left, right].
  [[nodiscard]] static std::uint64_t rangeMask(const std::size_t w,
                                               const std::int64_t left,
                                               const std::int64_t right) {
    const auto first = static_cast<std::int64_t>(w) * 64;
    const auto from =
        static_cast<unsigned>(std::max<std::int64_t>(left - first, 0));
    const auto to =
        static_cast<unsigned>(std::min<std::int64_t>(right - first, 63));
    const auto upTo =
        to == 63 ? ~std::uint64_t{0} : (std::uint64_t{1} << (to + 1)) - 1;
    return upTo & ~((std::uint64_t{1} << from) - 1);
  }
  [[nodiscard]] bool any(const std::vector<std::uint64_t>& bits,
                         const std::int64_t y, const std::int64_t left,
                         const std::int64_t right) const {
    for (auto w = static_cast<std::size_t>(left / 64);
         w <= static_cast<std::size_t>(right / 64); ++w) {
      if ((bits[(static_cast<std::size_t>(y) * words_) + w] &
           rangeMask(w, left, right)) != 0) {
        return true;
      }
    }
    return false;
  }
  void set(std::vector<std::uint64_t>& bits, const std::int64_t y,
           const std::int64_t left, const std::int64_t right) {
    for (auto w = static_cast<std::size_t>(left / 64);
         w <= static_cast<std::size_t>(right / 64); ++w) {
      const auto word = (static_cast<std::size_t>(y) * words_) + w;
      bits[word] |= rangeMask(w, left, right);
      touched_.push_back(word);
    }
  }

  const Grid& grid_;
  RasterBits& bits_;
  std::size_t words_;
  std::vector<std::uint64_t> line_;
  std::array<std::vector<std::uint64_t>, 2> seen_;
  /// The words a line or a fill set bits in, to clear for the next line.
  std::vector<std::size_t> touched_;
  std::vector<Cell> pending_;
};

/// The bottlenecks between a crossing slot and the open end of a stub: a
/// free line from a corner of a slot's rectangle to the end of a stub's
/// wall that closes a room. A wire that starts in the room and a wire that
/// crosses at a slot whose exit lies in the room both pass the line.
[[nodiscard]] std::vector<Group>
slotStubBottlenecks(const SlotGraph& graph, const Grid& grid, RasterBits& bits,
                    const std::vector<Stroke>& strokes,
                    const std::vector<Edge>& edges,
                    const std::vector<Wire>& wires) {
  const auto clear = graph.clearance;
  const auto margin = capacityMargin();
  const auto longest = 4 * clear;
  // The open ends of the stubs' walls: stub cells with one stub cell
  // around them.
  std::vector<Point> tips;
  for (const auto cell : *grid.stubCells) {
    const auto x = static_cast<std::int64_t>(cell) % grid.width;
    const auto y = static_cast<std::int64_t>(cell) / grid.width;
    int around = 0;
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        if ((dx != 0 || dy != 0) && grid.inside(x + dx, y + dy) &&
            grid.at(x + dx, y + dy) == STUB) {
          ++around;
        }
      }
    }
    if (around == 1) {
      tips.push_back({static_cast<double>(x), static_cast<double>(y)});
    }
  }
  if (tips.empty()) {
    return {};
  }
  std::vector<std::size_t> slots;
  for (std::size_t k = 0; k < edges.size(); ++k) {
    if (edges[k].stretch && !edges[k].corners.empty()) {
      slots.push_back(k);
    }
  }
  const auto limit = static_cast<std::size_t>(4 * longest * longest);
  // The wire every stub belongs to, by the middle line of the stub.
  std::vector<std::pair<std::string, const std::vector<Point>*>> stubLines;
  for (const auto& stroke : strokes) {
    if (stroke.kind != "stub" || stroke.points.empty()) {
      continue;
    }
    if (auto owner = runName(stroke.name, false)) {
      stubLines.emplace_back(std::move(*owner), &stroke.points);
    }
  }
  const auto ownerOf = [&](const Point& q) -> std::optional<std::string> {
    auto best = 2 * clear;
    std::optional<std::string> name;
    for (const auto& [owner, points] : stubLines) {
      double closest = std::numeric_limits<double>::infinity();
      for (const auto& point : *points) {
        closest = std::min(closest, distance(point, q));
      }
      if (closest < best) {
        best = closest;
        name = owner;
      }
    }
    return name;
  };

  // The free cells on the closed side of the line a-b, or nothing: the
  // side of the line whose fill (`sides`) is the room.
  std::optional<SideFill> sides;
  const auto roomOf = [&](const Point& a,
                          const Point& b) -> std::optional<std::size_t> {
    if (!sides.has_value()) {
      sides.emplace(grid, bits);
    }
    sides->clear();
    const auto span = std::hypot(b.x - a.x, b.y - a.y);
    const auto count = std::max<std::int64_t>(
        2, static_cast<std::int64_t>(std::ceil(span)) * 2);
    const auto norm = std::max(1e-9, span);
    const Point u{(b.x - a.x) / norm, (b.y - a.y) / norm};
    // Carry the line two cells into the walls at its ends.
    for (const auto t : unitSteps(static_cast<std::size_t>(count + 8))) {
      const auto qx = (a.x - (2 * u.x)) + (t * ((b.x - a.x) + (4 * u.x)));
      const auto qy = (a.y - (2 * u.y)) + (t * ((b.y - a.y) + (4 * u.y)));
      const auto x = rounded(qx);
      const auto y = rounded(qy);
      for (const auto& [dx, dy] :
           std::array<std::pair<std::int64_t, std::int64_t>, 3>{
               {{0, 0}, {1, 0}, {0, 1}}}) {
        if (grid.inside(x + dx, y + dy)) {
          sides->block(x + dx, y + dy);
        }
      }
    }
    const Point mid{(a.x + b.x) / 2, (a.y + b.y) / 2};
    const Point normal{-u.y, u.x};
    std::array<bool, 2> closed{false, false};
    std::array<std::size_t, 2> size{0, 0};
    for (std::size_t which = 0; which < 2; ++which) {
      const double sign = which == 0 ? 3.0 : -3.0;
      const auto x0 = rounded(mid.x + (sign * normal.x));
      const auto y0 = rounded(mid.y + (sign * normal.y));
      if (!grid.inside(x0, y0) || !sides->open(x0, y0)) {
        continue;
      }
      // The second side is the room only when it closes smaller than a
      // first side that closed: past that size its fill can stop. A second
      // side that meets the first is the same piece, no smaller.
      const auto stopAt = which == 1 && closed[0] ? size[0] : limit + 1;
      const auto filled = sides->fill(x0, y0, which, stopAt);
      size[which] = filled.cells;
      closed[which] =
          filled.whole && filled.cells <= limit && filled.cells < stopAt;
    }
    std::optional<std::size_t> room;
    for (std::size_t which = 0; which < 2; ++which) {
      if (closed[which] && (!room.has_value() || size[which] < size[*room])) {
        room = which;
      }
    }
    return room;
  };

  std::vector<Group> groups;
  std::map<std::pair<std::vector<std::size_t>, std::vector<std::string>>,
           std::size_t>
      index;
  for (const auto k : slots) {
    for (const auto& a : edges[k].corners) {
      for (const auto& b : tips) {
        // A tip further away along an axis lies further away.
        if (std::abs(b.x - a.x) > longest || std::abs(b.y - a.y) > longest) {
          continue;
        }
        const auto length = distance(b, a);
        if (!(length <= longest)) {
          continue;
        }
        auto capacity = wiresIn(length * headingShare(a, b), clear, margin);
        // The line must run free between its ends.
        bool free = true;
        const auto steps = static_cast<std::int64_t>(length > 2 ? length : 2.0);
        const auto samples = unitSteps(static_cast<std::size_t>(steps + 1));
        for (std::size_t s = 1; s + 1 < samples.size(); ++s) {
          const auto t = samples[s];
          const auto x = rounded(a.x + (t * (b.x - a.x)));
          const auto y = rounded(a.y + (t * (b.y - a.y)));
          if (!grid.inside(x, y)) {
            free = false;
            break;
          }
          const auto value = grid.at(x, y);
          if (value >= 1 && value <= 5 && std::min(t, 1 - t) * length > 3.0) {
            free = false;
            break;
          }
        }
        if (!free) {
          continue;
        }
        const auto room = roomOf(a, b);
        if (!room.has_value()) {
          continue;
        }
        const auto inRoom = [&](const Point& q) {
          const auto x = rounded(q.x);
          const auto y = rounded(q.y);
          return grid.inside(x, y) && sides->filled(*room, x, y);
        };
        std::vector<std::size_t> inside;
        for (const auto other : slots) {
          if (inRoom(edges[other].from) || inRoom(edges[other].to)) {
            inside.push_back(other);
          }
        }
        std::vector<std::string> enders;
        for (const auto& wire : wires) {
          if (inRoom(wire.source.cell) != inRoom(wire.target.cell)) {
            enders.push_back(wire.name);
          }
        }
        std::ranges::sort(enders);
        // The stub's own wire always leaves the stub: a line to the stub of
        // a wire that starts in the room takes that wire.
        if (const auto owner = ownerOf(b);
            owner.has_value() &&
            std::ranges::find(enders, *owner) != enders.end()) {
          capacity = std::max<std::int64_t>(capacity, 1);
        }
        // A room with no slot is a stub's own opening, which takes the
        // stub's wire.
        if (inside.empty() || enders.empty() ||
            std::cmp_less_equal(inside.size() + enders.size(), capacity)) {
          continue;
        }
        auto key = std::pair{inside, enders};
        const auto known = index.find(key);
        if (known != index.end() &&
            capacity >= groups[known->second].capacity) {
          continue;
        }
        Group group{.slots = inside,
                    .capacity = capacity,
                    .name = std::format("{} to stub ({}, {})", edges[k].name,
                                        static_cast<std::int64_t>(b.x),
                                        static_cast<std::int64_t>(b.y)),
                    .line = {{{rounded(a.x), rounded(a.y)},
                              {static_cast<std::int64_t>(b.x),
                               static_cast<std::int64_t>(b.y)}}},
                    .feedline = edges[k].feedline,
                    .enders = enders};
        if (known != index.end()) {
          groups[known->second] = std::move(group);
        } else {
          index.emplace(std::move(key), groups.size());
          groups.push_back(std::move(group));
        }
      }
    }
  }
  return groups;
}

/// A wire to route: where it starts and ends, and what it must cross.
struct Demand {
  std::string name;
  std::vector<std::int64_t> from;
  std::vector<std::int64_t> to;
  std::vector<std::string> crosses;
  /// The mouth the wire passes and no other.
  std::optional<std::string> mouth;
  /// Per edge: whether the wire may use it at all, whatever its load.
  std::vector<bool> open;
  /// Per edge: the bit of the feedline edge a slot crosses, or 0.
  std::vector<std::uint64_t> crossBit;
  /// Per edge: whether it is a gate of the wire's own mouth.
  std::vector<bool> ownMouth;
  /// Per chamber: the moves out of it along edges the wire may use, as
  /// numbers of `Checker::around_` in its order, found when the search
  /// first leaves the chamber.
  mutable std::vector<std::vector<std::uint32_t>> openMoves;
  mutable std::vector<bool> openMovesKnown;
  /// Per move out of the chambers of `from`, in turn: the distance from
  /// the wire's start to the middle of the move's edge, found once.
  mutable std::vector<double> startApart;
  mutable bool startApartKnown = false;
};

/// A way through the graph, by edge, and its length.
struct Way {
  std::optional<std::vector<std::size_t>> path;
  double length = 0.0;
};

/// A state of the search: the edge crossed last, the chamber entered and
/// the crossings so far, packed so that their order is the order of the
/// three numbers in turn.
constexpr int EDGE_SHIFT = 43;
constexpr int CHAMBER_SHIFT = 22;
constexpr std::uint64_t FIELD = (1ULL << 21) - 1;

[[nodiscard]] std::uint64_t stateOf(const std::size_t edge,
                                    const std::int64_t chamber,
                                    const std::uint64_t mask) {
  return (static_cast<std::uint64_t>(edge) << EDGE_SHIFT) |
         (static_cast<std::uint64_t>(chamber) << CHAMBER_SHIFT) | mask;
}

/// The check on one graph.
class Checker {
public:
  /// The check on `graph`; `page`, when given, is the same graph as JSON,
  /// which the graph for the page is built from.
  Checker(const SlotGraph& graph, const json* page)
      : graph_(graph), page_(page), grid_(gridOf(graph)),
        clear_(graph.clearance), margin_(capacityMargin()) {}

  /// The verdict, the lost wires and the edges over, and the graph as the
  /// check saw it.
  std::pair<SlotCheckResult, json> run();

private:
  const SlotGraph& graph_;
  /// The graph as JSON when the graph for the page is wanted, or nothing.
  const json* page_;
  Grid grid_;
  double clear_;
  /// The safety factor on the clearance (`capacityMargin`).
  double margin_;
  std::vector<Stroke> strokes_;
  std::vector<Wire> wires_;
  std::vector<Edge> edges_;
  std::vector<Point> mid_;
  /// A move of the search: along an edge out of a chamber into another,
  /// and the number of that pair of edge and chamber entered.
  struct Move {
    std::size_t edge = 0;
    std::int64_t into = 0;
    std::size_t pair = 0;
  };
  /// Per chamber: the moves out of it.
  std::vector<std::vector<Move>> around_;
  /// How many pairs of edge and chamber entered there are, and how many
  /// crossing masks a state may carry: a state of the search is a pair and
  /// a mask, `pair * maskCount_ + mask`.
  std::size_t pairCount_ = 0;
  std::size_t maskCount_ = 1;
  /// Per pair: its edge and the chamber it enters.
  std::vector<std::size_t> edgeOfPair_;
  std::vector<std::int64_t> intoOfPair_;
  /// Per pair: the distance from the edge's middle to the middle of each
  /// move out of the chamber it enters, in the order of `around_`, found
  /// when the search first takes the pair (`stepsOf`).
  mutable std::vector<std::vector<double>> stepsOut_;
  mutable std::vector<bool> stepsKnown_;
  [[nodiscard]] const std::vector<double>& stepsOf(std::size_t pair) const {
    if (!stepsKnown_[pair]) {
      const auto into = intoOfPair_[pair];
      if (into >= 0 && std::cmp_less(into, around_.size())) {
        for (const auto& move : around_[static_cast<std::size_t>(into)]) {
          stepsOut_[pair].push_back(
              distance(mid_[move.edge], mid_[edgeOfPair_[pair]]));
        }
      }
      stepsKnown_[pair] = true;
    }
    return stepsOut_[pair];
  }
  /// What the search knows of a state: its distance, the step into it and
  /// the state before it, valid in the search of generation `seen`.
  struct Best {
    double d = 0.0;
    double step = 0.0;
    std::uint64_t from = 0;
    std::uint32_t seen = 0;
  };
  /// The search's states, reset by a new generation.
  mutable std::vector<Best> best_;
  mutable std::uint32_t generation_ = 0;
  /// Per edge: whether it is a crossing slot.
  std::vector<std::uint8_t> isSlot_;
  /// A state the search has queued: its distance, its code (`stateOf`), its
  /// number, its edge and the chamber entered. The heap orders by distance
  /// and then by code.
  struct Queued {
    double d;
    std::uint64_t code;
    std::size_t at;
    std::size_t edge;
    std::int64_t chamber;
    [[nodiscard]] bool operator>(const Queued& other) const {
      return d != other.d ? d > other.d : code > other.code;
    }
  };
  mutable std::vector<Queued> queue_;
  std::vector<std::set<std::string>> owners_;
  std::vector<std::int64_t> capacity_;
  /// Per edge, what it holds with none, one and two users at their own
  /// run (`limitFor`): the negotiation asks for it per wire and per round.
  std::vector<std::array<std::int64_t, 3>> limits_;
  /// Per mouth gate: the mouth it belongs to, or nothing.
  std::unordered_map<std::size_t, std::optional<std::string>> mouthOfGate_;
  /// Per edge: its entry in `mouthOfGate_`, or none (`indexEdges`).
  std::vector<const std::optional<std::string>*> gateMouth_;
  /// Per wire by name: the edges whose users name it (`indexEdges`).
  std::unordered_map<std::string, std::vector<std::size_t>> edgesOfUser_;
  /// Per wire: the mouth it passes and no other.
  std::unordered_map<std::string, std::string> mouthOfWire_;
  /// Per terminal edge: the one mouth it may pass.
  std::unordered_map<std::string, std::string> barred_;
  std::unordered_map<std::string, std::pair<Point, Point>> ends_;

  void readStrokesAndWires();
  void addSides();
  void addSlots(const std::vector<EdgeSlots>& slots);
  void findOwners();
  void markMouthTwins();
  void findMouths();
  void indexEdges();
  void openEdges(Demand& demand) const;
  /// What edge k holds with `own` of its users at their own run, from
  /// `limits_`.
  [[nodiscard]] std::int64_t limitOf(std::size_t k, std::int64_t own) const {
    return limits_[k][static_cast<std::size_t>(std::min<std::int64_t>(own, 2))];
  }
  [[nodiscard]] std::int64_t limitFor(std::size_t k, std::int64_t own) const;
  [[nodiscard]] Way shortest(const Demand& demand,
                             const std::vector<double>& weight) const;
};

void Checker::readStrokesAndWires() {
  for (const auto& stroke : graph_.strokes) {
    std::vector<Point> points;
    points.reserve(stroke.points.size());
    for (const auto& [x, y] : stroke.points) {
      points.push_back({x, y});
    }
    strokes_.push_back({.name = stroke.name,
                        .kind = stroke.kind,
                        .points = std::move(points)});
  }
  const auto endOf = [](const SlotGraph::End& end) {
    return WireEnd{.cell = {end.cell[0], end.cell[1]},
                   .chambers = end.chambers,
                   .label = end.label};
  };
  for (const auto& wire : graph_.wires) {
    Wire one{.name = wire.name,
             .source = endOf(wire.source),
             .target = endOf(wire.target),
             .crosses = wire.crosses,
             .terminal = wire.terminal,
             .resonator = wire.resonator};
    ends_[one.name] = {one.source.cell, one.target.cell};
    wires_.push_back(std::move(one));
  }
}

void Checker::addSides() {
  std::vector<std::pair<std::string, std::pair<Point, Point>>> gateLines;
  for (const auto& edge : graph_.edges) {
    if (!edge.stretch) {
      gateLines.emplace_back(edge.name,
                             std::pair{Point{edge.from[0], edge.from[1]},
                                       Point{edge.to[0], edge.to[1]}});
    }
  }
  std::set<std::string> resonatorNames;
  for (const auto& wire : graph_.wires) {
    if (wire.resonator) {
      resonatorNames.insert(wire.name);
    }
  }
  for (std::size_t index = 0; index < graph_.edges.size(); ++index) {
    const auto& edge = graph_.edges[index];
    if (edge.stretch) {
      continue;
    }
    Edge one;
    if (page_ != nullptr) {
      one.out = page_->at("edges").at(index);
    }
    one.name = edge.name;
    one.chambers = edge.chambers;
    one.capacity = edge.capacity;
    one.users = edge.users;
    one.from = {edge.from[0], edge.from[1]};
    one.to = {edge.to[0], edge.to[1]};
    one.ends = edge.ends;
    one.mouth = edge.mouth;
    one.length = edge.length;
    // A gate too narrow for any wire that the check by stretches opened to
    // the resonators at whose own runs it ends holds none of them when it is
    // narrower than `resonatorPassage` of the clearance.
    if (one.capacity >= 1 && one.length < clear_ && one.users.has_value() &&
        !one.users->empty() && one.length < resonatorPassage() * clear_ &&
        std::ranges::all_of(
            *one.users,
            [&](const std::string& name) {
              return resonatorNames.contains(name);
            })) {
      one.capacity = 0;
    }
    // A gate that holds its length over the clearance holds it over the
    // clearance times the margin; a gate the check by stretches gave another
    // capacity (a slot, a mouth, an own run's passage) keeps it.
    if (one.capacity == wiresIn(one.length, clear_, 1.0)) {
      // A gap from a feedline edge to a static obstacle counts the bonus
      // that the obstacle grants (`staticGapBonus`).
      const auto bonus = feedlineToStatic(one.ends) ? staticGapBonus() : 0.0;
      one.capacity =
          wiresIn((one.length * headingShare(one.from, one.to)) + bonus, clear_,
                  margin_);
    }
    one.sides = edge.sides;
    // The gates near this one: a sample lies at most a cell and a half off
    // the line.
    const auto minX = std::min(one.from.x, one.to.x) - 2;
    const auto maxX = std::max(one.from.x, one.to.x) + 2;
    const auto minY = std::min(one.from.y, one.to.y) - 2;
    const auto maxY = std::max(one.from.y, one.to.y) + 2;
    std::vector<std::pair<Point, Point>> others;
    for (const auto& [name, line] : gateLines) {
      const auto& [c, d] = line;
      if (name != one.name && std::max(c.x, d.x) >= minX &&
          std::min(c.x, d.x) <= maxX && std::max(c.y, d.y) >= minY &&
          std::min(c.y, d.y) <= maxY) {
        others.push_back(line);
      }
    }
    const auto found = lineSides(grid_, one, others);
    std::vector<std::int64_t> left;
    std::vector<std::int64_t> right;
    std::ranges::set_difference(found[0], found[1], std::back_inserter(left));
    std::ranges::set_difference(found[1], found[0], std::back_inserter(right));
    if (!left.empty() && !right.empty()) {
      // The gate joins a chamber on one side of its line to one on the other
      // side, never two on the same side.
      std::vector<std::int64_t> both;
      std::ranges::set_union(left, right, std::back_inserter(both));
      one.sides = std::array{left, right};
      one.chambers = both;
      if (page_ != nullptr) {
        one.out["sides"] = json::array({left, right});
        one.out["chambers"] = both;
      }
    }
    edges_.push_back(std::move(one));
  }
}

void Checker::addSlots(const std::vector<EdgeSlots>& slots) {
  const auto reach = graph_.crossingReach + slotReachMargin() + 2;
  // An edge of a chain with a reflector end may have one chamber on both
  // sides, as the check's own stretches say.
  std::set<std::string> reflected;
  std::map<std::string, std::vector<std::string>> crossers;
  for (const auto& edge : graph_.edges) {
    if (edge.stretch &&
        std::set(edge.chambers.begin(), edge.chambers.end()).size() == 1) {
      reflected.insert(edge.feedline);
    }
  }
  for (const auto& wire : wires_) {
    for (const auto& feedline : wire.crosses) {
      crossers[feedline].push_back(wire.name);
    }
  }
  // The stretches of an edge of an open chain are prescribed to no wire but
  // stay open to the wires that bridge the edge (17q at the chain steps:
  // wires 12 to 14 bridge f2 before the chain is whole). Their slots stay
  // open to the same wires.
  for (const auto& edge : graph_.edges) {
    if (!edge.stretch || !edge.users.has_value()) {
      continue;
    }
    auto& open = crossers[edge.feedline];
    for (const auto& user : *edge.users) {
      if (std::ranges::find(open, user) == open.end()) {
        open.push_back(user);
      }
    }
  }
  const auto along = clear_;
  for (const auto& edge : slots) {
    for (std::size_t i = 0; i < edge.slots.size(); ++i) {
      const auto& slot = edge.slots[i];
      const auto ux = slot.direction.x;
      const auto uy = slot.direction.y;
      const auto a = sideChamber(grid_, slot.at, -uy, ux, reach);
      const auto b = sideChamber(grid_, slot.at, uy, -ux, reach);
      if (!a.has_value() || !b.has_value() ||
          (a->first == b->first && !reflected.contains(edge.name))) {
        continue;
      }
      Edge one;
      one.stretch = true;
      one.name = std::format("□ {} {}", edge.name, i + 1);
      one.feedline = edge.name;
      one.chambers = {a->first, b->first};
      one.capacity = 1;
      const auto found = crossers.find(edge.name);
      one.users =
          found == crossers.end() ? std::vector<std::string>{} : found->second;
      one.from = {static_cast<double>(a->second[0]),
                  static_cast<double>(a->second[1])};
      one.to = {static_cast<double>(b->second[0]),
                static_cast<double>(b->second[1])};
      one.length = along;
      one.at = slot.at;
      one.direction = slot.direction;
      one.corners.assign(slot.corners.begin(), slot.corners.end());
      one.ways = slot.ways;
      one.options = slot.options;
      if (page_ == nullptr) {
        edges_.push_back(std::move(one));
        continue;
      }
      auto corners = json::array();
      for (const auto& corner : slot.corners) {
        corners.push_back(realJson(corner));
      }
      one.out = json{
          {"kind", "stretch"},
          {"name", one.name},
          {"feedline", one.feedline},
          {"chambers", one.chambers},
          {"capacity", 1},
          {"users", *one.users},
          {"line", json::array({json::array({a->second[0], a->second[1]}),
                                json::array({b->second[0], b->second[1]})})},
          {"points", json::array({pointJson(slot.at)})},
          {"length", along},
          {"terminal", false},
          {"corners", std::move(corners)}};
      edges_.push_back(std::move(one));
    }
  }
}

void Checker::markMouthTwins() {
  if (!mouthTwins()) {
    return;
  }
  // A gate on the line of a mouth between the same two chambers is that
  // mouth: it passes what the mouth bars otherwise.
  const auto near = [](const Point& a, const Point& b) {
    return distance(a, b) <= MOUTH_TWIN_CELLS;
  };
  std::vector<std::size_t> mouths;
  for (std::size_t k = 0; k < edges_.size(); ++k) {
    if (!edges_[k].stretch && edges_[k].mouth) {
      mouths.push_back(k);
    }
  }
  for (auto& edge : edges_) {
    if (edge.stretch || edge.mouth) {
      continue;
    }
    const std::set<std::int64_t> chambers(edge.chambers.begin(),
                                          edge.chambers.end());
    for (const auto m : mouths) {
      const auto& mouth = edges_[m];
      const bool sameLine =
          (near(edge.from, mouth.from) && near(edge.to, mouth.to)) ||
          (near(edge.from, mouth.to) && near(edge.to, mouth.from));
      if (sameLine &&
          chambers == std::set<std::int64_t>(mouth.chambers.begin(),
                                             mouth.chambers.end())) {
        edge.mouth = true;
        if (page_ != nullptr) {
          edge.out["mouth"] = true;
        }
        break;
      }
    }
  }
}

void Checker::findOwners() {
  // The middle line of every port run, by its wire: port to open end.
  std::unordered_map<std::string, std::vector<const std::vector<Point>*>> runs;
  for (const auto& stroke : strokes_) {
    if (stroke.kind == "stub" && stroke.points.size() >= 2) {
      if (auto owner = runName(stroke.name, false)) {
        runs[*owner].push_back(&stroke.points);
      }
    }
  }
  // Whether the wire leaves (or enters) one of its runs straight through
  // the gate: the run's middle line, carried on past its open end for two
  // clearances, meets the gate's line.
  const auto leavesStraight = [&](const std::string& wire, const Edge& edge) {
    const auto found = runs.find(wire);
    if (found == runs.end()) {
      return false;
    }
    const auto& [ax, ay] = edge.from;
    const auto& [bx, by] = edge.to;
    const auto side = [](const double x1, const double y1, const double x2,
                         const double y2, const double x3, const double y3) {
      return ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
    };
    for (const auto* points : found->second) {
      const auto& [px, py] = (*points)[points->size() - 2];
      const auto& [qx, qy] = points->back();
      const auto dx = qx - px;
      const auto dy = qy - py;
      const auto n = std::hypot(dx, dy);
      if (n == 0) {
        continue;
      }
      const auto rx = qx + (((dx / n) * 2) * clear_);
      const auto ry = qy + (((dy / n) * 2) * clear_);
      if (side(qx, qy, rx, ry, ax, ay) * side(qx, qy, rx, ry, bx, by) <= 0 &&
          side(ax, ay, bx, by, qx, qy) * side(ax, ay, bx, by, rx, ry) <= 0) {
        return true;
      }
    }
    return false;
  };
  for (const auto& edge : edges_) {
    std::set<std::string> found;
    if (!edge.stretch) {
      for (const auto& end : edge.ends) {
        if (auto owner = runName(end, true);
            owner.has_value() && leavesStraight(*owner, edge)) {
          found.insert(*owner);
        }
      }
    }
    owners_.push_back(std::move(found));
  }
}

std::int64_t Checker::limitFor(const std::size_t k,
                               const std::int64_t own) const {
  if (edges_[k].stretch || owners_[k].empty() || !ownRunBonus()) {
    return capacity_[k];
  }
  // A gate with an end on a wire's own port run holds that wire half a
  // clearance more: the wire keeps no clearance from its own run.
  const auto margins =
      ((2 * clear_) / 2) -
      ((static_cast<double>(std::min<std::int64_t>(own, 2)) * clear_) / 2);
  const auto room =
      (edges_[k].length * headingShare(edges_[k].from, edges_[k].to)) - margins;
  return std::max(
      capacity_[k],
      room >= 0
          ? static_cast<std::int64_t>(std::floor(room / (clear_ * margin_))) + 1
          : 0);
}

void Checker::findMouths() {
  // The mouths between the chains. A mouth lies between the ending edge of
  // one chain (its launcher at its target) and the starting edge of the
  // next (its launcher at its source), the next terminal edge by number
  // round the ring; it is named by the ending edge, or, where that chain
  // ends at a reflector, by the starting edge of the chain after.
  std::unordered_map<std::string, const Wire*> byName;
  std::vector<std::string> terminals;
  for (const auto& wire : wires_) {
    byName[wire.name] = &wire;
  }
  for (const auto& wire : wires_) {
    if (wire.terminal) {
      terminals.push_back(wire.name);
    }
  }
  std::ranges::stable_sort(terminals, {}, numberOf);
  std::set<std::string> ending;
  for (const auto& name : terminals) {
    if (byName.at(name)->target.label.starts_with("Chip.port")) {
      ending.insert(name);
    }
  }
  std::unordered_map<std::string, std::string> following;
  std::unordered_map<std::string, std::string> preceding;
  for (std::size_t i = 0; i < terminals.size(); ++i) {
    following[terminals[i]] = terminals[(i + 1) % terminals.size()];
  }
  for (std::size_t i = 0; i < terminals.size(); ++i) {
    preceding[terminals[(i + 1) % terminals.size()]] = terminals[i];
  }
  // Every mouth gate belongs to the mouth whose terminal runs it names; a
  // piece cut from a mouth line with no terminal run at its ends belongs to
  // the mouth of the gate before it.
  std::vector<std::pair<std::size_t, std::optional<std::string>>> gates;
  std::optional<std::string> last;
  for (std::size_t k = 0; k < edges_.size(); ++k) {
    if (!edges_[k].mouth) {
      continue;
    }
    std::vector<std::string> named;
    for (const auto& end : edges_[k].ends) {
      if (auto terminal = couplerEnd(end)) {
        named.push_back(std::move(*terminal));
      }
    }
    std::optional<std::string> key;
    for (const auto& name : named) {
      if (ending.contains(name)) {
        key = name;
        break;
      }
    }
    if (!key.has_value() && !named.empty()) {
      // A starting edge names the mouth of the ending edge before it,
      // unless the chain before ends at a reflector.
      const auto before = preceding.find(named[0]);
      key = before != preceding.end() && ending.contains(before->second)
                ? before->second
                : named[0];
    }
    if (!key.has_value()) {
      key = last;
    }
    gates.emplace_back(k, key);
    last = key;
  }
  // A piece with no terminal run at its ends that comes first in the list
  // has no gate before it: it belongs to the mouth of a piece it shares a
  // wall with, both sides of one lead or one pad.
  bool changed = true;
  while (changed) {
    changed = false;
    for (auto& [k, key] : gates) {
      if (key.has_value()) {
        continue;
      }
      std::set<std::string> walls;
      for (const auto& end : edges_[k].ends) {
        walls.insert(wallOf(end));
      }
      for (const auto& [k2, key2] : gates) {
        if (!key2.has_value()) {
          continue;
        }
        const bool shared =
            std::ranges::any_of(edges_[k2].ends, [&](const std::string& end) {
              return walls.contains(wallOf(end));
            });
        if (shared) {
          key = key2;
          changed = true;
          break;
        }
      }
    }
  }
  for (const auto& [k, key] : gates) {
    mouthOfGate_[k] = key;
  }
  std::set<std::string> keys;
  for (const auto& [k, key] : gates) {
    if (key.has_value()) {
      keys.insert(*key);
    }
  }
  std::vector<std::string> sortedKeys(keys.begin(), keys.end());
  std::ranges::stable_sort(sortedKeys, {}, numberOf);
  // Per mouth, its two terminal edges.
  std::vector<std::pair<std::string, std::pair<std::string, std::string>>>
      interval;
  for (const auto& key : sortedKeys) {
    if (ending.contains(key)) {
      const auto next = following.find(key);
      if (next != following.end()) {
        interval.emplace_back(key, std::pair{key, next->second});
      }
    } else {
      const auto before = preceding.find(key);
      if (before != preceding.end()) {
        interval.emplace_back(key, std::pair{before->second, key});
      }
    }
  }
  // The mouth every wire belongs to: the two terminal edges of a mouth, and
  // a plain wire with no prescription whose launcher lies between the
  // launchers of a mouth's two terminal edges, round the middle of the
  // chip.
  const Point middle{static_cast<double>(graph_.width) / 2.0,
                     static_cast<double>(graph_.height) / 2.0};
  const auto angleOf = [&](const Wire& wire) -> std::optional<double> {
    for (const auto* end : {&wire.source, &wire.target}) {
      if (end->label.starts_with("Chip.port")) {
        return std::atan2(end->cell.y - middle.y, end->cell.x - middle.x);
      }
    }
    return std::nullopt;
  };
  std::vector<std::pair<double, std::string>> ring;
  for (const auto& wire : wires_) {
    const auto angle = angleOf(wire);
    if (angle.has_value() &&
        (wire.terminal || (!wire.resonator && wire.crosses.empty()))) {
      ring.emplace_back(*angle, wire.name);
    }
  }
  std::ranges::sort(ring);
  const auto count = static_cast<std::ptrdiff_t>(ring.size());
  for (std::ptrdiff_t i = 0; i < count; ++i) {
    const auto& name = ring[static_cast<std::size_t>(i)].second;
    if (byName.at(name)->terminal) {
      continue;
    }
    const auto nearest =
        [&](const std::ptrdiff_t sign) -> std::optional<std::string> {
      for (std::ptrdiff_t j = 1; j < count; ++j) {
        const auto at = (((i + (sign * j)) % count) + count) % count;
        const auto& other = ring[static_cast<std::size_t>(at)].second;
        if (following.contains(other)) {
          return other;
        }
      }
      return std::nullopt;
    };
    const auto left = nearest(-1);
    const auto right = nearest(1);
    for (const auto& [key, ends] : interval) {
      const auto& [x, y] = ends;
      const auto inEnds = [&](const std::optional<std::string>& name2) {
        return name2 == x || name2 == y;
      };
      const auto inSides = [&](const std::string& name2) {
        return left == name2 || right == name2;
      };
      if (inSides(x) && inSides(y) && inEnds(left) && inEnds(right)) {
        mouthOfWire_[name] = key;
      }
    }
  }
  // The terminal edges of a mouth pass no other mouth. A terminal edge
  // belongs to the mouth it stands at: the mouth it names, and the mouth of
  // the ending edge before a starting edge.
  for (const auto& [key, ends] : interval) {
    for (const auto& terminal : {ends.first, ends.second}) {
      if (terminal == key ||
          ending.contains(terminal) != ending.contains(key)) {
        barred_[terminal] = key;
      }
    }
  }
  // A mouth with no gate in the graph (one chain, or a mouth that could not
  // be closed) binds no wire.
  std::erase_if(mouthOfWire_, [&](const auto& entry) {
    return !keys.contains(entry.second);
  });
  std::erase_if(
      barred_, [&](const auto& entry) { return !keys.contains(entry.second); });
}

/// The users and the mouth of every edge, for `openEdges`.
void Checker::indexEdges() {
  gateMouth_.assign(edges_.size(), nullptr);
  for (const auto& [k, key] : mouthOfGate_) {
    gateMouth_[k] = &key;
  }
  edgesOfUser_.clear();
  for (std::size_t k = 0; k < edges_.size(); ++k) {
    if (!edges_[k].users.has_value()) {
      continue;
    }
    for (const auto& name : *edges_[k].users) {
      auto& edges = edgesOfUser_[name];
      if (edges.empty() || edges.back() != k) {
        edges.push_back(k);
      }
    }
  }
}

/// Which edges a demand may use whatever their load, the bit of each
/// feedline edge it crosses, and the gates of its own mouth.
void Checker::openEdges(Demand& demand) const {
  const auto own = mouthOfWire_.find(demand.name);
  if (own != mouthOfWire_.end()) {
    demand.mouth = own->second;
  }
  std::optional<std::string> mine = demand.mouth;
  if (const auto bar = barred_.find(demand.name);
      !mine.has_value() && bar != barred_.end()) {
    mine = bar->second;
  }
  const auto ne = edges_.size();
  demand.open.assign(ne, false);
  demand.crossBit.assign(ne, 0);
  demand.ownMouth.assign(ne, false);
  // The edges open to every wire, and those whose users name this one.
  std::vector<bool> user(ne, false);
  for (std::size_t k = 0; k < ne; ++k) {
    user[k] = !edges_[k].users.has_value();
  }
  if (const auto named = edgesOfUser_.find(demand.name);
      named != edgesOfUser_.end()) {
    for (const auto k : named->second) {
      user[k] = true;
    }
  }
  static const std::optional<std::string> NO_MOUTH;
  for (std::size_t k = 0; k < ne; ++k) {
    const auto& edge = edges_[k];
    bool ok = user[k];
    if (edge.stretch) {
      // A wire crosses the slots of an edge it is prescribed to cross, and
      // may cross those of an edge whose slots are open to it without a
      // prescription.
      const auto at = std::ranges::find(demand.crosses, edge.feedline);
      if (at != demand.crosses.end()) {
        demand.crossBit[k] = 1ULL << (at - demand.crosses.begin());
      }
    }
    // A wire prescribed to cross an edge passes the ring there and nowhere
    // else: not over a mouth between two chains either, where the terminal
    // edges run.
    if (!demand.crosses.empty() && edge.mouth) {
      ok = false;
    }
    const auto& mouthOf = gateMouth_[k] == nullptr ? NO_MOUTH : *gateMouth_[k];
    // A wire of a mouth passes that mouth and no other. A piece whose mouth
    // is not known bars no wire (9q case 09 of round 54: the line from f4
    // to f6 in the mouth of f5).
    if (mine.has_value() && edge.mouth && mouthOf.has_value() &&
        mouthOf != mine) {
      ok = false;
    }
    demand.open[k] = ok;
    demand.ownMouth[k] = demand.mouth.has_value() && mouthOf == demand.mouth;
  }
  demand.openMoves.assign(around_.size(), {});
  demand.openMovesKnown.assign(around_.size(), false);
}

Way Checker::shortest(const Demand& demand,
                      const std::vector<double>& weight) const {
  const auto nc = demand.crosses.size();
  const std::uint64_t mouthBit = demand.mouth.has_value() ? (1ULL << nc) : 0;
  const std::uint64_t full = ((1ULL << nc) - 1) | mouthBit;
  if (full == 0 && std::ranges::any_of(demand.from, [&](const auto chamber) {
        return std::ranges::find(demand.to, chamber) != demand.to.end();
      })) {
    return {std::vector<std::size_t>{}, 0.0};
  }
  const auto& [start, end] = ends_.at(demand.name);
  const auto allowed = [&](const std::size_t k) {
    return demand.open[k] && weight[k] >= 0.0;
  };
  // The states in flat arrays, numbered `pair * maskCount_ + mask`; the
  // heap orders by distance and then by the state's code (`stateOf`), as
  // the search always did, so that equal distances break alike.
  const auto states = pairCount_ * maskCount_;
  if (best_.size() < states) {
    best_.assign(states, Best{});
    generation_ = 0;
  }
  if (++generation_ == 0) {
    for (auto& state : best_) {
      state.seen = 0;
    }
    generation_ = 1;
  }
  constexpr auto NO_STATE = std::numeric_limits<std::uint64_t>::max();
  const auto known = [&](const std::size_t at) {
    return best_[at].seen == generation_
               ? best_[at].d
               : std::numeric_limits<double>::infinity();
  };
  const auto keep = [&](const std::size_t at, const double d,
                        const std::uint64_t from, const double step) {
    best_[at] = {.d = d, .step = step, .from = from, .seen = generation_};
  };
  // The heap of `Queued` states, kept from search to search.
  auto& heap = queue_;
  heap.clear();
  const auto push = [&](const Queued& item) {
    heap.push_back(item);
    std::push_heap(heap.begin(), heap.end(), std::greater<>());
  };
  static const std::vector<Move> NONE;
  const auto leaving =
      [&](const std::int64_t chamber) -> const std::vector<Move>& {
    return chamber >= 0 && std::cmp_less(chamber, around_.size())
               ? around_[static_cast<std::size_t>(chamber)]
               : NONE;
  };
  if (!demand.startApartKnown) {
    for (const auto chamber : demand.from) {
      for (const auto& move : leaving(chamber)) {
        demand.startApart.push_back(distance(mid_[move.edge], start));
      }
    }
    demand.startApartKnown = true;
  }
  std::size_t first = 0;
  for (const auto chamber : demand.from) {
    for (const auto& move : leaving(chamber)) {
      const auto k = move.edge;
      const auto apart = demand.startApart[first++];
      if (!allowed(k)) {
        continue;
      }
      const auto mask =
          demand.crossBit[k] | (demand.ownMouth[k] ? mouthBit : 0);
      const auto d = (apart * weight[k]) + 1.0;
      const auto at = (move.pair * maskCount_) + mask;
      if (d < known(at)) {
        keep(at, d, NO_STATE, apart);
        push({d, stateOf(k, move.into, mask), at, k, move.into});
      }
    }
  }
  std::optional<std::size_t> goal;
  auto goalCost = std::numeric_limits<double>::infinity();
  while (!heap.empty()) {
    std::pop_heap(heap.begin(), heap.end(), std::greater<>());
    const auto item = heap.back();
    heap.pop_back();
    const auto d = item.d;
    if (d > best_[item.at].d || d >= goalCost) {
      continue;
    }
    const auto k = item.edge;
    const auto c = item.chamber;
    const auto mask = static_cast<std::uint64_t>(item.at % maskCount_);
    const auto& steps = stepsOf(item.at / maskCount_);
    if (mask == full && std::ranges::find(demand.to, c) != demand.to.end()) {
      const auto total = d + distance(end, mid_[k]);
      if (total < goalCost) {
        goalCost = total;
        goal = item.at;
      }
    }
    if (c < 0 || std::cmp_greater_equal(c, around_.size())) {
      continue;
    }
    const auto chamber = static_cast<std::size_t>(c);
    const auto& moves = around_[chamber];
    auto& usable = demand.openMoves[chamber];
    if (!demand.openMovesKnown[chamber]) {
      for (std::size_t m = 0; m < moves.size(); ++m) {
        if (demand.open[moves[m].edge]) {
          usable.push_back(static_cast<std::uint32_t>(m));
        }
      }
      demand.openMovesKnown[chamber] = true;
    }
    for (const auto m : usable) {
      const auto& move = moves[m];
      const auto k2 = move.edge;
      if (!allowed(k2) || k2 == k) {
        continue;
      }
      auto mask2 = mask;
      if (isSlot_[k2] != 0) {
        if ((mask & demand.crossBit[k2]) != 0) {
          continue;
        }
        mask2 = mask | demand.crossBit[k2];
      }
      if (demand.ownMouth[k2]) {
        mask2 |= mouthBit;
      }
      const auto step = steps[m];
      const auto d2 = d + (step * weight[k2]) + 1.0;
      const auto at2 = (move.pair * maskCount_) + mask2;
      if (d2 < known(at2)) {
        keep(at2, d2, item.at, step);
        push({d2, stateOf(k2, move.into, mask2), at2, k2, move.into});
      }
    }
  }
  if (!goal.has_value()) {
    return {std::nullopt, std::numeric_limits<double>::infinity()};
  }
  std::vector<std::size_t> path;
  auto length = distance(end, mid_[edgeOfPair_[*goal / maskCount_]]);
  std::uint64_t state = *goal;
  while (state != NO_STATE) {
    path.push_back(edgeOfPair_[state / maskCount_]);
    length += best_[state].step;
    state = best_[state].from;
  }
  std::ranges::reverse(path);
  return {std::move(path), length};
}

std::pair<SlotCheckResult, json> Checker::run() {
  readStrokesAndWires();
  addSides();
  SlotPlacer placer(graph_, grid_, page_ != nullptr);
  auto slots = placer.run(strokes_);
  addSlots(slots);
  const auto ne = edges_.size();
  for (const auto& edge : edges_) {
    capacity_.push_back(edge.capacity);
  }
  auto groups = selfBottlenecks(graph_, grid_, strokes_, edges_);
  for (auto& group : slotStubBottlenecks(graph_, grid_, *placer.bits, strokes_,
                                         edges_, wires_)) {
    groups.push_back(std::move(group));
  }
  json escapesOut = json::array();
  if (slotEscape()) {
    for (auto& group : escapeLimits(graph_, grid_, edges_,
                                    page_ != nullptr ? &escapesOut : nullptr)) {
      groups.push_back(std::move(group));
    }
  }
  // Per edge: the groups whose slots it is.
  std::vector<std::vector<std::size_t>> member(ne);
  // Per wire: the groups whose room it ends in.
  std::unordered_map<std::string, std::vector<std::size_t>> inRoom;
  for (std::size_t gi = 0; gi < groups.size(); ++gi) {
    for (const auto k : groups[gi].slots) {
      member[k].push_back(gi);
    }
    for (const auto& name : groups[gi].enders) {
      inRoom[name].push_back(gi);
    }
  }
  // The groups whose line the wire passes: a wire enters or leaves the room
  // past the line at each slot of the room it crosses and at each of its
  // ends in the room, and passes the line when that count is odd.
  const auto passes = [&](const std::string& name,
                          const std::optional<std::vector<std::size_t>>& path) {
    std::map<std::size_t, std::size_t> count;
    if (path.has_value()) {
      for (const auto k : *path) {
        for (const auto gi : member[k]) {
          ++count[gi];
        }
      }
    }
    const auto found = inRoom.find(name);
    if (found != inRoom.end()) {
      for (const auto gi : found->second) {
        ++count[gi];
      }
    }
    std::vector<std::size_t> odd;
    for (const auto& [gi, times] : count) {
      if (times % 2 == 1) {
        odd.push_back(gi);
      }
    }
    return odd;
  };

  std::vector<Demand> demands;
  std::vector<std::string> lost;
  for (const auto& wire : wires_) {
    if (wire.source.chambers.empty() || wire.target.chambers.empty()) {
      lost.push_back(wire.name);
      continue;
    }
    demands.push_back({.name = wire.name,
                       .from = wire.source.chambers,
                       .to = wire.target.chambers,
                       .crosses = wire.crosses,
                       .mouth = std::nullopt,
                       .open = {},
                       .crossBit = {},
                       .ownMouth = {}});
  }
  // Every edge's middle, and its moves: from a chamber on one side to one
  // on the other (a gate whose line has two clear sides), or between any
  // two of its chambers. A slot of an edge of a chain with a reflector end
  // may have one chamber on both sides: its move leads from that chamber
  // back into it, so that a wire prescribed to cross the edge can (45q,
  // chain 3 checked alone: f20 had five such slots and wires 57 and 58,
  // prescribed to cross it, no way).
  std::int64_t mostChamber = 0;
  std::vector<std::tuple<std::int64_t, std::size_t, std::int64_t>> moves;
  for (std::size_t k = 0; k < ne; ++k) {
    const auto& edge = edges_[k];
    mid_.push_back(
        {(edge.from.x + edge.to.x) / 2.0, (edge.from.y + edge.to.y) / 2.0});
    std::vector<std::pair<std::int64_t, std::int64_t>> pairs;
    if (edge.sides.has_value()) {
      for (const auto x : (*edge.sides)[0]) {
        for (const auto y : (*edge.sides)[1]) {
          pairs.emplace_back(x, y);
        }
      }
      const auto forward = pairs.size();
      for (std::size_t p = 0; p < forward; ++p) {
        pairs.emplace_back(pairs[p].second, pairs[p].first);
      }
    } else {
      for (const auto x : edge.chambers) {
        for (const auto y : edge.chambers) {
          if (x != y) {
            pairs.emplace_back(x, y);
          }
        }
      }
      if (edge.stretch && !edge.chambers.empty() &&
          std::ranges::all_of(edge.chambers, [&](const auto chamber) {
            return chamber == edge.chambers.front();
          })) {
        pairs.emplace_back(edge.chambers.front(), edge.chambers.front());
      }
    }
    for (const auto& [x, y] : pairs) {
      moves.push_back({x, k, y});
      mostChamber = std::max({mostChamber, x, y});
    }
  }
  around_.assign(static_cast<std::size_t>(mostChamber) + 1, {});
  {
    std::map<std::pair<std::size_t, std::int64_t>, std::size_t> pairOf;
    for (const auto& [x, k, y] : moves) {
      const auto [at, fresh] = pairOf.try_emplace({k, y}, pairOf.size());
      around_[static_cast<std::size_t>(x)].push_back(
          {.edge = k, .into = y, .pair = at->second});
    }
    pairCount_ = pairOf.size();
    isSlot_.assign(ne, 0);
    for (std::size_t k = 0; k < ne; ++k) {
      isSlot_[k] = edges_[k].stretch ? 1 : 0;
    }
    edgeOfPair_.assign(pairCount_, 0);
    intoOfPair_.assign(pairCount_, 0);
    stepsOut_.assign(pairCount_, {});
    stepsKnown_.assign(pairCount_, false);
    for (const auto& [pair, number] : pairOf) {
      edgeOfPair_[number] = pair.first;
      intoOfPair_[number] = pair.second;
    }
  }
  std::size_t mostCrosses = 0;
  for (const auto& demand : demands) {
    mostCrosses = std::max(mostCrosses, demand.crosses.size());
  }
  maskCount_ = std::size_t{1} << (mostCrosses + 1);
  if (ne > FIELD || std::cmp_greater(mostChamber, FIELD) ||
      mostCrosses + 1 >= static_cast<std::size_t>(CHAMBER_SHIFT)) {
    throw std::invalid_argument(
        "slot check: the graph is too large for the search");
  }
  findOwners();
  limits_.resize(ne);
  for (std::size_t k = 0; k < ne; ++k) {
    for (std::int64_t own = 0; own <= 2; ++own) {
      limits_[k][static_cast<std::size_t>(own)] = limitFor(k, own);
    }
  }
  markMouthTwins();
  findMouths();
  indexEdges();
  for (auto& demand : demands) {
    openEdges(demand);
  }

  // Per wire and edge: whether the edge has an end at the wire's own run.
  std::unordered_map<std::string, std::size_t> demandOf;
  for (std::size_t d = 0; d < demands.size(); ++d) {
    demandOf.emplace(demands[d].name, d);
  }
  std::vector<std::vector<std::int64_t>> owns(demands.size(),
                                              std::vector<std::int64_t>(ne, 0));
  for (std::size_t k = 0; k < ne; ++k) {
    for (const auto& name : owners_[k]) {
      if (const auto found = demandOf.find(name); found != demandOf.end()) {
        owns[found->second][k] = 1;
      }
    }
  }
  std::unordered_map<std::string, Way> base;
  std::unordered_map<std::string, double> tautBase;
  for (std::size_t d = 0; d < demands.size(); ++d) {
    const auto& demand = demands[d];
    std::vector<double> weight(ne, -1.0);
    for (std::size_t k = 0; k < ne; ++k) {
      if (limitOf(k, owns[d][k]) >= 1) {
        weight[k] = 1.0;
      }
    }
    base[demand.name] = shortest(demand, weight);
  }
  // Per wire: the ways pulled tight so far and their lengths, as a wire
  // often takes a way again in a later round.
  std::vector<std::map<std::vector<std::size_t>, double>> tautWays(
      demands.size());
  const auto linesOf = [&](const std::vector<std::size_t>& path) {
    std::vector<std::pair<Point, Point>> lines;
    lines.reserve(path.size());
    for (const auto k : path) {
      lines.emplace_back(edges_[k].from, edges_[k].to);
    }
    return lines;
  };

  std::vector<double> hist(ne, 0.0);
  // The ways by wire, in the order the wires were first routed.
  std::vector<std::pair<std::string, std::optional<std::vector<std::size_t>>>>
      ways;
  std::unordered_map<std::string, std::size_t> wayOf;
  std::vector<std::int64_t> load(ne, 0);
  std::vector<std::int64_t> limits(ne, 0);
  std::vector<std::size_t> over;
  std::vector<std::size_t> gover;
  std::vector<std::int64_t> gload(groups.size(), 0);
  int round = 0;
  // Per wire: the edges with an end at its own run.
  std::vector<std::vector<std::size_t>> ownEdges(demands.size());
  for (std::size_t d = 0; d < demands.size(); ++d) {
    for (std::size_t k = 0; k < ne; ++k) {
      if (owns[d][k] != 0) {
        ownEdges[d].push_back(k);
      }
    }
  }
  for (round = 0; round < ROUNDS; ++round) {
    std::ranges::fill(load, 0);
    std::vector<std::int64_t> ownLoad(ne, 0);
    std::ranges::fill(gload, 0);
    // The price of edge k for a wire with `own` ends at its own run there,
    // from the loads of the wires routed so far in this round.
    const auto priceOf = [&](const std::size_t k, const std::int64_t own) {
      const auto lim = limitOf(k, ownLoad[k] + own);
      auto overK = std::max<std::int64_t>(0, load[k] + 1 - lim);
      for (const auto gi : member[k]) {
        overK += std::max<std::int64_t>(0, gload[gi] + 1 - groups[gi].capacity);
      }
      return lim >= 1 ? (1.0 + hist[k]) *
                            (1.0 + ((0.5 * static_cast<double>(round + 1)) *
                                    static_cast<double>(overK)))
                      : -1.0;
    };
    // The prices for a wire with no own run at an edge; a wire changes
    // them only on the edges of its way and the slots of the lines it
    // passes, so they are priced again there after each wire.
    std::vector<double> common(ne);
    for (std::size_t k = 0; k < ne; ++k) {
      common[k] = priceOf(k, 0);
    }
    std::vector<double> saved;
    for (std::size_t d = 0; d < demands.size(); ++d) {
      const auto& demand = demands[d];
      // The wire's prices: the common ones, and its own at the edges with
      // an end at its own run, for this search alone.
      saved.clear();
      for (const auto k : ownEdges[d]) {
        saved.push_back(common[k]);
        common[k] = priceOf(k, 1);
      }
      auto [path, length] = shortest(demand, common);
      for (std::size_t own = 0; own < ownEdges[d].size(); ++own) {
        common[ownEdges[d][own]] = saved[own];
      }
      const auto& [basePath, baseLength0] = base.at(demand.name);
      auto baseLength = baseLength0;
      if (path.has_value() && basePath.has_value()) {
        // The lengths of the two ways pulled tight through their gates: a
        // thin wedge between two gates adds a gate to a way but no length
        // to the wire.
        const auto& [start, end] = ends_.at(demand.name);
        auto cached = tautBase.find(demand.name);
        if (cached == tautBase.end()) {
          cached = tautBase
                       .emplace(demand.name,
                                tautLength(start, end, linesOf(*basePath)))
                       .first;
        }
        baseLength = cached->second;
        auto taut = tautWays[d].find(*path);
        if (taut == tautWays[d].end()) {
          taut = tautWays[d]
                     .emplace(*path, tautLength(start, end, linesOf(*path)))
                     .first;
        }
        length = taut->second;
      }
      // A way longer than DETOUR times the shortest over the gates that
      // take a wire is no way: the wire takes its shortest way and
      // overfills what it must.
      if (path.has_value() && basePath.has_value() &&
          length > (DETOUR * baseLength) + 1e-9) {
        path = basePath;
      }
      const auto known = wayOf.find(demand.name);
      if (known == wayOf.end()) {
        wayOf.emplace(demand.name, ways.size());
        ways.emplace_back(demand.name, path);
      } else {
        ways[known->second].second = path;
      }
      if (path.has_value()) {
        for (const auto k : *path) {
          ++load[k];
          ownLoad[k] += owns[d][k];
        }
      }
      const auto passed = passes(demand.name, path);
      for (const auto gi : passed) {
        ++gload[gi];
      }
      if (path.has_value()) {
        for (const auto k : *path) {
          common[k] = priceOf(k, 0);
        }
      }
      for (const auto gi : passed) {
        for (const auto k : groups[gi].slots) {
          common[k] = priceOf(k, 0);
        }
      }
    }
    over.clear();
    for (std::size_t k = 0; k < ne; ++k) {
      limits[k] = limitOf(k, ownLoad[k]);
      if (load[k] > limits[k]) {
        over.push_back(k);
      }
    }
    gover.clear();
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
      if (gload[gi] > groups[gi].capacity) {
        gover.push_back(gi);
      }
    }
    const bool routed = std::ranges::all_of(
        ways, [](const auto& entry) { return entry.second.has_value(); });
    if (over.empty() && gover.empty() && routed) {
      break;
    }
    for (const auto k : over) {
      hist[k] += 0.3 * static_cast<double>(load[k] - limits[k]);
    }
    for (const auto gi : gover) {
      for (const auto k : groups[gi].slots) {
        hist[k] += 0.3 * static_cast<double>(gload[gi] - groups[gi].capacity);
      }
    }
  }
  const auto rounds = std::min(round, ROUNDS - 1) + 1;
  std::vector<std::string> unrouted;
  for (const auto& demand : demands) {
    const auto at = wayOf.find(demand.name);
    if (at == wayOf.end() || !ways[at->second].second.has_value()) {
      unrouted.push_back(demand.name);
    }
  }
  const bool sat =
      over.empty() && gover.empty() && unrouted.empty() && lost.empty();
  SlotCheckResult result;
  result.verdict = sat ? "SAT" : "UNSAT";
  result.lost = lost;
  result.lost.insert(result.lost.end(), unrouted.begin(), unrouted.end());
  for (const auto k : over) {
    result.over.push_back(edges_[k].name);
  }
  for (const auto gi : gover) {
    result.over.push_back(std::format("{} {}/{}", groups[gi].name, gload[gi],
                                      groups[gi].capacity));
  }
  if (!over.empty() || !gover.empty()) {
    std::vector<bool> isOver(ne, false);
    for (const auto k : over) {
      isOver[k] = true;
    }
    for (const auto& [name, path] : ways) {
      bool crowded =
          path.has_value() &&
          std::ranges::any_of(*path, [&](const auto k) { return isOver[k]; });
      if (!crowded && !gover.empty()) {
        const auto passed = passes(name, path);
        crowded = std::ranges::any_of(passed, [&](const auto gi) {
          return std::ranges::find(gover, gi) != gover.end();
        });
      }
      if (crowded) {
        result.crowded.push_back(name);
      }
    }
  }

  if (page_ == nullptr) {
    return {std::move(result), json()};
  }
  // The graph for the page.
  std::vector<std::vector<std::string>> through(ne);
  for (const auto& [name, path] : ways) {
    if (path.has_value()) {
      for (const auto k : *path) {
        through[k].push_back(name);
      }
    }
  }
  auto edgesOut = json::array();
  for (std::size_t k = 0; k < ne; ++k) {
    auto& out = edges_[k].out;
    std::ranges::sort(through[k]);
    out["index"] = k;
    out["load"] = load[k];
    out["overflow"] = std::max<std::int64_t>(0, load[k] - limits[k]);
    // The capacity the check routed against: after the margin and the
    // resonator passage, which the graph of the check by stretches does not
    // carry (45q case 02 of round 55 showed 5 where the check took 4).
    out["capacity"] = limits[k];
    out["through"] = through[k];
    edgesOut.push_back(std::move(out));
  }
  // The self-bottlenecks as edges of the page: the line, what it takes, and
  // the wires that pass it.
  for (std::size_t gi = 0; gi < groups.size(); ++gi) {
    const auto& group = groups[gi];
    const auto ax = static_cast<double>(group.line[0][0]);
    const auto ay = static_cast<double>(group.line[0][1]);
    const auto bx = static_cast<double>(group.line[1][0]);
    const auto by = static_cast<double>(group.line[1][1]);
    const auto mx = (ax + bx) / 2.0;
    const auto my = (ay + by) / 2.0;
    const auto length = std::max(1e-9, std::hypot(bx - ax, by - ay));
    const auto nx = -(by - ay) / length;
    const auto ny = (bx - ax) / length;
    auto sides = json::array();
    for (const double sign : {1.0, -1.0}) {
      for (std::int64_t step = 2; step < 30; ++step) {
        const auto x = rounded(mx + ((sign * static_cast<double>(step)) * nx));
        const auto y = rounded(my + ((sign * static_cast<double>(step)) * ny));
        if (grid_.inside(x, y) && grid_.at(x, y) >= CHAMBER) {
          sides.push_back(grid_.at(x, y) - CHAMBER);
          break;
        }
      }
    }
    if (sides.empty()) {
      sides.push_back(0);
    }
    std::vector<std::string> names;
    for (const auto& [name, path] : ways) {
      const auto passed = passes(name, path);
      if (std::ranges::find(passed, gi) != passed.end()) {
        names.push_back(name);
      }
    }
    std::ranges::sort(names);
    auto slotNames = json::array();
    for (const auto k : group.slots) {
      slotNames.push_back(edges_[k].name);
    }
    edgesOut.push_back(
        {{"kind", "bottleneck"},
         {"name", "S " + group.name},
         {"index", edgesOut.size()},
         {"line", json::array({group.line[0], group.line[1]})},
         {"ends", json::array({group.feedline, group.feedline})},
         {"chambers", std::move(sides)},
         {"capacity", group.capacity},
         {"load", gload[gi]},
         {"overflow", std::max<std::int64_t>(0, gload[gi] - group.capacity)},
         {"through", names},
         {"users", nullptr},
         {"length", roundedTenth(length)},
         {"mouth", false},
         {"slot", false},
         {"terminal", false},
         {"crossingNow", json::array()},
         {"selfBottleneck", true},
         {"slots", std::move(slotNames)},
         {"endsInRoom", group.enders}});
  }
  auto wiresOut = json::array();
  for (const auto& wire : page_->at("wires")) {
    auto out = wire;
    const auto name = wire.at("name").get<std::string>();
    const auto at = wayOf.find(name);
    const bool routed =
        at != wayOf.end() && ways[at->second].second.has_value();
    out["way"] = routed ? json(*ways[at->second].second) : json::array();
    if (routed) {
      out["status"] = "routed";
    } else if (std::ranges::find(lost, name) == lost.end()) {
      out["status"] = "no way";
    } else if (!wire.contains("status")) {
      out["status"] = "no way";
    }
    wiresOut.push_back(std::move(out));
  }
  auto slotsOut = json::array();
  std::int64_t newStretches = 0;
  for (const auto& edge : edges_) {
    newStretches += edge.stretch ? 1 : 0;
  }
  for (const auto& edge : slots) {
    auto placed = json::array();
    for (const auto& slot : edge.slots) {
      auto corners = json::array();
      for (const auto& corner : slot.corners) {
        corners.push_back(realJson(corner));
      }
      placed.push_back({{"at", pointJson(slot.at)},
                        {"corners", std::move(corners)},
                        {"direction", realJson(slot.direction)}});
    }
    auto refused = json::array();
    for (const auto& place : edge.refused) {
      auto corners = json::array();
      for (const auto& corner : place.corners) {
        corners.push_back(realJson(corner));
      }
      refused.push_back({{"at", pointJson(place.at)},
                         {"why", place.why},
                         {"corners", std::move(corners)}});
    }
    slotsOut.push_back({{"name", edge.name},
                        {"length", roundedTenth(edge.length)},
                        {"slots", std::move(placed)},
                        {"count", edge.slots.size()},
                        {"check", edge.stretches},
                        {"spread", edge.spread},
                        {"rejected", std::move(refused)}});
  }
  std::int64_t oldStretches = 0;
  for (const auto& edge : page_->at("edges")) {
    if (edge.at("kind") == "stretch") {
      oldStretches += edge.at("capacity").get<std::int64_t>();
    }
  }
  // The wires by name, apart by spaces, as the check before lists them.
  const auto listed = [](const std::vector<std::string>& names) {
    std::string list;
    for (const auto& name : names) {
      list += (list.empty() ? "" : " ") + name;
    }
    return list;
  };
  const auto gates = std::ranges::count_if(
      edges_, [](const Edge& edge) { return !edge.stretch; });
  auto out = *page_;
  out["edges"] = std::move(edgesOut);
  out["wires"] = std::move(wiresOut);
  out["verdict"] = result.verdict;
  out["crossSlots"] = {{"reach", placer.reach},
                       {"along", placer.along},
                       {"edges", std::move(slotsOut)}};
  if (slotEscape()) {
    out["escapes"] = std::move(escapesOut);
    auto runs = json::array();
    for (const auto& run : couplerRunsOf(graph_)) {
      runs.push_back(json::array({realJson(run[0]), realJson(run[1])}));
    }
    out["couplerRuns"] = std::move(runs);
  }
  out["summary"] = json::array(
      {"the capacity check by crossing slots: the rectangle slots of every "
       "feedline edge in place of its stretches; a gate joins only chambers "
       "on different sides of its line; a way runs at most 1.5 times its "
       "shortest way, both pulled tight; a wire with a crossing "
       "prescription passes no mouth between chains; a plain wire not "
       "enclosed and the terminal edges pass their own mouth and no other" +
           std::string(ownRunBonus()
                           ? "; a gate at a wire's own run takes half a "
                             "clearance more for that wire when the wire "
                             "leaves the run straight through the gate"
                           : ""),
       std::format(
           "{} self-bottlenecks (S …): a free line from one place of a "
           "feedline edge to another, or from a slot to the open end of a "
           "stub, with more slots and wire ends behind it than it takes "
           "wires; {} chambers, {} bottlenecks; {} crossing slots (the "
           "stretches gave {})",
           groups.size(), graph_.chambers, gates, newStretches, oldStretches),
       std::format("{}; {} edges and {} self-bottlenecks over; routed by "
                   "negotiated congestion (PathFinder), {} rounds",
                   result.verdict, over.size(), gover.size(), rounds)});
  if (!lost.empty()) {
    out["summary"].push_back("a port in no chamber for " + listed(lost));
  }
  if (!unrouted.empty()) {
    out["summary"].push_back("no way in the graph for " + listed(unrouted));
  }
  return {std::move(result), std::move(out)};
}

/// The check on a graph given as JSON text. Errors of the JSON become
/// `std::invalid_argument`.
/// The graph in memory as the JSON of `final-capacity-graph.json` gives it.
[[nodiscard]] SlotGraph graphOf(const json& graph) {
  SlotGraph out;
  out.width = graph.at("width").get<std::int64_t>();
  out.height = graph.at("height").get<std::int64_t>();
  out.clearance = graph.at("clearance").get<double>();
  out.crossingReach = graph.at("crossingReach").get<double>();
  out.inflation = graph.at("inflation").get<double>();
  out.capacityMargin = graph.value("capacityMargin", out.capacityMargin);
  out.capacityMarginFromWires =
      graph.value("capacityMarginFromWires", out.capacityMarginFromWires);
  out.resonatorPassage = graph.value("resonatorPassage", out.resonatorPassage);
  out.slotPinCells = graph.value("slotPinCells", out.slotPinCells);
  out.escapeStraight = graph.value("escapeStraight", out.escapeStraight);
  out.escapeStubMargin = graph.value("escapeStubMargin", out.escapeStubMargin);
  out.wallKinds = graph.at("wallKinds").get<std::vector<std::string>>();
  out.codes.reserve(static_cast<std::size_t>(out.width * out.height));
  const auto& raster = graph.at("raster");
  for (std::size_t at = 0; at + 1 < raster.size(); at += 2) {
    const auto code = raster[at].get<std::int64_t>();
    const auto run = raster[at + 1].get<std::int64_t>();
    if (code < 0 || code > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument(
          std::format("slot check: raster code {} out of range", code));
    }
    if (code == STUB) {
      for (std::int64_t k = 0; k < run; ++k) {
        out.stubCells.push_back(out.codes.size() + static_cast<std::size_t>(k));
      }
    }
    out.codes.insert(out.codes.end(), static_cast<std::size_t>(run),
                     static_cast<std::uint16_t>(code));
  }
  for (const auto& cell : graph.at("slots")) {
    out.portSlots.push_back(
        {cell.at(0).get<std::int64_t>(), cell.at(1).get<std::int64_t>()});
  }
  const auto pairOf = [](const json& value) {
    const auto point = pointOf(value);
    return std::array{point.x, point.y};
  };
  for (const auto& stroke : graph.at("strokes")) {
    SlotGraph::Stroke one{.name = stroke.at("name").get<std::string>(),
                          .kind = stroke.at("kind").get<std::string>(),
                          .points = {}};
    for (const auto& point : stroke.at("points")) {
      one.points.push_back(pairOf(point));
    }
    out.strokes.push_back(std::move(one));
  }
  for (const auto& edge : graph.at("edges")) {
    SlotGraph::Edge one;
    one.stretch = edge.at("kind") == "stretch";
    one.name = textOf(edge, "name");
    one.chambers = edge.at("chambers").get<std::vector<std::int64_t>>();
    one.capacity = edge.at("capacity").get<std::int64_t>();
    if (edge.contains("users") && !edge.at("users").is_null()) {
      one.users = edge.at("users").get<std::vector<std::string>>();
    }
    if (one.stretch) {
      one.feedline = edge.at("feedline").get<std::string>();
    } else {
      one.from = pairOf(edge.at("line").front());
      one.to = pairOf(edge.at("line").back());
      if (edge.contains("ends")) {
        one.ends = edge.at("ends").get<std::vector<std::string>>();
      }
      one.mouth = truthy(edge, "mouth");
      one.length = edge.at("length").get<double>();
      if (edge.contains("sides")) {
        one.sides =
            std::array{edge.at("sides").at(0).get<std::vector<std::int64_t>>(),
                       edge.at("sides").at(1).get<std::vector<std::int64_t>>()};
      }
    }
    out.edges.push_back(std::move(one));
  }
  const auto endOf = [&](const json& end) {
    SlotGraph::End one{.cell = pairOf(end.at("cell")),
                       .chambers = {},
                       .label = textOf(end, "label")};
    if (truthy(end, "chambers")) {
      one.chambers = end.at("chambers").get<std::vector<std::int64_t>>();
    }
    return one;
  };
  for (const auto& wire : graph.at("wires")) {
    SlotGraph::Wire one{.name = wire.at("name").get<std::string>(),
                        .source = endOf(wire.at("source")),
                        .target = endOf(wire.at("target")),
                        .crosses = {},
                        .terminal = truthy(wire, "terminal"),
                        .resonator = truthy(wire, "resonator")};
    if (truthy(wire, "crosses")) {
      one.crosses = wire.at("crosses").get<std::vector<std::string>>();
    }
    out.wires.push_back(std::move(one));
  }
  out.chambers = graph.at("nodes").size();
  return out;
}

/// The check on a graph given as JSON, with the graph for the page.
std::pair<SlotCheckResult, json> judge(const std::string_view graph) {
  try {
    const auto parsed = json::parse(graph);
    const auto typed = graphOf(parsed);
    useSettings(typed);
    return Checker(typed, &parsed).run();
  } catch (const json::exception& error) {
    throw std::invalid_argument(std::format(
        "slot check: the graph is no capacity graph: {}", error.what()));
  }
}

} // namespace

SlotCheckResult slotCheck(const std::string_view graph) {
  auto [result, out] = judge(graph);
  result.graph = out.dump();
  return result;
}

SlotCheckResult slotCheck(const SlotGraph& graph) {
  useSettings(graph);
  return Checker(graph, nullptr).run().first;
}

SlotGraph slotGraphOf(const std::string_view graph) {
  try {
    return graphOf(json::parse(graph));
  } catch (const json::exception& error) {
    throw std::invalid_argument(std::format(
        "slot check: the graph is no capacity graph: {}", error.what()));
  }
}

std::string slotCheckJson(const std::string_view graph) {
  auto [result, out] = judge(graph);
  return json{{"verdict", result.verdict},
              {"lost", result.lost},
              {"over", result.over},
              {"graph", std::move(out)}}
      .dump();
}

} // namespace mqt::scpd::pipeline
