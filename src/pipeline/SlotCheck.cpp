/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/SlotCheck.hpp"

#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
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
  /// The codes of the graph's cells, row by row (`SlotGraph::codes`).
  const std::uint16_t* code = nullptr;
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
  [[nodiscard]] int at(const std::int64_t x, const std::int64_t y) const {
    return code[cell(x, y)];
  }
  /// Whether a wire may pass the cell: a chamber or a cut line.
  [[nodiscard]] bool open(const std::int64_t x, const std::int64_t y) const {
    const auto value = at(x, y);
    return value >= CHAMBER || value == 0;
  }
};

[[nodiscard]] Grid gridOf(const SlotGraph& graph) {
  if (std::cmp_not_equal(graph.codes.size(), graph.width * graph.height)) {
    throw std::invalid_argument(
        "slot check: the raster does not cover the chip");
  }
  return {.width = graph.width,
          .height = graph.height,
          .code = graph.codes.data(),
          .size = graph.codes.size(),
          .stubCells = &graph.stubCells};
}

/// A mark per cell, reset by a new generation instead of a pass over every
/// cell.
class Stamps {
public:
  explicit Stamps(const std::size_t cells) : mark_(cells, 0) {}

  /// `count` fresh generations in a row; returns the first.
  std::uint16_t fresh(const std::uint16_t count) {
    if (next_ + count >= std::numeric_limits<std::uint16_t>::max()) {
      std::ranges::fill(mark_, std::uint16_t{0});
      next_ = 1;
    }
    const auto first = next_;
    next_ = static_cast<std::uint16_t>(next_ + count);
    return first;
  }
  [[nodiscard]] std::uint16_t& operator[](const std::size_t cell) {
    return mark_[cell];
  }

private:
  std::vector<std::uint16_t> mark_;
  std::uint16_t next_ = 1;
};

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

/// A way out of one end of a crossing slot (`SCPD_SLOT_ESCAPE`): one of the
/// router's turns from the heading across the edge, then `SCPD_ESCAPE_STRAIGHT`
/// cells on.
struct EscapeWay {
  /// The cells the way covers.
  std::vector<Cell> cells;
  /// The way's curve, for the page.
  std::vector<Point> line;
};

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
  /// (`SCPD_SLOT_ESCAPE`), or none.
  std::vector<EscapeWay> ways;
  json out;
};

/// A crossing slot on a feedline edge.
struct Slot {
  Point at;
  std::array<Point, 4> corners;
  Point direction;
  /// The ways out of its two ends, the end to the left of `direction`
  /// first (`SCPD_SLOT_ESCAPE`), or none.
  std::vector<EscapeWay> ways;
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
  /// Whether the slots are spread evenly (`SCPD_EVEN_SLOTS`), or laid one
  /// after the other because a spread gave another number.
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
  bool gateHeading = true;
  double resonatorPassage = 0.8;
  bool slotCrossingRule = true;
  bool mouthTwins = true;
  bool evenSlots = true;
  double staticGapBonus = 0.0;
  double slotReachMargin = 0.0;
  bool slotEscape = true;
  std::size_t slotPinCells = SLOT_PIN_CELLS;
  double slotStubGap = 0.0;
  bool escapeQuarterTurns = true;
  std::int64_t escapeStraight = 5;
  bool escapeCouplerRuns = false;
  bool ownRunBonus = false;
  double escapeApartMargin = 0.0;
  double escapeStubMargin = ESCAPE_STUB_MARGIN;
};

[[nodiscard]] Settings& settings() {
  thread_local Settings current;
  return current;
}

// The switches of the running check; each is described where it is read
// from the environment below.
[[nodiscard]] double capacityMargin() { return settings().margin; }
[[nodiscard]] std::int64_t marginFrom() { return settings().marginFrom; }
[[nodiscard]] bool gateHeading() { return settings().gateHeading; }
[[nodiscard]] double resonatorPassage() { return settings().resonatorPassage; }
[[nodiscard]] bool slotCrossingRule() { return settings().slotCrossingRule; }
[[nodiscard]] bool mouthTwins() { return settings().mouthTwins; }
[[nodiscard]] bool evenSlots() { return settings().evenSlots; }
[[nodiscard]] double staticGapBonus() { return settings().staticGapBonus; }
[[nodiscard]] double slotReachMargin() { return settings().slotReachMargin; }
[[nodiscard]] bool slotEscape() { return settings().slotEscape; }
[[nodiscard]] std::size_t slotPinCells() { return settings().slotPinCells; }
[[nodiscard]] double slotStubGap() { return settings().slotStubGap; }
[[nodiscard]] bool escapeQuarterTurns() {
  return settings().escapeQuarterTurns;
}
[[nodiscard]] bool ownRunBonus() { return settings().ownRunBonus; }
[[nodiscard]] double escapeStubMargin() {
  return settings().escapeStubMargin;
}
[[nodiscard]] double escapeApartMargin() {
  return settings().escapeApartMargin;
}
[[nodiscard]] bool escapeCouplerRuns() {
  return settings().escapeCouplerRuns;
}
[[nodiscard]] std::int64_t escapeStraight() {
  return settings().escapeStraight;
}

/// The safety factor on the clearance a wire takes in a gap:
/// `SCPD_CAPACITY_MARGIN`, **1.06** (user, 2026-10-18). A gap holds
/// `floor(cells / (clearance * margin))` wires; above 1 the check counts a
/// gap a little narrower than measured, against the error of the grid,
/// which adds up over many wires (57q, cases 03 and 05 of the review: a
/// gate of 115.4 cells for six wires takes five). With the gap counted
/// across the wires' heading (`gateHeading`), 33q of round 54 (cases 10
/// and 12) has a gate of 98.3 cells for five wires that the router does
/// not pass, which takes four from a factor of 1.035 on. At 1.06, 45q of
/// round 55 (cases 06 and 09) has its gates of six wires that the router
/// does not pass take five. Read on every check, so that one process can
/// compare several factors.
[[nodiscard]] double capacityMarginOfEnvironment() {
  constexpr double FALLBACK = 1.06;
  const char* text = std::getenv("SCPD_CAPACITY_MARGIN");
  if (text == nullptr) {
    return FALLBACK;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 1.0 && value <= 2.0 ? value : FALLBACK;
}

/// The least number of wires a gap holds without the margin before the
/// margin applies to it: `SCPD_CAPACITY_MARGIN_FROM`, **3** (user,
/// 2026-10-17). A gap of one or two clearances is measured exactly on the
/// grid, often 19 or 38 cells along an axis, and the margin on it closed
/// eight gaps the routing used (17q and 33q of round 52).
[[nodiscard]] std::int64_t marginFromOfEnvironment() {
  constexpr std::int64_t FALLBACK = 3;
  const char* text = std::getenv("SCPD_CAPACITY_MARGIN_FROM");
  if (text == nullptr) {
    return FALLBACK;
  }
  char* end = nullptr;
  const auto value = std::strtol(text, &end, 10);
  return end != text && value >= 1 && value <= 100 ? value : FALLBACK;
}

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

/// Whether a line's gap is counted across the heading the wires cross it
/// on: `SCPD_GATE_HEADING`, **on** (user, 2026-10-18). A wire runs on one of
/// eight headings and keeps the clearance across its own heading, so
/// parallel wires that cross a line at an angle phi to the line's normal
/// take a clearance over cos(phi) of it each. The best heading lies at most
/// 22.5 degrees off the normal: a line of length L holds
/// floor(L cos(phi) / (clearance * margin)) wires (`headingShare`).
[[nodiscard]] bool gateHeadingOfEnvironment() {
  const char* text = std::getenv("SCPD_GATE_HEADING");
  return text == nullptr || std::string_view(text) != "0";
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

/// The narrowest gap through which a resonator still leaves its own port
/// run, as a share of the clearance: `SCPD_RESONATOR_PASSAGE`, **0.8** (user,
/// 2026-10-18). The check by stretches opens a gate too narrow for any wire
/// to the owner of a port run at its end, with capacity 1 (the own-run
/// passage). A resonator gets through a gap of about 17 cells but not of 14
/// or less (round 55 of the review: 45q, 33q), so a narrower gate keeps
/// capacity 0 for it; the check would rather be too careful than too bold.
[[nodiscard]] double resonatorPassageOfEnvironment() {
  constexpr double FALLBACK = 0.8;
  const char* text = std::getenv("SCPD_RESONATOR_PASSAGE");
  if (text == nullptr) {
    return FALLBACK;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 0.0 && value <= 1.0 ? value : FALLBACK;
}

/// Whether a crossing slot needs a line across its edge that the feedline
/// routing's crossing rule allows: `SCPD_SLOT_CROSSING_RULE`, **on** (user,
/// 2026-10-18). The router lets a wire cross a feedline edge only at a
/// right angle to every straight run within the crossing reach of the
/// cells it enters, and not at all within one cell of a bend or of the
/// first and last ten cells of an edge (`routing::CrossingConstraints`).
/// The slots were laid by the edge's geometry alone; on 33q (round 53,
/// case 11) the check sent wire 12 over a slot nine cells before the end
/// of f2, where the router may not cross, and the feedline routing found
/// no way for it.
[[nodiscard]] bool slotCrossingRuleOfEnvironment() {
  const char* text = std::getenv("SCPD_SLOT_CROSSING_RULE");
  return text == nullptr || std::string_view(text) != "0";
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

/// Whether a gate on the line of a mouth between the same two chambers counts
/// as that mouth: `SCPD_MOUTH_TWINS`, **on** (user, 2026-10-18). The capacity
/// graph can carry a mouth line twice, once as the mouth and once as a plain
/// gate; on 33q (round 54, case 16) the terminal edge f5 passed the mouth
/// between f39 and f0 by its plain twin g141, a way round the chip that the
/// feedline routing, which keeps f5 in its lane, never takes.
[[nodiscard]] bool mouthTwinsOfEnvironment() {
  const char* text = std::getenv("SCPD_MOUTH_TWINS");
  return text == nullptr || std::string_view(text) != "0";
}

/// Whether the crossing slots of a straight piece are spread evenly over
/// it: `SCPD_EVEN_SLOTS`, **on** (user, 2026-10-19). The slots were laid
/// one after the other from one end, so they crowd that end and leave the
/// other free. Spread, each run of places where a slot fits gets as many
/// slots as before, with equal gaps between them and to its two ends; an
/// edge that would get fewer slots keeps the slots as laid. Over the 372
/// cases of rounds 53m to 55m it judged two more right (33q case 15 of
/// 53m, 57q case 10 of 54m) and none wrong.
[[nodiscard]] bool evenSlotsOfEnvironment() {
  const char* text = std::getenv("SCPD_EVEN_SLOTS");
  return text == nullptr || std::string_view(text) != "0";
}

/// How many cells more a gap between a feedline edge and a static obstacle
/// (artwork or the chip's border) takes into account:
/// `SCPD_STATIC_GAP_BONUS`, 0 (user, 2026-10-19). A wire need not keep the
/// whole clearance from a static obstacle, so such a gap holds wires as if
/// it were up to half a clearance wider. A gap between a feedline edge and
/// a port run, a lead or a pad takes no bonus.
[[nodiscard]] double staticGapBonusOfEnvironment() {
  const char* text = std::getenv("SCPD_STATIC_GAP_BONUS");
  if (text == nullptr) {
    return 0.0;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 0.0 && value <= 50.0 ? value : 0.0;
}

/// How many cells further a crossing slot reaches to either side of its
/// edge than the crossing reach: `SCPD_SLOT_REACH_MARGIN`, 0 (user,
/// 2026-10-20). A wire that crosses at a slot runs straight through it, and
/// two wires at neighbouring slots need room to come and go beside the edge;
/// on 45q, chain 7 on options 6 1 5 37, wires 132 and 133 had a slot each
/// and the feedline routing could not lay both.
[[nodiscard]] double slotReachMarginOfEnvironment() {
  const char* text = std::getenv("SCPD_SLOT_REACH_MARGIN");
  if (text == nullptr) {
    return 0.0;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 0.0 && value <= 50.0 ? value : 0.0;
}

/// How many cells at either end of an edge between couplers the crossing
/// rule of the slots closes: `SCPD_SLOT_PIN_CELLS`, 10 as the router's
/// (`routing::CrossingConstraints::PIN_CELLS`), and one cell more around
/// them. Both close the cells of the edge's way and test the line a wire
/// crosses on, so the same figure is the same rule. Fewer let a slot's middle
/// into cells the router may not cross: on 33q, chain 0 on options launcher
/// 6 2 35, five cells put a slot of f2 seven cells before its end, and wire
/// 12 found no way across f2.
[[nodiscard]] std::size_t slotPinCellsOfEnvironment() {
  const char* text = std::getenv("SCPD_SLOT_PIN_CELLS");
  if (text == nullptr) {
    return SLOT_PIN_CELLS;
  }
  char* end = nullptr;
  const auto value = std::strtol(text, &end, 10);
  return end != text && value >= 0 && value <= 50
             ? static_cast<std::size_t>(value)
             : SLOT_PIN_CELLS;
}

/// How many cells nearer a stub a crossing slot may lie along its edge:
/// `SCPD_SLOT_STUB_GAP`, 0 (user, 2026-10-20). A slot keeps every cell of its
/// rectangle out of the band of a port run and off a stub's wall, so its
/// middle stays a clearance from the stub's line along the edge; a gap of
/// `g` checks the stubs over the middle part of the rectangle alone, `g`
/// cells shorter at either end. On 21q, chain 2 on the chip as begun, the
/// port run at the upper coupler of f11 left one slot on it, and wires 27
/// and 28 both crossed f11 in the rip-up.
[[nodiscard]] double slotStubGapOfEnvironment() {
  const char* text = std::getenv("SCPD_SLOT_STUB_GAP");
  if (text == nullptr) {
    return 0.0;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 0.0 && value <= 50.0 ? value : 0.0;
}

/// How many cells a way out of a crossing slot runs straight after its turn:
/// `SCPD_ESCAPE_STRAIGHT`, 5 (user, 2026-10-20).
[[nodiscard]] std::int64_t escapeStraightOfEnvironment() {
  const char* text = std::getenv("SCPD_ESCAPE_STRAIGHT");
  if (text == nullptr) {
    return 5;
  }
  char* end = nullptr;
  const auto value = std::strtol(text, &end, 10);
  return end != text && value >= 0 && value <= 200 ? value : 5;
}

/// How many cells more than a clearance a way out of a crossing slot keeps
/// from the line of a stub: `SCPD_ESCAPE_STUB_MARGIN`, **4** (user,
/// 2026-10-21). On 45q, chain 7 on options 6 1 2 37, wires 132 and 133 came
/// to the two slots of f46 from one side and ran a clearance apart there,
/// which the feedline routing does not keep; from 4 cells on, the second
/// slot has no way out and the check finds the two lost. On 21q it costs one
/// run with a clean rip-up.
[[nodiscard]] double escapeStubMarginOfEnvironment() {
  const char* text = std::getenv("SCPD_ESCAPE_STUB_MARGIN");
  if (text == nullptr) {
    return ESCAPE_STUB_MARGIN;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 0.0 && value <= 50.0 ? value
                                                      : ESCAPE_STUB_MARGIN;
}

/// How many cells more than a clearance two ways out of crossing slots keep
/// apart: `SCPD_ESCAPE_APART_MARGIN`, 0 (user, 2026-10-21).
[[nodiscard]] double escapeApartMarginOfEnvironment() {
  const char* text = std::getenv("SCPD_ESCAPE_APART_MARGIN");
  if (text == nullptr) {
    return 0.0;
  }
  char* end = nullptr;
  const auto value = std::strtod(text, &end);
  return end != text && value >= 0.0 && value <= 50.0 ? value : 0.0;
}

/// Whether a gate with an end on a wire's own port run holds that wire half a
/// clearance more, since the wire keeps no clearance from its own run:
/// `SCPD_OWN_RUN_BONUS`, off (user, 2026-10-21). On 69q, chain 9 on options
/// 0 0 0 32, it made g222 at the target run of 185 hold five wires where its
/// 94 cells across the heading hold four.
[[nodiscard]] bool ownRunBonusOfEnvironment() {
  const char* text = std::getenv("SCPD_OWN_RUN_BONUS");
  return text != nullptr && std::string_view(text) == "1";
}

/// Whether a way out of a crossing slot keeps a clearance from the run of a
/// feedline through every coupler (`couplerRunsOf`): `SCPD_ESCAPE_COUPLER_RUNS`,
/// off (user, 2026-10-20). On 45q, chain 7 on options 6 1 2 37, it took the
/// second slot of f46 and made the check find wires 132 and 133 lost, as the
/// rip-up did; on 21q it took slots that the rip-up crossed at, and 49 runs
/// against 4 came out UNSAT with a clean rip-up.
[[nodiscard]] bool escapeCouplerRunsOfEnvironment() {
  const char* text = std::getenv("SCPD_ESCAPE_COUPLER_RUNS");
  return text != nullptr && std::string_view(text) == "1";
}

/// Whether a way out of a crossing slot may be a quarter turn:
/// `SCPD_ESCAPE_QUARTER_TURNS`, on (user, 2026-10-20). `=0` leaves the
/// eighth turns alone, which keep a way out nearer the heading across the
/// edge.
[[nodiscard]] bool escapeQuarterTurnsOfEnvironment() {
  const char* text = std::getenv("SCPD_ESCAPE_QUARTER_TURNS");
  return text == nullptr || std::string_view(text) != "0";
}

/// Whether two crossing slots whose wires cannot both get away from them
/// take one wire between them: `SCPD_SLOT_ESCAPE`, **on** (user, 2026-10-20).
/// A wire leaves a slot at either end of its crossing straight and turns
/// there; its ways out are the router's turns (`routing::MovePrimitives`,
/// the router's bend radius) with `SCPD_ESCAPE_STRAIGHT` cells after them, every
/// cell free and a clearance from every stub. A slot goes only where each
/// of its ends has a way out a clearance from the ways out of the slots
/// placed before (`SlotPlacer`). Two slots whose ends lie near each other
/// and that have no pair of ways out a clearance apart take at most one
/// wire between them (`escapeLimits`). On 45q, chain 7 on options 6 1 5 37,
/// wires 132 and 133 had a slot each and the feedline routing could not lay
/// both. `=0` places the slots by the edge alone and sets no limit.
[[nodiscard]] bool slotEscapeOfEnvironment() {
  const char* text = std::getenv("SCPD_SLOT_ESCAPE");
  return text == nullptr || std::string_view(text) != "0";
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
          : std::clamp((((p.x - a.x) * dx) + ((p.y - a.y) * dy)) / length,
                       0.0, 1.0);
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
  const auto numberOf = [](const std::string& name) -> std::optional<std::int64_t> {
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
    if (const auto k = numberOf(wire.name); k.has_value() && !ends.contains(*k)) {
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

/// The ways out of the ends of crossing slots (`SCPD_SLOT_ESCAPE`). An end
/// lies the crossing reach across the edge from the slot's point. A way
/// counts when every cell of it is free and a clearance from the line of
/// every stub, as the router keeps a wire from a resonator's lead and a port
/// run. The raster holds a port run as the two walls of its band, half a
/// clearance to either side of its line, so a way keeps the clearance less
/// that half from the walls; a full clearance from the walls kept it one and
/// a half from the line, and on 21q f11 lost the slot that wire 28 crossed
/// at in the rip-up. Under `SCPD_ESCAPE_COUPLER_RUNS` a way also keeps a
/// clearance from the run of a feedline through every coupler.
class EscapeWays {
public:
  /// A way out that a placed slot takes, and the end it leaves.
  struct Taken {
    Point end;
    EscapeWay way;
  };
  /// The ways out of the two ends of a slot, the end to the left of its
  /// direction first.
  using Both = std::array<std::vector<EscapeWay>, 2>;

  EscapeWays(const SlotGraph& graph, const Grid& grid)
      : grid_(grid), clear_(graph.clearance),
        stubClear_(std::max(0.0, graph.clearance - graph.inflation) +
                   escapeStubMargin()),
        reach_(graph.crossingReach + slotReachMargin()),
        bucket_(static_cast<std::int64_t>(std::ceil(clear_))) {
    if (grid.stubCells != nullptr) {
      for (const auto cell : *grid.stubCells) {
        const auto x = static_cast<std::int64_t>(cell) % grid.width;
        const auto y = static_cast<std::int64_t>(cell) / grid.width;
        stubsNear_[keyOf(x / bucket_, y / bucket_)].emplace_back(x, y);
      }
    }
    if (escapeCouplerRuns()) {
      runs_ = couplerRunsOf(graph);
    }
  }

  /// Two ends this near can have ways out within a clearance of each other.
  [[nodiscard]] double near() const {
    return clear_ + escapeApartMargin() +
           (2.0 * ((2.0 * ESCAPE_RADIUS) +
                   static_cast<double>(escapeStraight()) + 2.0));
  }

  /// The end of a slot on one side of its edge: 0 left of `direction`, 1
  /// right of it.
  [[nodiscard]] Point endOf(const Point& at, const Point& direction,
                            const std::size_t side) const {
    const auto sign = side == 0 ? 1.0 : -1.0;
    return {at.x - (direction.y * sign * reach_),
            at.y + (direction.x * sign * reach_)};
  }

  /// Every way out of the two ends of a slot.
  [[nodiscard]] Both waysOf(const Point& at, const Point& direction) const {
    Both both;
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
        EscapeWay way;
        bool free = true;
        for (const auto& offset : move.swept) {
          const Cell cell{x0 + offset.dx, y0 + offset.dy};
          free = free && freeAt(cell.first, cell.second);
          way.cells.push_back(cell);
        }
        const auto onward = routing::headingVector(move.exitHeading);
        for (std::int64_t step = 1; free && step <= escapeStraight(); ++step) {
          const Cell cell{x0 + move.dx + (step * onward.dx),
                          y0 + move.dy + (step * onward.dy)};
          free = freeAt(cell.first, cell.second);
          way.cells.push_back(cell);
        }
        if (!free) {
          continue;
        }
        for (const auto& sample : move.samples) {
          way.line.push_back({static_cast<double>(x0) + sample.x(),
                              static_cast<double>(y0) + sample.y()});
        }
        way.line.push_back(
            {static_cast<double>(x0 + move.dx + (escapeStraight() * onward.dx)),
             static_cast<double>(y0 + move.dy +
                                 (escapeStraight() * onward.dy))});
        both.at(side).push_back(std::move(way));
      }
    }
    return both;
  }

  /// Whether every cell of one way lies a clearance, and
  /// `SCPD_ESCAPE_APART_MARGIN` cells more, from every cell of the other.
  [[nodiscard]] bool apart(const EscapeWay& a, const EscapeWay& b) const {
    const auto keep = clear_ + escapeApartMargin();
    for (const auto& [ax, ay] : a.cells) {
      for (const auto& [bx, by] : b.cells) {
        const auto dx = static_cast<double>(ax - bx);
        const auto dy = static_cast<double>(ay - by);
        if ((dx * dx) + (dy * dy) < keep * keep) {
          return false;
        }
      }
    }
    return true;
  }

  /// For each end of a slot the first of its ways out that keeps a clearance
  /// from every way taken at an end near it; nothing when an end has none.
  [[nodiscard]] std::optional<std::vector<EscapeWay>>
  choose(const Slot& slot, const Both& ways,
         const std::initializer_list<const std::vector<Taken>*> taken) const {
    std::vector<EscapeWay> chosen;
    for (std::size_t side = 0; side < 2; ++side) {
      const auto end = endOf(slot.at, slot.direction, side);
      const auto fits = [&](const EscapeWay& way) {
        for (const auto* list : taken) {
          for (const auto& other : *list) {
            if (distance(other.end, end) <= near() &&
                !apart(other.way, way)) {
              return false;
            }
          }
        }
        return true;
      };
      const auto found = std::ranges::find_if(ways.at(side), fits);
      if (found == ways.at(side).end()) {
        return std::nullopt;
      }
      chosen.push_back(*found);
    }
    return chosen;
  }

  /// The ways a placed slot takes, added to a list.
  void take(const Slot& slot, std::vector<Taken>& into) const {
    for (std::size_t side = 0; side < slot.ways.size(); ++side) {
      into.push_back(
          {.end = endOf(slot.at, slot.direction, side), .way = slot.ways[side]});
    }
  }

private:
  [[nodiscard]] static const routing::MovePrimitives& primitives() {
    static const routing::MovePrimitives table(ESCAPE_RADIUS);
    return table;
  }
  [[nodiscard]] std::int64_t keyOf(const std::int64_t bx,
                                   const std::int64_t by) const {
    return (by * (grid_.width + 1)) + bx;
  }
  /// Whether a cell lies within a clearance of the run of a feedline
  /// through a coupler.
  [[nodiscard]] bool nearRun(const std::int64_t x, const std::int64_t y) const {
    const Point cell{static_cast<double>(x), static_cast<double>(y)};
    return std::ranges::any_of(runs_, [&](const auto& run) {
      return distanceToSegment(cell, run[0], run[1]) < clear_;
    });
  }

  /// Whether a cell lies nearer than `radius` to a cell of `near`.
  [[nodiscard]] bool within(
      const std::unordered_map<std::int64_t, std::vector<Cell>>& near,
      const std::int64_t x, const std::int64_t y, const double radius) const {
    for (std::int64_t by = (y / bucket_) - 1; by <= (y / bucket_) + 1; ++by) {
      for (std::int64_t bx = (x / bucket_) - 1; bx <= (x / bucket_) + 1;
           ++bx) {
        const auto found = near.find(keyOf(bx, by));
        if (found == near.end()) {
          continue;
        }
        for (const auto& [sx, sy] : found->second) {
          const auto dx = static_cast<double>(sx - x);
          const auto dy = static_cast<double>(sy - y);
          if ((dx * dx) + (dy * dy) < radius * radius) {
            return true;
          }
        }
      }
    }
    return false;
  }
  [[nodiscard]] bool freeAt(const std::int64_t x, const std::int64_t y) const {
    if (!grid_.inside(x, y)) {
      return false;
    }
    const auto value = grid_.at(x, y);
    return (value == 0 || value >= CHAMBER) &&
           !within(stubsNear_, x, y, stubClear_) && !nearRun(x, y);
  }

  const Grid& grid_;
  double clear_;
  /// How far a way out keeps from the wall of a stub.
  double stubClear_;
  double reach_;
  /// The stubs' cells by square of a clearance.
  std::int64_t bucket_;
  std::unordered_map<std::int64_t, std::vector<Cell>> stubsNear_;
  /// The runs of the feedlines through the couplers.
  std::vector<std::array<Point, 2>> runs_;
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

/// Read the switches for one check.
void readSettings() {
  settings() = Settings{.margin = capacityMarginOfEnvironment(),
                        .marginFrom = marginFromOfEnvironment(),
                        .gateHeading = gateHeadingOfEnvironment(),
                        .resonatorPassage = resonatorPassageOfEnvironment(),
                        .slotCrossingRule = slotCrossingRuleOfEnvironment(),
                        .mouthTwins = mouthTwinsOfEnvironment(),
                        .evenSlots = evenSlotsOfEnvironment(),
                        .staticGapBonus = staticGapBonusOfEnvironment(),
                        .slotReachMargin = slotReachMarginOfEnvironment(),
                        .slotEscape = slotEscapeOfEnvironment(),
                        .slotPinCells = slotPinCellsOfEnvironment(),
                        .slotStubGap = slotStubGapOfEnvironment(),
                        .escapeQuarterTurns =
                            escapeQuarterTurnsOfEnvironment(),
                        .escapeStraight = escapeStraightOfEnvironment(),
                        .escapeCouplerRuns = escapeCouplerRunsOfEnvironment(),
                        .ownRunBonus = ownRunBonusOfEnvironment(),
                        .escapeApartMargin = escapeApartMarginOfEnvironment(),
                        .escapeStubMargin = escapeStubMarginOfEnvironment()};
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

/// The crossing slots of every feedline edge of the graph. Slots are laid
/// from either end of the edge, each at the first place past the one
/// before where it fits; the end that gives more slots counts. Under
/// `SCPD_EVEN_SLOTS` the same number is spread evenly (`placeEvenly`).
/// Under `SCPD_SLOT_ESCAPE` a slot fits a place only when each of its ends
/// has a way out a clearance from the ways out of the slots placed before
/// (`EscapeWays::choose`), so that a slot moves on past places where its
/// wire could not get away.
struct SlotPlacer {
  const SlotGraph& graph;
  const Grid& grid;
  Stamps& stamps;
  double clearance = 0.0;
  double reach = 0.0;
  double along = 0.0;
  double inflation = 0.0;
  int feedlineCode = FEEDLINE;
  int stubCode = 0;
  std::vector<std::string> wallKinds;
  std::vector<bool> portSlots;
  std::set<std::pair<std::int64_t, std::int64_t>> portSlotsOutside;
  /// The feedline routing's crossing rule over the edges between couplers
  /// (`slotCrossingRule`), and how far it reaches.
  routing::CrossingConstraints rule;
  int ruleReach = 0;
  /// The ways out of the slots (`SCPD_SLOT_ESCAPE`), and those the slots of
  /// the edges placed so far take.
  std::optional<EscapeWays> escapes;
  std::vector<EscapeWays::Taken> taken;

  SlotPlacer(const SlotGraph& graphIn, const Grid& gridIn, Stamps& stampsIn)
      : graph(graphIn), grid(gridIn), stamps(stampsIn),
        clearance(graphIn.clearance),
        reach(graphIn.crossingReach + slotReachMargin()),
        along(clearance), inflation(graphIn.inflation),
        wallKinds(graphIn.wallKinds), portSlots(gridIn.size, false) {
    if (slotEscape()) {
      escapes.emplace(graphIn, gridIn);
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
      if (grid.inside(x, y)) {
        portSlots[grid.cell(x, y)] = true;
      } else {
        portSlotsOutside.emplace(x, y);
      }
    }
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
    std::vector<Refused> refused;
    double length = 0.0;
  };

  /// What a slot centred on point k of a line comes to, other slots aside:
  /// no slot (`skip`: no straight run of a slot's length there), a refused
  /// place, or a slot and the cells it covers.
  struct Examined {
    enum class Kind { Skip, Refused, Fits } kind = Kind::Skip;
    std::string why;
    Slot slot{};
    std::vector<std::pair<std::int64_t, std::int64_t>> covered;
  };

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
    out.slot = {line[here], corners, {ux, uy}};
    out.kind = Examined::Kind::Refused;
    if (!rule.empty() && !crossable(cx, cy, *step)) {
      out.why = "crossing rule";
      return out;
    }
    auto& covered = out.covered;
    for (double t = -half; t <= half + 1e-9; t += 0.5) {
      for (double s = -reach; s <= reach + 1e-9; s += 0.5) {
        const std::pair cell{rounded(cx + (t * ux) + (s * nx)),
                             rounded(cy + (t * uy) + (s * ny))};
        // Half-cell samples fall on the same cell twice in a row; once is
        // enough, and the order of the first ones stays.
        if (covered.empty() || covered.back() != cell) {
          covered.push_back(cell);
        }
      }
    }

    // The edge's own straight piece: the points of the edge within
    // OWN_WINDOW of the slot's.
    const auto first =
        static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, k - OWN_WINDOW));
    const auto stop =
        static_cast<std::size_t>(std::min(count, k + OWN_WINDOW + 1));
    const auto ownReach = (inflation + 1.5) * (inflation + 1.5);
    // Stubs count over the middle of the rectangle alone
    // (`SCPD_SLOT_STUB_GAP`).
    const auto stubAlong = half - slotStubGap() + 1e-9;
    for (const auto& [x, y] : covered) {
      const bool in = grid.inside(x, y);
      const bool stubFree =
          std::abs(((static_cast<double>(x) - cx) * ux) +
                   ((static_cast<double>(y) - cy) * uy)) > stubAlong;
      if (!stubFree && (in ? portSlots[grid.cell(x, y)]
                           : portSlotsOutside.contains({x, y}))) {
        out.why = "slot of a port run";
        return out;
      }
      const int value = in ? std::min(grid.at(x, y), CHAMBER) : 1;
      if (value == CHAMBER || value == 0 ||
          (value == stubCode && stubFree)) {
        continue;
      }
      // The edge's own wall only of the straight piece the slot sits on,
      // not of another arm of the edge bent back beside it.
      if (value == feedlineCode) {
        // From the slot's own point outward: a cell of the slot is near
        // a point close to it along the edge, so the search stops soon.
        bool near = false;
        const auto nearPoint = [&](const std::size_t p) {
          const auto dx = static_cast<double>(x) - line[p].x;
          const auto dy = static_cast<double>(y) - line[p].y;
          return (dx * dx) + (dy * dy) <= ownReach;
        };
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
  /// the one before where it fits.
  [[nodiscard]] Placement place(const std::vector<Point>& line) {
    Placement result;
    const auto mark = stamps.fresh(1);
    std::vector<EscapeWays::Taken> mine;
    double last = -1e9;
    const auto distanceAt = distancesAlong(line);
    for (std::size_t here = 0; here < line.size(); ++here) {
      if (distanceAt[here] - last < along - 1e-9) {
        continue;
      }
      auto found = examine(line, here);
      if (found.kind == Examined::Kind::Skip) {
        continue;
      }
      if (found.kind == Examined::Kind::Fits &&
          std::ranges::any_of(found.covered, [&](const auto& cell) {
            return stamps[grid.cell(cell.first, cell.second)] == mark;
          })) {
        found.kind = Examined::Kind::Refused;
        found.why = "another slot";
      }
      if (found.kind == Examined::Kind::Fits && escapes.has_value()) {
        auto ways = escapes->choose(
            found.slot,
            escapes->waysOf(found.slot.at, found.slot.direction),
            {&taken, &mine});
        if (ways.has_value()) {
          found.slot.ways = std::move(*ways);
          escapes->take(found.slot, mine);
        } else {
          found.kind = Examined::Kind::Refused;
          found.why = "no way out";
        }
      }
      if (found.kind == Examined::Kind::Refused) {
        result.refused.push_back(
            {line[here], std::move(found.why), found.slot.corners});
        continue;
      }
      result.placed.push_back(found.slot);
      for (const auto& [x, y] : found.covered) {
        stamps[grid.cell(x, y)] = mark;
      }
      last = distanceAt[here];
    }
    result.length = distanceAt.back();
    return result;
  }

  /// Slots spread evenly (`SCPD_EVEN_SLOTS`): on every run of points where a
  /// slot fits, one after the other on one straight piece, as many slots as
  /// laid one after the other there, with equal gaps between them and to
  /// the two ends of the room they cover. Nothing when the line gets fewer
  /// than `wanted` slots this way.
  [[nodiscard]] std::optional<Placement>
  placeEvenly(const std::vector<Point>& line, const std::size_t wanted) {
    Placement result;
    const auto distanceAt = distancesAlong(line);
    result.length = distanceAt.back();
    std::vector<Examined> found;
    found.reserve(line.size());
    for (std::size_t here = 0; here < line.size(); ++here) {
      found.push_back(examine(line, here));
      if (found.back().kind == Examined::Kind::Refused) {
        result.refused.push_back(
            {line[here], found.back().why, found.back().slot.corners});
      }
    }
    // The runs: consecutive points where a slot fits, with one direction.
    std::vector<std::pair<std::size_t, std::size_t>> runs;
    for (std::size_t here = 0; here < line.size(); ++here) {
      if (found[here].kind != Examined::Kind::Fits) {
        continue;
      }
      const auto& direction = found[here].slot.direction;
      if (!runs.empty() && runs.back().second + 1 == here) {
        const auto& before = found[runs.back().second].slot.direction;
        if (before.x == direction.x && before.y == direction.y) {
          runs.back().second = here;
          continue;
        }
      }
      runs.emplace_back(here, here);
    }
    const auto mark = stamps.fresh(1);
    using Cells = std::set<Cell>;
    // The slots of one try at a run: their points, their cells and, under
    // `SCPD_SLOT_ESCAPE`, the ways out they take.
    struct Try {
      std::vector<std::size_t> at;
      Cells cells;
      std::vector<EscapeWays::Taken> taken;
      std::map<std::size_t, std::vector<EscapeWay>> ways;
    };
    // The ways out of the slots of this line's earlier runs, and of the
    // slot at every point, found once.
    std::vector<EscapeWays::Taken> mine;
    std::map<std::size_t, EscapeWays::Both> waysAt;
    // Whether the slot at a point shares no cell with the slots placed
    // before this run or with the try's, and gets away from them; it joins
    // the try.
    const auto free = [&](const std::size_t point, Try& tried) {
      // A slot's own samples may meet a cell twice; only another slot's
      // cell is in the way.
      const Cells own(found[point].covered.begin(),
                      found[point].covered.end());
      for (const auto& cell : own) {
        if (stamps[grid.cell(cell.first, cell.second)] == mark ||
            tried.cells.contains(cell)) {
          return false;
        }
      }
      if (escapes.has_value()) {
        const auto& slot = found[point].slot;
        auto known = waysAt.find(point);
        if (known == waysAt.end()) {
          known = waysAt
                      .emplace(point, escapes->waysOf(slot.at, slot.direction))
                      .first;
        }
        auto ways =
            escapes->choose(slot, known->second, {&taken, &mine, &tried.taken});
        if (!ways.has_value()) {
          return false;
        }
        Slot placed = slot;
        placed.ways = std::move(*ways);
        escapes->take(placed, tried.taken);
        tried.ways[point] = std::move(placed.ways);
      }
      tried.cells.insert(own.begin(), own.end());
      tried.at.push_back(point);
      return true;
    };
    // The slots of a run laid one after the other from its start.
    const auto packed = [&](const std::size_t from, const std::size_t to) {
      Try tried;
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
      Try tried;
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
        result.placed.push_back(found[point].slot);
        result.placed.back().ways = std::move(tried.ways[point]);
        for (const auto& [x, y] : found[point].covered) {
          stamps[grid.cell(x, y)] = mark;
        }
      }
      mine.insert(mine.end(), tried.taken.begin(), tried.taken.end());
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
    for (int t = -(ruleReach + 1); t <= ruleReach + 1; ++t) {
      const auto x = static_cast<std::int64_t>(std::lround(cx)) + (t * dx);
      const auto y = static_cast<std::int64_t>(std::lround(cy)) + (t * dy);
      if (x < 0 || y < 0 || !grid.inside(x, y) ||
          !rule.allowed(static_cast<std::uint32_t>(x),
                        static_cast<std::uint32_t>(y), *across)) {
        return false;
      }
    }
    return true;
  }

  /// The router's crossing rule, rebuilt from the feedline strokes of the
  /// graph as the feedline routing builds it: every edge between couplers,
  /// the terminal edges left out (`SCPD_FEEDLINE_PROTOTYPE`, on), with the
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
          cell.heading = headingOfStep(
                             signOf(static_cast<double>(x) - way.back().x),
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
      auto forward = place(stroke.points);
      const std::vector<Point> reversed(stroke.points.rbegin(),
                                        stroke.points.rend());
      auto backward = place(reversed);
      auto& greedy =
          backward.placed.size() > forward.placed.size() ? backward : forward;
      std::optional<Placement> even;
      if (evenSlots()) {
        even = placeEvenly(stroke.points, greedy.placed.size());
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
};

/// The limits the ways out of the crossing slots set (`slotEscape`): for
/// every two slots with an end of the one near an end of the other, and no
/// pair of ways out of those ends a clearance apart, a group of the two that
/// takes one wire. An end with no way out limits nothing; the placement
/// gives every slot a way out at each end.
[[nodiscard]] std::vector<Group> escapeLimits(const SlotGraph& graph,
                                              const Grid& grid,
                                              const std::vector<Edge>& edges,
                                              json* show = nullptr) {
  const EscapeWays escapes(graph, grid);
  struct End {
    std::size_t slot;
    std::size_t side;
    Point at;
    std::vector<EscapeWay> ways;
  };
  std::vector<End> ends;
  for (std::size_t k = 0; k < edges.size(); ++k) {
    const auto& edge = edges[k];
    if (!edge.stretch || edge.corners.empty()) {
      continue;
    }
    auto both = escapes.waysOf(edge.at, edge.direction);
    for (std::size_t side = 0; side < 2; ++side) {
      ends.push_back({.slot = k,
                      .side = side,
                      .at = escapes.endOf(edge.at, edge.direction, side),
                      .ways = std::move(both.at(side))});
    }
  }
  std::set<std::pair<std::size_t, std::size_t>> limited;
  std::vector<Group> groups;
  for (std::size_t i = 0; i < ends.size(); ++i) {
    for (std::size_t j = i + 1; j < ends.size(); ++j) {
      const auto& a = ends[i];
      const auto& b = ends[j];
      if (a.slot == b.slot || a.ways.empty() || b.ways.empty() ||
          distance(a.at, b.at) > escapes.near()) {
        continue;
      }
      const auto pair = std::minmax(a.slot, b.slot);
      if (limited.contains(pair)) {
        continue;
      }
      // As long as one way out of the one end and one of the other keep a
      // clearance apart, the two wires get away together.
      const bool together = std::ranges::any_of(a.ways, [&](const auto& wa) {
        return std::ranges::any_of(b.ways, [&](const auto& wb) {
          return escapes.apart(wa, wb);
        });
      });
      if (together) {
        continue;
      }
      limited.insert(pair);
      groups.push_back(
          {.slots = {pair.first, pair.second},
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
        for (const auto& [x, y] : chosen[end.side].cells) {
          cells.push_back(json::array({x, y}));
        }
        line = json::array();
        for (const auto& point : chosen[end.side].line) {
          line.push_back(realJson(point));
        }
      }
      show->push_back({{"slot", edges[end.slot].name},
                       {"at", json::array({end.at.x, end.at.y})},
                       {"ways", end.ways.size()},
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
  const auto near = (6 * clear) + (2 * r) + 2.0;
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
    std::unordered_map<std::size_t, std::size_t> at;
    std::unordered_map<std::size_t, std::array<Point, 2>> exits;
    for (const auto k : mine) {
      const auto& c0 = edges[k].at;
      std::size_t closest = 0;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t p = 0; p < n; ++p) {
        const auto apart = distance(pts[p], c0);
        if (apart < best) {
          best = apart;
          closest = p;
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
    // The points of the edge by square of side `near`: a pair of points
    // closer than `near` lies in one square or in two that touch.
    const auto bucketOf = [near](const Point& point) {
      return std::pair{static_cast<std::int64_t>(std::floor(point.x / near)),
                       static_cast<std::int64_t>(std::floor(point.y / near))};
    };
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>>
        buckets;
    for (std::size_t p = 0; p < n; ++p) {
      buckets[bucketOf(pts[p])].push_back(p);
    }
    std::vector<std::size_t> partners;
    std::vector<Point> polygon;
    for (std::size_t i = 0; i < n; ++i) {
      partners.clear();
      const auto [bx, by] = bucketOf(pts[i]);
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          const auto found = buckets.find({bx + dx, by + dy});
          if (found == buckets.end()) {
            continue;
          }
          for (const auto j : found->second) {
            if (j > i) {
              partners.push_back(j);
            }
          }
        }
      }
      std::ranges::sort(partners);
      for (const auto j : partners) {
        const auto d = distance(pts[j], pts[i]);
        if (!(along[j] - along[i] >= 1.5 * std::max(d, 1.0) &&
              d - (2 * r) < 6 * clear)) {
          continue;
        }
        const auto gap = (d - (2 * r)) * headingShare(pts[i], pts[j]);
        const auto capacity = wiresIn(gap, clear, margin);
        // Only the slots between the two places can lie past the line.
        const auto between = std::ranges::count_if(
            mine, [&](const auto k) { return i < at[k] && at[k] < j; });
        if (between < 1 || between <= capacity) {
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
              double closest = std::numeric_limits<double>::infinity();
              for (const auto& point : pts) {
                closest = std::min(closest, distance(point, q));
              }
              ownWall = closest <= r + 1.5;
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

/// The bottlenecks between a crossing slot and the open end of a stub: a
/// free line from a corner of a slot's rectangle to the end of a stub's
/// wall that closes a room. A wire that starts in the room and a wire that
/// crosses at a slot whose exit lies in the room both pass the line.
[[nodiscard]] std::vector<Group>
slotStubBottlenecks(const SlotGraph& graph, const Grid& grid, Stamps& stamps,
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

  // The free cells on the closed side of the line a-b, or nothing. The
  // room is marked in `stamps` with the returned generation.
  std::vector<std::size_t> queue;
  std::array<std::vector<std::size_t>, 2> found;
  const auto roomOf = [&](const Point& a,
                          const Point& b) -> std::optional<std::uint16_t> {
    const auto span = std::hypot(b.x - a.x, b.y - a.y);
    const auto count = std::max<std::int64_t>(
        2, static_cast<std::int64_t>(std::ceil(span)) * 2);
    const auto norm = std::max(1e-9, span);
    const Point u{(b.x - a.x) / norm, (b.y - a.y) / norm};
    const auto wall = stamps.fresh(4);
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
          stamps[grid.cell(x + dx, y + dy)] = wall;
        }
      }
    }
    const Point mid{(a.x + b.x) / 2, (a.y + b.y) / 2};
    const Point normal{-u.y, u.x};
    std::array<bool, 2> closed{false, false};
    for (std::size_t which = 0; which < 2; ++which) {
      const double sign = which == 0 ? 3.0 : -3.0;
      const auto seen = static_cast<std::uint16_t>(wall + 1 + which);
      found[which].clear();
      const auto x0 = rounded(mid.x + (sign * normal.x));
      const auto y0 = rounded(mid.y + (sign * normal.y));
      if (!grid.inside(x0, y0) || !grid.open(x0, y0) ||
          stamps[grid.cell(x0, y0)] == wall) {
        continue;
      }
      stamps[grid.cell(x0, y0)] = seen;
      found[which].push_back(grid.cell(x0, y0));
      queue.assign(1, grid.cell(x0, y0));
      std::size_t head = 0;
      // The second side is the room only when it closes smaller than a
      // first side that closed: past that size its fill can stop.
      const auto stopAt = which == 1 && closed[0] ? found[0].size() : limit + 1;
      while (head < queue.size() && found[which].size() <= limit &&
             found[which].size() < stopAt) {
        const auto cell = queue[head++];
        const auto x = static_cast<std::int64_t>(cell) % grid.width;
        const auto y = static_cast<std::int64_t>(cell) / grid.width;
        for (const auto& [nx, ny] :
             std::array<std::pair<std::int64_t, std::int64_t>, 4>{
                 {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}}}) {
          if (!grid.inside(nx, ny)) {
            continue;
          }
          const auto next = grid.cell(nx, ny);
          if (stamps[next] == seen || stamps[next] == wall ||
              !grid.open(nx, ny)) {
            continue;
          }
          stamps[next] = seen;
          found[which].push_back(next);
          queue.push_back(next);
        }
      }
      closed[which] = found[which].size() <= limit &&
                      found[which].size() < stopAt && head >= queue.size();
    }
    std::optional<std::size_t> room;
    for (std::size_t which = 0; which < 2; ++which) {
      if (closed[which] &&
          (!room.has_value() || found[which].size() < found[*room].size())) {
        room = which;
      }
    }
    if (!room.has_value()) {
      return std::nullopt;
    }
    const auto mark = static_cast<std::uint16_t>(wall + 3);
    for (const auto cell : found[*room]) {
      stamps[cell] = mark;
    }
    return mark;
  };

  std::vector<Group> groups;
  std::map<std::pair<std::vector<std::size_t>, std::vector<std::string>>,
           std::size_t>
      index;
  for (const auto k : slots) {
    for (const auto& a : edges[k].corners) {
      for (const auto& b : tips) {
        const auto length = distance(b, a);
        if (!(length <= longest)) {
          continue;
        }
        auto capacity =
            wiresIn(length * headingShare(a, b), clear, margin);
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
          return grid.inside(x, y) && stamps[grid.cell(x, y)] == *room;
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
      : graph_(graph), page_(page), grid_(gridOf(graph)), stamps_(grid_.size),
        clear_(graph.clearance), margin_(capacityMargin()) {}

  /// The verdict, the lost wires and the edges over, and the graph as the
  /// check saw it.
  std::pair<SlotCheckResult, json> run();

private:
  const SlotGraph& graph_;
  /// The graph as JSON when the graph for the page is wanted, or nothing.
  const json* page_;
  Grid grid_;
  Stamps stamps_;
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
  /// The search's marks per state, reset by a new generation.
  mutable std::vector<std::uint32_t> seen_;
  mutable std::uint32_t generation_ = 0;
  mutable std::vector<double> bestD_;
  mutable std::vector<std::uint64_t> bestFrom_;
  mutable std::vector<double> bestStep_;
  std::vector<std::set<std::string>> owners_;
  std::vector<std::int64_t> capacity_;
  /// Per edge, what it holds with none, one and two users at their own
  /// run (`limitFor`): the negotiation asks for it per wire and per round.
  std::vector<std::array<std::int64_t, 3>> limits_;
  /// Per mouth gate: the mouth it belongs to, or nothing.
  std::unordered_map<std::size_t, std::optional<std::string>> mouthOfGate_;
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
    strokes_.push_back(
        {.name = stroke.name, .kind = stroke.kind, .points = std::move(points)});
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
      gateLines.emplace_back(
          edge.name, std::pair{Point{edge.from[0], edge.from[1]},
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
        std::ranges::all_of(*one.users, [&](const std::string& name) {
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
          wiresIn((one.length * headingShare(one.from, one.to)) + bonus,
                  clear_, margin_);
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
      const bool sameLine = (near(edge.from, mouth.from) &&
                             near(edge.to, mouth.to)) ||
                            (near(edge.from, mouth.to) &&
                             near(edge.to, mouth.from));
      if (sameLine && chambers == std::set<std::int64_t>(mouth.chambers.begin(),
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
  // clearance more: the wire keeps no clearance from its own run
  // (`SCPD_OWN_RUN_BONUS`).
  const auto margins =
      ((2 * clear_) / 2) -
      ((static_cast<double>(std::min<std::int64_t>(own, 2)) * clear_) / 2);
  const auto room =
      (edges_[k].length * headingShare(edges_[k].from, edges_[k].to)) -
      margins;
  return std::max(capacity_[k],
                  room >= 0 ? static_cast<std::int64_t>(
                                  std::floor(room / (clear_ * margin_))) +
                                  1
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
  for (std::size_t k = 0; k < ne; ++k) {
    const auto& edge = edges_[k];
    bool ok = !edge.users.has_value() ||
              std::ranges::find(*edge.users, demand.name) != edge.users->end();
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
    const auto gate = mouthOfGate_.find(k);
    const auto mouthOf = gate == mouthOfGate_.end()
                             ? std::optional<std::string>()
                             : gate->second;
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
  if (seen_.size() < states) {
    seen_.assign(states, 0);
    bestD_.resize(states);
    bestFrom_.resize(states);
    bestStep_.resize(states);
    generation_ = 0;
  }
  if (++generation_ == 0) {
    std::ranges::fill(seen_, 0);
    generation_ = 1;
  }
  constexpr auto NO_STATE = std::numeric_limits<std::uint64_t>::max();
  const auto known = [&](const std::size_t at) {
    return seen_[at] == generation_ ? bestD_[at]
                                    : std::numeric_limits<double>::infinity();
  };
  const auto keep = [&](const std::size_t at, const double d,
                        const std::uint64_t from, const double step) {
    seen_[at] = generation_;
    bestD_[at] = d;
    bestFrom_[at] = from;
    bestStep_[at] = step;
  };
  // Per state number: its edge and the chamber entered.
  struct Item {
    double d;
    std::uint64_t code;
    std::size_t at;
    std::size_t edge;
    std::int64_t chamber;
    [[nodiscard]] bool operator>(const Item& other) const {
      return d != other.d ? d > other.d : code > other.code;
    }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
  static const std::vector<Move> NONE;
  const auto leaving = [&](const std::int64_t chamber) -> const std::vector<Move>& {
    return chamber >= 0 && std::cmp_less(chamber, around_.size())
               ? around_[static_cast<std::size_t>(chamber)]
               : NONE;
  };
  for (const auto chamber : demand.from) {
    for (const auto& move : leaving(chamber)) {
      const auto k = move.edge;
      if (!allowed(k)) {
        continue;
      }
      const auto mask =
          demand.crossBit[k] | (demand.ownMouth[k] ? mouthBit : 0);
      const auto apart = distance(mid_[k], start);
      const auto d = (apart * weight[k]) + 1.0;
      const auto at = (move.pair * maskCount_) + mask;
      if (d < known(at)) {
        keep(at, d, NO_STATE, apart);
        heap.push({d, stateOf(k, move.into, mask), at, k, move.into});
      }
    }
  }
  std::optional<std::size_t> goal;
  auto goalCost = std::numeric_limits<double>::infinity();
  while (!heap.empty()) {
    const auto item = heap.top();
    heap.pop();
    const auto d = item.d;
    if (d > bestD_[item.at] || d >= goalCost) {
      continue;
    }
    const auto k = item.edge;
    const auto c = item.chamber;
    const auto mask = static_cast<std::uint64_t>(item.at % maskCount_);
    if (mask == full && std::ranges::find(demand.to, c) != demand.to.end()) {
      const auto total = d + distance(end, mid_[k]);
      if (total < goalCost) {
        goalCost = total;
        goal = item.at;
      }
    }
    for (const auto& move : leaving(c)) {
      const auto k2 = move.edge;
      if (!allowed(k2) || k2 == k) {
        continue;
      }
      auto mask2 = mask;
      if (edges_[k2].stretch) {
        if ((mask & demand.crossBit[k2]) != 0) {
          continue;
        }
        mask2 = mask | demand.crossBit[k2];
      }
      if (demand.ownMouth[k2]) {
        mask2 |= mouthBit;
      }
      const auto step = distance(mid_[k2], mid_[k]);
      const auto d2 = d + (step * weight[k2]) + 1.0;
      const auto at2 = (move.pair * maskCount_) + mask2;
      if (d2 < known(at2)) {
        keep(at2, d2, item.at, step);
        heap.push({d2, stateOf(k2, move.into, mask2), at2, k2, move.into});
      }
    }
  }
  if (!goal.has_value()) {
    return {std::nullopt, std::numeric_limits<double>::infinity()};
  }
  std::vector<std::size_t> path;
  std::vector<std::size_t> edgeOfPair(pairCount_);
  for (const auto& moves : around_) {
    for (const auto& move : moves) {
      edgeOfPair[move.pair] = move.edge;
    }
  }
  auto length = distance(end, mid_[edgeOfPair[*goal / maskCount_]]);
  std::uint64_t state = *goal;
  while (state != NO_STATE) {
    path.push_back(edgeOfPair[state / maskCount_]);
    length += bestStep_[state];
    state = bestFrom_[state];
  }
  std::ranges::reverse(path);
  return {std::move(path), length};
}

std::pair<SlotCheckResult, json> Checker::run() {
  readStrokesAndWires();
  addSides();
  SlotPlacer placer(graph_, grid_, stamps_);
  auto slots = placer.run(strokes_);
  addSlots(slots);
  const auto ne = edges_.size();
  for (const auto& edge : edges_) {
    capacity_.push_back(edge.capacity);
  }
  auto groups = selfBottlenecks(graph_, grid_, strokes_, edges_);
  for (auto& group :
       slotStubBottlenecks(graph_, grid_, stamps_, strokes_, edges_, wires_)) {
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
  for (auto& demand : demands) {
    openEdges(demand);
  }

  // Per wire and edge: whether the edge has an end at the wire's own run.
  std::unordered_map<std::string, std::size_t> demandOf;
  for (std::size_t d = 0; d < demands.size(); ++d) {
    demandOf.emplace(demands[d].name, d);
  }
  std::vector<std::vector<std::int64_t>> owns(
      demands.size(), std::vector<std::int64_t>(ne, 0));
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
  // Per wire: the last way pulled tight and its length, as a way often
  // stays from one round to the next.
  std::vector<std::pair<std::vector<std::size_t>, double>> lastTaut(
      demands.size(), {{}, -1.0});
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
        overK +=
            std::max<std::int64_t>(0, gload[gi] + 1 - groups[gi].capacity);
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
    std::vector<double> weight;
    for (std::size_t d = 0; d < demands.size(); ++d) {
      const auto& demand = demands[d];
      weight = common;
      for (const auto k : ownEdges[d]) {
        weight[k] = priceOf(k, 1);
      }
      auto [path, length] = shortest(demand, weight);
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
        auto& last = lastTaut[d];
        if (last.second < 0.0 || last.first != *path) {
          last = {*path, tautLength(start, end, linesOf(*path))};
        }
        length = last.second;
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
          path.has_value() && std::ranges::any_of(*path, [&](const auto k) {
            return isOver[k];
          });
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
           groups.size(), graph_.chambers, gates, newStretches,
           oldStretches),
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
        one.sides = std::array{
            edge.at("sides").at(0).get<std::vector<std::int64_t>>(),
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
    readSettings();
    const auto parsed = json::parse(graph);
    const auto typed = graphOf(parsed);
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
  readSettings();
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
