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
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
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

[[nodiscard]] std::vector<Point> pointsOf(const json& list) {
  std::vector<Point> points;
  points.reserve(list.size());
  for (const auto& value : list) {
    points.push_back(pointOf(value));
  }
  return points;
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
  std::vector<std::uint16_t> code;

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

[[nodiscard]] Grid gridOf(const json& graph) {
  Grid grid;
  grid.width = graph.at("width").get<std::int64_t>();
  grid.height = graph.at("height").get<std::int64_t>();
  grid.code.reserve(static_cast<std::size_t>(grid.width * grid.height));
  const auto& raster = graph.at("raster");
  for (std::size_t at = 0; at + 1 < raster.size(); at += 2) {
    const auto code = raster[at].get<std::int64_t>();
    const auto run = raster[at + 1].get<std::int64_t>();
    if (code < 0 || code > std::numeric_limits<std::uint16_t>::max()) {
      throw std::invalid_argument(
          std::format("slot check: raster code {} out of range", code));
    }
    grid.code.insert(grid.code.end(), static_cast<std::size_t>(run),
                     static_cast<std::uint16_t>(code));
  }
  if (std::cmp_not_equal(grid.code.size(), grid.width * grid.height)) {
    throw std::invalid_argument(
        "slot check: the raster does not cover the chip");
  }
  return grid;
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
  /// The corners of a slot's rectangle.
  std::vector<Point> corners;
  json out;
};

/// A crossing slot on a feedline edge.
struct Slot {
  Point at;
  std::array<Point, 4> corners;
  Point direction;
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
[[nodiscard]] double capacityMargin() {
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
[[nodiscard]] std::int64_t marginFrom() {
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
[[nodiscard]] bool gateHeading() {
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
[[nodiscard]] double resonatorPassage() {
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
[[nodiscard]] bool slotCrossingRule() {
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
[[nodiscard]] bool mouthTwins() {
  const char* text = std::getenv("SCPD_MOUTH_TWINS");
  return text == nullptr || std::string_view(text) != "0";
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
/// before where it fits; the end that gives more slots counts.
struct SlotPlacer {
  const json& graph;
  const Grid& grid;
  Stamps& stamps;
  double clearance = 0.0;
  double reach = 0.0;
  double along = 0.0;
  double inflation = 0.0;
  int feedlineCode = FEEDLINE;
  std::vector<std::string> wallKinds;
  std::vector<bool> portSlots;
  std::set<std::pair<std::int64_t, std::int64_t>> portSlotsOutside;
  /// The feedline routing's crossing rule over the edges between couplers
  /// (`slotCrossingRule`), and how far it reaches.
  routing::CrossingConstraints rule;
  int ruleReach = 0;

  SlotPlacer(const json& graphIn, const Grid& gridIn, Stamps& stampsIn)
      : graph(graphIn), grid(gridIn), stamps(stampsIn),
        clearance(graphIn.at("clearance").get<double>()),
        reach(graphIn.at("crossingReach").get<double>()), along(clearance),
        inflation(graphIn.at("inflation").get<double>()),
        wallKinds(graphIn.at("wallKinds").get<std::vector<std::string>>()),
        portSlots(gridIn.code.size(), false) {
    const auto kind = std::ranges::find(wallKinds, "feedline");
    if (kind == wallKinds.end()) {
      throw std::invalid_argument("slot check: no feedline wall kind");
    }
    feedlineCode = 1 + static_cast<int>(kind - wallKinds.begin());
    for (const auto& cell : graph.at("slots")) {
      const auto x = cell.at(0).get<std::int64_t>();
      const auto y = cell.at(1).get<std::int64_t>();
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

  [[nodiscard]] Placement place(const std::vector<Point>& line) {
    Placement result;
    const auto taken = stamps.fresh(1);
    double last = -1e9;
    std::vector<double> distanceAt{0.0};
    for (std::size_t k = 1; k < line.size(); ++k) {
      distanceAt.push_back(distanceAt.back() + distance(line[k], line[k - 1]));
    }
    const auto count = static_cast<std::ptrdiff_t>(line.size());
    std::vector<std::pair<std::int64_t, std::int64_t>> covered;
    for (std::ptrdiff_t k = 0; k < count; ++k) {
      const auto here = static_cast<std::size_t>(k);
      if (distanceAt[here] - last < along - 1e-9) {
        continue;
      }
      // The slot sits on one straight piece: every step of the edge it
      // covers along the edge has the same direction; it may reach the
      // point where the piece bends.
      auto step = heading(line, here);
      if (!step.has_value() && k + 1 < count) {
        step = minus(line[here + 1], line[here]);
      }
      if (!step.has_value()) {
        continue;
      }
      const auto norm = std::hypot(step->x, step->y);
      const auto ux = step->x / norm;
      const auto uy = step->y / norm;
      const auto halfSteps =
          static_cast<std::ptrdiff_t>(std::ceil((along / 2.0 / norm) - 1e-9));
      if (k - halfSteps < 0 || k + halfSteps >= count) {
        continue;
      }
      bool straight = true;
      for (auto q = k - halfSteps; q < k + halfSteps; ++q) {
        const auto piece = minus(line[static_cast<std::size_t>(q) + 1],
                                 line[static_cast<std::size_t>(q)]);
        if (piece.x != step->x || piece.y != step->y) {
          straight = false;
          break;
        }
      }
      if (!straight) {
        continue;
      }
      const auto& [cx, cy] = line[here];
      const auto nx = -uy;
      const auto ny = ux;
      const auto half = along / 2.0;
      covered.clear();
      for (double t = -half; t <= half + 1e-9; t += 0.5) {
        for (double s = -reach; s <= reach + 1e-9; s += 0.5) {
          covered.emplace_back(rounded(cx + (t * ux) + (s * nx)),
                               rounded(cy + (t * uy) + (s * ny)));
        }
      }
      std::array<Point, 4> corners{};
      constexpr std::array<std::pair<double, double>, 4> SIGNS{
          {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}};
      for (std::size_t corner = 0; corner < 4; ++corner) {
        const auto [a, b] = SIGNS[corner];
        corners[corner] = {cx + ((a * half) * ux) + ((b * reach) * nx),
                           cy + ((a * half) * uy) + ((b * reach) * ny)};
      }
      std::optional<std::string> blocked;
      if (!rule.empty() && !crossable(cx, cy, *step)) {
        blocked = "crossing rule";
      }
      // The edge's own straight piece: the points of the edge within
      // OWN_WINDOW of the slot's.
      const auto first =
          static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, k - OWN_WINDOW));
      const auto stop =
          static_cast<std::size_t>(std::min(count, k + OWN_WINDOW + 1));
      for (const auto& [x, y] : covered) {
        if (blocked.has_value()) {
          break;
        }
        const bool in = grid.inside(x, y);
        if (in ? portSlots[grid.cell(x, y)]
               : portSlotsOutside.contains({x, y})) {
          blocked = "slot of a port run";
          break;
        }
        const int value = in ? std::min(grid.at(x, y), CHAMBER) : 1;
        if (value == CHAMBER || value == 0) {
          continue;
        }
        // The edge's own wall only of the straight piece the slot sits on,
        // not of another arm of the edge bent back beside it.
        if (value == feedlineCode) {
          bool near = false;
          for (auto p = first; p < stop; ++p) {
            if (std::hypot(static_cast<double>(x) - line[p].x,
                           static_cast<double>(y) - line[p].y) <=
                inflation + 1.5) {
              near = true;
              break;
            }
          }
          if (near) {
            continue;
          }
        }
        blocked = value >= 1 && value <= 5
                      ? wallKinds.at(static_cast<std::size_t>(value - 1))
                      : std::string("wall");
        break;
      }
      if (!blocked.has_value() &&
          std::ranges::any_of(covered, [&](const auto& cell) {
            return stamps[grid.cell(cell.first, cell.second)] == taken;
          })) {
        blocked = "another slot";
      }
      if (blocked.has_value()) {
        result.refused.push_back({line[here], *blocked, corners});
        continue;
      }
      result.placed.push_back({line[here], corners, {ux, uy}});
      for (const auto& [x, y] : covered) {
        stamps[grid.cell(x, y)] = taken;
      }
      last = distanceAt[here];
    }
    result.length = distanceAt.back();
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
    for (const auto& wire : graph.at("wires")) {
      if (wire.value("terminal", false)) {
        terminal.insert(wire.at("name").get<std::string>());
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
    ruleReach = static_cast<int>(
        std::lround(graph.at("crossingReach").get<double>())) - 1;
    rule.build(static_cast<std::uint32_t>(grid.width),
               static_cast<std::uint32_t>(grid.height), edges, {}, ruleReach);
  }

  [[nodiscard]] std::vector<EdgeSlots> run(const std::vector<Stroke>& strokes) {
    if (slotCrossingRule()) {
      buildRule(strokes);
    }
    std::map<std::string, std::int64_t> stretchCapacity;
    for (const auto& edge : graph.at("edges")) {
      if (edge.at("kind") == "stretch") {
        stretchCapacity[edge.at("feedline").get<std::string>()] +=
            edge.at("capacity").get<std::int64_t>();
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
      auto& best =
          backward.placed.size() > forward.placed.size() ? backward : forward;
      const auto found = stretchCapacity.find(stroke.name);
      edges.push_back(
          {.name = stroke.name,
           .length = best.length,
           .slots = std::move(best.placed),
           .stretches = found == stretchCapacity.end() ? 0 : found->second,
           .refused = std::move(best.refused)});
    }
    return edges;
  }
};

/// The bottlenecks between two places of one feedline edge: a free line
/// from the edge's surface back to its surface, past which the edge holds
/// more crossing slots than the line takes wires. A slot is past the line
/// when one of its two ends lies in the room the line and the edge
/// enclose. Every wire that crosses at one of those slots passes the line
/// on that side, so the slots share what the line takes.
[[nodiscard]] std::vector<Group>
selfBottlenecks(const json& graph, const Grid& grid,
                const std::vector<Stroke>& strokes,
                const std::vector<Edge>& edges) {
  const auto clear = graph.at("clearance").get<double>();
  const auto margin = capacityMargin();
  const auto r = graph.at("inflation").get<double>();
  const auto reach = graph.at("crossingReach").get<double>() + 2;
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
slotStubBottlenecks(const json& graph, const Grid& grid, Stamps& stamps,
                    const std::vector<Stroke>& strokes,
                    const std::vector<Edge>& edges,
                    const std::vector<Wire>& wires) {
  const auto clear = graph.at("clearance").get<double>();
  const auto margin = capacityMargin();
  const auto longest = 4 * clear;
  // The open ends of the stubs' walls: stub cells with one stub cell
  // around them.
  std::vector<Point> tips;
  for (std::int64_t y = 0; y < grid.height; ++y) {
    for (std::int64_t x = 0; x < grid.width; ++x) {
      if (grid.at(x, y) != STUB) {
        continue;
      }
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
      while (head < queue.size() && found[which].size() <= limit) {
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
      closed[which] = found[which].size() <= limit;
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
constexpr std::uint64_t MASK_FIELD = (1ULL << CHAMBER_SHIFT) - 1;

[[nodiscard]] std::uint64_t stateOf(const std::size_t edge,
                                    const std::int64_t chamber,
                                    const std::uint64_t mask) {
  return (static_cast<std::uint64_t>(edge) << EDGE_SHIFT) |
         (static_cast<std::uint64_t>(chamber) << CHAMBER_SHIFT) | mask;
}

/// The check on one graph.
class Checker {
public:
  explicit Checker(const json& graph)
      : graph_(graph), grid_(gridOf(graph)), stamps_(grid_.code.size()),
        clear_(graph.at("clearance").get<double>()),
        margin_(capacityMargin()) {}

  /// The verdict, the lost wires and the edges over, and the graph as the
  /// check saw it.
  std::pair<SlotCheckResult, json> run();

private:
  const json& graph_;
  Grid grid_;
  Stamps stamps_;
  double clear_;
  /// The safety factor on the clearance (`capacityMargin`).
  double margin_;
  std::vector<Stroke> strokes_;
  std::vector<Wire> wires_;
  std::vector<Edge> edges_;
  std::vector<Point> mid_;
  /// Per chamber: the edges that lead out of it and the chamber each leads
  /// into.
  std::unordered_map<std::int64_t,
                     std::vector<std::pair<std::size_t, std::int64_t>>>
      around_;
  std::vector<std::set<std::string>> owners_;
  std::vector<std::int64_t> capacity_;
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
  [[nodiscard]] std::int64_t limitOf(std::size_t k, std::int64_t own) const;
  [[nodiscard]] Way shortest(const Demand& demand,
                             const std::vector<double>& weight) const;
};

void Checker::readStrokesAndWires() {
  for (const auto& stroke : graph_.at("strokes")) {
    strokes_.push_back({.name = stroke.at("name").get<std::string>(),
                        .kind = stroke.at("kind").get<std::string>(),
                        .points = pointsOf(stroke.at("points"))});
  }
  const auto endOf = [](const json& end) {
    WireEnd one{.cell = pointOf(end.at("cell")),
                .chambers = {},
                .label = textOf(end, "label")};
    if (truthy(end, "chambers")) {
      one.chambers = end.at("chambers").get<std::vector<std::int64_t>>();
    }
    return one;
  };
  for (const auto& wire : graph_.at("wires")) {
    Wire one{.name = wire.at("name").get<std::string>(),
             .source = endOf(wire.at("source")),
             .target = endOf(wire.at("target")),
             .crosses = {},
             .terminal = truthy(wire, "terminal"),
             .resonator = truthy(wire, "resonator")};
    if (truthy(wire, "crosses")) {
      one.crosses = wire.at("crosses").get<std::vector<std::string>>();
    }
    ends_[one.name] = {one.source.cell, one.target.cell};
    wires_.push_back(std::move(one));
  }
}

void Checker::addSides() {
  std::vector<std::pair<std::string, std::pair<Point, Point>>> gateLines;
  for (const auto& edge : graph_.at("edges")) {
    if (edge.at("kind") == "bottleneck") {
      const auto& line = edge.at("line");
      gateLines.emplace_back(
          edge.at("name").get<std::string>(),
          std::pair{pointOf(line.front()), pointOf(line.back())});
    }
  }
  std::set<std::string> resonatorNames;
  for (const auto& wire : graph_.at("wires")) {
    if (truthy(wire, "resonator")) {
      resonatorNames.insert(wire.at("name").get<std::string>());
    }
  }
  for (const auto& edge : graph_.at("edges")) {
    if (edge.at("kind") != "bottleneck") {
      continue;
    }
    Edge one;
    one.out = edge;
    one.name = edge.at("name").get<std::string>();
    one.chambers = edge.at("chambers").get<std::vector<std::int64_t>>();
    one.capacity = edge.at("capacity").get<std::int64_t>();
    if (edge.contains("users") && !edge.at("users").is_null()) {
      one.users = edge.at("users").get<std::vector<std::string>>();
    }
    one.from = pointOf(edge.at("line").front());
    one.to = pointOf(edge.at("line").back());
    if (edge.contains("ends")) {
      one.ends = edge.at("ends").get<std::vector<std::string>>();
    }
    one.mouth = truthy(edge, "mouth");
    one.length = edge.at("length").get<double>();
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
      one.capacity = wiresIn(one.length * headingShare(one.from, one.to),
                             clear_, margin_);
    }
    if (edge.contains("sides")) {
      one.sides =
          std::array{edge.at("sides").at(0).get<std::vector<std::int64_t>>(),
                     edge.at("sides").at(1).get<std::vector<std::int64_t>>()};
    }
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
      one.out["sides"] = json::array({left, right});
      one.out["chambers"] = both;
    }
    edges_.push_back(std::move(one));
  }
}

void Checker::addSlots(const std::vector<EdgeSlots>& slots) {
  const auto reach = graph_.at("crossingReach").get<double>() + 2;
  // An edge of a chain with a reflector end may have one chamber on both
  // sides, as the check's own stretches say.
  std::set<std::string> reflected;
  std::map<std::string, std::vector<std::string>> crossers;
  for (const auto& edge : graph_.at("edges")) {
    if (edge.at("kind") == "stretch") {
      const auto chambers =
          edge.at("chambers").get<std::vector<std::int64_t>>();
      if (std::set(chambers.begin(), chambers.end()).size() == 1) {
        reflected.insert(edge.at("feedline").get<std::string>());
      }
    }
  }
  for (const auto& wire : wires_) {
    for (const auto& feedline : wire.crosses) {
      crossers[feedline].push_back(wire.name);
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
      one.corners.assign(slot.corners.begin(), slot.corners.end());
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
        edge.out["mouth"] = true;
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

std::int64_t Checker::limitOf(const std::size_t k,
                              const std::int64_t own) const {
  if (edges_[k].stretch || owners_[k].empty()) {
    return capacity_[k];
  }
  // A gate with an end on a wire's own port run holds that wire half a
  // clearance more: the wire keeps no clearance from its own run.
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
  const Point middle{graph_.at("width").get<double>() / 2.0,
                     graph_.at("height").get<double>() / 2.0};
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
      const auto at = std::ranges::find(demand.crosses, edge.feedline);
      if (at == demand.crosses.end()) {
        ok = false;
      } else {
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
  struct Best {
    double d = 0.0;
    std::optional<std::uint64_t> from;
    double step = 0.0;
  };
  std::unordered_map<std::uint64_t, Best> best;
  using Item = std::pair<double, std::uint64_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
  const auto known = [&](const std::uint64_t state) {
    const auto found = best.find(state);
    return found == best.end() ? std::numeric_limits<double>::infinity()
                               : found->second.d;
  };
  static const std::vector<std::pair<std::size_t, std::int64_t>> NONE;
  const auto leaving = [&](const std::int64_t chamber)
      -> const std::vector<std::pair<std::size_t, std::int64_t>>& {
    const auto found = around_.find(chamber);
    return found == around_.end() ? NONE : found->second;
  };
  for (const auto chamber : demand.from) {
    for (const auto& [k, y] : leaving(chamber)) {
      if (!allowed(k)) {
        continue;
      }
      const auto mask =
          demand.crossBit[k] | (demand.ownMouth[k] ? mouthBit : 0);
      const auto apart = distance(mid_[k], start);
      const auto d = (apart * weight[k]) + 1.0;
      const auto state = stateOf(k, y, mask);
      if (d < known(state)) {
        best[state] = {d, std::nullopt, apart};
        heap.emplace(d, state);
      }
    }
  }
  std::optional<std::uint64_t> goal;
  auto goalCost = std::numeric_limits<double>::infinity();
  while (!heap.empty()) {
    const auto [d, state] = heap.top();
    heap.pop();
    if (d > best.at(state).d || d >= goalCost) {
      continue;
    }
    const auto k = static_cast<std::size_t>(state >> EDGE_SHIFT);
    const auto c = static_cast<std::int64_t>((state >> CHAMBER_SHIFT) & FIELD);
    const auto mask = state & MASK_FIELD;
    if (mask == full && std::ranges::find(demand.to, c) != demand.to.end()) {
      const auto total = d + distance(end, mid_[k]);
      if (total < goalCost) {
        goalCost = total;
        goal = state;
      }
    }
    for (const auto& [k2, y] : leaving(c)) {
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
      const auto state2 = stateOf(k2, y, mask2);
      if (d2 < known(state2)) {
        best[state2] = {d2, state, step};
        heap.emplace(d2, state2);
      }
    }
  }
  if (!goal.has_value()) {
    return {std::nullopt, std::numeric_limits<double>::infinity()};
  }
  std::vector<std::size_t> path;
  auto length =
      distance(end, mid_[static_cast<std::size_t>(*goal >> EDGE_SHIFT)]);
  std::optional<std::uint64_t> state = goal;
  while (state.has_value()) {
    path.push_back(static_cast<std::size_t>(*state >> EDGE_SHIFT));
    const auto& entry = best.at(*state);
    length += entry.step;
    state = entry.from;
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
  std::unordered_map<std::size_t, std::vector<std::size_t>> member;
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
        const auto found = member.find(k);
        if (found != member.end()) {
          for (const auto gi : found->second) {
            ++count[gi];
          }
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
  // two of its chambers.
  std::int64_t mostChamber = 0;
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
    }
    for (const auto& [x, y] : pairs) {
      around_[x].emplace_back(k, y);
      mostChamber = std::max({mostChamber, x, y});
    }
  }
  std::size_t mostCrosses = 0;
  for (const auto& demand : demands) {
    mostCrosses = std::max(mostCrosses, demand.crosses.size());
  }
  if (ne > FIELD || std::cmp_greater(mostChamber, FIELD) ||
      mostCrosses + 1 >= static_cast<std::size_t>(CHAMBER_SHIFT)) {
    throw std::invalid_argument(
        "slot check: the graph is too large for the search");
  }
  findOwners();
  markMouthTwins();
  findMouths();
  for (auto& demand : demands) {
    openEdges(demand);
  }

  std::unordered_map<std::string, Way> base;
  std::unordered_map<std::string, double> tautBase;
  for (const auto& demand : demands) {
    std::vector<double> weight(ne, -1.0);
    for (std::size_t k = 0; k < ne; ++k) {
      if (limitOf(k, owners_[k].contains(demand.name) ? 1 : 0) >= 1) {
        weight[k] = 1.0;
      }
    }
    base[demand.name] = shortest(demand, weight);
  }
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
  for (round = 0; round < ROUNDS; ++round) {
    std::ranges::fill(load, 0);
    std::vector<std::int64_t> ownLoad(ne, 0);
    std::ranges::fill(gload, 0);
    std::vector<double> weight(ne, -1.0);
    for (const auto& demand : demands) {
      for (std::size_t k = 0; k < ne; ++k) {
        const std::int64_t mine = owners_[k].contains(demand.name) ? 1 : 0;
        const auto lim = limitOf(k, ownLoad[k] + mine);
        auto overK = std::max<std::int64_t>(0, load[k] + 1 - lim);
        const auto found = member.find(k);
        if (found != member.end()) {
          for (const auto gi : found->second) {
            overK +=
                std::max<std::int64_t>(0, gload[gi] + 1 - groups[gi].capacity);
          }
        }
        weight[k] = lim >= 1
                        ? (1.0 + hist[k]) *
                              (1.0 + ((0.5 * static_cast<double>(round + 1)) *
                                      static_cast<double>(overK)))
                        : -1.0;
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
        length = tautLength(start, end, linesOf(*path));
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
          if (owners_[k].contains(demand.name)) {
            ++ownLoad[k];
          }
        }
      }
      for (const auto gi : passes(demand.name, path)) {
        ++gload[gi];
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
  for (const auto& wire : graph_.at("wires")) {
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
                        {"rejected", std::move(refused)}});
  }
  std::int64_t oldStretches = 0;
  for (const auto& edge : graph_.at("edges")) {
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
  auto out = graph_;
  out["edges"] = std::move(edgesOut);
  out["wires"] = std::move(wiresOut);
  out["verdict"] = result.verdict;
  out["crossSlots"] = {{"reach", placer.reach},
                       {"along", placer.along},
                       {"edges", std::move(slotsOut)}};
  out["summary"] = json::array(
      {"the capacity check by crossing slots: the rectangle slots of every "
       "feedline edge in place of its stretches; a gate joins only chambers "
       "on different sides of its line; a way runs at most 1.5 times its "
       "shortest way, both pulled tight; a wire with a crossing "
       "prescription passes no mouth between chains; a plain wire not "
       "enclosed and the terminal edges pass their own mouth and no other; "
       "a gate at a wire's own run takes half a clearance more for that "
       "wire when the wire leaves the run straight through the gate",
       std::format(
           "{} self-bottlenecks (S …): a free line from one place of a "
           "feedline edge to another, or from a slot to the open end of a "
           "stub, with more slots and wire ends behind it than it takes "
           "wires; {} chambers, {} bottlenecks; {} crossing slots (the "
           "stretches gave {})",
           groups.size(), graph_.at("nodes").size(), gates, newStretches,
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
std::pair<SlotCheckResult, json> judge(const std::string_view graph) {
  try {
    const auto parsed = json::parse(graph);
    return Checker(parsed).run();
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

std::string slotCheckJson(const std::string_view graph) {
  auto [result, out] = judge(graph);
  return json{{"verdict", result.verdict},
              {"lost", result.lost},
              {"over", result.over},
              {"graph", std::move(out)}}
      .dump();
}

} // namespace mqt::scpd::pipeline
