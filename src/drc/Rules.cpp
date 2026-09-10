/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/drc/Rules.hpp"

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::drc {
namespace {

namespace fbd = flatbuffers::design;
namespace fbdrc = flatbuffers::drc;
namespace fbg = flatbuffers::geometry;

/// A cell of a wire, and which wire it belongs to.
struct Occupant {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::uint32_t wire = 0;
};

/// The two ends of a wire, in cells.
struct Ends {
  std::array<std::pair<double, double>, 2> at{};
  bool present = false;
};

[[nodiscard]] Ends endsOf(const CheckedWire& wire) {
  Ends ends;
  if (wire.cells.empty()) {
    return ends;
  }
  ends.present = true;
  ends.at[0] = {static_cast<double>(wire.cells.front().x()),
                static_cast<double>(wire.cells.front().y())};
  ends.at[1] = {static_cast<double>(wire.cells.back().x()),
                static_cast<double>(wire.cells.back().y())};
  return ends;
}

[[nodiscard]] bool sharesComponent(const CheckedWire& one,
                                   const CheckedWire& two) {
  for (const auto& here : one.components) {
    if (here.empty()) {
      continue;
    }
    for (const auto& there : two.components) {
      if (here == there) {
        return true;
      }
    }
  }
  return false;
}

/// Where two wires meet, in cells.
///
/// A terminal of one within the clearance of a terminal of the other is a
/// junction, and so is any pair of terminals on one component: the ports of a
/// component sit closer together than the wire spacing and each has to be
/// reached, so the two approaches converge and no arrangement holds them
/// apart. Everything within `junctionRadiusNorm` clearances of either terminal
/// belongs to that meeting.
[[nodiscard]] std::vector<std::pair<double, double>>
junctionsOf(const CheckedWire& one, const CheckedWire& two,
            const double clearance) {
  std::vector<std::pair<double, double>> centres;
  const auto here = endsOf(one);
  const auto there = endsOf(two);
  if (!here.present || !there.present) {
    return centres;
  }
  const bool shared = sharesComponent(one, two);
  for (const auto& a : here.at) {
    for (const auto& b : there.at) {
      if (shared ||
          std::hypot(a.first - b.first, a.second - b.second) <= clearance) {
        centres.push_back(a);
        centres.push_back(b);
      }
    }
  }
  return centres;
}

[[nodiscard]] std::unique_ptr<fbdrc::DrcFindingT>
findingOf(const fbdrc::DrcRule rule, const fbdrc::DrcSeverity severity,
          const std::vector<std::uint32_t>& wires, const fbg::Point where,
          const double measured, const double limit, std::string message) {
  auto finding = std::make_unique<fbdrc::DrcFindingT>();
  finding->rule = rule;
  finding->severity = severity;
  finding->wires.reserve(wires.size());
  for (const auto wire : wires) {
    finding->wires.emplace_back(wire);
  }
  finding->location = where;
  finding->measured = measured;
  finding->limit = limit;
  finding->message = std::move(message);
  return finding;
}

/// Rule 1: no two different wires closer than the wire spacing.
void checkClearance(const CellView& view, const fbd::DesignRulesT& rules,
                    const Settings& settings, fbdrc::DrcReportT& report) {
  if (rules.min_wire_spacing <= 0.0) {
    return;
  }
  const auto spans = grid::cellsFor(rules.min_wire_spacing, view.grid);
  if (spans == 0) {
    return;
  }
  // The rule is a length, and on this grid it is a length the grid can
  // express: a router cell is about ten layout units, so the rule spans
  // nineteen of them and what the router keeps — nineteen whole cells, about
  // 189.5 units — is a little more than the rule asks for. The check asks for
  // the rule itself, in layout units, because that is the physical contract;
  // the conversion is only used to bucket the cells and to size the junction
  // zones, where what matters is the router's own figure.
  const auto clearance = static_cast<double>(spans);
  const double unit = std::min(view.grid.cellWidth, view.grid.cellHeight);
  const double limit = rules.min_wire_spacing;

  // The cells of every wire, in buckets one clearance across, so that a pair
  // is compared only against the nine buckets that can hold it.
  const auto side = std::max<std::int32_t>(1, static_cast<std::int32_t>(spans));
  std::map<std::pair<std::int32_t, std::int32_t>, std::vector<Occupant>>
      buckets;
  for (std::uint32_t index = 0; index < view.wires.size(); ++index) {
    const auto& wire = view.wires[index];
    if (wire.feedline) {
      ++report.feedlines_skipped;
      continue;
    }
    for (const auto& cell : wire.cells) {
      const auto x = static_cast<std::int32_t>(cell.x());
      const auto y = static_cast<std::int32_t>(cell.y());
      buckets[{x / side, y / side}].push_back({x, y, index});
    }
  }

  // One finding per pair, at the closest approach the pair has.
  struct Closest {
    double distance = 0.0;
    std::int32_t x = 0;
    std::int32_t y = 0;
  };
  std::map<std::pair<std::uint32_t, std::uint32_t>, Closest> worst;
  std::map<std::pair<std::uint32_t, std::uint32_t>,
           std::vector<std::pair<double, double>>>
      meetings;

  for (const auto& [key, here] : buckets) {
    std::vector<Occupant> near;
    for (std::int32_t dx = -1; dx <= 1; ++dx) {
      for (std::int32_t dy = -1; dy <= 1; ++dy) {
        const auto found = buckets.find({key.first + dx, key.second + dy});
        if (found != buckets.end()) {
          near.insert(near.end(), found->second.begin(), found->second.end());
        }
      }
    }
    for (const auto& one : here) {
      for (const auto& two : near) {
        if (one.wire >= two.wire) {
          continue;
        }
        const auto distance = std::hypot(static_cast<double>(one.x - two.x),
                                         static_cast<double>(one.y - two.y));
        if (distance * unit > limit) {
          continue;
        }
        const std::pair pair{one.wire, two.wire};
        const auto known = meetings.find(pair);
        if (known == meetings.end()) {
          meetings.emplace(pair, junctionsOf(view.wires[one.wire],
                                             view.wires[two.wire], clearance));
        }
        const auto& centres = meetings.at(pair);
        const auto mx = 0.5 * (one.x + two.x);
        const auto my = 0.5 * (one.y + two.y);
        const bool exempt =
            std::ranges::any_of(centres, [&](const auto& centre) {
              return std::hypot(mx - centre.first, my - centre.second) <=
                     settings.junctionRadiusNorm * clearance;
            });
        if (exempt) {
          continue;
        }
        auto& record = worst[pair];
        if (record.distance == 0.0 && record.x == 0 && record.y == 0) {
          record = {.distance = distance, .x = one.x, .y = one.y};
        } else if (distance < record.distance) {
          record = {.distance = distance, .x = one.x, .y = one.y};
        }
      }
    }
  }

  for (const auto& [pair, record] : worst) {
    const auto kind = record.distance < settings.shortThreshold
                          ? fbdrc::ClearanceKind::Short
                          : fbdrc::ClearanceKind::NearMiss;
    const auto place = view.grid.toLayout(record.x, record.y);
    auto finding = findingOf(
        fbdrc::DrcRule::WireClearance, fbdrc::DrcSeverity::Active,
        {view.wires[pair.first].connection, view.wires[pair.second].connection},
        place, record.distance * unit, limit,
        std::format(
            "wires {} and {} come within {:.1f} layout units of each other",
            view.wires[pair.first].connection,
            view.wires[pair.second].connection, record.distance * unit));
    finding->clearance_kind = kind;
    report.findings.push_back(std::move(finding));
  }
}

/// Rule 3: a wire does not meet itself.
void checkLoops(const CellView& view, fbdrc::DrcReportT& report) {
  routing::PathLoopScratch scratch;
  std::vector<routing::PathLoopHit> hits;
  for (const auto& wire : view.wires) {
    routing::Path path;
    path.reserve(wire.cells.size());
    for (const auto& cell : wire.cells) {
      path.push_back({.x = cell.x(),
                      .y = cell.y(),
                      .heading = cell.heading(),
                      .primitive = 0});
    }
    hits.clear();
    // The router's own test, so that what is reported is exactly what the
    // router refuses to produce.
    const auto found = routing::findPathSelfIntersections(
        path, view.grid.width, view.grid.height, scratch, &hits);
    if (found == 0) {
      continue;
    }
    for (const auto& hit : hits) {
      report.findings.push_back(findingOf(
          fbdrc::DrcRule::WireLoop, fbdrc::DrcSeverity::Active,
          {wire.connection}, view.grid.toLayout(hit.x, hit.y), 0.0, 0.0,
          std::format(
              "wire {} meets itself at ({}, {}), between step {} and step {}",
              wire.connection, hit.x, hit.y, hit.firstIndex, hit.secondIndex)));
    }
  }
}

/// Rule 4: no wire closer to a chip obstacle than the obstacle spacing.
///
/// The distance is measured the way the raster measures it, from the cell to
/// the polygon edge in layout units, by rasterizing the obstacles with the
/// keepout and asking whether a wire cell is one of the cells that leaves. The
/// stage bakes the same mask in before it searches, so what this reports is a
/// cell the committed result holds and the search never offered.
void checkObstacles(const CellView& view, const fbd::DesignRulesT& rules,
                    fbdrc::DrcReportT& report) {
  if (view.chip == nullptr || rules.min_obstacle_spacing <= 0.0) {
    return;
  }
  const auto raster = grid::rasterizeObstacles(
      *view.chip, view.grid, {.keepout = rules.min_obstacle_spacing});
  for (const auto& wire : view.wires) {
    for (const auto& cell : wire.cells) {
      if (!view.grid.contains(cell.x(), cell.y()) ||
          !raster.blocked.testCell(cell.x(), cell.y())) {
        continue;
      }
      report.findings.push_back(findingOf(
          fbdrc::DrcRule::ObstacleClearance, fbdrc::DrcSeverity::Active,
          {wire.connection}, view.grid.toLayout(cell.x(), cell.y()), 0.0,
          rules.min_obstacle_spacing,
          std::format(
              "wire {} runs within {} layout units of an obstacle at ({}, {})",
              wire.connection, rules.min_obstacle_spacing, cell.x(),
              cell.y())));
      break;
    }
  }
}

} // namespace

fbdrc::DrcReportT checkCells(const CellView& view,
                             const fbd::DesignRulesT& rules,
                             const Settings& settings) {
  fbdrc::DrcReportT report;
  report.stage = fbdrc::DrcStage::Final;
  checkClearance(view, rules, settings, report);
  checkLoops(view, report);
  checkObstacles(view, rules, report);
  return report;
}

namespace {

[[nodiscard]] std::string_view nameOf(const fbdrc::DrcRule rule) {
  switch (rule) {
  case fbdrc::DrcRule::WireClearance:
    return "wire-clearance";
  case fbdrc::DrcRule::FeedlineOrthogonality:
    return "feedline-orthogonality";
  case fbdrc::DrcRule::WireLoop:
    return "wire-loop";
  case fbdrc::DrcRule::ObstacleClearance:
    return "obstacle-clearance";
  case fbdrc::DrcRule::ComponentOverlap:
    return "component-overlap";
  case fbdrc::DrcRule::MinStraightLength:
    return "min-straight-length";
  case fbdrc::DrcRule::ResonatorLength:
    return "resonator-length";
  case fbdrc::DrcRule::MinBendRadius:
    return "min-bend-radius";
  case fbdrc::DrcRule::Unset:
    break;
  }
  return "unset";
}

[[nodiscard]] std::string_view kindOf(const fbdrc::ClearanceKind kind) {
  switch (kind) {
  case fbdrc::ClearanceKind::NearMiss:
    return "near-miss";
  case fbdrc::ClearanceKind::Short:
    return "short";
  case fbdrc::ClearanceKind::Unset:
    break;
  }
  return "unset";
}

/// A string with the six characters JSON forbids escaped.
[[nodiscard]] std::string quoted(const std::string_view text) {
  std::string written = "\"";
  for (const char letter : text) {
    switch (letter) {
    case '"':
      written += "\\\"";
      break;
    case '\\':
      written += "\\\\";
      break;
    case '\n':
      written += "\\n";
      break;
    case '\r':
      written += "\\r";
      break;
    case '\t':
      written += "\\t";
      break;
    default:
      written += letter;
      break;
    }
  }
  return written + "\"";
}

} // namespace

std::string toJson(const fbdrc::DrcReportsT& reports) {
  std::string text = "{\n  \"reports\": [\n";
  for (std::size_t at = 0; at < reports.reports.size(); ++at) {
    const auto& report = *reports.reports[at];
    text += "    {\n";
    text += std::format("      \"stage\": \"{}\",\n",
                        report.stage == fbdrc::DrcStage::Finalize ? "finalize"
                                                                  : "final");
    text += std::format("      \"feedlines_skipped\": {},\n",
                        report.feedlines_skipped);
    text += "      \"findings\": [\n";
    for (std::size_t index = 0; index < report.findings.size(); ++index) {
      const auto& finding = *report.findings[index];
      std::string wires;
      for (std::size_t which = 0; which < finding.wires.size(); ++which) {
        wires += (which == 0 ? "" : ", ") +
                 std::to_string(finding.wires[which].index());
      }
      text += std::format(
          "        {{\"rule\": {}, \"severity\": \"{}\", \"wires\": [{}], "
          "\"location\": {{\"x\": {}, \"y\": {}}}, \"measured\": {}, "
          "\"limit\": {}, \"clearance_kind\": \"{}\", \"message\": {}}}",
          quoted(nameOf(finding.rule)),
          finding.severity == fbdrc::DrcSeverity::Advisory ? "advisory"
                                                           : "active",
          wires, finding.location.x(), finding.location.y(), finding.measured,
          finding.limit, kindOf(finding.clearance_kind),
          quoted(finding.message));
      text += (index + 1 < report.findings.size()) ? ",\n" : "\n";
    }
    text += "      ]\n    }";
    text += (at + 1 < reports.reports.size()) ? ",\n" : "\n";
  }
  return text + "  ]\n}\n";
}

} // namespace mqt::scpd::drc
