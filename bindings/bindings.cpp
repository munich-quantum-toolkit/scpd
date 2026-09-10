/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/drc/Rules.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/io/Chip.hpp"
#include "mqt-scpd/io/Config.hpp"
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <nanobind/nanobind.h>
// The type casters below take part through the conversions they enable, not
// through a name the code spells out.
#include <nanobind/stl/pair.h>        // IWYU pragma: keep
#include <nanobind/stl/string.h>      // IWYU pragma: keep
#include <nanobind/stl/string_view.h> // IWYU pragma: keep
#include <nanobind/stl/vector.h>      // IWYU pragma: keep

#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace {

std::span<const std::uint8_t> asSpan(const nb::bytes& bytes) {
  return {static_cast<const std::uint8_t*>(bytes.data()), bytes.size()};
}

nb::bytes asBytes(const std::vector<std::uint8_t>& bytes) {
  return nb::bytes(bytes.data(), bytes.size());
}

/// One stage output, wrapped as the artifact a run directory stores.
///
/// The producer comes from the caller because the version of the package is
/// what Python knows; the core has no build-time version stamped into it.
template <typename Output>
nb::bytes asArtifact(Output&& output, const std::string_view producer) {
  mqt::scpd::flatbuffers::artifacts::ArtifactT artifact;
  artifact.producer = producer;
  artifact.output.Set(std::forward<Output>(output));
  return asBytes(mqt::scpd::io::writeArtifact(artifact));
}

/// The capacity plan an artifact carries.
const mqt::scpd::flatbuffers::artifacts::CapacityPlanT&
capacityOf(const mqt::scpd::flatbuffers::artifacts::ArtifactT& artifact) {
  const auto* output = artifact.output.AsCapacityPlan();
  if (output == nullptr) {
    throw std::invalid_argument("the artifact is not a capacity plan");
  }
  return *output;
}

/// The global routing an artifact carries.
const mqt::scpd::flatbuffers::artifacts::GlobalRoutingT&
globalOf(const mqt::scpd::flatbuffers::artifacts::ArtifactT& artifact) {
  const auto* output = artifact.output.AsGlobalRouting();
  if (output == nullptr) {
    throw std::invalid_argument("the artifact is not a global routing");
  }
  return *output;
}

/// The assignment an artifact carries.
const mqt::scpd::flatbuffers::artifacts::AssignmentT&
assignmentOf(const mqt::scpd::flatbuffers::artifacts::ArtifactT& artifact) {
  const auto* output = artifact.output.AsAssignment();
  if (output == nullptr) {
    throw std::invalid_argument("the artifact is not an assignment");
  }
  return *output;
}

/// The corridor routing an artifact carries.
const mqt::scpd::flatbuffers::artifacts::CorridorRoutingT&
corridorOf(const mqt::scpd::flatbuffers::artifacts::ArtifactT& artifact) {
  const auto* output = artifact.output.AsCorridorRouting();
  if (output == nullptr) {
    throw std::invalid_argument("the artifact is not a corridor routing");
  }
  return *output;
}

/// The Python callable a stage reports its progress to, or nothing.
mqt::scpd::pipeline::Progress sayTo(const nb::object& progress) {
  if (progress.is_none()) {
    return {};
  }
  return [&progress](const std::string_view line) {
    progress(nb::str(line.data(), line.size()));
  };
}

/// The detail routing an artifact carries.
const mqt::scpd::flatbuffers::artifacts::DetailRoutingT&
detailOf(const mqt::scpd::flatbuffers::artifacts::ArtifactT& artifact) {
  const auto* output = artifact.output.AsDetailRouting();
  if (output == nullptr) {
    throw std::invalid_argument("the artifact is not a detail routing");
  }
  return *output;
}

/// What the design-rule check sees of a final routing: every wire as the cells
/// it runs over, with the components its two ends sit on.
mqt::scpd::drc::CellView
viewOf(const mqt::scpd::flatbuffers::design::ChipT& chip,
       const mqt::scpd::flatbuffers::artifacts::GlobalRoutingT& global,
       const mqt::scpd::flatbuffers::artifacts::AssignmentT& assignment,
       const mqt::scpd::flatbuffers::artifacts::FinalRoutingT& routing) {
  const auto componentOf =
      [&chip](const std::uint32_t port) -> std::string_view {
    return port < chip.ports.size() ? chip.ports[port]->component
                                    : std::string_view{};
  };
  mqt::scpd::drc::CellView view;
  if (routing.grid != nullptr) {
    view.grid = {.width = routing.grid->width,
                 .height = routing.grid->height,
                 .origin = routing.grid->origin,
                 .cellWidth = routing.grid->cell_width,
                 .cellHeight = routing.grid->cell_height};
  }
  view.chip = &chip;
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    if (routing.wires[index]->path.empty() ||
        index >= assignment.connections.size()) {
      continue;
    }
    const auto& connection = *assignment.connections[index];
    view.wires.push_back(
        {.connection = static_cast<std::uint32_t>(index),
         .cells = routing.wires[index]->path,
         .components = {std::string_view{},
                        componentOf(connection.target.index())},
         .feedline = false});
  }
  for (std::size_t index = 0; index < routing.inner.size(); ++index) {
    if (routing.inner[index]->path.empty() ||
        index >= global.connections.size()) {
      continue;
    }
    const auto& connection = *global.connections[index];
    view.wires.push_back(
        {.connection = static_cast<std::uint32_t>(routing.wires.size() + index),
         .cells = routing.inner[index]->path,
         .components = {connection.source == nullptr
                            ? std::string_view{}
                            : componentOf(connection.source->index()),
                        componentOf(connection.target.index())},
         .feedline = false});
  }
  return view;
}

/// One report as the text of drc.json.
std::string asReports(mqt::scpd::flatbuffers::drc::DrcReportT report) {
  mqt::scpd::flatbuffers::drc::DrcReportsT reports;
  reports.reports.push_back(
      std::make_unique<mqt::scpd::flatbuffers::drc::DrcReportT>(
          std::move(report)));
  return mqt::scpd::drc::toJson(reports);
}

} // namespace

// The bindings expose what the command-line interface needs and nothing else.
// Chips and configurations cross the boundary as FlatBuffers bytes, so both
// sides read them through the schema-generated code.
// NOLINTNEXTLINE(performance-unnecessary-value-param)
NB_MODULE(MQT_SCPD_MODULE_NAME, m) {
  m.doc() = "Internal bindings of the MQT SCPD core. The command-line "
            "interface is the supported product.";

  m.def(
      "load_chip",
      [](const std::string_view chipJson, const nb::bytes& config) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        return asBytes(mqt::scpd::io::writeChip(
            mqt::scpd::io::loadChip(chipJson, configuration)));
      },
      "chip_json"_a, "config"_a,
      "Read the chip input, classify its ports from the configuration's "
      "patterns and check the configured port sequences against the chip. "
      "Returns the classified chip as bytes of the design schema. Raises "
      "ValueError naming every problem.");

  m.def(
      "plan_capacity",
      [](const nb::bytes& chip, const nb::bytes& config,
         const std::string_view producer) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto name =
            mqt::scpd::pipeline::selectedCapacityPlanner(configuration);
        return asArtifact(
            mqt::scpd::pipeline::capacityPlanners().make(name)->run(
                design, configuration),
            producer);
      },
      "chip"_a, "config"_a, "producer"_a,
      "Run the Capacity stage. Returns 01-capacity.fb as bytes.");

  m.def(
      "route_global",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& config, const std::string_view producer) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto name =
            mqt::scpd::pipeline::selectedGlobalRouter(configuration);
        return asArtifact(mqt::scpd::pipeline::globalRouters().make(name)->run(
                              design, capacityOf(plan), configuration),
                          producer);
      },
      "chip"_a, "capacity"_a, "config"_a, "producer"_a,
      "Run the Global stage. Returns 02-global.fb as bytes.");

  m.def(
      "assign",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& global, const nb::bytes& config,
         const std::string_view producer) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto routing = mqt::scpd::io::readArtifact(asSpan(global));
        const auto name = mqt::scpd::pipeline::selectedAssigner(configuration);
        return asArtifact(
            mqt::scpd::pipeline::assigners().make(name)->run(
                design, capacityOf(plan), globalOf(routing), configuration),
            producer);
      },
      "chip"_a, "capacity"_a, "global"_a, "config"_a, "producer"_a,
      "Run the Assignment stage. Returns 03-assign.fb as bytes.");

  m.def(
      "route_corridor",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& assignment, const nb::bytes& config,
         const std::string_view producer, const nb::object& progress) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto assigned = mqt::scpd::io::readArtifact(asSpan(assignment));
        const auto name =
            mqt::scpd::pipeline::selectedCorridorRouter(configuration);
        return asArtifact(
            mqt::scpd::pipeline::corridorRouters().make(name)->run(
                design, capacityOf(plan), assignmentOf(assigned), configuration,
                sayTo(progress)),
            producer);
      },
      "chip"_a, "capacity"_a, "assignment"_a, "config"_a, "producer"_a,
      "progress"_a = nb::none(),
      "Run the Corridor stage. Returns 04-corridor.fb as bytes. progress, when "
      "given, is called with one line per round while the stage runs.");

  m.def(
      "route_detail",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& global, const nb::bytes& assignment,
         const nb::bytes& corridor, const nb::bytes& config,
         const std::string_view producer, const nb::object& progress) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto circuit = mqt::scpd::io::readArtifact(asSpan(global));
        const auto assigned = mqt::scpd::io::readArtifact(asSpan(assignment));
        const auto routed = mqt::scpd::io::readArtifact(asSpan(corridor));
        const auto name =
            mqt::scpd::pipeline::selectedDetailRouter(configuration);
        return asArtifact(mqt::scpd::pipeline::detailRouters().make(name)->run(
                              design, capacityOf(plan), globalOf(circuit),
                              assignmentOf(assigned), corridorOf(routed),
                              configuration, sayTo(progress)),
                          producer);
      },
      "chip"_a, "capacity"_a, "global"_a, "assignment"_a, "corridor"_a,
      "config"_a, "producer"_a, "progress"_a = nb::none(),
      "Run the Detail stage. Returns 05-detail.fb as bytes. progress, when "
      "given, is called with one line per pass while the stage runs.");

  m.def(
      "route_final",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& global, const nb::bytes& assignment,
         const nb::bytes& detail, const nb::bytes& config,
         const std::string_view producer, const nb::object& progress) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto circuit = mqt::scpd::io::readArtifact(asSpan(global));
        const auto assigned = mqt::scpd::io::readArtifact(asSpan(assignment));
        const auto drawn = mqt::scpd::io::readArtifact(asSpan(detail));
        const auto name =
            mqt::scpd::pipeline::selectedFinalRouter(configuration);
        return asArtifact(mqt::scpd::pipeline::finalRouters().make(name)->run(
                              design, capacityOf(plan), globalOf(circuit),
                              assignmentOf(assigned), detailOf(drawn),
                              configuration, sayTo(progress)),
                          producer);
      },
      "chip"_a, "capacity"_a, "global"_a, "assignment"_a, "detail"_a,
      "config"_a, "producer"_a, "progress"_a = nb::none(),
      "Run the Final stage. Returns 06-final.fb as bytes. progress, when "
      "given, is called with one line per round while the stage runs.");

  m.def(
      "check_final",
      [](const nb::bytes& chip, const nb::bytes& global,
         const nb::bytes& assignment, const nb::bytes& final,
         const nb::bytes& config) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto circuit = mqt::scpd::io::readArtifact(asSpan(global));
        const auto assigned = mqt::scpd::io::readArtifact(asSpan(assignment));
        const auto routed = mqt::scpd::io::readArtifact(asSpan(final));
        const auto* const routing = routed.output.AsFinalRouting();
        if (routing == nullptr) {
          throw std::invalid_argument("the artifact is not a final routing");
        }
        return asReports(mqt::scpd::drc::checkCells(
            viewOf(design, globalOf(circuit), assignmentOf(assigned), *routing),
            *configuration.rules));
      },
      "chip"_a, "global"_a, "assignment"_a, "final"_a, "config"_a,
      "Check a final routing against the design rules. Returns the text of "
      "drc.json.");

  m.def(
      "algorithms",
      [] {
        std::vector<std::pair<std::string, std::vector<std::string>>>
            registered;
        registered.emplace_back(
            "capacity-planner",
            mqt::scpd::pipeline::capacityPlanners().names());
        registered.emplace_back("global-router",
                                mqt::scpd::pipeline::globalRouters().names());
        registered.emplace_back("assigner",
                                mqt::scpd::pipeline::assigners().names());
        registered.emplace_back("corridor-router",
                                mqt::scpd::pipeline::corridorRouters().names());
        registered.emplace_back("detail-router",
                                mqt::scpd::pipeline::detailRouters().names());
        registered.emplace_back("final-router",
                                mqt::scpd::pipeline::finalRouters().names());
        return registered;
      },
      "The implementations this build ships, one list per stage.");

  m.def(
      "set_solver",
      [](const nb::object& solve) {
        if (solve.is_none()) {
          mqt::scpd::milp::setExternalBackend(nullptr);
          return;
        }
        // The external solver never sees the model, only the MPS text of it.
        // That is the whole contract, and it is what makes the two backends
        // interchangeable rather than merely both present.
        mqt::scpd::milp::setExternalBackend(mqt::scpd::milp::makeMpsBackend(
            "gurobi", [solve](const std::string_view mps,
                              const std::vector<std::string>& names,
                              const mqt::scpd::milp::SolveOptions& options) {
              const nb::gil_scoped_acquire gil;
              const auto answer = nb::cast<
                  std::tuple<std::string, double, std::vector<double>>>(
                  solve(nb::str(mps.data(), mps.size()), names,
                        options.timeLimit, options.relativeGap));
              const auto& [status, objective, values] = answer;
              mqt::scpd::milp::Solution solution;
              solution.backend = "gurobi";
              if (status == "optimal") {
                solution.status = mqt::scpd::milp::SolveStatus::Optimal;
              } else if (status == "feasible") {
                solution.status = mqt::scpd::milp::SolveStatus::Feasible;
              } else if (status == "infeasible") {
                solution.status = mqt::scpd::milp::SolveStatus::Infeasible;
              } else if (status == "unbounded") {
                solution.status = mqt::scpd::milp::SolveStatus::Unbounded;
              } else {
                solution.status = mqt::scpd::milp::SolveStatus::Error;
                solution.message = status;
              }
              solution.objective = objective;
              if (solution.hasValues()) {
                solution.values = values;
              }
              return solution;
            }));
      },
      "solve"_a.none(),
      "Register the external solver of this process, or clear it with None. "
      "The callable "
      "receives the MPS text, the variable names in model order, a time limit "
      "and a relative "
      "gap, and returns (status, objective, values).");

  m.def(
      "validate_config",
      [](const nb::bytes& config) {
        return mqt::scpd::design::validate(
            mqt::scpd::io::readConfig(asSpan(config)));
      },
      "config"_a,
      "The problems of a configuration that can be seen without the chip, "
      "empty when there are none.");
}
