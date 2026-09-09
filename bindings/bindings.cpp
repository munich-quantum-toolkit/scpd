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
#include "mqt-scpd/flatbuffers/artifacts.hpp"
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
         const std::string_view producer) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto assigned = mqt::scpd::io::readArtifact(asSpan(assignment));
        const auto name =
            mqt::scpd::pipeline::selectedCorridorRouter(configuration);
        return asArtifact(
            mqt::scpd::pipeline::corridorRouters().make(name)->run(
                design, capacityOf(plan), assignmentOf(assigned),
                configuration),
            producer);
      },
      "chip"_a, "capacity"_a, "assignment"_a, "config"_a, "producer"_a,
      "Run the Corridor stage. Returns 04-corridor.fb as bytes.");

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
