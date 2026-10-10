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
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Solver.hpp"

#include <nanobind/nanobind.h>
// The type casters below take part through the conversions they enable, not
// through a name the code spells out.
#include <nanobind/stl/pair.h>        // IWYU pragma: keep
#include <nanobind/stl/string.h>      // IWYU pragma: keep
#include <nanobind/stl/string_view.h> // IWYU pragma: keep
#include <nanobind/stl/tuple.h>       // IWYU pragma: keep
#include <nanobind/stl/vector.h>      // IWYU pragma: keep

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
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

/// One stage output, wrapped as the artifact a run directory stores. The
/// producer comes from the caller, because the version of the package is what
/// Python knows.
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

/// The name of a tone, as the Python side spells it.
const char* toneName(const mqt::scpd::pipeline::Tone tone) {
  switch (tone) {
  case mqt::scpd::pipeline::Tone::Good:
    return "good";
  case mqt::scpd::pipeline::Tone::Bad:
    return "bad";
  case mqt::scpd::pipeline::Tone::Plain:
    break;
  }
  return "plain";
}

/// Figures as a Python list of (text, tone) pairs.
nb::list figuresOf(const std::vector<mqt::scpd::pipeline::Figure>& figures) {
  nb::list list;
  for (const auto& figure : figures) {
    list.append(nb::make_tuple(figure.text, toneName(figure.tone)));
  }
  return list;
}

/// The report of a stage, handed to a Python object with the methods line,
/// entry, result and progress, or a report that drops everything for None.
///
/// The stage runs with the GIL released, so every call takes it back first.
/// The callables only refer to the object, which the caller keeps alive for
/// the whole call; copying it would touch its reference count without the GIL.
mqt::scpd::pipeline::Report reportTo(const nb::object& reporter,
                                     const int verbosity) {
  using mqt::scpd::pipeline::Detail;
  using mqt::scpd::pipeline::Figure;
  if (reporter.is_none()) {
    return {};
  }
  const auto level = static_cast<Detail>(std::clamp(verbosity, 0, 3));
  return {
      level,
      {.line =
           [&reporter](const Detail detail, const std::string_view text) {
             const nb::gil_scoped_acquire gil;
             reporter.attr("line")(static_cast<int>(detail),
                                   nb::str(text.data(), text.size()));
           },
       .entry =
           [&reporter](const Detail detail, const std::string_view label,
                       const std::vector<Figure>& figures) {
             const nb::gil_scoped_acquire gil;
             reporter.attr("entry")(static_cast<int>(detail),
                                    nb::str(label.data(), label.size()),
                                    figuresOf(figures));
           },
       .result =
           [&reporter](const std::vector<Figure>& figures,
                       const std::optional<std::uint64_t> fails) {
             const nb::gil_scoped_acquire gil;
             reporter.attr("result")(figuresOf(figures), fails.has_value()
                                                             ? nb::cast(*fails)
                                                             : nb::none());
           },
       .progress =
           [&reporter](const std::string_view task,
                       const std::string_view detail, const std::uint64_t done,
                       const std::uint64_t total) {
             const nb::gil_scoped_acquire gil;
             reporter.attr("progress")(nb::str(task.data(), task.size()),
                                       nb::str(detail.data(), detail.size()),
                                       done, total);
           }}};
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
      "validate_config",
      [](const nb::bytes& config) {
        return mqt::scpd::design::validate(
            mqt::scpd::io::readConfig(asSpan(config)));
      },
      "config"_a,
      "The problems of a configuration that can be seen without the chip, "
      "empty when there are none.");

  m.def(
      "plan_capacity",
      [](const nb::bytes& chip, const nb::bytes& config,
         const std::string_view producer, const nb::object& reporter,
         const int verbosity) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto report = reportTo(reporter, verbosity);
        const auto planner = mqt::scpd::pipeline::capacityPlanners().make(
            mqt::scpd::pipeline::selectedCapacityPlanner(configuration));
        mqt::scpd::flatbuffers::artifacts::CapacityPlanT plan;
        {
          const nb::gil_scoped_release release;
          plan = planner->run(design, configuration, report);
        }
        return asArtifact(std::move(plan), producer);
      },
      "chip"_a, "config"_a, "producer"_a, "reporter"_a.none() = nb::none(),
      "verbosity"_a = 0,
      "Run the Capacity stage on a classified chip. The reporter receives "
      "what the stage reports, through its methods line, entry, result and "
      "progress. Returns 01-capacity.fb as bytes.");

  m.def(
      "route_global",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& config, const std::string_view producer,
         const nb::object& reporter, const int verbosity) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto report = reportTo(reporter, verbosity);
        const auto router = mqt::scpd::pipeline::globalRouters().make(
            mqt::scpd::pipeline::selectedGlobalRouter(configuration));
        mqt::scpd::flatbuffers::artifacts::GlobalRoutingT routing;
        {
          const nb::gil_scoped_release release;
          routing =
              router->run(design, capacityOf(plan), configuration, report);
        }
        return asArtifact(std::move(routing), producer);
      },
      "chip"_a, "capacity"_a, "config"_a, "producer"_a,
      "reporter"_a.none() = nb::none(), "verbosity"_a = 0,
      "Run the Global stage on a classified chip and its 01-capacity.fb. The "
      "reporter receives what the stage reports, as for plan_capacity. "
      "Returns 02-global.fb as bytes.");

  m.def(
      "assign",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& globalRouting, const nb::bytes& config,
         const std::string_view producer, const nb::object& reporter,
         const int verbosity) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto routing = mqt::scpd::io::readArtifact(asSpan(globalRouting));
        const auto report = reportTo(reporter, verbosity);
        const auto assigner = mqt::scpd::pipeline::assigners().make(
            mqt::scpd::pipeline::selectedAssigner(configuration));
        mqt::scpd::flatbuffers::artifacts::AssignmentT assignment;
        {
          const nb::gil_scoped_release release;
          assignment = assigner->run(design, capacityOf(plan),
                                     globalOf(routing), configuration, report);
        }
        return asArtifact(std::move(assignment), producer);
      },
      "chip"_a, "capacity"_a, "global_routing"_a, "config"_a, "producer"_a,
      "reporter"_a.none() = nb::none(), "verbosity"_a = 0,
      "Run the Assignment stage on a classified chip, its 01-capacity.fb and "
      "its 02-global.fb. The reporter receives what the stage reports, as for "
      "plan_capacity. Returns 03-assign.fb as bytes.");

  m.def(
      "route_corridor",
      [](const nb::bytes& chip, const nb::bytes& capacity,
         const nb::bytes& assignment, const nb::bytes& config,
         const std::string_view producer, const nb::object& reporter,
         const int verbosity) {
        const auto configuration = mqt::scpd::io::readConfig(asSpan(config));
        const auto design = mqt::scpd::io::readChip(asSpan(chip));
        const auto plan = mqt::scpd::io::readArtifact(asSpan(capacity));
        const auto assigned = mqt::scpd::io::readArtifact(asSpan(assignment));
        const auto report = reportTo(reporter, verbosity);
        const auto router = mqt::scpd::pipeline::corridorRouters().make(
            mqt::scpd::pipeline::selectedCorridorRouter(configuration));
        mqt::scpd::flatbuffers::artifacts::CorridorRoutingT routing;
        {
          const nb::gil_scoped_release release;
          routing = router->run(design, capacityOf(plan),
                                assignmentOf(assigned), configuration, report);
        }
        return asArtifact(std::move(routing), producer);
      },
      "chip"_a, "capacity"_a, "assignment"_a, "config"_a, "producer"_a,
      "reporter"_a.none() = nb::none(), "verbosity"_a = 0,
      "Run the Corridor stage on a classified chip, its 01-capacity.fb and "
      "its 03-assign.fb. The reporter receives what the stage reports, as for "
      "plan_capacity. Returns 04-corridor.fb as bytes.");

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
      "The implementations this build ships, as (stage, names) pairs in "
      "pipeline order.");

  m.def(
      "set_solver",
      [](const nb::object& solve) {
        if (solve.is_none()) {
          mqt::scpd::milp::setExternalBackend(nullptr);
          return;
        }
        // The external solver sees the MPS text of the model and nothing
        // else, which is what makes the two backends interchangeable.
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
      "Register the external solver of the process, or clear it with None. "
      "The callable receives the MPS text, the variable names in model "
      "order, a time limit and a relative gap, and returns (status, "
      "objective, values).");

  m.def(
      "solver_info",
      [](const nb::bytes& config) {
        return mqt::scpd::pipeline::solverName(
            mqt::scpd::io::readConfig(asSpan(config)));
      },
      "config"_a,
      "The backend a solve with this configuration uses: HiGHS and its "
      "version, or the name of the external backend.");

  m.def(
      "artifact_to_json",
      [](const nb::bytes& artifact) {
        return mqt::scpd::io::artifactToJson(asSpan(artifact));
      },
      "artifact"_a,
      "Render a stage artifact as JSON. The field names, the enum names and "
      "the union tags come from the schema, so the JSON follows it without a "
      "second description of the model. Raises ValueError when the bytes are "
      "not a complete artifact of this schema version.");
}
