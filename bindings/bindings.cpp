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
#include "mqt-scpd/pipeline/Registry.hpp"
#include "mqt-scpd/pipeline/Report.hpp"

#include <nanobind/nanobind.h>
// The type casters below take part through the conversions they enable, not
// through a name the code spells out.
#include <nanobind/stl/pair.h>        // IWYU pragma: keep
#include <nanobind/stl/string.h>      // IWYU pragma: keep
#include <nanobind/stl/string_view.h> // IWYU pragma: keep
#include <nanobind/stl/vector.h>      // IWYU pragma: keep

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
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

/// The name of a tone, as the Python side spells it.
const char* toneName(const mqt::scpd::pipeline::Tone tone) {
  switch (tone) {
  case mqt::scpd::pipeline::Tone::Good:
    return "good";
  case mqt::scpd::pipeline::Tone::Bad:
    return "bad";
  case mqt::scpd::pipeline::Tone::Plain:
    return "plain";
  }
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
/// entry, result and progress. For None, the report drops everything and only
/// gives Python the chance to raise a pending interrupt where the stage
/// reports progress, so that Ctrl-C stops a stage without a reporter too.
///
/// The stage runs with the GIL released, so every call takes it back first.
/// The callables only refer to the object, which the caller keeps alive for
/// the whole call; copying it would touch its reference count without the GIL.
mqt::scpd::pipeline::Report reportTo(const nb::object& reporter,
                                     const int verbosity) {
  using mqt::scpd::pipeline::Detail;
  using mqt::scpd::pipeline::Figure;
  if (reporter.is_none()) {
    return {Detail::Summary,
            {.progress = [](const std::string_view /*task*/,
                            const std::string_view /*detail*/,
                            const std::uint64_t /*done*/,
                            const std::uint64_t /*total*/) {
              const nb::gil_scoped_acquire gil;
              // Python.h, which nanobind includes, declares the call in a
              // header of its own that an extension does not include itself.
              // NOLINTNEXTLINE(misc-include-cleaner)
              if (PyErr_CheckSignals() != 0) {
                throw nb::python_error();
              }
            }}};
  }
  const auto level = static_cast<Detail>(std::clamp(verbosity, 0, 2));
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
      "plan_problems",
      [](const nb::bytes& chip, const nb::bytes& config,
         const std::vector<std::string>& steps) {
        return mqt::scpd::pipeline::planProblems(
            mqt::scpd::io::readChip(asSpan(chip)),
            mqt::scpd::io::readConfig(asSpan(config)), steps);
      },
      "chip"_a, "config"_a, "steps"_a,
      "What would stop a run of the given steps on a classified chip, before "
      "any stage runs: an unknown implementation name or a detail grid above "
      "the cell limit. Empty when there is nothing.");

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
      "algorithms",
      [] {
        std::vector<std::pair<std::string, std::vector<std::string>>>
            registered;
        registered.emplace_back(
            "capacity-planner",
            mqt::scpd::pipeline::capacityPlanners().names());
        return registered;
      },
      "The implementations this build ships, as (stage, names) pairs in "
      "pipeline order.");

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
