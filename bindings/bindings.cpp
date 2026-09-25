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
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/io/Chip.hpp"
#include "mqt-scpd/io/Config.hpp"

#include <nanobind/nanobind.h>
// The type casters below take part through the conversions they enable, not
// through a name the code spells out.
#include <nanobind/stl/string.h>      // IWYU pragma: keep
#include <nanobind/stl/string_view.h> // IWYU pragma: keep
#include <nanobind/stl/vector.h>      // IWYU pragma: keep

#include <cstdint>
#include <span>
#include <string_view>
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
