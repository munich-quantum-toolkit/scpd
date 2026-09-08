/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/drc/Validation.hpp"

#include "mqt-scpd/flatbuffers/drc.hpp"

#include <cstddef>
#include <string>

namespace mqt::scpd::drc {

namespace {

using flatbuffers::drc::ClearanceKind;
using flatbuffers::drc::DrcFindingT;
using flatbuffers::drc::DrcRule;
using flatbuffers::drc::DrcSeverity;
using flatbuffers::drc::DrcStage;

void validateFinding(const DrcFindingT& finding, const std::string& prefix,
                     Problems& problems) {
  if (finding.rule == DrcRule::Unset) {
    problems.push_back(prefix + "rule is unset");
  }
  if (finding.severity == DrcSeverity::Unset) {
    problems.push_back(prefix + "severity is unset");
  }
  if (finding.wires.empty()) {
    problems.push_back(prefix + "names no wire");
  }
  if (!(finding.limit > 0.0)) {
    problems.push_back(prefix + "limit must be positive");
  }
  const bool clearance = finding.rule == DrcRule::WireClearance;
  if (clearance && finding.clearance_kind == ClearanceKind::Unset) {
    problems.push_back(prefix + "clearance kind is unset");
  }
  if (!clearance && finding.clearance_kind != ClearanceKind::Unset) {
    problems.push_back(prefix +
                       "clearance kind is set for a rule other than clearance");
  }
}

} // namespace

Problems validate(const flatbuffers::drc::DrcReportsT& reports) {
  Problems problems;
  for (std::size_t i = 0; i < reports.reports.size(); ++i) {
    const std::string prefix = "report " + std::to_string(i) + ": ";
    const auto* const report = reports.reports[i].get();
    if (report == nullptr) {
      problems.push_back(prefix + "missing");
      continue;
    }
    if (report->stage == DrcStage::Unset) {
      problems.push_back(prefix + "stage is unset");
    }
    for (std::size_t j = 0; j < report->findings.size(); ++j) {
      const std::string finding = prefix + "finding " + std::to_string(j) + " ";
      if (report->findings[j] == nullptr) {
        problems.push_back(finding + "missing");
        continue;
      }
      validateFinding(*report->findings[j], finding, problems);
    }
  }
  return problems;
}

} // namespace mqt::scpd::drc
