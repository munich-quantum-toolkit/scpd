/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/Report.hpp"

#include "mqt-scpd/milp/Backend.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {

namespace {

/// The shortest time between two progress updates of one task. Twenty updates
/// a second keep a live display moving without making a fast loop slow.
constexpr auto PROGRESS_INTERVAL = std::chrono::milliseconds(50);

} // namespace

/// The progress last handed out, shared by every copy of a report.
struct Report::State {
  std::string lastTask;
  std::chrono::steady_clock::time_point lastUpdate;
  bool reported = false;
};

Figure verdict(const std::string_view name, const std::uint64_t count) {
  return {.text = std::format("{} {}", name, count),
          .tone = count == 0 ? Tone::Good : Tone::Bad};
}

std::string counted(const std::uint64_t count, const std::string_view noun) {
  return std::format("{} {}{}", count, noun, count == 1 ? "" : "s");
}

Report::Report(const Detail level, Sinks receivers)
    : reportLevel(level),
      sinks(std::make_shared<const Sinks>(std::move(receivers))),
      state(std::make_shared<State>()) {}

bool Report::wants(const Detail detail) const {
  return sinks != nullptr && sinks->line && detail <= reportLevel;
}

void Report::line(const Detail detail, const std::string_view text) const {
  if (wants(detail)) {
    sinks->line(detail, text);
  }
}

void Report::entry(const Detail detail, const std::string_view label,
                   const std::vector<Figure>& figures) const {
  if (sinks != nullptr && sinks->entry && detail <= reportLevel) {
    sinks->entry(detail, label, figures);
  }
}

void Report::result(const std::vector<Figure>& figures,
                    const std::optional<std::uint64_t> fails) const {
  if (sinks != nullptr && sinks->result) {
    sinks->result(figures, fails);
  }
}

void Report::progress(const std::string_view task,
                      const std::string_view detail, const std::uint64_t done,
                      const std::uint64_t total) const {
  if (sinks == nullptr || !sinks->progress) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool newTask = !state->reported || task != state->lastTask;
  const bool finished = total != 0 && done >= total;
  if (!newTask && !finished && now - state->lastUpdate < PROGRESS_INTERVAL) {
    return;
  }
  state->reported = true;
  state->lastTask = task;
  state->lastUpdate = now;
  sinks->progress(task, detail, done, total);
}

milp::SolveObserver Report::solveObserver(const std::string_view task) const {
  milp::SolveObserver observer;
  if (wants(Detail::Solver)) {
    observer.log = [report = *this](const std::string_view text) {
      report.line(Detail::Solver, text);
    };
  }
  if (sinks != nullptr && sinks->progress) {
    observer.progress = [report = *this, name = std::string(task)](
                            const milp::SolveProgress& search) {
      std::string detail;
      const auto append = [&detail](const std::string& figure) {
        detail += detail.empty() ? "" : FIGURE_SEPARATOR;
        detail += figure;
      };
      if (!std::isnan(search.objective)) {
        append(std::format("objective {:.2f}", search.objective));
      }
      if (!std::isnan(search.gap)) {
        append(std::format("gap {:.1f} %", search.gap * 100.0));
      }
      append(counted(search.nodes, "node"));
      report.progress(name, detail, 0, 0);
    };
  }
  return observer;
}

} // namespace mqt::scpd::pipeline
