/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::pipeline {

/// How much a report says. The levels match how often the command line is
/// given -v.
enum class Detail : std::uint8_t {
  /// What a run says without -v: the result of a stage and its rounds.
  Summary = 0,
  /// With -v: the parameters, the steps of a stage and the result of a solve.
  Steps = 1,
  /// With -vv: one line per item, such as a chain or a connection.
  Items = 2,
  /// With -vvv: the own log of the solver.
  Solver = 3,
};

/// What joins two figures in one line of text: a middle dot between two
/// spaces. The bytes are those of U+00B7 in UTF-8, written out so that the
/// text does not depend on the character set the compiler assumes for the
/// source.
inline constexpr std::string_view FIGURE_SEPARATOR = " \xC2\xB7 ";

/// The sign between the two sides of a size, such as 12×12: U+00D7 in UTF-8.
inline constexpr std::string_view TIMES = "\xC3\x97";

/// The sign between the two ends of a connection, such as Q1.port1 → Q2.port1:
/// U+2192 in UTF-8.
inline constexpr std::string_view ARROW = "\xE2\x86\x92";

/// How a renderer colours a figure.
enum class Tone : std::uint8_t {
  /// A figure without a verdict.
  Plain,
  /// A verdict that holds, such as zero fails.
  Good,
  /// A verdict that does not hold.
  Bad,
};

/// One figure of a result or an entry, such as "363 partitions",
/// "objective 27.84" or "fails 2". The text follows the wording rules of the
/// terminal output; the renderer only lays it out and colours it.
struct Figure {
  /// The figure as it is shown.
  std::string text;
  /// How it is coloured.
  Tone tone = Tone::Plain;
};

/**
 * @brief Makes the figure of a count that should be zero.
 * @param name What is counted, such as "fails" or "unrouted".
 * @param count The count.
 * @return The figure "name count", good at zero and bad otherwise.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT Figure verdict(std::string_view name,
                                                      std::uint64_t count);

/**
 * @brief Makes the text of a count, such as "363 partitions" or "1 node".
 * @param count The count.
 * @param noun What is counted, in the singular. The plural adds an s.
 * @return The count and the noun, in the singular only for a count of one.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string
counted(std::uint64_t count, std::string_view noun);

/**
 * @brief Where a stage reports while it runs.
 *
 * The core never prints. A stage hands each line, entry, result and progress
 * update to the caller through this report, and the caller decides what is
 * shown and how. A report built without sinks drops everything.
 *
 * Copies share their state, so a stage may pass the report on by value.
 */
class MQT_SCPD_PIPELINE_EXPORT Report {
public:
  /// The receivers of what a stage reports. Each may be empty.
  struct Sinks {
    /// Receives a line of free text and its level.
    std::function<void(Detail, std::string_view)> line;
    /// Receives an entry: a label, such as "round 1  forward", and its
    /// figures, which a renderer aligns with those of the other entries.
    std::function<void(Detail, std::string_view, const std::vector<Figure>&)>
        entry;
    /// Receives the result of the stage and, for a stage that can fail on
    /// part of its work, how many parts failed.
    std::function<void(const std::vector<Figure>&,
                       std::optional<std::uint64_t>)>
        result;
    /// Receives progress: the task, a short detail, and how much of it is
    /// done out of a total, zero for a task without a known total.
    std::function<void(std::string_view, std::string_view, std::uint64_t,
                       std::uint64_t)>
        progress;
  };

  /**
   * @brief Creates a report that drops everything.
   */
  Report() = default;

  /**
   * @brief Creates a report.
   * @param level The most detailed level that reaches the sinks.
   * @param receivers The receivers.
   */
  Report(Detail level, Sinks receivers);

  /**
   * @brief Checks whether a line of a level would reach its sink.
   *
   * A stage asks before it builds a line that costs time to build.
   *
   * @param detail The level of the line.
   * @return @c true when a line sink is set and @p detail is not above the
   * level of the report.
   */
  [[nodiscard]] bool wants(Detail detail) const;

  /**
   * @brief Reports a line of free text.
   * @param detail The level of the line.
   * @param text The line, without a line break.
   */
  void line(Detail detail, std::string_view text) const;

  /**
   * @brief Reports an entry, such as one round of a stage.
   * @param detail The level of the entry.
   * @param label What the entry is about.
   * @param figures Its figures.
   */
  void entry(Detail detail, std::string_view label,
             const std::vector<Figure>& figures) const;

  /**
   * @brief Reports the result of the stage. A stage reports it once, at its
   * end, at every level.
   * @param figures The figures of the result.
   * @param fails For a stage that can fail on part of its work, how many
   * parts failed.
   */
  void result(const std::vector<Figure>& figures,
              std::optional<std::uint64_t> fails = std::nullopt) const;

  /**
   * @brief Reports progress.
   *
   * Progress reaches its sink at most about twenty times a second. The first
   * update of a task, and the update that completes it, always reach it.
   *
   * @param task What the stage is doing.
   * @param detail A short detail, or an empty one.
   * @param done How much of the task is done.
   * @param total The whole task, or zero for a task without a known total.
   */
  void progress(std::string_view task, std::string_view detail,
                std::uint64_t done, std::uint64_t total) const;

  /**
   * @brief Makes the observer through which a solve reports.
   *
   * The solver log reaches the line sink at the Solver level, and only when
   * the report is that detailed. The state of the search reaches the progress
   * sink, under @p task, as its objective, gap and node count.
   *
   * @param task What the solve is for, such as "solving the assignment".
   * @return The observer.
   */
  [[nodiscard]] milp::SolveObserver solveObserver(std::string_view task) const;

private:
  struct State;
  Detail reportLevel = Detail::Summary;
  std::shared_ptr<const Sinks> sinks;
  std::shared_ptr<State> state;
};

} // namespace mqt::scpd::pipeline
