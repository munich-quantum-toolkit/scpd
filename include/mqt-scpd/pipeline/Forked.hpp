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

#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

/// The parallel tasks of the Final stage.
///
/// A task runs in a copy of the stage forked off the state it starts from:
/// the copy shares every page of the stage until it writes one, so a task
/// sees exactly that state, whatever its siblings do, and nothing it does
/// reaches the stage but the bytes it hands back. That is what makes the
/// result of a schedule independent of how many tasks run at once.
///
/// How many run at once is one budget for the whole process tree, nested
/// tasks included: a pipe holds a token for every process beyond the first
/// (`threads - 1`), as a make jobserver does. A process waiting on its tasks
/// runs none itself, so it may always keep one task running without a
/// token; every further one takes a token and gives it back when it ends.
namespace mqt::scpd::pipeline::forked {

/// Opens the budget of the stage: `threads` processes at once, at least one.
/// Zero takes the performance cores of the machine. Called once by the
/// stage; the tasks inherit it. Closes the budget when it goes.
class MQT_SCPD_PIPELINE_EXPORT Budget {
public:
  explicit Budget(std::uint32_t threads);
  Budget(const Budget&) = delete;
  Budget& operator=(const Budget&) = delete;
  Budget(Budget&&) = delete;
  Budget& operator=(Budget&&) = delete;
  ~Budget();

  /// The number of processes the open budget allows, or one when none is
  /// open.
  [[nodiscard]] static std::uint32_t threads();

private:
  bool opened_ = false;
};

/// What `threads = 0` stands for: the performance cores, or every core where
/// the machine does not tell them apart.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::uint32_t defaultThreads();

/// Whether this process is a task, forked off the stage or off another task.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT bool inTask();

/// Runs `work(i)` for every `i` below `count`, each in a forked copy, at most
/// as many at once as the budget allows, and returns what each handed back,
/// in the order of `i`. `stop(i, bytes)` is asked of the answers in that
/// order as soon as they are in; once it says true, no task after `i` is
/// started any more and the answers after `i` come back empty.
///
/// A task that dies or hands back nothing whole is run once more on its own;
/// a second failure throws.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<std::string>
map(std::size_t count, const std::function<std::string(std::size_t)>& work,
    const std::function<bool(std::size_t, const std::string&)>& stop = {});

/// The bytes a task hands back, written field by field.
class Writer {
public:
  template <typename T>
    requires std::is_trivially_copyable_v<T>
  void put(const T& value) {
    bytes_.append(reinterpret_cast<const char*>(&value), sizeof(T));
  }
  void put(std::string_view text) {
    put(static_cast<std::uint64_t>(text.size()));
    bytes_.append(text);
  }
  template <typename T>
    requires std::is_trivially_copyable_v<T>
  void put(const std::vector<T>& values) {
    put(static_cast<std::uint64_t>(values.size()));
    bytes_.append(reinterpret_cast<const char*>(values.data()),
                  values.size() * sizeof(T));
  }
  [[nodiscard]] std::string take() { return std::move(bytes_); }

private:
  std::string bytes_;
};

/// Reads back what a `Writer` wrote, in the same order.
class Reader {
public:
  explicit Reader(std::string_view bytes) : bytes_(bytes) {}

  template <typename T>
    requires std::is_trivially_copyable_v<T>
  [[nodiscard]] T get() {
    T value{};
    need(sizeof(T));
    std::memcpy(&value, bytes_.data() + at_, sizeof(T));
    at_ += sizeof(T);
    return value;
  }
  [[nodiscard]] std::string text() {
    const auto size = static_cast<std::size_t>(get<std::uint64_t>());
    need(size);
    std::string out(bytes_.substr(at_, size));
    at_ += size;
    return out;
  }
  template <typename T>
    requires std::is_trivially_copyable_v<T>
  [[nodiscard]] std::vector<T> vector() {
    const auto size = static_cast<std::size_t>(get<std::uint64_t>());
    need(size * sizeof(T));
    std::vector<T> out(size);
    if (size > 0) {
      std::memcpy(out.data(), bytes_.data() + at_, size * sizeof(T));
    }
    at_ += size * sizeof(T);
    return out;
  }
  [[nodiscard]] bool done() const { return at_ == bytes_.size(); }

private:
  void need(const std::size_t size) const {
    if (at_ + size > bytes_.size()) {
      throw std::runtime_error("forked task: answer cut short");
    }
  }
  std::string_view bytes_;
  std::size_t at_ = 0;
};

} // namespace mqt::scpd::pipeline::forked
