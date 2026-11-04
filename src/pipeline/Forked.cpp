/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/Forked.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <format>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif
#ifdef __APPLE__
#include <sys/sysctl.h>
#endif

namespace mqt::scpd::pipeline::forked {
namespace {

/// The budget of the process tree: the jobserver pipe and how many processes
/// it allows. A task inherits both with the rest of the stage.
int tokensIn = -1;
int tokensOut = -1;
std::uint32_t budgetThreads = 0;
/// How deep in forked tasks this process is; zero for the stage itself.
std::uint32_t depth = 0;

/// The end of a task's answer: its length and a mark, so that an answer cut
/// short by a task that died is told from a whole one.
constexpr std::uint32_t MARK = 0x5CD0F0C4U;

[[nodiscard]] bool takeToken() {
#ifndef _WIN32
  if (tokensIn < 0) {
    return false;
  }
  char token = 0;
  while (true) {
    const auto got = ::read(tokensIn, &token, 1);
    if (got == 1) {
      return true;
    }
    if (got < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
#else
  return false;
#endif
}

void giveToken() {
#ifndef _WIN32
  if (tokensOut < 0) {
    return;
  }
  const char token = '+';
  while (::write(tokensOut, &token, 1) < 0 && errno == EINTR) {
  }
#endif
}

#ifndef _WIN32
void writeAll(const int fd, const char* data, std::size_t size) {
  while (size > 0) {
    const auto put = ::write(fd, data, size);
    if (put < 0) {
      if (errno == EINTR) {
        continue;
      }
      ::_exit(4);
    }
    data += put;
    size -= static_cast<std::size_t>(put);
  }
}

struct Running {
  pid_t pid = -1;
  int fd = -1;
  std::size_t index = 0;
  bool token = false;
  std::string bytes;
};

/// Forks the task `index`: the copy runs `work`, writes the answer and its
/// end mark into a pipe and exits without unwinding anything of the stage.
[[nodiscard]] Running
launch(const std::size_t index,
       const std::function<std::string(std::size_t)>& work,
       const std::vector<Running>& running) {
  int ends[2] = {-1, -1};
  if (::pipe(ends) != 0) {
    throw std::runtime_error("forked task: no pipe");
  }
  std::fflush(nullptr);
  const pid_t pid = ::fork();
  if (pid < 0) {
    ::close(ends[0]);
    ::close(ends[1]);
    throw std::runtime_error("forked task: fork failed");
  }
  if (pid == 0) {
    ::close(ends[0]);
    for (const auto& other : running) {
      ::close(other.fd);
    }
    ++depth;
    std::string answer;
    try {
      answer = work(index);
    } catch (...) {
      ::_exit(3);
    }
    const auto size = static_cast<std::uint64_t>(answer.size());
    answer.append(reinterpret_cast<const char*>(&size), sizeof(size));
    answer.append(reinterpret_cast<const char*>(&MARK), sizeof(MARK));
    writeAll(ends[1], answer.data(), answer.size());
    ::close(ends[1]);
    ::_exit(0);
  }
  ::close(ends[1]);
  Running child;
  child.pid = pid;
  child.fd = ends[0];
  child.index = index;
  return child;
}

/// Whether what a task wrote is whole, and its answer without the end.
[[nodiscard]] bool whole(std::string& bytes, const int status) {
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    return false;
  }
  constexpr auto TAIL = sizeof(std::uint64_t) + sizeof(MARK);
  if (bytes.size() < TAIL) {
    return false;
  }
  std::uint64_t size = 0;
  std::uint32_t mark = 0;
  std::memcpy(&size, bytes.data() + bytes.size() - TAIL, sizeof(size));
  std::memcpy(&mark, bytes.data() + bytes.size() - sizeof(MARK), sizeof(mark));
  if (mark != MARK || size != bytes.size() - TAIL) {
    return false;
  }
  bytes.resize(static_cast<std::size_t>(size));
  return true;
}

/// Reads a task's pipe to its end and reaps it.
[[nodiscard]] int drain(Running& child) {
  char chunk[1 << 16];
  while (true) {
    const auto got = ::read(child.fd, chunk, sizeof(chunk));
    if (got > 0) {
      child.bytes.append(chunk, static_cast<std::size_t>(got));
      continue;
    }
    if (got < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
  ::close(child.fd);
  child.fd = -1;
  int status = 0;
  while (::waitpid(child.pid, &status, 0) < 0 && errno == EINTR) {
  }
  return status;
}
#endif

} // namespace

std::uint32_t defaultThreads() {
#ifdef __APPLE__
  int cores = 0;
  std::size_t size = sizeof(cores);
  if (::sysctlbyname("hw.perflevel0.logicalcpu", &cores, &size, nullptr, 0) ==
          0 &&
      cores > 0) {
    return static_cast<std::uint32_t>(cores);
  }
#endif
  const auto all = std::thread::hardware_concurrency();
  return all > 0 ? all : 4U;
}

Budget::Budget(const std::uint32_t threads) {
#ifndef _WIN32
  if (budgetThreads != 0 || depth != 0) {
    return;
  }
  const auto wanted =
      std::clamp(threads == 0 ? defaultThreads() : threads, 1U, 64U);
  int ends[2] = {-1, -1};
  if (::pipe(ends) != 0) {
    return;
  }
  ::fcntl(ends[0], F_SETFL, ::fcntl(ends[0], F_GETFL) | O_NONBLOCK);
  ::fcntl(ends[0], F_SETFD, FD_CLOEXEC);
  ::fcntl(ends[1], F_SETFD, FD_CLOEXEC);
  tokensIn = ends[0];
  tokensOut = ends[1];
  budgetThreads = wanted;
  for (std::uint32_t token = 1; token < wanted; ++token) {
    giveToken();
  }
  opened_ = true;
#else
  (void)threads;
#endif
}

Budget::~Budget() {
#ifndef _WIN32
  if (!opened_) {
    return;
  }
  ::close(tokensIn);
  ::close(tokensOut);
  tokensIn = -1;
  tokensOut = -1;
  budgetThreads = 0;
#endif
}

std::uint32_t Budget::threads() {
  return budgetThreads == 0 ? 1U : budgetThreads;
}

bool inTask() { return depth != 0; }

std::vector<std::string>
map(const std::size_t count,
    const std::function<std::string(std::size_t)>& work,
    const std::function<bool(std::size_t, const std::string&)>& stop) {
  std::vector<std::string> answers(count);
#ifdef _WIN32
  // No fork: the tasks run one after the other in the stage itself.
  for (std::size_t index = 0; index < count; ++index) {
    answers[index] = work(index);
    if (stop && stop(index, answers[index])) {
      break;
    }
  }
  return answers;
#else
  std::vector<char> answered(count, 0);
  std::vector<Running> running;
  std::size_t next = 0;
  std::size_t asked = 0;
  std::size_t end = count;
  // A task that failed runs once more, alone; a second failure throws.
  const auto again = [&](Running failed) {
    auto child = launch(failed.index, work, running);
    const auto status = drain(child);
    if (!whole(child.bytes, status)) {
      for (auto& other : running) {
        ::kill(other.pid, SIGKILL);
        (void)drain(other);
        if (other.token) {
          giveToken();
        }
      }
      throw std::runtime_error(std::format(
          "forked task {} failed twice (status {})", failed.index, status));
    }
    return std::move(child.bytes);
  };
  while (true) {
    while (next < end) {
      bool token = false;
      if (!running.empty()) {
        if (!takeToken()) {
          break;
        }
        token = true;
      }
      auto child = launch(next++, work, running);
      child.token = token;
      running.push_back(std::move(child));
    }
    if (running.empty()) {
      break;
    }
    std::vector<pollfd> polls;
    polls.reserve(running.size());
    for (const auto& child : running) {
      polls.push_back({child.fd, POLLIN, 0});
    }
    if (::poll(polls.data(), polls.size(), -1) < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error("forked task: poll failed");
    }
    for (std::size_t at = running.size(); at-- > 0;) {
      if ((polls[at].revents & (POLLIN | POLLHUP | POLLERR)) == 0) {
        continue;
      }
      auto& child = running[at];
      char chunk[1 << 16];
      const auto got = ::read(child.fd, chunk, sizeof(chunk));
      if (got > 0) {
        child.bytes.append(chunk, static_cast<std::size_t>(got));
        continue;
      }
      if (got < 0 && errno == EINTR) {
        continue;
      }
      auto done = std::move(child);
      running.erase(running.begin() + static_cast<std::ptrdiff_t>(at));
      const auto status = drain(done);
      if (done.token) {
        giveToken();
      }
      if (!whole(done.bytes, status)) {
        done.bytes = again(std::move(done));
      }
      answers[done.index] = std::move(done.bytes);
      answered[done.index] = 1;
    }
    while (asked < end && answered[asked] != 0) {
      if (stop && stop(asked, answers[asked])) {
        end = asked + 1;
      }
      ++asked;
    }
  }
  for (std::size_t index = end; index < count; ++index) {
    answers[index].clear();
  }
  return answers;
#endif
}

} // namespace mqt::scpd::pipeline::forked
