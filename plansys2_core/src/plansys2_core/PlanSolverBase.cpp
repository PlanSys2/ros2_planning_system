// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "plansys2_core/PlanSolverBase.hpp"

#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cstring>
#include <iomanip>
#include <optional>
#include <vector>

using namespace std::chrono_literals;  // NOLINT

extern char ** environ;

namespace plansys2
{

char ** PlanSolverBase::tokenize(const std::string & command)
{
  std::vector<std::string> tokens;
  std::istringstream stream(command);
  std::string token;

  while (stream >> std::quoted(token)) {
    tokens.push_back(token);
  }

  char ** argv = new char *[tokens.size() + 2];

  for (size_t i = 0; i < tokens.size(); ++i) {
    argv[i] = new char[tokens[i].size() + 1];
    std::snprintf(argv[i], tokens[i].size() + 1, "%s", tokens[i].c_str());
  }

  argv[tokens.size()] = nullptr;  // Null-terminate the array

  return argv;
}

namespace
{

// Writes the whole buffer, retrying on short writes and EINTR
bool write_all(int fd, const char * data, size_t size)
{
  while (size > 0) {
    ssize_t written = write(fd, data, size);
    if (written < 0) {
      if (errno == EINTR) {continue;}
      return false;
    }
    data += written;
    size -= static_cast<size_t>(written);
  }
  return true;
}

}  // namespace

bool PlanSolverBase::execute_planner(
  const std::string & command, const rclcpp::Duration & solver_timeout,
  const std::string & plan_path)
{
  cancel_requested_ = false;
  auto logger = lc_node_->get_logger();

  // argv is built before spawning: nothing but exec runs in the child
  std::vector<std::string> args;
  {
    std::istringstream stream(command);
    std::string token;
    while (stream >> std::quoted(token)) {
      args.push_back(token);
    }
  }
  if (args.empty()) {
    RCLCPP_ERROR(logger, "Planner command is empty");
    return false;
  }
  std::vector<char *> argv;
  for (auto & arg : args) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);

  int output_fd = open(plan_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (output_fd == -1) {
    RCLCPP_ERROR(logger, "Cannot open %s: %s", plan_path.c_str(), std::strerror(errno));
    return false;
  }

  int pipe_fd[2];
  if (pipe2(pipe_fd, O_CLOEXEC) == -1) {
    RCLCPP_ERROR(logger, "Cannot create pipe for the planner: %s", std::strerror(errno));
    close(output_fd);
    return false;
  }

  // The planner gets its own process group, so on timeout or cancel the whole
  // group dies, including grandchildren (e.g. popf under `ros2 run`)
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attr;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, pipe_fd[1], STDOUT_FILENO);
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attr, 0);

  pid_t pid = -1;
  int spawn_error = posix_spawnp(&pid, argv[0], &actions, &attr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attr);
  close(pipe_fd[1]);

  if (spawn_error != 0 || pid <= 0) {
    RCLCPP_ERROR(
      logger, "Cannot run planner command [%s]: %s", command.c_str(), std::strerror(spawn_error));
    close(pipe_fd[0]);
    close(output_fd);
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::nanoseconds(solver_timeout.nanoseconds());
  // Set when the group has been killed: the pipe is then only drained for a while,
  // as a process that left the group could keep it open forever
  std::optional<std::chrono::steady_clock::time_point> drain_since;
  bool killed = false;
  bool timed_out = false;
  bool write_ok = true;
  bool eof = false;

  // Checks whether the planner has exited without reaping it, so its pid (and
  // process group id) cannot be reused while we may still signal the group
  auto planner_exited = [pid]() {
      siginfo_t info{};
      return waitid(P_PID, pid, &info, WEXITED | WNOHANG | WNOWAIT) == 0 && info.si_pid == pid;
    };

  while (true) {
    if (!drain_since && (cancel_requested_ || std::chrono::steady_clock::now() >= deadline)) {
      timed_out = !cancel_requested_;
      kill(-pid, SIGKILL);  // pid > 0 and not reaped yet: only its group gets the signal
      killed = true;
      drain_since = std::chrono::steady_clock::now();
    }

    if (eof) {
      if (planner_exited()) {
        break;
      }
      std::this_thread::sleep_for(10ms);
      continue;
    }

    if (drain_since && std::chrono::steady_clock::now() - *drain_since > 1s) {
      eof = true;
      continue;
    }

    struct pollfd pfd = {pipe_fd[0], POLLIN, 0};
    int ready = poll(&pfd, 1, 100);
    if (ready > 0) {
      char buffer[4096];
      ssize_t bytes_read = read(pipe_fd[0], buffer, sizeof(buffer));
      if (bytes_read > 0) {
        if (write_ok && !write_all(output_fd, buffer, static_cast<size_t>(bytes_read))) {
          RCLCPP_ERROR(logger, "Cannot write %s: %s", plan_path.c_str(), std::strerror(errno));
          write_ok = false;
        }
      } else if (bytes_read == 0 || errno != EINTR) {
        eof = true;
      }
    } else if (ready < 0 && errno != EINTR) {
      eof = true;
    } else if (ready == 0 && !drain_since && planner_exited()) {
      // The planner is done but something it left behind still holds the pipe
      kill(-pid, SIGKILL);
      drain_since = std::chrono::steady_clock::now();
    }
  }

  close(pipe_fd[0]);
  if (close(output_fd) == -1) {
    write_ok = false;
  }

  // Leftover processes of the planner group, if any, go away with it
  kill(-pid, SIGKILL);

  int status = 0;
  while (waitpid(pid, &status, 0) == -1 && errno == EINTR) {}

  if (killed) {
    if (timed_out) {
      RCLCPP_WARN(
        logger, "Planner timed out after %.2f seconds", solver_timeout.seconds());
    } else {
      RCLCPP_DEBUG(logger, "Planner terminated by cancel request");
    }
    return false;
  }

  if (WIFSIGNALED(status)) {
    RCLCPP_ERROR(logger, "Planner terminated by signal %d", WTERMSIG(status));
    return false;
  }

  if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
    RCLCPP_DEBUG(logger, "Planner exited with status %d", WEXITSTATUS(status));
    return false;
  }

  return write_ok;
}

}  // namespace plansys2
