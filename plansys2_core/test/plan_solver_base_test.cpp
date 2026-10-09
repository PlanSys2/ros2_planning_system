// Copyright 2026 Intelligent Robotics Lab
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

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "plansys2_core/PlanSolverBase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT

class TestSolver : public plansys2::PlanSolverBase
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, const std::string &) override
  {
    lc_node_ = lc_node;
  }

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string &, const std::string &,
    const std::string & = "", const rclcpp::Duration = 15s) override
  {
    return {};
  }

  bool isDomainValid(const std::string &, const std::string & = "") override {return true;}
};

class PlanSolverBaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("plan_solver_base_test");
    solver_.configure(node_, "test");
    dir_ = std::filesystem::temp_directory_path() /
      ("plan_solver_base_test_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir_);
    plan_path_ = (dir_ / "plan").string();
  }

  void TearDown() override
  {
    std::filesystem::remove_all(dir_);
  }

  std::string read_plan() const
  {
    std::ifstream ifs(plan_path_);
    std::stringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
  }

  // Runs execute_planner and returns its result and how long it took
  std::pair<bool, std::chrono::duration<double>> run(
    const std::string & command, const rclcpp::Duration & timeout)
  {
    auto start = std::chrono::steady_clock::now();
    bool ret = solver_.execute_planner(command, timeout, plan_path_);
    return {ret, std::chrono::steady_clock::now() - start};
  }

  // A zombie counts as dead: in containers whose PID 1 does not reap orphans, a
  // killed grandchild stays as a zombie forever
  static bool process_running(pid_t pid)
  {
    std::ifstream stat_ifs("/proc/" + std::to_string(pid) + "/stat");
    std::string stat;
    if (!std::getline(stat_ifs, stat)) {
      return false;
    }
    // Format: "pid (comm) state ...", and comm may contain spaces or parentheses
    auto close_paren = stat.rfind(')');
    if (close_paren == std::string::npos || close_paren + 2 >= stat.size()) {
      return false;
    }
    const char state = stat[close_paren + 2];
    return state != 'Z' && state != 'X';
  }

  static bool process_alive(pid_t pid)
  {
    // A killed grandchild may need a moment to die or be reaped by its new parent
    for (int i = 0; i < 50; i++) {
      if (!process_running(pid)) {
        return false;
      }
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  TestSolver solver_;
  std::filesystem::path dir_;
  std::string plan_path_;
};

TEST_F(PlanSolverBaseTest, output_is_written_to_plan_file)
{
  auto [ok, elapsed] = run("echo hello planner", 5s);
  ASSERT_TRUE(ok);
  ASSERT_EQ(read_plan(), "hello planner\n");
  ASSERT_LT(elapsed, 2s);
}

TEST_F(PlanSolverBaseTest, quoted_arguments_are_kept_together)
{
  ASSERT_TRUE(run("sh -c \"echo one; echo two\"", 5s).first);
  ASSERT_EQ(read_plan(), "one\ntwo\n");
}

TEST_F(PlanSolverBaseTest, output_bigger_than_pipe_buffer_is_complete)
{
  ASSERT_TRUE(run("seq 1 200000", 10s).first);
  const auto plan = read_plan();
  std::stringstream expected;
  for (int i = 1; i <= 200000; i++) {
    expected << i << "\n";
  }
  ASSERT_EQ(plan.size(), expected.str().size());
  ASSERT_EQ(plan, expected.str());
}

TEST_F(PlanSolverBaseTest, previous_plan_file_is_truncated)
{
  ASSERT_TRUE(run("seq 1 1000", 5s).first);
  ASSERT_TRUE(run("echo short", 5s).first);
  ASSERT_EQ(read_plan(), "short\n");
}

TEST_F(PlanSolverBaseTest, non_zero_exit_fails)
{
  ASSERT_FALSE(run("false", 5s).first);
  ASSERT_FALSE(run("sh -c \"echo partial; exit 3\"", 5s).first);
  ASSERT_EQ(read_plan(), "partial\n");
}

TEST_F(PlanSolverBaseTest, unknown_or_empty_command_fails_without_crashing)
{
  auto [ok, elapsed] = run("/nonexistent/planner arg", 5s);
  ASSERT_FALSE(ok);
  ASSERT_LT(elapsed, 2s);
  ASSERT_FALSE(run("", 5s).first);
  ASSERT_FALSE(run("   ", 5s).first);
}

TEST_F(PlanSolverBaseTest, killed_by_signal_fails)
{
  ASSERT_FALSE(run("sh -c \"kill -TERM $$\"", 5s).first);
  ASSERT_FALSE(run("sh -c \"kill -SEGV $$\"", 5s).first);
}

TEST_F(PlanSolverBaseTest, unwritable_plan_path_fails)
{
  plan_path_ = (dir_ / "missing_dir" / "plan").string();
  ASSERT_FALSE(run("echo hello", 5s).first);
}

TEST_F(PlanSolverBaseTest, timeout_is_enforced)
{
  auto [ok, elapsed] = run("sleep 30", 1s);
  ASSERT_FALSE(ok);
  ASSERT_GE(elapsed, 1s);
  ASSERT_LT(elapsed, 3s);
}

TEST_F(PlanSolverBaseTest, zero_timeout_fails_at_once)
{
  auto [ok, elapsed] = run("sleep 30", rclcpp::Duration(0, 0));
  ASSERT_FALSE(ok);
  ASSERT_LT(elapsed, 2s);
}

TEST_F(PlanSolverBaseTest, timeout_kills_grandchildren)
{
  // Like `ros2 run`: the planner is a grandchild that keeps the output pipe open
  auto [ok, elapsed] = run("sh -c \"sleep 60 & echo $!; wait\"", 1s);
  ASSERT_FALSE(ok);
  ASSERT_LT(elapsed, 3s);

  const pid_t grandchild = std::stoi(read_plan());
  ASSERT_GT(grandchild, 0);
  ASSERT_FALSE(process_alive(grandchild));
}

TEST_F(PlanSolverBaseTest, timeout_kills_grandchild_under_python_parent)
{
  // Same process layout as `ros2 run popf popf ...`
  auto [ok, elapsed] = run(
    "python3 -c \"import subprocess, sys; p = subprocess.Popen(['sleep', '61']); "
    "print(p.pid, flush=True); p.wait()\"", 2s);
  ASSERT_FALSE(ok);
  ASSERT_LT(elapsed, 4s);

  const pid_t grandchild = std::stoi(read_plan());
  ASSERT_FALSE(process_alive(grandchild));
}

TEST_F(PlanSolverBaseTest, finished_planner_with_background_child_succeeds)
{
  // The planner exits fine, but leaves a process holding the pipe
  auto [ok, elapsed] = run("sh -c \"sleep 62 & echo $!\"", 10s);
  ASSERT_TRUE(ok);
  ASSERT_LT(elapsed, 4s);

  const pid_t background = std::stoi(read_plan());
  ASSERT_FALSE(process_alive(background));
}

TEST_F(PlanSolverBaseTest, cancel_stops_the_planner)
{
  auto future = std::async(
    std::launch::async, [this]() {
      return run("sh -c \"sleep 63 & echo $!; wait\"", 30s);
    });

  std::this_thread::sleep_for(500ms);
  solver_.cancel();

  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  auto [ok, elapsed] = future.get();
  ASSERT_FALSE(ok);
  ASSERT_LT(elapsed, 3s);
  ASSERT_FALSE(process_alive(std::stoi(read_plan())));
}

TEST_F(PlanSolverBaseTest, runs_after_timeout_or_cancel_are_not_affected)
{
  ASSERT_FALSE(run("sleep 30", 500ms).first);
  ASSERT_TRUE(run("echo after timeout", 5s).first);
  ASSERT_EQ(read_plan(), "after timeout\n");

  auto future = std::async(std::launch::async, [this]() {return run("sleep 30", 30s);});
  std::this_thread::sleep_for(300ms);
  solver_.cancel();
  ASSERT_FALSE(future.get().first);

  ASSERT_TRUE(run("echo after cancel", 5s).first);
  ASSERT_EQ(read_plan(), "after cancel\n");
}

TEST_F(PlanSolverBaseTest, many_sequential_runs_leave_no_processes)
{
  for (int i = 0; i < 20; i++) {
    ASSERT_TRUE(run("echo " + std::to_string(i), 5s).first);
    ASSERT_EQ(read_plan(), std::to_string(i) + "\n");
  }
  for (int i = 0; i < 5; i++) {
    auto [ok, elapsed] = run("sh -c \"sleep 64 & echo $!; wait\"", 200ms);
    ASSERT_FALSE(ok);
    ASSERT_FALSE(process_alive(std::stoi(read_plan())));
  }
}

TEST_F(PlanSolverBaseTest, parallel_solvers_are_independent)
{
  auto node2 = rclcpp_lifecycle::LifecycleNode::make_shared("plan_solver_base_test_2");
  TestSolver solver2;
  solver2.configure(node2, "test2");
  const std::string plan_path2 = (dir_ / "plan2").string();

  // One times out while the other finishes normally
  auto slow = std::async(
    std::launch::async, [this]() {
      return solver_.execute_planner("sleep 30", 1s, plan_path_);
    });
  ASSERT_TRUE(solver2.execute_planner("echo fast", 5s, plan_path2));
  ASSERT_FALSE(slow.get());

  std::ifstream ifs(plan_path2);
  std::string line;
  std::getline(ifs, line);
  ASSERT_EQ(line, "fast");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
