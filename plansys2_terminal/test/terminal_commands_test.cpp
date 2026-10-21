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

// Malformed commands and plan files, errors, and Ctrl-C while a plan runs

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_terminal/Terminal.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;

namespace
{
class TestTerminal : public plansys2_terminal::Terminal
{
public:
  using Terminal::Terminal;
  using Terminal::init;
  using Terminal::clean_command;

  std::string run(std::string command)
  {
    std::ostringstream os;
    process_command(command, os);
    return os.str();
  }

  bool throw_in_check = false;

protected:
  void process_check(std::vector<std::string> & command, std::ostringstream & os) override
  {
    if (throw_in_check) {
      throw std::runtime_error("boom");
    }
    Terminal::process_check(command, os);
  }
};

std::string write_file(const std::string & name, const std::string & content)
{
  const std::string path = testing::TempDir() + name;
  std::ofstream(path) << content;
  return path;
}

// A move that never ends: only a cancel stops it
class EndlessMove : public plansys2::ActionExecutorClient
{
public:
  EndlessMove()
  : plansys2::ActionExecutorClient("endless_move")
  {
    set_parameter({"action_name", "move"});
    set_parameter({"rate", 10.0});
  }

  std::atomic<int> works{0};

private:
  void do_work() override
  {
    works++;
    send_feedback(0.5, "moving");
  }
};
}  // namespace

TEST(terminal_commands, tokenize_on_blanks)
{
  using plansys2_terminal::tokenize;
  EXPECT_EQ(tokenize("set\tinstance  r1\r"), (std::vector<std::string>{"set", "instance", "r1"}));
  EXPECT_EQ(tokenize("  get   plan "), (std::vector<std::string>{"get", "plan"}));
  EXPECT_EQ(tokenize("get plan\nquit"), (std::vector<std::string>{"get", "plan"}));
  EXPECT_TRUE(tokenize("").empty());
  EXPECT_TRUE(tokenize(" \t ").empty());
}

TEST(terminal_commands, plan_files)
{
  using plansys2_terminal::parse_plan;

  // As the planners write them, with comments, empty lines and Windows line ends
  auto plan = parse_plan(
    write_file(
      "plan_ok",
      "; Plan found\n\n 0.000:   (askcharge leia entrance chargingroom)  [5.000]\r\n"
      "5.001: (charge   leia chargingroom) [5]\n  ; Cost: 2\n6: (move leia a b)\n"));
  ASSERT_TRUE(plan.has_value());
  ASSERT_EQ(plan->items.size(), 3u);
  EXPECT_EQ(plan->items[0].action, "(askcharge leia entrance chargingroom)");
  EXPECT_FLOAT_EQ(plan->items[0].time, 0.0);
  EXPECT_FLOAT_EQ(plan->items[0].duration, 5.0);
  EXPECT_EQ(plan->items[1].action, "(charge leia chargingroom)");
  EXPECT_FLOAT_EQ(plan->items[1].time, 5.001);
  EXPECT_FLOAT_EQ(plan->items[2].duration, 0.0);  // No duration

  // Each of these used to abort the terminal, or to make up a duration
  for (const char * line : {
    "0.000 (move leia a b) [5]", "x: (move leia a b) [5]", "0: (move leia a b) [five]",
    "0: (move leia a b) 5", "0: (move leia a b) [5] more", "0: move leia a b [5]",
    "0: (move leia a b [5]", "(move leia a b): 0 [5]", "0: (move leia a b) [5"})
  {
    EXPECT_FALSE(parse_plan(write_file("plan_bad", line + std::string("\n"))).has_value()) <<
      line;
  }
  EXPECT_FALSE(parse_plan(write_file("plan_empty", "")).has_value());
  EXPECT_FALSE(parse_plan(write_file("plan_comments", "; nothing\n\n")).has_value());
  EXPECT_FALSE(parse_plan(testing::TempDir() + "does_not_exist").has_value());
}

// Malformed commands answer with their usage before calling any server
TEST(terminal_commands, malformed_commands)
{
  auto terminal = std::make_shared<TestTerminal>();
  plansys2::testing::NodeSpinner terminal_spinner(terminal);
  terminal->init();

  auto usage = [&](const std::string & command, const std::string & expected) {
      const auto out = terminal->run(command);
      EXPECT_NE(out.find(expected), std::string::npos) << command << " -> " << out;
    };

  for (const auto & command : {
    "set predicate (", "set predicate (robot_at r1", "set predicate robot_at r1)",
    "set predicate (robot_at r1) (robot_at r2)", "set predicate ((robot_at r1))",
    "set predicate (2robot r1)", "set predicate ()", "set predicate"})
  {
    usage(command, "set predicate (predicate)");
  }
  for (const auto & command : {
    "remove predicate (robot_at", "remove predicate robot_at r1)", "remove predicate (",
    "remove predicate", "remove predicate (robot_at r1))"})
  {
    usage(command, "remove predicate (predicate)");
  }
  for (const auto & command : {
    "run num_actions", "run num_actions abc", "run num_actions 99999999999",
    "run num_actions -1", "run num_actions 0", "run num_actions 3x", "run num_actions 1 2"})
  {
    usage(command, "run num_actions [number of actions to execute from plan]");
  }
  usage("run plan-file", "run plan-file [planfile]");
  usage("run plan-file a b", "run plan-file [planfile]");
  usage("run plan-file " + write_file("plan_cmd_bad", "0: (move\n"), "Plan could not be loaded");
  usage("run action", "run action [action to execute]");
  usage("run foo", "run num_actions");
  usage("check", "check actors");
  usage("check foo", "check actors");
  usage("get problem foo", "get problem [instances|predicates|functions|goal|metric]");
  usage("get model derived", "get model derived [predicate_name]");
  usage("remove function", "\t\tremove function [name]");
  usage("nothing", "Command not found");

  // Help of each command, not of another one
  const auto check_help = terminal->run("help check");
  EXPECT_NE(check_help.find("check [actors]"), std::string::npos) << check_help;
  EXPECT_EQ(check_help.find("run action"), std::string::npos) << check_help;
  EXPECT_NE(terminal->run("help get").find("derived <predicate_name>"), std::string::npos);
  EXPECT_EQ(terminal->run("help").find('\r'), std::string::npos);
  EXPECT_EQ(terminal->run("remove function").find('\r'), std::string::npos);
}

TEST(terminal_commands, errors_do_not_end_the_terminal)
{
  auto terminal = std::make_shared<TestTerminal>();
  plansys2::testing::NodeSpinner terminal_spinner(terminal);
  terminal->init();
  terminal->throw_in_check = true;
  EXPECT_EQ(terminal->run("check actors"), "Error: boom\n");
  terminal->throw_in_check = false;
  EXPECT_NE(terminal->run("help check").find("check [actors]"), std::string::npos);
}

TEST(terminal_commands, clean_command)
{
  auto terminal = std::make_shared<TestTerminal>();
  plansys2::testing::NodeSpinner terminal_spinner(terminal);
  std::string command = "\tset\tinstance   r1 robot \r";
  terminal->clean_command(command);
  EXPECT_EQ(command, "set instance r1 robot");
  command = "  quit ";
  terminal->clean_command(command);
  EXPECT_EQ(command, "quit");
}

// Ctrl-C while a plan runs cancels it in the executor, and the terminal goes on
TEST(terminal_commands, interrupt_cancels_the_plan)
{
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto planner_node = std::make_shared<plansys2::PlannerNode>();
    auto executor_node = std::make_shared<plansys2::ExecutorNode>();
    auto mover = std::make_shared<EndlessMove>();

    const std::string pkgpath =
      plansys2::share_path("plansys2_terminal").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/simple_example.pddl"});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/simple_example.pddl"});

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());
    exe.add_node(planner_node->get_node_base_interface());
    exe.add_node(executor_node->get_node_base_interface());
    exe.add_node(mover->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
      domain_node->trigger_transition(transition);
      problem_node->trigger_transition(transition);
      planner_node->trigger_transition(transition);
      executor_node->trigger_transition(transition);
    }
    mover->trigger_transition(Transition::TRANSITION_CONFIGURE);

    auto terminal = std::make_shared<TestTerminal>();
    plansys2::testing::NodeSpinner terminal_spinner(terminal);
    terminal->init();
    for (const auto & command : {
      "set instance leia robot", "set instance entrance room", "set instance kitchen room",
      "set predicate (robot_at leia entrance)", "set predicate (connected entrance kitchen)",
      "set predicate (battery_full leia)"})
    {
      ASSERT_EQ(terminal->run(command), "") << command;
    }

    std::thread ctrl_c([&]() {
        plansys2::testing::wait_for([&]() {return mover->works >= 3;}, 20s);
        plansys2_terminal::interrupt();
      });
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(terminal->run("run action (move leia entrance kitchen)"), "");
    ctrl_c.join();
    EXPECT_LT(std::chrono::steady_clock::now() - start, 25s);
    EXPECT_GE(mover->works, 3);

    // The performer is told to stop: it does no more work
    plansys2::testing::wait_for(
      [&]() {
        return mover->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      }, 5s);
    EXPECT_NE(mover->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    // And the terminal still works
    EXPECT_NE(terminal->run("get problem instances").find("Instances: 3"), std::string::npos);
  }
  plansys2::drain_ros(500ms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
