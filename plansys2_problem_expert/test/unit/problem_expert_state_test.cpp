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

// The state of the problem kept in state_file: a node that dies and starts again comes back
// with the problem it had; one cleaned up or shut down on purpose starts from nothing

#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace
{

std::string pddl(const std::string & name)
{
  return (plansys2::share_path("plansys2_problem_expert") / "pddl" /
         name).string();
}

std::string read(const std::filesystem::path & path)
{
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void write(const std::filesystem::path & path, const std::string & text)
{
  std::ofstream(path) << text;
}

}  // namespace

class ProblemExpertStateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() /
      ("plansys2_state_test_" + std::to_string(::getpid()) + "_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    state_ = dir_ / "problem.pddl";
    client_node_ = rclcpp::Node::make_shared("state_test_client");
    client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
  }

  void TearDown() override
  {
    crash();
    client_.reset();
    std::filesystem::remove_all(dir_);
  }

  // A problem expert started as the bringup starts it: configured and active
  void start(const std::string & state_file, const std::string & problem_file = "")
  {
    node_ = std::make_shared<plansys2::ProblemExpertNode>();
    node_->set_parameter({"model_file", pddl("domain_simple.pddl")});
    node_->set_parameter({"state_file", state_file});
    node_->set_parameter({"problem_file", problem_file});
    exe_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    exe_->add_node(node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(*exe_);
    configured_ = node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id() ==
      State::PRIMARY_STATE_INACTIVE;
    if (configured_) {
      node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }
  }

  // The process dies: no cleanup, no shutdown
  void crash()
  {
    spinner_.reset();
    exe_.reset();
    node_.reset();
  }

  void fill()
  {
    for (const auto & [name, type] : std::vector<std::pair<std::string, std::string>>{
      {"r2d2", "robot"}, {"paco", "person"}, {"kitchen", "room"}, {"bedroom", "room"}})
    {
      ASSERT_TRUE(client_->addInstance(plansys2::Instance(name, type)));
    }
    ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));
    ASSERT_TRUE(
      client_->addFunction(plansys2::Function("(= (room_distance kitchen bedroom) 7.5)")));
    ASSERT_TRUE(client_->setGoal(plansys2::Goal("(and (robot_at r2d2 bedroom))")));
  }

  // What it holds, as a client sees it
  void expect_filled(size_t instances = 4)
  {
    EXPECT_EQ(client_->getInstances().size(), instances);
    EXPECT_TRUE(client_->existPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));
    auto distance = client_->getFunction("(room_distance kitchen bedroom)");
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(distance->value, 7.5, 1e-9);
    EXPECT_EQ(parser::pddl::toString(client_->getGoal()), "(and (robot_at r2d2 bedroom))");
  }

  bool saved_with(const std::string & text, std::chrono::seconds timeout = 3s)
  {
    return plansys2::testing::wait_for(
      [&]() {
        return std::filesystem::exists(state_) && read(state_).find(text) != std::string::npos;
      }, timeout);
  }

  std::filesystem::path dir_;
  std::filesystem::path state_;
  std::shared_ptr<plansys2::ProblemExpertNode> node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  bool configured_ {false};
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<plansys2::ProblemExpertClient> client_;
};

TEST_F(ProblemExpertStateTest, without_state_file_nothing_is_kept)
{
  start("");
  fill();
  std::this_thread::sleep_for(500ms);
  EXPECT_TRUE(std::filesystem::is_empty(dir_));
  crash();
  start("");
  EXPECT_TRUE(client_->getInstances().empty());
}

TEST_F(ProblemExpertStateTest, a_restarted_node_comes_back_with_its_problem)
{
  start(state_.string());
  fill();
  ASSERT_TRUE(saved_with("robot_at r2d2 bedroom")) << read(state_);  // The goal, the last
  crash();
  ASSERT_TRUE(std::filesystem::exists(state_));
  start(state_.string());
  ASSERT_TRUE(configured_);
  expect_filled();

  // And it keeps saving what changes after
  ASSERT_TRUE(client_->removePredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));
  ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r2d2 bedroom)")));
  ASSERT_TRUE(plansys2::testing::wait_for(
      [&]() {return read(state_).find("robot_at r2d2 kitchen") == std::string::npos;}, 3s));
  crash();
  start(state_.string());
  EXPECT_TRUE(client_->existPredicate(plansys2::Predicate("(robot_at r2d2 bedroom)")));
  EXPECT_FALSE(client_->existPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));
}

TEST_F(ProblemExpertStateTest, crashes_in_a_row)
{
  start(state_.string());
  fill();
  for (int i = 0; i < 4; i++) {
    SCOPED_TRACE(i);
    const auto room = "room_" + std::to_string(i);
    ASSERT_TRUE(client_->addInstance(plansys2::Instance(room, "room")));
    ASSERT_TRUE(saved_with(room));
    crash();
    start(state_.string());
    ASSERT_TRUE(configured_);
    std::string names;
    for (const auto & instance : client_->getInstances()) {
      names += instance.name + " ";
    }
    EXPECT_EQ(client_->getInstances().size(), 5u + i) << names;
  }
  expect_filled(8);
}

TEST_F(ProblemExpertStateTest, a_burst_of_changes_is_kept_whole)
{
  start(state_.string());
  fill();
  for (int i = 0; i < 200; i++) {
    ASSERT_TRUE(client_->addInstance(plansys2::Instance("burst_" + std::to_string(i), "room")));
  }
  ASSERT_TRUE(saved_with("burst_199"));
  crash();
  start(state_.string());
  EXPECT_EQ(client_->getInstances().size(), 204u);
  // Written aside and renamed: nothing left beside it
  EXPECT_FALSE(std::filesystem::exists(state_.string() + ".tmp"));
}

TEST_F(ProblemExpertStateTest, a_cleared_problem_is_kept_cleared)
{
  start(state_.string());
  fill();
  ASSERT_TRUE(saved_with("robot_at r2d2 bedroom"));
  ASSERT_TRUE(client_->clearKnowledge());
  ASSERT_TRUE(plansys2::testing::wait_for(
      [&]() {return read(state_).find("r2d2") == std::string::npos;}, 3s));
  crash();
  start(state_.string());
  EXPECT_TRUE(client_->getInstances().empty());
}

TEST_F(ProblemExpertStateTest, cleanup_and_shutdown_forget_it)
{
  start(state_.string());
  fill();
  ASSERT_TRUE(saved_with("robot_at r2d2 bedroom"));
  node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  EXPECT_FALSE(std::filesystem::exists(state_));
  // Configured again in the same process: from nothing, as before
  node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  EXPECT_TRUE(client_->getInstances().empty());

  fill();
  ASSERT_TRUE(saved_with("robot_at r2d2 bedroom"));
  node_->shutdown();
  EXPECT_FALSE(std::filesystem::exists(state_));
  crash();
  start(state_.string());
  EXPECT_TRUE(client_->getInstances().empty());
}

TEST_F(ProblemExpertStateTest, the_state_wins_over_problem_file_only_when_there_is_one)
{
  const auto problem_file = dir_ / "initial.pddl";
  write(
    problem_file,
    "(define (problem p) (:domain simple) (:objects initial_room - room) (:init) "
    "(:goal (and)))");
  // A first start: no state yet, the problem_file
  start(state_.string(), problem_file.string());
  ASSERT_TRUE(configured_);
  EXPECT_TRUE(client_->getInstance("initial_room").has_value());
  ASSERT_TRUE(client_->addInstance(plansys2::Instance("later_room", "room")));
  ASSERT_TRUE(saved_with("later_room"));
  crash();
  // Restarted: what it had, not the problem_file again
  start(state_.string(), problem_file.string());
  EXPECT_TRUE(client_->getInstance("later_room").has_value());
  EXPECT_TRUE(client_->getInstance("initial_room").has_value());
  EXPECT_EQ(client_->getInstances().size(), 2u);
}

TEST_F(ProblemExpertStateTest, an_unreadable_state_starts_without_it)
{
  const auto problem_file = dir_ / "initial.pddl";
  write(
    problem_file,
    "(define (problem p) (:domain simple) (:objects initial_room - room) (:init) "
    "(:goal (and)))");
  for (const char * bad : {"", "(define (problem", "garbage ((( ,", "(define (problem p) "
      "(:domain simple) (:objects x - unknown_type) (:init) (:goal (and)))"})
  {
    SCOPED_TRACE(bad);
    write(state_, bad);
    start(state_.string(), problem_file.string());
    ASSERT_TRUE(configured_);
    // The problem_file instead, and nothing of the bad state
    EXPECT_TRUE(client_->getInstance("initial_room").has_value());
    EXPECT_EQ(client_->getInstances().size(), 1u);
    crash();
  }
  // Without problem_file: empty
  write(state_, "garbage ((( ,");
  start(state_.string());
  ASSERT_TRUE(configured_);
  EXPECT_TRUE(client_->getInstances().empty());
}

TEST_F(ProblemExpertStateTest, an_unwritable_place_does_not_stop_the_node)
{
  start("/proc/plansys2_cannot_be_here/problem.pddl");
  ASSERT_TRUE(configured_);
  fill();
  std::this_thread::sleep_for(500ms);
  expect_filled();
}

TEST_F(ProblemExpertStateTest, home_in_the_path)
{
  const char * home = std::getenv("HOME");
  ASSERT_NE(home, nullptr);
  const auto relative = std::filesystem::relative(dir_, home);
  if (relative.empty() || relative.string().rfind("..", 0) == 0) {
    // The temporary folder is not under HOME: a folder of its own there
    const auto under_home = std::filesystem::path(home) / ".plansys2_state_test_tmp";
    std::filesystem::create_directories(under_home);
    start("~/.plansys2_state_test_tmp/problem.pddl");
    fill();
    EXPECT_TRUE(plansys2::testing::wait_for(
        [&]() {return std::filesystem::exists(under_home / "problem.pddl");}, 3s));
    crash();
    std::filesystem::remove_all(under_home);
  } else {
    start("~/" + (relative / "problem.pddl").string());
    fill();
    EXPECT_TRUE(saved_with("robot_at r2d2 bedroom"));
  }
}

TEST_F(ProblemExpertStateTest, a_stale_temporary_file_is_not_taken)
{
  write(state_.string() + ".tmp", "(define (problem p) (:domain simple) (:objects ghost - room) "
    "(:init) (:goal (and)))");
  start(state_.string());
  ASSERT_TRUE(configured_);
  EXPECT_FALSE(client_->getInstance("ghost").has_value());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
