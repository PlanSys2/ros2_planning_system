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

// A plan the executor cannot set up must be rejected, never kill the process (#431).
// Each scenario runs in its own process (see CMakeLists.txt): the executor node is
// never destroyed (#430), so it must not leak into the next scenario.

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using Transition = lifecycle_msgs::msg::Transition;

namespace
{

// Finishes the action successfully after a few ticks
class QuickAction : public plansys2::ActionExecutorClient
{
public:
  explicit QuickAction(const std::string & name)
  : ActionExecutorClient(name) {}

private:
  void do_work() override
  {
    if (++ticks_ >= 3) {
      ticks_ = 0;
      finish(true, 1.0, "done");
    } else {
      send_feedback(ticks_ / 3.0, "working");
    }
  }

  int ticks_ {0};
};

class ScopedSpinner
{
public:
  explicit ScopedSpinner(rclcpp::Executor & exe)
  : thread_([this, &exe]() {while (!finish_) {exe.spin_once(10ms);}}) {}
  ~ScopedSpinner()
  {
    finish_ = true;
    thread_.join();
  }

private:
  std::atomic<bool> finish_ {false};
  std::thread thread_;
};

plansys2_msgs::msg::Plan make_plan(const std::vector<std::string> & actions)
{
  plansys2_msgs::msg::Plan plan;
  float time = 0.0;
  for (const auto & action : actions) {
    plansys2_msgs::msg::PlanItem item;
    item.time = time;
    item.action = action;
    item.duration = 1.0;
    plan.items.push_back(item);
    time += 1.001;
  }
  return plan;
}

}  // namespace

class PlanSetupErrorsTest : public ::testing::Test
{
protected:
  // Starts domain/problem experts, the executor (with extra parameters) and a performer
  void start(const std::vector<rclcpp::Parameter> & executor_params = {})
  {
    std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_executor").string();

    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    executor_node_ = std::make_shared<plansys2::ExecutorNode>();
    move_node_ = std::make_shared<QuickAction>("move_performer");
    move_node_->set_parameter({"action_name", "move"});
    move_node_->set_parameter({"rate", 20.0});

    domain_node_->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
    for (const auto & param : executor_params) {
      executor_node_->set_parameter(param);
    }

    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(domain_node_->get_node_base_interface());
    exe_->add_node(problem_node_->get_node_base_interface());
    exe_->add_node(executor_node_->get_node_base_interface());
    exe_->add_node(move_node_->get_node_base_interface());
    spinner_ = std::make_unique<ScopedSpinner>(*exe_);

    for (auto node : std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>{
      domain_node_, problem_node_, executor_node_, move_node_})
    {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    }
    for (auto node : std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>{
      domain_node_, problem_node_, executor_node_})
    {
      node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }

    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("r2d2", "robot")));
    for (auto zone : {"wheels_zone", "steering_wheels_zone", "assembly_zone"}) {
      ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance(zone, "zone")));
    }
    for (auto pred : {"(robot_at r2d2 wheels_zone)", "(robot_available r2d2)",
        "(battery_full r2d2)"})
    {
      ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate(pred)));
    }
  }

  void TearDown() override
  {
    spinner_.reset();
    exe_.reset();
  }

  // Runs a plan to completion and returns its result, if any
  std::optional<ExecutePlan::Result> run(const plansys2_msgs::msg::Plan & plan)
  {
    if (!executor_client_->start_plan_execution(plan)) {
      return {};
    }
    auto start = std::chrono::steady_clock::now();
    while (executor_client_->execute_and_check_plan()) {
      if (std::chrono::steady_clock::now() - start > 60s) {
        return {};
      }
      std::this_thread::sleep_for(50ms);
    }
    return executor_client_->getResult();
  }

  // After a broken plan, the executor still runs a good one
  void expect_executor_still_works()
  {
    auto result = run(make_plan({"(move r2d2 wheels_zone assembly_zone)"}));
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
    ASSERT_TRUE(
      problem_client_->existPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)")));
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  std::shared_ptr<plansys2::ExecutorNode> executor_node_;
  std::shared_ptr<QuickAction> move_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::unique_ptr<ScopedSpinner> spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

TEST_F(PlanSetupErrorsTest, valid_plan)
{
  start();
  expect_executor_still_works();
}

TEST_F(PlanSetupErrorsTest, empty_plan)
{
  start();
  // Nothing to do: succeeds at once
  auto result = run(make_plan({}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
  expect_executor_still_works();
}

TEST_F(PlanSetupErrorsTest, unknown_action)
{
  start();
  auto result = run(make_plan({"(fly r2d2 wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
  expect_executor_still_works();
}

TEST_F(PlanSetupErrorsTest, unknown_action_among_valid_ones)
{
  start();
  auto result = run(
    make_plan(
      {"(move r2d2 wheels_zone steering_wheels_zone)", "(fly r2d2 wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
  // Nothing was executed
  ASSERT_TRUE(
    problem_client_->existPredicate(plansys2::Predicate("(robot_at r2d2 wheels_zone)")));
  expect_executor_still_works();
}

TEST_F(PlanSetupErrorsTest, malformed_action)
{
  start();
  for (const std::string action : {"(move r2d2)", "move r2d2 a b", "()", ""}) {
    SCOPED_TRACE(action);
    auto result = run(make_plan({action}));
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
  }
  expect_executor_still_works();
}

TEST_F(PlanSetupErrorsTest, unknown_bt_builder_plugin)
{
  start({rclcpp::Parameter("bt_builder_plugin", "DoesNotExistBTBuilder")});
  auto result = run(make_plan({"(move r2d2 wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
  // The executor answers, and a later plan with the same setup fails the same way
  result = run(make_plan({"(move r2d2 wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
}

TEST_F(PlanSetupErrorsTest, malformed_action_bt_xml)
{
  const auto xml = std::filesystem::temp_directory_path() /
    ("plan_setup_errors_" + std::to_string(getpid()) + ".xml");
  std::ofstream(xml) << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"x\"><NotANode/>";
  start({rclcpp::Parameter("default_action_bt_xml_filename", xml.string())});

  auto result = run(make_plan({"(move r2d2 wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
  result = run(make_plan({"(move r2d2 wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);
  std::filesystem::remove(xml);
}

TEST_F(PlanSetupErrorsTest, sequential_bt_builder)
{
  start({rclcpp::Parameter("bt_builder_plugin", "SequentialBTBuilder")});
  auto result = run(
    make_plan(
      {"(move r2d2 wheels_zone steering_wheels_zone)",
        "(move r2d2 steering_wheels_zone assembly_zone)"}));
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
  ASSERT_TRUE(
    problem_client_->existPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)")));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
