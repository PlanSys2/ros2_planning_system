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

// Plans with required concurrency (MatchCellar: a fuse can only be mended while a match is
// lit) run with STNBTBuilder, which executes mend_fuse during light_match. SimpleBTBuilder,
// that only waits for actions to end, cannot. Each scenario runs in its own process.

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
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
using Clock = std::chrono::steady_clock;

namespace
{

// Takes `duration` and records when it started and finished
class TimedPerformer : public plansys2::ActionExecutorClient
{
public:
  TimedPerformer(const std::string & name, const std::string & action, double duration)
  : ActionExecutorClient(name), duration_(duration)
  {
    set_parameter({"action_name", action});
    set_parameter({"rate", 20.0});
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    started_at = Clock::now();
    return ActionExecutorClient::on_activate(state);
  }

  std::optional<Clock::time_point> started_at;
  std::optional<Clock::time_point> finished_at;
  std::mutex mutex_;

private:
  void do_work() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (Clock::now() - started_at.value() >= std::chrono::duration<double>(duration_)) {
      finished_at = Clock::now();
      finish(true, 1.0, "done");
    } else {
      send_feedback(0.5, "running");
    }
  }

  double duration_;
};

plansys2_msgs::msg::PlanItem item(double time, const std::string & action, double duration)
{
  plansys2_msgs::msg::PlanItem plan_item;
  plan_item.time = time;
  plan_item.action = action;
  plan_item.duration = duration;
  return plan_item;
}

}  // namespace

class ExecutorStnTest : public ::testing::Test
{
protected:
  void start(const std::string & bt_builder)
  {
    std::string pkgpath = plansys2::share_path("plansys2_executor").string();
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    executor_node_ = std::make_shared<plansys2::ExecutorNode>();
    domain_node_->set_parameter({"model_file", pkgpath + "/pddl/matchcellar.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath + "/pddl/matchcellar.pddl"});
    executor_node_->set_parameter({"bt_builder_plugin", bt_builder});
    // The performers below take less time than planned
    light_ = std::make_shared<TimedPerformer>("light_performer", "light_match", 2.0);
    mend_ = std::make_shared<TimedPerformer>("mend_performer", "mend_fuse", 0.5);

    for (auto node : nodes()) {
      exe_.add_node(node->get_node_base_interface());
    }
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
    for (auto node : nodes()) {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    }
    for (auto node : {domain_node_->shared_from_this(), problem_node_->shared_from_this(),
        executor_node_->shared_from_this()})
    {
      std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node)->trigger_transition(
        Transition::TRANSITION_ACTIVATE);
    }

    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(client_node_);
    ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("m1", "match")));
    ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("f1", "fuse")));
    ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate("(handfree)")));
    ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate("(unused m1)")));
  }

  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes()
  {
    return {domain_node_, problem_node_, executor_node_, light_, mend_};
  }

  void TearDown() override
  {
    executor_client_.reset();
    problem_client_.reset();
    spinner_.reset();
  }

  std::optional<ExecutePlan::Result> run(const plansys2_msgs::msg::Plan & plan)
  {
    if (!executor_client_->start_plan_execution(plan)) {
      return {};
    }
    auto deadline = Clock::now() + 30s;
    while (executor_client_->execute_and_check_plan() && Clock::now() < deadline) {
      std::this_thread::sleep_for(20ms);
    }
    return executor_client_->getResult();
  }

  // As POPF solves it: mend the fuse while the match burns
  plansys2_msgs::msg::Plan matchcellar_plan()
  {
    plansys2_msgs::msg::Plan plan;
    plan.items = {item(0.0, "(light_match m1)", 5.0), item(0.001, "(mend_fuse f1 m1)", 2.0)};
    return plan;
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  std::shared_ptr<plansys2::ExecutorNode> executor_node_;
  std::shared_ptr<TimedPerformer> light_;
  std::shared_ptr<TimedPerformer> mend_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

TEST_F(ExecutorStnTest, stn_executes_an_action_during_another)
{
  start("STNBTBuilder");
  auto result = run(matchcellar_plan());
  ASSERT_TRUE(result);
  ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
  ASSERT_TRUE(problem_client_->existPredicate(plansys2::Predicate("(mended f1)")));

  // mend_fuse ran inside light_match
  ASSERT_TRUE(light_->started_at && light_->finished_at);
  ASSERT_TRUE(mend_->started_at && mend_->finished_at);
  EXPECT_GE(mend_->started_at.value(), light_->started_at.value());
  EXPECT_LE(mend_->finished_at.value(), light_->finished_at.value());
}

TEST_F(ExecutorStnTest, simple_builder_cannot_run_required_concurrency)
{
  start("SimpleBTBuilder");
  auto result = run(matchcellar_plan());
  ASSERT_TRUE(result);
  EXPECT_NE(result->result, ExecutePlan::Result::SUCCESS);
  EXPECT_FALSE(problem_client_->existPredicate(plansys2::Predicate("(mended f1)")));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
