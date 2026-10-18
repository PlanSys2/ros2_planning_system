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

// Every solvable task of the bank that PlanSys2 loads is planned and executed to the end, and
// its goal holds in the problem expert afterwards

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;

namespace
{

// Not executable yet: none
const std::set<std::string> kNotLoaded = {};

// Does any action at once: the executor applies its effects
class Performer : public plansys2::ActionExecutorClient
{
public:
  Performer(const std::string & node_name, const std::string & action)
  : ActionExecutorClient(node_name)
  {
    set_parameter({"action_name", action});
    set_parameter({"rate", 50.0});
  }

private:
  void do_work() override {finish(true, 1.0, "done");}
};

// Time ten times faster than the wall: what a plan lasts (PDDL+, timed literals) does not slow
// the tests. Every node reads it with use_sim_time; each isolated test has its ROS domain
class FastClock
{
public:
  explicit FastClock(double factor = 10.0)
  : node_(rclcpp::Node::make_shared("bank_clock")),
    publisher_(node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10))
  {
    publish();
    thread_ = std::thread(
      [this, factor]() {
        while (running_) {
          std::this_thread::sleep_for(5ms);
          seconds_ += 0.005 * factor;
          publish();
        }
      });
  }

  ~FastClock()
  {
    running_ = false;
    thread_.join();
  }

  double seconds() const {return seconds_;}

private:
  void publish()
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(static_cast<int64_t>(seconds_ * 1e9));
    publisher_->publish(clock);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  std::atomic<double> seconds_ {1000.0};
  std::atomic<bool> running_ {true};
  std::thread thread_;
};

// The nodes of PlanSys2 for one task, with a performer per action
struct Run
{
  FastClock clock;
  std::shared_ptr<plansys2::DomainExpertNode> domain_node;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node;
  std::shared_ptr<plansys2::PlannerNode> planner_node;
  std::shared_ptr<plansys2::ExecutorNode> executor_node;
  std::vector<std::shared_ptr<Performer>> performers;
  rclcpp::executors::SingleThreadedExecutor exe;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner;
};

// Runs the task to its end; meanwhile, what the world does while the plan runs
void execute(
  const parser::pddl::BankCase & task, const std::string & bt_builder,
  const std::function<void(plansys2::ProblemExpertClient &)> & meanwhile = {},
  uint8_t expected = plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
{
  Run run;
  run.domain_node = std::make_shared<plansys2::DomainExpertNode>();
  run.problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  run.planner_node = std::make_shared<plansys2::PlannerNode>();
  run.executor_node = std::make_shared<plansys2::ExecutorNode>();
  run.executor_node->set_parameter({"bt_builder_plugin", bt_builder});
  run.domain_node->set_parameter({"model_file", task.domain_file.string()});
  run.problem_node->set_parameter({"model_file", task.domain_file.string()});
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> experts = {
    run.domain_node, run.problem_node, run.planner_node, run.executor_node};
  for (auto & node : experts) {
    node->set_parameter({"use_sim_time", true});
    run.exe.add_node(node->get_node_base_interface());
  }
  run.spinner = std::make_unique<plansys2::testing::ExecutorThread>(run.exe);
  ASSERT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        for (auto & node : experts) {
          if (node->now().seconds() < 999.0) {
            return false;
          }
        }
        return true;
      }, 5s));
  for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
    for (auto & node : experts) {
      node->trigger_transition(transition);
    }
  }
  for (auto & node : experts) {
    ASSERT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      << node->get_name();
  }
  run.planner_node->set_parameter({"KAIROS.anytime", false});  // The first plan is enough

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto actions = domain_client->getActions();
  for (const auto & action : domain_client->getDurativeActions()) {
    actions.push_back(action);
  }
  for (const auto & action : actions) {
    auto performer = std::make_shared<Performer>("bank_" + action, action);
    performer->set_parameter({"use_sim_time", true});
    run.exe.add_node(performer->get_node_base_interface());
    performer->trigger_transition(Transition::TRANSITION_CONFIGURE);
    run.performers.push_back(performer);
  }

  ASSERT_TRUE(problem_client->addProblem(task.problem));
  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan.has_value());

  // What went wrong, if anything: the plan and how each action ended
  std::string report = "plan:\n";
  for (const auto & item : plan->items) {
    report += "  " + std::to_string(item.time) + ": " + item.action + " [" +
      std::to_string(item.duration) + "]\n";
  }
  auto describe = [&report](const std::vector<plansys2_msgs::msg::ActionExecutionInfo> & s) {
      for (const auto & info : s) {
        report += "  " + info.action_full_name + " status " + std::to_string(info.status) +
          " " + info.message_status + " start " +
          std::to_string(rclcpp::Time(info.start_stamp).seconds()) + " end " +
          std::to_string(rclcpp::Time(info.status_stamp).seconds()) + "\n";
      }
    };

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));
  if (meanwhile) {
    meanwhile(*problem_client);
  }
  const bool ended = plansys2::testing::wait_for(
    [&]() {return !executor_client->execute_and_check_plan();}, 20s);
  if (!ended) {
    describe(executor_client->getFeedBack().action_execution_status);
    report += "state:\n";
    for (const auto & predicate : problem_client->getPredicates()) {
      report += "  " + parser::pddl::toString(predicate) + "\n";
    }
    executor_client->cancel_plan_execution();
  }
  ASSERT_TRUE(ended) << report;
  auto result = executor_client->getResult();
  ASSERT_TRUE(result.has_value());
  describe(result->action_execution_status);
  EXPECT_EQ(result->result, expected) << report;
  if (expected == plansys2_msgs::action::ExecutePlan::Result::SUCCESS) {
    EXPECT_TRUE(problem_client->isGoalSatisfied(problem_client->getGoal())) << report;
  }

  run.spinner.reset();
}

}  // namespace

class ExecutorBankTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(ExecutorBankTest, every_loaded_task_runs_to_its_goal)
{
  const auto cases = parser::pddl::read_pddl_bank(
    ament_index_cpp::get_package_share_path("plansys2_pddl_parser") / "bank");
  ASSERT_GE(cases.size(), 60u);
  const auto & bt_builder = GetParam();

  int executed = 0;
  const auto start = std::chrono::steady_clock::now();
  for (const auto & task : cases) {
    if (!task.solvable || kNotLoaded.count(task.feature)) {
      continue;
    }
    // Actions that have to overlap are not for the simple builder
    if (bt_builder == "SimpleBTBuilder" && task.feature == "required-concurrency") {
      continue;
    }
    const char * only = std::getenv("PLANSYS2_BANK_CASE");  // To look at one
    if (only && task.name != only) {
      continue;
    }
    SCOPED_TRACE(task.name);
    execute(task, bt_builder);
    plansys2::drain_ros(50ms);
    executed++;
  }
  if (!std::getenv("PLANSYS2_BANK_CASE")) {
    EXPECT_GE(executed, 28);
  }
  const auto took = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
  EXPECT_LT(took.count(), 120.0);
}

// PDDL3: a hard constraint the world breaks while the plan runs fails the plan
TEST_P(ExecutorBankTest, a_violated_constraint_fails_the_plan)
{
  const auto cases = parser::pddl::read_pddl_bank(
    ament_index_cpp::get_package_share_path("plansys2_pddl_parser") / "bank");
  const auto task = std::find_if(
    cases.begin(), cases.end(), [](const auto & c) {return c.name == "trajectory-constraints/p2";});
  ASSERT_NE(task, cases.end());
  // (always (item_at book office)): somebody takes the book away
  execute(
    *task, GetParam(), [](plansys2::ProblemExpertClient & problem) {
      ASSERT_TRUE(
        problem.removePredicate(parser::pddl::fromStringPredicate("(item_at book office)")));
    }, plansys2_msgs::action::ExecutePlan::Result::FAILURE);
}

INSTANTIATE_TEST_SUITE_P(
  builders, ExecutorBankTest, ::testing::Values("STNBTBuilder", "SimpleBTBuilder"),
  [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
