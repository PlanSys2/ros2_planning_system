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

// End-to-end tests for the dynamic (hot) domain swap feature: changing the PDDL
// domain at runtime through DomainExpertClient::changeDomain(), with all four core
// nodes (domain_expert, problem_expert, planner, executor) wired up as real nodes,
// mirroring test_4's setup. See refactor_dynamic_domain.md for the design.

#include <atomic>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_path.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "plansys2_tests/test_action_node.hpp"

namespace
{

std::string read_pddl(const std::string & pkgpath, const std::string & filename)
{
  std::ifstream ifs(pkgpath + "/test_5/pddl/" + filename);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

void sleep_for(rclcpp::Node::SharedPtr node, double seconds)
{
  rclcpp::Rate rate(10);
  auto start = node->now();
  while ((node->now() - start).seconds() < seconds) {
    rate.sleep();
  }
}

// Joins its spinning thread via RAII: an ASSERT_* failure returns from the test
// immediately, and a std::thread destroyed while still joinable calls
// std::terminate(), aborting the whole binary instead of just failing this test.
class ExecutorSpinner
{
public:
  explicit ExecutorSpinner(rclcpp::Executor & exe)
  : finish_(false), thread_([this, &exe]() {while (!finish_) {exe.spin_some();}}) {}

  ~ExecutorSpinner()
  {
    finish_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::atomic<bool> finish_;
  std::thread thread_;
};

}  // namespace

TEST(test_5, requirement_only_change_preserves_problem)
{
  auto test_node = rclcpp::Node::make_shared("test_5_test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_tests").string();
  domain_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  ExecutorSpinner spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sleep_for(test_node, 0.5);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  sleep_for(test_node, 0.5);

  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(dummy)")));
  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (door_open))")));

  // test_5_extra_requirement.pddl adds a harmless extra precondition to open_door;
  // no predicate/function signature changes, so nothing in the problem should move.
  ASSERT_TRUE(domain_client->changeDomain(read_pddl(pkgpath, "test_5_extra_requirement.pddl")));

  ASSERT_EQ(
    domain_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(
    problem_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(dummy)")));
  ASSERT_EQ(parser::pddl::toString(problem_client->getGoal()), "(and (door_open))");
}

TEST(test_5, removed_predicate_pruned)
{
  auto test_node = rclcpp::Node::make_shared("test_5_test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_tests").string();
  domain_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  ExecutorSpinner spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sleep_for(test_node, 0.5);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  sleep_for(test_node, 0.5);

  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(dummy)")));

  // test_5_removed_predicate.pddl drops `dummy` from the domain entirely.
  ASSERT_TRUE(domain_client->changeDomain(read_pddl(pkgpath, "test_5_removed_predicate.pddl")));

  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(dummy)")));
}

TEST(test_5, invalid_domain_rejected)
{
  auto test_node = rclcpp::Node::make_shared("test_5_test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_tests").string();
  domain_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  ExecutorSpinner spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sleep_for(test_node, 0.5);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  sleep_for(test_node, 0.5);

  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(not_door_open)")));
  auto domain_name_before = domain_client->getName();

  ASSERT_FALSE(domain_client->changeDomain(read_pddl(pkgpath, "test_5_error.pddl")));

  // Rejected before touching anything.
  ASSERT_EQ(
    domain_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(
    problem_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(domain_client->getName(), domain_name_before);
  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(not_door_open)")));
}

// Concern: the transient_local replay of domain_expert/domain that every subscriber
// sees on startup (ProblemExpertNode, ExecutorNode) must be treated as a baseline, not
// mistaken for a genuine runtime change — changeDomain() is never called in this test
// at all. If the baseline-skip in either node's domain_topic_callback() were wrong,
// this would show up as either a spurious problem-knowledge reconcile (pruning the
// predicate/goal set below right after they're set) or the executor aborting its own
// plan the moment it activates. A normal plan running start-to-finish with no
// changeDomain() call anywhere is a direct test that neither happens.
TEST(test_5, normal_startup_does_not_disturb_executor_or_problem)
{
  auto test_node = rclcpp::Node::make_shared("test_5_test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto open_door_action_node = plansys2_tests::TestAction::make_shared("open_door");
  auto close_door_action_node = plansys2_tests::TestAction::make_shared("close_door");

  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_tests").string();
  domain_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(open_door_action_node->get_node_base_interface());
  exe.add_node(close_door_action_node->get_node_base_interface());
  ExecutorSpinner spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sleep_for(test_node, 0.5);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  // Give the transient_local baseline replay of domain_expert/domain plenty of time to
  // reach problem_expert's and executor's subscriptions before anything else happens.
  sleep_for(test_node, 1.0);

  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (door_open))")));

  // Nothing should have been pruned by a spurious reconcile: both the instance-free
  // predicate and the goal set immediately above must still be exactly as set.
  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_EQ(parser::pddl::toString(problem_client->getGoal()), "(and (door_open))");

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  bool finished = false;
  rclcpp::Rate rate(5);
  auto start = test_node->now();
  while (!finished && (test_node->now() - start).seconds() < 10.0) {
    finished = !executor_client->execute_and_check_plan();
    rate.sleep();
  }
  ASSERT_TRUE(finished);

  auto result = executor_client->getResult();
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result.value().result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);

  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(door_open)")));
  ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(not_door_open)")));
}

TEST(test_5, change_domain_cancels_active_execution)
{
  auto test_node = rclcpp::Node::make_shared("test_5_test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  // Slow action (0.1/tick at 1 Hz => ~10 s to complete): gives us a wide window to
  // call changeDomain() while it is still mid-execution.
  auto open_door_action_node = plansys2_tests::TestAction::make_shared("open_door", 0.1f);
  auto close_door_action_node = plansys2_tests::TestAction::make_shared("close_door", 0.4f);

  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_tests").string();
  domain_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(open_door_action_node->get_node_base_interface());
  exe.add_node(close_door_action_node->get_node_base_interface());
  ExecutorSpinner spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sleep_for(test_node, 0.5);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  sleep_for(test_node, 0.5);

  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (door_open))")));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);
  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  // Let it start running (well before the ~10 s it needs to finish on its own).
  sleep_for(test_node, 1.0);

  // Same domain content back in: this test is about the executor being stopped and
  // the system staying usable, not about what changes in the domain this time.
  ASSERT_TRUE(domain_client->changeDomain(domain));

  // The in-flight plan must not have been left dangling: execute_and_check_plan()
  // has to converge to "finished" (one way or another) instead of hanging forever
  // waiting for an action that will never report back to a cancelled executor.
  bool finished = false;
  rclcpp::Rate rate(5);
  auto start = test_node->now();
  while (!finished && (test_node->now() - start).seconds() < 10.0) {
    finished = !executor_client->execute_and_check_plan();
    rate.sleep();
  }
  ASSERT_TRUE(finished);

  auto result = executor_client->getResult();
  ASSERT_TRUE(result.has_value());
  ASSERT_NE(result.value().result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);

  // The system is still usable afterwards: a fresh plan can be requested and *run to
  // completion* — not just accepted, actually executed successfully.
  ASSERT_EQ(
    domain_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (door_open))")));
  auto plan2 = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan2.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan2.value()));
  finished = false;
  start = test_node->now();
  while (!finished && (test_node->now() - start).seconds() < 20.0) {
    finished = !executor_client->execute_and_check_plan();
    rate.sleep();
  }
  ASSERT_TRUE(finished);

  auto result2 = executor_client->getResult();
  ASSERT_TRUE(result2.has_value());
  ASSERT_EQ(result2.value().result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);
}

TEST(test_5, plan_executes_successfully_after_domain_change)
{
  auto test_node = rclcpp::Node::make_shared("test_5_test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto open_door_action_node = plansys2_tests::TestAction::make_shared("open_door");
  auto close_door_action_node = plansys2_tests::TestAction::make_shared("close_door");

  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_tests").string();
  domain_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_5/pddl/test_5.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(open_door_action_node->get_node_base_interface());
  exe.add_node(close_door_action_node->get_node_base_interface());
  ExecutorSpinner spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  sleep_for(test_node, 0.5);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  sleep_for(test_node, 0.5);

  ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate("(not_door_open)")));
  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (door_open))")));

  ASSERT_TRUE(domain_client->changeDomain(read_pddl(pkgpath, "test_5_removed_predicate.pddl")));
  // Give problem_expert's own subscription time to reconcile before planning against
  // its (should-be-unaffected) post-change state.
  sleep_for(test_node, 0.5);

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  bool finished = false;
  rclcpp::Rate rate(5);
  auto start = test_node->now();
  while (!finished && (test_node->now() - start).seconds() < 20.0) {
    finished = !executor_client->execute_and_check_plan();
    rate.sleep();
  }
  ASSERT_TRUE(finished);

  auto result = executor_client->getResult();
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result.value().result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);

  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(door_open)")));
  ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(not_door_open)")));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
