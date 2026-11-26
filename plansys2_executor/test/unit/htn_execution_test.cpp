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

// HDDL mode: hierarchical (HTN) plans, executed by the tree of their decomposition

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/PackagePaths.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/bt_builder_plugins/htn_bt_builder.hpp"
#include "plansys2_msgs/msg/execution_event.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using Event = plansys2_msgs::msg::ExecutionEvent;
using Result = plansys2_msgs::action::ExecutePlan::Result;

namespace
{

plansys2_msgs::msg::PlanItem item(double time, const std::string & action, double duration = 0)
{
  plansys2_msgs::msg::PlanItem i;
  i.time = time;
  i.action = action;
  i.duration = duration;
  return i;
}

plansys2_msgs::msg::PlanTask task(
  const std::string & id, const std::string & name, const std::string & method,
  const std::vector<std::string> & subtasks, const std::vector<uint32_t> & ordering = {},
  const std::string & precondition = "", double applied = -0.5)
{
  plansys2_msgs::msg::PlanTask t;
  t.applied = applied;
  t.id = id;
  t.task = name;
  t.method = method;
  t.subtasks = subtasks;
  t.ordering = ordering;
  t.precondition = precondition;
  return t;
}

// Two deliveries that the initial network does not order, done one after the other
plansys2_msgs::msg::Plan two_tasks()
{
  plansys2_msgs::msg::Plan plan;
  plan.items = {
    item(0, "(pick tiago cup kitchen)"), item(1, "(drop tiago cup bedroom)"),
    item(2, "(pick tiago book bedroom)"), item(3, "(drop tiago book kitchen)")};
  plan.tasks = {
    task("root", "", "", {"1", "2"}),
    task("1", "(deliver cup bedroom)", "m", {"item:0", "item:1"}, {0, 1}, "(free tiago)"),
    task("2", "(deliver book kitchen)", "m", {"item:2", "item:3"}, {0, 1}, "(free tiago)", 1.5)};
  return plan;
}

// The house of house.hddl: the plan of an HDDL planner (Klimax) for house_problem.hddl, the
// cup to the bedroom and the book to the kitchen, interleaved
plansys2_msgs::msg::Plan house_plan()
{
  plansys2_msgs::msg::Plan plan;
  const std::vector<std::string> actions = {
    "(move tiago hall kitchen)", "(move tiago kitchen hall)", "(move tiago hall bedroom)",
    "(pick tiago book bedroom)", "(move tiago bedroom hall)", "(move tiago hall kitchen)",
    "(drop tiago book kitchen)", "(pick tiago cup kitchen)", "(move tiago kitchen hall)",
    "(move tiago hall bedroom)", "(drop tiago cup bedroom)"};
  for (size_t i = 0; i < actions.size(); ++i) {
    plan.items.push_back(item(static_cast<double>(i), actions[i]));
  }
  // Each method applied once the actions before it have ended: `after` of them
  auto go = [](const std::string & id, const std::string & robot_at, const std::string & from,
    const std::string & to, const std::vector<std::string> & subtasks, int after) {
      return task(
        id, "(go_to tiago " + to + ")", "go_step", subtasks, {0, 1},
        "(and (robot_at tiago " + robot_at + ") (connected " + robot_at + " " + from +
        ") (not (robot_at tiago " + to + ")))", after - 0.5);
    };
  auto arrived = [](const std::string & id, const std::string & to, int after) {
      return task(
        id, "(go_to tiago " + to + ")", "go_arrived", {}, {}, "(robot_at tiago " + to + ")",
        after - 0.5);
    };
  const std::vector<uint32_t> in_a_row = {0, 1, 1, 2, 2, 3};
  plan.tasks = {
    task("root", "", "", {"cup", "book"}),
    task(
      "cup", "(deliver cup bedroom)", "deliver_item", {"g1", "item:7", "g2", "item:10"},
      in_a_row, "(and (item_at cup kitchen) (free tiago))"),
    go("g1", "hall", "kitchen", "kitchen", {"item:0", "g1b"}, 0), arrived("g1b", "kitchen", 1),
    go("g2", "kitchen", "hall", "bedroom", {"item:8", "g2b"}, 8),
    go("g2b", "hall", "bedroom", "bedroom", {"item:9", "g2c"}, 9), arrived("g2c", "bedroom", 10),
    task(
      "book", "(deliver book kitchen)", "deliver_item", {"g3", "item:3", "g4", "item:6"},
      in_a_row, "(and (item_at book bedroom) (free tiago))"),
    go("g3", "kitchen", "hall", "bedroom", {"item:1", "g3b"}, 1),
    go("g3b", "hall", "bedroom", "bedroom", {"item:2", "g3c"}, 2), arrived("g3c", "bedroom", 3),
    go("g4", "bedroom", "hall", "kitchen", {"item:4", "g4b"}, 4),
    go("g4b", "hall", "kitchen", "kitchen", {"item:5", "g4c"}, 5), arrived("g4c", "kitchen", 6)};
  return plan;
}

size_t count(const std::string & text, const std::string & what)
{
  size_t n = 0;
  for (auto at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) {
    ++n;
  }
  return n;
}

}  // namespace

TEST(htn_bt_builder, total_order)
{
  using plansys2::HTNBTBuilder;
  EXPECT_EQ(HTNBTBuilder::total_order(1, {}), (std::vector<size_t>{0}));
  EXPECT_EQ(HTNBTBuilder::total_order(3, {0, 1, 1, 2}), (std::vector<size_t>{0, 1, 2}));
  EXPECT_EQ(HTNBTBuilder::total_order(3, {2, 0, 0, 1}), (std::vector<size_t>{2, 0, 1}));
  // Transitive pairs too
  EXPECT_EQ(HTNBTBuilder::total_order(3, {0, 1, 0, 2, 1, 2}), (std::vector<size_t>{0, 1, 2}));
  EXPECT_FALSE(HTNBTBuilder::total_order(2, {}));
  EXPECT_FALSE(HTNBTBuilder::total_order(3, {0, 1, 0, 2}));
  EXPECT_FALSE(HTNBTBuilder::total_order(2, {0, 1, 1, 0}));  // a cycle
  EXPECT_FALSE(HTNBTBuilder::total_order(2, {0, 5}));
}

TEST(htn_bt_builder, the_tree_is_the_decomposition)
{
  plansys2::HTNBTBuilder builder;
  builder.initialize();
  const std::string tree = builder.get_tree(two_tasks());
  // A node per task, with its method; the precondition of each method is checked at runtime
  EXPECT_NE(tree.find("<Sequence name=\"initial task network\">"), std::string::npos) << tree;
  EXPECT_NE(tree.find("<Sequence name=\"(deliver cup bedroom) ← m\">"), std::string::npos);
  EXPECT_EQ(count(tree, "<CheckMethodPrecondition"), 2u) << tree;
  // Not ordered by the network: at once, each action waiting for the one before in the plan
  EXPECT_EQ(count(tree, "<Parallel success_count=\"2\" failure_count=\"1\">"), 1u) << tree;
  EXPECT_EQ(count(tree, "<ExecuteAction"), 4u) << tree;
  EXPECT_NE(
    tree.find("<WaitAction action=\"(drop tiago cup bedroom):1000\"/>"), std::string::npos) << tree;
  // The second delivery checks its method once the first is done
  const auto check = tree.find("task=\"(deliver book kitchen)\"");
  const auto wait = tree.rfind("<WaitAction action=\"(drop tiago cup bedroom):1000\"/>", check);
  EXPECT_NE(wait, std::string::npos) << tree;
  EXPECT_LT(tree.find("(deliver cup bedroom) ←"), wait) << tree;

  // With times that overlap, the two picks start at once and wait for nothing; each drop
  // waits for what ends before it starts
  auto timed = two_tasks();
  timed.tasks[2].applied = 0.0;  // both deliveries at the start
  timed.items = {
    item(0, "(pick tiago cup kitchen)", 2), item(2, "(drop tiago cup bedroom)", 2),
    item(0, "(pick tiago book bedroom)", 1), item(1, "(drop tiago book kitchen)", 1)};
  const std::string at_once = builder.get_tree(timed);
  EXPECT_EQ(count(at_once, "<WaitAction"), 3u) << at_once;
  const auto book = at_once.find("<Sequence name=\"(pick tiago book bedroom):0\">");
  ASSERT_NE(book, std::string::npos) << at_once;
  const auto starts = at_once.find("<WaitAtStartReq", book);
  ASSERT_NE(starts, std::string::npos) << at_once;
  EXPECT_EQ(at_once.substr(book, starts - book).find("<WaitAction"), std::string::npos) << at_once;

  // A method with no subtasks, applied once an action has ended: its check waits for it, even
  // where nothing orders it after that action (as a robot that has to be there already)
  auto waiting = two_tasks();
  waiting.tasks[0].subtasks.push_back("3");
  waiting.tasks.push_back(
    task("3", "(ready tiago)", "here", {}, {}, "(robot_at tiago bedroom)", 1.5));
  const std::string ready = builder.get_tree(waiting);
  const auto here = ready.find("task=\"(ready tiago)\"");
  ASSERT_NE(here, std::string::npos) << ready;
  const auto before = ready.rfind("<WaitAction action=\"(drop tiago cup bedroom):1000\"/>", here);
  ASSERT_NE(before, std::string::npos) << ready;
  EXPECT_GT(before, ready.find("<Sequence name=\"(ready tiago) \u2190 here\">")) << ready;

  // Not a hierarchical plan, or one whose decomposition names what is not in it
  EXPECT_TRUE(builder.get_tree(plansys2_msgs::msg::Plan()).empty());
  auto wrong = two_tasks();
  wrong.tasks[1].subtasks.push_back("item:9");
  EXPECT_TRUE(builder.get_tree(wrong).empty());
}

namespace
{

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

// PlanSys2 in HDDL mode, and a performer per action of the house. The plan is given: no
// planner of HDDL is needed
struct House
{
  explicit House(const std::string & bt_builder = "HTNBTBuilder")
  {
    const std::string pkg = plansys2::share_path("plansys2_executor").string();
    domain_node = std::make_shared<plansys2::DomainExpertNode>();
    problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    executor_node = std::make_shared<plansys2::ExecutorNode>();
    domain_node->set_parameter({"model_file", pkg + "/pddl/house.hddl"});
    problem_node->set_parameter({"model_file", pkg + "/pddl/house.hddl"});
    problem_node->set_parameter({"problem_file", pkg + "/pddl/house_problem.hddl"});
    executor_node->set_parameter({"bt_builder_plugin", bt_builder});
    experts = {domain_node, problem_node, executor_node};
    for (auto & node : experts) {
      exe.add_node(node->get_node_base_interface());
    }
    listener = rclcpp::Node::make_shared("htn_trace");
    trace_sub = listener->create_subscription<Event>(
      "executor/execution_trace", rclcpp::QoS(1000), [this](Event::SharedPtr event) {
        std::lock_guard<std::mutex> lock(trace_mutex);
        trace.push_back(*event);
      });
    exe.add_node(listener);
    for (const std::string action : {"move", "pick", "drop"}) {
      performers.push_back(std::make_shared<Performer>("htn_" + action, action));
      exe.add_node(performers.back()->get_node_base_interface());
    }
    spinner = std::make_unique<plansys2::testing::ExecutorThread>(exe);
    for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
      for (auto & node : experts) {
        node->trigger_transition(transition);
      }
    }
    for (auto & performer : performers) {
      performer->trigger_transition(Transition::TRANSITION_CONFIGURE);
    }
    client_node = rclcpp::Node::make_shared("htn_client");
    client_spinner = std::make_unique<plansys2::testing::NodeSpinner>(client_node);
    domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);
    problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
    executor_client = std::make_shared<plansys2::ExecutorClient>(client_node);
  }

  ~House()
  {
    spinner.reset();
  }

  bool active() const
  {
    return std::all_of(
      experts.begin(), experts.end(), [](const auto & node) {
        return node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      });
  }


  // Runs it to its end: the result, or nullopt if it does not end
  std::optional<int8_t> execute(const plansys2_msgs::msg::Plan & plan)
  {
    if (!executor_client->start_plan_execution(plan)) {
      return std::nullopt;
    }
    if (!plansys2::testing::wait_for(
        [&]() {return !executor_client->execute_and_check_plan();}, 20s))
    {
      executor_client->cancel_plan_execution();
      return std::nullopt;
    }
    auto result = executor_client->getResult();
    return result ? std::optional<int8_t>(result->result) : std::nullopt;
  }

  std::vector<Event> events()
  {
    plansys2::testing::wait_for(
      [&]() {
        std::lock_guard<std::mutex> lock(trace_mutex);
        return !trace.empty() && trace.back().type == Event::PLAN_END;
      }, 2s);
    std::lock_guard<std::mutex> lock(trace_mutex);
    return trace;
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node;
  std::shared_ptr<plansys2::ExecutorNode> executor_node;
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> experts;
  std::vector<std::shared_ptr<plansys2::ActionExecutorClient>> performers;
  rclcpp::executors::MultiThreadedExecutor exe;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner;
  rclcpp::Node::SharedPtr listener;
  rclcpp::Subscription<Event>::SharedPtr trace_sub;
  std::mutex trace_mutex;
  std::vector<Event> trace;
  rclcpp::Node::SharedPtr client_node;
  std::unique_ptr<plansys2::testing::NodeSpinner> client_spinner;
  std::shared_ptr<plansys2::DomainExpertClient> domain_client;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client;
  std::shared_ptr<plansys2::ExecutorClient> executor_client;
};

}  // namespace

TEST(htn_execution, a_plan_runs_by_its_decomposition)
{
  {
    House house;
    ASSERT_TRUE(house.active());
    auto plan = std::optional<plansys2_msgs::msg::Plan>(house_plan());
    EXPECT_EQ(house.execute(*plan), Result::SUCCESS);
    EXPECT_TRUE(
      house.problem_client->existPredicate(
        parser::pddl::fromStringPredicate("(item_at cup bedroom)")));
    EXPECT_TRUE(
      house.problem_client->existPredicate(
        parser::pddl::fromStringPredicate("(item_at book kitchen)")));
  }
  plansys2::drain_ros(200ms);
}

TEST(htn_execution, a_method_that_no_longer_applies_fails_its_task)
{
  {
    House house;
    ASSERT_TRUE(house.active());
    auto plan = std::optional<plansys2_msgs::msg::Plan>(house_plan());
    // After planning, the cup is no longer in the kitchen: its delivery does not apply
    ASSERT_TRUE(
      house.problem_client->removePredicate(
        parser::pddl::fromStringPredicate("(item_at cup kitchen)")));
    ASSERT_TRUE(
      house.problem_client->addPredicate(
        parser::pddl::fromStringPredicate("(item_at cup hall)")));
    EXPECT_EQ(house.execute(*plan), Result::FAILURE);
    // The task fails, before any of its actions: the trace says which and why
    bool said = false;
    for (const auto & event : house.events()) {
      if (event.type == Event::FAILURE && event.action == "(deliver cup bedroom)") {
        said = event.detail.find("deliver_item") != std::string::npos;
      }
      EXPECT_FALSE(event.type == Event::START && event.action.rfind("(pick tiago cup", 0) == 0);
    }
    EXPECT_TRUE(said);
  }
  plansys2::drain_ros(200ms);
}

TEST(htn_execution, a_hierarchical_plan_needs_the_htn_builder)
{
  {
    House house("STNBTBuilder");
    ASSERT_TRUE(house.active());
    EXPECT_NE(house.execute(house_plan()), Result::SUCCESS);
  }
  plansys2::drain_ros(200ms);
}

TEST(htn_execution, the_htn_builder_needs_a_hierarchical_plan)
{
  {
    House house;
    ASSERT_TRUE(house.active());
    auto plan = house_plan();
    plan.tasks.clear();
    EXPECT_NE(house.execute(plan), Result::SUCCESS);
    // Nothing was done
    EXPECT_TRUE(
      house.problem_client->existPredicate(
        parser::pddl::fromStringPredicate("(item_at cup kitchen)")));
  }
  plansys2::drain_ros(200ms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
