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

// The orders the STN of a plan of the bank has to keep, whatever the performers take: the
// executor runs actions as soon as their parents allow it

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/bt_builder_plugins/stn_bt_builder.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"

using Transition = lifecycle_msgs::msg::Transition;
using plansys2::ActionType;

namespace
{

class TestableSTN : public plansys2::STNBTBuilder
{
public:
  using STNBTBuilder::build_stn;
  using STNBTBuilder::check_paths;
  using STNBTBuilder::get_tree;
};

plansys2_msgs::msg::Plan make_plan(
  const std::vector<std::tuple<double, std::string, double>> & items)
{
  plansys2_msgs::msg::Plan plan;
  for (const auto & [time, action, duration] : items) {
    plansys2_msgs::msg::PlanItem item;
    item.time = time;
    item.action = action;
    item.duration = duration;
    plan.items.push_back(item);
  }
  return plan;
}

plansys2::Node::Ptr find(
  const plansys2::Graph::Ptr & graph, const std::string & expression, double time,
  ActionType type)
{
  for (const auto & node : graph->nodes) {
    if (node->action.expression == expression && node->action.type == type &&
      std::abs(node->action.time - time) < 1e-6)
    {
      return node;
    }
  }
  return nullptr;
}

// One task of the bank loaded in the experts
class STNBankTest : public ::testing::Test
{
protected:
  void load(const std::string & name)
  {
    const auto cases = parser::pddl::read_pddl_bank(
      ament_index_cpp::get_package_share_path("plansys2_pddl_parser") / "bank");
    const auto it = std::find_if(
      cases.begin(), cases.end(), [&](const auto & c) {return c.name == name;});
    ASSERT_NE(it, cases.end()) << name;

    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    domain_node_->set_parameter({"model_file", it->domain_file.string()});
    problem_node_->set_parameter({"model_file", it->domain_file.string()});
    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
    for (auto node : {std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(domain_node_),
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(problem_node_)})
    {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
      node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }
    ASSERT_TRUE(std::make_shared<plansys2::ProblemExpertClient>()->addProblem(it->problem));
    builder_ = std::make_shared<TestableSTN>();
    builder_->initialize("", "", 3);
  }

  void TearDown() override
  {
    builder_.reset();
    spinner_.reset();
  }

  // b only after a: a path from a to b
  bool after(
    const plansys2::Graph::Ptr & graph, std::tuple<std::string, double, ActionType> a,
    std::tuple<std::string, double, ActionType> b)
  {
    auto from = find(graph, std::get<0>(a), std::get<1>(a), std::get<2>(a));
    auto to = find(graph, std::get<0>(b), std::get<1>(b), std::get<2>(b));
    EXPECT_TRUE(from && to);
    return from && to && builder_->check_paths(to, from);
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  rclcpp::executors::SingleThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  std::shared_ptr<TestableSTN> builder_;
};

}  // namespace

// A conditional effect, applied at the end of its action, gives what the plan says only after
// what makes its condition true
TEST_F(STNBankTest, a_conditional_effect_after_its_condition)
{
  load("conditional-effects/p2");
  auto graph = builder_->build_stn(
    make_plan({
    {0.0, "(restore_power tiago main_panel kitchen_button kitchen)", 0.0},
    {1.0, "(restore_power tiago main_panel lab_button kitchen)", 0.0},
    {2.0, "(press_button tiago kitchen_button kitchen_door kitchen)", 0.0},
    {3.0, "(move tiago kitchen_door kitchen corridor)", 0.0}}));
  EXPECT_TRUE(
    after(
      graph, {"(restore_power tiago main_panel kitchen_button kitchen)", 0.0, ActionType::END},
      {"(press_button tiago kitchen_button kitchen_door kitchen)", 2.0, ActionType::END}));
  EXPECT_TRUE(
    after(
      graph, {"(press_button tiago kitchen_button kitchen_door kitchen)", 2.0, ActionType::END},
      {"(move tiago kitchen_door kitchen corridor)", 3.0, ActionType::START}));
}

// A non-durative action that deletes what another one needs waits for it to end
TEST_F(STNBankTest, a_non_durative_threat_waits_for_the_end)
{
  load("negative-goals/p2");
  auto graph = builder_->build_stn(
    make_plan({
    {0.0, "(move r1 hall kitchen)", 0.0}, {1.0, "(clean r1 kitchen)", 0.0},
    {2.0, "(move r1 kitchen hall)", 0.0}}));
  EXPECT_TRUE(
    after(
      graph, {"(move r1 hall kitchen)", 0.0, ActionType::END},
      {"(clean r1 kitchen)", 1.0, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(move r1 hall kitchen)", 0.0, ActionType::END},
      {"(move r1 kitchen hall)", 2.0, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(clean r1 kitchen)", 1.0, ActionType::END},
      {"(move r1 kitchen hall)", 2.0, ActionType::START}));
}

// The door is held open until the robot that passes through it is on the other side
TEST_F(STNBankTest, required_concurrency_ends_in_order)
{
  load("required-concurrency/p2");
  auto graph = builder_->build_stn(
    make_plan({
    {0.0, "(hold_door r1 office_door corridor office)", 10.0},
    {8.002, "(hold_door r3 lab_door office lab)", 10.0},
    {10.001, "(hold_door r2 office_door corridor office)", 10.0},
    {10.001, "(pass_door r1 office_door corridor office)", 4.0},
    {14.002, "(pass_door r1 lab_door office lab)", 4.0},
    {20.002, "(hold_door r3 office_door office corridor)", 10.0},
    {20.002, "(pass_door r2 office_door corridor office)", 4.0},
    {30.003, "(hold_door r2 office_door office corridor)", 10.0},
    {30.003, "(pass_door r3 office_door office corridor)", 4.0}}));
  // Held open before it passes, closed after it has passed
  EXPECT_TRUE(
    after(
      graph, {"(hold_door r2 office_door corridor office)", 10.001, ActionType::START},
      {"(pass_door r1 office_door corridor office)", 10.001, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(pass_door r1 office_door corridor office)", 10.001, ActionType::END},
      {"(hold_door r2 office_door corridor office)", 10.001, ActionType::END}));
  EXPECT_TRUE(
    after(
      graph, {"(pass_door r1 lab_door office lab)", 14.002, ActionType::END},
      {"(hold_door r3 lab_door office lab)", 8.002, ActionType::END}));
  // The first hold closes the door before the second one opens it: the pass waits for that one
  EXPECT_TRUE(
    after(
      graph, {"(hold_door r1 office_door corridor office)", 0.0, ActionType::END},
      {"(hold_door r2 office_door corridor office)", 10.001, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(pass_door r2 office_door corridor office)", 20.002, ActionType::END},
      {"(hold_door r3 office_door office corridor)", 20.002, ActionType::END}));
}

// Each action is executed once in its tree, from its start: an end with another parent too
// had its execution block there as well, and checked its requirements before its start
TEST_F(STNBankTest, each_action_runs_once_in_its_tree)
{
  load("processes-events/p1");
  const auto tree = builder_->get_tree(
    make_plan({{0.0, "(start_charge r1 dock)", 0.0}, {10.0, "(move r1 dock hall)", 0.0}}));
  ASSERT_FALSE(tree.empty());
  for (const std::string action : {"(start_charge r1 dock):0", "(move r1 dock hall):10000"}) {
    size_t count = 0;
    for (auto pos = tree.find("<ExecuteAction action=\"" + action + "\"");
      pos != std::string::npos;
      pos = tree.find("<ExecuteAction action=\"" + action + "\"", pos + 1))
    {
      count++;
    }
    EXPECT_EQ(count, 1u) << action << "\n" << tree;
  }
}

// The action of a plan item, with the duration the plan chose wherever it is read
TEST_F(STNBankTest, plan_items_bind_their_duration)
{
  load("duration-inequalities/p1");
  plansys2::DomainExpertClient domain_client;
  const auto actions = domain_client.getActions();
  plansys2_msgs::msg::PlanItem item;
  item.action = "(charge r1 c1 dock)";
  item.time = 0.0;
  item.duration = 6.5;
  auto action = plansys2::BTBuilder::plan_item_action(domain_client, actions, item);
  ASSERT_TRUE(action.is_durative_action());
  const auto effects = parser::pddl::toString(action.get_at_end_effects());
  EXPECT_EQ(effects.find("?duration"), std::string::npos) << effects;
  EXPECT_NE(effects.find("6.500000"), std::string::npos) << effects;

  // The plan state follows: the charge gives 10 per second of it
  std::vector<plansys2::Predicate> predicates;
  auto battery = parser::pddl::fromStringFunction("(= (battery r1) 0)");
  std::vector<plansys2::Function> functions = {battery};
  ASSERT_TRUE(plansys2::apply(action.get_at_end_effects(), predicates, functions));
  EXPECT_DOUBLE_EQ(functions[0].value, 65.0);

  // Unknown actions give nothing
  item.action = "(fly r1)";
  EXPECT_TRUE(plansys2::BTBuilder::plan_item_action(domain_client, actions, item).is_empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
