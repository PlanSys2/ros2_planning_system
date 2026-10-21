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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;

// apply records what an effect changes, and undo takes it back
class EffectUndoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    domain_node_->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    domain_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    problem_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    domain_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    problem_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);

    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);

    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    for (const auto & robot : {"r1", "r2"}) {
      ASSERT_TRUE(client_->addInstance(plansys2::Instance(robot, "robot")));
    }
    for (const auto & room : {"kitchen", "bedroom"}) {
      ASSERT_TRUE(client_->addInstance(plansys2::Instance(room, "room")));
    }
    ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r1 kitchen)")));
    ASSERT_TRUE(client_->addFunction(plansys2::Function("(= (room_distance kitchen bedroom) 10)")));
  }

  void TearDown() override
  {
    client_.reset();
    spinner_.reset();
  }

  static plansys2_msgs::msg::Tree tree_of(const std::string & expression)
  {
    plansys2_msgs::msg::Tree ret;
    parser::pddl::fromString(ret, expression, false, plansys2_msgs::msg::Node::AND);
    return ret;
  }

  bool holds(const std::string & predicate)
  {
    return client_->existPredicate(plansys2::Predicate(predicate));
  }

  double distance()
  {
    auto function = client_->getFunction("(room_distance kitchen bedroom)");
    return function.has_value() ? function->value : -1.0;
  }

  void set_distance(double value)
  {
    auto function = plansys2::Function("(= (room_distance kitchen bedroom) 0)");
    function.value = value;
    ASSERT_TRUE(client_->updateFunction(function));
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> client_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

TEST_F(EffectUndoTest, predicates_go_back)
{
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of("(and (not (robot_at r1 kitchen)) (robot_at r1 bedroom))"), client_, changes));
  ASSERT_EQ(changes.size(), 2u);
  ASSERT_FALSE(holds("(robot_at r1 kitchen)"));
  ASSERT_TRUE(holds("(robot_at r1 bedroom)"));

  ASSERT_TRUE(plansys2::undo(changes, client_));
  ASSERT_TRUE(holds("(robot_at r1 kitchen)"));
  ASSERT_FALSE(holds("(robot_at r1 bedroom)"));
}

TEST_F(EffectUndoTest, facts_that_already_held_are_not_touched)
{
  // Both already are as the effect says: nothing changes, nothing to take back
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of("(and (robot_at r1 kitchen) (not (robot_at r1 bedroom)))"), client_, changes));
  ASSERT_TRUE(changes.empty());

  ASSERT_TRUE(plansys2::undo(changes, client_));
  ASSERT_TRUE(holds("(robot_at r1 kitchen)"));
  ASSERT_FALSE(holds("(robot_at r1 bedroom)"));
}

TEST_F(EffectUndoTest, predicates_changed_since_keep_their_value)
{
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of("(and (not (robot_at r1 kitchen)) (robot_at r1 bedroom))"), client_, changes));

  // Someone else already put both back, and changed another fact
  ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r1 kitchen)")));
  ASSERT_TRUE(client_->removePredicate(plansys2::Predicate("(robot_at r1 bedroom)")));
  ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r2 bedroom)")));

  ASSERT_TRUE(plansys2::undo(changes, client_));
  ASSERT_TRUE(holds("(robot_at r1 kitchen)"));
  ASSERT_FALSE(holds("(robot_at r1 bedroom)"));
  ASSERT_TRUE(holds("(robot_at r2 bedroom)"));
}

TEST_F(EffectUndoTest, numeric_effects_go_back)
{
  for (const auto & effect : {
    "(increase (room_distance kitchen bedroom) 5)",
    "(decrease (room_distance kitchen bedroom) 2.5)",
    "(assign (room_distance kitchen bedroom) 3)",
    "(scale-up (room_distance kitchen bedroom) 4)",
    "(scale-down (room_distance kitchen bedroom) 8)"})
  {
    std::vector<plansys2::FactChange> changes;
    ASSERT_TRUE(plansys2::apply(tree_of(effect), client_, changes)) << effect;
    ASSERT_EQ(changes.size(), 1u) << effect;
    ASSERT_NE(distance(), 10.0) << effect;

    ASSERT_TRUE(plansys2::undo(changes, client_)) << effect;
    ASSERT_DOUBLE_EQ(distance(), 10.0) << effect;
  }

  // Assigning the value it already has changes nothing
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(tree_of("(assign (room_distance kitchen bedroom) 10)"), client_, changes));
  ASSERT_TRUE(changes.empty());
}

TEST_F(EffectUndoTest, increases_changed_since_are_taken_back_as_a_delta)
{
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(tree_of("(increase (room_distance kitchen bedroom) 5)"), client_, changes));
  ASSERT_DOUBLE_EQ(distance(), 15.0);

  // Another action increased it by 3 meanwhile: that one stays
  set_distance(18.0);
  ASSERT_TRUE(plansys2::undo(changes, client_));
  ASSERT_DOUBLE_EQ(distance(), 13.0);

  changes.clear();
  ASSERT_TRUE(
    plansys2::apply(tree_of("(decrease (room_distance kitchen bedroom) 3)"), client_, changes));
  set_distance(20.0);
  ASSERT_TRUE(plansys2::undo(changes, client_));
  ASSERT_DOUBLE_EQ(distance(), 23.0);
}

TEST_F(EffectUndoTest, assignments_overwritten_since_keep_their_value)
{
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(tree_of("(assign (room_distance kitchen bedroom) 3)"), client_, changes));
  set_distance(7.0);

  std::vector<std::string> kept;
  ASSERT_TRUE(plansys2::undo(changes, client_, &kept));
  ASSERT_DOUBLE_EQ(distance(), 7.0);
  ASSERT_EQ(kept, std::vector<std::string>{"(room_distance kitchen bedroom)"});
}

TEST_F(EffectUndoTest, forall_effects_go_back)
{
  ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r2 kitchen)")));

  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of("(forall (?r - robot) (not (robot_at ?r kitchen)))"), client_, changes));
  ASSERT_EQ(changes.size(), 2u);
  ASSERT_FALSE(holds("(robot_at r1 kitchen)"));
  ASSERT_FALSE(holds("(robot_at r2 kitchen)"));

  ASSERT_TRUE(plansys2::undo(changes, client_));
  ASSERT_TRUE(holds("(robot_at r1 kitchen)"));
  ASSERT_TRUE(holds("(robot_at r2 kitchen)"));
}

TEST_F(EffectUndoTest, a_failed_numeric_effect_changes_nothing)
{
  // The function does not exist: the numeric effects are computed first, so the effect fails
  // before moving the robot, with nothing to take back
  std::vector<plansys2::FactChange> changes;
  ASSERT_FALSE(
    plansys2::apply(
      tree_of(
        "(and (not (robot_at r1 kitchen)) (increase (room_distance bedroom kitchen) 1))"),
      client_, changes));
  ASSERT_TRUE(changes.empty());
  ASSERT_TRUE(holds("(robot_at r1 kitchen)"));
}

TEST_F(EffectUndoTest, adding_a_fact_wins_over_deleting_it)
{
  for (const std::string effect : {
    "(and (robot_at r1 bedroom) (not (robot_at r1 bedroom)))",
    "(and (not (robot_at r1 bedroom)) (robot_at r1 bedroom))"})
  {
    std::vector<plansys2::FactChange> changes;
    ASSERT_TRUE(plansys2::apply(tree_of(effect), client_, changes)) << effect;
    EXPECT_TRUE(holds("(robot_at r1 bedroom)")) << effect;
    ASSERT_TRUE(plansys2::undo(changes, client_));
    EXPECT_FALSE(holds("(robot_at r1 bedroom)")) << effect;
  }
  // Also for a fact that held: it still holds
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of("(and (not (robot_at r1 kitchen)) (robot_at r1 kitchen))"), client_, changes));
  EXPECT_TRUE(holds("(robot_at r1 kitchen)"));
  EXPECT_TRUE(changes.empty());
}

TEST_F(EffectUndoTest, numeric_effects_use_the_values_before_the_action)
{
  ASSERT_TRUE(client_->addFunction(plansys2::Function("(= (room_distance bedroom kitchen) 0)")));
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of(
        "(and (increase (room_distance kitchen bedroom) 1)"
        " (assign (room_distance bedroom kitchen) (room_distance kitchen bedroom)))"),
      client_, changes));
  EXPECT_DOUBLE_EQ(distance(), 11.0);
  auto other = client_->getFunction("(room_distance bedroom kitchen)");
  ASSERT_TRUE(other.has_value());
  EXPECT_DOUBLE_EQ(other->value, 10.0);  // The value before the action
  ASSERT_TRUE(plansys2::undo(changes, client_));
  EXPECT_DOUBLE_EQ(distance(), 10.0);
  EXPECT_DOUBLE_EQ(client_->getFunction("(room_distance bedroom kitchen)")->value, 0.0);
}

TEST_F(EffectUndoTest, increases_of_the_same_function_add_up)
{
  std::vector<plansys2::FactChange> changes;
  ASSERT_TRUE(
    plansys2::apply(
      tree_of(
        "(and (increase (room_distance kitchen bedroom) 1)"
        " (increase (room_distance kitchen bedroom) 2)"
        " (decrease (room_distance kitchen bedroom) 0.5))"),
      client_, changes));
  EXPECT_DOUBLE_EQ(distance(), 12.5);
  ASSERT_EQ(changes.size(), 1u);  // One change for the function
  ASSERT_TRUE(plansys2::undo(changes, client_));
  EXPECT_DOUBLE_EQ(distance(), 10.0);

  // An assign and an increase: the increase goes on top of what is assigned
  ASSERT_TRUE(
    plansys2::apply(
      tree_of(
        "(and (assign (room_distance kitchen bedroom) 3)"
        " (increase (room_distance kitchen bedroom) 2))"),
      client_));
  EXPECT_DOUBLE_EQ(distance(), 5.0);
}

TEST(EffectSemantics, with_a_local_state)
{
  std::vector<plansys2::Predicate> predicates = {plansys2::Predicate("(robot_at r1 kitchen)")};
  std::vector<plansys2::Function> functions = {
    plansys2::Function("(= (x) 1)"), plansys2::Function("(= (y) 0)")};
  plansys2_msgs::msg::Tree effect;
  parser::pddl::fromString(
    effect,
    "(and (not (robot_at r1 kitchen)) (robot_at r1 kitchen) (robot_at r1 hall)"
    " (not (robot_at r1 hall)) (increase (x) 1) (assign (y) (x)) (increase (x) 1))",
    false, plansys2_msgs::msg::Node::AND);
  ASSERT_TRUE(plansys2::apply(effect, predicates, functions));
  auto has = [&](const std::string & text) {
      return std::find_if(
        predicates.begin(), predicates.end(), [&](const plansys2::Predicate & p) {
          return parser::pddl::toString(p) == text;
        }) != predicates.end();
    };
  EXPECT_TRUE(has("(robot_at r1 kitchen)"));
  EXPECT_TRUE(has("(robot_at r1 hall)"));
  EXPECT_DOUBLE_EQ(functions[0].value, 3.0);
  EXPECT_DOUBLE_EQ(functions[1].value, 1.0);  // x before the action
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
