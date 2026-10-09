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

// Trees with more than 255 nodes and malformed trees must not crash (#417)

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gtest/gtest.h"

#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/Utils.hpp"

using plansys2_msgs::msg::Node;

namespace
{

std::string read_file(const std::string & name)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_problem_expert").string();
  std::ifstream ifs(pkgpath + "/pddl/" + name);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

std::string room(int i) {return "room" + std::to_string(i);}

// (and (is_teleporter_destination room0) ... (is_teleporter_destination roomN-1))
std::string big_goal(int n)
{
  std::string goal = "(and";
  for (int i = 0; i < n; i++) {
    goal += " (is_teleporter_destination " + room(i) + ")";
  }
  return goal + ")";
}

class BigTreesTest : public ::testing::TestWithParam<int>
{
protected:
  void SetUp() override
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
    problem_expert_ = std::make_shared<plansys2::ProblemExpert>(domain_expert_);
    for (int i = 0; i < GetParam(); i++) {
      ASSERT_TRUE(problem_expert_->addInstance(plansys2::Instance(room(i), "room")));
    }
  }

  std::shared_ptr<plansys2::DomainExpert> domain_expert_;
  std::shared_ptr<plansys2::ProblemExpert> problem_expert_;
};

}  // namespace

TEST_P(BigTreesTest, goal_is_set_and_evaluated)
{
  const int n = GetParam();
  plansys2::Goal goal(big_goal(n));
  ASSERT_EQ(goal.nodes.size(), static_cast<size_t>(n + 1));

  ASSERT_TRUE(problem_expert_->setGoal(goal));
  ASSERT_EQ(problem_expert_->getGoal().nodes.size(), static_cast<size_t>(n + 1));
  ASSERT_FALSE(problem_expert_->isGoalSatisfied(goal));

  // Satisfied only once every single predicate holds, the last ones included
  for (int i = 0; i < n; i++) {
    ASSERT_TRUE(
      problem_expert_->addPredicate(
        plansys2::Predicate("(is_teleporter_destination " + room(i) + ")")));
    if (i == n - 2) {
      ASSERT_FALSE(problem_expert_->isGoalSatisfied(goal));
    }
  }
  ASSERT_TRUE(problem_expert_->isGoalSatisfied(goal));

  // The generated goal keeps all of them
  const auto problem = problem_expert_->getProblem();
  const auto goal_text = problem.substr(problem.find(":goal"));
  size_t count = 0;
  for (auto pos = goal_text.find("is_teleporter_destination"); pos != std::string::npos;
    pos = goal_text.find("is_teleporter_destination", pos + 1))
  {
    count++;
  }
  ASSERT_EQ(count, static_cast<size_t>(n));
  ASSERT_NE(goal_text.find(room(n - 1) + " "), std::string::npos);
}

TEST_P(BigTreesTest, goal_through_add_problem)
{
  const int n = GetParam();
  std::string problem = "(define (problem p) (:domain simple) (:objects";
  for (int i = 0; i < n; i++) {
    problem += " " + room(i);
  }
  problem += " - room) (:init (is_teleporter_destination room0)) (:goal " + big_goal(n) + "))";

  ASSERT_TRUE(problem_expert_->addProblem(problem));
  ASSERT_EQ(problem_expert_->getGoal().nodes.size(), static_cast<size_t>(n + 1));
  // Only room0 holds initially
  ASSERT_EQ(problem_expert_->isGoalSatisfied(problem_expert_->getGoal()), n == 1);
}

TEST_P(BigTreesTest, check_and_apply_with_many_nodes)
{
  const int n = GetParam();
  auto tree = parser::pddl::fromString(big_goal(n));
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;

  ASSERT_FALSE(plansys2::check(tree, predicates, functions));
  ASSERT_TRUE(plansys2::apply(tree, predicates, functions));
  ASSERT_EQ(predicates.size(), static_cast<size_t>(n));
  ASSERT_TRUE(plansys2::check(tree, predicates, functions));

  // Negated: removes them all again
  auto negated = parser::pddl::fromString("(not " + big_goal(n) + ")");
  ASSERT_TRUE(plansys2::apply(negated, predicates, functions));
  ASSERT_TRUE(predicates.empty());
}

INSTANTIATE_TEST_SUITE_P(
  problem_expert, BigTreesTest, ::testing::Values(1, 254, 255, 256, 257, 300, 1000));

namespace
{

plansys2::Goal goal_with(std::vector<Node> nodes)
{
  plansys2::Goal goal;
  for (size_t i = 0; i < nodes.size(); i++) {
    nodes[i].node_id = i;
    goal.nodes.push_back(nodes[i]);
  }
  return goal;
}

Node node(uint8_t type, std::vector<uint32_t> children = {})
{
  Node n;
  n.node_type = type;
  n.children = children;
  return n;
}

Node predicate(const std::string & expr)
{
  return parser::pddl::fromStringPredicate(expr);
}

}  // namespace

// Trees coming from services are checked before being walked
TEST(problem_expert_trees, malformed_goals_are_rejected)
{
  auto domain_expert =
    std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(
    problem_expert.addPredicate(plansys2::Predicate("(is_teleporter_destination kitchen)")));
  ASSERT_TRUE(problem_expert.setGoal(plansys2::Goal("(and (is_teleporter_destination kitchen))")));
  const auto goal_before = problem_expert.getGoal();
  const auto goal_before_str = parser::pddl::toString(goal_before);

  const auto pred = predicate("(is_teleporter_destination kitchen)");
  const std::vector<plansys2::Goal> malformed = {
    goal_with({node(Node::AND, {1, 7}), pred}),               // child out of range
    goal_with({node(Node::AND, {1, 256}), pred}),             // id that wrapped in uint8_t
    goal_with({node(Node::AND, {0})}),                        // self loop
    goal_with({node(Node::AND, {1}), node(Node::OR, {0})}),   // cycle
    goal_with({node(Node::AND, {1}), node(Node::NOT)}),       // NOT without child
    goal_with({node(Node::AND, {1, 1}), pred}),               // repeated child
    goal_with({node(Node::UNKNOWN)}),                          // unknown type
  };

  for (size_t i = 0; i < malformed.size(); i++) {
    SCOPED_TRACE("malformed goal " + std::to_string(i));
    ASSERT_FALSE(problem_expert.setGoal(malformed[i]));
    ASSERT_FALSE(problem_expert.isGoalSatisfied(malformed[i]));
    ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), goal_before_str);
  }
  ASSERT_TRUE(problem_expert.isGoalSatisfied(goal_before));
}

TEST(problem_expert_trees, evaluate_rejects_malformed_trees)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;

  auto cycle = goal_with({node(Node::NOT, {1}), node(Node::NOT, {0})});
  ASSERT_FALSE(plansys2::check(cycle, predicates, functions));
  ASSERT_FALSE(plansys2::apply(cycle, predicates, functions));

  auto out_of_range = goal_with({node(Node::NOT, {3})});
  ASSERT_FALSE(plansys2::check(out_of_range, predicates, functions));

  // A start node out of range is rejected too
  auto valid = parser::pddl::fromString("(and (robot_at r2d2 kitchen))");
  ASSERT_FALSE(plansys2::check(valid, predicates, functions, 5));
  ASSERT_TRUE(predicates.empty());
}

TEST(problem_expert_trees, total_cost_modifier_is_a_no_op)
{
  // The parser builds (increase (total-cost) 1) with the value as the only child
  auto tree = goal_with({
    node(Node::AND, {1, 3}),
    node(Node::FUNCTION_MODIFIER, {2}),
    node(Node::NUMBER),
    predicate("(robot_at r2d2 kitchen)")});
  tree.nodes[1].modifier_type = Node::INCREASE;
  tree.nodes[2].value = 1;

  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  ASSERT_TRUE(plansys2::apply(tree, predicates, functions));
  ASSERT_EQ(predicates.size(), 1u);
  ASSERT_TRUE(functions.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
