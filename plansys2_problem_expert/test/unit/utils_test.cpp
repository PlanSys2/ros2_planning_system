// Copyright 2019 Intelligent Robotics Lab
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
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <tuple>
#include <utility>

#include "plansys2_core/PackagePaths.hpp"

#include "gtest/gtest.h"

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"

#include "plansys2_msgs/msg/knowledge.hpp"

using namespace std::chrono_literals;

TEST(utils, evaluate_and)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  std::string expression = "(and (patrolled wp1) (patrolled wp2))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.clear();
  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp2)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_or)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(or (patrolled wp1) (patrolled wp2))", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  predicates.clear();
  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp2)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_not)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(not (patrolled wp1))", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  plansys2_msgs::msg::Tree test_tree2;
  parser::pddl::fromString(
    test_tree2, "(not (= wp1 wp2))");
  test_tree2.nodes[2].node_type = plansys2_msgs::msg::Node::CONSTANT;
  test_tree2.nodes[3].node_type = plansys2_msgs::msg::Node::CONSTANT;

  ASSERT_EQ(
    plansys2::evaluate(test_tree2, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  plansys2_msgs::msg::Tree test_tree3;
  parser::pddl::fromString(
    test_tree3, "(not (= wp1 wp1))");
  test_tree3.nodes[2].node_type = plansys2_msgs::msg::Node::CONSTANT;
  test_tree3.nodes[3].node_type = plansys2_msgs::msg::Node::CONSTANT;

  ASSERT_EQ(
    plansys2::evaluate(test_tree3, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));
}

TEST(utils, evaluate_predicate_use_state)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(patrolled wp1)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true, 0, true),
    std::make_tuple(true, true, 0));

  ASSERT_TRUE(plansys2::apply(test_tree, predicates, functions));
  ASSERT_EQ(predicates.size(), 1);
  ASSERT_EQ(parser::pddl::toString(*predicates.begin()), "(patrolled wp1)");

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true, 0, true),
    std::make_tuple(true, false, 0));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, true, true, 0, true),
    std::make_tuple(true, false, 0));
  ASSERT_TRUE(predicates.empty());
}

TEST(utils, evaluate_predicate_client)
{
  {
    std::vector<plansys2::Predicate> predicates;
    std::vector<plansys2::Function> functions;
    auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
    plansys2::testing::NodeSpinner test_node_spinner(test_node);
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());

    plansys2::testing::ExecutorThread spinner(exe);

    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));

    plansys2_msgs::msg::Tree test_tree;
    parser::pddl::fromString(
    test_tree, "(is_teleporter_destination bedroom)", false,
    plansys2_msgs::msg::Node::AND);

    ASSERT_FALSE(plansys2::check(test_tree, problem_client));
    ASSERT_TRUE(plansys2::apply(test_tree, problem_client));
    ASSERT_TRUE(plansys2::check(test_tree, problem_client));

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, true, false, 0, true),
    std::make_tuple(true, false, 0.0));
    ASSERT_FALSE(plansys2::check(test_tree, problem_client));
  }
  plansys2::drain_ros(2s);
}

TEST(utils, evaluate_function_use_state)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(distance wp1 wp2)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0.0));

  functions.push_back(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 1.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 1.0));
}

TEST(utils, evaluate_expression_ge)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(>= (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 2.9999)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = 4.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));

  functions[0].value = 3.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_gt)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(> (distance wp1 wp2) 3.0)", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = 3.00001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_le)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(<= (vx) -3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) -2.9999)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = -4.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));

  functions[0].value = -3.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_lt)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(< (distance wp1 wp2) -3.0)", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (distance wp1 wp2) -3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = -3.00001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_multiply)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(* (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 9.0));

  functions[0].value = -0.001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -0.003));
}

TEST(utils, evaluate_expression_divide)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(/ (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 1.0));

  functions[0].value = -9.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -3.0));

  // Divide by zero
  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(/ (vx) 0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_expression_add)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(+ (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 6.0));

  functions[0].value = -0.001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 2.999));
}

TEST(utils, evaluate_expression_subtract)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(- (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 2.5)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -0.5));

  functions[0].value = -0.001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -3.001));
}

TEST(utils, evaluate_expression_invalid)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  // Unknown expression type
  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(> (vx) 0)", false, plansys2_msgs::msg::Node::AND);
  test_tree.nodes[0].expression_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_expression_invalid_client)
{
  {
    auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
    plansys2::testing::NodeSpinner test_node_spinner(test_node);
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());

    plansys2::testing::ExecutorThread spinner(exe);

    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("leia", "robot")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("Jack", "person")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("m1", "message")));

    plansys2_msgs::msg::Tree test_tree;
    parser::pddl::fromString(
    test_tree, "(> (room_distance bedroom kitchen) 0)", false,
    plansys2_msgs::msg::Node::AND);

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));

    test_tree.nodes.clear();
    parser::pddl::fromString(
    test_tree, "(> 0 (room_distance bedroom kitchen))", false,
    plansys2_msgs::msg::Node::AND);

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));
  }
  plansys2::drain_ros(2s);
}

TEST(utils, evaluate_function_mod)
{
  std::vector<plansys2_msgs::msg::Node> predicates_msg;
  std::vector<plansys2_msgs::msg::Node> functions_msg;

  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(assign (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  parser::pddl::getPredicates(predicates_msg, test_tree);
  parser::pddl::getFunctions(functions_msg, test_tree);

  auto predicates = plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(
    predicates_msg);
  auto functions = plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
    functions_msg);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 0.0);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 3.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(increase (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 6.0));
  ASSERT_EQ(functions[0].value, 6.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(decrease (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 3.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(scale-up (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 9.0));
  ASSERT_EQ(functions[0].value, 9.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(scale-down (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 3.0);

  // divide by zero
  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(scale-down (vx) 0.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(false, false, 0.0));
  ASSERT_EQ(functions[0].value, 3.0);
}

TEST(utils, evaluate_function_mod_client)
{
  {
    auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
    plansys2::testing::NodeSpinner test_node_spinner(test_node);
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());

    plansys2::testing::ExecutorThread spinner(exe);

    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
    ASSERT_TRUE(
    problem_client->addFunction(
      plansys2::Function(
        "(= (room_distance bedroom kitchen) 1.0)")));

    plansys2_msgs::msg::Tree test_tree;
    parser::pddl::fromString(
    test_tree, "(assign (room_distance bedroom kitchen) 0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, true),
    std::make_tuple(true, false, 0));
    std::optional<plansys2_msgs::msg::Node> func = problem_client->getFunction(
    "(room_distance bedroom kitchen)");
    ASSERT_TRUE(func.has_value());
    ASSERT_EQ(func.value().value, 0.0);

    test_tree.nodes.clear();
    parser::pddl::fromString(
    test_tree, "(increase (room_distance bedroom kitchen) 10.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, true),
    std::make_tuple(true, false, 10.0));
    func = problem_client->getFunction("(room_distance bedroom kitchen)");
    ASSERT_TRUE(func.has_value());
    ASSERT_EQ(func.value().value, 10.0);
  }
  plansys2::drain_ros(2s);
}

// Each modifier, applied through the problem expert, keeps the exact value (it went through
// text with six digits: 1234567.891 became 1.23457e+06, and the update failed silently)
TEST(utils, evaluate_function_mod_client_precision)
{
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
    ASSERT_TRUE(
      problem_client->addFunction(plansys2::Function("(= (room_distance bedroom kitchen) 1.0)")));

    // modifier, operand, expected value
    const std::vector<std::tuple<std::string, double, double>> steps = {
      {"assign", 1234567.891, 1234567.891},
      {"increase", 1e-7, 1234567.891 + 1e-7},
      {"decrease", 2234567.891, 1234567.891 + 1e-7 - 2234567.891},
      {"scale-up", -2.5, (1234567.891 + 1e-7 - 2234567.891) * -2.5},
      {"scale-down", 3.0, (1234567.891 + 1e-7 - 2234567.891) * -2.5 / 3.0},
      {"assign", 1e-7, 1e-7},
    };
    for (const auto & [modifier, operand, value] : steps) {
      plansys2_msgs::msg::Tree tree;
      parser::pddl::fromString(
        tree, "(" + modifier + " (room_distance bedroom kitchen) " +
        parser::pddl::numberToString(operand) + ")", false,
        plansys2_msgs::msg::Node::EXPRESSION);
      ASSERT_EQ(tree.nodes[2].value, operand) << modifier;

      auto result = plansys2::evaluate(tree, problem_client, true);
      ASSERT_TRUE(std::get<0>(result)) << modifier;
      ASSERT_EQ(std::get<2>(result), value) << modifier;

      auto func = problem_client->getFunction("(room_distance bedroom kitchen)");
      ASSERT_TRUE(func.has_value()) << modifier;
      ASSERT_EQ(func->value, value) << modifier;
    }
  }
  plansys2::drain_ros(2s);
}

TEST(utils, evaluate_function_mod_invalid)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  // Unknown function modifier type
  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(assign (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);
  test_tree.nodes[0].node_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_function_mod_invalid_client)
{
  {
    auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
    plansys2::testing::NodeSpinner test_node_spinner(test_node);
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());

    plansys2::testing::ExecutorThread spinner(exe);

    plansys2_msgs::msg::Tree test_tree;
    parser::pddl::fromString(
    test_tree, "(assign (room_distance bedroom kitchen) 0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));

    test_tree.nodes.clear();
    parser::pddl::fromString(
    test_tree, "(assign 0 (room_distance bedroom kitchen))", false,
    plansys2_msgs::msg::Node::EXPRESSION);

    ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));
  }
  plansys2::drain_ros(2s);
}

TEST(utils, evaluate_number)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "3.0", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions),
    std::make_tuple(true, true, 3.0));
}

TEST(utils, evaluate_invalid)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  plansys2_msgs::msg::Tree test_tree;
  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions),
    std::make_tuple(true, true, 0));

  parser::pddl::fromString(test_tree, "(patrolled wp1)", false, plansys2_msgs::msg::Node::AND);
  test_tree.nodes.front().node_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_exists)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  std::string expression = "(exists (?1 ?2) (and (robot_at rob1 ?1)(connected ?1 ?2)))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(robot_at rob1 bedroom)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(connected bedroom kitchen)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_exists_client)
{
  {
    std::vector<plansys2::Predicate> predicates;
    std::vector<plansys2::Function> functions;
    auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
    plansys2::testing::NodeSpinner test_node_spinner(test_node);
  // No domain_node here: this test never calls changeDomain(), and problem_node's
  // domain_expert/domain subscription only needs to be remapped to a name unused by
  // any other test in this binary, so a still-lingering transient_local publisher
  // from an earlier *_client test (a pre-existing test-isolation gap: this whole file
  // reuses fixed lifecycle node names across TEST() cases) can't be mistaken for a
  // genuine runtime domain change here.
    rclcpp::NodeOptions problem_node_options;
    problem_node_options.arguments(
      {"--ros-args", "-r", "domain_expert/domain:=domain_expert/domain_evaluate_exists_client"});
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>(problem_node_options);
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();

    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_exists.pddl"});
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(problem_node->get_node_base_interface());

    plansys2::testing::ExecutorThread spinner(exe);

    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("rob1", "robot")));

    std::string expression = "(exists (?1 ?2) (and (robot_at rob1 ?1)(connected ?1 ?2)))";
    plansys2_msgs::msg::Tree goal;
    parser::pddl::fromString(goal, expression);

    ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, false),
    std::make_tuple(true, false, 0));

    ASSERT_TRUE(
    problem_client->addPredicate(
      parser::pddl::fromStringPredicate("(robot_at rob1 bedroom)")));

    ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, false),
    std::make_tuple(true, false, 0));

    ASSERT_TRUE(
    problem_client->addPredicate(
      parser::pddl::fromStringPredicate("(connected bedroom kitchen)")));

    ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, false),
    std::make_tuple(true, true, 0));
  }
  plansys2::drain_ros(2s);
}

TEST(utils, get_subtrees)
{
  std::vector<uint32_t> empty_expected;

  plansys2_msgs::msg::Tree invalid_goal;
  ASSERT_EQ(parser::pddl::getSubtreeIds(invalid_goal), empty_expected);

  parser::pddl::fromString(invalid_goal, "(or (patrolled wp1) (patrolled wp2))");
  ASSERT_EQ(parser::pddl::getSubtreeIds(invalid_goal), empty_expected);

  std::vector<uint32_t> expected;
  expected.push_back(1);
  expected.push_back(2);

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (patrolled wp1) (patrolled wp2))");
  auto actual = parser::pddl::getSubtreeIds(goal);
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < expected.size(); i++) {
    ASSERT_EQ(actual[i], expected[i]);
  }
}

TEST(utils, get_action_from_string)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  plansys2::testing::NodeSpinner test_node_spinner(test_node);
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);

  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());

  plansys2::testing::ExecutorThread spinner(exe);

  std::string invalid_action_str = "(invalid r2d2 kitchen bedroom)";
  ASSERT_EQ(
    domain_client->getAction(
      plansys2::get_action_name(invalid_action_str),
      plansys2::get_action_params(invalid_action_str)), nullptr);

  std::string action_str = "(teleport r2d2 kitchen bedroom)";

  std::shared_ptr<plansys2_msgs::msg::Action> expected =
    std::make_shared<plansys2_msgs::msg::Action>();
  expected->name = "teleport";

  expected->parameters.push_back(parser::pddl::fromStringParam("r2d2"));
  expected->parameters.push_back(parser::pddl::fromStringParam("kitchen"));
  expected->parameters.push_back(parser::pddl::fromStringParam("bedroom"));

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree,
    "(and "
    "(robot_at r2d2 kitchen)(is_teleporter_enabled kitchen)(is_teleporter_destination bedroom))");
  expected->preconditions = test_tree;

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree,
    "(and (not(robot_at r2d2 kitchen))(robot_at r2d2 bedroom))");
  expected->effects = test_tree;
  std::shared_ptr<plansys2_msgs::msg::Action> actual =
    domain_client->getAction(
    plansys2::get_action_name(action_str),
    plansys2::get_action_params(action_str));

  ASSERT_EQ(parser::pddl::nameActionsToString(actual), parser::pddl::nameActionsToString(expected));
  ASSERT_EQ(
    parser::pddl::toString(actual->preconditions),
    parser::pddl::toString(expected->preconditions));
  ASSERT_EQ(
    parser::pddl::toString(actual->effects),
    parser::pddl::toString(expected->effects));

  std::string durative_action_str = "(move r2d2 kitchen bedroom)";

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_expected =
    std::make_shared<plansys2_msgs::msg::DurativeAction>();

  durative_expected->name = "move";
  durative_expected->parameters.push_back(parser::pddl::fromStringParam("r2d2"));
  durative_expected->parameters.push_back(parser::pddl::fromStringParam("kitchen"));
  durative_expected->parameters.push_back(parser::pddl::fromStringParam("bedroom"));

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (robot_at r2d2 kitchen))");
  durative_expected->at_start_requirements = test_tree;

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (not(robot_at r2d2 kitchen)))");
  durative_expected->at_start_effects = test_tree;

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (robot_at r2d2 bedroom))");
  durative_expected->at_end_effects = test_tree;

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_actual =
    domain_client->getDurativeAction(
    plansys2::get_action_name(durative_action_str),
    plansys2::get_action_params(durative_action_str));

  ASSERT_EQ(
    parser::pddl::nameActionsToString(durative_actual),
    parser::pddl::nameActionsToString(durative_expected));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_start_requirements),
    parser::pddl::toString(durative_expected->at_start_requirements));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->over_all_requirements),
    parser::pddl::toString(durative_expected->over_all_requirements));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_end_requirements),
    parser::pddl::toString(durative_expected->at_end_requirements));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_start_effects),
    parser::pddl::toString(durative_expected->at_start_effects));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_end_effects),
    parser::pddl::toString(durative_expected->at_end_effects));

  std::string overall_action_str = "(approach leia kitchen Jack)";

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> overall_expected =
    std::make_shared<plansys2_msgs::msg::DurativeAction>();

  overall_expected->name = "approach";

  overall_expected->parameters.push_back(parser::pddl::fromStringParam("leia"));
  overall_expected->parameters.push_back(parser::pddl::fromStringParam("kitchen"));
  overall_expected->parameters.push_back(parser::pddl::fromStringParam("Jack"));

  parser::pddl::fromString(
    overall_expected->over_all_requirements,
    "(and (robot_at leia kitchen) (person_at Jack kitchen))");
  parser::pddl::fromString(
    overall_expected->at_end_requirements,
    "(and (person_at Jack kitchen))");
  parser::pddl::fromString(
    overall_expected->at_end_effects,
    "(and (robot_near_person leia Jack))");

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> overall_actual =
    domain_client->getDurativeAction(
    plansys2::get_action_name(overall_action_str),
    plansys2::get_action_params(overall_action_str));

  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_start_requirements),
    parser::pddl::toString(overall_expected->at_start_requirements));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->over_all_requirements),
    parser::pddl::toString(overall_expected->over_all_requirements));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_end_requirements),
    parser::pddl::toString(overall_expected->at_end_requirements));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_start_effects),
    parser::pddl::toString(overall_expected->at_start_effects));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_end_effects),
    parser::pddl::toString(overall_expected->at_end_effects));
}

TEST(utils, get_params)
{
  std::string action_str = "(move r2d2 bedroom)";

  std::vector<std::string> expected;
  expected.push_back("r2d2");
  expected.push_back("bedroom");

  ASSERT_EQ(plansys2::get_action_params(action_str), expected);
}

TEST(utils, get_name)
{
  std::string action_str = "(move r2d2 bedroom)";

  ASSERT_EQ(plansys2::get_action_name(action_str), "move");
}

TEST(utils, replace_children_param) {
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::string domain_file = pkgpath + "/pddl/domain_exists.pddl";

  std::ifstream domain_ifs(domain_file);
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());
  parser::pddl::Domain domain(domain_str);

  auto action = domain.actions.get("action_test");
  plansys2_msgs::msg::Tree tree;
  action->pre->getTree(tree, domain);
  std::map<std::string, std::string> replace;
  replace["?1"] = "bedroom";
  replace["?2"] = "bathroom";
  plansys2_msgs::msg::Tree tree2 = plansys2::replace_children_param(
    tree,
    1,
    replace
  );
  std::string str = parser::pddl::toString(tree2);
  ASSERT_EQ(
    str,
    "(and (exists (bedroom) (and (robot_at ?0 bedroom)(charging_point_at bedroom)))"
    "(and (> (battery_level ?0) 1.000000)(< (battery_level ?0) 200.000000)))");

  auto action2 = domain.actions.get("action_test2");
  plansys2_msgs::msg::Tree tree3;
  action2->pre->getTree(tree3, domain);
  plansys2_msgs::msg::Tree tree4 = plansys2::replace_children_param(
    tree3,
    0,
    replace
  );
  std::string str2 = parser::pddl::toString(tree4);
  ASSERT_EQ(
    str2,
    "(exists (bedroom bathroom) (and (robot_at ?0 bedroom)(connected bedroom bathroom)))");
}

// Truth value of an expression against a state: (success, value)
namespace
{
std::pair<bool, bool> truth(
  const std::string & expression,
  std::vector<plansys2::Predicate> predicates,
  std::vector<plansys2::Function> functions = {})
{
  plansys2_msgs::msg::Tree tree;
  parser::pddl::fromString(tree, expression);
  auto result = plansys2::evaluate(tree, predicates, functions);
  return {std::get<0>(result), std::get<1>(result)};
}

std::vector<plansys2::Predicate> state(const std::vector<std::string> & preds)
{
  std::vector<plansys2::Predicate> ret;
  for (const auto & p : preds) {
    ret.push_back(parser::pddl::fromStringPredicate(p));
  }
  return ret;
}
}  // namespace

TEST(utils, evaluate_not_and_or_de_morgan)
{
  const auto s = state({"(a x)", "(b y)"});
  const std::vector<std::pair<std::string, bool>> cases = {
    // and / or without not, for reference
    {"(and (a x) (b y))", true},
    {"(and (a x) (c x))", false},
    {"(or (a x) (c x))", true},
    {"(or (c x) (d x))", false},
    // not over and: true if any part is false
    {"(not (and (a x) (c x)))", true},
    {"(not (and (c x) (d x)))", true},
    {"(not (and (a x) (b y)))", false},
    // not over or: true only if every part is false
    {"(not (or (a x) (c x)))", false},
    {"(not (or (c x) (a x)))", false},
    {"(not (or (c x) (d x)))", true},
    // double negation
    {"(not (not (and (a x) (b y))))", true},
    {"(not (not (and (a x) (c x))))", false},
    {"(not (not (or (c x) (a x))))", true},
    // nested
    {"(not (and (a x) (or (c x) (b y))))", false},
    {"(not (and (a x) (or (c x) (d y))))", true},
    {"(not (or (c x) (and (a x) (b y))))", false},
    {"(not (or (c x) (and (a x) (c y))))", true},
    {"(and (a x) (not (c x)) (not (and (b y) (c y))))", true},
    {"(or (c x) (not (or (a x) (b y))))", false},
    // a single child
    {"(not (and (a x)))", false},
    {"(not (or (c x)))", true},
  };
  for (const auto & [expression, expected] : cases) {
    auto [success, value] = truth(expression, s);
    EXPECT_TRUE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }
}

TEST(utils, evaluate_not_exists)
{
  const auto s = state({"(a x)", "(b y)", "(link x y)"});
  const std::vector<std::pair<std::string, bool>> cases = {
    {"(exists (?1) (a ?1))", true},
    {"(exists (?1) (c ?1))", false},
    // not exists: true only if no object satisfies the body
    {"(not (exists (?1) (a ?1)))", false},
    {"(not (exists (?1) (b ?1)))", false},
    {"(not (exists (?1) (c ?1)))", true},
    {"(not (exists (?1) (and (a ?1) (b ?1))))", true},
    {"(not (exists (?1 ?2) (and (a ?1) (b ?2))))", false},
    {"(not (exists (?1 ?2) (and (a ?1) (link ?2 ?1))))", true},
    {"(not (exists (?1 ?2) (link ?1 ?2)))", false},
    // a negated body
    {"(exists (?1) (not (a ?1)))", true},
    {"(not (exists (?1) (not (link x ?1))))", false},
    // not exists inside and / or, as in action preconditions
    {"(and (a x) (not (exists (?1) (c ?1))))", true},
    {"(and (a x) (not (exists (?1) (b ?1))))", false},
    {"(or (c x) (not (exists (?1) (c ?1))))", true},
    // double negation
    {"(not (not (exists (?1) (a ?1))))", true},
    {"(not (not (exists (?1) (c ?1))))", false},
  };
  for (const auto & [expression, expected] : cases) {
    auto [success, value] = truth(expression, s);
    EXPECT_TRUE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }

  // With no objects at all, nothing exists
  EXPECT_EQ(truth("(exists (?1) (a ?1))", {}).second, false);
  EXPECT_EQ(truth("(not (exists (?1) (a ?1)))", {}).second, true);
}

// #394: (= (f) (g)) was never true
TEST(utils, evaluate_numeric_equality)
{
  std::vector<plansys2::Function> functions = {
    parser::pddl::fromStringFunction("(= (photos) 3)"),
    parser::pddl::fromStringFunction("(= (markers) 3)"),
    parser::pddl::fromStringFunction("(= (seen) 2)"),
    parser::pddl::fromStringFunction("(= (dist x y) 1.5)"),
    parser::pddl::fromStringFunction("(= (tenth) 0.1)"),
    parser::pddl::fromStringFunction("(= (big) 1000000000000)"),
  };
  const std::vector<std::pair<std::string, bool>> cases = {
    {"(= (photos) (markers))", true},
    {"(= (photos) (seen))", false},
    {"(= (photos) 3)", true},
    {"(= 3 (photos))", true},
    {"(= (photos) 4)", false},
    {"(= (photos) (+ (seen) 1))", true},
    {"(= (* (dist x y) 2) (markers))", true},
    {"(= (dist x y) 1.5)", true},
    {"(= 2 2)", true},
    {"(= 2 3)", false},
    // rounding: 0.1 + 0.1 + 0.1 is not exactly 0.3
    {"(= (+ (+ (tenth) (tenth)) (tenth)) 0.3)", true},
    {"(= (big) 1000000000000.0001)", true},
    {"(= (big) 1000000010000)", false},
    // negated
    {"(not (= (photos) (markers)))", false},
    {"(not (= (photos) (seen)))", true},
    {"(and (= (photos) (markers)) (not (= (seen) (markers))))", true},
  };
  for (const auto & [expression, expected] : cases) {
    auto [success, value] = truth(expression, {}, functions);
    EXPECT_TRUE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }

  // A function without a value cannot be compared
  EXPECT_FALSE(truth("(= (photos) (missing))", {}, functions).first);
}

// Comparisons under not: only the comparison is negated, not the numbers
TEST(utils, evaluate_not_comparisons)
{
  std::vector<plansys2::Function> functions = {
    parser::pddl::fromStringFunction("(= (battery) 30)"),
  };
  const std::vector<std::pair<std::string, bool>> cases = {
    {"(not (> (battery) 20))", false},
    {"(not (< (battery) 20))", true},
    {"(not (>= (battery) 30))", false},
    {"(not (<= (battery) 29))", true},
    {"(not (> (- (battery) 15) 10))", false},
    {"(not (and (> (battery) 20) (< (battery) 40)))", false},
    {"(not (or (> (battery) 40) (< (battery) 20)))", true},
  };
  for (const auto & [expression, expected] : cases) {
    auto [success, value] = truth(expression, {}, functions);
    EXPECT_TRUE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }
}

// forall, typed exists and imply. Types as in domain_quantifiers.pddl:
// robot > drone, ground > tank, infantry; and zone
namespace
{
const std::map<std::string, std::vector<std::string>> kSubtypes = {
  {"robot", {"drone", "ground", "tank", "infantry"}},
  {"ground", {"tank", "infantry"}},
};
const std::map<std::string, std::string> kObjects = {
  {"d1", "drone"}, {"t1", "tank"}, {"i1", "infantry"}, {"e1", "tank"},
  {"z1", "zone"}, {"z2", "zone"},
};

// A tree whose quantified variables carry the subtypes of their type, as from the domain
plansys2_msgs::msg::Tree typed_tree(const std::string & expression)
{
  auto tree = parser::pddl::fromString(expression);
  for (auto & node : tree.nodes) {
    for (auto & param : node.parameters) {
      auto it = kSubtypes.find(param.type);
      if (it != kSubtypes.end()) {param.sub_types = it->second;}
    }
  }
  return tree;
}

// A state whose parameters have the types of their objects, as the problem expert keeps them
std::vector<plansys2::Predicate> typed_state(const std::vector<std::string> & preds)
{
  std::vector<plansys2::Predicate> ret;
  for (const auto & p : preds) {
    auto pred = parser::pddl::fromStringPredicate(p);
    for (auto & param : pred.parameters) {
      auto it = kObjects.find(param.name);
      if (it != kObjects.end()) {param.type = it->second;}
    }
    ret.push_back(pred);
  }
  return ret;
}

std::pair<bool, bool> typed_truth(
  const std::string & expression, std::vector<plansys2::Predicate> predicates)
{
  std::vector<plansys2::Function> functions;
  auto result = plansys2::evaluate(typed_tree(expression), predicates, functions);
  return {std::get<0>(result), std::get<1>(result)};
}
}  // namespace

TEST(utils, evaluate_forall_exists_typed)
{
  const auto s = typed_state(
    {"(at i1 z1)", "(at t1 z1)", "(at e1 z2)", "(hostile e1)",
      "(available d1)", "(available t1)", "(available i1)", "(available e1)", "(owned z1)"});

  const std::vector<std::pair<std::string, bool>> cases = {
    // forall ranges over the objects of the type and its subtypes, not over the zones
    {"(forall (?r - robot) (available ?r))", true},
    {"(forall (?g - ground) (at ?g z1))", false},
    {"(forall (?i - infantry) (at ?i z1))", true},
    {"(forall (?t - tank) (available ?t))", true},
    {"(not (forall (?g - ground) (at ?g z1)))", true},
    {"(not (forall (?r - robot) (available ?r)))", false},
    // without a type, zones are objects too
    {"(forall (?o) (available ?o))", false},
    // exists with a type
    {"(exists (?d - drone) (at ?d z1))", false},
    {"(exists (?g - ground) (at ?g z2))", true},
    {"(exists (?z - zone) (owned ?z))", true},
    {"(exists (?r - robot) (owned ?r))", false},
    // combined
    {"(forall (?z - zone) (exists (?r - robot) (at ?r ?z)))", true},
    {"(exists (?z - zone) (forall (?g - ground) (at ?g ?z)))", false},
    {"(forall (?z - zone) (imply (owned ?z) (exists (?i - infantry) (at ?i ?z))))", true},
    // "no hostile robot in the zone", as capture says it
    {"(not (exists (?e - robot) (and (hostile ?e) (at ?e z1))))", true},
    {"(not (exists (?e - robot) (and (hostile ?e) (at ?e z2))))", false},
    {"(forall (?e - robot) (imply (hostile ?e) (not (at ?e z1))))", true},
    {"(forall (?e - robot) (imply (hostile ?e) (not (at ?e z2))))", false},
    {"(not (exists (?r - robot) (not (available ?r))))", true},
    // a type without objects: everything holds for none, and none exists
    {"(forall (?a - artillery) (at ?a z1))", true},
    {"(exists (?a - artillery) (at ?a z1))", false},
    {"(not (forall (?a - artillery) (at ?a z1)))", false},
    // two variables
    {"(exists (?g - ground ?z - zone) (and (hostile ?g) (at ?g ?z)))", true},
    {"(forall (?g - ground ?z - zone) (imply (at ?g ?z) (available ?g)))", true},
  };
  for (const auto & [expression, expected] : cases) {
    auto [success, value] = typed_truth(expression, s);
    EXPECT_TRUE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }
}

TEST(utils, evaluate_imply)
{
  const auto s = state({"(a x)", "(b y)"});
  const std::vector<std::pair<std::string, bool>> cases = {
    {"(imply (a x) (b y))", true},    // true -> true
    {"(imply (a x) (c x))", false},   // true -> false
    {"(imply (c x) (b y))", true},    // false -> true
    {"(imply (c x) (d x))", true},    // false -> false
    {"(not (imply (a x) (b y)))", false},
    {"(not (imply (a x) (c x)))", true},
    {"(not (imply (c x) (d x)))", false},
    {"(imply (and (a x) (b y)) (or (c x) (b y)))", true},
    {"(imply (not (c x)) (not (a x)))", false},
    {"(and (a x) (imply (b y) (a x)))", true},
  };
  for (const auto & [expression, expected] : cases) {
    auto [success, value] = truth(expression, s);
    EXPECT_TRUE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }
}

// A forall in an effect applies its body to every object of the type, and only to them
TEST(utils, apply_forall_effect)
{
  auto predicates = typed_state(
    {"(available d1)", "(available t1)", "(available i1)", "(owned z1)", "(owned z2)"});
  std::vector<plansys2::Function> functions;

  auto effect = typed_tree("(and (forall (?g - ground) (not (available ?g))))");
  auto result = plansys2::evaluate(effect, predicates, functions, true);
  ASSERT_TRUE(std::get<0>(result));

  auto has = [&predicates](const std::string & p) {
      auto pred = parser::pddl::fromStringPredicate(p);
      return std::any_of(
        predicates.begin(), predicates.end(),
        [&pred](const auto & q) {return parser::pddl::checkNodeEquality(q, pred);});
    };
  ASSERT_TRUE(has("(available d1)"));  // a drone is not ground
  ASSERT_FALSE(has("(available t1)"));
  ASSERT_FALSE(has("(available i1)"));
  ASSERT_TRUE(has("(owned z1)"));
  ASSERT_TRUE(has("(owned z2)"));
  ASSERT_EQ(predicates.size(), 3u);
}

TEST(utils, evaluate_forall_client)
{
  {
    std::vector<plansys2::Predicate> predicates;
    std::vector<plansys2::Function> functions;
    rclcpp::NodeOptions problem_node_options;
    problem_node_options.arguments(
      {"--ros-args", "-r", "domain_expert/domain:=domain_expert/domain_evaluate_forall_client"});
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>(problem_node_options);
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_quantifiers.pddl"});
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(problem_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    for (const auto & [name, type] : kObjects) {
      ASSERT_TRUE(problem_client->addInstance(plansys2::Instance(name, type))) << name;
    }
    for (const auto & p : {"(at i1 z1)", "(at e1 z2)", "(hostile e1)", "(available i1)"}) {
      ASSERT_TRUE(problem_client->addPredicate(parser::pddl::fromStringPredicate(p))) << p;
    }

    auto client_truth = [&](const std::string & expression) {
        auto r = plansys2::evaluate(
          typed_tree(expression), problem_client, predicates, functions, false, false);
        EXPECT_TRUE(std::get<0>(r)) << expression;
        return std::get<1>(r);
      };

    // The instances of the problem expert, with their types
    EXPECT_TRUE(client_truth("(not (exists (?e - robot) (and (hostile ?e) (at ?e z1))))"));
    EXPECT_FALSE(client_truth("(not (exists (?e - robot) (and (hostile ?e) (at ?e z2))))"));
    EXPECT_TRUE(client_truth("(forall (?i - infantry) (available ?i))"));
    EXPECT_FALSE(client_truth("(forall (?g - ground) (available ?g))"));  // t1 and e1 are not
    EXPECT_TRUE(client_truth("(forall (?e - robot) (imply (hostile ?e) (at ?e z2)))"));
    // d1 is in no predicate, but the problem expert knows it
    EXPECT_FALSE(client_truth("(forall (?r - robot) (exists (?z - zone) (at ?r ?z)))"));

    // The state changes: what the conditions say follows it
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::fromStringPredicate("(at e1 z1)")));
    EXPECT_FALSE(client_truth("(not (exists (?e - robot) (and (hostile ?e) (at ?e z1))))"));
    ASSERT_TRUE(problem_client->removeInstance(plansys2::Instance("e1", "tank")));
    EXPECT_TRUE(client_truth("(not (exists (?e - robot) (and (hostile ?e) (at ?e z1))))"));
  }
  plansys2::drain_ros(2s);
}

namespace
{
bool holds(const std::vector<plansys2::Predicate> & predicates, const std::string & p)
{
  auto pred = parser::pddl::fromStringPredicate(p);
  return std::any_of(
    predicates.begin(), predicates.end(),
    [&pred](const auto & q) {return parser::pddl::checkNodeEquality(q, pred);});
}

std::vector<plansys2::Predicate> facts(const std::vector<std::string> & preds)
{
  std::vector<plansys2::Predicate> ret;
  for (const auto & p : preds) {
    ret.push_back(parser::pddl::fromStringPredicate(p));
  }
  return ret;
}

bool apply_text(
  const std::string & effect, std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions)
{
  return plansys2::apply(parser::pddl::fromString(effect), predicates, functions);
}
}  // namespace

TEST(utils, apply_when)
{
  std::vector<plansys2::Function> functions;

  // The effect, only if the condition holds
  auto predicates = facts({"(dirty kitchen)"});
  ASSERT_TRUE(apply_text("(and (when (dirty kitchen) (clean kitchen)))", predicates, functions));
  EXPECT_TRUE(holds(predicates, "(clean kitchen)"));
  predicates = facts({});
  ASSERT_TRUE(apply_text("(and (when (dirty kitchen) (clean kitchen)))", predicates, functions));
  EXPECT_TRUE(predicates.empty());

  // Negated conditions and effects, and several of them
  predicates = facts({"(dirty kitchen)"});
  ASSERT_TRUE(
    apply_text(
      "(and (when (dirty kitchen) (and (clean kitchen) (not (dirty kitchen))))"
      " (when (not (dirty hall)) (clean hall)) (when (dirty hall) (lit hall)))",
      predicates, functions));
  EXPECT_TRUE(holds(predicates, "(clean kitchen)"));
  EXPECT_FALSE(holds(predicates, "(dirty kitchen)"));
  EXPECT_TRUE(holds(predicates, "(clean hall)"));
  EXPECT_FALSE(holds(predicates, "(lit hall)"));

  // As a condition it asks for nothing
  predicates = facts({});
  EXPECT_TRUE(
    plansys2::check(parser::pddl::fromString("(when (dirty kitchen) (clean kitchen))"),
    predicates, functions));
}

// All conditions are read before the action changes anything, as in PDDL
TEST(utils, apply_when_reads_the_state_before)
{
  std::vector<plansys2::Function> functions;

  // The first effect deletes what the second one reads, in either order
  for (const auto & effect : {
    "(and (when (a) (not (a))) (when (a) (b)))",
    "(and (when (a) (b)) (when (a) (not (a))))",
    "(and (not (a)) (when (a) (b)))"})
  {
    auto predicates = facts({"(a)"});
    ASSERT_TRUE(apply_text(effect, predicates, functions)) << effect;
    EXPECT_TRUE(holds(predicates, "(b)")) << effect;
    EXPECT_FALSE(holds(predicates, "(a)")) << effect;
  }

  // An effect that makes a condition true does not trigger it
  for (const auto & effect : {"(and (a) (when (a) (b)))", "(and (when (a) (b)) (a))"}) {
    auto predicates = facts({});
    ASSERT_TRUE(apply_text(effect, predicates, functions)) << effect;
    EXPECT_TRUE(holds(predicates, "(a)")) << effect;
    EXPECT_FALSE(holds(predicates, "(b)")) << effect;
  }

  // A condition that a delete makes true does not trigger either
  auto predicates = facts({"(a)"});
  ASSERT_TRUE(apply_text("(and (not (a)) (when (not (a)) (b)))", predicates, functions));
  EXPECT_FALSE(holds(predicates, "(b)"));
}

TEST(utils, apply_when_numeric)
{
  auto battery = [](double value) {
      auto f = parser::pddl::fromStringFunction("(= (battery r1) 0)");
      f.value = value;
      return std::vector<plansys2::Function>{f};
    };
  const std::string effect =
    "(and (when (>= (battery r1) 10) (and (decrease (battery r1) 10) (moved r1))))";
  std::vector<plansys2::Predicate> predicates;

  auto functions = battery(15);
  ASSERT_TRUE(apply_text(effect, predicates, functions));
  EXPECT_DOUBLE_EQ(functions[0].value, 5);
  EXPECT_TRUE(holds(predicates, "(moved r1)"));

  // Its own change does not count: it is read once, before
  predicates.clear();
  functions = battery(5);
  ASSERT_TRUE(apply_text(effect, predicates, functions));
  EXPECT_DOUBLE_EQ(functions[0].value, 5);
  EXPECT_TRUE(predicates.empty());

  // Two that read the same value, one decrease each: both, from the value before
  predicates.clear();
  functions = battery(10);
  ASSERT_TRUE(
    apply_text(
      "(and (when (>= (battery r1) 10) (decrease (battery r1) 4))"
      " (when (>= (battery r1) 10) (decrease (battery r1) 3)))",
      predicates, functions));
  EXPECT_DOUBLE_EQ(functions[0].value, 3);
}

TEST(utils, apply_forall_when)
{
  // Each room, with its own condition
  auto predicates = typed_state({"(at i1 z1)", "(hostile e1)", "(available i1)"});
  std::vector<plansys2::Function> functions;
  auto effect = typed_tree(
    "(and (forall (?g - ground) (when (available ?g) (and (deployed ?g) (not (available ?g))))))");
  ASSERT_TRUE(std::get<0>(plansys2::evaluate(effect, predicates, functions, true)));
  EXPECT_TRUE(holds(predicates, "(deployed i1)"));
  EXPECT_FALSE(holds(predicates, "(available i1)"));
  EXPECT_FALSE(holds(predicates, "(deployed e1)"));
  EXPECT_TRUE(holds(predicates, "(hostile e1)"));
}

// Through the problem expert: the changes are recorded, and undone exactly
TEST(utils, apply_when_client_and_undo)
{
  {
    rclcpp::NodeOptions problem_node_options;
    problem_node_options.arguments(
      {"--ros-args", "-r", "domain_expert/domain:=domain_expert/domain_apply_when_client"});
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>(problem_node_options);
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
    const std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_when.pddl"});
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(problem_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r1", "robot")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
    ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("hall", "room")));
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::fromStringPredicate("(dirty kitchen)")));
    ASSERT_TRUE(problem_client->addFunction(parser::pddl::fromStringFunction("(= (cleaned) 0)")));
    ASSERT_TRUE(
      problem_client->addFunction(parser::pddl::fromStringFunction("(= (battery r1) 10)")));

    auto sweep = [](const std::string & room) {
        return parser::pddl::fromString(
          "(and (when (dirty " + room + ") (and (clean " + room + ") (not (dirty " + room +
          ")) (increase (cleaned) 1))) (decrease (battery r1) 1))");
      };
    auto exists = [&](const std::string & p) {
        return problem_client->existPredicate(parser::pddl::fromStringPredicate(p));
      };
    auto value = [&](const std::string & f) {return problem_client->getFunction(f)->value;};

    std::vector<plansys2::FactChange> changes;
    ASSERT_TRUE(plansys2::apply(sweep("kitchen"), problem_client, changes));
    EXPECT_TRUE(exists("(clean kitchen)"));
    EXPECT_FALSE(exists("(dirty kitchen)"));
    EXPECT_DOUBLE_EQ(value("(cleaned)"), 1);
    EXPECT_DOUBLE_EQ(value("(battery r1)"), 9);
    EXPECT_EQ(changes.size(), 4u);

    // The hall is not dirty: only the battery
    std::vector<plansys2::FactChange> hall_changes;
    ASSERT_TRUE(plansys2::apply(sweep("hall"), problem_client, hall_changes));
    EXPECT_FALSE(exists("(clean hall)"));
    EXPECT_DOUBLE_EQ(value("(cleaned)"), 1);
    EXPECT_DOUBLE_EQ(value("(battery r1)"), 8);
    EXPECT_EQ(hall_changes.size(), 1u);

    ASSERT_TRUE(plansys2::undo(hall_changes, problem_client));
    ASSERT_TRUE(plansys2::undo(changes, problem_client));
    EXPECT_TRUE(exists("(dirty kitchen)"));
    EXPECT_FALSE(exists("(clean kitchen)"));
    EXPECT_DOUBLE_EQ(value("(cleaned)"), 0);
    EXPECT_DOUBLE_EQ(value("(battery r1)"), 10);
  }
  plansys2::drain_ros(2s);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Before the process exits: Zenoh aborts if ROS is shut down by atexit
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
