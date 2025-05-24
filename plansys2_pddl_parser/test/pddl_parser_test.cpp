// Copyright 2022 Marco Roveri - University of Trento
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
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Instance.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PDDLParserTestCase : public ::testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
};

TEST(PDDLParserTestCase, pddl_parser)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_pddl_parser");
  std::string domain_file = pkgpath + "/pddl/dom1.pddl";
  std::string instance_file = pkgpath + "/pddl/prob1.pddl";

  std::ifstream domain_ifs(domain_file);
  ASSERT_TRUE(domain_ifs.good());
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());
  ASSERT_NE(domain_str, "");
  std::ifstream instance_ifs(instance_file);
  ASSERT_TRUE(instance_ifs.good());
  std::string instance_str(
    (std::istreambuf_iterator<char>(instance_ifs)), std::istreambuf_iterator<char>());

  ASSERT_NE(instance_str, "");
  // Read domain and instance
  bool okparse = false;
  bool okprint = false;
  try {
    parser::pddl::Domain domain(domain_str);
    parser::pddl::Instance instance(domain, instance_str);
    okparse = true;
    try {
      std::cout << domain << std::endl;
      std::cout << instance << std::endl;
      okprint = true;
    } catch (std::runtime_error e) {
      std::cerr << e.what() << std::endl;
    }
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  ASSERT_TRUE(okparse);
  ASSERT_TRUE(okprint);
}

TEST(PDDLParserTestCase, exists_get_tree)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_pddl_parser");
  std::string domain_file = pkgpath + "/pddl/dom1.pddl";

  std::ifstream domain_ifs(domain_file);
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());
  parser::pddl::Domain domain(domain_str);

  auto action = domain.actions.get("action_test4");
  plansys2_msgs::msg::Tree tree;
  action->pre->getTree(tree, domain);
  std::string str = parser::pddl::toString(tree);

  ASSERT_EQ(
    str,
    "(and (exists (?1) (and (robot_at ?0 ?1)(charging_point_at ?1)))(and (>  (battery_level ?0) "
    "1.000000)(<  (battery_level ?0) 200.000000)))");

  plansys2_msgs::msg::Tree tree2;
  std::vector<std::string> replace = {"rob1"};
  action->pre->getTree(tree2, domain, replace);
  std::string str2 = parser::pddl::toString(tree2);
  ASSERT_EQ(
    str2,
    "(and (exists (?1) (and (robot_at rob1 ?1)(charging_point_at ?1)))(and (>  (battery_level "
    "rob1) 1.000000)(<  (battery_level rob1) 200.000000)))");

  auto action2 = domain.actions.get("action_test5");
  plansys2_msgs::msg::Tree tree3;
  action2->pre->getTree(tree3, domain);
  std::string str3 = parser::pddl::toString(tree3);
  ASSERT_EQ(str3, "(exists (?1 ?2) (and (robot_at ?0 ?1)(connected ?1 ?2)))");
}

TEST(PDDLParserTestCase, check_node_equality)
{
  auto predicate1 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate2 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate3 = parser::pddl::fromStringPredicate("(predicate ?x b)");
  auto predicate4 = parser::pddl::fromStringPredicate("(predicate a ?y)");
  auto predicate5 = parser::pddl::fromStringPredicate("(predicate a c)");
  auto predicate6 = parser::pddl::fromStringPredicate("(predicate ?x ?y)");

  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate2));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate3));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate4));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate5));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate6));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate6, predicate1));
}

TEST(PDDLParserTestCase, from_string_exists)
{
  plansys2_msgs::msg::Node exist_node;
  exist_node.node_type = plansys2_msgs::msg::Node::EXISTS;
  exist_node.parameters.push_back(parser::pddl::fromStringParam("?y"));
  exist_node.node_id = 0;
  exist_node.children.push_back(1);

  plansys2_msgs::msg::Node and_node;
  and_node.node_type = plansys2_msgs::msg::Node::AND;
  and_node.node_id = 1;
  and_node.children.push_back(2);

  plansys2_msgs::msg::Node predicate_node;
  predicate_node.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_node.name = "inferred-RequiresF";
  predicate_node.parameters.push_back(parser::pddl::fromStringParam("?x"));
  predicate_node.parameters.push_back(parser::pddl::fromStringParam("?y"));
  predicate_node.node_id = 2;

  plansys2_msgs::msg::Tree tree;
  tree.nodes.push_back(exist_node);
  tree.nodes.push_back(and_node);
  tree.nodes.push_back(predicate_node);

  auto from_exists = parser::pddl::fromString("(exists (?y) (and (inferred-RequiresF ?x ?y)))");
  ASSERT_EQ(tree, from_exists);
}
TEST(PDDLParserTestCase, from_string_hyphen)
{
  auto predicate_hyphen = parser::pddl::fromStringPredicate("(predicate-hyphen ?x ?y)");
  auto expression_sub = parser::pddl::fromString("( - 3 4)");

  ASSERT_EQ(predicate_hyphen.node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(predicate_hyphen.name, "predicate-hyphen");
  ASSERT_EQ(predicate_hyphen.parameters.size(), 2);
  ASSERT_EQ(predicate_hyphen.parameters[0].name, "?x");
  ASSERT_EQ(predicate_hyphen.parameters[1].name, "?y");
  
  ASSERT_EQ(expression_sub.nodes[0].node_type, plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(expression_sub.nodes[0].expression_type, plansys2_msgs::msg::Node::ARITH_SUB);
  ASSERT_EQ(expression_sub.nodes[0].children.size(), 2);
  ASSERT_EQ(expression_sub.nodes[1].node_type, plansys2_msgs::msg::Node::NUMBER);
  ASSERT_EQ(expression_sub.nodes[1].value, 3.0);
  ASSERT_EQ(expression_sub.nodes[2].node_type, plansys2_msgs::msg::Node::NUMBER);
  ASSERT_EQ(expression_sub.nodes[2].value, 4.0);
}

TEST(PDDLParserTestCase, test_remove_operators_before_parenthesis)
{
  std::string expr = "and (predicateA ?a)(predicateB ?b)";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "(predicateA ?a)(predicateB ?b)");

  expr = "or (predicateA ?a) (predicateB ?b)";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "(predicateA ?a) (predicateB ?b)");

  expr = "exists (?b) (and (predicateA ?a)(predicateB ?b))";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "(and (predicateA ?a)(predicateB ?b))");

  expr = "    exists (?b) (and (predicateA ?a)(predicateB ?b))";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "(and (predicateA ?a)(predicateB ?b))");

  expr = "= 3 5";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "3 5"); 

  expr = " = ?a a";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "?a a");
  
  expr = "+ ?a b";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "?a b");
  
  expr = "?s k";
  parser::pddl::removeOperatorBeforeParenthesis(expr);
  ASSERT_EQ(expr, "?s k");
}
