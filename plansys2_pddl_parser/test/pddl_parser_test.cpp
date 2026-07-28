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

#include "ament_index_cpp/get_package_share_path.hpp"
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
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_pddl_parser").string();
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

TEST(PDDLParserTestCase, pddl_parser_suave)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_pddl_parser").string();
  std::string domain_file = pkgpath + "/pddl/suave_domain.pddl";
  std::string instance_file = pkgpath + "/pddl/suave_problem.pddl";

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

  parser::pddl::Domain domain(domain_str);
  ASSERT_EQ(domain.name, "suave");
  ASSERT_EQ(domain.types.size(), 4);
  ASSERT_EQ(domain.types[0]->constants.size(), 34);
  ASSERT_EQ(domain.actions.size(), 5);
  ASSERT_EQ(domain.preds.size(), 47);
  ASSERT_EQ(domain.derived.size(), 54);
}

TEST(PDDLParserTestCase, pddl_parser_suave_extended_created)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_pddl_parser").string();
  std::string domain_file = pkgpath + "/pddl/suave_domain_extended_created.pddl";
  std::string instance_file = pkgpath + "/pddl/suave_problem_extended_created.pddl";

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

  parser::pddl::Domain domain(domain_str);
  ASSERT_EQ(domain.name, "suave_extended");
  ASSERT_EQ(domain.types.size(), 4);
  ASSERT_EQ(domain.types[0]->constants.size(), 39);
  ASSERT_EQ(domain.types[3]->constants.size(), 2);
  ASSERT_EQ(domain.actions.size(), 6);
  ASSERT_EQ(domain.preds.size(), 48);
  ASSERT_EQ(domain.derived.size(), 55);
}

TEST(PDDLParserTestCase, exists_get_tree)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_pddl_parser").string();
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

  auto action_test6 = domain.actions.get("action_test6");
  plansys2_msgs::msg::Tree action_test6_tree;
  action_test6->pre->getTree(action_test6_tree, domain);
  std::string action_test6_tree_str = parser::pddl::toString(action_test6_tree);
  ASSERT_EQ(
    action_test6_tree_str,
    "(and "
    "(exists (?1 ?2) (and (robot_at ?0 ?1)(connected ?1 ?2)))"
    "(exists (?3) (and (battery_full ?3)))"
    "(exists (?4) (and (battery_full ?4)))"
    "(exists (?5 ?6) (and (connected ?5 ?6)))"
    "(exists (?7) (and (battery_full ?7))))"
  );

  auto action_test7 = domain.actions.get("action_test7");
  plansys2_msgs::msg::Tree action_test7_tree;
  action_test7->pre->getTree(action_test7_tree, domain);
  std::string action_test7_tree_str = parser::pddl::toString(action_test7_tree);
  ASSERT_EQ(
    action_test7_tree_str,
    "(or (=  ?0 rob1)(not (exists (?2) (and (not (battery_full ?2))))))");
}

TEST(PDDLParserTestCase, check_node_equality)
{
  auto param1 = parser::pddl::fromStringParam("?x");
  auto param2 = parser::pddl::fromStringParam("?x", "object");
  auto param3 = parser::pddl::fromStringParam("?x", "param_type");
  auto param4 = parser::pddl::fromStringParam("?y");
  auto param5 = parser::pddl::fromStringParam("y");

  ASSERT_TRUE(parser::pddl::checkParamEquality(param1, param2));
  // ASSERT_FALSE(parser::pddl::checkParamEquality(param1, param3));
  // ASSERT_FALSE(parser::pddl::checkParamEquality(param2, param3));
  ASSERT_FALSE(parser::pddl::checkParamEquality(param1, param4));
  ASSERT_FALSE(parser::pddl::checkParamEquality(param1, param5));
  ASSERT_FALSE(parser::pddl::checkParamEquality(param4, param5));
  ASSERT_FALSE(parser::pddl::checkParamEquality(param4, param2));

  auto predicate1 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate2 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate3 = parser::pddl::fromStringPredicate("(predicate ?x b)");
  auto predicate4 = parser::pddl::fromStringPredicate("(predicate a ?y)");
  auto predicate5 = parser::pddl::fromStringPredicate("(predicate a c)");
  auto predicate6 = parser::pddl::fromStringPredicate("(predicate ?x ?y)");

  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate2));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate3));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate4));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate5));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate6));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate6, predicate1));

  auto expression1 = parser::pddl::fromString("(= ?x 5)");
  auto expression2 = parser::pddl::fromString("(= ?x 5)");
  auto expression3 = parser::pddl::fromString("(= ?x ?y)");
  auto expression4 = parser::pddl::fromString("(= ?x 6)");
  auto expression5 = parser::pddl::fromString("(= ?y 5)");
  auto expression6 = parser::pddl::fromString("(= 4 5)");
  auto expression7 = parser::pddl::fromString("(= 4 5)");

  ASSERT_TRUE(parser::pddl::checkTreeEquality(expression1, expression2));
  ASSERT_FALSE(parser::pddl::checkTreeEquality(expression1, expression3));
  ASSERT_FALSE(parser::pddl::checkTreeEquality(expression1, expression4));
  ASSERT_FALSE(parser::pddl::checkTreeEquality(expression1, expression5));
  ASSERT_FALSE(parser::pddl::checkTreeEquality(expression1, expression6));
  ASSERT_FALSE(parser::pddl::checkTreeEquality(expression1, expression7));
  ASSERT_TRUE(parser::pddl::checkTreeEquality(expression6, expression7));

  // with check_var_params = false
  ASSERT_TRUE(parser::pddl::checkParamEquality(param1, param2, false));
  // ASSERT_FALSE(parser::pddl::checkParamEquality(param1, param3, false));
  // ASSERT_FALSE(parser::pddl::checkParamEquality(param2, param3, false));
  ASSERT_TRUE(parser::pddl::checkParamEquality(param1, param4, false));
  ASSERT_TRUE(parser::pddl::checkParamEquality(param1, param5, false));
  ASSERT_TRUE(parser::pddl::checkParamEquality(param4, param5, false));
  ASSERT_TRUE(parser::pddl::checkParamEquality(param4, param2, false));

  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate2, false));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate3, false));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate4, false));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate5, false));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate6, false));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate6, predicate1, false));

  auto predicate1capital = parser::pddl::fromStringPredicate("(PREDICATE A B)");
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate1capital));
}

TEST(PDDLParserTestCase, check_action_equality)
{
  plansys2_msgs::msg::Action actionA;
  actionA.name = "actionA";
  actionA.parameters.push_back(parser::pddl::fromStringParam("a"));

  plansys2_msgs::msg::Tree actionA_preconditions;
  parser::pddl::fromString(actionA_preconditions, "(and (inferredAA a))");
  actionA.preconditions = actionA_preconditions;

  plansys2_msgs::msg::Action actionA_case_test;
  actionA_case_test.name = "aCtIONa";
  actionA_case_test.parameters.push_back(parser::pddl::fromStringParam("A"));

  plansys2_msgs::msg::Tree actionA_case_test_preconditions;
  parser::pddl::fromString(actionA_case_test_preconditions, "(and (inferredaa a))");
  actionA_case_test.preconditions = actionA_case_test_preconditions;

  plansys2_msgs::msg::Action actionB;
  actionB.name = "actionB";
  actionB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree actionB_preconditions;
  parser::pddl::fromString(actionB_preconditions, "(and (inferredBB b))");
  actionB.preconditions = actionB_preconditions;

  plansys2_msgs::msg::Action actionAB;
  actionAB.name = "actionAB";
  actionAB.parameters.push_back(parser::pddl::fromStringParam("a"));
  actionAB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree actionAB_preconditions;
  parser::pddl::fromString(actionAB_preconditions, "(and (inferredAB a b))");
  actionAB.preconditions = actionAB_preconditions;

  ASSERT_TRUE(parser::pddl::checkActionEquality(actionA, actionA));
  ASSERT_FALSE(parser::pddl::checkActionEquality(actionA, actionA_case_test));
  ASSERT_TRUE(parser::pddl::checkActionEquality(actionB, actionB));
  ASSERT_TRUE(parser::pddl::checkActionEquality(actionAB, actionAB));
  ASSERT_FALSE(parser::pddl::checkActionEquality(actionA, actionB));
  ASSERT_FALSE(parser::pddl::checkActionEquality(actionA, actionAB));

  plansys2_msgs::msg::DurativeAction dur_actionA;
  dur_actionA.name = "dur_actionA";
  dur_actionA.parameters.push_back(parser::pddl::fromStringParam("a"));

  plansys2_msgs::msg::Tree dur_actionA_over_all_requirements;
  parser::pddl::fromString(dur_actionA_over_all_requirements, "(and (over all (inferredAA a)))");
  dur_actionA.over_all_requirements = dur_actionA_over_all_requirements;

  plansys2_msgs::msg::DurativeAction dur_actionA_case_test;
  dur_actionA_case_test.name = "dur_aCtIONa";
  dur_actionA_case_test.parameters.push_back(parser::pddl::fromStringParam("A"));

  plansys2_msgs::msg::Tree dur_actionA_case_test_over_all_requirements;
  parser::pddl::fromString(
    dur_actionA_case_test_over_all_requirements,
    "(and (over all (inferredaa a)))");
  dur_actionA_case_test.over_all_requirements = dur_actionA_case_test_over_all_requirements;

  plansys2_msgs::msg::DurativeAction dur_actionB;
  dur_actionB.name = "dur_actionB";
  dur_actionB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree dur_actionB_over_all_requirements;
  parser::pddl::fromString(dur_actionB_over_all_requirements, "(and (over all (inferredBB b)))");
  dur_actionB.over_all_requirements = dur_actionB_over_all_requirements;

  plansys2_msgs::msg::DurativeAction dur_actionAB;
  dur_actionAB.name = "dur_actionAB";
  dur_actionAB.parameters.push_back(parser::pddl::fromStringParam("a"));
  dur_actionAB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree dur_actionAB_over_all_requirements;
  parser::pddl::fromString(dur_actionAB_over_all_requirements, "(and (over all (inferredAB a b)))");
  dur_actionAB.over_all_requirements = dur_actionAB_over_all_requirements;

  ASSERT_TRUE(parser::pddl::checkDurativeActionEquality(dur_actionA, dur_actionA));
  ASSERT_FALSE(parser::pddl::checkDurativeActionEquality(dur_actionA, dur_actionA_case_test));
  ASSERT_TRUE(parser::pddl::checkDurativeActionEquality(dur_actionB, dur_actionB));
  ASSERT_TRUE(parser::pddl::checkDurativeActionEquality(dur_actionAB, dur_actionAB));
  ASSERT_FALSE(parser::pddl::checkDurativeActionEquality(dur_actionA, dur_actionB));
  ASSERT_FALSE(parser::pddl::checkDurativeActionEquality(dur_actionA, dur_actionAB));
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

TEST(PDDLParserTestCase, open_door_test)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_pddl_parser").string();
  std::string domain_file = pkgpath + "/pddl/dom2.pddl";

  std::ifstream domain_ifs(domain_file);
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());
  parser::pddl::Domain domain(domain_str);

  auto action = domain.actions.get("open_door");
  plansys2_msgs::msg::Tree tree_pre;
  action->pre->getTree(tree_pre, domain);
  std::string str_c = parser::pddl::toString(tree_pre);
  plansys2_msgs::msg::Tree tree_eff;
  action->eff->getTree(tree_eff, domain);
  std::string str_e = parser::pddl::toString(tree_eff);
  ASSERT_EQ(str_c, "(and (not_door_open))");
  ASSERT_EQ(str_e, "(and (door_open)(not (not_door_open)))");
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

TEST(PDDLParserTestCase, get_sub_expr)
{
  std::string expr =
    "(and (predicateA ?a)"
    "(exists (?b) (and (inferredA ?a)(inferredB ?b)(inferredAB ?a ?b)(not(=?a ?b)))))";
  std::vector<std::string> subexprs = parser::pddl::getSubExpr(expr);
  ASSERT_EQ(subexprs.size(), 2);
  ASSERT_EQ(subexprs[0], "(predicateA ?a)");
  ASSERT_EQ(
    subexprs[1], "(exists (?b) (and (inferredA ?a)(inferredB ?b)(inferredAB ?a ?b)(not(=?a ?b))))");

  subexprs = parser::pddl::getSubExpr(subexprs[1]);
  ASSERT_EQ(subexprs.size(), 1);
  ASSERT_EQ(subexprs[0], "(and (inferredA ?a)(inferredB ?b)(inferredAB ?a ?b)(not(=?a ?b)))");

  expr = "(= 3 5)";
  subexprs = parser::pddl::getSubExpr(expr);
  ASSERT_EQ(subexprs.size(), 2);
  ASSERT_EQ(subexprs[0], "3");
  ASSERT_EQ(subexprs[1], "5");

  expr = "(= ?a a)";
  subexprs = parser::pddl::getSubExpr(expr);
  ASSERT_EQ(subexprs.size(), 2);
  ASSERT_EQ(subexprs[0], "?a");
  ASSERT_EQ(subexprs[1], "a");

  expr = "(+ ?a b)";
  subexprs = parser::pddl::getSubExpr(expr);
  ASSERT_EQ(subexprs.size(), 2);
  ASSERT_EQ(subexprs[0], "?a");
  ASSERT_EQ(subexprs[1], "b");

  expr = "(?s k)";
  subexprs = parser::pddl::getSubExpr(expr);
  ASSERT_EQ(subexprs.size(), 2);
  ASSERT_EQ(subexprs[0], "?s");
  ASSERT_EQ(subexprs[1], "k");

  expr = "(- b ?sas)";
  subexprs = parser::pddl::getSubExpr(expr);
  ASSERT_EQ(subexprs.size(), 2);
  ASSERT_EQ(subexprs[0], "b");
  ASSERT_EQ(subexprs[1], "?sas");
}

TEST(PDDLParserTestCase, from_string)
{
  std::string expr = "(not(=?a ?b))";
  plansys2_msgs::msg::Tree tree = parser::pddl::fromString(expr);
  ASSERT_EQ(tree.nodes.size(), 4);
  ASSERT_EQ(tree.nodes[0].node_type, plansys2_msgs::msg::Node::NOT);
  ASSERT_EQ(tree.nodes[0].node_id, 0);
  ASSERT_EQ(tree.nodes[0].children, std::vector<unsigned int>({1}));
  ASSERT_EQ(tree.nodes[1].node_type, plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(tree.nodes[1].node_id, 1);
  ASSERT_EQ(tree.nodes[1].expression_type, plansys2_msgs::msg::Node::COMP_EQ);
  ASSERT_EQ(tree.nodes[1].children, std::vector<unsigned int>({2, 3}));
  ASSERT_EQ(tree.nodes[2].node_type, plansys2_msgs::msg::Node::PARAMETER);
  ASSERT_EQ(tree.nodes[2].node_id, 2);
  ASSERT_EQ(tree.nodes[2].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[2].parameters[0].name, "?a");
  ASSERT_EQ(tree.nodes[3].node_type, plansys2_msgs::msg::Node::PARAMETER);
  ASSERT_EQ(tree.nodes[3].node_id, 3);
  ASSERT_EQ(tree.nodes[3].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[3].parameters[0].name, "?b");

  expr =
    "(and (predicateA ?a) (exists (?b) (and (inferredA ?a)(inferredB ?b)(inferredAB ?a ?b)(not(=?a "
    "?b)))))";
  tree = parser::pddl::fromString(expr);
  ASSERT_EQ(tree.nodes.size(), 11);
  ASSERT_EQ(tree.nodes[0].node_type, plansys2_msgs::msg::Node::AND);
  ASSERT_EQ(tree.nodes[0].node_id, 0);
  ASSERT_EQ(tree.nodes[0].children, std::vector<unsigned int>({1, 2}));

  ASSERT_EQ(tree.nodes[1].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(tree.nodes[1].node_id, 1);
  ASSERT_EQ(tree.nodes[1].name, "predicateA");
  ASSERT_EQ(tree.nodes[1].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[1].parameters[0].name, "?a");

  ASSERT_EQ(tree.nodes[2].node_type, plansys2_msgs::msg::Node::EXISTS);
  ASSERT_EQ(tree.nodes[2].node_id, 2);
  ASSERT_EQ(tree.nodes[2].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[2].parameters[0].name, "?b");
  ASSERT_EQ(tree.nodes[2].children, std::vector<unsigned int>({3}));

  ASSERT_EQ(tree.nodes[3].node_type, plansys2_msgs::msg::Node::AND);
  ASSERT_EQ(tree.nodes[3].node_id, 3);
  ASSERT_EQ(tree.nodes[3].children, std::vector<unsigned int>({4, 5, 6, 7}));

  ASSERT_EQ(tree.nodes[4].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(tree.nodes[4].node_id, 4);
  ASSERT_EQ(tree.nodes[4].name, "inferredA");
  ASSERT_EQ(tree.nodes[4].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[4].parameters[0].name, "?a");

  ASSERT_EQ(tree.nodes[5].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(tree.nodes[5].node_id, 5);
  ASSERT_EQ(tree.nodes[5].name, "inferredB");
  ASSERT_EQ(tree.nodes[5].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[5].parameters[0].name, "?b");

  ASSERT_EQ(tree.nodes[6].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(tree.nodes[6].node_id, 6);
  ASSERT_EQ(tree.nodes[6].name, "inferredAB");
  ASSERT_EQ(tree.nodes[6].parameters.size(), 2);
  ASSERT_EQ(tree.nodes[6].parameters[0].name, "?a");
  ASSERT_EQ(tree.nodes[6].parameters[1].name, "?b");

  ASSERT_EQ(tree.nodes[7].node_type, plansys2_msgs::msg::Node::NOT);
  ASSERT_EQ(tree.nodes[7].node_id, 7);
  ASSERT_EQ(tree.nodes[7].children, std::vector<unsigned int>({8}));

  ASSERT_EQ(tree.nodes[8].node_type, plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(tree.nodes[8].node_id, 8);
  ASSERT_EQ(tree.nodes[8].expression_type, plansys2_msgs::msg::Node::COMP_EQ);
  ASSERT_EQ(tree.nodes[8].children, std::vector<unsigned int>({9, 10}));

  ASSERT_EQ(tree.nodes[9].node_type, plansys2_msgs::msg::Node::PARAMETER);
  ASSERT_EQ(tree.nodes[9].node_id, 9);
  ASSERT_EQ(tree.nodes[9].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[9].parameters[0].name, "?a");

  ASSERT_EQ(tree.nodes[10].node_type, plansys2_msgs::msg::Node::PARAMETER);
  ASSERT_EQ(tree.nodes[10].node_id, 10);
  ASSERT_EQ(tree.nodes[10].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[10].parameters[0].name, "?b");

  expr = "(+ ?a b)";
  tree = parser::pddl::fromString(expr);
  ASSERT_EQ(tree.nodes.size(), 3);
  ASSERT_EQ(tree.nodes[0].node_type, plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(tree.nodes[0].node_id, 0);
  ASSERT_EQ(tree.nodes[0].expression_type, plansys2_msgs::msg::Node::ARITH_ADD);
  ASSERT_EQ(tree.nodes[0].children, std::vector<unsigned int>({1, 2}));

  ASSERT_EQ(tree.nodes[1].node_type, plansys2_msgs::msg::Node::PARAMETER);
  ASSERT_EQ(tree.nodes[1].node_id, 1);
  ASSERT_EQ(tree.nodes[1].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[1].parameters[0].name, "?a");

  // Should it be a function or a constant? IMO, it could be both.
  ASSERT_EQ(tree.nodes[2].node_type, plansys2_msgs::msg::Node::FUNCTION);
  ASSERT_EQ(tree.nodes[2].node_id, 2);

  expr = "(= 3 5)";
  tree = parser::pddl::fromString(expr);
  ASSERT_EQ(tree.nodes.size(), 3);
  ASSERT_EQ(tree.nodes[0].node_type, plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(tree.nodes[0].node_id, 0);
  ASSERT_EQ(tree.nodes[0].expression_type, plansys2_msgs::msg::Node::COMP_EQ);
  ASSERT_EQ(tree.nodes[0].children, std::vector<unsigned int>({1, 2}));

  ASSERT_EQ(tree.nodes[1].node_type, plansys2_msgs::msg::Node::NUMBER);
  ASSERT_EQ(tree.nodes[1].node_id, 1);
  ASSERT_EQ(tree.nodes[1].value, 3.0);

  ASSERT_EQ(tree.nodes[2].node_type, plansys2_msgs::msg::Node::NUMBER);
  ASSERT_EQ(tree.nodes[2].node_id, 2);
  ASSERT_EQ(tree.nodes[2].value, 5.0);

  expr = "(or (predicateA ?a) (predicateB ?b))";
  tree = parser::pddl::fromString(expr);
  ASSERT_EQ(tree.nodes.size(), 3);
  ASSERT_EQ(tree.nodes[0].node_type, plansys2_msgs::msg::Node::OR);
  ASSERT_EQ(tree.nodes[0].node_id, 0);
  ASSERT_EQ(tree.nodes[0].children, std::vector<unsigned int>({1, 2}));

  ASSERT_EQ(tree.nodes[1].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(tree.nodes[1].node_id, 1);
  ASSERT_EQ(tree.nodes[1].name, "predicateA");
  ASSERT_EQ(tree.nodes[1].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[1].parameters[0].name, "?a");

  ASSERT_EQ(tree.nodes[2].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(tree.nodes[2].node_id, 2);
  ASSERT_EQ(tree.nodes[2].name, "predicateB");
  ASSERT_EQ(tree.nodes[2].parameters.size(), 1);
  ASSERT_EQ(tree.nodes[2].parameters[0].name, "?b");
}

TEST(PDDLParserTestCase, get_node_type)
{
  ASSERT_EQ(
    parser::pddl::getNodeType("(and (predicateA ?a) (predicateB ?b))"),
    plansys2_msgs::msg::Node::AND);
  ASSERT_EQ(
    parser::pddl::getNodeType("(or (predicateA ?a) (predicateB ?b))"),
    plansys2_msgs::msg::Node::OR);
  ASSERT_EQ(parser::pddl::getNodeType("(not (predicateA ?a))"), plansys2_msgs::msg::Node::NOT);
  ASSERT_EQ(
    parser::pddl::getNodeType("(exists (?b) (and (predicateA ?a)(predicateB ?b)))"),
    plansys2_msgs::msg::Node::EXISTS);
  ASSERT_EQ(parser::pddl::getNodeType("(= 3 5)"), plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(parser::pddl::getNodeType("(+ ?a b)"), plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(parser::pddl::getNodeType("(- b ?sas)"), plansys2_msgs::msg::Node::EXPRESSION);
  ASSERT_EQ(parser::pddl::getNodeType("?a"), plansys2_msgs::msg::Node::PARAMETER);
  ASSERT_EQ(parser::pddl::getNodeType("3"), plansys2_msgs::msg::Node::NUMBER);
}

TEST(PDDLParserTestCase, get_subtree)
{
  plansys2_msgs::msg::Tree tree = parser::pddl::fromString("(and (predicateA ?a) (predicateB ?b))");
  auto subtree = parser::pddl::getSubtrees(tree);
  ASSERT_EQ(subtree.size(), 2);
  ASSERT_EQ(subtree[0].nodes.size(), 1);
  ASSERT_EQ(subtree[0].nodes[0].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(subtree[0].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[0].nodes[0].name, "predicateA");
  ASSERT_EQ(subtree[1].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[1].nodes[0].name, "predicateB");

  tree = parser::pddl::fromString(
    "(and (predicateA ?a) (exists (?b) (and (inferredA ?a)(inferredB ?b)"
    "(inferredAB ?a ?b)(not(=?a ?b)))))"
  );
  subtree = parser::pddl::getSubtrees(tree);
  ASSERT_EQ(subtree.size(), 2);
  ASSERT_EQ(subtree[0].nodes.size(), 1);
  ASSERT_EQ(subtree[0].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[0].nodes[0].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(subtree[0].nodes[0].name, "predicateA");
  ASSERT_EQ(subtree[1].nodes.size(), 9);
  ASSERT_EQ(subtree[1].nodes[0].node_type, plansys2_msgs::msg::Node::EXISTS);

  tree = parser::pddl::fromString(
    "(and (exists (?1 ?2) (and (robot_at ?0 ?1)(connected ?1 ?2)))"
    "(exists (?3) (and (battery_full ?3)))"
    "(exists (?4) (and (battery_full ?4)))"
    "(exists (?5 ?6) (and (connected ?5 ?6)))"
    "(exists (?7) (and (battery_full ?7))))"
  );

  subtree = parser::pddl::getSubtrees(tree);
  ASSERT_EQ(subtree.size(), 5);
  ASSERT_EQ(subtree[0].nodes.size(), 4);
  ASSERT_EQ(subtree[1].nodes.size(), 3);
  ASSERT_EQ(subtree[2].nodes.size(), 3);
  ASSERT_EQ(subtree[3].nodes.size(), 3);
  ASSERT_EQ(subtree[4].nodes.size(), 3);

  tree = parser::pddl::fromString(
    "(and"
    "  (Function ?f)"
    "  (inferred-SolvesF ?fd_goal ?f)"
    "  (FunctionDesign ?fd_goal)"
    "  (not (inferred-Fd_realisability ?fd_goal false_boolean))"
    "  (not"
    "    (exists (?fd)"
    "      (and"
    "        (inferred-SolvesF ?fd ?f)"
    "        (FunctionDesign ?fd)"
    "        (functionGrounding ?f ?fd)"
    "      )"
    "    )"
    "  )"
    "  (or "
    "   (= ?fd_goal fd_unground)"
    "    (not"
    "      (exists (?fd)"
    "        (and"
    "          (inferred-SolvesF ?fd ?f)"
    "          (not (inferred-Fd_realisability ?fd false_boolean))"
    "          (inferred-FdBetterUtility  ?fd ?fd_goal)"
    "        )"
    "      )"
    "    )"
    "  )"
    ")"
  );
  subtree = parser::pddl::getSubtrees(tree);
  ASSERT_EQ(subtree.size(), 6);

  ASSERT_EQ(subtree[0].nodes.size(), 1);
  ASSERT_EQ(subtree[0].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[0].nodes[0].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(subtree[0].nodes[0].name, "Function");

  ASSERT_EQ(subtree[1].nodes.size(), 1);
  ASSERT_EQ(subtree[1].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[1].nodes[0].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(subtree[1].nodes[0].name, "inferred-SolvesF");

  ASSERT_EQ(subtree[2].nodes.size(), 1);
  ASSERT_EQ(subtree[2].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[2].nodes[0].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(subtree[2].nodes[0].name, "FunctionDesign");

  ASSERT_EQ(subtree[3].nodes.size(), 2);
  ASSERT_EQ(subtree[3].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[3].nodes[0].node_type, plansys2_msgs::msg::Node::NOT);
  ASSERT_EQ(subtree[3].nodes[1].node_type, plansys2_msgs::msg::Node::PREDICATE);
  ASSERT_EQ(subtree[3].nodes[1].name, "inferred-Fd_realisability");

  ASSERT_EQ(subtree[4].nodes.size(), 6);
  ASSERT_EQ(subtree[4].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[4].nodes[0].node_type, plansys2_msgs::msg::Node::NOT);

  ASSERT_EQ(subtree[5].nodes.size(), 11);
  ASSERT_EQ(subtree[5].nodes[0].node_id, 0);
  ASSERT_EQ(subtree[5].nodes[0].node_type, plansys2_msgs::msg::Node::OR);
}
