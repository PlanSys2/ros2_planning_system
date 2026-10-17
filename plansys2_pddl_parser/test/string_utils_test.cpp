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

// String utilities of the parser: reading expressions into trees and writing them back

#include <chrono>
#include <clocale>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using Node = plansys2_msgs::msg::Node;
using parser::pddl::fromString;
using parser::pddl::toString;

namespace
{
// The text must be read and written back unchanged, as a valid tree
void expectRoundTrip(const std::string & expr, uint8_t parent = Node::UNKNOWN)
{
  auto tree = fromString(expr, false, parent);
  ASSERT_FALSE(tree.nodes.empty()) << expr;
  std::string error;
  EXPECT_TRUE(parser::pddl::validateTree(tree, error)) << expr << ": " << error;
  EXPECT_EQ(toString(tree), expr);
  EXPECT_GE(parser::pddl::getMaxNesting(toString(tree)), 0) << toString(tree);
}
}  // namespace

TEST(StringUtils, parse_number_accepts_pddl_numbers)
{
  const std::vector<std::pair<std::string, double>> good = {
    {"0", 0.0}, {"1", 1.0}, {"-1", -1.0}, {"+1", 1.0}, {"1.5", 1.5}, {".5", 0.5},
    {"5.", 5.0}, {"-0.25", -0.25}, {"1e3", 1000.0}, {"1E-3", 0.001}, {"-2.5e+2", -250.0},
    {"1e-7", 1e-7}, {"  7  ", 7.0}, {"\t3\n", 3.0}, {"123456789012", 123456789012.0},
    {"0.1", 0.1}};
  for (const auto & [text, expected] : good) {
    double value = -12345.0;
    EXPECT_TRUE(parser::pddl::parseNumber(text, value)) << "[" << text << "]";
    EXPECT_DOUBLE_EQ(value, expected) << "[" << text << "]";
  }
}

TEST(StringUtils, parse_number_rejects_the_rest)
{
  const std::vector<std::string> bad = {
    "", " ", "-", "+", "+-1", "e3", "1e", "1e+", "abc", "1.5.2", "1,5", "inf", "-inf", "nan",
    "0x10", "1e400", "5abc", "r1", "?x", "(5)", "1 2", "--1", "."};
  for (const auto & text : bad) {
    double value = 42.0;
    EXPECT_FALSE(parser::pddl::parseNumber(text, value)) << "[" << text << "]";
    EXPECT_EQ(value, 42.0) << "value changed by [" << text << "]";
  }
}

TEST(StringUtils, reduced_string_normalizes_blanks)
{
  using parser::pddl::getReducedString;
  EXPECT_EQ(getReducedString("(robot_at r1 wp1)"), "(robot_at r1 wp1)");
  EXPECT_EQ(getReducedString("(robot_at\tr1\twp1)"), "(robot_at r1 wp1)");
  EXPECT_EQ(getReducedString("(robot_at r1\nwp1)"), "(robot_at r1 wp1)");
  EXPECT_EQ(getReducedString("(robot_at r1\r\nwp1)"), "(robot_at r1 wp1)");
  EXPECT_EQ(getReducedString("(  robot_at   r1  wp1  )"), "(robot_at r1 wp1)");
  EXPECT_EQ(getReducedString("  (and (p)\n\t(q))  "), "(and (p) (q))");
  EXPECT_EQ(getReducedString("(and\n  (p)\n  (q)\n)"), "(and (p) (q))");
  EXPECT_EQ(getReducedString("((p))"), "((p))");
  EXPECT_EQ(getReducedString(""), "");
  EXPECT_EQ(getReducedString(" \t\n "), "");
  EXPECT_EQ(getReducedString("(move r1 a b):5"), "(move r1 a b):5");
}

TEST(StringUtils, node_type_comes_from_the_operator)
{
  using parser::pddl::getNodeType;
  const auto P = Node::PREDICATE;
  const std::vector<std::tuple<std::string, uint8_t, uint8_t>> cases = {
    {"(and (p) (q))", P, Node::AND},
    {"( and (p))", P, Node::AND},
    {"(and(p)(q))", P, Node::AND},
    {"(or (p) (q))", P, Node::OR},
    {"(not (p))", P, Node::NOT},
    {"(not(p))", P, Node::NOT},
    {"(exists (?x) (p ?x))", P, Node::EXISTS},
    {"(forall (?x) (p ?x))", P, Node::FORALL},
    {"(imply (p) (q))", P, Node::IMPLY},
    {"(assign (f) 1)", Node::FUNCTION, Node::FUNCTION_MODIFIER},
    {"(increase (f) 1)", Node::FUNCTION, Node::FUNCTION_MODIFIER},
    {"(decrease (f) 1)", Node::FUNCTION, Node::FUNCTION_MODIFIER},
    {"(scale-up (f) 2)", Node::FUNCTION, Node::FUNCTION_MODIFIER},
    {"(scale-down (f) 2)", Node::FUNCTION, Node::FUNCTION_MODIFIER},
    {"(> (f) 1)", P, Node::EXPRESSION},
    {"(>= (f) 1)", P, Node::EXPRESSION},
    {"(< (f) 1)", P, Node::EXPRESSION},
    {"(<= (f) 1)", P, Node::EXPRESSION},
    {"(= ?a ?b)", P, Node::EXPRESSION},
    {"(=?a ?b)", P, Node::EXPRESSION},
    {"(<=(f) 2)", P, Node::EXPRESSION},
    {"(+ (f) 1)", Node::FUNCTION, Node::EXPRESSION},
    {"(- (f) 1)", Node::FUNCTION, Node::EXPRESSION},
    {"(-(f) 1)", Node::FUNCTION, Node::EXPRESSION},
    {"(* (f) 2)", Node::FUNCTION, Node::EXPRESSION},
    {"(/ (f) 2)", Node::FUNCTION, Node::EXPRESSION},
    {"5", Node::FUNCTION, Node::NUMBER},
    {"-5", Node::FUNCTION, Node::NUMBER},
    {"+5", Node::FUNCTION, Node::NUMBER},
    {"2.5e-3", Node::FUNCTION, Node::NUMBER},
    {"?x", Node::FUNCTION, Node::PARAMETER},
    {"(?x)", P, Node::PARAMETER},
    {"r1", Node::FUNCTION, Node::FUNCTION},
    {"", P, P},
    // Names that contain an operator are predicates or functions
    {"(task_assigned r1)", P, P},
    {"(assigned r1)", P, P},
    {"(reassign r1)", P, P},
    {"(counter_increased r1)", Node::FUNCTION, Node::FUNCTION},
    {"(decreased_speed r1)", Node::FUNCTION, Node::FUNCTION},
    {"(can-scale-up r1)", P, P},
    {"(scale-down-ready r1)", P, P},
    {"(android r1)", P, P},
    {"(order_ready r1)", P, P},
    {"(notified r1)", P, P},
    {"(knot r1)", P, P},
    {"(exists_path a b)", P, P},
    {"(forall_done)", P, P},
    {"(implying a)", P, P},
    {"(is-free r1)", P, P},
    {"(at-robot r1 wp1)", P, P},
    {"(robot_at r1 wp1)", P, P},
  };
  for (const auto & [expr, def, expected] : cases) {
    EXPECT_EQ(getNodeType(expr, def), expected) << "[" << expr << "]";
  }
}

TEST(StringUtils, expression_and_modifier_types)
{
  using parser::pddl::getExprType;
  using parser::pddl::getFunModType;
  EXPECT_EQ(getExprType(">="), Node::COMP_GE);
  EXPECT_EQ(getExprType(">"), Node::COMP_GT);
  EXPECT_EQ(getExprType("<="), Node::COMP_LE);
  EXPECT_EQ(getExprType("<"), Node::COMP_LT);
  EXPECT_EQ(getExprType("="), Node::COMP_EQ);
  EXPECT_EQ(getExprType("*"), Node::ARITH_MULT);
  EXPECT_EQ(getExprType("/"), Node::ARITH_DIV);
  EXPECT_EQ(getExprType("+"), Node::ARITH_ADD);
  EXPECT_EQ(getExprType("-"), Node::ARITH_SUB);
  EXPECT_EQ(getExprType("(- (f) 1)"), Node::ARITH_SUB);
  EXPECT_EQ(getExprType("(>= (f) 1)"), Node::COMP_GE);
  EXPECT_EQ(getExprType("(task_assigned r1)"), Node::UNKNOWN);
  EXPECT_EQ(getExprType("-5"), Node::UNKNOWN);
  EXPECT_EQ(getExprType(""), Node::UNKNOWN);
  EXPECT_EQ(std::get<1>(parser::pddl::getExpr("(robot_at r1)")), std::numeric_limits<int>::max());

  EXPECT_EQ(getFunModType("assign"), Node::ASSIGN);
  EXPECT_EQ(getFunModType("increase"), Node::INCREASE);
  EXPECT_EQ(getFunModType("decrease"), Node::DECREASE);
  EXPECT_EQ(getFunModType("scale-up"), Node::SCALE_UP);
  EXPECT_EQ(getFunModType("scale-down"), Node::SCALE_DOWN);
  EXPECT_EQ(getFunModType("(increase (f) 1)"), Node::INCREASE);
  EXPECT_EQ(getFunModType("(task_assigned r1)"), Node::UNKNOWN);
  EXPECT_EQ(getFunModType("(counter_increased r1)"), Node::UNKNOWN);
  EXPECT_EQ(getFunModType("reassign"), Node::UNKNOWN);
}

TEST(StringUtils, names_with_operators_are_not_corrupted)
{
  const std::vector<std::string> exprs = {
    "(task_assigned r1)",
    "(not (task_assigned r1))",
    "(and (task_assigned r1)(robot_assigned r2 t1))",
    "(or (android r1)(order_ready r1))",
    "(and (notified r1)(not (knot r2)))",
    "(exists_path a b)",
    "(forall_done)",
    "(implying a)",
    "(can-scale-up r1)",
    "(scale-down-ready r1)",
    "(is-free r1)",
  };
  for (const auto & expr : exprs) {
    expectRoundTrip(expr);
  }

  auto tree = fromString("(task_assigned r1)");
  ASSERT_EQ(tree.nodes.size(), 1u);
  EXPECT_EQ(tree.nodes[0].node_type, Node::PREDICATE);
  EXPECT_EQ(tree.nodes[0].name, "task_assigned");
  ASSERT_EQ(tree.nodes[0].parameters.size(), 1u);
  EXPECT_EQ(tree.nodes[0].parameters[0].name, "r1");

  // Functions whose names contain a modifier, in comparisons and effects
  expectRoundTrip("(> (counter_increased r1) 1.000000)");
  expectRoundTrip("(< (decreased_speed r1) (assigned_load r1))");
  expectRoundTrip("(increase (counter_increased r1) 1.000000)", Node::AND);
  expectRoundTrip("(assign (reassign_count r1) (decreased_speed r1))", Node::AND);
  tree = fromString("(> (counter_increased r1) 1)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_EQ(tree.nodes[1].node_type, Node::FUNCTION);
  EXPECT_EQ(tree.nodes[1].name, "counter_increased");
}

TEST(StringUtils, tabs_and_newlines_separate_tokens)
{
  for (const std::string expr : {
    "(robot_at\tr1\twp1)", "(robot_at r1\nwp1)", "(robot_at r1\r\nwp1)",
    "(robot_at\n  r1\n  wp1\n)", "\n(robot_at r1 wp1)\n"})
  {
    auto tree = fromString(expr);
    ASSERT_EQ(tree.nodes.size(), 1u) << expr;
    EXPECT_EQ(tree.nodes[0].name, "robot_at");
    ASSERT_EQ(tree.nodes[0].parameters.size(), 2u) << expr;
    EXPECT_EQ(tree.nodes[0].parameters[0].name, "r1");
    EXPECT_EQ(tree.nodes[0].parameters[1].name, "wp1");
  }

  auto tree = fromString("(and\n\t(robot_at r1 wp1)\n\t(not (robot_at r2\twp1))\n)");
  EXPECT_EQ(toString(tree), "(and (robot_at r1 wp1)(not (robot_at r2 wp1)))");

  tree = fromString("(>\t(battery\nr1)\t10)");
  EXPECT_EQ(toString(tree), "(> (battery r1) 10.000000)");
}

TEST(StringUtils, number_equality_is_not_truncated)
{
  Node a, b;
  a.node_type = b.node_type = Node::NUMBER;
  a.value = 0.5;
  b.value = 0.9;
  EXPECT_FALSE(parser::pddl::checkNodeEquality(a, b));
  b.value = -0.5;
  EXPECT_FALSE(parser::pddl::checkNodeEquality(a, b));
  b.value = 0.5 + 1e-12;
  EXPECT_TRUE(parser::pddl::checkNodeEquality(a, b));
  b.value = 0.5;
  EXPECT_TRUE(parser::pddl::checkNodeEquality(a, b));

  EXPECT_FALSE(
    parser::pddl::checkTreeEquality(fromString("(> (f) 0.5)"), fromString("(> (f) 0.9)")));
  EXPECT_TRUE(
    parser::pddl::checkTreeEquality(fromString("(> (f) 0.5)"), fromString("(> (f) 0.50)")));
}

TEST(StringUtils, negated_expressions_are_balanced)
{
  expectRoundTrip("(not (> (+ (f) 1.000000) 2.000000))");
  expectRoundTrip("(not (= (* (f) 2.000000) (- (g) 1.000000)))");
  expectRoundTrip("(not (<= (/ (f a) (g b)) 0.500000))");
  expectRoundTrip("(and (p)(not (>= (- (f) (g)) 1.000000)))");

  // Negation pushed down: De Morgan for and/or, and never into the operands
  auto tree = fromString("(not (and (p) (> (+ (f) 1) 2)))");
  const auto text = toString(tree);
  EXPECT_EQ(text, "(or (not (p))(not (> (+ (f) 1.000000) 2.000000)))");
  EXPECT_GE(parser::pddl::getMaxNesting(text), 0);
  tree = fromString("(not (not (> (- (f) 1) 0)))");
  EXPECT_EQ(toString(tree), "(> (- (f) 1.000000) 0.000000)");
}

TEST(StringUtils, all_arithmetic_is_written)
{
  expectRoundTrip("(- (f) 1.000000)", Node::EXPRESSION);
  expectRoundTrip("(+ (f) (g))", Node::EXPRESSION);
  expectRoundTrip("(* 2.000000 (f))", Node::EXPRESSION);
  expectRoundTrip("(/ (f) (g))", Node::EXPRESSION);
  expectRoundTrip("(> (- (f) (g)) 0.000000)");
  expectRoundTrip("(< (- (f r1) (- (g r1) 1.000000)) -2.500000)");
  expectRoundTrip("(= (f) -1.000000)");

  auto tree = fromString("(- (f) 1)", false, Node::EXPRESSION);
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_EQ(tree.nodes[0].expression_type, Node::ARITH_SUB);
  EXPECT_EQ(tree.nodes[2].node_type, Node::NUMBER);
  EXPECT_DOUBLE_EQ(tree.nodes[2].value, 1.0);

  // Negative numbers as operands, and "-" glued to its first operand
  tree = fromString("(> (f) -3)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_DOUBLE_EQ(tree.nodes[2].value, -3.0);
  EXPECT_EQ(toString(fromString("(-(f) 1)", false, Node::EXPRESSION)), "(- (f) 1.000000)");
}

TEST(StringUtils, modifiers_are_written_with_spaces)
{
  expectRoundTrip("(increase (f r1) 1.000000)", Node::AND);
  expectRoundTrip("(decrease (f r1) (g r1))", Node::AND);
  expectRoundTrip("(assign (f) (+ (g) 2.000000))", Node::AND);
  expectRoundTrip("(scale-up (f) 2.000000)", Node::AND);
  expectRoundTrip("(scale-down (f) 0.500000)", Node::AND);
  expectRoundTrip("(increase (total-cost) 5.000000)", Node::AND);
  expectRoundTrip("(and (increase (f) 1.000000)(not (p)))");

  // Trees built before (total-cost) was a child: only the value
  plansys2_msgs::msg::Tree old;
  old.nodes.resize(2);
  old.nodes[0].node_type = Node::FUNCTION_MODIFIER;
  old.nodes[0].modifier_type = Node::INCREASE;
  old.nodes[0].children = {1};
  old.nodes[1].node_type = Node::NUMBER;
  old.nodes[1].node_id = 1;
  old.nodes[1].value = 3.0;
  std::string error;
  EXPECT_TRUE(parser::pddl::validateTree(old, error)) << error;
  EXPECT_EQ(toString(old), "(increase (total-cost) 3.000000)");
}

TEST(StringUtils, function_parameters_keep_their_question_mark)
{
  auto tree = fromString("(> (battery ?r) 10)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  ASSERT_EQ(tree.nodes[1].parameters.size(), 1u);
  EXPECT_EQ(tree.nodes[1].parameters[0].name, "?r");
  EXPECT_EQ(toString(tree), "(> (battery ?r) 10.000000)");

  auto func = parser::pddl::fromStringFunction("(distance ?a ?b) 1.5");
  EXPECT_EQ(func.name, "distance");
  ASSERT_EQ(func.parameters.size(), 2u);
  EXPECT_EQ(func.parameters[0].name, "?a");
  EXPECT_EQ(func.parameters[1].name, "?b");
  EXPECT_DOUBLE_EQ(func.value, 1.5);
}

TEST(StringUtils, functions_are_read_in_every_form)
{
  struct Case
  {
    std::string text;
    std::string name;
    std::vector<std::string> params;
    double value;
  };
  const std::vector<Case> cases = {
    {"(battery r1)", "battery", {"r1"}, 0.0},
    {"(battery r1) 1.5", "battery", {"r1"}, 1.5},
    {"(= (battery r1) 1.5)", "battery", {"r1"}, 1.5},
    {" (= (battery r1) 100)", "battery", {"r1"}, 100.0},
    {"(= (battery r1) -2.5)", "battery", {"r1"}, -2.5},
    {"(= (battery r1) 1e-7)", "battery", {"r1"}, 1e-7},
    {"(= (battery r1) 1.5e3)", "battery", {"r1"}, 1500.0},
    {"(= (battery r1) +4)", "battery", {"r1"}, 4.0},
    {"(= (distance wp1 wp2) 3)", "distance", {"wp1", "wp2"}, 3.0},
    {"(=\t(distance\nwp1 wp2)\n3)", "distance", {"wp1", "wp2"}, 3.0},
    {"(= (total-cost) 0)", "total-cost", {}, 0.0},
    {"(time)", "time", {}, 0.0},
    {"(robot-speed r_1) 0.25", "robot-speed", {"r_1"}, 0.25},
  };
  for (const auto & c : cases) {
    auto func = parser::pddl::fromStringFunction(c.text);
    EXPECT_EQ(func.node_type, Node::FUNCTION) << c.text;
    EXPECT_EQ(func.name, c.name) << c.text;
    ASSERT_EQ(func.parameters.size(), c.params.size()) << c.text;
    for (size_t i = 0; i < c.params.size(); i++) {
      EXPECT_EQ(func.parameters[i].name, c.params[i]) << c.text;
    }
    EXPECT_DOUBLE_EQ(func.value, c.value) << c.text;
  }
}

TEST(StringUtils, numbers_in_scientific_notation)
{
  auto tree = fromString("(> (f) 1e-7)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_DOUBLE_EQ(tree.nodes[2].value, 1e-7);
  tree = fromString("(< (f) 2.5E+3)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_DOUBLE_EQ(tree.nodes[2].value, 2500.0);
  // Written in fixed notation, read back the same
  EXPECT_EQ(toString(fromString("(> (f) 1e12)")), "(> (f) 1000000000000.000000)");
  EXPECT_DOUBLE_EQ(fromString(toString(fromString("(> (f) 1e-7)"))).nodes[2].value, 1e-7);
}

TEST(StringUtils, numbers_do_not_depend_on_the_locale)
{
  const char * previous = std::setlocale(LC_NUMERIC, nullptr);
  const std::string saved = previous ? previous : "C";
  bool comma_locale = false;
  for (const char * name : {"es_ES.UTF-8", "de_DE.UTF-8", "fr_FR.UTF-8", "es_ES", "de_DE"}) {
    if (std::setlocale(LC_NUMERIC, name) != nullptr) {
      comma_locale = true;
      break;
    }
  }
  if (!comma_locale) {
    GTEST_SKIP() << "No locale with decimal comma installed";
  }

  auto tree = fromString("(> (battery r1) 1.5)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_DOUBLE_EQ(tree.nodes[2].value, 1.5);
  EXPECT_EQ(toString(tree), "(> (battery r1) 1.500000)");
  EXPECT_DOUBLE_EQ(parser::pddl::fromStringFunction("(= (battery r1) 2.25)").value, 2.25);
  double value = 0.0;
  EXPECT_TRUE(parser::pddl::parseNumber("0.75", value));
  EXPECT_DOUBLE_EQ(value, 0.75);
  EXPECT_FALSE(parser::pddl::parseNumber("0,75", value));

  // Also numbers in a domain
  parser::pddl::Domain domain(
    "(define (domain d) (:requirements :strips :typing :fluents)"
    "(:types robot) (:predicates (charging ?r - robot)) (:functions (battery ?r - robot))"
    "(:action charge :parameters (?r - robot)"
    " :precondition (< (battery ?r) 0.5)"
    " :effect (increase (battery ?r) 2.25)))");
  auto action = domain.actions.get("charge");
  plansys2_msgs::msg::Tree pre, eff;
  action->pre->getTree(pre, domain);
  action->eff->getTree(eff, domain);
  EXPECT_EQ(toString(pre), "(< (battery ?0) 0.500000)");
  EXPECT_EQ(toString(eff), "(increase (battery ?0) 2.250000)");

  std::setlocale(LC_NUMERIC, saved.c_str());
}

TEST(StringUtils, remove_operator_only_at_the_start)
{
  using parser::pddl::removeOperatorBeforeParenthesis;
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"and (p) (q)", "(p) (q)"},
    {"  or (p) (q)", "(p) (q)"},
    {"not (p)", "(p)"},
    {"imply (p) (q)", "(p) (q)"},
    {"> (f) 1", "(f) 1"},
    {">= (f) 1", "(f) 1"},
    {"=?a ?b", "?a ?b"},
    {"- (f) 1", "(f) 1"},
    {"-(f) 1", "(f) 1"},
    {"increase (f) 1", "(f) 1"},
    {"scale-down (f) 2", "(f) 2"},
    {"exists (?x - t) (p ?x)", "(p ?x)"},
    {"forall (?x ?y) (and (p ?x) (q ?y))", "(and (p ?x) (q ?y))"},
    {"task_assigned r1", "task_assigned r1"},
    {"android (p)", "android (p)"},
    {"(p) (q)", "(p) (q)"},
    {"", ""},
  };
  for (const auto & [input, expected] : cases) {
    std::string text = input;
    removeOperatorBeforeParenthesis(text);
    EXPECT_EQ(text, expected) << "[" << input << "]";
  }
  std::string missing = "exists ?x";
  EXPECT_THROW(removeOperatorBeforeParenthesis(missing), std::runtime_error);
}

TEST(StringUtils, sub_expressions)
{
  using parser::pddl::getSubExpr;
  EXPECT_EQ(getSubExpr("(and (p) (q r))"), std::vector<std::string>({"(p)", "(q r)"}));
  EXPECT_EQ(getSubExpr("(=?a ?b)"), std::vector<std::string>({"?a", "?b"}));
  EXPECT_EQ(getSubExpr("(> (f (g)) 2)"), std::vector<std::string>({"(f (g))", "2"}));
  EXPECT_EQ(getSubExpr("(- -1 (f))"), std::vector<std::string>({"-1", "(f)"}));
  EXPECT_TRUE(getSubExpr("").empty());
  EXPECT_TRUE(getSubExpr("x").empty());
  EXPECT_TRUE(getSubExpr("()").empty());
  EXPECT_TRUE(getSubExpr("(and)").empty());
}

TEST(StringUtils, malformed_expressions_give_an_empty_tree)
{
  for (const std::string expr : {
    "()", "( )", "(and (p) ())", "(> (f) 1.5.2)", "(> (f) abc1)", "((",
    "(p))", ")"})
  {
    auto tree = fromString(expr);
    std::string error;
    EXPECT_TRUE(parser::pddl::validateTree(tree, error)) << expr << ": " << error;
    EXPECT_GE(parser::pddl::getMaxNesting(toString(tree)), 0) << expr;
  }
  EXPECT_TRUE(fromString("()").nodes.empty());
  EXPECT_TRUE(fromString("(p))").nodes.empty());
}

TEST(StringUtils, round_trip_of_written_expressions)
{
  const std::vector<std::string> exprs = {
    "(robot_at r1 wp1)",
    "(and (robot_at r1 wp1)(robot_at r2 wp2))",
    "(or (robot_at r1 wp1)(not (robot_at r1 wp2)))",
    "(not (robot_at r1 wp1))",
    "(and (robot_at ?r ?wp)(> (battery ?r) 10.000000))",
    "(exists (?wp - waypoint) (robot_at r1 ?wp))",
    "(forall (?r - robot) (charged ?r))",
    "(exists (?a - room ?b - room) (and (connected ?a ?b)(not (= ?a ?b))))",
    "(imply (robot_at r1 wp1) (charged r1))",
    "(and (>= (battery r1) 0.500000)(<= (battery r1) 100.000000))",
    "(= ?a ?b)",
    "(not (= ?a ?b))",
    "(> (+ (f) (* 2.000000 (g))) (/ (h) 3.000000))",
    "(< (f) 0.0000001)",
  };
  for (const auto & expr : exprs) {
    expectRoundTrip(expr);
  }

  // Variables that share a type are written each with it
  EXPECT_EQ(
    toString(fromString("(exists (?a ?b - room ?r) (p ?a ?b ?r))")),
    "(exists (?a - room ?b - room ?r) (p ?a ?b ?r))");
}

TEST(StringUtils, domain_effects_are_written_as_pddl)
{
  parser::pddl::Domain domain(
    "(define (domain d) (:requirements :strips :typing :fluents)"
    "(:types robot)"
    "(:predicates (task_assigned ?r - robot) (busy ?r - robot))"
    "(:functions (battery ?r - robot) (total-cost))"
    "(:action work :parameters (?r - robot)"
    " :precondition (and (task_assigned ?r) (> (- (battery ?r) 1) 0))"
    " :effect (and (not (task_assigned ?r)) (busy ?r)"
    "  (decrease (battery ?r) (- 3 1)) (increase (total-cost) 5))))");
  auto action = domain.actions.get("work");
  plansys2_msgs::msg::Tree pre, eff;
  action->pre->getTree(pre, domain);
  action->eff->getTree(eff, domain);

  std::string error;
  EXPECT_TRUE(parser::pddl::validateTree(pre, error)) << error;
  EXPECT_TRUE(parser::pddl::validateTree(eff, error)) << error;
  EXPECT_EQ(toString(pre), "(and (task_assigned ?0)(> (- (battery ?0) 1.000000) 0.000000))");
  EXPECT_EQ(
    toString(eff),
    "(and (not (task_assigned ?0))(busy ?0)(decrease (battery ?0) (- 3.000000 1.000000))"
    "(increase (total-cost) 5.000000))");

  // The text written is read back to the same tree
  EXPECT_TRUE(parser::pddl::checkTreeEquality(fromString(toString(pre)), pre));
  EXPECT_TRUE(parser::pddl::checkTreeEquality(fromString(toString(eff)), eff));
}

TEST(StringUtils, a_bare_name_in_an_expression_is_an_object)
{
  auto tree = fromString("(= ?r r2d2)");
  ASSERT_EQ(tree.nodes.size(), 3u);
  EXPECT_EQ(tree.nodes[1].node_type, Node::PARAMETER);
  EXPECT_EQ(tree.nodes[2].node_type, Node::CONSTANT);
  EXPECT_EQ(tree.nodes[2].name, "r2d2");
  expectRoundTrip("(= ?r r2d2)");
  expectRoundTrip("(not (= kitchen ?a))");
  expectRoundTrip("(and (robot_at ?r ?a)(not (= ?a hall)))");
  // In parentheses, a function
  tree = fromString("(= (battery r1) 3)");
  EXPECT_EQ(tree.nodes[1].node_type, Node::FUNCTION);
}

TEST(StringUtils, names_are_case_insensitive)
{
  // PDDL is case insensitive (#256)
  EXPECT_TRUE(
    parser::pddl::checkNodeEquality(
      parser::pddl::fromStringPredicate("(notFilling R2D2)"),
      parser::pddl::fromStringPredicate("(notfilling r2d2)")));
  EXPECT_TRUE(
    parser::pddl::checkTreeEquality(fromString("(> (Battery R1) 2)"),
    fromString("(> (battery r1) 2)")));
  EXPECT_FALSE(
    parser::pddl::checkNodeEquality(
      parser::pddl::fromStringPredicate("(notFilling r2d2)"),
      parser::pddl::fromStringPredicate("(notfilling r2d3)")));
}

TEST(StringUtils, reading_is_fast)
{
  // No regex compiled per call: a goal of 50 facts and comparisons, read 200 times
  std::string goal = "(and";
  for (int i = 0; i < 25; i++) {
    goal += " (robot_at r" + std::to_string(i) + " wp" + std::to_string(i) + ")";
    goal += " (> (battery r" + std::to_string(i) + ") " + std::to_string(i) + ".5)";
  }
  goal += ")";
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 200; i++) {
    auto tree = fromString(goal);
    ASSERT_EQ(tree.nodes.size(), 101u);
  }
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
  EXPECT_LT(elapsed.count(), 0.4);  // ~6 ms; 0.8 s with a regex per operator
}
