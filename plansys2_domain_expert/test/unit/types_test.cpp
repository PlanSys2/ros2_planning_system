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

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Tree.h"

TEST(domain_types, basic_types)
{
  parser::pddl::tree::Param param_1;
  param_1.name = "r2d2";
  param_1.type = "robot";

  parser::pddl::tree::Param param_2;
  param_2.name = "bedroom";
  param_2.type = "room";

  parser::pddl::tree::Predicate predicate_1;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(param_1);
  predicate_1.parameters.push_back(param_2);

  ASSERT_EQ(predicate_1.toString(), "(robot_at r2d2 bedroom)");
}

TEST(domain_types, predicate_tree_to_string)
{
  parser::pddl::tree::Param param_1;
  param_1.name = "r2d2";
  param_1.type = "robot";

  parser::pddl::tree::Param param_2;
  param_2.name = "bedroom";
  param_2.type = "room";

  parser::pddl::tree::Param param_3;
  param_3.name = "kitchen";
  param_3.type = "room";

  parser::pddl::tree::Param param_4;
  param_4.name = "paco";
  param_4.type = "person";

  parser::pddl::tree::Predicate predicate_1;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(param_1);
  predicate_1.parameters.push_back(param_2);

  parser::pddl::tree::Predicate predicate_2;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(param_1);
  predicate_2.parameters.push_back(param_3);

  parser::pddl::tree::Predicate predicate_3;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(param_4);
  predicate_3.parameters.push_back(param_2);

  parser::pddl::tree::Predicate predicate_4;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(param_4);
  predicate_4.parameters.push_back(param_3);

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_1 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_1->predicate_ = predicate_1;

  ASSERT_EQ(pn_1->toString(), "(robot_at r2d2 bedroom)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_2 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_2->predicate_ = predicate_2;

  ASSERT_EQ(pn_2->toString(), "(robot_at r2d2 kitchen)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_3 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_3->predicate_ = predicate_3;

  ASSERT_EQ(pn_3->toString(), "(person_at paco bedroom)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_4 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_4->predicate_ = predicate_4;

  ASSERT_EQ(pn_4->toString(), "(person_at paco kitchen)");

  std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
    std::make_shared<parser::pddl::tree::NotNode>();
  pn_not->op = pn_2;

  ASSERT_EQ(pn_not->toString(), "(not (robot_at r2d2 kitchen))");

  std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
    std::make_shared<parser::pddl::tree::OrNode>();
  pn_or->ops.push_back(pn_3);
  pn_or->ops.push_back(pn_4);

  std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
    std::make_shared<parser::pddl::tree::AndNode>();
  pn_and->ops.push_back(pn_1);
  pn_and->ops.push_back(pn_not);
  pn_and->ops.push_back(pn_or);

  parser::pddl::tree::PredicateTree tree;
  tree.root_ = pn_and;

  ASSERT_EQ(
    tree.toString(), std::string("(and (robot_at r2d2 bedroom)(not ") +
    std::string("(robot_at r2d2 kitchen))(or (person_at paco bedroom)(person_at paco kitchen)))"));
}

TEST(domain_types, predicate_tree_to_string_2)
{
  parser::pddl::tree::Param param_1;
  param_1.name = "r2d2";
  param_1.type = "robot";

  parser::pddl::tree::Param param_2;
  param_2.name = "bedroom";
  param_2.type = "room";

  parser::pddl::tree::Param param_3;
  param_3.name = "kitchen";
  param_3.type = "room";

  parser::pddl::tree::Param param_4;
  param_4.name = "paco";
  param_4.type = "person";

  parser::pddl::tree::Predicate predicate_1;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(param_1);
  predicate_1.parameters.push_back(param_2);

  parser::pddl::tree::Predicate predicate_2;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(param_1);
  predicate_2.parameters.push_back(param_3);

  parser::pddl::tree::Predicate predicate_3;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(param_4);
  predicate_3.parameters.push_back(param_2);

  parser::pddl::tree::Predicate predicate_4;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(param_4);
  predicate_4.parameters.push_back(param_3);

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_1 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_1->predicate_ = predicate_1;

  ASSERT_EQ(pn_1->toString(), "(robot_at r2d2 bedroom)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_2 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_2->predicate_ = predicate_2;

  ASSERT_EQ(pn_2->toString(), "(robot_at r2d2 kitchen)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_3 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_3->predicate_ = predicate_3;

  ASSERT_EQ(pn_3->toString(), "(person_at paco bedroom)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_4 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_4->predicate_ = predicate_4;

  ASSERT_EQ(pn_4->toString(), "(person_at paco kitchen)");

  std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
    std::make_shared<parser::pddl::tree::NotNode>();
  pn_not->op = pn_2;

  ASSERT_EQ(pn_not->toString(), "(not (robot_at r2d2 kitchen))");

  std::shared_ptr<parser::pddl::tree::NotNode> pn_not_2 =
    std::make_shared<parser::pddl::tree::NotNode>();
  pn_not_2->op = pn_not;

  ASSERT_EQ(pn_not_2->toString(), "(robot_at r2d2 kitchen)");

  std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
    std::make_shared<parser::pddl::tree::OrNode>();
  pn_or->ops.push_back(pn_3);
  pn_or->ops.push_back(pn_4);

  std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
    std::make_shared<parser::pddl::tree::AndNode>();
  pn_and->ops.push_back(pn_1);
  pn_and->ops.push_back(pn_not_2);
  pn_and->ops.push_back(pn_or);

  parser::pddl::tree::PredicateTree tree;
  tree.root_ = pn_and;

  ASSERT_EQ(
    tree.toString(), std::string("(and (robot_at r2d2 bedroom)") +
    std::string("(robot_at r2d2 kitchen)(or (person_at paco bedroom)(person_at paco kitchen)))"));
}

TEST(domain_types, predicate_tree_to_string_3)
{
  parser::pddl::tree::Param param_1;
  param_1.name = "r2d2";
  param_1.type = "robot";

  parser::pddl::tree::Param param_2;
  param_2.name = "bedroom";
  param_2.type = "room";

  parser::pddl::tree::Param param_3;
  param_3.name = "kitchen";
  param_3.type = "room";

  parser::pddl::tree::Param param_4;
  param_4.name = "paco";
  param_4.type = "person";

  parser::pddl::tree::Predicate predicate_1;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(param_1);
  predicate_1.parameters.push_back(param_2);

  parser::pddl::tree::Predicate predicate_2;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(param_1);
  predicate_2.parameters.push_back(param_3);

  parser::pddl::tree::Predicate predicate_3;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(param_4);
  predicate_3.parameters.push_back(param_2);

  parser::pddl::tree::Predicate predicate_4;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(param_4);
  predicate_4.parameters.push_back(param_3);

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_1 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_1->predicate_ = predicate_1;

  ASSERT_EQ(pn_1->toString(), "(robot_at r2d2 bedroom)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_2 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_2->predicate_ = predicate_2;

  ASSERT_EQ(pn_2->toString(), "(robot_at r2d2 kitchen)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_3 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_3->predicate_ = predicate_3;

  ASSERT_EQ(pn_3->toString(), "(person_at paco bedroom)");

  std::shared_ptr<parser::pddl::tree::PredicateNode> pn_4 =
    std::make_shared<parser::pddl::tree::PredicateNode>();
  pn_4->predicate_ = predicate_4;

  ASSERT_EQ(pn_4->toString(), "(person_at paco kitchen)");

  std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
    std::make_shared<parser::pddl::tree::NotNode>();
  pn_not->op = pn_2;

  ASSERT_EQ(pn_not->toString(), "(not (robot_at r2d2 kitchen))");

  std::shared_ptr<parser::pddl::tree::NotNode> pn_not_2 =
    std::make_shared<parser::pddl::tree::NotNode>();
  pn_not_2->op = pn_not;

  ASSERT_EQ(pn_not_2->toString(), "(robot_at r2d2 kitchen)");

  std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
    std::make_shared<parser::pddl::tree::OrNode>();
  pn_or->ops.push_back(pn_3);
  pn_or->ops.push_back(pn_4);

  std::shared_ptr<parser::pddl::tree::NotNode> pn_not_3 =
    std::make_shared<parser::pddl::tree::NotNode>();
  pn_not_3->op = pn_or;

  ASSERT_EQ(
    pn_not_3->toString(),
    "(and (not (person_at paco bedroom))(not (person_at paco kitchen)))");

  std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
    std::make_shared<parser::pddl::tree::AndNode>();
  pn_and->ops.push_back(pn_1);
  pn_and->ops.push_back(pn_not_2);
  pn_and->ops.push_back(pn_not_3);

  parser::pddl::tree::PredicateTree tree;
  tree.root_ = pn_and;

  ASSERT_EQ(
    tree.toString(), std::string("(and (robot_at r2d2 bedroom)") +
    std::string("(robot_at r2d2 kitchen)(and (not (person_at paco ") +
    std::string("bedroom))(not (person_at paco kitchen))))"));
}

TEST(domain_types, predicate_tree_from_string)
{
  std::string expresion = std::string("(and (robot_at r2d2 bedroom)(not ") +
    std::string("(robot_at r2d2 kitchen))(or (person_at paco bedroom)(person_at paco kitchen)))");

  parser::pddl::tree::PredicateTree tree;
  tree.fromString(expresion);

  ASSERT_NE(tree.root_, nullptr);
  std::shared_ptr<parser::pddl::tree::AndNode> and_node =
    std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(tree.root_);
  ASSERT_EQ(and_node->type_, parser::pddl::tree::AND);
  ASSERT_FALSE(and_node->negate_);

  std::shared_ptr<parser::pddl::tree::PredicateNode> p1_node =
    std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(and_node->ops[0]);
  ASSERT_EQ(p1_node->type_, parser::pddl::tree::PREDICATE);
  ASSERT_EQ(p1_node->predicate_.name, "robot_at");
  ASSERT_EQ(p1_node->predicate_.parameters[0].name, "r2d2");
  ASSERT_EQ(p1_node->predicate_.parameters[1].name, "bedroom");
  ASSERT_FALSE(and_node->negate_);
  ASSERT_FALSE(p1_node->predicate_.negative);

  std::shared_ptr<parser::pddl::tree::NotNode> pnot_node =
    std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(and_node->ops[1]);
  ASSERT_EQ(pnot_node->type_, parser::pddl::tree::NOT);
  ASSERT_FALSE(pnot_node->negate_);

  std::shared_ptr<parser::pddl::tree::PredicateNode> p2_node =
    std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(pnot_node->op);
  ASSERT_EQ(p2_node->type_, parser::pddl::tree::PREDICATE);
  ASSERT_EQ(p2_node->predicate_.name, "robot_at");
  ASSERT_EQ(p2_node->predicate_.parameters[0].name, "r2d2");
  ASSERT_EQ(p2_node->predicate_.parameters[1].name, "kitchen");
  ASSERT_TRUE(p2_node->negate_);
  ASSERT_TRUE(p2_node->predicate_.negative);


  ASSERT_EQ(tree.toString(), expresion);

  std::string expresion2 = std::string("(and (person_at ?0 ?2)(not (person_at ?0 ?1)))");
  parser::pddl::tree::PredicateTree tree2;
  tree2.fromString(expresion2);

  ASSERT_EQ(tree2.toString(), expresion2);
}

TEST(domain_types, predicate_tree_from_string_2)
{
  std::string expresion = std::string("(not (and (robot_at r2d2 bedroom)") +
    std::string("(robot_at r2d2 kitchen)))");

  parser::pddl::tree::PredicateTree tree;
  tree.fromString(expresion);

  ASSERT_NE(tree.root_, nullptr);
  std::shared_ptr<parser::pddl::tree::NotNode> not_node =
    std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(tree.root_);
  ASSERT_NE(not_node, nullptr);
  ASSERT_EQ(not_node->type_, parser::pddl::tree::NOT);
  ASSERT_FALSE(not_node->negate_);

  std::shared_ptr<parser::pddl::tree::AndNode> and_node =
    std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(not_node->op);
  ASSERT_EQ(and_node->type_, parser::pddl::tree::AND);
  ASSERT_TRUE(and_node->negate_);

  std::shared_ptr<parser::pddl::tree::PredicateNode> p1_node =
    std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(and_node->ops[0]);
  ASSERT_EQ(p1_node->type_, parser::pddl::tree::PREDICATE);
  ASSERT_EQ(p1_node->predicate_.name, "robot_at");
  ASSERT_EQ(p1_node->predicate_.parameters[0].name, "r2d2");
  ASSERT_EQ(p1_node->predicate_.parameters[1].name, "bedroom");
  ASSERT_TRUE(and_node->negate_);
  ASSERT_TRUE(p1_node->predicate_.negative);

  std::shared_ptr<parser::pddl::tree::PredicateNode> p2_node =
    std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(and_node->ops[1]);
  ASSERT_EQ(p2_node->type_, parser::pddl::tree::PREDICATE);
  ASSERT_EQ(p2_node->predicate_.name, "robot_at");
  ASSERT_EQ(p2_node->predicate_.parameters[0].name, "r2d2");
  ASSERT_EQ(p2_node->predicate_.parameters[1].name, "kitchen");
  ASSERT_TRUE(p2_node->negate_);
  ASSERT_TRUE(p2_node->predicate_.negative);

  std::string expresion_eq = std::string("(or (not (robot_at r2d2 bedroom))") +
    std::string("(not (robot_at r2d2 kitchen)))");

  ASSERT_EQ(tree.toString(), expresion_eq);

  std::string expresion2 = std::string("(and (person_at ?0 ?2)(not (person_at ?0 ?1)))");
  parser::pddl::tree::PredicateTree tree2;
  tree2.fromString(expresion2);

  ASSERT_EQ(tree2.toString(), expresion2);
}

TEST(domain_types, predicate_tree_from_string_3)
{
  std::string expresion = std::string("(and (patrolled ro1) (patrolled ro2) (patrolled ro3))");
  parser::pddl::tree::PredicateTree tree;
  tree.fromString(expresion);

  ASSERT_EQ(tree.toString(), "(and (patrolled ro1)(patrolled ro2)(patrolled ro3))");
}

TEST(domain_types, predicate_tree_from_string_4)
{
  std::string expresion = std::string("  (  and (patrolled ro1) (patrolled ro2) (patrolled ro3))");
  parser::pddl::tree::PredicateTree tree;
  tree.fromString(expresion);

  ASSERT_EQ(tree.toString(), "(and (patrolled ro1)(patrolled ro2)(patrolled ro3))");
}

TEST(domain_types, predicate_tree_from_string_negative)
{
  std::string expresion = std::string("(and (robot_at r2d2 bedroom)(not ") +
    std::string("(robot_at r2d2 kitchen))(or (person_at paco bedroom)(person_at paco kitchen)))");

  parser::pddl::tree::PredicateTree tree;
  tree.fromString(expresion);

  ASSERT_EQ(tree.toString(), expresion);

  std::string expresion2 = std::string("(and (person_at ?0 ?2)(not (person_at ?0 ?1)))");
  parser::pddl::tree::PredicateTree tree2;
  tree2.fromString(expresion2);

  ASSERT_EQ(tree2.toString(), expresion2);
}

TEST(domain_types, get_predicates)
{
  std::string expresion_1 = std::string("(and (robot_at r2d2 bedroom)(not ") +
    std::string("(robot_at r2d2 kitchen))(or (person_at paco bedroom)(person_at paco kitchen)))");

  parser::pddl::tree::PredicateTree tree_1;
  tree_1.fromString(expresion_1);

  std::vector<parser::pddl::tree::Predicate> predicates_1;
  tree_1.getPredicates(predicates_1);

  ASSERT_EQ(4u, predicates_1.size());
  ASSERT_EQ(predicates_1[0].name, "robot_at");
  ASSERT_EQ(predicates_1[0].parameters[0].name, "r2d2");
  ASSERT_EQ(predicates_1[0].parameters[1].name, "bedroom");
  ASSERT_FALSE(predicates_1[0].negative);

  ASSERT_EQ(predicates_1[1].name, "robot_at");
  ASSERT_EQ(predicates_1[1].parameters[0].name, "r2d2");
  ASSERT_EQ(predicates_1[1].parameters[1].name, "kitchen");
  ASSERT_TRUE(predicates_1[1].negative);

  ASSERT_EQ(predicates_1[2].name, "person_at");
  ASSERT_EQ(predicates_1[2].parameters[0].name, "paco");
  ASSERT_EQ(predicates_1[2].parameters[1].name, "bedroom");
  ASSERT_FALSE(predicates_1[2].negative);

  ASSERT_EQ(predicates_1[3].name, "person_at");
  ASSERT_EQ(predicates_1[3].parameters[0].name, "paco");
  ASSERT_EQ(predicates_1[3].parameters[1].name, "kitchen");
  ASSERT_FALSE(predicates_1[3].negative);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
