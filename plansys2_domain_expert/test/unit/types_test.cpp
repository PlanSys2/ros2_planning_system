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

#include "gtest/gtest.h"
#include "plansys2_domain_expert/Types.hpp"

TEST(domain_types, basic_types)
{
  plansys2::Param param_1;
  param_1.name = "r2d2";
  param_1.type = "robot";

  plansys2::Param param_2;
  param_2.name = "bedroom";
  param_2.type = "room";

  plansys2::Predicate predicate_1;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(param_1);
  predicate_1.parameters.push_back(param_2);

  ASSERT_EQ(predicate_1.toString(), "(robot_at r2d2 bedroom)");
}

TEST(domain_types, predicate_tree_to_string)
{
  plansys2::Param param_1;
  param_1.name = "r2d2";
  param_1.type = "robot";

  plansys2::Param param_2;
  param_2.name = "bedroom";
  param_2.type = "room";

  plansys2::Param param_3;
  param_3.name = "kitchen";
  param_3.type = "room";

  plansys2::Param param_4;
  param_4.name = "paco";
  param_4.type = "person";

  plansys2::Predicate predicate_1;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(param_1);
  predicate_1.parameters.push_back(param_2);

  plansys2::Predicate predicate_2;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(param_1);
  predicate_2.parameters.push_back(param_3);

  plansys2::Predicate predicate_3;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(param_4);
  predicate_3.parameters.push_back(param_2);

  plansys2::Predicate predicate_4;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(param_4);
  predicate_4.parameters.push_back(param_3);

  plansys2::PredicateNode * pn_1 = new plansys2::PredicateNode();
  pn_1->predicate_ = predicate_1;

  plansys2::PredicateNode * pn_2 = new plansys2::PredicateNode();
  pn_2->predicate_ = predicate_2;

  plansys2::PredicateNode * pn_3 = new plansys2::PredicateNode();
  pn_3->predicate_ = predicate_3;

  plansys2::PredicateNode * pn_4 = new plansys2::PredicateNode();
  pn_4->predicate_ = predicate_4;

  plansys2::NotNode * pn_not = new plansys2::NotNode();
  pn_not->op = pn_2;

  plansys2::OrNode * pn_or = new plansys2::OrNode();
  pn_or->ops.push_back(pn_3);
  pn_or->ops.push_back(pn_4);

  plansys2::AndNode * pn_and = new plansys2::AndNode();
  pn_and->ops.push_back(pn_1);
  pn_and->ops.push_back(pn_not);
  pn_and->ops.push_back(pn_or);

  plansys2::PredicateTree tree;
  tree.root = pn_and;

  ASSERT_EQ(tree.toString(), std::string("(AND (robot_at r2d2 bedroom)(NOT ") +
    std::string("(robot_at r2d2 kitchen))(OR (person_at paco bedroom)(person_at paco kitchen)))"));
}

TEST(domain_types, predicate_tree_from_string)
{
  std::string expresion = std::string("(AND (robot_at r2d2 bedroom)(NOT ") +
    std::string("(robot_at r2d2 kitchen))(OR (person_at paco bedroom)(person_at paco kitchen)))");

  plansys2::PredicateTree tree;
  tree.fromString(expresion);

  ASSERT_EQ(tree.toString(), expresion);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
