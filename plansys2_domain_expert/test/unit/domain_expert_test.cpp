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
#include <regex>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"


std::string getReducedString(const std::string & expr)  // Same function in Types.cpp
{
  std::regex nts_chars("[\n\t]*", std::regex_constants::ECMAScript);
  std::string ret = std::regex_replace(expr, nts_chars, "");
  std::regex open_paren("\\( ", std::regex_constants::ECMAScript);
  ret = std::regex_replace(ret, open_paren, "(");
  std::regex close_paren(" \\)", std::regex_constants::ECMAScript);
  ret = std::regex_replace(ret, close_paren, ")");
  return ret;
}

TEST(domain_expert, functions)
{
  std::string my_string("(AND)");
  ASSERT_EQ(getReducedString(my_string), "(AND)");

  my_string = "( AND)";
  ASSERT_EQ(getReducedString(my_string), "(AND)");

  my_string = "( \tAND)";
  ASSERT_EQ(getReducedString(my_string), "(AND)");

  my_string = "( \tAND\t)";
  ASSERT_EQ(getReducedString(my_string), "(AND)");

  my_string = "( AND\n)";
  ASSERT_EQ(getReducedString(my_string), "(AND)");
  my_string = "( AND\n\t)";
  ASSERT_EQ(getReducedString(my_string), "(AND)");
  my_string = "( ( AND\n\t ) )";
  ASSERT_EQ(getReducedString(my_string), "((AND))");
}


TEST(domain_expert, get_types)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<std::string> types = domain_expert.getTypes();
  std::vector<std::string> test_types {"PERSON", "MESSAGE", "ROBOT", "ROOM"};

  ASSERT_EQ(types, test_types);
}


TEST(domain_expert, get_predicates)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<std::string> predicates = domain_expert.getPredicates();
  std::vector<std::string> predicates_types {"ROBOT_TALK", "ROBOT_NEAR_PERSON", "ROBOT_AT",
    "PERSON_AT"};

  ASSERT_EQ(predicates, predicates_types);
}

TEST(domain_expert, get_predicate_params)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto params_1 = domain_expert.getPredicate("robot_talk");
  ASSERT_TRUE(params_1.has_value());
  ASSERT_EQ(params_1.value().name, "ROBOT_TALK");
  ASSERT_EQ(params_1.value().parameters.size(), 3);
  ASSERT_EQ(params_1.value().parameters[0].name, "?ROBOT0");
  ASSERT_EQ(params_1.value().parameters[0].type, "ROBOT");
  ASSERT_EQ(params_1.value().parameters[1].name, "?MESSAGE1");
  ASSERT_EQ(params_1.value().parameters[1].type, "MESSAGE");
  ASSERT_EQ(params_1.value().parameters[2].name, "?PERSON2");
  ASSERT_EQ(params_1.value().parameters[2].type, "PERSON");

  auto params_2 = domain_expert.getPredicate("robot_talkERROR");
  ASSERT_FALSE(params_2.has_value());

  auto params_3 = domain_expert.getPredicate("person_at");
  ASSERT_TRUE(params_3.has_value());
  ASSERT_EQ(params_3.value().parameters.size(), 2);
  ASSERT_EQ(params_3.value().parameters[0].name, "?PERSON0");
  ASSERT_EQ(params_3.value().parameters[0].type, "PERSON");
  ASSERT_EQ(params_3.value().parameters[1].name, "?ROOM1");
  ASSERT_EQ(params_3.value().parameters[1].type, "ROOM");
}

TEST(domain_expert, get_actions)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto actions = domain_expert.getActions();
  ASSERT_EQ(actions.size(), 1);
  ASSERT_EQ(actions[0], "MOVE_PERSON");

  auto durative_actions = domain_expert.getDurativeActions();
  ASSERT_EQ(durative_actions.size(), 3);
  ASSERT_EQ(durative_actions[0], "MOVE");
  ASSERT_EQ(durative_actions[1], "TALK");
  ASSERT_EQ(durative_actions[2], "APPROACH");
}


TEST(domain_expert, get_action_params)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  domain_expert.getAction("move");
  auto no_action = domain_expert.getAction("NOEXIST");
  ASSERT_FALSE(no_action.has_value());

  auto move_action = domain_expert.getDurativeAction("move");
  auto at_start = move_action.value().at_start_requirements;
  ASSERT_EQ(at_start.toString(), "(AND (ROBOT_AT ?0 ?1))");

  ASSERT_TRUE(move_action.has_value());
  ASSERT_EQ(move_action.value().name, "move");
  ASSERT_EQ(move_action.value().parameters.size(), 3);
  ASSERT_EQ(move_action.value().parameters[0].name, "?0");
  ASSERT_EQ(move_action.value().parameters[0].type, "ROBOT");
  ASSERT_EQ(move_action.value().parameters[1].name, "?1");
  ASSERT_EQ(move_action.value().parameters[1].type, "ROOM");
  ASSERT_EQ(move_action.value().parameters[2].name, "?2");
  ASSERT_EQ(move_action.value().parameters[2].type, "ROOM");

  ASSERT_FALSE(at_start.empty());
  ASSERT_TRUE(move_action.value().over_all_requirements.empty());
  ASSERT_TRUE(move_action.value().at_end_requirements.empty());

  ASSERT_EQ(move_action.value().at_start_requirements.toString(),
    "(AND (ROBOT_AT ?0 ?1))");
  ASSERT_EQ(move_action.value().at_start_effects.toString(),
    "(AND (NOT (ROBOT_AT ?0 ?1)))");
  ASSERT_EQ(move_action.value().at_end_effects.toString(),
    "(AND (ROBOT_AT ?0 ?2))");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
