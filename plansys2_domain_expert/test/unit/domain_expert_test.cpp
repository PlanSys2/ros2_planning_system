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

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"

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
  ASSERT_EQ(actions.size(), 3);
  ASSERT_EQ(actions[0], "MOVE");
  ASSERT_EQ(actions[1], "TALK");
  ASSERT_EQ(actions[2], "APPROACH");
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
  /*auto no_params = domain_expert.getActionParams("NOEXIST");
  ASSERT_FALSE(no_params.has_value());

  auto move_params = domain_expert.getActionParams("move");
  ASSERT_TRUE(move_params.has_value());
  ASSERT_EQ(move_params.value().size(), 3);
  ASSERT_EQ(move_params.value()[0], "ROBOT");
  ASSERT_EQ(move_params.value()[1], "ROOM");
  ASSERT_EQ(move_params.value()[2], "ROOM");

  auto talk_params = domain_expert.getActionParams("talk");
  ASSERT_TRUE(talk_params.has_value());
  ASSERT_EQ(talk_params.value().size(), 4);
  ASSERT_EQ(talk_params.value()[0], "ROBOT");
  ASSERT_EQ(talk_params.value()[1], "PERSON");
  ASSERT_EQ(talk_params.value()[2], "PERSON");
  ASSERT_EQ(talk_params.value()[3], "MESSAGE");*/
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
