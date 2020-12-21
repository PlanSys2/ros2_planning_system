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

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

TEST(problem_expert, addget_instances)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"Paco", "person"}));
  ASSERT_FALSE(problem_expert.addInstance(plansys2::Instance{"Paco", "person"}));
  ASSERT_FALSE(problem_expert.addInstance(plansys2::Instance{"Paco", "SCIENTIFIC"}));

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"r2d2", "robot"}));

  ASSERT_EQ(problem_expert.getInstances().size(), 2);
  ASSERT_EQ(problem_expert.getInstances()[0].name, "Paco");
  ASSERT_EQ(problem_expert.getInstances()[0].type, "person");
  ASSERT_EQ(problem_expert.getInstances()[1].name, "r2d2");
  ASSERT_EQ(problem_expert.getInstances()[1].type, "robot");

  ASSERT_TRUE(problem_expert.removeInstance("Paco"));
  ASSERT_EQ(problem_expert.getInstances().size(), 1);
  ASSERT_EQ(problem_expert.getInstances()[0].name, "r2d2");
  ASSERT_EQ(problem_expert.getInstances()[0].type, "robot");

  auto paco_instance = problem_expert.getInstance("Paco");
  ASSERT_FALSE(paco_instance);
  auto r2d2_instance = problem_expert.getInstance("r2d2");
  ASSERT_TRUE(r2d2_instance);
  ASSERT_EQ(r2d2_instance.value().name, "r2d2");
  ASSERT_EQ(r2d2_instance.value().type, "robot");
}

TEST(problem_expert, add_assignments)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"bedroom", "room"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"kitchen", "room_with_teleporter"}));

  plansys2::Param param_1;
  param_1.name = "bedroom";
  param_1.type = "room";

  plansys2::Param param_2;
  param_2.name = "kitchen";
  param_2.type = "room_with_teleporter";

  plansys2::Assignment assignment_1;
  assignment_1.name = "room_distance";
  assignment_1.parameters.push_back(param_1);
  assignment_1.parameters.push_back(param_2);
  assignment_1.value = 1.23;

  ASSERT_EQ(assignment_1.name, "room_distance");
  ASSERT_EQ(assignment_1.parameters.size(), 2);
  ASSERT_EQ(assignment_1.parameters[0].name, "bedroom");
  ASSERT_EQ(assignment_1.parameters[0].type, "room");
  ASSERT_EQ(assignment_1.parameters[1].name, "kitchen");
  ASSERT_EQ(assignment_1.parameters[1].type, "room_with_teleporter");
  ASSERT_EQ(assignment_1.value, 1.23);

  ASSERT_TRUE(problem_expert.addAssignment(assignment_1));

  ASSERT_EQ(
    problem_expert.getProblem(),
    "( define ( problem problem_1 )\n"
    "( :domain simple )\n"
    "( :objects\n"
    "\tbedroom - room\n"
    "\tkitchen - room_with_teleporter\n"
    ")\n"
    "( :init\n"
    "\t( = ( room_distance bedroom kitchen ) 1.23 )\n"
    ")\n"
    "( :goal\n"
    "\t( and\n"
    "\t)\n"
    ")\n"
    ")\n");

  plansys2::Assignment assignment_2;
  assignment_2.name = "room_distance";
  assignment_2.parameters.push_back(param_2);
  assignment_2.parameters.push_back(param_1);
  assignment_2.value = 2.34;

  ASSERT_EQ(assignment_2.name, "room_distance");
  ASSERT_EQ(assignment_2.parameters.size(), 2);
  ASSERT_EQ(assignment_2.parameters[0].name, "kitchen");
  ASSERT_EQ(assignment_2.parameters[0].type, "room_with_teleporter");
  ASSERT_EQ(assignment_2.parameters[1].name, "bedroom");
  ASSERT_EQ(assignment_2.parameters[1].type, "room");
  ASSERT_EQ(assignment_2.value, 2.34);

  ASSERT_TRUE(problem_expert.addAssignment(assignment_2));

  ASSERT_EQ(
    problem_expert.getProblem(),
    "( define ( problem problem_1 )\n"
    "( :domain simple )\n"
    "( :objects\n"
    "\tbedroom - room\n"
    "\tkitchen - room_with_teleporter\n"
    ")\n"
    "( :init\n"
    "\t( = ( room_distance bedroom kitchen ) 1.23 )\n"
    "\t( = ( room_distance kitchen bedroom ) 2.34 )\n"
    ")\n"
    "( :goal\n"
    "\t( and\n"
    "\t)\n"
    ")\n"
    ")\n");

  assignment_2.value = 3.45;

  ASSERT_TRUE(problem_expert.addAssignment(assignment_2));

  ASSERT_EQ(
    problem_expert.getProblem(),
    "( define ( problem problem_1 )\n"
    "( :domain simple )\n"
    "( :objects\n"
    "\tbedroom - room\n"
    "\tkitchen - room_with_teleporter\n"
    ")\n"
    "( :init\n"
    "\t( = ( room_distance bedroom kitchen ) 1.23 )\n"
    "\t( = ( room_distance kitchen bedroom ) 3.45 )\n"
    ")\n"
    "( :goal\n"
    "\t( and\n"
    "\t)\n"
    ")\n"
    ")\n");

  plansys2::Assignment assignment_3;
  assignment_3.name = "room_temperature";
  assignment_3.parameters.push_back(param_1);
  assignment_3.parameters.push_back(param_2);
  assignment_3.value = 2.34;

  ASSERT_FALSE(problem_expert.addAssignment(assignment_3));

  ASSERT_FALSE(problem_expert.removeAssignment(assignment_3));

  ASSERT_TRUE(problem_expert.removeInstance("kitchen"));
}

TEST(problem_expert, addget_predicates)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

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

  ASSERT_EQ(predicate_1.name, "robot_at");
  ASSERT_EQ(predicate_1.parameters.size(), 2);
  ASSERT_EQ(predicate_1.parameters[0].name, "r2d2");
  ASSERT_EQ(predicate_1.parameters[0].type, "robot");
  ASSERT_EQ(predicate_1.parameters[1].name, "bedroom");
  ASSERT_EQ(predicate_1.parameters[1].type, "room");

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

  ASSERT_EQ(predicate_4.name, "person_at");
  ASSERT_EQ(predicate_4.parameters.size(), 2);
  ASSERT_EQ(predicate_4.parameters[0].name, "paco");
  ASSERT_EQ(predicate_4.parameters[0].type, "person");
  ASSERT_EQ(predicate_4.parameters[1].name, "kitchen");
  ASSERT_EQ(predicate_4.parameters[1].type, "room");


  plansys2::Predicate predicate_5;
  predicate_5.name = "person_at";
  predicate_5.parameters.push_back(param_4);
  predicate_5.parameters.push_back(param_3);
  predicate_5.parameters.push_back(param_1);
  predicate_5.parameters.push_back(param_2);

  plansys2::Predicate predicate_6;
  predicate_6.name = "person_at";
  predicate_6.parameters.push_back(param_3);
  predicate_6.parameters.push_back(param_4);

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"paco", "person"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"r2d2", "robot"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"bedroom", "room"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"kitchen", "room"}));

  std::vector<plansys2::Predicate> predicates = problem_expert.getPredicates();
  ASSERT_TRUE(predicates.empty());

  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  predicates = problem_expert.getPredicates();
  ASSERT_FALSE(predicates.empty());
  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_2));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_3));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_4));
  ASSERT_FALSE(problem_expert.addPredicate(predicate_5));
  ASSERT_FALSE(problem_expert.addPredicate(predicate_6));

  predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 4);

  ASSERT_FALSE(problem_expert.removePredicate(predicate_5));
  ASSERT_TRUE(problem_expert.removePredicate(predicate_4));
  ASSERT_TRUE(problem_expert.removePredicate(predicate_4));

  predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 3);

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"bathroom", "room_with_teleporter"}));
  plansys2::Param param_5;
  param_5.name = "bathroom";
  param_5.type = "room_with_teleporter";

  plansys2::Predicate predicate_7;
  predicate_7.name = "is_teleporter_enabled";
  predicate_7.parameters.push_back(param_5);

  ASSERT_EQ(predicate_7.name, "is_teleporter_enabled");
  ASSERT_EQ(predicate_7.parameters.size(), 1);
  ASSERT_EQ(predicate_7.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_7.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_expert.addPredicate(predicate_7));

  plansys2::Predicate predicate_8;
  predicate_8.name = "is_teleporter_destination";
  predicate_8.parameters.push_back(param_5);

  ASSERT_EQ(predicate_8.name, "is_teleporter_destination");
  ASSERT_EQ(predicate_8.parameters.size(), 1);
  ASSERT_EQ(predicate_8.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_8.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_expert.addPredicate(predicate_8));

  ASSERT_TRUE(problem_expert.removeInstance("bathroom"));
}

TEST(problem_expert, addget_goals)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

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

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"paco", "person"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"r2d2", "robot"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"bedroom", "room"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"kitchen", "room"}));

  plansys2::Goal goal;
  goal.fromString("(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_EQ(goal.toString(), "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");

  plansys2::Goal goal2;
  goal2.fromString("(and (robot_at r2d2 bedroom)(not(person_at paco kitchen)))");
  ASSERT_EQ(goal2.toString(), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_expert.setGoal(goal));
  ASSERT_TRUE(problem_expert.setGoal(goal2));

  ASSERT_EQ(
    problem_expert.getGoal().toString(),
    "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  const plansys2::Goal & goal3 = problem_expert.getGoal();
  ASSERT_EQ(goal3.toString(), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_expert.clearGoal());
  ASSERT_TRUE(problem_expert.clearGoal());

  ASSERT_EQ(problem_expert.getGoal().toString(), "");

  plansys2::Goal goal4;
  goal4.fromString(
    "(and (or (robot_at r2d2 bedroom)(robot_at r2d2 kitchen))(not(person_at paco kitchen)))");
  ASSERT_TRUE(problem_expert.setGoal(goal4));
}

TEST(problem_expert, get_probem)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

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

  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"paco", "person"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"r2d2", "robot"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"bedroom", "room"}));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance{"kitchen", "room"}));

  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_2));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_3));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_4));

  plansys2::Goal goal;
  goal.fromString("(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_TRUE(problem_expert.setGoal(goal));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple ") +
    std::string(")\n( :objects\n\tpaco - person\n\tr2d2 - robot\n\tbedroom kitchen - room\n)\n") +
    std::string("( :init\n\t( robot_at r2d2 bedroom )\n\t( robot_at r2d2 kitchen )\n\t( ") +
    std::string("person_at paco bedroom )\n\t( person_at paco kitchen )\n)\n( :goal\n\t( ") +
    std::string("and\n\t\t( robot_at r2d2 bedroom )\n\t\t( person_at paco kitchen )\n\t)\n)\n)\n"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
