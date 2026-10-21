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
#include <utility>
#include <vector>
#include <memory>

#include "plansys2_core/PackagePaths.hpp"

#include "gtest/gtest.h"

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

TEST(problem_expert, addget_instances)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "person")));
  ASSERT_FALSE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "room")));
  ASSERT_FALSE(
    problem_expert.addInstance(
      parser::pddl::fromStringParam(
        "Paco",
        "SCIENTIFIC")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("ur5e", "Robot")));

  ASSERT_EQ(problem_expert.getInstances().size(), 3);
  ASSERT_EQ(problem_expert.getInstances()[0].name, "Paco");
  ASSERT_EQ(problem_expert.getInstances()[0].type, "person");
  ASSERT_EQ(problem_expert.getInstances()[1].name, "r2d2");
  ASSERT_EQ(problem_expert.getInstances()[1].type, "robot");
  ASSERT_EQ(problem_expert.getInstances()[2].name, "ur5e");
  ASSERT_EQ(problem_expert.getInstances()[2].type, "robot");

  ASSERT_TRUE(problem_expert.removeInstance(parser::pddl::fromStringParam("Paco", "person")));
  ASSERT_EQ(problem_expert.getInstances().size(), 2);
  ASSERT_EQ(problem_expert.getInstances()[0].name, "r2d2");
  ASSERT_EQ(problem_expert.getInstances()[0].type, "robot");
  ASSERT_EQ(problem_expert.getInstances()[1].name, "ur5e");
  ASSERT_EQ(problem_expert.getInstances()[1].type, "robot");

  auto paco_instance = problem_expert.getInstance("Paco");
  ASSERT_FALSE(paco_instance);
  auto r2d2_instance = problem_expert.getInstance("r2d2");
  ASSERT_TRUE(r2d2_instance);
  ASSERT_EQ(r2d2_instance.value().name, "r2d2");
  ASSERT_EQ(r2d2_instance.value().type, "robot");
  auto ur5e_instance = problem_expert.getInstance("ur5e");
  ASSERT_TRUE(ur5e_instance);
  ASSERT_EQ(ur5e_instance.value().name, "ur5e");
  ASSERT_EQ(ur5e_instance.value().type, "robot");
}

TEST(problem_expert, add_functions)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(
    problem_expert.addInstance(
      parser::pddl::fromStringParam(
        "kitchen",
        "room_with_teleporter")));

  plansys2_msgs::msg::Node function_1;
  function_1.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_1.name = "room_distance";
  function_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_1.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_1.value = 1.23;

  ASSERT_EQ(function_1.name, "room_distance");
  ASSERT_EQ(function_1.parameters.size(), 2);
  ASSERT_EQ(function_1.parameters[0].name, "bedroom");
  ASSERT_EQ(function_1.parameters[0].type, "room");
  ASSERT_EQ(function_1.parameters[1].name, "kitchen");
  ASSERT_EQ(function_1.parameters[1].type, "room_with_teleporter");
  ASSERT_EQ(function_1.value, 1.23);

  ASSERT_TRUE(problem_expert.addFunction(function_1));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tbedroom - room\n\tkitchen - room_with_teleporter\n)\n") +
    std::string("( :init\n\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t))\n)\n")
  );

  plansys2_msgs::msg::Node function_2;
  function_2.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_2.name = "room_distance";
  function_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_2.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_2.value = 2.34;

  ASSERT_EQ(function_2.name, "room_distance");
  ASSERT_EQ(function_2.parameters.size(), 2);
  ASSERT_EQ(function_2.parameters[0].name, "kitchen");
  ASSERT_EQ(function_2.parameters[0].type, "room_with_teleporter");
  ASSERT_EQ(function_2.parameters[1].name, "bedroom");
  ASSERT_EQ(function_2.parameters[1].type, "room");
  ASSERT_EQ(function_2.value, 2.34);

  ASSERT_TRUE(problem_expert.addFunction(function_2));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tbedroom - room\n\tkitchen - room_with_teleporter\n)\n") +
    std::string("( :init\n\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n") +
    std::string("\t( = ( room_distance kitchen bedroom ) 2.3400000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t))\n)\n"));

  function_2.value = 3.45;

  ASSERT_TRUE(problem_expert.addFunction(function_2));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tbedroom - room\n\tkitchen - room_with_teleporter\n)\n") +
    std::string("( :init\n\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n") +
    std::string("\t( = ( room_distance kitchen bedroom ) 3.4500000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t))\n)\n"));

  plansys2_msgs::msg::Node function_3;
  function_3.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_3.name = "room_temperature";
  function_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_3.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_3.value = 2.34;

  ASSERT_FALSE(problem_expert.addFunction(function_3));

  ASSERT_FALSE(problem_expert.removeFunction(function_3));

  ASSERT_TRUE(
    problem_expert.removeInstance(
      parser::pddl::fromStringParam(
        "kitchen",
        "room_with_teleporter")));
}

TEST(problem_expert, addget_predicates)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Node predicate_1;
  predicate_1.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  ASSERT_EQ(predicate_1.name, "robot_at");
  ASSERT_EQ(predicate_1.parameters.size(), 2);
  ASSERT_EQ(predicate_1.parameters[0].name, "r2d2");
  ASSERT_EQ(predicate_1.parameters[0].type, "robot");
  ASSERT_EQ(predicate_1.parameters[1].name, "bedroom");
  ASSERT_EQ(predicate_1.parameters[1].type, "room");

  plansys2_msgs::msg::Node predicate_2;
  predicate_2.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  plansys2_msgs::msg::Node predicate_3;
  predicate_3.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_4;
  predicate_4.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  ASSERT_EQ(predicate_4.name, "person_at");
  ASSERT_EQ(predicate_4.parameters.size(), 2);
  ASSERT_EQ(predicate_4.parameters[0].name, "paco");
  ASSERT_EQ(predicate_4.parameters[0].type, "person");
  ASSERT_EQ(predicate_4.parameters[1].name, "kitchen");
  ASSERT_EQ(predicate_4.parameters[1].type, "room");


  plansys2_msgs::msg::Node predicate_5;
  predicate_5.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_5.name = "person_at";
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_6;
  predicate_6.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_6.name = "person_at";
  predicate_6.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));
  predicate_6.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

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

  auto pred_2 = problem_expert.getPredicate("(robot_at r2d2 kitchen)");
  ASSERT_TRUE(pred_2);
  ASSERT_EQ(pred_2.value().name, "robot_at");
  ASSERT_EQ(pred_2.value().parameters.size(), 2);
  ASSERT_EQ(pred_2.value().parameters[0].name, "r2d2");
  ASSERT_EQ(pred_2.value().parameters[0].type, "robot");
  ASSERT_EQ(pred_2.value().parameters[1].name, "kitchen");
  ASSERT_EQ(pred_2.value().parameters[1].type, "room");

  ASSERT_FALSE(problem_expert.removePredicate(predicate_5));
  ASSERT_TRUE(problem_expert.removePredicate(predicate_4));
  ASSERT_TRUE(problem_expert.removePredicate(predicate_4));

  predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 3);

  ASSERT_TRUE(
    problem_expert.addInstance(
      parser::pddl::fromStringParam(
        "bathroom",
        "room_with_teleporter")));

  plansys2_msgs::msg::Node predicate_7;
  predicate_7.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_7.name = "is_teleporter_enabled";
  predicate_7.parameters.push_back(
    parser::pddl::fromStringParam(
      "bathroom",
      "room_with_teleporter"));

  ASSERT_EQ(predicate_7.name, "is_teleporter_enabled");
  ASSERT_EQ(predicate_7.parameters.size(), 1);
  ASSERT_EQ(predicate_7.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_7.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_expert.addPredicate(predicate_7));

  plansys2_msgs::msg::Node predicate_8;
  predicate_8.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_8.name = "is_teleporter_destination";
  predicate_8.parameters.push_back(
    parser::pddl::fromStringParam(
      "bathroom",
      "room_with_teleporter"));

  ASSERT_EQ(predicate_8.name, "is_teleporter_destination");
  ASSERT_EQ(predicate_8.parameters.size(), 1);
  ASSERT_EQ(predicate_8.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_8.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_expert.addPredicate(predicate_8));

  ASSERT_TRUE(
    problem_expert.removeInstance(
      parser::pddl::fromStringParam(
        "bathroom",
        "room_with_teleporter")));
}

TEST(problem_expert, addget_functions)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Node function_1;
  function_1.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_1.name = "speed";
  function_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  function_1.value = 3;

  ASSERT_EQ(function_1.name, "speed");
  ASSERT_EQ(function_1.parameters.size(), 1);
  ASSERT_EQ(function_1.parameters[0].name, "r2d2");
  ASSERT_EQ(function_1.parameters[0].type, "robot");
  ASSERT_EQ(function_1.value, 3);

  plansys2_msgs::msg::Node function_2;
  function_2.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_2.name = "distance";
  function_2.parameters.push_back(parser::pddl::fromStringParam("wp1", "waypoint"));
  function_2.parameters.push_back(parser::pddl::fromStringParam("wp2", "waypoint"));
  function_2.value = 15;

  ASSERT_EQ(function_2.name, "distance");
  ASSERT_EQ(function_2.parameters.size(), 2);
  ASSERT_EQ(function_2.parameters[0].name, "wp1");
  ASSERT_EQ(function_2.parameters[0].type, "waypoint");
  ASSERT_EQ(function_2.parameters[1].name, "wp2");
  ASSERT_EQ(function_2.parameters[1].type, "waypoint");
  ASSERT_EQ(function_2.value, 15);

  plansys2_msgs::msg::Node function_3;
  function_3.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_3.name = "speed";
  function_3.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  function_3.parameters.push_back(parser::pddl::fromStringParam("wp1", "waypoint"));

  plansys2_msgs::msg::Node function_4;
  function_4.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_4.name = "distance";
  function_4.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  function_4.parameters.push_back(parser::pddl::fromStringParam("wp1", "waypoint"));

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("wp1", "waypoint")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("wp2", "waypoint")));

  std::vector<plansys2::Function> functions = problem_expert.getFunctions();
  ASSERT_TRUE(functions.empty());

  ASSERT_TRUE(problem_expert.addFunction(function_1));
  functions = problem_expert.getFunctions();
  ASSERT_FALSE(functions.empty());
  ASSERT_TRUE(problem_expert.addFunction(function_1));
  ASSERT_TRUE(problem_expert.addFunction(function_2));
  ASSERT_FALSE(problem_expert.addFunction(function_3));
  ASSERT_FALSE(problem_expert.addFunction(function_4));

  functions = problem_expert.getFunctions();
  ASSERT_EQ(functions.size(), 2);

  auto func_2 = problem_expert.getFunction("(distance wp1 wp2)");
  ASSERT_TRUE(func_2);
  ASSERT_EQ(func_2.value().name, "distance");
  ASSERT_EQ(func_2.value().parameters.size(), 2);
  ASSERT_EQ(func_2.value().parameters[0].name, "wp1");
  ASSERT_EQ(func_2.value().parameters[0].type, "waypoint");
  ASSERT_EQ(func_2.value().parameters[1].name, "wp2");
  ASSERT_EQ(func_2.value().parameters[1].type, "waypoint");
  ASSERT_EQ(func_2.value().value, 15);

  ASSERT_FALSE(problem_expert.removeFunction(function_3));
  ASSERT_TRUE(problem_expert.removeFunction(function_2));

  functions = problem_expert.getFunctions();
  ASSERT_EQ(functions.size(), 1);
}

TEST(problem_expert, addget_goals)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_EQ(parser::pddl::toString(goal), "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");

  plansys2_msgs::msg::Tree goal2;
  parser::pddl::fromString(goal2, "(and (robot_at r2d2 bedroom)(not(person_at paco kitchen)))");
  ASSERT_EQ(
    parser::pddl::toString(
      goal2), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_expert.setGoal(goal));
  ASSERT_TRUE(problem_expert.setGoal(goal2));

  ASSERT_EQ(
    parser::pddl::toString(problem_expert.getGoal()),
    "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  const plansys2_msgs::msg::Tree & goal3 = problem_expert.getGoal();
  ASSERT_EQ(
    parser::pddl::toString(
      goal3), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_expert.clearGoal());
  ASSERT_TRUE(problem_expert.clearGoal());

  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "");

  plansys2_msgs::msg::Tree goal4;
  parser::pddl::fromString(
    goal4,
    "(and (or (robot_at r2d2 bedroom) (robot_at r2d2 kitchen)) (not (person_at paco kitchen)))");
  ASSERT_TRUE(problem_expert.setGoal(goal4));
}

TEST(problem_expert, empty_goals)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Tree goal;
  ASSERT_FALSE(problem_expert.isValidGoal(goal));
}

TEST(problem_expert, get_problem)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Node predicate_1;
  predicate_1.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_2;
  predicate_2.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  plansys2_msgs::msg::Node predicate_3;
  predicate_3.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_4;
  predicate_4.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_2));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_3));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_4));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_TRUE(problem_expert.setGoal(goal));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tpaco - person\n\tr2d2 - robot\n") +
    std::string("\tbedroom kitchen - room\n)\n") +
    std::string("( :init\n\t( robot_at r2d2 bedroom )\n") +
    std::string("\t( robot_at r2d2 kitchen )\n") +
    std::string("\t( person_at paco bedroom )\n") +
    std::string("\t( person_at paco kitchen )\n)\n") +
    std::string("( :goal\n\t( and\n\t\t( robot_at r2d2 bedroom )\n\t\t") +
    std::string("( person_at paco kitchen )\n\t))\n)\n"));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_EQ(problem_expert.getPredicates().size(), 0);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);
  ASSERT_EQ(problem_expert.getInstances().size(), 0);
}

TEST(problem_expert, add_problem)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_FALSE(problem_expert.addProblem(""));

  // Empty domain name
  std::ifstream problem_empty_domain_ifs(pkgpath + "/pddl/problem_empty_domain.pddl");
  std::string problem_empty_domain_str((
      std::istreambuf_iterator<char>(problem_empty_domain_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_FALSE(problem_expert.addProblem(problem_empty_domain_str));

  // Domain doesn't exist
  std::ifstream problem_charging_ifs(pkgpath + "/pddl/problem_charging.pddl");
  std::string problem_charging_str((
      std::istreambuf_iterator<char>(problem_charging_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_FALSE(problem_expert.addProblem(problem_charging_str));

  // missing syntax causes std::runtime_error
  std::ifstream problem_unexpected_syntax_ifs(pkgpath + "/pddl/problem_unexpected_syntax.pddl");
  std::string problem_unexpected_syntax_str((
      std::istreambuf_iterator<char>(problem_unexpected_syntax_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_FALSE(problem_expert.addProblem(problem_unexpected_syntax_str));

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_str));

  ASSERT_TRUE(problem_expert.isValidType("robot"));
  ASSERT_TRUE(problem_expert.isValidType("person"));
  ASSERT_TRUE(problem_expert.isValidType("room"));
  ASSERT_TRUE(problem_expert.isValidType("room_with_teleporter"));
  ASSERT_TRUE(problem_expert.isValidType("message"));
  ASSERT_TRUE(problem_expert.isValidType("ROBOT"));
  ASSERT_TRUE(problem_expert.isValidType("Person"));
  ASSERT_TRUE(problem_expert.isValidType("ROOM"));
  ASSERT_TRUE(problem_expert.isValidType("ROOM_with_TELEPORTER"));
  ASSERT_TRUE(problem_expert.isValidType("Message"));

  ASSERT_EQ(problem_expert.getInstances().size(), 6);
  ASSERT_EQ(problem_expert.getPredicates().size(), 2);
  ASSERT_EQ(problem_expert.getFunctions().size(), 1);

  ASSERT_TRUE(problem_expert.existInstance("leia"));
  ASSERT_TRUE(problem_expert.existInstance("jack"));
  ASSERT_TRUE(problem_expert.existInstance("kitchen"));
  ASSERT_TRUE(problem_expert.existInstance("bedroom"));
  ASSERT_TRUE(problem_expert.existInstance("m1"));

  ASSERT_FALSE(problem_expert.existInstance("r2d2"));
  ASSERT_FALSE(problem_expert.existInstance("hallway"));
  ASSERT_FALSE(problem_expert.existInstance("m2"));

  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(person_at jack bedroom)")));

  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "(and (robot_talk leia m1 jack))");

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n") +
    std::string("( :domain simple )\n( :objects\n") +
    std::string("\tjack alice - person\n") +
    std::string("\tm1 - message\n") +
    std::string("\tleia - robot\n") +
    std::string("\tkitchen bedroom - room\n)\n") +
    std::string("( :init\n\t( robot_at leia kitchen )\n") +
    std::string("\t( person_at jack bedroom )\n") +
    std::string("\t( = ( room_distance kitchen bedroom ) 10.0000000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t\t( robot_talk leia m1 jack )\n\t))\n)\n"));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_EQ(problem_expert.getPredicates().size(), 0);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);
  ASSERT_EQ(problem_expert.getInstances().size(), 0);
}


TEST(problem_expert, add_problem_with_constants)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  std::ifstream problem_1_ifs(pkgpath + "/pddl/problem_simple_constants_1.pddl");
  std::string problem_1_str((
      std::istreambuf_iterator<char>(problem_1_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_1_str));

  ASSERT_TRUE(problem_expert.isValidType("robot"));
  ASSERT_TRUE(problem_expert.isValidType("person"));
  ASSERT_TRUE(problem_expert.isValidType("room"));
  ASSERT_TRUE(problem_expert.isValidType("teleporter_room"));
  ASSERT_TRUE(problem_expert.isValidType("message"));

  ASSERT_EQ(problem_expert.getInstances().size(), 7);
  ASSERT_EQ(problem_expert.getPredicates().size(), 2);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);

  ASSERT_TRUE(problem_expert.existInstance("leia"));
  ASSERT_TRUE(problem_expert.existInstance("lema"));
  ASSERT_TRUE(problem_expert.existInstance("jack"));
  ASSERT_TRUE(problem_expert.existInstance("john"));
  ASSERT_TRUE(problem_expert.existInstance("kitchen"));
  ASSERT_TRUE(problem_expert.existInstance("bedroom"));
  ASSERT_TRUE(problem_expert.existInstance("m1"));

  ASSERT_FALSE(problem_expert.existInstance("r2d2"));
  ASSERT_FALSE(problem_expert.existInstance("hallway"));
  ASSERT_FALSE(problem_expert.existInstance("m2"));

  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(person_at jack bedroom)")));

  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "(and (robot_talk leia m1 jack))");

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain plansys2 )\n") +
    std::string("( :objects\n\tm1 - message\n\tkitchen bedroom - room\n)\n") +
    std::string("( :init\n\t( robot_at leia kitchen )\n") +
    std::string("\t( person_at jack bedroom )\n)\n") +
    std::string("( :goal\n\t( and\n\t\t( robot_talk leia m1 jack )\n\t))\n)\n"));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_EQ(problem_expert.getPredicates().size(), 0);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);
  ASSERT_EQ(problem_expert.getInstances().size(), 0);
  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain plansys2 )\n") +
    std::string("( :objects\n)\n( :init\n)\n( :goal\n\t( and\n\t))\n)\n"));


  std::ifstream problem_2_ifs(pkgpath + "/pddl/problem_simple_constants_2.pddl");
  std::string problem_2_str((
      std::istreambuf_iterator<char>(problem_2_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_2_str));

  ASSERT_NE(problem_1_str, problem_2_str);
  ASSERT_NE(problem_expert.getProblem(), problem_2_str);
  ASSERT_EQ(problem_expert.getProblem(), problem_1_str);
}

TEST(problem_expert, is_goal_satisfied)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("leia", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("Jack", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("m1", "message")));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(person_at Jack bedroom)")));

  std::string expression = "(and (robot_talk leia m1 Jack))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(parser::pddl::toString(goal), "(and (robot_talk leia m1 Jack))");
  ASSERT_TRUE(problem_expert.setGoal(goal));
  ASSERT_FALSE(problem_expert.isGoalSatisfied(goal));

  ASSERT_TRUE(
    problem_expert.addPredicate(
      parser::pddl::fromStringPredicate("(robot_talk leia m1 Jack)")));

  ASSERT_TRUE(problem_expert.isGoalSatisfied(goal));
}

TEST(problem_expert, exist_predicate)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_derived.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_str));

  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(inferred-robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(person_at jack bedroom)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(inferred-person_at jack bedroom)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(inferred-person_at jack kitchen)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(inferred-robot_at leia bedroom)")));

  problem_expert.removePredicate(
    parser::pddl::fromStringPredicate("(robot_at leia kitchen)"));
  problem_expert.removePredicate(
    parser::pddl::fromStringPredicate("(person_at jack bedroom)"));

  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(inferred-person_at jack bedroom)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(inferred-robot_at leia kitchen)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(person_at jack bedroom)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate(
        "(robot_at leia kitchen)")));
}

TEST(problem_expert, get_predicate_with_derived)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_derived.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_str));

  auto predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 4);
  std::vector<std::string> predicates_names;
  std::for_each(
    predicates.begin(), predicates.end(),
    [&](auto p) {predicates_names.push_back(parser::pddl::toString(p));}
  );
  ASSERT_TRUE(
    std::find(
      predicates_names.begin(), predicates_names.end(), "(inferred-robot_at leia kitchen)") !=
    predicates_names.end());
  ASSERT_TRUE(
    std::find(
      predicates_names.begin(), predicates_names.end(), "(person_at jack bedroom)") !=
    predicates_names.end());
  ASSERT_TRUE(
    std::find(
      predicates_names.begin(), predicates_names.end(), "(inferred-person_at jack bedroom)") !=
    predicates_names.end());
  ASSERT_FALSE(
    std::find(
      predicates_names.begin(), predicates_names.end(), "(inferred-person_at jack kitchen)") !=
    predicates_names.end());
  ASSERT_FALSE(
    std::find(
      predicates_names.begin(), predicates_names.end(), "(inferred-robot_at leia bedroom)") !=
    predicates_names.end());

  problem_expert.removePredicate(
    parser::pddl::fromStringPredicate("(robot_at leia kitchen)"));
  problem_expert.removePredicate(
    parser::pddl::fromStringPredicate("(person_at jack bedroom)"));

  predicates = problem_expert.getPredicates();
  std::vector<std::string> predicate_names2;
  std::for_each(
    predicates.begin(), predicates.end(),
    [&](auto p) {predicate_names2.push_back(parser::pddl::toString(p));}
  );

  ASSERT_FALSE(
    std::find(
      predicate_names2.begin(), predicate_names2.end(), "(inferred-person_at jack bedroom)") !=
    predicate_names2.end());
  ASSERT_FALSE(
    std::find(
      predicate_names2.begin(), predicate_names2.end(), "(inferred-robot_at leia kitchen)") !=
    predicate_names2.end());
}

namespace
{
std::string read_test_pddl(const std::string & filename)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream ifs(pkgpath + "/pddl/" + filename);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}
}  // namespace

// update_domain: hot domain swap. See refactor_dynamic_domain.md for the design.
// These tests use domain_simple.pddl as the starting domain, plus purpose-built
// variants under test/pddl/ that each change exactly one thing, to check that
// knowledge unaffected by that change survives untouched.

TEST(problem_expert, update_domain_add_requirement_preserves_everything)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));
  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(person_at paco kitchen)")));

  plansys2_msgs::msg::Node function_1;
  function_1.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_1.name = "room_distance";
  function_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_1.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));
  function_1.value = 5;
  ASSERT_TRUE(problem_expert.addFunction(function_1));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 kitchen))");
  ASSERT_TRUE(problem_expert.setGoal(goal));

  bool result = problem_expert.updateDomain(
    read_test_pddl("domain_simple_v2_extra_requirement.pddl"));

  ASSERT_TRUE(result);

  ASSERT_EQ(problem_expert.getInstances().size(), 4u);
  ASSERT_EQ(problem_expert.getPredicates().size(), 2u);
  ASSERT_EQ(problem_expert.getFunctions().size(), 1u);
  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "(and (robot_at r2d2 kitchen))");
}

TEST(problem_expert, update_domain_removes_only_affected_predicate)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));
  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(person_at paco bedroom)")));
  ASSERT_TRUE(
    problem_expert.addPredicate(
      parser::pddl::fromStringPredicate("(is_teleporter_destination bedroom)")));

  bool result = problem_expert.updateDomain(
    read_test_pddl("domain_simple_v3_removed_predicate.pddl"));

  ASSERT_TRUE(result);

  auto predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 2u);
  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate("(person_at paco bedroom)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate("(is_teleporter_destination bedroom)")));
  // Instances are untouched by a predicate-only domain change.
  ASSERT_EQ(problem_expert.getInstances().size(), 3u);
}

TEST(problem_expert, update_domain_clears_goal_that_uses_removed_predicate)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (is_teleporter_destination bedroom))");
  ASSERT_TRUE(problem_expert.setGoal(goal));

  bool result = problem_expert.updateDomain(
    read_test_pddl("domain_simple_v3_removed_predicate.pddl"));

  ASSERT_TRUE(result);
  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "");
}

TEST(problem_expert, update_domain_removes_only_affected_function)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));

  plansys2_msgs::msg::Node room_distance;
  room_distance.node_type = plansys2_msgs::msg::Node::FUNCTION;
  room_distance.name = "room_distance";
  room_distance.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  room_distance.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));
  room_distance.value = 5;
  ASSERT_TRUE(problem_expert.addFunction(room_distance));

  bool result = problem_expert.updateDomain(
    read_test_pddl("domain_simple_v4_removed_function.pddl"));

  ASSERT_TRUE(result);
  ASSERT_TRUE(problem_expert.getFunctions().empty());
  // Predicates/instances are untouched by a function-only domain change.
  ASSERT_EQ(problem_expert.getPredicates().size(), 1u);
  ASSERT_EQ(problem_expert.getInstances().size(), 3u);
}

TEST(problem_expert, update_domain_removed_type_cascades_to_instances_and_predicates)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("hello", "message")));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));
  ASSERT_TRUE(
    problem_expert.addPredicate(
      parser::pddl::fromStringPredicate("(robot_talk r2d2 hello paco)")));

  bool result = problem_expert.updateDomain(
    read_test_pddl("domain_simple_v5_removed_type.pddl"));

  ASSERT_TRUE(result);
  // robot_talk is gone too, cascaded from removing the "hello" (message) instance,
  // even though it's not directly a signature mismatch against the new domain.
  ASSERT_FALSE(
    problem_expert.existPredicate(
      parser::pddl::fromStringPredicate("(robot_talk r2d2 hello paco)")));

  ASSERT_EQ(problem_expert.getInstances().size(), 3u);
  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));
  ASSERT_FALSE(problem_expert.existInstance("hello"));
}

TEST(problem_expert, update_domain_full_replacement_prunes_everything)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 bedroom))");
  ASSERT_TRUE(problem_expert.setGoal(goal));

  bool result = problem_expert.updateDomain(read_test_pddl("domain_totally_different.pddl"));

  ASSERT_TRUE(result);

  ASSERT_TRUE(problem_expert.getInstances().empty());
  ASSERT_TRUE(problem_expert.getPredicates().empty());
  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "");
}

TEST(problem_expert, update_domain_rejects_invalid_pddl_and_keeps_everything)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 bedroom)")));

  bool result = problem_expert.updateDomain(
    "(define (domain broken) (:predicates (foo ?a - undeclared_type)))");

  ASSERT_FALSE(result);

  ASSERT_EQ(problem_expert.getInstances().size(), 2u);
  ASSERT_EQ(problem_expert.getPredicates().size(), 1u);
}

// Metric: what the planner optimizes. domain_simple has (room_distance ?r1 ?r2 - room)
namespace
{
plansys2::ProblemExpert make_metric_problem(std::shared_ptr<plansys2::DomainExpert> & domain)
{
  domain = std::make_shared<plansys2::DomainExpert>(read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain);
  problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot"));
  problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room"));
  problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room"));
  return problem_expert;
}
}  // namespace

TEST(problem_expert, metric_set_get_clear)
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  auto problem_expert = make_metric_problem(domain);

  // No metric at the start
  ASSERT_TRUE(problem_expert.getMetric().tree.nodes.empty());
  ASSERT_EQ(parser::pddl::toString(problem_expert.getMetric()), "");

  ASSERT_TRUE(problem_expert.setMetric(plansys2::Metric("minimize (total-time)")));
  ASSERT_EQ(parser::pddl::toString(problem_expert.getMetric()), "minimize (total-time)");

  ASSERT_TRUE(
    problem_expert.setMetric(plansys2::Metric("minimize (room_distance bedroom kitchen)")));
  ASSERT_EQ(
    parser::pddl::toString(problem_expert.getMetric()),
    "minimize (room_distance bedroom kitchen)");

  // A new metric replaces the old one
  ASSERT_TRUE(
    problem_expert.setMetric(
      plansys2::Metric("maximize (+ (room_distance bedroom kitchen) (* 2 (total-time)))")));
  auto metric = problem_expert.getMetric();
  ASSERT_EQ(metric.optimization, plansys2_msgs::msg::Metric::MAXIMIZE);
  ASSERT_EQ(
    parser::pddl::toString(metric),
    "maximize (+ (room_distance bedroom kitchen) (* 2.000000 (total-time)))");

  // A plain number is a valid, if useless, metric
  ASSERT_TRUE(problem_expert.setMetric(plansys2::Metric("minimize 0")));

  ASSERT_TRUE(problem_expert.clearMetric());
  ASSERT_TRUE(problem_expert.getMetric().tree.nodes.empty());
  // Clearing twice is fine
  ASSERT_TRUE(problem_expert.clearMetric());
}

TEST(problem_expert, metric_invalid_is_rejected_and_keeps_the_previous)
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  auto problem_expert = make_metric_problem(domain);

  const std::string valid = "minimize (room_distance bedroom kitchen)";
  ASSERT_TRUE(problem_expert.setMetric(plansys2::Metric(valid)));

  const std::vector<std::string> invalid = {
    "",                                                // nothing
    "minimize",                                        // no expression
    "optimize (total-time)",                           // unknown optimization
    "minimize (battery r2d2)",                         // function not in the domain
    "minimize (room_distance r2d2 kitchen)",           // argument of the wrong type
    "minimize (room_distance bedroom hall)",           // unknown instance
    "minimize (room_distance bedroom)",                // wrong arity
    "minimize (> (room_distance bedroom kitchen) 3)",  // a comparison, not a number
    "minimize (robot_at r2d2 bedroom)",                // a predicate has no value
    "minimize (and (total-time))",                     // logic is not arithmetic
    "minimize (+ (total-time) (total-time)",           // unbalanced
  };
  for (const auto & m : invalid) {
    EXPECT_FALSE(problem_expert.setMetric(plansys2::Metric(m))) << m;
    EXPECT_EQ(parser::pddl::toString(problem_expert.getMetric()), valid) << m;
  }

  // Built by hand: an optimization that does not exist, and a broken tree
  plansys2::Metric bad_optimization("minimize (total-time)");
  bad_optimization.optimization = 7;
  ASSERT_FALSE(problem_expert.setMetric(bad_optimization));

  plansys2::Metric broken_tree("minimize (+ (total-time) 1)");
  broken_tree.tree.nodes[0].children.push_back(42);
  ASSERT_FALSE(problem_expert.setMetric(broken_tree));

  ASSERT_FALSE(problem_expert.isValidMetric(plansys2::Metric()));
  ASSERT_EQ(parser::pddl::toString(problem_expert.getMetric()), valid);
}

TEST(problem_expert, metric_in_get_problem)
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  auto problem_expert = make_metric_problem(domain);
  ASSERT_TRUE(
    problem_expert.setGoal(plansys2::Goal("(and (robot_at r2d2 kitchen))")));

  // Without a metric the problem has no :metric section
  ASSERT_EQ(problem_expert.getProblem().find(":metric"), std::string::npos);

  ASSERT_TRUE(
    problem_expert.setMetric(
      plansys2::Metric("maximize (- (room_distance bedroom kitchen) (* 2 (total-time)))")));
  const std::string problem = problem_expert.getProblem();
  const std::string expected =
    "( :metric maximize ( - ( room_distance bedroom kitchen ) "
    "( * 2 ( total-time ) ) ) )\n)\n";
  ASSERT_GE(problem.size(), expected.size());
  ASSERT_EQ(problem.substr(problem.size() - expected.size()), expected);

  // The problem PlanSys2 builds can be read back, metric included
  auto domain2 = std::make_shared<plansys2::DomainExpert>(read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert2(domain2);
  ASSERT_TRUE(problem_expert2.addProblem(problem));
  ASSERT_EQ(
    parser::pddl::toString(problem_expert2.getMetric()),
    parser::pddl::toString(problem_expert.getMetric()));

  ASSERT_TRUE(problem_expert.clearMetric());
  ASSERT_EQ(problem_expert.getProblem().find(":metric"), std::string::npos);
}

TEST(problem_expert, metric_cleared_with_the_knowledge)
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  auto problem_expert = make_metric_problem(domain);
  ASSERT_TRUE(problem_expert.setMetric(plansys2::Metric("minimize (total-time)")));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_TRUE(problem_expert.getMetric().tree.nodes.empty());
}

TEST(problem_expert, metric_dropped_only_when_an_instance_it_uses_is_removed)
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  auto problem_expert = make_metric_problem(domain);

  ASSERT_TRUE(
    problem_expert.setMetric(
      plansys2::Metric("minimize (+ (room_distance bedroom kitchen) (total-time))")));

  // r2d2 is not in the metric: it stays
  ASSERT_TRUE(problem_expert.removeInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_FALSE(problem_expert.getMetric().tree.nodes.empty());

  // kitchen is: the whole metric goes, not just its term
  ASSERT_TRUE(problem_expert.removeInstance(parser::pddl::fromStringParam("kitchen", "room")));
  ASSERT_TRUE(problem_expert.getMetric().tree.nodes.empty());

  // A metric on total-time uses no instance
  ASSERT_TRUE(problem_expert.setMetric(plansys2::Metric("minimize (total-time)")));
  ASSERT_TRUE(problem_expert.removeInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_EQ(parser::pddl::toString(problem_expert.getMetric()), "minimize (total-time)");
}

TEST(problem_expert, metric_from_add_problem)
{
  auto domain = std::make_shared<plansys2::DomainExpert>(read_test_pddl("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain);

  const std::string head =
    "(define (problem p) (:domain simple)"
    "(:objects r2d2 - robot bedroom kitchen - room)"
    "(:init (robot_at r2d2 bedroom) (= (room_distance bedroom kitchen) 10))"
    "(:goal (and (robot_at r2d2 kitchen)))";

  // Old style: only total-time
  ASSERT_TRUE(problem_expert.addProblem(head + "(:metric minimize (total-time)))"));
  ASSERT_EQ(parser::pddl::toString(problem_expert.getMetric()), "minimize (total-time)");

  // Any numeric expression, with objects, upper case and maximize
  ASSERT_TRUE(
    problem_expert.addProblem(
      head + "(:metric MAXIMIZE (* (room_distance Bedroom kitchen) (total-time))))"));
  auto metric = problem_expert.getMetric();
  ASSERT_EQ(metric.optimization, plansys2_msgs::msg::Metric::MAXIMIZE);
  ASSERT_EQ(
    parser::pddl::toString(metric), "maximize (* (room_distance bedroom kitchen) (total-time))");

  // Without :metric the previous metric is kept, as the goal
  ASSERT_TRUE(problem_expert.addProblem(head + ")"));
  ASSERT_EQ(
    parser::pddl::toString(problem_expert.getMetric()),
    "maximize (* (room_distance bedroom kitchen) (total-time))");

  // Malformed metrics reject the whole problem and change nothing
  const auto instances = problem_expert.getInstances().size();
  const std::vector<std::string> bad = {
    "(:metric optimize (total-time)))",
    "(:metric minimize (room_distance bedroom hall)))",
    "(:metric minimize (battery r2d2)))",
    "(:metric minimize))",
    "(:metric minimize (total-time) 3))",
  };
  const std::string head_new_object =
    "(define (problem p) (:domain simple)"
    "(:objects r2d2 - robot bedroom kitchen hall2 - room)"
    "(:init (robot_at r2d2 bedroom))"
    "(:goal (and (robot_at r2d2 kitchen)))";
  for (const auto & m : bad) {
    EXPECT_FALSE(problem_expert.addProblem(head_new_object + m)) << m;
    EXPECT_FALSE(problem_expert.existInstance("hall2")) << m;
    EXPECT_EQ(problem_expert.getInstances().size(), instances) << m;
    EXPECT_EQ(
      parser::pddl::toString(problem_expert.getMetric()),
      "maximize (* (room_distance bedroom kitchen) (total-time))") << m;
  }

  // A metric that parses but has the wrong types rejects the whole problem: nothing of it
  // is kept, and the previous metric stays
  ASSERT_FALSE(
    problem_expert.addProblem(
      head_new_object + "(:metric minimize (room_distance r2d2 kitchen)))"));
  ASSERT_FALSE(problem_expert.existInstance("hall2"));
  ASSERT_EQ(problem_expert.getInstances().size(), instances);
  ASSERT_EQ(
    parser::pddl::toString(problem_expert.getMetric()),
    "maximize (* (room_distance bedroom kitchen) (total-time))");
}

TEST(problem_expert, metric_follows_domain_updates)
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  auto problem_expert = make_metric_problem(domain);

  // A change that does not touch the metric keeps it
  ASSERT_TRUE(
    problem_expert.setMetric(plansys2::Metric("minimize (room_distance bedroom kitchen)")));
  ASSERT_TRUE(
    problem_expert.updateDomain(read_test_pddl("domain_simple_v2_extra_requirement.pddl")));
  ASSERT_FALSE(problem_expert.getMetric().tree.nodes.empty());

  // A domain without room_distance cannot keep it
  ASSERT_TRUE(
    problem_expert.updateDomain(read_test_pddl("domain_simple_v4_removed_function.pddl")));
  ASSERT_TRUE(problem_expert.getMetric().tree.nodes.empty());

  // total-time is in every domain
  ASSERT_TRUE(problem_expert.setMetric(plansys2::Metric("minimize (total-time)")));
  ASSERT_TRUE(problem_expert.updateDomain(read_test_pddl("domain_simple.pddl")));
  ASSERT_EQ(parser::pddl::toString(problem_expert.getMetric()), "minimize (total-time)");
}

// Objects of a subtype of a subtype fit where the ancestor is expected
TEST(problem_expert, nested_sub_types)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_deep_types.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  for (const auto & [name, type] : std::vector<std::pair<std::string, std::string>>{
    {"d1", "drone"}, {"t1", "tank"}, {"i1", "infantry"}, {"a1", "artillery"}, {"z", "zone"}})
  {
    ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam(name, type))) << name;
  }

  const std::vector<std::pair<std::string, bool>> predicates = {
    {"(at d1 z)", true},       // child of robot
    {"(at t1 z)", true},       // grandchild
    {"(at i1 z)", true},       // great-grandchild
    {"(at a1 z)", true},
    {"(armored t1)", true},
    {"(armored i1)", true},
    {"(armored d1)", false},   // a drone is not ground
    {"(light i1)", true},
    {"(light t1)", false},     // a tank is not soft
    {"(at z z)", false},       // a zone is not a robot
    {"(captured i1)", false},  // nor a zone
  };
  for (const auto & [predicate, valid] : predicates) {
    EXPECT_EQ(
      problem_expert.addPredicate(parser::pddl::fromStringPredicate(predicate)), valid) <<
      predicate;
  }

  EXPECT_TRUE(problem_expert.addFunction(plansys2::Function("(= (ammo i1) 5)")));
  EXPECT_TRUE(problem_expert.addFunction(plansys2::Function("(= (ammo t1) 4)")));
  EXPECT_FALSE(problem_expert.addFunction(plansys2::Function("(= (ammo d1) 1)")));

  EXPECT_TRUE(problem_expert.setGoal(plansys2::Goal("(and (at i1 z) (armored a1))")));
  EXPECT_FALSE(problem_expert.setGoal(plansys2::Goal("(and (armored d1))")));

  // The same from a whole problem
  plansys2::ProblemExpert problem_expert2(domain_expert);
  ASSERT_TRUE(
    problem_expert2.addProblem(
      "(define (problem p) (:domain deep_types)"
      "(:objects d1 - drone t1 - tank i1 - infantry z - zone)"
      "(:init (at d1 z) (at t1 z) (at i1 z) (armored i1) (light i1) (= (ammo t1) 4))"
      "(:goal (and (captured z))))"));
  ASSERT_EQ(problem_expert2.getPredicates().size(), 5u);
  ASSERT_EQ(problem_expert2.getFunctions().size(), 1u);
}

// Goals with forall, exists and imply over typed variables (#314)
TEST(problem_expert, quantified_goals)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_quantifiers.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);
  ASSERT_TRUE(problem_expert.addProblem(read_test_pddl("problem_quantifiers.pddl")));

  // The goal of the problem: every zone with a hostile robot is owned
  auto goal = problem_expert.getGoal();
  ASSERT_FALSE(goal.nodes.empty());
  ASSERT_FALSE(problem_expert.isGoalSatisfied(goal));  // z2 has e1 and is not owned

  const std::vector<std::pair<std::string, bool>> valid = {
    {"(and (forall (?r - robot) (available ?r)))", true},
    {"(and (exists (?g - ground) (at ?g z1)))", true},
    {"(and (imply (hostile e1) (owned z2)))", true},
    {"(and (forall (?z - zone) (exists (?i - infantry) (at ?i ?z))))", true},
    {"(and (not (exists (?e - robot) (and (hostile ?e) (at ?e z1)))))", true},
    // a variable of a subtype where the ancestor is expected
    {"(and (forall (?t - tank) (at ?t z1)))", true},
    // wrong: unknown predicate, wrong arity, a zone where a robot goes, unknown object
    {"(and (forall (?r - robot) (flying ?r)))", false},
    {"(and (forall (?r - robot) (at ?r)))", false},
    {"(and (forall (?z - zone) (available ?z)))", false},
    {"(and (exists (?r - robot) (at ?r z9)))", false},
    {"(and (imply (hostile e1) (owned e1)))", false},
  };
  for (const auto & [expression, expected] : valid) {
    EXPECT_EQ(problem_expert.isValidGoal(plansys2::Goal(expression)), expected) << expression;
  }

  // Satisfaction against the state
  const std::vector<std::pair<std::string, bool>> satisfied = {
    {"(and (exists (?g - ground) (at ?g z1)))", true},
    {"(and (forall (?r - robot) (available ?r)))", false},  // t1 and e1 are not
    {"(and (forall (?i - infantry) (available ?i)))", true},
    {"(and (not (exists (?e - robot) (and (hostile ?e) (at ?e z1)))))", true},
    {"(and (not (exists (?e - robot) (and (hostile ?e) (at ?e z2)))))", false},
    {"(and (imply (hostile e1) (owned z2)))", false},
  };
  for (const auto & [expression, expected] : satisfied) {
    EXPECT_EQ(problem_expert.isGoalSatisfied(plansys2::Goal(expression)), expected) <<
      expression;
  }

  ASSERT_TRUE(problem_expert.addPredicate(parser::pddl::fromStringPredicate("(owned z2)")));
  ASSERT_TRUE(problem_expert.isGoalSatisfied(goal));

  // The goal goes into the problem for the planner, and back
  plansys2::ProblemExpert problem_expert2(domain_expert);
  ASSERT_TRUE(problem_expert2.addProblem(problem_expert.getProblem()));
  ASSERT_EQ(
    parser::pddl::toString(problem_expert2.getGoal()), parser::pddl::toString(goal));
}

// The state keeps the type of each object, so a state alone says what forall ranges over
TEST(problem_expert, predicates_keep_instance_types)
{
  auto domain_expert = std::make_shared<plansys2::DomainExpert>(
    read_test_pddl("domain_quantifiers.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance("t1", "tank")));
  ASSERT_TRUE(problem_expert.addInstance(plansys2::Instance("z1", "zone")));

  ASSERT_TRUE(problem_expert.addPredicate(parser::pddl::fromStringPredicate("(at t1 z1)")));
  auto predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 1u);
  ASSERT_EQ(predicates[0].parameters[0].type, "tank");
  ASSERT_EQ(predicates[0].parameters[1].type, "zone");

  // Also when the predicate comes with the type it declares, as from a problem
  auto declared = parser::pddl::fromStringPredicate("(hostile t1)");
  declared.parameters[0].type = "robot";
  ASSERT_TRUE(problem_expert.addPredicate(declared));
  ASSERT_EQ(problem_expert.getPredicate("(hostile t1)")->parameters[0].type, "tank");

  // Equality is by name: the predicate is found and removed without types
  ASSERT_TRUE(problem_expert.existPredicate(parser::pddl::fromStringPredicate("(at t1 z1)")));
  ASSERT_TRUE(problem_expert.removePredicate(parser::pddl::fromStringPredicate("(at t1 z1)")));
  ASSERT_EQ(problem_expert.getPredicates().size(), 1u);
}

// The problem keeps the functions' precision: ten decimals, or more digits when needed
TEST(problem_expert, functions_keep_precision)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

  for (double value : {1234567.891, 1e-12, -0.5}) {
    auto function = parser::pddl::fromStringFunction("(room_distance bedroom kitchen)");
    function.value = value;
    ASSERT_TRUE(problem_expert.updateFunction(function) || problem_expert.addFunction(function));
    const auto problem = problem_expert.getProblem();
    const auto expected = parser::pddl::numberToString(value, 10);
    ASSERT_NE(problem.find("( room_distance bedroom kitchen ) " + expected + " )"),
      std::string::npos) << problem;
    // Read back through the parser, the same value
    plansys2::ProblemExpert reread(domain_expert);
    ASSERT_TRUE(reread.addProblem(problem));
    auto function_read = reread.getFunction("(room_distance bedroom kitchen)");
    ASSERT_TRUE(function_read.has_value());
    ASSERT_EQ(function_read->value, value);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
