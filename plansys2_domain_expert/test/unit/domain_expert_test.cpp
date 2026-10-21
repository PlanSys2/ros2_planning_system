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

#include <fstream>
#include <map>
#include <string>
#include <set>
#include <algorithm>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <utility>

#include "plansys2_core/PackagePaths.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/node.hpp"

TEST(domain_expert, get_reduced_string)
{
  std::string my_string("(and)");
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( and)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( \tand)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( \tand\t)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( and\n)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");
  my_string = "( and\n\t)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");
  my_string = "( ( and\n\t ) )";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "((and))");
}

TEST(domain_expert, exist_domain)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_TRUE(domain_expert.existDomain("charging"));

  std::ifstream domain_simple_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_simple_str((
      std::istreambuf_iterator<char>(domain_simple_ifs)),
    std::istreambuf_iterator<char>());

  domain_expert.extendDomain(domain_simple_str);
  ASSERT_TRUE(domain_expert.existDomain("plansys2"));
}

TEST(domain_expert, get_domain)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_EQ(domain_expert.getDomain(), domain_str_p);
}

TEST(domain_expert, get_domain2)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/factory.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/factory_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_EQ(domain_expert.getDomain(), domain_str_p);
}

TEST(domain_expert, get_name)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/factory.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::string name = domain_expert.getName();
  std::string test_name("factory");

  ASSERT_EQ(name, test_name);
}

TEST(domain_expert, get_domain3)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_constants_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_EQ(domain_expert.getDomain(), domain_str_p);
}

TEST(domain_expert, get_types)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<std::string> types = domain_expert.getTypes();
  std::vector<std::string> test_types {"object", "person", "message", "robot",
    "room", "teleporter_room"};

  ASSERT_EQ(types, test_types);
}

TEST(domain_expert, get_constants)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<std::string> consts1 = domain_expert.getConstants("robot");
  std::vector<std::string> consts2 = domain_expert.getConstants("person");
  std::vector<std::string> test_consts1 {"leia", "lema"};
  std::vector<std::string> test_consts2 {"jack", "john"};

  ASSERT_EQ(consts1, test_consts1);
  ASSERT_EQ(consts2, test_consts2);

  // Asking for a type that does not exist does not add it
  const auto types = domain_expert.getTypes();
  ASSERT_TRUE(domain_expert.getConstants("spaceship").empty());
  ASSERT_TRUE(domain_expert.getConstants("").empty());
  ASSERT_EQ(domain_expert.getTypes(), types);
}

TEST(domain_expert, get_predicates)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<plansys2::Predicate> predicates = domain_expert.getPredicates();
  std::vector<std::string> predicates_types {"person_at", "robot_at", "robot_near_person",
    "robot_talk"};

  ASSERT_EQ(predicates.size(), predicates_types.size());
  for (unsigned i = 0; i < predicates.size(); i++) {
    ASSERT_EQ(predicates[i].name, predicates_types[i]);
  }
}

TEST(domain_expert, get_predicate_params)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto params_1 = domain_expert.getPredicate("robot_talk");
  ASSERT_TRUE(params_1);
  ASSERT_EQ(params_1.value().name, "robot_talk");
  ASSERT_EQ(params_1.value().parameters.size(), 3);
  ASSERT_EQ(params_1.value().parameters[0].name, "?robot0");
  ASSERT_EQ(params_1.value().parameters[0].type, "robot");
  ASSERT_EQ(params_1.value().parameters[1].name, "?message1");
  ASSERT_EQ(params_1.value().parameters[1].type, "message");
  ASSERT_EQ(params_1.value().parameters[2].name, "?person2");
  ASSERT_EQ(params_1.value().parameters[2].type, "person");

  auto params_2 = domain_expert.getPredicate("ROBOT_TALK");
  ASSERT_TRUE(params_2);

  auto params_3 = domain_expert.getPredicate("person_at");
  ASSERT_TRUE(params_3);
  ASSERT_EQ(params_3.value().parameters.size(), 2);
  ASSERT_EQ(params_3.value().parameters[0].name, "?person0");
  ASSERT_EQ(params_3.value().parameters[0].type, "person");
  ASSERT_EQ(params_3.value().parameters[1].name, "?room1");
  ASSERT_EQ(params_3.value().parameters[1].type, "room");
}

TEST(domain_expert, get_functions)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<plansys2::Function> functions = domain_expert.getFunctions();
  std::vector<std::string> functions_types {"speed", "max_range", "state_of_charge",
    "distance"};

  ASSERT_EQ(functions.size(), functions_types.size());
  for (unsigned i = 0; i < functions.size(); i++) {
    ASSERT_EQ(functions[i].name, functions_types[i]);
  }
}

TEST(domain_expert, get_function_params)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto params_1 = domain_expert.getFunction("speed");
  ASSERT_TRUE(params_1);
  ASSERT_EQ(params_1.value().name, "speed");
  ASSERT_EQ(params_1.value().parameters.size(), 1);
  ASSERT_EQ(params_1.value().parameters[0].name, "?robot0");
  ASSERT_EQ(params_1.value().parameters[0].type, "robot");

  auto params_2 = domain_expert.getFunction("SPEED");
  ASSERT_TRUE(params_2);

  auto params_3 = domain_expert.getFunction("distance");
  ASSERT_TRUE(params_3);
  ASSERT_EQ(params_3.value().parameters.size(), 2);
  ASSERT_EQ(params_3.value().parameters[0].name, "?waypoint0");
  ASSERT_EQ(params_3.value().parameters[0].type, "waypoint");
  ASSERT_EQ(params_3.value().parameters[1].name, "?waypoint1");
  ASSERT_EQ(params_3.value().parameters[1].type, "waypoint");
}

TEST(domain_expert, get_derived_predicates)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_derived.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<plansys2_msgs::msg::Derived> predicates = domain_expert.getDerivedPredicates();
  std::vector<std::string> predicates_types {"inferred-robot_at", "inferred-person_at"};

  ASSERT_EQ(predicates.size(), predicates_types.size());
  for (unsigned i = 0; i < predicates.size(); i++) {
    ASSERT_EQ(predicates[i].predicate.name, predicates_types[i]);
  }
}

TEST(domain_expert, get_derived_predicate_params)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_derived.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto pred_1 = domain_expert.getDerivedPredicate("inferred-robot_at");
  ASSERT_GT(pred_1.size(), 0);
  ASSERT_EQ(pred_1[0].predicate.name, "inferred-robot_at");
  ASSERT_EQ(pred_1[0].predicate.parameters.size(), 2);
  ASSERT_EQ(pred_1[0].predicate.parameters[0].name, "?robot0");
  ASSERT_EQ(pred_1[0].predicate.parameters[0].type, "robot");
  ASSERT_EQ(pred_1[0].predicate.parameters[1].name, "?room1");
  ASSERT_EQ(pred_1[0].predicate.parameters[1].type, "room");
  ASSERT_EQ(
    parser::pddl::toString(pred_1[0].preconditions), "(and (robot_at ?0 ?1))");
}

TEST(domain_expert, get_actions)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto actions = domain_expert.getActions();
  ASSERT_EQ(actions.size(), 1);
  ASSERT_EQ(actions[0], "move_person");

  auto durative_actions = domain_expert.getDurativeActions();
  ASSERT_EQ(durative_actions.size(), 3);
  ASSERT_EQ(durative_actions[0], "move");
  ASSERT_EQ(durative_actions[1], "talk");
  ASSERT_EQ(durative_actions[2], "approach");
}


TEST(domain_expert, get_action_params)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  domain_expert.getAction("move");
  auto no_action = domain_expert.getAction("noexist");
  ASSERT_FALSE(no_action);

  auto move_action = domain_expert.getDurativeAction("move");
  auto at_start = move_action->at_start_requirements;
  ASSERT_EQ(parser::pddl::toString(at_start), "(and (robot_at ?0 ?1))");

  ASSERT_TRUE(move_action);
  ASSERT_EQ(move_action->name, "move");
  ASSERT_EQ(move_action->parameters.size(), 3);
  ASSERT_EQ(move_action->parameters[0].name, "?0");
  ASSERT_EQ(move_action->parameters[0].type, "robot");
  ASSERT_EQ(move_action->parameters[1].name, "?1");
  ASSERT_EQ(move_action->parameters[1].type, "room");
  ASSERT_EQ(move_action->parameters[2].name, "?2");
  ASSERT_EQ(move_action->parameters[2].type, "room");

  ASSERT_FALSE(parser::pddl::empty(at_start));
  ASSERT_TRUE(parser::pddl::empty(move_action->over_all_requirements));
  ASSERT_TRUE(parser::pddl::empty(move_action->at_end_requirements));

  ASSERT_EQ(
    parser::pddl::toString(move_action->at_start_requirements),
    "(and (robot_at ?0 ?1))");
  ASSERT_EQ(
    parser::pddl::toString(move_action->at_start_effects),
    "(and (not (robot_at ?0 ?1)))");
  ASSERT_EQ(
    parser::pddl::toString(move_action->at_end_effects),
    "(and (robot_at ?0 ?2))");
}

TEST(domain_expert, multidomain_get_types)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);

  std::ifstream domain_ext_ifs(pkgpath + "/pddl/domain_simple_ext.pddl");
  std::string domain_ext_str((
      std::istreambuf_iterator<char>(domain_ext_ifs)),
    std::istreambuf_iterator<char>());

  domain_expert->extendDomain(domain_ext_str);

  std::vector<std::string> types = domain_expert->getTypes();
  std::vector<std::string> test_types {"object", "person", "message", "robot", "room",
    "teleporter_room", "pickable_object"};

  ASSERT_EQ(types, test_types);

  std::vector<plansys2::Predicate> predicates = domain_expert->getPredicates();
  std::vector<std::string> test_predicates {"object_at_robot", "object_at_room", "person_at",
    "robot_at", "robot_near_person", "robot_talk"};

  ASSERT_EQ(predicates.size(), test_predicates.size());
  for (unsigned i = 0; i < predicates.size(); i++) {
    ASSERT_EQ(predicates[i].name, test_predicates[i]);
  }

  std::vector<std::string> actions = domain_expert->getActions();
  std::vector<std::string> test_actions {"move_person"};

  ASSERT_EQ(actions, test_actions);

  std::vector<std::string> dactions = domain_expert->getDurativeActions();
  std::vector<std::string> test_dactions {"move", "talk", "approach", "pick_object",
    "place_object"};

  ASSERT_EQ(dactions, test_dactions);
}

TEST(domain_expert, sub_types)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  // Parameter subtypes with a durative-action
  plansys2_msgs::msg::DurativeAction::SharedPtr durative_action =
    domain_expert.getDurativeAction("move");
  if (durative_action) {
    if (durative_action->parameters.size() == 3) {
      ASSERT_EQ(durative_action->parameters[1].type, "room");

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(durative_action->parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `move` durative-action is expected to have 2 parameters";
    }
  } else {
    FAIL() << "No `move` durative-action found in the domain";
  }

  // Parameter subtypes with a predicate
  std::optional<plansys2::Predicate> predicate = domain_expert.getPredicate("robot_at");
  if (predicate.has_value()) {
    if (predicate.value().parameters.size() == 2) {
      ASSERT_EQ(predicate.value().parameters[1].type, "room");

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(predicate.value().parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `robot_at` predicate is expected to have 3 parameters";
    }
  } else {
    FAIL() << "No `robot_at` predicate found in the domain";
  }

  // Parameter subtypes with as action
  plansys2_msgs::msg::Action::SharedPtr action =
    domain_expert.getAction("move_person");
  if (action) {
    if (action->parameters.size() == 3) {
      ASSERT_EQ(action->parameters[1].type, "room");

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(action->parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `move_person` action is expected to have 3 parameters";
    }
  } else {
    FAIL() << "No `move_person` action found in the domain";
  }

  // Parameter subtypes with as function
  std::optional<plansys2::Function> function =
    domain_expert.getFunction("teleportation_time");
  if (function.has_value()) {
    if (function.value().parameters.size() == 2) {
      ASSERT_EQ(function.value().parameters[0].type, "teleporter_room");
      ASSERT_EQ(function.value().parameters[1].type, "room");

      std::vector<std::string> emptySubtypes {};
      ASSERT_EQ(function.value().parameters[0].sub_types, emptySubtypes);

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(function.value().parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `teleportation_time` function is expected to have 2 parameters";
    }
  } else {
    FAIL() << "No `teleportation_time` function found in the domain";
  }
}

// Subtypes of subtypes: an infantry is a soft, a ground and a robot
TEST(domain_expert, nested_sub_types)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_deep_types.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto sorted = [](std::vector<std::string> v) {
      std::sort(v.begin(), v.end());
      return v;
    };
  const std::vector<std::string> robot_subtypes =
    sorted({"drone", "ground", "tank", "soft", "infantry", "artillery"});
  const std::vector<std::string> ground_subtypes =
    sorted({"tank", "soft", "infantry", "artillery"});
  const std::vector<std::string> soft_subtypes = sorted({"infantry", "artillery"});
  const std::vector<std::string> none;

  auto at = domain_expert.getPredicate("at");
  ASSERT_TRUE(at.has_value());
  ASSERT_EQ(sorted(at->parameters[0].sub_types), robot_subtypes);
  ASSERT_EQ(at->parameters[1].sub_types, none);

  auto armored = domain_expert.getPredicate("armored");
  ASSERT_TRUE(armored.has_value());
  ASSERT_EQ(sorted(armored->parameters[0].sub_types), ground_subtypes);

  auto light = domain_expert.getPredicate("light");
  ASSERT_TRUE(light.has_value());
  ASSERT_EQ(sorted(light->parameters[0].sub_types), soft_subtypes);

  auto ammo = domain_expert.getFunction("ammo");
  ASSERT_TRUE(ammo.has_value());
  ASSERT_EQ(sorted(ammo->parameters[0].sub_types), ground_subtypes);

  auto move = domain_expert.getDurativeAction("move");
  ASSERT_NE(move, nullptr);
  ASSERT_EQ(sorted(move->parameters[0].sub_types), robot_subtypes);

  auto capture = domain_expert.getAction("capture");
  ASSERT_NE(capture, nullptr);
  ASSERT_EQ(capture->parameters[0].sub_types, none);

  // No subtype is listed twice
  auto all = at->parameters[0].sub_types;
  ASSERT_EQ(std::set<std::string>(all.begin(), all.end()).size(), all.size());
}

// Actions with forall, exists and imply: getting them threw UnsupportedConstruct (#314)
TEST(domain_expert, quantified_conditions_and_effects)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_quantifiers.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto capture = domain_expert.getDurativeAction("capture", {"i1", "z1"});
  ASSERT_NE(capture, nullptr);
  ASSERT_EQ(
    parser::pddl::toString(capture->at_start_requirements),
    "(and (forall (?2 - robot) (imply (hostile ?2) (not (at ?2 z1)))))");
  ASSERT_EQ(parser::pddl::toString(capture->over_all_requirements), "(and (at i1 z1))");

  auto secure = domain_expert.getDurativeAction("secure", {"z1"});
  ASSERT_NE(secure, nullptr);
  ASSERT_EQ(
    parser::pddl::toString(secure->at_start_requirements),
    "(and (exists (?1 - ground) (at ?1 z1)))");
  ASSERT_EQ(
    parser::pddl::toString(secure->at_end_effects),
    "(and (forall (?1 - robot) (not (available ?1))))");

  // The quantified variables say what they range over
  const auto & forall = secure->at_end_effects.nodes[1];
  ASSERT_EQ(forall.node_type, plansys2_msgs::msg::Node::FORALL);
  ASSERT_EQ(forall.parameters[0].type, "robot");
  std::vector<std::string> subtypes = forall.parameters[0].sub_types;
  std::sort(subtypes.begin(), subtypes.end());
  ASSERT_EQ(subtypes, (std::vector<std::string>{"drone", "ground", "infantry", "tank"}));
}

// A predicate written over several lines is one predicate, also when two domains are joined
// and one of them repeats it with other blanks (#286)
TEST(domain_expert, multiline_predicates)
{
  const std::string domain_1 =
    R"(
(define (domain first)
(:requirements :typing)
(:types foobar)
(:predicates
    (foo ?foo - foobar)
    (bar
         ?foo - foobar  ; a comment between the lines
         ?bar - foobar
         ?baz - foobar
    )
)
)
)";
  const std::string domain_2 =
    R"(
(define (domain second)
(:requirements :typing)
(:types foobar)
(:predicates
    (bar ?foo - foobar ?bar - foobar ?baz - foobar)
    (qux
      ?q - foobar)
)
)
)";

  plansys2::DomainExpert expert(domain_1);
  expert.extendDomain(domain_2);

  ASSERT_EQ(expert.getPredicates().size(), 3u);
  for (const auto & [name, arity] : std::map<std::string, size_t>{{"foo", 1}, {"bar", 3},
      {"qux", 1}})
  {
    auto predicate = expert.getPredicate(name);
    ASSERT_TRUE(predicate.has_value()) << name;
    EXPECT_EQ(predicate->parameters.size(), arity) << name;
  }

  // The joint domain is valid PDDL again
  plansys2::DomainExpert reparsed(expert.getDomain());
  EXPECT_EQ(reparsed.getPredicates().size(), 3u);
  ASSERT_TRUE(reparsed.getPredicate("bar").has_value());
  EXPECT_EQ(reparsed.getPredicate("bar")->parameters.size(), 3u);
}

// What PlanSys2 cannot read or execute is said, not left as a domain without actions
TEST(domain_expert, unreadable_and_unsupported_are_reported)
{
  const std::string pkgpath =
    plansys2::share_path("plansys2_domain_expert").string();
  auto read = [&](const std::string & file) {
      std::ifstream ifs(pkgpath + "/pddl/" + file);
      return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    };

  // A name never declared: the error names it
  plansys2::DomainExpert unknown(
    "(define (domain d) (:requirements :strips) (:predicates (p))"
    " (:action a :parameters () :precondition (and (undeclared)) :effect (and (p))))");
  EXPECT_NE(unknown.getParseError().find("undeclared"), std::string::npos)
    << unknown.getParseError();

  // ?duration, in the bounds and in an effect: the plan binds it
  plansys2::DomainExpert duration(read("domain_duration_effect.pddl"));
  EXPECT_TRUE(duration.getParseError().empty()) << duration.getParseError();
  EXPECT_TRUE(duration.getUnsupported().empty());
  auto charge = duration.getDurativeAction("charge", {"r1"});
  ASSERT_TRUE(charge);
  EXPECT_EQ(
    parser::pddl::toString(charge->duration),
    "(and (>= ?duration 1.000000)(<= ?duration 8.000000))");
  EXPECT_EQ(
    parser::pddl::toString(charge->at_end_effects),
    "(and (increase (battery r1) (* 10.000000 ?duration)))");
  parser::pddl::bindDuration(charge->at_end_effects, 6.5);
  parser::pddl::bindDuration(charge->duration, 6.5);
  EXPECT_EQ(
    parser::pddl::toString(charge->at_end_effects),
    "(and (increase (battery r1) (* 10.000000 6.500000)))");
  EXPECT_EQ(
    parser::pddl::toString(charge->duration),
    "(and (>= 6.500000 1.000000)(<= 6.500000 8.000000))");

  // A fixed duration is a bound too
  plansys2::DomainExpert fixed(read("domain_simple.pddl"));
  auto move = fixed.getDurativeAction("move", {"r2d2", "kitchen", "bedroom"});
  ASSERT_TRUE(move);
  EXPECT_EQ(parser::pddl::toString(move->duration).rfind("(and (= ?duration ", 0), 0u)
    << parser::pddl::toString(move->duration);

  // Conditional effects are
  plansys2::DomainExpert when(read("domain_when.pddl"));
  EXPECT_TRUE(when.getParseError().empty()) << when.getParseError();
  EXPECT_TRUE(when.getUnsupported().empty());
  auto push = when.getAction("push");
  ASSERT_TRUE(push);
  EXPECT_EQ(
    parser::pddl::toString(push->effects),
    "(and (pushed ?0 ?1)(when (not (locked ?1)) (open ?1)))");

  // PDDL3: the constraints are kept in the domain, for the planner and the executor
  plansys2::DomainExpert constraints(read("domain_constraints.pddl"));
  EXPECT_TRUE(constraints.getParseError().empty()) << constraints.getParseError();
  EXPECT_NE(
    constraints.getDomain().find("(:constraints (always (safe)))"), std::string::npos) <<
    constraints.getDomain();

  // Those of two domains hold both
  constraints.extendDomain(
    "(define (domain lights) (:requirements :strips :constraints) (:predicates (lit))"
    " (:action light :parameters () :precondition (and) :effect (and (lit)))"
    " (:constraints (sometime (lit))))");
  EXPECT_TRUE(constraints.getParseError().empty()) << constraints.getParseError();
  EXPECT_NE(
    constraints.getDomain().find("(:constraints (and (always (safe)) (sometime (lit))))"),
    std::string::npos) << constraints.getDomain();

  // A preference in a precondition, in its tree
  plansys2::DomainExpert preference(read("domain_preference.pddl"));
  EXPECT_TRUE(preference.getParseError().empty()) << preference.getParseError();
  EXPECT_TRUE(preference.getUnsupported().empty());
  auto open_door = preference.getAction("open_door");
  ASSERT_TRUE(open_door);
  EXPECT_EQ(
    parser::pddl::toString(open_door->preconditions), "(and (preference close (near ?0 ?1)))");

  plansys2::DomainExpert simple(read("domain_simple.pddl"));
  EXPECT_TRUE(simple.getParseError().empty());
  EXPECT_TRUE(simple.getUnsupported().empty());
}

// PDDL+: processes and events, kept in the domain and served as actions
TEST(domain_expert, processes_and_events)
{
  const std::string pkgpath =
    plansys2::share_path("plansys2_domain_expert").string();
  auto read = [&](const std::string & file) {
      std::ifstream ifs(pkgpath + "/pddl/" + file);
      return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    };

  plansys2::DomainExpert heating(read("domain_process.pddl"));
  ASSERT_TRUE(heating.getParseError().empty()) << heating.getParseError();
  EXPECT_TRUE(heating.getUnsupported().empty());
  EXPECT_EQ(heating.getProcesses(), std::vector<std::string>{"heat"});
  EXPECT_TRUE(heating.getEvents().empty());
  auto heat = heating.getProcess("heat", {"kitchen"});
  ASSERT_TRUE(heat);
  EXPECT_EQ(heat->parameters.size(), 1u);
  EXPECT_EQ(heat->parameters[0].type, "room");
  EXPECT_EQ(parser::pddl::toString(heat->preconditions), "(and (heating kitchen))");
  EXPECT_EQ(
    parser::pddl::toString(heat->effects),
    "(and (increase (temperature kitchen) (* #t 1.000000)))");
  EXPECT_FALSE(heating.getProcess("cool"));
  EXPECT_FALSE(heating.getEvent("heat"));
  // The domain the planner gets has them
  EXPECT_NE(heating.getDomain().find(":process heat"), std::string::npos) << heating.getDomain();

  plansys2::DomainExpert both(
    read("domain_process.pddl").substr(0, read("domain_process.pddl").rfind(')')) +
    " (:event overheat :parameters (?r - room) :precondition (and (>= (temperature ?r) 30))"
    " :effect (and (not (heating ?r)))))");
  ASSERT_TRUE(both.getParseError().empty()) << both.getParseError();
  EXPECT_EQ(both.getEvents(), std::vector<std::string>{"overheat"});
  auto overheat = both.getEvent("overheat", {"lab"});
  ASSERT_TRUE(overheat);
  EXPECT_EQ(
    parser::pddl::toString(overheat->preconditions),
    "(and (>= (temperature lab) 30.000000))");
  EXPECT_EQ(parser::pddl::toString(overheat->effects), "(and (not (heating lab)))");

  // And read back from what it gives
  plansys2::DomainExpert again(both.getDomain());
  EXPECT_TRUE(again.getParseError().empty()) << again.getParseError();
  EXPECT_EQ(again.getProcesses(), std::vector<std::string>{"heat"});
  EXPECT_EQ(again.getEvents(), std::vector<std::string>{"overheat"});
}

// PDDL 2.1: an effect of a durative action that acts while it runs
TEST(domain_expert, continuous_effects_of_a_durative_action)
{
  plansys2::DomainExpert domain(
    "(define (domain tank) (:requirements :strips :typing :durative-actions :fluents"
    " :continuous-effects) (:types tank)"
    " (:predicates (open ?t - tank)) (:functions (level ?t - tank) (flow))"
    " (:durative-action fill :parameters (?t - tank) :duration (<= ?duration 10)"
    "  :condition (at start (open ?t))"
    "  :effect (and (at end (not (open ?t))) (increase (level ?t) (* #t (flow)))"
    "   (decrease (flow) (* #t 0.1)))))");
  ASSERT_TRUE(domain.getParseError().empty()) << domain.getParseError();
  EXPECT_TRUE(domain.getUnsupported().empty());
  auto fill = domain.getDurativeAction("fill", {"t1"});
  ASSERT_TRUE(fill);
  EXPECT_EQ(
    parser::pddl::toString(fill->continuous_effects),
    "(and (increase (level t1) (* #t (flow)))(decrease (flow) (* #t 0.100000)))");
  EXPECT_EQ(parser::pddl::toString(fill->at_end_effects), "(and (not (open t1)))");

  // Read back from what it gives
  plansys2::DomainExpert again(domain.getDomain());
  ASSERT_TRUE(again.getParseError().empty()) << again.getParseError() << domain.getDomain();
  EXPECT_EQ(
    parser::pddl::toString(again.getDurativeAction("fill", {"t1"})->continuous_effects),
    parser::pddl::toString(fill->continuous_effects));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
