// Copyright 2026 Juan Carlos Manzanares Serrano
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

// Problem expert: several problem changes applied atomically

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/PackagePaths.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_msgs/msg/problem_change.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/Utils.hpp"

using Change = plansys2_msgs::msg::ProblemChange;

namespace
{

std::shared_ptr<plansys2::ProblemExpert> expert()
{
  const auto path =
    plansys2::share_path("plansys2_problem_expert").string() + "/pddl/domain_simple.pddl";
  std::ifstream ifs(path);
  auto domain = std::make_shared<plansys2::DomainExpert>(
    std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()));
  auto expert = std::make_shared<plansys2::ProblemExpert>(domain);
  EXPECT_TRUE(expert->addInstance(plansys2::Instance("r2d2", "robot")));
  EXPECT_TRUE(expert->addInstance(plansys2::Instance("kitchen", "room")));
  EXPECT_TRUE(expert->addInstance(plansys2::Instance("hall", "room")));
  return expert;
}

Change instance(uint8_t type, const std::string & name, const std::string & type_name)
{
  Change change;
  change.type = type;
  change.instance = plansys2::Instance(name, type_name);
  return change;
}

Change fact(uint8_t type, const std::string & expression)
{
  Change change;
  change.type = type;
  change.node = (type == Change::SET_FUNCTION || type == Change::REMOVE_FUNCTION) ?
    static_cast<plansys2_msgs::msg::Node>(parser::pddl::fromStringFunction(expression)) :
    static_cast<plansys2_msgs::msg::Node>(parser::pddl::fromStringPredicate(expression));
  return change;
}

bool holds(plansys2::ProblemExpert & expert, const std::string & predicate)
{
  return expert.existPredicate(parser::pddl::fromStringPredicate(predicate));
}

}  // namespace

TEST(apply_changes, an_instance_and_its_facts_at_once)
{
  auto pe = expert();
  std::string error;
  ASSERT_TRUE(pe->applyChanges({
    instance(Change::ADD_INSTANCE, "garden", "room"),
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 garden)"),
    fact(Change::SET_FUNCTION, "(= (room_distance kitchen garden) 7.5)"),
    fact(Change::SET_FUNCTION, "(= (room_distance kitchen garden) 8)"),  // Then updated
  }, error)) << error;
  EXPECT_TRUE(pe->existInstance("garden"));
  EXPECT_TRUE(holds(*pe, "(robot_at r2d2 garden)"));
  auto distance = pe->getFunction("(room_distance kitchen garden)");
  ASSERT_TRUE(distance.has_value());
  EXPECT_DOUBLE_EQ(distance->value, 8.0);
}

TEST(apply_changes, one_that_fails_and_none_is_made)
{
  auto pe = expert();
  ASSERT_TRUE(pe->addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 kitchen)")));
  ASSERT_TRUE(pe->addFunction(parser::pddl::fromStringFunction(
    "(= (room_distance kitchen hall) 3)")));
  const auto before = pe->getProblem();

  std::string error;
  EXPECT_FALSE(pe->applyChanges({
    fact(Change::REMOVE_PREDICATE, "(robot_at r2d2 kitchen)"),
    instance(Change::ADD_INSTANCE, "garden", "room"),
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 garden)"),
    fact(Change::SET_FUNCTION, "(= (room_distance kitchen hall) 9)"),
    fact(Change::REMOVE_FUNCTION, "(room_distance kitchen hall)"),
    instance(Change::REMOVE_INSTANCE, "hall", "room"),
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 moon)"),  // No such room
  }, error));
  EXPECT_NE(error.find("change 6"), std::string::npos) << error;
  EXPECT_NE(error.find("none was made"), std::string::npos) << error;
  // As it was, the instance to remove too: removals go last
  EXPECT_EQ(pe->getProblem(), before);
  EXPECT_TRUE(holds(*pe, "(robot_at r2d2 kitchen)"));
  EXPECT_FALSE(pe->existInstance("garden"));
  EXPECT_TRUE(pe->existInstance("hall"));
}

TEST(apply_changes, instances_are_removed_last_with_their_facts)
{
  auto pe = expert();
  std::string error;
  // The removal first in the list, but made after the predicate on it was added
  ASSERT_TRUE(pe->applyChanges({
    instance(Change::REMOVE_INSTANCE, "hall", "room"),
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 hall)"),
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 kitchen)"),
  }, error)) << error;
  EXPECT_FALSE(pe->existInstance("hall"));
  EXPECT_FALSE(holds(*pe, "(robot_at r2d2 hall)"));
  EXPECT_TRUE(holds(*pe, "(robot_at r2d2 kitchen)"));
}

TEST(apply_changes, what_is_there_or_is_not_and_nothing)
{
  auto pe = expert();
  ASSERT_TRUE(pe->addPredicate(parser::pddl::fromStringPredicate("(robot_at r2d2 kitchen)")));
  std::string error;
  EXPECT_TRUE(pe->applyChanges({}, error));
  ASSERT_TRUE(pe->applyChanges({
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 kitchen)"),       // Already there
    fact(Change::REMOVE_PREDICATE, "(robot_at r2d2 hall)"),       // Not there
    fact(Change::REMOVE_FUNCTION, "(room_distance hall kitchen)"),
    instance(Change::REMOVE_INSTANCE, "garden", "room"),
    instance(Change::ADD_INSTANCE, "kitchen", "room"),
  }, error)) << error;
  // A failure after them takes back nothing that was not made by the changes
  EXPECT_FALSE(pe->applyChanges({
    fact(Change::ADD_PREDICATE, "(robot_at r2d2 kitchen)"),
    instance(Change::ADD_INSTANCE, "kitchen", "robot"),           // Another type: not valid
  }, error));
  EXPECT_TRUE(holds(*pe, "(robot_at r2d2 kitchen)"));
  EXPECT_TRUE(pe->existInstance("kitchen"));
  Change unknown;
  unknown.type = 42;
  EXPECT_FALSE(pe->applyChanges({unknown}, error));
  EXPECT_NE(error.find("unknown type 42"), std::string::npos) << error;
}
