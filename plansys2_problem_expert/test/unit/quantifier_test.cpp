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

// exists and forall: the same answers as trying every combination of objects, and fast with many

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"

using Transition = lifecycle_msgs::msg::Transition;

namespace
{
plansys2_msgs::msg::Tree tree(const std::string & text)
{
  return parser::pddl::fromString(text);
}

// Rooms r0..rn-1 and robots b0..bm-1; robot_at, connected and free at random
struct World
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  std::vector<std::string> rooms;
  std::vector<std::string> robots;

  bool holds(const std::string & text) const
  {
    return std::any_of(
      predicates.begin(), predicates.end(), [&](const plansys2::Predicate & p) {
        return parser::pddl::toString(p) == text;
      });
  }
};

plansys2::Predicate fact(const std::string & text, const std::vector<std::string> & types)
{
  auto p = parser::pddl::fromStringPredicate(text);
  for (size_t i = 0; i < p.parameters.size(); i++) {
    p.parameters[i].type = types[i];
  }
  return p;
}

World randomWorld(std::mt19937 & rng, size_t rooms, size_t robots, double density)
{
  World w;
  std::bernoulli_distribution coin(density);
  for (size_t i = 0; i < rooms; i++) {
    w.rooms.push_back("r" + std::to_string(i));
  }
  for (size_t i = 0; i < robots; i++) {
    w.robots.push_back("b" + std::to_string(i));
  }
  for (const auto & a : w.rooms) {
    // Every room appears in some fact, so that its type is known
    w.predicates.push_back(fact("(room_known " + a + ")", {"room"}));
    if (coin(rng)) {w.predicates.push_back(fact("(free " + a + ")", {"room"}));}
    for (const auto & b : w.rooms) {
      if (coin(rng)) {
        w.predicates.push_back(fact("(connected " + a + " " + b + ")", {"room", "room"}));
      }
    }
  }
  for (const auto & r : w.robots) {
    w.predicates.push_back(fact("(robot_known " + r + ")", {"robot"}));
    for (const auto & a : w.rooms) {
      if (coin(rng)) {
        w.predicates.push_back(fact("(robot_at " + r + " " + a + ")", {"robot", "room"}));
      }
    }
  }
  return w;
}

// Brute force: whether some robot and two rooms make the body true
bool someWitness(const World & w, bool need_free)
{
  for (const auto & r : w.robots) {
    for (const auto & a : w.rooms) {
      for (const auto & b : w.rooms) {
        if (w.holds("(robot_at " + r + " " + a + ")") &&
          w.holds("(connected " + a + " " + b + ")") &&
          (!need_free || w.holds("(free " + b + ")")) && a != b)
        {
          return true;
        }
      }
    }
  }
  return false;
}
}  // namespace

TEST(quantifiers, exists_matches_brute_force)
{
  std::mt19937 rng(7);
  const std::string body =
    "(and (robot_at ?r ?a)(connected ?a ?b)(free ?b)(not (= ?a ?b)))";
  const auto exists = tree("(exists (?r - robot ?a - room ?b - room) " + body + ")");
  const auto not_exists = tree("(not (exists (?r - robot ?a - room ?b - room) " + body + "))");
  // Every robot is in some room: (forall (?r) (exists (?a) (robot_at ?r ?a)))
  const auto forall =
    tree("(forall (?r - robot) (exists (?a - room) (robot_at ?r ?a)))");

  for (int trial = 0; trial < 300; trial++) {
    std::uniform_int_distribution<size_t> size(1, 6);
    std::uniform_real_distribution<double> density(0.0, 0.5);
    auto w = randomWorld(rng, size(rng), size(rng), density(rng));
    const bool expected = someWitness(w, true);
    EXPECT_EQ(plansys2::check(exists, w.predicates, w.functions), expected) << trial;
    EXPECT_EQ(plansys2::check(not_exists, w.predicates, w.functions), !expected) << trial;

    bool all_placed = true;
    for (const auto & r : w.robots) {
      bool placed = false;
      for (const auto & a : w.rooms) {
        placed = placed || w.holds("(robot_at " + r + " " + a + ")");
      }
      all_placed = all_placed && placed;
    }
    EXPECT_EQ(plansys2::check(forall, w.predicates, w.functions), all_placed) << trial;
  }
}

TEST(quantifiers, exists_in_the_problem_expert_matches_brute_force)
{
  const auto path =
    plansys2::share_path("plansys2_problem_expert").string() +
    "/pddl/domain_simple.pddl";
  std::ifstream ifs(path);
  std::string domain((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  std::mt19937 rng(11);
  for (int trial = 0; trial < 40; trial++) {
    auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain);
    auto expert = std::make_shared<plansys2::ProblemExpert>(domain_expert);
    const size_t rooms = 2 + trial % 5;
    const size_t robots = 1 + trial % 3;
    std::vector<std::string> room_names, robot_names;
    for (size_t i = 0; i < rooms; i++) {
      room_names.push_back("room" + std::to_string(i));
      ASSERT_TRUE(expert->addInstance(plansys2::Instance(room_names.back(), "room")));
    }
    for (size_t i = 0; i < robots; i++) {
      robot_names.push_back("rob" + std::to_string(i));
      ASSERT_TRUE(expert->addInstance(plansys2::Instance(robot_names.back(), "robot")));
    }
    std::bernoulli_distribution coin(0.3);
    std::vector<std::pair<std::string, std::string>> at;
    for (const auto & r : robot_names) {
      for (const auto & a : room_names) {
        if (coin(rng)) {
          ASSERT_TRUE(expert->addPredicate(plansys2::Predicate("(robot_at " + r + " " + a + ")")));
          at.emplace_back(r, a);
        }
      }
    }
    const std::string target = room_names[trial % rooms];
    const bool expected = std::any_of(
      at.begin(), at.end(), [&](const auto & p) {return p.second == target;});
    const auto goal = tree(
      "(exists (?r - robot ?a - room) (and (robot_at ?r ?a)(= ?a " + target + ")))");
    EXPECT_EQ(plansys2::check(goal, *expert), expected) << trial;
    const auto none = tree("(not (exists (?r - robot) (robot_at ?r " + target + ")))");
    EXPECT_EQ(plansys2::check(none, *expert), !expected) << trial;
  }
}

TEST(quantifiers, many_objects_are_fast)
{
  // 60 rooms and 20 robots: 72000 combinations; the only witness is the last robot
  World w;
  for (int i = 0; i < 60; i++) {
    w.rooms.push_back("r" + std::to_string(i));
    w.predicates.push_back(fact("(room_known r" + std::to_string(i) + ")", {"room"}));
  }
  for (int i = 0; i < 20; i++) {
    w.predicates.push_back(fact("(robot_known b" + std::to_string(i) + ")", {"robot"}));
  }
  for (int i = 0; i < 59; i++) {
    w.predicates.push_back(
      fact("(connected r" + std::to_string(i) + " r" + std::to_string(i + 1) + ")",
      {"room", "room"}));
  }
  w.predicates.push_back(fact("(robot_at b19 r58)", {"robot", "room"}));
  w.predicates.push_back(fact("(free r59)", {"room"}));

  const auto exists = tree(
    "(exists (?r - robot ?a - room ?b - room) (and (robot_at ?r ?a)(connected ?a ?b)(free ?b)))");
  const auto missing = tree(
    "(exists (?r - robot ?a - room ?b - room) (and (robot_at ?r ?a)(connected ?b ?a)(free ?b)))");
  const auto everywhere = tree("(forall (?a - room ?b - room) (room_known ?a))");

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 20; i++) {
    ASSERT_TRUE(plansys2::check(exists, w.predicates, w.functions));
    ASSERT_FALSE(plansys2::check(missing, w.predicates, w.functions));
  }
  ASSERT_TRUE(plansys2::check(everywhere, w.predicates, w.functions));
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
  EXPECT_LT(elapsed.count(), 0.5);  // ~4 ms; 3.9 s trying every combination
}

TEST(quantifiers, exists_through_the_client)
{
  const auto path =
    plansys2::share_path("plansys2_problem_expert").string() +
    "/pddl/domain_simple.pddl";
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  domain_node->set_parameter({"model_file", path});
  problem_node->set_parameter({"model_file", path});
  for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
    domain_node->trigger_transition(transition);
    problem_node->trigger_transition(transition);
  }
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  plansys2::testing::ExecutorThread spinner(exe);

  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
  for (const auto & robot : {"rob0", "rob1", "rob2"}) {
    ASSERT_TRUE(client->addInstance(plansys2::Instance(robot, "robot")));
  }
  for (const auto & room : {"room0", "room1", "room2", "room3"}) {
    ASSERT_TRUE(client->addInstance(plansys2::Instance(room, "room")));
  }
  // 12 combinations: the candidates come from the facts
  const auto at_room3 = tree(
    "(exists (?r - robot ?a - room) (and (robot_at ?r ?a)(= ?a room3)))");
  EXPECT_FALSE(plansys2::check(at_room3, client));
  ASSERT_TRUE(client->addPredicate(plansys2::Predicate("(robot_at rob1 room2)")));
  EXPECT_FALSE(plansys2::check(at_room3, client));
  ASSERT_TRUE(client->addPredicate(plansys2::Predicate("(robot_at rob2 room3)")));
  EXPECT_TRUE(plansys2::check(at_room3, client));
  EXPECT_FALSE(
    plansys2::check(tree("(not (exists (?r - robot ?a - room) (robot_at ?r ?a)))"), client));
  EXPECT_TRUE(
    plansys2::check(
      tree("(exists (?r - robot ?a - room ?b - room) (and (robot_at ?r ?a)(robot_at ?r ?b)))"),
      client));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
