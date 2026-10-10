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

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using plansys2_msgs::msg::Node;
using plansys2_msgs::msg::Tree;

namespace
{

Node make_node(uint8_t type, std::vector<uint32_t> children = {})
{
  Node node;
  node.node_type = type;
  node.children = children;
  return node;
}

bool valid(const Tree & tree)
{
  std::string error;
  return parser::pddl::validateTree(tree, error);
}

// A chain of `depth` nodes: NOT, NOT, ..., predicate
Tree not_chain(int depth)
{
  Tree tree;
  for (int i = 0; i < depth - 1; i++) {
    tree.nodes.push_back(make_node(Node::NOT, {static_cast<uint32_t>(i + 1)}));
  }
  tree.nodes.push_back(make_node(Node::PREDICATE));
  return tree;
}

}  // namespace

TEST(validate_tree, empty_tree_is_valid)
{
  ASSERT_TRUE(valid(Tree()));
}

TEST(validate_tree, parsed_trees_are_valid)
{
  for (const std::string expr : {
    "(and (robot_at r1 kitchen) (not (robot_at r1 bedroom)))",
    "(or (robot_at r1 kitchen) (and (robot_at r1 bedroom) (person_at p1 bedroom)))",
    "(exists (?r) (and (robot_at r1 ?r)))",
    "(exists (?r) (robot_at r1 ?r))",
    "(and (> (battery r1) 10) (< (+ (battery r1) 5) 100))",
    "(and (increase (battery r1) 10) (assign (speed r1) (* (speed r1) 2)))",
    "(not (> (battery r1) 10))"})
  {
    SCOPED_TRACE(expr);
    auto tree = parser::pddl::fromString(expr);
    ASSERT_FALSE(tree.nodes.empty());
    ASSERT_TRUE(valid(tree));
  }
}

TEST(validate_tree, child_out_of_range)
{
  Tree tree;
  tree.nodes.push_back(make_node(Node::AND, {1, 2}));
  tree.nodes.push_back(make_node(Node::PREDICATE));
  std::string error;
  ASSERT_FALSE(parser::pddl::validateTree(tree, error));
  ASSERT_NE(error.find("out of range"), std::string::npos);

  // Ids that would wrap around in a narrower integer type are out of range too
  tree.nodes[0].children = {1, 256, 65536, 4294967295u};
  ASSERT_FALSE(valid(tree));
}

TEST(validate_tree, cycles_and_shared_nodes)
{
  Tree self_loop;
  self_loop.nodes.push_back(make_node(Node::AND, {0}));
  ASSERT_FALSE(valid(self_loop));

  Tree two_node_cycle;
  two_node_cycle.nodes.push_back(make_node(Node::AND, {1}));
  two_node_cycle.nodes.push_back(make_node(Node::OR, {0}));
  ASSERT_FALSE(valid(two_node_cycle));

  Tree back_to_root;
  back_to_root.nodes.push_back(make_node(Node::AND, {1, 2}));
  back_to_root.nodes.push_back(make_node(Node::PREDICATE));
  back_to_root.nodes.push_back(make_node(Node::NOT, {0}));
  ASSERT_FALSE(valid(back_to_root));

  Tree shared;
  shared.nodes.push_back(make_node(Node::AND, {1, 2}));
  shared.nodes.push_back(make_node(Node::NOT, {3}));
  shared.nodes.push_back(make_node(Node::NOT, {3}));
  shared.nodes.push_back(make_node(Node::PREDICATE));
  ASSERT_FALSE(valid(shared));

  Tree repeated_child;
  repeated_child.nodes.push_back(make_node(Node::AND, {1, 1}));
  repeated_child.nodes.push_back(make_node(Node::PREDICATE));
  ASSERT_FALSE(valid(repeated_child));
}

TEST(validate_tree, number_of_children)
{
  // NOT needs exactly one
  Tree tree;
  tree.nodes.push_back(make_node(Node::NOT));
  ASSERT_FALSE(valid(tree));
  tree.nodes = {make_node(Node::NOT, {1, 2}), make_node(Node::PREDICATE),
    make_node(Node::PREDICATE)};
  ASSERT_FALSE(valid(tree));

  // EXPRESSION needs exactly two
  tree.nodes = {make_node(Node::EXPRESSION, {1}), make_node(Node::FUNCTION)};
  ASSERT_FALSE(valid(tree));
  tree.nodes = {make_node(Node::EXPRESSION, {1, 2}), make_node(Node::FUNCTION),
    make_node(Node::NUMBER)};
  ASSERT_TRUE(valid(tree));

  // FUNCTION_MODIFIER: function and value, or only the value for total-cost
  tree.nodes = {make_node(Node::FUNCTION_MODIFIER)};
  ASSERT_FALSE(valid(tree));
  tree.nodes = {make_node(Node::FUNCTION_MODIFIER, {1}), make_node(Node::NUMBER)};
  ASSERT_TRUE(valid(tree));
  tree.nodes = {make_node(Node::FUNCTION_MODIFIER, {1, 2, 3}), make_node(Node::FUNCTION),
    make_node(Node::NUMBER), make_node(Node::NUMBER)};
  ASSERT_FALSE(valid(tree));

  // EXISTS needs a body
  tree.nodes = {make_node(Node::EXISTS)};
  ASSERT_FALSE(valid(tree));

  // Leaves have no children
  tree.nodes = {make_node(Node::PREDICATE, {1}), make_node(Node::PREDICATE)};
  ASSERT_FALSE(valid(tree));

  // AND / OR may be empty
  tree.nodes = {make_node(Node::AND)};
  ASSERT_TRUE(valid(tree));
  tree.nodes = {make_node(Node::OR)};
  ASSERT_TRUE(valid(tree));
}

TEST(validate_tree, unknown_node_type)
{
  Tree tree;
  tree.nodes.push_back(make_node(Node::UNKNOWN));
  ASSERT_FALSE(valid(tree));
  tree.nodes = {make_node(200)};
  ASSERT_FALSE(valid(tree));
}

TEST(validate_tree, depth_limit)
{
  ASSERT_TRUE(valid(not_chain(parser::pddl::kMaxNestingDepth)));
  ASSERT_FALSE(valid(not_chain(parser::pddl::kMaxNestingDepth + 1)));
  // Far deeper than the stack would take if this were recursive
  ASSERT_FALSE(valid(not_chain(200000)));
}

TEST(validate_tree, wide_trees_are_fine)
{
  for (int width : {255, 256, 257, 1000, 10000}) {
    Tree tree;
    tree.nodes.push_back(make_node(Node::AND));
    for (int i = 1; i <= width; i++) {
      tree.nodes[0].children.push_back(i);
      tree.nodes.push_back(make_node(Node::PREDICATE));
    }
    ASSERT_TRUE(valid(tree)) << width;
  }
}

TEST(validate_tree, unreachable_nodes_are_ignored)
{
  Tree tree;
  tree.nodes.push_back(make_node(Node::NOT, {1}));
  tree.nodes.push_back(make_node(Node::PREDICATE));
  tree.nodes.push_back(make_node(Node::NOT, {99}));  // never reached from the root
  ASSERT_TRUE(valid(tree));
}

TEST(is_valid_name, pddl_names)
{
  for (const std::string name : {"r", "r2d2", "Paco", "room_1", "room-1", "R1", "a-b_c-9"}) {
    ASSERT_TRUE(parser::pddl::isValidName(name)) << name;
  }
  for (const std::string name : {
    "", " ", "a b", "a)", "(a", "?x", "-a", "_a", "1room", "a;b", "a\tb", "a\nb", "a.b",
    "a,b", "a:b", "caf\xc3\xa9", " a", "a "})
  {
    ASSERT_FALSE(parser::pddl::isValidName(name)) << "[" << name << "]";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
