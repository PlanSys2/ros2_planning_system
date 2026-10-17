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

// The traversals of the plan graphs visit each node once: a ladder of 40 levels has 2^40 paths

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_executor/bt_builder_plugins/simple_bt_builder.hpp"
#include "plansys2_executor/bt_builder_plugins/stn_bt_builder.hpp"

namespace
{
constexpr int kLevels = 40;

class STNTest : public plansys2::STNBTBuilder
{
public:
  bool reaches(plansys2::Node::Ptr from, plansys2::Node::Ptr to) const
  {
    return check_paths(from, to);
  }
  void prune(plansys2::Node::Ptr current, plansys2::Node::Ptr previous) const
  {
    prune_paths(current, previous);
  }
  std::set<std::string> edges(plansys2::Node::Ptr node)
  {
    std::set<std::string> ret;
    get_flow_dotgraph(node, ret);
    return ret;
  }
};

class SimpleTest : public plansys2::SimpleBTBuilder
{
public:
  std::set<std::string> edges(plansys2::ActionNode::Ptr node)
  {
    std::set<std::string> ret;
    get_flow_dotgraph(node, ret);
    return ret;
  }
};

// Two nodes per level, each linked to both of the next one
std::vector<plansys2::Node::Ptr> stn_ladder()
{
  std::vector<plansys2::Node::Ptr> nodes;
  for (int i = 0; i < 2 * kLevels; i++) {
    nodes.push_back(plansys2::Node::make_shared(i));
    nodes.back()->action.expression = "(a" + std::to_string(i) + ")";
    nodes.back()->action.time = static_cast<float>(i / 2);
  }
  for (int level = 0; level + 1 < kLevels; level++) {
    for (int a = 0; a < 2; a++) {
      for (int b = 0; b < 2; b++) {
        auto from = nodes[2 * level + a];
        auto to = nodes[2 * (level + 1) + b];
        from->output_arcs.insert({to, 0.0, 1.0});
        to->input_arcs.insert({from, 0.0, 1.0});
      }
    }
  }
  return nodes;
}

template<typename F>
double seconds(F f)
{
  const auto start = std::chrono::steady_clock::now();
  f();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

TEST(graph_traversal, stn_paths_once_per_node)
{
  auto nodes = stn_ladder();
  STNTest builder;

  bool reached = false;
  bool unreached = true;
  const double took = seconds(
    [&]() {
      reached = builder.reaches(nodes.back(), nodes.front());
      // The other node of the first level does not reach the first one
      unreached = builder.reaches(nodes[1], nodes.front());
    });
  EXPECT_TRUE(reached);
  EXPECT_FALSE(unreached);
  EXPECT_LT(took, 1.0);

  std::set<std::string> edges;
  EXPECT_LT(seconds([&]() {edges = builder.edges(nodes.front());}), 1.0);
  EXPECT_EQ(edges.size(), 2u + 4u * (kLevels - 2));

  // A direct arc from the first node to the last one, taken out through the ladder
  nodes.front()->output_arcs.insert({nodes.back(), 0.0, 1.0});
  nodes.back()->input_arcs.insert({nodes.front(), 0.0, 1.0});
  EXPECT_LT(seconds([&]() {builder.prune(nodes.back(), nodes[nodes.size() - 3]);}), 1.0);
  EXPECT_EQ(nodes.back()->input_arcs.size(), 1u);
  EXPECT_EQ(nodes.front()->output_arcs.size(), 2u);
}

TEST(graph_traversal, simple_dotgraph_once_per_node)
{
  std::vector<plansys2::ActionNode::Ptr> nodes;
  for (int i = 0; i < 2 * kLevels; i++) {
    nodes.push_back(plansys2::ActionNode::make_shared());
    nodes.back()->node_num = i;
  }
  for (int level = 0; level + 1 < kLevels; level++) {
    for (int a = 0; a < 2; a++) {
      for (int b = 0; b < 2; b++) {
        nodes[2 * level + a]->out_arcs.push_back(nodes[2 * (level + 1) + b]);
      }
    }
  }

  SimpleTest builder;
  std::set<std::string> edges;
  EXPECT_LT(seconds([&]() {edges = builder.edges(nodes.front());}), 1.0);
  EXPECT_EQ(edges.size(), 2u + 4u * (kLevels - 2));

  for (auto & node : nodes) {
    node->out_arcs.clear();
  }
}
