// Copyright 2025 Intelligent Robotics Lab
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
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/DerivedResolutionGraph.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

class GraphExportToDOTTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Clean up any previous test file
    std::remove(filename.c_str());
  }
  std::string filename = "/tmp/test_graph.dot";
};

TEST_F(GraphExportToDOTTest, export_dot_file)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?a))");

  plansys2::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicate ?a))");

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");

  plansys2::DerivedResolutionGraph graph;
  graph.addEdge(predA, inferredA);
  graph.addEdge(predA, inferredAA2);

  graph.addEdge(inferredA, inferredAA);
  graph.addEdge(inferredA, inferredAB);

  graph.addEdge(predB, inferredB);
  graph.addEdge(inferredB, inferredBB);
  graph.addEdge(inferredB, inferredAB);

  ASSERT_EQ(graph.getEdgeNumber(), 7);
  ASSERT_EQ(graph.getNodeNumber(), 8);
  ASSERT_EQ(graph.getRootNumber(), 2);

  graph.exportToDOT(filename);
  std::ifstream file(filename);
  ASSERT_TRUE(file.is_open());
}

TEST(graph_test, depth_first_traverse)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?a))");

  plansys2::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicate ?a))");

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");

  plansys2::DerivedResolutionGraph graph;
  graph.addEdge(predA, inferredA);
  graph.addEdge(predA, inferredAA2);

  graph.addEdge(inferredA, inferredAA);
  graph.addEdge(inferredA, inferredAB);

  graph.addEdge(predB, inferredB);
  graph.addEdge(inferredB, inferredBB);
  graph.addEdge(inferredB, inferredAB);

  ASSERT_EQ(graph.getEdgeNumber(), 7);
  ASSERT_EQ(graph.getNodeNumber(), 8);
  ASSERT_EQ(graph.getRootNumber(), 2);
  ASSERT_TRUE(graph.getRoots().find(predA) != graph.getRoots().end());
  ASSERT_TRUE(graph.getRoots().find(predB) != graph.getRoots().end());

  std::vector<std::string> predA_children;
  std::function<void(const plansys2::NodeVariant &)> func =
    [&predA_children](const plansys2::NodeVariant & node) {
      predA_children.push_back(node.getNodeName());
    };

  graph.depthFirstTraverse(predA, func);

  ASSERT_EQ(predA_children.size(), 5);
  ASSERT_EQ(predA_children[0], "predicateA");
  ASSERT_EQ(predA_children[1], "inferredAA");
  ASSERT_EQ(predA_children[2], "inferredA");
  ASSERT_EQ(predA_children[3], "inferredAB");
  ASSERT_EQ(predA_children[4], "inferredAA");

  std::vector<std::string> predB_children;
  std::function<void(const plansys2::NodeVariant &)> funcB =
    [&predB_children](const plansys2::NodeVariant & node) {
      predB_children.push_back(node.getNodeName());
    };
  graph.depthFirstTraverse(predB, funcB);

  ASSERT_EQ(predB_children.size(), 4);
  ASSERT_EQ(predB_children[0], "predicateB");
  ASSERT_EQ(predB_children[1], "inferredB");
  ASSERT_EQ(predB_children[2], "inferredAB");
  ASSERT_EQ(predB_children[3], "inferredBB");

  std::vector<std::string> pred_children_deps;
  std::function<void(const plansys2::NodeVariant &)> func_deps =
    [&pred_children_deps](const plansys2::NodeVariant & node) {
      pred_children_deps.push_back(node.getNodeName());
    };

  graph.depthFirstTraverse(predA, func_deps, true);
  ASSERT_EQ(pred_children_deps.size(), 4);
  ASSERT_EQ(pred_children_deps[0], "predicateA");
  ASSERT_EQ(pred_children_deps[1], "inferredAA");
  ASSERT_EQ(pred_children_deps[2], "inferredA");
  ASSERT_EQ(pred_children_deps[3], "inferredAA");

  graph.depthFirstTraverse(predB, func_deps, true);
  ASSERT_EQ(pred_children_deps.size(), 7);
  ASSERT_EQ(pred_children_deps[0], "predicateA");
  ASSERT_EQ(pred_children_deps[1], "inferredAA");
  ASSERT_EQ(pred_children_deps[2], "inferredA");
  ASSERT_EQ(pred_children_deps[3], "inferredAA");
  ASSERT_EQ(pred_children_deps[4], "predicateB");
  ASSERT_EQ(pred_children_deps[5], "inferredB");
  ASSERT_EQ(pred_children_deps[6], "inferredBB");

  pred_children_deps.clear();
  std::unordered_set<plansys2::NodeVariant> visited;
  graph.depthFirstTraverse(predA, func_deps, visited, true);
  graph.depthFirstTraverse(predB, func_deps, visited, true);
  ASSERT_EQ(pred_children_deps.size(), 8);
  ASSERT_EQ(pred_children_deps[0], "predicateA");
  ASSERT_EQ(pred_children_deps[1], "inferredAA");
  ASSERT_EQ(pred_children_deps[2], "inferredA");
  ASSERT_EQ(pred_children_deps[3], "inferredAA");
  ASSERT_EQ(pred_children_deps[4], "predicateB");
  ASSERT_EQ(pred_children_deps[5], "inferredB");
  ASSERT_EQ(pred_children_deps[6], "inferredAB");
  ASSERT_EQ(pred_children_deps[7], "inferredBB");

  std::vector<std::string> all_nodes;
  auto func_all = [&all_nodes](const plansys2::NodeVariant & node) {
      all_nodes.push_back(node.getNodeName());
    };
  graph.depthFirstTraverseFromNodes(func_all, true, {predA, predB});
  ASSERT_EQ(all_nodes.size(), 8);
  ASSERT_EQ(all_nodes[0], "predicateB");
  ASSERT_EQ(all_nodes[1], "inferredB");
  ASSERT_EQ(all_nodes[2], "inferredBB");
  ASSERT_EQ(all_nodes[3], "predicateA");
  ASSERT_EQ(all_nodes[4], "inferredA");
  ASSERT_EQ(all_nodes[5], "inferredAA");
  ASSERT_EQ(all_nodes[6], "inferredAB");
  ASSERT_EQ(all_nodes[7], "inferredAA");

  all_nodes.clear();
  graph.depthFirstTraverseFromNodes(func_all, true, {});
  ASSERT_EQ(all_nodes.size(), 8);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredA");
  ASSERT_EQ(all_nodes[2], "inferredAA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "predicateB");
  ASSERT_EQ(all_nodes[5], "inferredB");
  ASSERT_EQ(all_nodes[6], "inferredBB");
  ASSERT_EQ(all_nodes[7], "inferredAB");

  all_nodes.clear();
  graph.depthFirstTraverseAll(func_all, true);
  ASSERT_EQ(all_nodes.size(), 8);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredA");
  ASSERT_EQ(all_nodes[2], "inferredAA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "predicateB");
  ASSERT_EQ(all_nodes[5], "inferredB");
  ASSERT_EQ(all_nodes[6], "inferredBB");
  ASSERT_EQ(all_nodes[7], "inferredAB");

  plansys2::Predicate nodeA = parser::pddl::fromStringPredicate("(A)");
  plansys2::Predicate nodeB = parser::pddl::fromStringPredicate("(B)");
  plansys2::Predicate nodeC = parser::pddl::fromStringPredicate("(C)");
  plansys2::Predicate nodeD = parser::pddl::fromStringPredicate("(D)");
  plansys2::Predicate nodeE = parser::pddl::fromStringPredicate("(E)");
  plansys2::Predicate nodeF = parser::pddl::fromStringPredicate("(F)");
  plansys2::Predicate nodeG = parser::pddl::fromStringPredicate("(G)");
  plansys2::Predicate nodeH = parser::pddl::fromStringPredicate("(H)");
  plansys2::Predicate nodeI = parser::pddl::fromStringPredicate("(I)");
  plansys2::Predicate nodeJ = parser::pddl::fromStringPredicate("(J)");
  plansys2::Predicate nodeK = parser::pddl::fromStringPredicate("(K)");
  plansys2::Predicate nodeL = parser::pddl::fromStringPredicate("(L)");

  // DerivedResolutionGraph
  plansys2::DerivedResolutionGraph graph2;

  // Main component
  graph2.addEdge(nodeA, nodeB);
  graph2.addEdge(nodeA, nodeC);
  graph2.addEdge(nodeB, nodeE);
  graph2.addEdge(nodeC, nodeE);
  graph2.addEdge(nodeC, nodeG);
  graph2.addEdge(nodeD, nodeG);
  graph2.addEdge(nodeF, nodeD);
  graph2.addEdge(nodeE, nodeH);
  graph2.addEdge(nodeG, nodeH);
  graph2.addEdge(nodeG, nodeI);
  graph2.addEdge(nodeI, nodeJ);

  // Disconnected cycle
  graph2.addEdge(nodeK, nodeL);
  graph2.addEdge(nodeL, nodeK);

  all_nodes.clear();
  graph2.depthFirstTraverseFromNodes(func_all, false, {nodeA, nodeF, nodeK});
  ASSERT_EQ(all_nodes.size(), 12);
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "A") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "B") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "C") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "D") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "E") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "F") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "G") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "H") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "I") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "J") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "K") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "L") != all_nodes.end());

  all_nodes.clear();
  graph2.depthFirstTraverseFromNodes(func_all, true, {nodeA, nodeF, nodeK});
  ASSERT_EQ(all_nodes.size(), 10);  //  Nodes K and L should not be all_nodes
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "A") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "B") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "C") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "D") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "E") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "F") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "G") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "H") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "I") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "J") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "K") == all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "L") == all_nodes.end());

  all_nodes.clear();
  graph2.depthFirstTraverseFromNodes(func_all, true, {nodeA});
  ASSERT_EQ(all_nodes.size(), 4);
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "A") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "B") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "C") != all_nodes.end());
  ASSERT_TRUE(std::find(all_nodes.begin(), all_nodes.end(), "E") != all_nodes.end());
}

TEST(graph_test, single_node)
{
  plansys2::Predicate nodeA = parser::pddl::fromStringPredicate("(A)");

  plansys2::DerivedResolutionGraph graph;
  graph.addEdge(nodeA, nodeA);  // Self-loop

  std::vector<std::string> visited;
  graph.depthFirstTraverse(
    nodeA, [&](const plansys2::NodeVariant & n) {visited.push_back(n.getNodeName());});

  // Only the single node should be visited
  ASSERT_EQ(visited.size(), 1);
  ASSERT_EQ(visited[0], "A");

  visited.clear();
  graph.depthFirstTraverse(
    nodeA, [&](const plansys2::NodeVariant & n) {visited.push_back(n.getNodeName());}, true);
  ASSERT_EQ(visited.size(), 0);
}

TEST(graph_test, dfs_handles_cycle)
{
  plansys2::Predicate nodeA = parser::pddl::fromStringPredicate("(A)");
  plansys2::Predicate nodeB = parser::pddl::fromStringPredicate("(B)");
  plansys2::Predicate nodeC = parser::pddl::fromStringPredicate("(C)");

  plansys2::DerivedResolutionGraph graph;
  graph.addEdge(nodeA, nodeB);
  graph.addEdge(nodeB, nodeC);
  graph.addEdge(nodeC, nodeA);  // Creates a cycle

  std::vector<std::string> visited;
  graph.depthFirstTraverse(
    nodeA, [&](const plansys2::NodeVariant & n) {visited.push_back(n.getNodeName());});

  // All nodes should be visited exactly once
  ASSERT_EQ(visited.size(), 3);
  ASSERT_TRUE(std::find(visited.begin(), visited.end(), "A") != visited.end());
  ASSERT_TRUE(std::find(visited.begin(), visited.end(), "B") != visited.end());
  ASSERT_TRUE(std::find(visited.begin(), visited.end(), "C") != visited.end());
}

TEST(graph_test, empty_graph)
{
  plansys2::DerivedResolutionGraph graph;

  // No nodes or edges should exist
  ASSERT_EQ(graph.getNodeNumber(), 0);
  ASSERT_EQ(graph.getEdgeNumber(), 0);
  ASSERT_EQ(graph.getRootNumber(), 0);

  std::vector<std::string> visited;
  graph.depthFirstTraverseAll(
    [&](const plansys2::NodeVariant & n) {visited.push_back(n.getNodeName());});

  // No nodes should be visited
  ASSERT_TRUE(visited.empty());
}

TEST(graph_test, back_traverse)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?a))");

  plansys2::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicate ?a))");

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");

  plansys2::DerivedResolutionGraph graph;
  graph.addEdge(predA, inferredA);
  graph.addEdge(predA, inferredAA2);

  graph.addEdge(inferredA, inferredAA);
  graph.addEdge(inferredA, inferredAB);

  graph.addEdge(predB, inferredB);
  graph.addEdge(inferredB, inferredBB);
  graph.addEdge(inferredB, inferredAB);

  ASSERT_EQ(graph.getEdgeNumber(), 7);
  ASSERT_EQ(graph.getNodeNumber(), 8);
  ASSERT_EQ(graph.getRootNumber(), 2);
  ASSERT_TRUE(graph.getRoots().find(predA) != graph.getRoots().end());
  ASSERT_TRUE(graph.getRoots().find(predB) != graph.getRoots().end());

  std::vector<std::string> node_parents;
  std::function<void(const plansys2::NodeVariant &)> func =
    [&node_parents](const plansys2::NodeVariant & node) {
      node_parents.push_back(node.getNodeName());
    };

  graph.backtrackTraverse(inferredAA, func);
  ASSERT_EQ(node_parents.size(), 3);
  ASSERT_EQ(node_parents[0], "inferredAA");
  ASSERT_EQ(node_parents[1], "inferredA");
  ASSERT_EQ(node_parents[2], "predicateA");

  node_parents.clear();
  graph.backtrackTraverse(inferredAA2, func);
  ASSERT_EQ(node_parents.size(), 2);
  ASSERT_EQ(node_parents[0], "inferredAA");
  ASSERT_EQ(node_parents[1], "predicateA");

  node_parents.clear();
  graph.backtrackTraverse(inferredAB, func);
  ASSERT_EQ(node_parents.size(), 5);
  ASSERT_EQ(node_parents[0], "inferredAB");
  ASSERT_EQ(node_parents[1], "inferredA");
  ASSERT_EQ(node_parents[2], "predicateA");
  ASSERT_EQ(node_parents[3], "inferredB");
  ASSERT_EQ(node_parents[4], "predicateB");

  node_parents.clear();
  graph.backtrackTraverse(inferredB, func);
  ASSERT_EQ(node_parents.size(), 2);
  ASSERT_EQ(node_parents[0], "inferredB");
  ASSERT_EQ(node_parents[1], "predicateB");

  node_parents.clear();
  graph.backtrackTraverse(inferredA, func);
  ASSERT_EQ(node_parents.size(), 2);
  ASSERT_EQ(node_parents[0], "inferredA");
  ASSERT_EQ(node_parents[1], "predicateA");

  node_parents.clear();
  graph.backtrackTraverse(predA, func);
  ASSERT_EQ(node_parents.size(), 1);
  ASSERT_EQ(node_parents[0], "predicateA");

  node_parents.clear();
  graph.backtrackTraverse(predB, func);
  ASSERT_EQ(node_parents.size(), 1);
  ASSERT_EQ(node_parents[0], "predicateB");

  plansys2::Predicate nodeA = parser::pddl::fromStringPredicate("(A)");
  plansys2::Predicate nodeB = parser::pddl::fromStringPredicate("(B)");
  plansys2::Predicate nodeC = parser::pddl::fromStringPredicate("(C)");
  plansys2::Predicate nodeD = parser::pddl::fromStringPredicate("(D)");
  plansys2::Predicate nodeE = parser::pddl::fromStringPredicate("(E)");
  plansys2::Predicate nodeF = parser::pddl::fromStringPredicate("(F)");
  plansys2::Predicate nodeG = parser::pddl::fromStringPredicate("(G)");
  plansys2::Predicate nodeH = parser::pddl::fromStringPredicate("(H)");
  plansys2::Predicate nodeI = parser::pddl::fromStringPredicate("(I)");
  plansys2::Predicate nodeJ = parser::pddl::fromStringPredicate("(J)");
  plansys2::Predicate nodeK = parser::pddl::fromStringPredicate("(K)");
  plansys2::Predicate nodeL = parser::pddl::fromStringPredicate("(L)");

  // DerivedResolutionGraph
  plansys2::DerivedResolutionGraph graph2;

  // Main component
  graph2.addEdge(nodeA, nodeB);
  graph2.addEdge(nodeA, nodeC);
  graph2.addEdge(nodeB, nodeE);
  graph2.addEdge(nodeC, nodeE);
  graph2.addEdge(nodeC, nodeG);
  graph2.addEdge(nodeD, nodeG);
  graph2.addEdge(nodeF, nodeD);
  graph2.addEdge(nodeE, nodeH);
  graph2.addEdge(nodeG, nodeH);
  graph2.addEdge(nodeG, nodeI);
  graph2.addEdge(nodeI, nodeJ);

  // Disconnected cycle
  graph2.addEdge(nodeK, nodeL);
  graph2.addEdge(nodeL, nodeK);

  node_parents.clear();
  graph2.backtrackTraverse(nodeJ, func);
  ASSERT_EQ(node_parents.size(), 7);
  ASSERT_EQ(node_parents[0], "J");
  ASSERT_EQ(node_parents[1], "I");
  ASSERT_EQ(node_parents[2], "G");
  ASSERT_EQ(node_parents[3], "C");
  ASSERT_EQ(node_parents[4], "A");
  ASSERT_EQ(node_parents[5], "D");
  ASSERT_EQ(node_parents[6], "F");

  node_parents.clear();
  graph2.backtrackTraverse(nodeH, func);
  ASSERT_EQ(node_parents.size(), 8);
  ASSERT_EQ(node_parents[0], "H");
  ASSERT_EQ(node_parents[1], "E");
  ASSERT_EQ(node_parents[2], "B");
  ASSERT_EQ(node_parents[3], "A");
  ASSERT_EQ(node_parents[4], "C");
  ASSERT_EQ(node_parents[5], "G");
  ASSERT_EQ(node_parents[6], "D");
  ASSERT_EQ(node_parents[7], "F");

  node_parents.clear();
  graph2.backtrackTraverse(nodeL, func);
  ASSERT_EQ(node_parents.size(), 2);
  ASSERT_EQ(node_parents[0], "L");
  ASSERT_EQ(node_parents[1], "K");
}

TEST(graph_test, get_subgraph)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?a))");

  plansys2::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicate ?a))");

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");

  plansys2::DerivedResolutionGraph graph;
  graph.addEdge(predA, inferredA);
  graph.addEdge(predA, inferredAA2);

  graph.addEdge(inferredA, inferredAA);
  graph.addEdge(inferredA, inferredAB);

  graph.addEdge(predB, inferredB);
  graph.addEdge(inferredB, inferredBB);
  graph.addEdge(inferredB, inferredAB);

  auto sub_graph_predA = graph.getSubGraphFromNodes({predA});
  ASSERT_EQ(sub_graph_predA.getEdgeNumber(), 4);
  ASSERT_EQ(sub_graph_predA.getNodeNumber(), 5);
  ASSERT_EQ(sub_graph_predA.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_predA.getRoots().find(predA) != sub_graph_predA.getRoots().end());

  auto sub_graph_predB = graph.getSubGraphFromNodes({predB});
  ASSERT_EQ(sub_graph_predB.getEdgeNumber(), 3);
  ASSERT_EQ(sub_graph_predB.getNodeNumber(), 4);
  ASSERT_EQ(sub_graph_predB.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_predB.getRoots().find(predB) != sub_graph_predB.getRoots().end());

  auto sub_graph_predA_predB = graph.getSubGraphFromNodes({predA, predB});
  ASSERT_EQ(sub_graph_predA_predB.getEdgeNumber(), 7);
  ASSERT_EQ(sub_graph_predA_predB.getNodeNumber(), 8);
  ASSERT_EQ(sub_graph_predA_predB.getRootNumber(), 2);
  ASSERT_TRUE(
    sub_graph_predA_predB.getRoots().find(predA) != sub_graph_predA_predB.getRoots().end());
  ASSERT_TRUE(
    sub_graph_predA_predB.getRoots().find(predB) != sub_graph_predA_predB.getRoots().end());

  auto sub_graph_predA_infB = graph.getSubGraphFromNodes({predA, inferredB});
  ASSERT_EQ(sub_graph_predA_infB.getEdgeNumber(), 6);
  ASSERT_EQ(sub_graph_predA_infB.getNodeNumber(), 7);
  ASSERT_EQ(sub_graph_predA_infB.getRootNumber(), 2);
  ASSERT_TRUE(sub_graph_predA_infB.getRoots().find(predA) != sub_graph_predA_infB.getRoots().end());
  ASSERT_TRUE(
    sub_graph_predA_infB.getRoots().find(inferredB) != sub_graph_predA_infB.getRoots().end());

  auto sub_graph_predA_infBB = graph.getSubGraphFromNodes({predA, inferredBB});
  ASSERT_EQ(sub_graph_predA_infBB.getEdgeNumber(), 4);
  ASSERT_EQ(sub_graph_predA_infBB.getNodeNumber(), 6);
  ASSERT_EQ(sub_graph_predA_infBB.getRootNumber(), 2);
  ASSERT_TRUE(
    sub_graph_predA_infBB.getRoots().find(predA) != sub_graph_predA_infBB.getRoots().end());
  ASSERT_TRUE(
    sub_graph_predA_infBB.getRoots().find(inferredBB) != sub_graph_predA_infBB.getRoots().end());

  auto sub_graph_infA = graph.getSubGraphFromNodes({inferredA});
  ASSERT_EQ(sub_graph_infA.getEdgeNumber(), 2);
  ASSERT_EQ(sub_graph_infA.getNodeNumber(), 3);
  ASSERT_EQ(sub_graph_infA.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_infA.getRoots().find(inferredA) != sub_graph_infA.getRoots().end());

  auto sub_graph_infAA = graph.getSubGraphFromNodes({inferredAA});
  ASSERT_EQ(sub_graph_infAA.getEdgeNumber(), 0);
  ASSERT_EQ(sub_graph_infAA.getNodeNumber(), 1);
  ASSERT_EQ(sub_graph_infAA.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_infAA.getRoots().find(inferredAA) != sub_graph_infAA.getRoots().end());

  auto sub_graph_infAA2 = graph.getSubGraphFromNodes({inferredAA2});
  ASSERT_EQ(sub_graph_infAA2.getEdgeNumber(), 0);
  ASSERT_EQ(sub_graph_infAA2.getNodeNumber(), 1);
  ASSERT_EQ(sub_graph_infAA2.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_infAA2.getRoots().find(inferredAA2) != sub_graph_infAA2.getRoots().end());

  auto sub_graph_infAB = graph.getSubGraphFromNodes({inferredAB});
  ASSERT_EQ(sub_graph_infAB.getEdgeNumber(), 0);
  ASSERT_EQ(sub_graph_infAB.getNodeNumber(), 1);
  ASSERT_EQ(sub_graph_infAB.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_infAB.getRoots().find(inferredAB) != sub_graph_infAB.getRoots().end());

  auto sub_graph_infB = graph.getSubGraphFromNodes({inferredB});
  ASSERT_EQ(sub_graph_infB.getEdgeNumber(), 2);
  ASSERT_EQ(sub_graph_infB.getNodeNumber(), 3);
  ASSERT_EQ(sub_graph_infB.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_infB.getRoots().find(inferredB) != sub_graph_infB.getRoots().end());

  auto sub_graph_infBB = graph.getSubGraphFromNodes({inferredBB});
  ASSERT_EQ(sub_graph_infBB.getEdgeNumber(), 0);
  ASSERT_EQ(sub_graph_infBB.getNodeNumber(), 1);
  ASSERT_EQ(sub_graph_infBB.getRootNumber(), 1);
  ASSERT_TRUE(sub_graph_infBB.getRoots().find(inferredBB) != sub_graph_infBB.getRoots().end());
}

TEST(graph_test, graph_derived_constructor)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  std::vector<plansys2_msgs::msg::Derived> derived_predicates;
  plansys2_msgs::msg::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredA);

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");
  derived_predicates.push_back(inferredB);

  plansys2_msgs::msg::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?aa)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?aa))");
  derived_predicates.push_back(inferredAA);

  plansys2_msgs::msg::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredAA2);

  plansys2_msgs::msg::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");
  derived_predicates.push_back(inferredBB);

  plansys2_msgs::msg::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");
  derived_predicates.push_back(inferredAB);

  plansys2::DerivedResolutionGraph graph(derived_predicates);

  auto nodes = graph.getNodesNames();
  auto roots = graph.getRoots();
  ASSERT_EQ(graph.getEdgeNumber(), 7);
  ASSERT_EQ(graph.getNodeNumber(), 8);
  ASSERT_EQ(graph.getRootNumber(), 2);
  ASSERT_TRUE(graph.getRoots().find(predA) != graph.getRoots().end());
  ASSERT_TRUE(graph.getRoots().find(predB) != graph.getRoots().end());

  std::vector<std::pair<std::string, std::string>> predA_children;
  std::function<void(const plansys2::NodeVariant &)> func = [&predA_children](auto & node) {
      predA_children.push_back({node.getNodeName(), node.getNodeType()});
    };

  graph.depthFirstTraverse(predA, func);

  ASSERT_EQ(predA_children.size(), 5);
  ASSERT_EQ(predA_children[0].first, "predicateA");
  ASSERT_EQ(predA_children[0].second, "predicate");
  ASSERT_EQ(predA_children[1].first, "inferredAA");
  ASSERT_EQ(predA_children[1].second, "derived");
  ASSERT_EQ(predA_children[2].first, "inferredA");
  ASSERT_EQ(predA_children[2].second, "derived");
  ASSERT_EQ(predA_children[3].first, "inferredAB");
  ASSERT_EQ(predA_children[3].second, "derived");
  ASSERT_EQ(predA_children[4].first, "inferredAA");
  ASSERT_EQ(predA_children[4].second, "derived");

  std::vector<std::string> predB_children;
  std::function<void(const plansys2::NodeVariant &)> funcB =
    [&predB_children](const plansys2::NodeVariant & node) {
      predB_children.push_back(node.getNodeName());
    };
  graph.depthFirstTraverse(predB, funcB);

  ASSERT_EQ(predB_children.size(), 4);
  ASSERT_EQ(predB_children[0], "predicateB");
  ASSERT_EQ(predB_children[1], "inferredB");
  ASSERT_EQ(predB_children[2], "inferredAB");
  ASSERT_EQ(predB_children[3], "inferredBB");

  auto all_nodes = graph.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(all_nodes.size(), 6);
  ASSERT_EQ(all_nodes[0].predicate.name, "inferredA");
  ASSERT_EQ(all_nodes[1].predicate.name, "inferredAA");
  ASSERT_EQ(all_nodes[2].predicate.name, "inferredAA");
  ASSERT_EQ(all_nodes[3].predicate.name, "inferredB");
  ASSERT_EQ(all_nodes[4].predicate.name, "inferredBB");
  ASSERT_EQ(all_nodes[5].predicate.name, "inferredAB");

  auto nodes_root_predAB = graph.getDerivedPredicatesDepthFirst({predA, predB});
  ASSERT_EQ(nodes_root_predAB.size(), 6);
  ASSERT_EQ(nodes_root_predAB[0].predicate.name, "inferredB");
  ASSERT_EQ(nodes_root_predAB[1].predicate.name, "inferredBB");
  ASSERT_EQ(nodes_root_predAB[2].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predAB[3].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predAB[4].predicate.name, "inferredAB");
  ASSERT_EQ(nodes_root_predAB[5].predicate.name, "inferredAA");

  auto nodes_root_predA = graph.getDerivedPredicatesDepthFirst({predA});
  ASSERT_EQ(nodes_root_predA.size(), 4);
  ASSERT_EQ(nodes_root_predA[0].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predA[1].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predA[2].predicate.name, "inferredAB");
  ASSERT_EQ(nodes_root_predA[3].predicate.name, "inferredAA");

  auto nodes_root_predB = graph.getDerivedPredicatesDepthFirst({predB});
  ASSERT_EQ(nodes_root_predB.size(), 3);
  ASSERT_EQ(nodes_root_predB[0].predicate.name, "inferredB");
  ASSERT_EQ(nodes_root_predB[1].predicate.name, "inferredAB");
  ASSERT_EQ(nodes_root_predB[2].predicate.name, "inferredBB");

  plansys2::Predicate predA_unified = parser::pddl::fromStringPredicate("(predicateA instance)");
  auto nodes_root_predA_unified = graph.getDerivedPredicatesDepthFirst({predA_unified});
  ASSERT_EQ(nodes_root_predA_unified.size(), 4);
  ASSERT_EQ(nodes_root_predA_unified[0].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predA_unified[1].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predA_unified[2].predicate.name, "inferredAB");
  ASSERT_EQ(nodes_root_predA_unified[3].predicate.name, "inferredAA");

  plansys2_msgs::msg::Derived inferred_exists;
  inferred_exists.predicate = parser::pddl::fromStringPredicate("(inferredExists ?a)");
  inferred_exists.preconditions = parser::pddl::fromString(
    "(and (predicateA ?a) (exists (?b) (and (inferredA ?a)(inferredB ?b)(inferredAB ?a ?b)(not(=?a "
    "?b)))))");
  derived_predicates.push_back(inferred_exists);
  plansys2_msgs::msg::Derived inferred_exists_2;
  inferred_exists_2.predicate = parser::pddl::fromStringPredicate("(inferredExists2 ?a)");
  inferred_exists_2.preconditions = parser::pddl::fromString(
    "(and (exists (?b) (and (inferredA ?a)(inferredB ?b)(inferredAB ?a ?b)(not(=?a ?b))) (not "
    "(inferredExists ?a)) ))");
  derived_predicates.push_back(inferred_exists_2);

  plansys2::DerivedResolutionGraph graph_2(derived_predicates);
  predA_children.clear();
  graph_2.depthFirstTraverse(predA, func);

  ASSERT_EQ(predA_children.size(), 7);
  ASSERT_EQ(predA_children[0].first, "predicateA");
  ASSERT_EQ(predA_children[0].second, "predicate");
  ASSERT_EQ(predA_children[1].first, "inferredExists");
  ASSERT_EQ(predA_children[1].second, "derived");
  ASSERT_EQ(predA_children[2].first, "inferredExists2");
  ASSERT_EQ(predA_children[2].second, "derived");
  ASSERT_EQ(predA_children[3].first, "inferredAA");
  ASSERT_EQ(predA_children[3].second, "derived");
  ASSERT_EQ(predA_children[4].first, "inferredA");
  ASSERT_EQ(predA_children[4].second, "derived");
  ASSERT_EQ(predA_children[5].first, "inferredAB");
  ASSERT_EQ(predA_children[5].second, "derived");
  ASSERT_EQ(predA_children[6].first, "inferredAA");
  ASSERT_EQ(predA_children[6].second, "derived");

  nodes_root_predA = graph_2.getDerivedPredicatesDepthFirst({predA});
  ASSERT_EQ(nodes_root_predA.size(), 6);
  ASSERT_EQ(nodes_root_predA[0].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predA[1].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predA[2].predicate.name, "inferredAB");
  ASSERT_EQ(nodes_root_predA[3].predicate.name, "inferredExists");
  ASSERT_EQ(nodes_root_predA[4].predicate.name, "inferredExists2");
  ASSERT_EQ(nodes_root_predA[5].predicate.name, "inferredAA");

  std::vector<std::pair<std::string, std::string>> inferredB_children;
  std::function<void(const plansys2::NodeVariant &)> func_inferredB =
    [&inferredB_children](auto & node) {
      inferredB_children.push_back({node.getNodeName(), node.getNodeType()});
    };
  graph_2.depthFirstTraverse(inferredB, func_inferredB);

  ASSERT_EQ(inferredB_children.size(), 5);
  ASSERT_EQ(inferredB_children[0].first, "inferredB");
  ASSERT_EQ(inferredB_children[0].second, "derived");
  ASSERT_EQ(inferredB_children[1].first, "inferredExists2");
  ASSERT_EQ(inferredB_children[1].second, "derived");
  ASSERT_EQ(inferredB_children[2].first, "inferredExists");
  ASSERT_EQ(inferredB_children[2].second, "derived");
  ASSERT_EQ(inferredB_children[3].first, "inferredAB");
  ASSERT_EQ(inferredB_children[3].second, "derived");
  ASSERT_EQ(inferredB_children[4].first, "inferredBB");
  ASSERT_EQ(inferredB_children[4].second, "derived");
}

TEST(graph_test, graph_derived_action)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  std::vector<plansys2::Derived> derived_predicates;
  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredA);

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");
  derived_predicates.push_back(inferredB);

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?aa)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?aa))");
  derived_predicates.push_back(inferredAA);

  plansys2::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredAA2);

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");
  derived_predicates.push_back(inferredBB);

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");
  derived_predicates.push_back(inferredAB);

  plansys2::DerivedResolutionGraph graph(derived_predicates);

  plansys2::Action actionA;
  actionA.name = "actionA";
  actionA.parameters.push_back(parser::pddl::fromStringParam("a"));

  plansys2_msgs::msg::Tree actionA_preconditions;
  parser::pddl::fromString(actionA_preconditions, "(and (inferredAA a))");
  actionA.preconditions = actionA_preconditions;

  plansys2::Action actionB;
  actionB.name = "actionB";
  actionB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree actionB_preconditions;
  parser::pddl::fromString(actionB_preconditions, "(and (inferredBB b))");
  actionB.preconditions = actionB_preconditions;

  plansys2::Action actionAB;
  actionAB.name = "actionAB";
  actionAB.parameters.push_back(parser::pddl::fromStringParam("a"));
  actionAB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree actionAB_preconditions;
  parser::pddl::fromString(actionAB_preconditions, "(and (inferredAB a b))");
  actionAB.preconditions = actionAB_preconditions;

  graph.appendAction(actionA);
  graph.appendAction(actionB);
  graph.appendAction(actionAB);

  std::vector<std::string> all_nodes;
  auto func_all = [&all_nodes](const plansys2::NodeVariant & node) {
      all_nodes.push_back(node.getNodeName());
    };
  graph.depthFirstTraverseAll(func_all, true);

  ASSERT_EQ(graph.getPredicates().size(), 2);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 6);
  ASSERT_EQ(graph.getActions().size(), 3);
  ASSERT_EQ(graph.getFunctions().size(), 0);

  ASSERT_EQ(all_nodes.size(), 11);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredA");
  ASSERT_EQ(all_nodes[2], "inferredAA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "actionA");
  ASSERT_EQ(all_nodes[5], "predicateB");
  ASSERT_EQ(all_nodes[6], "inferredB");
  ASSERT_EQ(all_nodes[7], "inferredBB");
  ASSERT_EQ(all_nodes[8], "actionB");
  ASSERT_EQ(all_nodes[9], "inferredAB");
  ASSERT_EQ(all_nodes[10], "actionAB");

  std::vector<std::string> actionAB_parent_nodes;
  auto func_actionAB_parents = [&actionAB_parent_nodes](const plansys2::NodeVariant & node) {
      actionAB_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(actionAB, func_actionAB_parents);

  ASSERT_EQ(actionAB_parent_nodes.size(), 6);
  ASSERT_EQ(actionAB_parent_nodes[0], "actionAB");
  ASSERT_EQ(actionAB_parent_nodes[1], "inferredAB");
  ASSERT_EQ(actionAB_parent_nodes[2], "inferredA");
  ASSERT_EQ(actionAB_parent_nodes[3], "predicateA");
  ASSERT_EQ(actionAB_parent_nodes[4], "inferredB");
  ASSERT_EQ(actionAB_parent_nodes[5], "predicateB");

  std::vector<std::string> actionA_parent_nodes;
  auto func_actionA_parents = [&actionA_parent_nodes](const plansys2::NodeVariant & node) {
      actionA_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(actionA, func_actionA_parents);

  ASSERT_EQ(actionA_parent_nodes.size(), 5);
  ASSERT_EQ(actionA_parent_nodes[0], "actionA");
  ASSERT_EQ(actionA_parent_nodes[1], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[2], "predicateA");
  ASSERT_EQ(actionA_parent_nodes[3], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[4], "inferredA");

  std::vector<std::string> actionB_parent_nodes;
  auto func_actionB_parents = [&actionB_parent_nodes](const plansys2::NodeVariant & node) {
      actionB_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(actionB, func_actionB_parents);

  ASSERT_EQ(actionB_parent_nodes.size(), 4);
  ASSERT_EQ(actionB_parent_nodes[0], "actionB");
  ASSERT_EQ(actionB_parent_nodes[1], "inferredBB");
  ASSERT_EQ(actionB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionB_parent_nodes[3], "predicateB");

  auto actionAB_parent_nodes_2 = graph.getDerivedPredicatesFromActions({actionAB});
  ASSERT_EQ(actionAB_parent_nodes_2.size(), 3);
  ASSERT_EQ(actionAB_parent_nodes_2[0].predicate.name, "inferredB");
  ASSERT_EQ(actionAB_parent_nodes_2[1].predicate.name, "inferredA");
  ASSERT_EQ(actionAB_parent_nodes_2[2].predicate.name, "inferredAB");

  auto graph_actionA = graph.pruneGraphToActions({actionA});

  all_nodes.clear();
  graph_actionA.depthFirstTraverseAll(func_all, true);

  ASSERT_EQ(all_nodes.size(), 5);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredAA");
  ASSERT_EQ(all_nodes[2], "inferredA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "actionA");

  actionA_parent_nodes.clear();
  graph_actionA.backtrackTraverse(actionA, func_actionA_parents);

  ASSERT_EQ(actionA_parent_nodes.size(), 5);
  ASSERT_EQ(actionA_parent_nodes[0], "actionA");
  ASSERT_EQ(actionA_parent_nodes[1], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[2], "inferredA");
  ASSERT_EQ(actionA_parent_nodes[3], "predicateA");
  ASSERT_EQ(actionA_parent_nodes[4], "inferredAA");

  auto derived_predicates_A = graph_actionA.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicates_A.size(), 3);
  ASSERT_EQ(derived_predicates_A[0].predicate.name, "inferredAA");
  ASSERT_EQ(derived_predicates_A[1].predicate.name, "inferredA");
  ASSERT_EQ(derived_predicates_A[2].predicate.name, "inferredAA");

  auto graph_actionB = graph.pruneGraphToActions({actionB});

  all_nodes.clear();
  graph_actionB.depthFirstTraverseAll(func_all, true);

  ASSERT_EQ(all_nodes.size(), 4);
  ASSERT_EQ(all_nodes[0], "predicateB");
  ASSERT_EQ(all_nodes[1], "inferredB");
  ASSERT_EQ(all_nodes[2], "inferredBB");
  ASSERT_EQ(all_nodes[3], "actionB");

  actionB_parent_nodes.clear();
  graph_actionB.backtrackTraverse(actionB, func_actionB_parents);

  ASSERT_EQ(actionB_parent_nodes.size(), 4);
  ASSERT_EQ(actionB_parent_nodes[0], "actionB");
  ASSERT_EQ(actionB_parent_nodes[1], "inferredBB");
  ASSERT_EQ(actionB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionB_parent_nodes[3], "predicateB");

  auto derived_predicates_B = graph_actionB.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicates_B.size(), 2);
  ASSERT_EQ(derived_predicates_B[0].predicate.name, "inferredB");
  ASSERT_EQ(derived_predicates_B[1].predicate.name, "inferredBB");

  auto graph_actionAB = graph.pruneGraphToActions({actionAB});

  all_nodes.clear();
  graph_actionAB.depthFirstTraverseAll(func_all, true);

  ASSERT_EQ(all_nodes.size(), 6);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredA");
  ASSERT_EQ(all_nodes[2], "predicateB");
  ASSERT_EQ(all_nodes[3], "inferredB");
  ASSERT_EQ(all_nodes[4], "inferredAB");
  ASSERT_EQ(all_nodes[5], "actionAB");

  actionAB_parent_nodes.clear();
  graph_actionAB.backtrackTraverse(actionAB, func_actionAB_parents);

  ASSERT_EQ(actionAB_parent_nodes.size(), 6);
  ASSERT_EQ(actionAB_parent_nodes[0], "actionAB");
  ASSERT_EQ(actionAB_parent_nodes[1], "inferredAB");
  ASSERT_EQ(actionAB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionAB_parent_nodes[3], "predicateB");
  ASSERT_EQ(actionAB_parent_nodes[4], "inferredA");
  ASSERT_EQ(actionAB_parent_nodes[5], "predicateA");

  auto derived_predicatesAB = graph_actionAB.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicatesAB.size(), 3);
  ASSERT_EQ(derived_predicatesAB[0].predicate.name, "inferredA");
  ASSERT_EQ(derived_predicatesAB[1].predicate.name, "inferredB");
  ASSERT_EQ(derived_predicatesAB[2].predicate.name, "inferredAB");

  auto graph_actionA_AB = graph.pruneGraphToActions({actionA, actionAB});

  all_nodes.clear();
  graph_actionA_AB.depthFirstTraverseAll(func_all, true);

  ASSERT_EQ(all_nodes.size(), 9);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredAA");
  ASSERT_EQ(all_nodes[2], "inferredA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "actionA");
  ASSERT_EQ(all_nodes[5], "predicateB");
  ASSERT_EQ(all_nodes[6], "inferredB");
  ASSERT_EQ(all_nodes[7], "inferredAB");
  ASSERT_EQ(all_nodes[8], "actionAB");

  actionA_parent_nodes.clear();
  graph_actionA_AB.backtrackTraverse(actionA, func_actionA_parents);

  ASSERT_EQ(actionA_parent_nodes.size(), 5);
  ASSERT_EQ(actionA_parent_nodes[0], "actionA");
  ASSERT_EQ(actionA_parent_nodes[1], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[2], "inferredA");
  ASSERT_EQ(actionA_parent_nodes[3], "predicateA");
  ASSERT_EQ(actionA_parent_nodes[4], "inferredAA");

  actionAB_parent_nodes.clear();
  graph_actionA_AB.backtrackTraverse(actionAB, func_actionAB_parents);

  ASSERT_EQ(actionAB_parent_nodes.size(), 6);
  ASSERT_EQ(actionAB_parent_nodes[0], "actionAB");
  ASSERT_EQ(actionAB_parent_nodes[1], "inferredAB");
  ASSERT_EQ(actionAB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionAB_parent_nodes[3], "predicateB");
  ASSERT_EQ(actionAB_parent_nodes[4], "inferredA");
  ASSERT_EQ(actionAB_parent_nodes[5], "predicateA");

  auto derived_predicates_A_AB = graph_actionA_AB.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicates_A_AB.size(), 5);
  ASSERT_EQ(derived_predicates_A_AB[0].predicate.name, "inferredAA");
  ASSERT_EQ(derived_predicates_A_AB[1].predicate.name, "inferredA");
  ASSERT_EQ(derived_predicates_A_AB[2].predicate.name, "inferredAA");
  ASSERT_EQ(derived_predicates_A_AB[3].predicate.name, "inferredB");
  ASSERT_EQ(derived_predicates_A_AB[4].predicate.name, "inferredAB");
}

TEST(graph_test, get_scc)
{
  std::vector<plansys2::Derived> derived_predicates;

  plansys2::Derived der1;
  der1.predicate = parser::pddl::fromStringPredicate("(der1 ?x)");
  der1.preconditions = parser::pddl::fromString("(and (pred1 ?x))");
  derived_predicates.push_back(der1);

  plansys2::Derived der2;
  der2.predicate = parser::pddl::fromStringPredicate("(der2 ?x)");
  der2.preconditions = parser::pddl::fromString("(or (pred2 ?x) (der3 ?x))");
  derived_predicates.push_back(der2);

  plansys2::Derived der3;
  der3.predicate = parser::pddl::fromStringPredicate("(der3 ?x)");
  der3.preconditions = parser::pddl::fromString("(or (der4 ?x) (pred3 ?x))");
  derived_predicates.push_back(der3);

  plansys2::Derived der4;
  der4.predicate = parser::pddl::fromStringPredicate("(der4 ?x)");
  der4.preconditions = parser::pddl::fromString("(or (der2 ?x) (pred4 ?x))");
  derived_predicates.push_back(der4);

  plansys2::Derived der5;
  der5.predicate = parser::pddl::fromStringPredicate("(der5 ?x)");
  der5.preconditions = parser::pddl::fromString("(or (der6 ?x) (pred2 ?x))");
  derived_predicates.push_back(der5);

  plansys2::Derived der6;
  der6.predicate = parser::pddl::fromStringPredicate("(der6 ?x)");
  der6.preconditions = parser::pddl::fromString("(exists (?y) (and (der5 ?y)))");
  derived_predicates.push_back(der6);

  plansys2::Derived der7;
  der7.predicate = parser::pddl::fromStringPredicate("(der7 ?x)");
  der7.preconditions = parser::pddl::fromString("(and (der2 ?x) (pred5 ?x))");
  derived_predicates.push_back(der7);

  plansys2::Derived der8;
  der8.predicate = parser::pddl::fromStringPredicate("(der8 ?x)");
  der8.preconditions = parser::pddl::fromString("(and (pred3 ?x))");
  derived_predicates.push_back(der8);

  plansys2::Derived der9;
  der9.predicate = parser::pddl::fromStringPredicate("(der9 ?x)");
  der9.preconditions = parser::pddl::fromString("(and (der4 ?x) (der6 ?x))");
  derived_predicates.push_back(der9);

  plansys2::DerivedResolutionGraph graph(derived_predicates);

  auto sccs = graph.computeSCCsTarjanDerivedPredicates();

  ASSERT_EQ(sccs.size(), 6);

  auto contains_scc = [](
    const std::vector<std::vector<plansys2::Derived>> & sccs,
    const std::vector<plansys2::Derived> & target) {
      for (const auto & scc : sccs) {
        if (scc.size() == target.size()) {
          std::unordered_set<plansys2::Derived> scc_set(scc.begin(), scc.end());
          std::unordered_set<plansys2::Derived> target_set(target.begin(), target.end());
          if (scc_set == target_set) {
            return true;
          }
        }
      }
      return false;
    };

  ASSERT_TRUE(contains_scc(sccs, {der2, der3, der4}));
  ASSERT_TRUE(contains_scc(sccs, {der5, der6}));
  ASSERT_TRUE(contains_scc(sccs, {der1}));
}

TEST(graph_test, graph_derived_action_suave)
{
  std::vector<plansys2::Derived> derived_predicates;

  plansys2::Derived inferred_action_1;
  inferred_action_1.predicate = parser::pddl::fromStringPredicate("(inferred-Action ?x)");
  inferred_action_1.preconditions = parser::pddl::fromString("(and (Action ?x))");
  derived_predicates.push_back(inferred_action_1);

  plansys2::Derived inferred_action_2;
  inferred_action_2.predicate = parser::pddl::fromStringPredicate("(inferred-Action ?x)");
  inferred_action_2.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-RequiresF ?x ?y)))");
  derived_predicates.push_back(inferred_action_2);

  plansys2::Derived inferred_c_status;
  inferred_c_status.predicate = parser::pddl::fromStringPredicate("(inferred-C_status ?x ?y)");
  inferred_c_status.preconditions = parser::pddl::fromString("(and (c_status ?x ?y))");
  derived_predicates.push_back(inferred_c_status);

  plansys2::Derived inferred_component;
  inferred_component.predicate = parser::pddl::fromStringPredicate("(inferred-Component ?x)");
  inferred_component.preconditions = parser::pddl::fromString("(and (Component ?x))");
  derived_predicates.push_back(inferred_component);

  plansys2::Derived inferred_component_1;
  inferred_component_1.predicate = parser::pddl::fromStringPredicate("(inferred-Component ?x)");
  inferred_component_1.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-C_status ?x ?y)))");
  derived_predicates.push_back(inferred_component_1);

  plansys2::Derived inferred_component_2;
  inferred_component_2.predicate = parser::pddl::fromStringPredicate("(inferred-Component ?y)");
  inferred_component_2.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-RequiresC ?x ?y)))");
  derived_predicates.push_back(inferred_component_2);

  plansys2::Derived inferred_different_from;
  inferred_different_from.predicate =
    parser::pddl::fromStringPredicate("(inferred-DifferentFrom ?x ?y)");
  inferred_different_from.preconditions = parser::pddl::fromString("(and (differentFrom ?x ?y))");
  derived_predicates.push_back(inferred_different_from);

  plansys2::Derived inferred_f_active;
  inferred_f_active.predicate =
    parser::pddl::fromStringPredicate("(inferred-F_active ?f ?true_boolean)");
  inferred_f_active.preconditions = parser::pddl::fromString(
    "(and (= ?true_boolean true_boolean) (exists (?fd) (and (inferred-Function ?f) "
    "(inferred-FunctionDesign ?fd) (not (= ?fd fd_unground)) (inferred-SolvesF ?fd ?f) "
    "(inferred-FunctionGrounding ?f ?fd))))");
  inferred_f_active.preconditions.nodes[3].node_type = plansys2_msgs::msg::Node::CONSTANT;
  inferred_f_active.preconditions.nodes[3].name = "true_boolean";
  inferred_f_active.preconditions.nodes[11].node_type = plansys2_msgs::msg::Node::CONSTANT;
  inferred_f_active.preconditions.nodes[11].name = "fd_unground";
  derived_predicates.push_back(inferred_f_active);

  plansys2::Derived inferred_f_active_1;
  inferred_f_active_1.predicate = parser::pddl::fromStringPredicate("(inferred-F_active ?x ?y)");
  inferred_f_active_1.preconditions = parser::pddl::fromString("(and (f_active ?x ?y))");
  derived_predicates.push_back(inferred_f_active_1);

  plansys2::Derived inferred_fd_better_utility_exists;
  inferred_fd_better_utility_exists.predicate =
    parser::pddl::fromStringPredicate("(inferred-FdBetterUtility ?fd_better ?fd)");
  inferred_fd_better_utility_exists.preconditions = parser::pddl::fromString(
    "(exists (?x_better ?f ?x) (and"
    " (inferred-Fd_utility ?fd_better ?x_better)"
    " (inferred-Function ?f)"
    " (inferred-FunctionDesign ?fd)"
    " (inferred-FunctionDesign ?fd_better)"
    " (lessThan ?x ?x_better)"
    " (inferred-SolvesF ?fd ?f)"
    " (inferred-Fd_utility ?fd ?x)"
    " (inferred-SolvesF ?fd_better ?f)"
    "))");
  derived_predicates.push_back(inferred_fd_better_utility_exists);

  plansys2::Derived inferred_fd_better_utility_simple;
  inferred_fd_better_utility_simple.predicate =
    parser::pddl::fromStringPredicate("(inferred-FdBetterUtility ?x ?y)");
  inferred_fd_better_utility_simple.preconditions =
    parser::pddl::fromString("(and (fdBetterUtility ?x ?y))");
  derived_predicates.push_back(inferred_fd_better_utility_simple);

  plansys2::Derived inferred_fd_realisability_1;
  inferred_fd_realisability_1.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_realisability ?fd ?false_boolean)");
  inferred_fd_realisability_1.preconditions = parser::pddl::fromString(
    "(and (= ?false_boolean false_boolean) (exists (?c) (and (inferred-RequiresC ?fd ?c) "
    "(inferred-Component ?c) (inferred-C_status ?c ERROR_string))))");
  inferred_fd_realisability_1.preconditions.nodes[3].node_type = plansys2_msgs::msg::Node::CONSTANT;
  inferred_fd_realisability_1.preconditions.nodes[3].name = "false_boolean";
  derived_predicates.push_back(inferred_fd_realisability_1);

  plansys2::Derived inferred_fd_realisability_2;
  inferred_fd_realisability_2.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_realisability ?fd1 ?false_boolean)");
  inferred_fd_realisability_2.preconditions = parser::pddl::fromString(
    "(and (= ?false_boolean false_boolean) (exists (?eqa ?eqav ?mqa ?mqav) (and "
    "(inferred-FunctionDesign ?fd1) (inferred-HasQAestimation ?fd1 ?eqa) (inferred-IsQAtype ?eqa "
    "water_visibility) (inferred-Qa_has_value ?eqa ?eqav) (inferred-QAvalue ?mqa) (= ?mqa "
    "obs_water_visibility) (inferred-Qa_has_value ?mqa ?mqav) (inferred-IsQAtype ?mqa "
    "water_visibility) (lessThan ?mqav ?eqav))))");
  inferred_fd_realisability_2.preconditions.nodes[3].node_type = plansys2_msgs::msg::Node::CONSTANT;
  inferred_fd_realisability_2.preconditions.nodes[3].name = "false_boolean";
  inferred_fd_realisability_2.preconditions.nodes[13].node_type =
    plansys2_msgs::msg::Node::CONSTANT;
  inferred_fd_realisability_2.preconditions.nodes[13].name = "obs_water_visibility";
  derived_predicates.push_back(inferred_fd_realisability_2);

  plansys2::Derived inferred_fd_realisability_direct;
  inferred_fd_realisability_direct.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_realisability ?x ?y)");
  inferred_fd_realisability_direct.preconditions =
    parser::pddl::fromString("(and (fd_realisability ?x ?y))");
  derived_predicates.push_back(inferred_fd_realisability_direct);

  plansys2::Derived inferred_fd_utility_follow;
  inferred_fd_utility_follow.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_utility ?fd ?qav)");
  inferred_fd_utility_follow.preconditions = parser::pddl::fromString(
    "(exists (?f ?qa) (and (inferred-Function f_follow_pipeline) (inferred-FunctionDesign ?fd) "
    "(inferred-SolvesF ?fd ?f) (inferred-HasQAestimation ?fd ?qa) (inferred-IsQAtype ?qa "
    "performance) (inferred-Qa_has_value ?qa ?qav)))");
  derived_predicates.push_back(inferred_fd_utility_follow);

  plansys2::Derived inferred_fd_utility_generate;
  inferred_fd_utility_generate.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_utility ?fd ?qav)");
  inferred_fd_utility_generate.preconditions = parser::pddl::fromString(
    "(exists (?f ?qa) (and (inferred-Function f_generate_search_path) (inferred-FunctionDesign "
    "?fd) (inferred-SolvesF ?fd ?f) (inferred-HasQAestimation ?fd ?qa) (inferred-IsQAtype ?qa "
    "performance) (inferred-Qa_has_value ?qa ?qav)))");
  derived_predicates.push_back(inferred_fd_utility_generate);

  plansys2::Derived inferred_fd_utility_motion;
  inferred_fd_utility_motion.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_utility ?fd ?qav)");
  inferred_fd_utility_motion.preconditions = parser::pddl::fromString(
    "(exists (?f ?qa) (and (inferred-Function f_maintain_motion) (inferred-FunctionDesign ?fd) "
    "(inferred-SolvesF ?fd ?f) (inferred-HasQAestimation ?fd ?qa) (inferred-IsQAtype ?qa "
    "performance) (inferred-Qa_has_value ?qa ?qav)))");
  derived_predicates.push_back(inferred_fd_utility_motion);

  plansys2::Derived inferred_fd_utility_direct;
  inferred_fd_utility_direct.predicate =
    parser::pddl::fromStringPredicate("(inferred-Fd_utility ?x ?y)");
  inferred_fd_utility_direct.preconditions = parser::pddl::fromString("(and (fd_utility ?x ?y))");
  derived_predicates.push_back(inferred_fd_utility_direct);

  plansys2::Derived inferred_function_direct;
  inferred_function_direct.predicate = parser::pddl::fromStringPredicate("(inferred-Function ?x)");
  inferred_function_direct.preconditions = parser::pddl::fromString("(and (Function ?x))");
  derived_predicates.push_back(inferred_function_direct);

  plansys2::Derived inferred_function_f_active;
  inferred_function_f_active.predicate =
    parser::pddl::fromStringPredicate("(inferred-Function ?x)");
  inferred_function_f_active.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-F_active ?x ?y)))");
  derived_predicates.push_back(inferred_function_f_active);

  plansys2::Derived inferred_function_grounding;
  inferred_function_grounding.predicate =
    parser::pddl::fromStringPredicate("(inferred-Function ?x)");
  inferred_function_grounding.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-FunctionGrounding ?x ?y)))");
  derived_predicates.push_back(inferred_function_grounding);

  plansys2::Derived inferred_function_requiresf;
  inferred_function_requiresf.predicate =
    parser::pddl::fromStringPredicate("(inferred-Function ?y)");
  inferred_function_requiresf.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-RequiresF ?x ?y)))");
  derived_predicates.push_back(inferred_function_requiresf);

  plansys2::Derived inferred_function_solvesf;
  inferred_function_solvesf.predicate = parser::pddl::fromStringPredicate("(inferred-Function ?y)");
  inferred_function_solvesf.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-SolvesF ?x ?y)))");
  derived_predicates.push_back(inferred_function_solvesf);

  plansys2::Derived inferred_function_design_direct;
  inferred_function_design_direct.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_direct.preconditions =
    parser::pddl::fromString("(and (FunctionDesign ?x))");
  derived_predicates.push_back(inferred_function_design_direct);

  plansys2::Derived inferred_function_design_fd_better;
  inferred_function_design_fd_better.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_fd_better.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-FdBetterUtility ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_fd_better);

  plansys2::Derived inferred_function_design_fd_realisability;
  inferred_function_design_fd_realisability.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_fd_realisability.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-Fd_realisability ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_fd_realisability);

  plansys2::Derived inferred_function_design_fd_utility;
  inferred_function_design_fd_utility.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_fd_utility.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-Fd_utility ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_fd_utility);

  plansys2::Derived inferred_function_design_qaestimation;
  inferred_function_design_qaestimation.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_qaestimation.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-HasQAestimation ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_qaestimation);

  plansys2::Derived inferred_function_design_requiresc;
  inferred_function_design_requiresc.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_requiresc.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-RequiresC ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_requiresc);

  plansys2::Derived inferred_function_design_solvesf;
  inferred_function_design_solvesf.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_solvesf.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-SolvesF ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_solvesf);

  plansys2::Derived inferred_function_design_fd_better_inverse;
  inferred_function_design_fd_better_inverse.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?y)");
  inferred_function_design_fd_better_inverse.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-FdBetterUtility ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_fd_better_inverse);

  plansys2::Derived inferred_function_design_grounding_inverse;
  inferred_function_design_grounding_inverse.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?y)");
  inferred_function_design_grounding_inverse.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-FunctionGrounding ?x ?y)))");
  derived_predicates.push_back(inferred_function_design_grounding_inverse);

  plansys2::Derived inferred_function_grounding_1;
  inferred_function_grounding_1.predicate =
    parser::pddl::fromStringPredicate("(inferred-FunctionGrounding ?x ?y)");
  inferred_function_grounding_1.preconditions =
    parser::pddl::fromString("(and (functionGrounding ?x ?y))");
  derived_predicates.push_back(inferred_function_grounding_1);

  plansys2::Derived inferred_has_qaestimation;
  inferred_has_qaestimation.predicate =
    parser::pddl::fromStringPredicate("(inferred-HasQAestimation ?x ?y)");
  inferred_has_qaestimation.preconditions =
    parser::pddl::fromString("(and (hasQAestimation ?x ?y))");
  derived_predicates.push_back(inferred_has_qaestimation);

  plansys2::Derived inferred_inconsistent_c_status;
  inferred_inconsistent_c_status.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_c_status.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-C_status ?x ?y) (inferred-C_status ?x ?z) (not (= ?y "
    "?z))))");
  derived_predicates.push_back(inferred_inconsistent_c_status);

  plansys2::Derived inferred_inconsistent_f_active;
  inferred_inconsistent_f_active.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_f_active.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-F_active ?x ?y) (inferred-F_active ?x ?z) (not (= ?y "
    "?z))))");
  derived_predicates.push_back(inferred_inconsistent_f_active);

  plansys2::Derived inferred_inconsistent_fd_realisability;
  inferred_inconsistent_fd_realisability.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_fd_realisability.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-Fd_realisability ?x ?y) (inferred-Fd_realisability ?x ?z) "
    "(not (= ?y ?z))))");
  derived_predicates.push_back(inferred_inconsistent_fd_realisability);

  plansys2::Derived inferred_inconsistent_fd_utility;
  inferred_inconsistent_fd_utility.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_fd_utility.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-Fd_utility ?x ?y) (inferred-Fd_utility ?x ?z) (not (= ?y "
    "?z))))");
  derived_predicates.push_back(inferred_inconsistent_fd_utility);

  plansys2::Derived inferred_inconsistent_isqatype;
  inferred_inconsistent_isqatype.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_isqatype.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-IsQAtype ?x ?y) (inferred-IsQAtype ?x ?z) (= ?y ?z)))");
  derived_predicates.push_back(inferred_inconsistent_isqatype);

  plansys2::Derived inferred_inconsistent_qavalue;
  inferred_inconsistent_qavalue.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_qavalue.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-Qa_has_value ?x ?y) (inferred-Qa_has_value ?x ?z) (not (= "
    "?y ?z))))");
  derived_predicates.push_back(inferred_inconsistent_qavalue);

  plansys2::Derived inferred_inconsistent_solvesf;
  inferred_inconsistent_solvesf.predicate =
    parser::pddl::fromStringPredicate("(inferred-Inconsistent)");
  inferred_inconsistent_solvesf.preconditions = parser::pddl::fromString(
    "(exists (?x ?y ?z) (and (inferred-SolvesF ?x ?y) (inferred-SolvesF ?x ?z) (= ?y ?z)))");
  derived_predicates.push_back(inferred_inconsistent_solvesf);

  plansys2::Derived inferred_isqatype;
  inferred_isqatype.predicate = parser::pddl::fromStringPredicate("(inferred-IsQAtype ?x ?y)");
  inferred_isqatype.preconditions = parser::pddl::fromString("(and (isQAtype ?x ?y))");
  derived_predicates.push_back(inferred_isqatype);

  plansys2::Derived inferred_qavalue_direct;
  inferred_qavalue_direct.predicate = parser::pddl::fromStringPredicate("(inferred-QAvalue ?x)");
  inferred_qavalue_direct.preconditions = parser::pddl::fromString("(and (QAvalue ?x))");
  derived_predicates.push_back(inferred_qavalue_direct);

  plansys2::Derived inferred_qavalue_from_type;
  inferred_qavalue_from_type.predicate = parser::pddl::fromStringPredicate("(inferred-QAvalue ?x)");
  inferred_qavalue_from_type.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-IsQAtype ?x ?y)))");
  derived_predicates.push_back(inferred_qavalue_from_type);

  plansys2::Derived inferred_qavalue_from_value;
  inferred_qavalue_from_value.predicate =
    parser::pddl::fromStringPredicate("(inferred-QAvalue ?x)");
  inferred_qavalue_from_value.preconditions =
    parser::pddl::fromString("(exists (?y) (and (inferred-Qa_has_value ?x ?y)))");
  derived_predicates.push_back(inferred_qavalue_from_value);

  plansys2::Derived inferred_qavalue_from_estimation;
  inferred_qavalue_from_estimation.predicate =
    parser::pddl::fromStringPredicate("(inferred-QAvalue ?y)");
  inferred_qavalue_from_estimation.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-HasQAestimation ?x ?y)))");
  derived_predicates.push_back(inferred_qavalue_from_estimation);

  plansys2::Derived inferred_qa_has_value;
  inferred_qa_has_value.predicate =
    parser::pddl::fromStringPredicate("(inferred-Qa_has_value ?x ?y)");
  inferred_qa_has_value.preconditions = parser::pddl::fromString("(and (qa_has_value ?x ?y))");
  derived_predicates.push_back(inferred_qa_has_value);

  plansys2::Derived inferred_quality_attribute_type_direct;
  inferred_quality_attribute_type_direct.predicate =
    parser::pddl::fromStringPredicate("(inferred-QualityAttributeType ?x)");
  inferred_quality_attribute_type_direct.preconditions =
    parser::pddl::fromString("(and (QualityAttributeType ?x))");
  derived_predicates.push_back(inferred_quality_attribute_type_direct);

  plansys2::Derived inferred_quality_attribute_type_from_type;
  inferred_quality_attribute_type_from_type.predicate =
    parser::pddl::fromStringPredicate("(inferred-QualityAttributeType ?y)");
  inferred_quality_attribute_type_from_type.preconditions =
    parser::pddl::fromString("(exists (?x) (and (inferred-IsQAtype ?x ?y)))");
  derived_predicates.push_back(inferred_quality_attribute_type_from_type);

  plansys2::Derived inferred_requires_c;
  inferred_requires_c.predicate = parser::pddl::fromStringPredicate("(inferred-RequiresC ?x ?y)");
  inferred_requires_c.preconditions = parser::pddl::fromString("(and (requiresC ?x ?y))");
  derived_predicates.push_back(inferred_requires_c);

  plansys2::Derived inferred_requires_f;
  inferred_requires_f.predicate = parser::pddl::fromStringPredicate("(inferred-RequiresF ?x ?y)");
  inferred_requires_f.preconditions = parser::pddl::fromString("(and (requiresF ?x ?y))");
  derived_predicates.push_back(inferred_requires_f);

  plansys2::Derived inferred_same_as;
  inferred_same_as.predicate = parser::pddl::fromStringPredicate("(inferred-SameAs ?x ?y)");
  inferred_same_as.preconditions = parser::pddl::fromString("(and (sameAs ?x ?y))");
  derived_predicates.push_back(inferred_same_as);

  plansys2::Derived inferred_solvesf_fd_unground;
  inferred_solvesf_fd_unground.predicate =
    parser::pddl::fromStringPredicate("(inferred-SolvesF ?fd_unground ?f)");
  inferred_solvesf_fd_unground.preconditions = parser::pddl::fromString(
    "(and (= ?fd_unground fd_unground) (and (inferred-Function ?f) (inferred-FunctionDesign "
    "fd_unground)))");
  inferred_solvesf_fd_unground.preconditions.nodes[3].node_type =
    plansys2_msgs::msg::Node::CONSTANT;
  inferred_solvesf_fd_unground.preconditions.nodes[3].name = "fd_unground";
  derived_predicates.push_back(inferred_solvesf_fd_unground);

  plansys2::Derived inferred_solvesf_direct;
  inferred_solvesf_direct.predicate = parser::pddl::fromStringPredicate("(inferred-SolvesF ?x ?y)");
  inferred_solvesf_direct.preconditions = parser::pddl::fromString("(and (solvesF ?x ?y))");
  derived_predicates.push_back(inferred_solvesf_direct);

  plansys2::DerivedResolutionGraph graph(derived_predicates);

  ASSERT_EQ(graph.getNodeNumber(), 75);
  ASSERT_EQ(graph.getRootNumber(), 21);
  ASSERT_EQ(graph.getPredicates().size(), 21);
  ASSERT_EQ(graph.getPredicatesNames().size(), 21);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 54);

  auto sccs = graph.computeSCCsTarjanDerivedPredicates();
  int total_size = 0;
  for (const auto & scc : sccs) {
    total_size += scc.size();
    for (const auto & node : scc) {
    }
  }
  ASSERT_EQ(total_size, 54);

  plansys2::Action start_robot;
  start_robot.name = "start_robot";
  start_robot.parameters.push_back(parser::pddl::fromStringParam("?r", "robot"));
  start_robot.preconditions = parser::pddl::fromString("(and (robot_not_started ?r))");
  start_robot.effects =
    parser::pddl::fromString("(and (not (robot_not_started ?r)) (robot_started ?r))");
  graph.appendAction(start_robot);

  ASSERT_EQ(graph.getNodeNumber(), 77);
  ASSERT_EQ(graph.getRootNumber(), 22);
  ASSERT_EQ(graph.getPredicates().size(), 22);
  ASSERT_EQ(graph.getPredicatesNames().size(), 22);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 54);
  ASSERT_EQ(graph.getActionsNumber(), 1);

  std::vector<std::string> parent_nodes_names = graph.getParentNodesNames(start_robot);
  ASSERT_EQ(parent_nodes_names.size(), 1);
  ASSERT_EQ(parent_nodes_names[0], "robot_not_started");

  plansys2::Action reconfigure1;
  reconfigure1.name = "reconfigure1";
  reconfigure1.parameters.push_back(parser::pddl::fromStringParam("?f"));
  reconfigure1.parameters.push_back(parser::pddl::fromStringParam("?fd_goal"));
  reconfigure1.preconditions = parser::pddl::fromString(
    "(and"
    " (Function ?f)"
    " (inferred-SolvesF ?fd_goal ?f)"
    " (FunctionDesign ?fd_goal)"
    " (not (inferred-Fd_realisability ?fd_goal false_boolean))"
    " (not (exists (?fd) (and"
    "   (inferred-SolvesF ?fd ?f)"
    "   (FunctionDesign ?fd)"
    "   (functionGrounding ?f ?fd)"
    " )))"
    " (or"
    "   (= ?fd_goal fd_unground)"
    "   (not (exists (?fd) (and"
    "     (inferred-SolvesF ?fd ?f)"
    "     (not (inferred-Fd_realisability ?fd false_boolean))"
    "     (inferred-FdBetterUtility ?fd ?fd_goal)"
    "   )))"
    " )"
    ")");
  reconfigure1.preconditions.nodes[15].node_type = plansys2_msgs::msg::Node::CONSTANT;
  reconfigure1.preconditions.nodes[15].name = "fd_unground";
  reconfigure1.effects = parser::pddl::fromString("(and (functionGrounding ?f ?fd_goal))");
  graph.appendAction(reconfigure1);

  ASSERT_EQ(graph.getNodeNumber(), 78);
  ASSERT_EQ(graph.getRootNumber(), 22);
  ASSERT_EQ(graph.getPredicates().size(), 22);
  ASSERT_EQ(graph.getPredicatesNames().size(), 22);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 54);
  ASSERT_EQ(graph.getActionsNumber(), 2);

  plansys2::Action reconfigure2;
  reconfigure2.name = "reconfigure2";
  reconfigure2.parameters.push_back(parser::pddl::fromStringParam("?f"));
  reconfigure2.parameters.push_back(parser::pddl::fromStringParam("?fd_initial"));
  reconfigure2.parameters.push_back(parser::pddl::fromStringParam("?fd_goal"));
  reconfigure2.preconditions = parser::pddl::fromString(
    "(and"
    " (not (= ?fd_initial ?fd_goal))"
    " (Function ?f)"
    " (FunctionDesign ?fd_initial)"
    " (functionGrounding ?f ?fd_initial)"
    " (inferred-SolvesF ?fd_goal ?f)"
    " (FunctionDesign ?fd_goal)"
    " (not (inferred-Fd_realisability ?fd_goal false_boolean))"
    " (or"
    "   (= ?fd_goal fd_unground)"
    "   (not (exists (?fd) (and"
    "     (inferred-SolvesF ?fd ?f)"
    "     (not (inferred-Fd_realisability ?fd false_boolean))"
    "     (inferred-FdBetterUtility ?fd ?fd_goal)"
    "   )))"
    " )"
    ")");
  reconfigure2.effects = parser::pddl::fromString(
    "(and"
    " (not (functionGrounding ?f ?fd_initial))"
    " (functionGrounding ?f ?fd_goal)"
    ")");
  reconfigure2.preconditions.nodes[15].node_type = plansys2_msgs::msg::Node::CONSTANT;
  reconfigure2.preconditions.nodes[15].name = "fd_unground";
  graph.appendAction(reconfigure2);

  ASSERT_EQ(graph.getPredicates().size(), 22);
  ASSERT_EQ(graph.getPredicatesNames().size(), 22);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 54);
  ASSERT_EQ(graph.getActionsNumber(), 3);
  ASSERT_EQ(graph.getRootNumber(), 22);
  ASSERT_EQ(graph.getNodeNumber(), 79);

  plansys2::Action search_pipeline;
  search_pipeline.name = "search_pipeline";
  search_pipeline.parameters.push_back(parser::pddl::fromStringParam("?p", "pipeline"));
  search_pipeline.parameters.push_back(parser::pddl::fromStringParam("?r", "robot"));
  search_pipeline.preconditions = parser::pddl::fromString(
    "(and"
    " (robot_started ?r)"
    " (exists (?a ?f1 ?f2 ?fd1 ?fd2) (and"
    "   (inferred-Action ?a)"
    "   (= ?a a_search_pipeline)"
    "   (not (= ?f1 ?f2))"
    "   (inferred-RequiresF ?a ?f1)"
    "   (inferred-RequiresF ?a ?f2)"
    "   (inferred-F_active ?f1 true_boolean)"
    "   (inferred-F_active ?f2 true_boolean)"
    " ))"
    " (not (inferred-F_active f_follow_pipeline true_boolean))"
    ")");
  search_pipeline.preconditions.nodes[7].node_type = plansys2_msgs::msg::Node::CONSTANT;
  search_pipeline.preconditions.nodes[7].name = "a_search_pipeline";
  search_pipeline.effects = parser::pddl::fromString("(and (pipeline_found ?p))");
  graph.appendAction(search_pipeline);

  ASSERT_EQ(graph.getFunctions().size(), 0);
  ASSERT_EQ(graph.getPredicates().size(), 23);
  ASSERT_EQ(graph.getPredicatesNames().size(), 23);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 54);
  ASSERT_EQ(graph.getActionsNumber(), 4);
  ASSERT_EQ(graph.getRootNumber(), 23);
  ASSERT_EQ(graph.getNodeNumber(), 81);

  plansys2::Action inspect_pipeline;
  inspect_pipeline.name = "inspect_pipeline";
  inspect_pipeline.parameters.push_back(parser::pddl::fromStringParam("?p", "pipeline"));
  inspect_pipeline.parameters.push_back(parser::pddl::fromStringParam("?r", "robot"));
  inspect_pipeline.preconditions = parser::pddl::fromString(
    "(and"
    " (robot_started ?r)"
    " (pipeline_found ?p)"
    " (exists (?a ?f1 ?f2 ?fd1 ?fd2) (and"
    "   (inferred-Action ?a)"
    "   (= ?a a_inspect_pipeline)"
    "   (not (= ?f1 ?f2))"
    "   (inferred-RequiresF ?a ?f1)"
    "   (inferred-RequiresF ?a ?f2)"
    "   (inferred-F_active ?f1 true_boolean)"
    "   (inferred-F_active ?f2 true_boolean)"
    " ))"
    " (not (inferred-F_active f_generate_search_path true_boolean))"
    ")");
  inspect_pipeline.preconditions.nodes[8].node_type = plansys2_msgs::msg::Node::CONSTANT;
  inspect_pipeline.preconditions.nodes[8].name = "a_inspect_pipeline";
  inspect_pipeline.effects = parser::pddl::fromString("(and (pipeline_inspected ?p))");
  graph.appendAction(inspect_pipeline);

  ASSERT_EQ(graph.getFunctions().size(), 0);
  ASSERT_EQ(graph.getPredicates().size(), 24);
  ASSERT_EQ(graph.getRootNumber(), 24);
  ASSERT_EQ(graph.getPredicatesNames().size(), 24);
  ASSERT_EQ(graph.getDerivedPredicates().size(), 54);
  ASSERT_EQ(graph.getActionsNumber(), 5);
  ASSERT_EQ(graph.getNodeNumber(), 83);

  auto graph_start_robot_action = graph.pruneGraphToActions({start_robot});

  ASSERT_EQ(graph_start_robot_action.getFunctions().size(), 0);
  ASSERT_EQ(graph_start_robot_action.getPredicates().size(), 1);
  ASSERT_EQ(graph_start_robot_action.getRootNumber(), 1);
  ASSERT_EQ(graph_start_robot_action.getPredicatesNames().size(), 1);
  ASSERT_EQ(graph_start_robot_action.getDerivedPredicates().size(), 0);
  ASSERT_EQ(graph_start_robot_action.getActionsNumber(), 1);
  ASSERT_EQ(graph_start_robot_action.getNodeNumber(), 2);

  std::vector<std::string> start_robot_parent_nodes;
  auto func_start_robot_parents = [&start_robot_parent_nodes](const plansys2::NodeVariant & node) {
      start_robot_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(start_robot, func_start_robot_parents);
  ASSERT_EQ(start_robot_parent_nodes.size(), 2);
  ASSERT_EQ(start_robot_parent_nodes[0], "start_robot");
  ASSERT_EQ(start_robot_parent_nodes[1], "robot_not_started");

  auto graph_reconfigure1_action = graph.pruneGraphToActions({reconfigure1});

  ASSERT_EQ(graph_reconfigure1_action.getFunctions().size(), 0);
  ASSERT_EQ(graph_reconfigure1_action.getPredicates().size(), 17);
  ASSERT_EQ(graph_reconfigure1_action.getRootNumber(), 17);
  ASSERT_EQ(graph_reconfigure1_action.getDerivedPredicates().size(), 41);
  ASSERT_EQ(graph_reconfigure1_action.getActionsNumber(), 1);
  ASSERT_EQ(graph_reconfigure1_action.getNodeNumber(), 59);

  std::vector<std::string> reconfigure1_parent_nodes;
  auto func_reconfigure1_parents =
    [&reconfigure1_parent_nodes](const plansys2::NodeVariant & node) {
      reconfigure1_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(reconfigure1, func_reconfigure1_parents);
  ASSERT_EQ(reconfigure1_parent_nodes.size(), 59);

  auto graph_reconfigure2_action = graph.pruneGraphToActions({reconfigure2});
  ASSERT_EQ(graph_reconfigure2_action.getFunctions().size(), 0);
  ASSERT_EQ(graph_reconfigure2_action.getPredicates().size(), 17);
  ASSERT_EQ(graph_reconfigure2_action.getRootNumber(), 17);
  ASSERT_EQ(graph_reconfigure2_action.getDerivedPredicates().size(), 41);
  ASSERT_EQ(graph_reconfigure2_action.getActionsNumber(), 1);
  ASSERT_EQ(graph_reconfigure2_action.getNodeNumber(), 59);

  std::vector<std::string> reconfigure2_parent_nodes;
  auto func_reconfigure2_parents =
    [&reconfigure2_parent_nodes](const plansys2::NodeVariant & node) {
      reconfigure2_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(reconfigure2, func_reconfigure2_parents);
  ASSERT_EQ(reconfigure2_parent_nodes.size(), 59);

  auto graph_search_pipeline_action = graph.pruneGraphToActions({search_pipeline});
  ASSERT_EQ(graph_search_pipeline_action.getFunctions().size(), 0);
  ASSERT_EQ(graph_search_pipeline_action.getPredicates().size(), 19);
  ASSERT_EQ(graph_search_pipeline_action.getRootNumber(), 19);
  ASSERT_EQ(graph_search_pipeline_action.getDerivedPredicates().size(), 43);
  ASSERT_EQ(graph_search_pipeline_action.getActionsNumber(), 1);
  ASSERT_EQ(graph_search_pipeline_action.getNodeNumber(), 63);

  std::vector<std::string> search_pipeline_parent_nodes;
  auto func_search_pipeline_parents =
    [&search_pipeline_parent_nodes](const plansys2::NodeVariant & node) {
      search_pipeline_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(search_pipeline, func_search_pipeline_parents);
  ASSERT_EQ(search_pipeline_parent_nodes.size(), 63);

  auto graph_inspect_pipeline_action = graph.pruneGraphToActions({inspect_pipeline});
  ASSERT_EQ(graph_inspect_pipeline_action.getFunctions().size(), 0);
  ASSERT_EQ(graph_inspect_pipeline_action.getPredicates().size(), 20);
  ASSERT_EQ(graph_inspect_pipeline_action.getRootNumber(), 20);
  ASSERT_EQ(graph_inspect_pipeline_action.getDerivedPredicates().size(), 43);
  ASSERT_EQ(graph_inspect_pipeline_action.getActionsNumber(), 1);
  ASSERT_EQ(graph_inspect_pipeline_action.getNodeNumber(), 64);

  std::vector<std::string> inspect_pipeline_parent_nodes;
  auto func_inspect_pipeline_parents =
    [&inspect_pipeline_parent_nodes](const plansys2::NodeVariant & node) {
      inspect_pipeline_parent_nodes.push_back(node.getNodeName());
    };
  graph.backtrackTraverse(inspect_pipeline, func_inspect_pipeline_parents);
  ASSERT_EQ(inspect_pipeline_parent_nodes.size(), 64);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
