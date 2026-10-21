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

// plansys2::State and its derived predicates graph: the facts it keeps stay consistent

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/DerivedResolutionGraph.hpp"
#include "plansys2_core/State.hpp"

namespace
{
plansys2::Derived derived(const std::string & predicate, const std::string & preconditions)
{
  plansys2_msgs::msg::Derived msg;
  msg.predicate = parser::pddl::fromStringPredicate(predicate);
  msg.preconditions = parser::pddl::fromString(preconditions);
  return plansys2::Derived(msg);
}

plansys2::Predicate fact(const std::string & text)
{
  return parser::pddl::fromStringPredicate(text);
}

std::vector<std::string> names(const std::vector<plansys2::Derived> & derived)
{
  std::vector<std::string> out;
  for (const auto & d : derived) {
    out.push_back(d.predicate.name);
  }
  std::sort(out.begin(), out.end());
  return out;
}
}  // namespace

TEST(state_consistency, asserted_and_inferred_fact)
{
  plansys2::State state;
  const auto d = derived("(at_home ?r)", "(and (at ?r home))");
  const auto p = fact("(at_home r1)");

  // Inferred, then asserted: in the union until neither
  EXPECT_TRUE(state.addInferredPredicate(d, p));
  EXPECT_TRUE(state.addPredicate(p));
  EXPECT_FALSE(state.addPredicate(p));
  EXPECT_EQ(state.getUnionPredicatesSize(), 1u);
  EXPECT_TRUE(state.removePredicate(p));
  EXPECT_EQ(state.getUnionPredicatesSize(), 1u);  // Still inferred
  EXPECT_TRUE(state.hasInferredPredicate(p));
  EXPECT_FALSE(state.removePredicate(p));
  state.ungroundSingleDerivedPredicate(d);
  EXPECT_EQ(state.getUnionPredicatesSize(), 0u);

  // Asserted, then inferred and no longer inferred: still asserted
  EXPECT_TRUE(state.addPredicate(p));
  EXPECT_TRUE(state.addInferredPredicate(d, p));
  EXPECT_EQ(state.getUnionPredicatesSize(), 1u);
  EXPECT_TRUE(state.ungroundSingleDerivedPredicate(d));
  EXPECT_EQ(state.getUnionPredicatesSize(), 1u);
  EXPECT_TRUE(state.hasPredicate(p));
  EXPECT_FALSE(state.hasInferredPredicate(p));

  // Removal through the iterator
  EXPECT_TRUE(state.addInferredPredicate(d, p));
  state.removePredicate(state.getPredicate(p));
  EXPECT_EQ(state.getUnionPredicatesSize(), 1u);
  EXPECT_EQ(state.getPredicatesSize(), 0u);

  // Two derived predicates infer the same fact: it stays until the last one goes
  const auto d2 = derived("(at_home ?r)", "(and (docked ?r))");
  EXPECT_FALSE(state.addInferredPredicate(d2, p));
  EXPECT_EQ(state.getInferredPredicateRefCount(p), 2u);
  state.ungroundSingleDerivedPredicate(d);
  EXPECT_TRUE(state.hasInferredPredicate(p));
  state.ungroundSingleDerivedPredicate(d2);
  EXPECT_FALSE(state.hasInferredPredicate(p));
  EXPECT_EQ(state.getInferredPredicateRefCount(p), 0u);
  EXPECT_EQ(state.getUnionPredicatesSize(), 0u);
}

TEST(state_consistency, has_derived_predicates)
{
  plansys2::State state;
  EXPECT_FALSE(state.hasDerivedPredicates());

  std::vector<plansys2_msgs::msg::Derived> derived_msgs = {
    derived("(at_home ?r)", "(and (at ?r home))")};
  state.setDerivedPredicates(plansys2::DerivedResolutionGraph(derived_msgs));
  EXPECT_TRUE(state.hasDerivedPredicates());

  state.clearState();
  EXPECT_FALSE(state.hasDerivedPredicates());
}

TEST(state_consistency, clear_leaves_nothing)
{
  std::vector<plansys2_msgs::msg::Derived> derived_msgs = {
    derived("(at_home ?r)", "(and (at ?r home))"),
    derived("(safe ?r)", "(and (at_home ?r))")};
  plansys2::DerivedResolutionGraph graph(derived_msgs);
  EXPECT_GT(graph.getNodeNumber(), 0u);
  graph.clear();
  EXPECT_EQ(graph.getNodeNumber(), 0u);
  EXPECT_EQ(graph.getEdgeNumber(), 0u);
  EXPECT_EQ(graph.getDerivedPredicatesNumber(), 0u);
  EXPECT_EQ(graph.getActionsNumber(), 0u);
  EXPECT_TRUE(graph == plansys2::DerivedResolutionGraph());
  EXPECT_EQ(
    std::hash<plansys2::DerivedResolutionGraph>{}(graph),
    std::hash<plansys2::DerivedResolutionGraph>{}(plansys2::DerivedResolutionGraph()));

  plansys2::State state;
  state.setDerivedPredicates(plansys2::DerivedResolutionGraph(derived_msgs));
  state.addPredicate(fact("(at r1 home)"));
  state.addInferredPredicate(derived_msgs[0], fact("(at_home r1)"));
  state.clearState();
  auto msg = state.getAsMsg();
  EXPECT_TRUE(msg.instances.empty());
  EXPECT_TRUE(msg.predicates.empty());
  EXPECT_TRUE(msg.inferred_predicates.empty());
  EXPECT_TRUE(msg.derived_predicates.empty());
  EXPECT_TRUE(state == plansys2::State());
}

TEST(state_consistency, looking_up_edges_adds_nothing)
{
  std::vector<plansys2_msgs::msg::Derived> derived_msgs = {
    derived("(at_home ?r)", "(and (at ?r home))")};
  plansys2::DerivedResolutionGraph graph(derived_msgs);
  const auto before = graph;
  const auto hash_before = std::hash<plansys2::DerivedResolutionGraph>{}(graph);

  plansys2::NodeVariant unknown(fact("(unrelated x)"));
  EXPECT_TRUE(graph.getNodeOutEdges(unknown).empty());
  EXPECT_TRUE(graph.getNodeInEdges(unknown).empty());
  for (const auto & node : before.getNodes()) {
    graph.getNodeOutEdges(node);
    graph.getNodeInEdges(node);
  }
  EXPECT_TRUE(graph == before);
  EXPECT_EQ(std::hash<plansys2::DerivedResolutionGraph>{}(graph), hash_before);
}

TEST(state_consistency, recursive_derived_predicates_are_not_skipped)
{
  // reachable is recursive: through itself
  std::vector<plansys2_msgs::msg::Derived> derived_msgs = {
    derived("(reachable ?a ?b)", "(and (connected ?a ?b))"),
    derived("(reachable ?a ?b)", "(and (connected ?a ?c)(reachable ?c ?b))"),
    derived("(goal_reachable ?a)", "(and (reachable ?a goal))"),
    derived("(home_ok)", "(and (at home))")};
  plansys2::DerivedResolutionGraph graph(derived_msgs);

  const auto all = names(graph.getDerivedPredicatesDepthFirst());
  EXPECT_EQ(
    all,
    std::vector<std::string>({"goal_reachable", "home_ok", "reachable", "reachable"}));

  // Dependencies first: goal_reachable after both reachable
  auto ordered = graph.getDerivedPredicatesDepthFirst();
  auto pos = [&](const std::string & name, bool last) {
      size_t p = ordered.size();
      for (size_t i = 0; i < ordered.size(); i++) {
        if (ordered[i].predicate.name == name) {
          p = i;
          if (!last) {break;}
        }
      }
      return p;
    };
  EXPECT_GT(pos("goal_reachable", false), pos("reachable", true));

  // Also through State, from a node of the cycle
  plansys2::State state;
  state.setDerivedPredicates(graph);
  EXPECT_EQ(names(state.getDerivedPredicatesDepthFirst()).size(), 4u);
  auto from_goal = state.getDerivedPredicatesDepthFirst({fact("(connected ?a ?b)")});
  EXPECT_GE(from_goal.size(), 3u);

  // A dependency that is not met still stops the traversal
  plansys2::Predicate a = fact("(A)"), b = fact("(B)"), c = fact("(C)"), d = fact("(D)");
  plansys2::DerivedResolutionGraph g;
  g.addEdge(a, c);
  g.addEdge(b, c);
  g.addEdge(c, d);
  g.addEdge(d, c);  // c and d form a cycle that also needs b
  std::vector<std::string> visited;
  g.depthFirstTraverseFromNodes(
    [&](const plansys2::NodeVariant & n) {visited.push_back(n.getNodeName());}, true, {a});
  EXPECT_EQ(visited, std::vector<std::string>({"A"}));
  visited.clear();
  g.depthFirstTraverseFromNodes(
    [&](const plansys2::NodeVariant & n) {visited.push_back(n.getNodeName());}, true, {a, b});
  EXPECT_EQ(visited.size(), 4u);
}

TEST(state_consistency, round_trip_through_the_message)
{
  std::vector<plansys2_msgs::msg::Derived> derived_msgs = {
    derived("(at_home ?r)", "(and (at ?r home))")};
  plansys2::State state;
  state.setDerivedPredicates(plansys2::DerivedResolutionGraph(derived_msgs));
  state.addInstance(parser::pddl::fromStringParam("r1", "robot"));
  state.addPredicate(fact("(at r1 home)"));
  state.addFunction(plansys2::Function("(= (battery r1) 10)"));
  state.addInferredPredicate(derived_msgs[0], fact("(at_home r1)"));

  plansys2::State copy(state.getAsMsg());
  EXPECT_EQ(copy.getInstancesSize(), 1u);
  EXPECT_EQ(copy.getPredicatesSize(), 1u);
  EXPECT_EQ(copy.getFunctionsSize(), 1u);
  EXPECT_EQ(copy.getInferredPredicatesSize(), 1u);
  EXPECT_EQ(copy.getUnionPredicatesSize(), 2u);
  EXPECT_TRUE(copy.hasDerivedPredicates());
  EXPECT_TRUE(copy.hasInferredPredicate(fact("(at_home r1)")));
  // Where an inferred fact came from does not travel: asking does not throw
  EXPECT_NO_THROW(copy.getInferredPredicateRefCount(fact("(at_home r1)")));
  EXPECT_NO_THROW(copy.getInferredPredicateRefCount(fact("(at_home r9)")));
  EXPECT_TRUE(copy.getAsMsg() == plansys2::State(copy.getAsMsg()).getAsMsg());
}

TEST(state_consistency, derived_hash_is_always_set)
{
  plansys2::Derived empty;
  plansys2::Derived empty2;
  EXPECT_EQ(std::hash<plansys2::Derived>{}(empty), std::hash<plansys2::Derived>{}(empty2));
  plansys2::Derived no_preconditions;
  no_preconditions.predicate = fact("(p ?x)");
  plansys2::Derived no_preconditions2;
  no_preconditions2.predicate = fact("(p ?y)");
  EXPECT_EQ(
    std::hash<plansys2::Derived>{}(no_preconditions),
    std::hash<plansys2::Derived>{}(no_preconditions2));
}
