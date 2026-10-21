// Copyright 2024 Intelligent Robotics Lab
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

#include "plansys2_core/State.hpp"

#include "gtest/gtest.h"
#include "plansys2_core/DerivedResolutionGraph.hpp"

TEST(state_test, state)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  std::vector<plansys2_msgs::msg::Derived> derived_predicates;
  plansys2_msgs::msg::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredA);

  plansys2_msgs::msg::Derived inferredB;
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
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?p)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?p)(inferredB ?p))");
  derived_predicates.push_back(inferredAB);

  plansys2::DerivedResolutionGraph graph(derived_predicates);

  plansys2::Instance instanceA = parser::pddl::fromStringParam("(instanceA)");
  plansys2::Instance instanceB = parser::pddl::fromStringParam("(instanceB)");
  plansys2::Instance instanceAB = parser::pddl::fromStringParam("(instanceAB)");
  std::unordered_set<plansys2::Instance> instances;
  instances.insert(instanceA);
  instances.insert(instanceB);
  instances.insert(instanceAB);

  plansys2::Predicate predA_instanceA = parser::pddl::fromStringPredicate("(predicateA instanceA)");
  plansys2::Predicate predA_instanceAB =
    parser::pddl::fromStringPredicate("(predicateA instanceAB)");
  plansys2::Predicate predB_instanceB = parser::pddl::fromStringPredicate("(predicateB instanceB)");
  plansys2::Predicate predB_instanceAB =
    parser::pddl::fromStringPredicate("(predicateB instanceAB)");
  std::unordered_set<plansys2::Predicate> predicates;
  predicates.insert(predA_instanceA);
  predicates.insert(predA_instanceAB);
  predicates.insert(predB_instanceB);
  predicates.insert(predB_instanceAB);

  plansys2::Predicate inferredA_instanceA =
    parser::pddl::fromStringPredicate("(inferredA instanceA)");
  plansys2::Predicate inferredAA_instanceA =
    parser::pddl::fromStringPredicate("(inferredAA instanceA)");
  plansys2::Predicate inferredAA2_instanceA =
    parser::pddl::fromStringPredicate("(inferredAA instanceA)");

  plansys2::Predicate inferredA_instanceAB =
    parser::pddl::fromStringPredicate("(inferredA instanceAB)");
  plansys2::Predicate inferredAA_instanceAB =
    parser::pddl::fromStringPredicate("(inferredAA instanceAB)");
  plansys2::Predicate inferredAA2_instanceAB =
    parser::pddl::fromStringPredicate("(inferredAA instanceAB)");

  plansys2::Predicate inferredB_instanceB =
    parser::pddl::fromStringPredicate("(inferredB instanceB)");
  plansys2::Predicate inferredBB_instanceB =
    parser::pddl::fromStringPredicate("(inferredBB instanceB)");

  plansys2::Predicate inferredB_instanceAB =
    parser::pddl::fromStringPredicate("(inferredB instanceAB)");
  plansys2::Predicate inferredBB_instanceAB =
    parser::pddl::fromStringPredicate("(inferredBB instanceAB)");

  plansys2::Predicate inferredAB_instanceAB =
    parser::pddl::fromStringPredicate("(inferredAB instanceAB)");

  std::unordered_set<plansys2::Function> functions;
  plansys2::State state(instances, functions, predicates);

  ASSERT_EQ(state.getInstancesSize(), 3);
  ASSERT_EQ(state.getPredicatesSize(), 4);
  ASSERT_EQ(state.getFunctionsSize(), 0);
  ASSERT_EQ(state.getInferredPredicatesSize(), 0);
  ASSERT_EQ(state.getUnionPredicatesSize(), 4);

  state.addInferredPredicate(inferredA, inferredA_instanceA);
  state.addInferredPredicate(inferredAA, inferredAA_instanceA);
  state.addInferredPredicate(inferredAA2, inferredAA2_instanceA);

  state.addInferredPredicate(inferredA, inferredA_instanceAB);
  state.addInferredPredicate(inferredAA, inferredAA_instanceAB);
  state.addInferredPredicate(inferredAA2, inferredAA2_instanceAB);

  state.addInferredPredicate(inferredB, inferredB_instanceB);
  state.addInferredPredicate(inferredBB, inferredBB_instanceB);

  state.addInferredPredicate(inferredB, inferredB_instanceAB);
  state.addInferredPredicate(inferredBB, inferredBB_instanceAB);

  state.addInferredPredicate(inferredAB, inferredAB_instanceAB);

  ASSERT_EQ(state.getPredicatesSize(), 4);
  ASSERT_EQ(state.getInferredPredicatesSize(), 9);
  ASSERT_EQ(state.getUnionPredicatesSize(), 13);
  ASSERT_EQ(state.getInferredPredicateRefCount(inferredAA_instanceA), 2);
  ASSERT_EQ(state.getInferredPredicateRefCount(inferredAA2_instanceAB), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredA), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredB), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredAA), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredBB), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredAB), 1);

  std::vector<std::tuple<plansys2::Derived, plansys2::Predicate>> inferred_predicates;
  inferred_predicates.push_back({inferredA, inferredA_instanceA});
  inferred_predicates.push_back({inferredAA, inferredAA_instanceA});
  inferred_predicates.push_back({inferredAA2, inferredAA2_instanceA});

  inferred_predicates.push_back({inferredA, inferredA_instanceAB});
  inferred_predicates.push_back({inferredAA, inferredAA_instanceAB});
  inferred_predicates.push_back({inferredAA2, inferredAA2_instanceAB});

  inferred_predicates.push_back({inferredB, inferredB_instanceB});
  inferred_predicates.push_back({inferredBB, inferredBB_instanceB});

  inferred_predicates.push_back({inferredB, inferredB_instanceAB});
  inferred_predicates.push_back({inferredBB, inferredBB_instanceAB});

  inferred_predicates.push_back({inferredAB, inferredAB_instanceAB});

  plansys2::State state_2(instances, functions, predicates, inferred_predicates, graph);

  ASSERT_EQ(state_2.getPredicatesSize(), 4);
  ASSERT_EQ(state_2.getInferredPredicatesSize(), 9);
  ASSERT_EQ(state_2.getUnionPredicatesSize(), 13);
  ASSERT_EQ(state_2.getInferredPredicateRefCount(inferredAA_instanceA), 2);
  ASSERT_EQ(state_2.getInferredPredicateRefCount(inferredAA2_instanceAB), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredA), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredB), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAA), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredBB), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAB), 1);

  state_2.ungroundDerivedPredicate(inferredA);
  ASSERT_EQ(state_2.getPredicatesSize(), 4);
  ASSERT_EQ(state_2.getInferredPredicatesSize(), 6);
  ASSERT_EQ(state_2.getUnionPredicatesSize(), 10);
  ASSERT_EQ(state_2.getInferredPredicateRefCount(inferredAA_instanceA), 1);
  ASSERT_EQ(state_2.getInferredPredicateRefCount(inferredAA2_instanceAB), 1);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAB), 0);

  state_2.ungroundDerivedPredicate(inferredA);
  ASSERT_EQ(state_2.getPredicatesSize(), 4);
  ASSERT_EQ(state_2.getInferredPredicatesSize(), 6);
  ASSERT_EQ(state_2.getUnionPredicatesSize(), 10);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAB), 0);

  plansys2::State state_3(instances, functions, predicates, inferred_predicates, graph);
  state_3.ungroundDerivedPredicate(inferredAB);
  ASSERT_EQ(state_3.getPredicatesSize(), 4);
  ASSERT_EQ(state_3.getInferredPredicatesSize(), 8);
  ASSERT_EQ(state_3.getUnionPredicatesSize(), 12);
  ASSERT_EQ(state_3.getNumberInferredFromDerived(inferredAB), 0);
}

TEST(state_test, get_derived_predicates_sccs)
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
  plansys2::State state;
  state.setDerivedPredicates(graph);

  auto sccs_full = state.getDerivedPredicatesSCCs();

  ASSERT_EQ(sccs_full.size(), 6);

  // Helper lambda to check if a set of derived predicates (regardless of order) is in sccs_full
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

  ASSERT_TRUE(contains_scc(sccs_full, {der2, der3, der4}));
  ASSERT_TRUE(contains_scc(sccs_full, {der5, der6}));
  ASSERT_TRUE(contains_scc(sccs_full, {der1}));

  plansys2::Predicate pred2 = parser::pddl::fromStringPredicate("(pred2 ?x)");
  auto sccs_pred2 = state.getDerivedPredicatesSCCs({pred2});
  ASSERT_TRUE(contains_scc(sccs_pred2, {der2, der3, der4}));
  ASSERT_TRUE(contains_scc(sccs_pred2, {der5, der6}));
  ASSERT_FALSE(contains_scc(sccs_pred2, {der1}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
