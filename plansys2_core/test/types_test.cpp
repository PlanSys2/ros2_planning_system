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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/Types.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

TEST(types_test, predicates)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predA2 = parser::pddl::fromStringPredicate("(predicateA ?c)");
  plansys2::Predicate predAconstant = parser::pddl::fromStringPredicate("(predicateA a)");
  
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");
  plansys2::Predicate predBconstant = parser::pddl::fromStringPredicate("(predicateB b)");

  ASSERT_EQ(predA, predA);
  ASSERT_NE(predA, predA2);
  ASSERT_NE(predA, predAconstant);
  
  ASSERT_EQ(std::hash<plansys2::Predicate>{}(predA), std::hash<plansys2::Predicate>{}(predA));
  ASSERT_NE(std::hash<plansys2::Predicate>{}(predA), std::hash<plansys2::Predicate>{}(predA2));
  ASSERT_NE(std::hash<plansys2::Predicate>{}(predA), std::hash<plansys2::Predicate>{}(predAconstant));
  
  ASSERT_EQ(predB, predB);
  ASSERT_NE(predB, predBconstant);
  
  ASSERT_NE(predA, predB);

  std::unordered_set<plansys2::Predicate> s;
  s.insert(predA);
  auto it = s.find(predA);
  ASSERT_NE(it, s.end());
  ASSERT_TRUE(*it == predA);

  auto it2 = s.find(predA2);
  ASSERT_EQ(it2, s.end());
}

TEST(types_test, derived_predicates)
{
  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

  plansys2::Derived inferredAconstant;
  inferredAconstant.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredAconstant.preconditions = parser::pddl::fromString("(and (predicateA a))");

  ASSERT_EQ(inferredA, inferredA);
  ASSERT_NE(inferredA, inferredAconstant);
  ASSERT_NE(std::hash<plansys2::Derived>{}(inferredA), std::hash<plansys2::Derived>{}(inferredAconstant));

  std::unordered_set<plansys2::Derived> s;
  s.insert(inferredA);
  auto it = s.find(inferredA);
  ASSERT_NE(it, s.end());
  ASSERT_TRUE(*it == inferredA);
  
  auto it2 = s.find(inferredAconstant);
  ASSERT_EQ(it2, s.end());

  plansys2::Derived inferred_function_design_fd_better;
  inferred_function_design_fd_better.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  inferred_function_design_fd_better.preconditions = parser::pddl::fromString("(exists (?y) (and (inferred-FdBetterUtility ?x ?y)))");
  s.insert(inferred_function_design_fd_better);
  
  plansys2::Derived inferred_function_design_fd_better_inverse;
  inferred_function_design_fd_better_inverse.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?y)");
  inferred_function_design_fd_better_inverse.preconditions = parser::pddl::fromString("(exists (?x) (and (inferred-FdBetterUtility ?x ?y)))");

  ASSERT_NE(inferred_function_design_fd_better, inferred_function_design_fd_better_inverse);
  ASSERT_NE(std::hash<plansys2::Derived>{}(inferred_function_design_fd_better), std::hash<plansys2::Derived>{}(inferred_function_design_fd_better_inverse));
  auto it3 = s.find(inferred_function_design_fd_better_inverse);
  ASSERT_EQ(it3, s.end());
}

TEST(types_test, derived_predicates_get_normalized)
{
  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  auto norm_inferredA = inferredA.getNormalizedDerived();

  plansys2::Derived inferredAconstant;
  inferredAconstant.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredAconstant.preconditions = parser::pddl::fromString("(and (predicateA a))");
  auto norm_inferredAconstant = inferredAconstant.getNormalizedDerived();

  ASSERT_NE(inferredA, norm_inferredA);
  ASSERT_EQ(norm_inferredA.predicate.parameters[0].name, "?0");
  ASSERT_EQ(norm_inferredA.preconditions.nodes[1].parameters[0].name, "?0");
  
  ASSERT_NE(inferredAconstant, norm_inferredAconstant);
  ASSERT_EQ(norm_inferredAconstant.predicate.parameters[0].name, "?0");
  ASSERT_EQ(norm_inferredAconstant.preconditions.nodes[1].parameters[0].name, "a");


  plansys2::Derived fd_better;
  fd_better.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
  fd_better.preconditions = parser::pddl::fromString("(exists (?y) (and (inferred-FdBetterUtility ?x ?y)))");
  auto fd_better_norm = fd_better.getNormalizedDerived();
  
  plansys2::Derived fd_better_inverse;
  fd_better_inverse.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?y)");
  fd_better_inverse.preconditions = parser::pddl::fromString("(exists (?x) (and (inferred-FdBetterUtility ?x ?y)))");
  auto fd_better_inverse_norm = fd_better_inverse.getNormalizedDerived();

  ASSERT_NE(fd_better, fd_better_inverse);
  ASSERT_NE(fd_better_norm, fd_better_inverse_norm);
  
  ASSERT_NE(fd_better, fd_better_norm);
  ASSERT_EQ(fd_better_norm.predicate.parameters[0].name, "?0");
  ASSERT_EQ(fd_better_norm.preconditions.nodes[0].parameters[0].name, "?1");
  ASSERT_EQ(fd_better_norm.preconditions.nodes[2].parameters[0].name, "?0");
  ASSERT_EQ(fd_better_norm.preconditions.nodes[2].parameters[1].name, "?1");

  ASSERT_NE(fd_better_inverse, fd_better_inverse_norm);
  ASSERT_EQ(fd_better_inverse_norm.predicate.parameters[0].name, "?0");
  ASSERT_EQ(fd_better_inverse_norm.preconditions.nodes[0].parameters[0].name, "?1");
  ASSERT_EQ(fd_better_inverse_norm.preconditions.nodes[2].parameters[0].name, "?1");
  ASSERT_EQ(fd_better_inverse_norm.preconditions.nodes[2].parameters[1].name, "?0");

  plansys2::Derived fd_better_2;
  fd_better_2.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?z)");
  fd_better_2.preconditions = parser::pddl::fromString("(exists (?w) (and (inferred-FdBetterUtility ?z ?w)))");
  auto fd_better_2_norm = fd_better_2.getNormalizedDerived();

  ASSERT_NE(fd_better, fd_better_2);
  ASSERT_EQ(fd_better_norm, fd_better_2_norm);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}