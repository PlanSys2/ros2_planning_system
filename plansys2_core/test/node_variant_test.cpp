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
#include "plansys2_core/NodeVariant.hpp"
#include "plansys2_pddl_parser/Utils.hpp"


TEST(types_test, node_variant_equality)
{
    plansys2::Predicate predA_var_a = parser::pddl::fromStringPredicate("(predA ?a)");
    plansys2::Predicate predA_var_x = parser::pddl::fromStringPredicate("(predA ?x)");
    plansys2::Predicate predA_const_a = parser::pddl::fromStringPredicate("(predA a)");
    plansys2::Predicate predA_const_b = parser::pddl::fromStringPredicate("(predA b)");
    plansys2::Predicate predA_2_vars = parser::pddl::fromStringPredicate("(predA ?a ?b)");

    auto nv_predA_var_a = plansys2::NodeVariant(predA_var_a);
    auto nv_predA_var_x = plansys2::NodeVariant(predA_var_x);
    auto nv_predA_const_a = plansys2::NodeVariant(predA_const_a);
    auto nv_predA_const_b = plansys2::NodeVariant(predA_const_b);
    auto nv_predA_2_vars = plansys2::NodeVariant(predA_2_vars);

    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_var_a), std::hash<plansys2::NodeVariant>{}(nv_predA_var_a));
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_var_a), std::hash<plansys2::NodeVariant>{}(nv_predA_var_x));
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_var_a), std::hash<plansys2::NodeVariant>{}(nv_predA_const_a));
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_var_a), std::hash<plansys2::NodeVariant>{}(nv_predA_const_b));
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_const_a), std::hash<plansys2::NodeVariant>{}(nv_predA_var_a));
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_const_a), std::hash<plansys2::NodeVariant>{}(nv_predA_var_x));
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_predA_const_a), std::hash<plansys2::NodeVariant>{}(nv_predA_const_a));
    ASSERT_NE(std::hash<plansys2::NodeVariant>{}(nv_predA_var_a), std::hash<plansys2::NodeVariant>{}(nv_predA_2_vars));
    
    ASSERT_EQ(nv_predA_var_a, nv_predA_var_a);
    ASSERT_EQ(nv_predA_var_a, nv_predA_var_x);
    ASSERT_EQ(nv_predA_var_a, nv_predA_const_a);
    ASSERT_EQ(nv_predA_var_a, nv_predA_const_b);
    ASSERT_EQ(nv_predA_const_a, nv_predA_var_a);
    ASSERT_EQ(nv_predA_const_a, nv_predA_var_x);
    ASSERT_EQ(nv_predA_const_a, nv_predA_const_a);
    ASSERT_NE(nv_predA_const_a, nv_predA_const_b);

    plansys2::Derived inferredA;
    inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
    inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

    plansys2::Derived inferredAconstant;
    inferredAconstant.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
    inferredAconstant.preconditions = parser::pddl::fromString("(and (predicateA a))");

    auto nv_inferredA = plansys2::NodeVariant(inferredA);
    auto nv_inferredAconstant = plansys2::NodeVariant(inferredAconstant);

    ASSERT_EQ(nv_inferredA, nv_inferredA);
    ASSERT_NE(nv_inferredA, nv_inferredAconstant);
    ASSERT_NE(std::hash<plansys2::NodeVariant>{}(nv_inferredA), std::hash<plansys2::NodeVariant>{}(nv_inferredAconstant));

    plansys2::Derived inferred_function_design_fd_better;
    inferred_function_design_fd_better.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?x)");
    inferred_function_design_fd_better.preconditions = parser::pddl::fromString("(exists (?y) (and (inferred-FdBetterUtility ?x ?y)))");
    
    plansys2::Derived inferred_function_design_fd_better_inverse;
    inferred_function_design_fd_better_inverse.predicate = parser::pddl::fromStringPredicate("(inferred-FunctionDesign ?y)");
    inferred_function_design_fd_better_inverse.preconditions = parser::pddl::fromString("(exists (?x) (and (inferred-FdBetterUtility ?x ?y)))");

    auto nv_inferred_function_design_fd_better = plansys2::NodeVariant(inferred_function_design_fd_better);
    auto nv_inferred_function_design_fd_better_inverse = plansys2::NodeVariant(inferred_function_design_fd_better_inverse);

    ASSERT_EQ(nv_inferred_function_design_fd_better, nv_inferred_function_design_fd_better);
    ASSERT_NE(nv_inferred_function_design_fd_better, nv_inferred_function_design_fd_better_inverse);
    ASSERT_NE(std::hash<plansys2::NodeVariant>{}(nv_inferred_function_design_fd_better), std::hash<plansys2::NodeVariant>{}(nv_inferred_function_design_fd_better_inverse));

    plansys2::Derived inferredAB;
    inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?x ?y)");
    inferredAB.preconditions = parser::pddl::fromString("(and (predicateA ?x)(predicateB ?y))");

    plansys2::Derived inferredABi;
    inferredABi.predicate = parser::pddl::fromStringPredicate("(inferredAB ?y ?x)");
    inferredABi.preconditions = parser::pddl::fromString("(and (predicateA ?y)(predicateB ?x))");

    auto nv_inferredAB = plansys2::NodeVariant(inferredAB);
    auto nv_inferredABi = plansys2::NodeVariant(inferredABi);

    ASSERT_EQ(nv_inferredAB, nv_inferredAB);
    ASSERT_EQ(nv_inferredAB, nv_inferredABi);
    ASSERT_EQ(std::hash<plansys2::NodeVariant>{}(nv_inferredAB), std::hash<plansys2::NodeVariant>{}(nv_inferredABi));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}