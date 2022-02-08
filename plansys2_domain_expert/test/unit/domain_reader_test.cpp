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

#include <string>
#include <fstream>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"

#include "plansys2_domain_expert/DomainReader.hpp"

class DomainReaderTest : public plansys2::DomainReader
{
public:
  std::size_t get_end_block_test(const std::string & domain, std::size_t init_pos)
  {
    return get_end_block(domain, init_pos);
  }

  std::string get_name_test(std::string & domain)
  {
    return get_name(domain);
  }

  std::string get_requirements_test(std::string & domain)
  {
    return get_requirements(domain);
  }

  std::string get_types_test(const std::string & domain)
  {
    return get_types(domain);
  }

  std::string get_constants_test(const std::string & domain)
  {
    return get_constants(domain);
  }

  std::string get_predicates_test(const std::string & domain)
  {
    return get_predicates(domain);
  }

  std::string get_functions_test(const std::string & domain)
  {
    return get_functions(domain);
  }
  std::vector<std::string> get_actions_test(const std::string & domain)
  {
    return get_actions(domain);
  }

  void add_domain_test(const std::string & domain)
  {
    add_domain(domain);
  }

  std::string get_joint_domain_test()
  {
    return get_joint_domain();
  }
};

TEST(domain_reader, get_block)
{
  DomainReaderTest dr;

  std::string block1 = "";
  std::string block2 = ")";
  std::string block3 = "\n)";
  std::string block4 = "word)";
  std::string block5 = "word )";
  std::string block6 = " word )";
  std::string block7 = " (word ) )";
  std::string block8 = " word ) \n)";
  std::string block9 = " ( ";
  std::string block10 = " :strips :typing :adl :fluents :durative-actions";

  ASSERT_EQ(-1, dr.get_end_block_test(block1, 0));
  ASSERT_EQ(0, dr.get_end_block_test(block2, 0));
  ASSERT_EQ(1, dr.get_end_block_test(block3, 0));
  ASSERT_EQ(4, dr.get_end_block_test(block4, 0));
  ASSERT_EQ(4, dr.get_end_block_test(block4, 1));
  ASSERT_EQ(4, dr.get_end_block_test(block4, 2));
  ASSERT_EQ(5, dr.get_end_block_test(block5, 0));
  ASSERT_EQ(6, dr.get_end_block_test(block6, 0));
  ASSERT_EQ(9, dr.get_end_block_test(block7, 0));
  ASSERT_EQ(6, dr.get_end_block_test(block8, 0));
  ASSERT_EQ(-1, dr.get_end_block_test(block9, 0));
  ASSERT_EQ(-1, dr.get_end_block_test(block10, 0));
}

TEST(domain_reader, name)
{
  DomainReaderTest dr;

  std::string name1_str = "(define (domain simple)";
  std::string name1_estr = "simple";

  std::string name2_str = "(\ndefine (\ndomain simple\n)";
  std::string name2_estr = "simple";

  std::string name3_str = "(define (domain simple\n) (\n))";
  std::string name3_estr = "simple";

  std::string name4_str = "(define (domain simple";
  std::string name4_estr = "";

  std::string name5_str = "(define (domain ";
  std::string name5_estr = "";

  std::string name6_str = "";
  std::string name6_estr = "";

  auto res1 = dr.get_name_test(name1_str);
  auto res2 = dr.get_name_test(name2_str);
  auto res3 = dr.get_name_test(name3_str);
  auto res4 = dr.get_name_test(name4_str);
  auto res5 = dr.get_name_test(name5_str);
  auto res6 = dr.get_name_test(name6_str);

  ASSERT_EQ(res1, name1_estr);
  ASSERT_EQ(res2, name2_estr);
  ASSERT_EQ(res3, name3_estr);
  ASSERT_EQ(res4, name4_estr);
  ASSERT_EQ(res5, name5_estr);
  ASSERT_EQ(res6, name6_estr);
}

TEST(domain_reader, requirements)
{
  DomainReaderTest dr;

  std::string req1_str = "(:requirements :strips :typing :adl :fluents :durative-actions)";
  std::string req1_estr = " :strips :typing :adl :fluents :durative-actions";

  std::string req2_str = "(\n:requirements :strips :typing :adl :fluents :durative-actions\n)";
  std::string req2_estr = " :strips :typing :adl :fluents :durative-actions";

  std::string req3_str = "(:requirements :strips :typing :adl :fluents :durative-actions\n) (\n))";
  std::string req3_estr = " :strips :typing :adl :fluents :durative-actions";

  std::string req4_str = "(:requirements :strips :typing :adl :fluents :durative-actions";
  std::string req4_estr = "";

  std::string req5_str = "(:requirements )";
  std::string req5_estr = "";

  std::string req6_str = "";
  std::string req6_estr = "";

  auto res1 = dr.get_requirements_test(req1_str);
  auto res2 = dr.get_requirements_test(req2_str);
  auto res3 = dr.get_requirements_test(req3_str);
  auto res4 = dr.get_requirements_test(req4_str);
  auto res5 = dr.get_requirements_test(req5_str);
  auto res6 = dr.get_requirements_test(req6_str);

  ASSERT_EQ(res1, req1_estr);
  ASSERT_EQ(req1_str, "(:requirements)");

  ASSERT_EQ(res2, req2_estr);
  ASSERT_EQ(req2_str, "(\n:requirements)");

  ASSERT_EQ(res3, req3_estr);
  ASSERT_EQ(res4, req4_estr);
  ASSERT_EQ(res5, req5_estr);
  ASSERT_EQ(res6, req6_estr);
}

TEST(domain_reader, types)
{
  DomainReaderTest dr;

  std::string req1_str = "(:types type1 type2)";
  std::string req1_estr = " type1 type2";

  std::string req2_str = "(:types\ntype1 type2\n)";
  std::string req2_estr = "type1 type2";

  std::string req3_str = "(:types\ntype1\ntype2\n)";
  std::string req3_estr = "type1\ntype2";

  std::string req4_str = "(:types\ntype1\ntype2\n) ) ";
  std::string req4_estr = "type1\ntype2";

  std::string req5_str = "(:types\ntype1\ntype2\n";
  std::string req5_estr = "";

  std::string req6_str = "(:types )";
  std::string req6_estr = "";

  std::string req7_str = "";
  std::string req7_estr = "";

  auto res1 = dr.get_types_test(req1_str);
  auto res2 = dr.get_types_test(req2_str);
  auto res3 = dr.get_types_test(req3_str);
  auto res4 = dr.get_types_test(req4_str);
  auto res5 = dr.get_types_test(req5_str);
  auto res6 = dr.get_types_test(req6_str);
  auto res7 = dr.get_types_test(req7_str);

  ASSERT_EQ(res1, req1_estr);
  ASSERT_EQ(res2, req2_estr);
  ASSERT_EQ(res3, req3_estr);
  ASSERT_EQ(res4, req4_estr);
  ASSERT_EQ(res5, req5_estr);
  ASSERT_EQ(res6, req6_estr);
  ASSERT_EQ(res7, req7_estr);
}

TEST(domain_reader, constants)
{
  DomainReaderTest dr;

  std::string req1_str = "(:constants start - location  robotino - robot)";
  std::string req1_estr = " start - location  robotino - robot";

  std::string req2_str = "(:constants\nstart end - location robotino - robot\n)";
  std::string req2_estr = "start end - location robotino - robot";

  std::string req3_str = "(:constants\nstart end - location\nrobotino - robot\n)";
  std::string req3_estr = "start end - location\nrobotino - robot";

  std::string req4_str = "(:constants\nstart end - location\nrobotino - robot\n) )";
  std::string req4_estr = "start end - location\nrobotino - robot";

  std::string req5_str = "(:constants\nstart end - location\nrobotino - robot\n";
  std::string req5_estr = "";

  std::string req6_str = "(:constants )";
  std::string req6_estr = "";

  std::string req7_str = "";
  std::string req7_estr = "";

  auto res1 = dr.get_constants_test(req1_str);
  auto res2 = dr.get_constants_test(req2_str);
  auto res3 = dr.get_constants_test(req3_str);
  auto res4 = dr.get_constants_test(req4_str);
  auto res5 = dr.get_constants_test(req5_str);
  auto res6 = dr.get_constants_test(req6_str);
  auto res7 = dr.get_constants_test(req7_str);

  ASSERT_EQ(res1, req1_estr);
  ASSERT_EQ(res2, req2_estr);
  ASSERT_EQ(res3, req3_estr);
  ASSERT_EQ(res4, req4_estr);
  ASSERT_EQ(res5, req5_estr);
  ASSERT_EQ(res6, req6_estr);
  ASSERT_EQ(res7, req7_estr);
}


TEST(domain_reader, predicates)
{
  DomainReaderTest dr;

  std::string req1_str = "(:predicates\n(robot_at leia bedroom) (person_at paco kitchen)\n)";
  std::string req1_estr = "(robot_at leia bedroom) (person_at paco kitchen)";

  std::string req2_str = "(:predicates\n(robot_at leia bedroom) (person_at paco kitchen\n";
  std::string req2_estr = "";

  std::string req3_str = "(:predicates )";
  std::string req3_estr = "";

  std::string req4_str = "";
  std::string req4_estr = "";


  auto res1 = dr.get_predicates_test(req1_str);
  auto res2 = dr.get_predicates_test(req2_str);
  auto res3 = dr.get_predicates_test(req3_str);
  auto res4 = dr.get_predicates_test(req4_str);

  ASSERT_EQ(res1, req1_estr);
  ASSERT_EQ(res2, req2_estr);
  ASSERT_EQ(res3, req3_estr);
  ASSERT_EQ(res4, req4_estr);
}

TEST(domain_reader, functions)
{
  DomainReaderTest dr;

  std::string req1_str =
    "(:functions\n(=(robot_at leia bedroom) 10)\n(=(person_at paco kitchen) 30)\n)";
  std::string req1_estr = "(=(robot_at leia bedroom) 10)\n(=(person_at paco kitchen) 30)";

  std::string req2_str =
    "(:functions\n(=(robot_at leia bedroom) 10)\n(=(person_at paco kitchen) 30\n)";
  std::string req2_estr = "";

  std::string req3_str = "(:functions )";
  std::string req3_estr = "";

  std::string req4_str = "";
  std::string req4_estr = "";


  auto res1 = dr.get_functions_test(req1_str);
  auto res2 = dr.get_functions_test(req2_str);
  auto res3 = dr.get_functions_test(req3_str);
  auto res4 = dr.get_functions_test(req4_str);

  ASSERT_EQ(res1, req1_estr);
  ASSERT_EQ(res2, req2_estr);
  ASSERT_EQ(res3, req3_estr);
  ASSERT_EQ(res4, req4_estr);
}

TEST(domain_reader, actions)
{
  DomainReaderTest dr;

  std::string req0_str = "(:predicates\n(robot_at leia bedroom) (person_at paco kitchen)\n)";

  std::string req1_str =
    "(:action\n whatever \n)";
  std::string req1_estr = "(:action\n whatever \n)";

  std::string req2_str =
    "\n text other (:action\n whatever \n) more text";
  std::string req2_estr = "(:action\n whatever \n)";

  std::string req3_str = std::string("((:types type1 type2)\n(:action\n   whatever\n) \n") +
    std::string("(:durative-action\n  whatever\n)");
  std::string req3_1_estr = "(:action\n   whatever\n)";
  std::string req3_2_estr = "(:durative-action\n  whatever\n)";

  std::string req4_str = std::string("((:types type1 type2)\n(:action\n   whatever\n) \n") +
    std::string("(:durative-action\n  whatever\n");
  std::string req4_1_estr = "(:action\n   whatever\n)";

  std::string req5_str = std::string("");


  auto res0 = dr.get_actions_test(req0_str);
  auto res1 = dr.get_actions_test(req1_str);
  auto res2 = dr.get_actions_test(req2_str);
  auto res3 = dr.get_actions_test(req3_str);
  auto res4 = dr.get_actions_test(req4_str);
  auto res5 = dr.get_actions_test(req5_str);

  ASSERT_TRUE(res0.empty());

  ASSERT_FALSE(res1.empty());
  ASSERT_EQ(res1.size(), 1u);
  ASSERT_EQ(res1[0], req1_estr);

  ASSERT_FALSE(res2.empty());
  ASSERT_EQ(res2.size(), 1u);
  ASSERT_EQ(res2[0], req2_estr);

  ASSERT_EQ(res3.size(), 2u);
  ASSERT_EQ(res3[0], req3_1_estr);
  ASSERT_EQ(res3[1], req3_2_estr);

  ASSERT_EQ(res4.size(), 1u);
  ASSERT_EQ(res4[0], req4_1_estr);

  ASSERT_EQ(res5.size(), 0u);
}

TEST(domain_reader, add_domain)
{
  DomainReaderTest dr;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  dr.add_domain(domain_str);

  ASSERT_EQ(dr.get_joint_domain_test(), domain_str_p);
}

TEST(domain_reader, add_2_domain)
{
  DomainReaderTest dr;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_2(pkgpath + "/pddl/domain_simple_ext.pddl");
  std::string domain_str_2((
      std::istreambuf_iterator<char>(domain_ifs_2)),
    std::istreambuf_iterator<char>());


  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_combined_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  dr.add_domain(domain_str);
  dr.add_domain(domain_str_2);

  ASSERT_EQ(dr.get_joint_domain_test(), domain_str_p);
}

TEST(domain_reader, add_domain_with_constants)
{
  DomainReaderTest dr;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_constants_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  dr.add_domain(domain_str);

  ASSERT_EQ(dr.get_joint_domain_test(), domain_str_p);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
