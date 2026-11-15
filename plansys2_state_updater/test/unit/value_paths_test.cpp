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

// Values, paths in messages, facts, and the problem from its knowledge

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "helpers.hpp"
#include "plansys2_state_updater/paths.hpp"

using su::PathStep;

TEST(values, text_as_python_shows_it)
{
  EXPECT_EQ(su::Value(7).text(), "7");
  EXPECT_EQ(su::Value(2.0).text(), "2.0");
  EXPECT_EQ(su::Value(0.1).text(), "0.1");
  EXPECT_EQ(su::Value(true).text(), "true");
  EXPECT_EQ(su::Value("wp1").text(), "wp1");
  EXPECT_EQ(su::Value().text(), "None");
  EXPECT_EQ(msg("{a: [1, x], b: {c: null}}").json(), R"({"a": [1, "x"], "b": {"c": null}})");
  EXPECT_EQ(su::format_g(95.0), "95");
  EXPECT_EQ(su::format_g(0.5), "0.5");
}

TEST(values, from_yaml_as_it_looks)
{
  const auto v = msg("{i: 2, r: 2.5, t: true, s: wp1, q: '2', n: ~, l: [1, 2]}").as_map();
  EXPECT_TRUE(v.at("i").is_int());
  EXPECT_TRUE(v.at("r").is_double());
  EXPECT_TRUE(v.at("t").is_bool());
  EXPECT_TRUE(v.at("s").is_string());
  EXPECT_TRUE(v.at("q").is_string());  // Quoted: a string
  EXPECT_TRUE(v.at("n").is_null());
  EXPECT_EQ(v.at("l").as_list().size(), 2u);
  // Numbers compare by value, whatever their type
  EXPECT_EQ(su::Value(2), su::Value(2.0));
  EXPECT_EQ(su::Value(true), su::Value(1));
  EXPECT_NE(su::Value("2"), su::Value(2));
  EXPECT_DOUBLE_EQ(su::Value("3.5").as_number(), 3.5);
  EXPECT_THROW(su::Value("wp1").as_number(), std::invalid_argument);
}

TEST(paths, fields_indexes_and_wildcard)
{
  const auto m = msg("{pose: {pose: {position: {z: -3.5}}}, cells: [{v: 1}, {v: 2}, {v: 3}]}");
  EXPECT_DOUBLE_EQ(su::get_field(m, su::parse_path("pose.pose.position.z")).as_number(), -3.5);
  EXPECT_DOUBLE_EQ(su::get_field(m, su::parse_path("cells[2].v")).as_number(), 3);
  const auto path = su::parse_path("status[*].level");
  ASSERT_EQ(path.size(), 3u);
  EXPECT_EQ(path[1].kind, PathStep::WILDCARD);
  EXPECT_THROW(su::parse_path("a[*].b[*]"), std::invalid_argument);
  EXPECT_THROW(su::parse_path("a..b"), std::invalid_argument);
  EXPECT_THROW(su::get_field(m, su::parse_path("cells[7].v")), std::out_of_range);
  EXPECT_THROW(su::get_field(m, su::parse_path("voltage")), std::out_of_range);
  // An empty path is the whole message
  EXPECT_TRUE(su::parse_path("").empty());
  EXPECT_EQ(su::get_field(m, {}), m);
}

TEST(paths, items_of_a_wildcard)
{
  const auto m = msg("{status: [{level: 0}, {level: 2}]}");
  const auto items = su::get_items(m, su::parse_path("status[*].level"));
  ASSERT_EQ(items.size(), 2u);
  EXPECT_EQ(items[1].second, su::Value(2));
  EXPECT_EQ(items[1].first, msg("{level: 2}"));
  EXPECT_EQ(su::get_items(m, su::parse_path("status")).size(), 1u);
}

TEST(paths, facts)
{
  using Fact = std::pair<std::string, std::vector<std::string>>;
  EXPECT_EQ(su::parse_fact("(energy)"), Fact("energy", {}));
  EXPECT_EQ(su::parse_fact("( sensor_stale  battery )"), Fact("sensor_stale", {"battery"}));
  EXPECT_THROW(su::parse_fact("energy"), std::invalid_argument);
  EXPECT_EQ(su::canonical("( a  b )"), "(a b)");
  EXPECT_TRUE(su::is_name("wp-1_a"));
  EXPECT_FALSE(su::is_name("1wp"));
  EXPECT_FALSE(su::is_name("bad name"));
}

TEST(paths, the_problem_from_its_knowledge)
{
  const auto [predicates, functions] = su::parse_knowledge(
    {"(leak_detected)", "( sensor_ok  imu )", "nonsense"},
    {"(= (energy) 95.5)", "(= (depth_of auv1) -3)", "(= (pos r1) kitchen)", "(x)"});
  EXPECT_EQ(predicates, (std::set<std::string>{"(leak_detected)", "(sensor_ok imu)"}));
  EXPECT_EQ(
    functions, (std::map<std::string, double>{{"(energy)", 95.5}, {"(depth_of auv1)", -3.0}}));
}
