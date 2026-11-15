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

// Messages of any type, known by their name, read by introspection

#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gtest/gtest.h"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "plansys2_state_updater/message_reader.hpp"
#include "plansys2_state_updater/paths.hpp"

namespace su = plansys2_state_updater;

namespace
{

template<typename MessageT>
su::Value read(const std::string & type, const MessageT & message)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<MessageT>().serialize_message(&message, &serialized);
  return su::MessageReader(type).read(serialized);
}

su::Value field(const su::Value & message, const std::string & path)
{
  return su::get_field(message, su::parse_path(path));
}

}  // namespace

TEST(message_reader, numbers_arrays_and_nested_messages)
{
  sensor_msgs::msg::BatteryState battery;
  battery.percentage = 0.5f;
  battery.voltage = 24.2f;
  battery.present = true;
  battery.power_supply_status = 3;
  battery.cell_voltage = {3.7f, 3.5f};
  battery.location = "aft";
  battery.header.frame_id = "base";
  battery.header.stamp.sec = 7;
  const su::Value message = read("sensor_msgs/msg/BatteryState", battery);
  EXPECT_DOUBLE_EQ(field(message, "percentage").as_number(), 0.5);
  EXPECT_NEAR(field(message, "voltage").as_number(), 24.2, 1e-5);
  EXPECT_EQ(field(message, "present"), su::Value(true));
  EXPECT_EQ(field(message, "power_supply_status"), su::Value(3));
  EXPECT_NEAR(field(message, "cell_voltage[1]").as_number(), 3.5, 1e-6);
  EXPECT_EQ(field(message, "cell_voltage").as_list().size(), 2u);
  EXPECT_EQ(field(message, "location"), su::Value("aft"));
  EXPECT_EQ(field(message, "header.frame_id"), su::Value("base"));
  EXPECT_EQ(field(message, "header.stamp.sec"), su::Value(7));
}

TEST(message_reader, bytes_are_numbers)
{
  // The level of a DiagnosticStatus is a byte: 2, not a character
  diagnostic_msgs::msg::DiagnosticArray array;
  array.status.resize(2);
  array.status[0].name = "imu";
  array.status[0].level = 0;
  array.status[1].name = "dvl";
  array.status[1].level = 2;
  array.status[1].values.resize(1);
  array.status[1].values[0].key = "temperature";
  const su::Value message = read("diagnostic_msgs/msg/DiagnosticArray", array);
  const auto items = su::get_items(message, su::parse_path("status[*].level"));
  ASSERT_EQ(items.size(), 2u);
  EXPECT_EQ(items[0].second, su::Value(0));
  EXPECT_EQ(items[1].second, su::Value(2));
  EXPECT_EQ(field(message, "status[1].values[0].key"), su::Value("temperature"));
}

TEST(message_reader, sequences_of_strings_and_of_messages)
{
  plansys2_msgs::msg::Knowledge knowledge;
  knowledge.instances = {"r2d2", "kitchen"};
  knowledge.predicates = {"(robot_at r2d2 kitchen)"};
  knowledge.goal = "(and (robot_at r2d2 hall))";
  const su::Value message = read("plansys2_msgs/msg/Knowledge", knowledge);
  EXPECT_EQ(field(message, "instances[1]"), su::Value("kitchen"));
  EXPECT_EQ(field(message, "goal"), su::Value("(and (robot_at r2d2 hall))"));

  plansys2_msgs::msg::Tree tree;
  tree.nodes.resize(2);
  tree.nodes[1].name = "energy";
  tree.nodes[1].negate = true;
  tree.nodes[0].children = {1};
  const su::Value nodes = read("plansys2_msgs/msg/Tree", tree);
  EXPECT_EQ(field(nodes, "nodes[1].name"), su::Value("energy"));
  EXPECT_EQ(field(nodes, "nodes[1].negate"), su::Value(true));
  EXPECT_EQ(field(nodes, "nodes[0].children[0]"), su::Value(1));
}

TEST(message_reader, unknown_types)
{
  EXPECT_THROW(su::MessageReader("no_package/msg/Nothing"), std::runtime_error);
  EXPECT_THROW(su::MessageReader("sensor_msgs/msg/Nothing"), std::runtime_error);
}
