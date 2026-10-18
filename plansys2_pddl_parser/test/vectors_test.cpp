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

// The vectors shared with the Python parser: see vectors/README.md

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Utils.hpp"

using Node = plansys2_msgs::msg::Node;

namespace
{
const std::map<std::string, uint8_t> kTypes = {
  {"UNKNOWN", Node::UNKNOWN}, {"AND", Node::AND}, {"OR", Node::OR}, {"NOT", Node::NOT},
  {"PREDICATE", Node::PREDICATE}, {"FUNCTION", Node::FUNCTION},
  {"EXPRESSION", Node::EXPRESSION}, {"FUNCTION_MODIFIER", Node::FUNCTION_MODIFIER},
  {"NUMBER", Node::NUMBER}, {"CONSTANT", Node::CONSTANT}, {"PARAMETER", Node::PARAMETER},
  {"EXISTS", Node::EXISTS}, {"FORALL", Node::FORALL}, {"IMPLY", Node::IMPLY},
  {"WHEN", Node::WHEN}, {"PREFERENCE", Node::PREFERENCE}, {"CONSTRAINT", Node::CONSTRAINT}};

std::string typeName(uint8_t type)
{
  for (const auto & [name, value] : kTypes) {
    if (value == type) {return name;}
  }
  return "?";
}

std::string unescape(const std::string & text)
{
  std::string out;
  for (size_t i = 0; i < text.size(); i++) {
    const char next = i + 1 < text.size() ? text[i + 1] : '\0';
    if (text[i] == '\\' && (next == 't' || next == 'n' || next == 'r')) {
      out += next == 't' ? '\t' : next == 'n' ? '\n' : '\r';
      i++;
    } else {
      out += text[i];
    }
  }
  return out;
}

// Rows of a vector file, each with its columns unescaped
std::vector<std::vector<std::string>> rows(const std::string & file, size_t columns)
{
  std::ifstream in(std::string(VECTORS_DIR) + "/" + file);
  EXPECT_TRUE(in.good()) << file;
  std::vector<std::vector<std::string>> out;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line[0] == '#') {continue;}
    std::vector<std::string> row(1);
    for (char c : line) {
      if (c == '\t') {row.emplace_back();} else {row.back() += c;}
    }
    EXPECT_EQ(row.size(), columns) << file << ": " << line;
    if (row.size() != columns) {continue;}
    for (auto & column : row) {
      column = unescape(column);
    }
    out.push_back(row);
  }
  EXPECT_FALSE(out.empty()) << file;
  return out;
}
}  // namespace

TEST(vectors, round_trip)
{
  for (const auto & row : rows("round_trip.tsv", 4)) {
    const auto tree = parser::pddl::fromString(row[1], false, kTypes.at(row[0]));
    const auto written = parser::pddl::toString(tree);
    EXPECT_EQ(written, row[2]) << row[1];
    std::string types;
    for (const auto & node : tree.nodes) {
      types += (types.empty() ? "" : " ") + typeName(node.node_type);
    }
    EXPECT_EQ(types, row[3]) << row[1];
    std::string error;
    EXPECT_TRUE(parser::pddl::validateTree(tree, error)) << row[1] << ": " << error;
    // What is written reads back the same
    if (!written.empty()) {
      EXPECT_EQ(
        parser::pddl::toString(parser::pddl::fromString(written, false, kTypes.at(row[0]))),
        written) << row[1];
    }
  }
}

TEST(vectors, node_types)
{
  for (const auto & row : rows("node_types.tsv", 3)) {
    EXPECT_EQ(typeName(parser::pddl::getNodeType(row[1], kTypes.at(row[0]))), row[2]) << row[1];
  }
}

TEST(vectors, functions)
{
  for (const auto & row : rows("functions.tsv", 4)) {
    const auto function = parser::pddl::fromStringFunction(row[0]);
    std::string parameters;
    for (const auto & p : function.parameters) {
      parameters += (parameters.empty() ? "" : " ") + p.name;
    }
    EXPECT_EQ(function.name, row[1]) << row[0];
    EXPECT_EQ(parameters, row[2]) << row[0];
    EXPECT_EQ(parser::pddl::numberToString(function.value, -1), row[3]) << row[0];
  }
}

TEST(vectors, object_functions)
{
  for (const auto & row : rows("object_functions.tsv", 4)) {
    const auto function = parser::pddl::fromStringFunction(row[0]);
    std::string parameters;
    for (const auto & p : function.parameters) {
      parameters += (parameters.empty() ? "" : " ") + p.name;
    }
    EXPECT_EQ(function.name, row[1]) << row[0];
    EXPECT_EQ(parameters, row[2]) << row[0];
    EXPECT_EQ(function.object, row[3]) << row[0];
  }
}

TEST(vectors, numbers)
{
  for (const auto & row : rows("numbers.tsv", 2)) {
    double value = 0.0;
    const bool ok = parser::pddl::parseNumber(row[0], value);
    EXPECT_EQ(ok ? parser::pddl::numberToString(value, -1) : "-", row[1]) << "[" << row[0] << "]";
  }
}

TEST(vectors, equality)
{
  for (const auto & row : rows("equality.tsv", 3)) {
    const auto first = parser::pddl::fromString(row[0]);
    const auto second = parser::pddl::fromString(row[1]);
    const std::string expected = row[2];
    EXPECT_EQ(parser::pddl::checkTreeEquality(first, second) ? "true" : "false", expected)
      << row[0] << " vs " << row[1];
    EXPECT_EQ(parser::pddl::checkTreeEquality(second, first) ? "true" : "false", expected)
      << row[1] << " vs " << row[0];
  }
}
