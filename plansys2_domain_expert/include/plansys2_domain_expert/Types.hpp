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

#ifndef PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_
#define PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_

#include <optional>
#include <string>
#include <vector>

namespace plansys2
{

struct Param
{
  std::string name;
  std::string type;
};

class Predicate
{
public:
  std::string toString()
  {
    std::string ret;
    ret = "(" + name;
    for (const auto & param : parameters) {
      ret += " " + param.name;
    }

    ret += ")";

    return ret;
  }

  std::string name;
  std::vector<Param> parameters;
};

typedef enum {AND, OR, NOT, PREDICATE, UNKNOWN} NodeType;

class TreeNode
{
public:
  explicit TreeNode(NodeType type)
  : type_(type) {}
  virtual ~TreeNode() {}

  virtual std::string toString() = 0;
  NodeType type_;
};

TreeNode * get_tree_node(const std::string & expr);

class PredicateNode : public TreeNode
{
public:
  PredicateNode()
  : TreeNode(PREDICATE) {}

  std::string toString()
  {
    return predicate_.toString();
  }

  Predicate predicate_;
};

class AndNode : public TreeNode
{
public:
  AndNode()
  : TreeNode(AND) {}

  ~AndNode() override
  {
    for (auto op : ops) {
      if (op != nullptr) {delete op;}
    }
  }

  std::string toString()
  {
    std::string ret;
    ret = "(AND ";
    for (auto op : ops) {
      ret += op->toString();
    }
    ret += ")";

    return ret;
  }

  std::vector<TreeNode *> ops;
};

class OrNode : public TreeNode
{
public:
  OrNode()
  : TreeNode(OR) {}

  ~OrNode() override
  {
    for (auto op : ops) {
      if (op != nullptr) {delete op;}
    }
  }
  std::string toString()
  {
    std::string ret;
    ret = "(OR ";
    for (auto op : ops) {
      ret += op->toString();
    }
    ret += ")";

    return ret;
  }

  std::vector<TreeNode *> ops;
};

class NotNode : public TreeNode
{
public:
  NotNode()
  : TreeNode(NOT), op(nullptr) {}

  ~NotNode() override
  {
    if (op != nullptr) {delete op;}
  }

  std::string toString()
  {
    std::string ret;
    ret = "(NOT ";
    ret += op->toString();
    ret += ")";

    return ret;
  }

  TreeNode * op;
};

class PredicateTree
{
public:
  std::string toString()
  {
    return root->toString();
  }

  void fromString(const std::string & expr)
  {
    root = get_tree_node(expr);
  }

  TreeNode * root;
};

struct Action
{
  std::string name;
  std::vector<Param> parameters;
  PredicateTree preconditions;
  PredicateTree effects;
};

struct DurativeAction
{
  std::string name;
  std::vector<Param> parameters;
  PredicateTree at_start_requirements;
  PredicateTree over_all_requirements;
  PredicateTree at_end_requirements;
  PredicateTree efects;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_
