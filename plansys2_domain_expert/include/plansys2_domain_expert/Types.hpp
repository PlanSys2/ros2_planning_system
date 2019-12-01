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
#include <memory>

namespace plansys2
{

struct Param
{
  std::string name;
  std::string type;
};

bool operator==(const Param & op1, const Param & op2);

class Predicate
{
public:
  std::string toString() const
  {
    std::string ret;
    ret = "(" + name;
    for (const auto & param : parameters) {
      ret += " " + param.name;
    }

    ret += ")";

    return ret;
  }

  void fromString(const std::string & predicate)
  {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;

    while (end != std::string::npos) {
      end = predicate.find(" ", start);
      tokens.push_back(predicate.substr(start,
        (end == std::string::npos) ? std::string::npos : end - start));
      start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
    }

    tokens[0].erase(0, 1);
    name = tokens[0];

    for (size_t i = 0; i < tokens.size(); i++) {
      parameters[i].name = tokens[i];
      parameters[i].type = "";
    }
  }

  friend bool operator==(const Predicate & op1, const Predicate & op2);

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
  virtual void getPredicates(std::vector<Predicate> & predicates) = 0;
  NodeType type_;
};

std::shared_ptr<TreeNode> get_tree_node(const std::string & expr);

class PredicateNode : public TreeNode
{
public:
  PredicateNode()
  : TreeNode(PREDICATE) {}

  std::string toString()
  {
    return predicate_.toString();
  }

  void getPredicates(std::vector<Predicate> & predicates)
  {
    predicates.push_back(predicate_);
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


  void getPredicates(std::vector<Predicate> & predicates)
  {
    for (auto op : ops) {
      op->getPredicates(predicates);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> ops;
};

class OrNode : public TreeNode
{
public:
  OrNode()
  : TreeNode(OR) {}

  ~OrNode() override
  {
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

  void getPredicates(std::vector<Predicate> & predicates)
  {
    for (auto op : ops) {
      op->getPredicates(predicates);
    }
  }

  std::vector<std::shared_ptr<TreeNode>> ops;
};

class NotNode : public TreeNode
{
public:
  NotNode()
  : TreeNode(NOT), op(nullptr) {}

  ~NotNode() override
  {
  }

  std::string toString()
  {
    std::string ret;
    ret = "(NOT ";
    ret += op->toString();
    ret += ")";

    return ret;
  }

  void getPredicates(std::vector<Predicate> & predicates)
  {
    op->getPredicates(predicates);
  }
  std::shared_ptr<TreeNode> op;
};

class PredicateTree
{
public:
  PredicateTree()
  : root_(nullptr) {}

  ~PredicateTree()
  {
  }

  void clear()
  {
    root_ = nullptr;
  }

  PredicateTree & operator=(const PredicateTree & other)
  {
    root_ = other.root_;
    return *this;
  }

  std::string toString() const
  {
    if (root_ != nullptr) {
      return root_->toString();
    } else {
      return "";
    }
  }

  void fromString(const std::string & expr)
  {
    if (expr == "") {
      root_ = nullptr;
    } else {
      root_ = get_tree_node(expr);
    }
  }

  void getPredicates(std::vector<Predicate> & predicates)
  {
    if (root_ != nullptr) {
      root_->getPredicates(predicates);
    }
  }

  bool empty()
  {
    return root_ == nullptr || root_->toString() == "(AND )";
  }

  std::shared_ptr<TreeNode> root_;
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
  PredicateTree at_start_effects;
  PredicateTree at_end_effects;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__TYPES_HPP_
