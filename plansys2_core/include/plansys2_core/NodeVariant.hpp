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

#ifndef PLANSYS2_CORE__NODEVARIANT_HPP_
#define PLANSYS2_CORE__NODEVARIANT_HPP_

#include <memory>
#include <string>

#include "plansys2_core/Action.hpp"
#include "plansys2_core/Types.hpp"

namespace plansys2
{

/**
 * @class NodeVariant
 * @brief A wrapper class for representing different types of nodes in the planning system.
 *
 * NodeVariant encapsulates a variant type that can hold one of several node types used in the planning system,
 * including Predicate, Function, Derived, and ActionVariant. It provides utility methods for type checking,
 * accessing the underlying node, and retrieving node-specific information such as name and type.
 *
 * The class supports hashing, equality comparison, and printing node information for debugging purposes.
 *
 * This class uses a custom hash function and '==' operators designed specifically for enabling the
 * resolution of derived predicates in the correct order, which might not be suitable for other uses.
 * For example (check the unit tests for more examples):
 *  - (predA ?a) == (predA ?x)
 *
 * @note The underlying node is stored as a shared pointer to allow for efficient copying and management.
 *
 * @author
 * @date
 */
class NodeVariant
{
public:
  using NodeType = std::variant<
    plansys2::Predicate, plansys2::Function, plansys2::Derived, plansys2::ActionVariant>;

  template<typename NodeT>
  NodeVariant(NodeT node)  // NOLINT(runtime/explicit)
  : node_(std::make_shared<NodeType>(node))
  {
  }

  size_t hash() const
  {
    return std::visit(
      [](auto && arg) {return std::hash<std::decay_t<decltype(arg)>>{}(arg);}, *node_);
  }

  bool operator==(const NodeVariant & other) const
  {
    if (this == &other) {
      return true;  // Same instance
    }
    // Check type first
    if (this->getNodeType() != other.getNodeType()) {return false;}

    // Predicate case
    if (this->isPredicate()) {
      return parser::pddl::checkNodeEquality(
        this->getPredicateNode(), other.getPredicateNode(), false);
    }

    // Function case
    if (this->isFunction()) {
      return parser::pddl::checkNodeEquality(
        this->getFunctionNode(), other.getFunctionNode(), false);
    }

    return *node_ == *other.node_;
  }

  bool operator!=(const NodeVariant & other) const {return !(*this == other);}

  const NodeType & getNode() const {return *node_;}

  std::string getNodeName() const
  {
    std::string node_name;
    if (std::holds_alternative<plansys2::Predicate>(*node_)) {
      node_name = std::get<plansys2::Predicate>(*node_).name;
    } else if (std::holds_alternative<plansys2::Function>(*node_)) {
      node_name = std::get<plansys2::Function>(*node_).name;
    } else if (std::holds_alternative<plansys2::Derived>(*node_)) {
      node_name = std::get<plansys2::Derived>(*node_).predicate.name;
    } else if (std::holds_alternative<plansys2::ActionVariant>(*node_)) {
      node_name = std::get<plansys2::ActionVariant>(*node_).get_action_name();
    }

    return node_name;
  }

  bool isPredicate() const {return std::holds_alternative<plansys2::Predicate>(*node_);}

  bool isFunction() const {return std::holds_alternative<plansys2::Function>(*node_);}

  bool isDerived() const {return std::holds_alternative<plansys2::Derived>(*node_);}

  bool isAction() const
  {
    return std::holds_alternative<plansys2::ActionVariant>(*node_) &&
           std::get<plansys2::ActionVariant>(*node_).is_action();
  }

  bool isDurativeAction() const
  {
    return std::holds_alternative<plansys2::ActionVariant>(*node_) &&
           std::get<plansys2::ActionVariant>(*node_).is_durative_action();
  }

  bool isActionVariant() const {return std::holds_alternative<plansys2::ActionVariant>(*node_);}

  plansys2::Function & getFunctionNode() const {return std::get<plansys2::Function>(*node_);}
  plansys2::Predicate getPredicateNode() const {return std::get<plansys2::Predicate>(*node_);}
  plansys2::Derived getDerivedNode() const {return std::get<plansys2::Derived>(*node_);}
  plansys2::ActionVariant getActionVariantNode() const
  {
    return std::get<plansys2::ActionVariant>(*node_);
  }

  auto & getDerivedPreconditions() const
  {
    return std::get<plansys2::Derived>(*node_).preconditions;
  }

  auto & getDerivedPredicate() const {return std::get<plansys2::Derived>(*node_).predicate;}

  std::string getNodeType() const
  {
    if (std::holds_alternative<plansys2::Predicate>(*node_)) {
      return "predicate";
    } else if (std::holds_alternative<plansys2::Function>(*node_)) {
      return "function";
    } else if (std::holds_alternative<plansys2::Derived>(*node_)) {
      return "derived";
    } else if (std::holds_alternative<plansys2::ActionVariant>(*node_)) {
      return std::get<plansys2::ActionVariant>(*node_).is_action() ? "action" : "durative action";
    }
    return "";
  }

  void printNode() const
  {
    if (isDerived()) {
      std::cout << "  Derived Predicate: " << getDerivedPredicate().name;
      for (const auto & param : getDerivedPredicate().parameters) {
        std::cout << " " << param.name;
      }
      std::cout << "\n";
    }
    if (isPredicate()) {
      std::cout << "  Predicate: " << getPredicateNode().name;
      for (const auto & param : getPredicateNode().parameters) {
        std::cout << " " << param.name;
      }
      std::cout << "\n";
    }
    if (isFunction()) {
      std::cout << "  Function: " << getFunctionNode().name;
      for (const auto & param : getFunctionNode().parameters) {
        std::cout << " " << param.name;
      }
      std::cout << "\n";
    }
    if (isAction()) {
      std::cout << "  Action: " << getNodeName() << "\n";
    }
    if (isDurativeAction()) {
      std::cout << "  Durative Action: " << getNodeName() << "\n";
    }
  }

private:
  std::shared_ptr<NodeType> node_;

  friend struct std::hash<NodeVariant>;
};

inline plansys2::NodeVariant nodeMsgToVariant(const plansys2_msgs::msg::Node & node_msg)
{
  if (node_msg.node_type == plansys2_msgs::msg::Node::PREDICATE) {
    return plansys2::Predicate(node_msg);
  } else if (node_msg.node_type == plansys2_msgs::msg::Node::FUNCTION) {
    return plansys2::Function(node_msg);
  } else {
    throw std::runtime_error("Unsupported node_type for NodeVariant conversion");
  }
}

inline bool operator==(const NodeVariant & lhs, const plansys2_msgs::msg::Node & rhs)
{
  if (lhs.isPredicate()) {
    return lhs.getPredicateNode() == static_cast<plansys2::Predicate>(rhs);
  } else if (lhs.isFunction()) {
    return lhs.getFunctionNode() == static_cast<plansys2::Function>(rhs);
  }
  return false;
}

inline std::ostream & operator<<(std::ostream & os, const NodeVariant & node)
{
  os << node.getNodeType() << ":" << node.getNodeName();

  // Print parameters, if available
  if (node.isPredicate()) {
    for (const auto & param : node.getPredicateNode().parameters) {
      os << " " << param.name;
    }
  } else if (node.isFunction()) {
    for (const auto & param : node.getFunctionNode().parameters) {
      os << " " << param.name;
    }
  } else if (node.isDerived()) {
    const auto & pred = node.getDerivedPredicate();
    for (const auto & param : pred.parameters) {
      os << " " << param.name;
    }
  } else if (node.isActionVariant()) {
    const auto & action = node.getActionVariantNode();
    for (const auto & param : action.get_action_params()) {
      os << " " << param.name;
    }
  }
  return os;
}

}  // namespace plansys2

namespace std
{

inline std::size_t hash_node_variant(const plansys2_msgs::msg::Node & node)
{
  std::size_t seed = 0;

  hash_combine(seed, node.name);
  hash_combine(seed, node.node_type);
  hash_combine(seed, node.parameters.size());

  return seed;
}

template<>
struct hash<plansys2::NodeVariant>
{
  std::size_t operator()(const plansys2::NodeVariant & nv) const noexcept
  {
    if (nv.isPredicate()) {
      return hash_node_variant(nv.getPredicateNode());
    } else if (nv.isFunction()) {
      return hash_node_variant(nv.getFunctionNode());
    }
    return nv.hash();
  }
};
}  // namespace std

#endif  // PLANSYS2_CORE__NODEVARIANT_HPP_
