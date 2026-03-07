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

#ifndef PLANSYS2_CORE__TYPES_HPP_
#define PLANSYS2_CORE__TYPES_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace std
{
template<typename T>
inline void hash_combine(std::size_t & seed, const T & value)
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

inline std::size_t hash_node(const plansys2_msgs::msg::Node & node)
{
  std::size_t seed = 0;

  hash_combine(seed, node.name);
  hash_combine(seed, node.node_type);
  hash_combine(seed, node.children.size());
  hash_combine(seed, node.parameters.size());

  for (const auto & param : node.parameters) {
    hash_combine(seed, param.name);
  }
  return seed;
}

template<typename UnorderedContainer>
inline std::size_t unordered_container_hash(const UnorderedContainer & container)
{
  std::size_t h = 0;
  for (const auto & elem : container) {
    // Combine using XOR for order independence
    h ^= std::hash<std::decay_t<decltype(elem)>>{}(elem);
  }
  // Optionally, mix in the size for more uniqueness
  h ^= container.size();
  return h;
}

// For unordered_map and similar
template<typename Key, typename T, typename Hash, typename KeyEqual, typename Alloc>
std::size_t unordered_container_hash(const std::unordered_map<Key, T, Hash, KeyEqual, Alloc> & map)
{
  std::size_t h = 0;
  for (const auto & [key, value] : map) {
    std::size_t entry_hash = std::hash<Key>{}(key);
    entry_hash ^= std::hash<T>{}(value) + 0x9e3779b9;
    h ^= entry_hash;
  }
  h ^= map.size();
  return h;
}

}  // namespace std

namespace plansys2
{

/**
 * @brief Utility function to convert a vector of one type to another.
 * @tparam toT The target type.
 * @tparam fromT The source type.
 * @param[in] in_vector The input vector to convert.
 * @return std::vector<toT> The converted vector.
 */
template<class toT, class fromT>
std::vector<toT> convertVector(const std::vector<fromT> & in_vector)
{
  std::vector<toT> ret(in_vector.begin(), in_vector.end());
  return ret;
}

template<class toT, class fromT>
std::unordered_set<toT> convertVectorToUnorderedSet(const std::vector<fromT> & in_vector)
{
  std::unordered_set<toT> ret(in_vector.begin(), in_vector.end());
  return ret;
}

template<class toT, class fromT>
std::vector<toT> convertUnorderedSetToVector(const std::unordered_set<fromT> & in_unordered_set)
{
  std::vector<toT> ret(in_unordered_set.begin(), in_unordered_set.end());
  return ret;
}

/**
 * @class plansys2::Instance
 * @brief Represents an instance (object) in the planning domain.
 *
 * Inherits from plansys2_msgs::msg::Param and provides constructors for easy creation
 * from strings or existing Param messages.
 */
class Instance : public plansys2_msgs::msg::Param
{
public:
  /**
   * @brief Default constructor.
   */
  Instance()
  : plansys2_msgs::msg::Param() {}

  /**
   * @brief Construct an Instance from a name and optional type.
   * @param[in] name The name of the instance.
   * @param[in] type The type of the instance (optional).
   */
  explicit Instance(const std::string & name, const std::string & type = {})
  : plansys2_msgs::msg::Param(parser::pddl::fromStringParam(name, type))
  {
  }

  /**
   * @brief Construct an Instance from an existing Param message.
   * @param[in] instance The Param message.
   */
  Instance(const plansys2_msgs::msg::Param & instance)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Param(instance)
  {
  }

  bool operator==(const Instance & i2) const
  {
    if (this == &i2) {return true;}
    return parser::pddl::checkParamEquality(*this, i2);
  }
};

/**
 * @class plansys2::Predicate
 * @brief Represents a predicate in the planning domain.
 *
 * Inherits from plansys2_msgs::msg::Node and provides constructors for easy creation
 * from strings or existing Node messages.
 */
class Predicate : public plansys2_msgs::msg::Node
{
public:
  /**
   * @brief Default constructor.
   */
  Predicate()
  : plansys2_msgs::msg::Node() {}

  /**
   * @brief Construct a Predicate from a string representation.
   * @param[in] pred The predicate as a string.
   */
  explicit Predicate(const std::string & pred)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringPredicate(pred))
  {
  }

  /**
   * @brief Construct a Predicate from an existing Node message.
   * @param[in] pred The Node message.
   */
  Predicate(const plansys2_msgs::msg::Node & pred)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(pred)
  {
  }

  bool operator==(const Predicate & p2) const
  {
    if (this == &p2) {return true;}
    return parser::pddl::checkNodeEquality(*this, p2);
  }
};

/**
 * @class plansys2::Derived
 * @brief Represents a derived predicate (i.e., axiom) in the planning domain.
 *
 * Inherits from plansys2_msgs::msg::Derived and provides constructors for easy creation
 * from strings or existing Node messages.
 */
class Derived : public plansys2_msgs::msg::Derived
{
public:
  Derived()
  : plansys2_msgs::msg::Derived() {computeNormalizedDerived();}
  Derived(const plansys2_msgs::msg::Derived & derived)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Derived(derived)
  {
    computeNormalizedDerived();
  }

  bool operator==(const Derived & d) const
  {
    if (this == &d) {return true;}
    if (!normalizedDerivedComputed()) {
      computeNormalizedDerived();
    }
    if (!d.normalizedDerivedComputed()) {
      d.computeNormalizedDerived();
    }
    return parser::pddl::checkNodeEquality(getNormalizedPredicate(), d.getNormalizedPredicate()) &&
           parser::pddl::checkTreeEquality(
      getNormalizedPreconditions(), d.getNormalizedPreconditions());
  }

  const size_t & getNormalizedHash() const {return normalized_hash_;}

  const plansys2_msgs::msg::Node & getNormalizedPredicate() const {return normalized_predicate_;}

  const plansys2_msgs::msg::Tree & getNormalizedPreconditions() const
  {
    return normalized_preconditions_;
  }

  Derived getNormalizedDerived()
  {
    if (!normalizedDerivedComputed()) {
      computeNormalizedDerived();
    }
    Derived new_derived;
    new_derived.predicate = normalized_predicate_;
    new_derived.preconditions = normalized_preconditions_;
    return new_derived;
  }

  bool normalizedDerivedComputed() const
  {
    return normalized_predicate_.name != "" && !normalized_preconditions_.nodes.empty();
  }

  std::string normalize_param(
    const std::string & old_name, std::map<std::string, std::string> & var_map, uint & i) const
  {
    if (old_name.empty() || old_name.front() != '?') {return old_name;}

    auto it = var_map.find(old_name);
    if (it != var_map.end()) {return it->second;}

    std::string normalized = "?" + std::to_string(i++);
    var_map[old_name] = normalized;
    return normalized;
  }

  void computeNormalizedDerived() const
  {
    if (normalizedDerivedComputed()) {return;}

    normalized_predicate_ = this->predicate;
    normalized_preconditions_ = this->preconditions;

    std::map<std::string, std::string> var_map;
    uint i = 0;

    // Normalize predicate parameters
    for (auto & p : normalized_predicate_.parameters) {
      p.name = this->normalize_param(p.name, var_map, i);
    }

    // Normalize precondition nodes and their parameters
    for (auto & node : normalized_preconditions_.nodes) {
      if (node.node_type == plansys2_msgs::msg::Node::PARAMETER) {
        node.name = this->normalize_param(node.name, var_map, i);
      }

      for (auto & p : node.parameters) {p.name = this->normalize_param(p.name, var_map, i);}
    }

    normalized_hash_ = 0;
    std::hash_combine(normalized_hash_, std::hash_node(normalized_predicate_));
    std::hash_combine(normalized_hash_, normalized_preconditions_.nodes.size());
    for (auto & node : normalized_preconditions_.nodes) {
      std::hash_combine(normalized_hash_, std::hash_node(node));
    }
  }

  std::string toNormalizedString() const
  {
    if (!normalizedDerivedComputed()) {
      computeNormalizedDerived();
    }

    std::ostringstream oss;

    // Predicate
    oss << "(" << normalized_predicate_.name;
    for (const auto & p : normalized_predicate_.parameters) {
      oss << " " << p.name;
    }
    oss << ")";

    // Preconditions
    if (!normalized_preconditions_.nodes.empty()) {
      oss << " :- ";

      bool first = true;
      for (const auto & node : normalized_preconditions_.nodes) {
        if (!first) {
          oss << " ";
        }
        first = false;

        oss << nodeToString(node);
      }
    }

    return oss.str();
  }

private:
  mutable plansys2_msgs::msg::Node normalized_predicate_;
  mutable plansys2_msgs::msg::Tree normalized_preconditions_;
  mutable bool normalized_cache_;
  mutable std::size_t normalized_hash_;

  static std::string
  nodeToString(const plansys2_msgs::msg::Node & node)
  {
    std::ostringstream oss;

    switch (node.node_type) {
      case plansys2_msgs::msg::Node::AND:
        oss << "(and";
        for (const auto & p : node.parameters) {
          oss << " " << p.name;
        }
        oss << ")";
        break;

      case plansys2_msgs::msg::Node::OR:
        oss << "(or";
        for (const auto & p : node.parameters) {
          oss << " " << p.name;
        }
        oss << ")";
        break;

      case plansys2_msgs::msg::Node::PREDICATE:
        oss << "(" << node.name;
        for (const auto & p : node.parameters) {
          oss << " " << p.name;
        }
        oss << ")";
        break;

      case plansys2_msgs::msg::Node::PARAMETER:
        oss << node.name;
        break;

      default:
        oss << "(unknown)";
        break;
    }

    return oss.str();
  }
};

/**
 * @class plansys2::Function
 * @brief Represents a function in the planning domain.
 *
 * Inherits from plansys2_msgs::msg::Node and provides constructors for easy creation
 * from strings or existing Node messages.
 */
class Function : public plansys2_msgs::msg::Node
{
public:
  /**
   * @brief Default constructor.
   */
  Function()
  : plansys2_msgs::msg::Node() {}

  /**
   * @brief Construct a Function from a string representation.
   * @param[in] func The function as a string.
   */
  explicit Function(const std::string & func)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringFunction(func))
  {
  }

  /**
   * @brief Construct a Function from an existing Node message.
   * @param[in] func The Node message.
   */
  Function(const plansys2_msgs::msg::Node & func)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(func)
  {
  }

  bool operator==(const Function & f2) const
  {
    if (this == &f2) {return true;}
    return parser::pddl::checkNodeEquality(*this, f2);
  }
};

/**
 * @class plansys2::Goal
 * @brief Represents a goal in the planning domain.
 *
 * Inherits from plansys2_msgs::msg::Tree and provides constructors for easy creation
 * from strings or existing Tree messages.
 */
class Goal : public plansys2_msgs::msg::Tree
{
public:
  /**
   * @brief Default constructor.
   */
  Goal()
  : plansys2_msgs::msg::Tree() {}

  /**
   * @brief Construct a Goal from a string representation.
   * @param[in] goal The goal as a string.
   */
  explicit Goal(const std::string & goal)
  : plansys2_msgs::msg::Tree(parser::pddl::fromString(goal))
  {
  }

  /**
   * @brief Construct a Goal from an existing Tree message.
   * @param[in] goal The Tree message.
   */
  Goal(const plansys2_msgs::msg::Tree & goal)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Tree(goal)
  {
  }
};

}  // namespace plansys2

namespace std
{

template<>
struct hash<plansys2_msgs::msg::Node>
{
  std::size_t operator()(const plansys2_msgs::msg::Node & node) const noexcept
  {
    return hash_node(node);
  }
};

template<>
struct hash<plansys2::Instance>
{
  std::size_t operator()(const plansys2::Instance & inst) const noexcept
  {
    return std::hash<std::string>{}(inst.name);
  }
};

template<>
struct hash<plansys2::Predicate>
{
  std::size_t operator()(const plansys2::Predicate & pred) const noexcept
  {
    return hash_node(pred);
  }
};

template<>
struct hash<plansys2::Derived>
{
  std::size_t operator()(const plansys2::Derived & derived) const noexcept
  {
    if (!derived.normalizedDerivedComputed()) {
      derived.computeNormalizedDerived();
    }

    return derived.getNormalizedHash();
  }
};

template<>
struct hash<plansys2::Function>
{
  std::size_t operator()(const plansys2::Function & func) const noexcept {return hash_node(func);}
};
}  // namespace std

#endif  // PLANSYS2_CORE__TYPES_HPP_
