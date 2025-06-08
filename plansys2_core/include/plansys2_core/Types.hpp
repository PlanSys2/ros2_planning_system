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

#include <string>
#include <unordered_set>
#include <variant>
#include <vector>
#include <map>

#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace std 
{
template<typename T>
inline void hash_combine(std::size_t & seed, const T & value)
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
}

namespace plansys2
{

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

class Instance : public plansys2_msgs::msg::Param
{
public:
  Instance()
  : plansys2_msgs::msg::Param() {}
  explicit Instance(const std::string & name, const std::string & type = {})
  : plansys2_msgs::msg::Param(parser::pddl::fromStringParam(name, type))
  {
  }
  Instance(const plansys2_msgs::msg::Param & instance)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Param(instance)
  {
  }

  bool operator==(const Instance & i2) const {return parser::pddl::checkParamEquality(*this, i2);}
};

class Predicate : public plansys2_msgs::msg::Node
{
public:
  Predicate()
  : plansys2_msgs::msg::Node() {}
  explicit Predicate(const std::string & pred)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringPredicate(pred))
  {
  }
  Predicate(const plansys2_msgs::msg::Node & pred)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(pred)
  {
  }

  bool operator==(const Predicate & p2) const {return parser::pddl::checkNodeEquality(*this, p2);}
};

class Derived : public plansys2_msgs::msg::Derived
{
public:
  Derived() : plansys2_msgs::msg::Derived() { computeNormalizedDerived(); }
  Derived(const plansys2_msgs::msg::Derived & derived)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Derived(derived) { computeNormalizedDerived(); }
  
  bool operator==(const Derived & d) const
  { 
    if (!normalizedDerivedComputed()) {
      computeNormalizedDerived();
    }
    if (!d.normalizedDerivedComputed()) {
      d.computeNormalizedDerived();
    }
    return parser::pddl::checkNodeEquality(getNormalizedPredicate(), d.getNormalizedPredicate()) &&
           parser::pddl::checkTreeEquality(getNormalizedPreconditions(), d.getNormalizedPreconditions());
  }

  const size_t& getNormalizedHash() const {return normalized_hash_;}

  const plansys2_msgs::msg::Node& getNormalizedPredicate() const
  { 
    return normalized_predicate_; 
  }
  
  const plansys2_msgs::msg::Tree& getNormalizedPreconditions() const
  { 
    return normalized_preconditions_; 
  }
  
  Derived getNormalizedDerived() 
  {
    if(!normalizedDerivedComputed())
    {
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
    const std::string& old_name, std::map<std::string, std::string>& var_map, uint& i) const
  {
    if (old_name.empty() || old_name.front() != '?')
      return old_name;

    auto it = var_map.find(old_name);
    if (it != var_map.end())
      return it->second;

    std::string normalized = "?" + std::to_string(i++);
    var_map[old_name] = normalized;
    return normalized;
  }

  void computeNormalizedDerived() const
  {
    if(normalizedDerivedComputed()) return;
    
    normalized_predicate_ = this->predicate;
    normalized_preconditions_ = this->preconditions;

    std::map<std::string, std::string> var_map;
    uint i = 0;

    // Normalize predicate parameters
    for (auto& p : normalized_predicate_.parameters)
    {
      p.name = this->normalize_param(p.name, var_map, i);
    }

    // Normalize precondition nodes and their parameters
    for (auto& node : normalized_preconditions_.nodes)
    {
      if (node.node_type == plansys2_msgs::msg::Node::PARAMETER)
        node.name = this->normalize_param(node.name, var_map, i);

      for (auto& p : node.parameters)
        p.name = this->normalize_param(p.name, var_map, i);
    }

    normalized_hash_ = 0;
    std::hash_combine(normalized_hash_, normalized_predicate_);
    for (auto &node : normalized_preconditions_.nodes)
      std::hash_combine(normalized_hash_, node);
    
  }

private:
  mutable plansys2_msgs::msg::Node normalized_predicate_;
  mutable plansys2_msgs::msg::Tree normalized_preconditions_;
  mutable bool normalized_cache_;
  mutable std::size_t normalized_hash_;
};

class Function : public plansys2_msgs::msg::Node
{
public:
  Function()
  : plansys2_msgs::msg::Node() {}
  explicit Function(const std::string & func)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringFunction(func))
  {
  }
  Function(const plansys2_msgs::msg::Node & func)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(func)
  {
  }

  bool operator==(const Function & f2) const {return parser::pddl::checkNodeEquality(*this, f2);}
};

class Goal : public plansys2_msgs::msg::Tree
{
public:
  Goal()
  : plansys2_msgs::msg::Tree() {}
  explicit Goal(const std::string & goal)
  : plansys2_msgs::msg::Tree(parser::pddl::fromString(goal))
  {
  }
  Goal(const plansys2_msgs::msg::Tree & goal)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Tree(goal)
  {
  }
};

}  // namespace plansys2

namespace std
{

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
    std::size_t seed = 0;
    
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
