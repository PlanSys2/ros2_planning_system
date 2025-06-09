// Copyright 2024 Intelligent Robotics Lab
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

#ifndef PLANSYS2_CORE__STATE_HPP_
#define PLANSYS2_CORE__STATE_HPP_

#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "plansys2_core/DerivedResolutionGraph.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/state.hpp"

namespace plansys2
{

class State
{
public:
  State() {}

  State(
    const std::unordered_set<plansys2::Instance> & instances,
    std::unordered_set<plansys2::Function> & functions,
    const std::unordered_set<plansys2::Predicate> & predicates)
  : instances_(instances),
    functions_(functions),
    predicates_(predicates),
    union_predicates_inferred_predicates_(predicates)
  {
  }
  
  State(
    const std::unordered_set<plansys2::Instance> & instances,
    std::unordered_set<plansys2::Function> & functions,
    const std::unordered_set<plansys2::Predicate> & predicates,
    const plansys2::DerivedResolutionGraph & derived_predicates)
  : instances_(instances),
    functions_(functions),
    predicates_(predicates),
    union_predicates_inferred_predicates_(predicates),
    derived_predicates_(derived_predicates)
  {
  }

  State(
    const std::unordered_set<plansys2::Instance> & instances,
    const std::unordered_set<plansys2::Function> & functions,
    const std::unordered_set<plansys2::Predicate> & predicates,
    const std::vector<std::tuple<plansys2::Derived, plansys2::Predicate>> & inferred_predicates);

  State(
    const std::unordered_set<plansys2::Instance> & instances,
    const std::unordered_set<plansys2::Function> & functions,
    const std::unordered_set<plansys2::Predicate> & predicates,
    const std::vector<std::tuple<plansys2::Derived, plansys2::Predicate>> & inferred_predicates,
    const plansys2::DerivedResolutionGraph & derived_predicates);

  State(const plansys2_msgs::msg::State & state);  //NOLINT

  bool operator==(const State & state) const;

  auto & getInstances() const {return instances_;}
  auto & getFunctions() const {return functions_;}
  auto & getPredicates() const {return predicates_;}
  auto & getInferredPredicates() const { return inferred_predicates_;}
  auto & getUnionPredicatesInferredPredicates() const {return union_predicates_inferred_predicates_;}
  auto & getDerivedPredicates() const {return derived_predicates_;}
  
  /**
   * @brief Retrieves the strongly connected components (SCCs) of derived predicates.
   *
   * This method computes the SCCs of the derived predicates using Tarjan's algorithm,
   * reverses the order of the resulting SCCs, and returns them as a vector of vectors
   * in topological order.
   *
   * @return A vector of vectors, where each inner vector contains derived predicates
   *         that form a strongly connected component.
   */
  std::vector<std::vector<Derived>> getDerivedPredicatesSCCs() const;

  /**
   * @brief Computes the strongly connected components (SCCs) of derived predicates.
   *
   * This function returns the SCCs of the derived predicates graph using Tarjan's algorithm.
   * If no root nodes are provided, it computes the SCCs for the entire derived predicates graph.
   * If a set of root nodes is provided, it computes the SCCs for the subgraph induced by those nodes.
   * The resulting SCCs are returned in reverse topological order.
   *
   * @param root_nodes A vector of root nodes to define the subgraph for SCC computation.
   *                   If empty, the entire graph is considered.
   * @return std::vector<std::vector<Derived>> A vector of SCCs, each represented as a vector of Derived predicates.
   */
  std::vector<std::vector<Derived>> getDerivedPredicatesSCCs(
    const std::vector<plansys2_msgs::msg::Node>& root_nodes) const;

  /**
   * @brief Retrieves all derived predicates in the state using a depth-first traversal.
   *
   * This method returns a vector containing all derived predicates found in the current state.
   * The predicates are collected by traversing the state in a depth-first manner, which may be
   * useful for scenarios where the order of discovery or dependency resolution is important.
   *
   * @return std::vector<plansys2::Derived> A vector of derived predicates present in the state.
   */
  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst() const;
  
  /**
   * @brief Retrieves all children derived predicates from root_nodes using a depth-first traversal.
   *
   * This method traverses the given root nodes in a depth-first manner to collect
   * all derived predicates present in the state. Derived predicates are computed
   * based on the current state and the logical structure of the provided nodes.
   *
   * @param root_nodes A vector of root nodes from which to start the depth-first traversal.
   * @return A vector containing all derived predicates found during the traversal.
   */
  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst(
    const std::vector<plansys2_msgs::msg::Node>& root_nodes) const;

  auto getInstancesSize() const {return instances_.size();}
  auto getFunctionsSize() const {return functions_.size();}
  auto getPredicatesSize() const {return predicates_.size();}
  auto getInferredPredicatesSize() const {return inferred_predicates_.size();}
  auto getUnionPredicatesSize() const {return union_predicates_inferred_predicates_.size();}
  auto getDerivedPredicatesSize() const {return derived_predicates_.getEdgeNumber();}

  size_t getInferredPredicateRefCount(const plansys2::Predicate & predicate) const {return inferred_predicate_refcount_.at(predicate);}

  size_t getNumberInferredFromDerived(const plansys2::Derived & derived) const;

  bool addInstance(const plansys2::Instance & instance)
  {
    return instances_.insert(instance).second;
  }
  bool addInstance(plansys2::Instance && instance)
  {
    return instances_.insert(std::move(instance)).second;
  }

  bool addPredicate(const plansys2::Predicate & predicate)
  {
    return predicates_.insert(predicate).second && union_predicates_inferred_predicates_.insert(predicate).second;
  }
  bool addPredicate(plansys2::Predicate && predicate)
  {
    return predicates_.insert(predicate).second && union_predicates_inferred_predicates_.insert(predicate).second;
  }

  bool addInferredPredicate(const plansys2::Derived & derived, const plansys2::Predicate & predicate);
  bool addInferredPredicate(const plansys2::Derived & derived, plansys2::Predicate && predicate);

  bool addFunction(const plansys2::Function & function)
  {
    return functions_.insert(function).second;
  }
  bool addFunction(plansys2::Function && function)
  {
    return functions_.insert(std::move(function)).second;
  }

  void removeInstance(const std::unordered_set<plansys2::Instance>::const_iterator it)
  {
    instances_.erase(it);
  }
  bool removeInstance(plansys2::Instance instance) {return instances_.erase(instance) == 1;}

  auto removePredicate(const std::unordered_set<plansys2::Predicate>::const_iterator it)
  {
    union_predicates_inferred_predicates_.erase(*it);
    return predicates_.erase(it);
  }
  bool removePredicate(const plansys2::Predicate & predicate)
  {
    return predicates_.erase(predicate) == 1 && union_predicates_inferred_predicates_.erase(predicate) == 1;
  }
  bool removePredicate(plansys2::Predicate && predicate)
  {
    return predicates_.erase(predicate) == 1 && union_predicates_inferred_predicates_.erase(predicate) == 1;
  }

  auto removeFunction(const std::unordered_set<plansys2::Function>::const_iterator it)
  {
    return functions_.erase(it);
  }
  bool removeFunction(const plansys2::Function & function)
  {
    return functions_.erase(function) == 1;
  }
  bool removeFunction(plansys2::Function && function)
  {
    return functions_.erase(std::move(function)) == 1;
  }

  bool hasInstance(const plansys2::Instance & instance)
  {
    return instances_.find(instance) != instances_.end();
  }
  bool hasInstance(plansys2::Instance && instance)
  {
    return instances_.find(std::move(instance)) != instances_.end();
  }

  bool hasPredicate(const std::string & predicate_str)
  {
    auto predicate = parser::pddl::fromStringPredicate(predicate_str);
    return predicates_.find(predicate) != predicates_.end();
  }
  bool hasPredicate(const plansys2::Predicate & predicate)
  {
    return predicates_.find(predicate) != predicates_.end();
  }
  bool hasPredicate(plansys2::Predicate && predicate)
  {
    return predicates_.find(std::move(predicate)) != predicates_.end();
  }

  bool hasInferredPredicate(const std::string & predicate_str)
  {
    auto predicate = parser::pddl::fromStringPredicate(predicate_str);
    return inferred_predicates_.find(predicate) != inferred_predicates_.end();
  }
  bool hasInferredPredicate(const plansys2::Predicate & predicate)
  {
    return inferred_predicates_.find(predicate) != inferred_predicates_.end();
  }
  bool hasInferredPredicate(plansys2::Predicate && predicate)
  {
    return inferred_predicates_.find(std::move(predicate)) != inferred_predicates_.end();
  }

  bool hasFunction(const plansys2::Function & function)
  {
    return functions_.find(function) != functions_.end();
  }
  bool hasFunction(plansys2::Function && function)
  {
    return functions_.find(std::move(function)) != functions_.end();
  }

  auto getInstance(const plansys2::Instance & instance) const {return instances_.find(instance);}
  auto getInstance(plansys2::Instance && instance) const
  {
    return instances_.find(std::move(instance));
  }

  auto getPredicate(const plansys2::Predicate & predicate) const
  {
    return predicates_.find(predicate);
  }
  auto getPredicate(plansys2::Predicate && predicate) const
  {
    return predicates_.find(std::move(predicate));
  }

  auto getFunction(const plansys2::Function & function) const {return functions_.find(function);}
  auto getFunction(plansys2::Function && function) const
  {
    return functions_.find(std::move(function));
  }

  void setDerivedPredicates(const plansys2::DerivedResolutionGraph derived_predicates)
  {
    derived_predicates_ = derived_predicates;
  }

  bool hasDerivedPredicates() {return derived_predicates_.getEdgeNumber() == 0;}

  void reserveInferredPredicates(size_t size)
  {
    inferred_predicates_.reserve(size);
    union_predicates_inferred_predicates_.reserve(predicates_.size() + size);
  }

  void clearPredicates();
  void clearState();
  void resetInferredPredicates();
  void initInferredPredicates();

  bool ungroundSingleDerivedPredicate(const plansys2::Derived & derived);
  std::unordered_set<plansys2::Derived> ungroundDerivedPredicate(const plansys2::Derived & derived);

  plansys2_msgs::msg::State getAsMsg();

  void addActionsAndPruneDerived(const std::vector<plansys2::ActionVariant> & actions);
  void pruneDerivedPredicatesToActions(const std::vector<plansys2::ActionVariant> & actions);

private:
  std::unordered_set<plansys2::Instance> instances_;
  std::unordered_set<plansys2::Function> functions_;
  std::unordered_set<plansys2::Predicate> predicates_;
  std::unordered_set<plansys2::Predicate> inferred_predicates_;
  std::unordered_set<plansys2::Predicate> union_predicates_inferred_predicates_;
  
  std::unordered_map<plansys2::Derived, std::unordered_set<plansys2::Predicate>> inferred_predicates_map_;
  std::unordered_map<plansys2::Predicate, size_t> inferred_predicate_refcount_;
  
  plansys2::DerivedResolutionGraph derived_predicates_;

  void removeInferredPredicate(const plansys2::Predicate & predicate);

  friend struct std::hash<State>;
};
}  // namespace plansys2

namespace std
{
template<>
struct hash<plansys2::State>
{
  std::size_t operator()(const plansys2::State & state) const
  {
    std::size_t seed = 0;
    seed ^= unordered_container_hash(state.instances_);
    seed ^= unordered_container_hash(state.functions_);
    seed ^= unordered_container_hash(state.predicates_);
    seed ^= unordered_container_hash(state.inferred_predicates_);
    seed ^= unordered_container_hash(state.union_predicates_inferred_predicates_);
    seed ^= std::hash<decltype(state.derived_predicates_)>{}(state.derived_predicates_);
    return seed;
  }
};
}  // namespace std

#endif  // PLANSYS2_CORE__STATE_HPP_
