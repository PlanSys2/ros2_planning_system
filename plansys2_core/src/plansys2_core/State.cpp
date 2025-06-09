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

#include "plansys2_core/State.hpp"

namespace plansys2
{
State::State(
  const std::unordered_set<plansys2::Instance> & instances,
  const std::unordered_set<plansys2::Function> & functions,
  const std::unordered_set<plansys2::Predicate> & predicates,
  const std::vector<std::tuple<plansys2::Derived, plansys2::Predicate>> & inferred_predicates)
: instances_(instances),
  functions_(functions),
  predicates_(predicates),
  union_predicates_inferred_predicates_(predicates)
{
  for (const auto& [derived, predicate] : inferred_predicates)
  {
    addInferredPredicate(derived, predicate);
  }
}

State::State(
  const std::unordered_set<plansys2::Instance> & instances,
  const std::unordered_set<plansys2::Function> & functions,
  const std::unordered_set<plansys2::Predicate> & predicates,
  const std::vector<std::tuple<plansys2::Derived, plansys2::Predicate>> & inferred_predicates,
  const plansys2::DerivedResolutionGraph & derived_predicates)
: instances_(instances),
  functions_(functions),
  predicates_(predicates),
  union_predicates_inferred_predicates_(predicates),
  derived_predicates_(derived_predicates)
{
  for (const auto& [derived, predicate] : inferred_predicates)
  {
    addInferredPredicate(derived, predicate);
  }
}

State::State(const plansys2_msgs::msg::State & state)
{
  instances_ = plansys2::convertVectorToUnorderedSet<plansys2::Instance, plansys2_msgs::msg::Param>(
    state.instances);
  functions_ = plansys2::convertVectorToUnorderedSet<plansys2::Function, plansys2_msgs::msg::Node>(
    state.functions);
  predicates_ =
    plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
    state.predicates);
  inferred_predicates_ =
    plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
    state.inferred_predicates);
  union_predicates_inferred_predicates_.insert(predicates_.begin(), predicates_.end());
  union_predicates_inferred_predicates_.insert(inferred_predicates_.begin(), inferred_predicates_.end());
  derived_predicates_ = state.derived_predicates;
}

bool State::operator==(const State & state) const
{
  if (this == &state) return true;
  if (this->instances_.size() != state.instances_.size()) return false;
  if (this->functions_.size() != state.functions_.size()) return false;
  if (this->predicates_.size() != state.predicates_.size()) return false;
  if (this->inferred_predicates_.size() != state.inferred_predicates_.size()) return false;
  if (this->union_predicates_inferred_predicates_.size() != state.union_predicates_inferred_predicates_.size()) return false;
  if (this->derived_predicates_.getNodeNumber() != state.derived_predicates_.getNodeNumber()) return false;
  return this->instances_ == state.instances_ && this->functions_ == state.functions_ &&
         this->predicates_ == state.predicates_ &&
         this->inferred_predicates_ == state.inferred_predicates_ &&
         this->union_predicates_inferred_predicates_ == state.union_predicates_inferred_predicates_ &&
         this->derived_predicates_ == state.derived_predicates_;
}

// std::unordered_set<plansys2::Predicate> State::getUnionPredicatesInferredPredicates() const
// {
//   std::unordered_set<plansys2::Predicate> result = predicates_;
//   result.insert(inferred_predicates_.begin(), inferred_predicates_.end());
//   return result;
// }

std::vector<std::vector<Derived>> State::getDerivedPredicatesSCCs() const
{
  auto sccs = derived_predicates_.computeSCCsTarjanDerivedPredicates();
  std::reverse(sccs.begin(), sccs.end());
  return sccs;
}

std::vector<std::vector<Derived>> State::getDerivedPredicatesSCCs(
  const std::vector<plansys2_msgs::msg::Node>& root_nodes) const
{
  if(root_nodes.empty())
  {
    auto sccs = derived_predicates_.computeSCCsTarjanDerivedPredicates();
    std::reverse(sccs.begin(), sccs.end());
    return sccs;
  }

  std::vector<plansys2::NodeVariant> root_nodes_variant;
  root_nodes_variant.reserve(root_nodes.size());
  for (const auto& n : root_nodes) {
    root_nodes_variant.push_back(nodeMsgToVariant(n));
  }
  
  auto sub_graph = derived_predicates_.getSubGraphFromNodes(root_nodes_variant);
  auto sccs = sub_graph.computeSCCsTarjanDerivedPredicates();
  std::reverse(sccs.begin(), sccs.end());
  return sccs;
}

std::vector<plansys2::Derived> State::getDerivedPredicatesDepthFirst() const
{
  std::vector<plansys2_msgs::msg::Node> root_nodes;
  return derived_predicates_.getDerivedPredicatesDepthFirst();
}
  
std::vector<plansys2::Derived> State::getDerivedPredicatesDepthFirst(
  const std::vector<plansys2_msgs::msg::Node>& root_nodes) const
{
  std::vector<plansys2::NodeVariant> root_nodes_variant;
  for (const auto& n : root_nodes) {
    root_nodes_variant.push_back(nodeMsgToVariant(n));
  }
  return derived_predicates_.getDerivedPredicatesDepthFirst(root_nodes_variant);
}

size_t State::getNumberInferredFromDerived(const plansys2::Derived & derived) const
{
  auto it = inferred_predicates_map_.find(derived);
  if (it == inferred_predicates_map_.end()) {
    return 0;
  }
  return it->second.size();
}

bool State::addInferredPredicate(const plansys2::Derived & derived, const plansys2::Predicate & predicate)
{
  auto res = inferred_predicates_.emplace(predicate);
  auto pred_it = res.first;

  union_predicates_inferred_predicates_.emplace(*pred_it);
  auto insert_result = inferred_predicates_map_[derived].insert(*pred_it);
  if (insert_result.second) {
    inferred_predicate_refcount_[*pred_it] += 1;
  }
  return res.second;
}

bool State::addInferredPredicate(const plansys2::Derived & derived, plansys2::Predicate && predicate)
{
  auto res = inferred_predicates_.emplace(std::move(predicate));
  auto pred_it = res.first;

  union_predicates_inferred_predicates_.emplace(*pred_it);
  auto insert_result = inferred_predicates_map_[derived].insert(*pred_it);
  if (insert_result.second) {
    inferred_predicate_refcount_[*pred_it] += 1;
  }
  return res.second;
}

void State::clearPredicates()
{
  predicates_.clear();
  inferred_predicates_.clear();
  union_predicates_inferred_predicates_.clear();
  inferred_predicates_map_.clear();
  inferred_predicate_refcount_.clear();
}

void State::clearState()
{
  instances_.clear();
  functions_.clear();
  predicates_.clear();
  inferred_predicates_.clear();
  union_predicates_inferred_predicates_.clear();
  derived_predicates_.clear();
  inferred_predicates_map_.clear();
  inferred_predicate_refcount_.clear();
}

void State::resetInferredPredicates()
{
  union_predicates_inferred_predicates_ = predicates_;
  inferred_predicates_.clear();
  inferred_predicates_map_.clear();
  inferred_predicate_refcount_.clear();
}

void State::initInferredPredicates()
{
  union_predicates_inferred_predicates_ = predicates_;
  inferred_predicates_.clear();
  inferred_predicates_map_.clear();
  inferred_predicate_refcount_.clear();
}

void State::removeInferredPredicate(const plansys2::Predicate & predicate)
{
  inferred_predicates_.erase(predicate);
  union_predicates_inferred_predicates_.erase(predicate);
  inferred_predicate_refcount_.erase(predicate);
}

bool State::ungroundSingleDerivedPredicate(const plansys2::Derived& derived)
{
  bool removed = false;
  auto it = inferred_predicates_map_.find(derived);
  if (it == inferred_predicates_map_.end()) {
    return false;
  }

  for (const auto& predicate : it->second) {
    auto rc_it = inferred_predicate_refcount_.find(predicate);
    if (rc_it != inferred_predicate_refcount_.end()) {
      if (--(rc_it->second) == 0) {
        removeInferredPredicate(predicate); // Remove only when last reference is gone
        removed = true;
      }
    }
  }
  inferred_predicates_map_.erase(it);
  return removed;
}

std::unordered_set<plansys2::Derived> State::ungroundDerivedPredicate(const plansys2::Derived & derived)
{
  std::unordered_set<plansys2::Derived> derived_removed;
  derived_predicates_.depthFirstTraverse(
    derived, [&](const plansys2::NodeVariant & node) {
      if (node.isDerived()) {
        const auto &d = node.getDerivedNode();
        if(ungroundSingleDerivedPredicate(d)) {
          derived_removed.emplace(std::move(d));
        }
      }
    });
  return derived_removed;
}

plansys2_msgs::msg::State State::getAsMsg()
{
  plansys2_msgs::msg::State state;
  state.instances =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Param, plansys2::Instance>(
    instances_);
  state.functions =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Function>(functions_);
  state.predicates =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
    predicates_);
  state.inferred_predicates =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
    inferred_predicates_);
  state.derived_predicates =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Derived, plansys2::Derived>(
    derived_predicates_.getDerivedPredicates());
  return state;
}

void State::addActionsAndPruneDerived(const std::vector<plansys2::ActionVariant> & actions)
{
  derived_predicates_.appendActions(actions);
  derived_predicates_ = derived_predicates_.pruneGraphToActions(actions);
}

void State::pruneDerivedPredicatesToActions(const std::vector<plansys2::ActionVariant> & actions)
{
  derived_predicates_ = derived_predicates_.pruneGraphToActions(actions);
}

}  // namespace plansys2
