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

#include "plansys2_executor/BTBuilder.hpp"

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

void
BTBuilder::set_clients(
  std::shared_ptr<plansys2::DomainExpertClient> domain_client,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  domain_client_ = domain_client;
  problem_client_ = problem_client;
}

void
BTBuilder::set_initial_state(
  std::vector<plansys2::Predicate> predicates, std::vector<plansys2::Function> functions)
{
  initial_state_ = std::make_pair(std::move(predicates), std::move(functions));
}

std::vector<plansys2::Predicate>
BTBuilder::initial_predicates() const
{
  return initial_state_ ? initial_state_->first : problem_client()->getPredicates();
}

std::vector<plansys2::Function>
BTBuilder::initial_functions() const
{
  return initial_state_ ? initial_state_->second : problem_client()->getFunctions();
}

void
BTBuilder::apply_effects(
  const plansys2_msgs::msg::Tree & effects, std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  apply(effects, predicates, functions);

  // Read once per plan; a domain without derived predicates pays nothing more
  if (!derived_rules_) {
    std::vector<plansys2::Derived> derived;
    std::set<std::string> names;
    for (const auto & d : domain_client()->getDerivedPredicates()) {
      if (names.insert(d.predicate.name).second) {
        for (const auto & rule : domain_client()->getDerivedPredicate(d.predicate.name)) {
          derived.push_back(rule);
        }
      }
    }
    derived_rules_ = std::make_shared<const std::vector<plansys2::DerivedRule>>(
      derived.empty() ? std::vector<plansys2::DerivedRule>{} :
      make_derived_rules(derived, problem_client()->getInstances()));
  }
  if (!derived_rules_->empty()) {
    predicates = derive(*derived_rules_, predicates, functions);
  }
}

std::shared_ptr<plansys2::DomainExpertClient>
BTBuilder::domain_client() const
{
  if (!domain_client_) {
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  }
  return domain_client_;
}

std::shared_ptr<plansys2::ProblemExpertClient>
BTBuilder::problem_client() const
{
  if (!problem_client_) {
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  }
  return problem_client_;
}

}  // namespace plansys2
