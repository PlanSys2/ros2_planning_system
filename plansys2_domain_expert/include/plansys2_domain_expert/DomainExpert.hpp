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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "pddl_parser/Domain.h"

#include "plansys2_domain_expert/DomainExpertInterface.hpp"
#include "plansys2_domain_expert/Types.hpp"

namespace plansys2
{

class DomainExpert : public DomainExpertInterface
{
public:
  explicit DomainExpert(const std::string & domain);

  std::vector<std::string> getTypes();
  std::vector<std::string> getPredicates();
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  std::vector<std::string> getActions();
  std::optional<plansys2::Action> getAction(const std::string & action);

  std::vector<std::string> getDurativeActions();
  std::optional<plansys2::DurativeAction> getDurativeAction(const std::string & action);

  std::string getDomain();

private:
  parser::pddl::Domain domain_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
