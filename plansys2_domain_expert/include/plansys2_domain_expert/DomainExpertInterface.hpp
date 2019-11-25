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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_

#include <string>
#include <vector>

#include "plansys2_domain_expert/Types.hpp"

namespace plansys2
{

class DomainExpertInterface
{
public:
  DomainExpertInterface() {}

  virtual std::vector<std::string> getTypes() = 0;
  virtual std::vector<std::string> getPredicates() = 0;
  virtual std::optional<plansys2::Predicate> getPredicate(const std::string & predicate)
  = 0;
  virtual std::vector<std::string> getActions() = 0;
  virtual std::optional<plansys2::Action> getAction(const std::string & action) = 0;

  virtual std::vector<std::string> getDurativeActions() = 0;
  virtual std::optional<plansys2::DurativeAction> getDurativeAction(const std::string & action) = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_
