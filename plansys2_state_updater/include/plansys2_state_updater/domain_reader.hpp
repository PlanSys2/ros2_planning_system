// Copyright 2026 Juan Carlos Manzanares Serrano
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

#ifndef PLANSYS2_STATE_UPDATER__DOMAIN_READER_HPP_
#define PLANSYS2_STATE_UPDATER__DOMAIN_READER_HPP_

#include <optional>
#include <set>
#include <string>

#include "plansys2_domain_expert/DomainExpertInterface.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"

#include "plansys2_state_updater/domain_check.hpp"

namespace plansys2_state_updater
{

/// Reads from the domain and the problem what the configuration checks need.
DomainInfo read_domain(
  plansys2::DomainExpertInterface & domain, plansys2::ProblemExpert & problem,
  const std::optional<std::set<std::string>> & measured_functions);

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__DOMAIN_READER_HPP_
