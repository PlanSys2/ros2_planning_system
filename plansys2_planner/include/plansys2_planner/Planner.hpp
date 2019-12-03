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

#ifndef PLANSYS2_PLANNER__PLANNER_HPP_
#define PLANSYS2_PLANNER__PLANNER_HPP_

#include <memory>
#include <string>

#include "plansys2_planner/PlannerInterface.hpp"

namespace plansys2
{

class Planner : public PlannerInterface
{
public:
  Planner();

  std::optional<Plan> getPlan(std::string domain, std::string problem);
};

}  // namespace plansys2

#endif  // PLANSYS2_PLANNER__PLANNER_HPP_
