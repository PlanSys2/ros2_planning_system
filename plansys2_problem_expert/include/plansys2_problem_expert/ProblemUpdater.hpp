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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMUPDATER_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMUPDATER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

struct ProblemAccess
{
  std::shared_ptr<ProblemExpert> problem;
  std::shared_ptr<DomainExpert> domain;
  /// Publishes update_notify, knowledge and problem; call once after a batch of changes
  std::function<void()> notify;
};

/// Plugin loaded by the problem expert (updater_plugins, class in <name>.plugin) that keeps the
/// problem in sync with the world.
/// Its callbacks share the node's mutually exclusive default callback group with the problem
/// expert services, so they never run concurrently with them; they must not block. Callbacks in a
/// callback group of its own must not touch the problem: they hand their data to one in the
/// default group.
class ProblemUpdater
{
public:
  using Ptr = std::shared_ptr<ProblemUpdater>;

  virtual ~ProblemUpdater() = default;

  /// Called on the problem expert configure, once the problem is loaded. Parameters are under
  /// name. Returning false makes the problem expert fail to configure.
  virtual bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & name,
    const ProblemAccess & access) = 0;

  /// Returning false makes the problem expert fail to activate
  virtual bool activate() = 0;

  /// No more writes until activated again
  virtual void deactivate() = 0;

  virtual void cleanup() {}

  /// Called after ChangeDomain; ProblemAccess::domain is already the new one
  virtual void domain_changed() {}
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMUPDATER_HPP_
