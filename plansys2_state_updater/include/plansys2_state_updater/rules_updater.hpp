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

#ifndef PLANSYS2_STATE_UPDATER__RULES_UPDATER_HPP_
#define PLANSYS2_STATE_UPDATER__RULES_UPDATER_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_msgs/msg/state_update.hpp"
#include "plansys2_problem_expert/ProblemUpdater.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

#include "plansys2_state_updater/engine.hpp"
#include "plansys2_state_updater/message_reader.hpp"
#include "plansys2_state_updater/problem_writer.hpp"

namespace plansys2_state_updater
{

/// Problem updater driven by YAML rules mapping topic fields to facts. Parameters under <name>:
/// config_file, dry_run, timer_period, check_domain, resync_period, disabled_rules, retry_after.
class RulesUpdater : public plansys2::ProblemUpdater
{
public:
  bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & name,
    const plansys2::ProblemAccess & access) override;
  bool activate() override;
  void deactivate() override;
  void cleanup() override;
  /// Rules that no longer fit the new domain write nothing until they do
  void domain_changed() override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const;
  rclcpp::Logger logger() const;
  std::string param(const std::string & key) const {return name_ + "." + key;}

  /// @throws ConfigError with every error
  std::pair<YAML::Node, std::unique_ptr<Engine>> load();
  std::unique_ptr<Engine> make_engine(const YAML::Node & config);
  void domain_facts(Engine & engine);
  std::vector<std::string> check(Engine & engine);

  void subscribe();
  /// The qos to subscribe to a topic with, from what its publishers offer (empty: none yet)
  std::optional<Qos> publishers_qos(const std::string & topic) const;
  /// Subscribe again where the publishers now offer another qos
  void follow_publishers();
  void stop();
  /// With use_sim_time, false until /clock publishes: the rules start then
  bool clock_ready();
  double seconds() const;

  void on_message(const std::string & topic, const rclcpp::SerializedMessage & message);
  /// A publisher that offers less than the subscription asks for: with "auto", subscribed again
  void on_incompatible_qos(
    const std::string & topic, const rclcpp::QOSRequestedIncompatibleQoSInfo & info);
  void on_timer();
  void write(const std::vector<Change> & changes);
  plansys2_msgs::msg::StateUpdate event(const Change & change, bool written) const;

  std::vector<std::string> disabled_rules() const;
  std::vector<std::string> not_enabled() const;
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void on_reload(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  plansys2::ProblemAccess access_;
  std::unique_ptr<ProblemWriter> writer_;
  YAML::Node config_;
  std::unique_ptr<Engine> engine_;
  bool active_ {false};
  bool started_ {false};
  // Rules that do not fit the domain since it changed, with the reason
  std::map<std::string, std::string> invalid_;

  std::map<std::string, std::string> topics_;
  std::map<std::string, Qos> qos_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  // Subscribed again on the next tick, to take what their publishers offer now
  std::set<std::string> resubscribe_;
  std::set<std::string> incompatible_logged_;
  // The qos each subscription took
  std::map<std::string, Qos> taken_;
  std::chrono::steady_clock::time_point last_publishers_;
  std::map<std::string, std::shared_ptr<MessageReader>> readers_;
  std::set<std::string> waiting_logged_;
  rclcpp::TimerBase::SharedPtr timer_;
  double last_status_ {0.0};
  double last_resync_ {0.0};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::StateUpdate>::SharedPtr events_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_handle_;
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__RULES_UPDATER_HPP_
