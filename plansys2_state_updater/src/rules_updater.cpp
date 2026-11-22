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

#include "plansys2_state_updater/rules_updater.hpp"

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "plansys2_state_updater/domain_check.hpp"
#include "plansys2_state_updater/domain_reader.hpp"

namespace plansys2_state_updater
{

using plansys2_msgs::msg::StateUpdate;
using std::placeholders::_1;
using std::placeholders::_2;

rclcpp_lifecycle::LifecycleNode::SharedPtr
RulesUpdater::node() const
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("the problem expert is gone");
  }
  return node;
}

rclcpp::Logger
RulesUpdater::logger() const
{
  return node()->get_logger().get_child(name_);
}

double
RulesUpdater::seconds() const
{
  return node()->now().seconds();
}

bool
RulesUpdater::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak, const std::string & name,
  const plansys2::ProblemAccess & access)
{
  node_ = weak;
  name_ = name;
  access_ = access;
  writer_ = std::make_unique<ProblemWriter>(access);
  auto node = this->node();

  auto declare = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      if (!node->has_parameter(param(key))) {
        node->declare_parameter(param(key), value);
      }
    };
  declare("config_file", rclcpp::ParameterValue(std::string()));
  // Log the changes without writing them
  declare("dry_run", rclcpp::ParameterValue(false));
  // Period of the debounce, min_period, stale and expire_after checks
  declare("timer_period", rclcpp::ParameterValue(0.1));
  // Check the rules against the domain and the problem on configure
  declare("check_domain", rclcpp::ParameterValue(true));
  // Seconds between rewrites of the facts the problem lost (0: never)
  declare("resync_period", rclcpp::ParameterValue(2.0));
  // Rules that write nothing; can be changed at run time
  declare("disabled_rules", rclcpp::ParameterValue(std::vector<std::string>{}));
  // Seconds before retrying a change the problem expert rejected
  declare("retry_after", rclcpp::ParameterValue(5.0));

  try {
    std::tie(config_, engine_) = load();
  } catch (const ConfigError & e) {
    RCLCPP_ERROR(logger(), "Invalid configuration:\n%s", e.what());
    engine_.reset();
    return false;
  }
  topics_ = engine_->topics();
  status_pub_ = node->create_publisher<std_msgs::msg::String>(name_ + "/status", 10);
  // For missions to replan on: update_notify is also published by the dynamics
  events_pub_ = node->create_publisher<StateUpdate>(name_ + "/events", 100);
  // The old rules stay if the new ones are not valid
  reload_service_ = node->create_service<std_srvs::srv::Trigger>(
    name_ + "/reload", std::bind(&RulesUpdater::on_reload, this, _1, _2));
  parameters_handle_ = node->add_on_set_parameters_callback(
    std::bind(&RulesUpdater::on_parameters, this, _1));
  RCLCPP_INFO(
    logger(), "%zu rules on %zu topics%s", engine_->rules().size(), topics_.size(),
    node->get_parameter(param("dry_run")).as_bool() ? " (dry run: nothing is written)" : "");
  return true;
}

bool
RulesUpdater::activate()
{
  // From scratch: the world may have changed while inactive
  engine_ = make_engine(config_);
  started_ = false;
  clock_ready();
  writer_->start(node());
  status_pub_->on_activate();
  events_pub_->on_activate();
  active_ = true;
  subscribe();
  timer_ = node()->create_wall_timer(
    std::chrono::duration<double>(node()->get_parameter(param("timer_period")).as_double()),
    std::bind(&RulesUpdater::on_timer, this));
  return true;
}

void
RulesUpdater::deactivate()
{
  stop();
  if (status_pub_) {
    status_pub_->on_deactivate();
    events_pub_->on_deactivate();
  }
}

void
RulesUpdater::cleanup()
{
  stop();
  if (auto node = node_.lock(); node && parameters_handle_) {
    node->remove_on_set_parameters_callback(parameters_handle_.get());
  }
  parameters_handle_.reset();
  reload_service_.reset();
  status_pub_.reset();
  events_pub_.reset();
  engine_.reset();
  readers_.clear();
  writer_.reset();
}

bool
RulesUpdater::clock_ready()
{
  // Starting at time 0 (use_sim_time, no /clock yet) would make every rule look stale
  if (started_) {
    return true;
  }
  const double now = seconds();
  if (now == 0.0) {
    return false;
  }
  engine_->start(now);
  engine_->set_disabled(not_enabled(), now);
  started_ = true;
  return true;
}

void
RulesUpdater::stop()
{
  active_ = false;
  subscriptions_.clear();
  waiting_logged_.clear();
  timer_.reset();
  if (writer_) {
    writer_->stop();
  }
}

std::unique_ptr<Engine>
RulesUpdater::make_engine(const YAML::Node & config)
{
  auto engine = Engine::from_config(
    config,
    [this](const std::string & fact) {return writer_->current_value(fact);},
    [this](const std::string & name) {return writer_->in_use(name);});
  domain_facts(*engine);
  return engine;
}

void
RulesUpdater::domain_facts(Engine & engine)
{
  const auto types = access_.domain->getTypes();
  for (const auto & base : engine.rules()) {
    if (auto * rule = dynamic_cast<Rule *>(base.get()); rule && rule->kind == "function") {
      const auto function = access_.domain->getFunction(rule->fact_name);
      rule->object_fluent = function && !function->object.empty();
    } else if (auto * instances = dynamic_cast<InstanceRule *>(base.get())) {
      instances->types = std::set<std::string>(types.begin(), types.end());
    }
  }
}

std::pair<YAML::Node, std::unique_ptr<Engine>>
RulesUpdater::load()
{
  const std::string config_file = node()->get_parameter(param("config_file")).as_string();
  if (config_file.empty()) {
    throw ConfigError("parameter " + param("config_file") + " is empty");
  }
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const YAML::Exception & e) {
    throw ConfigError("cannot read " + config_file + ": " + e.what());
  }
  auto engine = make_engine(config);
  std::vector<std::string> warnings;
  for (const auto & rule : engine->rules()) {
    warnings.insert(warnings.end(), rule->warnings.begin(), rule->warnings.end());
  }
  if (node()->get_parameter(param("check_domain")).as_bool()) {
    warnings = check(*engine);
  }
  for (const auto & warning : warnings) {
    RCLCPP_WARN(logger(), "%s", warning.c_str());
  }
  return {config, std::move(engine)};
}

std::vector<std::string>
RulesUpdater::check(Engine & engine)
{
  // A parameter of the problem expert itself, not under name_
  const auto measured = node()->get_parameter("measured_functions").as_string_array();
  const auto domain = read_domain(
    *access_.domain, *access_.problem, std::set<std::string>(measured.begin(), measured.end()));
  auto [errors, warnings] = check_domain(engine, domain);
  if (!errors.empty()) {
    std::string text;
    for (const auto & e : errors) {
      text += (text.empty() ? "" : "\n") + e;
    }
    throw ConfigError(text);
  }
  return warnings;
}

void
RulesUpdater::subscribe()
{
  auto node = this->node();
  const auto graph = node->get_topic_names_and_types();
  for (const auto & [topic, given] : topics_) {
    if (subscriptions_.count(topic)) {
      continue;
    }
    std::string type = given;
    if (type.empty()) {
      auto it = graph.find(node->get_node_topics_interface()->resolve_topic_name(topic));
      if (it == graph.end() || it->second.empty()) {
        if (waiting_logged_.insert(topic).second) {
          RCLCPP_INFO(logger(), "Waiting for '%s' to appear, to know its type", topic.c_str());
        }
        continue;
      }
      type = it->second.front();
    }
    try {
      if (!readers_.count(topic) || readers_[topic]->type() != type) {
        readers_[topic] = std::make_shared<MessageReader>(type);
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(logger(), "Unknown type '%s' of '%s': %s", type.c_str(), topic.c_str(),
        e.what());
      continue;
    }
    // Default callback group: mutually exclusive with the problem expert services
    subscriptions_[topic] = node->create_generic_subscription(
      topic, type, rclcpp::SensorDataQoS(),
      [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        on_message(topic, *message);
      });
    RCLCPP_INFO(logger(), "Subscribed to '%s' (%s)", topic.c_str(), type.c_str());
  }
}

void
RulesUpdater::on_message(const std::string & topic, const rclcpp::SerializedMessage & message)
{
  if (!active_ || !engine_ || !clock_ready()) {
    return;
  }
  // Read lazily: only if a rule samples it now (sample_period)
  std::vector<Change> changes;
  try {
    changes = engine_->receive(
      topic, [&]() {return readers_.at(topic)->read(message);}, seconds());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Cannot read a message of '%s': %s", topic.c_str(), e.what());
    return;
  }
  write(changes);
}

void
RulesUpdater::on_timer()
{
  if (!active_ || !engine_ || !clock_ready()) {
    return;
  }
  if (subscriptions_.size() < topics_.size()) {
    subscribe();
  }
  const double now = seconds();
  // The clock went back (simulation restarted): the periods count from now
  if (last_resync_ > now || last_status_ > now) {
    last_resync_ = now;
    last_status_ = now - 1.0;
  }
  write(engine_->on_timer(now));
  const double period = node()->get_parameter(param("resync_period")).as_double();
  const bool dry_run = node()->get_parameter(param("dry_run")).as_bool();
  if (period > 0.0 && !dry_run && now - last_resync_ >= period) {
    last_resync_ = now;
    ProblemState problem;
    problem.predicates = writer_->predicates();
    problem.functions = writer_->functions();
    problem.objects = writer_->objects();
    problem.instances = writer_->instances();
    write(engine_->resync(problem, now, period));
  }
  if (now - last_status_ >= 1.0) {
    last_status_ = now;
    std_msgs::msg::String status;
    status.data = engine_->status().json();
    status_pub_->publish(status);
  }
}

void
RulesUpdater::write(const std::vector<Change> & changes)
{
  if (changes.empty()) {
    return;
  }
  const bool dry_run = node()->get_parameter(param("dry_run")).as_bool();
  for (const auto & change : changes) {
    RCLCPP_INFO(logger(), "%s%s", dry_run ? "[dry run] " : "", change.reason.c_str());
  }
  const std::vector<bool> results = dry_run ?
    std::vector<bool>(changes.size(), false) : writer_->write(changes);
  const double retry_after = node()->get_parameter(param("retry_after")).as_double();
  for (size_t i = 0; i < changes.size(); i++) {
    if (!results[i] && !dry_run) {
      RCLCPP_ERROR(
        logger(), "Cannot write %s = %s: tried again in %g s", changes[i].fact.c_str(),
        to_text(changes[i].value).c_str(), retry_after);
      engine_->write_failed(changes[i], seconds(), retry_after);
    }
    events_pub_->publish(event(changes[i], results[i]));
  }
}

StateUpdate
RulesUpdater::event(const Change & change, bool written) const
{
  StateUpdate event;
  event.stamp = node()->now();
  event.rule = change.rule;
  event.fact = change.fact;
  event.reason = change.reason;
  event.written = written;
  if (change.kind == "function") {
    event.kind = StateUpdate::FUNCTION;
    if (std::holds_alternative<std::string>(change.value)) {
      event.object = std::get<std::string>(change.value);
    } else {
      event.value = std::get<double>(change.value);
    }
  } else if (change.kind == "instance") {
    event.kind = StateUpdate::INSTANCE;
    event.holds = std::holds_alternative<std::string>(change.value);
  } else {
    event.kind = StateUpdate::PREDICATE;
    event.holds = std::holds_alternative<bool>(change.value) && std::get<bool>(change.value);
  }
  const auto & previous = change.previous;
  event.has_previous = !std::holds_alternative<std::monostate>(previous);
  if (std::holds_alternative<double>(previous)) {
    event.previous_value = std::get<double>(previous);
  } else if (std::holds_alternative<std::string>(previous)) {
    event.previous_object = std::get<std::string>(previous);
  } else if (std::holds_alternative<bool>(previous)) {
    event.previous_holds = std::get<bool>(previous);
  }
  return event;
}

std::vector<std::string>
RulesUpdater::disabled_rules() const
{
  return node()->get_parameter(param("disabled_rules")).as_string_array();
}

std::vector<std::string>
RulesUpdater::not_enabled() const
{
  auto names = disabled_rules();
  for (const auto & [name, why] : invalid_) {
    names.push_back(name);
  }
  return names;
}

void
RulesUpdater::domain_changed()
{
  if (!engine_ || !node()->get_parameter(param("check_domain")).as_bool()) {
    return;
  }
  const auto measured = node()->get_parameter("measured_functions").as_string_array();
  const auto domain = read_domain(
    *access_.domain, *access_.problem, std::set<std::string>(measured.begin(), measured.end()));
  // Each error starts with its rule: rule 'battery': ...
  std::map<std::string, std::string> invalid;
  for (const auto & error : check_domain(*engine_, domain).first) {
    const auto start = error.find('\'') + 1, end = error.find('\'', start);
    auto & why = invalid[error.substr(start, end - start)];
    why += (why.empty() ? "" : "; ") + error;
  }
  domain_facts(*engine_);  // Types and object fluents may have changed
  for (const auto & rule : engine_->rules()) {
    auto it = invalid.find(rule->name);
    if (it != invalid.end()) {
      rule->error = "it does not fit the new domain: " + it->second;
      if (!invalid_.count(rule->name)) {
        RCLCPP_ERROR(logger(), "Rule %s disabled: %s", rule->name.c_str(), it->second.c_str());
      }
    } else if (invalid_.count(rule->name)) {
      rule->error.reset();
      RCLCPP_INFO(logger(), "Rule %s fits the domain again: enabled", rule->name.c_str());
    }
  }
  invalid_ = invalid;
  engine_->set_disabled(not_enabled(), seconds());
}

rcl_interfaces::msg::SetParametersResult
RulesUpdater::on_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() != param("disabled_rules") || !engine_) {
      continue;
    }
    const auto unknown = engine_->set_disabled(parameter.as_string_array(), seconds());
    if (!unknown.empty()) {
      // Restore: the parameter change is rejected
      engine_->set_disabled(not_enabled(), seconds());
      result.successful = false;
      result.reason = "no rules called";
      for (const auto & name : unknown) {
        result.reason += " '" + name + "'";
      }
      return result;
    }
    // Rules that do not fit the domain stay disabled
    std::vector<std::string> names = parameter.as_string_array();
    for (const auto & [name, why] : invalid_) {
      names.push_back(name);
    }
    engine_->set_disabled(names, seconds());
    std::string text;
    for (const auto & name : engine_->disabled()) {
      text += (text.empty() ? "" : ", ") + name;
    }
    RCLCPP_INFO(logger(), "Disabled rules: %s", text.empty() ? "none" : text.c_str());
  }
  return result;
}

void
RulesUpdater::on_reload(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!engine_) {
    response->success = false;
    response->message = "not configured";
    return;
  }
  YAML::Node config;
  std::unique_ptr<Engine> engine;
  try {
    std::tie(config, engine) = load();
  } catch (const ConfigError & e) {
    response->success = false;
    response->message = std::string("the rules stay as they were: ") + e.what();
    RCLCPP_ERROR(logger(), "Reload: %s", response->message.c_str());
    return;
  }
  if (active_) {
    subscriptions_.clear();
    waiting_logged_.clear();
  }
  // Unchanged rules keep their state
  std::set<std::string> kept;
  if (active_ && started_) {
    kept = engine->take_over(*engine_, seconds());
  }
  config_ = config;
  engine_ = std::move(engine);
  topics_ = engine_->topics();
  invalid_.clear();  // Checked against the domain as it is now
  std::vector<std::string> unknown;
  if (active_) {
    unknown = engine_->set_disabled(not_enabled(), seconds());
    subscribe();
  }
  response->success = true;
  response->message = std::to_string(engine_->rules().size()) + " rules on " +
    std::to_string(topics_.size()) + " topics, " + std::to_string(kept.size()) +
    " kept as they were";
  if (!unknown.empty()) {
    response->message += "; disabled_rules names no rules:";
    for (const auto & name : unknown) {
      response->message += " '" + name + "'";
    }
  }
  RCLCPP_INFO(logger(), "Reload: %s", response->message.c_str());
}

}  // namespace plansys2_state_updater

PLUGINLIB_EXPORT_CLASS(plansys2_state_updater::RulesUpdater, plansys2::ProblemUpdater)
