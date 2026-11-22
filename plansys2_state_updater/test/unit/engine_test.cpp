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

// The engine: writes that failed, the problem kept in sync, rules disabled at run time

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "helpers.hpp"

namespace
{

Facts sorted(Facts facts)
{
  std::sort(
    facts.begin(), facts.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
  return facts;
}

}  // namespace

TEST(engine, a_failed_write_is_tried_again_later)
{
  auto e = engine({leak()});
  e->write_failed(e->on_message("/leak", msg("{data: true}"), 1.0).at(0), 1.0);
  EXPECT_TRUE(e->on_message("/leak", msg("{data: true}"), 2.0).empty());  // Not before 5 s
  EXPECT_TRUE(e->on_timer(5.9).empty());
  EXPECT_EQ(facts(e->on_timer(6.0)), (Facts{{"(leak_detected)", true}}));
  // If the value went back to what was written before, nothing is left to write
  e->write_failed(e->on_message("/leak", msg("{data: false}"), 7.0).at(0), 7.0);
  e->on_message("/leak", msg("{data: true}"), 8.0);
  EXPECT_TRUE(e->on_timer(12.0).empty());
}

TEST(engine, resync_writes_again_what_the_problem_lost)
{
  auto e = engine({battery(), leak("{stale_after: 5.0, on_stale: '(sensor_stale leak)'}"),
        reachable()});
  e->on_message("/battery", msg("{percentage: 0.8}"), 0.0);
  e->on_message("/leak", msg("{data: true}"), 0.0);
  e->on_message("/reachable", msg("{waypoints: [wp1]}"), 0.0);
  // Too recent: the problem may not show it yet
  EXPECT_TRUE(e->resync(problem({}), 0.5).empty());
  EXPECT_TRUE(e->resync(problem({"(leak_detected)", "(reachable wp1)"}, {{"(energy)", 80.0}}),
    2.0).empty());
  // Cleared: all again
  const auto changes = e->resync(problem({}), 2.0);
  EXPECT_EQ(sorted(facts(changes)), (Facts{
    {"(energy)", 80.0}, {"(leak_detected)", true}, {"(reachable wp1)", true}}));
  for (const auto & change : changes) {
    EXPECT_TRUE(contains(change.reason, "resync"));
  }
  // Someone else changed them: the rule is their owner
  e->on_timer(6.0);  // The leak is stale
  EXPECT_EQ(
    sorted(facts(e->resync(problem({"(reachable wp1)", "(reachable wp2)"}, {{"(energy)", 50.0}}),
    9.0))),
    (Facts{{"(energy)", 80.0}, {"(leak_detected)", true}, {"(sensor_stale leak)", true}}));
}

TEST(engine, resync_leaves_the_model_its_functions)
{
  std::map<std::string, double> values = {{"(energy)", 90.0}};
  auto e = engine(
    {battery("{mode: correct, min_change: 5.0}")},
    [&values](const std::string & fact) -> std::optional<double> {
      auto it = values.find(fact);
      return it == values.end() ? std::nullopt : std::optional<double>(it->second);
    });
  e->on_message("/battery", msg("{percentage: 0.5}"), 0.0);
  EXPECT_TRUE(e->resync(problem({}, {{"(energy)", 47.0}}), 2.0).empty());  // The model moved it
  e->on_message("/battery", msg("{percentage: 0.48}"), 3.0);  // Close to the model: not written
  // Gone from the problem: the last reading again
  EXPECT_EQ(facts(e->resync(problem({}), 5.0)), (Facts{{"(energy)", 48.0}}));
}

TEST(engine, resync_waits_for_what_is_pending)
{
  auto e = engine({leak("{debounce: 2.0}")});
  e->on_message("/leak", msg("{data: true}"), 0.0);
  e->on_timer(2.0);
  e->on_message("/leak", msg("{data: false}"), 3.0);  // Pending for its debounce
  EXPECT_TRUE(e->resync(problem({}), 4.0).empty());
}

TEST(engine, a_disabled_rule_writes_nothing_and_comes_back)
{
  auto e = engine({battery("{stale_after: 5.0, on_stale: '(sensor_stale battery)'}"), leak()});
  EXPECT_EQ(e->set_disabled({"battery", "nothing"}, 1.0), (std::vector<std::string>{"nothing"}));
  EXPECT_TRUE(e->on_message("/battery", msg("{percentage: 0.5}"), 1.0).empty());
  EXPECT_TRUE(e->on_timer(20.0).empty());  // Not stale either: it does not listen
  EXPECT_EQ(status(*e, 0).at("enabled"), su::Value(false));
  EXPECT_EQ(status(*e, 1).at("enabled"), su::Value(true));
  EXPECT_EQ(facts(e->on_message("/leak", msg("{data: true}"), 2.0)),
    (Facts{{"(leak_detected)", true}}));
  // Enabled: its stale time counts from now
  EXPECT_TRUE(e->set_disabled({}, 30.0).empty());
  EXPECT_TRUE(e->on_timer(34.0).empty());
  EXPECT_EQ(facts(e->on_timer(35.0)), (Facts{{"(sensor_stale battery)", true}}));
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.5}"), 36.0)).back(),
    (std::pair<std::string, su::FactValue>{"(energy)", 50.0}));
}

TEST(engine, disabled_instances_do_not_expire)
{
  auto e = engine({contacts()});
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  e->set_disabled({"contacts"}, 1.0);
  EXPECT_TRUE(e->on_timer(50.0).empty());
  e->set_disabled({}, 50.0);
  EXPECT_TRUE(e->on_timer(59.0).empty());
  EXPECT_EQ(e->on_timer(60.0).back().fact, "contact_7");
}

TEST(engine, topics_and_their_types)
{
  auto e = engine({battery(), "{name: low, topic: /battery, type: sensor_msgs/msg/BatteryState, "
        "field: percentage, transform: {compare: {op: '<', value: 0.2}}, "
        "target: {predicate: '(low)'}}", leak()});
  const auto topics = e->topics();
  EXPECT_EQ(topics.at("/battery"), "sensor_msgs/msg/BatteryState");
  EXPECT_EQ(topics.at("/leak"), "");
  EXPECT_THROW(su::Engine::from_config(YAML::Load("rules: []")), su::ConfigError);
  EXPECT_THROW(su::Engine::from_config(YAML::Load("{}")), su::ConfigError);
  EXPECT_TRUE(contains(config_error({battery(), battery()}), "two rules are called 'battery'"));
}

TEST(engine, object_fluents_write_their_object)
{
  // (pos r1) has an object, as the domain says: its value is a name
  auto e = engine({
    "{name: pos, topic: /loc, field: area, target: {function: '(pos r1)'}, "
    "policy: {min_period: 1.0}}"});
  auto & rule = dynamic_cast<su::Rule &>(*e->rules()[0]);
  rule.object_fluent = true;
  EXPECT_EQ(facts(e->on_message("/loc", msg("{area: kitchen}"), 0.0)),
    (Facts{{"(pos r1)", S("kitchen")}}));
  EXPECT_TRUE(e->on_message("/loc", msg("{area: kitchen}"), 2.0).empty());
  const auto changes = e->on_message("/loc", msg("{area: hall}"), 3.0);
  EXPECT_EQ(facts(changes), (Facts{{"(pos r1)", S("hall")}}));
  EXPECT_TRUE(contains(changes[0].reason, "kitchen → hall")) << changes[0].reason;
  // A number is not an object
  EXPECT_TRUE(e->on_message("/loc", msg("{area: 7.5}"), 4.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "not a valid PDDL name"));
  // Resync: by its object
  EXPECT_TRUE(e->resync(problem({}, {}, std::nullopt, {{"(pos r1)", "hall"}}), 6.0).empty());
  EXPECT_EQ(facts(e->resync(problem({}, {}, std::nullopt, {{"(pos r1)", "kitchen"}}), 6.0)),
    (Facts{{"(pos r1)", S("hall")}}));
}

TEST(engine, a_reload_keeps_what_the_same_rules_knew)
{
  auto old = engine({leak("{debounce: 2.0}"), battery("{min_change: 2.0}"), contacts()});
  old->on_message("/leak", msg("{data: true}"), 0.0);  // Pending for its debounce
  old->on_message("/battery", msg("{percentage: 0.8}"), 0.0);
  old->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  // The leak and the contacts, the same; the battery, another threshold
  auto e = su::Engine::from_config(YAML::Load(
        "rules: [" + leak("{debounce: 2.0}") + ", " + battery("{min_change: 5.0}") + ", " +
        contacts() + "]"));
  EXPECT_EQ(e->take_over(*old, 1.0), (std::set<std::string>{"leak", "contacts"}));
  // The leak goes on with its debounce, the contact is not created again
  EXPECT_EQ(facts(e->on_timer(2.0)), (Facts{{"(leak_detected)", true}}));
  EXPECT_TRUE(e->on_message("/sonar", sonar({{7, "wp3"}}), 2.0).empty());
  // The battery starts: its first value is written again
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.8}"), 2.0)),
    (Facts{{"(energy)", 80.0}}));
}
