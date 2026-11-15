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

// The fact rules: targets, transforms, policies and configuration

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "helpers.hpp"

TEST(functions, first_value_and_min_change)
{
  auto e = engine({battery("{min_change: 2.0}")});
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.95}"), 1.0)),
    (Facts{{"(energy)", 95.0}}));
  EXPECT_TRUE(e->on_message("/battery", msg("{percentage: 0.94}"), 2.0).empty());  // 1 < 2
  const auto changes = e->on_message("/battery", msg("{percentage: 0.92}"), 3.0);
  EXPECT_EQ(facts(changes), (Facts{{"(energy)", 92.0}}));
  EXPECT_TRUE(contains(changes[0].reason, "deviation 3 ≥ 2")) << changes[0].reason;
}

TEST(functions, without_min_change_only_changes)
{
  auto e = engine({battery()});
  EXPECT_EQ(e->on_message("/battery", msg("{percentage: 0.5}"), 1.0).size(), 1u);
  EXPECT_TRUE(e->on_message("/battery", msg("{percentage: 0.5}"), 2.0).empty());
}

TEST(functions, correct_mode_compares_with_the_problem)
{
  // The model (#t) moved the value in the problem: the sensor writes only if it is far from it
  std::map<std::string, double> problem = {{"(energy)", 90.0}};
  auto current = [&problem](const std::string & fact) -> std::optional<double> {
      auto it = problem.find(fact);
      return it == problem.end() ? std::nullopt : std::optional<double>(it->second);
    };
  auto e = engine({battery("{mode: correct, min_change: 5.0}")}, current);
  EXPECT_TRUE(e->on_message("/battery", msg("{percentage: 0.93}"), 1.0).empty());
  problem["(energy)"] = 80.0;
  const auto changes = e->on_message("/battery", msg("{percentage: 0.93}"), 2.0);
  EXPECT_EQ(facts(changes), (Facts{{"(energy)", 93.0}}));
  EXPECT_TRUE(contains(changes[0].reason, "80 → 93")) << changes[0].reason;
}

TEST(functions, correct_mode_writes_one_not_in_the_problem)
{
  auto e = engine(
    {battery("{mode: correct, min_change: 5.0}")},
    [](const std::string &) {return std::optional<double>();});
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.5}"), 1.0)),
    (Facts{{"(energy)", 50.0}}));
}

TEST(functions, min_period_delays_the_write_to_the_timer)
{
  auto e = engine({battery("{min_period: 1.0}")});
  e->on_message("/battery", msg("{percentage: 0.9}"), 0.0);
  EXPECT_TRUE(e->on_message("/battery", msg("{percentage: 0.8}"), 0.5).empty());
  EXPECT_TRUE(e->on_timer(0.8).empty());
  EXPECT_EQ(facts(e->on_timer(1.0)), (Facts{{"(energy)", 80.0}}));
  EXPECT_TRUE(e->on_timer(1.5).empty());
}

TEST(functions, a_bad_field_is_an_error_of_the_rule)
{
  auto e = engine({battery()});
  EXPECT_TRUE(e->on_message("/battery", msg("{voltage: 22.0}"), 1.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "percentage"));
  e->on_message("/battery", msg("{percentage: 0.5}"), 2.0);
  EXPECT_TRUE(status(*e, 0).at("error").is_null());
}

TEST(functions, linear_round)
{
  auto e = engine({
    "{name: depths, topic: /depths, field: 'items[*].z', "
    "transform: {linear: {scale: -1.0, round: 1}}, target: {function: '(depth_of {id})'}}"});
  const auto changes = e->on_message(
    "/depths", msg("{items: [{id: auv1, z: -3.04}, {id: 'bad name', z: 1.0}]}"), 1.0);
  EXPECT_EQ(facts(changes), (Facts{{"(depth_of auv1)", 3.0}}));
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "not a valid PDDL name"));
}

TEST(predicates, follow_the_value)
{
  auto e = engine({leak()});
  EXPECT_EQ(facts(e->on_message("/leak", msg("{data: false}"), 1.0)),
    (Facts{{"(leak_detected)", false}}));
  EXPECT_EQ(facts(e->on_message("/leak", msg("{data: true}"), 2.0)),
    (Facts{{"(leak_detected)", true}}));
  EXPECT_TRUE(e->on_message("/leak", msg("{data: true}"), 3.0).empty());
}

TEST(predicates, debounce)
{
  auto e = engine({leak("{debounce: 2.0}")});
  e->on_message("/leak", msg("{data: false}"), 0.0);
  e->on_timer(2.0);  // False held: written
  EXPECT_TRUE(e->on_message("/leak", msg("{data: true}"), 10.0).empty());
  EXPECT_TRUE(e->on_message("/leak", msg("{data: false}"), 11.0).empty());  // Noise
  EXPECT_TRUE(e->on_timer(13.0).empty());
  e->on_message("/leak", msg("{data: true}"), 20.0);
  EXPECT_TRUE(e->on_timer(21.9).empty());
  const auto changes = e->on_timer(22.0);
  EXPECT_EQ(facts(changes), (Facts{{"(leak_detected)", true}}));
  EXPECT_TRUE(contains(changes[0].reason, "held 2.0 s")) << changes[0].reason;
}

TEST(predicates, compare)
{
  auto e = engine({
    "{name: low_voltage, topic: /battery, field: voltage, "
    "transform: {compare: {op: '<', value: 22.0}}, target: {predicate: '(low_voltage)'}}"});
  EXPECT_EQ(facts(e->on_message("/battery", msg("{voltage: 21.0}"), 1.0)),
    (Facts{{"(low_voltage)", true}}));
}

TEST(predicates, map)
{
  auto e = engine({
    "{name: degraded, topic: /mode, field: mode, "
    "transform: {map: {values: {2: false, 3: true}, default: false}}, "
    "target: {predicate: '(degraded)'}}"});
  EXPECT_EQ(facts(e->on_message("/mode", msg("{mode: 3}"), 1.0)), (Facts{{"(degraded)", true}}));
  EXPECT_EQ(facts(e->on_message("/mode", msg("{mode: 7}"), 2.0)), (Facts{{"(degraded)", false}}));
}

TEST(predicates, hysteresis)
{
  auto e = engine({
    "{name: low_battery, topic: /battery, field: percentage, "
    "transform: {hysteresis: {on_below: 0.25, off_above: 0.30}}, "
    "target: {predicate: '(low_battery)'}}"});
  std::vector<su::FactValue> values;
  const std::vector<double> readings = {0.5, 0.26, 0.24, 0.27, 0.29, 0.31, 0.28};
  for (size_t t = 0; t < readings.size(); t++) {
    const su::Value reading(su::Value::Map{{"percentage", readings[t]}});
    for (const auto & change : e->on_message("/battery", reading, static_cast<double>(t))) {
      values.push_back(change.value);
    }
  }
  // In 0.25-0.30 it stays as it was
  EXPECT_EQ(values, (std::vector<su::FactValue>{false, true, false}));
}

TEST(stale, a_predicate_while_stale)
{
  auto e = engine({battery("{stale_after: 10.0, on_stale: '(sensor_stale battery)'}")});
  EXPECT_TRUE(e->on_timer(9.0).empty());
  EXPECT_EQ(facts(e->on_timer(10.0)), (Facts{{"(sensor_stale battery)", true}}));  // Never came
  EXPECT_TRUE(e->on_timer(11.0).empty());
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.9}"), 12.0)),
    (Facts{{"(sensor_stale battery)", false}, {"(energy)", 90.0}}));
  EXPECT_EQ(facts(e->on_timer(22.0)), (Facts{{"(sensor_stale battery)", true}}));
}

TEST(stale, keep_writes_nothing)
{
  auto e = engine({battery("{stale_after: 1.0}")});
  EXPECT_TRUE(e->on_timer(5.0).empty());
  EXPECT_EQ(status(*e, 0).at("stale"), su::Value(true));
}

TEST(stale, a_value_for_a_function_with_a_predicate)
{
  auto e = engine(
    {battery("{stale_after: 5.0, on_stale: {predicate: '(sensor_stale battery)', value: 0.0}}")});
  e->on_message("/battery", msg("{percentage: 0.8}"), 1.0);
  const auto changes = e->on_timer(6.0);
  EXPECT_EQ(facts(changes), (Facts{{"(sensor_stale battery)", true}, {"(energy)", 0.0}}));
  EXPECT_TRUE(contains(changes[1].reason, "on_stale value"));
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.7}"), 7.0)),
    (Facts{{"(sensor_stale battery)", false}, {"(energy)", 70.0}}));
}

TEST(stale, a_value_for_an_enum_and_for_a_template)
{
  auto e = engine({mode("{stale_after: 2.0, on_stale: {value: lost}}")});
  e->on_message("/mode", msg("{mode: 2}"), 0.0);
  EXPECT_EQ(facts(e->on_timer(2.0)),
    (Facts{{"(mode_is normal)", false}, {"(mode_is lost)", true}}));

  e = engine({diagnostics("{stale_after: 2.0, on_stale: {value: false}}")});
  e->on_message("/diagnostics", status_msg({{"imu", 0}, {"dvl", 0}}), 0.0);
  e->on_message("/diagnostics", status_msg({{"imu", 0}}), 1.5);
  EXPECT_EQ(facts(e->on_timer(2.0)), (Facts{{"(sensor_ok dvl)", false}}));  // The silent one
}

TEST(templates, one_fact_per_element)
{
  auto e = engine({diagnostics()});
  const auto changes = e->on_message("/diagnostics", status_msg({{"imu", 0}, {"dvl", 2}}), 1.0);
  EXPECT_EQ(facts(changes), (Facts{{"(sensor_ok imu)", true}, {"(sensor_ok dvl)", false}}));
  EXPECT_TRUE(contains(changes[0].reason, "sensors (sensor_ok imu): None → True"));
  // An entry that does not come is left as it was; the one that changes is written
  EXPECT_EQ(facts(e->on_message("/diagnostics", status_msg({{"dvl", 0}}), 2.0)),
    (Facts{{"(sensor_ok dvl)", true}}));
}

TEST(templates, debounce_per_fact)
{
  auto e = engine({diagnostics("{debounce: 1.0}")});
  e->on_message("/diagnostics", status_msg({{"imu", 0}, {"dvl", 0}}), 0.0);
  EXPECT_EQ(e->on_timer(1.0).size(), 2u);
  e->on_message("/diagnostics", status_msg({{"imu", 2}}), 5.0);
  e->on_message("/diagnostics", status_msg({{"dvl", 2}}), 5.5);
  EXPECT_EQ(facts(e->on_timer(6.0)), (Facts{{"(sensor_ok imu)", false}}));
  EXPECT_EQ(facts(e->on_timer(6.5)), (Facts{{"(sensor_ok dvl)", false}}));
}

TEST(templates, stale_per_fact)
{
  auto e = engine({diagnostics("{stale_after: 5.0, on_stale: '(sensor_stale {name})'}")});
  e->on_message("/diagnostics", status_msg({{"imu", 0}, {"dvl", 0}}), 1.0);
  e->on_message("/diagnostics", status_msg({{"imu", 0}}), 4.0);
  EXPECT_EQ(facts(e->on_timer(6.0)), (Facts{{"(sensor_stale dvl)", true}}));  // imu came at 4
  EXPECT_EQ(facts(e->on_message("/diagnostics", status_msg({{"dvl", 0}}), 7.0)),
    (Facts{{"(sensor_stale dvl)", false}}));
}

TEST(sets, add_the_new_and_remove_the_gone)
{
  auto e = engine({reachable()});
  EXPECT_EQ(facts(e->on_message("/reachable", msg("{waypoints: [wp1, wp3]}"), 1.0)),
    (Facts{{"(reachable wp1)", true}, {"(reachable wp3)", true}}));
  EXPECT_EQ(facts(e->on_message("/reachable", msg("{waypoints: [wp3, wp4]}"), 2.0)),
    (Facts{{"(reachable wp4)", true}, {"(reachable wp1)", false}}));
  EXPECT_TRUE(e->on_message("/reachable", msg("{waypoints: [wp3, wp4]}"), 3.0).empty());
  EXPECT_EQ(facts(e->on_message("/reachable", msg("{waypoints: []}"), 4.0)),
    (Facts{{"(reachable wp3)", false}, {"(reachable wp4)", false}}));
}

TEST(sets, of_structs_with_debounce)
{
  auto e = engine({reachable("{debounce: 1.0}", "legs[*]", "(blocked {from} {to})")});
  e->on_message("/reachable", msg("{legs: [{from: wp1, to: wp2}]}"), 0.0);
  EXPECT_TRUE(e->on_timer(0.5).empty());
  EXPECT_EQ(facts(e->on_timer(1.0)), (Facts{{"(blocked wp1 wp2)", true}}));
  e->on_message("/reachable", msg("{legs: []}"), 2.0);
  e->on_message("/reachable", msg("{legs: [{from: wp1, to: wp2}]}"), 2.5);  // Back: noise
  EXPECT_TRUE(e->on_timer(3.5).empty());
}

TEST(sets, stale_for_the_whole_list)
{
  auto e = engine({reachable("{stale_after: 2.0, on_stale: '(sensor_stale sonar)'}")});
  e->on_message("/reachable", msg("{waypoints: [wp1]}"), 1.0);
  EXPECT_EQ(facts(e->on_timer(3.0)), (Facts{{"(sensor_stale sonar)", true}}));
  EXPECT_TRUE(contains(
      config_error({reachable("{stale_after: 2.0, on_stale: '(sensor_stale {})'}")}),
      "no placeholder"));
}

TEST(enums, one_holds_at_a_time)
{
  auto e = engine({mode()});
  // The first value: the others go too, whatever the problem file had
  EXPECT_EQ(facts(e->on_message("/mode", msg("{mode: 2}"), 1.0)),
    (Facts{{"(mode_is degraded)", false}, {"(mode_is lost)", false},
      {"(mode_is normal)", true}}));
  // The one that held goes before the new one holds: never two at once
  EXPECT_EQ(facts(e->on_message("/mode", msg("{mode: 3}"), 2.0)),
    (Facts{{"(mode_is normal)", false}, {"(mode_is degraded)", true}}));
  EXPECT_TRUE(e->on_message("/mode", msg("{mode: 3}"), 3.0).empty());
  EXPECT_EQ(facts(e->on_message("/mode", msg("{mode: 9}"), 4.0)),
    (Facts{{"(mode_is degraded)", false}, {"(mode_is lost)", true}}));
}

TEST(enums, debounce_without_values)
{
  auto e = engine({mode("{debounce: 1.0}", false)});
  e->on_message("/mode", msg("{mode: 2}"), 0.0);
  EXPECT_EQ(facts(e->on_timer(1.0)), (Facts{{"(mode_is normal)", true}}));
  e->on_message("/mode", msg("{mode: 3}"), 2.0);
  e->on_message("/mode", msg("{mode: 2}"), 2.5);  // Noise: back to normal before 1 s
  EXPECT_TRUE(e->on_timer(3.5).empty());
}

TEST(enums, a_value_out_of_its_values)
{
  auto e = engine({
    "{name: mode, topic: /mode, field: mode, "
    "target: {enum: '(mode_is {})', values: [normal, degraded, lost]}}"});
  EXPECT_TRUE(e->on_message("/mode", msg("{mode: flying}"), 1.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "not one of the values"));
}

TEST(enums, range_gives_the_names)
{
  auto e = engine({
    "{name: depth, topic: /depth, field: z, "
    "transform: {range: {limits: [5, 30], values: [shallow, mid, deep]}}, "
    "target: {enum: '(depth_is {})'}}"});
  std::vector<std::string> names;
  const std::vector<double> depths = {1.0, 4.99, 5.0, 29.0, 30.0, 400.0};
  for (size_t t = 0; t < depths.size(); t++) {
    const su::Value reading(su::Value::Map{{"z", depths[t]}});
    for (const auto & change : e->on_message("/depth", reading, static_cast<double>(t))) {
      if (change.value == su::FactValue(true)) {
        names.push_back(change.fact);
      }
    }
  }
  EXPECT_EQ(names,
    (std::vector<std::string>{"(depth_is shallow)", "(depth_is mid)", "(depth_is deep)"}));
}

TEST(plugins, on_the_whole_message)
{
  // The lowest of the cells, by their voltage, not below 3: MinOf, the plugin that comes with it
  auto e = engine({
    "{name: cell, topic: /battery, field: cells, transform: {plugin: {function: "
    "plansys2_state_updater/MinOf, args: {field: voltage, minimum: 3.0}}}, "
    "target: {function: '(lowest_cell)'}}"});
  EXPECT_EQ(facts(e->on_message("/battery", msg("{cells: [{voltage: 3.7}, {voltage: 3.5}]}"),
    1.0)), (Facts{{"(lowest_cell)", 3.5}}));
  EXPECT_EQ(facts(e->on_message("/battery", msg("{cells: [{voltage: 2.0}]}"), 2.0)),
    (Facts{{"(lowest_cell)", 3.0}}));
  // What the plugin throws is an error of its rule, not of the node
  EXPECT_TRUE(e->on_message("/battery", msg("{cells: []}"), 3.0).empty());
  EXPECT_TRUE(contains(
      status(*e, 0).at("error").text(), "plugin 'plansys2_state_updater/MinOf'"));
}

TEST(plugins, that_cannot_load)
{
  for (const auto & function : {"no_package/Nothing", "plansys2_state_updater/Nothing", ""}) {
    EXPECT_TRUE(contains(
        config_error({std::string(
          "{name: p, topic: /p, transform: {plugin: {function: '") + function +
          "'}}, target: {function: '(f)'}}"}), "plugin")) << function;
  }
}

TEST(configuration, every_error_is_reported)
{
  const std::string text = config_error({
    battery("{debounce: 1.0, colour: red}"),
    "{name: leak, topic: /leak, field: data, transform: {linear: {}}, "
    "target: {predicate: '(leak_detected)'}, policy: {mode: correct}}",
    "{name: x, topic: /x, field: a, transform: {compare: {op: '=~', value: 1}}, "
    "target: {function: '(f)'}}"});
  EXPECT_TRUE(contains(text, "unknown policy keys ['colour']")) << text;
  EXPECT_TRUE(contains(text, "debounce is for predicates")) << text;
  EXPECT_TRUE(contains(text, "mode 'correct' is for functions")) << text;
  EXPECT_TRUE(contains(text, "compare needs op")) << text;
}

TEST(configuration, one_owner_per_fact)
{
  EXPECT_TRUE(contains(
      config_error({battery(), "{name: battery2, topic: /b, field: p, "
        "target: {function: '(energy)'}}"}), "each fact has one owner"));
  // A template owns its whole predicate
  EXPECT_TRUE(contains(
      config_error({diagnostics(), "{name: leak, topic: /leak, field: data, "
        "target: {predicate: '(sensor_ok imu)'}}"}), "a template, its whole predicate"));
  EXPECT_TRUE(contains(
      config_error({diagnostics("{stale_after: 1.0, on_stale: '(sensor_stale {name})'}"),
        battery("{stale_after: 1.0, on_stale: '(sensor_stale battery)'}")}),
      "each fact has one owner"));
  // Different predicates, or different fixed facts of one predicate: fine
  EXPECT_EQ(config_error({diagnostics(),
      battery("{stale_after: 1.0, on_stale: '(sensor_stale battery)'}"),
      leak("{stale_after: 1.0, on_stale: '(sensor_stale leak)'}")}), "");
}

TEST(configuration, kinds_and_targets)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"{name: b, topic: /b, field: p, transform: {compare: {op: '<', value: 0.2}}, "
      "target: {function: '(energy)'}}", "the target must be a predicate"},
    {"{name: s, topic: /s, field: 'status[*]', target: {predicate: '(all_ok)'}}",
      "needs a placeholder"},
    {reachable().substr(0, reachable().size() - 1) + ", transform: {linear: {}}}",
      "a set takes no transform"},
    {"{name: m, topic: /m, field: m, target: {enum: '(mode_is)'}}", "one '{}'"},
    {"{name: m, topic: /m, field: m, target: {enum: '(mode_is {name})'}}", "one '{}'"},
    {"{name: m, topic: /m, field: m, target: {enum: '(mode_is {})', values: ['bad name']}}",
      "not PDDL names"},
    {"{name: m, topic: /m, field: m, transform: {compare: {op: '<', value: 1}}, "
      "target: {enum: '(mode_is {})'}}", "an enum takes a name"},
    {"{name: m, topic: /m, field: m, target: {range: '(x)'}}", "target must be"},
    {"{name: d, topic: /d, field: z, transform: {range: {limits: [5], values: [a]}}, "
      "target: {enum: '(d {})'}}", "range"},
    {"{name: d, topic: /d, field: z, transform: {range: {limits: [30, 5], values: [1, 2, 3]}}, "
      "target: {enum: '(d {})'}}", "range"},
    {battery("{stale_after: 1.0, on_stale: {value: true}}"), "is a number"},
    {leak("{stale_after: 1.0, on_stale: {value: 0.5}}"), "true or false"},
    {mode("{stale_after: 1.0, on_stale: {value: flying}}"), "not in the values"},
    {reachable("{stale_after: 1.0, on_stale: {value: true}}"), "takes no value"},
    {leak("{on_stale: {value: true}}"), "needs stale_after"},
    {leak("{stale_after: 1.0, on_stale: {colour: red}}"), "on_stale must be"},
    {"{name: t, topic: /t, field: 'a..b', target: {predicate: '(p)'}}", "field path"},
    {"{topic: /t, field: a, target: {predicate: '(p)'}}", "a rule has no name"},
  };
  for (const auto & [rule, error] : cases) {
    const std::string text = config_error({rule});
    EXPECT_TRUE(contains(text, error)) << rule << "\n  gives: " << text;
  }
}

TEST(configuration, warns_of_correct_without_min_change)
{
  auto e = engine({battery("{mode: correct}")});
  ASSERT_EQ(e->rules()[0]->warnings.size(), 1u);
  EXPECT_TRUE(contains(e->rules()[0]->warnings[0], "without min_change"));
}
