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

// Instance rules: objects created from the messages, their facts, and when they go

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "helpers.hpp"

TEST(instances, an_object_is_created_before_its_facts)
{
  auto e = engine({contacts()});
  const auto changes = e->on_message("/sonar", sonar({{7, "wp3"}}), 1.0);
  EXPECT_EQ(kinds(changes), (Kinds{
    {"instance", "contact_7", S("contact")},
    {"predicate", "(contact_at contact_7 wp3)", true},
    {"predicate", "(detected contact_7)", true}}));
  EXPECT_TRUE(contains(changes[0].reason, "contact_7 - contact seen"));
  EXPECT_TRUE(e->on_message("/sonar", sonar({{7, "wp3"}}), 2.0).empty());
}

TEST(instances, their_facts_follow_the_messages)
{
  auto e = engine({contacts()});
  e->on_message("/sonar", sonar({{7, "wp3"}}), 1.0);
  // It moved: the old fact goes before the new one holds
  EXPECT_EQ(facts(e->on_message("/sonar", sonar({{7, "wp4"}}), 2.0)), (Facts{
    {"(contact_at contact_7 wp3)", false}, {"(contact_at contact_7 wp4)", true}}));
  // Not in a message is not gone: only expire_after removes it
  EXPECT_EQ(e->on_message("/sonar", sonar({{8, "wp1"}}), 3.0)[0].fact, "contact_8");
  EXPECT_EQ(status(*e, 0).at("instances").as_map().count("contact_7"), 1u);
}

TEST(instances, one_not_seen_expires_with_its_facts)
{
  auto e = engine({contacts()});
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  e->on_message("/sonar", sonar({{8, "wp1"}}), 5.0);
  EXPECT_TRUE(e->on_timer(9.9).empty());
  const auto changes = e->on_timer(10.0);
  EXPECT_EQ(kinds(changes), (Kinds{
    {"predicate", "(contact_at contact_7 wp3)", false},
    {"predicate", "(detected contact_7)", false},
    {"instance", "contact_7", false}}));
  EXPECT_TRUE(contains(changes.back().reason, "not seen for 10.0 s")) << changes.back().reason;
  const auto & left = status(*e, 0).at("instances").as_map();
  ASSERT_EQ(left.size(), 1u);
  EXPECT_EQ(left.begin()->first, "contact_8");
  // Seen again: a new object
  EXPECT_EQ(e->on_message("/sonar", sonar({{7, "wp2"}}), 11.0)[0].kind, "instance");
}

TEST(instances, one_in_use_is_kept_until_it_is_not)
{
  std::set<std::string> used = {"contact_7"};
  std::vector<std::string> asked;
  auto in_use = [&](const std::string & name) {
      asked.push_back(name);
      return used.count(name) > 0;
    };
  auto e = engine({contacts()}, {}, 0.0, in_use);
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  EXPECT_TRUE(e->on_timer(10.0).empty());
  EXPECT_EQ(
    status(*e, 0).at("instances").as_map().at("contact_7").as_map().at("held"), su::Value(true));
  EXPECT_TRUE(e->on_timer(10.5).empty());  // Asked once a second, not every tick
  EXPECT_EQ(asked, (std::vector<std::string>{"contact_7"}));
  used.clear();
  const auto changes = e->on_timer(11.0);
  ASSERT_FALSE(changes.empty());
  EXPECT_EQ(changes.back().fact, "contact_7");
  EXPECT_EQ(changes.back().value, su::FactValue(false));
  EXPECT_TRUE(contains(changes.back().reason, "no longer used by the plan or the goal"));
}

TEST(instances, a_creation_that_fails_is_tried_again_with_its_facts)
{
  auto e = engine({contacts()});
  for (const auto & change : e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0)) {
    e->write_failed(change, 0.0);
  }
  EXPECT_TRUE(e->on_message("/sonar", sonar({{7, "wp3"}}), 1.0).empty());  // Waits 5 s
  std::vector<std::string> again;
  for (const auto & change : e->on_message("/sonar", sonar({{7, "wp3"}}), 5.0)) {
    again.push_back(change.fact);
  }
  EXPECT_EQ(again, (std::vector<std::string>{
    "contact_7", "(contact_at contact_7 wp3)", "(detected contact_7)"}));
}

TEST(instances, resync_creates_again_what_left_the_problem)
{
  auto e = engine({contacts()});
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  const std::set<std::string> holding = {"(contact_at contact_7 wp3)", "(detected contact_7)"};
  const std::set<std::string> objects = {"contact_7"};
  EXPECT_TRUE(e->resync(problem(holding, {}, objects), 2.0).empty());
  EXPECT_EQ(facts(e->resync(problem({"(detected contact_7)"}, {}, objects), 2.0)),
    (Facts{{"(contact_at contact_7 wp3)", true}}));
  // The object gone, its facts with it (cleared problem)
  const auto changes = e->resync(problem({}, {}, std::set<std::string>{}), 4.0);
  EXPECT_EQ(kinds(changes), (Kinds{
    {"instance", "contact_7", S("contact")},
    {"predicate", "(contact_at contact_7 wp3)", true},
    {"predicate", "(detected contact_7)", true}}));
}

TEST(instances, one_object_per_message_without_wildcard)
{
  auto e = engine({
    "{name: targets, topic: /sonar, target: {instance: {name: 'target_{id}', type: contact}}, "
    "policy: {expire_after: 10.0}}"});
  EXPECT_EQ(e->on_message("/sonar", msg("{id: 3, waypoint: wp1}"), 1.0)[0].fact, "target_3");
}

TEST(instances, names_from_the_messages_are_checked)
{
  auto e = engine({contacts()});
  EXPECT_TRUE(e->on_message("/sonar", sonar({{su::Value("7 b"), "wp3"}}), 1.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "not a valid PDDL name"));
  // A message without the list: an error of the rule, not of the node
  EXPECT_TRUE(e->on_message("/sonar", msg("{other: 1}"), 2.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "contacts"));
}

TEST(instances, without_expire_after_a_warning)
{
  auto e = engine({contacts("{}")});
  EXPECT_TRUE(contains(e->rules()[0]->warnings.at(0), "never removed"));
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  EXPECT_TRUE(e->on_timer(1000.0).empty());
}

TEST(instances, max_instances_caps_how_many_there_are)
{
  auto e = engine({contacts("{expire_after: 10.0, max_instances: 2}")});
  std::vector<std::string> created;
  for (const auto & change : e->on_message("/sonar", sonar({{1, "wp1"}, {2, "wp1"}, {3, "wp1"}}),
    0.0))
  {
    if (change.kind == "instance") {
      created.push_back(change.fact);
    }
  }
  EXPECT_EQ(created, (std::vector<std::string>{"contact_1", "contact_2"}));
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "max_instances 2: not created contact_3"));
  e->on_message("/sonar", sonar({{2, "wp1"}}), 5.0);
  e->on_timer(10.0);
  EXPECT_EQ(e->on_message("/sonar", sonar({{3, "wp1"}}), 11.0).at(0).fact, "contact_3");
}

TEST(instances, remove_when_missing_removes_what_the_message_does_not_have)
{
  std::set<std::string> used;
  auto e = engine({contacts("{remove_when_missing: true}")}, {}, 0.0,
      [&used](const std::string & name) {return used.count(name) > 0;});
  EXPECT_TRUE(e->rules()[0]->warnings.empty());
  e->on_message("/sonar", sonar({{7, "wp3"}, {8, "wp1"}}), 0.0);
  const auto changes = e->on_message("/sonar", sonar({{8, "wp1"}}), 1.0);
  EXPECT_EQ(kinds(changes), (Kinds{
    {"predicate", "(contact_at contact_7 wp3)", false},
    {"predicate", "(detected contact_7)", false},
    {"instance", "contact_7", false}}));
  EXPECT_TRUE(contains(changes.back().reason, "not in the last message")) << changes.back().reason;
  // One the plan uses waits until it does not
  used = {"contact_8"};
  EXPECT_TRUE(e->on_message("/sonar", sonar({}), 2.0).empty());
  used.clear();
  EXPECT_EQ(e->on_timer(3.0).back().fact, "contact_8");
}

TEST(instances, the_missing_ones_make_room_for_the_new_ones)
{
  auto e = engine({contacts("{remove_when_missing: true, max_instances: 2}")});
  e->on_message("/sonar", sonar({{1, "wp1"}, {2, "wp1"}, {3, "wp1"}}), 0.0);
  std::vector<std::string> instances;
  for (const auto & change : e->on_message("/sonar", sonar({{3, "wp1"}}), 1.0)) {
    if (change.kind == "instance") {
      instances.push_back(change.fact);
    }
  }
  EXPECT_EQ(instances, (std::vector<std::string>{"contact_1", "contact_2", "contact_3"}));
}

TEST(instances, remove_when_missing_keeps_one_in_the_message_that_had_expired)
{
  auto e = engine({contacts("{remove_when_missing: true, expire_after: 1.0}")});
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  // Seen again before the timer took it away: it stays
  EXPECT_TRUE(e->on_message("/sonar", sonar({{7, "wp3"}}), 5.0).empty());
}

TEST(instances, remove_when_missing_with_a_bad_element_removes_nothing)
{
  auto e = engine({contacts("{remove_when_missing: true}")});
  e->on_message("/sonar", sonar({{7, "wp3"}}), 0.0);
  // contact_7 may be the one that cannot be read
  EXPECT_TRUE(e->on_message("/sonar", sonar({{su::Value("7 b"), "wp3"}}), 1.0).empty());
  EXPECT_EQ(status(*e, 0).at("instances").as_map().count("contact_7"), 1u);
}

TEST(instances, the_type_from_the_message)
{
  auto e = engine({
    "{name: contacts, topic: /sonar, field: 'contacts[*]', "
    "target: {instance: {name: 'contact_{id}', type: '{class}'}}, policy: {expire_after: 10.0}}"});
  auto rule = dynamic_cast<su::InstanceRule *>(e->rules()[0].get());
  rule->types = std::set<std::string>{"diver", "vessel"};
  auto contact = [](int id, const std::string & type) {
      return su::Value(su::Value::Map{{"id", su::Value(id)}, {"class", su::Value(type)}});
    };
  auto message = [](su::Value::List list) {
      return su::Value(su::Value::Map{{"contacts", su::Value(list)}});
    };
  EXPECT_EQ(kinds(e->on_message("/sonar", message({contact(1, "diver"), contact(2, "vessel")}),
    0.0)), (Kinds{{"instance", "contact_1", S("diver")}, {"instance", "contact_2", S("vessel")}}));
  // Not a type of the domain: not created
  EXPECT_TRUE(e->on_message("/sonar", message({contact(3, "whale")}), 1.0).empty());
  EXPECT_TRUE(
    contains(status(*e, 0).at("error").text(), "'whale' of contact_3 is not in the domain"));
  // Another type for one already created: it stays as it is
  EXPECT_TRUE(e->on_message("/sonar", message({contact(1, "vessel")}), 2.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "contact_1 is a diver"));
  EXPECT_EQ(
    status(*e, 0).at("instances").as_map().at("contact_1").as_map().at("type"),
    su::Value("diver"));
}

TEST(instances, configuration_errors)
{
  const std::string base = "{name: contacts, topic: /sonar, field: 'contacts[*]', ";
  const std::vector<std::pair<std::string, std::string>> cases = {
    {base + "transform: {linear: {}}, target: {instance: {name: 'c_{id}', type: c}}}",
      "takes no transform"},
    {base + "target: {instance: {name: 'c_{id}'}}}", "instance must be"},
    {base + "target: {instance: {name: 'c_{id}', type: 'bad type'}}}", "not a PDDL name"},
    {base + "target: {instance: {name: 'c_{id}', type: c}, facts: ['(at {waypoint})']}}",
      "has its object"},
    {base + "target: {instance: {name: 'c_{id}', type: c}}, "
      "policy: {expire_after: 10.0, debounce: 1.0}}", "unknown policy keys"},
    {base + "target: {instance: {name: 'c_{id}', type: c}}, policy: {expire_after: -1}}",
      "more than 0"},
    {base + "target: {instance: {name: 'c_{id}', type: c}}, policy: {max_instances: 0}}",
      "1 or more"},
    {base + "target: {instance: {name: 'c_{id}', type: c}}, policy: {max_instances: 2.5}}",
      "1 or more"},
    {base + "target: {instance: {name: 'c_{id}', type: c}}, policy: {remove_when_missing: 1}}",
      "true or false"},
    {base + "target: {instance: {name: 'c_{id}', type: '{a[*]}'}}}", "cannot have [*]"},
  };
  for (const auto & [rule, error] : cases) {
    const std::string text = config_error({rule});
    EXPECT_TRUE(contains(text, error)) << rule << "\n  gives: " << text;
  }
  // Their facts have one owner, as any other
  EXPECT_TRUE(contains(
      config_error({contacts(), "{name: leak, topic: /leak, field: data, "
        "target: {predicate: '(detected x)'}}"}), "one owner"));
}
