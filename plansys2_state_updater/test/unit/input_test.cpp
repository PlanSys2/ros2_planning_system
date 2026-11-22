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

// Input: sample_period, message stamps, clocks that go back, and previous values

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "helpers.hpp"

namespace
{

su::Value stamped(double stamp, bool data)
{
  const auto sec = static_cast<int64_t>(stamp);
  const auto nanosec = static_cast<int64_t>((stamp - sec) * 1e9 + 0.5);
  return su::Value(su::Value::Map{
      {"header", su::Value(su::Value::Map{{"stamp", su::Value(su::Value::Map{
              {"sec", su::Value(sec)}, {"nanosec", su::Value(nanosec)}})}})},
      {"data", su::Value(data)}});
}

}  // namespace

TEST(sample_period, the_messages_between_are_not_even_read)
{
  auto e = engine({battery("{sample_period: 1.0}")});
  int reads = 0;
  auto reading = [&reads](double percentage) {
      return [&reads, percentage]() {
               reads++;
               return msg("{percentage: " + std::to_string(percentage) + "}");
             };
    };
  EXPECT_EQ(facts(e->receive("/battery", reading(0.9), 0.0)), (Facts{{"(energy)", 90.0}}));
  EXPECT_TRUE(e->receive("/battery", reading(0.5), 0.5).empty());
  EXPECT_EQ(reads, 1);
  EXPECT_EQ(facts(e->receive("/battery", reading(0.5), 1.0)), (Facts{{"(energy)", 50.0}}));
  EXPECT_EQ(reads, 2);
}

TEST(sample_period, each_rule_on_its_own)
{
  auto e = engine({
    battery("{sample_period: 10.0}"),
    "{name: low, topic: /battery, field: percentage, "
    "transform: {compare: {op: '<', value: 0.2}}, target: {predicate: '(low_battery)'}}"});
  e->on_message("/battery", msg("{percentage: 0.9}"), 0.0);
  // The one without sample_period reads them all
  EXPECT_EQ(facts(e->on_message("/battery", msg("{percentage: 0.1}"), 1.0)),
    (Facts{{"(low_battery)", true}}));
}

TEST(sample_period, a_message_not_read_still_says_that_the_sensor_is_alive)
{
  auto e = engine({
    leak("{sample_period: 2.0, stale_after: 3.0, on_stale: '(sensor_stale leak)'}")});
  e->on_message("/leak", msg("{data: false}"), 0.0);
  for (double t = 0.5; t < 2.0; t += 0.5) {
    e->on_message("/leak", msg("{data: false}"), t);
  }
  EXPECT_TRUE(e->on_timer(4.0).empty());  // Not stale: 1.5 s ago a message came
  EXPECT_EQ(facts(e->on_timer(4.5)), (Facts{{"(sensor_stale leak)", true}}));
}

TEST(sample_period, a_read_that_fails_changes_nothing)
{
  auto e = engine({battery("{sample_period: 1.0}")});
  EXPECT_THROW(
    e->receive("/battery", []() -> su::Value {throw std::runtime_error("bad bytes");}, 0.0),
    std::runtime_error);
  EXPECT_FALSE(e->on_message("/battery", msg("{percentage: 0.9}"), 0.0).empty());
}

TEST(stamp, an_old_reading_is_not_taken)
{
  auto e = engine({leak("{stamp: header.stamp, stale_after: 2.0}")});
  EXPECT_TRUE(e->on_message("/leak", stamped(7.0, true), 10.0).empty());  // 3 s old
  EXPECT_EQ(status(*e, 0).at("old_messages"), su::Value(1));
  EXPECT_EQ(facts(e->on_message("/leak", stamped(9.5, true), 10.0)),
    (Facts{{"(leak_detected)", true}}));
  // One made before the last one taken came late: what it says is older
  EXPECT_TRUE(e->on_message("/leak", stamped(9.0, false), 10.2).empty());
  EXPECT_EQ(status(*e, 0).at("old_messages"), su::Value(2));
}

TEST(stamp, stale_counts_from_the_reading)
{
  auto e = engine({leak("{stamp: header.stamp, stale_after: 2.0, "
        "on_stale: '(sensor_stale leak)'}")});
  e->on_message("/leak", stamped(9.0, false), 10.0);  // Came 1 s late
  EXPECT_TRUE(e->on_timer(10.9).empty());
  EXPECT_EQ(facts(e->on_timer(11.0)), (Facts{{"(sensor_stale leak)", true}}));
  // A reading from the future (another clock) counts as made when it came
  EXPECT_EQ(facts(e->on_message("/leak", stamped(50.0, false), 12.0)),
    (Facts{{"(sensor_stale leak)", false}}));
  EXPECT_TRUE(e->on_timer(13.9).empty());
}

TEST(stamp, per_fact_of_a_template)
{
  auto e = engine({diagnostics("{stamp: header.stamp, stale_after: 2.0, "
        "on_stale: '(sensor_stale {name})'}")});
  auto message = status_msg({{"imu", 0}});
  auto map = message.as_map();
  map["header"] = stamped(9.0, false).as_map().at("header");
  e->on_message("/diagnostics", su::Value(map), 10.0);
  EXPECT_EQ(facts(e->on_timer(11.0)), (Facts{{"(sensor_stale imu)", true}}));
}

TEST(stamp, an_old_reading_says_why)
{
  auto e = engine({leak("{stamp: header.stamp, stale_after: 2.0}")}, {}, 100.0);
  e->on_message("/leak", stamped(0.0, true), 100.0);  // A driver that leaves the stamp at 0
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "100.0 s old, by its stamp")) <<
    status(*e, 0).at("error").text();
  e->on_message("/leak", stamped(100.0, true), 100.0);
  EXPECT_TRUE(status(*e, 0).at("error").is_null());
}

TEST(stamp, a_message_without_it_is_an_error_of_the_rule)
{
  auto e = engine({leak("{stamp: header.stamp, stale_after: 2.0}")});
  EXPECT_TRUE(e->on_message("/leak", msg("{data: true}"), 1.0).empty());
  EXPECT_TRUE(contains(status(*e, 0).at("error").text(), "header.stamp")) <<
    status(*e, 0).at("error").text();
}

TEST(stamp, in_seconds)
{
  auto e = engine({leak("{stamp: t, stale_after: 2.0}")});
  EXPECT_EQ(facts(e->on_message("/leak", msg("{t: 9.5, data: true}"), 10.0)),
    (Facts{{"(leak_detected)", true}}));
}

// A clock that goes back (a simulation or a bag restarting): the times kept go back as much

TEST(clock, sample_period_after_it_goes_back)
{
  auto e = engine({battery("{sample_period: 1.0}")}, {}, 100.0);
  e->on_message("/battery", msg("{percentage: 0.9}"), 100.0);
  // Read just before it went back: a period from then, not at 101
  EXPECT_TRUE(e->on_message("/battery", msg("{percentage: 0.5}"), 5.0).empty());
  EXPECT_FALSE(e->on_message("/battery", msg("{percentage: 0.5}"), 6.0).empty());
}

TEST(clock, stamps_after_it_goes_back)
{
  auto e = engine({leak("{stamp: header.stamp, stale_after: 2.0}")}, {}, 100.0);
  e->on_message("/leak", stamped(100.0, true), 100.0);
  EXPECT_FALSE(e->on_message("/leak", stamped(1.0, false), 1.0).empty());
}

TEST(clock, stale_after_it_goes_back)
{
  auto e = engine({leak("{stale_after: 2.0, on_stale: '(sensor_stale leak)'}")}, {}, 100.0);
  e->on_message("/leak", msg("{data: false}"), 100.0);
  e->on_message("/leak", msg("{data: false}"), 1.0);
  EXPECT_TRUE(e->on_timer(2.9).empty());
  EXPECT_EQ(facts(e->on_timer(3.0)), (Facts{{"(sensor_stale leak)", true}}));
}

TEST(clock, debounce_after_it_goes_back)
{
  auto e = engine({leak("{debounce: 1.0}")}, {}, 100.0);
  e->on_message("/leak", msg("{data: true}"), 100.0);
  // Half of its debounce before the clock went back: half after it
  e->on_message("/leak", msg("{data: true}"), 100.5);
  EXPECT_TRUE(e->on_timer(1.0).empty());
  EXPECT_EQ(facts(e->on_timer(1.5)), (Facts{{"(leak_detected)", true}}));
}

TEST(clock, expire_after_it_goes_back)
{
  auto e = engine({contacts()}, {}, 100.0);
  e->on_message("/sonar", sonar({{7, "wp3"}}), 100.0);
  e->on_message("/sonar", sonar({{7, "wp3"}}), 1.0);
  EXPECT_TRUE(e->on_timer(10.9).empty());
  EXPECT_FALSE(e->on_timer(11.0).empty());
}

TEST(clock, a_reload_after_it_goes_back)
{
  auto previous = engine({leak("{debounce: 1.0}")}, {}, 100.0);
  previous->on_message("/leak", msg("{data: true}"), 100.0);
  auto e = engine({leak("{debounce: 1.0}")}, {}, 0.0);
  EXPECT_EQ(e->take_over(*previous, 1.0), (std::set<std::string>{"leak"}));
  EXPECT_TRUE(e->on_timer(1.5).empty());
  EXPECT_EQ(facts(e->on_timer(2.0)), (Facts{{"(leak_detected)", true}}));
}

TEST(input, configuration_errors)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {leak("{stamp: header.stamp}"), "stamp needs stale_after"},
    {leak("{stamp: 'status[*].stamp', stale_after: 1.0}"), "without [*]"},
    {leak("{sample_period: -1}"), "sample_period is a number of seconds"},
    {leak("{sample_period: often}"), "sample_period is a number of seconds"},
    {leak("{sample_period: 2.0, stale_after: 2.0}"), "longer than sample_period"},
    {contacts("{stamp: header.stamp}"), "stamp needs expire_after"},
    {contacts("{sample_period: 5.0, expire_after: 1.0}"), "longer than sample_period"},
  };
  for (const auto & [rule, error] : cases) {
    const std::string text = config_error({rule});
    EXPECT_TRUE(contains(text, error)) << rule << "\n  gives: " << text;
  }
}

TEST(previous, what_was_written)
{
  auto e = engine({battery("{min_change: 2.0}"), leak()});
  const auto first = e->on_message("/battery", msg("{percentage: 0.9}"), 0.0);
  EXPECT_EQ(first.at(0).previous, su::FactValue());  // Unknown
  EXPECT_EQ(e->on_message("/battery", msg("{percentage: 0.8}"), 1.0).at(0).previous,
    su::FactValue(90.0));
  EXPECT_EQ(e->on_message("/leak", msg("{data: true}"), 1.0).at(0).previous, su::FactValue());
  EXPECT_EQ(e->on_message("/leak", msg("{data: false}"), 2.0).at(0).previous,
    su::FactValue(true));
}

TEST(previous, in_mode_correct_what_the_problem_had)
{
  std::map<std::string, double> values = {{"(energy)", 70.0}};
  auto e = engine({battery("{mode: correct, min_change: 2.0}")},
      [&values](const std::string & fact) {return values.at(fact);});
  EXPECT_EQ(e->on_message("/battery", msg("{percentage: 0.9}"), 0.0).at(0).previous,
    su::FactValue(70.0));
}

TEST(previous, of_a_resync_what_it_found)
{
  auto e = engine({battery(), leak()});
  e->on_message("/battery", msg("{percentage: 0.8}"), 0.0);
  e->on_message("/leak", msg("{data: true}"), 0.0);
  std::map<std::string, su::FactValue> previous;
  for (const auto & change : e->resync(problem({}, {{"(energy)", 50.0}}), 2.0)) {
    previous[change.fact] = change.previous;
  }
  EXPECT_EQ(previous.at("(energy)"), su::FactValue(50.0));
  EXPECT_EQ(previous.at("(leak_detected)"), su::FactValue(false));
}

TEST(previous, of_stale_and_of_instances)
{
  auto e = engine({leak("{stale_after: 1.0, on_stale: '(sensor_stale leak)'}"), contacts()});
  e->on_message("/leak", msg("{data: false}"), 0.0);
  EXPECT_EQ(e->on_timer(1.0).at(0).previous, su::FactValue(false));
  EXPECT_EQ(e->on_message("/leak", msg("{data: false}"), 2.0).at(0).previous,
    su::FactValue(true));
  const auto created = e->on_message("/sonar", sonar({{7, "wp3"}}), 2.0);
  EXPECT_EQ(created.at(0).previous, su::FactValue());
  EXPECT_EQ(created.at(1).previous, su::FactValue(false));
}
