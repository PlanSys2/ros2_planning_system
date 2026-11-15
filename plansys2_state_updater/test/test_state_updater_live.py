# Copyright 2026 Juan Carlos Manzanares Serrano
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The rules updater in a running PlanSys2; each test reloads its own rules."""

import json
import os
import signal
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from plansys2_msgs.msg import StateUpdate
from plansys2_support_py.DomainExpertClient import DomainExpertClient
from plansys2_support_py.Parser import Parser
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient
import pytest
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(os.path.dirname(TEST_DIR), 'config')
DOMAIN = os.path.join(CONFIG_DIR, 'example_domain.pddl')
PROBLEM = os.path.join(CONFIG_DIR, 'example_problem.pddl')

# Relative topics: in the namespace of the test, nothing else running on this host mixes
RULES = """
state_updater:
  rules:
    - name: battery
      topic: robot/battery
      type: sensor_msgs/msg/BatteryState
      field: percentage
      transform: {linear: {scale: 100.0, round: 1}}
      target: {function: "(energy)"}
      policy: {mode: correct, min_change: 2.0}
    - name: sensors
      topic: diagnostics
      type: diagnostic_msgs/msg/DiagnosticArray
      field: "status[*]"
      transform: {compare: {field: level, op: "==", value: 0}}
      target: {predicate: "(sensor_ok {name})"}
      policy: {debounce: 0.5}
"""


def wait_until(condition, timeout, node=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if node is not None:
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            time.sleep(0.1)
        if condition():
            return True
    return False


class PlanSys2:
    """PlanSys2 with the rules updater, in a namespace of its own."""

    def __init__(self, launch_file, namespace, rules_file):
        self.namespace = namespace
        self.rules_file = rules_file
        self.launch = subprocess.Popen(
            ['ros2', 'launch', 'plansys2_bringup', launch_file, f'model_file:={DOMAIN}',
             f'problem_file:={PROBLEM}', f'namespace:={namespace}',
             f'state_updater_config:={rules_file}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        self.node = rclpy.create_node('pytest_state_updater', namespace=namespace)
        self.problem = ProblemExpertClient(self.node)
        self.events = []
        self.status = []
        self.node.create_subscription(
            StateUpdate, 'state_updater/events', self.events.append, 100)
        self.node.create_subscription(String, 'state_updater/status', self.status.append, 10)
        self.battery = self.node.create_publisher(BatteryState, 'robot/battery', 10)
        self.diagnostics = self.node.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.reload_client = self.node.create_client(Trigger, 'state_updater/reload')
        self.parameters_client = self.node.create_client(
            SetParameters, 'problem_expert/set_parameters')

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=30.0), client.srv_name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)
        assert future.done(), client.srv_name
        return future.result()

    def ready(self):
        """Whether the problem expert loaded the rules updater, and it is running."""
        return self.reload_client.service_is_ready() and bool(self.status)

    def use_rules(self, rules):
        """Load these rules: the answer of state_updater/reload."""
        with open(self.rules_file, 'w') as f:
            f.write(rules)
        self.events.clear()
        response = self.call(self.reload_client, Trigger.Request())
        # Its first writes are fast: the subscriptions are matched before the test publishes
        wait_until(lambda: False, 0.5, self.node)
        return response

    def set_parameter(self, name, value, type_):
        request = SetParameters.Request(
            parameters=[Parameter(f'state_updater.{name}', type_, value).to_parameter_msg()])
        return self.call(self.parameters_client, request).results[0]

    def energy(self):
        function = self.problem.get_problem_function('(energy)')
        return function.value if function is not None else None

    def holds(self, fact):
        return bool(self.problem.exist_problem_predicate(Parser.node_from_string_predicate(fact)))

    def publish_until(self, publisher, msg, condition, timeout=10.0):
        def step():
            publisher.publish(msg)
            return condition()
        return wait_until(step, timeout, self.node)

    def close(self):
        self.problem.destroy()
        self.node.destroy_node()
        os.killpg(self.launch.pid, signal.SIGINT)
        try:
            self.launch.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.launch.pid, signal.SIGKILL)
            self.launch.wait()


@pytest.fixture(scope='module')
def rules_file(tmp_path_factory):
    path = tmp_path_factory.mktemp('state_updater') / 'rules.yaml'
    path.write_text(RULES)
    return str(path)


@pytest.fixture(scope='module')
def monolithic(rules_file):
    if not rclpy.ok():
        rclpy.init()
    plansys2 = PlanSys2(
        'plansys2_bringup_launch_monolithic.py', f'su_pytest_{os.getpid()}', rules_file)
    try:
        assert wait_until(plansys2.ready, 60.0, plansys2.node), 'the rules updater is not running'
        yield plansys2
    finally:
        plansys2.close()


def test_the_problem_expert_loads_the_rules(monolithic):
    status = json.loads(monolithic.status[-1].data)
    assert [rule['rule'] for rule in status] == ['battery', 'sensors']


def test_the_battery_corrects_the_energy(monolithic):
    assert monolithic.energy() == pytest.approx(100.0)
    assert monolithic.publish_until(
        monolithic.battery, BatteryState(percentage=0.5),
        lambda: monolithic.energy() == pytest.approx(50.0))
    assert wait_until(lambda: any(e.rule == 'battery' for e in monolithic.events), 5.0,
                      monolithic.node)
    event = next(e for e in monolithic.events if e.rule == 'battery')
    assert event.kind == StateUpdate.FUNCTION
    assert event.fact == '(energy)'
    assert event.value == pytest.approx(50.0)
    assert event.written
    assert '100 → 50' in event.reason
    # What the problem had before: mode correct compares with it
    assert event.has_previous and event.previous_value == pytest.approx(100.0)


def test_a_template_writes_a_fact_per_sensor(monolithic):
    msg = DiagnosticArray(status=[DiagnosticStatus(name='imu', level=b'\x00'),
                                  DiagnosticStatus(name='dvl', level=b'\x02')])
    assert monolithic.publish_until(
        monolithic.diagnostics, msg, lambda: monolithic.holds('(sensor_ok imu)'))
    assert not monolithic.holds('(sensor_ok dvl)')
    wait_until(lambda: False, 0.5, monolithic.node)  # The events still in flight
    events = {e.fact: e for e in monolithic.events if e.rule == 'sensors'}
    assert events['(sensor_ok imu)'].kind == StateUpdate.PREDICATE
    assert events['(sensor_ok imu)'].holds
    assert not events['(sensor_ok dvl)'].holds


def test_what_the_problem_loses_is_written_again(monolithic):
    # Written again after resync_period
    msg = DiagnosticArray(status=[DiagnosticStatus(name='dvl', level=b'\x00')])
    assert monolithic.publish_until(
        monolithic.diagnostics, msg, lambda: monolithic.holds('(sensor_ok dvl)'), 15.0)
    assert monolithic.problem.remove_problem_predicate(
        Parser.node_from_string_predicate('(sensor_ok dvl)'))
    assert wait_until(lambda: monolithic.holds('(sensor_ok dvl)'), 10.0, monolithic.node)
    assert any(e.fact == '(sensor_ok dvl)' and 'resync' in e.reason for e in monolithic.events)


def test_dry_run_publishes_and_writes_nothing(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: leak
    topic: dry/leak
    type: std_msgs/msg/Bool
    field: data
    target: {predicate: "(leak_detected)"}
""").success
    assert monolithic.set_parameter('dry_run', True, Parameter.Type.BOOL).successful
    try:
        publisher = monolithic.node.create_publisher(Bool, 'dry/leak', 10)
        assert monolithic.publish_until(publisher, Bool(data=True), lambda: monolithic.events)
        event = monolithic.events[0]
        assert event.fact == '(leak_detected)' and event.holds and not event.written
        assert not monolithic.holds('(leak_detected)')

        # Once a second: one after the message has it
        def leak_in_status():
            return any(json.loads(m.data)[0]['facts'].get('(leak_detected)', {}).get('value')
                       for m in monolithic.status)
        assert wait_until(leak_in_status, 5.0, monolithic.node)
    finally:
        assert monolithic.set_parameter('dry_run', False, Parameter.Type.BOOL).successful


def test_the_type_is_found_in_the_graph(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: low_battery
    topic: graph/battery
    field: percentage
    transform: {hysteresis: {on_below: 0.25, off_above: 0.30}}
    target: {predicate: "(low_battery)"}
""").success
    publisher = monolithic.node.create_publisher(BatteryState, 'graph/battery', 10)
    assert monolithic.publish_until(
        publisher, BatteryState(percentage=0.1), lambda: monolithic.holds('(low_battery)'), 15.0)


@pytest.mark.parametrize('rules, error', [
    ("""
rules:
  - name: depth
    topic: bad/depth
    type: std_msgs/msg/Bool
    field: data
    target: {predicate: "(too_deep)"}
""", "'too_deep' is not in the domain"),
    ("""
rules:
  - name: stale
    topic: bad/depth
    type: std_msgs/msg/Bool
    field: data
    target: {predicate: "(sensor_stale)"}
""", 'has 0 parameters'),
    # inspect spends (energy) with #t: written as is, it would fight the executor
    ("""
rules:
  - name: battery
    topic: fight/battery
    type: sensor_msgs/msg/BatteryState
    field: percentage
    target: {function: "(energy)"}
""", 'is changed by the actions of the plan'),
])
def test_rules_that_do_not_fit_are_refused(monolithic, rules, error):
    response = monolithic.use_rules(rules)
    assert not response.success
    assert 'stay as they were' in response.message and error in response.message, \
        response.message


def test_an_enum_holds_one_mode_at_a_time(monolithic):
    # The problem file starts with (mode_is normal)
    assert monolithic.holds('(mode_is normal)')
    assert monolithic.use_rules("""
rules:
  - name: mode
    topic: enum/mode
    type: std_msgs/msg/Int32
    field: data
    transform: {map: {values: {2: normal, 3: degraded}, default: lost}}
    target: {enum: "(mode_is {})", values: [normal, degraded, lost]}
""").success
    publisher = monolithic.node.create_publisher(Int32, 'enum/mode', 10)
    assert monolithic.publish_until(
        publisher, Int32(data=3), lambda: monolithic.holds('(mode_is degraded)'))
    assert not monolithic.holds('(mode_is normal)')
    assert not monolithic.holds('(mode_is lost)')
    # The one that held went before the new one held
    assert wait_until(lambda: len(monolithic.events) >= 3, 5.0, monolithic.node)
    order = [(e.fact, e.holds) for e in monolithic.events]
    assert order.index(('(mode_is normal)', False)) < order.index(('(mode_is degraded)', True))


def test_the_changes_of_a_cycle_are_one_notification(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: reachable
    topic: batch/reachable
    type: rcl_interfaces/msg/ParameterValue
    field: string_array_value
    target: {set: "(reachable {})"}
""").success
    notifications = []
    monolithic.node.create_subscription(
        Empty, 'problem_expert/update_notify', notifications.append, 100)
    publisher = monolithic.node.create_publisher(ParameterValue, 'batch/reachable', 10)
    wait_until(lambda: False, 1.0, monolithic.node)
    notifications.clear()
    msg = ParameterValue(string_array_value=['wp1', 'wp2', 'wp3'])
    assert monolithic.publish_until(publisher, msg, lambda: len(monolithic.events) == 3)
    wait_until(lambda: False, 1.0, monolithic.node)
    assert all(e.written for e in monolithic.events)
    assert all(monolithic.holds(f'(reachable {wp})') for wp in ['wp1', 'wp2', 'wp3'])
    assert len(notifications) == 1


def test_instances_are_created_and_expire_unless_the_goal_uses_them(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: contacts
    topic: sonar/contacts
    type: diagnostic_msgs/msg/DiagnosticArray
    field: "status[*]"
    target:
      instance: {name: "contact_{name}", type: contact}
      facts: ["(contact_at {self} {hardware_id})"]
    policy: {expire_after: 1.5}
""").success
    publisher = monolithic.node.create_publisher(DiagnosticArray, 'sonar/contacts', 10)
    msg = DiagnosticArray(status=[DiagnosticStatus(name='7', hardware_id='wp3')])
    assert monolithic.publish_until(
        publisher, msg, lambda: monolithic.holds('(contact_at contact_7 wp3)'))
    assert monolithic.problem.get_problem_instance('contact_7') is not None

    # In the goal: kept, though not seen
    assert monolithic.problem.add_problem_goal(
        Parser.tree_from_string('(and (contact_at contact_7 wp3))'))
    wait_until(lambda: False, 3.0, monolithic.node)
    assert monolithic.holds('(contact_at contact_7 wp3)')
    held = json.loads(monolithic.status[-1].data)[0]['instances']['contact_7']['held']
    assert held is True

    # Out of the goal: removed, with its facts
    assert monolithic.problem.remove_problem_goal()
    assert wait_until(lambda: not monolithic.holds('(contact_at contact_7 wp3)'), 5.0,
                      monolithic.node)
    assert monolithic.problem.get_problem_instance('contact_7') is None

    def instance_events():
        return [(e.kind, e.fact, e.holds) for e in monolithic.events
                if e.kind == StateUpdate.INSTANCE]
    assert wait_until(lambda: len(instance_events()) == 2, 5.0, monolithic.node)
    assert instance_events() == [(StateUpdate.INSTANCE, 'contact_7', True),
                                 (StateUpdate.INSTANCE, 'contact_7', False)]


def test_rules_reloaded_disabled_and_enabled_at_run_time(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: leak
    topic: reload/leak
    type: std_msgs/msg/Bool
    field: data
    target: {predicate: "(leak_detected)"}
""").success
    leak = monolithic.node.create_publisher(Bool, 'reload/leak', 10)
    strings = Parameter.Type.STRING_ARRAY

    def holds():
        return monolithic.holds('(leak_detected)')

    # Disabled: it writes nothing; a name of no rule is refused
    assert not monolithic.set_parameter('disabled_rules', ['nothing'], strings).successful
    assert monolithic.set_parameter('disabled_rules', ['leak'], strings).successful
    assert not monolithic.publish_until(leak, Bool(data=True), holds, 2.0)
    assert monolithic.set_parameter('disabled_rules', [], strings).successful
    assert monolithic.publish_until(leak, Bool(data=True), holds)

    # Rules that are not valid: the old ones stay
    response = monolithic.use_rules('rules:\n  - name: broken\n    topic: x\n')
    assert not response.success and 'stay as they were' in response.message
    assert monolithic.publish_until(leak, Bool(data=False), lambda: not holds())

    # New rules: their topic, not the old one
    response = monolithic.use_rules("""
rules:
  - name: leak2
    topic: reload/leak2
    type: std_msgs/msg/Bool
    field: data
    target: {predicate: "(leak_detected)"}
""")
    assert response.success, response.message
    assert not monolithic.publish_until(leak, Bool(data=True), holds, 2.0)
    leak2 = monolithic.node.create_publisher(Bool, 'reload/leak2', 10)
    assert monolithic.publish_until(leak2, Bool(data=True), holds)
    assert monolithic.publish_until(leak2, Bool(data=False), lambda: not holds())


def test_an_object_fluent_gets_its_object(monolithic):
    # (location) - waypoint: the domain says its value is an object, (= (location) wp2)
    assert monolithic.use_rules("""
rules:
  - name: location
    topic: fluent/waypoint
    type: std_msgs/msg/String
    field: data
    target: {function: "(location)"}
""").success
    publisher = monolithic.node.create_publisher(String, 'fluent/waypoint', 10)

    def location():
        function = monolithic.problem.get_problem_function('(location)')
        return function.object if function is not None else None

    # Only its events: one of the rules of the test before may still be on its way
    def events():
        return [e for e in monolithic.events if e.rule == 'location']
    assert monolithic.publish_until(publisher, String(data='wp2'), lambda: location() == 'wp2')
    assert wait_until(events, 5.0, monolithic.node)
    assert events()[0].kind == StateUpdate.FUNCTION
    assert events()[0].object == 'wp2' and events()[0].written
    # Not an object of the problem: the write fails, and says so
    monolithic.events.clear()
    assert monolithic.publish_until(publisher, String(data='nowhere'), events, 5.0)
    assert not events()[0].written
    assert location() == 'wp2'


def test_sample_period_reads_a_message_a_period(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: low_battery
    topic: sample/battery
    type: sensor_msgs/msg/BatteryState
    field: percentage
    transform: {compare: {op: "<", value: 0.5}}
    target: {predicate: "(low_battery)"}
    policy: {sample_period: 1.0}
""").success
    publisher = monolithic.node.create_publisher(BatteryState, 'sample/battery', 10)
    # Low and high in turns, at 20 Hz: each message read would be a change
    start, i = time.monotonic(), 0
    while time.monotonic() - start < 3.0:
        publisher.publish(BatteryState(percentage=0.1 if i % 2 else 0.9))
        i += 1
        wait_until(lambda: False, 0.05, monolithic.node)
    wait_until(lambda: False, 0.5, monolithic.node)
    changes = [e.reason for e in monolithic.events if e.rule == 'low_battery']
    assert 1 <= len(changes) <= 4, (i, changes)


def test_a_reading_is_as_old_as_its_stamp(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: low_battery
    topic: stamp/battery
    type: sensor_msgs/msg/BatteryState
    field: percentage
    transform: {compare: {op: "<", value: 0.5}}
    target: {predicate: "(low_battery)"}
    policy: {stamp: header.stamp, stale_after: 2.0}
""").success
    publisher = monolithic.node.create_publisher(BatteryState, 'stamp/battery', 10)
    clock = monolithic.node.get_clock()
    # The test before may have left it
    monolithic.problem.remove_problem_predicate(
        Parser.node_from_string_predicate('(low_battery)'))

    def publish(age):
        msg = BatteryState(percentage=0.1)
        msg.header.stamp = (clock.now() - Duration(seconds=age)).to_msg()
        publisher.publish(msg)

    # Made 10 s ago: not what there is now
    for _ in range(20):
        publish(10.0)
        wait_until(lambda: False, 0.1, monolithic.node)
    assert not monolithic.holds('(low_battery)')
    assert not [e for e in monolithic.events if e.rule == 'low_battery']
    assert wait_until(
        lambda: 's old, by its stamp' in str(json.loads(monolithic.status[-1].data)[0]['error']),
        3.0, monolithic.node)

    def fresh():
        publish(0.0)
        return monolithic.holds('(low_battery)')
    assert wait_until(fresh, 10.0, monolithic.node)


def test_instances_of_the_types_the_messages_say(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: contacts
    topic: list/contacts
    type: diagnostic_msgs/msg/DiagnosticArray
    field: "status[*]"
    target:
      instance: {name: "contact_{name}", type: "{hardware_id}"}
      facts: ["(contact_at {self} wp1)"]
    policy: {remove_when_missing: true, max_instances: 2}
""").success
    publisher = monolithic.node.create_publisher(DiagnosticArray, 'list/contacts', 10)

    def instance(name):
        return monolithic.problem.get_problem_instance(name)

    def contacts(*seen):
        return DiagnosticArray(
            status=[DiagnosticStatus(name=name, hardware_id=type_) for name, type_ in seen])

    # Two at most; a whale is not a type of the domain
    assert monolithic.publish_until(
        publisher, contacts(('a', 'contact'), ('b', 'contact'), ('c', 'contact'), ('w', 'whale')),
        lambda: instance('contact_a') is not None and instance('contact_b') is not None)
    assert instance('contact_a').type == 'contact'
    assert monolithic.holds('(contact_at contact_a wp1)')
    wait_until(lambda: False, 1.5, monolithic.node)
    assert instance('contact_c') is None and instance('contact_w') is None
    error = str(json.loads(monolithic.status[-1].data)[0]['error'])
    assert 'max_instances 2' in error and 'whale' in error, error

    # The message is the whole list: the ones not in it go, and make room for c at once
    assert monolithic.publish_until(
        publisher, contacts(('c', 'contact')),
        lambda: instance('contact_c') is not None and instance('contact_a') is None and
        instance('contact_b') is None)
    assert monolithic.holds('(contact_at contact_c wp1)')
    assert not monolithic.holds('(contact_at contact_a wp1)')


def test_rules_that_no_longer_fit_a_new_domain_are_disabled(monolithic):
    assert monolithic.use_rules("""
rules:
  - name: low_battery
    topic: domain/battery
    type: sensor_msgs/msg/BatteryState
    field: percentage
    transform: {hysteresis: {on_below: 0.25, off_above: 0.30}}
    target: {predicate: "(low_battery)"}
""").success
    with open(DOMAIN) as f:
        domain = f.read()
    domain_client = DomainExpertClient(monolithic.node)

    def rule():
        return json.loads(monolithic.status[-1].data)[0] if monolithic.status else {}
    try:
        # A domain without (low_battery): the rule writes nothing, and says why
        assert domain_client.change_domain(domain.replace('    (low_battery)\n', ''))
        assert wait_until(lambda: rule().get('enabled') is False, 10.0, monolithic.node)
        assert 'does not fit the new domain' in rule()['error']
        assert domain_client.change_domain(domain)
        assert wait_until(lambda: rule().get('enabled') is True, 10.0, monolithic.node)
        assert rule()['error'] is None
    finally:
        domain_client.change_domain(domain)
        domain_client.destroy()


def test_the_distributed_bringup_loads_it_too(monolithic, tmp_path):
    # Runs after the monolithic tests: one PlanSys2 at a time
    rules = tmp_path / 'rules.yaml'
    rules.write_text(RULES)
    plansys2 = PlanSys2(
        'plansys2_bringup_launch_distributed.py', f'su_pytest_dist_{os.getpid()}', str(rules))
    try:
        assert wait_until(plansys2.ready, 60.0, plansys2.node)
        assert plansys2.publish_until(
            plansys2.battery, BatteryState(percentage=0.3),
            lambda: plansys2.energy() == pytest.approx(30.0), 15.0)
    finally:
        plansys2.close()
