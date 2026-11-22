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

"""End to end: sensors, through the state updater, against an executing plan."""

import os
import signal
import subprocess
import sys
import threading
import time

from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ExecutionEvent, StateUpdate
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from plansys2_support_py.DomainExpertClient import DomainExpertClient
from plansys2_support_py.ExecutorClient import ExecutorClient
from plansys2_support_py.Parser import Parser
from plansys2_support_py.PlannerClient import PlannerClient
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
# The replanning mission of the examples
sys.path.insert(0, os.path.join(os.path.dirname(TEST_DIR), 'examples'))
from replanning_mission import MissionLoop  # noqa: E402, I100
DOMAIN = os.path.join(TEST_DIR, 'pddl', 'inspect.pddl')
PROBLEM = os.path.join(TEST_DIR, 'pddl', 'inspect_problem.pddl')
NAMESPACE = f'su_mission_{os.getpid()}'

RULES = """
state_updater:
  rules:
    - name: battery
      topic: robot/battery
      type: sensor_msgs/msg/BatteryState
      field: percentage
      transform: {linear: {scale: 100.0, round: 1}}
      target: {function: "(energy)"}
      policy: {mode: correct, min_change: 5.0}
    - name: leak
      topic: robot/leak
      type: std_msgs/msg/Bool
      field: data
      target: {predicate: "(leak_detected)"}
      policy: {debounce: 1.0}
"""


class Inspector(ActionExecutorClient):
    """Performs inspect, which never ends by itself: the plan is cut before."""

    def __init__(self):
        super().__init__(
            'pytest_inspector', 0.2, namespace=NAMESPACE,
            parameter_overrides=[Parameter('action_name', value='inspect')])
        self.works = 0

    def do_work(self):
        self.works += 1
        self.send_feedback(0.1, 'inspecting')


def wait_until(condition, timeout, period=0.1):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(period)
    return condition()


@pytest.fixture(scope='module')
def mission(tmp_path_factory):
    if not rclpy.ok():
        rclpy.init()
    rules = tmp_path_factory.mktemp('mission') / 'rules.yaml'
    rules.write_text(RULES)
    launch = subprocess.Popen(
        ['ros2', 'launch', 'plansys2_bringup', 'plansys2_bringup_launch_monolithic.py',
         f'model_file:={DOMAIN}', f'problem_file:={PROBLEM}', f'namespace:={NAMESPACE}',
         f'state_updater_config:={rules}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    node = rclpy.create_node('pytest_mission', namespace=NAMESPACE)
    events = []
    node.create_subscription(StateUpdate, 'state_updater/events', events.append, 100)
    trace = []
    node.create_subscription(
        ExecutionEvent, 'executor/execution_trace', trace.append,
        QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publishers = {
        'battery': node.create_publisher(BatteryState, 'robot/battery', 10),
        'leak': node.create_publisher(Bool, 'robot/leak', 10),
    }
    inspector = Inspector()
    inspector.trigger_configure()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(inspector)
    spinning = threading.Thread(target=executor.spin, daemon=True)
    spinning.start()
    clients = {
        'domain': DomainExpertClient(node), 'problem': ProblemExpertClient(node),
        'planner': PlannerClient(node), 'executor': ExecutorClient(node),
    }
    try:
        assert wait_until(lambda: clients['problem'].get_problem() is not None, 60.0)
        yield {'node': node, 'events': events, 'trace': trace, 'inspector': inspector,
               **publishers, **clients}
    finally:
        for client in clients.values():
            client.destroy()
        executor.shutdown()
        spinning.join(timeout=5.0)
        inspector.destroy_node()
        node.destroy_node()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()


def energy(mission):
    function = mission['problem'].get_problem_function('(energy)')
    return function.value if function is not None else None


def publish_for(publisher, msg, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        publisher.publish(msg)
        time.sleep(0.1)


def battery_events(mission):
    return [e for e in mission['events'] if e.rule == 'battery']


def test_predict_correct_and_cut_by_over_all(mission):
    # The leak sensor says there is none: written once its debounce passes
    publish_for(mission['leak'], Bool(data=False), 1.5)

    plan = mission['planner'].get_plan(
        mission['domain'].get_domain(), mission['problem'].get_problem())
    assert plan is not None and [i.action for i in plan.items] == ['(inspect a1)']
    assert mission['executor'].start_plan_execution(plan)
    assert wait_until(lambda: mission['inspector'].works > 2, 30.0), 'inspect did not start'

    # The model (#t, 0.5/s) spends energy while inspect runs, with no sensor reading
    assert wait_until(lambda: energy(mission) < 99.0, 10.0), energy(mission)

    # A reading close to the model is not written (min_change 5)
    before = len(battery_events(mission))
    publish_for(mission['battery'], BatteryState(percentage=energy(mission) / 100.0), 1.0)
    assert len(battery_events(mission)) == before

    # A reading far from the model is written, and the model goes on from there
    publish_for(mission['battery'], BatteryState(percentage=0.6), 1.0)
    assert wait_until(lambda: len(battery_events(mission)) > before, 5.0)
    event = battery_events(mission)[-1]
    assert event.written and event.value == pytest.approx(60.0)
    assert 'deviation' in event.reason
    assert wait_until(lambda: energy(mission) < 60.0, 10.0), energy(mission)
    assert energy(mission) > 50.0

    # A leak held for its debounce violates the over all condition of inspect
    publish_for(mission['leak'], Bool(data=True), 2.0)
    assert wait_until(
        lambda: mission['problem'].exist_problem_predicate(
            Parser.node_from_string_predicate('(leak_detected)')), 5.0)
    executor = mission['executor']
    assert wait_until(lambda: not executor.execute_and_check_plan(verbose=False), 15.0)
    assert executor.get_result().result == ExecutePlan.Result.FAILURE
    # The executor reports the leak as the only cause
    failures = [e for e in mission['trace'] if e.type == ExecutionEvent.FAILURE and e.action]
    assert failures, [(e.type, e.action, e.detail) for e in mission['trace']]
    assert failures[0].action.startswith('(inspect a1)')
    assert 'over all' in failures[0].detail and 'leak_detected' in failures[0].detail, \
        failures[0].detail
    works = mission['inspector'].works
    time.sleep(1.0)
    assert mission['inspector'].works == works

    # Undoing inspect keeps the energy that the state updater wrote: not back to 100
    assert energy(mission) < 60.0


def test_the_mission_loop_replans_when_the_world_changes(mission):
    # After the cut: the leak is still there, so no plan reaches the goal, and the loop waits
    assert mission['problem'].exist_problem_predicate(
        Parser.node_from_string_predicate('(leak_detected)'))
    loop = MissionLoop(mission['node'])
    loop.step()
    assert loop.plans == 0 and not loop.executing
    loop.step()  # Nothing changed: it does not ask the planner again
    assert loop.plans == 0

    # The leak goes (its debounce): the state updater says so, and the loop plans again
    works = mission['inspector'].works
    publish_for(mission['leak'], Bool(data=False), 2.0)
    assert wait_until(lambda: loop.world_changed, 5.0)
    loop.step()
    assert loop.plans == 1 and loop.executing
    assert wait_until(lambda: mission['inspector'].works > works + 2, 15.0), 'inspect again'
    assert loop.executor.cancel_plan_execution()  # The loop started it: its client cancels it
