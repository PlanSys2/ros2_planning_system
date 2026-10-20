# Copyright 2026 Intelligent Robotics Lab
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

"""
Both bringups start with model_file only (#396), and load problem_file when given.

And with one node per component, once started.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from plansys2_msgs.srv import GetProblemInstances
import pytest
import rclpy

PDDL_DIR = os.path.join(get_package_share_directory('plansys2_popf_plan_solver'), 'pddl')
NODES = ['planner', 'domain_expert', 'problem_expert', 'executor']


@pytest.mark.launch_test
@launch_testing.parametrize('bringup, with_problem', [
    ('distributed', False),
    ('distributed', True),
    ('monolithic', False),
    ('monolithic', True),
])
def generate_test_description(bringup, with_problem):
    arguments = {'model_file': os.path.join(PDDL_DIR, 'domain_simple.pddl')}
    if with_problem:
        arguments['problem_file'] = os.path.join(PDDL_DIR, 'problem_simple_1.pddl')

    plansys2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_' + bringup + '.py')),
        launch_arguments=arguments.items())

    return launch.LaunchDescription([plansys2, launch_testing.actions.ReadyToTest()])


class TestBringupProblemFile(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_bringup_problem_file')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, srv_type, name, request, timeout=2.0):
        client = self.node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def test_starts_and_loads_the_problem_if_given(self, bringup, with_problem):
        deadline = time.monotonic() + 60.0
        for name in NODES:
            state = None
            while time.monotonic() < deadline:
                answer = self.call(GetState, '/' + name + '/get_state', GetState.Request())
                state = answer.current_state.id if answer else None
                if state == State.PRIMARY_STATE_ACTIVE:
                    break
                time.sleep(0.2)
            self.assertEqual(state, State.PRIMARY_STATE_ACTIVE, f'{bringup}: {name} not active')

        answer = self.call(
            GetProblemInstances, '/problem_expert/get_problem_instances',
            GetProblemInstances.Request())
        self.assertIsNotNone(answer)
        names = {instance.name for instance in answer.instances}
        if with_problem:
            self.assertIn('leia', names)
        else:
            self.assertEqual(names, set())

    def node_names(self):
        names = {name for name, _ in self.node.get_node_names_and_namespaces()}
        names.discard(self.node.get_name())
        return names

    def test_one_node_per_component(self, bringup, with_problem):
        # The distributed one has its lifecycle_manager too: it watches the four once active
        expected = set(NODES) | ({'lifecycle_manager'} if bringup == 'distributed' else set())
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline and self.node_names() != expected:
            time.sleep(0.2)
        self.assertEqual(self.node_names(), expected, bringup)
        # And no other appears later
        time.sleep(2.0)
        self.assertEqual(self.node_names(), expected, bringup)
