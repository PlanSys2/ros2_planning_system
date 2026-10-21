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

"""The monolithic plansys2_node starts and keeps serving while the planner works."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from plansys2_msgs.srv import GetDomainName, GetPlan, GetProblem
import pytest
import rclpy

# POPF's test files: problem_hard_unsolvable keeps POPF busy until the timeout
PDDL_DIR = os.path.join(get_package_share_directory('plansys2_popf_plan_solver'), 'pddl')
PLAN_TIMEOUT = 8.0
NODES = ['planner', 'domain_expert', 'problem_expert', 'executor']


@pytest.mark.launch_test
def generate_test_description():
    plansys2 = launch_ros.actions.Node(
        package='plansys2_bringup',
        executable='plansys2_node',
        output='screen',
        parameters=[{
            'model_file': os.path.join(PDDL_DIR, 'domain_simple.pddl'),
            # The domain expert asks the planner node, in the same process
            'validate_using_planner_node': True,
            'plan_solver_plugins': ['POPF'],
            'POPF.plugin': 'plansys2/POPFPlanSolver',
            'plan_solver_timeout': PLAN_TIMEOUT,
        }])

    return (
        launch.LaunchDescription([plansys2, launch_testing.actions.ReadyToTest()]),
        {'plansys2': plansys2})


def read_pddl(name):
    with open(os.path.join(PDDL_DIR, name)) as f:
        return f.read()


class TestMonolithicResponsive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_monolithic_responsive')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, srv_type, name, request, timeout):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), name + ' not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future

    def assert_all_active(self):
        deadline = time.monotonic() + 60.0
        for name in NODES:
            state = None
            while time.monotonic() < deadline:
                future = self.call(GetState, '/' + name + '/get_state', GetState.Request(), 1.0)
                state = future.result().current_state.id if future.done() else None
                if state == State.PRIMARY_STATE_ACTIVE:
                    break
                time.sleep(0.2)
            self.assertEqual(state, State.PRIMARY_STATE_ACTIVE, name + ' is not active')

    def test_1_starts_validating_with_planner_node(self):
        # Used to fail: the domain expert waited for the planner on the only thread
        self.assert_all_active()

    def test_2_serves_while_planning(self):
        self.assert_all_active()

        plan_client = self.node.create_client(GetPlan, '/planner/get_plan')
        self.assertTrue(plan_client.wait_for_service(timeout_sec=10.0))
        plan_future = plan_client.call_async(GetPlan.Request(
            domain=read_pddl('domain_simple.pddl'),
            problem=read_pddl('problem_hard_unsolvable.pddl')))
        rclpy.spin_until_future_complete(self.node, plan_future, timeout_sec=1.0)
        self.assertFalse(plan_future.done(), 'the planner should still be working')

        # The other nodes answer at once, not when the plan ends
        for srv_type, name in [
                (GetDomainName, '/domain_expert/get_domain_name'),
                (GetProblem, '/problem_expert/get_problem'),
                (GetPlan, '/executor/get_plan'),
                (GetState, '/executor/get_state')]:
            start = time.monotonic()
            future = self.call(srv_type, name, srv_type.Request(), 3.0)
            self.assertTrue(future.done(), name + ' blocked while planning')
            self.assertLess(time.monotonic() - start, 3.0)
        self.assertFalse(plan_future.done(), 'answers must arrive during the planning')

        rclpy.spin_until_future_complete(self.node, plan_future, timeout_sec=PLAN_TIMEOUT + 10.0)
        self.assertTrue(plan_future.done())
        self.assertFalse(plan_future.result().success)
        self.assertIn('timed out', plan_future.result().error_info)

    def test_3_plans_after_a_timeout(self):
        self.assert_all_active()
        future = self.call(
            GetPlan, '/planner/get_plan',
            GetPlan.Request(
                domain=read_pddl('domain_simple.pddl'),
                problem=read_pddl('problem_simple_1.pddl')),
            PLAN_TIMEOUT + 10.0)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success, future.result().error_info)
        self.assertGreater(len(future.result().plan.items), 0)


@launch_testing.post_shutdown_test()
class TestMonolithicShutdown(unittest.TestCase):

    def test_clean_exit(self, proc_info, plansys2):
        launch_testing.asserts.assertExitCodes(proc_info, process=plansys2)
