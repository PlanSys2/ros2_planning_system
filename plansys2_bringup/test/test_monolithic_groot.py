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

"""The monolithic bringup gives the executor the Groot2 monitoring of the plan it is asked for."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy

PDDL_DIR = os.path.join(get_package_share_directory('plansys2_popf_plan_solver'), 'pddl')


@pytest.mark.launch_test
def generate_test_description():
    arguments = {'model_file': os.path.join(PDDL_DIR, 'domain_simple.pddl'),
                 'enable_groot_monitoring': 'true', 'groot_port': '1850'}
    plansys2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments=arguments.items())
    return launch.LaunchDescription([plansys2, launch_testing.actions.ReadyToTest()])


class TestMonolithicGroot(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_monolithic_groot')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_the_executor_gets_groot(self):
        client = self.node.create_client(GetParameters, '/executor/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=60.0))
        request = GetParameters.Request()
        request.names = ['enable_groot_monitoring', 'server_port']
        deadline = time.monotonic() + 30.0
        values = None
        while values is None and time.monotonic() < deadline:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                values = future.result().values
        self.assertIsNotNone(values)
        self.assertTrue(values[0].bool_value)
        self.assertEqual(values[1].integer_value, 1850)
