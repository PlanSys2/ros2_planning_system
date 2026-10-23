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

"""The BT templates and the BT builder given to the distributed bringup reach the executor."""

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
BT_DIR = os.path.join(get_package_share_directory('plansys2_executor'), 'behavior_trees')
# Not the defaults, so that the test sees them arrive: each template given for another one
TEMPLATES = {
    'action_bt_file': os.path.join(BT_DIR, 'plansys2_start_action_bt.xml'),
    'start_action_bt_file': os.path.join(BT_DIR, 'plansys2_end_action_bt.xml'),
    'end_action_bt_file': os.path.join(BT_DIR, 'plansys2_action_bt.xml'),
}
PARAMETERS = {
    'action_bt_file': 'default_action_bt_xml_filename',
    'start_action_bt_file': 'default_start_action_bt_xml_filename',
    'end_action_bt_file': 'default_end_action_bt_xml_filename',
}
# Not the default either: the default parameters file set it, and won over the argument
BUILDER = 'SimpleBTBuilder'
# Groot2 monitoring of the plan, on a port that is not the default
GROOT = {'enable_groot_monitoring': 'true', 'groot_port': '1850'}


@pytest.mark.launch_test
def generate_test_description():
    arguments = {'model_file': os.path.join(PDDL_DIR, 'domain_simple.pddl'),
                 'bt_builder_plugin': BUILDER, **TEMPLATES, **GROOT}
    plansys2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'), 'launch',
            'plansys2_bringup_launch_distributed.py')),
        launch_arguments=arguments.items())
    return launch.LaunchDescription([plansys2, launch_testing.actions.ReadyToTest()])


class TestDistributedBtFiles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_distributed_bt_files')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_the_executor_gets_the_templates_the_builder_and_groot(self):
        client = self.node.create_client(GetParameters, '/executor/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=60.0))
        request = GetParameters.Request()
        request.names = [PARAMETERS[argument] for argument in TEMPLATES] + [
            'bt_builder_plugin', 'enable_groot_monitoring', 'server_port']
        deadline = time.monotonic() + 30.0
        values = None
        while values is None and time.monotonic() < deadline:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                values = future.result().values
        self.assertIsNotNone(values)
        self.assertEqual(
            [v.string_value for v in values[:4]], list(TEMPLATES.values()) + [BUILDER])
        self.assertTrue(values[4].bool_value)
        self.assertEqual(values[5].integer_value, 1850)
