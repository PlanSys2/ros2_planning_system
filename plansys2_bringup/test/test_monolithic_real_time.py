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

"""Without permissions for real time, plansys2_node exits with an error, not a crash."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest

# root ignores the real time limit that the test removes
CAN_REAL_TIME = os.geteuid() == 0
SKIP_REASON = 'root may always use real time scheduling'


@pytest.mark.launch_test
def generate_test_description():
    plansys2 = launch_ros.actions.Node(
        package='plansys2_bringup',
        executable='plansys2_node',
        # No permission for real time priorities, whatever the user's limits
        prefix='prlimit --rtprio=0:0 --',
        output='screen',
        arguments=['--use-real-time', 'true'],
        parameters=[{
            'model_file': os.path.join(
                get_package_share_directory('plansys2_popf_plan_solver'),
                'pddl', 'domain_simple.pddl'),
        }])

    # Keeps the launch alive until the tests run, as the node exits at once
    keep_alive = launch.actions.TimerAction(period=30.0, actions=[])

    return (
        launch.LaunchDescription([
            plansys2, keep_alive, launch_testing.actions.ReadyToTest()]),
        {'plansys2': plansys2})


class TestRealTimeWithoutPermissions(unittest.TestCase):

    @unittest.skipIf(CAN_REAL_TIME, SKIP_REASON)
    def test_exits(self, proc_info, plansys2):
        proc_info.assertWaitForShutdown(process=plansys2, timeout=30)


@launch_testing.post_shutdown_test()
class TestRealTimeExitCode(unittest.TestCase):

    @unittest.skipIf(CAN_REAL_TIME, SKIP_REASON)
    def test_error_exit_code(self, proc_info, plansys2):
        # It used to throw from a thread: std::terminate, SIGABRT
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[1], process=plansys2)
