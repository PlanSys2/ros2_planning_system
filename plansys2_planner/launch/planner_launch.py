# Copyright 2019 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    respawn = LaunchConfiguration('respawn')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # Specify the actions
    # Started again, 1 s later, if its process ends (not when the launch is stopped)
    declare_respawn_cmd = DeclareLaunchArgument(
        'respawn', default_value='false',
        description='Start the node again if its process ends')

    planner_cmd = Node(
        respawn=respawn,
        respawn_delay=1.0,
        package='plansys2_planner',
        executable='planner_node',
        name='planner',
        namespace=namespace,
        output='screen',
        parameters=[params_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(declare_respawn_cmd)
    ld.add_action(planner_cmd)

    return ld
