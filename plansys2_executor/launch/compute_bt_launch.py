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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (
    Node,
    PushRosNamespace,
)
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    use_groot = LaunchConfiguration('use_groot')
    use_rqt_dotgraph = LaunchConfiguration('use_rqt_dotgraph')

    namespace = LaunchConfiguration('namespace')
    action_bt_file = LaunchConfiguration('action_bt_file')
    start_action_bt_file = LaunchConfiguration('start_action_bt_file')
    end_action_bt_file = LaunchConfiguration('end_action_bt_file')
    bt_builder_plugin = LaunchConfiguration('bt_builder_plugin')
    domain = LaunchConfiguration('domain')
    problem = LaunchConfiguration('problem')

    declare_use_groot_cmd = DeclareLaunchArgument(
        'use_groot',
        default_value='False',
        description='Whether to start groot'
    )

    declare_use_rqt_dotgraph_cmd = DeclareLaunchArgument(
        'use_rqt_dotgraph',
        default_value='False',
        description='Whether to start rqt_dotgraph',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='ai_planning',
        description='Namespace')

    declare_action_bt_file_cmd = DeclareLaunchArgument(
        'action_bt_file',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_action_bt.xml'),
        description='BT representing a PDDL action')

    declare_start_action_bt_file_cmd = DeclareLaunchArgument(
        'start_action_bt_file',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_start_action_bt.xml'),
        description='BT representing a PDDL start action')

    declare_end_action_bt_file_cmd = DeclareLaunchArgument(
        'end_action_bt_file',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_end_action_bt.xml'),
        description='BT representing a PDDL end action')

    declare_bt_builder_plugin_cmd = DeclareLaunchArgument(
        'bt_builder_plugin',
        default_value='STNBTBuilder',
        description='Behavior tree builder plugin.',
    )

    declare_domain_cmd = DeclareLaunchArgument(
        'domain',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'pddl', 'road_trip_domain.pddl'),
        description='PDDL domain file.',
    )

    declare_problem_cmd = DeclareLaunchArgument(
        'problem',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'pddl', 'road_trip_problem.pddl'),
        description='PDDL domain file.',
    )

    # Specify the actions
    start_groot_cmd = ExecuteProcess(
        condition=IfCondition(use_groot),
        cmd=[
            ExecutableInPackage('Groot', 'groot'),
            '--mode',
            'monitor',
        ],
    )

    start_plan_viewer_cmd = Node(
        package='rqt_dotgraph',
        executable='rqt_dotgraph',
        name='plan_viewer',
        condition=IfCondition(use_rqt_dotgraph),
        output='screen',
        parameters=[
            {'title': 'Plan Viewer'},
            {'use_sim_time': False},
        ],
        remappings=[
            ('dot_graph', '/ai_planning/plan_dotgraph'),
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    compute_bt_cmd = Node(
        package='plansys2_executor',
        executable='compute_bt',
        name='compute_bt',
        namespace=namespace,
        output='screen',
        parameters=[
          {'action_bt_xml_filename': action_bt_file},
          {'start_action_bt_xml_filename': start_action_bt_file},
          {'end_action_bt_xml_filename': end_action_bt_file},
          {'bt_builder_plugin': bt_builder_plugin},
          {'domain': domain},
          {'problem': problem},
        ])

    visualization_group_cmd = GroupAction(
        [
            PushRosNamespace(namespace='visualization'),
            start_groot_cmd,
            start_plan_viewer_cmd,
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_groot_cmd)
    ld.add_action(declare_use_rqt_dotgraph_cmd)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_action_bt_file_cmd)
    ld.add_action(declare_start_action_bt_file_cmd)
    ld.add_action(declare_end_action_bt_file_cmd)
    ld.add_action(declare_bt_builder_plugin_cmd)
    ld.add_action(declare_domain_cmd)
    ld.add_action(declare_problem_cmd)

    # Add required actions
    ld.add_action(compute_bt_cmd)

    # Add conditioned actions
    ld.add_action(visualization_group_cmd)

    return ld
