#!/usr/bin/env python3
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

"""
Example mission that replans when the plan fails or the world changes.

When no plan is found, it waits for a state_updater/events message instead of replanning in a loop.
"""

from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import StateUpdate
from plansys2_support_py.DomainExpertClient import DomainExpertClient
from plansys2_support_py.ExecutorClient import ExecutorClient
from plansys2_support_py.PlannerClient import PlannerClient
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient
import rclpy
from rclpy.node import Node


class MissionLoop:
    """Plans for the problem goal, runs the plan, and replans when needed."""

    def __init__(self, node: Node, updater: str = 'state_updater'):
        self.node = node
        self.domain = DomainExpertClient(node)
        self.problem = ProblemExpertClient(node)
        self.planner = PlannerClient(node)
        self.executor = ExecutorClient(node)
        self.executing = False
        self.done = False
        self.world_changed = True
        self.plans = 0
        node.create_subscription(StateUpdate, f'{updater}/events', self.on_event, 100)

    def on_event(self, event: StateUpdate):
        if event.written:
            self.world_changed = True

    def log(self, text: str):
        self.node.get_logger().info(text)

    def step(self) -> None:
        if self.done:
            return
        if self.executing:
            if self.executor.execute_and_check_plan(verbose=False):
                return
            self.executing = False
            result = self.executor.get_result()
            if result is not None and result.result == ExecutePlan.Result.SUCCESS:
                self.log('The plan ended')
            else:
                self.log('The plan failed: planning again')
                self.world_changed = True
        goal = self.problem.get_problem_goal()
        if goal is not None and goal.nodes and self.problem.is_problem_goal_satisfied(goal):
            self.log('The goal holds: the mission is done')
            self.done = True
            return
        if not self.world_changed:
            return
        self.world_changed = False
        plan = self.planner.get_plan(self.domain.get_domain(), self.problem.get_problem())
        if plan is None or not plan.items:
            self.log('No plan reaches the goal: waiting for the world to change')
            return
        self.plans += 1
        self.log(f'Plan {self.plans}: ' + ' '.join(item.action for item in plan.items))
        self.executing = self.executor.start_plan_execution(plan)


def main():
    rclpy.init()
    node = rclpy.create_node('replanning_mission')
    mission = MissionLoop(node)
    node.create_timer(0.5, mission.step)
    try:
        while rclpy.ok() and not mission.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
