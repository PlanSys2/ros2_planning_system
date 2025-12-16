# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
PlanSys2 Executor Client.

This module provides specific classes for interacting with PlanSys2 Executor services.
"""

from typing import List, Optional


from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ActionExecutionInfo, Plan, Tree
from plansys2_msgs.srv import GetOrderedSubGoals, GetPlan
# Import for getting domain and problem
from plansys2_support_py.DomainExpertClient import DomainExpertClient
from plansys2_support_py.PlannerClient import PlannerClient
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient

import rclpy

from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node


class ExecutorClient(Node):
    """
    Executor client for PlanSys2.

    This class provides convenient methods to interact with all Executor services
    in PlanSys2, wrapping the ROS2 service calls with proper error handling.
    """

    def __init__(self, node_name: str = 'executor_client', namespace: str = ''):
        """
        Initialize the Executor client.

        Parameters
        ----------
        node_name : str, optional
            Name of the ROS2 node.
        namespace : str, optional
            Namespace prefix for services.

        """
        super().__init__(node_name)

        # Setup namespace prefix
        self._namespace_prefix = f'/{namespace}' if namespace else ''

        # Action client for plan execution
        self._execute_plan_action_client: ActionClient = ActionClient(
            self,
            ExecutePlan,
            f'{self._namespace_prefix}/execute_plan'
        )

        # Create clients for domain, problem, and planner
        self._domain_client = DomainExpertClient(namespace=namespace)
        self._problem_client = ProblemExpertClient(namespace=namespace)
        self._planner_client = PlannerClient(namespace=namespace)

        self.get_logger().debug(f'Executor Client "{node_name}" initialized')

    def _create_and_call_service(self, service_type, service_name: str, request):
        """
        Create a service client for the given service, call it and return the response.

        Parameters
        ----------
        service_type : Type
            The ROS2 service type class.
        service_name : str
            The name/topic of the service.
        request : Any
            The service request object.

        Returns
        -------
        Any
            The service response or None if failed.

        """
        try:
            client: Client = self.create_client(service_type, service_name)

            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {service_name} not available')
                return None

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.done():
                return future.result()
            else:
                self.get_logger().error(f'Service call to {service_name} timed out')
                return None

        except Exception as e:
            self.get_logger().error(f'Error calling service {service_name}: {str(e)}')
            return None

    def get_ordered_sub_goals(self) -> Optional[List[Tree]]:
        """
        Get the ordered sub-goals from the executor.

        This method retrieves the list of sub-goals that the executor has identified
        for achieving the current problem goal, ordered by execution priority.

        Returns
        -------
        Optional[List[Tree]]
            List of sub-goals as Tree objects, or None if failed.

        """
        service_name = f'{self._namespace_prefix}/executor/get_ordered_sub_goals'
        request = GetOrderedSubGoals.Request()

        response = self._create_and_call_service(GetOrderedSubGoals, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully retrieved ordered sub-goals')
            return response.sub_goals
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to retrieve ordered sub-goals: {error_msg}')
            return None

    def get_plan(self) -> Optional[Plan]:
        """
        Get the current plan from the executor.

        This method retrieves the complete plan that the executor is currently working on.

        Returns
        -------
        Optional[Plan]
            The current plan or None if failed.

        """
        service_name = f'{self._namespace_prefix}/executor/get_plan'
        request = GetPlan.Request()
        request.domain = ''
        request.problem = ''

        response = self._create_and_call_service(GetPlan, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully retrieved current plan')
            return response.plan
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to retrieve current plan: {error_msg}')
            return None

    def get_remaining_plan(self) -> Optional[Plan]:
        """
        Get the remaining plan from the executor.

        This method retrieves the portion of the plan that has not yet been executed,
        which is useful for monitoring progress or handling plan modifications.

        Returns
        -------
        Optional[Plan]
            The remaining plan or None if failed.

        """
        service_name = f'{self._namespace_prefix}/executor/get_remaining_plan'
        request = GetPlan.Request()
        request.domain = ''
        request.problem = ''

        response = self._create_and_call_service(GetPlan, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully retrieved remaining plan')
            return response.plan
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to retrieve remaining plan: {error_msg}')
            return None

    def print_executor_info(self) -> None:
        """
        Print comprehensive information about the executor state.

        Notes
        -----
        Call multiple services to provide an overview of the executor's current status,
        including ordered sub-goals, current plan, and remaining plan.

        """
        self.get_logger().info('=== Executor Information ===')

        # Get ordered sub-goals
        sub_goals = self.get_ordered_sub_goals()
        if sub_goals:
            print(f'Ordered Sub-Goals ({len(sub_goals)}):')
            for i, goal in enumerate(sub_goals):
                print(f'  [{i + 1}] {goal}')
        else:
            print('No sub-goals available')

        # Get current plan
        current_plan = self.get_plan()
        if current_plan:
            print(f'\nCurrent Plan ({len(current_plan.items)} items):')
            for i, item in enumerate(current_plan.items):
                action = item.action if hasattr(item, 'action') else 'N/A'
                duration = item.duration if hasattr(item, 'duration') else 0.0
                print(f'  [{i + 1}] Action: {action}, Duration: {duration:.2f}s')
        else:
            print('\nNo current plan available')

        # Get remaining plan
        remaining_plan = self.get_remaining_plan()
        if remaining_plan:
            print(f'\nRemaining Plan ({len(remaining_plan.items)} items):')
            for i, item in enumerate(remaining_plan.items):
                action = item.action if hasattr(item, 'action') else 'N/A'
                duration = item.duration if hasattr(item, 'duration') else 0.0
                print(f'  [{i + 1}] Action: {action}, Duration: {duration:.2f}s')

            # Calculate progress
            if current_plan and len(current_plan.items) > 0:
                total_items = len(current_plan.items)
                remaining_items = len(remaining_plan.items)
                completed_items = total_items - remaining_items
                progress = (completed_items / total_items) * 100
                print(f'\nExecution Progress: {completed_items}/{total_items} '
                      f'({progress:.1f}% complete)')
        else:
            print('\nNo remaining plan available')
            if current_plan and len(current_plan.items) > 0:
                print('Execution Progress: 100% (Plan completed)')

        print('=' * 40)

    def execute_plan(
        self, plan: Optional[Plan] = None,
        verbose: bool = True, rate_hz: float = 5.0
    ) -> bool:
        """
        Execute a plan using the executor action client.

        This method executes the provided plan (or computes a new plan if not provided)
        and monitors its execution, displaying feedback if verbose is enabled.

        Parameters
        ----------
        plan : Optional[Plan], optional
            The plan to execute. If None, computes a new plan from
            the current domain and problem.
        verbose : bool, optional
            Whether to print execution feedback to the console.
        rate_hz : float, optional
            Update rate in Hz for checking execution status.

        Returns
        -------
        bool
            True if plan execution succeeded, False otherwise.

        """
        # If no plan provided, compute a new plan
        if plan is None:
            # Get domain and problem
            domain = self._domain_client.get_domain()
            problem = self._problem_client.get_problem()

            if not domain:
                self.get_logger().error('Could not retrieve domain')
                return False

            if not problem:
                self.get_logger().error('Could not retrieve problem')
                return False

            # Get plan from planner
            plan = self._planner_client.get_plan(domain, problem)
            if plan is None:
                self.get_logger().error('Plan could not be computed')
                return False

        # Check if plan is empty
        if len(plan.items) == 0:
            self.get_logger().warning('Plan is empty, nothing to execute')
            return False

        # Wait for the action server
        if not self._execute_plan_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Execute plan action server not available')
            return False

        # Create the goal
        goal_msg = ExecutePlan.Goal()
        goal_msg.plan = plan

        # Send the goal
        self.get_logger().info(f'Starting plan execution with {len(plan.items)} actions')
        send_goal_future = self._execute_plan_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback if verbose else None
        )

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Plan execution goal was rejected')
            return False

        if verbose:
            self.get_logger().info('Plan execution started...')

        # Wait for the result
        result_future = goal_handle.get_result_async()

        # Monitor execution with feedback
        rate = self.create_rate(rate_hz)
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()

        # Get the result
        result = result_future.result()

        if result is None:
            self.get_logger().error('Plan execution result is None')
            return False

        # Process result
        if verbose:
            print()  # New line after feedback
            self._print_execution_result(result.result)

        return result.result.result == ExecutePlan.Result.SUCCESS

    def _feedback_callback(self, feedback_msg) -> None:
        """
        Handle feedback from the plan execution action.

        Parameters
        ----------
        feedback_msg
            The feedback message from the action server.

        """
        feedback = feedback_msg.feedback

        # Clear the line and print status
        print('\r\033[K', end='', flush=True)

        status_parts = []
        for action_status in feedback.action_execution_status:
            # Skip not executed and succeeded actions
            if action_status.status in [
                ActionExecutionInfo.NOT_EXECUTED,
                ActionExecutionInfo.SUCCEEDED
            ]:
                continue

            # Build action string
            action_str = f'({action_status.action}'
            for param in action_status.arguments:
                action_str += f' {param}'
            action_str += ')'

            # Add status
            if action_status.status == ActionExecutionInfo.EXECUTING:
                status_parts.append(f'[{action_str} {action_status.completion * 100.0:.1f}%]')
            elif action_status.status == ActionExecutionInfo.FAILED:
                status_parts.append(f'[{action_str} FAILED]')
            elif action_status.status == ActionExecutionInfo.CANCELLED:
                status_parts.append(f'[{action_str} CANCELLED]')

        if status_parts:
            print(' '.join(status_parts), end='', flush=True)

    def _print_execution_result(self, result) -> None:
        """
        Print the final execution result.

        Parameters
        ----------
        result
            The execution result from the action.

        """
        if result.result == ExecutePlan.Result.SUCCESS:
            print('✓ Plan execution finished successfully')
        elif result.result == ExecutePlan.Result.PREEMPT:
            print('⚠ Plan execution was preempted')
        elif result.result == ExecutePlan.Result.FAILURE:
            print('✗ Plan execution finished with error(s)')

            # Print failed actions
            for action_status in result.action_execution_status:
                if action_status.status == ActionExecutionInfo.FAILED:
                    action_str = f'({action_status.action}'
                    for param in action_status.arguments:
                        action_str += f' {param}'
                    action_str += ')'
                    print(f'  Failed action: {action_str}')
                    if action_status.message_status:
                        print(f'    Error: {action_status.message_status}')
        else:
            print(f'Plan execution finished with unknown result: {result.result}')
