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

from typing import Any, List, Optional

from action_msgs.msg import GoalStatus
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ActionExecutionInfo, Plan, Tree
from plansys2_msgs.srv import GetOrderedSubGoals, GetPlan

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from rclpy.type_support import GetResultServiceResponse


class ExecutorClient(Node):
    """
    Executor client for PlanSys2.

    This class provides convenient methods to interact with all Executor services
    in PlanSys2, wrapping the ROS2 service calls with proper error handling.
    """

    def __init__(self, node_name: str = 'executor_client', namespace: str = '') -> None:
        """
        Initialize the Executor client.

        Parameters
        ----------
        node_name : str, optional
            Name of the ROS2 node.
        namespace : str, optional
            Namespace prefix for services.

        """
        super().__init__(node_name=node_name, namespace=namespace)

        # Setup namespace prefix
        self._namespace_prefix = f'/{namespace}' if namespace else ''

        # Action client for plan execution
        self._execute_plan_action_client: ActionClient = ActionClient(
            self,
            ExecutePlan,
            f'{self._namespace_prefix}/execute_plan'
        )

        # State variables for plan execution
        self._executing_plan = False
        self.goal_handle: Optional[ClientGoalHandle[Any, Any, Any]] = None
        self.result_future: \
            Optional[Future[GetResultServiceResponse[Any]]] = None
        self.feedback: Any = None

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

    def start_plan_execution(self, plan: Plan, verbose: bool = False) -> bool:
        """
        Start the execution of a plan without blocking.

        This method initiates the execution of the provided plan and returns immediately.
        Use `execute_and_check_plan()` to monitor the execution.

        Parameters
        ----------
        plan : Plan
            The plan to execute.
        verbose : bool, optional
            Whether to print execution feedback to the console.

        Returns
        -------
        bool
            True if plan execution started successfully, False otherwise.

        """
        # Check if plan is empty
        if plan is None:
            self.get_logger().error('Plan cannot be None')
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
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=3.0)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Plan execution goal was rejected')
            return False

        self._goal_handle = goal_handle  # type: ignore[assignment]
        self._result_future = goal_handle.get_result_async()
        self._executing_plan = True
        return True

    def execute_and_check_plan(self, verbose: bool = True) -> bool:
        """
        Check the status of plan execution.

        This method should be called periodically to monitor plan execution.
        Returns True if the plan is still executing, False if it has finished.

        Parameters
        ----------
        verbose : bool, optional
            Whether to print status information to the console.

        Returns
        -------
        bool
            True if plan is still executing, False if plan has finished.

        """
        # Get the result of the future, if complete
        rclpy.spin_until_future_complete(self, self._result_future, timeout_sec=0.10)

        # Get the result
        result_response = self._result_future.result()

        if result_response:
            if verbose:
                # Print execution results
                self._print_execution_result(result_response)

            # Update state if not preempted
            if result_response.result:
                if result_response.result.result != ExecutePlan.Result.PREEMPT:
                    self._executing_plan = False

            return False  # Plan finished
        else:
            # Timed out, still processing, not complete yet
            return True

    def _feedback_callback(self, feedback_msg) -> None:
        """
        Handle feedback from the plan execution action.

        Parameters
        ----------
        feedback_msg
            The feedback message from the action server.

        """
        feedback = feedback_msg.feedback

        # Store feedback for later use if needed
        self._feedback = feedback

        # Clear the line and print status
        self.get_logger().info('\r\033[K')

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
            self.get_logger().info(' '.join(status_parts))

    def _print_execution_result(self, result_response: Any) -> None:
        """
        Print the execution result and detailed action status information.

        Parameters
        ----------
        result_response : Any
            The execution result response containing status and action information.

        """
        # Failed and preempted plans end as aborted goals (#436)
        if result_response.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
            if result_response.result is None:
                self.get_logger().warning('Plan empty')
            elif result_response.result.result == ExecutePlan.Result.SUCCESS:
                self.get_logger().info('Plan Succeeded')
            elif result_response.result.result == ExecutePlan.Result.PREEMPT:
                self.get_logger().info('Plan Preempted')
            else:
                self.get_logger().error('Plan Failed')
                # Log each action status
                for action_status in result_response.result.action_execution_status:
                    if action_status.status == ActionExecutionInfo.SUCCEEDED:
                        self.get_logger().warning(
                            f'Action: {action_status.action_full_name} succeeded '
                            f'with message_status: {action_status.message_status}'
                        )
                    elif action_status.status == ActionExecutionInfo.FAILED:
                        self.get_logger().error(
                            f'Action: {action_status.action_full_name} failed '
                            f'with message_status: {action_status.message_status}'
                        )
                    elif action_status.status == ActionExecutionInfo.NOT_EXECUTED:
                        self.get_logger().warning(
                            f'Action: {action_status.action_full_name} was not executed'
                        )
                    elif action_status.status == ActionExecutionInfo.CANCELLED:
                        self.get_logger().warning(
                            f'Action: {action_status.action_full_name} was cancelled'
                        )
                    elif action_status.status == ActionExecutionInfo.EXECUTING:
                        self.get_logger().warning(
                            f'Action: {action_status.action_full_name} was executing'
                        )
        elif result_response.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Plan Cancelled')
        else:
            self.get_logger().error(f'Invalid status value {result_response.status}')

    def is_executing_plan(self) -> bool:
        """
        Check if a plan is currently being executed.

        Returns
        -------
        bool
            True if a plan is executing, False otherwise.

        """
        return self._executing_plan

    def get_feedback(self):
        """
        Get the last feedback from plan execution.

        Returns
        -------
        feedback
            The last feedback message or None if no feedback available.

        """
        return self._feedback

    def get_result(self) -> Optional[ExecutePlan.Result]:
        """
        Get the result from the last plan execution.

        Returns
        -------
        Optional[ExecutePlan.Result]
            The execution result or None if no result is available.

        """
        if self._result_future is None:
            return None

        result_response = self._result_future.result()
        if result_response is not None and result_response.result is not None:
            return result_response.result
        return None
