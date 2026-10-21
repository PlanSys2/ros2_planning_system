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

import time
from typing import Any, List, Optional, TYPE_CHECKING

from action_msgs.msg import GoalStatus
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import ActionExecutionInfo, Plan, Tree
from plansys2_msgs.srv import GetOrderedSubGoals, GetPlan, GetProblem
from plansys2_support_py.ClientBase import ClientBase

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

if TYPE_CHECKING:  # Only in annotations: Humble and Jazzy do not have it
    from rclpy.type_support import GetResultServiceResponse


class ExecutorClient(ClientBase):
    """
    Executor client for PlanSys2.

    This class provides convenient methods to interact with all Executor services
    in PlanSys2, wrapping the ROS2 service calls with proper error handling.
    """

    _GOAL_ATTEMPTS = 3
    # Not running the plan sent for this long: the executor lost it (restarted)
    _NOT_RUNNING_TOLERANCE = 3.0
    # Then its result is asked for this many times (an answer lost) before the plan is lost
    _RESULT_ATTEMPTS = 2

    def __init__(self, node: Node) -> None:
        """
        Initialize the Executor client.

        Parameters
        ----------
        node : Node
            The node of the application the client works for.

        """
        super().__init__(node, 'executor')

        # Action client for plan execution
        self._execute_plan_action_client: ActionClient = ActionClient(
            self._node, ExecutePlan, 'execute_plan', callback_group=self._group)

        # State variables for plan execution
        self._executing_plan = False
        self._feedback: Any = None
        self._goal_handle: Optional[ClientGoalHandle[Any, Any, Any]] = None
        self._result_future: \
            Optional[Future[GetResultServiceResponse[Any]]] = None
        # Since when the executor's action server has been missing, while executing (#446)
        self._server_lost_since: Optional[float] = None
        # The plan sent, and whether the executor still runs it (a restarted one would not)
        self._sent_plan: Optional[Plan] = None
        self._not_running_since: Optional[float] = None
        self._plan_check: Optional[Any] = None
        self._last_plan_check = 0.0
        self._result_check: Optional[Any] = None
        self._result_attempts = 0
        self._recovering = False

    def destroy(self) -> None:
        """Destroy the client and its node; the node of the application is left as it was."""
        self._execute_plan_action_client.destroy()
        super().destroy()

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
        service_name = 'executor/get_ordered_sub_goals'
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
        service_name = 'executor/get_plan'
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
        service_name = 'executor/get_remaining_plan'
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

    def get_replanning_problem(self) -> Optional[str]:
        """
        Get the problem to plan again from while a plan runs.

        It is the problem of now before the at start effects of the actions that run: a plan
        made from it has them again, and those of the new plan that are running go on.

        Returns
        -------
        Optional[str]
            The problem, or None if the executor cannot give it.

        """
        response = self._create_and_call_service(
            GetProblem, 'executor/get_replanning_problem', GetProblem.Request())
        if response and response.success:
            return response.problem
        error_msg = response.error_info if response else 'Service call failed'
        self.get_logger().error(f'Failed to retrieve the replanning problem: {error_msg}')
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

        goal_msg = ExecutePlan.Goal()
        goal_msg.plan = plan
        self.get_logger().info(f'Starting plan execution with {len(plan.items)} actions')
        goal_handle = None
        for attempt in range(1, self._GOAL_ATTEMPTS + 1):
            # As every call: until the executor is active, and once again if its server goes
            if not self.wait_active():
                return False
            if not self._execute_plan_action_client.wait_for_server(timeout_sec=3.0):
                self._lost()
                if not self.wait_active() or \
                        not self._execute_plan_action_client.wait_for_server(timeout_sec=3.0):
                    self.get_logger().error('Execute plan action server not available')
                    return False
            send_goal_future = self._execute_plan_action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback if verbose else None
            )
            if not self._wait_for(send_goal_future, 3.0):
                # The answer lost (the executor may run the plan already): again with a new
                # client; a new goal replaces the running one, which goes on keeping what runs
                self.get_logger().warning(
                    'execute_plan: no answer to the goal, sending it again '
                    f'({attempt + 1}/{self._GOAL_ATTEMPTS})')
                self._execute_plan_action_client.destroy()
                self._execute_plan_action_client = ActionClient(
                    self._node, ExecutePlan, 'execute_plan', callback_group=self._group)
                continue
            goal_handle = send_goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                # The executor rejects goals only while it is not active: wait for it, again
                self.get_logger().warning(
                    'execute_plan: goal rejected, the executor is not active')
                goal_handle = None
                self._lost()
                continue
            break
        if goal_handle is None:
            self.get_logger().error('Plan execution could not start')
            self._lost()
            return False

        self._goal_handle = goal_handle  # type: ignore[assignment]
        self._feedback = None
        self._result_future = goal_handle.get_result_async()
        self._server_lost_since = None
        self._executing_plan = True
        self._sent_plan = plan
        self._not_running_since = None
        self._plan_check = None
        self._result_check = None
        self._result_attempts = 0
        self._recovering = False
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
        if self._result_future is None:
            return False  # No plan, or its executor is gone

        # Get the result of the future, if complete
        self._wait_for(self._result_future, 0.10)

        # Get the result
        result_response = self._result_future.result()

        if result_response:
            self._recovering = False
            self._result_check = None
            if verbose:
                # Print execution results
                self._print_execution_result(result_response)

            # Update state if not preempted
            if result_response.result:
                if result_response.result.result != ExecutePlan.Result.PREEMPT:
                    self._executing_plan = False

            return False  # Plan finished
        else:
            # A gone executor never answers: do not wait for it forever (#446)
            if self._executing_plan and not self._execute_plan_action_client.server_is_ready():
                now = time.monotonic()
                if self._server_lost_since is None:
                    self._server_lost_since = now
                elif now - self._server_lost_since > 2.0:
                    self.get_logger().error(
                        'execute_and_check_plan: the executor is gone, the plan is lost')
                    self._executing_plan = False
                    self._server_lost_since = None
                    self._result_future = None
                    return False
            else:
                self._server_lost_since = None
                # Not running our plan: ended with its result lost on the way, or a restarted
                # executor that has a server, but not our goal, whose result never comes
                if self._executing_plan and (self._recovering or self._plan_not_running()):
                    self._recovering = True
                    recovered = self._recover_result()
                    if recovered is None:
                        return True
                    self._recovering = False
                    if not recovered:
                        self.get_logger().error(
                            'execute_and_check_plan: the executor does not run the plan '
                            '(restarted?), it is lost')
                        self._executing_plan = False
                        self._result_future = None
                        return False
                    self.get_logger().warning(
                        'execute_and_check_plan: the result was lost, asked for again')
                    return self.execute_and_check_plan(verbose)
            # Timed out, still processing, not complete yet
            return True

    def _plan_not_running(self) -> bool:
        """Whether the executor has not run the plan sent for a while; asked without waiting."""
        now = time.monotonic()
        if self._plan_check is not None and self._plan_check[0].done():
            response = self._plan_check[0].result()
            self._plan_check = None
            running = response is not None and response.success and \
                self._same_plan(response.plan, self._sent_plan)
            if running:
                self._not_running_since = None
            elif self._not_running_since is None:
                self._not_running_since = now
            elif now - self._not_running_since > self._NOT_RUNNING_TOLERANCE:
                self._not_running_since = None
                return True
        elif self._plan_check is not None and now - self._plan_check[1] > 2.0:
            self._plan_check = None  # Without answer: asked again
        if self._plan_check is None and now - self._last_plan_check > 1.0:
            client = self._service_client(GetPlan, 'executor/get_plan')
            if client.service_is_ready():
                self._plan_check = (client.call_async(GetPlan.Request()), now)
                self._last_plan_check = now
        return False

    def _recover_result(self) -> Optional[bool]:
        """Ask for the result of our goal again, without waiting: None while pending."""
        now = time.monotonic()
        if self._result_check is not None and self._result_check[0].done():
            future = self._result_check[0]
            self._result_check = None
            self._result_attempts = 0
            response = future.result()
            if response is None or response.status not in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_CANCELED):
                return False  # Unknown to it: a restarted executor
            self._result_future = future
            return True
        if self._result_check is not None and now - self._result_check[1] > 2.0:
            # A goal that still runs answers only when it ends: not the plan sent any more
            self._result_check = None
            self._result_attempts += 1
            if self._result_attempts >= self._RESULT_ATTEMPTS:
                self._result_attempts = 0
                return False
        if self._result_check is None:
            if self._goal_handle is None:
                return False
            self._result_check = (self._goal_handle.get_result_async(), now)
        return None

    @staticmethod
    def _same_plan(a: Plan, b: Optional[Plan]) -> bool:
        """Whether two plans have the same actions at the same times."""
        if b is None or len(a.items) != len(b.items):
            return False
        return all(x.action == y.action and abs(x.time - y.time) < 1e-6
                   for x, y in zip(a.items, b.items))

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

    def cancel_plan_execution(self) -> bool:
        """
        Cancel the plan in execution, as the C++ cancel_plan_execution.

        Returns
        -------
        bool
            True if the executor accepted to cancel it; False if there was no plan running or
            the executor did not answer.

        """
        goal_handle = self._goal_handle
        running = (self._executing_plan and goal_handle is not None and
                   self._result_future is not None and not self._result_future.done())
        self._executing_plan = False
        if not running:
            return False
        cancel_future = goal_handle.cancel_goal_async()
        if not self._wait_for(cancel_future, 3.0):
            self.get_logger().error('Failed to cancel action server for execute_plan')
            return False
        response = cancel_future.result()
        return response is not None and len(response.goals_canceling) > 0

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
        self._serve()
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

        self._serve()
        result_response = self._result_future.result()
        if result_response is not None and result_response.result is not None:
            return result_response.result
        return None
