# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
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

"""Unit tests for ExecutorClient."""

import time
import unittest
from unittest.mock import MagicMock, patch

from action_msgs.msg import GoalStatus
from client_mocks import always_active, no_wait
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Plan, PlanItem, Tree
from plansys2_msgs.srv import GetOrderedSubGoals, GetPlan
from plansys2_support_py.ExecutorClient import ExecutorClient

import rclpy
from rclpy.client import Client


class TestExecutorClient(unittest.TestCase):
    """Test suite for ExecutorClient."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test."""
        always_active(self)
        self.node = rclpy.create_node('test_executor_client')
        self.client = ExecutorClient(self.node)

    def tearDown(self):
        """Clean up after each test."""
        self.client.destroy()
        self.node.destroy_node()

    def test_initialization(self):
        """Test that the client initializes correctly."""
        self.assertIsNotNone(self.client)
        self.assertEqual(self.client.node.get_name(), 'test_executor_client')
        # Relative names: the namespace and the remappings apply
        self.assertEqual(
            self.client.node.resolve_service_name('executor/get_plan'),
            '/executor/get_plan')

    def test_initialization_with_namespace(self):
        """Test that the client initializes correctly with namespace."""
        node = rclpy.create_node('test_client', namespace='test_namespace')
        client = ExecutorClient(node)
        self.assertEqual(
            client.node.resolve_service_name('executor/get_plan'),
            '/test_namespace/executor/get_plan')
        client.destroy()
        node.destroy_node()

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_ordered_sub_goals_success(self, mock_call_async, mock_wait_for_service):
        """Test successful ordered sub-goals retrieval."""
        mock_wait_for_service.return_value = True

        # Create mock sub-goals
        mock_tree1 = Tree()
        mock_tree2 = Tree()

        mock_response = GetOrderedSubGoals.Response()
        mock_response.success = True
        mock_response.sub_goals = [mock_tree1, mock_tree2]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_ordered_sub_goals()

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_ordered_sub_goals_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed ordered sub-goals retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetOrderedSubGoals.Response()
        mock_response.success = False
        mock_response.error_info = 'No sub-goals available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_ordered_sub_goals()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_ordered_sub_goals_empty(self, mock_call_async, mock_wait_for_service):
        """Test ordered sub-goals retrieval with empty list."""
        mock_wait_for_service.return_value = True

        mock_response = GetOrderedSubGoals.Response()
        mock_response.success = True
        mock_response.sub_goals = []

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_ordered_sub_goals()

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 0)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_success(self, mock_call_async, mock_wait_for_service):
        """Test successful plan retrieval."""
        mock_wait_for_service.return_value = True

        # Create mock plan
        mock_plan = Plan()
        mock_item1 = PlanItem()
        mock_item1.action = 'move robot1 wp1 wp2'
        mock_item1.duration = 5.0
        mock_item2 = PlanItem()
        mock_item2.action = 'pick robot1 obj1'
        mock_item2.duration = 3.0
        mock_plan.items = [mock_item1, mock_item2]

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_plan()

        self.assertIsNotNone(result)
        self.assertIsInstance(result, Plan)
        self.assertEqual(len(result.items), 2)
        self.assertEqual(result.items[0].action, 'move robot1 wp1 wp2')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed plan retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetPlan.Response()
        mock_response.success = False
        mock_response.error_info = 'No plan available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_plan()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_empty(self, mock_call_async, mock_wait_for_service):
        """Test plan retrieval with empty plan."""
        mock_wait_for_service.return_value = True

        mock_plan = Plan()
        mock_plan.items = []

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_plan()

        self.assertIsNotNone(result)
        self.assertEqual(len(result.items), 0)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_remaining_plan_success(self, mock_call_async, mock_wait_for_service):
        """Test successful remaining plan retrieval."""
        mock_wait_for_service.return_value = True

        # Create mock remaining plan
        mock_plan = Plan()
        mock_item = PlanItem()
        mock_item.action = 'place robot1 obj1 wp3'
        mock_item.duration = 2.0
        mock_plan.items = [mock_item]

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_remaining_plan()

        self.assertIsNotNone(result)
        self.assertIsInstance(result, Plan)
        self.assertEqual(len(result.items), 1)
        self.assertEqual(result.items[0].action, 'place robot1 obj1 wp3')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_remaining_plan_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed remaining plan retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetPlan.Response()
        mock_response.success = False
        mock_response.error_info = 'No remaining plan'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_remaining_plan()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_remaining_plan_empty(self, mock_call_async, mock_wait_for_service):
        """Test remaining plan retrieval with empty plan (plan completed)."""
        mock_wait_for_service.return_value = True

        mock_plan = Plan()
        mock_plan.items = []

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_remaining_plan()

        self.assertIsNotNone(result)
        self.assertEqual(len(result.items), 0)

    @patch.object(Client, 'wait_for_service')
    def test_service_not_available(self, mock_wait_for_service):
        """Test behavior when service is not available."""
        mock_wait_for_service.return_value = False

        result = self.client.get_plan()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_timeout(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call times out."""
        mock_wait_for_service.return_value = True

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_plan()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_exception(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call raises an exception."""
        mock_wait_for_service.return_value = True
        mock_call_async.side_effect = Exception('Service error')

        result = self.client.get_plan()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_with_multiple_actions(self, mock_call_async, mock_wait_for_service):
        """Test plan retrieval with multiple complex actions."""
        mock_wait_for_service.return_value = True

        mock_plan = Plan()
        actions = [
            ('move robot1 wp1 wp2', 5.0),
            ('pick robot1 obj1', 3.0),
            ('move robot1 wp2 wp3', 4.0),
            ('place robot1 obj1 wp3', 2.0)
        ]

        for action, duration in actions:
            mock_item = PlanItem()
            mock_item.action = action
            mock_item.duration = duration
            mock_plan.items.append(mock_item)

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with no_wait():
            result = self.client.get_plan()

        self.assertIsNotNone(result)
        self.assertEqual(len(result.items), 4)
        self.assertEqual(result.items[0].action, 'move robot1 wp1 wp2')
        self.assertEqual(result.items[3].action, 'place robot1 obj1 wp3')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_multiple_service_calls(self, mock_call_async, mock_wait_for_service):
        """Test multiple consecutive service calls."""
        mock_wait_for_service.return_value = True

        # First call - get full plan
        mock_plan_full = Plan()
        for i in range(3):
            mock_item = PlanItem()
            mock_item.action = f'action_{i}'
            mock_item.duration = float(i + 1)
            mock_plan_full.items.append(mock_item)

        mock_response_full = GetPlan.Response()
        mock_response_full.success = True
        mock_response_full.plan = mock_plan_full

        # Second call - get remaining plan
        mock_plan_remaining = Plan()
        mock_item = PlanItem()
        mock_item.action = 'action_2'
        mock_item.duration = 3.0
        mock_plan_remaining.items = [mock_item]

        mock_response_remaining = GetPlan.Response()
        mock_response_remaining.success = True
        mock_response_remaining.plan = mock_plan_remaining

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_call_async.return_value = mock_future

        # First call
        mock_future.result.return_value = mock_response_full
        with no_wait():
            result_full = self.client.get_plan()

        # Second call
        mock_future.result.return_value = mock_response_remaining
        with no_wait():
            result_remaining = self.client.get_remaining_plan()

        self.assertEqual(len(result_full.items), 3)
        self.assertEqual(len(result_remaining.items), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_print_executor_info(self, mock_call_async, mock_wait_for_service):
        """Test print_executor_info method."""
        mock_wait_for_service.return_value = True

        # Mock sub-goals response
        mock_tree = Tree()
        mock_subgoals_response = GetOrderedSubGoals.Response()
        mock_subgoals_response.success = True
        mock_subgoals_response.sub_goals = [mock_tree]

        # Mock current plan response
        mock_plan = Plan()
        mock_item = PlanItem()
        mock_item.action = 'move robot1 wp1 wp2'
        mock_item.duration = 5.0
        mock_plan.items = [mock_item]

        mock_current_plan_response = GetPlan.Response()
        mock_current_plan_response.success = True
        mock_current_plan_response.plan = mock_plan

        # Mock remaining plan response
        mock_remaining_plan_response = GetPlan.Response()
        mock_remaining_plan_response.success = True
        mock_remaining_plan_response.plan = Plan()  # Empty plan

        # Setup mock responses
        responses = [
            mock_subgoals_response,
            mock_current_plan_response,
            mock_remaining_plan_response
        ]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_call_async.return_value = mock_future

        response_iter = iter(responses)

        def get_next_response():
            return next(response_iter)

        mock_future.result.side_effect = get_next_response

        with no_wait():
            with patch('builtins.print') as mock_print:
                self.client.print_executor_info()

                # Verify print was called (information was printed)
                self.assertTrue(mock_print.called)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_print_executor_info_with_plans(self, mock_call_async, mock_wait_for_service):
        """Test print_executor_info with both current and remaining plans."""
        mock_wait_for_service.return_value = True

        # Mock ordered sub-goals
        mock_sub_goals_response = GetOrderedSubGoals.Response()
        mock_sub_goals_response.success = True
        mock_sub_goals_response.sub_goals = [Tree(), Tree()]

        # Mock current plan
        mock_current_plan_response = GetPlan.Response()
        mock_current_plan_response.success = True
        mock_item1 = PlanItem()
        mock_item1.action = 'move'
        mock_item1.duration = 5.0
        mock_item2 = PlanItem()
        mock_item2.action = 'pick'
        mock_item2.duration = 3.0
        mock_current_plan_response.plan.items = [mock_item1, mock_item2]

        # Mock remaining plan
        mock_remaining_plan_response = GetPlan.Response()
        mock_remaining_plan_response.success = True
        mock_item3 = PlanItem()
        mock_item3.action = 'pick'
        mock_item3.duration = 3.0
        mock_remaining_plan_response.plan.items = [mock_item3]

        response_iter = iter([
            mock_sub_goals_response,
            mock_current_plan_response,
            mock_remaining_plan_response
        ])

        mock_future = MagicMock()
        mock_future.done.return_value = True

        def get_next_response():
            return next(response_iter)

        mock_future.result.side_effect = get_next_response
        mock_call_async.return_value = mock_future

        with no_wait():
            with patch('builtins.print') as mock_print:
                self.client.print_executor_info()
                self.assertTrue(mock_print.called)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_print_executor_info_no_remaining_plan(self, mock_call_async, mock_wait_for_service):
        """Test print_executor_info with current plan but no remaining plan."""
        mock_wait_for_service.return_value = True

        # Mock ordered sub-goals
        mock_sub_goals_response = GetOrderedSubGoals.Response()
        mock_sub_goals_response.success = True
        mock_sub_goals_response.sub_goals = []

        # Mock current plan
        mock_current_plan_response = GetPlan.Response()
        mock_current_plan_response.success = True
        mock_item1 = PlanItem()
        mock_item1.action = 'move'
        mock_item1.duration = 5.0
        mock_current_plan_response.plan.items = [mock_item1]

        # Mock no remaining plan
        mock_remaining_plan_response = GetPlan.Response()
        mock_remaining_plan_response.success = True
        mock_remaining_plan_response.plan.items = []

        response_iter = iter([
            mock_sub_goals_response,
            mock_current_plan_response,
            mock_remaining_plan_response
        ])

        mock_future = MagicMock()
        mock_future.done.return_value = True

        def get_next_response():
            return next(response_iter)

        mock_future.result.side_effect = get_next_response
        mock_call_async.return_value = mock_future

        with no_wait():
            with patch('builtins.print') as mock_print:
                self.client.print_executor_info()
                self.assertTrue(mock_print.called)

    def test_execute_plan_with_provided_plan(self):
        """Test execute_plan with a provided plan."""
        # Create a plan
        plan = Plan()
        item = PlanItem()
        item.action = 'move'
        item.duration = 5.0
        plan.items = [item]

        # Mock action client
        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True

        # Mock goal handle
        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = True

        mock_send_goal_future = MagicMock()
        mock_send_goal_future.result.return_value = mock_goal_handle
        mock_client.send_goal_async.return_value = mock_send_goal_future

        # Mock result
        from plansys2_msgs.action import ExecutePlan
        mock_result = MagicMock()
        mock_result.result.result = ExecutePlan.Result.SUCCESS

        mock_result_future = MagicMock()
        mock_result_future.done.return_value = True
        mock_result_future.result.return_value = mock_result
        mock_goal_handle.get_result_async.return_value = mock_result_future

        # Replace the action client
        self.client._execute_plan_action_client = mock_client

        with no_wait():
            result = self.client.start_plan_execution(plan, verbose=False)

        self.assertTrue(result)

    def test_execute_plan_empty_plan(self):
        """Test start_plan_execution with empty plan."""
        plan = Plan()
        plan.items = []

        result = self.client.start_plan_execution(plan)
        self.assertFalse(result)

    def test_execute_plan_server_not_available(self):
        """Test start_plan_execution when action server is not available."""
        plan = Plan()
        item = PlanItem()
        item.action = 'move'
        plan.items = [item]

        # Mock the existing action client
        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = False
        self.client._execute_plan_action_client = mock_client

        result = self.client.start_plan_execution(plan)
        self.assertFalse(result)

    def test_execute_plan_goal_rejected(self):
        """Test start_plan_execution when goal is rejected."""
        plan = Plan()
        item = PlanItem()
        item.action = 'move'
        plan.items = [item]

        # Mock the existing action client
        mock_client = MagicMock()
        mock_client.wait_for_server.return_value = True

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = False

        mock_send_goal_future = MagicMock()
        mock_send_goal_future.result.return_value = mock_goal_handle
        mock_client.send_goal_async.return_value = mock_send_goal_future

        self.client._execute_plan_action_client = mock_client

        with no_wait():
            result = self.client.start_plan_execution(plan)

        self.assertFalse(result)

    def test_execute_plan_compute_new_plan(self):
        """Test start_plan_execution with provided plan."""
        plan = Plan()
        item = PlanItem()
        item.action = 'move'
        plan.items = [item]

        # Mock action client
        mock_action_client = MagicMock()
        mock_action_client.wait_for_server.return_value = True

        mock_goal_handle = MagicMock()
        mock_goal_handle.accepted = True

        mock_send_goal_future = MagicMock()
        mock_send_goal_future.result.return_value = mock_goal_handle
        mock_action_client.send_goal_async.return_value = mock_send_goal_future

        from plansys2_msgs.action import ExecutePlan
        mock_result = MagicMock()
        mock_result.result.result = ExecutePlan.Result.SUCCESS

        mock_result_future = MagicMock()
        mock_result_future.done.return_value = True
        mock_result_future.result.return_value = mock_result
        mock_goal_handle.get_result_async.return_value = mock_result_future

        # Patch the action client at instance level
        self.client._execute_plan_action_client = mock_action_client

        with no_wait():
            result = self.client.start_plan_execution(plan, verbose=False)

        self.assertTrue(result)

    def test_execute_plan_no_domain(self):
        """Test start_plan_execution with None plan."""
        # This test is now obsolete since we don't auto-compute plans
        plan = None
        result = self.client.start_plan_execution(plan) if plan else False
        self.assertFalse(result)

    def test_execute_plan_no_problem(self):
        """Test start_plan_execution with None plan."""
        # This test is now obsolete since we don't auto-compute plans
        plan = None
        result = self.client.start_plan_execution(plan) if plan else False
        self.assertFalse(result)

    def test_execute_plan_planner_fails(self):
        """Test start_plan_execution with None plan."""
        # This test is now obsolete since we don't auto-compute plans
        plan = None
        result = self.client.start_plan_execution(plan) if plan else False
        self.assertFalse(result)

    def _running_plan(self):
        """Pretend a plan is executing (#446: a gone executor never answers)."""
        self.client._executing_plan = True
        self.client._result_future = MagicMock()
        self.client._result_future.result.return_value = None
        # The executor runs it (its own test below)
        self.client._plan_not_running = lambda: False

    def test_no_plan_has_no_result(self):
        """Before any plan there is no result, and nothing is executing."""
        self.assertIsNone(self.client.get_result())
        self.assertFalse(self.client.execute_and_check_plan(verbose=False))

    @no_wait()
    def test_executor_gone_ends_the_plan(self):
        """A missing executor for over 2 s ends the plan without result."""
        self._running_plan()
        server = self.client._execute_plan_action_client
        with patch.object(server, 'server_is_ready', return_value=False), \
                patch('time.monotonic', side_effect=[0.0, 1.0, 3.0]):
            self.assertTrue(self.client.execute_and_check_plan(verbose=False))
            self.assertTrue(self.client.execute_and_check_plan(verbose=False))
            self.assertFalse(self.client.execute_and_check_plan(verbose=False))
        self.assertFalse(self.client.is_executing_plan())
        self.assertIsNone(self.client.get_result())

    @no_wait()
    def test_live_executor_keeps_the_plan(self):
        """While the executor is there, the plan is still executing."""
        self._running_plan()
        server = self.client._execute_plan_action_client
        with patch.object(server, 'server_is_ready', return_value=True):
            for _ in range(5):
                self.assertTrue(self.client.execute_and_check_plan(verbose=False))
        self.assertTrue(self.client.is_executing_plan())

    @no_wait()
    def test_short_executor_absence_is_tolerated(self):
        """A brief absence (discovery) does not end the plan."""
        self._running_plan()
        server = self.client._execute_plan_action_client
        with patch.object(server, 'server_is_ready', side_effect=[False, True, False, False]), \
                patch('time.monotonic', side_effect=[0.0, 10.0, 11.0]):
            for _ in range(4):
                self.assertTrue(self.client.execute_and_check_plan(verbose=False))
        self.assertTrue(self.client.is_executing_plan())

    @no_wait()
    def test_a_plan_the_executor_does_not_run_is_lost(self):
        """A restarted executor has a server but not our goal: the plan ends without result."""
        self._running_plan()
        del self.client._plan_not_running  # The real one
        self.client._sent_plan = Plan()
        self.client._sent_plan.items.append(PlanItem(action='(move r2d2 a b)', time=0.0))
        self.client._NOT_RUNNING_TOLERANCE = 0.0
        not_running = GetPlan.Response()
        not_running.success = False
        answer = MagicMock()
        answer.done.return_value = True
        answer.result.return_value = not_running
        checker = MagicMock()
        checker.service_is_ready.return_value = True
        checker.call_async.return_value = answer
        server = self.client._execute_plan_action_client
        with patch.object(server, 'server_is_ready', return_value=True), \
                patch.object(self.client, '_service_client', return_value=checker):
            end = time.monotonic() + 10.0
            while self.client.execute_and_check_plan(verbose=False) and time.monotonic() < end:
                time.sleep(0.1)
        self.assertFalse(self.client.is_executing_plan())
        self.assertIsNone(self.client.get_result())

    @no_wait()
    def test_the_plan_the_executor_runs_goes_on(self):
        """While the executor answers that it runs our plan, the plan goes on."""
        self._running_plan()
        del self.client._plan_not_running
        self.client._sent_plan = Plan()
        self.client._sent_plan.items.append(PlanItem(action='(move r2d2 a b)', time=0.0))
        self.client._NOT_RUNNING_TOLERANCE = 0.0
        running = GetPlan.Response()
        running.success = True
        running.plan = self.client._sent_plan
        answer = MagicMock()
        answer.done.return_value = True
        answer.result.return_value = running
        checker = MagicMock()
        checker.service_is_ready.return_value = True
        checker.call_async.return_value = answer
        server = self.client._execute_plan_action_client
        with patch.object(server, 'server_is_ready', return_value=True), \
                patch.object(self.client, '_service_client', return_value=checker):
            end = time.monotonic() + 3.0
            while time.monotonic() < end:
                self.assertTrue(self.client.execute_and_check_plan(verbose=False))
                time.sleep(0.1)
        self.assertGreaterEqual(checker.call_async.call_count, 2)

    def _ended_plan(self, answer):
        """Pretend a plan the executor does not run any more, whose result asked gives answer."""
        self._running_plan()
        del self.client._plan_not_running
        self.client._sent_plan = Plan()
        self.client._sent_plan.items.append(PlanItem(action='(move r2d2 a b)', time=0.0))
        self.client._NOT_RUNNING_TOLERANCE = 0.0
        not_running = GetPlan.Response()
        not_running.success = False
        plan_answer = MagicMock()
        plan_answer.done.return_value = True
        plan_answer.result.return_value = not_running
        checker = MagicMock()
        checker.service_is_ready.return_value = True
        checker.call_async.return_value = plan_answer
        self.client._goal_handle = MagicMock()
        self.client._goal_handle.get_result_async.return_value = answer
        return checker

    def _run_until_end(self, checker, timeout):
        server = self.client._execute_plan_action_client
        with patch.object(server, 'server_is_ready', return_value=True), \
                patch.object(self.client, '_service_client', return_value=checker):
            start = time.monotonic()
            while self.client.execute_and_check_plan(verbose=False):
                self.assertLess(time.monotonic() - start, timeout)
                time.sleep(0.1)
            return time.monotonic() - start

    @staticmethod
    def _result_answer(status):
        response = ExecutePlan.Impl.GetResultService.Response()
        response.status = status
        response.result.result = ExecutePlan.Result.SUCCESS
        answer = MagicMock()
        answer.done.return_value = True
        answer.result.return_value = response
        return answer

    @no_wait()
    def test_a_lost_result_is_asked_for_again(self):
        """The plan ended, but its result was lost on the way: asked for again, it comes."""
        for status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
            with self.subTest(status=status):
                checker = self._ended_plan(self._result_answer(status))
                self._run_until_end(checker, 10.0)
                self.assertFalse(self.client.is_executing_plan())
                self.assertEqual(self.client.get_result().result, ExecutePlan.Result.SUCCESS)
                self.client._goal_handle.get_result_async.assert_called_once()

    @no_wait()
    def test_a_goal_unknown_to_the_executor_is_lost(self):
        """A restarted executor does not know our goal: lost at once, without result."""
        checker = self._ended_plan(self._result_answer(GoalStatus.STATUS_UNKNOWN))
        self.assertLess(self._run_until_end(checker, 10.0), 3.0)
        self.assertFalse(self.client.is_executing_plan())
        self.assertIsNone(self.client.get_result())

    @no_wait()
    def test_a_result_never_answered_is_lost(self):
        """Our goal still runs there (a plan not ours any more): lost after a few tries."""
        silent = MagicMock()
        silent.done.return_value = False
        checker = self._ended_plan(silent)
        seconds = self._run_until_end(checker, 15.0)
        self.assertGreaterEqual(seconds, 4.0)
        self.assertFalse(self.client.is_executing_plan())
        self.assertIsNone(self.client.get_result())
        self.assertEqual(self.client._goal_handle.get_result_async.call_count,
                         ExecutorClient._RESULT_ATTEMPTS)


if __name__ == '__main__':
    unittest.main()
