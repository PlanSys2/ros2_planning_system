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

"""Unit tests for PlannerClient."""

import unittest
from unittest.mock import MagicMock, patch

from plansys2_msgs.msg import Plan, PlanItem
from plansys2_msgs.srv import GetPlan, GetPlanArray, ValidateDomain
from plansys2_support_py.PlannerClient import PlannerClient

import rclpy
from rclpy.client import Client


class TestPlannerClient(unittest.TestCase):
    """Test suite for PlannerClient."""

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
        self.client = PlannerClient('test_planner_client')
        self.test_domain = '(define (domain test_domain) ...)'
        self.test_problem = '(define (problem test_problem) ...)'

    def tearDown(self):
        """Clean up after each test."""
        self.client.destroy_node()

    def test_initialization(self):
        """Test that the client initializes correctly."""
        self.assertIsNotNone(self.client)
        self.assertEqual(self.client.get_name(), 'test_planner_client')
        self.assertEqual(self.client._namespace_prefix, '')

    def test_initialization_with_namespace(self):
        """Test that the client initializes correctly with namespace."""
        client = PlannerClient('test_client', namespace='test_namespace')
        self.assertEqual(client._namespace_prefix, '/test_namespace')
        client.destroy_node()

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_success(self, mock_call_async, mock_wait_for_service):
        """Test successful plan generation."""
        mock_wait_for_service.return_value = True

        # Create mock plan
        mock_plan = Plan()
        mock_item = PlanItem()
        mock_item.action = 'move robot1 wp1 wp2'
        mock_item.duration = 5.0
        mock_plan.items = [mock_item]

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNotNone(result)
        self.assertIsInstance(result, Plan)
        self.assertEqual(len(result.items), 1)
        self.assertEqual(result.items[0].action, 'move robot1 wp1 wp2')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed plan generation."""
        mock_wait_for_service.return_value = True

        mock_response = GetPlan.Response()
        mock_response.success = False
        mock_response.error_info = 'No plan found'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_empty(self, mock_call_async, mock_wait_for_service):
        """Test plan generation with empty plan."""
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

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNotNone(result)
        self.assertEqual(len(result.items), 0)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_array_success(self, mock_call_async, mock_wait_for_service):
        """Test successful plan array generation."""
        mock_wait_for_service.return_value = True

        # Create mock plan array
        mock_plan1 = Plan()
        mock_item1 = PlanItem()
        mock_item1.action = 'action1'
        mock_plan1.items = [mock_item1]

        mock_plan2 = Plan()
        mock_item2 = PlanItem()
        mock_item2.action = 'action2'
        mock_plan2.items = [mock_item2]

        mock_response = GetPlanArray.Response()
        mock_response.success = True
        mock_response.plan_array = [mock_plan1, mock_plan2]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan_array(self.test_domain, self.test_problem)

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_array_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed plan array generation."""
        mock_wait_for_service.return_value = True

        mock_response = GetPlanArray.Response()
        mock_response.success = False
        mock_response.error_info = 'Failed to generate plan array'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan_array(self.test_domain, self.test_problem)

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_validate_domain_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain validation."""
        mock_wait_for_service.return_value = True

        mock_response = ValidateDomain.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.validate_domain(self.test_domain)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_validate_domain_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain validation."""
        mock_wait_for_service.return_value = True

        mock_response = ValidateDomain.Response()
        mock_response.success = False
        mock_response.error_info = 'Invalid domain syntax'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.validate_domain(self.test_domain)

        self.assertFalse(result)

    @patch.object(Client, 'wait_for_service')
    def test_service_not_available(self, mock_wait_for_service):
        """Test behavior when service is not available."""
        mock_wait_for_service.return_value = False

        result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_timeout(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call times out."""
        mock_wait_for_service.return_value = True

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_exception(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call raises an exception."""
        mock_wait_for_service.return_value = True
        mock_call_async.side_effect = Exception('Service error')

        result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_validate_domain_with_empty_string(self, mock_call_async, mock_wait_for_service):
        """Test domain validation with empty domain string."""
        mock_wait_for_service.return_value = True

        mock_response = ValidateDomain.Response()
        mock_response.success = False
        mock_response.error_info = 'Empty domain string'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.validate_domain('')

        self.assertFalse(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_plan_with_complex_problem(self, mock_call_async, mock_wait_for_service):
        """Test plan generation with complex multi-step plan."""
        mock_wait_for_service.return_value = True

        mock_plan = Plan()
        actions = ['move robot1 wp1 wp2', 'pick robot1 obj1', 'place robot1 obj1 wp3']
        for i, action in enumerate(actions):
            mock_item = PlanItem()
            mock_item.action = action
            mock_item.duration = float(i + 1)
            mock_plan.items.append(mock_item)

        mock_response = GetPlan.Response()
        mock_response.success = True
        mock_response.plan = mock_plan

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_plan(self.test_domain, self.test_problem)

        self.assertIsNotNone(result)
        self.assertEqual(len(result.items), 3)
        self.assertEqual(result.items[0].action, 'move robot1 wp1 wp2')
        self.assertEqual(result.items[1].action, 'pick robot1 obj1')
        self.assertEqual(result.items[2].action, 'place robot1 obj1 wp3')


if __name__ == '__main__':
    unittest.main()
