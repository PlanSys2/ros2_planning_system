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

"""Unit tests for ProblemExpertClient."""

import unittest
from unittest.mock import MagicMock, patch

from plansys2_msgs.msg import Node as PlanSys2Node
from plansys2_msgs.msg import Param, Tree
from plansys2_msgs.srv import (AddProblem, AddProblemGoal, AffectNode,
                               AffectParam, ClearProblemKnowledge, ExistNode,
                               GetNodeDetails, GetProblem, GetProblemGoal,
                               GetProblemInstanceDetails, GetProblemInstances,
                               GetStates, IsProblemGoalSatisfied,
                               RemoveProblemGoal)
from plansys2_support_py.ProblemExpertClient import ProblemExpertClient

import rclpy
from rclpy.client import Client


class TestProblemExpertClient(unittest.TestCase):
    """Test suite for ProblemExpertClient."""

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
        self.client = ProblemExpertClient('test_problem_expert_client')

    def tearDown(self):
        """Clean up after each test."""
        self.client.destroy_node()

    def test_initialization(self):
        """Test that the client initializes correctly."""
        self.assertIsNotNone(self.client)
        self.assertEqual(self.client.get_name(), 'test_problem_expert_client')
        self.assertEqual(self.client._namespace_prefix, '')

    def test_initialization_with_namespace(self):
        """Test that the client initializes correctly with namespace."""
        client = ProblemExpertClient('test_client', namespace='test_namespace')
        self.assertEqual(client._namespace_prefix, '/test_namespace')
        client.destroy_node()

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_instance_success(self, mock_call_async, mock_wait_for_service):
        """Test successful instance addition."""
        mock_wait_for_service.return_value = True

        mock_response = AffectNode.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        # Create instance parameter
        instance = Param()
        instance.name = 'robot1'
        instance.type = 'robot'

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem_instance(instance)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_multiple_exist_checks(self, mock_call_async, mock_wait_for_service):
        """Test multiple consecutive exist checks."""
        mock_wait_for_service.return_value = True

        mock_response_true = ExistNode.Response()
        mock_response_true.exist = True

        mock_response_false = ExistNode.Response()
        mock_response_false.exist = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_call_async.return_value = mock_future

        # Create predicate nodes
        predicate1 = PlanSys2Node()
        predicate1.name = '(robot_at robot1 wp1)'

        predicate2 = PlanSys2Node()
        predicate2.name = '(robot_at robot1 wp2)'

        # First check - exists
        mock_future.result.return_value = mock_response_true
        with patch('rclpy.spin_until_future_complete'):
            result = self.client.exist_problem_predicate(predicate1)
        self.assertTrue(result)

        # Second check - doesn't exist
        mock_future.result.return_value = mock_response_false
        with patch('rclpy.spin_until_future_complete'):
            result = self.client.exist_problem_predicate(predicate2)
        self.assertFalse(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_print_problem_info(self, mock_call_async, mock_wait_for_service):
        """Test print_problem_info method."""
        mock_wait_for_service.return_value = True

        # Mock problem response
        mock_problem_response = GetProblem.Response()
        mock_problem_response.success = True
        mock_problem_response.problem = '(define (problem test_problem) ...)'

        # Mock goal response
        mock_tree = Tree()
        mock_tree_node = PlanSys2Node()
        mock_tree_node.name = 'and'
        mock_tree.nodes = [mock_tree_node]

        mock_goal_response = GetProblemGoal.Response()
        mock_goal_response.success = True
        mock_goal_response.tree = mock_tree

        # Mock instances response
        mock_node1 = PlanSys2Node()
        mock_node1.name = 'robot1'
        mock_node1.node_type = 'robot'

        mock_instances_response = GetProblemInstances.Response()
        mock_instances_response.success = True
        mock_instances_response.instances = [mock_node1]

        # Mock instance details response
        mock_instance_details = Param()
        mock_instance_details.name = 'robot1'
        mock_instance_details.type = 'robot'

        mock_instance_details_response = GetProblemInstanceDetails.Response()
        mock_instance_details_response.success = True
        mock_instance_details_response.instance = mock_instance_details

        # Mock predicates response
        mock_predicate_node = PlanSys2Node()
        mock_predicate_node.name = '(robot_at robot1 wp1)'

        mock_predicates_response = GetStates.Response()
        mock_predicates_response.success = True
        mock_predicates_response.states = [mock_predicate_node]

        # Mock predicate details response
        mock_predicate_details_response = GetNodeDetails.Response()
        mock_predicate_details_response.success = True
        mock_predicate_details_response.node = mock_predicate_node

        # Mock functions response
        mock_function_node = PlanSys2Node()
        mock_function_node.name = '(battery_level robot1)'

        mock_functions_response = GetStates.Response()
        mock_functions_response.success = True
        mock_functions_response.states = [mock_function_node]

        # Mock function details response
        mock_function_details_response = GetNodeDetails.Response()
        mock_function_details_response.success = True
        mock_function_details_response.node = mock_function_node

        # Setup mock responses in order
        responses = [
            mock_problem_response,
            mock_goal_response,
            mock_instances_response,
            mock_instance_details_response,
            mock_predicates_response,
            mock_predicate_details_response,
            mock_functions_response,
            mock_function_details_response,
        ]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_call_async.return_value = mock_future

        response_iter = iter(responses)

        def get_next_response():
            return next(response_iter)

        mock_future.result.side_effect = get_next_response

        with patch('rclpy.spin_until_future_complete'):
            with patch('builtins.print') as mock_print:
                self.client.print_problem_info()

                # Verify print was called (information was printed)
                self.assertTrue(mock_print.called)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_problem_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed problem addition."""
        mock_wait_for_service.return_value = True

        mock_response = AddProblem.Response()
        mock_response.success = False
        mock_response.error_info = 'Invalid problem format'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem('invalid problem')

        self.assertFalse(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_problem_goal_success(self, mock_call_async, mock_wait_for_service):
        """Test successful goal addition."""
        mock_wait_for_service.return_value = True

        mock_response = AddProblemGoal.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        goal = Tree()
        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem_goal(goal)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_problem_instance_success(self, mock_call_async, mock_wait_for_service):
        """Test successful instance addition."""
        mock_wait_for_service.return_value = True

        mock_response = AffectParam.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        instance = Param()
        instance.name = 'robot1'
        instance.type = 'robot'

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem_instance(instance)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_problem_predicate_success(self, mock_call_async, mock_wait_for_service):
        """Test successful predicate addition."""
        mock_wait_for_service.return_value = True

        mock_response = AffectNode.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        predicate = PlanSys2Node()
        predicate.node_type = 1

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem_predicate(predicate)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_problem_function_success(self, mock_call_async, mock_wait_for_service):
        """Test successful function addition."""
        mock_wait_for_service.return_value = True

        mock_response = AffectNode.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        function = PlanSys2Node()
        function.node_type = 2

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem_function(function)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_goal_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem goal retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetProblemGoal.Response()
        mock_response.success = True
        mock_response.tree = Tree()

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_goal()

        self.assertIsNotNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_instance_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem instance retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetProblemInstanceDetails.Response()
        mock_response.success = True
        mock_response.instance = Param()
        mock_response.instance.name = 'robot1'
        mock_response.instance.type = 'robot'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_instance('robot1')

        self.assertIsNotNone(result)
        self.assertEqual(result.name, 'robot1')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_instances_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem instances retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetProblemInstances.Response()
        mock_response.success = True
        param1 = Param()
        param1.name = 'robot1'
        param2 = Param()
        param2.name = 'robot2'
        mock_response.instances = [param1, param2]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_instances()

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].name, 'robot1')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_predicate_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem predicate retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetNodeDetails.Response()
        mock_response.success = True
        mock_response.node = PlanSys2Node()
        mock_response.node.node_type = 1

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_predicate('robot_at')

        self.assertIsNotNone(result)
        self.assertEqual(result.node_type, 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_predicates_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem predicates retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = True
        mock_node = PlanSys2Node()
        mock_response.states = [mock_node]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_predicates()

        self.assertEqual(len(result), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_functions_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem functions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = True
        mock_node = PlanSys2Node()
        mock_response.states = [mock_node]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_functions()

        self.assertEqual(len(result), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetProblem.Response()
        mock_response.success = True
        mock_response.problem = '(define (problem test) ...)'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem()

        self.assertEqual(result, '(define (problem test) ...)')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_remove_problem_goal_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem goal removal."""
        mock_wait_for_service.return_value = True

        mock_response = RemoveProblemGoal.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.remove_problem_goal()

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_clear_problem_knowledge_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem knowledge clearing."""
        mock_wait_for_service.return_value = True

        mock_response = ClearProblemKnowledge.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.clear_problem_knowledge()

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_remove_problem_instance_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem instance removal."""
        mock_wait_for_service.return_value = True

        mock_response = AffectParam.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        instance = Param()
        instance.name = 'robot1'

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.remove_problem_instance(instance)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_remove_problem_predicate_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem predicate removal."""
        mock_wait_for_service.return_value = True

        mock_response = AffectNode.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        predicate = PlanSys2Node()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.remove_problem_predicate(predicate)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_remove_problem_function_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem function removal."""
        mock_wait_for_service.return_value = True

        mock_response = AffectNode.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        function = PlanSys2Node()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.remove_problem_function(function)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_exist_problem_predicate_true(self, mock_call_async, mock_wait_for_service):
        """Test checking if a predicate exists (returns True)."""
        mock_wait_for_service.return_value = True

        mock_response = ExistNode.Response()
        mock_response.exist = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        predicate = PlanSys2Node()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.exist_problem_predicate(predicate)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_exist_problem_predicate_false(self, mock_call_async, mock_wait_for_service):
        """Test checking if a predicate exists (returns False)."""
        mock_wait_for_service.return_value = True

        mock_response = ExistNode.Response()
        mock_response.exist = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        predicate = PlanSys2Node()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.exist_problem_predicate(predicate)

        self.assertFalse(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_exist_problem_function_true(self, mock_call_async, mock_wait_for_service):
        """Test checking if a function exists (returns True)."""
        mock_wait_for_service.return_value = True

        mock_response = ExistNode.Response()
        mock_response.exist = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        function = PlanSys2Node()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.exist_problem_function(function)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_update_problem_function_success(self, mock_call_async, mock_wait_for_service):
        """Test successful problem function update."""
        mock_wait_for_service.return_value = True

        mock_response = AffectNode.Response()
        mock_response.success = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        function = PlanSys2Node()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.update_problem_function(function)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_is_problem_goal_satisfied_true(self, mock_call_async, mock_wait_for_service):
        """Test checking if goal is satisfied (returns True)."""
        mock_wait_for_service.return_value = True

        mock_response = IsProblemGoalSatisfied.Response()
        mock_response.satisfied = True

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        goal = Tree()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.is_problem_goal_satisfied(goal)

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_is_problem_goal_satisfied_false(self, mock_call_async, mock_wait_for_service):
        """Test checking if goal is satisfied (returns False)."""
        mock_wait_for_service.return_value = True

        mock_response = IsProblemGoalSatisfied.Response()
        mock_response.satisfied = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        goal = Tree()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.is_problem_goal_satisfied(goal)

        self.assertFalse(result)

    @patch.object(Client, 'wait_for_service')
    def test_service_not_available(self, mock_wait_for_service):
        """Test behavior when service is not available."""
        mock_wait_for_service.return_value = False

        result = self.client.get_problem()

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
            result = self.client.get_problem()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_exception(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call raises an exception."""
        mock_wait_for_service.return_value = True
        mock_call_async.side_effect = Exception('Service error')

        result = self.client.get_problem()

        self.assertIsNone(result)

    # Additional success tests for complete coverage
    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_add_problem_success(self, mock_call_async, mock_wait_for_service):
        """Test successful add_problem operation returns True."""
        mock_wait_for_service.return_value = True
        mock_response = AddProblem.Response()
        mock_response.success = True
        mock_response.error_info = ''
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.add_problem('(robot1 - robot)')

        self.assertTrue(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_problem_instance_details_success(self, mock_call_async, mock_wait_for_service):
        """Test successful get_problem_instance_details operation."""
        mock_wait_for_service.return_value = True
        mock_response = GetProblemInstanceDetails.Response()
        mock_response.success = True
        mock_param = Param()
        mock_param.name = 'robot1'
        mock_param.type = 'robot'
        mock_response.instance = mock_param
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_problem_instance('robot1')

        self.assertIsNotNone(result)
        self.assertEqual(result.name, 'robot1')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_is_problem_goal_satisfied_success_explicit(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test is_problem_goal_satisfied with explicit success response."""
        mock_wait_for_service.return_value = True
        mock_response = IsProblemGoalSatisfied.Response()
        mock_response.success = True
        mock_response.satisfied = True
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future
        goal = Tree()

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.is_problem_goal_satisfied(goal)

        self.assertTrue(result)


if __name__ == '__main__':
    unittest.main()
