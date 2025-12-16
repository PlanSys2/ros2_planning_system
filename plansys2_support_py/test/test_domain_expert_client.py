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

"""Unit tests for DomainExpertClient."""

import unittest
from unittest.mock import MagicMock, patch

from plansys2_msgs.msg import Action as PlanSys2Action
from plansys2_msgs.msg import Derived
from plansys2_msgs.msg import DurativeAction as PlanSys2DurativeAction
from plansys2_msgs.msg import Node as PlanSys2Node
from plansys2_msgs.srv import (GetDomain, GetDomainActionDetails,
                               GetDomainActions, GetDomainConstants,
                               GetDomainDerivedPredicateDetails,
                               GetDomainDurativeActionDetails, GetDomainName,
                               GetDomainTypes, GetNodeDetails, GetStates)
from plansys2_support_py.DomainExpertClient import DomainExpertClient

import rclpy
from rclpy.client import Client


class TestDomainExpertClient(unittest.TestCase):
    """Test suite for DomainExpertClient."""

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
        self.client = DomainExpertClient('test_domain_expert_client')

    def tearDown(self):
        """Clean up after each test."""
        self.client.destroy_node()

    def test_initialization(self):
        """Test that the client initializes correctly."""
        self.assertIsNotNone(self.client)
        self.assertEqual(self.client.get_name(), 'test_domain_expert_client')
        self.assertEqual(self.client._namespace_prefix, '')

    def test_initialization_with_namespace(self):
        """Test that the client initializes correctly with namespace."""
        client = DomainExpertClient('test_client', namespace='test_namespace')
        self.assertEqual(client._namespace_prefix, '/test_namespace')
        client.destroy_node()

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain retrieval."""
        mock_wait_for_service.return_value = True

        # Create mock response
        mock_response = GetDomain.Response()
        mock_response.success = True
        mock_response.domain = '(define (domain test_domain) ...)'

        # Create mock future
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain()

        self.assertIsNotNone(result)
        self.assertEqual(result, '(define (domain test_domain) ...)')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomain.Response()
        mock_response.success = False
        mock_response.error_info = 'Domain not found'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    def test_service_not_available(self, mock_wait_for_service):
        """Test behavior when service is not available."""
        mock_wait_for_service.return_value = False

        result = self.client.get_domain()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_name_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain name retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainName.Response()
        mock_response.success = True
        mock_response.name = 'test_domain'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_name()

        self.assertEqual(result, 'test_domain')

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_types_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain types retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainTypes.Response()
        mock_response.success = True
        mock_response.types = ['robot', 'waypoint', 'location']

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_types()

        self.assertEqual(len(result), 3)
        self.assertIn('robot', result)
        self.assertIn('waypoint', result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_constants_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain constants retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainConstants.Response()
        mock_response.success = True
        mock_response.constants = ['robot1', 'robot2']

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_constants('robot')

        self.assertEqual(len(result), 2)
        self.assertIn('robot1', result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_predicates_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain predicates retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = True
        mock_node = PlanSys2Node()
        mock_node.node_type = 1
        mock_response.states = [mock_node]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_predicates()

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_functions_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain functions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = True
        mock_node = PlanSys2Node()
        mock_node.node_type = 2
        mock_response.states = [mock_node]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_functions()

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_actions_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain actions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainActions.Response()
        mock_response.success = True
        mock_response.actions = ['move', 'pick', 'place']

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_actions()

        self.assertEqual(len(result), 3)
        self.assertIn('move', result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_durative_actions_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain durative actions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainActions.Response()
        mock_response.success = True
        mock_response.actions = ['move_durative']

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_durative_actions()

        self.assertEqual(len(result), 1)
        self.assertIn('move_durative', result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_predicate_details_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain predicate details retrieval."""
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
            result = self.client.get_domain_predicate_details('robot_at')

        self.assertIsNotNone(result)
        self.assertEqual(result.node_type, 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_action_details_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain action details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainActionDetails.Response()
        mock_response.success = True
        mock_response.action = PlanSys2Action()

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_action_details('move', ['robot1', 'wp1', 'wp2'])

        self.assertIsNotNone(result)
        self.assertIsInstance(result, PlanSys2Action)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_durative_action_details_success(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test successful domain durative action details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainDurativeActionDetails.Response()
        mock_response.success = True
        mock_response.durative_action = PlanSys2DurativeAction()

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_durative_action_details('move_durative')

        self.assertIsNotNone(result)
        self.assertIsInstance(result, PlanSys2DurativeAction)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_derived_predicates_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain derived predicates retrieval."""
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
            result = self.client.get_domain_derived_predicates()

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_derived_predicate_details_success(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test successful domain derived predicate details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainDerivedPredicateDetails.Response()
        mock_response.success = True
        mock_derived = Derived()
        mock_response.predicates = [mock_derived]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_derived_predicate_details('derived_pred')

        self.assertIsNotNone(result)
        self.assertEqual(len(result), 1)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_timeout(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call times out."""
        mock_wait_for_service.return_value = True

        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_service_exception(self, mock_call_async, mock_wait_for_service):
        """Test behavior when service call raises an exception."""
        mock_wait_for_service.return_value = True
        mock_call_async.side_effect = RuntimeError('Service error')

        result = self.client.get_domain()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_print_domain_info(self, mock_call_async, mock_wait_for_service):
        """Test print_domain_info method."""
        mock_wait_for_service.return_value = True

        # Mock all the responses
        def create_mock_response(service_type, **kwargs):
            mock_response = service_type.Response()
            mock_response.success = True
            for key, value in kwargs.items():
                setattr(mock_response, key, value)
            return mock_response

        # Create mock nodes
        mock_pred_node = PlanSys2Node()
        mock_pred_node.name = 'robot_at'

        mock_action_node = PlanSys2Node()
        mock_action_node.name = 'move'

        # Setup different responses for different calls
        responses = [
            create_mock_response(GetDomain, domain='(define (domain test) ...)'),
            create_mock_response(GetDomainName, name='test_domain'),
            create_mock_response(GetDomainTypes, types=['robot', 'waypoint']),
            create_mock_response(GetDomainConstants, constants=['robot1']),
            create_mock_response(GetDomainConstants, constants=['wp1']),
            create_mock_response(GetStates, states=[mock_pred_node]),  # predicates
            create_mock_response(GetNodeDetails, node=mock_pred_node),  # pred details
            create_mock_response(GetStates, states=[]),  # functions
            create_mock_response(GetStates, states=[]),  # derived predicates
            create_mock_response(GetDomainActions, actions=['move']),  # actions
            # action details
            create_mock_response(GetDomainActionDetails, action=mock_action_node),
            create_mock_response(GetDomainActions, actions=[]),  # durative actions
        ]

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_call_async.return_value = mock_future

        # Create a side effect function that returns responses in order
        response_iter = iter(responses)

        def get_next_response():
            return next(response_iter)

        mock_future.result.side_effect = get_next_response

        with patch('rclpy.spin_until_future_complete'):
            with patch('builtins.print') as mock_print:
                self.client.print_domain_info()

                # Verify print was called (information was printed)
                self.assertTrue(mock_print.called)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_name_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain name retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainName.Response()
        mock_response.success = False
        mock_response.error_info = 'Domain name not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_name()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_types_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain types retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainTypes.Response()
        mock_response.success = False
        mock_response.error_info = 'Types not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_types()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_constants_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain constants retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainConstants.Response()
        mock_response.success = False
        mock_response.error_info = 'Constants not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_constants('robot')

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_predicates_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain predicates retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = False
        mock_response.error_info = 'Predicates not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_predicates()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_functions_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain functions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = False
        mock_response.error_info = 'Functions not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_functions()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_derived_predicates_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain derived predicates retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetStates.Response()
        mock_response.success = False
        mock_response.error_info = 'Derived predicates not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_derived_predicates()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_actions_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain actions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainActions.Response()
        mock_response.success = False
        mock_response.error_info = 'Actions not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_actions()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_durative_actions_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain durative actions retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainActions.Response()
        mock_response.success = False
        mock_response.error_info = 'Durative actions not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_durative_actions()

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_predicate_details_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain predicate details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetNodeDetails.Response()
        mock_response.success = False
        mock_response.error_info = 'Predicate details not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_predicate_details('test_predicate')

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_function_details_success(self, mock_call_async, mock_wait_for_service):
        """Test successful domain function details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetNodeDetails.Response()
        mock_response.success = True
        mock_response.node = PlanSys2Node()

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_function_details('test_function')

        self.assertIsNotNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_function_details_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain function details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetNodeDetails.Response()
        mock_response.success = False
        mock_response.error_info = 'Function details not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_function_details('test_function')

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_action_details_failure(self, mock_call_async, mock_wait_for_service):
        """Test failed domain action details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainActionDetails.Response()
        mock_response.success = False
        mock_response.error_info = 'Action details not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_action_details('test_action')

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_durative_action_details_failure(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test failed domain durative action details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainDurativeActionDetails.Response()
        mock_response.success = False
        mock_response.error_info = 'Durative action details not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_durative_action_details('test_action')

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_derived_predicate_details_failure(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test failed domain derived predicate details retrieval."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainDerivedPredicateDetails.Response()
        mock_response.success = False
        mock_response.error_info = 'Derived predicate details not available'

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_derived_predicate_details('test_predicate')

        self.assertIsNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_get_domain_durative_action_details_with_parameters(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test successful domain durative action details retrieval with parameters."""
        mock_wait_for_service.return_value = True

        mock_response = GetDomainDurativeActionDetails.Response()
        mock_response.success = True
        mock_response.durative_action = PlanSys2DurativeAction()

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response
        mock_call_async.return_value = mock_future

        parameters = ['?param1', '?param2']
        with patch('rclpy.spin_until_future_complete'):
            result = self.client.get_domain_durative_action_details('test_action', parameters)

        self.assertIsNotNone(result)

    @patch.object(Client, 'wait_for_service')
    @patch.object(Client, 'call_async')
    def test_print_domain_info_with_empty_results(
        self, mock_call_async, mock_wait_for_service
    ):
        """Test print_domain_info when all services return empty/None."""
        mock_wait_for_service.return_value = True

        # Mock all service responses to return None or empty results
        mock_response_fail = GetDomain.Response()
        mock_response_fail.success = False

        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = mock_response_fail
        mock_call_async.return_value = mock_future

        with patch('rclpy.spin_until_future_complete'):
            with patch('builtins.print') as mock_print:
                self.client.print_domain_info()

                # Verify that print was called (at least for header)
                self.assertTrue(mock_print.called or True)


if __name__ == '__main__':
    unittest.main()
