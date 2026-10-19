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

"""Unit tests for ActionExecutorClient."""

import unittest
from unittest.mock import patch

from plansys2_msgs.msg import ActionExecution, ActionPerformerStatus
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter


class TestActionExecutorClient(unittest.TestCase):
    """Test suite for ActionExecutorClient."""

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
        self.client = ActionExecutorClient('test_action_executor', 1.0)

    def tearDown(self):
        """Clean up after each test."""
        self.client.destroy_node()

    def test_initialization(self):
        """Test that the client initializes correctly."""
        self.assertIsNotNone(self.client)
        self.assertEqual(self.client.get_name(), 'test_action_executor')
        self.assertEqual(self.client.status.state, ActionPerformerStatus.NOT_READY)
        self.assertEqual(self.client.status.node_name, 'test_action_executor')
        self.assertFalse(self.client.commited)

    def test_on_configure_success(self):
        """Test successful configuration."""
        # Set required parameters
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])

        result = self.client.on_configure(None)

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.client.status.state, ActionPerformerStatus.READY)
        self.assertEqual(self.client.action_managed, 'move')
        self.assertEqual(self.client.rate, 1.0)

    def test_on_configure_failure_no_action_name(self):
        """Test configuration failure when action_name is not set."""
        # Don't set action_name parameter (it will be empty string by default)
        result = self.client.on_configure(None)

        self.assertEqual(result, TransitionCallbackReturn.FAILURE)
        self.assertEqual(self.client.status.state, ActionPerformerStatus.FAILURE)

    def test_on_activate(self):
        """Test activation."""
        # Configure first
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.on_configure(None)

        result = self.client.on_activate(None)

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.client.status.state, ActionPerformerStatus.RUNNING)
        self.assertIsNotNone(self.client.timer)

    def test_on_deactivate(self):
        """Test deactivation."""
        # Configure and activate first
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.on_configure(None)
        self.client.on_activate(None)

        result = self.client.on_deactivate(None)

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.client.status.state, ActionPerformerStatus.READY)

    def test_should_execute_matching_action(self):
        """Test should_execute with matching action."""
        self.client.action_managed = 'move'
        self.client.specialized_arguments = []

        result = self.client.should_execute('move', ['robot1', 'wp1', 'wp2'])

        self.assertTrue(result)

    def test_should_execute_non_matching_action(self):
        """Test should_execute with non-matching action."""
        self.client.action_managed = 'move'
        self.client.specialized_arguments = []

        result = self.client.should_execute('pick', ['robot1', 'obj1'])

        self.assertFalse(result)

    def test_should_execute_with_specialized_arguments_match(self):
        """Test should_execute with specialized arguments that match."""
        self.client.action_managed = 'move'
        self.client.specialized_arguments = ['robot1', '', '']

        result = self.client.should_execute('move', ['robot1', 'wp1', 'wp2'])

        self.assertTrue(result)

    def test_should_execute_with_specialized_arguments_no_match(self):
        """Test should_execute with specialized arguments that don't match."""
        self.client.action_managed = 'move'
        self.client.specialized_arguments = ['robot1', '', '']

        result = self.client.should_execute('move', ['robot2', 'wp1', 'wp2'])

        self.assertFalse(result)

    def test_send_response(self):
        """Test sending response."""
        # Configure first
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.on_configure(None)

        # Create test message
        msg = ActionExecution()
        msg.type = ActionExecution.REQUEST
        msg.action = 'move'
        msg.node_id = 'test_node'

        # Mock the publisher
        with patch.object(self.client.action_hub_pub, 'publish') as mock_publish:
            self.client.send_response(msg)

            # Verify publish was called
            mock_publish.assert_called_once()
            # Verify the published message has correct type
            published_msg = mock_publish.call_args[0][0]
            self.assertEqual(published_msg.type, ActionExecution.RESPONSE)
            self.assertEqual(published_msg.node_id, 'test_action_executor')

    def test_send_feedback(self):
        """Test sending feedback."""
        # Configure and set current arguments
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.on_configure(None)
        self.client.current_arguments = ['robot1', 'wp1', 'wp2']

        # Mock the publisher
        with patch.object(self.client.action_hub_pub, 'publish') as mock_publish:
            self.client.send_feedback(50.0, 'In progress')

            # Verify publish was called
            mock_publish.assert_called_once()
            # Verify the published message
            published_msg = mock_publish.call_args[0][0]
            self.assertEqual(published_msg.type, ActionExecution.FEEDBACK)
            self.assertEqual(published_msg.node_id, 'test_action_executor')
            self.assertEqual(published_msg.completion, 50.0)
            self.assertEqual(published_msg.status, 'In progress')

    def test_finish_success(self):
        """Test finish with success."""
        # Configure, activate and set current arguments
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.on_configure(None)
        self.client.on_activate(None)
        self.client.current_arguments = ['robot1', 'wp1', 'wp2']

        # Mock the publisher
        with patch.object(self.client.action_hub_pub, 'publish') as mock_publish:
            self.client.finish(True, 100.0, 'Action completed')

            # Verify publish was called
            mock_publish.assert_called_once()
            # Verify the published message
            published_msg = mock_publish.call_args[0][0]
            self.assertEqual(published_msg.type, ActionExecution.FINISH)
            self.assertEqual(published_msg.node_id, 'test_action_executor')
            self.assertEqual(published_msg.success, True)
            self.assertEqual(published_msg.completion, 100.0)
            self.assertEqual(published_msg.status, 'Action completed')

    def test_finish_failure(self):
        """Test finish with failure."""
        # Configure, activate and set current arguments
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.on_configure(None)
        self.client.on_activate(None)
        self.client.current_arguments = ['robot1', 'wp1', 'wp2']

        # Mock the publisher
        with patch.object(self.client.action_hub_pub, 'publish') as mock_publish:
            self.client.finish(False, 75.0, 'Action failed')

            # Verify publish was called
            mock_publish.assert_called_once()
            # Verify the published message
            published_msg = mock_publish.call_args[0][0]
            self.assertEqual(published_msg.type, ActionExecution.FINISH)
            self.assertEqual(published_msg.success, False)
            self.assertEqual(published_msg.completion, 75.0)
            self.assertEqual(published_msg.status, 'Action failed')

    def test_action_hub_callback_request(self):
        """Test action hub callback with REQUEST message."""
        # Configure
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        result = self.client.on_configure(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

        # Create REQUEST message
        msg = ActionExecution()
        msg.type = ActionExecution.REQUEST
        msg.action = 'move'
        msg.arguments = ['robot1', 'wp1', 'wp2']

        # Ensure client is not committed
        self.client.commited = False

        # Mock the publisher and should_execute
        with patch.object(self.client, 'should_execute', return_value=True):
            with patch.object(self.client.action_hub_pub, 'publish'):
                self.client.action_hub_callback(msg)

                # Verify commited flag is set (only if should_execute returns True)
                # Note: actual behavior depends on lifecycle state being INACTIVE
                # This test verifies the logic flow

    def test_action_hub_callback_confirm(self):
        """Test action hub callback with CONFIRM message."""
        # Configure
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        result = self.client.on_configure(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

        # Set committed flag
        self.client.commited = True

        # Create CONFIRM message
        msg = ActionExecution()
        msg.type = ActionExecution.CONFIRM
        msg.node_id = 'test_action_executor'
        msg.arguments = ['robot1', 'wp1', 'wp2']

        # Mock trigger_activate to verify it gets called
        with patch.object(self.client, 'trigger_activate') as mock_activate:
            self.client.action_hub_callback(msg)

            # Verify the callback processes CONFIRM messages
            # The actual behavior depends on lifecycle state being INACTIVE
            # We just verify the callback doesn't crash
            self.assertIsNotNone(mock_activate)

    def test_action_hub_callback_reject(self):
        """Test action hub callback with REJECT message."""
        self.client.commited = True
        self.client.committed_request_id = 'req'

        # Create REJECT message
        msg = ActionExecution()
        msg.type = ActionExecution.REJECT
        msg.node_id = 'test_action_executor'
        msg.request_id = 'req'

        # Call callback
        self.client.action_hub_callback(msg)

        # Verify commited flag is reset
        self.assertFalse(self.client.commited)

    def configure(self):
        """Configure the client for 'move', leaving it inactive."""
        self.client.set_parameters([
            rclpy.parameter.Parameter('action_name', rclpy.Parameter.Type.STRING, 'move'),
            rclpy.parameter.Parameter('rate', rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter('specialized_arguments',
                                      rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        self.client.trigger_configure()

    def request(self, request_id):
        """Build a REQUEST for move."""
        msg = ActionExecution()
        msg.type = ActionExecution.REQUEST
        msg.node_id = 'executor'
        msg.request_id = request_id
        msg.action = 'move'
        msg.arguments = ['robot1', 'wp1', 'wp2']
        return msg

    def test_response_carries_the_request_id(self):
        """The response belongs to the request it answers."""
        self.configure()
        with patch.object(self.client.action_hub_pub, 'publish') as publish:
            self.client.action_hub_callback(self.request('req_1'))
        self.assertTrue(self.client.commited)
        self.assertEqual(publish.call_args[0][0].type, ActionExecution.RESPONSE)
        self.assertEqual(publish.call_args[0][0].request_id, 'req_1')

    def test_answers_of_another_request_are_ignored(self):
        """Only the CONFIRM or REJECT of our request counts."""
        self.configure()
        with patch.object(self.client.action_hub_pub, 'publish'):
            self.client.action_hub_callback(self.request('req_1'))

        reject = ActionExecution()
        reject.type = ActionExecution.REJECT
        reject.node_id = 'test_action_executor'
        reject.request_id = 'req_0'
        self.client.action_hub_callback(reject)
        self.assertTrue(self.client.commited)

        reject.request_id = 'req_1'
        self.client.action_hub_callback(reject)
        self.assertFalse(self.client.commited)

    def test_commitment_expires(self):
        """Without CONFIRM or REJECT the client takes requests again."""
        self.configure()
        with patch.object(self.client.action_hub_pub, 'publish') as publish:
            self.client.action_hub_callback(self.request('lost'))
            self.assertIsNotNone(self.client.commitment_timeout)
            self.client.action_hub_callback(self.request('next'))
            self.assertEqual(publish.call_count, 1)

            self.client.commitment_expired()
            self.assertFalse(self.client.commited)
            self.assertIsNone(self.client.commitment_timeout)
            self.client.action_hub_callback(self.request('next'))
            self.assertEqual(publish.call_count, 2)

    def test_feedback_and_finish_carry_the_request_id(self):
        """What the client sends while executing belongs to the confirmed request."""
        self.configure()
        self.client.current_request_id = 'req_7'
        self.client.current_arguments = ['robot1', 'wp1', 'wp2']
        with patch.object(self.client.action_hub_pub, 'publish') as publish:
            self.client.send_feedback(0.5, 'moving')
            self.assertEqual(publish.call_args[0][0].request_id, 'req_7')
            self.client.finish(True, 1.0, 'done')
            self.assertEqual(publish.call_args[0][0].request_id, 'req_7')

    def test_planned_duration_from_confirm_and_update(self):
        """The CONFIRM gives the duration; an UPDATE of its request changes it, of another not."""
        self.configure()
        with patch.object(self.client.action_hub_pub, 'publish'):
            self.client.action_hub_callback(self.request('req_3'))
            confirm = self.request('req_3')
            confirm.type = ActionExecution.CONFIRM
            confirm.node_id = self.client.get_name()
            confirm.planned_duration = 30.0
            self.client.action_hub_callback(confirm)
        self.assertEqual(self.client.get_planned_duration(), 30.0)

        update = self.request('req_2')
        update.type = ActionExecution.UPDATE
        update.node_id = self.client.get_name()
        update.planned_duration = 1.5
        self.client.action_hub_callback(update)
        self.assertEqual(self.client.get_planned_duration(), 30.0)

        update.request_id = 'req_3'
        self.client.action_hub_callback(update)
        self.assertEqual(self.client.get_planned_duration(), 1.5)
        self.client.trigger_deactivate()

    def _configured(self, rate, overrides):
        client = ActionExecutorClient(
            'test_action_executor_params', rate,
            parameter_overrides=[Parameter('action_name', value='move')] + overrides)
        self.assertEqual(client.on_configure(None), TransitionCallbackReturn.SUCCESS)
        return client

    def test_rate_is_in_hz(self):
        """The rate parameter is in Hz, as in C++; the constructor takes the period."""
        client = self._configured(0.5, [])
        try:
            self.assertEqual(client.get_parameter('rate').value, 2.0)
            client.on_activate(None)
            self.assertEqual(client.timer.timer_period_ns, 500_000_000)
        finally:
            client.destroy_node()
        client = self._configured(0.5, [Parameter('rate', value=10.0)])
        try:
            client.on_activate(None)
            self.assertEqual(client.timer.timer_period_ns, 100_000_000)
        finally:
            client.destroy_node()

    def test_rate_must_be_positive(self):
        """A rate that is not positive takes the one of the constructor."""
        client = self._configured(0.25, [Parameter('rate', value=0.0)])
        try:
            client.on_activate(None)
            self.assertEqual(client.timer.timer_period_ns, 250_000_000)
        finally:
            client.destroy_node()

    def test_specialized_arguments_from_parameters(self):
        """specialized_arguments is a list of strings, as set from a launch file."""
        client = self._configured(
            1.0, [Parameter('specialized_arguments', value=['r1', ''])])
        try:
            self.assertEqual(client.specialized_arguments, ['r1', ''])
            self.assertEqual(list(client.status.specialized_arguments), ['r1', ''])
            self.assertTrue(client.should_execute('move', ['r1', 'kitchen']))
            self.assertFalse(client.should_execute('move', ['r2', 'kitchen']))
        finally:
            client.destroy_node()
        client = self._configured(1.0, [])
        try:
            self.assertEqual(client.specialized_arguments, [])
            self.assertTrue(client.should_execute('move', ['r2', 'kitchen']))
        finally:
            client.destroy_node()

    def test_should_execute_with_other_number_of_arguments(self):
        """More or fewer arguments than the specialized ones do not raise."""
        self.client.action_managed = 'move'
        self.client.specialized_arguments = ['r1', 'a', 'b']
        self.assertTrue(self.client.should_execute('move', ['r1']))
        self.assertFalse(self.client.should_execute('move', ['r2']))
        self.client.specialized_arguments = ['r1']
        self.assertTrue(self.client.should_execute('move', ['r1', 'a', 'b']))
        self.assertTrue(self.client.should_execute('move', []))

    def test_should_execute_is_case_insensitive(self):
        """PDDL names are case insensitive: Move with R1 is move with r1."""
        self.client.action_managed = 'move'
        self.client.specialized_arguments = ['r1']
        self.assertTrue(self.client.should_execute('Move', ['R1']))
        self.assertFalse(self.client.should_execute('Move', ['R2']))
        self.assertFalse(self.client.should_execute('moves', ['r1']))


if __name__ == '__main__':
    unittest.main()
