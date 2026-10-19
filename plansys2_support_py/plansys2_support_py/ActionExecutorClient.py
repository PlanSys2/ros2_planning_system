# Copyright 2023 Intelligent Robotics Lab
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


"""Action Executor Client for PlanSys2."""

import copy

from lifecycle_msgs.msg import State
from plansys2_msgs.msg import ActionExecution, ActionPerformerStatus
from plansys2_support_py.Parser import Parser
from rclpy.exceptions import ParameterUninitializedException
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ActionExecutorClient(LifecycleNode):
    """Client for executing actions in PlanSys2."""

    def __init__(self, node_name, rate, **kwargs):
        """
        Initialize the ActionExecutorClient.

        Parameters
        ----------
        node_name : str
            Name of the node.
        rate : float
            Period of do_work in seconds, as the C++ constructor. The 'rate' parameter, in Hz as
            in C++, overrides it.
        **kwargs
            Passed to the node, such as namespace or parameter_overrides.

        """
        super().__init__(node_name, **kwargs)

        self._default_rate = 1.0 / rate if rate > 0.0 else 1.0
        self.declare_parameter('action_name', '')
        self.declare_parameter('specialized_arguments', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('rate', self._default_rate)

        self.status = ActionPerformerStatus()
        self.status.state = ActionPerformerStatus.NOT_READY
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.status.node_name = self.get_name()
        self.commited = False
        # The request we answered and wait to be confirmed or rejected for
        self.committed_request_id = ''
        # A lost CONFIRM or REJECT does not keep us committed for ever
        self.commitment_timeout = None
        # The request we execute: its feedback and finish carry it
        self.current_request_id = ''
        # The duration the plan gives the action, from its start; a new plan may change it
        self.current_planned_duration = 0.0

    def get_planned_duration(self):
        """
        Return the duration the plan gives the action, from its start.

        Returns
        -------
        float
            In seconds; 0 for a non-durative action.

        """
        return self.current_planned_duration

    def release_commitment(self):
        """Stop waiting for a CONFIRM or REJECT."""
        self.commited = False
        self.committed_request_id = ''
        if self.commitment_timeout is not None:
            self.commitment_timeout.cancel()
            self.destroy_timer(self.commitment_timeout)
            self.commitment_timeout = None

    def commitment_expired(self):
        """Become available again: nobody answered our response."""
        self.get_logger().warning('No answer to our response: available again')
        self.release_commitment()

    def on_configure(
        self, state: LifecycleState
    ) -> TransitionCallbackReturn:
        """
        Configure the node.

        Parameters
        ----------
        state : LifecycleState
            Current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS or FAILURE.

        """
        self.statuspub = self.create_publisher(
            ActionPerformerStatus, 'performers_status',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, depth=100
            )
        )

        def timer_callback():
            self.status.status_stamp = self.get_clock().now().to_msg()
            self.statuspub.publish(self.status)
        self.hearbeat_pub = self.create_timer(1.0, timer_callback)

        self.action_managed = self.get_parameter('action_name').value
        if self.action_managed == '':
            self.get_logger().error(
                'action_name parameter not set'
            )
            self.status.state = ActionPerformerStatus.FAILURE
            self.status.status_stamp = self.get_clock().now().to_msg()
            return TransitionCallbackReturn.FAILURE

        self.rate = self.get_parameter('rate').value
        if not isinstance(self.rate, (int, float)) or self.rate <= 0.0:
            self.get_logger().error(
                f'rate must be a positive number of Hz ({self.rate}), using {self._default_rate}')
            self.rate = self._default_rate
        try:
            specialized_args = self.get_parameter('specialized_arguments').value
        except ParameterUninitializedException:
            specialized_args = None  # Declared without a value: not specialized
        self.specialized_arguments = list(specialized_args) if specialized_args else []

        self.action_hub_pub = self.create_publisher(
            ActionExecution, 'actions_hub',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=100)
        )

        self.action_hub_sub = self.create_subscription(
            ActionExecution, 'actions_hub',
            self.action_hub_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, depth=100
            )
        )

        self.status.state = ActionPerformerStatus.READY
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.status.action = self.action_managed
        specialized_args = self.specialized_arguments
        self.status.specialized_arguments = specialized_args

        return TransitionCallbackReturn.SUCCESS

    def on_activate(
        self, state: LifecycleState
    ) -> TransitionCallbackReturn:
        """
        Activate the node.

        Parameters
        ----------
        state : LifecycleState
            Current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS.

        """
        self.status.state = ActionPerformerStatus.RUNNING
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.timer = self.create_timer(1.0 / self.rate, self.do_work)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(
        self, state: LifecycleState
    ) -> TransitionCallbackReturn:
        """
        Deactivate the node.

        Parameters
        ----------
        state : LifecycleState
            Current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS.

        """
        self.status.state = ActionPerformerStatus.READY
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.timer.destroy()

        return TransitionCallbackReturn.SUCCESS

    def action_hub_callback(self, msg: ActionExecution):
        """
        Handle action hub messages.

        Parameters
        ----------
        msg : ActionExecution
            Action execution message from the hub.

        """
        if msg.type == ActionExecution.REQUEST:
            curr_state = self._state_machine.current_state[0]
            state_inactive = curr_state == State.PRIMARY_STATE_INACTIVE
            should_exec = self.should_execute(msg.action, msg.arguments)
            if state_inactive and not self.commited and should_exec:
                self.commited = True
                self.committed_request_id = msg.request_id
                self.commitment_timeout = self.create_timer(5.0, self.commitment_expired)
                self.send_response(msg)
        elif msg.type == ActionExecution.CONFIRM:
            curr_state = self._state_machine.current_state[0]
            state_inactive = curr_state == State.PRIMARY_STATE_INACTIVE
            node_match = msg.node_id == self.get_name()
            request_match = msg.request_id == self.committed_request_id
            if state_inactive and self.commited and node_match and request_match:
                self.current_arguments = msg.arguments
                self.current_planned_duration = msg.planned_duration
                self.current_request_id = msg.request_id
                self.release_commitment()
                self.trigger_activate()
        elif msg.type == ActionExecution.REJECT:
            node_match = msg.node_id == self.get_name()
            if self.commited and node_match and msg.request_id == self.committed_request_id:
                self.release_commitment()
        elif msg.type == ActionExecution.CANCEL:
            curr_state = self._state_machine.current_state[0]
            state_active = curr_state == State.PRIMARY_STATE_ACTIVE
            node_match = msg.node_id == self.get_name()
            node_match = node_match and msg.request_id == self.current_request_id
            if state_active and node_match:
                self.trigger_deactivate()
        elif msg.type == ActionExecution.UPDATE:
            curr_state = self._state_machine.current_state[0]
            state_active = curr_state == State.PRIMARY_STATE_ACTIVE
            node_match = msg.node_id == self.get_name()
            if state_active and node_match and msg.request_id == self.current_request_id:
                self.current_planned_duration = msg.planned_duration
        elif (msg.type in [ActionExecution.RESPONSE,
                           ActionExecution.FEEDBACK,
                           ActionExecution.FINISH]):
            pass
        else:
            error_msg = (
                f'Msg {msg.type} type not recognized in '
                f'{self.get_name()} executor performer'
            )
            self.get_logger().error(error_msg)

    def should_execute(self, action, args) -> bool:
        """
        Check if action should be executed with given arguments.

        Parameters
        ----------
        action : str
            Action name.
        args : list
            Action arguments.

        Returns
        -------
        bool
            True if action should be executed.

        """
        # PDDL is case insensitive: Move performs move (#256)
        if not Parser.compare_str_case_insensitive(action, self.action_managed):
            return False
        if len(self.specialized_arguments) > 0:
            if len(self.specialized_arguments) != len(args):
                warning_msg = (
                    f'current and specialized arguments length '
                    f'does not match {len(args)} '
                    f'{len(self.specialized_arguments)}'
                )
                self.get_logger().warning(warning_msg)
            for specialized, arg in zip(self.specialized_arguments, args):
                args_differ = not Parser.compare_str_case_insensitive(specialized, arg)
                if specialized != '' and arg != '' and args_differ:
                    return False
        return True

    def do_work(self):
        """
        Execute work (should be overridden by subclass).

        Notes
        -----
        This method should be overridden by subclasses to implement
        the actual action execution logic.

        """
        print('do_work not overriden')

    def send_response(self, msg: ActionExecution):
        """
        Send response message.

        Parameters
        ----------
        msg : ActionExecution
            Original action execution message.

        """
        msg_resp = ActionExecution()
        msg_resp = copy.copy(msg)
        msg_resp.type = ActionExecution.RESPONSE
        msg_resp.node_id = self.get_name()

        self.action_hub_pub.publish(msg_resp)

    def send_feedback(self, completion, status):
        """
        Send feedback message.

        Parameters
        ----------
        completion : float
            Action completion percentage.
        status : str
            Action status.

        """
        msg_resp = ActionExecution()
        msg_resp.type = ActionExecution.FEEDBACK
        msg_resp.node_id = self.get_name()
        msg_resp.request_id = self.current_request_id
        msg_resp.action = self.action_managed
        msg_resp.arguments = self.current_arguments
        msg_resp.completion = completion
        msg_resp.status = status

        self.action_hub_pub.publish(msg_resp)

    def finish(self, success, completion, status):
        """
        Finish action execution.

        Parameters
        ----------
        success : bool
            Whether action succeeded.
        completion : float
            Action completion percentage.
        status : str
            Action status.

        """
        curr_state = self._state_machine.current_state[0]
        if curr_state == State.PRIMARY_STATE_ACTIVE:
            self.trigger_deactivate()

        msg_resp = ActionExecution()
        msg_resp.type = ActionExecution.FINISH
        msg_resp.node_id = self.get_name()
        msg_resp.request_id = self.current_request_id
        msg_resp.action = self.action_managed
        msg_resp.arguments = self.current_arguments
        msg_resp.completion = completion
        msg_resp.status = status
        msg_resp.success = success

        self.action_hub_pub.publish(msg_resp)
