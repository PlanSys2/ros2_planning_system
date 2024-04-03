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


import rclpy
import copy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from plansys2_msgs.msg import ActionExecution, ActionPerformerStatus, ActionExecutionInfo
from lifecycle_msgs.msg import State

class ActionExecutorClient(LifecycleNode):

    def __init__(self, node_name, rate):
        super().__init__(node_name)

        self.declare_parameter('action_name', '')
        self.declare_parameter('specialized_arguments', [])
        self.declare_parameter('rate', rate)

        self.status = ActionPerformerStatus()
        self.status.state = ActionPerformerStatus.NOT_READY
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.status.node_name = self.get_name()
        self.commited = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.statuspub = self.create_publisher(ActionPerformerStatus, 'performers_status',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,depth=100))
        
        def timer_callback():
            self.status.status_stamp = self.get_clock().now().to_msg()
            self.statuspub.publish(self.status)
        self.hearbeat_pub = self.create_timer(1.0, timer_callback)

        self.action_managed = self.get_parameter('action_name').value
        if self.action_managed == '':
            self.get_logger().error('action_name parameter not set')
            self.status.state = ActionPerformerStatus.FAILURE
            self.status.status_stamp = self.get_clock().now().to_msg()
            return TransitionCallbackReturn.FAILURE

        self.rate = self.get_parameter('rate').value
        self.specialized_arguments = self.get_parameter('specialized_arguments').value

        self.action_hub_pub = self.create_publisher(ActionExecution, 'actions_hub',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=100))

        self.action_hub_sub = self.create_subscription(ActionExecution, 'actions_hub', self.action_hub_callback,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,depth=100))

        self.status.state = ActionPerformerStatus.READY
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.status.action = self.action_managed
        self.status.specialized_arguments = self.specialized_arguments

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.status.state = ActionPerformerStatus.RUNNING
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.timer = self.create_timer(self.rate, self.do_work)

        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.status.state = ActionPerformerStatus.READY
        self.status.status_stamp = self.get_clock().now().to_msg()
        self.timer.destroy()

        return TransitionCallbackReturn.SUCCESS
    
    def action_hub_callback(self, msg: ActionExecution):
        if msg.type == ActionExecution.REQUEST:
            if (self._state_machine.current_state[0] == State.PRIMARY_STATE_INACTIVE and
                    not self.commited and self.should_execute(msg.action, msg.arguments)):
                self.commited = True
                self.send_response(msg)
        elif msg.type == ActionExecution.CONFIRM:
            if (self._state_machine.current_state[0] == State.PRIMARY_STATE_INACTIVE and
                    self.commited and msg.node_id == self.get_name()):
                self.current_arguments = msg.arguments
                self.trigger_activate()
                self.commited = False
        elif msg.type == ActionExecution.REJECT:
            if msg.node_id == self.get_name():
                self.commited = False
        elif msg.type == ActionExecution.CANCEL:
            if (self.self._state_machine.current_state[0] == State.PRIMARY_STATE_ACTIVE and
                    msg.node_id == self.get_name()):
                self.trigger_deactivate()
        elif msg.type == ActionExecution.RESPONSE or msg.type == ActionExecution.FEEDBACK or msg.type == ActionExecution.FINISH:
            pass
        else:
            self.get_logger().error('Msg %d type not recognized in %s executor performer'.format(msg.type, self.get_name()))

    def should_execute(self, action, args) -> bool:
        if action != self.action_managed:
            return False
        if len(self.specialized_arguments) > 0:
                if len(self.specialized_arguments) != len(args):
                   self.get_logger().warning('current and specialized arguments length doesnt match %zu %zu'.format(len(args), len(self.specialized_arguments)))
                for i in range(0, len(self.specialized_arguments)):
                    if self.specialized_arguments[i] != '' and args[i] != '' and self.specialized_arguments[i] != args[i]:
                        return False
        return True
    
    def do_work(self):
        print('do_work not overriden')

    def send_response(self, msg: ActionExecution):
        msg_resp = ActionExecution()
        msg_resp = copy.copy(msg)
        msg_resp.type = ActionExecution.RESPONSE
        msg_resp.node_id = self.get_name()

        self.action_hub_pub.publish(msg_resp)

    def send_feedback(self, completion, status):
        msg_resp = ActionExecution()
        msg_resp.type = ActionExecution.FEEDBACK
        msg_resp.node_id = self.get_name()
        msg_resp.action = self.action_managed
        msg_resp.arguments = self.current_arguments
        msg_resp.completion = completion
        msg_resp.status = status

        self.action_hub_pub.publish(msg_resp)

    def finish(self, success, completion, status):
        if self._state_machine.current_state[0] == State.PRIMARY_STATE_ACTIVE:
            self.trigger_deactivate()

        msg_resp = ActionExecution()
        msg_resp.type = ActionExecution.FINISH
        msg_resp.node_id = self.get_name()
        msg_resp.action = self.action_managed
        msg_resp.arguments = self.current_arguments
        msg_resp.completion = completion
        msg_resp.status = status
        msg_resp.success = success

        self.action_hub_pub.publish(msg_resp)
