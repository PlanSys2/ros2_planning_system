# Copyright 2026 Intelligent Robotics Lab
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

"""Common base of the PlanSys2 Python clients."""

import threading
import time
from typing import Any, Dict, Optional

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


# What a component answers (error_info) to a request it does not take because it is not active
NOT_ACTIVE = 'Requesting service in non-active state'


class ClientBase:
    """
    Calls PlanSys2 services on the node of the application, as the C++ clients do.

    rclpy executors take whole nodes, not callback groups: its service and action clients go in
    a node of its own, in the namespace of the node of the application, which the client serves
    while it waits for an answer. So it calls from anywhere: before the node of the application
    is spun, or from a callback of any executor, single threaded too. Names are relative, so the
    namespace applies.

    As the C++ clients: every call waits until its component is active, however long it takes
    while ROS runs (seen active, it is not asked again until a service of it goes or stops
    answering), and a request without answer is sent again, up to three times, each with a new
    client: some middlewares lose every answer to a client whose endpoints were matched badly.
    """

    _ATTEMPTS = 3
    # A component that says it is active and answers that it is not, so many times: an error
    _NOT_ACTIVE_WAITS = 3
    _PERIOD = 0.5
    _LOG_PERIOD = 5.0
    _SLICE = 0.05
    _SERVED_AT_ONCE = 20

    def __init__(self, node: Node, component: str) -> None:
        """
        Initialize the client.

        Parameters
        ----------
        node : Node
            The node of the application the client works for.
        component : str
            The PlanSys2 component it calls (domain_expert, problem_expert, planner, executor).

        """
        self._application_node = node
        # Its own node, without the arguments of the process: a remapped name would repeat
        self._node = rclpy.create_node(
            f'{node.get_name()}_{component}_client', namespace=node.get_namespace(),
            context=node.context, use_global_arguments=False,
            parameter_overrides=[
                Parameter('use_sim_time', value=node.get_parameter('use_sim_time').value)])
        self._executor = SingleThreadedExecutor(context=node.context)
        self._executor.add_node(self._node)
        self._serve_lock = threading.Lock()
        self._component = component
        self._active = False
        self._active_lock = threading.Lock()
        self._group = MutuallyExclusiveCallbackGroup()
        self._service_clients: Dict[str, Client] = {}

    @property
    def node(self) -> Node:
        """Return the node of the application the client works for."""
        return self._application_node

    def get_logger(self):
        """Return the logger of the node of the application."""
        return self._application_node.get_logger()

    def destroy(self) -> None:
        """Destroy the client and its node; the node of the application is left as it was."""
        for client in self._service_clients.values():
            self._node.destroy_client(client)
        self._service_clients.clear()
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        self._node.destroy_node()

    def wait_active(self) -> bool:
        """
        Wait until the component is active, as every call does.

        Returns
        -------
        bool
            True once active; False if ROS shuts down first.

        """
        if self._active:
            return True
        with self._active_lock:
            last_log = time.monotonic()
            while not self._active and rclpy.ok():
                # At most one question every _PERIOD, also when a wait ends at once
                next_ask = time.monotonic() + self._PERIOD
                if self._ask_active():
                    self._active = True
                    break
                time.sleep(max(0.0, next_ask - time.monotonic()))
                if time.monotonic() - last_log > self._LOG_PERIOD:
                    self.get_logger().info(f'Waiting for {self._component} to be active...')
                    last_log = time.monotonic()
        return self._active

    def _lost(self) -> None:
        """Wait for the component again: a service of it went or stopped answering."""
        self._active = False

    def _ask_active(self) -> bool:
        """Ask the component whether it is active."""
        service_name = f'{self._component}/get_state'
        client = self._service_client(GetState, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        future = client.call_async(GetState.Request())
        if not self._wait_for(future, 1.0):
            client.remove_pending_request(future)
            self._renew(GetState, service_name)
            return False
        return future.result().current_state.id == State.PRIMARY_STATE_ACTIVE

    def _renew(self, service_type, service_name: str) -> Client:
        """Replace the client of a service by a new one: new endpoints, discovered again."""
        old = self._service_clients.pop(service_name, None)
        if old is not None:
            self._node.destroy_client(old)
        return self._service_client(service_type, service_name)

    def _service_client(self, service_type, service_name: str) -> Client:
        """
        Return the client of a service, created the first time.

        Parameters
        ----------
        service_type : Type
            The ROS2 service type class.
        service_name : str
            The name of the service, relative to the namespace.

        Returns
        -------
        Client
            The client.

        """
        client = self._service_clients.get(service_name)
        if client is None or client.srv_type is not service_type:
            client = self._node.create_client(
                service_type, service_name, callback_group=self._group)
            self._service_clients[service_name] = client
        return client

    def _wait_for(self, future, timeout_sec: Optional[float]) -> bool:
        """
        Wait until a future is done or the timeout expires.

        Parameters
        ----------
        future : Future
            The future.
        timeout_sec : float, optional
            Seconds to wait; None waits forever.

        Returns
        -------
        bool
            Whether the future is done.

        """
        end = None if timeout_sec is None else time.monotonic() + timeout_sec
        while not future.done() and rclpy.ok(context=self._node.context):
            left = self._SLICE if end is None else min(self._SLICE, end - time.monotonic())
            if left <= 0.0:
                break
            # One thread serves the node of the client; the others see their answers come
            if self._serve_lock.acquire(blocking=False):
                try:
                    self._executor.spin_until_future_complete(future, timeout_sec=left)
                finally:
                    self._serve_lock.release()
            else:
                done = threading.Event()
                future.add_done_callback(lambda _: done.set())
                done.wait(left)
        return future.done()

    def _serve(self) -> None:
        """Bring what came for the client without waiting: feedback and results."""
        if self._serve_lock.acquire(blocking=False):
            try:
                for _ in range(self._SERVED_AT_ONCE):
                    self._executor.spin_once(timeout_sec=0.0)
            finally:
                self._serve_lock.release()

    def _create_and_call_service(
            self, service_type, service_name: str, request,
            timeout_sec: float = 10.0, attempts: int = _ATTEMPTS) -> Optional[Any]:
        """
        Call a service and return its response.

        Parameters
        ----------
        service_type : Type
            The ROS2 service type class.
        service_name : str
            The name of the service, relative to the namespace.
        request : Any
            The service request object.
        timeout_sec : float, optional
            How long each attempt waits for the response.
        attempts : int, optional
            How many times the request is sent, each with a new client but the first; more than
            one only for requests that can be repeated.

        Returns
        -------
        Any
            The service response or None if failed.

        """
        try:
            attempt = 1
            waited_again = False
            not_active_waits = 0
            while attempt <= attempts and rclpy.ok():
                if not self.wait_active():
                    return None
                client = self._service_client(service_type, service_name)
                if not client.wait_for_service(timeout_sec=5.0):
                    if waited_again:
                        # Active and still without this service: an error
                        self.get_logger().error(f'Service {client.srv_name} not available')
                        return None
                    # Gone (PlanSys2 stopped or restarted): back to waiting for it, once
                    self._lost()
                    waited_again = True
                    continue
                future = client.call_async(request)
                if self._wait_for(future, timeout_sec):
                    response = future.result()
                    if getattr(response, 'error_info', None) == NOT_ACTIVE and \
                            not_active_waits < self._NOT_ACTIVE_WAITS:
                        # Deactivated since it was seen active: wait for it again, ask again
                        not_active_waits += 1
                        self._lost()
                        continue
                    return response
                future.cancel()
                client.remove_pending_request(future)
                self._renew(service_type, service_name)
                if attempt < attempts:
                    self.get_logger().warning(
                        f'{client.srv_name}: no response, sending it again with a new client '
                        f'({attempt + 1}/{attempts})')
                attempt += 1
            self._lost()  # The next call checks that it is still active
            self.get_logger().error(f'Service call to {service_name}: no response')
            return None
        except Exception as e:
            self.get_logger().error(f'Error calling service {service_name}: {str(e)}')
            return None
