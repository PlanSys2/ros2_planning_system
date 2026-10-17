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
from typing import Any, Dict, Optional

import rclpy
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class ClientBase(Node):
    """
    A node that calls PlanSys2 services, as the C++ clients do.

    The service and action clients live in a hidden node with its own executor, created once
    and kept: a call waits spinning only that node, so it works whether this node is spun by
    another executor or not, even from one of its callbacks. Names are relative, so the
    namespace and the remappings apply.
    """

    def __init__(self, node_name: str, namespace: str = '') -> None:
        """
        Initialize the client.

        Parameters
        ----------
        node_name : str
            Name of the ROS2 node.
        namespace : str, optional
            Namespace of the node, and so of the services it calls.

        """
        super().__init__(node_name=node_name, namespace=namespace)
        self._calls_node = rclpy.create_node(
            f'_{node_name}', namespace=namespace, context=self.context)
        self._calls_executor = SingleThreadedExecutor(context=self.context)
        self._calls_lock = threading.Lock()
        self._service_clients: Dict[str, Client] = {}

    def destroy_node(self) -> None:
        """Destroy this node and the one of its calls."""
        self._calls_executor.shutdown()
        self._calls_node.destroy_node()
        super().destroy_node()

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
            client = self._calls_node.create_client(service_type, service_name)
            self._service_clients[service_name] = client
        return client

    def _wait_for(self, future, timeout_sec: Optional[float]) -> bool:
        """
        Spin the calls of this client until a future is done or the timeout expires.

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
        with self._calls_lock:
            rclpy.spin_until_future_complete(
                self._calls_node, future, executor=self._calls_executor,
                timeout_sec=timeout_sec)
        return future.done()

    def _create_and_call_service(
            self, service_type, service_name: str, request,
            timeout_sec: float = 10.0) -> Optional[Any]:
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
            How long to wait for the response before giving up.

        Returns
        -------
        Any
            The service response or None if failed.

        """
        try:
            client = self._service_client(service_type, service_name)
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {client.srv_name} not available')
                return None
            future = client.call_async(request)
            if not self._wait_for(future, timeout_sec):
                future.cancel()
                client.remove_pending_request(future)
                self.get_logger().error(f'Service call to {client.srv_name} timed out')
                return None
            return future.result()
        except Exception as e:
            self.get_logger().error(f'Error calling service {service_name}: {str(e)}')
            return None
