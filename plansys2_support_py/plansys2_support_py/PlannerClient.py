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
PlanSys2 Planner Client.

This module provides specific classes for interacting with PlanSys2 services.
"""

from typing import Optional

from plansys2_msgs.msg import Plan
from plansys2_msgs.srv import GetPlan, GetPlanArray, ValidateDomain

import rclpy
from rclpy.client import Client
from rclpy.node import Node


class PlannerClient(Node):
    """
    Planner client for PlanSys2.

    This class provides convenient methods to interact with all Planner services
    in PlanSys2, wrapping the ROS2 service calls with proper error handling.
    """

    def __init__(self, node_name: str = 'planner_client', namespace: str = ''):
        """
        Initialize the Planner client.

        Parameters
        ----------
        node_name : str, optional
            Name of the ROS2 node.
        namespace : str, optional
            Namespace prefix for services.

        """
        super().__init__(node_name)

        # Setup namespace prefix
        self._namespace_prefix = f'/{namespace}' if namespace else ''

        self.get_logger().debug(f'Domain Expert Client "{node_name}" initialized')

    def _create_and_call_service(self, service_type, service_name: str, request):
        """
        Create a service client for the given service, call it and return the response.

        Parameters
        ----------
        service_type : Type
            The ROS2 service type class.
        service_name : str
            The name/topic of the service.
        request : Any
            The service request object.

        Returns
        -------
        Any
            The service response or None if failed.

        """
        try:
            client: Client = self.create_client(service_type, service_name)

            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {service_name} not available')
                return None

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.done():
                return future.result()
            else:
                self.get_logger().error(f'Service call to {service_name} timed out')
                return None

        except Exception as e:
            self.get_logger().error(f'Error calling service {service_name}: {str(e)}')
            return None

    def get_plan(self, domain: str, problem: str) -> Optional[Plan]:
        """
        Get a plan that will satisfy the provided domain and problem.

        Parameters
        ----------
        domain : str
            The PDDL domain string.
        problem : str
            The PDDL problem string.

        Returns
        -------
        Optional[Plan]
            The plan response or None if failed.

        """
        service_name = f'{self._namespace_prefix}/planner/get_plan'
        request = GetPlan.Request()
        request.domain = domain
        request.problem = problem

        response = self._create_and_call_service(GetPlan, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully generated plan')
            return response.plan
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to generate plan: {error_msg}')
        return None

    def get_plan_array(self, domain: str, problem: str) -> Optional[GetPlanArray]:
        """
        Get a plan array that will satisfy the provided domain and problem.

        Parameters
        ----------
        domain : str
            The PDDL domain string.
        problem : str
            The PDDL problem string.

        Returns
        -------
        Optional[GetPlanArray]
            The plan array response or None if failed.

        """
        service_name = f'{self._namespace_prefix}/planner/get_plan_array'
        request = GetPlanArray.Request()
        request.domain = domain
        request.problem = problem

        response = self._create_and_call_service(GetPlanArray, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully retrieved plan array')
            return response.plan_array
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to retrieve plan array: {error_msg}')
            return None

    def validate_domain(self, domain: str) -> bool:
        """
        Validate the provided domain.

        Parameters
        ----------
        domain : str
            The PDDL domain string.

        Returns
        -------
        bool
            True if the domain is valid, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/planner/validate_domain'
        request = ValidateDomain.Request()
        request.domain = domain

        response = self._create_and_call_service(ValidateDomain, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully validated domain')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to validate domain: {error_msg}')
            return False
