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

from plansys2_msgs.msg import Plan, PlanArray
from plansys2_msgs.srv import GetPlan, GetPlanArray, ValidateDomain
from plansys2_support_py.ClientBase import ClientBase
from rclpy.node import Node


class PlannerClient(ClientBase):
    """
    Planner client for PlanSys2.

    This class provides convenient methods to interact with all Planner services
    in PlanSys2, wrapping the ROS2 service calls with proper error handling.
    """

    def __init__(self, node: Node) -> None:
        """
        Initialize the Planner client.

        Parameters
        ----------
        node : Node
            The node of the application the client works for.

        """
        super().__init__(node, 'planner')

        # As the C++ client: the planner may take its solver timeout, and some margin to answer
        # On the node of the application, unless it has it already
        if not node.has_parameter('plan_solver_timeout'):
            node.declare_parameter('plan_solver_timeout', 15.0)
        timeout = node.get_parameter('plan_solver_timeout').value
        if not isinstance(timeout, (int, float)) or timeout <= 0.0:
            timeout = 15.0
        self._answer_timeout = float(timeout) + 5.0

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
        service_name = 'planner/get_plan'
        request = GetPlan.Request()
        request.domain = domain
        request.problem = problem

        response = self._create_and_call_service(
            GetPlan, service_name, request, timeout_sec=self._answer_timeout, attempts=2)
        if response and response.success:
            self.get_logger().debug('Successfully generated plan')
            return response.plan
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to generate plan: {error_msg}')
        return None

    def get_plan_array(self, domain: str, problem: str) -> Optional[PlanArray]:
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
        Optional[PlanArray]
            The plan array or None if failed.

        """
        service_name = 'planner/get_plan_array'
        request = GetPlanArray.Request()
        request.domain = domain
        request.problem = problem

        response = self._create_and_call_service(
            GetPlanArray, service_name, request, timeout_sec=self._answer_timeout, attempts=2)
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
        service_name = 'planner/validate_domain'
        request = ValidateDomain.Request()
        request.domain = domain

        response = self._create_and_call_service(
            ValidateDomain, service_name, request, timeout_sec=self._answer_timeout)
        if response and response.success:
            self.get_logger().debug('Successfully validated domain')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to validate domain: {error_msg}')
            return False
