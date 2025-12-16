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
PlanSys2 Domain Expert Client.

This module provides specific classes for interacting with PlanSys2 services.
"""

from typing import List, Optional

from plansys2_msgs.msg import Action as PlanSys2Action
from plansys2_msgs.msg import Derived
from plansys2_msgs.msg import DurativeAction as PlanSys2DurativeAction
from plansys2_msgs.msg import Node as PlanSys2Node
from plansys2_msgs.srv import (GetDomain, GetDomainActionDetails,
                               GetDomainActions, GetDomainConstants,
                               GetDomainDerivedPredicateDetails,
                               GetDomainDurativeActionDetails, GetDomainName,
                               GetDomainTypes, GetNodeDetails, GetStates)

import rclpy

from rclpy.client import Client
from rclpy.node import Node


class DomainExpertClient(Node):
    """
    Domain Expert client for PlanSys2.

    This class provides convenient methods to interact with all Domain
    Expert services in PlanSys2, wrapping the ROS2 service calls with
    proper error handling.
    """

    def __init__(
        self, node_name: str = 'domain_expert_client',
        namespace: str = ''
    ):
        """
        Initialize the Domain Expert client.

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

        log_msg = f'Domain Expert Client "{node_name}" initialized'
        self.get_logger().debug(log_msg)

    def _create_and_call_service(
        self, service_type, service_name: str, request
    ):
        """
        Create service client, call it and return response.

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
            client: Client = self.create_client(
                service_type, service_name
            )

            if not client.wait_for_service(timeout_sec=5.0):
                error_msg = f'Service {service_name} not available'
                self.get_logger().error(error_msg)
                return None

            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=10.0
            )

            if future.done():
                return future.result()
            else:
                error_msg = (
                    f'Service call to {service_name} timed out'
                )
                self.get_logger().error(error_msg)
                return None

        except RuntimeError as e:
            error_msg = (
                f'Error calling service {service_name}: {str(e)}'
            )
            self.get_logger().error(error_msg)
            return None

    def get_domain(self) -> Optional[str]:
        """
        Get the PDDL domain as a string.

        Returns
        -------
        Optional[str]
            The PDDL domain string or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain'
        request = GetDomain.Request()

        response = self._create_and_call_service(GetDomain, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully retrieved domain')
            return response.domain
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get domain: {error_msg}')
            return None

    def get_domain_name(self) -> Optional[str]:
        """
        Get the name of the domain.

        Returns
        -------
        Optional[str]
            The domain name or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_name'
        request = GetDomainName.Request()

        response = self._create_and_call_service(GetDomainName, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully retrieved domain name: {response.name}')
            return response.name
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get domain name: {error_msg}')
            return None

    def get_domain_types(self) -> Optional[List[str]]:
        """
        Get the valid types in the domain.

        Returns
        -------
        Optional[List[str]]
            List of domain types or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_types'
        request = GetDomainTypes.Request()

        response = self._create_and_call_service(GetDomainTypes, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully retrieved {len(response.types)} domain types')
            return list(response.types)
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get domain types: {error_msg}')
            return None

    def get_domain_constants(self, type_name: str) -> Optional[List[str]]:
        """
        Get the constants of a specific type.

        Parameters
        ----------
        type_name : str
            The type name to get constants for.

        Returns
        -------
        Optional[List[str]]
            List of constants or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_constants'
        request = GetDomainConstants.Request()
        request.type = type_name

        response = self._create_and_call_service(GetDomainConstants, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.constants)} constants for type: {type_name}'
            )
            return list(response.constants)
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get constants for type {type_name}: {error_msg}')
            return None

    def get_domain_predicates(self) -> Optional[List[PlanSys2Node]]:
        """
        Get the predicates in the domain.

        Returns
        -------
        Optional[List[PlanSys2Node]]
            List of predicate nodes or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_predicates'
        request = GetStates.Request()

        response = self._create_and_call_service(GetStates, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.states)} domain predicates')
            return response.states
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get domain predicates: {error_msg}')
            return None

    def get_domain_functions(self) -> Optional[List[PlanSys2Node]]:
        """
        Get the functions in the domain.

        Returns
        -------
        Optional[List[PlanSys2Node]]
            List of function nodes or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_functions'
        request = GetStates.Request()

        response = self._create_and_call_service(GetStates, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.states)} domain functions')
            return response.states
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get domain functions: {error_msg}')
            return None

    def get_domain_derived_predicates(self) -> Optional[List[PlanSys2Node]]:
        """
        Get the derived predicates in the domain.

        Returns
        -------
        Optional[List[PlanSys2Node]]
            List of derived predicate nodes or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_derived_predicates'
        request = GetStates.Request()

        response = self._create_and_call_service(GetStates, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.states)} derived predicates'
            )
            return response.states
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get derived predicates: {error_msg}')
            return None

    def get_domain_derived_predicate_details(self, predicate_name: str) -> Optional[List[Derived]]:
        """
        Get the details of a specific derived predicate.

        Parameters
        ----------
        predicate_name : str
            Name of the derived predicate.

        Returns
        -------
        Optional[List[Derived]]
            List of derived predicate details or None if failed.

        """
        service_name = (
            f'{self._namespace_prefix}/domain_expert/get_domain_derived_predicate_details'
        )
        request = GetDomainDerivedPredicateDetails.Request()
        request.predicate = predicate_name

        response = self._create_and_call_service(
            GetDomainDerivedPredicateDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved details for derived predicate: {predicate_name}')
            return response.predicates
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(
                f'Failed to get derived predicate details for {predicate_name}: {error_msg}')
            return None

    def get_domain_actions(self) -> Optional[List[str]]:
        """
        Get the available actions in the domain.

        Returns
        -------
        Optional[List[str]]
            List of action names or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_actions'
        request = GetDomainActions.Request()

        response = self._create_and_call_service(GetDomainActions, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.actions)} domain actions'
            )
            return list(response.actions)
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get domain actions: {error_msg}')
            return None

    def get_domain_durative_actions(self) -> Optional[List[str]]:
        """
        Get the durative actions in the domain.

        Returns
        -------
        Optional[List[str]]
            List of durative action names or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_durative_actions'
        request = GetDomainActions.Request()

        response = self._create_and_call_service(GetDomainActions, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.actions)} durative actions')
            return response.actions
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get durative actions: {error_msg}')
            return None

    def get_domain_predicate_details(self, predicate_name: str) -> Optional[PlanSys2Node]:
        """
        Get the details of a specific predicate.

        Parameters
        ----------
        predicate_name : str
            Name of the predicate.

        Returns
        -------
        Optional[PlanSys2Node]
            Predicate details or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_predicate_details'
        request = GetNodeDetails.Request()
        request.expression = predicate_name

        response = self._create_and_call_service(GetNodeDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved details for predicate: {predicate_name}')
            return response.node
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(
                f'Failed to get predicate details for {predicate_name}: {error_msg}')
            return None

    def get_domain_function_details(self, function_name: str) -> Optional[PlanSys2Node]:
        """
        Get the details of a specific function.

        Parameters
        ----------
        function_name : str
            Name of the function.

        Returns
        -------
        Optional[PlanSys2Node]
            Function details or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_function_details'
        request = GetNodeDetails.Request()
        request.expression = function_name

        response = self._create_and_call_service(GetNodeDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved details for function: {function_name}')
            return response.node
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(
                f'Failed to get function details for {function_name}: {error_msg}')
            return None

    def get_domain_action_details(
        self, action_name: str, parameters: Optional[List[str]] = None
    ) -> Optional[PlanSys2Action]:
        """
        Get the details of a specific action.

        Parameters
        ----------
        action_name : str
            Name of the action.
        parameters : Optional[List[str]], optional
            Optional list of parameters.

        Returns
        -------
        Optional[PlanSys2Action]
            Action details or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_action_details'
        request = GetDomainActionDetails.Request()
        request.action = action_name
        if parameters:
            request.parameters = parameters

        response = self._create_and_call_service(GetDomainActionDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully retrieved details for action: {action_name}')
            return response.action
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get action details for {action_name}: {error_msg}')
            return None

    def get_domain_durative_action_details(
            self, action_name: str, parameters: Optional[List[str]] = None
    ) -> Optional[PlanSys2DurativeAction]:
        """
        Get the details of a specific durative action.

        Parameters
        ----------
        action_name : str
            Name of the durative action.
        parameters : Optional[List[str]], optional
            Optional list of parameters.

        Returns
        -------
        Optional[PlanSys2DurativeAction]
            Durative action details or None if failed.

        """
        service_name = f'{self._namespace_prefix}/domain_expert/get_domain_durative_action_details'
        request = GetDomainDurativeActionDetails.Request()
        request.durative_action = action_name
        if parameters:
            request.parameters = parameters

        response = self._create_and_call_service(
            GetDomainDurativeActionDetails, service_name, request
        )
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved durative action details: {action_name}'
            )
            return response.durative_action
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(
                f'Failed to get durative action details for {action_name}: {error_msg}'
            )
            return None

    def print_domain_info(self) -> None:
        """
        Print comprehensive information about the domain.

        Notes
        -----
        Call multiple services to provide an overview of the domain.

        """
        self.get_logger().info('=== Domain Expert Information ===')

        # Get the full domain
        domain = self.get_domain()
        if domain:
            self.get_logger().info(f'Domain PDDL length: {len(domain)} characters')
            print(f'Domain preview:\n{domain[:30]}...')

        # Get domain name
        name = self.get_domain_name()
        if name:
            print(f'Domain Name: {name}')

        # Get domain types
        types = self.get_domain_types()
        if types:
            print(f'Types ({len(types)}): {", ".join(types)}')

        # Get constants for each type
        if types:
            for t in types:
                constants = self.get_domain_constants(t)
                if constants:
                    print(f'  - Constants for type {t} ({len(constants)}): {", ".join(constants)}')

        # Get predicates
        predicates = self.get_domain_predicates()
        if predicates:
            predicate_names = [p.name for p in predicates]
            print(f'Predicates ({len(predicate_names)}): {", ".join(predicate_names)}')
            for pred in predicates:
                details = self.get_domain_predicate_details(pred.name)
                print(f'  - Predicate details for {pred.name}: {details}')

        # Get functions
        functions = self.get_domain_functions()
        if functions:
            function_names = [f.name for f in functions]
            print(f'Functions ({len(function_names)}): {", ".join(function_names)}')
            for func in functions:
                details = self.get_domain_function_details(func.name)
                print(f'  - Function details for {func.name}: {details}')

        # Get derived predicates
        derived_predicates = self.get_domain_derived_predicates()
        if derived_predicates:
            derived_predicate_names = [d.name for d in derived_predicates]
            print(
                f'Derived Predicates ({len(derived_predicate_names)}):'
                f' {", ".join(derived_predicate_names)}')
            for dpred in derived_predicates:
                derived_details = self.get_domain_derived_predicate_details(dpred.name)
                print(f'  - Derived predicate details for {dpred.name}: {derived_details}')

        # Get domain actions
        actions = self.get_domain_actions()
        if actions:
            print(f'Actions ({len(actions)}): {", ".join(actions)}')
            for action in actions:
                action_details = self.get_domain_action_details(action)
                print(f'  - Action details for {action}: {action_details}')

        # Get durative actions
        durative_actions = self.get_domain_durative_actions()
        if durative_actions:
            print(f'Durative Actions ({len(durative_actions)}): {", ".join(durative_actions)}')
            for daction in durative_actions:
                durative_details = self.get_domain_durative_action_details(daction)
                print(f'  - Durative action details for {daction}: {durative_details}')
