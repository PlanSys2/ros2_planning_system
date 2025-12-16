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
PlanSys2 Problem Expert Client.

This module provides specific classes for interacting with PlanSys2 Problem Expert services.
"""

from typing import List, Optional

from plansys2_msgs.msg import Node as PlanSys2Node  # Avoid name conflict
from plansys2_msgs.msg import Param, Tree
# Problem Expert services
from plansys2_msgs.srv import (AddProblem, AddProblemGoal, AffectNode,
                               AffectParam, ClearProblemKnowledge, ExistNode,
                               GetNodeDetails, GetProblem, GetProblemGoal,
                               GetProblemInstanceDetails, GetProblemInstances,
                               GetStates, IsProblemGoalSatisfied,
                               RemoveProblemGoal)

import rclpy
from rclpy.client import Client
from rclpy.node import Node


class ProblemExpertClient(Node):
    """
    Problem Expert client for PlanSys2.

    This class provides convenient methods to interact with all Problem Expert services
    in PlanSys2, wrapping the ROS2 service calls with proper error handling.
    """

    def __init__(self, node_name: str = 'problem_expert_client', namespace: str = ''):
        """
        Initialize the Problem Expert client.

        Parameters
        ----------
        node_name : str, optional
            Name of the ROS2 node.
        namespace : str, optional
            Namespace prefix for services.

        """
        super().__init__(node_name)
        self._namespace_prefix = f'/{namespace}' if namespace else ''
        self.get_logger().debug(f'Problem Expert Client "{node_name}" initialized')

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

    def add_problem(self, problem: str) -> bool:
        """
        Add a PDDL problem.

        Parameters
        ----------
        problem : str
            The PDDL problem string.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/add_problem'
        request = AddProblem.Request()
        request.problem = problem

        response = self._create_and_call_service(AddProblem, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully added problem')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to add problem: {error_msg}')
            return False

    def add_problem_goal(self, goal: Tree) -> bool:
        """
        Add a goal to the problem.

        Parameters
        ----------
        goal : Tree
            The goal to add.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/add_problem_goal'
        request = AddProblemGoal.Request()
        request.tree = goal

        response = self._create_and_call_service(AddProblemGoal, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully added problem goal: {goal}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to add problem goal {goal}: {error_msg}')
            return False

    def add_problem_instance(self, instance: Param) -> bool:
        """
        Add an instance to the problem.

        Parameters
        ----------
        instance : Param
            The instance to add.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/add_problem_instance'
        request = AffectParam.Request()
        request.param = instance

        response = self._create_and_call_service(AffectParam, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully added problem instance: {instance}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to add problem instance {instance}: {error_msg}')
            return False

    def add_problem_predicate(self, predicate: PlanSys2Node) -> bool:
        """
        Add a predicate to the problem.

        Parameters
        ----------
        predicate : PlanSys2Node
            The predicate to add.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/add_problem_predicate'
        request = AffectNode.Request()
        request.node = predicate

        response = self._create_and_call_service(AffectNode, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully added problem predicate: {predicate}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to add problem predicate {predicate}: {error_msg}')
            return False

    def add_problem_function(self, function: PlanSys2Node) -> bool:
        """
        Add a function to the problem.

        Parameters
        ----------
        function : PlanSys2Node
            The function to add.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/add_problem_function'
        request = AffectNode.Request()
        request.node = function

        response = self._create_and_call_service(AffectNode, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully added problem function: {function}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to add problem function {function}: {error_msg}')
            return False

    def get_problem_goal(self) -> Optional[str]:
        """
        Get the goals in the problem as a PDDL string.

        Returns
        -------
        Optional[str]: The goals string or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_goal'
        request = GetProblemGoal.Request()

        response = self._create_and_call_service(GetProblemGoal, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully retrieved the problem goal {response.tree}')
            return response.tree
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem goals: {error_msg}')
            return None

    def get_problem_instance(self, instance: str) -> Optional[Param]:
        """
        Get the details of an instance.

        Parameters
        ----------
        instance : str
            The name of the instance to retrieve.

        Returns
        -------
        Optional[Param]: Details of the instance or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_instance'
        request = GetProblemInstanceDetails.Request()
        request.instance = instance

        response = self._create_and_call_service(GetProblemInstanceDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved problem instance {response.instance}')
            return response.instance
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem instance: {error_msg}')
            return None

    def get_problem_instances(self) -> Optional[List[Param]]:
        """
        Get the instances in the problem.

        Returns
        -------
        Optional[List[Param]]: List of instance or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_instances'
        request = GetProblemInstances.Request()

        response = self._create_and_call_service(GetProblemInstances, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.instances)} problem instances')
            return response.instances
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem instances: {error_msg}')
            return None

    def get_problem_predicate(self, predicate: str) -> Optional[PlanSys2Node]:
        """
        Get the details of a predicate.

        Parameters
        ----------
        predicate : str
            The name of the predicate to retrieve.

        Returns
        -------
        Optional[PlanSys2Node]: List of predicate names or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_predicate'
        request = GetNodeDetails.Request()
        request.expression = predicate

        response = self._create_and_call_service(GetNodeDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved the problem predicate {response.node}')
            return response.node
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem predicate: {error_msg}')
            return None

    def get_problem_predicates(self) -> Optional[List[PlanSys2Node]]:
        """
        Get the predicates in the problem.

        Returns
        -------
        Optional[List[PlanSys2Node]]: List of predicate names or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_predicates'
        request = GetStates.Request()

        response = self._create_and_call_service(GetStates, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.states)} problem predicates')
            return list(response.states)
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem predicates: {error_msg}')
            return None

    def get_problem_function(self, function: str) -> Optional[PlanSys2Node]:
        """
        Get the details of a function.

        Parameters
        ----------
        function : str
            The name of the function to retrieve.

        Returns
        -------
        Optional[PlanSys2Node]: List of function names or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_function'
        request = GetNodeDetails.Request()
        request.expression = function

        response = self._create_and_call_service(GetNodeDetails, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved the problem function {response.node}')
            return response.node
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem functions: {error_msg}')
            return None

    def get_problem_functions(self) -> Optional[List[PlanSys2Node]]:
        """
        Get the functions in the problem.

        Returns
        -------
        Optional[List[PlanSys2Node]]: List of function names or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem_functions'
        request = GetStates.Request()

        response = self._create_and_call_service(GetStates, service_name, request)
        if response and response.success:
            self.get_logger().debug(
                f'Successfully retrieved {len(response.states)} problem functions')
            return list(response.states)
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem functions: {error_msg}')
            return None

    def get_problem(self) -> Optional[str]:
        """
        Get the PDDL problem as a string.

        Returns
        -------
        Optional[str]: The PDDL problem string or None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/get_problem'
        request = GetProblem.Request()

        response = self._create_and_call_service(GetProblem, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully retrieved problem')
            return response.problem
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to get problem: {error_msg}')
            return None

    def remove_problem_goal(self) -> bool:
        """
        Remove the current goal from the problem.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/remove_problem_goal'
        request = RemoveProblemGoal.Request()

        response = self._create_and_call_service(RemoveProblemGoal, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully removed the current problem goal')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to remove the current problem goal: {error_msg}')
            return False

    def clear_problem_knowledge(self) -> bool:
        """
        Clear the instances, predicates, and functions.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/clear_problem_knowledge'
        request = ClearProblemKnowledge.Request()

        response = self._create_and_call_service(ClearProblemKnowledge, service_name, request)
        if response and response.success:
            self.get_logger().debug('Successfully cleared problem knowledge')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(
                f'Failed to clear problem knowledge: {error_msg}')
            return False

    def remove_problem_instance(self, instance: Param) -> bool:
        """
        Remove an instance from the problem.

        Parameters
        ----------
        instance : Param
            The instance to remove.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/remove_problem_instance'
        request = AffectParam.Request()
        request.param = instance

        response = self._create_and_call_service(AffectParam, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully removed problem instance: {instance}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to remove problem instance {instance}: {error_msg}')
            return False

    def remove_problem_predicate(self, predicate: PlanSys2Node) -> bool:
        """
        Remove a predicate from the problem.

        Parameters
        ----------
        predicate : PlanSys2Node
            The predicate to remove.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/remove_problem_predicate'
        request = AffectNode.Request()
        request.node = predicate

        response = self._create_and_call_service(AffectNode, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully removed problem predicate: {predicate}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to remove problem predicate {predicate}: {error_msg}')
            return False

    def remove_problem_function(self, function: PlanSys2Node) -> bool:
        """
        Remove a function from the problem.

        Parameters
        ----------
        function : PlanSys2Node
            The function to remove.

        Returns
        -------
        bool: True if successful, False otherwise.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/remove_problem_function'
        request = AffectNode.Request()
        request.node = function

        response = self._create_and_call_service(AffectNode, service_name, request)
        if response and response.success:
            self.get_logger().debug(f'Successfully removed problem function: {function}')
            return True
        else:
            error_msg = response.error_info if response else 'Service call failed'
            self.get_logger().error(f'Failed to remove problem function {function}: {error_msg}')
            return False

    def exist_problem_predicate(self, predicate: PlanSys2Node) -> Optional[bool]:
        """
        Check if a predicate exists.

        Parameters
        ----------
        predicate : PlanSys2Node
            The predicate to check.

        Returns
        -------
        Optional[bool]: True if satisfied, False if not, None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/exist_problem_predicate'
        request = ExistNode.Request()
        request.node = predicate

        response = self._create_and_call_service(ExistNode, service_name, request)
        if response is not None:
            self.get_logger().debug(f'Problem goal exists: {response.exist}')
            return response.exist
        else:
            self.get_logger().error('Failed to check if problem goal exists')
            return None

    def exist_problem_function(self, function: PlanSys2Node) -> Optional[bool]:
        """
        Check if a function exists.

        Parameters
        ----------
        function : PlanSys2Node
            The function to check.

        Returns
        -------
        Optional[bool]: True if satisfied, False if not, None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/exist_problem_function'
        request = ExistNode.Request()
        request.node = function

        response = self._create_and_call_service(ExistNode, service_name, request)
        if response is not None:
            self.get_logger().debug(f'Problem goal exists: {response.exist}')
            return response.exist
        else:
            self.get_logger().error('Failed to check if problem goal exists')
            return None

    def update_problem_function(self, function: PlanSys2Node) -> Optional[bool]:
        """
        Update a function value.

        Parameters
        ----------
        function : PlanSys2Node
            The function to update.

        Returns
        -------
        Optional[bool]: True if satisfied, False if not, None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/update_problem_function'
        request = AffectNode.Request()
        request.node = function

        response = self._create_and_call_service(AffectNode, service_name, request)
        if response is not None:
            self.get_logger().debug('Problem goal updated')
            return True
        else:
            self.get_logger().error('Failed to update problem goal')
            return None

    def is_problem_goal_satisfied(self, goal: Tree) -> Optional[bool]:
        """
        Check if the current problem goal is satisfied.

        Returns
        -------
        Optional[bool]: True if satisfied, False if not, None if failed.

        """
        service_name = f'{self._namespace_prefix}/problem_expert/is_problem_goal_satisfied'
        request = IsProblemGoalSatisfied.Request()
        request.tree = goal

        response = self._create_and_call_service(IsProblemGoalSatisfied, service_name, request)
        if response is not None:
            self.get_logger().debug(f'Problem goal satisfied: {response.satisfied}')
            return response.satisfied
        else:
            self.get_logger().error('Failed to check if problem goal is satisfied')
            return None

    def print_problem_info(self) -> None:
        """
        Print comprehensive information about the current problem.

        Calls all available public methods to provide an overview of the problem state.

        """
        self.get_logger().info('=== Problem Expert Information ===')

        # Get the full problem (PDDL)
        problem = self.get_problem()
        if problem:
            self.get_logger().info(f'Problem PDDL length: {len(problem)} characters')
            print(f'Problem preview:\n{problem[:30]}...')

        # Get problem goal
        goal = self.get_problem_goal()
        print(f'Problem Goal: {goal}')

        # Get instances
        instances = self.get_problem_instances()
        if instances:
            print(f'Instances ({len(instances)})')
            for instance in instances:
                single_instance = self.get_problem_instance(instance.name)
                print(f'  - Instance details: {single_instance}')

        # Get predicates
        predicates = self.get_problem_predicates()
        if predicates:
            print(f'Predicates ({len(predicates)}): {predicates}')
            for predicate in predicates:
                single_predicate = self.get_problem_predicate(predicate.name)
                print(f'  - Predicate details: {single_predicate}')

        # Get functions
        functions = self.get_problem_functions()
        if functions:
            print(f'Functions ({len(functions)}): {functions}')
            for function in functions:
                single_function = self.get_problem_function(function.name)
                print(f'  - Function details: {single_function}')
