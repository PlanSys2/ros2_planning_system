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

"""Main planner node that loads and uses plan solver plugins."""

from concurrent.futures import Future, ThreadPoolExecutor

import time

from typing import Dict, List, Optional

from plansys2_msgs.msg import Plan, PlanArray
from plansys2_msgs.srv import GetPlan, GetPlanArray, ValidateDomain
from plansys2_support_py.core.PlanSolverBase import PlanSolverBase
from plansys2_support_py.core.PluginProvider import PluginProvider

import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn


class PlannerNode(LifecycleNode):
    """
    ROS2 Lifecycle Node that manages planning system.

    This node handles planning requests. Loads planner plugins based on
    configuration parameters, provides services to generate plans from
    PDDL domains and problems, and validates domains.
    """

    def __init__(self):
        """Initialize the PlannerNode."""
        super().__init__('planner')

        self.solvers_: Dict[str, PlanSolverBase] = {}
        self.solver_ids_: List[str] = []
        self.solver_types_: List[str] = []
        self.default_ids_: List[str] = ['']
        self.default_types_: List[str] = ['plansys2_popf_plan_solver/popf']
        self.solver_timeout_: Duration = Duration(seconds=15)

        # Declare parameters
        self.declare_parameter('plan_solver_plugins', self.default_ids_)
        self.declare_parameter(
            'plan_solver_timeout', self.solver_timeout_.nanoseconds / 1e9
        )

        # Services (will be created in on_configure)
        self.get_plan_service_ = None
        self.get_plan_array_service_ = None
        self.validate_domain_service_ = None

        # Plugin provider
        self.plugin_provider_ = PluginProvider(
            export_tag='plansys2_planner',
            base_class_type='plansys2_support_py.core.PlanSolverBase'
        )

    def _declare_parameter_if_not_declared(self, param_name: str, default_value) -> None:
        """
        Declare a parameter if it has not been declared yet.

        Parameters
        ----------
        param_name : str
            Name of the parameter to declare.
        default_value: Any
            Default value for the parameter.

        """
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, default_value)

    def _get_plugin_type_param(self, plugin_name: str) -> str:
        """
        Get the plugin type parameter for a given plugin name.

        Parameters
        ----------
        plugin_name : str
            Name of the plugin.

        Returns
        -------
        str: The plugin type.

        """
        param_name = f'{plugin_name}.plugin'
        self._declare_parameter_if_not_declared(param_name, '')

        plugin_type = self.get_parameter(param_name).value

        if not plugin_type:
            self.get_logger().fatal(
                f"'plugin' param not defined for {plugin_name}"
            )
            raise RuntimeError(
                f"'plugin' param not defined for {plugin_name}"
            )

        return plugin_type

    def on_configure(
        self, state: LifecycleState
    ) -> TransitionCallbackReturn:
        """
        Configure the node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn: SUCCESS if configuration successful,
            FAILURE otherwise.

        """
        self.get_logger().info(f'[{self.get_name()}] Configuring...')

        # Discover available plugins
        self.get_logger().info(f'[{self.get_name()}] Discovering plugins...')
        plugin_descriptors = self.plugin_provider_.discover(self)

        if not plugin_descriptors:
            self.get_logger().warning(f'[{self.get_name()}] No plugins found!')

        # Get parameters
        self.solver_ids_ = self.get_parameter('plan_solver_plugins').value
        timeout = self.get_parameter('plan_solver_timeout').value
        self.solver_timeout_ = Duration(seconds=timeout)

        if self.solver_ids_:
            # Configure solver parameters if using defaults
            if self.solver_ids_ == self.default_ids_:
                for i in range(len(self.default_ids_)):
                    self._declare_parameter_if_not_declared(
                        f'{self.default_ids_[i]}.plugin',
                        self.default_types_[i]
                    )

            # Load all configured solvers
            self.solver_types_ = [''] * len(self.solver_ids_)

            for i in range(len(self.solver_ids_)):
                try:
                    solver_id = self.solver_ids_[i]
                    plugin_type = self._get_plugin_type_param(solver_id)
                    self.solver_types_[i] = plugin_type

                    # Load plugin
                    solver = self.plugin_provider_.load(self.solver_types_[i], self)

                    if solver is None:
                        self.get_logger().fatal(
                            f'[{self.get_name()}] Failed to create '
                            f'solver: {solver_id} of type '
                            f'{self.solver_types_[i]}'
                        )
                        return TransitionCallbackReturn.FAILURE

                    # Configure the solver
                    solver.configure(self, solver_id)

                    self.get_logger().info(
                        f'[{self.get_name()}] Created solver: '
                        f'{solver_id} of type {self.solver_types_[i]}'
                    )

                    self.solvers_[solver_id] = solver

                except RuntimeError as ex:
                    self.get_logger().fatal(
                        f'[{self.get_name()}] Failed to create solver. '
                        f'Exception: {ex}'
                    )
                    return TransitionCallbackReturn.FAILURE
        else:
            # Load default POPF planner
            self.get_logger().info(
                f'[{self.get_name()}] No plan_solver_plugins '
                f'specified, loading default POPF solver'
            )
            try:
                default_solver = self.plugin_provider_.load('plansys2::POPFPlanSolver', self)
                if default_solver is None:
                    self.get_logger().fatal(
                        'Failed to load default POPF solver'
                    )
                    return TransitionCallbackReturn.FAILURE

                default_solver.configure(self, 'POPF')
                self.solvers_['POPF'] = default_solver
                self.get_logger().info(
                    f'[{self.get_name()}] Created default solver: '
                    'POPF of type plansys2::POPFPlanSolver'
                )
            except RuntimeError as ex:
                self.get_logger().fatal(
                    f'[{self.get_name()}] Failed to create default '
                    f'solver. Exception: {ex}'
                )
                return TransitionCallbackReturn.FAILURE

        timeout_seconds = self.solver_timeout_.nanoseconds / 1e9
        self.get_logger().info(
            f'[{self.get_name()}] Solver Timeout: {timeout_seconds}s'
        )

        # Create services
        self.get_plan_service_ = self.create_service(
            GetPlan,
            'planner/get_plan',
            self.get_plan_service_callback
        )

        self.get_plan_array_service_ = self.create_service(
            GetPlanArray,
            'planner/get_plan_array',
            self.get_plan_array_service_callback
        )

        self.validate_domain_service_ = self.create_service(
            ValidateDomain,
            'planner/validate_domain',
            self.validate_domain_service_callback
        )

        self.get_logger().info(f'[{self.get_name()}] Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Activate the node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn: SUCCESS if activation is successful.

        """
        self.get_logger().info(f'[{self.get_name()}] Activating...')
        self.get_logger().info(f'[{self.get_name()}] Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Deactivate the node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn: SUCCESS if deactivation is successful.

        """
        self.get_logger().info(f'[{self.get_name()}] Deactivating...')
        self.get_logger().info(f'[{self.get_name()}] Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Clean up the node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn: SUCCESS if cleanup is successful.

        """
        self.get_logger().info(f'[{self.get_name()}] Cleaning up...')

        # Destroy services
        if self.get_plan_service_:
            self.destroy_service(self.get_plan_service_)
            self.get_plan_service_ = None

        if self.get_plan_array_service_:
            self.destroy_service(self.get_plan_array_service_)
            self.get_plan_array_service_ = None

        if self.validate_domain_service_:
            self.destroy_service(self.validate_domain_service_)
            self.validate_domain_service_ = None

        # Clear solvers
        self.solvers_.clear()

        self.get_logger().info(f'[{self.get_name()}] Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Shut down the node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn: SUCCESS if shutdown is successful.

        """
        self.get_logger().info(f'[{self.get_name()}] Shutting down...')
        self.get_logger().info(f'[{self.get_name()}] Shut down')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handle errors in the node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn: SUCCESS if error handling is successful.

        """
        self.get_logger().error(f'[{self.get_name()}] Error transition')
        return TransitionCallbackReturn.SUCCESS

    def get_plan_array(self, domain: str, problem: str) -> PlanArray:
        """
        Generate plans for PDDL problem using all configured planners.

        Parameters
        ----------
        domain : str
            PDDL domain string.
        problem : str
            PDDL problem string.

        Returns
        -------
        PlanArray: An array of plans found by the planners.

        """
        futures: Dict[str, Future] = {}
        results: Dict[str, Optional[Plan]] = {}

        start_time = self.get_clock().now()

        # Launch all solvers in parallel using ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=len(self.solvers_)) as executor:
            for solver_id, solver in self.solvers_.items():
                future = executor.submit(
                    solver.get_plan,
                    domain,
                    problem,
                    self.get_namespace(),
                    self.solver_timeout_
                )
                futures[solver_id] = future

            # Wait for results with timeout
            pending_count = len(futures)
            elapsed = self.get_clock().now() - start_time
            while pending_count > 0 and elapsed < self.solver_timeout_:
                for solver_id, future in futures.items():
                    if solver_id not in results:
                        if future.done():
                            try:
                                result = future.result(timeout=0.001)
                                results[solver_id] = result
                                pending_count -= 1
                            except RuntimeError as e:
                                self.get_logger().warning(
                                    f'[{self.get_name()}] Solver '
                                    f'{solver_id} raised exception: {e}'
                                )
                                results[solver_id] = None
                                pending_count -= 1

                time.sleep(0.001)  # Small sleep to avoid busy waiting
                elapsed = self.get_clock().now() - start_time

            # Cancel remaining solvers
            for solver_id, solver in self.solvers_.items():
                if solver_id not in results:
                    solver.cancel()

            # Give cancelled solvers time to terminate
            time.sleep(0.1)

            # Collect remaining results
            for solver_id, future in futures.items():
                if solver_id not in results:
                    try:
                        future.result(timeout=0.1)
                    except RuntimeError as e:
                        self.get_logger().warning(
                            f'[{self.get_name()}] Exception while '
                            f'destroying future for {solver_id}: {e}'
                        )

        # Build plan array from results
        plan_array = PlanArray()
        for solver_id, plan in results.items():
            if plan is not None:
                # type: ignore[union-attr]
                plan_array.plan_array.append(plan)

        # Sort plans by number of items (shorter plans first)
        # type: ignore[union-attr]
        plan_array.plan_array.sort(key=lambda p: len(p.items))

        return plan_array

    def get_plan_service_callback(self, request, response):
        """
        Service callback to generate a plan for a PDDL problem.

        Parameters
        ----------
        request: GetPlan.Request
            Service request with domain and problem PDDL strings.
        response: GetPlan.Response
            Service response containing the generated plan.

        Returns
        -------
        GetPlan.Response: Response with plan and success status.

        """
        plans = self.get_plan_array(request.domain, request.problem)

        if plans.plan_array:
            response.success = True
            response.plan = plans.plan_array[0]
        else:
            response.success = False
            response.error_info = 'Plan not found'

        return response

    def get_plan_array_service_callback(self, request, response):
        """
        Service callback to generate multiple plans.

        Parameters
        ----------
        request: GetPlanArray.Request
            Service request with domain and problem strings.
        response: GetPlanArray.Response
            Service response containing array of plans.

        Returns
        -------
        GetPlanArray.Response: Response with plans array.

        """
        plans_result = self.get_plan_array(
            request.domain, request.problem
        )
        response.plan_array = plans_result

        if response.plan_array.plan_array:
            response.success = True
        else:
            response.success = False
            response.error_info = 'Plan not found'

        return response

    def validate_domain_service_callback(self, request, response):
        """
        Service callback to validate a PDDL domain.

        Parameters
        ----------
        request: ValidateDomain.Request
            Service request containing the domain and problem PDDL strings.
        response: ValidateDomain.Response
            Service response with the generated plans.

        Returns
        -------
        ValidateDomain.Response: The response with validation status.

        """
        if not self.solvers_:
            response.success = False
            response.error_info = 'No solvers available'
            return response

        # Use the first solver to validate the domain
        first_solver = next(iter(self.solvers_.values()))
        is_valid = first_solver.is_domain_valid(
            request.domain, self.get_namespace()
        )
        response.success = is_valid

        if not response.success:
            response.error_info = 'Domain is not valid'

        return response

    def set_timeout(self, solver_timeout: Duration) -> None:
        """
        Set the timeout for plan solvers.

        Parameters
        ----------
        solver_timeout : Duration
            The new timeout duration.

        """
        self.solver_timeout_ = solver_timeout


def main(args=None):
    """
    Run the main entry point for the planner node.

    Parameters
    ----------
    args: str
        Command line arguments (optional).

    """
    rclpy.init(args=args)

    try:
        planner_node = PlannerNode()
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
