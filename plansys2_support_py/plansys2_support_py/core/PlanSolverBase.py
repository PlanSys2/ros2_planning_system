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

"""Abstract base class for plan solvers in PlanSys2."""

from abc import ABC, abstractmethod

import shlex
import signal
import subprocess
import threading

from typing import Optional

from plansys2_msgs.msg import Plan
from rclpy.duration import Duration
from rclpy.node import Node


class PlanSolverBase(ABC):
    """
    Abstract base class for plan solvers in PlanSys2.

    This class defines the interface for plan solver plugins, including methods for
    configuration, plan generation, domain validation, and planner execution.
    """

    def __init__(self) -> None:
        """Initialize the PlanSolverBase."""
        self._lc_node: Optional[Node] = None
        self._cancel_requested: bool = False

    @abstractmethod
    def configure(self, lc_node: Node, plugin_name: str) -> None:
        """
        Configure the plan solver with a lifecycle node and plugin name.

        Parameters
        ----------
        lc_node : Node
            The ROS2 lifecycle node.
        plugin_name : str
            The name of the plugin.

        """
        pass

    @abstractmethod
    def get_plan(
        self,
        domain: str,
        problem: str,
        node_namespace: str = '',
        solver_timeout: Duration = Duration(seconds=15)
    ) -> Optional[Plan]:
        """
        Generate a plan given a PDDL domain and problem definition.

        Parameters
        ----------
        domain : str
            The PDDL domain as a string.
        problem : str
            The PDDL problem definition as a string.
        node_namespace : str, optional
            The node namespace (default is '').
        solver_timeout : Duration, optional
            Timeout for the solver (default is 15s).

        Returns
        -------
        Optional[Plan]
            The resulting plan if found, otherwise None.

        """
        pass

    @abstractmethod
    def is_domain_valid(self, domain: str, node_namespace: str = '') -> bool:
        """
        Validate a PDDL domain.

        Parameters
        ----------
        domain : str
            The PDDL domain as a string.
        node_namespace : str, optional
            The node namespace (default is '').

        Returns
        -------
        bool
            True if the domain is valid, False otherwise.

        """
        pass

    def cancel(self) -> None:
        """Request cancellation of the current planning process."""
        self._cancel_requested = True

    def _tokenize(self, command: str) -> list:
        """
        Tokenize a command string using shell-like syntax.

        Parameters
        ----------
        command : str
            The command string to tokenize.

        Returns
        -------
        list
            List of tokens from the command.

        """
        return shlex.split(command)

    def execute_planner(
        self,
        command: str,
        solver_timeout: Duration,
        plan_path: str
    ) -> bool:
        """
        Execute the planner with a command.

        Parameters
        ----------
        command : str
            The command to execute the planner.
        solver_timeout : Duration
            Timeout for the solver.
        plan_path : str
            Path to store the resulting plan.

        Returns
        -------
        bool
            True if the planner executed successfully, False otherwise.

        """
        if self._lc_node is None:
            raise RuntimeError('Node is not configured. Call configure() first.')

        self._cancel_requested = False
        child_finish = False
        start_time = self._lc_node.get_clock().now()

        try:
            # Open output file for writing planner results
            with open(plan_path, 'w') as output_file:
                # Start the planner process
                process = subprocess.Popen(
                    self._tokenize(command),
                    stdout=output_file,
                    stderr=subprocess.PIPE,
                    text=True
                )

                # Monitor thread to handle timeout and cancellation
                def monitor():
                    nonlocal child_finish
                    timeout_sec = solver_timeout.nanoseconds / 1e9
                    poll_interval = 0.1  # 100ms

                    while not self._cancel_requested and not child_finish:
                        elapsed = (
                            self._lc_node.get_clock().now() - start_time
                        ).nanoseconds / 1e9

                        if elapsed >= timeout_sec:
                            self._lc_node.get_logger().debug(
                                'Planner timeout reached, terminating process'
                            )
                            try:
                                process.kill()
                            except ProcessLookupError:
                                pass
                            break

                        threading.Event().wait(poll_interval)

                    if self._cancel_requested:
                        self._lc_node.get_logger().debug(
                            'Cancel requested, terminating planner process'
                        )
                        try:
                            process.kill()
                        except ProcessLookupError:
                            pass

                monitor_thread = threading.Thread(target=monitor)
                monitor_thread.start()

                # Wait for process to complete
                return_code = process.wait()
                child_finish = True
                monitor_thread.join()

                # Check if process was cancelled or timed out
                if self._cancel_requested:
                    self._lc_node.get_logger().debug(
                        'Child process was terminated by cancel request'
                    )
                    return False

                if return_code == -signal.SIGKILL:
                    self._lc_node.get_logger().debug(
                        'Child process was killed (timeout or cancel)'
                    )
                    return False

                # Check exit status
                if return_code != 0:
                    self._lc_node.get_logger().debug(
                        f'Child process exited with status: {return_code}'
                    )
                    return False

                self._lc_node.get_logger().debug(
                    f'Child process exited successfully with status: {return_code}'
                )

        except FileNotFoundError as e:
            self._lc_node.get_logger().error(f'Command not found: {e}')
            return False
        except OSError as e:
            self._lc_node.get_logger().error(f'Failed to open output file: {e}')
            return False
        except Exception as e:
            self._lc_node.get_logger().error(f'Unexpected error: {e}')
            return False

        return True
