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

"""Python plan solvers loaded as plugins by the tests of the Python planner."""

import threading
import time

from plansys2_msgs.msg import Plan, PlanItem
from plansys2_support_py.core.PlanSolverBase import PlanSolverBase

# Set by the tests when they end: a solver that ignores cancel stops then
RELEASE = threading.Event()
RUNNING = threading.Semaphore(0)  # Released by every get_plan that ends
STARTED = []


class FixedSolver(PlanSolverBase):
    """Plans (move r1 a b) after <name>.delay seconds; with ignore_cancel, it never stops."""

    def configure(self, lc_node, plugin_name):
        """Read the delay and whether to ignore cancel."""
        self._lc_node = lc_node
        self._stop = threading.Event()
        self.delay = lc_node.declare_parameter(f'{plugin_name}.delay', 0.0).value
        self.ignore_cancel = lc_node.declare_parameter(
            f'{plugin_name}.ignore_cancel', False).value

    def get_plan(self, domain, problem, node_namespace='', solver_timeout=None):
        """Wait the delay, unless cancelled, and return the plan."""
        self._stop.clear()
        STARTED.append(self)
        try:
            end = time.monotonic() + self.delay
            while time.monotonic() < end:
                if self._stop.wait(0.05) or RELEASE.is_set():
                    return None
        finally:
            RUNNING.release()
        plan = Plan()
        item = PlanItem()
        item.action = '(move r1 a b)'
        item.duration = 1.0
        plan.items.append(item)
        return plan

    def cancel(self):
        """Stop waiting, unless told to ignore it."""
        super().cancel()
        if not self.ignore_cancel:
            self._stop.set()

    def is_domain_valid(self, domain, node_namespace=''):
        """Accept domains that define one."""
        return '(define' in domain
