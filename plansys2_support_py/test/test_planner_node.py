# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for PlannerNode."""

import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from lifecycle_msgs.msg import State
from plansys2_msgs.msg import Plan, PlanItem
from plansys2_msgs.srv import GetPlan, GetPlanArray
from plansys2_support_py.core.PlanSolverBase import PlanSolverBase
from plansys2_support_py.core.PluginProvider import PluginProvider
from plansys2_support_py.Planner import PlannerNode

import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter


class MockPlanSolver(PlanSolverBase):
    """Mock implementation of PlanSolverBase for testing."""

    def __init__(self, node, plugin_name, success=True, plan_items=None):
        """Initialize mock solver."""
        super().__init__()
        self.success = success
        self.plan_items = plan_items if plan_items is not None else []
        self.configure(node, plugin_name)

    def configure(self, lc_node, plugin_name):
        """Configure the solver."""
        self._lc_node = lc_node
        self._plugin_name = plugin_name

    def get_plan(self, domain, problem, node_namespace='', solver_timeout=None):
        """Return mock plan."""
        plan = Plan()
        if self.success:
            plan.items = self.plan_items
        return plan

    def is_domain_valid(self, domain, node_namespace=''):
        """Check domain validity."""
        return True


class TestPlannerNode(unittest.TestCase):
    """Test suite for PlannerNode."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test."""
        self.planner_node = PlannerNode()

    def tearDown(self):
        """Clean up after each test."""
        self.planner_node.destroy_node()

    def _declare_test_solver_params(self, solver_id='test_solver', plugin_type='test/solver'):
        """Help to declare test solver parameters."""
        from rcl_interfaces.msg import ParameterDescriptor
        param_name = f'{solver_id}.plugin'
        if not self.planner_node.has_parameter(param_name):
            self.planner_node.declare_parameter(
                param_name,
                plugin_type,
                ParameterDescriptor(dynamic_typing=True, read_only=False)
            )

    def test_initialization(self):
        """Test PlannerNode initialization."""
        self.assertIsNotNone(self.planner_node)
        self.assertEqual(self.planner_node.get_name(), 'planner')

    def test_on_configure_success(self):
        """Test successful configuration."""
        # Recreate node for clean state
        self.planner_node.destroy_node()
        self.planner_node = PlannerNode()

        # Create mock solver instance
        mock_solver = MockPlanSolver(self.planner_node, 'test_solver', success=True)

        # Mock plugin provider at instance level
        self.planner_node.plugin_provider_.discover = MagicMock(return_value={
            'test_package/test_solver': MagicMock(
                plugin_id=MagicMock(return_value='test_package/test_solver'),
                attributes=MagicMock(return_value={
                    'module_name': 'test_module',
                    'class_from_class_type': 'TestSolver'
                })
            )
        })
        self.planner_node.plugin_provider_.load = MagicMock(return_value=mock_solver)

        # Declare new parameters
        self._declare_test_solver_params('test_solver', 'test_package/test_solver')

        # Set parameters
        self.planner_node.set_parameters([
            Parameter('plan_solver_plugins', value=['test_solver']),
            Parameter('plan_solver_timeout', value=5.0)
        ])

        # Configure
        result = self.planner_node.on_configure(State())

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIn('test_solver', self.planner_node.solvers_)

    def test_on_configure_no_plugins(self):
        """Test configuration with no plugins configured."""
        # Set parameter override before creating node
        self.planner_node.destroy_node()
        self.planner_node = PlannerNode()
        # Don't declare again, modify the existing parameter
        self.planner_node.set_parameters([Parameter('plan_solver_plugins', value=[])])

        result = self.planner_node.on_configure(State())

        self.assertEqual(result, TransitionCallbackReturn.FAILURE)

    def test_on_configure_plugin_load_failure(self):
        """Test configuration when plugin loading fails."""
        # Recreate node to avoid parameter already declared
        self.planner_node.destroy_node()
        self.planner_node = PlannerNode()

        # Mock plugin provider at instance level - return None for load
        self.planner_node.plugin_provider_.discover = MagicMock(return_value={
            'test_package/test_solver': MagicMock(
                plugin_id=MagicMock(return_value='test_package/test_solver')
            )
        })
        self.planner_node.plugin_provider_.load = MagicMock(return_value=None)

        self._declare_test_solver_params('test_solver', 'test_package/test_solver')
        self.planner_node.set_parameters([
            Parameter('plan_solver_plugins', value=['test_solver']),
            Parameter('test_solver.plugin', value='test_package/test_solver')
        ])

        result = self.planner_node.on_configure(State())

        self.assertEqual(result, TransitionCallbackReturn.FAILURE)

    def test_on_activate(self):
        """Test activation."""
        result = self.planner_node.on_activate(State())
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_deactivate(self):
        """Test deactivation."""
        result = self.planner_node.on_deactivate(State())
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_cleanup(self):
        """Test cleanup."""
        # Configure first - mock the plugin provider
        mock_solver = MockPlanSolver(self.planner_node, 'test_solver', success=True)
        self.planner_node.plugin_provider_.discover = MagicMock(return_value={})
        self.planner_node.plugin_provider_.load = MagicMock(return_value=mock_solver)

        # Add a solver manually for testing cleanup
        self.planner_node.solvers_['test_solver'] = mock_solver

        result = self.planner_node.on_cleanup(State())

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertEqual(len(self.planner_node.solvers_), 0)

    def test_on_shutdown(self):
        """Test shutdown."""
        result = self.planner_node.on_shutdown(State())
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_error(self):
        """Test error handling."""
        result = self.planner_node.on_error(State())
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_get_plan_service(self):
        """Test get_plan service callback."""
        # Create mock solver with a plan
        plan_item = PlanItem()
        plan_item.action = 'test_action'
        mock_solver = MockPlanSolver(
            self.planner_node,
            'test_solver',
            success=True,
            plan_items=[plan_item]
        )

        # Mock plugin provider at instance level
        self.planner_node.plugin_provider_.discover = MagicMock(return_value={
            'test/solver': MagicMock(plugin_id=MagicMock(return_value='test/solver'))
        })
        self.planner_node.plugin_provider_.load = MagicMock(return_value=mock_solver)

        self._declare_test_solver_params('test_solver', 'test/solver')
        self.planner_node.set_parameters([
            Parameter('plan_solver_plugins', value=['test_solver']),
            Parameter('test_solver.plugin', value='test/solver')
        ])
        self.planner_node.on_configure(State())

        # Create service request
        request = GetPlan.Request()
        request.domain = '(define (domain test))'
        request.problem = '(define (problem test))'
        response = GetPlan.Response()

        # Call service
        self.planner_node.get_plan_service_callback(request, response)

        # Verify response
        self.assertTrue(response.success)
        self.assertEqual(len(response.plan.items), 1)
        self.assertEqual(response.plan.items[0].action, 'test_action')

    def test_get_plan_array_service_success(self):
        """The plans of every solver, shortest first."""
        self.planner_node.solvers_ = {
            'one': MockPlanSolver(self.planner_node, 'one', plan_items=[PlanItem(), PlanItem()]),
            'two': MockPlanSolver(self.planner_node, 'two', plan_items=[PlanItem()]),
        }
        request = GetPlanArray.Request()
        response = GetPlanArray.Response()

        self.planner_node.get_plan_array_service_callback(request, response)

        self.assertTrue(response.success)
        self.assertEqual([len(p.items) for p in response.plan_array.plan_array], [1, 2])

    def test_get_plan_array_service_timeout(self):
        """Test get_plan_array service with timeout."""
        # Recreate node for clean parameters
        self.planner_node.destroy_node()
        self.planner_node = PlannerNode()

        # Create solver that will be slow but not cause actual timeout
        # The solver will be cancelled before completing
        slow_solver = MagicMock(spec=PlanSolverBase)

        # Mock a plan that takes longer than configured timeout
        # But return None to simulate cancellation
        slow_solver.get_plan.return_value = None
        slow_solver.configure = MagicMock()
        slow_solver.cancel = MagicMock()

        # Mock plugin provider at instance level
        self.planner_node.plugin_provider_.discover = MagicMock(return_value={
            'test/slow_solver': MagicMock(
                plugin_id=MagicMock(return_value='test/slow_solver')
            )
        })
        self.planner_node.plugin_provider_.load = MagicMock(return_value=slow_solver)

        # Configure with very short timeout
        self._declare_test_solver_params('slow_solver', 'test/slow_solver')
        self.planner_node.set_parameters([
            Parameter('plan_solver_plugins', value=['slow_solver']),
            Parameter('slow_solver.plugin', value='test/slow_solver'),
            Parameter('plan_solver_timeout', value=0.01)  # Very short timeout
        ])
        self.planner_node.on_configure(State())

        # Create service request
        request = GetPlanArray.Request()
        request.domain = '(define (domain test))'
        request.problem = '(define (problem test))'
        response = GetPlanArray.Response()

        # Call service - solver won't complete in time
        self.planner_node.get_plan_array_service_callback(request, response)

        # Response should be empty since solver returned None (timed out/cancelled)
        self.assertEqual(len(response.plan_array.plan_array), 0)
        self.assertFalse(response.success)

    def test_declare_parameter_if_not_declared(self):
        """Test conditional parameter declaration."""
        # First call should declare
        self.planner_node._declare_parameter_if_not_declared('test_param', 'value1')
        value1 = self.planner_node.get_parameter('test_param').value
        self.assertEqual(value1, 'value1')

        # Second call should not override
        self.planner_node._declare_parameter_if_not_declared('test_param', 'value2')
        value2 = self.planner_node.get_parameter('test_param').value
        self.assertEqual(value2, 'value1')  # Should still be original value

    def test_get_plugin_type_param(self):
        """Test getting plugin type parameter."""
        self.planner_node.declare_parameter('test_solver.plugin', 'test_package/test_solver')

        plugin_type = self.planner_node._get_plugin_type_param('test_solver')

        self.assertEqual(plugin_type, 'test_package/test_solver')

    def test_get_plugin_type_param_not_found(self):
        """Test getting non-existent plugin type parameter."""
        # Should raise RuntimeError instead of returning empty string
        with self.assertRaises(RuntimeError):
            self.planner_node._get_plugin_type_param('nonexistent_solver')

    @patch.object(PluginProvider, 'load')
    def test_on_configure_no_plugins_load_default_success(self, mock_load):
        """Without plan_solver_plugins it fails: the C++ solvers are not for this planner."""
        self.planner_node.set_parameters([
            rclpy.parameter.Parameter('plan_solver_plugins', rclpy.Parameter.Type.STRING_ARRAY, [])
        ])
        mock_load.return_value = MagicMock(spec=PlanSolverBase)

        result = self.planner_node.on_configure(None)

        self.assertEqual(result, TransitionCallbackReturn.FAILURE)
        self.assertEqual(len(self.planner_node.solvers_), 0)
        mock_load.assert_not_called()

    @patch.object(PluginProvider, 'load')
    def test_on_configure_no_plugins_load_default_failure(self, mock_load):
        """Test on_configure when default POPF solver fails to load."""
        # Mock empty plugins parameter
        self.planner_node.set_parameters([
            rclpy.parameter.Parameter('plan_solver_plugins', rclpy.Parameter.Type.STRING_ARRAY, [])
        ])

        # Mock solver loading failure
        mock_load.return_value = None

        result = self.planner_node.on_configure(None)

        self.assertEqual(result, TransitionCallbackReturn.FAILURE)

    @patch.object(PluginProvider, 'load')
    def test_on_configure_no_plugins_exception(self, mock_load):
        """Test on_configure when default solver raises exception."""
        # Mock empty plugins parameter
        self.planner_node.set_parameters([
            rclpy.parameter.Parameter('plan_solver_plugins', rclpy.Parameter.Type.STRING_ARRAY, [])
        ])

        # Mock solver loading exception
        mock_load.side_effect = RuntimeError('Failed to load')

        result = self.planner_node.on_configure(None)

        self.assertEqual(result, TransitionCallbackReturn.FAILURE)

    def test_on_cleanup_with_services(self):
        """Test cleanup with services created."""
        # Create mock services
        self.planner_node.get_plan_service_ = MagicMock()
        self.planner_node.get_plan_array_service_ = MagicMock()
        self.planner_node.validate_domain_service_ = MagicMock()
        self.planner_node.solvers_ = {'test': MagicMock()}

        with patch.object(self.planner_node, 'destroy_service') as mock_destroy:
            result = self.planner_node.on_cleanup(None)

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertEqual(mock_destroy.call_count, 3)
        self.assertEqual(len(self.planner_node.solvers_), 0)

    def _plan(self, *actions):
        plan = Plan()
        for action in actions:
            item = PlanItem()
            item.action = action
            plan.items.append(item)
        return plan

    def test_get_plan_array_with_exception(self):
        """A solver that raises gives no plan, and the others still answer."""
        failing = MagicMock(spec=PlanSolverBase)
        failing.get_plan.side_effect = ValueError('Solver error')
        working = MagicMock(spec=PlanSolverBase)
        working.get_plan.return_value = self._plan('(move r1 a b)')
        self.planner_node.solvers_ = {'failing': failing, 'working': working}

        result = self.planner_node.get_plan_array('domain', 'problem')

        self.assertEqual(len(result.plan_array), 1)
        self.assertEqual(result.plan_array[0].items[0].action, '(move r1 a b)')

    def test_get_plan_array_timeout(self):
        """A solver past the timeout is cancelled, and the plans found are returned."""
        release = threading.Event()
        slow = MagicMock(spec=PlanSolverBase)
        slow.get_plan.side_effect = lambda *args: release.wait(5.0) and None
        slow.cancel.side_effect = release.set
        fast = MagicMock(spec=PlanSolverBase)
        fast.get_plan.return_value = self._plan('(a)')
        self.planner_node.solvers_ = {'slow': slow, 'fast': fast}
        self.planner_node.solver_timeout_ = Duration(seconds=0.3)

        start = time.monotonic()
        result = self.planner_node.get_plan_array('domain', 'problem')

        self.assertLess(time.monotonic() - start, 1.0)
        slow.cancel.assert_called_once()
        fast.cancel.assert_not_called()
        self.assertEqual(len(result.plan_array), 1)

    def test_get_plan_array_does_not_wait_for_a_hung_solver(self):
        """A solver that ignores cancel is left behind, not waited for."""
        release = threading.Event()
        hung = MagicMock(spec=PlanSolverBase)
        hung.get_plan.side_effect = lambda *args: release.wait(10.0) and None
        self.planner_node.solvers_ = {'hung': hung}
        self.planner_node.solver_timeout_ = Duration(seconds=0.2)

        start = time.monotonic()
        result = self.planner_node.get_plan_array('domain', 'problem')

        self.assertLess(time.monotonic() - start, 1.0)
        hung.cancel.assert_called_once()
        self.assertEqual(len(result.plan_array), 0)
        release.set()

    def test_get_plan_array_without_solvers(self):
        """Without solvers there is no plan, and no error from the thread pool."""
        self.planner_node.solvers_ = {}
        self.assertEqual(len(self.planner_node.get_plan_array('d', 'p').plan_array), 0)

    def test_get_plan_array_order(self):
        """Shorter plans first; among equal ones, the order of the solvers."""
        solvers = {}
        for name, plan in [('long', self._plan('a', 'b', 'c')), ('first', self._plan('x')),
                           ('second', self._plan('y')), ('none', None)]:
            solvers[name] = MagicMock(spec=PlanSolverBase)
            solvers[name].get_plan.return_value = plan
        self.planner_node.solvers_ = solvers

        for _ in range(20):
            result = self.planner_node.get_plan_array('domain', 'problem')
            self.assertEqual(
                [p.items[0].action for p in result.plan_array], ['x', 'y', 'a'])

    def test_get_plan_array_uses_wall_time(self):
        """The timeout does not depend on the node clock, which may be simulated."""
        solver = MagicMock(spec=PlanSolverBase)
        solver.get_plan.return_value = self._plan('(a)')
        self.planner_node.solvers_ = {'solver': solver}
        with patch.object(self.planner_node, 'get_clock') as mock_clock:
            mock_clock.side_effect = AssertionError('node clock used')
            result = self.planner_node.get_plan_array('domain', 'problem')
        self.assertEqual(len(result.plan_array), 1)

    def test_on_configure_bad_timeout(self):
        """A timeout that is not positive becomes 15 seconds."""
        self.planner_node.plugin_provider_.discover = MagicMock(return_value=[])
        self.planner_node.plugin_provider_.load = MagicMock(
            return_value=MagicMock(spec=PlanSolverBase))
        self._declare_test_solver_params('test_solver', 'test/solver')
        self.planner_node.set_parameters([
            Parameter('plan_solver_plugins', value=['test_solver']),
            Parameter('plan_solver_timeout', value=-1.0),
        ])
        self.assertEqual(
            self.planner_node.on_configure(State()), TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.planner_node.solver_timeout_, Duration(seconds=15))


if __name__ == '__main__':
    unittest.main()
