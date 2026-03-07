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

# Todo: Fix test @ajtudela
#
#     def test_get_plan_array_service_success(self):
#         """Test get_plan_array service with successful solvers."""
#         # Create two successful solvers
#         plan_item1 = PlanItem()
#         plan_item1.action = 'action1'
#         solver1 = MockPlanSolver(
#             self.planner_node,
#             'solver1',
#             success=True,
#             plan_items=[plan_item1]
#         )
#
#         plan_item2 = PlanItem()
#         plan_item2.action = 'action2'
#         solver2 = MockPlanSolver(
#             self.planner_node,
#             'solver2',
#             success=True,
#             plan_items=[plan_item2]
#         )
#
#         # Mock discovery and loading
#         self.planner_node.plugin_provider_.discover = MagicMock(return_value={
#             'test/solver1': MagicMock(plugin_id=MagicMock(return_value='test/solver1')),
#             'test/solver2': MagicMock(plugin_id=MagicMock(return_value='test/solver2'))
#         })
#
#         # Mock loading
#         def load_solver(plugin_id):
#             if plugin_id == 'test/solver1':
#                 return solver1
#             elif plugin_id == 'test/solver2':
#                 return solver2
#             return None
#
#         self.planner_node.plugin_provider_.load = MagicMock(side_effect=load_solver)
#
#         # Configure
#         self._declare_test_solver_params('solver1', 'test/solver1')
#         self._declare_test_solver_params('solver2', 'test/solver2')
#         self.planner_node.set_parameters([
#             Parameter('plan_solver_plugins', value=['solver1', 'solver2']),
#             Parameter('solver1.plugin', value='test/solver1'),
#             Parameter('solver2.plugin', value='test/solver2')
#         ])
#         self.planner_node.on_configure(State())
#
#         # Create service request
#         request = GetPlanArray.Request()
#         request.domain = '(define (domain test))'
#         request.problem = '(define (problem test))'
#         response = GetPlanArray.Response()
#
#         # Call service
#         self.planner_node.get_plan_array_service_callback(request, response)
#
#         # Verify response
#         self.assertTrue(response.success)
#         self.assertEqual(len(response.plan_array.plan_array), 2)

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
        """Test on_configure loading default POPF solver."""
        # Mock empty plugins parameter
        self.planner_node.set_parameters([
            rclpy.parameter.Parameter('plan_solver_plugins', rclpy.Parameter.Type.STRING_ARRAY, [])
        ])

        # Mock solver
        mock_solver = MagicMock(spec=PlanSolverBase)
        mock_load.return_value = mock_solver

        result = self.planner_node.on_configure(None)

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertEqual(len(self.planner_node.solvers_), 1)
        self.assertIn('POPF', self.planner_node.solvers_)
        mock_solver.configure.assert_called_once()

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

    @patch('plansys2_support_py.Planner.ThreadPoolExecutor')
    def test_get_plan_array_with_exception(self, mock_executor):
        """Test get_plan_array when a solver raises exception."""
        # Mock solver
        mock_solver = MagicMock(spec=PlanSolverBase)
        self.planner_node.solvers_ = {'solver1': mock_solver}

        # Mock executor
        mock_executor_instance = MagicMock()
        mock_executor.return_value.__enter__.return_value = mock_executor_instance

        # Mock future that raises exception
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.side_effect = RuntimeError('Solver error')
        mock_executor_instance.submit.return_value = mock_future

        result = self.planner_node.get_plan_array('domain', 'problem')

        self.assertIsNotNone(result)
        self.assertEqual(len(result.plan_array), 0)

    @patch('plansys2_support_py.Planner.ThreadPoolExecutor')
    @patch('time.sleep')
    def test_get_plan_array_timeout(self, mock_sleep, mock_executor):
        """Test get_plan_array when solvers timeout."""
        # Mock solver
        mock_solver = MagicMock(spec=PlanSolverBase)
        self.planner_node.solvers_ = {'solver1': mock_solver}
        self.planner_node.solver_timeout_ = Duration(seconds=0.1)

        # Mock executor
        mock_executor_instance = MagicMock()
        mock_executor.return_value.__enter__.return_value = mock_executor_instance

        # Mock future that never completes
        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_executor_instance.submit.return_value = mock_future

        # Mock clock to simulate timeout
        with patch.object(self.planner_node, 'get_clock') as mock_clock:
            mock_time_start = MagicMock()
            mock_time_end = MagicMock()
            mock_time_start.__sub__.return_value = Duration(seconds=0.2)
            mock_time_end.__sub__.return_value = Duration(seconds=0.3)
            mock_clock.return_value.now.side_effect = [
                mock_time_start, mock_time_end, mock_time_end
            ]

            result = self.planner_node.get_plan_array('domain', 'problem')

        self.assertIsNotNone(result)
        mock_solver.cancel.assert_called_once()

    @patch('plansys2_support_py.Planner.ThreadPoolExecutor')
    def test_get_plan_array_cancel_exception(self, mock_executor):
        """Test get_plan_array when collecting cancelled solver results raises exception."""
        # Mock solver
        mock_solver = MagicMock(spec=PlanSolverBase)
        self.planner_node.solvers_ = {'solver1': mock_solver}

        # Mock executor
        mock_executor_instance = MagicMock()
        mock_executor.return_value.__enter__.return_value = mock_executor_instance

        # Mock future that times out and raises exception on result
        mock_future = MagicMock()
        mock_future.done.return_value = False
        mock_future.result.side_effect = RuntimeError('Cancelled')
        mock_executor_instance.submit.return_value = mock_future

        # Mock clock to simulate immediate timeout
        with patch.object(self.planner_node, 'get_clock') as mock_clock:
            mock_time = MagicMock()
            mock_time.__sub__.return_value = Duration(seconds=100)
            mock_clock.return_value.now.return_value = mock_time

            result = self.planner_node.get_plan_array('domain', 'problem')

        self.assertIsNotNone(result)


if __name__ == '__main__':
    unittest.main()
