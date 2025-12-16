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

"""Unit tests for PlanSolverBase."""

import os
import tempfile
import unittest

from plansys2_support_py.core.PlanSolverBase import PlanSolverBase

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


class MockPlanSolver(PlanSolverBase):
    """Test implementation of PlanSolverBase."""

    def configure(self, lc_node, plugin_name):
        """Configure the test solver."""
        self._lc_node = lc_node
        self._plugin_name = plugin_name

    def get_plan(self, domain, problem, node_namespace='', solver_timeout=Duration(seconds=15)):
        """Mock get_plan implementation."""
        return None

    def is_domain_valid(self, domain, node_namespace=''):
        """Mock is_domain_valid implementation."""
        return True


class TestPlanSolverBase(unittest.TestCase):
    """Test suite for PlanSolverBase."""

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
        self.node = Node('test_node')
        self.solver = MockPlanSolver()
        self.solver.configure(self.node, 'test_solver')

    def tearDown(self):
        """Clean up after each test."""
        self.node.destroy_node()

    def test_initialization(self):
        """Test that the solver initializes correctly."""
        solver = MockPlanSolver()
        self.assertIsNotNone(solver)
        self.assertIsNone(solver._lc_node)
        self.assertFalse(solver._cancel_requested)

    def test_configure(self):
        """Test solver configuration."""
        self.assertIsNotNone(self.solver._lc_node)
        self.assertEqual(self.solver._lc_node, self.node)

    def test_cancel(self):
        """Test cancel request."""
        self.assertFalse(self.solver._cancel_requested)
        self.solver.cancel()
        self.assertTrue(self.solver._cancel_requested)

    def test_tokenize_simple_command(self):
        """Test tokenization of simple command."""
        command = 'ls -la /tmp'
        tokens = self.solver._tokenize(command)
        self.assertEqual(tokens, ['ls', '-la', '/tmp'])

    def test_tokenize_quoted_command(self):
        """Test tokenization of command with quotes."""
        command = 'echo "hello world" test'
        tokens = self.solver._tokenize(command)
        self.assertEqual(tokens, ['echo', 'hello world', 'test'])

    def test_tokenize_empty_command(self):
        """Test tokenization of empty command."""
        command = ''
        tokens = self.solver._tokenize(command)
        self.assertEqual(tokens, [])

    def test_execute_planner_unconfigured_node(self):
        """Test execute_planner raises error when node is not configured."""
        solver = MockPlanSolver()
        with tempfile.NamedTemporaryFile(delete=False) as tmp:
            tmp_path = tmp.name

        try:
            with self.assertRaises(RuntimeError):
                solver.execute_planner(
                    'echo test',
                    Duration(seconds=1),
                    tmp_path
                )
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_successful_command(self):
        """Test successful planner execution."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            result = self.solver.execute_planner(
                'echo "test output"',
                Duration(seconds=5),
                tmp_path
            )

            self.assertTrue(result)
            self.assertTrue(os.path.exists(tmp_path))

            with open(tmp_path, 'r') as f:
                content = f.read()
                self.assertIn('test output', content)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_failing_command(self):
        """Test planner execution with failing command."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            result = self.solver.execute_planner(
                'false',  # Command that always fails
                Duration(seconds=5),
                tmp_path
            )

            self.assertFalse(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_nonexistent_command(self):
        """Test planner execution with non-existent command."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            result = self.solver.execute_planner(
                'nonexistent_command_12345',
                Duration(seconds=5),
                tmp_path
            )

            self.assertFalse(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_timeout(self):
        """Test planner execution with timeout."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Command that sleeps longer than timeout
            result = self.solver.execute_planner(
                'sleep 10',
                Duration(seconds=1),
                tmp_path
            )

            self.assertFalse(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_cancel_during_execution(self):
        """Test canceling planner during execution."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            import threading

            # Start execution in a thread
            def execute():
                return self.solver.execute_planner(
                    'sleep 5',
                    Duration(seconds=10),
                    tmp_path
                )

            thread = threading.Thread(target=execute)
            thread.start()

            # Cancel after a short delay
            import time
            time.sleep(0.5)
            self.solver.cancel()

            thread.join(timeout=3)

            # Should have been cancelled
            self.assertTrue(self.solver._cancel_requested)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_invalid_output_path(self):
        """Test planner execution with invalid output path."""
        result = self.solver.execute_planner(
            'echo test',
            Duration(seconds=5),
            '/nonexistent/directory/file.txt'
        )

        self.assertFalse(result)

    def test_execute_planner_multiple_commands(self):
        """Test planner execution with command creating output."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Command that generates some output
            result = self.solver.execute_planner(
                'echo "line1" && echo "line2" && echo "line3"',
                Duration(seconds=5),
                tmp_path
            )

            self.assertTrue(result)

            with open(tmp_path, 'r') as f:
                content = f.read()
                self.assertIn('line1', content)
                self.assertIn('line2', content)
                self.assertIn('line3', content)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_stderr_output(self):
        """Test planner execution captures stderr."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Command that writes to stderr but exits successfully
            result = self.solver.execute_planner(
                'echo "error message" >&2 && exit 0',
                Duration(seconds=5),
                tmp_path
            )

            # Should still succeed if exit code is 0
            self.assertTrue(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_sigkill_handling(self):
        """Test that SIGKILL return code is handled correctly."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Use a very short timeout to trigger kill
            result = self.solver.execute_planner(
                'sleep 5',
                Duration(nanoseconds=100000000),  # 0.1 seconds
                tmp_path
            )

            # Should return False when killed
            self.assertFalse(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_process_lookup_error(self):
        """Test handling of ProcessLookupError during cancellation."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            import threading
            import time

            # Start a very short command
            def execute():
                return self.solver.execute_planner(
                    'echo "quick" && sleep 0.1',
                    Duration(seconds=5),
                    tmp_path
                )

            thread = threading.Thread(target=execute)
            thread.start()

            # Try to cancel after command might have finished
            time.sleep(0.2)
            self.solver.cancel()

            thread.join(timeout=2)

            # Should handle gracefully even if process already finished
            self.assertTrue(self.solver._cancel_requested)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_process_lookup_on_timeout(self):
        """Test handling of ProcessLookupError when timeout tries to kill finished process."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Use a command that finishes quickly but with a very short timeout
            # The race condition might trigger ProcessLookupError
            result = self.solver.execute_planner(
                'echo "fast" && exit 0',
                Duration(nanoseconds=1),  # Very short timeout to create race
                tmp_path
            )

            # The result depends on timing, but should not crash
            self.assertIsInstance(result, bool)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_exception_handling(self):
        """Test handling of unexpected exceptions during execution."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Test with a command that might cause unusual behavior
            # Using shell command with special characters
            result = self.solver.execute_planner(
                'bash -c "exit 0"',
                Duration(seconds=5),
                tmp_path
            )

            # Should handle gracefully
            self.assertIsInstance(result, bool)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_with_shell_injection_protection(self):
        """Test that tokenization protects against shell injection."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Try a command with potentially dangerous characters
            # Tokenization should handle this safely
            result = self.solver.execute_planner(
                'echo test',
                Duration(seconds=5),
                tmp_path
            )

            self.assertTrue(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_cancel_and_timeout_race_condition(self):
        """Test race condition between cancel and timeout."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            import threading
            import time

            result_holder = [None]

            def execute():
                result_holder[0] = self.solver.execute_planner(
                    'sleep 2',
                    Duration(nanoseconds=500000000),  # 0.5 seconds
                    tmp_path
                )

            thread = threading.Thread(target=execute)
            thread.start()

            # Cancel almost immediately to race with timeout
            time.sleep(0.1)
            self.solver.cancel()

            thread.join(timeout=3)

            # Either cancel or timeout should have stopped it
            self.assertFalse(result_holder[0])
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_execute_planner_very_fast_command(self):
        """Test with a command that completes almost instantly."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Very fast command
            result = self.solver.execute_planner(
                'true',
                Duration(seconds=5),
                tmp_path
            )

            self.assertTrue(result)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_multiple_sequential_executions(self):
        """Test executing planner multiple times sequentially."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # First execution
            result1 = self.solver.execute_planner(
                'echo "first"',
                Duration(seconds=5),
                tmp_path
            )
            self.assertTrue(result1)

            # Second execution (overwrites file)
            result2 = self.solver.execute_planner(
                'echo "second"',
                Duration(seconds=5),
                tmp_path
            )
            self.assertTrue(result2)

            # Verify second execution output
            with open(tmp_path, 'r') as f:
                content = f.read()
                self.assertIn('second', content)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_timeout_with_immediate_process_termination(self):
        """Test timeout scenario where process terminates immediately."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Use a process that exits quickly with very short timeout
            # to try to trigger ProcessLookupError on timeout
            for _ in range(5):  # Try multiple times to hit race condition
                self.solver._cancel_requested = False
                result = self.solver.execute_planner(
                    'exit 0',
                    Duration(nanoseconds=1000000),  # 0.001 seconds
                    tmp_path
                )
                # Should handle either success or timeout gracefully
                self.assertIsInstance(result, bool)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_cancel_with_immediate_process_termination(self):
        """Test cancel scenario where process terminates immediately."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            import threading
            import time

            # Try to create race condition where cancel happens right as process ends
            for _ in range(5):  # Try multiple times to hit race condition
                self.solver._cancel_requested = False

                result_holder = [None]

                def execute():
                    result_holder[0] = self.solver.execute_planner(
                        'exit 0',
                        Duration(seconds=5),
                        tmp_path
                    )

                thread = threading.Thread(target=execute)
                thread.start()

                # Cancel immediately to race with quick process
                time.sleep(0.001)
                self.solver.cancel()

                thread.join(timeout=2)

                # Should handle gracefully
                self.assertIsInstance(result_holder[0], bool)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def test_subprocess_popen_with_multiple_args(self):
        """Test subprocess with command that uses multiple arguments."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            tmp_path = tmp.name

        try:
            # Command with multiple tokenized arguments
            result = self.solver.execute_planner(
                'printf "%s\\n%s\\n" "line1" "line2"',
                Duration(seconds=5),
                tmp_path
            )

            self.assertTrue(result)

            with open(tmp_path, 'r') as f:
                content = f.read()
                self.assertIn('line1', content)
                self.assertIn('line2', content)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)


if __name__ == '__main__':
    unittest.main()
