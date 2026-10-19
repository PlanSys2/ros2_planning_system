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

"""The TUI shows when PlanSys2 is down and closes cleanly (#446)."""

import threading
import unittest
from unittest.mock import MagicMock

from plansys2_tui_cli.tui.app import PlanSys2App


def bare_app():
    """App object without starting ROS nor Textual."""
    app = PlanSys2App.__new__(PlanSys2App)
    app._plan_text = 'plan'
    app._know_text = 'knowledge'
    app._executor_up = True
    app._problem_expert_up = True
    app._box_plan = MagicMock()
    app._box_know = MagicMock()
    app.node = MagicMock()
    app._pending = {}
    app._pending_events = []
    app._plan_log = None
    app._pending_lock = threading.Lock()
    return app


class TestPlanSys2Down(unittest.TestCase):

    def test_banner_only_when_down(self):
        self.assertEqual(PlanSys2App._with_banner('data', True, 'The executor'), 'data')
        down = PlanSys2App._with_banner('data', False, 'The executor')
        self.assertIn('The executor is not running', down)
        self.assertTrue(down.endswith('data'))

    def test_panels_follow_plansys2(self):
        app = bare_app()
        publishers = {'executing_plan': 1, 'problem_expert/knowledge': 1}
        app.node.count_publishers.side_effect = lambda topic: publishers[topic]

        app._check_plansys2()
        app._box_plan.update.assert_not_called()

        # The executor goes down: only its panel changes
        publishers['executing_plan'] = 0
        app._check_plansys2()
        self.assertIn('not running', app._box_plan.update.call_args[0][0])
        app._box_know.update.assert_not_called()

        # Checking again does not redraw
        app._check_plansys2()
        self.assertEqual(app._box_plan.update.call_count, 1)

        # Back: the last data without banner
        publishers['executing_plan'] = 1
        app._check_plansys2()
        app._box_plan.update.assert_called_with('plan')

    def test_messages_after_closing_are_dropped(self):
        # They only leave their text: nothing calls into the closed app
        app = bare_app()
        app.call_from_thread = MagicMock(side_effect=RuntimeError('App is not running'))
        app._update_from_ros(app._box_plan, 'late message')
        app.call_from_thread.assert_not_called()
        app._box_plan.update.assert_not_called()


if __name__ == '__main__':
    unittest.main()
