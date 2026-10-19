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

"""The Textual of the system is used only when it is recent enough."""

import threading
from unittest.mock import MagicMock

from plansys2_tui_cli.tui.app import PlanSys2App
from plansys2_tui_cli.vendor import use_system_textual, VENDORED_TEXTUAL_VERSION


def test_system_textual_only_if_recent():
    assert VENDORED_TEXTUAL_VERSION == '6.2.1'
    assert use_system_textual('6.2.1')
    assert use_system_textual('6.10.0')
    assert use_system_textual('8.2.8')
    assert not use_system_textual('2.1.2')
    assert not use_system_textual('6.2.0')
    assert not use_system_textual('6.2.0rc1')
    assert use_system_textual('7.0.0.dev1')


def test_panels_are_drawn_on_refresh():
    """ROS messages only leave text; the panels are drawn by the timer, the last text once."""
    app = PlanSys2App.__new__(PlanSys2App)
    app._pending = {}
    app._pending_events = []
    app._plan_log = None
    app._pending_lock = threading.Lock()
    box = MagicMock()
    for i in range(100):
        app._update_from_ros(box, f'text {i}')
    box.update.assert_not_called()
    app._refresh_panels()
    box.update.assert_called_once_with('text 99')
    app._refresh_panels()
    box.update.assert_called_once()
