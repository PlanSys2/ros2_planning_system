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

"""Helpers for the tests of the clients with mocked services."""

from unittest.mock import patch

from plansys2_support_py.ClientBase import ClientBase


def _done(self, future, timeout_sec):
    return bool(future.done())


def _active(self):
    return True


def no_wait():
    """Patch the waits of the clients: the component is active, a mocked future answers as is."""
    return patch.multiple(ClientBase, _wait_for=_done, wait_active=_active)


def always_active(test):
    """In a test with mocked services, the component is active: no real wait for it."""
    patcher = patch.object(ClientBase, 'wait_active', autospec=True, return_value=True)
    patcher.start()
    test.addCleanup(patcher.stop)
