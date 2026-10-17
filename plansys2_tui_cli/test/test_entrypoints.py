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

"""The plansys2_tui console script is registered and loads."""

from importlib.metadata import entry_points
import unittest


class TestConsoleEntryPoints(unittest.TestCase):

    def test_plansys2_tui_entrypoint_loads(self):
        eps = entry_points(group='console_scripts', name='plansys2_tui')
        ep = next(iter(eps), None)

        self.assertIsNotNone(
            ep,
            'Console script "plansys2_tui" is not registered in console_scripts.',
        )
        self.assertEqual(
            ep.value,
            'plansys2_tui_cli.tui.app:run_app',
            'The "plansys2_tui" entry point points to an unexpected target.',
        )

        try:
            loaded = ep.load()
        except Exception as exc:
            self.fail(
                'Failed to load the "plansys2_tui" entry point: '
                f'{type(exc).__name__}: {exc}'
            )

        self.assertTrue(
            callable(loaded),
            'The loaded "plansys2_tui" entry point is not callable.',
        )


if __name__ == '__main__':
    unittest.main()
