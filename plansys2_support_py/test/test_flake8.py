# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import os
import subprocess
import sys

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    env = os.environ.copy()
    env['FLAKE8_JOBS'] = '1'

    # Runs ament_flake8 in a fresh interpreter process.
    result = subprocess.run(
        [sys.executable, '-m', 'ament_flake8.main'],
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        'ament_flake8 failed.\n'
        f'STDOUT:\n{result.stdout}\n'
        f'STDERR:\n{result.stderr}\n'
    )
