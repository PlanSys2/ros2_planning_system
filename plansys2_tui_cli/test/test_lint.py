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

"""Linters over the code of this package; vendor/ is Textual, as published."""

import glob
import os

from ament_copyright.main import main as copyright_main
from ament_flake8.main import main_with_errors as flake8_main
from ament_pep257.main import main as pep257_main
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENDOR = os.path.join(ROOT, 'plansys2_tui_cli', 'vendor')


@pytest.mark.linter
def test_flake8():
    rc, errors = flake8_main(argv=[ROOT, '--exclude', VENDOR])
    assert rc == 0, f'{len(errors)} flake8 errors'


@pytest.mark.linter
def test_pep257():
    assert pep257_main(argv=[ROOT, '--exclude', VENDOR]) == 0


@pytest.mark.linter
def test_copyright():
    vendored = glob.glob(os.path.join(VENDOR, '**', '*'), recursive=True)
    assert copyright_main(argv=[ROOT, '--exclude'] + vendored) == 0
