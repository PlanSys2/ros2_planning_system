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


from plansys2_tui_cli.vendor import missing_requirements, TEXTUAL_REQUIREMENTS


def test_recent_enough_requirements_are_met():
    assert missing_requirements(dict(TEXTUAL_REQUIREMENTS)) == []
    assert missing_requirements(
        {'rich': '14.1.0', 'typing_extensions': '4.15.0', 'platformdirs': '4.3.8'}) == []


def test_old_or_absent_requirements_are_named():
    # As on Ubuntu 22.04 (Humble)
    old = {'rich': '11.2.0', 'typing_extensions': '3.10.0.2', 'platformdirs': '2.5.1'}
    missing = missing_requirements(old)
    assert len(missing) == 3
    assert missing[0] == 'rich >= 13.3.3 (found 11.2.0)'
    absent = missing_requirements({'rich': '13.3.3', 'platformdirs': '3.6.0'})
    assert absent == ['typing_extensions >= 4.4.0 (found none)']


def test_versions_compare_by_number():
    assert missing_requirements(
        {'rich': '13.10.0', 'typing_extensions': '4.4.0rc1', 'platformdirs': '3.6'}) == []
    assert missing_requirements(
        {'rich': '13.3.2', 'typing_extensions': '4.4.0', 'platformdirs': '3.6.0'}) == [
        'rich >= 13.3.3 (found 13.3.2)']
