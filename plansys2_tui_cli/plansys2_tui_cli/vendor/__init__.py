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

"""Textual, shipped for systems without a recent enough one (see README.md)."""

from importlib.metadata import PackageNotFoundError, version
import re

VENDORED_TEXTUAL_VERSION = '6.2.1'


def _numbers(text):
    # The release numbers: 6.2.0rc1 is 6.2.0
    numbers = []
    for part in text.split('.'):
        digits = re.match(r'\d*', part).group()
        if not digits:
            break
        numbers.append(int(digits))
        if digits != part:
            break
    return tuple(numbers)


def use_system_textual(system_version=None):
    """Return whether an installed Textual is at least as recent as the one shipped here."""
    if system_version is None:
        try:
            system_version = version('textual')
        except PackageNotFoundError:
            return False
    return _numbers(system_version) >= _numbers(VENDORED_TEXTUAL_VERSION)
