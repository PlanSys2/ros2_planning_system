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


def _padded(text, other):
    # The numbers of text, with zeros to as many as other has: 3.6 is 3.6.0
    numbers = _numbers(text)
    return numbers + (0,) * (len(_numbers(other)) - len(numbers))


# What the shipped Textual needs to be imported; Ubuntu 22.04 (Humble) has older ones
TEXTUAL_REQUIREMENTS = {'rich': '13.3.3', 'typing_extensions': '4.4.0', 'platformdirs': '3.6.0'}


def missing_requirements(installed=None):
    """Return the requirements of the shipped Textual this system does not meet."""
    missing = []
    for package, minimum in TEXTUAL_REQUIREMENTS.items():
        if installed is not None:
            found = installed.get(package)
        else:
            try:
                found = version(package)
            except PackageNotFoundError:
                found = None
        if found is None or _padded(found, minimum) < _padded(minimum, found):
            missing.append(f'{package} >= {minimum} (found {found or "none"})')
    return missing
