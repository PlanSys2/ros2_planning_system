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

"""Utility helpers shared by all PlanSys2 CLI verbs."""

import re

_TAGS = {
    'black': '30', 'red': '31', 'green': '32', 'yellow': '33',
    'blue': '34', 'magenta': '35', 'cyan': '36', 'white': '37',
    'bright_red': '91', 'bright_green': '92', 'bright_yellow': '93',
    'bright_blue': '94', 'bright_cyan': '96',
    'bold': '1', 'dim': '2', 'underline': '4', 'italic': '3',
}

# A tag not escaped: "\\[move]" is text, as rich.markup.escape writes it
_OPEN_TAG_RE = re.compile(r'(?<!\\)\[([a-zA-Z_ ]+)\]')
_CLOSE_TAG_RE = re.compile(r'(?<!\\)\[/([a-zA-Z_ ]+)\]')
_ESCAPED_RE = re.compile(r'\\(\[)')


def bbcode_to_ansi(s: str, enable_color: bool) -> str:
    """Convert Rich-style bbcode tags to ANSI escape sequences (or strip them)."""
    if not enable_color:
        s = _OPEN_TAG_RE.sub('', s)
        s = _CLOSE_TAG_RE.sub('', s)
        return _ESCAPED_RE.sub(r'\1', s)

    # Handle Rich-style tags, including composite ones like "[bold underline]...[/bold underline]".
    # We parse space-separated style tokens and, if all are known, emit a single ANSI sequence
    # combining their codes (e.g., "\033[1;4m...\033[0m").
    def _replace_tags(match: re.Match) -> str:
        raw_tags = match.group(1)
        content = match.group(2)
        tokens = raw_tags.split()
        codes = []
        for token in tokens:
            key = token.lower()
            code = _TAGS.get(key)
            if code is None:
                # Unknown style in this composite; leave the tag as-is for now so it will be
                # stripped by the generic cleanup below, preserving current behavior.
                return match.group(0)
            codes.append(code)
        ansi_codes = ';'.join(codes)
        return f'\033[{ansi_codes}m{content}\033[0m'

    pattern = re.compile(
        r'(?<!\\)\[([a-zA-Z_ ]+)\](.*?)(?<!\\)\[/\s*\1\]', re.DOTALL | re.IGNORECASE)
    s = pattern.sub(_replace_tags, s)

    # strip any unmatched / unknown tags, and unescape the text
    s = _OPEN_TAG_RE.sub('', s)
    s = _CLOSE_TAG_RE.sub('', s)
    return _ESCAPED_RE.sub(r'\1', s)
