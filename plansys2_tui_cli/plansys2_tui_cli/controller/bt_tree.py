# Copyright 2026 Juan Carlos Manzanares Serrano
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

"""Behavior tree model and rendering for the Behavior tree tab."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Set, Tuple
import xml.etree.ElementTree as ET

from rich.markup import escape

IDLE, RUNNING, SUCCESS, FAILURE, SKIPPED = range(5)
_IDLE_AFTER = 10

_METADATA = {'name', 'ID', '_uid', '_fullpath'}
PORTS_WIDTH = 90

_STYLE = {
    IDLE: ('○', 'dim'),
    RUNNING: ('●', 'bold yellow'),
    SUCCESS: ('✔', 'green'),
    FAILURE: ('✘', 'bold red'),
    SKIPPED: ('–', 'dim cyan'),
}
_NAME = {IDLE: 'idle', RUNNING: 'running', SUCCESS: 'success', FAILURE: 'failure',
         SKIPPED: 'skipped'}


@dataclass
class BTNode:
    """Node of a behavior tree."""

    uid: int
    kind: str
    name: str = ''
    ports: Dict[str, str] = field(default_factory=dict)
    children: List['BTNode'] = field(default_factory=list)

    def walk(self) -> Iterator['BTNode']:
        """Iterate over this node and its descendants, depth first."""
        yield self
        for child in self.children:
            yield from child.walk()


def parse_tree(xml: str) -> Optional[BTNode]:
    """Return the root of the tree in the XML of a publisher."""
    root = ET.fromstring(xml)
    trees = [e for e in root if e.tag == 'BehaviorTree']
    if not trees:
        return None
    by_path = {t.get('_fullpath', ''): t for t in trees}

    def build(elem: ET.Element) -> BTNode:
        node = BTNode(
            uid=int(elem.get('_uid', '-1')), kind=elem.tag,
            name=elem.get('name', elem.get('ID', '')),
            ports={k: v for k, v in elem.attrib.items() if k not in _METADATA})
        node.children = [build(child) for child in elem]
        if elem.get('ID') is not None and not node.children:
            path = elem.get('_fullpath', '')
            node.name = path.rsplit('/', 1)[-1] or elem.get('ID')
            node.ports = {'ID': elem.get('ID'), **node.ports}
            subtree = by_path.get(path)
            if subtree is not None and subtree is not trees[0] and len(subtree):
                node.children = [build(subtree[0])]
        return node

    main = by_path.get('', trees[0])
    return build(main[0]) if len(main) else None


def base_status(status: int) -> Tuple[int, bool]:
    """Return (status, idle)."""
    if status >= _IDLE_AFTER:
        return status - _IDLE_AFTER, True
    return status, status == IDLE


def node_label(node: BTNode, status: int, parent_name: str = '') -> str:
    """Return the label of a node as Rich markup."""
    shown, idle = base_status(status)
    symbol, style = _STYLE.get(shown, ('?', 'magenta'))
    if idle and shown != IDLE:
        style = f'{style} dim'
    text = f'[{style}]{symbol} {escape(node.kind)}[/]'
    if node.name and node.name != node.kind:
        text += f' [bold]{escape(node.name)}[/bold]'
    ports = ' '.join(f'{k}={v}' for k, v in node.ports.items()
                     if not parent_name or v != parent_name)
    if ports:
        if len(ports) > PORTS_WIDTH:
            ports = ports[:PORTS_WIDTH - 1] + '…'
        text += f' [dim]{escape(ports)}[/dim]'
    return text


def summary(root: Optional[BTNode], statuses: Dict[int, int]) -> str:
    """Return the number of nodes in each status."""
    if root is None:
        return 'empty tree'
    nodes = list(root.walk())
    counts: Dict[int, int] = {}
    for node in nodes:
        shown, idle = base_status(statuses.get(node.uid, IDLE))
        key = IDLE if idle and shown in (IDLE, RUNNING) else shown
        counts[key] = counts.get(key, 0) + 1
    parts = [f'{counts[s]} {_NAME[s]}' for s in (RUNNING, SUCCESS, FAILURE, SKIPPED, IDLE)
             if counts.get(s)]
    return f'{len(nodes)} nodes: ' + ', '.join(parts)


def follow(root: Optional[BTNode], statuses: Dict[int, int]) -> Tuple[Set[int], Set[int]]:
    """Return (expand, collapse) to follow the execution."""
    expand: Set[int] = set()
    collapse: Set[int] = set()
    if root is None:
        return expand, collapse
    root_running = statuses.get(root.uid, IDLE) == RUNNING

    def visit(node: BTNode) -> bool:
        shown, idle = base_status(statuses.get(node.uid, IDLE))
        hot = shown == RUNNING and not idle or shown == FAILURE and not (idle and root_running)
        for child in node.children:
            hot = visit(child) or hot
        if node.children:
            (expand if hot or node is root else collapse).add(node.uid)
        return hot

    visit(root)
    return expand, collapse


def follow_changes(previous: Optional[Set[int]], expand: Set[int],
                   collapse: Set[int]) -> Tuple[Set[int], Set[int]]:
    """Return (expand, collapse) for the nodes whose state changed since previous."""
    if previous is None:
        return set(expand), set(collapse)
    return expand - previous, collapse & previous
