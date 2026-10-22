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

"""Tests of the Behavior tree tab of the TUI."""

import threading
from unittest.mock import MagicMock

from plansys2_tui_cli.controller import bt_tree
from plansys2_tui_cli.controller.bt_monitor import BTSnapshot
from plansys2_tui_cli.tui.app import PlanSys2App
from rich.text import Text

from test_bt_tree import XML


class FakeTreeNode:
    """Node of the Tree widget."""

    def __init__(self, label='', data=None):
        self.label, self.data = label, data
        self.children = []
        self.expanded = False
        self.label_sets = 0

    def add(self, label, data=None, expand=False):
        node = FakeTreeNode(label, data)
        node.expanded = expand
        self.children.append(node)
        return node

    def add_leaf(self, label, data=None):
        return self.add(label, data)

    def set_label(self, label):
        self.label = label
        self.label_sets += 1

    def expand(self):
        self.expanded = True

    def collapse(self):
        self.expanded = False


class FakeTree:
    """Tree widget."""

    def __init__(self):
        self.root = FakeTreeNode()

    def clear(self):
        self.root = FakeTreeNode()

    def focus(self):
        pass


def _app():
    app = PlanSys2App.__new__(PlanSys2App)
    app._pending_lock = threading.Lock()
    app._bt_snapshot = None
    app._bt_generation = 1
    app._bt_tree = FakeTree()
    app._bt_header = MagicMock()
    app._bt_nodes = {}
    app._bt_drawn = {}
    app._bt_follow = True
    app._bt_followed = None
    app._bt_host, app._bt_port = 'localhost', 1800
    return app


def _snapshot(statuses, new_tree=False, address='tcp://localhost:1800'):
    return BTSnapshot(address, connected=True, tree_id=b'A' * 16,
                      root=bt_tree.parse_tree(XML), new_tree=new_tree, statuses=statuses)


def _header(app):
    return Text.from_markup(app._bt_header.update.call_args.args[0]).plain


def test_snapshots_of_an_old_monitor_are_dropped():
    app = _app()
    app._bt_generation = 2
    app._bt_cb(1, _snapshot({}))
    assert app._bt_snapshot is None
    app._bt_cb(2, _snapshot({}, new_tree=True))
    app._bt_cb(2, _snapshot({1: bt_tree.RUNNING}))
    assert app._bt_snapshot.new_tree and app._bt_snapshot.statuses == {1: bt_tree.RUNNING}


def test_draw_builds_the_tree_and_follows_it():
    app = _app()
    app._draw_bt(_snapshot({1: bt_tree.RUNNING, 3: bt_tree.RUNNING, 5: bt_tree.RUNNING},
                           new_tree=True))
    assert sorted(app._bt_nodes) == [1, 2, 3, 4, 5, 6, 7]
    assert app._bt_nodes[3].expanded and not app._bt_nodes[6].expanded
    assert '●' in app._bt_nodes[5].label
    assert '7 nodes: 3 running' in _header(app) and 'following' in _header(app)


def test_draw_sets_only_the_labels_that_changed():
    app = _app()
    statuses = {1: bt_tree.RUNNING, 5: bt_tree.RUNNING}
    app._draw_bt(_snapshot(statuses, new_tree=True))
    sets = {uid: node.label_sets for uid, node in app._bt_nodes.items()}
    app._draw_bt(_snapshot({**statuses, 5: bt_tree.SUCCESS}))
    changed = [uid for uid, node in app._bt_nodes.items() if node.label_sets != sets[uid]]
    assert changed == [5]


def test_a_node_opened_by_hand_stays_open():
    app = _app()
    app._draw_bt(_snapshot({1: bt_tree.RUNNING, 3: bt_tree.RUNNING}, new_tree=True))
    app._draw_bt(_snapshot({1: bt_tree.RUNNING, 3: bt_tree.SUCCESS}))
    assert not app._bt_nodes[3].expanded
    app._bt_nodes[3].expand()
    app._draw_bt(_snapshot({1: bt_tree.RUNNING, 3: bt_tree.SUCCESS}))
    assert app._bt_nodes[3].expanded


def test_header_is_text_whatever_the_address():
    app = _app()
    app._draw_bt(BTSnapshot('tcp://[::1]:1800', error='[bold]no answer'))
    assert _header(app).startswith('tcp://[::1]:1800: [bold]no answer')


def test_follow_key_shows_what_it_does():
    app = _app()
    app.refresh_bindings = MagicMock()
    app.notify = MagicMock()
    assert app.check_action('follow', (False,)) and not app.check_action('follow', (True,))
    app.action_follow(False)
    assert not app._bt_follow and app._bt_followed is None
    assert app.check_action('follow', (True,)) and not app.check_action('follow', (False,))
    app.refresh_bindings.assert_called_once()


def test_address_typed_changes_the_monitor():
    app = _app()
    app._start_bt_monitor = MagicMock()
    app.notify = MagicMock()

    def submit(value):
        event = MagicMock(value=value)
        event.input.id = 'bt_address'
        app.on_input_submitted(event)
        return event

    assert submit('robot:1820').input.value == 'robot:1820'
    assert (app._bt_host, app._bt_port) == ('robot', 1820)
    app._start_bt_monitor.assert_called_once()

    submit('1820')
    app._start_bt_monitor.assert_called_once()

    submit('robot:port')
    app.notify.assert_called_once()
    assert (app._bt_host, app._bt_port) == ('robot', 1820)
