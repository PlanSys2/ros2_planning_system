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

"""Tests of the Behavior tree tab."""

import struct

from plansys2_tui_cli.controller import bt_monitor_client as mc
from plansys2_tui_cli.controller import bt_tree
from plansys2_tui_cli.controller.bt_monitor import BTMonitor
import pytest

XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree" _fullpath="">
    <Sequence name="(move r1 a b):0" _uid="1">
      <WaitAtStartReq name="WaitAtStartReq" action="(move r1 a b):0" _uid="2"/>
      <ReactiveSequence name="(move r1 a b):0" _uid="3">
        <CheckOverAllReq name="CheckOverAllReq" action="(move r1 a b):0" _uid="4"/>
        <ExecuteAction name="ExecuteAction" action="(move r1 a b):0" _uid="5"/>
      </ReactiveSequence>
      <SubTree ID="Finish" _fullpath="finish" _uid="6"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Finish" _fullpath="finish">
    <ApplyAtEndEffect name="ApplyAtEndEffect" action="(move r1 a b):0" _uid="7"/>
  </BehaviorTree>
  <TreeNodesModel>
    <Action ID="ExecuteAction"><input_port name="action"/></Action>
  </TreeNodesModel>
</root>
"""


def test_request_and_reply_headers():
    request = mc.encode_request(mc.REQUEST_STATUS, unique_id=0x01020304)
    assert request == bytes([2]) + b'S' + bytes([4, 3, 2, 1])
    assert len(request) == mc.REQUEST_HEADER_SIZE

    tree_id = bytes(range(16))
    header = request + tree_id
    assert mc.decode_reply_header(header) == (2, b'S', 0x01020304, tree_id)
    assert mc.check_reply([header, b'payload'], request) == (tree_id, b'payload')


def test_bad_replies():
    request = mc.encode_request(mc.REQUEST_FULLTREE, unique_id=7)
    with pytest.raises(mc.BTMonitorError, match='wrong request header'):
        mc.check_reply([b'error', b'wrong request header'], request)
    with pytest.raises(mc.BTMonitorError, match='another request'):
        other = mc.encode_request(mc.REQUEST_FULLTREE, unique_id=8)
        mc.check_reply([other + bytes(16), b''], request)
    with pytest.raises(mc.BTMonitorError, match='protocol'):
        mc.check_reply([b'\x01' + request[1:] + bytes(16), b''], request)
    with pytest.raises(mc.BTMonitorError):
        mc.decode_reply_header(b'short')


def test_status_buffer():
    buffer = struct.pack('<HB', 1, bt_tree.RUNNING) + struct.pack('<HB', 300, 12)
    assert mc.decode_status(buffer) == {1: bt_tree.RUNNING, 300: 12}
    with pytest.raises(mc.BTMonitorError):
        mc.decode_status(b'\x00\x01')


def test_parse_tree_with_subtree():
    root = bt_tree.parse_tree(XML)
    assert [n.uid for n in root.walk()] == [1, 2, 3, 4, 5, 6, 7]
    assert root.name == '(move r1 a b):0'
    execute = [n for n in root.walk() if n.uid == 5][0]
    assert execute.kind == 'ExecuteAction' and execute.ports == {'action': '(move r1 a b):0'}
    subtree = [n for n in root.walk() if n.uid == 6][0]
    assert subtree.name == 'finish' and subtree.ports == {'ID': 'Finish'}
    assert subtree.children[0].kind == 'ApplyAtEndEffect'


def test_labels():
    node = bt_tree.BTNode(uid=5, kind='ExecuteAction', name='ExecuteAction',
                          ports={'action': '(move r1 a b):0'})
    assert '● ExecuteAction' in bt_tree.node_label(node, bt_tree.RUNNING)
    assert 'yellow' in bt_tree.node_label(node, bt_tree.RUNNING)
    assert '✘' in bt_tree.node_label(node, 13) and 'dim' in bt_tree.node_label(node, 13)
    assert '[bold]' not in bt_tree.node_label(node, bt_tree.IDLE)
    assert 'action=' not in bt_tree.node_label(node, bt_tree.IDLE, '(move r1 a b):0')
    assert 'action=' in bt_tree.node_label(node, bt_tree.IDLE, '(move r1 b c):1')
    node.ports = {'action': 'x' * 100}
    assert '…' in bt_tree.node_label(node, bt_tree.IDLE)
    node.name = '[red]'
    assert '\\[red]' in bt_tree.node_label(node, bt_tree.IDLE)


def test_summary_and_follow():
    root = bt_tree.parse_tree(XML)
    statuses = {1: bt_tree.RUNNING, 2: 12, 3: bt_tree.RUNNING, 4: bt_tree.SUCCESS,
                5: bt_tree.RUNNING}
    assert bt_tree.summary(root, statuses) == '7 nodes: 3 running, 2 success, 2 idle'
    assert bt_tree.follow(root, statuses) == ({1, 3}, {6})

    done = {n.uid: bt_tree.SUCCESS for n in root.walk()}
    assert bt_tree.follow(root, done) == ({1}, {3, 6})


def test_follow_touches_only_what_changed():
    assert bt_tree.follow_changes(None, {1, 3}, {6}) == ({1, 3}, {6})
    assert bt_tree.follow_changes({1, 3}, {1, 3}, {6}) == (set(), set())
    assert bt_tree.follow_changes({1, 3}, {1, 6}, {3}) == ({6}, {3})
    assert bt_tree.follow_changes({1, 6}, {1, 6}, {3}) == (set(), set())


def test_addresses_as_typed():
    assert mc.parse_address('1820') == ('localhost', 1820)
    assert mc.parse_address(' robot:1810 ') == ('robot', 1810)
    assert mc.parse_address('tcp://10.0.0.2:1800') == ('10.0.0.2', 1800)
    assert mc.parse_address(':1830', 'robot') == ('robot', 1830)
    for bad in ('', 'robot', 'robot:', 'robot:0', 'robot:70000', 'robot:18a0'):
        with pytest.raises(ValueError):
            mc.parse_address(bad)


class FakeClient:
    """Publisher with scripted answers."""

    def __init__(self, status=(), tree=()):
        self.status, self.tree = list(status), list(tree)

    @staticmethod
    def _next(answers):
        answer = answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    def fetch_status(self):
        return self._next(self.status)

    def fetch_tree(self):
        return self._next(self.tree)


def _monitor():
    return BTMonitor('localhost', 1800, callback=lambda snapshot: None)


def test_monitor_reads_the_tree_and_again_when_it_changes():
    running = {1: bt_tree.RUNNING}
    client = FakeClient(status=[(b'A', {}), (b'A', running), (b'A', running), (b'B', {}),
                                (b'B', {7: bt_tree.SUCCESS})],
                        tree=[(b'A', XML), (b'B', XML.replace('_uid="7"', '_uid="8"'))])
    monitor = _monitor()
    first = monitor.poll(client)
    assert first.connected and first.new_tree and first.statuses == running
    again = monitor.poll(client)
    assert again.connected and not again.new_tree
    changed = monitor.poll(client)
    assert changed.new_tree and changed.tree_id == b'B'
    assert 8 in [n.uid for n in changed.root.walk()]


def test_monitor_keeps_the_last_statuses_when_a_request_fails():
    running = {1: bt_tree.RUNNING}
    client = FakeClient(status=[(b'A', {}), (b'A', running), mc.BTMonitorError('no answer')],
                        tree=[(b'A', XML)])
    monitor = _monitor()
    monitor.poll(client)
    failed = monitor.poll(client)
    assert not failed.connected and failed.error == 'no answer'
    assert failed.statuses == running and failed.root is not None


def test_monitor_does_not_keep_a_tree_it_could_not_read():
    client = FakeClient(status=[(b'A', {}), (b'A', {}), (b'B', {}), (b'B', {}), (b'B', {})],
                        tree=[(b'A', XML), (b'B', '<root><not closed'), (b'B', XML)])
    monitor = _monitor()
    monitor.poll(client)
    broken = monitor.poll(client)
    assert broken.error.startswith('ParseError') and broken.tree_id == b'A'
    fixed = monitor.poll(client)
    assert fixed.new_tree and fixed.connected and fixed.tree_id == b'B'


def test_follow_old_failures_open_only_once_the_tree_ended():
    root = bt_tree.parse_tree(XML)
    statuses = {1: bt_tree.RUNNING, 2: bt_tree.RUNNING, 3: 13, 5: 13}
    expand, _ = bt_tree.follow(root, statuses)
    assert 3 not in expand
    statuses[1] = 13
    expand, _ = bt_tree.follow(root, statuses)
    assert 3 in expand
