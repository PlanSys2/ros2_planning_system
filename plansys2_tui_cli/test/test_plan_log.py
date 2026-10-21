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

"""The plan log: each event of the execution trace as a sentence."""

import asyncio
import threading
from unittest.mock import MagicMock

from plansys2_msgs.msg import ExecutionEvent
from plansys2_tui_cli.controller.ros_controllers import ExecutionTraceProcessor
from plansys2_tui_cli.tui.app import PlanSys2App
from rich.text import Text

TRICKY = ['[/x]', '[move]', '[bold]r1', 'a[b]c', '[', ']', '\\[x]']


def event(kind, action='', detail='', plan_time=0.0, planned_time=0.0, shift=0.0):
    msg = ExecutionEvent()
    msg.type = kind
    msg.action = action
    msg.detail = detail
    msg.plan_time = plan_time
    msg.planned_time = planned_time
    msg.shift = shift
    return msg


def line(msg):
    # Raises MarkupError on a closing tag that nothing opened
    return Text.from_markup(ExecutionTraceProcessor.event2text(msg)).plain


def test_every_event_is_a_sentence():
    move = '(move r1 dock hall):3000'
    assert 'Plan starts: timed, 9 actions' in line(
        event(ExecutionEvent.PLAN_START, detail='timed, 9 actions'))
    assert '(move r1 dock hall) waits for its time in the plan, 3.000' in line(
        event(ExecutionEvent.WAIT, move, 'for its time in the plan, 3.000'))
    assert '(move r1 dock hall) is performed by move_performer' in line(
        event(ExecutionEvent.PERFORM, move, 'by move_performer'))
    assert '(move r1 dock hall) starts' in line(event(ExecutionEvent.START, move))
    assert '(move r1 dock hall) ends' in line(event(ExecutionEvent.END, move))
    assert ('(move r1 dock hall): the rest of the plan goes 0.120 s later: it starts late'
            in line(event(ExecutionEvent.SHIFT, move, '0.120 s later: it starts late')))
    assert '(move r1 dock hall) fails: its performer move_performer says: blocked' in line(
        event(ExecutionEvent.FAILURE, move, 'its performer move_performer says: blocked'))
    assert 'The plan fails: no plan' in line(event(ExecutionEvent.FAILURE, '', 'no plan'))
    assert 'A constraint is violated: (always (idle r1))' in line(
        event(ExecutionEvent.VIOLATION, '', '(always (idle r1))'))
    assert 'A new plan replaces the running one: going on: none' in line(
        event(ExecutionEvent.REPLAN, '', 'going on: none'))
    for how in ('succeeded', 'failed', 'cancelled', 'aborted'):
        assert f'The plan ends: {how}' in line(event(ExecutionEvent.PLAN_END, '', how))
    # A type it does not know yet is still shown
    assert 'event 99: new' in line(event(99, move, 'new'))


def test_plan_time_leads_each_line():
    assert line(event(ExecutionEvent.START, '(a):0', plan_time=12.3456)).startswith('   12.346')


def test_a_dependency_wait_reads_as_it_comes():
    shown = line(event(
        ExecutionEvent.WAIT, '(plug r1):20899', 'its start waits for the end of (move r1):17899'))
    assert '(plug r1): its start waits for the end of (move r1)' in shown
    assert 'waits its' not in shown


def test_ids_are_left_out():
    shown = line(event(ExecutionEvent.WAIT, '(a b):1500', 'its start waits for the end of (c):0'))
    assert ':1500' not in shown and ':0' not in shown
    # Not what only looks like one
    assert '(at 3:30)' in line(event(ExecutionEvent.PERFORM, '(at 3:30)', 'by p'))
    assert 'a:b' in line(event(ExecutionEvent.PERFORM, 'a:b', 'by p'))


def test_timing_of_starts_and_ends():
    def timing(plan_time, planned_time, shift=0.0):
        return line(event(ExecutionEvent.END, '(a):0', '', plan_time, planned_time, shift))

    assert 'on time (planned at 3.000)' in timing(3.0, 3.0)
    assert 'on time' in timing(3.049, 3.0)
    assert 'on time' in timing(2.951, 3.0)
    assert '0.120 s late (planned at 3.000)' in timing(3.12, 3.0)
    assert '0.500 s early' in timing(2.5, 3.0)
    # Late on the plan as it was shifted, not on the original one
    assert 'on time (planned at 3.000, the plan 0.200 s later)' in timing(3.2, 3.0, 0.2)
    assert '0.100 s late (planned at 3.000, the plan 0.200 s later)' in timing(3.3, 3.0, 0.2)
    # A shift too small to tell is not said
    assert 'later' not in timing(3.0, 3.0, 0.0005)


def test_text_from_ros_is_not_markup():
    for text in TRICKY:
        for kind in (ExecutionEvent.PLAN_START, ExecutionEvent.WAIT, ExecutionEvent.PERFORM,
                     ExecutionEvent.START, ExecutionEvent.END, ExecutionEvent.SHIFT,
                     ExecutionEvent.FAILURE, ExecutionEvent.VIOLATION, ExecutionEvent.REPLAN,
                     ExecutionEvent.PLAN_END):
            shown = line(event(kind, f'(move {text}):0', f'detail {text}'))
            if kind not in (ExecutionEvent.START, ExecutionEvent.END):
                assert f'detail {text}' in shown, (kind, text)
            if kind not in (ExecutionEvent.PLAN_START, ExecutionEvent.PLAN_END,
                            ExecutionEvent.VIOLATION, ExecutionEvent.REPLAN):
                assert f'(move {text})' in shown, (kind, text)


def test_events_are_all_written_in_order():
    """Unlike the panels, the log gets every event, once, in order."""
    app = PlanSys2App.__new__(PlanSys2App)
    app._pending = {}
    app._pending_events = []
    app._pending_lock = threading.Lock()
    app._plan_log = MagicMock()
    app._plan_log.scrollable_content_region.width = 100
    app._plan_log_empty = True
    app._plan_log_lines = []
    for i in range(50):
        app._trace_cb(event(ExecutionEvent.WAIT, f'(a{i}):0', 'for x'))
    app._plan_log.write.assert_not_called()
    app._refresh_panels()
    # The waiting message goes with the first event
    app._plan_log.clear.assert_called_once()
    written = [Text.from_markup(c.args[0]).plain for c in app._plan_log.write.call_args_list]
    assert [w.split()[1] for w in written] == [f'(a{i})' for i in range(50)]
    app._refresh_panels()
    assert app._plan_log.write.call_count == 50
    app._trace_cb(event(ExecutionEvent.WAIT, '(b):0', 'for x'))
    app._refresh_panels()
    assert app._plan_log.write.call_count == 51
    app._plan_log.clear.assert_called_once()


def test_tabs_and_plan_log_in_the_app():
    async def run():
        app = PlanSys2App()
        async with app.run_test() as pilot:
            tabs = app.query_one('#tabs')
            assert tabs.active == 'overview'
            await pilot.press('2')
            assert tabs.active == 'plan_log_tab'
            app._trace_cb(event(ExecutionEvent.PLAN_START, detail='timed, 1 actions'))
            app._trace_cb(event(ExecutionEvent.START, '(move r1):0', '', 0.01, 0.0))
            await pilot.pause(0.3)
            log = app.query_one('#plan_log')
            shown = '\n'.join(strip.text for strip in log.lines)
            assert 'Plan starts: timed, 1 actions' in shown
            assert '(move r1) starts on time' in shown
            assert 'Waiting for a plan' not in shown
            await pilot.press('c')
            assert not log.lines
            await pilot.press('1')
            assert tabs.active == 'overview'
        app._ros_shutdown()

    asyncio.run(run())


def test_lines_use_the_width_of_the_screen():
    """Events that come while the overview is shown are not cut to a narrow log."""
    long_detail = 'for its at start requirements: ' + ' '.join(f'(at r{i} dock)' for i in range(8))

    async def run():
        app = PlanSys2App()
        async with app.run_test(size=(160, 40)) as pilot:
            app._trace_cb(event(ExecutionEvent.WAIT, '(move r1):0', long_detail, 1.0))
            await pilot.pause(0.3)
            await pilot.press('2')
            await pilot.pause(0.3)
            log = app.query_one('#plan_log')
            widths = [len(strip.text.rstrip()) for strip in log.lines]
            assert max(widths) > 100, widths
            # Narrower, it wraps again
            await pilot.resize_terminal(90, 40)
            await pilot.pause(0.3)
            assert max(len(strip.text.rstrip()) for strip in log.lines) <= 90
            assert len(log.lines) > 1
        app._ros_shutdown()

    asyncio.run(run())
