#!/usr/bin/env python3

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

"""PlanSys2 Textual TUI: an overview in four quadrants, the plan log and the behavior tree."""

from __future__ import annotations

import atexit
from collections import deque
import functools
import os
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException

from ..controller import bt_tree
from ..controller.bt_monitor import BTMonitor, BTSnapshot
from ..controller.bt_monitor_client import parse_address
from ..controller.ros_controllers import (
    ActionExecutionProcessor,
    ExecutionTraceProcessor,
    KnowledgeProcessor,
    PerformersProcessor,
    PlanMonitorProcessor,
)
from ..vendor import missing_requirements, use_system_textual

# The system Textual if it is as recent as the one shipped here (MIT, see vendor/), else ours
if not use_system_textual():
    _missing = missing_requirements()
    if _missing:
        sys.exit('The PlanSys2 TUI is not supported on this system (as on ROS 2 Humble, Ubuntu '
                 '22.04): its Textual needs ' + ', '.join(_missing))
    _vendor_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
    if _vendor_dir not in sys.path:
        sys.path.insert(0, _vendor_dir)

from textual.app import App, ComposeResult  # noqa: E402, I100
from textual.containers import Horizontal, ScrollableContainer, Vertical  # noqa: E402
from textual.widgets import (  # noqa: E402
    Footer, Input, Label, RichLog, Static, TabbedContent, TabPane, Tree,
)
from rich.markup import escape  # noqa: E402, I100


class PlanLog(RichLog):
    """A RichLog that wraps its lines again when its width changes."""

    def __init__(self, on_width, **kwargs):
        super().__init__(**kwargs)
        self._on_width = on_width
        self._width = 0

    def on_resize(self, event) -> None:
        super().on_resize(event)
        if event.size.width and event.size.width != self._width:
            self._width = event.size.width
            self.call_after_refresh(self._on_width)


class PlanSys2App(App):
    """
    PlanSys2 TUI, in three tabs.

    Overview, in four quadrants:

    ┌────────────────────┬─────────────────────┐
    │  Action Execution  │  Plan Monitor       │
    │  (actions_hub)     │  (executing_plan +  │
    │                    │  action_exec_info)  │
    ├────────────────────┼─────────────────────┤
    │  Performers        │  Knowledge          │
    │  (performers_stat) │  (problem_expert/   │
    │                    │   knowledge)        │
    └────────────────────┴─────────────────────┘

    Plan log: what the executor does while a plan runs, and why
    (executor/execution_trace), one sentence per event.

    Behavior tree: the tree that runs, node by node, from the monitor of BehaviorTree.CPP (the
    executor with enable_groot_monitoring, or a BT action), at the address typed on its top
    (port 1800, that of the executor, at first). Following, it opens
    what runs or failed and closes what ended well.
    """

    CSS = """
    Screen {
        layout: vertical;
    }

    #tabs {
        height: 1fr;
    }

    #tabs TabPane {
        padding: 0;
    }

    #plan_log {
        border: round $primary;
        padding: 0 1;
        height: 1fr;
    }

    #grid {
        layout: horizontal;
        width: 100%;
        height: 1fr;
    }

    /* Left column — 50% */
    #left_col {
        width: 50%;
        height: 100%;
        layout: vertical;
    }

    /* Right column — 50% */
    #right_col {
        width: 50%;
        height: 100%;
        layout: vertical;
        margin-left: 1;
    }

    /* Each quadrant occupies half the column height */
    .quadrant {
        height: 1fr;
        layout: vertical;
        margin-bottom: 1;
    }

    .q_title {
        height: 1;
        padding: 0 1;
        background: $panel;
        color: $text;
        text-style: bold;
    }

    .q_box {
        border: round $primary;
        padding: 0 1;
        height: 1fr;
    }

    .q_box > Static {
        width: 100%;
        height: auto;
    }

    #bt_bar {
        height: 1;
        background: $panel;
    }

    #bt_address {
        width: 24;
    }

    #bt_header {
        width: 1fr;
        height: 1;
        padding: 0 1;
    }

    #bt_tree {
        border: round $primary;
        height: 1fr;
    }
    """

    BINDINGS = [
        ('1', "show_tab('overview')", 'Overview'),
        ('2', "show_tab('plan_log_tab')", 'Plan log'),
        ('3', "show_tab('bt_tab')", 'Behavior tree'),
        ('c', 'clear_plan_log', 'Clear plan log'),
        # The same key, the one shown that does what follows: Unfollow while following
        ('f', 'follow(False)', 'Unfollow tree'),
        ('f', 'follow(True)', 'Follow tree'),
        ('p', 'edit_bt_address', 'Tree address'),
        ('q', 'quit', 'Quit'),
        ('ctrl+c', 'quit', 'Quit'),
    ]

    # The plan log keeps this many lines
    PLAN_LOG_LINES = 5000

    # The last snapshot of the Behavior tree tab not drawn yet, and whether it follows the tree
    _bt_snapshot: BTSnapshot | None = None
    _bt_follow = True

    # ------------------------------------------------------------------
    def __init__(self, bt_host: str = 'localhost', bt_port: int = 1800, **kwargs):
        super().__init__(**kwargs)

        rclpy.init(args=None)
        atexit.register(self._ros_shutdown)
        self.node = rclpy.create_node('plansys2_tui')

        # Widget references (set during compose / mount)
        self._box_exec: Static | None = None
        self._box_plan: Static | None = None
        self._box_perf: Static | None = None
        self._box_know: Static | None = None
        self._plan_log: PlanLog | None = None
        self._plan_log_empty = True
        # RichLog wraps a line once, as wide as it is then: hidden, it is narrow
        self._plan_log_lines: deque = deque(maxlen=self.PLAN_LOG_LINES)

        # Last rendered text of the panels fed by PlanSys2, and whether it is up (#446)
        self._plan_text = '[dim]Waiting for plan…[/dim]'
        self._know_text = '[dim]Waiting for knowledge…[/dim]'
        self._executor_up = True
        self._problem_expert_up = True
        self._pending: dict = {}
        self._pending_events: list = []
        self._pending_lock = threading.Lock()

        # Behavior tree tab: its monitor polls the tree publisher in a thread of its own
        self._bt_host = bt_host
        self._bt_port = bt_port
        self._bt_monitor: BTMonitor | None = None
        self._bt_header: Label | None = None
        self._bt_tree: Tree | None = None
        self._bt_nodes: dict = {}  # uid -> TreeNode of the widget
        self._bt_drawn: dict = {}  # uid -> status its label shows
        # Every monitor started has a generation: what an older one sends late is dropped
        self._bt_generation = 0
        self._bt_snapshot: BTSnapshot | None = None
        self._bt_follow = True
        # The nodes following opened at the last draw; None: lay the tree out again
        self._bt_followed: set | None = None

        # ROS processors
        self._proc_exec = ActionExecutionProcessor(self.node, self._exec_cb)
        self._proc_perf = PerformersProcessor(self.node, self._perf_cb)
        self._proc_plan = PlanMonitorProcessor(self.node, self._plan_cb)
        self._proc_know = KnowledgeProcessor(self.node, self._know_cb)
        self._proc_trace = ExecutionTraceProcessor(self.node, self._trace_cb)

    # ------------------------------------------------------------------
    def compose(self) -> ComposeResult:
        with TabbedContent(id='tabs', initial='overview'):
            with TabPane('Overview', id='overview'):
                with Horizontal(id='grid'):
                    # ── Left column ──────────────────────────────────────────
                    with Vertical(id='left_col'):
                        # Top-left: Action Execution
                        with Vertical(classes='quadrant'):
                            yield Label(
                                'Action Execution  [performers_status / actions_hub]',
                                classes='q_title',
                            )
                            with ScrollableContainer(classes='q_box'):
                                self._box_exec = Static('[dim]Waiting for actions…[/dim]')
                                yield self._box_exec

                        # Bottom-left: Performers
                        with Vertical(classes='quadrant'):
                            yield Label(
                                'Performers  [performers_status]', classes='q_title'
                            )
                            with ScrollableContainer(classes='q_box'):
                                self._box_perf = Static('[dim]Waiting for performers…[/dim]')
                                yield self._box_perf

                    # ── Right column ─────────────────────────────────────────
                    with Vertical(id='right_col'):
                        # Top-right: Plan Monitor
                        with Vertical(classes='quadrant'):
                            yield Label(
                                'Plan Monitor  [executing_plan / action_execution_info]',
                                classes='q_title',
                            )
                            with ScrollableContainer(classes='q_box'):
                                self._box_plan = Static('[dim]Waiting for plan…[/dim]')
                                yield self._box_plan

                        # Bottom-right: Knowledge
                        with Vertical(classes='quadrant'):
                            yield Label(
                                'Knowledge  [problem_expert/knowledge]', classes='q_title'
                            )
                            with ScrollableContainer(classes='q_box'):
                                self._box_know = Static('[dim]Waiting for knowledge…[/dim]')
                                yield self._box_know

            # What the executor does, and why, as it happens
            with TabPane('Plan log', id='plan_log_tab'):
                self._plan_log = PlanLog(
                    self._redraw_plan_log, id='plan_log', markup=True, wrap=True,
                    auto_scroll=True, max_lines=self.PLAN_LOG_LINES)
                self._plan_log.write('[dim]Waiting for a plan to run…[/dim]')
                yield self._plan_log

            # The tree that runs, from its publisher
            with TabPane('Behavior tree', id='bt_tab'):
                with Horizontal(id='bt_bar'):
                    # Where the tree is: edited here, it changes at once (Enter)
                    yield Input(value=f'{self._bt_host}:{self._bt_port}', id='bt_address',
                                placeholder='host:port', compact=True)
                    self._bt_header = Label(
                        f'[dim]Connecting to tcp://{self._bt_host}:{self._bt_port}…[/dim]',
                        id='bt_header')
                    yield self._bt_header
                self._bt_tree = Tree('(no tree)', id='bt_tree')
                self._bt_tree.show_root = False
                yield self._bt_tree

        yield Footer()

    def on_mount(self) -> None:
        self._ros_thread = threading.Thread(
            target=self._ros_spin, daemon=True
        )
        self._ros_thread.start()
        self.set_interval(1.0, self._check_plansys2)
        self.set_interval(0.1, self._refresh_panels)
        self._start_bt_monitor()

    def _refresh_panels(self) -> None:
        # The panels are drawn at most 10 times a second, not once per message
        with self._pending_lock:
            pending, self._pending = self._pending, {}
            events, self._pending_events = self._pending_events, []
        for box, text in pending.items():
            box.update(text)
        if self._plan_log is not None and events:
            if self._plan_log_empty:
                self._plan_log.clear()
                self._plan_log_empty = False
            self._plan_log_lines.extend(events)
            for text in events:
                self._write_plan_log(text)
        with self._pending_lock:
            snapshot, self._bt_snapshot = self._bt_snapshot, None
        if snapshot is not None:
            self._draw_bt(snapshot)

    def action_show_tab(self, tab: str) -> None:
        self.query_one(TabbedContent).active = tab

    def action_clear_plan_log(self) -> None:
        self._plan_log_lines.clear()
        if self._plan_log is not None:
            self._plan_log.clear()

    def check_action(self, action: str, parameters: tuple) -> bool | None:
        # Of the two bindings of the follow key, only the one that changes something
        if action == 'follow':
            return parameters[0] != self._bt_follow
        return True

    def action_follow(self, follow: bool) -> None:
        self._bt_follow = follow
        self._bt_followed = None
        self.refresh_bindings()
        self.notify('Following the tree' if follow else 'Not following the tree')

    def action_edit_bt_address(self) -> None:
        self.action_show_tab('bt_tab')
        self.query_one('#bt_address', Input).focus()

    # ------------------------------------------------------------------
    # Behavior tree tab
    # ------------------------------------------------------------------
    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != 'bt_address':
            return
        try:
            host, port = parse_address(event.value, self._bt_host)
        except ValueError as error:
            self.notify(str(error), severity='error')
            return
        event.input.value = f'{host}:{port}'
        if self._bt_tree is not None:
            self._bt_tree.focus()
        if (host, port) != (self._bt_host, self._bt_port):
            self._bt_host, self._bt_port = host, port
            self._start_bt_monitor()

    def _start_bt_monitor(self) -> None:
        # One monitor at a time: the one of the address before stops, and its tree goes
        old = self._bt_monitor
        with self._pending_lock:
            self._bt_generation += 1
            monitor = BTMonitor(self._bt_host, self._bt_port,
                                functools.partial(self._bt_cb, self._bt_generation))
            self._bt_monitor = monitor
            self._bt_snapshot = None
        if old is not None:
            old.stop()
        if self._bt_tree is not None:
            self._bt_tree.clear()
        self._bt_nodes = {}
        self._bt_drawn = {}
        self._bt_followed = None
        if self._bt_header is not None:
            self._bt_header.update(f'[dim]Connecting to {escape(monitor.address)}…[/dim]')
        monitor.start()

    def _bt_cb(self, generation: int, snapshot: BTSnapshot) -> None:
        # From the thread of the monitor: the last one, drawn by _refresh_panels
        with self._pending_lock:
            if generation != self._bt_generation:
                return  # Of a monitor stopped for another address, maybe the same one
            if self._bt_snapshot is not None and self._bt_snapshot.new_tree:
                snapshot.new_tree = True  # A new tree not drawn yet is still new
            self._bt_snapshot = snapshot

    def _draw_bt(self, snapshot: BTSnapshot) -> None:
        if self._bt_tree is None or self._bt_header is None:
            return
        if snapshot.new_tree:
            self._build_bt(snapshot.root)
            self._bt_followed = None
        for uid, tree_node in self._bt_nodes.items():
            # Only the labels whose status changed: a big tree is not redrawn whole
            status = snapshot.statuses.get(uid, bt_tree.IDLE)
            if self._bt_drawn.get(uid) != status:
                node, parent_name = tree_node.data
                tree_node.set_label(bt_tree.node_label(node, status, parent_name))
                self._bt_drawn[uid] = status
        if self._bt_follow and snapshot.connected:
            expand, collapse = bt_tree.follow(snapshot.root, snapshot.statuses)
            to_expand, to_collapse = bt_tree.follow_changes(self._bt_followed, expand, collapse)
            self._bt_followed = expand
            for uid in to_expand:
                if uid in self._bt_nodes:
                    self._bt_nodes[uid].expand()
            for uid in to_collapse:
                if uid in self._bt_nodes:
                    self._bt_nodes[uid].collapse()
        follow = ' · following' if self._bt_follow else ' · [bold]not following[/bold]'
        address, error = escape(snapshot.address), escape(snapshot.error)
        if snapshot.connected:
            text = (f'{address} · tree {snapshot.tree_id.hex()[:8]} · '
                    f'{bt_tree.summary(snapshot.root, snapshot.statuses)}{follow}')
        elif snapshot.root is not None:
            text = (f'[bold red]{address}: {error}[/bold red] '
                    f'[dim](last tree known)[/dim]')
        else:
            text = (f'[dim]{address}: {error or "waiting"}. Is '
                    f'enable_groot_monitoring true there?[/dim]')
        self._bt_header.update(text)

    def _build_bt(self, root) -> None:
        self._bt_tree.clear()
        self._bt_nodes = {}
        self._bt_drawn = {}

        def add(parent, node, parent_name: str) -> None:
            label = bt_tree.node_label(node, bt_tree.IDLE, parent_name)
            data = (node, parent_name)
            if node.children:
                item = parent.add(label, data=data, expand=True)
                for child in node.children:
                    add(item, child, node.name)
            else:
                item = parent.add_leaf(label, data=data)
            self._bt_nodes[node.uid] = item

        if root is not None:
            add(self._bt_tree.root, root, '')
        self._bt_tree.root.expand()

    def _redraw_plan_log(self) -> None:
        if self._plan_log is None or not self._plan_log_lines:
            return
        self._plan_log.clear()
        for text in self._plan_log_lines:
            self._write_plan_log(text)

    def _write_plan_log(self, text: str) -> None:
        # Wrapped as wide as the log is, not as the console
        width = self._plan_log.scrollable_content_region.width
        self._plan_log.write(text, width=width if width > 0 else None)

    # ------------------------------------------------------------------
    # PlanSys2 liveness: its topics lose their publishers when it goes down
    # ------------------------------------------------------------------
    @staticmethod
    def _with_banner(text: str, up: bool, who: str) -> str:
        if up:
            return text
        return f'[bold red]{who} is not running: last known data[/bold red]\n\n{text}'

    def _check_plansys2(self) -> None:
        executor_up = self.node.count_publishers('executing_plan') > 0
        problem_expert_up = self.node.count_publishers('problem_expert/knowledge') > 0
        if executor_up != self._executor_up:
            self._executor_up = executor_up
            if self._box_plan is not None:
                self._box_plan.update(
                    self._with_banner(self._plan_text, executor_up, 'The executor'))
        if problem_expert_up != self._problem_expert_up:
            self._problem_expert_up = problem_expert_up
            if self._box_know is not None:
                self._box_know.update(
                    self._with_banner(self._know_text, problem_expert_up, 'The problem expert'))

    # ------------------------------------------------------------------
    # ROS background spin
    # ------------------------------------------------------------------
    def _ros_spin(self) -> None:
        try:
            rclpy.spin(self.node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _update_from_ros(self, box, text: str) -> None:
        # The last text of each panel, drawn by _refresh_panels
        with self._pending_lock:
            self._pending[box] = text

    def _exec_cb(self, performers: dict, active: dict) -> None:
        if self._box_exec is None:
            return
        text = ActionExecutionProcessor.combined2text(performers, active)
        self._update_from_ros(self._box_exec, text)

    def _perf_cb(self, performers: dict) -> None:
        if self._box_perf is None:
            return
        text = PerformersProcessor.performers2text(performers)
        self._update_from_ros(self._box_perf, text)

    def _plan_cb(self, plan, info: dict) -> None:
        if self._box_plan is None:
            return
        self._plan_text = PlanMonitorProcessor.plan2text(plan, info)
        self._update_from_ros(
            self._box_plan,
            self._with_banner(self._plan_text, self._executor_up, 'The executor'))

    def _trace_cb(self, msg) -> None:
        # Every event, in order: the log is not a last value
        text = ExecutionTraceProcessor.event2text(msg)
        with self._pending_lock:
            self._pending_events.append(text)

    def _know_cb(self, msg) -> None:
        if self._box_know is None:
            return
        self._know_text = KnowledgeProcessor.knowledge2text(msg)
        self._update_from_ros(
            self._box_know,
            self._with_banner(self._know_text, self._problem_expert_up, 'The problem expert'))

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def _ros_shutdown(self) -> None:
        if self._bt_monitor is not None:
            self._bt_monitor.stop()
        if rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
        if hasattr(self, '_ros_thread'):
            self._ros_thread.join(timeout=1.0)


# ------------------------------------------------------------------
def run_app() -> None:
    """Entry point for the ``plansys2_tui`` console script."""
    PlanSys2App().run()


if __name__ == '__main__':
    run_app()
