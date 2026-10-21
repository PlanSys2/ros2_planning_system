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

"""PlanSys2 Textual TUI: an overview in four quadrants and the plan log."""

from __future__ import annotations

import atexit
from collections import deque
import os
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException

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
    Footer, Label, RichLog, Static, TabbedContent, TabPane,
)


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
    PlanSys2 TUI, in two tabs.

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
    """

    BINDINGS = [
        ('1', "show_tab('overview')", 'Overview'),
        ('2', "show_tab('plan_log_tab')", 'Plan log'),
        ('c', 'clear_plan_log', 'Clear plan log'),
        ('q', 'quit', 'Quit'),
        ('ctrl+c', 'quit', 'Quit'),
    ]

    # The plan log keeps this many lines
    PLAN_LOG_LINES = 5000

    # ------------------------------------------------------------------
    def __init__(self, **kwargs):
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

        yield Footer()

    def on_mount(self) -> None:
        self._ros_thread = threading.Thread(
            target=self._ros_spin, daemon=True
        )
        self._ros_thread.start()
        self.set_interval(1.0, self._check_plansys2)
        self.set_interval(0.1, self._refresh_panels)

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

    def action_show_tab(self, tab: str) -> None:
        self.query_one(TabbedContent).active = tab

    def action_clear_plan_log(self) -> None:
        self._plan_log_lines.clear()
        if self._plan_log is not None:
            self._plan_log.clear()

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
