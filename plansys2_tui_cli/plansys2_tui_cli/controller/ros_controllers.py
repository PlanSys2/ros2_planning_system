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

"""ROS 2 controller/processor classes for PlanSys2 TUI and CLI."""

from __future__ import annotations

import math
import re
import time

from plansys2_msgs.msg import (
    ActionExecution,
    ActionExecutionInfo,
    ActionPerformerStatus,
    ExecutionEvent,
    Knowledge,
    Plan,
)
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rich.markup import escape

# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------
_RELIABLE_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
_RELIABLE_DEEP_QOS = QoSProfile(depth=200, reliability=QoSReliabilityPolicy.RELIABLE)
_TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
# The executor keeps its last 1000 events for those who come later
_TRACE_QOS = QoSProfile(
    depth=1000,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# ---------------------------------------------------------------------------
# ColorMap helpers (Rich markup)
# ---------------------------------------------------------------------------
_PERFORMER_STATE_COLOR = {
    ActionPerformerStatus.NOT_READY: 'red',
    ActionPerformerStatus.READY: 'green',
    ActionPerformerStatus.RUNNING: 'yellow',
    ActionPerformerStatus.FAILURE: 'bright_red',
}
_PERFORMER_STATE_LABEL = {
    ActionPerformerStatus.NOT_READY: 'NOT_READY',
    ActionPerformerStatus.READY: 'READY',
    ActionPerformerStatus.RUNNING: 'RUNNING',
    ActionPerformerStatus.FAILURE: 'FAILURE',
}

_INFO_STATUS_COLOR = {
    ActionExecutionInfo.NOT_EXECUTED: 'dim',
    ActionExecutionInfo.EXECUTING: 'yellow',
    ActionExecutionInfo.FAILED: 'bright_red',
    ActionExecutionInfo.SUCCEEDED: 'green',
    ActionExecutionInfo.CANCELLED: 'red',
}
_INFO_STATUS_LABEL = {
    ActionExecutionInfo.NOT_EXECUTED: 'NOT_EXECUTED',
    ActionExecutionInfo.EXECUTING: 'EXECUTING',
    ActionExecutionInfo.FAILED: 'FAILED',
    ActionExecutionInfo.SUCCEEDED: 'SUCCEEDED',
    ActionExecutionInfo.CANCELLED: 'CANCELLED',
}

_ACTION_TYPE_LABEL = {
    ActionExecution.REQUEST: 'REQUEST',
    ActionExecution.RESPONSE: 'RESPONSE',
    ActionExecution.CONFIRM: 'CONFIRM',
    ActionExecution.REJECT: 'REJECT',
    ActionExecution.FEEDBACK: 'FEEDBACK',
    ActionExecution.FINISH: 'FINISH',
    ActionExecution.CANCEL: 'CANCEL',
    ActionExecution.UPDATE: 'UPDATE',
}

_ACTION_TYPE_COLOR = {
    ActionExecution.REQUEST:  'blue',
    ActionExecution.RESPONSE: 'cyan',
    ActionExecution.CONFIRM:  'green',
    ActionExecution.REJECT:   'red',
    ActionExecution.FEEDBACK: 'yellow',
    ActionExecution.FINISH:   'bright_green',
    ActionExecution.CANCEL:   'bright_red',
    ActionExecution.UPDATE:   'magenta',
}


def _fmt_args(args) -> str:
    return ' '.join(args) if args else ''


# Text from ROS is shown as it is: "[move]" or "[/x]" in it are not markup
def _e(text) -> str:
    return escape(str(text))


# ---------------------------------------------------------------------------
# ActionExecutionProcessor
# Performer-centric view: subscribes to both performers_status and
# actions_hub.  For each known performer shows its state; when RUNNING,
# overlays the latest ActionExecution details.
# ---------------------------------------------------------------------------
class ActionExecutionProcessor:
    """Subscribe to ``performers_status`` + ``actions_hub`` for a combined view."""

    def __init__(self, node, callback):
        self._performers: dict[str, ActionPerformerStatus] = {}  # node_name -> msg
        self._active: dict[str, ActionExecution] = {}            # node_id -> last msg
        self._callback = callback
        self._perf_sub = node.create_subscription(
            ActionPerformerStatus,
            'performers_status',
            self._perf_callback,
            _RELIABLE_QOS,
        )
        self._exec_sub = node.create_subscription(
            ActionExecution,
            'actions_hub',
            self._exec_callback,
            _RELIABLE_QOS,
        )

    def _perf_callback(self, msg: ActionPerformerStatus) -> None:
        self._performers[msg.node_name] = msg
        self._callback(self._performers, self._active)

    def _exec_callback(self, msg: ActionExecution) -> None:
        if msg.type == ActionExecution.FINISH:
            self._active.pop(msg.node_id, None)
        else:
            self._active[msg.node_id] = msg
        self._callback(self._performers, self._active)

    @staticmethod
    def combined2text(performers: dict, active: dict) -> str:
        """Render performers dict + active actions to a Rich-markup string."""
        if not performers:
            return '[dim]Waiting for performers…[/dim]'
        lines = []
        for node_name, perf in sorted(performers.items()):
            color = _PERFORMER_STATE_COLOR.get(perf.state, 'white')
            state_label = _PERFORMER_STATE_LABEL.get(perf.state, str(perf.state))
            lines.append(
                f'[bold]{_e(node_name)}[/bold]  [{color}]{state_label}[/{color}]'
            )
            if perf.state == ActionPerformerStatus.RUNNING:
                # find matching active action by node_id == node_name
                exec_msg = active.get(node_name)
                if exec_msg:
                    type_label = _ACTION_TYPE_LABEL.get(exec_msg.type, str(exec_msg.type))
                    type_color = _ACTION_TYPE_COLOR.get(exec_msg.type, 'white')
                    args_str = _fmt_args(exec_msg.arguments)
                    action_str = f'{exec_msg.action} {args_str}'.strip()
                    lines.append(f'  action : [white]{_e(action_str)}[/white]')
                    lines.append(
                        f'  type   : [{type_color}]{type_label}[/{type_color}]'
                    )
                    if exec_msg.type == ActionExecution.FEEDBACK:
                        pct = int(exec_msg.completion * 100)
                        bar_len = 20
                        filled = int(bar_len * exec_msg.completion)
                        bar = '█' * filled + '░' * (bar_len - filled)
                        lines.append(f'  [{bar}] {pct}%')
                        if exec_msg.status:
                            lines.append(f'  [dim]{_e(exec_msg.status)}[/dim]')
                    elif exec_msg.type == ActionExecution.RESPONSE:
                        ok = '[green]OK[/green]' if exec_msg.success else '[red]FAIL[/red]'
                        lines.append(f'  result : {ok}')
            lines.append('')
        return '\n'.join(lines).rstrip()


# ---------------------------------------------------------------------------
# PerformersProcessor
# Tracks performer status from /performers_status.
# ---------------------------------------------------------------------------
class PerformersProcessor:
    """Subscribe to ``performers_status`` and track each performer."""

    def __init__(self, node, callback):
        self._performers: dict[str, ActionPerformerStatus] = {}
        self._callback = callback
        self._sub = node.create_subscription(
            ActionPerformerStatus,
            'performers_status',
            self._internal_callback,
            _RELIABLE_QOS,
        )

    def _internal_callback(self, msg: ActionPerformerStatus) -> None:
        self._performers[msg.node_name] = msg
        self._callback(self._performers)

    @staticmethod
    def performers2text(performers: dict) -> str:
        """Render performers dict to a Rich-markup string."""
        if not performers:
            return '[dim]Waiting for performers…[/dim]'
        lines = []
        for node_name, msg in sorted(performers.items()):
            color = _PERFORMER_STATE_COLOR.get(msg.state, 'white')
            state_label = _PERFORMER_STATE_LABEL.get(msg.state, str(msg.state))
            args_str = _fmt_args(msg.specialized_arguments)
            action_str = f'{msg.action} {args_str}'.strip() if msg.action else '—'
            lines.append(
                f'[bold]{_e(node_name)}[/bold]  '
                f'[{color}]{state_label}[/{color}]'
            )
            lines.append(f'  action : {_e(action_str)}')
            lines.append('')
        return '\n'.join(lines).rstrip()


# ---------------------------------------------------------------------------
# PlanMonitorProcessor
# Combines the executing plan (/executing_plan) with execution info
# (/action_execution_info).
# ---------------------------------------------------------------------------
class PlanMonitorProcessor:
    """Subscribe to ``executing_plan`` and ``action_execution_info``."""

    def __init__(self, node, callback):
        self._plan: Plan | None = None
        self._info: dict[str, ActionExecutionInfo] = {}  # action_full_name -> msg
        self._callback = callback
        self._plan_sub = node.create_subscription(
            Plan,
            'executing_plan',
            self._plan_callback,
            _TRANSIENT_QOS,
        )
        self._info_sub = node.create_subscription(
            ActionExecutionInfo,
            'action_execution_info',
            self._info_callback,
            _RELIABLE_DEEP_QOS,
        )

    def _plan_callback(self, msg: Plan) -> None:
        self._plan = msg
        self._callback(self._plan, self._info)

    def _info_callback(self, msg: ActionExecutionInfo) -> None:
        self._info[msg.action_full_name] = msg
        self._callback(self._plan, self._info)

    @staticmethod
    def plan2text(plan: Plan | None, info: dict) -> str:
        """Render plan + execution info to a Rich-markup string."""
        if plan is None:
            return '[dim]Waiting for plan…[/dim]'
        if not plan.items:
            return '[dim]Plan is empty.[/dim]'

        lines = []
        bar_len = 15
        for item in plan.items:
            action_str = item.action.strip()

            # action_full_name is "action:<time in ms>", rounded as the executor does
            # Try exact key plus ±1 tolerance for floating-point differences.
            time_int = math.floor(item.time * 1000 + 0.5)
            ainfo = None
            for candidate in (time_int, time_int + 1, time_int - 1):
                probe = f'{action_str}:{candidate}'
                if probe in info:
                    ainfo = info[probe]
                    break

            if ainfo is not None:
                status_val = ainfo.status
                color = _INFO_STATUS_COLOR.get(status_val, 'white')
                status_label = _INFO_STATUS_LABEL.get(status_val, str(status_val))
                pct = int(ainfo.completion * 100)
                filled = int(bar_len * ainfo.completion)
                bar = '█' * filled + '░' * (bar_len - filled)
                lines.append(
                    f'[{color}]{status_label:12}[/{color}] '
                    f'[{bar}] {pct:3d}%  '
                    f'[bold]{_e(action_str)}[/bold]'
                )
            else:
                empty_bar = '░' * bar_len
                lines.append(
                    f'[dim]PENDING       [{empty_bar}]   0%  {_e(action_str)}[/dim]'
                )
        return '\n'.join(lines)


# ---------------------------------------------------------------------------
# KnowledgeProcessor
# Receives the problem expert knowledge (instances/predicates/functions/goal/metric).
# ---------------------------------------------------------------------------
class KnowledgeProcessor:
    """Subscribe to ``problem_expert/knowledge``."""

    def __init__(self, node, callback):
        self._knowledge: Knowledge | None = None
        self._callback = callback
        self._sub = node.create_subscription(
            Knowledge,
            'problem_expert/knowledge',
            self._internal_callback,
            _TRANSIENT_QOS,
        )

    def _internal_callback(self, msg: Knowledge) -> None:
        self._knowledge = msg
        self._callback(msg)

    @staticmethod
    def knowledge2text(msg: Knowledge) -> str:
        """Render a Knowledge message to a Rich-markup string."""
        lines = []

        # Instances
        lines.append('[bold underline]Instances[/bold underline]')
        if msg.instances:
            for inst in msg.instances:
                lines.append(f'  [cyan]{_e(inst)}[/cyan]')
        else:
            lines.append('  [dim](none)[/dim]')
        lines.append('')

        # Predicates
        lines.append('[bold underline]Predicates[/bold underline]')
        if msg.predicates:
            for pred in msg.predicates:
                lines.append(f'  [green]{_e(pred)}[/green]')
        else:
            lines.append('  [dim](none)[/dim]')
        lines.append('')

        # Functions
        lines.append('[bold underline]Functions[/bold underline]')
        if msg.functions:
            for func in msg.functions:
                lines.append(f'  [yellow]{_e(func)}[/yellow]')
        else:
            lines.append('  [dim](none)[/dim]')
        lines.append('')

        # Goal
        lines.append('[bold underline]Goal[/bold underline]')
        goal_text = msg.goal.strip() if msg.goal else ''
        if goal_text:
            lines.append(f'  [magenta]{_e(goal_text)}[/magenta]')
        else:
            lines.append('  [dim](none)[/dim]')

        # Metric, only when the problem has one
        metric_text = msg.metric.strip() if msg.metric else ''
        if metric_text:
            lines.append('')
            lines.append('[bold underline]Metric[/bold underline]')
            lines.append(f'  [blue]{_e(metric_text)}[/blue]')

        # Constraints, only when the problem has them
        constraints_text = msg.constraints.strip() if msg.constraints else ''
        if constraints_text:
            lines.append('')
            lines.append('[bold underline]Constraints[/bold underline]')
            lines.append(f'  [red]{_e(constraints_text)}[/red]')

        return '\n'.join(lines)


# ---------------------------------------------------------------------------
# ExecutionTraceProcessor
# What the executor does while a plan runs, and why, as sentences.
# ---------------------------------------------------------------------------
# Closer than this to its planned time, an action is on time
_ON_TIME = 0.05


def _action_name(action: str) -> str:
    # "(move r1 a b):3000" is the action planned at 3.000: its times are in the sentence
    expression, colon, ident = action.rpartition(':')
    return expression if colon and ident.isdigit() else action


def _without_ids(text: str) -> str:
    return re.sub(r'\):\d+', ')', text)


def _timing(msg: ExecutionEvent) -> str:
    late = msg.plan_time - msg.planned_time - msg.shift
    if abs(late) < _ON_TIME:
        when = '[green]on time[/green]'
    elif late > 0:
        when = f'[yellow]{late:.3f} s late[/yellow]'
    else:
        when = f'[cyan]{-late:.3f} s early[/cyan]'
    planned = f'planned at {msg.planned_time:.3f}'
    if msg.shift > 1e-3:
        planned += f', the plan {msg.shift:.3f} s later'
    return f'{when} [dim]({planned})[/dim]'


class ExecutionTraceProcessor:
    """Subscribe to ``executor/execution_trace``."""

    def __init__(self, node, callback):
        self._callback = callback
        self._sub = node.create_subscription(
            ExecutionEvent,
            'executor/execution_trace',
            self._callback,
            _TRACE_QOS,
        )

    @staticmethod
    def event2text(msg: ExecutionEvent) -> str:
        """Render an ExecutionEvent as a Rich-markup sentence."""
        stamp = f'[dim]{msg.plan_time:9.3f}[/dim]  '
        action = f'[bold]{_e(_action_name(msg.action))}[/bold]'
        detail = _e(_without_ids(msg.detail))
        kind = msg.type
        if kind == ExecutionEvent.PLAN_START:
            clock = time.strftime('%H:%M:%S', time.localtime(msg.stamp.sec))
            return f'\n[bold cyan]━━ {clock} Plan starts: {detail}[/bold cyan]'
        if kind == ExecutionEvent.PLAN_END:
            color = 'green' if msg.detail == 'succeeded' else 'bright_red'
            return f'{stamp}[bold {color}]━━ The plan ends: {detail}[/bold {color}]'
        if kind == ExecutionEvent.WAIT:
            # "its start waits for the end of ..." says it all
            if msg.detail.startswith('its '):
                return f'{stamp}{action}[dim]: {detail}[/dim]'
            return f'{stamp}{action} [dim]waits {detail}[/dim]'
        if kind == ExecutionEvent.PERFORM:
            return f'{stamp}{action} is performed {detail}'
        if kind == ExecutionEvent.START:
            return f'{stamp}{action} [green]starts[/green] {_timing(msg)}'
        if kind == ExecutionEvent.END:
            return f'{stamp}{action} [blue]ends[/blue] {_timing(msg)}'
        if kind == ExecutionEvent.SHIFT:
            return f'{stamp}{action}[magenta]: the rest of the plan goes {detail}[/magenta]'
        if kind == ExecutionEvent.FAILURE:
            who = f'{action} ' if msg.action else 'The plan '
            return f'{stamp}{who}[bold bright_red]fails: {detail}[/bold bright_red]'
        if kind == ExecutionEvent.VIOLATION:
            return f'{stamp}[bold bright_red]A constraint is violated: {detail}[/bold bright_red]'
        if kind == ExecutionEvent.REPLAN:
            replaces = 'A new plan replaces the running one'
            return f'{stamp}[bold yellow]{replaces}: {detail}[/bold yellow]'
        return f'{stamp}{action} event {kind}: {detail}'
