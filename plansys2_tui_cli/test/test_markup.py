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

"""Text from ROS is shown as it is, never read as markup."""

from plansys2_msgs.msg import ActionExecution, ActionPerformerStatus, Knowledge, Plan, PlanItem
from plansys2_tui_cli.cli.verb_utils import bbcode_to_ansi
from plansys2_tui_cli.controller.ros_controllers import (
    ActionExecutionProcessor,
    KnowledgeProcessor,
    PerformersProcessor,
    PlanMonitorProcessor,
)
from rich.text import Text

TRICKY = ['[/x]', '[move]', '[bold]r1', 'a[b]c', '[', ']', '\\[x]']


def plain(markup):
    # Raises MarkupError on a closing tag that nothing opened
    return Text.from_markup(markup).plain


def test_performers_and_actions():
    for text in TRICKY:
        perf = ActionPerformerStatus()
        perf.node_name = f'node{text}'
        perf.action = f'move{text}'
        perf.specialized_arguments = [text]
        perf.state = ActionPerformerStatus.RUNNING
        shown = plain(PerformersProcessor.performers2text({perf.node_name: perf}))
        assert f'node{text}' in shown and f'move{text} {text}' in shown

        msg = ActionExecution()
        msg.type = ActionExecution.FEEDBACK
        msg.action = 'move'
        msg.arguments = [text]
        msg.status = f'status {text}'
        msg.completion = 0.5
        shown = plain(ActionExecutionProcessor.combined2text(
            {perf.node_name: perf}, {perf.node_name: msg}))
        assert f'status {text}' in shown and f'move {text}' in shown


def test_plan_and_knowledge():
    for text in TRICKY:
        plan = Plan()
        item = PlanItem()
        item.action = f'(move r1 {text})'
        plan.items.append(item)
        assert f'(move r1 {text})' in plain(PlanMonitorProcessor.plan2text(plan, {}))

        knowledge = Knowledge()
        knowledge.instances = [f'r{text}']
        knowledge.predicates = [f'(p {text})']
        knowledge.functions = [f'(f {text})']
        knowledge.goal = f'(and (p {text}))'
        knowledge.metric = f'minimize (f {text})'
        shown = plain(KnowledgeProcessor.knowledge2text(knowledge))
        for expected in knowledge.instances + knowledge.predicates + knowledge.functions + [
                knowledge.goal, knowledge.metric]:
            assert expected in shown


def test_cli_keeps_escaped_text():
    knowledge = Knowledge()
    knowledge.predicates = ['(at [move] r1)']
    markup = KnowledgeProcessor.knowledge2text(knowledge)
    for color in (True, False):
        out = bbcode_to_ansi(markup, color)
        assert '(at [move] r1)' in out
        assert '[bold underline]' not in out and '[green]' not in out
    assert '\033[32m' in bbcode_to_ansi(markup, True)
