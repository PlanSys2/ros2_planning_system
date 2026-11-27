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


from plansys2_msgs.msg import ActionExecutionInfo, Plan, PlanItem, PlanTask
from plansys2_tui_cli.controller.ros_controllers import PlanMonitorProcessor
from rich.text import Text


def plain(markup):
    return Text.from_markup(markup).plain


def item(time, action):
    i = PlanItem()
    i.time = time
    i.action = action
    i.duration = 5.0
    return i


def task(id_, name, method, subtasks):
    t = PlanTask()
    t.id = id_
    t.task = name
    t.method = method
    t.subtasks = subtasks
    return t


def executed(action, time, status, completion):
    info = ActionExecutionInfo()
    info.action_full_name = f'{action}:{int(time * 1000)}'
    info.status = status
    info.completion = completion
    return info


def test_a_hierarchical_plan_is_shown_as_its_decomposition():
    plan = Plan()
    plan.items = [item(0.0, '(transport r2d2 wheel_1 wheels_zone assembly_zone)'),
                  item(0.0, '(transport c3po body_1 body_zone assembly_zone)'),
                  item(5.001, '(assemble r2d2 assembly_zone wheel_1 body_1 car_1)')]
    plan.tasks = [task('root', '', '', ['1']),
                  task('1', '(build car_1)', 'build_car', ['2', '3', '4', 'item:2']),
                  task('2', '(fetch wheel_1 assembly_zone)', 'fetch_carry', ['item:0']),
                  task('3', '(fetch body_1 assembly_zone)', 'fetch_carry', ['item:1']),
                  task('4', '(ready r2d2 assembly_zone)', 'ready_here', [])]
    info = {}
    for i in (executed(plan.items[0].action, 0.0, ActionExecutionInfo.SUCCEEDED, 1.0),
              executed(plan.items[1].action, 0.0, ActionExecutionInfo.EXECUTING, 0.5)):
        info[i.action_full_name] = i
    lines = plain(PlanMonitorProcessor.plan2text(plan, info)).splitlines()
    assert len(lines) == 7, lines
    # The task, running: one fetch done, the other on its way
    assert lines[0].startswith('RUNNING') and lines[0].endswith('(build car_1) ← build_car')
    assert lines[1].startswith('DONE') and lines[1].endswith(
        '├─ (fetch wheel_1 assembly_zone) ← fetch_carry')
    assert lines[2].startswith('SUCCEEDED') and lines[2].endswith(
        '│  └─ (transport r2d2 wheel_1 wheels_zone assembly_zone)')
    assert lines[3].startswith('RUNNING')
    assert lines[4].startswith('EXECUTING') and '50%' in lines[4]
    assert lines[5].startswith('-') and lines[5].endswith(
        '├─ (ready r2d2 assembly_zone) ← ready_here  (nothing to do)')
    assert lines[6].startswith('PENDING') and lines[6].endswith(
        '└─ (assemble r2d2 assembly_zone wheel_1 body_1 car_1)')


def test_a_pddl_plan_is_a_list_of_actions():
    plan = Plan()
    plan.items = [item(0.0, '(move r1 a b)'), item(5.001, '(move r1 b c)')]
    lines = plain(PlanMonitorProcessor.plan2text(plan, {})).splitlines()
    assert len(lines) == 2 and lines[0].endswith('(move r1 a b)')
