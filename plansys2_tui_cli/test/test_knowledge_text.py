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

"""The knowledge panel shows the goal, and the metric and constraints when it has them."""

import unittest

from plansys2_msgs.msg import Knowledge
from plansys2_tui_cli.controller.ros_controllers import KnowledgeProcessor


class TestKnowledgeText(unittest.TestCase):

    def test_metric_when_set(self):
        msg = Knowledge()
        msg.goal = '(and (robot_at r1 kitchen))'
        msg.metric = 'minimize (+ (spent) (total-time))'
        text = KnowledgeProcessor.knowledge2text(msg)
        self.assertIn('Goal', text)
        self.assertIn('Metric', text)
        self.assertIn('minimize (+ (spent) (total-time))', text)
        # The metric goes after the goal
        self.assertLess(text.index('Goal'), text.index('Metric'))

    def test_no_metric_section_without_metric(self):
        msg = Knowledge()
        msg.goal = '(and (robot_at r1 kitchen))'
        text = KnowledgeProcessor.knowledge2text(msg)
        self.assertIn('Goal', text)
        self.assertNotIn('Metric', text)
        self.assertNotIn('Constraints', text)

    def test_constraints_when_set(self):
        msg = Knowledge()
        msg.goal = '(and (robot_at r1 kitchen))'
        msg.metric = 'minimize (is-violated tidy)'
        msg.constraints = '(and (always (not (alarm)))(preference tidy (sometime (lit))))'
        text = KnowledgeProcessor.knowledge2text(msg)
        self.assertIn('Constraints', text)
        self.assertIn('(always (not (alarm)))', text)
        self.assertLess(text.index('Metric'), text.index('Constraints'))


if __name__ == '__main__':
    unittest.main()
