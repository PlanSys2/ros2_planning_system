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

"""``ros2 plansys2 knowledge`` verb."""

import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension

from .verb_utils import bbcode_to_ansi
from ..controller.ros_controllers import KnowledgeProcessor


class KnowledgeVerb(VerbExtension):
    """Print the current PlanSys2 problem-expert knowledge."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--duration', type=float, default=5000.0,
            help='Seconds to run (default: 5000 = run until Ctrl+C)',
        )
        parser.add_argument(
            '--once', action='store_true',
            help='Print once and exit',
        )

    def _callback(self, msg):
        raw = KnowledgeProcessor.knowledge2text(msg)
        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        text = bbcode_to_ansi(raw, enable_color)

        sys.stdout.write('\033[1;1H\033[J')
        sys.stdout.write('PlanSys2 Knowledge:\n\n')
        sys.stdout.write(text)
        if enable_color:
            sys.stdout.write('\033[0m')
        sys.stdout.write('\n')
        sys.stdout.flush()

        if getattr(self, '_once', False):
            self._received = True

    def main(self, *, args):
        self._once = getattr(args, 'once', False)
        self._received = False

        with NodeStrategy(args) as node:
            try:
                self._sub = KnowledgeProcessor(node, self._callback)

                t_end = time.time() + args.duration
                while time.time() < t_end:
                    if self._once and self._received:
                        break
                    rclpy.spin_once(node, timeout_sec=0.1)

            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                sys.stdout.write('\n')
                sys.stdout.flush()
        return 0
