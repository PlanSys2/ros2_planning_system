"""Horizontal Bar Chart"""

import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import re

import rclpy
from rclpy.node import Node


class HorizontalBarChart(Node):
    """Horizontal Bar Chart"""

    def __init__(self):
        """Horizontal Bar Chart"""
        super().__init__("horizontal_bar_chart")
        self.declare_parameter("plan_file")
        plan_data = self.get_plan_from_file()
        
        if 'start_times' not in plan_data:
            return

        filename = os.path.basename(self.get_parameter("plan_file").value)
        filename_without_ext = os.path.splitext(filename)[0]

        fig, ax = plt.subplots(1, figsize=(9.5, 4))
        ax.invert_yaxis()
        ax.barh(plan_data['actions'], plan_data['durations'], left=plan_data['start_times'])
        plt.title(filename_without_ext)
        plt.savefig(filename_without_ext + '.pdf', format="pdf", bbox_inches="tight")
        plt.show()

    def get_plan_from_file(self):
        """Get plan from file."""
        ret = dict()
        lines = []
        with open(
            self.get_parameter("plan_file").value, 'r'
        ) as fid:
            try:
                lines = fid.readlines()
            except (yaml.YAMLError, yaml.scanner.ScannerError) as ex:
                self.get_logger().error(f"{fid} is not a valid YAML file: {ex}")
                return ret
        ret['start_times'] = []
        ret['actions'] = []
        ret['durations'] = []
        for line in lines:
            m = re.match(r'.*?'
                         r'(?P<start_time>[+]?([0-9]+([.][0-9]*)?|[.][0-9]+))'
                         r':\s*?\('
                         r'(?P<action>.*?)'
                         r'\)\s*?\['
                         r'(?P<duration>[+]?([0-9]+([.][0-9]*)?|[.][0-9]+))', line)
            if m:
                ret['start_times'].append(float(m.group('start_time')))
                ret['actions'].append(m.group('action'))
                ret['durations'].append(float(m.group('duration')))
        return ret


def main(args=None):
    """Run node"""
    rclpy.init(args=args)
    horizontal_bar_chart = HorizontalBarChart()
    try:
        rclpy.spin(horizontal_bar_chart)
    except KeyboardInterrupt:
        pass
    finally:
        horizontal_bar_chart.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
