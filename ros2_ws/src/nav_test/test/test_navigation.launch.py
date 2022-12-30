import unittest
import sys
import os

import ament_index_python
from ament_index_python.packages import get_package_share_directory

import pytest
import launch_testing
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    TEST_PROC_PATH = os.path.join(
        ament_index_python.get_package_prefix('nav_test'),
        'nav_test',
        'robot.py'
    )

    rosbag_cmd = ['ros2', 'bag', 'record']
    bag_recorder = ExecuteProcess(
        cmd=rosbag_cmd,
        output='screen', additional_env={'PYTHONUNBUFFERED': '1'})

    reach_goal = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav_test'), 'launch'),
         '/reach_goal.launch.py'])
      )

    return LaunchDescription([
        reach_goal,
        bag_recorder,
        launch_testing.actions.ReadyToTest()
    ])


class TestTurtle(unittest.TestCase):
    def test_tank_reach_target(self, proc_output):
        # This will match stdout from test_process.
        proc_output.assertWaitFor("Goal reached!", timeout=50)
