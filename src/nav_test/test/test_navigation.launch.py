import unittest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import pytest
from launch_testing.actions import ReadyToTest
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PARAMS_FILE = '/tmp/scenario_params.yaml'

def read_params():
    try:
        with open(PARAMS_FILE, "r") as f:
            configuration = yaml.safe_load(f)
            return configuration["test"]["ros__parameters"]
    except FileExistsError as e:
        print(f"Failed to read '{PARAMS_FILE}': {e}")
        return {}

@pytest.mark.launch_test
def generate_test_description():
    rosbag_cmd = ["ros2", "bag", "record"]
    bag_recorder = ExecuteProcess(
        cmd=rosbag_cmd, output="screen", additional_env={"PYTHONUNBUFFERED": "1"}
    )
    params = read_params()
    # snapshot sky
    snapshot_sky = Node(
        package="nav_test",
        executable="snapshot_sky",
    )

    video_recorder_topdown = Node(
        package="image_view",
        executable="video_recorder",
        parameters=[
            {
                "filename": "/tmp/artefacts_output/topdown.mp4",
                "codec": "h264",
                "use_sim_time": True,
            }
        ],
        remappings=[("image", "/sky_cam")],
    )

    video_recorder_onboard = Node(
        package="image_view",
        executable="video_recorder",
        parameters=[
            {
                "filename": "/tmp/artefacts_output/onboard.mp4",
                "codec": "h264",
                "use_sim_time": True,
            }
        ],
        remappings=[("image", "/camera/image")],
    )

    reach_goal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("nav_test"), "launch"),
                "/reach_goal.launch.py",
            ]
        ),
        launch_arguments={
            "seed": str(params.get('seed', 0)),
            "max_tree_density": str(params.get('max_tree_density', 5.0)),
        }.items(),
    )

    return LaunchDescription(
        [
            reach_goal,
            bag_recorder,
            snapshot_sky,
            video_recorder_topdown,
            video_recorder_onboard,
            ReadyToTest(),
        ]
    )


class TestNavigation(unittest.TestCase):
    def test_tank_reach_target(self, proc_output):
        # This will match stdout from test_process.
        proc_output.assertWaitFor("Goal reached!", timeout=180)
