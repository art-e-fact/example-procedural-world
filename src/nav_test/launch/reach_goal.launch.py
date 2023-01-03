import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav_test"), "launch/simulation.launch.py"]
                )
            ]
        ),
        launch_arguments={
            # "seed": "7",
            # "use_provided_red": "True",
            # "new_background_r": TextSubstitution(text=str(colors["background_r"])),
        }.items(),
    )
    navigation = Node(
        package="nav_test",
        executable="robot",
        name="navigation",
        output="screen",
        parameters=[
            {
                "goal_x": "3.0",
                "goal_y": "0.0",
            }
        ],
    )

    return launch.LaunchDescription([simulation, navigation])
