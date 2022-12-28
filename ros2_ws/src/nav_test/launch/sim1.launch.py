import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess  # , IncludeLaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    models_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../models")
    world_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "../worlds/scene1.sdf"
    )
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v 4", "-r", world_path],
        additional_env={
            "IGN_GAZEBO_MODEL_PATH": models_path,
            "IGN_GAZEBO_RESOURCE_PATH": models_path,
        },
        output="screen",
    )

    # pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')
    # teleop = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py'))
    # )

    # keyboard_teleop = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     output='screen'
    # )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/demo/model/costar_husky_sensor_config_1/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/model/costar_husky_sensor_config_1/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/model/costar_husky_sensor_config_1/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/model/costar_husky_sensor_config_1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
        ],
        remappings=[
            (
                "/world/demo/model/costar_husky_sensor_config_1/link/base_link/sensor/front_laser/scan",
                "/front_laser/scan",
            ),
            # ("/model/costar_husky_sensor_config_1/pose", "/tf"),
            ("/model/costar_husky_sensor_config_1/pose_static", "/tf"),
            ("/model/costar_husky_sensor_config_1/odometry", "odometry"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            # teleop,
            bridge,
        ]
    )
