import launch
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package="nav_test").find("nav_test")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/navigation.rviz")
    world_path = os.path.join(pkg_share, "world/my_world.sdf")

    models_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../models")
    world_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "../worlds/scene1.sdf"
    )

    pkg_share = FindPackageShare(package="nav_test").find("nav_test")

    generate_field_model = ExecuteProcess(
        cmd=[
            FindExecutable(name="blender"),
            "-b",
            os.path.join(pkg_share, "blender/navigation.blend"),
            "--python",
            os.path.join(pkg_share, "blender/export_model.py"),
            "--",
            launch.substitutions.LaunchConfiguration("seed"),
            launch.substitutions.LaunchConfiguration("max_tree_density"),
        ],
        output="screen",
    )

    # delay starting gazebo to make sure the model generation is done
    gazebo = ExecuteProcess(
        cmd=[FindExecutable(name="ign"), "gazebo", "-v 4",  "-r", world_path],
        additional_env={
            "IGN_GAZEBO_MODEL_PATH": models_path,
            "IGN_GAZEBO_RESOURCE_PATH": models_path,
        },
        output="screen",
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/model/costar_husky/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/costar_husky/pose_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/costar_husky/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/costar_husky/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
        ],
        remappings=[
            ("/lidar", "/scan"),
            ("/model/costar_husky/pose", "/pose"),
            ("/model/costar_husky/pose_static", "/pose_static"),
            ("/model/costar_husky/tf", "/pose_odom"),
            ("/model/costar_husky/odometry", "/odom"),
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="seed",
                default_value="1",
                description="Random seed for generating procedural assets",
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_tree_density",
                default_value="5.0",
                description="Sets the density of the trees in the generated model",
            ),
            generate_field_model,
            # Wait until the assets are generated
            RegisterEventHandler(
                OnProcessExit(
                    target_action=generate_field_model,
                    on_exit=[
                        LogInfo(msg="Model ganaration is done"),
                        gazebo,
                        bridge,
                        rviz_node,
                    ],
                )
            ),
        ]
    )
