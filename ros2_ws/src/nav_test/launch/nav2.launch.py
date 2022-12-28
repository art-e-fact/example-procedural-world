import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="nav_test").find(
        "nav_test"
    )
    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    world_path = os.path.join(pkg_share, "world/my_world.sdf")

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

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/demo/model/costar_husky/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/model/costar_husky/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/costar_husky/pose_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/costar_husky/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/costar_husky/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            (
                "/world/demo/model/costar_husky/link/base_link/sensor/front_laser/scan",
                "/scan",
            ),
            ("/model/costar_husky/pose", "/pose"),
            ("/model/costar_husky/pose_static", "/pose_static"),
            ("/model/costar_husky/tf", "/pose_odom"),
            ("/model/costar_husky/odometry", "/odom"),
        ],
        output="screen",
    )

    lidar_stf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # namespace = namespace,
        name="lidar_stf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "front_laser"],
    )
    imu_stf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # namespace = namespace,
        name="imu_stf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu"],
    )
    odom_broadcaster = Node(
        package="nav_test",
        executable="pose_broadcaster",
        name="odom_broadcaster",
        output="screen",
    )

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[
    #         {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
    #     ],
    # )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    # )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    # spawn_entity = launch_ros.actions.Node(
    # 	package='gazebo_ros',
    # 	executable='spawn_entity.py',
    #     arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
    #     output='screen'
    # )
    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return launch.LaunchDescription(
        [
            # launch.actions.DeclareLaunchArgument(
            #     name="gui",
            #     default_value="True",
            #     description="Flag to enable joint_state_publisher_gui",
            # ),
            # launch.actions.DeclareLaunchArgument(
            #     name="model",
            #     default_value=default_model_path,
            #     description="Absolute path to robot urdf file",
            # ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            # joint_state_publisher_node,
            # robot_state_publisher_node,
            # spawn_entity,
            # robot_localization_node,
            rviz_node,
            gazebo,
            bridge,
            lidar_stf,
            imu_stf,
            odom_broadcaster,
        ]
    )
