from geometry_msgs.msg import Twist, Point, TransformStamped, Pose
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan

import numpy as np

import rclpy
import rclpy.time
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

ROBOT_TF_PREFIX = "costar_husky/"


class Robot(Node):
    def __init__(self):
        super().__init__("robot")

        self.world_frame = (
            self.declare_parameter("world_frame", "test_world")
            .get_parameter_value()
            .string_value
        )
        self.robot_frame = (
            self.declare_parameter("robot_frame", "costar_husky/base_link")
            .get_parameter_value()
            .string_value
        )
        self.lidar_frame = (
            self.declare_parameter("lidar_frame", "costar_husky/base_link/front_laser")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trace_markers = TraceMarkers(self)
        self.trace_markers.add_point(
            Point(x=1.0, y=1.0, z=1.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        )

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # self.subscription_pose = self.create_subscription(
        #     TFMessage, f"/pose_odom", self.handle_pose, 1
        # )
        self.subscription_pose_static = self.create_subscription(
            TFMessage, f"/pose_static", self.handle_pose_static, 1
        )
        self.subscription_pose_static = self.create_subscription(
            LaserScan, f"/scan", self.handle_scan, 1
        )
        self.publisher_cmd_vel = self.create_publisher(Twist, "cmd_vel", 1)

        # Calculate and send new navigation commands periodically
        self.timer = self.create_timer(1.0, self.navigate)

    def handle_pose(self, t):
        self.tf_broadcaster.sendTransform(t.transforms)

    def handle_pose_static(self, t):
        self.tf_static_broadcaster.sendTransform(t.transforms)

    def handle_scan(self, msg: LaserScan):
        self.trace_markers.reset()

        try:
            sensor_pose = self.tf_buffer.lookup_transform(
                self.world_frame, self.lidar_frame, rclpy.time.Time()
            )
            print(sensor_pose)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform "{self.world_frame}" to "{self.lidar_frame}": {ex}'
            )
            return

        for i, angle in enumerate(
            np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ):
            range = msg.ranges[i]
            if np.isinf(range) or np.isnan(range):
                continue
            x = np.cos(angle) * range
            y = np.sin(angle) * range
            p = do_transform_pose(
                Pose(position=Point(x=x, y=y)),
                TransformStamped(transform=sensor_pose.transform),
            ).position
            color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            self.trace_markers.add_point(p, color)

        self.trace_markers.publish()

    def navigate(self):
        pass


# TODO integrate into the robot class
class TraceMarkers:
    def __init__(self, node: Node):
        topic = "debug_markers"
        self.publisher = node.create_publisher(Marker, topic, 10)
        self.reset()

    def reset(self):
        points = Marker()
        points.id = 10
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.header.frame_id = "test_world"  # TODO read from robot
        points.scale.x = 0.05
        points.scale.y = 0.05
        points.color.a = 1.0
        points.color.r = 1.0
        points.color.g = 0.0
        points.color.b = 0.0
        points.pose.orientation.w = 1.0
        self.marker = points

    def add_point(self, point, color):
        self.marker.points.append(point)
        self.marker.colors.append(color)

    def publish(self):
        self.publisher.publish(self.marker)


def main():
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
