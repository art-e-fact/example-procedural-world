from geometry_msgs.msg import TransformStamped, Point
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROBOT_TF_PREFIX = "costar_husky/"

class TraceMarkers():
    def __init__(self, node: Node):
        topic = 'debug_markers'
        self.publisher = node.create_publisher(Marker, topic, 10)
        self.reset()
    def reset(self):
        points = Marker()
        points.id = 10
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.header.frame_id = "base_link"#"costar_husky/odom"
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



class Robot(Node):
    def __init__(self):
        super().__init__("robot")

        self.trace_markers = TraceMarkers(self)
        self.trace_markers.add_point(Point(x=1.0, y=1.0, z=1.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0, ))

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.subscription_pose = self.create_subscription(
            TFMessage, f"/pose_static", self.handle_pose, 1
        )
        # self.subscription_pose_static = self.create_subscription(
        #     TFMessage, f"/pose_static", self.handle_pose_static, 1
        # )

    def handle_pose(self, t):
        self.trace_markers.publish()
        for transform in t.transforms:
            self.get_logger().info(f"TF {transform.child_frame_id} > {transform.header.frame_id}")
            if transform.header.frame_id == "demo":
                self.get_logger().info(f"TF {transform.transform}")
        # self.tf_broadcaster.sendTransform(t.transforms)

    def handle_pose_static(self, t):
        self.tf_static_broadcaster.sendTransform(t.transforms)


def main():
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
