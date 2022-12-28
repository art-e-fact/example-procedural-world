from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROBOT_TF_PREFIX = "costar_husky/"

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__("pose_broadcaster")

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
        for transform in t.transforms:
            self.get_logger().info(f"TF {transform.child_frame_id} > {transform.header.frame_id}")
            if transform.header.frame_id == "demo":
                self.get_logger().info(f"TF {transform.transform}")
        # self.tf_broadcaster.sendTransform(t.transforms)

    def handle_pose_static(self, t):
        self.tf_static_broadcaster.sendTransform(t.transforms)


def main():
    rclpy.init()
    node = PoseBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
