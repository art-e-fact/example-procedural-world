# based on https://github.com/osrf/subt/blob/master/subt_ros/src/pose_tf_broadcaster.cpp
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
            TFMessage, f"/pose_odom", self.handle_pose, 1
        )
        self.subscription_pose_static = self.create_subscription(
            TFMessage, f"/pose_static", self.handle_pose_static, 1
        )

    # def handle_pose(self, t):
    #     for transform in t.transforms:
    #         transform.child_frame_id = transform.child_frame_id.removeprefix(ROBOT_TF_PREFIX)
    #         transform.header.frame_id = transform.header.frame_id.removeprefix(ROBOT_TF_PREFIX)
    #         self.get_logger().info(f"TF {transform.child_frame_id} < {transform.header.frame_id}")
    #     self.tf_broadcaster.sendTransform(t.transforms)

    def handle_pose(self, t):
        self.tf_broadcaster.sendTransform(t.transforms)

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
