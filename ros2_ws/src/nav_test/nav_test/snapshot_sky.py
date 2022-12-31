import os
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.duration import Duration
from .wait_for_message import wait_for_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

bridge = CvBridge()

class SnapshotSky(Node):
    def __init__(self):
        super().__init__('snapshot_sky')
        self.declare_parameter('out_dir', '/tmp/artefacts_output')


def main(args=None):
    clock = Clock()
    rclpy.init(args=args)

    stop_test = SnapshotSky()

    stop_test.get_logger().info("Waiting...")
    time.sleep(30)
    stop_test.get_logger().info("Waiting for image...")
    (ok, image_msg) = wait_for_message(Image, stop_test, '/sky_cam')
    out_dir = stop_test.get_parameter('out_dir').get_parameter_value().string_value
    if ok:
        stop_test.get_logger().info("Got image")
        try:
            cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            stop_test.get_logger().error(e)
        else:
            filepath = os.path.join(out_dir, 'camera_image.jpeg')
            cv2.imwrite(filepath, cv2_img)
            stop_test.get_logger().info(f"Saved: {filepath}")
        
    else:
        stop_test.get_logger().info("Failed to receive image")

    stop_test.get_logger().info("Destroying node...")
    stop_test.destroy_node()
    stop_test.get_logger().info("Shutting down...")
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()