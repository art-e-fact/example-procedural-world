from geometry_msgs.msg import Twist, Point, TransformStamped, Pose, PoseStamped
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path

import numpy as np
import cv2

import rclpy
import rclpy.time
from rclpy import qos
from rclpy.node import Node

import tf_transformations
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
        # Set the default map resolution to 10cm
        self.map_resolution = float(
            self.declare_parameter("map_resolution", "0.1")
            .get_parameter_value()
            .string_value
        )
        goal_x = float(
            self.declare_parameter("goal_x", "3.0").get_parameter_value().string_value
        )
        goal_y = float(
            self.declare_parameter("goal_y", "0.0").get_parameter_value().string_value
        )
        # TODO copy name from navigation2
        self.max_goal_distance = float(
            self.declare_parameter("max_goal_distance", "0.5")
            .get_parameter_value()
            .string_value
        )
        self.goal = PoseStamped(pose=Pose(position=Point(x=goal_x, y=goal_y)))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.subscription_pose_static = self.create_subscription(
            TFMessage, f"/pose_static", self.handle_pose_static, 10
        )
        self.subscription_pose_static = self.create_subscription(
            LaserScan, f"/scan", self.handle_scan, 10
        )
        self.publisher_cmd_vel = self.create_publisher(Twist, "cmd_vel", 1)
        self.publisher_map = self.create_publisher(OccupancyGrid, "map", 1)
        self.publisher_score_map = self.create_publisher(OccupancyGrid, "score_map", 1)
        self.publisher_path = self.create_publisher(Path, "path", 1)
        self.publisher_goal = self.create_publisher(PoseStamped, "goal", 1)
        self.publisher_local_goal = self.create_publisher(PoseStamped, "local_goal", 1)

        # Calculate and send new navigation commands periodically
        self.timer = self.create_timer(0.1, self.navigate)

        # Save map periodically
        self.timer = self.create_timer(2.0, self.save_map_to_image)

        self.map = np.zeros((256, 256), dtype=np.int8)
        self.map_origin = Point(
            x=-self.map.shape[0] / 2.0 * self.map_resolution,
            y=-self.map.shape[1] / 2.0 * self.map_resolution,
        )
        self.path = Path()

    def handle_pose(self, t):
        self.tf_broadcaster.sendTransform(t.transforms)

    def handle_pose_static(self, t):
        self.tf_static_broadcaster.sendTransform(t.transforms)

    def world_pose_to_map(self, pose):
        return (
            round((pose.x - self.map_origin.x) / self.map_resolution),
            round((pose.y - self.map_origin.y) / self.map_resolution),
        )

    def handle_scan(self, msg: LaserScan):
        map_free = np.zeros(self.map.shape, dtype=np.int8)
        map_wall = np.zeros(self.map.shape, dtype=np.int8)

        try:
            sensor_pose = self.tf_buffer.lookup_transform(
                self.world_frame, self.lidar_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform "{self.world_frame}" to "{self.lidar_frame}": {ex}'
            )
            return

        robot_map_pose = self.world_pose_to_map(sensor_pose.transform.translation)

        for i, angle in enumerate(
            np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ):
            range = msg.ranges[i]
            if np.isinf(range) or np.isnan(range):
                range = msg.range_max

            x = np.cos(angle) * range
            y = np.sin(angle) * range
            laser_hit_pose = do_transform_pose(
                Pose(position=Point(x=x, y=y)),
                TransformStamped(transform=sensor_pose.transform),
            ).position
            lidar_hit_map_pose = self.world_pose_to_map(laser_hit_pose)
            cv2.line(map_free, robot_map_pose, lidar_hit_map_pose, 1)
            if range < msg.range_max:
                cv2.circle(map_wall, lidar_hit_map_pose, radius=1, color=1, thickness=5)
            self.map[(map_free == 1) & (map_wall == 0) & (self.map < 120)] += 1
            self.map[(map_wall == 1) & (self.map > -110)] -= -12

        self.publish_map()
        self.find_path(robot_map_pose, self.world_pose_to_map(self.goal.pose.position))
        self.navigate()
        self.check_goal_reached()

    def create_occupancy_grid_msg(self, map):
        grid = OccupancyGrid()
        grid.data = map.ravel().data
        grid.header.frame_id = self.world_frame
        grid.info = MapMetaData()
        grid.info.height = map.shape[0]
        grid.info.width = map.shape[1]
        grid.info.origin.position = self.map_origin
        grid.info.resolution = self.map_resolution
        return grid

    def publish_map(self):
        grid = self.create_occupancy_grid_msg(self.map)
        self.publisher_map.publish(grid)

    def save_map_to_image(self):
        map_img = np.full((*self.map.shape, 3), 0, np.uint8)
        # save free nodes into the green channel
        map_img[:, :, 1] = np.maximum(self.map, 0).view(np.uint8) * 2
        # save occupied nodes into the red channel
        map_img[:, :, 2] = (np.maximum(np.minimum(self.map, 0), -127) * -1).view(
            np.uint8
        ) * 2
        cv2.imwrite("/tmp/artefacts_output/map.png", map_img)

    # minimal a-start path finder
    def find_path(self, start: tuple[int, int], end: tuple[int, int]):
        # flag the computer nodes
        discovered = np.full(self.map.shape, 0.0)
        # position of the best parent node
        parent_map = np.full((*self.map.shape, 2), -1)
        # the g-cost and h-cost of each node
        score_map = np.full((*self.map.shape, 2), np.inf)

        # discover one node, and update its neighbours
        def score(xy):
            x, y = xy
            ways = [
                (-1, 0),
                (0, -1),
                (1, 0),
                (0, 1),
                (-1, -1),
                (1, -1),
                (1, 1),
                (-1, 1),
            ]
            step_costs = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

            parent_g_cost = score_map[x, y][0]

            for step_cost, (sx, sy) in zip(step_costs, ways):
                pos = np.array([x + sx, y + sy])
                # skip out of map nodes
                if (
                    pos[0] < 0
                    or pos[1] < 0
                    or pos[0] >= self.map.shape[0]
                    or pos[1] >= self.map.shape[1]
                ):
                    continue
                # skip descovered nodes and obstacles
                if discovered[pos[0], pos[1]]:
                    continue
                # self.map is in row-major (y, x indexed)
                if self.map[pos[1], pos[0]] < 0:
                    danger_cost = 1000.0
                else:
                    danger_cost = 0.0
                dist_cost = np.linalg.norm(pos - end)
                g_cost = parent_g_cost + step_cost
                h_cost =  danger_cost + dist_cost
                if g_cost + h_cost < score_map[pos[0], pos[1]].sum():
                    score_map[pos[0], pos[1]] = [g_cost, h_cost]
                    parent_map[pos[0], pos[1]] = [x, y]
                    if parent_map[x, y][0] == pos[0] and parent_map[x, y][1] == pos[1]:
                        raise Exception(f"Circular parenting {[x,y]}<>{pos}")

            discovered[x, y] = True

        # discover the first node (current pose of the robot)
        score_map[start[0], start[1], :] = 0
        score(start)

        # keep discovering nodes until the end-node is reached
        while not np.all(discovered):
            # select an undiscovered node with the lowest score
            next = np.unravel_index(
                (score_map.sum(axis=2) + discovered * 999999999).argmin(),
                (score_map.shape[0], score_map.shape[1]),
            )
            # found a path to the goal
            if next[0] == end[0] and next[1] == end[1]:
                break
            score(next)

        # build the path by walking backwards from the end node
        path = [end]
        fuse = 1000
        while True:
            fuse -= 1
            if fuse < 0:
                self.get_logger().error(f"Failed to build path")
                return

            prev = path[0]
            self.get_logger().info(f"Score trail {prev}: {score_map[prev[0], prev[1]]}")
            parent = parent_map[prev[0], prev[1]]
            path.insert(0, parent)

            if parent[0] == start[0] and parent[1] == start[1]:
                break

        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.path.poses = [
            PoseStamped(
                header=Header(frame_id=self.world_frame),
                pose=Pose(
                    position=Point(
                        x=pos[0] * self.map_resolution + self.map_origin.x,
                        y=pos[1] * self.map_resolution + self.map_origin.y,
                    )
                ),
            )
            for pos in path
        ]

        # Publish the score map for debugging
        score_map = np.transpose(score_map.sum(axis=2))
        min_score = score_map.min()
        max_score = score_map[score_map != np.inf].max()
        int_scores = (score_map - min_score) / (max_score - min_score) * -255 + 127
        for xy in path:
            int_scores[xy[0], xy[1]] = 0
        grid = self.create_occupancy_grid_msg(int_scores.astype(np.int8))
        self.publisher_score_map.publish(grid)

        self.publisher_path.publish(self.path)

    def lookup_transform(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform "{target_frame}" to "{source_frame}": {ex}'
            )
            return None

    def navigate(self):
        robot_pose = self.lookup_transform(self.world_frame, self.robot_frame)
        if robot_pose == None:
            return

        robot_xy = np.array(
            [
                robot_pose.transform.translation.x,
                robot_pose.transform.translation.y,
            ]
        )

        next_pose = None
        min_pose_distance = 0.5
        for pose in self.path.poses:
            dist = np.linalg.norm(
                robot_xy - np.array([pose.pose.position.x, pose.pose.position.y])
            )
            if dist >= min_pose_distance:
                next_pose = pose
                break

        if next_pose is None and len(self.path.poses) > 0:
            next_pose = self.path.poses[-1]

        if next_pose:
            self.publisher_local_goal.publish(next_pose)
            self.publisher_goal.publish(self.goal)

            diff_xy = np.array([pose.pose.position.x, pose.pose.position.y]) - robot_xy
            angle = np.arctan2(diff_xy[1], diff_xy[0])

            q_target = tf_transformations.quaternion_from_euler(0, 0, angle)
            q1_inv = [0.0, 0.0, 0.0, 1.0]
            q1_inv[0] = robot_pose.transform.rotation.x
            q1_inv[1] = robot_pose.transform.rotation.y
            q1_inv[2] = robot_pose.transform.rotation.z
            q1_inv[3] = -robot_pose.transform.rotation.w  # Negate for inverse
            q_diff = tf_transformations.quaternion_multiply(q_target, q1_inv)
            angle = tf_transformations.euler_from_quaternion(q_diff)[2]

            # TODO move speeds into args
            cmd_msg = Twist()
            if np.abs(angle) > 0.4:
                cmd_msg.angular.z = angle * 20
            else:
                cmd_msg.angular.z = angle
                cmd_msg.linear.x = 10.0 - np.abs(angle) * 8
            self.publisher_cmd_vel.publish(cmd_msg)

    def check_goal_reached(self):
        robot_pose = self.lookup_transform(self.world_frame, self.robot_frame)
        if robot_pose == None or len(self.path.poses) == 0:
            return

        distance = np.linalg.norm(
            np.array(
                [robot_pose.transform.translation.x, robot_pose.transform.translation.y]
            )
            - np.array([self.goal.pose.position.x, self.goal.pose.position.y])
        )

        if distance <= self.max_goal_distance:
            self.get_logger().info("Goal reached!")
        else:
            self.get_logger().info(f"Goal is {distance}m away")


def main():
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
