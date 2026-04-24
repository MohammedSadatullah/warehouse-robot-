#!/usr/bin/env python3
"""Explore the warehouse by visiting the nearest pending waypoint first."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


ALL_WAYPOINTS = [
    (0.0, -9.0),
    (0.0, -6.0),
    (-4.0, -9.0),
    (4.0, -9.0),
    (-7.0, -6.0),
    (-7.0, 0.0),
    (-7.0, 6.0),
    (-7.0, 11.0),
    (-6.5, 10.0),
    (-6.5, 5.0),
    (-6.5, 2.0),
    (-3.0, 11.0),
    (0.0, 11.0),
    (3.0, 11.0),
    (6.5, 10.0),
    (6.5, 5.0),
    (6.5, 2.0),
    (7.0, 6.0),
    (7.0, 0.0),
    (7.0, -6.0),
    (7.0, -11.0),
    (0.0, 0.0),
    (0.0, 5.0),
    (2.0, 2.0),
    (-2.0, 2.0),
]

WAYPOINT_TOLERANCE = 0.6
LINEAR_SPEED = 0.4
ANGULAR_SPEED = 0.8
OBSTACLE_DISTANCE = 0.45


class WarehouseExplorer(Node):
    """Simple reactive explorer for quick map coverage tests."""

    def __init__(self) -> None:
        super().__init__("warehouse_explorer")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.x = None
        self.y = None
        self.yaw = 0.0
        self.min_front_dist = float("inf")
        self.obstacle_ahead = False
        self.waypoints = []
        self.wp_idx = 0
        self.ready = False
        self.control_timer = None
        self.get_logger().info("Waiting for odometry...")
        self.init_timer = self.create_timer(0.5, self.wait_for_odom)

    def wait_for_odom(self) -> None:
        if self.x is None:
            return

        self.init_timer.cancel()
        self.get_logger().info(f"Start position: ({self.x:.2f}, {self.y:.2f})")
        self.waypoints = sorted(
            ALL_WAYPOINTS,
            key=lambda waypoint: math.hypot(waypoint[0] - self.x, waypoint[1] - self.y),
        )
        self.get_logger().info(f"First waypoint: {self.waypoints[0]}")
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")
        self.ready = True
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def scan_cb(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return

        cone = 15
        front_indices = list(range(0, cone)) + list(range(len(msg.ranges) - cone, len(msg.ranges)))
        valid_ranges = [
            msg.ranges[index]
            for index in front_indices
            if msg.range_min < msg.ranges[index] < msg.range_max
        ]
        self.min_front_dist = min(valid_ranges) if valid_ranges else float("inf")
        self.obstacle_ahead = self.min_front_dist < OBSTACLE_DISTANCE

    def odom_cb(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def distance_to(self, waypoint) -> float:
        return math.hypot(waypoint[0] - self.x, waypoint[1] - self.y)

    def angle_to(self, waypoint) -> float:
        return math.atan2(waypoint[1] - self.y, waypoint[0] - self.x)

    @staticmethod
    def angle_diff(target: float, current: float) -> float:
        delta = target - current
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        return delta

    def publish_vel(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        self.publish_vel(0.0, 0.0)

    def control_loop(self) -> None:
        if not self.ready or self.x is None:
            return

        if self.wp_idx >= len(self.waypoints):
            self.stop()
            self.get_logger().info("Exploration complete!")
            self.get_logger().info(
                "Save map: ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map"
            )
            self.control_timer.cancel()
            return

        waypoint = self.waypoints[self.wp_idx]
        distance = self.distance_to(waypoint)

        if distance < WAYPOINT_TOLERANCE:
            self.stop()
            self.get_logger().info(
                f"[{self.wp_idx + 1}/{len(self.waypoints)}] "
                f"Reached ({waypoint[0]:.1f}, {waypoint[1]:.1f})"
            )
            self.wp_idx += 1
            return

        if self.obstacle_ahead:
            self.get_logger().warn(f"Obstacle {self.min_front_dist:.2f} m - avoiding")
            self.publish_vel(0.0, ANGULAR_SPEED * 1.5)
            return

        yaw_error = self.angle_diff(self.angle_to(waypoint), self.yaw)
        if abs(yaw_error) > 0.3:
            self.publish_vel(0.0, ANGULAR_SPEED if yaw_error > 0 else -ANGULAR_SPEED)
            return

        correction = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, yaw_error * 1.2))
        speed = LINEAR_SPEED if distance > 1.0 else LINEAR_SPEED * 0.5
        self.publish_vel(speed, correction)


def main() -> None:
    rclpy.init()
    node = WarehouseExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
