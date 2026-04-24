#!/usr/bin/env python3
"""Drive the robot through a fixed warehouse coverage path using odometry."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


FWD_FAST = 0.18
FWD_AISLE = 0.13
TURN_SPEED = 0.6

XY_TOLERANCE = 0.25
YAW_TOLERANCE = 0.05


# Format: (x, y, speed_override, label)
WAYPOINTS = [
    (8.8, -10.5, FWD_FAST, "south wall east end"),
    (-8.8, -10.5, FWD_FAST, "south wall west end"),
    (0.0, -10.5, FWD_FAST, "south wall center return"),
    (-8.8, -10.5, FWD_FAST, "west wall south start"),
    (-8.8, 10.8, FWD_FAST, "west wall north end"),
    (-8.8, -10.5, FWD_FAST, "west wall south return"),
    (-8.8, 10.8, FWD_FAST, "north wall west start"),
    (8.8, 10.8, FWD_FAST, "north wall east end"),
    (-8.8, 10.8, FWD_FAST, "north wall west return"),
    (8.8, 10.8, FWD_FAST, "east wall north start"),
    (8.8, -10.5, FWD_FAST, "east wall south end"),
    (0.0, -10.5, FWD_FAST, "return center south"),
    (-8.5, -10.0, FWD_FAST, "west outer aisle south"),
    (-8.5, 10.5, FWD_AISLE, "west outer aisle north"),
    (-8.5, -10.0, FWD_AISLE, "west outer aisle south return"),
    (-6.75, -9.5, FWD_FAST, "west inner aisle entry"),
    (-6.75, 10.5, FWD_AISLE, "west inner aisle north"),
    (-6.75, -9.5, FWD_AISLE, "west inner aisle south return"),
    (0.0, -10.0, FWD_FAST, "center aisle south"),
    (0.0, 10.8, FWD_FAST, "center aisle north"),
    (0.0, -10.0, FWD_FAST, "center aisle south return"),
    (6.75, -9.5, FWD_FAST, "east inner aisle entry"),
    (6.75, 10.5, FWD_AISLE, "east inner aisle north"),
    (6.75, -9.5, FWD_AISLE, "east inner aisle south return"),
    (8.5, -10.0, FWD_FAST, "east outer aisle south"),
    (8.5, 10.5, FWD_AISLE, "east outer aisle north"),
    (8.5, -10.0, FWD_AISLE, "east outer aisle south return"),
    (-5.0, -7.0, FWD_FAST, "cross sweep y=-7 west"),
    (5.0, -7.0, FWD_FAST, "cross sweep y=-7 east"),
    (0.0, -7.0, FWD_FAST, "cross sweep y=-7 center"),
    (-5.0, -3.0, FWD_FAST, "cross sweep y=-3 west"),
    (5.0, -3.0, FWD_FAST, "cross sweep y=-3 east"),
    (0.0, -3.0, FWD_FAST, "cross sweep y=-3 center"),
    (-5.0, 1.0, FWD_FAST, "cross sweep y=1 west"),
    (5.0, 1.0, FWD_FAST, "cross sweep y=1 east"),
    (0.0, 1.0, FWD_FAST, "cross sweep y=1 center"),
    (0.0, -10.0, FWD_FAST, "return to dock"),
]


class OdomMapper(Node):
    """Waypoint follower that relies on `/odom` pose feedback."""

    def __init__(self) -> None:
        super().__init__("odom_mapper")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )
        self.x = None
        self.y = None
        self.yaw = 0.0
        self.get_logger().info("Waiting for odometry...")

    def _odom_callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _publish_velocity(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    def _spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.02)

    def _stop(self, pause: float = 0.5) -> None:
        for _ in range(8):
            self._publish_velocity(0.0, 0.0)
            self._spin_once()
        time.sleep(pause)

    def _wait_for_odom(self) -> None:
        while self.x is None:
            self._spin_once()
            time.sleep(0.05)

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        delta = target - current
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        return delta

    def go_to(self, target_x: float, target_y: float, speed: float, label: str) -> bool:
        """Navigate to a waypoint using rotate-then-drive control."""
        self._wait_for_odom()
        self.get_logger().info(
            f"  -> [{label}] target=({target_x:.2f}, {target_y:.2f}) "
            f"current=({self.x:.2f}, {self.y:.2f})"
        )

        for _ in range(3):
            self._spin_once()
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

            if distance < XY_TOLERANCE:
                self._stop(0.4)
                return True

            target_yaw = math.atan2(dy, dx)
            yaw_error = self._angle_diff(target_yaw, self.yaw)

            while abs(yaw_error) > YAW_TOLERANCE:
                self._spin_once()
                yaw_error = self._angle_diff(
                    math.atan2(target_y - self.y, target_x - self.x), self.yaw
                )
                turn_speed = TURN_SPEED if yaw_error > 0 else -TURN_SPEED
                if abs(yaw_error) < 0.3:
                    turn_speed *= 0.5
                self._publish_velocity(0.0, turn_speed)
                time.sleep(0.02)

            self._stop(0.2)

            while True:
                self._spin_once()
                dx = target_x - self.x
                dy = target_y - self.y
                distance = math.hypot(dx, dy)

                if distance < XY_TOLERANCE:
                    break

                target_yaw = math.atan2(dy, dx)
                yaw_error = self._angle_diff(target_yaw, self.yaw)

                if abs(yaw_error) > 0.35:
                    break

                linear_speed = speed if distance > 0.8 else speed * 0.6
                angular_correction = max(-0.4, min(0.4, yaw_error * 1.5))
                self._publish_velocity(linear_speed, angular_correction)
                time.sleep(0.02)

        self._stop(0.4)
        self.get_logger().warn(
            f"  !! Could not fully reach ({target_x:.1f}, {target_y:.1f}); "
            f"ended at ({self.x:.2f}, {self.y:.2f})"
        )
        return False

    def run(self) -> None:
        self._wait_for_odom()
        self.get_logger().info(f"Start position: ({self.x:.2f}, {self.y:.2f})")
        self.get_logger().info(f"Total waypoints: {len(WAYPOINTS)}")
        time.sleep(1.0)

        for index, (target_x, target_y, speed, label) in enumerate(WAYPOINTS, start=1):
            self.get_logger().info(
                f"[{index}/{len(WAYPOINTS)}] Heading to "
                f"({target_x:.2f}, {target_y:.2f}) - {label}"
            )
            self.go_to(target_x, target_y, speed, label)

        self._stop(2.0)
        self.get_logger().info("=" * 50)
        self.get_logger().info("MAPPING COMPLETE")
        self.get_logger().info(
            "ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map "
            "--ros-args -p save_map_timeout:=5000.0"
        )
        self.get_logger().info("=" * 50)


def main() -> None:
    rclpy.init()
    node = OdomMapper()
    try:
        node.run()
    except KeyboardInterrupt:
        node._stop()
        node.get_logger().info("Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
