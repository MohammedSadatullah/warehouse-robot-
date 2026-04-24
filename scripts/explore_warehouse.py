#!/usr/bin/env python3
"""
scripts/explore_warehouse.py
Starts from wherever the robot is, sorts waypoints nearest-first.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

ALL_WAYPOINTS = [
    ( 0.0,  -9.0), ( 0.0,  -6.0), (-4.0,  -9.0), ( 4.0,  -9.0),
    (-7.0,  -6.0), (-7.0,   0.0), (-7.0,   6.0), (-7.0,  11.0),
    (-6.5,  10.0), (-6.5,   5.0), (-6.5,   2.0),
    (-3.0,  11.0), ( 0.0,  11.0), ( 3.0,  11.0),
    ( 6.5,  10.0), ( 6.5,   5.0), ( 6.5,   2.0),
    ( 7.0,   6.0), ( 7.0,   0.0), ( 7.0,  -6.0), ( 7.0, -11.0),
    ( 0.0,   0.0), ( 0.0,   5.0), ( 2.0,   2.0), (-2.0,   2.0),
]

WAYPOINT_TOLERANCE = 0.6
LINEAR_SPEED       = 0.4
ANGULAR_SPEED      = 0.8
OBSTACLE_DIST      = 0.45


class WarehouseExplorer(Node):
    def __init__(self):
        super().__init__('warehouse_explorer')
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.x = None; self.y = None; self.yaw = 0.0
        self.min_front_dist = float('inf'); self.obstacle_ahead = False
        self.waypoints = []; self.wp_idx = 0; self.ready = False
        self.get_logger().info('Waiting for odometry...')
        self.init_timer    = self.create_timer(0.5, self.wait_for_odom)
        self.control_timer = None

    def wait_for_odom(self):
        if self.x is None:
            return
        self.init_timer.cancel()
        self.get_logger().info(f'Start position: ({self.x:.2f}, {self.y:.2f})')
        self.waypoints = sorted(ALL_WAYPOINTS,
            key=lambda wp: math.sqrt((wp[0]-self.x)**2 + (wp[1]-self.y)**2))
        self.get_logger().info(f'First waypoint: {self.waypoints[0]}')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.ready = True
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0: return
        cone = 15
        front = list(range(0, cone)) + list(range(n - cone, n))
        valid = [msg.ranges[i] for i in front
                 if msg.range_min < msg.ranges[i] < msg.range_max]
        self.min_front_dist = min(valid) if valid else float('inf')
        self.obstacle_ahead = self.min_front_dist < OBSTACLE_DIST

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def distance_to(self, wp):
        return math.sqrt((wp[0]-self.x)**2 + (wp[1]-self.y)**2)

    def angle_to(self, wp):
        return math.atan2(wp[1]-self.y, wp[0]-self.x)

    def angle_diff(self, a, b):
        d = a - b
        while d >  math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    def publish_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear); msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_vel(0.0, 0.0)

    def control_loop(self):
        if not self.ready or self.x is None: return
        if self.wp_idx >= len(self.waypoints):
            self.stop()
            self.get_logger().info('Exploration complete!')
            self.get_logger().info('Save map: ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map')
            self.control_timer.cancel()
            return
        wp   = self.waypoints[self.wp_idx]
        dist = self.distance_to(wp)
        if dist < WAYPOINT_TOLERANCE:
            self.stop()
            self.get_logger().info(f'[{self.wp_idx+1}/{len(self.waypoints)}] Reached ({wp[0]:.1f},{wp[1]:.1f})')
            self.wp_idx += 1
            return
        if self.obstacle_ahead:
            self.get_logger().warn(f'Obstacle {self.min_front_dist:.2f}m — avoiding')
            self.publish_vel(0.0, ANGULAR_SPEED * 1.5)
            return
        yaw_err = self.angle_diff(self.angle_to(wp), self.yaw)
        if abs(yaw_err) > 0.3:
            self.publish_vel(0.0, ANGULAR_SPEED if yaw_err > 0 else -ANGULAR_SPEED)
        else:
            corr  = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, yaw_err * 1.2))
            speed = LINEAR_SPEED if dist > 1.0 else LINEAR_SPEED * 0.5
            self.publish_vel(speed, corr)


def main():
    rclpy.init()
    node = WarehouseExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
