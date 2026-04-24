#!/usr/bin/env python3
"""
scripts/hardcoded_mapping_v3.py

Odometry-based movement — drives to EXACT coordinates using
/odom feedback. No timing drift. Works regardless of start position.

Reads current pose from /odom, then navigates to each waypoint
in sequence using precise position + heading control.

Run:
    python3 hardcoded_mapping_v3.py

Save map after:
    ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map \
      --ros-args -p save_map_timeout:=5000.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


# ── Speeds (TB3 Burger safe limits) ──────────────────────────────────────────
FWD_FAST  = 0.18   # m/s open runs
FWD_AISLE = 0.13   # m/s tight aisles
TURN_SPD  = 0.6    # rad/s turning

# ── Tolerances ────────────────────────────────────────────────────────────────
XY_TOL    = 0.25   # metres — close enough to waypoint
YAW_TOL   = 0.05   # radians — close enough to heading


# ── Full warehouse coverage waypoints ────────────────────────────────────────
# Derived from SDF: walls at x=±10, y=±12; shelves x=±6.0/±7.5, y=2..10
# Robot stays 1.2m from walls, 0.6m from shelves
#
# Format: (x, y, speed_override_or_None, label)

WAYPOINTS = [
    # ── Phase 1: South wall sweep ──────────────────────────
    ( 8.8, -10.5, FWD_FAST,  'S-wall east end'),
    (-8.8, -10.5, FWD_FAST,  'S-wall west end'),
    ( 0.0, -10.5, FWD_FAST,  'S-wall centre return'),

    # ── Phase 2: West wall sweep ───────────────────────────
    (-8.8, -10.5, FWD_FAST,  'W-wall south start'),
    (-8.8,  10.8, FWD_FAST,  'W-wall north end'),
    (-8.8, -10.5, FWD_FAST,  'W-wall south return'),

    # ── Phase 3: North wall sweep ──────────────────────────
    (-8.8,  10.8, FWD_FAST,  'N-wall west start'),
    ( 8.8,  10.8, FWD_FAST,  'N-wall east end'),
    (-8.8,  10.8, FWD_FAST,  'N-wall west return'),

    # ── Phase 4: East wall sweep ───────────────────────────
    ( 8.8,  10.8, FWD_FAST,  'E-wall north start'),
    ( 8.8, -10.5, FWD_FAST,  'E-wall south end'),
    ( 0.0, -10.5, FWD_FAST,  'return centre south'),

    # ── Phase 5: West outer aisle x=-8.5 ──────────────────
    (-8.5, -10.0, FWD_FAST,  'W-outer aisle south'),
    (-8.5,  10.5, FWD_AISLE, 'W-outer aisle north'),
    (-8.5, -10.0, FWD_AISLE, 'W-outer aisle south return'),

    # ── Phase 6: West inner aisle x=-6.75 (between A & B) ─
    (-6.75, -9.5, FWD_FAST,  'W-inner aisle entry'),
    (-6.75, 10.5, FWD_AISLE, 'W-inner aisle north'),
    (-6.75, -9.5, FWD_AISLE, 'W-inner aisle south return'),

    # ── Phase 7: Centre aisle x=0 ─────────────────────────
    ( 0.0,  -10.0, FWD_FAST,  'centre aisle south'),
    ( 0.0,   10.8, FWD_FAST,  'centre aisle north'),
    ( 0.0,  -10.0, FWD_FAST,  'centre aisle south return'),

    # ── Phase 8: East inner aisle x=+6.75 (between C & D) ─
    ( 6.75, -9.5, FWD_FAST,  'E-inner aisle entry'),
    ( 6.75, 10.5, FWD_AISLE, 'E-inner aisle north'),
    ( 6.75, -9.5, FWD_AISLE, 'E-inner aisle south return'),

    # ── Phase 9: East outer aisle x=+8.5 ──────────────────
    ( 8.5, -10.0, FWD_FAST,  'E-outer aisle south'),
    ( 8.5,  10.5, FWD_AISLE, 'E-outer aisle north'),
    ( 8.5, -10.0, FWD_AISLE, 'E-outer aisle south return'),

    # ── Phase 10: Cross sweeps (open floor) ───────────────
    # y=-7: south open area
    (-5.0,  -7.0, FWD_FAST, 'cross sweep y=-7 west'),
    ( 5.0,  -7.0, FWD_FAST, 'cross sweep y=-7 east'),
    ( 0.0,  -7.0, FWD_FAST, 'cross sweep y=-7 centre'),
    # y=-3: between entry and shelf zone
    (-5.0,  -3.0, FWD_FAST, 'cross sweep y=-3 west'),
    ( 5.0,  -3.0, FWD_FAST, 'cross sweep y=-3 east'),
    ( 0.0,  -3.0, FWD_FAST, 'cross sweep y=-3 centre'),
    # y=+1: middle shelf zone (between rows, open centre)
    (-5.0,   1.0, FWD_FAST, 'cross sweep y=+1 west'),
    ( 5.0,   1.0, FWD_FAST, 'cross sweep y=+1 east'),
    ( 0.0,   1.0, FWD_FAST, 'cross sweep y=+1 centre'),

    # ── Final: return to dock ──────────────────────────────
    ( 0.0, -10.0, FWD_FAST, 'return to dock'),
]


class OdomMapper(Node):

    def __init__(self):
        super().__init__('odom_mapper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.x   = None
        self.y   = None
        self.yaw = 0.0
        self.get_logger().info('Waiting for odometry...')

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def _vel(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

    def _stop(self, pause=0.5):
        for _ in range(8):
            self._vel(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(pause)

    def _spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.02)

    def _angle_diff(self, a, b):
        d = a - b
        while d >  math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    def _wait_odom(self):
        while self.x is None:
            self._spin_once()
            time.sleep(0.05)

    def go_to(self, tx, ty, speed, label):
        """Navigate to (tx, ty) using odom feedback."""
        self._wait_odom()
        self.get_logger().info(
            f'  → [{label}]  target=({tx:.2f},{ty:.2f})  '
            f'current=({self.x:.2f},{self.y:.2f})'
        )

        max_attempts = 3
        attempt = 0

        while attempt < max_attempts:
            self._spin_once()
            dx   = tx - self.x
            dy   = ty - self.y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < XY_TOL:
                self._stop(0.4)
                return True   # reached

            # ── Step 1: rotate to face target ──
            target_yaw = math.atan2(dy, dx)
            yaw_err    = self._angle_diff(target_yaw, self.yaw)

            while abs(yaw_err) > YAW_TOL:
                self._spin_once()
                yaw_err = self._angle_diff(
                    math.atan2(ty - self.y, tx - self.x),
                    self.yaw
                )
                turn = TURN_SPD if yaw_err > 0 else -TURN_SPD
                # Slow down near target angle
                if abs(yaw_err) < 0.3:
                    turn *= 0.5
                self._vel(0.0, turn)
                time.sleep(0.02)

            self._stop(0.2)

            # ── Step 2: drive straight to target ──
            while True:
                self._spin_once()
                dx   = tx - self.x
                dy   = ty - self.y
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < XY_TOL:
                    break

                # Heading correction while driving
                target_yaw = math.atan2(dy, dx)
                yaw_err    = self._angle_diff(target_yaw, self.yaw)

                # If we've drifted more than 20° re-rotate
                if abs(yaw_err) > 0.35:
                    break

                # Slow down when close
                spd = speed if dist > 0.8 else speed * 0.6
                correction = max(-0.4, min(0.4, yaw_err * 1.5))
                self._vel(spd, correction)
                time.sleep(0.02)

            attempt += 1

        self._stop(0.4)
        self.get_logger().warn(
            f'  !! Could not fully reach ({tx:.1f},{ty:.1f}) — '
            f'ended at ({self.x:.2f},{self.y:.2f})'
        )
        return False

    def run(self):
        self._wait_odom()
        self.get_logger().info(
            f'Start position: ({self.x:.2f}, {self.y:.2f})'
        )
        self.get_logger().info(f'Total waypoints: {len(WAYPOINTS)}')
        time.sleep(1.0)

        for i, (tx, ty, spd, label) in enumerate(WAYPOINTS):
            self.get_logger().info(
                f'[{i+1}/{len(WAYPOINTS)}] Heading to: ({tx:.2f},{ty:.2f}) — {label}'
            )
            self.go_to(tx, ty, spd, label)

        self._stop(2.0)
        self.get_logger().info('═' * 50)
        self.get_logger().info('MAPPING COMPLETE')
        self.get_logger().info(
            'ros2 run nav2_map_server map_saver_cli '
            '-f ~/warehouse_map '
            '--ros-args -p save_map_timeout:=5000.0'
        )
        self.get_logger().info('═' * 50)


def main():
    rclpy.init()
    node = OdomMapper()
    try:
        node.run()
    except KeyboardInterrupt:
        node._stop()
        node.get_logger().info('Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
