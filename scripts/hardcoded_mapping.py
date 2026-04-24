#!/usr/bin/env python3
"""
scripts/hardcoded_mapping.py

Pure timed velocity commands — no sensor feedback needed.
Traces the exact warehouse geometry to build a complete map.

ROUTE (in order):
  Phase 1 — Perimeter: S-wall → W-wall → N-wall → E-wall → back south
  Phase 2 — West shelf aisles: aisle A-B (x=-6.75), outer west (x=-8.5)
  Phase 3 — East shelf aisles: aisle C-D (x=+6.75), outer east (x=+8.5)
  Phase 4 — Centre aisle: full N-S sweep at x=0
  Phase 5 — Cross sweeps: 3 horizontal passes to fill centre open space

Assumes robot starts near (0, -10) facing +Y (north).
Robot must already be spawned and cmd_vel bridge must be running.

Run:
    python3 hardcoded_mapping.py

Save map after completion:
    ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map --ros-args -p save_map_timeout:=5000.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


# ── Speed settings ────────────────────────────────────────────────────────────
FWD   = 0.6    # m/s forward  — fast but safe for hardcoded straight runs
TURN  = 0.8    # rad/s turn
SLOW  = 0.35   # m/s for tight aisle sections


def make_twist(linear=0.0, angular=0.0):
    t = Twist()
    t.linear.x  = float(linear)
    t.angular.z = float(angular)
    return t


class HardcodedMapper(Node):

    def __init__(self):
        super().__init__('hardcoded_mapper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Hardcoded mapper ready. Starting in 3 seconds...')

    def pub_vel(self, linear, angular):
        self.pub.publish(make_twist(linear, angular))
        rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, linear, angular, duration, label=''):
        """Publish constant velocity for `duration` seconds."""
        if label:
            self.get_logger().info(f'  >> {label}')
        end = time.time() + duration
        while time.time() < end:
            self.pub_vel(linear, angular)
            time.sleep(0.05)

    def stop(self, pause=0.5):
        """Stop and pause briefly to let SLAM process the scan."""
        self.pub_vel(0.0, 0.0)
        time.sleep(pause)

    def turn_deg(self, degrees, label=''):
        """Turn by approximately `degrees` in place."""
        if label:
            self.get_logger().info(f'  >> Turn {degrees}° — {label}')
        radians  = abs(degrees) * math.pi / 180.0
        direction = 1.0 if degrees > 0 else -1.0
        duration = radians / TURN
        self.drive(0.0, direction * TURN, duration)
        self.stop(0.3)

    def forward(self, metres, speed=None, label=''):
        """Drive forward `metres` at given speed."""
        spd = speed or FWD
        duration = metres / spd
        self.drive(spd, 0.0, duration, label)
        self.stop(0.3)

    def run(self):
        time.sleep(3.0)   # let ROS settle

        # ═══════════════════════════════════════════════════════
        # PHASE 1 — PERIMETER  (hug each wall in sequence)
        # Robot starts at approx (0, -10) facing north (+Y)
        # Wall clearance: 0.5m from each wall
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('━━━ PHASE 1: Perimeter sweep ━━━')

        # --- South wall: drive east along south wall ---
        self.turn_deg(-90, 'face east')          # now facing +X
        self.forward(8.5, label='S-wall east run')   # to x≈+8.5

        # --- SE corner, turn north ---
        self.turn_deg(90, 'SE corner → face north')
        self.forward(22.5, label='E-wall north run')  # y: -11 → +11.5

        # --- NE corner, turn west ---
        self.turn_deg(90, 'NE corner → face west')
        self.forward(17.5, label='N-wall west run')   # x: +8.5 → -9

        # --- NW corner, turn south ---
        self.turn_deg(90, 'NW corner → face south')
        self.forward(22.5, label='W-wall south run')  # y: +11.5 → -11

        # --- SW corner, turn east back to centre ---
        self.turn_deg(90, 'SW corner → face east')
        self.forward(9.0, label='return to centre south')  # x: -9 → 0

        self.stop(1.0)

        # ═══════════════════════════════════════════════════════
        # PHASE 2 — WEST SHELF AISLES
        # Aisle A-B at x=-6.75 (between shelves A and B)
        # Outer west pass at x=-8.5 (between shelf A and west wall)
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('━━━ PHASE 2: West shelf aisles ━━━')

        # Move to entry of west aisle A-B
        self.turn_deg(90, 'face north')
        self.forward(2.0, label='move north to shelf entry y=-8')
        self.turn_deg(-90, 'face west')
        self.forward(6.75, label='enter west aisle A-B x=-6.75')

        # Drive north through aisle A-B (y: -8 → +11)
        self.turn_deg(90, 'face north in aisle A-B')
        self.forward(19.0, speed=SLOW, label='aisle A-B northward')

        # Turn around, drive south back through aisle
        self.turn_deg(180, 'U-turn south in aisle A-B')
        self.forward(19.0, speed=SLOW, label='aisle A-B southward')

        # Outer west pass: move to x=-8.5
        self.turn_deg(-90, 'face further west')
        self.forward(1.75, label='move to outer west x=-8.5')
        self.turn_deg(90, 'face north for outer west pass')
        self.forward(19.0, speed=SLOW, label='outer west northward')
        self.turn_deg(180, 'U-turn')
        self.forward(19.0, speed=SLOW, label='outer west southward')

        # Return to centre
        self.turn_deg(-90, 'face east to return')
        self.forward(8.5, label='return to x=0')
        self.stop(1.0)

        # ═══════════════════════════════════════════════════════
        # PHASE 3 — EAST SHELF AISLES
        # Aisle C-D at x=+6.75
        # Outer east pass at x=+8.5
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('━━━ PHASE 3: East shelf aisles ━━━')

        # Move to entry of east aisle C-D
        self.turn_deg(90, 'face north at centre')
        self.forward(2.0, label='north to shelf entry')
        self.turn_deg(90, 'face east')
        self.forward(6.75, label='enter east aisle C-D x=+6.75')

        # Drive north through aisle C-D
        self.turn_deg(-90, 'face north in aisle C-D')
        self.forward(19.0, speed=SLOW, label='aisle C-D northward')
        self.turn_deg(180, 'U-turn')
        self.forward(19.0, speed=SLOW, label='aisle C-D southward')

        # Outer east pass: move to x=+8.5
        self.turn_deg(90, 'face further east')
        self.forward(1.75, label='move to outer east x=+8.5')
        self.turn_deg(-90, 'face north for outer east pass')
        self.forward(19.0, speed=SLOW, label='outer east northward')
        self.turn_deg(180, 'U-turn')
        self.forward(19.0, speed=SLOW, label='outer east southward')

        # Return to centre
        self.turn_deg(90, 'face west to return')
        self.forward(8.5, label='return to x=0')
        self.stop(1.0)

        # ═══════════════════════════════════════════════════════
        # PHASE 4 — CENTRE AISLE  (full N-S sweep at x=0)
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('━━━ PHASE 4: Centre aisle ━━━')

        self.turn_deg(90, 'face north for centre aisle')
        self.forward(22.0, label='centre aisle full northward')
        self.turn_deg(180, 'U-turn at north wall')
        self.forward(22.0, label='centre aisle full southward')
        self.stop(1.0)

        # ═══════════════════════════════════════════════════════
        # PHASE 5 — HORIZONTAL CROSS SWEEPS
        # 3 east-west passes to cover open centre space
        # at y=-5, y=0, y=+5  (between shelves and walls)
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('━━━ PHASE 5: Cross sweeps ━━━')

        cross_rows = [
            (-5.0, 'south cross sweep  y=-5'),
            ( 0.0, 'centre cross sweep y=0'),
            ( 5.0, 'north cross sweep  y=+5'),
        ]

        current_y = -10.0   # approximate current y after phase 4

        for target_y, label in cross_rows:
            # Navigate to correct y
            dy = target_y - current_y
            self.turn_deg(90 if dy > 0 else -90, f'face for y={target_y}')
            self.forward(abs(dy), label=f'move to {label}')
            current_y = target_y

            # Sweep west to east
            self.turn_deg(90, 'face east for sweep')
            self.forward(17.0, label=f'{label} → east')

            # Sweep east back to west
            self.turn_deg(180, 'U-turn west')
            self.forward(17.0, label=f'{label} ← west')

            # Return to centre x
            self.turn_deg(-90, 'face east to recentre')
            self.forward(8.5, label='recentre to x=0')

        self.stop(2.0)

        # ═══════════════════════════════════════════════════════
        # DONE
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('MAPPING COMPLETE — all zones covered.')
        self.get_logger().info('')
        self.get_logger().info('Save your map now:')
        self.get_logger().info('  ros2 run nav2_map_server map_saver_cli \\')
        self.get_logger().info('    -f ~/warehouse_map \\')
        self.get_logger().info('    --ros-args -p save_map_timeout:=5000.0')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')


def main():
    rclpy.init()
    node = HardcodedMapper()
    try:
        node.run()
    except KeyboardInterrupt:
        node.pub_vel(0.0, 0.0)
        node.get_logger().info('Stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
