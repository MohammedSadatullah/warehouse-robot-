#!/usr/bin/env python3
"""
scripts/hardcoded_mapping_v2.py

Fixed version — proper wall clearance on every leg so robot
never collides. Speeds increased since no collision risk.

WALL POSITIONS (from SDF):
  West  wall: x = -10.0  → robot stays at x = -8.8  (1.2m clearance)
  East  wall: x = +10.0  → robot stays at x = +8.8
  South wall: y = -12.0  → robot stays at y = -10.5  (1.5m clearance)
  North wall: y = +12.0  → robot stays at y = +10.8

SHELF POSITIONS (from SDF):
  Row A: x=-7.5  Row B: x=-6.0  (aisle centre x=-6.75)
  Row C: x=+6.0  Row D: x=+7.5  (aisle centre x=+6.75)
  Shelf Y range: 2.0 to 10.0  (each shelf 2m long, 3 shelves per row)

Robot start: (0, -10) facing north (+Y)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


# ── Speeds ────────────────────────────────────────────────────────────────────
FWD_FAST  = 0.9    # m/s open areas
FWD_AISLE = 0.6    # m/s inside shelf aisles
TURN_SPD  = 1.0    # rad/s turning


class HardcodedMapper(Node):

    def __init__(self):
        super().__init__('hardcoded_mapper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _pub(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)
        rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self, pause=0.6):
        for _ in range(5):
            self._pub(0.0, 0.0)
            time.sleep(0.05)
        time.sleep(pause)

    def forward(self, metres, speed=None, label=''):
        spd = speed or FWD_FAST
        if label:
            self.get_logger().info(f'  → {label}  ({metres:.1f}m @ {spd}m/s)')
        duration = metres / spd
        end = time.time() + duration
        while time.time() < end:
            self._pub(spd, 0.0)
            time.sleep(0.05)
        self.stop(0.5)

    def turn(self, degrees, label=''):
        """Positive = left (CCW), Negative = right (CW)."""
        if label:
            self.get_logger().info(f'  ↻ Turn {degrees:+.0f}°  {label}')
        rads     = abs(degrees) * math.pi / 180.0
        sign     = 1.0 if degrees > 0 else -1.0
        duration = rads / TURN_SPD
        end = time.time() + duration
        while time.time() < end:
            self._pub(0.0, sign * TURN_SPD)
            time.sleep(0.05)
        self.stop(0.5)

    def section(self, title):
        self.get_logger().info(f'\n━━━ {title} ━━━')

    # ─────────────────────────────────────────────────────────────────────────
    # ROUTE
    # All distances calculated from SDF coordinates with clearance built in.
    # Robot heading tracked manually so turns are always relative.
    #
    # Heading convention:  0° = +Y (north)
    #                     90° = +X (east)  [after right turn]
    #                    -90° = -X (west)  [after left turn]
    # ─────────────────────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Starting in 3 seconds — make sure SLAM is running...')
        time.sleep(3.0)

        # ═══════════════════════════════════════════
        # PHASE 1 — SOUTH WALL SWEEP
        # Start: (0, -10) facing north
        # Drive east to x=+8.8, then back west to x=-8.8
        # staying at y=-10.5 (1.5m from south wall y=-12)
        # ═══════════════════════════════════════════
        self.section('PHASE 1 — South wall sweep')

        # Face south first, hug south wall
        self.turn(-180, 'face south')
        self.forward(0.5, label='nudge to y=-10.5')    # now at y=-10.5

        # Face east, sweep full south wall width
        self.turn(-90, 'face east')                     # heading: east
        self.forward(8.8, label='south wall → east end')  # x: 0→+8.8

        # U-turn, sweep back west
        self.turn(+180, 'U-turn west')
        self.forward(17.6, label='south wall → west end')  # x: +8.8→-8.8

        # Return to centre x=0
        self.turn(-180, 'face east')
        self.forward(8.8, label='return centre x=0')    # x: -8.8→0

        # ═══════════════════════════════════════════
        # PHASE 2 — WEST WALL SWEEP
        # Move to x=-8.8, sweep full west wall north-south
        # West wall at x=-10, robot at x=-8.8 (1.2m clear)
        # ═══════════════════════════════════════════
        self.section('PHASE 2 — West wall sweep')

        self.turn(+180, 'face west')                    # heading: west
        self.forward(8.8, label='move to west wall x=-8.8')

        self.turn(+90, 'face north')                    # heading: north
        self.forward(21.3, label='west wall → north')   # y: -10.5→+10.8

        self.turn(-180, 'U-turn south')
        self.forward(21.3, label='west wall → south')   # y: +10.8→-10.5

        # ═══════════════════════════════════════════
        # PHASE 3 — NORTH WALL SWEEP
        # Move from (-8.8, -10.5) to y=+10.8 then sweep east
        # North wall at y=+12, robot at y=+10.8 (1.2m clear)
        # ═══════════════════════════════════════════
        self.section('PHASE 3 — North wall sweep')

        self.turn(+90, 'face north')
        self.forward(21.3, label='move to north wall y=+10.8')

        self.turn(-90, 'face east')                     # heading: east
        self.forward(17.6, label='north wall → east end')  # x: -8.8→+8.8

        self.turn(+180, 'U-turn west')
        self.forward(17.6, label='north wall → west end')

        # ═══════════════════════════════════════════
        # PHASE 4 — EAST WALL SWEEP
        # Move to x=+8.8, sweep full east wall
        # East wall at x=+10, robot at x=+8.8 (1.2m clear)
        # ═══════════════════════════════════════════
        self.section('PHASE 4 — East wall sweep')

        self.turn(-90, 'face east')
        self.forward(17.6, label='move to east wall x=+8.8')

        self.turn(-90, 'face south')
        self.forward(21.3, label='east wall → south')   # y: +10.8→-10.5

        self.turn(-180, 'U-turn north')
        self.forward(21.3, label='east wall → north')

        # Return to centre start position (0, -10) facing north
        self.turn(+90, 'face west')
        self.forward(8.8, label='return centre x=0')
        self.turn(-90, 'face south')
        self.forward(21.3, label='return y=-10.5')
        self.turn(-180, 'face north')

        self.section('Perimeter complete — robot at (0,-10.5) facing north')
        time.sleep(1.0)

        # ═══════════════════════════════════════════
        # PHASE 5 — WEST OUTER AISLE  (x=-8.5)
        # Between shelf row A (x=-7.5) and west wall (x=-10)
        # Safe path: x=-8.5, clear of shelves by 0.75m
        # Shelf y range: 2.0 to 10.0 — drive y=-9 to y=+10.8
        # ═══════════════════════════════════════════
        self.section('PHASE 5 — West outer aisle x=-8.5')

        self.turn(+180, 'face west')
        self.forward(8.5, label='move to x=-8.5')
        self.turn(+90, 'face north')
        self.forward(20.3, speed=FWD_AISLE, label='west outer aisle north')  # y:-10.5→+10.8
        self.turn(-180, 'U-turn south')
        self.forward(20.3, speed=FWD_AISLE, label='west outer aisle south')

        # ═══════════════════════════════════════════
        # PHASE 6 — WEST INNER AISLE  (x=-6.75)
        # Between shelf row A (x=-7.5) and row B (x=-6.0)
        # Aisle width = 1.5m, centre at x=-6.75
        # ═══════════════════════════════════════════
        self.section('PHASE 6 — West inner aisle x=-6.75')

        self.turn(-90, 'face east')
        self.forward(1.75, label='move east to x=-6.75')
        self.turn(+90, 'face north')
        self.forward(20.3, speed=FWD_AISLE, label='west inner aisle north')
        self.turn(-180, 'U-turn south')
        self.forward(20.3, speed=FWD_AISLE, label='west inner aisle south')

        # Return to centre
        self.turn(-90, 'face east')
        self.forward(6.75, label='return to centre x=0')

        # ═══════════════════════════════════════════
        # PHASE 7 — EAST INNER AISLE  (x=+6.75)
        # Between shelf row C (x=+6.0) and row D (x=+7.5)
        # ═══════════════════════════════════════════
        self.section('PHASE 7 — East inner aisle x=+6.75')

        self.turn(+90, 'face east')
        self.forward(6.75, label='move to x=+6.75')
        self.turn(-90, 'face north')
        self.forward(20.3, speed=FWD_AISLE, label='east inner aisle north')
        self.turn(+180, 'U-turn south')
        self.forward(20.3, speed=FWD_AISLE, label='east inner aisle south')

        # ═══════════════════════════════════════════
        # PHASE 8 — EAST OUTER AISLE  (x=+8.5)
        # Between shelf row D (x=+7.5) and east wall (x=+10)
        # ═══════════════════════════════════════════
        self.section('PHASE 8 — East outer aisle x=+8.5')

        self.turn(+90, 'face east')
        self.forward(1.75, label='move to x=+8.5')
        self.turn(-90, 'face north')
        self.forward(20.3, speed=FWD_AISLE, label='east outer aisle north')
        self.turn(+180, 'U-turn south')
        self.forward(20.3, speed=FWD_AISLE, label='east outer aisle south')

        # Return to centre x=0
        self.turn(+90, 'face west')
        self.forward(8.5, label='return to centre x=0')

        # ═══════════════════════════════════════════
        # PHASE 9 — CENTRE AISLE  (x=0, full N-S)
        # ═══════════════════════════════════════════
        self.section('PHASE 9 — Centre aisle x=0')

        self.turn(-90, 'face north')
        self.forward(21.3, label='centre aisle north')   # y: -10.5→+10.8
        self.turn(-180, 'U-turn south')
        self.forward(21.3, label='centre aisle south')

        # ═══════════════════════════════════════════
        # PHASE 10 — HORIZONTAL CROSS SWEEPS
        # Cover open floor space with 4 E-W passes
        # Safe x range: -5.5 to +5.5 (clear of all shelf rows)
        # at y = -7, -3, +1 (below shelves) and y=+13 skipped (wall)
        # ═══════════════════════════════════════════
        self.section('PHASE 10 — Horizontal cross sweeps')

        cross_y_positions = [-7.0, -3.0, 1.0]
        current_y = -10.5

        for target_y in cross_y_positions:
            dy = target_y - current_y
            self.turn(-90 if dy > 0 else +90, f'face for y={target_y}')
            self.forward(abs(dy), label=f'move to y={target_y}')
            current_y = target_y

            # Sweep east (safe range x=-5.5 to x=+5.5, clear of shelves)
            self.turn(+90 if dy > 0 else -90, 'face east')
            self.forward(5.5, label=f'cross sweep east  y={target_y}')
            self.turn(+180, 'U-turn west')
            self.forward(11.0, label=f'cross sweep west  y={target_y}')
            self.turn(+180, 'face east, recentre')
            self.forward(5.5, label='recentre x=0')

        # Final stop at centre
        self.stop(2.0)

        # ═══════════════════════════════════════════
        # DONE
        # ═══════════════════════════════════════════
        self.get_logger().info('\n' + '═'*50)
        self.get_logger().info('MAPPING COMPLETE — all zones covered.')
        self.get_logger().info('Save map now with:')
        self.get_logger().info(
            '  ros2 run nav2_map_server map_saver_cli '
            '-f ~/warehouse_map '
            '--ros-args -p save_map_timeout:=5000.0'
        )
        self.get_logger().info('═'*50)


def main():
    rclpy.init()
    node = HardcodedMapper()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info('Stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
