#!/usr/bin/env python3
"""
scripts/dynamic_actors.py

Moves the three pedestrian cylinder models in Gazebo Harmonic
using the /world/warehouse/set_pose service via gz-transport.

This is the reliable Harmonic alternative to <actor> tags,
which require a Fuel mesh download that often fails offline.

Run AFTER gz sim is already running:
    python3 dynamic_actors.py

Requirements:
    pip3 install gz-transport13   # matches Gazebo Harmonic
    OR use the system package:
    sudo apt install python3-gz-transport13
"""

import math
import time
import threading

try:
    from gz.transport13 import Node
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.vector3d_pb2 import Vector3d
    from gz.msgs10.quaternion_pb2 import Quaternion
    GZ_TRANSPORT_AVAILABLE = True
except ImportError:
    GZ_TRANSPORT_AVAILABLE = False
    print("[dynamic_actors] gz-transport not found — using fallback ROS2 mode")

import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import Pose as RosPose
from std_msgs.msg import String


# ── Patrol paths for each pedestrian ────────────────────────────────────────
# Each entry: (x, y, z, heading_rad)
# z is fixed at 0.85 (half cylinder height)

PATROL_CENTRE = [
    (0.0,  -8.0, 0.85,  1.5708),   # south end, facing north
    (0.0,   8.0, 0.85,  1.5708),   # north end
    (0.0,  -8.0, 0.85, -1.5708),   # back south
]

PATROL_WEST = [
    (-6.75,  2.0, 0.85,  1.5708),
    (-6.75, 10.5, 0.85,  1.5708),
    (-6.75,  2.0, 0.85, -1.5708),
]

PATROL_CROSS = [
    (-8.5,   0.0, 0.85,  0.0),     # west wall, facing east
    ( 8.5,   0.0, 0.85,  0.0),     # east wall
    (-8.5,   0.0, 0.85,  3.1416),  # back west
]

SPEEDS = {
    "pedestrian_1": 1.2,   # m/s  centre aisle
    "pedestrian_2": 0.9,   # m/s  west aisle (slower, narrower)
    "pedestrian_3": 1.4,   # m/s  crossing (fast — harder for replanner)
}


def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion (w, x, y, z)."""
    return (
        math.cos(yaw / 2),   # w
        0.0,                  # x
        0.0,                  # y
        math.sin(yaw / 2),   # z
    )


def interpolate_pose(p1, p2, t):
    """Linear interpolation between two waypoints at fraction t (0..1)."""
    x = p1[0] + (p2[0] - p1[0]) * t
    y = p1[1] + (p2[1] - p1[1]) * t
    z = p1[2]
    # Use heading of p2 (direction of travel)
    heading = p2[3]
    return x, y, z, heading


class PedestrianMover:
    """Moves one pedestrian model along its patrol path."""

    def __init__(self, name, waypoints, speed, set_pose_fn):
        self.name = name
        self.waypoints = waypoints
        self.speed = speed
        self.set_pose_fn = set_pose_fn
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        wp_idx = 0
        while True:
            p1 = self.waypoints[wp_idx % len(self.waypoints)]
            p2 = self.waypoints[(wp_idx + 1) % len(self.waypoints)]

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dist = math.sqrt(dx * dx + dy * dy)
            duration = dist / self.speed
            steps = max(int(duration / 0.05), 1)   # 20 Hz updates

            for i in range(steps + 1):
                t = i / steps
                x, y, z, yaw = interpolate_pose(p1, p2, t)
                self.set_pose_fn(self.name, x, y, z, yaw)
                time.sleep(0.05)

            wp_idx += 1


# ── gz-transport pose setter ─────────────────────────────────────────────────

class GzPoseSetter:
    def __init__(self, world_name="warehouse"):
        if not GZ_TRANSPORT_AVAILABLE:
            raise RuntimeError("gz-transport not available")
        self.node = Node()
        self.world = world_name
        # Request publisher for set_pose
        self._pub = self.node.advertise(
            f"/world/{world_name}/set_pose",
            Pose
        )

    def set_pose(self, model_name, x, y, z, yaw):
        msg = Pose()
        msg.name = model_name
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        w, qx, qy, qz = yaw_to_quaternion(yaw)
        msg.orientation.w = w
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        self._pub.publish(msg)


# ── ROS2 fallback pose setter (uses gz service via subprocess) ────────────────

class Ros2PoseSetter:
    """Fallback: calls gz service via subprocess if gz-transport Python not available."""
    import subprocess

    def set_pose(self, model_name, x, y, z, yaw):
        w, qx, qy, qz = yaw_to_quaternion(yaw)
        cmd = (
            f"gz service -s /world/warehouse/set_pose "
            f"--reqtype gz.msgs.Pose "
            f"--reptype gz.msgs.Boolean "
            f"--timeout 50 "
            f"--req 'name: \"{model_name}\", "
            f"position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
            f"orientation: {{w: {w:.4f}, x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}}}'"
        )
        self.subprocess.run(cmd, shell=True, capture_output=True)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("[dynamic_actors] Starting pedestrian motion controller...")
    print("[dynamic_actors] Make sure 'gz sim warehouse_v2.sdf' is already running.")
    time.sleep(2.0)   # give Gazebo time to fully start

    # Try gz-transport first, fall back to subprocess
    try:
        setter = GzPoseSetter(world_name="warehouse")
        print("[dynamic_actors] Using gz-transport Python bindings")
    except Exception as e:
        print(f"[dynamic_actors] gz-transport failed ({e}), using subprocess fallback")
        setter = Ros2PoseSetter()

    movers = [
        PedestrianMover("pedestrian_1", PATROL_CENTRE, SPEEDS["pedestrian_1"], setter.set_pose),
        PedestrianMover("pedestrian_2", PATROL_WEST,   SPEEDS["pedestrian_2"], setter.set_pose),
        PedestrianMover("pedestrian_3", PATROL_CROSS,  SPEEDS["pedestrian_3"], setter.set_pose),
    ]

    for m in movers:
        m.start()
        print(f"[dynamic_actors] Started {m.name} patrol")

    print("[dynamic_actors] All pedestrians moving. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[dynamic_actors] Stopped.")


if __name__ == "__main__":
    main()
