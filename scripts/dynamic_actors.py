#!/usr/bin/env python3
"""Move pedestrian obstacles through scripted patrols in Gazebo Harmonic."""

import math
import threading
import time

try:
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node as GzNode

    GZ_TRANSPORT_AVAILABLE = True
except ImportError:
    GZ_TRANSPORT_AVAILABLE = False
    print("[dynamic_actors] gz-transport not found; using subprocess fallback")


PATROL_CENTRE = [
    (0.0, -8.0, 0.85, 1.5708),
    (0.0, 8.0, 0.85, 1.5708),
    (0.0, -8.0, 0.85, -1.5708),
]

PATROL_WEST = [
    (-6.75, 2.0, 0.85, 1.5708),
    (-6.75, 10.5, 0.85, 1.5708),
    (-6.75, 2.0, 0.85, -1.5708),
]

PATROL_CROSS = [
    (-8.5, 0.0, 0.85, 0.0),
    (8.5, 0.0, 0.85, 0.0),
    (-8.5, 0.0, 0.85, 3.1416),
]

SPEEDS = {
    "pedestrian_1": 1.2,
    "pedestrian_2": 0.9,
    "pedestrian_3": 1.4,
}


def yaw_to_quaternion(yaw):
    """Convert yaw to a quaternion tuple in w, x, y, z order."""
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def interpolate_pose(start, end, fraction):
    """Linearly interpolate between two path points."""
    x = start[0] + (end[0] - start[0]) * fraction
    y = start[1] + (end[1] - start[1]) * fraction
    z = start[2]
    yaw = end[3]
    return x, y, z, yaw


class PedestrianMover:
    """Drive one pedestrian model along a repeating waypoint sequence."""

    def __init__(self, name, waypoints, speed, set_pose_fn):
        self.name = name
        self.waypoints = waypoints
        self.speed = speed
        self.set_pose_fn = set_pose_fn
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        waypoint_index = 0
        while True:
            start = self.waypoints[waypoint_index % len(self.waypoints)]
            end = self.waypoints[(waypoint_index + 1) % len(self.waypoints)]
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            duration = distance / self.speed
            steps = max(int(duration / 0.05), 1)

            for step in range(steps + 1):
                fraction = step / steps
                x, y, z, yaw = interpolate_pose(start, end, fraction)
                self.set_pose_fn(self.name, x, y, z, yaw)
                time.sleep(0.05)

            waypoint_index += 1


class GzPoseSetter:
    """Publish pose updates directly over gz-transport."""

    def __init__(self, world_name="warehouse"):
        if not GZ_TRANSPORT_AVAILABLE:
            raise RuntimeError("gz-transport not available")
        self.node = GzNode()
        self.publisher = self.node.advertise(f"/world/{world_name}/set_pose", Pose)

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
        self.publisher.publish(msg)


class Ros2PoseSetter:
    """Fallback implementation using `gz service` calls."""

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


def main():
    print("[dynamic_actors] Starting pedestrian motion controller...")
    print("[dynamic_actors] Make sure 'gz sim warehouse_v2.sdf' is already running.")
    time.sleep(2.0)

    try:
        setter = GzPoseSetter(world_name="warehouse")
        print("[dynamic_actors] Using gz-transport Python bindings")
    except Exception as error:
        print(f"[dynamic_actors] gz-transport failed ({error}); using subprocess fallback")
        setter = Ros2PoseSetter()

    movers = [
        PedestrianMover("pedestrian_1", PATROL_CENTRE, SPEEDS["pedestrian_1"], setter.set_pose),
        PedestrianMover("pedestrian_2", PATROL_WEST, SPEEDS["pedestrian_2"], setter.set_pose),
        PedestrianMover("pedestrian_3", PATROL_CROSS, SPEEDS["pedestrian_3"], setter.set_pose),
    ]

    for mover in movers:
        mover.start()
        print(f"[dynamic_actors] Started {mover.name} patrol")

    print("[dynamic_actors] All pedestrians moving. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[dynamic_actors] Stopped.")


if __name__ == "__main__":
    main()
