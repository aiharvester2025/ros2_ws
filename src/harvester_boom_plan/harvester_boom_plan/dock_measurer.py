#!/usr/bin/env python3
"""Measure docking accuracy: compare the c_channel_reference achieved pose
(computed by forward kinematics from measured joint states) against the target
docking point (trunk centre at trunk_top - 2.0 m).

Reads live topics; no joint command is issued.
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener


# URDF constants (mirror kinematics.py; also validated there).
TURRET_ORIGIN = (-1.05, 0.0, 1.02)
ELEVATION_ORIGIN = (0.18, 0.0, 0.74)
BOOM_PIVOT_HEIGHT = TURRET_ORIGIN[2] + ELEVATION_ORIGIN[2]
BOOM_FIXED_LENGTH = 2.40
PLATFORM_TAIL = 1.02
STAGE_COUNT = 4
BASE_LINK_Z_WORLD = 0.05
BOOM_PIVOT_WORLD_Z = BASE_LINK_Z_WORLD + BOOM_PIVOT_HEIGHT


class DockMeasurer(Node):
    def __init__(self):
        super().__init__("dock_measurer")
        self.js = None
        self.dock = None  # (x, y, z) trunk top estimate in world
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, "/harvester/joint_states", self._js, 10)
        self.create_subscription(PointStamped, "/harvester/tree/docking_estimate", self._dock, 10)

    def _js(self, m):
        self.js = m

    def _dock(self, m):
        self.dock = (m.point.x, m.point.y, m.point.z)

    def joint(self, name):
        if self.js is None or name not in self.js.name:
            return 0.0
        return float(self.js.position[self.js.name.index(name)])

    def base_x(self):
        try:
            tf = self.tf_buffer.lookup_transform("world", "base_link", rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return 0.0, 0.0

    def fk(self):
        """c_channel_reference world (x, y, z) from measured joints."""
        theta_b = self.joint("boom_elevation_joint")
        e = sum(self.joint(f"boom_extension_{i}_joint") for i in range(1, 5))
        bx, by = self.base_x()
        # Boom pivot world X (turret yaw ~ 0).
        pivot_x = bx + TURRET_ORIGIN[0] + ELEVATION_ORIGIN[0]
        pivot_z = BOOM_PIVOT_WORLD_Z
        L = BOOM_FIXED_LENGTH + e
        # Platform leveled => body contributes horizontal PLATFORM_TAIL.
        x = pivot_x + L * math.cos(theta_b) + PLATFORM_TAIL
        z = pivot_z + L * math.sin(theta_b)
        return x, by, z


def main():
    rclpy.init()
    n = DockMeasurer()
    # let subscriptions populate
    end = time.time() + 2
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    dock = n.dock or (8.5, 0.0, 12.0)
    target_x, target_y, trunk_top_z = dock
    target_z = trunk_top_z - 2.0  # docking height

    x, y, z = n.fk()
    err = math.hypot(x - target_x, y - target_y, z - target_z)
    h_err = math.hypot(x - target_x, y - target_y)
    z_err = z - target_z

    print("=== DOCKING ACCURACY ===")
    print(f"target c_channel_reference: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) m")
    print(f"achieved c_channel_reference: ({x:.3f}, {y:.3f}, {z:.3f}) m")
    print(f"3D position error: {err:.4f} m")
    print(f"  horizontal error: {h_err:.4f} m")
    print(f"  vertical error:   {z_err:+.4f} m")
    theta_b = n.joint("boom_elevation_joint")
    e = sum(n.joint(f"boom_extension_{i}_joint") for i in range(1, 5))
    print(f"measured boom_angle: {math.degrees(theta_b):.3f} deg, extension: {e:.4f} m")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
