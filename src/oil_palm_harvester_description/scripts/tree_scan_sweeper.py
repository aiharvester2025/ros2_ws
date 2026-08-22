#!/usr/bin/env python3
"""Sweep ONLY the cutting_arm_lift_joint to scan ground then up to canopy.

Publishes JointState messages that contain ONLY ``cutting_arm_lift_joint`` so
the Gazebo kinematic bridge changes only that one joint and leaves every other
joint at its current position.  No joint_states feedback is required.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import time
import numpy as np


class LiftOnlySweeper(Node):
    def __init__(self):
        super().__init__('lift_only_sweeper')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.cmd_pub = self.create_publisher(JointState, '/harvester/joint_commands', 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/harvester/lidar/raw_points', self._lidar_cb, sensor_qos)

        # Only this one joint is ever commanded.
        self.sweep_joint = 'cutting_arm_lift_joint'
        self.lift_min = -0.35   # down (ground)
        self.lift_max = 1.05    # up (canopy)
        self.steps = 40
        self.step_wait = 2.0    # seconds to let the rate-limited bridge move
        self.lidar_wait = 0.8   # seconds to capture a fresh LiDAR scan

        self.lidar_msgs = []

    def _lidar_cb(self, msg):
        self.lidar_msgs.append(msg)

    def spin_for(self, duration):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_lift(self, lift):
        # Message contains ONLY the sweep joint, so the bridge leaves every
        # other joint untouched.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.sweep_joint]
        msg.position = [float(lift)]
        self.cmd_pub.publish(msg)

    def run(self):
        self.get_logger().info(
            f'Sweeping ONLY {self.sweep_joint} from {self.lift_min:.3f} (ground) '
            f'up to {self.lift_max:.3f} (canopy) in {self.steps} steps. '
            f'All other joints are left unchanged.')

        lifts = np.linspace(self.lift_min, self.lift_max, self.steps)
        for i, lift in enumerate(lifts):
            self.get_logger().info(f'Step {i+1}/{self.steps}: lift={lift:.3f}')
            self.publish_lift(lift)
            # Wait for the rate-limited bridge to move the joint, then a fresh
            # LiDAR scan (10 Hz).
            self.spin_for(self.step_wait)
            self.lidar_msgs.clear()
            self.spin_for(self.lidar_wait)
            if self.lidar_msgs:
                self.get_logger().info(
                    f'  captured {len(self.lidar_msgs)} LiDAR msgs '
                    f'({len(self.lidar_msgs[0].data)} bytes)')
            else:
                self.get_logger().warn('  no LiDAR msg captured')

        self.get_logger().info('Lift-only sweep complete.')


def main():
    rclpy.init()
    node = LiftOnlySweeper()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
