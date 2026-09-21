"""Simulation-only platform approach driver for safety-guidance validation.

Drives the c-channel platform toward the trunk at a **controllable closing
speed** by ramping the boom joints, and logs the forward gap, derived closing
speed, and the safety-guidance state at each step.  This is the harness that
validates the operator safety HUD against real (simulated) docking motion.

This node is **simulation-only** and separate from the deployed orchestrator
FSM on purpose: it never participates in the real human-in-the-loop flow.  It
publishes joint commands to /harvester/joint_commands (the same topic the
orchestrator and slider GUI use), so it must be run **exclusively** (with the
slider GUI and orchestrator both off).

Usage (after launching Gazebo + gateway):

    python3 -m harvester_dock.approach_driver \
        --speed-cm-s 10 --target-distance-m 0.4 --duration-s 60

Speed profiles for validation:
    slow      ~ 5-10 cm/s
    moderate  ~ 20-30 cm/s
    fast      ~ 60 cm/s (near the rate-limited bridge's capability)
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState, Range

from harvester_boom_plan import kinematics as kin

try:
    from harvester_dashboard.safety_guidance import (
        SafetyConfig, evaluate, default_config_path)
    _HAS_GUIDANCE = True
except ImportError:  # pragma: no cover - dashboard not on PYTHONPATH
    _HAS_GUIDANCE = False


class ApproachDriver(Node):
    def __init__(self, speed_cm_s: float, target_distance_m: float,
                 duration_s: float):
        super().__init__("approach_driver")
        self.speed_cm_s = speed_cm_s
        self.target_distance_m = target_distance_m
        self.duration_s = duration_s

        self.cmd_pub = self.create_publisher(
            JointState, "/harvester/joint_commands", 10)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.range_sub = self.create_subscription(
            Range, "/harvester/center_range", self._on_range, sensor_qos)
        self.js_sub = self.create_subscription(
            JointState, "/harvester/joint_states", self._on_js, 10)

        self.center_range = None
        self.joint_state = {}
        self.last_range_stamp = None

        self.extension_goal_m = 0.0
        self._prev_range = None
        self._prev_t = None

        if _HAS_GUIDANCE:
            self.safety_cfg = SafetyConfig.load(default_config_path())

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"approach_driver: speed={speed_cm_s} cm/s, "
            f"target_distance={target_distance_m} m, duration={duration_s} s")

    def _on_range(self, msg: Range):
        value = float(msg.range)
        if math.isfinite(value) and value <= float(msg.max_range):
            self.center_range = value
            self.last_range_stamp = time.monotonic()

    def _on_js(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.joint_state[name] = float(msg.position[i])

    def _publish_extension(self, ext_m: float) -> None:
        per = kin.split_extension(ext_m)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "boom_extension_1_joint", "boom_extension_2_joint",
            "boom_extension_3_joint", "boom_extension_4_joint",
        ]
        msg.position = list(per)
        self.cmd_pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        # Estimate closing speed from the center range derivative.
        closing_cm_s = None
        if (self.center_range is not None and self._prev_range is not None
                and self._prev_t is not None):
            dt = now - self._prev_t
            if dt > 0.02:
                closing_cm_s = -(self.center_range - self._prev_range) / dt * 100.0
        self._prev_range = self.center_range
        self._prev_t = now

        # Advance the platform toward the trunk at the requested closing speed.
        # Closing speed = -d(range)/dt, so the target gap decreases at speed.
        if self.center_range is not None:
            gap = self.center_range
            if gap > self.target_distance_m:
                # Decrease the gap by speed_cm_s * dt (bounded).
                dt_step = 0.1
                new_gap = max(self.target_distance_m,
                              gap - self.speed_cm_s / 100.0 * dt_step)
                # Convert desired gap reduction to an extension increment:
                # extension moves the c-channel forward toward the trunk.
                # Approximate: d(extension) ~ d(gap) (trunk roughly ahead).
                delta = gap - new_gap
                self.extension_goal_m += delta
                self._publish_extension(self.extension_goal_m)

        # Log + evaluate guidance.
        if _HAS_GUIDANCE and self.center_range is not None:
            stale = (self.last_range_stamp is not None
                     and (now - self.last_range_stamp) > self.safety_cfg.stale_s)
            g = evaluate(closing_cm_s, self.center_range, self.safety_cfg,
                         stale=stale)
            self.get_logger().info(
                f"[guidance] gap={self.center_range:.3f}m "
                f"speed={closing_cm_s if closing_cm_s is not None else float('nan'):.1f}cm/s "
                f"state={g.state} ttc={g.ttc_s if g.ttc_s is not None else float('nan'):.1f}s "
                f"stop={g.stop_distance_m if g.stop_distance_m is not None else float('nan'):.3f}m "
                f"max={g.max_speed_cm_s if g.max_speed_cm_s is not None else float('nan'):.1f}cm/s")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed-cm-s", type=float, default=10.0,
                        help="desired platform closing speed toward the trunk")
    parser.add_argument("--target-distance-m", type=float, default=0.4,
                        help="stop approaching once the gap reaches this")
    parser.add_argument("--duration-s", type=float, default=60.0,
                        help="maximum run duration")
    args = parser.parse_args()

    rclpy.init()
    node = ApproachDriver(args.speed_cm_s, args.target_distance_m,
                          args.duration_s)
    try:
        # Run for the requested duration or until interrupted.
        end = time.monotonic() + args.duration_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
