"""Simulation-only cutter approach driver for safety-guidance validation.

Drives the **cutter tip** toward a cutting object at a controllable closing
speed by ramping the cutting-arm joints, and logs the raw range, the
offset-corrected tip clearance, and the cutter safety-guidance state (including
the cut-sequence phase).  This is the harness that validates the operator cutter
HUD against real (simulated) cutting-arm motion.

This node is **simulation-only** and separate from any deployed flow.  It
publishes joint commands to /harvester/joint_commands (the same topic the
docking orchestrator and slider GUI use), so it must be run **exclusively**
(with the slider GUI and any orchestrator both off).

The cutter tip advance is produced by ``cutting_arm_extension_joint`` (prismatic
+X, 0..0.375 m), which moves the tool -- and the tool-mounted range sensor --
toward the object.  (The depth camera and LiDAR are on cutting_arm_base_link and
do NOT follow this extension, so the range sensor is the only clearance input.)

Usage (after launching Gazebo + gateway, with the arm already raised/yawed to
point at the object):

    python3 -m harvester_dock.cutter_approach_driver \
        --speed-cm-s 5 --target-clearance-m 0.15 --duration-s 60

Speed profiles for validation:
    slow      ~ 3-5 cm/s
    moderate  ~ 10-15 cm/s
    fast      ~ 30 cm/s
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState, Range

try:
    from harvester_dashboard.cutter_safety_guidance import (
        CutterConfig, evaluate, next_phase, tip_clearance_m,
        default_config_path)
    _HAS_GUIDANCE = True
except ImportError:  # pragma: no cover - dashboard not on PYTHONPATH
    _HAS_GUIDANCE = False


class CutterApproachDriver(Node):
    def __init__(self, speed_cm_s: float, target_clearance_m: float,
                 duration_s: float):
        super().__init__("cutter_approach_driver")
        self.speed_cm_s = speed_cm_s
        self.target_clearance_m = target_clearance_m
        self.duration_s = duration_s

        self.cmd_pub = self.create_publisher(
            JointState, "/harvester/joint_commands", 10)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.range_sub = self.create_subscription(
            Range, "/harvester/cutting_tool_left_range", self._on_range,
            sensor_qos)
        self.js_sub = self.create_subscription(
            JointState, "/harvester/joint_states", self._on_js, 10)

        self.raw_range = None
        self.last_range_stamp = None
        self.joint_state = {}

        self.extension_goal_m = 0.0
        self._prev_clearance = None
        self._prev_t = None
        self._phase = 'idle'

        if _HAS_GUIDANCE:
            self.safety_cfg = CutterConfig.load(default_config_path())

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"cutter_approach_driver: speed={speed_cm_s} cm/s, "
            f"target_clearance={target_clearance_m} m, duration={duration_s} s")

    def _on_range(self, msg: Range):
        value = float(msg.range)
        if math.isfinite(value) and value <= float(msg.max_range):
            self.raw_range = value
            self.last_range_stamp = time.monotonic()

    def _on_js(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.joint_state[name] = float(msg.position[i])

    def _publish_extension(self, ext_m: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["cutting_arm_extension_joint"]
        msg.position = [max(0.0, min(0.375, ext_m))]
        self.cmd_pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        if not _HAS_GUIDANCE or self.raw_range is None:
            return

        clearance = tip_clearance_m(self.raw_range, self.safety_cfg)

        # Closing speed from the clearance derivative.
        closing_cm_s = None
        if self._prev_clearance is not None and self._prev_t is not None:
            dt = now - self._prev_t
            if dt > 0.02:
                closing_cm_s = -(clearance - self._prev_clearance) / dt * 100.0
        self._prev_clearance = clearance
        self._prev_t = now

        # Advance the cutter toward the object at the requested closing speed.
        if clearance > self.target_clearance_m:
            delta = self.speed_cm_s / 100.0 * 0.1
            self.extension_goal_m += delta
            self._publish_extension(self.extension_goal_m)

        stale = (self.last_range_stamp is not None
                 and (now - self.last_range_stamp) > self.safety_cfg.stale_s)
        self._phase = next_phase(
            self._phase, clearance, closing_cm_s or 0.0, self.safety_cfg,
            stale=stale)
        g = evaluate(closing_cm_s, clearance, self.safety_cfg,
                     stale=stale, phase=self._phase)
        self.get_logger().info(
            f"[cutter] raw={self.raw_range:.3f}m clear={clearance:.3f}m "
            f"speed={closing_cm_s if closing_cm_s is not None else float('nan'):.1f}cm/s "
            f"state={g.state} phase={g.phase} "
            f"stop={g.stop_distance_m if g.stop_distance_m is not None else float('nan'):.3f}m "
            f"max={g.max_speed_cm_s if g.max_speed_cm_s is not None else float('nan'):.1f}cm/s")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed-cm-s", type=float, default=5.0,
                        help="desired cutter closing speed toward the object")
    parser.add_argument("--target-clearance-m", type=float, default=0.15,
                        help="stop approaching once the tip clearance reaches this")
    parser.add_argument("--duration-s", type=float, default=60.0,
                        help="maximum run duration")
    args = parser.parse_args()

    rclpy.init()
    node = CutterApproachDriver(args.speed_cm_s, args.target_clearance_m,
                                args.duration_s)
    try:
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
