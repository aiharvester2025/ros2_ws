"""Opt-in simulation experiment: execute the boom docking plan.

This script is SEPARATE from the read-only ``boom_plan_node``.  It subscribes to
the published plan, validates it, and then writes joint targets to
``/harvester/joint_commands`` so the docking maneuver can be demonstrated and
its accuracy measured in Gazebo/RViz.

It is gated by ``--execute``: without that flag it only prints the plan it would
execute (dry run), so nothing is commanded accidentally.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class BoomDockingDemo(Node):
    def __init__(self, execute: bool):
        super().__init__("boom_docking_demo")
        self.execute = execute
        self.latest = None
        self.cmd_pub = self.create_publisher(JointState, "/harvester/joint_commands", 10)
        self.create_subscription(String, "/harvester/boom/plan", self._on_plan, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            "boom_docking_demo started (execute=%s)" % execute)

    def _on_plan(self, msg: String):
        try:
            self.latest = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning(f"bad plan JSON: {exc}")

    def _tick(self):
        if self.latest is None:
            return
        plan = self.latest
        if not plan.get("feasible", True):
            self.get_logger().warn(
                f"plan infeasible: {plan.get('infeasible_reason')}; skipping")
            return

        # Two-phase safety:
        #   Phase A (raise + extend to docking height) is open-loop IK: the
        #     range sensors are legitimately out of range, so WAITING_FOR_TREE /
        #     READY / EXTEND_OK do NOT block.  Only hard-safety phases block.
        #   Phase B (final centring / lower) is range-gated; once the platform
        #     is near the trunk the sensor phases (ALIGN_OK/LEVEL_OK/DOCKING_OK)
        #     become meaningful and HOLD_AT_DISTANCE/EMERGENCY_STOP gate motion.
        phase = plan.get("phase", "READY")
        hard_block = {"EMERGENCY_STOP", "INFEASIBLE_DOCK_HEIGHT"}
        if phase in hard_block:
            self.get_logger().warn(
                f"plan phase '{phase}' blocks execution; skipping "
                f"(notes: {plan.get('notes')})")
            return

        theta_b = plan["boom_angle_rad"]
        level = plan["leveling_angle_rad"]
        per_stage = plan["boom_extension_per_stage_m"]

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = [
            "boom_elevation_joint",
            "platform_level_joint",
            "boom_extension_1_joint",
            "boom_extension_2_joint",
            "boom_extension_3_joint",
            "boom_extension_4_joint",
        ]
        cmd.position = [
            theta_b, level,
            per_stage[0], per_stage[1], per_stage[2], per_stage[3],
        ]

        if self.execute:
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"EXECUTED boom plan: theta_b={plan['boom_angle_deg']:.2f}deg, "
                f"level={plan['leveling_angle_deg']:.2f}deg, "
                f"ext={plan['boom_extension_total_m']:.3f}m "
                f"(phase={plan['phase']})")
        else:
            self.get_logger().info(
                f"[dry-run] would command theta_b={plan['boom_angle_deg']:.2f}deg, "
                f"level={plan['leveling_angle_deg']:.2f}deg, "
                f"ext={plan['boom_extension_total_m']:.3f}m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true",
                        help="actually publish joint commands (dry-run otherwise)")
    args = parser.parse_args()

    rclpy.init()
    node = BoomDockingDemo(args.execute)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
