"""Read-only boom docking-plan estimator node.

Subscribes to the tree docking estimate, the trunk-centre estimate, the five
docking ranges, and the cutter range; computes the closed-form boom IK targets
(theta_b, theta_L, extension), and publishes the plan plus a safety state.

This node NEVER publishes a joint command, velocity command, TF transform, or
Gazebo service request.  It is observation/advice only.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from harvester_boom_plan import kinematics as kin
from harvester_boom_plan import safety as safety_mod


SENSOR_TOPICS = {
    "center": "/harvester/center_range",
    "left_45": "/harvester/left_45_range",
    "right_45": "/harvester/right_45_range",
    "left_side": "/harvester/left_side_range",
    "right_side": "/harvester/right_side_range",
}
CUTTER_TOPIC = "/harvester/cutting_tool_left_range"


class BoomPlanNode(Node):
    def __init__(self):
        super().__init__("boom_plan_node")

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            # Default alongside the package share dir.
            from ament_index_python.packages import get_package_share_directory
            config_file = str(Path(get_package_share_directory("harvester_boom_plan"))
                              / "config" / "boom_docking.yaml")
        self.config = self._load_config(config_file)

        self.offset = float(self.config["docking_offset_below_trunk_top_m"])
        self.leveling_mode = str(self.config.get("leveling_mode", "active"))
        self.tree_fallback = tuple(self.config.get("tree_world_xy_fallback_m", [8.5, 0.0]))
        self.trunk_top_gt = float(self.config.get("trunk_top_ground_truth_m", 12.0))

        safety_cfg = self.config.get("safety", {})
        self.safety_cfg = safety_mod.SafetyConfig(
            extend_safe_m=float(safety_cfg["extend_safe_m"]),
            extend_stop_m=float(safety_cfg["extend_stop_m"]),
            align_skew_m=float(safety_cfg["align_skew_m"]),
            docking_gap_m=float(safety_cfg["docking_gap_m"]),
            emergency_m=float(safety_cfg["emergency_m"]),
            stale_receipt_timeout_s=float(safety_cfg["stale_receipt_timeout_s"]),
            centering_tolerance_m=float(safety_cfg["centering_tolerance_m"]),
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tree_dock = None       # (x, y, z) world docking estimate
        self.ranges = {name: None for name in SENSOR_TOPICS}
        self.ranges_stamp = {name: 0.0 for name in SENSOR_TOPICS}
        self.cutter_range = None

        self._sub_tree = self.create_subscription(
            PointStamped, "/harvester/tree/docking_estimate", self._on_tree_dock, 10)
        self._sub_ranges = {
            name: self.create_subscription(Range, topic,
                                           lambda m, n=name: self._on_range(n, m),
                                           sensor_qos)
            for name, topic in SENSOR_TOPICS.items()
        }
        self._sub_cutter = self.create_subscription(
            Range, CUTTER_TOPIC, self._on_cutter, sensor_qos)

        self._pub_plan = self.create_publisher(String, "/harvester/boom/plan", 10)
        self._pub_state = self.create_publisher(String, "/harvester/boom/safety_state", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.2, self._tick)   # 5 Hz

        self.get_logger().info(
            f"boom_plan_node started: offset={self.offset} m below trunk top, "
            f"leveling_mode={self.leveling_mode}")

    # ------------------------------------------------------------------ config
    @staticmethod
    def _load_config(path: str) -> dict:
        import yaml  # noqa: F401  (rclpy env already has PyYAML)
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    # -------------------------------------------------------------- callbacks
    def _on_tree_dock(self, msg: PointStamped):
        self.tree_dock = (msg.point.x, msg.point.y, msg.point.z)

    def _on_range(self, name: str, msg: Range):
        # Reject the Gazebo "no return" sentinel (range > max_range) so an
        # out-of-range sensor does not pollute the centering estimate with
        # FLT_MAX-like values.  A sensor with no valid return is left as None.
        value = float(msg.range)
        max_range = float(msg.max_range)
        if math.isfinite(value) and (max_range <= 0.0 or value <= max_range):
            self.ranges[name] = value
            self.ranges_stamp[name] = time.monotonic()
        else:
            self.ranges[name] = None

    def _on_cutter(self, msg: Range):
        value = float(msg.range)
        max_range = float(msg.max_range)
        if math.isfinite(value) and (max_range <= 0.0 or value <= max_range):
            self.cutter_range = value
        else:
            self.cutter_range = None

    # ------------------------------------------------------------------ tick
    def _tick(self):
        # --- Determine the docking point (world) ----------------------------
        dock_x, dock_y, dock_z = self._docking_point_world()

        # --- Resolve horizontal distance pivot -> trunk ---------------------
        # Use the base_link world pose from TF2 to place the boom pivot.
        d_horiz = self._horizontal_distance(dock_x, dock_y)

        # --- IK --------------------------------------------------------------
        try:
            ik = kin.inverse_kinematics(d_horiz, dock_z, self.leveling_mode)
        except kin.InfeasibleDock as exc:
            self._publish("INFEASIBLE_DOCK_HEIGHT", {
                "error": str(exc),
                "deficit_m": exc.deficit_m,
                "recommended_advance_m": exc.recommended_advance_m,
            })
            return

        # --- Safety ----------------------------------------------------------
        now = time.monotonic()
        stale = any(
            (now - self.ranges_stamp[n]) > self.safety_cfg.stale_receipt_timeout_s
            and self.ranges[n] is not None
            for n in SENSOR_TOPICS
        )
        safety = safety_mod.evaluate(
            self.ranges["center"], self.ranges["left_45"], self.ranges["right_45"],
            self.ranges["left_side"], self.ranges["right_side"], self.cutter_range,
            self.safety_cfg, stale=stale)

        theta_d = kin.docking_lower_angle(ik.theta_b_rad, dock_z, ik.extension_total_m)

        plan = {
            "tree_height_m": dock_z + self.offset,  # reconstructed trunk top
            "docking_height_m": dock_z,
            "docking_offset_below_trunk_top_m": self.offset,
            "trunk_distance_m": d_horiz,
            "boom_angle_rad": ik.theta_b_rad,
            "boom_angle_deg": math.degrees(ik.theta_b_rad),
            "platform_level_rad": ik.platform_level_rad,
            "leveling_angle_rad": ik.leveling_angle_rad,
            "leveling_angle_deg": math.degrees(ik.leveling_angle_rad),
            "boom_extension_total_m": ik.extension_total_m,
            "boom_extension_per_stage_m": ik.extension_per_stage_m,
            "boom_horizontal_distance_m": ik.boom_horizontal_distance_m,
            "docking_lower_angle_rad": theta_d,
            "leveling_mode": self.leveling_mode,
            "phase": safety.phase,
            "centering": safety.centering,
            "feasible": ik.feasible,
            "infeasible_reason": ik.infeasible_reason,
            "notes": safety.notes,
        }
        self._publish(safety.phase, plan)

    def _docking_point_world(self) -> tuple[float, float, float]:
        """Return the docking point (x, y, z) in world coordinates."""
        if self.tree_dock is not None:
            tx, ty, tz = self.tree_dock
            # tree_dock is expected to be the trunk-top estimate in world frame.
            return (tx, ty, tz - self.offset)
        # Fallback: static tree ground truth (for running without the estimator).
        tx, ty = self.tree_fallback
        return (tx, ty, self.trunk_top_gt - self.offset)

    def _horizontal_distance(self, dock_x: float, dock_y: float) -> float:
        """Horizontal distance from the boom pivot to the docking point (world X)."""
        # Boom pivot world X = base_link.x + turret.x + elevation.x (yaw ~ 0).
        base_x = 0.0
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", "base_link", rclpy.time.Time())
            base_x = tf.transform.translation.x
        except Exception:
            pass
        pivot_x, _ = kin.boom_pivot_world_xy(base_x, 0.0)
        return dock_x - pivot_x

    def _publish(self, phase: str, plan: dict):
        msg = String()
        msg.data = json.dumps(plan)
        self._pub_plan.publish(msg)

        state = String()
        state.data = phase
        self._pub_state.publish(state)


def main():
    rclpy.init()
    node = BoomPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
