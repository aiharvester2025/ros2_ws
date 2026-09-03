"""Interactive docking orchestrator (4-state FSM).

States: SWEEP -> PLAN -> DOCK -> UNDOCK -> SWEEP ...

  SWEEP  : command the cutting_arm_lift_joint sweep, accumulate world-frame
           LiDAR points, estimate tree height + platform->trunk distance.
  PLAN   : if reachable, hold and display the plan (boom angle, extension);
           otherwise advise "move prime mover" and reset to SWEEP.
  DOCK   : compute joint targets from measured height/distance, raise/level/
           extend the boom to dock autonomously.
  UNDOCK : retract extension, lower boom, level to 0, return lift joint home.

The orchestrator listens for DOCK presses on an out-of-contract ZMQ SUB
endpoint (default tcp://127.0.0.1:5593) and publishes joint commands to
/harvester/joint_commands.  It is the ONLY node in this package that writes
joint commands; the dashboard button only sends a single "dock" press.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState, PointCloud2, Range
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from harvester_boom_plan import kinematics as kin
from harvester_boom_plan import safety as safety_mod

from harvester_dock import live_height_estimator as lhe
from harvester_dock import distance_estimator as dist_est

STATES = ('SWEEP', 'PLAN', 'DOCK', 'UNDOCK')

SENSOR_TOPICS = {
    "center": "/harvester/center_range",
    "left_45": "/harvester/left_45_range",
    "right_45": "/harvester/right_45_range",
    "left_side": "/harvester/left_side_range",
    "right_side": "/harvester/right_side_range",
}


def _pointcloud_to_xyz(msg) -> np.ndarray:
    """Decode a PointCloud2 into an Nx3 float32 array of (x, y, z).

    Foxy's sensor_msgs does not ship the ``point_cloud2`` Python helper, so this
    decodes the fields directly (mirrors the gateway's encoders approach).
    """
    import struct
    fields = {f.name: f for f in msg.fields}
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return np.zeros((0, 3), dtype='<f4')
    fmt = '>' if msg.is_bigendian else '<'
    x_off, y_off, z_off = fields['x'].offset, fields['y'].offset, fields['z'].offset
    x_r = struct.Struct(fmt + 'f').unpack_from
    y_r = struct.Struct(fmt + 'f').unpack_from
    z_r = struct.Struct(fmt + 'f').unpack_from
    point_step = int(msg.point_step)
    row_step = int(msg.row_step)
    payload = bytes(msg.data)
    rows = []
    for row in range(int(msg.height)):
        row_base = row * row_step
        for col in range(int(msg.width)):
            base = row_base + col * point_step
            x = x_r(payload, base + x_off)[0]
            y = y_r(payload, base + y_off)[0]
            z = z_r(payload, base + z_off)[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                rows.append((x, y, z))
    return np.array(rows, dtype='<f4') if rows else np.zeros((0, 3), dtype='<f4')


def _quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Return a 3x3 rotation matrix (list of 3 tuples) for quaternion (x,y,z,w)."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


class DockOrchestrator(Node):
    def __init__(self):
        super().__init__("dock_orchestrator")

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            from ament_index_python.packages import get_package_share_directory
            config_file = str(Path(get_package_share_directory("harvester_dock"))
                              / "config" / "dock.yaml")
        self.config = self._load_config(config_file)

        self.command_endpoint = str(self.config.get("command_endpoint",
                                                    "tcp://127.0.0.1:5593"))
        self.sweep_cfg = self.config.get("sweep", {})
        # Offset applied BELOW the detected crown base (trunk-end) to get the
        # docking height.  For the reference tree (crown base 9.2 m) this gives
        # ~7.2 m, safely below the lowest fronds.  Also the legacy fallback
        # offset (tree_top - offset) when the crown base is undetectable.
        self.docking_offset = float(self.config.get("docking_offset_below_trunk_top_m", 2.0))
        # Canopy-annulus density threshold for crown-base detection.  Must be
        # matched to the live sweep density (5-step nod ~ 100-150 pts/bin), NOT
        # the offline 40-step sweep (~28k pts/bin).
        self.crown_density_threshold = int(self.config.get("crown_density_threshold", 50))
        # Lower bound (world z) for the crown-base density scan: above the
        # harvester self-clutter (~3 m), below the crown base (~9 m).
        self.crown_z_min_m = float(self.config.get("crown_z_min_m", 5.0))
        self.reach_margin = float(self.config.get("reachability_margin_m", 0.2))
        self.home = self.config.get("home", {})

        safety_cfg = self.config.get("safety", {})
        self.safety_cfg = safety_mod.SafetyConfig(
            extend_safe_m=float(safety_cfg.get("extend_safe_m", 0.6)),
            extend_stop_m=float(safety_cfg.get("extend_stop_m", 0.4)),
            align_skew_m=float(safety_cfg.get("align_skew_m", 0.08)),
            docking_gap_m=float(safety_cfg.get("docking_gap_m", 0.18)),
            emergency_m=float(safety_cfg.get("emergency_m", 0.05)),
            stale_receipt_timeout_s=float(safety_cfg.get("stale_receipt_timeout_s", 0.30)),
            centering_tolerance_m=float(safety_cfg.get("centering_tolerance_m", 0.10)),
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/harvester/lidar/points', self._on_lidar, sensor_qos)
        self.cmd_pub = self.create_publisher(JointState, '/harvester/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/harvester/dock/status', 10)
        self.js_sub = self.create_subscription(JointState, '/harvester/joint_states',
                                               self._on_joint_states, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ranges = {name: None for name in SENSOR_TOPICS}
        self.ranges_stamp = {name: 0.0 for name in SENSOR_TOPICS}
        for name, topic in SENSOR_TOPICS.items():
            self.create_subscription(
                Range, topic, lambda m, n=name: self._on_range(n, m), sensor_qos)

        self.state = 'SWEEP'
        self.points = []          # list of Nx3 world-frame arrays
        self.height_est = None
        self.dist_est = None
        self.joint_state = {}
        self._sweep_done = False
        self._dock_started = False
        self._undock_started = False
        self._press_pending = False
        self._last_status = None   # re-published periodically for the dashboard
        # Non-blocking sweep state (driven by _tick, not a nested spin).
        self._sweep_lifts = []
        self._sweep_step = 0
        self._sweep_phase = 'idle'     # idle -> move -> dwell
        self._sweep_deadline = 0.0

        # ZMQ command listener thread.
        self._zmq_running = True
        self._zmq_thread = threading.Thread(target=self._zmq_loop, daemon=True)
        self._zmq_thread.start()

        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status_periodic)

        self.get_logger().info(
            f"dock_orchestrator ready: command endpoint {self.command_endpoint}")

    # ------------------------------------------------------------------ config
    @staticmethod
    def _load_config(path: str) -> dict:
        import yaml
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    # ------------------------------------------------------------------ ZMQ
    def _zmq_loop(self):
        import zmq
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.command_endpoint)
        sock.setsockopt_string(zmq.SUBSCRIBE, 'v1/operator/dock_request')
        self.get_logger().info(f"ZMQ SUB connected to {self.command_endpoint}")
        while self._zmq_running:
            try:
                parts = sock.recv_multipart(flags=zmq.NOBLOCK)
                if parts:
                    self._on_dock_request(parts)
            except zmq.Again:
                time.sleep(0.05)
            except zmq.ZMQError:
                time.sleep(0.1)
        sock.close(0)

    def _on_dock_request(self, parts):
        try:
            # parts: [topic, header_bytes, payload_bytes]
            payload = json.loads(parts[-1].decode('utf-8'))
            if payload.get('action') == 'dock':
                self._press_pending = True
                self.get_logger().info("DOCK press received")
        except Exception as exc:
            self.get_logger().warn(f"bad dock request: {exc}")

    # ------------------------------------------------------------------ callbacks
    def _on_lidar(self, msg):
        try:
            # Only accumulate during the DWELL phase, when the cutting arm is
            # STATIONARY at its step angle.  Points captured while the arm is
            # moving (move/settle) are in a changing sensor frame and would
            # smear the cloud when leveled.
            if self._sweep_phase != 'dwell':
                return
            xyz = _pointcloud_to_xyz(msg)
            if len(xyz) == 0:
                return
            # Level at receipt: the /harvester/lidar/points topic is
            # zero-stamped, so Time() resolves to the CURRENT (dwell-stationary)
            # arm pose.  Each scan is registered in its own correct world frame.
            leveled = self._level_scan(xyz)
            if leveled is not None and len(leveled):
                self.points.append(leveled)
        except Exception:
            pass

    def _level_scan(self, xyz: np.ndarray) -> Optional[np.ndarray]:
        """Transform one sensor-frame scan to world using the latest TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', 'vehicle_lidar_link', rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        r = _quaternion_to_rotation_matrix(
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w)
        out = np.empty_like(xyz)
        out[:, 0] = r[0][0] * xyz[:, 0] + r[0][1] * xyz[:, 1] + r[0][2] * xyz[:, 2] + t.x
        out[:, 1] = r[1][0] * xyz[:, 0] + r[1][1] * xyz[:, 1] + r[1][2] * xyz[:, 2] + t.y
        out[:, 2] = r[2][0] * xyz[:, 0] + r[2][1] * xyz[:, 1] + r[2][2] * xyz[:, 2] + t.z
        return out

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_state[name] = float(msg.position[i])

    def _on_range(self, name, msg):
        value = float(msg.range)
        if math.isfinite(value) and (float(msg.max_range) <= 0.0 or value <= float(msg.max_range)):
            self.ranges[name] = value
            self.ranges_stamp[name] = time.monotonic()
        else:
            self.ranges[name] = None

    # ------------------------------------------------------------------ helpers
    def _publish_joints(self, mapping):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(mapping.keys())
        msg.position = [float(v) for v in mapping.values()]
        self.cmd_pub.publish(msg)

    def _publish_status(self, state, plan=None):
        data = {'state': state, 'ts': time.time()}
        if plan:
            data.update(plan)
        self._last_status = data
        m = String()
        m.data = json.dumps(data)
        self.status_pub.publish(m)

    def _publish_status_periodic(self):
        """Re-publish the last status so the dashboard reliably shows the plan."""
        if self._last_status is not None:
            m = String()
            m.data = json.dumps(self._last_status)
            self.status_pub.publish(m)

    def _joint(self, name):
        return float(self.joint_state.get(name, 0.0))

    def _base_x(self):
        # No TF dependency: base_link is fixed at world (0, 0, 0.05) in the
        # reference simulation; the operator repositions the vehicle out-of-band.
        return 0.0

    # ------------------------------------------------------------------ sweep
    def _start_sweep(self):
        """Begin a non-blocking nod sweep (arm lift through the trunk band)."""
        joint = self.sweep_cfg.get('joint', 'cutting_arm_lift_joint')
        lo = float(self.sweep_cfg.get('lift_min', -0.25))
        hi = float(self.sweep_cfg.get('lift_max', 0.05))
        steps = int(self.sweep_cfg.get('steps', 5))
        self._sweep_lifts = list(np.linspace(lo, hi, steps))
        self._sweep_step = 0
        self._sweep_phase = 'move'
        self._sweep_deadline = 0.0
        self.get_logger().info(
            f"Sweeping {joint} from {lo} to {hi} in {steps} steps (non-blocking)")

    def _sweep_tick(self):
        """Advance the nod sweep one phase per timer tick.

        Each step: (1) command the lift angle, (2) dwell ``lidar_wait_s`` seconds
        so the 10 Hz LiDAR accumulates several scans at that angle (density
        stabilises the trunk-top ``max``), then advance.  The final
        (canopy-reaching) step dwells ``final_step_dwell_s`` instead, so the
        crown-base / frond / FFB / trunk-end transition is captured densely.
        """
        if self._sweep_phase == 'idle' or self._sweep_done:
            return
        joint = self.sweep_cfg.get('joint', 'cutting_arm_lift_joint')
        step_wait = float(self.sweep_cfg.get('step_wait_s', 2.0))
        dwell = float(self.sweep_cfg.get('lidar_wait_s', 3.0))
        # Extended dwell on the FINAL step: the last lift angle reaches the
        # canopy/crown, where the trunk-end (crown base) density jump is
        # measured.  Holding longer there lets the Mid-360's non-repetitive
        # scanning accumulate more frond/FFB/trunk-end points in the crown-base
        # histogram bin, sharpening the crown-base detection.
        final_dwell = float(self.sweep_cfg.get('final_step_dwell_s', dwell))
        is_last = (self._sweep_step + 1 >= len(self._sweep_lifts))
        dwell = final_dwell if is_last else dwell
        now = time.time()

        if self._sweep_phase == 'move':
            lift = self._sweep_lifts[self._sweep_step]
            self._publish_joints({joint: float(lift)})
            self._sweep_phase = 'settle'
            self._sweep_deadline = now + step_wait
        elif self._sweep_phase == 'settle':
            if now >= self._sweep_deadline:
                self._sweep_phase = 'dwell'
                self._sweep_deadline = now + dwell
        elif self._sweep_phase == 'dwell':
            if now >= self._sweep_deadline:
                if self._sweep_step + 1 < len(self._sweep_lifts):
                    self._sweep_step += 1
                    self._sweep_phase = 'move'
                else:
                    self._sweep_done = True
                    self._sweep_phase = 'idle'
                    self.get_logger().info(
                        f"Sweep complete: {len(self.points)} scans accumulated")
                    self._estimate()
                    self.state = 'PLAN'
                    self._evaluate_plan()

    # ------------------------------------------------------------------ tick
    def _tick(self):
        if self._press_pending:
            self._press_pending = False
            self._advance()

        if self.state == 'SWEEP':
            self._sweep_tick()
        elif self.state == 'DOCK':
            self._dock_tick()
        elif self.state == 'UNDOCK':
            self._undock_tick()

    def _advance(self):
        if self.state == 'SWEEP':
            # Begin (or re-begin) the sweep + estimation (non-blocking).
            self.points = []
            self._sweep_done = False
            self._start_sweep()
        elif self.state == 'PLAN':
            self.state = 'DOCK'
            self._dock_started = False
            self._dock_tick()
        elif self.state == 'DOCK':
            self.state = 'UNDOCK'
            self._undock_started = False
            self._undock_tick()
        elif self.state == 'UNDOCK':
            self.state = 'SWEEP'
            self._sweep_done = False
            self._publish_status('SWEEP', {'note': 'reset'})

    def _estimate(self):
        if not self.points:
            self.height_est = lhe.HeightEstimate(0.0, (8.5, 0.0), 0.0, 0,
                                                 valid=False, reason='no lidar')
            self.dist_est = None
            return
        # Points are already world-leveled per-scan at capture time (see
        # _on_lidar/_level_scan), so no end-of-sweep re-level is needed.
        merged = np.vstack(self.points)
        if len(merged) == 0:
            self.height_est = lhe.HeightEstimate(0.0, (8.5, 0.0), 0.0, 0,
                                                  valid=False, reason='no leveled cloud')
            self.dist_est = None
            return
        self.height_est = lhe.estimate_height(
            merged, world_frame=True,
            crown_density_threshold=self.crown_density_threshold,
            crown_z_min_m=self.crown_z_min_m)
        if not self.height_est.valid:
            x, y, z = merged[:, 0], merged[:, 1], merged[:, 2]
            band = (z >= 1.0) & (z <= 8.0) & (np.abs(y) <= 2.0)
            self.get_logger().warn(
                f"height invalid: reason={self.height_est.reason!r} "
                f"n_total={len(merged)} z_range=[{z.min():.1f},{z.max():.1f}] "
                f"band_n={int(band.sum())} y_range=[{y.min():.1f},{y.max():.1f}] "
                f"x_range=[{x.min():.1f},{x.max():.1f}]")
        if self.height_est.valid:
            self.dist_est = dist_est.estimate_distance(
                self.height_est.trunk_axis_xy_m, self._base_x(), self.reach_margin)
        else:
            self.dist_est = None

    def _docking_height(self):
        """Return (docking_height_m, source, valid) for the boom IK target.

        The docking height must be derived from the CROWN BASE (the trunk-end,
        where fronds/FFBs first appear), NOT from ``tree_top - offset``.  The
        tree top (12 m) sits well inside the canopy; ``tree_top - 2.0 = 10.0 m``
        lands in the frond/FFB zone (fronds at 9.45 m, FFBs at 9.55 m), which is
        exactly why the platform crashed into them.

        Preferred: ``crown_base - docking_offset`` — grips clean trunk 2.0 m
        below the trunk-end.  For the reference tree (crown base 9.2 m) this is
        ~7.2 m, safely below the lowest fronds.  Falls back to the legacy
        ``tree_top - offset`` (flagged invalid) only when the crown base is
        undetectable.
        """
        if (self.height_est is not None
                and self.height_est.trunk_end_valid
                and self.height_est.crown_base_m > 0.0):
            h = self.height_est.crown_base_m - self.docking_offset
            return h, 'crown_base', True
        # Fallback: legacy behaviour, flagged unsafe.
        h = self.height_est.height_m - self.docking_offset
        return h, 'tree_top_offset', False

    def _evaluate_plan(self):
        if self.dist_est is not None and self.dist_est.reachable:
            h = self.height_est.height_m
            h_dock, dock_src, dock_ok = self._docking_height()
            # Refuse to plan at the unsafe tree_top-offset fallback (the whole
            # point of this fix: never grip inside the frond/FFB zone).
            if not dock_ok:
                self.get_logger().warn(
                    f"PLAN REFUSED: crown base undetected; unsafe fallback "
                    f"docking height {h_dock:.2f} m would land in the canopy")
                self._publish_status('PLAN', {
                    'reachable': False,
                    'note': 'crown_base_undetected',
                    'crown_base_reason': self.height_est.trunk_end_reason,
                })
                self.state = 'SWEEP'
                self._sweep_done = False
                return
            try:
                ik = kin.inverse_kinematics(self.dist_est.d_horiz_m, h_dock, 'active')
                if not ik.feasible:
                    self.get_logger().warn(
                        f"PLAN REFUSED: infeasible IK: {ik.infeasible_reason}")
                    self._publish_status('PLAN', {
                        'reachable': False,
                        'note': 'infeasible_dock',
                        'reason': ik.infeasible_reason,
                    })
                    self.state = 'SWEEP'
                    self._sweep_done = False
                    return
                plan = {
                    'reachable': True,
                    'tree_height_m': h,
                    'crown_base_m': self.height_est.crown_base_m,
                    'docking_height_m': h_dock,
                    'docking_height_source': dock_src,
                    'docking_height_valid': dock_ok,
                    'distance_m': self.dist_est.d_horiz_m,
                    'boom_angle_deg': math.degrees(ik.theta_b_rad),
                    'boom_extension_m': ik.extension_total_m,
                    'leveling_angle_deg': math.degrees(ik.leveling_angle_rad),
                }
                self._publish_status('PLAN', plan)
                self.get_logger().info(f"PLAN: {json.dumps(plan)}")
            except kin.InfeasibleDock as exc:
                self._publish_status('PLAN', {'reachable': False,
                                              'note': str(exc)})
                self.state = 'SWEEP'
                self._sweep_done = False
        else:
            note = (self.dist_est.note if self.dist_est is not None
                    else 'height estimate invalid')
            if self.height_est is not None:
                note = note + f" (reason: {self.height_est.reason})"
            self.get_logger().warn(f"NOT REACHABLE: {note}")
            self._publish_status('PLAN', {
                'reachable': False,
                'note': note,
                'needed_advance_m': (self.dist_est.needed_advance_m
                                     if self.dist_est is not None else None),
            })
            self.get_logger().warn(f"NOT REACHABLE: {note}")
            self.state = 'SWEEP'
            self._sweep_done = False

    # ------------------------------------------------------------------ dock
    def _dock_tick(self):
        if self.height_est is None or not self.height_est.valid or self.dist_est is None:
            self._publish_status('DOCK', {'note': 'no valid plan'})
            return
        h_dock, dock_src, dock_ok = self._docking_height()
        # Refuse to dock on the unsafe tree_top-offset fallback (the whole point
        # of this fix: never grip inside the frond/FFB zone).
        if not dock_ok:
            self.get_logger().warn(
                f"DOCK REFUSED: crown base undetected, unsafe fallback "
                f"docking height {h_dock:.2f} m (would land in the canopy)")
            self._publish_status('DOCK', {
                'note': 'crown_base_undetected',
                'docking_height_source': dock_src,
                'crown_base_reason': self.height_est.trunk_end_reason,
            })
            return
        try:
            ik = kin.inverse_kinematics(self.dist_est.d_horiz_m, h_dock, 'active')
        except kin.InfeasibleDock as exc:
            self._publish_status('DOCK', {'note': str(exc)})
            return

        # The IK can return feasible=False (e.g. theta_b outside the elevation
        # limit when the docking height is below the boom pivot).  Guard against
        # indexing an empty extension_per_stage_m in that case.
        if not ik.feasible:
            self.get_logger().warn(
                f"DOCK REFUSED: infeasible IK: {ik.infeasible_reason}")
            self._publish_status('DOCK', {
                'note': 'infeasible_dock',
                'reason': ik.infeasible_reason,
                'docking_height_m': h_dock,
            })
            return

        # Safety: refuse to keep pushing on near-contact.
        now = time.monotonic()
        present = [self.ranges[n] for n in SENSOR_TOPICS if self.ranges[n] is not None]
        if present and min(present) <= self.safety_cfg.emergency_m:
            self.get_logger().warn("EMERGENCY_STOP: near-contact; halting dock")
            self._publish_status('DOCK', {'note': 'emergency_stop'})
            return

        targets = {
            'boom_elevation_joint': ik.theta_b_rad,
            'platform_level_joint': ik.leveling_angle_rad,
            'boom_extension_1_joint': ik.extension_per_stage_m[0],
            'boom_extension_2_joint': ik.extension_per_stage_m[1],
            'boom_extension_3_joint': ik.extension_per_stage_m[2],
            'boom_extension_4_joint': ik.extension_per_stage_m[3],
        }
        self._publish_joints(targets)
        self._dock_started = True
        self._publish_status('DOCK', {
            'boom_angle_deg': math.degrees(ik.theta_b_rad),
            'boom_extension_m': ik.extension_total_m,
            'leveling_angle_deg': math.degrees(ik.leveling_angle_rad),
        })

    # ------------------------------------------------------------------ undock
    def _undock_tick(self):
        home = self.home
        self._publish_joints(home)
        self._undock_started = True
        # Confirm reached home, then reset to SWEEP automatically after a grace.
        if self._at_home():
            self.state = 'SWEEP'
            self._sweep_done = False
            self._publish_status('SWEEP', {'note': 'undocked to home'})
            self.get_logger().info("Undocked to home; reset to SWEEP")

    def _at_home(self):
        for name, value in self.home.items():
            if abs(self._joint(name) - float(value)) > 0.01:
                return False
        return True


def main():
    rclpy.init()
    node = DockOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._zmq_running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
