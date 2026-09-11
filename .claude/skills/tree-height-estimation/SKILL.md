---
name: tree-height-estimation
description: "Implement or extend the oil-palm tree height / trunk-end (crown base) estimation from LiDAR point clouds in this ROS2 Foxy workspace, including the encoder-free and IMU-assisted strategies."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Tree Height & Crown-Base Estimation

Estimate oil-palm tree height, trunk-end (crown base), and trunk axis from arm-mounted LiDAR
point clouds, in either the offline `analyze_tree_scan*.py` scripts or the live
`harvester_dock.live_height_estimator` module.

This skill records the **non-obvious sensor/kinematics constraints** of the real machine and the
proven estimation algorithms, so a smaller model doesn't re-derive them or repeat the mistakes.

## When this applies

- Editing `analyze_tree_scan.py` / `analyze_tree_scan_v2.py` or
  `src/harvester_dock/harvester_dock/live_height_estimator.py`.
- Adding a tree-height / trunk-top / crown-base / trunk-axis estimator or a new sensor-fusion path.
- Sweeping the arm (`tree_scan_sweeper.py`) to record a LiDAR scan.

## Core facts (internalize before editing)

<critical>
- **The docking-height rule lives here too:** trunk-end = crown base (9.2 m), NOT `tree_top - offset`.
  `tree_top - 2.0 = 10.0 m` lands inside the fronds (9.45 m) / FFBs (9.55 m) — the exact cause of
  the platform crash. Docking height = `crown_base - 2.0 ≈ 7.2 m`.
- **The real machine's sensor/encoder inventory is asymmetric** (this changed repeatedly during the
  session and is now the settled truth):
  - ✅ Available: boom angle encoder, boom extension encoder, platform pitch/roll (2-axis tilt, no
    yaw), cutter-forward range sensor (0.05–3.0 m), 5 docking range sensors, **IMUs on the OAK
    cameras and the Mid-360 LiDAR**.
  - ❌ Absent: `cutting_arm_lift_joint` encoder, `cutting_arm_extension_joint` encoder, boom turret
    yaw encoder, the 4 prismatic boom-extension encoders (see note), no GPS/absolute pose.
  - The LiDAR/camera sit **downstream of the un-instrumented `cutting_arm_lift_joint`**, so their
    orientation is NOT recoverable from the boom/platform encoders alone — that gap is closed by the
    **sensor IMUs** (orientation only; IMU yaw is unobservable, which is fine because height needs
    only pitch/roll).
- **IMU gives orientation (pitch/roll), never yaw, never absolute position.** Do not use it for
  heading or world-frame registration.
</critical>

## Reference map

- **Estimation algorithms (trunk axis, crown base, taper fit, d_short):** [references/estimation-algorithms.md](references/estimation-algorithms.md)
- **Recording format + sweep procedure:** [references/recording-and-sweep.md](references/recording-and-sweep.md)

## Quick workflow

1. Read `src/harvester_dock/harvester_dock/live_height_estimator.py` and
   `src/harvester_dock/harvester_dock/distance_estimator.py` — they are the **live ROS-free ports**
   of `analyze_tree_scan_v2.py`. Prefer editing the live module over the offline script; keep the
   offline script in sync.

2. Pure modules are pytest-testable without a ROS graph:
   ```bash
   cd ~/ros2_ws
   PYTHONPATH=src/harvester_dock:src/harvester_boom_plan \
     python3 -m pytest src/harvester_dock/test -v
   ```

3. Two strategies are supported and compared (see the estimation-algorithms reference):
   - **Strategy 1 (encoder-free):** trunk-axis fit + trunk-taper base fit + crown-density, no IMU/encoders.
   - **Strategy 2 (IMU-assisted):** `H_L` from boom/platform encoders + LiDAR IMU attitude → `H_L − d_short + r_trunk`.
   The IMU-assisted path activates only when `v1_imu_lidar` recordings exist (the older
   `tree_scan_001` recording predates the IMUs and has none).

## Key invariants

- Crown-base density threshold MUST match sweep density: ~5000 pts/bin for the offline 40-step
  sweep, ~50 for the live 5-step nod. Tune it, don't hard-code 5000 everywhere.
- `crown_z_min_m` is a **fixed** bound (5.0 m), above self-clutter (~2–3 m), below crown base (~9 m).
  Never tie it to the trunk top (`top - 3.5` biases crown base low).
- Trunk-top uses the **`max`** (upper envelope), not a percentile (percentiles are biased low).
- Trunk axis uses a **radius-corrected median** (add `(2/pi)*r`, r≈0.25 m) for the half-cylinder bias.
- When crown-base detection fails, `estimate_height` falls back to `trunk_top - 2.0` **with
  `trunk_end_valid=False`** — callers must refuse to dock on an invalid trunk-end.

## Related skills

- `harvester-simulation` — the LiDAR mount (`vehicle_lidar_link` on `cutting_arm_base_link`) and the
  `cutting_arm_lift_joint` sweep used to capture the scan.
- `boom-dock-ik` — the consumer of the crown-base/trunk-axis estimates (via `_docking_height`).
- `lidar-hud-leveling` — the frame/leveling model that determines whether the recorded cloud is world
  or leveled-sensor frame.
