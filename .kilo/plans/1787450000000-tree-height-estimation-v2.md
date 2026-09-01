# Tree Height Estimation — v2 Implementation Plan

**Date:** 2026-09-01
**Workspace:** `/home/ubuntu/ros2_ws`
**Baseline:** existing `analyze_tree_scan.py` (density-based) + `tree_scan_sweeper.py` (lift sweep)

## Objective

Replace/augment the current single-estimate (canopy 99th percentile) tree height
estimator with an **encoder-free** estimator that:

1. Does **not** assume the tree axis is known (`world (8.5, 0, 0)`).
2. Does **not** assume the LiDAR height `H_L` is known from arm encoders.
3. Estimates tree height from **trunk-relative** LiDAR geometry + the trunk
   distance sensor, cross-checked by three independent sub-estimates.

## Ground truth (unchanged, from `tree_targets.yaml`)

- Total height: 12.0 m (trunk top)
- Crown base: 9.2 m
- Trunk diameter: 0.70 m base -> 0.50 m crown (0.60 m nominal)

## Confirmed constraints (verified against the codebase)

| Signal | Availability | Source |
|---|---|---|
| Measured joint states (incl. `cutting_arm_lift_joint`) | sim only | `/harvester/joint_states` (Gazebo bridge) |
| LiDAR cloud (world-frame) | yes | `v1/lidar/raw` recordings |
| 5 docking ranges + trunk center/diameter | yes | `range_sensor_calibration.py` |
| Trunk distance sensor | yes | `/harvester/cutting_tool_left_range` (0.05–3.0 m) |
| Depth camera IMU | **no** | `libgazebo_ros_camera.so` — no IMU |
| LiDAR IMU | **no** | `libgazebo_ros_ray_sensor.so` (gpu_ray) — no IMU |
| Arm pitch/roll/yaw encoders | **no** (real) | docs: "no joint encoders" |

**Decision:** The sensor IMUs and arm encoders are **not** available, so the
estimator must be encoder-free and IMU-free. In simulation, `/harvester/joint_states`
provides the measured lift angle; we use it **only as ground truth for validation**,
not as a required input to the algorithm.

## New estimator design

### Sub-estimate A — trunk-top crossing via minimum slant range (d_short)

Sweep the lift joint; at each step record the LiDAR cloud (world-frame). For the
trunk cylinder (see axis below), compute the minimum slant range from the LiDAR
origin to any trunk point:

```
d_short = min over steps of ( min ||p - lidar_origin||  for p in trunk points )
step*   = argmin of the above
```

The step where `d_short` is minimum is where the LiDAR passes the trunk-top
horizon. At that step, the LiDAR's height `H_L` equals the trunk top Z (plus a
trunk-half-radius correction, because we hit the surface not the centerline):

```
trunk_top_z = H_L(step*) - d_short + r_trunk
```

`H_L(step*)` is recovered **without an arm encoder** by reading the LiDAR
origin's world-frame Z from the recorded cloud's own header/frame transform
(available because the gateway records world-frame clouds with
`lidar_level_translation: true`). This is the key: the LiDAR origin in the world
frame is observable from the recorded data, not from encoders.

### Sub-estimate B — trunk-taper base fit (encoder-free ground plane)

The trunk tapers linearly `0.70 m -> 0.50 m` over the full height. Fit a line to
the measured trunk diameter vs. height:

```
diameter(z) = D_base - (D_base - D_top) * (z / H)
```

Extrapolate to `diameter = 0.70 m` to find the base (`z = 0`) — this recovers the
ground plane without encoders and without the occluded low LiDAR returns (the
current method's ~1.8 m occlusion problem).

```
base_z = z where diameter(z) = 0.70 m
```

### Sub-estimate C — crown-base density transition (existing method)

Already implemented in `analyze_tree_scan.py`: first histogram bin where the
canopy annulus exceeds a 5000-point threshold. Retain as the third estimate.

### Fusion

```
H_tree = weighted median of { A: trunk_top_z - base_z,
                             B: highest trunk-cylinder z - base_z,
                             C: crown_base + (crown_base - base_z)  # crown span proxy }
```

plus a reported uncertainty = spread of the three.

## Implementation steps

1. **Add trunk-axis estimation (remove `TREE_X/TREE_Y` hard-code).**
   In `analyze_tree_scan.py`, replace the fixed `(8.5, 0)` with a trunk axis
   estimated either (a) from the recorded side-pair docking `trunk_center`, or
   (b) by a RANSAC vertical-cylinder fit on the trunk points themselves. Keep
   `(8.5, 0)` only as the simulation fallback/ground-truth reference.

2. **Add the d_short sweep-crossing estimate (Sub-estimate A).**
   - For each recorded world-frame cloud, compute the LiDAR origin (from the
     header `frame_id` transform or from the gateway's `lidar_world_frame`).
   - Compute `d_short` and `step*`.
   - Compute `trunk_top_z = H_L(step*) - d_short + r_trunk`.

3. **Add the trunk-taper base fit (Sub-estimate B).**
   - For each height band, fit the trunk XY spread -> diameter.
   - Fit `diameter(z)` line; solve `base_z`.

4. **Fuse the three estimates and emit uncertainty.**

5. **Record a new sweep** with `lidar_level_translation: true` so the LiDAR
   origin Z (H_L) is present in the world-frame cloud.

6. **Validate** against `trunk_top_reference = 12.0 m` and `crown_base = 9.2 m`.
   Report the improvement vs. the current 12.26 m (0.26 m error).

## Optional: add sensor IMUs to Gazebo (future work, not required)

If a true sensor-IMU path is desired later, add `<sensor type="imu">` +
`libgazebo_ros_imu_sensor.so` blocks to the LiDAR/camera links in
`oil_palm_harvester_kinematic.urdf`, publish `sensor_msgs/Imu`, and extend the
gateway/contract. This is **out of scope** for the encoder-free estimator, which
must work without IMUs.

## Deliverables

- Updated `analyze_tree_scan.py` (or a new `analyze_tree_scan_v2.py`) with the
  three sub-estimates + fusion.
- A validation report against 12.0 m / 9.2 m ground truth.
- Updated `docs/TREE_HEIGHT_SCAN.md` documenting the new method.

## Validation command

```
cd ~/ros2_ws
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway \
  python3 analyze_tree_scan_v2.py ~/harvester_audits/tree_scan_001
```

## Implementation outcome (2026-09-01)

Implemented `analyze_tree_scan_v2.py` and validated against `tree_scan_001`.

### What worked

- **Trunk axis auto-fit**: median XY in mid-trunk band → (8.35, 0) vs true
  (8.5, 0), 0.15 m error, encoder-free.
- **Trunk top via tight cylinder** (`r < 0.35 m`): max z = 11.903 m, p99.9 =
  11.900 m. Fused height **11.90 m (−0.10 m, −0.8%)** vs legacy **12.26 m
  (+0.26 m, +2.1%)** — ~2.6× better and encoder-free.
- **Crown base density transition**: 9.00 m (unchanged, 9.2 m ground truth).

### What did NOT work (honest finding)

- **Trunk-taper base fit** (Sub-estimate B) is **not robust** with this sparse
  recording. The trunk taper is subtle (0.70 m → 0.50 m over 12 m = 0.2 m
  diameter change), and the radial mode of each height band is noisy
  (~0.1–0.5 m scatter), so linear extrapolation to the 0.70 m base diameter is
  numerically unstable (returned a nonsensical −13.7 m base). Demoted to
  diagnostic-only; the fused result uses the world-frame trunk top directly.

### IMU-assisted path

IMUs were added to the URDF and telemetry pipeline, but the `tree_scan_001`
recording predates them. `analyze_imu_assisted()` currently reports
`no_imu_recordings`. A new sweep with the updated URDF is required to populate
`v1/imu/lidar` and complete the `d_short` crossing estimate.

### Rationale for the trunk-top-tight-cylinder win

Fronds/FFBs attach at the crown base (9.2 m) and extend *outward*; within a tight
cylinder about the axis (r < 0.35 m) they do not exceed the trunk top (12.0 m).
The legacy 99th-percentile canopy estimate includes frond tips and reads ~12.26 m.
Restricting to the trunk cylinder removes that bias.
