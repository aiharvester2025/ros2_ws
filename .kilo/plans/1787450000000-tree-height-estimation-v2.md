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
| Depth camera IMU | added (see IMU section) | `platform_depth_camera_imu` |
| LiDAR IMU | added (see IMU section) | `vehicle_lidar_imu` |
| Arm pitch/roll/yaw encoders | **no** (real) | docs: "no joint encoders" |

**Decision:** The estimator is **encoder-free** — it never depends on arm joint
encoders. It must also work **without IMUs** (Strategy 1). The sensor IMUs were
subsequently added to Gazebo so a second, **IMU-assisted** strategy (Strategy 2)
could be A/B-tested against the encoder-free path. In simulation,
`/harvester/joint_states` provides the measured lift angle; we use it **only as
ground truth for validation**, not as a required input to the algorithm.

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

> **Note (outcome):** this sub-estimate did **not** work robustly with the sparse
> recordings (see "What did NOT work" below); it was kept in the design but
> demoted to diagnostic-only. The method that actually shipped is Sub-estimate B′
> below.

### Sub-estimate B′ — trunk top via tight cylinder (the implementation that shipped)

The trunk is a solid vertical cylinder, so its top is simply the **highest point
within a tight cylinder about the estimated axis**. Fronds/FFBs attach at the
crown base and extend *outward*, not above the trunk top, so a tight radius
(`r < 0.35 m`) rejects the frond canopy that biases the legacy estimate.

```
trunk_axis   = median XY of points in a mid-trunk band (z in [1, 8] m)
r            = hypot(x - axis_x, y - axis_y)
trunk_top    = max( z  where r < 0.35 m )          # primary
trunk_top_p  = percentile(z[r < 0.35 m], 99.9)     # robust variant
```

This is the estimate that actually produced the validated results (see outcome
section). It is fully encoder-free: the axis is *measured*, not hard-coded to
`world (8.5, 0)`.

### Sub-estimate C — crown-base density transition (existing method)

Already implemented in `analyze_tree_scan.py`: first histogram bin where the
canopy annulus exceeds a 5000-point threshold. Retain as the third estimate.

### Fusion

The shipped fusion uses the tight-cylinder trunk-top estimates (B′), with the
crown-base transition (C) as a cross-check. The taper-fit base (B) is computed
only as a diagnostic and is **not** included in the fused value because it proved
unstable.

```
H_tree = median of { B′: trunk_top (tight-cylinder max),
                     B′: trunk_top_p (tight-cylinder 99.9 pct),
                     legacy: canopy_top (99 pct) }          # for uncertainty only
```

plus a reported uncertainty = spread of the estimates. In world-frame recordings
the base is world `z = 0`, so height = trunk top directly; the taper-fit base
would be needed only for a sensor-frame recording.

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

## IMU sensors in Gazebo (implemented)

Two IMU sensors were added so the IMU-assisted strategy could be A/B-tested
against the encoder-free path:

- `vehicle_lidar_imu` on `vehicle_lidar_link` → `/harvester/lidar/imu`
- `platform_depth_camera_imu` on `platform_depth_camera_link` →
  `/harvester/platform_camera/imu`

Both use `libgazebo_ros_imu_sensor.so` and are transported through the gateway
as canonical `v1/imu/lidar` and `v1/imu/camera` channels (see
`docs/TELEMETRY_HANDOFF.md`). The IMU is rigid on `cutting_arm_base_link` (child
of the un-instrumented `cutting_arm_lift_joint`), so its gravity-referenced
orientation measures the lift pitch directly: `theta_lift = -2 * asin(orientation.y)`.

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

## Implementation outcome (2026-09-01 → 2026-09-02)

Implemented `analyze_tree_scan_v2.py` with both strategies and validated them
against two recordings.

### What worked (encoder-free, Strategy 1)

- **Trunk axis auto-fit**: median XY in a mid-trunk band → (8.35, 0) on
  `tree_scan_001`, (8.28, 0) on `tree_scan_002` (true 8.5, 0) — encoder-free.
- **Trunk top via tight cylinder** (`r < 0.35 m`): the trunk is a vertical
  cylinder, so its top is the highest point within the tight cylinder (fronds
  extend *outward*, not above it). This removes the frond-tip bias of the legacy
  canopy-99th-percentile estimate.
- **Crown base density transition**: ~9.0–9.25 m (9.2 m ground truth).

### What did NOT work (honest finding)

- **Trunk-taper base fit** (Sub-estimate B) is **not robust** with this sparse
  recording. The trunk taper is subtle (0.70 m → 0.50 m over 12 m = 0.2 m
  diameter change), and the radial mode of each height band is noisy
  (~0.1–0.5 m scatter), so linear extrapolation to the 0.70 m base diameter is
  numerically unstable (returned a nonsensical −13.7 m base). Demoted to
  diagnostic-only; the fused result uses the world-frame trunk top directly.

### IMU-assisted path (Strategy 2) — completed

IMUs were added to the URDF and telemetry pipeline, and a fresh sweep
(`tree_scan_002`) was recorded with them (5,596 IMU samples per channel, 1,029
world-frame clouds). `analyze_imu_assisted()` now:

- recovers the lift pitch from the LiDAR IMU (`theta_lift = -2*asin(orientation.y)`),
  confirming the IMU closes the un-instrumented `cutting_arm_lift_joint`
  (recovered range −0.349 .. +1.016 rad matches the −0.35 .. +1.05 limits);
- confirms the sweep reached the canopy;
- reports an IMU-assisted trunk-top estimate (global max over the canopy-reaching
  sweep) and its agreement with the encoder-free fused height.

### Final A/B results (against 12.0 m ground truth)

| Strategy | Recording | Height | Error |
|---|---|---|---|
| Encoder-free (fused) | tree_scan_002 | 11.83 m | −0.17 m (−1.4%) |
| IMU-assisted (global max) | tree_scan_002 | 11.927 m | −0.07 m (−0.6%) |
| Legacy (canopy 99th pct) | tree_scan_002 | 12.22 m | +0.22 m (+1.8%) |
| Encoder-free (fused) | tree_scan_001 | 11.90 m | −0.10 m (−0.8%) |

The two new strategies agree to within ~0.1 m and both beat the legacy method.
On real hardware (sensor-frame clouds) the IMU would be *required* to level the
cloud; here the recording is already world-registered, so both strategies measure
the same physical quantity.

### Rationale for the trunk-top-tight-cylinder win

Fronds/FFBs attach at the crown base (9.2 m) and extend *outward*; within a tight
cylinder about the axis (r < 0.35 m) they do not exceed the trunk top (12.0 m).
The legacy 99th-percentile canopy estimate includes frond tips and reads ~12.22–
12.26 m. Restricting to the trunk cylinder removes that bias.
