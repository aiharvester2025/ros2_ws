# The frame model + root cause

## Three frames the pipeline can emit

1. **Sensor frame** (`vehicle_lidar_link`): raw points as the LiDAR sees them, rotating with the arm.
2. **World frame** (`world`): full `T_world_sensor` applied — rotation AND translation, tree at its
   absolute world position (x≈8.5).
3. **Leveled sensor frame**: rotation-only (`R_world_sensor`), translation dropped — LiDAR stays at
   origin, points gravity-aligned. This is what the sensor-relative HUD needs.

## Root cause of "tree bending"

The HUD rendered points directly in the **sensor frame** and labeled the side-view axis "+z up" as if
it were world-up. When the arm/platform pitches:
- Pitching **up** tips the sensor's +z backward relative to world → world-vertical tree appears to
  "bend backward."
- Pitching **down** brings sensor +z back toward world-vertical → tree "straightens."

The tree never moves; the frame of reference rotates and the view is anchored to the sensor.

## The fix (and its follow-on bug)

Fix = transform points into a fixed frame before rendering.

- **First attempt: full world transform** (`T_world_sensor`). This fixed orientation but moved points
  to absolute world coords, so the tree (at x≈8.5 m) sat far from the HUD's blue-square origin
  (vehicle origin), near the ±8 m window edge, clipping the left/right/iso views.
- **Correct fix: rotation-only ("leveled sensor frame")**. `lidar_level_translation: false` (zero
  translation). Result: tree stands vertical AND stays ~1 m from the LiDAR origin, matching the
  cutter camera distance; all views fit the window.

## Stamped vs latest TF

Use the transform at the **message's own stamp** (`lookup_transform(target, source, stamp)`) when the
sensor and TF share a clock domain. Using "latest" for a buffered/stale cloud re-rotates already
captured points through the wrong pose → smearing/leaning.

In this sim, Gazebo stamps the raw cloud in **sim time** but `robot_state_publisher` publishes TF in
**wall time** (the launch file documents this mismatch). Hence `lidar_transform_latest: true` is the
correct default here (same convention as the existing `lidar_timestamp_bridge.py`).

## Diagnosis checklist (in order)

1. Is the cloud published in `lidar_link`? (check `frame_id` AND point coordinates).
2. Is there a live TF `lidar_link → map/world`? (`tf2_echo world vehicle_lidar_link` while moving —
   should change smoothly).
3. Is the HUD transforming with stamped time, or reusing a fixed transform?
4. Does the HUD accumulate scans then render? Are points stored in world frame at their own
   timestamps?

## Code shape

```python
# WRONG: apply current (moving) pose to the whole accumulated cloud
cloud_map = tf.apply(current_lidar_pose, accumulated_cloud_in_sensor)

# RIGHT: level each scan (rotation-only) at capture; accumulated cloud is already leveled
cloud_leveled = rotate_only(current_orientation, accumulated_cloud)   # LiDAR stays at origin
```

The gateway's `_lidar_in_world` implements this; the shared pure helpers
`quaternion_to_rotation_matrix()` and `rotate_point()` live in `encoders.py` (extracted so they're
unit-testable without a ROS node). The test `test/test_lidar_transform.py` asserts a world-vertical
point stays vertical through a sensor-pitch transform.
