# Staging & validation procedure

Migrating from the placeholder to a real URDF must be staged — never overwrite the active baseline in
place. This is the safe order of operations.

## 1. Stage, don't swap

- Copy the real URDF into `urdf/` under a NEW name (e.g. `oil_palm_harvester_real.urdf`), keeping
  `oil_palm_harvester_kinematic.urdf` as the launch default.
- Add the real meshes under `meshes/` (new subdirs, or replace only when the old model is fully
  retired).
- Introduce a **launch flag** (or a parallel launch file) to select the real URDF, defaulting to the
  kinematic baseline, so both can be A/B tested.

## 2. Offline validation (no Gazebo needed)

```bash
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash

# Structural validity
check_urdf oil_palm_harvester_real.urdf

# SDF conversion (this is exactly what the launch does)
gz sdf -p oil_palm_harvester_real.urdf > /tmp/real_model.sdf && echo "sdf ok"

# Joint names survive conversion (must match the plugin's expected set)
grep -o '<joint name="[^"]*"' /tmp/real_model.sdf
```

Confirm the joint-name list matches: `boom_turret_joint`, `boom_elevation_joint`,
`boom_extension_1..4_joint`, `platform_level_joint`, `rail_carriage_joint`,
`cutting_arm_lift_joint`, `cutting_arm_extension_joint`, and root `base_link`.

## 3. Live launch + verification

Launch the real model behind the new flag, then run the standard checklist (also in
`harvester-simulation`):

```bash
gz model -l                                        # oil_palm_harvester + oil_palm_tree listed
ros2 topic info /robot_description -v              # ONE harvester publisher
ros2 topic info /tree/robot_description -v         # tree still namespaced
ros2 topic list | grep -E '/harvester/(joint_commands|joint_states|cmd_vel)'
ros2 run tf2_ros tf2_echo world tree_base          # static
ros2 run tf2_ros tf2_echo world base_link          # live
```

Then the control-path smoke test: move a GUI slider and confirm BOTH Gazebo and RViz harvester move
(proves the plugin found the joints by name). Move the base:

```bash
ros2 topic pub -r 10 /harvester/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

## 4. Sensor re-mount verification

After the geometry swap, confirm every frozen sensor topic still publishes at the expected rate:

```bash
ros2 topic hz /harvester/lidar/raw_points
ros2 topic hz /harvester/center_range
ros2 topic hz /harvester/cutting_tool_left_range
ros2 topic hz /harvester/platform_camera/depth/image_raw
```

And re-run the calibration validators (they assert the calibration JSON still matches the URDF):

```bash
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_range_sensor_calibration.py"
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_camera_lidar_calibration.py"
```

A real model that relocates a sensor will change the calibration transform — update the nominal JSON
mount-derived values (or re-derive them from the URDF) and re-validate; do NOT hand-edit the JSON to
"make it pass" while the URDF is wrong.

## 5. Downstream regression (the skills that depend on this)

The real URDF change propagates to every dependent layer. Re-check:

- `boom-dock-ik` — IK constants are resolved from the URDF at import time;
  `test_urdf_constants_consistent` asserts the literals still match. If geometry changed, update the
  URDF (not the literal) and keep the test green.
- `tree-height-estimation` / `lidar-hud-leveling` — the LiDAR mount (`vehicle_lidar_joint` offset) and
  the leveled/world-frame behavior must still hold.
- `sensor-calibration` — re-derive extrinsics (`t_camera_lidar`, range transforms) from the new URDF.
- `telemetry-gateway` — raw topic names unchanged (they should be), but re-verify the gateway still
  receives each stream.

## 6. Cut over

Only after all of the above pass, switch the launch default to the real URDF and retire the
placeholder. Keep the old kinematic URDF in the repo for reference/history until the new model is
proven stable across multiple sessions.

## Rollback

If the real model destabilizes Gazebo (flight/oscillation), revert to the kinematic baseline flag and
re-investigate inertials/collision mode — do not force the physical model through the kinematic bridge.
