# Launch flags, build/run, sensors, and troubleshooting

Source of truth: `README.md`, `SIMULATION_HANDOFF.md`, `MODEL_ASSUMPTIONS.md`.

## Build & launch

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select oil_palm_harvester_description --merge-install --symlink-install
source install/setup.bash

ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic
```

Stop every earlier combined launch first (Gazebo Classic uses port `11345`).

## Launch flags

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo client window. |
| `rviz` | `true` | Open RViz (started after an 8 s TimerAction to avoid TF race warnings). |
| `joint_gui` | `true` | Slider GUI. Set `false` when a scripted/autonomous command needs exclusive `/harvester/joint_commands`. |
| `boom_plan` | `true` | Read-only boom docking-plan stack. |
| `harvester_collision_mode` | `off` | `off` = stable sensor-dev mode (contact bodies removed); `on` = future physics. |
| `articulation_control_mode` | `kinematic` | `kinematic` = safe; `pid` = legacy diagnostic fallback. |
| `render_mode` | `mesh` | `primitive` only as a graphics-driver fallback. |
| `docking_camera` | `true` | Low-rate docking depth camera; `false` reduces Xavier load. |
| `range_calibration` | `true` | Project the 5 docking ranges into `c_channel_reference`. |
| `camera_lidar_projection` | `false` | Optional cutter-camera/LiDAR RGB overlay. |
| `camera_lidar_view` | `false` | Second RViz camera-frame cloud window (keep false on Xavier). |

Keep `harvester_collision_mode:=off`, `articulation_control_mode:=kinematic`,
`camera_lidar_view:=false`, `camera_lidar_projection:=false` during normal operation.

## The generated SDF world (what the launch actually does)

1. Converts the active URDF to SDF via `gz sdf -p`.
2. Writes a temp world `/tmp/oil_palm_harvester_dynamic_scene.world`.
3. Rewrites mesh URIs to absolute installed `file://` paths.
4. Embeds the harvester directly (avoids Foxy's large-URDF `spawn_entity` race).
5. Includes the static tree at `(8.5, 0, 0)`.
6. Sets a local `GAZEBO_MODEL_DATABASE_URI` (Gazebo 11 GUI otherwise stalls on the online DB).
7. Passes `server_required:=true` (a `gzserver` failure kills the whole launch).

Generated SDF: `<static>false</static>`, `base_link <kinematic>true</kinematic>`, all link gravity
disabled. "Kinematic base" ≠ static robot — it means the plugin owns the base pose so arm commands
can't push the vehicle through ODE.

## Sensor inventory (frozen raw topics)

| Sensor | Raw topic(s) | Mount / config |
|---|---|---|
| Cutter depth camera | `/harvester/platform_camera/depth/{image_raw,camera_info,depth/image_raw,points}` | `platform_depth_camera_optical_frame`, fixed to `cutting_arm_base_link` at `(0.125,0,0.25)`; 640×400 @15 Hz. |
| Docking depth camera | `/harvester/docking_camera/depth/{image_raw,camera_info}` | `front_depth_camera_optical_frame`, on the platform carrier; 320×240 @8 Hz. |
| Mid-360 coverage LiDAR | `/harvester/lidar/raw_points` | `vehicle_lidar_link`, fixed to `cutting_arm_base_link` at `(0,0,0.30)`; 107×64 grid, ±60° H, ~−7°…+52° V, 0.1–40 m, 10 Hz. |
| RViz LiDAR copy | `/harvester/lidar/points` | Zero-stamped/latest-TF; RViz only, never for fusion. |
| 5 docking ranges | `/harvester/{center,left_45,right_45,left_side,right_side}_range` | One-ray, 20 Hz, 0.05–3.0 m, projected into `c_channel_reference`. |
| Cutter-forward range | `/harvester/cutting_tool_left_range` | On `cutting_tool_link`; follows rail/lift/extension/cutter — separate from the 5-sensor estimator. |

Coordinate convention: `base_link` X toward boom/platform, Y left, Z up; C-opening faces +X; each
range sensor uses +X as its measurement direction; camera optical frames follow the ROS optical
convention (Z forward, X right, Y down).

The LiDAR is a regular-grid Gazebo approximation, NOT a vendor-faithful Livox Mid-360 non-repetitive
scan.

## Troubleshooting

- **Gazebo exit 255 / only old tree visible** → another `gzserver` owns port `11345`. Find it
  (`pgrep -af 'gzserver|gzclient'`, `ss -ltnp | rg ':11345'`) and kill only that PID.
- **RViz shows tree as robot / harvester absent** → tree publisher un-namespaced; check
  `ros2 topic info /robot_description -v` (one harvester publisher only).
- **Slider changes RViz but not Gazebo** → `gzserver` didn't start (check the "bridge ready" log line);
  verify `/harvester/joint_commands` publisher is the GUI and `/harvester/joint_states` is the bridge.
- **Robot flies / joints oscillate / ODESliderJoint flood** → relaunch with `harvester_collision_mode:=off
  articulation_control_mode:=kinematic`; never add a physics-rate direct joint-position loop.
- **Meshes don't render** → `render_mode:=primitive` for diagnosis only; normal is `mesh`.

## Manual sensor-mount adjustment points (URDF joint `xyz`/`rpy`)

- Cutter depth camera → `platform_depth_camera_joint` (parent `cutting_arm_base_link`).
- Arm LiDAR → `vehicle_lidar_joint` (parent `cutting_arm_base_link`).
- Front sensor carrier (centre range + docking camera) → `front_sensor_mount_joint`; fine
  per-device tweaks via `center_range_sensor_joint` and `front_depth_camera_joint`.
- Cutter range → `cutting_tool_left_range_sensor_joint` (in `cutting_tool_link`).
