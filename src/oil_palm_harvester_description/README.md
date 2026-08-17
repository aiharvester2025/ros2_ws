# Oil-Palm Harvester Gazebo + RViz Sensor Simulation

This package is an estimated oil-palm harvester model derived from supplied
orthographic and isometric drawings. The supported baseline is a ROS 2 Foxy
and Gazebo Classic 11 sensor-development simulation: a movable harvester
approaches one static oil-palm tree while RViz presents the measured robot
state, cameras, LiDAR, range rays, and range-based docking estimate.

It is not a force-accurate vehicle, cutting, or collision simulation. The
articulated model uses bounded kinematic control and its own collision bodies
are disabled by default so the Xavier can run Gazebo, RViz, cameras, and
LiDAR reliably.

## Source-of-truth files

| File | Current role |
|---|---|
| `urdf/oil_palm_harvester_kinematic.urdf` | Active harvester geometry, fixed sensor mounts, and Gazebo sensor blocks. It drives both combined Gazebo and RViz. |
| `launch/gazebo_harvester_and_tree.launch.py` | Supported combined Gazebo + RViz launch. It converts the active URDF to a local dynamic SDF world. |
| `src/harvester_kinematic_gazebo_plugin.cpp` | Joint command/feedback bridge, 20 Hz kinematic articulation, `/harvester/cmd_vel` base motion, and `world -> base_link` TF. |
| `rviz/harvester_tree_combined.rviz` | Single-RViz scene, active camera viewport, LiDAR/range displays, and docking panel. |
| `config/range_sensor_calibration.nominal.json` | Simulation-only five-sensor docking calibration profile. |
| `config/camera_lidar_calibration.nominal.json` | Simulation-only cutter-camera/LiDAR calibration and projection profile. |
| `urdf/oil_palm_harvester_estimated.urdf` | Alternate physical-control reference only; do not substitute it into the combined launch. |

Read [SIMULATION_HANDOFF.md](SIMULATION_HANDOFF.md) before changing the
launch, TF tree, sensor mounts, or controller path. The calibration contracts
are [CALIBRATION_FRAME_CONTRACT.md](CALIBRATION_FRAME_CONTRACT.md) and
[CAMERA_LIDAR_CALIBRATION_CONTRACT.md](CAMERA_LIDAR_CALIBRATION_CONTRACT.md).

## Build

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select oil_palm_harvester_description --merge-install --symlink-install
source install/setup.bash
```

## Start the supported scene

Stop every earlier combined launch before starting another one. Gazebo Classic
normally uses port `11345`, so two launches can leave RViz connected to an old
tree-only world.

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off \
  articulation_control_mode:=kinematic
```

The scene contains one static tree at `world = (8.5, 0, 0)`, one movable
harvester, the joint-state-publisher GUI, and one RViz process with the
**Docking Sensor Values** panel. The optional second RViz camera/LiDAR view
and RGB/LiDAR overlay are disabled by default for the Xavier.

## Controls and state flow

```text
joint_state_publisher_gui
  /harvester/joint_commands
              │
              ▼
  Gazebo harvester bridge (rate-limited kinematic articulation)
              │
              ▼
  measured /harvester/joint_states ──► robot_state_publisher ──► RViz

/harvester/cmd_vel ──► Gazebo bridge ──► movable base + world -> base_link TF
```

The GUI command stream and measured feedback stream are intentionally
different. RViz, camera frames, LiDAR frames, and range frames therefore
follow the actual Gazebo pose instead of an immediate slider target.

`articulation_control_mode:=kinematic` is the normal mode. Every movable
joint is clamped to its URDF limits, rate-limited at 20 Hz using its velocity
limit, and the bridge clears residual physics velocity after each changed pose
batch. The large turret is additionally limited to `0.05 rad/s`.

`articulation_control_mode:=pid` is a legacy diagnostic fallback only. It can
reintroduce force reactions in the long boom/rail/arm chain and is not the
recommended operating mode.

Move the base through `/harvester/cmd_vel`:

```bash
ros2 topic pub -r 10 /harvester/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

The bridge stops base motion 0.5 seconds after the last command.

## Simulated sensors

| Sensor | Primary topics | Mount and configuration |
|---|---|---|
| Cutter depth camera | `/harvester/platform_camera/depth/image_raw`, `.../camera_info`, `.../depth/image_raw`, `.../points` | On `cutting_arm_base_link`; follows rail and arm lift, not cutter extension. 640×400 at 15 Hz. |
| Docking depth camera | `/harvester/docking_camera/depth/image_raw`, `.../camera_info` | On the platform sensor carrier. 320×240 at 8 Hz to protect Xavier resources. |
| Mid-360 coverage approximation | `/harvester/lidar/raw_points` | On `cutting_arm_base_link`; 107×64 GPU-ray grid, −60° to +60° horizontal, approximately −7° to +52° vertical, 0.1–40 m, 10 Hz. |
| RViz LiDAR copy | `/harvester/lidar/points` | Zero-stamped/latest-TF copy for RViz only. Do not use it for time-correlated algorithms. |
| Five docking ranges | `/harvester/{center,left_45,right_45,left_side,right_side}_range` | One-ray 20 Hz sensors, 0.05–3.0 m, rigidly calibrated into `c_channel_reference`. |
| Cutter-forward range | `/harvester/cutting_tool_left_range` | Fixed to `cutting_tool_link`; follows rail, lift, extension, and cutter. It is intentionally outside the five-sensor docking estimator. |

The Mid-360 model is a regular-grid Gazebo approximation. It does not
reproduce the non-repetitive Livox scan pattern or a hardware FOV-region
configuration. The restricted ±60° front FOV was chosen to reduce GPU load
while viewing the tree.

Useful live checks:

```bash
ros2 topic hz /harvester/center_range
ros2 topic hz /harvester/cutting_tool_left_range
ros2 topic hz /harvester/lidar/raw_points
ros2 topic hz /harvester/platform_camera/depth/image_raw
ros2 topic hz /harvester/docking_camera/depth/image_raw
```

## RViz displays

The supplied RViz configuration contains one 3-D scene and one **Active Camera
View** image viewport. The right-side panel has **Cutter camera** and
**Docking camera** buttons. These publish the selected view to the stable topic
`/harvester/camera_view/image_raw`; they do not move either camera and do not
start another RViz process.

The panel also shows the five raw docking readings and one **Cutting sensor**
reading. Calibrated five-sensor rays are published to
`/harvester/docking/range_markers`. The moving cutter sensor publishes its
separate yellow ray to `/harvester/cutter/range_markers`.

The 3-D LiDAR cloud is enabled in the main scene. The optional RGB/LiDAR
projection is disabled by default:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  camera_lidar_projection:=true
```

This still uses the existing RViz process. Enable **Camera + LiDAR Overlay
(optional)** in the RViz Displays panel. Keep `camera_lidar_view:=false` on
the Xavier unless a second RViz is specifically required.

## Calibration boundaries

The range and camera/LiDAR JSON profiles are nominal simulation data only.
They do not change URDF sensor mounts or publish corrective TFs.

For algorithms:

- use raw Gazebo RGB, CameraInfo, and `/harvester/lidar/raw_points` for
  camera/LiDAR pairing;
- never use zero-stamped `/harvester/lidar/points` for fusion;
- use `c_channel_reference` for the five fixed docking sensors; and
- keep the cutter range stream separate because it crosses moving joints.

Before using a physical harvester, survey every mount, calibrate camera
intrinsics and camera-to-LiDAR extrinsics, establish a synchronized timestamp
domain, quantify error, and approve a deployment profile. Do not copy any
nominal simulation profile to hardware.

## Troubleshooting essentials

- **Gazebo only shows an old tree or exits 255:** stop the previous launch and
  ensure no old `gzserver` owns port `11345`.
- **RViz tree links replace harvester links:** there must be one harvester
  `/robot_description` publisher; the tree publisher must stay namespaced as
  `/tree`.
- **Robot flies, joints oscillate, or ODE SliderJoint warnings grow:** launch
  with `harvester_collision_mode:=off articulation_control_mode:=kinematic`.
  Do not add a physics-rate direct joint-position loop.
- **GPU/RViz pressure on Xavier:** keep `camera_lidar_view:=false` and
  `camera_lidar_projection:=false`; optionally launch `docking_camera:=false`.

See [MODEL_ASSUMPTIONS.md](MODEL_ASSUMPTIONS.md) for the estimated geometry,
coordinate conventions, and manual sensor-mount adjustment points.
