# Harvester–Tree Simulation Handoff

**Baseline updated:** 2026-08-17
**Workspace:** `/home/ubuntu/ros2_ws`
**ROS / simulator:** ROS 2 Foxy with Gazebo Classic 11

This document records the working Gazebo + RViz baseline for the oil-palm
harvester and tree.  It is intended to be supplied to a future assistant or
used as a checklist before making sensor-simulation changes.

## 1. Current working result

The following behaviour is the approved baseline:

- Gazebo displays one fully assembled harvester and one oil-palm tree.
- RViz displays the same harvester and tree as separate `RobotModel` displays.
- The joint-state-publisher GUI changes the boom, platform, and cutting arm in
  RViz and in Gazebo.
- The tree is a **static environment object** at `world` position `(8.5, 0, 0)`.
- The harvester is **not static**.  Its base is moved kinematically with
  `/harvester/cmd_vel` so it can approach the fixed tree.
- The slider interface no longer makes the Gazebo harvester fly away.
- The active URDF supplies two depth cameras, a cropped Mid-360 coverage
  LiDAR model, five docking range sensors, and one cutter-attached range
  sensor. Their raw topics are live in Gazebo.
- RViz uses one process: the right-side panel switches the single camera image
  viewport and shows the raw range values. Calibrated docking rays, the
  cutter ray, and the LiDAR cloud are available in the same 3-D view.

This is a **commanded sensor-development simulation**, not yet a full
force-accurate vehicle simulation.  The current priority is reliable movement
and a stable scene for future range, depth-camera, and LiDAR simulation.

## 2. Start the known-good scene

Open one terminal only for this combined simulation:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py
```

If source files have changed, rebuild before the final `source` command:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select oil_palm_harvester_description --symlink-install
source install/setup.bash
```

Useful launch options:

| Argument | Default | Meaning |
|---|---:|---|
| `gui:=false` | `true` | Run Gazebo server without the Gazebo client window. |
| `rviz:=false` | `true` | Do not open RViz. |
| `render_mode:=mesh` | `mesh` | Detailed STL mesh render. Use `primitive` only as a graphics-driver fallback. |
| `harvester_collision_mode:=off` | `off` | Stable sensor-development mode; harvester contact bodies are removed. `on` is reserved for later physics/controller work. |
| `articulation_control_mode:=kinematic` | `kinematic` | Safe 20 Hz, rate-limited articulation for sensor development. `pid` is a legacy diagnostic fallback only. |
| `docking_camera:=true` | `true` | Enable the low-rate simulated docking depth camera. Disable it for a lower Xavier GPU load. |
| `range_calibration:=true` | `true` | Publish the five fixed docking-sensor endpoints, markers, status, and side-pair trunk diagnostic. |
| `camera_lidar_projection:=false` | `false` | Start optional cutter-camera/LiDAR RGB overlay and camera-frame cloud in the existing RViz process. |
| `camera_lidar_view:=false` | `false` | Start a second RViz camera-frame cloud view. Keep false on the Xavier unless explicitly needed. |

Before starting another run, stop the previous launch with `Ctrl-C`.  Do not
run two copies of `gazebo_harvester_and_tree.launch.py` at once: Gazebo Classic
normally uses TCP port `11345`.

## 3. System architecture

```text
joint_state_publisher_gui
          │  /harvester/joint_commands
          └──────────────────────────────► Gazebo harvester model plugin
                                                    │
                                      measured /harvester/joint_states
                                                    └──────────────► robot_state_publisher ─► RViz harvester model

/harvester/cmd_vel ───────────────────────────────► Gazebo model plugin
                                                    ├─ moves Gazebo base
                                                    └─ publishes world → base_link TF

static_transform_publisher ──────────────────────────► world → tree_base TF
tree_state_publisher (namespace /tree) ───────────────► tree TF frames
tree description publisher ───────────────────────────► /tree_description for RViz
```

### Models and frames

| Object | Gazebo role | RViz role | Root frame / pose |
|---|---|---|---|
| Harvester | Dynamic SDF model generated at launch | Harvester `RobotModel` | `base_link`, moved by `world -> base_link` |
| Tree | Static SDF environment model | Tree `RobotModel` | `tree_base`, fixed at `(8.5, 0, 0)` in `world` |
| Ground / sun | Gazebo world assets | Not a robot model | `world` |

The frame ownership is deliberate:

- `world -> tree_base` is static and belongs to the launch file.
- `world -> base_link` is dynamic and belongs to the Gazebo model plugin.
- The harvester and tree `robot_state_publisher` nodes publish the rest of
  their respective link chains.

**Do not add a permanent static `world -> base_link` transform.** It conflicts
with the plugin's moving base transform and breaks `/harvester/cmd_vel`.

### Description topics

RViz requires separate descriptions because it displays two separate models:

| Topic | Required publisher | Consumer |
|---|---|---|
| `/robot_description` | Harvester `robot_state_publisher` only | RViz “Harvester Robot” display |
| `/tree/robot_description` | Namespaced tree `robot_state_publisher` | Internal tree publisher topic; not used by the harvester display |
| `/tree_description` | Retained tree-description script | RViz “Oil Palm Tree” display |

The tree state publisher must remain in namespace `tree`.  On ROS 2 Foxy,
`robot_state_publisher` automatically republishes its description.  Without
that namespace, the tree URDF also appears on `/robot_description`, causing
RViz to show tree links as the robot.

## 4. User controls

### 4.1 Articulated joints

Use the `joint_state_publisher_gui` window.  It publishes:

```text
/harvester/joint_commands   (sensor_msgs/msg/JointState)
```

It sends targets to the Gazebo model plugin. The plugin publishes measured
positions on `/harvester/joint_states`, which is the only joint-state stream
consumed by `robot_state_publisher` and RViz. Individual sliders and
**Randomize pose** are supported. In the default kinematic articulation mode,
the bridge clamps targets to URDF limits and rate-limits them at 20 Hz using
the URDF velocity limits (the turret is capped at 0.05 rad/s). RViz
deliberately follows that actual motion instead of jumping ahead to the
requested target.

The movable joints include:

```text
boom_turret_joint
boom_elevation_joint
boom_extension_1_joint ... boom_extension_4_joint
platform_level_joint
rail_carriage_joint
cutting_arm_lift_joint
cutting_arm_extension_joint
```

### 4.1.1 Half-length cutting-arm layout

The cutting-arm base, extension link, and cutter are half their original
length along X.  Their mounting chain is shortened by the same factor.

```text
rail_carriage_joint:        -2.55 to +2.55 rad
cutting_arm_lift_joint:     -0.35 to +1.05 rad
cutting_arm_extension_joint: 0.00 to 0.375 m
```

The arm meshes use X scale `0.5` while preserving their original width and
height.  The lift-joint position is unchanged; the extension-joint origin is
0.31 m and the cutter attachment is 0.46 m, so no visual gap is introduced.

### 4.2 Harvester base motion

Publish a standard velocity command while the combined Gazebo launch is
running:

```bash
ros2 topic pub -r 10 /harvester/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

Press `Ctrl-C` to stop publishing.  The model plugin has a 0.5-second command
timeout, so the base stops automatically if the publisher exits.

The current controller uses only `linear.x` and `angular.z`.  It intentionally
does not simulate wheel traction, suspension, terrain forces, or inertia.

## 5. Important implementation details

### Combined launch

The working launch is:

```text
launch/gazebo_harvester_and_tree.launch.py
```

It does the following:

1. Converts `urdf/oil_palm_harvester_kinematic.urdf` to SDF with `gz sdf -p`.
2. Creates a temporary world file at
   `/tmp/oil_palm_harvester_dynamic_scene.world`.
3. Rewrites harvester mesh URIs to absolute installed `file://` paths.
4. Inserts the harvester directly into the generated world instead of using
   `spawn_entity.py`.  This avoids the large-URDF spawn race seen on Foxy.
5. Includes the oil-palm tree as a static model at `(8.5, 0, 0)`.
6. Configures a local Gazebo model database so the Gazebo GUI does not wait on
   the public online model database.
7. Passes `server_required:=true` to Gazebo.  If `gzserver` fails, the whole
   launch stops instead of leaving a misleading tree-only GUI/RViz session.

### Gazebo harvester model

The generated SDF deliberately has:

```text
<static>false</static>           # the harvester is movable
base_link <kinematic>true</kinematic>
all link gravity disabled
```

The base-link kinematic setting is not the same as making the robot static.
It means the plugin owns the base pose and can move it from `/harvester/cmd_vel`
without arm joint commands pushing the entire vehicle through ODE physics.

With the default `harvester_collision_mode:=off`, harvester collision elements
are removed from the generated SDF.  The tree collision geometry remains, so
future ray/range/LiDAR sensing can still detect the tree.  Do not enable
harvester collision mode until a proper physical base and joint controller are
implemented and tested.

### Kinematic Gazebo bridge

The plugin is implemented in:

```text
src/harvester_kinematic_gazebo_plugin.cpp
```

Its responsibilities are:

- subscribe to `/harvester/joint_commands`;
- apply changed articulated-joint targets kinematically at 20 Hz;
- publish measured Gazebo joint positions on `/harvester/joint_states`;
- subscribe to `/harvester/cmd_vel`;
- integrate and apply the commanded base pose at 20 Hz;
- publish the measured `world -> base_link` transform for RViz;
- stop base motion after 0.5 seconds without a new velocity command.

Stability rules already encoded in the plugin:

- The default `kinematic` mode uses joint limits and each URDF velocity limit
  to ramp targets at 20 Hz.  The turret is additionally capped at 0.05 rad/s.
- It applies one changed-joint batch, then clears residual Gazebo velocity and
  force state exactly once.  It never replays a full joint map every physics
  update.
- This is deliberately a collision-off sensor-development model.  Do not use
  kinematic articulation with `harvester_collision_mode:=on` as contact
  physics.
- `pid` preserves the former controller path only as a fresh-launch fallback
  for diagnosis; it is not the recommended operating mode.

Do not undo these rules without reproducing and testing the entire scene.  The
previous direct joint-position loop at physics rate produced a huge
`ODESliderJoint::Anchor not implemented` log flood and was a direct cause of
Gazebo instability and robot flight.

## 6. Problems that were solved

| Symptom | Root cause | Current protection |
|---|---|---|
| Slider pose appeared briefly, then returned | Competing description/joint-state startup paths | GUI loads the harvester URDF directly and owns `/harvester/joint_commands`; only Gazebo publishes feedback `/harvester/joint_states`. |
| Robot missing from Gazebo | Foxy large-URDF spawn path was unreliable | Harvester is embedded in a generated SDF world rather than spawned later. |
| Gazebo splash stalled / only a partial scene appeared | Gazebo client waited on the online model database | Launch points `GAZEBO_MODEL_DATABASE_URI` to a local database. |
| Boom/platform/arm fell or flew away | Persistent force PID reactions or direct poses at physics rate | 20 Hz rate-limited changed-joint kinematic batches, velocity reset, kinematic base, and no harvester contact bodies by default. |
| RViz showed tree links as the robot | Tree publisher also advertised `/robot_description` | Tree state publisher is namespaced as `/tree`; only harvester supplies `/robot_description`. |
| Tree visible but no robot / no slider effect in Gazebo | `gzserver` exited because port `11345` was occupied by another Gazebo session | Run one Gazebo session; `server_required:=true` now shuts the whole launch down on server failure. |
| Invalid `boom_*_link` TF warnings | Gazebo bridge was not running or descriptions/TF sources were conflicting | One description publisher per model and one successful Gazebo server are required. |

## 7. Verification checklist

After a fresh launch, verify the following before changing code:

```bash
# Both Gazebo models must be listed.
gz model -l

# The harvester description should have one publisher only.
ros2 topic info /robot_description -v

# The tree's automatic description must be namespaced.
ros2 topic info /tree/robot_description -v

# Core command and measured-feedback streams must be visible.
ros2 topic list | grep -E '/harvester/(joint_commands|joint_states|cmd_vel)'

# Both root-frame chains must resolve.
ros2 run tf2_ros tf2_echo world tree_base
ros2 run tf2_ros tf2_echo world base_link
```

Expected results:

- `gz model -l` includes `oil_palm_harvester` and `oil_palm_tree`.
- `/robot_description` has one harvester publisher.
- `world -> tree_base` is static.
- `world -> base_link` is available while Gazebo is running.
- Moving a GUI slider changes the RViz harvester and the Gazebo harvester.

## 8. Troubleshooting guide

### A. Gazebo reports exit code 255, or only an old tree world is visible

The common error is:

```text
EXCEPTION: Unable to start server [bind: Address already in use].
```

This means another Gazebo server holds the default master port (`11345`).

1. Stop the terminal running the previous launch with `Ctrl-C`.
2. Identify processes; do not kill everything blindly:

   ```bash
   pgrep -af 'gzserver|gzclient|ros2 launch oil_palm_harvester_description'
   ss -ltnp | rg ':11345'
   ```

3. If a stale process remains, stop only the exact identified PID:

   ```bash
   kill <PID>
   ```

4. Relaunch once using the command in section 2.

Do not run a second combined launch in another terminal on the same default
Gazebo master port.

### B. RViz displays tree links as the harvester, or the harvester is absent

Check the description-topic publishers:

```bash
ros2 topic info /robot_description -v
```

The correct fresh launch has one harvester publisher.  More than one publisher
usually means an old launch, a manually started tree `robot_state_publisher`,
or an outdated installed launch file is still active.

Recovery:

1. Stop all old combined launches.
2. Rebuild/source the workspace if source files were changed.
3. Start one clean combined launch.

### C. Slider changes RViz but not Gazebo

First check the terminal that launched Gazebo.  It must contain:

```text
Harvester kinematic bridge ready: joint commands on /harvester/joint_commands, measured joint states on /harvester/joint_states ...
```

If it does not, `gzserver` did not start successfully.  Follow section A.

Then check the command and feedback graph:

```bash
ros2 topic info /harvester/joint_commands -v
ros2 topic info /harvester/joint_states -v
```

The GUI must be the command publisher and the Gazebo bridge must be the
feedback publisher. Do not start an additional `joint_state_publisher` or
manual feedback publisher alongside the combined launch.

### D. Robot flies, falls, or the Gazebo log grows rapidly

1. Relaunch with the default `harvester_collision_mode:=off`.
2. Keep `articulation_control_mode:=kinematic`; do not reintroduce a direct
   pose loop at physics rate or persistent PID targets in this mode.
3. Check the Gazebo server log for thousands of `ODESliderJoint` lines.  A
   healthy run has only a small number of startup `SetAnchor` notices, not a
   continuous flood.
4. Confirm only one Gazebo server is running.

### E. Gazebo meshes do not render

Use the primitive visual fallback only for diagnosis:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  render_mode:=primitive
```

The normal setting is `render_mode:=mesh`.  Mesh URIs are intentionally
rewritten to absolute `file://` paths by the launch file.

## 9. Current sensor-simulation baseline

All sensor blocks below are active in
`urdf/oil_palm_harvester_kinematic.urdf`, which is the shared Gazebo/RViz
source. Do not replace it with `oil_palm_harvester_estimated.urdf`: that
alternate file contains `gazebo_ros2_control` and different physics assumptions
that are not part of the stable combined launch.

| Sensor | Raw topic(s) | Frame / key configuration |
|---|---|---|
| Cutter depth camera | `/harvester/platform_camera/depth/image_raw`, `.../camera_info`, `.../depth/image_raw`, `.../points` | `platform_depth_camera_optical_frame`; fixed to `cutting_arm_base_link`; 640×400 at 15 Hz. |
| Docking depth camera | `/harvester/docking_camera/depth/image_raw`, `.../camera_info` | `front_depth_camera_optical_frame`; fixed to the compact platform sensor carrier; 320×240 at 8 Hz. |
| Five docking ranges | `/harvester/{center,left_45,right_45,left_side,right_side}_range` | One ray each, 20 Hz, 0.05–3.0 m. Rigidly projected into `c_channel_reference`. |
| Cutter-forward range | `/harvester/cutting_tool_left_range` | `cutting_tool_left_range_sensor_link`; follows rail, lift, extension, and cutter. It is not part of the five-sensor calibration estimator. |
| Mid-360 coverage LiDAR | `/harvester/lidar/raw_points` | `vehicle_lidar_link`; 107×64 GPU-ray grid, −60° to +60° horizontal, about −7° to +52° vertical, 0.1–40 m, 10 Hz. |
| RViz LiDAR copy | `/harvester/lidar/points` | Zero-stamped/latest-TF display copy only; never use for time-correlated fusion. |

The LiDAR is a regular-grid Gazebo coverage approximation, not a vendor-faithful
Livox Mid-360 non-repetitive scan. Its restricted front FOV keeps the Xavier
load moderate while the harvester approaches the tree.

### Sensor visualization and calibration

- **One RViz process:** `harvester_tree_combined.rviz` displays the robot/tree,
  LiDAR cloud, calibrated docking rays, cutter range ray, and one camera image
  viewport.
- **Camera selector:** the panel buttons publish `cutter` or `docking` on
  `/harvester/camera_view/select`; the selector forwards the chosen image to
  `/harvester/camera_view/image_raw`. It never changes TF, mounts, or raw
  sensor topics.
- **Five docking ranges:** `range_sensor_calibration.py` launches by default.
  It publishes endpoints, markers, a gated side-pair trunk-centre diagnostic,
  and status. See `CALIBRATION_FRAME_CONTRACT.md`.
- **Cutter range:** `cutter_range_marker.py` publishes its independent yellow
  ray to `/harvester/cutter/range_markers`. It remains separate because its
  transform crosses moving arm joints.
- **Camera/LiDAR:** `camera_lidar_projection:=true` starts an optional,
  low-rate raw-timestamp projector in the existing RViz process. It stays off
  by default. See `CAMERA_LIDAR_CALIBRATION_CONTRACT.md`.

### Validate after a fresh launch

```bash
ros2 topic hz /harvester/center_range
ros2 topic hz /harvester/cutting_tool_left_range
ros2 topic hz /harvester/lidar/raw_points
ros2 topic hz /harvester/platform_camera/depth/image_raw
ros2 topic hz /harvester/docking_camera/depth/image_raw
```

Keep `harvester_collision_mode:=off`, `articulation_control_mode:=kinematic`,
`camera_lidar_view:=false`, and `camera_lidar_projection:=false` during normal
Xavier operation. Enable optional processing one component at a time only
after the baseline is stable.

## 10. Files that define the baseline

| File | Purpose |
|---|---|
| `launch/gazebo_harvester_and_tree.launch.py` | Main working combined Gazebo + RViz launch. |
| `launch/display_harvester_and_tree.launch.py` | RViz-only harvester/tree display; preserves description-topic separation. |
| `src/harvester_kinematic_gazebo_plugin.cpp` | Stable joint/base Gazebo bridge. |
| `urdf/oil_palm_harvester_kinematic.urdf` | Shared RViz and current Gazebo kinematic structure. |
| `urdf/oil_palm_harvester_estimated.urdf` | Alternate future physical-controller reference; not the combined-launch model. |
| `rviz/harvester_tree_combined.rviz` | One-RViz configuration with robot/tree displays, active camera image, LiDAR, range markers, and panel. |
| `scripts/range_sensor_calibration.py` | Five fixed docking-sensor projection, markers, status, and side-pair trunk diagnostic. |
| `scripts/cutter_range_marker.py` | Separate moving cutter-range ray marker. |
| `scripts/camera_view_selector.py` | Header-preserving two-camera selector for the one RViz image viewport. |
| `scripts/camera_lidar_projection.py` | Optional raw-timestamp cutter-camera/LiDAR image projection. |
| `CALIBRATION_FRAME_CONTRACT.md` | Five fixed docking-range frame and calibration contract. |
| `CAMERA_LIDAR_CALIBRATION_CONTRACT.md` | Cutter-camera/LiDAR extrinsic, timing, and projection contract. |
| `MODEL_ASSUMPTIONS.md` | Geometry, scale, frame, and sensor-mount assumptions. |
| `README.md` | Current quick-start, sensor inventory, and resource-safe operation guide. |

## 11. Copyable context for a future assistant

```text
I have a ROS 2 Foxy / Gazebo Classic 11 workspace at ~/ros2_ws.
The working baseline is:
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic

Read src/oil_palm_harvester_description/SIMULATION_HANDOFF.md first.

Important constraints:
- The oil-palm tree is static at world=(8.5, 0, 0); do not attach it to base_link.
- The harvester is non-static but has a kinematically commanded base.
- GUI publishes /harvester/joint_commands; the Gazebo model plugin publishes
  measured /harvester/joint_states for RViz.
- /harvester/cmd_vel moves the harvester base in Gazebo and RViz.
- /robot_description must belong only to the harvester; tree RSP is namespaced /tree.
- Do not run duplicate Gazebo launches or add static world->base_link TF.
- Keep harvester_collision_mode:=off during initial sensor development.
- Keep the changed-only, 20 Hz kinematic articulation path; never reintroduce
  direct joint-position calls at physics rate or unbounded PID commands.
- The active kinematic URDF already contains two depth cameras, five docking
  ranges, one moving cutter range, and the cropped 107 x 64 Mid-360 coverage
  LiDAR. Do not move a sensor or replace a raw topic while changing perception.
- The five docking ranges are calibrated only in c_channel_reference; the
  moving cutter range stays separate. Raw camera/LiDAR fusion uses raw
  simulation-time topics, never /harvester/lidar/points.
- Keep one RViz by default. The panel switches the selected camera image;
  camera_lidar_view and camera_lidar_projection are optional Xavier-costed
  features and default false.

Read the two calibration contracts before changing camera/LiDAR or range
calibration. Preserve the current Gazebo/RViz/GUI graph while adding any new
perception feature.
```

## 12. External canonical telemetry boundary

The additive package `harvester_telemetry_gateway` reads the existing raw
camera, LiDAR, range, trunk-estimate, and calibration topics and publishes
canonical ZeroMQ v1 packets. It must stay read-only: no ROS publications, TF,
Gazebo service calls, joint/base commands, or hardware-control messages.

- The live Xavier PUB endpoint is `tcp://*:5590`; the read-only status REP
  endpoint is `tcp://*:5600`.
- `/harvester/lidar/raw_points` is the LiDAR input. Do not replace it with the
  zero-stamped RViz display topic `/harvester/lidar/points`.
- Sensor observations declare `ros_sim_time`. Gateway/status-only data uses
  `utc_host` where no Gazebo measurement timestamp exists.
- Recording is opt-in via `record_dir` and saves exact multipart packets for
  dashboard development/audit; replay uses a separate endpoint (default 5591).
- The future Orin adapter/dashboard is not part of this simulation package.
  The real machine has no joint encoders, so hardware must not claim a
  world-fixed clicked target without a separate validated pose source.

Read [`../../docs/TELEMETRY_HANDOFF.md`](../../docs/TELEMETRY_HANDOFF.md) and
[`../../docs/canonical_zmq_v1.md`](../../docs/canonical_zmq_v1.md) before
changing telemetry sources, timestamps, recording, or the future Orin adapter.
