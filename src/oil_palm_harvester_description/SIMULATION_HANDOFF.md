# Harvester–Tree Simulation Handoff

**Baseline verified:** 2026-08-11
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

Before starting another run, stop the previous launch with `Ctrl-C`.  Do not
run two copies of `gazebo_harvester_and_tree.launch.py` at once: Gazebo Classic
normally uses TCP port `11345`.

## 3. System architecture

```text
joint_state_publisher_gui
          │  /harvester/joint_states
          ├──────────────────────────────► robot_state_publisher ─► RViz harvester model
          │
          └──────────────────────────────► Gazebo harvester model plugin
                                                    │
                                              Gazebo articulated model

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
/harvester/joint_states   (sensor_msgs/msg/JointState)
```

It controls the RViz harvester immediately and sends the same target to the
Gazebo model plugin.  Individual sliders and **Randomize pose** are supported.
Gazebo moves toward the requested position within the joint limits, damping,
and velocity limits; it may animate more smoothly than RViz.

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

- subscribe to `/harvester/joint_states`;
- maintain Gazebo position-controller targets for changed joint values;
- subscribe to `/harvester/cmd_vel`;
- integrate and apply the commanded base pose at 20 Hz;
- publish the matching `world -> base_link` transform for RViz;
- stop base motion after 0.5 seconds without a new velocity command.

Stability rules already encoded in the plugin:

- It uses Gazebo's persistent `JointController` with scoped joint names.
- Position PID commands are bounded to `±2000`, preventing a large GUI jump
  from creating an unbounded reaction force.
- It does **not** call `Joint::SetPosition()` continuously in every physics
  update.
- It does **not** manually call `JointController::Update()`; Gazebo performs
  that for a non-static model.

Do not undo these rules without reproducing and testing the entire scene.  The
previous direct joint-position loop produced a huge
`ODESliderJoint::Anchor not implemented` log flood and was a direct cause of
Gazebo instability and robot flight.

## 6. Problems that were solved

| Symptom | Root cause | Current protection |
|---|---|---|
| Slider pose appeared briefly, then returned | Competing description/joint-state startup paths | GUI loads the harvester URDF directly and owns `/harvester/joint_states`. |
| Robot missing from Gazebo | Foxy large-URDF spawn path was unreliable | Harvester is embedded in a generated SDF world rather than spawned later. |
| Gazebo splash stalled / only a partial scene appeared | Gazebo client waited on the online model database | Launch points `GAZEBO_MODEL_DATABASE_URI` to a local database. |
| Boom/platform/arm fell or flew away | Repeated ODE position teleports and unbounded controller forces | Persistent bounded controller targets, kinematic base, no harvester contact bodies by default. |
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

# Core controls must be visible.
ros2 topic list | rg '/harvester/(joint_states|cmd_vel)'

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
Harvester kinematic bridge ready: joint states on /harvester/joint_states ...
```

If it does not, `gzserver` did not start successfully.  Follow section A.

Then check that only the intended GUI publishes the joint-state topic:

```bash
ros2 topic info /harvester/joint_states -v
```

Do not start an additional `joint_state_publisher` or manual zero-joint-state
publisher alongside `joint_state_publisher_gui`; it can overwrite GUI values.

### D. Robot flies, falls, or the Gazebo log grows rapidly

1. Relaunch with the default `harvester_collision_mode:=off`.
2. Do not reintroduce a repeated `Joint::SetPosition()` loop in the plugin.
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

## 9. Sensor-simulation status and next task

The combined working launch currently prioritizes stable robot/tree movement.
It does **not yet guarantee live simulated sensor topics**.

The package already contains sensor definitions in:

```text
urdf/oil_palm_harvester_estimated.urdf
```

Those definitions are useful source material, but the combined launch uses
`oil_palm_harvester_kinematic.urdf` plus the custom model plugin.  Do not
replace the entire current harvester with `oil_palm_harvester_estimated.urdf`
without a migration plan: it also contains `gazebo_ros2_control` and different
physics assumptions that can reintroduce the earlier stability problems.

### Existing sensor mount frames

| Sensor | Mount frame | Candidate output | Definition present in estimated URDF |
|---|---|---|---|
| Centre range sensor | `center_range_sensor_link` | `/harvester/center_range` | Ray sensor, 20 Hz |
| Left 45° range sensor | `left_45_range_sensor_link` | `/harvester/left_45_range` | Ray sensor, 20 Hz |
| Right 45° range sensor | `right_45_range_sensor_link` | `/harvester/right_45_range` | Ray sensor, 20 Hz |
| Left side range sensor | `left_side_range_sensor_link` | `/harvester/left_side_range` | Ray sensor, 20 Hz |
| Right side range sensor | `right_side_range_sensor_link` | `/harvester/right_side_range` | Ray sensor, 20 Hz |
| Cutting-arm 3D LiDAR | `vehicle_lidar_link` | `/harvester/lidar/points` | GPU ray, 720 × 16, 10 Hz; fixed to `cutting_arm_base_link` |
| Cutting-arm depth camera | `platform_depth_camera_link` / `platform_depth_camera_optical_frame` | `/harvester/platform_camera/depth/depth/image_raw`, `/harvester/platform_camera/depth/points` | Depth camera, 640 × 400, 15 Hz; fixed to `cutting_arm_base_link` |

The depth-camera and LiDAR blocks are integrated in the active kinematic URDF.
Confirm the live topics after a fresh launch with
`ros2 topic list | grep '/harvester/'`; Gazebo plugin versions can vary
slightly. Range-sensor blocks remain in the estimated URDF as source material
only.

### Required approach for the next sensor task

1. Keep `gazebo_harvester_and_tree.launch.py` as the primary launch.
2. Keep the tree static at `(8.5, 0, 0)` and collidable.
3. Keep the harvester model non-static, base kinematic, and its collision mode
   off during initial sensor testing.
4. Keep the integrated depth-camera Gazebo block in the kinematic URDF. Add
   only one further sensor block at a time from the estimated URDF.
5. Do not add a second harvester, a second tree state publisher, or another
   `robot_description` publisher.
6. Test one sensor type at a time: depth camera first, then range sensors,
   then 3D LiDAR.
7. For each sensor, verify its ROS topic, frame ID, values versus distance to
   the fixed tree, and response while the base moves through `/harvester/cmd_vel`.

## 10. Files that define the baseline

| File | Purpose |
|---|---|
| `launch/gazebo_harvester_and_tree.launch.py` | Main working combined Gazebo + RViz launch. |
| `launch/display_harvester_and_tree.launch.py` | RViz-only harvester/tree display; preserves description-topic separation. |
| `src/harvester_kinematic_gazebo_plugin.cpp` | Stable joint/base Gazebo bridge. |
| `urdf/oil_palm_harvester_kinematic.urdf` | Shared RViz and current Gazebo kinematic structure. |
| `urdf/oil_palm_harvester_estimated.urdf` | Sensor-plugin source material and future physical-controller reference. |
| `rviz/harvester_tree_combined.rviz` | RViz configuration with separate harvester and tree `RobotModel` displays. |
| `MODEL_ASSUMPTIONS.md` | Geometry, scale, frame, and sensor-mount assumptions. |
| `README.md` | Short usage instructions. |

## 11. Copyable context for a future assistant

```text
I have a ROS 2 Foxy / Gazebo Classic 11 workspace at ~/ros2_ws.
The working baseline is:
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py

Read src/oil_palm_harvester_description/SIMULATION_HANDOFF.md first.

Important constraints:
- The oil-palm tree is static at world=(8.5, 0, 0); do not attach it to base_link.
- The harvester is non-static but has a kinematically commanded base.
- GUI publishes /harvester/joint_states; the Gazebo model plugin mirrors it.
- /harvester/cmd_vel moves the harvester base in Gazebo and RViz.
- /robot_description must belong only to the harvester; tree RSP is namespaced /tree.
- Do not run duplicate Gazebo launches or add static world->base_link TF.
- Keep harvester_collision_mode:=off during initial sensor development.
- Do not reintroduce repeated Joint::SetPosition calls or unbounded joint PID commands.

Next requested task: verify the arm-mounted depth-camera data after a fresh
launch, then add reliable simulated five range sensors and 3D LiDAR data that
respond to the harvester motion relative to the static tree.
Preserve the currently working Gazebo/RViz/GUI behavior while doing so.
```
