# Oil-Palm Harvester Estimated URDF Package

This package is an estimated simulation model created from the supplied orthographic and isometric drawings. It models the vehicle, a yaw/elevation boom, four prismatic telescopic stages, a platform-level joint, a C-channel platform, five distance sensors, an arm-mounted 3D LiDAR, an arm-mounted depth camera, and a simplified rail-mounted cutting arm.

## Files

- `urdf/oil_palm_harvester_kinematic.urdf` — simulator-neutral URDF for RViz and kinematic testing.
- `urdf/oil_palm_harvester_estimated.urdf` — Gazebo Classic-oriented URDF with sensor and `ros2_control` plugins.
- `meshes/visual/*.stl` — estimated visual meshes.
- `worlds/docking.world` — ground plane plus a 0.60 m diameter, 12 m tall trunk.
- `config/controllers.yaml` — joint trajectory controller.
- `config/example_joint_positions.yaml` — an example stop/freeze checkpoint pose.
- `preview/oil_palm_harvester_estimated.glb` — combined preview model at a representative pose.

## Build and view in RViz

```bash
mkdir -p ~/harvester_ws/src
cp -r oil_palm_harvester_description ~/harvester_ws/src/
cd ~/harvester_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch oil_palm_harvester_description display.launch.py
```

Use the joint-state publisher sliders to move the boom elevation, each telescopic stage, platform levelling joint and rail/cutting-arm joints.

## Gazebo Classic docking example

Install the ROS packages that provide `gazebo_ros`, `gazebo_ros2_control`, controllers and the sensor plugins, then run:

```bash
source ~/harvester_ws/install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_docking.launch.py
```

Expected topics include:

```text
/harvester/center_range
/harvester/left_45_range
/harvester/right_45_range
/harvester/left_side_range
/harvester/right_side_range
/harvester/lidar/points
/harvester/platform_camera/depth/image_raw
/harvester/platform_camera/depth/depth/image_raw
/harvester/platform_camera/depth/points
```

Plugin topic names can differ slightly by Gazebo/ROS package release. The pure URDF, mesh geometry and sensor frames are independent of those plugins.

## Gazebo + RViz with the full tree

Launch the harvester, the full collision-enabled oil-palm tree, Gazebo Classic and the combined RViz view:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py
```

The tree is a static Gazebo world object at `world` position `(8.5, 0, 0)`.
The harvester is a movable, fully assembled Gazebo model. For a headless
simulator run, add `gui:=false rviz:=false`.
This launch is separate from `display_harvester_and_tree.launch.py`, so the
joint-state-publisher GUI workflow is unchanged.

The slider GUI publishes commands on `/harvester/joint_commands`. The Gazebo
harvester model subscribes to those commands, then publishes its **measured**
joint positions on `/harvester/joint_states`; RViz uses that feedback. This
keeps the RViz robot and all sensor frames aligned with the physical Gazebo
model while slider and **Randomize pose** changes move the boom, platform and
cutting arm. The harvester model is non-static; its base is kinematically commanded at this stage through
`geometry_msgs/msg/Twist` on `/harvester/cmd_vel`. It also publishes the
matching `world -> base_link` TF for RViz. For example, drive forward briefly,
then press `Ctrl-C` to stop the command:

```bash
ros2 topic pub -r 10 /harvester/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

The bridge has a 0.5-second command timeout, so the harvester stops safely if
the velocity publisher exits unexpectedly.

This is a commanded sensor-development simulation: it deliberately avoids
free-body contact forces while sensor models are added. By default the
harvester's own contact bodies are disabled for stable GUI pose control, while
the tree remains a static, collidable world object. The tree description
publisher is used only for its RViz visual and TF tree.

The arm-mounted cutter depth camera is enabled in the combined launch. Its legacy
frame/topic prefix remains `platform_depth_camera_*` /
`/harvester/platform_camera` for compatibility, but its fixed joint is now on
`cutting_arm_base_link`. It follows the rail-carriage and arm-lift motion, not
the extension stroke. After launching, verify its expected outputs with:

```bash
ros2 topic list | grep '/harvester/platform_camera'
```

The depth image is expected on
`/harvester/platform_camera/depth/depth/image_raw`; its point cloud is
`/harvester/platform_camera/depth/points`. The colour image is
`/harvester/platform_camera/depth/image_raw`. The supplied combined RViz
configuration opens that colour stream in a separate **Cutting-Arm Camera
Image** window while retaining the 3D robot/tree view.

The platform-mounted docking camera is also enabled by default, with an
independent `/harvester/docking_camera/depth/image_raw` colour stream. To keep
the Xavier load modest it runs at **320 x 240 pixels and 8 Hz**; it does not
move or alter the cutter camera, LiDAR, TF, or joint-control path. The existing
right-side **Docking Sensor Values** RViz panel contains **Cutter camera** and
**Docking camera** buttons. They switch the single **Active Camera View** image
viewport; no second RViz window is created. The selector output is
`/harvester/camera_view/image_raw` and preserves the selected camera's original
header and frame ID. The optional camera--LiDAR overlay remains calibrated to
the cutter camera only.

For a lower-resource fallback, omit the rendered docking sensor at launch:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  docking_camera:=false
```

The arm-mounted 3D LiDAR is also enabled in the combined launch. Its
`/harvester/lidar/points` stream appears as **Cutting-Arm LiDAR Point Cloud**
in the supplied RViz configuration.

## Camera--LiDAR calibration and optional overlay

The camera and LiDAR have nominal fixed mounts in the active kinematic URDF,
and both now follow Gazebo's measured joint feedback in RViz. The simulation
profile derives their relative transform from those fixed joints; it does not
move either sensor or add a competing TF. Validate the profile with:

```bash
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_camera_lidar_calibration.py"
```

The low-rate RGB/LiDAR overlay is optional and disabled by default to protect
the Xavier. It uses only the raw acquisition-time streams, never the
zero-stamped RViz LiDAR copy:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  camera_lidar_projection:=true
```

In the existing RViz process, enable **Camera + LiDAR Overlay (optional)** in
the Displays panel. This does not start a second RViz. See
[CAMERA_LIDAR_CALIBRATION_CONTRACT.md](CAMERA_LIDAR_CALIBRATION_CONTRACT.md)
for the frame equation, output topics, timestamp policy, and physical-machine
commissioning boundary.

## Docking range-sensor calibration

The combined launch also projects the five raw Gazebo range readings into the
moving `c_channel_reference` docking frame. It adds calibrated endpoint topics,
uncluttered RViz rays, and a gated side-pair trunk-centre estimate without
changing the raw sensor streams or physical Gazebo mounts. The configuration is
simulation-only; do not reuse it for a physical harvester.

See [CALIBRATION_FRAME_CONTRACT.md](CALIBRATION_FRAME_CONTRACT.md) for the
frame convention, configuration files, validation command, output topics, and
real-machine commissioning boundary.

## Suggested docking control sequence

1. Move the boom/platform only while the current checkpoint is invalid.
2. Stop hydraulic motion and wait for vibration/velocity to settle.
3. Read the five range sensors, LiDAR and depth camera.
4. Estimate `trunk_reference_estimate` relative to `c_channel_reference`.
5. Freeze the estimate and display X/Y/Z and alignment guidance.
6. Move by one controlled increment, invalidate the checkpoint, then repeat.
7. Declare docked only after centre offset, side clearance and insertion-depth thresholds are simultaneously satisfied.

See `MODEL_ASSUMPTIONS.md` before using this model for collision clearance or reachability decisions.
