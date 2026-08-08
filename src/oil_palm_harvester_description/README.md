# Oil-Palm Harvester Estimated URDF Package

This package is an estimated simulation model created from the supplied orthographic and isometric drawings. It models the vehicle, a yaw/elevation boom, four prismatic telescopic stages, a platform-level joint, a C-channel platform, five distance sensors, a vehicle-mounted 3D LiDAR, a platform depth camera, and a simplified rail-mounted cutting arm.

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
/harvester/platform_camera/depth/points
```

Plugin topic names can differ slightly by Gazebo/ROS package release. The pure URDF, mesh geometry and sensor frames are independent of those plugins.

## Suggested docking control sequence

1. Move the boom/platform only while the current checkpoint is invalid.
2. Stop hydraulic motion and wait for vibration/velocity to settle.
3. Read the five range sensors, LiDAR and depth camera.
4. Estimate `trunk_reference_estimate` relative to `c_channel_reference`.
5. Freeze the estimate and display X/Y/Z and alignment guidance.
6. Move by one controlled increment, invalidate the checkpoint, then repeat.
7. Declare docked only after centre offset, side clearance and insertion-depth thresholds are simultaneously satisfied.

See `MODEL_ASSUMPTIONS.md` before using this model for collision clearance or reachability decisions.
