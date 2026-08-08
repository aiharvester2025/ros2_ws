# Oil Palm Tree Description — Low-Polygon 12 m Model

This package contains a procedural low-polygon **12 m oil palm tree** intended for RViz visualization and Gazebo sensor/docking simulation.

## Geometry

- Tree height: **12.0 m**
- Trunk diameter: **0.70 m at base**, tapering to **0.50 m at crown** (about 0.60 m nominal)
- Crown begins at approximately **9.2 m**
- 18 fronds/branches in three crown whorls
- 7 FFB bunches distributed between approximately **9.55–10.88 m**
- Low-poly trunk includes irregular ring/scar geometry rather than a perfectly smooth cylinder
- Reusable frond meshes include a curved rachis plus paired leaflets
- FFB mesh includes a short peduncle plus low-poly fruitlets

Approximate triangle counts per reusable mesh:

```text
{
  "trunk": 992,
  "frond_lower.stl": 528,
  "frond_mid.stl": 504,
  "frond_upper.stl": 480,
  "ffb": 664
}
```

## TF / target frames

The URDF exposes target frames useful for the cutting and docking pipeline:

- `tree_base` — ground-level trunk centre
- `trunk_center_reference`
- `crown_base_reference`
- `crown_center_reference`
- `trunk_top_reference`
- `branch_cut_target_01` ... `branch_cut_target_18`
- `ffb_cut_target_01` ... `ffb_cut_target_07`

The numeric target coordinates are also in `config/tree_targets.yaml`.

## RViz

Copy this package to your ROS 2 workspace `src/`, build it, source the workspace, then:

```bash
colcon build --packages-select oil_palm_tree_description
source install/setup.bash
ros2 launch oil_palm_tree_description display_tree.launch.py
```

RViz visualizes the tree, TF frames, robot model, LiDAR point clouds, camera data, and distance-sensor markers. **RViz itself does not generate simulated sensor returns.**

## Sensor simulation

For actual synthetic depth, 3D LiDAR, or range-sensor measurements, use the included `gazebo_model/oil_palm_tree` model in Gazebo / Ignition (or another physics/ray sensor simulator). The SDF contains collision meshes for the trunk, fronds, and FFBs so rays can intersect the actual low-poly geometry.

## Coordinate convention

ROS convention: +Z up; X/Y form the horizontal ground plane. `tree_base` is located at the centre of the trunk at ground level.

## Important

This is a simulation model, not a botanical reconstruction of one measured palm. Branch/FFB positions are deliberately varied to give the perception and docking system non-symmetric targets. For experimental repeatability, positions are fixed in the URDF and YAML file.
