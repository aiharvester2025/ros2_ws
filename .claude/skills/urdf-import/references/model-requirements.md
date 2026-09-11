# What a real model must provide

Source of truth: `MODEL_ASSUMPTIONS.md` §"Required information for an accurate conversion", plus the
current placeholder's structure (`urdf/oil_palm_harvester_kinematic.urdf`).

## The hard requirement (why the placeholder is a placeholder)

The supplied drawings have **no dimensional scale**. Orthographic/isometric screenshots cannot recover
hidden geometry, reliable dimensions, or joint axes. A real model must supply, at minimum:

1. **SolidWorks assembly or STEP/Parasolid export** — the source of geometry.
2. **Joint axes** — the exact axis of rotation/translation for every joint (the placeholder guessed
   these; e.g. `boom_elevation_joint` axis `(0,-1,0)` vs `platform_level_joint` axis `(0,+1,0)` — the
   opposite-axis leveling convention is a hard-won fact, see `boom-dock-ik`).
3. **Retracted and extended stop positions** — the physical joint limits (placeholder: boom extension
   0–2.4 m/stage ×4, cutting-arm lift −0.35…+1.05 rad, etc.).
4. **Component masses and centres of gravity** — for the `<inertial>` blocks (placeholder has none in
   the kinematic URDF; `oil_palm_harvester_estimated.urdf` has estimated values, e.g. base_link
   3100 kg).
5. **Collision clearances** — to build `<collision>` geometry without interpenetration.
6. **Exact sensor mounting transforms** — where each LiDAR/camera/range sensor sits relative to its
   parent link.

## The current joint set (must be preserved by name)

```
boom_turret_joint            (revolute, ±0.35 rad; plugin special-cases this one)
boom_elevation_joint         (revolute, 0 … 1.309 rad, axis (0,-1,0))
boom_extension_1..4_joint    (prismatic, 0 … 2.4 m each)
platform_level_joint         (revolute, ±1.57 rad, axis (0,+1,0))
rail_carriage_joint          (revolute, -2.55 … +2.55 rad)
cutting_arm_lift_joint       (revolute, -0.35 … +1.05 rad)
cutting_arm_extension_joint  (prismatic, 0 … 0.375 m)
```

Root link `base_link` (X toward boom/platform, Y left, Z up). The C-opening faces +X.

## Kinematic vs physical model (choose deliberately)

The placeholder ships as **two** URDFs:

- `oil_palm_harvester_kinematic.urdf` — the **active** baseline: no `<inertial>` masses, no
  transmissions, driven by the custom `harvester_kinematic_gazebo_plugin.cpp` (20 Hz kinematic
  articulation).
- `oil_palm_harvester_estimated.urdf` — a **partial physical template**: full `<inertial>` blocks
  (base_link 3100 kg, wheels 85 kg, etc.), collision boxes, and `ros2_control` `transmission` blocks
  using `hardware_interface/PositionJointInterface`. NOT part of the stable launch.

The real model must decide which of these it extends:
- **Extend the kinematic URDF** (recommended for the immediate task): add real meshes/inertials/joint
  limits but keep the custom kinematic bridge. The plugin drives joints by name, so this "just works"
  as long as names are preserved.
- **Adopt `ros2_control`** (only later, and only if the hardware actually has encoders — it does not):
  requires a `gazebo_ros2_control` plugin + controller manager, which is currently out of scope.

## Mesh / asset expectations

Current meshes are coarse STLs under `meshes/visual/` (and a few `meshes/collision/`), tracked by
`meshes/mesh_inventory.csv` (triangles, vertices, bounds). A real model will replace these with the
exported geometry. Expectations:

- Use `package://oil_palm_harvester_description/meshes/...` URIs — the launch file rewrites
  `model://oil_palm_harvester_description/...` to absolute `file://` paths at SDF conversion time, so
  package URIs resolve reliably regardless of `GAZEBO_MODEL_PATH`.
- Prefer COLLADA (`.dae`) for colored/multi-material meshes if fidelity matters; STL is fine for
  single-color collision geometry (STL carries no material).
- Keep `<collision>` geometry simpler than `<visual>` (convex hulls or boxes) to avoid ODE cost and
  interpenetration.
- If a mesh fails to render, the `render_mode:=primitive` fallback (collision-primitive visuals) is
  the diagnostic path — see `harvester-simulation`.

## Sensor mounts to preserve (frozen topics)

The real URDF may relocate sensors, but keep these frames/topics:

- `vehicle_lidar_link` → `/harvester/lidar/raw_points` (and the RViz copy `/harvester/lidar/points`).
- `platform_depth_camera_*` → cutter camera topics.
- `front_depth_camera_*` → docking camera topics.
- five docking range sensor links → `/harvester/{center,left_45,right_45,left_side,right_side}_range`.
- `cutting_tool_left_range_sensor_link` → `/harvester/cutting_tool_left_range` (stays separate from the
  five-sensor calibration).

The manual mount-adjustment points (which URDF joint `xyz`/`rpy` to edit) are documented in
`MODEL_ASSUMPTIONS.md` — reuse them for the real model.
