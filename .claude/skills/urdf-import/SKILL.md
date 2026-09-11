---
name: urdf-import
description: "Import and simulate a real, detailed oil-palm harvester URDF (actual meshes, masses, inertials, joint axes, sensor mounts) into this ROS2 Foxy + Gazebo Classic 11 workspace, replacing the current estimated/kinematic placeholder model."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# URDF Import & Real-Model Simulation

Take a future real/detailed harvester URDF and bring it into the existing Gazebo + RViz simulation
without regressing the working baseline. The current model is a placeholder derived from orthographic
drawings (`oil_palm_harvester_kinematic.urdf`, no real masses, coarse STL meshes); the future task is
to substitute a geometry-faithful model while preserving the control path, TF graph, and sensor topics
that every other skill depends on.

This skill records the **migration constraints and conversion gotchas** that are specific to this
codebase: what the Gazebo plugin hardcodes (joint names, `base_link`, the turret special-case), what
the launch file's `gz sdf -p` pipeline expects, and how to stage the real model without breaking the
sensor-development baseline.

## When this applies

- Importing a new real harvester URDF (with actual meshes/inertials/joints) into the workspace.
- Converting a detailed URDF to SDF for Gazebo, or re-mounting sensors onto the new geometry.
- Migrating from `oil_palm_harvester_kinematic.urdf` / `oil_palm_harvester_estimated.urdf` to a
  production-faithful model.

## Core facts (internalize before editing)

<critical>
- **The Gazebo bridge plugin is name-coupled.** `harvester_kinematic_gazebo_plugin.cpp` matches joints
  by name (`joint->GetName()`), special-cases exactly one joint — `kTurretJointName =
  "boom_turret_joint"` — and publishes `world -> base_link` with hardcoded frame names. A real URDF
  **must keep the same joint names and the `base_link` root name**, or the bridge silently stops
  driving the new joints. Any rename requires a coordinated plugin edit.
- **The launch file converts URDF→SDF with `gz sdf -p` at startup** and injects the result into a
  temp world. It rewrites `model://oil_palm_harvester_description/...` mesh URIs to absolute `file://`
  paths. A real URDF's meshes must resolve to the installed `meshes/` share directory (package URIs are
  preferred; they get rewritten automatically).
- **Do not break the frozen raw sensor topics.** The real URDF may relocate sensors, but
  `/harvester/lidar/raw_points`, the five `*_range` topics, the camera topics, and
  `/harvester/cutting_tool_left_range` are load-bearing for the gateway, calibration, docking, and
  dashboard skills. Re-mount, don't rename.
- **Stage it, don't swap it in one shot.** Keep the kinematic baseline as the launch default; add the
  real model behind a new launch flag/separate launch file, validate it, then cut over. Never overwrite
  `oil_palm_harvester_kinematic.urdf` in place while it is the active baseline.
</critical>

## Reference map

- **What a real model must provide (geometry/inertial/joint checklist):** [references/model-requirements.md](references/model-requirements.md)
- **Conversion pipeline + plugin/launch constraints:** [references/conversion-and-plugin.md](references/conversion-and-plugin.md)
- **Staging & validation procedure:** [references/staging-and-validation.md](references/staging-and-validation.md)

## Quick workflow

1. Read `MODEL_ASSUMPTIONS.md` (the "Required information for an accurate conversion" section lists
   exactly what a real model must supply) and `SIMULATION_HANDOFF.md` before touching anything.
2. Inventory the incoming URDF: joint names vs the current set, mesh paths, inertial blocks, sensor
   mounts, and whether it is a kinematic or `ros2_control`-style model.
3. Decide the migration path: (a) re-mount sensors onto the new geometry but keep the kinematic bridge,
   or (b) adopt the physical model's own controller (only if encoders/`ros2_control` are actually
   present — they are NOT on the real machine).
4. Stage under a new filename + launch flag, validate offline (`check_urdf`, `gz sdf -p`), then do a
   live launch and run the verification checklist before cutting over.

## Key invariants

- Joint names to preserve (the plugin drives them): `boom_turret_joint`, `boom_elevation_joint`,
  `boom_extension_1..4_joint`, `platform_level_joint`, `rail_carriage_joint`, `cutting_arm_lift_joint`,
  `cutting_arm_extension_joint`. Root link must remain `base_link`.
- The `oil_palm_harvester_estimated.urdf` already has inertials + collision boxes + `ros2_control`
  `PositionJointInterface` transmissions — it is a useful *template* for a physical model, but its
  `gazebo_ros2_control` path is NOT part of the stable combined launch. Do not blindly enable it.
- This is still a sensor-development sim, not a force-accurate one. A real mass/inertial set changes
  nothing unless you also revisit the collision/physics mode (`harvester_collision_mode`) — which the
  handoff says to keep `off` until a proper physical controller exists.
