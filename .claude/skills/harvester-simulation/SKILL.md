---
name: harvester-simulation
description: "Build, run, or modify the ROS2 Foxy + Gazebo Classic 11 + RViz oil-palm harvester simulation (oil_palm_harvester_description): the kinematic URDF, combined launch, Gazebo bridge plugin, TF tree, and sensor mounts."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Harvester Simulation (ROS2 + URDF + Gazebo + RViz)

Operate and modify the foundational simulation that makes the oil-palm harvester move in Gazebo and
RViz: the kinematic URDF, the combined launch, the Gazebo bridge plugin, and the TF/description
graph. Every other skill (boom-dock, tree-height, telemetry, dashboard, LiDAR HUD) builds on top of
this package.

This skill records the **non-obvious launch/TF/control-path invariants and the stability rules** that
a smaller model would otherwise break — the flying-robot bug, the description-topic collision, the
GUI-vs-bridge command split, and the port-11345 single-instance rule.

## When this applies

- Editing `src/oil_palm_harvester_description/**` (URDF, launch, plugin, scripts, RViz configs).
- Adding/moving a joint, sensor mount, or changing the control path.
- Debugging "robot flies / joints oscillate / Gazebo exit 255 / RViz shows tree as robot / slider
  doesn't move Gazebo".

## Core facts (internalize before editing)

<critical>
- **Two URDFs; only one is active.** `urdf/oil_palm_harvester_kinematic.urdf` is the shared
  Gazebo+RViz source. `urdf/oil_palm_harvester_estimated.urdf` contains `gazebo_ros2_control` and
  different physics assumptions — do NOT substitute it into the combined launch.
- **Control path split (never collapse):** the `joint_state_publisher_gui` owns
  `/harvester/joint_commands` (targets); the Gazebo bridge publishes **measured**
  `/harvester/joint_states`; `robot_state_publisher` + RViz follow the *measured* stream, not the
  slider target. The GUI continuously re-publishes slider (zero) positions, so any scripted command
  needs `joint_gui:=false`.
- **Stability rules are load-bearing.** `harvester_collision_mode:=off` (collision bodies removed) +
  `articulation_control_mode:=kinematic` (20 Hz rate-limited changed-joint batches, velocity reset).
  Never reintroduce a physics-rate direct joint-position loop or unbounded PID — that caused the
  `ODESliderJoint` log flood and robot flight. `pid` is a diagnostic fallback only.
- **Do not add a static `world -> base_link` TF.** The plugin owns that transform (moves the base via
  `/harvester/cmd_vel`). A static one breaks base motion.
- **Run one Gazebo instance only** (port `11345`). Two launches leave RViz on a stale tree world or
  exit 255 (`bind: Address already in use`).
</critical>

## Reference map

- **Control path, TF tree, description topics, movable joints:** [references/control-and-tf.md](references/control-and-tf.md)
- **Launch flags, build/run, sensor inventory, troubleshooting:** [references/launch-and-sensors.md](references/launch-and-sensors.md)

## Quick workflow

1. Read `SIMULATION_HANDOFF.md` (in this package) before changing the launch, TF, sensor mounts, or
   controller path — it is the checklist and the authoritative baseline.
2. Read `MODEL_ASSUMPTIONS.md` for geometry/coordinate/frame conventions and the manual sensor-mount
   adjustment points (which URDF joint `xyz`/`rpy` to edit for each sensor).
3. Build: `colcon build --packages-select oil_palm_harvester_description --merge-install
   --symlink-install` then `source install/setup.bash`.
4. Launch: `ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py
   harvester_collision_mode:=off articulation_control_mode:=kinematic`.
5. Verify with the checklist in `references/control-and-tf.md` before changing anything.

## Key invariants

- `/robot_description` belongs **only** to the harvester `robot_state_publisher`. The tree publisher
  must stay namespaced `/tree` (and its description on `/tree_description`) or RViz renders tree
  links as the robot.
- `world -> tree_base` is a static TF (`8.5,0,0`); the tree is a static environment object, not
  attached to `base_link`.
- `/harvester/cmd_vel` uses only `linear.x` and `angular.z`; the plugin stops base motion 0.5 s after
  the last command. No traction/suspension/inertia is simulated.
- `/harvester/lidar/raw_points` (sim-time) is the fusion/recording source; `/harvester/lidar/points`
  is a zero-stamped/latest-TF RViz-only copy — never use it for time-correlated work (except the dock
  orchestrator's dwell-leveling, which is a deliberate documented exception).
- Sensor raw topics are frozen; do not move a sensor or rename a raw topic while changing perception.
- This is a **sensor-development** model, not a force-accurate vehicle/cutting/collision simulation.

## Related skills

- `urdf-import` — replacing this placeholder URDF with a real, detailed harvester model (the future
  task).
- `sensor-calibration` — the range + camera/LiDAR calibration layered on this package's URDF/sensors.
- `boom-dock-ik` — the docking IK/safety built on the joints this bridge actuates.
- `tree-height-estimation` — the LiDAR sweep/height estimation driven through `cutting_arm_lift_joint`.
