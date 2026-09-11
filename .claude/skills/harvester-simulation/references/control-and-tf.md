# Control path, TF tree, and description topics

Source of truth: `launch/gazebo_harvester_and_tree.launch.py`,
`src/harvester_kinematic_gazebo_plugin.cpp`, and `SIMULATION_HANDOFF.md`.

## Control path (the split that must not be collapsed)

```text
joint_state_publisher_gui
      │  /harvester/joint_commands  (targets, GUI owns this topic)
      └──────────────────────────────► Gazebo harvester bridge plugin
                                              │
                                measured /harvester/joint_states
                                              └──► robot_state_publisher ─► RViz harvester model

/harvester/cmd_vel ───► Gazebo plugin ──► moves base + publishes world -> base_link TF

static_transform_publisher ───► world -> tree_base TF  (fixed 8.5,0,0)
tree_state_publisher (/tree) ───► tree TF frames
tree description publisher ───► /tree_description (RViz "Oil Palm Tree" display)
```

The GUI command stream and the Gazebo-measured feedback stream are **intentionally different**.
RViz and all sensor frames follow the measured pose, not the immediate slider target.

## Movable joints (and their limits)

```text
boom_turret_joint            (±0.35 rad; capped 0.05 rad/s by the plugin)
boom_elevation_joint         (0 … 1.309 rad)
boom_extension_1..4_joint    (prismatic, 0 … 2.4 m each)
platform_level_joint         (±1.57 rad)
rail_carriage_joint          (-2.55 … +2.55 rad)
cutting_arm_lift_joint       (-0.35 … +1.05 rad)
cutting_arm_extension_joint  (0 … 0.375 m)
```

The cutting-arm chain is **half-length in X only** (mesh X-scale 0.5, Y/Z unchanged): extension
origin 0.31 m, stroke 0–0.375 m, cutter attachment 0.46 m beyond the extension link.

## Frames / ownership

| Object | Gazebo role | RViz role | Root frame |
|---|---|---|---|
| Harvester | Dynamic SDF model (generated at launch) | Harvester RobotModel | `base_link` (moved by `world -> base_link`) |
| Tree | Static SDF environment model | Tree RobotModel | `tree_base` (fixed `8.5,0,0` in `world`) |

- `world -> tree_base` is **static**, owned by the launch file.
- `world -> base_link` is **dynamic**, owned by the Gazebo plugin. Never add a static one.
- The tree is NOT attached to `base_link`.

## Description topics (collision is a real bug)

| Topic | Publisher | Consumer |
|---|---|---|
| `/robot_description` | harvester `robot_state_publisher` only | RViz "Harvester Robot" |
| `/tree/robot_description` | namespaced tree `robot_state_publisher` | (internal) |
| `/tree_description` | retained tree-description script | RViz "Oil Palm Tree" |

On Foxy, `robot_state_publisher` auto-republishes its description. If the tree publisher is
**un-namespaced**, its URDF lands on `/robot_description` and RViz renders tree links as the robot.
Keep the tree publisher namespaced `/tree` and remap its `robot_description` → `/tree_description`.

## The Gazebo bridge plugin (`harvester_kinematic_gazebo_plugin.cpp`)

Responsibilities: subscribe `/harvester/joint_commands`, apply changed articulated-joint targets
kinematically at 20 Hz, publish measured `/harvester/joint_states`, subscribe `/harvester/cmd_vel`,
integrate base pose, publish `world -> base_link` TF, stop base after 0.5 s idle.

Stability rules (do not undo):
- `kinematic` mode ramps targets at 20 Hz using URDF joint + velocity limits; turret capped 0.05 rad/s.
- Applies one changed-joint batch then clears residual Gazebo velocity/force **once**; never replays a
  full joint map every physics update.
- `pid` is a fresh-launch diagnostic fallback only, NOT the recommended mode.

## Verification checklist (run before changing code)

```bash
gz model -l                                        # must list oil_palm_harvester and oil_palm_tree
ros2 topic info /robot_description -v              # ONE harvester publisher only
ros2 topic info /tree/robot_description -v         # tree description must be namespaced
ros2 topic list | grep -E '/harvester/(joint_commands|joint_states|cmd_vel)'
ros2 run tf2_ros tf2_echo world tree_base          # static
ros2 run tf2_ros tf2_echo world base_link          # live while Gazebo runs
```

Expected: moving a GUI slider changes both the RViz harvester and the Gazebo harvester.
