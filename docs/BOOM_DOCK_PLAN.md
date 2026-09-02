# Boom Docking Plan Handoff

## Purpose and safety boundary

This document hands off the boom distance / angle / extension estimation and the
c-channel docking maneuver.  It records the **closed-form inverse kinematics**,
the **safety state machine**, and — most importantly — the **non-obvious
conventions** that a future maintainer must not regress.

The package is `src/harvester_boom_plan/`.  It is **strictly read-only with one
exception**:

| Node | May write `/harvester/joint_commands`? | Role |
|---|---|---|
| `boom_plan_node` | **No** (never) | Closed-form IK + 5-range safety estimator |
| `tree_docking_estimate_node` | **No** (never) | Publishes `/harvester/tree/docking_estimate` (static trunk top) |
| `boom_docking_demo` | **Only with `--execute`** | Dry-run executor by default |

This mirrors the `TELEMETRY_HANDOFF.md` boundary: the estimator publishes
observation/advice topics only; joint actuation is a separate, conscious,
opt-in step.

## Goal

Given a tree-height (trunk-top) estimate and the trunk horizontal position,
compute the joint targets that place the c-channel platform onto the trunk:

```
theta_b  = atan2(H_dock - pivot_world_z, d_horiz - L_plat)   # boom elevation
e        = max(0, sqrt(dx^2 + dz^2) - L_fixed)               # total extension
level    = +theta_b   (active)                               # platform level joint
theta_d  = 0 at the solved configuration                     # diagram step-5 lower
```

Docking point (user decision, **offset = 2.0 m below trunk top**):

```
H_dock = trunk_top - 2.0 m  = 12.0 - 2.0 = 10.0 m   (reference tree)
```

Docking reference point:
- **Tree side:** trunk **centreline** at `(trunk_x, trunk_y, H_dock)`.
- **Platform side:** `c_channel_reference`, whose +X points through the C-opening
  toward the tree.  The IK places `c_channel_reference` exactly at the trunk-centre
  docking point.

## URDF-derived constants (do NOT hard-code elsewhere)

These are resolved from `oil_palm_harvester_kinematic.urdf` at import time in
`harvester_boom_plan/kinematics.py` and validated by
`test_urdf_constants_consistent`:

| Constant | Value | Source joint |
|---|---|---|
| `BOOM_PIVOT_HEIGHT` | 1.76 m | `boom_turret_joint` z (1.02) + `boom_elevation_joint` z (0.74) |
| `BOOM_FIXED_LENGTH` | 2.40 m | `platform_level_joint` origin x |
| `PLATFORM_TAIL` | 1.02 m | `platform_fixed_joint` origin x |
| `EXTENSION_PER_STAGE` | 2.40 m | `boom_extension_1_joint` limit |
| `ELEVATION_LIMIT` | (0, 1.309) rad | `boom_elevation_joint` |
| `LEVEL_LIMIT` | (-1.57, 1.57) rad | `platform_level_joint` |

## Non-obvious conventions (the two simulation-discovered bugs)

These are the exact reasons the experiment initially failed and must not be
reverted:

1. **Leveling sign is `+theta_b`, not `-theta_b`.**  `boom_elevation_joint`
   axis is `(0,-1,0)` while `platform_level_joint` axis is `(0,+1,0)`.  A
   positive boom angle pitches the tip UP; to counter-rotate the platform mount
   back to horizontal we command `platform_level_joint = +theta_b`.  Using
   `-theta_b` leaves the platform tilted ~89° and ~1.47 m off-target.

2. **The boom pivot world Z includes the base-link offset.**  The Gazebo base
   parks `base_link` at world `z = 0.05 m`, but `BOOM_PIVOT_HEIGHT = 1.76 m` is
   expressed in the `base_link` frame.  The IK must use
   `BOOM_PIVOT_WORLD_Z = 0.05 + 1.76 = 1.81 m`, otherwise the docking height is
   0.05 m too high.

3. **Range sensors report `3.4e38` (= `FLT_MAX`) for "no return".**  This is
   `> max_range = 3.0 m`, not a valid measurement.  `boom_plan_node` rejects
   `range > max_range` and leaves the sensor as `None`, so a far/absent sensor
   cannot pollute the centering diameter (which otherwise becomes `6.8e38`).

## Safety state machine (two-phase semantics)

`harvester_boom_plan/safety.py` combines two roles over the five fixed docking
ranges plus the cutter range:

1. **Clearance/skew watchdog** — trips `EMERGENCY_STOP` on near-contact
   (`<= emergency_m = 0.05 m`), `HOLD_AT_DISTANCE` when front clearance drops
   below `extend_stop_m = 0.4 m`.
2. **Trunk-centering confirmation** — the side pair and ±45° pair reconstruct
   the trunk centreline inside the C-opening; **both must pass** to admit
   `DOCKING_OK`.  A missing ±45° pair now correctly **blocks** `DOCKING_OK`.

Phase priority (highest first): `WAITING_FOR_TREE` → `EMERGENCY_STOP` →
`DOCKING_OK` → `LEVEL_OK`/`ALIGN_OK` → `HOLD_AT_DISTANCE`/`EXTEND_OK` → `READY`.

**Two-phase gating** (in `boom_docking_demo`): the raise/extend phase is
open-loop (sensors legitimately out of range → `WAITING_FOR_TREE` must NOT
block).  Only `EMERGENCY_STOP` and `INFEASIBLE_DOCK_HEIGHT` hard-block.

**Post-dock observation:** once the trunk is inside the C-opening, the forward
`center_range` reads ~0.23 m (correct final contact) while the side/±45° beams
pass the trunk and report "no return".  The centre sensor alone confirms final
contact; the side pair matters during **approach**.

## Topic surface

| Topic | Type | Direction |
|---|---|---|
| `/harvester/tree/docking_estimate` | `geometry_msgs/PointStamped` | sub (trunk-top, world) |
| `/harvester/{center,left_45,right_45,left_side,right_side}_range` | `sensor_msgs/Range` | sub (watchdog + centering) |
| `/harvester/cutting_tool_left_range` | `sensor_msgs/Range` | sub (kept separate from the 5-sensor estimator) |
| `/harvester/boom/plan` | `std_msgs/String` (JSON) | pub (full plan) |
| `/harvester/boom/safety_state` | `std_msgs/String` | pub (phase) |

The plan JSON keys include `boom_angle_rad`, `leveling_angle_rad`,
`boom_extension_total_m`, `boom_extension_per_stage_m`,
`docking_lower_angle_rad`, `phase`, and `centering`.

## Run

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select harvester_boom_plan --merge-install --symlink-install
source install/setup.bash

# Full simulation (headless): harvester + tree + boom-plan stack
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  gui:=false rviz:=false harvester_collision_mode:=off \
  articulation_control_mode:=kinematic docking_camera:=false

# IMPORTANT: stop the GUI, it overrides scripted joint commands (see
# TREE_HEIGHT_SCAN.md "The GUI overrides commands").
pkill -f joint_state_publisher_gui

# Execute the plan (moves the boom)
python3 -m harvester_boom_plan.boom_docking_demo --execute

# Measure accuracy (independent FK + TF ground truth)
python3 src/harvester_boom_plan/harvester_boom_plan/dock_measurer.py
ros2 run tf2_ros tf2_echo world c_channel_reference
```

## Validation

- 18 unit tests pass (`test/test_kinematics.py`, `test/test_safety.py`).
- Live experiment result: `c_channel_reference = (8.500, -0.000, 10.000)`
  (both FK and TF ground truth) vs. target `(8.5, 0, 10.0)` → **0.000 m error**.

## Authoritative files

- `src/harvester_boom_plan/harvester_boom_plan/kinematics.py` — IK + constants.
- `src/harvester_boom_plan/harvester_boom_plan/safety.py` — safety state machine.
- `src/harvester_boom_plan/harvester_boom_plan/boom_plan_node.py` — estimator node.
- `src/harvester_boom_plan/config/boom_docking.yaml` — docking offset + thresholds.
- `src/oil_palm_harvester_description/urdf/oil_palm_harvester_kinematic.urdf` — geometry.

## Remaining next steps

1. Feed a real LiDAR-derived trunk-top estimate into `/harvester/tree/docking_estimate`
   (currently a static 12.0 m ground truth in `tree_docking_estimate_node.py`).
2. Add a dashboard read-out / canonical ZMQ channel for `/harvester/boom/plan`.
3. Confirm the real-machine leveling mechanism (active vs. passive stabilizer);
   the `leveling_mode` config flag defaults to `active` for this simulation.
