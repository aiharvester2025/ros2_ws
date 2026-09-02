# harvester_boom_plan

Read-only boom distance/angle/extension estimation for the oil-palm harvester,
plus a docking-safety watchdog.  This package turns a tree-height estimate into
the boom joint targets needed to place the c-channel platform onto the trunk,
**without ever issuing a joint command**.

> Full handoff (conventions, safety boundary, experiment results): see
> [`docs/BOOM_DOCK_PLAN.md`](../docs/BOOM_DOCK_PLAN.md).

## What it computes

Given a docking point (trunk centreline at `trunk_top − 2.0 m`), it solves the
closed-form boom inverse kinematics from the active URDF:

```
theta_b      = atan2(H_dock - pivot_world_z, d_horiz - L_plat)  # boom elevation
e            = max(0, sqrt(dx^2 + dz^2) - L_fixed)              # total extension
theta_L      = +theta_b   (leveling correction)                 # active leveling
d_b          = d_horiz - L_plat                                 # horizontal distance
```

where `pivot_world_z = 1.81 m` (boom pivot height 1.76 m + base-link world
offset 0.05 m), `L_fixed = 2.4 m`, `L_plat = 1.02 m`, and `d_horiz` is the
horizontal distance from the boom pivot to the trunk.

> **Note the leveling sign is `+theta_b`.**  The `platform_level_joint` axis is
> `(0,+1,0)`, opposite the boom's `(0,-1,0)`, so the level joint must be
> commanded `+theta_b` to counter-rotate the platform to horizontal (see
> `docs/BOOM_DOCK_PLAN.md`).

## Nodes

| Node | Role | Topic |
|---|---|---|
| `tree_docking_estimate_node` | Publish the trunk-top estimate (static ground truth) | `/harvester/tree/docking_estimate` |
| `boom_plan_node` | Closed-form IK + 5-range safety evaluator (read-only) | `/harvester/boom/plan`, `/harvester/boom/safety_state` |
| `boom_docking_demo` | Dry-run executor (commands joints only with `--execute`) | subscribes `/harvester/boom/plan` |

## Build

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select harvester_boom_plan --merge-install --symlink-install
source install/setup.bash
```

## Run (standalone, no Gazebo)

```bash
ros2 launch harvester_boom_plan boom_plan.launch.py
```

The demo reports the plan in dry-run mode.  To actually command joints in a
running Gazebo simulation (the experiment), start the demo manually:

```bash
python3 -m harvester_boom_plan.boom_docking_demo --execute
```

## Run (with the full Gazebo + RViz scene)

The combined launch includes this stack by default:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic
```

Disable it with `boom_plan:=false`.

## Safety boundary

`boom_plan_node` and `tree_docking_estimate_node` are **strictly read-only**.
They never publish to `/harvester/joint_commands`, `/harvester/cmd_vel`, or any
Gazebo service.  Only `boom_docking_demo` may write joint commands, and only
when started with `--execute`.

## Test

```bash
PYTHONPATH=src/harvester_boom_plan python3 -m pytest src/harvester_boom_plan/test -v
```

## Docking reference point

- **Tree side:** trunk centreline at the docking height
  `H_dock = trunk_top − 2.0 m = 10.0 m` (reference tree trunk top is 12.0 m).
- **Platform side:** `c_channel_reference`, whose +X points through the C-opening
  toward the tree.  The IK goal places `c_channel_reference` exactly at the
  trunk-centre docking point.
