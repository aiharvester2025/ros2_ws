---
name: boom-dock-ik
description: "Plan, implement, or debug the oil-palm harvester boom docking stack (closed-form boom IK, docking safety state machine, crown-base height estimation, Gazebo joint-command execution) in this ROS2 Foxy workspace."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Boom Docking IK & Estimation

Compute the boom joint targets (elevation `theta_b`, extension `e`, leveling `theta_L`) that
place the harvester's c-channel platform onto an oil-palm trunk, and operate the read-only
`harvester_boom_plan` / autonomous `harvester_dock` packages in this workspace.

This skill captures the **non-obvious, repo-specific invariants** a model would otherwise have
to rediscover: the URDF-derived kinematic constants, the leveling-sign gotcha, the crown-base
(not tree-top) docking-height rule, and the Gazebo plugin's joint-command execution quirks.

## When this applies

- Writing or editing `src/harvester_boom_plan/**` or `src/harvester_dock/**`.
- Adding/changing a boom joint target, safety threshold, or docking-height calculation.
- Debugging why commanded boom joints do not move in Gazebo.
- Extending the SWEEP → PLAN → DOCK → UNDOCK docking flow.

It does **not** apply to telemetry/vision/dashboard work, the LiDAR HUD, or the tree-height
*scanning* pipeline itself (those have their own handoffs in `docs/`).

## Core facts to internalize (load before editing)

<critical>
- **Docking height MUST come from the crown base (trunk-end), never `tree_top - offset`.**
  The reference tree top is z=12.0 m but the crown base is z=9.2 m; `12.0 - 2.0 = 10.0 m` lands
  inside the frond (9.45 m) / FFB (9.55 m) zone and is exactly why the platform crashed into the
  canopy. Docking height = `crown_base - 2.0 m ≈ 7.2 m`.
- **Leveling sign is `+theta_b`, not `-theta_b`.** `platform_level_joint` axis is `(0,+1,0)`,
  OPPOSITE the boom elevation axis `(0,-1,0)`. Commanding `-theta_b` leaves the c-channel ~89°
  tilted and ~1 m off-target; `+theta_b` recovers the target.
- **`boom_plan` is strictly read-only.** It never publishes to `/harvester/joint_commands`.
  Only the `harvester_dock` orchestrator and the opt-in `boom_docking_demo --execute` may command
  joints. Never run the advisor and the autonomous orchestrator at the same time — they fight over
  the joint-command topic.
- **Two roles, both retained (deployment model).** The `harvester_dock` orchestrator
  **auto-actuates** and is retained deliberately as the *simulation validation / idea-testing
  harness*. On the physical machine the flow is **human-in-the-loop**: a human operator performs the
  sweep, moves the boom, and levels/docks, while the software only **guides** (sweep steps),
  **advises** (target θ_b / distance / extension), and **monitors** (safety state + current-vs-target).
  The real machine has no joint encoders and no joint cmd — control is hydraulic via solenoid/PLC/valve.
  Do NOT auto-actuate in deployment; do NOT treat the orchestrator's auto-actuation as a production path.
</critical>

## Reference map

Read only what the current task needs:

- **Kinematics (constants, math, IKResult, InfeasibleDock):** [references/kinematics.md](references/kinematics.md)
- **Safety FSM + crown-base/height estimation:** [references/safety-and-crown-base.md](references/safety-and-crown-base.md)
- **Gazebo joint-command execution pitfalls:** [references/gazebo-plugin-debug.md](references/gazebo-plugin-debug.md)

## Quick workflow

1. **Understand the existing code before changing it.** The IK math, safety evaluator, and
   height estimator are all **ROS-free modules** (unit-testable without a graph) that live in
   `src/harvester_boom_plan/harvester_boom_plan/{kinematics,safety}.py` and
   `src/harvester_dock/harvester_dock/{live_height_estimator,distance_estimator}.py`. Read them
   and their configs (`config/boom_docking.yaml`, `config/dock.yaml`) first.

2. **Make the change in the right layer.** Kinematic math → `kinematics.py`; safety/centering →
   `safety.py`; docking-height source → `dock_orchestrator.py::_docking_height` (and the
   `boom_docking.yaml` NOTE); thresholds → the YAML (single source of truth).

3. **Build and unit-test.** The pure modules are tested with plain pytest, no ROS graph needed:
   ```bash
   cd ~/ros2_ws && source /opt/ros/foxy/setup.bash
   PYTHONPATH=src/harvester_boom_plan:src/harvester_dock \
     python3 -m pytest src/harvester_boom_plan/test src/harvester_dock/test -v
   ```
   Then rebuild the affected package(s):
   ```bash
   colcon build --packages-select harvester_boom_plan harvester_dock --merge-install --symlink-install
   source install/setup.bash
   ```

4. **If touching joint execution or the docking flow, read the Gazebo pitfalls reference first.**

## Key invariants (don't let these drift)

- URDF-derived constants in `kinematics.py` are resolved from
  `oil_palm_harvester_kinematic.urdf` at import time and fall back to documented literals;
  `test_kinematics.py::test_urdf_constants_consistent` asserts they still match, so a model
  tuning change is caught. Keep that test green — do not "just fix" it by editing the literal.
- The boom has **four** prismatic extension stages, each 0..2.4 m, total stroke 9.6 m, plus a
  2.4 m fixed length = 12.0 m max boom length. `split_extension` splits the total evenly.
- Five fixed range sensors (`center`, `left_45`, `right_45`, `left_side`, `right_side`) plus the
  moving cutter range feed the safety FSM. Near-contact (`<= emergency_m 0.05 m`) always wins.
- **Range sensors report `3.4e38` (= `FLT_MAX`) for "no return"**, which is `> max_range = 3.0 m`.
  `boom_plan_node` rejects `range > max_range` and leaves the sensor as `None`, so a far/absent
  sensor cannot pollute the centering diameter (which would otherwise read `6.8e38`). Preserve this
  `None`-on-out-of-range behavior.
- There are two mutually-exclusive docking modes at launch time (`boom_plan:=true` read-only
  advisor vs `boom_plan:=false` + `harvester_dock` autonomous). The autonomous dock flow also
  requires `joint_gui:=false` (the slider GUI continuously republishes zero positions to
  `/harvester/joint_commands`, overriding scripted commands) and `boom_plan:=false`. See
  `harvester_boom_plan/README.md` and `harvester_dock/README.md`.

## Validation

After a change, confirm the observable result, not just that tests pass:
- IK math recovers the reference case: tree at world X=8.5, docking height 7.2 m → `theta_b`
  feasible, `extension_total_m` within 9.6 m.
- `python3 -m harvester_boom_plan.boom_docking_demo --execute` actually moves joints in a running
  (unpaused) Gazebo, and `ros2 topic echo` on the correct namespaced joint topic shows messages.

## Related skills

- `harvester-simulation` — the ROS2/URDF/Gazebo/RViz foundation and the joint-command bridge this
  package actuates through (`/harvester/joint_commands`, `joint_gui:=false`).
- `tree-height-estimation` — the crown-base/trunk-axis estimation that feeds `_docking_height`.
- `telemetry-gateway` — publishes `/harvester/dock/status` as `v1/docking/plan` (observation only).
