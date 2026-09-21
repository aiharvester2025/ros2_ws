# harvester_dock

Interactive docking **orchestrator** for the oil-palm harvester, used for
**simulation validation, idea testing, and experimentation**.

## Role (important)

This package **auto-actuates** the harvester: it sweeps the cutting-arm LiDAR,
runs the live tree-height + platform→trunk distance estimation, and commands the
boom joints to dock/undock.  It is deliberately **retained as a simulation
harness**.

> On the real machine the flow is **human-in-the-loop**: the operator performs
> the sweep, moves the boom, and levels/docks, while the software only *guides*,
> *advises*, and *monitors*.  A separate human-in-the-loop system that mimics
> this inside the simulation will be built later.  See
> [`docs/BOOM_DOCK_PLAN.md`](../docs/BOOM_DOCK_PLAN.md) for the deployment
> requirements and the research findings carried forward to the physical
> hardware.

## Components

| File | Role |
|---|---|
| `dock_orchestrator.py` | 4-state FSM (SWEEP → PLAN → DOCK → UNDOCK). ZMQ SUB on `tcp://127.0.0.1:5593` (DOCK presses); subscribes `/harvester/lidar/points` (zero-stamped, levels per scan during the dwell phase); publishes `/harvester/joint_commands` and `/harvester/dock/status`. |
| `live_height_estimator.py` | ROS-free port of the offline tree-height estimator, with the half-cylinder axis correction, `max` top envelope, and crown-base (trunk-end) density detection. |
| `distance_estimator.py` | Platform→trunk horizontal distance + reachability advisory. |
| `approach_driver.py` | **Simulation-only** validation harness: drives the c-channel platform toward the trunk at a controllable closing speed (ramps the boom extension) and logs the forward gap, derived closing speed, and safety-guidance state. Run it exclusively (slider GUI + orchestrator both off). |
| `config/dock.yaml` | Sweep band (short nod), crown-base density threshold, docking offset (2.0 m below crown base), home pose, safety thresholds. |

The IK and safety logic are imported from `harvester_boom_plan` (not duplicated).

## Run

Four processes.  **Two launch flags are required** for the autonomous dock flow
(they prevent the slider GUI and the older `boom_plan` stack from overriding the
orchestrator's joint commands):

- `joint_gui:=false` — the `joint_state_publisher_gui` continuously publishes
  slider (zero) positions to `/harvester/joint_commands`, overriding any scripted
  command (this was the "no visible sweep / stale values" bug).
- `boom_plan:=false` — the older `boom_plan` stack also publishes to
  `/harvester/joint_commands` / `/harvester/boom/plan`, conflicting with the
  orchestrator.

Each terminal needs `source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash`
first, **except T4** (the dashboard uses `/usr/bin/python3`, not the ROS env).

```bash
# T1 — Gazebo headless + RViz, slider GUI off, old boom_plan stack off
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  gui:=false rviz:=true harvester_collision_mode:=off \
  articulation_control_mode:=kinematic docking_camera:=true \
  joint_gui:=false boom_plan:=false

# T2 — telemetry gateway
ros2 launch harvester_telemetry_gateway gateway.launch.py

# T3 — dock orchestrator (auto-actuates)
python3 -m harvester_dock.dock_orchestrator

# T4 — dashboard (system python) with DOCK button; DISPLAY is your xrdp session
cd ~/ros2_ws
DISPLAY=:10 PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600 \
  --dock-pub tcp://127.0.0.1:5593
```

Press **DOCK** three times: sweep+estimate (amber) → dock (green) → undock (red).

> **Note on the display:** this environment's desktop is the xrdp session on
> `:10` (2560×1440).  `:0` is a headless 640×480 fallback, and `:1` does not
> exist — use `DISPLAY=:10`.

## Test

```bash
PYTHONPATH=src/harvester_dock:src/harvester_boom_plan \
  python3 -m pytest src/harvester_dock/test -v
```

## Key research findings (carried to physical hardware)

See `docs/BOOM_DOCK_PLAN.md` §"Research findings for physical-hardware
implementation".  Highlights:

- **1 short aimed nod (3–5 steps) suffices**; the LiDAR's 59° vertical FOV
  already spans the trunk's ~62° angular height at standoff.
- The **half-cylinder axis bias** (`centre = median + (2/π)·radius`) is the
  dominant estimation error and is physical (present on real hardware too).
- The **`max` trunk-top envelope** is correct; percentiles are biased low.
- The LiDAR is rigid on the cutting arm → pitch = lift angle, so leveling can
  use the known lift angle (encoders/IMU) instead of a flaky TF lookup.
