# Interactive DOCK Button — LiDAR Sweep → Height/Distance Estimate → Dock/Undock

**Date:** 2026-09-03
**Workspace:** `/home/ubuntu/ros2_ws`
**Status:** Implemented (simulation validation / experimentation harness — retained)

> **2026-09-03 — retained as the simulation harness.**  The `harvester_dock`
> orchestrator **auto-actuates** (sweeps, estimates, commands joints) and is
> **deliberately kept** for simulation testing, idea testing, and experimentation.
> On the real machine a **human operator** performs the sweep, moves the boom, and
> levels/docks; the software only *guides*, *advises*, and *monitors*.  A
> **human-in-the-loop system that mimics this flow inside the simulation** will be
> built later.  See
> [`docs/BOOM_DOCK_PLAN.md`](docs/BOOM_DOCK_PLAN.md) §"Real-machine deployment:
> human-in-the-loop operating model" and §"Research findings for
> physical-hardware implementation" for the full deployment requirements and the
> research findings relevant to the physical machine.

---

## 1. Goal

Add a **single "DOCK" HUD button** to the operator dashboard that drives the
whole docking workflow with three presses, each a distinct colour/state:

| Press | Button state | Action |
|---|---|---|
| 1st | `SWEEP` (e.g. amber) | Sweep the arm LiDAR → accumulate point cloud → estimate **tree height** and **platform-to-trunk distance** → compute docking point, boom angle, boom extension |
| — | (auto) | If the distance is **not suitable** (outside the reachable envelope), show "move prime mover" guidance on the dashboard and **reset to `SWEEP`** |
| 2nd | `DOCK` (e.g. green) | Raise the boom, level the platform, extend, and **dock autonomously** (visible in RViz) |
| 3rd | `UNDOCK` (e.g. red) | Undock safely: retract extension, lower boom, return platform + boom to home, then **reset the button to `SWEEP`** |

The goal of this plan is to make the workflow **live and interactive end to end**:
measure (LiDAR) → estimate (height + distance) → plan (boom IK) → actuate (dock)
→ recover (undock), all from one HUD button.

---

## 2. Current state (verified in this workspace)

### 2.1 What already exists and works

- **Boom IK** — `harvester_boom_plan/kinematics.py` (closed-form `θ_b`, `e`,
  `θ_L=+θ_b`), validated to **0.000 m** docking error in simulation
  (`docs/BOOM_DOCK_PLAN.md` §13).
- **Docking executor** — `harvester_boom_plan/boom_docking_demo.py` publishes
  `/harvester/joint_commands` when run with `--execute`.
- **Height estimation** — `analyze_tree_scan_v2.py` (offline) reads recorded
  `.msgpack` `v1/lidar/raw` frames and produces fused tree height + trunk axis
  (`fit_trunk_axis`, `trunk_top_estimate`). Ground truth 12.0 m / crown 9.2 m.
- **LiDAR sweep** — `tree_scan_sweeper.py` sweeps only `cutting_arm_lift_joint`
  (−0.35 → +1.05 rad) and subscribes to `/harvester/lidar/raw_points`.
- **Dashboard** — `harvester_dashboard` (QML + `bridge.py` + ZMQ), renders
  camera/LiDAR/range/trunk. Read-only by contract.
- **Telemetry gateway** — `harvester_telemetry_gateway` subscribes to ROS
  topics and publishes canonical ZMQ (`5590` PUB, `5600` status REP).

### 2.2 The gap this plan closes

1. Height estimation is **offline** (reads `.msgpack` from disk), not live.
2. The dashboard has **no command path** — `canonical_zmq_v1.md` and
   `TELEMETRY_HANDOFF.md` explicitly forbid motion commands on the canonical
   `5590/5600` endpoints.
3. The tree-top input to `boom_plan_node` is a **static** `12.0 m`
   (`tree_docking_estimate_node.py`), not the measured height.
4. There is no orchestration tying sweep → estimate → plan → dock → undock.

---

## 3. Architecture (new components)

```
Dashboard (QML) ──DOCK click──► DockCommandPublisher (ZMQ PUB, out-of-contract)
                                        │  tcp://127.0.0.1:5593  "v1/operator/dock_request"
                                        ▼
                        DockOrchestrator (ROS 2 node, new)
                        ┌──────────────────────────────────────────────────────┐
                        │ state machine: SWEEP → PLAN → DOCK → UNDOCK → SWEEP    │
                        │  - commands cutting_arm_lift_joint (sweep)             │
                        │  - accumulates /harvester/lidar/raw_points             │
                        │  - runs live height+axis estimation (port of analyze)  │
                        │  - publishes plan on /harvester/boom/plan (via node)   │
                        │  - publishes joint targets on /harvester/joint_commands│
                        │  - reports state/result on /harvester/dock/status      │
                        └──────────────────────────────────────────────────────┘
                                        │  /harvester/dock/status (ROS String)
                                        ▼
                     Gateway (new channel) → 5590 PUB "v1/docking/plan"
                                        │
                                        ▼
                        Dashboard bridge (new DOCK HUD button + plan readout)
```

### 3.1 Out-of-contract command channel (the key design decision)

The canonical ZMQ contract **must not** carry motion commands. Following the
existing `annotation_publisher.py` precedent (a separate, non-canonical PUB on
`tcp://127.0.0.1:5592`), the DOCK button uses a **new, dedicated PUB endpoint**:

- Dashboard `DockCommandPublisher` (analogous to `AnnotationPublisher`) binds
  `tcp://127.0.0.1:5593` and publishes `v1/operator/dock_request` JSON packets
  (`{action: "dock"}` — one button, three presses, same single action; the
  orchestrator owns the state machine and advances on each press).
- The **orchestrator** subscribes to `5593` (or, more robustly, receives the
  press via a tiny ROS bridge). **Decision:** the orchestrator is a ROS 2 node
  that itself subscribes to the `5593` ZMQ PUB directly (it already lives in
  the ROS/anaconda environment where `zmq` exists). This avoids a second hop.

The `canonical_zmq_v1.md` doc is **updated** to record `5593` as an
out-of-contract command channel (same status as `5592`), not a canonical v1
channel.

### 3.2 New package: `harvester_dock` (ROS 2, ament_python)

`src/harvester_dock/`:

| File | Role |
|---|---|
| `dock_orchestrator.py` | The 4-state FSM (SWEEP → PLAN → DOCK → UNDOCK). Subscribes to `5593` ZMQ and `/harvester/lidar/raw_points`; publishes `/harvester/joint_commands` and `/harvester/dock/status`. |
| `live_height_estimator.py` | ROS-free port of `analyze_tree_scan_v2.py` `fit_trunk_axis` + `trunk_top_estimate` (operates on accumulated NumPy XYZ, no disk I/O). Returns `(height_m, trunk_axis_xy_m, uncertainty_m)`. |
| `distance_estimator.py` | Platform→trunk horizontal distance from the trunk axis (trunk XY vs. the boom-pivot world X) + reachability check against `kinematics.MAX_BOOM_LENGTH`. |
| `config/dock.yaml` | Sweep params (steps, wait), docking offset (2.0 m), reachability margins, home-joint pose. |

Reuses `harvester_boom_plan.kinematics` for IK (import it; no duplication).

### 3.3 Dashboard changes (`harvester_dashboard`)

- `qml/HudOverlay.qml` (or `Dashboard.qml` toolbar): add a **DOCK button**
  (QtQuick 2 primitive `Rectangle` + `MouseArea`, matching existing style) with
  a colour + label bound to `bridge.dockState`.
- `bridge.py`: add `dockState` property + `dockStateChanged` signal +
  `@Slot dock_pressed()` that forwards to a new `DockCommandPublisher`
  (`--dock-pub tcp://127.0.0.1:5593`, default disabled like `--annotation-pub`).
- `config.py`: add `dock_endpoint` (`--dock-pub`), default `''`.
- Add plan readout: `bridge.dockPlanLine` (height, distance, boom angle,
  extension) rendered in the HUD, fed from a new canonical channel
  `v1/docking/plan` (see §3.4).

### 3.4 Telemetry gateway addition (one new outbound-only channel)

The gateway already publishes sensor → ZMQ. Add one **outbound** channel so the
dashboard can display the plan results (this is observation, not command, so it
is allowed on the canonical `5590` PUB):

- Subscribe `/harvester/dock/status` (`std_msgs/String`, JSON) and republish as
  `v1/docking/plan` (`codec: json`).
- Header fields: `schema_version 1`, `source_mode simulation`,
  `capabilities {dock.autonomous: true}`.

This keeps the command path (5593) and the observation path (5590) cleanly
separate, matching the existing contract.

---

## 4. The three-press state machine (detailed)

### State `SWEEP` (press 1, amber)

1. Orchestrator commands `cutting_arm_lift_joint` from −0.35 → +1.05 rad in N
   steps (reuse `tree_scan_sweeper.py` timing: ~40 steps, step wait ~2 s,
   LiDAR capture ~0.8 s) **only if not already swept**.
2. Accumulate `/harvester/lidar/raw_points` (PointCloud2 → XYZ) across the sweep.
3. On completion, run `live_height_estimator` → `H_est`, `(ax, ay)`, uncertainty.
4. Run `distance_estimator` → `d_horiz` (platform→trunk) and reachability.
5. Publish `v1/docking/plan` (via `/harvester/dock/status`) with
   `{height, distance, boom_angle, boom_extension, reachable}`.
6. Transition → `PLAN`.

### State `PLAN` (auto, between presses)

- If `reachable == false` (distance outside the boom envelope): publish
  `{action:"move_prime_mover", needed:"+X m"}` guidance on the status/plan
  channel, show a "MOVE PRIME MOVER" toast + direction on the dashboard, and
  **transition back to `SWEEP`** (button resets to amber).
- If `reachable == true`: hold, display the good values (distance, boom angle,
  extension) and **enable the button as `DOCK` (green)**.

### State `DOCK` (press 2, green)

1. Compute joint targets from the measured `H_est` and `d_horiz` using
   `harvester_boom_plan.kinematics.inverse_kinematics`.
2. Publish the full joint command set to `/harvester/joint_commands`
   (`boom_elevation_joint`, `platform_level_joint`, 4 × `boom_extension_N_joint`)
   — same command surface as `boom_docking_demo --execute`.
3. Watch the 5-range safety state (reuse `harvester_boom_plan.safety`) during
   the final approach; stop on `EMERGENCY_STOP`.
4. On `DOCKING_OK`/`HOLD_AT_DISTANCE` (arrived), transition → docked; button
   becomes `UNDOCK` (red).

### State `UNDOCK` (press 3, red)

1. Reverse sequence: retract the 4 extension joints → lower `boom_elevation_joint`
   → return `platform_level_joint` to 0 → return `cutting_arm_lift_joint` to its
   home value (the pre-sweep pose).
2. Confirm all joints at home (via `/harvester/joint_states`).
3. Transition → `SWEEP` and reset the button to amber.

---

## 5. Reuse decisions (avoid duplication)

- **Do not re-implement IK** — import `harvester_boom_plan.kinematics`.
- **Do not re-implement safety** — import `harvester_boom_plan.safety`.
- **Do not re-implement sweep timing** — mirror `tree_scan_sweeper.py` constants
  (single-joint-only `JointState` so the bridge leaves other joints untouched).
- **Port, not copy-paste**, `analyze_tree_scan_v2.py`'s `fit_trunk_axis` and
  `trunk_top_estimate` into `live_height_estimator.py` (operating on an in-memory
  array). Keep the offline script intact for regression comparison.

---

## 6. Key risks and mitigations

| Risk | Mitigation |
|---|---|
| **Command path violates safety contract** | Separate `5593` out-of-contract PUB; `canonical_zmq_v1.md` updated; the gateway never forwards `5593` to ROS; only the orchestrator (a deliberate actuator) consumes it. |
| **Two Python interpreters** (anaconda for ROS/zmq, `/usr/bin/python3` for PySide2) | Dashboard `DockCommandPublisher` uses `/usr/bin/python3` + apt `zmq`; orchestrator uses anaconda `zmq`. Both talk only via `5593` on localhost (mirrors `annotation_publisher` split). |
| **`joint_state_publisher_gui` overrides scripted commands** (confirmed root cause of "no visible sweep" + stale plan values) | Launch with **`joint_gui:=false`** (new flag) so the orchestrator has exclusive control of `/harvester/joint_commands`; alternatively `pkill -f joint_state_publisher_gui`. |
| **Older `boom_plan` stack conflicts with the orchestrator** (both publish to `/harvester/joint_commands` / plan topics) | Launch with **`boom_plan:=false`** for the autonomous dock flow; keep it `true` (default) only for the read-only advisor mode. |
| **Distance estimate frame confusion** (world vs base_link, 0.05 m base offset) | `distance_estimator` uses `kinematics.BOOM_PIVOT_WORLD_Z` and the trunk axis in world X; validate against the known tree at world X=8.5. |
| **LiDAR cloud frame** (sensor vs world) | Use the same `lidar_world_frame: "world"` / `lidar_transform_latest: true` convention as the gateway; accumulate **world-frame** points so the tree stands vertical (see `gateway.yaml`). |
| **Undock collides / oscillates** | Undock is a fixed reverse sequence with joint-limit checks; safety `EMERGENCY_STOP` still monitored; rate-limited bridge (20 Hz) bounds speed. |

---

## 7. How to run (the demo the user wants to see)

Four processes (Gazebo headless, RViz visible, dashboard visible).  **Two launch
flags are required** to stop conflicting publishers from overriding the
orchestrator: `joint_gui:=false` (the slider GUI) and `boom_plan:=false` (the
older advisor stack).  Each terminal sources ROS first, except T4 (dashboard).

```bash
# T1 — Gazebo headless + RViz (boom motion visible), slider GUI off, boom_plan off
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  gui:=false rviz:=true harvester_collision_mode:=off \
  articulation_control_mode:=kinematic docking_camera:=false \
  joint_gui:=false boom_plan:=false

# T2 — telemetry gateway (canonical 5590/5600 + new plan channel)
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch harvester_telemetry_gateway gateway.launch.py

# T3 — dock orchestrator (new) — subscribes 5593 + lidar, publishes joint cmds
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
python3 -m harvester_dock.dock_orchestrator

# T4 — dashboard (system python, NO ROS sourcing), DOCK button + plan readout
cd ~/ros2_ws
DISPLAY=:10 PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600 \
  --dock-pub tcp://127.0.0.1:5593
```

Click **DOCK** (amber) → watch LiDAR sweep + plan readout → **DOCK** (green) →
watch the boom raise/level/extend and dock in RViz → **UNDOCK** (red) → boom
returns home, button resets amber.

> **Display note:** this machine's desktop is the xrdp session on `:10`
> (2560×1440); `:0` is a headless 640×480 fallback and `:1` does not exist.

---

## 8. Implementation steps (ordered)

1. **`harvester_dock` package skeleton** (package.xml, setup.py, config).
2. **`live_height_estimator.py`** — port `fit_trunk_axis` + `trunk_top_estimate`;
   unit test against a synthetic cylinder cloud and (later) the recorded
   `tree_scan_002` audit for parity with the offline script.
3. **`distance_estimator.py`** — trunk-axis world X vs boom-pivot world X →
   `d_horiz`; reachability vs `MAX_BOOM_LENGTH`; unit test.
4. **`dock_orchestrator.py`** — 4-state FSM; ZMQ SUB (5593) + LiDAR sub; joint
   command publishing; `/harvester/dock/status` publisher; reuses boom IK +
   safety.
5. **Gateway** — subscribe `/harvester/dock/status`, republish `v1/docking/plan`.
6. **Dashboard** — `DockCommandPublisher` + `bridge.dockState`/`dock_pressed` +
   `HudOverlay` DOCK button (3 colours) + plan readout; wire `--dock-pub`.
7. **Contract doc** — update `canonical_zmq_v1.md` (5593 out-of-contract) and
   `TELEMETRY_HANDOFF.md` (new node, new channel, command boundary).
8. **Tests** — unit tests for estimator port; FSM state-transition tests
   (mock joints/lidar); a dashboard no-emit guard updated to prove `5593` is
   the only new outbound socket.
9. **End-to-end validation** — run §7; assert: (a) press 1 shows height ≈ 11.9 m
   and a plausible distance, (b) press 2 docks with c_channel at the dock point
   in RViz, (c) press 3 returns all joints to home and resets the button.

---

## 9. Acceptance criteria

- Press 1 produces a `v1/docking/plan` readout with tree height (≈ 11.9 m, not
  12.0) and a distance; if unreachable, shows "MOVE PRIME MOVER" and resets.
- Press 2 raises/levels/extends the boom and docks (RViz shows c_channel at the
  trunk); the 5-range safety stops motion on near-contact.
- Press 3 returns boom + platform to home and resets the button to amber.
- `canonical_zmq_v1.md` no longer implies motion commands can flow on 5590/5600;
  the new 5593 command channel is explicitly documented as out-of-contract.
- All existing boom-plan unit tests still pass; new estimator + FSM tests pass.

---

## 10. Out of scope

- Real hardware / Orin adapter (this is simulation-only).
- Automatic prime-mover repositioning (we only *advise* the operator; the
  operator physically moves the vehicle, then re-presses DOCK).
- Collision physics (kinematic mode, as today).
- Dashboard over the real `5591` replay path for the dock button (the button is
  live-simulation only; replay remains view-only).
- **Human-in-the-loop mimic** — a separate system that replaces the
  orchestrator's auto-actuation with operator action + dashboard feedback,
  mirroring the real machine.  Planned as a later task; see the handoff.

---

## 11. Research findings (carried forward for physical hardware)

The simulation experimentation produced findings that are directly relevant to
the real machine (geometry, sensor physics, estimation — not Gazebo plumbing).
They are recorded in
[`docs/BOOM_DOCK_PLAN.md`](docs/BOOM_DOCK_PLAN.md) §"Research findings for
physical-hardware implementation" and summarised here:

1. **Sweep passes needed:** 1 short aimed nod (3–5 steps) suffices; the 59° LiDAR
   FOV already spans the trunk's ~62° angular height at standoff.  A full-range
   sweep overshoots and adds no density.
2. **Mid-360 density:** 200,000 pts/s non-repetitive ≈ 2.9× the Gazebo grid, and
   it densifies on dwell → fewer passes, better trunk-top capture on hardware.
3. **Half-cylinder axis bias:** single-sided scan biases the trunk axis toward the
   sensor by ~radius; correct with `centre = median + (2/π)·radius`.  This is
   physical and affects docking-X accuracy on hardware too.
4. **Trunk-top estimator:** `max` is the correct upper envelope (percentiles are
   biased low).
5. **LiDAR is rigid on the cutting arm:** pitch = lift angle, yaw = carriage yaw →
   sweep guidance can use joint angles, and leveling can use the known lift angle
   (encoders/IMU), not TF.
6. **Leveling sign:** `platform_level_joint = +θ_b` (axis `(0,+1,0)` is opposite
   the boom's `(0,-1,0)`).
7. **Docking reference:** trunk centreline at `H_dock = trunk_top − 2.0 m`;
   platform `c_channel_reference` (+X through the C-opening).
8. **Reachability:** if the distance exceeds the boom envelope, advise "move prime
   mover" (with the needed advance), never extend past limits.
9. **Two-phase safety:** raise/extend is open-loop; only the final approach is
   range-gated (`EMERGENCY_STOP` on near-contact); once docked only the centre
   sensor sees the trunk.
