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

Docking point (**2.0 m below the crown base**, NOT below the trunk top):

```
H_dock = crown_base - 2.0 m  = 9.2 - 2.0 = 7.2 m   (reference tree)
```

> **Why crown base, not trunk top.** The trunk top (12.0 m) sits *inside* the
> canopy.  `trunk_top - 2.0 = 10.0 m` lands in the frond/FFB zone (fronds at
> 9.45 m, FFBs at 9.55 m) and caused the platform to crash into them.  The
> crown base (trunk-end, 9.2 m) is where the fronds first appear; docking
> `crown_base - 2.0` grips clean trunk well below the lowest fronds.  The
> crown base is measured live from the LiDAR sweep (see
> `harvester_dock/live_height_estimator.py` `crown_base_from_density`).

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

4. **The LiDAR is on the moving arm — level per scan, not once per sweep.**
   `vehicle_lidar_link` is a fixed child of `cutting_arm_base_link`, which the
   `cutting_arm_lift_joint` pitches through the sweep.  Leveling the whole
   accumulated cloud with a *single* end-of-sweep transform smears the tree
   across every arm pose and corrupts the crown-base density profile (the
   canopy annulus becomes dense at all heights).  `dock_orchestrator` must
   accumulate only during the stationary `dwell` phase and level each scan at
   receipt via the zero-stamped `/harvester/lidar/points` topic (latest TF =
   the current dwell pose).  This is a simulation AND hardware concern: the
   sensor genuinely moves with the arm on the real machine.

5. **Dock from the crown base, not the trunk top.**  `H_dock = crown_base - 2.0 m`.
   The crown base (trunk-end, ~9.2 m) is measured live from the canopy-annulus
   density transition, not from `trunk_top - 2.0` (which is inside the frond/FFB
   zone).  See `crown_base_from_density` in `live_height_estimator.py`.

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

## Operator safety-guidance model (speed + gap HUD)

Complementary to the five-range centering/skew watchdog above is a **continuous
approach envelope** shown to the operator on the dashboard HUD
(`harvester_dashboard/harvester_dashboard/safety_guidance.py` + `qml/HudOverlay.qml`).
It is **advisory/operator-facing only** (the dashboard never commands joints);
`emergency_m`/`extend_stop_m` here remain the hard-contact authority.

The model fuses the **forward gap** (`center_range`) with the **platform closing
speed** (`-d(center_range)/dt`, EMA-smoothed in `bridge.py`) into three states:

| State | Trigger |
|---|---|
| 🟢 SAFE | moving away, or gap + TTC + speed all clear |
| 🟠 WARN | `gap ≤ warn_distance` **or** `TTC ≤ warn_ttc` **or** `v > warn_margin·v_max(gap)` |
| 🔴 DANGER | `gap ≤ d_stop(v)` **or** `gap ≤ danger_distance` |
| ⬜ NO DATA | range absent or stale (never a false green) |

The authoritative speed bound is the **stopping distance**:

```
d_stop(v) = v·t_latency + v²/(2·a_max)
v_max(d)  = -a_max·t_latency + sqrt((a_max·t_latency)² + 2·a_max·d)
```

`v_max(d)` is the distance-aware "slow to X cm/s" figure — it honours the
platform's actual deceleration capability `a_max` and reaction/actuation latency
`t_latency`, unlike a `d/warn_ttc_s` heuristic.  Thresholds live in
`config/safety_guidance.json` (`a_max_m_s2`, `latency_s`, `warn_margin`,
`warn_ttc_s`, `danger_ttc_s`, `warn_distance_m`, `danger_distance_m`,
`stale_s`, `debounce_s`) and are **validated in simulation** by
`harvester_dock/approach_driver.py` (a simulation-only harness that drives the
platform toward the trunk at a controllable closing speed).  Hysteresis
(`debounce_s`) suppresses flicker at state boundaries; `NO_DATA`/`DANGER` are
adopted immediately (a loss of telemetry or a collision warning is never delayed).

**Research finding (2026-09-21):** the raw per-sample closing-speed derivative is
noisy (±5 cm/s) under the 20 Hz / σ=3 mm `center_range` sensor, so the EMA
smoothing in the bridge is essential, not cosmetic.

**Tuned constants (2026-09-21):** `a_max_m_s2 = 0.10` (very gentle hydraulic
deceleration) and `latency_s = 0.30` (operator reaction + actuation), confirmed
with the operator.  With these, the physical speed limit at a 1.0 m gap is
~42 cm/s and WARN fires at `warn_margin = 0.7` of that (~29 cm/s).  The config
loader sanitizes out-of-range values (e.g. `a_max ≤ 0` clamps to `1e-3`) so a bad
tuning file cannot crash the HUD.

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

There are **two docking modes** — pick one (they are mutually exclusive; running
both makes them fight over `/harvester/joint_commands`):

1. **Autonomous dock** — the `harvester_dock` orchestrator (sweep → estimate →
   dock → undock).  Requires `joint_gui:=false` and `boom_plan:=false` so nothing
   overrides its joint commands.  See `src/harvester_dock/README.md`.
2. **Read-only advisor** — this `boom_plan` stack, which only *displays* θ_b /
   extension / distance (no auto-actuation).  Keep `boom_plan:=true` and use
   `boom_docking_demo --execute` to command joints manually.

### Read-only advisor (this package)

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select harvester_boom_plan --merge-install --symlink-install
source install/setup.bash

# Full simulation (headless): harvester + tree + boom-plan stack
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  gui:=false rviz:=false harvester_collision_mode:=off \
  articulation_control_mode:=kinematic docking_camera:=false

# IMPORTANT: stop the slider GUI, it overrides scripted joint commands (or
# launch with joint_gui:=false).
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
- `src/harvester_dock/harvester_dock/dock_orchestrator.py` — autonomous FSM (simulation harness).
- `src/harvester_dock/harvester_dock/live_height_estimator.py` — live height estimation (half-cylinder-corrected).
- `src/harvester_dock/harvester_dock/distance_estimator.py` — distance + reachability.
- `src/harvester_dock/config/dock.yaml` — sweep band, docking offset, home pose.

## Remaining next steps

1. Feed a real LiDAR-derived trunk-top estimate into `/harvester/tree/docking_estimate`
   (currently a static 12.0 m ground truth in `tree_docking_estimate_node.py`).
2. Add a dashboard read-out / canonical ZMQ channel for `/harvester/boom/plan`.
3. Confirm the real-machine leveling mechanism (active vs. passive stabilizer);
   the `leveling_mode` config flag defaults to `active` for this simulation.

---

## Real-machine deployment: human-in-the-loop operating model (2026-09-03)

> **Architectural distinction — two roles, both retained.**  The `harvester_dock`
> orchestrator (`dock_orchestrator.py`) **auto-actuates** (sweeps the LiDAR, runs
> the estimator, commands the joints) and is **retained deliberately** as the
> *simulation validation / idea-testing / experimentation harness*.  On the
> physical machine, a **human operator** performs every physical action; the
> software only **guides** (sweep procedure), **advises** (target boom angle /
> distance / extension), and **monitors** (live status feedback).  A separate
> **human-in-the-loop** system that *mimics* this flow inside the simulation will
> be built later — it will reuse the orchestrator's guidance/plan/monitoring
> logic while replacing auto-actuation with operator action + dashboard feedback.

The real workflow is three human-performed phases, each needing dashboard
guidance + monitoring (do **not** auto-actuate in deployment):

### Phase 1 — LiDAR sweep (human performs, dashboard guides)

- The dashboard guides the operator through the sweep **step by step, start to
  end** (e.g. "raise arm to X° → hold → sweep to Y° → hold"), reading back and
  confirming each step before the next.
- The operator physically moves the cutting arm; the system accumulates the
  live LiDAR cloud and reports progress (points captured, coverage %).
- **Research note:** a single, correctly-aimed nod through the trunk's ~62°
  elevation span (a 3–5 step short sweep) is sufficient; the earlier 40-step
  full-range sweep overshoots above/below the trunk and adds no density.  The
  real Mid-360 (200,000 pts/s non-repetitive) densifies on dwell, so far fewer
  passes are needed than in the sparse Gazebo grid.

### Phase 2 — Boom to target (human moves, dashboard advises + monitors)

- Once the sweep completes, the dashboard displays the **optimum target values**:
  boom distance to trunk, boom angle (`θ_b`), and boom extension (`l_b + l_p`).
- The human operator raises/extends the boom toward those targets.
- The system **monitors** the live joint states and displays **current vs.
  target** on the dashboard (e.g. "angle 42°/45°, extension 8.2/9.0 m").

### Phase 3 — Level + dock (human levels and lowers, dashboard monitors)

- Once the boom reaches the target extension, the human operator **levels the
  platform** and **lowers the boom angle** to dock the c-channel onto the trunk.
- The system monitors this final approach via the five docking range sensors and
  the cutter range, displaying the safety state machine (see "Safety state
  machine") on the dashboard, including `EMERGENCY_STOP` on near-contact.

### Implications for the codebase

- The read-only `boom_plan_node` (guidance/advice) is directly reusable.
- The `harvester_dock` orchestrator is **retained as-is** for simulation/testing;
  its *sweep guidance*, *plan display*, and *monitoring* states will be reused by
  the future human-in-the-loop mimic, while its *auto-actuation* (publishing
  `/harvester/joint_commands`) will be swapped for operator action + feedback.
- The dashboard needs: (a) a step-by-step sweep guide panel, (b) a
  current-vs-target boom readout, and (c) the docking safety-state display.

### Simulation-only artifacts (do NOT treat as deployment blockers)

Two remaining accuracy items are **pure simulation artifacts** and were
deliberately not pursued:

1. **`world`-frame TF startup race** — Gazebo/TF2 plumbing; real hardware has a
   rigidly-calibrated, instrumented arm (encoders/IMU), no such race.
2. **Gazebo LiDAR density (107×64 grid)** — the real Mid-360 is *already denser*
   (200,000 pts/s non-repetitive) and captures the trunk top/full cylinder better
   than the grid; no point tuning the sim to match a sensor we already have.

---

## Research findings for physical-hardware implementation (2026-09-03)

The following findings were established during simulation experimentation and
are **directly relevant to the real machine** (they are about geometry, sensor
physics, and estimation, not Gazebo plumbing).

### 1. LiDAR coverage — how many sweep passes are needed

- The cutting-arm LiDAR vertical FOV is **−7°…+52° (59°)**, horizontal **±60° (120°)**.
- At the reference 6.6 m standoff, the trunk (0.5–0.7 m diameter, 12 m tall)
  subtends **~4° horizontally × ~62° vertically** from the LiDAR.
- **One correctly-aimed nod (3–5 steps) spans the whole trunk** — the 59° vertical
  FOV already covers the 62° trunk height at this range.  A full −20°…+60° sweep
  overshoots far above/below the trunk and contributes almost no trunk points.
- The real **Livox Mid-360** emits **200,000 points/s non-repetitive** vs the
  Gazebo grid's **68,480 rays/s (~2.9×)**; more importantly its non-repetitive
  pattern **densifies with dwell**, so a stationary hold keeps adding trunk
  points — even fewer passes are needed than in the sparse grid.

### 2. Trunk-axis "half-cylinder" bias (the dominant estimation error)

- A single-sided LiDAR sees only the **near face** of the trunk, so the naive
  median X/Y of trunk points lands at the *surface*, ~`radius` short of the true
  centreline (measured −0.22 m at 0.3 m radius).
- **Correction:** `centre_x = median_x + (2/π)·radius` (the mean of `cos θ` over
  the visible half).  This recovered the axis error from −0.22 m to ~−0.06 m.
- This bias is **physical and unavoidable** for a single stationary LiDAR; it
  exists identically on real hardware.  It also matters for **horizontal docking
  distance** (a 0.2 m axis error becomes a 0.2 m docking-X error).

### 3. Trunk-top estimation — `max` is the correct upper envelope

- Within a tight cylinder about the axis, fronds/occlusion can only **hide** the
  trunk top, never exceed it, so the **maximum** z is the right estimate.
- Percentile estimators (`p99`/`p99.5`) are **biased low** because they are
  dragged down by the bulk of lower trunk points.
- The trunk top may be **sensor-limited**: if the upper FOV edge does not clear
  the trunk top, the highest *visible* point is a few cm below the true top.
  The real Mid-360's wider/denser vertical coverage mitigates this.

### 4. The LiDAR is rigidly mounted on the cutting arm

- `vehicle_lidar_link` is a fixed child of `cutting_arm_base_link` (offset
  +0.3 m Z), which is pitched by `cutting_arm_lift_joint` (axis `0 -1 0`,
  range −0.35…+1.05 rad) and yawed by `rail_carriage_joint` (±2.55 rad).
- So the LiDAR's **pitch = lift angle** and **yaw = carriage yaw**.  This means
  sweep guidance can be expressed in *joint angles* the operator controls
  directly, and leveling can be done from the **known lift angle** (encoders/IMU)
  rather than a TF lookup — robust on hardware.

### 5. Leveling sign convention

- `boom_elevation_joint` axis is `(0,-1,0)`; `platform_level_joint` axis is
  `(0,+1,0)` — opposite.  To level the platform after raising the boom, command
  `platform_level_joint = +θ_b` (NOT `-θ_b`).  The same sign-convention care
  applies to the real hydraulic valves.

### 6. Docking reference point

- **Tree side:** trunk **centreline** at `H_dock = crown_base − 2.0 m` (NOT
  `trunk_top − 2.0 m`, which lands in the canopy).  The crown base is the
  trunk-end where fronds first appear (~9.2 m for the reference tree), measured
  live from the LiDAR sweep.
- **Platform side:** `c_channel_reference` (+X through the C-opening toward the
  tree).  The IK places `c_channel_reference` exactly at the trunk-centre dock
  point.  The 2.0 m offset below the crown base keeps the grip on clean trunk
  below the crown.

### 7. Reachability / "move prime mover" advisory

- If the estimated platform→trunk distance exceeds the boom's reachable
  envelope, the system must **advise the operator to reposition the vehicle**
  (with the needed +X advance), not attempt to extend past limits.

### 8. Safety — two-phase gating

- The **raise/extend** phase is open-loop (range sensors legitimately out of
  range; `WAITING_FOR_TREE` must not block).  The **final approach** is
  range-gated; only `EMERGENCY_STOP` (near-contact ≤0.05 m) and
  `INFEASIBLE_DOCK_HEIGHT` are hard blocks.  Once docked, only the forward
  centre sensor still sees the trunk (side beams pass it) — the side pair is
  meaningful during *approach*, the centre sensor confirms *final contact*.
