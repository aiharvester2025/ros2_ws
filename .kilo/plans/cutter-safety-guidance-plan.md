# Cutter Safety-Guide HUD — Cutting-Arm Approach & Cut-Sequence Guidance

**Date:** 2026-09-22
**Workspace:** `/home/ubuntu/ros2_ws`
**Status:** Plan (research complete, not yet implemented)

---

## 1. Goal

Add an operator-facing **Cutter Safety-Guide HUD** to the dashboard, shown **only on the cutter
camera view**.  It mirrors the docking safety-guidance design (states, config, hysteresis, HUD
panel) but for the **cutting arm**:

1. **Monitor the cutter-tip clearance** to the trunk / FFB / frond and warn the operator to avoid
   crashing the tool into them (green / orange / red / grey, same scheme as docking).
2. **Guide the cut sequence** step by step once the tip is at a good standoff:
   stop → open the cutting scissors **wide** → advance the cutter forward a tuned distance →
   perform the cut (gripper holds the FFB).

It is **advisory/operator-facing only** — like the rest of the dashboard it never commands motion.

---

## 2. Research findings (verified in this workspace)

### 2.1 The cutting arm chain and geometry

Chain (all URDF-derived, `oil_palm_harvester_kinematic.urdf`):

```
rail_carriage_joint (yaw ±2.55) -> cutting_arm_base_link
  -> cutting_arm_lift_joint (rev, -0.35..+1.05) -> still base
  -> cutting_arm_extension_joint (prismatic +X, 0..0.375 m) -> cutting_arm_extension_link
  -> cutting_tool_fixed_joint (fixed +X 0.46) -> cutting_tool_link
```

- **Advance joint** = `cutting_arm_extension_joint`, a **prismatic +X stroke of only 0.375 m** —
  this is the "move the cutter forward for a certain specific distance" action.
- **Cutter working point / tip:** `cutting_tool.stl` spans local X `-0.03..0.58` with visual scale
  `0.5 1 1` → the forward working geometry is at tool-local X ≈ **0.29 m**.  The scissor **jaws**
  occupy tool-local X ≈ 0.21..0.29 m (dense mesh region).
- **Chain to the tip** (extension = 0): `0.31 + 0.46 + 0.29 = 1.06 m` from `cutting_arm_base_link`;
  at full extension `1.06 + 0.375 = 1.435 m`.

### 2.2 The one distance sensor and the **critical offset**

- **Sensor:** `/harvester/cutting_tool_left_range` (`telemetry_key: cutter_forward`), a single ray on
  `cutting_tool_left_range_sensor_link`, a child of `cutting_tool_link` at tool-local
  `(0.10, 0.12, 0.00)`, facing **+X**; 20 Hz, σ = 3 mm, range 0.05–10 m.
- **Offset (the brief's key point):** the sensor sits at tool-local X = **0.10 m**, but the tip is at
  X ≈ **0.29 m** → the sensor is **~0.19 m behind the tip**.  So the raw reading is the distance from
  the *sensor face* to the object; the **true tip-to-object clearance is
  `tip_clearance = range − (tip_x_local − sensor_x_local) = range − ~0.19 m`** (when the object is
  forward of the tip).  This offset must be an explicit, tunable, calibratable constant — not
  hard-coded magic — and it must be re-derivable from the URDF.

### 2.3 Fusion is NOT available for the tip-clearance input (corrected 2026-09-22)

The depth camera (`platform_depth_camera`) and LiDAR (`vehicle_lidar_link`) are both mounted on
**`cutting_arm_base_link`**, so they follow the arm's **lift (pitch)** and **rail-carriage (yaw)**
but **NOT** the `cutting_arm_extension_joint` stroke.  They therefore **do not move with the cutter
tip** during the forward cut motion and **cannot measure the tip-to-object clearance**.

> **Consequence:** the **single range sensor** (`cutting_tool_left_range`, a child of
> `cutting_tool_link`, so it *does* follow the extension) is the **only** clearance measurement.
> Depth/LiDAR fusion is **out of scope** for this HUD.  (A future option would be to mount a sensor
> on `cutting_tool_link`, or to derive the tip pose from forward kinematics + the arm joints — but
> the brief confirms the distance is measured by the distance sensor only.)

### 2.4 No scissor/gripper instrumentation

There are **no scissor or gripper joints** in the URDF — the cutting tool is one fixed link.  The
"open scissors wide" and "gripper holds the FFB" actions are **hydraulic end-effector actions the
operator performs**; the HUD can only *prompt* them (and later, optionally, accept a manual
"done" confirmation).  This is a hard constraint on the guidance design (see §3.3).

### 2.5 Targets to protect

Tree collision objects: `trunk` (vertical, z 0–12), **fronds** (`branch_*`, z 9.45/10.55) and
**FFBs** (`ffb_01..07`, z 9.55–10.88).  The cutter works in the crown/base region, so the closest
protected object may be a frond/FFB rather than the trunk — the guidance must reason about
**nearest protected surface**, not only the trunk.

---

## 3. Design

### 3.1 Reuse the docking safety-guidance scheme

Mirror `harvester_dashboard/harvester_dashboard/safety_guidance.py` exactly in structure:

- A pure-Python model module (no Qt/ROS), unit-tested, with a `SafetyConfig`-style frozen dataclass,
  a `Guidance`-style result, `evaluate(...)`, and a `state_message(...)` helper for hysteresis
  consistency.
- Four states: `safe` / `warn` / `danger` / `no_data` (grey; never a false green).
- Stopping-distance bound `d_stop(v) = v·t_latency + v²/(2·a_max)` and `v_max(d)`, so the "slow to
  X cm/s" advisory is deceleration-aware.
- Hysteresis debounce; `danger`/`no_data` adopted immediately.
- Config sanitized on load (no divide-by-zero on a bad tuning file).

**Difference from docking:** the clearance input is the **corrected tip-to-object distance**
(`range − sensor_to_tip_offset`), and the protected object is the **nearest of trunk/FFB/frond**.

### 3.2 Cutter clearance model (new module)

`cutter_safety_guidance.py` (pure Python), inputs:

- `tip_clearance_m` — the corrected tip-to-object distance (from the single range sensor by default,
  with the offset applied; optionally from the fusion provider).
- `closing_cm_s` — the tip's closing speed toward the object, derived like the docking speed
  (`-d(clearance)/dt`, least-squares slope + EMA).

States (same semantics as docking, with cutter-specific defaults to be tuned):

| State | Trigger |
|---|---|
| `no_data` | clearance missing or stale (`age > stale_s`) |
| `danger` | `clearance ≤ danger_clearance` **or** `clearance ≤ d_stop(v)` |
| `warn` | `clearance ≤ warn_clearance` **or** `v > warn_margin·v_max(clearance)` |
| `safe` | moving away, or stationary clear, or all warn/danger clear |

### 3.3 Cut-sequence guidance (a small, explicit FSM)

The brief's sequence, encoded as guidance **phases** shown on the HUD (guidance-only; the operator
performs each action):

```
APPROACH   tip closing on the object            -> colour-coded safe/warn/danger as above
ALIGN      tip at the target standoff (a good value, tunable) -> "STOP — ready to cut"
OPEN       prompt: open the cutting scissors WIDE (operator action; no sensor)
ADVANCE    prompt: move the cutter forward the tuned advance distance (cutting_arm_extension_joint)
CUT        prompt: perform the cut (gripper holds the FFB; operator action)
```

Phase transitions are driven by measured quantities where available and by operator confirmation
where not instrumented (`OPEN`, `ADVANCE`, `CUT`).  The FSM is a **guide**, never an actuator.

`APPROACH → ALIGN` requires **both** conditions (final, after review):

1. the tip is inside the ready band (`clearance ≤ ready_standoff + align_tolerance`), **and**
2. the tip is **settled and not warn-speed** (`speed ≤ warn_margin · v_max(clearance)`).

Condition 2 ensures the "STOP — ready to cut" prompt can never appear while the state is simultaneously
telling the operator to slow down (WARN) or stop (DANGER).  Once in ALIGN/OPEN/ADVANCE/CUT, the phase
**holds** through a brief range dropout (an in-progress cut must not be reset) and the bridge **blocks
operator confirmation while the state is DANGER or NO_DATA**.

### 3.4 Configuration (single tunable file)

`config/cutter_safety_guidance.json` (mirrors `safety_guidance.json`):

```json
{
  "sensor_to_tip_offset_m": 0.19,
  "a_max_m_s2": 0.10, "latency_s": 0.30, "warn_margin": 0.7,
  "warn_clearance_m": 0.30, "danger_clearance_m": 0.10,
  "ready_standoff_m": 0.20, "advance_distance_m": 0.05,
  "stale_s": 2.0, "debounce_s": 0.4, "speed_ema_alpha": 0.35
}
```

- `sensor_to_tip_offset_m` — the §2.2 offset (re-derivable from URDF; the JSON is the tunable copy).
- `ready_standoff_m` — the "good value" where the HUD says STOP/ready-to-cut.
- `advance_distance_m` — the tuned forward move before cutting.

### 3.5 Clearance input (range-only — corrected)

- The clearance input is the **single range sensor** plus the tip/sensor offset (§2.2).  There is
  **no depth/LiDAR fusion** for the clearance: those sensors are mounted on `cutting_arm_base_link`
  and do not follow the extension, so they cannot see the tip.
- To improve *robustness* (not precision) with one sensor, the model uses: the tip/sensor offset,
  least-squares slope + EMA for the closing speed, staleness detection, and hysteresis.  Optionally,
  forward kinematics from the arm joints (`rail_carriage` yaw, `cutting_arm_lift`, `cutting_arm_extension`)
  can provide an independent predicted clearance for a **consistency check** (range vs FK sanity),
  but the range remains the primary measurement.

### 3.6 HUD (cutter view only)

`qml/HudOverlay.qml` (or a sibling `CutterGuidanceHud.qml`), visible only when
`bridge.view === "cutter"`:

1. **Phase banner** — the current cut-sequence phase + actionable line
   (e.g. "STOP — ready to cut", "OPEN SCISSORS WIDE", "ADVANCE 5 cm", "CUT").
2. **Clearance bar** — like the docking stop-bar: tip clearance vs required stopping distance.
3. **Metrics row** — tip clearance (corrected) · closing speed · safe/warn/danger state.
4. **No-data** grey state; **danger** pulses red.

Same colour palette and QtQuick-2-primitives-only constraint as the docking HUD.

### 3.7 Bridge wiring

Extend `bridge.py` (cutter section, parallel to the docking section):
- read the `cutter_forward` record from `v1/range/cutter`; apply `sensor_to_tip_offset_m`;
  track local receipt monotonic time for staleness.
- derive/EMA-smooth the closing speed; call `evaluate(...)`; apply debounce; expose properties
  (`cutterPhase`, `cutterClearanceM`, `cutterSafetyState`, `cutterGuidanceText`, `cutterStopDistanceM`,
  `cutterMaxSpeedCmS`, `cutterTtcS`, ...).

---

## 4. Validation / research plan

Mirror the docking `approach_driver.py` harness with a **cutter approach driver** that ramps
`cutting_arm_extension_joint` (and/or the lift) to produce a controlled closing speed of the tip
toward a chosen object, logging the corrected clearance and the guidance state.  Run profiles
(slow/moderate/fast) and verify:

1. `danger` trips before the tip would contact the object (advisory is more conservative than a
   contact), for every profile.
2. The corrected clearance matches the TF/FK ground truth (tip vs sensor offset verified).
3. No flicker at boundaries; `no_data` when the range drops/stales — never a false green.
4. The cut-sequence phases appear in order and at the expected clearance.

Run recipe: `./run_simulation.sh manual`, select the **cutter** view, then drive the arm via the
cutter approach driver (exclusive of the slider GUI).

---

## 5. Implementation steps

1. **Model** — new `cutter_safety_guidance.py` (mirror docking model; add the sensor→tip offset, the
   clearance-based states, and the cut-sequence phase FSM).
2. **Config** — new `config/cutter_safety_guidance.json` with the constants in §3.4.
3. **Bridge** — cutter section in `bridge.py` (clearance input, staleness, debounce, new properties).
4. **HUD** — cutter-view panel in `qml/HudOverlay.qml` (phase banner + clearance bar + metrics).
5. **Tests** — `test_cutter_safety_guidance.py` (offset math, states, sequence phases, hysteresis,
   no-data), mirroring the docking tests.
6. **Harness** — cutter approach driver under `harvester_dock` (simulation-only).
7. **Validate** — run the profiles in §4; tune `ready_standoff_m`, `advance_distance_m`, the
   clearance thresholds, and `sensor_to_tip_offset_m`.
8. **(Removed)** depth/LiDAR fusion is out of scope — the range sensor is the only clearance input.
9. **Docs + skill** — update `docs/`, `harvester_dashboard/README.md`, and add a
   `cutter-safety-guidance` agent skill (portable to Orin without the simulation), mirroring
   `docking-safety-guidance`.

---

## 6. Open decisions to confirm

1. **Protected-object scope:** should the clearance alarm treat the nearest of **trunk + FFB + frond**
   as one "object" (simplest, conservative), or distinguish the target class (cut FFB vs avoid
   frond)?  I recommend nearest-surface first, with class awareness as a later refinement.
2. **`sensor_to_tip_offset_m`:** I derived ~0.19 m from the URDF/mesh (`tip 0.29` − `sensor 0.10`).
   Confirm whether the real machine's cutter tip is at the same place, or provide the surveyed value.
3. **Scissor/gripper instrumentation:** confirm these stay **operator-actions with HUD prompts**
   (guidance-only) — there are no joints to sense, so no automatic state detection.
4. **Fusion scope:** **Resolved (2026-09-22):** out of scope — the depth camera and LiDAR are on
   `cutting_arm_base_link` and do not follow the extension, so only the range sensor measures the
   tip clearance.
5. **Advance distance & ready standoff:** seeded `advance_distance_m = 0.05`,
   `ready_standoff_m = 0.20` (raised from 0.15 after simulation tuning — see §7), to be confirmed.

---

## 7. Implementation & simulation-validation results (2026-09-22)

Implemented: `cutter_safety_guidance.py`, `config/cutter_safety_guidance.json`,
`test_cutter_safety_guidance.py` (22 tests), bridge cutter section, cutter HUD panel, and the
`cutter_approach_driver.py` harness.  All 114 dashboard tests pass.

Validated live in ROS2 + Gazebo (headless) by spawning a test target box ~0.5 m ahead of the cutter
tip and ramping `cutting_arm_extension_joint`:

- **Offset applied correctly**: raw 0.29 m → tip clearance 0.10 m (0.19 m offset).  Range tracks the
  extension ~1:1 (0.375 m extension → 0.375 m range change), confirming the tip model.
- **States**: observed `no_data → safe → warn → danger` (and back) driven by real motion; `danger`
  tripped at clearance ≤ 0.10 m, **well before** tip contact.
- **Cut sequence**: phase advanced `approach → align` when clearance ≤ 0.25 m
  (`ready_standoff 0.20 + tolerance 0.05`) **and** the tip is settled (≤ `warn_margin·v_max`);
  operator-confirmed `align → open → advance → cut` verified.
- **Deceleration-aware advisory**: `v_max` scaled 24.7 → 17.9 cm/s as clearance shrank 0.38 → 0.21 m.
- **Research finding**: the raw per-sample closing speed is noisy (±5–16 cm/s) under the 20 Hz /
  3 mm sensor, so the EMA smoothing is essential (same as docking).

**Tuning change**: `ready_standoff_m` raised 0.15 → 0.20 so the ready-to-cut point sits clear of the
0.10 m danger band; the tip then settles in a **stable WARN "ready-to-cut" state** instead of
oscillating warn/danger at the boundary.

**Environment finding**: the Gazebo world has no object within the cutter's short 0.375 m extension
reach (the tree is ~5.4 m away), so exercising warn/danger required a spawned close target.  On the
real machine the arm is maneuvered close first; the extension performs only the final approach.

### Code-review fixes (2026-09-22)

Three safety issues were found and fixed in review before commit:

1. **Premature "ready to cut".** `approach → align` fired on clearance alone, so at 30 cm/s & 0.20 m
   the HUD showed DANGER ("STOP NOW") together with "STOP — ready to cut". Fixed by requiring the tip
   to be settled (`speed ≤ warn_margin · v_max(clearance)`), so the two signals never disagree.
2. **Range dropout wiped operator progress.** A stale range reset the phase to `approach`, losing an
   in-progress ALIGN/OPEN/ADVANCE/CUT. Fixed: operator-confirmed phases hold through a dropout.
3. **Confirm during DANGER.** The CONFIRM STEP button advanced the phase regardless of state. Fixed:
   `cutter_confirm_phase` is a no-op while DANGER/NO_DATA, and the button is disabled/relabelled
   ("WAIT — tip too close").

Component tests: `test_cutter_safety_guidance.py` (24 tests, incl. regressions for all three).
