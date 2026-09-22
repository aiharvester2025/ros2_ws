---
name: cutter-safety-guidance
description: "Implement, port, or extend the operator-facing cutter safety-guide HUD (cutting-arm tip-clearance model with sensor-to-tip offset, stopping-distance + TTC states, cut-sequence phases, config thresholds, Qt Quick HUD) for the oil-palm harvester, on the Orin hardware dashboard without the simulation harness."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Cutter Safety-Guide HUD (cutting-arm tip clearance + cut sequence)

Implement the operator-facing **cutter safety-guide** system that (1) warns the operator before the
cutting tool tip crashes into the trunk / FFB / frond, and (2) prompts the cut sequence
(approach → stop/ready → open scissors → advance → cut).  This skill captures the complete design so
an agent on the **Orin** (real machine) can rebuild the HUD **without the simulation harness** — the
model, config, HUD, and bridge wiring are pure/dashboard-side and portable as-is.

It is the sibling of `docking-safety-guidance` and shares the same scheme; this skill notes only what
is **different** for the cutter.

## When this applies

- Building the cutter safety-guide HUD on the Orin dashboard from scratch (no simulation).
- Editing `src/harvester_dashboard/harvester_dashboard/cutter_safety_guidance.py`,
  `config/cutter_safety_guidance.json`, `bridge.py` (the cutter section), or `qml/HudOverlay.qml`.
- Diagnosing the cutter clearance/phase HUD (wrong state, offset error, phase not advancing).

## Core facts to internalize (load before editing)

<critical>
- **The only clearance measurement is the single range sensor**
  (`/harvester/cutting_tool_left_range`, telemetry key `cutter_forward`).  The depth camera and
  LiDAR are mounted on **`cutting_arm_base_link`** and follow the arm's lift (pitch) and rail (yaw)
  but **NOT** the `cutting_arm_extension_joint` stroke — so they do **not** move with the tip and
  **cannot** measure the tip clearance.  **There is no fusion input.**  (Confirmed by the operator
  2026-09-22.)
- **The sensor sits BEHIND the cutter tip**, so the raw range overstates the true clearance.  The
  model MUST apply `tip_clearance = raw_range − sensor_to_tip_offset_m` (URDF-derived ~0.19 m).
  Getting this sign/offset wrong invalidates every state.
- **The cutter advance is `cutting_arm_extension_joint`** (prismatic +X, **0..0.375 m** only).  The
  range tracks the extension ~1:1 (validated in Gazebo).  Because the stroke is short, the operator
  must first bring the arm *close* (boom/rail/lift) before the extension does the final approach.
- **Advisory/operator-facing only — the dashboard never commands motion.**  The scissors and gripper
  are hydraulic operator actions with **no simulated joints**; the HUD only *prompts* them.
- **`no_data` must never be a false green**; `danger`/`no_data` bypass debounce (adopted immediately).
- **Units:** clearance in **m**, speed in **cm/s**, TTC in **s**.
</critical>

## The model (same scheme as docking)

```
d_stop(v) = v·latency + v²/(2·a_max)
v_max(d)  = -a_max·latency + sqrt((a_max·latency)² + 2·a_max·d)   # → cm/s
tip_clearance = raw_range − sensor_to_tip_offset_m
```

States: `no_data` / `danger` (`clearance ≤ danger_clearance` or `clearance ≤ d_stop(v)`) /
`warn` (`clearance ≤ warn_clearance` or `v > warn_margin·v_max`) / `safe`.  Hysteresis debounce and
config sanitization are identical to docking.

### Cut-sequence phase machine

```
approach → (clearance ≤ ready_standoff + align_tolerance AND settled) → align
align → open → advance → cut      (operator-confirmed via advance_phase / CONFIRM STEP)
```

- `approach → align` is **automatic**, but requires BOTH clearance in the ready band **and** that the
  tip is settled and not being warned to slow (`speed ≤ warn_margin·v_max(clearance)`).  The
  "ready to cut" prompt must never appear while the operator is still warned to slow/stop.
- `open`, `advance`, `cut` have **no sensors** — the operator confirms each step on the HUD
  (bridge slot `cutter_confirm_phase` → `advance_phase`), which is **blocked while DANGER/NO_DATA**.
- A brief range dropout **holds** ALIGN/OPEN/ADVANCE/CUT (it must not wipe operator progress).
- `advance` prompts moving the cutter forward `advance_distance_m` before cutting.

## Reference map

- **Model + config (offset, states, phases, sanitization):** [references/model-and-config.md](references/model-and-config.md)
- **Bridge wiring (speed, staleness, phase machine, QML properties):** [references/bridge-wiring.md](references/bridge-wiring.md)
- **Qt Quick HUD (phase banner, clearance bar, metrics, CONFIRM STEP):** [references/hud-qml.md](references/hud-qml.md)

## Quick workflow (Orin, no simulation)

1. **Port the model verbatim** — `cutter_safety_guidance.py` is pure Python (no Qt/ROS).  Its tests
   run with plain pytest.
2. **Wire the bridge** — read the `cutter_forward` record from `v1/range/cutter`, apply the offset,
   derive/EMA the closing speed, track local receipt time for staleness, advance the phase, apply
   debounce, and expose the properties listed in the bridge reference.
3. **Render the HUD** on the **cutter view** (`bridge.view === "cutter"`), QtQuick 2 primitives only.
4. **Validate against the model tests**, not the simulator:
   ```bash
   PYTHONPATH=src/harvester_dashboard /usr/bin/python3 -m pytest src/harvester_dashboard/test/test_cutter_safety_guidance.py -v
   ```
5. On the Orin the data source is the same canonical `v1/range/cutter` channel; no ROS import added.

## Key invariants (don't let these drift)

- **Offset sign:** `clearance = range − offset` (sensor behind tip).  URDF-derived ~0.19 m for the
  reference tool; make it a **tunable, calibratable** config value and re-derive from the URDF.
- **`ready_standoff_m` should sit clear of the danger band** (validated: 0.20 m ready vs 0.10 m
  danger, tolerance 0.05 m) so the tip settles in a stable WARN "ready-to-cut" state rather than
  oscillating warn/danger at the boundary.
- **`approach → align` is automatic but gated on both clearance and speed** (settled, not
  warn-speed); later phases are operator-confirmed (no sensors) and blocked while DANGER/NO_DATA.
  Never let the "ready to cut" prompt appear while the operator is being warned to slow/stop.
- **The range is the sole measurement** — never imply the depth/LiDAR feed the clearance.

## Validation

- `pytest test_cutter_safety_guidance.py` green (offset math, states, phase machine, hysteresis,
  no-data, config sanitization).
- In simulation (harness `harvester_dock/cutter_approach_driver.py`), spawn a close target, ramp
  `cutting_arm_extension_joint`, and confirm: raw→clearance offset applied; safe→warn→danger as the
  tip closes; phase approach→align at the ready standoff; danger trips **before contact**.

## Related skills

- `docking-safety-guidance` — the parent scheme (stopping-distance + TTC, config, HUD, bridge) this
  skill mirrors.
- `harvester-dashboard` — the Qt Quick/QML dashboard, interpreter split, view-only boundary.
- `telemetry-gateway` — produces `v1/range/cutter` (the `cutter_forward` range) this HUD reads.
- `boom-dock-ik` — the arm/boom geometry and the hard safety FSM.
