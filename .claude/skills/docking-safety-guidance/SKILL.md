---
name: docking-safety-guidance
description: "Implement, port, or extend the operator-facing docking safety-guidance HUD (stopping-distance + TTC model, speed/gap states, config thresholds, Qt Quick HUD) for the oil-palm harvester, on the Orin hardware dashboard without the simulation harness."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Docking Safety-Guidance HUD (operator speed/gap envelope)

Implement the operator-facing **docking safety-guidance** system that tells the human operator,
during the c-channel platform's final approach to the trunk, whether the closing speed is safe and
at what speed to close.  This skill captures the complete design so an agent on the **Orin** (real
machine) can rebuild the HUD **without the simulation harness** — the model, the config contract,
the Qt Quick HUD, and the bridge wiring are all pure/dashboard-side and portable as-is.

This is the **advisory/operator-facing** layer, distinct from (and complementary to) the hard-contact
safety FSM in `boom-dock-ik`.

## When this applies

- Building the docking safety-guidance HUD on the Orin dashboard from scratch (no simulation).
- Editing `src/harvester_dashboard/harvester_dashboard/safety_guidance.py`,
  `config/safety_guidance.json`, `bridge.py` (the guidance section), or `qml/HudOverlay.qml`.
- Changing a threshold, the state logic, or the HUD layout.
- Diagnosing why the HUD shows the wrong state/colour, flickers, or shows a false green.

It does **not** apply to: the five-range centering/skew watchdog (`harvester_boom_plan/safety.py`),
the boom IK, tree-height estimation, or the telemetry wire format (see related skills).

## Core facts to internalize (load before editing)

<critical>
- **The guidance is about the c-channel platform docking onto the trunk, NOT the prime mover
  driving.** Inputs are the *forward gap* (center range) and the *platform closing speed*
  (`-d(gap)/dt`), produced by the boom joints (elevation + extension + leveling). Never use
  `/harvester/cmd_vel` or vehicle speed.
- **It is advisory/operator-facing only — the dashboard NEVER commands motion.** The real machine is
  human-operated (hydraulics via PLC/valve, no joint encoders, no joint cmd). The hard-contact
  authority is `harvester_boom_plan/safety.py` (`emergency_m=0.05`, `extend_stop_m=0.4`); this model
  only *warns* the operator to slow before those gates trip.
- **Four states, and `no_data` must never be a false green.** `safe` / `warn` / `danger` / `no_data`
  (grey). A missing or stale range reads `no_data`, never `safe`.
- **`danger` and `no_data` are adopted immediately (no debounce delay).** A collision warning or a
  loss of telemetry must never be held back by hysteresis. Only `safe`/`warn` transitions are
  debounced (`debounce_s`) to prevent flicker.
- **The config loader must sanitize out-of-range values.** `a_max_m_s2 <= 0` would divide-by-zero in
  the stopping-distance math; the loader clamps `a_max >= 1e-3`, `latency >= 0`, and `warn_margin` /
  `speed_ema_alpha` to (0,1]. A bad tuning file must never crash the HUD.
- **Units are fixed and must not drift:** speed in **cm/s**, distance in **m**, TTC in **s**.
  `_stop_distance_m` converts cm/s → m/s before the physics.
</critical>

## The model (single source of truth)

Stopping distance (the authoritative speed bound):

```
d_stop(v) = v·t_latency + v²/(2·a_max)          # v in m/s, a_max in m/s²
v_max(d)  = -a_max·t_latency + sqrt((a_max·t_latency)² + 2·a_max·d)   # → cm/s
```

`v_max(d)` is the distance-aware, deceleration-aware "slow to X cm/s" figure (replaces the naive
`d / warn_ttc_s`).  TTC = `d / v` stays as the glanceable advisory.  State assignment:

| State | Trigger |
|---|---|
| `no_data` | gap or speed missing, or range stale (`age > stale_s`) |
| `danger` | `gap ≤ danger_distance` (absolute floor) **or** `gap ≤ d_stop(v)` **or** `TTC ≤ danger_ttc` |
| `warn`   | `gap ≤ warn_distance` **or** `TTC ≤ warn_ttc` **or** `v > warn_margin·v_max(gap)` |
| `safe`   | moving away, or stationary outside danger distance, or all warn/danger conditions clear |

`warn_margin` (0.7) makes WARN fire at 70% of the physical stop speed, leaving operator headroom.
A stationary platform (`|v| ≤ stationary_epsilon_cm_s`) is only `danger` via the absolute distance
floor (too close to linger), never by stopping distance (no motion).

## Reference map

- **Model + config contract (thresholds, units, sanitization):** [references/model-and-config.md](references/model-and-config.md)
- **Bridge wiring (speed derivation, staleness, hysteresis debounce, QML properties):** [references/bridge-wiring.md](references/bridge-wiring.md)
- **Qt Quick HUD (state banner, stop-bar, metrics, no-data grey):** [references/hud-qml.md](references/hud-qml.md)

## Quick workflow (Orin, no simulation)

1. **Port the model verbatim.** `safety_guidance.py` is pure Python (no Qt, no ROS) — copy it as-is,
   including `SafetyConfig.load` (with sanitization) and `state_message`.  Its tests
   (`test_safety_guidance.py`) run with plain pytest and no graph.
2. **Wire the bridge.** In `bridge.py`, derive closing speed as the least-squares slope of the gap
   over ~1.5 s, EMA-smooth it (`speed_ema_alpha`), track the *local receipt monotonic time* of the
   latest gap to compute staleness, and apply the hysteresis debounce (adopt `danger`/`no_data`
   immediately).  Expose the QML properties listed in the bridge reference.
3. **Render the HUD.** Recreate the `HudOverlay.qml` docking panel (state banner, stop-bar, metrics
   row, grey `no_data`) using **QtQuick 2 primitives only** (no Controls2 on PySide2 5.14).
4. **Validate against the model tests, not the simulator.** Port `test_safety_guidance.py` and run:
   ```bash
   PYTHONPATH=src/harvester_dashboard /usr/bin/python3 -m pytest src/harvester_dashboard/test/test_safety_guidance.py -v
   ```
5. **On the Orin, the data source differs but the contract is identical.** The dashboard consumes the
   canonical `v1/range/docking` channel (the `center_line` record = forward gap); the speed is still
   derived dashboard-side from `-d(gap)/dt`.  No ROS import is added.

## Key invariants (don't let these drift)

- **`no_data` is a real state, not a fallthrough.** A gap of `None`, a speed of `None`, or `stale`
  must return `no_data` with a grey banner — never `safe`.
- **Hysteresis only gates `safe`/`warn`.** `danger` and `no_data` bypass debounce.  When a state is
  held during debounce, the displayed *message* must match the *held* state colour
  (`state_message(state, guidance)`), never the candidate's — so the HUD text and colour never
  disagree (e.g. no "STOP NOW" text on an orange banner).
- **The stop-bar compares `gap` (fill) against `d_stop(v)` (white marker);** when the marker meets
  or passes the fill the bar turns red.  `stopbar_range_m` (1.5 m) is the full-scale of the bar's
  near field.
- **Config is the single source of truth** (`config/safety_guidance.json`): `a_max_m_s2`,
  `latency_s`, `warn_margin`, `warn_ttc_s`, `danger_ttc_s`, `warn_distance_m`, `danger_distance_m`,
  `stationary_epsilon_cm_s`, `stale_s`, `debounce_s`, `speed_ema_alpha`.  Missing keys keep defaults;
  out-of-range values are sanitized, not rejected.
- **Confirmed constants:** `a_max_m_s2 = 0.10` (gentle), `latency_s = 0.30`.  At these, `v_max(1.0 m)`
  ≈ 42 cm/s and WARN fires ≈ 29 cm/s.  Do not silently change these without the operator's sign-off.

## Validation

- `pytest test_safety_guidance.py` green (pure logic: stopping distance, hysteresis consistency,
  no-data, moving-away, monotonic `v_max`).
- On hardware, drive a real slow approach and confirm green→orange→red transitions and that the
  recommendation `v_max(d)` is physically reachable (no "slow to an impossible speed" advisory).

## Related skills

- `cutter-safety-guidance` — the **sibling** operator safety HUD for the cutting arm (cutter tip
  clearance + cut sequence), which reuses this skill's stopping-distance + TTC scheme.
- `boom-dock-ik` — the five-range centering/skew watchdog and hard-contact authority
  (`emergency_m` / `extend_stop_m`) this guidance *advises around*, and the boom geometry.
- `harvester-dashboard` — the Qt Quick/QML dashboard, interpreter split, and view-only boundary
  this HUD lives inside.
- `telemetry-gateway` — produces `v1/range/docking` (the `center_line` gap) this HUD reads.
- `harvester-simulation` — the Gazebo harness; **not needed on Orin**, but the source of the
  `approach_driver.py` validation script referenced in docs.
