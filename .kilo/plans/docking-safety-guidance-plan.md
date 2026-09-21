# Docking Safety-Guidance System — Physics-Based Model + Operator HUD

**Date:** 2026-09-19
**Workspace:** `/home/ubuntu/ros2_ws`
**Status:** Plan (not yet implemented)

---

## 0. Scope correction (important)

This guidance system is about the **c-channel platform docking onto the trunk**,
**not** the prime mover (vehicle) driving forward.  The "speed" and "distance to
trunk" inputs are the **platform's closing speed** and **front gap** during the
final docking approach, which is produced by the **boom joints** (elevation
`θ_b`, extension `e`, leveling) — not by `/harvester/cmd_vel`.

Reference (authoritative):
- `docs/BOOM_DOCK_PLAN.md` §"Real-machine deployment" Phase 3 "Level + dock":
  the operator levels the platform and lowers the boom to dock the c-channel onto
  the trunk; the system monitors this final approach via the **five docking range
  sensors** and shows the safety state.
- `harvester_boom_plan/safety.py` already implements the **centering/skew +
  clearance watchdog** over the five ranges.  This plan's model is the
  **orthogonal, missing piece**: a *continuous* speed+gap safety envelope and an
  operator HUD for the approach *motion* itself.

---

## 1. Problem

The previous session added a **pure TTC model** (`safety_guidance.py`) and a
single-colour HUD panel, but:

1. It was **only unit-tested** (`test_safety_guidance.py`); it was **never
   validated in simulation** against a real, moving docking approach.
2. The model is **TTC-only** with no physical stopping-distance bound — it cannot
   answer "can I still stop before the trunk?", only "how many seconds to impact
   at the current speed?".
3. The "recommended speed" uses `d / warn_ttc_s`, which is **independent of the
   platform's actual deceleration capability** and therefore not a true "slow to
   X cm/s" limit.
4. The thresholds (`warn_ttc_s=4`, `danger_ttc_s=1.5`, `warn_distance_m=1.0`,
   `danger_distance_m=0.3`) are **arbitrary guesses**, not derived from the
   platform geometry/physics or from sensor noise.
5. The HUD is a **single static panel**, not a HUD designed for a human operator
   (no glanceable stop-bar, no distance-to-stop visual, no stale-data handling,
   no hysteresis to avoid flicker at a state boundary).

Goal: a **validated** safety-guidance system — physics-based model + tunable
config + an effective operator HUD — demonstrated live in ROS2/Gazebo/RViz with a
scripted, repeatable slow approach so the transitions (green→orange→red) and the
"slow to X cm/s" advisory can actually be observed and tuned.

---

## 2. Current state (verified in this workspace)

### 2.1 What already exists

| Piece | File | Notes |
|---|---|---|
| Pure TTC model + JSON config | `harvester_dashboard/harvester_dashboard/safety_guidance.py`, `config/safety_guidance.json` | SAFE/WARN/DANGER on (TTC, distance) floors; recommended speed = `d/warn_ttc_s` |
| Dashboard bridge wiring | `harvester_dashboard/harvester_dashboard/bridge.py` | derives platform closing speed `-d(center_range)/dt` (least-squares slope over ~1.5 s), EMA-smooths it, calls `evaluate()`, exposes `dockSafetyState/dockGuidanceText/dockRecommendedSpeedCmS/dockTtcS/dockSpeedSmoothedCmS` |
| HUD panel | `harvester_dashboard/qml/HudOverlay.qml` | single colour-coded panel (green/orange/red), DANGER pulses |
| Tests | `harvester_dashboard/test/test_safety_guidance.py` | 11 unit tests, all pass |
| Five docking ranges + centering/skew watchdog | `harvester_boom_plan/safety.py`, `boom_plan_node.py` | clearance (`extend_safe_m=0.6`, `extend_stop_m=0.4`, `emergency_m=0.05`) + trunk-centering; **authoritative for hard contact safety** |
| Platform ground-truth pose | `harvester_boom_plan/dock_measurer.py` | FK from measured joints → `c_channel_reference` world pose |
| Telemetry | gateway → `v1/range/docking` (five ranges), `v1/docking/plan` | dashboard reads `center_line` for speed+gap |

### 2.2 Correct input semantics (verified)

- **Distance to trunk** = `center_range` (the single forward ray on
  `center_range_sensor_link`, `front_sensor_mount_link`, `+X` = measurement
  direction; URDF lines 683–687, Gazebo plugin lines 992–1008).  It reads the
  **front surface of the trunk** as the c-channel approaches.
- **Platform closing speed** = `-d(center_range)/dt` (already computed in
  `bridge._update_dock_speed()`).
- The **other four ranges** (`left_45`, `right_45`, `left_side`, `right_side`)
  are for **centering/skew** — already handled by `harvester_boom_plan/safety.py`.
  This plan does **not** duplicate that logic; it focuses on the *speed+gap*
  approach envelope and the operator-facing HUD.

### 2.3 Key platform geometry (from `kinematics.py` / URDF)

- `BOOM_PIVOT_HEIGHT = 1.76 m`, `BOOM_FIXED_LENGTH = 2.40 m`,
  `PLATFORM_TAIL = 1.02 m`, `EXTENSION_PER_STAGE = 2.40 m` × 4 stages.
- `center_range` Gazebo noise: **σ = 3 mm**; update rate **20 Hz**; max range 10 m.
- The final contact point: once docked, `center_range` reads ~0.23 m (trunk
  surface at the C-opening; see `BOOM_DOCK_PLAN.md` §"Post-dock observation").

### 2.4 The gap this plan closes

1. No **physics-derived** speed/distance thresholds (they are guesses).
2. No **simulation validation** of the guidance/HUD against real motion.
3. HUD not designed for operator **glanceability** (no stop-bar, no hysteresis,
   no stale-data "no telemetry" state distinct from SAFE).
4. The model does not distinguish "moving away" / "stationary" / "no data" in a
   way the operator can act on (current code maps "no data" → SAFE, which is
   misleadingly reassuring).

---

## 3. Design

### 3.1 Safety model: stopping-distance (physical) + TTC (advisory)

Replace the TTC-only floors with a **two-layer** model.  The authoritative
hard-stop gate is the existing `harvester_boom_plan/safety.py` `emergency_m`
near-contact + `extend_stop_m` hold; this plan's model is the *advisory* envelope
that tells the operator to slow before those hard gates trip.

**Stopping-distance bound (physics):**

Given a maximum safe deceleration `a_max` (m/s²) and a reaction/actuation latency
`t_latency` (s), the distance required to stop from closing speed `v` is:

```
d_stop(v) = v·t_latency + v² / (2·a_max)
```

The platform is **DANGER** when the current gap cannot stop within it, i.e.
`d_gap ≤ d_stop(v)`.  Solving for the maximum *safe* closing speed at a gap `d`:

```
v_max(d) = -a_max·t_latency + sqrt((a_max·t_latency)² + 2·a_max·d)
```

This is the physically correct "slow to X cm/s" figure the operator asked for —
it is *distance-dependent and deceleration-aware*, unlike `d / warn_ttc_s`.

**TTC (advisory, retained):** `TTC = d / v` stays as the glanceable "seconds to
impact" number and as a secondary **warn** trigger (a fast-but-far approach).

**Three states (with hysteresis to avoid flicker):**

| State | Trigger (enter) | Trigger (exit, hysteresis) |
|---|---|---|
| 🟢 SAFE | `d > warn_distance` **and** `TTC > warn_ttc` **and** `v ≤ v_warn(d)` | — |
| 🟠 WARN | `d ≤ warn_distance` **or** `TTC ≤ warn_ttc` **or** `v > v_warn(d)` | exit when all SAFE conditions hold for `debounce_s` |
| 🔴 DANGER | `d ≤ d_stop(v)` **or** `d ≤ danger_distance` | exit when `d > d_stop(v) + margin` **and** `d > danger_distance` for `debounce_s` |

where `v_warn(d) = v_max(d)` scaled by a `warn_margin` (< 1.0) so WARN fires
*before* the physical stop limit, leaving the operator headroom.

**Special inputs (must be explicit, not silently SAFE):**
- No center-range data / stale (`age > stale_s`) → `NO DATA` (grey), "awaiting
  range" — never green.
- `v ≈ 0` (stationary, `|v| < epsilon`) → SAFE only if `d > danger_distance`,
  else WARN/DANGER (too close to linger).
- `v < 0` (moving away) → SAFE (gap increasing).

### 3.2 Configuration (single tunable file)

Extend `config/safety_guidance.json` (kept backwards-compatible: missing keys fall
back to defaults, as today):

```json
{
  "a_max_m_s2": 0.10,          // max safe platform deceleration (tunable)
  "latency_s": 0.30,           // operator reaction + actuation latency
  "warn_margin": 0.7,          // WARN fires at 70% of the physical stop speed
  "warn_ttc_s": 4.0,           // advisory TTC warn band (retained)
  "danger_ttc_s": 1.5,         // advisory TTC danger band (retained)
  "warn_distance_m": 1.0,
  "danger_distance_m": 0.3,
  "stationary_epsilon_cm_s": 0.5,
  "stale_s": 2.0,              // no-data threshold
  "debounce_s": 0.4,           // hysteresis debounce (anti-flicker)
  "speed_ema_alpha": 0.35
}
```

`a_max_m_s2` is the single physical constant to tune (the boom's hydraulic
deceleration capability).  `latency_s` covers operator reaction + valve/actuator
delay.  Both are documented and exposed so the operator/tuning engineer can dial
the envelope to the real machine.

### 3.3 HUD redesign (operator-glanceable)

Replace the single panel in `HudOverlay.qml` with a purpose-built docking HUD
(shown only when `bridge.view === "docking"`), still QtQuick-2-primitives only
(no Controls2 on PySide2 5.14):

1. **State banner** (top): colour-coded word + one-line action:
   - 🟢 `APPROACH OK` · 🟠 `SLOW TO <X> cm/s` · 🔴 `STOP NOW` · ⬜ `NO RANGE DATA`
2. **Stop-bar** (the primary glanceable element): a horizontal bar showing the
   *current gap* vs the *required stopping distance* `d_stop(v)`.  A moving
   "STOP" marker at `d_stop(v)`; when the current-gap marker crosses it, the bar
   turns red.  This lets the operator see *margin* at a glance, not just a colour.
3. **Metrics row**: closing speed · gap · TTC · recommended speed.
4. **Progress-to-contact strip**: a thin vertical/horizontal strip of remaining
   distance to the final contact point (~0.23 m) so the operator sees how much
   approach travel is left.
5. **Hysteresis + debounce** fed from the bridge so the colour does not flicker.
6. **NO DATA** state is grey and distinct from SAFE.

### 3.4 Data flow

```
center_range (20 Hz, σ=3 mm)
   └─ gateway → v1/range/docking → dashboard TelemetryModel
        └─ bridge._update_dock_speed(): least-squares slope → closing speed v
             └─ EMA-smooth v
             └─ safety_guidance.evaluate(v, d, cfg)  → Guidance{state, ttc,
                    d_stop, v_max, recommended_speed, message}
                  └─ QML HUD (state banner, stop-bar, metrics)
```

No changes to the telemetry contract, gateway, or the read-only boundary: the
dashboard stays **view-only** (it never commands joints).  The existing
`harvester_boom_plan/safety.py` remains the hard-contact authority; this model is
advisory and complements it.

---

## 4. Simulation research / validation plan (the "perform research" part)

Because the guidance must be *validated*, not just unit-tested, the plan includes
a **repeatable scripted platform approach** driven by the boom joints (not the
vehicle), run in the existing Gazebo/RViz/dashboard stack.

### 4.1 Scripted approach harness (new, simulation-only)

Add a small **`harvester_dock` approach driver** (or a standalone script) that
commands a **slow, monotonic closing approach** of the platform toward the trunk
by ramping the boom joints, so the center_range gap decreases at a known,
controllable rate:

- **Reuse the existing autonomous dock path** (`dock_orchestrator.py` / the
  `boom_docking_demo --execute` IK targets) to raise + extend to the docking
  height, then **ramp the final approach** (extension/elevation) at several
  distinct speeds.
- Because the kinematic bridge is **rate-limited** (joint updates at 20 Hz, max
  step bounded), the platform's *closing speed* is already bounded — but the
  **research question** is what those achievable speeds are and whether the
  thresholds match.  The driver logs (timestamp, `center_range`, closing speed,
  guidance state) so the state transitions can be correlated with truth.

**Speed profiles to run (at least 3):**
1. **Slow** (~5–10 cm/s closing) — expect a long SAFE→WARN→DANGER sequence,
   green→orange→red, with the advisory readable.
2. **Moderate** (~20–30 cm/s) — expect WARN/DANGER earlier; validate the
   `v_max(d)` recommendation is physically reachable.
3. **Fast / near-limit** (as fast as the bridge allows) — expect DANGER to trip
   well before contact; validate the stop-bar and "STOP NOW".

### 4.2 Validation criteria

1. The **DANGER** state trips *before* the hard `emergency_m = 0.05 m` contact
   (i.e. the advisory envelope is more conservative than the hard watchdog) for
   every profile.
2. The **recommended speed** `v_max(d)` is ≤ the actual deceleration capability
   (no "slow to an impossible speed" advisory).
3. **No flicker** at state boundaries (hysteresis works) under the 20 Hz / σ=3 mm
   sensor.
4. **NO DATA** appears (grey) when the range stream is absent/stale, never a
   false green.
5. The HUD stop-bar crosses red at the correct gap across all profiles.

### 4.3 Run recipe (matches `run_simulation.sh`)

```
cd ~/ros2_ws && ./run_simulation.sh auto        # Gazebo+RViz, gateway, orchestrator, dashboard
# then drive the scripted approach (new driver), observe HUD + RViz
```

Or headless with `gui:=false rviz:=true` for CI-style capture.  The dashboard
logs guidance transitions; `dock_measurer.py` gives ground-truth pose for
correlating guidance state vs actual gap.

---

## 5. Implementation steps

1. **Model** — rewrite `safety_guidance.py` to add the stopping-distance bound,
   `NO DATA`/stationary/moving-away handling, hysteresis inputs, and expose
   `d_stop_m` / `v_max_cm_s` in `Guidance`.  Keep `evaluate()` signature
   compatible or update callers deliberately.
2. **Config** — extend `config/safety_guidance.json` with the new fields and
   document each (units + meaning); keep `SafetyConfig.load()` fallback-safe.
3. **Bridge** — in `bridge.py`, pass staleness + debounce state into the model,
   expose the new HUD properties (`dockStopDistanceM`, `dockMaxSpeedCmS`,
   `dockGapM`, `dockState` with `no_data`).
4. **HUD** — rebuild the docking HUD in `HudOverlay.qml` (state banner, stop-bar,
   metrics, progress strip, NO-DATA grey state), QtQuick-2 primitives only.
5. **Tests** — extend `test_safety_guidance.py` to cover stopping-distance math,
   hysteresis, NO-DATA, moving-away, and the deceleration-aware recommendation.
6. **Approach driver** — add the scripted approach harness under `harvester_dock`
   (simulation-only, view-consistent with the existing orchestrator boundary).
7. **Validate in sim** — run the three speed profiles, log transitions, and
   confirm the five criteria in §4.2; tune `a_max_m_s2` / `latency_s` /
   `warn_margin` from the results.
8. **Document** — update `docs/BOOM_DOCK_PLAN.md` §"Safety" and the
   `harvester_dock` README with the validated thresholds and the physical
   constants (`a_max`, `latency`) and where they came from.

---

## 6. Open decisions to confirm

1. **Deceleration constant `a_max_m_s2`**: I will seed it from the simulation
   (what the rate-limited bridge actually achieves) and make it a config knob.
   Confirm the *target* physical deceleration for the real machine, if known.
2. **Scope of the approach driver**: confirm the scripted approach should reuse
   the existing autonomous `dock_orchestrator` FSM (add a "slow approach" mode),
   vs a separate standalone script.  I recommend a **separate simulation-only
   script** so the orchestrator's deployed FSM semantics are untouched.
3. **Hard contact authority**: confirm `harvester_boom_plan/safety.py`
   (`emergency_m=0.05`, `extend_stop_m=0.4`) remains the hard-stop authority and
   this guidance stays **advisory/operator-facing only** (no auto-stop command),
   consistent with the dashboard's view-only boundary.
