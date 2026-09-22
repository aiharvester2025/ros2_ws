# Cutter Model & Config Contract

Pure Python (no Qt, no ROS) in
`src/harvester_dashboard/harvester_dashboard/cutter_safety_guidance.py`.  Ports to Orin verbatim.

## Public surface

- `CutterConfig` (frozen dataclass) — thresholds with defaults; `CutterConfig.load(path)`.
- `CutterGuidance` (frozen dataclass) — `state, phase, clearance_m, speed_cm_s, ttc_s,
  stop_distance_m, max_speed_cm_s, recommended_speed_cm_s, message, phase_message`.
- `evaluate(speed_cm_s, clearance_m, cfg, *, stale=False, phase='idle') -> CutterGuidance`.
- `tip_clearance_m(range_m, cfg) -> float` — applies the sensor→tip offset.
- `next_phase(phase, clearance_m, speed_cm_s, cfg, *, stale=False) -> str` — measured step.
- `advance_phase(phase) -> str` — operator-confirmed step.
- `phase_message(phase, cfg)`, `state_message(state, guidance)`.
- Constants: `SAFE/WARN/DANGER/NO_DATA`; phases `PHASE_IDLE/APPROACH/ALIGN/OPEN/ADVANCE/CUT`;
  `PHASES` tuple.
- `default_config_path()`.

## The offset (critical)

The sensor is mounted **behind** the cutter tip (on `cutting_tool_left_range_sensor_link`, a child of
`cutting_tool_link`, facing +X).  Therefore:

```
tip_clearance_m = raw_range_m - sensor_to_tip_offset_m
```

Derivation (reference tool): the tool mesh visual extends to tool-local X ≈ 0.29 m (mesh 0.58 with
0.5 X-scale); the sensor is at tool-local X = 0.10 m → offset ≈ 0.19 m.  **Validate the sign**: if
clearance goes negative while the tip presses on an object, the offset is slightly too large (contact
penetration) — acceptable, but re-derive if it is large.

## States (same order as docking)

1. `speed is None or clearance is None or stale` → `no_data`.
2. `clearance <= danger_clearance` → `danger` (absolute floor, even stationary).
3. `approaching and stop_dist >= clearance` → `danger` (stopping distance).
4. `clearance <= warn_clearance` **or** `speed > warn_margin·v_max` → `warn`.
5. else → `safe`.

## Cut sequence

```
approach → (clearance <= ready_standoff + align_tolerance AND settled) → align
align → open → advance → cut    (advance_phase, operator-confirmed)
```

- `next_phase` only advances `approach → align` (measured).  **Two conditions are required:**
  1. the tip is inside the ready band (`clearance <= ready_standoff + align_tolerance`), **and**
  2. it is **settled and not being warned to slow** (`speed <= warn_margin * v_max(clearance)`).
  This prevents the "STOP, ready to cut" prompt from ever appearing while the operator is still
  being told to slow down (WARN) or stop (DANGER) — the two HUD signals must never disagree.
- `next_phase` returns `approach` when the range is stale/missing **from IDLE/APPROACH only**.
- **ALIGN/OPEN/ADVANCE/CUT hold their phase** through a stale/missing range (a dropout must NOT wipe
  the operator's in-progress steps) and are only advanced by `advance_phase`.
- `advance_phase` walks `align → open → advance → cut` (terminal at `cut`).
- `phase_message(phase, cfg)` is the operator prompt shown on the banner.
- The bridge also **blocks `advance_phase` while the state is `danger`/`no_data`** (the CONFIRM STEP
  button is disabled and relabelled "WAIT — tip too close"), so the cut sequence cannot advance with
  the tip too close or with no range.

## Config file (`config/cutter_safety_guidance.json`)

| Key | Unit | Meaning |
|---|---|---|
| `sensor_to_tip_offset_m` | m | sensor face → cutter tip (forward) |
| `a_max_m_s2` | m/s² | max safe cutter deceleration |
| `latency_s` | s | operator reaction + actuation latency |
| `warn_margin` | — | WARN at this fraction of `v_max` (0..1) |
| `warn_clearance_m` | m | warn clearance floor |
| `danger_clearance_m` | m | danger clearance floor |
| `ready_standoff_m` | m | tip distance to "STOP, ready to cut" |
| `advance_distance_m` | m | forward move before cutting |
| `align_tolerance_m` | m | band around `ready_standoff` for align |
| `stationary_epsilon_cm_s` | cm/s | stationary threshold |
| `stale_s` | s | range age → `no_data` |
| `debounce_s` | s | hysteresis (safe/warn only) |
| `speed_ema_alpha` | — | EMA smoothing factor |

**Validated values (2026-09-22, Gazebo):** `sensor_to_tip_offset_m=0.19`, `a_max_m_s2=0.10`,
`latency_s=0.30`, `warn_clearance_m=0.30`, `danger_clearance_m=0.10`, `ready_standoff_m=0.20`,
`align_tolerance_m=0.05`.  The ready standoff is deliberately clear of the danger band so the tip
settles in a stable WARN "ready-to-cut" state.

## Loader contract (must preserve)

Missing keys → defaults; absent/malformed JSON → defaults (never raise); sanitize out-of-range:
`a_max >= 1e-3` (divide-by-zero guard, also applied inside the math helpers so a hand-built config
cannot crash), `warn_margin`/`speed_ema_alpha` in `[0,1]`, distances/`stale_s`/`debounce_s` `>= 0`.
