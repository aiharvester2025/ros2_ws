# Model & Config Contract

The guidance model is **pure Python** (no Qt, no ROS) in
`src/harvester_dashboard/harvester_dashboard/safety_guidance.py`.  It ports to Orin verbatim.

## Public surface

- `SafetyConfig` (frozen dataclass) — all thresholds with defaults; `SafetyConfig.load(path)`.
- `Guidance` (frozen dataclass) — one evaluation result:
  `state, ttc_s, speed_cm_s, distance_m, stop_distance_m, max_speed_cm_s, recommended_speed_cm_s, message`.
- `evaluate(speed_cm_s, distance_m, cfg, *, stale=False, min_ttc_speed_cm_s=0.05) -> Guidance`.
- `state_message(state, guidance) -> str` — a message consistent with a *held* state (used during
  hysteresis debounce so text and colour never disagree).
- `SAFE/WARN/DANGER/NO_DATA` string constants (`'safe' / 'warn' / 'danger' / 'no_data'`).
- `default_config_path()` — best-effort path to `config/safety_guidance.json`.

## The physics (exact formulas)

```
d_stop(v) = v·latency + v²/(2·a_max)                 # v in m/s
v_max(d)  = -a_max·latency + sqrt((a_max·latency)² + 2·a_max·d)   # returns cm/s
```

- `_stop_distance_m(speed_cm_s, cfg)` converts `speed_cm_s/100` to m/s first.
- `_max_safe_speed_cm_s(distance_m, cfg)` returns cm/s.
- TTC = `distance / (speed/100)` only when `speed > stationary_epsilon_cm_s` (approaching).

## State logic (order matters — first match wins)

1. `speed is None or distance is None or stale` → `no_data`.
2. `distance <= 0` → `no_data`.
3. `distance <= danger_distance` → `danger` (absolute floor, even stationary).
4. `approaching and stop_dist >= distance` → `danger` (stopping distance).
5. `ttc is not None and ttc <= danger_ttc` → `danger` (advisory TTC).
6. `distance <= warn_distance` **or** `ttc <= warn_ttc` **or** `speed > warn_margin·v_max` → `warn`.
7. else → `safe` (includes moving away and stationary-outside-danger).

`evaluate()` is **stateless** — it does *not* apply hysteresis; the bridge does that across frames.

## Config file (`config/safety_guidance.json`)

```json
{
  "a_max_m_s2": 0.10, "latency_s": 0.30, "warn_margin": 0.7,
  "warn_ttc_s": 4.0, "danger_ttc_s": 1.5,
  "warn_distance_m": 1.0, "danger_distance_m": 0.3,
  "stationary_epsilon_cm_s": 0.5, "stale_s": 2.0,
  "debounce_s": 0.4, "speed_ema_alpha": 0.35
}
```

| Key | Unit | Meaning |
|---|---|---|
| `a_max_m_s2` | m/s² | max safe platform deceleration (the physical knob) |
| `latency_s` | s | operator reaction + actuation latency |
| `warn_margin` | — | WARN fires at this fraction of `v_max(d)` (0..1) |
| `warn_ttc_s` / `danger_ttc_s` | s | advisory TTC bands |
| `warn_distance_m` / `danger_distance_m` | m | absolute standoff floors |
| `stationary_epsilon_cm_s` | cm/s | below this → stationary |
| `stale_s` | s | range age beyond which → `no_data` |
| `debounce_s` | s | hysteresis debounce for safe/warn transitions |
| `speed_ema_alpha` | — | EMA smoothing factor for closing speed |

## Loader contract (must preserve)

- Missing keys → keep dataclass defaults.
- Absent/malformed JSON file → return default config (never raise).
- Non-dict JSON → return default config.
- **Sanitize out-of-range values** after load, never reject:
  - `a_max_m_s2 = max(1e-3, value)` (prevents divide-by-zero).
  - `latency_s = max(0.0, value)`.
  - `warn_margin`, `speed_ema_alpha` clamped to `[0, 1]`.
  - `stale_s`, `debounce_s` clamped to `>= 0`.
- A `TypeError`/`ValueError` during sanitization (e.g. a non-numeric string) falls back to defaults.

## Tests to port

`src/harvester_dashboard/test/test_safety_guidance.py` — pure pytest, no graph.  It covers:
missing/stale → `no_data`; stationary/moving-away → `safe`; TTC warn/danger; stopping-distance danger;
`recommended_speed` decreases with distance; `v_max` monotonic; `warn_margin` scaling; config
fallback + sanitization; `state_message` matches held state (never "STOP" on an orange banner).
