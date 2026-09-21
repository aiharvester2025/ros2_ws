# Bridge Wiring

How the dashboard bridge (`src/harvester_dashboard/harvester_dashboard/bridge.py`) turns the raw
`center_line` gap into a smoothed closing speed, a safety state (with hysteresis), and QML
properties.  Port this section to Orin as-is; the data source is the same canonical
`v1/range/docking` channel.

## 1. Closing-speed derivation (`_update_dock_speed`)

- Read the `center_line` record from `model.snapshot_ranges()` (the `v1/range/docking` list).
- Keep a rolling `deque` of `(acquisition_timestamp_ns, distance_m)` samples.
- Prune samples older than `_SPEED_MAX_SAMPLE_AGE_S = 1.5` s.
- Compute the **least-squares slope** of `distance` vs `time` over the window; closing speed =
  `-slope * 100.0` cm/s (a *shrinking* gap → positive closing speed).
- If no/invalid center range, age out samples and set speed/center-distance to `None`.

## 2. Staleness

- Record `self._center_range_recv_monotonic = time.monotonic()` whenever a valid center range is
  processed.
- In `_recompute_safety_guidance`, `stale = (now - recv) > cfg.stale_s`; if there is a center
  distance but no receipt timestamp, treat as stale.

## 3. Hysteresis debounce (`_recompute_safety_guidance`)

1. EMA-smooth the raw speed with `speed_ema_alpha`.
2. Call `evaluate(smoothed_speed, center_distance, cfg, stale=stale)` → `candidate`.
3. Only adopt a *new* `self._guidance_state` when:
   - the candidate state is `danger` or `no_data` (adopt **immediately**), or
   - `now - self._guidance_state_since >= cfg.debounce_s`.
4. **Message consistency:** if the candidate state differs from the (held) `_guidance_state`, use
   `state_message(self._guidance_state, candidate)` for the displayed message — so the text matches
   the shown colour during debounce.  Otherwise use `candidate.message`.
5. Emit `dock_safety_changed` only when the resulting `Guidance` changed.

## 4. QML properties to expose (via `Property`)

| Property | Type | Source |
|---|---|---|
| `dockSpeedCmS` | float | raw (unsmoothed) closing speed, NaN when absent |
| `dockCenterDistanceM` | float | forward gap, NaN when absent |
| `dockSpeedSmoothedCmS` | float | EMA-smoothed closing speed |
| `dockSafetyState` | str | `safe` / `warn` / `danger` / `no_data` |
| `dockGuidanceText` | str | the actionable message |
| `dockRecommendedSpeedCmS` | float | `warn_margin·v_max(d)` (NaN when safe/no-data) |
| `dockMaxSpeedCmS` | float | `v_max(d)` physical limit (NaN when absent) |
| `dockStopDistanceM` | float | `d_stop(v)` at current speed (NaN when absent) |
| `dockTtcS` | float | time-to-collision (NaN when stationary/absent) |

All float getters return `float('nan')` when there is no measurement, so QML can render an explicit
`—` via `isFinite(...)`.

## Invariants

- `dockSafetyState` default/initial is `'no_data'` (never `'safe'`), and the initial `Guidance`
  message is `'approach: awaiting center range'`.
- `danger` and `no_data` bypass debounce; only `safe`/`warn` are debounced.
- Never emit socket traffic from this path — the bridge stays view-only.
