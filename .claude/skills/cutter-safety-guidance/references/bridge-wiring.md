# Cutter Bridge Wiring

How `src/harvester_dashboard/harvester_dashboard/bridge.py` turns the raw cutter range into a
smoothed closing speed, a clearance state, and a cut-sequence phase.  Port to Orin as-is; the data
source is the same canonical `v1/range/cutter` channel.

## 1. Input

- Read the cutter record from `model.snapshot_ranges()` (second element; the
  `v1/range/cutter` payload, `telemetry_key: cutter_forward`).
- Store the **raw range**; the offset is applied in the model (`tip_clearance_m`), so the derivative
  and the clearance are consistent.
- Track `self._cutter_range_recv_monotonic = time.monotonic()` for staleness.

## 2. Closing-speed derivation (`_update_cutter_speed`)

- Rolling `deque` of `(acquisition_timestamp_ns, raw_range)`.
- Prune older than `_CUTTER_SPEED_MAX_SAMPLE_AGE_S = 1.5` s.
- Least-squares slope; closing speed = `-slope * 100.0` cm/s (shrinking range → positive).

## 3. Evaluate + phase (`_recompute_cutter_guidance`)

1. EMA-smooth the raw speed with `speed_ema_alpha`.
2. `clearance = tip_clearance_m(raw_range, cfg)` (None when no range).
3. `stale = (now - recv) > cfg.stale_s` (or True when a range exists but no receipt time).
4. Advance the phase: `self._cutter_phase = next_phase(self._cutter_phase, clearance, speed, cfg,
   stale=stale)` (only the measured `approach→align` step).
5. `candidate = cutter_evaluate(smoothed, clearance, cfg, stale=stale, phase=self._cutter_phase)`.
6. Hysteresis: adopt a new `_cutter_state` immediately for `danger`/`no_data`, else after
   `debounce_s`.  Use `cutter_state_message(held_state, candidate)` when the held state differs, so
   the message matches the shown colour.
7. Emit `cutter_safety_changed` only when the result changed.

## 4. Operator confirmation

- Slot `cutter_confirm_phase()` calls `advance_phase(self._cutter_phase)` then recomputes — this
  advances the OPEN → ADVANCE → CUT prompts (no sensors for those actions).
- **It is a no-op while the clearance state is `danger` or `no_data`**, so the cut sequence cannot be
  advanced with the tip too close (or with no range).  The HUD disables the button in the same
  condition.

## 5. QML properties to expose (via `Property`)

| Property | Type | Source |
|---|---|---|
| `cutterSafetyState` | str | `safe` / `warn` / `danger` / `no_data` |
| `cutterPhase` | str | cut-sequence phase |
| `cutterGuidanceText` | str | clearance alert message |
| `cutterPhaseText` | str | operator prompt for the phase |
| `cutterClearanceM` | float | offset-corrected tip clearance |
| `cutterRawRangeM` | float | raw sensor reading |
| `cutterMaxSpeedCmS` | float | `v_max(clearance)` |
| `cutterStopDistanceM` | float | `d_stop(v)` |
| `cutterSpeedSmoothedCmS` | float | EMA-smoothed closing speed |

All float getters return `float('nan')` when there is no measurement, so QML renders `—` via
`isFinite(...)`.

## Invariants

- Initial `cutterSafetyState = 'no_data'` and `cutterPhase = 'idle'` (never a false green).
- `danger`/`no_data` bypass debounce; only `safe`/`warn` are debounced.
- The cutter section is **independent** of the docking section — do not share their state/samples.
- Never emit socket traffic from this path (view-only).
