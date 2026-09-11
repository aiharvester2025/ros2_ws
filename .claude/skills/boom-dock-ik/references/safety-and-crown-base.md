# Docking safety FSM + crown-base / height estimation

Two ROS-free modules carry the perception and safety logic. This page records the non-obvious
design decisions and the ground-truth numbers.

- Safety: `src/harvester_boom_plan/harvester_boom_plan/safety.py`
- Height/crown-base: `src/harvester_dock/harvester_dock/live_height_estimator.py`
- Distance/reachability: `src/harvester_dock/harvester_dock/distance_estimator.py`

## Five-sensor safety state machine (`safety.py`)

Five fixed range sensors: `center`, `left_45`, `right_45`, `left_side`, `right_side`, plus an
optional moving `cutter` range. `evaluate(center, c45_l, c45_r, side_l, side_r, cutter, cfg,
stale)` returns a `SafetyResult(phase, centering, notes)`.

It does **two jobs** (per the plan):

1. **Clearance/skew watchdog** — monotonic gap guard, trips `EMERGENCY_STOP` on near-contact.
2. **Trunk-centering confirmation** — side pair + ±45 pair reconstruct the trunk centreline
   inside the C-opening and *gate* `DOCKING_OK`.

### Phase priority (highest first — order matters, do not reorder casually)

1. `WAITING_FOR_TREE` — no fresh data (all `None` or `stale`).
2. `EMERGENCY_STOP` — `min(present) <= emergency_m` (near-contact always wins).
3. `DOCKING_OK` — all present ranges `<= docking_gap_m` AND centred (`side_pair_ok` AND `diag_ok`).
4. `LEVEL_OK` / `ALIGN_OK` — centering sub-gates while approaching.
5. `HOLD_AT_DISTANCE` / `EXTEND_OK` — watchdog clearance gates (approach only).
6. `READY` — default.

### Centering math

- `diameter = side_l + side_r`
- `centre_offset_y = (side_l - side_r) / 2`; `side_pair_ok = |centre_offset_y| <= centering_tolerance_m`
- `skew45 = |c45_l - c45_r|`; `diag_ok = skew45 <= align_skew_m`

### The subtle ordering bug (why it's ordered this way)

A **docked** state has all sensors naturally small (0.1–0.3 m — the C-channel is *around* the
trunk), which is the **success** state, not "hold". The watchdog clearance gates (`EXTEND_OK` /
`HOLD_AT_DISTANCE`) only apply **during approach**, i.e. before centering confirms the trunk is
inside. So centering/docking must be checked **before** the front-clearance watchdog — otherwise
a fully-docked pose is mis-reported as `HOLD_AT_DISTANCE`. This was a real bug fixed in the
session: the evaluator checks `DOCKING_OK` (step 3) before the clearance gates (step 5).

Also note the emergency threshold is `0.05 m` — a test value of exactly `0.05` trips
`EMERGENCY_STOP`, not `LEVEL_OK`, so test fixtures must use values above it when probing centering.

### Thresholds (single source of truth: `config/boom_docking.yaml`, mirrored in `dock.yaml`)

```yaml
safety:
  extend_safe_m: 0.6          # min clearance to permit further extension
  extend_stop_m: 0.4          # hold extension below this
  align_skew_m: 0.08          # |left_45 - right_45| centering gate
  docking_gap_m: 0.18         # all five within this -> DOCKING_OK
  emergency_m: 0.05           # any sensor <= this -> EMERGENCY_STOP
  stale_receipt_timeout_s: 0.30
  centering_tolerance_m: 0.10 # trunk centre within this of C-opening centreline
```

`SafetyConfig` is a dataclass with **no defaults** — the caller must supply every field from YAML
so the dataclass and config cannot silently diverge.

### Range "no return" convention (critical)

Range sensors report `3.4e38` (= `FLT_MAX`) for "no return", which is `> max_range = 3.0 m`.
`boom_plan_node` treats any `range > max_range` as absent (`None`), so a far/absent sensor cannot
pollute the centering diameter (which would otherwise become `6.8e38`). Keep this rejection.

### Two-phase gating + post-dock observation

- **Raise/extend phase is open-loop**: sensors legitimately out of range → `WAITING_FOR_TREE` must
  NOT block. Only `EMERGENCY_STOP` and `INFEASIBLE_DOCK_HEIGHT` hard-block.
- **Post-dock**: once the trunk is inside the C-opening, the forward `center_range` reads ~0.23 m
  (correct final contact) while the side/±45° beams pass the trunk and report "no return". The
  **centre sensor alone confirms final contact**; the side pair matters during **approach**.

## Crown-base detection and the docking-height rule

**The central, costly lesson of this session:** the docking height must be derived from the
**crown base** (trunk-end, where fronds/FFBs first appear), NOT from `tree_top - offset`.

Reference tree ground truth (`oil_palm_tree_description/config/tree_targets.yaml`):
- trunk top `z = 12.0 m`, crown base `z = 9.2 m`.
- lowest branch cut targets `z = 9.45 m`; lowest FFB `z = 9.55 m`.

So `tree_top - 2.0 = 10.0 m` lands **inside the frond/FFB zone** and caused the platform to
crash into the canopy. Correct docking height = `crown_base - 2.0 m ≈ 7.2 m` (clean trunk).

### `crown_base_from_density` (proven method)

The canopy annulus (radial distance `r in [top_radius, 2.0 m]`) is nearly empty along the bare
trunk and densely populated above the crown base. A per-height-bin histogram (bin 0.25 m) shows a
sharp jump; the crown base is the bottom of the **first bin** crossing `threshold`.

- Default threshold `5000` pts/bin is for the dense offline 40-step sweep (`tree_scan_002`,
  ~28k pts/bin). The live 5-step nod yields only ~100–150 pts/bin, so the live threshold is
  `crown_density_threshold: 50` (config). **The threshold must match the sweep density.**
- `crown_z_min_m: 5.0` is a **fixed** lower bound, well above the harvester self-clutter (~2–3 m)
  and below the true crown base (~9.2 m). Do NOT tie it to the trunk top (e.g. `top - 3.5`) — that
  clips into the frond zone and biases the crown base low (observed: `crown_base == top - 3.5`).

When detection fails, `estimate_height` falls back to `trunk_top - 2.0` **but sets
`trunk_end_valid = False`**. The orchestrator **refuses to plan/dock** on an invalid trunk-end
(`PLAN REFUSED` / `DOCK REFUSED`, `crown_base_undetected`), because that fallback lands in the
canopy. Never dock on `trunk_end_valid == False`.

### Trunk-axis and top estimation

- `fit_trunk_axis`: median XY in a mid-trunk band `z in [1, 8]`, `|y| <= 1`, then **radius-correct**
  for half-cylinder bias: a single-sided scan's median X sits at `centre - r*(2/pi)`, so add
  `(2/pi)*r` (r ≈ 0.25 m). Verified: median 8.283 + 0.637*0.35 ≈ 8.51 vs true 8.5.
- `trunk_top_estimate`: the **`max`** (upper envelope) is correct — fronds/occlusion can only
  *hide* the top, never exceed it, so percentiles are biased low.

### Distance / reachability (`distance_estimator.py`)

```python
d_horiz = trunk_axis_x - pivot_x          # pivot offset -0.87 m from base_link
max_horiz_reach = MAX_BOOM_LENGTH + PLATFORM_TAIL   # 12.0 + 1.02 = 13.02 m
reachable = d_horiz <= max_horiz_reach - margin      # margin 0.2 m
needed_advance_m = max(0, d_horiz - (max_horiz_reach - margin))  # >0 => move prime mover closer
```

## Docking reference point (decision)

- **Tree side:** trunk **centreline** at `H_dock` = `(trunk_x, trunk_y, H_dock)`.
- **Platform side:** `c_channel_reference` (co-located with the platform, +X through the C-opening
  toward the tree). IK goal = place `c_channel_reference` exactly at the trunk-centre docking point.
