# Estimation algorithms

Source of truth: `src/harvester_dock/harvester_dock/live_height_estimator.py` (live ROS-free port)
and `analyze_tree_scan_v2.py` / `analyze_tree_scan.py` (offline). This page records the algorithms
and the *why*, including the debugging lessons.

## Ground truth (reference tree, `tree_targets.yaml`)

- trunk top `z = 12.0 m`; crown base `z = 9.2 m`; crown center `z = 10.6 m`.
- lowest branch cut targets `z = 9.45 m`; lowest FFB `z = 9.55 m`.
- trunk diameter: base 0.70 m → top 0.50 m (radius ~0.25 m at top, tapers).
- tree is at world `(8.5, 0, 0)` in the reference sim.

## Trunk axis fit (`fit_trunk_axis`)

Median XY of points in a mid-trunk band `z in [1, 8]`, `|y| <= 1`, then **radius-correct**:

```python
med_x = median(x[band]); med_y = median(y[band])
axis_x = med_x + (2/pi) * trunk_radius   # half-cylinder bias correction
```

A single-sided scan of a cylinder of radius r has its median X at `centre - r*(2/pi)`, so adding
`r*(2/pi)` recovers the true centreline. Verified: median 8.283 + 0.637*0.35 ≈ 8.51 vs true 8.5.

## Trunk top (`trunk_top_estimate`)

The **`max`** Z of trunk-cylinder points is the correct estimator: fronds/occlusion can only *hide*
the top, never exceed it, so `max` is an upper envelope. Percentiles (p99.9) are dragged low by the
bulk of lower trunk points.

## Crown base (`crown_base_from_density`)

The canopy annulus `r in [top_radius, 2.0]` is nearly empty on the bare trunk and densely populated
above the crown base. Per-height-bin histogram (bin 0.25 m); the crown base is the **bottom of the
first bin** crossing `threshold`.

- `threshold` default 5000 for the dense 40-step sweep (~28k pts/bin); **50** for the live 5-step nod
  (~100–150 pts/bin). Must match sweep density.
- `z_min` is a FIXED lower bound (5.0 m). The harvester arm/body self-occlusion puts dense clutter in
  the annulus at low z (~2 m) which would otherwise be misread as the crown base. Never tie `z_min`
  to the trunk top — `top - 3.5` clips into the frond zone and biases crown base low (observed:
  `crown_base == top - 3.5` exactly).

## Trunk-taper base fit (encoder-free z=0 recovery)

The LiDAR cannot see below ~1.8 m (harvester/arm occludes the downward view), so "ground" from the
5th percentile is unreliable (~1.8 m, not 0). The taper fit recovers z=0 encoder-free: fit
diameter-vs-height (0.70 m base → 0.50 m crown) and extrapolate to the nominal 0.70 m base.

### The taper-fit debugging lesson (important)

The first taper-fit attempt was garbage (`diameter(z) = -2.621*z + 38.5`, base_z=14.4 m) because
`np.percentile(band_r, 95)` picks up **far clutter** (other trees/ground at the same z but different
XY — the merged cloud spans X up to 39 m). The fix: estimate trunk diameter with a **robust median /
histogram mode of the radial distribution in a tight annulus**, not a high percentile. The trunk is
the *dense* cluster near r ≈ 0.25–0.35 m; sparse far points must be rejected.

## d_short (minimum slant range) — the user's diagram idea

When the LiDAR sweeps vertically past the trunk top, the slant range to the trunk cylinder passes
through a minimum; that frame marks the trunk-top crossing.

- **Sound and portable** as a sensor-only signal (needs only the LiDAR, not encoders).
- **`H_L − d_short` is only valid** if `H_L` (LiDAR height) is known. `H_L` requires the full
  kinematic chain INCLUDING the un-instrumented `cutting_arm_lift_joint` — so it's only recoverable
  when (a) the sensor IMU supplies the attitude, or (b) the lift joint is at a known hard stop.
- **Caveats:** the minimum is flat near the top (use a parabolic fit over ~5 samples, not `argmin`);
  restrict d_short to trunk-cylinder points (r < 0.45 m), not the canopy annulus, or a frond becomes
  the "closest" point; add the trunk half-radius (~0.225 m) for the centreline.

## Fused architecture (final recommended)

Primary: kinematic `H_L` (boom angle + boom ext + platform pitch/roll + LiDAR IMU pitch) → `trunk_top
= H_L(at d_short step) − d_short + r_trunk`.

Cross-checks:
1. Trunk-taper base fit (encoder-free z=0).
2. Crown-base density transition.
3. Range-sensor closed-loop `θ_lift` solve (trunk range + known axis) vs IMU `θ_lift` — a redundancy
   check between two independent attitude sources.

## `estimate_height` result contract

`HeightEstimate` carries `height_m`, `trunk_axis_xy_m`, `crown_base_m`, `crown_base_method`,
`trunk_end_valid`, `trunk_end_reason`. When crown-base detection fails it returns
`trunk_end_valid=False` with `crown_base = max(0, height - 2.0)` and a reason string — callers MUST
treat `trunk_end_valid=False` as "do not dock".
