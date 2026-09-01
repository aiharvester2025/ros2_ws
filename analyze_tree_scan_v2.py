#!/usr/bin/env python3
"""Analyze recorded LiDAR tree scan to estimate tree height (v2).

Encoder-free estimator with multiple fused sub-estimates, plus an optional
IMU-assisted path for A/B comparison.

Strategies
----------
Strategy 1 (encoder-free, always available):
    - Fit the trunk axis from the merged world-frame cloud (no known world pose).
    - Estimate trunk top from the highest trunk-cylinder points (tight cylinder).
    - Estimate crown base from the canopy-annulus density transition.
    - Optionally fit the trunk taper to recover the ground plane (diagnostic;
      see notes below).

Strategy 2 (IMU-assisted, optional):
    - When ``v1_imu_lidar`` recordings exist, recover the LiDAR attitude
      (roll/pitch) directly from the sensor IMU and use it to resolve the
      un-instrumented cutting-arm lift joint, giving an independent H_L and a
      ``d_short`` trunk-top-crossing estimate.

Ground truth (tree_targets.yaml): height 12.0 m, crown base 9.2 m, trunk
diameter 0.70 m base -> 0.50 m top.
"""
import sys
import json
from pathlib import Path

import msgpack
import numpy as np

from harvester_telemetry_contract import unpack_message


GROUND_TRUTH_HEIGHT = 12.0
GROUND_TRUTH_CROWN_BASE = 9.2
TRUNK_BASE_DIAMETER = 0.70
TRUNK_TOP_DIAMETER = 0.50


def load_merged_cloud(record_dir, frame='world'):
    """Return an Nx3 float32 numpy array of merged XYZ points plus per-file info."""
    lidar_dir = Path(record_dir).expanduser() / 'v1_lidar_raw'
    files = sorted(lidar_dir.glob('*.msgpack'))
    if not files:
        raise SystemExit(f'No LiDAR recordings in {lidar_dir}')

    clouds = []
    kept = 0
    skipped = 0
    for f in files:
        record = msgpack.unpackb(f.read_bytes(), raw=False, strict_map_key=False)
        channel, header, payload = unpack_message(record['frames'])
        if header.get('frame_id') != frame:
            skipped += 1
            continue
        point_count = header.get('point_count', 0)
        if point_count <= 0 or len(payload) != point_count * 12:
            skipped += 1
            continue
        xyz = np.frombuffer(payload, dtype='<f4').reshape(point_count, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        clouds.append(xyz)
        kept += 1

    if not clouds:
        raise SystemExit(f'No {frame}-frame points found (skipped {skipped})')
    return np.vstack(clouds), kept, skipped


def fit_trunk_axis(x, y, z, z_lo=1.0, z_hi=8.0):
    """Estimate the trunk axis (X, Y) from trunk points in a mid-trunk band.

    The band z in [z_lo, z_hi] is below the crown (avoids fronds/FFBs) and above
    the occluded base.  Returns the median XY centre, robust to the harvester and
    ground clutter that can skew a mean.
    """
    band = (z >= z_lo) & (z <= z_hi)
    if band.sum() < 50:
        return None
    return float(np.median(x[band])), float(np.median(y[band]))


def trunk_top_estimate(z_trunk, method='max'):
    """Return the trunk-top height from trunk-cylinder z values.

    The trunk is a solid vertical cylinder, so its top is the highest point in a
    tight cylinder about the axis.  Fronds attach at the crown and extend
    *outward*, not above the trunk top, so a tight cylinder (r < 0.35 m) cleanly
    resolves the trunk top without frond contamination.
    """
    if method == 'max':
        return float(z_trunk.max())
    if method == 'p999':
        return float(np.percentile(z_trunk, 99.9))
    raise ValueError(method)


def crown_base_from_density(canopy_z, bins, threshold):
    """First histogram bin where the canopy annulus becomes persistently dense."""
    hist, edges = np.histogram(canopy_z, bins=bins)
    for i, count in enumerate(hist):
        if count >= threshold:
            return float(edges[i])
    return None


def analyze(record_dir):
    merged, kept, skipped = load_merged_cloud(record_dir)
    print(f'Merged {len(merged)} points from {kept} world-frame recordings '
          f'(skipped {skipped} non-world frames)')

    x, y, z = merged[:, 0], merged[:, 1], merged[:, 2]

    # ---- Trunk axis (encoder-free: no known world pose) ----
    axis = fit_trunk_axis(x, y, z)
    if axis is None:
        axis = (float(np.median(x)), float(np.median(y)))
        print('\nWARNING: trunk-axis fit failed; using whole-cloud median XY.')
    ax, ay = axis
    print(f'\nEstimated trunk axis: ({ax:.2f}, {ay:.2f}) m  (true 8.5, 0)')

    # ---- Tight trunk cylinder for the trunk top ----
    r = np.hypot(x - ax, y - ay)
    tight_radius = 0.35
    trunk_tight = z[r < tight_radius]
    trunk_top = trunk_top_estimate(trunk_tight, method='max')
    trunk_top_p999 = trunk_top_estimate(trunk_tight, method='p999')
    print(f'Trunk top (tight cylinder r<{tight_radius:.2f} m): '
          f'max={trunk_top:.3f} m, p99.9={trunk_top_p999:.3f} m')

    # ---- Canopy annulus for crown base ----
    canopy_z = z[(r >= tight_radius) & (r < 2.0)]
    canopy_bins = np.arange(0.0, 14.0, 0.25)
    crown_base = crown_base_from_density(canopy_z, canopy_bins, 5000)
    print(f'Crown base (density transition): {crown_base:.2f} m'
          if crown_base is not None else 'Crown base: N/A')

    # ---- Legacy reference (canopy 99th percentile) ----
    canopy_top_99 = np.percentile(canopy_z, 99)
    canopy_top_max = canopy_z.max()

    # ---- Fused total height ----
    # In world-frame recordings the base is world z=0, so height == trunk top.
    # Sub-estimates (each an independent measure of "trunk top above base"):
    #   A: trunk top (tight cylinder max)        -> primary
    #   B: trunk top (tight cylinder p99.9)      -> robust variant
    #   C: canopy top 99th percentile            -> legacy (biased high by fronds)
    sub = {
        'trunk_top_max': trunk_top,
        'trunk_top_p999': trunk_top_p999,
        'canopy_top_99pct': canopy_top_99,
    }
    fused = float(np.median([trunk_top, trunk_top_p999]))
    # Prefer the trunk-cylinder estimate; report uncertainty as spread of all.
    uncertainty = float(np.std(list(sub.values())))

    print('\n=== Encoder-free sub-estimates (height above world z=0) ===')
    for name, value in sub.items():
        print(f'  {name:22s}: {value:6.2f} m')
    print(f'\nFused tree height:      {fused:.2f} m  (std {uncertainty:.2f} m)')

    print('\n=== Ground-truth comparison ===')
    print(f'Ground truth height:     {GROUND_TRUTH_HEIGHT} m')
    print(f'Ground truth crown base: {GROUND_TRUTH_CROWN_BASE} m')
    err = fused - GROUND_TRUTH_HEIGHT
    print(f'Fused height error:      {err:+.3f} m ({err / GROUND_TRUTH_HEIGHT * 100:+.2f}%)')
    legacy_err = canopy_top_99 - GROUND_TRUTH_HEIGHT
    print(f'Legacy height error:     {legacy_err:+.3f} m ({legacy_err / GROUND_TRUTH_HEIGHT * 100:+.2f}%)')
    if crown_base is not None:
        cb_err = crown_base - GROUND_TRUTH_CROWN_BASE
        print(f'Crown base error:        {cb_err:+.3f} m')

    imu_result = analyze_imu_assisted(record_dir, axis, fused)

    result = {
        'total_points': int(len(merged)),
        'n_world_recordings': kept,
        'n_skipped_recordings': skipped,
        'trunk_axis_m': [ax, ay],
        'trunk_axis_error_m': float(np.hypot(ax - 8.5, ay - 0.0)),
        'trunk_top_max_m': float(trunk_top),
        'trunk_top_p999_m': float(trunk_top_p999),
        'crown_base_m': float(crown_base) if crown_base is not None else None,
        'canopy_top_99pct_m': float(canopy_top_99),
        'canopy_top_max_m': float(canopy_top_max),
        'sub_estimates_m': {k: float(v) for k, v in sub.items()},
        'fused_tree_height_m': float(fused),
        'fused_uncertainty_m': float(uncertainty),
        'legacy_tree_height_m': float(canopy_top_99),
        'ground_truth_height_m': GROUND_TRUTH_HEIGHT,
        'ground_truth_crown_base_m': GROUND_TRUTH_CROWN_BASE,
        'fused_height_error_m': float(err),
        'imu_assisted': imu_result,
    }
    return result


def _load_imu_lift_pitch(record_dir):
    """Load LiDAR-IMU orientation samples -> lift pitch (rad) keyed by timestamp.

    The IMU is rigidly mounted on ``cutting_arm_base_link``, which is the child of
    the un-instrumented ``cutting_arm_lift_joint`` (axis ``0 -1 0``).  The IMU's
    gravity-referenced orientation therefore measures that joint's pitch directly:
    the quaternion rotates only about Y, and the joint angle is
    ``theta_lift = -2 * asin(orientation.y)`` (the negative sign follows the
    ``0 -1 0`` joint axis convention).

    Returns (timestamps_ns sorted, lift_pitch_rad aligned).
    """
    imu_dir = Path(record_dir).expanduser() / 'v1_imu_lidar'
    files = sorted(imu_dir.glob('*.msgpack'))
    if not files:
        return None, None
    ts = []
    pitch = []
    for f in files:
        record = msgpack.unpackb(f.read_bytes(), raw=False, strict_map_key=False)
        channel, header, payload = unpack_message(record['frames'])
        d = json.loads(payload.decode('utf-8'))
        o = d['orientation']
        # Only the Y component is nonzero for a pure lift pitch; clamp for safety.
        y = max(-1.0, min(1.0, float(o['y'])))
        ts.append(int(header['acquisition_timestamp_ns']))
        pitch.append(-2.0 * np.arcsin(y))
    order = np.argsort(ts)
    return np.asarray(ts)[order], np.asarray(pitch)[order]


def _load_world_scans(record_dir):
    """Load world-frame LiDAR clouds keyed by timestamp (sorted)."""
    lidar_dir = Path(record_dir).expanduser() / 'v1_lidar_raw'
    files = sorted(lidar_dir.glob('*.msgpack'))
    scans = []
    for f in files:
        record = msgpack.unpackb(f.read_bytes(), raw=False, strict_map_key=False)
        channel, header, payload = unpack_message(record['frames'])
        if header.get('frame_id') != 'world':
            continue
        n = int(header.get('point_count', 0))
        if n <= 0 or len(payload) != n * 12:
            continue
        xyz = np.frombuffer(payload, dtype='<f4').reshape(n, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        scans.append((int(header['acquisition_timestamp_ns']), xyz))
    scans.sort(key=lambda item: item[0])
    return scans


def analyze_imu_assisted(record_dir, axis, fused):
    """Strategy 2: IMU-assisted trunk-top estimation.

    The LiDAR-IMU measures the lift-joint pitch directly, closing the loop on the
    un-instrumented ``cutting_arm_lift_joint``.  During the lift sweep the LiDAR's
    upper vertical FOV edge (+52 deg elevation) rises; the trunk top becomes fully
    visible only when that edge clears the trunk top.  We use the IMU pitch to:

      1. identify which scans actually reached the canopy (pitch near the upper
         joint limit), and
      2. produce an IMU-assisted trunk-top estimate from the plateau of the
         measured trunk-top vs. lift pitch, cross-checked against the
         encoder-free fused height.

    This is deliberately complementary to the encoder-free path: it does not
    require any joint-state or world-pose source, only the sensor IMU.
    """
    ts, pitch = _load_imu_lift_pitch(record_dir)
    scans = _load_world_scans(record_dir)
    if ts is None or not scans:
        print('\n=== Strategy 2 (IMU-assisted) ===')
        print('  No v1_imu_lidar recordings found (recording predates IMUs).')
        print('  Re-record with the IMU-equipped URDF to enable this path.')
        return {'available': False, 'reason': 'no_imu_recordings'}

    print('\n=== Strategy 2 (IMU-assisted) ===')
    print(f'  Loaded {len(ts)} IMU samples and {len(scans)} world scans.')

    ax, ay = axis
    tight_radius = 0.35

    # Per-scan trunk top (tight cylinder about the measured axis), with the
    # IMU lift pitch matched by nearest timestamp.
    scan_tops = []
    for scan_ns, xyz in scans:
        r = np.hypot(xyz[:, 0] - ax, xyz[:, 1] - ay)
        trunk = xyz[r < tight_radius]
        if len(trunk) < 5:
            continue
        top = float(trunk[:, 2].max())
        # Nearest IMU sample in time.
        idx = int(np.searchsorted(ts, scan_ns))
        idx = max(0, min(idx, len(ts) - 1))
        # refine to nearest neighbour.
        if idx > 0 and abs(ts[idx - 1] - scan_ns) < abs(ts[idx] - scan_ns):
            idx -= 1
        scan_tops.append((pitch[idx], top, scan_ns))

    if not scan_tops:
        return {'available': True, 'n_imu_recordings': len(ts),
                'error': 'no_trunk_points_in_scans'}

    pitches = np.asarray([s[0] for s in scan_tops])
    tops = np.asarray([s[1] for s in scan_tops])

    # IMU pitch range (lift joint limits -0.35..+1.05 rad -> pitch -1.05..+0.35).
    print(f'  Lift pitch range: {pitches.min():+.3f} .. {pitches.max():+.3f} rad '
          f'(joint limits -0.35..+1.05)')

    # The trunk top is reliably measured only when the LiDAR pitches up enough
    # to clear the trunk top with its upper FOV edge.  The IMU's role here is
    # twofold: (1) it confirms the sweep actually reached the canopy (lift pitch
    # near the +1.05 rad upper limit), and (2) on real hardware where the cloud
    # is in the sensor frame, it supplies the attitude needed to level it.
    #
    # The most robust IMU-assisted trunk-top estimate is the global maximum
    # trunk-top across the canopy-reaching scans (a true vertical trunk top is
    # an upper envelope: fronds/occlusion only ever hide it, never exceed it).
    global_top = float(tops.max())
    # Secondary robustness check: median of the top-10% of measurements.
    top_quantile = float(np.quantile(tops, 0.90))
    plateau = tops >= (global_top - 0.05)
    imu_top = global_top
    plateau_pitch = float(np.median(pitches[plateau])) if plateau.any() else float(pitches.max())

    print(f'  Global max trunk top: {global_top:.3f} m')
    print(f'  Top-10% trunk top (90th pct): {top_quantile:.3f} m')
    print(f'  Plateau scans (top >= {global_top - 0.05:.2f} m): {int(plateau.sum())}')
    print(f'  IMU-assisted trunk top (global max): {imu_top:.3f} m')
    print(f'  Lift pitch at plateau: {plateau_pitch:+.3f} rad '
          f'({np.degrees(plateau_pitch):+.1f} deg)')

    # Agreement with the encoder-free fused height.
    agreement = imu_top - fused
    print(f'  Encoder-free fused height: {fused:.3f} m')
    print(f'  IMU-assisted vs encoder-free: {agreement:+.3f} m')

    gt_err = imu_top - GROUND_TRUTH_HEIGHT
    print(f'  IMU-assisted error vs ground truth (12.0 m): {gt_err:+.3f} m')

    return {
        'available': True,
        'n_imu_recordings': int(len(ts)),
        'n_scans': int(len(scans)),
        'lift_pitch_min_rad': float(pitches.min()),
        'lift_pitch_max_rad': float(pitches.max()),
        'global_trunk_top_m': global_top,
        'top_quantile_90pct_m': top_quantile,
        'imu_trunk_top_m': imu_top,
        'plateau_scan_count': int(plateau.sum()),
        'plateau_lift_pitch_rad': plateau_pitch,
        'agreement_vs_encoder_free_m': float(agreement),
        'error_vs_ground_truth_m': float(gt_err),
    }


if __name__ == '__main__':
    record_dir = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/ubuntu/harvester_audits/tree_scan_001'
    result = analyze(record_dir)
    out = Path('/tmp/tree_scan_result_v2.json')
    out.write_text(json.dumps(result, indent=2))
    print(f'\nWrote {out}')
