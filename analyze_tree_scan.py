#!/usr/bin/env python3
"""Analyze recorded LiDAR tree scan to estimate tree height.

Reads the canonical v1/lidar/raw recordings produced by the telemetry gateway,
merges all world-frame points, and estimates the trunk top (transition to
canopy) and total tree height.
"""
import sys
import json
import struct
from pathlib import Path

import msgpack
import numpy as np

from harvester_telemetry_contract import unpack_message


def load_merged_cloud(record_dir, frame='world'):
    """Return an Nx3 float32 numpy array of merged XYZ points."""
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
        expected = point_count * 12
        if len(payload) != expected:
            skipped += 1
            continue
        if point_count <= 0:
            skipped += 1
            continue
        xyz = np.frombuffer(payload, dtype='<f4').reshape(point_count, 3)
        # Drop non-finite and clearly-invalid points.
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        clouds.append(xyz)
        kept += 1

    merged = np.vstack(clouds)
    return merged, kept, skipped


def analyze(record_dir):
    merged, kept, skipped = load_merged_cloud(record_dir)
    print(f'Merged {len(merged)} points from {kept} world-frame recordings '
          f'(skipped {skipped} non-world frames)')

    x = merged[:, 0]
    y = merged[:, 1]
    z = merged[:, 2]

    print('\n=== XYZ bounds (world-aligned frame) ===')
    for name, arr in (('X', x), ('Y', y), ('Z', z)):
        print(f'{name}: min={arr.min():.3f}  max={arr.max():.3f}  '
              f'mean={arr.mean():.3f}  std={arr.std():.3f}')

    # The tree is a vertical cylinder.  In the leveled frame the LiDAR is at
    # origin; the trunk is a tall narrow column of points at roughly constant
    # horizontal distance.  Histogram Z to find the trunk extent.
    z_bins = np.arange(np.floor(z.min()), np.ceil(z.max()) + 0.1, 0.1)
    hist, edges = np.histogram(z, bins=z_bins)
    bin_centers = (edges[:-1] + edges[1:]) / 2

    # Find contiguous occupied vertical extent.
    occupied = hist > 0
    print('\n=== Z histogram (0.1 m bins) ===')
    for center, count in zip(bin_centers, hist):
        if count > 0:
            bar = '#' * min(50, count // 20)
            print(f'z={center:7.2f}  n={count:5d}  {bar}')

    # Estimate trunk: points within a horizontal radius of the trunk axis.
    # In the leveled (rotation-only) frame, the trunk axis is roughly vertical
    # through the centroid of lower trunk points.
    # First find ground-level trunk centroid using lowest 2 m of points.
    z_min = z.min()
    lower_mask = z < (z_min + 2.0)
    if lower_mask.sum() < 10:
        lower_mask = z < (z_min + 4.0)

    lower_x = x[lower_mask]
    lower_y = y[lower_mask]
    trunk_cx = np.median(lower_x)
    trunk_cy = np.median(lower_y)
    print(f'\nLower-trunk centroid (median): x={trunk_cx:.3f}, y={trunk_cy:.3f}')

    # Horizontal distance from trunk axis for all points.
    dist = np.hypot(x - trunk_cx, y - trunk_cy)

    # Trunk radius ~0.3 m; canopy extends much wider.  Points within 0.6 m of
    # the axis are trunk; beyond that are canopy/branches.
    trunk_mask = dist < 0.6
    trunk_z = z[trunk_mask]
    canopy_z = z[~trunk_mask]

    print(f'Trunk points (r<0.6m): {trunk_mask.sum()}')
    print(f'Canopy points (r>=0.6m): {(~trunk_mask).sum()}')

    if trunk_mask.sum() > 0:
        print(f'Trunk Z range: {trunk_z.min():.3f} to {trunk_z.max():.3f}')
        # The top of the trunk is where trunk points end.  Use a percentile to
        # avoid outliers from a few stray canopy points near the axis.
        trunk_top = np.percentile(trunk_z, 95)
        trunk_bottom = np.percentile(trunk_z, 5)
        print(f'Trunk bottom (5th pct): {trunk_bottom:.3f}')
        print(f'Trunk top (95th pct):   {trunk_top:.3f}')
        estimated_height = trunk_top - trunk_bottom
        print(f'Estimated trunk height: {estimated_height:.3f} m')
    else:
        estimated_height = None
        trunk_top = trunk_bottom = None

    if canopy_z.size > 0:
        print(f'Canopy Z range: {canopy_z.min():.3f} to {canopy_z.max():.3f}')
        canopy_top = canopy_z.max()
        print(f'Total tree height (canopy top - trunk bottom): '
              f'{canopy_top - z_min:.3f} m' if trunk_bottom is None else
              f'Total tree height: {canopy_top - trunk_bottom:.3f} m')

    # Ground-truth comparison
    ground_truth = 12.0
    print(f'\n=== Comparison ===')
    print(f'Ground truth trunk height: {ground_truth} m')
    if estimated_height is not None:
        err = estimated_height - ground_truth
        print(f'Estimated trunk height:   {estimated_height:.3f} m')
        print(f'Error: {err:+.3f} m ({err/ground_truth*100:+.1f}%)')

    result = {
        'n_points_total': int(len(merged)),
        'n_world_recordings': kept,
        'n_skipped_recordings': skipped,
        'trunk_axis_xy': [float(trunk_cx), float(trunk_cy)],
        'trunk_bottom_m': float(trunk_bottom) if trunk_bottom is not None else None,
        'trunk_top_m': float(trunk_top) if trunk_top is not None else None,
        'estimated_trunk_height_m': float(estimated_height) if estimated_height is not None else None,
        'ground_truth_height_m': ground_truth,
    }
    return result


if __name__ == '__main__':
    record_dir = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/ubuntu/harvester_audits/tree_scan_001'
    result = analyze(record_dir)
    out = Path('/tmp/tree_scan_result.json')
    out.write_text(json.dumps(result, indent=2))
    print(f'\nWrote {out}')
