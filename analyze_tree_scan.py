#!/usr/bin/env python3
"""Analyze recorded LiDAR tree scan to estimate tree height.

Reads the canonical v1/lidar/raw recordings produced by the telemetry gateway,
merges all world-frame points, isolates the trunk cylinder, and estimates the
trunk-to-crown transition and total tree height.

The tree is static at world (8.5, 0, 0) with trunk radius ~0.3 m, crown base at
z=9.2 m, and trunk top at z=12.0 m (ground truth for validation).
"""
import sys
import json
from pathlib import Path

import msgpack
import numpy as np

from harvester_telemetry_contract import unpack_message


# Static world pose of the oil-palm tree and its trunk geometry.
TREE_X = 8.5
TREE_Y = 0.0
TRUNK_RADIUS = 0.45     # slightly generous to capture the trunk surface
GROUND_TRUTH_HEIGHT = 12.0
GROUND_TRUTH_CROWN_BASE = 9.2


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
        if point_count <= 0 or len(payload) != point_count * 12:
            skipped += 1
            continue
        xyz = np.frombuffer(payload, dtype='<f4').reshape(point_count, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        clouds.append(xyz)
        kept += 1

    return np.vstack(clouds), kept, skipped


def analyze(record_dir):
    merged, kept, skipped = load_merged_cloud(record_dir)
    print(f'Merged {len(merged)} points from {kept} world-frame recordings '
          f'(skipped {skipped} non-world frames)')

    x, y, z = merged[:, 0], merged[:, 1], merged[:, 2]
    # Horizontal distance from the known tree axis.
    r = np.hypot(x - TREE_X, y - TREE_Y)

    trunk_mask = r < TRUNK_RADIUS
    trunk_z = z[trunk_mask]
    canopy_z = z[(r >= TRUNK_RADIUS) & (r < 2.0)]

    print(f'\nTrunk points (r<{TRUNK_RADIUS:.2f} m): {trunk_mask.sum()}')
    print(f'Canopy points (r {TRUNK_RADIUS:.2f}..2.0 m): {canopy_z.size}')

    # Ground level = lowest trunk points (5th percentile to reject stray lows).
    # The LiDAR's downward view is occluded near the base, so this reads ~1.8 m
    # rather than 0; report it, but use the known world z=0 for total height.
    ground_z = np.percentile(trunk_z, 5)

    # Canopy top = 99th percentile of canopy-zone points (robust to frond tips).
    canopy_top = np.percentile(canopy_z, 99)
    canopy_top_max = canopy_z.max()

    # Trunk-to-crown transition: the canopy annulus (r in 0.45..2.0 m) is empty
    # below the crown base and populated above it (fronds/branches/FFBs).  Find
    # the lowest height where the canopy annulus becomes persistently populated.
    bins = np.arange(0.0, 14.0, 0.25)
    hist, edges = np.histogram(canopy_z, bins=bins)
    # The canopy annulus is nearly empty along the bare trunk (harvester/ground
    # clutter stays below ~2500 points per 0.25 m) and jumps to tens of
    # thousands at the crown.  A 5000-point threshold separates the two.
    canopy_threshold = 5000
    crown_base = None
    for i, count in enumerate(hist):
        if count >= canopy_threshold:
            crown_base = edges[i]
            break

    # Total height is measured from the known tree base at world z=0 (the
    # static world->tree_base transform), not the occluded LiDAR "ground".
    total_height = canopy_top - 0.0

    print('\n=== Results ===')
    print(f'Ground level (5th pct trunk z): {ground_z:.2f} m (occluded; true base = 0)')
    print(f'Crown base (density drop):      {crown_base:.2f} m' if crown_base else 'Crown base: N/A')
    print(f'Canopy top (99th pct):          {canopy_top:.2f} m')
    print(f'Canopy top (max frond tip):     {canopy_top_max:.2f} m')
    print(f'Estimated total tree height:    {total_height:.2f} m')

    print('\n=== Ground-truth comparison ===')
    print(f'Ground truth height:     {GROUND_TRUTH_HEIGHT} m')
    print(f'Ground truth crown base: {GROUND_TRUTH_CROWN_BASE} m')
    err = total_height - GROUND_TRUTH_HEIGHT
    print(f'Total height error:      {err:+.2f} m ({err / GROUND_TRUTH_HEIGHT * 100:+.1f}%)')

    result = {
        'total_points': int(len(merged)),
        'n_world_recordings': kept,
        'n_skipped_recordings': skipped,
        'trunk_points': int(trunk_mask.sum()),
        'ground_z_m': float(ground_z),
        'crown_base_m': float(crown_base) if crown_base is not None else None,
        'canopy_top_99pct_m': float(canopy_top),
        'canopy_top_max_m': float(canopy_top_max),
        'total_tree_height_m': float(total_height),
        'ground_truth_height_m': GROUND_TRUTH_HEIGHT,
        'ground_truth_crown_base_m': GROUND_TRUTH_CROWN_BASE,
        'height_error_m': float(err),
    }
    return result


if __name__ == '__main__':
    record_dir = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/ubuntu/harvester_audits/tree_scan_001'
    result = analyze(record_dir)
    out = Path('/tmp/tree_scan_result.json')
    out.write_text(json.dumps(result, indent=2))
    print(f'\nWrote {out}')
