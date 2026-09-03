"""Live tree-height estimation (ROS-free port of analyze_tree_scan_v2.py).

Operates on an in-memory Nx3 float32 array of world-frame LiDAR points
accumulated during a sweep.  Ports the encoder-free strategy from the offline
``analyze_tree_scan_v2.py``:

  - ``fit_trunk_axis``  : median XY of trunk points in a mid-trunk band.
  - ``trunk_top_estimate``: highest point in a tight cylinder about the axis.

This module is ROS-free so it can be unit-tested without a running graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Ground truth for the reference tree (tree_targets.yaml), for parity checks.
GROUND_TRUTH_HEIGHT = 12.0
# Crown base reference (z) = the point where the trunk ends and the fronds/FFBs
# begin.  In the reference URDF this is at z = 9.2 m, and the lowest branch is
# at z = 9.45 m, the lowest FFB at z = 9.55 m.  Used as a sanity-check only;
# the live estimate derives it from the cloud (see crown_base_from_density).
GROUND_TRUTH_CROWN_BASE = 9.2


@dataclass
class HeightEstimate:
    height_m: float
    trunk_axis_xy_m: Tuple[float, float]
    uncertainty_m: float
    n_points: int
    valid: bool = True
    reason: str = ''
    # --- Crown base (trunk-end) ------------------------------------------------
    # The height (world z) at which the trunk's clean vertical cylinder ends and
    # the fronds/FFBs first begin.  This is the height the boom should target,
    # NOT the trunk top minus an offset.  Defaulting to height_m is the legacy
    # "tree_top" behaviour; new code should require this explicitly.
    crown_base_m: float = 0.0
    crown_base_method: str = ''   # 'density_drop' | 'none'
    trunk_end_valid: bool = False
    trunk_end_reason: str = ''


def fit_trunk_axis(x: np.ndarray, y: np.ndarray, z: np.ndarray,
                   z_lo: float = 1.0, z_hi: float = 8.0,
                   y_max: float = 1.0, min_points: int = 50,
                   trunk_radius: float = 0.25
                   ) -> Optional[Tuple[float, float]]:
    """Estimate the trunk axis (X, Y) centreline from a mid-trunk band.

    The band z in [z_lo, z_hi] is below the crown and above the occluded base;
    ``y_max`` drops fronds/FFB clutter that extends sideways.  A single-sided
    (half-cylinder) scan biases the median X toward the sensor, so the near-face
    median is corrected by ``trunk_radius`` (the trunk's nominal radius, ~0.25 m
    at the top) to recover the true centreline (verified against tree_scan_002:
    median 8.28 -> corrected 8.53 vs true 8.5).
    """
    band = (z >= z_lo) & (z <= z_hi) & (np.abs(y) <= y_max)
    if band.sum() < min_points:
        return None
    med_x = float(np.median(x[band]))
    med_y = float(np.median(y[band]))
    # Half-cylinder bias: the median X of a single-sided scan of a cylinder of
    # radius r sits at centre - r*(2/pi) (the mean of cos over the visible half),
    # so the centreline is recovered by adding r*(2/pi).  Verified against
    # tree_scan_002: median 8.283 + 0.637*0.35 = 8.506 (true 8.5).
    return med_x + (2.0 / math.pi) * trunk_radius, med_y


def trunk_top_estimate(z_trunk: np.ndarray, method: str = 'max') -> Optional[float]:
    """Highest point in a trunk cylinder; None if too few points.

    ``max`` is the correct estimator for the trunk top: within a cylinder about
    the axis, fronds/occlusion can only hide the top, never exceed it, so the
    maximum is an upper envelope (percentiles are dragged down by the bulk of
    lower trunk points and give a biased-low top).
    """
    if len(z_trunk) < 5:
        return None
    if method == 'max':
        return float(z_trunk.max())
    if method == 'p999':
        return float(np.percentile(z_trunk, 99.9))
    raise ValueError(method)


def crown_base_from_density(canopy_z: np.ndarray,
                            bin_height_m: float = 0.25,
                            z_hi_cap: float = 14.0,
                            z_min: float = 0.0,
                            threshold: int = 5000) -> Tuple[Optional[float], str, str]:
    """Detect the trunk-end (crown base) from the canopy-annulus density jump.

    This is the SAME proven method as ``analyze_tree_scan_v2.py`` /
    ``analyze_tree_scan.py`` (validated on tree_scan_001/002: crown base
    9.00-9.25 m vs ground truth 9.2 m).

    The canopy annulus (radial distance between the trunk cylinder and ~2 m)
    is nearly empty along the bare trunk: only sparse harvester/ground clutter
    appears.  At the crown base the fronds/FFBs populate that annulus densely,
    so a per-height-bin histogram of ``canopy_z`` shows a sharp jump from
    < threshold to >> threshold points per bin.  The crown base is the bottom
    of the first bin that crosses ``threshold``.

    Parameters
    ----------
    canopy_z      : world z of points in the canopy annulus (r in [top_radius, 2.0]).
    bin_height_m  : histogram bin height (0.25 m matches tree_targets geometry).
    z_hi_cap      : histogram upper bound (default 14 m, above the 12 m top).
    z_min         : LOWEST height to scan.  The crown base is near the trunk top,
                    never at the base; the harvester arm/body self-occlusion puts
                    dense clutter in the annulus at low z (~2 m) during a live
                    sweep, which would otherwise be misread as the crown base.
                    Callers should pass a FIXED bound well above the harvester
                    (e.g. 4-5 m) but well below the crown base (~9 m).  A bound
                    tied to the trunk top (e.g. top-3.5) clips into the frond
                    zone and biases the crown base low.
    threshold     : points-per-bin density threshold.  For the dense Mid-360
                    sweep this is ~5000; a sparse sweep needs a lower value.

    Returns
    -------
    (crown_base_m, method, reason) where method is:
        'density_drop' : confident detection.
        'none'         : no bin crossed the threshold (insufficient density).
    """
    if len(canopy_z) == 0:
        return None, 'none', 'no canopy-annulus points'
    canopy_z = canopy_z[canopy_z >= z_min]
    if len(canopy_z) == 0:
        return None, 'none', f'no canopy-annulus points above z_min={z_min:.2f}'
    bins = np.arange(z_min, z_hi_cap + bin_height_m, bin_height_m)
    hist, edges = np.histogram(canopy_z, bins=bins)
    for i, count in enumerate(hist):
        if count >= threshold:
            return float(edges[i]), 'density_drop', (
                f'{count} canopy-annulus points in z=[{edges[i]:.2f},'
                f'{edges[i+1]:.2f}] >= threshold {threshold}')
    return None, 'none', (
        f'max canopy-annulus bin = {int(hist.max())} < threshold {threshold} '
        f'(scan too sparse or too low, z_min={z_min:.2f})')


def estimate_height(points: np.ndarray,
                    axis_band: Tuple[float, float] = (1.0, 8.0),
                    axis_y_max: float = 2.0,
                    trunk_radius: float = 0.25,
                    top_radius: float = 0.35,
                    min_points: int = 30,
                    world_frame: bool = True,
                    crown_bin_height_m: float = 0.25,
                    crown_density_threshold: int = 5000,
                    crown_z_min_m: Optional[float] = None) -> HeightEstimate:
    """Estimate tree height + trunk axis + crown base from a merged cloud.

    ``points`` is an Nx3 float32 array (x, y, z) of a level (vertical-tree)
    cloud.  When ``world_frame`` is true the tree base is at world z=0 and
    height == trunk-top z.  When false (sensor-origin, rotation-only leveled),
    the ground sits at some negative z; height is then ``top_z - base_z``
    (translation-invariant), and the returned axis is sensor-relative.

    Refinements over a naive median+max:
      * the trunk axis is radius-corrected (half-cylinder bias),
      * fronds are dropped with ``axis_y_max``,
      * the top uses a wider ``top_radius`` (a tapered trunk's top is wider
        relative to the axis) and the ``max`` upper envelope,
      * the crown base (trunk-end) is detected from the canopy-annulus density
        transition (same proven method as ``analyze_tree_scan_v2.py``): the
        annulus r in [``top_radius``, 2.0] is nearly empty on the bare trunk
        and densely populated above the crown base.  The bottom of the first
        histogram bin crossing ``crown_density_threshold`` is the crown base.

    Crown-base detector parameters:
      * ``crown_bin_height_m`` (default 0.25 m): the fronds in tree_targets start
        0.25 m above the crown base, so 0.25 m is the natural bin resolution.
      * ``crown_density_threshold`` (default 5000 points/bin): the density
        threshold separating bare-trunk clutter from canopy.  Lower it for a
        sparse sweep.
    """
    if points is None or len(points) == 0:
        return HeightEstimate(0.0, (0.0, 0.0), 0.0, 0,
                              valid=False, reason='no points')

    pts = points[np.isfinite(points).all(axis=1)]
    if len(pts) == 0:
        return HeightEstimate(0.0, (0.0, 0.0), 0.0, 0,
                              valid=False, reason='all points non-finite')

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    # In the sensor-origin (non-world) frame the ground is below z=0; use a
    # band anchored to the cloud rather than the fixed world [1, 8].
    if world_frame:
        z_lo, z_hi = axis_band
    else:
        z_lo, z_hi = float(z.min()) + 0.5, float(z.min()) + 8.0

    axis = fit_trunk_axis(x, y, z, z_lo=z_lo, z_hi=z_hi,
                          y_max=axis_y_max, min_points=min_points,
                          trunk_radius=trunk_radius)
    if axis is None:
        ax, ay = float(np.median(x)), float(np.median(y))
        valid = False
        reason = 'trunk-axis fit failed (using median XY, no radius correction)'
    else:
        ax, ay = axis
        valid = True
        reason = ''

    r = np.hypot(x - ax, y - ay)
    trunk = z[r < top_radius]
    top = trunk_top_estimate(trunk, method='max')

    if top is None:
        return HeightEstimate(0.0, (ax, ay), 0.0, len(pts),
                              valid=False, reason='no trunk cylinder points')

    if world_frame:
        height = top
    else:
        # Sensor-origin: height = top_z - base_z (translation-invariant).
        base = float(trunk.min())
        height = top - base

    # --- Crown base (trunk-end) ---------------------------------------------
    # The crown base is the height at which the trunk's clean vertical cylinder
    # ends and fronds/FFBs first appear.  This is the height the boom should
    # target for docking (NOT the trunk top minus an offset, which lands inside
    # the canopy/frond/FFB zone and is why the platform crashed into them).
    #
    # Density method (proven on tree_scan_001/002): the canopy annulus
    # (r in [top_radius, 2.0]) is nearly empty on the bare trunk and densely
    # populated above the crown base.  The first histogram bin that crosses
    # ``crown_density_threshold`` is the crown base.
    #
    # Scan only ABOVE a FIXED lower bound ``crown_z_min_m``: the harvester
    # arm/body self-occlusion puts dense clutter in the annulus at low z (~2 m)
    # during a live sweep, which would otherwise be misread as the crown base.
    # The bound must be well above the harvester (~3 m) but well below the true
    # crown base (~9.2 m) so the real density transition is still found.  A
    # value tied to ``top`` (e.g. top-3.5) clips into the frond zone and biases
    # the crown base low (observed: crown_base == top-3.5 exactly).
    canopy_z = z[(r >= top_radius) & (r < 2.0)]
    if crown_z_min_m is None:
        crown_z_min_m = 5.0
    crown_base, cb_method, cb_reason = crown_base_from_density(
        canopy_z,
        bin_height_m=crown_bin_height_m,
        z_min=crown_z_min_m,
        threshold=crown_density_threshold,
    )
    if crown_base is None:
        # No confident trunk-end: fall back to a SAFE default of
        # (trunk_top - 2.0) and mark trunk_end_valid=False so callers know
        # NOT to dock on the fallback.
        crown_base = max(0.0, height - 2.0)
        trunk_end_valid = False
        trunk_end_reason = (
            f'crown-base detector failed ({cb_reason}); '
            f'unsafe default trunk_top-2.0 = {crown_base:.2f} m '
            '(NOT recommended for docking)')
    else:
        trunk_end_valid = True
        trunk_end_reason = cb_reason

    uncertainty = 0.0

    return HeightEstimate(
        height, (ax, ay), uncertainty, len(pts),
        valid=valid, reason=reason,
        crown_base_m=float(crown_base),
        crown_base_method=cb_method,
        trunk_end_valid=bool(trunk_end_valid),
        trunk_end_reason=trunk_end_reason,
    )


__all__ = ['HeightEstimate', 'estimate_height', 'crown_base_from_density',
           'fit_trunk_axis', 'trunk_top_estimate',
           'GROUND_TRUTH_HEIGHT', 'GROUND_TRUTH_CROWN_BASE']
