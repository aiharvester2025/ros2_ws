"""Unit tests for the live height estimator and distance estimator."""

import numpy as np
import pytest

from harvester_dock import live_height_estimator as lhe
from harvester_dock import distance_estimator as dist_est


def _synthetic_cloud(trunk_x=8.5, trunk_y=0.0, height=12.0,
                     trunk_radius=0.3, n_trunk=2000, n_canopy=500,
                     half_cylinder=True):
    """Build a synthetic world-frame cloud: a vertical trunk cylinder plus a
    canopy annulus above crown base (9.2 m).

    By default the trunk is a HALF-cylinder (only the near face toward the
    sensor at -X, i.e. angles within +-90 deg of the -X direction), matching
    how a single-sided LiDAR actually sees a trunk.  This makes the radius
    correction meaningful (the median X is the near face, not the centre).
    """
    rng = np.random.default_rng(42)
    z_trunk = rng.uniform(0.0, height, n_trunk)
    if half_cylinder:
        # Near face only: the -X half facing the sensor, angle in [pi/2, 3pi/2]
        # (cos(theta) <= 0), so the sensor (at -X) sees this face.
        th = rng.uniform(np.pi / 2, 3 * np.pi / 2, n_trunk)
    else:
        th = rng.uniform(0, 2 * np.pi, n_trunk)
    rr = trunk_radius * np.sqrt(rng.uniform(0, 1, n_trunk))
    tx = trunk_x + rr * np.cos(th)
    ty = trunk_y + rr * np.sin(th)
    trunk = np.column_stack([tx, ty, z_trunk])
    z_can = rng.uniform(9.2, height, n_canopy)
    th2 = rng.uniform(0, 2 * np.pi, n_canopy)
    rc = rng.uniform(0.6, 1.5, n_canopy)
    canopy = np.column_stack([trunk_x + rc * np.cos(th2),
                              trunk_y + rc * np.sin(th2), z_can])
    return np.vstack([trunk, canopy]).astype('<f4')


def test_estimate_height_synthetic():
    cloud = _synthetic_cloud()  # half-cylinder, matching real LiDAR
    est = lhe.estimate_height(cloud)
    assert est.valid
    # Height within ~0.1 m of true.
    assert est.height_m == pytest.approx(12.0, abs=0.15)
    # Radius-corrected axis should recover the true centre (8.5), not the
    # near-face median (~8.2).
    assert est.trunk_axis_xy_m[0] == pytest.approx(8.5, abs=0.2)
    assert est.trunk_axis_xy_m[1] == pytest.approx(0.0, abs=0.2)


def test_radius_correction_recovers_center():
    """The half-cylinder bias is corrected: without the radius term the axis
    would land ~trunk_radius short of the true centre."""
    cloud = _synthetic_cloud(trunk_x=8.5, trunk_radius=0.3)
    est = lhe.estimate_height(cloud, trunk_radius=0.3)
    # centre recovered to within ~0.1 m
    assert est.trunk_axis_xy_m[0] == pytest.approx(8.5, abs=0.1)


def test_estimate_height_empty():
    est = lhe.estimate_height(np.zeros((0, 3), dtype='<f4'))
    assert not est.valid


def test_trunk_top_estimate():
    z = np.array([1.0, 2.0, 11.9, 12.0, 9.0])
    assert lhe.trunk_top_estimate(z, 'max') == pytest.approx(12.0)


def test_distance_estimate_reachable():
    d = dist_est.estimate_distance((8.5, 0.0), base_x_m=0.0, margin_m=0.2)
    # Boom pivot X = 0 + (-1.05 + 0.18) = -0.87, so d_horiz = 8.5 - (-0.87) = 9.37.
    assert d.d_horiz_m == pytest.approx(9.37, abs=0.01)
    # MAX_BOOM_LENGTH + PLATFORM_TAIL = 12.0 + 1.02 = 13.02; 9.37 < 13.02-margin.
    assert d.reachable


def test_distance_estimate_unreachable():
    d = dist_est.estimate_distance((100.0, 0.0), base_x_m=0.0, margin_m=0.2)
    assert not d.reachable
    assert d.needed_advance_m > 0
    assert 'move prime mover' in d.note


# ----------------------------------------------------------------------------
# Crown base (trunk-end) detection
# ----------------------------------------------------------------------------
def test_crown_base_detected_on_synthetic():
    """Synthetic cloud has canopy starting at z=9.2 m; the detector should
    locate the crown base (trunk-end) near 9.2 m and mark it as valid."""
    cloud = _synthetic_cloud()  # canopy at z in [9.2, 12.0]
    est = lhe.estimate_height(cloud, crown_density_threshold=50)
    assert est.trunk_end_valid, f"crown base should be valid, got: {est.trunk_end_reason}"
    # Within one bin (0.25 m) of the true 9.2 m.
    assert est.crown_base_m == pytest.approx(9.2, abs=0.5), (
        f"crown_base_m={est.crown_base_m:.2f} method={est.crown_base_method!r} "
        f"reason={est.trunk_end_reason!r}")
    assert est.crown_base_method == 'density_drop'


def test_crown_base_no_canopy_falls_back():
    """A cloud with NO canopy/fronds (just a clean trunk) should NOT report a
    confident trunk-end — the detector must return trunk_end_valid=False so the
    caller knows the docking height is unsafe."""
    rng = np.random.default_rng(7)
    z_trunk = rng.uniform(0.0, 12.0, 3000)
    th = rng.uniform(np.pi / 2, 3 * np.pi / 2, 3000)
    rr = 0.3 * np.sqrt(rng.uniform(0, 1, 3000))
    cloud = np.column_stack([8.5 + rr * np.cos(th), rr * np.sin(th), z_trunk])
    est = lhe.estimate_height(cloud, crown_density_threshold=50)
    # No canopy annulus points -> crown base detection fails -> trunk_end_valid False.
    assert not est.trunk_end_valid, (
        f"expected invalid trunk-end, got valid with crown_base_m="
        f"{est.crown_base_m:.2f} method={est.crown_base_method!r}")


def test_crown_base_higher_canopy_still_detected():
    """A taller canopy starting at z=10.0 m should be detected at ~10.0 m."""
    rng = np.random.default_rng(11)
    z_trunk = rng.uniform(0.0, 10.0, 2000)
    th = rng.uniform(np.pi / 2, 3 * np.pi / 2, 2000)
    rr = 0.3 * np.sqrt(rng.uniform(0, 1, 2000))
    trunk = np.column_stack([8.5 + rr * np.cos(th), rr * np.sin(th), z_trunk])
    z_can = rng.uniform(10.0, 12.0, 500)
    th2 = rng.uniform(0, 2 * np.pi, 500)
    rc = rng.uniform(0.6, 1.5, 500)
    canopy = np.column_stack([8.5 + rc * np.cos(th2),
                              rc * np.sin(th2), z_can])
    cloud = np.vstack([trunk, canopy]).astype('<f4')
    est = lhe.estimate_height(cloud, crown_density_threshold=50)
    assert est.trunk_end_valid
    assert est.crown_base_m == pytest.approx(10.0, abs=0.5)


def test_crown_base_from_density_unit():
    """Direct unit test of crown_base_from_density with hand-crafted data."""
    # Bare trunk below 9.0 m (sparse), dense canopy at/above 9.0 m.
    canopy_z = np.concatenate([
        np.full(10, 8.0),          # sparse clutter below crown
        np.full(100, 9.0),         # dense canopy onset
        np.full(100, 9.5),
        np.full(100, 10.0),
    ])
    cb, method, reason = lhe.crown_base_from_density(
        canopy_z, bin_height_m=0.25, threshold=50)
    assert method == 'density_drop'
    # First bin crossing 50 is the [9.0, 9.25) bin -> cb == 9.0.
    assert cb == pytest.approx(9.0, abs=0.25), f"got cb={cb} reason={reason}"


def test_crown_base_from_density_empty():
    cb, method, reason = lhe.crown_base_from_density(
        np.array([], dtype=float), threshold=50)
    assert method == 'none'
    assert cb is None


def test_crown_base_threshold_must_match_sparse_sweep():
    """Regression guard: the crown-base density threshold must be low enough to
    fire on a sparse 5-step nod (~130 pts in the crown bin), NOT the offline
    40-step sweep (~28k pts).  A threshold of 5000 fails to detect the crown on
    sparse data; 50 succeeds.  See dock.yaml crown_density_threshold."""
    rng = np.random.default_rng(3)
    # Sparse canopy: bare trunk (a few pts) below 9.0, ~130 pts in the crown bin.
    canopy_z = np.concatenate([
        rng.uniform(6.0, 9.0, 40),      # sparse bare-trunk clutter
        rng.uniform(9.25, 9.5, 130),    # crown onset (sparse-sweep density)
        rng.uniform(9.5, 11.0, 300),    # canopy above
    ])
    # A 5000 threshold must NOT fire (the historical offline calibration).
    _, method_high, _ = lhe.crown_base_from_density(canopy_z, threshold=5000)
    assert method_high == 'none'
    # A 50 threshold must fire at the crown base.
    cb, method_low, _ = lhe.crown_base_from_density(canopy_z, threshold=50)
    assert method_low == 'density_drop'
    assert cb == pytest.approx(9.25, abs=0.25)


def test_crown_base_ignores_low_z_harvester_clutter():
    """Regression guard: during a LIVE sweep the harvester arm/body self-occlusion
    puts DENSE clutter in the canopy annulus at low z (~2 m).  Without a z_min
    bound the density detector reads that clutter as the crown base (the 2.0 m
    bug).  With a FIXED z_min (e.g. 5.0 m, above clutter but below the ~9 m crown
    base) the detector correctly finds the crown base near the trunk top."""
    rng = np.random.default_rng(21)
    # Dense low-z clutter (harvester arm) + sparse crown at ~9.2 m.
    canopy_z = np.concatenate([
        rng.uniform(1.5, 2.5, 800),     # dense harvester clutter at z~2
        rng.uniform(8.0, 9.0, 20),      # bare trunk
        rng.uniform(9.25, 9.5, 150),    # crown onset
        rng.uniform(9.5, 11.5, 400),    # canopy
    ])
    # Without z_min: the dense z~2 clutter is picked as the crown base.
    cb_bad, method_bad, _ = lhe.crown_base_from_density(canopy_z, threshold=50, z_min=0.0)
    assert method_bad == 'density_drop'
    assert cb_bad < 3.0, "without z_min, low-z clutter is misread as crown base"
    # With z_min=6.0: the clutter is excluded, crown base found near 9.25 m.
    cb, method, _ = lhe.crown_base_from_density(canopy_z, threshold=50, z_min=6.0)
    assert method == 'density_drop'
    assert cb == pytest.approx(9.25, abs=0.25)
