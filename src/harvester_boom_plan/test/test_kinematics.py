"""Unit tests for the boom inverse kinematics module."""

import math
from xml.etree import ElementTree as ET

import pytest

from harvester_boom_plan import kinematics as kin


def _read_urdf():
    """Return the parsed active URDF, if available on this machine."""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        pytest.skip("ament_index_python not on PYTHONPATH (not a ROS shell)")
    from pathlib import Path
    share = Path(get_package_share_directory("oil_palm_harvester_description"))
    path = share / "urdf" / "oil_palm_harvester_kinematic.urdf"
    if not path.exists():
        pytest.skip("active URDF not installed")
    return ET.parse(str(path)).getroot()


def _joint_origin(root, joint_name):
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            o = joint.find("origin")
            return [float(v) for v in o.get("xyz").split()]
    return None


def _joint_limit(root, joint_name):
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            l = joint.find("limit")
            return [float(l.get("lower")), float(l.get("upper"))]
    return None


def test_urdf_constants_consistent():
    root = _read_urdf()

    turret = _joint_origin(root, "boom_turret_joint")
    elev = _joint_origin(root, "boom_elevation_joint")
    level = _joint_origin(root, "platform_level_joint")
    plat = _joint_origin(root, "platform_fixed_joint")
    ext = _joint_limit(root, "boom_extension_1_joint")
    elev_lim = _joint_limit(root, "boom_elevation_joint")
    level_lim = _joint_limit(root, "platform_level_joint")

    assert tuple(turret) == pytest.approx(kin.TURRET_ORIGIN)
    assert tuple(elev) == pytest.approx(kin.ELEVATION_ORIGIN)
    assert kin.BOOM_PIVOT_HEIGHT == pytest.approx(turret[2] + elev[2])
    assert kin.BOOM_FIXED_LENGTH == pytest.approx(level[0])
    assert kin.PLATFORM_TAIL == pytest.approx(plat[0])
    assert kin.EXTENSION_PER_STAGE == pytest.approx(ext[1] - ext[0])
    assert kin.ELEVATION_LIMIT == pytest.approx(tuple(elev_lim))
    assert kin.LEVEL_LIMIT == pytest.approx(tuple(level_lim))
    assert kin.MAX_STROKE == pytest.approx(kin.EXTENSION_PER_STAGE * kin.STAGE_COUNT)
    assert kin.MAX_BOOM_LENGTH == pytest.approx(kin.BOOM_FIXED_LENGTH + kin.MAX_STROKE)


def test_forward_kinematics_recovers_dock_point():
    # Trunk at world (8.5, 0), docking height 10.0, harvester base X=4.0.
    base_x = 4.0
    pivot_x, _ = kin.boom_pivot_world_xy(base_x, 0.0)
    d_horiz = 8.5 - pivot_x
    h_dock = 10.0

    ik = kin.inverse_kinematics(d_horiz, h_dock, "active")
    x, z = kin.forward_kinematics(ik.theta_b_rad, ik.leveling_angle_rad,
                                  ik.extension_total_m, pivot_x, kin.BOOM_PIVOT_WORLD_Z)
    assert x == pytest.approx(8.5, abs=1e-6)
    assert z == pytest.approx(10.0, abs=1e-6)


@pytest.mark.parametrize("base_x,theta_b_deg,e_m", [
    (0.0, 44.45, 9.296),
    (4.0, 62.03, 6.874),
    (6.0, 73.99, 6.120),
])
def test_ik_values(base_x, theta_b_deg, e_m):
    pivot_x, _ = kin.boom_pivot_world_xy(base_x, 0.0)
    d_horiz = 8.5 - pivot_x
    ik = kin.inverse_kinematics(d_horiz, 10.0, "active")
    assert math.degrees(ik.theta_b_rad) == pytest.approx(theta_b_deg, abs=0.05)
    assert ik.extension_total_m == pytest.approx(e_m, abs=0.01)
    assert ik.feasible


def test_ik_respects_elevation_limit():
    # Move the tree so far that the required elevation exceeds 75 deg.
    pivot_x, _ = kin.boom_pivot_world_xy(8.0, 0.0)
    # Tree nearly overhead.
    d_horiz = 0.2
    ik = kin.inverse_kinematics(d_horiz, 10.0, "active")
    assert not ik.feasible
    assert "theta_b" in (ik.infeasible_reason or "")


def test_infeasible_dock_raises():
    pivot_x, _ = kin.boom_pivot_world_xy(-10.0, 0.0)  # very far base
    d_horiz = 8.5 - pivot_x  # large
    with pytest.raises(kin.InfeasibleDock):
        kin.inverse_kinematics(d_horiz, 10.0, "active")


def test_split_extension_even():
    stages = kin.split_extension(6.0)
    assert len(stages) == 4
    assert sum(stages) == pytest.approx(6.0)
    assert all(abs(s - 1.5) < 1e-9 for s in stages)


def test_split_extension_clamps():
    stages = kin.split_extension(100.0)
    assert sum(stages) == pytest.approx(kin.MAX_STROKE)


def test_leveling_active_vs_passive():
    pivot_x, _ = kin.boom_pivot_world_xy(4.0, 0.0)
    d_horiz = 8.5 - pivot_x
    active = kin.inverse_kinematics(d_horiz, 10.0, "active")
    passive = kin.inverse_kinematics(d_horiz, 10.0, "passive")
    # Leveling axis (0,+1,0) is opposite the boom elevation axis (0,-1,0), so
    # the level joint must be commanded +theta_b to counter-rotate to horizontal.
    assert active.leveling_angle_rad == pytest.approx(active.theta_b_rad)
    assert passive.leveling_angle_rad == pytest.approx(0.0)
    # Active-leveled platform world angle == 0 (horizontal).
    assert active.platform_level_rad == pytest.approx(0.0, abs=1e-9)


def test_docking_lower_angle_non_negative():
    theta_d = kin.docking_lower_angle(0.5, 10.0)
    assert theta_d >= 0.0


def test_docking_lower_angle_zero_at_solved_config():
    # At the solved IK configuration the platform is already at H_dock, so the
    # lower angle must be ~0 (not a spurious large angle from ignoring extension).
    pivot_x, _ = kin.boom_pivot_world_xy(4.0, 0.0)
    d_horiz = 8.5 - pivot_x
    ik = kin.inverse_kinematics(d_horiz, 10.0, "active")
    theta_d = kin.docking_lower_angle(ik.theta_b_rad, 10.0, ik.extension_total_m)
    assert theta_d == pytest.approx(0.0, abs=1e-6)
