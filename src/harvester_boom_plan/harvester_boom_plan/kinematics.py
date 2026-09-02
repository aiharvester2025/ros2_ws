"""Closed-form boom inverse kinematics, derived from the active URDF.

The boom is a planar 2-link system: a rigid telescopic boom (length
``L_fixed + e`` where ``e`` is the summed prismatic stroke) with elevation
``theta_b``, carrying a c-channel platform at its tip via a level joint.

The platform's tail offset ``L_plat`` is the fixed joint that carries
``c_channel_reference`` (the C-opening) out from the level joint.

Frames and values follow ``urdf/oil_palm_harvester_kinematic.urdf``:

    boom_turret_joint     origin (-1.05, 0, 1.02) in base_link
    boom_elevation_joint  origin ( 0.18, 0, 0.74) in boom_turret_link
    platform_level_joint  origin ( 2.40, 0, 0   ) in boom_stage_4_link
    platform_fixed_joint  origin ( 1.02, 0, 0   ) in platform_mount_link
    boom_extension_1..4_joint  prismatic X, 0..2.4 m each

This module is ROS-free so it can be unit-tested without a running graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional
from xml.etree import ElementTree as ET


def _urdf_joint_origin(urdf_path: str, joint_name: str) -> Optional[List[float]]:
    """Return the [x, y, z] origin of a named joint, or None if absent."""
    try:
        root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError):
        return None
    for joint in root.findall("joint"):
        if joint.get("name") != joint_name:
            continue
        origin = joint.find("origin")
        if origin is None:
            continue
        xyz = origin.get("xyz")
        if not xyz:
            continue
        return [float(v) for v in xyz.split()]
    return None


def _urdf_joint_limit(urdf_path: str, joint_name: str) -> Optional[List[float]]:
    """Return the [lower, upper] limits of a named revolute/prismatic joint."""
    try:
        root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError):
        return None
    for joint in root.findall("joint"):
        if joint.get("name") != joint_name:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        lower = limit.get("lower")
        upper = limit.get("upper")
        if lower is None or upper is None:
            continue
        return [float(lower), float(upper)]
    return None


def _resolve_urdf_path() -> Optional[str]:
    """Locate the active kinematic URDF, or None when it is not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        from pathlib import Path
        share = Path(get_package_share_directory("oil_palm_harvester_description"))
        path = share / "urdf" / "oil_palm_harvester_kinematic.urdf"
        return str(path) if path.exists() else None
    except Exception:
        return None


_URDF_PATH = _resolve_urdf_path()


# --- URDF-derived constants --------------------------------------------------
# These are the kinematic geometry used by the closed-form IK.  They are
# resolved from the active URDF at import time when the URDF is available;
# otherwise they fall back to the documented literals (which match the current
# URDF).  ``test_urdf_constants_consistent`` asserts equality against the URDF
# so a model tuning change is caught rather than silently drifting.
def _origin_or(default: List[float], joint: str) -> List[float]:
    if _URDF_PATH is None:
        return default
    got = _urdf_joint_origin(_URDF_PATH, joint)
    return got if got is not None else default


def _limit_or(default: List[float], joint: str) -> List[float]:
    if _URDF_PATH is None:
        return default
    got = _urdf_joint_limit(_URDF_PATH, joint)
    return got if got is not None else default


_TURRET_ORIGIN = _origin_or([-1.05, 0.0, 1.02], "boom_turret_joint")
_ELEVATION_ORIGIN = _origin_or([0.18, 0.0, 0.74], "boom_elevation_joint")
_LEVEL_ORIGIN = _origin_or([2.40, 0.0, 0.0], "platform_level_joint")
_PLATFORM_ORIGIN = _origin_or([1.02, 0.0, 0.0], "platform_fixed_joint")
_EXT_STAGE_LIMIT = _limit_or([0.0, 2.4], "boom_extension_1_joint")
_ELEV_LIMIT = _limit_or([0.0, 1.309], "boom_elevation_joint")
_LEVEL_LIMIT = _limit_or([-1.57, 1.57], "platform_level_joint")
_TURRET_LIMIT = _limit_or([-0.35, 0.35], "boom_turret_joint")

TURRET_ORIGIN = tuple(_TURRET_ORIGIN)
ELEVATION_ORIGIN = tuple(_ELEVATION_ORIGIN)
BOOM_PIVOT_HEIGHT = TURRET_ORIGIN[2] + ELEVATION_ORIGIN[2]
BOOM_FIXED_LENGTH = _LEVEL_ORIGIN[0]
PLATFORM_TAIL = _PLATFORM_ORIGIN[0]
EXTENSION_PER_STAGE = _EXT_STAGE_LIMIT[1] - _EXT_STAGE_LIMIT[0]
STAGE_COUNT = 4
MAX_STROKE = EXTENSION_PER_STAGE * STAGE_COUNT
MAX_BOOM_LENGTH = BOOM_FIXED_LENGTH + MAX_STROKE

# The Gazebo kinematic base controller parks base_link at world z = 0.05 m
# (see gazebo_base_kinematic_controller.py).  Docking heights are expressed in
# the world frame, so the boom pivot's world Z is the base offset plus the
# base_link-frame pivot height.
BASE_LINK_Z_WORLD = 0.05
BOOM_PIVOT_WORLD_Z = BASE_LINK_Z_WORLD + BOOM_PIVOT_HEIGHT

ELEVATION_LIMIT = tuple(_ELEV_LIMIT)
LEVEL_LIMIT = tuple(_LEVEL_LIMIT)
TURRET_LIMIT = tuple(_TURRET_LIMIT)


class InfeasibleDock(Exception):
    """Raised when the docking point is unreachable from the current base pose."""

    def __init__(self, needed_length_m: float, deficit_m: float,
                 recommended_advance_m: float, message: str = ""):
        super().__init__(message or (
            f"Docking point unreachable: required boom length "
            f"{needed_length_m:.3f} m exceeds max {MAX_BOOM_LENGTH:.3f} m "
            f"(deficit {deficit_m:.3f} m). Advance the harvester base at least "
            f"{recommended_advance_m:.3f} m toward the tree."
        ))
        self.needed_length_m = needed_length_m
        self.deficit_m = deficit_m
        self.recommended_advance_m = recommended_advance_m


@dataclass
class IKResult:
    """Joint targets that place c_channel_reference at the docking point."""
    theta_b_rad: float                    # boom elevation from horizontal
    leveling_angle_rad: float             # theta_L applied to platform_level_joint
    platform_level_rad: float             # resulting platform world angle (0 when leveled)
    extension_total_m: float              # e = sum of stage strokes
    boom_horizontal_distance_m: float     # d_b: pivot->docking horizontal distance
    docking_height_m: float               # H_dock above base_link ground plane
    extension_per_stage_m: List[float] = field(default_factory=list)
    feasible: bool = True
    infeasible_reason: Optional[str] = None


def boom_pivot_world_xy(base_x: float, turret_yaw_rad: float = 0.0) -> tuple[float, float]:
    """Boom pivot X/Y in world, given the harvester base X and turret yaw.

    Assumes base at world y=0 and turret yaw small (we align the harvester
    straight at the tree before the maneuver, so yaw ~ 0).
    """
    # Turret origin at base_link x=-1.05, elevation origin at +0.18 in the
    # turret frame.  With yaw==0 this collapses to a single X offset.
    offset_x = TURRET_ORIGIN[0] + ELEVATION_ORIGIN[0]   # -0.87
    return (base_x + offset_x, 0.0)


def forward_kinematics(theta_b_rad: float, leveling_angle_rad: float,
                       extension_total_m: float,
                       pivot_x: float, pivot_z: float) -> tuple[float, float]:
    """Return (x, z) of c_channel_reference in world for a given configuration.

    ``pivot_x``/``pivot_z`` are the boom pivot world coordinates.  The platform
    is assumed leveled (leveling_angle_rad is the *correction* applied to the
    level joint; when the platform is horizontal its body contribution is the
    horizontal ``PLATFORM_TAIL`` offset).
    """
    L = BOOM_FIXED_LENGTH + extension_total_m
    x = pivot_x + L * math.cos(theta_b_rad) + PLATFORM_TAIL
    z = pivot_z + L * math.sin(theta_b_rad)
    return x, z


def inverse_kinematics(d_horiz_m: float, h_dock_m: float,
                       leveling_mode: str = "active") -> IKResult:
    """Solve theta_b and extension e so c_channel_reference lands on the dock.

    Parameters
    ----------
    d_horiz_m : horizontal distance (m) from the *boom pivot* to the trunk centre
        in the world X direction (positive = tree ahead).
    h_dock_m  : docking height above ground (world z of the docking point).

    The platform tail ``PLATFORM_TAIL`` is subtracted from d_horiz before
    solving because the level joint sits that far behind the C-opening.
    """
    dx = d_horiz_m - PLATFORM_TAIL
    dz = h_dock_m - BOOM_PIVOT_WORLD_Z

    L_needed = math.hypot(dx, dz)
    theta_b = math.atan2(dz, dx)

    if L_needed > MAX_BOOM_LENGTH:
        deficit = L_needed - MAX_BOOM_LENGTH
        # How much closer the base must be for the dock to become reachable:
        # reduce d_horiz until L_needed == MAX_BOOM_LENGTH (dz fixed).
        max_dx = math.sqrt(max(0.0, MAX_BOOM_LENGTH ** 2 - dz ** 2))
        required_d_horiz = max_dx + PLATFORM_TAIL
        advance = d_horiz_m - required_d_horiz
        raise InfeasibleDock(L_needed, deficit, max(0.0, advance))

    e = max(0.0, L_needed - BOOM_FIXED_LENGTH)
    # Leveling sign: the platform_level_joint axis is (0, +1, 0), OPPOSITE the
    # boom_elevation_joint axis (0, -1, 0).  A positive boom angle pitches the
    # boom tip UP; to counter-rotate the platform mount back to horizontal we
    # command the level joint to +theta_b (verified against the live TF chain:
    # level = -theta_b leaves the c-channel platform tilted ~89 deg and ~1 m
    # off-target, while level = +theta_b recovers (8.500, 0, 10.050) world).
    leveling = theta_b if leveling_mode == "active" else 0.0
    platform_world_angle = 0.0 if leveling_mode == "active" else theta_b

    # Clamp to joint limits and re-validate.
    if not (ELEVATION_LIMIT[0] <= theta_b <= ELEVATION_LIMIT[1]):
        res = IKResult(theta_b, leveling, platform_world_angle, e,
                       boom_horizontal_distance_m=d_horiz_m, docking_height_m=h_dock_m,
                       feasible=False,
                       infeasible_reason=f"theta_b {theta_b:.3f} outside "
                                         f"elevation limit {ELEVATION_LIMIT}")
        return res
    if not (LEVEL_LIMIT[0] <= leveling <= LEVEL_LIMIT[1]):
        res = IKResult(theta_b, leveling, platform_world_angle, e,
                       boom_horizontal_distance_m=d_horiz_m, docking_height_m=h_dock_m,
                       feasible=False,
                       infeasible_reason=f"leveling {leveling:.3f} outside "
                                         f"level-joint limit {LEVEL_LIMIT}")
        return res

    per_stage = split_extension(e, STAGE_COUNT)
    return IKResult(theta_b, leveling, platform_world_angle, e,
                    boom_horizontal_distance_m=d_horiz_m, docking_height_m=h_dock_m,
                    extension_per_stage_m=per_stage)


def split_extension(total_m: float, stage_count: int = STAGE_COUNT) -> List[float]:
    """Split total extension evenly across prismatic stages, clamped to limits."""
    total = max(0.0, min(total_m, MAX_STROKE))
    per = total / float(stage_count)
    return [per for _ in range(stage_count)]


def docking_lower_angle(theta_b_rad: float, h_dock_m: float,
                        extension_total_m: float = 0.0,
                        pivot_z: float = BOOM_PIVOT_WORLD_Z) -> float:
    """theta_d: extra boom-lower angle to descend onto the docking point.

    Used only for the diagram's step-5 "move down" phase: after raising and
    leveling, the platform may still sit above H_dock; theta_d is the angle
    the boom must lower to close that vertical gap while the C-opening stays
    facing the trunk.  Returns 0 when already at height.

    ``extension_total_m`` is the current prismatic stroke, so the full boom
    length (``BOOM_FIXED_LENGTH + extension_total_m``) is used to compute the
    platform height.
    """
    boom_length = BOOM_FIXED_LENGTH + extension_total_m
    current_z = pivot_z + boom_length * math.sin(theta_b_rad)
    dz = current_z - h_dock_m
    return max(0.0, math.atan2(-dz, boom_length * math.cos(theta_b_rad)))
