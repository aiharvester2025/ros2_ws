"""Platform-to-trunk distance and reachability (ROS-free).

Computes the horizontal distance from the boom pivot to the trunk axis, then
checks whether the boom can actually reach the docking point for that distance
given the closed-form IK limits.
"""

from __future__ import annotations

from dataclasses import dataclass

from harvester_boom_plan import kinematics as kin


@dataclass
class DistanceEstimate:
    trunk_axis_x_m: float
    trunk_axis_y_m: float
    base_x_m: float
    pivot_x_m: float
    d_horiz_m: float          # boom pivot -> trunk centre, world X
    reachable: bool
    max_reach_m: float
    needed_advance_m: float   # >0 means move the prime mover closer by this much
    note: str = ''


def estimate_distance(trunk_axis_xy_m, base_x_m: float,
                      margin_m: float = 0.2) -> DistanceEstimate:
    """Compute platform->trunk horizontal distance and reachability.

    ``trunk_axis_xy_m`` is the (x, y) of the trunk centreline in the world frame
    (as produced by live_height_estimator).  ``base_x_m`` is the harvester
    base-link world X.  The boom pivot is offset from base_link by the URDF
    turret + elevation X offsets (see kinematics.boom_pivot_world_xy).
    """
    ax, ay = trunk_axis_xy_m
    pivot_x, _ = kin.boom_pivot_world_xy(base_x_m, 0.0)
    d_horiz = ax - pivot_x

    # The boom can reach the docking point only when the required boom length
    # (horizontal gap to the trunk minus the platform tail, plus the vertical
    # gap) is within MAX_BOOM_LENGTH.  Use a horizontal-only reachability bound
    # for the "move prime mover" advisory: the boom's max horizontal reach is
    # MAX_BOOM_LENGTH + PLATFORM_TAIL.
    max_horiz_reach = kin.MAX_BOOM_LENGTH + kin.PLATFORM_TAIL
    reachable = d_horiz <= (max_horiz_reach - margin_m)

    needed_advance = max(0.0, d_horiz - (max_horiz_reach - margin_m))

    note = ('ok' if reachable else
            'out of reach: move prime mover +{:.2f} m'.format(needed_advance))

    return DistanceEstimate(
        trunk_axis_x_m=ax, trunk_axis_y_m=ay, base_x_m=base_x_m,
        pivot_x_m=pivot_x, d_horiz_m=d_horiz, reachable=reachable,
        max_reach_m=max_horiz_reach, needed_advance_m=needed_advance, note=note)


__all__ = ['DistanceEstimate', 'estimate_distance']
