"""Docking-safety state machine for the five fixed range sensors.

Two roles (per the plan):

1. **Clearance/skew watchdog** — a continuous, monotonic guard that only
   permits approach motion when the relevant sensors read above a minimum gap,
   and trips ``EMERGENCY_STOP`` on any near-contact.

2. **Trunk-centering confirmation** — the side pair and the +-45 pair
   reconstruct the trunk centreline inside the C-channel opening.  This is an
   independent geometry gate that must pass (together with the watchdog) before
   ``DOCKING_OK`` is admitted.

This module is ROS-free and operates on plain sensor readings so it can be
unit-tested without a running graph.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Optional


# Phase ordering (for reporting only; the evaluator computes phases fresh).
PHASES = [
    "INIT",
    "WAITING_FOR_TREE",
    "READY",
    "EXTEND_OK",
    "HOLD_AT_DISTANCE",
    "ALIGN_OK",
    "LEVEL_OK",
    "DOCKING_OK",
    "EMERGENCY_STOP",
    "INFEASIBLE_DOCK_HEIGHT",
]


@dataclass
class SafetyConfig:
    """Safety/centering thresholds (single source of truth: boom_docking.yaml).

    There are no defaults: the caller must supply every field from the YAML so
    the dataclass and the configuration cannot silently diverge.
    """
    extend_safe_m: float
    extend_stop_m: float
    align_skew_m: float
    docking_gap_m: float
    emergency_m: float
    stale_receipt_timeout_s: float
    centering_tolerance_m: float


@dataclass
class SafetyResult:
    phase: str
    centering: Dict[str, float] = field(default_factory=dict)
    notes: str = ""


def evaluate(center: Optional[float], c45_l: Optional[float],
             c45_r: Optional[float], side_l: Optional[float],
             side_r: Optional[float], cutter: Optional[float],
             cfg: SafetyConfig, stale: bool = False) -> SafetyResult:
    """Evaluate the docking safety state from the five (plus cutter) ranges.

    ``stale`` is a precomputed flag meaning "data older than the timeout".
    Callers pass ``None`` for any sensor that has no fresh return.

    Phase priority (highest first):
      1. WAITING_FOR_TREE  — no fresh data at all
      2. EMERGENCY_STOP    — any sensor below the near-contact threshold
      3. DOCKING_OK        — all sensors small AND trunk centred (success)
      4. LEVEL_OK / ALIGN_OK — centering sub-gates while approaching
      5. HOLD_AT_DISTANCE / EXTEND_OK — watchdog clearance gates (approach only)
      6. READY             — default
    """
    if stale or all(v is None for v in (center, c45_l, c45_r, side_l, side_r)):
        return SafetyResult("WAITING_FOR_TREE", {}, "no fresh range returns")

    present = [v for v in (center, c45_l, c45_r, side_l, side_r, cutter)
               if v is not None]

    # --- 2. EMERGENCY_STOP: any sensor in near-contact ----------------------
    if present and min(present) <= cfg.emergency_m:
        return SafetyResult("EMERGENCY_STOP", {},
                            f"near-contact: min range {min(present):.3f} <= "
                            f"{cfg.emergency_m}")

    # --- Centering reconstruction (side pair) -------------------------------
    centering = {}
    side_pair_ok = False
    if side_l is not None and side_r is not None:
        diameter = side_l + side_r
        centre_offset_y = (side_l - side_r) / 2.0
        centering["diameter_m"] = diameter
        centering["centre_offset_y_m"] = centre_offset_y
        side_pair_ok = abs(centre_offset_y) <= cfg.centering_tolerance_m

    diag_ok = False
    if c45_l is not None and c45_r is not None:
        skew45 = abs(c45_l - c45_r)
        centering["diag_skew_m"] = skew45
        diag_ok = skew45 <= cfg.align_skew_m

    # --- 3. DOCKING_OK: all present sensors within gap AND centred ----------
    all_in_gap = present and max(present) <= cfg.docking_gap_m
    if all_in_gap and side_pair_ok and diag_ok:
        return SafetyResult("DOCKING_OK", centering, "centred and in gap")

    # --- 4. Centering sub-gates while still approaching ---------------------
    if side_l is not None and side_r is not None and not side_pair_ok:
        return SafetyResult("LEVEL_OK", centering,
                            f"trunk centre offset {centre_offset_y:+.3f} m "
                            f"exceeds {cfg.centering_tolerance_m} m")
    if c45_l is not None and c45_r is not None and not diag_ok:
        return SafetyResult("ALIGN_OK", centering,
                            f"diag skew {skew45:.3f} > {cfg.align_skew_m}")

    # --- 5. Watchdog clearance gates (approach only) ------------------------
    front = [v for v in (center, c45_l, c45_r) if v is not None]
    if front:
        front_min = min(front)
        if front_min < cfg.extend_stop_m:
            return SafetyResult("HOLD_AT_DISTANCE", centering,
                                f"front clearance {front_min:.3f} < "
                                f"{cfg.extend_stop_m}")
        if front_min < cfg.extend_safe_m:
            return SafetyResult("EXTEND_OK", centering,
                                f"front clearance {front_min:.3f} within safe "
                                f"extension band")

    # --- 6. Default ---------------------------------------------------------
    return SafetyResult("READY", centering, "approaching docking point")
