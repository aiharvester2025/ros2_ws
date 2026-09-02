"""Unit tests for the docking-safety state machine."""

import os

import pytest
import yaml

from harvester_boom_plan import safety as safety_mod


def cfg():
    """Construct SafetyConfig from the same YAML used in production."""
    here = os.path.dirname(__file__)
    yaml_path = os.path.join(here, "..", "config", "boom_docking.yaml")
    with open(yaml_path, "r", encoding="utf-8") as fh:
        s = yaml.safe_load(fh)["safety"]
    return safety_mod.SafetyConfig(
        extend_safe_m=float(s["extend_safe_m"]),
        extend_stop_m=float(s["extend_stop_m"]),
        align_skew_m=float(s["align_skew_m"]),
        docking_gap_m=float(s["docking_gap_m"]),
        emergency_m=float(s["emergency_m"]),
        stale_receipt_timeout_s=float(s["stale_receipt_timeout_s"]),
        centering_tolerance_m=float(s["centering_tolerance_m"]),
    )


def test_waiting_when_no_data():
    r = safety_mod.evaluate(None, None, None, None, None, None, cfg(), stale=True)
    assert r.phase == "WAITING_FOR_TREE"


def test_emergency_stop_on_near_contact():
    r = safety_mod.evaluate(0.03, 0.5, 0.5, 0.5, 0.5, None, cfg(), stale=False)
    assert r.phase == "EMERGENCY_STOP"


def test_hold_at_distance():
    r = safety_mod.evaluate(0.35, 0.35, 0.35, None, None, None, cfg(), stale=False)
    assert r.phase == "HOLD_AT_DISTANCE"


def test_docking_ok_when_centred_and_in_gap():
    r = safety_mod.evaluate(0.10, 0.10, 0.10, 0.15, 0.15, None, cfg(), stale=False)
    assert r.phase == "DOCKING_OK"
    assert r.centering["centre_offset_y_m"] == pytest.approx(0.0)


def test_docking_not_ok_when_off_centre():
    r = safety_mod.evaluate(0.12, 0.12, 0.12, 0.10, 0.50, None, cfg(), stale=False)
    assert r.phase != "DOCKING_OK"
    assert abs(r.centering["centre_offset_y_m"]) > cfg().centering_tolerance_m


def test_diagonal_skew_blocks_docking():
    r = safety_mod.evaluate(0.10, 0.10, 0.30, 0.15, 0.15, None, cfg(), stale=False)
    assert r.phase in ("ALIGN_OK", "LEVEL_OK", "READY")
    assert r.phase != "DOCKING_OK"


def test_missing_diagonal_pair_blocks_docking():
    # ±45° sensors absent: DOCKING_OK must NOT be admitted even when the side
    # pair is centred and all present sensors are within the docking gap.
    r = safety_mod.evaluate(0.10, None, None, 0.15, 0.15, None, cfg(), stale=False)
    assert r.phase != "DOCKING_OK"
