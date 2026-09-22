"""Tests for the cutter safety-guidance model (pure logic, no Qt)."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

from harvester_dashboard.cutter_safety_guidance import (
    PHASE_ADVANCE,
    PHASE_ALIGN,
    PHASE_APPROACH,
    PHASE_CUT,
    PHASE_IDLE,
    PHASE_OPEN,
    SAFE,
    WARN,
    DANGER,
    NO_DATA,
    CutterConfig,
    CutterGuidance,
    advance_phase,
    evaluate,
    next_phase,
    phase_message,
    state_message,
    tip_clearance_m,
)


def cfg(**overrides):
    values = {
        'sensor_to_tip_offset_m': 0.19,
        'a_max_m_s2': 0.10,
        'latency_s': 0.30,
        'warn_margin': 0.7,
        'warn_clearance_m': 0.30,
        'danger_clearance_m': 0.10,
        'ready_standoff_m': 0.20,
        'advance_distance_m': 0.05,
        'align_tolerance_m': 0.05,
        'stationary_epsilon_cm_s': 0.5,
        'stale_s': 2.0,
        'debounce_s': 0.4,
        'speed_ema_alpha': 0.35,
    }
    values.update(overrides)
    return CutterConfig(**values)


# --- offset correction ------------------------------------------------------

def test_tip_clearance_applies_offset():
    # Raw range 1.00 m, sensor 0.19 m behind the tip -> true clearance 0.81 m.
    assert abs(tip_clearance_m(1.00, cfg()) - 0.81) < 1e-9


def test_offset_is_configurable():
    assert abs(tip_clearance_m(1.00, cfg(sensor_to_tip_offset_m=0.5)) - 0.5) < 1e-9


# --- states -----------------------------------------------------------------

def test_missing_input_is_no_data():
    g = evaluate(None, None, cfg())
    assert g.state == NO_DATA
    assert g.ttc_s is None


def test_stale_input_is_no_data():
    assert evaluate(5.0, 0.5, cfg(), stale=True).state == NO_DATA


def test_moving_away_is_safe():
    assert evaluate(-10.0, 0.5, cfg()).state == SAFE


def test_slow_far_is_safe():
    g = evaluate(10.0, 0.8, cfg())
    assert g.state == SAFE
    assert g.clearance_m == 0.8


def test_close_clearance_warns():
    # 5 cm/s at 0.25 m -> slow, but <= warn_clearance (0.30).
    assert evaluate(5.0, 0.25, cfg()).state == WARN


def test_danger_by_absolute_clearance_even_stationary():
    assert evaluate(0.0, 0.08, cfg()).state == DANGER


def test_danger_by_stopping_distance():
    # 60 cm/s -> d_stop = 0.6*0.3 + 0.36/0.2 = 1.98 m; at 0.5 m -> DANGER.
    g = evaluate(60.0, 0.5, cfg())
    assert g.state == DANGER
    assert g.stop_distance_m >= 0.5


def test_recommended_speed_decreases_with_clearance():
    near = evaluate(5.0, 0.15, cfg())
    far = evaluate(5.0, 0.6, cfg())
    assert near.max_speed_cm_s < far.max_speed_cm_s


def test_max_safe_speed_monotonic_positive():
    a = evaluate(0.0, 0.2, cfg())
    b = evaluate(0.0, 0.8, cfg())
    assert a.max_speed_cm_s > 0.0
    assert b.max_speed_cm_s > a.max_speed_cm_s


def test_hand_built_zero_a_max_does_not_crash():
    # A directly-constructed (unsanitized) zero deceleration must be guarded.
    g = evaluate(40.0, 0.5, CutterConfig(a_max_m_s2=0.0))
    assert g.state == DANGER


# --- cut sequence -----------------------------------------------------------

def test_phase_approaches_then_aligns():
    assert next_phase(PHASE_APPROACH, 0.8, 20.0, cfg()) == PHASE_APPROACH
    assert next_phase(PHASE_APPROACH, 0.20, 5.0, cfg()) == PHASE_ALIGN


def test_phase_align_within_tolerance():
    # ready_standoff 0.20 + tolerance 0.05 -> align at <= 0.25 m
    assert next_phase(PHASE_APPROACH, 0.24, 5.0, cfg()) == PHASE_ALIGN
    # just outside
    assert next_phase(PHASE_APPROACH, 0.30, 5.0, cfg()) == PHASE_APPROACH


def test_phase_align_requires_settled_speed():
    # Regression: ALIGN ("STOP, ready to cut") must NOT fire while the tip is
    # still closing fast, or the prompt would contradict a DANGER/WARN warning.
    # At 0.20 m the max safe speed is well below 30 cm/s.
    assert next_phase(PHASE_APPROACH, 0.20, 30.0, cfg()) == PHASE_APPROACH
    # Once settled, the same clearance does align.
    assert next_phase(PHASE_APPROACH, 0.20, 3.0, cfg()) == PHASE_ALIGN


def test_phase_align_gate_is_warn_margin_speed():
    # ALIGN must not appear while the operator is being told to slow (speed
    # above warn_margin * v_max), so the two signals never disagree.
    c = cfg()
    v_max = __import__(
        'harvester_dashboard.cutter_safety_guidance', fromlist=['_max_safe_speed_cm_s']
    )._max_safe_speed_cm_s(0.20, c)
    assert next_phase(PHASE_APPROACH, 0.20, c.warn_margin * v_max + 1.0, c) == PHASE_APPROACH
    assert next_phase(PHASE_APPROACH, 0.20, c.warn_margin * v_max - 1.0, c) == PHASE_ALIGN


def test_phase_align_does_not_contradict_danger():
    # A DANGER reading must never be paired with an ALIGN prompt from APPROACH.
    fast = evaluate(30.0, 0.20, cfg())
    assert fast.state == DANGER
    assert next_phase(PHASE_APPROACH, 0.20, 30.0, cfg()) == PHASE_APPROACH


def test_phase_holds_operator_steps_through_stale():
    # A range dropout must NOT wipe in-progress operator-confirmed phases.
    assert next_phase(PHASE_ALIGN, 0.20, 5.0, cfg(), stale=True) == PHASE_ALIGN
    assert next_phase(PHASE_OPEN, None, 0.0, cfg(), stale=True) == PHASE_OPEN
    assert next_phase(PHASE_ADVANCE, None, 0.0, cfg(), stale=True) == PHASE_ADVANCE
    # But an un-started approach with stale data stays APPROACH (never aligns).
    assert next_phase(PHASE_APPROACH, None, 0.0, cfg(), stale=True) == PHASE_APPROACH
    assert next_phase(PHASE_APPROACH, 0.10, 5.0, cfg(), stale=True) == PHASE_APPROACH


def test_advance_phase_order():
    assert advance_phase(PHASE_ALIGN) == PHASE_OPEN
    assert advance_phase(PHASE_OPEN) == PHASE_ADVANCE
    assert advance_phase(PHASE_ADVANCE) == PHASE_CUT
    assert advance_phase(PHASE_CUT) == PHASE_CUT  # terminal


def test_phase_messages_present():
    for phase in (PHASE_APPROACH, PHASE_ALIGN, PHASE_OPEN, PHASE_ADVANCE,
                  PHASE_CUT, PHASE_IDLE):
        assert phase_message(phase, cfg())


def test_state_message_matches_held_state():
    candidate = evaluate(DANGER and 60.0, 0.5, cfg())  # DANGER
    held = CutterGuidance(WARN, PHASE_ALIGN, candidate.clearance_m,
                          candidate.speed_cm_s, candidate.ttc_s,
                          candidate.stop_distance_m, candidate.max_speed_cm_s,
                          candidate.recommended_speed_cm_s,
                          candidate.message, candidate.phase_message)
    msg = state_message(WARN, held)
    assert 'SLOW' in msg and 'STOP' not in msg
    assert 'STOP' in state_message(DANGER, held)


def test_phase_echoed_in_result():
    g = evaluate(5.0, 0.5, cfg(), phase=PHASE_OPEN)
    assert g.phase == PHASE_OPEN
    assert g.phase_message  # non-empty prompt


# --- config -----------------------------------------------------------------

def test_config_defaults():
    assert CutterConfig() == cfg()


def test_config_load_missing_file_falls_back():
    assert CutterConfig.load(None) == CutterConfig()


def test_config_load_sanitizes():
    fd, path = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    with open(path, 'w') as fh:
        json.dump({'a_max_m_s2': 0.0, 'warn_margin': 5.0,
                   'sensor_to_tip_offset_m': 0.25}, fh)
    loaded = CutterConfig.load(Path(path))
    assert loaded.a_max_m_s2 >= 1e-3
    assert loaded.warn_margin == 1.0
    assert loaded.sensor_to_tip_offset_m == 0.25
    os.remove(path)
