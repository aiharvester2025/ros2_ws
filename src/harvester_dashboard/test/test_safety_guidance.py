"""Tests for the docking safety-guidance model (pure logic, no Qt)."""

from __future__ import annotations

from harvester_dashboard.safety_guidance import (
    DANGER,
    NO_DATA,
    SAFE,
    WARN,
    Guidance,
    SafetyConfig,
    evaluate,
    state_message,
)


def cfg(**overrides):
    values = {
        'a_max_m_s2': 0.10,
        'latency_s': 0.30,
        'warn_margin': 0.7,
        'warn_ttc_s': 4.0,
        'danger_ttc_s': 1.5,
        'warn_distance_m': 1.0,
        'danger_distance_m': 0.3,
        'stationary_epsilon_cm_s': 0.5,
        'stale_s': 2.0,
        'debounce_s': 0.4,
        'speed_ema_alpha': 0.35,
    }
    values.update(overrides)
    return SafetyConfig(**values)


def test_missing_input_is_no_data():
    g = evaluate(None, None, cfg())
    assert g.state == NO_DATA
    assert g.ttc_s is None
    assert g.recommended_speed_cm_s is None


def test_stale_input_is_no_data():
    g = evaluate(5.0, 2.0, cfg(), stale=True)
    assert g.state == NO_DATA


def test_stationary_is_safe_outside_danger():
    g = evaluate(0.0, 5.0, cfg())
    assert g.state == SAFE
    assert g.approaching is False


def test_stationary_inside_danger_distance_is_danger():
    g = evaluate(0.0, 0.2, cfg())
    assert g.state == DANGER


def test_moving_away_is_safe():
    g = evaluate(-10.0, 5.0, cfg())
    assert g.state == SAFE


def test_fast_far_approach_is_safe():
    # 20 cm/s at 5 m -> TTC = 25 s; stopping distance ~0.8 m << 5 m gap.
    g = evaluate(20.0, 5.0, cfg())
    assert g.state == SAFE
    assert g.ttc_s == 25.0
    assert g.stop_distance_m < 5.0


def test_moderate_approach_warns_by_ttc():
    # 40 cm/s at 1.5 m -> TTC = 3.75 s <= warn(4) but > danger(1.5).
    g = evaluate(40.0, 1.5, cfg())
    assert g.state == WARN
    assert g.recommended_speed_cm_s is not None


def test_fast_approach_danger_by_ttc():
    # 50 cm/s at 0.6 m -> TTC = 1.2 s <= danger(1.5).
    g = evaluate(50.0, 0.6, cfg())
    assert g.state == DANGER


def test_danger_by_stopping_distance():
    # A fast approach where the stopping distance exceeds the remaining gap
    # must trip DANGER even if TTC is above the danger band.
    # v = 60 cm/s = 0.6 m/s; d_stop = 0.6*0.3 + 0.36/0.2 = 0.18 + 1.8 = 1.98 m.
    # At a 1.5 m gap, d_stop (1.98) > 1.5 -> DANGER.  TTC = 1.5/0.6 = 2.5 s > 1.5.
    g = evaluate(60.0, 1.5, cfg())
    assert g.state == DANGER
    assert g.stop_distance_m >= 1.5
    assert g.ttc_s > 1.5  # above the danger TTC band -> stopping distance tripped


def test_close_distance_warns_even_when_slow():
    # 5 cm/s at 0.8 m -> TTC large, but distance <= warn_distance (1.0).
    g = evaluate(5.0, 0.8, cfg())
    assert g.state == WARN


def test_recommended_speed_decreases_with_distance():
    # The recommended max-safe speed must shrink as the gap shrinks.
    near = evaluate(5.0, 0.8, cfg())
    far = evaluate(5.0, 2.0, cfg())
    assert near.max_speed_cm_s < far.max_speed_cm_s


def test_max_safe_speed_monotonic_and_physical():
    # v_max(d) grows with distance and is positive.
    g1 = evaluate(0.0, 0.5, cfg())
    g2 = evaluate(0.0, 2.0, cfg())
    assert g1.max_speed_cm_s > 0.0
    assert g2.max_speed_cm_s > g1.max_speed_cm_s


def test_warn_margin_scales_recommendation():
    # recommended = warn_margin * v_max, so it is below the physical limit.
    g = evaluate(30.0, 1.0, cfg())
    assert g.recommended_speed_cm_s is not None
    assert g.recommended_speed_cm_s < g.max_speed_cm_s


def test_config_load_falls_back_on_missing_file():
    loaded = SafetyConfig.load(None)
    assert loaded.a_max_m_s2 == 0.10
    assert loaded == SafetyConfig()


def test_config_load_keeps_new_fields():
    import json
    import tempfile
    import os
    fd, path = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    with open(path, 'w') as fh:
        json.dump({'a_max_m_s2': 0.25, 'latency_s': 0.5}, fh)
    loaded = SafetyConfig.load(__import__('pathlib').Path(path))
    assert loaded.a_max_m_s2 == 0.25
    assert loaded.latency_s == 0.5
    assert loaded.warn_margin == 0.7  # default preserved
    os.remove(path)


def test_guidance_is_dataclass_equality():
    a = Guidance(SAFE, 25.0, 20.0, 5.0, 0.8, 30.0, None, 'clear')
    b = Guidance(SAFE, 25.0, 20.0, 5.0, 0.8, 30.0, None, 'clear')
    assert a == b


def test_state_message_matches_held_state():
    # A held WARN state must never render a DANGER ("STOP") message even when
    # the candidate has advanced (hysteresis debounce consistency).
    candidate = evaluate(60.0, 1.5, cfg())  # DANGER by stopping distance
    held = Guidance(WARN, candidate.ttc_s, candidate.speed_cm_s,
                    candidate.distance_m, candidate.stop_distance_m,
                    candidate.max_speed_cm_s, candidate.recommended_speed_cm_s,
                    candidate.message)
    msg = state_message(WARN, held)
    assert 'SLOW' in msg
    assert 'STOP' not in msg
    # And DANGER always reads as a stop.
    assert 'STOP' in state_message(DANGER, held)
