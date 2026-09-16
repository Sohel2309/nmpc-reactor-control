import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.constraints import clip_input, clip_rate, clip_physical, check_violations
from src.config import load_config

cfg = load_config()


def test_clip_input_respects_bounds():
    c = cfg["constraints"]
    u = clip_input(np.array([1000.0, 1000.0]), cfg)
    assert u[0] == c["F_max"]
    assert u[1] == c["Qk_max"]

    u2 = clip_input(np.array([-1000.0, -1e9]), cfg)
    assert u2[0] == c["F_min"]
    assert u2[1] == c["Qk_min"]


def test_clip_rate_limits_step_size():
    c = cfg["constraints"]
    u_prev = np.array([15.0, -5000.0])
    u_requested = np.array([100.0, 5000.0])  # huge jump, also outside bounds
    u_applied = clip_rate(u_requested, u_prev, cfg)
    assert abs(u_applied[0] - u_prev[0]) <= c["dF_max"] + 1e-9
    assert abs(u_applied[1] - u_prev[1]) <= c["dQk_max"] + 1e-9


def test_clip_rate_then_bounds_never_violated():
    c = cfg["constraints"]
    u_prev = np.array([c["F_max"] - 1.0, c["Qk_max"] - 100.0])
    u_requested = np.array([c["F_max"] + 50, c["Qk_max"] + 5000])
    u_applied = clip_rate(u_requested, u_prev, cfg)
    assert u_applied[0] <= c["F_max"] + 1e-9
    assert u_applied[1] <= c["Qk_max"] + 1e-9


def test_clip_physical_prevents_negative_concentrations():
    x = np.array([-0.5, -0.1, 130.0, 120.0])
    x_clipped = clip_physical(x)
    assert x_clipped[0] >= 0.0
    assert x_clipped[1] >= 0.0


def test_check_violations_detects_temperature_violation():
    c = cfg["constraints"]
    x = np.array([1.0, 0.9, c["T_safe_max"] + 5.0, 120.0])
    u = np.array([15.0, -5000.0])
    v = check_violations(x, u, u, cfg)
    assert v["T_violation"] > 0
    assert v["any_violation"] is True


def test_check_violations_clean_state_has_no_violations():
    c = cfg["constraints"]
    x = np.array([cfg["nominal_state"]["cA"], cfg["nominal_state"]["cB"],
                  cfg["nominal_state"]["T"], cfg["nominal_state"]["TK"]])
    u = np.array([cfg["nominal_input"]["F"], cfg["nominal_input"]["Qk"]])
    v = check_violations(x, u, u, cfg)
    assert v["any_violation"] is False


def test_check_violations_rate_violation_detected():
    u_prev = np.array([15.0, -5000.0])
    u_now = np.array([15.0 + 100.0, -5000.0])  # way beyond dF_max
    x = np.array([1.0, 0.9, 130.0, 120.0])
    v = check_violations(x, u_now, u_prev, cfg)
    assert v["dF_violation"] > 0
