import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.metrics import (iae, ise, mae, rmse, settling_time, overshoot,
                          control_effort, total_variation, saturation_time,
                          summarize_run)
from src.config import load_config
from src.simulation import Scenario, run_closed_loop

cfg = load_config()


def test_iae_zero_for_zero_error():
    t = np.linspace(0, 1, 100)
    e = np.zeros_like(t)
    assert iae(t, e) == 0.0


def test_iae_matches_analytical_constant_error():
    """IAE of a constant error e over duration T is exactly |e|*T."""
    t = np.linspace(0, 2.0, 201)
    e = np.full_like(t, 0.5)
    assert abs(iae(t, e) - 0.5 * 2.0) < 1e-6


def test_ise_matches_analytical_constant_error():
    t = np.linspace(0, 2.0, 201)
    e = np.full_like(t, 0.5)
    assert abs(ise(t, e) - (0.5 ** 2) * 2.0) < 1e-6


def test_mae_rmse_known_values():
    e = np.array([1.0, -1.0, 2.0, -2.0])
    assert mae(e) == 1.5
    assert abs(rmse(e) - np.sqrt((1 + 1 + 4 + 4) / 4)) < 1e-9


def test_settling_time_detects_immediate_settling():
    t = np.linspace(0, 5, 50)
    y = np.full_like(t, 1.0)
    st = settling_time(t, y, setpoint=1.0, band=0.02)
    assert st == 0.0


def test_settling_time_infinite_if_never_settles():
    t = np.linspace(0, 5, 50)
    y = np.full_like(t, 0.5)  # always far from setpoint
    st = settling_time(t, y, setpoint=1.0, band=0.02)
    assert st == float("inf")


def test_overshoot_zero_for_monotonic_approach():
    setpoint = 1.0
    y0 = 0.5
    y = np.linspace(y0, setpoint, 20)  # monotonic, no overshoot
    assert overshoot(y, setpoint, y0) == 0.0


def test_overshoot_detects_peak_above_setpoint():
    setpoint = 1.0
    y0 = 0.5
    y = np.array([0.5, 0.8, 1.2, 1.05, 1.0])  # overshoots to 1.2
    os_pct = overshoot(y, setpoint, y0)
    expected = 100.0 * (1.2 - 1.0) / (1.0 - 0.5)
    assert abs(os_pct - expected) < 1e-9


def test_control_effort_and_total_variation_known_sequence():
    u = np.array([[1.0], [2.0], [4.0]])
    assert control_effort(u) == (1.0 ** 2 + 2.0 ** 2)
    assert total_variation(u) == (1.0 + 2.0)


def test_saturation_time_counts_only_saturated_samples():
    u = np.array([0.0, 5.0, 10.0, 10.0, 3.0])
    dt = 0.05
    st = saturation_time(u, u_min=0.0, u_max=10.0, dt=dt)
    assert abs(st - 3 * dt) < 1e-9  # samples at 0, 10, 10 are saturated


def test_summarize_run_produces_expected_keys_for_pid():
    scenario = Scenario(name="metrics_test", t_final=1.0,
                         setpoint_changes=[(0.0, cfg["nominal_state"]["cB"]),
                                            (0.3, cfg["nominal_state"]["cB"] + 0.05)])
    run = run_closed_loop("pid", cfg, scenario)
    m = summarize_run(run)
    for key in ["IAE", "ISE", "MAE", "RMSE", "control_effort_F",
                "n_T_violations", "overshoot_pct_max", "segments"]:
        assert key in m
    assert m["IAE"] >= 0


def test_summarize_run_includes_solver_stats_for_nmpc():
    scenario = Scenario(name="metrics_test_nmpc", t_final=0.5,
                         setpoint_changes=[(0.0, cfg["nominal_state"]["cB"])])
    run = run_closed_loop("nmpc", cfg, scenario, N=8)
    m = summarize_run(run)
    assert "solve_time_mean" in m
    assert m["solve_time_mean"] > 0
