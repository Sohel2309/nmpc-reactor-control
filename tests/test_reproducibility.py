import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import Scenario, run_closed_loop

cfg = load_config()


def test_pid_run_is_deterministic():
    scenario = Scenario(name="repro_pid", t_final=1.0,
                         setpoint_changes=[(0.0, cfg["nominal_state"]["cB"] + 0.05)],
                         process_noise_std=0.01, meas_noise_std=0.01, seed=123)
    run1 = run_closed_loop("pid", cfg, scenario)
    run2 = run_closed_loop("pid", cfg, scenario)
    assert np.allclose(run1["x"], run2["x"])
    assert np.allclose(run1["u"], run2["u"])


def test_nmpc_run_is_deterministic():
    scenario = Scenario(name="repro_nmpc", t_final=0.5,
                         setpoint_changes=[(0.0, cfg["nominal_state"]["cB"] + 0.05)],
                         process_noise_std=0.005, meas_noise_std=0.005, seed=7)
    run1 = run_closed_loop("nmpc", cfg, scenario, N=8)
    run2 = run_closed_loop("nmpc", cfg, scenario, N=8)
    assert np.allclose(run1["x"], run2["x"])
    assert np.allclose(run1["u"], run2["u"])


def test_different_seeds_give_different_noisy_trajectories():
    scenario_a = Scenario(name="repro_seedA", t_final=0.5,
                           setpoint_changes=[(0.0, cfg["nominal_state"]["cB"])],
                           meas_noise_std=0.05, seed=1)
    scenario_b = Scenario(name="repro_seedB", t_final=0.5,
                           setpoint_changes=[(0.0, cfg["nominal_state"]["cB"])],
                           meas_noise_std=0.05, seed=2)
    run_a = run_closed_loop("pid", cfg, scenario_a)
    run_b = run_closed_loop("pid", cfg, scenario_b)
    assert not np.allclose(run_a["u"], run_b["u"])
