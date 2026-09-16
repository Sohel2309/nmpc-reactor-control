import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.pid_controller import PIDController
from src.config import load_config
from src.simulation import Scenario, run_closed_loop
from src.metrics import summarize_run

cfg = load_config()
U_SS = [cfg["nominal_input"]["F"], cfg["nominal_input"]["Qk"]]


def test_pid_zero_error_gives_zero_proportional_and_derivative_action():
    pid = PIDController(cfg, U_SS)
    sp = cfg["nominal_state"]["cB"]
    u1 = pid.compute(sp, sp, dt=0.05)
    # at exactly zero error with zero initial integral, F should not move
    # far from steady state on the first call.
    assert abs(u1[0] - U_SS[0]) < 1e-6


def test_pid_positive_error_increases_F():
    """A measurement below setpoint (need more conversion) should command
    an increase in F, since dcB/dF > 0 at the nominal point (documented
    pairing choice)."""
    pid = PIDController(cfg, U_SS)
    sp = cfg["nominal_state"]["cB"] + 0.05
    meas = cfg["nominal_state"]["cB"]
    u1 = pid.compute(sp, meas, dt=0.05)
    assert u1[0] > U_SS[0]


def test_pid_output_respects_input_bounds():
    pid = PIDController(cfg, U_SS)
    c = cfg["constraints"]
    # Command an enormous fictitious error to try to force saturation
    u = None
    for _ in range(50):
        u = pid.compute(setpoint=10.0, measurement=0.0, dt=0.05)
    assert c["F_min"] - 1e-9 <= u[0] <= c["F_max"] + 1e-9


def test_pid_output_respects_rate_limits():
    pid = PIDController(cfg, U_SS)
    c = cfg["constraints"]
    u_first = pid.compute(setpoint=5.0, measurement=0.0, dt=0.05)
    assert abs(u_first[0] - U_SS[0]) <= c["dF_max"] + 1e-9


def test_pid_reset_restores_initial_state():
    pid = PIDController(cfg, U_SS)
    pid.compute(1.2, 0.5, dt=0.05)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error is None
    assert np.allclose(pid.u_prev, U_SS)


def test_pid_tracks_small_setpoint_step_without_violating_constraints():
    """Closed-loop integration test: PID baseline on a small setpoint
    step must reduce tracking error over time and must not violate the
    reactor's hard safety/actuator constraints."""
    scenario = Scenario(name="pid_small_step", t_final=2.0,
                         setpoint_changes=[(0.0, cfg["nominal_state"]["cB"]),
                                            (0.5, cfg["nominal_state"]["cB"] + 0.05)])
    run = run_closed_loop("pid", cfg, scenario)
    m = summarize_run(run)
    assert m["n_T_violations"] == 0
    # error should have decreased substantially by the end of the run
    final_err = abs(run["setpoint"][-1] - run["x"][-1, 1])
    initial_err_after_step = abs(run["setpoint"][11] - run["x"][11, 1])
    assert final_err < initial_err_after_step
