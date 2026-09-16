import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.nmpc_controller import NMPCController
from src.config import load_config
from src.dynamics import apply_mismatch
from src.simulation import Scenario, run_closed_loop

cfg = load_config()
P = cfg["plant"]
X_SS = np.array([cfg["nominal_state"][k] for k in ["cA", "cB", "T", "TK"]])
U_SS = np.array([cfg["nominal_input"]["F"], cfg["nominal_input"]["Qk"]])


def _fresh_controller(N=8):
    return NMPCController(P, cfg, N=N)


def test_nmpc_solve_succeeds_at_steady_state_with_matching_setpoint():
    ctrl = _fresh_controller()
    u, info = ctrl.solve(X_SS, U_SS, cB_setpoint=X_SS[1])
    assert info["success"] is True
    # Already at the setpoint at steady state: optimal input should stay
    # very close to the steady input (no reason to move).
    assert np.allclose(u, U_SS, atol=0.5)


def test_nmpc_moves_input_towards_higher_setpoint():
    ctrl = _fresh_controller()
    u, info = ctrl.solve(X_SS, U_SS, cB_setpoint=X_SS[1] + 0.15)
    assert u[0] > U_SS[0]  # increasing cB target requires increasing F


def test_nmpc_output_respects_input_bounds():
    ctrl = _fresh_controller()
    c = cfg["constraints"]
    # Ask for an unreasonably high setpoint to try to force saturation
    u, info = ctrl.solve(X_SS, U_SS, cB_setpoint=5.0)
    assert c["F_min"] - 1e-6 <= u[0] <= c["F_max"] + 1e-6
    assert c["Qk_min"] - 1e-6 <= u[1] <= c["Qk_max"] + 1e-6


def test_nmpc_output_respects_rate_limits():
    ctrl = _fresh_controller()
    c = cfg["constraints"]
    u, info = ctrl.solve(X_SS, U_SS, cB_setpoint=5.0)
    assert abs(u[0] - U_SS[0]) <= c["dF_max"] + 1e-6
    assert abs(u[1] - U_SS[1]) <= c["dQk_max"] + 1e-6


def test_nmpc_predicted_temperature_respects_soft_safety_limit():
    """Even under an aggressive setpoint push, the predicted trajectory's
    temperature should not exceed T_safe_max by more than a small amount
    (the slack penalty is large but finite, so tiny overshoot is allowed
    and reported, not silently hidden -- see docs/limitations.md)."""
    ctrl = _fresh_controller()
    c = cfg["constraints"]
    u, info = ctrl.solve(X_SS, U_SS, cB_setpoint=5.0)
    predicted_T = info["predicted_X"][2, :]
    assert predicted_T.max() <= c["T_safe_max"] + 1.0


def test_nmpc_objective_prefers_setpoint_over_large_moves():
    """A setpoint exactly at the current state should not trigger large
    control moves purely due to the move-suppression terms fighting a
    nonzero tracking term (sanity check that weights are sensibly scaled)."""
    ctrl = _fresh_controller()
    u, info = ctrl.solve(X_SS, U_SS, cB_setpoint=X_SS[1])
    assert abs(u[0] - U_SS[0]) < 1.0
    assert abs(u[1] - U_SS[1]) < 200.0


def test_nmpc_receding_horizon_uses_only_first_move_and_replans():
    """Verify genuine receding-horizon behaviour: solving twice in a row
    from two DIFFERENT states must give different first-move outputs
    (i.e. NMPC is not just replaying a precomputed open-loop sequence)."""
    ctrl = _fresh_controller()
    u_a, _ = ctrl.solve(X_SS, U_SS, cB_setpoint=1.0)
    x_perturbed = X_SS.copy()
    x_perturbed[1] -= 0.1  # simulate a disturbance pushing cB down
    u_b, _ = ctrl.solve(x_perturbed, u_a, cB_setpoint=1.0)
    assert not np.allclose(u_a, u_b)
    # Perturbed (further from setpoint) state should command a larger F
    assert u_b[0] > u_a[0]


def test_model_mismatch_produces_different_internal_model():
    """The NMPC's internal model, when built with a nonzero mismatch
    multiplier, must give different predictions than the unmismatched
    model for the same state/input (otherwise the mismatch experiment
    would not be testing anything real)."""
    mismatched_params = apply_mismatch(P, {"k10_ab_mult": 0.5})
    ctrl_matched = NMPCController(P, cfg, N=5)
    ctrl_mismatched = NMPCController(mismatched_params, cfg, N=5)

    u_matched, info_matched = ctrl_matched.solve(X_SS, U_SS, cB_setpoint=1.0)
    u_mismatched, info_mismatched = ctrl_mismatched.solve(X_SS, U_SS, cB_setpoint=1.0)

    pred_matched = info_matched["predicted_X"]
    pred_mismatched = info_mismatched["predicted_X"]
    assert not np.allclose(pred_matched, pred_mismatched)


def test_nmpc_closed_loop_run_has_no_solver_failures_nominal_case():
    """Full closed-loop integration test: a modest setpoint step under
    nominal (matched-model, no-disturbance, no-noise) conditions should
    run with zero solver failures."""
    scenario = Scenario(name="nmpc_nominal_step", t_final=1.0,
                         setpoint_changes=[(0.0, cfg["nominal_state"]["cB"]),
                                            (0.3, cfg["nominal_state"]["cB"] + 0.1)])
    run = run_closed_loop("nmpc", cfg, scenario, N=10)
    assert run["n_solver_failures"] == 0
