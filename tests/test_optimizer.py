import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.steady_state_optimizer import solve_steady_state_optimum
from src.dynamics import cstr_rhs
from src.config import load_config

cfg = load_config()
P = cfg["plant"]


def test_ssopt_converges():
    x_star, u_star, info = solve_steady_state_optimum(P, cfg)
    assert info["success"] is True
    assert info["return_status"] == "Solve_Succeeded"


def test_ssopt_solution_is_a_genuine_steady_state():
    x_star, u_star, info = solve_steady_state_optimum(P, cfg)
    dx = cstr_rhs(x_star, u_star, P)
    assert max(abs(v) for v in dx) < 1e-4


def test_ssopt_respects_input_bounds():
    x_star, u_star, info = solve_steady_state_optimum(P, cfg)
    c = cfg["constraints"]
    assert c["F_min"] - 1e-6 <= u_star[0] <= c["F_max"] + 1e-6
    assert c["Qk_min"] - 1e-6 <= u_star[1] <= c["Qk_max"] + 1e-6


def test_ssopt_respects_safety_constraint():
    x_star, u_star, info = solve_steady_state_optimum(P, cfg)
    c = cfg["constraints"]
    assert x_star[2] <= c["T_safe_max"] + 1e-6


def test_ssopt_outperforms_nominal_point_in_objective():
    """The optimizer must find a cB at least as good as the arbitrarily
    chosen nominal operating point used to initialise closed-loop runs
    (otherwise the 'optimization' would be pointless)."""
    x_star, u_star, info = solve_steady_state_optimum(P, cfg)
    assert x_star[1] >= cfg["nominal_state"]["cB"] - 1e-6
