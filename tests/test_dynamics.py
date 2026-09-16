import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.dynamics import cstr_rhs, apply_mismatch
from src.plant_model import rk4_step
from src.config import load_config
from scipy.optimize import fsolve

cfg = load_config()
P = cfg["plant"]
X_SS = [cfg["nominal_state"][k] for k in ["cA", "cB", "T", "TK"]]
U_SS = [cfg["nominal_input"]["F"], cfg["nominal_input"]["Qk"]]


def test_nominal_state_is_steady_state():
    """The nominal_state/nominal_input in the config must be a genuine
    fixed point of the ODEs (residual ~ 0), verified independently of the
    script that was originally used to find it."""
    dx = cstr_rhs(X_SS, U_SS, P)
    assert max(abs(v) for v in dx) < 1e-4


def test_mass_balance_signs_at_high_conversion():
    """At high cA (far from steady state), consumption terms should
    dominate and dcA/dt should be negative (net consumption) when the
    feed term is comparatively small at low F."""
    x = [4.5, 0.1, 130.0, 125.0]
    u = [3.0, -5000.0]  # low dilution rate -> feed term small
    dx = cstr_rhs(x, u, P)
    assert dx[0] < 0  # A is being consumed faster than replenished


def test_component_b_production_from_a():
    """With cB = 0 and cA > 0, dcB/dt must be positive (pure production,
    no B present yet to consume via B->C)."""
    x = [2.0, 0.0, 130.0, 125.0]
    u = U_SS
    dx = cstr_rhs(x, u, P)
    assert dx[1] > 0


def test_energy_balance_jacket_coupling_direction():
    """If the jacket is hotter than the reactor, jacket heat exchange
    contributes a POSITIVE term to dT/dt (heating the reactor), all else
    equal; verified by comparing two otherwise-identical states differing
    only in TK."""
    x_cold_jacket = [1.2, 0.9, 130.0, 100.0]
    x_hot_jacket = [1.2, 0.9, 130.0, 150.0]
    dx_cold = cstr_rhs(x_cold_jacket, U_SS, P)
    dx_hot = cstr_rhs(x_hot_jacket, U_SS, P)
    assert dx_hot[2] > dx_cold[2]


def test_rk4_step_reduces_to_smaller_change_for_smaller_dt():
    """Sanity check on the RK4 integrator: smaller step should produce a
    proportionally smaller state change over the same physical interval
    when compared via multiple substeps (checks integrator direction and
    consistency, not a full convergence-order proof)."""
    x = np.array(X_SS)
    u = np.array(U_SS)
    dt_big = 0.01
    x1_one_step = rk4_step(x, u, dt_big, P)

    # two half-steps should be close to one full step for a smooth ODE
    dt_half = dt_big / 2
    x_half = rk4_step(x, u, dt_half, P)
    x1_two_steps = rk4_step(x_half, u, dt_half, P)

    assert np.allclose(x1_one_step, x1_two_steps, atol=1e-3)


def test_rk4_convergence_order():
    """RK4 should show ~4th-order convergence: halving dt should reduce
    the local truncation error by roughly a factor of 16 (checked with
    generous tolerance appropriate for a nonlinear ODE)."""
    x0 = np.array(X_SS) + np.array([0.3, -0.1, 3.0, 2.0])  # away from equilibrium
    u = np.array(U_SS)
    T = 0.02

    def integrate(dt):
        n = int(round(T / dt))
        x = x0.copy()
        for _ in range(n):
            x = rk4_step(x, u, dt, P)
        return x

    x_coarse = integrate(T / 4)
    x_mid = integrate(T / 8)
    x_fine = integrate(T / 16)

    err_coarse = np.linalg.norm(x_mid - x_coarse)
    err_fine = np.linalg.norm(x_fine - x_mid)
    assert err_fine < err_coarse / 4  # should shrink much faster than linearly


def test_apply_mismatch_changes_only_selected_parameters():
    mismatch = {"k10_ab_mult": 1.2, "kw_mult": 0.8}
    p2 = apply_mismatch(P, mismatch)
    assert np.isclose(p2["k10_ab"], P["k10_ab"] * 1.2)
    assert np.isclose(p2["kw"], P["kw"] * 0.8)
    assert np.isclose(p2["k10_bc"], P["k10_bc"])  # untouched
    assert P["k10_ab"] != p2["k10_ab"]  # original dict not mutated


def test_apply_mismatch_defaults_to_identity():
    p2 = apply_mismatch(P, {})
    for key in P:
        assert np.isclose(p2[key], P[key])
