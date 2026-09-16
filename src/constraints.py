"""
constraints.py
--------------
Centralised definition of the process/actuator/safety constraints used by
every controller and by the metrics/evaluation code, so that the *same*
numeric limits are enforced and reported everywhere (no controller is
allowed to define its own private notion of "safe").

Constraint set
--------------
Manipulated variables (hard, actuator):
    F_min  <= F  <= F_max
    Qk_min <= Qk <= Qk_max

Manipulated-variable rate limits (hard, actuator slew rate):
    |F_k - F_{k-1}|   <= dF_max
    |Qk_k - Qk_{k-1}| <= dQk_max

State / safety constraints:
    T <= T_safe_max          (hard ceiling -- runaway protection)
    cB >= cB_min              (physical realizability, >= 0)

`T_safe_max` is treated as a SOFT constraint inside the NMPC formulation
(implemented via a slack variable with a large penalty, see
nmpc_controller.py) because a hard infeasible constraint on a predicted
state can make the finite-horizon NLP infeasible under disturbances/noise
that classical MPC theory does not guarantee recursive feasibility
against; using a slacked constraint is the standard, defensible
engineering choice (see docs/mathematical_formulation.md) while still
counting and reporting any activation of it as a "violation" in the
evaluation metrics.
"""

from __future__ import annotations
import numpy as np


def clip_input(u, cfg):
    """Clip a manipulated-variable vector [F, Qk] to its hard bounds."""
    c = cfg["constraints"]
    F = np.clip(u[0], c["F_min"], c["F_max"])
    Qk = np.clip(u[1], c["Qk_min"], c["Qk_max"])
    return np.array([F, Qk])


def clip_rate(u, u_prev, cfg):
    """Clip the requested move (u - u_prev) to the rate limits, then clip
    the resulting absolute input to its hard bounds."""
    c = cfg["constraints"]
    dF = np.clip(u[0] - u_prev[0], -c["dF_max"], c["dF_max"])
    dQk = np.clip(u[1] - u_prev[1], -c["dQk_max"], c["dQk_max"])
    u_rate_limited = np.array([u_prev[0] + dF, u_prev[1] + dQk])
    return clip_input(u_rate_limited, cfg)


def clip_physical(x):
    """Prevent concentrations from becoming non-physical (negative) after
    numerical integration with noise; used only as a numerical safety net,
    never to hide true model behaviour."""
    x = np.asarray(x, dtype=float).copy()
    x[0] = max(x[0], 0.0)
    x[1] = max(x[1], 0.0)
    return x


def check_violations(x, u, u_prev, cfg):
    """
    Given a realised state x=[cA,cB,T,TK], applied input u=[F,Qk], and the
    previously applied input u_prev, return a dict describing any
    constraint violations at this single time step (used to accumulate
    Experiment-4/safety metrics across a run).
    """
    c = cfg["constraints"]
    out = {
        "T_violation": max(0.0, x[2] - c["T_safe_max"]),
        "cB_violation": max(0.0, c["cB_min"] - x[1]),
        "F_violation": max(0.0, c["F_min"] - u[0]) + max(0.0, u[0] - c["F_max"]),
        "Qk_violation": max(0.0, c["Qk_min"] - u[1]) + max(0.0, u[1] - c["Qk_max"]),
        "dF_violation": max(0.0, abs(u[0] - u_prev[0]) - c["dF_max"]),
        "dQk_violation": max(0.0, abs(u[1] - u_prev[1]) - c["dQk_max"]),
    }
    out["any_violation"] = any(v > 1e-9 for v in out.values())
    return out
