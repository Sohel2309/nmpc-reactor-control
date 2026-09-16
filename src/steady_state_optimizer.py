"""
steady_state_optimizer.py
--------------------------
Baseline 2: a steady-state (economic) operating-point optimizer. This is
NOT a feedback controller -- it solves, once, for the input (F, Qk) and
resulting steady state that maximizes a simple process objective subject
to the steady-state process equations and the same operating/safety
constraints as everything else in this project, and then that single
input is applied open-loop for the whole simulation.

Its purpose in this project is entirely to demonstrate (Experiment 3)
that finding a good *steady-state* operating point is a fundamentally
different problem from *dynamic, closed-loop* disturbance rejection: once
a feed disturbance or setpoint change occurs, the open-loop optimum input
no longer corresponds to the new best operating point and there is no
mechanism to correct for that until a human/optimizer re-solves it.

Optimization problem
---------------------
    maximize   cB_ss(F, Qk) - w_energy*(Qk_ss/1000)^2   (production value
                                                           minus a simple
                                                           quadratic utility/
                                                           energy cost on
                                                           cooling duty)
    subject to F_min  <= F  <= F_max
               Qk_min <= Qk <= Qk_max
               T_ss(F, Qk) <= T_safe_max
               cA_ss, cB_ss, T_ss, TK_ss = steady_state(F, Qk)   [from cstr_rhs = 0]

Solved with CasADi/IPOPT: the steady-state equations are enforced as
equality constraints on the state at a single collocation "point" (a
0-length horizon), which reuses exactly the same symbolic `cstr_rhs` as
the plant/NMPC, so the optimizer is solving on a *model* consistent with
the rest of the project (and can equally be run with the mismatched
internal model to reproduce Experiment 7 style questions for this
baseline if desired).
"""

from __future__ import annotations
import numpy as np
import casadi as ca
from .dynamics import cstr_rhs


def solve_steady_state_optimum(params: dict, cfg: dict, x_guess=None, u_guess=None):
    """
    Solve for the steady-state (x*, u*) that maximizes a simple economic
    objective -- production value of cB minus a utility/energy cost on
    the cooling duty |Qk| -- subject to the process equations at steady
    state and operating/safety constraints. Returns (x_star, u_star, info_dict).

    The energy penalty (cfg["economics"]["cooling_cost_weight"]) is
    IMPORTANT for Experiment 3's validity, not just economic realism: a
    pure "maximize cB" objective (zero energy cost) was found empirically
    to push the optimum to the exact actuator corner (F_max, Qk_min) --
    with NO remaining control authority, a dynamic controller started
    from that point literally cannot do anything differently than holding
    the same saturated inputs, which would make the open-loop-vs-NMPC
    comparison in Experiment 3 vacuous (both trajectories were found to
    be numerically identical when this was first tried, see
    docs/limitations.md). A small quadratic cooling-cost term moves the
    optimum to an interior point, leaving genuine control headroom.
    """
    c = cfg["constraints"]
    w_energy = cfg.get("economics", {}).get("cooling_cost_weight", 0.0)

    x = ca.MX.sym("x", 4)   # cA, cB, T, TK
    u = ca.MX.sym("u", 2)   # F, Qk

    rhs = cstr_rhs(x, u, params)
    g_ss = ca.vertcat(*rhs)  # steady state: rhs == 0

    z = ca.vertcat(x, u)
    # Objective: maximize cB, minus a quadratic cooling-utility cost AND
    # a quadratic feed-rate (raw-material/pumping) cost -- without some
    # cost on F as well, F alone always saturates at F_max regardless of
    # the cooling cost (more feed always makes more product), which
    # would again leave zero control headroom on that axis.
    w_feed = cfg.get("economics", {}).get("feed_cost_weight", 0.0)
    f = -x[1] + w_energy * (u[1] / 1000.0) ** 2 + w_feed * (u[0] / 35.0) ** 2

    lbg = [0.0] * 4
    ubg = [0.0] * 4
    # Safety constraint T <= T_safe_max, and cB >= cB_min, enforced directly
    g_extra = ca.vertcat(x[2], x[1])
    lbg_extra = [-ca.inf, c["cB_min"]]
    ubg_extra = [c["T_safe_max"], ca.inf]

    g = ca.vertcat(g_ss, g_extra)
    lbg = lbg + lbg_extra
    ubg = ubg + ubg_extra

    lbz = [0.0, c["cB_min"], -ca.inf, -ca.inf, c["F_min"], c["Qk_min"]]
    ubz = [ca.inf, ca.inf, c["T_safe_max"], ca.inf, c["F_max"], c["Qk_max"]]

    if x_guess is None:
        x_guess = [1.2, 0.9, 130.0, 125.0]
    if u_guess is None:
        u_guess = [14.0, -5000.0]
    z0 = list(x_guess) + list(u_guess)

    nlp = {"x": z, "f": f, "g": g}
    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes"}
    solver = ca.nlpsol("ssopt", "ipopt", nlp, opts)

    sol = solver(x0=z0, lbx=lbz, ubx=ubz, lbg=lbg, ubg=ubg)
    z_star = np.array(sol["x"]).flatten()
    x_star, u_star = z_star[:4], z_star[4:]

    stats = solver.stats()
    info = {"success": stats["success"], "return_status": stats["return_status"]}
    return x_star, u_star, info
