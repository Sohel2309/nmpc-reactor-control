"""
pid_controller.py
------------------
Classical single-loop PID baseline.

Design choice: PID manipulates F (dilution rate) to track the cB setpoint,
and Qk is held at its nominal steady value. This mirrors common industrial
practice for a "simple baseline" controller on a MIMO process: a single
PID loop is paired with the manipulated variable that has the most direct
and monotonic steady-state effect on the primary controlled variable
(dcB/dF at the nominal point is large and single-signed over the operating
range used here; dcB/dQk is much weaker and only acts indirectly, through
temperature -> the Arrhenius rate constants). Using a second PID loop on
Qk to hold reactor temperature was considered and rejected for the
baseline: it would turn the "baseline" into a bespoke 2x2 decentralized
control system with its own pairing/tuning/interaction problems, which is
a research question in itself and would make the PID-vs-NMPC comparison
less clean about what is actually being credited to NMPC. This is
documented explicitly so the comparison is not accidentally unfair (see
README "Fair Comparison" section).

Controller form (velocity/incremental form, which gives natural
anti-windup when combined with actuator saturation):

    e_k = setpoint - measurement_k
    P_k = Kc * e_k
    I_k = I_{k-1} + Kc/tau_I * e_k * dt      (only accumulated while output
                                               is not saturated -> conditional
                                               integration anti-windup)
    D_k = Kc*tau_D * (e_k - e_{k-1}) / dt
    u_unsat = u_ss + P_k + I_k + D_k
    u = saturate_and_rate_limit(u_unsat)

Tuning method: an initial gain was estimated with a Ziegler-Nichols-style
open-loop reaction-curve test (see docs/mathematical_formulation.md for
the step-test data used), then refined by simulation-based manual tuning
against the same setpoint-tracking scenario used in Experiment 1, because
the process is nonlinear and a linear ZN rule alone under- or
over-shoots depending on operating point.
"""

from __future__ import annotations
import numpy as np
from .constraints import clip_input, clip_rate


class PIDController:
    def __init__(self, cfg: dict, u_ss):
        pid_cfg = cfg["pid"]
        self.Kc = pid_cfg["Kc"]
        self.tau_I = pid_cfg["tau_I"]
        self.tau_D = pid_cfg["tau_D"]
        self.u_ss = np.array(u_ss, dtype=float)  # [F_ss, Qk_ss]
        self.cfg = cfg

        self.integral = 0.0
        self.prev_error = None
        self.u_prev = self.u_ss.copy()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.u_prev = self.u_ss.copy()

    def compute(self, setpoint: float, measurement: float, dt: float):
        """One PID execution. Returns the applied (constrained) input [F, Qk]."""
        error = setpoint - measurement
        P = self.Kc * error

        if self.prev_error is None:
            D = 0.0
        else:
            D = self.Kc * self.tau_D * (error - self.prev_error) / dt

        # Tentative (pre anti-windup-check) integral update
        integral_candidate = self.integral + (self.Kc / self.tau_I) * error * dt
        F_unsat = self.u_ss[0] + P + integral_candidate + D

        c = self.cfg["constraints"]
        saturated = F_unsat < c["F_min"] or F_unsat > c["F_max"]

        # Conditional-integration anti-windup: only integrate if doing so
        # does not push further into saturation.
        if not saturated or (error > 0 and F_unsat < c["F_min"]) or \
           (error < 0 and F_unsat > c["F_max"]):
            self.integral = integral_candidate

        F_cmd = self.u_ss[0] + P + self.integral + D
        u_cmd = np.array([F_cmd, self.u_ss[1]])  # Qk held at nominal

        u_applied = clip_rate(u_cmd, self.u_prev, self.cfg)

        self.prev_error = error
        self.u_prev = u_applied
        return u_applied
