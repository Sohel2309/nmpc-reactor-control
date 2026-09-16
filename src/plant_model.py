"""
plant_model.py
--------------
The TRUE PLANT: a fixed-step explicit RK4 integrator of the Van de Vusse
CSTR ODEs (dynamics.cstr_rhs), always using the *true* physical parameter
set from the config file's `plant` section.

This is deliberately a plain NumPy object with no symbolic/optimization
machinery, and no controller is allowed to call into it during its own
prediction step -- controllers only ever see plant STATE via
`Plant.state` (optionally corrupted by measurement noise) and only ever
affect the plant by calling `Plant.step(u)`. This physical separation is
what makes the model-mismatch experiment (Experiment 7) meaningful: the
NMPC's internal prediction model is a *different* Python object built
from perturbed parameters (see dynamics.apply_mismatch), never the
Plant object itself.

Explicit RK4 (not an adaptive/implicit integrator) is used for the true
plant because:
  * it is simple, deterministic, and easy to defend/verify by hand,
  * the CSTR states here are not extremely stiff at the chosen dt_plant
    (verified in tests/test_dynamics.py by step-halving convergence),
  * using a different integration technology for "plant truth" than for
    the NMPC's discrete prediction model (which uses multiple-shooting
    RK4 steps built directly into the CasADi NLP) keeps the two code
    paths independent, avoiding an implementation bug being invisible
    simply because both controller and "reality" share one function call.
"""

from __future__ import annotations
import numpy as np
from .dynamics import cstr_rhs, N_STATES


def rk4_step(x, u, dt, params):
    """One explicit RK4 integration step of cstr_rhs, plain NumPy."""
    x = np.asarray(x, dtype=float)
    k1 = np.asarray(cstr_rhs(x, u, params), dtype=float)
    k2 = np.asarray(cstr_rhs(x + 0.5 * dt * k1, u, params), dtype=float)
    k3 = np.asarray(cstr_rhs(x + 0.5 * dt * k2, u, params), dtype=float)
    k4 = np.asarray(cstr_rhs(x + dt * k3, u, params), dtype=float)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


class PlantDivergedError(RuntimeError):
    """
    Raised by Plant.step when the TRUE PLANT's own numerical integration
    produces a non-finite (NaN/Inf) state. This is deliberately NOT
    silently caught: a nonlinear exothermic CSTR with strong Arrhenius
    feedback genuinely CAN be driven into a runaway/blow-up condition by
    a sufficiently aggressive or badly tuned controller (this was
    observed empirically while tuning the NMPC weights -- see
    docs/limitations.md). Rather than crash with an opaque CasADi/NumPy
    assertion deep in a downstream call, the plant detects this directly
    and raises a clearly named exception so the calling experiment script
    can catch it, log which scenario/controller/tuning caused it, and
    report the finding honestly instead of silently producing garbage
    metrics (e.g. the astronomical IAE values that first exposed this
    during development).
    """
    pass


class Plant:
    """
    Ground-truth Van de Vusse CSTR. Integrated with fixed-step RK4 at
    `dt_plant` (finer than the controller sampling interval `dt_control`)
    to resolve the fast temperature dynamics accurately regardless of how
    coarsely a given controller samples the process.
    """

    def __init__(self, params: dict, x0, dt_plant: float,
                 process_noise_std: float = 0.0, rng: np.random.Generator = None):
        self.params = dict(params)
        self.state = np.asarray(x0, dtype=float).copy()
        self.dt_plant = dt_plant
        self.process_noise_std = process_noise_std
        self.rng = rng if rng is not None else np.random.default_rng()
        self.t = 0.0
        self.history = {"t": [0.0], "x": [self.state.copy()]}

    def step(self, u, dt_control: float):
        """
        Advance the plant by one controller sampling interval `dt_control`,
        holding the manipulated input `u` constant (zero-order hold), by
        taking `n_sub` fine RK4 substeps of size `dt_plant`.

        Optional additive process noise (a small Gaussian perturbation on
        the concentration states at each fine substep, representing
        unmodelled disturbances / mixing imperfections) is injected when
        `process_noise_std > 0`, and is clipped so that concentrations
        cannot be forced non-physical (see constraints.clip_physical).
        """
        n_sub = max(1, int(round(dt_control / self.dt_plant)))
        dt = dt_control / n_sub
        x = self.state.copy()
        for _ in range(n_sub):
            x = rk4_step(x, u, dt, self.params)
            if self.process_noise_std > 0:
                noise = self.rng.normal(0.0, self.process_noise_std, size=2)
                x[0] = max(0.0, x[0] + noise[0])
                x[1] = max(0.0, x[1] + noise[1])
            if not np.all(np.isfinite(x)):
                raise PlantDivergedError(
                    f"True plant state became non-finite at t={self.t:.4f} h "
                    f"(applied u={list(u)}). This indicates genuine numerical/"
                    f"physical divergence (reactor runaway), not merely a "
                    f"solver warning -- see docs/limitations.md."
                )
            self.t += dt
            self.history["t"].append(self.t)
            self.history["x"].append(x.copy())
        self.state = x
        return self.state.copy()

    def measure(self, meas_noise_std: float = 0.0):
        """Return a (possibly noisy) measurement of cB, the controlled variable."""
        cB_true = self.state[1]
        if meas_noise_std > 0:
            return cB_true + self.rng.normal(0.0, meas_noise_std)
        return cB_true

    def measure_full_state(self, meas_noise_std: float = 0.0):
        """Return a (possibly noisy) measurement of the full state vector.
        Used by NMPC, which needs cA/T/TK as well as cB (see
        docs/limitations.md for the resulting state-estimation assumption)."""
        x = self.state.copy()
        if meas_noise_std > 0:
            x = x + self.rng.normal(0.0, meas_noise_std, size=N_STATES) * \
                np.array([0.3, 1.0, 0.05, 0.05])  # relative noise scaling by state
        return x
