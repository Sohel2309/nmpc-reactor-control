"""
simulation.py
--------------
Closed-loop (and open-loop) simulation harness. This module is the single
place where a controller is connected to the TRUE PLANT (plant_model.Plant)
and driven through a scenario (setpoint changes + feed disturbances +
noise). Every experiment script in scripts/ builds a `Scenario` and calls
`run_closed_loop`, so PID, steady-state-optimization, and NMPC are always
evaluated through exactly the same code path -- required for a fair
comparison (see README "Fair Comparison Requirements").
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

from .plant_model import Plant, PlantDivergedError
from .pid_controller import PIDController
from .nmpc_controller import NMPCController
from .dynamics import apply_mismatch
from .constraints import clip_physical, check_violations


@dataclass
class Scenario:
    name: str
    t_final: float
    setpoint_changes: list = field(default_factory=list)   # [(t, cB_sp), ...]
    feed_disturbances: list = field(default_factory=list)  # [(t, {"cA0": .., "T_in": ..}), ...]
    process_noise_std: float = 0.0
    meas_noise_std: float = 0.0
    seed: int = 42

    def setpoint_at(self, t):
        sp = self.setpoint_changes[0][1]
        for (tc, val) in self.setpoint_changes:
            if t >= tc:
                sp = val
        return sp

    def plant_params_at(self, t, base_params):
        p = dict(base_params)
        for (tc, overrides) in self.feed_disturbances:
            if t >= tc:
                p.update(overrides)
        return p


def run_closed_loop(controller_type: str, cfg: dict, scenario: Scenario,
                     x0=None, u0=None, N: int = None,
                     internal_mismatch: dict = None,
                     fixed_input=None):
    """
    Run one closed-loop (or open-loop, for controller_type='openloop')
    simulation of `scenario` against the true plant.

    controller_type: 'pid' | 'nmpc' | 'openloop'
        'openloop' applies `fixed_input` = [F, Qk] for the whole run
        (used for the steady-state-optimization baseline, Experiment 3).
    N: NMPC prediction horizon (required if controller_type == 'nmpc').
    internal_mismatch: dict of multipliers passed to dynamics.apply_mismatch
        to build the NMPC's internal model (defaults to cfg's
        internal_model_mismatch, i.e. matched / zero mismatch, if None).
    """
    plant_params = cfg["plant"]
    dt_c = cfg["simulation"]["dt_control"]
    dt_p = cfg["simulation"]["dt_plant"]

    if x0 is None:
        x0 = np.array([cfg["nominal_state"][k] for k in ["cA", "cB", "T", "TK"]])
    if u0 is None:
        u0 = np.array([cfg["nominal_input"]["F"], cfg["nominal_input"]["Qk"]])

    rng = np.random.default_rng(scenario.seed)
    plant = Plant(plant_params, x0, dt_p,
                  process_noise_std=scenario.process_noise_std, rng=rng)

    controller = None
    if controller_type == "pid":
        controller = PIDController(cfg, u0)
    elif controller_type == "nmpc":
        mismatch = internal_mismatch if internal_mismatch is not None \
            else cfg.get("internal_model_mismatch", {})
        internal_params = apply_mismatch(plant_params, mismatch)
        controller = NMPCController(internal_params, cfg, N=N)
    elif controller_type == "openloop":
        assert fixed_input is not None, "openloop requires fixed_input=[F,Qk]"
    else:
        raise ValueError(f"unknown controller_type {controller_type}")

    n_steps = int(round(scenario.t_final / dt_c))
    t_log = [0.0]
    x_log = [x0.copy()]
    u_log = [u0.copy()]
    sp_log = [scenario.setpoint_at(0.0)]
    solve_times = []
    n_solver_failures = 0
    viol_log = {"T_violation": [], "cB_violation": [], "F_violation": [],
                "Qk_violation": [], "dF_violation": [], "dQk_violation": []}

    u_prev = u0.copy()
    t = 0.0
    diverged = False
    diverged_reason = None
    for step in range(n_steps):
        # Update plant parameters for any active feed disturbance
        plant.params = scenario.plant_params_at(t, plant_params)

        sp = scenario.setpoint_at(t)

        if controller_type == "pid":
            meas = plant.measure(scenario.meas_noise_std)
            u_cmd = controller.compute(sp, meas, dt_c)
        elif controller_type == "nmpc":
            x_hat = plant.measure_full_state(scenario.meas_noise_std)
            u_cmd, info = controller.solve(x_hat, u_prev, sp)
            solve_times.append(info["solve_time"])
            if not info["success"]:
                n_solver_failures += 1
        else:  # openloop
            u_cmd = np.array(fixed_input, dtype=float)

        try:
            x_next = plant.step(u_cmd, dt_c)
        except PlantDivergedError as e:
            diverged = True
            diverged_reason = str(e)
            break
        x_next = clip_physical(x_next)
        plant.state = x_next

        viol = check_violations(x_next, u_cmd, u_prev, cfg)
        for k in viol_log:
            viol_log[k].append(viol[k])

        t += dt_c
        t_log.append(t)
        x_log.append(x_next.copy())
        u_log.append(u_cmd.copy())
        sp_log.append(scenario.setpoint_at(t))

        u_prev = u_cmd

    run = {
        "name": scenario.name,
        "controller_type": controller_type,
        "N": N,
        "t": np.array(t_log),
        "x": np.array(x_log),
        "u": np.array(u_log),
        "setpoint": np.array(sp_log),
        "violations": viol_log,
        "solve_times": np.array(solve_times) if controller_type == "nmpc" else None,
        "n_solver_failures": n_solver_failures,
        "cfg": cfg,
        "diverged": diverged,
        "diverged_reason": diverged_reason,
    }
    return run
