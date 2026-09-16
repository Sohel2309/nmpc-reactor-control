"""
scenarios.py
------------
Canonical scenario definitions used across the experiment scripts, so
that "the same setpoint trajectory" / "the same disturbance sequence"
requirement (README, Fair Comparison Requirements) is satisfied by
construction: every script imports these instead of redefining its own
numbers.
"""

from .simulation import Scenario


def setpoint_scenario(cfg, seed=42):
    """Experiment 1: two setpoint changes (up, then down), no disturbance,
    low measurement noise only (a small amount of realistic sensor noise
    is present in all scenarios except the explicit noise-free unit tests,
    since a perfectly noise-free measurement is not physically realistic)."""
    cB0 = cfg["nominal_state"]["cB"]
    return Scenario(
        name="setpoint_tracking",
        t_final=cfg["simulation"]["t_final"],
        setpoint_changes=[(0.0, cB0), (1.0, cB0 + 0.15), (3.5, cB0 - 0.10)],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )


def disturbance_scenario(cfg, seed=42):
    """Experiment 2: constant setpoint, feed concentration and feed
    temperature disturbances introduced partway through the run."""
    cB0 = cfg["nominal_state"]["cB"]
    cA0_nom = cfg["plant"]["cA0"]
    Tin_nom = cfg["plant"]["T_in"]
    return Scenario(
        name="disturbance_rejection",
        t_final=cfg["simulation"]["t_final"],
        setpoint_changes=[(0.0, cB0)],
        feed_disturbances=[
            (1.5, {"cA0": cA0_nom * 1.15}),          # +15% feed concentration step
            (3.5, {"cA0": cA0_nom * 1.15, "T_in": Tin_nom + 8.0}),  # + feed temp step
        ],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )


def constraint_activity_scenario_mild(cfg, seed=42):
    """Experiment 4 (inactive-constraint case): a small setpoint increase
    that stays well within the temperature safety limit."""
    cB0 = cfg["nominal_state"]["cB"]
    return Scenario(
        name="constraint_inactive",
        t_final=3.0,
        setpoint_changes=[(0.0, cB0), (0.5, cB0 + 0.08)],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )


def constraint_activity_scenario_aggressive(cfg, seed=42):
    """Experiment 4 (active-constraint case): an aggressive setpoint push
    that drives the reactor temperature close to / against T_safe_max,
    combined with a feed disturbance that adds heat, to genuinely exercise
    constraint handling rather than making it decorative."""
    cB0 = cfg["nominal_state"]["cB"]
    Tin_nom = cfg["plant"]["T_in"]
    return Scenario(
        name="constraint_active",
        t_final=3.0,
        setpoint_changes=[(0.0, cB0), (0.5, cB0 + 0.35)],
        feed_disturbances=[(1.5, {"T_in": Tin_nom + 10.0})],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )


def horizon_sweep_scenario(cfg, seed=42):
    """Experiment 5: a moderately aggressive setpoint step used to
    differentiate horizon lengths (too short a horizon should visibly
    under-perform on this scenario)."""
    cB0 = cfg["nominal_state"]["cB"]
    return Scenario(
        name="horizon_sweep",
        t_final=2.5,
        setpoint_changes=[(0.0, cB0), (0.4, cB0 + 0.20)],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )


def noise_robustness_scenario(cfg, meas_noise_std, process_noise_std, seed=42):
    """Experiment 6: fixed setpoint step, varying noise levels."""
    cB0 = cfg["nominal_state"]["cB"]
    return Scenario(
        name=f"noise_meas{meas_noise_std}_proc{process_noise_std}",
        t_final=3.0,
        setpoint_changes=[(0.0, cB0), (0.5, cB0 + 0.15)],
        meas_noise_std=meas_noise_std,
        process_noise_std=process_noise_std,
        seed=seed,
    )


def mismatch_scenario(cfg, seed=42):
    """Experiment 7: fixed setpoint step + a disturbance, used to probe
    model-mismatch tolerance."""
    cB0 = cfg["nominal_state"]["cB"]
    cA0_nom = cfg["plant"]["cA0"]
    return Scenario(
        name="model_mismatch",
        t_final=3.0,
        setpoint_changes=[(0.0, cB0), (0.5, cB0 + 0.15)],
        feed_disturbances=[(2.0, {"cA0": cA0_nom * 1.10})],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )


def computational_feasibility_scenario(cfg, seed=42):
    """Experiment 8: a long run combining setpoint changes and
    disturbances, to gather a large, representative sample of solver
    call timings for the feasibility analysis."""
    cB0 = cfg["nominal_state"]["cB"]
    cA0_nom = cfg["plant"]["cA0"]
    return Scenario(
        name="computational_feasibility",
        t_final=cfg["simulation"]["t_final"],
        setpoint_changes=[(0.0, cB0), (1.0, cB0 + 0.15), (2.5, cB0 - 0.1),
                           (4.0, cB0 + 0.25)],
        feed_disturbances=[(3.0, {"cA0": cA0_nom * 1.1})],
        meas_noise_std=cfg["noise"]["cB_std_low"],
        seed=seed,
    )
