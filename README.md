# Dynamic Optimization and Model Predictive Control of a Nonlinear Chemical Reactor

**Designed and implemented a nonlinear Model Predictive Controller (NMPC)
for a Van-de-Vusse-type CSTR, benchmarked against classical PID control
and open-loop steady-state optimization, with quantified
setpoint-tracking, disturbance-rejection, and computational-cost
trade-offs.**

This repository is a from-scratch, tested, and actually-run simulation
study -- not a wrapper around an existing MPC library, not a notebook,
and not a set of claims without numbers behind them. Every figure and
table in `results/` was produced by running the code in this repository;
none were hand-typed or estimated. See `docs/limitations.md` for a
detailed, honest account of every bug, instability, and re-tuning
decision made while building it, and `docs/experiments.md` for the
per-experiment results and discussion.

## 1. Project Overview

Three control/optimization strategies are implemented and fairly
compared on the identical nonlinear reactor, true plant, initial
conditions, setpoint trajectories, and disturbance sequences:

1. **PID** -- a classical, properly-tuned single-loop feedback controller.
2. **Steady-state optimization** -- an open-loop economic operating-point
   optimizer with no feedback.
3. **NMPC** -- constrained nonlinear model predictive control, solved as
   a genuine receding-horizon problem with CasADi + IPOPT.

The engineering question this project answers:

> Given a nonlinear chemical reactor with operating constraints and
> disturbances, can constrained NMPC achieve better process performance
> than conventional PID control, and what computational/robustness
> trade-offs does that improvement introduce?

**Short answer** (full discussion in `docs/experiments.md`): yes, clearly,
for nominal setpoint tracking (Experiment 1: ~5.5x lower tracking error)
and for coordinated multi-input disturbance handling with active
constraints (Experiments 3, 4); but **not unconditionally** -- under a
large, sustained, unmeasured disturbance, a properly tuned PID is
competitive and in one experiment (Experiment 2) has lower tracking error
than this NMPC, because this NMPC deliberately has no offset-free /
disturbance-estimation extension (see Limitations). Computationally, this
NMPC is comfortably feasible at the simulated 3-minute control interval
(worst-case solve time is 507 ms, a 354x margin -- Experiment 8), and
degrades gracefully (not catastrophically) under both measurement noise
(Experiment 6) and up to +/-50% kinetic-parameter model mismatch
(Experiment 7).

## 2. Engineering Motivation

Steady-state process optimization (find the best operating point) and
classical PID feedback control are both mature, well-understood tools,
but neither is designed to jointly (a) react quickly and optimally to
disturbances, (b) respect multiple simultaneous state and input
constraints, and (c) coordinate multiple manipulated variables toward one
objective. NMPC is designed to do exactly this, at the cost of solving a
nonlinear optimization problem at every control interval. This project
exists to demonstrate, quantitatively and honestly (including where it
does NOT help), when that trade-off is worthwhile for a representative
nonlinear reactor.

## 3-6. Chemical Process, Reaction Network, Mathematical Model, Assumptions

See **`docs/mathematical_formulation.md`** for the complete, exact
formulation: the Van de Vusse reaction network (A->B->C, 2A->D),
Arrhenius kinetics, the full material and energy balance ODEs (including
the jacket energy balance), the verified nominal steady-state operating
point, and every constraint with its numeric value. Summary:

- **States**: `cA`, `cB` (mol/L), `T`, `TK` (°C, reactor & jacket temp).
- **Manipulated variables**: `F` (dilution rate, h⁻¹), `Qk` (jacket heat
  duty, kJ/h).
- **Disturbances**: feed concentration `cA0` and feed temperature `T_in`
  step changes; measurement and process noise.
- All physical parameters are the standard Klatt & Engell (1998) Van de
  Vusse CSTR benchmark values -- **simulation-scale values for a
  benchmark reactor, not measurements from a real industrial unit.**

## 7. Control Problem

- **Controlled variable**: `cB` (product concentration), tracked to a
  setpoint.
- **Safety variable**: `T` (reactor temperature), must stay under
  `T_safe_max = 145 °C`.
- **Manipulated variables**: `F` and `Qk`, both bounded and rate-limited.
- **Disturbances**: `cA0`, `T_in` step changes; Gaussian measurement and
  process noise.

Full constraint set (input bounds, rate limits, safety limit) is in
`docs/mathematical_formulation.md` Section 2 and `src/constraints.py`.

## 8. Optimization Formulation (NMPC)

Multiple-shooting NLP, RK4-discretized internal model, quadratic
tracking + move-suppression + soft-safety-slack objective, solved with
CasADi/IPOPT, receding horizon (only the first move is ever applied).
Full formulation: `docs/mathematical_formulation.md` Section 4.

## 9. PID Methodology

Single-loop PID on `cB` -> `F`, with conditional-integration anti-windup,
rate/saturation limits, tuned via a simulation-based gain sweep (not
shipped with a lazy/weak tuning -- see `docs/mathematical_formulation.md`
Section 3 for the tuning process, including the first, too-sluggish
tuning that was caught and rejected).

## 10. NMPC Methodology

See Section 8 above / `docs/mathematical_formulation.md` Section 4,
including the internal-model-vs-true-plant separation that makes the
model-mismatch experiment (Experiment 7) genuine rather than fabricated.

## 11. Constraints

```
3   <= F  <= 35    [h^-1]              |F_k - F_{k-1}|  <= 8     [h^-1/step]
-9000 <= Qk <= 0   [kJ/h]              |Qk_k - Qk_{k-1}| <= 3000 [kJ/h/step]
T <= 145 C  (soft, slack-penalized in NMPC; hard-checked in all metrics)
cB >= 0
```

## 12. Experimental Design & 13. Results

Eight experiments, each run against the identical true plant with fair,
documented, shared conditions (see Section 14, "Fair Comparison"):

| # | Experiment | Headline result |
|---|---|---|
| 1 | Setpoint tracking | NMPC IAE 5.5x lower than tuned PID |
| 2 | Disturbance rejection | PID competitive / better here -- reported honestly |
| 3 | SS-opt vs. dynamic control | NMPC tracks disturbed target 37% better than open loop |
| 4 | Constraint activity | Aggressive case: F saturated 75% of run, T held within 1.8 °C of limit |
| 5 | Horizon sensitivity | IAE improves 25% from N=5->20; solve time grows ~linearly |
| 6 | Noise robustness | PID control effort explodes 15x under high noise; NMPC's does not |
| 7 | Model mismatch (required) | Clean, monotonic V-shaped degradation; stable to +/-50% mismatch |
| 8 | Computational feasibility | Worst-case solve 507 ms vs. 180 s interval (354x margin) |

**Full numeric tables, plots, and per-experiment discussion:
`docs/experiments.md`** and `results/tables/*.csv`, `results/plots/*.png`.

## 14. Fair Comparison Requirements

Every experiment uses: the same true plant (`src/plant_model.py`, always
nominal parameters), the same initial conditions, the same setpoint and
disturbance trajectories (`src/scenarios.py`, shared by every script),
the same random seed for a given comparison, and the same evaluation
window and metric definitions (`src/metrics.py`). PID was re-tuned after
an initial tuning was found to be unfairly weak (Section 9). NMPC's
weights were tuned against the disturbance scenario specifically because
an initial, more aggressive tuning produced unsafe oscillatory behaviour
under disturbance while looking great on setpoint tracking alone --
tuning against only the "easy" experiment would have been an unfair
comparison by omission. Where PID outperforms NMPC (Experiment 2), this
is reported as the headline finding of that experiment, not minimized.

## 15. Robustness Analysis

Noise robustness (Experiment 6) and model-mismatch robustness (Experiment
7, required) are both dedicated experiments with their own sections in
`docs/experiments.md`. Additional robustness-relevant findings that
emerged during development (solver failures under fast disturbance
transients, a genuine plant-divergence instability under one weight
combination) are documented in full in `docs/limitations.md` Section 2,
since they are as informative about this NMPC's robustness properties as
the planned experiments.

## 16. Computational Analysis

Experiment 8 (`docs/experiments.md`) plus the horizon-sensitivity data in
Experiment 5. Bottom line: this NMPC, solved with general-purpose
CasADi/IPOPT (no code generation, no embedded solver) on ordinary
hardware, is comfortably real-time feasible at the simulated 3-minute
control interval, with hundreds-of-times margin.

## 17. Limitations

**See `docs/limitations.md` for the complete, itemized list.** Headline
items: no offset-free/disturbance-estimation extension (directly
responsible for Experiment 2's result), no formal terminal-stability
guarantee, full-state feedback assumed for NMPC (vs. PID's single noisy
measurement), soft (not hard-guaranteed) safety constraint, and every
significant bug found and fixed during development (YAML parsing,
NumPy API change, NaN/domain guards, solver-robustness root-causing, and
one genuine plant-divergence instability) is documented with its
diagnostic trail rather than silently fixed and forgotten.

## 18. Reproducibility

```bash
git clone <this-repo> && cd nmpc_reactor_project
pip install -r requirements.txt

# Run the full test suite (54 tests, ~10-15s)
pytest tests/ -v

# Verify the nominal steady-state operating point
python scripts/find_steady_state.py

# Run any single experiment, e.g.:
python scripts/run_setpoint_experiment.py
python scripts/run_model_mismatch.py

# Or run everything (all 8 experiments + steady-state check), ~2-3 min:
python scripts/run_all_experiments.py
```

All experiments are deterministic given the fixed random seed
(`config/experiment_config.yaml: random_seed`, and each `Scenario` object
carries its own explicit `seed`); re-running any script reproduces its
CSV/plot outputs exactly (verified: `tests/test_reproducibility.py`).
Results in `results/` were generated by exactly the commands above, in
this environment, and are committed to the repository so they can be
inspected without re-running anything.

No GPU is required or used anywhere in this project.

## 19. Project Structure

```
nmpc_reactor_project/
├── README.md                        <- this file
├── requirements.txt
├── .gitignore
├── LICENSE
├── config/experiment_config.yaml    <- every numeric parameter, one place
├── data/                            <- scenario definitions (see data/README.md)
├── src/
│   ├── kinetics.py                  <- Arrhenius rate constants & rate laws
│   ├── dynamics.py                  <- material/energy balance ODEs (cstr_rhs)
│   ├── plant_model.py               <- TRUE PLANT: RK4 integrator, noise, divergence guard
│   ├── pid_controller.py            <- PID baseline
│   ├── steady_state_optimizer.py    <- economic steady-state optimizer (CasADi/IPOPT)
│   ├── nmpc_controller.py           <- NMPC (CasADi Opti + IPOPT, receding horizon)
│   ├── constraints.py               <- centralized constraint definitions/checking
│   ├── simulation.py                <- closed-loop simulation harness (Scenario, run_closed_loop)
│   ├── scenarios.py                 <- canonical scenario definitions, shared by all scripts
│   ├── metrics.py                   <- IAE/ISE/settling-time/overshoot/solver-time metrics
│   ├── evaluation.py                <- CSV/plot generation helpers
│   └── config.py                    <- YAML config loader
├── scripts/
│   ├── find_steady_state.py
│   ├── run_pid_baseline.py, run_nmpc.py            <- standalone demos
│   ├── run_setpoint_experiment.py                   <- Experiment 1
│   ├── run_disturbance_experiment.py                <- Experiment 2
│   ├── run_ssopt_baseline.py                        <- Experiment 3
│   ├── run_constraint_experiment.py                 <- Experiment 4
│   ├── run_horizon_sweep.py                         <- Experiment 5
│   ├── run_noise_robustness.py                      <- Experiment 6
│   ├── run_model_mismatch.py                        <- Experiment 7
│   ├── run_computational_feasibility.py             <- Experiment 8
│   └── run_all_experiments.py                       <- orchestrator
├── results/{plots,tables,raw}/      <- actual generated outputs (committed)
├── tests/                           <- 54 pytest tests, see "Testing" below
└── docs/
    ├── mathematical_formulation.md  <- complete formulation + tuning/design rationale
    ├── experiments.md               <- per-experiment results tables & discussion
    └── limitations.md               <- honest limitations + full bug/instability log
```

## Testing

```bash
pytest tests/ -v
```

54 tests across 8 files, covering: reaction kinetics (Arrhenius scaling),
material/energy balances (steady-state consistency, RK4 convergence
order, mismatch application), constraints (bounds, rate limits,
violation detection), the PID controller (saturation, rate limits,
closed-loop convergence), the steady-state optimizer (convergence,
feasibility, constraint satisfaction), the NMPC controller (objective
behaviour, constraint satisfaction, **genuine receding-horizon
behaviour** -- verified by checking that two different states produce
two different first-moves, not a replayed open-loop sequence --, and
model-mismatch producing genuinely different predictions), metrics
(closed-form checks against known analytical values), and
reproducibility (identical seeds -> bit-identical trajectories).

## Relation to "Physics-Guided Soft Sensing for Nonlinear Chemical Processes" (a separate project by the same author)

That project uses a similar Van-de-Vusse-type CSTR to demonstrate
**physics-guided machine learning, soft sensing, and uncertainty
quantification** (PLS vs. MLP vs. PGNN, split conformal prediction). This
project is a completely different engineering problem built around the
same family of reactor: **dynamic optimization, closed-loop control,
constraint handling, disturbance rejection, and controller computational
feasibility.** No modeling code, no soft-sensing logic, and no ML model
is shared between the two projects -- the only thing in common is that
both happen to use a Van de Vusse CSTR as their nonlinear chemical
process, because it is a standard, well-documented benchmark for exactly
these two very different kinds of study.

## Future Work

- Add an offset-free NMPC extension (augmented integrating-disturbance
  state estimator) and re-run Experiment 2 to quantify the improvement.
- Construct a terminal cost/terminal region for a formal nominal
  stability guarantee.
- Combine the model-mismatch and disturbance-rejection experiments (a
  stress test neither experiment alone captures).
- Explore a code-generated/embedded solver (e.g. acados) and re-run
  Experiment 8 at a much faster control interval.
- Extend the state estimator to something closer to what a real plant
  would provide (partial, noisy measurements only, with an explicit
  observer) rather than the full-state feedback assumption used here.


    absence is not just a theoretical gap but a measurable performance
    and safety-margin cost under a realistic disturbance.
