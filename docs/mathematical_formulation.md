# Mathematical Formulation

This document gives the complete, exact mathematical formulation implemented
in this repository: the process model, the three control/optimization
strategies, and the numerical methods used to solve each of them.

## 1. Reaction network and process description

The process is a continuously-stirred tank reactor (CSTR) running the
**Van de Vusse reaction network**:

```
A --k1--> B --k2--> C          (series reaction, B is the desired product)
2A --k3--> D                    (parallel, second-order side reaction)
```

This is a classical nonlinear process-control benchmark (Van de Vusse,
1964; the jacketed, non-isothermal version with the parameter set used
here follows Klatt & Engell, 1998, and is the same benchmark used in many
NMPC papers and in do-mpc's example library). It is chosen because:

- it is a genuinely nonlinear reactor (second-order side reaction,
  Arrhenius kinetics, exothermic heat effects coupled to a jacket), so a
  linear controller cannot be exactly optimal across the operating range;
- **the input-output steady-state map from feed rate to product
  concentration is non-monotonic** in the isothermal version of this
  benchmark (a classical textbook illustration of why nonlinear process
  control is hard) -- with the energy balance included here the
  temperature feedback changes this map, but the process retains strong
  nonlinearity and multiple simultaneously-relevant constraints, which is
  what this project needs;
- it is well documented in the literature, so the modelling choices here
  can be independently checked against published sources rather than
  taken on faith.

### 1.1 States and inputs

| Symbol | Units | Description |
|---|---|---|
| `cA` | mol/L | concentration of A in the reactor |
| `cB` | mol/L | concentration of B (desired product) in the reactor |
| `T`  | °C | reactor (liquid) temperature |
| `TK` | °C | cooling-jacket temperature |
| `F`  | h⁻¹ | dilution rate = feed volumetric flow / reactor volume |
| `Qk` | kJ/h | heat duty applied to the jacket (negative = net cooling) |

`F` and `Qk` are the two manipulated variables. `cB` is the primary
controlled variable (the thing we want at a setpoint); `T` is a
controlled *safety* variable (must stay under a ceiling).

### 1.2 Reaction kinetics

All three reactions are Arrhenius-temperature-dependent:

```
k1(T) = k10_ab * exp(-EA_ab_R / T_K)      [A -> B]
k2(T) = k10_bc * exp(-EA_bc_R / T_K)      [B -> C]
k3(T) = k10_ad * exp(-EA_ad_R / T_K)      [2A -> D]
```

where `T_K = T + 273.15` is the absolute reactor temperature. Rate laws:

```
r1 = k1 * cA          (mol/(L h))
r2 = k2 * cB
r3 = k3 * cA^2
```

(`kinetics.py`)

### 1.3 Material and energy balances (the "true plant" ODEs)

```
dcA/dt = F*(cA0 - cA) - r1 - r3

dcB/dt = -F*cB + r1 - r2

dT/dt  = F*(T_in - T)
         + (kw*AR)/(rho*Cp*VR) * (TK - T)
         - (1/(rho*Cp)) * (r1*dH_ab + r2*dH_bc + r3*dH_ad)

dTK/dt = (1/(mk*CPK)) * (Qk + kw*AR*(T - TK))
```

(`dynamics.py::cstr_rhs`). All physical parameter values (`k10_*`,
`EA_*_R`, `dH_*`, `rho`, `Cp`, `kw`, `AR`, `VR`, `mk`, `CPK`, `cA0`,
`T_in`) are listed with sources in `config/experiment_config.yaml`; they
are the standard Klatt & Engell (1998) benchmark values and are
**simulation-scale parameters for a benchmark reactor, not measurements
from a real industrial unit** -- this is stated once here and is not
repeated as a disclaimer throughout the rest of the documentation, but it
applies everywhere.

### 1.4 Nominal operating point

The nominal state used to initialise every simulation is a numerically
verified fixed point of the ODEs above (residual < 1e-12), found with
`scipy.optimize.fsolve` and independently re-checked by
`scripts/find_steady_state.py` (also exercised as
`tests/test_dynamics.py::test_nominal_state_is_steady_state`):

```
F = 14.19 h^-1, Qk = -5000 kJ/h
  -> cA = 1.1842 mol/L, cB = 0.8762 mol/L, T = 130.334 C, TK = 124.566 C
```

This point was chosen (over the first candidate the model produced) to
leave roughly 15 °C of headroom to `T_safe_max = 145 °C`, so that upward
setpoint changes are meaningful experiments rather than immediately
hitting a safety limit at t=0.

## 2. Constraints

All constraints are centralised in `constraints.py` and are the same set
used by every controller and every metric:

```
F_min  = 3     <= F  <= F_max  = 35      [h^-1]
Qk_min = -9000 <= Qk <= Qk_max = 0       [kJ/h]

|F_k  - F_{k-1}|  <= dF_max  = 8         [h^-1 per 0.05 h control interval]
|Qk_k - Qk_{k-1}| <= dQk_max = 3000      [kJ/h per control interval]

T  <= T_safe_max = 145 C                  (safety ceiling)
cB >= cB_min = 0                          (physical realizability)
```

These are explicitly **simulation-scale limits for this benchmark
reactor**, not real industrial operating limits, chosen to be tight
enough that they are actually relevant to the experiments (Experiment 4
requires this).

`T_safe_max` is enforced as a **soft** constraint inside the NMPC (a
slack variable with a large quadratic penalty), for reasons explained in
Section 4.3. It remains a genuine hard boundary for what counts as a
safety violation in `metrics.py`/`constraints.check_violations` --
softening only changes what the *solver* is allowed to do internally
while searching, not what counts as a safety violation when the results
are reported.

## 3. PID baseline

**Design decision**: PID manipulates only `F` to track `cB`; `Qk` is held
at its nominal value. See `pid_controller.py`'s module docstring for the
full justification (in short: pairing a single SISO baseline with the
manipulated variable that has the strongest, most monotonic steady-state
effect on the controlled variable is standard practice, and adding a
second PID loop on `Qk` would turn "the baseline" into its own bespoke
2x2 decentralized-control research question, which would make the
PID-vs-NMPC comparison less clean about what should be credited to NMPC).

**Form** (velocity/incremental-style with conditional-integration
anti-windup):

```
e_k = setpoint - measurement_k
P_k = Kc * e_k
I_k = I_{k-1} + (Kc/tau_I) * e_k * dt        [only integrated when not
                                               driving further into
                                               saturation]
D_k = Kc*tau_D * (e_k - e_{k-1}) / dt
F_unsaturated = F_ss + P_k + I_k + D_k
F_applied = rate_limit(saturate(F_unsaturated))
```

**Tuning**: An initial gain estimate from an open-loop step-test /
Ziegler-Nichols-style reaction-curve heuristic (`Kc=6, tau_I=0.35h`)
produced unacceptably slow, offset-heavy closed-loop behaviour on the
Experiment-1 setpoint step (>1.5 h to approach the new setpoint, still not
converged after 6 h in one case) -- this was caught, not shipped, because
an artificially weak baseline would violate the project's fair-comparison
requirement. The final tuning (`Kc=20, tau_I=0.15h, tau_D=0.01h`) was
selected by a small simulation-based gain sweep against the same scenario
used in Experiment 1 (see git history / development notes in
`docs/limitations.md`), giving ~1.6 h settling with no overshoot.

## 4. Nonlinear Model Predictive Control (NMPC)

Implemented in `nmpc_controller.py` with CasADi's `Opti` interface and
IPOPT as the NLP solver.

### 4.1 Formulation

Multiple shooting over a prediction horizon of `N` control intervals
(`dt = 0.05 h` each), discretized with 4 RK4 substeps per interval using
the controller's **internal model** (see Section 4.4 for why 4, not
fewer):

```
minimize_{U_0..U_{N-1}, X_1..X_N, s_1..s_N}
    sum_{k=0}^{N-1} [ w_cB * (cB_{k+1} - cB_sp)^2
                       + w_dF * (F_k - F_{k-1})^2
                       + w_dQk * (Qk_k - Qk_{k-1})^2 ]
    + w_slack * sum_{k=1}^{N} s_k^2

subject to, for k = 0..N-1:
    X_{k+1} = F_RK4(X_k, U_k ; internal_model_params)     [multiple shooting]
    F_min  <= F_k  <= F_max
    Qk_min <= Qk_k <= Qk_max
    -dF_max  <= F_k  - F_{k-1}  <= dF_max
    -dQk_max <= Qk_k - Qk_{k-1} <= dQk_max
    cB_{k+1} >= cB_min
    T_{k+1}  <= T_safe_max + s_{k+1},   s_{k+1} >= 0        [soft safety constraint]
    0 <= cA_{k+1}, cB_{k+1} <= 20                            [numerical domain bound]
    20 <= T_{k+1}, TK_{k+1} <= 250                           [numerical domain bound]
    X_0 = x_hat                                              [current state estimate]
    U_{-1} = u_prev                                          [previously applied input]
```

Only the first optimal input `U_0` is applied to the plant; the plant is
advanced one control interval, a new state estimate `x_hat` is obtained,
and the whole NLP is re-solved -- this is genuine receding-horizon
control (verified in `tests/test_nmpc.py::test_nmpc_receding_horizon_uses_only_first_move_and_replans`).

### 4.2 Weights (final, after tuning -- see Section 4.5)

```
w_cB    = 80        (tracking weight)
w_dF    = 0.02       (F move-suppression)
w_dQk   = 6.0e-6     (Qk move-suppression, applied to PHYSICAL kJ/h moves)
w_slack = 2.0e+4      (soft safety-constraint penalty)
```

### 4.3 Why the safety constraint is soft

A hard constraint on a *predicted* state under disturbances/noise/model
mismatch can make the finite-horizon NLP infeasible at some control
step, with no well-defined fallback. Softening it with a slack variable
and a large quadratic penalty guarantees the NLP always has a feasible
point (hold current inputs is always feasible, and any resulting
temperature excursion is merely expensive, not infeasible), at the cost
of no longer being able to make an absolute guarantee that `T` never
exceeds `T_safe_max` in the closed loop. This project does NOT hide that
trade-off: Experiments 2, 3 and 4 report genuine, nonzero `T_safe_max`
violations under sufficiently large disturbances, exactly because the
constraint is soft and the internal model does not know about the
disturbance in advance.

### 4.4 Internal model discretization: why 4 RK4 substeps, not 2

This is one of the few implementation details in this repository that
was changed **after** it caused a measurable failure, so it is documented
in detail here (see `docs/limitations.md` for the full diagnostic trail).
With only 2 RK4 substeps per 0.05 h control interval (i.e. 0.025 h per
substep), IPOPT reported `Infeasible_Problem_Detected` /
`Restoration_Failed` on roughly 40% of control steps during the
disturbance-rejection experiment specifically when the reactor needed to
respond quickly to a large, fast disturbance. Diagnosis (verbose IPOPT
output, `ipopt.print_level=5`) showed IPOPT entering a long restoration
phase and terminating at "a point of local infeasibility" -- this was
**not** a truly infeasible continuous problem (a feasible point always
exists because of the slack variable), but a *numerically* inconsistent
discretization: the multiple-shooting equality constraints
`X_{k+1} = F_RK4(X_k, U_k)` were too coarse relative to the fast
temperature dynamics at those operating points for IPOPT's Newton
iterations to satisfy well. Two other hypotheses were tested and ruled
out first: (1) poor variable scaling of `Qk` (fixed anyway, since it is
good practice regardless, but did not fix the failures), and (2) IPOPT
line-search options (`mu_strategy=adaptive`, `honor_original_bounds`),
which did not help and in one configuration made results worse.
Increasing to 4 substeps eliminated all solver failures across every
experiment in this repository (`n_solver_failures = 0` everywhere except
where a robustness experiment deliberately explores extreme conditions).

### 4.5 Weight tuning process

The final weights above were **not** the first weights tried. An
initial, more aggressive tuning (`w_dQk=2e-6, w_slack=1e5`) achieved
excellent nominal setpoint-tracking performance but, when tested under
the disturbance-rejection scenario (Experiment 2), produced a genuinely
oscillatory, safety-relevant closed loop (`T` reaching up to 152.6 °C
against a 145 °C limit, `Qk` swinging by its full rate limit repeatedly).
A grid sweep over `(w_dQk, w_slack)` combinations against the
disturbance scenario (while also checking Experiment 1 was not badly
degraded) was used to select the final weights, which give zero `T`
violations on the standard disturbance scenario while keeping
setpoint-tracking IAE roughly 2-3x better than the tuned PID baseline.
One combination explored during this sweep
(`w_dQk=4e-6, w_slack=5e4`) caused the TRUE PLANT's own RK4 integration to
diverge to non-finite values (a genuine reactor-runaway-like numerical
instability, not merely a solver warning) -- this is now caught cleanly
by `plant_model.PlantDivergedError` instead of crashing with an opaque
assertion error, and is reported as a finding in `docs/limitations.md`
rather than hidden.

### 4.6 Warm-starting

The previous NLP solution, shifted by one time step (with the tail
repeated), is used as the initial guess for the next solve
(`NMPCController._warm_X` / `_warm_U`). This standard receding-horizon
warm-start noticeably reduces solve time on subsequent calls (observed:
first solve of a run ~50-400 ms, subsequent warm-started solves in the
same regime often <50 ms) and was left as the default rather than made
configurable, since cold-starting every step is not representative of
how any real NMPC deployment would run.

## 5. Steady-state (economic) optimizer

Formulated and solved with CasADi/IPOPT in `steady_state_optimizer.py`:

```
minimize_{x, u}   -cB + w_energy*(Qk/1000)^2 + w_feed*(F/35)^2
subject to        cstr_rhs(x, u; plant_params) = 0     [steady state]
                   F_min <= F <= F_max
                   Qk_min <= Qk <= Qk_max
                   T <= T_safe_max
                   cB >= cB_min
```

**Why the energy/feed cost terms are necessary, not just economically
nice-to-have**: a pure "maximize cB" objective (no cost terms) was found
to push the optimum to the exact corner of the actuator bounds
(`F=F_max, Qk=Qk_min`), leaving **zero spare control authority**. When
this was first tried, the open-loop (steady-state-input) and NMPC
(closed-loop) trajectories in Experiment 3 turned out numerically
identical to 4 decimal places, because NMPC quite literally cannot do
anything differently from a controller with no available control action
left. Adding small quadratic costs on `Qk` and `F` (`w_energy=2.0`,
`w_feed=0.3`, chosen by a manual sweep to land at an interior point close
to the nominal operating region) restored genuine control headroom, and
Experiment 3 shows real, meaningfully different behaviour between the two
strategies as a result.

## 6. Numerical integration

- **True plant** (`plant_model.py`): fixed-step explicit RK4 at
  `dt_plant = 0.005 h`, substepped within each `dt_control = 0.05 h`
  control interval (10 fine steps per control interval). Convergence
  order (~4th order, halving step size) is checked in
  `tests/test_dynamics.py::test_rk4_convergence_order`.
- **NMPC internal model**: RK4 with 4 substeps per control interval (see
  Section 4.4).
- **Numerical domain guard**: `kinetics.rate_constants` floors the
  absolute temperature at 1 K before evaluating the Arrhenius exponent.
  This has no effect anywhere in the physically meaningful range of this
  reactor (hundreds of Kelvin above the floor) -- it exists purely so
  that a transient IPOPT trial iterate exploring an unphysical region
  during line search cannot produce NaN/Inf that poisons the NLP
  Jacobian (this was observed empirically before the guard was added; see
  `docs/limitations.md`).
