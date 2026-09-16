# Limitations

This document is deliberately explicit about what this project does NOT
show, what was deviated from the original request and why, and every
significant bug/instability found during development. Nothing here is
hidden in the README's success narrative.

## 1. Formulation-level limitations (known by design)

1. **No offset-free / disturbance-estimation extension.** The NMPC here
   is a "textbook" tracking NMPC: it has no augmented disturbance model,
   no state/disturbance estimator (e.g. Kalman filter), and no integral
   action of any kind. Experiment 2 shows the direct consequence: under a
   sustained unmeasured feed disturbance, NMPC can underperform a
   properly tuned PID and can carry more (though still individually
   modest) temperature-safety violations, because it has no persistent
   memory of past prediction error. A production NMPC deployment would
   normally add exactly this extension; it was deliberately left out here
   to keep the core optimization formulation (the actual point of the
   project) legible and to keep the project's scope bounded.

2. **No terminal cost or terminal constraint set.** Nominal closed-loop
   stability of this NMPC formulation is not formally proven anywhere in
   this repository -- it is only verified empirically, across every
   experiment, in simulation. This is a standard simplification in
   practical (as opposed to theoretical) NMPC work, but it is a real gap:
   a formal proof would require constructing a terminal region and
   terminal cost consistent with the nonlinear dynamics, which is a
   substantial undertaking in its own right for a system with this
   kinetics/energy-balance coupling.

3. **Full-state feedback assumed for NMPC.** `Plant.measure_full_state()`
   gives the NMPC direct (optionally noisy) access to all four states,
   including `cA` and the jacket temperature `TK`, which in a real plant
   would typically require either extra instrumentation or a state
   estimator (e.g. an EKF/UKF, or a soft sensor -- interestingly, the
   author's separate "Physics-Guided Soft Sensing" project addresses
   exactly this kind of problem, but is a different project with a
   different purpose; see the README for the explicit distinction). PID,
   by contrast, only ever sees a noisy `cB` measurement, which is a more
   realistic (and harder) information setting -- this asymmetry favours
   NMPC and is disclosed rather than hidden.

4. **Soft safety constraint, not a hard guarantee.** `T_safe_max` is
   enforced via a slack variable with a large penalty (see
   `docs/mathematical_formulation.md` Section 4.3). This guarantees NLP
   feasibility but not that `T` never exceeds the limit in the closed
   loop -- and indeed it does, under large disturbances (Experiments 2,
   3). This is standard practical NMPC design, but it means this
   controller cannot honestly be described as providing a hard safety
   guarantee.

5. **PID is single-loop (manipulates only `F`).** This was a deliberate
   fairness/scope decision (see `pid_controller.py` docstring), not an
   oversight -- but it does mean the PID baseline has strictly less
   control authority than NMPC (one manipulated variable instead of two).
   A 2x2 decentralized PID (e.g. with a second loop pairing `Qk` to `T`)
   was considered and explicitly rejected because tuning/pairing such a
   system is itself a research question that would muddy what should be
   credited to NMPC specifically.

6. **Single reactor operating region tested.** All experiments start from
   the same nominal operating point (or the steady-state-optimizer's
   interior point in Experiment 3) with setpoint/disturbance excursions
   of similar order of magnitude. The mismatch and noise sweeps
   (Experiments 6, 7) vary one axis at a time; a combined
   "mismatch AND large disturbance AND high noise simultaneously" stress
   test was not run and would very plausibly show worse degradation than
   any single-axis result reported here.

## 2. Real bugs and instabilities found and fixed during development

These are documented in detail because they are directly relevant to
interview questions about numerical robustness in NMPC, and because
hiding them would violate this project's explicit anti-fabrication
requirement.

### 2.1 PyYAML scientific-notation parsing

`config/experiment_config.yaml` originally wrote large/small numbers as
`1.287e12`. PyYAML's default (1.1-compatible) resolver does not
recognize this as a float without an explicit sign after `e` (`1.287e+12`
is required) -- it silently parsed as a **string**, which later caused a
`TypeError` deep inside NMPC cost-function construction
(`can't multiply sequence by non-int of type 'MX'`). Fixed by rewriting
every scientific-notation value in the config with an explicit `+`.
Caught immediately because it fails loudly (`TypeError`) the first time
the affected value is used arithmetically -- worth flagging because a
similarly-malformed config value elsewhere (one YAML never happens to use
arithmetically until much later) would fail silently for longer.

### 2.2 NumPy 2.0 removed `np.trapz`

`metrics.py`'s IAE/ISE functions originally called `np.trapz`, which was
removed (renamed to `np.trapezoid`) in the NumPy version installed in
this environment. Fixed by resolving whichever name is available at
import time (`_trapz = getattr(np, "trapezoid", None) or getattr(np,
"trapz")`), so the code works across NumPy versions.

### 2.3 Badly-tuned initial PID baseline

The first PID tuning (`Kc=6, tau_I=0.35h`) gave >1.5h settling time and a
persistent offset that had not converged even after 6 simulated hours on
the Experiment-1 setpoint step. This would have made the PID-vs-NMPC
comparison look far more favourable to NMPC than a fair comparison
warrants. Caught by inspecting the actual closed-loop trajectory (not
just the summary IAE number) before accepting the baseline, and fixed by
a small simulation-based gain sweep (final: `Kc=20, tau_I=0.15h`).

### 2.4 NMPC state-domain NaNs (Arrhenius blow-up)

The first NMPC formulation placed no bounds at all on the predicted
states beyond the constraints that were explicitly part of the physical
problem (input bounds, `T<=T_safe_max+slack`, `cB>=0`). Under a large
disturbance, IPOPT's internal line search evaluated the dynamics at
trial iterates where absolute temperature approached zero, producing
NaN/Inf in the Arrhenius exponential terms and poisoning the NLP
Jacobian (`CasADi WARNING: NaN detected for output g/jac_g_x`). Fixed
with (a) generous numerical domain bounds on every predicted state
(`0<=cA,cB<=20`, `20<=T,TK<=250`, clearly documented as numerical
guards, not process constraints) and (b) a 1 K floor on absolute
temperature inside `kinetics.rate_constants` before the exponential is
evaluated. Neither change affects behaviour anywhere in the physically
meaningful operating range.

### 2.5 ~40% NMPC solver failures during disturbance rejection

Even after fix 2.4, roughly 40% of NMPC solves during the
disturbance-rejection experiment failed to converge
(`Infeasible_Problem_Detected`, `Maximum_Iterations_Exceeded`,
`Restoration_Failed`). Two hypotheses were tested and both were largely
ruled out:
- **Poor variable scaling** (`Qk` ~10^3-10^4 vs. `cA/cB` ~1 vs. `T`
  ~10^2): added explicit scaling of the `Qk` decision variable
  (`QK_SCALE=1000`); this is good practice and was kept, but it did NOT
  reduce the failure rate (still 48/120 failures).
- **IPOPT line-search options** (`mu_strategy=adaptive`,
  `honor_original_bounds=yes`): tried and found to make results *worse*
  in this case (more failures, larger constraint violations); reverted.

Verbose IPOPT diagnostics (`ipopt.print_level=5`) on a specific failing
step showed dozens of restoration-phase ("r"-suffixed) iterations
followed by "Converged to a point of local infeasibility. Problem may be
infeasible." The actual root cause was the internal model's RK4
discretization being too coarse (2 substeps per 0.05h control interval)
relative to the fast temperature dynamics during an aggressive transient
-- the multiple-shooting equality constraints became numerically
inconsistent for IPOPT's Newton iterations to satisfy, even though a
feasible point of the true continuous problem always exists (via the
slack variable). Increasing to 4 RK4 substeps in the internal model
eliminated all solver failures (0/120) across every subsequent
experiment. See `docs/mathematical_formulation.md` Section 4.4 for the
full formulation-level explanation.

### 2.6 A genuinely unstable weight combination (true-plant divergence)

While sweeping NMPC weights to fix 2.5's oscillatory behaviour, one
combination (`w_dQk=4e-6, w_slack=5e4`) caused the TRUE PLANT's own RK4
integration to diverge to non-finite (`NaN`/overflow) values -- this is
not a solver-convergence issue but a genuine numerical/physical
runaway of the underlying exothermic reactor under a badly-tuned
controller (a real CSTR thermal-runaway-like phenomenon). The first time
this happened, it surfaced as an opaque, unhandled CasADi assertion error
several call-frames away from the actual cause
(`opti.set_value` complaining the value being set was not "regular",
i.e. contained a NaN, because the *previous* step's plant state had
already diverged). This was fixed properly rather than avoided: `Plant.step()`
now checks `np.isfinite` after every fine RK4 substep and raises a clearly
named `PlantDivergedError` with the offending time and input, which
`simulation.run_closed_loop()` catches, flags in the returned run
(`run["diverged"] = True`), and `metrics.summarize_run()` reports as a
divergence rather than computing misleading numerical metrics (e.g. the
astronomically large IAE, `~9.7e82`, that first exposed this bug) on a
partially-completed trajectory. This weight combination is not used
anywhere in the final experiments, but the guard remains in the codebase
as a permanent safety net, and its existence -- and the fact that it was
triggered during weight tuning -- is reported here rather than removed
from history.

### 2.7 Steady-state optimizer landing exactly on both actuator bounds

The first version of `steady_state_optimizer.py` (a pure "maximize cB"
objective with no cost terms) found an optimum at exactly `F=F_max,
Qk=Qk_min`. Starting NMPC from that same point in Experiment 3 produced a
trajectory numerically identical to the open-loop baseline (to several
decimal places) -- with literally zero spare control authority in either
direction, there was nothing for a dynamic controller to do differently.
Recognized as a formulation problem, not a numerics problem, and fixed by
adding small economic cost terms on both manipulated variables (Section
5 of `mathematical_formulation.md`) so the optimum is a genuine interior
point.

## 3. Scope boundaries (explicitly out of scope)

- **No embedded/code-generated solver.** IPOPT via CasADi's general NLP
  interface was used throughout (as the project brief requires); a real
  embedded deployment would likely use a code-generated QP/NLP solver
  (e.g. acados, or CasADi's C code generation) for tighter real-time
  guarantees and lower per-solve latency. Experiment 8's feasibility
  conclusion (350x margin at a 3-minute control interval) would need to
  be re-derived for a much faster control loop.
- **No real plant data.** Every parameter in this project is a
  simulation-scale benchmark value (see `mathematical_formulation.md`
  Section 1.3); nothing here has been validated against, or claims to
  represent, measurements from an actual industrial reactor.
- **Single reactor, single reaction network.** Generalization to other
  CSTR configurations, other reaction networks, or plant-wide control
  (multiple interacting units) was not attempted.

## 4. What would need to change before applying this approach to a real
   chemical plant

1. Add a disturbance/state estimator (e.g. augmented EKF/UKF with an
   integrating disturbance model) so the controller has offset-free
   tracking and does not rely on full, low-noise state feedback.
2. Establish a terminal cost/terminal constraint set (or otherwise
   formally verify) for closed-loop stability guarantees, rather than
   relying on empirical simulation checks alone.
3. Replace the general-purpose IPOPT NLP solve with a code-generated or
   otherwise real-time-certified solver, and re-validate computational
   feasibility at the plant's actual required control frequency (which
   may be much faster than the 3-minute interval used here).
4. Re-identify all kinetic and heat-transfer parameters from real plant
   data (with proper uncertainty quantification -- see Experiment 7 for
   why this matters) rather than using benchmark literature values.
5. Harden the safety-constraint handling: a soft/slack-based safety
   constraint, as used here, is appropriate for a simulation study but
   would need independent, hard-wired safety-instrumented-system (SIS)
   layers in a real plant, entirely separate from the NMPC's own
   constraint handling.
6. Extensive commissioning-phase testing against the specific failure
   modes documented in Section 2 above (numerical domain guards, solver
   robustness under fast transients, weight tuning against genuine
   disturbance scenarios, not just nominal setpoint tracking) before any
   real deployment.
