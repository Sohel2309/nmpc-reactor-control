"""
metrics.py
----------
Quantitative performance metrics computed from a closed-loop simulation
run (see simulation.py for the run structure). All metrics are computed
directly from the returned time-series arrays -- nothing here is
fabricated or hand-picked after the fact.
"""

from __future__ import annotations
import numpy as np

# NumPy >= 2.0 renamed trapz -> trapezoid; support both installed versions.
_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


def iae(t, error):
    return float(_trapz(np.abs(error), t))


def ise(t, error):
    return float(_trapz(error ** 2, t))


def mae(error):
    return float(np.mean(np.abs(error)))


def rmse(error):
    return float(np.sqrt(np.mean(error ** 2)))


def settling_time(t, y, setpoint, band=0.02, t_change=0.0):
    """
    Time (measured from t_change) for y to enter and REMAIN within
    +/- band (as a fraction of the setpoint step size, minimum absolute
    band of 0.02*|setpoint| ) of the setpoint, to the end of the window.
    Returns np.inf if it never settles within the given series.
    """
    mask = t >= t_change
    tt, yy = t[mask], y[mask]
    if len(tt) == 0:
        return float("inf")
    tol = max(abs(band * setpoint), 1e-6)
    within = np.abs(yy - setpoint) <= tol
    # find the last index where it's still outside the band; settling
    # time is just after that.
    if within.all():
        return float(tt[0] - t_change)
    outside_idx = np.where(~within)[0]
    last_outside = outside_idx[-1]
    if last_outside == len(tt) - 1:
        return float("inf")
    return float(tt[last_outside + 1] - t_change)


def overshoot(y, setpoint, y0):
    """Percent overshoot relative to the step size (y0 -> setpoint)."""
    step = setpoint - y0
    if abs(step) < 1e-9:
        return 0.0
    if step > 0:
        peak = np.max(y) - setpoint
    else:
        peak = setpoint - np.min(y)
    peak = max(peak, 0.0)
    return float(100.0 * peak / abs(step))


def control_effort(u_series):
    """Sum of squared control moves (total variation-adjacent effort metric)."""
    du = np.diff(u_series, axis=0)
    return float(np.sum(du ** 2))


def total_variation(u_series):
    du = np.diff(u_series, axis=0)
    return float(np.sum(np.abs(du)))


def saturation_time(u_series, u_min, u_max, dt, tol=1e-6):
    at_min = u_series <= (u_min + tol)
    at_max = u_series >= (u_max - tol)
    return float(np.sum(at_min | at_max) * dt)


def solver_time_stats(solve_times):
    arr = np.asarray(solve_times, dtype=float)
    if len(arr) == 0:
        return {"mean": None, "median": None, "p95": None, "max": None}
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def summarize_run(run: dict) -> dict:
    """
    Compute the full standard metric set for a single closed-loop run
    dict as produced by simulation.run_closed_loop.
    """
    t = run["t"]
    cB = run["x"][:, 1]
    T = run["x"][:, 2]
    sp = run["setpoint"]
    error = sp - cB

    out = {}
    out["diverged"] = bool(run.get("diverged", False))
    if out["diverged"]:
        # Do not compute misleading numerical metrics on a run whose plant
        # integration diverged partway through -- report the divergence
        # itself as the finding instead (see docs/limitations.md).
        out["diverged_reason"] = run.get("diverged_reason")
        out["diverged_at_h"] = float(t[-1]) if len(t) else 0.0
        out["segments"] = []
        out["overshoot_pct_max"] = None
        return out
    out["IAE"] = iae(t, error)
    out["ISE"] = ise(t, error)
    out["MAE"] = mae(error)
    out["RMSE"] = rmse(error)

    out["control_effort_F"] = control_effort(run["u"][:, 0])
    out["control_effort_Qk"] = control_effort(run["u"][:, 1])
    out["TV_F"] = total_variation(run["u"][:, 0])
    out["TV_Qk"] = total_variation(run["u"][:, 1])

    cfg = run["cfg"]
    c = cfg["constraints"]
    dt = cfg["simulation"]["dt_control"]
    out["sat_time_F"] = saturation_time(run["u"][:, 0], c["F_min"], c["F_max"], dt)
    out["sat_time_Qk"] = saturation_time(run["u"][:, 1], c["Qk_min"], c["Qk_max"], dt)

    viol = run["violations"]
    out["n_T_violations"] = int(np.sum(np.array(viol["T_violation"]) > 1e-9))
    out["max_T_violation"] = float(np.max(viol["T_violation"])) if len(viol["T_violation"]) else 0.0
    out["total_T_violation_duration_h"] = float(
        np.sum(np.array(viol["T_violation"]) > 1e-9) * dt)
    out["n_rate_violations"] = int(
        np.sum(np.array(viol["dF_violation"]) > 1e-9)
        + np.sum(np.array(viol["dQk_violation"]) > 1e-9))

    if "solve_times" in run and run["solve_times"] is not None and len(run["solve_times"]) > 0:
        st = solver_time_stats(run["solve_times"])
        out["solve_time_mean"] = st["mean"]
        out["solve_time_median"] = st["median"]
        out["solve_time_p95"] = st["p95"]
        out["solve_time_max"] = st["max"]
        out["n_solver_failures"] = int(run.get("n_solver_failures", 0))

    # Overshoot / settling time per distinct setpoint segment
    seg_boundaries = np.where(np.diff(sp) != 0)[0]
    seg_starts = [0] + list(seg_boundaries + 1)
    seg_metrics = []
    for i, s0 in enumerate(seg_starts):
        s1 = seg_starts[i + 1] if i + 1 < len(seg_starts) else len(t)
        sp_val = sp[s0]
        y0 = cB[s0 - 1] if s0 > 0 else cB[s0]
        seg_y = cB[s0:s1]
        seg_t = t[s0:s1]
        seg_metrics.append({
            "t_start": float(t[s0]),
            "setpoint": float(sp_val),
            "overshoot_pct": overshoot(seg_y, sp_val, y0),
            "settling_time_h": settling_time(seg_t, seg_y, sp_val, t_change=t[s0]),
        })
    out["segments"] = seg_metrics
    out["overshoot_pct_max"] = max((s["overshoot_pct"] for s in seg_metrics), default=0.0)

    return out
