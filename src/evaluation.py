"""
evaluation.py
--------------
Shared helpers for the experiment scripts in scripts/: writing metric
tables to CSV, and generating the standard set of plots. Kept separate
from metrics.py (pure numeric metric computation) so that the plotting
backend / matplotlib usage is isolated from the numerics.
"""

from __future__ import annotations
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "results"))
PLOTS_DIR = os.path.join(RESULTS_DIR, "plots")
TABLES_DIR = os.path.join(RESULTS_DIR, "tables")
RAW_DIR = os.path.join(RESULTS_DIR, "raw")

for d in (PLOTS_DIR, TABLES_DIR, RAW_DIR):
    os.makedirs(d, exist_ok=True)


def write_csv(rows: list, fieldnames: list, filename: str):
    path = os.path.join(TABLES_DIR, filename)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})
    return path


def save_run_raw(run: dict, filename: str):
    """Persist a run's raw time series as a .npz for later inspection."""
    path = os.path.join(RAW_DIR, filename)
    np.savez(path, t=run["t"], x=run["x"], u=run["u"], setpoint=run["setpoint"],
              solve_times=run["solve_times"] if run["solve_times"] is not None else np.array([]))
    return path


def plot_tracking_comparison(runs: dict, title: str, filename: str, cB_ylim=None):
    """
    runs: dict of {label: run_dict}. Plots cB tracking, T, F, Qk for each
    controller overlaid, sharing the same setpoint/disturbance timeline.
    """
    fig, axes = plt.subplots(4, 1, figsize=(9, 11), sharex=True)
    colors = {"PID": "tab:blue", "NMPC": "tab:red", "Open-loop (ss-opt)": "tab:green"}

    sp_plotted = False
    for label, run in runs.items():
        t = run["t"]
        color = colors.get(label, None)
        axes[0].plot(t, run["x"][:, 1], label=label, color=color)
        if not sp_plotted:
            axes[0].plot(t, run["setpoint"], "k--", label="Setpoint", linewidth=1)
        axes[1].plot(t, run["x"][:, 2], label=label, color=color)
        axes[2].plot(t, run["u"][:, 0], label=label, color=color)
        axes[3].plot(t, run["u"][:, 1], label=label, color=color)
    sp_plotted = True

    T_safe = None
    for run in runs.values():
        T_safe = run["cfg"]["constraints"]["T_safe_max"]
        break
    if T_safe is not None:
        axes[1].axhline(T_safe, color="k", linestyle=":", linewidth=1, label="T_safe_max")

    axes[0].set_ylabel("cB [mol/L]")
    axes[1].set_ylabel("T [deg C]")
    axes[2].set_ylabel("F [1/h]")
    axes[3].set_ylabel("Qk [kJ/h]")
    axes[3].set_xlabel("time [h]")
    if cB_ylim:
        axes[0].set_ylim(*cB_ylim)
    for ax in axes:
        ax.legend(loc="best", fontsize=8)
        ax.grid(alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, filename)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_horizon_sweep(results: list, filename: str):
    """results: list of dicts with keys N, IAE, solve_time_mean, solve_time_max"""
    Ns = [r["N"] for r in results]
    iae_vals = [r["IAE"] for r in results]
    mean_t = [r["solve_time_mean"] * 1000 for r in results]
    max_t = [r["solve_time_max"] * 1000 for r in results]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(Ns, iae_vals, "o-")
    axes[0].set_xlabel("Prediction horizon N")
    axes[0].set_ylabel("IAE (tracking error)")
    axes[0].set_title("Tracking performance vs horizon")
    axes[0].grid(alpha=0.3)

    axes[1].plot(Ns, mean_t, "o-", label="mean solve time")
    axes[1].plot(Ns, max_t, "s-", label="max solve time")
    axes[1].set_xlabel("Prediction horizon N")
    axes[1].set_ylabel("Solve time [ms]")
    axes[1].set_title("Computational cost vs horizon")
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, filename)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_degradation(x_values: list, y_values_by_label: dict, xlabel: str, ylabel: str,
                      title: str, filename: str):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for label, y in y_values_by_label.items():
        ax.plot(x_values, y, "o-", label=label)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, filename)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_solver_time_histogram(solve_times: np.ndarray, dt_control: float, filename: str):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.hist(np.array(solve_times) * 1000, bins=40, color="tab:red", alpha=0.8)
    ax.axvline(dt_control * 3600 * 1000, color="k", linestyle="--",
               label=f"control interval = {dt_control * 3600:.0f} s")
    ax.set_xlabel("NMPC solve time [ms]")
    ax.set_ylabel("count")
    ax.set_title("NMPC per-step solve time distribution")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(PLOTS_DIR, filename)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path
