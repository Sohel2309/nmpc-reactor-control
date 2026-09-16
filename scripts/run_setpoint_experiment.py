"""
run_setpoint_experiment.py
----------------------------
EXPERIMENT 1 -- Baseline setpoint tracking: PID vs NMPC on identical
setpoint changes (same true plant, same initial state, same setpoint
trajectory, same measurement-noise realization/seed).
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import setpoint_scenario
from src.evaluation import write_csv, plot_tracking_comparison, save_run_raw


def main():
    cfg = load_config()
    scenario = setpoint_scenario(cfg)
    N = cfg["nmpc"]["N_default"]

    run_pid = run_closed_loop("pid", cfg, scenario)
    run_nmpc = run_closed_loop("nmpc", cfg, scenario, N=N)

    m_pid = summarize_run(run_pid)
    m_nmpc = summarize_run(run_nmpc)

    save_run_raw(run_pid, "exp1_pid.npz")
    save_run_raw(run_nmpc, "exp1_nmpc.npz")

    rows = []
    for label, m, run in [("PID", m_pid, run_pid), ("NMPC", m_nmpc, run_nmpc)]:
        rows.append({
            "controller": label,
            "IAE": round(m["IAE"], 5), "ISE": round(m["ISE"], 5),
            "MAE": round(m["MAE"], 5), "RMSE": round(m["RMSE"], 5),
            "overshoot_pct_max": round(m["overshoot_pct_max"], 3),
            "control_effort_F": round(m["control_effort_F"], 3),
            "control_effort_Qk": round(m["control_effort_Qk"], 1),
            "n_T_violations": m["n_T_violations"],
            "n_rate_violations": m["n_rate_violations"],
            "solve_time_mean_ms": round(m["solve_time_mean"] * 1000, 3) if "solve_time_mean" in m else "",
            "solve_time_max_ms": round(m["solve_time_max"] * 1000, 3) if "solve_time_max" in m else "",
        })
    path = write_csv(rows, list(rows[0].keys()), "experiment1_setpoint_tracking.csv")
    print("Wrote", path)

    plot_path = plot_tracking_comparison(
        {"PID": run_pid, "NMPC": run_nmpc},
        title="Experiment 1: Setpoint Tracking -- PID vs NMPC",
        filename="experiment1_setpoint_tracking.png",
    )
    print("Wrote", plot_path)

    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
