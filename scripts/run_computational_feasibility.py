"""
run_computational_feasibility.py
-----------------------------------
EXPERIMENT 8 -- Computational feasibility: gather NMPC solve-time
statistics (mean, median, p95, max, failure count) over a long,
representative run combining setpoint changes and disturbances, and
compare against the control sampling interval to assess real-time
feasibility.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import computational_feasibility_scenario
from src.evaluation import write_csv, plot_solver_time_histogram, save_run_raw


def main():
    cfg = load_config()
    N = cfg["nmpc"]["N_default"]
    dt_control_s = cfg["simulation"]["dt_control"] * 3600.0

    scenario = computational_feasibility_scenario(cfg)
    run = run_closed_loop("nmpc", cfg, scenario, N=N)
    m = summarize_run(run)
    save_run_raw(run, "exp8_nmpc_feasibility.npz")

    row = {
        "N": N,
        "control_interval_s": dt_control_s,
        "n_steps": len(run["solve_times"]),
        "mean_solve_ms": round(m["solve_time_mean"] * 1000, 3),
        "median_solve_ms": round(m["solve_time_median"] * 1000, 3),
        "p95_solve_ms": round(m["solve_time_p95"] * 1000, 3),
        "max_solve_ms": round(m["solve_time_max"] * 1000, 3),
        "n_solver_failures": m["n_solver_failures"],
        "worst_case_margin_x": round(dt_control_s / (m["solve_time_max"] + 1e-9), 1),
        "feasible_realtime": bool(m["solve_time_max"] * 1000 < dt_control_s * 1000),
    }
    path = write_csv([row], list(row.keys()), "experiment8_computational_feasibility.csv")
    print("Wrote", path)

    plot_path = plot_solver_time_histogram(run["solve_times"], cfg["simulation"]["dt_control"],
                                            "experiment8_solver_time_histogram.png")
    print("Wrote", plot_path)
    print(json.dumps(row, indent=2))
    return row


if __name__ == "__main__":
    main()
