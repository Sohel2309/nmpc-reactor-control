"""
run_horizon_sweep.py
----------------------
EXPERIMENT 5 -- Prediction horizon sensitivity: sweep N over
cfg["nmpc"]["N_sweep"] on the same scenario, and report tracking
performance vs. computational cost (mean/median/p95/max solve time).
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import horizon_sweep_scenario
from src.evaluation import write_csv, plot_horizon_sweep, save_run_raw


def main():
    cfg = load_config()
    scenario = horizon_sweep_scenario(cfg)

    rows = []
    for N in cfg["nmpc"]["N_sweep"]:
        run = run_closed_loop("nmpc", cfg, scenario, N=N)
        m = summarize_run(run)
        save_run_raw(run, f"exp5_nmpc_N{N}.npz")
        rows.append({
            "N": N,
            "IAE": round(m["IAE"], 5),
            "RMSE": round(m["RMSE"], 5),
            "overshoot_pct_max": round(m["overshoot_pct_max"], 3),
            "n_T_violations": m["n_T_violations"],
            "solve_time_mean": m["solve_time_mean"],
            "solve_time_median": m["solve_time_median"],
            "solve_time_p95": m["solve_time_p95"],
            "solve_time_max": m["solve_time_max"],
            "n_solver_failures": m["n_solver_failures"],
        })
        print(f"N={N}: IAE={m['IAE']:.4f} mean_solve={m['solve_time_mean']*1000:.2f}ms "
              f"max_solve={m['solve_time_max']*1000:.2f}ms failures={m['n_solver_failures']}")

    path = write_csv(rows, list(rows[0].keys()), "experiment5_horizon_sweep.csv")
    print("Wrote", path)

    plot_path = plot_horizon_sweep(rows, "experiment5_horizon_sweep.png")
    print("Wrote", plot_path)
    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
