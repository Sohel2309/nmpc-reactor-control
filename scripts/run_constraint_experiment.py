"""
run_constraint_experiment.py
-------------------------------
EXPERIMENT 4 -- Constraint activity: compare NMPC behaviour in a MILD
scenario (constraints stay inactive) against an AGGRESSIVE scenario
(the temperature safety constraint genuinely becomes active), to
demonstrate that NMPC's constraint handling is not decorative.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import (constraint_activity_scenario_mild,
                            constraint_activity_scenario_aggressive)
from src.evaluation import write_csv, plot_tracking_comparison, save_run_raw


def main():
    cfg = load_config()
    N = cfg["nmpc"]["N_default"]

    mild = constraint_activity_scenario_mild(cfg)
    aggressive = constraint_activity_scenario_aggressive(cfg)

    run_mild = run_closed_loop("nmpc", cfg, mild, N=N)
    run_aggr = run_closed_loop("nmpc", cfg, aggressive, N=N)

    m_mild = summarize_run(run_mild)
    m_aggr = summarize_run(run_aggr)

    save_run_raw(run_mild, "exp4_nmpc_mild.npz")
    save_run_raw(run_aggr, "exp4_nmpc_aggressive.npz")

    rows = [
        {"scenario": "Mild (constraints inactive)",
         "IAE": round(m_mild["IAE"], 5),
         "max_T": round(run_mild["x"][:, 2].max(), 3),
         "n_T_violations": m_mild["n_T_violations"],
         "max_T_violation": round(m_mild["max_T_violation"], 3),
         "max_slack_used": "n/a (not logged per-step)",
         "F_at_max_frac": round(float((run_mild["u"][:, 0] >=
                                        cfg["constraints"]["F_max"] - 1e-3).mean()), 3)},
        {"scenario": "Aggressive (T constraint active)",
         "IAE": round(m_aggr["IAE"], 5),
         "max_T": round(run_aggr["x"][:, 2].max(), 3),
         "n_T_violations": m_aggr["n_T_violations"],
         "max_T_violation": round(m_aggr["max_T_violation"], 3),
         "max_slack_used": "n/a (not logged per-step)",
         "F_at_max_frac": round(float((run_aggr["u"][:, 0] >=
                                        cfg["constraints"]["F_max"] - 1e-3).mean()), 3)},
    ]
    path = write_csv(rows, list(rows[0].keys()), "experiment4_constraint_activity.csv")
    print("Wrote", path)

    plot_path = plot_tracking_comparison(
        {"NMPC (mild)": run_mild, "NMPC (aggressive)": run_aggr},
        title="Experiment 4: Constraint Activity -- Mild vs Aggressive Setpoint Push",
        filename="experiment4_constraint_activity.png",
    )
    print("Wrote", plot_path)
    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
