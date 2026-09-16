"""
run_disturbance_experiment.py
-------------------------------
EXPERIMENT 2 -- Disturbance rejection: constant setpoint, with feed
concentration and feed temperature step disturbances applied to the SAME
true plant, at the SAME times, for both PID and NMPC.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import disturbance_scenario
from src.evaluation import write_csv, plot_tracking_comparison, save_run_raw


def main():
    cfg = load_config()
    scenario = disturbance_scenario(cfg)
    N = cfg["nmpc"]["N_default"]

    run_pid = run_closed_loop("pid", cfg, scenario)
    run_nmpc = run_closed_loop("nmpc", cfg, scenario, N=N)

    m_pid = summarize_run(run_pid)
    m_nmpc = summarize_run(run_nmpc)

    save_run_raw(run_pid, "exp2_pid.npz")
    save_run_raw(run_nmpc, "exp2_nmpc.npz")

    rows = []
    for label, m in [("PID", m_pid), ("NMPC", m_nmpc)]:
        rows.append({
            "controller": label,
            "IAE": round(m["IAE"], 5), "ISE": round(m["ISE"], 5),
            "MAE": round(m["MAE"], 5), "RMSE": round(m["RMSE"], 5),
            "control_effort_F": round(m["control_effort_F"], 3),
            "control_effort_Qk": round(m["control_effort_Qk"], 1),
            "n_T_violations": m["n_T_violations"],
            "max_T_violation": round(m["max_T_violation"], 3),
        })
    path = write_csv(rows, list(rows[0].keys()), "experiment2_disturbance_rejection.csv")
    print("Wrote", path)

    plot_path = plot_tracking_comparison(
        {"PID": run_pid, "NMPC": run_nmpc},
        title="Experiment 2: Disturbance Rejection -- PID vs NMPC\n"
              "(+15% feed conc. step at t=1.5h, +8C feed temp step at t=3.5h)",
        filename="experiment2_disturbance_rejection.png",
    )
    print("Wrote", plot_path)
    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
