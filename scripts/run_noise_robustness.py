"""
run_noise_robustness.py
--------------------------
EXPERIMENT 6 -- Noise robustness: evaluate PID and NMPC on the same
setpoint-step scenario under LOW and HIGH measurement/process noise
levels (cfg["noise"]).
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import noise_robustness_scenario
from src.evaluation import write_csv, plot_degradation, save_run_raw


def main():
    cfg = load_config()
    N = cfg["nmpc"]["N_default"]
    noise_cfg = cfg["noise"]

    levels = [
        ("none", 0.0, 0.0),
        ("low", noise_cfg["cB_std_low"], noise_cfg["process_std_low"]),
        ("high", noise_cfg["cB_std_high"], noise_cfg["process_std_high"]),
    ]

    rows = []
    iae_by_controller = {"PID": [], "NMPC": []}
    labels = []
    for label, meas_std, proc_std in levels:
        labels.append(label)
        scenario = noise_robustness_scenario(cfg, meas_std, proc_std)
        run_pid = run_closed_loop("pid", cfg, scenario)
        run_nmpc = run_closed_loop("nmpc", cfg, scenario, N=N)
        m_pid = summarize_run(run_pid)
        m_nmpc = summarize_run(run_nmpc)

        save_run_raw(run_pid, f"exp6_pid_{label}.npz")
        save_run_raw(run_nmpc, f"exp6_nmpc_{label}.npz")

        iae_by_controller["PID"].append(m_pid["IAE"])
        iae_by_controller["NMPC"].append(m_nmpc["IAE"])

        for ctrl_name, m in [("PID", m_pid), ("NMPC", m_nmpc)]:
            rows.append({
                "noise_level": label, "meas_noise_std": meas_std,
                "process_noise_std": proc_std, "controller": ctrl_name,
                "IAE": round(m["IAE"], 5), "RMSE": round(m["RMSE"], 5),
                "control_effort_F": round(m["control_effort_F"], 3),
                "n_T_violations": m["n_T_violations"],
            })

    path = write_csv(rows, list(rows[0].keys()), "experiment6_noise_robustness.csv")
    print("Wrote", path)

    plot_path = plot_degradation(
        labels, iae_by_controller, xlabel="Noise level",
        ylabel="IAE (tracking error)",
        title="Experiment 6: Noise Robustness -- PID vs NMPC",
        filename="experiment6_noise_robustness.png",
    )
    print("Wrote", plot_path)
    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
