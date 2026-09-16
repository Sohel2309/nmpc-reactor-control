"""
run_model_mismatch.py
------------------------
EXPERIMENT 7 (REQUIRED) -- Model-mismatch robustness.

The TRUE PLANT always uses the nominal parameters from cfg["plant"]. The
NMPC's INTERNAL MODEL is built from a deliberately perturbed parameter
set (dynamics.apply_mismatch) at several mismatch levels on the
pre-exponential factor of the A->B reaction (k10_ab) -- physically, this
represents the NMPC being designed/calibrated against a reaction-rate
constant that is not perfectly known (a very common real situation,
since pre-exponential factors and activation energies are usually
identified from noisy kinetic experiments).

We explicitly answer: "how much model mismatch can this controller
tolerate before performance becomes unacceptable?"
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import mismatch_scenario
from src.evaluation import write_csv, plot_degradation, save_run_raw


def main():
    cfg = load_config()
    N = cfg["nmpc"]["N_default"]
    scenario = mismatch_scenario(cfg)

    # Multiplicative mismatch on the internal model's A->B pre-exponential
    # factor: 1.0 = perfectly matched (no mismatch); < 1.0 = internal
    # model UNDER-estimates how fast A converts to B; > 1.0 = OVER-estimates.
    mismatch_levels = [1.0, 0.9, 0.75, 1.25, 1.5, 0.5]

    rows = []
    iae_vals = []
    labels = []
    for mult in mismatch_levels:
        mismatch = {"k10_ab_mult": mult}
        run = run_closed_loop("nmpc", cfg, scenario, N=N, internal_mismatch=mismatch)
        m = summarize_run(run)
        save_run_raw(run, f"exp7_nmpc_mismatch_{mult}.npz")

        label = f"{mult:.2f}x"
        labels.append(label)
        iae_vals.append(m["IAE"])

        rows.append({
            "k10_ab_multiplier": mult,
            "IAE": round(m["IAE"], 5),
            "RMSE": round(m["RMSE"], 5),
            "overshoot_pct_max": round(m["overshoot_pct_max"], 3),
            "n_T_violations": m["n_T_violations"],
            "max_T_violation": round(m["max_T_violation"], 3),
            "n_solver_failures": m["n_solver_failures"],
        })
        print(f"mult={mult}: IAE={m['IAE']:.4f} Tviol={m['n_T_violations']} "
              f"maxTviol={m['max_T_violation']:.2f}")

    path = write_csv(rows, list(rows[0].keys()), "experiment7_model_mismatch.csv")
    print("Wrote", path)

    # Sort by multiplier for a clean degradation curve
    order = sorted(range(len(mismatch_levels)), key=lambda i: mismatch_levels[i])
    sorted_labels = [labels[i] for i in order]
    sorted_iae = [iae_vals[i] for i in order]

    plot_path = plot_degradation(
        sorted_labels, {"NMPC IAE": sorted_iae},
        xlabel="Internal-model k10_ab multiplier (1.0 = matched)",
        ylabel="IAE (tracking error)",
        title="Experiment 7: NMPC Performance vs Model Mismatch",
        filename="experiment7_model_mismatch.png",
    )
    print("Wrote", plot_path)
    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
