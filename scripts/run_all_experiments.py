"""
run_all_experiments.py
------------------------
Runs every experiment (1-8) in sequence and prints a final summary. This
is the single command referenced in the README for reproducing all
results:

    python scripts/run_all_experiments.py

Each individual script can also be run on its own; this just chains them
and reports overall timing/status, and does not duplicate any numerics.
"""
import sys, os, time, traceback
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # scripts/ itself, for importlib

SCRIPTS = [
    ("Steady-state verification", "find_steady_state"),
    ("Experiment 1: Setpoint tracking", "run_setpoint_experiment"),
    ("Experiment 2: Disturbance rejection", "run_disturbance_experiment"),
    ("Experiment 3: SS-opt vs dynamic control", "run_ssopt_baseline"),
    ("Experiment 4: Constraint activity", "run_constraint_experiment"),
    ("Experiment 5: Horizon sweep", "run_horizon_sweep"),
    ("Experiment 6: Noise robustness", "run_noise_robustness"),
    ("Experiment 7: Model mismatch", "run_model_mismatch"),
    ("Experiment 8: Computational feasibility", "run_computational_feasibility"),
]


def main():
    import importlib
    results = []
    for label, module_name in SCRIPTS:
        print("=" * 70)
        print(label)
        print("=" * 70)
        t0 = time.time()
        try:
            mod = importlib.import_module(module_name)
            mod.main()
            status = "OK"
        except Exception:
            traceback.print_exc()
            status = "FAILED"
        dt = time.time() - t0
        results.append((label, status, dt))
        print(f"--- {label}: {status} ({dt:.1f}s) ---\n")

    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    for label, status, dt in results:
        print(f"{status:8s} {dt:7.1f}s  {label}")

    n_failed = sum(1 for _, s, _ in results if s != "OK")
    if n_failed:
        print(f"\n{n_failed} experiment(s) FAILED -- see output above.")
        sys.exit(1)
    else:
        print("\nAll experiments completed successfully.")
        print("Plots:  results/plots/")
        print("Tables: results/tables/")
        print("Raw data: results/raw/")


if __name__ == "__main__":
    main()
