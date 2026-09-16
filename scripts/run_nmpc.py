"""
run_nmpc.py
-----------
Standalone demo: run the NMPC controller alone on the standard setpoint
scenario at the default prediction horizon and print/save its metrics.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.simulation import run_closed_loop
from src.metrics import summarize_run
from src.scenarios import setpoint_scenario
from src.evaluation import save_run_raw


def main():
    cfg = load_config()
    scenario = setpoint_scenario(cfg)
    N = cfg["nmpc"]["N_default"]
    run = run_closed_loop("nmpc", cfg, scenario, N=N)
    m = summarize_run(run)
    save_run_raw(run, "nmpc_setpoint.npz")
    print(json.dumps({k: v for k, v in m.items() if k != "segments"}, indent=2))
    print("segments:", m["segments"])
    print("solver failures:", run["n_solver_failures"])


if __name__ == "__main__":
    main()
