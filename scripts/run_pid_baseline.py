"""
run_pid_baseline.py
--------------------
Standalone demo: run the PID baseline alone on the standard setpoint
scenario and print/save its metrics. Mainly useful for quick manual
checks; the full comparative experiments are in run_setpoint_experiment.py
etc.
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
    run = run_closed_loop("pid", cfg, scenario)
    m = summarize_run(run)
    save_run_raw(run, "pid_baseline_setpoint.npz")
    print(json.dumps({k: v for k, v in m.items() if k != "segments"}, indent=2))
    print("segments:", m["segments"])


if __name__ == "__main__":
    main()
