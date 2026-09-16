"""
run_ssopt_baseline.py
------------------------
EXPERIMENT 3 -- Steady-state optimization vs. dynamic (closed-loop)
control.

The steady-state optimizer (src/steady_state_optimizer.py) finds the
input that maximizes cB at steady state, then that single input is
applied OPEN LOOP (no feedback at all) for the whole run, exactly like a
plant engineer who solves an optimization once and leaves the valves at
the computed setting. We then subject that open-loop input, and
separately the NMPC controller, to the SAME feed disturbance, to show
quantitatively that a good steady-state operating point is not the same
thing as good dynamic disturbance rejection.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import load_config
from src.steady_state_optimizer import solve_steady_state_optimum
from src.simulation import run_closed_loop, Scenario
from src.metrics import summarize_run
from src.scenarios import disturbance_scenario
from src.evaluation import write_csv, plot_tracking_comparison, save_run_raw


def main():
    cfg = load_config()

    # Step 1: solve the steady-state economic optimum once.
    x_star, u_star, info = solve_steady_state_optimum(cfg["plant"], cfg)
    print("Steady-state optimum: x* =", x_star.round(4), "u* =", u_star.round(2),
          "solver:", info)

    # Step 2: apply that input OPEN LOOP, starting from the reactor already
    # AT that optimal steady state, then hit it with the standard feed
    # disturbance sequence. A perfect open-loop controller would only do
    # well here if the process were undisturbed.
    scenario = disturbance_scenario(cfg)
    # Override initial condition/setpoint to the steady-state optimum so
    # the comparison isolates "disturbance response", not "reaching the
    # optimum in the first place".
    scenario.setpoint_changes = [(0.0, x_star[1])]

    run_openloop = run_closed_loop("openloop", cfg, scenario, x0=x_star, u0=u_star,
                                    fixed_input=u_star)
    run_nmpc = run_closed_loop("nmpc", cfg, scenario, x0=x_star, u0=u_star,
                                N=cfg["nmpc"]["N_default"])

    m_ol = summarize_run(run_openloop)
    m_nmpc = summarize_run(run_nmpc)

    save_run_raw(run_openloop, "exp3_openloop_ssopt.npz")
    save_run_raw(run_nmpc, "exp3_nmpc.npz")

    rows = [
        {"controller": "Open-loop (ss-opt input)",
         "IAE": round(m_ol["IAE"], 5), "RMSE": round(m_ol["RMSE"], 5),
         "n_T_violations": m_ol["n_T_violations"],
         "max_T_violation": round(m_ol["max_T_violation"], 3),
         "final_cB": round(run_openloop["x"][-1, 1], 4),
         "target_cB": round(x_star[1], 4)},
        {"controller": "NMPC (closed loop)",
         "IAE": round(m_nmpc["IAE"], 5), "RMSE": round(m_nmpc["RMSE"], 5),
         "n_T_violations": m_nmpc["n_T_violations"],
         "max_T_violation": round(m_nmpc["max_T_violation"], 3),
         "final_cB": round(run_nmpc["x"][-1, 1], 4),
         "target_cB": round(x_star[1], 4)},
    ]
    path = write_csv(rows, list(rows[0].keys()), "experiment3_ssopt_vs_dynamic.csv")
    print("Wrote", path)

    plot_path = plot_tracking_comparison(
        {"Open-loop (ss-opt)": run_openloop, "NMPC": run_nmpc},
        title="Experiment 3: Steady-State-Optimal Input (open loop) vs NMPC\n"
              "under the same feed disturbances",
        filename="experiment3_ssopt_vs_dynamic.png",
    )
    print("Wrote", plot_path)
    print(json.dumps(rows, indent=2))
    return rows


if __name__ == "__main__":
    main()
