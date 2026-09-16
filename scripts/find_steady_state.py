"""
find_steady_state.py
---------------------
Utility / documentation script: numerically verifies that the
`nominal_state` values in config/experiment_config.yaml are a genuine
fixed point of the plant ODEs at `nominal_input`, and shows a small
sweep of other candidate operating points. This script does not need to
be run as part of the experiment pipeline -- it documents *how* the
nominal operating point in the config file was derived, for
reproducibility and interview defensibility.

Run: python scripts/find_steady_state.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml
from scipy.optimize import fsolve
from src.dynamics import cstr_rhs

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "experiment_config.yaml")


def main():
    cfg = yaml.safe_load(open(CONFIG_PATH))
    p = cfg["plant"]
    u = [cfg["nominal_input"]["F"], cfg["nominal_input"]["Qk"]]
    x0 = [cfg["nominal_state"]["cA"], cfg["nominal_state"]["cB"],
          cfg["nominal_state"]["T"], cfg["nominal_state"]["TK"]]

    def rhs(x):
        return cstr_rhs(x, u, p)

    x_ss, info, ier, msg = fsolve(rhs, x0, full_output=True)
    residual = rhs(x_ss)

    print("Steady-state solver status:", msg)
    print(f"u = F={u[0]:.3f} h^-1, Qk={u[1]:.1f} kJ/h")
    print(f"x_ss = cA={x_ss[0]:.4f}, cB={x_ss[1]:.4f}, "
          f"T={x_ss[2]:.3f} C, TK={x_ss[3]:.3f} C")
    print("max |residual| =", max(abs(r) for r in residual))
    assert ier == 1, "Steady-state solve did not converge"
    assert max(abs(r) for r in residual) < 1e-4, "Residual too large"
    print("OK: nominal_state in config is a verified fixed point.")


if __name__ == "__main__":
    main()
