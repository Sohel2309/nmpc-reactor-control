import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.kinetics import rate_constants, reaction_rates
from src.config import load_config

cfg = load_config()
P = cfg["plant"]


def test_rate_constants_positive():
    k1, k2, k3 = rate_constants(130.0, P["k10_ab"], P["EA_ab_R"],
                                 P["k10_bc"], P["EA_bc_R"],
                                 P["k10_ad"], P["EA_ad_R"])
    assert k1 > 0 and k2 > 0 and k3 > 0


def test_rate_constants_increase_with_temperature():
    """Arrhenius kinetics: all rate constants must increase monotonically
    with temperature."""
    k1_low, k2_low, k3_low = rate_constants(100.0, P["k10_ab"], P["EA_ab_R"],
                                             P["k10_bc"], P["EA_bc_R"],
                                             P["k10_ad"], P["EA_ad_R"])
    k1_high, k2_high, k3_high = rate_constants(140.0, P["k10_ab"], P["EA_ab_R"],
                                                P["k10_bc"], P["EA_bc_R"],
                                                P["k10_ad"], P["EA_ad_R"])
    assert k1_high > k1_low
    assert k2_high > k2_low
    assert k3_high > k3_low


def test_reaction_rates_zero_at_zero_concentration():
    r1, r2, r3, _ = reaction_rates(0.0, 0.0, 130.0, P)
    assert r1 == 0.0 and r2 == 0.0 and r3 == 0.0


def test_reaction_rates_scale_correctly():
    """r1, r2 first order; r3 second order in cA."""
    r1a, r2a, r3a, _ = reaction_rates(1.0, 1.0, 130.0, P)
    r1b, r2b, r3b, _ = reaction_rates(2.0, 2.0, 130.0, P)
    assert np.isclose(r1b, 2 * r1a)         # first order in cA
    assert np.isclose(r2b, 2 * r2a)         # first order in cB
    assert np.isclose(r3b, 4 * r3a)         # second order in cA
