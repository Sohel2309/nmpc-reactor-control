"""
kinetics.py
-----------
Reaction kinetics for the Van de Vusse reaction network:

    A --k1--> B --k2--> C
    2A --k3--> D

All three reactions are modelled as elementary / power-law kinetics with
Arrhenius temperature dependence:

    k1(T) = k10_ab * exp(-EA_ab_R / T_K)
    k2(T) = k10_bc * exp(-EA_bc_R / T_K)
    k3(T) = k10_ad * exp(-EA_ad_R / T_K)

where T_K is the ABSOLUTE reactor temperature in Kelvin. Reactor
temperature elsewhere in this project is tracked in degrees Celsius
(the convention used in the underlying benchmark and in most CSTR
process-control papers), so kinetics functions take T_degC and convert
internally.

Rate laws (mol / (L h)):
    r1 = k1 * cA                (A -> B, first order in A)
    r2 = k2 * cB                (B -> C, first order in B)
    r3 = k3 * cA**2             (2A -> D, second order in A)

This module is written to work transparently with both plain floats
(NumPy) and CasADi symbols (SX/MX), which is required because the same
rate expressions are reused inside the NMPC's symbolic optimization
model and inside the plain-Python "true plant" integrator.
"""

from __future__ import annotations
import numpy as np

CELSIUS_TO_KELVIN = 273.15


def _exp(x):
    """exp() that works for both numpy arrays/floats and CasADi symbols."""
    try:
        import casadi as ca
        if isinstance(x, (ca.SX, ca.MX, ca.DM)):
            return ca.exp(x)
    except ImportError:
        pass
    return np.exp(x)


def _fmax(a, b):
    """max() that works for both numpy arrays/floats and CasADi symbols."""
    try:
        import casadi as ca
        if isinstance(a, (ca.SX, ca.MX, ca.DM)) or isinstance(b, (ca.SX, ca.MX, ca.DM)):
            return ca.fmax(a, b)
    except ImportError:
        pass
    return np.maximum(a, b)


def rate_constants(T_degC, k10_ab, EA_ab_R, k10_bc, EA_bc_R, k10_ad, EA_ad_R):
    """Arrhenius rate constants k1, k2, k3 at reactor temperature T_degC.

    A numerical floor of 1 K is applied to the absolute temperature before
    the Arrhenius exponent is evaluated. This has NO effect anywhere in
    the physically meaningful operating range of this reactor (all
    feasible temperatures here are hundreds of Kelvin above the floor);
    its only purpose is to guarantee a finite, well-defined result if an
    optimizer's internal line search (e.g. IPOPT evaluating a candidate
    Newton step before it has been projected back inside the variable
    bounds) transiently evaluates this function at an unphysical
    temperature. Without this guard, such a transient evaluation can
    produce NaN/Inf (division by a near-zero or negative absolute
    temperature) that poisons the NLP's Jacobian for that iteration --
    this was observed empirically (see docs/limitations.md) and this
    guard is the standard, minimal fix.
    """
    T_K = _fmax(T_degC + CELSIUS_TO_KELVIN, 1.0)
    k1 = k10_ab * _exp(-EA_ab_R / T_K)
    k2 = k10_bc * _exp(-EA_bc_R / T_K)
    k3 = k10_ad * _exp(-EA_ad_R / T_K)
    return k1, k2, k3


def reaction_rates(cA, cB, T_degC, params):
    """
    Compute the three reaction rates r1, r2, r3 (mol/(L h)) given
    concentrations and temperature, using a parameter dict/object with
    fields k10_ab, EA_ab_R, k10_bc, EA_bc_R, k10_ad, EA_ad_R.
    """
    k1, k2, k3 = rate_constants(
        T_degC,
        params["k10_ab"], params["EA_ab_R"],
        params["k10_bc"], params["EA_bc_R"],
        params["k10_ad"], params["EA_ad_R"],
    )
    r1 = k1 * cA
    r2 = k2 * cB
    r3 = k3 * cA ** 2
    return r1, r2, r3, (k1, k2, k3)
