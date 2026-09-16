"""
dynamics.py
-----------
First-principles material and energy balances for the jacketed Van de Vusse
CSTR (Klatt & Engell, 1998 benchmark formulation, also used e.g. in
Chen/Kremling/Allgower 1995 and in numerous nonlinear MPC papers).

States (x):
    cA  [mol/L]   concentration of reactant A in the reactor
    cB  [mol/L]   concentration of desired product B in the reactor
    T   [deg C]   reactor (liquid) temperature
    TK  [deg C]   cooling-jacket temperature

Manipulated inputs (u):
    F   [h^-1]    dilution rate = feed volumetric flow / reactor volume
                  (equivalent to manipulating feed flow at fixed volume)
    Qk  [kJ/h]    heat duty supplied to/removed from the jacket by the
                  cooling system (negative = net cooling, as used here)

Governing equations
--------------------
Component A (accounts for A -> B consumption and 2A -> D consumption):
    dcA/dt = F*(cA0 - cA) - r1 - r3

Component B (produced by A -> B, consumed by B -> C):
    dcB/dt = -F*cB + r1 - r2

Reactor energy balance (convective feed term + jacket heat exchange +
reaction heat generation/consumption):
    dT/dt  = F*(T_in - T)
             + (kw*AR)/(rho*Cp*VR) * (TK - T)
             - (1/(rho*Cp)) * (r1*dH_ab + r2*dH_bc + r3*dH_ad)

Jacket energy balance:
    dTK/dt = (1/(mk*CPK)) * (Qk + kw*AR*(T - TK))

with reaction rates r1 = k1*cA, r2 = k2*cB, r3 = k3*cA**2 and Arrhenius
rate constants k1(T), k2(T), k3(T) from kinetics.py.

Design note (why this is the "true plant" vs "internal model" split):
The exact same symbolic right-hand side, `cstr_rhs`, is reused both by
the explicit RK4 integrator that plays the role of the TRUE PLANT
(plant_model.py) and by the CasADi symbolic model used inside the NMPC
(nmpc_controller.py). What differs between the two is only the
*parameter values* passed in: the plant always uses the nominal
parameter set, while the NMPC's internal model can be given deliberately
perturbed parameters (see internal_model_mismatch in the config) to
create a genuine plant-model mismatch, rather than an NMPC that
secretly "knows" the true plant.
"""

from __future__ import annotations
from .kinetics import reaction_rates

# State / input index conventions used throughout the project
STATE_NAMES = ["cA", "cB", "T", "TK"]
INPUT_NAMES = ["F", "Qk"]
N_STATES = 4
N_INPUTS = 2


def cstr_rhs(x, u, params):
    """
    Right-hand side dx/dt = f(x, u; params) of the CSTR ODEs.

    Works transparently with plain floats / numpy arrays (used by the
    true-plant RK4 integrator) and with CasADi SX/MX symbols (used by
    the NMPC's symbolic prediction model), because all operations used
    here (+, -, *, /, exp via kinetics._exp) are polymorphic.

    Parameters
    ----------
    x : sequence of 4 (cA, cB, T, TK)
    u : sequence of 2 (F, Qk)
    params : dict with plant/model physical parameters (see config yaml)

    Returns
    -------
    list of 4 symbolic/numeric time derivatives [dcA, dcB, dT, dTK]
    """
    cA, cB, T, TK = x[0], x[1], x[2], x[3]
    F, Qk = u[0], u[1]

    r1, r2, r3, _ = reaction_rates(cA, cB, T, params)

    rho = params["rho"]
    Cp = params["Cp"]
    kw = params["kw"]
    AR = params["AR"]
    VR = params["VR"]
    mk = params["mk"]
    CPK = params["CPK"]
    cA0 = params["cA0"]
    T_in = params["T_in"]

    dcA = F * (cA0 - cA) - r1 - r3
    dcB = -F * cB + r1 - r2
    dT = (F * (T_in - T)
          + (kw * AR) / (rho * Cp * VR) * (TK - T)
          - (1.0 / (rho * Cp)) * (r1 * params["dH_ab"]
                                   + r2 * params["dH_bc"]
                                   + r3 * params["dH_ad"]))
    dTK = (1.0 / (mk * CPK)) * (Qk + kw * AR * (T - TK))

    return [dcA, dcB, dT, dTK]


def apply_mismatch(params: dict, mismatch: dict) -> dict:
    """
    Build the NMPC's INTERNAL MODEL parameter set by applying multiplicative
    perturbations to a copy of the true-plant parameters. Never mutates the
    input dict. `mismatch` supplies multipliers such as k10_ab_mult,
    k10_bc_mult, kw_mult (missing keys default to 1.0 = no mismatch).
    """
    p = dict(params)
    p["k10_ab"] = params["k10_ab"] * mismatch.get("k10_ab_mult", 1.0)
    p["k10_bc"] = params["k10_bc"] * mismatch.get("k10_bc_mult", 1.0)
    p["kw"] = params["kw"] * mismatch.get("kw_mult", 1.0)
    return p
