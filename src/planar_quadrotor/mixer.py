# Mixer function (aka actuator allocation / motor mixing):
# Converts "nice" controller commands (du, dtau) into physical rotor thrusts (u1, u2).
#
# - du   : deviation in total thrust from hover [N]
# - dtau : deviation in pitch torque from hover [N·m]
#
# The plant (nonlinear dynamics) expects individual rotor thrusts u1 and u2 [N].
# We also add the hover bias (mg/2 on each rotor) so that du=0 corresponds to hover.

from .params import Params


def clamp(val: float, lo: float, hi: float):
    """Clamp a scalar value into [lo, hi]."""
    return max(lo, min(hi, val))


def mixer_func(du: float, dtau: float, p: Params):
    """
    Map controller outputs (du, dtau) to rotor thrust commands (u1, u2).

    Physics:
      u  = u1 + u2                 (total thrust)
      tau = L * (u2 - u1)          (pitch torque, positive when right rotor > left rotor)

    Here we work in deviations around hover:
      u_hover = m*g
      u1_hover = u2_hover = (m*g)/2

      du1 = 0.5*du - dtau/(2L)
      du2 = 0.5*du + dtau/(2L)

      u1 = u1_hover + du1
      u2 = u2_hover + du2

    We finally clamp u1,u2 to actuator limits [u_min, u_max].
    """
    # Hover bias: total thrust required to cancel gravity
    u_hover_total = p.m * p.g
    u1_hover = 0.5 * u_hover_total
    u2_hover = 0.5 * u_hover_total

    # Convert (du, dtau) to per-rotor thrust deviations (derived from dynamics)
    du1 = 0.5 * du - (dtau / (2.0 * p.L))   
    du2 = 0.5 * du + (dtau / (2.0 * p.L))

    # Add hover bias to get absolute rotor thrust commands
    u1 = u1_hover + du1
    u2 = u2_hover + du2

    # Saturate to physically valid thrust range (per-rotor)
    u1 = clamp(u1, p.u_min, p.u_max)
    u2 = clamp(u2, p.u_min, p.u_max)

    return u1, u2