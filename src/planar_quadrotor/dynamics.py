import numpy as np
from .params import Params

def f_quad2d(x: np.ndarray, u1: float, u2: float, p: Params, dist=None):
    # unpack state
    px, pz, th, vx, vz, om = x

    u = u1 + u2  # total thrust

    xdot = np.zeros_like(x, dtype=float)

    # World-frame disturbances (forces in Newtons, torque in N*m)
    Fx = Fz = tau_d = 0.0
    if dist is not None:
        Fx = float(dist.get("Fx", 0.0))
        Fz = float(dist.get("Fz", 0.0))
        tau_d = float(dist.get("tau", 0.0))

    # kinematics
    xdot[0] = vx  # xdot
    xdot[1] = vz # zdot
    xdot[2] = om  # thetadot

    # dynamics
    xdot[3] = -(u/p.m) * np.sin(th) + Fx / p.m      # xdd
    xdot[4] =  (u/p.m) * np.cos(th) - p.g + Fz / p.m    # zdd  
    xdot[5] =  (p.L/p.I) * (u2 - u1) + tau_d / p.I      # thdd

    return xdot

def linearize_hover(p: Params):
    A = np.zeros((6, 6))
    B = np.zeros((6, 2))

    # Kinematics
    A[0, 3] =  1  # xdot = vx
    A[1, 4] = 1  # zdot = vz
    A[2, 5] = 1   # thetadot = omega

    # Dynamics (linearized at hover)
    A[3, 2] = -p.g   # vxdot = -g * theta

    B[4, 0] = 1/p.m  # vzdot = (1/m) * du
    B[5, 1] = 1/p.I   # omegadot = (1/I) * dtau

    return A, B