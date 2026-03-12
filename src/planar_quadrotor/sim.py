import numpy as np

from .dynamics import f_quad2d
from .integrators import rk4_step
from .params import Params
from .mixer import mixer_func
from .params import Params

def simulate_openloop(x0: np.ndarray, T: float, p: Params):
    dt = p.dt
    N = int(T / dt)

    # logs
    t_log = np.zeros(N + 1)
    x_log = np.zeros((N + 1, 6))

    # initial condition
    x = x0.astype(float).copy()
    x_log[0, :] = x

    # constant hover thrust
    u_hover_total = p.m * p.g        # should be m*g
    u1 = u_hover_total * 0.5              # half of it
    u2 = u_hover_total * 0.5

    for k in range(N):
        t_log[k + 1] = (k + 1) * dt
        x = rk4_step(f_quad2d, x, u1, u2, dt, p)
        x_log[k + 1, :] = x

    return t_log, x_log


def simulate_closedloop(x0, T, p, K, x_ref, disturbance=None):
    dt = p.dt
    N = int(T / dt)

    t_log = np.zeros(N + 1)
    x_log = np.zeros((N + 1, 6))
    u_log = np.zeros((N + 1, 2))   # [du, dtau]
    m_log = np.zeros((N + 1, 2))   # [u1, u2]

    x = x0.astype(float).copy()
    x_log[0, :] = x

    for k in range(N):

        t = (k + 1) * dt
        t_log[k + 1] = t

        dist = None
        if disturbance is not None:
            dist = disturbance(t, x)

        # 1) error
        e = x - x_ref

        # 2) controller
        u = -K @ e
        du, dtau = u[0], u[1]
        u_log[k, :] = [du, dtau]

        # 3) mixer
        u1, u2 = mixer_func(du, dtau, p)
        m_log[k, :] = [u1, u2]

        # 4) integrate nonlinear plant
        x = rk4_step(f_quad2d, x, u1, u2, dt, p, dist=dist)
        x_log[k + 1, :] = x

    # log last input too (optional)
    u_log[-1, :] = u_log[-2, :]
    m_log[-1, :] = m_log[-2, :]

    return t_log, x_log, u_log, m_log


def simulate_closedloop_lqi(x0, T, p, K_aug, x_ref, disturbance=None):
    dt = p.dt
    N = int(T / dt)

    t_log = np.zeros(N + 1)
    x_log = np.zeros((N + 1, 6))
    u_log = np.zeros((N + 1, 2))   # [du, dtau]
    m_log = np.zeros((N + 1, 2))   # [u1, u2]

    # NEW: log integrators if you want (helps debugging/plots)
    i_log = np.zeros((N + 1, 2))   # [xI, zI]

    x = x0.astype(float).copy()
    x_log[0, :] = x

    # NEW: integral states start at 0
    xI = 0.0
    zI = 0.0
    i_log[0, :] = [xI, zI]

    for k in range(N):
        t = (k + 1) * dt
        t_log[k + 1] = t

        dist = None
        if disturbance is not None:
            dist = disturbance(t, x)

        # error in the original 6-state coordinates
        e = x - x_ref

        # NEW: integrate position errors (simple Euler is fine here)
        xI += dt * e[0]   # integral of x error
        zI += dt * e[1]   # integral of z error
        i_log[k + 1, :] = [xI, zI]

        # NEW: augmented error state (length 8)
        xa = np.hstack([e, [xI, zI]])

        # controller (LQI gain)
        u = -K_aug @ xa
        du, dtau = u[0], u[1]
        u_log[k, :] = [du, dtau]

        # mixer
        u1, u2 = mixer_func(du, dtau, p)
        m_log[k, :] = [u1, u2]

        # integrate nonlinear plant
        x = rk4_step(f_quad2d, x, u1, u2, dt, p, dist=dist)
        x_log[k + 1, :] = x

    u_log[-1, :] = u_log[-2, :]
    m_log[-1, :] = m_log[-2, :]

    return t_log, x_log, u_log, m_log, i_log