import numpy as np

def rk4_step(f, x, u1, u2, dt, p, dist=None):
    k1 = f(x, u1, u2, p, dist)
    k2 = f(x + (dt/2)*k1, u1, u2, p, dist)
    k3 = f(x + (dt/2)*k2, u1, u2, p, dist)
    k4 = f(x + dt*k3, u1, u2, p, dist)

    return x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)