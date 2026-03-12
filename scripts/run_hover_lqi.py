import numpy as np
import matplotlib.pyplot as plt

from planar_quadrotor.params import Params
from planar_quadrotor.dynamics import linearize_hover
from planar_quadrotor.controllers import LQR, lqr_K
from planar_quadrotor.sim import simulate_closedloop_lqi
from planar_quadrotor.disturbances import wind_step
from planar_quadrotor.metrics import (
    steady_state_error, max_abs_errors, max_abs_theta,
    settling_time, thrust_metrics, effort_metrics
)
from planar_quadrotor.monitors import run_monitors
import argparse


def augmented_AB(A, B):
    """
    Augment the hover-linearized system with integrators of x and z error:

        xI_dot = e_x
        zI_dot = e_z

    State ordering:
        original e = [x, z, theta, vx, vz, omega]
        augmented xa = [e, xI, zI]  (length 8)
    """
    A_aug = np.zeros((8, 8))
    B_aug = np.zeros((8, 2))

    # Top-left: original dynamics
    A_aug[:6, :6] = A
    B_aug[:6, :] = B

    # Bottom-left: integrators take x and z components of e
    # xI_dot = e_x  -> row 6 picks column 0
    # zI_dot = e_z  -> row 7 picks column 1
    A_aug[6, 0] = 1.0
    A_aug[7, 1] = 1.0

    return A_aug, B_aug

# -- parse args for simplicity -- #

parser = argparse.ArgumentParser()
parser.add_argument("--wind", choices=["none", "step"], default="none",
                    help="Disturbance mode: none or step wind at t=2s.")
parser.add_argument("--T", type=float, default=10.0, help="Simulation duration [s].")
args = parser.parse_args()


p = Params(dt=0.01)
T = args.T

# Linear model at hover (for design only)
A, B = linearize_hover(p)
A_aug, B_aug = augmented_AB(A, B)

# --- Bryson limits (same as your LQR baseline) ---
# original state limits: [x, z, theta, vx, vz, omega]
x_max = [0.5, 0.1, 0.174, 1.0, 0.5, 1.0]

# integrator limits (units: m*s)
# simple, intentional choice: (error_max) * (2 seconds)
xI_max = x_max[0] * 0.8   # 1.0 m*s
zI_max = x_max[1] * 2.0   # 0.2 m*s

x_max_aug = x_max + [xI_max, zI_max]

# input limits: [du, dtau]
u_max = [10.0, 2.0]

Q_aug, R = LQR(x_max_aug, u_max)
K_aug = lqr_K(A_aug, B_aug, Q_aug, R)

print("K_aug =\n", K_aug)

# reference hover
x_ref = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])

# initial condition (disturbed)
x0 = np.array([0.2, 0.8, 0.15, 0.0, 0.0, 0.0])

# simulate with wind step (Fx turns on at 2s)
t, X, U, M, Ilog = simulate_closedloop_lqi(
    x0, T, p, K_aug, x_ref, disturbance=wind_step
)

report = run_monitors(X, U, M, p, Ilog=Ilog)
print("\n=== Monitors ===")
for k, v in report.items():
    print(k.upper(), v)

# --- plots ---
plt.figure()
plt.plot(t, X[:, 0], label="x(t)")
plt.axhline(x_ref[0], linestyle="--", label="x_ref")
plt.xlabel("t [s]"); plt.ylabel("x [m]"); plt.title("LQI: x(t) with wind step")
plt.grid(True); plt.legend(); plt.show()

plt.figure()
plt.plot(t, X[:, 1], label="z(t)")
plt.axhline(x_ref[1], linestyle="--", label="z_ref")
plt.xlabel("t [s]"); plt.ylabel("z [m]"); plt.title("LQI: z(t) with wind step")
plt.grid(True); plt.legend(); plt.show()

plt.figure()
plt.plot(t, X[:, 2], label="theta(t)")
plt.axhline(0.0, linestyle="--")
plt.xlabel("t [s]"); plt.ylabel("theta [rad]"); plt.title("LQI: theta(t) with wind step")
plt.grid(True); plt.legend(); plt.show()

plt.figure()
plt.plot(X[:, 0], X[:, 1], label="trajectory")
plt.plot(X[0, 0], X[0, 1], "go", label="start")
plt.plot(x_ref[0], x_ref[1], "ro", label="reference")
idx = np.argmin(np.abs(t - 2.0))
plt.plot(X[idx, 0], X[idx, 1], "mo", label="wind on (t=2s)")
plt.xlabel("x [m]"); plt.ylabel("z [m]"); plt.title("LQI: trajectory (z vs x)")
plt.grid(True); plt.legend(); plt.show()

# see the integrators
plt.figure()
plt.plot(t, Ilog[:, 0], label="xI")
plt.plot(t, Ilog[:, 1], label="zI")
plt.xlabel("t [s]"); plt.title("LQI integrator states")
plt.grid(True); plt.legend(); plt.show()

ssx, ssz = steady_state_error(t, X, x_ref, frac=0.1)
mx, mz = max_abs_errors(X, x_ref)
tset_x = settling_time(t, X[:,0], x_ref[0], tol=0.02)  # 2 cm band
tset_z = settling_time(t, X[:,1], x_ref[1], tol=0.02)
max_theta = max_abs_theta(X)
max_u, sat_ratio = thrust_metrics(M, p.u_min, p.u_max)
max_du, max_dtau = effort_metrics(U)

print("\n=== Metrics ===")
print(f"ss_error:   x={ssx:.4f} m, z={ssz:.4f} m")
print(f"max_error:  x={mx:.4f} m, z={mz:.4f} m")
print(f"settle:     x={tset_x:.3f} s, z={tset_z:.3f} s")
print(f"max_theta:  {max_theta:.4f} rad")
print(f"max_u:      {max_u:.2f} N, sat_ratio={sat_ratio*100:.2f}%")
print(f"max_effort: |du|={max_du:.2f} N, |dtau|={max_dtau:.2f} N*m")