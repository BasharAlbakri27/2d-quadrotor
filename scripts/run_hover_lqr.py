import numpy as np
import matplotlib.pyplot as plt
import argparse

from planar_quadrotor.params import Params
from planar_quadrotor.dynamics import linearize_hover
from planar_quadrotor.controllers import LQR, lqr_K
from planar_quadrotor.sim import simulate_closedloop
from planar_quadrotor.disturbances import wind_step
from planar_quadrotor.metrics import (
    steady_state_error, max_abs_errors, max_abs_theta,
    settling_time, thrust_metrics, effort_metrics
)
from planar_quadrotor.monitors import run_monitors

# -- parse args for simplicity -- #

parser = argparse.ArgumentParser()
parser.add_argument("--wind", choices=["none", "step"], default="none",
                    help="Disturbance mode: none or step wind at t=2s.")
parser.add_argument("--T", type=float, default=10.0, help="Simulation duration [s].")
args = parser.parse_args()

# --- params / model ---
p = Params(dt=0.01)
T = args.T

# Linearized model around hover (for LQR design)
A, B = linearize_hover(p)

# --- Q,R ---
# state: [x, z, theta, vx, vz, omega]
x_max = [
    0.5,    # x [m]
    0.1,    # z [m]
    0.174,  # theta [rad] (~10 deg)
    1.0,    # vx [m/s]
    0.5,    # vz [m/s]
    1.0     # omega [rad/s]
]

# input: [du (N), dtau (N*m)]
u_max = [
    10.0,   # du [N]
    2.0     # dtau [N*m]
]

Q, R = LQR(x_max, u_max)
K = lqr_K(A, B, Q, R)

print("K =\n", K)

# --- reference hover point ---
x_ref = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])

# --- initial condition (start disturbed) ---
# Example: start with tilt and slight position error
x0 = np.array([0.2, 0.8, 0.15, 0.0, 0.0, 0.0])

# --- disturbance depending on user's preference --- #
disturbance = None
if args.wind == "step":
    disturbance = wind_step

# --- simulate closed-loop nonlinear plant ---
t, X, U, M = simulate_closedloop(x0, T, p, K, x_ref, disturbance=disturbance)

report = run_monitors(X, U, M, p)
print("\n=== Monitors ===")
for k, v in report.items():
    print(k.upper(), v)

# --- plots: x(t), z(t), theta(t) ---
plt.figure()
plt.plot(t, X[:, 0])
plt.axhline(x_ref[0], linestyle="--")
plt.xlabel("t [s]"); plt.ylabel("x [m]"); plt.title("Closed-loop x(t)")
plt.grid(True)
plt.show()

plt.figure()
plt.plot(t, X[:, 1])
plt.axhline(x_ref[1], linestyle="--")
plt.xlabel("t [s]"); plt.ylabel("z [m]"); plt.title("Closed-loop z(t)")
plt.grid(True)
plt.show()

plt.figure()
plt.plot(t, X[:, 2])
plt.axhline(x_ref[2], linestyle="--")
plt.xlabel("t [s]"); plt.ylabel("theta [rad]"); plt.title("Closed-loop theta(t)")
plt.grid(True)
plt.show()

# Trajectory z vs x
plt.figure()
plt.plot(X[:, 0], X[:, 1])
plt.plot(x_ref[0], x_ref[1], "o")
plt.xlabel("x [m]"); plt.ylabel("z [m]"); plt.title("Trajectory (z vs x)")
plt.grid(True)
plt.show()

# Control effort (optional but VERY useful)
plt.figure()
plt.plot(t, U[:, 0], label="du [N]")
plt.plot(t, U[:, 1], label="dtau [N*m]")
plt.xlabel("t [s]"); plt.title("LQR commands")
plt.grid(True); plt.legend()
plt.show()

plt.figure()
plt.plot(t, M[:, 0], label="u1 [N]")
plt.plot(t, M[:, 1], label="u2 [N]")
plt.xlabel("t [s]"); plt.title("Rotor thrusts (after mixing + saturation)")
plt.grid(True); plt.legend()
plt.show()

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