import numpy as np
import matplotlib.pyplot as plt

from planar_quadrotor.params import Params
from planar_quadrotor.sim import simulate_openloop


def plot_two_cases(title, tA, yA, tB, yB, ylabel):
    fig, ax = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    fig.suptitle(title)

    ax[0].plot(tA, yA)
    ax[0].set_title("Case A")
    ax[0].set_xlabel("t [s]")
    ax[0].set_ylabel(ylabel)
    ax[0].grid(True)

    ax[1].plot(tB, yB)
    ax[1].set_title("Case B")
    ax[1].set_xlabel("t [s]")
    ax[1].grid(True)

    plt.tight_layout()
    plt.show()



p = Params(dt=0.01)
T = 5.0

# Case A: theta=0 (should hover)
x0_A = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
tA, xA = simulate_openloop(x0_A, T, p)

# Case B: theta=0.1 rad (should drift)
x0_B = np.array([0.0, 1.0, 0.1, 0.0, 0.0, 0.0])
tB, xB = simulate_openloop(x0_B, T, p)

# x vs t
plot_two_cases("x(t)", tA, xA[:, 0], tB, xB[:, 0], ylabel="x [m]")

# z vs t
plot_two_cases("z(t)", tA, xA[:, 1], tB, xB[:, 1], ylabel="z [m]")

# theta vs t
plot_two_cases("theta(t)", tA, xA[:, 2], tB, xB[:, 2], ylabel="theta [rad]")

# Trajectory: z vs x (two subplots)
fig, ax = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
fig.suptitle("Trajectory (z vs x)")

ax[0].plot(xA[:, 0], xA[:, 1])
ax[0].set_title("Case A")
ax[0].set_xlabel("x [m]")
ax[0].set_ylabel("z [m]")
ax[0].grid(True)

ax[1].plot(xB[:, 0], xB[:, 1])
ax[1].set_title("Case B")
ax[1].set_xlabel("x [m]")
ax[1].grid(True)

plt.tight_layout()
plt.show()

