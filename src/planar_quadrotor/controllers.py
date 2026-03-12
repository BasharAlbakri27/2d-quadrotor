import numpy as np
from scipy.linalg import solve_continuous_are

def LQR(x_max, u_max):
    """
    x_max: array-like length 6 (max acceptable state deviations)
    u_max: array-like length 2 (max acceptable input deviations)
    """
    # Building Q and R using Bryson's Rule (1/x^2);
    Q = np.diag(1.0 / (np.array(x_max, dtype=float) ** 2))
    R = np.diag(1.0 / (np.array(u_max, dtype=float) ** 2))
    return Q, R

def lqr_K(A, B, Q, R):
    """
    Continuous-time LQR: solves ARE and returns K such that u = -K x
    """
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)  # equivalent to inv(R) @ B.T @ P
    return K