import numpy as np


def steady_state_error(t, X, x_ref, frac=0.1):
    """
    Average error over the last `frac` portion of the run.
    Returns (ss_x, ss_z).
    """
    N = len(t)
    start = int((1.0 - frac) * N)
    ex = X[start:, 0] - x_ref[0]
    ez = X[start:, 1] - x_ref[1]
    return float(np.mean(ex)), float(np.mean(ez))

def max_abs_errors(X, x_ref):
    """
    Max absolute tracking error for x and z.
    """
    ex = X[:, 0] - x_ref[0]
    ez = X[:, 1] - x_ref[1]
    return float(np.max(np.abs(ex))), float(np.max(np.abs(ez)))

def max_abs_theta(X):
    return float(np.max(np.abs(X[:, 2])))

def settling_time(t, y, y_ref, tol):
    """
    Settling time: earliest time after which |y - y_ref| <= tol forever.
    Returns np.nan if never settles.
    """
    err = np.abs(y - y_ref)
    ok = err <= tol

    # Find earliest index i such that ok[i:] are all True
    for i in range(len(t)):
        if np.all(ok[i:]):  # if all remaining are within tolerance
            return float(t[i])
    return float("nan")

def thrust_metrics(M, u_min, u_max):
    """
    Returns:
      max_u: maximum rotor thrust over both motors
      sat_ratio: fraction of samples where any rotor hits saturation
    """
    u1 = M[:, 0]
    u2 = M[:, 1]
    max_u = float(np.max(np.maximum(u1, u2)))

    saturated = (u1 <= u_min) | (u1 >= u_max) | (u2 <= u_min) | (u2 >= u_max)
    sat_ratio = float(np.mean(saturated))
    return max_u, sat_ratio

def effort_metrics(U):
    """
    Returns max absolute du and dtau.
    """
    du = U[:, 0]
    dtau = U[:, 1]
    return float(np.max(np.abs(du))), float(np.max(np.abs(dtau)))

