import numpy as np

def saturation_monitor(M, u_min, u_max):
    """
    M: (N,2) rotor thrusts [u1,u2]
    Returns a dict with saturation stats.
    """
    u1 = M[:, 0]
    u2 = M[:, 1]
    sat = (u1 <= u_min) | (u1 >= u_max) | (u2 <= u_min) | (u2 >= u_max)

    sat_ratio = float(np.mean(sat))
    sat_any = bool(np.any(sat))

    # longest consecutive saturation streak (in samples)
    longest = 0
    current = 0
    for s in sat:
        if s:
            current += 1
            longest = max(longest, current)
        else:
            current = 0

    return {
        "sat_any": sat_any,
        "sat_ratio": sat_ratio,
        "sat_longest_samples": int(longest),
    }

def bounds_monitor(X, limits):
    """
    X: (N,6) state log
    limits: dict of limits like:
      {"x": 5.0, "z_min": 0.0, "z_max": 2.0, "theta": 0.5, "vx": 10.0, "vz": 10.0, "omega": 10.0}
    Returns dict with which limits were violated.
    """
    x = X[:, 0]
    z = X[:, 1]
    th = X[:, 2]
    vx = X[:, 3]
    vz = X[:, 4]
    om = X[:, 5]

    violations = {
        "x": bool(np.any(np.abs(x) > limits.get("x", np.inf))),
        "z": bool(np.any((z < limits.get("z_min", -np.inf)) | (z > limits.get("z_max", np.inf)))),
        "theta": bool(np.any(np.abs(th) > limits.get("theta", np.inf))),
        "vx": bool(np.any(np.abs(vx) > limits.get("vx", np.inf))),
        "vz": bool(np.any(np.abs(vz) > limits.get("vz", np.inf))),
        "omega": bool(np.any(np.abs(om) > limits.get("omega", np.inf))),
    }

    any_violation = any(violations.values())
    return {"any_violation": any_violation, "violations": violations}

def nan_monitor(*arrays):
    """
    Checks if any array contains NaN or Inf.
    Pass in X, U, M, Ilog, etc.
    """
    for a in arrays:
        if a is None:
            continue
        if not np.isfinite(a).all():
            return {"ok": False}
    return {"ok": True}

def windup_monitor(Ilog, sat_info, thresholds=(5.0, 5.0)):
    """
    Ilog: (N,2) integrator states [xI, zI]
    sat_info: output dict from saturation_monitor (or you can pass sat mask later)
    thresholds: (xI_limit, zI_limit) in units of (m*s)

    This is a simple heuristic:
      if integrators get large AND saturation is happening => likely windup risk.
    """
    if Ilog is None:
        return {"enabled": False}

    xI = Ilog[:, 0]
    zI = Ilog[:, 1]
    xI_limit, zI_limit = thresholds

    big_int = bool((np.max(np.abs(xI)) > xI_limit) or (np.max(np.abs(zI)) > zI_limit))
    sat_any = bool(sat_info.get("sat_any", False))

    return {
        "enabled": True,
        "windup_risk": bool(big_int and sat_any),
        "max_abs_xI": float(np.max(np.abs(xI))),
        "max_abs_zI": float(np.max(np.abs(zI))),
    }

def run_monitors(X, U, M, p, Ilog=None, limits=None, windup_thresholds=(2.0, 1.0)):
    if limits is None:
        limits = {"x": 5.0, "z_min": 0.0, "z_max": 2.0, "theta": 0.5}

    sat = saturation_monitor(M, p.u_min, p.u_max)
    bounds = bounds_monitor(X, limits)
    nanok = nan_monitor(X, U, M, Ilog)
    windup = windup_monitor(Ilog, sat, thresholds=windup_thresholds)

    return {"sat": sat, "bounds": bounds, "nan": nanok, "windup": windup}