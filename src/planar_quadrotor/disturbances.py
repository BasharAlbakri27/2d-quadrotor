def wind_step(t, x, t_on=2.0, Fx=2.0):
    # wind pushes +x after t_on seconds
    return {"Fx": Fx, "Fz": 0.0, "tau": 0.0} if t >= t_on else {"Fx": 0.0, "Fz": 0.0, "tau": 0.0}