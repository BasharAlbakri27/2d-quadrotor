from dataclasses import dataclass

@dataclass(frozen=True)
class Params:
    # Physical parameters
    m: float = 1.5       # kg
    I: float = 0.03      # kg*m^2  (pitch inertia)
    L: float = 0.22      # m       (center-to-rotor moment arm; span = 2L)
    g: float = 9.81      # m/s^2

    # Simulation
    dt: float = 0.01     # s

    # Actuator limits (per-rotor thrust, Newtons)
    u_min: float = 0.0   # N
    u_max: float = 25.0  # N