# drone/default_quadrotor.py

import numpy as np
from drone.parameters import DroneParameters


DEFAULT_QUADROTOR = DroneParameters(
    mass=1.5,

    inertia=np.diag([
        0.025,
        0.025,
        0.045
    ])
)