from src.drone.drone import Drone
from src.drone.state import DroneState
from src.drone.parameters import DroneParameters

from src.dynamics.rigid_body import RigidBodyDynamics
import numpy as np


def create_default_quadrotor():

    parameters = DroneParameters(
        mass=1.5,
        inertia=np.diag([
            0.025,
            0.025,
            0.045
        ])
    )

    dynamics = RigidBodyDynamics(parameters=parameters)
    state = DroneState.zero()

    return Drone(
        parameters=parameters,
        dynamics=dynamics,
        state=state
    )