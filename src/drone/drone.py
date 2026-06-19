from src.drone.state import DroneState
from src.drone.parameters import DroneParameters

from src.dynamics.rigid_body import (
    RigidBodyDynamics
)


class Drone:

    def __init__(
        self,
        parameters: DroneParameters,
        dynamics: RigidBodyDynamics,
        state: DroneState
    ):

        self.parameters = parameters

        self.dynamics = dynamics

        self.state = state