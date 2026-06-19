import numpy as np

from src.drone.state import DroneState
from src.drone.parameters import DroneParameters


class AltitudeController:

    def __init__(
        self,
        parameters: DroneParameters,
        kp: float = 6.0,
        kd: float = 4.0
    ):

        self.parameters = parameters

        self.kp = kp
        self.kd = kd

    def compute(
        self,
        state: DroneState,
        z_ref: float
    ) -> float:

        z = state.position[2]

        vz = state.velocity[2]

        phi = state.attitude[0]
        theta = state.attitude[1]

        error = z_ref - z

        error_dot = -vz

        kh = np.cos(phi) * np.cos(theta)

        u = (
            self.kp * error
            + self.kd * error_dot
        )

        thrust = (
            self.parameters.mass
            * (
                self.parameters.gravity
                + u
            )
            / kh
        )

        return thrust