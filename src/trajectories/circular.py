# trajectories/circular.py

import numpy as np

from trajectories.base import Trajectory

from trajectories.reference import (
    TrajectoryReference
)


class CircularTrajectory(Trajectory):

    def __init__(
        self,
        radius: float = 2.0,
        omega: float = 0.3
    ):

        self.radius = radius
        self.omega = omega

    def reference(
        self,
        t: float
    ) -> TrajectoryReference:

        x = (
            self.radius
            * np.cos(
                self.omega * t
            )
        )

        y = (
            self.radius
            * np.sin(
                self.omega * t
            )
        )

        vx = (
            -self.radius
            * self.omega
            * np.sin(
                self.omega * t
            )
        )

        vy = (
            self.radius
            * self.omega
            * np.cos(
                self.omega * t
            )
        )

        return TrajectoryReference(
            x=x,
            y=y,
            vx=vx,
            vy=vy
        )