# trajectories/line.py

from trajectories.base import Trajectory

from trajectories.reference import (
    TrajectoryReference
)


class LineTrajectory(Trajectory):

    def __init__(
        self,
        velocity: float = 0.2,
        y_ref: float = 1.0
    ):

        self.velocity = velocity
        self.y_ref = y_ref

    def reference(
        self,
        t: float
    ) -> TrajectoryReference:

        x = self.velocity * t

        return TrajectoryReference(
            x=x,
            y=self.y_ref,
            vx=self.velocity,
            vy=0.0
        )