# trajectories/base.py

from abc import ABC
from abc import abstractmethod

from trajectories.reference import (
    TrajectoryReference
)


class Trajectory(ABC):

    @abstractmethod
    def reference(
        self,
        t: float
    ) -> TrajectoryReference:
        pass