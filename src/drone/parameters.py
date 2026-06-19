# drone/parameters.py

from dataclasses import dataclass
import numpy as np


@dataclass
class DroneParameters:

    mass: float

    inertia: np.ndarray

    gravity: float = 9.81

    @property
    def inertia_inv(self):

        return np.linalg.inv(self.inertia)