# drone/state.py

from dataclasses import dataclass
import numpy as np

@dataclass
class DroneState:
    """
    Complete UAV state.

    position = [x, y, z]
    velocity = [vx, vy, vz]

    Euler angles (attitude) = [roll (φ), pitch (θ), yaw (ψ)]

    body angular rates = [p, q, r]
    """

    position: np.ndarray
    velocity: np.ndarray

    attitude: np.ndarray
    angular_velocity: np.ndarray

    @classmethod
    def zero(cls):
        return cls(
            position=np.zeros(3),
            velocity=np.zeros(3),
            attitude=np.zeros(3),
            angular_velocity=np.zeros(3)
        )

    @classmethod
    def from_vector(cls, vector: np.ndarray):
        return cls(
            position=vector[0:3],
            velocity=vector[3:6],
            attitude=vector[6:9],
            angular_velocity=vector[9:12]
        )

    def to_vector(self):
        return np.concatenate([
            self.position,
            self.velocity,
            self.attitude,
            self.angular_velocity
        ])